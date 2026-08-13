"""R2 자동 witness의 독립 ground-truth hard validator.

검색기의 pruning, objective 또는 기존 ``dynamic_corpus`` private validator를 호출하지
않는다. 정확한 open-loop Actor 원과 200 Hz 보수 표본만 사용한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, ceil, cos, hypot, isfinite, pi, sin

from hospital_path_lab.collision import (
    CollisionChecker,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.contracts import Pose2D, Twist2D
from hospital_path_lab.dynamic_witness_contracts import (
    WITNESS_VALIDATOR_VERSION,
    AutomatedWitness,
    PassingPolicy,
    WitnessKind,
    WitnessTerminalMode,
    WitnessWorldSnapshot,
)
from hospital_path_lab.map_factory import canonical_content_hash

GROUND_TRUTH_VALIDATION_SCHEMA_VERSION = "ground-truth-witness-validation-v1"
_POSITION_TOLERANCE_M = 1e-9
_ANGLE_TOLERANCE_RAD = 1e-9
_TIME_TOLERANCE_S = 1e-12
_DEPARTURE_THRESHOLD_M = 0.10
_REJOIN_DISTANCE_M = 0.10
_REJOIN_HEADING_TOLERANCE_RAD = 10.0 * pi / 180.0
_REJOIN_DWELL_S = 0.50


@dataclass(frozen=True, slots=True)
class GroundTruthWitnessMetrics:
    sample_count: int
    minimum_static_clearance_m: float
    minimum_forbidden_clearance_m: float
    minimum_actor_clearance_m: float | None
    maximum_reference_deviation_m: float
    maximum_left_offset_m: float
    maximum_right_offset_m: float
    departure_time_s: float | None
    pass_times_by_actor: tuple[tuple[str, float], ...]
    rejoin_started_at_s: float | None
    rejoin_confirmed_at_s: float | None
    terminal_dwell_observed_s: float
    final_goal_distance_m: float
    actual_path_length_m: float
    full_stop_count: int
    absolute_angular_travel_rad: float

    def __post_init__(self) -> None:
        if self.sample_count <= 0 or self.full_stop_count < 0:
            raise ValueError("ground-truth metric counts are invalid")
        finite_values = (
            self.minimum_static_clearance_m,
            self.minimum_forbidden_clearance_m,
            self.maximum_reference_deviation_m,
            self.maximum_left_offset_m,
            self.maximum_right_offset_m,
            self.terminal_dwell_observed_s,
            self.final_goal_distance_m,
            self.actual_path_length_m,
            self.absolute_angular_travel_rad,
        )
        if not all(isfinite(value) for value in finite_values):
            raise ValueError("ground-truth metrics must be finite")
        if self.minimum_actor_clearance_m is not None and not isfinite(
            self.minimum_actor_clearance_m
        ):
            raise ValueError("Actor clearance must be finite when present")


@dataclass(frozen=True, slots=True)
class GroundTruthWitnessValidation:
    schema_version: str
    validator_version: str
    source_projection_hash: str
    world_content_hash: str
    witness_content_hash: str
    passed: bool
    failures: tuple[str, ...]
    metrics: GroundTruthWitnessMetrics

    def __post_init__(self) -> None:
        if self.schema_version != GROUND_TRUTH_VALIDATION_SCHEMA_VERSION:
            raise ValueError("unsupported ground-truth validation schema")
        if self.validator_version != WITNESS_VALIDATOR_VERSION:
            raise ValueError("unsupported ground-truth witness validator")
        if not all(
            (
                self.source_projection_hash,
                self.world_content_hash,
                self.witness_content_hash,
            )
        ):
            raise ValueError("validation provenance must not be empty")
        failures = tuple(dict.fromkeys(self.failures))
        if self.passed == bool(failures):
            raise ValueError("passed must be true exactly when failures is empty")
        object.__setattr__(self, "failures", failures)

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class _ValidationSample:
    time_s: float
    pose: Pose2D
    twist: Twist2D


@dataclass(frozen=True, slots=True)
class _ReferenceProjection:
    distance_m: float
    progress_m: float
    signed_offset_m: float
    tangent_yaw: float


def validate_ground_truth_witness(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
) -> GroundTruthWitnessValidation:
    """Validate one witness without using category, oracle or prediction data."""

    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(witness, AutomatedWitness):
        raise TypeError("witness must be an AutomatedWitness")

    failures: list[str] = []
    if witness.source_projection_hash != world.source_projection_hash:
        failures.append("source_projection_hash_mismatch")
    if witness.world_content_hash != world.content_hash:
        failures.append("world_content_hash_mismatch")
    if witness.vehicle_profile_hash != world.vehicle_profile_hash:
        failures.append("vehicle_profile_hash_mismatch")
    if witness.search_config_hash != world.search_config_hash:
        failures.append("search_config_hash_mismatch")

    actor_by_id = {actor.actor_binding_id: actor for actor in world.actors}
    missing_actor_ids = tuple(
        actor_id
        for actor_id in witness.required_pass_actor_ids
        if actor_id not in actor_by_id
    )
    if missing_actor_ids:
        failures.append("required_pass_actor_missing")
    if (
        witness.kind in (WitnessKind.PASS_LEFT, WitnessKind.PASS_RIGHT)
        and world.maneuver_constraints.passing_policy is PassingPolicy.PROHIBITED
    ):
        failures.append("passing_policy_prohibited")

    first = witness.points[0]
    if abs(first.time_s) > _TIME_TOLERANCE_S:
        failures.append("witness_must_start_at_zero")
    if not _poses_close(first.pose, world.initial_state.pose):
        failures.append("witness_start_pose_mismatch")
    if not _twists_close(first.twist, world.initial_state.twist):
        failures.append("witness_start_twist_mismatch")
    if witness.points[-1].time_s > world.duration_s + _TIME_TOLERANCE_S:
        failures.append("witness_exceeds_world_duration")

    samples = _validate_motion_and_build_samples(world, witness, failures)
    geometry = _validate_geometry_and_events(world, witness, samples, failures)
    terminal_dwell_s = _validate_terminal(world, witness, failures)
    _validate_kind_semantics(world, witness, samples, failures)

    path_length_m = sum(
        hypot(right.pose.x - left.pose.x, right.pose.y - left.pose.y)
        for left, right in zip(witness.points, witness.points[1:], strict=False)
    )
    full_stop_count = sum(
        not _twist_stopped(left.twist) and _twist_stopped(right.twist)
        for left, right in zip(witness.points, witness.points[1:], strict=False)
    )
    angular_travel_rad = sum(
        abs(left.twist.angular) * (right.time_s - left.time_s)
        for left, right in zip(witness.points, witness.points[1:], strict=False)
    )
    metrics = GroundTruthWitnessMetrics(
        sample_count=len(samples),
        minimum_static_clearance_m=geometry["minimum_static_clearance_m"],
        minimum_forbidden_clearance_m=geometry["minimum_forbidden_clearance_m"],
        minimum_actor_clearance_m=geometry["minimum_actor_clearance_m"],
        maximum_reference_deviation_m=geometry["maximum_reference_deviation_m"],
        maximum_left_offset_m=geometry["maximum_left_offset_m"],
        maximum_right_offset_m=geometry["maximum_right_offset_m"],
        departure_time_s=geometry["departure_time_s"],
        pass_times_by_actor=geometry["pass_times_by_actor"],
        rejoin_started_at_s=geometry["rejoin_started_at_s"],
        rejoin_confirmed_at_s=geometry["rejoin_confirmed_at_s"],
        terminal_dwell_observed_s=terminal_dwell_s,
        final_goal_distance_m=hypot(
            witness.points[-1].pose.x - world.goal_pose.x,
            witness.points[-1].pose.y - world.goal_pose.y,
        ),
        actual_path_length_m=path_length_m,
        full_stop_count=full_stop_count,
        absolute_angular_travel_rad=angular_travel_rad,
    )
    unique_failures = tuple(dict.fromkeys(failures))
    return GroundTruthWitnessValidation(
        schema_version=GROUND_TRUTH_VALIDATION_SCHEMA_VERSION,
        validator_version=WITNESS_VALIDATOR_VERSION,
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        witness_content_hash=witness.semantic_content_hash,
        passed=not unique_failures,
        failures=unique_failures,
        metrics=metrics,
    )


def _validate_motion_and_build_samples(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    failures: list[str],
) -> tuple[_ValidationSample, ...]:
    contract = world.kinematic_contract
    profile = contract.vehicle_profile
    samples: list[_ValidationSample] = []
    for left, right in zip(witness.points, witness.points[1:], strict=False):
        duration_s = right.time_s - left.time_s
        if abs(duration_s - contract.control_period_s) > _TIME_TOLERANCE_S:
            failures.append("witness_not_20hz")
        if not (
            -profile.max_reverse_speed_mps - 1e-9
            <= left.twist.linear
            <= profile.max_forward_speed_mps + 1e-9
        ):
            failures.append("linear_speed_exceeded")
        if abs(left.twist.angular) > profile.max_angular_speed_radps + 1e-9:
            failures.append("angular_speed_exceeded")
        if left.twist.linear * right.twist.linear < -1e-12:
            failures.append("reverse_without_stop")
        if duration_s > 0.0:
            linear_rate = abs(right.twist.linear - left.twist.linear) / duration_s
            increasing = abs(right.twist.linear) > abs(left.twist.linear) + 1e-12
            linear_limit = (
                profile.max_acceleration_mps2
                if increasing
                else profile.max_deceleration_mps2
            )
            if linear_rate > linear_limit + 1e-9:
                failures.append("linear_acceleration_exceeded")
            angular_rate = abs(right.twist.angular - left.twist.angular) / duration_s
            if angular_rate > contract.maximum_angular_acceleration_radps2 + 1e-9:
                failures.append("angular_acceleration_exceeded")
            expected_pose = _integrate_pose(left.pose, left.twist, duration_s)
            if not _poses_close(expected_pose, right.pose):
                failures.append("kinematic_pose_mismatch")
            subdivisions = max(1, ceil(duration_s / contract.evaluator_period_s))
            evaluation_times_s = {
                left.time_s
                + min(
                    duration_s,
                    subdivision * contract.evaluator_period_s,
                )
                for subdivision in range(subdivisions)
            }
            for actor in world.actors:
                for event_time_s in (actor.active_from_s, actor.active_until_s):
                    if left.time_s <= event_time_s < right.time_s:
                        evaluation_times_s.add(event_time_s)
            for sample_time_s in sorted(evaluation_times_s):
                offset_s = sample_time_s - left.time_s
                samples.append(
                    _ValidationSample(
                        time_s=sample_time_s,
                        pose=_integrate_pose(left.pose, left.twist, offset_s),
                        twist=left.twist,
                    )
                )
    final_twist = witness.points[-1].twist
    if not (
        -profile.max_reverse_speed_mps - 1e-9
        <= final_twist.linear
        <= profile.max_forward_speed_mps + 1e-9
    ):
        failures.append("linear_speed_exceeded")
    if abs(final_twist.angular) > profile.max_angular_speed_radps + 1e-9:
        failures.append("angular_speed_exceeded")
    samples.append(
        _ValidationSample(
            time_s=witness.points[-1].time_s,
            pose=witness.points[-1].pose,
            twist=witness.points[-1].twist,
        )
    )
    return tuple(samples)


def _validate_geometry_and_events(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    samples: tuple[_ValidationSample, ...],
    failures: list[str],
) -> dict[str, object]:
    profile = world.kinematic_contract.vehicle_profile
    grid = world.grid.to_grid_map()
    static_checker = CollisionChecker(grid, profile)
    policy_forbidden = set(world.grid.forbidden_cells)
    if world.maneuver_constraints.allowed_cells:
        allowed = set(world.maneuver_constraints.allowed_cells)
        policy_forbidden.update(
            (x, y)
            for y in range(world.grid.height)
            for x in range(world.grid.width)
            if (x, y) not in allowed
        )
    forbidden_checker = CollisionChecker(
        grid,
        profile,
        forbidden_cells=frozenset(policy_forbidden),
    )
    half_diagonal_m = hypot(
        profile.collision_length_m / 2.0,
        profile.collision_width_m / 2.0,
    )
    minimum_static = 1.0
    minimum_forbidden = 1.0
    minimum_actor: float | None = None
    max_deviation = 0.0
    max_left = 0.0
    max_right = 0.0
    departure_time_s: float | None = None
    actor_initially_ahead: set[str] = set()
    pass_times: dict[str, float] = {}
    projections: list[tuple[_ValidationSample, _ReferenceProjection]] = []

    required_actor_ids = set(witness.required_pass_actor_ids)
    for sample_index, sample in enumerate(samples):
        next_time_s = (
            samples[sample_index + 1].time_s
            if sample_index + 1 < len(samples)
            else sample.time_s
        )
        local_step_s = min(
            world.kinematic_contract.evaluator_period_s,
            max(0.0, next_time_s - sample.time_s),
        )
        robot_speed_bound = abs(sample.twist.linear) + (
            abs(sample.twist.angular) * half_diagonal_m
        )
        robot_half_step_m = robot_speed_bound * local_step_s / 2.0
        static_clearance = static_checker.clearance(sample.pose) - robot_half_step_m
        forbidden_clearance = (
            forbidden_checker.forbidden_clearance(sample.pose) - robot_half_step_m
        )
        minimum_static = min(minimum_static, static_clearance)
        minimum_forbidden = min(minimum_forbidden, forbidden_clearance)
        if static_clearance < profile.minimum_clearance_m - 1e-9:
            failures.append("static_clearance_violation")
        if forbidden_clearance < -1e-9:
            failures.append("forbidden_region_entry")

        projection = _project_to_reference(sample.pose, world.reference_path)
        projections.append((sample, projection))
        max_deviation = max(max_deviation, projection.distance_m)
        max_left = max(max_left, projection.signed_offset_m)
        max_right = max(max_right, -projection.signed_offset_m)
        if (
            departure_time_s is None
            and projection.distance_m > _DEPARTURE_THRESHOLD_M
        ):
            departure_time_s = sample.time_s

        for actor in world.actor_states_at(min(sample.time_s, world.duration_s)):
            actor_speed = actor.velocity.magnitude
            clearance = oriented_footprint_circle_surface_distance(
                sample.pose,
                circle_center=(actor.position.x, actor.position.y),
                circle_radius_m=actor.radius_m,
                profile=profile,
            ) - (robot_speed_bound + actor_speed) * local_step_s / 2.0
            minimum_actor = clearance if minimum_actor is None else min(
                minimum_actor,
                clearance,
            )
            if clearance < profile.minimum_clearance_m - 1e-9:
                failures.append("actor_clearance_violation")
            if actor.actor_id not in required_actor_ids:
                continue
            actor_projection = _project_to_reference(
                Pose2D(actor.position.x, actor.position.y, 0.0),
                world.reference_path,
            )
            order_m = actor_projection.progress_m - projection.progress_m
            longitudinal_extent_m = (
                profile.collision_length_m / 2.0 + actor.radius_m
            )
            if order_m > longitudinal_extent_m:
                actor_initially_ahead.add(actor.actor_id)
            elif (
                actor.actor_id in actor_initially_ahead
                and actor.actor_id not in pass_times
                and order_m < -longitudinal_extent_m
                and departure_time_s is not None
            ):
                pass_times[actor.actor_id] = sample.time_s

    if witness.kind in (WitnessKind.PASS_LEFT, WitnessKind.PASS_RIGHT):
        if departure_time_s is None:
            failures.append("pass_departure_missing")
        if witness.kind is WitnessKind.PASS_LEFT and max_left <= _DEPARTURE_THRESHOLD_M:
            failures.append("pass_left_direction_missing")
        if witness.kind is WitnessKind.PASS_RIGHT and max_right <= _DEPARTURE_THRESHOLD_M:
            failures.append("pass_right_direction_missing")
        if not required_actor_ids.issubset(pass_times):
            failures.append("ordered_overtake_missing")

    latest_pass_s = max(pass_times.values(), default=None)
    rejoin_start_s, rejoin_confirmed_s = _find_sustained_rejoin(
        projections,
        after_s=latest_pass_s,
    )
    if witness.kind in (WitnessKind.PASS_LEFT, WitnessKind.PASS_RIGHT):
        if rejoin_confirmed_s is None:
            failures.append("sustained_rejoin_missing")
        if latest_pass_s is not None and departure_time_s is not None:
            if not departure_time_s < latest_pass_s:
                failures.append("overtake_before_departure")
            if rejoin_start_s is not None and rejoin_start_s < latest_pass_s:
                failures.append("rejoin_before_overtake")

    _compare_declared_events(
        witness,
        departure_time_s=departure_time_s,
        pass_times=pass_times,
        rejoin_started_at_s=rejoin_start_s,
        rejoin_confirmed_at_s=rejoin_confirmed_s,
        failures=failures,
    )
    return {
        "minimum_static_clearance_m": minimum_static,
        "minimum_forbidden_clearance_m": minimum_forbidden,
        "minimum_actor_clearance_m": minimum_actor,
        "maximum_reference_deviation_m": max_deviation,
        "maximum_left_offset_m": max_left,
        "maximum_right_offset_m": max_right,
        "departure_time_s": departure_time_s,
        "pass_times_by_actor": tuple(sorted(pass_times.items())),
        "rejoin_started_at_s": rejoin_start_s,
        "rejoin_confirmed_at_s": rejoin_confirmed_s,
    }


def _validate_terminal(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    failures: list[str],
) -> float:
    final = witness.points[-1]
    dwell_start_s = final.time_s - witness.terminal_dwell_s
    dwell_points = tuple(
        point
        for point in witness.points
        if point.time_s >= dwell_start_s - _TIME_TOLERANCE_S
    )
    terminal_dwell_s = 0.0
    if (
        dwell_points
        and dwell_points[0].time_s <= dwell_start_s + _TIME_TOLERANCE_S
        and all(
            _twist_stopped(point.twist) and _poses_close(point.pose, final.pose)
            for point in dwell_points
        )
    ):
        terminal_dwell_s = final.time_s - dwell_points[0].time_s
    else:
        failures.append("terminal_dwell_missing")

    final_projection = _project_to_reference(final.pose, world.reference_path)
    final_heading_error = abs(_normalize_angle(final.pose.yaw - final_projection.tangent_yaw))
    if witness.terminal_mode is WitnessTerminalMode.GOAL_DWELL:
        if hypot(final.pose.x - world.goal_pose.x, final.pose.y - world.goal_pose.y) > 0.05:
            failures.append("goal_position_not_reached")
        if abs(_normalize_angle(final.pose.yaw - world.goal_pose.yaw)) > (
            _REJOIN_HEADING_TOLERANCE_RAD
        ):
            failures.append("goal_heading_not_reached")
    elif witness.terminal_mode is WitnessTerminalMode.REJOIN_DWELL:
        if final_projection.distance_m > _REJOIN_DISTANCE_M:
            failures.append("terminal_rejoin_distance_exceeded")
        if final_heading_error > _REJOIN_HEADING_TOLERANCE_RAD:
            failures.append("terminal_rejoin_heading_exceeded")
    elif witness.terminal_mode is WitnessTerminalMode.SAFE_HOLD:
        if abs(final.time_s - world.duration_s) > _TIME_TOLERANCE_S:
            failures.append("safe_hold_does_not_cover_world")
        if any(
            not _twist_stopped(point.twist)
            or not _poses_close(point.pose, world.initial_state.pose)
            for point in witness.points
        ):
            failures.append("safe_hold_moved")
    return terminal_dwell_s


def _validate_kind_semantics(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    samples: tuple[_ValidationSample, ...],
    failures: list[str],
) -> None:
    if witness.kind is not WitnessKind.WAIT_AND_FOLLOW:
        return
    projections = tuple(
        _project_to_reference(sample.pose, world.reference_path) for sample in samples
    )
    if max((projection.distance_m for projection in projections), default=0.0) > (
        _REJOIN_DISTANCE_M + 1e-9
    ):
        failures.append("wait_follow_left_reference_corridor")
    initial_progress_m = projections[0].progress_m
    final_progress_m = projections[-1].progress_m
    if final_progress_m <= initial_progress_m + 0.10:
        failures.append("wait_follow_has_no_forward_progress")
    stopped_duration_s = 0.0
    maximum_stopped_duration_s = 0.0
    qualified_wait_end_indices: list[int] = []
    terminal_start_s = witness.points[-1].time_s - witness.terminal_dwell_s
    for index, (left, right) in enumerate(
        zip(samples, samples[1:], strict=False)
    ):
        if left.time_s >= terminal_start_s - _TIME_TOLERANCE_S:
            break
        duration_s = right.time_s - left.time_s
        if _twist_stopped(left.twist):
            stopped_duration_s += duration_s
            maximum_stopped_duration_s = max(
                maximum_stopped_duration_s,
                stopped_duration_s,
            )
        else:
            if stopped_duration_s >= _REJOIN_DWELL_S - 1e-12:
                qualified_wait_end_indices.append(index)
            stopped_duration_s = 0.0
    if maximum_stopped_duration_s < _REJOIN_DWELL_S - 1e-12:
        failures.append("wait_follow_has_no_actual_wait")
    elif not any(
        _progressed_after_wait(
            samples,
            projections,
            wait_end_index=wait_end_index,
            terminal_start_s=terminal_start_s,
        )
        for wait_end_index in qualified_wait_end_indices
    ):
        failures.append("wait_follow_did_not_progress_after_wait")
    if witness.terminal_mode not in (
        WitnessTerminalMode.GOAL_DWELL,
        WitnessTerminalMode.REJOIN_DWELL,
    ):
        failures.append("wait_follow_terminal_mode_invalid")


def _progressed_after_wait(
    samples: tuple[_ValidationSample, ...],
    projections: tuple[_ReferenceProjection, ...],
    *,
    wait_end_index: int,
    terminal_start_s: float,
) -> bool:
    progress_at_wait_end_m = projections[wait_end_index].progress_m
    return any(
        sample.time_s < terminal_start_s - _TIME_TOLERANCE_S
        and sample.twist.linear > 1e-12
        and projection.progress_m >= progress_at_wait_end_m + 0.10 - 1e-12
        for sample, projection in zip(
            samples[wait_end_index:],
            projections[wait_end_index:],
            strict=True,
        )
    )


def _find_sustained_rejoin(
    projections: list[tuple[_ValidationSample, _ReferenceProjection]],
    *,
    after_s: float | None,
) -> tuple[float | None, float | None]:
    if after_s is None:
        return None, None
    active_start_s: float | None = None
    for sample, projection in projections:
        if sample.time_s + _TIME_TOLERANCE_S < after_s:
            continue
        heading_error = abs(_normalize_angle(sample.pose.yaw - projection.tangent_yaw))
        aligned = (
            projection.distance_m <= _REJOIN_DISTANCE_M + 1e-12
            and heading_error <= _REJOIN_HEADING_TOLERANCE_RAD + 1e-12
        )
        if not aligned:
            active_start_s = None
            continue
        if active_start_s is None:
            active_start_s = sample.time_s
        if sample.time_s - active_start_s >= _REJOIN_DWELL_S - 1e-12:
            return active_start_s, sample.time_s
    return active_start_s, None


def _compare_declared_events(
    witness: AutomatedWitness,
    *,
    departure_time_s: float | None,
    pass_times: dict[str, float],
    rejoin_started_at_s: float | None,
    rejoin_confirmed_at_s: float | None,
    failures: list[str],
) -> None:
    tolerance_s = 0.005 + 1e-12
    pairs = (
        (witness.departure_time_s, departure_time_s, "declared_departure_mismatch"),
        (
            witness.rejoin_started_at_s,
            rejoin_started_at_s,
            "declared_rejoin_start_mismatch",
        ),
        (
            witness.rejoin_confirmed_at_s,
            rejoin_confirmed_at_s,
            "declared_rejoin_confirmation_mismatch",
        ),
    )
    for declared, measured, code in pairs:
        if declared is not None and (
            measured is None or abs(declared - measured) > tolerance_s
        ):
            failures.append(code)
    for actor_id, declared_s in witness.pass_times_by_actor:
        measured_s = pass_times.get(actor_id)
        if measured_s is None or abs(declared_s - measured_s) > tolerance_s:
            failures.append("declared_pass_time_mismatch")


def _project_to_reference(
    pose: Pose2D,
    path: tuple[Pose2D, ...],
) -> _ReferenceProjection:
    best_distance = float("inf")
    best_progress = 0.0
    best_signed = 0.0
    best_tangent = path[0].yaw
    cumulative_m = 0.0
    for source, target in zip(path, path[1:], strict=False):
        dx = target.x - source.x
        dy = target.y - source.y
        length_m = hypot(dx, dy)
        if length_m <= 1e-18:
            continue
        fraction = min(
            1.0,
            max(
                0.0,
                ((pose.x - source.x) * dx + (pose.y - source.y) * dy)
                / (length_m * length_m),
            ),
        )
        projected_x = source.x + fraction * dx
        projected_y = source.y + fraction * dy
        delta_x = pose.x - projected_x
        delta_y = pose.y - projected_y
        distance_m = hypot(delta_x, delta_y)
        if distance_m < best_distance:
            tangent_x = dx / length_m
            tangent_y = dy / length_m
            best_distance = distance_m
            best_progress = cumulative_m + fraction * length_m
            best_signed = tangent_x * delta_y - tangent_y * delta_x
            best_tangent = atan2(dy, dx)
        cumulative_m += length_m
    return _ReferenceProjection(
        distance_m=best_distance,
        progress_m=best_progress,
        signed_offset_m=best_signed,
        tangent_yaw=best_tangent,
    )


def _integrate_pose(pose: Pose2D, twist: Twist2D, dt_s: float) -> Pose2D:
    return Pose2D(
        pose.x + twist.linear * cos(pose.yaw) * dt_s,
        pose.y + twist.linear * sin(pose.yaw) * dt_s,
        _normalize_angle(pose.yaw + twist.angular * dt_s),
    )


def _poses_close(left: Pose2D, right: Pose2D) -> bool:
    return (
        hypot(left.x - right.x, left.y - right.y) <= _POSITION_TOLERANCE_M
        and abs(_normalize_angle(left.yaw - right.yaw)) <= _ANGLE_TOLERANCE_RAD
    )


def _twists_close(left: Twist2D, right: Twist2D) -> bool:
    return (
        abs(left.linear - right.linear) <= 1e-12
        and abs(left.angular - right.angular) <= 1e-12
    )


def _twist_stopped(twist: Twist2D) -> bool:
    return abs(twist.linear) <= 1e-12 and abs(twist.angular) <= 1e-12


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


__all__ = [
    "GROUND_TRUTH_VALIDATION_SCHEMA_VERSION",
    "GroundTruthWitnessMetrics",
    "GroundTruthWitnessValidation",
    "validate_ground_truth_witness",
]
