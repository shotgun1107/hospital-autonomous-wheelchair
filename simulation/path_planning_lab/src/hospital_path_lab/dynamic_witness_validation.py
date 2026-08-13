"""R2 자동 witness의 독립 ground-truth hard validator.

검색기의 pruning, objective 또는 기존 ``dynamic_corpus`` private validator를 호출하지
않는다. 정확한 open-loop Actor 원과 200 Hz 보수 표본만 사용한다.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
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
from hospital_path_lab.dynamic_witness_events import (
    crossing_targets,
    straight_reference_segments,
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
_PROJECTION_TIE_TOLERANCE_M = 1e-12
_PASS_KINDS = frozenset((WitnessKind.PASS_LEFT, WitnessKind.PASS_RIGHT))
_CROSSING_KINDS = frozenset(
    (WitnessKind.CROSSING_BYPASS_LEFT, WitnessKind.CROSSING_BYPASS_RIGHT)
)


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
    segment_index: int
    segment_length_m: float
    ambiguous: bool


def validate_ground_truth_witness(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    *,
    strict_declarations: bool = False,
    _strict_pass_semantics: bool | None = None,
) -> GroundTruthWitnessValidation:
    """Validate one witness without using category, oracle or prediction data.

    Measurement validation (the default) permits a draft witness with no event
    declarations and returns independently measured events in ``metrics``.  A
    canonical PASS witness must then be checked with ``strict_declarations=True``;
    that mode requires every declaration and compares it to the measurement.
    """

    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(witness, AutomatedWitness):
        raise TypeError("witness must be an AutomatedWitness")

    strict_pass_semantics = (
        strict_declarations
        if _strict_pass_semantics is None
        else _strict_pass_semantics
    )
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
        witness.kind in _PASS_KINDS
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
    geometry = _validate_geometry_and_events(
        world,
        witness,
        samples,
        failures,
        strict_declarations=strict_declarations,
        strict_pass_semantics=strict_pass_semantics,
    )
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


def canonicalize_and_validate_ground_truth_pass(
    world: WitnessWorldSnapshot,
    draft: AutomatedWitness,
) -> tuple[AutomatedWitness | None, GroundTruthWitnessValidation]:
    """Measure and strictly validate one declaration-free PASS in one sweep.

    The expensive 200 Hz geometry and event pass is run once with every strict
    PASS semantic enabled.  When that succeeds, the independently measured
    event times are bound into a canonical witness and the returned validation
    is rebound to that canonical content hash.  This is equivalent to the old
    measurement-then-strict two-pass sequence without repeating geometry.
    """

    if not isinstance(draft, AutomatedWitness):
        raise TypeError("draft must be an AutomatedWitness")
    if draft.kind not in (WitnessKind.PASS_LEFT, WitnessKind.PASS_RIGHT):
        raise ValueError("canonical PASS validation requires a PASS witness")
    if (
        draft.departure_time_s is not None
        or draft.pass_times_by_actor
        or draft.rejoin_started_at_s is not None
        or draft.rejoin_confirmed_at_s is not None
    ):
        raise ValueError("canonical PASS validation requires an undeclared draft")

    measured = validate_ground_truth_witness(
        world,
        draft,
        _strict_pass_semantics=True,
    )
    if not measured.passed:
        return None, measured
    canonical = replace(
        draft,
        departure_time_s=measured.metrics.departure_time_s,
        pass_times_by_actor=measured.metrics.pass_times_by_actor,
        rejoin_started_at_s=measured.metrics.rejoin_started_at_s,
        rejoin_confirmed_at_s=measured.metrics.rejoin_confirmed_at_s,
    )
    strict = GroundTruthWitnessValidation(
        schema_version=measured.schema_version,
        validator_version=measured.validator_version,
        source_projection_hash=measured.source_projection_hash,
        world_content_hash=measured.world_content_hash,
        witness_content_hash=canonical.semantic_content_hash,
        passed=True,
        failures=(),
        metrics=measured.metrics,
    )
    return canonical, strict


def canonicalize_and_validate_ground_truth_crossing_bypass(
    world: WitnessWorldSnapshot,
    draft: AutomatedWitness,
) -> tuple[AutomatedWitness | None, GroundTruthWitnessValidation]:
    """Measure and strictly bind one declaration-free crossing bypass."""

    if not isinstance(draft, AutomatedWitness):
        raise TypeError("draft must be an AutomatedWitness")
    if draft.kind not in _CROSSING_KINDS:
        raise ValueError("canonical crossing validation requires a crossing witness")
    if (
        draft.departure_time_s is not None
        or draft.pass_times_by_actor
        or draft.rejoin_started_at_s is not None
        or draft.rejoin_confirmed_at_s is not None
    ):
        raise ValueError("canonical crossing validation requires an undeclared draft")
    measured = validate_ground_truth_witness(world, draft)
    if not measured.passed:
        return None, measured
    canonical = replace(
        draft,
        departure_time_s=measured.metrics.departure_time_s,
        pass_times_by_actor=measured.metrics.pass_times_by_actor,
        rejoin_started_at_s=measured.metrics.rejoin_started_at_s,
        rejoin_confirmed_at_s=measured.metrics.rejoin_confirmed_at_s,
    )
    strict = validate_ground_truth_witness(
        world,
        canonical,
        strict_declarations=True,
    )
    return (canonical if strict.passed else None), strict


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
    *,
    strict_declarations: bool,
    strict_pass_semantics: bool,
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
    departure_index: int | None = None
    projections: list[tuple[_ValidationSample, _ReferenceProjection]] = []

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
            departure_index is None
            and projection.distance_m > _DEPARTURE_THRESHOLD_M
        ):
            departure_index = len(projections) - 1

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

    departure_time_s = (
        projections[departure_index][0].time_s
        if departure_index is not None
        else None
    )
    pass_times: dict[str, float] = {}
    if witness.kind in _PASS_KINDS:
        pass_times = _measure_ordered_passes(
            world,
            witness,
            projections,
            departure_index=departure_index,
            failures=failures,
        )

    elif witness.kind in _CROSSING_KINDS:
        pass_times = _measure_crossing_bypasses(
            world,
            witness,
            projections,
            departure_index=departure_index,
            failures=failures,
        )

    if witness.kind in _PASS_KINDS:
        if departure_time_s is None:
            failures.append("pass_departure_missing")
        if not set(witness.required_pass_actor_ids).issubset(pass_times):
            failures.append("ordered_overtake_missing")

    latest_pass_s = max(pass_times.values(), default=None)
    rejoin_start_s, rejoin_confirmed_s = _find_sustained_rejoin(
        projections,
        after_s=latest_pass_s,
    )
    if witness.kind in _PASS_KINDS:
        if rejoin_confirmed_s is None:
            failures.append("sustained_rejoin_missing")
        if latest_pass_s is not None and departure_time_s is not None:
            if not departure_time_s < latest_pass_s:
                failures.append("overtake_before_departure")
            if rejoin_start_s is not None and rejoin_start_s < latest_pass_s:
                failures.append("rejoin_before_overtake")
        _validate_pass_side_and_order_retention(
            world,
            witness,
            projections,
            departure_index=departure_index,
            pass_times=pass_times,
            rejoin_confirmed_at_s=rejoin_confirmed_s,
            failures=failures,
            enforce_order_retention=strict_pass_semantics,
        )
        if strict_pass_semantics:
            _validate_strict_pass_reference_segment(
                projections,
                departure_index=departure_index,
                pass_times=pass_times,
                rejoin_started_at_s=rejoin_start_s,
                rejoin_confirmed_at_s=rejoin_confirmed_s,
                failures=failures,
            )
            _validate_multi_actor_pass_scope(
                world,
                witness,
                projections,
                departure_index=departure_index,
                rejoin_started_at_s=rejoin_start_s,
                rejoin_confirmed_at_s=rejoin_confirmed_s,
                failures=failures,
            )
    elif witness.kind in _CROSSING_KINDS:
        if departure_time_s is None:
            failures.append("crossing_departure_missing")
        if not set(witness.required_pass_actor_ids).issubset(pass_times):
            failures.append("active_blocking_bypass_missing")
        if rejoin_confirmed_s is None:
            failures.append("crossing_sustained_rejoin_missing")
        latest_bypass_s = max(pass_times.values(), default=None)
        if latest_bypass_s is not None and departure_time_s is not None:
            if not departure_time_s < latest_bypass_s:
                failures.append("crossing_bypass_before_departure")
            if rejoin_start_s is not None and rejoin_start_s < latest_bypass_s:
                failures.append("crossing_rejoin_before_bypass")
        _validate_crossing_side(
            witness,
            projections,
            departure_index=departure_index,
            bypass_times=pass_times,
            rejoin_confirmed_at_s=rejoin_confirmed_s,
            failures=failures,
        )

    _compare_declared_events(
        witness,
        departure_time_s=departure_time_s,
        pass_times=pass_times,
        rejoin_started_at_s=rejoin_start_s,
        rejoin_confirmed_at_s=rejoin_confirmed_s,
        failures=failures,
        strict_declarations=strict_declarations,
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


def _measure_ordered_passes(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    projections: list[tuple[_ValidationSample, _ReferenceProjection]],
    *,
    departure_index: int | None,
    failures: list[str],
) -> dict[str, float]:
    """Measure target eligibility and ordered passes from ground truth only."""

    if departure_index is None:
        return {}
    profile = world.kinematic_contract.vehicle_profile
    departure_sample, departure_projection = projections[departure_index]
    actor_by_id = {actor.actor_binding_id: actor for actor in world.actors}
    pass_times: dict[str, float] = {}
    for actor_id in witness.required_pass_actor_ids:
        actor = actor_by_id.get(actor_id)
        if actor is None:
            continue
        state = actor.state_at(departure_sample.time_s)
        if state is None:
            failures.append("target_inactive_at_departure")
            continue
        actor_projection = _project_to_reference(
            Pose2D(state.position.x, state.position.y, 0.0),
            world.reference_path,
        )
        longitudinal_extent_m = profile.collision_length_m / 2.0 + state.radius_m
        order_m = actor_projection.progress_m - departure_projection.progress_m
        eligible = True
        if order_m <= longitudinal_extent_m:
            failures.append("target_not_ahead_at_departure")
            eligible = False
        lane_overlap_limit_m = (
            profile.collision_width_m / 2.0
            + state.radius_m
            + profile.minimum_clearance_m
        )
        if abs(actor_projection.signed_offset_m) > lane_overlap_limit_m + 1e-12:
            failures.append("target_not_lane_overlapping_at_departure")
            eligible = False
        tangent_x = cos(actor_projection.tangent_yaw)
        tangent_y = sin(actor_projection.tangent_yaw)
        tangent_speed_mps = (
            state.velocity.x * tangent_x + state.velocity.y * tangent_y
        )
        direction_cosine = (
            tangent_speed_mps / state.velocity.magnitude
            if state.velocity.magnitude > 1e-6
            else -1.0
        )
        if (
            state.velocity.magnitude <= 1e-6
            or tangent_speed_mps <= 0.0
            or direction_cosine < cos(_REJOIN_HEADING_TOLERANCE_RAD) - 1e-12
        ):
            failures.append("target_not_same_direction_at_departure")
            eligible = False
        if not eligible:
            continue

        crossed_without_robot_progress = False
        for sample, robot_projection in projections[departure_index:]:
            state = actor.state_at(sample.time_s)
            if state is None:
                break
            actor_projection = _project_to_reference(
                Pose2D(state.position.x, state.position.y, 0.0),
                world.reference_path,
            )
            order_m = actor_projection.progress_m - robot_projection.progress_m
            if order_m >= -longitudinal_extent_m:
                continue
            robot_progress_m = (
                robot_projection.progress_m - departure_projection.progress_m
            )
            if robot_progress_m < 0.10 - 1e-12:
                crossed_without_robot_progress = True
                continue
            pass_times[actor_id] = sample.time_s
            break
        if actor_id not in pass_times and crossed_without_robot_progress:
            failures.append("ordered_overtake_robot_progress_missing")
    return pass_times


def _measure_crossing_bypasses(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    projections: list[tuple[_ValidationSample, _ReferenceProjection]],
    *,
    departure_index: int | None,
    failures: list[str],
) -> dict[str, float]:
    """Measure progress across a target station while its direct lane is blocked."""

    if departure_index is None:
        return {}
    targets = {
        item.actor_binding_id: item
        for item in crossing_targets(world)
        if item.actor_binding_id in witness.required_pass_actor_ids
    }
    segments = {item.index: item for item in straight_reference_segments(world)}
    result: dict[str, float] = {}
    for actor_id in witness.required_pass_actor_ids:
        target = targets.get(actor_id)
        if target is None:
            failures.append("crossing_target_not_eligible")
            continue
        segment = segments[target.segment_index]
        station = segment.cumulative_start_m + target.crossing_station_progress_m
        previous_progress: float | None = None
        for sample, projection in projections[departure_index:]:
            if projection.segment_index != target.segment_index:
                previous_progress = projection.progress_m
                continue
            if sample.time_s < target.blocking_starts_at_s - _TIME_TOLERANCE_S:
                previous_progress = projection.progress_m
                continue
            if sample.time_s > target.blocking_ends_at_s + _TIME_TOLERANCE_S:
                break
            crossed = (
                previous_progress is not None
                and previous_progress < station - _PROJECTION_TIE_TOLERANCE_M
                and projection.progress_m >= station - _PROJECTION_TIE_TOLERANCE_M
            )
            if crossed and projection.distance_m > _DEPARTURE_THRESHOLD_M:
                result[actor_id] = sample.time_s
                break
            previous_progress = projection.progress_m
    return result


def _validate_crossing_side(
    witness: AutomatedWitness,
    projections: list[tuple[_ValidationSample, _ReferenceProjection]],
    *,
    departure_index: int | None,
    bypass_times: dict[str, float],
    rejoin_confirmed_at_s: float | None,
    failures: list[str],
) -> None:
    if departure_index is None or not bypass_times:
        return
    latest_bypass = max(bypass_times.values())
    left = witness.kind is WitnessKind.CROSSING_BYPASS_LEFT
    for sample, projection in projections[departure_index:]:
        if sample.time_s > latest_bypass + _TIME_TOLERANCE_S:
            break
        correct = (
            projection.signed_offset_m > _DEPARTURE_THRESHOLD_M
            if left
            else projection.signed_offset_m < -_DEPARTURE_THRESHOLD_M
        )
        if not correct:
            failures.append("crossing_bypass_wrong_side")
            break
    if rejoin_confirmed_at_s is None:
        return
    for sample, projection in projections:
        if sample.time_s + _TIME_TOLERANCE_S < latest_bypass:
            continue
        if sample.time_s > rejoin_confirmed_at_s + _TIME_TOLERANCE_S:
            break
        opposite = projection.signed_offset_m < -1e-9 if left else projection.signed_offset_m > 1e-9
        if opposite:
            failures.append("crossing_bypass_crossed_opposite_side")
            break


def _validate_pass_side_and_order_retention(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    projections: list[tuple[_ValidationSample, _ReferenceProjection]],
    *,
    departure_index: int | None,
    pass_times: dict[str, float],
    rejoin_confirmed_at_s: float | None,
    failures: list[str],
    enforce_order_retention: bool,
) -> None:
    if departure_index is None or not pass_times:
        return
    latest_pass_s = max(pass_times.values())
    side_failed = False
    for sample, projection in projections[departure_index:]:
        if sample.time_s > latest_pass_s + _TIME_TOLERANCE_S:
            break
        correct_side = (
            projection.signed_offset_m > _DEPARTURE_THRESHOLD_M
            if witness.kind is WitnessKind.PASS_LEFT
            else projection.signed_offset_m < -_DEPARTURE_THRESHOLD_M
        )
        if not correct_side:
            side_failed = True
            break
    if rejoin_confirmed_at_s is not None:
        for sample, projection in projections:
            if sample.time_s + _TIME_TOLERANCE_S < latest_pass_s:
                continue
            if sample.time_s > rejoin_confirmed_at_s + _TIME_TOLERANCE_S:
                break
            crossed_opposite_side = (
                projection.signed_offset_m < -1e-9
                if witness.kind is WitnessKind.PASS_LEFT
                else projection.signed_offset_m > 1e-9
            )
            if crossed_opposite_side:
                side_failed = True
                break
    if side_failed:
        failures.append("pass_wrong_side")

    if not enforce_order_retention:
        return
    actor_by_id = {actor.actor_binding_id: actor for actor in world.actors}
    profile = world.kinematic_contract.vehicle_profile
    for actor_id, pass_time_s in pass_times.items():
        actor = actor_by_id[actor_id]
        end_time_s = min(
            actor.active_until_s,
            rejoin_confirmed_at_s
            if rejoin_confirmed_at_s is not None
            else projections[-1][0].time_s,
        )
        for sample, robot_projection in projections:
            if sample.time_s + _TIME_TOLERANCE_S < pass_time_s:
                continue
            if sample.time_s > end_time_s + _TIME_TOLERANCE_S:
                break
            state = actor.state_at(sample.time_s)
            if state is None:
                continue
            actor_projection = _project_to_reference(
                Pose2D(state.position.x, state.position.y, 0.0),
                world.reference_path,
            )
            longitudinal_extent_m = profile.collision_length_m / 2.0 + state.radius_m
            if (
                actor_projection.progress_m - robot_projection.progress_m
                >= -longitudinal_extent_m
            ):
                failures.append("post_pass_reversal")
                break


def _validate_strict_pass_reference_segment(
    projections: list[tuple[_ValidationSample, _ReferenceProjection]],
    *,
    departure_index: int | None,
    pass_times: dict[str, float],
    rejoin_started_at_s: float | None,
    rejoin_confirmed_at_s: float | None,
    failures: list[str],
) -> None:
    """Bind strict PASS anchors to one unambiguous non-zero segment."""

    if departure_index is None:
        return
    anchors = [projections[departure_index][1]]
    anchor_times = (
        *pass_times.values(),
        rejoin_started_at_s,
        rejoin_confirmed_at_s,
    )
    for time_s in anchor_times:
        if time_s is None:
            continue
        projection = _projection_at_time(projections, time_s)
        if projection is not None:
            anchors.append(projection)
    if any(
        projection.ambiguous or projection.segment_length_m <= 1e-18
        for projection in anchors
    ):
        failures.append("ambiguous_reference_projection")
        return
    if len({projection.segment_index for projection in anchors}) != 1:
        failures.append("pass_reference_segment_mismatch")


def _validate_multi_actor_pass_scope(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    projections: list[tuple[_ValidationSample, _ReferenceProjection]],
    *,
    departure_index: int | None,
    rejoin_started_at_s: float | None,
    rejoin_confirmed_at_s: float | None,
    failures: list[str],
) -> None:
    """Reject a second same-direction lane blocker in the PASS interval."""

    if (
        departure_index is None
        or rejoin_started_at_s is None
        or rejoin_confirmed_at_s is None
    ):
        return
    required = set(witness.required_pass_actor_ids)
    profile = world.kinematic_contract.vehicle_profile
    departure_projection = projections[departure_index][1]
    rejoin_projections = tuple(
        projection
        for time_s in (rejoin_started_at_s, rejoin_confirmed_at_s)
        if (projection := _projection_at_time(projections, time_s)) is not None
    )
    if len(rejoin_projections) != 2:
        return
    departure_segment = departure_projection.segment_index
    planned_rejoin_progress_m = max(
        projection.progress_m for projection in rejoin_projections
    )
    for actor in world.actors:
        if actor.actor_binding_id in required:
            continue
        for sample, robot_projection in projections[departure_index:]:
            if sample.time_s > rejoin_confirmed_at_s + _TIME_TOLERANCE_S:
                break
            state = actor.state_at(sample.time_s)
            if state is None:
                continue
            actor_projection = _project_to_reference(
                Pose2D(state.position.x, state.position.y, 0.0),
                world.reference_path,
            )
            if actor_projection.ambiguous:
                failures.append("ambiguous_reference_projection")
                return
            if actor_projection.segment_index != departure_segment:
                continue
            lane_overlap_limit_m = (
                profile.collision_width_m / 2.0
                + state.radius_m
                + profile.minimum_clearance_m
            )
            if (
                abs(actor_projection.signed_offset_m)
                > lane_overlap_limit_m + 1e-12
            ):
                continue
            tangent_x = cos(actor_projection.tangent_yaw)
            tangent_y = sin(actor_projection.tangent_yaw)
            tangent_speed_mps = (
                state.velocity.x * tangent_x + state.velocity.y * tangent_y
            )
            direction_cosine = (
                tangent_speed_mps / state.velocity.magnitude
                if state.velocity.magnitude > 1e-6
                else -1.0
            )
            if (
                state.velocity.magnitude <= 1e-6
                or tangent_speed_mps <= 0.0
                or direction_cosine
                < cos(_REJOIN_HEADING_TOLERANCE_RAD) - 1e-12
            ):
                continue
            longitudinal_extent_m = (
                profile.collision_length_m / 2.0 + state.radius_m
            )
            if (
                actor_projection.progress_m - robot_projection.progress_m
                > longitudinal_extent_m
                and actor_projection.progress_m
                >= departure_projection.progress_m - 1e-12
                and actor_projection.progress_m
                <= planned_rejoin_progress_m + longitudinal_extent_m + 1e-12
            ):
                failures.append("multi_actor_pass_out_of_scope")
                return


def _projection_at_time(
    projections: list[tuple[_ValidationSample, _ReferenceProjection]],
    time_s: float,
) -> _ReferenceProjection | None:
    return next(
        (
            projection
            for sample, projection in projections
            if abs(sample.time_s - time_s) <= _TIME_TOLERANCE_S
        ),
        None,
    )


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
    strict_declarations: bool,
) -> None:
    tolerance_s = 0.005 + 1e-12
    if strict_declarations and witness.kind in (_PASS_KINDS | _CROSSING_KINDS):
        declared_actor_ids = {
            actor_id for actor_id, _ in witness.pass_times_by_actor
        }
        if (
            witness.departure_time_s is None
            or witness.rejoin_started_at_s is None
            or witness.rejoin_confirmed_at_s is None
            or declared_actor_ids != set(witness.required_pass_actor_ids)
        ):
            failures.append("strict_event_declaration_missing")
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
    candidates: list[_ReferenceProjection] = []
    cumulative_m = 0.0
    for segment_index, (source, target) in enumerate(
        zip(path, path[1:], strict=False)
    ):
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
        tangent_x = dx / length_m
        tangent_y = dy / length_m
        projection = _ReferenceProjection(
            distance_m=distance_m,
            progress_m=cumulative_m + fraction * length_m,
            signed_offset_m=tangent_x * delta_y - tangent_y * delta_x,
            tangent_yaw=atan2(dy, dx),
            segment_index=segment_index,
            segment_length_m=length_m,
            ambiguous=False,
        )
        if distance_m < best_distance - _PROJECTION_TIE_TOLERANCE_M:
            best_distance = distance_m
            candidates = [projection]
        elif abs(distance_m - best_distance) <= _PROJECTION_TIE_TOLERANCE_M:
            candidates.append(projection)
        cumulative_m += length_m
    if not candidates:
        return _ReferenceProjection(
            distance_m=float("inf"),
            progress_m=0.0,
            signed_offset_m=0.0,
            tangent_yaw=path[0].yaw,
            segment_index=-1,
            segment_length_m=0.0,
            ambiguous=True,
        )
    candidates.sort(key=lambda projection: (projection.segment_index, projection.progress_m))
    selected = candidates[0]
    ambiguous = not _adjacent_collinear_projection_tie(candidates)
    return _ReferenceProjection(
        distance_m=selected.distance_m,
        progress_m=selected.progress_m,
        signed_offset_m=selected.signed_offset_m,
        tangent_yaw=selected.tangent_yaw,
        segment_index=selected.segment_index,
        segment_length_m=selected.segment_length_m,
        ambiguous=ambiguous,
    )


def _adjacent_collinear_projection_tie(
    candidates: list[_ReferenceProjection],
) -> bool:
    if len(candidates) <= 1:
        return True
    if any(
        right.segment_index != left.segment_index + 1
        for left, right in zip(candidates, candidates[1:], strict=False)
    ):
        return False
    tangent_yaw = candidates[0].tangent_yaw
    return all(
        abs(_normalize_angle(candidate.tangent_yaw - tangent_yaw))
        <= _ANGLE_TOLERANCE_RAD
        for candidate in candidates[1:]
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
    "canonicalize_and_validate_ground_truth_pass",
    "validate_ground_truth_witness",
]
