"""동적 controller와 독립된 200 Hz ground-truth 평가기.

이 모듈은 controller가 사용한 prediction tube가 아니라 evaluator 전용 Actor 실제
궤적을 사용한다. 결과는 Python ``simulation_only`` 연구 증거이며 실제 사람 탑승
안전성이나 제품 알고리즘 채택 근거가 아니다.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from math import cos, hypot, isfinite, pi, sin, sqrt
from statistics import fmean

from hospital_path_lab.collision import (
    CollisionChecker,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.contracts import GridSnapshot, Pose2D, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    ActorState,
    DynamicHoldReason,
    DynamicMotionState,
)
from hospital_path_lab.simulation import (
    DynamicControllerPipelineResult,
    DynamicControllerPipelineStep,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile

EVALUATOR_FREQUENCY_HZ = 200.0
EVALUATOR_PERIOD_S = 1.0 / EVALUATOR_FREQUENCY_HZ
_GEOMETRY_TOLERANCE_M = 1e-9
_REJOIN_TOLERANCE_M = 0.10
_DEADLOCK_WINDOW_S = 3.0
_DEADLOCK_PROGRESS_M = 0.02

ActorStateProvider = Callable[[float], tuple[ActorState, ...]]
GridSnapshotProvider = Callable[[int], GridSnapshot]


@dataclass(frozen=True, slots=True)
class DynamicHardSafetyVerdict:
    passed: bool
    first_failure_time_s: float | None
    collision_count: int
    clearance_violation_count: int
    forbidden_entry_count: int
    stale_or_invalid_propulsion_count: int
    unauthorized_resume_count: int
    late_command_applied_count: int
    nonfinite_or_provenance_failure_count: int
    failures: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class DynamicEvaluationMetrics:
    completion_time_s: float | None
    safety_hold_duration_s: float
    hold_duration_by_reason_s: tuple[tuple[str, float], ...]
    controller_stop_request_count: int
    gate_override_count: int
    no_safe_candidate_count: int
    path_length_m: float
    signed_path_length_delta_m: float
    positive_detour_length_m: float
    maximum_reference_deviation_m: float
    rms_reference_deviation_m: float
    longitudinal_jerk_rms_mps3: float
    angular_acceleration_rms_radps2: float
    angular_jerk_rms_radps3: float
    peak_angular_velocity_radps: float
    angular_direction_change_count: int
    minimum_surface_clearance_m: float | None
    minimum_ttc_s: float | None
    rejoin_observed: bool
    overtaking_observed: bool
    planner_deadlock: bool


@dataclass(frozen=True, slots=True)
class DynamicEvaluationResult:
    controller_name: str
    episode_id: str
    expectation_category: str
    simulation_only: bool
    evaluator_frequency_hz: float
    hard_safety: DynamicHardSafetyVerdict
    metrics: DynamicEvaluationMetrics
    functional_qualified: bool
    functional_failures: tuple[str, ...]


def evaluate_dynamic_pipeline(
    pipeline: DynamicControllerPipelineResult,
    *,
    episode_id: str,
    expectation_category: str,
    progressable: bool,
    reference_path: tuple[Pose2D, ...],
    goal_pose: Pose2D,
    actor_states_at: ActorStateProvider,
    grid_snapshot_at: GridSnapshotProvider,
    blocking_cleared_at_s: float | None = None,
    completion_deadline_after_clear_s: float = 30.0,
    profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
) -> DynamicEvaluationResult:
    """Stage 4 pipeline trace를 독립 ground truth로 평가한다."""

    if not episode_id or not expectation_category:
        raise ValueError("evaluation identity fields must not be empty")
    if len(reference_path) < 2:
        raise ValueError("reference_path must contain at least two poses")
    if completion_deadline_after_clear_s <= 0.0:
        raise ValueError("completion deadline must be positive")
    if not profile.simulation_only or not pipeline.simulation_only:
        raise ValueError("dynamic evaluator is simulation-only")

    collision_count = 0
    clearance_violation_count = 0
    forbidden_entry_count = 0
    stale_or_invalid_propulsion_count = 0
    unauthorized_resume_count = 0
    late_command_applied_count = 0
    provenance_failure_count = 0
    minimum_clearance: float | None = None
    minimum_ttc: float | None = None
    evaluator_poses: list[Pose2D] = []
    evaluator_times: list[float] = []
    overtaking_observed = False
    actor_order: dict[str, float] = {}
    previous_motion_state: DynamicMotionState | None = None
    first_failure_time_s: float | None = None

    half_diagonal = hypot(
        profile.collision_length_m / 2.0,
        profile.collision_width_m / 2.0,
    )
    subdivisions = round(DYNAMIC_CONTROL_PERIOD_S / EVALUATOR_PERIOD_S)
    if subdivisions * EVALUATOR_PERIOD_S != DYNAMIC_CONTROL_PERIOD_S:
        raise AssertionError("200 Hz evaluator must divide the 20 Hz control period")

    for expected_tick, step in enumerate(pipeline.steps):
        if (
            step.tick_id != expected_tick
            or step.controller_result.source_tick_id != step.tick_id
            or step.safety_decision.tick_id != step.tick_id
            or step.safety_decision.source_tick_id != step.tick_id
            or not step.controller_result.input_content_hash
        ):
            provenance_failure_count += 1
            if first_failure_time_s is None:
                first_failure_time_s = step.simulation_time_s
        if not _pipeline_step_is_finite(step):
            provenance_failure_count += 1
            if first_failure_time_s is None:
                first_failure_time_s = step.simulation_time_s

        context_grid = grid_snapshot_at(step.tick_id)
        result = step.controller_result
        metadata = context_grid.metadata
        if (
            result.map_id != metadata.map_id
            or result.map_revision != metadata.map_revision
            or result.mission_revision != metadata.mission_revision
            or result.observation_revision != metadata.observation_revision
            or result.grid_content_hash != metadata.content_hash
        ):
            provenance_failure_count += 1
            if first_failure_time_s is None:
                first_failure_time_s = step.simulation_time_s

        invalid_reason = step.safety_decision.primary_hold_reason in {
            DynamicHoldReason.STALE,
            DynamicHoldReason.INVALID_SOURCE,
            DynamicHoldReason.DEADLINE,
        }
        if invalid_reason and not _is_braking_only(
            step.robot_state_before.twist,
            step.safety_decision.command,
        ):
            stale_or_invalid_propulsion_count += 1
            if first_failure_time_s is None:
                first_failure_time_s = step.simulation_time_s
        if (
            step.safety_decision.primary_hold_reason is DynamicHoldReason.DEADLINE
            and not _is_braking_only(
                step.robot_state_before.twist,
                step.safety_decision.command,
            )
        ):
            late_command_applied_count += 1
            if first_failure_time_s is None:
                first_failure_time_s = step.simulation_time_s
        if (
            previous_motion_state is DynamicMotionState.HOLDING
            and step.safety_decision.motion_state is DynamicMotionState.MOVING
            and not step.safety_decision.resume_allowed
        ):
            unauthorized_resume_count += 1
            if first_failure_time_s is None:
                first_failure_time_s = step.simulation_time_s
        previous_motion_state = step.safety_decision.motion_state

        checker = CollisionChecker(
            context_grid.grid,
            profile,
            forbidden_cells=context_grid.forbidden_cells,
        )
        start_index = 0 if expected_tick == 0 else 1
        for substep in range(start_index, subdivisions + 1):
            offset_s = substep * EVALUATOR_PERIOD_S
            evaluation_time_s = step.simulation_time_s + offset_s
            pose = _integrate(
                step.robot_state_before.pose,
                step.robot_state_before.twist,
                offset_s,
            )
            actors = actor_states_at(evaluation_time_s)
            robot_speed_bound = abs(step.robot_state_before.twist.linear) + (
                abs(step.robot_state_before.twist.angular) * half_diagonal
            )
            static_clearance = checker.clearance(pose) - (
                robot_speed_bound * EVALUATOR_PERIOD_S / 2.0
            )
            forbidden_clearance = checker.forbidden_clearance(pose) - (
                robot_speed_bound * EVALUATOR_PERIOD_S / 2.0
            )
            actor_clearances: list[float] = []
            for actor in actors:
                raw_clearance = oriented_footprint_circle_surface_distance(
                    pose,
                    circle_center=(actor.position.x, actor.position.y),
                    circle_radius_m=actor.radius_m,
                    profile=profile,
                )
                conservative_clearance = raw_clearance - (
                    (robot_speed_bound + actor.velocity.magnitude)
                    * EVALUATOR_PERIOD_S
                    / 2.0
                )
                actor_clearances.append(conservative_clearance)
                ttc = _time_to_collision(
                    pose,
                    step.robot_state_before.twist,
                    actor,
                    max(0.0, raw_clearance),
                )
                if ttc is not None:
                    minimum_ttc = ttc if minimum_ttc is None else min(minimum_ttc, ttc)

                robot_arc, _ = _project_to_path(pose, reference_path)
                actor_arc, actor_deviation = _project_point_to_path(
                    actor.position.x,
                    actor.position.y,
                    reference_path,
                )
                relevant_lateral_distance = (
                    actor.radius_m
                    + profile.collision_width_m / 2.0
                    + profile.minimum_clearance_m
                )
                if actor_deviation <= relevant_lateral_distance:
                    order = actor_arc - robot_arc
                    previous_order = actor_order.get(actor.actor_id)
                    if previous_order is not None and previous_order > 0.0 >= order:
                        overtaking_observed = True
                    actor_order[actor.actor_id] = order

            sample_clearance = min(
                (static_clearance, *actor_clearances),
            )
            minimum_clearance = (
                sample_clearance
                if minimum_clearance is None
                else min(minimum_clearance, sample_clearance)
            )
            collision_count += int(sample_clearance <= _GEOMETRY_TOLERANCE_M)
            clearance_violation_count += int(
                sample_clearance
                < profile.minimum_clearance_m - _GEOMETRY_TOLERANCE_M
            )
            forbidden_entry_count += int(
                forbidden_clearance <= _GEOMETRY_TOLERANCE_M
            )
            if (
                first_failure_time_s is None
                and (
                    sample_clearance
                    < profile.minimum_clearance_m - _GEOMETRY_TOLERANCE_M
                    or forbidden_clearance <= _GEOMETRY_TOLERANCE_M
                )
            ):
                first_failure_time_s = evaluation_time_s
            evaluator_poses.append(pose)
            evaluator_times.append(evaluation_time_s)

        expected_after = _integrate_simulation_tick(
            step.robot_state_before.pose,
            step.robot_state_before.twist,
            DYNAMIC_CONTROL_PERIOD_S,
        )
        if not _poses_close(expected_after, step.robot_state_after.pose):
            provenance_failure_count += 1
            if first_failure_time_s is None:
                first_failure_time_s = step.simulation_time_s

    failures: list[str] = []
    for count, name in (
        (collision_count, "ground_truth_collision"),
        (clearance_violation_count, "ground_truth_clearance_below_minimum"),
        (forbidden_entry_count, "forbidden_zone_entry"),
        (stale_or_invalid_propulsion_count, "stale_or_invalid_propulsion"),
        (unauthorized_resume_count, "unauthorized_resume"),
        (late_command_applied_count, "late_command_applied"),
        (provenance_failure_count, "nonfinite_or_provenance_mismatch"),
    ):
        if count:
            failures.append(name)

    hard_safety = DynamicHardSafetyVerdict(
        passed=not failures,
        first_failure_time_s=first_failure_time_s,
        collision_count=collision_count,
        clearance_violation_count=clearance_violation_count,
        forbidden_entry_count=forbidden_entry_count,
        stale_or_invalid_propulsion_count=stale_or_invalid_propulsion_count,
        unauthorized_resume_count=unauthorized_resume_count,
        late_command_applied_count=late_command_applied_count,
        nonfinite_or_provenance_failure_count=provenance_failure_count,
        failures=tuple(failures),
    )

    poses = [step.robot_state_before.pose for step in pipeline.steps]
    if pipeline.steps:
        poses.append(pipeline.steps[-1].robot_state_after.pose)
    twists = [step.robot_state_before.twist for step in pipeline.steps]
    if pipeline.steps:
        twists.append(pipeline.steps[-1].robot_state_after.twist)
    deviations = tuple(
        _project_to_path(pose, reference_path)[1] for pose in evaluator_poses
    )
    departed = False
    rejoin_observed = False
    for deviation in deviations:
        if deviation > _REJOIN_TOLERANCE_M:
            departed = True
        elif departed and deviation <= _REJOIN_TOLERANCE_M:
            rejoin_observed = True
            break

    hold_durations: dict[str, float] = {}
    hold_duration_s = 0.0
    for step in pipeline.steps:
        if step.safety_decision.motion_state in {
            DynamicMotionState.BRAKING,
            DynamicMotionState.HOLDING,
        }:
            hold_duration_s += DYNAMIC_CONTROL_PERIOD_S
            reason = step.safety_decision.primary_hold_reason
            key = reason.value if reason is not None else "unspecified"
            hold_durations[key] = hold_durations.get(key, 0.0) + DYNAMIC_CONTROL_PERIOD_S

    path_length_m = sum(
        hypot(target.x - source.x, target.y - source.y)
        for source, target in zip(poses, poses[1:], strict=False)
    )
    reference_length_m = sum(
        hypot(target.x - source.x, target.y - source.y)
        for source, target in zip(reference_path, reference_path[1:], strict=False)
    )
    signed_delta = path_length_m - reference_length_m
    completion_time_s = (
        len(pipeline.steps) * DYNAMIC_CONTROL_PERIOD_S if pipeline.completed else None
    )
    planner_deadlock = _planner_deadlock(
        pipeline,
        reference_path,
        progressable=progressable,
    )
    functional_failures: list[str] = []
    if progressable and not pipeline.completed:
        functional_failures.append("progressable_episode_not_completed")
    if (
        progressable
        and blocking_cleared_at_s is not None
        and completion_time_s is not None
        and completion_time_s
        > blocking_cleared_at_s + completion_deadline_after_clear_s
    ):
        functional_failures.append("completion_after_30s_post_clear_deadline")
    if planner_deadlock:
        functional_failures.append("planner_deadlock")
    if pipeline.completed and hypot(
        pipeline.final_state.pose.x - goal_pose.x,
        pipeline.final_state.pose.y - goal_pose.y,
    ) > 0.05 + _GEOMETRY_TOLERANCE_M:
        functional_failures.append("completed_outside_goal_tolerance")

    metrics = DynamicEvaluationMetrics(
        completion_time_s=completion_time_s,
        safety_hold_duration_s=hold_duration_s,
        hold_duration_by_reason_s=tuple(sorted(hold_durations.items())),
        controller_stop_request_count=pipeline.controller_stop_request_count,
        gate_override_count=pipeline.gate_override_count,
        no_safe_candidate_count=pipeline.no_safe_candidate_count,
        path_length_m=path_length_m,
        signed_path_length_delta_m=signed_delta,
        positive_detour_length_m=max(0.0, signed_delta),
        maximum_reference_deviation_m=max(deviations, default=0.0),
        rms_reference_deviation_m=_rms(deviations),
        longitudinal_jerk_rms_mps3=_jerk_rms(
            tuple(twist.linear for twist in twists),
        ),
        angular_acceleration_rms_radps2=_acceleration_rms(
            tuple(twist.angular for twist in twists),
        ),
        angular_jerk_rms_radps3=_jerk_rms(
            tuple(twist.angular for twist in twists),
        ),
        peak_angular_velocity_radps=max(
            (abs(twist.angular) for twist in twists),
            default=0.0,
        ),
        angular_direction_change_count=_direction_changes(
            tuple(twist.angular for twist in twists),
        ),
        minimum_surface_clearance_m=minimum_clearance,
        minimum_ttc_s=minimum_ttc,
        rejoin_observed=rejoin_observed,
        overtaking_observed=overtaking_observed,
        planner_deadlock=planner_deadlock,
    )
    return DynamicEvaluationResult(
        controller_name=pipeline.controller_name,
        episode_id=episode_id,
        expectation_category=expectation_category,
        simulation_only=True,
        evaluator_frequency_hz=EVALUATOR_FREQUENCY_HZ,
        hard_safety=hard_safety,
        metrics=metrics,
        functional_qualified=not functional_failures,
        functional_failures=tuple(functional_failures),
    )


def _pipeline_step_is_finite(step: DynamicControllerPipelineStep) -> bool:
    values = []
    for state_name in ("robot_state_before", "robot_state_after"):
        state = getattr(step, state_name)
        values.extend(
            (
                state.pose.x,
                state.pose.y,
                state.pose.yaw,
                state.twist.linear,
                state.twist.angular,
            )
        )
    command = step.safety_decision.command
    values.extend((command.linear, command.angular))
    return all(isfinite(value) for value in values)


def _is_braking_only(current: Twist2D, requested: Twist2D) -> bool:
    return _toward_zero_only(current.linear, requested.linear) and _toward_zero_only(
        current.angular,
        requested.angular,
    )


def _toward_zero_only(current: float, requested: float) -> bool:
    if abs(requested) <= 1e-12:
        return True
    return current * requested > 0.0 and abs(requested) <= abs(current) + 1e-12


def _time_to_collision(
    robot_pose: Pose2D,
    robot_twist: Twist2D,
    actor: ActorState,
    surface_clearance_m: float,
) -> float | None:
    dx = actor.position.x - robot_pose.x
    dy = actor.position.y - robot_pose.y
    distance = hypot(dx, dy)
    if distance <= 1e-12:
        return 0.0
    robot_vx = robot_twist.linear * cos(robot_pose.yaw)
    robot_vy = robot_twist.linear * sin(robot_pose.yaw)
    separation_rate = (
        dx * (actor.velocity.x - robot_vx)
        + dy * (actor.velocity.y - robot_vy)
    ) / distance
    closing_speed = -separation_rate
    if closing_speed <= 1e-12:
        return None
    return surface_clearance_m / closing_speed


def _planner_deadlock(
    pipeline: DynamicControllerPipelineResult,
    path: tuple[Pose2D, ...],
    *,
    progressable: bool,
) -> bool:
    if not progressable or pipeline.completed:
        return False
    window = round(_DEADLOCK_WINDOW_S / DYNAMIC_CONTROL_PERIOD_S)
    if len(pipeline.steps) < window:
        return False
    for start in range(len(pipeline.steps) - window + 1):
        sample = pipeline.steps[start : start + window]
        if any(
            step.safety_decision.motion_state
            in {DynamicMotionState.BRAKING, DynamicMotionState.HOLDING}
            for step in sample
        ):
            continue
        first_arc, _ = _project_to_path(sample[0].robot_state_before.pose, path)
        last_arc, _ = _project_to_path(sample[-1].robot_state_after.pose, path)
        if last_arc - first_arc < _DEADLOCK_PROGRESS_M:
            return True
    return False


def _project_to_path(pose: Pose2D, path: tuple[Pose2D, ...]) -> tuple[float, float]:
    return _project_point_to_path(pose.x, pose.y, path)


def _project_point_to_path(
    x: float,
    y: float,
    path: tuple[Pose2D, ...],
) -> tuple[float, float]:
    best_arc = 0.0
    best_distance = float("inf")
    accumulated = 0.0
    for source, target in zip(path, path[1:], strict=False):
        dx = target.x - source.x
        dy = target.y - source.y
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-18:
            continue
        fraction = min(
            1.0,
            max(0.0, ((x - source.x) * dx + (y - source.y) * dy) / length_squared),
        )
        projection_x = source.x + fraction * dx
        projection_y = source.y + fraction * dy
        distance = hypot(x - projection_x, y - projection_y)
        length = sqrt(length_squared)
        if distance < best_distance:
            best_distance = distance
            best_arc = accumulated + fraction * length
        accumulated += length
    return best_arc, best_distance


def _acceleration_rms(values: tuple[float, ...]) -> float:
    accelerations = tuple(
        (current - previous) / DYNAMIC_CONTROL_PERIOD_S
        for previous, current in zip(values, values[1:], strict=False)
    )
    return _rms(accelerations)


def _jerk_rms(values: tuple[float, ...]) -> float:
    accelerations = tuple(
        (current - previous) / DYNAMIC_CONTROL_PERIOD_S
        for previous, current in zip(values, values[1:], strict=False)
    )
    jerks = tuple(
        (current - previous) / DYNAMIC_CONTROL_PERIOD_S
        for previous, current in zip(accelerations, accelerations[1:], strict=False)
    )
    return _rms(jerks)


def _direction_changes(values: tuple[float, ...]) -> int:
    signs = tuple(1 if value > 1e-12 else -1 for value in values if abs(value) > 1e-12)
    return sum(left != right for left, right in zip(signs, signs[1:], strict=False))


def _rms(values: tuple[float, ...]) -> float:
    return sqrt(fmean(value * value for value in values)) if values else 0.0


def _integrate(pose: Pose2D, command: Twist2D, dt_s: float) -> Pose2D:
    if abs(command.angular) <= 1e-12:
        return Pose2D(
            pose.x + command.linear * cos(pose.yaw) * dt_s,
            pose.y + command.linear * sin(pose.yaw) * dt_s,
            pose.yaw,
        )
    next_yaw = pose.yaw + command.angular * dt_s
    radius = command.linear / command.angular
    return Pose2D(
        pose.x + radius * (sin(next_yaw) - sin(pose.yaw)),
        pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
        _normalize_angle(next_yaw),
    )


def _integrate_simulation_tick(
    pose: Pose2D,
    command: Twist2D,
    dt_s: float,
) -> Pose2D:
    """Stage 4 simulator의 명시적 Euler endpoint 계약을 재검산한다."""

    return Pose2D(
        pose.x + command.linear * cos(pose.yaw) * dt_s,
        pose.y + command.linear * sin(pose.yaw) * dt_s,
        _normalize_angle(pose.yaw + command.angular * dt_s),
    )


def _poses_close(first: Pose2D, second: Pose2D) -> bool:
    return all(
        abs(left - right) <= 1e-8
        for left, right in (
            (first.x, second.x),
            (first.y, second.y),
            (_normalize_angle(first.yaw), _normalize_angle(second.yaw)),
        )
    )


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi
