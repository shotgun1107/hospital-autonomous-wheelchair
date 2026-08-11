"""동적 controller와 독립된 200 Hz ground-truth 평가기.

이 모듈은 controller가 사용한 prediction tube가 아니라 evaluator 전용 Actor 실제
궤적을 사용한다. 결과는 Python ``simulation_only`` 연구 증거이며 실제 사람 탑승
안전성이나 제품 알고리즘 채택 근거가 아니다.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin, sqrt
from statistics import fmean
from typing import TYPE_CHECKING

from hospital_path_lab.collision import (
    CollisionChecker,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.contracts import GridSnapshot, Pose2D, RobotState, Twist2D
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

if TYPE_CHECKING:
    from hospital_path_lab.dynamic_corpus import DynamicOracleSpec

EVALUATOR_FREQUENCY_HZ = 200.0
EVALUATOR_PERIOD_S = 1.0 / EVALUATOR_FREQUENCY_HZ
_GEOMETRY_TOLERANCE_M = 1e-9
_REJOIN_TOLERANCE_M = 0.10
_REJOIN_HEADING_TOLERANCE_RAD = 10.0 * pi / 180.0
_REJOIN_HOLD_S = 0.50
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
    maximum_rejoin_sustained_duration_s: float
    overtaking_observed: bool
    same_direction_overtaking_actor_ids: tuple[str, ...]
    protective_stop_epoch_count: int
    hazard_interval_stop_epoch_ids: tuple[tuple[int, ...], ...]
    resume_after_hold_count: int
    authorized_resume_stop_epochs: tuple[int, ...]
    authorized_resume_times_s: tuple[float, ...]
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
    category_oracle_applied: bool
    category_oracle_failures: tuple[str, ...]


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
    oracle_spec: DynamicOracleSpec | None = None,
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
    if (
        oracle_spec is not None
        and oracle_spec.expectation_category.value != expectation_category
    ):
        raise ValueError("evaluation category and oracle category must match")

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
    overtaken_actor_ids: set[str] = set()
    actor_overtake_times_s: dict[str, float] = {}
    actor_order: dict[str, float] = {}
    actor_longitudinal_overlap: set[str] = set()
    previous_motion_state: DynamicMotionState | None = None
    previous_decision_stop_epoch: int | None = None
    previous_robot_state_after: RobotState | None = None
    first_failure_time_s: float | None = None

    half_diagonal = hypot(
        profile.collision_length_m / 2.0,
        profile.collision_width_m / 2.0,
    )
    subdivisions = round(DYNAMIC_CONTROL_PERIOD_S / EVALUATOR_PERIOD_S)
    if subdivisions * EVALUATOR_PERIOD_S != DYNAMIC_CONTROL_PERIOD_S:
        raise AssertionError("200 Hz evaluator must divide the 20 Hz control period")

    for expected_tick, step in enumerate(pipeline.steps):
        expected_simulation_time_s = expected_tick * DYNAMIC_CONTROL_PERIOD_S
        if (
            step.tick_id != expected_tick
            or abs(step.simulation_time_s - expected_simulation_time_s) > 1e-12
            or step.controller_result.source_tick_id != step.tick_id
            or step.safety_decision.tick_id != step.tick_id
            or step.safety_decision.source_tick_id != step.tick_id
            or not step.controller_result.input_content_hash
            or (
                previous_robot_state_after is not None
                and not _robot_states_close(
                    previous_robot_state_after,
                    step.robot_state_before,
                )
            )
            or not _twists_close(
                step.robot_state_after.twist,
                step.safety_decision.command,
            )
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
        returning_to_motion = (
            previous_motion_state
            in {DynamicMotionState.BRAKING, DynamicMotionState.HOLDING}
            and step.safety_decision.motion_state is DynamicMotionState.MOVING
        )
        if returning_to_motion and (
            previous_motion_state is not DynamicMotionState.HOLDING
            or not step.safety_decision.resume_allowed
            or previous_decision_stop_epoch is None
            or previous_decision_stop_epoch <= 0
            or step.safety_decision.stop_epoch != previous_decision_stop_epoch
        ):
            unauthorized_resume_count += 1
            if first_failure_time_s is None:
                first_failure_time_s = step.simulation_time_s
        if previous_decision_stop_epoch is not None:
            epoch_delta = step.safety_decision.stop_epoch - previous_decision_stop_epoch
            epoch_transition_valid = epoch_delta in {0, 1} and (
                epoch_delta == 0
                or step.safety_decision.motion_state is DynamicMotionState.HOLDING
            )
            if not epoch_transition_valid:
                provenance_failure_count += 1
                if first_failure_time_s is None:
                    first_failure_time_s = step.simulation_time_s
        previous_motion_state = step.safety_decision.motion_state
        previous_decision_stop_epoch = step.safety_decision.stop_epoch

        checker = CollisionChecker(
            context_grid.grid,
            profile,
            forbidden_cells=context_grid.forbidden_cells,
        )
        start_index = 0 if expected_tick == 0 else 1
        for substep in range(start_index, subdivisions + 1):
            offset_s = substep * EVALUATOR_PERIOD_S
            evaluation_time_s = step.simulation_time_s + offset_s
            pose = _integrate_simulation_tick(
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

                robot_arc, _, path_heading = _project_to_path_with_heading(
                    pose,
                    reference_path,
                )
                actor_arc, actor_deviation, _ = _project_point_to_path_with_heading(
                    actor.position.x,
                    actor.position.y,
                    reference_path,
                )
                relevant_lateral_distance = (
                    actor.radius_m
                    + profile.collision_width_m / 2.0
                    + profile.minimum_clearance_m
                )
                eligible_overtake_actor = (
                    oracle_spec is None
                    or actor.actor_id in oracle_spec.same_direction_actor_ids
                )
                if actor_deviation <= relevant_lateral_distance and eligible_overtake_actor:
                    order = actor_arc - robot_arc
                    yaw_delta = _normalize_angle(pose.yaw - path_heading)
                    robot_longitudinal_extent = (
                        profile.collision_length_m / 2.0 * abs(cos(yaw_delta))
                        + profile.collision_width_m / 2.0 * abs(sin(yaw_delta))
                    )
                    if abs(order) <= robot_longitudinal_extent + actor.radius_m:
                        actor_longitudinal_overlap.add(actor.actor_id)
                    previous_order = actor_order.get(actor.actor_id)
                    if (
                        previous_order is not None
                        and previous_order > 0.0 >= order
                        and actor.actor_id in actor_longitudinal_overlap
                    ):
                        overtaken_actor_ids.add(actor.actor_id)
                        actor_overtake_times_s.setdefault(
                            actor.actor_id,
                            evaluation_time_s,
                        )
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
        previous_robot_state_after = step.robot_state_after

    if pipeline.steps and not _robot_states_close(
        pipeline.steps[-1].robot_state_after,
        pipeline.final_state,
    ):
        provenance_failure_count += 1
        if first_failure_time_s is None:
            first_failure_time_s = pipeline.steps[-1].simulation_time_s

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
    projections = tuple(
        _project_to_path_with_heading(pose, reference_path)
        for pose in evaluator_poses
    )
    deviations = tuple(projection[1] for projection in projections)
    heading_errors = tuple(
        abs(_normalize_angle(pose.yaw - projection[2]))
        for pose, projection in zip(evaluator_poses, projections, strict=True)
    )
    departure_threshold_m = (
        oracle_spec.departure_threshold_m
        if oracle_spec is not None
        else _REJOIN_TOLERANCE_M
    )
    rejoin_distance_m = (
        oracle_spec.rejoin_distance_m
        if oracle_spec is not None
        else _REJOIN_TOLERANCE_M
    )
    rejoin_heading_tolerance_rad = (
        oracle_spec.rejoin_heading_tolerance_deg * pi / 180.0
        if oracle_spec is not None
        else _REJOIN_HEADING_TOLERANCE_RAD
    )
    rejoin_hold_s = (
        oracle_spec.rejoin_hold_s if oracle_spec is not None else _REJOIN_HOLD_S
    )
    departed = False
    departure_time_s: float | None = None
    rejoin_observed = False
    current_rejoin_duration_s = 0.0
    maximum_rejoin_duration_s = 0.0
    for time_s, deviation, heading_error in zip(
        evaluator_times,
        deviations,
        heading_errors,
        strict=True,
    ):
        if deviation > departure_threshold_m:
            departed = True
            if departure_time_s is None:
                departure_time_s = time_s
            current_rejoin_duration_s = 0.0
        elif (
            departed
            and deviation <= rejoin_distance_m
            and heading_error <= rejoin_heading_tolerance_rad
        ):
            current_rejoin_duration_s += EVALUATOR_PERIOD_S
            maximum_rejoin_duration_s = max(
                maximum_rejoin_duration_s,
                current_rejoin_duration_s,
            )
            if current_rejoin_duration_s + 1e-12 >= rejoin_hold_s:
                rejoin_observed = True
        elif departed:
            current_rejoin_duration_s = 0.0

    expected_overtake_actor_ids = (
        frozenset(oracle_spec.same_direction_actor_ids)
        if oracle_spec is not None
        else frozenset()
    )
    ordered_detour_observed = False
    if expected_overtake_actor_ids and expected_overtake_actor_ids.issubset(
        actor_overtake_times_s
    ):
        earliest_overtake_s = min(
            actor_overtake_times_s[actor_id]
            for actor_id in expected_overtake_actor_ids
        )
        latest_overtake_s = max(
            actor_overtake_times_s[actor_id]
            for actor_id in expected_overtake_actor_ids
        )
        ordered_rejoin_at_s = _sustained_rejoin_completion_time(
            evaluator_times,
            deviations,
            heading_errors,
            start_time_s=latest_overtake_s,
            distance_m=rejoin_distance_m,
            heading_tolerance_rad=rejoin_heading_tolerance_rad,
            hold_s=rejoin_hold_s,
        )
        ordered_detour_observed = all(
            (
                departure_time_s is not None,
                departure_time_s is not None
                and departure_time_s < earliest_overtake_s,
                ordered_rejoin_at_s is not None,
                all(
                    actor_order.get(actor_id, float("inf")) <= 0.0
                    for actor_id in expected_overtake_actor_ids
                ),
            )
        )

    hold_durations: dict[str, float] = {}
    hold_duration_s = 0.0
    protective_stop_epochs: set[int] = set()
    hazard_stop_epochs = (
        [set() for _ in oracle_spec.hazard_intervals_s]
        if oracle_spec is not None
        else []
    )
    pending_hazard_intervals: set[int] = set()
    previous_stop_epoch = 0
    stop_epoch_confirmed_at_s: dict[int, float] = {}
    authorized_resume_events: list[tuple[int, float]] = []
    previous_motion_state = None
    for step in pipeline.steps:
        protective_stop = step.safety_decision.motion_state in {
            DynamicMotionState.BRAKING,
            DynamicMotionState.HOLDING,
        }
        if protective_stop:
            hold_duration_s += DYNAMIC_CONTROL_PERIOD_S
            reason = step.safety_decision.primary_hold_reason
            key = reason.value if reason is not None else "unspecified"
            hold_durations[key] = hold_durations.get(key, 0.0) + DYNAMIC_CONTROL_PERIOD_S
            if reason in {
                DynamicHoldReason.TRAFFIC,
                DynamicHoldReason.NO_SAFE_CANDIDATE,
            }:
                for index, (start_s, end_s) in enumerate(
                    oracle_spec.hazard_intervals_s if oracle_spec is not None else ()
                ):
                    if start_s <= step.simulation_time_s <= end_s:
                        pending_hazard_intervals.add(index)
            if step.safety_decision.stop_epoch > 0:
                protective_stop_epochs.add(step.safety_decision.stop_epoch)
            if step.safety_decision.stop_epoch == previous_stop_epoch + 1:
                stop_epoch_confirmed_at_s[step.safety_decision.stop_epoch] = (
                    step.simulation_time_s
                )
                for index in pending_hazard_intervals:
                    hazard_stop_epochs[index].add(step.safety_decision.stop_epoch)
                pending_hazard_intervals.clear()
        elif (
            previous_motion_state is DynamicMotionState.HOLDING
            and step.safety_decision.motion_state is DynamicMotionState.MOVING
            and step.safety_decision.resume_allowed
            and previous_stop_epoch > 0
            and step.safety_decision.stop_epoch == previous_stop_epoch
        ):
            authorized_resume_events.append(
                (step.safety_decision.stop_epoch, step.simulation_time_s)
            )
            pending_hazard_intervals.clear()
        if step.safety_decision.stop_epoch in {
            previous_stop_epoch,
            previous_stop_epoch + 1,
        }:
            previous_stop_epoch = step.safety_decision.stop_epoch
        previous_motion_state = step.safety_decision.motion_state

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

    category_oracle_failures = _category_oracle_failures(
        pipeline,
        expectation_category=expectation_category,
        oracle_spec=oracle_spec,
        maximum_reference_deviation_m=max(deviations, default=0.0),
        rejoin_observed=rejoin_observed,
        overtaken_actor_ids=frozenset(overtaken_actor_ids),
        ordered_detour_observed=ordered_detour_observed,
        protective_stop_epoch_count=len(protective_stop_epochs),
        hazard_interval_stop_epoch_ids=tuple(
            tuple(sorted(epochs)) for epochs in hazard_stop_epochs
        ),
        authorized_resume_events=tuple(authorized_resume_events),
        stop_epoch_confirmed_at_s=stop_epoch_confirmed_at_s,
    )
    functional_failures.extend(category_oracle_failures)

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
        maximum_rejoin_sustained_duration_s=maximum_rejoin_duration_s,
        overtaking_observed=bool(overtaken_actor_ids),
        same_direction_overtaking_actor_ids=tuple(sorted(overtaken_actor_ids)),
        protective_stop_epoch_count=len(protective_stop_epochs),
        hazard_interval_stop_epoch_ids=tuple(
            tuple(sorted(epochs)) for epochs in hazard_stop_epochs
        ),
        resume_after_hold_count=len(authorized_resume_events),
        authorized_resume_stop_epochs=tuple(
            epoch for epoch, _ in authorized_resume_events
        ),
        authorized_resume_times_s=tuple(
            time_s for _, time_s in authorized_resume_events
        ),
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
        category_oracle_applied=oracle_spec is not None,
        category_oracle_failures=category_oracle_failures,
    )


def _sustained_rejoin_completion_time(
    times_s: list[float],
    deviations_m: tuple[float, ...],
    heading_errors_rad: tuple[float, ...],
    *,
    start_time_s: float,
    distance_m: float,
    heading_tolerance_rad: float,
    hold_s: float,
) -> float | None:
    qualifying_since_s: float | None = None
    for time_s, deviation_m, heading_error_rad in zip(
        times_s,
        deviations_m,
        heading_errors_rad,
        strict=True,
    ):
        if time_s + 1e-12 < start_time_s:
            continue
        if (
            deviation_m <= distance_m
            and heading_error_rad <= heading_tolerance_rad
        ):
            if qualifying_since_s is None:
                qualifying_since_s = time_s
            if time_s - qualifying_since_s + 1e-12 >= hold_s:
                return time_s
        else:
            qualifying_since_s = None
    return None


def _category_oracle_failures(
    pipeline: DynamicControllerPipelineResult,
    *,
    expectation_category: str,
    oracle_spec: DynamicOracleSpec | None,
    maximum_reference_deviation_m: float,
    rejoin_observed: bool,
    overtaken_actor_ids: frozenset[str],
    ordered_detour_observed: bool,
    protective_stop_epoch_count: int,
    hazard_interval_stop_epoch_ids: tuple[tuple[int, ...], ...],
    authorized_resume_events: tuple[tuple[int, float], ...],
    stop_epoch_confirmed_at_s: dict[int, float],
) -> tuple[str, ...]:
    if oracle_spec is None:
        return ()
    failures: list[str] = []
    controller_is_dwa = pipeline.controller_name == "dynamic_dwa"
    controller_is_pp = pipeline.controller_name == "dynamic_pure_pursuit"
    if not controller_is_dwa and not controller_is_pp:
        return ("unsupported_controller_for_category_oracle",)
    required_stops = oracle_spec.required_protective_stop_epochs
    hazard_epochs = {
        epoch
        for interval_epochs in hazard_interval_stop_epoch_ids
        for epoch in interval_epochs
    }
    hazard_intervals_covered = sum(bool(epochs) for epochs in hazard_interval_stop_epoch_ids)
    resumed_stop_epochs = {epoch for epoch, _ in authorized_resume_events}

    if expectation_category == "wait_and_resume":
        # PP는 기준경로를 유지하므로 보호정지/재개를 요구한다. DWA는 hard-safety를
        # 지키며 진행할 수 있어 정지를 강제하지 않는다.
        if controller_is_pp and len(hazard_epochs) < required_stops:
            failures.append("wait_and_resume_missing_protective_stop")
        if (
            controller_is_pp
            and required_stops
            and not hazard_epochs.intersection(resumed_stop_epochs)
        ):
            failures.append("wait_and_resume_missing_authorized_resume")
    elif expectation_category == "local_detour_feasible":
        if oracle_spec.feasible_witness is None:
            failures.append("feasible_label_missing_independent_witness")
        if controller_is_dwa:
            if maximum_reference_deviation_m <= oracle_spec.departure_threshold_m:
                failures.append("dwa_detour_departure_not_observed")
            if not rejoin_observed:
                failures.append("dwa_sustained_rejoin_not_observed")
            expected_overtakes = frozenset(oracle_spec.same_direction_actor_ids)
            if expected_overtakes and not expected_overtakes.issubset(
                overtaken_actor_ids
            ):
                failures.append("dwa_same_direction_overtaking_not_observed")
            if not ordered_detour_observed:
                failures.append("dwa_detour_overtake_rejoin_sequence_invalid")
        elif controller_is_pp:
            if len(hazard_epochs) < required_stops:
                failures.append("pp_feasible_case_did_not_wait")
            if required_stops and not hazard_epochs.intersection(resumed_stop_epochs):
                failures.append("pp_feasible_case_did_not_resume")
    elif expectation_category == "local_detour_forbidden":
        if overtaken_actor_ids:
            failures.append("forbidden_overtaking_observed")
        if len(hazard_epochs) < required_stops:
            failures.append("forbidden_case_missing_protective_stop")
        if required_stops and not hazard_epochs.intersection(resumed_stop_epochs):
            failures.append("forbidden_case_missing_resume_after_clear")
    elif expectation_category == "no_safe_solution":
        if pipeline.completed:
            failures.append("no_safe_solution_completed")
        if any(
            step.safety_decision.motion_state is DynamicMotionState.MOVING
            and (
                abs(step.safety_decision.command.linear) > 1e-12
                or abs(step.safety_decision.command.angular) > 1e-12
            )
            for step in pipeline.steps
        ):
            failures.append("no_safe_solution_motion_observed")
    elif expectation_category == "observation_invalid":
        if protective_stop_epoch_count < max(1, required_stops):
            failures.append("observation_invalid_missing_brake_and_hold")
    elif expectation_category == "dynamic_change_restop":
        if (
            len(hazard_epochs) < required_stops
            or hazard_intervals_covered < len(hazard_interval_stop_epoch_ids)
        ):
            failures.append("second_protective_stop_epoch_not_observed")
        ordered_hazard_epochs = tuple(
            min(epochs) if epochs else None
            for epochs in hazard_interval_stop_epoch_ids
        )
        if any(
            left is not None and right is not None and left >= right
            for left, right in zip(
                ordered_hazard_epochs,
                ordered_hazard_epochs[1:],
                strict=False,
            )
        ):
            failures.append("hazard_stop_epochs_out_of_order")
        intermediate_resume_missing = any(
            current_epoch is None
            or not any(
                resumed_epoch == current_epoch
                and stop_epoch_confirmed_at_s.get(current_epoch, float("inf"))
                <= resume_time_s
                < oracle_spec.hazard_intervals_s[index + 1][0]
                for resumed_epoch, resume_time_s in authorized_resume_events
            )
            for index, current_epoch in enumerate(ordered_hazard_epochs[:-1])
        )
        if required_stops > 1 and intermediate_resume_missing:
            failures.append("dynamic_change_missing_intermediate_resume")
    else:
        failures.append("unsupported_v6_expectation_category")
    return tuple(failures)


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


def _project_to_path_with_heading(
    pose: Pose2D,
    path: tuple[Pose2D, ...],
) -> tuple[float, float, float]:
    return _project_point_to_path_with_heading(pose.x, pose.y, path)


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


def _project_point_to_path_with_heading(
    x: float,
    y: float,
    path: tuple[Pose2D, ...],
) -> tuple[float, float, float]:
    best_arc = 0.0
    best_distance = float("inf")
    best_heading = 0.0
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
            best_heading = _normalize_angle(atan2(dy, dx))
        accumulated += length
    return best_arc, best_distance, best_heading


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


def _twists_close(first: Twist2D, second: Twist2D) -> bool:
    return (
        abs(first.linear - second.linear) <= 1e-12
        and abs(first.angular - second.angular) <= 1e-12
    )


def _robot_states_close(first: RobotState, second: RobotState) -> bool:
    return _poses_close(first.pose, second.pose) and _twists_close(
        first.twist,
        second.twist,
    )


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi
