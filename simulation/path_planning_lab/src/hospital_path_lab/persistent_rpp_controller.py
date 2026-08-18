"""R5 immutable reference를 실행하는 persistent RPP 연구 adapter.

기존 :class:`RegulatedPurePursuitFollower`의 동결 수치를 사용하지만, sliding
window 끝을 goal로 오인하지 않도록 lookahead 경로와 감속 기준을 분리한다.
Translation 이외의 planned stop, rotation, terminal dwell과 HOLD는 공통
``ReferenceSectionExecutor``에 위임한다. 이 모듈은 shared safety gate나 실제 이동
허가를 대신하지 않는 Python ``simulation_only`` 기능 하네스다.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, isclose, isfinite, sin, sqrt
from time import perf_counter_ns

from hospital_path_lab.contracts import Pose2D, TrajectoryPoint, Twist2D
from hospital_path_lab.local_reference_contracts import (
    ReferenceKnot,
    ReferenceKnotRole,
    ReferenceSection,
    ReferenceSectionKind,
    ReferenceTravelDirection,
)
from hospital_path_lab.persistent_controller_contracts import (
    PERSISTENT_CONTROLLER_RESULT_SCHEMA_VERSION,
    PersistentControllerResult,
    PersistentControllerStatus,
    PersistentControllerTickInput,
)
from hospital_path_lab.reference_section_executor import (
    R5_ANGULAR_ACCELERATION_RADPS2,
    R5_ANGULAR_DECELERATION_RADPS2,
    R5_CONTROL_PERIOD_S,
    ReferenceExecutorAction,
    ReferenceSectionExecutionDecision,
    ReferenceSectionExecutor,
    translation_completion_tolerance_m,
)

PERSISTENT_RPP_CONTROLLER_NAME = "persistent_rpp_reference"
PERSISTENT_RPP_CONTROLLER_VERSION = "persistent-rpp-reference-v4"
PERSISTENT_RPP_LOOKAHEAD_MIN_M = 0.25
PERSISTENT_RPP_LOOKAHEAD_MAX_M = 0.50
PERSISTENT_RPP_LOOKAHEAD_VELOCITY_GAIN = 0.75
PERSISTENT_RPP_MINIMUM_TRACKING_SPEED_MPS = 0.05
PERSISTENT_RPP_CURVATURE_GAIN = 2.0
PERSISTENT_RPP_ROLLOUT_HORIZON_S = 2.0
PERSISTENT_RPP_ROLLOUT_STEP_S = 0.05
PERSISTENT_RPP_MAXIMUM_REVERSE_SPEED_MPS = 0.10

_TRANSLATION_SECTION_KINDS = frozenset(
    {
        ReferenceSectionKind.FOLLOW_ORIGINAL,
        ReferenceSectionKind.DEPART,
        ReferenceSectionKind.BYPASS,
        ReferenceSectionKind.RETURN,
        ReferenceSectionKind.REJOIN,
    }
)
_REFERENCE_INPUT_FAILURES = frozenset(
    {
        "candidate_changed_without_maneuver_revision",
        "controller_tick_gap",
        "controller_tick_regression",
        "current_binding_is_terminal",
        "initial_binding_not_available",
        "incoming_binding_not_available",
        "maneuver_revision_requires_new_session",
        "maneuver_revision_advanced_without_new_session",
        "mission_id_mismatch",
        "new_path_requires_new_session",
        "path_revision_requires_new_session",
        "path_revision_without_content_change",
        "path_changed_without_path_revision",
        "reference_session_changed_without_revision",
        "revision_regression",
        "same_revision_different_content",
        "same_path_revision_different_reference",
        "same_tick_input_changed",
        "stop_epoch_regression",
        "stop_epoch_requires_new_maneuver_revision",
        "stop_epoch_requires_new_session",
        "subgoal_revision_without_window_change",
        "subgoal_revision_advanced_without_window_change",
        "window_update_changed_session",
    }
)
_TOLERANCE = 1e-12


@dataclass(frozen=True, slots=True)
class PersistentRppConfig:
    """R5-3에서 동결한 기존 RPP 수치와 rollout 의미."""

    control_period_s: float = R5_CONTROL_PERIOD_S
    lookahead_min_m: float = PERSISTENT_RPP_LOOKAHEAD_MIN_M
    lookahead_max_m: float = PERSISTENT_RPP_LOOKAHEAD_MAX_M
    lookahead_velocity_gain: float = PERSISTENT_RPP_LOOKAHEAD_VELOCITY_GAIN
    minimum_tracking_speed_mps: float = PERSISTENT_RPP_MINIMUM_TRACKING_SPEED_MPS
    curvature_gain: float = PERSISTENT_RPP_CURVATURE_GAIN
    rollout_horizon_s: float = PERSISTENT_RPP_ROLLOUT_HORIZON_S
    rollout_step_s: float = PERSISTENT_RPP_ROLLOUT_STEP_S
    angular_acceleration_radps2: float = R5_ANGULAR_ACCELERATION_RADPS2
    angular_deceleration_radps2: float = R5_ANGULAR_DECELERATION_RADPS2
    maximum_reverse_speed_mps: float = PERSISTENT_RPP_MAXIMUM_REVERSE_SPEED_MPS

    def __post_init__(self) -> None:
        fields = PersistentRppConfig.__dataclass_fields__
        for name, field in fields.items():
            value = getattr(self, name)
            if not isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            if not isclose(value, field.default, rel_tol=0.0, abs_tol=_TOLERANCE):
                raise ValueError(f"{name} is frozen for R5-3")
        rollout_steps = self.rollout_horizon_s / self.rollout_step_s
        if not isclose(rollout_steps, round(rollout_steps), abs_tol=_TOLERANCE):
            raise ValueError("rollout horizon must contain an exact number of steps")


@dataclass(frozen=True, slots=True)
class _TranslationCommand:
    command: Twist2D
    lookahead_point: Pose2D
    lookahead_distance_m: float
    active_full_progress_m: float
    active_section_remaining_m: float
    tracking_error_m: float
    curvature_inverse_m: float
    curvature_regulated_speed_mps: float
    stop_limited_speed_mps: float | None
    terminal_goal_active: bool
    explicit_stop_active: bool
    travel_direction: ReferenceTravelDirection


class PersistentRppController:
    """공통 section executor 위에서 translation만 persistent RPP로 계산한다."""

    name = PERSISTENT_RPP_CONTROLLER_NAME

    def __init__(
        self,
        config: PersistentRppConfig | None = None,
        executor: ReferenceSectionExecutor | None = None,
    ) -> None:
        self.config = config or PersistentRppConfig()
        self._executor = executor or ReferenceSectionExecutor()
        if not isclose(
            self.config.control_period_s,
            self._executor.config.control_period_s,
            rel_tol=0.0,
            abs_tol=_TOLERANCE,
        ):
            raise ValueError("RPP and section executor control periods must match")
        self._last_tick: int | None = None
        self._last_input_hash: str | None = None
        self._last_result: PersistentControllerResult | None = None
        self._false_local_goal_deceleration_count = 0

    @property
    def session_reset_count(self) -> int:
        return self._executor.session_reset_count

    @property
    def window_update_count(self) -> int:
        return self._executor.window_update_count

    @property
    def active_section_index(self) -> int | None:
        return self._executor.active_section_index

    @property
    def false_local_goal_deceleration_count(self) -> int:
        return self._false_local_goal_deceleration_count

    def step(self, tick_input: PersistentControllerTickInput) -> PersistentControllerResult:
        started_at = perf_counter_ns()
        if not isinstance(tick_input, PersistentControllerTickInput):
            raise TypeError("tick_input must be a PersistentControllerTickInput")
        if (
            self._last_tick == tick_input.controller_tick
            and self._last_input_hash == tick_input.tick_input_content_hash
        ):
            if self._last_result is None:  # pragma: no cover - internal invariant
                raise RuntimeError("cached RPP tick has no result")
            return self._last_result

        decision = self._executor.step(tick_input)
        if decision.action is ReferenceExecutorAction.DELEGATE_TRANSLATION:
            result = self._translation_result(tick_input, decision, started_at)
        else:
            result = self._common_result(tick_input, decision, started_at)
        self._last_tick = tick_input.controller_tick
        self._last_input_hash = tick_input.tick_input_content_hash
        self._last_result = result
        return result

    def _translation_result(
        self,
        tick_input: PersistentControllerTickInput,
        decision: ReferenceSectionExecutionDecision,
        started_at: int,
    ) -> PersistentControllerResult:
        if decision.active_section_index is None or decision.active_section_kind not in (
            _TRANSLATION_SECTION_KINDS
        ):
            return self._result(
                tick_input,
                decision,
                started_at,
                status=PersistentControllerStatus.SECTION_EXECUTION_FAILED,
                failure_reason="translation_delegate_requires_translation_section",
                controller_requested_protective_stop=True,
            )
        try:
            translation = _compute_translation_command(
                tick_input,
                decision.active_section_index,
                self.config,
            )
        except (IndexError, TypeError, ValueError) as error:
            return self._result(
                tick_input,
                decision,
                started_at,
                status=PersistentControllerStatus.SECTION_EXECUTION_FAILED,
                failure_reason=f"persistent_rpp_translation_invalid:{type(error).__name__}",
                controller_requested_protective_stop=True,
                decision_trace=(str(error),),
            )

        planned_stop_rollout = (
            translation.explicit_stop_active or translation.terminal_goal_active
        )
        if planned_stop_rollout:
            trajectory = _post_apply_bounded_stop_rollout(
                tick_input.robot_state.pose,
                tick_input.robot_state.twist,
                translation.command,
                linear_deceleration_mps2=(
                    tick_input.vehicle_profile.max_deceleration_mps2
                ),
                config=self.config,
            )
        else:
            trajectory = _post_apply_constant_rollout(
                tick_input.robot_state.pose,
                tick_input.robot_state.twist,
                translation.command,
                self.config,
            )
        trace = (
            f"rpp_version={PERSISTENT_RPP_CONTROLLER_VERSION}",
            f"lookahead_x={translation.lookahead_point.x:.12g}",
            f"lookahead_y={translation.lookahead_point.y:.12g}",
            f"lookahead_distance_m={translation.lookahead_distance_m:.12g}",
            f"active_full_progress_m={translation.active_full_progress_m:.12g}",
            f"active_section_remaining_m={translation.active_section_remaining_m:.12g}",
            f"curvature_inverse_m={translation.curvature_inverse_m:.12g}",
            f"curvature_regulated_speed_mps={translation.curvature_regulated_speed_mps:.12g}",
            "stop_limited_speed_mps=none"
            if translation.stop_limited_speed_mps is None
            else f"stop_limited_speed_mps={translation.stop_limited_speed_mps:.12g}",
            f"terminal_goal_active={str(translation.terminal_goal_active).lower()}",
            f"explicit_stop_active={str(translation.explicit_stop_active).lower()}",
            f"travel_direction={translation.travel_direction.value}",
            "rollout_policy=one_step_then_bounded_stop"
            if planned_stop_rollout
            else "rollout_policy=constant_command",
            "local_window_endpoint_is_not_goal=true",
        )
        diagnostics = tuple(
            sorted(
                {
                    "detour_generated=false",
                    "false_local_goal_deceleration=false",
                    "lookahead_source=current_window_translation",
                    "progress_source=full_reference_active_section",
                    f"travel_direction={translation.travel_direction.value}",
                }
            )
        )
        return self._result(
            tick_input,
            decision,
            started_at,
            status=PersistentControllerStatus.COMMAND_FOUND,
            requested_twist=translation.command,
            predicted_trajectory=trajectory,
            tracking_error_m=translation.tracking_error_m,
            decision_trace=trace,
            candidate_diagnostics=diagnostics,
        )

    def _common_result(
        self,
        tick_input: PersistentControllerTickInput,
        decision: ReferenceSectionExecutionDecision,
        started_at: int,
    ) -> PersistentControllerResult:
        command = decision.common_command or Twist2D()
        trace = (f"rpp_version={PERSISTENT_RPP_CONTROLLER_VERSION}",)
        if decision.action is ReferenceExecutorAction.MISSION_COMPLETED:
            return self._result(
                tick_input,
                decision,
                started_at,
                status=PersistentControllerStatus.COMPLETED,
                decision_trace=trace,
            )
        if decision.action is ReferenceExecutorAction.REQUEST_PROTECTIVE_HOLD:
            failure = decision.failure_reason or "reference_executor_protective_hold"
            if decision.active_section_kind is ReferenceSectionKind.HOLD:
                status = PersistentControllerStatus.HOLD_REQUESTED
            elif failure in _REFERENCE_INPUT_FAILURES:
                status = PersistentControllerStatus.INVALID_REFERENCE_INPUT
            else:
                status = PersistentControllerStatus.SECTION_EXECUTION_FAILED
            return self._result(
                tick_input,
                decision,
                started_at,
                status=status,
                failure_reason=failure,
                controller_requested_protective_stop=True,
                decision_trace=trace,
            )
        trajectory = _post_apply_constant_rollout(
            tick_input.robot_state.pose,
            tick_input.robot_state.twist,
            command,
            self.config,
        )
        if decision.planned_section_stop:
            return self._result(
                tick_input,
                decision,
                started_at,
                status=PersistentControllerStatus.PLANNED_STOP,
                requested_twist=command,
                predicted_trajectory=trajectory,
                planned_section_stop=True,
                decision_trace=trace,
            )
        return self._result(
            tick_input,
            decision,
            started_at,
            status=PersistentControllerStatus.COMMAND_FOUND,
            requested_twist=command,
            predicted_trajectory=trajectory,
            decision_trace=trace,
        )

    def _result(
        self,
        tick_input: PersistentControllerTickInput,
        decision: ReferenceSectionExecutionDecision,
        started_at: int,
        *,
        status: PersistentControllerStatus,
        requested_twist: Twist2D | None = None,
        predicted_trajectory: tuple[TrajectoryPoint, ...] = (),
        failure_reason: str | None = None,
        decision_trace: tuple[str, ...] = (),
        tracking_error_m: float | None = None,
        candidate_diagnostics: tuple[str, ...] = (),
        planned_section_stop: bool = False,
        controller_requested_protective_stop: bool = False,
    ) -> PersistentControllerResult:
        return PersistentControllerResult(
            schema_version=PERSISTENT_CONTROLLER_RESULT_SCHEMA_VERSION,
            controller_name=self.name,
            source_controller_tick=tick_input.controller_tick,
            status=status,
            requested_twist=Twist2D() if requested_twist is None else requested_twist,
            predicted_trajectory=predicted_trajectory,
            failure_reason=failure_reason,
            decision_trace=decision.decision_trace + decision_trace,
            reference_binding_echo=tick_input.reference_binding,
            tick_input_content_hash=tick_input.tick_input_content_hash,
            controller_session_transition=decision.session_transition,
            executor_state=decision.executor_state,
            active_section_index=decision.active_section_index,
            active_section_kind=decision.active_section_kind,
            tracking_error_m=tracking_error_m,
            candidate_diagnostics=tuple(sorted(set(candidate_diagnostics))),
            planned_section_stop=planned_section_stop,
            controller_requested_protective_stop=controller_requested_protective_stop,
            no_safe_candidate=False,
            elapsed_nonqualification_ns=perf_counter_ns() - started_at,
        )


def _compute_translation_command(
    tick_input: PersistentControllerTickInput,
    active_section_index: int,
    config: PersistentRppConfig,
) -> _TranslationCommand:
    reference = tick_input.full_reference
    if not 0 <= active_section_index < len(reference.sections):
        raise ValueError("active section index is outside the full reference")
    section = reference.sections[active_section_index]
    if section.section_kind not in _TRANSLATION_SECTION_KINDS:
        raise ValueError("active section is not translational")
    direction = section.travel_direction
    if direction not in {
        ReferenceTravelDirection.FORWARD,
        ReferenceTravelDirection.REVERSE,
    }:
        raise ValueError("active translation section has no executable direction")
    window_section = next(
        (
            candidate
            for candidate in tick_input.local_window.sections
            if candidate.section_index == active_section_index
        ),
        None,
    )
    if window_section != section:
        raise ValueError("active full-reference section is not exact in current window")
    full_path = _section_path(reference.knots, section)
    window_path = _section_path(tick_input.local_window.knots, section)
    if window_path != full_path:
        raise ValueError("window translation path differs from active full section")

    pose = tick_input.robot_state.pose
    full_projection = _project_polyline(full_path, pose)
    window_projection = _project_polyline(window_path, pose)
    lookahead_distance = _clip(
        config.lookahead_min_m
        + config.lookahead_velocity_gain * abs(tick_input.robot_state.twist.linear),
        config.lookahead_min_m,
        config.lookahead_max_m,
    )
    lookahead = _point_at_arc(
        window_path,
        window_projection.cumulative,
        min(window_projection.progress_m + lookahead_distance, window_projection.total_arc_m),
    )
    curvature = _curvature_to_point(pose, lookahead)
    profile = tick_input.vehicle_profile
    maximum_speed = (
        profile.nominal_speed_mps
        if direction is ReferenceTravelDirection.FORWARD
        else min(profile.max_reverse_speed_mps, config.maximum_reverse_speed_mps)
    )
    curvature_speed = _clip(
        maximum_speed / (1.0 + config.curvature_gain * abs(curvature)),
        config.minimum_tracking_speed_mps,
        maximum_speed,
    )

    terminal_goal_active = (
        active_section_index == len(reference.sections) - 1
        and tick_input.local_window.terminal_rejoin_included
    )
    explicit_stop_active = _section_has_upcoming_stop(reference.sections, reference.knots, section)
    stop_limited_speed: float | None = None
    desired_speed = curvature_speed
    if terminal_goal_active or explicit_stop_active:
        section_end = full_path[-1]
        geometric_remaining = hypot(section_end.x - pose.x, section_end.y - pose.y)
        completion_tolerance = translation_completion_tolerance_m(
            reference,
            active_section_index,
        )
        distance_to_tolerance = max(
            0.0,
            max(
                full_projection.total_arc_m - full_projection.progress_m,
                geometric_remaining,
            )
            - completion_tolerance,
        )
        stop_limited_speed = min(
            maximum_speed,
            sqrt(2.0 * profile.max_deceleration_mps2 * distance_to_tolerance),
        )
        desired_speed = min(desired_speed, stop_limited_speed)

    desired_linear = (
        desired_speed
        if direction is ReferenceTravelDirection.FORWARD
        else -desired_speed
    )
    current_linear = tick_input.robot_state.twist.linear
    if (
        direction is ReferenceTravelDirection.FORWARD
        and current_linear < -0.01 - _TOLERANCE
    ) or (
        direction is ReferenceTravelDirection.REVERSE
        and current_linear > 0.01 + _TOLERANCE
    ):
        raise ValueError("translation direction changed before actual stop")

    linear = _rate_limited_velocity(
        current_linear,
        desired_linear,
        acceleration_step=profile.max_acceleration_mps2 * config.control_period_s,
        deceleration_step=profile.max_deceleration_mps2 * config.control_period_s,
    )
    desired_angular = _clip(
        linear * curvature,
        -profile.max_angular_speed_radps,
        profile.max_angular_speed_radps,
    )
    angular = _rate_limited_velocity(
        tick_input.robot_state.twist.angular,
        desired_angular,
        acceleration_step=config.angular_acceleration_radps2 * config.control_period_s,
        deceleration_step=config.angular_deceleration_radps2 * config.control_period_s,
    )
    return _TranslationCommand(
        command=Twist2D(linear, angular),
        lookahead_point=lookahead,
        lookahead_distance_m=lookahead_distance,
        active_full_progress_m=(
            reference.knots[section.first_knot_index].cumulative_translation_arc_m
            + full_projection.progress_m
        ),
        active_section_remaining_m=max(
            0.0,
            full_projection.total_arc_m - full_projection.progress_m,
        ),
        tracking_error_m=full_projection.distance_m,
        curvature_inverse_m=curvature,
        curvature_regulated_speed_mps=curvature_speed,
        stop_limited_speed_mps=stop_limited_speed,
        terminal_goal_active=terminal_goal_active,
        explicit_stop_active=explicit_stop_active,
        travel_direction=direction,
    )


@dataclass(frozen=True, slots=True)
class _PolylineProjection:
    cumulative: tuple[float, ...]
    progress_m: float
    distance_m: float
    total_arc_m: float


def _section_path(
    knots: tuple[ReferenceKnot, ...],
    section: ReferenceSection,
) -> tuple[Pose2D, ...]:
    poses = tuple(
        knot.pose
        for knot in knots
        if section.first_knot_index <= knot.knot_index <= section.last_knot_index
    )
    if not poses:
        raise ValueError("active section has no knots in the selected path source")
    if any(not all(isfinite(value) for value in (pose.x, pose.y, pose.yaw)) for pose in poses):
        raise ValueError("active translation path contains nonfinite pose")
    return poses


def _project_polyline(path: tuple[Pose2D, ...], pose: Pose2D) -> _PolylineProjection:
    if len(path) == 1:
        return _PolylineProjection((0.0,), 0.0, hypot(pose.x - path[0].x, pose.y - path[0].y), 0.0)
    cumulative = [0.0]
    best: tuple[float, float] | None = None
    for left, right in zip(path, path[1:], strict=False):
        cumulative.append(cumulative[-1] + hypot(right.x - left.x, right.y - left.y))
    for index, (left, right) in enumerate(zip(path, path[1:], strict=False)):
        dx = right.x - left.x
        dy = right.y - left.y
        length = hypot(dx, dy)
        if length <= _TOLERANCE:
            continue
        fraction = _clip(
            ((pose.x - left.x) * dx + (pose.y - left.y) * dy) / (length * length),
            0.0,
            1.0,
        )
        projected_x = left.x + fraction * dx
        projected_y = left.y + fraction * dy
        candidate = (
            hypot(pose.x - projected_x, pose.y - projected_y),
            cumulative[index] + fraction * length,
        )
        if best is None or (candidate[0], -candidate[1]) < (best[0], -best[1]):
            best = candidate
    if best is None:
        distance = min(hypot(pose.x - point.x, pose.y - point.y) for point in path)
        return _PolylineProjection(tuple(cumulative), 0.0, distance, cumulative[-1])
    return _PolylineProjection(tuple(cumulative), best[1], best[0], cumulative[-1])


def _point_at_arc(
    path: tuple[Pose2D, ...],
    cumulative: tuple[float, ...],
    target_arc_m: float,
) -> Pose2D:
    if len(path) == 1:
        return path[0]
    for index, (left, right) in enumerate(zip(path, path[1:], strict=False)):
        start = cumulative[index]
        end = cumulative[index + 1]
        if target_arc_m > end + _TOLERANCE and index < len(path) - 2:
            continue
        length = end - start
        if length <= _TOLERANCE:
            continue
        fraction = _clip((target_arc_m - start) / length, 0.0, 1.0)
        dx = right.x - left.x
        dy = right.y - left.y
        return Pose2D(
            left.x + fraction * dx,
            left.y + fraction * dy,
            atan2(dy, dx),
        )
    return path[-1]


def _section_has_upcoming_stop(
    sections: tuple[ReferenceSection, ...],
    knots: tuple[ReferenceKnot, ...],
    section: ReferenceSection,
) -> bool:
    end_knot = knots[section.last_knot_index]
    if section.exit_requires_stopped or ReferenceKnotRole.STOP_MARKER in end_knot.knot_roles:
        return True
    end_arc = end_knot.cumulative_translation_arc_m
    for following in sections[section.section_index + 1 :]:
        following_end = knots[following.last_knot_index]
        if following.section_kind in {ReferenceSectionKind.ROTATE, ReferenceSectionKind.HOLD}:
            return following_end.cumulative_translation_arc_m <= end_arc + _TOLERANCE
        if following.entry_requires_stopped:
            return True
        if following_end.cumulative_translation_arc_m > end_arc + _TOLERANCE:
            return False
        if following.exit_requires_stopped or (
            ReferenceKnotRole.STOP_MARKER in following_end.knot_roles
        ):
            return True
    return section.section_index == len(sections) - 1


def _curvature_to_point(current: Pose2D, target: Pose2D) -> float:
    dx = target.x - current.x
    dy = target.y - current.y
    distance_sq = dx * dx + dy * dy
    if distance_sq <= _TOLERANCE:
        return 0.0
    lateral_local = -sin(current.yaw) * dx + cos(current.yaw) * dy
    return 2.0 * lateral_local / distance_sq


def _rate_limited_velocity(
    current: float,
    target: float,
    *,
    acceleration_step: float,
    deceleration_step: float,
) -> float:
    if current * target < 0.0:
        if abs(current) <= deceleration_step:
            return 0.0
        return current - (deceleration_step if current > 0.0 else -deceleration_step)
    maximum_change = deceleration_step if abs(target) < abs(current) else acceleration_step
    difference = target - current
    if abs(difference) <= maximum_change:
        return target
    return current + (maximum_change if difference > 0.0 else -maximum_change)


def _post_apply_constant_rollout(
    current_pose: Pose2D,
    current_twist: Twist2D,
    command: Twist2D,
    config: PersistentRppConfig,
) -> tuple[TrajectoryPoint, ...]:
    pose = _integrate_pose(current_pose, current_twist, config.control_period_s)
    points = [TrajectoryPoint(0.0, pose, command)]
    steps = int(round(config.rollout_horizon_s / config.rollout_step_s))
    for step in range(1, steps + 1):
        pose = _integrate_pose(pose, command, config.rollout_step_s)
        points.append(TrajectoryPoint(step * config.rollout_step_s, pose, command))
    return tuple(points)


def _post_apply_bounded_stop_rollout(
    current_pose: Pose2D,
    current_twist: Twist2D,
    command: Twist2D,
    *,
    linear_deceleration_mps2: float,
    config: PersistentRppConfig,
) -> tuple[TrajectoryPoint, ...]:
    """Project one accepted command interval, then a bounded stop and hold.

    Translation sections with an explicit upcoming stop must not claim that the
    current command will continue through that stop for the full two-second
    safety horizon.  The external gate still checks the current apply interval,
    this conservative executable fallback, and its own terminal-stop tail.
    """

    if not isfinite(linear_deceleration_mps2) or linear_deceleration_mps2 <= 0.0:
        raise ValueError("linear deceleration must be finite and positive")
    pose = _integrate_pose(current_pose, current_twist, config.control_period_s)
    twist = command
    points = [TrajectoryPoint(0.0, pose, twist)]
    steps = int(round(config.rollout_horizon_s / config.rollout_step_s))
    for step in range(1, steps + 1):
        pose = _integrate_pose(pose, twist, config.rollout_step_s)
        twist = Twist2D(
            _move_toward_zero(
                twist.linear,
                linear_deceleration_mps2 * config.rollout_step_s,
            ),
            _move_toward_zero(
                twist.angular,
                config.angular_deceleration_radps2 * config.rollout_step_s,
            ),
        )
        points.append(TrajectoryPoint(step * config.rollout_step_s, pose, twist))
    return tuple(points)


def _move_toward_zero(value: float, maximum_delta: float) -> float:
    if abs(value) <= maximum_delta:
        return 0.0
    return value - (maximum_delta if value > 0.0 else -maximum_delta)


def _integrate_pose(pose: Pose2D, twist: Twist2D, duration_s: float) -> Pose2D:
    if abs(twist.angular) <= _TOLERANCE:
        return Pose2D(
            pose.x + twist.linear * cos(pose.yaw) * duration_s,
            pose.y + twist.linear * sin(pose.yaw) * duration_s,
            pose.yaw,
        )
    next_yaw = pose.yaw + twist.angular * duration_s
    radius = twist.linear / twist.angular
    return Pose2D(
        pose.x + radius * (sin(next_yaw) - sin(pose.yaw)),
        pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
        atan2(sin(next_yaw), cos(next_yaw)),
    )


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


__all__ = [
    "PERSISTENT_RPP_CONTROLLER_NAME",
    "PERSISTENT_RPP_CONTROLLER_VERSION",
    "PersistentRppConfig",
    "PersistentRppController",
]
