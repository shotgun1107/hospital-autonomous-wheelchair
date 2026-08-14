"""R5 RPP·DWB가 공유하는 immutable reference section executor.

Translation section은 선택된 controller에 위임하고, planned stop·제자리회전·terminal
stop/dwell·HOLD만 동일한 상태기계로 처리한다. 이 모듈은 shared safety gate를 대신하지
않으며 실제 이동 허가를 생성하지 않는 Python ``simulation_only`` 연구 구성요소다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from math import atan2, ceil, copysign, cos, hypot, isclose, isfinite, sin, sqrt
from re import fullmatch

from hospital_path_lab.contracts import Pose2D, Twist2D
from hospital_path_lab.dynamic_contracts import DynamicMotionState
from hospital_path_lab.local_reference_contracts import (
    R4_COMPARISON_TOLERANCE,
    LocalManeuverReference,
    ReferenceSection,
    ReferenceSectionKind,
    ReferenceTravelDirection,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.persistent_controller_contracts import (
    PersistentControllerSessionTransition,
    PersistentControllerTickInput,
    PersistentReferenceBinding,
    PersistentReferenceSessionGuard,
    ReferenceExecutorState,
)

REFERENCE_SECTION_EXECUTOR_VERSION = "reference-section-executor-v2"
REFERENCE_SECTION_EXECUTION_DECISION_SCHEMA_VERSION = (
    "reference-section-execution-decision-v1"
)

R5_CONTROL_PERIOD_S = 0.05
R5_POSITION_TOLERANCE_M = 0.05
R5_YAW_TOLERANCE_RAD = 0.08
R5_STOPPED_LINEAR_VELOCITY_MPS = 0.01
R5_STOPPED_ANGULAR_VELOCITY_RADPS = 0.02
R5_STOPPED_CONFIRMATION_TICKS = 3
R5_TERMINAL_DWELL_S = 0.50
R5_LINEAR_DECELERATION_MPS2 = 0.50
R5_ANGULAR_ACCELERATION_RADPS2 = 1.60
R5_ANGULAR_DECELERATION_RADPS2 = 1.60
R5_MAXIMUM_ANGULAR_SPEED_RADPS = 0.80

_TOLERANCE = 1e-12


class ReferenceExecutorAction(StrEnum):
    DELEGATE_TRANSLATION = "delegate_translation"
    APPLY_COMMON_COMMAND = "apply_common_command"
    PRESERVE_DURING_GATE_STOP = "preserve_during_gate_stop"
    REQUEST_PROTECTIVE_HOLD = "request_protective_hold"
    MISSION_COMPLETED = "mission_completed"


@dataclass(frozen=True, slots=True)
class ReferenceSectionExecutorConfig:
    control_period_s: float = R5_CONTROL_PERIOD_S
    position_tolerance_m: float = R5_POSITION_TOLERANCE_M
    yaw_tolerance_rad: float = R5_YAW_TOLERANCE_RAD
    stopped_linear_velocity_mps: float = R5_STOPPED_LINEAR_VELOCITY_MPS
    stopped_angular_velocity_radps: float = R5_STOPPED_ANGULAR_VELOCITY_RADPS
    stopped_confirmation_ticks: int = R5_STOPPED_CONFIRMATION_TICKS
    terminal_dwell_s: float = R5_TERMINAL_DWELL_S
    linear_deceleration_mps2: float = R5_LINEAR_DECELERATION_MPS2
    angular_acceleration_radps2: float = R5_ANGULAR_ACCELERATION_RADPS2
    angular_deceleration_radps2: float = R5_ANGULAR_DECELERATION_RADPS2
    maximum_angular_speed_radps: float = R5_MAXIMUM_ANGULAR_SPEED_RADPS

    def __post_init__(self) -> None:
        expected = ReferenceSectionExecutorConfig.__dataclass_fields__
        for name, field in expected.items():
            default = field.default
            value = getattr(self, name)
            if name == "stopped_confirmation_ticks":
                _require_exact_positive_int(value, name)
                if value != default:
                    raise ValueError(f"{name} is frozen for R5 v1")
                continue
            _require_finite_positive(value, name)
            if not isclose(value, default, rel_tol=0.0, abs_tol=_TOLERANCE):
                raise ValueError(f"{name} is frozen for R5 v1")
        dwell_ticks = self.terminal_dwell_s / self.control_period_s
        if not isclose(dwell_ticks, round(dwell_ticks), rel_tol=0.0, abs_tol=_TOLERANCE):
            raise ValueError("terminal dwell must contain an exact number of control ticks")

    @property
    def terminal_dwell_ticks(self) -> int:
        return ceil(self.terminal_dwell_s / self.control_period_s)


@dataclass(frozen=True, slots=True)
class ReferenceSectionExecutionDecision:
    schema_version: str
    executor_version: str
    source_controller_tick: int
    tick_input_content_hash: str
    reference_binding_echo: PersistentReferenceBinding
    session_transition: PersistentControllerSessionTransition
    executor_state: ReferenceExecutorState
    action: ReferenceExecutorAction
    active_section_index: int | None
    active_section_kind: ReferenceSectionKind | None
    common_command: Twist2D | None
    target_pose: Pose2D | None
    position_error_m: float | None
    yaw_error_rad: float | None
    stopped_confirmation_ticks: int
    terminal_dwell_ticks: int
    session_reset_count: int
    window_update_count: int
    planned_section_stop: bool
    controller_requested_protective_stop: bool
    completed: bool
    failure_reason: str | None
    decision_trace: tuple[str, ...]
    semantic_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != REFERENCE_SECTION_EXECUTION_DECISION_SCHEMA_VERSION:
            raise ValueError("unsupported reference section execution decision schema")
        if self.executor_version != REFERENCE_SECTION_EXECUTOR_VERSION:
            raise ValueError("unsupported reference section executor version")
        _require_exact_nonnegative_int(
            self.source_controller_tick,
            "source_controller_tick",
        )
        _require_sha256(self.tick_input_content_hash, "tick_input_content_hash")
        if not isinstance(self.reference_binding_echo, PersistentReferenceBinding):
            raise TypeError("reference_binding_echo must be a PersistentReferenceBinding")
        if self.reference_binding_echo.source_window_control_tick != self.source_controller_tick:
            raise ValueError("decision tick must match its reference delivery")
        if not isinstance(self.session_transition, PersistentControllerSessionTransition):
            raise TypeError("session_transition has an unsupported type")
        if not isinstance(self.executor_state, ReferenceExecutorState):
            raise TypeError("executor_state has an unsupported type")
        if not isinstance(self.action, ReferenceExecutorAction):
            raise TypeError("action has an unsupported type")
        if self.active_section_index is not None:
            _require_exact_nonnegative_int(self.active_section_index, "active_section_index")
        if self.active_section_kind is not None and not isinstance(
            self.active_section_kind,
            ReferenceSectionKind,
        ):
            raise TypeError("active_section_kind has an unsupported type")
        if (self.active_section_index is None) != (self.active_section_kind is None):
            raise ValueError("active section index and kind must be present together")
        if self.common_command is not None:
            _validate_twist(self.common_command, "common_command")
        if self.target_pose is not None:
            _validate_pose(self.target_pose, "target_pose")
        for name in ("position_error_m",):
            value = getattr(self, name)
            if value is not None:
                _require_finite_nonnegative(value, name)
        if self.yaw_error_rad is not None and not isfinite(self.yaw_error_rad):
            raise ValueError("yaw_error_rad must be finite")
        for name in (
            "stopped_confirmation_ticks",
            "terminal_dwell_ticks",
            "session_reset_count",
            "window_update_count",
        ):
            _require_exact_nonnegative_int(getattr(self, name), name)
        for name in (
            "planned_section_stop",
            "controller_requested_protective_stop",
            "completed",
        ):
            if not isinstance(getattr(self, name), bool):
                raise TypeError(f"{name} must be a bool")
        trace = tuple(self.decision_trace)
        if any(not isinstance(item, str) or not item for item in trace):
            raise ValueError("decision_trace must contain non-empty strings")
        object.__setattr__(self, "decision_trace", trace)
        self._validate_action_semantics()
        _bind_or_check_hash(self, "semantic_content_hash", self.expected_semantic_hash)

    def _validate_action_semantics(self) -> None:
        if self.action is ReferenceExecutorAction.DELEGATE_TRANSLATION:
            if self.common_command is not None:
                raise ValueError("translation delegation cannot carry a common command")
        elif self.common_command is None:
            raise ValueError("non-delegated executor action requires a common command")
        if self.action is ReferenceExecutorAction.REQUEST_PROTECTIVE_HOLD:
            if not self.controller_requested_protective_stop or self.failure_reason is None:
                raise ValueError("protective hold requires its reason and stop request")
            if self.common_command != Twist2D():
                raise ValueError("protective hold command must be zero")
        elif self.controller_requested_protective_stop:
            raise ValueError("only protective hold may request a protective stop")
        if self.planned_section_stop and self.action is not (
            ReferenceExecutorAction.APPLY_COMMON_COMMAND
        ):
            raise ValueError("planned stop must be a common command")
        if self.completed != (self.action is ReferenceExecutorAction.MISSION_COMPLETED):
            raise ValueError("completed flag must match MISSION_COMPLETED")
        if self.completed and self.common_command != Twist2D():
            raise ValueError("completed decision command must be zero")
        if self.failure_reason is not None and self.action is not (
            ReferenceExecutorAction.REQUEST_PROTECTIVE_HOLD
        ):
            raise ValueError("only protective hold may carry a failure reason")

    @property
    def expected_semantic_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "executor_version": self.executor_version,
                "source_controller_tick": self.source_controller_tick,
                "tick_input_content_hash": self.tick_input_content_hash,
                "reference_binding_echo": self.reference_binding_echo,
                "session_transition": self.session_transition,
                "executor_state": self.executor_state,
                "action": self.action,
                "active_section_index": self.active_section_index,
                "active_section_kind": self.active_section_kind,
                "common_command": self.common_command,
                "target_pose": self.target_pose,
                "position_error_m": self.position_error_m,
                "yaw_error_rad": self.yaw_error_rad,
                "stopped_confirmation_ticks": self.stopped_confirmation_ticks,
                "terminal_dwell_ticks": self.terminal_dwell_ticks,
                "session_reset_count": self.session_reset_count,
                "window_update_count": self.window_update_count,
                "planned_section_stop": self.planned_section_stop,
                "controller_requested_protective_stop": (
                    self.controller_requested_protective_stop
                ),
                "completed": self.completed,
                "failure_reason": self.failure_reason,
                "decision_trace": self.decision_trace,
            }
        )


class ReferenceSectionExecutor:
    """R5 v1의 persistent, one-advance-per-tick common executor."""

    def __init__(self, config: ReferenceSectionExecutorConfig | None = None) -> None:
        self.config = config or ReferenceSectionExecutorConfig()
        self._guard = PersistentReferenceSessionGuard()
        self._reference: LocalManeuverReference | None = None
        self._active_section_index: int | None = None
        self._state = ReferenceExecutorState.UNBOUND
        self._pending_after_stop_index: int | None = None
        self._stopped_confirmation_ticks = 0
        self._terminal_dwell_ticks = 0
        self._completion_armed_tick: int | None = None
        self._session_reset_count = 0
        self._window_update_count = 0
        self._last_processed_tick: int | None = None
        self._last_input_hash: str | None = None
        self._last_decision: ReferenceSectionExecutionDecision | None = None

    @property
    def state(self) -> ReferenceExecutorState:
        return self._state

    @property
    def active_section_index(self) -> int | None:
        return self._active_section_index

    @property
    def session_reset_count(self) -> int:
        return self._session_reset_count

    @property
    def window_update_count(self) -> int:
        return self._window_update_count

    def step(
        self,
        tick_input: PersistentControllerTickInput,
    ) -> ReferenceSectionExecutionDecision:
        if not isinstance(tick_input, PersistentControllerTickInput):
            raise TypeError("tick_input must be a PersistentControllerTickInput")
        if self._last_processed_tick is not None:
            if tick_input.controller_tick == self._last_processed_tick and (
                tick_input.tick_input_content_hash == self._last_input_hash
            ):
                if self._last_decision is None:  # pragma: no cover - internal invariant
                    raise RuntimeError("cached executor tick has no decision")
                return self._last_decision
            if tick_input.controller_tick > self._last_processed_tick + 1:
                return self._rejected_input_decision(tick_input, "controller_tick_gap")

        acceptance = self._guard.evaluate(tick_input)
        if not acceptance.accepted:
            return self._rejected_input_decision(tick_input, acceptance.reason_code)
        if acceptance.duplicate_tick:
            if self._last_decision is None:  # pragma: no cover - internal invariant
                raise RuntimeError("guard duplicate has no cached executor decision")
            return self._last_decision

        if acceptance.state_reset_required:
            self._reset_for_reference(tick_input.full_reference)
        elif acceptance.transition is PersistentControllerSessionTransition.WINDOW_ADVANCED:
            self._window_update_count += 1

        if not self._active_section_is_in_window(tick_input):
            decision = self._invalidate_execution(
                tick_input,
                acceptance.transition,
                "active_section_not_in_current_window",
            )
        elif tick_input.current_gate_motion_state in {
            DynamicMotionState.BRAKING,
            DynamicMotionState.HOLDING,
        }:
            decision = self._decision(
                tick_input,
                acceptance.transition,
                action=ReferenceExecutorAction.PRESERVE_DURING_GATE_STOP,
                common_command=Twist2D(),
                trace=("shared_gate_stop_preserves_executor_state",),
            )
        else:
            decision = self._advance(tick_input, acceptance.transition)
        self._last_processed_tick = tick_input.controller_tick
        self._last_input_hash = tick_input.tick_input_content_hash
        self._last_decision = decision
        return decision

    def _reset_for_reference(self, reference: LocalManeuverReference) -> None:
        self._reference = reference
        self._active_section_index = 0
        first = reference.sections[0]
        if first.section_kind is ReferenceSectionKind.HOLD:
            self._state = ReferenceExecutorState.HOLD_REQUESTED
        elif first.section_kind is ReferenceSectionKind.ROTATE:
            self._state = ReferenceExecutorState.APPROACH_PLANNED_STOP
            self._pending_after_stop_index = 0
        else:
            self._state = ReferenceExecutorState.TRACK_TRANSLATION
            self._pending_after_stop_index = None
        self._stopped_confirmation_ticks = 0
        self._terminal_dwell_ticks = 0
        self._completion_armed_tick = None
        self._window_update_count = 0
        self._session_reset_count += 1

    def _advance(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
    ) -> ReferenceSectionExecutionDecision:
        if self._state is ReferenceExecutorState.INVALIDATED:
            return self._protective_decision(
                tick_input,
                transition,
                "executor_session_invalidated",
            )
        if self._state is ReferenceExecutorState.COMPLETED:
            return self._decision(
                tick_input,
                transition,
                action=ReferenceExecutorAction.MISSION_COMPLETED,
                common_command=Twist2D(),
                completed=True,
                trace=("terminal_completion_reported",),
            )
        if self._state is ReferenceExecutorState.HOLD_REQUESTED:
            return self._protective_decision(
                tick_input,
                transition,
                "hold_section_requires_new_authorized_reference",
            )
        if self._state is ReferenceExecutorState.TRACK_TRANSLATION:
            return self._advance_translation(tick_input, transition)
        if self._state is ReferenceExecutorState.APPROACH_PLANNED_STOP:
            return self._advance_planned_stop_approach(tick_input, transition)
        if self._state is ReferenceExecutorState.CONFIRM_PLANNED_STOP:
            return self._advance_planned_stop_confirmation(tick_input, transition)
        if self._state is ReferenceExecutorState.ROTATE_IN_PLACE:
            return self._advance_rotation(tick_input, transition)
        if self._state is ReferenceExecutorState.CONFIRM_ROTATION_STOP:
            return self._advance_rotation_confirmation(tick_input, transition)
        if self._state is ReferenceExecutorState.TERMINAL_STOP:
            return self._advance_terminal_stop(tick_input, transition)
        if self._state is ReferenceExecutorState.TERMINAL_DWELL:
            return self._advance_terminal_dwell(tick_input, transition)
        raise RuntimeError(f"unhandled executor state: {self._state}")

    def _advance_translation(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
    ) -> ReferenceSectionExecutionDecision:
        assert self._reference is not None and self._active_section_index is not None
        for _ in range(len(self._reference.sections) + 1):
            section = self._active_section()
            if section.section_kind is ReferenceSectionKind.HOLD:
                self._state = ReferenceExecutorState.HOLD_REQUESTED
                return self._protective_decision(
                    tick_input,
                    transition,
                    "hold_section_requires_new_authorized_reference",
                )
            if section.section_kind is ReferenceSectionKind.ROTATE:
                self._pending_after_stop_index = section.section_index
                self._state = ReferenceExecutorState.APPROACH_PLANNED_STOP
                return self._advance_planned_stop_approach(tick_input, transition)

            direction = section.travel_direction
            if direction is ReferenceTravelDirection.NONE:
                connector_start = self._section_start_pose(section)
                target = self._section_end_pose(section)
                position_error, yaw_error = _pose_errors(
                    tick_input.robot_state.pose,
                    target,
                )
                connector_displacement = hypot(
                    target.x - connector_start.x,
                    target.y - connector_start.y,
                )
                if (
                    connector_displacement > R4_COMPARISON_TOLERANCE
                    and not (
                        section.entry_requires_stopped
                        and section.exit_requires_stopped
                    )
                ):
                    return self._invalidate_execution(
                        tick_input,
                        transition,
                        "non_command_connector_requires_stopped_boundaries",
                    )
                if position_error > self.config.position_tolerance_m + _TOLERANCE:
                    return self._invalidate_execution(
                        tick_input,
                        transition,
                        "non_command_connector_pose_unreachable",
                    )
            elif direction not in {
                ReferenceTravelDirection.FORWARD,
                ReferenceTravelDirection.REVERSE,
            }:
                return self._invalidate_execution(
                    tick_input,
                    transition,
                    "translation_section_direction_invalid",
                )

            target = self._section_end_pose(section)
            position_error, yaw_error = _pose_errors(tick_input.robot_state.pose, target)
            terminal = section.section_index == len(self._reference.sections) - 1
            at_position = position_error <= self.config.position_tolerance_m + _TOLERANCE
            at_terminal_yaw = abs(yaw_error) <= self.config.yaw_tolerance_rad + _TOLERANCE
            if direction is ReferenceTravelDirection.NONE and not _actually_stopped(
                tick_input.robot_state.twist,
                self.config,
            ):
                self._pending_after_stop_index = (
                    section.section_index
                    if section.section_index == len(self._reference.sections) - 1
                    else section.section_index + 1
                )
                self._state = ReferenceExecutorState.APPROACH_PLANNED_STOP
                self._stopped_confirmation_ticks = 0
                return self._advance_planned_stop_approach(tick_input, transition)
            if not at_position or (terminal and not at_terminal_yaw):
                if direction is ReferenceTravelDirection.NONE:
                    return self._invalidate_execution(
                        tick_input,
                        transition,
                        "non_command_connector_cannot_translate",
                    )
                return self._decision(
                    tick_input,
                    transition,
                    action=ReferenceExecutorAction.DELEGATE_TRANSLATION,
                    common_command=None,
                    target_pose=target,
                    position_error_m=position_error,
                    yaw_error_rad=yaw_error,
                    trace=("translation_section_delegated",),
                )
            if terminal:
                self._state = ReferenceExecutorState.TERMINAL_STOP
                self._stopped_confirmation_ticks = 0
                return self._advance_terminal_stop(tick_input, transition)

            next_index = section.section_index + 1
            next_section = self._reference.sections[next_index]
            if next_section.section_kind is ReferenceSectionKind.HOLD:
                self._active_section_index = next_index
                self._state = ReferenceExecutorState.HOLD_REQUESTED
                return self._protective_decision(
                    tick_input,
                    transition,
                    "hold_section_requires_new_authorized_reference",
                )
            if (
                section.exit_requires_stopped
                or next_section.section_kind is ReferenceSectionKind.ROTATE
                or _signed_direction_changes(
                    self._reference.sections,
                    section.section_index,
                )
            ):
                self._pending_after_stop_index = next_index
                self._state = ReferenceExecutorState.APPROACH_PLANNED_STOP
                self._stopped_confirmation_ticks = 0
                return self._advance_planned_stop_approach(tick_input, transition)
            self._active_section_index = next_index
        return self._invalidate_execution(
            tick_input,
            transition,
            "section_advance_loop_exhausted",
        )

    def _advance_planned_stop_approach(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
    ) -> ReferenceSectionExecutionDecision:
        if _actually_stopped(tick_input.robot_state.twist, self.config):
            self._state = ReferenceExecutorState.CONFIRM_PLANNED_STOP
            self._stopped_confirmation_ticks = 1
        else:
            self._stopped_confirmation_ticks = 0
        return self._decision(
            tick_input,
            transition,
            action=ReferenceExecutorAction.APPLY_COMMON_COMMAND,
            common_command=_bounded_stop_command(tick_input.robot_state.twist, self.config),
            planned_section_stop=True,
            trace=("planned_stop_approach",),
        )

    def _advance_planned_stop_confirmation(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
    ) -> ReferenceSectionExecutionDecision:
        if not _actually_stopped(tick_input.robot_state.twist, self.config):
            self._state = ReferenceExecutorState.APPROACH_PLANNED_STOP
            self._stopped_confirmation_ticks = 0
            return self._advance_planned_stop_approach(tick_input, transition)
        self._stopped_confirmation_ticks += 1
        trace = ("planned_stop_confirmation",)
        if self._stopped_confirmation_ticks >= self.config.stopped_confirmation_ticks:
            self._finish_planned_stop()
            trace = ("planned_stop_confirmed",)
        return self._decision(
            tick_input,
            transition,
            action=ReferenceExecutorAction.APPLY_COMMON_COMMAND,
            common_command=Twist2D(),
            planned_section_stop=True,
            trace=trace,
        )

    def _finish_planned_stop(self) -> None:
        if self._reference is None or self._pending_after_stop_index is None:
            self._state = ReferenceExecutorState.INVALIDATED
            return
        self._active_section_index = self._pending_after_stop_index
        self._pending_after_stop_index = None
        self._stopped_confirmation_ticks = 0
        section = self._active_section()
        if section.section_kind is ReferenceSectionKind.ROTATE:
            self._state = ReferenceExecutorState.ROTATE_IN_PLACE
        elif section.section_kind is ReferenceSectionKind.HOLD:
            self._state = ReferenceExecutorState.HOLD_REQUESTED
        else:
            self._state = ReferenceExecutorState.TRACK_TRANSLATION

    def _advance_rotation(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
    ) -> ReferenceSectionExecutionDecision:
        section = self._active_section()
        if section.section_kind is not ReferenceSectionKind.ROTATE:
            return self._invalidate_execution(
                tick_input,
                transition,
                "rotation_state_requires_rotation_section",
            )
        entry = self._section_start_pose(section)
        target = self._section_end_pose(section)
        assert self._reference is not None
        section_knots = self._reference.knots[
            section.first_knot_index : section.last_knot_index + 1
        ]
        if any(
            hypot(knot.pose.x - entry.x, knot.pose.y - entry.y)
            > R4_COMPARISON_TOLERANCE
            for knot in section_knots[1:]
        ):
            return self._invalidate_execution(
                tick_input,
                transition,
                "rotation_section_is_not_position_atomic",
            )
        position_error, yaw_error = _pose_errors(tick_input.robot_state.pose, target)
        entry_error = hypot(
            tick_input.robot_state.pose.x - entry.x,
            tick_input.robot_state.pose.y - entry.y,
        )
        if entry_error > self.config.position_tolerance_m + _TOLERANCE:
            return self._invalidate_execution(
                tick_input,
                transition,
                "rotation_position_tolerance_exceeded",
            )
        if abs(tick_input.robot_state.twist.linear) > (
            self.config.stopped_linear_velocity_mps + _TOLERANCE
        ):
            return self._decision(
                tick_input,
                transition,
                action=ReferenceExecutorAction.APPLY_COMMON_COMMAND,
                common_command=_bounded_stop_command(
                    tick_input.robot_state.twist,
                    self.config,
                ),
                target_pose=target,
                position_error_m=position_error,
                yaw_error_rad=yaw_error,
                planned_section_stop=True,
                trace=("rotation_waiting_for_linear_stop",),
            )
        if abs(yaw_error) <= self.config.yaw_tolerance_rad + _TOLERANCE:
            self._state = ReferenceExecutorState.CONFIRM_ROTATION_STOP
            self._stopped_confirmation_ticks = (
                1 if _actually_stopped(tick_input.robot_state.twist, self.config) else 0
            )
            return self._decision(
                tick_input,
                transition,
                action=ReferenceExecutorAction.APPLY_COMMON_COMMAND,
                common_command=_bounded_stop_command(
                    tick_input.robot_state.twist,
                    self.config,
                ),
                target_pose=target,
                position_error_m=position_error,
                yaw_error_rad=yaw_error,
                planned_section_stop=True,
                trace=("rotation_stop_confirmation_started",),
            )
        return self._decision(
            tick_input,
            transition,
            action=ReferenceExecutorAction.APPLY_COMMON_COMMAND,
            common_command=Twist2D(
                0.0,
                _bounded_rotation_command(
                    tick_input.robot_state.twist.angular,
                    yaw_error,
                    self.config,
                ),
            ),
            target_pose=target,
            position_error_m=position_error,
            yaw_error_rad=yaw_error,
            trace=("shortest_direction_rotation",),
        )

    def _advance_rotation_confirmation(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
    ) -> ReferenceSectionExecutionDecision:
        section = self._active_section()
        target = self._section_end_pose(section)
        position_error, yaw_error = _pose_errors(tick_input.robot_state.pose, target)
        if position_error > self.config.position_tolerance_m + _TOLERANCE:
            return self._invalidate_execution(
                tick_input,
                transition,
                "rotation_position_tolerance_exceeded",
            )
        if abs(yaw_error) > self.config.yaw_tolerance_rad + _TOLERANCE:
            self._state = ReferenceExecutorState.ROTATE_IN_PLACE
            self._stopped_confirmation_ticks = 0
            return self._advance_rotation(tick_input, transition)
        if not _actually_stopped(tick_input.robot_state.twist, self.config):
            self._stopped_confirmation_ticks = 0
            return self._decision(
                tick_input,
                transition,
                action=ReferenceExecutorAction.APPLY_COMMON_COMMAND,
                common_command=_bounded_stop_command(
                    tick_input.robot_state.twist,
                    self.config,
                ),
                target_pose=target,
                position_error_m=position_error,
                yaw_error_rad=yaw_error,
                planned_section_stop=True,
                trace=("rotation_stop_interrupted",),
            )
        self._stopped_confirmation_ticks += 1
        trace = ("rotation_stop_confirmation",)
        if self._stopped_confirmation_ticks >= self.config.stopped_confirmation_ticks:
            self._advance_after_rotation()
            trace = ("rotation_section_completed",)
        return self._decision(
            tick_input,
            transition,
            action=ReferenceExecutorAction.APPLY_COMMON_COMMAND,
            common_command=Twist2D(),
            planned_section_stop=True,
            trace=trace,
        )

    def _advance_after_rotation(self) -> None:
        assert self._reference is not None and self._active_section_index is not None
        next_index = self._active_section_index + 1
        self._stopped_confirmation_ticks = 0
        if next_index >= len(self._reference.sections):
            self._state = ReferenceExecutorState.TERMINAL_STOP
            return
        self._active_section_index = next_index
        next_section = self._active_section()
        if next_section.section_kind is ReferenceSectionKind.HOLD:
            self._state = ReferenceExecutorState.HOLD_REQUESTED
        elif next_section.section_kind is ReferenceSectionKind.ROTATE:
            self._pending_after_stop_index = next_index
            self._state = ReferenceExecutorState.APPROACH_PLANNED_STOP
        else:
            self._state = ReferenceExecutorState.TRACK_TRANSLATION

    def _advance_terminal_stop(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
    ) -> ReferenceSectionExecutionDecision:
        target = self._terminal_pose()
        position_error, yaw_error = _pose_errors(tick_input.robot_state.pose, target)
        if position_error > self.config.position_tolerance_m + _TOLERANCE or (
            abs(yaw_error) > self.config.yaw_tolerance_rad + _TOLERANCE
        ):
            return self._invalidate_execution(
                tick_input,
                transition,
                "terminal_pose_tolerance_lost",
            )
        if _actually_stopped(tick_input.robot_state.twist, self.config):
            self._stopped_confirmation_ticks += 1
        else:
            self._stopped_confirmation_ticks = 0
        trace = ("terminal_stop_confirmation",)
        if self._stopped_confirmation_ticks >= self.config.stopped_confirmation_ticks:
            self._state = ReferenceExecutorState.TERMINAL_DWELL
            self._terminal_dwell_ticks = 0
            trace = ("terminal_stop_confirmed",)
        return self._decision(
            tick_input,
            transition,
            action=ReferenceExecutorAction.APPLY_COMMON_COMMAND,
            common_command=_bounded_stop_command(tick_input.robot_state.twist, self.config),
            target_pose=target,
            position_error_m=position_error,
            yaw_error_rad=yaw_error,
            planned_section_stop=True,
            trace=trace,
        )

    def _advance_terminal_dwell(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
    ) -> ReferenceSectionExecutionDecision:
        target = self._terminal_pose()
        position_error, yaw_error = _pose_errors(tick_input.robot_state.pose, target)
        if position_error > self.config.position_tolerance_m + _TOLERANCE or (
            abs(yaw_error) > self.config.yaw_tolerance_rad + _TOLERANCE
        ):
            return self._invalidate_execution(
                tick_input,
                transition,
                "terminal_dwell_pose_tolerance_lost",
            )
        if not _actually_stopped(tick_input.robot_state.twist, self.config):
            self._state = ReferenceExecutorState.TERMINAL_STOP
            self._stopped_confirmation_ticks = 0
            self._terminal_dwell_ticks = 0
            self._completion_armed_tick = None
            return self._advance_terminal_stop(tick_input, transition)
        if self._completion_armed_tick is not None:
            if tick_input.controller_tick > self._completion_armed_tick:
                self._state = ReferenceExecutorState.COMPLETED
                return self._decision(
                    tick_input,
                    transition,
                    action=ReferenceExecutorAction.MISSION_COMPLETED,
                    common_command=Twist2D(),
                    target_pose=target,
                    position_error_m=position_error,
                    yaw_error_rad=yaw_error,
                    completed=True,
                    trace=("terminal_completion_reported",),
                )
        else:
            self._terminal_dwell_ticks += 1
            if self._terminal_dwell_ticks >= self.config.terminal_dwell_ticks:
                self._completion_armed_tick = tick_input.controller_tick
        return self._decision(
            tick_input,
            transition,
            action=ReferenceExecutorAction.APPLY_COMMON_COMMAND,
            common_command=Twist2D(),
            target_pose=target,
            position_error_m=position_error,
            yaw_error_rad=yaw_error,
            planned_section_stop=True,
            trace=("terminal_stopped_dwell",),
        )

    def _active_section(self) -> ReferenceSection:
        if self._reference is None or self._active_section_index is None:
            raise RuntimeError("executor has no active reference section")
        return self._reference.sections[self._active_section_index]

    def _section_start_pose(self, section: ReferenceSection) -> Pose2D:
        assert self._reference is not None
        return self._reference.knots[section.first_knot_index].pose

    def _section_end_pose(self, section: ReferenceSection) -> Pose2D:
        assert self._reference is not None
        return self._reference.knots[section.last_knot_index].pose

    def _terminal_pose(self) -> Pose2D:
        if self._reference is None:
            raise RuntimeError("executor has no terminal reference pose")
        return self._reference.knots[-1].pose

    def _active_section_is_in_window(
        self,
        tick_input: PersistentControllerTickInput,
    ) -> bool:
        if self._active_section_index is None:
            return False
        return any(
            section.section_index == self._active_section_index
            for section in tick_input.local_window.sections
        )

    def _invalidate_execution(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
        reason: str,
    ) -> ReferenceSectionExecutionDecision:
        self._state = ReferenceExecutorState.INVALIDATED
        return self._protective_decision(tick_input, transition, reason)

    def _rejected_input_decision(
        self,
        tick_input: PersistentControllerTickInput,
        reason: str,
    ) -> ReferenceSectionExecutionDecision:
        return self._decision(
            tick_input,
            PersistentControllerSessionTransition.INVALIDATED,
            action=ReferenceExecutorAction.REQUEST_PROTECTIVE_HOLD,
            common_command=Twist2D(),
            controller_requested_protective_stop=True,
            failure_reason=reason,
            state_override=ReferenceExecutorState.INVALIDATED,
            trace=(reason,),
        )

    def _protective_decision(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
        reason: str,
    ) -> ReferenceSectionExecutionDecision:
        return self._decision(
            tick_input,
            transition,
            action=ReferenceExecutorAction.REQUEST_PROTECTIVE_HOLD,
            common_command=Twist2D(),
            controller_requested_protective_stop=True,
            failure_reason=reason,
            trace=(reason,),
        )

    def _decision(
        self,
        tick_input: PersistentControllerTickInput,
        transition: PersistentControllerSessionTransition,
        *,
        action: ReferenceExecutorAction,
        common_command: Twist2D | None,
        target_pose: Pose2D | None = None,
        position_error_m: float | None = None,
        yaw_error_rad: float | None = None,
        planned_section_stop: bool = False,
        controller_requested_protective_stop: bool = False,
        completed: bool = False,
        failure_reason: str | None = None,
        state_override: ReferenceExecutorState | None = None,
        trace: tuple[str, ...],
    ) -> ReferenceSectionExecutionDecision:
        section = None
        if (
            self._reference is not None
            and self._active_section_index is not None
            and self._active_section_index < len(self._reference.sections)
        ):
            section = self._reference.sections[self._active_section_index]
        return ReferenceSectionExecutionDecision(
            schema_version=REFERENCE_SECTION_EXECUTION_DECISION_SCHEMA_VERSION,
            executor_version=REFERENCE_SECTION_EXECUTOR_VERSION,
            source_controller_tick=tick_input.controller_tick,
            tick_input_content_hash=tick_input.tick_input_content_hash,
            reference_binding_echo=tick_input.reference_binding,
            session_transition=transition,
            executor_state=self._state if state_override is None else state_override,
            action=action,
            active_section_index=None if section is None else section.section_index,
            active_section_kind=None if section is None else section.section_kind,
            common_command=common_command,
            target_pose=target_pose,
            position_error_m=position_error_m,
            yaw_error_rad=yaw_error_rad,
            stopped_confirmation_ticks=self._stopped_confirmation_ticks,
            terminal_dwell_ticks=self._terminal_dwell_ticks,
            session_reset_count=self._session_reset_count,
            window_update_count=self._window_update_count,
            planned_section_stop=planned_section_stop,
            controller_requested_protective_stop=(
                controller_requested_protective_stop
            ),
            completed=completed,
            failure_reason=failure_reason,
            decision_trace=trace,
        )


def shortest_angular_distance(current_yaw_rad: float, target_yaw_rad: float) -> float:
    if not isfinite(current_yaw_rad) or not isfinite(target_yaw_rad):
        raise ValueError("yaw values must be finite")
    difference = target_yaw_rad - current_yaw_rad
    return atan2(sin(difference), cos(difference))


def _pose_errors(current: Pose2D, target: Pose2D) -> tuple[float, float]:
    return (
        hypot(target.x - current.x, target.y - current.y),
        shortest_angular_distance(current.yaw, target.yaw),
    )


def _bounded_stop_command(
    actual: Twist2D,
    config: ReferenceSectionExecutorConfig,
) -> Twist2D:
    return Twist2D(
        _approach_zero(
            actual.linear,
            config.linear_deceleration_mps2 * config.control_period_s,
        ),
        _approach_zero(
            actual.angular,
            config.angular_deceleration_radps2 * config.control_period_s,
        ),
    )


def _bounded_rotation_command(
    actual_angular: float,
    yaw_error: float,
    config: ReferenceSectionExecutorConfig,
) -> float:
    distance_to_tolerance = max(0.0, abs(yaw_error) - config.yaw_tolerance_rad)
    stopping_limited_speed = sqrt(
        2.0 * config.angular_deceleration_radps2 * distance_to_tolerance
    )
    target_magnitude = min(config.maximum_angular_speed_radps, stopping_limited_speed)
    target = copysign(target_magnitude, yaw_error)
    return _approach_velocity(
        actual_angular,
        target,
        acceleration_step=config.angular_acceleration_radps2 * config.control_period_s,
        deceleration_step=config.angular_deceleration_radps2 * config.control_period_s,
    )


def _actually_stopped(
    twist: Twist2D,
    config: ReferenceSectionExecutorConfig,
) -> bool:
    return abs(twist.linear) <= config.stopped_linear_velocity_mps and (
        abs(twist.angular) <= config.stopped_angular_velocity_radps
    )


def _signed_direction_changes(
    sections: tuple[ReferenceSection, ...],
    current_index: int,
) -> bool:
    """Detect the next executable translation sign without geometric inference."""

    current = sections[current_index].travel_direction
    if current not in {
        ReferenceTravelDirection.FORWARD,
        ReferenceTravelDirection.REVERSE,
    }:
        return False
    for following in sections[current_index + 1 :]:
        if following.section_kind in {
            ReferenceSectionKind.ROTATE,
            ReferenceSectionKind.HOLD,
        }:
            return False
        if following.travel_direction is ReferenceTravelDirection.NONE:
            continue
        return following.travel_direction is not current
    return False


def _approach_zero(value: float, maximum_change: float) -> float:
    if abs(value) <= maximum_change:
        return 0.0
    return value - copysign(maximum_change, value)


def _approach_velocity(
    current: float,
    target: float,
    *,
    acceleration_step: float,
    deceleration_step: float,
) -> float:
    if current * target < 0.0:
        return _approach_zero(current, deceleration_step)
    maximum_change = deceleration_step if abs(target) < abs(current) else acceleration_step
    difference = target - current
    if abs(difference) <= maximum_change:
        return target
    return current + copysign(maximum_change, difference)


def _validate_pose(pose: Pose2D, name: str) -> None:
    if not isinstance(pose, Pose2D):
        raise TypeError(f"{name} must be a Pose2D")
    if not all(isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
        raise ValueError(f"{name} must contain finite values")


def _validate_twist(twist: Twist2D, name: str) -> None:
    if not isinstance(twist, Twist2D):
        raise TypeError(f"{name} must be a Twist2D")
    if not all(isfinite(value) for value in (twist.linear, twist.angular)):
        raise ValueError(f"{name} must contain finite values")


def _bind_or_check_hash(value: object, field_name: str, expected: str) -> None:
    current = getattr(value, field_name)
    if current:
        _require_sha256(current, field_name)
        if current != expected:
            raise ValueError(f"{field_name} mismatch")
    else:
        object.__setattr__(value, field_name, expected)


def _require_sha256(value: str, name: str) -> None:
    if not isinstance(value, str) or fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{name} must be a lowercase SHA-256")


def _require_exact_nonnegative_int(value: int, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative exact integer")


def _require_exact_positive_int(value: int, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive exact integer")


def _require_finite_nonnegative(value: float, name: str) -> None:
    if not isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")


def _require_finite_positive(value: float, name: str) -> None:
    if not isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")
