"""R5 persistent-reference adapter for the source-derived DWB research core.

The existing dynamic DWB adapter treats every changed path as a new controller
path and therefore resets all stateful critics.  R5 sliding windows have a
different lifetime: the immutable full reference owns the controller session,
while the moving local window only updates the four map-grid scoring critics.

This module keeps that split explicit.  It is a Python simulation-research
adapter, not a ROS plugin, product controller, or real-time qualification.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from hashlib import sha256
from json import dumps
from math import cos, hypot, pi, sin
from time import perf_counter_ns

from hospital_path_lab.contracts import PlanStatus, Pose2D, TrajectoryPoint, Twist2D
from hospital_path_lab.dynamic_contracts import ControllerSnapshot, build_controller_snapshot
from hospital_path_lab.dynamic_safety import DynamicTrajectorySafetyEvidence
from hospital_path_lab.dynamic_trajectory_constraints import (
    ProjectDynamicSafetyConstraintCritic,
)
from hospital_path_lab.local_reference_contracts import ReferenceSectionKind
from hospital_path_lab.persistent_controller_contracts import (
    PERSISTENT_CONTROLLER_RESULT_SCHEMA_VERSION,
    PersistentControllerResult,
    PersistentControllerStatus,
    PersistentControllerTickInput,
)
from hospital_path_lab.reference_section_executor import (
    ReferenceExecutorAction,
    ReferenceSectionExecutionDecision,
    ReferenceSectionExecutor,
)

from .adapter import SourceDerivedDwbController
from .composition import (
    SourceDerivedDwbConfig,
    _critic_grid,
    _generator_config_for,
    _map_grid_scale,
    _static_geometry_signature,
    _validate_generator_profile,
)
from .contracts import DwbGeneratorRequest, DwbPose2D
from .core import DwbCoreResult, DwbCriticBinding, DwbReferenceCore
from .critics import (
    GoalAlignCritic,
    GoalDistCritic,
    OscillationCritic,
    PathAlignCritic,
    PathDistCritic,
    RotateToGoalCritic,
)
from .trajectory_generator import DwbReferenceTrajectoryGenerator

PERSISTENT_DWB_CONTROLLER_NAME = "persistent_dwb_reference"
PERSISTENT_DWB_ADAPTER_VERSION = "persistent-dwb-reference-v1"

_TOLERANCE = 1e-12
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


@dataclass(frozen=True, slots=True)
class PersistentDwbSessionDiagnostics:
    """Observable critic-lifetime state used by deterministic R5 evidence."""

    session_reset_count: int
    scoring_window_update_count: int
    full_terminal_goal: DwbPose2D | None
    scoring_path_hash: str | None
    oscillation_restrictions_active: bool


class PersistentDwbCoreSession:
    """Split immutable-session state from mutable local scoring-path state.

    ``SourceDerivedDwbController`` calls :meth:`set_path` whenever its project
    path signature changes.  Here that protocol method is deliberately only an
    alias for :meth:`update_scoring_window`; it never resets Oscillation or the
    full-terminal RotateToGoal binding.
    """

    _SCORING_CRITIC_NAMES = (
        "goal_align",
        "path_align",
        "path_dist",
        "goal_dist",
    )

    def __init__(
        self,
        core: DwbReferenceCore,
        *,
        scoring_critics: Sequence[object],
        rotate_to_goal_critic: RotateToGoalCritic,
        oscillation_critic: OscillationCritic,
    ) -> None:
        if len(tuple(scoring_critics)) != len(self._SCORING_CRITIC_NAMES):
            raise ValueError("persistent DWB requires exactly four scoring critics")
        self._core = core
        self._scoring_critics = tuple(scoring_critics)
        self._rotate_to_goal = rotate_to_goal_critic
        self._oscillation = oscillation_critic
        self._session_path: tuple[DwbPose2D, ...] | None = None
        self._scoring_path: tuple[DwbPose2D, ...] | None = None
        self._session_reset_count = 0
        self._scoring_window_update_count = 0

    @property
    def critic_names(self) -> tuple[str, ...]:
        return self._core.critic_names

    @property
    def diagnostics(self) -> PersistentDwbSessionDiagnostics:
        terminal = None if self._session_path is None else self._session_path[-1]
        return PersistentDwbSessionDiagnostics(
            session_reset_count=self._session_reset_count,
            scoring_window_update_count=self._scoring_window_update_count,
            full_terminal_goal=terminal,
            scoring_path_hash=(
                None if self._scoring_path is None else _dwb_path_hash(self._scoring_path)
            ),
            oscillation_restrictions_active=self._oscillation.has_restrictions,
        )

    def begin_reference_session(
        self,
        full_reference_path: Sequence[DwbPose2D],
        scoring_path: Sequence[DwbPose2D],
    ) -> None:
        """Reset all critics once and bind the immutable full terminal goal."""

        full = _freeze_dwb_path(full_reference_path, "full reference")
        local = _freeze_dwb_path(scoring_path, "scoring path")
        self._core.reset()
        self._rotate_to_goal.set_path(full)
        self._session_path = full
        self._scoring_path = None
        self._session_reset_count += 1
        self._scoring_window_update_count = 0
        self.update_scoring_window(local)

    def update_scoring_window(self, scoring_path: Sequence[DwbPose2D]) -> None:
        """Update only Path/Goal Dist and Align fields for one local window."""

        local = _freeze_dwb_path(scoring_path, "scoring path")
        if local == self._scoring_path:
            return
        if self._session_path is None:
            raise ValueError("begin_reference_session must precede scoring-window updates")
        for critic in self._scoring_critics:
            set_path = getattr(critic, "set_path", None)
            if set_path is None:  # pragma: no cover - construction invariant
                raise TypeError("scoring critic does not expose set_path")
            set_path(local)
        self._scoring_path = local
        self._scoring_window_update_count += 1

    def set_path(self, path: Sequence[DwbPose2D]) -> None:
        """Adapter protocol hook: a changed project path is a scoring update only."""

        self.update_scoring_window(path)

    def compute(self, request: DwbGeneratorRequest) -> DwbCoreResult:
        if self._session_path is None or self._scoring_path is None:
            raise ValueError("persistent DWB session is not initialized")
        return self._core.compute(request)


@dataclass(slots=True)
class _PersistentDwbStack:
    geometry_signature: str
    session_core: PersistentDwbCoreSession
    adapter: SourceDerivedDwbController
    safety_critic: ProjectDynamicSafetyConstraintCritic


class PersistentSourceDerivedDwbController:
    """Run source-derived DWB translation under the shared R5 section executor."""

    name = PERSISTENT_DWB_CONTROLLER_NAME

    def __init__(
        self,
        *,
        config: SourceDerivedDwbConfig | None = None,
        executor: ReferenceSectionExecutor | None = None,
    ) -> None:
        self._executor = executor or ReferenceSectionExecutor()
        self._config = config
        self._stack: _PersistentDwbStack | None = None
        self._stack_build_count = 0
        self._bound_reference_session_id: str | None = None
        self._bound_full_reference_hash: str | None = None
        self._last_tick: int | None = None
        self._last_input_hash: str | None = None
        self._last_result: PersistentControllerResult | None = None

    @property
    def session_reset_count(self) -> int:
        return self._executor.session_reset_count

    @property
    def window_update_count(self) -> int:
        return self._executor.window_update_count

    @property
    def stack_build_count(self) -> int:
        return self._stack_build_count

    @property
    def dwb_session_diagnostics(self) -> PersistentDwbSessionDiagnostics | None:
        return None if self._stack is None else self._stack.session_core.diagnostics

    @property
    def selected_safety_evidence(self) -> DynamicTrajectorySafetyEvidence | None:
        return None if self._stack is None else self._stack.safety_critic.selected_evidence

    def step(self, tick_input: PersistentControllerTickInput) -> PersistentControllerResult:
        started_at = perf_counter_ns()
        if not isinstance(tick_input, PersistentControllerTickInput):
            raise TypeError("tick_input must be a PersistentControllerTickInput")
        if (
            self._last_tick == tick_input.controller_tick
            and self._last_input_hash == tick_input.tick_input_content_hash
        ):
            if self._last_result is None:  # pragma: no cover - internal invariant
                raise RuntimeError("cached persistent DWB tick has no result")
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
            stack = self._ensure_stack(tick_input)
            scoring_path = _window_dwb_path(tick_input)
            if self._needs_session_bind(tick_input):
                stack.session_core.begin_reference_session(
                    _full_reference_dwb_path(tick_input),
                    scoring_path,
                )
                self._bound_reference_session_id = tick_input.reference_binding.reference_session_id
                self._bound_full_reference_hash = tick_input.full_reference.reference_content_hash
            else:
                stack.session_core.update_scoring_window(scoring_path)
            snapshot = _controller_snapshot(tick_input)
            inner = stack.adapter.step(snapshot)
        except (TypeError, ValueError) as error:
            return self._result(
                tick_input,
                decision,
                started_at,
                status=PersistentControllerStatus.SECTION_EXECUTION_FAILED,
                failure_reason=f"persistent_dwb_input_invalid:{type(error).__name__}",
                controller_requested_protective_stop=True,
                decision_trace=(str(error),),
            )

        diagnostics = _candidate_diagnostics(inner.decision_trace, stack.session_core.diagnostics)
        trace = (
            f"persistent_dwb_version={PERSISTENT_DWB_ADAPTER_VERSION}",
            "terminal_goal_source=immutable_full_reference",
            "local_window_endpoint_is_not_rotate_goal=true",
            *inner.decision_trace,
        )
        if inner.status is PlanStatus.FOUND:
            return self._result(
                tick_input,
                decision,
                started_at,
                status=PersistentControllerStatus.COMMAND_FOUND,
                requested_twist=inner.requested_twist,
                predicted_trajectory=inner.predicted_trajectory,
                tracking_error_m=_active_section_tracking_error(
                    tick_input,
                    decision.active_section_index,
                ),
                decision_trace=trace,
                candidate_diagnostics=diagnostics,
            )
        if inner.status is PlanStatus.NO_PATH and inner.no_safe_candidate:
            return self._result(
                tick_input,
                decision,
                started_at,
                status=PersistentControllerStatus.NO_SAFE_COMMAND,
                failure_reason=inner.failure_reason or "no_legal_dwb_trajectory",
                controller_requested_protective_stop=True,
                no_safe_candidate=True,
                decision_trace=trace,
                candidate_diagnostics=diagnostics,
            )
        return self._result(
            tick_input,
            decision,
            started_at,
            status=PersistentControllerStatus.SECTION_EXECUTION_FAILED,
            failure_reason=inner.failure_reason or "persistent_dwb_core_failed",
            controller_requested_protective_stop=True,
            decision_trace=trace,
            candidate_diagnostics=diagnostics,
        )

    def _ensure_stack(self, tick_input: PersistentControllerTickInput) -> _PersistentDwbStack:
        snapshot = _controller_snapshot(tick_input)
        signature = _static_geometry_signature(snapshot, tick_input.vehicle_profile)
        if self._stack is not None and self._stack.geometry_signature == signature:
            return self._stack

        profile = tick_input.vehicle_profile
        config = self._config or SourceDerivedDwbConfig(generator=_generator_config_for(profile))
        _validate_generator_profile(config.generator, profile)
        grid = _critic_grid(snapshot, profile)
        safety = ProjectDynamicSafetyConstraintCritic()
        rotate = RotateToGoalCritic(
            xy_goal_tolerance_m=0.05,
            path_length_tolerance_m=0.10,
            stopped_linear_velocity_mps=0.01,
        )
        oscillation = OscillationCritic()
        goal_align = GoalAlignCritic(
            grid,
            forward_point_distance_m=config.forward_point_distance_m,
        )
        path_align = PathAlignCritic(
            grid,
            forward_point_distance_m=config.forward_point_distance_m,
        )
        path_dist = PathDistCritic(grid)
        goal_dist = GoalDistCritic(grid)
        bindings = (
            DwbCriticBinding("project_safety", safety, config.safety_scale),
            DwbCriticBinding("rotate_to_goal", rotate, config.rotate_to_goal_scale),
            DwbCriticBinding("oscillation", oscillation, config.oscillation_scale),
            DwbCriticBinding(
                "goal_align",
                goal_align,
                _map_grid_scale(config.goal_align_scale, grid.resolution_m),
            ),
            DwbCriticBinding(
                "path_align",
                path_align,
                _map_grid_scale(config.path_align_scale, grid.resolution_m),
            ),
            DwbCriticBinding(
                "path_dist",
                path_dist,
                _map_grid_scale(config.path_dist_scale, grid.resolution_m),
            ),
            DwbCriticBinding(
                "goal_dist",
                goal_dist,
                _map_grid_scale(config.goal_dist_scale, grid.resolution_m),
            ),
        )
        generator = DwbReferenceTrajectoryGenerator(config.generator)
        core = DwbReferenceCore(generator, bindings)
        session_core = PersistentDwbCoreSession(
            core,
            scoring_critics=(goal_align, path_align, path_dist, goal_dist),
            rotate_to_goal_critic=rotate,
            oscillation_critic=oscillation,
        )
        adapter = SourceDerivedDwbController(
            profile,
            core=session_core,
            snapshot_binders=(safety,),
            generator_config=config.generator,
        )
        self._stack = _PersistentDwbStack(signature, session_core, adapter, safety)
        self._stack_build_count += 1
        self._bound_reference_session_id = None
        self._bound_full_reference_hash = None
        return self._stack

    def _needs_session_bind(self, tick_input: PersistentControllerTickInput) -> bool:
        return (
            self._bound_reference_session_id != tick_input.reference_binding.reference_session_id
            or self._bound_full_reference_hash != tick_input.full_reference.reference_content_hash
        )

    def _common_result(
        self,
        tick_input: PersistentControllerTickInput,
        decision: ReferenceSectionExecutionDecision,
        started_at: int,
    ) -> PersistentControllerResult:
        command = decision.common_command or Twist2D()
        trace = (f"persistent_dwb_version={PERSISTENT_DWB_ADAPTER_VERSION}",)
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
        trajectory = _post_apply_constant_rollout(tick_input, command)
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
        no_safe_candidate: bool = False,
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
            no_safe_candidate=no_safe_candidate,
            elapsed_nonqualification_ns=perf_counter_ns() - started_at,
        )


def _controller_snapshot(tick_input: PersistentControllerTickInput) -> ControllerSnapshot:
    return build_controller_snapshot(
        tick_id=tick_input.controller_tick,
        simulation_time_s=tick_input.simulation_time_s,
        mission_id=tick_input.full_reference.mission_id,
        robot_state=tick_input.robot_state,
        goal_pose=tick_input.full_reference.knots[-1].pose,
        reference_path=tuple(knot.pose for knot in tick_input.local_window.knots),
        static_grid_snapshot=tick_input.static_grid_snapshot,
        validated_observation=tick_input.validated_observation,
        actor_tubes=tick_input.actor_prediction_set,
        vehicle_profile=tick_input.vehicle_profile,
    )


def _full_reference_dwb_path(
    tick_input: PersistentControllerTickInput,
) -> tuple[DwbPose2D, ...]:
    return tuple(_dwb_pose(knot.pose) for knot in tick_input.full_reference.knots)


def _window_dwb_path(tick_input: PersistentControllerTickInput) -> tuple[DwbPose2D, ...]:
    return tuple(_dwb_pose(knot.pose) for knot in tick_input.local_window.knots)


def _freeze_dwb_path(path: Sequence[DwbPose2D], label: str) -> tuple[DwbPose2D, ...]:
    frozen = tuple(path)
    if len(frozen) < 2:
        raise ValueError(f"{label} must contain at least two poses")
    if any(not isinstance(pose, DwbPose2D) for pose in frozen):
        raise TypeError(f"{label} must contain DwbPose2D values")
    return frozen


def _dwb_pose(pose: Pose2D) -> DwbPose2D:
    return DwbPose2D(pose.x, pose.y, pose.yaw)


def _dwb_path_hash(path: Sequence[DwbPose2D]) -> str:
    payload = tuple((pose.x_m.hex(), pose.y_m.hex(), pose.yaw_rad.hex()) for pose in path)
    return sha256(dumps(payload, separators=(",", ":")).encode("ascii")).hexdigest()


def _candidate_diagnostics(
    trace: tuple[str, ...],
    session: PersistentDwbSessionDiagnostics,
) -> tuple[str, ...]:
    prefixes = (
        "candidate_count=",
        "legal_candidates=",
        "illegal_candidates=",
        "short_circuited_candidates=",
        "selected_candidate_index=",
        "total_score=",
        "rejection.",
        "selected_critic.",
    )
    selected = [item for item in trace if item.startswith(prefixes)]
    selected.extend(
        (
            f"session_reset_count={session.session_reset_count}",
            f"scoring_window_update_count={session.scoring_window_update_count}",
            "terminal_goal_source=immutable_full_reference",
        )
    )
    return tuple(sorted(set(selected)))


def _active_section_tracking_error(
    tick_input: PersistentControllerTickInput,
    section_index: int,
) -> float:
    section = tick_input.full_reference.sections[section_index]
    poses = tuple(
        tick_input.full_reference.knots[index].pose
        for index in range(section.first_knot_index, section.last_knot_index + 1)
    )
    return _distance_to_polyline(tick_input.robot_state.pose, poses)


def _distance_to_polyline(pose: Pose2D, path: Sequence[Pose2D]) -> float:
    if len(path) == 1:
        return hypot(pose.x - path[0].x, pose.y - path[0].y)
    best = float("inf")
    for start, end in zip(path, path[1:], strict=False):
        dx = end.x - start.x
        dy = end.y - start.y
        length_sq = dx * dx + dy * dy
        ratio = (
            0.0
            if length_sq <= _TOLERANCE
            else ((pose.x - start.x) * dx + (pose.y - start.y) * dy) / length_sq
        )
        ratio = max(0.0, min(1.0, ratio))
        nearest_x = start.x + ratio * dx
        nearest_y = start.y + ratio * dy
        best = min(best, hypot(pose.x - nearest_x, pose.y - nearest_y))
    return best


def _post_apply_constant_rollout(
    tick_input: PersistentControllerTickInput,
    command: Twist2D,
) -> tuple[TrajectoryPoint, ...]:
    pose = _integrate_pose(
        tick_input.robot_state.pose,
        tick_input.robot_state.twist,
        tick_input.vehicle_profile.control_period_s,
    )
    points = [TrajectoryPoint(0.0, pose, command)]
    for index in range(1, 41):
        pose = _integrate_pose(pose, command, 0.05)
        points.append(TrajectoryPoint(index * 0.05, pose, command))
    return tuple(points)


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
        (next_yaw + pi) % (2.0 * pi) - pi,
    )


__all__ = [
    "PERSISTENT_DWB_ADAPTER_VERSION",
    "PERSISTENT_DWB_CONTROLLER_NAME",
    "PersistentDwbCoreSession",
    "PersistentDwbSessionDiagnostics",
    "PersistentSourceDerivedDwbController",
]
