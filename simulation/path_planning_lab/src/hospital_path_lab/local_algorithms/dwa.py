"""가상 축소 차체용 결정론적 Dynamic Window Approach 기준 구현."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace
from enum import StrEnum
from hashlib import sha256
from json import dumps
from math import atan2, copysign, cos, hypot, inf, isfinite, pi, sin
from time import perf_counter_ns

from hospital_path_lab.collision import (
    CollisionChecker,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.contracts import (
    GridSnapshot,
    LocalPlanResult,
    PlanStatus,
    Pose2D,
    RobotState,
    TrajectoryPoint,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_COMMAND_APPLY_LATENCY_S,
    ControllerCommandResult,
    ControllerSnapshot,
    DynamicCommandProposal,
)
from hospital_path_lab.dynamic_prediction import (
    ActorPredictionSet,
    ActorTubeCircle,
    sample_actor_tubes,
)
from hospital_path_lab.dynamic_safety import (
    DYNAMIC_ANGULAR_DECELERATION_RADPS2,
    evaluate_dynamic_trajectory_safety,
)
from hospital_path_lab.grid import GridMap
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile


@dataclass(frozen=True, slots=True)
class _Candidate:
    linear: float
    angular: float
    path: tuple[Pose2D, ...]
    trajectory: tuple[TrajectoryPoint, ...]
    progress: float
    reference_distance: float
    heading_error: float
    minimum_clearance: float


class DynamicWindowPlanner:
    """한 제어주기 내 도달 가능한 속도로 2초 궤적을 비교한다."""

    name = "dwa"

    def __init__(
        self,
        *,
        vehicle: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        horizon_s: float = 2.0,
        integration_dt_s: float = 0.05,
        linear_samples: int = 7,
        angular_samples: int = 31,
        max_angular_acceleration_radps2: float = 1.6,
        goal_tolerance_m: float = 0.05,
    ) -> None:
        if horizon_s <= 0 or integration_dt_s <= 0:
            raise ValueError("horizon과 integration dt는 양수여야 합니다.")
        if linear_samples < 2 or angular_samples < 2:
            raise ValueError("속도 표본 수는 각각 2 이상이어야 합니다.")
        if max_angular_acceleration_radps2 <= 0:
            raise ValueError("각가속도 한계는 양수여야 합니다.")
        if goal_tolerance_m <= 0:
            raise ValueError("goal tolerance must be positive")
        self.vehicle = vehicle
        self.horizon_s = horizon_s
        self.integration_dt_s = integration_dt_s
        self.linear_sample_count = linear_samples
        self.angular_sample_count = angular_samples
        # 공통 차체 계약에 아직 각가속도가 없어 연구용 planner 설정으로만 둔다.
        self.max_angular_acceleration_radps2 = max_angular_acceleration_radps2
        self.goal_tolerance_m = goal_tolerance_m
        self._cached_snapshot: GridSnapshot | None = None
        self._cached_obstacle_checker: CollisionChecker | None = None
        self._cached_collision_checker: CollisionChecker | None = None

    def plan(
        self,
        snapshot: GridSnapshot,
        reference_path: tuple[Pose2D, ...],
        robot_state: RobotState,
        goal: Pose2D,
    ) -> LocalPlanResult:
        started_at = perf_counter_ns()
        if not snapshot.input_valid:
            return self._result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="snapshot_input_invalidated",
            )
        invalid_reason = _invalid_input_reason(
            snapshot.grid, reference_path, robot_state, goal, self.vehicle
        )
        if invalid_reason is not None:
            return self._result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason=invalid_reason,
            )

        obstacle_checker, collision_checker = self._collision_checkers_for(snapshot)
        if collision_checker.pose_enters_forbidden(robot_state.pose):
            return self._result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="start_forbidden",
            )
        if collision_checker.pose_enters_forbidden(goal):
            return self._result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="goal_forbidden",
            )
        if not obstacle_checker.pose_is_collision_free(robot_state.pose):
            return self._result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="start_footprint_occupied",
            )
        if not obstacle_checker.pose_is_collision_free(goal):
            return self._result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="goal_footprint_occupied",
            )

        if (
            _distance(robot_state.pose, goal) <= self.goal_tolerance_m
            and _twist_is_stopped(robot_state.twist)
        ):
            trajectory = self._rollout(robot_state.pose, 0.0, 0.0)
            clearance = min(
                collision_checker.conservative_clearance(point.pose)
                for point in trajectory
            )
            return self._result(
                snapshot,
                status=PlanStatus.FOUND,
                path=tuple(point.pose for point in trajectory),
                trajectory=trajectory,
                cost=0.0,
                elapsed_ns=perf_counter_ns() - started_at,
                minimum_clearance=clearance,
            )

        linear_values, angular_values = self._dynamic_window(robot_state)
        sampled_trajectories = len(linear_values) * len(angular_values)
        candidates: list[_Candidate] = []
        for linear in linear_values:
            for angular in angular_values:
                trajectory = self._rollout(robot_state.pose, linear, angular)
                path = tuple(point.pose for point in trajectory)
                if not collision_checker.conservative_path_is_collision_free(path):
                    continue

                minimum_clearance = min(
                    collision_checker.conservative_clearance(pose) for pose in path
                )
                if minimum_clearance < self.vehicle.minimum_clearance_m:
                    continue
                stopping_distance = (
                    linear * linear / (2.0 * self.vehicle.max_deceleration_mps2)
                    + self.vehicle.stopping_margin_m
                )
                # 정지거리를 모든 방향의 최소 여유와 직접 비교하면 복도 옆 벽까지의
                # lateral clearance 때문에 안전한 직진도 전부 탈락한다. 대신 현재
                # 곡률을 따라 정지 여유만큼 더 진행하는 swept path가 비어 있는지
                # 검사해 전방 정지 가능성과 측면 여유를 분리한다.
                stopping_sweep = _sweep_distance(
                    path[-1],
                    linear,
                    angular,
                    stopping_distance,
                    step_m=snapshot.grid.resolution_m / 2.0,
                )
                if not collision_checker.conservative_path_is_collision_free(stopping_sweep):
                    continue

                start_distance = _distance(robot_state.pose, goal)
                end_distance = _distance(path[-1], goal)
                candidates.append(
                    _Candidate(
                        linear=linear,
                        angular=angular,
                        path=path,
                        trajectory=trajectory,
                        progress=start_distance - end_distance,
                        reference_distance=_mean_reference_distance(path, reference_path),
                        heading_error=_heading_error(path[-1], goal),
                        minimum_clearance=minimum_clearance,
                    )
                )

        active_candidates = [
            candidate
            for candidate in candidates
            if abs(candidate.linear) > 1e-12 or abs(candidate.angular) > 1e-12
        ]
        if not active_candidates:
            return self._result(
                snapshot,
                status=PlanStatus.NO_PATH,
                elapsed_ns=perf_counter_ns() - started_at,
                sampled_trajectories=sampled_trajectories,
                failure_reason="no_safe_moving_trajectory",
            )

        best, score = _select_candidate(
            active_candidates,
            max(
                self.vehicle.max_forward_speed_mps,
                self.vehicle.max_reverse_speed_mps,
            ),
        )
        return self._result(
            snapshot,
            status=PlanStatus.FOUND,
            path=best.path,
            trajectory=best.trajectory,
            cost=score,
            elapsed_ns=perf_counter_ns() - started_at,
            sampled_trajectories=sampled_trajectories,
            collision=False,
            minimum_clearance=best.minimum_clearance,
        )

    def _collision_checkers_for(
        self, snapshot: GridSnapshot
    ) -> tuple[CollisionChecker, CollisionChecker]:
        if self._cached_snapshot is not snapshot:
            self._cached_snapshot = snapshot
            self._cached_obstacle_checker = CollisionChecker(snapshot.grid, self.vehicle)
            self._cached_collision_checker = CollisionChecker(
                snapshot.grid,
                self.vehicle,
                forbidden_cells=snapshot.forbidden_cells,
            )
        if (
            self._cached_obstacle_checker is None
            or self._cached_collision_checker is None
        ):  # pragma: no cover - defensive
            raise RuntimeError("collision checker cache was not initialized")
        return self._cached_obstacle_checker, self._cached_collision_checker

    def _dynamic_window(
        self,
        robot_state: RobotState,
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        period = self.vehicle.control_period_s
        current_linear = robot_state.twist.linear
        if current_linear >= 0.0:
            lower_delta = self.vehicle.max_deceleration_mps2 * period
            upper_delta = self.vehicle.max_acceleration_mps2 * period
        else:
            lower_delta = self.vehicle.max_acceleration_mps2 * period
            upper_delta = self.vehicle.max_deceleration_mps2 * period
        reachable_linear_min = max(
            -self.vehicle.max_reverse_speed_mps,
            current_linear - lower_delta,
        )
        reachable_linear_max = min(
            self.vehicle.max_forward_speed_mps,
            current_linear + upper_delta,
        )
        if reachable_linear_max < reachable_linear_min:
            clamped = min(
                self.vehicle.max_forward_speed_mps,
                max(-self.vehicle.max_reverse_speed_mps, current_linear),
            )
            reachable_linear_min = clamped
            reachable_linear_max = clamped
        linear_values = _linear_samples_with_stop(
            reachable_linear_min,
            reachable_linear_max,
            self.linear_sample_count,
        )

        angular_delta = self.max_angular_acceleration_radps2 * period
        reachable_angular_min = max(
            -self.vehicle.max_angular_speed_radps,
            robot_state.twist.angular - angular_delta,
        )
        reachable_angular_max = min(
            self.vehicle.max_angular_speed_radps,
            robot_state.twist.angular + angular_delta,
        )
        if reachable_angular_max < reachable_angular_min:
            clamped = min(
                self.vehicle.max_angular_speed_radps,
                max(-self.vehicle.max_angular_speed_radps, robot_state.twist.angular),
            )
            reachable_angular_min = clamped
            reachable_angular_max = clamped
        angular_values = _linspace(
            reachable_angular_min,
            reachable_angular_max,
            self.angular_sample_count,
        )
        return linear_values, angular_values

    def _rollout(
        self,
        start: Pose2D,
        linear: float,
        angular: float,
    ) -> tuple[TrajectoryPoint, ...]:
        command = Twist2D(linear=linear, angular=angular)
        points = [TrajectoryPoint(time_s=0.0, pose=start, twist=command)]
        x, y, yaw = start.x, start.y, start.yaw
        steps = int(round(self.horizon_s / self.integration_dt_s))
        for step in range(1, steps + 1):
            if abs(angular) <= 1e-12:
                x += linear * cos(yaw) * self.integration_dt_s
                y += linear * sin(yaw) * self.integration_dt_s
            else:
                next_yaw = yaw + angular * self.integration_dt_s
                radius = linear / angular
                x += radius * (sin(next_yaw) - sin(yaw))
                y -= radius * (cos(next_yaw) - cos(yaw))
                yaw = next_yaw
            yaw = _normalize_angle(yaw)
            points.append(
                TrajectoryPoint(
                    time_s=step * self.integration_dt_s,
                    pose=Pose2D(x=x, y=y, yaw=yaw),
                    twist=command,
                )
            )
        return tuple(points)

    def _result(
        self,
        snapshot: GridSnapshot,
        *,
        status: PlanStatus,
        elapsed_ns: int,
        path: tuple[Pose2D, ...] = (),
        trajectory: tuple[TrajectoryPoint, ...] = (),
        cost: float | None = None,
        sampled_trajectories: int = 0,
        collision: bool = False,
        minimum_clearance: float | None = None,
        failure_reason: str | None = None,
    ) -> LocalPlanResult:
        metadata = snapshot.metadata
        return LocalPlanResult(
            planner=self.name,
            status=status,
            path=path,
            trajectory=trajectory,
            cost=cost,
            elapsed_ns=elapsed_ns,
            expanded_nodes=0,
            sampled_trajectories=sampled_trajectories,
            map_revision=metadata.map_revision,
            mission_revision=metadata.mission_revision,
            observation_revision=metadata.observation_revision,
            collision=collision,
            minimum_clearance=minimum_clearance,
            map_id=metadata.map_id,
            input_content_hash=metadata.content_hash,
            failure_reason=failure_reason,
        )


@dataclass(frozen=True, slots=True)
class _DynamicCandidate:
    command: Twist2D
    trajectory: tuple[TrajectoryPoint, ...]
    progress: float
    minimum_clearance: float
    progress_cost: float
    reference_path_cost: float
    heading_cost: float
    clearance_cost: float
    speed_cost: float
    oscillation_cost: float
    score: float
    sample_index: int = -1

    @property
    def rank(self) -> tuple[float, ...]:
        return (
            self.score,
            -self.minimum_clearance,
            -self.progress,
            self.reference_path_cost,
            self.heading_cost,
            self.oscillation_cost,
            abs(self.command.angular),
            -self.command.linear,
            self.command.angular,
        )


class DynamicDwaCandidatePhase(StrEnum):
    """v6 candidate 판정 단계. exact 내부 구간은 shared API 한계로 합친다."""

    INPUT = "INPUT"
    COARSE_ROLLOUT = "COARSE_ROLLOUT"
    COARSE_TERMINAL = "COARSE_TERMINAL"
    EXACT_SHARED_GATE = "EXACT_SHARED_GATE"
    RANKING = "RANKING"


class DynamicDwaCandidateCause(StrEnum):
    """v6 DWA 후보 결과 taxonomy."""

    STATIC_OCCUPANCY = "STATIC_OCCUPANCY"
    STATIC_CLEARANCE = "STATIC_CLEARANCE"
    FORBIDDEN_ZONE = "FORBIDDEN_ZONE"
    ACTOR_TUBE = "ACTOR_TUBE"
    PREDICTION_INVALID = "PREDICTION_INVALID"
    TERMINAL_STOPPING = "TERMINAL_STOPPING"
    SHARED_GATE = "SHARED_GATE"
    ADMISSIBLE_NOT_SELECTED = "ADMISSIBLE_NOT_SELECTED"
    NOT_EVALUATED_AFTER_SELECTION = "NOT_EVALUATED_AFTER_SELECTION"
    SELECTED = "SELECTED"


@dataclass(frozen=True, slots=True)
class DynamicDwaCandidateDiagnostic:
    """IPC에 싣기 전에 제한하는 결정론적 후보 진단 한 건."""

    sample_index: int
    command: Twist2D
    phase: DynamicDwaCandidatePhase
    cause: DynamicDwaCandidateCause
    failure_time_s: float | None = None
    minimum_static_clearance_m: float | None = None
    minimum_actor_clearance_m: float | None = None
    shared_gate_failures: tuple[str, ...] = ()
    underlying_terminal_cause: DynamicDwaCandidateCause | None = None


@dataclass(frozen=True, slots=True)
class DynamicDwaDiagnosticSummary:
    """한 step의 고정 순서 집계와 제한된 detail."""

    schema_version: str
    sampled_candidates: int
    moving_candidates: int
    coarse_admissible_candidates: int
    nonmoving_samples: int
    ordered_counts: tuple[tuple[str, str, int], ...]
    selected_sample_index: int | None
    selected_rank: int | None
    selected_score: float | None
    selected_rank_key: tuple[float, ...] | None
    details: tuple[DynamicDwaCandidateDiagnostic, ...]
    exact_phase_granularity: str
    semantic_digest: str


@dataclass(frozen=True, slots=True)
class _CoarseCandidateEvaluation:
    minimum_clearance_m: float | None
    failure_phase: DynamicDwaCandidatePhase | None = None
    failure_cause: DynamicDwaCandidateCause | None = None
    failure_time_s: float | None = None
    minimum_static_clearance_m: float | None = None
    minimum_actor_clearance_m: float | None = None
    underlying_terminal_cause: DynamicDwaCandidateCause | None = None
    used_certified_actor_dominance: bool = False

    @property
    def accepted(self) -> bool:
        return self.failure_cause is None


@dataclass(frozen=True, slots=True)
class DynamicDwaWorkspaceMetrics:
    """Non-semantic counters proving step-local work reduction."""

    coarse_candidates: int = 0
    certified_actor_dominated_candidates: int = 0
    reference_geometry_candidates: int = 0


class _StepActorTubeSampler:
    """한 DWA step 안에서 동일 rollout 시각의 immutable tube만 재사용한다."""

    def __init__(self, prediction_set: ActorPredictionSet, *, enabled: bool) -> None:
        self._prediction_set = prediction_set
        self._enabled = enabled
        self._samples: dict[float, tuple[ActorTubeCircle, ...]] = {}

    def sample(self, rollout_time_s: float) -> tuple[ActorTubeCircle, ...]:
        if not self._enabled:
            return sample_actor_tubes(
                self._prediction_set,
                rollout_time_s=rollout_time_s,
            )
        cached = self._samples.get(rollout_time_s)
        if cached is None:
            cached = sample_actor_tubes(
                self._prediction_set,
                rollout_time_s=rollout_time_s,
            )
            self._samples[rollout_time_s] = cached
        return cached


_DWA_DIAGNOSTIC_SCHEMA = "dynamic-dwa-candidate-v6"
_DWA_DIAGNOSTIC_DETAIL_LIMIT = 8
_DWA_SEMANTIC_DIGEST_TRACE_PREFIX = "dwa_controller_semantic_digest="
_DWA_COUNT_ORDER = (
    (DynamicDwaCandidatePhase.INPUT, DynamicDwaCandidateCause.PREDICTION_INVALID),
    (DynamicDwaCandidatePhase.INPUT, DynamicDwaCandidateCause.SHARED_GATE),
    (DynamicDwaCandidatePhase.COARSE_ROLLOUT, DynamicDwaCandidateCause.STATIC_OCCUPANCY),
    (DynamicDwaCandidatePhase.COARSE_ROLLOUT, DynamicDwaCandidateCause.STATIC_CLEARANCE),
    (DynamicDwaCandidatePhase.COARSE_ROLLOUT, DynamicDwaCandidateCause.FORBIDDEN_ZONE),
    (DynamicDwaCandidatePhase.COARSE_ROLLOUT, DynamicDwaCandidateCause.ACTOR_TUBE),
    (DynamicDwaCandidatePhase.COARSE_ROLLOUT, DynamicDwaCandidateCause.PREDICTION_INVALID),
    (DynamicDwaCandidatePhase.COARSE_TERMINAL, DynamicDwaCandidateCause.TERMINAL_STOPPING),
    (DynamicDwaCandidatePhase.EXACT_SHARED_GATE, DynamicDwaCandidateCause.STATIC_CLEARANCE),
    (DynamicDwaCandidatePhase.EXACT_SHARED_GATE, DynamicDwaCandidateCause.FORBIDDEN_ZONE),
    (DynamicDwaCandidatePhase.EXACT_SHARED_GATE, DynamicDwaCandidateCause.ACTOR_TUBE),
    (DynamicDwaCandidatePhase.EXACT_SHARED_GATE, DynamicDwaCandidateCause.PREDICTION_INVALID),
    (DynamicDwaCandidatePhase.EXACT_SHARED_GATE, DynamicDwaCandidateCause.SHARED_GATE),
    (DynamicDwaCandidatePhase.RANKING, DynamicDwaCandidateCause.ADMISSIBLE_NOT_SELECTED),
    (
        DynamicDwaCandidatePhase.RANKING,
        DynamicDwaCandidateCause.NOT_EVALUATED_AFTER_SELECTION,
    ),
    (DynamicDwaCandidatePhase.RANKING, DynamicDwaCandidateCause.SELECTED),
)


class DynamicDwaController:
    """v5 고정 비용식과 Actor tube를 사용하는 동적 DWA adapter."""

    name = "dynamic_dwa"
    horizon_s = 2.0
    integration_dt_s = 0.05
    linear_sample_count = 7
    angular_sample_count = 31
    max_angular_acceleration_radps2 = 1.60
    goal_tolerance_m = 0.05

    def __init__(
        self,
        vehicle: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        *,
        use_step_local_workspace: bool = True,
        verify_all_ranked_candidates: bool = False,
    ) -> None:
        if not vehicle.simulation_only:
            raise ValueError("dynamic DWA requires a simulation-only vehicle profile")
        self.vehicle = vehicle
        self.use_step_local_workspace = use_step_local_workspace
        self.verify_all_ranked_candidates = verify_all_ranked_candidates
        self.last_diagnostics: DynamicDwaDiagnosticSummary | None = None
        self.last_workspace_metrics = DynamicDwaWorkspaceMetrics()

    def step(self, snapshot: ControllerSnapshot) -> ControllerCommandResult:
        started_at = perf_counter_ns()
        self.last_diagnostics = None
        self.last_workspace_metrics = DynamicDwaWorkspaceMetrics()
        invalid_reason = self._invalid_reason(snapshot)
        if invalid_reason is not None:
            input_cause = _input_failure_cause(invalid_reason)
            diagnostics = _dynamic_dwa_diagnostic_summary(
                sampled_candidates=0,
                moving_candidates=0,
                coarse_admissible_candidates=0,
                nonmoving_samples=0,
                counts=Counter({(DynamicDwaCandidatePhase.INPUT, input_cause): 1}),
                selected=None,
                selected_rank=None,
                details=[
                    DynamicDwaCandidateDiagnostic(
                        sample_index=-1,
                        command=Twist2D(),
                        phase=DynamicDwaCandidatePhase.INPUT,
                        cause=input_cause,
                        shared_gate_failures=(invalid_reason,),
                    )
                ],
            )
            self.last_diagnostics = diagnostics
            return _dynamic_controller_result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.INVALID_INPUT,
                failure_reason=invalid_reason,
                decision_trace=_dynamic_dwa_diagnostic_trace(diagnostics),
                controller_requested_stop=True,
            )

        if (
            _distance(snapshot.robot_state.pose, snapshot.goal_pose)
            <= self.goal_tolerance_m
            and _twist_is_stopped(snapshot.robot_state.twist)
        ):
            apply_end = _integrate_pose(
                snapshot.robot_state.pose,
                snapshot.robot_state.twist,
                DYNAMIC_COMMAND_APPLY_LATENCY_S,
            )
            trajectory = _dynamic_constant_rollout(
                apply_end,
                Twist2D(),
                horizon_s=self.horizon_s,
                step_s=self.integration_dt_s,
            )
            diagnostics = _dynamic_dwa_diagnostic_summary(
                sampled_candidates=0,
                moving_candidates=0,
                coarse_admissible_candidates=0,
                nonmoving_samples=0,
                counts=Counter(),
                selected=None,
                selected_rank=None,
                details=[],
            )
            self.last_diagnostics = diagnostics
            return _dynamic_controller_result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.FOUND,
                predicted_trajectory=trajectory,
                decision_trace=(
                    "goal_reached=true",
                    "sampled_candidates=0",
                    *_dynamic_dwa_diagnostic_trace(diagnostics),
                ),
            )

        linear_values, angular_values = self._dynamic_window(snapshot.robot_state)
        sampled_candidates = len(linear_values) * len(angular_values)
        apply_end = _integrate_pose(
            snapshot.robot_state.pose,
            snapshot.robot_state.twist,
            DYNAMIC_COMMAND_APPLY_LATENCY_S,
        )
        physical_checker = CollisionChecker(
            snapshot.static_grid_snapshot.grid,
            self.vehicle,
            use_optimized_geometry=self.use_step_local_workspace,
        )
        combined_checker = (
            CollisionChecker(
                snapshot.static_grid_snapshot.grid,
                self.vehicle,
                forbidden_cells=snapshot.static_grid_snapshot.forbidden_cells,
                use_optimized_geometry=self.use_step_local_workspace,
            )
            if snapshot.static_grid_snapshot.forbidden_cells
            else physical_checker
        )
        if snapshot.actor_tubes is None:  # pragma: no cover - _invalid_reason owns this
            raise RuntimeError("validated Actor prediction unexpectedly disappeared")
        actor_sampler = _StepActorTubeSampler(
            snapshot.actor_tubes,
            enabled=self.use_step_local_workspace,
        )
        reference_segments = _prepare_reference_segments(
            snapshot.reference_path
        )
        start_goal_distance = _distance(snapshot.robot_state.pose, snapshot.goal_pose)
        counts: Counter[tuple[DynamicDwaCandidatePhase, DynamicDwaCandidateCause]] = Counter()
        details: list[DynamicDwaCandidateDiagnostic] = []
        nonmoving_samples = 0
        candidates: list[_DynamicCandidate] = []
        coarse_candidates = 0
        certified_actor_dominated_candidates = 0
        sample_index = -1
        for linear in linear_values:
            for angular in angular_values:
                sample_index += 1
                if linear <= 1e-12:
                    nonmoving_samples += 1
                    continue
                command = Twist2D(linear=linear, angular=angular)
                trajectory = _dynamic_constant_rollout(
                    apply_end,
                    command,
                    horizon_s=self.horizon_s,
                    step_s=self.integration_dt_s,
                )
                coarse = _coarse_dynamic_candidate_clearance(
                    trajectory,
                    snapshot=snapshot,
                    physical_checker=physical_checker,
                    combined_checker=combined_checker,
                    vehicle=self.vehicle,
                    actor_sampler=actor_sampler,
                    use_certified_actor_dominance=self.use_step_local_workspace,
                    preserve_rejection_detail=(
                        len(details) < _DWA_DIAGNOSTIC_DETAIL_LIMIT
                    ),
                )
                coarse_candidates += 1
                certified_actor_dominated_candidates += int(
                    coarse.used_certified_actor_dominance
                )
                if not coarse.accepted:
                    if coarse.failure_phase is None or coarse.failure_cause is None:
                        raise RuntimeError("coarse rejection must have a structured reason")
                    counts[(coarse.failure_phase, coarse.failure_cause)] += 1
                    _append_diagnostic_detail(
                        details,
                        DynamicDwaCandidateDiagnostic(
                            sample_index=sample_index,
                            command=command,
                            phase=coarse.failure_phase,
                            cause=coarse.failure_cause,
                            failure_time_s=coarse.failure_time_s,
                            minimum_static_clearance_m=(coarse.minimum_static_clearance_m),
                            minimum_actor_clearance_m=coarse.minimum_actor_clearance_m,
                            underlying_terminal_cause=(coarse.underlying_terminal_cause),
                        ),
                    )
                    continue
                if coarse.minimum_clearance_m is None:  # pragma: no cover - invariant
                    raise RuntimeError("accepted coarse candidate must have clearance")
                candidates.append(
                    _dynamic_candidate(
                        command,
                        trajectory,
                        start=snapshot.robot_state.pose,
                        goal=snapshot.goal_pose,
                        reference_path=snapshot.reference_path,
                        minimum_clearance=coarse.minimum_clearance_m,
                        previous_angular=snapshot.robot_state.twist.angular,
                        sample_index=sample_index,
                        start_goal_distance=start_goal_distance,
                        reference_segments=reference_segments,
                    )
                )

        self.last_workspace_metrics = DynamicDwaWorkspaceMetrics(
            coarse_candidates=coarse_candidates,
            certified_actor_dominated_candidates=(
                certified_actor_dominated_candidates
            ),
            reference_geometry_candidates=(
                coarse_candidates - certified_actor_dominated_candidates
            ),
        )

        candidates.sort(key=lambda candidate: candidate.rank)
        selected: _DynamicCandidate | None = None
        selected_rank: int | None = None
        for rank, candidate in enumerate(candidates):
            proposal = _dynamic_proposal(snapshot, candidate.command, candidate.trajectory)
            evidence = evaluate_dynamic_trajectory_safety(
                proposal,
                robot_state=snapshot.robot_state,
                grid_snapshot=snapshot.static_grid_snapshot,
                prediction_set=snapshot.actor_tubes,
                profile=self.vehicle,
            )
            if evidence.safe:
                if selected is None:
                    selected = candidate
                    selected_rank = rank
                    counts[
                        (
                            DynamicDwaCandidatePhase.RANKING,
                            DynamicDwaCandidateCause.SELECTED,
                        )
                    ] += 1
                    if not self.verify_all_ranked_candidates:
                        remaining = len(candidates) - rank - 1
                        counts[
                            (
                                DynamicDwaCandidatePhase.RANKING,
                                DynamicDwaCandidateCause.NOT_EVALUATED_AFTER_SELECTION,
                            )
                        ] += remaining
                        break
                else:
                    counts[
                        (
                            DynamicDwaCandidatePhase.RANKING,
                            DynamicDwaCandidateCause.ADMISSIBLE_NOT_SELECTED,
                        )
                    ] += 1
                    _append_diagnostic_detail(
                        details,
                        DynamicDwaCandidateDiagnostic(
                            sample_index=candidate.sample_index,
                            command=candidate.command,
                            phase=DynamicDwaCandidatePhase.RANKING,
                            cause=DynamicDwaCandidateCause.ADMISSIBLE_NOT_SELECTED,
                            minimum_static_clearance_m=evidence.minimum_static_clearance_m,
                            minimum_actor_clearance_m=evidence.minimum_actor_clearance_m,
                        ),
                    )
                continue
            exact_cause = _shared_gate_failure_cause(evidence.failures)
            counts[(DynamicDwaCandidatePhase.EXACT_SHARED_GATE, exact_cause)] += 1
            _append_diagnostic_detail(
                details,
                DynamicDwaCandidateDiagnostic(
                    sample_index=candidate.sample_index,
                    command=candidate.command,
                    phase=DynamicDwaCandidatePhase.EXACT_SHARED_GATE,
                    cause=exact_cause,
                    minimum_static_clearance_m=evidence.minimum_static_clearance_m,
                    minimum_actor_clearance_m=evidence.minimum_actor_clearance_m,
                    shared_gate_failures=evidence.failures,
                ),
            )

        diagnostics = _dynamic_dwa_diagnostic_summary(
            sampled_candidates=sampled_candidates,
            moving_candidates=sampled_candidates - nonmoving_samples,
            coarse_admissible_candidates=len(candidates),
            nonmoving_samples=nonmoving_samples,
            counts=counts,
            selected=selected,
            selected_rank=selected_rank,
            details=details,
        )
        self.last_diagnostics = diagnostics
        diagnostic_trace = _dynamic_dwa_diagnostic_trace(diagnostics)

        if selected is None:
            return _dynamic_controller_result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.NO_PATH,
                failure_reason="no_safe_candidate",
                decision_trace=(
                    f"sampled_candidates={sampled_candidates}",
                    f"coarse_admissible_candidates={len(candidates)}",
                    *diagnostic_trace,
                ),
                controller_requested_stop=True,
                no_safe_candidate=True,
            )

        trace = (
            f"sampled_candidates={sampled_candidates}",
            f"coarse_admissible_candidates={len(candidates)}",
            "pose_samples=41",
            f"score={selected.score:.12g}",
            f"progress_cost={selected.progress_cost:.12g}",
            f"reference_path_cost={selected.reference_path_cost:.12g}",
            f"heading_cost={selected.heading_cost:.12g}",
            f"clearance_cost={selected.clearance_cost:.12g}",
            f"speed_cost={selected.speed_cost:.12g}",
            f"oscillation_cost={selected.oscillation_cost:.12g}",
            f"minimum_clearance_m={selected.minimum_clearance:.12g}",
            *diagnostic_trace,
        )
        return _dynamic_controller_result(
            self.name,
            snapshot,
            started_at,
            status=PlanStatus.FOUND,
            requested_twist=selected.command,
            predicted_trajectory=selected.trajectory,
            decision_trace=trace,
        )

    def _invalid_reason(self, snapshot: ControllerSnapshot) -> str | None:
        if snapshot.vehicle_profile != self.vehicle:
            return "vehicle_profile_mismatch"
        if not snapshot.static_grid_snapshot.input_valid:
            return "grid_snapshot_invalid"
        if snapshot.actor_tubes is None:
            return "actor_prediction_missing"
        twist = snapshot.robot_state.twist
        if not (0.0 <= twist.linear <= self.vehicle.nominal_speed_mps):
            return "dynamic_dwa_linear_state_outside_frozen_range"
        if abs(twist.angular) > self.vehicle.max_angular_speed_radps:
            return "dynamic_dwa_angular_state_outside_vehicle_limits"
        return None

    def _dynamic_window(
        self,
        robot_state: RobotState,
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        period = self.vehicle.control_period_s
        linear_min = max(
            0.0,
            robot_state.twist.linear - self.vehicle.max_deceleration_mps2 * period,
        )
        linear_max = min(
            self.vehicle.nominal_speed_mps,
            robot_state.twist.linear + self.vehicle.max_acceleration_mps2 * period,
        )
        angular_delta = self.max_angular_acceleration_radps2 * period
        angular_min = max(
            -self.vehicle.max_angular_speed_radps,
            robot_state.twist.angular - angular_delta,
        )
        angular_max = min(
            self.vehicle.max_angular_speed_radps,
            robot_state.twist.angular + angular_delta,
        )
        return (
            _samples_with_zero(linear_min, linear_max, self.linear_sample_count),
            _samples_with_zero(angular_min, angular_max, self.angular_sample_count),
        )


def _dynamic_controller_result(
    controller_name: str,
    snapshot: ControllerSnapshot,
    started_at: int,
    *,
    status: PlanStatus,
    requested_twist: Twist2D | None = None,
    predicted_trajectory: tuple[TrajectoryPoint, ...] = (),
    failure_reason: str | None = None,
    decision_trace: tuple[str, ...] = (),
    controller_requested_stop: bool = False,
    no_safe_candidate: bool = False,
) -> ControllerCommandResult:
    metadata = snapshot.static_grid_snapshot.metadata
    result = ControllerCommandResult(
        controller_name=controller_name,
        source_tick_id=snapshot.tick_id,
        status=status,
        requested_twist=requested_twist if requested_twist is not None else Twist2D(),
        predicted_trajectory=predicted_trajectory,
        failure_reason=failure_reason,
        decision_trace=decision_trace,
        mission_id=snapshot.mission_id,
        map_id=snapshot.map_id,
        map_revision=snapshot.map_revision,
        mission_revision=snapshot.mission_revision,
        observation_revision=snapshot.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=snapshot.observation_content_hash,
        input_content_hash=snapshot.input_content_hash,
        elapsed_ns=perf_counter_ns() - started_at,
        controller_requested_stop=controller_requested_stop,
        no_safe_candidate=no_safe_candidate,
    )
    semantic_digest = dynamic_dwa_controller_semantic_digest(result)
    return replace(
        result,
        elapsed_ns=perf_counter_ns() - started_at,
        decision_trace=(
            *result.decision_trace,
            f"{_DWA_SEMANTIC_DIGEST_TRACE_PREFIX}{semantic_digest}",
        ),
    )


def dynamic_dwa_controller_semantic_digest(result: ControllerCommandResult) -> str:
    """elapsed를 제외한 DWA controller 결과 전체의 결정론적 digest."""

    payload = {
        "controller_name": result.controller_name,
        "source_tick_id": result.source_tick_id,
        "status": result.status.value,
        "requested_twist": _twist_payload(result.requested_twist),
        "predicted_trajectory": [
            {
                "time_s": _float_token(point.time_s),
                "pose": _pose_payload(point.pose),
                "twist": _twist_payload(point.twist),
            }
            for point in result.predicted_trajectory
        ],
        "failure_reason": result.failure_reason,
        "decision_trace": [
            item
            for item in result.decision_trace
            if not item.startswith(_DWA_SEMANTIC_DIGEST_TRACE_PREFIX)
        ],
        "mission_id": result.mission_id,
        "map_id": result.map_id,
        "map_revision": result.map_revision,
        "mission_revision": result.mission_revision,
        "observation_revision": result.observation_revision,
        "grid_content_hash": result.grid_content_hash,
        "observation_content_hash": result.observation_content_hash,
        "input_content_hash": result.input_content_hash,
        "controller_requested_stop": result.controller_requested_stop,
        "no_safe_candidate": result.no_safe_candidate,
    }
    return sha256(
        dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()


def _dynamic_proposal(
    snapshot: ControllerSnapshot,
    command: Twist2D,
    trajectory: tuple[TrajectoryPoint, ...],
) -> DynamicCommandProposal:
    metadata = snapshot.static_grid_snapshot.metadata
    return DynamicCommandProposal(
        source_tick_id=snapshot.tick_id,
        command=command,
        computation_time_s=0.0,
        mission_id=snapshot.mission_id,
        map_id=snapshot.map_id,
        map_revision=snapshot.map_revision,
        mission_revision=snapshot.mission_revision,
        observation_revision=snapshot.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=snapshot.observation_content_hash,
        trajectory=trajectory,
    )


def _dynamic_candidate(
    command: Twist2D,
    trajectory: tuple[TrajectoryPoint, ...],
    *,
    start: Pose2D,
    goal: Pose2D,
    reference_path: tuple[Pose2D, ...],
    minimum_clearance: float,
    previous_angular: float,
    sample_index: int = -1,
    start_goal_distance: float | None = None,
    reference_segments: tuple[
        tuple[Pose2D, Pose2D, float, float, float], ...
    ]
    | None = None,
) -> _DynamicCandidate:
    if start_goal_distance is None:
        start_goal_distance = _distance(start, goal)
    progress = start_goal_distance - _distance(trajectory[-1].pose, goal)
    progress_cost = 1.0 - _clip(progress / 0.40, 0.0, 1.0)
    if reference_segments is None:
        reference_segments = _prepare_reference_segments(reference_path)
    reference_distance = _mean_trajectory_polyline_distance(
        trajectory,
        reference_segments,
    )
    reference_path_cost = _clip(reference_distance / 0.50, 0.0, 1.0)
    heading_cost = _clip(_heading_error(trajectory[-1].pose, goal) / pi, 0.0, 1.0)
    clearance_cost = (
        0.0
        if minimum_clearance == inf
        else 1.0 - _clip((minimum_clearance - 0.08) / (0.50 - 0.08), 0.0, 1.0)
    )
    speed_cost = _clip((0.20 - command.linear) / 0.20, 0.0, 1.0)
    oscillation_cost = float(
        abs(previous_angular) > 0.05
        and abs(command.angular) > 0.05
        and previous_angular * command.angular < 0.0
    )
    score = (
        progress_cost
        + reference_path_cost
        + 0.5 * heading_cost
        + 1.5 * clearance_cost
        + 0.2 * speed_cost
        + 0.3 * oscillation_cost
    )
    return _DynamicCandidate(
        command=command,
        trajectory=trajectory,
        progress=progress,
        minimum_clearance=minimum_clearance,
        progress_cost=progress_cost,
        reference_path_cost=reference_path_cost,
        heading_cost=heading_cost,
        clearance_cost=clearance_cost,
        speed_cost=speed_cost,
        oscillation_cost=oscillation_cost,
        score=score,
        sample_index=sample_index,
    )


def _coarse_dynamic_candidate_clearance(
    trajectory: tuple[TrajectoryPoint, ...],
    *,
    snapshot: ControllerSnapshot,
    physical_checker: CollisionChecker,
    combined_checker: CollisionChecker,
    vehicle: VehicleProfile,
    actor_sampler: _StepActorTubeSampler | None = None,
    use_certified_actor_dominance: bool = False,
    preserve_rejection_detail: bool = False,
) -> _CoarseCandidateEvaluation:
    """50 ms DWA sampling prefilter; 선택 후보는 공통 5 ms gate로 다시 검사한다."""

    if snapshot.actor_tubes is None:
        return _CoarseCandidateEvaluation(
            None,
            failure_phase=DynamicDwaCandidatePhase.COARSE_ROLLOUT,
            failure_cause=DynamicDwaCandidateCause.PREDICTION_INVALID,
            failure_time_s=0.0,
        )
    if actor_sampler is None:
        actor_sampler = _StepActorTubeSampler(snapshot.actor_tubes, enabled=False)
    if use_certified_actor_dominance:
        actor_dominated = _certified_actor_dominated_clearance(
            trajectory,
            combined_checker=combined_checker,
            vehicle=vehicle,
            actor_sampler=actor_sampler,
            preserve_rejection_detail=preserve_rejection_detail,
        )
        if actor_dominated is not None:
            return actor_dominated
    minimum_clearance = inf
    minimum_static_clearance = inf
    minimum_actor_clearance = inf
    configuration_grid = combined_checker.configuration_grid
    for phase in (
        DynamicDwaCandidatePhase.COARSE_ROLLOUT,
        DynamicDwaCandidatePhase.COARSE_TERMINAL,
    ):
        if phase is DynamicDwaCandidatePhase.COARSE_ROLLOUT:
            points = trajectory
        else:
            terminal = _dynamic_terminal_rollout(
                trajectory[-1],
                linear_deceleration_mps2=vehicle.max_deceleration_mps2,
                angular_deceleration_radps2=DYNAMIC_ANGULAR_DECELERATION_RADPS2,
                step_s=0.05,
            )
            points = tuple(
                TrajectoryPoint(
                    time_s=trajectory[-1].time_s + point.time_s,
                    pose=point.pose,
                    twist=point.twist,
                )
                for point in terminal[1:]
            )
        for point in points:
            failure_cause, static_clearance, actor_clearance = _coarse_point_outcome(
                point,
                configuration_grid=configuration_grid,
                physical_checker=physical_checker,
                combined_checker=combined_checker,
                vehicle=vehicle,
                actor_sampler=actor_sampler,
            )
            if static_clearance is not None:
                minimum_static_clearance = min(minimum_static_clearance, static_clearance)
                minimum_clearance = min(minimum_clearance, static_clearance)
            if actor_clearance is not None:
                minimum_actor_clearance = min(
                    minimum_actor_clearance,
                    actor_clearance,
                )
                minimum_clearance = min(minimum_clearance, actor_clearance)
            if failure_cause is not None:
                terminal_cause = (
                    failure_cause if phase is DynamicDwaCandidatePhase.COARSE_TERMINAL else None
                )
                return _CoarseCandidateEvaluation(
                    None,
                    failure_phase=phase,
                    failure_cause=(
                        DynamicDwaCandidateCause.TERMINAL_STOPPING
                        if phase is DynamicDwaCandidatePhase.COARSE_TERMINAL
                        else failure_cause
                    ),
                    failure_time_s=point.time_s,
                    minimum_static_clearance_m=(
                        None if minimum_static_clearance == inf else minimum_static_clearance
                    ),
                    minimum_actor_clearance_m=(
                        None if minimum_actor_clearance == inf else minimum_actor_clearance
                    ),
                    underlying_terminal_cause=terminal_cause,
                )
    return _CoarseCandidateEvaluation(
        minimum_clearance,
        minimum_static_clearance_m=(
            None if minimum_static_clearance == inf else minimum_static_clearance
        ),
        minimum_actor_clearance_m=(
            None if minimum_actor_clearance == inf else minimum_actor_clearance
        ),
    )


def _coarse_point_outcome(
    point: TrajectoryPoint,
    *,
    configuration_grid: GridMap,
    physical_checker: CollisionChecker,
    combined_checker: CollisionChecker,
    vehicle: VehicleProfile,
    actor_sampler: _StepActorTubeSampler,
) -> tuple[DynamicDwaCandidateCause | None, float | None, float | None]:
    configuration_cell = configuration_grid.world_to_cell(point.pose)
    if configuration_grid.is_occupied(configuration_cell):
        physical_collision_grid = physical_checker.collision_grid
        physical_collision = physical_collision_grid.is_occupied(
            physical_collision_grid.world_to_cell(point.pose)
        )
        combined_collision = False
        if combined_checker is not physical_checker:
            combined_collision_grid = combined_checker.collision_grid
            combined_collision = combined_collision_grid.is_occupied(
                combined_collision_grid.world_to_cell(point.pose)
            )
        if combined_collision and not physical_collision:
            cause = DynamicDwaCandidateCause.FORBIDDEN_ZONE
        elif physical_collision:
            cause = DynamicDwaCandidateCause.STATIC_OCCUPANCY
        else:
            cause = DynamicDwaCandidateCause.STATIC_CLEARANCE
        return cause, None, None

    physical_clearance = physical_checker.clearance(point.pose)
    combined_clearance = (
        physical_clearance
        if combined_checker is physical_checker
        else combined_checker.clearance(point.pose)
    )
    static_clearance = min(physical_clearance, combined_clearance)
    forbidden_entry = combined_checker.pose_enters_forbidden(point.pose)
    if static_clearance < vehicle.minimum_clearance_m - 1e-12:
        cause = (
            DynamicDwaCandidateCause.STATIC_OCCUPANCY
            if physical_clearance <= 1e-12
            else DynamicDwaCandidateCause.STATIC_CLEARANCE
        )
        return cause, static_clearance, None
    if forbidden_entry:
        return DynamicDwaCandidateCause.FORBIDDEN_ZONE, static_clearance, None

    try:
        actor_circles = actor_sampler.sample(point.time_s)
    except ValueError:
        return DynamicDwaCandidateCause.PREDICTION_INVALID, static_clearance, None
    minimum_actor_clearance: float | None = None
    for circle in actor_circles:
        actor_clearance = oriented_footprint_circle_surface_distance(
            point.pose,
            circle_center=(circle.center.x, circle.center.y),
            circle_radius_m=circle.radius_m,
            profile=vehicle,
            inputs_validated=True,
        )
        minimum_actor_clearance = (
            actor_clearance
            if minimum_actor_clearance is None
            else min(minimum_actor_clearance, actor_clearance)
        )
    if minimum_actor_clearance is not None and minimum_actor_clearance < (
        vehicle.minimum_clearance_m - 1e-12
    ):
        return (
            DynamicDwaCandidateCause.ACTOR_TUBE,
            static_clearance,
            minimum_actor_clearance,
        )
    return None, static_clearance, minimum_actor_clearance


def _append_diagnostic_detail(
    details: list[DynamicDwaCandidateDiagnostic],
    detail: DynamicDwaCandidateDiagnostic,
) -> None:
    if len(details) < _DWA_DIAGNOSTIC_DETAIL_LIMIT:
        details.append(detail)


def _shared_gate_failure_cause(
    failures: tuple[str, ...],
) -> DynamicDwaCandidateCause:
    for failure in failures:
        if failure == "forbidden_zone_entry":
            return DynamicDwaCandidateCause.FORBIDDEN_ZONE
        if failure == "static_clearance_below_minimum":
            return DynamicDwaCandidateCause.STATIC_CLEARANCE
        if failure == "actor_clearance_below_minimum":
            return DynamicDwaCandidateCause.ACTOR_TUBE
        if "prediction" in failure or "trajectory_invalid" in failure or "non_finite" in failure:
            return DynamicDwaCandidateCause.PREDICTION_INVALID
    return DynamicDwaCandidateCause.SHARED_GATE


def _input_failure_cause(invalid_reason: str) -> DynamicDwaCandidateCause:
    if invalid_reason == "actor_prediction_missing":
        return DynamicDwaCandidateCause.PREDICTION_INVALID
    return DynamicDwaCandidateCause.SHARED_GATE


def _dynamic_dwa_diagnostic_summary(
    *,
    sampled_candidates: int,
    moving_candidates: int,
    coarse_admissible_candidates: int,
    nonmoving_samples: int,
    counts: Counter[tuple[DynamicDwaCandidatePhase, DynamicDwaCandidateCause]],
    selected: _DynamicCandidate | None,
    selected_rank: int | None,
    details: list[DynamicDwaCandidateDiagnostic],
) -> DynamicDwaDiagnosticSummary:
    ordered_counts = tuple(
        (phase.value, cause.value, counts[(phase, cause)]) for phase, cause in _DWA_COUNT_ORDER
    )
    selected_rank_key = selected.rank if selected is not None else None
    payload = {
        "schema_version": _DWA_DIAGNOSTIC_SCHEMA,
        "sampled_candidates": sampled_candidates,
        "moving_candidates": moving_candidates,
        "coarse_admissible_candidates": coarse_admissible_candidates,
        "nonmoving_samples": nonmoving_samples,
        "ordered_counts": ordered_counts,
        "selected": (
            None
            if selected is None
            else {
                "sample_index": selected.sample_index,
                "command": _twist_payload(selected.command),
                "trajectory": [
                    {
                        "time_s": _float_token(point.time_s),
                        "pose": _pose_payload(point.pose),
                        "twist": _twist_payload(point.twist),
                    }
                    for point in selected.trajectory
                ],
                "score": _float_token(selected.score),
                "rank": tuple(_float_token(value) for value in selected.rank),
                "costs": {
                    "progress": _float_token(selected.progress_cost),
                    "reference_path": _float_token(selected.reference_path_cost),
                    "heading": _float_token(selected.heading_cost),
                    "clearance": _float_token(selected.clearance_cost),
                    "speed": _float_token(selected.speed_cost),
                    "oscillation": _float_token(selected.oscillation_cost),
                },
            }
        ),
        "selected_rank": selected_rank,
        "details": [_diagnostic_detail_payload(detail) for detail in details],
        "exact_phase_granularity": "shared_gate_public_api",
    }
    semantic_digest = sha256(
        dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    return DynamicDwaDiagnosticSummary(
        schema_version=_DWA_DIAGNOSTIC_SCHEMA,
        sampled_candidates=sampled_candidates,
        moving_candidates=moving_candidates,
        coarse_admissible_candidates=coarse_admissible_candidates,
        nonmoving_samples=nonmoving_samples,
        ordered_counts=ordered_counts,
        selected_sample_index=(selected.sample_index if selected is not None else None),
        selected_rank=selected_rank,
        selected_score=selected.score if selected is not None else None,
        selected_rank_key=selected_rank_key,
        details=tuple(details),
        exact_phase_granularity="shared_gate_public_api",
        semantic_digest=semantic_digest,
    )


def _dynamic_dwa_diagnostic_trace(
    summary: DynamicDwaDiagnosticSummary,
) -> tuple[str, ...]:
    counts = ",".join(f"{phase}.{cause}:{count}" for phase, cause, count in summary.ordered_counts)
    selected_sample = (
        "none" if summary.selected_sample_index is None else str(summary.selected_sample_index)
    )
    selected_rank = "none" if summary.selected_rank is None else str(summary.selected_rank)
    return (
        f"diagnostic_schema={summary.schema_version}",
        f"exact_phase_granularity={summary.exact_phase_granularity}",
        "ranking_admissibility_scope=exact_checked_only",
        f"moving_candidates={summary.moving_candidates}",
        f"nonmoving_samples={summary.nonmoving_samples}",
        f"candidate_taxonomy_counts={counts}",
        f"selected_sample_index={selected_sample}",
        f"selected_rank={selected_rank}",
        f"candidate_diagnostic_digest={summary.semantic_digest}",
    )


def _diagnostic_detail_payload(
    detail: DynamicDwaCandidateDiagnostic,
) -> dict[str, object]:
    return {
        "sample_index": detail.sample_index,
        "command": _twist_payload(detail.command),
        "phase": detail.phase.value,
        "cause": detail.cause.value,
        "failure_time_s": (
            None if detail.failure_time_s is None else _float_token(detail.failure_time_s)
        ),
        "minimum_static_clearance_m": (
            None
            if detail.minimum_static_clearance_m is None
            else _float_token(detail.minimum_static_clearance_m)
        ),
        "minimum_actor_clearance_m": (
            None
            if detail.minimum_actor_clearance_m is None
            else _float_token(detail.minimum_actor_clearance_m)
        ),
        "shared_gate_failures": detail.shared_gate_failures,
        "underlying_terminal_cause": (
            None
            if detail.underlying_terminal_cause is None
            else detail.underlying_terminal_cause.value
        ),
    }


def _float_token(value: float) -> str:
    return value.hex()


def _pose_payload(pose: Pose2D) -> tuple[str, str, str]:
    return (_float_token(pose.x), _float_token(pose.y), _float_token(pose.yaw))


def _twist_payload(twist: Twist2D) -> tuple[str, str]:
    return (_float_token(twist.linear), _float_token(twist.angular))


def _dynamic_constant_rollout(
    start: Pose2D,
    command: Twist2D,
    *,
    horizon_s: float,
    step_s: float,
) -> tuple[TrajectoryPoint, ...]:
    steps = int(round(horizon_s / step_s))
    pose = start
    points = [TrajectoryPoint(0.0, pose, command)]
    if abs(command.angular) <= 1e-12:
        delta_x = command.linear * cos(pose.yaw) * step_s
        delta_y = command.linear * sin(pose.yaw) * step_s
        for step in range(1, steps + 1):
            pose = Pose2D(
                x=pose.x + delta_x,
                y=pose.y + delta_y,
                yaw=pose.yaw,
            )
            points.append(TrajectoryPoint(step * step_s, pose, command))
        return tuple(points)

    delta_yaw = command.angular * step_s
    radius = command.linear / command.angular
    for step in range(1, steps + 1):
        next_yaw = pose.yaw + delta_yaw
        pose = Pose2D(
            x=pose.x + radius * (sin(next_yaw) - sin(pose.yaw)),
            y=pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
            yaw=_normalize_angle(next_yaw),
        )
        points.append(TrajectoryPoint(step * step_s, pose, command))
    return tuple(points)


def _dynamic_terminal_rollout(
    start: TrajectoryPoint,
    *,
    linear_deceleration_mps2: float,
    angular_deceleration_radps2: float,
    step_s: float,
) -> tuple[TrajectoryPoint, ...]:
    pose = start.pose
    twist = start.twist
    elapsed_s = 0.0
    points = [TrajectoryPoint(0.0, pose, twist)]
    while abs(twist.linear) > 1e-12 or abs(twist.angular) > 1e-12:
        pose = _integrate_pose(pose, twist, step_s)
        twist = Twist2D(
            linear=_toward_zero(twist.linear, linear_deceleration_mps2 * step_s),
            angular=_toward_zero(twist.angular, angular_deceleration_radps2 * step_s),
        )
        elapsed_s += step_s
        points.append(TrajectoryPoint(elapsed_s, pose, twist))
    return tuple(points)


def _integrate_pose(pose: Pose2D, command: Twist2D, dt_s: float) -> Pose2D:
    if abs(command.angular) <= 1e-12:
        return Pose2D(
            x=pose.x + command.linear * cos(pose.yaw) * dt_s,
            y=pose.y + command.linear * sin(pose.yaw) * dt_s,
            yaw=pose.yaw,
        )
    next_yaw = pose.yaw + command.angular * dt_s
    radius = command.linear / command.angular
    return Pose2D(
        x=pose.x + radius * (sin(next_yaw) - sin(pose.yaw)),
        y=pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
        yaw=_normalize_angle(next_yaw),
    )


def _toward_zero(value: float, delta: float) -> float:
    if value > 0.0:
        return max(0.0, value - delta)
    if value < 0.0:
        return min(0.0, value + delta)
    return 0.0


def _samples_with_zero(start: float, stop: float, count: int) -> tuple[float, ...]:
    samples = list(_linspace(start, stop, count))
    if start <= 0.0 <= stop:
        closest = min(range(len(samples)), key=lambda index: (abs(samples[index]), index))
        samples[closest] = 0.0
        samples.sort()
    return tuple(samples)


def _mean_trajectory_polyline_distance(
    trajectory: tuple[TrajectoryPoint, ...],
    reference_segments: tuple[tuple[Pose2D, Pose2D, float, float, float], ...],
) -> float:
    if len(reference_segments) == 1:
        segment = reference_segments[0]
        return sum(
            _point_to_prepared_segment_distance(point.pose, segment)
            for point in trajectory
        ) / len(trajectory)
    return sum(
        min(
            _point_to_prepared_segment_distance(point.pose, segment)
            for segment in reference_segments
        )
        for point in trajectory
    ) / len(trajectory)


def _prepare_reference_segments(
    reference_path: tuple[Pose2D, ...],
) -> tuple[tuple[Pose2D, Pose2D, float, float, float], ...]:
    prepared: list[tuple[Pose2D, Pose2D, float, float, float]] = []
    for source, target in zip(reference_path, reference_path[1:], strict=False):
        dx = target.x - source.x
        dy = target.y - source.y
        prepared.append((source, target, dx, dy, dx * dx + dy * dy))
    return tuple(prepared)


def _point_to_prepared_segment_distance(
    point: Pose2D,
    segment: tuple[Pose2D, Pose2D, float, float, float],
) -> float:
    source, _target, dx, dy, length_sq = segment
    if length_sq <= 1e-15:
        return _distance(point, source)
    fraction = _clip(
        ((point.x - source.x) * dx + (point.y - source.y) * dy) / length_sq,
        0.0,
        1.0,
    )
    projection_x = source.x + fraction * dx
    projection_y = source.y + fraction * dy
    return hypot(point.x - projection_x, point.y - projection_y)


def _point_to_segment_distance(point: Pose2D, source: Pose2D, target: Pose2D) -> float:
    dx = target.x - source.x
    dy = target.y - source.y
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-15:
        return _distance(point, source)
    fraction = _clip(
        ((point.x - source.x) * dx + (point.y - source.y) * dy) / length_sq,
        0.0,
        1.0,
    )
    projection_x = source.x + fraction * dx
    projection_y = source.y + fraction * dy
    return hypot(point.x - projection_x, point.y - projection_y)


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _invalid_input_reason(
    grid: GridMap,
    reference_path: tuple[Pose2D, ...],
    robot_state: RobotState,
    goal: Pose2D,
    vehicle: VehicleProfile,
) -> str | None:
    if not _pose_is_finite(robot_state.pose) or not all(
        isfinite(value) for value in (robot_state.twist.linear, robot_state.twist.angular)
    ):
        return "robot_state_non_finite"
    if not _pose_is_finite(goal):
        return "goal_non_finite"
    if not reference_path:
        return "reference_path_empty"
    if any(not _pose_is_finite(pose) for pose in reference_path):
        return "reference_path_non_finite"

    start_cell = grid.world_to_cell(robot_state.pose)
    goal_cell = grid.world_to_cell(goal)
    if not grid.in_bounds(start_cell):
        return "start_out_of_bounds"
    if not grid.in_bounds(goal_cell):
        return "goal_out_of_bounds"
    if grid.is_occupied(start_cell):
        return "start_occupied"
    if grid.is_occupied(goal_cell):
        return "goal_occupied"
    if not (
        -vehicle.max_reverse_speed_mps
        <= robot_state.twist.linear
        <= vehicle.max_forward_speed_mps
    ) or abs(robot_state.twist.angular) > vehicle.max_angular_speed_radps:
        return "robot_twist_outside_vehicle_limits"
    return None


def _pose_is_finite(pose: Pose2D) -> bool:
    return all(isfinite(value) for value in (pose.x, pose.y, pose.yaw))


def _linspace(start: float, stop: float, count: int) -> tuple[float, ...]:
    if count == 1:
        return (start,)
    step = (stop - start) / (count - 1)
    return tuple(start + index * step for index in range(count))


def _linear_samples_with_stop(start: float, stop: float, count: int) -> tuple[float, ...]:
    samples = list(_linspace(start, stop, count))
    if start <= 0.0 <= stop:
        closest = min(range(len(samples)), key=lambda index: (abs(samples[index]), index))
        samples[closest] = 0.0
        samples.sort()
    return tuple(samples)


def _distance(source: Pose2D, target: Pose2D) -> float:
    return hypot(source.x - target.x, source.y - target.y)


def _mean_reference_distance(
    path: tuple[Pose2D, ...],
    reference_path: tuple[Pose2D, ...],
) -> float:
    distances = (
        min(_distance(pose, reference) for reference in reference_path) for pose in path
    )
    return sum(distances) / len(path)


def _certified_actor_dominated_clearance(
    trajectory: tuple[TrajectoryPoint, ...],
    *,
    combined_checker: CollisionChecker,
    vehicle: VehicleProfile,
    actor_sampler: _StepActorTubeSampler,
    preserve_rejection_detail: bool,
) -> _CoarseCandidateEvaluation | None:
    """Skip exact static geometry only when a proof-safe lower bound dominates.

    This is step-local screening, not a cached controller result.  If the
    conservative proof is insufficient, the caller runs the historical exact
    path unchanged.  Bounded rejection details deliberately use that exact
    path so their semantic digest remains stable.
    """

    minimum_actor_clearance = inf
    minimum_actor_witnesses: list[tuple[Pose2D, ActorTubeCircle]] = []
    evaluated_poses: list[Pose2D] = []
    half_length = vehicle.collision_length_m / 2.0
    half_width = vehicle.collision_width_m / 2.0
    terminal = _dynamic_terminal_rollout(
        trajectory[-1],
        linear_deceleration_mps2=vehicle.max_deceleration_mps2,
        angular_deceleration_radps2=DYNAMIC_ANGULAR_DECELERATION_RADPS2,
        step_s=0.05,
    )
    terminal_points = tuple(
        TrajectoryPoint(
            time_s=trajectory[-1].time_s + point.time_s,
            pose=point.pose,
            twist=point.twist,
        )
        for point in terminal[1:]
    )
    phased_points = (
        (DynamicDwaCandidatePhase.COARSE_ROLLOUT, trajectory),
        (DynamicDwaCandidatePhase.COARSE_TERMINAL, terminal_points),
    )

    # The historical configuration-grid occupancy check owns precedence over
    # Actor-tube screening.  Inspect the complete rollout and stopping tail
    # before using the shortcut so a later boundary/static/forbidden hit falls
    # back to the unchanged exact path instead of being masked by an earlier
    # Actor encounter.
    configuration_grid = combined_checker.configuration_grid
    if any(
        configuration_grid.is_occupied(
            configuration_grid.world_to_cell(point.pose)
        )
        for _phase, points in phased_points
        for point in points
    ):
        return None

    for phase, points in phased_points:
        for point in points:
            evaluated_poses.append(point.pose)
            try:
                actor_circles = actor_sampler.sample(point.time_s)
            except ValueError:
                minimum_static_lower_bound = (
                    combined_checker.certified_minimum_clearance_lower_bound(
                        tuple(evaluated_poses)
                    )
                )
                if minimum_static_lower_bound < vehicle.minimum_clearance_m:
                    return None
                if preserve_rejection_detail:
                    return None
                return _CoarseCandidateEvaluation(
                    None,
                    failure_phase=phase,
                    failure_cause=(
                        DynamicDwaCandidateCause.TERMINAL_STOPPING
                        if phase is DynamicDwaCandidatePhase.COARSE_TERMINAL
                        else DynamicDwaCandidateCause.PREDICTION_INVALID
                    ),
                    failure_time_s=point.time_s,
                    minimum_actor_clearance_m=(
                        None
                        if minimum_actor_clearance == inf
                        else minimum_actor_clearance
                    ),
                    underlying_terminal_cause=(
                        DynamicDwaCandidateCause.PREDICTION_INVALID
                        if phase is DynamicDwaCandidatePhase.COARSE_TERMINAL
                        else None
                    ),
                    used_certified_actor_dominance=True,
                )
            point_actor_clearance = inf
            point_actor_circles: list[ActorTubeCircle] = []
            threshold_guard = 0.0
            for circle in actor_circles:
                delta_x = circle.center.x - point.pose.x
                delta_y = circle.center.y - point.pose.y
                cosine = cos(point.pose.yaw)
                sine = sin(point.pose.yaw)
                local_x = cosine * delta_x + sine * delta_y
                local_y = -sine * delta_x + cosine * delta_y
                outside_x = max(abs(local_x) - half_length, 0.0)
                outside_y = max(abs(local_y) - half_width, 0.0)
                actor_clearance = (
                    -circle.radius_m
                    if outside_x == 0.0 and outside_y == 0.0
                    else hypot(outside_x, outside_y) - circle.radius_m
                )
                witness_tolerance = 1e-9 * max(
                    1.0,
                    abs(point.pose.x),
                    abs(point.pose.y),
                    abs(circle.center.x),
                    abs(circle.center.y),
                    circle.radius_m,
                )
                threshold_guard = max(threshold_guard, witness_tolerance)
                if actor_clearance < point_actor_clearance - witness_tolerance:
                    point_actor_clearance = actor_clearance
                    point_actor_circles = [circle]
                elif actor_clearance <= point_actor_clearance + witness_tolerance:
                    point_actor_circles.append(circle)

                if actor_clearance < minimum_actor_clearance - witness_tolerance:
                    minimum_actor_clearance = actor_clearance
                    minimum_actor_witnesses = [(point.pose, circle)]
                elif actor_clearance <= minimum_actor_clearance + witness_tolerance:
                    minimum_actor_witnesses.append((point.pose, circle))

            actor_threshold = vehicle.minimum_clearance_m - 1e-12
            if point_actor_circles and (
                abs(point_actor_clearance - actor_threshold) <= threshold_guard
            ):
                point_actor_clearance = min(
                    oriented_footprint_circle_surface_distance(
                        point.pose,
                        circle_center=(circle.center.x, circle.center.y),
                        circle_radius_m=circle.radius_m,
                        profile=vehicle,
                        inputs_validated=True,
                    )
                    for circle in actor_circles
                )
            if point_actor_clearance < vehicle.minimum_clearance_m - 1e-12:
                minimum_static_lower_bound = (
                    combined_checker.certified_minimum_clearance_lower_bound(
                        tuple(evaluated_poses)
                    )
                )
                if minimum_static_lower_bound < vehicle.minimum_clearance_m:
                    return None
                if preserve_rejection_detail:
                    return None
                return _CoarseCandidateEvaluation(
                    None,
                    failure_phase=phase,
                    failure_cause=(
                        DynamicDwaCandidateCause.TERMINAL_STOPPING
                        if phase is DynamicDwaCandidatePhase.COARSE_TERMINAL
                        else DynamicDwaCandidateCause.ACTOR_TUBE
                    ),
                    failure_time_s=point.time_s,
                    minimum_actor_clearance_m=minimum_actor_clearance,
                    underlying_terminal_cause=(
                        DynamicDwaCandidateCause.ACTOR_TUBE
                        if phase is DynamicDwaCandidatePhase.COARSE_TERMINAL
                        else None
                    ),
                    used_certified_actor_dominance=True,
                )

    if minimum_actor_clearance == inf:
        return None
    minimum_actor_clearance = inf
    for pose, circle in minimum_actor_witnesses:
        exact_actor_clearance = oriented_footprint_circle_surface_distance(
            pose,
            circle_center=(circle.center.x, circle.center.y),
            circle_radius_m=circle.radius_m,
            profile=vehicle,
            inputs_validated=True,
        )
        if exact_actor_clearance < minimum_actor_clearance:
            minimum_actor_clearance = exact_actor_clearance
    minimum_static_lower_bound = (
        combined_checker.certified_minimum_clearance_lower_bound(
            tuple(evaluated_poses)
        )
    )
    # A strict numeric margin keeps the proof on the conservative side of any
    # last-bit equality in the lower-bound construction.
    if minimum_static_lower_bound < minimum_actor_clearance + 1e-12:
        return None
    return _CoarseCandidateEvaluation(
        minimum_actor_clearance,
        minimum_actor_clearance_m=minimum_actor_clearance,
        used_certified_actor_dominance=True,
    )




def _heading_error(pose: Pose2D, goal: Pose2D) -> float:
    desired = atan2(goal.y - pose.y, goal.x - pose.x)
    return abs(_normalize_angle(desired - pose.yaw))


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def _sweep_distance(
    start: Pose2D,
    linear: float,
    angular: float,
    distance: float,
    *,
    step_m: float,
) -> tuple[Pose2D, ...]:
    """현재 곡률을 유지한 채 주어진 호 길이까지의 보수적 정지 sweep를 만든다."""

    if distance <= 0.0 or abs(linear) <= 1e-12:
        return (start,)
    steps = max(1, int(distance / step_m + 0.999999999))
    arc_step = copysign(distance / steps, linear)
    curvature = angular / linear
    x, y, yaw = start.x, start.y, start.yaw
    poses = [start]
    for _ in range(steps):
        if abs(curvature) <= 1e-12:
            x += arc_step * cos(yaw)
            y += arc_step * sin(yaw)
        else:
            next_yaw = yaw + curvature * arc_step
            radius = 1.0 / curvature
            x += radius * (sin(next_yaw) - sin(yaw))
            y -= radius * (cos(next_yaw) - cos(yaw))
            yaw = next_yaw
        yaw = _normalize_angle(yaw)
        poses.append(Pose2D(x=x, y=y, yaw=yaw))
    return tuple(poses)


def _normalized(value: float, lower: float, upper: float) -> float:
    if upper - lower <= 1e-15:
        return 0.0
    return (value - lower) / (upper - lower)


def _select_candidate(
    candidates: list[_Candidate],
    maximum_speed: float,
) -> tuple[_Candidate, float]:
    progress_values = [candidate.progress for candidate in candidates]
    reference_values = [candidate.reference_distance for candidate in candidates]
    heading_values = [candidate.heading_error for candidate in candidates]
    clearance_values = [candidate.minimum_clearance for candidate in candidates]

    progress_bounds = min(progress_values), max(progress_values)
    reference_bounds = min(reference_values), max(reference_values)
    heading_bounds = min(heading_values), max(heading_values)
    clearance_bounds = min(clearance_values), max(clearance_values)

    ranked: list[tuple[tuple[float, ...], _Candidate, float]] = []
    for candidate in candidates:
        progress_cost = 1.0 - _normalized(candidate.progress, *progress_bounds)
        reference_cost = _normalized(candidate.reference_distance, *reference_bounds)
        heading_cost = _normalized(candidate.heading_error, *heading_bounds)
        clearance_cost = 1.0 - _normalized(candidate.minimum_clearance, *clearance_bounds)
        speed_cost = 1.0 - abs(candidate.linear) / maximum_speed
        score = (
            1.0 * progress_cost
            + 1.0 * reference_cost
            + 0.5 * heading_cost
            + 1.5 * clearance_cost
            + 0.2 * speed_cost
        )
        tie_break = (
            score,
            progress_cost,
            reference_cost,
            heading_cost,
            clearance_cost,
            speed_cost,
            abs(candidate.angular),
            candidate.angular,
            candidate.linear,
        )
        ranked.append((tie_break, candidate, score))
    _, best, score = min(ranked, key=lambda item: item[0])
    return best, score


def _twist_is_stopped(twist: Twist2D, *, tolerance: float = 1e-9) -> bool:
    return abs(twist.linear) <= tolerance and abs(twist.angular) <= tolerance
