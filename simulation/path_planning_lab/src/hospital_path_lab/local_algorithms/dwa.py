"""가상 축소 차체용 결정론적 Dynamic Window Approach 기준 구현."""

from __future__ import annotations

from dataclasses import dataclass
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
from hospital_path_lab.dynamic_prediction import sample_actor_tubes
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
    ) -> None:
        if not vehicle.simulation_only:
            raise ValueError("dynamic DWA requires a simulation-only vehicle profile")
        self.vehicle = vehicle

    def step(self, snapshot: ControllerSnapshot) -> ControllerCommandResult:
        started_at = perf_counter_ns()
        invalid_reason = self._invalid_reason(snapshot)
        if invalid_reason is not None:
            return _dynamic_controller_result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.INVALID_INPUT,
                failure_reason=invalid_reason,
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
            return _dynamic_controller_result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.FOUND,
                predicted_trajectory=trajectory,
                decision_trace=("goal_reached=true", "sampled_candidates=0"),
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
        )
        combined_checker = CollisionChecker(
            snapshot.static_grid_snapshot.grid,
            self.vehicle,
            forbidden_cells=snapshot.static_grid_snapshot.forbidden_cells,
        )
        candidates: list[_DynamicCandidate] = []
        for linear in linear_values:
            for angular in angular_values:
                if linear <= 1e-12:
                    continue
                command = Twist2D(linear=linear, angular=angular)
                trajectory = _dynamic_constant_rollout(
                    apply_end,
                    command,
                    horizon_s=self.horizon_s,
                    step_s=self.integration_dt_s,
                )
                minimum_clearance = _coarse_dynamic_candidate_clearance(
                    trajectory,
                    snapshot=snapshot,
                    physical_checker=physical_checker,
                    combined_checker=combined_checker,
                    vehicle=self.vehicle,
                )
                if minimum_clearance is None:
                    continue
                candidates.append(
                    _dynamic_candidate(
                        command,
                        trajectory,
                        start=snapshot.robot_state.pose,
                        goal=snapshot.goal_pose,
                        reference_path=snapshot.reference_path,
                        minimum_clearance=minimum_clearance,
                        previous_angular=snapshot.robot_state.twist.angular,
                    )
                )

        candidates.sort(key=lambda candidate: candidate.rank)
        selected: _DynamicCandidate | None = None
        for candidate in candidates:
            proposal = _dynamic_proposal(snapshot, candidate.command, candidate.trajectory)
            evidence = evaluate_dynamic_trajectory_safety(
                proposal,
                robot_state=snapshot.robot_state,
                grid_snapshot=snapshot.static_grid_snapshot,
                prediction_set=snapshot.actor_tubes,
                profile=self.vehicle,
            )
            if evidence.safe:
                selected = candidate
                break

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
    return ControllerCommandResult(
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
) -> _DynamicCandidate:
    progress = _distance(start, goal) - _distance(trajectory[-1].pose, goal)
    progress_cost = 1.0 - _clip(progress / 0.40, 0.0, 1.0)
    reference_distance = _mean_polyline_distance(
        tuple(point.pose for point in trajectory),
        reference_path,
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
    )


def _coarse_dynamic_candidate_clearance(
    trajectory: tuple[TrajectoryPoint, ...],
    *,
    snapshot: ControllerSnapshot,
    physical_checker: CollisionChecker,
    combined_checker: CollisionChecker,
    vehicle: VehicleProfile,
) -> float | None:
    """50 ms DWA sampling prefilter; 선택 후보는 공통 5 ms gate로 다시 검사한다."""

    if snapshot.actor_tubes is None:
        return None
    minimum_clearance = inf
    terminal = _dynamic_terminal_rollout(
        trajectory[-1],
        linear_deceleration_mps2=vehicle.max_deceleration_mps2,
        angular_deceleration_radps2=DYNAMIC_ANGULAR_DECELERATION_RADPS2,
        step_s=0.05,
    )
    timed_points = tuple(trajectory) + tuple(
        TrajectoryPoint(
            time_s=trajectory[-1].time_s + point.time_s,
            pose=point.pose,
            twist=point.twist,
        )
        for point in terminal[1:]
    )
    configuration_grid = combined_checker.configuration_grid
    for point in timed_points:
        configuration_cell = configuration_grid.world_to_cell(point.pose)
        if configuration_grid.is_occupied(configuration_cell):
            return None
        static_clearance = min(
            physical_checker.clearance(point.pose),
            combined_checker.clearance(point.pose),
        )
        if (
            static_clearance < vehicle.minimum_clearance_m - 1e-12
            or combined_checker.pose_enters_forbidden(point.pose)
        ):
            return None
        minimum_clearance = min(minimum_clearance, static_clearance)
        try:
            actor_circles = sample_actor_tubes(
                snapshot.actor_tubes,
                rollout_time_s=point.time_s,
            )
        except ValueError:
            return None
        for circle in actor_circles:
            actor_clearance = oriented_footprint_circle_surface_distance(
                point.pose,
                circle_center=(circle.center.x, circle.center.y),
                circle_radius_m=circle.radius_m,
                profile=vehicle,
            )
            if actor_clearance < vehicle.minimum_clearance_m - 1e-12:
                return None
            minimum_clearance = min(minimum_clearance, actor_clearance)
    return minimum_clearance


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
    for step in range(1, steps + 1):
        pose = _integrate_pose(pose, command, step_s)
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


def _mean_polyline_distance(
    path: tuple[Pose2D, ...],
    reference_path: tuple[Pose2D, ...],
) -> float:
    return sum(
        min(
            _point_to_segment_distance(pose, source, target)
            for source, target in zip(reference_path, reference_path[1:], strict=False)
        )
        for pose in path
    ) / len(path)


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
    projection = Pose2D(source.x + fraction * dx, source.y + fraction * dy)
    return _distance(point, projection)


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
