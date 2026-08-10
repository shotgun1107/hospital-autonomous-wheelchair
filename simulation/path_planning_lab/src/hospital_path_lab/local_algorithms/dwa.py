"""가상 축소 차체용 결정론적 Dynamic Window Approach 기준 구현."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, copysign, cos, hypot, isfinite, pi, sin
from time import perf_counter_ns

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import (
    GridSnapshot,
    LocalPlanResult,
    PlanStatus,
    Pose2D,
    RobotState,
    TrajectoryPoint,
    Twist2D,
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
