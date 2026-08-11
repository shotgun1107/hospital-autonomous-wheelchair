"""Pure Pursuit와 곡률 감속을 더한 연구용 RPP 추종기."""

from __future__ import annotations

from math import atan2, cos, hypot, isfinite, sin, sqrt
from time import perf_counter_ns

from hospital_path_lab.contracts import (
    FollowerResult,
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    TrajectoryPoint,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_COMMAND_APPLY_LATENCY_S,
    ControllerCommandResult,
    ControllerSnapshot,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile

_GOAL_TOLERANCE_M = 0.05
_PURE_PURSUIT_LOOKAHEAD_M = 0.35
_RPP_MIN_LOOKAHEAD_M = 0.25
_RPP_MAX_LOOKAHEAD_M = 0.50
_RPP_LOOKAHEAD_VELOCITY_GAIN = 0.75
_RPP_MIN_SPEED_MPS = 0.05
_RPP_CURVATURE_GAIN = 2.0
_DYNAMIC_ROLLOUT_HORIZON_S = 2.0
_DYNAMIC_ROLLOUT_STEP_S = 0.05
_DYNAMIC_MAX_ANGULAR_ACCELERATION_RADPS2 = 1.60


class PurePursuitFollower:
    name = "pure_pursuit"

    def __init__(
        self,
        vehicle_profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    ) -> None:
        self.vehicle_profile = vehicle_profile

    def step(
        self,
        path: tuple[Pose2D, ...],
        robot_state: RobotState,
        metadata: SnapshotMetadata,
    ) -> FollowerResult:
        return _follow(
            follower_name=self.name,
            path=path,
            robot_state=robot_state,
            metadata=metadata,
            vehicle_profile=self.vehicle_profile,
            lookahead_m=_PURE_PURSUIT_LOOKAHEAD_M,
            regulated=False,
        )


class RegulatedPurePursuitFollower:
    name = "rpp"

    def __init__(
        self,
        vehicle_profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    ) -> None:
        self.vehicle_profile = vehicle_profile

    def step(
        self,
        path: tuple[Pose2D, ...],
        robot_state: RobotState,
        metadata: SnapshotMetadata,
    ) -> FollowerResult:
        lookahead_m = _clip(
            _RPP_MIN_LOOKAHEAD_M
            + _RPP_LOOKAHEAD_VELOCITY_GAIN * abs(robot_state.twist.linear),
            _RPP_MIN_LOOKAHEAD_M,
            _RPP_MAX_LOOKAHEAD_M,
        )
        return _follow(
            follower_name=self.name,
            path=path,
            robot_state=robot_state,
            metadata=metadata,
            vehicle_profile=self.vehicle_profile,
            lookahead_m=lookahead_m,
            regulated=True,
        )


class DynamicPurePursuitController:
    """v5 PP 명령을 gate-compatible post-apply rollout으로 변환한다."""

    name = "dynamic_pure_pursuit"

    def __init__(
        self,
        vehicle_profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    ) -> None:
        if not vehicle_profile.simulation_only:
            raise ValueError("dynamic PP requires a simulation-only vehicle profile")
        self.vehicle_profile = vehicle_profile
        self._follower = PurePursuitFollower(vehicle_profile)

    def step(self, snapshot: ControllerSnapshot) -> ControllerCommandResult:
        started_at = perf_counter_ns()
        if snapshot.vehicle_profile != self.vehicle_profile:
            return _dynamic_result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.INVALID_INPUT,
                failure_reason="vehicle_profile_mismatch",
                controller_requested_stop=True,
            )
        output = self._follower.step(
            snapshot.reference_path,
            snapshot.robot_state,
            snapshot.static_grid_snapshot.metadata,
        )
        if output.status is not PlanStatus.FOUND:
            return _dynamic_result(
                self.name,
                snapshot,
                started_at,
                status=output.status,
                failure_reason=output.failure_reason,
                controller_requested_stop=True,
            )

        angular = _rate_limited_angular_speed(
            output.command.angular,
            snapshot.robot_state.twist.angular,
            self.vehicle_profile,
        )
        command = Twist2D(linear=output.command.linear, angular=angular)
        apply_end = _integrate_pose(
            snapshot.robot_state.pose,
            snapshot.robot_state.twist,
            DYNAMIC_COMMAND_APPLY_LATENCY_S,
        )
        trajectory = _constant_rollout(
            apply_end,
            command,
            horizon_s=_DYNAMIC_ROLLOUT_HORIZON_S,
            step_s=_DYNAMIC_ROLLOUT_STEP_S,
        )
        remaining = _remaining_arc_length(
            snapshot.reference_path,
            snapshot.robot_state.pose,
        )
        trace = (
            f"remaining_arc_length_m={remaining:.12g}",
            f"lookahead_x={output.lookahead_point.x:.12g}",
            f"lookahead_y={output.lookahead_point.y:.12g}",
            "detour=false",
        ) if output.lookahead_point is not None else (
            f"remaining_arc_length_m={remaining:.12g}",
            "detour=false",
        )
        return _dynamic_result(
            self.name,
            snapshot,
            started_at,
            status=PlanStatus.FOUND,
            requested_twist=command,
            predicted_trajectory=trajectory,
            decision_trace=trace,
        )
def _follow(
    *,
    follower_name: str,
    path: tuple[Pose2D, ...],
    robot_state: RobotState,
    metadata: SnapshotMetadata,
    vehicle_profile: VehicleProfile,
    lookahead_m: float,
    regulated: bool,
) -> FollowerResult:
    started_at = perf_counter_ns()
    if not metadata.input_valid:
        return _result(
            follower_name,
            metadata,
            started_at,
            status=PlanStatus.INVALID_INPUT,
            failure_reason="snapshot_input_invalidated",
        )
    invalid_reason = _invalid_reason(path, robot_state, vehicle_profile)
    if invalid_reason is not None:
        return _result(
            follower_name,
            metadata,
            started_at,
            status=PlanStatus.INVALID_INPUT,
            failure_reason=invalid_reason,
        )

    goal = path[-1]
    goal_distance = hypot(goal.x - robot_state.pose.x, goal.y - robot_state.pose.y)
    if goal_distance <= _GOAL_TOLERANCE_M:
        linear_speed = _rate_limited_linear_speed(
            0.0, robot_state.twist.linear, vehicle_profile
        )
        return _result(
            follower_name,
            metadata,
            started_at,
            status=PlanStatus.FOUND,
            command=Twist2D(linear=linear_speed),
            lookahead_point=goal,
        )

    lookahead_point = _lookahead_after_current_position(path, robot_state.pose, lookahead_m)
    curvature = _curvature_to_point(robot_state.pose, lookahead_point)
    remaining_arc_length = _remaining_arc_length(path, robot_state.pose)
    goal_limited_speed = min(
        vehicle_profile.nominal_speed_mps,
        sqrt(
            2.0
            * vehicle_profile.max_deceleration_mps2
            * max(0.0, remaining_arc_length - _GOAL_TOLERANCE_M)
        ),
    )

    nominal_speed = vehicle_profile.nominal_speed_mps
    if regulated:
        desired_linear_speed = _clip(
            nominal_speed / (1.0 + _RPP_CURVATURE_GAIN * abs(curvature)),
            _RPP_MIN_SPEED_MPS,
            nominal_speed,
        )
        desired_linear_speed = min(desired_linear_speed, goal_limited_speed)
    else:
        desired_linear_speed = goal_limited_speed
    linear_speed = _rate_limited_linear_speed(
        desired_linear_speed, robot_state.twist.linear, vehicle_profile
    )
    angular_speed = _clip(
        linear_speed * curvature,
        -vehicle_profile.max_angular_speed_radps,
        vehicle_profile.max_angular_speed_radps,
    )

    return _result(
        follower_name,
        metadata,
        started_at,
        status=PlanStatus.FOUND,
        command=Twist2D(linear=linear_speed, angular=angular_speed),
        lookahead_point=lookahead_point,
    )


def _invalid_reason(
    path: tuple[Pose2D, ...],
    robot_state: RobotState,
    vehicle_profile: VehicleProfile,
) -> str | None:
    if not path:
        return "empty_path"
    if any(not all(isfinite(value) for value in (pose.x, pose.y, pose.yaw)) for pose in path):
        return "nonfinite_path"
    state_values = (
        robot_state.pose.x,
        robot_state.pose.y,
        robot_state.pose.yaw,
        robot_state.twist.linear,
        robot_state.twist.angular,
    )
    if not all(isfinite(value) for value in state_values):
        return "nonfinite_robot_state"
    if not (
        -vehicle_profile.max_reverse_speed_mps
        <= robot_state.twist.linear
        <= vehicle_profile.max_forward_speed_mps
    ) or abs(robot_state.twist.angular) > vehicle_profile.max_angular_speed_radps:
        return "robot_twist_outside_vehicle_limits"
    return None


def _rate_limited_linear_speed(
    target: float,
    current: float,
    vehicle_profile: VehicleProfile,
) -> float:
    """목표 선속도로 가되 한 제어 주기의 가속·감속 한계를 넘지 않는다."""

    target = _clip(
        target,
        -vehicle_profile.max_reverse_speed_mps,
        vehicle_profile.max_forward_speed_mps,
    )
    period = vehicle_profile.control_period_s
    if current == target:
        return target

    # 부호 전환은 먼저 현재 방향의 속도를 감속해 0에 도달한 다음 시작한다.
    if current * target < 0.0:
        change = min(
            abs(current), vehicle_profile.max_deceleration_mps2 * period
        )
        if change >= abs(current):
            return 0.0
        return current - change if current > 0.0 else current + change

    increasing_magnitude = abs(target) > abs(current)
    rate = (
        vehicle_profile.max_acceleration_mps2
        if increasing_magnitude
        else vehicle_profile.max_deceleration_mps2
    )
    maximum_change = rate * period
    if abs(target - current) <= maximum_change:
        return target
    delta = _clip(target - current, -maximum_change, maximum_change)
    return current + delta


def _lookahead_after_current_position(
    path: tuple[Pose2D, ...],
    current_pose: Pose2D,
    lookahead_m: float,
) -> Pose2D:
    if len(path) == 1:
        return path[0]

    cumulative, best_progress = _path_projection(path, current_pose)
    return _point_at_progress(path, cumulative, min(best_progress + lookahead_m, cumulative[-1]))


def _path_projection(
    path: tuple[Pose2D, ...],
    current_pose: Pose2D,
) -> tuple[list[float], float]:
    cumulative = [0.0]
    for source, target in zip(path, path[1:], strict=False):
        cumulative.append(cumulative[-1] + hypot(target.x - source.x, target.y - source.y))

    best_distance_sq = float("inf")
    best_progress = 0.0
    for index, (source, target) in enumerate(zip(path, path[1:], strict=False)):
        dx = target.x - source.x
        dy = target.y - source.y
        length_sq = dx * dx + dy * dy
        if length_sq == 0.0:
            fraction = 0.0
        else:
            fraction = _clip(
                ((current_pose.x - source.x) * dx + (current_pose.y - source.y) * dy)
                / length_sq,
                0.0,
                1.0,
            )
        projected_x = source.x + fraction * dx
        projected_y = source.y + fraction * dy
        distance_sq = (current_pose.x - projected_x) ** 2 + (
            current_pose.y - projected_y
        ) ** 2
        progress = cumulative[index] + fraction * (length_sq**0.5)
        candidate = distance_sq, progress
        if candidate < (best_distance_sq, best_progress):
            best_distance_sq, best_progress = candidate
    return cumulative, best_progress


def _remaining_arc_length(path: tuple[Pose2D, ...], current_pose: Pose2D) -> float:
    if len(path) == 1:
        return hypot(path[0].x - current_pose.x, path[0].y - current_pose.y)
    cumulative, progress = _path_projection(path, current_pose)
    return max(0.0, cumulative[-1] - progress)


def _point_at_progress(
    path: tuple[Pose2D, ...],
    cumulative: list[float],
    target_progress: float,
) -> Pose2D:
    for index, (source, target) in enumerate(zip(path, path[1:], strict=False)):
        segment_start = cumulative[index]
        segment_end = cumulative[index + 1]
        if target_progress > segment_end and index < len(path) - 2:
            continue
        segment_length = segment_end - segment_start
        if segment_length == 0.0:
            continue
        fraction = _clip((target_progress - segment_start) / segment_length, 0.0, 1.0)
        dx = target.x - source.x
        dy = target.y - source.y
        return Pose2D(
            x=source.x + fraction * dx,
            y=source.y + fraction * dy,
            yaw=atan2(dy, dx),
        )
    return path[-1]


def _curvature_to_point(current_pose: Pose2D, target: Pose2D) -> float:
    dx = target.x - current_pose.x
    dy = target.y - current_pose.y
    distance_sq = dx * dx + dy * dy
    if distance_sq == 0.0:
        return 0.0
    lateral_local = -sin(current_pose.yaw) * dx + cos(current_pose.yaw) * dy
    return 2.0 * lateral_local / distance_sq


def _result(
    follower_name: str,
    metadata: SnapshotMetadata,
    started_at: int,
    *,
    status: PlanStatus,
    command: Twist2D | None = None,
    lookahead_point: Pose2D | None = None,
    failure_reason: str | None = None,
) -> FollowerResult:
    return FollowerResult(
        follower=follower_name,
        status=status,
        command=command if command is not None else Twist2D(),
        lookahead_point=lookahead_point,
        elapsed_ns=perf_counter_ns() - started_at,
        map_id=metadata.map_id,
        map_revision=metadata.map_revision,
        mission_revision=metadata.mission_revision,
        observation_revision=metadata.observation_revision,
        input_content_hash=metadata.content_hash,
        failure_reason=failure_reason,
    )


def _dynamic_result(
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
    )


def _rate_limited_angular_speed(
    target: float,
    current: float,
    vehicle_profile: VehicleProfile,
) -> float:
    target = _clip(
        target,
        -vehicle_profile.max_angular_speed_radps,
        vehicle_profile.max_angular_speed_radps,
    )
    maximum_change = (
        _DYNAMIC_MAX_ANGULAR_ACCELERATION_RADPS2 * vehicle_profile.control_period_s
    )
    return current + _clip(target - current, -maximum_change, maximum_change)


def _constant_rollout(
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
        yaw=atan2(sin(next_yaw), cos(next_yaw)),
    )


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
