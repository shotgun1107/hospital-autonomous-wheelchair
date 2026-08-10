from __future__ import annotations

from math import isclose

import numpy as np
import pytest

from hospital_path_lab.contracts import (
    GridSnapshot,
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    Twist2D,
)
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_algorithms.dwa import DynamicWindowPlanner, _sweep_distance
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _snapshot(occupancy: np.ndarray, *, resolution_m: float = 0.05) -> GridSnapshot:
    return GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="dwa_test",
            map_revision=3,
            mission_revision=5,
            observation_revision=8,
            seed=0,
            content_hash="dwa-grid",
        ),
        grid=GridMap(occupancy=occupancy, resolution_m=resolution_m),
    )


def _reference(start: Pose2D, goal: Pose2D, count: int = 9) -> tuple[Pose2D, ...]:
    return tuple(
        Pose2D(
            x=start.x + (goal.x - start.x) * index / (count - 1),
            y=start.y + (goal.y - start.y) * index / (count - 1),
        )
        for index in range(count)
    )


def _plan(
    snapshot: GridSnapshot,
    start: Pose2D,
    goal: Pose2D,
    *,
    twist: Twist2D | None = None,
):
    return DynamicWindowPlanner().plan(
        snapshot,
        _reference(start, goal),
        RobotState(start, twist or Twist2D()),
        goal,
    )


def test_dwa_selects_straight_progress_in_open_space() -> None:
    snapshot = _snapshot(np.zeros((30, 50), dtype=np.bool_))
    start = Pose2D(0.5, 0.75)
    goal = Pose2D(2.0, 0.75)
    result = _plan(snapshot, start, goal, twist=Twist2D(linear=0.10))

    assert result.status is PlanStatus.FOUND
    assert result.trajectory
    command = result.trajectory[-1].twist
    assert command.linear > 0.0
    assert abs(command.angular) < 1e-12
    assert result.path[-1].x > start.x
    assert isclose(result.trajectory[-1].time_s, 2.0)
    assert result.sampled_trajectories == 7 * 31
    assert result.minimum_clearance is not None
    assert result.collision is False
    assert (result.map_revision, result.mission_revision, result.observation_revision) == (3, 5, 8)


def test_dwa_turns_toward_offset_goal() -> None:
    snapshot = _snapshot(np.zeros((50, 50), dtype=np.bool_))
    start = Pose2D(0.5, 0.5)
    goal = Pose2D(1.8, 1.4)
    result = _plan(snapshot, start, goal, twist=Twist2D(linear=0.10))

    assert result.status is PlanStatus.FOUND
    assert result.trajectory[-1].twist.angular > 0.0
    assert result.path[-1].y > start.y


def test_dwa_stops_when_obstacle_is_inside_stopping_margin() -> None:
    occupancy = np.zeros((35, 50), dtype=np.bool_)
    occupancy[13:18, 16:19] = True
    snapshot = _snapshot(occupancy)
    start = Pose2D(0.5, 0.75)
    goal = Pose2D(2.0, 0.75)
    result = _plan(snapshot, start, goal, twist=Twist2D(linear=0.10))

    assert result.status is PlanStatus.NO_PATH
    assert result.path == ()
    assert result.failure_reason == "no_safe_moving_trajectory"


def test_dwa_keeps_offset_obstacle_outside_footprint() -> None:
    occupancy = np.zeros((35, 50), dtype=np.bool_)
    occupancy[11:14, 26:29] = True
    snapshot = _snapshot(occupancy)
    start = Pose2D(0.5, 0.75)
    goal = Pose2D(2.0, 0.75)
    result = _plan(
        snapshot,
        start,
        goal,
        twist=Twist2D(linear=0.10, angular=0.20),
    )

    assert result.status is PlanStatus.FOUND
    assert result.trajectory
    assert result.minimum_clearance is not None
    assert result.minimum_clearance > 0.0
    assert result.collision is False


def test_dwa_does_not_confuse_lateral_corridor_clearance_with_stopping_distance() -> None:
    occupancy = np.ones((30, 100), dtype=np.bool_)
    occupancy[5:21, :] = False  # 0.05m 해상도에서 폭 0.80m 복도
    snapshot = _snapshot(occupancy)
    start = Pose2D(0.5, 0.65)
    goal = Pose2D(1.8, 0.65)

    result = _plan(snapshot, start, goal)

    assert result.status is PlanStatus.FOUND
    assert result.minimum_clearance is not None
    assert result.minimum_clearance >= VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m


def test_dwa_rejects_start_footprint_that_is_already_in_collision() -> None:
    occupancy = np.ones((9, 9), dtype=np.bool_)
    occupancy[4, 4] = False
    occupancy[4, 6] = False
    snapshot = _snapshot(occupancy)
    start = Pose2D(0.225, 0.225)
    goal = Pose2D(0.325, 0.225)
    result = _plan(snapshot, start, goal)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.path == ()
    assert result.trajectory == ()
    assert result.cost is None
    assert result.sampled_trajectories == 0
    assert result.collision is False
    assert result.minimum_clearance is None
    assert result.failure_reason == "start_footprint_occupied"


def test_dwa_rejects_goal_footprint_that_is_already_in_collision() -> None:
    occupancy = np.zeros((60, 60), dtype=np.bool_)
    occupancy[20, 44] = True
    snapshot = _snapshot(occupancy)
    start = Pose2D(1.0, 1.0)
    goal = Pose2D(2.0, 1.0)

    result = _plan(snapshot, start, goal)

    assert not snapshot.grid.is_occupied(snapshot.grid.world_to_cell(goal))
    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "goal_footprint_occupied"
    assert result.sampled_trajectories == 0


def test_dwa_command_stays_inside_one_period_dynamic_window() -> None:
    snapshot = _snapshot(np.zeros((40, 80), dtype=np.bool_))
    start = Pose2D(0.5, 1.0)
    goal = Pose2D(3.0, 1.4)
    current = Twist2D(linear=0.20, angular=0.30)
    result = _plan(snapshot, start, goal, twist=current)

    assert result.status is PlanStatus.FOUND
    command = result.trajectory[-1].twist
    profile = VIRTUAL_DOLL_WHEELCHAIR_V0_1
    period = profile.control_period_s
    assert command.linear == 0.0 or (
        current.linear - profile.max_deceleration_mps2 * period
        <= command.linear
        <= current.linear + profile.max_acceleration_mps2 * period
    )
    assert current.angular - 1.6 * period <= command.angular <= current.angular + 1.6 * period
    assert -profile.max_reverse_speed_mps <= command.linear


def test_dwa_is_deterministic_except_for_elapsed_time() -> None:
    snapshot = _snapshot(np.zeros((30, 50), dtype=np.bool_))
    start = Pose2D(0.5, 0.75)
    goal = Pose2D(2.0, 1.0)
    signatures = []
    for _ in range(10):
        result = _plan(snapshot, start, goal, twist=Twist2D(linear=0.1))
        signatures.append(
            (
                result.status,
                result.path,
                result.trajectory,
                result.cost,
                result.sampled_trajectories,
                result.minimum_clearance,
            )
        )

    assert len(set(signatures)) == 1


def test_dwa_rejects_empty_reference_path() -> None:
    snapshot = _snapshot(np.zeros((20, 20), dtype=np.bool_))
    result = DynamicWindowPlanner().plan(
        snapshot,
        (),
        RobotState(Pose2D(0.5, 0.5)),
        Pose2D(0.8, 0.5),
    )

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "reference_path_empty"
    assert result.sampled_trajectories == 0


@pytest.mark.parametrize(
    "twist",
    [
        Twist2D(linear=0.301),
        Twist2D(linear=-0.101),
        Twist2D(angular=0.801),
        Twist2D(angular=-0.801),
    ],
)
def test_dwa_rejects_current_twist_outside_vehicle_limits(twist: Twist2D) -> None:
    snapshot = _snapshot(np.zeros((50, 50), dtype=np.bool_))
    result = _plan(snapshot, Pose2D(1.0, 1.0), Pose2D(2.0, 1.0), twist=twist)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "robot_twist_outside_vehicle_limits"
    assert result.sampled_trajectories == 0


def test_dwa_dynamic_window_contains_reverse_and_in_place_rotation_from_rest() -> None:
    planner = DynamicWindowPlanner()
    linear_values, angular_values = planner._dynamic_window(
        RobotState(Pose2D(1.0, 1.0), Twist2D())
    )

    assert min(linear_values) < 0.0 < max(linear_values)
    assert 0.0 in linear_values
    assert min(angular_values) < 0.0 < max(angular_values)


def test_dwa_goal_behind_returns_active_safe_candidate() -> None:
    snapshot = _snapshot(np.zeros((70, 70), dtype=np.bool_))
    start = Pose2D(2.0, 1.5, 0.0)
    goal = Pose2D(0.8, 1.5, 0.0)
    result = _plan(snapshot, start, goal)

    assert result.status is PlanStatus.FOUND
    command = result.trajectory[-1].twist
    assert command != Twist2D()
    assert command.linear < 0.0 or abs(command.angular) > 0.0
    assert result.collision is False


def test_dwa_stopped_at_goal_returns_zero_trajectory() -> None:
    snapshot = _snapshot(np.zeros((50, 50), dtype=np.bool_))
    goal = Pose2D(1.0, 1.0)
    result = _plan(snapshot, goal, goal)

    assert result.status is PlanStatus.FOUND
    assert result.trajectory
    assert all(point.pose == goal and point.twist == Twist2D() for point in result.trajectory)
    assert result.cost == 0.0
    assert result.collision is False


def test_reverse_stopping_sweep_extends_behind_robot() -> None:
    start = Pose2D(1.0, 1.0, 0.0)
    sweep = _sweep_distance(
        start,
        linear=-0.1,
        angular=0.0,
        distance=0.2,
        step_m=0.02,
    )

    assert sweep[-1].x < start.x
    assert isclose(sweep[-1].x, 0.8, abs_tol=1e-12)
    assert all(isclose(pose.y, start.y, abs_tol=1e-12) for pose in sweep)


def test_dwa_reuses_collision_checker_cache_for_same_snapshot() -> None:
    snapshot = _snapshot(np.zeros((50, 50), dtype=np.bool_))
    planner = DynamicWindowPlanner()
    start, goal = Pose2D(0.8, 1.0), Pose2D(1.8, 1.0)
    reference = _reference(start, goal)

    first = planner.plan(snapshot, reference, RobotState(start), goal)
    obstacle_checker = planner._cached_obstacle_checker
    collision_checker = planner._cached_collision_checker
    second = planner.plan(snapshot, reference, RobotState(start), goal)

    assert first.status is second.status is PlanStatus.FOUND
    assert planner._cached_obstacle_checker is obstacle_checker
    assert planner._cached_collision_checker is collision_checker
