from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite

import numpy as np
import pytest

from hospital_path_lab.contracts import (
    FollowerResult,
    GridSnapshot,
    PathFollower,
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    Twist2D,
)
from hospital_path_lab.followers import PurePursuitFollower, RegulatedPurePursuitFollower
from hospital_path_lab.grid import GridMap, inflate_occupancy
from hospital_path_lab.local_algorithms import DynamicWindowPlanner
from hospital_path_lab.simulation import (
    SimulationResult,
    simulate_dynamic_local_evidence,
    simulate_follower,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


@dataclass
class _RecordingFollower:
    delegate: PathFollower
    seen_metadata: list[SnapshotMetadata] = field(default_factory=list)

    @property
    def name(self) -> str:
        return self.delegate.name

    def step(
        self,
        path: tuple[Pose2D, ...],
        robot_state: RobotState,
        metadata: SnapshotMetadata,
    ) -> FollowerResult:
        self.seen_metadata.append(metadata)
        return self.delegate.step(path, robot_state, metadata)


def _snapshot(occupancy: np.ndarray | None = None) -> GridSnapshot:
    cells = np.zeros((80, 100), dtype=np.bool_) if occupancy is None else occupancy
    return GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="simulation_integration",
            map_revision=13,
            mission_revision=5,
            observation_revision=21,
            seed=17,
            content_hash="simulation-integration-v1",
        ),
        grid=GridMap(cells, resolution_m=0.05),
    )


def _dynamic_snapshot(
    occupancy: np.ndarray,
    *,
    observation_revision: int,
    content_hash: str,
) -> GridSnapshot:
    return GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="dynamic_local_evidence",
            map_revision=7,
            mission_revision=3,
            observation_revision=observation_revision,
            seed=23,
            content_hash=content_hash,
        ),
        grid=GridMap(occupancy, resolution_m=0.05),
    )


def _assert_successful_tracking(
    result: SimulationResult,
    recorder: _RecordingFollower,
    snapshot: GridSnapshot,
) -> None:
    profile = VIRTUAL_DOLL_WHEELCHAIR_V0_1
    assert result.status is PlanStatus.FOUND
    assert result.goal_reached is True
    assert result.collision is False
    assert result.failure_reason is None
    assert result.commands
    assert result.commands[-1] == Twist2D()
    assert all(
        -profile.max_reverse_speed_mps <= command.linear <= profile.max_forward_speed_mps
        and abs(command.angular) <= profile.max_angular_speed_radps
        for command in result.commands
    )
    assert all(isfinite(value) for value in _numeric_metrics(result))
    assert all(
        all(isfinite(value) for value in (pose.x, pose.y, pose.yaw)) for pose in result.poses
    )
    assert recorder.seen_metadata
    assert all(metadata == snapshot.metadata for metadata in recorder.seen_metadata)
    previous_linear = 0.0
    for command in result.commands:
        maximum_delta = (
            profile.max_acceleration_mps2
            if command.linear >= previous_linear
            else profile.max_deceleration_mps2
        ) * profile.control_period_s
        assert abs(command.linear - previous_linear) <= maximum_delta + 1e-12
        previous_linear = command.linear


def _numeric_metrics(result: SimulationResult) -> tuple[float, ...]:
    return (
        result.elapsed_s,
        result.mean_tracking_error_m,
        result.maximum_tracking_error_m,
        result.jerk_rms_mps3,
        result.final_goal_distance_m,
        result.minimum_clearance_m if result.minimum_clearance_m is not None else 0.0,
    )


@pytest.mark.parametrize(
    "delegate",
    [PurePursuitFollower(), RegulatedPurePursuitFollower()],
    ids=lambda follower: follower.name,
)
def test_followers_reach_goal_on_straight_open_grid(delegate: PathFollower) -> None:
    snapshot = _snapshot()
    path = (Pose2D(0.50, 1.00), Pose2D(1.50, 1.00), Pose2D(2.50, 1.00))
    recorder = _RecordingFollower(delegate)

    result = simulate_follower(
        recorder,
        path,
        snapshot,
        RobotState(path[0]),
        path[-1],
        max_time_s=20.0,
    )

    _assert_successful_tracking(result, recorder, snapshot)
    assert result.maximum_tracking_error_m <= 0.50


@pytest.mark.parametrize(
    "delegate",
    [PurePursuitFollower(), RegulatedPurePursuitFollower()],
    ids=lambda follower: follower.name,
)
def test_followers_reach_goal_on_ninety_degree_polyline(delegate: PathFollower) -> None:
    snapshot = _snapshot()
    path = (
        Pose2D(0.75, 0.75, 0.0),
        Pose2D(2.00, 0.75, 0.0),
        Pose2D(2.00, 2.00, 1.5707963267948966),
    )
    recorder = _RecordingFollower(delegate)

    result = simulate_follower(
        recorder,
        path,
        snapshot,
        RobotState(path[0]),
        path[-1],
        max_time_s=40.0,
    )

    _assert_successful_tracking(result, recorder, snapshot)
    assert result.maximum_tracking_error_m <= 0.75
    assert any(abs(command.angular) > 0.0 for command in result.commands)


@pytest.mark.parametrize(
    "delegate",
    [PurePursuitFollower(), RegulatedPurePursuitFollower()],
    ids=lambda follower: follower.name,
)
def test_inflated_obstacle_on_path_is_recorded_conservatively(delegate: PathFollower) -> None:
    raw = np.zeros((60, 80), dtype=np.bool_)
    raw[20, 30] = True
    inflated = inflate_occupancy(raw, resolution_m=0.05, radius_m=0.18)
    snapshot = _snapshot(inflated)
    path = (Pose2D(0.50, 1.025), Pose2D(2.50, 1.025))

    result = simulate_follower(
        delegate,
        path,
        snapshot,
        RobotState(path[0]),
        path[-1],
        max_time_s=20.0,
    )

    assert result.status is PlanStatus.NO_PATH
    assert not result.goal_reached
    assert result.collision or result.failure_reason == "goal_not_reached_before_timeout"
    assert result.failure_reason in {"collision", "goal_not_reached_before_timeout"}
    assert all(isfinite(value) for value in _numeric_metrics(result))


@pytest.mark.parametrize(
    "delegate",
    [PurePursuitFollower(), RegulatedPurePursuitFollower()],
    ids=lambda follower: follower.name,
)
def test_empty_path_is_invalid_simulation_input(delegate: PathFollower) -> None:
    snapshot = _snapshot()
    start = Pose2D(0.50, 0.50)
    goal = Pose2D(2.00, 0.50)

    result = simulate_follower(delegate, (), snapshot, RobotState(start), goal)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.goal_reached is False
    assert result.collision is False
    assert result.poses == (start,)
    assert result.commands == ()
    assert result.failure_reason == "invalid_simulation_input"


@pytest.mark.parametrize(
    "delegate",
    [PurePursuitFollower(), RegulatedPurePursuitFollower()],
    ids=lambda follower: follower.name,
)
def test_goal_reached_requires_position_and_actual_stop(delegate: PathFollower) -> None:
    snapshot = _snapshot()
    goal = Pose2D(1.0, 1.0)
    result = simulate_follower(
        delegate,
        (Pose2D(0.5, 1.0), goal),
        snapshot,
        RobotState(goal, Twist2D(0.1, 0.0)),
        goal,
        max_time_s=1.0,
    )

    assert result.goal_reached is True
    assert result.status is PlanStatus.FOUND
    assert len(result.commands) >= 2
    assert result.commands[0].linear > 0.0
    assert result.commands[-1] == Twist2D()
    assert result.elapsed_s > 0.0


def test_dynamic_local_evidence_records_stop_deadlock_recovery_and_rejoin() -> None:
    open_grid = np.zeros((60, 80), dtype=np.bool_)
    blocked_grid = open_grid.copy()
    # 차체와 겹치지는 않지만 최소 여유보다 가까워 모든 DWA 이동 후보를 거부한다.
    blocked_grid[32, 27] = True
    blocked = _dynamic_snapshot(
        blocked_grid,
        observation_revision=10,
        content_hash="obstacle-created",
    )
    reopened = _dynamic_snapshot(
        open_grid,
        observation_revision=11,
        content_hash="obstacle-removed",
    )
    snapshots = (blocked, blocked, blocked) + (reopened,) * 60
    events = (
        "obstacle_create",
        "obstacle_hold",
        "obstacle_hold",
        "obstacle_remove",
    ) + ("obstacle_hold",) * 59
    start = Pose2D(1.025, 1.525)
    initial_pose = Pose2D(start.x, start.y + 0.11)
    goal = Pose2D(3.025, 1.525)
    reference_path = (start, goal)

    def run_once():
        planner = DynamicWindowPlanner(
            horizon_s=0.4,
            integration_dt_s=0.1,
            linear_samples=3,
            angular_samples=5,
        )
        return simulate_dynamic_local_evidence(
            planner,
            reference_path,
            snapshots,
            RobotState(initial_pose),
            goal,
            event_kinds=events,
            deadlock_threshold_steps=2,
        )

    result = run_once()

    assert result == run_once()
    assert result.simulation_only is True
    assert result.component == "dwa"
    assert result.collision_count == 0
    assert result.rejected_command_count == 0
    assert result.safe_stop_count == 3
    assert result.no_path_count == 3
    assert result.deadlock_observed is True
    assert result.maximum_no_path_streak == 3
    assert result.maximum_no_progress_streak == 3
    assert result.recovery_observed is True
    assert result.path_deviation_observed is True
    assert result.rejoin_observed is True
    assert result.commands_finite is True
    assert result.metrics_finite is True
    assert result.minimum_clearance_m is not None
    assert result.minimum_clearance_m > 0.0
    assert all(step.command == Twist2D() for step in result.steps[:3])
    assert result.steps[2].deadlock_observed is True
    assert result.steps[3].recovery_observed is True
    rejoin_steps = [step for step in result.steps if step.rejoin_observed]
    assert rejoin_steps
    assert rejoin_steps[0].tracking_error_m <= 0.10
    assert any(step.tracking_error_m > 0.10 for step in result.steps[:3])
    assert result.steps[3].command.linear > 0.0
    assert result.steps[3].observation_revision == 11
    assert result.steps[3].input_content_hash == "obstacle-removed"


def test_dynamic_local_evidence_rejects_mismatched_event_count() -> None:
    snapshot = _snapshot()

    with pytest.raises(ValueError, match="same length"):
        simulate_dynamic_local_evidence(
            DynamicWindowPlanner(
                horizon_s=0.2,
                integration_dt_s=0.1,
                linear_samples=3,
                angular_samples=3,
            ),
            (Pose2D(0.5, 1.0), Pose2D(2.0, 1.0)),
            (snapshot,),
            RobotState(Pose2D(0.5, 1.0)),
            Pose2D(2.0, 1.0),
            event_kinds=("obstacle_create", "obstacle_remove"),
        )
