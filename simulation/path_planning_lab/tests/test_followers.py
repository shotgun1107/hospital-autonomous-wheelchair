from __future__ import annotations

from math import inf, isclose

import pytest

from hospital_path_lab.contracts import (
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    Twist2D,
)
from hospital_path_lab.followers import PurePursuitFollower, RegulatedPurePursuitFollower


@pytest.fixture
def metadata() -> SnapshotMetadata:
    return SnapshotMetadata(
        map_id="follower_test",
        map_revision=4,
        mission_revision=2,
        observation_revision=7,
        seed=11,
        content_hash="follower-test",
    )


@pytest.mark.parametrize(
    ("follower", "expected_lookahead"),
    [
        (PurePursuitFollower(), 1.15),
        (RegulatedPurePursuitFollower(), 1.05),
    ],
    ids=lambda follower: getattr(follower, "name", str(follower)),
)
def test_straight_path_selects_lookahead_after_current_position(
    metadata: SnapshotMetadata,
    follower: PurePursuitFollower | RegulatedPurePursuitFollower,
    expected_lookahead: float,
) -> None:
    path = (Pose2D(0.0, 0.0), Pose2D(1.0, 0.0), Pose2D(2.0, 0.0))
    state = RobotState(Pose2D(0.8, 0.0, 0.0))

    result = follower.step(path, state, metadata)

    assert result.status is PlanStatus.FOUND
    assert result.lookahead_point is not None
    assert isclose(result.lookahead_point.x, expected_lookahead, abs_tol=1e-12)
    assert result.lookahead_point.y == 0.0
    assert result.command.linear == 0.0125
    assert result.command.angular == 0.0
    assert result.map_revision == 4
    assert result.mission_revision == 2
    assert result.observation_revision == 7
    assert result.elapsed_ns >= 0


@pytest.mark.parametrize("follower", [PurePursuitFollower(), RegulatedPurePursuitFollower()])
def test_ninety_degree_path_turns_left_and_clips_angular_speed(
    metadata: SnapshotMetadata,
    follower: PurePursuitFollower | RegulatedPurePursuitFollower,
) -> None:
    path = (Pose2D(0.0, 0.0), Pose2D(1.0, 0.0), Pose2D(1.0, 1.0))
    state = RobotState(Pose2D(0.90, 0.0, 0.0))

    result = follower.step(path, state, metadata)

    assert result.status is PlanStatus.FOUND
    assert result.lookahead_point is not None
    assert result.lookahead_point.x == 1.0
    assert result.lookahead_point.y > 0.0
    assert 0.0 < result.command.angular <= 0.80


def test_s_path_lookahead_continues_on_the_next_segment(metadata: SnapshotMetadata) -> None:
    path = (
        Pose2D(0.0, 0.0),
        Pose2D(1.0, 1.0),
        Pose2D(2.0, -1.0),
        Pose2D(3.0, 0.0),
    )
    state = RobotState(Pose2D(1.0, 1.0, -0.5))

    result = PurePursuitFollower().step(path, state, metadata)

    assert result.lookahead_point is not None
    assert result.lookahead_point.x > 1.0
    assert result.lookahead_point.y < 1.0


@pytest.mark.parametrize("follower", [PurePursuitFollower(), RegulatedPurePursuitFollower()])
def test_goal_within_tolerance_decelerates_instead_of_jumping_to_zero(
    metadata: SnapshotMetadata,
    follower: PurePursuitFollower | RegulatedPurePursuitFollower,
) -> None:
    goal = Pose2D(1.0, 0.0)
    result = follower.step(
        (Pose2D(0.0, 0.0), goal),
        RobotState(Pose2D(0.96, 0.0), Twist2D(0.2, 0.3)),
        metadata,
    )

    assert result.status is PlanStatus.FOUND
    assert isclose(result.command.linear, 0.175, abs_tol=1e-12)
    assert result.command.angular == 0.0
    assert result.lookahead_point == goal


@pytest.mark.parametrize("follower", [PurePursuitFollower(), RegulatedPurePursuitFollower()])
@pytest.mark.parametrize(
    ("path", "failure_reason"),
    [
        ((), "empty_path"),
        ((Pose2D(0.0, 0.0), Pose2D(inf, 0.0)), "nonfinite_path"),
    ],
)
def test_empty_and_nonfinite_paths_are_invalid(
    metadata: SnapshotMetadata,
    follower: PurePursuitFollower | RegulatedPurePursuitFollower,
    path: tuple[Pose2D, ...],
    failure_reason: str,
) -> None:
    result = follower.step(path, RobotState(Pose2D(0.0, 0.0)), metadata)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.command == Twist2D()
    assert result.lookahead_point is None
    assert result.failure_reason == failure_reason


def test_pure_pursuit_angular_command_is_limited(metadata: SnapshotMetadata) -> None:
    result = PurePursuitFollower().step(
        (Pose2D(0.0, 0.0), Pose2D(0.0, 1.0)),
        RobotState(Pose2D(0.0, 0.0, 0.0), Twist2D(0.2, 0.0)),
        metadata,
    )

    assert result.command.angular == 0.80


def test_rpp_slows_down_for_high_curvature(metadata: SnapshotMetadata) -> None:
    follower = RegulatedPurePursuitFollower()
    state = RobotState(Pose2D(0.0, 0.0, 0.0), Twist2D(0.1, 0.0))
    straight = follower.step(
        (Pose2D(0.0, 0.0), Pose2D(1.0, 0.0)), state, metadata
    )
    curved = follower.step((Pose2D(0.0, 0.0), Pose2D(0.0, 1.0)), state, metadata)

    assert straight.command.linear == 0.1125
    assert curved.command.linear == 0.07500000000000001
    assert curved.command.linear < straight.command.linear
    assert curved.command.angular <= 0.80


def test_rpp_adaptive_lookahead_stays_inside_profile_range(
    metadata: SnapshotMetadata,
) -> None:
    follower = RegulatedPurePursuitFollower()
    path = (Pose2D(0.0, 0.0), Pose2D(2.0, 0.0))

    slow = follower.step(path, RobotState(Pose2D(0.0, 0.0), Twist2D(0.0, 0.0)), metadata)
    fast = follower.step(path, RobotState(Pose2D(0.0, 0.0), Twist2D(0.3, 0.0)), metadata)

    assert slow.lookahead_point is not None
    assert fast.lookahead_point is not None
    assert slow.lookahead_point.x == 0.25
    assert fast.lookahead_point.x == 0.475


@pytest.mark.parametrize(
    "follower", [PurePursuitFollower(), RegulatedPurePursuitFollower()]
)
@pytest.mark.parametrize(
    ("current_linear", "expected_linear"),
    [(0.0, 0.0125), (0.1, 0.1125), (0.25, 0.225)],
)
def test_follower_linear_command_obeys_one_period_acceleration_limits(
    metadata: SnapshotMetadata,
    follower: PurePursuitFollower | RegulatedPurePursuitFollower,
    current_linear: float,
    expected_linear: float,
) -> None:
    result = follower.step(
        (Pose2D(0.0, 0.0), Pose2D(2.0, 0.0)),
        RobotState(Pose2D(0.0, 0.0), Twist2D(current_linear, 0.0)),
        metadata,
    )

    assert result.status is PlanStatus.FOUND
    assert isclose(result.command.linear, expected_linear, abs_tol=1e-12)


def test_rpp_registry_name_matches_result_name(metadata: SnapshotMetadata) -> None:
    follower = RegulatedPurePursuitFollower()
    result = follower.step(
        (Pose2D(0.0, 0.0), Pose2D(1.0, 0.0)),
        RobotState(Pose2D(0.0, 0.0)),
        metadata,
    )

    assert follower.name == "rpp"
    assert result.follower == "rpp"


@pytest.mark.parametrize("follower", [PurePursuitFollower(), RegulatedPurePursuitFollower()])
def test_follower_rejects_twist_outside_vehicle_profile(
    metadata: SnapshotMetadata,
    follower: PurePursuitFollower | RegulatedPurePursuitFollower,
) -> None:
    result = follower.step(
        (Pose2D(0.0, 0.0), Pose2D(1.0, 0.0)),
        RobotState(Pose2D(0.0, 0.0), Twist2D(0.31, 0.0)),
        metadata,
    )

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "robot_twist_outside_vehicle_limits"


@pytest.mark.parametrize("follower", [PurePursuitFollower(), RegulatedPurePursuitFollower()])
def test_follower_is_deterministic_except_for_elapsed_time(
    metadata: SnapshotMetadata,
    follower: PurePursuitFollower | RegulatedPurePursuitFollower,
) -> None:
    path = (Pose2D(0.0, 0.0), Pose2D(1.0, 0.5), Pose2D(2.0, 0.0))
    state = RobotState(Pose2D(0.2, 0.1, 0.1), Twist2D(0.12, 0.0))
    results = [follower.step(path, state, metadata) for _ in range(10)]
    signatures = {
        (
            result.status,
            result.command,
            result.lookahead_point,
            result.map_revision,
            result.mission_revision,
            result.observation_revision,
            result.failure_reason,
        )
        for result in results
    }

    assert len(signatures) == 1
