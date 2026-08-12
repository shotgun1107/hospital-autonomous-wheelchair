from __future__ import annotations

from math import pi

import pytest

from hospital_path_lab.local_algorithms.dwb_reference.contracts import DwbPose2D, DwbTwist2D
from hospital_path_lab.local_algorithms.dwb_reference.goal_controller import (
    DwbGoalControllerConfig,
    DwbGoalControlRequest,
    DwbGoalControlState,
    DwbLatchedGoalController,
    shortest_angular_distance,
)


def _request(
    tick: int,
    *,
    session: str = "mission-1",
    x: float = 0.0,
    y: float = 0.0,
    yaw: float = 0.0,
    linear: float = 0.0,
    angular: float = 0.0,
    goal_x: float = 1.0,
    goal_y: float = 0.0,
    goal_yaw: float = 0.0,
) -> DwbGoalControlRequest:
    return DwbGoalControlRequest(
        session_key=session,
        tick=tick,
        pose=DwbPose2D(x, y, yaw),
        actual_twist=DwbTwist2D(linear, angular),
        goal_pose=DwbPose2D(goal_x, goal_y, goal_yaw),
    )


def test_outside_xy_tolerance_leaves_path_tracking_in_control() -> None:
    controller = DwbLatchedGoalController()

    result = controller.update(_request(0, x=0.0))

    assert result.state is DwbGoalControlState.TRACK_PATH
    assert result.command is None
    assert not result.overrides_path_tracking
    assert not result.xy_tolerance_latched
    assert not result.goal_complete


def test_xy_tolerance_latches_and_both_axes_decelerate_with_limits() -> None:
    controller = DwbLatchedGoalController()

    result = controller.update(
        _request(0, x=0.95, linear=0.20, angular=-0.40, goal_yaw=1.0)
    )

    assert result.state is DwbGoalControlState.DECELERATE_TO_STOP
    assert result.xy_tolerance_latched
    assert result.command is not None
    assert result.command.linear_mps == pytest.approx(0.175)
    assert result.command.angular_radps == pytest.approx(-0.32)
    assert not result.goal_complete


def test_rotation_never_starts_until_linear_and_angular_motion_are_stopped() -> None:
    controller = DwbLatchedGoalController()
    controller.update(_request(0, x=0.95, linear=0.20, angular=0.20, goal_yaw=1.0))

    still_moving = controller.update(
        _request(1, x=0.95, linear=0.02, angular=0.03, goal_yaw=1.0)
    )
    stopped = controller.update(_request(2, x=0.95, goal_yaw=1.0))

    assert still_moving.state is DwbGoalControlState.DECELERATE_TO_STOP
    assert still_moving.command == DwbTwist2D(0.0, 0.0)
    assert stopped.state is DwbGoalControlState.ROTATE_TO_GOAL
    assert stopped.command is not None
    assert stopped.command.linear_mps == 0.0
    assert stopped.command.angular_radps > 0.0


def test_shortest_angle_rotation_crosses_wrap_boundary() -> None:
    controller = DwbLatchedGoalController()
    request = _request(
        0,
        x=1.0,
        yaw=pi - 0.10,
        goal_yaw=-pi + 0.10,
    )

    result = controller.update(request)

    angular_distance = shortest_angular_distance(
        request.pose.yaw_rad,
        request.goal_pose.yaw_rad,
    )
    assert angular_distance == pytest.approx(0.20)
    assert result.state is DwbGoalControlState.ROTATE_TO_GOAL
    assert result.command is not None
    assert result.command.angular_radps > 0.0


def test_yaw_tolerance_requires_bounded_deceleration_and_actual_stop() -> None:
    controller = DwbLatchedGoalController()
    controller.update(_request(0, x=1.0, yaw=0.0, goal_yaw=1.0))

    braking = controller.update(
        _request(1, x=1.0, yaw=0.95, angular=0.40, goal_yaw=1.0)
    )
    complete = controller.update(_request(2, x=1.0, yaw=0.99, goal_yaw=1.0))

    assert braking.state is DwbGoalControlState.ROTATE_TO_GOAL
    assert braking.command == DwbTwist2D(0.0, 0.32)
    assert not braking.goal_complete
    assert complete.state is DwbGoalControlState.ALIGNED_STOP
    assert complete.command == DwbTwist2D(0.0, 0.0)
    assert complete.goal_complete


def test_xy_latch_is_not_lost_when_pose_drifts_outside_tolerance() -> None:
    controller = DwbLatchedGoalController()
    controller.update(_request(0, x=0.95, goal_yaw=1.0))

    drifted = controller.update(_request(1, x=0.50, yaw=0.1, goal_yaw=1.0))

    assert drifted.xy_tolerance_latched
    assert drifted.state is DwbGoalControlState.ROTATE_TO_GOAL
    assert drifted.command is not None


def test_same_tick_same_input_is_idempotent() -> None:
    controller = DwbLatchedGoalController()
    request = _request(5, x=0.95, linear=0.20, angular=0.20, goal_yaw=1.0)

    first = controller.update(request)
    second = controller.update(request)

    assert second is first
    assert controller.state is DwbGoalControlState.DECELERATE_TO_STOP


def test_same_tick_changed_input_and_backward_tick_are_rejected() -> None:
    controller = DwbLatchedGoalController()
    controller.update(_request(5))

    with pytest.raises(ValueError, match="same session tick"):
        controller.update(_request(5, x=0.1))
    with pytest.raises(ValueError, match="monotonically"):
        controller.update(_request(4))


def test_session_change_resets_xy_latch_and_cached_tick() -> None:
    controller = DwbLatchedGoalController()
    completed = controller.update(_request(10, x=1.0))

    next_session = controller.update(_request(0, session="mission-2", x=0.0))

    assert completed.state is DwbGoalControlState.ALIGNED_STOP
    assert next_session.state is DwbGoalControlState.TRACK_PATH
    assert not next_session.xy_tolerance_latched
    assert next_session.command is None


def test_explicit_reset_clears_completion_and_accepts_new_goal() -> None:
    controller = DwbLatchedGoalController()
    controller.update(_request(0, x=1.0))

    controller.reset("mission-1")
    restarted = controller.update(_request(0, goal_x=2.0, x=0.0))

    assert restarted.state is DwbGoalControlState.TRACK_PATH
    assert not restarted.xy_tolerance_latched


def test_goal_change_without_reset_is_rejected() -> None:
    controller = DwbLatchedGoalController()
    controller.update(_request(0))

    with pytest.raises(ValueError, match="goal_pose changed"):
        controller.update(_request(1, goal_x=2.0))


def test_rotation_reversal_brakes_before_changing_direction() -> None:
    controller = DwbLatchedGoalController()
    controller.update(_request(0, x=1.0, yaw=0.0, goal_yaw=-1.0))

    result = controller.update(
        _request(1, x=1.0, yaw=0.0, angular=0.40, goal_yaw=-1.0)
    )

    assert result.command == DwbTwist2D(0.0, 0.32)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("control_period_s", 0.0),
        ("xy_goal_tolerance_m", -0.1),
        ("maximum_angular_speed_radps", float("nan")),
    ],
)
def test_config_rejects_non_positive_or_non_finite_values(field: str, value: float) -> None:
    with pytest.raises(ValueError, match=field):
        DwbGoalControllerConfig(**{field: value})
