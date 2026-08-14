from __future__ import annotations

from math import pi

import pytest

from hospital_path_lab.local_algorithms.dwb_reference.contracts import (
    DwbGeneratorRequest,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)
from hospital_path_lab.local_algorithms.dwb_reference.core import IllegalTrajectoryError
from hospital_path_lab.local_algorithms.dwb_reference.critics import (
    DwbCriticGrid,
    GoalAlignCritic,
    GoalDistCritic,
    OscillationCritic,
    PathAlignCritic,
    PathDistCritic,
    RotateToGoalCritic,
    build_manhattan_distance_field,
)


def _request(
    x: float,
    y: float,
    *,
    yaw: float = 0.0,
    linear: float = 0.0,
    angular: float = 0.0,
) -> DwbGeneratorRequest:
    return DwbGeneratorRequest(
        pose=DwbPose2D(x, y, yaw),
        current_twist=DwbTwist2D(linear, angular),
    )


def _trajectory(
    final_pose: DwbPose2D,
    *,
    linear: float = 0.0,
    angular: float = 0.0,
    initial_pose: DwbPose2D | None = None,
) -> DwbTrajectory:
    return DwbTrajectory(
        command=DwbTwist2D(linear, angular),
        poses=(initial_pose or final_pose, final_pose),
        integration_step_s=0.05,
    )


def test_manhattan_field_uses_four_neighbours_and_classifies_illegal_cells() -> None:
    grid = DwbCriticGrid(
        width=5,
        height=3,
        resolution_m=1.0,
        blocked_cells=frozenset({(2, 0), (2, 1), (2, 2)}),
    )
    field = build_manhattan_distance_field(grid, ((0, 1),))

    assert field.score_pose(DwbPose2D(1.5, 2.5, 0.0)) == 2.0
    with pytest.raises(IllegalTrajectoryError) as blocked:
        field.score_pose(DwbPose2D(2.5, 1.5, 0.0))
    assert blocked.value.reason_code == "blocked_grid_cell"
    with pytest.raises(IllegalTrajectoryError) as unreachable:
        field.score_pose(DwbPose2D(4.5, 1.5, 0.0))
    assert unreachable.value.reason_code == "unreachable_grid_cell"
    with pytest.raises(IllegalTrajectoryError) as boundary:
        field.score_pose(DwbPose2D(-0.1, 1.5, 0.0))
    assert boundary.value.reason_code == "off_grid"


def test_path_dist_scores_only_the_last_pose_against_rasterized_path() -> None:
    critic = PathDistCritic(DwbCriticGrid(width=5, height=4, resolution_m=1.0))
    critic.set_path((DwbPose2D(0.5, 1.5, 0.0), DwbPose2D(4.5, 1.5, 0.0)))

    assert critic.prepare(_request(0.5, 1.5))
    trajectory = _trajectory(
        DwbPose2D(3.5, 2.5, 0.0),
        initial_pose=DwbPose2D(3.5, 3.5, 0.0),
    )
    assert critic.score(trajectory) == 1.0


def test_goal_dist_uses_last_local_path_cell_as_only_source() -> None:
    critic = GoalDistCritic(DwbCriticGrid(width=5, height=4, resolution_m=1.0))
    critic.set_path((DwbPose2D(0.5, 1.5, 0.0), DwbPose2D(4.5, 1.5, 0.0)))

    assert critic.prepare(_request(0.5, 1.5))
    assert critic.score(_trajectory(DwbPose2D(2.5, 2.5, 0.0))) == 3.0


def test_path_and_goal_alignment_use_forward_projected_geometry() -> None:
    grid = DwbCriticGrid(width=14, height=10, resolution_m=0.1)
    path = (DwbPose2D(0.05, 0.55, 0.0), DwbPose2D(0.95, 0.55, 0.0))
    path_align = PathAlignCritic(grid, forward_point_distance_m=0.1)
    goal_align = GoalAlignCritic(grid, forward_point_distance_m=0.1)
    path_align.set_path(path)
    goal_align.set_path(path)

    assert path_align.prepare(_request(0.05, 0.55))
    assert goal_align.prepare(_request(0.05, 0.55))
    pointing_toward_path = _trajectory(DwbPose2D(0.45, 0.45, pi / 2.0))
    pointing_along_path = _trajectory(DwbPose2D(0.95, 0.55, 0.0))

    assert path_align.score(pointing_toward_path) == 0.0
    assert goal_align.score(pointing_along_path) == 0.0


def test_path_align_penalizes_blocked_and_unreachable_projected_pose() -> None:
    grid = DwbCriticGrid(
        width=6,
        height=3,
        resolution_m=1.0,
        blocked_cells=frozenset({(2, 0), (2, 1), (2, 2)}),
    )
    critic = PathAlignCritic(grid, forward_point_distance_m=0.6)
    critic.set_path((DwbPose2D(0.5, 1.5, 0.0), DwbPose2D(1.5, 1.5, 0.0)))
    assert critic.prepare(_request(0.1, 1.5))

    blocked_projection = _trajectory(DwbPose2D(1.5, 1.5, 0.0))
    unreachable_projection = _trajectory(DwbPose2D(3.5, 1.5, 0.0))

    assert critic.score(blocked_projection) == float(grid.width * grid.height)
    assert critic.score(unreachable_projection) == float(grid.width * grid.height + 1)


def test_goal_align_penalizes_blocked_and_unreachable_projected_pose() -> None:
    blocked_grid = DwbCriticGrid(
        width=7,
        height=4,
        resolution_m=1.0,
        blocked_cells=frozenset({(2, 2)}),
    )
    blocked = GoalAlignCritic(blocked_grid, forward_point_distance_m=1.0)
    blocked.set_path(
        (DwbPose2D(0.5, 1.5, 0.0), DwbPose2D(4.5, 1.5, 0.0))
    )
    assert blocked.prepare(_request(0.5, 1.5))

    blocked_projection = _trajectory(DwbPose2D(2.5, 1.5, pi / 2.0))
    assert blocked.score(blocked_projection) == float(
        blocked_grid.width * blocked_grid.height
    )

    unreachable_grid = DwbCriticGrid(
        width=7,
        height=3,
        resolution_m=1.0,
        blocked_cells=frozenset({(3, 0), (3, 1), (3, 2)}),
    )
    unreachable = GoalAlignCritic(unreachable_grid, forward_point_distance_m=0.4)
    unreachable.set_path(
        (DwbPose2D(0.5, 1.5, 0.0), DwbPose2D(5.5, 1.5, 0.0))
    )
    assert unreachable.prepare(_request(0.5, 1.5))

    unreachable_projection = _trajectory(DwbPose2D(4.5, 1.5, 0.0))
    assert unreachable.score(unreachable_projection) == float(
        unreachable_grid.width * unreachable_grid.height + 1
    )


@pytest.mark.parametrize("critic_type", [PathDistCritic, GoalDistCritic])
def test_distance_critics_keep_blocked_pose_illegal(critic_type) -> None:
    grid = DwbCriticGrid(
        width=5,
        height=3,
        resolution_m=1.0,
        blocked_cells=frozenset({(2, 1)}),
    )
    critic = critic_type(grid)
    critic.set_path((DwbPose2D(0.5, 1.5, 0.0), DwbPose2D(1.5, 1.5, 0.0)))
    assert critic.prepare(_request(0.5, 1.5))

    with pytest.raises(IllegalTrajectoryError) as rejected:
        critic.score(_trajectory(DwbPose2D(2.5, 1.5, 0.0)))
    assert rejected.value.reason_code == "blocked_grid_cell"


def test_path_align_disables_forward_point_close_to_goal() -> None:
    critic = PathAlignCritic(
        DwbCriticGrid(width=5, height=3, resolution_m=0.1),
        forward_point_distance_m=0.2,
    )
    critic.set_path((DwbPose2D(0.05, 0.15, 0.0), DwbPose2D(0.35, 0.15, 0.0)))

    assert critic.prepare(_request(0.25, 0.15))
    assert critic.disabled_near_goal
    assert critic.score(_trajectory(DwbPose2D(99.0, 99.0, 0.0))) == 0.0


def test_goal_align_can_disable_forward_projection_close_to_a_section_goal() -> None:
    critic = GoalAlignCritic(
        DwbCriticGrid(width=8, height=4, resolution_m=0.1),
        forward_point_distance_m=0.2,
        disable_near_goal=True,
    )
    critic.set_path((DwbPose2D(0.05, 0.15, 0.0), DwbPose2D(0.35, 0.15, 0.0)))

    assert critic.prepare(_request(0.25, 0.15))
    assert critic.disabled_near_goal
    assert critic.score(_trajectory(DwbPose2D(99.0, 99.0, 0.0))) == 0.0

    critic.reset()
    assert not critic.disabled_near_goal


def test_reverse_projection_keeps_alignment_critics_active_near_section_goal() -> None:
    grid = DwbCriticGrid(width=8, height=8, resolution_m=0.1)
    path = (DwbPose2D(0.35, 0.55, pi / 2.0), DwbPose2D(0.35, 0.45, pi / 2.0))
    path_align = PathAlignCritic(grid, forward_point_distance_m=0.2)
    goal_align = GoalAlignCritic(
        grid,
        forward_point_distance_m=0.2,
        disable_near_goal=True,
    )
    for critic in (path_align, goal_align):
        critic.set_path(path)
        critic.set_projection_sign(-1.0)
        critic.set_disable_near_goal(False)
        assert critic.prepare(_request(0.35, 0.50, yaw=pi / 2.0))
        assert not critic.disabled_near_goal


def test_oscillation_rejects_second_sign_reversal_until_reset_distance() -> None:
    critic = OscillationCritic(reset_distance_m=0.05, reset_angle_rad=0.2)

    critic.prepare(_request(0.0, 0.0))
    critic.debrief(DwbTwist2D(0.1, 0.0))
    critic.prepare(_request(0.0, 0.0))
    critic.debrief(DwbTwist2D(-0.1, 0.0))

    assert critic.has_restrictions
    with pytest.raises(IllegalTrajectoryError) as reversal:
        critic.score(_trajectory(DwbPose2D(0.0, 0.0, 0.0), linear=0.1))
    assert reversal.value.reason_code == "oscillation_sign_reversal"
    assert critic.score(_trajectory(DwbPose2D(0.0, 0.0, 0.0), linear=-0.1)) == 0.0

    critic.prepare(_request(0.051, 0.0))
    critic.debrief(DwbTwist2D(-0.1, 0.0))
    assert not critic.has_restrictions
    assert critic.score(_trajectory(DwbPose2D(0.051, 0.0, 0.0), linear=0.1)) == 0.0


def test_oscillation_tracks_rotation_only_at_low_linear_speed() -> None:
    critic = OscillationCritic(linear_only_threshold_mps=0.05)

    for angular in (0.2, -0.2, 0.2):
        critic.prepare(_request(0.0, 0.0))
        critic.debrief(DwbTwist2D(0.1, angular))
    assert not critic.has_restrictions

    critic.prepare(_request(0.0, 0.0))
    critic.debrief(DwbTwist2D(0.0, 0.2))
    critic.prepare(_request(0.0, 0.0))
    critic.debrief(DwbTwist2D(0.0, -0.2))
    with pytest.raises(IllegalTrajectoryError):
        critic.score(_trajectory(DwbPose2D(0.0, 0.0, 0.0), angular=0.2))


def test_rotate_to_goal_is_neutral_outside_window_then_requires_slowing() -> None:
    critic = RotateToGoalCritic(
        xy_goal_tolerance_m=0.25,
        path_length_tolerance_m=1.0,
        stopped_linear_velocity_mps=0.05,
    )
    critic.set_path((DwbPose2D(0.0, 0.0, 0.0), DwbPose2D(1.0, 0.0, pi / 2.0)))

    assert critic.prepare(_request(0.0, 0.0, linear=0.2))
    assert critic.score(_trajectory(DwbPose2D(0.1, 0.0, 0.0), linear=0.2)) == 0.0

    assert critic.prepare(_request(0.9, 0.0, linear=0.2))
    assert critic.in_window and not critic.rotating
    with pytest.raises(IllegalTrajectoryError) as no_slowing:
        critic.score(_trajectory(DwbPose2D(0.95, 0.0, 0.0), linear=0.2))
    assert no_slowing.value.reason_code == "not_slowing_near_goal"
    assert critic.score(_trajectory(DwbPose2D(0.95, 0.0, 0.0), linear=0.1)) > 0.0


def test_rotate_to_goal_latches_rotate_only_and_scores_goal_yaw() -> None:
    critic = RotateToGoalCritic(
        xy_goal_tolerance_m=0.25,
        path_length_tolerance_m=1.0,
        stopped_linear_velocity_mps=0.05,
    )
    critic.set_path((DwbPose2D(0.0, 0.0, 0.0), DwbPose2D(1.0, 0.0, pi / 2.0)))

    assert critic.prepare(_request(0.9, 0.0, linear=0.04))
    assert critic.rotating
    with pytest.raises(IllegalTrajectoryError) as translation:
        critic.score(_trajectory(DwbPose2D(0.9, 0.0, pi / 2.0), linear=0.01))
    assert translation.value.reason_code == "translation_during_goal_rotation"

    aligned = _trajectory(DwbPose2D(0.9, 0.0, pi / 2.0), angular=0.2)
    unaligned = _trajectory(DwbPose2D(0.9, 0.0, 0.0), angular=0.2)
    assert critic.score(aligned) == pytest.approx(0.0)
    assert critic.score(unaligned) == pytest.approx(pi / 2.0)

    # Latching is intentional: leaving the window does not revert to translation.
    assert critic.prepare(_request(0.0, 0.0, linear=0.0))
    assert critic.in_window and critic.rotating


def test_rotate_to_goal_lookahead_interpolates_yaw() -> None:
    critic = RotateToGoalCritic(
        xy_goal_tolerance_m=0.25,
        stopped_linear_velocity_mps=0.05,
        lookahead_time_s=0.025,
    )
    critic.set_path((DwbPose2D(0.0, 0.0, 0.0), DwbPose2D(0.1, 0.0, pi / 2.0)))
    critic.prepare(_request(0.0, 0.0, linear=0.0))
    trajectory = _trajectory(
        DwbPose2D(0.0, 0.0, pi / 2.0),
        angular=0.2,
        initial_pose=DwbPose2D(0.0, 0.0, 0.0),
    )

    assert critic.score(trajectory) == pytest.approx(pi / 4.0)
