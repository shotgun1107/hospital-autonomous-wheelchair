from __future__ import annotations

from math import ceil, hypot, isclose, isfinite, pi

import numpy as np
import pytest

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import PlanStatus, Pose2D, RobotState
from hospital_path_lab.grid import GridMap, inflate_occupancy
from hospital_path_lab.local_algorithms.grid_astar import BoundedGridAStarPlanner
from hospital_path_lab.map_factory import (
    WorldFamily,
    build_grid_snapshot,
    episode_state_at,
    generate_episode,
    generate_world,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _grid(
    *,
    width: int = 201,
    height: int = 201,
    resolution_m: float = 0.01,
    occupied: tuple[tuple[int, int], ...] = (),
) -> GridMap:
    occupancy = np.zeros((height, width), dtype=np.bool_)
    for x, y in occupied:
        occupancy[y, x] = True
    return GridMap(occupancy=occupancy, resolution_m=resolution_m)


def _brute_force_inflate(
    occupancy: np.ndarray,
    *,
    resolution_m: float,
    radius_m: float,
) -> np.ndarray:
    """Small-grid oracle that does not share the production slicing algorithm."""

    source = np.asarray(occupancy, dtype=np.bool_)
    expected = np.zeros_like(source)
    occupied_cells = tuple(zip(*np.nonzero(source), strict=True))
    for target_y in range(source.shape[0]):
        for target_x in range(source.shape[1]):
            expected[target_y, target_x] = any(
                hypot(target_x - source_x, target_y - source_y) * resolution_m
                <= radius_m
                for source_y, source_x in occupied_cells
            )
    return expected


def test_virtual_doll_profile_is_explicitly_simulation_only() -> None:
    profile = VIRTUAL_DOLL_WHEELCHAIR_V0_1

    assert profile.profile_id == "virtual_doll_wheelchair_v0_1"
    assert profile.simulation_only is True
    assert (profile.body_width_m, profile.body_length_m) == (0.32, 0.40)
    assert (profile.collision_width_m, profile.collision_length_m) == (0.36, 0.44)
    assert profile.minimum_clearance_m == 0.08
    assert profile.stopping_margin_m == 0.15
    assert profile.differential_drive is True
    assert profile.in_place_rotation is True
    assert isclose(profile.control_period_s, 0.05)


def test_rectangular_footprint_detects_overlap_ignored_by_point_check() -> None:
    grid = _grid(occupied=((121, 100),))
    pose = grid.cell_to_pose((100, 100))
    checker = CollisionChecker(grid)

    assert not grid.is_occupied(grid.world_to_cell(pose))
    assert grid.path_is_collision_free((pose,))
    assert checker.clearance(pose) == 0.0
    assert not checker.pose_is_collision_free(pose)


def test_rectangular_footprint_changes_extent_between_zero_and_ninety_degrees() -> None:
    grid = _grid(occupied=((121, 100),))
    center = grid.cell_to_pose((100, 100))
    checker = CollisionChecker(grid)
    length_toward_obstacle = Pose2D(center.x, center.y, yaw=0.0)
    width_toward_obstacle = Pose2D(center.x, center.y, yaw=pi / 2.0)

    assert checker.clearance(length_toward_obstacle) == 0.0
    assert checker.clearance(width_toward_obstacle) > 0.0
    assert not checker.pose_is_collision_free(length_toward_obstacle)
    assert checker.pose_is_collision_free(width_toward_obstacle)


def test_grid_and_checker_reject_footprint_outside_boundary() -> None:
    raw = np.zeros((100, 100), dtype=np.bool_)
    grid = GridMap(raw, resolution_m=0.01)
    raw[50, 50] = True
    checker = CollisionChecker(grid)
    center_inside_but_footprint_outside = Pose2D(0.20, 0.50, yaw=0.0)

    assert not grid.occupancy[50, 50]
    assert not grid.occupancy.flags.writeable
    assert grid.is_occupied((-1, 50))
    assert grid.is_occupied((100, 50))
    assert grid.in_bounds(grid.world_to_cell(center_inside_but_footprint_outside))
    assert checker.clearance(center_inside_but_footprint_outside) == 0.0
    assert not checker.pose_is_collision_free(center_inside_but_footprint_outside)


def test_clearance_is_finite_positive_when_separated_and_zero_on_contact() -> None:
    grid = _grid(occupied=((130, 100),))
    center = grid.cell_to_pose((100, 100))
    checker = CollisionChecker(grid)

    separated = checker.clearance(center)
    touching = checker.clearance(Pose2D(center.x + 0.08, center.y))

    assert isfinite(separated)
    assert separated > 0.0
    assert touching == 0.0


def test_configuration_grid_uses_half_diagonal_plus_clearance_without_mutation() -> None:
    raw = np.zeros((101, 101), dtype=np.bool_)
    raw[50, 50] = True
    original = raw.copy()
    grid = GridMap(raw, resolution_m=0.02)
    checker = CollisionChecker(grid)
    profile = VIRTUAL_DOLL_WHEELCHAIR_V0_1
    radius_m = hypot(
        profile.collision_width_m / 2.0,
        profile.collision_length_m / 2.0,
    ) + profile.minimum_clearance_m

    configuration_grid = checker.configuration_grid
    expected = _brute_force_inflate(
        original,
        resolution_m=grid.resolution_m,
        radius_m=radius_m,
    )
    border = max(0, int(ceil(radius_m / grid.resolution_m - 0.5)))
    expected[:border, :] = True
    expected[-border:, :] = True
    expected[:, :border] = True
    expected[:, -border:] = True

    np.testing.assert_array_equal(raw, original)
    np.testing.assert_array_equal(grid.occupancy, original)
    np.testing.assert_array_equal(configuration_grid.occupancy, expected)
    assert configuration_grid.occupancy[50, 68]
    assert not configuration_grid.occupancy[50, 69]
    assert configuration_grid.resolution_m == grid.resolution_m
    assert configuration_grid.origin_x_m == grid.origin_x_m
    assert configuration_grid.origin_y_m == grid.origin_y_m


@pytest.mark.parametrize("radius_m", [0.0, 0.09, 0.10, 0.14, 0.21, 0.31])
def test_inflate_occupancy_matches_small_grid_brute_force_oracle(radius_m: float) -> None:
    occupancy = np.asarray(
        [
            [0, 0, 0, 0, 0, 0, 1],
            [0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0],
            [1, 0, 0, 0, 0, 0, 0],
        ],
        dtype=np.bool_,
    )
    before = occupancy.copy()

    actual = inflate_occupancy(occupancy, resolution_m=0.10, radius_m=radius_m)
    expected = _brute_force_inflate(
        occupancy,
        resolution_m=0.10,
        radius_m=radius_m,
    )

    np.testing.assert_array_equal(actual, expected)
    np.testing.assert_array_equal(occupancy, before)
    assert not np.shares_memory(actual, occupancy)


@pytest.mark.parametrize("shape", [(2, 9), (3, 11), (4, 20), (20, 3)])
def test_forbidden_certification_small_non_square_grid_falls_back_exactly(
    shape: tuple[int, int],
) -> None:
    height, width = shape
    grid = GridMap(
        np.zeros(shape, dtype=np.bool_),
        resolution_m=0.02,
    )
    forbidden_cells = frozenset({(width // 2, height // 2)})
    poses = (
        Pose2D((width // 2 + 0.5) * 0.02, (height // 2 + 0.5) * 0.02),
        Pose2D(0.01, 0.01),
        Pose2D((width - 0.5) * 0.02, (height - 0.5) * 0.02, 0.7),
    )
    reference = CollisionChecker(
        grid,
        forbidden_cells=forbidden_cells,
        use_optimized_geometry=False,
    )
    optimized = CollisionChecker(
        grid,
        forbidden_cells=forbidden_cells,
        use_optimized_geometry=True,
    )

    reference_results = tuple(
        (
            reference.pose_enters_forbidden(pose),
            reference.forbidden_clearance(pose),
        )
        for pose in poses
    )
    optimized_results = tuple(
        (
            optimized.pose_enters_forbidden(pose),
            optimized.forbidden_clearance(pose),
        )
        for pose in poses
    )

    assert optimized_results == reference_results


def test_map_factory_grid_astar_path_is_footprint_collision_free() -> None:
    world = generate_world(1, WorldFamily.CORRIDOR)
    episode = generate_episode(world, seed=101)
    snapshot = build_grid_snapshot(world, episode)
    state = episode_state_at(episode)
    nodes = {node.node_id: node for node in world.nodes}
    start_node = nodes[state.start]
    goal_node = nodes[state.goal]
    start = Pose2D(start_node.x, start_node.y)
    goal = Pose2D(goal_node.x, goal_node.y)

    result = BoundedGridAStarPlanner().plan(
        snapshot,
        (start, goal),
        RobotState(start),
        goal,
    )
    checker = CollisionChecker(snapshot.grid)

    assert result.status is PlanStatus.FOUND
    assert result.path
    assert result.collision is False
    assert result.minimum_clearance is not None
    assert result.minimum_clearance > 0.0
    assert checker.path_is_collision_free(result.path)
    assert all(isfinite(checker.clearance(pose)) for pose in result.path)
