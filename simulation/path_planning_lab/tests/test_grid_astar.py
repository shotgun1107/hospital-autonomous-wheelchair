from __future__ import annotations

from heapq import heappop, heappush
from math import inf, isclose

import numpy as np
import pytest

from hospital_path_lab.contracts import (
    GridSnapshot,
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
)
from hospital_path_lab.grid import GridCell, GridMap
from hospital_path_lab.local_algorithms.grid_astar import (
    BoundedGridAStarPlanner,
    GridSearchBounds,
    reference_search_bounds,
)


def _snapshot(rows: list[str]) -> GridSnapshot:
    occupancy = np.asarray([[cell == "#" for cell in row] for row in rows], dtype=np.bool_)
    return GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="grid_test",
            map_revision=7,
            mission_revision=11,
            observation_revision=13,
            seed=0,
            content_hash="test-grid",
        ),
        grid=GridMap(occupancy=occupancy, resolution_m=1.0),
    )


def _pose(cell: GridCell) -> Pose2D:
    return Pose2D(x=cell[0] + 0.5, y=cell[1] + 0.5)


def _cells(snapshot: GridSnapshot, path: tuple[Pose2D, ...]) -> tuple[GridCell, ...]:
    return tuple(snapshot.grid.world_to_cell(pose) for pose in path)


def _plan(
    snapshot: GridSnapshot,
    start: GridCell,
    goal: GridCell,
    *,
    reference_path: tuple[Pose2D, ...] | None = None,
    search_margin_m: float = 0.5,
):
    reference = reference_path or (_pose(start), _pose(goal))
    return BoundedGridAStarPlanner(search_margin_m=search_margin_m).plan(
        snapshot,
        reference,
        RobotState(_pose(start)),
        _pose(goal),
    )


def _grid_dijkstra_cost(
    grid: GridMap,
    start: GridCell,
    goal: GridCell,
    bounds: GridSearchBounds | None = None,
) -> float | None:
    frontier: list[tuple[float, int, int]] = [(0.0, start[0], start[1])]
    best = {start: 0.0}
    while frontier:
        cost, x, y = heappop(frontier)
        current = (x, y)
        if cost > best.get(current, inf):
            continue
        if current == goal:
            return cost
        for neighbor, edge_cost in grid.neighbors8(current):
            if bounds is not None and not bounds.contains(neighbor):
                continue
            candidate = cost + edge_cost
            if candidate >= best.get(neighbor, inf):
                continue
            best[neighbor] = candidate
            heappush(frontier, (candidate, neighbor[0], neighbor[1]))
    return None


@pytest.mark.parametrize(
    ("rows", "start", "goal", "search_margin_m"),
    [
        ([".......", ".......", ".......", ".......", "......."], (0, 2), (6, 2), 0.5),
        (
            ["...#...", "...#...", ".......", "...#...", "...#..."],
            (0, 2),
            (6, 2),
            0.5,
        ),
        (
            [".......", ".#####.", ".#...#.", ".#.#.#.", ".#...#.", ".#####.", "......."],
            (0, 3),
            (6, 3),
            3.0,
        ),
    ],
    ids=["wide_corridor", "door", "u_trap"],
)
def test_grid_astar_matches_independent_dijkstra_oracle(
    rows: list[str], start: GridCell, goal: GridCell, search_margin_m: float
) -> None:
    snapshot = _snapshot(rows)
    reference = (_pose(start), _pose(goal))
    result = _plan(
        snapshot,
        start,
        goal,
        reference_path=reference,
        search_margin_m=search_margin_m,
    )
    bounds = reference_search_bounds(snapshot.grid, reference, search_margin_m)
    expected_cost = _grid_dijkstra_cost(snapshot.grid, start, goal, bounds)

    assert expected_cost is not None
    assert result.status is PlanStatus.FOUND
    assert result.cost is not None
    assert isclose(result.cost, expected_cost, rel_tol=1e-12)
    assert snapshot.grid.path_is_collision_free(result.path)
    assert result.collision is False
    assert result.minimum_clearance is not None
    assert result.failure_reason is None
    revisions = (result.map_revision, result.mission_revision, result.observation_revision)
    assert revisions == (7, 11, 13)


@pytest.mark.parametrize(
    ("rows", "start", "goal"),
    [
        (["...#...", "...#...", "...#...", "...#...", "...#..."], (0, 2), (6, 2)),
        ([".......", "#######", "......."], (0, 0), (6, 2)),
    ],
    ids=["narrow_blocked", "no_rejoin"],
)
def test_grid_astar_reports_no_path(
    rows: list[str], start: GridCell, goal: GridCell
) -> None:
    snapshot = _snapshot(rows)
    result = _plan(snapshot, start, goal)

    assert _grid_dijkstra_cost(snapshot.grid, start, goal) is None
    assert result.status is PlanStatus.NO_PATH
    assert result.path == ()
    assert result.cost is None
    assert result.expanded_nodes > 0
    assert result.collision is False
    assert result.minimum_clearance is None
    assert result.failure_reason == "no_path"


@pytest.mark.parametrize(
    ("start", "goal", "reason"),
    [((1, 1), (2, 2), "start_occupied"), ((0, 0), (1, 1), "goal_occupied")],
)
def test_grid_astar_rejects_occupied_endpoint(
    start: GridCell, goal: GridCell, reason: str
) -> None:
    snapshot = _snapshot(["...", ".#.", "..."])
    result = _plan(snapshot, start, goal)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == reason
    assert result.path == ()
    assert result.expanded_nodes == 0


def test_grid_astar_does_not_cut_diagonal_corners() -> None:
    snapshot = _snapshot([".#", "#."])
    result = _plan(snapshot, (0, 0), (1, 1))

    assert result.status is PlanStatus.NO_PATH


def test_grid_astar_uses_deterministic_tie_break() -> None:
    snapshot = _snapshot([".....", "..#..", "....."])
    signatures = []
    for _ in range(20):
        result = _plan(snapshot, (0, 1), (4, 1))
        signatures.append((_cells(snapshot, result.path), result.cost, result.expanded_nodes))

    assert len(set(signatures)) == 1
    assert signatures[0][0] == ((0, 1), (1, 0), (2, 0), (3, 0), (4, 1))


def test_grid_astar_rejects_out_of_bounds_or_non_finite_input() -> None:
    snapshot = _snapshot(["...", "...", "..."])
    planner = BoundedGridAStarPlanner()

    reference = (_pose((0, 0)), _pose((2, 2)))
    outside = planner.plan(
        snapshot, reference, RobotState(_pose((-1, 0))), _pose((2, 2))
    )
    non_finite = planner.plan(
        snapshot,
        reference,
        RobotState(Pose2D(float("nan"), 0.5)),
        _pose((2, 2)),
    )

    assert outside.status is PlanStatus.INVALID_INPUT
    assert outside.failure_reason == "start_out_of_bounds"
    assert non_finite.status is PlanStatus.INVALID_INPUT
    assert non_finite.failure_reason == "robot_state_non_finite"


def test_grid_astar_rejects_empty_reference_path() -> None:
    snapshot = _snapshot(["...", "...", "..."])
    result = BoundedGridAStarPlanner().plan(
        snapshot, (), RobotState(_pose((0, 0))), _pose((2, 2))
    )

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "reference_path_empty"


def test_grid_astar_does_not_escape_reference_search_bounds() -> None:
    snapshot = _snapshot(
        [
            ".......",
            ".......",
            ".......",
            "...#...",
            ".......",
            ".......",
            ".......",
        ]
    )
    start, goal = (0, 3), (6, 3)
    reference = (_pose(start), _pose(goal))
    bounds = reference_search_bounds(snapshot.grid, reference, margin_m=0.0)

    assert _grid_dijkstra_cost(snapshot.grid, start, goal) is not None
    assert _grid_dijkstra_cost(snapshot.grid, start, goal, bounds) is None
    result = _plan(
        snapshot,
        start,
        goal,
        reference_path=reference,
        search_margin_m=0.0,
    )

    assert result.status is PlanStatus.NO_PATH
    assert result.failure_reason == "no_path"


def test_grid_astar_reuses_collision_checker_for_same_snapshot() -> None:
    snapshot = _snapshot([".......", ".......", "......."])
    planner = BoundedGridAStarPlanner()

    first = planner.plan(
        snapshot,
        (_pose((0, 1)), _pose((6, 1))),
        RobotState(_pose((0, 1))),
        _pose((6, 1)),
    )
    checker = planner._cached_collision_checker
    second = planner.plan(
        snapshot,
        (_pose((0, 1)), _pose((6, 1))),
        RobotState(_pose((0, 1))),
        _pose((6, 1)),
    )

    assert first.status is second.status is PlanStatus.FOUND
    assert planner._cached_collision_checker is checker
