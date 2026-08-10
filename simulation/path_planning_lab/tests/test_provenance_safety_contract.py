from __future__ import annotations

from dataclasses import replace

import numpy as np

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import (
    GraphSnapshot,
    GridSnapshot,
    LocalPlanResult,
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
)
from hospital_path_lab.evaluation import (
    run_stateless_global,
    validate_follower_result,
    validate_global_result,
    validate_local_result,
    validate_result_provenance,
)
from hospital_path_lab.followers import PurePursuitFollower
from hospital_path_lab.global_algorithms import DStarLitePlanner
from hospital_path_lab.graph import Edge, GraphMap, Node
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_algorithms import BoundedGridAStarPlanner, DynamicWindowPlanner
from hospital_path_lab.planners import AStarPlanner


def _metadata(**changes: object) -> SnapshotMetadata:
    values: dict[str, object] = {
        "map_id": "provenance_map",
        "map_revision": 4,
        "mission_revision": 3,
        "observation_revision": 2,
        "seed": 17,
        "content_hash": "sha256-input-a",
        "input_valid": True,
    }
    values.update(changes)
    return SnapshotMetadata(**values)  # type: ignore[arg-type]


def _graph_snapshot(**metadata_changes: object) -> GraphSnapshot:
    graph = GraphMap(
        [Node("start", 0.0, 0.0), Node("goal", 1.0, 0.0)],
        [Edge("start", "goal", 1.0)],
    )
    return GraphSnapshot(_metadata(**metadata_changes), graph)


def _grid_snapshot(
    *,
    forbidden_cells: frozenset[tuple[int, int]] = frozenset(),
    occupancy: np.ndarray | None = None,
    **metadata_changes: object,
) -> GridSnapshot:
    raw = np.zeros((100, 100), dtype=np.bool_) if occupancy is None else occupancy
    return GridSnapshot(
        _metadata(**metadata_changes),
        GridMap(raw, resolution_m=0.05),
        forbidden_cells,
    )


def _local_found(snapshot: GridSnapshot, path: tuple[Pose2D, ...]) -> LocalPlanResult:
    metadata = snapshot.metadata
    return LocalPlanResult(
        planner="contract_fixture",
        status=PlanStatus.FOUND,
        path=path,
        trajectory=(),
        cost=1.0,
        elapsed_ns=1,
        expanded_nodes=1,
        sampled_trajectories=0,
        map_revision=metadata.map_revision,
        mission_revision=metadata.mission_revision,
        observation_revision=metadata.observation_revision,
        collision=False,
        minimum_clearance=0.1,
        map_id=metadata.map_id,
        input_content_hash=metadata.content_hash,
    )


def test_global_result_carries_map_identity_and_input_hash() -> None:
    snapshot = _graph_snapshot()
    result = run_stateless_global(AStarPlanner(), snapshot, "start", "goal")

    assert result.map_id == snapshot.metadata.map_id
    assert result.input_content_hash == snapshot.metadata.content_hash
    assert validate_global_result(snapshot, "start", "goal", result).executable


def test_same_revisions_with_different_map_id_or_hash_are_rejected() -> None:
    snapshot = _graph_snapshot()
    result = run_stateless_global(AStarPlanner(), snapshot, "start", "goal")

    wrong_map = validate_global_result(
        snapshot,
        "start",
        "goal",
        result,
        current_metadata=replace(snapshot.metadata, map_id="other_map"),
    )
    wrong_hash = validate_global_result(
        snapshot,
        "start",
        "goal",
        result,
        current_metadata=replace(snapshot.metadata, content_hash="sha256-input-b"),
    )

    assert not wrong_map.executable
    assert "stale_map_id" in wrong_map.failures
    assert not wrong_hash.executable
    assert "stale_content_hash" in wrong_hash.failures


def test_local_and_follower_results_use_the_same_provenance_gate() -> None:
    snapshot = _grid_snapshot()
    start = Pose2D(1.0, 1.0)
    goal = Pose2D(2.0, 1.0)
    local_result = _local_found(snapshot, (start, goal))
    follower_result = PurePursuitFollower().step(
        (start, goal), RobotState(start), snapshot.metadata
    )

    assert validate_local_result(snapshot, start, goal, local_result).executable
    assert validate_follower_result(snapshot.metadata, follower_result).executable
    assert follower_result.map_id == snapshot.metadata.map_id
    assert follower_result.input_content_hash == snapshot.metadata.content_hash

    current = replace(snapshot.metadata, content_hash="different-content")
    assert not validate_local_result(
        snapshot, start, goal, local_result, current_metadata=current
    ).executable
    follower_validation = validate_follower_result(
        snapshot.metadata, follower_result, current_metadata=current
    )
    assert not follower_validation.executable
    assert "stale_content_hash" in follower_validation.failures
    provenance_only = validate_result_provenance(
        snapshot.metadata,
        follower_result,
        current_metadata=current,
    )
    assert not provenance_only.executable
    assert provenance_only.failures == ("stale_content_hash",)


def test_invalidated_snapshot_is_refused_before_planning_or_following() -> None:
    graph_snapshot = _graph_snapshot(input_valid=False)
    global_result = run_stateless_global(
        AStarPlanner(), graph_snapshot, "start", "goal"
    )
    assert global_result.status is PlanStatus.INVALID_INPUT
    assert global_result.failure_reason == "snapshot_input_invalidated"
    assert not validate_global_result(
        graph_snapshot, "start", "goal", global_result
    ).executable

    grid_snapshot = _grid_snapshot(input_valid=False)
    start = Pose2D(1.0, 1.0)
    goal = Pose2D(2.0, 1.0)
    for result in (
        BoundedGridAStarPlanner().plan(
            grid_snapshot, (start, goal), RobotState(start), goal
        ),
        DynamicWindowPlanner().plan(
            grid_snapshot, (start, goal), RobotState(start), goal
        ),
    ):
        assert result.status is PlanStatus.INVALID_INPUT
        assert result.failure_reason == "snapshot_input_invalidated"

    follower_result = PurePursuitFollower().step(
        (start, goal), RobotState(start), grid_snapshot.metadata
    )
    assert follower_result.status is PlanStatus.INVALID_INPUT
    assert follower_result.command.linear == 0.0
    assert follower_result.failure_reason == "snapshot_input_invalidated"


def test_dstar_rejects_hash_change_when_all_revisions_are_unchanged() -> None:
    planner = DStarLitePlanner()
    initial = _graph_snapshot()
    planner.reset(initial, "start", "goal")
    changed_hash = GraphSnapshot(
        replace(initial.metadata, content_hash="sha256-input-b"), initial.graph
    )

    result = planner.replan(changed_hash, "start", "goal")

    assert result.status is PlanStatus.STALE_RESULT
    assert result.failure_reason == "content_hash_changed_without_revision"
    assert result.map_id == changed_hash.metadata.map_id
    assert result.input_content_hash == changed_hash.metadata.content_hash


def test_forbidden_zone_entry_has_a_distinct_validation_reason() -> None:
    forbidden_cell = (50, 50)
    snapshot = _grid_snapshot(forbidden_cells=frozenset({forbidden_cell}))
    start = Pose2D(1.0, 1.0)
    goal = Pose2D(4.0, 1.0)
    forbidden_pose = snapshot.grid.cell_to_pose(forbidden_cell)
    result = _local_found(snapshot, (start, forbidden_pose, goal))

    validation = validate_local_result(snapshot, start, goal, result)

    assert not validation.executable
    assert "forbidden_zone_entry" in validation.failures
    assert "collision" not in validation.failures


def test_grid_astar_routes_around_forbidden_cells() -> None:
    forbidden_cells = frozenset((x, y) for x in range(45, 56) for y in range(45, 56))
    snapshot = _grid_snapshot(forbidden_cells=forbidden_cells)
    start = Pose2D(1.0, 2.5)
    goal = Pose2D(4.0, 2.5)

    result = BoundedGridAStarPlanner().plan(
        snapshot, (start, goal), RobotState(start), goal
    )
    checker = CollisionChecker(
        snapshot.grid, forbidden_cells=snapshot.forbidden_cells
    )

    assert result.status is PlanStatus.FOUND
    assert not checker.path_enters_forbidden(result.path)
    assert validate_local_result(snapshot, start, goal, result).executable


def test_dwa_reports_forbidden_goal_as_invalid_input() -> None:
    snapshot = _grid_snapshot(forbidden_cells=frozenset({(60, 20)}))
    start = Pose2D(1.0, 1.0)
    goal = snapshot.grid.cell_to_pose((60, 20))

    result = DynamicWindowPlanner().plan(
        snapshot, (start, goal), RobotState(start), goal
    )

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "goal_forbidden"


def test_exact_cell_aabb_check_detects_one_millimetre_corner_overlap() -> None:
    occupancy = np.zeros((100, 100), dtype=np.bool_)
    occupancy[50, 50] = True  # cell AABB: [1.00, 1.02] x [1.00, 1.02]
    grid = GridMap(occupancy, resolution_m=0.02)
    checker = CollisionChecker(grid)

    overlapping = Pose2D(0.781, 0.821)  # footprint corner reaches (1.001, 1.001)
    separated = Pose2D(0.778, 0.818)  # footprint corner ends at (0.998, 0.998)

    assert checker.clearance(overlapping) == 0.0
    assert not checker.pose_is_collision_free(overlapping)
    assert checker.clearance(separated) > 0.0
