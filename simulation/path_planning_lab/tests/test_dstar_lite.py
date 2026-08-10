from __future__ import annotations

from math import isclose

import networkx as nx

from hospital_path_lab.contracts import GraphSnapshot, PlanStatus, SnapshotMetadata
from hospital_path_lab.global_algorithms import DStarLitePlanner
from hospital_path_lab.graph import Edge, GraphMap, Node, canonical_edge
from hospital_path_lab.planners import DijkstraPlanner


def _graph(*, closed_edges: set[tuple[str, str]] | None = None) -> GraphMap:
    return GraphMap(
        nodes=[
            Node("start", 0.0, 0.0),
            Node("upper", 1.0, 1.0),
            Node("lower", 1.0, -1.0),
            Node("goal", 2.0, 0.0),
            Node("waiting", 0.0, 2.0),
        ],
        edges=[
            Edge("start", "upper", 1.4),
            Edge("upper", "goal", 1.4),
            Edge("start", "lower", 1.5),
            Edge("lower", "goal", 1.5),
            Edge("upper", "waiting", 1.0),
        ],
        closed_edges=closed_edges,
    )


def _snapshot(
    revision: int,
    *,
    closed_edges: set[tuple[str, str]] | None = None,
    mission_revision: int = 0,
) -> GraphSnapshot:
    closed_label = ",".join("-".join(sorted(edge)) for edge in sorted(closed_edges or set()))
    return GraphSnapshot(
        metadata=SnapshotMetadata(
            map_id="dstar_sequence",
            map_revision=revision,
            mission_revision=mission_revision,
            observation_revision=revision,
            seed=7,
            content_hash=f"revision-{revision}:{closed_label}",
        ),
        graph=_graph(closed_edges=closed_edges),
    )


def _assert_matches_oracles(
    snapshot: GraphSnapshot,
    start: str,
    goal: str,
    result: object,
) -> None:
    graph = snapshot.graph
    dijkstra = DijkstraPlanner().plan(graph, start, goal)

    oracle = nx.DiGraph() if graph.directed else nx.Graph()
    oracle.add_nodes_from(graph.nodes)
    for edge in graph.edges:
        key = canonical_edge(edge.source, edge.target, directed=graph.directed)
        if key not in graph.closed_edges:
            oracle.add_edge(edge.source, edge.target, weight=edge.cost)
    try:
        expected_cost = nx.shortest_path_length(oracle, start, goal, weight="weight")
    except nx.NetworkXNoPath:
        expected_cost = None

    assert result.status is dijkstra.status
    if expected_cost is None:
        assert result.status is PlanStatus.NO_PATH
        assert result.path == ()
        assert result.cost is None
        assert result.failure_reason == "no_path"
    else:
        assert result.status is PlanStatus.FOUND
        assert result.cost is not None
        assert isclose(result.cost, expected_cost, rel_tol=1e-9)
        assert isclose(graph.path_cost(result.path), expected_cost, rel_tol=1e-9)
    assert result.map_revision == snapshot.metadata.map_revision
    assert result.mission_revision == snapshot.metadata.mission_revision
    assert result.observation_revision == snapshot.metadata.observation_revision
    assert result.elapsed_ns >= 0


def test_sequential_close_open_move_start_and_no_path_reuses_state() -> None:
    planner = DStarLitePlanner()

    initial = _snapshot(0)
    result = planner.reset(initial, "start", "goal")
    _assert_matches_oracles(initial, "start", "goal", result)
    assert result.path == ("start", "upper", "goal")
    initial_g_identity = id(planner.g)

    upper_closed = {("upper", "goal")}
    snapshot = _snapshot(1, closed_edges=upper_closed)
    result = planner.replan(snapshot, "start", "goal")
    _assert_matches_oracles(snapshot, "start", "goal", result)
    assert result.path == ("start", "lower", "goal")

    all_closed = {("upper", "goal"), ("lower", "goal")}
    snapshot = _snapshot(2, closed_edges=all_closed)
    result = planner.replan(snapshot, "start", "goal")
    _assert_matches_oracles(snapshot, "start", "goal", result)

    lower_only = {("upper", "goal")}
    snapshot = _snapshot(3, closed_edges=lower_only)
    result = planner.replan(snapshot, "lower", "goal")
    _assert_matches_oracles(snapshot, "lower", "goal", result)
    assert result.path == ("lower", "goal")

    snapshot = _snapshot(4)
    result = planner.replan(snapshot, "lower", "goal")
    _assert_matches_oracles(snapshot, "lower", "goal", result)

    assert id(planner.g) == initial_g_identity
    assert planner.reset_count == 1
    assert planner.state_reuse_count == 4
    assert planner.total_expanded_nodes > 0
    assert planner.last_changed_arcs == (("goal", "upper"), ("upper", "goal"))


def test_goal_change_resets_state_and_matches_oracles() -> None:
    planner = DStarLitePlanner()
    initial = _snapshot(0)
    planner.reset(initial, "start", "goal")
    previous_g_identity = id(planner.g)

    changed_mission = _snapshot(1, mission_revision=1)
    result = planner.replan(changed_mission, "start", "waiting")
    _assert_matches_oracles(changed_mission, "start", "waiting", result)

    assert result.path == ("start", "upper", "waiting")
    assert planner.reset_count == 2
    assert planner.state_reuse_count == 0
    assert id(planner.g) != previous_g_identity


def test_mission_change_with_same_map_revision_is_not_stale() -> None:
    planner = DStarLitePlanner()
    initial = _snapshot(2, mission_revision=0)
    planner.reset(initial, "start", "goal")

    moved_start = _snapshot(2, mission_revision=1)
    moved_result = planner.replan(moved_start, "lower", "goal")
    _assert_matches_oracles(moved_start, "lower", "goal", moved_result)

    changed_goal = _snapshot(2, mission_revision=2)
    goal_result = planner.replan(changed_goal, "lower", "waiting")
    _assert_matches_oracles(changed_goal, "lower", "waiting", goal_result)


def test_map_change_without_map_revision_is_rejected() -> None:
    planner = DStarLitePlanner()
    initial = _snapshot(2)
    planner.reset(initial, "start", "goal")

    tampered = _snapshot(2, closed_edges={("upper", "goal")})
    result = planner.replan(tampered, "start", "goal")

    assert result.status is PlanStatus.STALE_RESULT
    assert result.failure_reason == "map_content_changed_without_map_revision"


def test_regressed_snapshot_is_rejected_without_mutating_incremental_state() -> None:
    planner = DStarLitePlanner()
    current = _snapshot(2, closed_edges={("upper", "goal")})
    planner.reset(current, "start", "goal")
    reset_count = planner.reset_count
    reuse_count = planner.state_reuse_count

    stale = _snapshot(1)
    result = planner.replan(stale, "start", "goal")

    assert result.status is PlanStatus.STALE_RESULT
    assert result.failure_reason == "map_revision_regressed"
    assert result.map_revision == 1
    assert planner.reset_count == reset_count
    assert planner.state_reuse_count == reuse_count


def test_unknown_start_returns_invalid_input_with_snapshot_revisions() -> None:
    planner = DStarLitePlanner()
    snapshot = _snapshot(5, mission_revision=3)

    result = planner.reset(snapshot, "missing", "goal")

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "unknown_start_or_goal"
    assert result.map_revision == 5
    assert result.mission_revision == 3
