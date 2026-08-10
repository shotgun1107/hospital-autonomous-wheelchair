from math import isclose

import networkx as nx
import pytest

from hospital_path_lab.graph import canonical_edge
from hospital_path_lab.planners import AStarPlanner, DijkstraPlanner, Planner, SearchStatus
from hospital_path_lab.scenario import ScenarioSuite


@pytest.mark.parametrize("planner", [DijkstraPlanner(), AStarPlanner()], ids=lambda p: p.name)
def test_all_scenarios_match_expected(suite: ScenarioSuite, planner: Planner) -> None:
    for case in suite.cases:
        graph = suite.graph_for(case)
        result = planner.plan(graph, case.start, case.goal)
        assert result.status is case.expected_status, case.name
        if case.expected_cost is not None:
            assert result.cost is not None
            assert isclose(result.cost, case.expected_cost, rel_tol=1e-9), case.name
            assert isclose(graph.path_cost(result.path), result.cost, rel_tol=1e-9)
        else:
            assert result.cost is None
            assert result.path == ()


def test_planners_match_networkx_oracle(suite: ScenarioSuite) -> None:
    planners = [DijkstraPlanner(), AStarPlanner()]
    for case in suite.cases:
        graph = suite.graph_for(case)
        oracle = nx.DiGraph() if graph.directed else nx.Graph()
        for node_id in graph.nodes:
            oracle.add_node(node_id)
        for edge in graph.edges:
            key = canonical_edge(edge.source, edge.target, directed=graph.directed)
            if key not in graph.closed_edges:
                oracle.add_edge(edge.source, edge.target, weight=edge.cost)

        try:
            expected_cost = nx.shortest_path_length(oracle, case.start, case.goal, weight="weight")
        except nx.NetworkXNoPath:
            expected_cost = None

        for planner in planners:
            result = planner.plan(graph, case.start, case.goal)
            if expected_cost is None:
                assert result.status is SearchStatus.NO_PATH
            else:
                assert result.status is SearchStatus.FOUND
                assert result.cost is not None
                assert isclose(result.cost, expected_cost, rel_tol=1e-9)


def test_blocked_corridor_uses_lower_route(suite: ScenarioSuite) -> None:
    case = next(case for case in suite.cases if case.name == "upper_corridor_blocked")
    result = AStarPlanner().plan(suite.graph_for(case), case.start, case.goal)
    assert "lower_left" in result.path
    assert "lower_right" in result.path
    assert "upper_left" not in result.path


def test_astar_expands_no_more_nodes_than_dijkstra_on_normal_case(
    suite: ScenarioSuite,
) -> None:
    case = next(case for case in suite.cases if case.name == "normal")
    graph = suite.graph_for(case)
    dijkstra = DijkstraPlanner().plan(graph, case.start, case.goal)
    astar = AStarPlanner().plan(graph, case.start, case.goal)
    assert astar.expanded_nodes <= dijkstra.expanded_nodes
