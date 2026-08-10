"""YAML 시나리오를 그래프와 반복 가능한 시험 사례로 변환한다."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from hospital_path_lab.graph import Edge, GraphMap, Node
from hospital_path_lab.planners import SearchStatus


@dataclass(frozen=True, slots=True)
class ScenarioCase:
    name: str
    description: str
    start: str
    goal: str
    closed_edges: frozenset[tuple[str, str]]
    expected_status: SearchStatus
    expected_cost: float | None = None


@dataclass(frozen=True, slots=True)
class ScenarioSuite:
    nodes: tuple[Node, ...]
    edges: tuple[Edge, ...]
    directed: bool
    cases: tuple[ScenarioCase, ...]

    def graph_for(self, case: ScenarioCase) -> GraphMap:
        return GraphMap(
            list(self.nodes),
            list(self.edges),
            directed=self.directed,
            closed_edges=set(case.closed_edges),
        )


def load_scenario_suite(path: str | Path) -> ScenarioSuite:
    scenario_path = Path(path)
    with scenario_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError("시나리오 YAML의 최상위 값은 객체여야 합니다.")

    map_data = _mapping(raw.get("map"), "map")
    node_data = _mapping(map_data.get("nodes"), "map.nodes")
    nodes = tuple(
        Node(node_id=str(node_id), x=float(coords[0]), y=float(coords[1]))
        for node_id, coords in node_data.items()
    )
    edges = tuple(
        Edge(source=str(item["from"]), target=str(item["to"]), cost=float(item["cost"]))
        for item in _list(map_data.get("edges"), "map.edges")
    )
    directed = bool(map_data.get("directed", False))

    cases: list[ScenarioCase] = []
    for item in _list(raw.get("cases"), "cases"):
        case = _mapping(item, "cases[]")
        expected_status = SearchStatus(str(case["expected_status"]))
        expected_cost_value = case.get("expected_cost")
        cases.append(
            ScenarioCase(
                name=str(case["name"]),
                description=str(case.get("description", "")),
                start=str(case["start"]),
                goal=str(case["goal"]),
                closed_edges=frozenset(
                    (str(edge[0]), str(edge[1]))
                    for edge in _list(case.get("closed_edges", []), "closed_edges")
                ),
                expected_status=expected_status,
                expected_cost=(
                    float(expected_cost_value) if expected_cost_value is not None else None
                ),
            )
        )

    suite = ScenarioSuite(nodes=nodes, edges=edges, directed=directed, cases=tuple(cases))
    for case in suite.cases:
        suite.graph_for(case)
    return suite


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label}은 객체여야 합니다.")
    return value


def _list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise ValueError(f"{label}은 목록이어야 합니다.")
    return value
