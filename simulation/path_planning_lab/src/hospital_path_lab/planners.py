"""동일한 출력 계약을 사용하는 Dijkstra와 A* 기준 구현."""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from math import inf
from time import perf_counter_ns
from typing import Protocol

from hospital_path_lab.contracts import PlanStatus
from hospital_path_lab.graph import GraphMap

SearchStatus = PlanStatus


@dataclass(frozen=True, slots=True)
class SearchResult:
    planner: str
    status: SearchStatus
    path: tuple[str, ...]
    cost: float | None
    expanded_nodes: int
    elapsed_ns: int
    map_revision: int = 0
    mission_revision: int = 0
    observation_revision: int = 0
    map_id: str = ""
    input_content_hash: str = ""
    peak_memory_bytes: int = 0
    failure_reason: str | None = None


class Planner(Protocol):
    name: str

    def plan(self, graph: GraphMap, start: str, goal: str) -> SearchResult: ...


class DijkstraPlanner:
    name = "dijkstra"

    def plan(self, graph: GraphMap, start: str, goal: str) -> SearchResult:
        return _best_first_search(graph, start, goal, planner_name=self.name, use_heuristic=False)


class AStarPlanner:
    name = "astar"

    def plan(self, graph: GraphMap, start: str, goal: str) -> SearchResult:
        return _best_first_search(graph, start, goal, planner_name=self.name, use_heuristic=True)


def _best_first_search(
    graph: GraphMap,
    start: str,
    goal: str,
    *,
    planner_name: str,
    use_heuristic: bool,
) -> SearchResult:
    started_at = perf_counter_ns()
    if start not in graph.nodes or goal not in graph.nodes:
        return SearchResult(
            planner=planner_name,
            status=SearchStatus.INVALID_INPUT,
            path=(),
            cost=None,
            expanded_nodes=0,
            elapsed_ns=perf_counter_ns() - started_at,
        )

    frontier: list[tuple[float, float, str]] = []
    start_h = graph.heuristic(start, goal) if use_heuristic else 0.0
    heappush(frontier, (start_h, 0.0, start))
    came_from: dict[str, str] = {}
    best_cost: dict[str, float] = {start: 0.0}
    expanded_nodes = 0

    while frontier:
        _, current_cost, current = heappop(frontier)
        if current_cost > best_cost.get(current, inf):
            continue

        expanded_nodes += 1
        if current == goal:
            path = _reconstruct_path(came_from, start, goal)
            return SearchResult(
                planner=planner_name,
                status=SearchStatus.FOUND,
                path=path,
                cost=current_cost,
                expanded_nodes=expanded_nodes,
                elapsed_ns=perf_counter_ns() - started_at,
            )

        for neighbor, edge_cost in graph.neighbors(current):
            candidate_cost = current_cost + edge_cost
            if candidate_cost >= best_cost.get(neighbor, inf):
                continue
            best_cost[neighbor] = candidate_cost
            came_from[neighbor] = current
            heuristic = graph.heuristic(neighbor, goal) if use_heuristic else 0.0
            heappush(frontier, (candidate_cost + heuristic, candidate_cost, neighbor))

    return SearchResult(
        planner=planner_name,
        status=SearchStatus.NO_PATH,
        path=(),
        cost=None,
        expanded_nodes=expanded_nodes,
        elapsed_ns=perf_counter_ns() - started_at,
    )


def _reconstruct_path(came_from: dict[str, str], start: str, goal: str) -> tuple[str, ...]:
    path = [goal]
    current = goal
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return tuple(path)
