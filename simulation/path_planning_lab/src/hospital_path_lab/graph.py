"""결정론적 전역 경로 비교를 위한 작은 등록 통로 그래프."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, inf


@dataclass(frozen=True, slots=True)
class Node:
    node_id: str
    x: float
    y: float


@dataclass(frozen=True, slots=True)
class Edge:
    source: str
    target: str
    cost: float


def canonical_edge(source: str, target: str, *, directed: bool) -> tuple[str, str]:
    if directed:
        return source, target
    return tuple(sorted((source, target)))


class GraphMap:
    """노드·간선과 현재 폐쇄 간선을 함께 보유하는 불변에 가까운 그래프."""

    def __init__(
        self,
        nodes: list[Node],
        edges: list[Edge],
        *,
        directed: bool = False,
        closed_edges: set[tuple[str, str]] | None = None,
    ) -> None:
        self.directed = directed
        self.nodes = {node.node_id: node for node in nodes}
        if len(self.nodes) != len(nodes):
            raise ValueError("node_id는 중복될 수 없습니다.")

        self.edges = tuple(edges)
        self._adjacency: dict[str, list[tuple[str, float]]] = {
            node_id: [] for node_id in self.nodes
        }
        for edge in self.edges:
            if edge.source not in self.nodes or edge.target not in self.nodes:
                raise ValueError(f"알 수 없는 노드가 간선에 포함됐습니다: {edge}")
            if edge.cost <= 0:
                raise ValueError(f"간선 비용은 양수여야 합니다: {edge}")
            self._adjacency[edge.source].append((edge.target, edge.cost))
            if not self.directed:
                self._adjacency[edge.target].append((edge.source, edge.cost))

        for neighbors in self._adjacency.values():
            neighbors.sort(key=lambda item: item[0])

        self.closed_edges = frozenset(
            canonical_edge(source, target, directed=directed)
            for source, target in (closed_edges or set())
        )
        known_edges = {
            canonical_edge(edge.source, edge.target, directed=directed) for edge in self.edges
        }
        unknown_closed = self.closed_edges - known_edges
        if unknown_closed:
            raise ValueError(f"그래프에 없는 간선을 폐쇄할 수 없습니다: {sorted(unknown_closed)}")

        self._heuristic_scale = self._calculate_heuristic_scale()

    def neighbors(self, node_id: str) -> tuple[tuple[str, float], ...]:
        result = []
        for neighbor, cost in self._adjacency[node_id]:
            key = canonical_edge(node_id, neighbor, directed=self.directed)
            if key not in self.closed_edges:
                result.append((neighbor, cost))
        return tuple(result)

    def heuristic(self, source: str, target: str) -> float:
        """모든 간선 비용에 대해 admissible한 좌표 기반 하한을 반환한다."""

        a = self.nodes[source]
        b = self.nodes[target]
        return self._heuristic_scale * hypot(a.x - b.x, a.y - b.y)

    def path_cost(self, path: tuple[str, ...]) -> float:
        if len(path) < 2:
            return 0.0
        total = 0.0
        for source, target in zip(path, path[1:], strict=False):
            matches = [cost for neighbor, cost in self.neighbors(source) if neighbor == target]
            if not matches:
                return inf
            total += matches[0]
        return total

    def _calculate_heuristic_scale(self) -> float:
        ratios: list[float] = []
        for edge in self.edges:
            source = self.nodes[edge.source]
            target = self.nodes[edge.target]
            distance = hypot(source.x - target.x, source.y - target.y)
            if distance > 0:
                ratios.append(edge.cost / distance)
        return min(ratios, default=0.0)
