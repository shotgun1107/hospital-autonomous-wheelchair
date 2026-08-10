"""변경된 간선과 이동한 시작점에서 탐색 상태를 재사용하는 D* Lite."""

from __future__ import annotations

from heapq import heappop, heappush
from math import inf, isfinite
from time import perf_counter_ns

from hospital_path_lab.contracts import GraphSnapshot, PlanStatus
from hospital_path_lab.graph import GraphMap, canonical_edge
from hospital_path_lab.planners import SearchResult

Key = tuple[float, float]
Arc = tuple[str, str]


class _PriorityQueue:
    """노드 ID를 최종 tie-break로 사용하는 lazy deletion 우선순위 큐."""

    def __init__(self) -> None:
        self._heap: list[tuple[float, float, str]] = []
        self._live: dict[str, Key] = {}

    def clear(self) -> None:
        self._heap.clear()
        self._live.clear()

    def discard(self, node_id: str) -> None:
        self._live.pop(node_id, None)

    def insert(self, node_id: str, key: Key) -> None:
        self._live[node_id] = key
        heappush(self._heap, (key[0], key[1], node_id))

    def top_key(self) -> Key:
        self._discard_stale_entries()
        if not self._heap:
            return inf, inf
        first, second, _ = self._heap[0]
        return first, second

    def pop(self) -> tuple[Key, str]:
        self._discard_stale_entries()
        first, second, node_id = heappop(self._heap)
        del self._live[node_id]
        return (first, second), node_id

    def _discard_stale_entries(self) -> None:
        while self._heap:
            first, second, node_id = self._heap[0]
            if self._live.get(node_id) == (first, second):
                return
            heappop(self._heap)


class DStarLitePlanner:
    """표준 D* Lite 상태를 snapshot 사이에서 유지하는 전역 planner.

    `expanded_nodes`는 해당 호출에서 실제로 큐에서 확장한 노드 수다.
    `total_expanded_nodes`, `reset_count`, `state_reuse_count`는 증분 상태가
    실제로 재사용됐는지 시험에서 관찰하기 위한 누적 계수다.
    """

    name = "dstar_lite"

    def __init__(self) -> None:
        self.g: dict[str, float] = {}
        self.rhs: dict[str, float] = {}
        self.km = 0.0
        self.total_expanded_nodes = 0
        self.reset_count = 0
        self.state_reuse_count = 0
        self.last_changed_arcs: tuple[Arc, ...] = ()

        self._queue = _PriorityQueue()
        self._snapshot: GraphSnapshot | None = None
        self._graph: GraphMap | None = None
        self._start: str | None = None
        self._last_start: str | None = None
        self._goal: str | None = None
        self._base_signature: tuple[object, ...] | None = None
        self._active_costs: dict[Arc, float] = {}

    def reset(self, snapshot: GraphSnapshot, start: str, goal: str) -> SearchResult:
        """기존 상태를 폐기하고 주어진 snapshot에서 새 탐색을 시작한다."""

        started_at = perf_counter_ns()
        if not snapshot.input_valid:
            self._clear_state()
            return self._result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                started_at=started_at,
                expanded_nodes=0,
                failure_reason="snapshot_input_invalidated",
            )
        graph = snapshot.graph
        invalid_reason = self._validate_request(graph, start, goal)
        if invalid_reason is not None:
            self._clear_state()
            return self._result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                started_at=started_at,
                expanded_nodes=0,
                failure_reason=invalid_reason,
            )

        self._snapshot = snapshot
        self._graph = graph
        self._start = start
        self._last_start = start
        self._goal = goal
        self._base_signature = _base_graph_signature(graph)
        self._active_costs = _active_arc_costs(graph)
        self.g = {node_id: inf for node_id in graph.nodes}
        self.rhs = {node_id: inf for node_id in graph.nodes}
        self.rhs[goal] = 0.0
        self.km = 0.0
        self.last_changed_arcs = ()
        self._queue.clear()
        self._queue.insert(goal, self._calculate_key(goal))
        self.reset_count += 1

        expanded_nodes = self._compute_shortest_path()
        return self._build_path_result(snapshot, started_at, expanded_nodes)

    def replan(self, snapshot: GraphSnapshot, start: str, goal: str) -> SearchResult:
        """snapshot delta와 시작점 이동을 반영해 이전 탐색 결과를 재사용한다."""

        started_at = perf_counter_ns()
        if not snapshot.input_valid:
            return self._result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                started_at=started_at,
                expanded_nodes=0,
                failure_reason="snapshot_input_invalidated",
            )
        if self._snapshot is None or self._graph is None:
            return self.reset(snapshot, start, goal)

        invalid_reason = self._validate_request(snapshot.graph, start, goal)
        if invalid_reason is not None:
            return self._result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                started_at=started_at,
                expanded_nodes=0,
                failure_reason=invalid_reason,
            )

        map_changed = snapshot.metadata.map_id != self._snapshot.metadata.map_id
        if map_changed:
            return self.reset(snapshot, start, goal)

        stale_reason = self._stale_snapshot_reason(snapshot)
        if stale_reason is not None:
            return self._result(
                snapshot,
                status=PlanStatus.STALE_RESULT,
                started_at=started_at,
                expanded_nodes=0,
                failure_reason=stale_reason,
            )

        graph_changed = _base_graph_signature(snapshot.graph) != self._base_signature
        if goal != self._goal or graph_changed:
            return self.reset(snapshot, start, goal)

        assert self._start is not None
        assert self._last_start is not None
        self.km += self._graph.heuristic(self._last_start, start)
        self._start = start
        self._last_start = start

        previous_costs = self._active_costs
        current_costs = _active_arc_costs(snapshot.graph)
        changed_arcs = tuple(
            sorted(
                arc
                for arc in previous_costs.keys() | current_costs.keys()
                if previous_costs.get(arc, inf) != current_costs.get(arc, inf)
            )
        )

        self._snapshot = snapshot
        self._graph = snapshot.graph
        self._active_costs = current_costs
        self.last_changed_arcs = changed_arcs
        for source in sorted({source for source, _ in changed_arcs}):
            self._update_vertex(source)

        self.state_reuse_count += 1
        expanded_nodes = self._compute_shortest_path()
        return self._build_path_result(snapshot, started_at, expanded_nodes)

    def _compute_shortest_path(self) -> int:
        assert self._start is not None
        expanded_nodes = 0
        while (
            self._queue.top_key() < self._calculate_key(self._start)
            or self.rhs[self._start] != self.g[self._start]
        ):
            old_key, node_id = self._queue.pop()
            new_key = self._calculate_key(node_id)
            if old_key < new_key:
                self._queue.insert(node_id, new_key)
            elif self.g[node_id] > self.rhs[node_id]:
                self.g[node_id] = self.rhs[node_id]
                for predecessor in self._predecessors(node_id):
                    self._update_vertex(predecessor)
            else:
                self.g[node_id] = inf
                self._update_vertex(node_id)
                for predecessor in self._predecessors(node_id):
                    self._update_vertex(predecessor)
            expanded_nodes += 1

        self.total_expanded_nodes += expanded_nodes
        return expanded_nodes

    def _update_vertex(self, node_id: str) -> None:
        assert self._graph is not None
        assert self._goal is not None
        if node_id != self._goal:
            candidates = (
                edge_cost + self.g[successor]
                for successor, edge_cost in self._graph.neighbors(node_id)
            )
            self.rhs[node_id] = min(candidates, default=inf)
        self._queue.discard(node_id)
        if self.g[node_id] != self.rhs[node_id]:
            self._queue.insert(node_id, self._calculate_key(node_id))

    def _calculate_key(self, node_id: str) -> Key:
        assert self._graph is not None
        assert self._start is not None
        best = min(self.g[node_id], self.rhs[node_id])
        return best + self._graph.heuristic(self._start, node_id) + self.km, best

    def _predecessors(self, node_id: str) -> tuple[str, ...]:
        return tuple(
            sorted(source for source, target in self._active_costs if target == node_id)
        )

    def _build_path_result(
        self,
        snapshot: GraphSnapshot,
        started_at: int,
        expanded_nodes: int,
    ) -> SearchResult:
        assert self._graph is not None
        assert self._start is not None
        assert self._goal is not None
        if not isfinite(self.g[self._start]):
            return self._result(
                snapshot,
                status=PlanStatus.NO_PATH,
                started_at=started_at,
                expanded_nodes=expanded_nodes,
                failure_reason="no_path",
            )

        path = [self._start]
        total_cost = 0.0
        current = self._start
        visited = {current}
        while current != self._goal:
            candidates = sorted(
                (
                    edge_cost + self.g[successor],
                    successor,
                    edge_cost,
                )
                for successor, edge_cost in self._graph.neighbors(current)
                if isfinite(self.g[successor])
            )
            if not candidates:
                return self._result(
                    snapshot,
                    status=PlanStatus.NO_PATH,
                    started_at=started_at,
                    expanded_nodes=expanded_nodes,
                    failure_reason="path_extraction_failed",
                )
            _, successor, edge_cost = candidates[0]
            if successor in visited:
                return self._result(
                    snapshot,
                    status=PlanStatus.NO_PATH,
                    started_at=started_at,
                    expanded_nodes=expanded_nodes,
                    failure_reason="path_cycle_detected",
                )
            path.append(successor)
            visited.add(successor)
            total_cost += edge_cost
            current = successor

        return self._result(
            snapshot,
            status=PlanStatus.FOUND,
            started_at=started_at,
            expanded_nodes=expanded_nodes,
            path=tuple(path),
            cost=total_cost,
        )

    def _result(
        self,
        snapshot: GraphSnapshot,
        *,
        status: PlanStatus,
        started_at: int,
        expanded_nodes: int,
        path: tuple[str, ...] = (),
        cost: float | None = None,
        failure_reason: str | None = None,
    ) -> SearchResult:
        metadata = snapshot.metadata
        return SearchResult(
            planner=self.name,
            status=status,
            path=path,
            cost=cost,
            expanded_nodes=expanded_nodes,
            elapsed_ns=perf_counter_ns() - started_at,
            map_id=metadata.map_id,
            map_revision=metadata.map_revision,
            mission_revision=metadata.mission_revision,
            observation_revision=metadata.observation_revision,
            input_content_hash=metadata.content_hash,
            failure_reason=failure_reason,
        )

    def _validate_request(self, graph: GraphMap, start: str, goal: str) -> str | None:
        if start not in graph.nodes or goal not in graph.nodes:
            return "unknown_start_or_goal"
        if any(edge.cost <= 0 for edge in graph.edges):
            return "non_positive_edge_cost"
        return None

    def _stale_snapshot_reason(self, snapshot: GraphSnapshot) -> str | None:
        assert self._snapshot is not None
        previous = self._snapshot.metadata
        current = snapshot.metadata
        if current.map_revision < previous.map_revision:
            return "map_revision_regressed"
        if current.mission_revision < previous.mission_revision:
            return "mission_revision_regressed"
        if current.observation_revision < previous.observation_revision:
            return "observation_revision_regressed"
        # Snapshot content_hash에는 지도뿐 아니라 시작점·목적지와 모든 revision이
        # 들어간다. 따라서 mission 변경을 map 변조로 오인하지 말고, 실제 graph
        # topology/비용/폐쇄 상태만 같은 map revision에서 달라졌는지 검사한다.
        if current.map_revision == previous.map_revision:
            graph = snapshot.graph
            if (
                _base_graph_signature(graph) != self._base_signature
                or _active_arc_costs(graph) != self._active_costs
            ):
                return "map_content_changed_without_map_revision"
        same_revisions = (
            current.map_revision,
            current.mission_revision,
            current.observation_revision,
        ) == (
            previous.map_revision,
            previous.mission_revision,
            previous.observation_revision,
        )
        if same_revisions and current.content_hash != previous.content_hash:
            return "content_hash_changed_without_revision"
        return None

    def _clear_state(self) -> None:
        self.g.clear()
        self.rhs.clear()
        self.km = 0.0
        self.last_changed_arcs = ()
        self._queue.clear()
        self._snapshot = None
        self._graph = None
        self._start = None
        self._last_start = None
        self._goal = None
        self._base_signature = None
        self._active_costs = {}


def _active_arc_costs(graph: GraphMap) -> dict[Arc, float]:
    costs: dict[Arc, float] = {}
    for edge in graph.edges:
        key = canonical_edge(edge.source, edge.target, directed=graph.directed)
        if key in graph.closed_edges:
            continue
        arcs = [(edge.source, edge.target)]
        if not graph.directed:
            arcs.append((edge.target, edge.source))
        for arc in arcs:
            costs[arc] = min(costs.get(arc, inf), edge.cost)
    return costs


def _base_graph_signature(graph: GraphMap) -> tuple[object, ...]:
    nodes = tuple(sorted((node.node_id, node.x, node.y) for node in graph.nodes.values()))
    if graph.directed:
        edges = tuple(sorted((edge.source, edge.target, edge.cost) for edge in graph.edges))
    else:
        edges = tuple(
            sorted(
                (*canonical_edge(edge.source, edge.target, directed=False), edge.cost)
                for edge in graph.edges
            )
        )
    return graph.directed, nodes, edges
