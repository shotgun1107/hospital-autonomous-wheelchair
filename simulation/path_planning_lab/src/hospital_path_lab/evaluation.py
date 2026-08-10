"""후보 생성과 독립된 경로 결과 검증·성능 측정 도구."""

from __future__ import annotations

import tracemalloc
from collections.abc import Iterable
from dataclasses import dataclass, replace
from math import isclose
from statistics import median
from time import perf_counter_ns

import networkx as nx
import numpy as np

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import (
    FollowerResult,
    GraphSnapshot,
    GridSnapshot,
    LocalPlanResult,
    PlanStatus,
    Pose2D,
    SnapshotMetadata,
)
from hospital_path_lab.graph import canonical_edge
from hospital_path_lab.planners import Planner, SearchResult
from hospital_path_lab.safety import AutomaticResumeGate, MotionState


@dataclass(frozen=True, slots=True)
class ValidationOutcome:
    passed: bool
    executable: bool
    failures: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class PerformanceSummary:
    samples: int
    elapsed_ns_p50: int
    elapsed_ns_p95: int
    elapsed_ns_p99: int
    elapsed_ns_worst: int
    peak_memory_bytes: int


@dataclass(frozen=True, slots=True)
class GlobalEvaluationRecord:
    planner: str
    map_id: str
    map_revision: int
    mission_revision: int
    status: str
    path: tuple[str, ...]
    cost: float | None
    expanded_nodes: int
    oracle_matched: bool
    deterministic: bool
    route_churn: float
    validation_failures: tuple[str, ...]
    performance: PerformanceSummary


def validate_result_provenance(
    input_metadata: SnapshotMetadata,
    result: object,
    *,
    current_metadata: SnapshotMetadata | None = None,
) -> ValidationOutcome:
    """역할과 무관하게 결과의 지도 identity·revision·hash 최신성을 검사한다."""

    current = current_metadata or input_metadata
    failures = _provenance_failures(
        getattr(result, "map_id", ""),
        getattr(result, "map_revision", -1),
        getattr(result, "mission_revision", -1),
        getattr(result, "observation_revision", -1),
        getattr(result, "input_content_hash", ""),
        input_metadata,
        current,
    )
    return ValidationOutcome(
        passed=not failures,
        executable=not failures,
        failures=tuple(failures),
    )


def run_stateless_global(
    planner: Planner,
    snapshot: GraphSnapshot,
    start: str,
    goal: str,
) -> SearchResult:
    metadata = snapshot.metadata
    if not snapshot.input_valid:
        return SearchResult(
            planner=planner.name,
            status=PlanStatus.INVALID_INPUT,
            path=(),
            cost=None,
            expanded_nodes=0,
            elapsed_ns=0,
            map_id=metadata.map_id,
            map_revision=metadata.map_revision,
            mission_revision=metadata.mission_revision,
            observation_revision=metadata.observation_revision,
            input_content_hash=metadata.content_hash,
            failure_reason="snapshot_input_invalidated",
        )
    result = planner.plan(snapshot.graph, start, goal)
    return replace(
        result,
        map_id=metadata.map_id,
        map_revision=metadata.map_revision,
        mission_revision=metadata.mission_revision,
        observation_revision=metadata.observation_revision,
        input_content_hash=metadata.content_hash,
    )


def validate_global_result(
    snapshot: GraphSnapshot,
    start: str,
    goal: str,
    result: SearchResult,
    *,
    current_metadata: SnapshotMetadata | None = None,
) -> ValidationOutcome:
    current = current_metadata or snapshot.metadata
    failures = list(
        validate_result_provenance(
            snapshot.metadata,
            result,
            current_metadata=current,
        ).failures
    )

    if "input_invalidated" in failures:
        return ValidationOutcome(False, False, tuple(failures))

    oracle_status, oracle_cost = _graph_oracle(snapshot, start, goal)
    if result.status is not oracle_status:
        failures.append("oracle_status_mismatch")

    if result.status is PlanStatus.FOUND:
        if not result.path or result.path[0] != start or result.path[-1] != goal:
            failures.append("invalid_endpoints")
        path_cost = snapshot.graph.path_cost(result.path)
        if not np.isfinite(path_cost):
            failures.append("path_collision_or_disconnection")
        if result.cost is None or not isclose(path_cost, result.cost, rel_tol=1e-9):
            failures.append("reported_cost_mismatch")
        if oracle_cost is None or result.cost is None or not isclose(
            result.cost, oracle_cost, rel_tol=1e-9
        ):
            failures.append("oracle_cost_mismatch")
    elif result.path or result.cost is not None:
        failures.append("nonempty_failure_result")

    return ValidationOutcome(
        passed=not failures,
        executable=not failures and result.status is PlanStatus.FOUND,
        failures=tuple(failures),
    )


def validate_local_result(
    snapshot: GridSnapshot,
    start: Pose2D,
    goal: Pose2D,
    result: LocalPlanResult,
    *,
    current_metadata: SnapshotMetadata | None = None,
    require_goal: bool = True,
) -> ValidationOutcome:
    current = current_metadata or snapshot.metadata
    failures = list(
        validate_result_provenance(
            snapshot.metadata,
            result,
            current_metadata=current,
        ).failures
    )

    if "input_invalidated" in failures:
        return ValidationOutcome(False, False, tuple(failures))

    if result.status is PlanStatus.FOUND:
        if not result.path:
            failures.append("empty_found_path")
        else:
            cell_start = snapshot.grid.world_to_cell(start)
            cell_goal = snapshot.grid.world_to_cell(goal)
            if snapshot.grid.world_to_cell(result.path[0]) != cell_start:
                failures.append("invalid_start")
            if require_goal and snapshot.grid.world_to_cell(result.path[-1]) != cell_goal:
                failures.append("invalid_goal")
            checker = CollisionChecker(
                snapshot.grid, forbidden_cells=snapshot.forbidden_cells
            )
            forbidden_entry = checker.path_enters_forbidden(result.path)
            if forbidden_entry:
                failures.append("forbidden_zone_entry")
            obstacle_checker = CollisionChecker(snapshot.grid)
            if not forbidden_entry and (
                not obstacle_checker.conservative_path_is_collision_free(result.path)
                or result.collision
            ):
                failures.append("collision")
    elif result.path or result.trajectory:
        failures.append("nonempty_failure_result")

    return ValidationOutcome(
        passed=not failures,
        executable=not failures and result.status is PlanStatus.FOUND,
        failures=tuple(failures),
    )


def validate_follower_result(
    metadata: SnapshotMetadata,
    result: FollowerResult,
    *,
    current_metadata: SnapshotMetadata | None = None,
) -> ValidationOutcome:
    """Follower 명령도 동일한 입력 출처와 최신성 계약으로 검증한다."""

    current = current_metadata or metadata
    failures = list(
        validate_result_provenance(
            metadata,
            result,
            current_metadata=current,
        ).failures
    )
    if not np.isfinite(result.command.linear) or not np.isfinite(result.command.angular):
        failures.append("nonfinite_command")
    return ValidationOutcome(
        passed=not failures,
        executable=not failures and result.status is PlanStatus.FOUND,
        failures=tuple(failures),
    )


def _provenance_failures(
    result_map_id: str,
    result_map_revision: int,
    result_mission_revision: int,
    result_observation_revision: int,
    result_content_hash: str,
    input_metadata: SnapshotMetadata,
    current_metadata: SnapshotMetadata,
) -> list[str]:
    failures: list[str] = []
    if not input_metadata.input_valid or not current_metadata.input_valid:
        failures.append("input_invalidated")
    if result_map_id != current_metadata.map_id:
        failures.append("stale_map_id")
    if (
        result_map_revision,
        result_mission_revision,
        result_observation_revision,
    ) != (
        current_metadata.map_revision,
        current_metadata.mission_revision,
        current_metadata.observation_revision,
    ):
        failures.append("stale_revision")
    if result_content_hash != current_metadata.content_hash:
        failures.append("stale_content_hash")
    return failures


def authorize_after_protective_stop(
    validation: ValidationOutcome,
    gate: AutomaticResumeGate,
) -> bool:
    return bool(
        validation.executable
        and gate.state is MotionState.MOVING
        and gate.stop_confirmed
        and gate.path_revalidated
        and gate.resume_instruction_revalidated
        and gate.local_safety_authorized
        and not gate.hazard_active
    )


def benchmark_global(
    planner: Planner,
    snapshot: GraphSnapshot,
    start: str,
    goal: str,
    *,
    repeats: int,
    previous_path: tuple[str, ...] = (),
) -> GlobalEvaluationRecord:
    if repeats < 1:
        raise ValueError("repeats는 1 이상이어야 합니다.")

    tracemalloc.start()
    results: list[SearchResult] = []
    elapsed: list[int] = []
    try:
        for _ in range(repeats):
            started = perf_counter_ns()
            results.append(run_stateless_global(planner, snapshot, start, goal))
            elapsed.append(perf_counter_ns() - started)
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()

    representative = results[0]
    signatures = {
        (result.status, result.path, result.cost, result.expanded_nodes) for result in results
    }
    validation = validate_global_result(snapshot, start, goal, representative)
    array = np.asarray(elapsed, dtype=np.int64)
    return GlobalEvaluationRecord(
        planner=planner.name,
        map_id=snapshot.metadata.map_id,
        map_revision=snapshot.metadata.map_revision,
        mission_revision=snapshot.metadata.mission_revision,
        status=representative.status.value,
        path=representative.path,
        cost=representative.cost,
        expanded_nodes=representative.expanded_nodes,
        oracle_matched=not any(item.startswith("oracle_") for item in validation.failures),
        deterministic=len(signatures) == 1,
        route_churn=route_churn(previous_path, representative.path),
        validation_failures=validation.failures,
        performance=PerformanceSummary(
            samples=repeats,
            elapsed_ns_p50=int(median(elapsed)),
            elapsed_ns_p95=int(np.percentile(array, 95)),
            elapsed_ns_p99=int(np.percentile(array, 99)),
            elapsed_ns_worst=max(elapsed),
            peak_memory_bytes=int(peak),
        ),
    )


def route_churn(previous: tuple[str, ...], current: tuple[str, ...]) -> float:
    previous_edges = _path_edges(previous)
    current_edges = _path_edges(current)
    union = previous_edges | current_edges
    if not union:
        return 0.0
    return len(previous_edges ^ current_edges) / len(union)


def records_as_dicts(records: Iterable[GlobalEvaluationRecord]) -> list[dict[str, object]]:
    return [
        {
            "planner": record.planner,
            "map_id": record.map_id,
            "map_revision": record.map_revision,
            "mission_revision": record.mission_revision,
            "status": record.status,
            "path": list(record.path),
            "cost": record.cost,
            "expanded_nodes": record.expanded_nodes,
            "oracle_matched": record.oracle_matched,
            "deterministic": record.deterministic,
            "route_churn": record.route_churn,
            "validation_failures": list(record.validation_failures),
            "performance": {
                "samples": record.performance.samples,
                "elapsed_ns_p50": record.performance.elapsed_ns_p50,
                "elapsed_ns_p95": record.performance.elapsed_ns_p95,
                "elapsed_ns_p99": record.performance.elapsed_ns_p99,
                "elapsed_ns_worst": record.performance.elapsed_ns_worst,
                "peak_memory_bytes": record.performance.peak_memory_bytes,
            },
        }
        for record in records
    ]


def _graph_oracle(
    snapshot: GraphSnapshot,
    start: str,
    goal: str,
) -> tuple[PlanStatus, float | None]:
    graph = snapshot.graph
    if start not in graph.nodes or goal not in graph.nodes:
        return PlanStatus.INVALID_INPUT, None
    oracle = nx.DiGraph() if graph.directed else nx.Graph()
    oracle.add_nodes_from(graph.nodes)
    for edge in graph.edges:
        key = canonical_edge(edge.source, edge.target, directed=graph.directed)
        if key not in graph.closed_edges:
            oracle.add_edge(edge.source, edge.target, weight=edge.cost)
    try:
        cost = nx.shortest_path_length(oracle, start, goal, weight="weight")
    except nx.NetworkXNoPath:
        return PlanStatus.NO_PATH, None
    return PlanStatus.FOUND, float(cost)


def _path_edges(path: tuple[str, ...]) -> set[tuple[str, str]]:
    return {
        tuple(sorted((source, target))) for source, target in zip(path, path[1:], strict=False)
    }
