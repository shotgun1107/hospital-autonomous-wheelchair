"""동일 시나리오에서 planner 결과와 계산 지표를 수집한다."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import asdict, dataclass
from math import isclose

import numpy as np

from hospital_path_lab.planners import Planner, SearchResult
from hospital_path_lab.scenario import ScenarioCase, ScenarioSuite


@dataclass(frozen=True, slots=True)
class BenchmarkRecord:
    scenario: str
    planner: str
    status: str
    path: tuple[str, ...]
    cost: float | None
    expanded_nodes: int
    elapsed_ns_p50: int
    elapsed_ns_p95: int
    repeats: int
    expected_result_matched: bool

    def as_json_dict(self) -> dict[str, object]:
        result = asdict(self)
        result["path"] = list(self.path)
        return result


def benchmark_suite(
    suite: ScenarioSuite,
    planners: Iterable[Planner],
    *,
    repeats: int,
) -> list[BenchmarkRecord]:
    if repeats < 1:
        raise ValueError("repeats는 1 이상이어야 합니다.")

    records: list[BenchmarkRecord] = []
    for case in suite.cases:
        graph = suite.graph_for(case)
        for planner in planners:
            results = [planner.plan(graph, case.start, case.goal) for _ in range(repeats)]
            representative = results[0]
            _assert_deterministic(results)
            elapsed = np.asarray([result.elapsed_ns for result in results], dtype=np.int64)
            records.append(
                BenchmarkRecord(
                    scenario=case.name,
                    planner=planner.name,
                    status=representative.status.value,
                    path=representative.path,
                    cost=representative.cost,
                    expanded_nodes=representative.expanded_nodes,
                    elapsed_ns_p50=int(np.percentile(elapsed, 50)),
                    elapsed_ns_p95=int(np.percentile(elapsed, 95)),
                    repeats=repeats,
                    expected_result_matched=_matches_expected(case, representative),
                )
            )
    return records


def _assert_deterministic(results: list[SearchResult]) -> None:
    signatures = {
        (result.status, result.path, result.cost, result.expanded_nodes) for result in results
    }
    if len(signatures) != 1:
        raise RuntimeError("동일 입력에서 planner 결과가 결정론적이지 않습니다.")


def _matches_expected(case: ScenarioCase, result: SearchResult) -> bool:
    if result.status is not case.expected_status:
        return False
    if case.expected_cost is None:
        return result.cost is None
    return result.cost is not None and isclose(result.cost, case.expected_cost, rel_tol=1e-9)
