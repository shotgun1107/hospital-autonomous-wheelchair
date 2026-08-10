from dataclasses import replace

from hospital_path_lab.contracts import GraphSnapshot, SnapshotMetadata
from hospital_path_lab.evaluation import (
    authorize_after_protective_stop,
    benchmark_global,
    route_churn,
    run_stateless_global,
    validate_global_result,
)
from hospital_path_lab.planners import AStarPlanner
from hospital_path_lab.safety import AutomaticResumeGate
from hospital_path_lab.scenario import ScenarioSuite


def _snapshot(suite: ScenarioSuite, case_name: str) -> tuple[GraphSnapshot, str, str]:
    case = next(case for case in suite.cases if case.name == case_name)
    metadata = SnapshotMetadata(
        map_id=case.name,
        map_revision=1,
        mission_revision=1,
        observation_revision=1,
        seed=7,
        content_hash=f"hash-{case.name}",
    )
    return GraphSnapshot(metadata, suite.graph_for(case)), case.start, case.goal


def test_stale_global_result_is_not_executable(suite: ScenarioSuite) -> None:
    snapshot, start, goal = _snapshot(suite, "normal")
    result = run_stateless_global(AStarPlanner(), snapshot, start, goal)
    current = replace(snapshot.metadata, map_revision=2)
    validation = validate_global_result(snapshot, start, goal, result, current_metadata=current)
    assert not validation.passed
    assert not validation.executable
    assert "stale_revision" in validation.failures


def test_benchmark_global_records_distribution_and_oracle(suite: ScenarioSuite) -> None:
    snapshot, start, goal = _snapshot(suite, "upper_corridor_blocked")
    record = benchmark_global(AStarPlanner(), snapshot, start, goal, repeats=5)
    assert record.oracle_matched
    assert record.deterministic
    assert not record.validation_failures
    assert record.performance.samples == 5
    assert record.performance.elapsed_ns_worst >= record.performance.elapsed_ns_p50


def test_route_churn_uses_path_edges() -> None:
    assert route_churn(("a", "b", "c"), ("a", "b", "d")) == 2 / 3
    assert route_churn((), ()) == 0.0


def test_protective_stop_requires_validation_and_every_gate(suite: ScenarioSuite) -> None:
    snapshot, start, goal = _snapshot(suite, "normal")
    result = run_stateless_global(AStarPlanner(), snapshot, start, goal)
    validation = validate_global_result(snapshot, start, goal, result)
    gate = AutomaticResumeGate()
    gate.hazard_detected()
    gate.confirm_stop()
    gate.hazard_cleared()
    assert not authorize_after_protective_stop(validation, gate)
    gate.record_path_revalidation(original_path_safe=True)
    gate.revalidate_resume_instruction()
    gate.authorize_local_safety()
    assert gate.try_automatic_resume()
    assert authorize_after_protective_stop(validation, gate)
