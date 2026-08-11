import json
from collections import Counter
from math import isclose
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from hospital_path_lab.contracts import (
    GridSnapshot,
    PlanStatus,
    Pose2D,
    SnapshotMetadata,
)
from hospital_path_lab.corpus_records import preserve_hidden_failure
from hospital_path_lab.experiment_runner import (
    ExperimentConfig,
    _episode_step_range,
    _follower_completion_failure,
    _follower_expected_class,
    _follower_time_budget_s,
    _grid_dijkstra_oracle,
    _local_hard_safety_failures,
    _Measured,
    _pipeline_records,
    _preserve_hidden_failures,
    _record_deadline_miss,
    _select_corpus,
    run_experiment,
)
from hospital_path_lab.grid import GridMap
from hospital_path_lab.map_factory import generate_batch, generate_golden_cases
from hospital_path_lab.simulation import SimulationResult


def test_full_twenty_case_experiment_writes_reproducible_evidence(tmp_path: Path) -> None:
    config = ExperimentConfig()
    result = run_experiment(tmp_path / "run", config)

    assert result.case_count == 20
    assert result.split_counts == {
        "development": 4,
        "golden": 12,
        "hidden": 2,
        "regressions": 2,
    }
    for path in (result.results_path, result.pareto_path, result.summary_path):
        assert path.is_file() and path.stat().st_size > 0

    results = json.loads(result.results_path.read_text(encoding="utf-8"))
    pareto = json.loads(result.pareto_path.read_text(encoding="utf-8"))
    assert Counter(item["split"] for item in results["corpus"]) == result.split_counts
    assert len(results["cases"]) == 20
    assert all(
        [step["step"] for step in case["steps"]] == list(range(len(case["steps"])))
        for case in results["cases"]
    )
    assert not results["hard_failures"]
    assert not result.hard_failures
    coverage = results["evaluation_coverage"]
    evaluated_steps = sum(len(case["steps"]) for case in results["cases"])
    assert coverage["global"] == {
        "policy": "all_cases_initial_through_episode_max_event_step",
        "evaluated_steps": evaluated_steps,
    }
    assert coverage["grid_astar"]["policy"] == "all_valid_steps_with_reference_path"
    assert coverage["dwa"]["policy"] == "all_valid_steps_with_reference_path"
    assert coverage["path_followers"]["policy"] == ("all_steps_with_found_grid_astar_path")
    assert coverage["hidden_visualizations"]["policy"] == (
        "graph_and_grid_for_every_evaluated_hidden_step"
    )
    assert coverage["deadline_policy"] == {
        "scope": "simulation_only_research_threshold_not_product_requirement",
        "comparison": "measured_elapsed_ns_strictly_greater_than_deadline_ns",
        "global_deadline_ns": config.global_deadline_ns,
        "local_deadline_ns": config.local_deadline_ns,
        "path_follower_deadline_ns": config.path_follower_deadline_ns,
    }

    valid_steps = [
        step for case in results["cases"] for step in case["steps"] if step["input_valid"]
    ]
    global_records = [record for step in valid_steps for record in step["global_results"]]
    assert global_records
    assert all(record["oracle_matched"] for record in global_records)
    assert all(record["expanded_nodes"] >= 0 for record in global_records)
    assert all(0.0 <= record["route_churn"] <= 1.0 for record in global_records)
    dstar_records = [record for record in global_records if record["algorithm"] == "dstar_lite"]
    assert any(record["incremental_state"]["state_reuse_count"] > 0 for record in dstar_records)
    assert all(record["map_id"] for record in global_records)
    assert all(record["input_content_hash"] for record in global_records)
    assert all(record["provenance"]["map_id"] == record["map_id"] for record in global_records)
    assert all(not record["deadline_miss"] for record in global_records)

    grid_records = [
        record
        for step in valid_steps
        for record in step["local_results"]
        if record["algorithm"] == "grid_astar"
    ]
    assert grid_records
    assert all(record["grid_oracle"]["algorithm"] == "grid_dijkstra" for record in grid_records)
    assert all(record["oracle_matched"] for record in grid_records)
    for record in grid_records:
        assert record["status"] == record["grid_oracle"]["status"]
        if record["status"] == "found":
            assert isclose(record["cost"], record["grid_oracle"]["cost"], rel_tol=1e-9)
            assert record["minimum_clearance"] >= 0.0
        assert record["map_id"]
        assert record["input_content_hash"]
        assert not record["deadline_miss"]

    dwa_records = [
        record
        for step in valid_steps
        for record in step["local_results"]
        if record["algorithm"] == "dwa"
    ]
    follower_records = [record for step in valid_steps for record in step["follower_results"]]
    pipeline_records = [record for step in valid_steps for record in step["pipeline_results"]]
    assert len(dwa_records) == len(grid_records)
    assert coverage["grid_astar"]["result_count"] == len(grid_records)
    assert coverage["dwa"]["result_count"] == len(dwa_records)
    assert coverage["path_followers"]["result_count"] == len(follower_records)
    assert coverage["path_followers"]["compatible_step_count"] * 2 == len(follower_records)
    assert len(pipeline_records) == len(follower_records)
    assert coverage["end_to_end_pipelines"] == {
        "policy": "astar_to_grid_astar_to_each_path_follower",
        "compatible_step_count": coverage["path_followers"]["compatible_step_count"],
        "result_count": len(pipeline_records),
    }
    assert coverage["dynamic_local_closed_loop"] == {
        "policy": "synthetic_create_hold_remove_stateful_dwa_rejoin_contract",
        "evaluated_steps": 63,
        "metrics": [
            "safe_stop",
            "deadlock",
            "recovery",
            "path_deviation",
            "rejoin",
            "collision",
            "clearance",
            "tracking_error",
        ],
    }
    assert coverage["not_measured"] == ["full_corpus_dynamic_local_closed_loop"]
    assert {record["pipeline"] for record in pipeline_records} == {
        "astar_grid_astar_pure_pursuit",
        "astar_grid_astar_rpp",
    }
    assert all(record["success"] for record in pipeline_records)
    assert all(not record["collision"] for record in pipeline_records)
    assert all(record["expected_outcome_class"] for record in follower_records)
    assert all(
        record["simulation_time_budget_s"] >= config.follower_max_time_s
        for record in follower_records
    )
    assert all(
        record["simulation_elapsed_s"] <= record["simulation_time_budget_s"]
        for record in follower_records
    )
    assert all(record["map_id"] for record in follower_records)
    assert all(record["input_content_hash"] for record in follower_records)
    assert all(not record["deadline_miss"] for record in follower_records)

    algorithm_names = {item["name"] for item in results["algorithm_manifest"]}
    assert algorithm_names == {
        "dijkstra",
        "astar",
        "dstar_lite",
        "grid_astar",
        "dwa",
            "pure_pursuit",
            "rpp",
            "dynamic_pure_pursuit",
            "dynamic_dwa",
            "teb",
        "mppi",
        "state_lattice",
        "hybrid_astar",
    }
    implemented = {
        item["name"]
        for item in results["algorithm_manifest"]
        if item["implementation_status"] == "implemented"
    }
    aggregate_names = {name for algorithms in pareto["roles"].values() for name in algorithms}
    assert aggregate_names == implemented
    required_performance = {
        "elapsed_ns_p50",
        "elapsed_ns_p95",
        "elapsed_ns_p99",
        "elapsed_ns_worst",
        "peak_memory_bytes",
        "pass_count",
        "collision_count",
    }
    for algorithms in pareto["roles"].values():
        for aggregate in algorithms.values():
            assert required_performance <= aggregate.keys()
            assert aggregate["elapsed_ns_worst"] >= aggregate["elapsed_ns_p99"]
            assert aggregate["elapsed_ns_p99"] >= aggregate["elapsed_ns_p95"]
            assert aggregate["elapsed_ns_p95"] >= aggregate["elapsed_ns_p50"]
    assert set(pareto["roles"]["global"]) == {"dijkstra", "astar", "dstar_lite"}
    for aggregate in pareto["roles"]["global"].values():
        assert aggregate["expanded_nodes_p95"] >= 0.0
        assert 0.0 <= aggregate["route_churn_p50"] <= 1.0
        assert aggregate["deterministic_count"] == aggregate["samples"]
    assert set(pareto["roles"]["local"]) == {"grid_astar", "dwa"}
    for aggregate in pareto["roles"]["local"].values():
        assert aggregate["minimum_clearance_m_min"] >= 0.0
        assert aggregate["minimum_clearance_m_worst"] >= aggregate["minimum_clearance_m_min"]
        assert aggregate["deterministic_count"] == aggregate["samples"]
    assert pareto["roles"]["local"]["dwa"]["samples"] == len(dwa_records)
    for aggregate in pareto["roles"]["path_follower"].values():
        assert aggregate["samples"] == coverage["path_followers"]["compatible_step_count"]
        assert aggregate["mean_tracking_error_m_p95"] >= 0.0
        assert aggregate["maximum_tracking_error_m_worst"] >= 0.0
        assert aggregate["jerk_rms_mps3_p95"] >= 0.0
        assert aggregate["additional_distance_m_mean"] >= 0.0
        assert aggregate["overshoot_m_worst"] >= 0.0
    assert set(pareto["pipelines"]) == {
        "astar_grid_astar_pure_pursuit",
        "astar_grid_astar_rpp",
    }
    for aggregate in pareto["pipelines"].values():
        assert aggregate["samples"] == coverage["path_followers"]["compatible_step_count"]
        assert aggregate["success_count"] == aggregate["samples"]
        assert aggregate["component_validation_pass_count"] == aggregate["samples"]
        assert aggregate["collision_count"] == 0
        assert aggregate["deadline_miss_count"] == 0
        assert aggregate["elapsed_ns_p99"] >= aggregate["elapsed_ns_p95"]
        assert aggregate["elapsed_ns_worst"] >= aggregate["elapsed_ns_p99"]
        assert aggregate["peak_memory_bytes"] >= 0

    assert config.base_seed != config.hidden_seed
    assert all(
        item["source_batch_seed"] == config.hidden_seed
        for item in results["corpus"]
        if item["split"] == "hidden"
    )
    assert all(
        item["source_batch_seed"] == config.base_seed
        for item in results["corpus"]
        if item["split"] in {"development", "regressions"}
    )
    assert all(
        item["source_batch_seed"] == item["world_seed"]
        for item in results["corpus"]
        if item["split"] == "golden"
    )
    assert len({item["episode_hash"] for item in results["corpus"]}) == 20

    stale = results["stale_result_evidence"]
    assert stale["check_count"] > 0
    assert stale["all_rejected"]
    assert stale["rejected_count"] == stale["check_count"]
    assert {check["role"] for check in stale["checks"]} == {
        "global",
        "local",
        "follower",
    }
    safety = results["protective_stop_evidence"]
    assert safety["available"]
    assert not safety["authorization_before_revalidation"]
    assert not safety["automatic_resume_before_revalidation"]
    assert safety["automatic_resume_after_all_gates"]
    assert safety["authorization_after_all_gates"]
    dynamic_local = results["dynamic_local_evidence"]
    assert dynamic_local["contract_passed"]
    assert dynamic_local["simulation_only"]
    assert dynamic_local["collision_count"] == 0
    assert dynamic_local["safe_stop_count"] == 3
    assert dynamic_local["deadlock_observed"]
    assert dynamic_local["recovery_observed"]
    assert dynamic_local["path_deviation_observed"]
    assert dynamic_local["rejoin_observed"]
    assert dynamic_local["commands_finite"]
    assert dynamic_local["metrics_finite"]

    hidden_step_count = sum(
        len(case["steps"]) for case in results["cases"] if case["split"] == "hidden"
    )
    assert len(result.visualization_paths) == hidden_step_count * 2
    for image in result.visualization_paths:
        assert image.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert len(results["visualizations"]) == hidden_step_count * 2
    assert coverage["hidden_visualizations"] == {
        "policy": "graph_and_grid_for_every_evaluated_hidden_step",
        "evaluated_steps": hidden_step_count,
        "png_count": hidden_step_count * 2,
    }
    freeze = results["freeze_evidence"]
    assert len(freeze["freeze_sha256"]) == 64
    assert len(freeze["algorithm_source_tree"]["sha256"]) == 64
    assert len(freeze["frozen_public_corpus"]["cases"]) == 18
    assert freeze["frozen_public_corpus"]["contract_case_count"] == 18
    assert freeze["hidden_selection"]["selected_after_freeze"]
    assert freeze["capture_order"][-1] == "hidden_generation_and_selection"
    assert not results["regression_candidates"]
    assert "simulation_only" in result.summary_path.read_text(encoding="utf-8")


def test_episode_steps_follow_last_event_instead_of_fixed_six() -> None:
    episode = SimpleNamespace(events=(SimpleNamespace(step=2), SimpleNamespace(step=9)))
    assert tuple(_episode_step_range(episode)) == tuple(range(10))


def test_follower_time_budget_is_shared_and_scales_with_path_length() -> None:
    short_path = (
        SimpleNamespace(x=0.0, y=0.0),
        SimpleNamespace(x=1.0, y=0.0),
    )
    long_path = (
        SimpleNamespace(x=0.0, y=0.0),
        SimpleNamespace(x=10.0, y=0.0),
    )
    assert _follower_time_budget_s(short_path, floor_s=30.0) == 30.0
    assert _follower_time_budget_s(long_path, floor_s=30.0) > _follower_time_budget_s(
        short_path, floor_s=30.0
    )


def test_forbidden_entry_and_general_follower_timeout_are_hard_failures() -> None:
    assert _local_hard_safety_failures(
        result_collision=False,
        validation_failures=("forbidden_zone_entry",),
    ) == frozenset({"forbidden_zone_entry"})
    timeout = SimulationResult(
        component="rpp",
        status=PlanStatus.NO_PATH,
        goal_reached=False,
        collision=False,
        poses=(Pose2D(0.0, 0.0),),
        commands=(),
        elapsed_s=30.0,
        minimum_clearance_m=1.0,
        mean_tracking_error_m=0.0,
        maximum_tracking_error_m=0.0,
        jerk_rms_mps3=0.0,
        final_goal_distance_m=1.0,
        failure_reason="goal_not_reached_before_timeout",
    )
    assert _follower_completion_failure(timeout) == "follower_timeout"


def test_grid_dijkstra_oracle_uses_same_reference_bounds_as_grid_astar() -> None:
    occupancy = np.zeros((100, 100), dtype=np.bool_)
    occupancy[30:71, 50] = True
    snapshot = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="bounded_oracle",
            map_revision=0,
            mission_revision=0,
            observation_revision=0,
            seed=1,
            content_hash="bounded-oracle-v1",
        ),
        grid=GridMap(occupancy, resolution_m=0.1),
    )
    start = Pose2D(1.5, 5.0)
    goal = Pose2D(8.5, 5.0)
    reference = (start, goal)

    status, cost = _grid_dijkstra_oracle(snapshot, reference, start, goal)

    assert status is PlanStatus.NO_PATH
    assert cost is None


def test_grid_dijkstra_oracle_treats_forbidden_cells_as_non_traversable() -> None:
    snapshot = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="forbidden_oracle",
            map_revision=0,
            mission_revision=0,
            observation_revision=0,
            seed=2,
            content_hash="forbidden-oracle-v1",
        ),
        grid=GridMap(np.zeros((60, 100), dtype=np.bool_), resolution_m=0.1),
        forbidden_cells=frozenset((50, y) for y in range(60)),
    )
    start = Pose2D(1.5, 3.0)
    goal = Pose2D(8.5, 3.0)

    status, cost = _grid_dijkstra_oracle(snapshot, (start, goal), start, goal)

    assert status is PlanStatus.NO_PATH
    assert cost is None


def test_pipeline_success_requires_every_component_validation_and_oracle() -> None:
    global_records = [
        {
            "algorithm": "astar",
            "status": "found",
            "deadline_miss": False,
            "deterministic": True,
            "validation": {"passed": False},
            "oracle_matched": False,
            "measured_elapsed_ns": 1,
            "peak_memory_bytes": 10,
        }
    ]
    local_records = [
        {
            "algorithm": "grid_astar",
            "status": "found",
            "collision": False,
            "deadline_miss": False,
            "deterministic": True,
            "validation": {"passed": True},
            "oracle_matched": True,
            "measured_elapsed_ns": 2,
            "peak_memory_bytes": 20,
            "minimum_clearance": 0.2,
        }
    ]
    follower_records = [
        {
            "algorithm": "rpp",
            "goal_reached": True,
            "collision": False,
            "deadline_miss": False,
            "deterministic": True,
            "initial_command_validation": {"passed": True},
            "measured_elapsed_ns": 3,
            "peak_memory_bytes": 30,
            "minimum_clearance_m": 0.3,
            "mean_tracking_error_m": 0.0,
            "maximum_tracking_error_m": 0.0,
            "jerk_rms_mps3": 0.0,
            "additional_distance_m": 0.0,
            "overshoot_m": 0.0,
            "provenance": {"map_id": "pipeline_contract"},
        }
    ]

    record = _pipeline_records(global_records, local_records, follower_records)[0]

    assert record["component_validation_passed"] is False
    assert record["success"] is False
    assert record["peak_memory_bytes"] == 30


def test_corpus_is_frozen_before_separate_hidden_selection() -> None:
    config = ExperimentConfig()
    corpus, freeze = _select_corpus(config)
    assert Counter(case.episode.split.value for case, _ in corpus) == {
        "golden": 12,
        "development": 4,
        "hidden": 2,
        "regressions": 2,
    }
    assert len(freeze["frozen_public_corpus"]["cases"]) == 18
    assert freeze["hidden_selection"]["selected_after_freeze"] is True
    assert freeze["hidden_selection"]["source_batch_seed"] == config.hidden_seed
    assert freeze["promoted_regressions"]["case_count"] == 0


def test_previous_hidden_failure_can_feed_the_next_regression_corpus(
    tmp_path: Path,
) -> None:
    hidden = generate_batch(base_seed=20_260_810)[6]
    preserve_hidden_failure(
        hidden,
        reason="earlier_timeout:rpp",
        failing_step=1,
        output_directory=tmp_path
        / "first_run"
        / "regression_candidates"
        / "timeout"
        / "rpp",
    )
    preserve_hidden_failure(
        hidden,
        reason="oracle_mismatch:astar",
        failing_step=2,
        output_directory=tmp_path
        / "second_run"
        / "regression_candidates"
        / "oracle"
        / "astar",
    )
    config = ExperimentConfig(regression_input_dir=str(tmp_path))

    corpus, freeze = _select_corpus(config)

    counts = Counter(case.episode.split.value for case, _ in corpus)
    assert counts == {
        "golden": 12,
        "development": 4,
        "hidden": 2,
        "regressions": 3,
    }
    promoted = freeze["promoted_regressions"]
    assert promoted["loaded_before_hidden_selection"] is True
    assert promoted["input_enabled"] is True
    assert promoted["requested_limit"] is None
    assert promoted["case_count"] == 1
    promoted_case = next(
        case
        for case, _ in corpus
        if case.episode.episode_id == promoted["case_ids"][0]
    )
    assert max(event.step for event in promoted_case.episode.events) == 2
    assert len(freeze["frozen_public_corpus"]["cases"]) == 19


def test_tiny_simulation_deadline_is_classified_as_hard_failure() -> None:
    case = generate_batch(base_seed=20_260_810)[0]
    failures: list[dict[str, object]] = []
    missed = _record_deadline_miss(
        _Measured(value=None, elapsed_ns=2, peak_memory_bytes=0),
        1,
        failures,
        case,
        0,
        "astar",
    )
    assert missed
    assert failures == [
        {
            "type": "deadline_miss",
            "case_id": case.episode.episode_id,
            "step": 0,
            "algorithm": "astar",
            "detail": "measured_elapsed_ns=2 > deadline_ns=1",
        }
    ]


def test_follower_expectation_and_hidden_failure_preservation(tmp_path: Path) -> None:
    batch = generate_batch(base_seed=20_260_810)
    golden = generate_golden_cases()[0]
    assert (
        _follower_expected_class(
            golden,
            input_valid=True,
            grid_oracle_status=PlanStatus.FOUND,
            grid_result=SimpleNamespace(status=PlanStatus.FOUND),
        )
        == "validated_reachable_golden"
    )

    hidden = batch[6]
    failure = {
        "type": "oracle_mismatch",
        "case_id": hidden.episode.episode_id,
        "step": 0,
        "algorithm": "astar",
        "detail": "test evidence",
    }
    limitations: list[dict[str, object]] = []
    corpus = ((hidden, 20_260_810),)
    manifest = _preserve_hidden_failures(
        tmp_path,
        corpus,
        [failure],
        limitations,
        freeze_sha256="a" * 64,
    )
    assert not limitations
    assert len(manifest) == 1
    record_path = tmp_path / str(manifest[0]["path"])
    original = record_path.read_bytes()
    assert manifest[0]["created"] is True
    assert manifest[0]["retuned_during_hidden_run"] is False

    repeated = _preserve_hidden_failures(
        tmp_path,
        corpus,
        [failure],
        limitations,
        freeze_sha256="a" * 64,
    )
    assert repeated[0]["created"] is False
    assert record_path.read_bytes() == original
