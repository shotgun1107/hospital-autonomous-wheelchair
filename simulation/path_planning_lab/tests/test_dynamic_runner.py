from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import pytest

import hospital_path_lab.dynamic_runner as dynamic_runner_module
from hospital_path_lab.cli import _build_parser
from hospital_path_lab.dynamic_corpus import (
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_runner import (
    DYNAMIC_RUNNER_VERSION,
    DynamicPublicQualificationConfig,
    _full_public_evidence,
    _public_record_set_hash,
    _public_run_scope,
    _qualification_snapshot_cases,
    _run_corpus,
    _run_wall_clock_qualification,
    _scenario_oracle_matrix_hash,
    run_dynamic_public_qualification,
)
from hospital_path_lab.map_factory import canonical_content_hash


def _load(path: Path) -> dict[str, object] | list[dict[str, object]]:
    return json.loads(path.read_text(encoding="utf-8"))


def _public_config(**changes) -> DynamicPublicQualificationConfig:
    return replace(
        DynamicPublicQualificationConfig(base_seed=20260811),
        **changes,
    )


def _rigid_signature(controller_name: str) -> dict[str, object]:
    signature: dict[str, object] = {
        "schema_version": "dynamic_rigid_metamorphic_signature_v6",
        "numeric_tolerance_version": "dynamic_numeric_tolerance_v1",
        "controller_command_trace_hash": f"command-{controller_name}",
        "shared_gate_trace_hash": f"gate-{controller_name}",
        "pipeline_result_hash": f"pipeline-{controller_name}",
        "hard_safety_result_hash": "hard-safety",
        "category_result_hash": "category",
        "functional_result_hash": "functional",
    }
    signature["content_hash"] = canonical_content_hash(signature)
    return signature


def _record(
    controller_name: str,
    *,
    elapsed_ns: int = 10,
    process_id: int = 100,
) -> dict[str, object]:
    return {
        "episode_id": "synthetic-public-episode",
        "episode_content_hash": "episode-hash",
        "split": "golden",
        "expectation_category": "wait_and_resume",
        "seed": 20260811,
        "progressable": True,
        "observation_profile": "normal",
        "controller_name": controller_name,
        "hard_safety": {"passed": True, "failures": [], "first_failure_time_s": None},
        "functional_qualified": True,
        "functional_failures": [],
        "category_oracle_applied": True,
        "category_oracle_failures": [],
        "scenario": None,
        "metrics": {"completion_time_s": 1.0},
        "pipeline": {"completed": True, "tick_count": 1},
        "worker_elapsed_ns_nonqualification": elapsed_ns,
        "worker_process_id_nonqualification": process_id,
        "pair_id": "synthetic-pair",
        "observation_stream_hash": "observation-stream",
        "command_state_event_hash": f"command-{controller_name}",
        "controller_semantic_digest": f"semantic-{controller_name}",
        "rigid_metamorphic_signature": _rigid_signature(controller_name),
        "post_controller_gate_diagnostics": {
            "schema_version": "dynamic_post_controller_gate_diagnostics_v6",
            "stage": "POST_CONTROLLER_GATE",
            "event_count": 0,
            "override_count": 0,
            "hold_reason_counts": {},
            "events": [],
        },
    }


def _records() -> list[dict[str, object]]:
    return [_record("dynamic_pure_pursuit"), _record("dynamic_dwa")]


def _synthetic_corpus_records(episodes, config) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    for episode in episodes:
        scenario = (
            {
                "family": episode.scenario_family.value,
                "variant": episode.variant,
                "orientation": episode.orientation.value,
                "latent_case_id": episode.latent_case_id,
                "semantic_world_hash": episode.semantic_world_hash,
                "oracle_hash": episode.oracle_hash,
            }
            if hasattr(episode, "scenario_family")
            else None
        )
        for profile_name in config.profiles:
            pair_id = f"pair-{episode.episode_id}-{profile_name}"
            stream_hash = f"stream-{episode.episode_id}-{profile_name}"
            for controller_name in ("dynamic_pure_pursuit", "dynamic_dwa"):
                record = _record(controller_name)
                record.update(
                    {
                        "episode_id": episode.episode_id,
                        "episode_content_hash": episode.content_hash,
                        "split": episode.split.value,
                        "expectation_category": episode.expectation_category.value,
                        "seed": episode.seed,
                        "progressable": episode.progressable,
                        "observation_profile": profile_name,
                        "scenario": scenario,
                        "pair_id": pair_id,
                        "observation_stream_hash": stream_hash,
                    }
                )
                records.append(record)
    return records


def _contract_pass() -> dict[str, object]:
    return {
        "passed": True,
        "test_source_hash": "contract-source",
        "case_count": 25,
        "cases": [],
        "pytest": {"returncode": 0, "stdout_tail": "", "stderr_tail": ""},
    }


def _functional_pass(_records, **_kwargs) -> dict[str, object]:
    return {
        "passed": True,
        "generic_functional_failures": [],
        "local_detour_feasible": {
            "golden_count": 1,
            "golden_passed": True,
            "development_count": 5,
            "development_pass_count": 5,
            "development_pass_ratio": 1.0,
            "required_development_pass_ratio": 0.80,
            "passed": True,
        },
    }


def _controller_timing(*, samples: int = 500, cold_samples: int = 5) -> dict[str, object]:
    return {
        "passed": True,
        "samples": samples,
        "cold_samples": cold_samples,
        "cold_maximum_ns": 2,
        "p50_ns": 1,
        "p95_ns": 1,
        "p99_ns": 2,
        "maximum_ns": 2,
        "deadline_ns": 50_000_000,
        "deadline_miss_count": 0,
        "peak_memory_bytes": 100,
    }


def _qualification_pass(_corpus, config) -> dict[str, object]:
    warmups = config.qualification_warmups if config is not None else 30
    repeats = config.qualification_repeats if config is not None else 100
    timing = _controller_timing(samples=5 * repeats)
    return {
        "schema_version": "dynamic_wall_clock_qualification_v6",
        "status": "completed",
        "passed": True,
        "not_run_reason": None,
        "failure_detail": None,
        "machine_identifier": "test-machine",
        "parent_process_id": 999,
        "process_affinity": [0],
        "active_worker_process_ids_before": [],
        "execution_mode": "serial_parent_after_simulation_worker_pool_shutdown",
        "parallelized": False,
        "numeric_thread_environment": {
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        },
        "snapshot_cases": [
            {
                "case_id": f"test-case-{index}",
                "input_content_hash": f"input-{index}",
                "snapshot_content_hash": f"snapshot-{index}",
            }
            for index in range(5)
        ],
        "snapshot_set_hash": "snapshot-set",
        "warmups_per_snapshot": warmups,
        "repeats_per_snapshot": repeats,
        "controllers": {
            "dynamic_pure_pursuit": timing,
            "dynamic_dwa": timing,
        },
    }


def _patch_successful_public_evidence(monkeypatch) -> None:
    monkeypatch.setattr(
        dynamic_runner_module,
        "_run_corpus",
        lambda episodes, *, config, **_kwargs: _synthetic_corpus_records(
            episodes,
            config,
        ),
    )
    monkeypatch.setattr(
        dynamic_runner_module,
        "_contract_fault_qualification",
        lambda _config, **_kwargs: _contract_pass(),
    )
    monkeypatch.setattr(dynamic_runner_module, "_public_functional_qualification", _functional_pass)
    monkeypatch.setattr(dynamic_runner_module, "_run_wall_clock_qualification", _qualification_pass)
    monkeypatch.setattr(dynamic_runner_module, "_git_state", lambda: ("test-commit", False))


def test_full_public_run_seals_v6_receipt_with_complete_hashes(tmp_path, monkeypatch) -> None:
    _patch_successful_public_evidence(monkeypatch)
    monkeypatch.setattr(dynamic_runner_module, "_source_freeze_hash", lambda: "source-freeze")
    output = tmp_path / "sealing"

    result = run_dynamic_public_qualification(output, _public_config())

    assert result.passed
    assert result.receipt_path is not None and result.receipt_path.exists()
    report = _load(result.report_path)
    gate = _load(result.gate_path)
    receipt = _load(result.receipt_path)
    assert isinstance(report, dict) and isinstance(gate, dict) and isinstance(receipt, dict)
    assert DYNAMIC_RUNNER_VERSION == "dynamic_runner_v6"
    assert report["schema_version"] == "dynamic_public_qualification_report_v6"
    assert gate["schema_version"] == "dynamic_public_qualification_gate_v6"
    assert receipt["schema_version"] == "dynamic_public_qualification_receipt_v6"
    assert gate["receipt_sealed"] and not gate["report_only"]
    assert gate["record_coverage_passed"]
    assert gate["source_freeze_hash_at_receipt_write"] == "source-freeze"
    assert "hidden_generation_allowed" not in gate
    assert "parent_process_id" not in receipt
    assert "worker_elapsed_ns_nonqualification" not in receipt
    assert "worker_process_id_nonqualification" not in receipt

    records = _load(output / "paired_episode_results.json")
    contract = _load(output / "contract_fault_results.json")
    prequalification = _load(output / "public_prequalification.json")
    qualification = _load(output / "qualification_results.json")
    assert isinstance(records, list)
    assert isinstance(contract, dict)
    assert isinstance(prequalification, dict)
    assert isinstance(qualification, dict)
    expected_evidence = _full_public_evidence(
        public_records=records,
        contract_results=contract,
        public_functional=prequalification["functional_qualification"],
        qualification=qualification,
    )
    corpus = generate_dynamic_corpus() + generate_dynamic_v6_public_corpus()
    assert receipt["public_record_set_hash"] == _public_record_set_hash(records)
    assert receipt["controller_semantic_digest_set_hash"] == canonical_content_hash(
        tuple(
            (
                record["pair_id"],
                record["controller_name"],
                record["controller_semantic_digest"],
            )
            for record in records
        )
    )
    assert receipt["scenario_oracle_matrix_hash"] == _scenario_oracle_matrix_hash(corpus)
    assert receipt["full_evidence_hash"] == canonical_content_hash(expected_evidence)
    assert receipt["source_freeze_hash_at_receipt_write"] == "source-freeze"
    assert len(receipt["qualification_snapshot_cases"]) == 5
    assert receipt["qualification_execution"] == {
        "execution_mode": "serial_parent_after_simulation_worker_pool_shutdown",
        "parallelized": False,
        "active_worker_process_ids_before": [],
    }
    recorded_hash = receipt.pop("receipt_content_hash")
    assert recorded_hash == canonical_content_hash(receipt)


@pytest.mark.parametrize(
    ("changes", "reason"),
    (
        ({"base_seed": 20260812}, "noncanonical_base_seed"),
        ({"public_episode_limit": 1}, "public_episode_limit_set"),
        ({"profiles": ("normal",)}, "profiles_not_full_normal_stress"),
        ({"evaluation_tick_limit": 1}, "evaluation_tick_limit_set"),
        ({"contract_test_evidence": True}, "contract_evidence_injected"),
        ({"qualification_warmups": 29}, "qualification_warmups_not_30"),
        ({"qualification_repeats": 99}, "qualification_repeats_not_100"),
    ),
)
def test_limited_or_injected_run_is_non_sealing(changes, reason) -> None:
    scope = _public_run_scope(_public_config(**changes))

    assert not scope["sealing_eligible"]
    assert scope["mode"] == "non_sealing_report"
    assert reason in scope["non_sealing_reasons"]


def test_non_sealing_run_writes_report_but_never_receipt(tmp_path, monkeypatch) -> None:
    _patch_successful_public_evidence(monkeypatch)
    monkeypatch.setattr(dynamic_runner_module, "_source_freeze_hash", lambda: "source-freeze")
    output = tmp_path / "report-only"

    result = run_dynamic_public_qualification(
        output,
        _public_config(
            profiles=("normal",),
            public_episode_limit=1,
            evaluation_tick_limit=1,
            qualification_warmups=0,
            qualification_repeats=1,
            contract_test_evidence=True,
        ),
    )

    assert not result.passed
    assert result.receipt_path is None
    assert result.report_path.exists()
    assert not (output / "public_qualification_receipt.json").exists()
    gate = _load(result.gate_path)
    assert isinstance(gate, dict)
    assert gate["evidence_passed"]
    assert gate["report_only"]
    assert not gate["sealing_eligible"]
    assert "hidden_generation_allowed" not in gate


def test_source_change_after_run_or_before_seal_fails_closed(tmp_path, monkeypatch) -> None:
    _patch_successful_public_evidence(monkeypatch)
    hashes = iter(("source-a", "source-a", "source-b"))
    monkeypatch.setattr(dynamic_runner_module, "_source_freeze_hash", lambda: next(hashes))
    output = tmp_path / "source-race"

    result = run_dynamic_public_qualification(output, _public_config())

    assert not result.passed
    assert result.receipt_path is None
    gate = _load(result.gate_path)
    assert isinstance(gate, dict)
    assert not gate["source_freeze_consistent"]
    assert not gate["receipt_sealed"]
    assert not (output / "public_qualification_receipt.json").exists()


def test_source_change_at_receipt_write_fails_closed(tmp_path, monkeypatch) -> None:
    _patch_successful_public_evidence(monkeypatch)
    hashes = iter(("source-a", "source-a", "source-a", "source-b"))
    monkeypatch.setattr(dynamic_runner_module, "_source_freeze_hash", lambda: next(hashes))
    output = tmp_path / "receipt-write-race"

    result = run_dynamic_public_qualification(output, _public_config())

    assert not result.passed
    assert result.receipt_path is None
    gate = _load(result.gate_path)
    assert isinstance(gate, dict)
    assert gate["source_freeze_hash_at_receipt_write"] == "source-b"
    assert not gate["source_freeze_consistent"]
    assert "source_changed_at_receipt_write" in gate["non_sealing_reasons"]
    assert not (output / "public_qualification_receipt.json").exists()


def test_source_change_immediately_before_receipt_file_write_fails_closed(
    tmp_path,
    monkeypatch,
) -> None:
    _patch_successful_public_evidence(monkeypatch)
    hashes = iter(("source-a", "source-a", "source-a", "source-a", "source-b"))
    monkeypatch.setattr(dynamic_runner_module, "_source_freeze_hash", lambda: next(hashes))
    output = tmp_path / "receipt-file-write-race"

    result = run_dynamic_public_qualification(output, _public_config())

    assert not result.passed
    assert result.receipt_path is None
    gate = _load(result.gate_path)
    assert isinstance(gate, dict)
    assert gate["source_freeze_hash_at_receipt_write"] == "source-b"
    assert not gate["source_freeze_consistent"]
    assert "source_changed_at_receipt_write" in gate["non_sealing_reasons"]
    assert not (output / "public_qualification_receipt.json").exists()


def test_incomplete_public_cross_product_never_seals_receipt(tmp_path, monkeypatch) -> None:
    _patch_successful_public_evidence(monkeypatch)
    monkeypatch.setattr(dynamic_runner_module, "_run_corpus", lambda *_args, **_kwargs: _records())
    monkeypatch.setattr(dynamic_runner_module, "_source_freeze_hash", lambda: "source-freeze")
    output = tmp_path / "incomplete-records"

    result = run_dynamic_public_qualification(output, _public_config())

    assert not result.passed
    assert result.receipt_path is None
    gate = _load(result.gate_path)
    prequalification = _load(output / "public_prequalification.json")
    assert isinstance(gate, dict) and isinstance(prequalification, dict)
    assert not gate["record_coverage_passed"]
    coverage = prequalification["record_coverage"]
    assert coverage["expected_record_count"] > coverage["actual_record_count"]
    assert coverage["missing"]


def test_qualification_failure_uses_fixed_fail_closed_schema(tmp_path, monkeypatch) -> None:
    _patch_successful_public_evidence(monkeypatch)
    monkeypatch.setattr(dynamic_runner_module, "_source_freeze_hash", lambda: "source-freeze")
    monkeypatch.setattr(
        dynamic_runner_module,
        "_run_wall_clock_qualification",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("timing failed")),
    )
    output = tmp_path / "qualification-fallback"

    result = run_dynamic_public_qualification(output, _public_config())

    assert not result.passed
    qualification = _load(output / "qualification_results.json")
    assert isinstance(qualification, dict)
    assert qualification["schema_version"] == "dynamic_wall_clock_qualification_v6"
    assert qualification["status"] == "failed"
    assert not qualification["passed"]
    assert qualification["not_run_reason"] == "wall_clock_qualification_failed"
    assert set(qualification["controllers"]) == {"dynamic_pure_pursuit", "dynamic_dwa"}
    assert qualification["controllers"]["dynamic_dwa"] == qualification["controllers"][
        "dynamic_pure_pursuit"
    ]
    assert not (output / "public_qualification_receipt.json").exists()


def test_contract_fault_pytest_uses_workspace_local_unique_basetemp(
    tmp_path,
    monkeypatch,
) -> None:
    output = tmp_path / "contract-workspace"
    captured_basetemps: list[Path] = []

    def fake_run(command, **_kwargs):
        option_index = command.index("--basetemp")
        basetemp = Path(command[option_index + 1])
        captured_basetemps.append(basetemp)
        assert basetemp.is_relative_to(output)
        assert basetemp.exists()

        class Completed:
            returncode = 0
            stdout = "contract tests passed"
            stderr = ""

        return Completed()

    monkeypatch.setattr(dynamic_runner_module.subprocess, "run", fake_run)

    result = dynamic_runner_module._contract_fault_qualification(
        _public_config(),
        workspace_basetemp_parent=output,
    )

    assert result["passed"]
    assert len(captured_basetemps) == 1
    assert not captured_basetemps[0].exists()


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("schema_version", "dynamic_wall_clock_qualification_v5"),
        ("active_worker_process_ids_before", [42]),
        ("snapshot_cases", []),
        (
            "numeric_thread_environment",
            {
                "OMP_NUM_THREADS": "2",
                "OPENBLAS_NUM_THREADS": "1",
                "MKL_NUM_THREADS": "1",
                "NUMEXPR_NUM_THREADS": "1",
            },
        ),
    ),
)
def test_malformed_qualification_evidence_is_fail_closed(
    tmp_path,
    monkeypatch,
    field,
    value,
) -> None:
    _patch_successful_public_evidence(monkeypatch)
    monkeypatch.setattr(dynamic_runner_module, "_source_freeze_hash", lambda: "source-freeze")
    evidence = _qualification_pass(None, None)
    evidence[field] = value
    monkeypatch.setattr(
        dynamic_runner_module,
        "_run_wall_clock_qualification",
        lambda *_args, **_kwargs: evidence,
    )
    output = tmp_path / f"invalid-{field}"

    result = run_dynamic_public_qualification(output, _public_config())

    assert not result.passed
    assert result.receipt_path is None
    qualification = _load(output / "qualification_results.json")
    assert isinstance(qualification, dict)
    assert qualification["schema_version"] == "dynamic_wall_clock_qualification_v6"
    assert qualification["status"] == "invalid_evidence"
    assert qualification["not_run_reason"] == "wall_clock_qualification_invalid_schema"
    assert set(qualification["controllers"]) == {"dynamic_pure_pursuit", "dynamic_dwa"}


def test_public_record_hash_excludes_only_worker_pid_and_elapsed() -> None:
    first = _records()
    nondeterministic_change = json.loads(json.dumps(first))
    for record in nondeterministic_change:
        record["worker_elapsed_ns_nonqualification"] = 999_999
        record["worker_process_id_nonqualification"] = 4242
    semantic_change = json.loads(json.dumps(first))
    semantic_change[0]["post_controller_gate_diagnostics"]["override_count"] = 1

    assert _public_record_set_hash(first) == _public_record_set_hash(nondeterministic_change)
    assert _public_record_set_hash(first) != _public_record_set_hash(semantic_change)


def test_rigid_pair_metamorphic_gate_rejects_orientation_mismatch() -> None:
    corpus = generate_dynamic_v6_public_corpus()
    rigid_episodes = tuple(
        episode
        for episode in corpus
        if episode.latent_case_id == "diagonal-rigid-pair-v6"
    )
    records = _synthetic_corpus_records(rigid_episodes, _public_config())

    passed = dynamic_runner_module._rigid_pair_metamorphic_qualification(
        records,
        public_corpus=corpus,
    )
    assert passed["passed"]
    assert passed["comparison_count"] == 4

    changed = json.loads(json.dumps(records))
    target = next(
        record
        for record in changed
        if record["controller_name"] == "dynamic_dwa"
        and record["observation_profile"] == "stress"
        and record["scenario"]["orientation"] == "vertical"
    )
    target_signature = target["rigid_metamorphic_signature"]
    target_signature["pipeline_result_hash"] = "changed"
    target_signature["content_hash"] = canonical_content_hash(
        {
            key: value
            for key, value in target_signature.items()
            if key != "content_hash"
        }
    )
    failed = dynamic_runner_module._rigid_pair_metamorphic_qualification(
        changed,
        public_corpus=corpus,
    )

    assert not failed["passed"]
    assert any(
        "rigid_pair_orientation_signature_mismatch" in failure["reasons"]
        for failure in failed["failures"]
    )


def test_rigid_pair_metamorphic_gate_rejects_stale_component_hash() -> None:
    corpus = generate_dynamic_v6_public_corpus()
    rigid_episodes = tuple(
        episode
        for episode in corpus
        if episode.latent_case_id == "diagonal-rigid-pair-v6"
    )
    records = _synthetic_corpus_records(rigid_episodes, _public_config())
    target = next(
        record
        for record in records
        if record["controller_name"] == "dynamic_dwa"
        and record["observation_profile"] == "stress"
        and record["scenario"]["orientation"] == "vertical"
    )
    target["rigid_metamorphic_signature"]["pipeline_result_hash"] = "changed"

    result = dynamic_runner_module._rigid_pair_metamorphic_qualification(
        records,
        public_corpus=corpus,
    )

    assert not result["passed"]
    assert any(
        "rigid_pair_signature_content_hash_mismatch" in failure["reasons"]
        for failure in result["failures"]
    )


def test_rigid_pair_actual_records_have_orientation_independent_signatures() -> None:
    corpus = generate_dynamic_v6_public_corpus()
    rigid_episodes = tuple(
        episode
        for episode in corpus
        if episode.latent_case_id == "diagonal-rigid-pair-v6"
    )
    config = _public_config(
        evaluation_tick_limit=4,
        simulation_workers=1,
        qualification_repeats=1,
        contract_test_evidence=True,
    )

    records = _run_corpus(rigid_episodes, config=config, worker_count=1)
    result = dynamic_runner_module._rigid_pair_metamorphic_qualification(
        records,
        public_corpus=corpus,
    )

    assert result["passed"]
    assert result["comparison_count"] == 4


def test_public_runner_exports_no_dynamic_hidden_execution_or_override() -> None:
    config = _public_config()
    help_text = _build_parser().format_help()

    assert not hasattr(dynamic_runner_module, "DynamicExperimentConfig")
    assert not hasattr(dynamic_runner_module, "run_dynamic_experiment")
    assert not hasattr(dynamic_runner_module, "generate_dynamic_hidden_corpus")
    assert not hasattr(config, "hidden_seed")
    assert not hasattr(config, "hidden_seed_commitment")
    assert not hasattr(config, "test_only_public_gate_override")
    assert "dynamic-public-qualification" in help_text
    assert "dynamic-experiment" not in help_text


def test_wall_clock_suite_uses_frozen_zero_one_two_and_static_cases(monkeypatch) -> None:
    corpus = generate_dynamic_corpus() + generate_dynamic_v6_public_corpus()
    cases = _qualification_snapshot_cases(corpus)
    metadata = {case_id: details for case_id, _snapshot, details in cases}

    assert [details["actor_tube_count"] for details in metadata.values()] == [0, 1, 2, 1, 1]
    assert metadata["corner-static-forbidden"]["has_static_occupancy"]
    assert metadata["corner-static-forbidden"]["has_forbidden_cells"]
    assert metadata["staggered-risk-multisegment"]["reference_path_segment_count"] >= 2

    class FastController:
        name = "fast"

        def step(self, _snapshot):
            return None

    class FastPp(FastController):
        name = "dynamic_pure_pursuit"

    class FastDwa(FastController):
        name = "dynamic_dwa"

    monkeypatch.setattr(dynamic_runner_module, "DynamicPurePursuitController", FastPp)
    monkeypatch.setattr(dynamic_runner_module, "DynamicDwaController", FastDwa)
    qualification = _run_wall_clock_qualification(
        corpus,
        _public_config(
            profiles=("normal",),
            qualification_warmups=0,
            qualification_repeats=1,
            contract_test_evidence=True,
        ),
    )
    assert qualification["schema_version"] == "dynamic_wall_clock_qualification_v6"
    assert qualification["execution_mode"].startswith("serial_parent")
    assert qualification["active_worker_process_ids_before"] == []
    assert qualification["controllers"]["dynamic_pure_pursuit"]["samples"] == 5
    assert qualification["controllers"]["dynamic_dwa"]["samples"] == 5


def test_public_output_is_never_overwritten(tmp_path, monkeypatch) -> None:
    _patch_successful_public_evidence(monkeypatch)
    monkeypatch.setattr(dynamic_runner_module, "_source_freeze_hash", lambda: "source-freeze")
    output = tmp_path / "single"
    config = _public_config(
        public_episode_limit=1,
        evaluation_tick_limit=1,
        profiles=("normal",),
        qualification_warmups=0,
        qualification_repeats=1,
        contract_test_evidence=True,
    )
    run_dynamic_public_qualification(output, config)

    with pytest.raises(FileExistsError, match="already contains artifacts"):
        run_dynamic_public_qualification(output, config)


def test_process_parallel_results_match_serial_order_and_semantics() -> None:
    episodes = generate_dynamic_corpus(base_seed=20260811)[:2]
    serial_config = _public_config(
        profiles=("normal",),
        evaluation_tick_limit=4,
        simulation_workers=1,
        qualification_repeats=1,
        contract_test_evidence=True,
    )
    parallel_config = replace(serial_config, simulation_workers=2)
    serial = _run_corpus(episodes, config=serial_config, worker_count=1)
    parallel = _run_corpus(episodes, config=parallel_config, worker_count=2)

    def stable(records: list[dict[str, object]]) -> list[dict[str, object]]:
        result = json.loads(json.dumps(records))
        for record in result:
            record.pop("worker_elapsed_ns_nonqualification")
            record.pop("worker_process_id_nonqualification")
        return result

    assert stable(parallel) == stable(serial)
    assert [record["controller_name"] for record in parallel] == [
        "dynamic_pure_pursuit",
        "dynamic_dwa",
        "dynamic_pure_pursuit",
        "dynamic_dwa",
    ]
    for records in (serial, parallel):
        for index in range(0, len(records), 2):
            pair = records[index : index + 2]
            assert len({record["pair_id"] for record in pair}) == 1
            assert len({record["observation_stream_hash"] for record in pair}) == 1
            assert len({record["worker_process_id_nonqualification"] for record in pair}) == 1
            assert all(
                record["post_controller_gate_diagnostics"]["stage"]
                == "POST_CONTROLLER_GATE"
                for record in pair
            )
