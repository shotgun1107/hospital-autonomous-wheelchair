from __future__ import annotations

import json
from pathlib import Path

import pytest

from hospital_path_lab.dynamic_corpus import (
    generate_dynamic_corpus,
    hidden_seed_commitment,
)
from hospital_path_lab.dynamic_runner import (
    DynamicExperimentConfig,
    _promotion_decision,
    _run_corpus,
    _source_freeze_hash,
    run_dynamic_experiment,
)
from hospital_path_lab.map_factory import canonical_content_hash


def _config(hidden_seed: int) -> DynamicExperimentConfig:
    return DynamicExperimentConfig(
        base_seed=20260811,
        hidden_seed=hidden_seed,
        hidden_seed_commitment=hidden_seed_commitment(hidden_seed),
        bootstrap_iterations=20,
        qualification_warmups=0,
        qualification_repeats=1,
        profiles=("normal",),
        public_episode_limit=1,
        hidden_episode_limit=1,
        evaluation_tick_limit=2,
        simulation_workers=2,
        contract_test_evidence=True,
        generate_visualizations=True,
    )


def _load(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def test_runner_writes_hashed_manifest_results_markdown_and_png(tmp_path) -> None:
    output = tmp_path / "run-a"
    result = run_dynamic_experiment(output, _config(70001))

    assert result.public_run_count == 2
    assert result.hidden_run_count == 2
    assert result.simulation_worker_count == 2
    assert not result.promoted_dwa
    required = (
        "experiment_manifest.json",
        "hidden_consumption_receipt.json",
        "public_prequalification.json",
        "qualification_results.json",
        "contract_fault_results.json",
        "hard_safety_results.json",
        "paired_episode_results.json",
        "paired_statistics.json",
        "pareto_summary.json",
        "promotion_decision.json",
        "summary.md",
    )
    assert all((output / name).is_file() for name in required)
    assert len(tuple((output / "visualizations").rglob("*.png"))) == 2

    manifest = _load(result.manifest_path)
    recorded_hash = manifest.pop("manifest_content_hash")
    assert recorded_hash == canonical_content_hash(manifest)
    assert manifest["source_freeze_hash"] == _source_freeze_hash()
    assert manifest["simulation_execution"]["worker_count"] == 2
    qualification = _load(output / "qualification_results.json")
    assert not qualification["parallelized"]
    assert qualification["execution_mode"].startswith("serial_parent")
    assert set(qualification["numeric_thread_environment"].values()) == {"1"}
    summary = result.summary_path.read_text(encoding="utf-8")
    assert summary.startswith(
        "이 결과는 open-loop 원형 Actor와 동결된 합성 관측을 사용하는 Python "
        "simulation_only 비교이며 제품 알고리즘 또는 실제 사람 탑승 안전성의 증거가 아니다."
    )

    records = json.loads(result.paired_results_path.read_text(encoding="utf-8"))
    hidden_records = [record for record in records if record["split"] == "hidden"]
    hidden_records[0]["hard_safety"]["passed"] = False
    statistics = _load(result.statistics_path)
    qualification = _load(output / "qualification_results.json")
    contract = _load(output / "contract_fault_results.json")
    decision = _promotion_decision(
        records,
        hidden_records,
        statistics,
        qualification,
        contract,
        full_run=True,
    )
    assert not decision["conditions"]["01_both_controllers_normal_stress_hard_safety"]
    assert not decision["promote_dynamic_dwa"]


def test_consumed_hidden_commitment_cannot_be_reused(tmp_path) -> None:
    config = _config(70002)
    run_dynamic_experiment(tmp_path / "first", config)

    with pytest.raises(ValueError, match="already consumed"):
        run_dynamic_experiment(tmp_path / "second", config)


def test_manifest_is_never_overwritten(tmp_path) -> None:
    output = tmp_path / "single"
    config = _config(70003)
    run_dynamic_experiment(output, config)

    with pytest.raises(FileExistsError, match="already contains"):
        run_dynamic_experiment(output, config)


def test_process_parallel_results_match_serial_order_and_semantics(tmp_path) -> None:
    episodes = generate_dynamic_corpus(base_seed=20260811)[:2]
    serial_config = DynamicExperimentConfig(
        base_seed=20260811,
        hidden_seed=71001,
        hidden_seed_commitment=hidden_seed_commitment(71001),
        bootstrap_iterations=10,
        qualification_repeats=1,
        profiles=("normal",),
        evaluation_tick_limit=4,
        simulation_workers=1,
        contract_test_evidence=True,
        generate_visualizations=False,
    )
    parallel_config = DynamicExperimentConfig(
        base_seed=20260811,
        hidden_seed=71002,
        hidden_seed_commitment=hidden_seed_commitment(71002),
        bootstrap_iterations=10,
        qualification_repeats=1,
        profiles=("normal",),
        evaluation_tick_limit=4,
        simulation_workers=2,
        contract_test_evidence=True,
        generate_visualizations=False,
    )
    serial = _run_corpus(
        episodes,
        config=serial_config,
        output_directory=tmp_path / "serial",
        hidden=False,
        worker_count=1,
    )
    parallel = _run_corpus(
        episodes,
        config=parallel_config,
        output_directory=tmp_path / "parallel",
        hidden=False,
        worker_count=2,
    )

    def stable(records: list[dict[str, object]]) -> list[dict[str, object]]:
        result = json.loads(json.dumps(records))
        for record in result:
            record.pop("worker_elapsed_ns_nonqualification")
        return result

    assert stable(parallel) == stable(serial)
    assert [record["controller_name"] for record in parallel] == [
        "dynamic_pure_pursuit",
        "dynamic_dwa",
        "dynamic_pure_pursuit",
        "dynamic_dwa",
    ]
