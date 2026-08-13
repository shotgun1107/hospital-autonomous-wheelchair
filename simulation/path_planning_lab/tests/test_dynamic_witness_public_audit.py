from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import pytest

from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_witness_reporting import (
    ExpectationAssessment,
    WitnessAuditOutputWriter,
    WitnessEvidenceClass,
    WitnessPublicAudit,
    audit_public_witness_episode,
    build_witness_audit_manifest,
    public_witness_audit_episodes,
)
from hospital_path_lab.map_factory import canonical_content_hash

_R1_HASH = "a" * 64


@pytest.fixture(scope="module")
def observation_invalid_result():
    episode = next(
        item
        for item in generate_dynamic_corpus()
        if item.split is DynamicCorpusSplit.GOLDEN
        and item.expectation_category is DynamicExpectationCategory.OBSERVATION_INVALID
    )
    return audit_public_witness_episode(
        episode,
        corpus_lane="legacy_mechanism",
        corpus_ordinal=18,
    )


def test_public_scope_is_exactly_v6_thirteen_plus_legacy_golden_six() -> None:
    scope = public_witness_audit_episodes()
    assert len(scope) == 19
    assert [lane for lane, _ in scope].count("v6_primary") == 13
    assert [lane for lane, _ in scope].count("legacy_mechanism") == 6
    assert all(
        episode.split in (DynamicCorpusSplit.GOLDEN, DynamicCorpusSplit.DEVELOPMENT)
        for _, episode in scope
    )


def test_narrow_public_case_is_independently_forbidden_and_wait_only() -> None:
    episode = next(
        item
        for item in generate_dynamic_v6_public_corpus()
        if item.expectation_category is DynamicExpectationCategory.LOCAL_DETOUR_FORBIDDEN
    )
    result = audit_public_witness_episode(
        episode,
        corpus_lane="v6_primary",
        corpus_ordinal=5,
    )
    assert result.expectation_assessment is ExpectationAssessment.MATCHED
    assert set(result.evidence_classes) >= {
        WitnessEvidenceClass.FORBIDDEN,
        WitnessEvidenceClass.WAIT_ONLY,
    }
    assert WitnessEvidenceClass.FEASIBLE not in result.evidence_classes
    assert not result.hard_failures


def test_legacy_no_safe_solution_requires_full_hold_and_analytic_block() -> None:
    episode = next(
        item
        for item in generate_dynamic_corpus()
        if item.split is DynamicCorpusSplit.GOLDEN
        and item.expectation_category is DynamicExpectationCategory.NO_SAFE_SOLUTION
    )
    result = audit_public_witness_episode(
        episode,
        corpus_lane="legacy_mechanism",
        corpus_ordinal=16,
    )
    assert result.expectation_assessment is ExpectationAssessment.MATCHED
    assert WitnessEvidenceClass.NO_SAFE_SOLUTION in result.evidence_classes
    assert {record.witness.kind.value for record in result.witness_records} == {"hold_only"}
    assert not result.hard_failures


def test_staggered_public_case_preserves_two_hazard_wait_diagnostic() -> None:
    episode = tuple(
        item
        for item in generate_dynamic_v6_public_corpus()
        if item.expectation_category is DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP
    )[1]
    result = audit_public_witness_episode(
        episode,
        corpus_lane="v6_primary",
        corpus_ordinal=12,
    )
    assert result.expectation_assessment is ExpectationAssessment.MATCHED
    diagnostic = next(
        record
        for record in result.witness_records
        if "multi_hazard_wait_diagnostic" in record.roles
    )
    assert diagnostic.validation.metrics.full_stop_count >= 2
    assert result.hard_failures == (
        "ideal_profile:ideal_capsule_ground_truth_miss",
        "unexpected_pass_for_expected_category",
    )


def test_evaluator_label_changes_report_but_not_search_semantics() -> None:
    episode = next(
        item
        for item in generate_dynamic_corpus()
        if item.split is DynamicCorpusSplit.GOLDEN
        and item.expectation_category is DynamicExpectationCategory.OBSERVATION_INVALID
    )
    relabeled = replace(
        episode,
        expectation_category=DynamicExpectationCategory.WAIT_AND_RESUME,
    )
    first = audit_public_witness_episode(
        episode,
        corpus_lane="legacy_mechanism",
        corpus_ordinal=18,
    )
    second = audit_public_witness_episode(
        relabeled,
        corpus_lane="legacy_mechanism",
        corpus_ordinal=18,
    )
    assert first.world.content_hash == second.world.content_hash
    assert first.search_semantic_hash == second.search_semantic_hash
    assert first.report_content_hash != second.report_content_hash


def test_observation_invalid_fault_is_replayed_only_after_label_free_search(
    observation_invalid_result,
) -> None:
    result = observation_invalid_result
    assert WitnessEvidenceClass.OBSERVATION_UNDECIDABLE in result.evidence_classes
    assert result.expectation_assessment is ExpectationAssessment.MATCHED
    assert result.observation_fault_replay is not None
    assert result.observation_fault_replay.passed
    assert result.observation_fault_replay.recovery_grants_motion_authority is False
    assert "observation_fault_replay_is_evaluator_only_after_search" in result.limitations
    assert not result.hard_failures


def test_manifest_semantics_exclude_worker_and_shard_operational_settings() -> None:
    repository_root = Path(__file__).resolve().parents[3]
    first = build_witness_audit_manifest(
        repository_root=repository_root,
        r1_audit_content_hash=_R1_HASH,
        max_workers=1,
        shard_size=1,
    )
    second = build_witness_audit_manifest(
        repository_root=repository_root,
        r1_audit_content_hash=_R1_HASH,
        max_workers=14,
        shard_size=2_048,
    )
    assert first.semantic_content_hash == second.semantic_content_hash
    assert first.content_hash != second.content_hash


def test_writer_preserves_partial_state_and_refuses_overwrite(
    tmp_path: Path,
    observation_invalid_result,
) -> None:
    repository_root = Path(__file__).resolve().parents[3]
    manifest = build_witness_audit_manifest(
        repository_root=repository_root,
        r1_audit_content_hash=_R1_HASH,
        max_workers=1,
        shard_size=1,
    )
    result = observation_invalid_result
    manifest = replace(
        manifest,
        episode_order=((result.public_id, result.world.content_hash),),
        semantic_content_hash="b" * 64,
        content_hash="c" * 64,
    )
    output = tmp_path / "witness-audit"
    writer = WitnessAuditOutputWriter(output, manifest)
    writer.start()
    writer.write_episode(result)
    state = json.loads((output / "run_state.incomplete.json").read_text("utf-8"))
    assert state["partial"] is True
    assert state["completed_public_ids"] == [result.public_id]
    assert not (output / "witness_audit_completion.json").exists()
    assert (output / "episodes" / result.public_id / "trajectory.png").stat().st_size > 0
    with pytest.raises(FileExistsError):
        WitnessAuditOutputWriter(output, manifest).start()


def test_completed_subset_is_a_run_record_not_r2_completion(
    tmp_path: Path,
    observation_invalid_result,
) -> None:
    repository_root = Path(__file__).resolve().parents[3]
    result = observation_invalid_result
    manifest = build_witness_audit_manifest(
        repository_root=repository_root,
        r1_audit_content_hash=_R1_HASH,
        max_workers=1,
        shard_size=1,
    )
    manifest = replace(
        manifest,
        episode_order=((result.public_id, result.world.content_hash),),
        semantic_content_hash="d" * 64,
        content_hash="e" * 64,
    )
    semantic_hash = canonical_content_hash(
        {"episode_search_hashes": (result.search_semantic_hash,)}
    )
    report_hash = canonical_content_hash({"episode_report_hashes": (result.report_content_hash,)})
    audit = WitnessPublicAudit(
        schema_version=result.schema_version,
        audit_version=result.audit_version,
        simulation_only=True,
        hidden_used=False,
        r1_audit_content_hash=_R1_HASH,
        v6_public_corpus_content_hash="1" * 64,
        legacy_golden_corpus_content_hash="2" * 64,
        search_config_hash=result.world.search_config_hash,
        episode_results=(result,),
        hard_failures=(),
        limitations=("test_subset",),
        semantic_content_hash=semantic_hash,
        report_content_hash=report_hash,
        elapsed_nonqualification_ns=0,
    )
    output = tmp_path / "completed-subset"
    writer = WitnessAuditOutputWriter(output, manifest)
    writer.start()
    writer.write_episode(result)
    writer.complete(audit)
    receipt = json.loads((output / "witness_audit_completion.json").read_text(encoding="utf-8"))
    assert receipt["episode_count"] == 1
    assert receipt["r2_completion_qualified"] is False
    assert not (output / "run_state.incomplete.json").exists()
    assert (output / "run_state.complete.json").exists()
