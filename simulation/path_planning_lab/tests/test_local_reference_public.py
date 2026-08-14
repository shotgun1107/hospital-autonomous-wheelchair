from __future__ import annotations

import ast
import json
from dataclasses import fields, replace
from pathlib import Path

import pytest

import hospital_path_lab.local_reference_reporting as reporting
from hospital_path_lab.local_reference_contracts import (
    ReferenceBuildStatus,
    ReferenceEvidenceLevel,
    ReferenceTravelDirection,
)
from hospital_path_lab.local_reference_reporting import (
    LOCAL_REFERENCE_PUBLIC_CASE_COUNT,
    LOCAL_REFERENCE_PUBLIC_MANIFEST_VERSION,
    LOCAL_REFERENCE_PUBLIC_REPORT_VERSION,
    LocalReferencePublicAudit,
    LocalReferencePublicOutputWriter,
    build_local_reference_public_manifest,
    build_public_reference_context,
    evaluate_local_reference_public_case,
    evaluate_local_reference_public_cases,
    public_local_reference_cases,
)
from hospital_path_lab.map_factory import canonical_content_hash


@pytest.fixture(scope="module")
def wide_left_result():
    case = public_local_reference_cases()[0]
    return evaluate_local_reference_public_case(case)


def test_r4_public_catalog_preserves_all_r3_ordinals_and_expected_statuses() -> None:
    cases = public_local_reference_cases()

    assert len(cases) == LOCAL_REFERENCE_PUBLIC_CASE_COUNT == 21
    assert tuple(case.ordinal for case in cases) == tuple(range(21))
    assert tuple(case.public_id for case in cases) == (
        "wide-straight-left",
        "wide-straight-right",
        "wide-mirror-left",
        "wide-mirror-right",
        "narrow-corridor",
        "narrow-door",
        "just-wide-door",
        "dead-end",
        "corner-safe",
        "corner-rotation-blocked",
        "vertical-left",
        "vertical-right",
        "forbidden-only-block",
        "allowed-region-pinch",
        "start-unsafe",
        "goal-unsafe",
        "resource-exact",
        "resource-plus-one",
        "invalid-provenance",
        "crossing-static-left",
        "crossing-static-right",
    )
    ready = tuple(
        case.public_id
        for case in cases
        if case.expected_build_status is ReferenceBuildStatus.REFERENCE_SET_READY
    )
    assert ready == (
        "wide-straight-left",
        "wide-straight-right",
        "wide-mirror-left",
        "wide-mirror-right",
        "vertical-left",
        "vertical-right",
        "crossing-static-left",
        "crossing-static-right",
    )
    assert cases[17].expected_build_status is ReferenceBuildStatus.SEARCH_INCONCLUSIVE
    assert cases[18].expected_build_status is ReferenceBuildStatus.INVALID_INPUT


def test_reporting_context_is_deterministic_static_only_and_label_free() -> None:
    case = public_local_reference_cases()[0]

    first = build_public_reference_context(case)
    second = build_public_reference_context(case)

    assert first == second
    assert first.context_content_hash == second.context_content_hash
    assert first.observation_revision is None
    assert first.observation_content_hash is None
    context_fields = {field.name for field in fields(first)}
    assert context_fields.isdisjoint(
        {"expectation_category", "oracle_spec", "split", "hidden_seed"}
    )


@pytest.mark.parametrize(
    "public_id,expected_status,expected_reason",
    (
        ("start-unsafe", ReferenceBuildStatus.NO_REFERENCE, "no_spatial_candidate"),
        ("invalid-provenance", ReferenceBuildStatus.INVALID_INPUT, "invalid_spatial_source"),
    ),
)
def test_non_reference_public_cases_preserve_source_taxonomy(
    public_id: str,
    expected_status: ReferenceBuildStatus,
    expected_reason: str,
) -> None:
    case = next(item for item in public_local_reference_cases() if item.public_id == public_id)

    result = evaluate_local_reference_public_case(case)

    assert result.hard_passed
    assert result.reference_set.status is expected_status
    assert result.reference_set.termination_reason == expected_reason
    assert not result.reference_set.candidates
    assert not result.validations
    assert not result.window_sequences


def test_supported_feasible_case_builds_valid_spatial_reference_and_full_window_sequence(
    wide_left_result,
) -> None:
    result = wide_left_result

    assert result.hard_passed
    assert result.reference_set.status is ReferenceBuildStatus.REFERENCE_SET_READY
    assert len(result.reference_set.candidates) == 1
    reference = result.reference_set.candidates[0]
    validation = result.validations[0]
    sequence = result.window_sequences[0]
    assert reference.evidence_level is ReferenceEvidenceLevel.SPATIAL_ONLY
    assert validation.passed
    assert sequence.all_ready
    assert len(sequence.updates) == len(reference.knots) == 24
    assert tuple(section.travel_direction for section in reference.sections) == (
        ReferenceTravelDirection.NONE,
        ReferenceTravelDirection.FORWARD,
        ReferenceTravelDirection.NONE,
        ReferenceTravelDirection.FORWARD,
        ReferenceTravelDirection.NONE,
        ReferenceTravelDirection.REVERSE,
        ReferenceTravelDirection.NONE,
        ReferenceTravelDirection.NONE,
    )
    assert {update.window.reference_session_id for update in sequence.updates} == {
        reference.reference_session_id
    }
    revisions = tuple(update.window.subgoal_revision for update in sequence.updates)
    assert revisions == tuple(sorted(revisions))
    assert sorted(set(revisions)) == [0, 1, 2, 3, 4, 5]
    assert sequence.updates[-1].window.terminal_rejoin_included


def test_process_batch_matches_serial_semantics_and_input_order() -> None:
    by_id = {case.public_id: case for case in public_local_reference_cases()}
    cases = (
        by_id["invalid-provenance"],
        by_id["start-unsafe"],
        by_id["goal-unsafe"],
    )

    serial = evaluate_local_reference_public_cases(cases, max_workers=1)
    parallel = evaluate_local_reference_public_cases(cases, max_workers=2)

    assert tuple(item.ordinal for item in parallel) == tuple(sorted(case.ordinal for case in cases))
    assert tuple(item.semantic_content_hash for item in parallel) == tuple(
        item.semantic_content_hash for item in serial
    )


def test_repeat_evaluation_excludes_elapsed_time_from_semantics() -> None:
    case = next(
        item for item in public_local_reference_cases() if item.public_id == "invalid-provenance"
    )

    first = evaluate_local_reference_public_case(case)
    second = evaluate_local_reference_public_case(case)

    assert first.semantic_content_hash == second.semantic_content_hash
    assert first.report_content_hash == second.report_content_hash


def test_writer_preserves_partial_state_and_seals_complete_clean_run(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    wide_left_result,
) -> None:
    result = wide_left_result
    case = public_local_reference_cases()[0]
    repository_root = Path(__file__).resolve().parents[3]
    manifest = build_local_reference_public_manifest(
        repository_root=repository_root,
        max_workers=1,
    )
    monkeypatch.setattr(reporting, "LOCAL_REFERENCE_PUBLIC_CASE_COUNT", 1)
    case_order = ((case.public_id, case.semantic_content_hash),)
    catalog_hash = canonical_content_hash({"case_hashes": (case.semantic_content_hash,)})
    manifest_semantic = canonical_content_hash(
        {
            "manifest_version": LOCAL_REFERENCE_PUBLIC_MANIFEST_VERSION,
            "simulation_only": True,
            "hidden_used": False,
            "source_freeze_hash": manifest.source_freeze_hash,
            "catalog_content_hash": catalog_hash,
            "case_order": case_order,
        }
    )
    manifest_content = canonical_content_hash(
        {
            "semantic_content_hash": manifest_semantic,
            "git_head": manifest.git_head,
            "git_tree": manifest.git_tree,
            "git_dirty": False,
            "max_workers_nonsemantic": manifest.max_workers_nonsemantic,
            "logical_cpu_count_nonsemantic": manifest.logical_cpu_count_nonsemantic,
        }
    )
    manifest = replace(
        manifest,
        git_dirty=False,
        catalog_content_hash=catalog_hash,
        case_order=case_order,
        semantic_content_hash=manifest_semantic,
        content_hash=manifest_content,
    )
    audit_semantic = canonical_content_hash(
        {
            "report_version": LOCAL_REFERENCE_PUBLIC_REPORT_VERSION,
            "catalog_content_hash": catalog_hash,
            "case_result_hashes": (result.semantic_content_hash,),
            "relation_failures": (),
            "parity_case_id": case.public_id,
            "serial_process_parity_passed": True,
            "repeat_determinism_passed": True,
            "hard_failures": (),
        }
    )
    audit_report = canonical_content_hash(
        {
            "semantic_content_hash": audit_semantic,
            "case_report_hashes": (result.report_content_hash,),
        }
    )
    audit = LocalReferencePublicAudit(
        report_version=LOCAL_REFERENCE_PUBLIC_REPORT_VERSION,
        simulation_only=True,
        hidden_used=False,
        catalog_content_hash=catalog_hash,
        case_results=(result,),
        relation_failures=(),
        parity_case_id=case.public_id,
        serial_process_parity_passed=True,
        repeat_determinism_passed=True,
        hard_failures=(),
        limitations=("test_only",),
        semantic_content_hash=audit_semantic,
        report_content_hash=audit_report,
        elapsed_nonqualification_ns=0,
    )
    output = tmp_path / "r4-public"
    writer = LocalReferencePublicOutputWriter(
        output,
        manifest,
        repository_root=repository_root,
    )
    monkeypatch.setattr(writer, "_verify_git_state", lambda: None)
    monkeypatch.setattr(reporting, "_source_freeze_hash", lambda _root: manifest.source_freeze_hash)
    writer.start()

    assert (output / "partial-state.json").exists()
    with pytest.raises(RuntimeError, match="every R4 public case"):
        writer.complete(audit)
    writer.write_case(result)
    with pytest.raises(FileExistsError):
        writer.write_case(result)
    summary_json, summary_md, receipt_path = writer.complete(audit)

    assert summary_json.exists() and summary_md.exists()
    assert receipt_path is not None and receipt_path.exists()
    assert not (output / "partial-state.json").exists()
    assert (output / "complete-state.json").exists()
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    assert receipt["qualified"] is True
    assert receipt["case_count"] == 1
    assert next((output / "cases").glob("*/reference.png")).stat().st_size > 0
    with pytest.raises(FileExistsError):
        LocalReferencePublicOutputWriter(
            output,
            manifest,
            repository_root=repository_root,
        ).start()


def test_core_r4_modules_do_not_depend_on_reporting_corpus_or_hidden() -> None:
    root = Path(__file__).parents[1] / "src" / "hospital_path_lab"
    for filename in (
        "local_reference_builder.py",
        "local_reference_validation.py",
        "local_reference_window.py",
    ):
        tree = ast.parse((root / filename).read_text(encoding="utf-8"))
        imported = " ".join(
            node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
        )
        names = " ".join(node.id for node in ast.walk(tree) if isinstance(node, ast.Name))
        for forbidden in (
            "local_reference_reporting",
            "dynamic_corpus",
            "expectation_category",
            "oracle_spec",
            "hidden",
        ):
            assert forbidden not in imported
            assert forbidden not in names
