from __future__ import annotations

import json
from dataclasses import fields, replace
from pathlib import Path

import pytest

import hospital_path_lab.spatial_oracle_reporting as reporting
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.spatial_oracle_contracts import SpatialOracleStatus
from hospital_path_lab.spatial_oracle_reporting import (
    SPATIAL_PUBLIC_CASE_COUNT,
    SpatialPublicAudit,
    SpatialPublicOutputWriter,
    build_spatial_public_manifest,
    evaluate_spatial_public_case,
    evaluate_spatial_public_cases,
    public_spatial_cases,
)
from hospital_path_lab.spatial_oracle_validation import spatial_pose_is_safe


def test_frozen_public_catalog_has_complete_stable_order_and_label_free_requests() -> None:
    cases = public_spatial_cases()

    assert len(cases) == SPATIAL_PUBLIC_CASE_COUNT == 21
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
    assert len({case.semantic_content_hash for case in cases}) == 21
    request_fields = {field.name for field in fields(cases[0].request)}
    assert request_fields.isdisjoint(
        {
            "actors",
            "observation",
            "controller",
            "expectation_category",
            "oracle_spec",
            "hidden_seed",
        }
    )
    assert all(
        case.request.integrity_failure() is None
        for case in cases
        if case.public_id != "invalid-provenance"
    )
    assert cases[18].request.integrity_failure() == "request_content_hash_mismatch"


@pytest.mark.parametrize(
    "public_id, expected_status, expected_reason",
    (
        ("start-unsafe", SpatialOracleStatus.SPATIALLY_INFEASIBLE, "start_footprint_unsafe"),
        ("goal-unsafe", SpatialOracleStatus.SPATIALLY_INFEASIBLE, "goal_footprint_unsafe"),
        ("invalid-provenance", SpatialOracleStatus.INVALID_INPUT, "request_content_hash_mismatch"),
    ),
)
def test_analytic_and_invalid_public_cases_fail_closed_without_search_work(
    public_id: str,
    expected_status: SpatialOracleStatus,
    expected_reason: str,
) -> None:
    case = next(item for item in public_spatial_cases() if item.public_id == public_id)

    result = evaluate_spatial_public_case(case)

    assert result.hard_passed
    assert result.result.status is expected_status
    assert result.result.termination_reason == expected_reason
    assert result.result.generated_edges == 0
    assert result.result.expanded_states == 0


def test_exact_resource_boundary_is_exhaustive_and_one_less_is_not_infeasible() -> None:
    by_id = {case.public_id: case for case in public_spatial_cases()}

    exact = evaluate_spatial_public_case(by_id["resource-exact"])
    limited = evaluate_spatial_public_case(by_id["resource-plus-one"])

    assert exact.hard_passed
    assert exact.result.status is SpatialOracleStatus.SPATIALLY_INFEASIBLE
    assert exact.result.exhaustive
    assert exact.result.expanded_states == 1_056
    assert exact.result.generated_edges == 4_320
    assert limited.hard_passed
    assert limited.result.status is SpatialOracleStatus.RESOURCE_LIMIT
    assert not limited.result.exhaustive


@pytest.mark.parametrize("public_id", ("corner-rotation-blocked", "allowed-region-pinch"))
def test_geometry_negative_cases_keep_safe_endpoints_and_exhaust_the_lattice(
    public_id: str,
) -> None:
    case = next(item for item in public_spatial_cases() if item.public_id == public_id)

    assert spatial_pose_is_safe(case.request, case.request.start_pose)
    assert spatial_pose_is_safe(case.request, case.request.rejoin_goal.pose)
    result = evaluate_spatial_public_case(case)

    assert result.hard_passed
    assert result.result.status is SpatialOracleStatus.SPATIALLY_INFEASIBLE
    assert result.result.termination_reason == "bounded_lattice_exhausted"
    assert result.result.exhaustive


def test_process_batch_preserves_serial_semantics_and_input_order() -> None:
    by_id = {case.public_id: case for case in public_spatial_cases()}
    cases = (
        by_id["invalid-provenance"],
        by_id["start-unsafe"],
        by_id["goal-unsafe"],
    )

    serial = evaluate_spatial_public_cases(cases, max_workers=1)
    parallel = evaluate_spatial_public_cases(cases, max_workers=2)

    assert tuple(item.ordinal for item in parallel) == tuple(sorted(case.ordinal for case in cases))
    assert tuple(item.semantic_content_hash for item in parallel) == tuple(
        item.semantic_content_hash for item in serial
    )


def test_writer_preserves_partial_state_refuses_overwrite_and_seals_only_complete_run(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    case = next(item for item in public_spatial_cases() if item.public_id == "invalid-provenance")
    result = replace(evaluate_spatial_public_case(case), ordinal=0)
    catalog_hash = canonical_content_hash({"case_hashes": (case.semantic_content_hash,)})
    repository_root = Path(__file__).resolve().parents[3]
    manifest = build_spatial_public_manifest(
        repository_root=repository_root,
        max_workers=1,
    )
    monkeypatch.setattr(reporting, "SPATIAL_PUBLIC_CASE_COUNT", 1)
    request_order = ((case.public_id, case.request.request_content_hash),)
    manifest_semantic_hash = canonical_content_hash(
        {
            "manifest_version": manifest.manifest_version,
            "simulation_only": True,
            "hidden_used": False,
            "source_freeze_hash": manifest.source_freeze_hash,
            "catalog_content_hash": catalog_hash,
            "request_order": request_order,
        }
    )
    manifest_content_hash = canonical_content_hash(
        {
            "semantic_content_hash": manifest_semantic_hash,
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
        request_order=request_order,
        semantic_content_hash=manifest_semantic_hash,
        content_hash=manifest_content_hash,
    )
    audit_semantic_hash = canonical_content_hash(
        {
            "report_version": reporting.SPATIAL_PUBLIC_REPORT_VERSION,
            "catalog_content_hash": catalog_hash,
            "case_result_hashes": (result.semantic_content_hash,),
            "relation_failures": (),
            "parity_case_id": case.public_id,
            "parity_passed": True,
            "hard_failures": (),
        }
    )
    audit_report_hash = canonical_content_hash(
        {
            "semantic_content_hash": audit_semantic_hash,
            "case_report_hashes": (result.report_content_hash,),
        }
    )
    audit = SpatialPublicAudit(
        report_version=reporting.SPATIAL_PUBLIC_REPORT_VERSION,
        simulation_only=True,
        hidden_used=False,
        catalog_content_hash=catalog_hash,
        case_results=(result,),
        relation_failures=(),
        parity_case_id=case.public_id,
        parity_passed=True,
        hard_failures=(),
        limitations=("test_only",),
        semantic_content_hash=audit_semantic_hash,
        report_content_hash=audit_report_hash,
        elapsed_nonqualification_ns=0,
    )
    output = tmp_path / "spatial-public"
    writer = SpatialPublicOutputWriter(output, manifest, repository_root=repository_root)
    monkeypatch.setattr(writer, "_verify_git_state", lambda: None)
    writer.start()

    with pytest.raises(RuntimeError, match="every manifest case"):
        writer.complete(audit)
    writer.write_case(result)
    with pytest.raises(FileExistsError):
        writer.write_case(result)
    results_path, summary_path, receipt_path = writer.complete(audit)

    assert results_path.exists() and summary_path.exists()
    assert receipt_path is not None and receipt_path.exists()
    assert not (output / "run_state.incomplete.json").exists()
    assert (output / "run_state.complete.json").exists()
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    assert receipt["qualified"] is True
    assert receipt["case_count"] == 1
    plot = next((output / "requests").glob("*/path.png"))
    assert plot.stat().st_size > 0
    with pytest.raises(FileExistsError):
        SpatialPublicOutputWriter(output, manifest, repository_root=repository_root).start()


def test_search_core_still_has_no_reporting_or_corpus_dependency() -> None:
    import ast

    source = Path(__file__).parents[1] / "src" / "hospital_path_lab" / "spatial_oracle_lattice.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    imported = " ".join(
        node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
    )
    names = " ".join(node.id for node in ast.walk(tree) if isinstance(node, ast.Name))

    for forbidden in (
        "spatial_oracle_reporting",
        "dynamic_corpus",
        "expectation_category",
        "oracle_spec",
        "hidden",
    ):
        assert forbidden not in imported
        assert forbidden not in names
