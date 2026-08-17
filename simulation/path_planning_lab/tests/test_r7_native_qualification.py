from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

import hospital_path_lab.r7_native_qualification as r7
from hospital_path_lab.dynamic_observation import dynamic_observation_content_hash
from hospital_path_lab.local_algorithms.dwb_reference.cpp_full_core import (
    CPP_DWB_FULL_CORE_AVAILABLE,
)
from hospital_path_lab.map_factory import canonical_content_hash


def test_r7_snapshot_catalog_is_the_frozen_five_case_set() -> None:
    cases = r7.r7_snapshot_cases()

    assert tuple(case_id for case_id, _, _ in cases) == (
        "actor-0-free",
        "actor-1-active",
        "actor-2-active",
        "corner-static-forbidden",
        "staggered-risk-multisegment",
    )
    assert tuple(metadata["actor_tube_count"] for _, _, metadata in cases) == (
        0,
        1,
        2,
        1,
        1,
    )
    assert cases[3][2]["has_static_occupancy"] is True
    assert cases[3][2]["has_forbidden_cells"] is True
    assert cases[4][2]["reference_path_segment_count"] >= 2


def test_r7_retimed_snapshots_are_fresh_unique_and_monotonic() -> None:
    _, base, _ = r7.r7_snapshot_cases()[1]
    variants = tuple(
        r7.retime_controller_snapshot(base, offset=index) for index in range(3)
    )

    assert tuple(item.tick_id for item in variants) == (
        base.tick_id,
        base.tick_id + 1,
        base.tick_id + 2,
    )
    assert len({item.input_content_hash for item in variants}) == 3
    assert len({item.observation_revision for item in variants}) == 3
    for variant in variants:
        frame = variant.validated_observation.frame
        assert frame is not None
        assert frame.content_hash == dynamic_observation_content_hash(frame)
        assert variant.actor_tubes is not None
        assert variant.actor_tubes.source_content_hash == frame.content_hash


@pytest.mark.skipif(
    not CPP_DWB_FULL_CORE_AVAILABLE,
    reason="optional C++ full DWB core has not been built",
)
def test_r7_native_parity_and_short_timing_use_the_real_native_core() -> None:
    case = (r7.r7_snapshot_cases()[0],)

    parity = r7.run_native_parity(case)
    timing = r7.run_native_timing(case, warmups=1, repeats=2)

    assert parity["passed"] is True
    assert parity["records"][0]["native_full_core_used"] is True
    assert timing["sample_count"] == 2
    assert timing["parallelized"] is False
    assert timing["active_child_process_ids_before"] == []
    assert timing["active_child_process_ids_after"] == []
    assert timing["memory_after"]["working_set_bytes"] is not None
    assert timing["memory_after"]["page_fault_count"] is not None


def test_r7_r6_receipt_validation_rejects_tamper(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repository_root = Path(__file__).resolve().parents[3]
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=repository_root,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    payload = {
        "case_catalog_hash": "catalog",
        "hard_failure_count": 0,
        "head": head,
        "hidden_executed": False,
        "native_dwb_full_core_sha256": "native",
        "required_case_count": 17,
        "result_set_hash": "result",
        "schema": r7.R6_RECEIPT_SCHEMA,
        "source_freeze_hash": "source",
        "tree": "tree",
        "wall_clock_is_qualification": False,
    }
    receipt_hash = canonical_content_hash(payload)
    receipt = {**payload, "receipt_content_hash": receipt_hash}
    path = tmp_path / "receipt.json"
    path.write_text(json.dumps(receipt), encoding="utf-8")
    monkeypatch.setattr(r7, "R6_EXPECTED_RECEIPT_HASH", receipt_hash)
    monkeypatch.setattr(r7, "R6_EXPECTED_RESULT_HASH", "result")

    assert r7.validate_r6_receipt(repository_root, path)["passed"] is True

    receipt["hard_failure_count"] = 1
    path.write_text(json.dumps(receipt), encoding="utf-8")
    assert r7.validate_r6_receipt(repository_root, path)["passed"] is False


def test_r7_source_freeze_and_machine_metadata_are_explicit() -> None:
    repository_root = Path(__file__).resolve().parents[3]

    frozen = r7.source_freeze(repository_root)
    machine = r7.machine_metadata()

    assert len(frozen["records"]) == 18
    assert len(frozen["content_hash"]) == 64
    assert machine["logical_core_count"]
    assert machine["process_affinity"]
