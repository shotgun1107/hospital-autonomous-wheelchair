from __future__ import annotations

from pathlib import Path

import pytest

from hospital_path_lab.local_reference_contracts import ReferenceBuildStatus
from hospital_path_lab.local_reference_reporting import public_local_reference_cases
from hospital_path_lab.persistent_controller_reporting import (
    PersistentPublicOutputWriter,
    PersistentPublicRunStatus,
    _is_ordered_subsequence,
    _maximum_forward_progress_m,
    build_persistent_public_manifest,
    evaluate_persistent_public_case,
    evaluate_persistent_public_cases,
)


@pytest.fixture(scope="module")
def public_cases():
    return public_local_reference_cases()


def test_non_ready_public_case_never_calls_a_controller(public_cases) -> None:
    case = next(
        item
        for item in public_cases
        if item.expected_build_status is ReferenceBuildStatus.NO_REFERENCE
    )

    result = evaluate_persistent_public_case(case, tick_limit_override=2)

    assert result.reference_status is ReferenceBuildStatus.NO_REFERENCE
    assert result.controller_call_count == 0
    assert result.paired_input_hash is None
    assert result.rpp_result is None
    assert result.dwb_result is None
    assert result.hard_failures == ()


def test_deadlock_window_counts_recovery_after_a_short_backward_transition() -> None:
    assert _maximum_forward_progress_m(((0, 1.15), (1, 1.12), (2, 1.15))) == pytest.approx(
        0.03
    )
    assert _maximum_forward_progress_m(((0, 1.15), (1, 1.15), (2, 1.15))) == 0.0


def test_crossing_relation_allows_an_extra_ordered_return_section() -> None:
    wide = ("depart", "rotate", "depart", "bypass", "return", "rotate", "rejoin")
    crossing = (
        "depart",
        "rotate",
        "depart",
        "bypass",
        "return",
        "rotate",
        "return",
        "rejoin",
    )

    assert _is_ordered_subsequence(wide, crossing)
    assert not _is_ordered_subsequence(wide, tuple(reversed(crossing)))


def test_ready_pair_uses_one_frozen_input_and_fresh_controller_runs(public_cases) -> None:
    case = next(item for item in public_cases if item.expected_candidate_count == 1)

    first = evaluate_persistent_public_case(case, tick_limit_override=2)
    second = evaluate_persistent_public_case(case, tick_limit_override=2)

    assert first.rpp_result is not None
    assert first.dwb_result is not None
    assert first.rpp_result.paired_input_hash == first.dwb_result.paired_input_hash
    assert first.rpp_result.reference_session_id == first.dwb_result.reference_session_id
    assert first.rpp_result.status is PersistentPublicRunStatus.TIMED_OUT
    assert first.dwb_result.status is PersistentPublicRunStatus.TIMED_OUT
    assert first.controller_call_count == 4
    assert first.semantic_content_hash == second.semantic_content_hash
    assert first.worker_pid_nonqualification == second.worker_pid_nonqualification


def test_process_results_preserve_input_order_and_serial_semantics(public_cases) -> None:
    selected = (public_cases[0], public_cases[4])
    serial = evaluate_persistent_public_cases(
        selected,
        max_workers=1,
        tick_limit_override=2,
    )
    process = evaluate_persistent_public_cases(
        selected,
        max_workers=2,
        tick_limit_override=2,
    )

    assert tuple(item.ordinal for item in process) == (0, 4)
    assert tuple(item.semantic_content_hash for item in process) == tuple(
        item.semantic_content_hash for item in serial
    )
    assert process[0].worker_pid_nonqualification != process[1].worker_pid_nonqualification
    assert process[0].rpp_result is not None
    assert process[0].dwb_result is not None


def test_limited_manifest_is_report_only_and_writer_preserves_partial_state(
    public_cases,
    tmp_path: Path,
) -> None:
    repository_root = Path(__file__).resolve().parents[3]
    manifest = build_persistent_public_manifest(
        repository_root=repository_root,
        max_workers=1,
        public_case_limit=5,
        tick_limit_override=2,
    )
    assert not manifest.sealing_run
    assert len(manifest.case_order) == 5

    writer = PersistentPublicOutputWriter(
        tmp_path / "partial-r5",
        manifest,
        repository_root=repository_root,
    )
    writer.start()
    result = evaluate_persistent_public_case(public_cases[4], tick_limit_override=2)
    writer.write_case(result)

    assert (writer.output_dir / "run-manifest.json").is_file()
    assert (writer.output_dir / "partial-state.json").is_file()
    assert (writer.output_dir / "cases" / "04-narrow-corridor" / "paired-summary.json").is_file()
    assert not (writer.output_dir / "qualification-receipt.json").exists()


def test_full_manifest_freezes_all_21_cases_without_development_limits() -> None:
    repository_root = Path(__file__).resolve().parents[3]
    manifest = build_persistent_public_manifest(
        repository_root=repository_root,
        max_workers=8,
    )

    assert manifest.sealing_run
    assert len(manifest.case_order) == 21
    assert manifest.public_case_limit is None
    assert manifest.tick_limit_override is None
    assert not manifest.hidden_used
