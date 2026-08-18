from __future__ import annotations

import json

import pytest

from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
)
from hospital_path_lab.r5c_observation_diagnostic import (
    run_r5c_crossing_completion_diagnostic,
)
from hospital_path_lab.r7_failure_trace import (
    R7_FAILURE_RUN_MANIFEST_SCHEMA_VERSION,
    R7_FAILURE_TRACE_SCHEMA_VERSION,
    R7FailureTraceCollector,
    write_r7_failure_run_manifest,
)

_FULL_OBSERVATION_HORIZON_TICKS = 1_600


def _run_prefix(*, side_index: int, profile, seed: int, tick_limit: int):
    trace = R7FailureTraceCollector()
    result = run_r5c_crossing_completion_diagnostic(
        side_index=side_index,
        profile=profile,
        tick_limit=tick_limit,
        observation_horizon_ticks=_FULL_OBSERVATION_HORIZON_TICKS,
        observation_seed=seed,
        failure_trace=trace,
    )
    assert tuple(record["tick"] for record in trace.records) == tuple(range(tick_limit))
    return result, trace.records


def test_trace_is_out_of_band_and_hash_chained(tmp_path) -> None:
    without_trace = run_r5c_crossing_completion_diagnostic(
        side_index=0,
        profile=NORMAL_OBSERVATION_PROFILE,
        tick_limit=82,
        observation_horizon_ticks=_FULL_OBSERVATION_HORIZON_TICKS,
        observation_seed=1993037174228324916,
    )
    trace = R7FailureTraceCollector()
    with_trace = run_r5c_crossing_completion_diagnostic(
        side_index=0,
        profile=NORMAL_OBSERVATION_PROFILE,
        tick_limit=82,
        observation_horizon_ticks=_FULL_OBSERVATION_HORIZON_TICKS,
        observation_seed=1993037174228324916,
        failure_trace=trace,
    )

    assert with_trace == without_trace
    assert trace.records[0]["previous_record_hash"] == "TRACE_START"
    assert all(
        current["previous_record_hash"] == previous["record_content_hash"]
        for previous, current in zip(trace.records, trace.records[1:], strict=False)
    )
    output = tmp_path / "tick-trace.jsonl"
    trace.write_jsonl(output)
    written = tuple(json.loads(line) for line in output.read_text(encoding="utf-8").splitlines())
    assert tuple(record["record_content_hash"] for record in written) == tuple(
        record["record_content_hash"] for record in trace.records
    )

    manifest_path = tmp_path / "run-manifest.json"
    manifest = write_r7_failure_run_manifest(
        manifest_path,
        git_head="a" * 40,
        git_tree="b" * 40,
        working_tree_clean=True,
        public_case_id="public-case",
        side="left",
        profile_name="normal",
        observation_seed=123,
        tick_limit=2,
        control_period_s=0.05,
        observation_period_s=0.10,
        source_file_hashes={"source.py": "c" * 64},
    )
    persisted = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert persisted == manifest
    assert manifest["schema"] == R7_FAILURE_RUN_MANIFEST_SCHEMA_VERSION
    assert manifest["trace_schema_version"] == R7_FAILURE_TRACE_SCHEMA_VERSION
    assert len(manifest["manifest_content_hash"]) == 64
    with pytest.raises(FileExistsError):
        write_r7_failure_run_manifest(
            manifest_path,
            git_head="a" * 40,
            git_tree="b" * 40,
            working_tree_clean=True,
            public_case_id="public-case",
            side="left",
            profile_name="normal",
            observation_seed=123,
            tick_limit=2,
            control_period_s=0.05,
            observation_period_s=0.10,
            source_file_hashes={"source.py": "c" * 64},
        )


def test_normal_left_seed_1993037174228324916_does_not_continue_while_braking() -> None:
    result, records = _run_prefix(
        side_index=0,
        profile=NORMAL_OBSERVATION_PROFILE,
        seed=1993037174228324916,
        tick_limit=260,
    )

    assert result.hard_failures == ()
    assert not any(
        record["temporal_authorization_phase"] == "continuation"
        and (
            record["gate_state_before"] in {"braking", "holding"}
            or record["gate_state_after"] in {"braking", "holding"}
        )
        for record in records
    )
    for record in records:
        if record["runtime_present_before"] is False and record["runtime_present_after"] is True:
            assert record["gate_state_before"] == "holding"
            assert record["reference_stop_epoch"] == record["stop_epoch_before"]


def test_normal_right_seed_4525333994236990214_keeps_active_section_representable() -> None:
    result, records = _run_prefix(
        side_index=1,
        profile=NORMAL_OBSERVATION_PROFILE,
        seed=4525333994236990214,
        tick_limit=504,
    )

    assert result.hard_failures == ()
    record = records[503]
    assert record["controller_status"] != "section_execution_failed"
    assert record["executor_active_before"] is not None
    assert record["window_first_section"] <= record["executor_active_before"]
    assert record["executor_active_before"] <= record["window_last_section"]


def test_stress_left_seed_6422064046178126625_requires_gate_confirmed_frames() -> None:
    result, records = _run_prefix(
        side_index=0,
        profile=STRESS_OBSERVATION_PROFILE,
        seed=6422064046178126625,
        tick_limit=533,
    )

    assert result.hard_failures == ()
    assert result.release_ticks == ()
    assert result.first_motion_tick is None
    assert not any(record["controller_called"] for record in records)
    for record in records:
        if record["release_permitted"]:
            assert record["confirmed_safe_frame_count_before"] >= 11


def test_normal_right_seed_8970341022568507592_completes_after_stop_bound_recovery() -> None:
    trace = R7FailureTraceCollector()
    result = run_r5c_crossing_completion_diagnostic(
        side_index=1,
        profile=NORMAL_OBSERVATION_PROFILE,
        tick_limit=1_600,
        observation_seed=8970341022568507592,
        failure_trace=trace,
    )

    assert result.hard_failures == ()
    assert result.outcome.value == "completed"
    assert result.completion_tick == 1_178
    assert result.post_pass_proof_tick is not None
    assert result.follow_original_release_tick is not None
    assert trace.records[-1]["gate_state_after"] == "completed"


def test_normal_left_seed_6422064046178126625_remains_fail_closed_without_pass_proof() -> None:
    trace = R7FailureTraceCollector()
    result = run_r5c_crossing_completion_diagnostic(
        side_index=0,
        profile=NORMAL_OBSERVATION_PROFILE,
        tick_limit=1_600,
        observation_seed=6422064046178126625,
        failure_trace=trace,
    )

    assert result.hard_failures == ()
    assert result.outcome.value == "conservative_hold"
    assert result.post_pass_proof_tick is None
    assert result.follow_original_release_tick is None
    assert trace.records[-1]["directional_status"] == "empty_frame"
    assert trace.records[-1]["release_input_usable"] is False
    assert trace.records[-1]["gate_state_after"] == "holding"
