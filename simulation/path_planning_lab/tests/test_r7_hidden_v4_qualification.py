from __future__ import annotations

from dataclasses import replace
from hashlib import sha256

import pytest

from hospital_path_lab.r7_hidden_qualification import (
    R7_HIDDEN_OBSERVATION_VERSION,
    build_hidden_case_specs,
)
from hospital_path_lab.r7_hidden_v4_qualification import (
    R7_HIDDEN_V4_OBSERVATION_VERSION,
    R7HiddenV4CaseResult,
    _stress_result_is_conditionally_safe,
    audit_hidden_v4_results,
    build_hidden_v4_case_specs,
    hidden_v4_seed_commitment,
)


def test_hidden_v4_catalog_uses_a_new_paired_deterministic_namespace() -> None:
    first = build_hidden_v4_case_specs(123_456)
    second = build_hidden_v4_case_specs(123_456)

    assert first == second
    assert len(first) == 20
    assert tuple(item.ordinal for item in first) == tuple(range(20))
    assert len({item.case_id for item in first}) == 20
    assert R7_HIDDEN_V4_OBSERVATION_VERSION == "r7-hidden-observation-v3"
    assert R7_HIDDEN_V4_OBSERVATION_VERSION != R7_HIDDEN_OBSERVATION_VERSION
    assert all(item.case_id.startswith("hidden-v4-") for item in first)
    assert all(
        item.expected_outcome
        == ("completed" if item.profile_name == "normal" else "conditionally_safe_hold")
        for item in first
    )
    for replica in range(5):
        for side_name in ("left", "right"):
            pair = tuple(
                item
                for item in first
                if item.replica == replica and item.side_name == side_name
            )
            assert tuple(item.profile_name for item in pair) == ("normal", "stress")
            assert pair[0].observation_seed == pair[1].observation_seed
            assert pair[0].seed_tag == pair[1].seed_tag

    assert build_hidden_v4_case_specs(123_457) != first
    assert build_hidden_case_specs(123_456) != first


def test_hidden_v4_commitment_reuses_neither_v1_nor_v3_namespace() -> None:
    root_seed = 123_456
    old_v1 = sha256(f"r7-hidden-observation-v1:{root_seed}".encode()).hexdigest()
    old_v3 = sha256(f"{R7_HIDDEN_OBSERVATION_VERSION}:{root_seed}".encode()).hexdigest()

    assert hidden_v4_seed_commitment(root_seed) not in {old_v1, old_v3}
    assert hidden_v4_seed_commitment(root_seed) == hidden_v4_seed_commitment(root_seed)


@pytest.mark.parametrize("bad", (-1, True, 1.5, 1 << 63))
def test_hidden_v4_root_seed_validation_rejects_invalid_values(bad) -> None:
    with pytest.raises(ValueError, match="root_seed"):
        build_hidden_v4_case_specs(bad)


def test_stress_v4_accepts_both_no_release_hold_and_authorized_release_restop() -> None:
    spec = next(
        item for item in build_hidden_v4_case_specs(123_456) if item.profile_name == "stress"
    )
    no_release = _passing_stress_result(spec, released=False)
    released = _passing_stress_result(spec, released=True)

    assert _stress_result_is_conditionally_safe(no_release)
    assert _stress_result_is_conditionally_safe(released)


@pytest.mark.parametrize(
    ("changes"),
    (
        {"first_motion_tick": None},
        {"protective_stop_started_tick": None},
        {"stop_confirmed_tick": None},
        {"confirmed_stop_ticks": ()},
        {"final_stop_epoch": 1},
        {"final_motion_state": "braking"},
        {"minimum_actor_clearance_m": 0.079},
        {"hard_failures": ("collision",)},
    ),
)
def test_stress_v4_rejects_incomplete_or_unsafe_release_restop(changes) -> None:
    spec = next(
        item for item in build_hidden_v4_case_specs(123_456) if item.profile_name == "stress"
    )
    result = _passing_stress_result(spec, released=True)
    changed = replace(result, **changes, content_hash="")

    assert not _stress_result_is_conditionally_safe(changed)


def test_hidden_v4_audit_requires_all_normal_complete_and_all_stress_policy_pass() -> None:
    specs = build_hidden_v4_case_specs(123_456)
    results = tuple(
        _passing_normal_result(spec)
        if spec.profile_name == "normal"
        else _passing_stress_result(spec, released=spec.replica % 2 == 0)
        for spec in specs
    )
    audit = audit_hidden_v4_results(specs, results)

    assert audit.passed
    assert audit.normal_completed_count == 10
    assert audit.stress_conditionally_safe_count == 10
    assert audit.stress_release_count == 6
    assert audit.hard_failure_count == 0

    failed = replace(results[1], passed=False, content_hash="")
    rejected = audit_hidden_v4_results(specs, (results[0], failed, *results[2:]))
    assert not rejected.passed
    assert "stress_conditionally_safe_count_mismatch" in rejected.failures
    assert f"{specs[1].case_id}:expected_outcome_failed" in rejected.failures


def test_historical_hidden_v3_contract_remains_unchanged() -> None:
    specs = build_hidden_case_specs(123_456)

    assert R7_HIDDEN_OBSERVATION_VERSION == "r7-hidden-observation-v2"
    assert all(item.case_id.startswith("hidden-v3-") for item in specs)
    assert all(
        item.expected_outcome
        == ("completed" if item.profile_name == "normal" else "conservative_hold")
        for item in specs
    )


def _base_result(spec, *, passed: bool, outcome: str) -> R7HiddenV4CaseResult:
    return R7HiddenV4CaseResult(
        ordinal=spec.ordinal,
        case_id=spec.case_id,
        replica=spec.replica,
        side_name=spec.side_name,
        profile_name=spec.profile_name,
        seed_tag=spec.seed_tag,
        expected_outcome=spec.expected_outcome,
        passed=passed,
        outcome=outcome,
        completion_tick=None,
        post_pass_proof_tick=None,
        follow_original_release_tick=None,
        actual_release_tick=None,
        first_motion_tick=None,
        protective_stop_started_tick=None,
        stop_confirmed_tick=None,
        controller_call_count=0,
        release_ticks=(),
        confirmed_stop_ticks=(),
        session_stop_epochs=(),
        final_motion_state="holding",
        final_stop_epoch=1,
        minimum_actor_clearance_m=0.20,
        minimum_static_clearance_m=0.30,
        gate_override_count=0,
        hard_failures=(),
        trace_content_hash=f"trace-{spec.ordinal}",
        elapsed_s=1.0,
    )


def _passing_normal_result(spec) -> R7HiddenV4CaseResult:
    return replace(
        _base_result(spec, passed=True, outcome="completed"),
        completion_tick=1_200,
        post_pass_proof_tick=600,
        follow_original_release_tick=700,
        actual_release_tick=80,
        first_motion_tick=81,
        controller_call_count=500,
        release_ticks=(80,),
        session_stop_epochs=(1,),
        final_motion_state="completed",
        final_stop_epoch=1,
        content_hash="",
    )


def _passing_stress_result(spec, *, released: bool) -> R7HiddenV4CaseResult:
    base = _base_result(spec, passed=True, outcome="conservative_hold")
    if not released:
        return base
    return replace(
        base,
        actual_release_tick=497,
        first_motion_tick=498,
        protective_stop_started_tick=499,
        stop_confirmed_tick=502,
        controller_call_count=2,
        release_ticks=(497,),
        confirmed_stop_ticks=(502,),
        session_stop_epochs=(1,),
        final_stop_epoch=2,
        content_hash="",
    )
