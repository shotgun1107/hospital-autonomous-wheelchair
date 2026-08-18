from __future__ import annotations

from dataclasses import replace

import pytest

from hospital_path_lab.dynamic_observation import NORMAL_OBSERVATION_PROFILE
from hospital_path_lab.r5b_temporal_reference import build_r5b_crossing_reference_bundles
from hospital_path_lab.r5c_observation_diagnostic import _ProfileObservationStream
from hospital_path_lab.r7_hidden_qualification import (
    R7_HIDDEN_OBSERVATION_VERSION,
    R7HiddenCaseResult,
    _clearances_pass,
    _normal_progress_is_ordered,
    audit_hidden_results,
    build_hidden_case_specs,
    hidden_seed_commitment,
)


def test_hidden_catalog_is_exact_paired_and_deterministic() -> None:
    first = build_hidden_case_specs(123_456)
    second = build_hidden_case_specs(123_456)

    assert first == second
    assert len(first) == 20
    assert tuple(item.ordinal for item in first) == tuple(range(20))
    assert len({item.case_id for item in first}) == 20
    assert R7_HIDDEN_OBSERVATION_VERSION == "r7-hidden-observation-v2"
    assert all(item.case_id.startswith("hidden-v3-") for item in first)
    assert len({item.content_hash for item in first}) == 20
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

    assert build_hidden_case_specs(123_457) != first
    assert hidden_seed_commitment(123_456) == hidden_seed_commitment(123_456)
    assert hidden_seed_commitment(123_457) != hidden_seed_commitment(123_456)


def test_hidden_v3_commitment_does_not_reuse_v1_namespace() -> None:
    from hashlib import sha256

    root_seed = 123_456
    old = sha256(f"r7-hidden-observation-v1:{root_seed}".encode()).hexdigest()
    assert hidden_seed_commitment(root_seed) != old


@pytest.mark.parametrize("bad", (-1, True, 1.5, 1 << 63))
def test_hidden_root_seed_validation_rejects_invalid_values(bad) -> None:
    with pytest.raises(ValueError, match="root_seed"):
        build_hidden_case_specs(bad)


def test_explicit_observation_seed_is_deterministic_and_changes_latent_draws() -> None:
    world = build_r5b_crossing_reference_bundles()[0].source.world

    def snapshots(seed: int | None):
        stream = _ProfileObservationStream(
            world,
            profile=NORMAL_OBSERVATION_PROFILE,
            tick_limit=80,
            stream_id="r7-hidden-test",
            mission_revision=1,
            observation_seed=seed,
        )
        return tuple(stream.tick(tick)[0] for tick in range(81))

    default = snapshots(None)
    explicit_default = snapshots(world.seed)
    other_a = snapshots(987_654_321)
    other_b = snapshots(987_654_321)

    assert default == explicit_default
    assert other_a == other_b
    default_frames = tuple(item.frame for item in default if item.frame is not None)
    other_frames = tuple(item.frame for item in other_a if item.frame is not None)
    assert default_frames
    assert other_frames
    assert tuple(item.content_hash for item in default_frames) != tuple(
        item.content_hash for item in other_frames
    )


def test_hidden_audit_requires_all_normal_complete_and_all_stress_hold() -> None:
    specs = build_hidden_case_specs(123_456)
    results = tuple(_passing_result(spec) for spec in specs)
    audit = audit_hidden_results(specs, results)

    assert audit.passed
    assert audit.normal_completed_count == 10
    assert audit.stress_holding_count == 10
    assert audit.hard_failure_count == 0

    failed = replace(results[0], passed=False, outcome="failed", content_hash="")
    rejected = audit_hidden_results(specs, (failed, *results[1:]))
    assert not rejected.passed
    assert "normal_completion_count_mismatch" in rejected.failures
    assert f"{specs[0].case_id}:expected_outcome_failed" in rejected.failures

    unsafe = replace(
        results[1],
        passed=False,
        hard_failures=("collision",),
        content_hash="",
    )
    rejected_unsafe = audit_hidden_results(specs, (results[0], unsafe, *results[2:]))
    assert not rejected_unsafe.passed
    assert rejected_unsafe.hard_failure_count == 1
    assert "hidden_hard_failure_nonzero" in rejected_unsafe.failures


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("post_pass_proof_tick", 70),
        ("follow_original_release_tick", 600),
        ("completion_tick", 700),
        ("minimum_actor_clearance_m", 0.079),
        ("minimum_static_clearance_m", 0.079),
    ),
)
def test_normal_hidden_result_rejects_bad_order_or_clearance(
    field: str,
    value,
) -> None:
    result = _passing_result(build_hidden_case_specs(123_456)[0])
    changed = replace(result, **{field: value}, content_hash="")

    assert not (
        _normal_progress_is_ordered(changed) and _clearances_pass(changed)
    )


def _passing_result(spec) -> R7HiddenCaseResult:
    normal = spec.profile_name == "normal"
    return R7HiddenCaseResult(
        ordinal=spec.ordinal,
        case_id=spec.case_id,
        replica=spec.replica,
        side_name=spec.side_name,
        profile_name=spec.profile_name,
        seed_tag=spec.seed_tag,
        expected_outcome=spec.expected_outcome,
        passed=True,
        outcome="completed" if normal else "conservative_hold",
        completion_tick=1200 if normal else None,
        post_pass_proof_tick=600 if normal else None,
        follow_original_release_tick=700 if normal else None,
        first_motion_tick=80 if normal else None,
        controller_call_count=500 if normal else 0,
        release_ticks=(80,) if normal else (),
        final_motion_state="completed" if normal else "holding",
        final_stop_epoch=2 if normal else 1,
        minimum_actor_clearance_m=0.20,
        minimum_static_clearance_m=0.30,
        gate_override_count=0,
        hard_failures=(),
        trace_content_hash=f"trace-{spec.ordinal}",
        elapsed_s=1.0,
    )
