from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from hospital_path_lab.dynamic_contracts import DYNAMIC_CONTROL_PERIOD_S
from hospital_path_lab.dynamic_safety import DynamicMotionState
from hospital_path_lab.r5b_temporal_evidence import frozen_r2_archive_path
from hospital_path_lab.r5b_temporal_reference import (
    build_r5b_crossing_reference_bundles,
    build_r5b_temporal_reference_bundles,
)
from hospital_path_lab.r6_public_qualification import (
    R6CaseKind,
    R6ExpectedOutcome,
    audit_r6_public_results,
    public_r6_case_specs,
    run_r6_public_case,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


def test_r6_public_catalog_is_exact_and_ordered() -> None:
    specs = public_r6_case_specs(REPOSITORY_ROOT)
    assert len(specs) == 17
    assert tuple(item.ordinal for item in specs) == tuple(range(17))
    assert len({item.case_id for item in specs}) == 17
    assert sum(item.kind is R6CaseKind.SAME_DIRECTION_IDEAL for item in specs) == 10
    assert sum(item.kind is R6CaseKind.CROSSING_IDEAL for item in specs) == 2
    assert sum(item.kind is R6CaseKind.RESTOP_IDEAL for item in specs) == 1
    assert sum(item.kind is R6CaseKind.CROSSING_NORMAL for item in specs) == 2
    assert sum(item.kind is R6CaseKind.CROSSING_STRESS for item in specs) == 2

    temporal_worlds = tuple(
        item.source.world
        for item in build_r5b_temporal_reference_bundles(
            frozen_r2_archive_path(REPOSITORY_ROOT)
        )
    ) + tuple(item.source.world for item in build_r5b_crossing_reference_bundles())
    temporal_specs = tuple(
        item
        for item in specs
        if item.kind
        in {R6CaseKind.SAME_DIRECTION_IDEAL, R6CaseKind.CROSSING_IDEAL}
    )
    assert tuple(item.tick_limit for item in temporal_specs) == tuple(
        int(round(world.duration_s / DYNAMIC_CONTROL_PERIOD_S))
        for world in temporal_worlds
    )


def test_r6_stress_case_requires_zero_release_and_conservative_hold() -> None:
    spec = next(
        item
        for item in public_r6_case_specs(REPOSITORY_ROOT)
        if item.kind is R6CaseKind.CROSSING_STRESS
    )
    result = run_r6_public_case(REPOSITORY_ROOT, spec)
    assert result.expected_outcome is R6ExpectedOutcome.CONSERVATIVE_HOLD
    assert result.passed
    assert result.outcome == "conservative_hold"
    assert result.release_ticks == ()
    assert result.first_motion_tick is None
    assert result.controller_call_count == 0
    assert result.final_motion_state == DynamicMotionState.HOLDING.value
    assert result.hard_failures == ()


def test_r6_audit_rejects_one_failed_case() -> None:
    specs = public_r6_case_specs(REPOSITORY_ROOT)
    stress = tuple(
        item for item in specs if item.kind is R6CaseKind.CROSSING_STRESS
    )
    results = tuple(run_r6_public_case(REPOSITORY_ROOT, item) for item in stress)
    partial = audit_r6_public_results(stress, results)
    assert not partial.passed
    assert "required_case_catalog_incomplete" in partial.failures

    failed = replace(results[0], passed=False, content_hash="")
    failed_partial = audit_r6_public_results(stress, (failed, results[1]))
    assert not failed_partial.passed
    assert f"{stress[0].case_id}:case_failed" in failed_partial.failures
