from __future__ import annotations

import pytest

from hospital_path_lab.dynamic_witness_events import ground_truth_hazard_intervals
from hospital_path_lab.local_reference_contracts import (
    LocalManeuverKind,
    ReferenceSectionKind,
)
from hospital_path_lab.r5b_restop_execution import (
    R5B_RESTOP_FIRST_RELEASE_TICK,
    R5B_RESTOP_MINIMUM_INTERMEDIATE_PROGRESS_M,
    R5B_RESTOP_SECOND_HAZARD_DELAY_S,
    build_r5b_follow_reference,
    build_r5b_restop_evidence,
    run_r5b_restop_case,
)


def test_restop_evidence_keeps_two_ordered_public_hazards() -> None:
    evidence = build_r5b_restop_evidence()
    hazards = ground_truth_hazard_intervals(evidence.controller_world)

    assert len(hazards) == 2
    assert hazards[0].starts_at_s == pytest.approx(0.0)
    assert hazards[0].ends_at_s == pytest.approx(evidence.first_hazard_end_s)
    assert hazards[1].starts_at_s == pytest.approx(evidence.second_hazard_start_s)
    assert hazards[1].ends_at_s == pytest.approx(evidence.second_hazard_end_s)
    assert evidence.second_hazard_start_s > evidence.first_hazard_end_s
    assert pytest.approx(7.0) == R5B_RESTOP_SECOND_HAZARD_DELAY_S
    assert all(actor.active_from_s == 0.0 for actor in evidence.controller_world.actors)
    assert all(
        actor.active_until_s == evidence.controller_world.duration_s
        for actor in evidence.controller_world.actors
    )


def test_restop_reissues_distinct_original_path_reference_for_each_stop_epoch() -> None:
    evidence = build_r5b_restop_evidence()
    first = build_r5b_follow_reference(
        evidence,
        current_pose=evidence.controller_world.initial_state.pose,
        stop_epoch=1,
        valid_from_tick=R5B_RESTOP_FIRST_RELEASE_TICK,
    )
    second = build_r5b_follow_reference(
        evidence,
        current_pose=evidence.controller_world.initial_state.pose,
        stop_epoch=2,
        valid_from_tick=264,
    )

    for bundle, stop_epoch in ((first, 1), (second, 2)):
        assert bundle.validation.passed
        assert bundle.reference.maneuver_kind is LocalManeuverKind.FOLLOW_ORIGINAL
        assert len(bundle.reference.sections) == 1
        assert (
            bundle.reference.sections[0].section_kind
            is ReferenceSectionKind.FOLLOW_ORIGINAL
        )
        assert bundle.reference.stop_epoch == stop_epoch
        assert bundle.reference.validity.required_stop_epoch == stop_epoch
    assert first.reference.reference_session_id != second.reference.reference_session_id
    assert first.reference.reference_content_hash != second.reference.reference_content_hash


def test_cpp_dwb_restarts_twice_and_completes_public_restop_case() -> None:
    result = run_r5b_restop_case()

    assert result.passed
    assert result.completed
    assert result.first_release_tick == R5B_RESTOP_FIRST_RELEASE_TICK
    assert result.first_motion_tick == 45
    assert result.second_stop_tick == 232
    assert result.second_stop_epoch == 2
    assert result.second_release_tick == 264
    assert result.completion_tick == 490
    assert result.intermediate_progress_m >= R5B_RESTOP_MINIMUM_INTERMEDIATE_PROGRESS_M
    assert result.minimum_actor_clearance_m == pytest.approx(0.10016717226362759)
    assert result.minimum_actor_clearance_tick == result.second_stop_tick
    assert result.gate_override_count == 0
    assert result.controller_session_count == 2
    assert result.native_full_core_used
    assert result.hard_failures == ()
