from __future__ import annotations

import pytest

from hospital_path_lab.dynamic_contracts import DYNAMIC_CONTROL_PERIOD_S
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_witness_contracts import project_public_witness_world
from hospital_path_lab.dynamic_witness_restop import search_multi_hazard_restop
from hospital_path_lab.dynamic_witness_search import generate_wait_and_follow_witness
from hospital_path_lab.r2b_entry_coverage import build_r2b_entry_coverage_contract
from hospital_path_lab.r2b_ultrasonic_audit import (
    R2BUltrasonicCoverageStatus,
    audit_r2b_ultrasonic_entry_coverage,
)


@pytest.fixture(scope="module")
def v6_audit():
    episode = next(
        item
        for item in generate_dynamic_v6_public_corpus()
        if item.variant == "second-risk-after-corner"
    )
    world = project_public_witness_world(episode)
    departure_tick = round(world.actors[0].active_from_s / DYNAMIC_CONTROL_PERIOD_S)
    witness = generate_wait_and_follow_witness(
        world,
        departure_tick=departure_tick,
        linear_target_mps=0.20,
    )
    assert witness is not None
    contract = build_r2b_entry_coverage_contract(world)
    return audit_r2b_ultrasonic_entry_coverage(world, witness, contract)


@pytest.fixture(scope="module")
def legacy_audit():
    episode = next(
        item
        for item in generate_dynamic_corpus()
        if item.split is DynamicCorpusSplit.GOLDEN
        and item.expectation_category is DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP
    )
    world = project_public_witness_world(episode)
    search = search_multi_hazard_restop(world)
    assert search.witness is not None
    contract = build_r2b_entry_coverage_contract(world)
    return audit_r2b_ultrasonic_entry_coverage(world, search.witness, contract)


@pytest.mark.parametrize("fixture_name", ("v6_audit", "legacy_audit"))
def test_original_internal_appearance_has_no_preentry_ultrasonic_detection(
    fixture_name: str,
    request: pytest.FixtureRequest,
) -> None:
    audit = request.getfixturevalue(fixture_name)
    assert all(
        actor.status is not R2BUltrasonicCoverageStatus.PREENTRY_DETECTED
        and actor.raw_preentry_detection_count == 0
        for actor in audit.source.actor_coverage
    )


def test_v6_covered_world_exposes_one_short_lead_and_one_blind_entry(v6_audit) -> None:
    first, second = v6_audit.covered.actor_coverage
    assert first.status is R2BUltrasonicCoverageStatus.PREENTRY_DETECTED
    assert first.raw_preentry_detection_count == 1
    assert first.maximum_raw_preentry_lead_s == pytest.approx(0.353)
    assert second.status is R2BUltrasonicCoverageStatus.DETECTED_AFTER_ENTRY
    assert second.raw_preentry_detection_count == 0


def test_legacy_covered_world_has_raw_preentry_echo_but_no_accepted_frame(
    legacy_audit,
) -> None:
    actor = legacy_audit.covered.actor_coverage[0]
    assert actor.status is R2BUltrasonicCoverageStatus.PREENTRY_DETECTED
    assert actor.raw_preentry_detection_count == 6
    assert actor.maximum_raw_preentry_lead_s == pytest.approx(2.4042820512820513)
    assert actor.accepted_preentry_detection_count == 0


@pytest.mark.parametrize("fixture_name", ("v6_audit", "legacy_audit"))
def test_seven_sensor_full_frames_are_stale_under_frozen_r2b_ttl(
    fixture_name: str,
    request: pytest.FixtureRequest,
) -> None:
    audit = request.getfixturevalue(fixture_name)
    for world in (audit.source, audit.covered):
        assert world.scan_count > 0
        assert world.accepted_frame_count == 0
        assert world.stale_frame_count == world.scan_count
        assert world.invalid_frame_count == 0
    assert "full_seven_sensor_frame_exceeds_frozen_300ms_ttl" in audit.failures


@pytest.mark.parametrize("fixture_name", ("v6_audit", "legacy_audit"))
def test_sequential_rate_and_range_only_contract_keep_r2b_unqualified(
    fixture_name: str,
    request: pytest.FixtureRequest,
) -> None:
    audit = request.getfixturevalue(fixture_name)
    assert audit.per_sensor_repeat_period_s == pytest.approx(0.427)
    assert audit.frozen_history_frame_count == 20
    assert audit.frozen_history_span_s == pytest.approx(1.9)
    assert audit.sequential_history_span_s == pytest.approx(8.113)
    assert audit.range_only_track_contract_supported is False
    assert audit.r2b_observation_qualified is False
    assert "per_sensor_sampling_cannot_supply_frozen_10hz_history" in audit.failures
    assert "range_only_frame_has_no_actor_identity_position_or_velocity" in audit.failures


def test_r2b_ultrasonic_audit_is_deterministic(v6_audit) -> None:
    episode = next(
        item
        for item in generate_dynamic_v6_public_corpus()
        if item.variant == "second-risk-after-corner"
    )
    world = project_public_witness_world(episode)
    departure_tick = round(world.actors[0].active_from_s / DYNAMIC_CONTROL_PERIOD_S)
    witness = generate_wait_and_follow_witness(
        world,
        departure_tick=departure_tick,
        linear_target_mps=0.20,
    )
    assert witness is not None
    repeated = audit_r2b_ultrasonic_entry_coverage(
        world,
        witness,
        build_r2b_entry_coverage_contract(world),
    )
    assert repeated == v6_audit
    assert repeated.content_hash == v6_audit.content_hash


def test_controller_facing_ultrasonic_frame_identity_does_not_leak_into_audit_claim(
    v6_audit,
) -> None:
    assert all(event.actor_binding_id for event in v6_audit.covered.detection_events)
    assert (
        "controller_facing_frame_contains_no_actor_identity_or_ground_truth"
        in v6_audit.limitations
    )
