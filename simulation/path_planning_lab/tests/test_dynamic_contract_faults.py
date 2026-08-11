from __future__ import annotations

from hospital_path_lab.dynamic_corpus import (
    DynamicContractFaultDomain,
    DynamicContractFaultResponse,
    DynamicExpectationCategory,
    dynamic_contract_fault_cases,
    generate_dynamic_corpus,
    generate_episode_ground_truth_frames,
)
from hospital_path_lab.dynamic_observation import (
    BOUNDARY_300_OBSERVATION_PROFILE,
    NORMAL_OBSERVATION_PROFILE,
    DynamicObservationAvailability,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    FourFrameBurstDropout,
    generate_dynamic_observation_slots,
)


def _empty_actor_episode():
    return next(
        episode
        for episode in generate_dynamic_corpus()
        if episode.expectation_category is DynamicExpectationCategory.OBSERVATION_INVALID
    )


def _source(episode):
    return DynamicObservationSourceIdentity(
        stream_id="fault-stream",
        episode_id=episode.episode_id,
        episode_seed=episode.seed,
        map_id=episode.map_id,
        map_revision=1,
    )


def test_fault_corpus_contains_every_frozen_observation_authority_and_deadline_case() -> None:
    cases = dynamic_contract_fault_cases()
    assert len(cases) == 25
    assert len({case.case_id for case in cases}) == len(cases)
    assert {case.domain for case in cases} == set(DynamicContractFaultDomain)
    assert sum(case.domain is DynamicContractFaultDomain.OBSERVATION for case in cases) == 13
    assert sum(case.domain is DynamicContractFaultDomain.AUTHORITY for case in cases) == 7
    assert sum(case.domain is DynamicContractFaultDomain.DEADLINE for case in cases) == 5
    by_fault = {case.injected_fault: case.expected_response for case in cases}
    assert by_fault["fresh_empty_frame"] is (
        DynamicContractFaultResponse.CONTINUE_WITH_FRESH_EMPTY
    )
    assert by_fault["single_dropout"] is (
        DynamicContractFaultResponse.HOLD_LAST_VALID_FRAME
    )
    assert by_fault["age_equal_ttl"] is DynamicContractFaultResponse.ACCEPT_TTL_BOUNDARY
    assert by_fault["age_greater_than_ttl"] is (
        DynamicContractFaultResponse.BRAKE_AND_HOLD
    )
    assert by_fault["result_50ms"] is DynamicContractFaultResponse.ACCEPT_CURRENT_TICK
    assert by_fault["result_51ms"] is DynamicContractFaultResponse.DISCARD_RESULT


def test_fresh_empty_is_valid_but_no_frame_is_not_interpreted_as_empty() -> None:
    episode = _empty_actor_episode()
    slots = generate_dynamic_observation_slots(
        generate_episode_ground_truth_frames(episode),
        source=_source(episode),
        profile=NORMAL_OBSERVATION_PROFILE,
    )
    frame_slot = next(slot for slot in slots if slot.frame is not None)
    validator = DynamicObservationValidator(_source(episode), NORMAL_OBSERVATION_PROFILE)
    assert validator.accept(
        frame_slot.frame,
        received_at_s=frame_slot.scheduled_delivery_at_s,
    ).accepted
    snapshot = validator.snapshot(control_time_s=frame_slot.scheduled_delivery_at_s)
    assert snapshot.availability is DynamicObservationAvailability.FRESH
    assert snapshot.frame is not None
    assert not snapshot.frame.tracks
    assert not snapshot.last_event_was_no_frame

    next_sequence = frame_slot.sequence + 1
    no_frame_time = (
        next_sequence * NORMAL_OBSERVATION_PROFILE.observation_period_s
        + NORMAL_OBSERVATION_PROFILE.latency_s
    )
    validator.record_no_frame(
        sequence=next_sequence,
        delivery_time_s=no_frame_time,
    )
    held = validator.snapshot(control_time_s=no_frame_time)
    assert held.frame == snapshot.frame
    assert held.last_event_was_no_frame


def test_ttl_boundary_is_fresh_and_greater_than_ttl_is_stale() -> None:
    episode = _empty_actor_episode()
    slots = generate_dynamic_observation_slots(
        generate_episode_ground_truth_frames(episode),
        source=_source(episode),
        profile=BOUNDARY_300_OBSERVATION_PROFILE,
    )
    frame = next(slot.frame for slot in slots if slot.frame is not None)
    validator = DynamicObservationValidator(
        _source(episode),
        BOUNDARY_300_OBSERVATION_PROFILE,
    )
    assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted
    assert validator.snapshot(control_time_s=frame.observed_at_s + 0.300).availability is (
        DynamicObservationAvailability.FRESH
    )
    assert validator.snapshot(control_time_s=frame.observed_at_s + 0.350).availability is (
        DynamicObservationAvailability.STALE
    )


def test_four_frame_burst_expires_last_valid_frame_and_requires_hold() -> None:
    episode = _empty_actor_episode()
    source = _source(episode)
    slots = generate_dynamic_observation_slots(
        generate_episode_ground_truth_frames(episode),
        source=source,
        profile=NORMAL_OBSERVATION_PROFILE,
        burst_dropout=FourFrameBurstDropout(start_sequence=1),
    )
    validator = DynamicObservationValidator(source, NORMAL_OBSERVATION_PROFILE)
    first = slots[0]
    assert first.frame is not None
    assert validator.accept(
        first.frame,
        received_at_s=first.scheduled_delivery_at_s,
    ).accepted
    for slot in slots[1:5]:
        assert slot.frame is None
        validator.record_no_frame(
            sequence=slot.sequence,
            delivery_time_s=slot.scheduled_delivery_at_s,
        )
    assert validator.snapshot(control_time_s=slots[4].scheduled_delivery_at_s).availability is (
        DynamicObservationAvailability.STALE
    )
