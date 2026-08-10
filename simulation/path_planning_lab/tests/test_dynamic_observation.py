from __future__ import annotations

from dataclasses import fields, replace
from math import isclose

import pytest

from hospital_path_lab.dynamic_actor import generate_corridor_crossing_scenario
from hospital_path_lab.dynamic_contracts import (
    ActorState,
    ActorTrack,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
)
from hospital_path_lab.dynamic_observation import (
    BOUNDARY_300_OBSERVATION_PROFILE,
    BOUNDARY_350_OBSERVATION_PROFILE,
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationAvailability,
    DynamicObservationDropKind,
    DynamicObservationSourceIdentity,
    DynamicObservationValidationReason,
    DynamicObservationValidator,
    FourFrameBurstDropout,
    dynamic_observation_content_hash,
    generate_dynamic_observation_slots,
)
from hospital_path_lab.simulation import simulate_dynamic_actor_scenario


def _trace_and_source(seed: int = 20260810):
    scenario = generate_corridor_crossing_scenario(seed)
    trace = simulate_dynamic_actor_scenario(scenario)
    source = DynamicObservationSourceIdentity(
        stream_id="corridor_actor_observation_v1",
        episode_id=scenario.episode_id,
        episode_seed=scenario.seed,
        map_id="corridor_map_v1",
        map_revision=scenario.map_revision,
    )
    return trace, source


def _delivered_frames(slots):
    return tuple(slot.frame for slot in slots if slot.frame is not None)


def _rehash(frame: DynamicObservationFrame) -> DynamicObservationFrame:
    return replace(frame, content_hash=dynamic_observation_content_hash(frame))


def _wire_clone(instance, **changes):
    clone = object.__new__(type(instance))
    for field in fields(instance):
        object.__setattr__(
            clone,
            field.name,
            changes[field.name] if field.name in changes else getattr(instance, field.name),
        )
    return clone


def test_frozen_profiles_match_v5_values() -> None:
    assert (
        NORMAL_OBSERVATION_PROFILE.observation_period_s,
        NORMAL_OBSERVATION_PROFILE.latency_s,
        NORMAL_OBSERVATION_PROFILE.ttl_s,
        NORMAL_OBSERVATION_PROFILE.position_sigma_m,
        NORMAL_OBSERVATION_PROFILE.velocity_sigma_mps,
        NORMAL_OBSERVATION_PROFILE.dropout_probability,
    ) == (0.1, 0.1, 0.3, 0.03, 0.05, 0.05)
    assert (
        STRESS_OBSERVATION_PROFILE.latency_s,
        STRESS_OBSERVATION_PROFILE.position_sigma_m,
        STRESS_OBSERVATION_PROFILE.velocity_sigma_mps,
        STRESS_OBSERVATION_PROFILE.dropout_probability,
    ) == (0.25, 0.08, 0.15, 0.2)
    assert BOUNDARY_300_OBSERVATION_PROFILE.latency_s == 0.3
    assert BOUNDARY_350_OBSERVATION_PROFILE.latency_s == 0.35
    for profile in (BOUNDARY_300_OBSERVATION_PROFILE, BOUNDARY_350_OBSERVATION_PROFILE):
        assert profile.position_sigma_m == 0.0
        assert profile.velocity_sigma_mps == 0.0
        assert profile.dropout_probability == 0.0


def test_twenty_hz_truth_is_sampled_at_exact_ten_hz_with_frozen_latency() -> None:
    trace, source = _trace_and_source()
    slots = generate_dynamic_observation_slots(
        trace.ground_truth_frames,
        source=source,
        profile=BOUNDARY_300_OBSERVATION_PROFILE,
    )

    assert len(slots) == 66
    assert tuple(slot.sequence for slot in slots) == tuple(range(66))
    assert all(slot.frame is not None for slot in slots)
    assert all(
        slot.observed_at_s == sequence * 0.1
        and slot.scheduled_delivery_at_s == sequence * 0.1 + 0.3
        for sequence, slot in enumerate(slots)
    )
    assert all(
        slot.frame is not None
        and slot.frame.observed_at_s
        == trace.ground_truth_frames[2 * slot.sequence].simulation_time_s
        for slot in slots
    )


def test_normal_and_stress_are_reproducible_and_share_latent_noise_draws() -> None:
    trace, source = _trace_and_source(71)
    first_normal = generate_dynamic_observation_slots(
        trace.ground_truth_frames,
        source=source,
        profile=NORMAL_OBSERVATION_PROFILE,
    )
    second_normal = generate_dynamic_observation_slots(
        trace.ground_truth_frames,
        source=source,
        profile=NORMAL_OBSERVATION_PROFILE,
    )
    first_stress = generate_dynamic_observation_slots(
        trace.ground_truth_frames,
        source=source,
        profile=STRESS_OBSERVATION_PROFILE,
    )
    second_stress = generate_dynamic_observation_slots(
        trace.ground_truth_frames,
        source=source,
        profile=STRESS_OBSERVATION_PROFILE,
    )

    assert first_normal == second_normal
    assert first_stress == second_stress
    assert first_normal != first_stress

    paired = next(
        (normal, stress)
        for normal, stress in zip(first_normal, first_stress, strict=True)
        if normal.frame is not None and stress.frame is not None
    )
    truth = trace.ground_truth_frames[2 * paired[0].sequence].actors[0]
    normal_track = paired[0].frame.tracks[0]
    stress_track = paired[1].frame.tracks[0]
    assert isclose(
        (normal_track.observed_position.x - truth.position.x)
        / NORMAL_OBSERVATION_PROFILE.position_sigma_m,
        (stress_track.observed_position.x - truth.position.x)
        / STRESS_OBSERVATION_PROFILE.position_sigma_m,
        abs_tol=1e-12,
    )
    assert isclose(
        (normal_track.observed_velocity.y - truth.velocity.y)
        / NORMAL_OBSERVATION_PROFILE.velocity_sigma_mps,
        (stress_track.observed_velocity.y - truth.velocity.y)
        / STRESS_OBSERVATION_PROFILE.velocity_sigma_mps,
        abs_tol=1e-12,
    )


def test_dropout_removes_whole_frame_and_preserves_sequence_gaps() -> None:
    trace, source = _trace_and_source(73)
    slots = generate_dynamic_observation_slots(
        trace.ground_truth_frames,
        source=source,
        profile=STRESS_OBSERVATION_PROFILE,
    )

    dropped = tuple(slot for slot in slots if slot.frame is None)
    assert dropped
    assert all(slot.drop_kind is DynamicObservationDropKind.INDEPENDENT for slot in dropped)
    delivered_sequences = tuple(slot.frame.sequence for slot in slots if slot.frame is not None)
    assert any(
        following - current > 1
        for current, following in zip(delivered_sequences, delivered_sequences[1:], strict=False)
    )


def test_forced_four_frame_burst_does_not_shift_following_frames() -> None:
    trace, source = _trace_and_source(75)
    baseline = generate_dynamic_observation_slots(
        trace.ground_truth_frames,
        source=source,
        profile=BOUNDARY_300_OBSERVATION_PROFILE,
    )
    faulted = generate_dynamic_observation_slots(
        trace.ground_truth_frames,
        source=source,
        profile=BOUNDARY_300_OBSERVATION_PROFILE,
        burst_dropout=FourFrameBurstDropout(start_sequence=10),
    )

    assert tuple(slot.sequence for slot in faulted[10:14]) == (10, 11, 12, 13)
    assert all(slot.frame is None for slot in faulted[10:14])
    assert all(
        slot.drop_kind is DynamicObservationDropKind.FORCED_BURST for slot in faulted[10:14]
    )
    assert baseline[:10] == faulted[:10]
    assert baseline[14:] == faulted[14:]


def test_fresh_empty_frame_and_no_frame_have_distinct_state_effects() -> None:
    trace, source = _trace_and_source(77)
    first_slot = generate_dynamic_observation_slots(
        trace.ground_truth_frames[:1],
        source=source,
        profile=BOUNDARY_300_OBSERVATION_PROFILE,
    )[0]
    empty_truth = replace(trace.ground_truth_frames[2], actors=())
    empty_slot = generate_dynamic_observation_slots(
        (empty_truth,),
        source=source,
        profile=BOUNDARY_300_OBSERVATION_PROFILE,
    )[0]
    assert first_slot.frame is not None
    assert empty_slot.frame is not None
    assert empty_slot.frame.frame_kind is DynamicObservationFrameKind.EMPTY
    assert empty_slot.frame.tracks == ()

    empty_validator = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE)
    assert empty_validator.accept(
        first_slot.frame,
        received_at_s=first_slot.scheduled_delivery_at_s,
    ).accepted
    assert empty_validator.accept(
        empty_slot.frame,
        received_at_s=empty_slot.scheduled_delivery_at_s,
    ).accepted
    empty_snapshot = empty_validator.snapshot(control_time_s=empty_slot.scheduled_delivery_at_s)
    assert empty_snapshot.usable
    assert empty_snapshot.frame is not None and empty_snapshot.frame.tracks == ()
    assert empty_snapshot.last_event_was_no_frame is False

    no_frame_validator = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE)
    assert no_frame_validator.accept(
        first_slot.frame,
        received_at_s=first_slot.scheduled_delivery_at_s,
    ).accepted
    no_frame_validator.record_no_frame(
        sequence=empty_slot.sequence,
        delivery_time_s=empty_slot.scheduled_delivery_at_s,
    )
    no_frame_snapshot = no_frame_validator.snapshot(
        control_time_s=empty_slot.scheduled_delivery_at_s
    )
    assert no_frame_snapshot.availability is DynamicObservationAvailability.STALE
    assert no_frame_snapshot.frame is first_slot.frame
    assert no_frame_snapshot.frame.tracks
    assert no_frame_snapshot.last_event_was_no_frame is True


def test_no_frame_event_must_advance_sequence_and_delivery_time() -> None:
    trace, source = _trace_and_source(78)
    slots = generate_dynamic_observation_slots(
        trace.ground_truth_frames,
        source=source,
        profile=BOUNDARY_300_OBSERVATION_PROFILE,
    )
    validator = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE)
    assert slots[0].frame is not None and slots[2].frame is not None
    assert validator.accept(
        slots[0].frame,
        received_at_s=slots[0].scheduled_delivery_at_s,
    ).accepted
    validator.record_no_frame(
        sequence=slots[1].sequence,
        delivery_time_s=slots[1].scheduled_delivery_at_s,
    )

    with pytest.raises(ValueError, match="sequence must increase"):
        validator.record_no_frame(
            sequence=slots[0].sequence,
            delivery_time_s=slots[2].scheduled_delivery_at_s,
        )

    regressed_delivery = _rehash(
        replace(
            slots[2].frame,
            observed_at_s=0.05,
            delivered_at_s=0.35,
        )
    )
    result = validator.accept(regressed_delivery, received_at_s=0.40)
    assert not result.accepted
    assert DynamicObservationValidationReason.DELIVERY_TIME_REGRESSED in result.failures

    # The rejected frame must not advance event state; the real next frame remains valid.
    assert validator.accept(
        slots[2].frame,
        received_at_s=slots[2].scheduled_delivery_at_s,
    ).accepted


@pytest.mark.parametrize(
    ("change", "reason"),
    (
        ({"stream_id": "wrong"}, DynamicObservationValidationReason.STREAM_ID_MISMATCH),
        ({"episode_id": "wrong"}, DynamicObservationValidationReason.EPISODE_ID_MISMATCH),
        ({"episode_seed": -1}, DynamicObservationValidationReason.EPISODE_SEED_MISMATCH),
        ({"map_id": "wrong"}, DynamicObservationValidationReason.MAP_ID_MISMATCH),
        ({"map_revision": 99}, DynamicObservationValidationReason.MAP_REVISION_MISMATCH),
    ),
)
def test_each_source_identity_mismatch_returns_a_structured_reason(change, reason) -> None:
    trace, source = _trace_and_source(79)
    frame = _delivered_frames(
        generate_dynamic_observation_slots(
            trace.ground_truth_frames,
            source=source,
            profile=BOUNDARY_300_OBSERVATION_PROFILE,
        )
    )[0]
    tampered = replace(frame, **change)
    tampered = _rehash(tampered)

    result = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE).accept(
        tampered,
        received_at_s=tampered.delivered_at_s,
    )
    assert not result.accepted
    assert reason in result.failures


def test_sequence_revision_hash_duplicate_and_binding_faults_are_transactional() -> None:
    trace, source = _trace_and_source(81)
    frames = _delivered_frames(
        generate_dynamic_observation_slots(
            trace.ground_truth_frames,
            source=source,
            profile=BOUNDARY_300_OBSERVATION_PROFILE,
        )
    )
    validator = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE)
    assert validator.accept(frames[1], received_at_s=frames[1].delivered_at_s).accepted

    duplicate_sequence = _rehash(replace(frames[2], sequence=frames[1].sequence))
    result = validator.accept(
        duplicate_sequence,
        received_at_s=duplicate_sequence.delivered_at_s,
    )
    assert DynamicObservationValidationReason.SEQUENCE_NOT_INCREASING in result.failures

    regressed_revision = _rehash(replace(frames[2], observation_revision=0))
    result = validator.accept(
        regressed_revision,
        received_at_s=regressed_revision.delivered_at_s,
    )
    assert DynamicObservationValidationReason.OBSERVATION_REVISION_REGRESSED in result.failures

    hash_tamper = replace(frames[2], content_hash="0" * 64)
    result = validator.accept(hash_tamper, received_at_s=hash_tamper.delivered_at_s)
    assert DynamicObservationValidationReason.CONTENT_HASH_MISMATCH in result.failures

    track = frames[2].tracks[0]
    duplicate_tracks = _rehash(replace(frames[2], tracks=(track, track)))
    result = validator.accept(
        duplicate_tracks,
        received_at_s=duplicate_tracks.delivered_at_s,
    )
    assert DynamicObservationValidationReason.DUPLICATE_TRACK_ID in result.failures

    rebound = replace(track, actor_binding_id="another_actor")
    binding_change = _rehash(replace(frames[2], tracks=(rebound,)))
    result = validator.accept(binding_change, received_at_s=binding_change.delivered_at_s)
    assert DynamicObservationValidationReason.ACTOR_BINDING_CHANGED in result.failures

    # Invalid candidates above must not advance state; the original next frame remains acceptable.
    assert validator.accept(frames[2], received_at_s=frames[2].delivered_at_s).accepted


def test_rehashed_uncertainty_smaller_than_the_active_profile_is_rejected() -> None:
    trace, source = _trace_and_source(82)
    frame = _delivered_frames(
        generate_dynamic_observation_slots(
            trace.ground_truth_frames,
            source=source,
            profile=NORMAL_OBSERVATION_PROFILE,
        )
    )[0]
    track = replace(frame.tracks[0], position_sigma_m=0.0, velocity_sigma_mps=0.0)
    understated = _rehash(replace(frame, tracks=(track,)))

    result = DynamicObservationValidator(source, NORMAL_OBSERVATION_PROFILE).accept(
        understated,
        received_at_s=understated.delivered_at_s,
    )

    assert not result.accepted
    assert (
        DynamicObservationValidationReason.UNCERTAINTY_PROFILE_MISMATCH
        in result.failures
    )


def test_frame_kind_nonfinite_and_time_faults_return_reasons_without_crashing() -> None:
    trace, source = _trace_and_source(83)
    frame = _delivered_frames(
        generate_dynamic_observation_slots(
            trace.ground_truth_frames,
            source=source,
            profile=BOUNDARY_300_OBSERVATION_PROFILE,
        )
    )[0]

    kind_fault = _rehash(replace(frame, frame_kind=DynamicObservationFrameKind.EMPTY))
    result = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE).accept(
        kind_fault,
        received_at_s=kind_fault.delivered_at_s,
    )
    assert DynamicObservationValidationReason.FRAME_KIND_TRACK_MISMATCH in result.failures

    point = _wire_clone(frame.tracks[0].observed_position, x=float("nan"))
    track = _wire_clone(frame.tracks[0], observed_position=point)
    nonfinite = _wire_clone(frame, tracks=(track,))
    result = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE).accept(
        nonfinite,
        received_at_s=nonfinite.delivered_at_s,
    )
    assert DynamicObservationValidationReason.NON_FINITE_TRACK in result.failures

    observation_after_delivery = _wire_clone(frame, delivered_at_s=-0.1)
    result = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE).accept(
        observation_after_delivery,
        received_at_s=0.0,
    )
    assert DynamicObservationValidationReason.NEGATIVE_TIMESTAMP in result.failures
    assert DynamicObservationValidationReason.OBSERVATION_AFTER_DELIVERY in result.failures

    delivery_in_future = _rehash(replace(frame, delivered_at_s=0.35))
    result = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE).accept(
        delivery_in_future,
        received_at_s=0.30,
    )
    assert DynamicObservationValidationReason.DELIVERY_IN_FUTURE in result.failures
    assert DynamicObservationValidationReason.LATENCY_MISMATCH in result.failures


def test_ttl_exactly_300ms_is_fresh() -> None:
    trace, source = _trace_and_source(85)
    frame_300 = _delivered_frames(
        generate_dynamic_observation_slots(
            trace.ground_truth_frames[:1],
            source=source,
            profile=BOUNDARY_300_OBSERVATION_PROFILE,
        )
    )[0]
    validator_300 = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE)
    assert validator_300.accept(frame_300, received_at_s=0.3).accepted
    assert validator_300.snapshot(
        control_time_s=0.3
    ).availability is DynamicObservationAvailability.FRESH


def test_ttl_any_later_nanosecond_is_stale() -> None:
    trace, source = _trace_and_source(85)
    frame_300 = _delivered_frames(
        generate_dynamic_observation_slots(
            trace.ground_truth_frames[:1],
            source=source,
            profile=BOUNDARY_300_OBSERVATION_PROFILE,
        )
    )[0]
    validator_300 = DynamicObservationValidator(source, BOUNDARY_300_OBSERVATION_PROFILE)
    assert validator_300.accept(frame_300, received_at_s=0.3).accepted
    assert validator_300.snapshot(
        control_time_s=0.3000000004
    ).availability is DynamicObservationAvailability.STALE
    assert validator_300.snapshot(
        control_time_s=0.300000001
    ).availability is DynamicObservationAvailability.STALE


def test_350ms_boundary_profile_arrives_stale() -> None:
    trace, source = _trace_and_source(85)
    frame_350 = _delivered_frames(
        generate_dynamic_observation_slots(
            trace.ground_truth_frames[:1],
            source=source,
            profile=BOUNDARY_350_OBSERVATION_PROFILE,
        )
    )[0]
    validator_350 = DynamicObservationValidator(source, BOUNDARY_350_OBSERVATION_PROFILE)
    assert validator_350.accept(frame_350, received_at_s=0.35).accepted
    stale = validator_350.snapshot(control_time_s=0.35)
    assert stale.availability is DynamicObservationAvailability.STALE
    assert DynamicObservationValidationReason.STALE in stale.failures


def test_snapshot_before_delivery_and_negative_time_return_precise_reasons() -> None:
    trace, source = _trace_and_source(86)
    frame = _delivered_frames(
        generate_dynamic_observation_slots(
            trace.ground_truth_frames[:1],
            source=source,
            profile=NORMAL_OBSERVATION_PROFILE,
        )
    )[0]
    validator = DynamicObservationValidator(source, NORMAL_OBSERVATION_PROFILE)
    assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted

    before_delivery = validator.snapshot(control_time_s=frame.delivered_at_s - 0.001)
    negative = validator.snapshot(control_time_s=-0.001)

    assert before_delivery.availability is DynamicObservationAvailability.INVALID
    assert before_delivery.failures == (
        DynamicObservationValidationReason.DELIVERY_IN_FUTURE,
    )
    assert negative.availability is DynamicObservationAvailability.INVALID
    assert negative.failures == (DynamicObservationValidationReason.NEGATIVE_TIMESTAMP,)


def test_controller_observation_contract_leaks_no_ground_truth_or_labels() -> None:
    frame_fields = {field.name for field in fields(DynamicObservationFrame)}
    track_fields = {field.name for field in fields(ActorTrack)}

    forbidden = {
        "actors",
        "ground_truth",
        "future_waypoints",
        "expectation_category",
        "scenario_label",
    }
    assert frame_fields.isdisjoint(forbidden)
    assert track_fields.isdisjoint(forbidden)
    assert not any(field.type is ActorState for field in fields(ActorTrack))
