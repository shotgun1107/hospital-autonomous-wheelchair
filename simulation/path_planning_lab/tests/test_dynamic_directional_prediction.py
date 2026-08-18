from __future__ import annotations

import json
from dataclasses import asdict, replace
from math import hypot

import pytest

from hospital_path_lab.dynamic_contracts import (
    ACTOR_RADIUS_M,
    ActorTrack,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    Point2D,
    Vector2D,
)
from hospital_path_lab.dynamic_corpus import (
    controller_episode_id,
    generate_dynamic_v6_public_corpus,
    generate_episode_observation_slots,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DIRECTIONAL_PREDICTION_VERSION,
    FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS,
    DirectionalActorPredictor,
    DirectionalPredictionParameters,
    DirectionalPredictionStatus,
    sample_directional_actor_circles,
    sample_directional_capsules,
    validate_directional_prediction_set,
)
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationAvailability,
    DynamicObservationProfile,
    DynamicObservationSnapshot,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    dynamic_observation_content_hash,
)
from hospital_path_lab.dynamic_prediction import ActorTubeCircle


def _source(*, episode_id: str = "directional-v7") -> DynamicObservationSourceIdentity:
    return DynamicObservationSourceIdentity(
        stream_id="directional-stream-v7",
        episode_id=episode_id,
        episode_seed=701,
        map_id="directional-map-v7",
        map_revision=7,
    )


def _frame(
    source: DynamicObservationSourceIdentity,
    profile: DynamicObservationProfile,
    sequence: int,
    *,
    velocity: Vector2D,
    empty: bool = False,
    actors: tuple[tuple[str, Point2D, Vector2D], ...] | None = None,
) -> DynamicObservationFrame:
    observed_at_s = sequence * profile.observation_period_s
    if actors is None:
        actors = (
            (
                "actor-001",
                Point2D(
                    1.0 + velocity.x * observed_at_s,
                    2.0 + velocity.y * observed_at_s,
                ),
                velocity,
            ),
        )
    tracks = () if empty else tuple(
        ActorTrack(
            track_id=actor_id,
            actor_binding_id=actor_id,
            observed_position=position,
            observed_velocity=actor_velocity,
            position_sigma_m=profile.position_sigma_m,
            velocity_sigma_mps=profile.velocity_sigma_mps,
        )
        for actor_id, position, actor_velocity in actors
    )
    frame = DynamicObservationFrame(
        stream_id=source.stream_id,
        episode_id=source.episode_id,
        episode_seed=source.episode_seed,
        map_id=source.map_id,
        map_revision=source.map_revision,
        observation_revision=sequence,
        sequence=sequence,
        observed_at_s=observed_at_s,
        delivered_at_s=observed_at_s + profile.latency_s,
        frame_kind=(
            DynamicObservationFrameKind.EMPTY
            if empty
            else DynamicObservationFrameKind.TRACKS
        ),
        tracks=tracks,
        content_hash="pending",
    )
    return replace(frame, content_hash=dynamic_observation_content_hash(frame))


def _accepted_snapshots(
    *,
    velocity: Vector2D,
    count: int = 20,
    profile: DynamicObservationProfile = NORMAL_OBSERVATION_PROFILE,
    source: DynamicObservationSourceIdentity | None = None,
) -> tuple[DynamicObservationSnapshot, ...]:
    selected_source = source or _source()
    validator = DynamicObservationValidator(selected_source, profile)
    snapshots: list[DynamicObservationSnapshot] = []
    for sequence in range(count):
        frame = _frame(selected_source, profile, sequence, velocity=velocity)
        accepted = validator.accept(frame, received_at_s=frame.delivered_at_s)
        assert accepted.accepted
        snapshots.append(validator.snapshot(control_time_s=frame.delivered_at_s))
    return tuple(snapshots)


def _ready_prediction(*, velocity: Vector2D | None = None):
    predictor = DirectionalActorPredictor()
    selected_velocity = velocity if velocity is not None else Vector2D(0.065, 0.0)
    result = None
    for snapshot in _accepted_snapshots(velocity=selected_velocity):
        result = predictor.update(snapshot)
    assert result is not None
    assert result.status is DirectionalPredictionStatus.READY
    assert result.prediction_set is not None
    return predictor, result


def test_frozen_v7_parameters_make_the_research_scope_explicit() -> None:
    parameters = FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS

    assert parameters.model_version == DIRECTIONAL_PREDICTION_VERSION
    assert parameters.history_frame_count == 20
    assert parameters.sigma_multiplier == 2.0
    assert parameters.lateral_turn_bound_m == 0.0

    with pytest.raises(ValueError, match="exactly 20"):
        DirectionalPredictionParameters(history_frame_count=19)
    with pytest.raises(ValueError, match="lateral turn"):
        DirectionalPredictionParameters(lateral_turn_bound_m=0.01)


def test_direction_locks_only_after_twenty_unique_accepted_tracks_frames() -> None:
    predictor = DirectionalActorPredictor()
    snapshots = _accepted_snapshots(velocity=Vector2D(0.065, 0.0))

    for index, snapshot in enumerate(snapshots[:-1], start=1):
        result = predictor.update(snapshot)
        assert result.status is DirectionalPredictionStatus.WARMING_UP
        assert result.hold_required
        assert result.prediction_set is None
        assert result.history_counts == (("actor-001", index),)

    result = predictor.update(snapshots[-1])
    assert result.status is DirectionalPredictionStatus.READY
    assert not result.hold_required
    assert result.prediction_set is not None
    tube = result.prediction_set.tubes[0]
    assert tube.history_count == 20
    assert tube.history_span_s == pytest.approx(1.9)
    assert tube.estimated_speed_mps == pytest.approx(0.065)
    assert tube.heading_unit == Vector2D(1.0, 0.0)


def test_duplicate_observation_is_idempotent_and_does_not_grow_history() -> None:
    predictor = DirectionalActorPredictor()
    snapshots = _accepted_snapshots(velocity=Vector2D(0.20, 0.0))
    for snapshot in snapshots:
        first = predictor.update(snapshot)

    duplicate = predictor.update(snapshots[-1])

    assert first.prediction_set == duplicate.prediction_set
    assert duplicate.status is DirectionalPredictionStatus.READY
    assert duplicate.duplicate_observation
    assert duplicate.history_counts == (("actor-001", 20),)


def test_capsule_uses_forward_only_braking_and_bounded_acceleration() -> None:
    _, result = _ready_prediction(velocity=Vector2D(0.20, 0.0))
    assert result.prediction_set is not None
    tube = result.prediction_set.tubes[0]
    capsule = sample_directional_capsules(
        result.prediction_set,
        rollout_time_s=2.0,
    )[0]
    parameters = FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS
    horizon_s = result.prediction_set.snapshot_age_s + 0.05 + 2.0
    expected_min_m = tube.estimated_speed_mps**2 / (
        2.0 * parameters.maximum_longitudinal_deceleration_mps2
    )
    expected_sigma_m = hypot(
        tube.position_sigma_m,
        horizon_s * tube.velocity_sigma_mps,
    )

    assert capsule.prediction_horizon_s == pytest.approx(horizon_s)
    assert capsule.longitudinal_min_m == pytest.approx(expected_min_m)
    assert capsule.longitudinal_min_m >= 0.0
    assert capsule.longitudinal_max_m <= parameters.maximum_speed_mps * horizon_s
    assert capsule.measurement_sigma_m == pytest.approx(expected_sigma_m)
    assert capsule.base_radius_m == pytest.approx(
        ACTOR_RADIUS_M + 2.0 * expected_sigma_m
    )
    assert capsule.start.x >= tube.anchor_position.x
    assert capsule.end.x >= capsule.start.x
    assert capsule.start.y == pytest.approx(tube.anchor_position.y)
    assert capsule.end.y == pytest.approx(tube.anchor_position.y)


def test_capsule_sampling_rejects_non_frozen_parameter_override() -> None:
    _, result = _ready_prediction(velocity=Vector2D(0.20, 0.0))
    assert result.prediction_set is not None
    alternate = replace(
        FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS,
        command_apply_latency_s=(
            FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS.command_apply_latency_s + 0.001
        ),
    )

    with pytest.raises(ValueError, match="requires frozen parameters"):
        sample_directional_capsules(
            result.prediction_set,
            rollout_time_s=1.0,
            parameters=alternate,
        )


def test_circle_chain_is_deterministic_and_conservatively_covers_capsule() -> None:
    _, result = _ready_prediction(velocity=Vector2D(0.20, 0.0))
    assert result.prediction_set is not None
    first = sample_directional_capsules(result.prediction_set, rollout_time_s=1.0)[0]
    second = sample_directional_capsules(result.prediction_set, rollout_time_s=1.0)[0]

    assert first == second
    assert all(isinstance(circle, ActorTubeCircle) for circle in first.circles)
    assert first.circles[0].center == first.start
    assert first.circles[-1].center == first.end
    assert first.covering_circle_radius_m >= first.base_radius_m >= ACTOR_RADIUS_M
    spacings = tuple(
        hypot(right.center.x - left.center.x, right.center.y - left.center.y)
        for left, right in zip(first.circles, first.circles[1:], strict=False)
    )
    assert all(
        spacing <= FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS.maximum_circle_spacing_m
        + 1e-12
        for spacing in spacings
    )
    assert sample_directional_actor_circles(
        result.prediction_set,
        rollout_time_s=1.0,
    ) == first.circles


def test_speed_inside_two_sigma_returns_no_prediction_and_an_explicit_hold() -> None:
    predictor = DirectionalActorPredictor()
    for snapshot in _accepted_snapshots(velocity=Vector2D(0.01, 0.0)):
        result = predictor.update(snapshot)

    assert result.status is DirectionalPredictionStatus.LOW_CONFIDENCE
    assert result.hold_required
    assert result.prediction_set is None
    assert result.reason_code == "low_confidence"


def test_zero_mean_speed_is_reported_as_low_speed() -> None:
    predictor = DirectionalActorPredictor()
    for snapshot in _accepted_snapshots(velocity=Vector2D(0.0, 0.0)):
        result = predictor.update(snapshot)

    assert result.status is DirectionalPredictionStatus.LOW_SPEED
    assert result.hold_required
    assert result.history_counts == (("actor-001", 20),)


def test_declared_stress_uncertainty_can_prevent_direction_lock() -> None:
    predictor = DirectionalActorPredictor()
    for snapshot in _accepted_snapshots(
        velocity=Vector2D(0.065, 0.0),
        profile=STRESS_OBSERVATION_PROFILE,
    ):
        result = predictor.update(snapshot)

    assert result.status is DirectionalPredictionStatus.LOW_CONFIDENCE
    assert result.hold_required
    assert result.prediction_set is None


def test_session_identity_change_resets_history_instead_of_mixing_actors() -> None:
    predictor, ready = _ready_prediction(velocity=Vector2D(0.20, 0.0))
    assert ready.status is DirectionalPredictionStatus.READY
    new_snapshot = _accepted_snapshots(
        velocity=Vector2D(0.20, 0.0),
        count=1,
        source=_source(episode_id="new-directional-session"),
    )[0]

    result = predictor.update(new_snapshot)

    assert result.status is DirectionalPredictionStatus.WARMING_UP
    assert result.session_reset
    assert result.history_counts == (("actor-001", 1),)


def test_regressed_frame_resets_history_and_requests_hold() -> None:
    predictor = DirectionalActorPredictor()
    snapshots = _accepted_snapshots(velocity=Vector2D(0.20, 0.0))
    for snapshot in snapshots:
        assert predictor.update(snapshot).status in {
            DirectionalPredictionStatus.WARMING_UP,
            DirectionalPredictionStatus.READY,
        }

    result = predictor.update(snapshots[-2])

    assert result.status is DirectionalPredictionStatus.ORDER_VIOLATION
    assert result.hold_required
    assert result.history_counts == ()


def test_fresh_empty_is_distinct_from_dropout_and_does_not_request_hold() -> None:
    source = _source()
    validator = DynamicObservationValidator(source, NORMAL_OBSERVATION_PROFILE)
    frame = _frame(
        source,
        NORMAL_OBSERVATION_PROFILE,
        0,
        velocity=Vector2D(0.0, 0.0),
        empty=True,
    )
    assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted

    result = DirectionalActorPredictor().update(
        validator.snapshot(control_time_s=frame.delivered_at_s)
    )

    assert result.status is DirectionalPredictionStatus.EMPTY_FRAME
    assert not result.hold_required
    assert result.prediction_set is not None
    assert result.prediction_set.tubes == ()


def test_single_frame_dropout_returns_explicit_hold_without_reusing_prediction() -> None:
    source = _source()
    validator = DynamicObservationValidator(source, NORMAL_OBSERVATION_PROFILE)
    first = _frame(
        source,
        NORMAL_OBSERVATION_PROFILE,
        0,
        velocity=Vector2D(0.20, 0.0),
    )
    assert validator.accept(first, received_at_s=first.delivered_at_s).accepted
    predictor = DirectionalActorPredictor()
    assert predictor.update(
        validator.snapshot(control_time_s=first.delivered_at_s)
    ).status is DirectionalPredictionStatus.WARMING_UP
    validator.record_no_frame(sequence=1, delivery_time_s=0.2)

    result = predictor.update(validator.snapshot(control_time_s=0.2))

    assert result.status is DirectionalPredictionStatus.DROPOUT
    assert result.hold_required
    assert result.prediction_set is None
    assert result.history_counts == (("actor-001", 1),)


def test_single_frame_dropout_reuses_locked_direction_only_until_ttl() -> None:
    source = _source()
    validator = DynamicObservationValidator(source, NORMAL_OBSERVATION_PROFILE)
    predictor = DirectionalActorPredictor()
    result = None
    for sequence in range(20):
        frame = _frame(
            source,
            NORMAL_OBSERVATION_PROFILE,
            sequence,
            velocity=Vector2D(0.20, 0.0),
        )
        assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted
        result = predictor.update(
            validator.snapshot(control_time_s=frame.delivered_at_s)
        )
    assert result is not None
    assert result.status is DirectionalPredictionStatus.READY
    original_history = result.prediction_set.history_content_hash

    validator.record_no_frame(sequence=20, delivery_time_s=2.1)
    held = predictor.update(validator.snapshot(control_time_s=2.1))

    assert held.status is DirectionalPredictionStatus.READY
    assert held.reason_code == "ttl_holdover"
    assert held.duplicate_observation
    assert held.history_counts == (("actor-001", 20),)
    assert held.prediction_set is not None
    assert held.prediction_set.history_content_hash == original_history
    assert held.prediction_set.snapshot_age_s == pytest.approx(0.2)
    assert held.prediction_set.controller_time_s == pytest.approx(2.1)

    validator.record_no_frame(sequence=21, delivery_time_s=2.2)
    at_ttl = predictor.update(validator.snapshot(control_time_s=2.2))
    assert at_ttl.status is DirectionalPredictionStatus.READY
    assert at_ttl.prediction_set is not None
    assert at_ttl.prediction_set.snapshot_age_s == pytest.approx(0.3)

    validator.record_no_frame(sequence=22, delivery_time_s=2.3)
    stale = predictor.update(validator.snapshot(control_time_s=2.3))
    assert stale.status is DirectionalPredictionStatus.STALE
    assert stale.hold_required
    assert stale.prediction_set is None
    assert stale.history_counts == ()


def test_stale_invalid_and_unavailable_states_are_explicit_holds() -> None:
    source = _source()
    validator = DynamicObservationValidator(source, NORMAL_OBSERVATION_PROFILE)
    frame = _frame(
        source,
        NORMAL_OBSERVATION_PROFILE,
        0,
        velocity=Vector2D(0.20, 0.0),
    )
    assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted
    predictor = DirectionalActorPredictor()

    stale = predictor.update(validator.snapshot(control_time_s=0.300001))
    invalid = predictor.update(
        DynamicObservationSnapshot(
            availability=DynamicObservationAvailability.INVALID,
            frame=frame,
            age_s=None,
            failures=(),
            last_event_was_no_frame=False,
        )
    )
    unavailable = predictor.update(
        DynamicObservationSnapshot(
            availability=DynamicObservationAvailability.UNAVAILABLE,
            frame=None,
            age_s=None,
            failures=(),
            last_event_was_no_frame=False,
        )
    )

    assert stale.status is DirectionalPredictionStatus.STALE
    assert invalid.status is DirectionalPredictionStatus.INVALID
    assert unavailable.status is DirectionalPredictionStatus.UNAVAILABLE
    assert all(result.hold_required for result in (stale, invalid, unavailable))


def test_forged_frame_content_hash_resets_history_and_holds() -> None:
    predictor = DirectionalActorPredictor()
    snapshot = _accepted_snapshots(velocity=Vector2D(0.20, 0.0), count=1)[0]
    assert snapshot.frame is not None
    forged = replace(
        snapshot,
        frame=replace(snapshot.frame, content_hash="forged-content"),
    )

    result = predictor.update(forged)

    assert result.status is DirectionalPredictionStatus.INVALID
    assert result.hold_required
    assert result.reason_code == "observation_content_hash_mismatch"
    assert result.history_counts == ()


def test_stale_discards_history_and_requires_twenty_new_frames() -> None:
    predictor = DirectionalActorPredictor()
    snapshots = _accepted_snapshots(
        velocity=Vector2D(0.20, 0.0),
        count=20,
    )
    for snapshot in snapshots[:19]:
        assert predictor.update(snapshot).status is DirectionalPredictionStatus.WARMING_UP

    stale = replace(
        snapshots[18],
        availability=DynamicObservationAvailability.STALE,
        age_s=0.300001,
    )
    assert predictor.update(stale).status is DirectionalPredictionStatus.STALE
    result = predictor.update(snapshots[19])

    assert result.status is DirectionalPredictionStatus.WARMING_UP
    assert result.history_counts == (("actor-001", 1),)


def test_track_identity_change_resets_only_that_binding_history() -> None:
    source = _source()
    profile = NORMAL_OBSERVATION_PROFILE
    validator = DynamicObservationValidator(source, profile)
    predictor = DirectionalActorPredictor()
    for sequence in range(19):
        frame = _frame(source, profile, sequence, velocity=Vector2D(0.20, 0.0))
        assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted
        predictor.update(validator.snapshot(control_time_s=frame.delivered_at_s))

    time_s = 19 * profile.observation_period_s
    changed = _frame(
        source,
        profile,
        19,
        velocity=Vector2D(0.20, 0.0),
        actors=(("actor-001", Point2D(1.0 + 0.20 * time_s, 2.0), Vector2D(0.20, 0.0)),),
    )
    changed_track = replace(changed.tracks[0], track_id="replacement-track")
    changed = replace(changed, tracks=(changed_track,), content_hash="pending")
    changed = replace(changed, content_hash=dynamic_observation_content_hash(changed))
    assert validator.accept(changed, received_at_s=changed.delivered_at_s).accepted
    result = predictor.update(validator.snapshot(control_time_s=changed.delivered_at_s))

    assert result.status is DirectionalPredictionStatus.WARMING_UP
    assert result.history_counts == (("actor-001", 1),)


def test_prediction_capability_and_semantic_commitments_fail_closed() -> None:
    _, result = _ready_prediction(velocity=Vector2D(0.20, 0.0))
    assert result.prediction_set is not None
    prediction = result.prediction_set
    validate_directional_prediction_set(prediction)

    copied = replace(prediction)
    with pytest.raises(ValueError, match="not issued"):
        validate_directional_prediction_set(copied)

    original_hash = prediction.history_content_hash
    object.__setattr__(prediction, "history_content_hash", "tampered-history")
    try:
        with pytest.raises(ValueError, match="commitment"):
            validate_directional_prediction_set(prediction)
        with pytest.raises(ValueError, match="commitment"):
            sample_directional_capsules(prediction, rollout_time_s=0.0)
    finally:
        object.__setattr__(prediction, "history_content_hash", original_hash)


def test_velocity_mean_and_latest_position_are_the_direction_contract() -> None:
    source = _source()
    profile = NORMAL_OBSERVATION_PROFILE
    validator = DynamicObservationValidator(source, profile)
    predictor = DirectionalActorPredictor()
    velocities = tuple(
        Vector2D(0.18 if index % 2 == 0 else 0.22, 0.02)
        for index in range(20)
    )
    latest_position = None
    for sequence, velocity in enumerate(velocities):
        position = Point2D(1.0 + 0.20 * sequence * 0.1, 2.0)
        latest_position = position
        frame = _frame(
            source,
            profile,
            sequence,
            velocity=velocity,
            actors=(("actor-001", position, velocity),),
        )
        assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted
        result = predictor.update(validator.snapshot(control_time_s=frame.delivered_at_s))

    assert result.status is DirectionalPredictionStatus.READY
    assert result.prediction_set is not None
    tube = result.prediction_set.tubes[0]
    assert tube.anchor_position == latest_position
    assert tube.estimated_speed_mps == pytest.approx(hypot(0.20, 0.02))
    assert tube.heading_unit.x == pytest.approx(0.20 / hypot(0.20, 0.02))
    assert tube.heading_unit.y == pytest.approx(0.02 / hypot(0.20, 0.02))
    assert tube.position_sigma_m == profile.position_sigma_m
    assert tube.velocity_sigma_mps == pytest.approx(
        profile.velocity_sigma_mps / (20**0.5)
    )


def test_abrupt_position_turn_breaks_constant_heading_fit() -> None:
    source = _source()
    profile = NORMAL_OBSERVATION_PROFILE
    validator = DynamicObservationValidator(source, profile)
    predictor = DirectionalActorPredictor()
    for sequence in range(20):
        time_s = sequence * profile.observation_period_s
        if sequence < 10:
            position = Point2D(1.0 + 0.20 * time_s, 2.0)
            velocity = Vector2D(0.20, 0.0)
        else:
            position = Point2D(1.18, 2.0 + 0.50 * (time_s - 0.9))
            velocity = Vector2D(0.0, 0.50)
        frame = _frame(
            source,
            profile,
            sequence,
            velocity=velocity,
            actors=(("actor-001", position, velocity),),
        )
        assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted
        result = predictor.update(validator.snapshot(control_time_s=frame.delivered_at_s))

    assert result.status is DirectionalPredictionStatus.LOW_CONFIDENCE
    assert result.hold_required
    assert result.history_counts == (("actor-001", 20),)


def test_multiple_actor_result_is_sorted_and_waits_for_every_binding() -> None:
    source = _source()
    profile = NORMAL_OBSERVATION_PROFILE
    validator = DynamicObservationValidator(source, profile)
    predictor = DirectionalActorPredictor()
    for sequence in range(20):
        time_s = sequence * profile.observation_period_s
        actors = (
            ("z-actor", Point2D(1.0, 1.0 + 0.20 * time_s), Vector2D(0.0, 0.20)),
            ("a-actor", Point2D(1.0 + 0.20 * time_s, 2.0), Vector2D(0.20, 0.0)),
        )
        frame = _frame(
            source,
            profile,
            sequence,
            velocity=Vector2D(0.0, 0.0),
            actors=actors,
        )
        assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted
        result = predictor.update(validator.snapshot(control_time_s=frame.delivered_at_s))

    assert result.status is DirectionalPredictionStatus.READY
    assert result.prediction_set is not None
    assert tuple(tube.actor_binding_id for tube in result.prediction_set.tubes) == (
        "a-actor",
        "z-actor",
    )


def test_prediction_and_circle_output_are_byte_deterministic() -> None:
    _, first_result = _ready_prediction(velocity=Vector2D(0.20, 0.0))
    _, second_result = _ready_prediction(velocity=Vector2D(0.20, 0.0))
    assert first_result.prediction_set is not None
    assert second_result.prediction_set is not None
    first_circles = sample_directional_actor_circles(
        first_result.prediction_set,
        rollout_time_s=1.0,
    )
    second_circles = sample_directional_actor_circles(
        second_result.prediction_set,
        rollout_time_s=1.0,
    )

    def encoded(value: object) -> bytes:
        return json.dumps(
            asdict(value),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode()

    assert encoded(first_result.prediction_set) == encoded(second_result.prediction_set)
    assert tuple(encoded(circle) for circle in first_circles) == tuple(
        encoded(circle) for circle in second_circles
    )


def test_normal_public_same_direction_cases_lock_without_hidden_data() -> None:
    episodes = tuple(
        episode
        for episode in generate_dynamic_v6_public_corpus()
        if episode.latent_case_id.startswith("same-direction-wide")
    )
    assert len(episodes) == 5

    for episode in episodes:
        source = DynamicObservationSourceIdentity(
            stream_id="dynamic-stage5-stream",
            episode_id=controller_episode_id(episode),
            episode_seed=episode.seed,
            map_id=episode.map_id,
            map_revision=1,
        )
        validator = DynamicObservationValidator(source, NORMAL_OBSERVATION_PROFILE)
        predictor = DirectionalActorPredictor()
        ready_sequences: list[int] = []
        for slot in generate_episode_observation_slots(
            episode,
            profile=NORMAL_OBSERVATION_PROFILE,
        ):
            if slot.frame is None:
                validator.record_no_frame(
                    sequence=slot.sequence,
                    delivery_time_s=slot.scheduled_delivery_at_s,
                )
            else:
                accepted = validator.accept(
                    slot.frame,
                    received_at_s=slot.scheduled_delivery_at_s,
                )
                assert accepted.accepted
            result = predictor.update(
                validator.snapshot(control_time_s=slot.scheduled_delivery_at_s)
            )
            if result.status is DirectionalPredictionStatus.READY:
                ready_sequences.append(slot.sequence)

        assert ready_sequences
        assert min(ready_sequences) >= 19


def test_stress_public_same_direction_cases_never_lock_direction() -> None:
    episodes = tuple(
        episode
        for episode in generate_dynamic_v6_public_corpus()
        if episode.latent_case_id.startswith("same-direction-wide")
    )
    assert len(episodes) == 5

    for episode in episodes:
        source = DynamicObservationSourceIdentity(
            stream_id="dynamic-stage5-stream",
            episode_id=controller_episode_id(episode),
            episode_seed=episode.seed,
            map_id=episode.map_id,
            map_revision=1,
        )
        validator = DynamicObservationValidator(source, STRESS_OBSERVATION_PROFILE)
        predictor = DirectionalActorPredictor()
        ready_sequences: list[int] = []
        for slot in generate_episode_observation_slots(
            episode,
            profile=STRESS_OBSERVATION_PROFILE,
        ):
            if slot.frame is None:
                validator.record_no_frame(
                    sequence=slot.sequence,
                    delivery_time_s=slot.scheduled_delivery_at_s,
                )
            else:
                accepted = validator.accept(
                    slot.frame,
                    received_at_s=slot.scheduled_delivery_at_s,
                )
                assert accepted.accepted
            result = predictor.update(
                validator.snapshot(control_time_s=slot.scheduled_delivery_at_s)
            )
            if result.status is DirectionalPredictionStatus.READY:
                ready_sequences.append(slot.sequence)

        assert ready_sequences == []
