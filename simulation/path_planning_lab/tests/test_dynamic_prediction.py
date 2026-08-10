from __future__ import annotations

import json
from dataclasses import asdict, fields, replace
from math import isfinite

import pytest

from hospital_path_lab.dynamic_contracts import (
    ACTOR_RADIUS_M,
    ActorTrack,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    Point2D,
    Vector2D,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationProfile,
    DynamicObservationProfileName,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    dynamic_observation_content_hash,
)
from hospital_path_lab.dynamic_prediction import (
    ActorPredictionSet,
    ActorPredictionTube,
    ActorTubeCircle,
    build_actor_prediction_set,
    sample_actor_tubes,
)


def _frame(
    *,
    velocity: Vector2D | None = None,
    position_sigma_m: float = 0.03,
    velocity_sigma_mps: float = 0.05,
    observed_at_s: float = 0.0,
    delivered_at_s: float = 0.1,
    empty: bool = False,
) -> DynamicObservationFrame:
    selected_velocity = velocity if velocity is not None else Vector2D(0.3, 0.4)
    tracks = () if empty else (
        ActorTrack(
            track_id="track_001",
            actor_binding_id="actor_001",
            observed_position=Point2D(1.0, 2.0),
            observed_velocity=selected_velocity,
            position_sigma_m=position_sigma_m,
            velocity_sigma_mps=velocity_sigma_mps,
        ),
    )
    frame = DynamicObservationFrame(
        stream_id="observation_stream_v1",
        episode_id="corridor_crossing_v1",
        episode_seed=20260810,
        map_id="hospital_floor_v1",
        map_revision=1,
        observation_revision=1,
        sequence=7,
        observed_at_s=observed_at_s,
        delivered_at_s=delivered_at_s,
        frame_kind=(
            DynamicObservationFrameKind.EMPTY
            if empty
            else DynamicObservationFrameKind.TRACKS
        ),
        tracks=tracks,
        content_hash="pending",
    )
    return replace(frame, content_hash=dynamic_observation_content_hash(frame))


def _validated_snapshot(
    frame: DynamicObservationFrame,
    *,
    control_time_s: float,
):
    validator = _validator_for_frame(frame)
    assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted
    return validator.snapshot(control_time_s=control_time_s)


def _validator_for_frame(frame: DynamicObservationFrame) -> DynamicObservationValidator:
    position_sigma_m = frame.tracks[0].position_sigma_m if frame.tracks else 0.03
    velocity_sigma_mps = frame.tracks[0].velocity_sigma_mps if frame.tracks else 0.05
    profile = DynamicObservationProfile(
        name=DynamicObservationProfileName.NORMAL,
        latency_s=frame.delivered_at_s - frame.observed_at_s,
        ttl_s=0.3,
        position_sigma_m=position_sigma_m,
        velocity_sigma_mps=velocity_sigma_mps,
        dropout_probability=0.0,
    )
    source = DynamicObservationSourceIdentity(
        stream_id=frame.stream_id,
        episode_id=frame.episode_id,
        episode_seed=frame.episode_seed,
        map_id=frame.map_id,
        map_revision=frame.map_revision,
    )
    return DynamicObservationValidator(source, profile)


def test_actor_tube_matches_independent_center_sigma_and_acceleration_oracle() -> None:
    frame = _frame()
    prediction_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.1)
    )
    sample = sample_actor_tubes(prediction_set, rollout_time_s=0.2)[0]

    expected_tau = 0.1 + 0.05 + 0.2
    expected_center_x = 1.0 + 0.3 * expected_tau
    expected_center_y = 2.0 + 0.4 * expected_tau
    expected_sigma = (0.03**2 + (expected_tau * 0.05) ** 2) ** 0.5
    expected_acceleration_bound = 0.5 * 0.50 * expected_tau**2
    expected_radius = ACTOR_RADIUS_M + 2.0 * expected_sigma + expected_acceleration_bound

    assert sample.prediction_horizon_s == pytest.approx(0.35)
    assert sample.center.x == pytest.approx(expected_center_x)
    assert sample.center.y == pytest.approx(expected_center_y)
    assert sample.position_sigma_m == pytest.approx(expected_sigma)
    assert sample.acceleration_bound_m == pytest.approx(expected_acceleration_bound)
    assert sample.radius_m == pytest.approx(expected_radius)
    assert sample.radius_m == pytest.approx(0.280087219947249)


def test_speed_cap_preserves_vector_direction_instead_of_clamping_each_axis() -> None:
    frame = _frame(velocity=Vector2D(0.5, 0.5))
    prediction_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.1)
    )
    capped = prediction_set.tubes[0].capped_velocity

    assert capped.x == pytest.approx(0.353553390593274)
    assert capped.y == pytest.approx(0.353553390593274)
    assert capped.magnitude == pytest.approx(0.50)
    assert capped.x / capped.y == pytest.approx(1.0)


def test_speed_cap_preserves_direction_for_extreme_finite_components() -> None:
    frame = _frame(velocity=Vector2D(1e308, -1e308))
    prediction_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.1)
    )
    capped = prediction_set.tubes[0].capped_velocity

    assert capped.x == pytest.approx(0.353553390593274)
    assert capped.y == pytest.approx(-0.353553390593274)
    assert capped.magnitude == pytest.approx(0.50)


def test_zero_velocity_is_not_divided_during_speed_cap() -> None:
    frame = _frame(velocity=Vector2D(0.0, 0.0))
    prediction_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.1)
    )

    assert prediction_set.tubes[0].capped_velocity == Vector2D(0.0, 0.0)
    assert all(
        isfinite(value)
        for sample in sample_actor_tubes(prediction_set, rollout_time_s=1.0)
        for value in (sample.center.x, sample.center.y, sample.radius_m)
    )


def test_piecewise_acceleration_bound_is_continuous_at_velocity_delta_time() -> None:
    frame = _frame(observed_at_s=0.1, delivered_at_s=0.4)
    prediction_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.4)
    )

    at_boundary = sample_actor_tubes(prediction_set, rollout_time_s=1.65)[0]
    after_boundary = sample_actor_tubes(prediction_set, rollout_time_s=2.15)[0]
    just_before = sample_actor_tubes(prediction_set, rollout_time_s=1.65 - 1e-9)[0]
    just_after = sample_actor_tubes(prediction_set, rollout_time_s=1.65 + 1e-9)[0]

    assert at_boundary.prediction_horizon_s == pytest.approx(2.0)
    assert at_boundary.acceleration_bound_m == pytest.approx(1.0)
    assert after_boundary.prediction_horizon_s == pytest.approx(2.5)
    assert after_boundary.acceleration_bound_m == pytest.approx(1.5)
    assert just_before.acceleration_bound_m <= at_boundary.acceleration_bound_m
    assert just_after.acceleration_bound_m >= at_boundary.acceleration_bound_m
    assert just_before.acceleration_bound_m == pytest.approx(
        just_after.acceleration_bound_m,
        abs=3e-9,
    )


@pytest.mark.parametrize(
    ("position_sigma_m", "velocity_sigma_mps", "controller_time_s"),
    ((0.03, 0.05, 0.1), (0.08, 0.15, 0.25), (0.0, 0.0, 0.3)),
)
def test_radius_is_finite_and_monotonic_for_all_frozen_profiles(
    position_sigma_m: float,
    velocity_sigma_mps: float,
    controller_time_s: float,
) -> None:
    frame = _frame(
        position_sigma_m=position_sigma_m,
        velocity_sigma_mps=velocity_sigma_mps,
        delivered_at_s=controller_time_s,
    )
    prediction_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=controller_time_s)
    )
    samples = tuple(
        sample_actor_tubes(prediction_set, rollout_time_s=rollout)[0]
        for rollout in (0.0, 0.05, 0.2, 0.5, 1.0, 2.0, 3.0)
    )
    radii = tuple(sample.radius_m for sample in samples)

    assert all(
        isfinite(value)
        for sample in samples
        for value in (
            sample.rollout_time_s,
            sample.prediction_horizon_s,
            sample.center.x,
            sample.center.y,
            sample.radius_m,
            sample.position_sigma_m,
            sample.acceleration_bound_m,
        )
    )
    assert all(
        current <= following
        for current, following in zip(radii, radii[1:], strict=False)
    )


def test_ttl_exact_float_boundary_is_fresh() -> None:
    boundary = _frame(observed_at_s=0.1, delivered_at_s=0.4)

    prediction_set = build_actor_prediction_set(
        _validated_snapshot(boundary, control_time_s=0.4)
    )

    assert prediction_set.snapshot_age_s == 0.3


def test_ttl_value_above_boundary_is_rejected_before_prediction() -> None:
    boundary = _frame(observed_at_s=0.1, delivered_at_s=0.4)
    stale_snapshot = _validated_snapshot(boundary, control_time_s=0.400001)

    with pytest.raises(ValueError, match="fresh validated"):
        build_actor_prediction_set(stale_snapshot)


@pytest.mark.parametrize("controller_time_s", (-1.0, float("nan"), float("inf")))
def test_invalid_controller_times_are_rejected(controller_time_s: float) -> None:
    snapshot = _validated_snapshot(_frame(), control_time_s=controller_time_s)
    with pytest.raises(ValueError):
        build_actor_prediction_set(snapshot)


def test_prediction_set_rejects_an_age_that_disagrees_with_its_timestamps() -> None:
    frame = _frame()
    prediction_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.1)
    )

    with pytest.raises(ValueError, match="must match"):
        replace(prediction_set, snapshot_age_s=0.2)


def test_raw_frame_and_not_yet_delivered_snapshot_are_rejected() -> None:
    with pytest.raises(TypeError, match="validated observation snapshot"):
        build_actor_prediction_set(_frame())  # type: ignore[arg-type]

    not_yet_delivered = _frame(observed_at_s=0.0, delivered_at_s=0.2)
    snapshot = _validated_snapshot(not_yet_delivered, control_time_s=0.1)
    with pytest.raises(ValueError, match="fresh validated"):
        build_actor_prediction_set(snapshot)


def test_hash_invalid_snapshot_cannot_reach_prediction() -> None:
    invalid_hash_frame = replace(_frame(), content_hash="0" * 64)
    validator = _validator_for_frame(invalid_hash_frame)
    result = validator.accept(
        invalid_hash_frame,
        received_at_s=invalid_hash_frame.delivered_at_s,
    )
    assert not result.accepted

    snapshot = validator.snapshot(control_time_s=invalid_hash_frame.delivered_at_s)
    with pytest.raises(ValueError, match="fresh validated"):
        build_actor_prediction_set(snapshot)


@pytest.mark.parametrize("rollout_time_s", (-0.001, float("nan"), float("inf")))
def test_invalid_rollout_times_are_rejected(rollout_time_s: float) -> None:
    frame = _frame()
    prediction_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.1)
    )

    with pytest.raises(ValueError):
        sample_actor_tubes(prediction_set, rollout_time_s=rollout_time_s)


def test_finite_inputs_that_overflow_the_formula_are_rejected() -> None:
    frame = _frame(
        velocity=Vector2D(0.5, 0.0),
        position_sigma_m=0.0,
        velocity_sigma_mps=1e308,
    )
    prediction_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.1)
    )

    with pytest.raises(ValueError, match="non-finite"):
        sample_actor_tubes(prediction_set, rollout_time_s=1e308)


def test_fresh_empty_frame_produces_an_empty_prediction_set_and_sample() -> None:
    frame = _frame(empty=True)
    prediction_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.1)
    )

    assert prediction_set.tubes == ()
    assert sample_actor_tubes(prediction_set, rollout_time_s=0.0) == ()


def test_same_frame_and_query_have_byte_equivalent_canonical_json() -> None:
    frame = _frame()
    first_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.1)
    )
    second_set = build_actor_prediction_set(
        _validated_snapshot(frame, control_time_s=0.1)
    )
    first = sample_actor_tubes(first_set, rollout_time_s=0.2)
    second = sample_actor_tubes(second_set, rollout_time_s=0.2)

    def encoded(samples: tuple[ActorTubeCircle, ...]) -> bytes:
        return json.dumps(
            [asdict(sample) for sample in samples],
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")

    assert first == second
    assert encoded(first) == encoded(second)


def test_prediction_contracts_do_not_expose_ground_truth_or_expectation_fields() -> None:
    forbidden_fragments = ("ground_truth", "waypoint", "expectation", "scenario_label")

    for contract in (ActorPredictionTube, ActorPredictionSet, ActorTubeCircle):
        names = tuple(field.name for field in fields(contract))
        assert not any(fragment in name for name in names for fragment in forbidden_fragments)
