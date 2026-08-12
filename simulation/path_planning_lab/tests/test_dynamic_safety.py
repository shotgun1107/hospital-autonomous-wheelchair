from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from hospital_path_lab.contracts import (
    GridSnapshot,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    TrajectoryPoint,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DYNAMIC_OBSERVATION_PERIOD_S,
    ActorTrack,
    DynamicHoldReason,
    DynamicMotionState,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    Point2D,
    Vector2D,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalActorPredictor,
    DirectionalPredictionSet,
    DirectionalPredictionStatus,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
    dynamic_observation_content_hash,
)
from hospital_path_lab.dynamic_prediction import ActorPredictionSet, ActorPredictionTube
from hospital_path_lab.dynamic_safety import (
    DynamicSafetyContext,
    DynamicSafetyGate,
    _normalized_rollout,
    build_dynamic_command_proposal,
    evaluate_dynamic_trajectory_safety,
)
from hospital_path_lab.grid import GridMap
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _grid_snapshot(
    *,
    sequence: int = 0,
    occupied_cells: tuple[tuple[int, int], ...] = (),
    forbidden_cells: frozenset[tuple[int, int]] = frozenset(),
) -> GridSnapshot:
    occupancy = np.zeros((200, 200), dtype=np.bool_)
    for x, y in occupied_cells:
        occupancy[y, x] = True
    return GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="map_v1",
            map_revision=1,
            mission_revision=1,
            observation_revision=sequence,
            seed=1,
            content_hash=f"grid-{sequence}",
        ),
        grid=GridMap(occupancy=occupancy, resolution_m=0.02),
        forbidden_cells=forbidden_cells,
    )


def _fresh_observation(
    *,
    sequence: int,
    simulation_time_s: float,
    actor_position: Point2D | None = None,
) -> tuple[DynamicObservationSnapshot, ActorPredictionSet]:
    tracks = ()
    tubes = ()
    if actor_position is not None:
        tracks = (
            ActorTrack(
                track_id="track-1",
                actor_binding_id="actor-1",
                observed_position=actor_position,
                observed_velocity=Vector2D(0.0, 0.0),
                position_sigma_m=0.0,
                velocity_sigma_mps=0.0,
            ),
        )
        tubes = (
            ActorPredictionTube(
                track_id="track-1",
                actor_binding_id="actor-1",
                observed_position=actor_position,
                capped_velocity=Vector2D(0.0, 0.0),
                position_sigma_m=0.0,
                velocity_sigma_mps=0.0,
            ),
        )
    content_hash = f"frame-{sequence}"
    frame = DynamicObservationFrame(
        stream_id="stream-v1",
        episode_id="episode-v1",
        episode_seed=1,
        map_id="map_v1",
        map_revision=1,
        observation_revision=sequence,
        sequence=sequence,
        observed_at_s=simulation_time_s,
        delivered_at_s=simulation_time_s,
        frame_kind=(
            DynamicObservationFrameKind.TRACKS
            if tracks
            else DynamicObservationFrameKind.EMPTY
        ),
        tracks=tracks,
        content_hash=content_hash,
    )
    snapshot = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.FRESH,
        frame=frame,
        age_s=0.0,
        failures=(),
        last_event_was_no_frame=False,
    )
    prediction = ActorPredictionSet(
        stream_id=frame.stream_id,
        episode_id=frame.episode_id,
        map_id=frame.map_id,
        map_revision=frame.map_revision,
        observation_revision=frame.observation_revision,
        sequence=frame.sequence,
        source_content_hash=frame.content_hash,
        observed_at_s=simulation_time_s,
        controller_time_s=simulation_time_s,
        snapshot_age_s=0.0,
        tubes=tubes,
    )
    return snapshot, prediction


def _context(
    *,
    tick_id: int,
    observation: DynamicObservationSnapshot,
    prediction: ActorPredictionSet | DirectionalPredictionSet | None,
    grid_snapshot: GridSnapshot | None = None,
) -> DynamicSafetyContext:
    return DynamicSafetyContext(
        tick_id=tick_id,
        simulation_time_s=tick_id * 0.05,
        mission_id="mission-v1",
        authorization_revision=1,
        grid_snapshot=grid_snapshot or _grid_snapshot(sequence=tick_id),
        observation_snapshot=observation,
        prediction_set=prediction,
        path_still_valid=True,
        local_safety_recheck_passed=True,
        observation_safe=True,
    )


def _fresh_directional_observation(
    *,
    sequence: int,
    simulation_time_s: float,
    actor_position: Point2D,
    heading_unit: Vector2D | None = None,
    estimated_speed_mps: float = 0.04,
) -> tuple[DynamicObservationSnapshot, DirectionalPredictionSet]:
    if heading_unit is None:
        heading_unit = Vector2D(1.0, 0.0)
    velocity = Vector2D(
        heading_unit.x * estimated_speed_mps,
        heading_unit.y * estimated_speed_mps,
    )
    predictor = DirectionalActorPredictor()
    snapshot = None
    result = None
    history_count = predictor.parameters.history_frame_count
    for history_index in range(history_count):
        frame_sequence = sequence + history_index
        observed_at_s = (
            simulation_time_s + history_index * DYNAMIC_OBSERVATION_PERIOD_S
        )
        remaining_history_s = (
            history_count - 1 - history_index
        ) * DYNAMIC_OBSERVATION_PERIOD_S
        track = ActorTrack(
            track_id="track-1",
            actor_binding_id="actor-1",
            observed_position=Point2D(
                actor_position.x - velocity.x * remaining_history_s,
                actor_position.y - velocity.y * remaining_history_s,
            ),
            observed_velocity=velocity,
            position_sigma_m=0.0,
            velocity_sigma_mps=0.0,
        )
        frame = DynamicObservationFrame(
            stream_id="stream-v1",
            episode_id="episode-v1",
            episode_seed=1,
            map_id="map_v1",
            map_revision=1,
            observation_revision=frame_sequence,
            sequence=frame_sequence,
            observed_at_s=observed_at_s,
            delivered_at_s=observed_at_s,
            frame_kind=DynamicObservationFrameKind.TRACKS,
            tracks=(track,),
            content_hash="pending",
        )
        frame = replace(frame, content_hash=dynamic_observation_content_hash(frame))
        snapshot = DynamicObservationSnapshot(
            availability=DynamicObservationAvailability.FRESH,
            frame=frame,
            age_s=0.0,
            failures=(),
            last_event_was_no_frame=False,
        )
        result = predictor.update(snapshot)

    assert snapshot is not None
    assert result is not None
    assert result.status is DirectionalPredictionStatus.READY
    assert result.prediction_set is not None
    return snapshot, result.prediction_set


def _directional_context(
    observation: DynamicObservationSnapshot,
    prediction: DirectionalPredictionSet,
) -> DynamicSafetyContext:
    tick_id = round(prediction.controller_time_s / DYNAMIC_CONTROL_PERIOD_S)
    assert tick_id * DYNAMIC_CONTROL_PERIOD_S == pytest.approx(
        prediction.controller_time_s
    )
    return _context(
        tick_id=tick_id,
        observation=observation,
        prediction=prediction,
        grid_snapshot=_grid_snapshot(sequence=prediction.observation_revision),
    )


def _proposal(
    context: DynamicSafetyContext,
    command: Twist2D | None = None,
    **changes,
):
    return build_dynamic_command_proposal(
        context,
        command=command if command is not None else Twist2D(0.20, 0.0),
        computation_time_s=0.001,
        **changes,
    )


def test_safe_fresh_command_is_accepted_without_gate_override() -> None:
    observation, prediction = _fresh_observation(sequence=0, simulation_time_s=0.0)
    context = _context(
        tick_id=0,
        observation=observation,
        prediction=prediction,
    )
    decision = DynamicSafetyGate().step(
        _proposal(context),
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        context=context,
    )

    assert decision.motion_state is DynamicMotionState.MOVING
    assert decision.proposal_accepted
    assert decision.command == Twist2D(0.20, 0.0)
    assert decision.primary_hold_reason is None
    assert decision.counters.gate_overrides == 0
    assert decision.minimum_static_clearance_m == pytest.approx(1.0)
    assert decision.minimum_actor_clearance_m is None


@pytest.mark.parametrize(
    ("observation_availability", "expected_reason"),
    (
        (DynamicObservationAvailability.STALE, DynamicHoldReason.STALE),
        (DynamicObservationAvailability.INVALID, DynamicHoldReason.INVALID_SOURCE),
        (DynamicObservationAvailability.UNAVAILABLE, DynamicHoldReason.INVALID_SOURCE),
    ),
)
def test_stale_and_invalid_sources_only_allow_limited_deceleration(
    observation_availability: DynamicObservationAvailability,
    expected_reason: DynamicHoldReason,
) -> None:
    observation = DynamicObservationSnapshot(
        availability=observation_availability,
        frame=None,
        age_s=0.31 if observation_availability is DynamicObservationAvailability.STALE else None,
        failures=(),
        last_event_was_no_frame=True,
    )
    context = _context(tick_id=0, observation=observation, prediction=None)
    decision = DynamicSafetyGate().step(
        _proposal(context, Twist2D(0.30, 0.80)),
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D(0.30, 0.80)),
        context=context,
    )

    assert decision.motion_state is DynamicMotionState.BRAKING
    assert decision.command.linear == pytest.approx(0.275)
    assert decision.command.angular == pytest.approx(0.72)
    assert decision.primary_hold_reason is expected_reason
    assert not decision.proposal_accepted
    assert decision.counters.gate_overrides == 1


def test_actor_tube_static_obstacle_and_forbidden_cell_are_all_rejected() -> None:
    robot_state = RobotState(Pose2D(2.0, 2.0))

    actor_observation, actor_prediction = _fresh_observation(
        sequence=0,
        simulation_time_s=0.0,
        actor_position=Point2D(2.45, 2.0),
    )
    actor_context = _context(
        tick_id=0,
        observation=actor_observation,
        prediction=actor_prediction,
    )
    actor = DynamicSafetyGate().step(
        _proposal(actor_context),
        robot_state=robot_state,
        context=actor_context,
    )
    assert actor.primary_hold_reason is DynamicHoldReason.TRAFFIC
    assert "actor_clearance_below_minimum" in actor.failure_reasons
    assert actor.minimum_actor_clearance_m is not None
    assert actor.minimum_actor_clearance_m < 0.08

    empty_observation, empty_prediction = _fresh_observation(
        sequence=0,
        simulation_time_s=0.0,
    )
    static_context = _context(
        tick_id=0,
        observation=empty_observation,
        prediction=empty_prediction,
        grid_snapshot=_grid_snapshot(sequence=0, occupied_cells=((116, 100),)),
    )
    static = DynamicSafetyGate().step(
        _proposal(static_context),
        robot_state=robot_state,
        context=static_context,
    )
    assert static.primary_hold_reason is DynamicHoldReason.GATE_REJECTION
    assert "static_clearance_below_minimum" in static.failure_reasons

    forbidden_context = _context(
        tick_id=0,
        observation=empty_observation,
        prediction=empty_prediction,
        grid_snapshot=_grid_snapshot(
            sequence=0,
            forbidden_cells=frozenset({(113, 100)}),
        ),
    )
    forbidden = DynamicSafetyGate().step(
        _proposal(forbidden_context),
        robot_state=robot_state,
        context=forbidden_context,
    )
    assert forbidden.primary_hold_reason is DynamicHoldReason.GATE_REJECTION
    assert "forbidden_zone_entry" in forbidden.failure_reasons or (
        "static_clearance_below_minimum" in forbidden.failure_reasons
    )


def test_legacy_circle_prediction_keeps_historical_apply_clearance() -> None:
    observation, prediction = _fresh_observation(
        sequence=0,
        simulation_time_s=0.0,
        actor_position=Point2D(2.45, 2.0),
    )
    context = _context(
        tick_id=0,
        observation=observation,
        prediction=prediction,
    )

    evidence = evaluate_dynamic_trajectory_safety(
        _proposal(context),
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        grid_snapshot=context.grid_snapshot,
        prediction_set=prediction,
        profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )

    assert not evidence.safe
    assert evidence.minimum_actor_clearance_m == pytest.approx(-0.063)
    assert evidence.failures == ("actor_clearance_below_minimum",)


@pytest.mark.parametrize(
    ("actor_x", "safe"),
    ((4.0, True), (2.45, False)),
)
def test_directional_capsule_is_checked_with_exact_footprint_clearance(
    actor_x: float,
    safe: bool,
) -> None:
    observation, prediction = _fresh_directional_observation(
        sequence=0,
        simulation_time_s=0.0,
        actor_position=Point2D(actor_x, 2.0),
    )
    context = _directional_context(observation, prediction)

    evidence = evaluate_dynamic_trajectory_safety(
        _proposal(context, Twist2D()),
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        grid_snapshot=context.grid_snapshot,
        prediction_set=prediction,
        profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )

    assert evidence.safe is safe
    assert evidence.actor_hazard is (not safe)
    assert evidence.minimum_actor_clearance_m is not None


def test_directional_capsule_apply_sweep_adds_remaining_motion_expansion() -> None:
    observation, prediction = _fresh_directional_observation(
        sequence=0,
        simulation_time_s=0.0,
        actor_position=Point2D(2.49, 2.0),
    )
    context = _directional_context(observation, prediction)

    evidence = evaluate_dynamic_trajectory_safety(
        _proposal(context, Twist2D()),
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        grid_snapshot=context.grid_snapshot,
        prediction_set=prediction,
        profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )

    assert not evidence.safe
    assert evidence.actor_hazard
    # The issued fixture uses the smallest clear test speed above the frozen
    # directional-confidence threshold; the qualitative apply-sweep hazard and
    # original Actor coordinate remain unchanged.
    assert evidence.minimum_actor_clearance_m == pytest.approx(0.066375)


def test_directional_capsule_rollout_uses_absolute_rollout_time() -> None:
    observation, prediction = _fresh_directional_observation(
        sequence=0,
        simulation_time_s=0.0,
        actor_position=Point2D(2.80, 2.0),
        heading_unit=Vector2D(-1.0, 0.0),
        estimated_speed_mps=0.20,
    )
    context = _directional_context(observation, prediction)
    stationary = RobotState(Pose2D(2.0, 2.0))
    trajectory = (
        TrajectoryPoint(0.0, stationary.pose, Twist2D()),
        TrajectoryPoint(0.8, stationary.pose, Twist2D()),
    )

    evidence = evaluate_dynamic_trajectory_safety(
        _proposal(context, Twist2D(), trajectory=trajectory),
        robot_state=stationary,
        grid_snapshot=context.grid_snapshot,
        prediction_set=prediction,
        profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )

    assert not evidence.safe
    assert evidence.actor_hazard
    assert evidence.minimum_actor_clearance_m is not None
    assert evidence.minimum_actor_clearance_m < 0.08


def test_directional_capsule_terminal_sweep_uses_absolute_terminal_time() -> None:
    observation, prediction = _fresh_directional_observation(
        sequence=0,
        simulation_time_s=0.0,
        actor_position=Point2D(2.64, 2.0),
        heading_unit=Vector2D(-1.0, 0.0),
        estimated_speed_mps=0.20,
    )
    context = _directional_context(observation, prediction)
    robot_state = RobotState(Pose2D(2.0, 2.0))
    rollout = (
        TrajectoryPoint(0.0, robot_state.pose, Twist2D(0.20, 0.0)),
    )

    evidence = evaluate_dynamic_trajectory_safety(
        _proposal(context, Twist2D(0.20, 0.0), trajectory=rollout),
        robot_state=robot_state,
        grid_snapshot=context.grid_snapshot,
        prediction_set=prediction,
        profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )

    assert not evidence.safe
    assert evidence.actor_hazard
    assert evidence.minimum_actor_clearance_m is not None
    assert evidence.minimum_actor_clearance_m < 0.08


def test_directional_prediction_malformed_type_and_nan_fail_closed() -> None:
    observation, prediction = _fresh_directional_observation(
        sequence=0,
        simulation_time_s=0.0,
        actor_position=Point2D(4.0, 2.0),
    )
    valid_context = _directional_context(observation, prediction)
    malformed_type_context = replace(valid_context, prediction_set=object())

    malformed_type = DynamicSafetyGate().step(
        _proposal(malformed_type_context),
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        context=malformed_type_context,
    )

    assert malformed_type.primary_hold_reason is DynamicHoldReason.INVALID_SOURCE
    assert "prediction_type_invalid" in malformed_type.failure_reasons

    object.__setattr__(prediction.tubes[0].anchor_position, "x", float("nan"))
    malformed_nan = DynamicSafetyGate().step(
        _proposal(valid_context),
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        context=valid_context,
    )

    assert not malformed_nan.proposal_accepted
    assert "prediction_set_malformed" in malformed_nan.failure_reasons


def test_tampered_directional_prediction_version_fails_closed() -> None:
    observation, prediction = _fresh_directional_observation(
        sequence=0,
        simulation_time_s=0.0,
        actor_position=Point2D(4.0, 2.0),
    )
    object.__setattr__(prediction, "model_version", "tampered-model")
    context = _directional_context(observation, prediction)

    decision = DynamicSafetyGate().step(
        _proposal(context),
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        context=context,
    )

    assert not decision.proposal_accepted
    assert decision.primary_hold_reason is DynamicHoldReason.GATE_REJECTION
    assert "prediction_set_malformed" in decision.failure_reasons


@pytest.mark.parametrize(
    ("elapsed_s", "expected_reason", "expected_failure"),
    (
        (
            DYNAMIC_CONTROL_PERIOD_S,
            DynamicHoldReason.INVALID_SOURCE,
            "observation_prediction_time_mismatch",
        ),
        (0.35, DynamicHoldReason.STALE, "observation_replay_stale"),
    ),
)
def test_replayed_fresh_issued_directional_prediction_fails_closed(
    elapsed_s: float,
    expected_reason: DynamicHoldReason,
    expected_failure: str,
) -> None:
    observation, prediction = _fresh_directional_observation(
        sequence=0,
        simulation_time_s=0.0,
        actor_position=Point2D(4.0, 2.0),
    )
    original = _directional_context(observation, prediction)
    replayed = replace(
        original,
        tick_id=original.tick_id + round(elapsed_s / DYNAMIC_CONTROL_PERIOD_S),
        simulation_time_s=original.simulation_time_s + elapsed_s,
    )

    decision = DynamicSafetyGate().step(
        _proposal(replayed),
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        context=replayed,
    )

    assert not decision.proposal_accepted
    assert decision.command == Twist2D()
    assert decision.primary_hold_reason is expected_reason
    assert expected_failure in decision.failure_reasons


def test_directional_prediction_is_rejected_after_no_frame_event() -> None:
    observation, prediction = _fresh_directional_observation(
        sequence=0,
        simulation_time_s=0.0,
        actor_position=Point2D(4.0, 2.0),
    )
    dropped = replace(observation, last_event_was_no_frame=True)
    context = _directional_context(dropped, prediction)

    decision = DynamicSafetyGate().step(
        _proposal(context),
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        context=context,
    )

    assert not decision.proposal_accepted
    assert decision.command == Twist2D()
    assert decision.primary_hold_reason is DynamicHoldReason.INVALID_SOURCE
    assert "directional_frame_dropout" in decision.failure_reasons


def test_normalized_rollout_subdivides_seven_milliseconds_below_five_ms() -> None:
    observation, prediction = _fresh_observation(sequence=0, simulation_time_s=0.0)
    context = _context(tick_id=0, observation=observation, prediction=prediction)
    start = Pose2D(2.0, 2.0, 0.0)
    proposal = _proposal(
        context,
        Twist2D(),
        trajectory=(
            TrajectoryPoint(0.0, start, Twist2D()),
            TrajectoryPoint(0.007, Pose2D(2.007, 2.0, 0.0), Twist2D()),
        ),
    )

    samples = _normalized_rollout(
        proposal,
        start,
        VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )
    gaps = tuple(
        target.time_s - source.time_s
        for source, target in zip(samples, samples[1:], strict=False)
    )

    assert tuple(point.time_s for point in samples) == pytest.approx((0.0, 0.0035, 0.007))
    assert max(gaps) <= 0.005


def test_prediction_identity_mismatch_is_invalid_source() -> None:
    observation, prediction = _fresh_observation(sequence=0, simulation_time_s=0.0)
    mismatched = replace(prediction, source_content_hash="different")

    context = _context(
        tick_id=0,
        observation=observation,
        prediction=mismatched,
    )
    decision = DynamicSafetyGate().step(
        _proposal(context),
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        context=context,
    )

    assert decision.primary_hold_reason is DynamicHoldReason.INVALID_SOURCE
    assert "prediction_source_mismatch" in decision.failure_reasons


def test_malformed_rollout_is_rejected_instead_of_raising() -> None:
    observation, prediction = _fresh_observation(sequence=0, simulation_time_s=0.0)
    context = _context(
        tick_id=0,
        observation=observation,
        prediction=prediction,
    )
    malformed = _proposal(
        context,
        trajectory=(
            TrajectoryPoint(
                time_s=0.0,
                pose=Pose2D(2.5, 2.0),
                twist=Twist2D(0.20, 0.0),
            ),
        ),
    )

    decision = DynamicSafetyGate().step(
        malformed,
        robot_state=RobotState(Pose2D(2.0, 2.0)),
        context=context,
    )

    assert decision.primary_hold_reason is DynamicHoldReason.GATE_REJECTION
    assert any(
        reason.startswith("proposal_trajectory_invalid:")
        for reason in decision.failure_reasons
    )


def test_hold_reason_priority_and_event_counters_remain_separate() -> None:
    observation, prediction = _fresh_observation(sequence=0, simulation_time_s=0.0)
    context = _context(
        tick_id=0,
        observation=observation,
        prediction=prediction,
    )
    proposal = _proposal(
        context,
        controller_requested_stop=True,
        no_safe_candidate=True,
    )

    decision = DynamicSafetyGate().step(
        proposal,
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D(0.10, 0.0)),
        context=context,
    )

    assert decision.primary_hold_reason is DynamicHoldReason.NO_SAFE_CANDIDATE
    assert decision.counters.controller_stop_requests == 1
    assert decision.counters.gate_overrides == 1
    assert decision.counters.candidate_rejected_by_gate == 0
