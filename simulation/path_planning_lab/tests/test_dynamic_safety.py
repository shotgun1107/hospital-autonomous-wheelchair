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
    ActorTrack,
    DynamicHoldReason,
    DynamicMotionState,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    Point2D,
    Vector2D,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import ActorPredictionSet, ActorPredictionTube
from hospital_path_lab.dynamic_safety import (
    DynamicSafetyContext,
    DynamicSafetyGate,
    build_dynamic_command_proposal,
)
from hospital_path_lab.grid import GridMap


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
    prediction: ActorPredictionSet | None,
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
