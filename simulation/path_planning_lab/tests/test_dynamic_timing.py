from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from hospital_path_lab.contracts import (
    GridSnapshot,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import (
    DynamicHoldReason,
    DynamicMotionState,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import ActorPredictionSet
from hospital_path_lab.dynamic_safety import (
    DynamicSafetyContext,
    DynamicSafetyGate,
    build_dynamic_command_proposal,
)
from hospital_path_lab.grid import GridMap


def _context(tick_id: int) -> DynamicSafetyContext:
    simulation_time_s = tick_id * 0.05
    frame = DynamicObservationFrame(
        stream_id="stream-v1",
        episode_id="episode-v1",
        episode_seed=1,
        map_id="map-v1",
        map_revision=1,
        observation_revision=tick_id,
        sequence=tick_id,
        observed_at_s=simulation_time_s,
        delivered_at_s=simulation_time_s,
        frame_kind=DynamicObservationFrameKind.EMPTY,
        tracks=(),
        content_hash=f"frame-{tick_id}",
    )
    observation = DynamicObservationSnapshot(
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
        tubes=(),
    )
    grid = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="map-v1",
            map_revision=1,
            mission_revision=1,
            observation_revision=tick_id,
            seed=1,
            content_hash=f"grid-{tick_id}",
        ),
        grid=GridMap(np.zeros((200, 200), dtype=np.bool_), resolution_m=0.02),
    )
    return DynamicSafetyContext(
        tick_id=tick_id,
        simulation_time_s=simulation_time_s,
        mission_id="mission-v1",
        authorization_revision=1,
        grid_snapshot=grid,
        observation_snapshot=observation,
        prediction_set=prediction,
        path_still_valid=True,
        local_safety_recheck_passed=True,
        observation_safe=True,
    )


@pytest.mark.parametrize("elapsed_s", (0.049, 0.050))
def test_49_and_50_millisecond_results_are_valid_for_current_tick(
    elapsed_s: float,
) -> None:
    context = _context(0)
    decision = DynamicSafetyGate().step(
        build_dynamic_command_proposal(
            context,
            command=Twist2D(0.20, 0.0),
            computation_time_s=elapsed_s,
        ),
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
        context=context,
    )

    assert decision.proposal_accepted
    assert decision.motion_state is DynamicMotionState.MOVING
    assert decision.counters.late_results_discarded == 0


def test_51_millisecond_result_is_discarded_and_braking_is_applied() -> None:
    context = _context(0)
    decision = DynamicSafetyGate().step(
        build_dynamic_command_proposal(
            context,
            command=Twist2D(0.20, 0.40),
            computation_time_s=0.051,
        ),
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D(0.10, 0.20)),
        context=context,
    )

    assert not decision.proposal_accepted
    assert decision.primary_hold_reason is DynamicHoldReason.DEADLINE
    assert decision.command.linear == pytest.approx(0.075)
    assert decision.command.angular == pytest.approx(0.12)
    assert decision.counters.late_results_discarded == 1
    assert "late_or_wrong_tick_result" in decision.failure_reasons


@pytest.mark.parametrize("source_tick_id", (0, 2))
def test_past_or_future_tick_result_is_never_applied(source_tick_id: int) -> None:
    context = _context(1)
    decision = DynamicSafetyGate().step(
        build_dynamic_command_proposal(
            context,
            command=Twist2D(0.20, 0.0),
            computation_time_s=0.001,
            source_tick_id=source_tick_id,
        ),
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D(0.05, 0.0)),
        context=context,
    )

    assert not decision.proposal_accepted
    assert decision.command.linear == pytest.approx(0.025)
    assert decision.primary_hold_reason is DynamicHoldReason.DEADLINE
    assert decision.counters.late_results_discarded == 1


def test_late_old_result_cannot_replace_a_previously_accepted_newer_result() -> None:
    gate = DynamicSafetyGate()
    first_context = _context(0)
    first = gate.step(
        build_dynamic_command_proposal(
            first_context,
            command=Twist2D(0.20, 0.0),
            computation_time_s=0.001,
        ),
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
        context=first_context,
    )
    second_context = _context(1)
    old = gate.step(
        build_dynamic_command_proposal(
            second_context,
            command=Twist2D(0.30, 0.80),
            computation_time_s=0.001,
            source_tick_id=0,
        ),
        robot_state=RobotState(Pose2D(2.01, 2.0), first.command),
        context=second_context,
    )

    assert first.proposal_accepted
    assert not old.proposal_accepted
    assert old.command != Twist2D(0.30, 0.80)
    assert old.primary_hold_reason is DynamicHoldReason.DEADLINE


def test_context_tick_ids_must_increase() -> None:
    gate = DynamicSafetyGate()
    context = _context(0)
    proposal = build_dynamic_command_proposal(
        context,
        command=Twist2D(),
        computation_time_s=0.001,
    )
    gate.step(
        proposal,
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
        context=context,
    )

    with pytest.raises(ValueError, match="tick_id must increase"):
        gate.step(
            proposal,
            robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
            context=context,
        )


@pytest.mark.parametrize(
    "change",
    (
        {"mission_id": "other-mission"},
        {"map_id": "other-map"},
        {"map_revision": 2},
        {"mission_revision": 2},
        {"observation_revision": 2},
        {"grid_content_hash": "other-grid"},
        {"observation_content_hash": "other-observation"},
    ),
)
def test_proposal_provenance_mismatch_is_rejected(change) -> None:
    context = _context(0)
    proposal = build_dynamic_command_proposal(
        context,
        command=Twist2D(0.20, 0.0),
        computation_time_s=0.001,
    )
    proposal = replace(proposal, **change)

    decision = DynamicSafetyGate().step(
        proposal,
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D(0.10, 0.0)),
        context=context,
    )

    assert not decision.proposal_accepted
    assert decision.primary_hold_reason is DynamicHoldReason.INVALID_SOURCE
    assert "proposal_provenance_mismatch" in decision.failure_reasons
