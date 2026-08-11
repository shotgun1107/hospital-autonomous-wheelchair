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
    DynamicCommandProposal,
    DynamicMotionState,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    ResumeAuthorization,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import ActorPredictionSet
from hospital_path_lab.dynamic_safety import (
    DynamicSafetyContext,
    DynamicSafetyGate,
    build_resume_authorization,
)
from hospital_path_lab.grid import GridMap


def _grid(sequence: int) -> GridSnapshot:
    return GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="map-v1",
            map_revision=1,
            mission_revision=1,
            observation_revision=sequence,
            seed=1,
            content_hash=f"grid-{sequence}",
        ),
        grid=GridMap(np.zeros((200, 200), dtype=np.bool_), resolution_m=0.02),
    )


def _fresh_bundle(
    sequence: int,
    simulation_time_s: float,
    *,
    no_frame: bool = False,
) -> tuple[DynamicObservationSnapshot, ActorPredictionSet]:
    frame = DynamicObservationFrame(
        stream_id="stream-v1",
        episode_id="episode-v1",
        episode_seed=1,
        map_id="map-v1",
        map_revision=1,
        observation_revision=sequence,
        sequence=sequence,
        observed_at_s=simulation_time_s,
        delivered_at_s=simulation_time_s,
        frame_kind=DynamicObservationFrameKind.EMPTY,
        tracks=(),
        content_hash=f"frame-{sequence}",
    )
    snapshot = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.FRESH,
        frame=frame,
        age_s=0.0,
        failures=(),
        last_event_was_no_frame=no_frame,
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
    return snapshot, prediction


def _context(
    tick_id: int,
    sequence: int,
    *,
    authorization: ResumeAuthorization | None = None,
    no_frame: bool = False,
) -> DynamicSafetyContext:
    simulation_time_s = tick_id * 0.05
    snapshot, prediction = _fresh_bundle(
        sequence,
        simulation_time_s,
        no_frame=no_frame,
    )
    return DynamicSafetyContext(
        tick_id=tick_id,
        simulation_time_s=simulation_time_s,
        mission_id="mission-v1",
        authorization_revision=7,
        grid_snapshot=_grid(sequence),
        observation_snapshot=snapshot,
        prediction_set=prediction,
        path_still_valid=True,
        local_safety_recheck_passed=True,
        observation_safe=True,
        resume_authorization=authorization,
    )


def _proposal(tick_id: int, sequence: int | None = None) -> DynamicCommandProposal:
    observation_revision = tick_id if sequence is None else sequence
    return DynamicCommandProposal(
        source_tick_id=tick_id,
        command=Twist2D(0.20, 0.0),
        computation_time_s=0.001,
        mission_id="mission-v1",
        map_id="map-v1",
        map_revision=1,
        mission_revision=1,
        observation_revision=observation_revision,
        grid_content_hash=f"grid-{observation_revision}",
        observation_content_hash=f"frame-{observation_revision}",
    )


def _drive_to_holding(gate: DynamicSafetyGate) -> None:
    stale = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.STALE,
        frame=None,
        age_s=0.31,
        failures=(),
        last_event_was_no_frame=True,
    )
    for tick_id in range(3):
        decision = gate.step(
            _proposal(tick_id),
            robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
            context=DynamicSafetyContext(
                tick_id=tick_id,
                simulation_time_s=tick_id * 0.05,
                mission_id="mission-v1",
                authorization_revision=7,
                grid_snapshot=_grid(tick_id),
                observation_snapshot=stale,
                prediction_set=None,
                path_still_valid=True,
                local_safety_recheck_passed=True,
                observation_safe=False,
            ),
        )
    assert decision.motion_state is DynamicMotionState.HOLDING


def test_stop_epoch_increments_once_per_distinct_protective_stop() -> None:
    gate = DynamicSafetyGate()
    _drive_to_holding(gate)

    assert gate.stop_epoch == 1
    for offset in range(3):
        decision = gate.step(
            _proposal(3 + offset, offset),
            robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
            context=_context(3 + offset, offset),
        )
        assert decision.motion_state is DynamicMotionState.HOLDING
        assert decision.stop_epoch == 1


def test_stop_confirmation_requires_both_thresholds_for_three_consecutive_ticks() -> None:
    gate = DynamicSafetyGate()
    stale = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.STALE,
        frame=None,
        age_s=0.31,
        failures=(),
        last_event_was_no_frame=True,
    )
    twists = (
        Twist2D(0.01, 0.02),
        Twist2D(0.010001, 0.02),
        Twist2D(0.01, 0.02),
        Twist2D(0.01, 0.02),
        Twist2D(0.01, 0.02),
    )
    counts = []
    for tick_id, twist in enumerate(twists):
        decision = gate.step(
            _proposal(tick_id),
            robot_state=RobotState(Pose2D(2.0, 2.0), twist),
            context=DynamicSafetyContext(
                tick_id=tick_id,
                simulation_time_s=tick_id * 0.05,
                mission_id="mission-v1",
                authorization_revision=7,
                grid_snapshot=_grid(tick_id),
                observation_snapshot=stale,
                prediction_set=None,
                path_still_valid=True,
                local_safety_recheck_passed=True,
                observation_safe=False,
            ),
        )
        counts.append(decision.consecutive_stop_ticks)

    assert counts == [1, 0, 1, 2, 3]
    assert decision.motion_state is DynamicMotionState.HOLDING
    assert decision.stop_epoch == 1


def test_normal_goal_completion_does_not_create_stop_epoch() -> None:
    gate = DynamicSafetyGate()
    first_context = replace(_context(0, 0), goal_reached=True)
    first = gate.step(
        _proposal(0, 0),
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D(0.20, 0.40)),
        context=first_context,
    )
    assert first.motion_state is DynamicMotionState.BRAKING
    assert first.command.linear == pytest.approx(0.175)
    assert first.command.angular == pytest.approx(0.32)

    for tick_id in range(1, 4):
        context = _context(tick_id, tick_id)
        decision = gate.step(
            _proposal(tick_id),
            robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
            context=context,
        )
    assert decision.motion_state is DynamicMotionState.COMPLETED
    assert decision.consecutive_stop_ticks == 3
    assert decision.stop_epoch == 0
    assert decision.command == Twist2D()


def test_mission_cancel_from_confirmed_hold_completes_without_new_epoch() -> None:
    gate = DynamicSafetyGate()
    _drive_to_holding(gate)
    context = replace(_context(3, 0), mission_cancelled=True)

    decision = gate.step(
        _proposal(3, 0),
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
        context=context,
    )

    assert decision.motion_state is DynamicMotionState.COMPLETED
    assert decision.stop_epoch == 1
    assert decision.command == Twist2D()


def test_eleven_new_safe_frames_and_current_epoch_authorization_resume() -> None:
    gate = DynamicSafetyGate()
    _drive_to_holding(gate)
    authorization = build_resume_authorization(
        mission_id="mission-v1",
        stop_epoch=1,
        issued_or_revalidated_at_s=0.10,
        authorization_revision=7,
    )

    for sequence in range(11):
        tick_id = 3 + sequence
        decision = gate.step(
            _proposal(tick_id, sequence),
            robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
            context=_context(tick_id, sequence, authorization=authorization),
        )

    assert decision.consecutive_safe_frames == 11
    assert decision.resume_allowed
    assert decision.motion_state is DynamicMotionState.MOVING
    assert decision.command == Twist2D(0.20, 0.0)
    assert decision.stop_epoch == 1


def test_hazard_clear_and_safe_frames_without_new_authorization_do_not_resume() -> None:
    gate = DynamicSafetyGate()
    _drive_to_holding(gate)

    for sequence in range(11):
        tick_id = 3 + sequence
        decision = gate.step(
            _proposal(tick_id, sequence),
            robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
            context=_context(tick_id, sequence),
        )

    assert decision.consecutive_safe_frames == 11
    assert not decision.resume_allowed
    assert decision.motion_state is DynamicMotionState.HOLDING
    assert decision.command == Twist2D()


@pytest.mark.parametrize(
    "change",
    (
        {"path_still_valid": False},
        {"local_safety_recheck_passed": False},
    ),
)
def test_path_and_local_safety_recheck_are_both_required_to_resume(change) -> None:
    gate = DynamicSafetyGate()
    _drive_to_holding(gate)
    authorization = build_resume_authorization(
        mission_id="mission-v1",
        stop_epoch=1,
        issued_or_revalidated_at_s=0.10,
        authorization_revision=7,
    )
    for sequence in range(11):
        tick_id = 3 + sequence
        context = replace(
            _context(tick_id, sequence, authorization=authorization),
            **change,
        )
        decision = gate.step(
            _proposal(tick_id, sequence),
            robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
            context=context,
        )

    assert not decision.resume_allowed
    assert decision.motion_state is DynamicMotionState.HOLDING
    assert decision.command == Twist2D()


def test_no_frame_resets_continuous_safe_frame_count() -> None:
    gate = DynamicSafetyGate()
    _drive_to_holding(gate)
    authorization = build_resume_authorization(
        mission_id="mission-v1",
        stop_epoch=1,
        issued_or_revalidated_at_s=0.10,
        authorization_revision=7,
    )
    for sequence in range(5):
        tick_id = 3 + sequence
        decision = gate.step(
            _proposal(tick_id, sequence),
            robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
            context=_context(tick_id, sequence, authorization=authorization),
        )
    assert decision.consecutive_safe_frames == 5

    reset = gate.step(
        _proposal(8, 5),
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
        context=_context(8, 5, authorization=authorization, no_frame=True),
    )
    assert reset.consecutive_safe_frames == 0
    assert not reset.resume_allowed


@pytest.mark.parametrize("fault", ("mission", "epoch", "time", "revision", "hash"))
def test_wrong_or_pre_stop_authorization_is_rejected(fault: str) -> None:
    gate = DynamicSafetyGate()
    _drive_to_holding(gate)
    if fault == "mission":
        authorization = build_resume_authorization(
            mission_id="other-mission",
            stop_epoch=1,
            issued_or_revalidated_at_s=0.10,
            authorization_revision=7,
        )
    elif fault == "epoch":
        authorization = build_resume_authorization(
            mission_id="mission-v1",
            stop_epoch=0,
            issued_or_revalidated_at_s=0.10,
            authorization_revision=7,
        )
    elif fault == "time":
        authorization = build_resume_authorization(
            mission_id="mission-v1",
            stop_epoch=1,
            issued_or_revalidated_at_s=0.05,
            authorization_revision=7,
        )
    elif fault == "revision":
        authorization = build_resume_authorization(
            mission_id="mission-v1",
            stop_epoch=1,
            issued_or_revalidated_at_s=0.10,
            authorization_revision=6,
        )
    else:
        authorization = replace(
            build_resume_authorization(
                mission_id="mission-v1",
                stop_epoch=1,
                issued_or_revalidated_at_s=0.10,
                authorization_revision=7,
            ),
            content_hash="tampered",
        )

    decision = gate.step(
        _proposal(3, 0),
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D()),
        context=_context(3, 0, authorization=authorization),
    )

    assert not decision.resume_allowed
    assert decision.motion_state is DynamicMotionState.HOLDING
    assert decision.counters.resume_authorizations_rejected == 1
