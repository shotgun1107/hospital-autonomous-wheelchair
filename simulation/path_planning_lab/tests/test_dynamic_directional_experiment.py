from __future__ import annotations

from dataclasses import fields, replace

import pytest

from hospital_path_lab.contracts import Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicMotionState,
    build_controller_snapshot,
)
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_directional_experiment import (
    DirectionalPublicEpisodeContextFactory,
)
from hospital_path_lab.dynamic_directional_prediction import DirectionalPredictionStatus
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
)
from hospital_path_lab.dynamic_safety import (
    DYNAMIC_SAFE_OBSERVATION_FRAMES,
    DynamicSafetyContext,
    DynamicSafetyGate,
    build_dynamic_command_proposal,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _same_direction_public_episode():
    return next(
        episode
        for episode in generate_dynamic_v6_public_corpus()
        if episode.latent_case_id == "same-direction-wide-r00"
    )


def _zero_proposal(context: DynamicSafetyContext):
    return build_dynamic_command_proposal(
        context,
        command=Twist2D(),
        computation_time_s=0.001,
    )


def test_public_context_rejects_hidden_before_generating_inputs() -> None:
    hidden = replace(
        _same_direction_public_episode(),
        split=DynamicCorpusSplit.HIDDEN,
    )

    with pytest.raises(ValueError, match="non-public"):
        DirectionalPublicEpisodeContextFactory(hidden, NORMAL_OBSERVATION_PROFILE)


def test_20_hz_duplicate_snapshot_does_not_grow_direction_history() -> None:
    episode = _same_direction_public_episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        NORMAL_OBSERVATION_PROFILE,
    )
    gate = DynamicSafetyGate()

    factory(0, 0.0, episode.initial_state, gate)
    factory(1, 0.05, episode.initial_state, gate)
    first = factory(2, 0.1, episode.initial_state, gate)
    first_result = factory.last_prediction_result
    duplicate = factory(3, 0.15, episode.initial_state, gate)
    duplicate_result = factory.last_prediction_result

    assert first_result is not None
    assert duplicate_result is not None
    assert first_result.status is DirectionalPredictionStatus.WARMING_UP
    assert duplicate_result.status is DirectionalPredictionStatus.WARMING_UP
    assert first_result.history_counts == duplicate_result.history_counts
    assert duplicate_result.duplicate_observation is True
    assert first.prediction_set is None and not first.observation_safe
    assert duplicate.prediction_set is None and not duplicate.observation_safe


def test_warmup_stops_then_ready_unique_frames_and_current_epoch_authorization_resume() -> None:
    episode = _same_direction_public_episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        NORMAL_OBSERVATION_PROFILE,
        authorization_revision=7,
    )
    gate = DynamicSafetyGate()
    stopped_state = episode.initial_state
    saw_warmup = False
    saw_holding = False
    resumed = None

    for tick_id in range(round(8.0 / DYNAMIC_CONTROL_PERIOD_S)):
        simulation_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        context = factory(
            tick_id,
            simulation_time_s,
            stopped_state,
            gate,
        )
        result = factory.last_prediction_result
        assert result is not None
        saw_warmup |= result.status is DirectionalPredictionStatus.WARMING_UP
        decision = gate.step(
            _zero_proposal(context),
            robot_state=stopped_state,
            context=context,
        )
        saw_holding |= decision.motion_state is DynamicMotionState.HOLDING
        if decision.resume_allowed:
            resumed = decision
            break

    assert saw_warmup
    assert saw_holding
    assert resumed is not None
    assert resumed.motion_state is DynamicMotionState.MOVING
    assert resumed.stop_epoch == 1
    assert resumed.consecutive_safe_frames == DYNAMIC_SAFE_OBSERVATION_FRAMES


def test_stress_low_confidence_is_fail_closed_hold() -> None:
    episode = _same_direction_public_episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        STRESS_OBSERVATION_PROFILE,
    )
    gate = DynamicSafetyGate()
    saw_low_confidence = False

    # The Stress profile deliberately resets directional history after stale
    # gaps.  This deterministic public seed reaches a low-confidence 20-frame
    # fit late in the active interval; the context must still fail closed.
    for tick_id in range(round(30.0 / DYNAMIC_CONTROL_PERIOD_S)):
        simulation_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        context = factory(
            tick_id,
            simulation_time_s,
            episode.initial_state,
            gate,
        )
        result = factory.last_prediction_result
        assert result is not None
        decision = gate.step(
            _zero_proposal(context),
            robot_state=episode.initial_state,
            context=context,
        )
        if result.status in {
            DirectionalPredictionStatus.LOW_SPEED,
            DirectionalPredictionStatus.LOW_CONFIDENCE,
        }:
            saw_low_confidence = True
            assert context.prediction_set is None
            assert context.observation_safe is False
            assert decision.motion_state in {
                DynamicMotionState.BRAKING,
                DynamicMotionState.HOLDING,
            }
            assert not decision.resume_allowed

    assert saw_low_confidence
    assert gate.motion_state is DynamicMotionState.HOLDING


@pytest.mark.parametrize(
    ("path_still_valid", "local_safety_recheck_passed"),
    ((False, True), (True, False)),
)
def test_current_epoch_authorization_cannot_bypass_path_and_local_recheck(
    path_still_valid: bool,
    local_safety_recheck_passed: bool,
) -> None:
    episode = _same_direction_public_episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        NORMAL_OBSERVATION_PROFILE,
        path_still_valid=path_still_valid,
        local_safety_recheck_passed=local_safety_recheck_passed,
    )
    gate = DynamicSafetyGate()
    saw_current_epoch_authorization = False

    for tick_id in range(round(8.0 / DYNAMIC_CONTROL_PERIOD_S)):
        simulation_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        context = factory(
            tick_id,
            simulation_time_s,
            episode.initial_state,
            gate,
        )
        if context.resume_authorization is not None:
            saw_current_epoch_authorization = True
            assert context.resume_authorization.stop_epoch == gate.stop_epoch
        decision = gate.step(
            _zero_proposal(context),
            robot_state=episode.initial_state,
            context=context,
        )
        assert not decision.resume_allowed

    assert saw_current_epoch_authorization
    assert gate.motion_state is DynamicMotionState.HOLDING


def test_fresh_empty_frame_is_the_only_non_ready_status_exposed_as_safe() -> None:
    episode = _same_direction_public_episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        NORMAL_OBSERVATION_PROFILE,
    )
    gate = DynamicSafetyGate()
    empty_context = None

    for tick_id in range(round(32.0 / DYNAMIC_CONTROL_PERIOD_S)):
        simulation_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        context = factory(
            tick_id,
            simulation_time_s,
            episode.initial_state,
            gate,
        )
        result = factory.last_prediction_result
        assert result is not None
        if result.status is DirectionalPredictionStatus.EMPTY_FRAME:
            empty_context = context
            break

    assert empty_context is not None
    assert empty_context.observation_safe is True
    assert empty_context.prediction_set is not None
    assert not empty_context.prediction_set.tubes


def test_context_exposes_only_controller_contract_not_evaluator_metadata() -> None:
    episode = _same_direction_public_episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        NORMAL_OBSERVATION_PROFILE,
    )
    gate = DynamicSafetyGate()
    context = None
    for tick_id in range(round(4.0 / DYNAMIC_CONTROL_PERIOD_S)):
        simulation_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        candidate = factory(
            tick_id,
            simulation_time_s,
            episode.initial_state,
            gate,
        )
        if candidate.prediction_set is not None:
            context = candidate
            break

    assert context is not None
    controller_snapshot = build_controller_snapshot(
        tick_id=context.tick_id,
        simulation_time_s=context.simulation_time_s,
        mission_id=context.mission_id,
        robot_state=episode.initial_state,
        goal_pose=episode.goal_pose,
        reference_path=episode.reference_path,
        static_grid_snapshot=context.grid_snapshot,
        validated_observation=context.observation_snapshot,
        actor_tubes=context.prediction_set,
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )

    context_fields = {item.name for item in fields(context)}
    controller_fields = {item.name for item in fields(controller_snapshot)}
    forbidden = {
        "expectation_category",
        "scenario_label",
        "oracle_spec",
        "ground_truth",
    }
    assert context_fields.isdisjoint(forbidden)
    assert controller_fields.isdisjoint(forbidden)
    assert context.grid_snapshot.metadata.map_id == episode.map_id
    assert context.grid_snapshot.metadata.map_revision == 1
    assert context.observation_snapshot.frame is not None
    assert (
        context.grid_snapshot.metadata.observation_revision
        == context.observation_snapshot.frame.observation_revision
        == context.prediction_set.observation_revision
    )
    assert factory.grid_at(context.tick_id) is context.grid_snapshot
