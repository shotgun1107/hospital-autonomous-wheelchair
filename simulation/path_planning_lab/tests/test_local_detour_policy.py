from __future__ import annotations

from dataclasses import replace

import pytest

from hospital_path_lab.dynamic_contracts import build_controller_snapshot
from hospital_path_lab.dynamic_corpus import generate_dynamic_v6_public_corpus
from hospital_path_lab.dynamic_directional_experiment import (
    DirectionalPublicEpisodeContextFactory,
)
from hospital_path_lab.dynamic_directional_prediction import DirectionalPredictionStatus
from hospital_path_lab.dynamic_observation import (
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
)
from hospital_path_lab.dynamic_safety import DynamicSafetyGate
from hospital_path_lab.local_algorithms.dwb_reference import (
    SourceDerivedDwbConfig,
    SourceDerivedDynamicDwbController,
)
from hospital_path_lab.local_detour_policy import (
    DirectionalLocalDetourDwbController,
    DirectionalLocalDetourPolicy,
    LocalDetourPolicyState,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _episode():
    return next(
        item
        for item in generate_dynamic_v6_public_corpus()
        if item.latent_case_id == "same-direction-wide-r00"
    )


def _snapshot(episode, factory, gate, tick_id):
    time_s = tick_id * VIRTUAL_DOLL_WHEELCHAIR_V0_1.control_period_s
    context = factory(tick_id, time_s, episode.initial_state, gate)
    return build_controller_snapshot(
        tick_id=tick_id,
        simulation_time_s=time_s,
        mission_id=context.mission_id,
        robot_state=episode.initial_state,
        goal_pose=episode.goal_pose,
        reference_path=episode.reference_path,
        static_grid_snapshot=context.grid_snapshot,
        validated_observation=context.observation_snapshot,
        actor_tubes=context.prediction_set,
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )


def test_policy_is_inert_without_a_ready_directional_actor() -> None:
    episode = _episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    )
    policy = DirectionalLocalDetourPolicy()
    snapshot = _snapshot(episode, factory, DynamicSafetyGate(), 0)

    assert policy.apply(snapshot) is snapshot
    assert policy.state is LocalDetourPolicyState.TRACK_REFERENCE


def test_ready_same_heading_actor_latches_label_free_local_reference() -> None:
    episode = _episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    )
    gate = DynamicSafetyGate()
    snapshot = None
    for tick_id in range(episode.tick_count):
        snapshot = _snapshot(episode, factory, gate, tick_id)
        if (
            factory.last_prediction_result is not None
            and factory.last_prediction_result.status is DirectionalPredictionStatus.READY
        ):
            break
    assert snapshot is not None

    policy = DirectionalLocalDetourPolicy()
    transformed = policy.apply(snapshot)

    assert policy.state is LocalDetourPolicyState.DETOUR_REFERENCE_ACTIVE
    assert transformed.reference_path != snapshot.reference_path
    assert transformed.reference_path[-1] == snapshot.goal_pose
    assert len(transformed.reference_path) == 5
    assert transformed.reference_path[1].y < snapshot.robot_state.pose.y
    assert transformed.reference_path[2].y == pytest.approx(
        transformed.reference_path[1].y
    )
    assert transformed.reference_path[3].y == pytest.approx(snapshot.goal_pose.y)
    assert policy.apply(snapshot).reference_path == transformed.reference_path


def test_local_reference_turn_is_selected_without_weakening_candidate_safety() -> None:
    episode = _episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    )
    gate = DynamicSafetyGate()
    snapshot = None
    for tick_id in range(episode.tick_count):
        snapshot = _snapshot(episode, factory, gate, tick_id)
        if (
            factory.last_prediction_result is not None
            and factory.last_prediction_result.status is DirectionalPredictionStatus.READY
        ):
            break
    assert snapshot is not None

    policy = DirectionalLocalDetourPolicy()
    result = SourceDerivedDynamicDwbController().step(policy.apply(snapshot))

    assert result.status.value == "found"
    assert result.requested_twist.linear == 0.0
    assert result.requested_twist.angular < 0.0
    assert not result.no_safe_candidate
    assert len(result.predicted_trajectory) == 41


def test_detour_research_config_removes_only_final_goal_heading_pull() -> None:
    config = SourceDerivedDwbConfig(goal_align_scale=0.0)

    assert config.goal_align_scale == 0.0
    assert config.goal_dist_scale == SourceDerivedDwbConfig().goal_dist_scale
    assert config.path_align_scale == SourceDerivedDwbConfig().path_align_scale
    assert config.path_dist_scale == SourceDerivedDwbConfig().path_dist_scale


def test_segment_wrapper_selects_safe_forward_motion_when_entry_is_aligned() -> None:
    episode = _episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    )
    gate = DynamicSafetyGate()
    snapshot = None
    for tick_id in range(episode.tick_count):
        snapshot = _snapshot(episode, factory, gate, tick_id)
        if (
            factory.last_prediction_result is not None
            and factory.last_prediction_result.status
            is DirectionalPredictionStatus.READY
        ):
            break
    assert snapshot is not None
    aligned = build_controller_snapshot(
        tick_id=snapshot.tick_id,
        simulation_time_s=snapshot.simulation_time_s,
        mission_id=snapshot.mission_id,
        robot_state=replace(
            snapshot.robot_state,
            pose=replace(snapshot.robot_state.pose, yaw=-1.5707963267948966),
        ),
        goal_pose=snapshot.goal_pose,
        reference_path=snapshot.reference_path,
        static_grid_snapshot=snapshot.static_grid_snapshot,
        validated_observation=snapshot.validated_observation,
        actor_tubes=snapshot.actor_tubes,
        vehicle_profile=snapshot.vehicle_profile,
    )
    controller = DirectionalLocalDetourDwbController()

    result = controller.step(aligned)

    assert controller.active_waypoint_index == 1
    assert result.status.value == "found"
    assert result.requested_twist.linear > 0.0
    assert not result.no_safe_candidate
