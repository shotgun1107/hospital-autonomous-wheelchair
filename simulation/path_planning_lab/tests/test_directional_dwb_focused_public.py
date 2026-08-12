"""Focused PUBLIC-only source-derived DWB behavioral qualification.

This test intentionally uses exactly one frozen development episode,
``same-direction-wide-r00`` under the Normal observation profile.  Corpus labels
and evaluator oracles are used only after the closed-loop trace has been
produced; neither the controller nor its context receives them as inputs.

The 50 ms wall-clock qualification is deliberately outside this behavioral
test.  Candidate count, rollout length, terminal stopping, shared safety gate,
Actor prediction and every corpus tick remain unchanged.
"""

from __future__ import annotations

from dataclasses import replace

from hospital_path_lab.contracts import PlanStatus
from hospital_path_lab.dynamic_contracts import (
    ControllerSnapshot,
    DynamicMotionState,
    build_controller_snapshot,
)
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    V6DynamicCorpusEpisode,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_directional_experiment import (
    DirectionalPublicEpisodeContextFactory,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalPredictionStatus,
)
from hospital_path_lab.dynamic_evaluation import evaluate_dynamic_pipeline
from hospital_path_lab.dynamic_observation import (
    FUNCTIONAL_NO_DROPOUT_OBSERVATION_PROFILE,
    NORMAL_OBSERVATION_PROFILE,
)
from hospital_path_lab.dynamic_safety import DynamicSafetyGate
from hospital_path_lab.local_algorithms.dwb_reference import (
    SourceDerivedDynamicDwbController,
)
from hospital_path_lab.simulation import simulate_dynamic_controller_pipeline
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _episode() -> V6DynamicCorpusEpisode:
    matches = tuple(
        episode
        for episode in generate_dynamic_v6_public_corpus()
        if episode.latent_case_id == "same-direction-wide-r00"
    )
    assert len(matches) == 1
    episode = matches[0]
    assert episode.split in {
        DynamicCorpusSplit.GOLDEN,
        DynamicCorpusSplit.DEVELOPMENT,
    }
    return episode


def _first_ready_snapshot(
    episode: V6DynamicCorpusEpisode,
) -> tuple[ControllerSnapshot, DirectionalPublicEpisodeContextFactory]:
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        NORMAL_OBSERVATION_PROFILE,
    )
    gate = DynamicSafetyGate()
    for tick_id in range(episode.tick_count):
        time_s = tick_id * VIRTUAL_DOLL_WHEELCHAIR_V0_1.control_period_s
        context = factory(tick_id, time_s, episode.initial_state, gate)
        result = factory.last_prediction_result
        if result is None or result.status is not DirectionalPredictionStatus.READY:
            continue
        snapshot = build_controller_snapshot(
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
        return snapshot, factory
    raise AssertionError("Normal public episode never produced a READY prediction")


def _trace_value(trace: tuple[str, ...], key: str) -> int:
    prefix = f"{key}="
    match = tuple(item for item in trace if item.startswith(prefix))
    assert len(match) == 1, trace
    return int(match[0][len(prefix) :])


def test_first_ready_tick_has_a_legal_source_derived_dwb_candidate() -> None:
    """Diagnostic gate: public READY evidence must produce a legal batch."""

    episode = _episode()
    snapshot, _factory = _first_ready_snapshot(episode)
    controller = SourceDerivedDynamicDwbController()

    result = controller.step(snapshot)

    assert _trace_value(result.decision_trace, "candidate_count") == 217
    assert result.status is PlanStatus.FOUND, result.decision_trace
    assert _trace_value(result.decision_trace, "legal_candidates") > 0
    assert len(result.predicted_trajectory) == 41


def _manual_dropout_free_same_direction_wide_r00_completes_ordered_detour_and_rejoin(
) -> None:
    """Manual long-run continuation; it is not part of the regular test suite.

    This functional-isolation lane keeps Normal latency and noise but removes
    random frame dropout.  The frozen Normal profile and its safety regression
    remain unchanged.  Keep this hours-long harness outside regular pytest.
    """

    episode = _episode()
    factory = DirectionalPublicEpisodeContextFactory(
        episode,
        FUNCTIONAL_NO_DROPOUT_OBSERVATION_PROFILE,
    )
    pipeline = simulate_dynamic_controller_pipeline(
        SourceDerivedDynamicDwbController(),
        initial_state=episode.initial_state,
        reference_path=episode.reference_path,
        goal=episode.goal_pose,
        context_factory=factory,
        max_ticks=episode.tick_count,
        simulated_computation_time_s=0.001,
    )

    # The current evaluator's DWA category oracle predates the source-derived
    # controller name.  This immutable post-run role projection changes no
    # controller input, command, gate decision or trace content.
    evaluation = evaluate_dynamic_pipeline(
        replace(pipeline, controller_name="dynamic_dwa"),
        episode_id=episode.episode_id,
        expectation_category=episode.expectation_category.value,
        progressable=episode.progressable,
        reference_path=episode.reference_path,
        goal_pose=episode.goal_pose,
        actor_states_at=episode.actor_states_at,
        grid_snapshot_at=factory.grid_at,
        blocking_cleared_at_s=episode.blocking_cleared_at_s,
        oracle_spec=episode.oracle_spec,
    )

    moving_steps = tuple(
        step
        for step in pipeline.steps
        if step.safety_decision.motion_state is DynamicMotionState.MOVING
        and (
            abs(step.safety_decision.command.linear) > 1e-12
            or abs(step.safety_decision.command.angular) > 1e-12
        )
    )
    assert evaluation.hard_safety.passed, evaluation.hard_safety.failures
    assert moving_steps, "source-derived DWB never departed"
    assert evaluation.metrics.maximum_reference_deviation_m > (
        episode.oracle_spec.departure_threshold_m
    )
    assert evaluation.metrics.overtaking_observed
    assert set(evaluation.metrics.same_direction_overtaking_actor_ids) >= set(
        episode.oracle_spec.same_direction_actor_ids
    )
    assert evaluation.metrics.rejoin_observed
    assert pipeline.completed
    assert evaluation.functional_qualified, evaluation.functional_failures
