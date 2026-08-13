from __future__ import annotations

import ast
import inspect
from dataclasses import replace

import pytest

import hospital_path_lab.dynamic_witness_profile_replay as replay_module
from hospital_path_lab.dynamic_corpus import generate_dynamic_v6_public_corpus
from hospital_path_lab.dynamic_observation import (
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    FUNCTIONAL_NO_DROPOUT_OBSERVATION_PROFILE,
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationProfileName,
)
from hospital_path_lab.dynamic_witness_contracts import (
    ManeuverConstraintSpec,
    PassingPolicy,
    PassSide,
    project_public_witness_world,
)
from hospital_path_lab.dynamic_witness_pass import (
    generate_frozen_frontier_pass_candidate,
)
from hospital_path_lab.dynamic_witness_profile_replay import (
    replay_witness_profile,
    replay_witness_profiles,
)
from hospital_path_lab.dynamic_witness_validation import (
    validate_ground_truth_witness,
)


def _r00_world_and_witness(side: PassSide = PassSide.RIGHT):
    episode = next(
        episode
        for episode in generate_dynamic_v6_public_corpus()
        if episode.latent_case_id == "same-direction-wide-r00"
    )
    world = project_public_witness_world(
        episode,
        maneuver_constraints=ManeuverConstraintSpec(
            passing_policy=PassingPolicy.ALLOWED,
        ),
    )
    witness = generate_frozen_frontier_pass_candidate(
        world,
        actor_binding_id=world.actors[0].actor_binding_id,
        side=side,
    )
    assert witness is not None
    validation = validate_ground_truth_witness(
        world,
        witness,
        strict_declarations=True,
    )
    assert validation.passed, validation.failures
    return world, witness, validation


@pytest.fixture(scope="module")
def r00_bundle():
    world, witness, validation = _r00_world_and_witness()
    return world, witness, validation, replay_witness_profiles(world, witness, validation)


def test_bundle_uses_frozen_profile_order_and_is_deterministic(r00_bundle) -> None:
    world, witness, validation, first = r00_bundle
    second = replay_witness_profiles(world, witness, validation)

    assert tuple(result.observation_profile_name for result in first.results) == (
        DynamicObservationProfileName.FUNCTIONAL_IDEAL,
        DynamicObservationProfileName.NORMAL,
        DynamicObservationProfileName.STRESS,
    )
    assert first == second
    assert first.semantic_content_hash == second.semantic_content_hash
    assert first.world_content_hash == world.content_hash
    assert first.witness_content_hash == witness.semantic_content_hash
    assert first.ground_truth_validation_hash == validation.content_hash


def test_ideal_replay_exposes_that_first_ready_delay_invalidates_the_pass(
    r00_bundle,
) -> None:
    _world, witness, _validation, bundle = r00_bundle
    ideal = bundle.results[0]

    assert ideal.observation_profile_name is DynamicObservationProfileName.FUNCTIONAL_IDEAL
    assert ideal.dropout_count == 0
    assert ideal.observation_decidable
    assert ideal.first_ready_tick is not None
    assert ideal.first_ready_tick > 0
    assert ideal.delayed_witness is not None
    assert ideal.delayed_witness.points[0].pose == witness.points[0].pose
    assert ideal.delayed_witness.points[0].twist.linear == 0.0
    assert ideal.shifted_completion_within_episode
    assert not ideal.delayed_ground_truth_valid
    assert ideal.hard_failures == ()
    assert ideal.actual_actor_containment_miss_count == 0
    assert ideal.capsule_sample_count > 0
    assert ideal.minimum_predicted_clearance_m is not None
    assert not ideal.capsule_geometry_admissible_when_observed
    assert not ideal.prediction_admissible
    assert "delayed_ground_truth_invalid" in ideal.limitations
    assert "predicted_clearance_rejected" in ideal.limitations
    assert "actor_clearance_violation" in ideal.delayed_validation_failures
    assert "declared_pass_time_mismatch" in ideal.delayed_validation_failures

    delay_s = ideal.first_ready_time_s
    assert delay_s is not None
    assert ideal.delayed_witness.departure_time_s == pytest.approx(
        witness.departure_time_s + delay_s,
    )
    assert ideal.delayed_witness.pass_times_by_actor[0][1] == pytest.approx(
        witness.pass_times_by_actor[0][1] + delay_s,
    )
    assert ideal.delayed_witness.rejoin_confirmed_at_s == pytest.approx(
        witness.rejoin_confirmed_at_s + delay_s,
    )


def test_normal_and_stress_keep_degradation_separate_from_ground_truth(r00_bundle) -> None:
    _world, _witness, _validation, bundle = r00_bundle
    normal = bundle.results[1]
    stress = bundle.results[2]

    assert normal.observation_profile_name is DynamicObservationProfileName.NORMAL
    assert stress.observation_profile_name is DynamicObservationProfileName.STRESS
    assert normal.hard_failures == ()
    assert stress.hard_failures == ()
    assert normal.dropout_count > 0
    assert stress.dropout_count >= normal.dropout_count
    assert "online_controller_and_gate_not_evaluated" in normal.limitations
    assert "simulation_only_open_loop_circular_actor" in stress.limitations
    assert not stress.prediction_admissible
    assert (
        not stress.observation_decidable
        or not stress.observation_continuous_for_witness
        or not stress.capsule_geometry_admissible_when_observed
    )


def test_status_intervals_cover_every_control_tick_without_overlap(r00_bundle) -> None:
    world, _witness, _validation, bundle = r00_bundle
    expected_tick_count = round(world.duration_s / 0.05) + 1

    for result in bundle.results:
        assert sum(interval.tick_count for interval in result.status_intervals) == (
            expected_tick_count
        )
        assert result.status_intervals[0].start_tick == 0
        assert result.status_intervals[-1].end_tick == expected_tick_count - 1
        for left, right in zip(
            result.status_intervals,
            result.status_intervals[1:],
            strict=False,
        ):
            assert right.start_tick == left.end_tick + 1


def test_profile_replay_rejects_non_frozen_profile() -> None:
    world, witness, validation = _r00_world_and_witness()

    with pytest.raises(ValueError, match="frozen Ideal/Normal/Stress"):
        replay_witness_profile(
            world,
            witness,
            validation,
            FUNCTIONAL_NO_DROPOUT_OBSERVATION_PROFILE,
        )


def test_profile_replay_rejects_validation_for_a_different_witness() -> None:
    world, right, _right_validation = _r00_world_and_witness(PassSide.RIGHT)
    _same_world, left, left_validation = _r00_world_and_witness(PassSide.LEFT)
    assert right.semantic_content_hash != left.semantic_content_hash

    with pytest.raises(ValueError, match="exact passing ground-truth validation"):
        replay_witness_profiles(world, right, left_validation)


def test_ideal_result_tamper_changes_semantic_hash(r00_bundle) -> None:
    _world, _witness, _validation, bundle = r00_bundle
    ideal = bundle.results[0]
    tampered = replace(
        ideal,
        limitations=ideal.limitations + ("tampered-replay",),
    )

    assert tampered.semantic_content_hash != ideal.semantic_content_hash


def test_replay_module_has_no_corpus_label_or_hidden_dependency() -> None:
    tree = ast.parse(inspect.getsource(replay_module))
    forbidden = {
        "dynamic_corpus",
        "expectation_category",
        "oracle_spec",
        "latent_case_id",
        "hidden",
        "controller",
    }
    imported_modules: set[str] = set()
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                imported_modules.add(node.module)
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)

    joined_modules = " ".join(sorted(imported_modules))
    assert "dynamic_corpus" not in joined_modules
    assert not (forbidden & names)


@pytest.mark.parametrize(
    "profile",
    (
        FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
        NORMAL_OBSERVATION_PROFILE,
        STRESS_OBSERVATION_PROFILE,
    ),
)
def test_single_profile_result_matches_bundle(profile, r00_bundle) -> None:
    world, witness, validation, bundle = r00_bundle
    single = replay_witness_profile(world, witness, validation, profile)
    bundled = next(
        result
        for result in bundle.results
        if result.observation_profile_name is profile.name
    )

    assert single == bundled
    assert single.semantic_content_hash == bundled.semantic_content_hash
