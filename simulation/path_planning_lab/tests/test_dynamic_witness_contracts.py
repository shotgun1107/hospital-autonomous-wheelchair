from __future__ import annotations

from dataclasses import fields, replace

import pytest

from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_witness_contracts import (
    FROZEN_WITNESS_SEARCH_CONFIG,
    AutomatedWitness,
    ManeuverConstraintSpec,
    PassingPolicy,
    WitnessKind,
    WitnessObjective,
    WitnessSearchConfig,
    WitnessSearchResult,
    WitnessSearchStatus,
    WitnessTerminalMode,
    project_public_witness_world,
)

_FORBIDDEN_PROJECTION_NAMES = {
    "split",
    "expectation_category",
    "scenario_family",
    "orientation",
    "variant",
    "latent_case_id",
    "oracle_spec",
    "feasible_witness",
    "progressable",
    "blocking_cleared_at_s",
    "observation_fault",
    "controller_id",
    "critic_results",
    "episode_id",
    "mission_id",
}


def test_public_projection_is_label_and_oracle_free() -> None:
    episode = generate_dynamic_v6_public_corpus()[0]

    world = project_public_witness_world(episode)

    field_names = {field.name for field in fields(world)}
    assert field_names.isdisjoint(_FORBIDDEN_PROJECTION_NAMES)
    assert episode.episode_id not in repr(world)
    assert episode.map_id not in repr(world)
    assert episode.actors[0].actor_id not in repr(world)
    assert world.actors[0].actor_binding_id.startswith("actor-000-")
    assert world.simulation_only


def test_legacy_label_changes_do_not_change_projection_semantics() -> None:
    episode = generate_dynamic_corpus()[0]
    changed_label = replace(
        episode,
        expectation_category=DynamicExpectationCategory.NO_SAFE_SOLUTION,
        progressable=not episode.progressable,
        blocking_cleared_at_s=None,
        observation_fault="forged-evaluator-only-value",
    )

    original_world = project_public_witness_world(episode)
    changed_world = project_public_witness_world(changed_label)

    assert changed_world == original_world
    assert changed_world.content_hash == original_world.content_hash
    assert (
        changed_world.source_projection_hash
        == original_world.source_projection_hash
    )


def test_hidden_projection_is_rejected_before_content_is_exposed() -> None:
    hidden = replace(generate_dynamic_corpus()[0], split=DynamicCorpusSplit.HIDDEN)

    with pytest.raises(ValueError, match="rejects hidden"):
        project_public_witness_world(hidden)


def test_explicit_policy_is_hashed_without_category_inference() -> None:
    episode = generate_dynamic_v6_public_corpus()[0]
    unspecified = project_public_witness_world(episode)
    prohibited = project_public_witness_world(
        episode,
        maneuver_constraints=ManeuverConstraintSpec(
            passing_policy=PassingPolicy.PROHIBITED,
        ),
    )

    assert unspecified.maneuver_constraints.passing_policy is PassingPolicy.UNSPECIFIED
    assert prohibited.maneuver_constraints.passing_policy is PassingPolicy.PROHIBITED
    assert prohibited.content_hash != unspecified.content_hash
    assert prohibited.source_projection_hash != unspecified.source_projection_hash


def test_frozen_config_rejects_ambiguous_or_invalid_search_space() -> None:
    with pytest.raises(ValueError, match="unique and sorted"):
        WitnessSearchConfig(linear_targets_mps=(0.20, 0.10))
    with pytest.raises(ValueError, match="include zero"):
        WitnessSearchConfig(angular_targets_radps=(-0.8, 0.8))
    with pytest.raises(ValueError, match="positive"):
        WitnessSearchConfig(max_geometry_candidates_per_episode=0)


def test_search_result_semantic_hash_excludes_wall_clock() -> None:
    world = project_public_witness_world(generate_dynamic_v6_public_corpus()[0])
    base = WitnessSearchResult(
        status=WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE,
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        search_config_hash=FROZEN_WITNESS_SEARCH_CONFIG.content_hash,
        generated_count=7,
        geometry_pruned_count=3,
        dynamic_rejected_count=4,
        validated_count=0,
        selected_witness=None,
        termination_reason="structured_template_exhausted",
        deterministic_objective=None,
        elapsed_nonqualification_ns=10,
    )

    slower = replace(base, elapsed_nonqualification_ns=99_999)

    assert slower.semantic_content_hash == base.semantic_content_hash


def test_found_result_requires_witness_and_objective_together() -> None:
    world = project_public_witness_world(generate_dynamic_v6_public_corpus()[0])
    objective = WitnessObjective(
        hard_failure_count=0,
        terminal_completion_time_s=1.0,
        actual_path_length_m=0.0,
        maximum_reference_deviation_m=0.0,
        full_stop_count=0,
        absolute_angular_travel_rad=0.0,
        kind_rank=3,
        frozen_parameter_tuple=(),
    )

    with pytest.raises(ValueError, match="selected witness"):
        WitnessSearchResult(
            status=WitnessSearchStatus.WITNESS_FOUND,
            source_projection_hash=world.source_projection_hash,
            world_content_hash=world.content_hash,
            search_config_hash=world.search_config_hash,
            generated_count=1,
            geometry_pruned_count=0,
            dynamic_rejected_count=0,
            validated_count=1,
            selected_witness=None,
            termination_reason="found",
            deterministic_objective=objective,
            elapsed_nonqualification_ns=0,
        )


def test_automated_witness_contract_names_are_search_not_evaluator_terms() -> None:
    names = {field.name for field in fields(AutomatedWitness)}

    assert names.isdisjoint(_FORBIDDEN_PROJECTION_NAMES)
    assert WitnessKind.PASS_LEFT.value == "pass_left"
    assert WitnessTerminalMode.REJOIN_DWELL.value == "rejoin_dwell"
