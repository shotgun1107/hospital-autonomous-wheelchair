from __future__ import annotations

import ast
import inspect
from dataclasses import replace

import pytest

import hospital_path_lab.r2b_entry_coverage as coverage_module
from hospital_path_lab.dynamic_contracts import DYNAMIC_CONTROL_PERIOD_S
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_witness_contracts import project_public_witness_world
from hospital_path_lab.dynamic_witness_restop import search_multi_hazard_restop
from hospital_path_lab.dynamic_witness_search import generate_wait_and_follow_witness
from hospital_path_lab.dynamic_witness_validation import validate_ground_truth_witness
from hospital_path_lab.r2b_entry_coverage import (
    derive_r2b_covered_world,
    replay_r2b_entry_coverage,
)


@pytest.fixture(scope="module")
def v6_replay():
    episode = next(
        item
        for item in generate_dynamic_v6_public_corpus()
        if item.variant == "second-risk-after-corner"
    )
    world = project_public_witness_world(episode)
    first_entry_tick = round(world.actors[0].active_from_s / DYNAMIC_CONTROL_PERIOD_S)
    witness = generate_wait_and_follow_witness(
        world,
        departure_tick=first_entry_tick,
        linear_target_mps=0.20,
    )
    assert witness is not None
    validation = validate_ground_truth_witness(world, witness)
    assert validation.passed, validation.failures
    return world, replay_r2b_entry_coverage(world, witness, validation)


@pytest.fixture(scope="module")
def legacy_replay():
    episode = next(
        item
        for item in generate_dynamic_corpus()
        if item.split is DynamicCorpusSplit.GOLDEN
        and item.expectation_category is DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP
    )
    world = project_public_witness_world(episode)
    search = search_multi_hazard_restop(world)
    assert search.witness is not None
    assert search.validation is not None
    assert search.validation.base_validation.passed
    return world, replay_r2b_entry_coverage(
        world,
        search.witness,
        search.validation.base_validation,
    )


@pytest.mark.parametrize("fixture_name", ("v6_replay", "legacy_replay"))
def test_original_failure_is_preserved_and_covered_ideal_has_no_miss(
    fixture_name: str,
    request: pytest.FixtureRequest,
) -> None:
    _world, replay = request.getfixturevalue(fixture_name)
    source_ideal = replay.source_profiles.results[0]
    covered_ideal = replay.covered_profiles.results[0]

    assert source_ideal.hard_failures == ("ideal_capsule_ground_truth_miss",)
    assert source_ideal.actual_actor_containment_miss_count > 0
    assert covered_ideal.hard_failures == ()
    assert covered_ideal.actual_actor_containment_miss_count == 0
    assert covered_ideal.maximum_actor_containment_miss_m == 0.0
    assert replay.source_world_content_hash != replay.covered_world_content_hash
    assert "original_r2b_negative_world_preserved" in replay.limitations


@pytest.mark.parametrize("fixture_name", ("v6_replay", "legacy_replay"))
def test_covered_actor_is_continuous_at_every_entry(
    fixture_name: str,
    request: pytest.FixtureRequest,
) -> None:
    world, replay = request.getfixturevalue(fixture_name)
    covered = derive_r2b_covered_world(world, replay.contract)
    source_by_id = {actor.actor_binding_id: actor for actor in world.actors}
    covered_by_id = {actor.actor_binding_id: actor for actor in covered.actors}

    for approach in replay.contract.approaches:
        source_actor = source_by_id[approach.actor_binding_id]
        covered_actor = covered_by_id[approach.actor_binding_id]
        assert covered_actor.active_from_s == approach.monitored_from_s == 0.0
        assert source_actor.active_from_s == approach.entry_time_s
        source_state = source_actor.state_at(approach.entry_time_s)
        covered_state = covered_actor.state_at(approach.entry_time_s)
        assert source_state is not None
        assert covered_state is not None
        assert covered_state.position.x == pytest.approx(source_state.position.x)
        assert covered_state.position.y == pytest.approx(source_state.position.y)
        assert covered_state.velocity == source_state.velocity
        assert covered_state.radius_m == source_state.radius_m
        assert covered_state.trajectory_revision == source_state.trajectory_revision
        assert (
            approach.entry_time_s - approach.monitored_from_s
            >= replay.contract.required_lead_time_s
        )


def test_missing_delayed_actor_coverage_is_rejected(v6_replay) -> None:
    world, replay = v6_replay
    missing = replace(
        replay.contract,
        approaches=replay.contract.approaches[:-1],
    )

    with pytest.raises(ValueError, match="every and only delayed Actor"):
        derive_r2b_covered_world(world, missing)


def test_insufficient_entry_lead_is_rejected(v6_replay) -> None:
    _world, replay = v6_replay
    approach = replay.contract.approaches[0]
    too_late = replace(
        approach,
        monitored_from_s=approach.entry_time_s - 1.0,
    )

    with pytest.raises(ValueError, match="enough prediction lead time"):
        replace(replay.contract, approaches=(too_late, *replay.contract.approaches[1:]))


def test_tampered_approach_position_is_rejected_at_entry(v6_replay) -> None:
    world, replay = v6_replay
    approach = replay.contract.approaches[0]
    shifted = replace(
        approach,
        monitored_start_x_m=approach.monitored_start_x_m + 0.01,
    )
    tampered = replace(
        replay.contract,
        approaches=(shifted, *replay.contract.approaches[1:]),
    )

    with pytest.raises(ValueError, match="discontinuous at the entry boundary"):
        derive_r2b_covered_world(world, tampered)


def test_contract_for_a_different_world_is_rejected(v6_replay, legacy_replay) -> None:
    v6_world, _v6 = v6_replay
    _legacy_world, legacy = legacy_replay

    with pytest.raises(ValueError, match="source identity mismatch"):
        derive_r2b_covered_world(v6_world, legacy.contract)


def test_entry_coverage_module_has_no_controller_or_evaluator_label_dependency() -> None:
    tree = ast.parse(inspect.getsource(coverage_module))
    forbidden = {
        "controller",
        "expectation_category",
        "oracle_spec",
        "hidden",
    }
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            names.add(node.id)
        elif isinstance(node, ast.Attribute):
            names.add(node.attr)

    assert not (forbidden & names)
