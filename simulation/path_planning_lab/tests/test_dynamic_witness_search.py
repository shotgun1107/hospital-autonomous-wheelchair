from __future__ import annotations

import ast
import inspect
from dataclasses import replace

import pytest

import hospital_path_lab.dynamic_witness_search as witness_search_module
from hospital_path_lab.contracts import Twist2D
from hospital_path_lab.dynamic_corpus import (
    DynamicExpectationCategory,
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_witness_contracts import (
    FROZEN_WITNESS_SEARCH_CONFIG,
    WitnessKind,
    WitnessPhase,
    WitnessSearchStatus,
    WitnessTerminalMode,
    build_automated_witness,
    project_public_witness_world,
)
from hospital_path_lab.dynamic_witness_search import (
    generate_hold_only_witness,
    generate_wait_and_follow_witness,
    search_wait_and_hold,
)
from hospital_path_lab.dynamic_witness_validation import (
    validate_ground_truth_witness,
)


@pytest.fixture(scope="module")
def representative_public_searches():
    episodes = generate_dynamic_v6_public_corpus()
    tokens = (
        "offset-head-on",
        "corner-intersection",
        "simultaneous-two-actor",
    )
    selected = {}
    for token in tokens:
        episode = next(episode for episode in episodes if token in episode.episode_id)
        world = project_public_witness_world(episode)
        selected[token] = (episode, world, search_wait_and_hold(world))
    return selected


def _legacy_public_world(category: DynamicExpectationCategory):
    episode = next(
        episode
        for episode in generate_dynamic_corpus()
        if episode.expectation_category is category
        and episode.split.value == "golden"
    )
    return project_public_witness_world(episode)


def _without_initial_wait(world, candidate):
    skipped_wait = tuple(
        replace(
            point,
            time_s=round(point.time_s - 0.50, 12),
            phase=(WitnessPhase.START if index == 0 else point.phase),
        )
        for index, point in enumerate(candidate.points[10:])
    )
    return build_automated_witness(
        world,
        witness_id="adversarial-terminal-dwell-only-wait",
        kind=WitnessKind.WAIT_AND_FOLLOW,
        terminal_mode=WitnessTerminalMode.GOAL_DWELL,
        points=skipped_wait,
        terminal_dwell_s=0.50,
    )


def test_full_duration_hold_passes_but_short_hold_is_rejected(
    representative_public_searches,
) -> None:
    _, world, _ = representative_public_searches["offset-head-on"]
    full_hold = generate_hold_only_witness(world)

    full_result = validate_ground_truth_witness(world, full_hold)
    short_result = validate_ground_truth_witness(
        world,
        replace(full_hold, points=full_hold.points[:11]),
    )

    assert full_hold.points[-1].time_s == pytest.approx(world.duration_s)
    assert full_result.passed, full_result.failures
    assert not short_result.passed
    assert "safe_hold_does_not_cover_world" in short_result.failures


def test_terminal_dwell_alone_does_not_count_as_actual_wait() -> None:
    world = _legacy_public_world(DynamicExpectationCategory.OBSERVATION_INVALID)
    candidate = generate_wait_and_follow_witness(
        world,
        departure_tick=0,
        linear_target_mps=0.20,
    )

    assert candidate is not None
    no_wait_candidate = _without_initial_wait(world, candidate)
    result = validate_ground_truth_witness(world, no_wait_candidate)

    assert result.metrics.terminal_dwell_observed_s >= 0.50
    assert result.metrics.final_goal_distance_m <= 0.05
    assert not result.passed
    assert "wait_follow_has_no_actual_wait" in result.failures


def test_wait_after_all_progress_does_not_satisfy_wait_then_follow_order() -> None:
    world = _legacy_public_world(DynamicExpectationCategory.OBSERVATION_INVALID)
    generated = generate_wait_and_follow_witness(
        world,
        departure_tick=0,
        linear_target_mps=0.20,
    )
    assert generated is not None
    no_initial_wait = _without_initial_wait(world, generated)
    terminal_index = next(
        index
        for index, point in enumerate(no_initial_wait.points)
        if point.phase is WitnessPhase.TERMINAL_DWELL
    )
    prefix = no_initial_wait.points[:terminal_index]
    terminal = no_initial_wait.points[terminal_index:]
    stopped = prefix[-1]
    inserted_wait = tuple(
        replace(
            stopped,
            time_s=round(stopped.time_s + tick * 0.05, 12),
            phase=WitnessPhase.WAIT,
        )
        for tick in range(1, 11)
    )
    delayed_terminal = tuple(
        replace(point, time_s=round(point.time_s + 0.50, 12))
        for point in terminal
    )
    wait_at_goal = build_automated_witness(
        world,
        witness_id="adversarial-wait-after-all-progress",
        kind=WitnessKind.WAIT_AND_FOLLOW,
        terminal_mode=WitnessTerminalMode.GOAL_DWELL,
        points=(*prefix, *inserted_wait, *delayed_terminal),
        terminal_dwell_s=0.50,
    )

    result = validate_ground_truth_witness(world, wait_at_goal)

    assert not result.passed
    assert "wait_follow_did_not_progress_after_wait" in result.failures


def test_all_stop_witness_cannot_be_relabelled_as_wait_and_follow(
    representative_public_searches,
) -> None:
    _, world, _ = representative_public_searches["offset-head-on"]
    hold = generate_hold_only_witness(world)
    fake_wait = build_automated_witness(
        world,
        witness_id="adversarial-all-stop-fake-wait",
        kind=WitnessKind.WAIT_AND_FOLLOW,
        terminal_mode=WitnessTerminalMode.REJOIN_DWELL,
        points=hold.points,
        terminal_dwell_s=0.50,
    )

    result = validate_ground_truth_witness(world, fake_wait)

    assert not result.passed
    assert "wait_follow_has_no_forward_progress" in result.failures


def test_search_module_has_no_dynamic_corpus_or_evaluator_label_import() -> None:
    tree = ast.parse(inspect.getsource(witness_search_module))
    imported_modules = {
        module_name
        for node in ast.walk(tree)
        for module_name in (
            (
                *(alias.name for alias in node.names),
                *((node.module,) if isinstance(node, ast.ImportFrom) else ()),
            )
            if isinstance(node, (ast.Import, ast.ImportFrom))
            else ()
        )
        if module_name is not None
    }
    identifiers = {
        node.id for node in ast.walk(tree) if isinstance(node, ast.Name)
    } | {
        node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
    }

    assert not any("dynamic_corpus" in name for name in imported_modules)
    assert identifiers.isdisjoint(
        {
            "expectation_category",
            "oracle_spec",
            "feasible_witness",
            "scenario_family",
            "orientation",
            "latent_case_id",
            "progressable",
            "blocking_cleared_at_s",
        }
    )


def test_evaluator_label_and_oracle_changes_do_not_change_search_result(
    representative_public_searches,
) -> None:
    episode, original_world, original = representative_public_searches[
        "offset-head-on"
    ]
    changed_oracle = replace(
        episode.oracle_spec,
        expectation_category=DynamicExpectationCategory.NO_SAFE_SOLUTION,
        hazard_intervals_s=((0.25, 0.75),),
        same_direction_actor_ids=(),
        required_protective_stop_epochs=99,
        feasible_witness=None,
    )
    evaluator_only_tamper = replace(
        episode,
        expectation_category=DynamicExpectationCategory.NO_SAFE_SOLUTION,
        progressable=not episode.progressable,
        blocking_cleared_at_s=None,
        observation_fault="forged-evaluator-only-fault",
        oracle_spec=changed_oracle,
        oracle_hash="forged-evaluator-only-oracle-hash",
    )
    changed_world = project_public_witness_world(evaluator_only_tamper)
    changed = search_wait_and_hold(changed_world)

    assert changed_world == original_world
    assert changed.semantic_content_hash == original.semantic_content_hash
    assert changed.selected_witness == original.selected_witness
    assert changed.deterministic_objective == original.deterministic_objective


def test_result_counts_validation_hash_and_semantics_are_deterministic(
    representative_public_searches,
) -> None:
    _, world, result = representative_public_searches["offset-head-on"]
    selected = result.selected_witness

    assert result.status is WitnessSearchStatus.WITNESS_FOUND
    assert selected is not None
    assert result.generated_count == (
        result.geometry_pruned_count
        + result.dynamic_rejected_count
        + result.validated_count
    )
    independent = validate_ground_truth_witness(world, selected)
    assert independent.passed, independent.failures
    assert result.selected_validation_hash == independent.content_hash
    assert result.semantic_content_hash == replace(
        result,
        elapsed_nonqualification_ns=result.elapsed_nonqualification_ns + 1_000_000,
    ).semantic_content_hash


def test_small_timed_candidate_limit_is_inconclusive_resource_limit() -> None:
    episode = next(
        episode
        for episode in generate_dynamic_v6_public_corpus()
        if "offset-head-on" in episode.episode_id
    )
    limited_config = replace(
        FROZEN_WITNESS_SEARCH_CONFIG,
        max_timed_candidates_per_episode=1,
    )
    world = project_public_witness_world(episode, search_config=limited_config)

    first = search_wait_and_hold(world, search_config=limited_config)
    second = search_wait_and_hold(world, search_config=limited_config)

    assert first.status is WitnessSearchStatus.RESOURCE_LIMIT
    assert first.selected_witness is None
    assert first.deterministic_objective is None
    assert first.selected_validation_hash is None
    assert first.termination_reason == "timed_candidate_limit_reached"
    assert first.generated_count == (
        first.geometry_pruned_count
        + first.dynamic_rejected_count
        + first.validated_count
    )
    assert second.semantic_content_hash == first.semantic_content_hash


def test_nonzero_initial_twist_does_not_let_hold_abort_valid_wait_search() -> None:
    episode = next(
        episode
        for episode in generate_dynamic_corpus()
        if episode.expectation_category
        is DynamicExpectationCategory.OBSERVATION_INVALID
        and episode.split.value == "golden"
    )
    moving_episode = replace(
        episode,
        initial_state=replace(
            episode.initial_state,
            twist=Twist2D(linear=0.05),
        ),
    )
    world = project_public_witness_world(moving_episode)

    result = search_wait_and_hold(world)

    assert result.status is WitnessSearchStatus.WITNESS_FOUND
    assert result.selected_witness is not None
    assert result.selected_witness.kind is WitnessKind.WAIT_AND_FOLLOW
    assert validate_ground_truth_witness(world, result.selected_witness).passed


def test_initial_twist_above_vehicle_limit_is_invalid_input() -> None:
    episode = next(
        episode
        for episode in generate_dynamic_corpus()
        if episode.expectation_category
        is DynamicExpectationCategory.OBSERVATION_INVALID
        and episode.split.value == "golden"
    )
    invalid_episode = replace(
        episode,
        initial_state=replace(
            episode.initial_state,
            twist=Twist2D(linear=0.31),
        ),
    )
    world = project_public_witness_world(invalid_episode)

    result = search_wait_and_hold(world)

    assert result.status is WitnessSearchStatus.INVALID_INPUT
    assert result.termination_reason == "initial_linear_speed_exceeds_vehicle_profile"
    assert result.generated_count == 0


def test_effective_departure_ticks_are_deduplicated_and_hashed() -> None:
    world = _legacy_public_world(DynamicExpectationCategory.OBSERVATION_INVALID)

    result = search_wait_and_hold(world)

    assert result.status is WitnessSearchStatus.WITNESS_FOUND
    assert result.generated_count == (
        len(FROZEN_WITNESS_SEARCH_CONFIG.linear_targets_mps) + 1
    )
    assert result.deterministic_objective is not None
    assert result.deterministic_objective.frozen_parameter_tuple[0] == 10.0
    assert result.selected_witness is not None
    first_moving = next(
        point
        for point in result.selected_witness.points
        if abs(point.twist.linear) > 1e-12
    )
    assert first_moving.time_s == pytest.approx(0.55)


@pytest.mark.parametrize(
    "case_token",
    (
        "offset-head-on",
        "corner-intersection",
        "simultaneous-two-actor",
    ),
)
def test_public_straight_corner_and_multi_actor_candidate_pass_independent_validator(
    representative_public_searches,
    case_token: str,
) -> None:
    _, world, result = representative_public_searches[case_token]

    assert result.status is WitnessSearchStatus.WITNESS_FOUND
    assert result.selected_witness is not None
    assert result.selected_witness.kind is WitnessKind.WAIT_AND_FOLLOW
    independent = validate_ground_truth_witness(world, result.selected_witness)
    assert independent.passed, independent.failures
    assert result.selected_validation_hash == independent.content_hash
