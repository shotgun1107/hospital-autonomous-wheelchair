from __future__ import annotations

from dataclasses import fields, replace

import numpy as np

from hospital_path_lab.dynamic_corpus import (
    DynamicControllerCorpusInput,
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    build_dynamic_grid_snapshot,
    generate_dynamic_corpus,
    generate_episode_observation_slots,
    paired_controller_inputs,
    paired_controller_snapshots,
    validate_dynamic_corpus,
)
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
)
from hospital_path_lab.followers import DynamicPurePursuitController
from hospital_path_lab.local_algorithms import DynamicDwaController
from hospital_path_lab.map_factory import canonical_content_hash


def test_golden_and_development_corpus_are_balanced_and_valid() -> None:
    corpus = generate_dynamic_corpus()
    validation = validate_dynamic_corpus(corpus)

    assert validation.passed, validation.failures
    assert validation.golden_count == 6
    assert validation.development_count == 30
    assert len(corpus) == 36
    assert dict(validation.category_counts) == {
        category.value: 6 for category in DynamicExpectationCategory
    }
    assert sum(item.split is DynamicCorpusSplit.GOLDEN for item in corpus) == 6


def test_same_seed_reproduces_corpus_map_and_observation_streams() -> None:
    first = generate_dynamic_corpus(base_seed=61)
    second = generate_dynamic_corpus(base_seed=61)

    assert first == second
    assert canonical_content_hash(first) == canonical_content_hash(second)
    for left, right in zip(first, second, strict=True):
        left_grid = build_dynamic_grid_snapshot(left)
        right_grid = build_dynamic_grid_snapshot(right)
        assert left_grid.metadata == right_grid.metadata
        assert left_grid.forbidden_cells == right_grid.forbidden_cells
        assert np.array_equal(left_grid.grid.occupancy, right_grid.grid.occupancy)
        for profile in (NORMAL_OBSERVATION_PROFILE, STRESS_OBSERVATION_PROFILE):
            assert canonical_content_hash(
                generate_episode_observation_slots(left, profile=profile)
            ) == canonical_content_hash(
                generate_episode_observation_slots(right, profile=profile)
            )


def test_pp_and_dwa_receive_the_exact_same_label_free_paired_input() -> None:
    field_names = {field.name for field in fields(DynamicControllerCorpusInput)}
    forbidden_names = {"split", "expectation_category", "oracle", "scenario_label"}
    assert field_names.isdisjoint(forbidden_names)

    for episode in generate_dynamic_corpus():
        pp_input, dwa_input = paired_controller_inputs(episode)
        assert pp_input is dwa_input
        assert pp_input.observation_stream_hash == dwa_input.observation_stream_hash


def test_both_controllers_replay_each_golden_first_snapshot_with_same_provenance() -> None:
    golden = tuple(
        episode
        for episode in generate_dynamic_corpus()
        if episode.split is DynamicCorpusSplit.GOLDEN
    )
    for episode in golden:
        pp_snapshot, dwa_snapshot = paired_controller_snapshots(episode)
        pp_result = DynamicPurePursuitController().step(pp_snapshot)
        dwa_result = DynamicDwaController().step(dwa_snapshot)
        assert pp_snapshot is dwa_snapshot
        assert pp_result.input_content_hash == dwa_result.input_content_hash
        assert pp_result.observation_content_hash == dwa_result.observation_content_hash


def test_corpus_validator_rejects_duplicate_and_invalid_category_geometry() -> None:
    corpus = generate_dynamic_corpus()
    duplicate = corpus + (corpus[0],)
    duplicate_validation = validate_dynamic_corpus(duplicate)
    assert not duplicate_validation.passed
    assert "duplicate_episode_id" in duplicate_validation.failures
    assert "duplicate_episode_hash" in duplicate_validation.failures

    feasible_index = next(
        index
        for index, episode in enumerate(corpus)
        if episode.expectation_category
        is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
    )
    invalid = list(corpus)
    invalid[feasible_index] = replace(invalid[feasible_index], corridor_width_m=1.0)
    invalid_validation = validate_dynamic_corpus(tuple(invalid))
    assert not invalid_validation.passed
    assert any(
        failure.endswith("detour_geometry_too_narrow")
        for failure in invalid_validation.failures
    )
