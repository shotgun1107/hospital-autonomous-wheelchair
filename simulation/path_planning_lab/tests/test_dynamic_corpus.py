from __future__ import annotations

from dataclasses import fields, replace

import numpy as np

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.dynamic_contracts import Point2D, Vector2D
from hospital_path_lab.dynamic_corpus import (
    LEGACY_V1_PUBLIC_CORPUS_HASH,
    DynamicAxisAlignedRegion,
    DynamicControllerCorpusInput,
    DynamicCorpusActor,
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    DynamicScenarioFamily,
    DynamicScenarioOrientation,
    DynamicStaticLayoutSpec,
    build_dynamic_grid_snapshot,
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
    generate_episode_observation_slots,
    paired_controller_inputs,
    paired_controller_snapshots,
    validate_dynamic_corpus,
    validate_dynamic_v6_public_corpus,
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


def test_legacy_v1_public_lane_keeps_exact_36_episode_hash() -> None:
    corpus = generate_dynamic_corpus()

    assert len(corpus) == 36
    assert canonical_content_hash(corpus) == LEGACY_V1_PUBLIC_CORPUS_HASH


def test_v6_public_matrix_is_deterministic_separate_and_valid() -> None:
    first = generate_dynamic_v6_public_corpus()
    second = generate_dynamic_v6_public_corpus()
    validation = validate_dynamic_v6_public_corpus(first)

    assert first == second
    assert validation.passed, validation.failures
    assert len(first) == validation.episode_count == 13
    assert validation.golden_count == 9
    assert validation.development_count == 4
    assert {episode.scenario_family for episode in first} == set(
        DynamicScenarioFamily
    )
    assert {episode.orientation for episode in first} == set(
        DynamicScenarioOrientation
    )
    seed_counts = {
        seed: sum(episode.seed == seed for episode in first)
        for seed in {episode.seed for episode in first}
    }
    assert sorted(seed_counts.values()) == [1] * 11 + [2]
    assert len({episode.semantic_world_hash for episode in first}) == len(first)
    assert all(episode.oracle_hash for episode in first)

    other_base = generate_dynamic_v6_public_corpus(base_seed=20260812)
    assert {
        (episode.variant, episode.seed) for episode in first
    }.isdisjoint((episode.variant, episode.seed) for episode in other_base)


def test_v6_metadata_and_oracle_never_enter_controller_input_contract() -> None:
    forbidden_names = {
        "split",
        "expectation_category",
        "scenario_family",
        "variant",
        "orientation",
        "latent_case_id",
        "static_layout_spec",
        "oracle_spec",
        "semantic_world_hash",
        "oracle_hash",
    }
    controller_fields = {field.name for field in fields(DynamicControllerCorpusInput)}
    assert controller_fields.isdisjoint(forbidden_names)

    for episode in generate_dynamic_v6_public_corpus():
        pp_input, dwa_input = paired_controller_inputs(episode)
        assert pp_input is dwa_input
        assert not hasattr(pp_input, "oracle_spec")
        assert not hasattr(pp_input, "scenario_family")
        delivered_frames = tuple(
            slot.frame
            for slot in pp_input.observation_slots
            if slot.frame is not None
        )
        track_identity_values = tuple(
            identity
            for frame in delivered_frames
            for track in frame.tracks
            for identity in (track.track_id, track.actor_binding_id)
        )
        expected_actor_ids = {
            f"dynamic-v6-actor-{index:03d}"
            for index in range(len(episode.actors))
        }
        assert {actor.actor_id for actor in episode.actors} == expected_actor_ids
        assert set(track_identity_values).issubset(expected_actor_ids)
        opaque_values = (
            pp_input.episode_id,
            episode.map_id,
            build_dynamic_grid_snapshot(episode).metadata.content_hash,
            *track_identity_values,
        )
        sensitive_tokens = {
            episode.split.value,
            episode.expectation_category.value,
            episode.scenario_family.value,
            episode.variant,
            episode.orientation.value,
            episode.latent_case_id,
        }
        assert all(
            token not in value
            for token in sensitive_tokens
            for value in opaque_values
        )


def test_v6_evaluator_metadata_relabel_does_not_change_controller_world() -> None:
    episode = generate_dynamic_v6_public_corpus()[0]
    relabeled = replace(
        episode,
        episode_id="admin-only-relabeled-episode",
        split=(
            DynamicCorpusSplit.DEVELOPMENT
            if episode.split is DynamicCorpusSplit.GOLDEN
            else DynamicCorpusSplit.GOLDEN
        ),
        scenario_family=DynamicScenarioFamily.MULTI_ACTOR,
        variant="admin-only-relabeled-variant",
        orientation=DynamicScenarioOrientation.INTERSECTION,
        latent_case_id="admin-only-relabeled-latent-case",
    )

    original_input, _ = paired_controller_inputs(episode)
    relabeled_input, _ = paired_controller_inputs(relabeled)
    original_grid = build_dynamic_grid_snapshot(episode)
    relabeled_grid = build_dynamic_grid_snapshot(relabeled)

    assert relabeled.semantic_world_hash == episode.semantic_world_hash
    assert relabeled_input.episode_id == original_input.episode_id
    assert relabeled_input.seed == original_input.seed
    assert relabeled_input.mission_id == original_input.mission_id
    assert relabeled_input.initial_state == original_input.initial_state
    assert relabeled_input.goal_pose == original_input.goal_pose
    assert relabeled_input.reference_path == original_input.reference_path
    assert relabeled_input.observation_slots == original_input.observation_slots
    assert (
        relabeled_input.observation_stream_hash
        == original_input.observation_stream_hash
    )
    assert relabeled_grid.metadata == original_grid.metadata
    assert relabeled_grid.forbidden_cells == original_grid.forbidden_cells
    assert np.array_equal(relabeled_grid.grid.occupancy, original_grid.grid.occupancy)


def test_v6_full_actor_trajectory_and_semantic_duplicate_are_rejected() -> None:
    corpus = list(generate_dynamic_v6_public_corpus())
    same_direction = corpus[0]
    original_actor = same_direction.actors[0]
    corpus[0] = replace(
        same_direction,
        actors=(
            DynamicCorpusActor(
                actor_id=original_actor.actor_id,
                active_from_s=original_actor.active_from_s,
                active_until_s=same_direction.duration_s,
                start_position=original_actor.start_position,
                velocity=Vector2D(0.50, 0.0),
            ),
        ),
    )
    corpus[1] = replace(
        corpus[1],
        semantic_world_hash=corpus[2].semantic_world_hash,
    )

    validation = validate_dynamic_v6_public_corpus(tuple(corpus))

    assert not validation.passed
    assert "duplicate_v6_semantic_world" in validation.failures
    assert any("trajectory_out_of_map" in failure for failure in validation.failures)


def test_v6_multi_actor_overlap_static_topology_and_normal_two_tracks() -> None:
    corpus = generate_dynamic_v6_public_corpus()
    corner = next(
        episode
        for episode in corpus
        if episode.scenario_family is DynamicScenarioFamily.CORNER_INTERSECTION
    )
    corner_grid = build_dynamic_grid_snapshot(corner)
    assert np.any(corner_grid.grid.occupancy)
    assert corner_grid.forbidden_cells

    simultaneous = next(
        episode for episode in corpus if episode.variant == "simultaneous-overlap"
    )
    slots = generate_episode_observation_slots(
        simultaneous,
        profile=NORMAL_OBSERVATION_PROFILE,
    )
    assert any(slot.frame is not None and len(slot.frame.tracks) == 2 for slot in slots)

    first, second = simultaneous.actors
    invalid = replace(
        simultaneous,
        actors=(
            first,
            DynamicCorpusActor(
                actor_id=second.actor_id,
                active_from_s=first.active_from_s,
                active_until_s=first.active_until_s,
                start_position=Point2D(
                    first.start_position.x,
                    first.start_position.y,
                ),
                velocity=first.velocity,
            ),
        ),
    )
    replaced = tuple(invalid if item is simultaneous else item for item in corpus)
    validation = validate_dynamic_v6_public_corpus(replaced)
    assert not validation.passed
    assert any("actor_trajectory_overlap" in item for item in validation.failures)


def test_v6_actor_trajectory_uses_the_rasterized_static_cell_extent() -> None:
    corpus = generate_dynamic_v6_public_corpus()
    head_on = next(
        episode
        for episode in corpus
        if episode.scenario_family is DynamicScenarioFamily.HEAD_ON
    )
    actor = head_on.actors[0]
    raster_boundary_actor = replace(
        actor,
        start_position=Point2D(actor.start_position.x, 1.421),
    )
    layout = DynamicStaticLayoutSpec(
        occupied_regions=(
            DynamicAxisAlignedRegion(
                min_x_m=0.8,
                min_y_m=1.604,
                max_x_m=4.2,
                max_y_m=1.624,
            ),
        )
    )
    mutated = replace(
        head_on,
        actors=(raster_boundary_actor,),
        static_layout_spec=layout,
    )
    mutated_corpus = tuple(
        mutated if episode is head_on else episode for episode in corpus
    )

    validation = validate_dynamic_v6_public_corpus(mutated_corpus)

    assert any(
        "trajectory_intersects_static_layout" in failure
        for failure in validation.failures
    )


def test_v6_feasible_witness_and_rigid_transform_pair_are_evaluator_only() -> None:
    corpus = generate_dynamic_v6_public_corpus()
    feasible = tuple(
        item
        for item in corpus
        if item.expectation_category
        is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
    )
    assert len(feasible) == 5
    assert all(item.oracle_spec.feasible_witness is not None for item in feasible)

    rigid_pair = tuple(
        item for item in corpus if item.latent_case_id == "diagonal-rigid-pair-v6"
    )
    assert len(rigid_pair) == 2
    assert {item.orientation for item in rigid_pair} == {
        DynamicScenarioOrientation.DIAGONAL,
        DynamicScenarioOrientation.VERTICAL,
    }
    assert rigid_pair[0].semantic_world_hash != rigid_pair[1].semantic_world_hash
    assert rigid_pair[0].seed == rigid_pair[1].seed


def test_v6_feasible_witness_is_dense_and_rejects_nonholonomic_tampering() -> None:
    corpus = generate_dynamic_v6_public_corpus()
    feasible = next(
        item
        for item in corpus
        if item.expectation_category
        is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
    )
    witness = feasible.oracle_spec.feasible_witness
    assert witness is not None
    assert witness.points[-1].time_s <= feasible.duration_s
    assert all(
        abs(right.time_s - left.time_s - 0.05) <= 1e-12
        for left, right in zip(witness.points, witness.points[1:], strict=False)
    )
    assert len(witness.points) > 800

    tampered_points = list(witness.points)
    target_index = len(tampered_points) // 3
    target = tampered_points[target_index]
    tampered_points[target_index] = replace(
        target,
        pose=Pose2D(target.pose.x, target.pose.y + 0.05, target.pose.yaw),
    )
    tampered_witness = replace(witness, points=tuple(tampered_points))
    tampered_oracle = replace(
        feasible.oracle_spec,
        feasible_witness=tampered_witness,
    )
    tampered_episode = replace(
        feasible,
        oracle_spec=tampered_oracle,
        oracle_hash=canonical_content_hash(tampered_oracle),
    )
    tampered_corpus = tuple(
        tampered_episode if item is feasible else item for item in corpus
    )

    validation = validate_dynamic_v6_public_corpus(tampered_corpus)
    assert not validation.passed
    assert any(
        "feasible_witness_kinematic_mismatch" in failure
        for failure in validation.failures
    )


def test_v6_witness_time_grid_boundary_and_rigid_duration_are_fail_closed() -> None:
    corpus = generate_dynamic_v6_public_corpus()
    feasible = next(
        item
        for item in corpus
        if item.expectation_category
        is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
    )
    witness = feasible.oracle_spec.feasible_witness
    assert witness is not None
    shifted_witness = replace(
        witness,
        points=tuple(
            replace(point, time_s=point.time_s + 0.50) for point in witness.points
        ),
    )
    shifted_oracle = replace(feasible.oracle_spec, feasible_witness=shifted_witness)
    shifted_episode = replace(
        feasible,
        oracle_spec=shifted_oracle,
        oracle_hash=canonical_content_hash(shifted_oracle),
    )
    shifted_corpus = tuple(
        shifted_episode if item is feasible else item for item in corpus
    )
    shifted_validation = validate_dynamic_v6_public_corpus(shifted_corpus)
    assert any(
        "feasible_witness_must_start_at_zero" in failure
        for failure in shifted_validation.failures
    )

    misaligned = replace(feasible, corridor_width_m=feasible.corridor_width_m + 0.002)
    misaligned_corpus = tuple(
        misaligned if item is feasible else item for item in corpus
    )
    misaligned_validation = validate_dynamic_v6_public_corpus(misaligned_corpus)
    assert any(
        "grid_dimension_not_resolution_aligned" in failure
        for failure in misaligned_validation.failures
    )

    vertical = next(
        item
        for item in corpus
        if item.latent_case_id == "diagonal-rigid-pair-v6"
        and item.orientation is DynamicScenarioOrientation.VERTICAL
    )
    shortened = replace(vertical, duration_s=vertical.duration_s - 1.0)
    shortened_corpus = tuple(
        shortened if item is vertical else item for item in corpus
    )
    shortened_validation = validate_dynamic_v6_public_corpus(shortened_corpus)
    assert "rigid_transform_geometry_mismatch" in shortened_validation.failures


def test_v6_rigid_pair_rotates_the_same_observation_noise_draws() -> None:
    pair = tuple(
        item
        for item in generate_dynamic_v6_public_corpus()
        if item.latent_case_id == "diagonal-rigid-pair-v6"
    )
    horizontal = next(
        item
        for item in pair
        if item.orientation is DynamicScenarioOrientation.DIAGONAL
    )
    vertical = next(
        item
        for item in pair
        if item.orientation is DynamicScenarioOrientation.VERTICAL
    )
    horizontal_slots = generate_episode_observation_slots(
        horizontal,
        profile=STRESS_OBSERVATION_PROFILE,
    )
    vertical_slots = generate_episode_observation_slots(
        vertical,
        profile=STRESS_OBSERVATION_PROFILE,
    )
    assert len(horizontal_slots) == len(vertical_slots)
    observed_track_count = 0
    for horizontal_slot, vertical_slot in zip(
        horizontal_slots,
        vertical_slots,
        strict=True,
    ):
        assert (horizontal_slot.frame is None) == (vertical_slot.frame is None)
        if horizontal_slot.frame is None or vertical_slot.frame is None:
            continue
        assert len(horizontal_slot.frame.tracks) == len(vertical_slot.frame.tracks)
        for horizontal_track, vertical_track in zip(
            horizontal_slot.frame.tracks,
            vertical_slot.frame.tracks,
            strict=True,
        ):
            observed_track_count += 1
            assert abs(
                vertical_track.observed_position.x
                - (horizontal.map_length_m - horizontal_track.observed_position.y)
            ) <= 1e-12
            assert abs(
                vertical_track.observed_position.y
                - horizontal_track.observed_position.x
            ) <= 1e-12
            assert abs(
                vertical_track.observed_velocity.x
                + horizontal_track.observed_velocity.y
            ) <= 1e-12
            assert abs(
                vertical_track.observed_velocity.y
                - horizontal_track.observed_velocity.x
            ) <= 1e-12
    assert observed_track_count > 0
