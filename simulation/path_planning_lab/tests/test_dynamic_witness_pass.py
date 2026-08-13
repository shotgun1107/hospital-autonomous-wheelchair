from __future__ import annotations

import ast
import inspect
from dataclasses import replace
from math import pi

import pytest

import hospital_path_lab.dynamic_witness_pass as pass_module
from hospital_path_lab.contracts import Pose2D, RobotState
from hospital_path_lab.dynamic_contracts import Point2D, Vector2D
from hospital_path_lab.dynamic_corpus import generate_dynamic_v6_public_corpus
from hospital_path_lab.dynamic_witness_contracts import (
    FROZEN_WITNESS_SEARCH_CONFIG,
    ManeuverConstraintSpec,
    PassingPolicy,
    PassSide,
    PassSideWaitPolicy,
    WitnessKind,
    WitnessSearchStatus,
    project_public_witness_world,
)
from hospital_path_lab.dynamic_witness_pass import (
    PassCandidateRequest,
    generate_frozen_frontier_pass_candidate,
    generate_pass_candidate,
    search_pass_structured,
    search_pass_structured_parallel,
)
from hospital_path_lab.dynamic_witness_validation import validate_ground_truth_witness


def _public_episode(latent_case_id: str):
    matches = tuple(
        episode
        for episode in generate_dynamic_v6_public_corpus()
        if episode.latent_case_id == latent_case_id
    )
    assert len(matches) == 1
    return matches[0]


def _allowed_world(latent_case_id: str, *, search_config=FROZEN_WITNESS_SEARCH_CONFIG):
    return project_public_witness_world(
        _public_episode(latent_case_id),
        maneuver_constraints=ManeuverConstraintSpec(passing_policy=PassingPolicy.ALLOWED),
        search_config=search_config,
    )


@pytest.mark.parametrize(
    ("side", "kind"),
    (
        (PassSide.LEFT, WitnessKind.PASS_LEFT),
        (PassSide.RIGHT, WitnessKind.PASS_RIGHT),
    ),
)
def test_r00_direct_structured_candidate_passes_canonical_strict_validation(
    side: PassSide,
    kind: WitnessKind,
) -> None:
    world = _allowed_world("same-direction-wide-r00")

    witness = generate_pass_candidate(
        world,
        PassCandidateRequest(
            actor_binding_id=world.actors[0].actor_binding_id,
            side=side,
            departure_progress_m=0.0,
            lateral_offset_m=0.45,
            release_tick=0,
            linear_target_mps=0.30,
            angular_magnitude_radps=0.80,
            wait_policy=PassSideWaitPolicy.IMMEDIATE,
        ),
    )

    assert witness is not None
    assert witness.kind is kind
    assert witness.departure_time_s is not None
    assert witness.pass_times_by_actor
    assert witness.rejoin_started_at_s is not None
    assert witness.rejoin_confirmed_at_s is not None
    assert witness.departure_time_s < witness.pass_times_by_actor[0][1]
    assert witness.pass_times_by_actor[0][1] < witness.rejoin_confirmed_at_s
    strict = validate_ground_truth_witness(
        world,
        witness,
        strict_declarations=True,
    )
    assert strict.passed, strict.failures
    assert strict.metrics.minimum_actor_clearance_m is not None
    assert strict.metrics.minimum_actor_clearance_m >= 0.08
    assert strict.metrics.terminal_dwell_observed_s >= 0.50


@pytest.mark.parametrize(
    "latent_case_id",
    tuple(f"same-direction-wide-r{replica:02d}" for replica in range(5)),
)
@pytest.mark.parametrize("side", (PassSide.LEFT, PassSide.RIGHT))
def test_geometry_derived_frozen_frontier_passes_all_public_wide_cases(
    latent_case_id: str,
    side: PassSide,
) -> None:
    world = _allowed_world(latent_case_id)

    witness = generate_frozen_frontier_pass_candidate(
        world,
        actor_binding_id=world.actors[0].actor_binding_id,
        side=side,
    )

    assert witness is not None
    strict = validate_ground_truth_witness(world, witness, strict_declarations=True)
    assert strict.passed, strict.failures


def test_narrow_same_direction_has_no_pass_geometry() -> None:
    world = _allowed_world("same-direction-narrow-v6")

    result = search_pass_structured(world)

    assert result.best_pass_left is None
    assert result.best_pass_right is None
    assert result.left.status is WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE
    assert result.right.status is WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE
    assert result.left.counts.generated_count == 0
    assert result.right.counts.generated_count == 0


def test_head_on_actor_is_excluded_before_candidate_generation() -> None:
    world = _allowed_world("offset-head-on-v6")

    result = search_pass_structured(world)

    assert result.left.reason == "no_eligible_same_direction_target"
    assert result.right.reason == "no_eligible_same_direction_target"
    assert result.left.counts.generated_count == 0
    assert result.right.counts.generated_count == 0


def test_explicit_prohibited_policy_generates_no_pass_candidate() -> None:
    world = project_public_witness_world(
        _public_episode("same-direction-wide-r00"),
        maneuver_constraints=ManeuverConstraintSpec(
            passing_policy=PassingPolicy.PROHIBITED,
        ),
    )

    result = search_pass_structured(world)

    assert result.left.reason == "passing_policy_prohibited"
    assert result.right.reason == "passing_policy_prohibited"
    assert result.left.counts.generated_count == 0
    assert result.right.counts.generated_count == 0


def test_symmetric_public_world_has_mirrored_direct_pass_metrics() -> None:
    world = _allowed_world("same-direction-wide-r00")
    witnesses = tuple(
        generate_frozen_frontier_pass_candidate(
            world,
            actor_binding_id=world.actors[0].actor_binding_id,
            side=side,
        )
        for side in (PassSide.LEFT, PassSide.RIGHT)
    )

    assert all(witness is not None for witness in witnesses)
    validations = tuple(
        validate_ground_truth_witness(
            world,
            witness,
            strict_declarations=True,
        )
        for witness in witnesses
        if witness is not None
    )
    assert len(validations) == 2
    left, right = validations
    assert left.passed and right.passed
    assert left.metrics.actual_path_length_m == pytest.approx(
        right.metrics.actual_path_length_m,
    )
    assert left.metrics.maximum_reference_deviation_m == pytest.approx(
        right.metrics.maximum_reference_deviation_m,
    )
    assert left.metrics.absolute_angular_travel_rad == pytest.approx(
        right.metrics.absolute_angular_travel_rad,
    )
    assert left.metrics.minimum_actor_clearance_m == pytest.approx(
        right.metrics.minimum_actor_clearance_m,
    )


def test_allowed_region_can_reject_only_left_pass() -> None:
    episode = _public_episode("same-direction-wide-r00")
    unrestricted = _allowed_world("same-direction-wide-r00")
    upper_limit_m = episode.initial_state.pose.y + 0.31
    allowed_cells = tuple(
        (x, y)
        for y in range(unrestricted.grid.height)
        for x in range(unrestricted.grid.width)
        if unrestricted.grid.origin_y_m
        + (y + 0.5) * unrestricted.grid.resolution_m
        <= upper_limit_m
    )
    world = project_public_witness_world(
        episode,
        maneuver_constraints=ManeuverConstraintSpec(
            passing_policy=PassingPolicy.ALLOWED,
            allowed_cells=allowed_cells,
        ),
    )

    left = generate_frozen_frontier_pass_candidate(
        world,
        actor_binding_id=world.actors[0].actor_binding_id,
        side=PassSide.LEFT,
    )
    right = generate_frozen_frontier_pass_candidate(
        world,
        actor_binding_id=world.actors[0].actor_binding_id,
        side=PassSide.RIGHT,
    )

    assert left is None
    assert right is not None
    assert validate_ground_truth_witness(
        world,
        right,
        strict_declarations=True,
    ).passed


def test_vertical_reference_uses_signed_left_right_cell_center_offsets() -> None:
    source = _public_episode("same-direction-wide-r00")
    center_x = 2.50
    initial_pose = Pose2D(center_x, 0.60, pi / 2.0)
    goal_pose = Pose2D(center_x, 4.40, pi / 2.0)
    actor = replace(
        source.actors[0],
        start_position=Point2D(center_x, source.actors[0].start_position.x),
        velocity=Vector2D(0.0, source.actors[0].velocity.x),
    )
    episode = replace(
        source,
        corridor_width_m=5.0,
        initial_state=RobotState(initial_pose),
        goal_pose=goal_pose,
        reference_path=(initial_pose, goal_pose),
        actors=(actor,),
    )
    world = project_public_witness_world(
        episode,
        maneuver_constraints=ManeuverConstraintSpec(passing_policy=PassingPolicy.ALLOWED),
    )
    target = pass_module._eligible_targets(
        world,
        pass_module._straight_segments(world),
        FROZEN_WITNESS_SEARCH_CONFIG,
    )[0]

    left = pass_module._lateral_offsets(
        world,
        target,
        PassSide.LEFT,
        FROZEN_WITNESS_SEARCH_CONFIG,
    )
    right = pass_module._lateral_offsets(
        world,
        target,
        PassSide.RIGHT,
        FROZEN_WITNESS_SEARCH_CONFIG,
    )

    assert left[0] == pytest.approx(0.45)
    assert right[0] == pytest.approx(0.45)
    assert left == right


def test_terminal_stop_guard_inserts_raw_actor_event_times(monkeypatch) -> None:
    source = _public_episode("same-direction-wide-r00")
    actor = replace(
        source.actors[0],
        active_from_s=0.023,
        active_until_s=0.077,
        # Keep the Actor close enough to force the exact fallback instead of the
        # conservative circumscribed-circle fast acceptance path.
        start_position=Point2D(1.0, source.actors[0].start_position.y),
    )
    episode = replace(source, actors=(actor,))
    world = project_public_witness_world(
        episode,
        maneuver_constraints=ManeuverConstraintSpec(passing_policy=PassingPolicy.ALLOWED),
    )
    actor_x_samples: list[float] = []

    def record_clearance(_pose, *, circle_center, circle_radius_m, profile):
        del circle_radius_m, profile
        actor_x_samples.append(circle_center[0])
        return 10.0

    monkeypatch.setattr(
        pass_module,
        "oriented_footprint_circle_surface_distance",
        record_clearance,
    )
    point = pass_module.WitnessPoint(
        0.0,
        world.initial_state.pose,
        world.initial_state.twist,
        pass_module.WitnessPhase.START,
        "guard_test",
    )
    pass_module._target_is_safely_stoppable.cache_clear()

    assert pass_module._target_is_safely_stoppable(
        point,
        pass_module.Twist2D(0.30, 0.0),
        world,
    )
    assert actor.start_position.x in actor_x_samples


def test_validator_failure_taxonomy_is_explicit_and_exhaustive() -> None:
    assert pass_module._rejection_bucket(("actor_clearance_violation",)) == "dynamic"
    assert pass_module._rejection_bucket(("multi_actor_pass_out_of_scope",)) == "dynamic"
    assert pass_module._rejection_bucket(("static_clearance_violation",)) == "geometry"
    with pytest.raises(RuntimeError, match="unmapped PASS validator failure"):
        pass_module._rejection_bucket(("future_unclassified_failure",))


def test_pass_module_has_no_corpus_label_or_oracle_import() -> None:
    tree = ast.parse(inspect.getsource(pass_module))
    imported_modules = {
        name
        for node in ast.walk(tree)
        for name in (
            (
                *(alias.name for alias in node.names),
                *((node.module,) if isinstance(node, ast.ImportFrom) else ()),
            )
            if isinstance(node, (ast.Import, ast.ImportFrom))
            else ()
        )
        if name is not None
    }
    identifiers = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)} | {
        node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)
    }

    assert not any("dynamic_corpus" in module for module in imported_modules)
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


def test_timed_candidate_preflight_limit_is_atomic_across_sides() -> None:
    config = replace(
        FROZEN_WITNESS_SEARCH_CONFIG,
        max_timed_candidates_per_episode=1,
    )
    world = _allowed_world("same-direction-wide-r00", search_config=config)

    result = search_pass_structured(world, search_config=config)

    assert result.left.status is WitnessSearchStatus.RESOURCE_LIMIT
    assert result.right.status is WitnessSearchStatus.RESOURCE_LIMIT
    assert result.left.reason == "timed_candidate_preflight_limit"
    assert result.right.reason == "timed_candidate_preflight_limit"
    assert result.left.counts.generated_count == 0
    assert result.right.counts.generated_count == 0


def _small_exhaustive_world():
    source = _public_episode("same-direction-wide-r00")
    center_y = 0.85
    initial_pose = Pose2D(0.60, center_y, 0.0)
    goal_pose = Pose2D(4.40, center_y, 0.0)
    actor = replace(
        source.actors[0],
        start_position=Point2D(source.actors[0].start_position.x, center_y),
    )
    episode = replace(
        source,
        corridor_width_m=1.70,
        initial_state=RobotState(initial_pose),
        goal_pose=goal_pose,
        reference_path=(initial_pose, goal_pose),
        actors=(actor,),
    )
    config = replace(
        FROZEN_WITNESS_SEARCH_CONFIG,
        geometry_progress_step_m=1.0,
        linear_targets_mps=(0.30,),
    )
    return project_public_witness_world(
        episode,
        maneuver_constraints=ManeuverConstraintSpec(passing_policy=PassingPolicy.ALLOWED),
        search_config=config,
    ), config


def test_small_public_derived_world_is_exhaustive_and_finds_both_sides() -> None:
    """Exercise the real exhaustive path without launching the 27k public run."""

    world, config = _small_exhaustive_world()

    result = search_pass_structured(world, search_config=config)

    assert result.left.status is WitnessSearchStatus.WITNESS_FOUND
    assert result.right.status is WitnessSearchStatus.WITNESS_FOUND
    for side in (result.left, result.right):
        counts = side.counts
        assert counts.generated_count > 0
        assert counts.generated_count == (
            counts.geometry_pruned_count + counts.dynamic_rejected_count + counts.validated_count
        )
        assert side.best_witness is not None
        assert validate_ground_truth_witness(
            world,
            side.best_witness,
            strict_declarations=True,
        ).passed


def test_parallel_shards_match_serial_semantics_and_counts() -> None:
    world, config = _small_exhaustive_world()

    serial = search_pass_structured(world, search_config=config)
    parallel = search_pass_structured_parallel(
        world,
        search_config=config,
        max_workers=2,
        shard_size=7,
    )

    assert parallel.semantic_content_hash == serial.semantic_content_hash
    assert parallel.count_by_side == serial.count_by_side
    assert parallel.validation_hash_by_side == serial.validation_hash_by_side
    assert parallel.best_pass_left == serial.best_pass_left
    assert parallel.best_pass_right == serial.best_pass_right


@pytest.mark.parametrize(("max_workers", "shard_size"), ((0, 1), (1, 0), (True, 1)))
def test_parallel_operational_knobs_are_positive_exact_integers(
    max_workers,
    shard_size,
) -> None:
    world, config = _small_exhaustive_world()

    with pytest.raises(ValueError):
        search_pass_structured_parallel(
            world,
            search_config=config,
            max_workers=max_workers,
            shard_size=shard_size,
        )


def test_direct_candidate_is_semantically_deterministic() -> None:
    world = _allowed_world("same-direction-wide-r00")
    request = PassCandidateRequest(
        actor_binding_id=world.actors[0].actor_binding_id,
        side=PassSide.RIGHT,
        departure_progress_m=0.0,
        lateral_offset_m=0.45,
        release_tick=0,
        linear_target_mps=0.30,
        angular_magnitude_radps=0.80,
        wait_policy=PassSideWaitPolicy.IMMEDIATE,
    )

    first = generate_pass_candidate(world, request)
    second = generate_pass_candidate(world, request)

    assert first is not None
    assert second is not None
    assert first.semantic_content_hash == second.semantic_content_hash
    assert first == second
