from __future__ import annotations

from dataclasses import replace

import pytest

from hospital_path_lab.collision import oriented_footprint_circle_surface_distance
from hospital_path_lab.contracts import Twist2D
from hospital_path_lab.dynamic_contracts import Point2D, Vector2D
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusActor,
    DynamicCorpusEpisode,
    DynamicExpectationCategory,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_witness_contracts import (
    ManeuverConstraintSpec,
    PassingPolicy,
    WitnessKind,
    WitnessPhase,
    WitnessPoint,
    WitnessTerminalMode,
    build_automated_witness,
    project_public_witness_world,
)
from hospital_path_lab.dynamic_witness_validation import (
    validate_ground_truth_witness,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


@pytest.fixture(scope="module")
def feasible_episode():
    return next(
        episode
        for episode in generate_dynamic_v6_public_corpus()
        if episode.expectation_category
        is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
    )


def _legacy_witness_for_new_contract(episode, *, world=None):
    world = world or project_public_witness_world(episode)
    legacy = episode.oracle_spec.feasible_witness
    assert legacy is not None
    points = tuple(
        WitnessPoint(
            time_s=point.time_s,
            pose=point.pose,
            twist=point.twist,
            phase=WitnessPhase.UNSPECIFIED,
            source_primitive_id="legacy-regression-only",
        )
        for point in legacy.points
    )
    return world, build_automated_witness(
        world,
        witness_id=f"converted-{legacy.witness_id}",
        kind=WitnessKind.PASS_RIGHT,
        terminal_mode=WitnessTerminalMode.GOAL_DWELL,
        points=points,
        required_pass_actor_ids=(world.actors[0].actor_binding_id,),
        terminal_dwell_s=legacy.terminal_dwell_s,
    )


def _stationary_hold_contract(template, actor: DynamicCorpusActor):
    episode = DynamicCorpusEpisode(
        schema_version=template.schema_version,
        generator_version=template.generator_version,
        episode_id="synthetic-200hz-contract-case",
        split=template.split,
        expectation_category=DynamicExpectationCategory.WAIT_AND_RESUME,
        seed=template.seed,
        simulation_only=True,
        map_id="synthetic-200hz-contract-map",
        mission_id=template.mission_id,
        duration_s=0.50,
        corridor_width_m=template.corridor_width_m,
        map_length_m=template.map_length_m,
        grid_resolution_m=template.grid_resolution_m,
        initial_state=template.initial_state,
        goal_pose=template.goal_pose,
        reference_path=template.reference_path,
        actors=(actor,),
        progressable=False,
        blocking_cleared_at_s=None,
    )
    world = project_public_witness_world(episode)
    points = tuple(
        WitnessPoint(
            time_s=tick * 0.05,
            pose=world.initial_state.pose,
            twist=Twist2D(),
            phase=WitnessPhase.HOLD,
            source_primitive_id="stationary-hold",
        )
        for tick in range(11)
    )
    witness = build_automated_witness(
        world,
        witness_id="stationary-hold-contract",
        kind=WitnessKind.HOLD_ONLY,
        terminal_mode=WitnessTerminalMode.SAFE_HOLD,
        points=points,
    )
    return world, witness


def test_legacy_positive_is_reproduced_by_independent_validator(
    feasible_episode,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import hospital_path_lab.dynamic_corpus as dynamic_corpus

    def forbidden_private_validator(*_args, **_kwargs):
        raise AssertionError("new validator must not call the private legacy validator")

    monkeypatch.setattr(
        dynamic_corpus,
        "_feasible_witness_failures",
        forbidden_private_validator,
    )
    world, witness = _legacy_witness_for_new_contract(feasible_episode)

    result = validate_ground_truth_witness(world, witness)

    assert result.passed, result.failures
    assert result.failures == ()
    assert result.metrics.departure_time_s is not None
    assert result.metrics.pass_times_by_actor
    assert result.metrics.rejoin_confirmed_at_s is not None
    assert result.metrics.terminal_dwell_observed_s >= 0.50
    assert result.metrics.minimum_actor_clearance_m is not None
    assert result.metrics.minimum_actor_clearance_m >= 0.08
    assert result.metrics.maximum_right_offset_m > 0.10
    assert result.metrics.final_goal_distance_m <= 0.05
    assert len(result.content_hash) == 64


def test_all_five_public_feasible_replicas_pass_ground_truth_validation() -> None:
    episodes = tuple(
        episode
        for episode in generate_dynamic_v6_public_corpus()
        if episode.expectation_category
        is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
    )

    results = tuple(
        validate_ground_truth_witness(*_legacy_witness_for_new_contract(episode))
        for episode in episodes
    )

    assert len(results) == 5
    assert all(result.passed for result in results), tuple(
        result.failures for result in results
    )


def test_single_pose_tamper_fails_kinematics(feasible_episode) -> None:
    world, witness = _legacy_witness_for_new_contract(feasible_episode)
    middle = len(witness.points) // 2
    point = witness.points[middle]
    tampered = replace(
        witness,
        points=(
            *witness.points[:middle],
            replace(point, pose=replace(point.pose, y=point.pose.y + 0.01)),
            *witness.points[middle + 1 :],
        ),
    )

    result = validate_ground_truth_witness(world, tampered)

    assert not result.passed
    assert "kinematic_pose_mismatch" in result.failures


def test_single_timestamp_tamper_fails_20hz(feasible_episode) -> None:
    world, witness = _legacy_witness_for_new_contract(feasible_episode)
    index = 100
    point = witness.points[index]
    tampered = replace(
        witness,
        points=(
            *witness.points[:index],
            replace(point, time_s=point.time_s + 0.001),
            *witness.points[index + 1 :],
        ),
    )

    result = validate_ground_truth_witness(world, tampered)

    assert not result.passed
    assert "witness_not_20hz" in result.failures


def test_acceleration_tamper_is_rejected(feasible_episode) -> None:
    world, witness = _legacy_witness_for_new_contract(feasible_episode)
    point = witness.points[1]
    tampered = replace(
        witness,
        points=(
            witness.points[0],
            replace(point, twist=Twist2D(linear=0.30, angular=0.0)),
            *witness.points[2:],
        ),
    )

    result = validate_ground_truth_witness(world, tampered)

    assert not result.passed
    assert "linear_acceleration_exceeded" in result.failures


def test_terminal_motion_tamper_removes_actual_dwell(feasible_episode) -> None:
    world, witness = _legacy_witness_for_new_contract(feasible_episode)
    last = witness.points[-1]
    tampered = replace(
        witness,
        points=(
            *witness.points[:-1],
            replace(last, twist=Twist2D(0.01, 0.0)),
        ),
    )

    result = validate_ground_truth_witness(world, tampered)

    assert not result.passed
    assert "terminal_dwell_missing" in result.failures


def test_world_provenance_tamper_is_fail_closed(feasible_episode) -> None:
    world, witness = _legacy_witness_for_new_contract(feasible_episode)
    tampered = replace(witness, world_content_hash="0" * 64)

    result = validate_ground_truth_witness(world, tampered)

    assert not result.passed
    assert "world_content_hash_mismatch" in result.failures


def test_explicit_no_passing_policy_rejects_identical_pass(feasible_episode) -> None:
    prohibited_world = project_public_witness_world(
        feasible_episode,
        maneuver_constraints=ManeuverConstraintSpec(
            policy_revision=7,
            passing_policy=PassingPolicy.PROHIBITED,
        ),
    )
    world, witness = _legacy_witness_for_new_contract(
        feasible_episode,
        world=prohibited_world,
    )

    result = validate_ground_truth_witness(world, witness)

    assert not result.passed
    assert "passing_policy_prohibited" in result.failures


def test_actor_circle_collision_is_not_hidden_by_prediction_logic(
    feasible_episode,
) -> None:
    actor = feasible_episode.actors[0]
    colliding_actor = replace(
        actor,
        active_from_s=0.0,
        active_until_s=feasible_episode.duration_s,
        start_position=replace(
            actor.start_position,
            x=feasible_episode.initial_state.pose.x,
            y=feasible_episode.initial_state.pose.y,
        ),
        velocity=replace(actor.velocity, x=0.0, y=0.0),
    )
    projected_episode = DynamicCorpusEpisode(
        schema_version=feasible_episode.schema_version,
        generator_version=feasible_episode.generator_version,
        episode_id="synthetic-collision-contract-case",
        split=feasible_episode.split,
        expectation_category=feasible_episode.expectation_category,
        seed=feasible_episode.seed,
        simulation_only=True,
        map_id="synthetic-collision-contract-map",
        mission_id=feasible_episode.mission_id,
        duration_s=feasible_episode.duration_s,
        corridor_width_m=feasible_episode.corridor_width_m,
        map_length_m=feasible_episode.map_length_m,
        grid_resolution_m=feasible_episode.grid_resolution_m,
        initial_state=feasible_episode.initial_state,
        goal_pose=feasible_episode.goal_pose,
        reference_path=feasible_episode.reference_path,
        actors=(colliding_actor,),
        progressable=True,
        blocking_cleared_at_s=None,
    )
    colliding_world = project_public_witness_world(projected_episode)
    legacy = feasible_episode.oracle_spec.feasible_witness
    assert legacy is not None
    colliding_witness = build_automated_witness(
        colliding_world,
        witness_id="colliding-ground-truth-regression",
        kind=WitnessKind.PASS_RIGHT,
        terminal_mode=WitnessTerminalMode.GOAL_DWELL,
        points=tuple(
            WitnessPoint(
                time_s=point.time_s,
                pose=point.pose,
                twist=point.twist,
                source_primitive_id="legacy-regression-only",
            )
            for point in legacy.points
        ),
        required_pass_actor_ids=(colliding_world.actors[0].actor_binding_id,),
    )

    result = validate_ground_truth_witness(colliding_world, colliding_witness)

    assert not result.passed
    assert "actor_clearance_violation" in result.failures


def test_exact_actor_clearance_boundary_passes_and_one_millimetre_fails(
    feasible_episode,
) -> None:
    profile = VIRTUAL_DOLL_WHEELCHAIR_V0_1
    initial = feasible_episode.initial_state.pose
    boundary_x = (
        initial.x
        + profile.collision_length_m / 2.0
        + 0.18
        + profile.minimum_clearance_m
    )
    boundary_actor = DynamicCorpusActor(
        actor_id="boundary-actor",
        active_from_s=0.0,
        active_until_s=0.50,
        start_position=Point2D(boundary_x, initial.y),
        velocity=Vector2D(0.0, 0.0),
    )
    safe_world, safe_witness = _stationary_hold_contract(
        feasible_episode,
        boundary_actor,
    )
    unsafe_world, unsafe_witness = _stationary_hold_contract(
        feasible_episode,
        replace(
            boundary_actor,
            start_position=Point2D(boundary_x - 0.001, initial.y),
        ),
    )

    safe = validate_ground_truth_witness(safe_world, safe_witness)
    unsafe = validate_ground_truth_witness(unsafe_world, unsafe_witness)

    assert safe.passed, safe.failures
    assert safe.metrics.minimum_actor_clearance_m == pytest.approx(0.08)
    assert not unsafe.passed
    assert unsafe.metrics.minimum_actor_clearance_m == pytest.approx(0.079)
    assert "actor_clearance_violation" in unsafe.failures


def test_200hz_validator_catches_between_tick_actor_clearance(feasible_episode) -> None:
    profile = VIRTUAL_DOLL_WHEELCHAIR_V0_1
    pose = feasible_episode.initial_state.pose
    corner_x = pose.x + profile.collision_length_m / 2.0
    corner_y = pose.y + profile.collision_width_m / 2.0
    closest_center_distance_m = 0.18 + profile.minimum_clearance_m - 0.0001
    diagonal_component_m = closest_center_distance_m / 2.0**0.5
    midpoint = Point2D(
        corner_x + diagonal_component_m,
        corner_y + diagonal_component_m,
    )
    component_speed_mps = 0.50 / 2.0**0.5
    velocity = Vector2D(-component_speed_mps, component_speed_mps)
    actor = DynamicCorpusActor(
        actor_id="between-tick-actor",
        active_from_s=0.0,
        active_until_s=0.50,
        start_position=Point2D(
            midpoint.x - velocity.x * 0.025,
            midpoint.y - velocity.y * 0.025,
        ),
        velocity=velocity,
    )
    world, witness = _stationary_hold_contract(feasible_episode, actor)
    endpoint_clearances = tuple(
        oriented_footprint_circle_surface_distance(
            pose,
            circle_center=(state.position.x, state.position.y),
            circle_radius_m=state.radius_m,
        )
        for time_s in (0.0, 0.05)
        if (state := actor.state_at(time_s)) is not None
    )

    result = validate_ground_truth_witness(world, witness)

    assert min(endpoint_clearances) > profile.minimum_clearance_m
    assert not result.passed
    assert result.metrics.minimum_actor_clearance_m is not None
    assert result.metrics.minimum_actor_clearance_m < profile.minimum_clearance_m
    assert "actor_clearance_violation" in result.failures


def test_validator_samples_exact_off_grid_actor_appearance_time(
    feasible_episode,
) -> None:
    profile = VIRTUAL_DOLL_WHEELCHAIR_V0_1
    pose = feasible_episode.initial_state.pose
    actor = DynamicCorpusActor(
        actor_id="off-grid-appearance-actor",
        active_from_s=0.002,
        active_until_s=0.50,
        start_position=Point2D(
            pose.x
            + profile.collision_length_m / 2.0
            + 0.18
            + profile.minimum_clearance_m
            - 0.0001,
            pose.y,
        ),
        velocity=Vector2D(0.50, 0.0),
    )
    world, witness = _stationary_hold_contract(feasible_episode, actor)

    result = validate_ground_truth_witness(world, witness)

    assert not result.passed
    assert result.metrics.minimum_actor_clearance_m is not None
    assert result.metrics.minimum_actor_clearance_m < profile.minimum_clearance_m
    assert "actor_clearance_violation" in result.failures


def test_validator_samples_exact_inclusive_off_grid_actor_end_time(
    feasible_episode,
) -> None:
    profile = VIRTUAL_DOLL_WHEELCHAIR_V0_1
    pose = feasible_episode.initial_state.pose
    actor = DynamicCorpusActor(
        actor_id="off-grid-inclusive-end-actor",
        active_from_s=0.0,
        active_until_s=0.002,
        start_position=Point2D(
            pose.x
            + profile.collision_length_m / 2.0
            + 0.18
            + profile.minimum_clearance_m
            + 0.0009,
            pose.y,
        ),
        velocity=Vector2D(-0.50, 0.0),
    )
    world, witness = _stationary_hold_contract(feasible_episode, actor)

    result = validate_ground_truth_witness(world, witness)

    assert not result.passed
    assert result.metrics.minimum_actor_clearance_m is not None
    assert result.metrics.minimum_actor_clearance_m < profile.minimum_clearance_m
    assert "actor_clearance_violation" in result.failures


def test_actor_event_timestamp_is_not_rounded_out_of_its_active_interval(
    feasible_episode,
) -> None:
    profile = VIRTUAL_DOLL_WHEELCHAIR_V0_1
    pose = feasible_episode.initial_state.pose
    active_until_s = 0.4048999999996
    exact_unsafe_clearance_m = profile.minimum_clearance_m - 0.0001
    actor = DynamicCorpusActor(
        actor_id="sub-picosecond-event-rounding-actor",
        active_from_s=0.0,
        active_until_s=active_until_s,
        start_position=Point2D(
            pose.x
            + profile.collision_length_m / 2.0
            + 0.18
            + exact_unsafe_clearance_m
            + 0.50 * active_until_s,
            pose.y,
        ),
        velocity=Vector2D(-0.50, 0.0),
    )
    world, witness = _stationary_hold_contract(feasible_episode, actor)

    result = validate_ground_truth_witness(world, witness)

    assert not result.passed
    assert result.metrics.minimum_actor_clearance_m is not None
    assert result.metrics.minimum_actor_clearance_m <= exact_unsafe_clearance_m
    assert "actor_clearance_violation" in result.failures


def test_declared_event_time_must_match_independent_measurement(
    feasible_episode,
) -> None:
    world, witness = _legacy_witness_for_new_contract(feasible_episode)
    tampered = replace(witness, departure_time_s=0.0)

    result = validate_ground_truth_witness(world, tampered)

    assert not result.passed
    assert "declared_departure_mismatch" in result.failures


def test_validator_does_not_need_original_episode_identity(feasible_episode) -> None:
    world, witness = _legacy_witness_for_new_contract(feasible_episode)

    result = validate_ground_truth_witness(world, witness)

    assert result.passed, result.failures
    assert feasible_episode.episode_id not in repr(result)
