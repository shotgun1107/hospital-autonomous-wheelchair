from __future__ import annotations

from dataclasses import replace

import pytest

from hospital_path_lab.collision import oriented_footprint_circle_surface_distance
from hospital_path_lab.contracts import Pose2D, Twist2D
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
    canonicalize_and_validate_ground_truth_pass,
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


def _legacy_pass_with_actor(
    template,
    actor: DynamicCorpusActor,
    *,
    extra_actors: tuple[DynamicCorpusActor, ...] = (),
    reference_path: tuple[Pose2D, ...] | None = None,
):
    episode = DynamicCorpusEpisode(
        schema_version=template.schema_version,
        generator_version=template.generator_version,
        episode_id="synthetic-pass-validator-case",
        split=template.split,
        expectation_category=DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE,
        seed=template.seed,
        simulation_only=True,
        map_id="synthetic-pass-validator-map",
        mission_id=template.mission_id,
        duration_s=template.duration_s,
        corridor_width_m=template.corridor_width_m,
        map_length_m=template.map_length_m,
        grid_resolution_m=template.grid_resolution_m,
        initial_state=template.initial_state,
        goal_pose=template.goal_pose,
        reference_path=reference_path or template.reference_path,
        actors=(actor, *extra_actors),
        progressable=True,
        blocking_cleared_at_s=None,
    )
    world = project_public_witness_world(episode)
    legacy = template.oracle_spec.feasible_witness
    assert legacy is not None
    witness = build_automated_witness(
        world,
        witness_id="synthetic-pass-validator-witness",
        kind=WitnessKind.PASS_RIGHT,
        terminal_mode=WitnessTerminalMode.GOAL_DWELL,
        points=tuple(
            WitnessPoint(
                time_s=point.time_s,
                pose=point.pose,
                twist=point.twist,
                source_primitive_id="pass-validator-test",
            )
            for point in legacy.points
        ),
        required_pass_actor_ids=(world.actors[0].actor_binding_id,),
        terminal_dwell_s=legacy.terminal_dwell_s,
    )
    return world, witness


def _with_measured_declarations(world, witness):
    measurement = validate_ground_truth_witness(world, witness)
    assert measurement.passed, measurement.failures
    return replace(
        witness,
        departure_time_s=measurement.metrics.departure_time_s,
        pass_times_by_actor=measurement.metrics.pass_times_by_actor,
        rejoin_started_at_s=measurement.metrics.rejoin_started_at_s,
        rejoin_confirmed_at_s=measurement.metrics.rejoin_confirmed_at_s,
    )


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


def test_strict_pass_requires_and_accepts_independently_measured_declarations(
    feasible_episode,
) -> None:
    actor = replace(feasible_episode.actors[0], active_until_s=21.0)
    world, draft = _legacy_pass_with_actor(feasible_episode, actor)

    missing = validate_ground_truth_witness(
        world,
        draft,
        strict_declarations=True,
    )
    canonical = _with_measured_declarations(world, draft)
    strict = validate_ground_truth_witness(
        world,
        canonical,
        strict_declarations=True,
    )

    assert not missing.passed
    assert "strict_event_declaration_missing" in missing.failures
    assert strict.passed, strict.failures


def test_one_sweep_canonical_pass_matches_two_pass_strict_result(
    feasible_episode,
) -> None:
    actor = replace(feasible_episode.actors[0], active_until_s=21.0)
    world, draft = _legacy_pass_with_actor(feasible_episode, actor)
    expected_witness = _with_measured_declarations(world, draft)
    expected_validation = validate_ground_truth_witness(
        world,
        expected_witness,
        strict_declarations=True,
    )

    canonical, validation = canonicalize_and_validate_ground_truth_pass(
        world,
        draft,
    )

    assert canonical == expected_witness
    assert validation == expected_validation


def test_one_sweep_canonical_pass_applies_strict_multi_actor_scope(
    feasible_episode,
) -> None:
    target = replace(feasible_episode.actors[0], active_until_s=21.0)
    extra_actor = DynamicCorpusActor(
        actor_id="one-sweep-extra-blocker",
        active_from_s=0.0,
        active_until_s=21.0,
        start_position=Point2D(2.20, target.start_position.y),
        velocity=Vector2D(0.06, 0.0),
    )
    world, draft = _legacy_pass_with_actor(
        feasible_episode,
        target,
        extra_actors=(extra_actor,),
    )

    canonical, validation = canonicalize_and_validate_ground_truth_pass(
        world,
        draft,
    )

    assert canonical is None
    assert not validation.passed
    assert "multi_actor_pass_out_of_scope" in validation.failures


def test_strict_event_declaration_tolerance_is_five_milliseconds(
    feasible_episode,
) -> None:
    actor = replace(feasible_episode.actors[0], active_until_s=21.0)
    world, draft = _legacy_pass_with_actor(feasible_episode, actor)
    canonical = _with_measured_declarations(world, draft)
    assert canonical.departure_time_s is not None

    at_boundary = validate_ground_truth_witness(
        world,
        replace(
            canonical,
            departure_time_s=canonical.departure_time_s + 0.005,
        ),
        strict_declarations=True,
    )
    outside_boundary = validate_ground_truth_witness(
        world,
        replace(
            canonical,
            departure_time_s=canonical.departure_time_s + 0.00501,
        ),
        strict_declarations=True,
    )

    assert at_boundary.passed, at_boundary.failures
    assert not outside_boundary.passed
    assert "declared_departure_mismatch" in outside_boundary.failures


def test_strict_pass_rejects_round_trip_and_nonadjacent_duplicate_reference(
    feasible_episode,
) -> None:
    start, end = feasible_episode.reference_path
    actor = replace(feasible_episode.actors[0], active_until_s=21.0)
    round_trip = (start, end, start)
    duplicate = (
        start,
        end,
        Pose2D(end.x, end.y + 1.0, 0.0),
        Pose2D(start.x, start.y + 1.0, 0.0),
        start,
        end,
    )

    for reference_path in (round_trip, duplicate):
        world, draft = _legacy_pass_with_actor(
            feasible_episode,
            actor,
            reference_path=reference_path,
        )
        canonical = _with_measured_declarations(world, draft)

        result = validate_ground_truth_witness(
            world,
            canonical,
            strict_declarations=True,
        )

        assert not result.passed
        assert "ambiguous_reference_projection" in result.failures


def test_strict_pass_requires_departure_pass_and_rejoin_on_same_segment(
    feasible_episode,
) -> None:
    start, end = feasible_episode.reference_path
    split_reference = (
        start,
        Pose2D(2.0, start.y, start.yaw),
        end,
    )
    actor = replace(feasible_episode.actors[0], active_until_s=21.0)
    world, draft = _legacy_pass_with_actor(
        feasible_episode,
        actor,
        reference_path=split_reference,
    )
    canonical = _with_measured_declarations(world, draft)

    result = validate_ground_truth_witness(
        world,
        canonical,
        strict_declarations=True,
    )

    assert not result.passed
    assert "pass_reference_segment_mismatch" in result.failures


def test_strict_pass_rejects_additional_same_direction_lane_blocker(
    feasible_episode,
) -> None:
    target = replace(feasible_episode.actors[0], active_until_s=21.0)
    extra_actor = DynamicCorpusActor(
        actor_id="extra-same-direction-blocker",
        active_from_s=0.0,
        active_until_s=21.0,
        start_position=Point2D(2.20, target.start_position.y),
        velocity=Vector2D(0.06, 0.0),
    )
    world, draft = _legacy_pass_with_actor(
        feasible_episode,
        target,
        extra_actors=(extra_actor,),
    )
    canonical = _with_measured_declarations(world, draft)

    result = validate_ground_truth_witness(
        world,
        canonical,
        strict_declarations=True,
    )

    assert not result.passed
    assert "multi_actor_pass_out_of_scope" in result.failures


def test_strict_pass_allows_rear_same_direction_actor(feasible_episode) -> None:
    target = replace(feasible_episode.actors[0], active_until_s=21.0)
    extra_actor = DynamicCorpusActor(
        actor_id="extra-same-direction-behind",
        active_from_s=0.0,
        active_until_s=21.0,
        start_position=Point2D(-1.00, target.start_position.y),
        velocity=Vector2D(0.01, 0.0),
    )
    world, draft = _legacy_pass_with_actor(
        feasible_episode,
        target,
        extra_actors=(extra_actor,),
    )
    canonical = _with_measured_declarations(world, draft)

    result = validate_ground_truth_witness(
        world,
        canonical,
        strict_declarations=True,
    )

    assert result.passed, result.failures


def test_strict_pass_allows_far_ahead_actor_beyond_planned_rejoin(
    feasible_episode,
) -> None:
    target = replace(feasible_episode.actors[0], active_until_s=21.0)
    extra_actor = DynamicCorpusActor(
        actor_id="extra-same-direction-far-ahead",
        active_from_s=0.0,
        active_until_s=21.0,
        start_position=Point2D(4.20, target.start_position.y),
        velocity=Vector2D(0.01, 0.0),
    )
    world, draft = _legacy_pass_with_actor(
        feasible_episode,
        target,
        extra_actors=(extra_actor,),
    )
    canonical = _with_measured_declarations(world, draft)

    result = validate_ground_truth_witness(
        world,
        canonical,
        strict_declarations=True,
    )

    assert result.passed, result.failures


@pytest.mark.parametrize(
    ("actor_change", "expected_failure"),
    (
        (
            {"active_from_s": 4.0},
            "target_inactive_at_departure",
        ),
        (
            {"start_position": Point2D(0.50, 2.32)},
            "target_not_ahead_at_departure",
        ),
        (
            {"start_position": Point2D(1.518, 2.77)},
            "target_not_lane_overlapping_at_departure",
        ),
        (
            {"velocity": Vector2D(0.0, 0.0)},
            "target_not_same_direction_at_departure",
        ),
    ),
)
def test_pass_target_is_active_ahead_lane_overlapping_and_same_direction(
    feasible_episode,
    actor_change,
    expected_failure,
) -> None:
    actor = replace(feasible_episode.actors[0], **actor_change)
    world, witness = _legacy_pass_with_actor(feasible_episode, actor)

    result = validate_ground_truth_witness(world, witness)

    assert not result.passed
    assert expected_failure in result.failures


def test_pass_kind_requires_selected_side_until_overtake(feasible_episode) -> None:
    world, witness = _legacy_witness_for_new_contract(feasible_episode)
    reference_y = world.reference_path[0].y
    mirrored = replace(
        witness,
        points=tuple(
            replace(
                point,
                pose=replace(
                    point.pose,
                    y=2.0 * reference_y - point.pose.y,
                    yaw=-point.pose.yaw,
                ),
                twist=replace(point.twist, angular=-point.twist.angular),
            )
            for point in witness.points
        ),
    )

    result = validate_ground_truth_witness(world, mirrored)

    assert not result.passed
    assert "pass_wrong_side" in result.failures


def test_pass_does_not_round_actor_end_past_raw_off_grid_event(
    feasible_episode,
) -> None:
    actor = replace(feasible_episode.actors[0], active_until_s=19.5353)
    world, witness = _legacy_pass_with_actor(feasible_episode, actor)

    result = validate_ground_truth_witness(world, witness)

    assert not result.passed
    assert result.metrics.pass_times_by_actor == ()
    assert "ordered_overtake_missing" in result.failures


def test_strict_pass_rejects_actor_retaking_lead_before_rejoin(
    feasible_episode,
) -> None:
    world, draft = _legacy_witness_for_new_contract(feasible_episode)
    canonical = _with_measured_declarations(world, draft)

    result = validate_ground_truth_witness(
        world,
        canonical,
        strict_declarations=True,
    )

    assert not result.passed
    assert "post_pass_reversal" in result.failures


def test_rejoin_and_terminal_dwell_may_share_stationary_time(
    feasible_episode,
) -> None:
    actor = replace(feasible_episode.actors[0], active_until_s=21.0)
    world, draft = _legacy_pass_with_actor(feasible_episode, actor)
    stationary = next(
        point for point in draft.points if point.time_s == pytest.approx(38.90)
    )
    shared_dwell_points = tuple(
        replace(stationary, time_s=stationary.time_s + tick * 0.05)
        for tick in range(1, 11)
    )
    overlap_draft = replace(
        draft,
        terminal_mode=WitnessTerminalMode.REJOIN_DWELL,
        points=(
            *(point for point in draft.points if point.time_s <= stationary.time_s),
            *shared_dwell_points,
        ),
    )
    canonical = _with_measured_declarations(world, overlap_draft)

    result = validate_ground_truth_witness(
        world,
        canonical,
        strict_declarations=True,
    )

    assert result.passed, result.failures
    assert result.metrics.rejoin_started_at_s is not None
    assert result.metrics.rejoin_confirmed_at_s is not None
    assert result.metrics.terminal_dwell_observed_s >= 0.50
    assert canonical.points[-1].time_s < (
        result.metrics.rejoin_started_at_s + 1.0
    )
