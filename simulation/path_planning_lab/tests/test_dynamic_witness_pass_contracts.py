from __future__ import annotations

from dataclasses import fields, replace

import pytest

from hospital_path_lab.dynamic_corpus import generate_dynamic_v6_public_corpus
from hospital_path_lab.dynamic_witness_contracts import (
    FROZEN_WITNESS_SEARCH_CONFIG,
    PASS_MAX_EVALUATED_CANDIDATES_PER_EPISODE,
    PASS_STRUCTURED_SEARCH_VERSION,
    WITNESS_SEARCH_CONFIG_VERSION,
    WITNESS_VALIDATOR_VERSION,
    PassCandidateCounts,
    PassSide,
    PassSideSearchResult,
    PassSideWaitPolicy,
    PassStructuredSearchResult,
    WitnessKind,
    WitnessObjective,
    WitnessPoint,
    WitnessSearchConfig,
    WitnessSearchStatus,
    WitnessTerminalMode,
    build_automated_witness,
    build_pass_candidate_parameter_hash,
    project_public_witness_world,
)


@pytest.fixture(scope="module")
def pass_contract_world():
    return project_public_witness_world(generate_dynamic_v6_public_corpus()[0])


def _objective() -> WitnessObjective:
    return WitnessObjective(
        hard_failure_count=0,
        terminal_completion_time_s=2.0,
        actual_path_length_m=0.4,
        maximum_reference_deviation_m=0.2,
        full_stop_count=2,
        absolute_angular_travel_rad=1.0,
        kind_rank=0,
        frozen_parameter_tuple=(0.1, 0.2),
    )


def _pass_witness(world, *, kind: WitnessKind = WitnessKind.PASS_LEFT):
    points = tuple(
        WitnessPoint(
            time_s=index * world.kinematic_contract.control_period_s,
            pose=world.initial_state.pose,
            twist=world.initial_state.twist,
            source_primitive_id=f"contract-{index}",
        )
        for index in range(3)
    )
    return build_automated_witness(
        world,
        witness_id=f"contract-{kind.value}",
        kind=kind,
        terminal_mode=WitnessTerminalMode.REJOIN_DWELL,
        points=points,
        required_pass_actor_ids=(world.actors[0].actor_binding_id,),
        departure_time_s=0.01,
        pass_times_by_actor=((world.actors[0].actor_binding_id, 0.02),),
        rejoin_started_at_s=0.03,
        rejoin_confirmed_at_s=0.04,
    )


def _empty_side(side: PassSide) -> PassSideSearchResult:
    return PassSideSearchResult(
        side=side,
        status=WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE,
        reason="structured_template_exhausted",
        counts=PassCandidateCounts(
            generated_count=2,
            geometry_pruned_count=1,
            dynamic_rejected_count=1,
            validated_count=0,
        ),
        best_witness=None,
        objective=None,
        selected_validation_hash=None,
        selected_candidate_parameter_hash=None,
    )


def _found_left(world) -> PassSideSearchResult:
    witness = _pass_witness(world)
    objective = _objective()
    return PassSideSearchResult(
        side=PassSide.LEFT,
        status=WitnessSearchStatus.WITNESS_FOUND,
        reason="validated_pass_left",
        counts=PassCandidateCounts(
            generated_count=4,
            geometry_pruned_count=1,
            dynamic_rejected_count=2,
            validated_count=1,
        ),
        best_witness=witness,
        objective=objective,
        selected_validation_hash="a" * 64,
        selected_candidate_parameter_hash=build_pass_candidate_parameter_hash(
            side=PassSide.LEFT,
            witness=witness,
            objective=objective,
        ),
    )


def _result(world, **changes) -> PassStructuredSearchResult:
    values = {
        "source_projection_hash": world.source_projection_hash,
        "world_content_hash": world.content_hash,
        "vehicle_profile_hash": world.vehicle_profile_hash,
        "maneuver_policy_hash": world.maneuver_constraints.content_hash,
        "maneuver_policy_revision": world.maneuver_constraints.policy_revision,
        "search_config_hash": world.search_config_hash,
        "search_config_version": WITNESS_SEARCH_CONFIG_VERSION,
        "left": _found_left(world),
        "right": _empty_side(PassSide.RIGHT),
        "limitations": ("passing_policy_unspecified",),
        "elapsed_nonqualification_ns": 10,
    }
    values.update(changes)
    return PassStructuredSearchResult(**values)


def test_pass_search_versions_and_frozen_axes_are_explicit() -> None:
    config = FROZEN_WITNESS_SEARCH_CONFIG

    assert WITNESS_SEARCH_CONFIG_VERSION == "structured-witness-search-v2"
    assert WITNESS_VALIDATOR_VERSION == "ground-truth-witness-validator-v3"
    assert PASS_STRUCTURED_SEARCH_VERSION == "pass-structured-search-v1"
    assert config.pass_side_order == (PassSide.LEFT, PassSide.RIGHT)
    assert config.pass_lateral_step_resolution_multiplier == 1.0
    assert config.pass_angular_magnitudes_radps == (0.40, 0.60, 0.80)
    assert config.pass_side_wait_policies == (
        PassSideWaitPolicy.IMMEDIATE,
        PassSideWaitPolicy.UNTIL_TARGET_INACTIVE,
    )
    assert config.pass_minimum_actor_speed_mps == 1e-6
    assert config.pass_same_direction_heading_tolerance_rad == pytest.approx(
        0.17453292519943295
    )
    assert config.pass_speed_advantage_epsilon_mps == 1e-9
    assert config.pass_synthesis_pose_tolerance_m == 0.025
    assert config.pass_synthesis_heading_tolerance_rad == 0.025
    assert (
        config.max_pass_evaluated_candidates_per_episode
        == PASS_MAX_EVALUATED_CANDIDATES_PER_EPISODE
        == 50_000
    )


def test_pass_search_config_hashes_axes_but_not_hard_validator_semantics() -> None:
    config_fields = {field.name for field in fields(WitnessSearchConfig)}
    forbidden_hard_semantics = {
        "minimum_ground_truth_clearance_m",
        "departure_threshold_m",
        "rejoin_distance_m",
        "rejoin_heading_error_rad",
        "rejoin_dwell_s",
        "terminal_dwell_s",
        "event_declaration_tolerance_s",
    }

    assert config_fields.isdisjoint(forbidden_hard_semantics)
    assert len(FROZEN_WITNESS_SEARCH_CONFIG.content_hash) == 64


@pytest.mark.parametrize(
    ("field_name", "value", "message"),
    (
        ("pass_side_order", (PassSide.RIGHT, PassSide.LEFT), "LEFT then RIGHT"),
        ("pass_lateral_step_resolution_multiplier", 0.0, "frozen v2 value"),
        ("pass_angular_magnitudes_radps", (0.8, 0.4), "frozen v2 values"),
        ("pass_minimum_actor_speed_mps", 2e-6, "frozen v2 value"),
        (
            "pass_same_direction_heading_tolerance_rad",
            0.18,
            "frozen v2 value",
        ),
        ("pass_speed_advantage_epsilon_mps", 2e-9, "frozen v2 value"),
        ("pass_synthesis_pose_tolerance_m", 0.03, "frozen v2 value"),
        ("pass_synthesis_heading_tolerance_rad", 0.03, "frozen v2 value"),
        (
            "pass_side_wait_policies",
            (PassSideWaitPolicy.IMMEDIATE,),
            "frozen order",
        ),
        (
            "max_pass_evaluated_candidates_per_episode",
            9_999,
            "frozen v2 value",
        ),
        (
            "max_pass_evaluated_candidates_per_episode",
            0,
            "must be positive",
        ),
    ),
)
def test_pass_search_config_rejects_nonfrozen_axes(
    field_name: str,
    value: object,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        replace(FROZEN_WITNESS_SEARCH_CONFIG, **{field_name: value})


@pytest.mark.parametrize(
    ("field_name", "value", "message"),
    (
        ("pass_side_order", ("left", "right"), "PassSide values"),
        (
            "pass_side_wait_policies",
            ("immediate", "until_target_inactive"),
            "PassSideWaitPolicy values",
        ),
        ("pass_lateral_step_resolution_multiplier", True, "exact float"),
        ("pass_angular_magnitudes_radps", (0.4, True, 0.8), "exact float"),
        ("max_pass_evaluated_candidates_per_episode", True, "exact int"),
    ),
)
def test_pass_search_config_rejects_bool_string_and_wrong_types(
    field_name: str,
    value: object,
    message: str,
) -> None:
    with pytest.raises(TypeError, match=message):
        replace(FROZEN_WITNESS_SEARCH_CONFIG, **{field_name: value})


def test_pass_side_counts_are_exhaustive() -> None:
    with pytest.raises(TypeError, match="exact integers"):
        PassCandidateCounts(True, 0, 0, 1)
    with pytest.raises(ValueError, match="inconsistent"):
        PassCandidateCounts(
            generated_count=3,
            geometry_pruned_count=1,
            dynamic_rejected_count=1,
            validated_count=0,
        )


def test_pass_side_result_binds_kind_status_objective_and_validation_hash(
    pass_contract_world,
) -> None:
    right_witness = _pass_witness(
        pass_contract_world,
        kind=WitnessKind.PASS_RIGHT,
    )

    with pytest.raises(ValueError, match="kind does not match"):
        replace(_found_left(pass_contract_world), best_witness=right_witness)
    with pytest.raises(ValueError, match="validated count"):
        replace(
            _empty_side(PassSide.RIGHT),
            counts=PassCandidateCounts(
                generated_count=1,
                geometry_pruned_count=0,
                dynamic_rejected_count=0,
                validated_count=1,
            ),
        )
    with pytest.raises(ValueError, match="lowercase SHA-256"):
        replace(_found_left(pass_contract_world), selected_validation_hash="BAD")
    with pytest.raises(ValueError, match="does not match witness"):
        replace(
            _found_left(pass_contract_world),
            selected_candidate_parameter_hash="b" * 64,
        )
    draft_witness = replace(
        _pass_witness(pass_contract_world),
        departure_time_s=None,
        pass_times_by_actor=(),
        rejoin_started_at_s=None,
        rejoin_confirmed_at_s=None,
    )
    with pytest.raises(ValueError, match="canonical event declarations"):
        replace(_found_left(pass_contract_world), best_witness=draft_witness)


def test_pass_result_preserves_both_sides_and_excludes_elapsed_time_from_hash(
    pass_contract_world,
) -> None:
    result = _result(
        pass_contract_world,
        limitations=("z_limit", "passing_policy_unspecified", "z_limit"),
    )
    slower = replace(result, elapsed_nonqualification_ns=999_999)

    assert result.best_pass_left is result.left.best_witness
    assert result.best_pass_right is None
    assert result.status_by_side == (
        (PassSide.LEFT, WitnessSearchStatus.WITNESS_FOUND),
        (PassSide.RIGHT, WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE),
    )
    assert result.objective_by_side[0][1] is result.left.objective
    assert result.validation_hash_by_side[0][1] == "a" * 64
    assert result.count_by_side[1][1] is result.right.counts
    assert result.limitations == ("passing_policy_unspecified", "z_limit")
    assert slower.semantic_content_hash == result.semantic_content_hash


def test_pass_result_rejects_provenance_tamper(pass_contract_world) -> None:
    tampered_witness = replace(
        _found_left(pass_contract_world).best_witness,
        world_content_hash="b" * 64,
    )
    assert tampered_witness is not None
    tampered_left = replace(
        _found_left(pass_contract_world),
        best_witness=tampered_witness,
        selected_candidate_parameter_hash=build_pass_candidate_parameter_hash(
            side=PassSide.LEFT,
            witness=tampered_witness,
            objective=_objective(),
        ),
    )

    with pytest.raises(ValueError, match="provenance"):
        _result(pass_contract_world, left=tampered_left)


def test_episode_resource_limit_is_atomic_across_pass_sides(
    pass_contract_world,
) -> None:
    zero = PassCandidateCounts(0, 0, 0, 0)
    limited_left = PassSideSearchResult(
        side=PassSide.LEFT,
        status=WitnessSearchStatus.RESOURCE_LIMIT,
        reason="timed_candidate_preflight_limit",
        counts=zero,
        best_witness=None,
        objective=None,
        selected_validation_hash=None,
        selected_candidate_parameter_hash=None,
    )

    with pytest.raises(ValueError, match="both sides"):
        _result(pass_contract_world, left=limited_left)
