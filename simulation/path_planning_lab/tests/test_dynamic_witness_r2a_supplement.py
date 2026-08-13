from __future__ import annotations

from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    generate_dynamic_corpus,
)
from hospital_path_lab.dynamic_witness_contracts import (
    WitnessKind,
    WitnessSearchStatus,
    project_public_witness_world,
)
from hospital_path_lab.dynamic_witness_crossing import search_crossing_bypass
from hospital_path_lab.dynamic_witness_events import (
    crossing_targets,
    ground_truth_hazard_intervals,
)
from hospital_path_lab.dynamic_witness_restop import (
    RestopEvidenceLevel,
    search_multi_hazard_restop,
    validate_multi_hazard_restop,
)
from hospital_path_lab.dynamic_witness_search import generate_hold_only_witness


def _legacy_world(category: DynamicExpectationCategory):
    episode = next(
        item
        for item in generate_dynamic_corpus()
        if item.split is DynamicCorpusSplit.GOLDEN
        and item.expectation_category is category
    )
    return project_public_witness_world(episode)


def test_legacy_crossing_has_distinct_bypass_witnesses_on_both_sides() -> None:
    world = _legacy_world(DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE)
    targets = crossing_targets(world)
    assert len(targets) == 1
    result = search_crossing_bypass(world)
    for side in (result.left, result.right):
        assert side.status is WitnessSearchStatus.WITNESS_FOUND
        assert side.termination_reason == "crossing_bypass_found"
        assert side.selected_witness is not None
        witness = side.selected_witness
        assert witness.kind in (
            WitnessKind.CROSSING_BYPASS_LEFT,
            WitnessKind.CROSSING_BYPASS_RIGHT,
        )
        assert witness.departure_time_s is not None
        assert len(witness.pass_times_by_actor) == 1
        bypass_time = witness.pass_times_by_actor[0][1]
        assert witness.rejoin_started_at_s is not None
        assert witness.rejoin_confirmed_at_s is not None
        assert (
            witness.departure_time_s
            < bypass_time
            < witness.rejoin_started_at_s
            <= witness.rejoin_confirmed_at_s
        )
        assert side.selected_validation_hash is not None


def test_legacy_two_hazard_search_proves_ordered_restop_and_recovery() -> None:
    world = _legacy_world(DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP)
    hazards = ground_truth_hazard_intervals(world)
    assert len(hazards) == 2
    assert hazards[0].ends_at_s < hazards[1].starts_at_s
    result = search_multi_hazard_restop(world)
    assert result.witness is not None
    assert result.validation is not None
    assert result.termination_reason == "restop_and_recovery_found"
    assert result.validation.evidence_level is (
        RestopEvidenceLevel.RESTOP_AND_RECOVERY_PROVEN
    )
    assert result.validation.core_passed
    assert result.validation.recovery_passed
    assert result.validation.intermediate_progress_m >= 0.10
    bound = tuple(
        interval
        for interval in result.validation.stop_intervals
        if interval.bound_hazard_ids
    )
    assert len(bound) == 2
    assert bound[0].stopped_until_s < bound[1].stopped_from_s
    assert bound[0].following_motion_observed
    assert bound[1].preceding_motion_observed


def test_continuous_hold_is_not_misclassified_as_two_restops() -> None:
    world = _legacy_world(DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP)
    validation = validate_multi_hazard_restop(
        world,
        generate_hold_only_witness(world),
    )
    assert not validation.core_passed
    assert validation.evidence_level is RestopEvidenceLevel.NONE
    assert "continuous_hold_misclassified_as_restop" in validation.core_failures
    assert "second_stop_not_distinct" in validation.core_failures
