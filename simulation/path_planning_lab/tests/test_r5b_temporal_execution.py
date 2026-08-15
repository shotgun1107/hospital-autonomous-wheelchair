from __future__ import annotations

from pathlib import Path

import pytest

from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PersistentSourceDerivedDwbController,
)
from hospital_path_lab.persistent_rpp_controller import PersistentRppController
from hospital_path_lab.r5b_temporal_evidence import frozen_r2_archive_path
from hospital_path_lab.r5b_temporal_execution import (
    assert_finite_r5b_result,
    run_r5b_temporal_case,
)
from hospital_path_lab.r5b_temporal_reference import (
    build_r5b_temporal_reference_bundles,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def first_left_bundle():
    return build_r5b_temporal_reference_bundles(
        frozen_r2_archive_path(REPOSITORY_ROOT)
    )[0]


@pytest.mark.parametrize(
    "controller",
    (PersistentRppController(), PersistentSourceDerivedDwbController()),
    ids=("rpp", "dwb"),
)
def test_ideal_temporal_execution_holds_until_release_and_then_moves(
    first_left_bundle,
    controller,
) -> None:
    result = run_r5b_temporal_case(
        first_left_bundle,
        controller=controller,
        tick_limit=45,
    )
    assert result.first_controller_tick == 40
    assert result.first_motion_tick == 44
    assert result.controller_call_count == 5
    assert result.gate_override_count == 0
    assert result.final_pose == first_left_bundle.source.world.initial_state.pose
    assert "departure_not_observed" in result.hard_failures
    assert_finite_r5b_result(result)


def test_current_rpp_is_still_overlapping_target_at_last_ground_truth_tick(
    first_left_bundle,
) -> None:
    result = run_r5b_temporal_case(
        first_left_bundle,
        controller=PersistentRppController(),
        tick_limit=610,
    )
    assert result.last_target_present_tick == 599
    assert result.last_target_progress_gap_m == pytest.approx(0.18533733027495725)
    required_separation = (
        first_left_bundle.build_context.vehicle_profile.collision_length_m / 2.0
        + first_left_bundle.source.world.actors[0].radius_m
    )
    assert result.last_target_progress_gap_m < required_separation
    assert result.overtake_tick is None
    assert result.rejoin_tick is None
    assert result.completed is False
    assert result.hard_failures[0].startswith(
        "directional_prediction_not_ready:604:empty_frame"
    )
    assert_finite_r5b_result(result)
