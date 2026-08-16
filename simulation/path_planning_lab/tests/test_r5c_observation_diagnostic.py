from __future__ import annotations

import pytest

from hospital_path_lab.dynamic_contracts import DynamicMotionState
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
)
from hospital_path_lab.r5c_observation_diagnostic import (
    R5CDiagnosticOutcome,
    run_r5c_crossing_diagnostic,
    run_r5c_restop_diagnostic,
)


@pytest.fixture(scope="module")
def normal_crossing_results():
    return tuple(
        run_r5c_crossing_diagnostic(
            side_index=side_index,
            profile=NORMAL_OBSERVATION_PROFILE,
        )
        for side_index in (0, 1)
    )


@pytest.fixture(scope="module")
def normal_restop_result():
    return run_r5c_restop_diagnostic(profile=NORMAL_OBSERVATION_PROFILE)


@pytest.fixture(scope="module")
def stress_results():
    return (
        run_r5c_crossing_diagnostic(
            side_index=0,
            profile=STRESS_OBSERVATION_PROFILE,
        ),
        run_r5c_restop_diagnostic(profile=STRESS_OBSERVATION_PROFILE),
    )


@pytest.mark.parametrize("side_index", (0, 1), ids=("left", "right"))
def test_normal_crossing_stops_after_first_directional_input_loss(
    normal_crossing_results,
    side_index: int,
) -> None:
    result = normal_crossing_results[side_index]

    assert result.actual_release_tick == result.planned_release_tick == 80
    assert result.initial_stop_confirmed_tick is not None
    assert result.first_motion_tick is not None
    assert result.first_prediction_loss_tick is not None
    assert result.protective_stop_started_tick == result.first_prediction_loss_tick
    assert result.stop_confirmed_tick is not None
    assert result.stop_confirmed_tick > result.first_prediction_loss_tick
    assert result.completion_tick is None
    assert result.outcome is R5CDiagnosticOutcome.CONSERVATIVE_HOLD
    assert result.final_motion_state is DynamicMotionState.HOLDING
    assert result.final_stop_epoch == 2
    assert result.controller_call_count > 0
    assert result.minimum_actor_clearance_m is not None
    assert result.minimum_actor_clearance_m >= 0.08
    assert result.minimum_static_clearance_m >= 0.08
    assert result.hard_failures == ()
    assert result.passed_safety_boundary


def test_normal_restop_stops_after_first_directional_input_loss(
    normal_restop_result,
) -> None:
    result = normal_restop_result

    assert result.planned_release_tick == 44
    assert result.actual_release_tick is not None
    assert result.actual_release_tick >= result.planned_release_tick
    assert result.initial_stop_confirmed_tick is not None
    assert result.first_motion_tick is not None
    assert result.first_prediction_loss_tick is not None
    assert result.stop_confirmed_tick is not None
    assert result.stop_confirmed_tick > result.first_prediction_loss_tick
    assert result.outcome is R5CDiagnosticOutcome.CONSERVATIVE_HOLD
    assert result.final_motion_state is DynamicMotionState.HOLDING
    assert result.final_stop_epoch == 2
    assert result.controller_call_count > 0
    assert result.minimum_actor_clearance_m is not None
    assert result.minimum_actor_clearance_m >= 0.08
    assert result.minimum_static_clearance_m >= 0.08
    assert result.hard_failures == ()


@pytest.mark.parametrize("result_index", (0, 1), ids=("crossing", "restop"))
def test_stress_never_releases_without_ready_directional_prediction(
    stress_results,
    result_index: int,
) -> None:
    result = stress_results[result_index]

    assert result.actual_release_tick is None
    assert result.initial_stop_confirmed_tick is not None
    assert result.first_motion_tick is None
    assert result.first_prediction_loss_tick is None
    assert result.controller_call_count == 0
    assert result.controller_session_count == 0
    assert result.outcome is R5CDiagnosticOutcome.CONSERVATIVE_HOLD
    assert result.final_motion_state is DynamicMotionState.HOLDING
    assert result.final_stop_epoch == 1
    assert dict(result.observation_status_counts).get("ready", 0) == 0
    assert result.hard_failures == ()
    assert result.passed_safety_boundary


def test_crossing_sides_share_the_same_normal_observation_status_stream(
    normal_crossing_results,
) -> None:
    left, right = normal_crossing_results

    assert left.observation_status_counts == right.observation_status_counts
    assert left.no_frame_tick_count == right.no_frame_tick_count
    assert left.first_prediction_loss_tick == right.first_prediction_loss_tick
