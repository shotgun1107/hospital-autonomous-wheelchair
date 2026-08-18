from __future__ import annotations

import pytest

from hospital_path_lab.dynamic_contracts import DynamicMotionState
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
)
from hospital_path_lab.r5c_observation_diagnostic import (
    R5CDiagnosticOutcome,
    run_r5c_crossing_completion_diagnostic,
    run_r5c_crossing_diagnostic,
    run_r5c_crossing_recovery_diagnostic,
    run_r5c_restop_diagnostic,
    run_r5c_restop_recovery_diagnostic,
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
def normal_restop_recovery_result():
    return run_r5c_restop_recovery_diagnostic(profile=NORMAL_OBSERVATION_PROFILE)


@pytest.fixture(scope="module")
def stress_results():
    return (
        run_r5c_crossing_diagnostic(
            side_index=0,
            profile=STRESS_OBSERVATION_PROFILE,
        ),
        run_r5c_restop_diagnostic(profile=STRESS_OBSERVATION_PROFILE),
    )


@pytest.mark.parametrize(
    ("side_index", "expected_outcome", "expected_stop_epoch"),
    (
        (0, R5CDiagnosticOutcome.CONSERVATIVE_HOLD, 2),
        (1, R5CDiagnosticOutcome.COMPLETED, 1),
    ),
    ids=("left", "right"),
)
def test_normal_crossing_reuses_ttl_valid_dropouts_without_prediction_loss(
    normal_crossing_results,
    side_index: int,
    expected_outcome: R5CDiagnosticOutcome,
    expected_stop_epoch: int,
) -> None:
    result = normal_crossing_results[side_index]

    assert result.actual_release_tick == result.planned_release_tick == 80
    assert result.initial_stop_confirmed_tick is not None
    assert result.first_motion_tick is not None
    assert result.no_frame_tick_count > 0
    assert result.first_prediction_loss_tick is None
    assert result.prediction_loss_ticks == ()
    assert result.outcome is expected_outcome
    assert result.final_stop_epoch == expected_stop_epoch
    if expected_outcome is R5CDiagnosticOutcome.COMPLETED:
        assert result.completion_tick is not None
        assert result.final_motion_state is DynamicMotionState.COMPLETED
    else:
        assert result.completion_tick is None
        assert result.stop_confirmed_tick is not None
        assert result.final_motion_state is DynamicMotionState.HOLDING
    assert result.controller_call_count > 0
    assert result.minimum_actor_clearance_m is not None
    assert result.minimum_actor_clearance_m >= 0.08
    assert result.minimum_static_clearance_m >= 0.08
    assert result.hard_failures == ()
    assert result.passed_safety_boundary


def test_normal_restop_reuses_ttl_valid_dropouts_and_still_stops_safely(
    normal_restop_result,
) -> None:
    result = normal_restop_result

    assert result.planned_release_tick == 44
    assert result.actual_release_tick is not None
    assert result.actual_release_tick >= result.planned_release_tick
    assert result.initial_stop_confirmed_tick is not None
    assert result.first_motion_tick is not None
    assert result.no_frame_tick_count > 0
    assert result.first_prediction_loss_tick is None
    assert result.prediction_loss_ticks == ()
    assert result.stop_confirmed_tick is not None
    assert result.outcome is R5CDiagnosticOutcome.CONSERVATIVE_HOLD
    assert result.final_motion_state is DynamicMotionState.HOLDING
    assert result.final_stop_epoch == 2
    assert result.controller_call_count > 0
    assert result.minimum_actor_clearance_m is not None
    assert result.minimum_actor_clearance_m >= 0.08
    assert result.minimum_static_clearance_m >= 0.08
    assert result.hard_failures == ()


def test_normal_restop_recovery_uses_new_stop_epoch_and_session_after_each_loss(
    normal_restop_recovery_result,
) -> None:
    result = normal_restop_recovery_result

    assert result.outcome is R5CDiagnosticOutcome.COMPLETED
    assert result.final_motion_state is DynamicMotionState.COMPLETED
    assert result.completion_tick is not None
    assert len(result.release_ticks) >= 2
    assert result.no_frame_tick_count > 0
    assert result.prediction_loss_ticks == ()
    assert len(result.confirmed_stop_ticks) == len(result.release_ticks) - 1
    assert result.controller_session_count == len(result.release_ticks)
    assert result.session_stop_epochs == tuple(range(1, len(result.release_ticks) + 1))
    assert all(
        stop_tick < next_release_tick
        for stop_tick, next_release_tick in zip(
            result.confirmed_stop_ticks,
            result.release_ticks[1:],
            strict=True,
        )
    )
    assert result.final_stop_epoch == result.session_stop_epochs[-1]
    assert result.minimum_actor_clearance_m is not None
    assert result.minimum_actor_clearance_m >= 0.08
    assert result.minimum_static_clearance_m >= 0.08
    assert result.hard_failures == ()
    assert result.passed_safety_boundary


def test_stress_recovery_never_uses_empty_or_unready_frames_to_launch() -> None:
    result = run_r5c_restop_recovery_diagnostic(profile=STRESS_OBSERVATION_PROFILE)

    assert result.release_ticks == ()
    assert result.session_stop_epochs == ()
    assert result.first_motion_tick is None
    assert result.controller_call_count == 0
    assert result.final_motion_state is DynamicMotionState.HOLDING
    assert result.maximum_consecutive_ready_frames < 11
    assert result.hard_failures == ()


@pytest.mark.parametrize("side_index", (0, 1), ids=("left", "right"))
def test_stress_completion_extension_does_not_treat_empty_as_post_pass_proof(
    side_index: int,
) -> None:
    result = run_r5c_crossing_completion_diagnostic(
        side_index=side_index,
        profile=STRESS_OBSERVATION_PROFILE,
        tick_limit=1000,
    )

    assert result.release_ticks == ()
    assert result.post_pass_proof_tick is None
    assert result.follow_original_release_tick is None
    assert result.first_motion_tick is None
    assert result.controller_call_count == 0
    assert result.outcome is R5CDiagnosticOutcome.CONSERVATIVE_HOLD
    assert result.final_motion_state is DynamicMotionState.HOLDING
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


def test_crossing_sides_share_the_same_ttl_dropout_events_before_they_diverge(
    normal_crossing_results,
) -> None:
    left, right = normal_crossing_results

    left_counts = dict(left.observation_status_counts)
    right_counts = dict(right.observation_status_counts)
    assert left_counts["dropout"] == right_counts["dropout"] == 2
    assert left.no_frame_tick_count > 0
    assert right.no_frame_tick_count > 0
    assert left.first_prediction_loss_tick is None
    assert right.first_prediction_loss_tick is None


@pytest.mark.parametrize(
    ("side_index", "expected_session_count"),
    ((0, 2), (1, 1)),
    ids=("left", "right"),
)
def test_normal_crossing_recovery_uses_only_confirmed_stop_bound_sessions(
    side_index: int,
    expected_session_count: int,
) -> None:
    result = run_r5c_crossing_recovery_diagnostic(
        side_index=side_index,
        profile=NORMAL_OBSERVATION_PROFILE,
    )

    assert result.outcome in {
        R5CDiagnosticOutcome.COMPLETED,
        R5CDiagnosticOutcome.CONSERVATIVE_HOLD,
    }
    assert result.outcome is R5CDiagnosticOutcome.COMPLETED
    assert len(result.release_ticks) == expected_session_count
    assert result.session_stop_epochs == tuple(range(1, len(result.release_ticks) + 1))
    assert result.prediction_loss_ticks == ()
    assert len(result.confirmed_stop_ticks) == len(result.release_ticks) - 1
    assert all(
        stop_tick < next_release_tick
        for stop_tick, next_release_tick in zip(
            result.confirmed_stop_ticks,
            result.release_ticks[1:],
            strict=True,
        )
    )
    assert result.minimum_actor_clearance_m is not None
    assert result.minimum_actor_clearance_m >= 0.08
    assert result.minimum_static_clearance_m >= 0.08
    assert result.hard_failures == ()
