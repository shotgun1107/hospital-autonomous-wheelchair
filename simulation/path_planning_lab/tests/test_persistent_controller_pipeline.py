from __future__ import annotations

from dataclasses import replace

import pytest

from hospital_path_lab.contracts import RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DynamicHoldReason,
    DynamicMotionState,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import ActorPredictionSet
from hospital_path_lab.dynamic_safety import DynamicSafetyGate
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PersistentSourceDerivedDwbController,
)
from hospital_path_lab.local_reference_reporting import (
    evaluate_local_reference_public_case,
    public_local_reference_cases,
)
from hospital_path_lab.persistent_controller_contracts import PersistentControllerStatus
from hospital_path_lab.persistent_controller_pipeline import (
    PersistentControllerPipeline,
    integrate_persistent_chassis_pose,
    persistent_result_to_dynamic_proposal,
)
from hospital_path_lab.persistent_rpp_controller import PersistentRppController


@pytest.fixture(scope="module")
def public_wide_left():
    case = next(
        item for item in public_local_reference_cases() if item.public_id == "wide-straight-left"
    )
    result = evaluate_local_reference_public_case(case)
    assert result.hard_failures == ()
    assert len(result.reference_set.candidates) == 1
    return result


def _fresh_empty(build_context, tick: int):
    simulation_time_s = tick * 0.05
    metadata = build_context.static_grid_snapshot.metadata
    frame = DynamicObservationFrame(
        stream_id="r5-functional-stream-v1",
        episode_id="r5-functional-public-v1",
        episode_seed=101,
        map_id=metadata.map_id,
        map_revision=metadata.map_revision,
        observation_revision=metadata.observation_revision,
        sequence=tick,
        observed_at_s=simulation_time_s,
        delivered_at_s=simulation_time_s,
        frame_kind=DynamicObservationFrameKind.EMPTY,
        tracks=(),
        content_hash=f"r5-empty-frame-{tick}",
    )
    observation = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.FRESH,
        frame=frame,
        age_s=0.0,
        failures=(),
        last_event_was_no_frame=False,
    )
    prediction = ActorPredictionSet(
        stream_id=frame.stream_id,
        episode_id=frame.episode_id,
        map_id=frame.map_id,
        map_revision=frame.map_revision,
        observation_revision=frame.observation_revision,
        sequence=frame.sequence,
        source_content_hash=frame.content_hash,
        observed_at_s=simulation_time_s,
        controller_time_s=simulation_time_s,
        snapshot_age_s=0.0,
        tubes=(),
    )
    return observation, prediction


def _pipeline(public_wide_left, controller):
    context = public_wide_left.build_context
    reference = public_wide_left.reference_set.candidates[0]
    return PersistentControllerPipeline(
        controller=controller,
        build_context=context,
        full_reference=reference,
        validation=public_wide_left.validations[0],
        initial_robot_state=RobotState(reference.knots[0].pose, Twist2D()),
    )


def _gate_for(reference_binding):
    assert reference_binding is not None
    return DynamicSafetyGate(initial_stop_epoch=reference_binding.stop_epoch)


def test_rpp_runs_a_reference_bound_20hz_prefix_through_the_shared_gate(
    public_wide_left,
) -> None:
    pipeline = _pipeline(public_wide_left, PersistentRppController())
    records = []

    for _ in range(60):
        observation, prediction = _fresh_empty(
            public_wide_left.build_context,
            pipeline.tick_id,
        )
        record = pipeline.step(
            observation_snapshot=observation,
            prediction_set=prediction,
        )
        records.append(record)
        assert record.controller_result is not None
        assert record.proposal.reference_binding == record.safety_context.reference_binding
        assert "reference_binding_mismatch" not in record.safety_decision.failure_reasons
        assert record.safety_decision.proposal_accepted
        assert record.safety_decision.motion_state is DynamicMotionState.MOVING

    assert len(records) == 60
    assert pipeline.robot_state.pose.x > records[0].robot_state_before.pose.x
    assert records[-1].safety_decision.counters.controller_stop_requests == 0
    assert pipeline.gate.stop_epoch == public_wide_left.build_context.stop_epoch


def test_real_dwb_selected_command_is_rechecked_by_the_external_shared_gate(
    public_wide_left,
) -> None:
    pipeline = _pipeline(public_wide_left, PersistentSourceDerivedDwbController())
    observation, prediction = _fresh_empty(public_wide_left.build_context, 0)

    record = pipeline.step(
        observation_snapshot=observation,
        prediction_set=prediction,
    )

    assert record.controller_result is not None
    assert record.controller_result.status is PersistentControllerStatus.COMMAND_FOUND
    assert record.controller_result.requested_twist != Twist2D()
    assert record.safety_decision.proposal_accepted
    assert record.safety_decision.command == record.controller_result.requested_twist
    assert record.safety_decision.primary_hold_reason is None
    assert len(record.controller_result.predicted_trajectory) == 41


def test_reference_binding_missing_or_tampered_never_applies_motion(
    public_wide_left,
) -> None:
    pipeline = _pipeline(public_wide_left, PersistentRppController())
    observation, prediction = _fresh_empty(public_wide_left.build_context, 0)
    baseline = pipeline.step(
        observation_snapshot=observation,
        prediction_set=prediction,
    )
    binding = baseline.proposal.reference_binding
    assert binding is not None
    changed_window = replace(
        binding,
        window_content_hash="0" * 64,
        binding_content_hash="",
    )
    changed_epoch = replace(binding, stop_epoch=binding.stop_epoch + 1, binding_content_hash="")
    cases = (
        (
            replace(baseline.proposal, reference_binding=None),
            baseline.safety_context,
        ),
        (
            replace(baseline.proposal, reference_binding=changed_window),
            baseline.safety_context,
        ),
        (
            replace(baseline.proposal, reference_binding=changed_epoch),
            replace(baseline.safety_context, reference_binding=changed_epoch),
        ),
    )

    for proposal, context in cases:
        decision = _gate_for(binding).step(
            proposal,
            robot_state=baseline.robot_state_before,
            context=context,
        )
        assert not decision.proposal_accepted
        assert decision.command == Twist2D()
        assert decision.primary_hold_reason is DynamicHoldReason.INVALID_REFERENCE
        assert "reference_binding_mismatch" in decision.failure_reasons


def test_planned_stop_is_not_attributed_as_a_protective_stop(public_wide_left) -> None:
    pipeline = _pipeline(public_wide_left, PersistentRppController())
    observation, prediction = _fresh_empty(public_wide_left.build_context, 0)
    baseline = pipeline.step(
        observation_snapshot=observation,
        prediction_set=prediction,
    )
    planned = replace(
        _require_controller_result(baseline),
        status=PersistentControllerStatus.PLANNED_STOP,
        planned_section_stop=True,
        semantic_content_hash="",
    )
    proposal = persistent_result_to_dynamic_proposal(
        planned,
        tick_input=baseline.tick_input,
        computation_time_s=0.050,
    )

    decision = _gate_for(baseline.safety_context.reference_binding).step(
        proposal,
        robot_state=baseline.robot_state_before,
        context=baseline.safety_context,
    )

    assert decision.proposal_accepted
    assert decision.primary_hold_reason is None
    assert decision.counters.controller_stop_requests == 0
    assert decision.stop_epoch == public_wide_left.build_context.stop_epoch


def _require_controller_result(record):
    assert record.controller_result is not None
    return record.controller_result


def test_old_session_result_and_51ms_result_are_discarded_without_reuse(
    public_wide_left,
) -> None:
    pipeline = _pipeline(public_wide_left, PersistentRppController())
    observation0, prediction0 = _fresh_empty(public_wide_left.build_context, 0)
    first = pipeline.step(observation_snapshot=observation0, prediction_set=prediction0)
    observation1, prediction1 = _fresh_empty(public_wide_left.build_context, 1)
    second = pipeline.step(observation_snapshot=observation1, prediction_set=prediction1)

    old_at_new_tick = replace(first.proposal, computation_time_s=0.049)
    old_decision = _gate_for(second.safety_context.reference_binding).step(
        old_at_new_tick,
        robot_state=second.robot_state_before,
        context=second.safety_context,
    )
    assert not old_decision.proposal_accepted
    assert old_decision.command == Twist2D()
    assert old_decision.primary_hold_reason is DynamicHoldReason.INVALID_REFERENCE
    assert "reference_binding_mismatch" in old_decision.failure_reasons
    assert "late_or_wrong_tick_result" in old_decision.failure_reasons

    late = replace(second.proposal, computation_time_s=0.051)
    late_decision = _gate_for(second.safety_context.reference_binding).step(
        late,
        robot_state=second.robot_state_before,
        context=second.safety_context,
    )
    assert not late_decision.proposal_accepted
    assert late_decision.command == Twist2D()
    assert late_decision.primary_hold_reason is DynamicHoldReason.DEADLINE
    assert "late_or_wrong_tick_result" in late_decision.failure_reasons


def test_stale_reference_bound_input_brakes_and_never_applies_new_motion(
    public_wide_left,
) -> None:
    pipeline = _pipeline(public_wide_left, PersistentRppController())
    observation, prediction = _fresh_empty(public_wide_left.build_context, 0)
    baseline = pipeline.step(observation_snapshot=observation, prediction_set=prediction)
    stale_observation = replace(
        baseline.safety_context.observation_snapshot,
        availability=DynamicObservationAvailability.STALE,
        age_s=0.301,
    )
    stale_context = replace(
        baseline.safety_context,
        simulation_time_s=0.301,
        observation_snapshot=stale_observation,
    )
    moving_state = RobotState(
        baseline.robot_state_before.pose,
        Twist2D(0.10, 0.20),
    )

    decision = _gate_for(stale_context.reference_binding).step(
        baseline.proposal,
        robot_state=moving_state,
        context=stale_context,
    )

    assert not decision.proposal_accepted
    assert decision.primary_hold_reason is DynamicHoldReason.STALE
    assert decision.command.linear == pytest.approx(0.075)
    assert decision.command.angular == pytest.approx(0.12)
    assert abs(decision.command.linear) < abs(moving_state.twist.linear)
    assert abs(decision.command.angular) < abs(moving_state.twist.angular)


def test_protective_stop_epoch_change_holds_without_recalling_the_controller(
    public_wide_left,
) -> None:
    pipeline = _pipeline(public_wide_left, PersistentRppController())
    observation0, prediction0 = _fresh_empty(public_wide_left.build_context, 0)
    first = pipeline.step(observation_snapshot=observation0, prediction_set=prediction0)
    assert first.safety_decision.proposal_accepted

    observation1, prediction1 = _fresh_empty(public_wide_left.build_context, 1)
    stale = replace(
        observation1,
        availability=DynamicObservationAvailability.STALE,
        age_s=0.301,
    )
    stopped = pipeline.step(observation_snapshot=stale, prediction_set=prediction1)
    assert stopped.safety_decision.primary_hold_reason is DynamicHoldReason.STALE

    invalidated = None
    for _ in range(8):
        observation, prediction = _fresh_empty(
            public_wide_left.build_context,
            pipeline.tick_id,
        )
        record = pipeline.step(
            observation_snapshot=observation,
            prediction_set=prediction,
        )
        if record.controller_result is None:
            invalidated = record
            break

    assert invalidated is not None
    assert invalidated.tick_input is None
    assert invalidated.safety_decision.motion_state is DynamicMotionState.HOLDING
    assert invalidated.safety_decision.command == Twist2D()
    assert invalidated.safety_decision.primary_hold_reason is DynamicHoldReason.INVALID_REFERENCE
    assert "reference_binding_mismatch" in invalidated.safety_decision.failure_reasons
    assert pipeline.gate.stop_epoch == public_wide_left.build_context.stop_epoch + 1


def test_pipeline_applies_gate_output_on_next_tick_not_current_interval(
    public_wide_left,
) -> None:
    pipeline = _pipeline(public_wide_left, PersistentRppController())
    observation, prediction = _fresh_empty(public_wide_left.build_context, 0)

    first = pipeline.step(observation_snapshot=observation, prediction_set=prediction)

    assert first.robot_state_before.twist == Twist2D()
    assert first.robot_state_after.pose == first.robot_state_before.pose
    assert first.robot_state_after.twist == first.safety_decision.command
    expected_next_pose = integrate_persistent_chassis_pose(
        first.robot_state_after.pose,
        first.robot_state_after.twist,
        0.05,
    )
    observation1, prediction1 = _fresh_empty(public_wide_left.build_context, 1)
    second = pipeline.step(observation_snapshot=observation1, prediction_set=prediction1)
    assert second.robot_state_after.pose == expected_next_pose
