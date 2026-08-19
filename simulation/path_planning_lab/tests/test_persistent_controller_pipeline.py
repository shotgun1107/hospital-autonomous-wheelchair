from __future__ import annotations

from dataclasses import replace
from math import atan2, cos, hypot, sin

import pytest

from hospital_path_lab.contracts import Pose2D, RobotState, Twist2D
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
from hospital_path_lab.local_algorithms.dwb_reference.cpp_full_core import (
    CPP_DWB_FULL_CORE_AVAILABLE,
)
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PersistentSourceDerivedDwbController,
)
from hospital_path_lab.local_reference_contracts import (
    ReferenceSectionKind,
    ReferenceTravelDirection,
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
from hospital_path_lab.r5b_restop_execution import build_world_follow_reference
from hospital_path_lab.r5b_temporal_reference import (
    R5B_REFERENCE_MISSION_ID,
    build_r5b_crossing_reference_bundles,
)


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


def test_rpp_reaches_the_first_planned_stop_without_a_false_terminal_tail_rejection(
    public_wide_left,
) -> None:
    pipeline = _pipeline(public_wide_left, PersistentRppController())
    saw_planned_stop = False

    for _ in range(180):
        observation, prediction = _fresh_empty(
            public_wide_left.build_context,
            pipeline.tick_id,
        )
        record = pipeline.step(
            observation_snapshot=observation,
            prediction_set=prediction,
        )
        assert "static_clearance_below_minimum" not in (
            record.safety_decision.failure_reasons
        )
        assert record.safety_decision.primary_hold_reason is not DynamicHoldReason.GATE_REJECTION
        if (
            record.controller_result is not None
            and record.controller_result.status is PersistentControllerStatus.PLANNED_STOP
        ):
            saw_planned_stop = True
            assert record.safety_decision.proposal_accepted
            break

    assert saw_planned_stop


def test_rpp_completes_public_wide_left_through_the_external_shared_gate(
    public_wide_left,
) -> None:
    pipeline = _pipeline(public_wide_left, PersistentRppController())
    statuses = []
    reverse_commands = []
    reference = public_wide_left.reference_set.candidates[0]

    for _ in range(700):
        observation, prediction = _fresh_empty(
            public_wide_left.build_context,
            pipeline.tick_id,
        )
        record = pipeline.step(
            observation_snapshot=observation,
            prediction_set=prediction,
        )
        assert record.controller_result is not None
        statuses.append(record.controller_result.status)
        if record.controller_result.active_section_index is not None:
            direction = reference.sections[
                record.controller_result.active_section_index
            ].travel_direction
            if direction is ReferenceTravelDirection.REVERSE:
                reverse_commands.append(record.controller_result.requested_twist.linear)
        assert record.safety_decision.primary_hold_reason is None
        assert record.safety_decision.failure_reasons == ()
        assert record.safety_decision.counters.candidate_rejected_by_gate == 0
        if record.safety_decision.motion_state is DynamicMotionState.COMPLETED:
            break
    else:
        pytest.fail("persistent RPP did not complete through the external shared gate")

    terminal = public_wide_left.reference_set.candidates[0].knots[-1].pose
    assert PersistentControllerStatus.COMMAND_FOUND in statuses
    assert PersistentControllerStatus.PLANNED_STOP in statuses
    assert PersistentControllerStatus.COMPLETED in statuses
    assert reverse_commands
    assert min(reverse_commands) < 0.0
    assert all(-0.10 <= value <= 0.0 for value in reverse_commands)
    assert pipeline.robot_state.twist == Twist2D()
    assert pipeline.robot_state.pose.x == pytest.approx(terminal.x, abs=0.05)
    assert pipeline.robot_state.pose.y == pytest.approx(terminal.y, abs=0.05)


def test_real_dwb_selected_command_is_rechecked_by_the_external_shared_gate(
    public_wide_left,
) -> None:
    pipeline = _pipeline(public_wide_left, PersistentSourceDerivedDwbController())
    record = None
    for _ in range(80):
        observation, prediction = _fresh_empty(
            public_wide_left.build_context,
            pipeline.tick_id,
        )
        candidate = pipeline.step(
            observation_snapshot=observation,
            prediction_set=prediction,
        )
        if (
            candidate.controller_result is not None
            and candidate.controller_result.status
            is PersistentControllerStatus.COMMAND_FOUND
        ):
            record = candidate
            break

    assert record is not None
    assert record.controller_result is not None
    assert record.controller_result.status is PersistentControllerStatus.COMMAND_FOUND
    assert record.controller_result.requested_twist != Twist2D()
    assert record.safety_decision.proposal_accepted
    assert record.safety_decision.command == record.controller_result.requested_twist
    assert record.safety_decision.primary_hold_reason is None
    assert len(record.controller_result.predicted_trajectory) == 41


def test_dwb_rejects_an_unproven_mid_route_restart(
    public_wide_left,
) -> None:
    context = public_wide_left.build_context
    reference = public_wide_left.reference_set.candidates[0]
    pipeline = PersistentControllerPipeline(
        controller=PersistentSourceDerivedDwbController(),
        build_context=context,
        full_reference=reference,
        validation=public_wide_left.validations[0],
        initial_robot_state=RobotState(
            Pose2D(1.38, 0.8933, -0.0304),
            Twist2D(),
        ),
        initial_tick=250,
    )
    observation, prediction = _fresh_empty(context, pipeline.tick_id)

    record = pipeline.step(
        observation_snapshot=observation,
        prediction_set=prediction,
    )

    assert record.controller_result is not None
    assert (
        record.controller_result.status
        is PersistentControllerStatus.SECTION_EXECUTION_FAILED
    )
    assert record.controller_result.requested_twist == Twist2D()
    assert not record.safety_decision.proposal_accepted
    assert record.robot_state_after == record.robot_state_before


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


@pytest.fixture(scope="module")
def terminal_follow_bundle():
    source = build_r5b_crossing_reference_bundles()[1]
    world = source.source.world
    goal = source.reference.knots[-1].pose
    stalled_pose = Pose2D(4.314218, 2.306159, 0.239733)
    travel_yaw = atan2(goal.y - stalled_pose.y, goal.x - stalled_pose.x)
    start = Pose2D(
        goal.x - 0.50 * cos(travel_yaw),
        goal.y - 0.50 * sin(travel_yaw),
        travel_yaw,
    )
    bundle = build_world_follow_reference(
        world,
        mission_id=R5B_REFERENCE_MISSION_ID,
        current_pose=start,
        stop_epoch=3,
        valid_from_tick=422,
        identity={"public_regression": "high_speed_terminal_closed_loop"},
        generation_reason_codes=("high_speed_terminal_closed_loop",),
        goal_pose=goal,
    )
    assert tuple(section.section_kind for section in bundle.reference.sections) == (
        ReferenceSectionKind.FOLLOW_ORIGINAL,
        ReferenceSectionKind.ROTATE,
    )
    return bundle, travel_yaw


def _terminal_pipeline(bundle, travel_yaw: float, *, distance_m: float, speed_mps: float):
    terminal = bundle.reference.knots[bundle.reference.sections[0].last_knot_index].pose
    pose = Pose2D(
        terminal.x - distance_m * cos(travel_yaw),
        terminal.y - distance_m * sin(travel_yaw),
        travel_yaw,
    )
    return PersistentControllerPipeline(
        controller=PersistentSourceDerivedDwbController(
            use_cpp_safety_core=True,
            use_cpp_full_core=True,
        ),
        build_context=bundle.build_context,
        full_reference=bundle.reference,
        validation=bundle.validation,
        initial_robot_state=RobotState(pose, Twist2D(speed_mps, 0.0)),
        initial_tick=422,
    )


def _run_terminal_pipeline(pipeline, build_context, *, tick_count: int):
    records = []
    for _ in range(tick_count):
        observation, prediction = _fresh_empty(build_context, pipeline.tick_id)
        record = pipeline.step(
            observation_snapshot=observation,
            prediction_set=prediction,
        )
        records.append(record)
        if record.safety_decision.motion_state in {
            DynamicMotionState.COMPLETED,
            DynamicMotionState.HOLDING,
        }:
            break
    return tuple(records)


@pytest.mark.skipif(
    not CPP_DWB_FULL_CORE_AVAILABLE,
    reason="optional C++ full DWB core has not been built",
)
def test_max_speed_terminal_approach_decelerates_rotates_and_completes(
    terminal_follow_bundle,
) -> None:
    bundle, travel_yaw = terminal_follow_bundle
    pipeline = _terminal_pipeline(
        bundle,
        travel_yaw,
        distance_m=0.30,
        speed_mps=0.30,
    )
    records = _run_terminal_pipeline(
        pipeline,
        bundle.build_context,
        tick_count=180,
    )
    terminal = bundle.reference.knots[bundle.reference.sections[0].last_knot_index].pose

    assert records[-1].safety_decision.motion_state is DynamicMotionState.COMPLETED
    assert any(
        record.controller_result is not None
        and record.controller_result.status is PersistentControllerStatus.PLANNED_STOP
        for record in records
    )
    rotation_records = tuple(
        record
        for record in records
        if record.controller_result is not None
        and record.controller_result.active_section_kind is ReferenceSectionKind.ROTATE
    )
    assert rotation_records
    first_rotation = rotation_records[0]
    assert hypot(
        first_rotation.robot_state_before.pose.x - terminal.x,
        first_rotation.robot_state_before.pose.y - terminal.y,
    ) <= 0.05 + 1e-12
    assert all(
        "static_clearance_below_minimum" not in record.safety_decision.failure_reasons
        and "forbidden_zone_entry" not in record.safety_decision.failure_reasons
        and "actor_clearance_below_minimum" not in record.safety_decision.failure_reasons
        for record in records
    )
    deceleration_steps = tuple(
        record
        for record in records
        if abs(record.safety_decision.command.linear)
        < abs(record.robot_state_before.twist.linear) - 1e-12
    )
    assert deceleration_steps
    assert all(
        abs(record.robot_state_before.twist.linear)
        - abs(record.safety_decision.command.linear)
        <= 0.025 + 1e-12
        for record in deceleration_steps
    )
    assert pipeline.robot_state.twist == Twist2D()


@pytest.mark.skipif(
    not CPP_DWB_FULL_CORE_AVAILABLE,
    reason="optional C++ full DWB core has not been built",
)
def test_unstoppable_close_terminal_approach_fails_closed_instead_of_forcing_completion(
    terminal_follow_bundle,
) -> None:
    bundle, travel_yaw = terminal_follow_bundle
    pipeline = _terminal_pipeline(
        bundle,
        travel_yaw,
        distance_m=0.08,
        speed_mps=0.30,
    )
    records = _run_terminal_pipeline(
        pipeline,
        bundle.build_context,
        tick_count=40,
    )

    assert records[-1].safety_decision.motion_state is DynamicMotionState.HOLDING
    assert all(
        record.safety_decision.motion_state is not DynamicMotionState.COMPLETED
        for record in records
    )
    braking = tuple(
        record
        for record in records
        if record.safety_decision.motion_state is DynamicMotionState.BRAKING
    )
    assert braking
    assert all(
        abs(record.safety_decision.command.linear)
        <= abs(record.robot_state_before.twist.linear) + 1e-12
        for record in braking
    )
    assert all(
        abs(record.robot_state_before.twist.linear)
        - abs(record.safety_decision.command.linear)
        <= 0.025 + 1e-12
        for record in braking
        if abs(record.safety_decision.command.linear)
        < abs(record.robot_state_before.twist.linear) - 1e-12
    )
    assert all(
        "static_clearance_below_minimum" not in record.safety_decision.failure_reasons
        and "forbidden_zone_entry" not in record.safety_decision.failure_reasons
        and "actor_clearance_below_minimum" not in record.safety_decision.failure_reasons
        for record in records
    )
    assert pipeline.robot_state.twist == Twist2D()
