from __future__ import annotations

import ast
from dataclasses import replace
from math import hypot, pi
from pathlib import Path

import pytest
from test_reference_section_executor import _fixture, _tick_input

from hospital_path_lab.contracts import Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import DynamicMotionState
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.local_reference_contracts import (
    ReferenceKnot,
    ReferenceKnotRole,
    ReferenceSection,
    ReferenceSectionKind,
    ReferenceTravelDirection,
)
from hospital_path_lab.local_reference_reporting import (
    evaluate_local_reference_public_case,
    public_local_reference_cases,
)
from hospital_path_lab.local_reference_window import LocalReferenceWindowManager
from hospital_path_lab.persistent_controller_contracts import (
    PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
    PersistentControllerStatus,
    PersistentControllerTickInput,
    build_persistent_reference_binding,
)
from hospital_path_lab.persistent_rpp_controller import (
    PERSISTENT_RPP_CONTROLLER_NAME,
    PERSISTENT_RPP_ROLLOUT_HORIZON_S,
    PERSISTENT_RPP_ROLLOUT_STEP_S,
    PersistentRppConfig,
    PersistentRppController,
    _compute_translation_command,
    _integrate_pose,
    _post_apply_bounded_stop_rollout,
    _section_has_upcoming_stop,
)
from hospital_path_lab.reference_section_executor import R5_CONTROL_PERIOD_S


@pytest.fixture(scope="module")
def public_wide_left():
    case = next(
        item for item in public_local_reference_cases() if item.public_id == "wide-straight-left"
    )
    result = evaluate_local_reference_public_case(case)
    assert result.hard_failures == ()
    assert len(result.reference_set.candidates) == 1
    return result


def _observation() -> DynamicObservationSnapshot:
    return DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.UNAVAILABLE,
        frame=None,
        age_s=None,
        failures=(),
        last_event_was_no_frame=False,
    )


def _persistent_input(context, reference, window, *, tick, pose, twist):
    current = replace(window, source_control_tick=tick)
    return PersistentControllerTickInput(
        schema_version=PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
        controller_tick=tick,
        simulation_time_s=tick * R5_CONTROL_PERIOD_S,
        full_reference=reference,
        local_window=current,
        reference_binding=build_persistent_reference_binding(reference, current),
        robot_state=RobotState(pose, twist),
        static_grid_snapshot=context.static_grid_snapshot,
        validated_observation=_observation(),
        actor_prediction_set=None,
        vehicle_profile=context.vehicle_profile,
        current_gate_motion_state=DynamicMotionState.MOVING,
        current_gate_stop_epoch=context.stop_epoch,
        current_resume_authorization_revision=None,
    )


def test_rpp_config_is_exactly_frozen_to_existing_research_values() -> None:
    config = PersistentRppConfig()

    assert config.lookahead_min_m == 0.25
    assert config.lookahead_max_m == 0.50
    assert config.lookahead_velocity_gain == 0.75
    assert config.minimum_tracking_speed_mps == 0.05
    assert config.curvature_gain == 2.0
    assert config.rollout_horizon_s == 2.0
    assert config.rollout_step_s == 0.05
    with pytest.raises(ValueError, match="frozen"):
        PersistentRppConfig(lookahead_min_m=0.20)


def test_nonterminal_window_edge_without_stop_marker_is_not_a_goal() -> None:
    knots = (
        ReferenceKnot(
            0,
            Pose2D(0.0, 0.0, 0.0),
            0.0,
            0.0,
            0,
            0,
            (ReferenceKnotRole.TRANSLATION,),
        ),
        ReferenceKnot(
            1,
            Pose2D(0.30, 0.0, 0.0),
            0.0,
            0.30,
            1,
            0,
            (ReferenceKnotRole.TRANSLATION,),
        ),
        ReferenceKnot(
            2,
            Pose2D(1.0, 0.0, 0.0),
            0.0,
            1.0,
            2,
            1,
            (ReferenceKnotRole.TRANSLATION,),
        ),
    )
    sections = (
        ReferenceSection(
            0,
            ReferenceSectionKind.DEPART,
            ReferenceTravelDirection.FORWARD,
            0,
            1,
            False,
            False,
            (0,),
        ),
        ReferenceSection(
            1,
            ReferenceSectionKind.RETURN,
            ReferenceTravelDirection.FORWARD,
            2,
            2,
            False,
            False,
            (1,),
        ),
    )

    assert not _section_has_upcoming_stop(sections, knots, sections[0])


def test_translation_uses_window_for_lookahead_and_full_section_for_progress() -> None:
    context, _seed, reference, _validation, full_window = _fixture()
    section = reference.sections[5]
    start = reference.knots[section.first_knot_index]
    tick_input = _tick_input(
        context,
        reference,
        full_window,
        tick=40,
        pose=start.pose,
        twist=Twist2D(0.20, 0.0),
    )

    command = _compute_translation_command(tick_input, section.section_index, PersistentRppConfig())

    assert command.active_full_progress_m == pytest.approx(start.cumulative_translation_arc_m)
    assert command.active_section_remaining_m == pytest.approx(1.20)
    assert command.lookahead_distance_m == pytest.approx(0.40)
    assert command.lookahead_point.x == pytest.approx(start.pose.x + 0.40)
    assert command.explicit_stop_active
    assert not command.terminal_goal_active
    assert command.command.linear == pytest.approx(0.20)


def test_explicit_section_stop_limit_and_curve_regulation_obey_one_tick_limits() -> None:
    context, _seed, reference, _validation, full_window = _fixture()
    stopped_translation = next(
        section
        for section in reversed(reference.sections)
        if section.travel_direction is ReferenceTravelDirection.FORWARD
    )
    section_end = reference.knots[stopped_translation.last_knot_index].pose
    near_terminal = Pose2D(section_end.x, section_end.y + 0.08, section_end.yaw)
    terminal_input = _tick_input(
        context,
        reference,
        full_window,
        tick=40,
        pose=near_terminal,
        twist=Twist2D(0.20, 0.0),
    )
    terminal_command = _compute_translation_command(
        terminal_input,
        stopped_translation.section_index,
        PersistentRppConfig(),
    )

    assert not terminal_command.terminal_goal_active
    assert terminal_command.explicit_stop_active
    assert terminal_command.stop_limited_speed_mps is not None
    assert terminal_command.command.linear == pytest.approx(0.175)

    curved_section = reference.sections[5]
    curved_start = reference.knots[curved_section.first_knot_index].pose
    curved_input = _tick_input(
        context,
        reference,
        full_window,
        tick=40,
        pose=replace(curved_start, yaw=pi / 4.0),
        twist=Twist2D(0.20, 0.0),
    )
    curved = _compute_translation_command(
        curved_input,
        curved_section.section_index,
        PersistentRppConfig(),
    )

    assert curved.curvature_regulated_speed_mps < 0.20
    assert curved.command.linear == pytest.approx(0.175)
    assert abs(curved.command.angular) <= 0.08 + 1e-12


def test_same_tick_duplicate_returns_identical_41_pose_post_apply_rollout() -> None:
    context, _seed, reference, _validation, window = _fixture()
    tick_input = _tick_input(
        context,
        reference,
        window,
        tick=40,
        pose=reference.knots[0].pose,
        twist=Twist2D(0.10, 0.20),
    )
    controller = PersistentRppController()

    first = controller.step(tick_input)
    duplicate = controller.step(tick_input)

    assert duplicate is first
    assert len(first.predicted_trajectory) == 41
    assert first.predicted_trajectory[0].time_s == 0.0
    assert first.predicted_trajectory[-1].time_s == pytest.approx(PERSISTENT_RPP_ROLLOUT_HORIZON_S)
    assert first.predicted_trajectory[1].time_s == pytest.approx(PERSISTENT_RPP_ROLLOUT_STEP_S)
    expected_post_apply = _integrate_pose(
        tick_input.robot_state.pose,
        tick_input.robot_state.twist,
        R5_CONTROL_PERIOD_S,
    )
    assert first.predicted_trajectory[0].pose == expected_post_apply


def test_planned_stop_rollout_applies_one_interval_then_decelerates_and_holds() -> None:
    context, _seed, reference, _validation, _window = _fixture()
    start = reference.knots[0].pose

    trajectory = _post_apply_bounded_stop_rollout(
        start,
        Twist2D(0.20, 0.08),
        Twist2D(0.20, 0.08),
        linear_deceleration_mps2=context.vehicle_profile.max_deceleration_mps2,
        config=PersistentRppConfig(),
    )

    assert len(trajectory) == 41
    assert trajectory[0].twist == Twist2D(0.20, 0.08)
    assert trajectory[1].twist.linear == pytest.approx(0.175)
    assert trajectory[1].twist.angular == pytest.approx(0.0)
    assert trajectory[-1].twist == Twist2D()
    first_stationary = next(
        index for index, point in enumerate(trajectory) if point.twist == Twist2D()
    )
    assert all(
        point.pose == trajectory[first_stationary].pose
        for point in trajectory[first_stationary + 1 :]
    )


def test_same_tick_changed_input_is_a_zero_invalid_reference_result() -> None:
    context, _seed, reference, _validation, window = _fixture()
    tick_input = _tick_input(
        context,
        reference,
        window,
        tick=40,
        pose=reference.knots[0].pose,
    )
    controller = PersistentRppController()
    controller.step(tick_input)
    changed = replace(
        tick_input,
        robot_state=RobotState(
            replace(tick_input.robot_state.pose, x=tick_input.robot_state.pose.x + 0.01),
            tick_input.robot_state.twist,
        ),
        tick_input_content_hash="",
    )

    rejected = controller.step(changed)

    assert rejected.status is PersistentControllerStatus.INVALID_REFERENCE_INPUT
    assert rejected.requested_twist == Twist2D()
    assert rejected.failure_reason == "same_tick_input_changed"
    assert rejected.controller_requested_protective_stop


def test_public_wide_straight_left_completes_without_subgoal_reset(
    public_wide_left,
) -> None:
    context = public_wide_left.build_context
    reference = public_wide_left.reference_set.candidates[0]
    validation = public_wide_left.validations[0]
    manager = LocalReferenceWindowManager()
    controller = PersistentRppController()
    pose = reference.knots[0].pose
    twist = Twist2D()
    outputs = []
    actual_twists = []
    maximum_tracking_error = 0.0
    subgoal_revisions = []

    for tick in range(1_400):
        current_context = replace(
            context,
            current_robot_pose=pose,
            control_tick=tick,
            simulation_time_s=tick * R5_CONTROL_PERIOD_S,
            context_content_hash="",
        )
        update = manager.update(current_context, reference, validation)
        assert update.window is not None, update.reason_code
        subgoal_revisions.append(update.window.subgoal_revision)
        tick_input = _persistent_input(
            context,
            reference,
            update.window,
            tick=tick,
            pose=pose,
            twist=twist,
        )
        result = controller.step(tick_input)
        if tick == 0:
            assert controller.step(tick_input) is result
        outputs.append(result)
        actual_twists.append(twist)
        maximum_tracking_error = max(
            maximum_tracking_error,
            0.0 if result.tracking_error_m is None else result.tracking_error_m,
        )
        assert not result.controller_requested_protective_stop, result.failure_reason
        if result.status is PersistentControllerStatus.COMPLETED:
            break
        pose = _integrate_pose(pose, twist, R5_CONTROL_PERIOD_S)
        twist = result.requested_twist
    else:
        pytest.fail("persistent RPP did not complete wide-straight-left")

    terminal = reference.knots[-1].pose
    assert outputs[-1].status is PersistentControllerStatus.COMPLETED
    assert controller.name == PERSISTENT_RPP_CONTROLLER_NAME
    assert controller.session_reset_count == 1
    assert controller.window_update_count >= 1
    assert max(subgoal_revisions) >= 1
    assert controller.false_local_goal_deceleration_count == 0
    assert hypot(pose.x - terminal.x, pose.y - terminal.y) <= 0.05
    assert twist == Twist2D()
    assert any(output.status is PersistentControllerStatus.COMMAND_FOUND for output in outputs)
    assert any(output.status is PersistentControllerStatus.PLANNED_STOP for output in outputs)
    assert all(
        output.requested_twist.linear == 0.0
        for output in outputs
        if output.active_section_kind is ReferenceSectionKind.ROTATE
    )
    translation = next(output for output in outputs if output.tracking_error_m is not None)
    assert len(translation.predicted_trajectory) == 41
    assert "local_window_endpoint_is_not_goal=true" in translation.decision_trace
    reverse_outputs = [
        output
        for output in outputs
        if output.active_section_index is not None
        and reference.sections[output.active_section_index].travel_direction
        is ReferenceTravelDirection.REVERSE
    ]
    assert reverse_outputs
    assert any(output.requested_twist.linear < 0.0 for output in reverse_outputs)
    assert all(output.requested_twist.linear <= 0.0 for output in reverse_outputs)
    assert min(output.requested_twist.linear for output in reverse_outputs) >= -0.10
    first_reverse_command = next(
        index
        for index, output in enumerate(outputs)
        if output.requested_twist.linear < 0.0
    )
    assert first_reverse_command >= 3
    assert all(
        abs(actual.linear) <= 0.01 and abs(actual.angular) <= 0.02
        for actual in actual_twists[first_reverse_command - 3 : first_reverse_command]
    )
    assert maximum_tracking_error <= 0.10


def test_rpp_controller_has_no_corpus_or_evaluator_label_channel() -> None:
    module_path = (
        Path(__file__).parents[1] / "src" / "hospital_path_lab" / "persistent_rpp_controller.py"
    )
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    forbidden = {
        "dynamic_corpus",
        "expectation_category",
        "oracle_spec",
        "latent_case_id",
        "hidden_seed",
        "ground_truth_actor",
        "feasible_witness",
        "evaluator",
    }
    names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
    attributes = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    imported_modules = {
        node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
    }

    assert forbidden.isdisjoint(names | attributes)
    assert not any(any(token in module for token in forbidden) for module in imported_modules)
