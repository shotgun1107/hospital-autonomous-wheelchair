from __future__ import annotations

import ast
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from hospital_path_lab.contracts import Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DynamicMotionState,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import build_actor_prediction_set
from hospital_path_lab.dynamic_safety import DynamicSafetyContext, DynamicSafetyGate
from hospital_path_lab.local_algorithms.dwb_reference.composition import (
    SourceDerivedDwbConfig,
)
from hospital_path_lab.local_algorithms.dwb_reference.contracts import (
    DwbGeneratorConfig,
    DwbGeneratorRequest,
    DwbPose2D,
    DwbTwist2D,
)
from hospital_path_lab.local_algorithms.dwb_reference.core import (
    DwbCriticBinding,
    DwbReferenceCore,
)
from hospital_path_lab.local_algorithms.dwb_reference.critics import (
    DwbCriticGrid,
    GoalAlignCritic,
    GoalDistCritic,
    OscillationCritic,
    PathAlignCritic,
    PathDistCritic,
    RotateToGoalCritic,
)
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PERSISTENT_DWB_CONTROLLER_NAME,
    R5_DWB_BYPASS_SCORING_LOOKAHEAD_M,
    PersistentDwbCoreSession,
    PersistentSourceDerivedDwbController,
    SectionBoundDwbReferenceTrajectoryGenerator,
    _active_translation_dwb_path,
    _active_translation_dwb_scoring_path,
    _aligned_forward_section,
    _connector_tightened_forward_section,
)
from hospital_path_lab.local_algorithms.dwb_reference.trajectory_generator import (
    DwbReferenceTrajectoryGenerator,
)
from hospital_path_lab.local_reference_contracts import (
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
from hospital_path_lab.persistent_controller_pipeline import (
    persistent_result_to_dynamic_proposal,
)
from hospital_path_lab.r5b_temporal_evidence import frozen_r2_archive_path
from hospital_path_lab.r5b_temporal_reference import (
    build_r5b_temporal_reference_bundles,
)
from hospital_path_lab.reference_section_executor import (
    R5_CONTROL_PERIOD_S,
    ReferenceSectionExecutor,
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


def _fresh_empty_input(context, reference, window, *, tick, pose, twist=None):
    if twist is None:
        twist = Twist2D()
    current = replace(window, source_control_tick=tick)
    metadata = context.static_grid_snapshot.metadata
    frame = DynamicObservationFrame(
        stream_id="r5-public-empty-stream",
        episode_id="r5-public-empty-episode",
        episode_seed=metadata.seed,
        map_id=metadata.map_id,
        map_revision=metadata.map_revision,
        observation_revision=metadata.observation_revision,
        sequence=tick,
        observed_at_s=tick * R5_CONTROL_PERIOD_S,
        delivered_at_s=tick * R5_CONTROL_PERIOD_S,
        frame_kind=DynamicObservationFrameKind.EMPTY,
        tracks=(),
        content_hash=f"r5-public-empty-{tick}",
    )
    observation = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.FRESH,
        frame=frame,
        age_s=0.0,
        failures=(),
        last_event_was_no_frame=False,
    )
    prediction = build_actor_prediction_set(observation)
    return PersistentControllerTickInput(
        schema_version=PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
        controller_tick=tick,
        simulation_time_s=tick * R5_CONTROL_PERIOD_S,
        full_reference=reference,
        local_window=current,
        reference_binding=build_persistent_reference_binding(reference, current),
        robot_state=RobotState(pose, twist),
        static_grid_snapshot=context.static_grid_snapshot,
        validated_observation=observation,
        actor_prediction_set=prediction,
        vehicle_profile=context.vehicle_profile,
        current_gate_motion_state=DynamicMotionState.MOVING,
        current_gate_stop_epoch=context.stop_epoch,
        current_resume_authorization_revision=None,
    )


def test_forward_goal_align_stays_active_until_short_section_heading_is_aligned() -> None:
    assert not _aligned_forward_section(
        ReferenceTravelDirection.FORWARD,
        0.278,
    )
    assert _aligned_forward_section(
        ReferenceTravelDirection.FORWARD,
        0.08,
    )
    assert not _aligned_forward_section(
        ReferenceTravelDirection.REVERSE,
        0.0,
    )
    assert _connector_tightened_forward_section(
        ReferenceTravelDirection.FORWARD,
        0.027,
    )
    assert not _connector_tightened_forward_section(
        ReferenceTravelDirection.FORWARD,
        0.05,
    )


def _advance_to_first_signed_translation(
    controller,
    context,
    reference,
    window,
    *,
    unavailable_on_translation: bool = False,
):
    start = reference.knots[0].pose
    first_rotation = next(
        section
        for section in reference.sections
        if section.travel_direction is ReferenceTravelDirection.NONE
        and section.first_knot_index != section.last_knot_index
    )
    rotated = reference.knots[first_rotation.last_knot_index].pose
    result = None
    for tick, pose in enumerate((start, start, start, rotated, rotated, rotated, rotated)):
        tick_input = _fresh_empty_input(
            context,
            reference,
            window,
            tick=tick,
            pose=pose,
        )
        if unavailable_on_translation and tick == 6:
            tick_input = replace(
                tick_input,
                validated_observation=DynamicObservationSnapshot(
                    availability=DynamicObservationAvailability.UNAVAILABLE,
                    frame=None,
                    age_s=None,
                    failures=(),
                    last_event_was_no_frame=False,
                ),
                actor_prediction_set=None,
                tick_input_content_hash="",
            )
        result = controller.step(tick_input)
    assert result is not None
    return result


@pytest.fixture(scope="module")
def r5b_first_left():
    repository_root = Path(__file__).resolve().parents[3]
    return build_r5b_temporal_reference_bundles(
        frozen_r2_archive_path(repository_root)
    )[0]


@pytest.fixture(scope="module")
def public_crossing_left():
    case = next(
        item
        for item in public_local_reference_cases()
        if item.public_id == "crossing-static-left"
    )
    result = evaluate_local_reference_public_case(case)
    assert result.hard_failures == ()
    assert len(result.reference_set.candidates) == 1
    return result


def _advance_to_first_reverse_translation(context, reference, window):
    executor = ReferenceSectionExecutor()
    controller = PersistentSourceDerivedDwbController(executor=executor)
    for tick in range(80):
        index = executor.active_section_index
        section = None if index is None else reference.sections[index]
        if section is None:
            pose = reference.knots[0].pose
        elif section.travel_direction is ReferenceTravelDirection.REVERSE:
            pose = reference.knots[section.first_knot_index].pose
        else:
            pose = reference.knots[section.last_knot_index].pose
        tick_input = _fresh_empty_input(
            context,
            reference,
            window,
            tick=tick,
            pose=pose,
        )
        result = controller.step(tick_input)
        if (
            result.status is PersistentControllerStatus.COMMAND_FOUND
            and result.active_section_index is not None
            and reference.sections[result.active_section_index].travel_direction
            is ReferenceTravelDirection.REVERSE
        ):
            return controller, result, tick_input
    pytest.fail("persistent DWB never reached the first reverse translation")


def _session_core():
    grid = DwbCriticGrid(100, 100, 0.02, 0.0, 0.0, frozenset())
    rotate = RotateToGoalCritic()
    oscillation = OscillationCritic()
    goal_align = GoalAlignCritic(grid)
    path_align = PathAlignCritic(grid)
    path_dist = PathDistCritic(grid)
    goal_dist = GoalDistCritic(grid)
    bindings = (
        DwbCriticBinding("rotate_to_goal", rotate, 1.0),
        DwbCriticBinding("oscillation", oscillation, 1.0),
        DwbCriticBinding("goal_align", goal_align, 1.0),
        DwbCriticBinding("path_align", path_align, 1.0),
        DwbCriticBinding("path_dist", path_dist, 1.0),
        DwbCriticBinding("goal_dist", goal_dist, 1.0),
    )
    core = DwbReferenceCore(DwbReferenceTrajectoryGenerator(), bindings)
    session = PersistentDwbCoreSession(
        core,
        scoring_critics=(goal_align, path_align, path_dist, goal_dist),
        rotate_to_goal_critic=rotate,
        oscillation_critic=oscillation,
    )
    return session, rotate, oscillation, (goal_align, path_align, path_dist, goal_dist)


def test_session_reset_and_scoring_window_lifetimes_are_separate() -> None:
    session, rotate, oscillation, scoring = _session_core()
    full = (DwbPose2D(0.0, 0.0, 0.0), DwbPose2D(3.0, 0.0, 0.0))
    first = (DwbPose2D(0.0, 0.0, 0.0), DwbPose2D(1.0, 0.0, 0.0))
    second = (DwbPose2D(0.5, 0.0, 0.0), DwbPose2D(1.5, 0.0, 0.0))

    session.begin_reference_session(full, first)
    rotate.prepare(DwbGeneratorRequest(first[-1], DwbTwist2D(0.0, 0.0)))
    assert not rotate.in_window
    request = DwbGeneratorRequest(first[0], DwbTwist2D(0.0, 0.0))
    oscillation.prepare(request)
    oscillation.debrief(DwbTwist2D(0.0, 0.2))
    oscillation.prepare(request)
    oscillation.debrief(DwbTwist2D(0.0, -0.2))
    assert oscillation.has_restrictions

    session.update_scoring_window(second)

    assert rotate.path == full
    assert all(critic.path == second for critic in scoring)
    assert oscillation.has_restrictions
    assert session.diagnostics.session_reset_count == 1
    assert session.diagnostics.scoring_window_update_count == 2
    assert session.diagnostics.full_terminal_goal == full[-1]
    assert session.critic_names == (
        "rotate_to_goal",
        "oscillation",
        "goal_align",
        "path_align",
        "path_dist",
        "goal_dist",
    )


def test_same_scoring_window_replay_is_idempotent() -> None:
    session, _rotate, _oscillation, _scoring = _session_core()
    full = (DwbPose2D(0.0, 0.0, 0.0), DwbPose2D(3.0, 0.0, 0.0))
    local = (DwbPose2D(0.0, 0.0, 0.0), DwbPose2D(1.0, 0.0, 0.0))
    session.begin_reference_session(full, local)

    session.set_path(local)
    session.update_scoring_window(local)

    assert session.diagnostics.session_reset_count == 1
    assert session.diagnostics.scoring_window_update_count == 1


def test_default_generator_contract_keeps_upstream_zero_insertion_bound() -> None:
    config = SourceDerivedDwbConfig().generator

    assert config.linear_sample_count == 7
    assert config.angular_sample_count == 31
    assert config.rollout_duration_s == 2.0
    assert config.integration_step_s == 0.05
    assert config.rollout_step_count == 40
    assert not config.allow_reverse
    assert (config.linear_sample_count + 1) * (config.angular_sample_count + 1) == 256


def test_section_bound_generator_never_samples_the_opposite_translation_sign() -> None:
    generator = SectionBoundDwbReferenceTrajectoryGenerator(
        replace(DwbGeneratorConfig(), allow_reverse=True)
    )
    request = DwbGeneratorRequest(
        pose=DwbPose2D(0.0, 0.0, 0.0),
        current_twist=DwbTwist2D(0.0, 0.0),
    )

    generator.set_travel_direction(ReferenceTravelDirection.FORWARD)
    forward = generator.generate(request)
    generator.set_travel_direction(ReferenceTravelDirection.REVERSE)
    reverse = generator.generate(request)

    assert all(value >= 0.0 for value in forward.linear_samples_mps)
    assert all(-0.10 <= value <= 0.0 for value in reverse.linear_samples_mps)
    assert any(value < 0.0 for value in reverse.linear_samples_mps)


def test_aligned_forward_progress_tie_order_only_reverses_linear_blocks() -> None:
    generator = SectionBoundDwbReferenceTrajectoryGenerator(
        replace(DwbGeneratorConfig(), allow_reverse=True)
    )
    request = DwbGeneratorRequest(
        pose=DwbPose2D(0.0, 0.0, 0.0),
        current_twist=DwbTwist2D(0.0, 0.0),
    )

    generator.set_travel_direction(ReferenceTravelDirection.FORWARD)
    ordinary = generator.generate(request)
    generator.set_prefer_forward_progress_on_exact_ties(True)
    connector = generator.generate(request)

    assert connector.linear_samples_mps == tuple(reversed(ordinary.linear_samples_mps))
    assert connector.angular_samples_radps == ordinary.angular_samples_radps
    assert len(connector.trajectories) == len(ordinary.trajectories)
    assert {trajectory.command for trajectory in connector.trajectories} == {
        trajectory.command for trajectory in ordinary.trajectories
    }
    assert connector.trajectories[0].command.linear_mps == max(
        ordinary.linear_samples_mps
    )

    generator.set_prefer_forward_progress_on_exact_ties(False)
    assert generator.generate(request) == ordinary


def test_bypass_scoring_lookahead_does_not_change_the_executable_reference(
    r5b_first_left,
) -> None:
    context = r5b_first_left.build_context
    reference = r5b_first_left.reference
    validation = r5b_first_left.validation
    initial = LocalReferenceWindowManager().update(context, reference, validation)
    assert initial.window is not None
    full_window = replace(
        initial.window,
        end_knot_index=reference.knots[-1].knot_index,
        knots=reference.knots,
        sections=reference.sections,
        terminal_rejoin_included=True,
        window_content_hash="",
    )
    bypass = next(
        section
        for section in reference.sections
        if section.section_kind is ReferenceSectionKind.BYPASS
    )
    tick_input = SimpleNamespace(
        full_reference=reference,
        local_window=full_window,
    )

    executable = _active_translation_dwb_path(
        tick_input,
        bypass.section_index,
    )
    scoring = _active_translation_dwb_scoring_path(
        tick_input,
        bypass.section_index,
    )

    assert scoring[:-1] == executable
    assert len(scoring) == len(executable) + 1
    distance = (
        (scoring[-1].x_m - executable[-1].x_m) ** 2
        + (scoring[-1].y_m - executable[-1].y_m) ** 2
    ) ** 0.5
    assert distance == pytest.approx(R5_DWB_BYPASS_SCORING_LOOKAHEAD_M)
    assert reference.knots[bypass.last_knot_index].pose.x == pytest.approx(
        executable[-1].x_m
    )


def test_crossing_final_forward_section_does_not_stall_at_minimum_speed(
    public_crossing_left,
) -> None:
    context = public_crossing_left.build_context
    reference = public_crossing_left.reference_set.candidates[0]
    validation = public_crossing_left.validations[0]
    initial = LocalReferenceWindowManager().update(context, reference, validation)
    assert initial.window is not None
    full_window = replace(
        initial.window,
        end_knot_index=reference.knots[-1].knot_index,
        knots=reference.knots,
        sections=reference.sections,
        terminal_rejoin_included=True,
        window_content_hash="",
    )
    executor = ReferenceSectionExecutor()
    controller = PersistentSourceDerivedDwbController(executor=executor)

    tick = 0
    while executor.active_section_index != 7 and tick < 100:
        section = reference.sections[executor.active_section_index or 0]
        pose = reference.knots[section.last_knot_index].pose
        controller.step(
            _fresh_empty_input(
                context,
                reference,
                full_window,
                tick=tick,
                pose=pose,
            )
        )
        tick += 1
    assert executor.active_section_index == 7

    result = controller.step(
        _fresh_empty_input(
            context,
            reference,
            full_window,
            tick=tick,
            pose=Pose2D(4.306496036677228, 2.361426434749244, 0.00479997302950654),
            twist=Twist2D(linear=0.0025, angular=0.0),
        )
    )

    assert result.status is PersistentControllerStatus.COMMAND_FOUND
    assert result.requested_twist.linear > 0.0025, result.candidate_diagnostics


def test_public_reverse_section_selects_only_a_bounded_negative_dwb_command(
    public_wide_left,
) -> None:
    context = public_wide_left.build_context
    reference = public_wide_left.reference_set.candidates[0]
    validation = public_wide_left.validations[0]
    initial = LocalReferenceWindowManager().update(context, reference, validation)
    assert initial.window is not None
    full_window = replace(
        initial.window,
        end_knot_index=reference.knots[-1].knot_index,
        knots=reference.knots,
        sections=reference.sections,
        terminal_rejoin_included=True,
        window_content_hash="",
    )

    controller, result, tick_input = _advance_to_first_reverse_translation(
        context,
        reference,
        full_window,
    )

    assert -0.10 <= result.requested_twist.linear < 0.0
    assert "travel_direction=reverse" in result.decision_trace
    assert len(result.predicted_trajectory) == 41
    assert result.predicted_trajectory[-1].pose != result.predicted_trajectory[0].pose
    assert controller.selected_safety_evidence is not None
    assert controller.selected_safety_evidence.safe

    proposal = persistent_result_to_dynamic_proposal(
        result,
        tick_input=tick_input,
        computation_time_s=R5_CONTROL_PERIOD_S,
    )
    gate = DynamicSafetyGate(initial_stop_epoch=context.stop_epoch)
    decision = gate.step(
        proposal,
        robot_state=tick_input.robot_state,
        context=DynamicSafetyContext(
            tick_id=tick_input.controller_tick,
            simulation_time_s=tick_input.simulation_time_s,
            mission_id=reference.mission_id,
            authorization_revision=0,
            grid_snapshot=tick_input.static_grid_snapshot,
            observation_snapshot=tick_input.validated_observation,
            prediction_set=tick_input.actor_prediction_set,
            path_still_valid=True,
            local_safety_recheck_passed=True,
            observation_safe=True,
            resume_authorization=None,
            goal_reached=False,
            mission_cancelled=False,
            reference_binding=tick_input.reference_binding,
        ),
    )
    assert decision.proposal_accepted
    assert decision.command == result.requested_twist
    assert decision.command.linear < 0.0
    assert decision.failure_reasons == ()


def test_short_reverse_section_does_not_prefer_zero_after_yaw_alignment(
    public_wide_left,
) -> None:
    context = public_wide_left.build_context
    reference = public_wide_left.reference_set.candidates[0]
    validation = public_wide_left.validations[0]
    initial = LocalReferenceWindowManager().update(context, reference, validation)
    assert initial.window is not None
    full_window = replace(
        initial.window,
        end_knot_index=reference.knots[-1].knot_index,
        knots=reference.knots,
        sections=reference.sections,
        terminal_rejoin_included=True,
        window_content_hash="",
    )
    controller, _first, first_input = _advance_to_first_reverse_translation(
        context,
        reference,
        full_window,
    )
    aligned_input = _fresh_empty_input(
        context,
        reference,
        full_window,
        tick=first_input.controller_tick + 1,
        pose=Pose2D(
            1.5201507246898855,
            0.9384748649171547,
            1.7716896432411682,
        ),
        twist=Twist2D(),
    )

    result = controller.step(aligned_input)

    assert result.status is PersistentControllerStatus.COMMAND_FOUND
    assert -0.10 <= result.requested_twist.linear < 0.0
    assert "travel_direction=reverse" in result.decision_trace
    assert any(
        item == "selected_critic.path_align=0x0.0p+0*0x1.47ae147ae147bp-2"
        for item in result.candidate_diagnostics
    )


def test_public_wide_first_tick_has_complete_candidate_diagnostics(public_wide_left) -> None:
    context = public_wide_left.build_context
    reference = public_wide_left.reference_set.candidates[0]
    validation = public_wide_left.validations[0]
    pose = reference.knots[0].pose
    current_context = replace(
        context,
        current_robot_pose=pose,
        control_tick=0,
        simulation_time_s=0.0,
        context_content_hash="",
    )
    update = LocalReferenceWindowManager().update(current_context, reference, validation)
    assert update.window is not None
    controller = PersistentSourceDerivedDwbController()

    result = _advance_to_first_signed_translation(
        controller,
        context,
        reference,
        update.window,
    )

    assert result.status is PersistentControllerStatus.COMMAND_FOUND
    assert result.controller_name == PERSISTENT_DWB_CONTROLLER_NAME
    assert controller.session_reset_count == 1
    assert controller.stack_build_count == 1
    diagnostics = controller.dwb_session_diagnostics
    assert diagnostics is not None
    assert diagnostics.session_reset_count == 1
    assert diagnostics.scoring_window_update_count == 1
    assert diagnostics.full_terminal_goal == DwbPose2D(
        reference.knots[-1].pose.x,
        reference.knots[-1].pose.y,
        reference.knots[-1].pose.yaw,
    )
    assert len(result.predicted_trajectory) == 41
    assert "candidate_count=217" in result.candidate_diagnostics
    assert any(item.startswith("legal_candidates=") for item in result.candidate_diagnostics)
    assert any(
        item.startswith("selected_candidate_index=") for item in result.candidate_diagnostics
    )
    assert "terminal_goal_source=immutable_full_reference" in result.candidate_diagnostics
    assert "local_window_endpoint_is_not_rotate_goal=true" in result.decision_trace
    assert "scoring_path_source=active_translation_section" in result.decision_trace
    assert controller.selected_safety_evidence is not None
    assert controller.selected_safety_evidence.safe

    assert "travel_direction=forward" in result.decision_trace


def test_cpp_safety_batch_preserves_first_public_dwb_command(public_wide_left) -> None:
    context = public_wide_left.build_context
    reference = public_wide_left.reference_set.candidates[0]
    validation = public_wide_left.validations[0]
    pose = reference.knots[0].pose
    current_context = replace(
        context,
        current_robot_pose=pose,
        control_tick=0,
        simulation_time_s=0.0,
        context_content_hash="",
    )
    update = LocalReferenceWindowManager().update(current_context, reference, validation)
    assert update.window is not None
    python_controller = PersistentSourceDerivedDwbController(use_cpp_safety_core=False)
    native_controller = PersistentSourceDerivedDwbController(use_cpp_safety_core=True)

    python_result = _advance_to_first_signed_translation(
        python_controller,
        context,
        reference,
        update.window,
    )
    native_result = _advance_to_first_signed_translation(
        native_controller,
        context,
        reference,
        update.window,
    )

    assert native_controller.native_safety_batch_used
    assert native_result.status is python_result.status
    assert native_result.requested_twist == python_result.requested_twist
    assert native_result.predicted_trajectory == python_result.predicted_trajectory
    assert native_result.failure_reason == python_result.failure_reason
    assert native_result.decision_trace == python_result.decision_trace
    assert native_result.candidate_diagnostics == python_result.candidate_diagnostics
    assert native_controller.selected_safety_evidence == (
        python_controller.selected_safety_evidence
    )


def test_missing_fresh_observation_fails_closed_before_candidate_motion(public_wide_left) -> None:
    context = public_wide_left.build_context
    reference = public_wide_left.reference_set.candidates[0]
    validation = public_wide_left.validations[0]
    update = LocalReferenceWindowManager().update(context, reference, validation)
    assert update.window is not None
    result = _advance_to_first_signed_translation(
        PersistentSourceDerivedDwbController(),
        context,
        reference,
        update.window,
        unavailable_on_translation=True,
    )

    assert result.status is PersistentControllerStatus.SECTION_EXECUTION_FAILED
    assert result.requested_twist == Twist2D()
    assert result.controller_requested_protective_stop
    assert result.failure_reason == "fresh_observation_required"


def test_persistent_dwb_adapter_has_no_corpus_or_evaluator_label_channel() -> None:
    module_path = (
        Path(__file__).parents[1]
        / "src"
        / "hospital_path_lab"
        / "local_algorithms"
        / "dwb_reference"
        / "persistent_adapter.py"
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
