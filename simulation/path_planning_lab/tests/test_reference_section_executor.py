from __future__ import annotations

import ast
from dataclasses import replace
from math import atan2, hypot
from pathlib import Path

import pytest
from test_local_reference_builder import _source

from hospital_path_lab.contracts import Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import DynamicMotionState
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.local_reference_builder import (
    build_spatial_local_reference,
    project_validated_spatial_seed,
)
from hospital_path_lab.local_reference_contracts import (
    LOCAL_REFERENCE_CONTRACT_VERSION,
    LOCAL_REFERENCE_SCHEMA_VERSION,
    LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION,
    LocalManeuverKind,
    LocalManeuverReference,
    LocalReferenceWindow,
    ObservationDependency,
    ReferenceEvidenceLevel,
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
from hospital_path_lab.local_reference_validation import validate_local_maneuver_reference
from hospital_path_lab.local_reference_window import LocalReferenceWindowManager
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.persistent_controller_contracts import (
    PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
    PersistentControllerSessionTransition,
    PersistentControllerTickInput,
    ReferenceExecutorState,
    build_persistent_reference_binding,
)
from hospital_path_lab.reference_section_executor import (
    R5_ANGULAR_DECELERATION_RADPS2,
    R5_CONTROL_PERIOD_S,
    R5_LINEAR_DECELERATION_MPS2,
    REFERENCE_SECTION_EXECUTION_DECISION_SCHEMA_VERSION,
    ReferenceExecutorAction,
    ReferenceSectionExecutor,
    ReferenceSectionExecutorConfig,
    shortest_angular_distance,
    translation_completion_reached,
    translation_completion_tolerance_m,
)


def _fixture(*, maneuver_revision: int = 4, path_revision: int = 2):
    source, context = _source()
    seed = project_validated_spatial_seed(context, source)
    reference = build_spatial_local_reference(
        context,
        seed,
        maneuver_revision=maneuver_revision,
        path_revision=path_revision,
    )
    validation = validate_local_maneuver_reference(
        context,
        reference,
        spatial_seed=seed,
    )
    assert validation.passed
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
    return context, seed, reference, validation, full_window


def _tick_input(
    context,
    reference,
    window,
    *,
    tick: int,
    pose: Pose2D,
    twist: Twist2D | None = None,
    authorization_revision: int | None = None,
    gate_motion_state: DynamicMotionState = DynamicMotionState.MOVING,
):
    current_window = replace(window, source_control_tick=tick)
    binding = build_persistent_reference_binding(reference, current_window)
    return PersistentControllerTickInput(
        schema_version=PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
        controller_tick=tick,
        simulation_time_s=tick * R5_CONTROL_PERIOD_S,
        full_reference=reference,
        local_window=current_window,
        reference_binding=binding,
        robot_state=RobotState(pose, Twist2D() if twist is None else twist),
        static_grid_snapshot=context.static_grid_snapshot,
        validated_observation=DynamicObservationSnapshot(
            availability=DynamicObservationAvailability.UNAVAILABLE,
            frame=None,
            age_s=None,
            failures=(),
            last_event_was_no_frame=False,
        ),
        actor_prediction_set=None,
        vehicle_profile=context.vehicle_profile,
        current_gate_motion_state=gate_motion_state,
        current_gate_stop_epoch=context.stop_epoch,
        current_resume_authorization_revision=authorization_revision,
    )


def _advance_to_first_rotation(executor, context, reference, window, *, start_tick=40):
    start = reference.knots[0].pose
    first = executor.step(
        _tick_input(
            context,
            reference,
            window,
            tick=start_tick,
            pose=start,
            twist=Twist2D(0.20, 0.30),
        )
    )
    decisions = [first]
    for tick in range(start_tick + 1, start_tick + 4):
        decisions.append(
            executor.step(
                _tick_input(
                    context,
                    reference,
                    window,
                    tick=tick,
                    pose=start,
                )
            )
        )
    return decisions


def _wait_reference(context, template: LocalManeuverReference) -> LocalManeuverReference:
    start = context.current_robot_pose
    end = context.original_reference[-1]
    length = hypot(end.x - start.x, end.y - start.y)
    yaw = atan2(end.y - start.y, end.x - start.x)
    knots = (
        ReferenceKnot(
            knot_index=0,
            pose=start,
            tangent_yaw=start.yaw,
            cumulative_translation_arc_m=0.0,
            source_path_index=0,
            section_index=0,
            knot_roles=(ReferenceKnotRole.ANCHOR,),
        ),
        ReferenceKnot(
            knot_index=1,
            pose=start,
            tangent_yaw=yaw,
            cumulative_translation_arc_m=0.0,
            source_path_index=0,
            section_index=1,
            knot_roles=(ReferenceKnotRole.ANCHOR,),
        ),
        ReferenceKnot(
            knot_index=2,
            pose=end,
            tangent_yaw=yaw,
            cumulative_translation_arc_m=length,
            source_path_index=1,
            section_index=1,
            knot_roles=(
                ReferenceKnotRole.TRANSLATION,
                ReferenceKnotRole.REJOIN,
                ReferenceKnotRole.STOP_MARKER,
            ),
        ),
    )
    sections = (
        ReferenceSection(
            section_index=0,
            section_kind=ReferenceSectionKind.HOLD,
            travel_direction=ReferenceTravelDirection.NONE,
            first_knot_index=0,
            last_knot_index=0,
            entry_requires_stopped=True,
            exit_requires_stopped=True,
            source_primitive_indices=(),
        ),
        ReferenceSection(
            section_index=1,
            section_kind=ReferenceSectionKind.FOLLOW_ORIGINAL,
            travel_direction=ReferenceTravelDirection.FORWARD,
            first_knot_index=1,
            last_knot_index=2,
            entry_requires_stopped=False,
            exit_requires_stopped=True,
            source_primitive_indices=(),
        ),
    )
    identity = {
        "kind": "wait-reference-test-fixture",
        "mission": context.mission_id,
        "maneuver_revision": 6,
        "path_revision": 1,
    }
    return LocalManeuverReference(
        schema_version=LOCAL_REFERENCE_SCHEMA_VERSION,
        reference_contract_version=LOCAL_REFERENCE_CONTRACT_VERSION,
        candidate_id=canonical_content_hash({"candidate": identity}),
        maneuver_kind=LocalManeuverKind.WAIT_OR_FOLLOW,
        evidence_level=ReferenceEvidenceLevel.SPATIAL_ONLY,
        mission_id=context.mission_id,
        stop_epoch=context.stop_epoch,
        map_id=context.map_id,
        map_revision=context.map_revision,
        mission_revision=context.mission_revision,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        maneuver_revision=6,
        path_revision=1,
        reference_session_id=canonical_content_hash({"session": identity}),
        source_spatial_seed_hash=None,
        source_temporal_evidence_hash=None,
        original_reference_hash=context.original_reference_hash,
        grid_content_hash=context.grid_content_hash,
        vehicle_profile_hash=context.vehicle_profile_hash,
        allowed_region_hash=context.allowed_region_hash,
        forbidden_region_hash=context.forbidden_region_hash,
        knots=knots,
        sections=sections,
        departure_knot_index=None,
        pass_section_index=None,
        rejoin_knot_index=2,
        minimum_validated_static_clearance_m=(
            template.minimum_validated_static_clearance_m
        ),
        validity=replace(
            template.validity,
            valid_from_control_tick=context.control_tick,
        ),
        generation_reason_codes=("test_wait_reference",),
        limitations=("simulation_only", "wait_release_not_implemented"),
    )


def _full_window_for_reference(reference, *, tick: int) -> LocalReferenceWindow:
    return LocalReferenceWindow(
        schema_version=LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION,
        reference_session_id=reference.reference_session_id,
        maneuver_revision=reference.maneuver_revision,
        path_revision=reference.path_revision,
        subgoal_revision=0,
        full_reference_hash=reference.reference_content_hash,
        source_control_tick=tick,
        start_knot_index=0,
        end_knot_index=reference.knots[-1].knot_index,
        knots=reference.knots,
        sections=reference.sections,
        terminal_rejoin_included=True,
    )


def test_config_is_frozen_to_the_r5_v1_stop_and_rotation_contract() -> None:
    config = ReferenceSectionExecutorConfig()

    assert config.control_period_s == 0.05
    assert config.stopped_confirmation_ticks == 3
    assert config.terminal_dwell_ticks == 10
    with pytest.raises(ValueError, match="frozen"):
        ReferenceSectionExecutorConfig(position_tolerance_m=0.06)
    with pytest.raises(ValueError, match="cannot exceed"):
        ReferenceSectionExecutor(bypass_completion_tolerance_m=0.06)
    with pytest.raises(ValueError, match="positive"):
        ReferenceSectionExecutor(bypass_completion_tolerance_m=0.0)


def test_stopped_abstract_connector_consumes_existing_position_tolerance() -> None:
    case = next(
        item
        for item in public_local_reference_cases()
        if item.public_id == "crossing-static-left"
    )
    result = evaluate_local_reference_public_case(case)
    assert result.hard_failures == ()
    reference = result.reference_set.candidates[0]
    translation = reference.sections[-2]
    connector = reference.sections[-1]
    connector_start = reference.knots[connector.first_knot_index].pose
    connector_end = reference.knots[connector.last_knot_index].pose
    connector_displacement = hypot(
        connector_end.x - connector_start.x,
        connector_end.y - connector_start.y,
    )

    assert translation.travel_direction is ReferenceTravelDirection.FORWARD
    assert connector.travel_direction is ReferenceTravelDirection.NONE
    assert connector.entry_requires_stopped and connector.exit_requires_stopped
    assert translation_completion_tolerance_m(
        reference,
        translation.section_index,
    ) == pytest.approx(0.05 - connector_displacement)
    assert translation_completion_reached(
        reference,
        translation.section_index,
        Pose2D(4.354002177999232, 2.360831204890371, -0.005066693637160212),
    )
    assert not translation_completion_reached(
        reference,
        translation.section_index,
        Pose2D(4.306496036677228, 2.361426434749244, 0.00479997302950654),
    )


def test_planned_stop_uses_bounded_deceleration_and_three_actual_stop_ticks() -> None:
    context, _seed, reference, _validation, window = _fixture()
    executor = ReferenceSectionExecutor()
    decisions = _advance_to_first_rotation(executor, context, reference, window)
    first = decisions[0]

    assert first.action is ReferenceExecutorAction.APPLY_COMMON_COMMAND
    assert first.executor_state is ReferenceExecutorState.APPROACH_PLANNED_STOP
    assert first.planned_section_stop
    assert not first.controller_requested_protective_stop
    assert first.common_command is not None
    assert first.common_command.linear == pytest.approx(
        0.20 - R5_LINEAR_DECELERATION_MPS2 * R5_CONTROL_PERIOD_S
    )
    assert first.common_command.angular == pytest.approx(
        0.30 - R5_ANGULAR_DECELERATION_RADPS2 * R5_CONTROL_PERIOD_S
    )
    assert decisions[1].stopped_confirmation_ticks == 1
    assert decisions[2].stopped_confirmation_ticks == 2
    assert decisions[3].executor_state is ReferenceExecutorState.ROTATE_IN_PLACE
    assert decisions[3].active_section_kind is ReferenceSectionKind.ROTATE


def test_rotation_uses_shortest_direction_and_waits_for_actual_stop() -> None:
    context, _seed, reference, _validation, window = _fixture()
    executor = ReferenceSectionExecutor()
    _advance_to_first_rotation(executor, context, reference, window)
    start = reference.knots[0].pose

    rotating = executor.step(
        _tick_input(context, reference, window, tick=44, pose=start)
    )
    assert rotating.executor_state is ReferenceExecutorState.ROTATE_IN_PLACE
    assert rotating.common_command is not None
    assert rotating.common_command.linear == 0.0
    assert rotating.common_command.angular == pytest.approx(0.08)
    assert not rotating.planned_section_stop

    target = reference.knots[reference.sections[1].last_knot_index].pose
    confirm_one = executor.step(
        _tick_input(context, reference, window, tick=45, pose=target)
    )
    confirm_two = executor.step(
        _tick_input(context, reference, window, tick=46, pose=target)
    )
    completed_rotation = executor.step(
        _tick_input(context, reference, window, tick=47, pose=target)
    )
    next_section = executor.step(
        _tick_input(context, reference, window, tick=48, pose=target)
    )

    assert confirm_one.executor_state is ReferenceExecutorState.CONFIRM_ROTATION_STOP
    assert confirm_two.stopped_confirmation_ticks == 2
    assert completed_rotation.executor_state is ReferenceExecutorState.TRACK_TRANSLATION
    assert completed_rotation.active_section_index == 2
    assert next_section.action is ReferenceExecutorAction.DELEGATE_TRANSLATION
    assert shortest_angular_distance(3.10, -3.10) > 0.0


def test_planned_stop_confirmation_resets_when_angular_feedback_is_not_stopped() -> None:
    context, _seed, reference, _validation, window = _fixture()
    executor = ReferenceSectionExecutor()
    start = reference.knots[0].pose
    executor.step(
        _tick_input(
            context,
            reference,
            window,
            tick=40,
            pose=start,
            twist=Twist2D(0.10, 0.0),
        )
    )
    first_stopped = executor.step(
        _tick_input(context, reference, window, tick=41, pose=start)
    )
    interrupted = executor.step(
        _tick_input(
            context,
            reference,
            window,
            tick=42,
            pose=start,
            twist=Twist2D(0.0, 0.021),
        )
    )
    restarted = executor.step(
        _tick_input(context, reference, window, tick=43, pose=start)
    )

    assert first_stopped.stopped_confirmation_ticks == 1
    assert interrupted.executor_state is ReferenceExecutorState.APPROACH_PLANNED_STOP
    assert interrupted.stopped_confirmation_ticks == 0
    assert restarted.executor_state is ReferenceExecutorState.CONFIRM_PLANNED_STOP
    assert restarted.stopped_confirmation_ticks == 1


def test_duplicate_tick_is_cached_and_changed_same_tick_does_not_mutate_state() -> None:
    context, _seed, reference, _validation, window = _fixture()
    executor = ReferenceSectionExecutor()
    _advance_to_first_rotation(executor, context, reference, window)
    start = reference.knots[0].pose
    original_input = _tick_input(context, reference, window, tick=44, pose=start)
    original = executor.step(original_input)
    duplicate = executor.step(original_input)
    changed_input = _tick_input(
        context,
        reference,
        window,
        tick=44,
        pose=start,
        twist=Twist2D(0.0, 0.01),
    )
    rejected = executor.step(changed_input)
    duplicate_after_rejection = executor.step(original_input)

    assert duplicate is original
    assert rejected.action is ReferenceExecutorAction.REQUEST_PROTECTIVE_HOLD
    assert rejected.failure_reason == "same_tick_input_changed"
    assert executor.state is ReferenceExecutorState.ROTATE_IN_PLACE
    assert duplicate_after_rejection is original


def test_tick_gap_fails_closed_without_consuming_the_missing_tick() -> None:
    context, _seed, reference, _validation, window = _fixture()
    executor = ReferenceSectionExecutor()
    start = reference.knots[0].pose
    tick_40 = _tick_input(context, reference, window, tick=40, pose=start)
    executor.step(tick_40)

    gap = executor.step(
        _tick_input(context, reference, window, tick=42, pose=start)
    )
    recovered = executor.step(
        _tick_input(context, reference, window, tick=41, pose=start)
    )

    assert gap.action is ReferenceExecutorAction.REQUEST_PROTECTIVE_HOLD
    assert gap.failure_reason == "controller_tick_gap"
    assert recovered.source_controller_tick == 41
    assert recovered.controller_requested_protective_stop is False


def test_rotation_position_drift_invalidates_execution_and_requests_hold() -> None:
    context, _seed, reference, _validation, window = _fixture()
    executor = ReferenceSectionExecutor()
    _advance_to_first_rotation(executor, context, reference, window)
    start = reference.knots[0].pose
    drifted = Pose2D(start.x + 0.051, start.y, start.yaw)

    rejected = executor.step(
        _tick_input(context, reference, window, tick=44, pose=drifted)
    )
    held = executor.step(
        _tick_input(context, reference, window, tick=45, pose=start)
    )

    assert rejected.executor_state is ReferenceExecutorState.INVALIDATED
    assert rejected.failure_reason == "rotation_position_tolerance_exceeded"
    assert rejected.common_command == Twist2D()
    assert held.failure_reason == "executor_session_invalidated"


def test_protective_gate_stop_preserves_rotation_state_without_claiming_new_hold() -> None:
    context, _seed, reference, _validation, window = _fixture()
    executor = ReferenceSectionExecutor()
    _advance_to_first_rotation(executor, context, reference, window)
    start = reference.knots[0].pose

    gate_held = executor.step(
        _tick_input(
            context,
            reference,
            window,
            tick=44,
            pose=start,
            gate_motion_state=DynamicMotionState.HOLDING,
        )
    )
    resumed = executor.step(
        _tick_input(context, reference, window, tick=45, pose=start)
    )

    assert gate_held.action is ReferenceExecutorAction.PRESERVE_DURING_GATE_STOP
    assert gate_held.common_command == Twist2D()
    assert not gate_held.controller_requested_protective_stop
    assert gate_held.executor_state is ReferenceExecutorState.ROTATE_IN_PLACE
    assert resumed.executor_state is ReferenceExecutorState.ROTATE_IN_PLACE
    assert resumed.common_command is not None
    assert resumed.common_command.angular > 0.0


def test_new_path_session_resets_an_existing_rotation_state_once() -> None:
    context, seed, reference, _validation, window = _fixture()
    executor = ReferenceSectionExecutor()
    _advance_to_first_rotation(executor, context, reference, window)
    new_reference = build_spatial_local_reference(
        context,
        seed,
        maneuver_revision=5,
        path_revision=3,
    )
    new_validation = validate_local_maneuver_reference(
        context,
        new_reference,
        spatial_seed=seed,
    )
    update = LocalReferenceWindowManager().update(context, new_reference, new_validation)
    assert update.window is not None
    new_window = replace(
        update.window,
        source_control_tick=44,
        end_knot_index=new_reference.knots[-1].knot_index,
        knots=new_reference.knots,
        sections=new_reference.sections,
        terminal_rejoin_included=True,
        window_content_hash="",
    )

    reset = executor.step(
        _tick_input(
            context,
            new_reference,
            new_window,
            tick=44,
            pose=new_reference.knots[0].pose,
        )
    )

    assert reset.session_transition is PersistentControllerSessionTransition.SESSION_RESET
    assert reset.session_reset_count == 2
    assert reset.executor_state is ReferenceExecutorState.CONFIRM_PLANNED_STOP


def test_hold_section_never_auto_releases_even_with_authorization_field() -> None:
    context, _seed, template, _validation, _window = _fixture()
    reference = _wait_reference(context, template)
    window = _full_window_for_reference(reference, tick=context.control_tick)
    executor = ReferenceSectionExecutor()

    first = executor.step(
        _tick_input(
            context,
            reference,
            window,
            tick=40,
            pose=reference.knots[0].pose,
            authorization_revision=9,
        )
    )
    later = executor.step(
        _tick_input(
            context,
            reference,
            window,
            tick=41,
            pose=reference.knots[0].pose,
            authorization_revision=9,
        )
    )

    assert first.action is ReferenceExecutorAction.REQUEST_PROTECTIVE_HOLD
    assert later.action is ReferenceExecutorAction.REQUEST_PROTECTIVE_HOLD
    assert later.active_section_kind is ReferenceSectionKind.HOLD
    assert later.failure_reason == "hold_section_requires_new_authorized_reference"


def test_full_reference_reaches_terminal_only_after_stop_confirmation_and_dwell() -> None:
    context, _seed, reference, validation, _window = _fixture()
    executor = ReferenceSectionExecutor()
    window_manager = LocalReferenceWindowManager()
    pose = reference.knots[0].pose
    twist = Twist2D()
    first_dwell_tick = None
    completed_tick = None
    decisions = []

    for tick in range(40, 160):
        current_context = replace(
            context,
            current_robot_pose=pose,
            control_tick=tick,
            simulation_time_s=tick * R5_CONTROL_PERIOD_S,
            context_content_hash="",
        )
        window_update = window_manager.update(current_context, reference, validation)
        assert window_update.window is not None
        decision = executor.step(
            _tick_input(
                context,
                reference,
                window_update.window,
                tick=tick,
                pose=pose,
                twist=twist,
            )
        )
        decisions.append(decision)
        if decision.executor_state is ReferenceExecutorState.TERMINAL_DWELL:
            first_dwell_tick = tick if first_dwell_tick is None else first_dwell_tick
        if decision.completed:
            completed_tick = tick
            break
        if decision.action is ReferenceExecutorAction.DELEGATE_TRANSLATION or (
            decision.executor_state is ReferenceExecutorState.ROTATE_IN_PLACE
            and not decision.planned_section_stop
        ):
            assert decision.target_pose is not None
            pose = decision.target_pose
            twist = Twist2D()
        else:
            twist = Twist2D()

    assert completed_tick is not None and first_dwell_tick is not None
    assert completed_tick - first_dwell_tick >= 11
    assert decisions[-1].action is ReferenceExecutorAction.MISSION_COMPLETED
    assert decisions[-1].common_command == Twist2D()
    assert decisions[-1].controller_requested_protective_stop is False
    assert executor.session_reset_count == 1
    assert executor.window_update_count >= 1


def test_decision_hash_and_contract_are_deterministic() -> None:
    context, _seed, reference, _validation, window = _fixture()
    tick_input = _tick_input(
        context,
        reference,
        window,
        tick=40,
        pose=reference.knots[0].pose,
    )
    left = ReferenceSectionExecutor().step(tick_input)
    right = ReferenceSectionExecutor().step(tick_input)

    assert left.schema_version == REFERENCE_SECTION_EXECUTION_DECISION_SCHEMA_VERSION
    assert left.semantic_content_hash == left.expected_semantic_hash
    assert left.semantic_content_hash == right.semantic_content_hash


def test_executor_module_has_no_corpus_or_evaluator_label_channel() -> None:
    module_path = (
        Path(__file__).parents[1]
        / "src"
        / "hospital_path_lab"
        / "reference_section_executor.py"
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
    names = {
        node.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Name)
    }
    attributes = {
        node.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
    }
    imported_modules = {
        node.module or ""
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    }

    assert forbidden.isdisjoint(names | attributes)
    assert not any(
        any(token in module for token in forbidden)
        for module in imported_modules
    )
