from __future__ import annotations

import ast
from dataclasses import replace
from pathlib import Path

import pytest
from test_local_reference_builder import _source

from hospital_path_lab.contracts import (
    GridSnapshot,
    Pose2D,
    RobotState,
    TrajectoryPoint,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import DynamicMotionState
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_reference_builder import (
    build_spatial_local_reference,
    project_validated_spatial_seed,
)
from hospital_path_lab.local_reference_contracts import (
    ReferenceEvidenceLevel,
    ReferenceLifecycleStatus,
)
from hospital_path_lab.local_reference_validation import validate_local_maneuver_reference
from hospital_path_lab.local_reference_window import LocalReferenceWindowManager
from hospital_path_lab.persistent_controller_contracts import (
    PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
    PERSISTENT_CONTROLLER_RESULT_SCHEMA_VERSION,
    PersistentControllerResult,
    PersistentControllerSessionTransition,
    PersistentControllerStatus,
    PersistentControllerTickInput,
    PersistentReferenceSessionGuard,
    ReferenceExecutorState,
    build_persistent_reference_binding,
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
    return context, seed, reference, validation


def _at(context, *, tick: int, pose: Pose2D | None = None):
    return replace(
        context,
        current_robot_pose=context.current_robot_pose if pose is None else pose,
        control_tick=tick,
        simulation_time_s=tick * 0.05,
        context_content_hash="",
    )


def _window(manager, context, reference, validation):
    update = manager.update(context, reference, validation)
    assert update.window is not None
    return update.window


def _tick_input(context, reference, window, *, pose: Pose2D | None = None):
    binding = build_persistent_reference_binding(reference, window)
    observation = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.UNAVAILABLE,
        frame=None,
        age_s=None,
        failures=(),
        last_event_was_no_frame=False,
    )
    return PersistentControllerTickInput(
        schema_version=PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
        controller_tick=context.control_tick,
        simulation_time_s=context.simulation_time_s,
        full_reference=reference,
        local_window=window,
        reference_binding=binding,
        robot_state=RobotState(
            context.current_robot_pose if pose is None else pose,
            Twist2D(),
        ),
        static_grid_snapshot=context.static_grid_snapshot,
        validated_observation=observation,
        actor_prediction_set=None,
        vehicle_profile=context.vehicle_profile,
        current_gate_motion_state=DynamicMotionState.MOVING,
        current_gate_stop_epoch=context.stop_epoch,
        current_resume_authorization_revision=None,
    )


def _initial_tick_input():
    context, _seed, reference, validation = _fixture()
    window = _window(LocalReferenceWindowManager(), context, reference, validation)
    return _tick_input(context, reference, window), context, reference, validation


def _command_result(tick_input, **changes):
    section = tick_input.full_reference.sections[0]
    values = {
        "schema_version": PERSISTENT_CONTROLLER_RESULT_SCHEMA_VERSION,
        "controller_name": "persistent_rpp_reference",
        "source_controller_tick": tick_input.controller_tick,
        "status": PersistentControllerStatus.COMMAND_FOUND,
        "requested_twist": Twist2D(0.01, 0.0),
        "predicted_trajectory": (
            TrajectoryPoint(
                0.0,
                tick_input.robot_state.pose,
                Twist2D(0.01, 0.0),
            ),
        ),
        "failure_reason": None,
        "decision_trace": ("reference_bound",),
        "reference_binding_echo": tick_input.reference_binding,
        "tick_input_content_hash": tick_input.tick_input_content_hash,
        "controller_session_transition": (
            PersistentControllerSessionTransition.INITIAL_BIND
        ),
        "executor_state": ReferenceExecutorState.TRACK_TRANSLATION,
        "active_section_index": section.section_index,
        "active_section_kind": section.section_kind,
        "tracking_error_m": 0.0,
        "candidate_diagnostics": (),
        "planned_section_stop": False,
        "controller_requested_protective_stop": False,
        "no_safe_candidate": False,
        "elapsed_nonqualification_ns": 10,
    }
    values.update(changes)
    return PersistentControllerResult(**values)


def test_binding_and_tick_hash_cover_the_complete_reference_delivery() -> None:
    tick_input, context, reference, _validation = _initial_tick_input()

    assert tick_input.reference_binding.full_reference_hash == (
        reference.reference_content_hash
    )
    assert tick_input.reference_binding.source_window_control_tick == context.control_tick
    assert tick_input.tick_input_content_hash == tick_input.expected_content_hash

    with pytest.raises(ValueError, match="binding_content_hash mismatch"):
        replace(
            tick_input.reference_binding,
            binding_content_hash="0" * 64,
        )


def test_tick_input_rejects_grid_epoch_and_delivery_tampering() -> None:
    tick_input, _context, _reference, _validation = _initial_tick_input()
    occupancy = tick_input.static_grid_snapshot.grid.occupancy.copy()
    occupancy[0, 0] = not occupancy[0, 0]
    changed_grid = GridMap(
        occupancy,
        resolution_m=tick_input.static_grid_snapshot.grid.resolution_m,
        origin_x_m=tick_input.static_grid_snapshot.grid.origin_x_m,
        origin_y_m=tick_input.static_grid_snapshot.grid.origin_y_m,
    )
    changed_snapshot = GridSnapshot(
        tick_input.static_grid_snapshot.metadata,
        changed_grid,
        tick_input.static_grid_snapshot.forbidden_cells,
    )

    with pytest.raises(ValueError, match="grid content"):
        replace(
            tick_input,
            static_grid_snapshot=changed_snapshot,
            tick_input_content_hash="",
        )
    with pytest.raises(ValueError, match="stop epoch"):
        replace(
            tick_input,
            current_gate_stop_epoch=tick_input.current_gate_stop_epoch + 1,
            tick_input_content_hash="",
        )
    with pytest.raises(ValueError, match="freshly delivered"):
        replace(
            tick_input,
            controller_tick=tick_input.controller_tick + 1,
            simulation_time_s=tick_input.simulation_time_s + 0.05,
            tick_input_content_hash="",
        )


def test_tick_input_rejects_a_rehashed_but_modified_window_slice() -> None:
    tick_input, _context, reference, _validation = _initial_tick_input()
    first_knot = tick_input.local_window.knots[0]
    changed_knot = replace(
        first_knot,
        tangent_yaw=first_knot.tangent_yaw + 0.001,
    )
    changed_window = replace(
        tick_input.local_window,
        knots=(changed_knot, *tick_input.local_window.knots[1:]),
        window_content_hash="",
    )
    changed_binding = build_persistent_reference_binding(reference, changed_window)

    with pytest.raises(ValueError, match="exact full-reference slice"):
        replace(
            tick_input,
            local_window=changed_window,
            reference_binding=changed_binding,
            tick_input_content_hash="",
        )


def test_guard_is_idempotent_and_rejects_same_tick_semantic_change() -> None:
    tick_input, _context, _reference, _validation = _initial_tick_input()
    guard = PersistentReferenceSessionGuard()

    initial = guard.evaluate(tick_input)
    duplicate = guard.evaluate(tick_input)
    changed = replace(
        tick_input,
        robot_state=RobotState(
            Pose2D(
                tick_input.robot_state.pose.x + 0.001,
                tick_input.robot_state.pose.y,
                tick_input.robot_state.pose.yaw,
            ),
            tick_input.robot_state.twist,
        ),
        tick_input_content_hash="",
    )
    rejected = guard.evaluate(changed)
    duplicate_after_rejection = guard.evaluate(tick_input)

    assert initial.transition is PersistentControllerSessionTransition.INITIAL_BIND
    assert initial.state_reset_required
    assert duplicate.accepted and duplicate.duplicate_tick
    assert not rejected.accepted
    assert rejected.reason_code == "same_tick_input_changed"
    assert duplicate_after_rejection.accepted
    assert guard.current_binding == tick_input.reference_binding


def test_same_window_next_tick_preserves_state_and_window_advance_does_not_reset() -> None:
    context, _seed, reference, validation = _fixture()
    manager = LocalReferenceWindowManager()
    guard = PersistentReferenceSessionGuard()
    first_window = _window(manager, context, reference, validation)
    first = _tick_input(context, reference, first_window)
    assert guard.evaluate(first).state_reset_required

    next_context = _at(context, tick=context.control_tick + 1)
    same_window = _window(manager, next_context, reference, validation)
    unchanged = guard.evaluate(_tick_input(next_context, reference, same_window))
    assert unchanged.transition is PersistentControllerSessionTransition.WINDOW_UNCHANGED
    assert not unchanged.state_reset_required

    later_pose = min(
        reference.knots,
        key=lambda knot: abs(knot.cumulative_translation_arc_m - 0.60),
    ).pose
    later_context = _at(context, tick=context.control_tick + 2, pose=later_pose)
    advanced_window = _window(manager, later_context, reference, validation)
    advanced = guard.evaluate(
        _tick_input(later_context, reference, advanced_window, pose=later_pose)
    )
    assert advanced.transition is PersistentControllerSessionTransition.WINDOW_ADVANCED
    assert not advanced.state_reset_required
    assert advanced_window.subgoal_revision == first_window.subgoal_revision + 1


def test_new_path_session_resets_state_and_revision_regression_is_rejected() -> None:
    tick_input, context, _reference, _validation = _initial_tick_input()
    guard = PersistentReferenceSessionGuard()
    assert guard.evaluate(tick_input).accepted

    source, _ = _source()
    seed = project_validated_spatial_seed(context, source)
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
    next_context = _at(context, tick=context.control_tick + 1)
    new_window = _window(
        LocalReferenceWindowManager(),
        next_context,
        new_reference,
        new_validation,
    )
    reset = guard.evaluate(_tick_input(next_context, new_reference, new_window))

    assert reset.accepted
    assert reset.state_reset_required
    assert reset.transition is PersistentControllerSessionTransition.SESSION_RESET

    stale_window = replace(
        tick_input.local_window,
        source_control_tick=context.control_tick + 2,
    )
    stale_context = _at(context, tick=context.control_tick + 2)
    stale_input = _tick_input(
        stale_context,
        tick_input.full_reference,
        stale_window,
    )
    rejected = guard.evaluate(stale_input)
    assert not rejected.accepted
    assert rejected.reason_code == "revision_regression"


def test_nonavailable_binding_is_rejected_before_controller_use() -> None:
    tick_input, _context, reference, _validation = _initial_tick_input()
    withdrawn = build_persistent_reference_binding(
        reference,
        tick_input.local_window,
        lifecycle=ReferenceLifecycleStatus.WITHDRAWN,
    )
    with pytest.raises(ValueError, match="AVAILABLE"):
        replace(
            tick_input,
            reference_binding=withdrawn,
            tick_input_content_hash="",
        )


def test_temporal_reference_without_tick_bound_authorization_is_rejected() -> None:
    tick_input, _context, reference, _validation = _initial_tick_input()
    temporal_reference = replace(
        reference,
        evidence_level=ReferenceEvidenceLevel.GROUND_TRUTH_TEMPORAL,
        source_spatial_seed_hash=None,
        source_temporal_evidence_hash="1" * 64,
        source_temporal_geometry_hash="2" * 64,
        reference_content_hash="",
    )
    temporal_window = replace(
        tick_input.local_window,
        full_reference_hash=temporal_reference.reference_content_hash,
        window_content_hash="",
    )
    temporal_binding = build_persistent_reference_binding(
        temporal_reference,
        temporal_window,
    )

    with pytest.raises(ValueError, match="tick-bound authorization"):
        replace(
            tick_input,
            full_reference=temporal_reference,
            local_window=temporal_window,
            reference_binding=temporal_binding,
            tick_input_content_hash="",
        )


def test_result_distinguishes_planned_section_stop_from_protective_stop() -> None:
    tick_input, _context, _reference, _validation = _initial_tick_input()
    moving = _command_result(tick_input)
    planned = _command_result(
        tick_input,
        status=PersistentControllerStatus.PLANNED_STOP,
        requested_twist=Twist2D(),
        predicted_trajectory=(
            TrajectoryPoint(0.0, tick_input.robot_state.pose, Twist2D()),
        ),
        executor_state=ReferenceExecutorState.CONFIRM_PLANNED_STOP,
        planned_section_stop=True,
    )
    protective = _command_result(
        tick_input,
        status=PersistentControllerStatus.HOLD_REQUESTED,
        requested_twist=Twist2D(),
        predicted_trajectory=(
            TrajectoryPoint(0.0, tick_input.robot_state.pose, Twist2D()),
        ),
        failure_reason="invalid_source_hold",
        executor_state=ReferenceExecutorState.HOLD_REQUESTED,
        controller_requested_protective_stop=True,
    )

    assert moving.semantic_content_hash == moving.expected_semantic_hash
    assert planned.planned_section_stop and not planned.controller_requested_protective_stop
    assert protective.controller_requested_protective_stop
    assert not protective.planned_section_stop

    with pytest.raises(ValueError, match="planned stop"):
        _command_result(
            tick_input,
            status=PersistentControllerStatus.PLANNED_STOP,
            requested_twist=Twist2D(),
            planned_section_stop=True,
            controller_requested_protective_stop=True,
        )


def test_result_semantic_hash_excludes_wall_clock_diagnostic() -> None:
    tick_input, _context, _reference, _validation = _initial_tick_input()
    first = _command_result(tick_input, elapsed_nonqualification_ns=10)
    second = _command_result(tick_input, elapsed_nonqualification_ns=999_999)

    assert first.semantic_content_hash == second.semantic_content_hash


def test_result_rejects_incoherent_failure_flags() -> None:
    tick_input, _context, _reference, _validation = _initial_tick_input()

    with pytest.raises(ValueError, match="no_safe_candidate"):
        _command_result(
            tick_input,
            status=PersistentControllerStatus.NO_SAFE_COMMAND,
            requested_twist=Twist2D(),
            failure_reason="no_legal_candidate",
            controller_requested_protective_stop=True,
        )
    with pytest.raises(ValueError, match="protective stop"):
        _command_result(
            tick_input,
            status=PersistentControllerStatus.HOLD_REQUESTED,
            requested_twist=Twist2D(),
            failure_reason="stale_input",
        )


def test_contract_module_has_no_corpus_or_evaluator_label_channel() -> None:
    module_path = (
        Path(__file__).parents[1]
        / "src"
        / "hospital_path_lab"
        / "persistent_controller_contracts.py"
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
