from __future__ import annotations

import ast
from copy import deepcopy
from dataclasses import replace
from hashlib import sha256
from pathlib import Path

from test_local_reference_builder import _source

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.local_reference_builder import (
    build_spatial_local_reference,
    project_validated_spatial_seed,
)
from hospital_path_lab.local_reference_contracts import ReferenceSectionKind
from hospital_path_lab.local_reference_reporting import (
    evaluate_local_reference_public_case,
    public_local_reference_cases,
)
from hospital_path_lab.local_reference_validation import (
    validate_local_maneuver_reference,
)
from hospital_path_lab.local_reference_window import (
    LocalReferenceWindowManager,
    WindowUpdateStatus,
    project_reference_cursor,
    window_is_exact_slice,
)


def _hash(label: str) -> str:
    return sha256(label.encode("utf-8")).hexdigest()


def _fixture():
    source, context = _source()
    seed = project_validated_spatial_seed(context, source)
    reference = build_spatial_local_reference(
        context,
        seed,
        maneuver_revision=4,
        path_revision=2,
    )
    validation = validate_local_maneuver_reference(
        context,
        reference,
        spatial_seed=seed,
    )
    assert validation.passed
    return context, seed, reference, validation


def _at(context, pose: Pose2D, tick: int):
    return replace(
        context,
        current_robot_pose=pose,
        control_tick=tick,
        simulation_time_s=tick * 0.05,
        context_content_hash="",
    )


def _knot_near_arc(reference, arc_m: float):
    return min(
        reference.knots,
        key=lambda knot: abs(knot.cumulative_translation_arc_m - arc_m),
    )


def test_initial_window_is_an_exact_atomic_slice() -> None:
    context, _seed, reference, validation = _fixture()

    update = LocalReferenceWindowManager().update(context, reference, validation)

    assert update.status is WindowUpdateStatus.WINDOW_READY
    assert update.reason_code == "initial_window"
    assert update.window is not None
    assert window_is_exact_slice(reference, update.window)
    assert update.window.reference_session_id == reference.reference_session_id
    assert update.window.subgoal_revision == 0
    assert update.window.start_knot_index == 0
    assert update.window.end_knot_index > update.window.start_knot_index


def test_same_tick_is_idempotent_and_same_slice_next_tick_keeps_revision_hash() -> None:
    context, _seed, reference, validation = _fixture()
    manager = LocalReferenceWindowManager()

    first = manager.update(context, reference, validation)
    duplicate = manager.update(context, reference, validation)
    next_tick = manager.update(
        _at(context, context.current_robot_pose, context.control_tick + 1),
        reference,
        validation,
    )

    assert duplicate is first
    assert next_tick.reason_code == "window_unchanged"
    assert first.window is not None and next_tick.window is not None
    assert next_tick.window.subgoal_revision == first.window.subgoal_revision
    assert next_tick.window.window_content_hash == first.window.window_content_hash
    assert next_tick.window.source_control_tick == first.window.source_control_tick + 1


def test_same_tick_different_pose_is_invalid_without_state_mutation() -> None:
    context, _seed, reference, validation = _fixture()
    manager = LocalReferenceWindowManager()
    first = manager.update(context, reference, validation)
    changed = _at(
        context,
        Pose2D(context.current_robot_pose.x + 0.01, context.current_robot_pose.y),
        context.control_tick,
    )

    rejected = manager.update(changed, reference, validation)
    next_tick = manager.update(
        _at(context, context.current_robot_pose, context.control_tick + 1),
        reference,
        validation,
    )

    assert rejected.status is WindowUpdateStatus.INVALID_INPUT
    assert rejected.reason_code == "same_tick_different_input"
    assert next_tick.window is not None and first.window is not None
    assert next_tick.window.subgoal_revision == first.window.subgoal_revision


def test_section_boundary_advance_changes_only_subgoal_revision() -> None:
    context, _seed, reference, validation = _fixture()
    manager = LocalReferenceWindowManager()
    first = manager.update(context, reference, validation)
    later_pose = _knot_near_arc(reference, 0.60).pose

    advanced = manager.update(
        _at(context, later_pose, context.control_tick + 1),
        reference,
        validation,
    )

    assert advanced.status is WindowUpdateStatus.WINDOW_READY
    assert advanced.reason_code == "window_advanced"
    assert first.window is not None and advanced.window is not None
    assert advanced.window.subgoal_revision == first.window.subgoal_revision + 1
    assert advanced.window.window_content_hash != first.window.window_content_hash
    assert advanced.window.reference_session_id == first.window.reference_session_id
    assert advanced.window.full_reference_hash == first.window.full_reference_hash
    assert advanced.window.path_revision == first.window.path_revision


def test_forward_motion_inside_one_atomic_slice_keeps_subgoal_revision() -> None:
    context, _seed, reference, validation = _fixture()
    manager = LocalReferenceWindowManager()
    first_knot = _knot_near_arc(reference, 0.60)
    first = manager.update(
        _at(context, first_knot.pose, context.control_tick),
        reference,
        validation,
    )
    moved_pose = Pose2D(
        first_knot.pose.x + 0.03,
        first_knot.pose.y,
        first_knot.pose.yaw,
    )
    moved = manager.update(
        _at(context, moved_pose, context.control_tick + 1),
        reference,
        validation,
    )

    assert first.window is not None and moved.window is not None
    assert moved.reason_code == "window_unchanged"
    assert moved.effective_cursor_arc_m > first.effective_cursor_arc_m
    assert moved.window.subgoal_revision == first.window.subgoal_revision
    assert moved.window.window_content_hash == first.window.window_content_hash


def test_every_included_rotation_section_remains_whole() -> None:
    context, _seed, reference, validation = _fixture()
    rotation = next(
        section
        for section in reference.sections
        if section.section_kind is ReferenceSectionKind.ROTATE
    )
    rotation_pose = reference.knots[rotation.first_knot_index].pose

    update = LocalReferenceWindowManager().update(
        _at(context, rotation_pose, context.control_tick),
        reference,
        validation,
    )

    assert update.window is not None
    assert window_is_exact_slice(reference, update.window)
    for section in update.window.sections:
        if section.section_kind is ReferenceSectionKind.ROTATE:
            assert section is reference.sections[section.section_index]
            assert update.window.start_knot_index <= section.first_knot_index
            assert update.window.end_knot_index >= section.last_knot_index


def test_terminal_window_includes_rejoin_and_stop_marker() -> None:
    context, _seed, reference, validation = _fixture()
    terminal_pose = reference.knots[-2].pose

    update = LocalReferenceWindowManager().update(
        _at(context, terminal_pose, context.control_tick),
        reference,
        validation,
    )

    assert update.window is not None
    assert update.window.terminal_rejoin_included
    assert update.window.end_knot_index == reference.rejoin_knot_index


def test_small_cursor_regression_is_clamped_but_large_regression_is_stale() -> None:
    context, _seed, reference, validation = _fixture()
    manager = LocalReferenceWindowManager()
    forward_knot = _knot_near_arc(reference, 0.60)
    forward = manager.update(
        _at(context, forward_knot.pose, context.control_tick),
        reference,
        validation,
    )
    small_back_pose = Pose2D(
        forward_knot.pose.x - 0.02,
        forward_knot.pose.y,
        forward_knot.pose.yaw,
    )

    clamped = manager.update(
        _at(context, small_back_pose, context.control_tick + 1),
        reference,
        validation,
    )
    large_back = manager.update(
        _at(
            context,
            _knot_near_arc(reference, 0.40).pose,
            context.control_tick + 2,
        ),
        reference,
        validation,
    )

    assert forward.effective_cursor_arc_m is not None
    assert clamped.raw_cursor_arc_m is not None
    assert clamped.raw_cursor_arc_m < forward.effective_cursor_arc_m
    assert clamped.effective_cursor_arc_m == forward.effective_cursor_arc_m
    assert large_back.status is WindowUpdateStatus.STALE_INPUT
    assert large_back.reason_code == "cursor_regression_exceeded"


def test_control_tick_regression_is_stale_and_does_not_replace_state() -> None:
    context, _seed, reference, validation = _fixture()
    manager = LocalReferenceWindowManager()
    current = _at(context, context.current_robot_pose, context.control_tick + 1)
    first = manager.update(current, reference, validation)

    stale = manager.update(context, reference, validation)
    next_tick = manager.update(
        _at(context, context.current_robot_pose, context.control_tick + 2),
        reference,
        validation,
    )

    assert stale.status is WindowUpdateStatus.STALE_INPUT
    assert stale.reason_code == "source_control_tick_regression"
    assert first.window is not None and next_tick.window is not None
    assert next_tick.window.subgoal_revision == first.window.subgoal_revision


def test_new_path_or_session_requires_a_new_manager() -> None:
    context, seed, reference, validation = _fixture()
    manager = LocalReferenceWindowManager()
    manager.update(context, reference, validation)
    changed = replace(
        reference,
        path_revision=reference.path_revision + 1,
        reference_session_id=_hash("new-r4-window-session"),
        reference_content_hash="",
    )
    changed_validation = validate_local_maneuver_reference(
        context,
        changed,
        spatial_seed=seed,
    )
    assert changed_validation.passed

    rejected = manager.update(context, changed, changed_validation)
    accepted_by_new_manager = LocalReferenceWindowManager().update(
        context,
        changed,
        changed_validation,
    )

    assert rejected.status is WindowUpdateStatus.STALE_INPUT
    assert rejected.reason_code == "reference_session_or_path_changed"
    assert accepted_by_new_manager.status is WindowUpdateStatus.WINDOW_READY


def test_failed_or_mismatched_validation_is_never_window_ready() -> None:
    context, _seed, reference, validation = _fixture()
    failed = replace(
        validation,
        passed=False,
        failure_codes=("test_failure",),
        validation_content_hash="",
    )

    rejected = LocalReferenceWindowManager().update(context, reference, failed)

    assert rejected.status is WindowUpdateStatus.INVALID_INPUT
    assert rejected.reason_code == "reference_validation_failed"
    assert rejected.window is None


def test_tick_time_mismatch_is_invalid() -> None:
    context, _seed, reference, validation = _fixture()
    mismatched = replace(
        context,
        simulation_time_s=context.simulation_time_s + 0.01,
        context_content_hash="",
    )

    result = LocalReferenceWindowManager().update(mismatched, reference, validation)

    assert result.status is WindowUpdateStatus.INVALID_INPUT
    assert result.reason_code == "control_tick_time_mismatch"


def test_exact_slice_checker_rejects_in_place_window_tamper() -> None:
    context, _seed, reference, validation = _fixture()
    update = LocalReferenceWindowManager().update(context, reference, validation)
    assert update.window is not None
    object.__setattr__(
        update.window,
        "knots",
        update.window.knots[:-1],
    )

    assert not window_is_exact_slice(reference, update.window)


def test_projection_is_deterministic_on_rotation_anchor() -> None:
    _context, _seed, reference, _validation = _fixture()
    rotation = next(
        section
        for section in reference.sections
        if section.section_kind is ReferenceSectionKind.ROTATE
    )
    pose = reference.knots[rotation.first_knot_index].pose

    first = project_reference_cursor(reference, pose)
    second = project_reference_cursor(reference, pose)

    assert first == second
    assert not first.ambiguous


def test_nonadjacent_self_overlap_projection_is_ambiguous() -> None:
    _context, _seed, reference, _validation = _fixture()
    overlapped = deepcopy(reference)
    first_left = overlapped.knots[10]
    first_right = overlapped.knots[11]
    later_left = overlapped.knots[20]
    later_right = overlapped.knots[21]
    object.__setattr__(later_left, "pose", first_left.pose)
    object.__setattr__(later_right, "pose", first_right.pose)
    midpoint = Pose2D(
        (first_left.pose.x + first_right.pose.x) / 2.0,
        (first_left.pose.y + first_right.pose.y) / 2.0,
        first_left.pose.yaw,
    )

    projection = project_reference_cursor(overlapped, midpoint)
    resolved = project_reference_cursor(
        overlapped,
        midpoint,
        cursor_hint_m=later_left.cumulative_translation_arc_m,
    )

    assert projection.ambiguous
    assert projection.ambiguity_reason in {
        "ambiguous_reference_projection",
        "opposite_tangent_projection",
    }
    assert not resolved.ambiguous
    assert resolved.cursor_arc_m >= later_left.cumulative_translation_arc_m


def test_cursor_hint_precedes_heading_during_in_place_rotation_at_overlap() -> None:
    _context, _seed, reference, _validation = _fixture()
    overlapped = deepcopy(reference)
    first_left = overlapped.knots[10]
    first_right = overlapped.knots[11]
    later_left = overlapped.knots[20]
    later_right = overlapped.knots[21]
    object.__setattr__(later_left, "pose", first_right.pose)
    object.__setattr__(later_right, "pose", first_left.pose)
    midpoint = Pose2D(
        (first_left.pose.x + first_right.pose.x) / 2.0,
        (first_left.pose.y + first_right.pose.y) / 2.0,
        first_left.pose.yaw,
    )

    resolved = project_reference_cursor(
        overlapped,
        midpoint,
        cursor_hint_m=later_left.cumulative_translation_arc_m,
    )

    assert not resolved.ambiguous
    assert resolved.cursor_arc_m >= later_left.cumulative_translation_arc_m


def test_self_near_reverse_branch_does_not_project_to_completed_forward_edge() -> None:
    case = next(
        item for item in public_local_reference_cases() if item.public_id == "wide-straight-left"
    )
    result = evaluate_local_reference_public_case(case)
    reference = result.reference_set.candidates[0]
    cursor_hint_m = 1.0797968221734229
    failure_pose = Pose2D(
        1.5137222365350886,
        0.9258915970585265,
        1.41115630999078435,
    )

    unhinted = project_reference_cursor(reference, failure_pose)
    resolved = project_reference_cursor(
        reference,
        failure_pose,
        cursor_hint_m=cursor_hint_m,
    )

    assert unhinted.source_section_index == 1
    assert unhinted.cursor_arc_m < cursor_hint_m - 0.05
    assert not resolved.ambiguous
    assert resolved.source_section_index == 5
    assert resolved.cursor_arc_m >= cursor_hint_m


def test_window_module_does_not_import_builder_corpus_or_evaluator() -> None:
    module_path = (
        Path(__file__).parents[1] / "src" / "hospital_path_lab" / "local_reference_window.py"
    )
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    forbidden = (
        "local_reference_builder",
        "dynamic_corpus",
        "dynamic_evaluation",
        "expectation_category",
        "oracle_spec",
        "hidden",
    )
    names: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            names.append(node.module or "")
            names.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.Name):
            names.append(node.id)
    assert not any(token in value for token in forbidden for value in names)
