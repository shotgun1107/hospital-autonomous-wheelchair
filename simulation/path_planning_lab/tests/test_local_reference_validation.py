from __future__ import annotations

import ast
from copy import deepcopy
from pathlib import Path

from test_local_reference_builder import _source

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.local_reference_builder import (
    build_spatial_local_reference,
    project_validated_spatial_seed,
)
from hospital_path_lab.local_reference_contracts import (
    LocalManeuverKind,
    ReferenceKnotRole,
    ReferenceSectionKind,
)
from hospital_path_lab.local_reference_validation import (
    LOCAL_REFERENCE_VALIDATOR_VERSION,
    validate_local_maneuver_reference,
)
from hospital_path_lab.spatial_oracle_contracts import ManeuverSide, SpatialAllowedRegion


def _reference_fixture(side: ManeuverSide = ManeuverSide.LEFT):
    source, context = _source(side)
    seed = project_validated_spatial_seed(context, source)
    reference = build_spatial_local_reference(
        context,
        seed,
        maneuver_revision=4,
        path_revision=2,
    )
    return context, seed, reference


def _deepcopy_reference():
    context, seed, reference = _reference_fixture()
    return context, seed, deepcopy(reference)


def test_independent_validation_passes_and_is_deterministic() -> None:
    context, seed, reference = _reference_fixture()

    first = validate_local_maneuver_reference(
        context,
        reference,
        spatial_seed=seed,
    )
    second = validate_local_maneuver_reference(
        context,
        reference,
        spatial_seed=seed,
    )

    assert first.passed
    assert first.failure_codes == ()
    assert first == second
    assert first.validator_version == LOCAL_REFERENCE_VALIDATOR_VERSION
    assert first.validation_content_hash == first.expected_content_hash
    assert first.minimum_clearance_m is not None
    assert first.minimum_clearance_m >= 0.08
    assert first.maximum_signed_side_excursion_m is not None
    assert first.maximum_signed_side_excursion_m >= 0.10
    assert first.rotation_section_count > 0
    assert first.swept_sample_count > len(reference.knots)


def test_right_side_source_and_reference_are_bound() -> None:
    context, seed, reference = _reference_fixture(ManeuverSide.RIGHT)

    result = validate_local_maneuver_reference(context, reference, spatial_seed=seed)

    assert result.passed
    assert reference.maneuver_kind is LocalManeuverKind.PASS_RIGHT


def test_spatial_only_reference_requires_the_bound_seed() -> None:
    context, _seed, reference = _reference_fixture()

    result = validate_local_maneuver_reference(context, reference)

    assert not result.passed
    assert "source_validation_missing" in result.failure_codes


def test_reference_and_context_hash_tamper_fail_closed() -> None:
    context, seed, reference = _reference_fixture()
    object.__setattr__(context, "mission_id", "tampered-mission")

    result = validate_local_maneuver_reference(context, reference, spatial_seed=seed)

    assert not result.passed
    assert "build_context_hash_mismatch" in result.failure_codes
    assert "map_or_mission_provenance_mismatch" in result.failure_codes


def test_malformed_reference_hash_is_reported_without_validator_crash() -> None:
    context, seed, reference = _reference_fixture()
    object.__setattr__(reference, "reference_content_hash", "not-a-sha256")

    result = validate_local_maneuver_reference(context, reference, spatial_seed=seed)

    assert not result.passed
    assert "reference_hash_mismatch" in result.failure_codes
    assert len(result.reference_content_hash) == 64


def test_rotation_marker_loss_is_detected_independently() -> None:
    context, seed, reference = _deepcopy_reference()
    rotation = next(
        section
        for section in reference.sections
        if section.section_kind is ReferenceSectionKind.ROTATE
    )
    entry = reference.knots[rotation.first_knot_index]
    object.__setattr__(
        entry,
        "knot_roles",
        (ReferenceKnotRole.ROTATION_ENTRY,),
    )

    result = validate_local_maneuver_reference(context, reference, spatial_seed=seed)

    assert not result.passed
    assert "reference_hash_mismatch" in result.failure_codes
    assert "rotation_marker_lost" in result.failure_codes


def test_source_primitive_mapping_loss_is_detected() -> None:
    context, seed, reference = _deepcopy_reference()
    mapped = next(section for section in reference.sections if section.source_primitive_indices)
    object.__setattr__(mapped, "source_primitive_indices", ())

    result = validate_local_maneuver_reference(context, reference, spatial_seed=seed)

    assert not result.passed
    assert "source_primitive_mapping_mismatch" in result.failure_codes


def test_source_path_pose_tamper_is_detected() -> None:
    context, seed, reference = _deepcopy_reference()
    knot = reference.knots[len(reference.knots) // 2]
    object.__setattr__(
        knot,
        "pose",
        Pose2D(knot.pose.x + 0.01, knot.pose.y, knot.pose.yaw),
    )

    result = validate_local_maneuver_reference(context, reference, spatial_seed=seed)

    assert not result.passed
    assert "source_path_index_mismatch" in result.failure_codes


def test_oriented_swept_geometry_rejects_a_boundary_intrusion() -> None:
    context, seed, reference = _deepcopy_reference()
    knot = reference.knots[len(reference.knots) // 2]
    object.__setattr__(knot, "pose", Pose2D(0.01, 0.01, 0.0))

    result = validate_local_maneuver_reference(context, reference, spatial_seed=seed)

    assert not result.passed
    assert "physical_clearance_violation" in result.failure_codes


def test_independent_geometry_rejects_forbidden_and_allowed_region_intrusions() -> None:
    context, seed, reference = _reference_fixture()
    sample_cell = context.static_grid_snapshot.grid.world_to_cell(
        reference.knots[len(reference.knots) // 2].pose
    )

    forbidden_context = deepcopy(context)
    object.__setattr__(forbidden_context, "forbidden_cells", (sample_cell,))
    object.__setattr__(
        forbidden_context.static_grid_snapshot,
        "forbidden_cells",
        frozenset((sample_cell,)),
    )
    forbidden_result = validate_local_maneuver_reference(
        forbidden_context,
        reference,
        spatial_seed=seed,
    )

    allowed_context = deepcopy(context)
    object.__setattr__(
        allowed_context,
        "allowed_region",
        SpatialAllowedRegion(cells=(sample_cell,), unrestricted=False),
    )
    allowed_result = validate_local_maneuver_reference(
        allowed_context,
        reference,
        spatial_seed=seed,
    )

    assert "forbidden_clearance_violation" in forbidden_result.failure_codes
    assert "allowed_boundary_clearance_violation" in allowed_result.failure_codes


def test_claimed_clearance_must_be_reproduced() -> None:
    context, seed, reference = _deepcopy_reference()
    object.__setattr__(reference, "minimum_validated_static_clearance_m", 0.9)

    result = validate_local_maneuver_reference(context, reference, spatial_seed=seed)

    assert not result.passed
    assert "source_minimum_clearance_not_reproduced" in result.failure_codes
    assert "source_minimum_clearance_mismatch" in result.failure_codes


def test_side_flip_without_geometry_change_is_rejected() -> None:
    context, seed, reference = _deepcopy_reference()
    object.__setattr__(reference, "maneuver_kind", LocalManeuverKind.PASS_RIGHT)

    result = validate_local_maneuver_reference(context, reference, spatial_seed=seed)

    assert not result.passed
    assert "source_side_mismatch" in result.failure_codes
    assert "opposite_side_excursion" in result.failure_codes


def test_expired_reference_validity_is_rejected() -> None:
    context, seed, reference = _reference_fixture()
    object.__setattr__(
        reference.validity,
        "valid_until_control_tick",
        context.control_tick,
    )
    object.__setattr__(context, "control_tick", context.control_tick + 1)

    result = validate_local_maneuver_reference(context, reference, spatial_seed=seed)

    assert not result.passed
    assert "reference_outside_validity_window" in result.failure_codes


def test_validator_source_does_not_import_builder_or_corpus_or_evaluator() -> None:
    module_path = (
        Path(__file__).parents[1] / "src" / "hospital_path_lab" / "local_reference_validation.py"
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
    imported_or_named: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_or_named.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imported_or_named.append(node.module or "")
            imported_or_named.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.Name):
            imported_or_named.append(node.id)
    assert not any(token in value for token in forbidden for value in imported_or_named)
