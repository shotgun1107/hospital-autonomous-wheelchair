from __future__ import annotations

import ast
from dataclasses import replace
from hashlib import sha256
from pathlib import Path

import numpy as np
import pytest

from hospital_path_lab.contracts import GridSnapshot, Pose2D, SnapshotMetadata
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_reference_builder import (
    LOCAL_REFERENCE_BUILDER_VERSION,
    LocalReferenceSourceError,
    SpatialReferenceSource,
    build_spatial_local_reference,
    build_spatial_reference_set,
    project_validated_spatial_seed,
)
from hospital_path_lab.local_reference_contracts import (
    REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
    LocalManeuverKind,
    ObservationDependency,
    ReferenceBuildContext,
    ReferenceBuildStatus,
    ReferenceEvidenceLevel,
    ReferenceKnotRole,
    ReferenceSectionKind,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.spatial_oracle_contracts import (
    SPATIAL_ORACLE_RESULT_SCHEMA_VERSION,
    SPATIAL_ORACLE_SCHEMA_VERSION,
    SPATIAL_ORACLE_VERSION,
    BoundedSpatialOracleResult,
    ManeuverSide,
    SpatialAllowedRegion,
    SpatialLatticeConfig,
    SpatialLatticeState,
    SpatialOracleStatus,
    SpatialPrimitive,
    SpatialPrimitiveKind,
    SpatialReferenceSegment,
    SpatialRejoinGoal,
    SpatialSearchRegion,
    build_bounded_spatial_request,
    spatial_grid_content_hash,
)
from hospital_path_lab.spatial_oracle_validation import validate_spatial_oracle_path
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _hash(label: str) -> str:
    return sha256(label.encode("utf-8")).hexdigest()


def _state_pose(grid: GridMap, state: SpatialLatticeState) -> Pose2D:
    pose = grid.cell_to_pose((state.x_cell, state.y_cell))
    return Pose2D(pose.x, pose.y, state.heading_index * np.pi / 4.0)


def _source(
    side: ManeuverSide = ManeuverSide.LEFT,
) -> tuple[SpatialReferenceSource, ReferenceBuildContext]:
    grid = GridMap(np.zeros((100, 120), dtype=np.bool_), resolution_m=0.02)
    start_state = SpatialLatticeState(20, 50, 0, False)
    states = [start_state]
    kinds: list[SpatialPrimitiveKind] = []

    def add(kind: SpatialPrimitiveKind, state: SpatialLatticeState) -> None:
        kinds.append(kind)
        states.append(state)

    turn_out = (
        (SpatialPrimitiveKind.ROTATE_LEFT_45, 1)
        if side is ManeuverSide.LEFT
        else (SpatialPrimitiveKind.ROTATE_RIGHT_45, 7)
    )
    lateral_heading = 2 if side is ManeuverSide.LEFT else 6
    lateral_y = 55 if side is ManeuverSide.LEFT else 45
    add(turn_out[0], SpatialLatticeState(20, 50, turn_out[1], False))
    add(turn_out[0], SpatialLatticeState(20, 50, lateral_heading, False))
    add(
        SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
        SpatialLatticeState(20, lateral_y, lateral_heading, True),
    )
    turn_to_east = (
        SpatialPrimitiveKind.ROTATE_RIGHT_45
        if side is ManeuverSide.LEFT
        else SpatialPrimitiveKind.ROTATE_LEFT_45
    )
    intermediate_heading = 1 if side is ManeuverSide.LEFT else 7
    add(turn_to_east, SpatialLatticeState(20, lateral_y, intermediate_heading, True))
    add(turn_to_east, SpatialLatticeState(20, lateral_y, 0, True))
    for x_cell in range(25, 81, 5):
        add(
            SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
            SpatialLatticeState(x_cell, lateral_y, 0, True),
        )
    turn_return = (
        SpatialPrimitiveKind.ROTATE_RIGHT_45
        if side is ManeuverSide.LEFT
        else SpatialPrimitiveKind.ROTATE_LEFT_45
    )
    return_heading = 6 if side is ManeuverSide.LEFT else 2
    first_return_heading = 7 if side is ManeuverSide.LEFT else 1
    add(turn_return, SpatialLatticeState(80, lateral_y, first_return_heading, True))
    add(turn_return, SpatialLatticeState(80, lateral_y, return_heading, True))
    add(
        SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
        SpatialLatticeState(80, 50, return_heading, True),
    )
    turn_align = (
        SpatialPrimitiveKind.ROTATE_LEFT_45
        if side is ManeuverSide.LEFT
        else SpatialPrimitiveKind.ROTATE_RIGHT_45
    )
    final_intermediate = 7 if side is ManeuverSide.LEFT else 1
    add(turn_align, SpatialLatticeState(80, 50, final_intermediate, True))
    add(turn_align, SpatialLatticeState(80, 50, 0, True))

    path = tuple(_state_pose(grid, state) for state in states)
    primitives = tuple(
        SpatialPrimitive(
            kind=kind,
            start_pose=path[index],
            end_pose=path[index + 1],
            start_state=states[index],
            end_state=states[index + 1],
        )
        for index, kind in enumerate(kinds)
    )
    reference = (path[0], path[-1])
    region = tuple((x, y) for y in range(10, 90) for x in range(5, 115))
    request = build_bounded_spatial_request(
        schema_version=SPATIAL_ORACLE_SCHEMA_VERSION,
        map_id=f"r4-builder-{side.value}",
        map_revision=3,
        mission_revision=7,
        static_grid=grid,
        forbidden_cells=(),
        allowed_region=SpatialAllowedRegion(),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        start_pose=path[0],
        rejoin_goal=SpatialRejoinGoal(path[-1]),
        reference_segment=SpatialReferenceSegment(*reference),
        maneuver_side=side,
        search_region=SpatialSearchRegion(region),
        lattice_config=SpatialLatticeConfig(),
        source_projection_hash=_hash(f"projection-{side.value}"),
    )
    validation = validate_spatial_oracle_path(request, path, primitives)
    assert validation.passed, validation.failure_codes
    result = BoundedSpatialOracleResult(
        schema_version=SPATIAL_ORACLE_RESULT_SCHEMA_VERSION,
        oracle_version=SPATIAL_ORACLE_VERSION,
        status=SpatialOracleStatus.SPATIALLY_FEASIBLE,
        termination_reason="goal_reached",
        request_content_hash=request.request_content_hash,
        map_id=request.map_id,
        map_revision=request.map_revision,
        mission_revision=request.mission_revision,
        grid_content_hash=request.grid_content_hash,
        vehicle_profile_hash=request.vehicle_profile_hash,
        search_region_hash=request.search_region.content_hash,
        lattice_config_hash=request.lattice_config.content_hash,
        path=path,
        primitive_sequence=primitives,
        path_length_m=validation.path_length_m,
        reverse_length_m=validation.reverse_length_m,
        rotation_count=validation.rotation_count,
        minimum_clearance_m=validation.minimum_clearance_m,
        minimum_physical_clearance_m=validation.minimum_physical_clearance_m,
        minimum_forbidden_clearance_m=validation.minimum_forbidden_clearance_m,
        minimum_allowed_boundary_clearance_m=(validation.minimum_allowed_boundary_clearance_m),
        generated_edges=10,
        expanded_states=5,
        peak_open_states=3,
        exhaustive=False,
        validation=validation,
        limitations=("simulation_only",),
        elapsed_nonqualification_ns=10,
    )
    snapshot = GridSnapshot(
        SnapshotMetadata(
            map_id=request.map_id,
            map_revision=request.map_revision,
            mission_revision=request.mission_revision,
            observation_revision=0,
            seed=20260814,
            content_hash=_hash("grid-snapshot"),
        ),
        grid,
    )
    context = ReferenceBuildContext(
        schema_version=REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
        mission_id="mission-r4-builder",
        stop_epoch=2,
        map_id=request.map_id,
        map_revision=request.map_revision,
        mission_revision=request.mission_revision,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        static_grid_snapshot=snapshot,
        grid_content_hash=spatial_grid_content_hash(grid),
        allowed_region=request.allowed_region,
        allowed_region_hash=request.allowed_region.content_hash,
        forbidden_cells=request.forbidden_cells,
        forbidden_region_hash=canonical_content_hash(request.forbidden_cells),
        vehicle_profile=request.vehicle_profile,
        vehicle_profile_hash=request.vehicle_profile_hash,
        original_reference=reference,
        original_reference_hash=canonical_content_hash(reference),
        current_robot_pose=request.start_pose,
        control_tick=40,
        simulation_time_s=2.0,
    )
    return SpatialReferenceSource(request, result), context


def _nonfeasible_source(
    status: SpatialOracleStatus,
) -> tuple[SpatialReferenceSource, ReferenceBuildContext]:
    source, context = _source()
    if status is SpatialOracleStatus.RESOURCE_LIMIT:
        reason = "max_expanded_states"
        exhaustive = False
        generated, expanded, peak = 10, 5, 3
    elif status is SpatialOracleStatus.INVALID_INPUT:
        reason = "request_content_hash_mismatch"
        exhaustive = False
        generated, expanded, peak = 0, 0, 0
    else:
        reason = "bounded_lattice_exhausted"
        exhaustive = True
        generated, expanded, peak = 10, 5, 3
    result = replace(
        source.result,
        status=status,
        termination_reason=reason,
        path=(),
        primitive_sequence=(),
        path_length_m=None,
        reverse_length_m=None,
        rotation_count=None,
        minimum_clearance_m=None,
        minimum_physical_clearance_m=None,
        minimum_forbidden_clearance_m=None,
        minimum_allowed_boundary_clearance_m=None,
        generated_edges=generated,
        expanded_states=expanded,
        peak_open_states=peak,
        exhaustive=exhaustive,
        validation=None,
        semantic_content_hash="",
    )
    return SpatialReferenceSource(source.request, result), context


def test_project_feasible_result_binds_request_validation_path_and_context() -> None:
    source, context = _source()

    seed = project_validated_spatial_seed(context, source)

    assert seed.source_spatial_result_hash == source.result.semantic_content_hash
    assert seed.source_spatial_request_hash == source.request.request_content_hash
    assert seed.source_validation_hash == source.result.validation.validation_content_hash
    assert seed.pose_heading_path == source.result.path
    assert seed.primitive_sequence == source.result.primitive_sequence


def test_builder_preserves_rotation_sections_arc_and_spatial_only_limit() -> None:
    source, context = _source()
    seed = project_validated_spatial_seed(context, source)

    reference = build_spatial_local_reference(
        context,
        seed,
        maneuver_revision=8,
        path_revision=3,
    )

    kinds = tuple(section.section_kind for section in reference.sections)
    assert kinds[0] is ReferenceSectionKind.DEPART
    assert kinds[-1] is ReferenceSectionKind.REJOIN
    assert ReferenceSectionKind.BYPASS in kinds
    assert ReferenceSectionKind.RETURN in kinds
    rotations = tuple(
        section
        for section in reference.sections
        if section.section_kind is ReferenceSectionKind.ROTATE
    )
    assert rotations
    for section in rotations:
        first = reference.knots[section.first_knot_index]
        last = reference.knots[section.last_knot_index]
        assert ReferenceKnotRole.ROTATION_ENTRY in first.knot_roles
        assert ReferenceKnotRole.ROTATION_EXIT in last.knot_roles
        assert first.pose.x == last.pose.x
        assert first.pose.y == last.pose.y
    assert reference.knots[-1].cumulative_translation_arc_m == pytest.approx(
        source.result.path_length_m
    )
    assert reference.evidence_level is ReferenceEvidenceLevel.SPATIAL_ONLY
    assert reference.source_temporal_evidence_hash is None
    assert "spatial_only_no_ordered_overtake_claim" in reference.limitations
    assert reference.reference_content_hash == reference.expected_content_hash


def test_left_and_right_sources_build_distinct_deterministic_candidates() -> None:
    left_source, left_context = _source(ManeuverSide.LEFT)
    right_source, _ = _source(ManeuverSide.RIGHT)
    right_request = replace(
        right_source.request,
        map_id=left_context.map_id,
        request_content_hash="",
    )
    right_result = replace(
        right_source.result,
        map_id=left_context.map_id,
        request_content_hash=right_request.request_content_hash,
        validation=replace(
            right_source.result.validation,
            request_content_hash=right_request.request_content_hash,
            validation_content_hash="",
        ),
        semantic_content_hash="",
    )
    right_source = SpatialReferenceSource(right_request, right_result)

    first = build_spatial_reference_set(
        left_context,
        (right_source, left_source),
        maneuver_revision=9,
        path_revision=4,
        elapsed_nonqualification_ns=1,
    )
    repeated = build_spatial_reference_set(
        left_context,
        (left_source, right_source),
        maneuver_revision=9,
        path_revision=4,
        elapsed_nonqualification_ns=999,
    )

    assert tuple(candidate.maneuver_kind for candidate in first.candidates) == (
        LocalManeuverKind.PASS_LEFT,
        LocalManeuverKind.PASS_RIGHT,
    )
    assert first.semantic_content_hash == repeated.semantic_content_hash


def test_resource_invalid_and_infeasible_statuses_are_not_turned_into_paths() -> None:
    resource, context = _nonfeasible_source(SpatialOracleStatus.RESOURCE_LIMIT)
    invalid, _ = _nonfeasible_source(SpatialOracleStatus.INVALID_INPUT)
    infeasible, _ = _nonfeasible_source(SpatialOracleStatus.SPATIALLY_INFEASIBLE)

    resource_set = build_spatial_reference_set(
        context,
        (resource,),
        maneuver_revision=1,
        path_revision=1,
        elapsed_nonqualification_ns=0,
    )
    invalid_set = build_spatial_reference_set(
        context,
        (invalid,),
        maneuver_revision=1,
        path_revision=1,
        elapsed_nonqualification_ns=0,
    )
    infeasible_set = build_spatial_reference_set(
        context,
        (infeasible,),
        maneuver_revision=1,
        path_revision=1,
        elapsed_nonqualification_ns=0,
    )

    assert resource_set.status is ReferenceBuildStatus.SEARCH_INCONCLUSIVE
    assert invalid_set.status is ReferenceBuildStatus.INVALID_INPUT
    assert infeasible_set.status is ReferenceBuildStatus.NO_REFERENCE
    assert not resource_set.candidates
    assert not invalid_set.candidates
    assert not infeasible_set.candidates


def test_any_invalid_or_resource_source_prevents_partial_candidate_set() -> None:
    feasible, context = _source()
    resource, _ = _nonfeasible_source(SpatialOracleStatus.RESOURCE_LIMIT)
    invalid, _ = _nonfeasible_source(SpatialOracleStatus.INVALID_INPUT)

    with_resource = build_spatial_reference_set(
        context,
        (feasible, resource),
        maneuver_revision=1,
        path_revision=1,
        elapsed_nonqualification_ns=0,
    )
    with_invalid = build_spatial_reference_set(
        context,
        (feasible, invalid),
        maneuver_revision=1,
        path_revision=1,
        elapsed_nonqualification_ns=0,
    )

    assert with_resource.status is ReferenceBuildStatus.SEARCH_INCONCLUSIVE
    assert with_invalid.status is ReferenceBuildStatus.INVALID_INPUT
    assert not with_resource.candidates
    assert not with_invalid.candidates


def test_source_and_context_hash_or_provenance_tamper_is_fail_closed() -> None:
    source, context = _source()
    object.__setattr__(source.result, "semantic_content_hash", _hash("forged"))
    with pytest.raises(LocalReferenceSourceError, match="source_hash_mismatch"):
        project_validated_spatial_seed(context, source)

    source, context = _source()
    wrong_snapshot = replace(
        context.static_grid_snapshot,
        metadata=replace(
            context.static_grid_snapshot.metadata,
            map_revision=context.map_revision + 1,
        ),
    )
    wrong_context = replace(
        context,
        map_revision=context.map_revision + 1,
        static_grid_snapshot=wrong_snapshot,
        context_content_hash="",
    )
    with pytest.raises(LocalReferenceSourceError, match="provenance"):
        project_validated_spatial_seed(wrong_context, source)

    source, context = _source()
    wrong_heading = replace(
        context,
        current_robot_pose=replace(
            context.current_robot_pose,
            yaw=context.current_robot_pose.yaw + np.pi / 4.0,
        ),
        context_content_hash="",
    )
    with pytest.raises(LocalReferenceSourceError, match="start_pose_mismatch"):
        project_validated_spatial_seed(wrong_heading, source)


def test_builder_rejects_multi_segment_projection_without_changing_source() -> None:
    source, context = _source()
    seed = project_validated_spatial_seed(context, source)
    extra = Pose2D(
        context.original_reference[-1].x + 0.20,
        context.original_reference[-1].y,
        0.0,
    )
    multi = (*context.original_reference, extra)
    multi_context = replace(
        context,
        original_reference=multi,
        original_reference_hash=canonical_content_hash(multi),
        context_content_hash="",
    )

    with pytest.raises(LocalReferenceSourceError, match="multi_segment"):
        build_spatial_local_reference(
            multi_context,
            seed,
            maneuver_revision=1,
            path_revision=1,
        )

    result = build_spatial_reference_set(
        multi_context,
        (source,),
        maneuver_revision=1,
        path_revision=1,
        elapsed_nonqualification_ns=0,
    )
    assert result.status is ReferenceBuildStatus.NO_REFERENCE
    assert "multi_segment_projection_unsupported" in result.limitations


def test_result_provenance_must_match_request_even_when_result_hash_is_recomputed() -> None:
    source, context = _source()
    changed = replace(
        source.result,
        search_region_hash=_hash("wrong-search-region"),
        semantic_content_hash="",
    )

    with pytest.raises(LocalReferenceSourceError, match="source_result_provenance"):
        project_validated_spatial_seed(
            context,
            SpatialReferenceSource(source.request, changed),
        )


def test_builder_module_has_no_corpus_label_or_hidden_dependency() -> None:
    path = Path(__file__).parents[1] / "src/hospital_path_lab/local_reference_builder.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    imported = {node.module or "" for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
    identifiers = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
    source_text = path.read_text(encoding="utf-8")

    assert LOCAL_REFERENCE_BUILDER_VERSION in source_text
    assert not any("dynamic_corpus" in module for module in imported)
    assert (
        not {
            "expectation_category",
            "oracle_spec",
            "latent_case_id",
            "hidden_seed",
        }
        & identifiers
    )
