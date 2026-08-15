"""R4 local maneuver reference의 builder 독립 정적 검증기.

이 모듈은 builder의 내부 판정이나 R3 validation 결과를 성공 oracle로 재사용하지
않는다. immutable reference와 그 source seed/context만 입력받아 provenance, 구조,
side/rejoin 의미와 oriented swept-footprint clearance를 다시 계산한다.

Actor, corpus category, expectation oracle, controller와 이동 허가는 이 검증기의 범위가
아니다.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, ceil, cos, hypot, isfinite, pi, sin
from re import fullmatch

import numpy as np

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_reference_contracts import (
    LOCAL_REFERENCE_CONTRACT_VERSION,
    LOCAL_REFERENCE_SCHEMA_VERSION,
    R4_COMPARISON_TOLERANCE,
    R4_MINIMUM_CLEARANCE_M,
    LocalManeuverKind,
    LocalManeuverReference,
    ObservationDependency,
    ReferenceBuildContext,
    ReferenceEvidenceLevel,
    ReferenceKnotRole,
    ReferenceSectionKind,
    ReferenceTravelDirection,
    SpatialReferenceSeed,
    TemporalReferenceEvidence,
    TemporalReferenceGeometryEvidence,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.spatial_oracle_contracts import (
    ManeuverSide,
    SpatialPrimitiveKind,
    spatial_path_content_hash,
)

LOCAL_REFERENCE_VALIDATION_SCHEMA_VERSION = "local-reference-validation-v2"
LOCAL_REFERENCE_VALIDATOR_VERSION = "local-reference-validator-v2"
R4_TRANSLATION_SWEEP_STEP_M = 0.005
R4_ROTATION_SWEEP_STEP_RAD = pi / 360.0

_ROTATION_KINDS = frozenset(
    (SpatialPrimitiveKind.ROTATE_LEFT_45, SpatialPrimitiveKind.ROTATE_RIGHT_45)
)


@dataclass(frozen=True, slots=True)
class LocalReferenceValidation:
    """재현 가능한 R4 reference 검증 결과.

    wall-clock 시간은 연구 운영 진단일 뿐이므로 semantic hash에 넣지 않는다.
    """

    schema_version: str
    validator_version: str
    build_context_hash: str
    reference_content_hash: str
    source_spatial_seed_hash: str | None
    passed: bool
    failure_codes: tuple[str, ...]
    minimum_clearance_m: float | None
    minimum_physical_clearance_m: float | None
    minimum_forbidden_clearance_m: float | None
    minimum_allowed_boundary_clearance_m: float | None
    maximum_signed_side_excursion_m: float | None
    rotation_section_count: int
    swept_sample_count: int
    validation_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != LOCAL_REFERENCE_VALIDATION_SCHEMA_VERSION:
            raise ValueError("unsupported local reference validation schema")
        if self.validator_version != LOCAL_REFERENCE_VALIDATOR_VERSION:
            raise ValueError("unsupported local reference validator version")
        _require_sha256(self.build_context_hash, "build_context_hash")
        _require_sha256(self.reference_content_hash, "reference_content_hash")
        if self.source_spatial_seed_hash is not None:
            _require_sha256(self.source_spatial_seed_hash, "source_spatial_seed_hash")
        if not isinstance(self.passed, bool):
            raise TypeError("passed must be a bool")
        failures = tuple(sorted(set(self.failure_codes)))
        if any(not isinstance(code, str) or not code for code in failures):
            raise ValueError("failure_codes must contain non-empty strings")
        if self.passed == bool(failures):
            raise ValueError("passed validation must have no failures")
        object.__setattr__(self, "failure_codes", failures)
        for name in ("rotation_section_count", "swept_sample_count"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative exact integer")
        metrics = (
            self.minimum_clearance_m,
            self.minimum_physical_clearance_m,
            self.minimum_forbidden_clearance_m,
            self.minimum_allowed_boundary_clearance_m,
            self.maximum_signed_side_excursion_m,
        )
        if any(value is not None and not isfinite(value) for value in metrics):
            raise ValueError("validation metrics must be finite when present")
        if self.passed:
            if any(value is None for value in metrics):
                raise ValueError("passed validation requires all metrics")
            assert self.minimum_clearance_m is not None
            if self.minimum_clearance_m + R4_COMPARISON_TOLERANCE < R4_MINIMUM_CLEARANCE_M:
                raise ValueError("passed validation cannot violate frozen clearance")
        expected = self.expected_content_hash
        if self.validation_content_hash:
            _require_sha256(self.validation_content_hash, "validation_content_hash")
            if self.validation_content_hash != expected:
                raise ValueError("validation_content_hash mismatch")
        else:
            object.__setattr__(self, "validation_content_hash", expected)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "validator_version": self.validator_version,
                "build_context_hash": self.build_context_hash,
                "reference_content_hash": self.reference_content_hash,
                "source_spatial_seed_hash": self.source_spatial_seed_hash,
                "passed": self.passed,
                "failure_codes": self.failure_codes,
                "minimum_clearance_m": self.minimum_clearance_m,
                "minimum_physical_clearance_m": self.minimum_physical_clearance_m,
                "minimum_forbidden_clearance_m": self.minimum_forbidden_clearance_m,
                "minimum_allowed_boundary_clearance_m": (self.minimum_allowed_boundary_clearance_m),
                "maximum_signed_side_excursion_m": (self.maximum_signed_side_excursion_m),
                "rotation_section_count": self.rotation_section_count,
                "swept_sample_count": self.swept_sample_count,
            }
        )


def validate_local_maneuver_reference(
    context: ReferenceBuildContext,
    reference: LocalManeuverReference,
    *,
    spatial_seed: SpatialReferenceSeed | None = None,
    temporal_evidence: TemporalReferenceEvidence | None = None,
    temporal_geometry: TemporalReferenceGeometryEvidence | None = None,
) -> LocalReferenceValidation:
    """R4 reference를 source builder와 독립적으로 다시 검증한다."""

    if not isinstance(context, ReferenceBuildContext):
        raise TypeError("context must be a ReferenceBuildContext")
    if not isinstance(reference, LocalManeuverReference):
        raise TypeError("reference must be a LocalManeuverReference")
    if spatial_seed is not None and not isinstance(spatial_seed, SpatialReferenceSeed):
        raise TypeError("spatial_seed must be a SpatialReferenceSeed or None")
    if temporal_evidence is not None and not isinstance(
        temporal_evidence,
        TemporalReferenceEvidence,
    ):
        raise TypeError("temporal_evidence must be a TemporalReferenceEvidence or None")
    if temporal_geometry is not None and not isinstance(
        temporal_geometry,
        TemporalReferenceGeometryEvidence,
    ):
        raise TypeError(
            "temporal_geometry must be a TemporalReferenceGeometryEvidence or None"
        )

    failures: set[str] = set()
    _validate_integrity(
        context,
        reference,
        spatial_seed,
        temporal_evidence,
        temporal_geometry,
        failures,
    )
    _validate_structure(reference, spatial_seed, failures)
    maximum_excursion = _validate_side_and_rejoin(
        context,
        reference,
        spatial_seed,
        failures,
    )
    geometry = _validate_geometry(context, reference, failures)

    minimum_physical, minimum_forbidden, minimum_allowed, sample_count = geometry
    minimum = _minimum_present(minimum_physical, minimum_forbidden, minimum_allowed)
    if minimum is not None and (
        minimum + R4_COMPARISON_TOLERANCE < reference.minimum_validated_static_clearance_m
    ):
        failures.add("source_minimum_clearance_not_reproduced")
    if spatial_seed is not None and (
        abs(reference.minimum_validated_static_clearance_m - spatial_seed.minimum_clearance_m)
        > R4_COMPARISON_TOLERANCE
    ):
        failures.add("source_minimum_clearance_mismatch")

    return LocalReferenceValidation(
        schema_version=LOCAL_REFERENCE_VALIDATION_SCHEMA_VERSION,
        validator_version=LOCAL_REFERENCE_VALIDATOR_VERSION,
        build_context_hash=_safe_evidence_hash(
            context.context_content_hash,
            "invalid_build_context_hash",
        ),
        reference_content_hash=_safe_evidence_hash(
            reference.reference_content_hash,
            "invalid_reference_content_hash",
        ),
        source_spatial_seed_hash=(
            _safe_evidence_hash(
                spatial_seed.seed_content_hash,
                "invalid_spatial_seed_hash",
            )
            if spatial_seed is not None
            else None
        ),
        passed=not failures,
        failure_codes=tuple(failures),
        minimum_clearance_m=minimum,
        minimum_physical_clearance_m=minimum_physical,
        minimum_forbidden_clearance_m=minimum_forbidden,
        minimum_allowed_boundary_clearance_m=minimum_allowed,
        maximum_signed_side_excursion_m=maximum_excursion,
        rotation_section_count=sum(
            section.section_kind is ReferenceSectionKind.ROTATE for section in reference.sections
        ),
        swept_sample_count=sample_count,
    )


def _validate_integrity(
    context: ReferenceBuildContext,
    reference: LocalManeuverReference,
    spatial_seed: SpatialReferenceSeed | None,
    temporal_evidence: TemporalReferenceEvidence | None,
    temporal_geometry: TemporalReferenceGeometryEvidence | None,
    failures: set[str],
) -> None:
    try:
        if context.context_content_hash != context.expected_content_hash:
            failures.add("build_context_hash_mismatch")
    except (AttributeError, TypeError, ValueError):
        failures.add("build_context_hash_mismatch")
    try:
        if reference.reference_content_hash != reference.expected_content_hash:
            failures.add("reference_hash_mismatch")
    except (AttributeError, TypeError, ValueError):
        failures.add("reference_hash_mismatch")
    if reference.schema_version != LOCAL_REFERENCE_SCHEMA_VERSION:
        failures.add("unsupported_schema")
    if reference.reference_contract_version != LOCAL_REFERENCE_CONTRACT_VERSION:
        failures.add("unsupported_contract_version")
    provenance = (
        reference.mission_id == context.mission_id
        and reference.stop_epoch == context.stop_epoch
        and reference.map_id == context.map_id
        and reference.map_revision == context.map_revision
        and reference.mission_revision == context.mission_revision
        and reference.grid_content_hash == context.grid_content_hash
        and reference.vehicle_profile_hash == context.vehicle_profile_hash
        and reference.allowed_region_hash == context.allowed_region_hash
        and reference.forbidden_region_hash == context.forbidden_region_hash
        and reference.original_reference_hash == context.original_reference_hash
    )
    if not provenance:
        failures.add("map_or_mission_provenance_mismatch")
    if reference.stop_epoch != context.stop_epoch:
        failures.add("stop_epoch_mismatch")
    if (
        reference.observation_dependency != context.observation_dependency
        or reference.observation_revision != context.observation_revision
        or reference.observation_content_hash != context.observation_content_hash
    ):
        failures.add("observation_dependency_mismatch")
    validity = reference.validity
    if (
        validity.required_mission_id != context.mission_id
        or validity.required_stop_epoch != context.stop_epoch
        or validity.required_map_revision != context.map_revision
        or validity.required_mission_revision != context.mission_revision
        or validity.required_observation_revision != context.observation_revision
    ):
        failures.add("reference_validity_provenance_mismatch")
    if context.control_tick < validity.valid_from_control_tick or (
        validity.valid_until_control_tick is not None
        and context.control_tick > validity.valid_until_control_tick
    ):
        failures.add("reference_outside_validity_window")
    if not (
        validity.requires_actual_stop_confirmation
        and validity.requires_resume_authorization
        and validity.requires_local_safety_recheck
    ):
        failures.add("reference_safety_gate_requirement_missing")

    if reference.evidence_level is ReferenceEvidenceLevel.SPATIAL_ONLY:
        if reference.observation_dependency is not ObservationDependency.STATIC_ONLY:
            failures.add("observation_dependency_mismatch")
        if reference.source_temporal_evidence_hash is not None:
            failures.add("spatial_reference_claims_temporal_evidence")
        if spatial_seed is None:
            failures.add("source_validation_missing")
        if temporal_evidence is not None or temporal_geometry is not None:
            failures.add("spatial_reference_claims_temporal_evidence")
    elif reference.evidence_level is ReferenceEvidenceLevel.GROUND_TRUTH_TEMPORAL:
        is_pass = reference.maneuver_kind in (
            LocalManeuverKind.PASS_LEFT,
            LocalManeuverKind.PASS_RIGHT,
        )
        if is_pass and spatial_seed is not None:
            failures.add("temporal_reference_claims_spatial_seed")
        if temporal_evidence is None or (is_pass and temporal_geometry is None):
            failures.add("source_validation_missing")
        elif temporal_geometry is not None:
            _validate_temporal_sources(
                context,
                reference,
                temporal_evidence,
                temporal_geometry,
                failures,
            )
    if spatial_seed is None:
        return
    try:
        if spatial_seed.seed_content_hash != spatial_seed.expected_content_hash:
            failures.add("source_hash_mismatch")
    except (AttributeError, TypeError, ValueError):
        failures.add("source_hash_mismatch")
    if reference.source_spatial_seed_hash != spatial_seed.seed_content_hash:
        failures.add("source_hash_mismatch")
    if (
        spatial_seed.map_id != context.map_id
        or spatial_seed.map_revision != context.map_revision
        or spatial_seed.mission_revision != context.mission_revision
        or spatial_seed.grid_content_hash != context.grid_content_hash
        or spatial_seed.vehicle_profile_hash != context.vehicle_profile_hash
    ):
        failures.add("map_or_mission_provenance_mismatch")
    if not _poses_close(spatial_seed.start_pose, context.current_robot_pose):
        failures.add("start_pose_mismatch")
    try:
        expected_path_hash = spatial_path_content_hash(
            spatial_seed.pose_heading_path,
            spatial_seed.primitive_sequence,
        )
        if spatial_seed.source_path_content_hash != expected_path_hash:
            failures.add("source_hash_mismatch")
    except (AttributeError, TypeError, ValueError):
        failures.add("source_hash_mismatch")


def _validate_temporal_sources(
    context: ReferenceBuildContext,
    reference: LocalManeuverReference,
    temporal_evidence: TemporalReferenceEvidence,
    temporal_geometry: TemporalReferenceGeometryEvidence,
    failures: set[str],
) -> None:
    try:
        if temporal_evidence.evidence_content_hash != temporal_evidence.expected_content_hash:
            failures.add("source_hash_mismatch")
        if temporal_geometry.geometry_content_hash != temporal_geometry.expected_content_hash:
            failures.add("source_hash_mismatch")
    except (AttributeError, TypeError, ValueError):
        failures.add("source_hash_mismatch")
        return
    if reference.source_temporal_evidence_hash != temporal_evidence.evidence_content_hash:
        failures.add("source_hash_mismatch")
    if reference.source_temporal_geometry_hash != temporal_geometry.geometry_content_hash:
        failures.add("source_hash_mismatch")
    if (
        reference.maneuver_kind is not temporal_evidence.maneuver_kind
        or reference.maneuver_kind is not temporal_geometry.maneuver_kind
        or temporal_evidence.target_actor_binding_ids
        != temporal_geometry.target_actor_binding_ids
        or temporal_evidence.source_witness_hash != temporal_geometry.source_witness_hash
        or temporal_evidence.source_validation_hash != temporal_geometry.source_validation_hash
        or temporal_geometry.grid_content_hash != context.grid_content_hash
    ):
        failures.add("temporal_source_provenance_mismatch")
    expected_geometry_hash = canonical_content_hash(
        {
            "maneuver_kind": reference.maneuver_kind,
            "knots": reference.knots,
            "sections": reference.sections,
            "departure_knot_index": reference.departure_knot_index,
            "pass_section_index": reference.pass_section_index,
            "rejoin_knot_index": reference.rejoin_knot_index,
            "minimum_validated_static_clearance_m": (
                reference.minimum_validated_static_clearance_m
            ),
        }
    )
    if temporal_geometry.reference_geometry_hash != expected_geometry_hash:
        failures.add("temporal_geometry_hash_mismatch")
    if abs(
        temporal_geometry.minimum_static_clearance_m
        - reference.minimum_validated_static_clearance_m
    ) > R4_COMPARISON_TOLERANCE:
        failures.add("source_minimum_clearance_mismatch")


def _validate_structure(
    reference: LocalManeuverReference,
    spatial_seed: SpatialReferenceSeed | None,
    failures: set[str],
) -> None:
    knots = tuple(reference.knots)
    sections = tuple(reference.sections)
    if not knots or not sections:
        failures.add("reference_structure_invalid")
        return
    if tuple(knot.knot_index for knot in knots) != tuple(range(len(knots))):
        failures.add("reference_structure_invalid")
    if tuple(section.section_index for section in sections) != tuple(range(len(sections))):
        failures.add("reference_structure_invalid")
    expected_first = 0
    for section in sections:
        try:
            if section.section_content_hash != section.expected_content_hash:
                failures.add("section_hash_mismatch")
        except (AttributeError, TypeError, ValueError):
            failures.add("section_hash_mismatch")
        if (
            section.first_knot_index != expected_first
            or section.last_knot_index < section.first_knot_index
            or section.last_knot_index >= len(knots)
        ):
            failures.add("reference_structure_invalid")
            break
        for knot_index in range(section.first_knot_index, section.last_knot_index + 1):
            if knots[knot_index].section_index != section.section_index:
                failures.add("reference_structure_invalid")
        if section.entry_requires_stopped and (
            ReferenceKnotRole.STOP_MARKER not in knots[section.first_knot_index].knot_roles
        ):
            failures.add("section_stop_marker_lost")
        if section.exit_requires_stopped and (
            ReferenceKnotRole.STOP_MARKER not in knots[section.last_knot_index].knot_roles
        ):
            failures.add("section_stop_marker_lost")
        if section.section_kind is ReferenceSectionKind.HOLD:
            anchor = knots[section.first_knot_index]
            if any(
                _pose_distance(anchor.pose, knot.pose) > R4_COMPARISON_TOLERANCE
                or abs(anchor.cumulative_translation_arc_m - knot.cumulative_translation_arc_m)
                > R4_COMPARISON_TOLERANCE
                for knot in knots[section.first_knot_index + 1 : section.last_knot_index + 1]
            ):
                failures.add("hold_section_moved")
        expected_first = section.last_knot_index + 1
    if expected_first != len(knots):
        failures.add("reference_structure_invalid")

    previous_arc = knots[0].cumulative_translation_arc_m
    for left, right in zip(knots, knots[1:], strict=False):
        values = (
            left.pose.x,
            left.pose.y,
            left.pose.yaw,
            left.tangent_yaw,
            left.cumulative_translation_arc_m,
            right.pose.x,
            right.pose.y,
            right.pose.yaw,
            right.tangent_yaw,
            right.cumulative_translation_arc_m,
        )
        if not all(isfinite(value) for value in values):
            failures.add("non_finite_reference")
            continue
        if abs(_angle_delta(left.tangent_yaw, left.pose.yaw)) > (R4_COMPARISON_TOLERANCE):
            failures.add("tangent_heading_mismatch")
        distance = _pose_distance(left.pose, right.pose)
        arc_delta = right.cumulative_translation_arc_m - left.cumulative_translation_arc_m
        if arc_delta < -R4_COMPARISON_TOLERANCE:
            failures.add("translation_arc_regression")
        if distance > R4_COMPARISON_TOLERANCE:
            if abs(arc_delta - distance) > R4_COMPARISON_TOLERANCE:
                failures.add("translation_arc_mismatch")
        elif abs(arc_delta) > R4_COMPARISON_TOLERANCE:
            failures.add("translation_arc_mismatch")
        previous_arc = right.cumulative_translation_arc_m
    if previous_arc < -R4_COMPARISON_TOLERANCE:
        failures.add("translation_arc_regression")
    if abs(_angle_delta(knots[-1].tangent_yaw, knots[-1].pose.yaw)) > (R4_COMPARISON_TOLERANCE):
        failures.add("tangent_heading_mismatch")

    pass_kind = reference.maneuver_kind in (
        LocalManeuverKind.PASS_LEFT,
        LocalManeuverKind.PASS_RIGHT,
    )
    if pass_kind:
        kinds = tuple(section.section_kind for section in sections)
        required = (
            ReferenceSectionKind.DEPART,
            ReferenceSectionKind.BYPASS,
            ReferenceSectionKind.RETURN,
            ReferenceSectionKind.REJOIN,
        )
        try:
            positions = tuple(kinds.index(kind) for kind in required)
        except ValueError:
            failures.add("reference_structure_invalid")
        else:
            if positions != tuple(sorted(positions)):
                failures.add("reference_structure_invalid")
        if kinds[0] is not ReferenceSectionKind.DEPART or (
            kinds[-1] is not ReferenceSectionKind.REJOIN
        ):
            failures.add("reference_structure_invalid")
        if (
            reference.departure_knot_index is None
            or not 0 <= reference.departure_knot_index < len(knots)
            or knots[reference.departure_knot_index].section_index >= len(sections)
            or sections[knots[reference.departure_knot_index].section_index].section_kind
            is not ReferenceSectionKind.DEPART
        ):
            failures.add("reference_structure_invalid")
        if (
            reference.pass_section_index is None
            or not 0 <= reference.pass_section_index < len(sections)
            or sections[reference.pass_section_index].section_kind
            is not ReferenceSectionKind.BYPASS
        ):
            failures.add("reference_structure_invalid")

    rotation_count = 0
    for section in sections:
        if section.section_kind is not ReferenceSectionKind.ROTATE:
            continue
        rotation_count += 1
        if (
            section.first_knot_index == section.last_knot_index
            or not section.entry_requires_stopped
            or not section.exit_requires_stopped
        ):
            failures.add("rotation_marker_lost")
            continue
        section_knots = knots[section.first_knot_index : section.last_knot_index + 1]
        if (
            ReferenceKnotRole.ROTATION_ENTRY not in section_knots[0].knot_roles
            or ReferenceKnotRole.STOP_MARKER not in section_knots[0].knot_roles
            or ReferenceKnotRole.ROTATION_EXIT not in section_knots[-1].knot_roles
            or ReferenceKnotRole.STOP_MARKER not in section_knots[-1].knot_roles
        ):
            failures.add("rotation_marker_lost")
        anchor = section_knots[0]
        if any(
            _pose_distance(anchor.pose, knot.pose) > R4_COMPARISON_TOLERANCE
            or abs(anchor.cumulative_translation_arc_m - knot.cumulative_translation_arc_m)
            > R4_COMPARISON_TOLERANCE
            for knot in section_knots[1:]
        ):
            failures.add("rotation_not_atomic")
        if abs(_angle_delta(section_knots[-1].pose.yaw, anchor.pose.yaw)) <= (
            R4_COMPARISON_TOLERANCE
        ):
            failures.add("rotation_marker_lost")

    terminal_roles = set(knots[-1].knot_roles)
    if (
        reference.rejoin_knot_index != len(knots) - 1
        or not {
            ReferenceKnotRole.REJOIN,
            ReferenceKnotRole.STOP_MARKER,
        }
        <= terminal_roles
    ):
        failures.add("rejoin_marker_lost")

    _validate_travel_directions(knots, sections, spatial_seed, failures)

    if spatial_seed is None:
        return
    if len(spatial_seed.primitive_sequence) != len(spatial_seed.pose_heading_path) - 1:
        failures.add("path_primitive_length_mismatch")
        return
    primitive_indices = tuple(
        primitive_index
        for section in sections
        for primitive_index in section.source_primitive_indices
    )
    if tuple(sorted(primitive_indices)) != tuple(
        range(len(spatial_seed.primitive_sequence))
    ) or len(set(primitive_indices)) != len(primitive_indices):
        failures.add("source_primitive_mapping_mismatch")
    for section in sections:
        for primitive_index in section.source_primitive_indices:
            if not 0 <= primitive_index < len(spatial_seed.primitive_sequence):
                failures.add("source_primitive_mapping_mismatch")
                continue
            primitive = spatial_seed.primitive_sequence[primitive_index]
            if not _poses_close(
                primitive.start_pose,
                spatial_seed.pose_heading_path[primitive_index],
            ) or not _poses_close(
                primitive.end_pose,
                spatial_seed.pose_heading_path[primitive_index + 1],
            ):
                failures.add("path_primitive_endpoint_mismatch")
            is_rotation = primitive.kind in _ROTATION_KINDS or (
                _pose_distance(primitive.start_pose, primitive.end_pose) <= R4_COMPARISON_TOLERANCE
                and abs(_angle_delta(primitive.end_pose.yaw, primitive.start_pose.yaw))
                > R4_COMPARISON_TOLERANCE
            )
            if (section.section_kind is ReferenceSectionKind.ROTATE) != is_rotation:
                failures.add("source_primitive_section_mismatch")
        if section.source_primitive_indices:
            expected_source_indices = tuple(
                range(
                    section.source_primitive_indices[0],
                    section.source_primitive_indices[-1] + 2,
                )
            )
            actual_source_indices = tuple(
                knot.source_path_index
                for knot in knots[section.first_knot_index : section.last_knot_index + 1]
            )
            if actual_source_indices != expected_source_indices:
                failures.add("source_primitive_mapping_mismatch")
    for knot in knots:
        if not 0 <= knot.source_path_index < len(spatial_seed.pose_heading_path):
            failures.add("source_path_index_mismatch")
            continue
        if not _poses_close(knot.pose, spatial_seed.pose_heading_path[knot.source_path_index]):
            failures.add("source_path_index_mismatch")


def _validate_travel_directions(
    knots: tuple[object, ...],
    sections: tuple[object, ...],
    spatial_seed: SpatialReferenceSeed | None,
    failures: set[str],
) -> None:
    previous_travel: object | None = None
    intermediaries: list[object] = []
    for section in sections:
        try:
            section_knots = knots[section.first_knot_index : section.last_knot_index + 1]
            movement_present = any(
                _pose_distance(left.pose, right.pose) > R4_COMPARISON_TOLERANCE
                for left, right in zip(section_knots, section_knots[1:], strict=False)
            )
            direction = section.travel_direction
            if not isinstance(direction, ReferenceTravelDirection):
                failures.add("section_travel_direction_mismatch")
                continue
            if direction is ReferenceTravelDirection.NONE:
                primitive_indices = tuple(section.source_primitive_indices)
                abstract_connector = (
                    spatial_seed is not None
                    and bool(primitive_indices)
                    and all(
                        0 <= index < len(spatial_seed.primitive_sequence)
                        and spatial_seed.primitive_sequence[index].kind
                        is SpatialPrimitiveKind.ANCHOR_CONNECTOR
                        for index in primitive_indices
                    )
                )
                if movement_present and (
                    not abstract_connector
                    or not section.entry_requires_stopped
                    or not section.exit_requires_stopped
                    or ReferenceKnotRole.STOP_MARKER not in section_knots[0].knot_roles
                    or ReferenceKnotRole.STOP_MARKER not in section_knots[-1].knot_roles
                    or any(
                        ReferenceKnotRole.ANCHOR not in knot.knot_roles
                        for knot in section_knots
                    )
                ):
                    failures.add("section_travel_direction_mismatch")
                if previous_travel is not None:
                    intermediaries.append(section)
                continue
            if section.section_kind in (
                ReferenceSectionKind.ROTATE,
                ReferenceSectionKind.HOLD,
            ):
                failures.add("section_travel_direction_mismatch")
            if (
                previous_travel is not None
                and previous_travel.travel_direction is not direction
            ):
                previous_last = knots[previous_travel.last_knot_index]
                current_first = knots[section.first_knot_index]
                if (
                    not previous_travel.exit_requires_stopped
                    or not section.entry_requires_stopped
                    or ReferenceKnotRole.STOP_MARKER not in previous_last.knot_roles
                    or ReferenceKnotRole.STOP_MARKER not in current_first.knot_roles
                ):
                    failures.add("direction_transition_stop_missing")
                for intermediary in intermediaries:
                    first = knots[intermediary.first_knot_index]
                    last = knots[intermediary.last_knot_index]
                    if (
                        not intermediary.entry_requires_stopped
                        or not intermediary.exit_requires_stopped
                        or ReferenceKnotRole.STOP_MARKER not in first.knot_roles
                        or ReferenceKnotRole.STOP_MARKER not in last.knot_roles
                    ):
                        failures.add("direction_transition_stop_missing")
            previous_travel = section
            intermediaries = []
        except (AttributeError, IndexError, TypeError, ValueError):
            failures.add("section_travel_direction_mismatch")

    if spatial_seed is None:
        return
    for section in sections:
        for primitive_index in getattr(section, "source_primitive_indices", ()):
            if not 0 <= primitive_index < len(spatial_seed.primitive_sequence):
                continue
            try:
                expected = _source_primitive_travel_direction(
                    spatial_seed.primitive_sequence[primitive_index]
                )
            except ValueError:
                failures.add("source_primitive_direction_ambiguous")
                continue
            if section.travel_direction is not expected:
                failures.add("source_primitive_direction_mismatch")


def _source_primitive_travel_direction(primitive: object) -> ReferenceTravelDirection:
    kind = primitive.kind
    if kind is SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION:
        return ReferenceTravelDirection.FORWARD
    if kind is SpatialPrimitiveKind.REVERSE_ONE_TRANSLATION:
        return ReferenceTravelDirection.REVERSE
    if kind in _ROTATION_KINDS:
        return ReferenceTravelDirection.NONE
    if kind is not SpatialPrimitiveKind.ANCHOR_CONNECTOR:
        raise ValueError("unsupported source primitive direction")
    return ReferenceTravelDirection.NONE


def _validate_side_and_rejoin(
    context: ReferenceBuildContext,
    reference: LocalManeuverReference,
    spatial_seed: SpatialReferenceSeed | None,
    failures: set[str],
) -> float | None:
    if len(context.original_reference) != 2:
        failures.add("multi_segment_projection_unsupported")
        return None
    start, end = context.original_reference
    dx = end.x - start.x
    dy = end.y - start.y
    length = hypot(dx, dy)
    if length <= R4_COMPARISON_TOLERANCE:
        failures.add("ambiguous_reference_projection")
        return None
    tx = dx / length
    ty = dy / length
    offsets = tuple(
        tx * (knot.pose.y - start.y) - ty * (knot.pose.x - start.x) for knot in reference.knots
    )
    if not offsets or not all(isfinite(offset) for offset in offsets):
        failures.add("non_finite_reference")
        return None
    if reference.maneuver_kind is LocalManeuverKind.PASS_LEFT:
        excursion = max(offsets)
        if min(offsets) < -R4_COMPARISON_TOLERANCE:
            failures.add("opposite_side_excursion")
        expected_side = ManeuverSide.LEFT
    elif reference.maneuver_kind is LocalManeuverKind.PASS_RIGHT:
        excursion = -min(offsets)
        if max(offsets) > R4_COMPARISON_TOLERANCE:
            failures.add("opposite_side_excursion")
        expected_side = ManeuverSide.RIGHT
    else:
        excursion = max(abs(offset) for offset in offsets)
        expected_side = None
    if spatial_seed is not None:
        if expected_side is not None and spatial_seed.side is not expected_side:
            failures.add("source_side_mismatch")
        if excursion + R4_COMPARISON_TOLERANCE < (
            spatial_seed.rejoin_goal.minimum_side_excursion_m
        ):
            failures.add("required_side_excursion_missing")
        if not _poses_close(reference.knots[0].pose, spatial_seed.start_pose):
            failures.add("start_pose_mismatch")
        terminal = reference.knots[-1].pose
        goal = spatial_seed.rejoin_goal
        if (
            _pose_distance(terminal, goal.pose)
            > goal.position_tolerance_m + R4_COMPARISON_TOLERANCE
            or abs(_angle_delta(terminal.yaw, goal.pose.yaw))
            > goal.heading_tolerance_rad + R4_COMPARISON_TOLERANCE
        ):
            failures.add("rejoin_goal_mismatch")
    return excursion


def _validate_geometry(
    context: ReferenceBuildContext,
    reference: LocalManeuverReference,
    failures: set[str],
) -> tuple[float | None, float | None, float | None, int]:
    grid = context.static_grid_snapshot.grid
    physical = CollisionChecker(grid, context.vehicle_profile)
    forbidden = CollisionChecker(
        grid,
        context.vehicle_profile,
        forbidden_cells=frozenset(context.forbidden_cells),
    )
    allowed = (
        None
        if context.allowed_region.unrestricted
        else CollisionChecker(
            _complement_grid(grid, context.allowed_region.cells),
            context.vehicle_profile,
        )
    )
    minimum_physical: float | None = None
    minimum_forbidden: float | None = None
    minimum_allowed: float | None = None
    sample_count = 0
    half_diagonal = hypot(
        context.vehicle_profile.collision_length_m / 2.0,
        context.vehicle_profile.collision_width_m / 2.0,
    )
    knots = tuple(reference.knots)
    if not knots:
        failures.add("reference_structure_invalid")
        return None, None, None, 0
    pairs = tuple(zip(knots, knots[1:], strict=False))
    if not pairs:
        pairs = ((knots[0], knots[0]),)
    for pair_index, (left, right) in enumerate(pairs):
        try:
            samples, gap_bound = _swept_samples(
                left.pose,
                right.pose,
                half_diagonal,
            )
            if pair_index:
                samples = samples[1:]
            for sample in samples:
                sample_count += 1
                physical_clearance = physical.clearance(sample, limit_m=1.0) - gap_bound
                forbidden_clearance = forbidden.forbidden_clearance(sample, limit_m=1.0) - gap_bound
                allowed_clearance = (
                    1.0 if allowed is None else allowed.clearance(sample, limit_m=1.0) - gap_bound
                )
                minimum_physical = _optional_min(minimum_physical, physical_clearance)
                minimum_forbidden = _optional_min(minimum_forbidden, forbidden_clearance)
                minimum_allowed = _optional_min(minimum_allowed, allowed_clearance)
                if physical_clearance + R4_COMPARISON_TOLERANCE < R4_MINIMUM_CLEARANCE_M:
                    failures.add("physical_clearance_violation")
                if forbidden_clearance + R4_COMPARISON_TOLERANCE < R4_MINIMUM_CLEARANCE_M:
                    failures.add("forbidden_clearance_violation")
                if allowed_clearance + R4_COMPARISON_TOLERANCE < R4_MINIMUM_CLEARANCE_M:
                    failures.add("allowed_boundary_clearance_violation")
        except (AttributeError, TypeError, ValueError, OverflowError):
            failures.add("independent_geometry_validation_failed")
    return minimum_physical, minimum_forbidden, minimum_allowed, sample_count


def _swept_samples(
    start: Pose2D,
    end: Pose2D,
    half_diagonal: float,
) -> tuple[tuple[Pose2D, ...], float]:
    distance = _pose_distance(start, end)
    yaw_delta = _angle_delta(end.yaw, start.yaw)
    interval_count = max(
        1,
        ceil(distance / R4_TRANSLATION_SWEEP_STEP_M),
        ceil(abs(yaw_delta) / R4_ROTATION_SWEEP_STEP_RAD),
    )
    samples = tuple(
        Pose2D(
            x=start.x + (end.x - start.x) * index / interval_count,
            y=start.y + (end.y - start.y) * index / interval_count,
            yaw=_normalize_angle(start.yaw + yaw_delta * index / interval_count),
        )
        for index in range(interval_count + 1)
    )
    gap_bound = 0.5 * (distance / interval_count + half_diagonal * abs(yaw_delta) / interval_count)
    return samples, gap_bound


def _complement_grid(grid: GridMap, cells: tuple[tuple[int, int], ...]) -> GridMap:
    occupancy = np.ones((grid.height, grid.width), dtype=np.bool_)
    for x, y in cells:
        if grid.in_bounds((x, y)):
            occupancy[y, x] = False
    return GridMap(
        occupancy,
        resolution_m=grid.resolution_m,
        origin_x_m=grid.origin_x_m,
        origin_y_m=grid.origin_y_m,
    )


def _optional_min(current: float | None, candidate: float) -> float:
    return candidate if current is None else min(current, candidate)


def _minimum_present(*values: float | None) -> float | None:
    present = tuple(value for value in values if value is not None)
    return min(present) if present else None


def _pose_distance(left: Pose2D, right: Pose2D) -> float:
    return hypot(right.x - left.x, right.y - left.y)


def _poses_close(left: Pose2D, right: Pose2D) -> bool:
    return (
        _pose_distance(left, right) <= R4_COMPARISON_TOLERANCE
        and abs(_angle_delta(left.yaw, right.yaw)) <= R4_COMPARISON_TOLERANCE
    )


def _angle_delta(left: float, right: float) -> float:
    return atan2(sin(left - right), cos(left - right))


def _normalize_angle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def _require_sha256(value: str | None, field_name: str) -> None:
    if not isinstance(value, str) or fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{field_name} must be a lowercase SHA-256 hex digest")


def _safe_evidence_hash(value: object, fallback_label: str) -> str:
    if isinstance(value, str) and fullmatch(r"[0-9a-f]{64}", value) is not None:
        return value
    return canonical_content_hash({"invalid_evidence": fallback_label, "value": repr(value)})


__all__ = [
    "LOCAL_REFERENCE_VALIDATION_SCHEMA_VERSION",
    "LOCAL_REFERENCE_VALIDATOR_VERSION",
    "LocalReferenceValidation",
    "validate_local_maneuver_reference",
]
