"""R5-B causal witness를 GROUND_TRUTH_TEMPORAL R4 reference로 변환한다.

이 모듈은 R2 archive에서 검증·파생된 path-only evidence만 사용한다. Actor
ground truth는 reference의 시간 증거를 검증하는 데만 쓰며 controller 입력으로
전달하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, sin
from pathlib import Path

from hospital_path_lab.contracts import GridSnapshot, Pose2D, SnapshotMetadata
from hospital_path_lab.dynamic_witness_contracts import (
    PassSide,
    WitnessPhase,
    WitnessPoint,
)
from hospital_path_lab.local_reference_contracts import (
    LOCAL_REFERENCE_CONTRACT_VERSION,
    LOCAL_REFERENCE_SCHEMA_VERSION,
    REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
    TEMPORAL_REFERENCE_EVIDENCE_SCHEMA_VERSION,
    TEMPORAL_REFERENCE_GEOMETRY_SCHEMA_VERSION,
    LocalManeuverKind,
    LocalManeuverReference,
    ObservationDependency,
    ReferenceBuildContext,
    ReferenceEvidenceLevel,
    ReferenceKnot,
    ReferenceKnotRole,
    ReferenceSection,
    ReferenceSectionKind,
    ReferenceTravelDirection,
    ReferenceValidity,
    TemporalReferenceEvidence,
    TemporalReferenceGeometryEvidence,
)
from hospital_path_lab.local_reference_validation import (
    LocalReferenceValidation,
    validate_local_maneuver_reference,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r5b_temporal_evidence import (
    CausalR5BPassEvidence,
    build_causal_r5b_pass_evidence,
)
from hospital_path_lab.spatial_oracle_contracts import (
    SpatialAllowedRegion,
    spatial_grid_content_hash,
)

R5B_TEMPORAL_REFERENCE_BUNDLE_SCHEMA_VERSION = "r5b-temporal-reference-bundle-v1"
R5B_TEMPORAL_REFERENCE_BUILDER_VERSION = "r5b-temporal-reference-builder-v1"
R5B_REFERENCE_MISSION_ID = "r5b-path-only-public-mission"
R5B_REFERENCE_STOP_EPOCH = 1
R5B_REFERENCE_MANEUVER_REVISION = 1
R5B_REFERENCE_PATH_REVISION = 1
_TOLERANCE = 1e-9


@dataclass(frozen=True, slots=True)
class R5BTemporalReferenceBundle:
    schema_version: str
    source: CausalR5BPassEvidence
    build_context: ReferenceBuildContext
    temporal_evidence: TemporalReferenceEvidence
    temporal_geometry: TemporalReferenceGeometryEvidence
    reference: LocalManeuverReference
    validation: LocalReferenceValidation
    bundle_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != R5B_TEMPORAL_REFERENCE_BUNDLE_SCHEMA_VERSION:
            raise ValueError("unsupported R5-B temporal reference bundle schema")
        if not self.validation.passed:
            raise ValueError("R5-B temporal reference bundle requires a passing validation")
        if self.validation.reference_content_hash != self.reference.reference_content_hash:
            raise ValueError("R5-B temporal reference validation provenance mismatch")
        expected = self.expected_content_hash
        if self.bundle_content_hash and self.bundle_content_hash != expected:
            raise ValueError("R5-B temporal reference bundle hash mismatch")
        object.__setattr__(self, "bundle_content_hash", expected)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "source_evidence_hash": self.source.evidence_content_hash,
                "build_context_hash": self.build_context.context_content_hash,
                "temporal_evidence_hash": self.temporal_evidence.evidence_content_hash,
                "temporal_geometry_hash": self.temporal_geometry.geometry_content_hash,
                "reference_hash": self.reference.reference_content_hash,
                "validation_hash": self.validation.validation_content_hash,
            }
        )


def build_r5b_temporal_reference_bundles(
    archive_path: Path,
) -> tuple[R5BTemporalReferenceBundle, ...]:
    sources = build_causal_r5b_pass_evidence(Path(archive_path))
    bundles = tuple(_build_bundle(source) for source in sources)
    if len(bundles) != 10 or len({item.bundle_content_hash for item in bundles}) != 10:
        raise RuntimeError("R5-B temporal reference build did not produce ten unique bundles")
    return bundles


def _build_bundle(source: CausalR5BPassEvidence) -> R5BTemporalReferenceBundle:
    context = _build_context(source)
    kind = (
        LocalManeuverKind.PASS_LEFT
        if source.side is PassSide.LEFT
        else LocalManeuverKind.PASS_RIGHT
    )
    knots, sections, departure_knot, pass_section = _convert_witness_geometry(source)
    minimum_clearance = min(
        source.validation.metrics.minimum_static_clearance_m,
        source.validation.metrics.minimum_forbidden_clearance_m,
    )
    # R4 독립 validator가 재현할 수 있는 동결 하한만 계약에 싣는다.
    minimum_clearance = max(0.08, min(minimum_clearance, 0.08))
    actor_ids = tuple(source.witness.required_pass_actor_ids)
    departure_progress = _arc_at_time(
        source.witness.points,
        source.witness.departure_time_s,
        release_tick=source.release_tick,
    )
    pass_progress = _arc_at_time(
        source.witness.points,
        source.witness.pass_times_by_actor[0][1],
        release_tick=source.release_tick,
    )
    rejoin_progress = _arc_at_time(
        source.witness.points,
        source.witness.rejoin_confirmed_at_s,
        release_tick=source.release_tick,
    )
    temporal_evidence = TemporalReferenceEvidence(
        schema_version=TEMPORAL_REFERENCE_EVIDENCE_SCHEMA_VERSION,
        source_witness_hash=source.witness.semantic_content_hash,
        source_validation_hash=source.validation.content_hash,
        maneuver_kind=kind,
        target_actor_binding_ids=actor_ids,
        departure_progress_m=departure_progress,
        pass_progress_m=pass_progress,
        rejoin_progress_m=rejoin_progress,
        ground_truth_only=True,
        limitations=(
            "ideal_causal_stream_only",
            "path_only_not_perception_evidence",
            "public_simulation_only",
        ),
    )
    geometry_hash = canonical_content_hash(
        {
            "maneuver_kind": kind,
            "knots": knots,
            "sections": sections,
            "departure_knot_index": departure_knot,
            "pass_section_index": pass_section,
            "rejoin_knot_index": len(knots) - 1,
            "minimum_validated_static_clearance_m": minimum_clearance,
        }
    )
    temporal_geometry = TemporalReferenceGeometryEvidence(
        schema_version=TEMPORAL_REFERENCE_GEOMETRY_SCHEMA_VERSION,
        source_causal_evidence_hash=source.evidence_content_hash,
        source_witness_hash=source.witness.semantic_content_hash,
        source_validation_hash=source.validation.content_hash,
        world_content_hash=source.witness.world_content_hash,
        grid_content_hash=context.grid_content_hash,
        maneuver_kind=kind,
        target_actor_binding_ids=actor_ids,
        reference_geometry_hash=geometry_hash,
        minimum_static_clearance_m=minimum_clearance,
        limitations=(
            "geometry_derived_from_causal_ground_truth_witness",
            "no_controller_command_replay",
            "public_simulation_only",
        ),
    )
    identity = {
        "builder_version": R5B_TEMPORAL_REFERENCE_BUILDER_VERSION,
        "source_causal_evidence_hash": source.evidence_content_hash,
        "temporal_evidence_hash": temporal_evidence.evidence_content_hash,
        "temporal_geometry_hash": temporal_geometry.geometry_content_hash,
        "build_context_hash": context.context_content_hash,
        "maneuver_revision": R5B_REFERENCE_MANEUVER_REVISION,
        "path_revision": R5B_REFERENCE_PATH_REVISION,
    }
    reference = LocalManeuverReference(
        schema_version=LOCAL_REFERENCE_SCHEMA_VERSION,
        reference_contract_version=LOCAL_REFERENCE_CONTRACT_VERSION,
        candidate_id=canonical_content_hash({"r5b_candidate": identity}),
        maneuver_kind=kind,
        evidence_level=ReferenceEvidenceLevel.GROUND_TRUTH_TEMPORAL,
        mission_id=context.mission_id,
        stop_epoch=context.stop_epoch,
        map_id=context.map_id,
        map_revision=context.map_revision,
        mission_revision=context.mission_revision,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        maneuver_revision=R5B_REFERENCE_MANEUVER_REVISION,
        path_revision=R5B_REFERENCE_PATH_REVISION,
        reference_session_id=canonical_content_hash({"r5b_reference_session": identity}),
        source_spatial_seed_hash=None,
        source_temporal_evidence_hash=temporal_evidence.evidence_content_hash,
        original_reference_hash=context.original_reference_hash,
        grid_content_hash=context.grid_content_hash,
        vehicle_profile_hash=context.vehicle_profile_hash,
        allowed_region_hash=context.allowed_region_hash,
        forbidden_region_hash=context.forbidden_region_hash,
        knots=knots,
        sections=sections,
        departure_knot_index=departure_knot,
        pass_section_index=pass_section,
        rejoin_knot_index=len(knots) - 1,
        minimum_validated_static_clearance_m=minimum_clearance,
        validity=ReferenceValidity(
            required_mission_id=context.mission_id,
            required_stop_epoch=context.stop_epoch,
            required_map_revision=context.map_revision,
            required_mission_revision=context.mission_revision,
            required_observation_revision=None,
            valid_from_control_tick=0,
            valid_until_control_tick=None,
        ),
        generation_reason_codes=("validated_r5b_causal_temporal_witness",),
        limitations=(
            "ground_truth_temporal_path_only",
            "ideal_causal_stream_only",
            "no_perception_claim",
            "public_simulation_only",
        ),
        source_temporal_geometry_hash=temporal_geometry.geometry_content_hash,
    )
    validation = validate_local_maneuver_reference(
        context,
        reference,
        temporal_evidence=temporal_evidence,
        temporal_geometry=temporal_geometry,
    )
    return R5BTemporalReferenceBundle(
        schema_version=R5B_TEMPORAL_REFERENCE_BUNDLE_SCHEMA_VERSION,
        source=source,
        build_context=context,
        temporal_evidence=temporal_evidence,
        temporal_geometry=temporal_geometry,
        reference=reference,
        validation=validation,
    )


def _build_context(source: CausalR5BPassEvidence) -> ReferenceBuildContext:
    world = source.world
    grid = world.grid.to_grid_map()
    forbidden = tuple(sorted(world.grid.forbidden_cells))
    snapshot = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id=world.map_id,
            map_revision=world.map_revision,
            mission_revision=0,
            observation_revision=0,
            seed=world.seed,
            content_hash=world.grid_content_hash,
            input_valid=True,
        ),
        grid=grid,
        forbidden_cells=frozenset(forbidden),
    )
    allowed = SpatialAllowedRegion()
    return ReferenceBuildContext(
        schema_version=REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
        mission_id=R5B_REFERENCE_MISSION_ID,
        stop_epoch=R5B_REFERENCE_STOP_EPOCH,
        map_id=world.map_id,
        map_revision=world.map_revision,
        mission_revision=0,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        static_grid_snapshot=snapshot,
        grid_content_hash=spatial_grid_content_hash(grid),
        allowed_region=allowed,
        allowed_region_hash=allowed.content_hash,
        forbidden_cells=forbidden,
        forbidden_region_hash=canonical_content_hash(forbidden),
        vehicle_profile=world.kinematic_contract.vehicle_profile,
        vehicle_profile_hash=canonical_content_hash(
            world.kinematic_contract.vehicle_profile
        ),
        original_reference=world.reference_path,
        original_reference_hash=canonical_content_hash(world.reference_path),
        current_robot_pose=world.initial_state.pose,
        control_tick=0,
        simulation_time_s=0.0,
    )


def _convert_witness_geometry(
    source: CausalR5BPassEvidence,
) -> tuple[tuple[ReferenceKnot, ...], tuple[ReferenceSection, ...], int, int]:
    points = tuple(source.witness.points)
    release_index = source.release_tick
    end_index = next(
        index
        for index in range(len(points) - 1, release_index, -1)
        if points[index].phase is not WitnessPhase.TERMINAL_DWELL
    )
    groups: list[tuple[str, int, int]] = []
    group_start = release_index
    group_id = points[release_index + 1].source_primitive_id
    for right_index in range(release_index + 1, end_index + 1):
        current_id = points[right_index].source_primitive_id
        if current_id != group_id:
            groups.append((group_id, group_start, right_index - 1))
            group_start = right_index - 1
            group_id = current_id
    groups.append((group_id, group_start, end_index))

    section_specs: list[tuple[ReferenceSectionKind, int, int, ReferenceTravelDirection]] = [
        (ReferenceSectionKind.DEPART, release_index, release_index, ReferenceTravelDirection.NONE)
    ]
    for primitive_id, first, last in groups:
        if last <= first:
            continue
        motion = _group_motion(points[first : last + 1])
        if motion == "rotation":
            section_specs.append(
                (ReferenceSectionKind.ROTATE, first, last, ReferenceTravelDirection.NONE)
            )
        elif motion == "translation":
            section_specs.append(
                (_stage_kind(primitive_id), first, last, ReferenceTravelDirection.FORWARD)
            )
        elif motion != "stationary":
            raise ValueError("R5-B witness contains an unsupported combined motion group")
    section_specs.append(
        (ReferenceSectionKind.REJOIN, end_index, end_index, ReferenceTravelDirection.NONE)
    )

    knots: list[ReferenceKnot] = []
    sections: list[ReferenceSection] = []
    cumulative_arc = 0.0
    previous_pose: Pose2D | None = None
    for section_index, (kind, first, last, direction) in enumerate(section_specs):
        first_knot = len(knots)
        for source_index in range(first, last + 1):
            point = points[source_index]
            if previous_pose is not None:
                cumulative_arc += hypot(
                    point.pose.x - previous_pose.x,
                    point.pose.y - previous_pose.y,
                )
            roles = {ReferenceKnotRole.ANCHOR}
            if direction is not ReferenceTravelDirection.NONE:
                roles.add(ReferenceKnotRole.TRANSLATION)
            if kind is ReferenceSectionKind.ROTATE:
                if source_index == first:
                    roles.update(
                        (ReferenceKnotRole.ROTATION_ENTRY, ReferenceKnotRole.STOP_MARKER)
                    )
                if source_index == last:
                    roles.update(
                        (ReferenceKnotRole.ROTATION_EXIT, ReferenceKnotRole.STOP_MARKER)
                    )
            if source_index in (first, last):
                roles.add(ReferenceKnotRole.STOP_MARKER)
            if section_index == len(section_specs) - 1:
                roles.update((ReferenceKnotRole.REJOIN, ReferenceKnotRole.STOP_MARKER))
            knots.append(
                ReferenceKnot(
                    knot_index=len(knots),
                    pose=point.pose,
                    tangent_yaw=point.pose.yaw,
                    cumulative_translation_arc_m=cumulative_arc,
                    source_path_index=source_index,
                    section_index=section_index,
                    knot_roles=tuple(roles),
                )
            )
            previous_pose = point.pose
        last_knot = len(knots) - 1
        stopped = kind is ReferenceSectionKind.ROTATE
        sections.append(
            ReferenceSection(
                section_index=section_index,
                section_kind=kind,
                travel_direction=direction,
                first_knot_index=first_knot,
                last_knot_index=last_knot,
                entry_requires_stopped=stopped,
                exit_requires_stopped=stopped,
                source_primitive_indices=(),
            )
        )
    kinds = tuple(section.section_kind for section in sections)
    pass_section = kinds.index(ReferenceSectionKind.BYPASS)
    return tuple(knots), tuple(sections), 0, pass_section


def _group_motion(points: tuple[WitnessPoint, ...]) -> str:
    translation = False
    rotation = False
    for left, right in zip(points, points[1:], strict=False):
        translation |= hypot(right.pose.x - left.pose.x, right.pose.y - left.pose.y) > _TOLERANCE
        rotation |= abs(_angle_delta(right.pose.yaw, left.pose.yaw)) > _TOLERANCE
    if translation and rotation:
        return "combined"
    if translation:
        return "translation"
    if rotation:
        return "rotation"
    return "stationary"


def _stage_kind(primitive_id: str) -> ReferenceSectionKind:
    if primitive_id == "move_lateral":
        return ReferenceSectionKind.DEPART
    if primitive_id == "move_past":
        return ReferenceSectionKind.BYPASS
    if primitive_id == "move_to_reference":
        return ReferenceSectionKind.RETURN
    raise ValueError(f"R5-B witness translation primitive is unsupported: {primitive_id}")


def _arc_at_time(
    points: tuple[WitnessPoint, ...],
    event_time_s: float | None,
    *,
    release_tick: int,
) -> float:
    if event_time_s is None:
        raise ValueError("R5-B PASS event time is missing")
    arc = 0.0
    for left, right in zip(points[release_tick:], points[release_tick + 1 :], strict=False):
        distance = hypot(right.pose.x - left.pose.x, right.pose.y - left.pose.y)
        if event_time_s <= right.time_s + _TOLERANCE:
            duration = right.time_s - left.time_s
            fraction = 0.0 if duration <= 0.0 else (event_time_s - left.time_s) / duration
            return arc + distance * min(1.0, max(0.0, fraction))
        arc += distance
    return arc


def _angle_delta(left: float, right: float) -> float:
    return atan2(sin(left - right), cos(left - right))


__all__ = [
    "R5B_TEMPORAL_REFERENCE_BUNDLE_SCHEMA_VERSION",
    "R5B_TEMPORAL_REFERENCE_BUILDER_VERSION",
    "R5BTemporalReferenceBundle",
    "build_r5b_temporal_reference_bundles",
]
