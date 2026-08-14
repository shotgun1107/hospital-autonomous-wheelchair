"""R3의 검증된 정적 경로를 R4 immutable 지역 reference로 변환한다.

이 모듈은 Actor, corpus label, evaluator oracle, controller와 hidden 정보를 읽지 않는다.
생성 결과는 ``SPATIAL_ONLY`` 연구 reference이며 이동 허가가 아니다.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import atan2, cos, hypot, isfinite, sin
from time import perf_counter_ns

from hospital_path_lab.local_reference_contracts import (
    LOCAL_REFERENCE_CONTRACT_VERSION,
    LOCAL_REFERENCE_SCHEMA_VERSION,
    LOCAL_REFERENCE_SET_SCHEMA_VERSION,
    SPATIAL_REFERENCE_SEED_SCHEMA_VERSION,
    LocalManeuverKind,
    LocalManeuverReference,
    LocalManeuverReferenceSet,
    ObservationDependency,
    ReferenceBuildContext,
    ReferenceBuildStatus,
    ReferenceEvidenceLevel,
    ReferenceKnot,
    ReferenceKnotRole,
    ReferenceSection,
    ReferenceSectionKind,
    ReferenceSourceRejection,
    ReferenceTravelDirection,
    ReferenceValidity,
    SpatialReferenceSeed,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.spatial_oracle_contracts import (
    BoundedSpatialOracleRequest,
    BoundedSpatialOracleResult,
    ManeuverSide,
    SpatialOracleStatus,
    SpatialPrimitive,
    SpatialPrimitiveKind,
    spatial_path_content_hash,
)

LOCAL_REFERENCE_BUILDER_VERSION = "local-reference-builder-v2"
_TOLERANCE = 1e-9
_ROTATION_KINDS = frozenset(
    (SpatialPrimitiveKind.ROTATE_LEFT_45, SpatialPrimitiveKind.ROTATE_RIGHT_45)
)
_NORMAL_UNSUPPORTED_CODES = frozenset(
    ("multi_segment_projection_unsupported", "unspecified_side_not_supported")
)


class LocalReferenceSourceError(ValueError):
    """하나의 source가 R4 변환 자격을 만족하지 못했음을 stable code로 보존한다."""

    def __init__(self, reason_code: str) -> None:
        super().__init__(reason_code)
        self.reason_code = reason_code


@dataclass(frozen=True, slots=True)
class SpatialReferenceSource:
    request: BoundedSpatialOracleRequest
    result: BoundedSpatialOracleResult

    def __post_init__(self) -> None:
        if not isinstance(self.request, BoundedSpatialOracleRequest):
            raise TypeError("request must be a BoundedSpatialOracleRequest")
        if not isinstance(self.result, BoundedSpatialOracleResult):
            raise TypeError("result must be a BoundedSpatialOracleResult")

    @property
    def source_content_hash(self) -> str:
        return self.result.semantic_content_hash


def project_validated_spatial_seed(
    context: ReferenceBuildContext,
    source: SpatialReferenceSource,
) -> SpatialReferenceSeed:
    """검증된 feasible R3 source만 hash-bound R4 seed로 투영한다."""

    if not isinstance(context, ReferenceBuildContext):
        raise TypeError("context must be a ReferenceBuildContext")
    if not isinstance(source, SpatialReferenceSource):
        raise TypeError("source must be a SpatialReferenceSource")
    request = source.request
    result = source.result

    if context.observation_dependency is not ObservationDependency.STATIC_ONLY:
        raise LocalReferenceSourceError("observation_dependency_mismatch")
    integrity_failure = request.integrity_failure()
    if integrity_failure is not None:
        raise LocalReferenceSourceError(integrity_failure)
    if request.request_content_hash != request.expected_content_hash:
        raise LocalReferenceSourceError("source_hash_mismatch")
    if result.semantic_content_hash != result.expected_semantic_hash:
        raise LocalReferenceSourceError("source_hash_mismatch")
    if result.status is not SpatialOracleStatus.SPATIALLY_FEASIBLE:
        raise LocalReferenceSourceError("source_status_not_feasible")
    validation = result.validation
    if validation is None:
        raise LocalReferenceSourceError("source_validation_missing")
    if validation.validation_content_hash != validation.expected_content_hash:
        raise LocalReferenceSourceError("source_hash_mismatch")
    if not validation.passed:
        raise LocalReferenceSourceError("source_validation_failed")
    if result.request_content_hash != request.request_content_hash:
        raise LocalReferenceSourceError("source_hash_mismatch")
    if validation.request_content_hash != request.request_content_hash:
        raise LocalReferenceSourceError("source_hash_mismatch")
    expected_path_hash = spatial_path_content_hash(result.path, result.primitive_sequence)
    if validation.path_content_hash != expected_path_hash:
        raise LocalReferenceSourceError("source_hash_mismatch")
    if len(result.primitive_sequence) != len(result.path) - 1:
        raise LocalReferenceSourceError("path_primitive_length_mismatch")
    for index, primitive in enumerate(result.primitive_sequence):
        if (
            primitive.start_pose != result.path[index]
            or primitive.end_pose != result.path[index + 1]
        ):
            raise LocalReferenceSourceError("path_primitive_endpoint_mismatch")

    if (
        result.map_id != request.map_id
        or result.map_revision != request.map_revision
        or result.mission_revision != request.mission_revision
        or result.grid_content_hash != request.grid_content_hash
        or result.vehicle_profile_hash != request.vehicle_profile_hash
        or result.search_region_hash != request.search_region.content_hash
        or result.lattice_config_hash != request.lattice_config.content_hash
    ):
        raise LocalReferenceSourceError("source_result_provenance_mismatch")

    _require_context_matches_request(context, request)
    if request.maneuver_side is ManeuverSide.UNSPECIFIED:
        raise LocalReferenceSourceError("unspecified_side_not_supported")
    if result.minimum_clearance_m is None or not isfinite(result.minimum_clearance_m):
        raise LocalReferenceSourceError("source_validation_failed")

    return SpatialReferenceSeed(
        schema_version=SPATIAL_REFERENCE_SEED_SCHEMA_VERSION,
        source_spatial_result_hash=result.semantic_content_hash,
        source_spatial_request_hash=request.request_content_hash,
        source_validation_hash=validation.validation_content_hash,
        map_id=result.map_id,
        map_revision=result.map_revision,
        mission_revision=result.mission_revision,
        grid_content_hash=result.grid_content_hash,
        vehicle_profile_hash=result.vehicle_profile_hash,
        side=request.maneuver_side,
        start_pose=request.start_pose,
        rejoin_goal=request.rejoin_goal,
        pose_heading_path=result.path,
        primitive_sequence=result.primitive_sequence,
        minimum_clearance_m=result.minimum_clearance_m,
        limitations=result.limitations,
    )


def build_spatial_local_reference(
    context: ReferenceBuildContext,
    seed: SpatialReferenceSeed,
    *,
    maneuver_revision: int,
    path_revision: int,
) -> LocalManeuverReference:
    """R3 seed를 회전과 section 경계를 보존한 SPATIAL_ONLY reference로 바꾼다."""

    if not isinstance(context, ReferenceBuildContext):
        raise TypeError("context must be a ReferenceBuildContext")
    if not isinstance(seed, SpatialReferenceSeed):
        raise TypeError("seed must be a SpatialReferenceSeed")
    _require_exact_nonnegative_int(maneuver_revision, "maneuver_revision")
    _require_exact_nonnegative_int(path_revision, "path_revision")
    _require_seed_matches_context(context, seed)

    kind = _kind_for_side(seed.side)
    knots, sections, pass_section_index, conversion_limitations = _canonical_geometry(
        context,
        seed,
    )
    identity_payload = {
        "builder_version": LOCAL_REFERENCE_BUILDER_VERSION,
        "build_context_hash": context.context_content_hash,
        "spatial_seed_hash": seed.seed_content_hash,
        "maneuver_kind": kind,
        "maneuver_revision": maneuver_revision,
        "path_revision": path_revision,
    }
    candidate_id = canonical_content_hash({"candidate": identity_payload})
    session_id = canonical_content_hash({"reference_session": identity_payload})
    limitations = tuple(
        sorted(
            set(
                (
                    *seed.limitations,
                    *conversion_limitations,
                    "spatial_only_no_ordered_overtake_claim",
                )
            )
        )
    )
    if any(
        primitive.kind is SpatialPrimitiveKind.REVERSE_ONE_TRANSLATION
        for primitive in seed.primitive_sequence
    ):
        limitations = tuple(sorted((*limitations, "reverse_primitive_simulation_only")))

    validity = ReferenceValidity(
        required_mission_id=context.mission_id,
        required_stop_epoch=context.stop_epoch,
        required_map_revision=context.map_revision,
        required_mission_revision=context.mission_revision,
        required_observation_revision=None,
        valid_from_control_tick=context.control_tick,
        valid_until_control_tick=None,
    )
    return LocalManeuverReference(
        schema_version=LOCAL_REFERENCE_SCHEMA_VERSION,
        reference_contract_version=LOCAL_REFERENCE_CONTRACT_VERSION,
        candidate_id=candidate_id,
        maneuver_kind=kind,
        evidence_level=ReferenceEvidenceLevel.SPATIAL_ONLY,
        mission_id=context.mission_id,
        stop_epoch=context.stop_epoch,
        map_id=context.map_id,
        map_revision=context.map_revision,
        mission_revision=context.mission_revision,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        maneuver_revision=maneuver_revision,
        path_revision=path_revision,
        reference_session_id=session_id,
        source_spatial_seed_hash=seed.seed_content_hash,
        source_temporal_evidence_hash=None,
        original_reference_hash=context.original_reference_hash,
        grid_content_hash=context.grid_content_hash,
        vehicle_profile_hash=context.vehicle_profile_hash,
        allowed_region_hash=context.allowed_region_hash,
        forbidden_region_hash=context.forbidden_region_hash,
        knots=knots,
        sections=sections,
        departure_knot_index=0,
        pass_section_index=pass_section_index,
        rejoin_knot_index=len(knots) - 1,
        minimum_validated_static_clearance_m=seed.minimum_clearance_m,
        validity=validity,
        generation_reason_codes=("validated_spatial_seed",),
        limitations=limitations,
    )


def build_spatial_reference_set(
    context: ReferenceBuildContext,
    sources: tuple[SpatialReferenceSource, ...],
    *,
    maneuver_revision: int,
    path_revision: int,
    elapsed_nonqualification_ns: int | None = None,
) -> LocalManeuverReferenceSet:
    """한 generation의 R3 source를 candidate set으로 결정론적으로 변환한다."""

    if not isinstance(context, ReferenceBuildContext):
        raise TypeError("context must be a ReferenceBuildContext")
    _require_exact_nonnegative_int(maneuver_revision, "maneuver_revision")
    _require_exact_nonnegative_int(path_revision, "path_revision")
    source_tuple = tuple(sources)
    if any(not isinstance(source, SpatialReferenceSource) for source in source_tuple):
        raise TypeError("sources must contain SpatialReferenceSource values")
    if len({source.source_content_hash for source in source_tuple}) != len(source_tuple):
        raise ValueError("spatial sources must be unique")
    started_ns = perf_counter_ns()

    candidates: list[LocalManeuverReference] = []
    rejections: list[ReferenceSourceRejection] = []
    limitations: set[str] = {"simulation_only", "r4_builder_spatial_only"}
    hard_invalid = False
    resource_inconclusive = False
    for source in sorted(source_tuple, key=lambda item: item.source_content_hash):
        result = source.result
        limitations.update(result.limitations)
        if result.status is SpatialOracleStatus.RESOURCE_LIMIT:
            resource_inconclusive = True
            limitations.add("search_resource_limit_passthrough")
            rejections.append(
                ReferenceSourceRejection(
                    source.source_content_hash,
                    ("source_status_resource_limit",),
                )
            )
            continue
        if result.status is SpatialOracleStatus.INVALID_INPUT:
            hard_invalid = True
            rejections.append(
                ReferenceSourceRejection(
                    source.source_content_hash,
                    ("source_status_invalid_input",),
                )
            )
            continue
        if result.status is SpatialOracleStatus.SPATIALLY_INFEASIBLE:
            rejections.append(
                ReferenceSourceRejection(
                    source.source_content_hash,
                    ("source_status_spatially_infeasible",),
                )
            )
            continue
        try:
            seed = project_validated_spatial_seed(context, source)
            candidates.append(
                build_spatial_local_reference(
                    context,
                    seed,
                    maneuver_revision=maneuver_revision,
                    path_revision=path_revision,
                )
            )
        except LocalReferenceSourceError as error:
            if error.reason_code in _NORMAL_UNSUPPORTED_CODES:
                limitations.add(error.reason_code)
            else:
                hard_invalid = True
            rejections.append(
                ReferenceSourceRejection(
                    source.source_content_hash,
                    (error.reason_code,),
                )
            )

    if hard_invalid:
        status = ReferenceBuildStatus.INVALID_INPUT
        reason = "invalid_spatial_source"
        candidates = []
    elif resource_inconclusive:
        status = ReferenceBuildStatus.SEARCH_INCONCLUSIVE
        reason = "spatial_source_resource_limit"
        candidates = []
    elif candidates:
        status = ReferenceBuildStatus.REFERENCE_SET_READY
        reason = "validated_spatial_candidates_built"
    else:
        status = ReferenceBuildStatus.NO_REFERENCE
        reason = "no_spatial_candidate"

    elapsed = (
        perf_counter_ns() - started_ns
        if elapsed_nonqualification_ns is None
        else elapsed_nonqualification_ns
    )
    _require_exact_nonnegative_int(elapsed, "elapsed_nonqualification_ns")
    return LocalManeuverReferenceSet(
        schema_version=LOCAL_REFERENCE_SET_SCHEMA_VERSION,
        status=status,
        termination_reason=reason,
        build_context_hash=context.context_content_hash,
        maneuver_revision=maneuver_revision,
        candidates=tuple(candidates),
        upper_dispositions=(),
        rejected_sources=tuple(rejections),
        limitations=tuple(sorted(limitations)),
        elapsed_nonqualification_ns=elapsed,
    )


def _canonical_geometry(
    context: ReferenceBuildContext,
    seed: SpatialReferenceSeed,
) -> tuple[
    tuple[ReferenceKnot, ...],
    tuple[ReferenceSection, ...],
    int,
    tuple[str, ...],
]:
    if len(context.original_reference) != 2:
        raise LocalReferenceSourceError("multi_segment_projection_unsupported")
    start, end = context.original_reference
    tangent_x = end.x - start.x
    tangent_y = end.y - start.y
    length = hypot(tangent_x, tangent_y)
    if length <= _TOLERANCE:
        raise LocalReferenceSourceError("reference_structure_invalid")
    tangent_x /= length
    tangent_y /= length
    offsets = tuple(
        tangent_x * (pose.y - start.y) - tangent_y * (pose.x - start.x)
        for pose in seed.pose_heading_path
    )
    if seed.side is ManeuverSide.LEFT:
        maximum_index = max(range(len(offsets)), key=offsets.__getitem__)
        excursion = offsets[maximum_index]
        if min(offsets) < -_TOLERANCE:
            raise LocalReferenceSourceError("opposite_side_excursion")
    else:
        maximum_index = min(range(len(offsets)), key=offsets.__getitem__)
        excursion = -offsets[maximum_index]
        if max(offsets) > _TOLERANCE:
            raise LocalReferenceSourceError("opposite_side_excursion")
    if excursion + _TOLERANCE < seed.rejoin_goal.minimum_side_excursion_m:
        raise LocalReferenceSourceError("required_side_excursion_missing")
    if maximum_index in (0, len(seed.pose_heading_path) - 1):
        raise LocalReferenceSourceError("reference_structure_invalid")

    builder = _GeometryBuilder(seed)
    pre_indices = tuple(range(0, maximum_index))
    post_indices = tuple(range(maximum_index, len(seed.primitive_sequence)))
    if not pre_indices or _primitive_is_rotation(seed.primitive_sequence[pre_indices[0]]):
        builder.add_anchor(0, ReferenceSectionKind.DEPART)
    builder.add_stage(pre_indices, ReferenceSectionKind.DEPART)

    pass_section_index = builder.add_anchor(maximum_index, ReferenceSectionKind.BYPASS)
    final_index = len(seed.primitive_sequence) - 1
    return_indices = tuple(index for index in post_indices if index < final_index)
    builder.add_stage(return_indices, ReferenceSectionKind.RETURN)
    if not any(
        section.section_kind is ReferenceSectionKind.RETURN
        for section in builder.sections[pass_section_index + 1 :]
    ):
        builder.add_anchor(maximum_index, ReferenceSectionKind.RETURN)

    final_primitive = seed.primitive_sequence[final_index]
    if _primitive_is_rotation(final_primitive):
        builder.add_stage((final_index,), ReferenceSectionKind.RETURN)
        builder.add_anchor(len(seed.pose_heading_path) - 1, ReferenceSectionKind.REJOIN)
    else:
        builder.add_group((final_index,), ReferenceSectionKind.REJOIN)
    knots, sections = builder.finish()
    pass_section_index = next(
        section.section_index
        for section in sections
        if section.section_kind is ReferenceSectionKind.BYPASS
    )
    return knots, sections, pass_section_index, ("spatial_bypass_anchor_only",)


class _GeometryBuilder:
    def __init__(self, seed: SpatialReferenceSeed) -> None:
        self.seed = seed
        self.knots: list[ReferenceKnot] = []
        self.sections: list[ReferenceSection] = []
        self.arc_m = 0.0

    def add_stage(
        self,
        primitive_indices: tuple[int, ...],
        movement_kind: ReferenceSectionKind,
    ) -> None:
        group: list[int] = []
        current_key: tuple[ReferenceSectionKind, ReferenceTravelDirection] | None = None
        for primitive_index in primitive_indices:
            primitive = self.seed.primitive_sequence[primitive_index]
            kind = (
                ReferenceSectionKind.ROTATE if _primitive_is_rotation(primitive) else movement_kind
            )
            direction = _primitive_travel_direction(primitive)
            key = (kind, direction)
            if group and key != current_key:
                assert current_key is not None
                self.add_group(tuple(group), current_key[0], current_key[1])
                group = []
            group.append(primitive_index)
            current_key = key
        if group:
            assert current_key is not None
            self.add_group(tuple(group), current_key[0], current_key[1])

    def add_group(
        self,
        primitive_indices: tuple[int, ...],
        kind: ReferenceSectionKind,
        direction: ReferenceTravelDirection | None = None,
    ) -> int:
        if not primitive_indices:
            raise ValueError("primitive group must not be empty")
        section_index = len(self.sections)
        primitive_directions = {
            _primitive_travel_direction(self.seed.primitive_sequence[index])
            for index in primitive_indices
        }
        if direction is None:
            if len(primitive_directions) != 1:
                raise LocalReferenceSourceError("mixed_travel_direction_section")
            direction = next(iter(primitive_directions))
        elif primitive_directions != {direction}:
            raise LocalReferenceSourceError("source_primitive_direction_mismatch")
        abstract_connector = direction is ReferenceTravelDirection.NONE and all(
            self.seed.primitive_sequence[index].kind is SpatialPrimitiveKind.ANCHOR_CONNECTOR
            for index in primitive_indices
        )
        if direction is ReferenceTravelDirection.NONE and not (
            abstract_connector or kind is ReferenceSectionKind.ROTATE
        ):
            raise LocalReferenceSourceError("unsupported_none_direction_section")
        direction_transition = self._prepare_direction_transition(direction)
        first_primitive = primitive_indices[0]
        first_pose = self.seed.pose_heading_path[first_primitive]
        first_roles: tuple[ReferenceKnotRole, ...]
        if kind is ReferenceSectionKind.ROTATE:
            first_roles = (
                ReferenceKnotRole.ROTATION_ENTRY,
                ReferenceKnotRole.STOP_MARKER,
            )
        elif abstract_connector:
            first_roles = (
                ReferenceKnotRole.ANCHOR,
                ReferenceKnotRole.STOP_MARKER,
            )
        else:
            first_roles = (ReferenceKnotRole.TRANSLATION,)
            if not self.knots:
                first_roles = (ReferenceKnotRole.ANCHOR, *first_roles)
            if direction_transition:
                first_roles = (*first_roles, ReferenceKnotRole.STOP_MARKER)
        self._append_knot(first_pose, first_primitive, section_index, first_roles)
        for offset, primitive_index in enumerate(primitive_indices):
            primitive = self.seed.primitive_sequence[primitive_index]
            end_pose = self.seed.pose_heading_path[primitive_index + 1]
            self.arc_m += hypot(
                primitive.end_pose.x - primitive.start_pose.x,
                primitive.end_pose.y - primitive.start_pose.y,
            )
            roles = (
                (ReferenceKnotRole.ANCHOR, ReferenceKnotRole.STOP_MARKER)
                if abstract_connector
                else (ReferenceKnotRole.TRANSLATION,)
            )
            if kind is ReferenceSectionKind.ROTATE:
                roles = (
                    (ReferenceKnotRole.ROTATION_EXIT, ReferenceKnotRole.STOP_MARKER)
                    if offset == len(primitive_indices) - 1
                    else (ReferenceKnotRole.ROTATION_ENTRY,)
                )
            if kind is ReferenceSectionKind.REJOIN and offset == len(primitive_indices) - 1:
                roles = (*roles, ReferenceKnotRole.REJOIN, ReferenceKnotRole.STOP_MARKER)
            self._append_knot(end_pose, primitive_index + 1, section_index, roles)
        self.sections.append(
            ReferenceSection(
                section_index=section_index,
                section_kind=kind,
                travel_direction=direction,
                first_knot_index=len(self.knots) - len(primitive_indices) - 1,
                last_knot_index=len(self.knots) - 1,
                entry_requires_stopped=(
                    kind is ReferenceSectionKind.ROTATE
                    or abstract_connector
                    or direction_transition
                ),
                exit_requires_stopped=(
                    kind in (ReferenceSectionKind.ROTATE, ReferenceSectionKind.REJOIN)
                    or abstract_connector
                ),
                source_primitive_indices=primitive_indices,
            )
        )
        return section_index

    def add_anchor(self, source_path_index: int, kind: ReferenceSectionKind) -> int:
        section_index = len(self.sections)
        roles = (ReferenceKnotRole.ANCHOR,)
        if kind is ReferenceSectionKind.REJOIN:
            roles = (ReferenceKnotRole.REJOIN, ReferenceKnotRole.STOP_MARKER)
        self._append_knot(
            self.seed.pose_heading_path[source_path_index],
            source_path_index,
            section_index,
            roles,
        )
        self.sections.append(
            ReferenceSection(
                section_index=section_index,
                section_kind=kind,
                travel_direction=ReferenceTravelDirection.NONE,
                first_knot_index=len(self.knots) - 1,
                last_knot_index=len(self.knots) - 1,
                entry_requires_stopped=False,
                exit_requires_stopped=kind is ReferenceSectionKind.REJOIN,
                source_primitive_indices=(),
            )
        )
        return section_index

    def _prepare_direction_transition(
        self,
        direction: ReferenceTravelDirection,
    ) -> bool:
        if direction is ReferenceTravelDirection.NONE:
            return False
        previous_index = next(
            (
                index
                for index in range(len(self.sections) - 1, -1, -1)
                if self.sections[index].travel_direction is not ReferenceTravelDirection.NONE
            ),
            None,
        )
        if previous_index is None:
            return False
        previous = self.sections[previous_index]
        if previous.travel_direction is direction:
            return False

        self.sections[previous_index] = replace(
            previous,
            exit_requires_stopped=True,
            section_content_hash="",
        )
        self._add_stop_marker(previous.last_knot_index)
        for index in range(previous_index + 1, len(self.sections)):
            section = self.sections[index]
            self.sections[index] = replace(
                section,
                entry_requires_stopped=True,
                exit_requires_stopped=True,
                section_content_hash="",
            )
            self._add_stop_marker(section.first_knot_index)
            self._add_stop_marker(section.last_knot_index)
        return True

    def _add_stop_marker(self, knot_index: int) -> None:
        knot = self.knots[knot_index]
        self.knots[knot_index] = replace(
            knot,
            knot_roles=(*knot.knot_roles, ReferenceKnotRole.STOP_MARKER),
        )

    def _append_knot(
        self,
        pose: object,
        source_path_index: int,
        section_index: int,
        roles: tuple[ReferenceKnotRole, ...],
    ) -> None:
        from hospital_path_lab.contracts import Pose2D

        if not isinstance(pose, Pose2D):
            raise TypeError("source path pose must be Pose2D")
        self.knots.append(
            ReferenceKnot(
                knot_index=len(self.knots),
                pose=pose,
                tangent_yaw=pose.yaw,
                cumulative_translation_arc_m=self.arc_m,
                source_path_index=source_path_index,
                section_index=section_index,
                knot_roles=roles,
            )
        )

    def finish(self) -> tuple[tuple[ReferenceKnot, ...], tuple[ReferenceSection, ...]]:
        return tuple(self.knots), tuple(self.sections)


def _require_context_matches_request(
    context: ReferenceBuildContext,
    request: BoundedSpatialOracleRequest,
) -> None:
    if (
        context.map_id != request.map_id
        or context.map_revision != request.map_revision
        or context.mission_revision != request.mission_revision
        or context.grid_content_hash != request.grid_content_hash
        or context.vehicle_profile_hash != request.vehicle_profile_hash
        or context.allowed_region != request.allowed_region
        or context.forbidden_cells != request.forbidden_cells
    ):
        raise LocalReferenceSourceError("map_or_mission_provenance_mismatch")
    expected_reference = (request.reference_segment.start, request.reference_segment.end)
    context_segments = tuple(
        (left, right)
        for left, right in zip(
            context.original_reference,
            context.original_reference[1:],
            strict=False,
        )
    )
    if expected_reference not in context_segments:
        raise LocalReferenceSourceError("original_reference_mismatch")
    if not _poses_match(context.current_robot_pose, request.start_pose):
        raise LocalReferenceSourceError("start_pose_mismatch")


def _require_seed_matches_context(
    context: ReferenceBuildContext,
    seed: SpatialReferenceSeed,
) -> None:
    if seed.seed_content_hash != seed.expected_content_hash:
        raise LocalReferenceSourceError("source_hash_mismatch")
    if (
        seed.map_id != context.map_id
        or seed.map_revision != context.map_revision
        or seed.mission_revision != context.mission_revision
        or seed.grid_content_hash != context.grid_content_hash
        or seed.vehicle_profile_hash != context.vehicle_profile_hash
    ):
        raise LocalReferenceSourceError("map_or_mission_provenance_mismatch")
    if not _poses_match(seed.start_pose, context.current_robot_pose):
        raise LocalReferenceSourceError("start_pose_mismatch")


def _kind_for_side(side: ManeuverSide) -> LocalManeuverKind:
    if side is ManeuverSide.LEFT:
        return LocalManeuverKind.PASS_LEFT
    if side is ManeuverSide.RIGHT:
        return LocalManeuverKind.PASS_RIGHT
    raise LocalReferenceSourceError("unspecified_side_not_supported")


def _primitive_is_rotation(primitive: SpatialPrimitive) -> bool:
    distance = _pose_distance(primitive.start_pose, primitive.end_pose)
    heading_change = abs(_angle_delta(primitive.start_pose.yaw, primitive.end_pose.yaw))
    if distance > _TOLERANCE and heading_change > _TOLERANCE:
        raise LocalReferenceSourceError("non_atomic_anchor_connector")
    return primitive.kind in _ROTATION_KINDS or (
        distance <= _TOLERANCE and heading_change > _TOLERANCE
    )


def _primitive_travel_direction(primitive: SpatialPrimitive) -> ReferenceTravelDirection:
    if primitive.kind is SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION:
        return ReferenceTravelDirection.FORWARD
    if primitive.kind is SpatialPrimitiveKind.REVERSE_ONE_TRANSLATION:
        return ReferenceTravelDirection.REVERSE
    if primitive.kind in _ROTATION_KINDS:
        return ReferenceTravelDirection.NONE
    if primitive.kind is not SpatialPrimitiveKind.ANCHOR_CONNECTOR:
        raise LocalReferenceSourceError("unsupported_source_primitive_direction")

    return ReferenceTravelDirection.NONE


def _pose_distance(left: object, right: object) -> float:
    from hospital_path_lab.contracts import Pose2D

    if not isinstance(left, Pose2D) or not isinstance(right, Pose2D):
        raise TypeError("pose distance requires Pose2D values")
    return hypot(right.x - left.x, right.y - left.y)


def _poses_match(left: object, right: object) -> bool:
    from hospital_path_lab.contracts import Pose2D

    return (
        isinstance(left, Pose2D)
        and isinstance(right, Pose2D)
        and _pose_distance(left, right) <= _TOLERANCE
        and abs(_angle_delta(left.yaw, right.yaw)) <= _TOLERANCE
    )


def _angle_delta(left: float, right: float) -> float:
    return atan2(sin(left - right), cos(left - right))


def _require_exact_nonnegative_int(value: int, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative exact integer")


__all__ = [
    "LOCAL_REFERENCE_BUILDER_VERSION",
    "LocalReferenceSourceError",
    "SpatialReferenceSource",
    "build_spatial_local_reference",
    "build_spatial_reference_set",
    "project_validated_spatial_seed",
]
