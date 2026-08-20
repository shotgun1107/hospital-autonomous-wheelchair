"""Known-map FOLLOW_ORIGINAL reference setup for the R7 runtime facade."""

from __future__ import annotations

from math import atan2, hypot, pi

from hospital_path_lab.contracts import GridSnapshot, Pose2D
from hospital_path_lab.dynamic_contracts import DYNAMIC_CONTROL_PERIOD_S
from hospital_path_lab.local_reference_contracts import (
    LOCAL_REFERENCE_CONTRACT_VERSION,
    LOCAL_REFERENCE_SCHEMA_VERSION,
    REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
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
)
from hospital_path_lab.local_reference_validation import (
    LocalReferenceValidation,
    validate_local_maneuver_reference,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.reference_section_executor import R5_YAW_TOLERANCE_RAD
from hospital_path_lab.spatial_oracle_contracts import (
    SpatialAllowedRegion,
    spatial_grid_content_hash,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1

from .adapters import to_pose
from .contracts import RuntimeMission

_TOLERANCE = 1e-12


class RuntimeReferenceError(ValueError):
    """A known-map reference could not be represented or independently validated."""


def build_runtime_follow_reference(
    mission: RuntimeMission,
    *,
    grid_snapshot: GridSnapshot,
    valid_from_tick: int = 0,
) -> tuple[ReferenceBuildContext, LocalManeuverReference, LocalReferenceValidation]:
    """Adapt a resolved global path to a validated R7 FOLLOW reference.

    This function does not plan a new global path and does not claim temporal
    Actor evidence.  It only binds the runtime-resolved known-map reference to
    the existing R7 contracts, then calls the existing independent validator.
    """

    if not isinstance(mission, RuntimeMission):
        raise TypeError("mission must be a RuntimeMission")
    if not isinstance(grid_snapshot, GridSnapshot):
        raise TypeError("grid_snapshot must be a GridSnapshot")
    if (
        isinstance(valid_from_tick, bool)
        or not isinstance(valid_from_tick, int)
        or valid_from_tick < 0
    ):
        raise RuntimeReferenceError("valid_from_tick_invalid")
    metadata = grid_snapshot.metadata
    if (
        metadata.map_id != mission.runtime_map.map_id
        or metadata.map_revision != mission.runtime_map.map_revision
        or metadata.mission_revision != mission.mission_revision
    ):
        raise RuntimeReferenceError("runtime_map_provenance_mismatch")

    path = _translation_path(mission)
    start = path[0]
    goal = to_pose(mission.goal_pose)
    allowed = SpatialAllowedRegion()
    forbidden = tuple(sorted(grid_snapshot.forbidden_cells))
    context = ReferenceBuildContext(
        schema_version=REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
        mission_id=mission.mission_id,
        stop_epoch=0,
        map_id=metadata.map_id,
        map_revision=metadata.map_revision,
        mission_revision=metadata.mission_revision,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        static_grid_snapshot=grid_snapshot,
        grid_content_hash=spatial_grid_content_hash(grid_snapshot.grid),
        allowed_region=allowed,
        allowed_region_hash=allowed.content_hash,
        forbidden_cells=forbidden,
        forbidden_region_hash=canonical_content_hash(forbidden),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        vehicle_profile_hash=canonical_content_hash(VIRTUAL_DOLL_WHEELCHAIR_V0_1),
        # The frozen local validator defines ``original_reference`` as the
        # two-point mission request line.  The actual planned route is carried
        # by the reference knots below and is independently swept in full.
        original_reference=(path[0], path[-1]),
        original_reference_hash=canonical_content_hash((path[0], path[-1])),
        current_robot_pose=start,
        control_tick=valid_from_tick,
        simulation_time_s=valid_from_tick * DYNAMIC_CONTROL_PERIOD_S,
    )
    reference = _build_reference(
        context,
        mission,
        path=path,
        goal=goal,
        valid_from_tick=valid_from_tick,
    )
    validation = validate_local_maneuver_reference(context, reference)
    if not validation.passed:
        raise RuntimeReferenceError(
            "runtime_reference_validation_failed:" + ",".join(validation.failure_codes)
        )
    return context, reference, validation


def _translation_path(mission: RuntimeMission) -> tuple[Pose2D, ...]:
    if mission.reference_path is None:  # pragma: no cover - start_mission resolves it
        raise RuntimeReferenceError("runtime_reference_path_unresolved")
    raw = tuple(to_pose(point) for point in mission.reference_path)
    result: list[Pose2D] = []
    for index, pose in enumerate(raw):
        yaw = _outgoing_tangent(raw, index)
        if index == 0:
            yaw = pose.yaw
        result.append(Pose2D(pose.x, pose.y, yaw))
    return tuple(result)


def _outgoing_tangent(path: tuple[Pose2D, ...], index: int) -> float:
    current = path[index]
    for next_pose in path[index + 1 :]:
        delta_x = next_pose.x - current.x
        delta_y = next_pose.y - current.y
        if hypot(delta_x, delta_y) > _TOLERANCE:
            return atan2(delta_y, delta_x)
    previous = path[index - 1]
    return atan2(current.y - previous.y, current.x - previous.x)


def _build_reference(
    context: ReferenceBuildContext,
    mission: RuntimeMission,
    *,
    path: tuple[Pose2D, ...],
    goal: Pose2D,
    valid_from_tick: int,
) -> LocalManeuverReference:
    translation_length = _translation_arcs(path)
    travel_yaw = path[-1].yaw
    terminal_yaw_error = abs((goal.yaw - travel_yaw + pi) % (2.0 * pi) - pi)
    needs_terminal_rotation = terminal_yaw_error > R5_YAW_TOLERANCE_RAD
    translation_terminal = Pose2D(goal.x, goal.y, travel_yaw) if needs_terminal_rotation else goal
    translation_path = (*path[:-1], translation_terminal)
    knots = tuple(
        ReferenceKnot(
            knot_index=index,
            pose=pose,
            tangent_yaw=pose.yaw,
            cumulative_translation_arc_m=translation_length[index],
            source_path_index=index,
            section_index=0,
            knot_roles=_translation_roles(index, len(translation_path), needs_terminal_rotation),
        )
        for index, pose in enumerate(translation_path)
    )
    sections: tuple[ReferenceSection, ...] = (
        ReferenceSection(
            section_index=0,
            section_kind=ReferenceSectionKind.FOLLOW_ORIGINAL,
            travel_direction=ReferenceTravelDirection.FORWARD,
            first_knot_index=0,
            last_knot_index=len(knots) - 1,
            entry_requires_stopped=False,
            exit_requires_stopped=needs_terminal_rotation,
            source_primitive_indices=(),
        ),
    )
    if needs_terminal_rotation:
        rotation_entry_index = len(knots)
        rotation_exit_index = rotation_entry_index + 1
        knots = (
            *knots,
            ReferenceKnot(
                knot_index=rotation_entry_index,
                pose=translation_terminal,
                tangent_yaw=translation_terminal.yaw,
                cumulative_translation_arc_m=translation_length[-1],
                source_path_index=rotation_entry_index,
                section_index=1,
                knot_roles=(
                    ReferenceKnotRole.ANCHOR,
                    ReferenceKnotRole.ROTATION_ENTRY,
                    ReferenceKnotRole.STOP_MARKER,
                ),
            ),
            ReferenceKnot(
                knot_index=rotation_exit_index,
                pose=goal,
                tangent_yaw=goal.yaw,
                cumulative_translation_arc_m=translation_length[-1],
                source_path_index=rotation_exit_index,
                section_index=1,
                knot_roles=(
                    ReferenceKnotRole.ANCHOR,
                    ReferenceKnotRole.ROTATION_EXIT,
                    ReferenceKnotRole.REJOIN,
                    ReferenceKnotRole.STOP_MARKER,
                ),
            ),
        )
        sections = (
            *sections,
            ReferenceSection(
                section_index=1,
                section_kind=ReferenceSectionKind.ROTATE,
                travel_direction=ReferenceTravelDirection.NONE,
                first_knot_index=rotation_entry_index,
                last_knot_index=rotation_exit_index,
                entry_requires_stopped=True,
                exit_requires_stopped=True,
                source_primitive_indices=(),
            ),
        )
    identity = {
        "runtime_reference_v1": True,
        "mission_id": context.mission_id,
        "mission_revision": context.mission_revision,
        "map_id": context.map_id,
        "map_revision": context.map_revision,
        "original_reference_hash": context.original_reference_hash,
        "resolved_path_hash": canonical_content_hash(path),
        "valid_from_tick": valid_from_tick,
    }
    return LocalManeuverReference(
        schema_version=LOCAL_REFERENCE_SCHEMA_VERSION,
        reference_contract_version=LOCAL_REFERENCE_CONTRACT_VERSION,
        candidate_id=canonical_content_hash({"runtime_follow_candidate": identity}),
        maneuver_kind=LocalManeuverKind.FOLLOW_ORIGINAL,
        evidence_level=ReferenceEvidenceLevel.SPATIAL_ONLY,
        mission_id=context.mission_id,
        stop_epoch=0,
        map_id=context.map_id,
        map_revision=context.map_revision,
        mission_revision=context.mission_revision,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        maneuver_revision=0,
        path_revision=1,
        reference_session_id=canonical_content_hash({"runtime_follow_session": identity}),
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
        rejoin_knot_index=len(knots) - 1,
        minimum_validated_static_clearance_m=0.08,
        validity=ReferenceValidity(
            required_mission_id=context.mission_id,
            required_stop_epoch=0,
            required_map_revision=context.map_revision,
            required_mission_revision=context.mission_revision,
            required_observation_revision=None,
            valid_from_control_tick=valid_from_tick,
            valid_until_control_tick=None,
        ),
        generation_reason_codes=("runtime_known_map_reference",),
        limitations=(
            "known_map_assumption",
            "no_perception_claim",
            "runtime_facade_simulation_only",
        ),
    )


def _translation_arcs(path: tuple[Pose2D, ...]) -> tuple[float, ...]:
    arcs = [0.0]
    for first, second in zip(path[:-1], path[1:], strict=True):
        arcs.append(arcs[-1] + hypot(second.x - first.x, second.y - first.y))
    return tuple(arcs)


def _translation_roles(
    index: int,
    count: int,
    needs_terminal_rotation: bool,
) -> tuple[ReferenceKnotRole, ...]:
    if index == count - 1:
        if needs_terminal_rotation:
            return (
                ReferenceKnotRole.ANCHOR,
                ReferenceKnotRole.TRANSLATION,
                ReferenceKnotRole.STOP_MARKER,
            )
        return (
            ReferenceKnotRole.ANCHOR,
            ReferenceKnotRole.TRANSLATION,
            ReferenceKnotRole.REJOIN,
            ReferenceKnotRole.STOP_MARKER,
        )
    return (ReferenceKnotRole.ANCHOR, ReferenceKnotRole.TRANSLATION)
