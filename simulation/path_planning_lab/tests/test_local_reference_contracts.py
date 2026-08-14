from __future__ import annotations

from dataclasses import replace
from hashlib import sha256
from math import hypot

import numpy as np
import pytest

from hospital_path_lab.contracts import GridSnapshot, Pose2D, SnapshotMetadata
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_reference_contracts import (
    LOCAL_REFERENCE_CONTRACT_VERSION,
    LOCAL_REFERENCE_SCHEMA_VERSION,
    LOCAL_REFERENCE_SET_SCHEMA_VERSION,
    LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION,
    REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
    REFERENCE_SESSION_BINDING_VERSION,
    SPATIAL_REFERENCE_SEED_SCHEMA_VERSION,
    TEMPORAL_REFERENCE_EVIDENCE_SCHEMA_VERSION,
    LocalManeuverKind,
    LocalManeuverReference,
    LocalManeuverReferenceSet,
    LocalReferenceWindow,
    ObservationDependency,
    ReferenceBuildContext,
    ReferenceBuildStatus,
    ReferenceEvidenceLevel,
    ReferenceKnot,
    ReferenceKnotRole,
    ReferenceLifecycleStatus,
    ReferenceRevisionBinding,
    ReferenceSection,
    ReferenceSectionKind,
    ReferenceSourceRejection,
    ReferenceUpperDisposition,
    ReferenceValidity,
    SpatialReferenceSeed,
    TemporalReferenceEvidence,
    evaluate_reference_revision_update,
    reference_revision_binding,
    transition_reference_lifecycle,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.spatial_oracle_contracts import (
    ManeuverSide,
    SpatialAllowedRegion,
    SpatialPrimitive,
    SpatialPrimitiveKind,
    SpatialRejoinGoal,
    spatial_grid_content_hash,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _hash(label: str) -> str:
    return sha256(label.encode("utf-8")).hexdigest()


def _context(
    *,
    dependency: ObservationDependency = ObservationDependency.STATIC_ONLY,
) -> ReferenceBuildContext:
    grid = GridMap(np.zeros((80, 120), dtype=np.bool_), resolution_m=0.02)
    observation_revision = 11 if dependency is ObservationDependency.REQUIRED else None
    snapshot = GridSnapshot(
        SnapshotMetadata(
            map_id="r4-map",
            map_revision=3,
            mission_revision=7,
            observation_revision=observation_revision or 0,
            seed=20260814,
            content_hash=_hash("snapshot"),
        ),
        grid,
    )
    allowed = SpatialAllowedRegion()
    original = (Pose2D(0.60, 1.00, 0.0), Pose2D(1.20, 1.00, 0.0))
    return ReferenceBuildContext(
        schema_version=REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
        mission_id="mission-r4",
        stop_epoch=2,
        map_id="r4-map",
        map_revision=3,
        mission_revision=7,
        observation_dependency=dependency,
        observation_revision=observation_revision,
        observation_content_hash=(
            _hash("observation") if dependency is ObservationDependency.REQUIRED else None
        ),
        static_grid_snapshot=snapshot,
        grid_content_hash=spatial_grid_content_hash(grid),
        allowed_region=allowed,
        allowed_region_hash=allowed.content_hash,
        forbidden_cells=(),
        forbidden_region_hash=canonical_content_hash(()),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        vehicle_profile_hash=canonical_content_hash(VIRTUAL_DOLL_WHEELCHAIR_V0_1),
        original_reference=original,
        original_reference_hash=canonical_content_hash(original),
        current_robot_pose=original[0],
        control_tick=40,
        simulation_time_s=2.0,
    )


def _seed() -> SpatialReferenceSeed:
    start = Pose2D(0.60, 1.00, 0.0)
    end = Pose2D(0.80, 1.00, 0.0)
    primitive = SpatialPrimitive(
        kind=SpatialPrimitiveKind.ANCHOR_CONNECTOR,
        start_pose=start,
        end_pose=end,
        start_state=None,
        end_state=None,
    )
    return SpatialReferenceSeed(
        schema_version=SPATIAL_REFERENCE_SEED_SCHEMA_VERSION,
        source_spatial_result_hash=_hash("spatial-result"),
        source_spatial_request_hash=_hash("spatial-request"),
        source_validation_hash=_hash("spatial-validation"),
        map_id="r4-map",
        map_revision=3,
        mission_revision=7,
        grid_content_hash=_hash("grid"),
        vehicle_profile_hash=_hash("vehicle"),
        side=ManeuverSide.LEFT,
        start_pose=start,
        rejoin_goal=SpatialRejoinGoal(end),
        pose_heading_path=(start, end),
        primitive_sequence=(primitive,),
        minimum_clearance_m=0.20,
        limitations=("simulation_only",),
    )


def _pass_geometry() -> tuple[tuple[ReferenceKnot, ...], tuple[ReferenceSection, ...]]:
    poses = (
        Pose2D(0.60, 1.00, 0.0),
        Pose2D(0.80, 1.20, 0.0),
        Pose2D(1.00, 1.20, 0.0),
        Pose2D(1.20, 1.00, 0.0),
    )
    first_delta = hypot(0.2, 0.2)
    arcs = (0.0, first_delta, first_delta + 0.2, first_delta * 2.0 + 0.2)
    roles = (
        (ReferenceKnotRole.ANCHOR,),
        (ReferenceKnotRole.TRANSLATION,),
        (ReferenceKnotRole.TRANSLATION,),
        (
            ReferenceKnotRole.TRANSLATION,
            ReferenceKnotRole.REJOIN,
            ReferenceKnotRole.STOP_MARKER,
        ),
    )
    knots = tuple(
        ReferenceKnot(
            knot_index=index,
            pose=pose,
            tangent_yaw=0.0,
            cumulative_translation_arc_m=arcs[index],
            source_path_index=index,
            section_index=index,
            knot_roles=roles[index],
        )
        for index, pose in enumerate(poses)
    )
    kinds = (
        ReferenceSectionKind.DEPART,
        ReferenceSectionKind.BYPASS,
        ReferenceSectionKind.RETURN,
        ReferenceSectionKind.REJOIN,
    )
    sections = tuple(
        ReferenceSection(
            section_index=index,
            section_kind=kind,
            first_knot_index=index,
            last_knot_index=index,
            entry_requires_stopped=False,
            exit_requires_stopped=kind is ReferenceSectionKind.REJOIN,
            source_primitive_indices=(index,),
        )
        for index, kind in enumerate(kinds)
    )
    return knots, sections


def _validity(*, observation_revision: int | None = None) -> ReferenceValidity:
    return ReferenceValidity(
        required_mission_id="mission-r4",
        required_stop_epoch=2,
        required_map_revision=3,
        required_mission_revision=7,
        required_observation_revision=observation_revision,
        valid_from_control_tick=40,
        valid_until_control_tick=None,
    )


def _pass_reference(
    kind: LocalManeuverKind = LocalManeuverKind.PASS_LEFT,
    *,
    maneuver_revision: int = 5,
    path_revision: int = 8,
    session_label: str = "left-session",
    candidate_label: str | None = None,
) -> LocalManeuverReference:
    knots, sections = _pass_geometry()
    candidate_label = candidate_label or kind.value
    return LocalManeuverReference(
        schema_version=LOCAL_REFERENCE_SCHEMA_VERSION,
        reference_contract_version=LOCAL_REFERENCE_CONTRACT_VERSION,
        candidate_id=_hash(candidate_label),
        maneuver_kind=kind,
        evidence_level=ReferenceEvidenceLevel.SPATIAL_ONLY,
        mission_id="mission-r4",
        stop_epoch=2,
        map_id="r4-map",
        map_revision=3,
        mission_revision=7,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        maneuver_revision=maneuver_revision,
        path_revision=path_revision,
        reference_session_id=_hash(session_label),
        source_spatial_seed_hash=_hash("spatial-seed"),
        source_temporal_evidence_hash=None,
        original_reference_hash=_hash("original-reference"),
        grid_content_hash=_hash("grid"),
        vehicle_profile_hash=_hash("vehicle"),
        allowed_region_hash=_hash("allowed"),
        forbidden_region_hash=_hash("forbidden"),
        knots=knots,
        sections=sections,
        departure_knot_index=0,
        pass_section_index=1,
        rejoin_knot_index=3,
        minimum_validated_static_clearance_m=0.20,
        validity=_validity(),
        generation_reason_codes=("validated_spatial_seed",),
        limitations=("simulation_only",),
    )


def _wait_reference(*, maneuver_revision: int = 5) -> LocalManeuverReference:
    poses = (Pose2D(0.60, 1.00, 0.0), Pose2D(1.20, 1.00, 0.0))
    knots = (
        ReferenceKnot(
            0,
            poses[0],
            0.0,
            0.0,
            0,
            0,
            (ReferenceKnotRole.ANCHOR, ReferenceKnotRole.STOP_MARKER),
        ),
        ReferenceKnot(
            1,
            poses[1],
            0.0,
            0.60,
            1,
            1,
            (ReferenceKnotRole.REJOIN, ReferenceKnotRole.STOP_MARKER),
        ),
    )
    sections = (
        ReferenceSection(
            0,
            ReferenceSectionKind.HOLD,
            0,
            0,
            True,
            True,
            (0,),
        ),
        ReferenceSection(
            1,
            ReferenceSectionKind.FOLLOW_ORIGINAL,
            1,
            1,
            False,
            True,
            (1,),
        ),
    )
    return LocalManeuverReference(
        schema_version=LOCAL_REFERENCE_SCHEMA_VERSION,
        reference_contract_version=LOCAL_REFERENCE_CONTRACT_VERSION,
        candidate_id=_hash("wait"),
        maneuver_kind=LocalManeuverKind.WAIT_OR_FOLLOW,
        evidence_level=ReferenceEvidenceLevel.GROUND_TRUTH_TEMPORAL,
        mission_id="mission-r4",
        stop_epoch=2,
        map_id="r4-map",
        map_revision=3,
        mission_revision=7,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        maneuver_revision=maneuver_revision,
        path_revision=2,
        reference_session_id=_hash("wait-session"),
        source_spatial_seed_hash=None,
        source_temporal_evidence_hash=_hash("wait-evidence"),
        original_reference_hash=_hash("original-reference"),
        grid_content_hash=_hash("grid"),
        vehicle_profile_hash=_hash("vehicle"),
        allowed_region_hash=_hash("allowed"),
        forbidden_region_hash=_hash("forbidden"),
        knots=knots,
        sections=sections,
        departure_knot_index=None,
        pass_section_index=None,
        rejoin_knot_index=1,
        minimum_validated_static_clearance_m=0.20,
        validity=_validity(),
        generation_reason_codes=("validated_wait_evidence",),
        limitations=("simulation_only",),
    )


def _window(
    reference: LocalManeuverReference,
    *,
    subgoal_revision: int = 0,
    full: bool = False,
    source_tick: int = 40,
) -> LocalReferenceWindow:
    if full:
        knots = reference.knots
        sections = reference.sections
    else:
        knots = reference.knots[:2]
        sections = reference.sections[:2]
    return LocalReferenceWindow(
        schema_version=LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION,
        reference_session_id=reference.reference_session_id,
        maneuver_revision=reference.maneuver_revision,
        path_revision=reference.path_revision,
        subgoal_revision=subgoal_revision,
        full_reference_hash=reference.reference_content_hash,
        source_control_tick=source_tick,
        start_knot_index=knots[0].knot_index,
        end_knot_index=knots[-1].knot_index,
        knots=knots,
        sections=sections,
        terminal_rejoin_included=full,
    )


def test_build_context_binds_raw_grid_regions_profile_reference_and_observation() -> None:
    static = _context()
    observed = _context(dependency=ObservationDependency.REQUIRED)

    assert static.context_content_hash == static.expected_content_hash
    assert static.observation_revision is None
    assert observed.observation_revision == 11
    assert observed.context_content_hash != static.context_content_hash

    with pytest.raises(ValueError, match="original_reference_hash mismatch"):
        replace(
            static,
            original_reference=(static.original_reference[0], Pose2D(1.40, 1.00)),
        )
    with pytest.raises(ValueError, match="cannot claim observation"):
        replace(static, observation_revision=1)


def test_build_context_rejects_profile_drift_and_out_of_bounds_allowed_region() -> None:
    context = _context()
    changed_profile = replace(VIRTUAL_DOLL_WHEELCHAIR_V0_1, nominal_speed_mps=0.19)
    with pytest.raises(ValueError, match="frozen virtual wheelchair profile"):
        replace(
            context,
            vehicle_profile=changed_profile,
            vehicle_profile_hash=canonical_content_hash(changed_profile),
            context_content_hash="",
        )

    invalid_allowed = SpatialAllowedRegion(cells=((120, 0),), unrestricted=False)
    with pytest.raises(ValueError, match="out-of-bounds"):
        replace(
            context,
            allowed_region=invalid_allowed,
            allowed_region_hash=invalid_allowed.content_hash,
            context_content_hash="",
        )


def test_spatial_seed_binds_path_primitives_clearance_and_semantic_hash() -> None:
    seed = _seed()

    assert seed.seed_content_hash == seed.expected_content_hash
    assert seed.source_path_content_hash

    with pytest.raises(ValueError, match="primitive sequence length mismatch"):
        replace(seed, primitive_sequence=(), seed_content_hash="")
    with pytest.raises(ValueError, match="minimum clearance"):
        replace(seed, minimum_clearance_m=0.079, seed_content_hash="")
    with pytest.raises(ValueError, match="seed_content_hash mismatch"):
        replace(seed, source_validation_hash=_hash("changed-validation"))


def test_temporal_evidence_requires_ordered_single_actor_pass_anchors() -> None:
    evidence = TemporalReferenceEvidence(
        schema_version=TEMPORAL_REFERENCE_EVIDENCE_SCHEMA_VERSION,
        source_witness_hash=_hash("witness"),
        source_validation_hash=_hash("validation"),
        maneuver_kind=LocalManeuverKind.PASS_LEFT,
        target_actor_binding_ids=("actor-opaque",),
        departure_progress_m=0.2,
        pass_progress_m=0.8,
        rejoin_progress_m=1.2,
        ground_truth_only=True,
        limitations=("open_loop_actor",),
    )

    assert evidence.evidence_content_hash == evidence.expected_content_hash
    with pytest.raises(ValueError, match="must be ordered"):
        replace(evidence, pass_progress_m=1.3, evidence_content_hash="")
    with pytest.raises(ValueError, match="one Actor"):
        replace(evidence, target_actor_binding_ids=(), evidence_content_hash="")


def test_reference_hash_and_structure_reject_terminal_tamper_and_wrong_evidence_claims() -> None:
    reference = _pass_reference()

    assert reference.reference_content_hash == reference.expected_content_hash
    terminal = replace(
        reference.knots[-1],
        knot_roles=(ReferenceKnotRole.REJOIN,),
    )
    with pytest.raises(ValueError, match="REJOIN and STOP_MARKER"):
        replace(reference, knots=(*reference.knots[:-1], terminal), reference_content_hash="")
    with pytest.raises(ValueError, match="cannot claim temporal evidence"):
        replace(
            reference,
            source_temporal_evidence_hash=_hash("false-temporal"),
            reference_content_hash="",
        )
    with pytest.raises(ValueError, match="reference_content_hash mismatch"):
        replace(reference, generation_reason_codes=("changed",))


def test_pass_reference_binds_departure_and_terminal_rejoin_sections() -> None:
    reference = _pass_reference()

    with pytest.raises(ValueError, match="departure_knot_index must identify a DEPART"):
        replace(reference, departure_knot_index=1, reference_content_hash="")
    terminal_not_rejoin = replace(
        reference.sections[-1],
        section_kind=ReferenceSectionKind.RETURN,
        section_content_hash="",
    )
    with pytest.raises(ValueError, match="missing a required section"):
        replace(
            reference,
            sections=(*reference.sections[:-1], terminal_not_rejoin),
            reference_content_hash="",
        )


def test_wait_reference_requires_hold_before_following_original_path() -> None:
    reference = _wait_reference()
    first = replace(
        reference.sections[0],
        section_kind=ReferenceSectionKind.FOLLOW_ORIGINAL,
        section_content_hash="",
    )
    last = replace(
        reference.sections[1],
        section_kind=ReferenceSectionKind.HOLD,
        entry_requires_stopped=True,
        section_content_hash="",
    )

    with pytest.raises(ValueError, match="hold before following"):
        replace(reference, sections=(first, last), reference_content_hash="")


def test_rotate_and_hold_sections_enforce_stopped_geometry_and_markers() -> None:
    with pytest.raises(ValueError, match="distinct entry and exit"):
        ReferenceSection(
            0,
            ReferenceSectionKind.ROTATE,
            0,
            0,
            True,
            True,
            (),
        )

    rotate_knots = (
        ReferenceKnot(
            0,
            Pose2D(0.60, 1.00, 0.0),
            0.0,
            0.0,
            0,
            0,
            (ReferenceKnotRole.ROTATION_ENTRY,),
        ),
        ReferenceKnot(
            1,
            Pose2D(0.60, 1.00, 1.57),
            1.57,
            0.0,
            0,
            0,
            (ReferenceKnotRole.ROTATION_EXIT,),
        ),
    )
    rotate = ReferenceSection(
        0,
        ReferenceSectionKind.ROTATE,
        0,
        1,
        True,
        True,
        (),
    )
    LocalReferenceWindow(
        LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION,
        _hash("rotate-session"),
        1,
        1,
        0,
        _hash("rotate-full"),
        1,
        0,
        1,
        rotate_knots,
        (rotate,),
        False,
    )
    missing_exit = replace(
        rotate_knots[-1],
        knot_roles=(ReferenceKnotRole.STOP_MARKER,),
    )
    with pytest.raises(ValueError, match="ROTATION_EXIT"):
        LocalReferenceWindow(
            LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION,
            _hash("rotate-session"),
            1,
            1,
            0,
            _hash("rotate-full"),
            1,
            0,
            1,
            (rotate_knots[0], missing_exit),
            (rotate,),
            False,
        )

    moving_hold_knots = (
        ReferenceKnot(
            0,
            Pose2D(0.60, 1.00, 0.0),
            0.0,
            0.0,
            0,
            0,
            (ReferenceKnotRole.ANCHOR,),
        ),
        ReferenceKnot(
            1,
            Pose2D(0.61, 1.00, 0.0),
            0.0,
            0.01,
            0,
            0,
            (ReferenceKnotRole.STOP_MARKER,),
        ),
    )
    hold = ReferenceSection(
        0,
        ReferenceSectionKind.HOLD,
        0,
        1,
        True,
        True,
        (),
    )
    with pytest.raises(ValueError, match="one stopped pose"):
        LocalReferenceWindow(
            LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION,
            _hash("hold-session"),
            1,
            1,
            0,
            _hash("hold-full"),
            1,
            0,
            1,
            moving_hold_knots,
            (hold,),
            False,
        )


def test_reference_validity_keeps_stop_resume_and_local_recheck_separate() -> None:
    with pytest.raises(ValueError, match="must remain true"):
        replace(_validity(), requires_resume_authorization=False)
    with pytest.raises(ValueError, match="must not be reversed"):
        replace(_validity(), valid_until_control_tick=39)


def test_reference_set_sorts_wait_left_right_without_implying_preference() -> None:
    wait = _wait_reference()
    left = _pass_reference()
    right = _pass_reference(
        LocalManeuverKind.PASS_RIGHT,
        session_label="right-session",
        candidate_label="right",
    )
    result = LocalManeuverReferenceSet(
        schema_version=LOCAL_REFERENCE_SET_SCHEMA_VERSION,
        status=ReferenceBuildStatus.REFERENCE_SET_READY,
        termination_reason="public_candidates_built",
        build_context_hash=_context().context_content_hash,
        maneuver_revision=5,
        candidates=(right, left, wait),
        upper_dispositions=(ReferenceUpperDisposition.SUPPORT_REQUEST,),
        rejected_sources=(
            ReferenceSourceRejection(_hash("rejected"), ("source_status_not_feasible",)),
        ),
        limitations=("simulation_only",),
        elapsed_nonqualification_ns=10,
    )

    assert tuple(candidate.maneuver_kind for candidate in result.candidates) == (
        LocalManeuverKind.WAIT_OR_FOLLOW,
        LocalManeuverKind.PASS_LEFT,
        LocalManeuverKind.PASS_RIGHT,
    )
    later = replace(result, elapsed_nonqualification_ns=999, semantic_content_hash="")
    assert later.semantic_content_hash == result.semantic_content_hash


def test_reference_set_status_contract_does_not_turn_resource_or_invalid_into_path() -> None:
    common = dict(
        schema_version=LOCAL_REFERENCE_SET_SCHEMA_VERSION,
        termination_reason="none",
        build_context_hash=_context().context_content_hash,
        maneuver_revision=5,
        candidates=(),
        upper_dispositions=(),
        rejected_sources=(),
        limitations=("simulation_only",),
        elapsed_nonqualification_ns=0,
    )

    LocalManeuverReferenceSet(status=ReferenceBuildStatus.SEARCH_INCONCLUSIVE, **common)
    with pytest.raises(ValueError, match="cannot carry candidates"):
        LocalManeuverReferenceSet(
            status=ReferenceBuildStatus.INVALID_INPUT,
            **(common | {"candidates": (_pass_reference(),)}),
        )
    with pytest.raises(ValueError, match="exactly one"):
        LocalManeuverReferenceSet(status=ReferenceBuildStatus.WAIT_ONLY, **common)


def test_window_is_contiguous_atomic_and_tick_is_not_part_of_semantic_hash() -> None:
    reference = _pass_reference()
    first = _window(reference, source_tick=40)
    repeated_next_tick = _window(reference, source_tick=41)
    terminal = _window(reference, subgoal_revision=1, full=True, source_tick=42)

    assert first.window_content_hash == repeated_next_tick.window_content_hash
    assert terminal.window_content_hash != first.window_content_hash
    with pytest.raises(ValueError, match="window range"):
        replace(first, end_knot_index=3, window_content_hash="")


def test_revision_update_is_idempotent_and_preserves_session_for_window_only_update() -> None:
    reference = _pass_reference()
    initial = reference_revision_binding(reference, _window(reference))
    duplicate = reference_revision_binding(reference, _window(reference, source_tick=41))
    advanced = reference_revision_binding(
        reference,
        _window(reference, subgoal_revision=1, full=True, source_tick=42),
    )

    first = evaluate_reference_revision_update(None, initial)
    repeated = evaluate_reference_revision_update(initial, duplicate)
    moved = evaluate_reference_revision_update(initial, advanced)

    assert first.accepted and not first.duplicate
    assert repeated.accepted and repeated.duplicate
    assert moved.accepted and moved.reason_code == "subgoal_revision_advanced"
    assert moved.next_binding.reference_session_id == initial.reference_session_id


def test_same_revision_different_content_and_revision_regression_are_fail_closed() -> None:
    reference = _pass_reference()
    current = reference_revision_binding(reference, _window(reference))
    forged = replace(current, window_content_hash=_hash("forged-window"))
    regressed = replace(current, path_revision=current.path_revision - 1)

    assert (
        evaluate_reference_revision_update(current, forged).reason_code
        == "same_revision_different_content"
    )
    assert (
        evaluate_reference_revision_update(current, regressed).reason_code == "revision_regression"
    )


def test_path_and_stop_epoch_changes_require_new_session_and_maneuver_revision() -> None:
    reference = _pass_reference()
    current = reference_revision_binding(reference, _window(reference))

    same_session_new_path = replace(
        current,
        path_revision=current.path_revision + 1,
        full_reference_hash=_hash("new-path"),
        window_content_hash=_hash("new-window"),
    )
    assert (
        evaluate_reference_revision_update(current, same_session_new_path).reason_code
        == "path_revision_requires_new_session"
    )

    new_path = replace(
        same_session_new_path,
        reference_session_id=_hash("new-path-session"),
    )
    assert evaluate_reference_revision_update(current, new_path).accepted

    wrong_epoch = replace(
        current,
        stop_epoch=current.stop_epoch + 1,
        reference_session_id=_hash("new-epoch-session"),
    )
    assert (
        evaluate_reference_revision_update(current, wrong_epoch).reason_code
        == "stop_epoch_requires_new_maneuver_revision"
    )

    new_maneuver = replace(
        wrong_epoch,
        maneuver_revision=current.maneuver_revision + 1,
        candidate_id=_hash("new-candidate"),
    )
    assert evaluate_reference_revision_update(current, new_maneuver).accepted


def test_subgoal_revision_without_window_change_is_rejected() -> None:
    current = ReferenceRevisionBinding(
        REFERENCE_SESSION_BINDING_VERSION,
        "mission-r4",
        2,
        5,
        8,
        0,
        _hash("candidate"),
        _hash("session"),
        _hash("full"),
        _hash("window"),
    )
    meaningless = replace(current, subgoal_revision=1)

    assert (
        evaluate_reference_revision_update(current, meaningless).reason_code
        == "subgoal_revision_without_window_change"
    )


def test_lifecycle_is_one_way_and_terminal_binding_rejects_future_updates() -> None:
    current = ReferenceRevisionBinding(
        REFERENCE_SESSION_BINDING_VERSION,
        "mission-r4",
        2,
        5,
        8,
        0,
        _hash("candidate"),
        _hash("session"),
        _hash("full"),
        _hash("window"),
    )
    stale = transition_reference_lifecycle(current, ReferenceLifecycleStatus.STALE)

    assert stale.lifecycle is ReferenceLifecycleStatus.STALE
    assert (
        evaluate_reference_revision_update(stale, current).reason_code
        == "current_binding_is_terminal"
    )
    with pytest.raises(ValueError, match="cannot transition again"):
        transition_reference_lifecycle(stale, ReferenceLifecycleStatus.WITHDRAWN)
