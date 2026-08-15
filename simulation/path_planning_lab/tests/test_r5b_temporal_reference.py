from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from hospital_path_lab.local_reference_contracts import (
    LocalManeuverKind,
    ReferenceEvidenceLevel,
    ReferenceKnotRole,
    ReferenceSectionKind,
)
from hospital_path_lab.local_reference_validation import validate_local_maneuver_reference
from hospital_path_lab.r5b_temporal_evidence import frozen_r2_archive_path
from hospital_path_lab.r5b_temporal_reference import (
    build_r5b_temporal_reference_bundles,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def bundles():
    return build_r5b_temporal_reference_bundles(
        frozen_r2_archive_path(REPOSITORY_ROOT)
    )


def test_builds_ten_unique_ground_truth_temporal_references(bundles) -> None:
    assert len(bundles) == 10
    assert len({item.bundle_content_hash for item in bundles}) == 10
    assert all(item.validation.passed for item in bundles)
    assert all(
        item.reference.evidence_level is ReferenceEvidenceLevel.GROUND_TRUTH_TEMPORAL
        for item in bundles
    )
    assert all(item.reference.source_spatial_seed_hash is None for item in bundles)
    assert all(item.reference.source_temporal_evidence_hash for item in bundles)
    assert all(item.reference.source_temporal_geometry_hash for item in bundles)


def test_temporal_reference_keeps_ordered_pass_structure_and_terminal_stop(bundles) -> None:
    for item in bundles:
        reference = item.reference
        kinds = tuple(section.section_kind for section in reference.sections)
        assert kinds[0] is ReferenceSectionKind.DEPART
        assert kinds[-1] is ReferenceSectionKind.REJOIN
        assert kinds.index(ReferenceSectionKind.DEPART) < kinds.index(
            ReferenceSectionKind.BYPASS
        )
        assert kinds.index(ReferenceSectionKind.BYPASS) < kinds.index(
            ReferenceSectionKind.RETURN
        )
        assert kinds.index(ReferenceSectionKind.RETURN) < kinds.index(
            ReferenceSectionKind.REJOIN
        )
        terminal_roles = set(reference.knots[-1].knot_roles)
        assert ReferenceKnotRole.REJOIN in terminal_roles
        assert ReferenceKnotRole.STOP_MARKER in terminal_roles
        assert reference.knots[0].pose == item.source.witness.points[40].pose
        assert reference.knots[-1].pose == item.source.witness.points[-11].pose


def test_temporal_evidence_progress_and_actor_binding_are_ordered(bundles) -> None:
    for item in bundles:
        evidence = item.temporal_evidence
        assert evidence.ground_truth_only
        assert evidence.maneuver_kind in (
            LocalManeuverKind.PASS_LEFT,
            LocalManeuverKind.PASS_RIGHT,
        )
        assert evidence.target_actor_binding_ids == item.source.witness.required_pass_actor_ids
        assert evidence.departure_progress_m is not None
        assert evidence.pass_progress_m is not None
        assert evidence.rejoin_progress_m is not None
        assert (
            evidence.departure_progress_m
            < evidence.pass_progress_m
            < evidence.rejoin_progress_m
        )


def test_temporal_reference_geometry_is_independently_reproduced(bundles) -> None:
    for item in bundles:
        validation = validate_local_maneuver_reference(
            item.build_context,
            item.reference,
            temporal_evidence=item.temporal_evidence,
            temporal_geometry=item.temporal_geometry,
        )
        assert validation == item.validation
        assert validation.minimum_clearance_m is not None
        assert validation.minimum_clearance_m >= 0.08
        assert validation.maximum_signed_side_excursion_m is not None
        assert validation.maximum_signed_side_excursion_m >= 0.649


def test_temporal_geometry_hash_tampering_fails_closed(bundles) -> None:
    item = bundles[0]
    with pytest.raises(ValueError, match="geometry_content_hash mismatch"):
        replace(item.temporal_geometry, geometry_content_hash="0" * 64)


def test_temporal_reference_build_is_deterministic(bundles) -> None:
    repeated = build_r5b_temporal_reference_bundles(
        frozen_r2_archive_path(REPOSITORY_ROOT)
    )
    assert tuple(item.bundle_content_hash for item in repeated) == tuple(
        item.bundle_content_hash for item in bundles
    )
