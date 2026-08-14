from __future__ import annotations

from dataclasses import replace
from hashlib import sha256
from math import pi

import numpy as np

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.grid import GridMap
from hospital_path_lab.spatial_oracle_contracts import (
    SPATIAL_ORACLE_SCHEMA_VERSION,
    ManeuverSide,
    SpatialAllowedRegion,
    SpatialLatticeConfig,
    SpatialLatticeState,
    SpatialPrimitive,
    SpatialPrimitiveKind,
    SpatialReferenceSegment,
    SpatialRejoinGoal,
    SpatialSearchRegion,
    build_bounded_spatial_request,
)
from hospital_path_lab.spatial_oracle_validation import validate_spatial_oracle_path
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _pose(grid: GridMap, state: SpatialLatticeState) -> Pose2D:
    center = grid.cell_to_pose((state.x_cell, state.y_cell))
    return Pose2D(center.x, center.y, state.heading_index * pi / 4.0)


def _left_bypass(
    grid: GridMap,
) -> tuple[tuple[Pose2D, ...], tuple[SpatialPrimitive, ...]]:
    states = (
        SpatialLatticeState(30, 50, 0, False),
        SpatialLatticeState(30, 50, 1, False),
        SpatialLatticeState(34, 54, 1, False),
        SpatialLatticeState(38, 58, 1, True),
        SpatialLatticeState(38, 58, 0, True),
        SpatialLatticeState(38, 58, 7, True),
        SpatialLatticeState(42, 54, 7, True),
        SpatialLatticeState(46, 50, 7, True),
        SpatialLatticeState(46, 50, 0, True),
        SpatialLatticeState(51, 50, 0, True),
        SpatialLatticeState(56, 50, 0, True),
        SpatialLatticeState(61, 50, 0, True),
        SpatialLatticeState(66, 50, 0, True),
    )
    kinds = (
        SpatialPrimitiveKind.ROTATE_LEFT_45,
        SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
        SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
        SpatialPrimitiveKind.ROTATE_RIGHT_45,
        SpatialPrimitiveKind.ROTATE_RIGHT_45,
        SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
        SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
        SpatialPrimitiveKind.ROTATE_LEFT_45,
        SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
        SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
        SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
        SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
    )
    path = tuple(_pose(grid, state) for state in states)
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
    return path, primitives


def _request(
    grid: GridMap | None = None,
    *,
    side: ManeuverSide = ManeuverSide.LEFT,
    allowed_region: SpatialAllowedRegion | None = None,
):
    grid = grid or GridMap(np.zeros((110, 150), dtype=np.bool_), resolution_m=0.02)
    path, _ = _left_bypass(grid)
    region = tuple((x, y) for y in range(10, 100) for x in range(10, 140))
    return build_bounded_spatial_request(
        schema_version=SPATIAL_ORACLE_SCHEMA_VERSION,
        map_id="validator-map",
        map_revision=1,
        mission_revision=2,
        static_grid=grid,
        forbidden_cells=(),
        allowed_region=allowed_region or SpatialAllowedRegion(),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        start_pose=path[0],
        rejoin_goal=SpatialRejoinGoal(path[-1]),
        reference_segment=SpatialReferenceSegment(path[0], path[-1]),
        maneuver_side=side,
        search_region=SpatialSearchRegion(region),
        lattice_config=SpatialLatticeConfig(),
        source_projection_hash=sha256(b"validator-source").hexdigest(),
    )


def test_independent_validator_accepts_oriented_left_bypass_and_rejoin() -> None:
    request = _request()
    path, primitives = _left_bypass(request.static_grid)

    validation = validate_spatial_oracle_path(request, path, primitives)

    assert validation.passed
    assert validation.failure_codes == ()
    assert validation.maximum_signed_side_excursion_m >= 0.16 - 1e-9
    assert validation.minimum_clearance_m is not None
    assert validation.minimum_clearance_m >= 0.08


def test_same_left_path_cannot_claim_right_side() -> None:
    request = _request(side=ManeuverSide.RIGHT)
    path, primitives = _left_bypass(request.static_grid)

    validation = validate_spatial_oracle_path(request, path, primitives)

    assert not validation.passed
    assert "opposite_side_excursion" in validation.failure_codes
    assert "required_side_excursion_missing" in validation.failure_codes


def test_static_obstacle_on_swept_path_is_rejected() -> None:
    occupancy = np.zeros((110, 150), dtype=np.bool_)
    occupancy[54, 42] = True
    grid = GridMap(occupancy, resolution_m=0.02)
    request = _request(grid)
    path, primitives = _left_bypass(grid)

    validation = validate_spatial_oracle_path(request, path, primitives)

    assert not validation.passed
    assert "physical_clearance_violation" in validation.failure_codes


def test_restricted_allowed_region_checks_whole_footprint_not_center_only() -> None:
    grid = GridMap(np.zeros((110, 150), dtype=np.bool_), resolution_m=0.02)
    thin_center_strip = tuple((x, y) for y in range(45, 56) for x in range(10, 140))
    request = _request(
        grid,
        allowed_region=SpatialAllowedRegion(thin_center_strip, unrestricted=False),
    )
    path, primitives = _left_bypass(grid)

    validation = validate_spatial_oracle_path(request, path, primitives)

    assert not validation.passed
    assert "allowed_boundary_clearance_violation" in validation.failure_codes


def test_excursion_phase_cannot_regress_after_becoming_true() -> None:
    request = _request()
    path, primitives = _left_bypass(request.static_grid)
    regressed_end = replace(primitives[4].end_state, required_excursion_reached=False)
    tampered = list(primitives)
    tampered[4] = replace(tampered[4], end_state=regressed_end)

    validation = validate_spatial_oracle_path(request, path, tuple(tampered))

    assert not validation.passed
    assert "excursion_phase_regressed" in validation.failure_codes


def test_request_hash_tamper_is_reported_without_trusting_path() -> None:
    request = _request()
    path, primitives = _left_bypass(request.static_grid)
    tampered = replace(request, map_revision=request.map_revision + 1)

    validation = validate_spatial_oracle_path(tampered, path, primitives)

    assert not validation.passed
    assert "request_content_hash_mismatch" in validation.failure_codes


def test_rotation_sweep_rejects_mid_angle_corner_collision() -> None:
    occupancy = np.zeros((220, 220), dtype=np.bool_)
    occupancy[84, 91] = True
    grid = GridMap(occupancy, resolution_m=0.02)
    start_state = SpatialLatticeState(100, 100, 0, True)
    end_state = SpatialLatticeState(100, 100, 1, True)
    start = _pose(grid, start_state)
    end = _pose(grid, end_state)
    region = tuple((x, y) for y in range(60, 141) for x in range(60, 141))
    request = build_bounded_spatial_request(
        schema_version=SPATIAL_ORACLE_SCHEMA_VERSION,
        map_id="rotation-sweep-map",
        map_revision=1,
        mission_revision=1,
        static_grid=grid,
        forbidden_cells=(),
        allowed_region=SpatialAllowedRegion(),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        start_pose=start,
        rejoin_goal=SpatialRejoinGoal(end),
        reference_segment=SpatialReferenceSegment(
            Pose2D(start.x - 1.0, start.y - 0.10),
            Pose2D(start.x + 1.0, start.y - 0.10),
        ),
        maneuver_side=ManeuverSide.LEFT,
        search_region=SpatialSearchRegion(region),
        lattice_config=SpatialLatticeConfig(),
        source_projection_hash=sha256(b"rotation-source").hexdigest(),
    )
    primitive = SpatialPrimitive(
        kind=SpatialPrimitiveKind.ROTATE_LEFT_45,
        start_pose=start,
        end_pose=end,
        start_state=start_state,
        end_state=end_state,
    )

    validation = validate_spatial_oracle_path(request, (start, end), (primitive,))

    assert not validation.passed
    assert "physical_clearance_violation" in validation.failure_codes
