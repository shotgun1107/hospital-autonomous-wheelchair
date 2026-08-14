from __future__ import annotations

from dataclasses import replace
from hashlib import sha256

import numpy as np
import pytest

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.grid import GridMap
from hospital_path_lab.spatial_oracle_contracts import (
    SPATIAL_LATTICE_CONFIG_VERSION,
    SPATIAL_ORACLE_RESULT_SCHEMA_VERSION,
    SPATIAL_ORACLE_SCHEMA_VERSION,
    SPATIAL_ORACLE_VERSION,
    SPATIAL_VALIDATOR_VERSION,
    BoundedSpatialOracleRequest,
    BoundedSpatialOracleResult,
    ManeuverSide,
    SpatialAllowedRegion,
    SpatialLatticeConfig,
    SpatialOracleStatus,
    SpatialOracleValidation,
    SpatialReferenceSegment,
    SpatialRejoinGoal,
    SpatialSearchRegion,
    build_bounded_spatial_request,
    spatial_path_content_hash,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _request() -> BoundedSpatialOracleRequest:
    grid = GridMap(np.zeros((100, 150), dtype=np.bool_), resolution_m=0.02)
    region = tuple((x, y) for y in range(10, 90) for x in range(10, 140))
    return build_bounded_spatial_request(
        schema_version=SPATIAL_ORACLE_SCHEMA_VERSION,
        map_id="contract-map",
        map_revision=3,
        mission_revision=7,
        static_grid=grid,
        forbidden_cells=(),
        allowed_region=SpatialAllowedRegion(),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        start_pose=Pose2D(0.60, 1.00, 0.0),
        rejoin_goal=SpatialRejoinGoal(Pose2D(2.40, 1.00, 0.0)),
        reference_segment=SpatialReferenceSegment(
            Pose2D(0.60, 1.00, 0.0), Pose2D(2.40, 1.00, 0.0)
        ),
        maneuver_side=ManeuverSide.LEFT,
        search_region=SpatialSearchRegion(region),
        lattice_config=SpatialLatticeConfig(),
        source_projection_hash=sha256(b"contract-source").hexdigest(),
    )


def _validation(request: BoundedSpatialOracleRequest) -> SpatialOracleValidation:
    path = (request.start_pose,)
    return SpatialOracleValidation(
        validator_version=SPATIAL_VALIDATOR_VERSION,
        passed=True,
        failure_codes=(),
        request_content_hash=request.request_content_hash,
        path_content_hash=spatial_path_content_hash(path, ()),
        minimum_clearance_m=0.25,
        minimum_physical_clearance_m=0.25,
        minimum_forbidden_clearance_m=1.0,
        minimum_allowed_boundary_clearance_m=1.0,
        path_length_m=0.0,
        reverse_length_m=0.0,
        rotation_count=0,
        maximum_signed_side_excursion_m=0.10,
    )


def test_request_factory_binds_grid_profile_region_and_projection_hashes() -> None:
    request = _request()

    assert request.request_content_hash == request.expected_content_hash
    assert request.integrity_failure() is None
    assert request.search_region.content_hash
    assert request.lattice_config.content_hash
    assert request.vehicle_profile_hash


def test_same_shape_grid_byte_tamper_is_fail_closed_by_request_hash() -> None:
    request = _request()
    occupancy = np.array(request.static_grid.occupancy, copy=True)
    occupancy[50, 50] = True
    changed_grid = GridMap(occupancy, resolution_m=request.static_grid.resolution_m)

    tampered = replace(request, static_grid=changed_grid)

    assert tampered.integrity_failure() == "request_content_hash_mismatch"


def test_explicit_wrong_request_hash_is_not_silently_rewritten() -> None:
    request = _request()
    wrong_hash = sha256(b"wrong-request").hexdigest()

    tampered = replace(request, request_content_hash=wrong_hash)

    assert tampered.request_content_hash == wrong_hash
    assert tampered.integrity_failure() == "request_content_hash_mismatch"


def test_frozen_lattice_rejects_geometry_or_primitive_drift() -> None:
    assert SpatialLatticeConfig().config_version == SPATIAL_LATTICE_CONFIG_VERSION

    with pytest.raises(ValueError, match="frozen"):
        SpatialLatticeConfig(heading_bin_count=16)
    with pytest.raises(ValueError, match="frozen"):
        SpatialLatticeConfig(allow_reverse=False)
    with pytest.raises(TypeError, match="must be a bool"):
        SpatialLatticeConfig(allow_forward=1)  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="positive exact integer"):
        SpatialLatticeConfig(max_expanded_states=True)


def test_side_request_requires_frozen_point_one_metre_excursion() -> None:
    with pytest.raises(ValueError, match="minimum_side_excursion_m"):
        SpatialRejoinGoal(Pose2D(1.0, 1.0), minimum_side_excursion_m=0.0)


def test_result_semantic_hash_excludes_elapsed_nonqualification_time() -> None:
    request = _request()
    validation = _validation(request)
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
        path=(request.start_pose,),
        primitive_sequence=(),
        path_length_m=0.0,
        reverse_length_m=0.0,
        rotation_count=0,
        minimum_clearance_m=0.25,
        minimum_physical_clearance_m=0.25,
        minimum_forbidden_clearance_m=1.0,
        minimum_allowed_boundary_clearance_m=1.0,
        generated_edges=0,
        expanded_states=0,
        peak_open_states=1,
        exhaustive=False,
        validation=validation,
        limitations=("simulation_only",),
        elapsed_nonqualification_ns=10,
    )

    later = replace(result, elapsed_nonqualification_ns=999, semantic_content_hash="")

    assert later.semantic_content_hash == result.semantic_content_hash
    with pytest.raises(ValueError, match="semantic_content_hash mismatch"):
        replace(result, semantic_content_hash=sha256(b"wrong-result").hexdigest())


def test_resource_limit_cannot_be_serialized_as_exhaustive_or_carry_path() -> None:
    request = _request()
    common = dict(
        schema_version=SPATIAL_ORACLE_RESULT_SCHEMA_VERSION,
        oracle_version=SPATIAL_ORACLE_VERSION,
        status=SpatialOracleStatus.RESOURCE_LIMIT,
        termination_reason="max_expanded_states",
        request_content_hash=request.request_content_hash,
        map_id=request.map_id,
        map_revision=request.map_revision,
        mission_revision=request.mission_revision,
        grid_content_hash=request.grid_content_hash,
        vehicle_profile_hash=request.vehicle_profile_hash,
        search_region_hash=request.search_region.content_hash,
        lattice_config_hash=request.lattice_config.content_hash,
        path=(),
        primitive_sequence=(),
        path_length_m=None,
        reverse_length_m=None,
        rotation_count=None,
        minimum_clearance_m=None,
        minimum_physical_clearance_m=None,
        minimum_forbidden_clearance_m=None,
        minimum_allowed_boundary_clearance_m=None,
        generated_edges=10,
        expanded_states=4,
        peak_open_states=5,
        validation=None,
        limitations=("simulation_only",),
        elapsed_nonqualification_ns=0,
    )

    with pytest.raises(ValueError, match="cannot be exhaustive"):
        BoundedSpatialOracleResult(**common, exhaustive=True)
    with pytest.raises(ValueError, match="cannot carry"):
        BoundedSpatialOracleResult(
            **(common | {"path": (request.start_pose,)}), exhaustive=False
        )
