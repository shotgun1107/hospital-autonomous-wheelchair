"""공개 R2-A witness world를 Actor-free R3 정적 request로 투영한다."""

from __future__ import annotations

from math import ceil

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.dynamic_witness_contracts import WitnessWorldSnapshot
from hospital_path_lab.grid import GridMap
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.spatial_oracle_contracts import (
    SPATIAL_ORACLE_SCHEMA_VERSION,
    BoundedSpatialOracleRequest,
    ManeuverSide,
    SpatialAllowedRegion,
    SpatialLatticeConfig,
    SpatialReferenceSegment,
    SpatialRejoinGoal,
    SpatialSearchRegion,
    build_bounded_spatial_request,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1

SPATIAL_PUBLIC_PROJECTION_VERSION = "r2a-static-to-r3-v1"
SPATIAL_PUBLIC_SEARCH_MARGIN_M = 1.0


def project_witness_world_to_spatial_request(
    world: WitnessWorldSnapshot,
    *,
    maneuver_side: ManeuverSide,
    lattice_config: SpatialLatticeConfig | None = None,
) -> BoundedSpatialOracleRequest:
    """Actor·시간·관측·evaluator label을 버리고 straight static request를 만든다."""

    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("R3 projection requires a WitnessWorldSnapshot")
    if not world.simulation_only:
        raise ValueError("R3 projection requires simulation_only input")
    if not isinstance(maneuver_side, ManeuverSide):
        raise TypeError("maneuver_side must be a ManeuverSide")
    if len(world.reference_path) != 2:
        raise ValueError("R3 v1 public projection requires one explicit straight segment")
    if world.vehicle_profile_hash != canonical_content_hash(
        VIRTUAL_DOLL_WHEELCHAIR_V0_1
    ):
        raise ValueError("witness world vehicle profile does not match frozen R3 profile")

    grid = world.grid.to_grid_map()
    reference_start, reference_end = world.reference_path
    region = _reference_bounded_cells(
        grid,
        reference_start.x,
        reference_start.y,
        reference_end.x,
        reference_end.y,
        margin_m=SPATIAL_PUBLIC_SEARCH_MARGIN_M,
    )
    allowed_cells = world.maneuver_constraints.allowed_cells
    allowed = (
        SpatialAllowedRegion()
        if not allowed_cells
        else SpatialAllowedRegion(allowed_cells, unrestricted=False)
    )
    projection_payload = {
        "projection_version": SPATIAL_PUBLIC_PROJECTION_VERSION,
        "source_schema_version": world.source_schema_version,
        "source_generator_version": world.source_generator_version,
        "map_id": world.map_id,
        "map_revision": world.map_revision,
        "grid_content_hash": world.grid.content_hash,
        "forbidden_cells": world.grid.forbidden_cells,
        "allowed_region": allowed,
        "start_pose": world.initial_state.pose,
        "goal_pose": world.goal_pose,
        "reference_path": world.reference_path,
        "maneuver_side": maneuver_side,
        "search_region": region,
    }
    return build_bounded_spatial_request(
        schema_version=SPATIAL_ORACLE_SCHEMA_VERSION,
        map_id=world.map_id,
        map_revision=world.map_revision,
        mission_revision=0,
        static_grid=grid,
        forbidden_cells=world.grid.forbidden_cells,
        allowed_region=allowed,
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        start_pose=world.initial_state.pose,
        rejoin_goal=SpatialRejoinGoal(world.goal_pose),
        reference_segment=SpatialReferenceSegment(reference_start, reference_end),
        maneuver_side=maneuver_side,
        search_region=region,
        lattice_config=lattice_config or SpatialLatticeConfig(),
        source_projection_hash=canonical_content_hash(projection_payload),
    )


def _reference_bounded_cells(
    grid: GridMap,
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    *,
    margin_m: float,
) -> SpatialSearchRegion:
    margin_cells = ceil(margin_m / grid.resolution_m)
    start_cell = grid.world_to_cell(Pose2D(start_x, start_y))
    end_cell = grid.world_to_cell(Pose2D(end_x, end_y))
    min_x = max(0, min(start_cell[0], end_cell[0]) - margin_cells)
    max_x = min(grid.width - 1, max(start_cell[0], end_cell[0]) + margin_cells)
    min_y = max(0, min(start_cell[1], end_cell[1]) - margin_cells)
    max_y = min(grid.height - 1, max(start_cell[1], end_cell[1]) + margin_cells)
    return SpatialSearchRegion(
        tuple((x, y) for y in range(min_y, max_y + 1) for x in range(min_x, max_x + 1))
    )
