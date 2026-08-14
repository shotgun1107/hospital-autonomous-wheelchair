from __future__ import annotations

from dataclasses import replace
from hashlib import sha256

import numpy as np
import pytest

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.grid import GridMap
from hospital_path_lab.spatial_oracle_contracts import (
    SPATIAL_ORACLE_SCHEMA_VERSION,
    ManeuverSide,
    SpatialAllowedRegion,
    SpatialLatticeConfig,
    SpatialOracleStatus,
    SpatialReferenceSegment,
    SpatialRejoinGoal,
    SpatialSearchRegion,
    build_bounded_spatial_request,
)
from hospital_path_lab.spatial_oracle_lattice import search_bounded_spatial_oracle
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _request(
    *,
    side: ManeuverSide = ManeuverSide.LEFT,
    occupancy: np.ndarray | None = None,
    config: SpatialLatticeConfig | None = None,
):
    occupancy = (
        np.zeros((100, 120), dtype=np.bool_) if occupancy is None else occupancy
    )
    grid = GridMap(occupancy, resolution_m=0.02)
    start = grid.cell_to_pose((30, 50))
    goal = grid.cell_to_pose((75, 50))
    region = tuple((x, y) for y in range(12, 89) for x in range(12, 108))
    return build_bounded_spatial_request(
        schema_version=SPATIAL_ORACLE_SCHEMA_VERSION,
        map_id="lattice-map",
        map_revision=4,
        mission_revision=9,
        static_grid=grid,
        forbidden_cells=(),
        allowed_region=SpatialAllowedRegion(),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        start_pose=Pose2D(start.x, start.y, 0.0),
        rejoin_goal=SpatialRejoinGoal(Pose2D(goal.x, goal.y, 0.0)),
        reference_segment=SpatialReferenceSegment(
            Pose2D(start.x, start.y, 0.0), Pose2D(goal.x, goal.y, 0.0)
        ),
        maneuver_side=side,
        search_region=SpatialSearchRegion(region),
        lattice_config=config or SpatialLatticeConfig(),
        source_projection_hash=sha256(b"lattice-source").hexdigest(),
    )


@pytest.mark.parametrize("side", (ManeuverSide.LEFT, ManeuverSide.RIGHT))
def test_open_map_finds_required_side_excursion_and_independent_rejoin(
    side: ManeuverSide,
) -> None:
    result = search_bounded_spatial_oracle(_request(side=side))

    assert result.status is SpatialOracleStatus.SPATIALLY_FEASIBLE
    assert result.validation is not None and result.validation.passed
    assert result.validation.maximum_signed_side_excursion_m >= 0.10 - 1e-9
    assert result.path[0].y == pytest.approx(result.path[-1].y)
    assert result.minimum_clearance_m is not None
    assert result.minimum_clearance_m >= 0.08


def test_unspecified_runs_both_sides_and_returns_deterministic_semantics() -> None:
    request = _request(side=ManeuverSide.UNSPECIFIED)

    first = search_bounded_spatial_oracle(request)
    second = search_bounded_spatial_oracle(request)

    assert first.status is SpatialOracleStatus.SPATIALLY_FEASIBLE
    assert second.semantic_content_hash == first.semantic_content_hash
    assert second.path == first.path
    assert second.generated_edges == first.generated_edges
    assert second.expanded_states == first.expanded_states


def test_resource_limit_is_not_serialized_as_spatial_infeasibility() -> None:
    config = SpatialLatticeConfig(
        max_expanded_states=1,
        max_generated_edges=2_000_000,
        max_open_states=250_000,
    )

    result = search_bounded_spatial_oracle(_request(config=config))

    assert result.status is SpatialOracleStatus.RESOURCE_LIMIT
    assert not result.exhaustive
    assert result.path == ()
    assert result.validation is None


def test_request_provenance_tamper_fails_before_search_work() -> None:
    request = _request()
    tampered = replace(request, mission_revision=request.mission_revision + 1)

    result = search_bounded_spatial_oracle(tampered)

    assert result.status is SpatialOracleStatus.INVALID_INPUT
    assert result.termination_reason == "request_content_hash_mismatch"
    assert result.generated_edges == 0
    assert result.expanded_states == 0


def test_full_static_wall_exhausts_only_the_frozen_lattice() -> None:
    occupancy = np.zeros((100, 120), dtype=np.bool_)
    occupancy[12:89, 48:53] = True

    result = search_bounded_spatial_oracle(_request(occupancy=occupancy))

    assert result.status is SpatialOracleStatus.SPATIALLY_INFEASIBLE
    assert result.termination_reason == "bounded_lattice_exhausted"
    assert result.exhaustive
    assert "orthogonal_lattice_motion_only" in result.limitations


def test_exact_resource_counts_finish_but_one_less_is_resource_limit() -> None:
    occupancy = np.zeros((100, 120), dtype=np.bool_)
    occupancy[12:89, 48:53] = True
    baseline = search_bounded_spatial_oracle(_request(occupancy=occupancy))
    exact = SpatialLatticeConfig(
        max_expanded_states=baseline.expanded_states,
        max_generated_edges=baseline.generated_edges,
        max_open_states=baseline.peak_open_states,
    )

    exact_result = search_bounded_spatial_oracle(
        _request(occupancy=occupancy, config=exact)
    )
    one_less = replace(exact, max_expanded_states=exact.max_expanded_states - 1)
    limited_result = search_bounded_spatial_oracle(
        _request(occupancy=occupancy, config=one_less)
    )

    assert exact_result.status is SpatialOracleStatus.SPATIALLY_INFEASIBLE
    assert exact_result.exhaustive
    assert limited_result.status is SpatialOracleStatus.RESOURCE_LIMIT
    assert not limited_result.exhaustive


def test_right_anchor_uses_safe_adjacent_cell_when_pose_is_on_grid_line() -> None:
    request = _request(side=ManeuverSide.RIGHT)
    start = Pose2D(request.start_pose.x, 1.00, 0.0)
    goal = Pose2D(request.rejoin_goal.pose.x, 1.00, 0.0)
    aligned = build_bounded_spatial_request(
        schema_version=request.schema_version,
        map_id=request.map_id,
        map_revision=request.map_revision,
        mission_revision=request.mission_revision,
        static_grid=request.static_grid,
        forbidden_cells=request.forbidden_cells,
        allowed_region=request.allowed_region,
        vehicle_profile=request.vehicle_profile,
        start_pose=start,
        rejoin_goal=SpatialRejoinGoal(goal),
        reference_segment=SpatialReferenceSegment(start, goal),
        maneuver_side=ManeuverSide.RIGHT,
        search_region=request.search_region,
        lattice_config=request.lattice_config,
        source_projection_hash=sha256(b"grid-line-source").hexdigest(),
    )

    result = search_bounded_spatial_oracle(aligned)

    assert result.status is SpatialOracleStatus.SPATIALLY_FEASIBLE
    assert result.validation is not None and result.validation.passed


def test_search_source_has_no_actor_category_or_hidden_dependency() -> None:
    import ast
    from pathlib import Path

    source_path = (
        Path(__file__).parents[1]
        / "src"
        / "hospital_path_lab"
        / "spatial_oracle_lattice.py"
    )
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    forbidden = ("dynamic_corpus", "expectation_category", "oracle_spec", "hidden")
    imported = " ".join(
        node.module or ""
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom)
    )
    identifiers = " ".join(
        node.id for node in ast.walk(tree) if isinstance(node, ast.Name)
    )

    assert all(term not in imported for term in forbidden)
    assert all(term not in identifiers for term in forbidden)
