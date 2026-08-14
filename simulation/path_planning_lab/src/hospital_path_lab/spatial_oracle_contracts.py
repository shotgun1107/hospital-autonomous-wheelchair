"""R3 bounded 공간 oracle의 label-free 입출력 계약.

이 모듈은 정적 지도와 가상 차체만 다룬다. Actor, 관측, controller, corpus 정답과 hidden
수명주기를 입력으로 받지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from hashlib import sha256
from math import isfinite, pi
from re import fullmatch

import numpy as np

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.grid import GridMap
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile

SPATIAL_ORACLE_SCHEMA_VERSION = "bounded-spatial-oracle-request-v1"
SPATIAL_ORACLE_RESULT_SCHEMA_VERSION = "bounded-spatial-oracle-result-v1"
SPATIAL_ORACLE_VERSION = "bounded-spatial-oracle-v1"
SPATIAL_VALIDATOR_VERSION = "bounded-spatial-validator-v1"
SPATIAL_LATTICE_CONFIG_VERSION = "bounded-spatial-lattice-v1"
SPATIAL_POSITION_TOLERANCE_M = 0.05
SPATIAL_HEADING_TOLERANCE_RAD = pi / 18.0
SPATIAL_MINIMUM_SIDE_EXCURSION_M = 0.10
SPATIAL_TRANSLATION_SWEEP_STEP_M = 0.005
SPATIAL_ROTATION_SWEEP_STEP_RAD = pi / 360.0
SPATIAL_COMPARISON_TOLERANCE_M = 1e-9


class SpatialOracleStatus(StrEnum):
    SPATIALLY_FEASIBLE = "spatially_feasible"
    SPATIALLY_INFEASIBLE = "spatially_infeasible"
    RESOURCE_LIMIT = "resource_limit"
    INVALID_INPUT = "invalid_input"


class ManeuverSide(StrEnum):
    LEFT = "left"
    RIGHT = "right"
    UNSPECIFIED = "unspecified"


class SpatialPrimitiveKind(StrEnum):
    ANCHOR_CONNECTOR = "anchor_connector"
    FORWARD_ONE_TRANSLATION = "forward_one_translation"
    REVERSE_ONE_TRANSLATION = "reverse_one_translation"
    ROTATE_LEFT_45 = "rotate_left_45"
    ROTATE_RIGHT_45 = "rotate_right_45"


@dataclass(frozen=True, slots=True)
class SpatialRejoinGoal:
    pose: Pose2D
    position_tolerance_m: float = SPATIAL_POSITION_TOLERANCE_M
    heading_tolerance_rad: float = SPATIAL_HEADING_TOLERANCE_RAD
    require_stopped: bool = True
    minimum_side_excursion_m: float = SPATIAL_MINIMUM_SIDE_EXCURSION_M

    def __post_init__(self) -> None:
        _require_finite_pose(self.pose, "rejoin goal")
        _require_exact_float(
            self.position_tolerance_m,
            SPATIAL_POSITION_TOLERANCE_M,
            "position_tolerance_m",
        )
        _require_exact_float(
            self.heading_tolerance_rad,
            SPATIAL_HEADING_TOLERANCE_RAD,
            "heading_tolerance_rad",
        )
        if self.require_stopped is not True:
            raise ValueError("R3 v1 rejoin goal must require the abstract stopped marker")
        _require_exact_float(
            self.minimum_side_excursion_m,
            SPATIAL_MINIMUM_SIDE_EXCURSION_M,
            "minimum_side_excursion_m",
        )


@dataclass(frozen=True, slots=True)
class SpatialReferenceSegment:
    start: Pose2D
    end: Pose2D
    progress_start_m: float = 0.0
    progress_end_m: float | None = None

    def __post_init__(self) -> None:
        _require_finite_pose(self.start, "reference start")
        _require_finite_pose(self.end, "reference end")
        if not isfinite(self.progress_start_m) or self.progress_start_m < 0.0:
            raise ValueError("reference progress_start_m must be finite and non-negative")
        length = float(np.hypot(self.end.x - self.start.x, self.end.y - self.start.y))
        if length <= 0.0:
            raise ValueError("reference segment must have positive length")
        expected_end = self.progress_start_m + length
        if self.progress_end_m is None:
            object.__setattr__(self, "progress_end_m", expected_end)
        elif not isfinite(self.progress_end_m) or abs(self.progress_end_m - expected_end) > 1e-9:
            raise ValueError("reference progress range must equal the segment metric length")

    @property
    def tangent(self) -> tuple[float, float]:
        length = self.progress_end_m - self.progress_start_m  # type: ignore[operator]
        return (self.end.x - self.start.x) / length, (self.end.y - self.start.y) / length

    def signed_offset(self, pose: Pose2D) -> float:
        tangent_x, tangent_y = self.tangent
        delta_x = pose.x - self.start.x
        delta_y = pose.y - self.start.y
        return tangent_x * delta_y - tangent_y * delta_x


@dataclass(frozen=True, slots=True)
class SpatialSearchRegion:
    cells: tuple[tuple[int, int], ...]

    def __post_init__(self) -> None:
        normalized = _normalize_cells(self.cells, "search region")
        if not normalized:
            raise ValueError("search region must contain at least one cell")
        object.__setattr__(self, "cells", normalized)

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class SpatialAllowedRegion:
    cells: tuple[tuple[int, int], ...] = ()
    unrestricted: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.unrestricted, bool):
            raise TypeError("unrestricted must be a bool")
        normalized = _normalize_cells(self.cells, "allowed region")
        if self.unrestricted and normalized:
            raise ValueError("unrestricted allowed region cannot also list cells")
        if not self.unrestricted and not normalized:
            raise ValueError("restricted allowed region must contain cells")
        object.__setattr__(self, "cells", normalized)

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class SpatialLatticeConfig:
    config_version: str = SPATIAL_LATTICE_CONFIG_VERSION
    axis_translation_cells: int = 5
    diagonal_translation_cells: int = 4
    heading_bin_count: int = 8
    allow_forward: bool = True
    allow_reverse: bool = True
    allow_in_place_rotation: bool = True
    allow_combined_arc: bool = False
    reverse_cost_multiplier: float = 1.25
    translation_sweep_step_m: float = SPATIAL_TRANSLATION_SWEEP_STEP_M
    rotation_sweep_step_rad: float = SPATIAL_ROTATION_SWEEP_STEP_RAD
    max_expanded_states: int = 250_000
    max_generated_edges: int = 2_000_000
    max_open_states: int = 250_000

    def __post_init__(self) -> None:
        for name in (
            "axis_translation_cells",
            "diagonal_translation_cells",
            "heading_bin_count",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an exact integer")
        for name in (
            "allow_forward",
            "allow_reverse",
            "allow_in_place_rotation",
            "allow_combined_arc",
        ):
            if not isinstance(getattr(self, name), bool):
                raise TypeError(f"{name} must be a bool")
        expected = (
            SPATIAL_LATTICE_CONFIG_VERSION,
            5,
            4,
            8,
            True,
            True,
            True,
            False,
            1.25,
            SPATIAL_TRANSLATION_SWEEP_STEP_M,
            SPATIAL_ROTATION_SWEEP_STEP_RAD,
        )
        actual = (
            self.config_version,
            self.axis_translation_cells,
            self.diagonal_translation_cells,
            self.heading_bin_count,
            self.allow_forward,
            self.allow_reverse,
            self.allow_in_place_rotation,
            self.allow_combined_arc,
            self.reverse_cost_multiplier,
            self.translation_sweep_step_m,
            self.rotation_sweep_step_rad,
        )
        if actual != expected:
            raise ValueError("R3 v1 lattice geometry and primitive contract is frozen")
        for name in ("max_expanded_states", "max_generated_edges", "max_open_states"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive exact integer")

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True, order=True)
class SpatialLatticeState:
    x_cell: int
    y_cell: int
    heading_index: int
    required_excursion_reached: bool = False

    def __post_init__(self) -> None:
        for name in ("x_cell", "y_cell", "heading_index"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise TypeError(f"{name} must be an exact integer")
        if not 0 <= self.heading_index < 8:
            raise ValueError("heading_index must be in [0, 8)")
        if not isinstance(self.required_excursion_reached, bool):
            raise TypeError("required_excursion_reached must be a bool")


@dataclass(frozen=True, slots=True)
class SpatialPrimitive:
    kind: SpatialPrimitiveKind
    start_pose: Pose2D
    end_pose: Pose2D
    start_state: SpatialLatticeState | None
    end_state: SpatialLatticeState | None

    def __post_init__(self) -> None:
        if not isinstance(self.kind, SpatialPrimitiveKind):
            raise TypeError("kind must be a SpatialPrimitiveKind")
        _require_finite_pose(self.start_pose, "primitive start")
        _require_finite_pose(self.end_pose, "primitive end")
        if self.kind is not SpatialPrimitiveKind.ANCHOR_CONNECTOR and (
            self.start_state is None or self.end_state is None
        ):
            raise ValueError("lattice primitive must bind both endpoint states")


@dataclass(frozen=True, slots=True)
class BoundedSpatialOracleRequest:
    schema_version: str
    map_id: str
    map_revision: int
    mission_revision: int
    static_grid: GridMap
    forbidden_cells: tuple[tuple[int, int], ...]
    allowed_region: SpatialAllowedRegion
    vehicle_profile: VehicleProfile
    start_pose: Pose2D
    rejoin_goal: SpatialRejoinGoal
    reference_segment: SpatialReferenceSegment
    maneuver_side: ManeuverSide
    search_region: SpatialSearchRegion
    lattice_config: SpatialLatticeConfig
    source_projection_hash: str
    request_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != SPATIAL_ORACLE_SCHEMA_VERSION:
            raise ValueError("unsupported spatial oracle request schema")
        if not self.map_id:
            raise ValueError("map_id must not be empty")
        for name in ("map_revision", "mission_revision"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative exact integer")
        if not isinstance(self.static_grid, GridMap):
            raise TypeError("static_grid must be a GridMap")
        if not isinstance(self.allowed_region, SpatialAllowedRegion):
            raise TypeError("allowed_region must be a SpatialAllowedRegion")
        if not isinstance(self.vehicle_profile, VehicleProfile):
            raise TypeError("vehicle_profile must be a VehicleProfile")
        _require_finite_pose(self.start_pose, "start pose")
        if not isinstance(self.rejoin_goal, SpatialRejoinGoal):
            raise TypeError("rejoin_goal must be a SpatialRejoinGoal")
        if not isinstance(self.reference_segment, SpatialReferenceSegment):
            raise TypeError("reference_segment must be a SpatialReferenceSegment")
        if not isinstance(self.maneuver_side, ManeuverSide):
            raise TypeError("maneuver_side must be a ManeuverSide")
        if not isinstance(self.search_region, SpatialSearchRegion):
            raise TypeError("search_region must be a SpatialSearchRegion")
        if not isinstance(self.lattice_config, SpatialLatticeConfig):
            raise TypeError("lattice_config must be a SpatialLatticeConfig")
        forbidden = _normalize_cells(self.forbidden_cells, "forbidden cells")
        object.__setattr__(self, "forbidden_cells", forbidden)
        _require_sha256(self.source_projection_hash, "source_projection_hash")
        if self.request_content_hash:
            _require_sha256(self.request_content_hash, "request_content_hash")
        else:
            object.__setattr__(self, "request_content_hash", self.expected_content_hash)

    @property
    def grid_content_hash(self) -> str:
        return spatial_grid_content_hash(self.static_grid)

    @property
    def vehicle_profile_hash(self) -> str:
        return canonical_content_hash(self.vehicle_profile)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "map_id": self.map_id,
                "map_revision": self.map_revision,
                "mission_revision": self.mission_revision,
                "grid_content_hash": self.grid_content_hash,
                "forbidden_cells": self.forbidden_cells,
                "allowed_region": self.allowed_region,
                "vehicle_profile": self.vehicle_profile,
                "start_pose": self.start_pose,
                "rejoin_goal": self.rejoin_goal,
                "reference_segment": self.reference_segment,
                "maneuver_side": self.maneuver_side,
                "search_region": self.search_region,
                "lattice_config": self.lattice_config,
                "source_projection_hash": self.source_projection_hash,
            }
        )

    def integrity_failure(self) -> str | None:
        grid = self.static_grid
        if not all(
            isfinite(value)
            for value in (grid.resolution_m, grid.origin_x_m, grid.origin_y_m)
        ) or grid.resolution_m <= 0.0:
            return "invalid_grid_geometry"
        all_cells = set(self.forbidden_cells)
        all_cells.update(self.search_region.cells)
        all_cells.update(self.allowed_region.cells)
        if any(not grid.in_bounds(cell) for cell in all_cells):
            return "region_cell_out_of_bounds"
        if grid.world_to_cell(self.start_pose) not in self.search_region.cells:
            return "search_region_excludes_start"
        if grid.world_to_cell(self.rejoin_goal.pose) not in self.search_region.cells:
            return "search_region_excludes_goal"
        if self.vehicle_profile != VIRTUAL_DOLL_WHEELCHAIR_V0_1:
            return "unsupported_vehicle_profile"
        if self.request_content_hash != self.expected_content_hash:
            return "request_content_hash_mismatch"
        return None


@dataclass(frozen=True, slots=True)
class SpatialOracleValidation:
    validator_version: str
    passed: bool
    failure_codes: tuple[str, ...]
    request_content_hash: str
    path_content_hash: str
    minimum_clearance_m: float | None
    minimum_physical_clearance_m: float | None
    minimum_forbidden_clearance_m: float | None
    minimum_allowed_boundary_clearance_m: float | None
    path_length_m: float
    reverse_length_m: float
    rotation_count: int
    maximum_signed_side_excursion_m: float
    validation_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.validator_version != SPATIAL_VALIDATOR_VERSION:
            raise ValueError("unsupported spatial validator version")
        if not isinstance(self.passed, bool):
            raise TypeError("passed must be a bool")
        failures = tuple(sorted(set(self.failure_codes)))
        if self.passed == bool(failures):
            raise ValueError(
                "passed validation must have no failures and failed validation must have one"
            )
        object.__setattr__(self, "failure_codes", failures)
        _require_sha256(self.request_content_hash, "request_content_hash")
        _require_sha256(self.path_content_hash, "path_content_hash")
        for name in ("path_length_m", "reverse_length_m", "maximum_signed_side_excursion_m"):
            value = getattr(self, name)
            if not isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if (
            isinstance(self.rotation_count, bool)
            or not isinstance(self.rotation_count, int)
            or self.rotation_count < 0
        ):
            raise ValueError("rotation_count must be a non-negative exact integer")
        if self.validation_content_hash:
            _require_sha256(self.validation_content_hash, "validation_content_hash")
            if self.validation_content_hash != self.expected_content_hash:
                raise ValueError("validation_content_hash mismatch")
        else:
            object.__setattr__(self, "validation_content_hash", self.expected_content_hash)
        if self.passed:
            clearances = (
                self.minimum_clearance_m,
                self.minimum_physical_clearance_m,
                self.minimum_forbidden_clearance_m,
                self.minimum_allowed_boundary_clearance_m,
            )
            if any(value is None or not isfinite(value) for value in clearances):
                raise ValueError("passed validation requires finite clearance metrics")
            if (
                self.minimum_clearance_m + SPATIAL_COMPARISON_TOLERANCE_M
                < VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m
            ):
                raise ValueError("passed validation cannot violate frozen minimum clearance")
            if (
                self.maximum_signed_side_excursion_m + SPATIAL_COMPARISON_TOLERANCE_M
                < SPATIAL_MINIMUM_SIDE_EXCURSION_M
            ):
                raise ValueError("passed validation requires the frozen side excursion")

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "validator_version": self.validator_version,
                "passed": self.passed,
                "failure_codes": self.failure_codes,
                "request_content_hash": self.request_content_hash,
                "path_content_hash": self.path_content_hash,
                "minimum_clearance_m": self.minimum_clearance_m,
                "minimum_physical_clearance_m": self.minimum_physical_clearance_m,
                "minimum_forbidden_clearance_m": self.minimum_forbidden_clearance_m,
                "minimum_allowed_boundary_clearance_m": self.minimum_allowed_boundary_clearance_m,
                "path_length_m": self.path_length_m,
                "reverse_length_m": self.reverse_length_m,
                "rotation_count": self.rotation_count,
                "maximum_signed_side_excursion_m": self.maximum_signed_side_excursion_m,
            }
        )


@dataclass(frozen=True, slots=True)
class BoundedSpatialOracleResult:
    schema_version: str
    oracle_version: str
    status: SpatialOracleStatus
    termination_reason: str
    request_content_hash: str
    map_id: str
    map_revision: int
    mission_revision: int
    grid_content_hash: str
    vehicle_profile_hash: str
    search_region_hash: str
    lattice_config_hash: str
    path: tuple[Pose2D, ...]
    primitive_sequence: tuple[SpatialPrimitive, ...]
    path_length_m: float | None
    reverse_length_m: float | None
    rotation_count: int | None
    minimum_clearance_m: float | None
    minimum_physical_clearance_m: float | None
    minimum_forbidden_clearance_m: float | None
    minimum_allowed_boundary_clearance_m: float | None
    generated_edges: int
    expanded_states: int
    peak_open_states: int
    exhaustive: bool
    validation: SpatialOracleValidation | None
    limitations: tuple[str, ...]
    elapsed_nonqualification_ns: int
    semantic_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != SPATIAL_ORACLE_RESULT_SCHEMA_VERSION:
            raise ValueError("unsupported spatial oracle result schema")
        if self.oracle_version != SPATIAL_ORACLE_VERSION:
            raise ValueError("unsupported spatial oracle version")
        if not isinstance(self.status, SpatialOracleStatus):
            raise TypeError("status must be a SpatialOracleStatus")
        if not self.termination_reason:
            raise ValueError("termination_reason must not be empty")
        if not self.map_id:
            raise ValueError("map_id must not be empty")
        for name in ("map_revision", "mission_revision"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative exact integer")
        for name in (
            "request_content_hash",
            "grid_content_hash",
            "vehicle_profile_hash",
            "search_region_hash",
            "lattice_config_hash",
        ):
            _require_sha256(getattr(self, name), name)
        for name in (
            "generated_edges",
            "expanded_states",
            "peak_open_states",
            "elapsed_nonqualification_ns",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative exact integer")
        if not isinstance(self.exhaustive, bool):
            raise TypeError("exhaustive must be a bool")
        object.__setattr__(self, "path", tuple(self.path))
        object.__setattr__(self, "primitive_sequence", tuple(self.primitive_sequence))
        object.__setattr__(self, "limitations", tuple(sorted(set(self.limitations))))
        self._validate_status_contract()
        if self.semantic_content_hash:
            _require_sha256(self.semantic_content_hash, "semantic_content_hash")
            if self.semantic_content_hash != self.expected_semantic_hash:
                raise ValueError("semantic_content_hash mismatch")
        else:
            object.__setattr__(self, "semantic_content_hash", self.expected_semantic_hash)

    def _validate_status_contract(self) -> None:
        feasible = self.status is SpatialOracleStatus.SPATIALLY_FEASIBLE
        if feasible:
            if not self.path or self.validation is None or not self.validation.passed:
                raise ValueError(
                    "feasible result requires a non-empty independently validated path"
                )
            if len(self.primitive_sequence) != max(0, len(self.path) - 1):
                raise ValueError("feasible path and primitive sequence length mismatch")
            if self.validation.request_content_hash != self.request_content_hash:
                raise ValueError("validation request hash mismatch")
            if self.validation.path_content_hash != spatial_path_content_hash(
                self.path, self.primitive_sequence
            ):
                raise ValueError("validation path hash mismatch")
            validation_metrics = (
                self.path_length_m,
                self.reverse_length_m,
                self.rotation_count,
                self.minimum_clearance_m,
                self.minimum_physical_clearance_m,
                self.minimum_forbidden_clearance_m,
                self.minimum_allowed_boundary_clearance_m,
            )
            expected_metrics = (
                self.validation.path_length_m,
                self.validation.reverse_length_m,
                self.validation.rotation_count,
                self.validation.minimum_clearance_m,
                self.validation.minimum_physical_clearance_m,
                self.validation.minimum_forbidden_clearance_m,
                self.validation.minimum_allowed_boundary_clearance_m,
            )
            if validation_metrics != expected_metrics:
                raise ValueError("feasible result metrics must equal independent validation")
        else:
            if self.path or self.primitive_sequence or self.validation is not None:
                raise ValueError("non-feasible result cannot carry a selected path or validation")
            optional_metrics = (
                self.path_length_m,
                self.reverse_length_m,
                self.rotation_count,
                self.minimum_clearance_m,
                self.minimum_physical_clearance_m,
                self.minimum_forbidden_clearance_m,
                self.minimum_allowed_boundary_clearance_m,
            )
            if any(value is not None for value in optional_metrics):
                raise ValueError("non-feasible result cannot carry selected path metrics")
        if self.status is SpatialOracleStatus.RESOURCE_LIMIT and self.exhaustive:
            raise ValueError("resource-limit result cannot be exhaustive")
        if self.status is SpatialOracleStatus.INVALID_INPUT and (
            self.exhaustive
            or any((self.generated_edges, self.expanded_states, self.peak_open_states))
        ):
            raise ValueError("invalid-input result cannot contain search work")
        analytic_reasons = {
            "start_footprint_unsafe",
            "goal_footprint_unsafe",
            "analytic_cross_section_blocked",
        }
        if (
            self.status is SpatialOracleStatus.SPATIALLY_INFEASIBLE
            and not self.exhaustive
            and self.termination_reason not in analytic_reasons
        ):
            raise ValueError(
                "non-exhaustive infeasible result needs an independent analytic reason"
            )

    @property
    def expected_semantic_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "oracle_version": self.oracle_version,
                "status": self.status,
                "termination_reason": self.termination_reason,
                "request_content_hash": self.request_content_hash,
                "map_id": self.map_id,
                "map_revision": self.map_revision,
                "mission_revision": self.mission_revision,
                "grid_content_hash": self.grid_content_hash,
                "vehicle_profile_hash": self.vehicle_profile_hash,
                "search_region_hash": self.search_region_hash,
                "lattice_config_hash": self.lattice_config_hash,
                "path": self.path,
                "primitive_sequence": self.primitive_sequence,
                "path_length_m": self.path_length_m,
                "reverse_length_m": self.reverse_length_m,
                "rotation_count": self.rotation_count,
                "minimum_clearance_m": self.minimum_clearance_m,
                "minimum_physical_clearance_m": self.minimum_physical_clearance_m,
                "minimum_forbidden_clearance_m": self.minimum_forbidden_clearance_m,
                "minimum_allowed_boundary_clearance_m": self.minimum_allowed_boundary_clearance_m,
                "generated_edges": self.generated_edges,
                "expanded_states": self.expanded_states,
                "peak_open_states": self.peak_open_states,
                "exhaustive": self.exhaustive,
                "validation": self.validation,
                "limitations": self.limitations,
            }
        )


def build_bounded_spatial_request(**kwargs: object) -> BoundedSpatialOracleRequest:
    """정본 content hash가 결박된 request를 만든다."""

    return BoundedSpatialOracleRequest(**kwargs)  # type: ignore[arg-type]


def spatial_grid_content_hash(grid: GridMap) -> str:
    occupancy = np.ascontiguousarray(grid.occupancy, dtype=np.bool_)
    occupancy_hash = sha256(occupancy.tobytes(order="C")).hexdigest()
    return canonical_content_hash(
        {
            "width": grid.width,
            "height": grid.height,
            "resolution_m": grid.resolution_m,
            "origin_x_m": grid.origin_x_m,
            "origin_y_m": grid.origin_y_m,
            "occupancy_sha256": occupancy_hash,
        }
    )


def spatial_path_content_hash(
    path: tuple[Pose2D, ...], primitive_sequence: tuple[SpatialPrimitive, ...]
) -> str:
    return canonical_content_hash({"path": path, "primitive_sequence": primitive_sequence})


def _normalize_cells(
    cells: tuple[tuple[int, int], ...], field_name: str
) -> tuple[tuple[int, int], ...]:
    normalized = tuple(sorted(set(cells)))
    if any(
        not isinstance(cell, tuple)
        or len(cell) != 2
        or any(isinstance(value, bool) or not isinstance(value, int) for value in cell)
        for cell in normalized
    ):
        raise TypeError(f"{field_name} must contain exact integer (x, y) tuples")
    return normalized


def _require_finite_pose(pose: Pose2D, field_name: str) -> None:
    if not isinstance(pose, Pose2D) or not all(
        isfinite(value) for value in (pose.x, pose.y, pose.yaw)
    ):
        raise ValueError(f"{field_name} must be a finite Pose2D")


def _require_exact_float(value: float, expected: float, field_name: str) -> None:
    if not isfinite(value) or value != expected:
        raise ValueError(f"{field_name} must equal the frozen R3 v1 value {expected}")


def _require_sha256(value: str, field_name: str) -> None:
    if not isinstance(value, str) or fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{field_name} must be a lowercase SHA-256 hex digest")
