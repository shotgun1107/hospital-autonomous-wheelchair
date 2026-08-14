"""R3 bounded 공간 경로의 search 독립 검증기."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, ceil, cos, hypot, pi, sin

import numpy as np

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.grid import GridMap
from hospital_path_lab.spatial_oracle_contracts import (
    SPATIAL_COMPARISON_TOLERANCE_M,
    SPATIAL_VALIDATOR_VERSION,
    BoundedSpatialOracleRequest,
    ManeuverSide,
    SpatialOracleValidation,
    SpatialPrimitive,
    SpatialPrimitiveKind,
    spatial_path_content_hash,
)

_FORWARD_OFFSETS = (
    (5, 0),
    (4, 4),
    (0, 5),
    (-4, 4),
    (-5, 0),
    (-4, -4),
    (0, -5),
    (4, -4),
)


@dataclass(frozen=True, slots=True)
class SpatialPoseClearance:
    physical_m: float
    forbidden_m: float
    allowed_boundary_m: float
    search_region_m: float

    @property
    def safety_minimum_m(self) -> float:
        return min(self.physical_m, self.forbidden_m, self.allowed_boundary_m)


class SpatialGeometryEvaluator:
    """원본 정적 geometry에서 pose와 primitive clearance를 계산한다."""

    def __init__(self, request: BoundedSpatialOracleRequest) -> None:
        self.request = request
        grid = request.static_grid
        profile = request.vehicle_profile
        self._physical = CollisionChecker(grid, profile)
        self._forbidden = CollisionChecker(
            grid,
            profile,
            forbidden_cells=frozenset(request.forbidden_cells),
        )
        self._allowed = (
            None
            if request.allowed_region.unrestricted
            else CollisionChecker(_complement_grid(grid, request.allowed_region.cells), profile)
        )
        self._search_region = CollisionChecker(
            _complement_grid(grid, request.search_region.cells), profile
        )

    def pose_clearance(self, pose: Pose2D) -> SpatialPoseClearance:
        limit = 1.0
        physical = self._physical.clearance(pose, limit_m=limit)
        forbidden = self._forbidden.forbidden_clearance(pose, limit_m=limit)
        allowed = limit if self._allowed is None else self._allowed.clearance(pose, limit_m=limit)
        search_region = self._search_region.clearance(pose, limit_m=limit)
        return SpatialPoseClearance(
            physical_m=physical,
            forbidden_m=forbidden,
            allowed_boundary_m=allowed,
            search_region_m=search_region,
        )

    def primitive_samples(
        self, primitive: SpatialPrimitive
    ) -> tuple[tuple[Pose2D, ...], float]:
        start = primitive.start_pose
        end = primitive.end_pose
        distance = hypot(end.x - start.x, end.y - start.y)
        yaw_delta = _shortest_angle(end.yaw - start.yaw)
        config = self.request.lattice_config
        interval_count = max(
            1,
            ceil(distance / config.translation_sweep_step_m),
            ceil(abs(yaw_delta) / config.rotation_sweep_step_rad),
        )
        samples = tuple(
            Pose2D(
                x=start.x + (end.x - start.x) * index / interval_count,
                y=start.y + (end.y - start.y) * index / interval_count,
                yaw=_normalize_angle(start.yaw + yaw_delta * index / interval_count),
            )
            for index in range(interval_count + 1)
        )
        interval_translation = distance / interval_count
        interval_rotation = abs(yaw_delta) / interval_count
        half_diagonal = hypot(
            self.request.vehicle_profile.collision_length_m / 2.0,
            self.request.vehicle_profile.collision_width_m / 2.0,
        )
        nearest_sample_motion_bound = 0.5 * (
            interval_translation + half_diagonal * interval_rotation
        )
        return samples, nearest_sample_motion_bound

    def primitive_is_certifiably_safe(
        self,
        samples: tuple[Pose2D, ...],
        gap_bound: float,
    ) -> bool:
        """보수 lower bound만으로 전체 primitive 안전을 증명할 수 있으면 참이다."""

        threshold = self.request.vehicle_profile.minimum_clearance_m + gap_bound
        physical = self._physical.certified_minimum_clearance_lower_bound(samples)
        effective = self._forbidden.certified_minimum_clearance_lower_bound(samples)
        allowed = (
            1.0
            if self._allowed is None
            else self._allowed.certified_minimum_clearance_lower_bound(samples)
        )
        search_region = self._search_region.certified_minimum_clearance_lower_bound(
            samples
        )
        return (
            physical + SPATIAL_COMPARISON_TOLERANCE_M >= threshold
            and effective + SPATIAL_COMPARISON_TOLERANCE_M >= threshold
            and allowed + SPATIAL_COMPARISON_TOLERANCE_M >= threshold
            and search_region > gap_bound
        )


def validate_spatial_oracle_path(
    request: BoundedSpatialOracleRequest,
    path: tuple[Pose2D, ...],
    primitive_sequence: tuple[SpatialPrimitive, ...],
) -> SpatialOracleValidation:
    """search 상태·cache·성공 flag 없이 경로 전체를 다시 검증한다."""

    path = tuple(path)
    primitive_sequence = tuple(primitive_sequence)
    path_hash = spatial_path_content_hash(path, primitive_sequence)
    failures: set[str] = set()
    integrity_failure = request.integrity_failure()
    if integrity_failure is not None:
        failures.add(integrity_failure)

    path_length = 0.0
    reverse_length = 0.0
    rotation_count = 0
    minimum_physical: float | None = None
    minimum_forbidden: float | None = None
    minimum_allowed: float | None = None
    maximum_excursion = 0.0

    if not path:
        failures.add("empty_path")
    if len(primitive_sequence) != max(0, len(path) - 1):
        failures.add("path_primitive_count_mismatch")
    if path and not _poses_close(path[0], request.start_pose):
        failures.add("path_start_mismatch")
    if path and not _goal_satisfied(path[-1], request):
        failures.add("rejoin_goal_mismatch")

    geometry: SpatialGeometryEvaluator | None = None
    if integrity_failure is None:
        geometry = SpatialGeometryEvaluator(request)
    clearance_threshold = request.vehicle_profile.minimum_clearance_m

    sampled_poses: list[Pose2D] = []
    if path and not primitive_sequence:
        sampled_poses.append(path[0])

    for index, primitive in enumerate(primitive_sequence):
        if index + 1 >= len(path):
            break
        if not _poses_close(primitive.start_pose, path[index]) or not _poses_close(
            primitive.end_pose, path[index + 1]
        ):
            failures.add("primitive_endpoint_mismatch")
        failures.update(_primitive_contract_failures(request, primitive, index, len(path)))
        distance = hypot(
            primitive.end_pose.x - primitive.start_pose.x,
            primitive.end_pose.y - primitive.start_pose.y,
        )
        path_length += distance
        if primitive.kind is SpatialPrimitiveKind.REVERSE_ONE_TRANSLATION:
            reverse_length += distance
        if primitive.kind in {
            SpatialPrimitiveKind.ROTATE_LEFT_45,
            SpatialPrimitiveKind.ROTATE_RIGHT_45,
        }:
            rotation_count += 1
        if geometry is not None:
            samples, gap_bound = geometry.primitive_samples(primitive)
            if sampled_poses:
                samples = samples[1:]
            sampled_poses.extend(samples)
            for sample in samples:
                clearance = geometry.pose_clearance(sample)
                physical = clearance.physical_m - gap_bound
                forbidden = clearance.forbidden_m - gap_bound
                allowed = clearance.allowed_boundary_m - gap_bound
                minimum_physical = _optional_min(minimum_physical, physical)
                minimum_forbidden = _optional_min(minimum_forbidden, forbidden)
                minimum_allowed = _optional_min(minimum_allowed, allowed)
                if clearance.search_region_m <= gap_bound:
                    failures.add("search_region_footprint_violation")
                if physical + SPATIAL_COMPARISON_TOLERANCE_M < clearance_threshold:
                    failures.add("physical_clearance_violation")
                if forbidden + SPATIAL_COMPARISON_TOLERANCE_M < clearance_threshold:
                    failures.add("forbidden_clearance_violation")
                if allowed + SPATIAL_COMPARISON_TOLERANCE_M < clearance_threshold:
                    failures.add("allowed_boundary_clearance_violation")

    if geometry is not None and path and not primitive_sequence:
        clearance = geometry.pose_clearance(path[0])
        minimum_physical = clearance.physical_m
        minimum_forbidden = clearance.forbidden_m
        minimum_allowed = clearance.allowed_boundary_m
        if clearance.search_region_m <= 0.0:
            failures.add("search_region_footprint_violation")
        if clearance.physical_m + SPATIAL_COMPARISON_TOLERANCE_M < clearance_threshold:
            failures.add("physical_clearance_violation")
        if clearance.forbidden_m + SPATIAL_COMPARISON_TOLERANCE_M < clearance_threshold:
            failures.add("forbidden_clearance_violation")
        if (
            clearance.allowed_boundary_m + SPATIAL_COMPARISON_TOLERANCE_M
            < clearance_threshold
        ):
            failures.add("allowed_boundary_clearance_violation")

    side_failures, maximum_excursion = _side_failures(request, tuple(sampled_poses or path))
    failures.update(side_failures)
    minimum = _minimum_present(minimum_physical, minimum_forbidden, minimum_allowed)
    return SpatialOracleValidation(
        validator_version=SPATIAL_VALIDATOR_VERSION,
        passed=not failures,
        failure_codes=tuple(failures),
        request_content_hash=request.request_content_hash,
        path_content_hash=path_hash,
        minimum_clearance_m=minimum,
        minimum_physical_clearance_m=minimum_physical,
        minimum_forbidden_clearance_m=minimum_forbidden,
        minimum_allowed_boundary_clearance_m=minimum_allowed,
        path_length_m=path_length,
        reverse_length_m=reverse_length,
        rotation_count=rotation_count,
        maximum_signed_side_excursion_m=maximum_excursion,
    )


def spatial_pose_is_safe(request: BoundedSpatialOracleRequest, pose: Pose2D) -> bool:
    """검색 precheck용 보수 pose 판정. 최종 성공은 독립 validator가 다시 검사한다."""

    evaluator = SpatialGeometryEvaluator(request)
    clearance = evaluator.pose_clearance(pose)
    threshold = request.vehicle_profile.minimum_clearance_m
    return (
        clearance.physical_m + SPATIAL_COMPARISON_TOLERANCE_M >= threshold
        and clearance.forbidden_m + SPATIAL_COMPARISON_TOLERANCE_M >= threshold
        and clearance.allowed_boundary_m + SPATIAL_COMPARISON_TOLERANCE_M >= threshold
        and clearance.search_region_m > 0.0
    )


def spatial_primitive_is_safe(
    request: BoundedSpatialOracleRequest,
    primitive: SpatialPrimitive,
    *,
    evaluator: SpatialGeometryEvaluator | None = None,
) -> bool:
    """검색 edge precheck. 호출자는 최종 path를 독립 validator에 다시 제출해야 한다."""

    checker = evaluator or SpatialGeometryEvaluator(request)
    samples, gap_bound = checker.primitive_samples(primitive)
    if checker.primitive_is_certifiably_safe(samples, gap_bound):
        return True
    threshold = request.vehicle_profile.minimum_clearance_m
    for sample in samples:
        clearance = checker.pose_clearance(sample)
        if clearance.search_region_m <= gap_bound:
            return False
        if clearance.physical_m - gap_bound + SPATIAL_COMPARISON_TOLERANCE_M < threshold:
            return False
        if clearance.forbidden_m - gap_bound + SPATIAL_COMPARISON_TOLERANCE_M < threshold:
            return False
        if clearance.allowed_boundary_m - gap_bound + SPATIAL_COMPARISON_TOLERANCE_M < threshold:
            return False
    return True


def _primitive_contract_failures(
    request: BoundedSpatialOracleRequest,
    primitive: SpatialPrimitive,
    index: int,
    path_size: int,
) -> set[str]:
    failures: set[str] = set()
    kind = primitive.kind
    if kind is SpatialPrimitiveKind.ANCHOR_CONNECTOR:
        if index not in {0, path_size - 2}:
            failures.add("anchor_connector_not_at_path_boundary")
        first_connector = index == 0 and primitive.start_state is None
        last_connector = index == path_size - 2 and primitive.end_state is None
        if first_connector:
            if primitive.end_state is None or not _poses_close(
                primitive.start_pose, request.start_pose
            ):
                failures.add("invalid_start_anchor_connector")
            elif not _adjacent_to_anchor_cell(
                request.static_grid.world_to_cell(request.start_pose),
                (primitive.end_state.x_cell, primitive.end_state.y_cell),
            ):
                failures.add("start_anchor_not_adjacent_lattice_cell")
        elif last_connector:
            if primitive.start_state is None or not _poses_close(
                primitive.end_pose, request.rejoin_goal.pose
            ):
                failures.add("invalid_goal_anchor_connector")
            elif not _adjacent_to_anchor_cell(
                request.static_grid.world_to_cell(request.rejoin_goal.pose),
                (primitive.start_state.x_cell, primitive.start_state.y_cell),
            ):
                failures.add("goal_anchor_not_adjacent_lattice_cell")
        else:
            failures.add("anchor_connector_without_one_lattice_endpoint")
        return failures

    start_state = primitive.start_state
    end_state = primitive.end_state
    if start_state is None or end_state is None:
        return {"lattice_primitive_missing_state"}
    grid = request.static_grid
    if not _state_matches_pose(grid, start_state, primitive.start_pose):
        failures.add("primitive_start_state_pose_mismatch")
    if not _state_matches_pose(grid, end_state, primitive.end_pose):
        failures.add("primitive_end_state_pose_mismatch")
    if start_state.required_excursion_reached and not end_state.required_excursion_reached:
        failures.add("excursion_phase_regressed")

    delta_x = end_state.x_cell - start_state.x_cell
    delta_y = end_state.y_cell - start_state.y_cell
    same_heading = end_state.heading_index == start_state.heading_index
    offset_x, offset_y = _FORWARD_OFFSETS[start_state.heading_index]
    if kind is SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION:
        if not same_heading or (delta_x, delta_y) != (offset_x, offset_y):
            failures.add("invalid_forward_primitive")
    elif kind is SpatialPrimitiveKind.REVERSE_ONE_TRANSLATION:
        if not same_heading or (delta_x, delta_y) != (-offset_x, -offset_y):
            failures.add("invalid_reverse_primitive")
    elif kind is SpatialPrimitiveKind.ROTATE_LEFT_45:
        if (delta_x, delta_y) != (0, 0) or end_state.heading_index != (
            start_state.heading_index + 1
        ) % 8:
            failures.add("invalid_left_rotation_primitive")
    elif kind is SpatialPrimitiveKind.ROTATE_RIGHT_45 and (
        (delta_x, delta_y) != (0, 0)
        or end_state.heading_index != (start_state.heading_index - 1) % 8
    ):
        failures.add("invalid_right_rotation_primitive")
    return failures


def _side_failures(
    request: BoundedSpatialOracleRequest, poses: tuple[Pose2D, ...]
) -> tuple[set[str], float]:
    if not poses:
        return {"required_side_excursion_missing"}, 0.0
    offsets = tuple(request.reference_segment.signed_offset(pose) for pose in poses)
    tolerance = SPATIAL_COMPARISON_TOLERANCE_M
    if request.maneuver_side is ManeuverSide.LEFT:
        maximum = max(0.0, max(offsets))
        failures = {"opposite_side_excursion"} if min(offsets) < -tolerance else set()
    elif request.maneuver_side is ManeuverSide.RIGHT:
        maximum = max(0.0, max(-offset for offset in offsets))
        failures = {"opposite_side_excursion"} if max(offsets) > tolerance else set()
    else:
        left = max(0.0, max(offsets))
        right = max(0.0, max(-offset for offset in offsets))
        maximum = max(left, right)
        failures = set()
        if left > tolerance and right > tolerance:
            failures.add("unspecified_side_switched_during_maneuver")
    if maximum + tolerance < request.rejoin_goal.minimum_side_excursion_m:
        failures.add("required_side_excursion_missing")
    return failures, maximum


def _state_matches_pose(grid: GridMap, state: object, pose: Pose2D) -> bool:
    from hospital_path_lab.spatial_oracle_contracts import SpatialLatticeState

    if not isinstance(state, SpatialLatticeState):
        return False
    expected = grid.cell_to_pose((state.x_cell, state.y_cell))
    expected_yaw = state.heading_index * pi / 4.0
    return (
        hypot(expected.x - pose.x, expected.y - pose.y) <= 1e-9
        and abs(_shortest_angle(expected_yaw - pose.yaw)) <= 1e-9
    )


def _goal_satisfied(pose: Pose2D, request: BoundedSpatialOracleRequest) -> bool:
    goal = request.rejoin_goal
    return (
        hypot(pose.x - goal.pose.x, pose.y - goal.pose.y) <= goal.position_tolerance_m
        and abs(_shortest_angle(pose.yaw - goal.pose.yaw)) <= goal.heading_tolerance_rad
        and goal.require_stopped
    )


def _poses_close(first: Pose2D, second: Pose2D) -> bool:
    return (
        hypot(first.x - second.x, first.y - second.y) <= 1e-9
        and abs(_shortest_angle(first.yaw - second.yaw)) <= 1e-9
    )


def _adjacent_to_anchor_cell(
    anchor_cell: tuple[int, int], lattice_cell: tuple[int, int]
) -> bool:
    return max(
        abs(anchor_cell[0] - lattice_cell[0]),
        abs(anchor_cell[1] - lattice_cell[1]),
    ) <= 1


def _complement_grid(grid: GridMap, cells: tuple[tuple[int, int], ...]) -> GridMap:
    occupancy = np.ones((grid.height, grid.width), dtype=np.bool_)
    for x, y in cells:
        if grid.in_bounds((x, y)):
            occupancy[y, x] = False
    return GridMap(
        occupancy,
        resolution_m=grid.resolution_m,
        origin_x_m=grid.origin_x_m,
        origin_y_m=grid.origin_y_m,
    )


def _normalize_angle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def _shortest_angle(angle: float) -> float:
    return _normalize_angle(angle)


def _optional_min(current: float | None, candidate: float) -> float:
    return candidate if current is None else min(current, candidate)


def _minimum_present(*values: float | None) -> float | None:
    present = tuple(value for value in values if value is not None)
    return min(present) if present else None
