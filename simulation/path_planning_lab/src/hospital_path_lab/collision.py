"""모든 local 후보가 공유하는 가상 차체 footprint 충돌 검사기."""

from __future__ import annotations

from functools import cached_property
from math import ceil, cos, hypot, sin

import numpy as np

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.grid import GridMap, inflate_occupancy
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile

Point = tuple[float, float]
Polygon = tuple[Point, ...]


class CollisionChecker:
    def __init__(
        self,
        grid: GridMap,
        profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        *,
        forbidden_cells: frozenset[tuple[int, int]] = frozenset(),
    ) -> None:
        normalized = frozenset(forbidden_cells)
        if any(not grid.in_bounds(cell) for cell in normalized):
            raise ValueError("forbidden_cells must be inside the grid")
        self.grid = grid
        self.profile = profile
        self.forbidden_cells = normalized
        self._half_width = profile.collision_width_m / 2.0
        self._half_length = profile.collision_length_m / 2.0
        self._half_diagonal = hypot(self._half_width, self._half_length)

    @cached_property
    def _forbidden_occupancy(self) -> np.ndarray:
        occupancy = np.zeros_like(self.grid.occupancy)
        for x, y in self.forbidden_cells:
            occupancy[y, x] = True
        return occupancy

    @cached_property
    def _effective_occupancy(self) -> np.ndarray:
        occupancy = np.array(self.grid.occupancy, copy=True)
        occupancy |= self._forbidden_occupancy
        return occupancy

    @cached_property
    def _has_forbidden_occupancy(self) -> bool:
        return bool(np.any(self._forbidden_occupancy))

    @cached_property
    def _has_effective_occupancy(self) -> bool:
        return bool(np.any(self._effective_occupancy))

    @cached_property
    def collision_grid(self) -> GridMap:
        """임의 yaw의 직사각형을 감싸는 원형 footprint 구성공간."""

        return self._inflated_grid(self._half_diagonal)

    @cached_property
    def configuration_grid(self) -> GridMap:
        """Grid A*용 원형 footprint+최소 여유 구성공간."""

        return self._inflated_grid(
            self._half_diagonal + self.profile.minimum_clearance_m
        )

    @cached_property
    def _center_distance_field_m(self) -> np.ndarray:
        distance = np.where(self._effective_occupancy, 0.0, np.inf)
        diagonal = 2.0**0.5
        for y in range(self.grid.height):
            for x in range(self.grid.width):
                best = distance[y, x]
                if x:
                    best = min(best, distance[y, x - 1] + 1.0)
                if y:
                    best = min(best, distance[y - 1, x] + 1.0)
                if x and y:
                    best = min(best, distance[y - 1, x - 1] + diagonal)
                if x + 1 < self.grid.width and y:
                    best = min(best, distance[y - 1, x + 1] + diagonal)
                distance[y, x] = best
        for y in range(self.grid.height - 1, -1, -1):
            for x in range(self.grid.width - 1, -1, -1):
                best = distance[y, x]
                if x + 1 < self.grid.width:
                    best = min(best, distance[y, x + 1] + 1.0)
                if y + 1 < self.grid.height:
                    best = min(best, distance[y + 1, x] + 1.0)
                if x + 1 < self.grid.width and y + 1 < self.grid.height:
                    best = min(best, distance[y + 1, x + 1] + diagonal)
                if x and y + 1 < self.grid.height:
                    best = min(best, distance[y + 1, x - 1] + diagonal)
                distance[y, x] = best
        return distance * self.grid.resolution_m

    def conservative_path_is_collision_free(self, path: tuple[Pose2D, ...]) -> bool:
        grid = self.collision_grid
        return bool(path) and all(
            not grid.is_occupied(grid.world_to_cell(pose)) for pose in path
        )

    def conservative_clearance(self, pose: Pose2D, *, limit_m: float = 1.0) -> float:
        cell = self.grid.world_to_cell(pose)
        if not self.grid.in_bounds(cell):
            return 0.0
        x, y = cell
        obstacle_clearance = max(
            0.0,
            float(self._center_distance_field_m[y, x])
            - self._half_diagonal
            - self.grid.resolution_m / 2.0,
        )
        return min(self._boundary_clearance(pose), obstacle_clearance, limit_m)

    def _inflated_grid(self, radius: float) -> GridMap:
        occupancy = inflate_occupancy(
            self._effective_occupancy,
            resolution_m=self.grid.resolution_m,
            radius_m=radius,
        )
        border = max(0, int(ceil(radius / self.grid.resolution_m - 0.5)))
        if border:
            occupancy[:border, :] = True
            occupancy[-border:, :] = True
            occupancy[:, :border] = True
            occupancy[:, -border:] = True
        return GridMap(
            occupancy=occupancy,
            resolution_m=self.grid.resolution_m,
            origin_x_m=self.grid.origin_x_m,
            origin_y_m=self.grid.origin_y_m,
        )

    def pose_is_collision_free(self, pose: Pose2D) -> bool:
        return self.clearance(pose) > 0.0

    def path_is_collision_free(self, path: tuple[Pose2D, ...]) -> bool:
        return bool(path) and all(self.pose_is_collision_free(pose) for pose in path)

    def pose_enters_forbidden(self, pose: Pose2D) -> bool:
        """차체 footprint가 금지 cell과 접촉하거나 겹치면 참을 반환한다."""

        return bool(self.forbidden_cells) and self.forbidden_clearance(pose) <= 0.0

    def forbidden_clearance(self, pose: Pose2D, *, limit_m: float = 1.0) -> float:
        """회전 footprint와 가장 가까운 금지 cell의 표면 여유를 반환한다.

        금지 cell이 없으면 ``limit_m``을 반환한다. Stage 5 ground-truth evaluator가
        20 Hz pose 사이의 금지구역 진입을 보수적으로 판정할 때 사용한다.
        """

        if limit_m <= 0.0:
            raise ValueError("limit_m must be positive")
        return self._occupancy_clearance(
            pose,
            self._forbidden_occupancy,
            limit_m=limit_m,
        )

    def path_enters_forbidden(self, path: tuple[Pose2D, ...]) -> bool:
        return any(self.pose_enters_forbidden(pose) for pose in path)

    def clearance(self, pose: Pose2D, *, limit_m: float = 1.0) -> float:
        """회전 직사각 footprint와 cell AABB 사이의 정확한 최소 여유."""

        boundary = self._boundary_clearance(pose)
        if boundary <= 0.0:
            return 0.0
        obstacle_clearance = self._occupancy_clearance(
            pose, self._effective_occupancy, limit_m=limit_m
        )
        return min(boundary, obstacle_clearance, limit_m)

    def _occupancy_clearance(
        self,
        pose: Pose2D,
        occupancy: np.ndarray,
        *,
        limit_m: float,
    ) -> float:
        if occupancy is self._effective_occupancy and not self._has_effective_occupancy:
            return limit_m
        if occupancy is self._forbidden_occupancy and not self._has_forbidden_occupancy:
            return limit_m
        cell_half_diagonal = self.grid.resolution_m / 2.0**0.5
        radius = self._half_diagonal + limit_m + cell_half_diagonal
        min_cell = self.grid.world_to_cell(Pose2D(pose.x - radius, pose.y - radius))
        max_cell = self.grid.world_to_cell(Pose2D(pose.x + radius, pose.y + radius))
        min_x = max(0, min_cell[0])
        min_y = max(0, min_cell[1])
        max_x = min(self.grid.width - 1, max_cell[0])
        max_y = min(self.grid.height - 1, max_cell[1])
        if min_x > max_x or min_y > max_y:
            return limit_m

        window = occupancy[min_y : max_y + 1, min_x : max_x + 1]
        occupied_y, occupied_x = np.nonzero(window)
        if not occupied_x.size:
            return limit_m

        world_x = self.grid.origin_x_m + (
            occupied_x.astype(np.float64) + min_x + 0.5
        ) * self.grid.resolution_m
        world_y = self.grid.origin_y_m + (
            occupied_y.astype(np.float64) + min_y + 0.5
        ) * self.grid.resolution_m
        center_lower_bounds = (
            np.hypot(world_x - pose.x, world_y - pose.y)
            - self._half_diagonal
            - cell_half_diagonal
        )
        candidates = np.nonzero(center_lower_bounds < limit_m)[0]
        if not candidates.size:
            return limit_m
        candidates = candidates[np.argsort(center_lower_bounds[candidates])]

        footprint = _footprint_polygon(pose, self._half_length, self._half_width)
        best = limit_m
        half_cell = self.grid.resolution_m / 2.0
        for index in candidates:
            if center_lower_bounds[index] >= best:
                break
            cell = _axis_aligned_cell_polygon(
                float(world_x[index]), float(world_y[index]), half_cell
            )
            best = min(best, _convex_polygon_distance(footprint, cell))
            if best <= 0.0:
                return 0.0
        return best

    def _boundary_clearance(self, pose: Pose2D) -> float:
        cosine = abs(cos(pose.yaw))
        sine = abs(sin(pose.yaw))
        extent_x = cosine * self._half_length + sine * self._half_width
        extent_y = sine * self._half_length + cosine * self._half_width
        min_x = self.grid.origin_x_m
        min_y = self.grid.origin_y_m
        max_x = min_x + self.grid.width * self.grid.resolution_m
        max_y = min_y + self.grid.height * self.grid.resolution_m
        return min(
            pose.x - extent_x - min_x,
            max_x - pose.x - extent_x,
            pose.y - extent_y - min_y,
            max_y - pose.y - extent_y,
        )


def _footprint_polygon(pose: Pose2D, half_length: float, half_width: float) -> Polygon:
    cosine = cos(pose.yaw)
    sine = sin(pose.yaw)
    corners: list[Point] = []
    for local_x, local_y in (
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
    ):
        corners.append(
            (
                pose.x + cosine * local_x - sine * local_y,
                pose.y + sine * local_x + cosine * local_y,
            )
        )
    return tuple(corners)


def _axis_aligned_cell_polygon(center_x: float, center_y: float, half: float) -> Polygon:
    return (
        (center_x - half, center_y - half),
        (center_x + half, center_y - half),
        (center_x + half, center_y + half),
        (center_x - half, center_y + half),
    )


def _convex_polygon_distance(first: Polygon, second: Polygon) -> float:
    if _convex_polygons_overlap(first, second):
        return 0.0
    return min(
        _point_segment_distance(point, source, target)
        for polygon, other in ((first, second), (second, first))
        for point in polygon
        for source, target in _segments(other)
    )


def _convex_polygons_overlap(first: Polygon, second: Polygon) -> bool:
    for polygon in (first, second):
        for source, target in _segments(polygon):
            axis_x = -(target[1] - source[1])
            axis_y = target[0] - source[0]
            first_projection = tuple(x * axis_x + y * axis_y for x, y in first)
            second_projection = tuple(x * axis_x + y * axis_y for x, y in second)
            if max(first_projection) < min(second_projection) or max(
                second_projection
            ) < min(first_projection):
                return False
    return True


def _segments(polygon: Polygon) -> tuple[tuple[Point, Point], ...]:
    return tuple(
        (polygon[index], polygon[(index + 1) % len(polygon)])
        for index in range(len(polygon))
    )


def _point_segment_distance(point: Point, source: Point, target: Point) -> float:
    dx = target[0] - source[0]
    dy = target[1] - source[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return hypot(point[0] - source[0], point[1] - source[1])
    fraction = min(
        1.0,
        max(
            0.0,
            ((point[0] - source[0]) * dx + (point[1] - source[1]) * dy)
            / length_squared,
        ),
    )
    closest_x = source[0] + fraction * dx
    closest_y = source[1] + fraction * dy
    return hypot(point[0] - closest_x, point[1] - closest_y)


def oriented_footprint_circle_surface_distance(
    pose: Pose2D,
    *,
    circle_center: Point,
    circle_radius_m: float,
    profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
) -> float:
    """회전 직사각형 footprint와 원 사이의 signed surface distance.

    양수는 분리된 표면 여유, 0은 접촉, 음수는 겹침을 뜻한다. 동적 Actor의
    ground truth 또는 prediction tube 원과 동일한 primitive를 공유하기 위한 API다.
    """

    if circle_radius_m < 0.0:
        raise ValueError("circle_radius_m must not be negative")
    if not all(
        np.isfinite(value)
        for value in (
            pose.x,
            pose.y,
            pose.yaw,
            circle_center[0],
            circle_center[1],
            circle_radius_m,
        )
    ):
        raise ValueError("footprint-circle geometry must be finite")

    footprint = _footprint_polygon(
        pose,
        profile.collision_length_m / 2.0,
        profile.collision_width_m / 2.0,
    )
    if _point_inside_convex_polygon(circle_center, footprint):
        return -circle_radius_m
    center_distance = min(
        _point_segment_distance(circle_center, source, target)
        for source, target in _segments(footprint)
    )
    return center_distance - circle_radius_m


def _point_inside_convex_polygon(point: Point, polygon: Polygon) -> bool:
    signs: list[bool] = []
    for source, target in _segments(polygon):
        cross = (target[0] - source[0]) * (point[1] - source[1]) - (
            target[1] - source[1]
        ) * (point[0] - source[0])
        if abs(cross) <= 1e-15:
            continue
        signs.append(cross > 0.0)
    return not signs or all(sign == signs[0] for sign in signs)
