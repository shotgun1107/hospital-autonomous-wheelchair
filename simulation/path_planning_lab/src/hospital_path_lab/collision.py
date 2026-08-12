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
        use_optimized_geometry: bool = True,
    ) -> None:
        normalized = frozenset(forbidden_cells)
        width = grid.width
        height = grid.height
        min_forbidden_x = width
        max_forbidden_x = -1
        min_forbidden_y = height
        max_forbidden_y = -1
        for x, y in normalized:
            if x < 0 or x >= width or y < 0 or y >= height:
                raise ValueError("forbidden_cells must be inside the grid")
            if x < min_forbidden_x:
                min_forbidden_x = x
            if x > max_forbidden_x:
                max_forbidden_x = x
            if y < min_forbidden_y:
                min_forbidden_y = y
            if y > max_forbidden_y:
                max_forbidden_y = y
        forbidden_cell_bounds = (
            None
            if not normalized
            else (
                min_forbidden_x,
                max_forbidden_x,
                min_forbidden_y,
                max_forbidden_y,
            )
        )
        self.grid = grid
        self.profile = profile
        self.forbidden_cells = normalized
        self.use_optimized_geometry = use_optimized_geometry
        self._half_width = profile.collision_width_m / 2.0
        self._half_length = profile.collision_length_m / 2.0
        self._half_diagonal = hypot(self._half_width, self._half_length)
        self._forbidden_cell_bounds = forbidden_cell_bounds

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
    def _forbidden_world_bounds(self) -> tuple[float, float, float, float] | None:
        if self._forbidden_cell_bounds is None:
            return None
        min_cell_x, max_cell_x, min_cell_y, max_cell_y = self._forbidden_cell_bounds
        resolution_m = self.grid.resolution_m
        return (
            self.grid.origin_x_m + min_cell_x * resolution_m,
            self.grid.origin_x_m + (max_cell_x + 1) * resolution_m,
            self.grid.origin_y_m + min_cell_y * resolution_m,
            self.grid.origin_y_m + (max_cell_y + 1) * resolution_m,
        )

    @cached_property
    def _forbidden_overlap_certification_grid(self) -> GridMap:
        """Conservative grid used only to prove non-entry into forbidden cells."""

        radius_m = self._half_diagonal + 2.0**0.5 * self.grid.resolution_m
        radius_cells = int(ceil(radius_m / self.grid.resolution_m))
        if radius_cells > min(self.grid.height, self.grid.width):
            # ``inflate_occupancy`` assumes each shifted source slice still
            # overlaps the grid.  Very small non-square maps can violate that
            # assumption.  A fully occupied certification grid proves nothing
            # and therefore sends every non-bbox query to the historical exact
            # forbidden-clearance path without weakening the result.
            occupancy = np.ones_like(self.grid.occupancy)
        else:
            occupancy = inflate_occupancy(
                self._forbidden_occupancy,
                resolution_m=self.grid.resolution_m,
                radius_m=radius_m,
            )
        return GridMap(
            occupancy=occupancy,
            resolution_m=self.grid.resolution_m,
            origin_x_m=self.grid.origin_x_m,
            origin_y_m=self.grid.origin_y_m,
        )

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

    @cached_property
    def _center_chebyshev_distance_field_m(self) -> np.ndarray:
        """Nearest occupied-cell centre distance in the L-infinity metric.

        The two-pass unit 3x3 chamfer transform is exact for the grid
        Chebyshev metric.  Unlike the Euclidean chamfer field above it never
        overestimates Euclidean distance, so it can certify a lower bound used
        to skip exact geometry only when another hazard is already closer.
        """

        if not self._has_effective_occupancy:
            distance = np.full(self.grid.occupancy.shape, np.inf, dtype=np.float64)
            distance.setflags(write=False)
            return distance

        occupancy = self._effective_occupancy
        height, width = occupancy.shape
        unreachable = height + width + 1
        distance_cells = np.where(occupancy, 0, unreachable).astype(np.int32)
        column = np.arange(width, dtype=np.int32)
        neighbor_row = np.empty(width, dtype=np.int32)

        # Exact two-pass 3x3 chamfer transform with unit edge/diagonal cost.
        # Horizontal recurrences use cumulative minima, so only the row axis
        # remains in Python while producing the same chessboard distance as
        # one-cell-at-a-time 3x3 dilation.
        for y in range(height):
            row = distance_cells[y]
            if y:
                previous = distance_cells[y - 1]
                neighbor_row[:] = previous
                neighbor_row[1:] = np.minimum(neighbor_row[1:], previous[:-1])
                neighbor_row[:-1] = np.minimum(neighbor_row[:-1], previous[1:])
                np.minimum(row, neighbor_row + 1, out=row)
            row[:] = column + np.minimum.accumulate(row - column)
        for y in range(height - 1, -1, -1):
            row = distance_cells[y]
            if y + 1 < height:
                following = distance_cells[y + 1]
                neighbor_row[:] = following
                neighbor_row[1:] = np.minimum(neighbor_row[1:], following[:-1])
                neighbor_row[:-1] = np.minimum(neighbor_row[:-1], following[1:])
                np.minimum(row, neighbor_row + 1, out=row)
            row[:] = -column + np.minimum.accumulate((row + column)[::-1])[::-1]
        distance = distance_cells.astype(np.float64) * self.grid.resolution_m
        distance.setflags(write=False)
        return distance

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

    def certified_clearance_lower_bound(
        self,
        pose: Pose2D,
        *,
        limit_m: float = 1.0,
    ) -> float:
        """Return a proof-safe lower bound for exact footprint clearance.

        The nearest occupied-cell centre uses the L-infinity metric, then one
        cell width covers both the continuous query offset and the occupied
        cell AABB.  Subtracting the footprint half diagonal leaves a lower
        bound on the exact oriented-rectangle clearance.  It is intentionally
        conservative and is not a replacement for :meth:`clearance`.
        """

        if limit_m <= 0.0:
            raise ValueError("limit_m must be positive")
        return min(
            self._boundary_clearance(pose),
            self._certified_obstacle_clearance_lower_bound(pose),
            limit_m,
        )

    def _certified_obstacle_clearance_lower_bound(self, pose: Pose2D) -> float:
        resolution_m = self.grid.resolution_m
        x = int((pose.x - self.grid.origin_x_m) // resolution_m)
        y = int((pose.y - self.grid.origin_y_m) // resolution_m)
        if not (0 <= x < self.grid.width and 0 <= y < self.grid.height):
            return 0.0
        return max(
            0.0,
            float(self._center_chebyshev_distance_field_m[y, x])
            - resolution_m
            - self._half_diagonal,
        )

    def certified_minimum_clearance_lower_bound(
        self,
        poses: tuple[Pose2D, ...],
        *,
        limit_m: float = 1.0,
    ) -> float:
        """Return the minimum batch bound without allocating per-pose output."""

        if limit_m <= 0.0:
            raise ValueError("limit_m must be positive")
        if not poses:
            return limit_m
        field = self._center_chebyshev_distance_field_m
        resolution_m = self.grid.resolution_m
        origin_x_m = self.grid.origin_x_m
        origin_y_m = self.grid.origin_y_m
        width = self.grid.width
        height = self.grid.height
        max_x_m = origin_x_m + width * resolution_m
        max_y_m = origin_y_m + height * resolution_m
        half_length = self._half_length
        half_width = self._half_width
        half_diagonal = self._half_diagonal
        minimum = limit_m
        for pose in poses:
            cell_x = int((pose.x - origin_x_m) // resolution_m)
            cell_y = int((pose.y - origin_y_m) // resolution_m)
            if not (0 <= cell_x < width and 0 <= cell_y < height):
                return 0.0
            cosine = abs(cos(pose.yaw))
            sine = abs(sin(pose.yaw))
            extent_x = cosine * half_length + sine * half_width
            extent_y = sine * half_length + cosine * half_width
            boundary = pose.x - extent_x - origin_x_m
            candidate = max_x_m - pose.x - extent_x
            if candidate < boundary:
                boundary = candidate
            candidate = pose.y - extent_y - origin_y_m
            if candidate < boundary:
                boundary = candidate
            candidate = max_y_m - pose.y - extent_y
            if candidate < boundary:
                boundary = candidate
            obstacle_lower_bound = (
                float(field[cell_y, cell_x]) - resolution_m - half_diagonal
            )
            if obstacle_lower_bound < 0.0:
                obstacle_lower_bound = 0.0
            if boundary < minimum:
                minimum = boundary
            if obstacle_lower_bound < minimum:
                minimum = obstacle_lower_bound
            if minimum <= 0.0:
                return minimum
        return minimum

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

        if not self.forbidden_cells:
            return False
        if self.use_optimized_geometry:
            bounds = self._forbidden_world_bounds
            if bounds is None:  # pragma: no cover - guarded above
                return False
            min_x, max_x, min_y, max_y = bounds
            delta_x = max(min_x - pose.x, 0.0, pose.x - max_x)
            delta_y = max(min_y - pose.y, 0.0, pose.y - max_y)
            if hypot(delta_x, delta_y) > self._half_diagonal:
                return False
            grid = self._forbidden_overlap_certification_grid
            if not grid.is_occupied(grid.world_to_cell(pose)):
                return False
        return self.forbidden_clearance(pose) <= 0.0

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
        if not self._has_effective_occupancy:
            return min(boundary, limit_m)
        exact_upper_bound = min(boundary, limit_m)
        if (
            self.use_optimized_geometry
            and self._certified_obstacle_clearance_lower_bound(pose)
            >= exact_upper_bound
        ):
            return exact_upper_bound
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
        if self.use_optimized_geometry:
            delta_x = world_x - pose.x
            delta_y = world_y - pose.y
            cosine = cos(pose.yaw)
            sine = sin(pose.yaw)
            local_x = np.abs(cosine * delta_x + sine * delta_y)
            local_y = np.abs(-sine * delta_x + cosine * delta_y)
            outside_x = np.maximum(local_x - self._half_length, 0.0)
            outside_y = np.maximum(local_y - self._half_width, 0.0)
            center_lower_bounds = np.hypot(outside_x, outside_y) - cell_half_diagonal
        else:
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
            distance = (
                _oriented_footprint_cell_distance(
                    pose,
                    footprint,
                    cell,
                    half_length=self._half_length,
                    half_width=self._half_width,
                )
                if self.use_optimized_geometry
                else _convex_polygon_distance(footprint, cell)
            )
            best = min(best, distance)
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
    first_segments = _segments(first)
    second_segments = _segments(second)
    if _convex_polygons_overlap(
        first,
        second,
        first_segments=first_segments,
        second_segments=second_segments,
    ):
        return 0.0
    # Keep the historical point/segment visitation order (and therefore the
    # exact floating result) while avoiding eight identical segment-tuple
    # constructions for every occupied cell candidate.
    best = float("inf")
    for point in first:
        for source, target in second_segments:
            best = min(best, _point_segment_distance(point, source, target))
    for point in second:
        for source, target in first_segments:
            best = min(best, _point_segment_distance(point, source, target))
    return best


def _oriented_footprint_cell_distance(
    pose: Pose2D,
    footprint: Polygon,
    cell: Polygon,
    *,
    half_length: float,
    half_width: float,
) -> float:
    """Historical exact polygon distance with a rectangle-specific shortlist."""

    footprint_segments = _segments(footprint)
    cell_segments = _segments(cell)
    if _convex_polygons_overlap(
        footprint,
        cell,
        first_segments=footprint_segments,
        second_segments=cell_segments,
    ):
        return 0.0

    left = cell[0][0]
    right = cell[1][0]
    bottom = cell[0][1]
    top = cell[2][1]
    approximate: list[tuple[float, bool, int]] = []
    for index, (x, y) in enumerate(footprint):
        delta_x = max(left - x, 0.0, x - right)
        delta_y = max(bottom - y, 0.0, y - top)
        approximate.append((hypot(delta_x, delta_y), True, index))

    cosine = cos(pose.yaw)
    sine = sin(pose.yaw)
    for index, (x, y) in enumerate(cell):
        delta_x = x - pose.x
        delta_y = y - pose.y
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        approximate.append(
            (
                hypot(
                    max(abs(local_x) - half_length, 0.0),
                    max(abs(local_y) - half_width, 0.0),
                ),
                False,
                index,
            )
        )

    approximate_minimum = min(item[0] for item in approximate)
    coordinate_scale = max(
        1.0,
        *(abs(value) for point in (*footprint, *cell) for value in point),
    )
    selection_tolerance = 1e-12 * coordinate_scale
    best = float("inf")
    for distance, footprint_vertex, index in approximate:
        if distance > approximate_minimum + selection_tolerance:
            continue
        if footprint_vertex:
            point = footprint[index]
            segments = cell_segments
        else:
            point = cell[index]
            segments = footprint_segments
        for source, target in segments:
            best = min(best, _point_segment_distance(point, source, target))
    return best


def _convex_polygons_overlap(
    first: Polygon,
    second: Polygon,
    *,
    first_segments: tuple[tuple[Point, Point], ...] | None = None,
    second_segments: tuple[tuple[Point, Point], ...] | None = None,
) -> bool:
    segment_sets = (
        first_segments if first_segments is not None else _segments(first),
        second_segments if second_segments is not None else _segments(second),
    )
    for segments in segment_sets:
        for source, target in segments:
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
    use_optimized_geometry: bool = True,
    inputs_validated: bool = False,
) -> float:
    """회전 직사각형 footprint와 원 사이의 signed surface distance.

    양수는 분리된 표면 여유, 0은 접촉, 음수는 겹침을 뜻한다. 동적 Actor의
    ground truth 또는 prediction tube 원과 동일한 primitive를 공유하기 위한 API다.
    """

    if not inputs_validated:
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

    half_length = profile.collision_length_m / 2.0
    half_width = profile.collision_width_m / 2.0
    footprint = _footprint_polygon(pose, half_length, half_width)
    if not use_optimized_geometry:
        return _footprint_circle_surface_distance_reference(
            footprint,
            circle_center=circle_center,
            circle_radius_m=circle_radius_m,
        )

    cosine = cos(pose.yaw)
    sine = sin(pose.yaw)
    delta_x = circle_center[0] - pose.x
    delta_y = circle_center[1] - pose.y
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    outside_x = abs(local_x) - half_length
    outside_y = abs(local_y) - half_width
    coordinate_scale = max(
        1.0,
        abs(pose.x),
        abs(pose.y),
        abs(circle_center[0]),
        abs(circle_center[1]),
    )
    ambiguity_tolerance = 1e-12 * coordinate_scale
    if outside_x < -ambiguity_tolerance and outside_y < -ambiguity_tolerance:
        return -circle_radius_m
    if abs(outside_x) <= ambiguity_tolerance or abs(outside_y) <= ambiguity_tolerance:
        return _footprint_circle_surface_distance_reference(
            footprint,
            circle_center=circle_center,
            circle_radius_m=circle_radius_m,
        )

    candidate_segments: list[int] = []
    if outside_x > 0.0:
        candidate_segments.append(1 if local_x > 0.0 else 3)
    if outside_y > 0.0:
        candidate_segments.append(2 if local_y > 0.0 else 0)
    if not candidate_segments:
        return -circle_radius_m
    center_distance = float("inf")
    for index in candidate_segments:
        distance = _point_segment_distance(
            circle_center,
            footprint[index],
            footprint[(index + 1) % len(footprint)],
        )
        if distance < center_distance:
            center_distance = distance
    return center_distance - circle_radius_m


def oriented_footprint_capsule_surface_distance(
    pose: Pose2D,
    *,
    segment_start: Point,
    segment_end: Point,
    capsule_radius_m: float,
    profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    use_optimized_geometry: bool = True,
    inputs_validated: bool = False,
) -> float:
    """Return signed clearance from the wheelchair footprint to a capsule.

    The capsule is the Minkowski sum of the closed centerline segment and a
    circle with ``capsule_radius_m``.  A positive result is free surface
    clearance, zero is contact, and a negative result is overlap.  When the
    centerline intersects or lies inside the footprint, the function returns
    ``-capsule_radius_m``.  This matches the existing circle contract when a
    capsule's two endpoints are equal.

    ``use_optimized_geometry`` evaluates the centerline in the wheelchair's
    axis-aligned local frame.  The reference path evaluates the same geometry
    against the world-frame footprint polygon and is retained as an independent
    parity oracle.
    """

    if not inputs_validated:
        if capsule_radius_m < 0.0:
            raise ValueError("capsule_radius_m must not be negative")
        if not all(
            np.isfinite(value)
            for value in (
                pose.x,
                pose.y,
                pose.yaw,
                segment_start[0],
                segment_start[1],
                segment_end[0],
                segment_end[1],
                capsule_radius_m,
            )
        ):
            raise ValueError("footprint-capsule geometry must be finite")

    if segment_start == segment_end:
        return oriented_footprint_circle_surface_distance(
            pose,
            circle_center=segment_start,
            circle_radius_m=capsule_radius_m,
            profile=profile,
            use_optimized_geometry=use_optimized_geometry,
            inputs_validated=True,
        )

    half_length = profile.collision_length_m / 2.0
    half_width = profile.collision_width_m / 2.0
    if not use_optimized_geometry:
        footprint = _footprint_polygon(pose, half_length, half_width)
        return _footprint_capsule_surface_distance_reference(
            footprint,
            segment_start=segment_start,
            segment_end=segment_end,
            capsule_radius_m=capsule_radius_m,
        )

    cosine = cos(pose.yaw)
    sine = sin(pose.yaw)

    def to_local(point: Point) -> Point:
        delta_x = point[0] - pose.x
        delta_y = point[1] - pose.y
        return (
            cosine * delta_x + sine * delta_y,
            -sine * delta_x + cosine * delta_y,
        )

    local_footprint = (
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
    )
    centerline_distance = _convex_polygon_segment_distance(
        local_footprint,
        segment_start=to_local(segment_start),
        segment_end=to_local(segment_end),
    )
    return centerline_distance - capsule_radius_m


def _footprint_circle_surface_distance_reference(
    footprint: Polygon,
    *,
    circle_center: Point,
    circle_radius_m: float,
) -> float:
    if _point_inside_convex_polygon(circle_center, footprint):
        return -circle_radius_m
    return min(
        _point_segment_distance(
            circle_center,
            footprint[index],
            footprint[(index + 1) % len(footprint)],
        )
        for index in range(len(footprint))
    ) - circle_radius_m


def _footprint_capsule_surface_distance_reference(
    footprint: Polygon,
    *,
    segment_start: Point,
    segment_end: Point,
    capsule_radius_m: float,
) -> float:
    centerline_distance = _convex_polygon_segment_distance(
        footprint,
        segment_start=segment_start,
        segment_end=segment_end,
    )
    return centerline_distance - capsule_radius_m


def _convex_polygon_segment_distance(
    polygon: Polygon,
    *,
    segment_start: Point,
    segment_end: Point,
) -> float:
    """Return the exact unsigned distance between a convex polygon and segment."""

    if _point_inside_convex_polygon(
        segment_start,
        polygon,
    ) or _point_inside_convex_polygon(segment_end, polygon):
        return 0.0

    polygon_segments = _segments(polygon)
    if any(
        _line_segments_intersect(
            segment_start,
            segment_end,
            edge_start,
            edge_end,
        )
        for edge_start, edge_end in polygon_segments
    ):
        return 0.0

    return min(
        *(
            _point_segment_distance(point, segment_start, segment_end)
            for point in polygon
        ),
        *(
            _point_segment_distance(segment_start, edge_start, edge_end)
            for edge_start, edge_end in polygon_segments
        ),
        *(
            _point_segment_distance(segment_end, edge_start, edge_end)
            for edge_start, edge_end in polygon_segments
        ),
    )


def _line_segments_intersect(
    first_start: Point,
    first_end: Point,
    second_start: Point,
    second_end: Point,
) -> bool:
    """Return whether two closed line segments intersect, including contact."""

    scale = max(
        1.0,
        *(
            abs(value)
            for point in (first_start, first_end, second_start, second_end)
            for value in point
        ),
    )
    cross_tolerance = 1e-15 * scale * scale
    coordinate_tolerance = 1e-15 * scale
    first_to_second_start = _orientation_cross(
        first_start,
        first_end,
        second_start,
    )
    first_to_second_end = _orientation_cross(
        first_start,
        first_end,
        second_end,
    )
    second_to_first_start = _orientation_cross(
        second_start,
        second_end,
        first_start,
    )
    second_to_first_end = _orientation_cross(
        second_start,
        second_end,
        first_end,
    )

    if (
        first_to_second_start > cross_tolerance
        and first_to_second_end < -cross_tolerance
        or first_to_second_start < -cross_tolerance
        and first_to_second_end > cross_tolerance
    ) and (
        second_to_first_start > cross_tolerance
        and second_to_first_end < -cross_tolerance
        or second_to_first_start < -cross_tolerance
        and second_to_first_end > cross_tolerance
    ):
        return True

    collinear_candidates = (
        (first_to_second_start, second_start, first_start, first_end),
        (first_to_second_end, second_end, first_start, first_end),
        (second_to_first_start, first_start, second_start, second_end),
        (second_to_first_end, first_end, second_start, second_end),
    )
    return any(
        abs(cross) <= cross_tolerance
        and _point_inside_segment_bounds(
            point,
            segment_start,
            segment_end,
            tolerance=coordinate_tolerance,
        )
        for cross, point, segment_start, segment_end in collinear_candidates
    )


def _orientation_cross(source: Point, target: Point, point: Point) -> float:
    return (target[0] - source[0]) * (point[1] - source[1]) - (
        target[1] - source[1]
    ) * (point[0] - source[0])


def _point_inside_segment_bounds(
    point: Point,
    segment_start: Point,
    segment_end: Point,
    *,
    tolerance: float,
) -> bool:
    return (
        min(segment_start[0], segment_end[0]) - tolerance
        <= point[0]
        <= max(segment_start[0], segment_end[0]) + tolerance
        and min(segment_start[1], segment_end[1]) - tolerance
        <= point[1]
        <= max(segment_start[1], segment_end[1]) + tolerance
    )


def _point_inside_convex_polygon(point: Point, polygon: Polygon) -> bool:
    first_sign: bool | None = None
    for index, source in enumerate(polygon):
        target = polygon[(index + 1) % len(polygon)]
        cross = (target[0] - source[0]) * (point[1] - source[1]) - (
            target[1] - source[1]
        ) * (point[0] - source[0])
        if abs(cross) <= 1e-15:
            continue
        sign = cross > 0.0
        if first_sign is None:
            first_sign = sign
        elif sign != first_sign:
            return False
    return True
