"""전역 graph와 분리된 local planner용 결정론적 점유 grid."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot

import numpy as np

from hospital_path_lab.contracts import Pose2D

GridCell = tuple[int, int]


@dataclass(frozen=True, slots=True)
class GridMap:
    occupancy: np.ndarray
    resolution_m: float = 0.02
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0

    def __post_init__(self) -> None:
        array = np.asarray(self.occupancy, dtype=np.bool_)
        if array.ndim != 2 or not array.size:
            raise ValueError("occupancy는 비어 있지 않은 2차원 배열이어야 합니다.")
        if self.resolution_m <= 0:
            raise ValueError("resolution_m은 양수여야 합니다.")
        array = array.copy()
        array.setflags(write=False)
        object.__setattr__(self, "occupancy", array)

    @property
    def height(self) -> int:
        return int(self.occupancy.shape[0])

    @property
    def width(self) -> int:
        return int(self.occupancy.shape[1])

    def in_bounds(self, cell: GridCell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_occupied(self, cell: GridCell) -> bool:
        if not self.in_bounds(cell):
            return True
        x, y = cell
        return bool(self.occupancy[y, x])

    def world_to_cell(self, pose: Pose2D) -> GridCell:
        x = int((pose.x - self.origin_x_m) // self.resolution_m)
        y = int((pose.y - self.origin_y_m) // self.resolution_m)
        return x, y

    def cell_to_pose(self, cell: GridCell) -> Pose2D:
        x, y = cell
        return Pose2D(
            x=self.origin_x_m + (x + 0.5) * self.resolution_m,
            y=self.origin_y_m + (y + 0.5) * self.resolution_m,
        )

    def neighbors8(self, cell: GridCell) -> tuple[tuple[GridCell, float], ...]:
        x, y = cell
        result: list[tuple[GridCell, float]] = []
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)):
            neighbor = x + dx, y + dy
            if self.is_occupied(neighbor):
                continue
            if dx and dy and (self.is_occupied((x + dx, y)) or self.is_occupied((x, y + dy))):
                continue
            result.append((neighbor, hypot(dx, dy) * self.resolution_m))
        return tuple(sorted(result, key=lambda item: item[0]))

    def clearance(self, pose: Pose2D, *, limit_m: float = 1.0) -> float:
        cell_x, cell_y = self.world_to_cell(pose)
        radius = max(1, int(limit_m / self.resolution_m))
        best = limit_m
        for y in range(max(0, cell_y - radius), min(self.height, cell_y + radius + 1)):
            for x in range(max(0, cell_x - radius), min(self.width, cell_x + radius + 1)):
                if not self.occupancy[y, x]:
                    continue
                obstacle = self.cell_to_pose((x, y))
                best = min(best, hypot(pose.x - obstacle.x, pose.y - obstacle.y))
        return best

    def path_is_collision_free(self, path: tuple[Pose2D, ...]) -> bool:
        return bool(path) and all(not self.is_occupied(self.world_to_cell(pose)) for pose in path)


def inflate_occupancy(
    occupancy: np.ndarray,
    *,
    resolution_m: float,
    radius_m: float,
) -> np.ndarray:
    source = np.asarray(occupancy, dtype=np.bool_)
    radius_cells = int(np.ceil(radius_m / resolution_m))
    inflated = source.copy()
    height, width = source.shape
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if hypot(dx, dy) * resolution_m > radius_m:
                continue
            source_y_start = max(0, -dy)
            source_y_end = min(height, height - dy)
            source_x_start = max(0, -dx)
            source_x_end = min(width, width - dx)
            target_y_start = source_y_start + dy
            target_y_end = source_y_end + dy
            target_x_start = source_x_start + dx
            target_x_end = source_x_end + dx
            inflated[target_y_start:target_y_end, target_x_start:target_x_end] |= source[
                source_y_start:source_y_end, source_x_start:source_x_end
            ]
    return inflated
