"""Inflation이 끝난 유한 점유 grid를 탐색하는 결정론적 A*."""

from __future__ import annotations

from dataclasses import dataclass
from heapq import heappop, heappush
from math import atan2, ceil, hypot, inf, isfinite
from time import perf_counter_ns

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import (
    GridSnapshot,
    LocalPlanResult,
    PlanStatus,
    Pose2D,
    RobotState,
)
from hospital_path_lab.grid import GridCell, GridMap

DEFAULT_LOCAL_SEARCH_MARGIN_M = 1.00


@dataclass(frozen=True, slots=True)
class GridSearchBounds:
    """Reference path에서 만든 폐구간 cell 경계."""

    min_x: int
    max_x: int
    min_y: int
    max_y: int

    def contains(self, cell: GridCell) -> bool:
        x, y = cell
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y


def reference_search_bounds(
    grid: GridMap,
    reference_path: tuple[Pose2D, ...],
    margin_m: float = DEFAULT_LOCAL_SEARCH_MARGIN_M,
) -> GridSearchBounds:
    """Reference path의 AABB에 명시적 margin을 더한 탐색 경계를 반환한다.

    독립 oracle도 이 함수를 사용하면 planner와 똑같은 유한 탐색 영역에서
    비용을 비교할 수 있다. 입력 유효성 검사는 planner가 먼저 수행한다.
    """

    if not reference_path:
        raise ValueError("reference_path must not be empty")
    if not isfinite(margin_m) or margin_m < 0.0:
        raise ValueError("margin_m must be finite and non-negative")
    cells = tuple(grid.world_to_cell(pose) for pose in reference_path)
    margin_cells = int(ceil(margin_m / grid.resolution_m))
    return GridSearchBounds(
        min_x=max(0, min(cell[0] for cell in cells) - margin_cells),
        max_x=min(grid.width - 1, max(cell[0] for cell in cells) + margin_cells),
        min_y=max(0, min(cell[1] for cell in cells) - margin_cells),
        max_y=min(grid.height - 1, max(cell[1] for cell in cells) + margin_cells),
    )


class BoundedGridAStarPlanner:
    """Reference path 주변의 유한 경계 안에서 8방향 최단 경로를 계산한다.

    공통 ``CollisionChecker``가 가상 차체와 최소 여유만큼 구성공간을
    inflation한다. 탐색은 ``reference_path``의 축 정렬 경계 상자에
    ``search_margin_m``를 더한 영역으로 제한한다.
    """

    name = "grid_astar"

    def __init__(self, *, search_margin_m: float = DEFAULT_LOCAL_SEARCH_MARGIN_M) -> None:
        if not isfinite(search_margin_m) or search_margin_m < 0.0:
            raise ValueError("search_margin_m must be finite and non-negative")
        self.search_margin_m = search_margin_m
        self._cached_snapshot: GridSnapshot | None = None
        self._cached_collision_checker: CollisionChecker | None = None

    def plan(
        self,
        snapshot: GridSnapshot,
        reference_path: tuple[Pose2D, ...],
        robot_state: RobotState,
        goal: Pose2D,
    ) -> LocalPlanResult:
        started_at = perf_counter_ns()
        grid = snapshot.grid
        collision_checker = self._collision_checker_for(snapshot)

        if not snapshot.input_valid:
            return _result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="snapshot_input_invalidated",
            )
        planning_grid = collision_checker.configuration_grid

        invalid_reason = _invalid_input_reason(grid, reference_path, robot_state, goal)
        if invalid_reason is not None:
            return _result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason=invalid_reason,
            )

        search_bounds = reference_search_bounds(
            planning_grid, reference_path, self.search_margin_m
        )

        start_cell = planning_grid.world_to_cell(robot_state.pose)
        goal_cell = planning_grid.world_to_cell(goal)
        if not search_bounds.contains(start_cell):
            return _result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="start_outside_search_bounds",
            )
        if not search_bounds.contains(goal_cell):
            return _result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="goal_outside_search_bounds",
            )
        if collision_checker.pose_enters_forbidden(robot_state.pose):
            return _result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="start_forbidden",
            )
        if collision_checker.pose_enters_forbidden(goal):
            return _result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="goal_forbidden",
            )
        if planning_grid.is_occupied(start_cell):
            return _result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="start_footprint_occupied",
            )
        if planning_grid.is_occupied(goal_cell):
            return _result(
                snapshot,
                status=PlanStatus.INVALID_INPUT,
                elapsed_ns=perf_counter_ns() - started_at,
                failure_reason="goal_footprint_occupied",
            )
        if start_cell == goal_cell:
            path = (planning_grid.cell_to_pose(start_cell),)
            return _found_result(
                snapshot,
                collision_checker,
                path,
                cost=0.0,
                expanded_nodes=1,
                started_at=started_at,
            )

        # f, h, g, x, y 순으로 비교해 같은 입력의 동률을 항상 같은 방식으로 푼다.
        start_h = _heuristic(start_cell, goal_cell, planning_grid.resolution_m)
        frontier: list[tuple[float, float, float, int, int]] = [
            (start_h, start_h, 0.0, start_cell[0], start_cell[1])
        ]
        came_from: dict[GridCell, GridCell] = {}
        best_cost: dict[GridCell, float] = {start_cell: 0.0}
        expanded_nodes = 0

        while frontier:
            _, _, current_cost, x, y = heappop(frontier)
            current = (x, y)
            if current_cost > best_cost.get(current, inf):
                continue

            expanded_nodes += 1
            if current == goal_cell:
                cells = _reconstruct_path(came_from, start_cell, goal_cell)
                path = _cells_to_poses(planning_grid, cells, goal_yaw=goal.yaw)
                return _found_result(
                    snapshot,
                    collision_checker,
                    path,
                    cost=current_cost,
                    expanded_nodes=expanded_nodes,
                    started_at=started_at,
                )

            for neighbor, edge_cost in planning_grid.neighbors8(current):
                if not search_bounds.contains(neighbor):
                    continue
                candidate_cost = current_cost + edge_cost
                if candidate_cost >= best_cost.get(neighbor, inf):
                    continue
                best_cost[neighbor] = candidate_cost
                came_from[neighbor] = current
                heuristic = _heuristic(neighbor, goal_cell, planning_grid.resolution_m)
                heappush(
                    frontier,
                    (
                        candidate_cost + heuristic,
                        heuristic,
                        candidate_cost,
                        neighbor[0],
                        neighbor[1],
                    ),
                )

        return _result(
            snapshot,
            status=PlanStatus.NO_PATH,
            elapsed_ns=perf_counter_ns() - started_at,
            expanded_nodes=expanded_nodes,
            failure_reason="no_path",
        )

    def _collision_checker_for(self, snapshot: GridSnapshot) -> CollisionChecker:
        if self._cached_snapshot is not snapshot:
            self._cached_snapshot = snapshot
            self._cached_collision_checker = CollisionChecker(
                snapshot.grid, forbidden_cells=snapshot.forbidden_cells
            )
        if self._cached_collision_checker is None:  # pragma: no cover - defensive
            raise RuntimeError("collision checker cache was not initialized")
        return self._cached_collision_checker


def _invalid_input_reason(
    grid: GridMap,
    reference_path: tuple[Pose2D, ...],
    robot_state: RobotState,
    goal: Pose2D,
) -> str | None:
    if not _pose_is_finite(robot_state.pose) or not all(
        isfinite(value) for value in (robot_state.twist.linear, robot_state.twist.angular)
    ):
        return "robot_state_non_finite"
    if not _pose_is_finite(goal):
        return "goal_non_finite"
    if not reference_path:
        return "reference_path_empty"
    if any(not _pose_is_finite(pose) for pose in reference_path):
        return "reference_path_non_finite"
    if any(not grid.in_bounds(grid.world_to_cell(pose)) for pose in reference_path):
        return "reference_path_out_of_bounds"

    start_cell = grid.world_to_cell(robot_state.pose)
    goal_cell = grid.world_to_cell(goal)
    if not grid.in_bounds(start_cell):
        return "start_out_of_bounds"
    if not grid.in_bounds(goal_cell):
        return "goal_out_of_bounds"
    if grid.is_occupied(start_cell):
        return "start_occupied"
    if grid.is_occupied(goal_cell):
        return "goal_occupied"
    return None


def _pose_is_finite(pose: Pose2D) -> bool:
    return all(isfinite(value) for value in (pose.x, pose.y, pose.yaw))


def _heuristic(source: GridCell, target: GridCell, resolution_m: float) -> float:
    return hypot(source[0] - target[0], source[1] - target[1]) * resolution_m


def _reconstruct_path(
    came_from: dict[GridCell, GridCell],
    start: GridCell,
    goal: GridCell,
) -> tuple[GridCell, ...]:
    path = [goal]
    current = goal
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return tuple(path)


def _cells_to_poses(
    grid: GridMap,
    cells: tuple[GridCell, ...],
    *,
    goal_yaw: float,
) -> tuple[Pose2D, ...]:
    centers = [grid.cell_to_pose(cell) for cell in cells]
    poses: list[Pose2D] = []
    for index, center in enumerate(centers):
        if index + 1 < len(centers):
            following = centers[index + 1]
            yaw = atan2(following.y - center.y, following.x - center.x)
        else:
            yaw = goal_yaw
        poses.append(Pose2D(x=center.x, y=center.y, yaw=yaw))
    return tuple(poses)


def _found_result(
    snapshot: GridSnapshot,
    collision_checker: CollisionChecker,
    path: tuple[Pose2D, ...],
    *,
    cost: float,
    expanded_nodes: int,
    started_at: int,
) -> LocalPlanResult:
    # 이미 half-diagonal+여유로 팽창된 configuration grid의 경로이므로,
    # 더 보수적인 원형 footprint field로 빠르게 결과를 재검증한다.
    collision = not collision_checker.conservative_path_is_collision_free(path)
    minimum_clearance = min(
        collision_checker.conservative_clearance(pose) for pose in path
    )
    return _result(
        snapshot,
        status=PlanStatus.FOUND,
        path=path,
        cost=cost,
        elapsed_ns=perf_counter_ns() - started_at,
        expanded_nodes=expanded_nodes,
        collision=collision,
        minimum_clearance=minimum_clearance,
    )


def _result(
    snapshot: GridSnapshot,
    *,
    status: PlanStatus,
    elapsed_ns: int,
    path: tuple[Pose2D, ...] = (),
    cost: float | None = None,
    expanded_nodes: int = 0,
    collision: bool = False,
    minimum_clearance: float | None = None,
    failure_reason: str | None = None,
) -> LocalPlanResult:
    metadata = snapshot.metadata
    return LocalPlanResult(
        planner=BoundedGridAStarPlanner.name,
        status=status,
        path=path,
        trajectory=(),
        cost=cost,
        elapsed_ns=elapsed_ns,
        expanded_nodes=expanded_nodes,
        sampled_trajectories=0,
        map_revision=metadata.map_revision,
        mission_revision=metadata.mission_revision,
        observation_revision=metadata.observation_revision,
        collision=collision,
        minimum_clearance=minimum_clearance,
        map_id=metadata.map_id,
        input_content_hash=metadata.content_hash,
        failure_reason=failure_reason,
    )
