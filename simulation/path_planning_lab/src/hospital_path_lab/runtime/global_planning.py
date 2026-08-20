"""Known-map global planning adapter for the R7 runtime facade."""

from __future__ import annotations

from math import hypot

from hospital_path_lab.contracts import GridSnapshot, PlanStatus, RobotState
from hospital_path_lab.local_algorithms.grid_astar import BoundedGridAStarPlanner

from .adapters import to_pose
from .contracts import (
    RuntimeGlobalPlannerKind,
    RuntimeMission,
    RuntimePose,
)

_POSITION_TOLERANCE_M = 1e-12


class RuntimePlanningError(ValueError):
    """The existing global planner could not produce a usable reference path."""


def plan_runtime_reference_path(
    mission: RuntimeMission,
    *,
    grid_snapshot: GridSnapshot,
    planner_kind: RuntimeGlobalPlannerKind,
) -> tuple[RuntimePose, ...]:
    """Run an existing grid planner across the complete known map.

    ``BoundedGridAStarPlanner`` normally limits a local search around an input
    reference.  Supplying only the start/goal as its bounds seed and a margin
    larger than the full map diagonal makes that existing implementation search
    the complete map.  The seed is not a fallback path and is never returned
    when A* fails.
    """

    if not isinstance(mission, RuntimeMission):
        raise TypeError("mission must be a RuntimeMission")
    if not isinstance(grid_snapshot, GridSnapshot):
        raise TypeError("grid_snapshot must be a GridSnapshot")
    if not isinstance(planner_kind, RuntimeGlobalPlannerKind):
        raise TypeError("planner_kind must be a RuntimeGlobalPlannerKind")
    if planner_kind is not RuntimeGlobalPlannerKind.GRID_ASTAR:
        raise RuntimePlanningError(f"runtime_global_planner_unsupported:{planner_kind}")

    start = to_pose(mission.start_pose)
    goal = to_pose(mission.goal_pose)
    if _same_position(mission.start_pose, mission.goal_pose):
        raise RuntimePlanningError("runtime_planning_start_equals_goal")

    grid = grid_snapshot.grid
    start_cell = grid.world_to_cell(start)
    goal_cell = grid.world_to_cell(goal)
    if not grid.in_bounds(start_cell):
        raise RuntimePlanningError("runtime_global_planning_failed:start_out_of_bounds")
    if not grid.in_bounds(goal_cell):
        raise RuntimePlanningError("runtime_global_planning_failed:goal_out_of_bounds")
    full_map_margin_m = hypot(grid.width, grid.height) * grid.resolution_m
    planner = BoundedGridAStarPlanner(search_margin_m=full_map_margin_m)
    result = planner.plan(
        grid_snapshot,
        (start, goal),
        RobotState(start),
        goal,
    )
    if result.status is not PlanStatus.FOUND:
        reason = result.failure_reason or result.status.value
        raise RuntimePlanningError(f"runtime_global_planning_failed:{reason}")
    if result.collision:
        raise RuntimePlanningError("runtime_global_planning_collision")

    path = _replace_exact_endpoints(
        tuple(RuntimePose(pose.x, pose.y, pose.yaw) for pose in result.path),
        start=mission.start_pose,
        goal=mission.goal_pose,
    )
    if len(path) < 2:
        raise RuntimePlanningError("runtime_global_planning_path_too_short")
    return path


def _replace_exact_endpoints(
    path: tuple[RuntimePose, ...],
    *,
    start: RuntimePose,
    goal: RuntimePose,
) -> tuple[RuntimePose, ...]:
    if not path:
        raise RuntimePlanningError("runtime_global_planning_empty_path")
    candidates = (start, *path[1:-1], goal)
    result: list[RuntimePose] = []
    for pose in candidates:
        if result and _same_position(result[-1], pose):
            result[-1] = pose
            continue
        result.append(pose)
    return tuple(result)


def _same_position(left: RuntimePose, right: RuntimePose) -> bool:
    return (
        abs(left.x_m - right.x_m) <= _POSITION_TOLERANCE_M
        and abs(left.y_m - right.y_m) <= _POSITION_TOLERANCE_M
    )
