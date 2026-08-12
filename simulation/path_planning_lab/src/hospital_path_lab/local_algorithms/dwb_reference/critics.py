"""Source-derived DWB trajectory critics for the simulation reference lane.

The lifecycle and rejection semantics in this module are behavior-level
reconstructions of Nav2 DWB critics at ``NAV2_NAVIGATION_COMMIT``.  No upstream
implementation text is copied.  The important upstream behaviors retained here
are:

* path and goal costs use a four-neighbour Manhattan distance field;
* map-grid critics score the final trajectory pose by default;
* alignment critics score a point in front of the final robot pose;
* oscillation restrictions are updated only after a command is selected; and
* goal handling latches through approach, stop, and rotate-only phases.

Project simplifications are explicit: this pure-Python module uses a compact
immutable grid instead of a ROS costmap, supports differential drive only, has no
clock-based oscillation reset, and receives a stored path through ``set_path``.
Dynamic actors, footprint collision, terminal stopping, and the shared safety
gate are deliberately outside these *preference* critics.  This module is a
simulation research reference, not a ROS plugin or safety component.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass, field
from math import atan2, ceil, cos, floor, hypot, isfinite, pi, sin
from typing import Protocol, runtime_checkable

from .contracts import DwbGeneratorRequest, DwbPose2D, DwbTrajectory, DwbTwist2D
from .core import IllegalTrajectoryError

GridCell = tuple[int, int]
_VELOCITY_EPSILON = 1e-12


def _require_finite(name: str, value: float) -> None:
    if not isfinite(value):
        raise ValueError(f"{name} must be finite")


def _shortest_angular_distance(from_yaw: float, to_yaw: float) -> float:
    """Return the signed shortest turn in ``[-pi, pi)``."""

    return (to_yaw - from_yaw + pi) % (2.0 * pi) - pi


@runtime_checkable
class DwbReferenceCritic(Protocol):
    """Common critic surface consumed structurally by :class:`DwbReferenceCore`."""

    def prepare(self, request: DwbGeneratorRequest) -> bool | None:
        """Prepare per-control-tick state."""

    def score(self, trajectory: DwbTrajectory) -> float:
        """Return a non-negative raw cost or raise ``IllegalTrajectoryError``."""

    def debrief(self, selected_command: DwbTwist2D) -> None:
        """Observe the command selected after all candidates were scored."""

    def reset(self) -> None:
        """Clear state whose lifetime must not cross a controller reset."""


@dataclass(frozen=True, slots=True)
class DwbCriticGrid:
    """Small immutable cost-grid contract used by the reference critics.

    ``blocked_cells`` represents both known obstacles and cells that callers do
    not permit the local planner to enter.  Off-grid coordinates are always
    illegal.  A later adapter is responsible for converting the project's
    ``GridSnapshot`` into this deliberately narrow contract.
    """

    width: int
    height: int
    resolution_m: float
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    blocked_cells: frozenset[GridCell] = field(default_factory=frozenset)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("grid width and height must be positive")
        if not isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be finite and positive")
        _require_finite("origin_x_m", self.origin_x_m)
        _require_finite("origin_y_m", self.origin_y_m)
        normalized = frozenset(self.blocked_cells)
        if any(not self.in_bounds(cell) for cell in normalized):
            raise ValueError("blocked cells must be inside the grid")
        object.__setattr__(self, "blocked_cells", normalized)

    def in_bounds(self, cell: GridCell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_blocked(self, cell: GridCell) -> bool:
        return not self.in_bounds(cell) or cell in self.blocked_cells

    def world_to_cell(self, pose: DwbPose2D) -> GridCell:
        return (
            floor((pose.x_m - self.origin_x_m) / self.resolution_m),
            floor((pose.y_m - self.origin_y_m) / self.resolution_m),
        )


@dataclass(frozen=True, slots=True)
class ManhattanDistanceField:
    """Deterministic four-neighbour distance-to-source field in grid cells."""

    grid: DwbCriticGrid
    source_cells: tuple[GridCell, ...]
    distances: tuple[tuple[int | None, ...], ...]

    @property
    def obstacle_score(self) -> float:
        """Nav2 MapGrid's finite sentinel for a blocked cell."""

        return float(self.grid.width * self.grid.height)

    @property
    def unreachable_score(self) -> float:
        """Nav2 MapGrid's finite sentinel for an unreachable free cell."""

        return self.obstacle_score + 1.0

    def score_pose(
        self,
        pose: DwbPose2D,
        *,
        stop_on_failure: bool = True,
    ) -> float:
        """Score one pose with Nav2's configurable MapGrid failure behavior.

        Going off-grid is always illegal because Nav2 ``scorePose`` rejects it
        before consulting ``stop_on_failure``.  Blocked and unreachable cells are
        either illegal (PathDist/GoalDist) or returned as large finite sentinels
        (PathAlign/GoalAlign, whose upstream subclasses set
        ``stop_on_failure_=false``).
        """

        cell = self.grid.world_to_cell(pose)
        if not self.grid.in_bounds(cell):
            raise IllegalTrajectoryError(
                "off_grid",
                f"final evaluation pose maps outside grid at {cell}",
            )
        if cell in self.grid.blocked_cells:
            if not stop_on_failure:
                return self.obstacle_score
            raise IllegalTrajectoryError(
                "blocked_grid_cell",
                f"final evaluation pose maps to blocked cell {cell}",
            )
        x, y = cell
        distance = self.distances[y][x]
        if distance is None:
            if not stop_on_failure:
                return self.unreachable_score
            raise IllegalTrajectoryError(
                "unreachable_grid_cell",
                f"final evaluation pose maps to unreachable cell {cell}",
            )
        return float(distance)


def build_manhattan_distance_field(
    grid: DwbCriticGrid,
    source_cells: Sequence[GridCell],
) -> ManhattanDistanceField:
    """Flood free cells from sorted unique sources using a fixed neighbour order.

    This retains DWB's breadth-first Manhattan-distance meaning.  The compact
    reference explicitly treats blocked cells as barriers; invalid source cells
    are ignored, and an all-invalid source set is a preparation error.
    """

    sources = tuple(
        sorted(
            {
                cell
                for cell in source_cells
                if grid.in_bounds(cell) and cell not in grid.blocked_cells
            }
        )
    )
    if not sources:
        raise ValueError("distance field requires at least one free in-grid source")

    mutable: list[list[int | None]] = [
        [None for _ in range(grid.width)] for _ in range(grid.height)
    ]
    queue: deque[GridCell] = deque()
    for x, y in sources:
        mutable[y][x] = 0
        queue.append((x, y))

    # The fixed order is part of reproducibility even though shortest distances
    # themselves are independent of tie order.
    neighbours = ((-1, 0), (0, -1), (0, 1), (1, 0))
    while queue:
        x, y = queue.popleft()
        current = mutable[y][x]
        assert current is not None
        for dx, dy in neighbours:
            adjacent = x + dx, y + dy
            if grid.is_blocked(adjacent):
                continue
            ax, ay = adjacent
            if mutable[ay][ax] is not None:
                continue
            mutable[ay][ax] = current + 1
            queue.append(adjacent)

    return ManhattanDistanceField(
        grid=grid,
        source_cells=sources,
        distances=tuple(tuple(row) for row in mutable),
    )


def _sample_path_cells(grid: DwbCriticGrid, path: Sequence[DwbPose2D]) -> tuple[GridCell, ...]:
    """Rasterize a polyline at no more than one grid resolution per sample.

    Nav2 first adjusts plan resolution before seeding its distance field.  This is
    the Python reference's explicit geometric equivalent; it is not a byte-for-
    byte port of ``nav_2d_utils::adjustPlanResolution``.
    """

    if not path:
        return ()
    sampled: list[GridCell] = []
    for start, end in zip(path, path[1:], strict=False):
        distance = hypot(end.x_m - start.x_m, end.y_m - start.y_m)
        intervals = max(1, ceil(distance / grid.resolution_m))
        for index in range(intervals):
            ratio = index / intervals
            sampled.append(
                grid.world_to_cell(
                    DwbPose2D(
                        start.x_m + (end.x_m - start.x_m) * ratio,
                        start.y_m + (end.y_m - start.y_m) * ratio,
                        start.yaw_rad,
                    )
                )
            )
    sampled.append(grid.world_to_cell(path[-1]))
    return tuple(dict.fromkeys(sampled))


def _local_path_cells(grid: DwbCriticGrid, path: Sequence[DwbPose2D]) -> tuple[GridCell, ...]:
    """Keep the first contiguous in-grid, free portion of a rasterized plan."""

    result: list[GridCell] = []
    started = False
    for cell in _sample_path_cells(grid, path):
        valid = grid.in_bounds(cell) and cell not in grid.blocked_cells
        if valid:
            result.append(cell)
            started = True
        elif started:
            break
    return tuple(dict.fromkeys(result))


class _PathHoldingCritic:
    """No-op lifecycle helpers and explicit stored-plan boundary."""

    def __init__(self) -> None:
        self._path: tuple[DwbPose2D, ...] = ()

    @property
    def path(self) -> tuple[DwbPose2D, ...]:
        return self._path

    def set_path(self, path: Sequence[DwbPose2D]) -> None:
        frozen = tuple(path)
        if not frozen:
            raise ValueError("path must not be empty")
        self._path = frozen
        self.reset()

    def debrief(self, selected_command: DwbTwist2D) -> None:
        del selected_command


class PathDistCritic(_PathHoldingCritic):
    """Score the final pose by Manhattan distance from the local path.

    Upstream behavior: plan cells seed a four-neighbour field and the default
    aggregation type is ``last``.  Project simplification: blocked path samples
    are not valid seeds, and the plan is stored via ``set_path`` because the
    compact generator request does not carry a ROS path.
    """

    def __init__(self, grid: DwbCriticGrid) -> None:
        super().__init__()
        self.grid = grid
        self._field: ManhattanDistanceField | None = None

    @property
    def distance_field(self) -> ManhattanDistanceField | None:
        return self._field

    def prepare(self, request: DwbGeneratorRequest) -> bool:
        del request
        cells = _local_path_cells(self.grid, self.path)
        if not cells:
            self._field = None
            return False
        self._field = build_manhattan_distance_field(self.grid, cells)
        return True

    def score(self, trajectory: DwbTrajectory) -> float:
        return self._score_pose(trajectory.poses[-1])

    def reset(self) -> None:
        self._field = None

    def _score_pose(self, pose: DwbPose2D) -> float:
        if self._field is None:
            raise RuntimeError("PathDistCritic must be prepared before scoring")
        return self._field.score_pose(pose)


class GoalDistCritic(_PathHoldingCritic):
    """Score the final pose by Manhattan distance from the local path endpoint.

    As in Nav2, the source is the last valid plan cell on the local grid.  This
    standalone reference uses a stored path and treats blocked cells as invalid.
    """

    def __init__(self, grid: DwbCriticGrid) -> None:
        super().__init__()
        self.grid = grid
        self._field: ManhattanDistanceField | None = None

    @property
    def distance_field(self) -> ManhattanDistanceField | None:
        return self._field

    def prepare(self, request: DwbGeneratorRequest) -> bool:
        del request
        cells = _local_path_cells(self.grid, self.path)
        if not cells:
            self._field = None
            return False
        self._field = build_manhattan_distance_field(self.grid, (cells[-1],))
        return True

    def score(self, trajectory: DwbTrajectory) -> float:
        return self._score_pose(trajectory.poses[-1])

    def reset(self) -> None:
        self._field = None

    def _score_pose(self, pose: DwbPose2D) -> float:
        if self._field is None:
            raise RuntimeError("GoalDistCritic must be prepared before scoring")
        return self._field.score_pose(pose)


def _forward_pose(pose: DwbPose2D, distance_m: float) -> DwbPose2D:
    return DwbPose2D(
        pose.x_m + distance_m * cos(pose.yaw_rad),
        pose.y_m + distance_m * sin(pose.yaw_rad),
        pose.yaw_rad,
    )


class PathAlignCritic(PathDistCritic):
    """Use a forward-projected final point as a heading-alignment proxy.

    This preserves Nav2 PathAlign's geometry rather than inventing a direct yaw
    penalty.  It also disables itself when the current robot center is within one
    forward-point distance of the goal, matching the upstream stabilization rule.
    """

    def __init__(self, grid: DwbCriticGrid, *, forward_point_distance_m: float = 0.325) -> None:
        if not isfinite(forward_point_distance_m) or forward_point_distance_m < 0.0:
            raise ValueError("forward_point_distance_m must be finite and non-negative")
        super().__init__(grid)
        self.forward_point_distance_m = forward_point_distance_m
        self._disabled_near_goal = False

    @property
    def disabled_near_goal(self) -> bool:
        return self._disabled_near_goal

    def prepare(self, request: DwbGeneratorRequest) -> bool:
        if not self.path:
            self._field = None
            return False
        goal = self.path[-1]
        self._disabled_near_goal = (
            hypot(request.pose.x_m - goal.x_m, request.pose.y_m - goal.y_m)
            <= self.forward_point_distance_m
        )
        if self._disabled_near_goal:
            self._field = None
            return True
        return super().prepare(request)

    def score(self, trajectory: DwbTrajectory) -> float:
        if self._disabled_near_goal:
            return 0.0
        if self._field is None:
            raise RuntimeError("PathAlignCritic must be prepared before scoring")
        return self._field.score_pose(
            _forward_pose(trajectory.poses[-1], self.forward_point_distance_m),
            stop_on_failure=False,
        )

    def reset(self) -> None:
        super().reset()
        self._disabled_near_goal = False


class GoalAlignCritic(GoalDistCritic):
    """Draw the forward-projected final robot point toward a shifted goal.

    Nav2 shifts the local goal away from the robot along the current robot-to-goal
    bearing, then scores the candidate's forward point in that goal distance field.
    This implements the same explicit geometry on the compact grid.
    """

    def __init__(self, grid: DwbCriticGrid, *, forward_point_distance_m: float = 0.325) -> None:
        if not isfinite(forward_point_distance_m) or forward_point_distance_m < 0.0:
            raise ValueError("forward_point_distance_m must be finite and non-negative")
        super().__init__(grid)
        self.forward_point_distance_m = forward_point_distance_m

    def prepare(self, request: DwbGeneratorRequest) -> bool:
        if not self.path:
            self._field = None
            return False
        goal = self.path[-1]
        bearing = atan2(goal.y_m - request.pose.y_m, goal.x_m - request.pose.x_m)
        shifted_goal = DwbPose2D(
            goal.x_m + self.forward_point_distance_m * cos(bearing),
            goal.y_m + self.forward_point_distance_m * sin(bearing),
            goal.yaw_rad,
        )
        cells = _local_path_cells(self.grid, (*self.path[:-1], shifted_goal))
        if not cells:
            self._field = None
            return False
        self._field = build_manhattan_distance_field(self.grid, (cells[-1],))
        return True

    def score(self, trajectory: DwbTrajectory) -> float:
        if self._field is None:
            raise RuntimeError("GoalAlignCritic must be prepared before scoring")
        return self._field.score_pose(
            _forward_pose(trajectory.poses[-1], self.forward_point_distance_m),
            stop_on_failure=False,
        )


class _CommandTrend:
    """One-dimensional sign memory reconstructed from DWB OscillationCritic."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._sign = 0
        self._positive_only = False
        self._negative_only = False

    def update(self, velocity: float) -> bool:
        flag_set = False
        if velocity < 0.0:
            if self._sign > 0:
                self._negative_only = True
                flag_set = True
            self._sign = -1
        elif velocity > 0.0:
            if self._sign < 0:
                self._positive_only = True
                flag_set = True
            self._sign = 1
        return flag_set

    def is_oscillating(self, velocity: float) -> bool:
        return (self._positive_only and velocity < 0.0) or (
            self._negative_only and velocity > 0.0
        )

    @property
    def has_sign_flipped(self) -> bool:
        return self._positive_only or self._negative_only


class OscillationCritic:
    """Reject a second command-sign reversal before sufficient chassis motion.

    The state lifetime matches DWB: ``prepare`` captures the current pose,
    ``score`` only reads restrictions, and ``debrief`` updates them from the one
    selected command.  Restrictions clear after sufficient translation or yaw, or
    explicit ``reset``.  Project simplification: differential drive omits the
    lateral trend and this request contract has no clock, so the optional upstream
    time-based reset is intentionally absent.
    """

    def __init__(
        self,
        *,
        reset_distance_m: float = 0.05,
        reset_angle_rad: float = 0.20,
        linear_only_threshold_mps: float = 0.05,
    ) -> None:
        for name, value in (
            ("reset_distance_m", reset_distance_m),
            ("reset_angle_rad", reset_angle_rad),
            ("linear_only_threshold_mps", linear_only_threshold_mps),
        ):
            _require_finite(name, value)
        self.reset_distance_m = reset_distance_m
        self.reset_angle_rad = reset_angle_rad
        self.linear_only_threshold_mps = linear_only_threshold_mps
        self._linear_trend = _CommandTrend()
        self._angular_trend = _CommandTrend()
        self._prepared_pose: DwbPose2D | None = None
        self._restriction_origin: DwbPose2D | None = None

    @property
    def has_restrictions(self) -> bool:
        return (
            self._linear_trend.has_sign_flipped
            or self._angular_trend.has_sign_flipped
        )

    def prepare(self, request: DwbGeneratorRequest) -> bool:
        self._prepared_pose = request.pose
        return True

    def score(self, trajectory: DwbTrajectory) -> float:
        if self._linear_trend.is_oscillating(trajectory.command.linear_mps) or (
            self._angular_trend.is_oscillating(trajectory.command.angular_radps)
        ):
            raise IllegalTrajectoryError(
                "oscillation_sign_reversal",
                "candidate repeats a command-sign reversal before reset motion",
            )
        return 0.0

    def debrief(self, selected_command: DwbTwist2D) -> None:
        if self._prepared_pose is None:
            raise RuntimeError("OscillationCritic must be prepared before debrief")
        flag_set = self._linear_trend.update(selected_command.linear_mps)
        if (
            self.linear_only_threshold_mps < 0.0
            or abs(selected_command.linear_mps) <= self.linear_only_threshold_mps
        ):
            flag_set |= self._angular_trend.update(selected_command.angular_radps)
        if flag_set:
            self._restriction_origin = self._prepared_pose
        if self.has_restrictions and self._reset_motion_available():
            self.reset()

    def reset(self) -> None:
        self._linear_trend.reset()
        self._angular_trend.reset()
        self._restriction_origin = None

    def _reset_motion_available(self) -> bool:
        if self._prepared_pose is None or self._restriction_origin is None:
            return False
        dx = self._prepared_pose.x_m - self._restriction_origin.x_m
        dy = self._prepared_pose.y_m - self._restriction_origin.y_m
        if self.reset_distance_m >= 0.0 and hypot(dx, dy) > self.reset_distance_m:
            return True
        yaw_change = abs(
            _shortest_angular_distance(
                self._restriction_origin.yaw_rad,
                self._prepared_pose.yaw_rad,
            )
        )
        return self.reset_angle_rad >= 0.0 and yaw_change > self.reset_angle_rad


def _remaining_path_length(pose: DwbPose2D, path: Sequence[DwbPose2D]) -> float:
    """Approximate a pruned local-plan length by projecting onto its nearest segment.

    Nav2 receives an already transformed/pruned local plan.  The compact reference
    stores the complete polyline, so this explicit projection is the required
    adaptation before applying the same path-length goal-window guard.
    """

    if not path:
        raise ValueError("path must not be empty")
    if len(path) == 1:
        return hypot(pose.x_m - path[0].x_m, pose.y_m - path[0].y_m)

    segment_lengths = [
        hypot(end.x_m - start.x_m, end.y_m - start.y_m)
        for start, end in zip(path, path[1:], strict=False)
    ]
    suffix = [0.0] * len(path)
    for index in range(len(segment_lengths) - 1, -1, -1):
        suffix[index] = suffix[index + 1] + segment_lengths[index]

    best_distance_sq = float("inf")
    best_remaining = float("inf")
    for index, (start, end) in enumerate(zip(path, path[1:], strict=False)):
        dx = end.x_m - start.x_m
        dy = end.y_m - start.y_m
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-24:
            ratio = 0.0
        else:
            ratio = max(
                0.0,
                min(
                    1.0,
                    ((pose.x_m - start.x_m) * dx + (pose.y_m - start.y_m) * dy)
                    / length_sq,
                ),
            )
        projection_x = start.x_m + ratio * dx
        projection_y = start.y_m + ratio * dy
        distance_sq = (pose.x_m - projection_x) ** 2 + (pose.y_m - projection_y) ** 2
        remaining = (1.0 - ratio) * segment_lengths[index] + suffix[index + 1]
        if distance_sq < best_distance_sq or (
            distance_sq == best_distance_sq and remaining < best_remaining
        ):
            best_distance_sq = distance_sq
            best_remaining = remaining
    return best_remaining


def _project_trajectory_pose(trajectory: DwbTrajectory, time_s: float) -> DwbPose2D:
    """Interpolate trajectory pose at time, including shortest-yaw interpolation."""

    if time_s <= 0.0:
        return trajectory.poses[0]
    maximum_time = (len(trajectory.poses) - 1) * trajectory.integration_step_s
    if time_s >= maximum_time:
        return trajectory.poses[-1]
    position = time_s / trajectory.integration_step_s
    lower_index = floor(position)
    ratio = position - lower_index
    lower = trajectory.poses[lower_index]
    upper = trajectory.poses[lower_index + 1]
    return DwbPose2D(
        lower.x_m + ratio * (upper.x_m - lower.x_m),
        lower.y_m + ratio * (upper.y_m - lower.y_m),
        lower.yaw_rad + ratio * _shortest_angular_distance(lower.yaw_rad, upper.yaw_rad),
    )


class RotateToGoalCritic(_PathHoldingCritic):
    """Latch approach, stopped, and rotate-only phases near the path goal.

    Upstream behavior retained: outside the goal window this critic is neutral;
    inside it but still translating, only strictly slower commands are legal;
    after the stopped threshold is reached, non-zero translation is illegal and
    remaining candidates are ranked by final (or lookahead) yaw error.

    The stored-plan reference explicitly computes remaining arc length by nearest
    segment projection because Nav2 normally supplies an already-pruned local plan.
    """

    def __init__(
        self,
        *,
        xy_goal_tolerance_m: float = 0.25,
        path_length_tolerance_m: float = 1.0,
        stopped_linear_velocity_mps: float = 0.25,
        slowing_factor: float = 5.0,
        lookahead_time_s: float = -1.0,
    ) -> None:
        super().__init__()
        for name, value in (
            ("xy_goal_tolerance_m", xy_goal_tolerance_m),
            ("path_length_tolerance_m", path_length_tolerance_m),
            ("stopped_linear_velocity_mps", stopped_linear_velocity_mps),
            ("slowing_factor", slowing_factor),
            ("lookahead_time_s", lookahead_time_s),
        ):
            _require_finite(name, value)
        if min(
            xy_goal_tolerance_m,
            path_length_tolerance_m,
            stopped_linear_velocity_mps,
            slowing_factor,
        ) < 0.0:
            raise ValueError("goal critic distances, velocities, and factor must be non-negative")
        self.xy_goal_tolerance_m = xy_goal_tolerance_m
        self.path_length_tolerance_m = path_length_tolerance_m
        self.stopped_linear_velocity_mps = stopped_linear_velocity_mps
        self.slowing_factor = slowing_factor
        self.lookahead_time_s = lookahead_time_s
        self._in_window = False
        self._rotating = False
        self._goal_yaw_rad = 0.0
        self._current_speed_sq = 0.0

    @property
    def in_window(self) -> bool:
        return self._in_window

    @property
    def rotating(self) -> bool:
        return self._rotating

    def prepare(self, request: DwbGeneratorRequest) -> bool:
        if not self.path:
            return False
        goal = self.path[-1]
        distance_sq = (request.pose.x_m - goal.x_m) ** 2 + (
            request.pose.y_m - goal.y_m
        ) ** 2
        remaining_length = _remaining_path_length(request.pose, self.path)
        self._in_window = self._in_window or (
            distance_sq <= self.xy_goal_tolerance_m**2
            and remaining_length <= self.path_length_tolerance_m
        )
        self._current_speed_sq = request.current_twist.linear_mps**2
        self._rotating = self._rotating or (
            self._in_window
            and self._current_speed_sq <= self.stopped_linear_velocity_mps**2
        )
        self._goal_yaw_rad = goal.yaw_rad
        return True

    def score(self, trajectory: DwbTrajectory) -> float:
        if not self._in_window:
            return 0.0
        candidate_speed_sq = trajectory.command.linear_mps**2
        if not self._rotating:
            if candidate_speed_sq >= self._current_speed_sq:
                raise IllegalTrajectoryError(
                    "not_slowing_near_goal",
                    "candidate does not strictly reduce translational speed near goal",
                )
            return candidate_speed_sq * self.slowing_factor + self._rotation_score(
                trajectory
            )
        if abs(trajectory.command.linear_mps) > _VELOCITY_EPSILON:
            raise IllegalTrajectoryError(
                "translation_during_goal_rotation",
                "only in-place rotation is legal after translational stop",
            )
        return self._rotation_score(trajectory)

    def debrief(self, selected_command: DwbTwist2D) -> None:
        del selected_command

    def reset(self) -> None:
        self._in_window = False
        self._rotating = False
        self._goal_yaw_rad = 0.0
        self._current_speed_sq = 0.0

    def _rotation_score(self, trajectory: DwbTrajectory) -> float:
        pose = (
            trajectory.poses[-1]
            if self.lookahead_time_s < 0.0
            else _project_trajectory_pose(trajectory, self.lookahead_time_s)
        )
        return abs(_shortest_angular_distance(pose.yaw_rad, self._goal_yaw_rad))


__all__ = [
    "DwbCriticGrid",
    "DwbReferenceCritic",
    "GoalAlignCritic",
    "GoalDistCritic",
    "ManhattanDistanceField",
    "OscillationCritic",
    "PathAlignCritic",
    "PathDistCritic",
    "RotateToGoalCritic",
    "build_manhattan_distance_field",
]
