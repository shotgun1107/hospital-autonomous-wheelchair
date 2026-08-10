"""경로 알고리즘 실험실의 공통 입출력 계약."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from hospital_path_lab.graph import GraphMap
    from hospital_path_lab.grid import GridMap


class PlanStatus(StrEnum):
    FOUND = "found"
    NO_PATH = "no_path"
    INVALID_INPUT = "invalid_input"
    STALE_RESULT = "stale_result"


@dataclass(frozen=True, slots=True)
class SnapshotMetadata:
    map_id: str
    map_revision: int
    mission_revision: int
    observation_revision: int
    seed: int
    content_hash: str
    input_valid: bool = True

    def __post_init__(self) -> None:
        if not self.map_id:
            raise ValueError("map_id는 비어 있을 수 없습니다.")
        if min(self.map_revision, self.mission_revision, self.observation_revision) < 0:
            raise ValueError("revision은 음수일 수 없습니다.")
        if not self.content_hash:
            raise ValueError("content_hash는 비어 있을 수 없습니다.")
        if not isinstance(self.input_valid, bool):
            raise TypeError("input_valid must be a bool")


@dataclass(frozen=True, slots=True)
class GraphSnapshot:
    metadata: SnapshotMetadata
    graph: GraphMap

    @property
    def input_valid(self) -> bool:
        return self.metadata.input_valid


@dataclass(frozen=True, slots=True)
class GridSnapshot:
    metadata: SnapshotMetadata
    grid: GridMap
    forbidden_cells: frozenset[tuple[int, int]] = frozenset()

    def __post_init__(self) -> None:
        normalized = frozenset(self.forbidden_cells)
        if any(
            not isinstance(cell, tuple)
            or len(cell) != 2
            or any(not isinstance(value, int) for value in cell)
            for cell in normalized
        ):
            raise TypeError("forbidden_cells must contain (x, y) integer tuples")
        if any(not self.grid.in_bounds(cell) for cell in normalized):
            raise ValueError("forbidden_cells must be inside the grid")
        object.__setattr__(self, "forbidden_cells", normalized)

    @property
    def input_valid(self) -> bool:
        return self.metadata.input_valid


@dataclass(frozen=True, slots=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True, slots=True)
class Twist2D:
    linear: float = 0.0
    angular: float = 0.0


@dataclass(frozen=True, slots=True)
class RobotState:
    pose: Pose2D
    twist: Twist2D = Twist2D()


@dataclass(frozen=True, slots=True)
class TrajectoryPoint:
    time_s: float
    pose: Pose2D
    twist: Twist2D


@dataclass(frozen=True, slots=True)
class LocalPlanResult:
    planner: str
    status: PlanStatus
    path: tuple[Pose2D, ...]
    trajectory: tuple[TrajectoryPoint, ...]
    cost: float | None
    elapsed_ns: int
    expanded_nodes: int
    sampled_trajectories: int
    map_revision: int
    mission_revision: int
    observation_revision: int
    collision: bool
    minimum_clearance: float | None
    map_id: str = ""
    input_content_hash: str = ""
    failure_reason: str | None = None


@dataclass(frozen=True, slots=True)
class FollowerResult:
    follower: str
    status: PlanStatus
    command: Twist2D
    lookahead_point: Pose2D | None
    elapsed_ns: int
    map_revision: int
    mission_revision: int
    observation_revision: int
    map_id: str = ""
    input_content_hash: str = ""
    failure_reason: str | None = None


class LocalPlanner(Protocol):
    name: str

    def plan(
        self,
        snapshot: GridSnapshot,
        reference_path: tuple[Pose2D, ...],
        robot_state: RobotState,
        goal: Pose2D,
    ) -> LocalPlanResult: ...


class PathFollower(Protocol):
    name: str

    def step(
        self,
        path: tuple[Pose2D, ...],
        robot_state: RobotState,
        metadata: SnapshotMetadata,
    ) -> FollowerResult: ...
