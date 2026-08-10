"""동적 원형 Actor 비교실험의 simulation-only 자료 계약."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isfinite

from hospital_path_lab.contracts import Pose2D, RobotState, Twist2D

DYNAMIC_SCHEMA_VERSION = "1.0"
DYNAMIC_ACTOR_GENERATOR_VERSION = "dynamic_actor_v1"
DYNAMIC_CONTROL_FREQUENCY_HZ = 20.0
DYNAMIC_CONTROL_PERIOD_S = 1.0 / DYNAMIC_CONTROL_FREQUENCY_HZ
ACTOR_RADIUS_M = 0.18
MAX_ACTOR_SPEED_MPS = 0.50


@dataclass(frozen=True, slots=True)
class Point2D:
    x: float
    y: float

    def __post_init__(self) -> None:
        _require_finite("point", self.x, self.y)


@dataclass(frozen=True, slots=True)
class Vector2D:
    x: float
    y: float

    def __post_init__(self) -> None:
        _require_finite("vector", self.x, self.y)

    @property
    def magnitude(self) -> float:
        return hypot(self.x, self.y)


@dataclass(frozen=True, slots=True)
class ActorWaypoint:
    simulation_time_s: float
    position: Point2D

    def __post_init__(self) -> None:
        _require_finite("waypoint time", self.simulation_time_s)
        if self.simulation_time_s < 0.0:
            raise ValueError("waypoint time must not be negative")


@dataclass(frozen=True, slots=True)
class ActorState:
    actor_id: str
    position: Point2D
    velocity: Vector2D
    radius_m: float
    trajectory_revision: int

    def __post_init__(self) -> None:
        if not self.actor_id:
            raise ValueError("actor_id must not be empty")
        _require_finite("actor radius", self.radius_m)
        if self.radius_m <= 0.0:
            raise ValueError("actor radius must be positive")
        if self.trajectory_revision < 0:
            raise ValueError("trajectory_revision must not be negative")


@dataclass(frozen=True, slots=True)
class DynamicGroundTruthFrame:
    episode_id: str
    seed: int
    tick_id: int
    simulation_time_s: float
    robot_state: RobotState
    actors: tuple[ActorState, ...]
    map_revision: int
    mission_revision: int

    def __post_init__(self) -> None:
        if not self.episode_id:
            raise ValueError("episode_id must not be empty")
        if self.tick_id < 0:
            raise ValueError("tick_id must not be negative")
        _require_finite("frame time", self.simulation_time_s)
        if self.simulation_time_s < 0.0:
            raise ValueError("frame time must not be negative")
        _validate_robot_state(self.robot_state)
        normalized = tuple(self.actors)
        actor_ids = tuple(actor.actor_id for actor in normalized)
        if len(actor_ids) != len(set(actor_ids)):
            raise ValueError("actor IDs must be unique within a frame")
        if min(self.map_revision, self.mission_revision) < 0:
            raise ValueError("revisions must not be negative")
        object.__setattr__(self, "actors", normalized)


@dataclass(frozen=True, slots=True)
class DynamicControllerInputFrame:
    """Stage 1 controller adapter input with no Actor ground truth payload."""

    tick_id: int
    simulation_time_s: float
    robot_state: RobotState
    reference_path: tuple[Pose2D, ...]
    map_revision: int
    mission_revision: int

    def __post_init__(self) -> None:
        if self.tick_id < 0:
            raise ValueError("tick_id must not be negative")
        _require_finite("controller frame time", self.simulation_time_s)
        if self.simulation_time_s < 0.0:
            raise ValueError("controller frame time must not be negative")
        _validate_robot_state(self.robot_state)
        reference_path = tuple(self.reference_path)
        if len(reference_path) < 2:
            raise ValueError("reference_path must contain at least two poses")
        for pose in reference_path:
            _validate_pose(pose)
        if min(self.map_revision, self.mission_revision) < 0:
            raise ValueError("revisions must not be negative")
        object.__setattr__(self, "reference_path", reference_path)


@dataclass(frozen=True, slots=True)
class DynamicAcceptedCommand:
    source_tick_id: int
    applied_tick_id: int
    command: Twist2D

    def __post_init__(self) -> None:
        if self.source_tick_id < 0 or self.applied_tick_id != self.source_tick_id + 1:
            raise ValueError("accepted command must apply on the next tick")
        _require_finite("accepted command", self.command.linear, self.command.angular)


@dataclass(frozen=True, slots=True)
class DynamicStateEvent:
    tick_id: int
    simulation_time_s: float
    kind: str
    actor_id: str | None = None

    def __post_init__(self) -> None:
        if self.tick_id < 0:
            raise ValueError("event tick_id must not be negative")
        _require_finite("event time", self.simulation_time_s)
        if self.simulation_time_s < 0.0:
            raise ValueError("event time must not be negative")
        if not self.kind:
            raise ValueError("event kind must not be empty")


@dataclass(frozen=True, slots=True)
class DynamicTraceMetadata:
    schema_version: str
    generator_version: str
    episode_id: str
    seed: int
    simulation_only: bool
    world_content_hash: str
    control_frequency_hz: float
    tick_count: int
    map_revision: int
    mission_revision: int

    def __post_init__(self) -> None:
        if not self.schema_version or not self.generator_version or not self.episode_id:
            raise ValueError("trace identity fields must not be empty")
        if not self.simulation_only:
            raise ValueError("dynamic Actor trace must remain simulation_only")
        if not self.world_content_hash:
            raise ValueError("world_content_hash must not be empty")
        _require_finite("control frequency", self.control_frequency_hz)
        if self.control_frequency_hz <= 0.0 or self.tick_count <= 0:
            raise ValueError("trace frequency and tick_count must be positive")
        if min(self.map_revision, self.mission_revision) < 0:
            raise ValueError("revisions must not be negative")


@dataclass(frozen=True, slots=True)
class DynamicTrace:
    metadata: DynamicTraceMetadata
    reference_path: tuple[Pose2D, ...]
    ground_truth_frames: tuple[DynamicGroundTruthFrame, ...]
    controller_input_frames: tuple[DynamicControllerInputFrame, ...]
    accepted_commands: tuple[DynamicAcceptedCommand, ...]
    state_events: tuple[DynamicStateEvent, ...]

    def __post_init__(self) -> None:
        reference_path = tuple(self.reference_path)
        truth_frames = tuple(self.ground_truth_frames)
        controller_frames = tuple(self.controller_input_frames)
        commands = tuple(self.accepted_commands)
        events = tuple(self.state_events)
        if len(reference_path) < 2:
            raise ValueError("trace reference_path must contain at least two poses")
        if len(truth_frames) != self.metadata.tick_count + 1:
            raise ValueError("ground truth frame count must be tick_count + 1")
        if len(controller_frames) != len(truth_frames):
            raise ValueError("controller and ground truth frame counts must match")
        if len(commands) != self.metadata.tick_count:
            raise ValueError("accepted command count must equal tick_count")
        period_s = 1.0 / self.metadata.control_frequency_hz
        for expected_tick, (truth, controller) in enumerate(
            zip(truth_frames, controller_frames, strict=True)
        ):
            expected_time = expected_tick * period_s
            if truth.tick_id != expected_tick or controller.tick_id != expected_tick:
                raise ValueError("trace tick IDs must be consecutive")
            if abs(truth.simulation_time_s - expected_time) > 1e-12:
                raise ValueError("ground truth frame time must derive from tick_id")
            if abs(controller.simulation_time_s - expected_time) > 1e-12:
                raise ValueError("controller frame time must derive from tick_id")
            if truth.robot_state != controller.robot_state:
                raise ValueError("controller robot state must match ground truth robot state")
            if truth.episode_id != self.metadata.episode_id:
                raise ValueError("ground truth episode_id must match trace metadata")
            if truth.seed != self.metadata.seed:
                raise ValueError("ground truth seed must match trace metadata")
            expected_revisions = (self.metadata.map_revision, self.metadata.mission_revision)
            if (truth.map_revision, truth.mission_revision) != expected_revisions:
                raise ValueError("ground truth revisions must match trace metadata")
            if (controller.map_revision, controller.mission_revision) != expected_revisions:
                raise ValueError("controller revisions must match trace metadata")
            if controller.reference_path != reference_path:
                raise ValueError("controller reference path must match trace reference path")
        for expected_tick, command in enumerate(commands):
            if command.source_tick_id != expected_tick:
                raise ValueError("accepted command source ticks must be consecutive")
        if any(
            current.tick_id > following.tick_id
            for current, following in zip(events, events[1:], strict=False)
        ):
            raise ValueError("state events must be ordered by tick")
        for event in events:
            if event.tick_id > self.metadata.tick_count:
                raise ValueError("state event tick must be inside the trace")
            if abs(event.simulation_time_s - event.tick_id * period_s) > 1e-12:
                raise ValueError("state event time must derive from tick_id")
        object.__setattr__(self, "reference_path", reference_path)
        object.__setattr__(self, "ground_truth_frames", truth_frames)
        object.__setattr__(self, "controller_input_frames", controller_frames)
        object.__setattr__(self, "accepted_commands", commands)
        object.__setattr__(self, "state_events", events)


def _validate_pose(pose: Pose2D) -> None:
    _require_finite("pose", pose.x, pose.y, pose.yaw)


def _validate_robot_state(state: RobotState) -> None:
    _validate_pose(state.pose)
    _require_finite("robot twist", state.twist.linear, state.twist.angular)


def _require_finite(label: str, *values: float) -> None:
    if not all(isfinite(value) for value in values):
        raise ValueError(f"{label} values must be finite")
