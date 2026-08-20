"""Small backend-facing data contracts for the R7 simulation runtime.

These DTOs deliberately describe processed map/world observations, not camera
frames or motor packets.  They are converted to the immutable R7 contracts in
``hospital_path_lab.runtime.adapters``.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from math import isfinite
from numbers import Real

from hospital_path_lab.dynamic_contracts import DynamicMotionState
from hospital_path_lab.dynamic_observation import DynamicObservationProfileName


class RuntimeControllerKind(StrEnum):
    """Existing R7 controllers exposed by the runtime facade."""

    DWB = "dwb"
    RPP = "rpp"


class RuntimeGlobalPlannerKind(StrEnum):
    """Existing known-map planner exposed by the runtime integration."""

    GRID_ASTAR = "grid_astar"


@dataclass(frozen=True, slots=True)
class RuntimeConfig:
    """Runtime-wide choices that do not change frozen R7 controller values."""

    controller_kind: RuntimeControllerKind = RuntimeControllerKind.DWB
    global_planner_kind: RuntimeGlobalPlannerKind = RuntimeGlobalPlannerKind.GRID_ASTAR
    observation_profile: DynamicObservationProfileName = DynamicObservationProfileName.NORMAL
    require_native_dwb: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.controller_kind, RuntimeControllerKind):
            raise TypeError("controller_kind must be a RuntimeControllerKind")
        if not isinstance(self.global_planner_kind, RuntimeGlobalPlannerKind):
            raise TypeError("global_planner_kind must be a RuntimeGlobalPlannerKind")
        if not isinstance(self.observation_profile, DynamicObservationProfileName):
            raise TypeError("observation_profile must be a DynamicObservationProfileName")
        if self.observation_profile not in {
            DynamicObservationProfileName.NORMAL,
            DynamicObservationProfileName.FUNCTIONAL_IDEAL,
            DynamicObservationProfileName.STRESS,
        }:
            raise ValueError("runtime supports only the frozen Normal, Ideal, or Stress profiles")
        if not isinstance(self.require_native_dwb, bool):
            raise TypeError("require_native_dwb must be bool")


@dataclass(frozen=True, slots=True)
class RuntimePose:
    """Map/world pose in metres and radians."""

    x_m: float
    y_m: float
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        for name in ("x_m", "y_m", "yaw_rad"):
            object.__setattr__(self, name, _finite_float(getattr(self, name), name))


@dataclass(frozen=True, slots=True)
class RuntimeRobotState:
    """Externally estimated robot state for one 20 Hz control tick."""

    pose: RuntimePose
    linear_mps: float = 0.0
    angular_radps: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.pose, RuntimePose):
            raise TypeError("pose must be a RuntimePose")
        for name in ("linear_mps", "angular_radps"):
            object.__setattr__(self, name, _finite_float(getattr(self, name), name))


@dataclass(frozen=True, slots=True)
class RuntimeMap:
    """Known static map input.

    ``occupancy_rows[y][x]`` is ``True`` for a static obstacle.  Coordinates
    use the same map/world frame as every pose and Actor observation.
    """

    map_id: str
    map_revision: int
    occupancy_rows: tuple[tuple[bool, ...], ...]
    resolution_m: float = 0.02
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    forbidden_cells: tuple[tuple[int, int], ...] = ()

    def __post_init__(self) -> None:
        if not isinstance(self.map_id, str) or not self.map_id:
            raise ValueError("map_id must be a non-empty string")
        _require_exact_nonnegative_int(self.map_revision, "map_revision")
        for name in ("resolution_m", "origin_x_m", "origin_y_m"):
            object.__setattr__(self, name, _finite_float(getattr(self, name), name))
        if self.resolution_m <= 0.0:
            raise ValueError("runtime map resolution must be positive")
        rows = tuple(tuple(row) for row in self.occupancy_rows)
        if not rows or not rows[0] or any(len(row) != len(rows[0]) for row in rows):
            raise ValueError("occupancy_rows must be a non-empty rectangular grid")
        if any(not isinstance(value, bool) for row in rows for value in row):
            raise TypeError("occupancy_rows must contain bool values")
        object.__setattr__(self, "occupancy_rows", rows)
        raw_forbidden_cells = tuple(self.forbidden_cells)
        if any(
            not isinstance(cell, tuple)
            or len(cell) != 2
            or any(isinstance(value, bool) or not isinstance(value, int) for value in cell)
            for cell in raw_forbidden_cells
        ):
            raise TypeError("forbidden_cells must contain integer (x, y) cells")
        normalized = tuple(sorted(set(raw_forbidden_cells)))
        width = len(rows[0])
        height = len(rows)
        if any(not (0 <= x < width and 0 <= y < height) for x, y in normalized):
            raise ValueError("forbidden_cells must be inside occupancy_rows")
        object.__setattr__(self, "forbidden_cells", normalized)


@dataclass(frozen=True, slots=True)
class RuntimeActorObservation:
    """One already-detected Actor track in the map/world frame."""

    track_id: str
    actor_binding_id: str
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float

    def __post_init__(self) -> None:
        if not isinstance(self.track_id, str) or not self.track_id:
            raise ValueError("track_id must be a non-empty string")
        if not isinstance(self.actor_binding_id, str) or not self.actor_binding_id:
            raise ValueError("actor_binding_id must be a non-empty string")
        for name in ("x_m", "y_m", "vx_mps", "vy_mps"):
            object.__setattr__(self, name, _finite_float(getattr(self, name), name))


@dataclass(frozen=True, slots=True)
class RuntimeObservation:
    """One new 10 Hz processed observation frame.

    An empty ``actors`` tuple is a valid fresh EMPTY observation.  Omitting the
    whole observation from :class:`RuntimeStepInput` means no new frame arrived
    at that 20 Hz control tick and can therefore become stale.
    """

    sequence: int
    observation_revision: int
    observed_at_s: float
    actors: tuple[RuntimeActorObservation, ...] = ()
    map_id: str | None = None
    map_revision: int | None = None

    def __post_init__(self) -> None:
        _require_exact_nonnegative_int(self.sequence, "sequence")
        _require_exact_nonnegative_int(self.observation_revision, "observation_revision")
        object.__setattr__(
            self,
            "observed_at_s",
            _finite_nonnegative_float(self.observed_at_s, "observed_at_s"),
        )
        actors = tuple(self.actors)
        if any(not isinstance(actor, RuntimeActorObservation) for actor in actors):
            raise TypeError("actors must contain RuntimeActorObservation values")
        object.__setattr__(self, "actors", actors)
        if self.map_id is not None and (not isinstance(self.map_id, str) or not self.map_id):
            raise ValueError("map_id must be a non-empty string when present")
        if self.map_revision is not None:
            _require_exact_nonnegative_int(self.map_revision, "map_revision")


@dataclass(frozen=True, slots=True)
class RuntimeResumeAuthorization:
    """Authorization issued by the backend/authority layer, never by runtime."""

    mission_id: str
    stop_epoch: int
    issued_or_revalidated_at_s: float
    authorization_revision: int
    content_hash: str

    def __post_init__(self) -> None:
        if not isinstance(self.mission_id, str) or not self.mission_id:
            raise ValueError("mission_id must be a non-empty string")
        _require_exact_nonnegative_int(self.stop_epoch, "stop_epoch")
        _require_exact_nonnegative_int(self.authorization_revision, "authorization_revision")
        object.__setattr__(
            self,
            "issued_or_revalidated_at_s",
            _finite_nonnegative_float(
                self.issued_or_revalidated_at_s,
                "issued_or_revalidated_at_s",
            ),
        )
        if not isinstance(self.content_hash, str) or not self.content_hash:
            raise ValueError("content_hash must be a non-empty string")


@dataclass(frozen=True, slots=True)
class RuntimeMission:
    """Known-map mission setup for one stateful runtime instance."""

    mission_id: str
    mission_revision: int
    runtime_map: RuntimeMap
    start_pose: RuntimePose
    goal_pose: RuntimePose
    observation_stream_id: str
    observation_session_seed: int
    authorization_revision: int = 0
    reference_path: tuple[RuntimePose, ...] | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.mission_id, str) or not self.mission_id:
            raise ValueError("mission_id must be a non-empty string")
        _require_exact_nonnegative_int(self.mission_revision, "mission_revision")
        if not isinstance(self.runtime_map, RuntimeMap):
            raise TypeError("runtime_map must be a RuntimeMap")
        if not isinstance(self.start_pose, RuntimePose) or not isinstance(
            self.goal_pose,
            RuntimePose,
        ):
            raise TypeError("start_pose and goal_pose must be RuntimePose values")
        if not isinstance(self.observation_stream_id, str) or not self.observation_stream_id:
            raise ValueError("observation_stream_id must be a non-empty string")
        _require_exact_nonnegative_int(self.observation_session_seed, "observation_session_seed")
        _require_exact_nonnegative_int(self.authorization_revision, "authorization_revision")
        if self.reference_path is None:
            return
        path = tuple(self.reference_path)
        if len(path) < 2 or any(not isinstance(pose, RuntimePose) for pose in path):
            raise ValueError("reference_path must contain at least two RuntimePose values")
        if not _same_position(path[0], self.start_pose) or not _same_position(
            path[-1],
            self.goal_pose,
        ):
            raise ValueError("reference_path must start at start_pose and end at goal_pose")
        if any(
            _same_position(first, second)
            for first, second in zip(path[:-1], path[1:], strict=True)
        ):
            raise ValueError("reference_path cannot contain consecutive duplicate positions")
        object.__setattr__(self, "reference_path", path)


@dataclass(frozen=True, slots=True)
class RuntimeStepInput:
    """One requested R7 control tick; all times are relative to mission start."""

    control_tick: int
    robot: RuntimeRobotState
    observation: RuntimeObservation | None = None
    path_still_valid: bool = True
    local_safety_recheck_passed: bool = True
    resume_authorization: RuntimeResumeAuthorization | None = None
    mission_cancelled: bool = False

    def __post_init__(self) -> None:
        _require_exact_nonnegative_int(self.control_tick, "control_tick")
        if not isinstance(self.robot, RuntimeRobotState):
            raise TypeError("robot must be a RuntimeRobotState")
        if self.observation is not None and not isinstance(self.observation, RuntimeObservation):
            raise TypeError("observation must be a RuntimeObservation or None")
        if self.resume_authorization is not None and not isinstance(
            self.resume_authorization,
            RuntimeResumeAuthorization,
        ):
            raise TypeError("resume_authorization has an unsupported type")
        for name in (
            "path_still_valid",
            "local_safety_recheck_passed",
            "mission_cancelled",
        ):
            if not isinstance(getattr(self, name), bool):
                raise TypeError(f"{name} must be bool")


@dataclass(frozen=True, slots=True)
class RuntimeCommand:
    """Backend-facing command produced by the existing shared R7 gate."""

    linear_mps: float
    angular_radps: float
    motion_state: DynamicMotionState
    stop_reason: str | None
    control_tick: int
    stop_epoch: int
    failure_reasons: tuple[str, ...] = ()
    observation_status: str | None = None
    prediction_status: str | None = None

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.linear_mps, self.angular_radps)):
            raise ValueError("runtime command values must be finite")
        if not isinstance(self.motion_state, DynamicMotionState):
            raise TypeError("motion_state must be a DynamicMotionState")
        _require_exact_nonnegative_int(self.control_tick, "control_tick")
        _require_exact_nonnegative_int(self.stop_epoch, "stop_epoch")
        if self.stop_reason is not None and not isinstance(self.stop_reason, str):
            raise TypeError("stop_reason must be a string or None")
        object.__setattr__(self, "failure_reasons", tuple(self.failure_reasons))


@dataclass(frozen=True, slots=True)
class RuntimeDiagnostics:
    """Small read-only state summary for logging and public integration tests."""

    mission_id: str | None
    next_control_tick: int | None
    motion_state: DynamicMotionState | None
    stop_epoch: int | None
    predictor_status: str | None
    predictor_history_counts: tuple[tuple[str, int], ...]
    last_event_was_no_frame: bool | None
    controller_name: str | None
    native_dwb_active: bool | None


def _require_exact_nonnegative_int(value: object, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative exact integer")


def _finite_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError(f"{name} must be a real number")
    normalized = float(value)
    if not isfinite(normalized):
        raise ValueError(f"{name} must be finite")
    return normalized


def _finite_nonnegative_float(value: object, name: str) -> float:
    normalized = _finite_float(value, name)
    if normalized < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return normalized


def _same_position(left: RuntimePose, right: RuntimePose) -> bool:
    return abs(left.x_m - right.x_m) <= 1e-12 and abs(left.y_m - right.y_m) <= 1e-12
