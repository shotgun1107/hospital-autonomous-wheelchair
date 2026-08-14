"""동적 원형 Actor 비교실험의 simulation-only 자료 계약."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from hashlib import sha256
from json import dumps
from math import hypot, isfinite
from typing import TYPE_CHECKING, Protocol

from hospital_path_lab.contracts import PlanStatus, Pose2D, RobotState, TrajectoryPoint, Twist2D

if TYPE_CHECKING:
    from hospital_path_lab.contracts import GridSnapshot
    from hospital_path_lab.dynamic_directional_prediction import DirectionalPredictionSet
    from hospital_path_lab.dynamic_observation import DynamicObservationSnapshot
    from hospital_path_lab.dynamic_prediction import ActorPredictionSet
    from hospital_path_lab.persistent_controller_contracts import PersistentReferenceBinding
    from hospital_path_lab.vehicle import VehicleProfile

DYNAMIC_SCHEMA_VERSION = "1.0"
DYNAMIC_ACTOR_GENERATOR_VERSION = "dynamic_actor_v1"
DYNAMIC_CONTROL_FREQUENCY_HZ = 20.0
DYNAMIC_CONTROL_PERIOD_S = 1.0 / DYNAMIC_CONTROL_FREQUENCY_HZ
ACTOR_RADIUS_M = 0.18
MAX_ACTOR_SPEED_MPS = 0.50
MAX_ACTOR_ACCELERATION_MPS2 = 0.50
DYNAMIC_OBSERVATION_FREQUENCY_HZ = 10.0
DYNAMIC_OBSERVATION_PERIOD_S = 1.0 / DYNAMIC_OBSERVATION_FREQUENCY_HZ
DYNAMIC_OBSERVATION_TTL_S = 0.300
DYNAMIC_COMMAND_APPLY_LATENCY_S = 0.050


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


class DynamicObservationFrameKind(StrEnum):
    TRACKS = "tracks"
    EMPTY = "empty"


class DynamicMotionState(StrEnum):
    """동적 safety gate가 소유하는 실제 운동 상태."""

    MOVING = "moving"
    BRAKING = "braking"
    HOLDING = "holding"
    COMPLETED = "completed"


class DynamicHoldReason(StrEnum):
    """한 tick의 hold 시간을 중복 집계하지 않기 위한 주 원인."""

    INVALID_SOURCE = "invalid_source"
    INVALID_REFERENCE = "invalid_reference"
    STALE = "stale"
    DEADLINE = "deadline"
    UNAUTHORIZED = "unauthorized"
    GATE_REJECTION = "gate_rejection"
    NO_SAFE_CANDIDATE = "no_safe_candidate"
    TRAFFIC = "traffic"


@dataclass(frozen=True, slots=True)
class ResumeAuthorization:
    """현재 보호정지 사건에 귀속되는 재개 권한."""

    mission_id: str
    stop_epoch: int
    issued_or_revalidated_at_s: float
    authorization_revision: int
    content_hash: str

    def __post_init__(self) -> None:
        if not self.mission_id or not self.content_hash:
            raise ValueError("resume authorization identity fields must not be empty")
        if min(self.stop_epoch, self.authorization_revision) < 0:
            raise ValueError("resume authorization revisions must not be negative")
        _require_finite(
            "resume authorization time",
            self.issued_or_revalidated_at_s,
        )
        if self.issued_or_revalidated_at_s < 0.0:
            raise ValueError("resume authorization time must not be negative")


@dataclass(frozen=True, slots=True)
class DynamicCommandProposal:
    """Controller 종류와 무관한 현재 tick 명령 후보.

    ``trajectory``는 command-apply 50 ms가 끝난 시각을 ``time_s=0``으로 하는
    post-apply rollout이다. 비어 있으면 gate가 한 control tick을 직접 적분한다.
    """

    source_tick_id: int
    command: Twist2D
    computation_time_s: float
    mission_id: str
    map_id: str
    map_revision: int
    mission_revision: int
    observation_revision: int
    grid_content_hash: str
    observation_content_hash: str
    trajectory: tuple[TrajectoryPoint, ...] = ()
    controller_requested_stop: bool = False
    no_safe_candidate: bool = False
    reference_binding: PersistentReferenceBinding | None = None

    def __post_init__(self) -> None:
        if self.source_tick_id < 0:
            raise ValueError("proposal source_tick_id must not be negative")
        if not all(
            (
                self.mission_id,
                self.map_id,
                self.grid_content_hash,
                self.observation_content_hash,
            )
        ):
            raise ValueError("proposal provenance identity fields must not be empty")
        if (
            min(
                self.map_revision,
                self.mission_revision,
                self.observation_revision,
            )
            < 0
        ):
            raise ValueError("proposal provenance revisions must not be negative")
        _require_finite(
            "command proposal",
            self.command.linear,
            self.command.angular,
            self.computation_time_s,
        )
        if self.computation_time_s < 0.0:
            raise ValueError("proposal computation_time_s must not be negative")
        if not isinstance(self.controller_requested_stop, bool) or not isinstance(
            self.no_safe_candidate, bool
        ):
            raise TypeError("proposal flags must be bool values")
        if self.reference_binding is not None:
            from hospital_path_lab.persistent_controller_contracts import (
                PersistentReferenceBinding,
            )

            if not isinstance(self.reference_binding, PersistentReferenceBinding):
                raise TypeError(
                    "reference_binding must be a PersistentReferenceBinding when present"
                )
        object.__setattr__(self, "trajectory", tuple(self.trajectory))


@dataclass(frozen=True, slots=True)
class ControllerSnapshot:
    """PP와 DWA가 공유하는 ground-truth 비포함 controller 입력."""

    tick_id: int
    simulation_time_s: float
    mission_id: str
    robot_state: RobotState
    goal_pose: Pose2D
    reference_path: tuple[Pose2D, ...]
    static_grid_snapshot: GridSnapshot
    validated_observation: DynamicObservationSnapshot
    actor_tubes: ActorPredictionSet | DirectionalPredictionSet | None
    vehicle_profile: VehicleProfile
    map_id: str
    map_revision: int
    mission_revision: int
    observation_revision: int
    input_content_hash: str

    def __post_init__(self) -> None:
        if (
            self.tick_id < 0
            or min(
                self.map_revision,
                self.mission_revision,
                self.observation_revision,
            )
            < 0
        ):
            raise ValueError("controller snapshot counters must not be negative")
        if not self.mission_id or not self.map_id or not self.input_content_hash:
            raise ValueError("controller snapshot identity fields must not be empty")
        _require_finite("controller snapshot time", self.simulation_time_s)
        if self.simulation_time_s < 0.0:
            raise ValueError("controller snapshot time must not be negative")
        _validate_robot_state(self.robot_state)
        _validate_pose(self.goal_pose)
        path = tuple(self.reference_path)
        if len(path) < 2:
            raise ValueError("controller reference_path must contain at least two poses")
        for pose in path:
            _validate_pose(pose)
        metadata = self.static_grid_snapshot.metadata
        if (
            self.map_id,
            self.map_revision,
            self.mission_revision,
            self.observation_revision,
        ) != (
            metadata.map_id,
            metadata.map_revision,
            metadata.mission_revision,
            metadata.observation_revision,
        ):
            raise ValueError("controller snapshot grid provenance mismatch")
        if not self.vehicle_profile.simulation_only:
            raise ValueError("controller snapshot requires a simulation-only vehicle profile")
        object.__setattr__(self, "reference_path", path)

    @property
    def observation_content_hash(self) -> str:
        frame = self.validated_observation.frame
        return frame.content_hash if frame is not None else "observation-unavailable"


@dataclass(frozen=True, slots=True)
class ControllerCommandResult:
    """한 controller tick의 명령·rollout·출처와 결정 trace."""

    controller_name: str
    source_tick_id: int
    status: PlanStatus
    requested_twist: Twist2D
    predicted_trajectory: tuple[TrajectoryPoint, ...]
    failure_reason: str | None
    decision_trace: tuple[str, ...]
    mission_id: str
    map_id: str
    map_revision: int
    mission_revision: int
    observation_revision: int
    grid_content_hash: str
    observation_content_hash: str
    input_content_hash: str
    elapsed_ns: int
    controller_requested_stop: bool = False
    no_safe_candidate: bool = False

    def __post_init__(self) -> None:
        if not self.controller_name or not self.mission_id or not self.map_id:
            raise ValueError("controller result identity fields must not be empty")
        if not all(
            (
                self.grid_content_hash,
                self.observation_content_hash,
                self.input_content_hash,
            )
        ):
            raise ValueError("controller result content hashes must not be empty")
        if (
            self.source_tick_id < 0
            or self.elapsed_ns < 0
            or min(
                self.map_revision,
                self.mission_revision,
                self.observation_revision,
            )
            < 0
        ):
            raise ValueError("controller result counters must not be negative")
        if not isinstance(self.status, PlanStatus):
            raise TypeError("controller result status must be a PlanStatus")
        _require_finite(
            "controller requested twist",
            self.requested_twist.linear,
            self.requested_twist.angular,
        )
        trajectory = tuple(self.predicted_trajectory)
        previous_time = -1.0
        for point in trajectory:
            _require_finite(
                "controller trajectory",
                point.time_s,
                point.pose.x,
                point.pose.y,
                point.pose.yaw,
                point.twist.linear,
                point.twist.angular,
            )
            if point.time_s <= previous_time:
                raise ValueError("controller trajectory time must strictly increase")
            previous_time = point.time_s
        if trajectory and trajectory[0].time_s != 0.0:
            raise ValueError("controller trajectory must start at post-apply time zero")
        if not isinstance(self.controller_requested_stop, bool) or not isinstance(
            self.no_safe_candidate, bool
        ):
            raise TypeError("controller result flags must be bool values")
        expected_input_hash = controller_snapshot_content_hash(
            tick_id=self.source_tick_id,
            mission_id=self.mission_id,
            map_id=self.map_id,
            map_revision=self.map_revision,
            mission_revision=self.mission_revision,
            observation_revision=self.observation_revision,
            grid_content_hash=self.grid_content_hash,
            observation_content_hash=self.observation_content_hash,
        )
        if self.input_content_hash != expected_input_hash:
            raise ValueError("controller result input_content_hash mismatch")
        object.__setattr__(self, "predicted_trajectory", trajectory)
        object.__setattr__(self, "decision_trace", tuple(self.decision_trace))


class DynamicController(Protocol):
    name: str

    def step(self, snapshot: ControllerSnapshot) -> ControllerCommandResult: ...


def controller_snapshot_content_hash(
    *,
    tick_id: int,
    mission_id: str,
    map_id: str,
    map_revision: int,
    mission_revision: int,
    observation_revision: int,
    grid_content_hash: str,
    observation_content_hash: str,
) -> str:
    payload = {
        "grid_content_hash": grid_content_hash,
        "map_id": map_id,
        "map_revision": map_revision,
        "mission_id": mission_id,
        "mission_revision": mission_revision,
        "observation_content_hash": observation_content_hash,
        "observation_revision": observation_revision,
        "tick_id": tick_id,
    }
    serialized = dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return sha256(serialized.encode("utf-8")).hexdigest()


def build_controller_snapshot(
    *,
    tick_id: int,
    simulation_time_s: float,
    mission_id: str,
    robot_state: RobotState,
    goal_pose: Pose2D,
    reference_path: tuple[Pose2D, ...],
    static_grid_snapshot: GridSnapshot,
    validated_observation: DynamicObservationSnapshot,
    actor_tubes: ActorPredictionSet | DirectionalPredictionSet | None,
    vehicle_profile: VehicleProfile,
) -> ControllerSnapshot:
    """검증된 Stage 2/3 입력의 provenance를 하나의 controller snapshot으로 묶는다."""

    metadata = static_grid_snapshot.metadata
    frame = validated_observation.frame
    observation_hash = frame.content_hash if frame is not None else "observation-unavailable"
    if frame is not None and (
        frame.map_id,
        frame.map_revision,
        frame.observation_revision,
    ) != (
        metadata.map_id,
        metadata.map_revision,
        metadata.observation_revision,
    ):
        raise ValueError("controller observation and grid provenance mismatch")
    if actor_tubes is not None:
        if frame is None:
            raise ValueError("controller Actor tubes require an observation frame")
        if (
            actor_tubes.map_id,
            actor_tubes.map_revision,
            actor_tubes.observation_revision,
            actor_tubes.sequence,
            actor_tubes.source_content_hash,
        ) != (
            frame.map_id,
            frame.map_revision,
            frame.observation_revision,
            frame.sequence,
            frame.content_hash,
        ):
            raise ValueError("controller Actor tube provenance mismatch")
        # Imported lazily to preserve the contracts -> predictor dependency.
        # A history-derived set cannot be rebuilt from one frame, so its
        # stateful factory capability and semantic commitments are verified.
        from hospital_path_lab.dynamic_directional_prediction import (
            DirectionalPredictionSet,
            validate_directional_prediction_set,
        )

        if isinstance(actor_tubes, DirectionalPredictionSet):
            validate_directional_prediction_set(actor_tubes, current_frame=frame)
    content_hash = controller_snapshot_content_hash(
        tick_id=tick_id,
        mission_id=mission_id,
        map_id=metadata.map_id,
        map_revision=metadata.map_revision,
        mission_revision=metadata.mission_revision,
        observation_revision=metadata.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=observation_hash,
    )
    return ControllerSnapshot(
        tick_id=tick_id,
        simulation_time_s=simulation_time_s,
        mission_id=mission_id,
        robot_state=robot_state,
        goal_pose=goal_pose,
        reference_path=reference_path,
        static_grid_snapshot=static_grid_snapshot,
        validated_observation=validated_observation,
        actor_tubes=actor_tubes,
        vehicle_profile=vehicle_profile,
        map_id=metadata.map_id,
        map_revision=metadata.map_revision,
        mission_revision=metadata.mission_revision,
        observation_revision=metadata.observation_revision,
        input_content_hash=content_hash,
    )


def controller_result_to_proposal(
    result: ControllerCommandResult,
    *,
    computation_time_s: float,
) -> DynamicCommandProposal:
    """결정론 lane의 시간을 명시적으로 주입해 result를 gate proposal로 변환한다."""

    return DynamicCommandProposal(
        source_tick_id=result.source_tick_id,
        command=result.requested_twist,
        computation_time_s=computation_time_s,
        mission_id=result.mission_id,
        map_id=result.map_id,
        map_revision=result.map_revision,
        mission_revision=result.mission_revision,
        observation_revision=result.observation_revision,
        grid_content_hash=result.grid_content_hash,
        observation_content_hash=result.observation_content_hash,
        trajectory=result.predicted_trajectory,
        controller_requested_stop=result.controller_requested_stop,
        no_safe_candidate=result.no_safe_candidate,
    )


@dataclass(frozen=True, slots=True)
class DynamicSafetyEventCounters:
    controller_stop_requests: int = 0
    gate_overrides: int = 0
    candidate_rejected_by_gate: int = 0
    late_results_discarded: int = 0
    resume_authorizations_rejected: int = 0

    def __post_init__(self) -> None:
        if (
            min(
                self.controller_stop_requests,
                self.gate_overrides,
                self.candidate_rejected_by_gate,
                self.late_results_discarded,
                self.resume_authorizations_rejected,
            )
            < 0
        ):
            raise ValueError("dynamic safety event counters must not be negative")


@dataclass(frozen=True, slots=True)
class DynamicSafetyDecision:
    tick_id: int
    source_tick_id: int
    motion_state: DynamicMotionState
    stop_epoch: int
    command: Twist2D
    proposal_accepted: bool
    resume_allowed: bool
    primary_hold_reason: DynamicHoldReason | None
    consecutive_stop_ticks: int
    consecutive_safe_frames: int
    minimum_static_clearance_m: float | None
    minimum_actor_clearance_m: float | None
    counters: DynamicSafetyEventCounters
    failure_reasons: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if (
            min(
                self.tick_id,
                self.source_tick_id,
                self.stop_epoch,
                self.consecutive_stop_ticks,
                self.consecutive_safe_frames,
            )
            < 0
        ):
            raise ValueError("dynamic safety decision counters must not be negative")
        _require_finite(
            "dynamic safety decision command",
            self.command.linear,
            self.command.angular,
        )
        for clearance in (
            self.minimum_static_clearance_m,
            self.minimum_actor_clearance_m,
        ):
            if clearance is not None and not isfinite(clearance):
                raise ValueError("dynamic safety decision clearance must be finite or None")
        object.__setattr__(self, "failure_reasons", tuple(self.failure_reasons))


@dataclass(frozen=True, slots=True)
class ActorTrack:
    """Controller-facing Actor observation without future ground truth."""

    track_id: str
    actor_binding_id: str
    observed_position: Point2D
    observed_velocity: Vector2D
    position_sigma_m: float
    velocity_sigma_mps: float

    def __post_init__(self) -> None:
        if not self.track_id or not self.actor_binding_id:
            raise ValueError("track identity fields must not be empty")
        _require_finite(
            "track uncertainty",
            self.position_sigma_m,
            self.velocity_sigma_mps,
        )
        if min(self.position_sigma_m, self.velocity_sigma_mps) < 0.0:
            raise ValueError("track uncertainty must not be negative")


@dataclass(frozen=True, slots=True)
class DynamicObservationFrame:
    """A delivered 10 Hz observation frame; dropout is represented by no frame."""

    stream_id: str
    episode_id: str
    episode_seed: int
    map_id: str
    map_revision: int
    observation_revision: int
    sequence: int
    observed_at_s: float
    delivered_at_s: float
    frame_kind: DynamicObservationFrameKind
    tracks: tuple[ActorTrack, ...]
    content_hash: str

    def __post_init__(self) -> None:
        if not self.stream_id or not self.episode_id or not self.map_id:
            raise ValueError("observation identity fields must not be empty")
        if not self.content_hash:
            raise ValueError("observation content_hash must not be empty")
        if min(self.map_revision, self.observation_revision, self.sequence) < 0:
            raise ValueError("observation revisions and sequence must not be negative")
        _require_finite(
            "observation timestamp",
            self.observed_at_s,
            self.delivered_at_s,
        )
        if min(self.observed_at_s, self.delivered_at_s) < 0.0:
            raise ValueError("observation timestamps must not be negative")
        if not isinstance(self.frame_kind, DynamicObservationFrameKind):
            raise TypeError("frame_kind must be a DynamicObservationFrameKind")
        object.__setattr__(self, "tracks", tuple(self.tracks))


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
