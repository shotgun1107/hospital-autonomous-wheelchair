"""HTTP request and response models for the R7 runtime demo API."""

from __future__ import annotations

from hospital_path_lab.runtime import (
    RuntimeActorObservation,
    RuntimeCommand,
    RuntimeDiagnostics,
    RuntimeMap,
    RuntimeMission,
    RuntimeObservation,
    RuntimePose,
    RuntimeResumeAuthorization,
    RuntimeRobotState,
    RuntimeStepInput,
)
from pydantic import BaseModel, ConfigDict, Field


class _StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)


class PosePayload(_StrictModel):
    x_m: float
    y_m: float
    yaw_rad: float = 0.0

    def to_runtime(self) -> RuntimePose:
        return RuntimePose(self.x_m, self.y_m, self.yaw_rad)


class RobotStatePayload(_StrictModel):
    pose: PosePayload
    linear_mps: float = 0.0
    angular_radps: float = 0.0

    def to_runtime(self) -> RuntimeRobotState:
        return RuntimeRobotState(
            pose=self.pose.to_runtime(),
            linear_mps=self.linear_mps,
            angular_radps=self.angular_radps,
        )


class MapPayload(_StrictModel):
    map_id: str
    map_revision: int = Field(ge=0)
    occupancy_rows: list[list[bool]]
    resolution_m: float = 0.02
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    forbidden_cells: list[tuple[int, int]] = Field(default_factory=list)

    def to_runtime(self) -> RuntimeMap:
        return RuntimeMap(
            map_id=self.map_id,
            map_revision=self.map_revision,
            occupancy_rows=tuple(tuple(row) for row in self.occupancy_rows),
            resolution_m=self.resolution_m,
            origin_x_m=self.origin_x_m,
            origin_y_m=self.origin_y_m,
            forbidden_cells=tuple(self.forbidden_cells),
        )


class MissionPayload(_StrictModel):
    mission_id: str
    mission_revision: int = Field(ge=0)
    runtime_map: MapPayload
    start_pose: PosePayload
    goal_pose: PosePayload
    reference_path: list[PosePayload]
    observation_stream_id: str
    observation_session_seed: int = Field(ge=0)
    authorization_revision: int = Field(default=0, ge=0)

    def to_runtime(self) -> RuntimeMission:
        return RuntimeMission(
            mission_id=self.mission_id,
            mission_revision=self.mission_revision,
            runtime_map=self.runtime_map.to_runtime(),
            start_pose=self.start_pose.to_runtime(),
            goal_pose=self.goal_pose.to_runtime(),
            reference_path=tuple(pose.to_runtime() for pose in self.reference_path),
            observation_stream_id=self.observation_stream_id,
            observation_session_seed=self.observation_session_seed,
            authorization_revision=self.authorization_revision,
        )


class ActorObservationPayload(_StrictModel):
    track_id: str
    actor_binding_id: str
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float

    def to_runtime(self) -> RuntimeActorObservation:
        return RuntimeActorObservation(
            track_id=self.track_id,
            actor_binding_id=self.actor_binding_id,
            x_m=self.x_m,
            y_m=self.y_m,
            vx_mps=self.vx_mps,
            vy_mps=self.vy_mps,
        )


class ObservationPayload(_StrictModel):
    sequence: int = Field(ge=0)
    observation_revision: int = Field(ge=0)
    observed_at_s: float = Field(ge=0.0)
    actors: list[ActorObservationPayload] = Field(default_factory=list)
    map_id: str | None = None
    map_revision: int | None = Field(default=None, ge=0)

    def to_runtime(self) -> RuntimeObservation:
        return RuntimeObservation(
            sequence=self.sequence,
            observation_revision=self.observation_revision,
            observed_at_s=self.observed_at_s,
            actors=tuple(actor.to_runtime() for actor in self.actors),
            map_id=self.map_id,
            map_revision=self.map_revision,
        )


class ResumeAuthorizationPayload(_StrictModel):
    mission_id: str
    stop_epoch: int = Field(ge=0)
    issued_or_revalidated_at_s: float = Field(ge=0.0)
    authorization_revision: int = Field(ge=0)
    content_hash: str

    def to_runtime(self) -> RuntimeResumeAuthorization:
        return RuntimeResumeAuthorization(
            mission_id=self.mission_id,
            stop_epoch=self.stop_epoch,
            issued_or_revalidated_at_s=self.issued_or_revalidated_at_s,
            authorization_revision=self.authorization_revision,
            content_hash=self.content_hash,
        )


class StepPayload(_StrictModel):
    control_tick: int = Field(ge=0)
    robot: RobotStatePayload
    observation: ObservationPayload | None = None
    path_still_valid: bool = True
    local_safety_recheck_passed: bool = True
    resume_authorization: ResumeAuthorizationPayload | None = None
    mission_cancelled: bool = False

    def to_runtime(self) -> RuntimeStepInput:
        return RuntimeStepInput(
            control_tick=self.control_tick,
            robot=self.robot.to_runtime(),
            observation=None if self.observation is None else self.observation.to_runtime(),
            path_still_valid=self.path_still_valid,
            local_safety_recheck_passed=self.local_safety_recheck_passed,
            resume_authorization=(
                None
                if self.resume_authorization is None
                else self.resume_authorization.to_runtime()
            ),
            mission_cancelled=self.mission_cancelled,
        )


class HealthResponse(_StrictModel):
    status: str


class MissionCreatedResponse(_StrictModel):
    mission_id: str
    state: str


class CommandResponse(_StrictModel):
    linear_mps: float
    angular_radps: float
    motion_state: str
    stop_reason: str | None
    control_tick: int
    stop_epoch: int
    failure_reasons: list[str]
    observation_status: str | None
    prediction_status: str | None

    @classmethod
    def from_runtime(cls, command: RuntimeCommand) -> CommandResponse:
        return cls(
            linear_mps=command.linear_mps,
            angular_radps=command.angular_radps,
            motion_state=command.motion_state.value,
            stop_reason=command.stop_reason,
            control_tick=command.control_tick,
            stop_epoch=command.stop_epoch,
            failure_reasons=list(command.failure_reasons),
            observation_status=command.observation_status,
            prediction_status=command.prediction_status,
        )


class DiagnosticsResponse(_StrictModel):
    mission_id: str | None
    next_control_tick: int | None
    motion_state: str | None
    stop_epoch: int | None
    predictor_status: str | None
    predictor_history_counts: list[tuple[str, int]]
    last_event_was_no_frame: bool | None
    controller_name: str | None
    native_dwb_active: bool | None

    @classmethod
    def from_runtime(cls, diagnostics: RuntimeDiagnostics) -> DiagnosticsResponse:
        return cls(
            mission_id=diagnostics.mission_id,
            next_control_tick=diagnostics.next_control_tick,
            motion_state=(
                None if diagnostics.motion_state is None else diagnostics.motion_state.value
            ),
            stop_epoch=diagnostics.stop_epoch,
            predictor_status=diagnostics.predictor_status,
            predictor_history_counts=list(diagnostics.predictor_history_counts),
            last_event_was_no_frame=diagnostics.last_event_was_no_frame,
            controller_name=diagnostics.controller_name,
            native_dwb_active=diagnostics.native_dwb_active,
        )
