"""Interactive functional demo app; not the native R7 release configuration."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from threading import RLock

from fastapi import FastAPI, HTTPException
from hospital_path_lab.dynamic_contracts import DYNAMIC_CONTROL_PERIOD_S
from hospital_path_lab.dynamic_observation import DynamicObservationProfileName
from hospital_path_lab.runtime import (
    RuntimeActorObservation,
    RuntimeConfig,
    RuntimeControllerKind,
    RuntimeMap,
    RuntimeMission,
    RuntimeObservation,
    RuntimePose,
    RuntimeRobotState,
    RuntimeStepInput,
)
from pydantic import BaseModel, ConfigDict, Field

from .app import create_app
from .models import CommandResponse, DiagnosticsResponse
from .registry import MissionNotFoundError, RuntimeRegistry

_DEMO_MISSION_ID = "interactive-demo"
_DEMO_MAP_SIZE = 200
_DEMO_OBSERVATION_PERIOD_S = 0.1
_DEMO_OBSERVATION_LATENCY_S = 0.1


class _DemoModel(BaseModel):
    model_config = ConfigDict(extra="forbid")


class DemoObservationMode(StrEnum):
    """What the simulated camera pipeline reports on the next due frame."""

    EMPTY = "empty"
    ACTOR = "actor"
    MISSING = "missing"


class DemoStartPayload(_DemoModel):
    goal_x_m: float = Field(default=2.0, gt=0.8, le=3.5)


class DemoStartResponse(_DemoModel):
    mission_id: str
    message: str
    next_control_tick: int


class DemoStepPayload(_DemoModel):
    robot_x_m: float = 0.5
    robot_y_m: float = 0.5
    robot_yaw_rad: float = 0.0
    robot_linear_mps: float = 0.0
    robot_angular_radps: float = 0.0
    observation_mode: DemoObservationMode = DemoObservationMode.EMPTY
    actor_x_m: float = 1.2
    actor_y_m: float = 0.5
    actor_vx_mps: float = 0.1
    actor_vy_mps: float = 0.0


class DemoStepResponse(_DemoModel):
    input_control_tick: int
    input_observation_mode: DemoObservationMode
    observation_frame_sent: bool
    observation_sequence: int | None
    command: CommandResponse
    next_control_tick: int


@dataclass(slots=True)
class _DemoState:
    started: bool = False
    next_observation_sequence: int = 0
    lock: RLock = field(default_factory=RLock)


def create_demo_app() -> FastAPI:
    """Create a Swagger-friendly RPP/Ideal app with simple editable inputs."""

    app = create_app(
        RuntimeConfig(
            controller_kind=RuntimeControllerKind.RPP,
            observation_profile=DynamicObservationProfileName.FUNCTIONAL_IDEAL,
            require_native_dwb=False,
        )
    )
    state = _DemoState()
    registry: RuntimeRegistry = app.state.runtime_registry

    @app.post("/demo/start", response_model=DemoStartResponse, tags=["interactive demo"])
    def demo_start(payload: DemoStartPayload) -> DemoStartResponse:
        with state.lock:
            if state.started:
                raise HTTPException(
                    status_code=409,
                    detail="demo already started; restart the Uvicorn process to start over",
                )
            registry.create(_demo_mission(payload.goal_x_m))
            state.started = True
            return DemoStartResponse(
                mission_id=_DEMO_MISSION_ID,
                message="Call /demo/step repeatedly and change observation_mode.",
                next_control_tick=0,
            )

    @app.post("/demo/step", response_model=DemoStepResponse, tags=["interactive demo"])
    def demo_step(payload: DemoStepPayload) -> DemoStepResponse:
        with state.lock:
            if not state.started:
                raise HTTPException(status_code=409, detail="call /demo/start first")
            try:
                diagnostics = registry.diagnostics(_DEMO_MISSION_ID)
            except MissionNotFoundError as error:
                raise HTTPException(
                    status_code=409,
                    detail="demo mission is unavailable",
                ) from error
            tick = diagnostics.next_control_tick
            if tick is None:
                raise HTTPException(status_code=409, detail="demo mission has no control tick")
            observation, sequence = _observation_for_step(state, tick, payload)
            command = registry.step(
                _DEMO_MISSION_ID,
                RuntimeStepInput(
                    control_tick=tick,
                    robot=RuntimeRobotState(
                        pose=RuntimePose(
                            payload.robot_x_m,
                            payload.robot_y_m,
                            payload.robot_yaw_rad,
                        ),
                        linear_mps=payload.robot_linear_mps,
                        angular_radps=payload.robot_angular_radps,
                    ),
                    observation=observation,
                ),
            )
            next_tick = registry.diagnostics(_DEMO_MISSION_ID).next_control_tick
            if next_tick is None:
                raise HTTPException(status_code=409, detail="demo mission lost its control tick")
            return DemoStepResponse(
                input_control_tick=tick,
                input_observation_mode=payload.observation_mode,
                observation_frame_sent=sequence is not None,
                observation_sequence=sequence,
                command=CommandResponse.from_runtime(command),
                next_control_tick=next_tick,
            )

    @app.get("/demo/status", response_model=DiagnosticsResponse, tags=["interactive demo"])
    def demo_status() -> DiagnosticsResponse:
        with state.lock:
            if not state.started:
                raise HTTPException(status_code=409, detail="call /demo/start first")
            return DiagnosticsResponse.from_runtime(registry.diagnostics(_DEMO_MISSION_ID))

    return app


def _demo_mission(goal_x_m: float) -> RuntimeMission:
    occupancy_rows = tuple(
        tuple(False for _ in range(_DEMO_MAP_SIZE)) for _ in range(_DEMO_MAP_SIZE)
    )
    start_pose = RuntimePose(0.5, 0.5, 0.0)
    goal_pose = RuntimePose(goal_x_m, 0.5, 0.0)
    return RuntimeMission(
        mission_id=_DEMO_MISSION_ID,
        mission_revision=1,
        runtime_map=RuntimeMap(
            map_id="interactive-demo-map",
            map_revision=1,
            occupancy_rows=occupancy_rows,
        ),
        start_pose=start_pose,
        goal_pose=goal_pose,
        reference_path=(start_pose, goal_pose),
        observation_stream_id="interactive-demo-camera",
        observation_session_seed=9001,
    )


def _observation_for_step(
    state: _DemoState,
    tick: int,
    payload: DemoStepPayload,
) -> tuple[RuntimeObservation | None, int | None]:
    sequence = state.next_observation_sequence
    due_at_s = sequence * _DEMO_OBSERVATION_PERIOD_S + _DEMO_OBSERVATION_LATENCY_S
    control_time_s = tick * DYNAMIC_CONTROL_PERIOD_S
    if control_time_s < due_at_s - 1e-12:
        return None, None
    state.next_observation_sequence += 1
    if payload.observation_mode is DemoObservationMode.MISSING:
        return None, sequence
    actors: tuple[RuntimeActorObservation, ...] = ()
    if payload.observation_mode is DemoObservationMode.ACTOR:
        actors = (
            RuntimeActorObservation(
                track_id="interactive-demo-track",
                actor_binding_id="interactive-demo-actor",
                x_m=payload.actor_x_m,
                y_m=payload.actor_y_m,
                vx_mps=payload.actor_vx_mps,
                vy_mps=payload.actor_vy_mps,
            ),
        )
    return (
        RuntimeObservation(
            sequence=sequence,
            observation_revision=sequence,
            observed_at_s=sequence * _DEMO_OBSERVATION_PERIOD_S,
            actors=actors,
        ),
        sequence,
    )


app = create_demo_app()
