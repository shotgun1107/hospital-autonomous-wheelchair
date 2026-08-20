"""Minimal FastAPI application for exercising the existing R7 runtime."""

from __future__ import annotations

from fastapi import FastAPI, HTTPException, Response, status
from hospital_path_lab.runtime import RuntimeConfig, RuntimeStateError

from .models import (
    CommandResponse,
    DiagnosticsResponse,
    HealthResponse,
    MissionCreatedResponse,
    MissionPayload,
    StepPayload,
)
from .registry import (
    MissionAlreadyExistsError,
    MissionNotFoundError,
    MissionPreviouslyUsedError,
    RuntimeRegistry,
)


def create_app(runtime_config: RuntimeConfig | None = None) -> FastAPI:
    """Create an in-memory demo app around the persistent R7 runtime facade."""

    config = RuntimeConfig() if runtime_config is None else runtime_config
    registry = RuntimeRegistry(config)
    app = FastAPI(
        title="Hospital Wheelchair R7 Runtime Demo",
        version="0.1.0",
        description="Simulation-only FastAPI adapter; no DB, auth, camera, or motor transport.",
    )
    app.state.runtime_registry = registry

    @app.get("/health", response_model=HealthResponse)
    def health() -> HealthResponse:
        return HealthResponse(status="ok")

    @app.post(
        "/v1/missions",
        response_model=MissionCreatedResponse,
        status_code=status.HTTP_201_CREATED,
    )
    def start_mission(payload: MissionPayload) -> MissionCreatedResponse:
        try:
            mission = payload.to_runtime()
            registry.create(mission)
        except MissionAlreadyExistsError as error:
            raise HTTPException(status_code=409, detail="mission already exists") from error
        except MissionPreviouslyUsedError as error:
            raise HTTPException(
                status_code=409,
                detail="mission id/revision was already used; start a new revision",
            ) from error
        except RuntimeStateError as error:
            raise HTTPException(status_code=503, detail=str(error)) from error
        except (TypeError, ValueError) as error:
            raise HTTPException(status_code=400, detail=str(error)) from error
        return MissionCreatedResponse(mission_id=mission.mission_id, state="started")

    @app.post(
        "/v1/missions/{mission_id}/steps",
        response_model=CommandResponse,
    )
    def step_mission(mission_id: str, payload: StepPayload) -> CommandResponse:
        try:
            command = registry.step(mission_id, payload.to_runtime())
        except MissionNotFoundError as error:
            raise HTTPException(status_code=404, detail="mission not found") from error
        except RuntimeStateError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        except (TypeError, ValueError) as error:
            raise HTTPException(status_code=400, detail=str(error)) from error
        return CommandResponse.from_runtime(command)

    @app.get(
        "/v1/missions/{mission_id}",
        response_model=DiagnosticsResponse,
    )
    def mission_status(mission_id: str) -> DiagnosticsResponse:
        try:
            diagnostics = registry.diagnostics(mission_id)
        except MissionNotFoundError as error:
            raise HTTPException(status_code=404, detail="mission not found") from error
        return DiagnosticsResponse.from_runtime(diagnostics)

    @app.delete(
        "/v1/missions/{mission_id}",
        status_code=status.HTTP_204_NO_CONTENT,
    )
    def reset_mission(mission_id: str) -> Response:
        try:
            registry.reset(mission_id)
        except MissionNotFoundError as error:
            raise HTTPException(status_code=404, detail="mission not found") from error
        except RuntimeStateError as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        return Response(status_code=status.HTTP_204_NO_CONTENT)

    return app


app = create_app()
