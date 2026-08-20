"""Thread-safe in-memory ownership of mission-scoped R7 runtime instances."""

from __future__ import annotations

from dataclasses import dataclass, field
from threading import Lock, RLock

from hospital_path_lab.runtime import (
    R7Runtime,
    RuntimeCommand,
    RuntimeConfig,
    RuntimeDiagnostics,
    RuntimeMission,
    RuntimeStepInput,
)


class MissionAlreadyExistsError(RuntimeError):
    """A mission id already owns a runtime instance."""


class MissionPreviouslyUsedError(RuntimeError):
    """A stopped mission id/revision cannot mint a fresh runtime authority."""


class MissionNotFoundError(KeyError):
    """The requested mission id has no runtime instance."""


@dataclass(slots=True)
class _RuntimeHandle:
    runtime: R7Runtime
    lock: RLock = field(default_factory=RLock)


class RuntimeRegistry:
    """Own one persistent R7Runtime and one serial lock per mission."""

    def __init__(self, runtime_config: RuntimeConfig) -> None:
        self._runtime_config = runtime_config
        self._handles: dict[str, _RuntimeHandle] = {}
        self._started_mission_keys: set[tuple[str, int]] = set()
        self._registry_lock = Lock()

    def create(self, mission: RuntimeMission) -> RuntimeDiagnostics:
        with self._registry_lock:
            if mission.mission_id in self._handles:
                raise MissionAlreadyExistsError(mission.mission_id)
            mission_key = (mission.mission_id, mission.mission_revision)
            if mission_key in self._started_mission_keys:
                raise MissionPreviouslyUsedError(mission_key)
            runtime = R7Runtime(self._runtime_config)
            runtime.start_mission(mission)
            self._handles[mission.mission_id] = _RuntimeHandle(runtime)
            self._started_mission_keys.add(mission_key)
            return runtime.diagnostics

    def step(self, mission_id: str, value: RuntimeStepInput) -> RuntimeCommand:
        handle = self._get(mission_id)
        with handle.lock:
            return handle.runtime.step(value)

    def diagnostics(self, mission_id: str) -> RuntimeDiagnostics:
        handle = self._get(mission_id)
        with handle.lock:
            return handle.runtime.diagnostics

    def reset(self, mission_id: str) -> None:
        handle = self._get(mission_id)
        with handle.lock:
            handle.runtime.reset()
            with self._registry_lock:
                if self._handles.get(mission_id) is handle:
                    del self._handles[mission_id]

    def _get(self, mission_id: str) -> _RuntimeHandle:
        with self._registry_lock:
            handle = self._handles.get(mission_id)
        if handle is None:
            raise MissionNotFoundError(mission_id)
        return handle
