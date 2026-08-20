"""HTTP-level smoke tests for the minimal FastAPI R7 runtime adapter."""

from __future__ import annotations

from hospital_path_lab.dynamic_observation import DynamicObservationProfileName
from hospital_path_lab.runtime import RuntimeConfig, RuntimeControllerKind
from httpx import ASGITransport, AsyncClient
from pytest import fixture, mark

from hospital_server import create_app
from hospital_server.demo import create_demo_app

pytestmark = mark.anyio


@fixture
def anyio_backend() -> str:
    return "asyncio"


def _client() -> AsyncClient:
    app = create_app(
        RuntimeConfig(
            controller_kind=RuntimeControllerKind.RPP,
            observation_profile=DynamicObservationProfileName.FUNCTIONAL_IDEAL,
            require_native_dwb=False,
        )
    )
    return AsyncClient(
        transport=ASGITransport(app=app),
        base_url="http://testserver",
    )


def _mission_payload(*, mission_revision: int = 1) -> dict[str, object]:
    empty_row = [False] * 200
    return {
        "mission_id": "api-smoke-mission",
        "mission_revision": mission_revision,
        "runtime_map": {
            "map_id": "api-smoke-map",
            "map_revision": 1,
            "occupancy_rows": [list(empty_row) for _ in range(200)],
            "resolution_m": 0.02,
        },
        "start_pose": {"x_m": 0.5, "y_m": 0.5, "yaw_rad": 0.0},
        "goal_pose": {"x_m": 2.0, "y_m": 0.5, "yaw_rad": 0.0},
        "reference_path": [
            {"x_m": 0.5, "y_m": 0.5, "yaw_rad": 0.0},
            {"x_m": 2.0, "y_m": 0.5, "yaw_rad": 0.0},
        ],
        "observation_stream_id": "api-smoke-camera",
        "observation_session_seed": 100 + mission_revision,
    }


def _step_payload(tick: int, observation: dict[str, object] | None = None) -> dict[str, object]:
    return {
        "control_tick": tick,
        "robot": {
            "pose": {"x_m": 0.5, "y_m": 0.5, "yaw_rad": 0.0},
            "linear_mps": 0.0,
            "angular_radps": 0.0,
        },
        "observation": observation,
    }


async def test_health_and_mission_lifecycle_over_http() -> None:
    async with _client() as client:
        assert (await client.get("/health")).json() == {"status": "ok"}

        created = await client.post("/v1/missions", json=_mission_payload())
        assert created.status_code == 201
        assert created.json() == {
            "mission_id": "api-smoke-mission",
            "state": "started",
        }

        first = await client.post(
            "/v1/missions/api-smoke-mission/steps",
            json=_step_payload(0),
        )
        assert first.status_code == 200
        assert first.json()["motion_state"] == "holding"

        latency = await client.post(
            "/v1/missions/api-smoke-mission/steps",
            json=_step_payload(1),
        )
        assert latency.status_code == 200
        assert latency.json()["motion_state"] == "holding"

        empty_frame = {
            "sequence": 0,
            "observation_revision": 0,
            "observed_at_s": 0.0,
            "actors": [],
        }
        moving = await client.post(
            "/v1/missions/api-smoke-mission/steps",
            json=_step_payload(2, empty_frame),
        )
        assert moving.status_code == 200
        assert moving.json()["motion_state"] == "moving"
        assert moving.json()["linear_mps"] > 0.0

        diagnostics = await client.get("/v1/missions/api-smoke-mission")
        assert diagnostics.status_code == 200
        assert diagnostics.json()["next_control_tick"] == 3
        assert diagnostics.json()["controller_name"] == "persistent_rpp_reference"

        blocked_reset = await client.delete("/v1/missions/api-smoke-mission")
        assert blocked_reset.status_code == 409


async def test_http_errors_do_not_bypass_runtime_lifecycle() -> None:
    async with _client() as client:
        created = await client.post("/v1/missions", json=_mission_payload())
        assert created.status_code == 201
        duplicate = await client.post("/v1/missions", json=_mission_payload())
        assert duplicate.status_code == 409

        missing = await client.post(
            "/v1/missions/unknown/steps",
            json=_step_payload(0),
        )
        assert missing.status_code == 404

        invalid = await client.post(
            "/v1/missions/api-smoke-mission/steps",
            json={**_step_payload(0), "unknown_field": True},
        )
        assert invalid.status_code == 422

        reset = await client.delete("/v1/missions/api-smoke-mission")
        assert reset.status_code == 204
        missing_status = await client.get("/v1/missions/api-smoke-mission")
        assert missing_status.status_code == 404

        reused = await client.post("/v1/missions", json=_mission_payload())
        assert reused.status_code == 409

        new_revision = await client.post(
            "/v1/missions",
            json=_mission_payload(mission_revision=2),
        )
        assert new_revision.status_code == 201


async def test_interactive_demo_changes_output_when_actor_input_arrives() -> None:
    async with AsyncClient(
        transport=ASGITransport(app=create_demo_app()),
        base_url="http://testserver",
    ) as client:
        started = await client.post("/demo/start", json={"goal_x_m": 2.0})
        assert started.status_code == 200

        first = await client.post("/demo/step", json={})
        second = await client.post("/demo/step", json={})
        moving = await client.post("/demo/step", json={})
        before_actor_due = await client.post(
            "/demo/step",
            json={"observation_mode": "actor", "actor_x_m": 1.2},
        )
        actor_due = await client.post(
            "/demo/step",
            json={"observation_mode": "actor", "actor_x_m": 1.2},
        )

        assert first.json()["command"]["motion_state"] == "holding"
        assert second.json()["command"]["motion_state"] == "holding"
        assert moving.json()["observation_frame_sent"] is True
        assert moving.json()["command"]["motion_state"] == "moving"
        assert before_actor_due.status_code == 200, before_actor_due.text
        assert actor_due.status_code == 200, actor_due.text
        assert before_actor_due.json()["observation_frame_sent"] is False
        assert actor_due.json()["observation_frame_sent"] is True
        assert actor_due.json()["input_observation_mode"] == "actor"
        assert actor_due.json()["command"]["motion_state"] == "braking"
