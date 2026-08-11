from __future__ import annotations

from math import isclose

import numpy as np

from hospital_path_lab.contracts import (
    GridSnapshot,
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import (
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    build_controller_snapshot,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import build_actor_prediction_set
from hospital_path_lab.followers import DynamicPurePursuitController
from hospital_path_lab.grid import GridMap
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _controller_snapshot(
    *,
    tick_id: int = 0,
    state: RobotState | None = None,
    goal: Pose2D | None = None,
    path: tuple[Pose2D, ...] | None = None,
):
    state = state or RobotState(Pose2D(1.0, 1.0))
    goal = goal or Pose2D(2.0, 1.0)
    path = path or (Pose2D(1.0, 1.0), Pose2D(2.0, 1.0))
    simulation_time_s = tick_id * 0.05
    frame = DynamicObservationFrame(
        stream_id="stream-v1",
        episode_id="episode-v1",
        episode_seed=1,
        map_id="map-v1",
        map_revision=1,
        observation_revision=tick_id,
        sequence=tick_id,
        observed_at_s=simulation_time_s,
        delivered_at_s=simulation_time_s,
        frame_kind=DynamicObservationFrameKind.EMPTY,
        tracks=(),
        content_hash=f"observation-{tick_id}",
    )
    observation = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.FRESH,
        frame=frame,
        age_s=0.0,
        failures=(),
        last_event_was_no_frame=False,
    )
    grid = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="map-v1",
            map_revision=1,
            mission_revision=1,
            observation_revision=tick_id,
            seed=1,
            content_hash=f"grid-{tick_id}",
        ),
        grid=GridMap(np.zeros((160, 160), dtype=np.bool_), resolution_m=0.02),
    )
    return build_controller_snapshot(
        tick_id=tick_id,
        simulation_time_s=simulation_time_s,
        mission_id="mission-v1",
        robot_state=state,
        goal_pose=goal,
        reference_path=path,
        static_grid_snapshot=grid,
        validated_observation=observation,
        actor_tubes=build_actor_prediction_set(observation),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )


def test_dynamic_pp_tracks_reference_without_creating_a_detour() -> None:
    snapshot = _controller_snapshot(
        state=RobotState(Pose2D(1.0, 1.0), Twist2D(0.20, 0.0))
    )

    result = DynamicPurePursuitController().step(snapshot)

    assert result.status is PlanStatus.FOUND
    assert result.requested_twist == Twist2D(0.20, 0.0)
    assert len(result.predicted_trajectory) == 41
    assert result.predicted_trajectory[0].time_s == 0.0
    assert result.predicted_trajectory[0].pose.x == 1.01
    assert all(isclose(point.pose.y, 1.0, abs_tol=1e-12) for point in result.predicted_trajectory)
    assert "detour=false" in result.decision_trace


def test_dynamic_pp_uses_remaining_arc_goal_speed_and_angular_rate_limit() -> None:
    snapshot = _controller_snapshot(
        state=RobotState(Pose2D(1.94, 1.0), Twist2D(0.10, 0.30)),
        path=(Pose2D(1.0, 1.0), Pose2D(2.0, 1.0)),
    )

    result = DynamicPurePursuitController().step(snapshot)

    assert result.status is PlanStatus.FOUND
    assert isclose(result.requested_twist.linear, 0.10, abs_tol=1e-12)
    assert isclose(result.requested_twist.angular, 0.22, abs_tol=1e-12)


def test_dynamic_pp_result_preserves_snapshot_provenance() -> None:
    snapshot = _controller_snapshot(tick_id=4)

    result = DynamicPurePursuitController().step(snapshot)

    assert result.source_tick_id == snapshot.tick_id
    assert result.input_content_hash == snapshot.input_content_hash
    assert result.observation_content_hash == snapshot.observation_content_hash
    assert (
        result.map_revision,
        result.mission_revision,
        result.observation_revision,
    ) == (
        snapshot.map_revision,
        snapshot.mission_revision,
        snapshot.observation_revision,
    )
