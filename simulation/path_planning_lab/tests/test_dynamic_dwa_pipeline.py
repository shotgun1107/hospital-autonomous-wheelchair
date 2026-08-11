from __future__ import annotations

from math import inf, isclose

import numpy as np

from hospital_path_lab.contracts import (
    GridSnapshot,
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    TrajectoryPoint,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import (
    ActorTrack,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    Point2D,
    Vector2D,
    build_controller_snapshot,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import build_actor_prediction_set
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_algorithms.dwa import (
    DynamicDwaController,
    _dynamic_candidate,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _controller_snapshot(
    *,
    tick_id: int = 0,
    state: RobotState | None = None,
    occupancy: np.ndarray | None = None,
    actor: ActorTrack | None = None,
):
    state = state or RobotState(Pose2D(1.0, 1.0), Twist2D(0.20, 0.0))
    simulation_time_s = tick_id * 0.05
    tracks = () if actor is None else (actor,)
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
        frame_kind=(
            DynamicObservationFrameKind.EMPTY
            if actor is None
            else DynamicObservationFrameKind.TRACKS
        ),
        tracks=tracks,
        content_hash=f"observation-{tick_id}",
    )
    observation = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.FRESH,
        frame=frame,
        age_s=0.0,
        failures=(),
        last_event_was_no_frame=False,
    )
    if occupancy is None:
        occupancy = np.zeros((180, 180), dtype=np.bool_)
    grid = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="map-v1",
            map_revision=1,
            mission_revision=1,
            observation_revision=tick_id,
            seed=1,
            content_hash=f"grid-{tick_id}",
        ),
        grid=GridMap(occupancy, resolution_m=0.02),
    )
    return build_controller_snapshot(
        tick_id=tick_id,
        simulation_time_s=simulation_time_s,
        mission_id="mission-v1",
        robot_state=state,
        goal_pose=Pose2D(2.4, 1.0),
        reference_path=(Pose2D(1.0, 1.0), Pose2D(2.4, 1.0)),
        static_grid_snapshot=grid,
        validated_observation=observation,
        actor_tubes=build_actor_prediction_set(observation),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )


def test_dynamic_dwa_uses_217_candidates_and_41_post_apply_poses() -> None:
    snapshot = _controller_snapshot()

    result = DynamicDwaController().step(snapshot)

    assert result.status is PlanStatus.FOUND
    assert result.requested_twist.linear == 0.20
    assert result.requested_twist.linear >= 0.0
    assert len(result.predicted_trajectory) == 41
    assert result.predicted_trajectory[0].time_s == 0.0
    assert "sampled_candidates=217" in result.decision_trace
    assert "pose_samples=41" in result.decision_trace


def test_dynamic_dwa_cost_equations_match_the_frozen_oracle() -> None:
    trajectory = tuple(
        TrajectoryPoint(
            time_s=index * 0.05,
            pose=Pose2D(index * 0.01, 0.0),
            twist=Twist2D(0.20, 0.0),
        )
        for index in range(41)
    )

    candidate = _dynamic_candidate(
        Twist2D(0.20, 0.0),
        trajectory,
        start=Pose2D(0.0, 0.0),
        goal=Pose2D(1.0, 0.0),
        reference_path=(Pose2D(0.0, 0.0), Pose2D(1.0, 0.0)),
        minimum_clearance=inf,
        previous_angular=0.0,
    )

    assert candidate.progress_cost == 0.0
    assert candidate.reference_path_cost == 0.0
    assert candidate.heading_cost == 0.0
    assert candidate.clearance_cost == 0.0
    assert candidate.speed_cost == 0.0
    assert candidate.oscillation_cost == 0.0
    assert candidate.score == 0.0


def test_dynamic_dwa_reverse_is_disabled_and_zero_does_not_add_a_sample() -> None:
    controller = DynamicDwaController()
    linear, angular = controller._dynamic_window(
        RobotState(Pose2D(1.0, 1.0), Twist2D(0.0, 0.03))
    )

    assert len(linear) == 7
    assert len(angular) == 31
    assert min(linear) == 0.0
    assert max(linear) <= 0.20
    assert 0.0 in angular


def test_dynamic_dwa_rejects_every_candidate_when_terminal_stop_is_blocked() -> None:
    occupancy = np.ones((120, 180), dtype=np.bool_)
    occupancy[30:80, 10:72] = False
    snapshot = _controller_snapshot(
        state=RobotState(Pose2D(1.0, 1.0), Twist2D(0.20, 0.0)),
        occupancy=occupancy,
    )

    result = DynamicDwaController().step(snapshot)

    assert result.status is PlanStatus.NO_PATH
    assert result.requested_twist == Twist2D()
    assert result.failure_reason == "no_safe_candidate"
    assert result.no_safe_candidate
    assert result.controller_requested_stop


def test_dynamic_dwa_is_deterministic_except_for_elapsed_time() -> None:
    snapshot = _controller_snapshot()
    results = [DynamicDwaController().step(snapshot) for _ in range(2)]
    signatures = {
        (
            result.status,
            result.requested_twist,
            result.predicted_trajectory,
            result.decision_trace,
            result.failure_reason,
        )
        for result in results
    }

    assert len(signatures) == 1
    assert isclose(results[0].requested_twist.linear, 0.20)


def test_dynamic_dwa_can_receive_a_moving_actor_without_ground_truth() -> None:
    actor = ActorTrack(
        track_id="track-1",
        actor_binding_id="actor-1",
        observed_position=Point2D(1.6, 1.15),
        observed_velocity=Vector2D(0.0, 0.50),
        position_sigma_m=0.0,
        velocity_sigma_mps=0.0,
    )
    snapshot = _controller_snapshot(actor=actor)

    result = DynamicDwaController().step(snapshot)

    assert result.status in {PlanStatus.FOUND, PlanStatus.NO_PATH}
    assert result.observation_content_hash == snapshot.observation_content_hash
    assert not hasattr(snapshot, "ground_truth_actors")
