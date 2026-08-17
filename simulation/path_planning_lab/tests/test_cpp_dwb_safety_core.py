from __future__ import annotations

from dataclasses import replace
from math import isclose

import numpy as np
import pytest

from hospital_path_lab.contracts import (
    GridSnapshot,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    Twist2D,
)
from hospital_path_lab.cpp_dwb_safety_core import (
    CPP_DWB_SAFETY_CORE_AVAILABLE,
    CppDwbSafetyFailure,
    evaluate_dwb_safety_batch,
)
from hospital_path_lab.dynamic_contracts import (
    ActorTrack,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    Point2D,
    Vector2D,
    build_controller_snapshot,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DIRECTIONAL_PREDICTION_VERSION,
    DirectionalPredictionSet,
    DirectionalPredictionTube,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import build_actor_prediction_set
from hospital_path_lab.dynamic_safety import (
    build_dynamic_trajectory_safety_checkers,
    evaluate_dynamic_trajectory_safety,
)
from hospital_path_lab.dynamic_trajectory_constraints import _proposal_from_trajectory
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_algorithms.dwb_reference.contracts import (
    DwbGeneratorRequest,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)
from hospital_path_lab.local_algorithms.dwb_reference.trajectory_generator import (
    DwbReferenceTrajectoryGenerator,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1

pytestmark = pytest.mark.skipif(
    not CPP_DWB_SAFETY_CORE_AVAILABLE,
    reason="optional C++ DWB safety core has not been built",
)


def _snapshot(*, actor_x: float = 2.80):
    tick = 7
    simulation_time_s = tick * 0.05
    track = ActorTrack(
        track_id="track-1",
        actor_binding_id="actor-1",
        observed_position=Point2D(actor_x, 2.0),
        observed_velocity=Vector2D(0.10, 0.0),
        position_sigma_m=0.03,
        velocity_sigma_mps=0.05,
    )
    frame = DynamicObservationFrame(
        stream_id="cpp-dwb-stream",
        episode_id="cpp-dwb-episode",
        episode_seed=71,
        map_id="cpp-dwb-map",
        map_revision=3,
        observation_revision=tick,
        sequence=tick,
        observed_at_s=simulation_time_s,
        delivered_at_s=simulation_time_s,
        frame_kind=DynamicObservationFrameKind.TRACKS,
        tracks=(track,),
        content_hash="cpp-dwb-observation",
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
            map_id="cpp-dwb-map",
            map_revision=3,
            mission_revision=5,
            observation_revision=tick,
            seed=71,
            content_hash="cpp-dwb-grid",
        ),
        grid=GridMap(np.zeros((220, 300), dtype=np.bool_), resolution_m=0.02),
        forbidden_cells=frozenset(),
    )
    return build_controller_snapshot(
        tick_id=tick,
        simulation_time_s=simulation_time_s,
        mission_id="cpp-dwb-mission",
        robot_state=RobotState(Pose2D(2.0, 2.0), Twist2D(0.20, 0.0)),
        goal_pose=Pose2D(4.0, 2.0),
        reference_path=(Pose2D(2.0, 2.0), Pose2D(4.0, 2.0)),
        static_grid_snapshot=grid,
        validated_observation=observation,
        actor_tubes=build_actor_prediction_set(observation),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )


def _directional_prediction(snapshot) -> DirectionalPredictionSet:
    source = snapshot.actor_tubes
    return DirectionalPredictionSet(
        model_version=DIRECTIONAL_PREDICTION_VERSION,
        stream_id=source.stream_id,
        episode_id=source.episode_id,
        episode_seed=71,
        map_id=source.map_id,
        map_revision=source.map_revision,
        observation_revision=source.observation_revision,
        sequence=source.sequence,
        source_content_hash=source.source_content_hash,
        observed_at_s=source.observed_at_s,
        controller_time_s=source.controller_time_s,
        snapshot_age_s=source.snapshot_age_s,
        tubes=(
            DirectionalPredictionTube(
                track_id="track-1",
                actor_binding_id="actor-1",
                anchor_position=Point2D(2.80, 2.0),
                heading_unit=Vector2D(1.0, 0.0),
                estimated_speed_mps=0.10,
                position_sigma_m=0.03,
                velocity_sigma_mps=0.05,
                history_count=20,
                history_span_s=1.9,
                fit_rms_m=0.01,
                history_content_hash="history-tube",
            ),
        ),
        parameter_content_hash="directional-parameters",
        history_content_hash="directional-history",
        content_hash="directional-content",
    )


def _trajectories(snapshot):
    pose = snapshot.robot_state.pose
    request = DwbGeneratorRequest(
        pose=DwbPose2D(pose.x + 0.01, pose.y, pose.yaw),
        current_twist=DwbTwist2D(0.20, 0.0),
    )
    return DwbReferenceTrajectoryGenerator().generate(request).trajectories


@pytest.mark.parametrize("directional", [False, True])
def test_cpp_dwb_batch_matches_python_shared_safety(directional: bool) -> None:
    snapshot = _snapshot()
    if directional:
        snapshot = replace(snapshot, actor_tubes=_directional_prediction(snapshot))
    trajectories = _trajectories(snapshot)
    checkers = build_dynamic_trajectory_safety_checkers(
        grid_snapshot=snapshot.static_grid_snapshot,
        profile=snapshot.vehicle_profile,
    )

    native = evaluate_dwb_safety_batch(
        trajectories=trajectories,
        snapshot=snapshot,
        checkers=checkers,
    )

    assert native is not None
    assert len(native) == 217
    reason_to_failure = {
        "forbidden_zone_entry": CppDwbSafetyFailure.FORBIDDEN_ZONE,
        "static_clearance_below_minimum": CppDwbSafetyFailure.STATIC_CLEARANCE,
        "actor_clearance_below_minimum": CppDwbSafetyFailure.ACTOR_CLEARANCE,
        "prediction_set_malformed": CppDwbSafetyFailure.PREDICTION_INVALID,
    }
    for trajectory, actual in zip(trajectories, native, strict=True):
        evidence = evaluate_dynamic_trajectory_safety(
            _proposal_from_trajectory(snapshot, trajectory),
            robot_state=snapshot.robot_state,
            grid_snapshot=snapshot.static_grid_snapshot,
            prediction_set=snapshot.actor_tubes,
            profile=snapshot.vehicle_profile,
            checkers=checkers,
        )
        expected = (
            CppDwbSafetyFailure.SAFE
            if evidence.safe
            else reason_to_failure[evidence.failures[0]]
        )
        assert actual.failure is expected
        if evidence.safe:
            assert actual.minimum_static_clearance_m is not None
            assert evidence.minimum_static_clearance_m is not None
            assert isclose(
                actual.minimum_static_clearance_m,
                evidence.minimum_static_clearance_m,
                rel_tol=0.0,
                abs_tol=2e-12,
            )
            assert actual.minimum_actor_clearance_m is not None
            assert evidence.minimum_actor_clearance_m is not None
            assert isclose(
                actual.minimum_actor_clearance_m,
                evidence.minimum_actor_clearance_m,
                rel_tol=0.0,
                abs_tol=2e-12,
            )


def test_cpp_static_lower_bound_keeps_nearby_noncolliding_cell_in_minimum() -> None:
    snapshot = _snapshot(actor_x=5.0)
    occupancy = snapshot.static_grid_snapshot.grid.occupancy.copy()
    occupancy[100, 155] = True
    snapshot = replace(
        snapshot,
        static_grid_snapshot=replace(
            snapshot.static_grid_snapshot,
            grid=GridMap(occupancy, resolution_m=0.02),
        ),
    )
    post_apply = DwbPose2D(2.01, 2.0, 0.0)
    trajectory = DwbTrajectory(
        command=DwbTwist2D(0.0, 0.0),
        poses=(post_apply,) * 41,
        integration_step_s=0.05,
    )
    checkers = build_dynamic_trajectory_safety_checkers(
        grid_snapshot=snapshot.static_grid_snapshot,
        profile=snapshot.vehicle_profile,
    )

    native = evaluate_dwb_safety_batch(
        trajectories=(trajectory,),
        snapshot=snapshot,
        checkers=checkers,
    )
    expected = evaluate_dynamic_trajectory_safety(
        _proposal_from_trajectory(snapshot, trajectory),
        robot_state=snapshot.robot_state,
        grid_snapshot=snapshot.static_grid_snapshot,
        prediction_set=snapshot.actor_tubes,
        profile=snapshot.vehicle_profile,
        checkers=checkers,
    )

    assert native is not None
    assert native[0].failure is CppDwbSafetyFailure.SAFE
    assert native[0].minimum_static_clearance_m is not None
    assert expected.minimum_static_clearance_m is not None
    assert native[0].minimum_static_clearance_m < 1.0
    assert isclose(
        native[0].minimum_static_clearance_m,
        expected.minimum_static_clearance_m,
        rel_tol=0.0,
        abs_tol=2e-12,
    )
