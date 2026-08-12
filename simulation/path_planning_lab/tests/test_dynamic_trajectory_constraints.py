from __future__ import annotations

from contextlib import suppress
from math import cos, pi, sin

import numpy as np
import pytest

import hospital_path_lab.dynamic_safety as dynamic_safety
from hospital_path_lab.contracts import (
    GridSnapshot,
    Pose2D,
    RobotState,
    SnapshotMetadata,
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
from hospital_path_lab.dynamic_safety import (
    DynamicTrajectorySafetyCheckers,
    build_dynamic_trajectory_safety_checkers,
    evaluate_dynamic_trajectory_safety,
)
from hospital_path_lab.dynamic_trajectory_constraints import (
    ProjectDynamicSafetyConstraintCritic,
)
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_algorithms.dwb_reference.contracts import (
    DwbGeneratorRequest,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)
from hospital_path_lab.local_algorithms.dwb_reference.core import IllegalTrajectoryError
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _snapshot(
    *,
    occupancy: np.ndarray | None = None,
    forbidden_cells: frozenset[tuple[int, int]] = frozenset(),
    actor_position: Point2D | None = None,
    robot_twist: Twist2D | None = None,
    tick_id: int = 7,
):
    simulation_time_s = tick_id * 0.05
    tracks = ()
    if actor_position is not None:
        tracks = (
            ActorTrack(
                track_id="track-1",
                actor_binding_id="actor-1",
                observed_position=actor_position,
                observed_velocity=Vector2D(0.0, 0.0),
                position_sigma_m=0.0,
                velocity_sigma_mps=0.0,
            ),
        )
    frame = DynamicObservationFrame(
        stream_id="stream-v7",
        episode_id="episode-v7",
        episode_seed=71,
        map_id="map-v7",
        map_revision=3,
        observation_revision=tick_id,
        sequence=tick_id,
        observed_at_s=simulation_time_s,
        delivered_at_s=simulation_time_s,
        frame_kind=(
            DynamicObservationFrameKind.TRACKS
            if tracks
            else DynamicObservationFrameKind.EMPTY
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
        occupancy = np.zeros((300, 300), dtype=np.bool_)
    grid = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="map-v7",
            map_revision=3,
            mission_revision=5,
            observation_revision=tick_id,
            seed=71,
            content_hash=f"grid-{tick_id}",
        ),
        grid=GridMap(occupancy, resolution_m=0.02),
        forbidden_cells=forbidden_cells,
    )
    return build_controller_snapshot(
        tick_id=tick_id,
        simulation_time_s=simulation_time_s,
        mission_id="mission-v7",
        robot_state=RobotState(
            Pose2D(2.0, 2.0),
            robot_twist if robot_twist is not None else Twist2D(0.20, 0.0),
        ),
        goal_pose=Pose2D(4.0, 2.0),
        reference_path=(Pose2D(2.0, 2.0), Pose2D(4.0, 2.0)),
        static_grid_snapshot=grid,
        validated_observation=observation,
        actor_tubes=build_actor_prediction_set(observation),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )


def _request(snapshot, *, x_offset: float = 0.0) -> DwbGeneratorRequest:
    pose = snapshot.robot_state.pose
    twist = snapshot.robot_state.twist
    duration_s = 0.05
    if abs(twist.angular) <= 1e-12:
        post_apply = DwbPose2D(
            pose.x + twist.linear * cos(pose.yaw) * duration_s + x_offset,
            pose.y + twist.linear * sin(pose.yaw) * duration_s,
            pose.yaw,
        )
    else:
        next_yaw = pose.yaw + twist.angular * duration_s
        radius = twist.linear / twist.angular
        post_apply = DwbPose2D(
            pose.x + radius * (sin(next_yaw) - sin(pose.yaw)) + x_offset,
            pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
            (next_yaw + pi) % (2.0 * pi) - pi,
        )
    return DwbGeneratorRequest(
        pose=post_apply,
        current_twist=DwbTwist2D(twist.linear, twist.angular),
    )


def _trajectory(
    request: DwbGeneratorRequest,
    *,
    linear_mps: float = 0.20,
    duration_s: float = 0.05,
) -> DwbTrajectory:
    step_s = 0.05
    steps = round(duration_s / step_s)
    poses = [request.pose]
    pose = request.pose
    for _ in range(steps):
        pose = DwbPose2D(
            pose.x_m + linear_mps * step_s,
            pose.y_m,
            pose.yaw_rad,
        )
        poses.append(pose)
    return DwbTrajectory(
        command=DwbTwist2D(linear_mps, 0.0),
        poses=tuple(poses),
        integration_step_s=step_s,
    )


def _prepared(snapshot):
    critic = ProjectDynamicSafetyConstraintCritic()
    request = _request(snapshot)
    critic.bind_snapshot(snapshot)
    assert critic.prepare(request)
    return critic, request


def test_safe_candidate_records_snapshot_provenance_and_selected_evidence() -> None:
    snapshot = _snapshot()
    critic, request = _prepared(snapshot)
    trajectory = _trajectory(request)

    assert critic.score(trajectory) == 0.0
    proposal = critic.proposal_for(trajectory)
    evidence = critic.evidence_for(trajectory)

    assert proposal is not None
    assert (
        proposal.source_tick_id,
        proposal.mission_id,
        proposal.map_id,
        proposal.map_revision,
        proposal.mission_revision,
        proposal.observation_revision,
        proposal.grid_content_hash,
        proposal.observation_content_hash,
    ) == (
        snapshot.tick_id,
        snapshot.mission_id,
        snapshot.map_id,
        snapshot.map_revision,
        snapshot.mission_revision,
        snapshot.observation_revision,
        snapshot.static_grid_snapshot.metadata.content_hash,
        snapshot.observation_content_hash,
    )
    assert proposal.trajectory[0].time_s == 0.0
    assert evidence is not None and evidence.safe

    critic.debrief(trajectory.command)
    assert critic.selected_evidence == evidence
    assert critic.selected_record is not None


def test_prepare_rejects_request_that_does_not_start_at_post_apply_pose() -> None:
    snapshot = _snapshot()
    critic = ProjectDynamicSafetyConstraintCritic()
    critic.bind_snapshot(snapshot)

    assert not critic.prepare(_request(snapshot, x_offset=0.001))
    with pytest.raises(RuntimeError, match="prepare must succeed"):
        critic.score(_trajectory(_request(snapshot)))


def test_prepare_uses_exact_arc_for_rotating_post_apply_motion() -> None:
    snapshot = _snapshot(robot_twist=Twist2D(0.20, 0.80))
    critic = ProjectDynamicSafetyConstraintCritic()
    critic.bind_snapshot(snapshot)

    exact_arc_request = _request(snapshot)
    assert critic.prepare(exact_arc_request)

    pose = snapshot.robot_state.pose
    twist = snapshot.robot_state.twist
    duration_s = 0.05
    euler_request = DwbGeneratorRequest(
        pose=DwbPose2D(
            pose.x + twist.linear * cos(pose.yaw) * duration_s,
            pose.y + twist.linear * sin(pose.yaw) * duration_s,
            pose.yaw + twist.angular * duration_s,
        ),
        current_twist=DwbTwist2D(twist.linear, twist.angular),
    )

    assert not critic.prepare(euler_request)


@pytest.mark.parametrize(
    ("hazard", "expected_reason"),
    (
        ("static", "static_clearance_below_minimum"),
        ("forbidden", "forbidden_zone_entry"),
        ("actor", "actor_clearance_below_minimum"),
    ),
)
def test_project_hard_hazards_reject_a_single_candidate(
    hazard: str,
    expected_reason: str,
) -> None:
    occupancy = np.zeros((300, 300), dtype=np.bool_)
    forbidden = frozenset()
    actor_position = None
    if hazard == "static":
        occupancy[100, 112] = True
    elif hazard == "forbidden":
        forbidden = frozenset({(111, 100)})
    else:
        actor_position = Point2D(2.45, 2.0)

    snapshot = _snapshot(
        occupancy=occupancy,
        forbidden_cells=forbidden,
        actor_position=actor_position,
    )
    critic, request = _prepared(snapshot)
    trajectory = _trajectory(request)

    with pytest.raises(IllegalTrajectoryError) as caught:
        critic.score(trajectory)

    assert caught.value.reason_code == expected_reason
    assert expected_reason in caught.value.detail
    evidence = critic.evidence_for(trajectory)
    assert evidence is not None and not evidence.safe


def test_terminal_stopping_sweep_rejects_clear_rollout_that_cannot_stop_safely() -> None:
    occupancy = np.zeros((300, 300), dtype=np.bool_)
    occupancy[100, 117] = True  # lower x edge 2.34 m
    snapshot = _snapshot(occupancy=occupancy)
    critic, request = _prepared(snapshot)
    trajectory = _trajectory(request, duration_s=0.05)

    with pytest.raises(IllegalTrajectoryError) as caught:
        critic.score(trajectory)

    assert caught.value.reason_code == "static_clearance_below_minimum"
    evidence = critic.evidence_for(trajectory)
    assert evidence is not None
    assert evidence.minimum_static_clearance_m is not None
    assert evidence.minimum_static_clearance_m < 0.08


def test_bind_and_reset_prevent_evidence_from_crossing_snapshot_boundaries() -> None:
    first = _snapshot(tick_id=7)
    critic, request = _prepared(first)
    trajectory = _trajectory(request)
    assert critic.score(trajectory) == 0.0
    assert critic.cached_candidate_count == 1

    second = _snapshot(tick_id=8)
    critic.bind_snapshot(second)
    assert critic.cached_candidate_count == 0
    assert critic.selected_evidence is None
    with pytest.raises(RuntimeError, match="prepare must succeed"):
        critic.score(trajectory)

    critic.reset()
    assert critic.bound_snapshot is None
    with pytest.raises(RuntimeError, match="bind_snapshot"):
        critic.prepare(_request(second))


@pytest.mark.parametrize("hazard", ("safe", "static", "forbidden", "actor"))
def test_prebuilt_checker_path_is_semantically_identical_to_default_path(
    hazard: str,
) -> None:
    occupancy = np.zeros((300, 300), dtype=np.bool_)
    forbidden = frozenset()
    actor_position = None
    if hazard == "static":
        occupancy[100, 112] = True
    elif hazard == "forbidden":
        forbidden = frozenset({(111, 100)})
    elif hazard == "actor":
        actor_position = Point2D(2.45, 2.0)

    snapshot = _snapshot(
        occupancy=occupancy,
        forbidden_cells=forbidden,
        actor_position=actor_position,
    )
    critic, request = _prepared(snapshot)
    trajectory = _trajectory(request)
    with suppress(IllegalTrajectoryError):
        critic.score(trajectory)
    proposal = critic.proposal_for(trajectory)
    assert proposal is not None

    default = evaluate_dynamic_trajectory_safety(
        proposal,
        robot_state=snapshot.robot_state,
        grid_snapshot=snapshot.static_grid_snapshot,
        prediction_set=snapshot.actor_tubes,
        profile=snapshot.vehicle_profile,
    )
    prebuilt = evaluate_dynamic_trajectory_safety(
        proposal,
        robot_state=snapshot.robot_state,
        grid_snapshot=snapshot.static_grid_snapshot,
        prediction_set=snapshot.actor_tubes,
        profile=snapshot.vehicle_profile,
        checkers=build_dynamic_trajectory_safety_checkers(
            grid_snapshot=snapshot.static_grid_snapshot,
            profile=snapshot.vehicle_profile,
        ),
    )

    assert prebuilt == default


def test_critic_constructs_checker_pair_once_and_reuses_it_for_all_candidates(
    monkeypatch,
) -> None:
    original_checker = dynamic_safety.CollisionChecker
    constructions: list[tuple[object, object, frozenset[tuple[int, int]]]] = []

    def counting_checker(grid, profile, *, forbidden_cells=frozenset()):
        constructions.append((grid, profile, forbidden_cells))
        return original_checker(
            grid,
            profile,
            forbidden_cells=forbidden_cells,
        )

    monkeypatch.setattr(dynamic_safety, "CollisionChecker", counting_checker)
    snapshot = _snapshot()
    critic = ProjectDynamicSafetyConstraintCritic()

    critic.bind_snapshot(snapshot)
    assert len(constructions) == 2
    assert constructions[0][2] == frozenset()
    assert constructions[1][2] == snapshot.static_grid_snapshot.forbidden_cells

    request = _request(snapshot)
    assert critic.prepare(request)
    for linear_mps in (0.10, 0.15, 0.20):
        assert critic.score(_trajectory(request, linear_mps=linear_mps)) == 0.0

    assert len(constructions) == 2


def test_prebuilt_checkers_cannot_cross_grid_snapshot_boundaries() -> None:
    first = _snapshot(tick_id=7)
    second = _snapshot(tick_id=8)
    critic, request = _prepared(first)
    trajectory = _trajectory(request)
    assert critic.score(trajectory) == 0.0
    proposal = critic.proposal_for(trajectory)
    assert proposal is not None
    checkers = build_dynamic_trajectory_safety_checkers(
        grid_snapshot=first.static_grid_snapshot,
        profile=first.vehicle_profile,
    )

    with pytest.raises(ValueError, match="different grid snapshot"):
        evaluate_dynamic_trajectory_safety(
            proposal,
            robot_state=second.robot_state,
            grid_snapshot=second.static_grid_snapshot,
            prediction_set=second.actor_tubes,
            profile=second.vehicle_profile,
            checkers=checkers,
        )


class _DuckTypedAlwaysSafeChecker:
    """Adversarial checker with the old public bundle's accepted attributes."""

    def __init__(self, snapshot, *, forbidden_cells) -> None:
        self.grid = snapshot.static_grid_snapshot.grid
        self.profile = snapshot.vehicle_profile
        self.forbidden_cells = forbidden_cells
        self.use_optimized_geometry = True

    def clearance(self, pose) -> float:
        del pose
        return 999.0

    def pose_enters_forbidden(self, pose) -> bool:
        del pose
        return False


def _static_hazard_proposal():
    occupancy = np.zeros((300, 300), dtype=np.bool_)
    occupancy[100, 112] = True
    snapshot = _snapshot(occupancy=occupancy)
    critic, request = _prepared(snapshot)
    trajectory = _trajectory(request)
    with suppress(IllegalTrajectoryError):
        critic.score(trajectory)
    proposal = critic.proposal_for(trajectory)
    assert proposal is not None
    return snapshot, proposal


def test_checker_bundle_constructor_is_factory_only() -> None:
    snapshot, _ = _static_hazard_proposal()
    physical = _DuckTypedAlwaysSafeChecker(snapshot, forbidden_cells=frozenset())
    combined = _DuckTypedAlwaysSafeChecker(
        snapshot,
        forbidden_cells=snapshot.static_grid_snapshot.forbidden_cells,
    )

    with pytest.raises(TypeError, match="must be created by"):
        DynamicTrajectorySafetyCheckers(
            snapshot.static_grid_snapshot,
            snapshot.vehicle_profile,
            physical,
            combined,
        )


def test_factory_registry_rejects_forged_checker_bundle() -> None:
    snapshot, proposal = _static_hazard_proposal()
    issued = build_dynamic_trajectory_safety_checkers(
        grid_snapshot=snapshot.static_grid_snapshot,
        profile=snapshot.vehicle_profile,
    )
    forged = object.__new__(DynamicTrajectorySafetyCheckers)
    for attribute in (
        "grid_snapshot",
        "grid_source",
        "profile",
        "forbidden_cells_source",
        "physical_checker",
        "combined_checker",
        "_factory_capability",
    ):
        object.__setattr__(forged, attribute, getattr(issued, attribute))

    with pytest.raises(ValueError, match="not issued by the checker factory"):
        evaluate_dynamic_trajectory_safety(
            proposal,
            robot_state=snapshot.robot_state,
            grid_snapshot=snapshot.static_grid_snapshot,
            prediction_set=snapshot.actor_tubes,
            profile=snapshot.vehicle_profile,
            checkers=forged,
        )


def test_factory_issued_bundle_rejects_duck_typed_checker_injection() -> None:
    snapshot, proposal = _static_hazard_proposal()
    checkers = build_dynamic_trajectory_safety_checkers(
        grid_snapshot=snapshot.static_grid_snapshot,
        profile=snapshot.vehicle_profile,
    )
    object.__setattr__(
        checkers,
        "physical_checker",
        _DuckTypedAlwaysSafeChecker(snapshot, forbidden_cells=frozenset()),
    )
    object.__setattr__(
        checkers,
        "combined_checker",
        _DuckTypedAlwaysSafeChecker(
            snapshot,
            forbidden_cells=snapshot.static_grid_snapshot.forbidden_cells,
        ),
    )

    with pytest.raises(ValueError, match="do not match the safety inputs"):
        evaluate_dynamic_trajectory_safety(
            proposal,
            robot_state=snapshot.robot_state,
            grid_snapshot=snapshot.static_grid_snapshot,
            prediction_set=snapshot.actor_tubes,
            profile=snapshot.vehicle_profile,
            checkers=checkers,
        )
