from __future__ import annotations

from collections.abc import Callable
from math import hypot

import numpy as np

from hospital_path_lab.contracts import (
    GridSnapshot,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import (
    ActorTrack,
    DynamicHoldReason,
    DynamicMotionState,
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
    DynamicSafetyContext,
    DynamicSafetyGate,
    build_resume_authorization,
)
from hospital_path_lab.followers import DynamicPurePursuitController
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_algorithms import DynamicDwaController
from hospital_path_lab.registry import DYNAMIC_CONTROLLERS
from hospital_path_lab.simulation import simulate_dynamic_controller_pipeline
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1

ActorFactory = Callable[[int, float, RobotState], ActorTrack | None]


def _context_factory(
    *,
    occupancy: np.ndarray | None = None,
    actor_factory: ActorFactory | None = None,
    authorize_resume: bool = True,
):
    if occupancy is None:
        occupancy = np.zeros((180, 180), dtype=np.bool_)

    def factory(
        tick_id: int,
        simulation_time_s: float,
        state: RobotState,
        gate: DynamicSafetyGate,
    ) -> DynamicSafetyContext:
        sequence = tick_id // 2
        observed_at_s = sequence * 0.10
        actor = (
            actor_factory(tick_id, simulation_time_s, state)
            if actor_factory is not None
            else None
        )
        tracks = () if actor is None else (actor,)
        kind = (
            DynamicObservationFrameKind.EMPTY
            if actor is None
            else DynamicObservationFrameKind.TRACKS
        )
        frame = DynamicObservationFrame(
            stream_id="stream-v1",
            episode_id="stage4-golden",
            episode_seed=44,
            map_id="map-v1",
            map_revision=1,
            observation_revision=sequence,
            sequence=sequence,
            observed_at_s=observed_at_s,
            delivered_at_s=observed_at_s,
            frame_kind=kind,
            tracks=tracks,
            content_hash=f"observation-{sequence}-{kind.value}",
        )
        observation = DynamicObservationSnapshot(
            availability=DynamicObservationAvailability.FRESH,
            frame=frame,
            age_s=simulation_time_s - observed_at_s,
            failures=(),
            last_event_was_no_frame=False,
        )
        prediction = build_actor_prediction_set(observation)
        grid = GridSnapshot(
            metadata=SnapshotMetadata(
                map_id="map-v1",
                map_revision=1,
                mission_revision=1,
                observation_revision=sequence,
                seed=44,
                content_hash=f"grid-{sequence}",
            ),
            grid=GridMap(occupancy, resolution_m=0.02),
        )
        authorization = None
        if (
            authorize_resume
            and gate.motion_state is DynamicMotionState.HOLDING
            and gate.stop_confirmed_at_s is not None
        ):
            authorization = build_resume_authorization(
                mission_id="mission-v1",
                stop_epoch=gate.stop_epoch,
                issued_or_revalidated_at_s=simulation_time_s,
                authorization_revision=3,
            )
        return DynamicSafetyContext(
            tick_id=tick_id,
            simulation_time_s=simulation_time_s,
            mission_id="mission-v1",
            authorization_revision=3,
            grid_snapshot=grid,
            observation_snapshot=observation,
            prediction_set=prediction,
            path_still_valid=True,
            local_safety_recheck_passed=True,
            observation_safe=actor is None,
            resume_authorization=authorization,
        )

    return factory


def _snapshot_for_both_controllers(
    state: RobotState,
    goal: Pose2D,
    path: tuple[Pose2D, ...],
):
    gate = DynamicSafetyGate()
    context = _context_factory()(0, 0.0, state, gate)
    return build_controller_snapshot(
        tick_id=0,
        simulation_time_s=0.0,
        mission_id=context.mission_id,
        robot_state=state,
        goal_pose=goal,
        reference_path=path,
        static_grid_snapshot=context.grid_snapshot,
        validated_observation=context.observation_snapshot,
        actor_tubes=context.prediction_set,
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )


def test_dynamic_controllers_share_input_hash_and_free_space_target_speed() -> None:
    state = RobotState(Pose2D(1.0, 1.0), Twist2D(0.20, 0.0))
    goal = Pose2D(2.4, 1.0)
    path = (state.pose, goal)
    snapshot = _snapshot_for_both_controllers(state, goal, path)

    pp = DynamicPurePursuitController().step(snapshot)
    dwa = DynamicDwaController().step(snapshot)

    assert set(DYNAMIC_CONTROLLERS) == {"dynamic_pure_pursuit", "dynamic_dwa"}
    assert pp.input_content_hash == dwa.input_content_hash == snapshot.input_content_hash
    assert pp.observation_content_hash == dwa.observation_content_hash
    assert pp.requested_twist.linear == dwa.requested_twist.linear == 0.20


def test_no_actor_pipeline_reaches_normal_completion_for_both_controllers() -> None:
    start = RobotState(Pose2D(1.0, 1.0), Twist2D())
    goal = start.pose
    path = (Pose2D(0.8, 1.0), goal)
    signatures = []
    for controller in (DynamicPurePursuitController(), DynamicDwaController()):
        result = simulate_dynamic_controller_pipeline(
            controller,
            initial_state=start,
            reference_path=path,
            goal=goal,
            context_factory=_context_factory(),
            max_ticks=5,
        )
        assert result.completed
        assert result.static_collision_count == 0
        assert result.forbidden_entry_count == 0
        assert result.steps[-1].safety_decision.stop_epoch == 0
        signatures.append(tuple(step.safety_decision.command for step in result.steps))
    assert signatures[0] == signatures[1] == (Twist2D(), Twist2D(), Twist2D())


def test_pp_crossing_actor_stops_holds_and_resumes_with_current_epoch_authority() -> None:
    def crossing_actor(tick_id: int, _time: float, _state: RobotState):
        if tick_id >= 2:
            return None
        return ActorTrack(
            track_id="crossing-track",
            actor_binding_id="crossing-actor",
            observed_position=Point2D(1.25, 1.0),
            observed_velocity=Vector2D(0.0, 0.0),
            position_sigma_m=0.0,
            velocity_sigma_mps=0.0,
        )

    result = simulate_dynamic_controller_pipeline(
        DynamicPurePursuitController(),
        initial_state=RobotState(Pose2D(1.0, 1.0), Twist2D(0.20, 0.0)),
        reference_path=(Pose2D(1.0, 1.0), Pose2D(1.15, 1.0)),
        goal=Pose2D(1.15, 1.0),
        context_factory=_context_factory(actor_factory=crossing_actor),
        max_ticks=120,
    )

    states = tuple(step.safety_decision.motion_state for step in result.steps)
    assert result.completed
    assert DynamicMotionState.BRAKING in states
    assert DynamicMotionState.HOLDING in states
    assert any(step.safety_decision.resume_allowed for step in result.steps)
    assert result.gate_override_count > 0
    assert result.static_collision_count == result.forbidden_entry_count == 0


def test_wide_space_dwa_detours_then_reduces_reference_error_after_actor_clears() -> None:
    def moving_actor(tick_id: int, simulation_time_s: float, _state: RobotState):
        if tick_id >= 2:
            return None
        return ActorTrack(
            track_id="wide-track",
            actor_binding_id="wide-actor",
            observed_position=Point2D(1.70, 1.60 + 0.50 * simulation_time_s),
            observed_velocity=Vector2D(0.0, 0.50),
            position_sigma_m=0.0,
            velocity_sigma_mps=0.0,
        )

    result = simulate_dynamic_controller_pipeline(
        DynamicDwaController(),
        initial_state=RobotState(Pose2D(1.0, 1.0), Twist2D(0.20, 0.0)),
        reference_path=(Pose2D(1.0, 1.0), Pose2D(1.40, 1.0)),
        goal=Pose2D(1.40, 1.0),
        context_factory=_context_factory(actor_factory=moving_actor),
        max_ticks=100,
    )

    lateral_errors = [abs(step.robot_state_after.pose.y - 1.0) for step in result.steps]
    assert any(step.controller_result.requested_twist.angular < 0.0 for step in result.steps[:2])
    assert max(lateral_errors) > 0.0
    assert lateral_errors[-1] < max(lateral_errors)
    assert result.completed
    assert result.static_collision_count == result.forbidden_entry_count == 0


def test_narrow_blocked_dwa_reaches_no_candidate_hold_without_reverse() -> None:
    occupancy = np.ones((120, 180), dtype=np.bool_)
    occupancy[30:80, 10:66] = False
    result = simulate_dynamic_controller_pipeline(
        DynamicDwaController(),
        initial_state=RobotState(Pose2D(1.0, 1.0), Twist2D()),
        reference_path=(Pose2D(1.0, 1.0), Pose2D(2.4, 1.0)),
        goal=Pose2D(2.4, 1.0),
        context_factory=_context_factory(
            occupancy=occupancy,
            authorize_resume=False,
        ),
        max_ticks=8,
        stop_when_holding=True,
    )

    assert result.expected_hold_reached
    assert result.no_safe_candidate_count >= 1
    assert all(
        step.controller_result.requested_twist.linear >= 0.0 for step in result.steps
    )
    assert result.steps[-1].safety_decision.primary_hold_reason in {
        DynamicHoldReason.NO_SAFE_CANDIDATE,
        DynamicHoldReason.UNAUTHORIZED,
    }
    assert result.static_collision_count == result.forbidden_entry_count == 0


def test_new_actor_risk_restarts_braking_for_both_pipelines() -> None:
    def new_actor(tick_id: int, _time: float, state: RobotState):
        if tick_id < 2:
            return None
        return ActorTrack(
            track_id="new-risk-track",
            actor_binding_id="new-risk-actor",
            observed_position=Point2D(state.pose.x + 0.25, state.pose.y),
            observed_velocity=Vector2D(0.0, 0.0),
            position_sigma_m=0.0,
            velocity_sigma_mps=0.0,
        )

    for controller in (DynamicPurePursuitController(), DynamicDwaController()):
        result = simulate_dynamic_controller_pipeline(
            controller,
            initial_state=RobotState(Pose2D(1.0, 1.0), Twist2D(0.20, 0.0)),
            reference_path=(Pose2D(1.0, 1.0), Pose2D(2.0, 1.0)),
            goal=Pose2D(2.0, 1.0),
            context_factory=_context_factory(
                actor_factory=new_actor,
                authorize_resume=False,
            ),
            max_ticks=15,
            stop_when_holding=True,
        )
        assert any(
            step.safety_decision.motion_state is DynamicMotionState.BRAKING
            for step in result.steps[2:]
        )
        assert result.expected_hold_reached
        assert result.static_collision_count == result.forbidden_entry_count == 0


def test_same_seed_pipeline_command_state_and_event_sequence_is_deterministic() -> None:
    def signature():
        result = simulate_dynamic_controller_pipeline(
            DynamicPurePursuitController(),
            initial_state=RobotState(Pose2D(1.0, 1.0), Twist2D()),
            reference_path=(Pose2D(1.0, 1.0), Pose2D(1.10, 1.0)),
            goal=Pose2D(1.10, 1.0),
            context_factory=_context_factory(),
            max_ticks=60,
        )
        return tuple(
            (
                step.controller_result.requested_twist,
                step.safety_decision.command,
                step.safety_decision.motion_state,
                step.safety_decision.primary_hold_reason,
                step.robot_state_after,
            )
            for step in result.steps
        )

    first = signature()
    second = signature()
    assert first == second
    assert hypot(first[-1][-1].pose.x - 1.10, first[-1][-1].pose.y - 1.0) <= 0.05
