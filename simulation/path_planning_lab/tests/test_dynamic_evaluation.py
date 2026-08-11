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
    ACTOR_RADIUS_M,
    ActorState,
    ActorTrack,
    ControllerCommandResult,
    DynamicMotionState,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    DynamicSafetyDecision,
    DynamicSafetyEventCounters,
    Point2D,
    Vector2D,
    controller_snapshot_content_hash,
)
from hospital_path_lab.dynamic_evaluation import (
    EVALUATOR_FREQUENCY_HZ,
    evaluate_dynamic_pipeline,
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
from hospital_path_lab.simulation import (
    DynamicControllerPipelineResult,
    DynamicControllerPipelineStep,
    simulate_dynamic_controller_pipeline,
)


def _grid(tick_id: int, *, forbidden_cells=frozenset()) -> GridSnapshot:
    sequence = tick_id // 2
    return GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="stage5-map",
            map_revision=1,
            mission_revision=1,
            observation_revision=sequence,
            seed=5,
            content_hash=f"stage5-grid-{sequence}",
        ),
        grid=GridMap(np.zeros((120, 120), dtype=np.bool_), resolution_m=0.02),
        forbidden_cells=forbidden_cells,
    )


def _context_factory(actor_until_tick: int = 0):
    def factory(
        tick_id: int,
        simulation_time_s: float,
        _state: RobotState,
        gate: DynamicSafetyGate,
    ) -> DynamicSafetyContext:
        sequence = tick_id // 2
        actor = (
            ActorTrack(
                track_id="synthetic-track",
                actor_binding_id="synthetic-actor",
                observed_position=Point2D(1.25, 1.0),
                observed_velocity=Vector2D(0.0, 0.0),
                position_sigma_m=0.0,
                velocity_sigma_mps=0.0,
            )
            if tick_id < actor_until_tick
            else None
        )
        frame = DynamicObservationFrame(
            stream_id="stage5-stream",
            episode_id="stage5-episode",
            episode_seed=5,
            map_id="stage5-map",
            map_revision=1,
            observation_revision=sequence,
            sequence=sequence,
            observed_at_s=sequence * 0.10,
            delivered_at_s=sequence * 0.10,
            frame_kind=(
                DynamicObservationFrameKind.TRACKS
                if actor is not None
                else DynamicObservationFrameKind.EMPTY
            ),
            tracks=() if actor is None else (actor,),
            content_hash=f"stage5-observation-{sequence}-{actor is not None}",
        )
        observation = DynamicObservationSnapshot(
            availability=DynamicObservationAvailability.FRESH,
            frame=frame,
            age_s=simulation_time_s - frame.observed_at_s,
            failures=(),
            last_event_was_no_frame=False,
        )
        authorization = None
        if (
            gate.motion_state is DynamicMotionState.HOLDING
            and gate.stop_confirmed_at_s is not None
        ):
            authorization = build_resume_authorization(
                mission_id="stage5-mission",
                stop_epoch=gate.stop_epoch,
                issued_or_revalidated_at_s=simulation_time_s,
                authorization_revision=1,
            )
        return DynamicSafetyContext(
            tick_id=tick_id,
            simulation_time_s=simulation_time_s,
            mission_id="stage5-mission",
            authorization_revision=1,
            grid_snapshot=_grid(tick_id),
            observation_snapshot=observation,
            prediction_set=build_actor_prediction_set(observation),
            path_still_valid=True,
            local_safety_recheck_passed=True,
            observation_safe=actor is None,
            resume_authorization=authorization,
        )

    return factory


def _stationary_pipeline(*, actor_until_tick: int = 0):
    start = RobotState(Pose2D(1.0, 1.0), Twist2D())
    return simulate_dynamic_controller_pipeline(
        DynamicPurePursuitController(),
        initial_state=start,
        reference_path=(Pose2D(0.80, 1.0), start.pose),
        goal=start.pose,
        context_factory=_context_factory(actor_until_tick),
        max_ticks=40,
    )


def _evaluate(pipeline, actor_provider, *, progressable=True):
    return evaluate_dynamic_pipeline(
        pipeline,
        episode_id="stage5-evaluator-test",
        expectation_category="wait_and_resume",
        progressable=progressable,
        reference_path=(Pose2D(0.80, 1.0), Pose2D(1.0, 1.0)),
        goal_pose=Pose2D(1.0, 1.0),
        actor_states_at=actor_provider,
        grid_snapshot_at=_grid,
    )


def test_200hz_ground_truth_evaluator_passes_empty_stationary_trace() -> None:
    result = _evaluate(_stationary_pipeline(), lambda _time: ())

    assert result.evaluator_frequency_hz == EVALUATOR_FREQUENCY_HZ == 200.0
    assert result.hard_safety.passed
    assert result.functional_qualified
    assert result.metrics.completion_time_s is not None
    assert result.metrics.minimum_surface_clearance_m is not None
    assert result.metrics.minimum_surface_clearance_m > 0.08


def test_exact_actor_surface_clearance_threshold_is_a_hard_pass() -> None:
    actor_x = 1.0 + 0.44 / 2.0 + ACTOR_RADIUS_M + 0.08

    def actor_provider(_time: float):
        return (
            ActorState(
                actor_id="threshold-actor",
                position=Point2D(actor_x, 1.0),
                velocity=Vector2D(0.0, 0.0),
                radius_m=ACTOR_RADIUS_M,
                trajectory_revision=1,
            ),
        )

    result = _evaluate(_stationary_pipeline(), actor_provider)

    assert result.hard_safety.passed
    assert isclose(
        result.metrics.minimum_surface_clearance_m or 0.0,
        0.08,
        abs_tol=1e-9,
    )


def test_swept_margin_detects_collision_between_5ms_samples() -> None:
    actor_x = 1.0 + 0.44 / 2.0 + ACTOR_RADIUS_M - 1e-7

    def actor_provider(time_s: float):
        if time_s > 0.005 + 1e-12:
            return ()
        return (
            ActorState(
                actor_id="between-samples",
                position=Point2D(actor_x, 1.0 + 0.50 * (time_s - 0.0025)),
                velocity=Vector2D(0.0, 0.50),
                radius_m=ACTOR_RADIUS_M,
                trajectory_revision=1,
            ),
        )

    result = _evaluate(_stationary_pipeline(), actor_provider)

    assert not result.hard_safety.passed
    assert result.hard_safety.collision_count > 0
    assert "ground_truth_collision" in result.hard_safety.failures


def test_authorized_safety_hold_is_not_planner_deadlock() -> None:
    pipeline = simulate_dynamic_controller_pipeline(
        DynamicPurePursuitController(),
        initial_state=RobotState(Pose2D(1.0, 1.0), Twist2D(0.20, 0.0)),
        reference_path=(Pose2D(1.0, 1.0), Pose2D(1.15, 1.0)),
        goal=Pose2D(1.15, 1.0),
        context_factory=_context_factory(actor_until_tick=2),
        max_ticks=120,
    )
    result = evaluate_dynamic_pipeline(
        pipeline,
        episode_id="hold-not-deadlock",
        expectation_category="wait_and_resume",
        progressable=True,
        reference_path=(Pose2D(1.0, 1.0), Pose2D(1.15, 1.0)),
        goal_pose=Pose2D(1.15, 1.0),
        actor_states_at=lambda _time: (),
        grid_snapshot_at=_grid,
    )

    assert result.metrics.safety_hold_duration_s > 0.0
    assert not result.metrics.planner_deadlock
    assert result.functional_qualified


def test_forbidden_entry_is_a_hard_failure_not_a_metric_only_warning() -> None:
    pipeline = _stationary_pipeline()
    forbidden_cell = _grid(0).grid.world_to_cell(Pose2D(1.0, 1.0))

    result = evaluate_dynamic_pipeline(
        pipeline,
        episode_id="forbidden-false-pass-regression",
        expectation_category="no_safe_solution",
        progressable=False,
        reference_path=(Pose2D(0.80, 1.0), Pose2D(1.0, 1.0)),
        goal_pose=Pose2D(1.0, 1.0),
        actor_states_at=lambda _time: (),
        grid_snapshot_at=lambda tick: _grid(
            tick,
            forbidden_cells=frozenset({forbidden_cell}),
        ),
    )

    assert not result.hard_safety.passed
    assert result.hard_safety.forbidden_entry_count > 0
    assert "forbidden_zone_entry" in result.hard_safety.failures


def test_rejoin_and_reference_projection_overtaking_have_explicit_oracles() -> None:
    grid = _grid(0)
    angular_commands = (0.8,) * 20 + (-0.8,) * 40 + (0.8,) * 20 + (0.0,) * 20
    state = RobotState(Pose2D(0.50, 1.0), Twist2D(0.20, angular_commands[0]))
    steps: list[DynamicControllerPipelineStep] = []
    for tick_id, _angular in enumerate(angular_commands):
        next_pose = _test_integrate(state.pose, state.twist, 0.05)
        next_twist = Twist2D(
            0.20,
            angular_commands[min(tick_id + 1, len(angular_commands) - 1)],
        )
        input_hash = controller_snapshot_content_hash(
            tick_id=tick_id,
            mission_id="stage5-mission",
            map_id=grid.metadata.map_id,
            map_revision=grid.metadata.map_revision,
            mission_revision=grid.metadata.mission_revision,
            observation_revision=grid.metadata.observation_revision,
            grid_content_hash=grid.metadata.content_hash,
            observation_content_hash="manual-observation",
        )
        controller_result = ControllerCommandResult(
            controller_name="manual-controller",
            source_tick_id=tick_id,
            status=PlanStatus.FOUND,
            requested_twist=next_twist,
            predicted_trajectory=(),
            failure_reason=None,
            decision_trace=("manual_stage5_metric_oracle",),
            mission_id="stage5-mission",
            map_id=grid.metadata.map_id,
            map_revision=grid.metadata.map_revision,
            mission_revision=grid.metadata.mission_revision,
            observation_revision=grid.metadata.observation_revision,
            grid_content_hash=grid.metadata.content_hash,
            observation_content_hash="manual-observation",
            input_content_hash=input_hash,
            elapsed_ns=0,
        )
        decision = DynamicSafetyDecision(
            tick_id=tick_id,
            source_tick_id=tick_id,
            motion_state=DynamicMotionState.MOVING,
            stop_epoch=0,
            command=next_twist,
            proposal_accepted=True,
            resume_allowed=False,
            primary_hold_reason=None,
            consecutive_stop_ticks=0,
            consecutive_safe_frames=0,
            minimum_static_clearance_m=1.0,
            minimum_actor_clearance_m=None,
            counters=DynamicSafetyEventCounters(),
        )
        next_state = RobotState(next_pose, next_twist)
        steps.append(
            DynamicControllerPipelineStep(
                tick_id=tick_id,
                simulation_time_s=tick_id * 0.05,
                controller_result=controller_result,
                safety_decision=decision,
                robot_state_before=state,
                robot_state_after=next_state,
                gate_overrode_controller=False,
                static_collision=False,
                forbidden_entry=False,
            )
        )
        state = next_state
    pipeline = DynamicControllerPipelineResult(
        controller_name="manual-controller",
        simulation_only=True,
        status=PlanStatus.NO_PATH,
        completed=False,
        expected_hold_reached=False,
        final_state=state,
        steps=tuple(steps),
        static_collision_count=0,
        forbidden_entry_count=0,
        gate_override_count=0,
        controller_stop_request_count=0,
        no_safe_candidate_count=0,
        failure_reason="metric_oracle_only",
    )

    def actor_provider(_time: float):
        return (
            ActorState(
                actor_id="projected-order-actor",
                position=Point2D(0.90, 0.56),
                velocity=Vector2D(0.0, 0.0),
                radius_m=ACTOR_RADIUS_M,
                trajectory_revision=1,
            ),
        )

    result = evaluate_dynamic_pipeline(
        pipeline,
        episode_id="rejoin-overtake-oracle",
        expectation_category="metric_oracle",
        progressable=False,
        reference_path=(Pose2D(0.50, 1.0), Pose2D(2.0, 1.0)),
        goal_pose=Pose2D(2.0, 1.0),
        actor_states_at=actor_provider,
        grid_snapshot_at=lambda _tick: grid,
    )

    assert result.metrics.maximum_reference_deviation_m > 0.10
    assert result.metrics.rejoin_observed
    assert result.metrics.overtaking_observed
    assert result.hard_safety.nonfinite_or_provenance_failure_count == 0


def _test_integrate(pose: Pose2D, twist: Twist2D, dt_s: float) -> Pose2D:
    from math import cos, pi, sin

    return Pose2D(
        pose.x + twist.linear * cos(pose.yaw) * dt_s,
        pose.y + twist.linear * sin(pose.yaw) * dt_s,
        (pose.yaw + twist.angular * dt_s + pi) % (2.0 * pi) - pi,
    )
