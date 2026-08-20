"""Public contract tests for the thin, stateful R7 backend runtime facade."""

from __future__ import annotations

from subprocess import run
from sys import executable
from time import sleep

from pytest import raises

from hospital_path_lab.dynamic_contracts import (
    DynamicHoldReason,
    DynamicMotionState,
)
from hospital_path_lab.dynamic_directional_prediction import DirectionalActorPredictor
from hospital_path_lab.dynamic_observation import (
    DynamicObservationProfileName,
    DynamicObservationValidator,
)
from hospital_path_lab.dynamic_safety import (
    DYNAMIC_COMMAND_DEADLINE_S,
    DynamicSafetyGate,
)
from hospital_path_lab.persistent_controller_pipeline import PersistentControllerPipeline
from hospital_path_lab.persistent_rpp_controller import PersistentRppController
from hospital_path_lab.runtime import (
    R7Runtime,
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
from hospital_path_lab.runtime.adapters import (
    build_observation_source,
    build_runtime_grid_snapshot,
    grid_snapshot_for_observation,
    observation_profile_for,
    to_observation_frame,
    to_robot_state,
)
from hospital_path_lab.runtime.reference import build_runtime_follow_reference


def _runtime_map() -> RuntimeMap:
    return RuntimeMap(
        map_id="runtime-public-map",
        map_revision=1,
        occupancy_rows=tuple(tuple(False for _ in range(200)) for _ in range(200)),
    )


def _mission(
    *,
    mission_id: str = "runtime-public-mission",
    session_seed: int = 101,
) -> RuntimeMission:
    return RuntimeMission(
        mission_id=mission_id,
        mission_revision=1,
        runtime_map=_runtime_map(),
        start_pose=RuntimePose(0.5, 0.5, 0.0),
        goal_pose=RuntimePose(2.0, 0.5, 0.0),
        reference_path=(RuntimePose(0.5, 0.5, 0.0), RuntimePose(2.0, 0.5, 0.0)),
        observation_stream_id="runtime-public-camera",
        observation_session_seed=session_seed,
    )


def _config() -> RuntimeConfig:
    return RuntimeConfig(
        controller_kind=RuntimeControllerKind.RPP,
        observation_profile=DynamicObservationProfileName.FUNCTIONAL_IDEAL,
        require_native_dwb=False,
    )


def _robot(
    *,
    x_m: float = 0.5,
    y_m: float = 0.5,
    yaw_rad: float = 0.0,
    linear_mps: float = 0.0,
    angular_radps: float = 0.0,
) -> RuntimeRobotState:
    return RuntimeRobotState(
        pose=RuntimePose(x_m, y_m, yaw_rad),
        linear_mps=linear_mps,
        angular_radps=angular_radps,
    )


def _empty_observation(sequence: int) -> RuntimeObservation:
    return RuntimeObservation(
        sequence=sequence,
        observation_revision=sequence,
        observed_at_s=sequence * 0.1,
    )


def _actor_observation(
    sequence: int,
    *,
    start_x_m: float,
    speed_mps: float,
) -> RuntimeObservation:
    observed_at_s = sequence * 0.1
    return RuntimeObservation(
        sequence=sequence,
        observation_revision=sequence,
        observed_at_s=observed_at_s,
        actors=(
            RuntimeActorObservation(
                track_id="public-actor-track",
                actor_binding_id="public-actor-binding",
                x_m=start_x_m + speed_mps * observed_at_s,
                y_m=0.5,
                vx_mps=speed_mps,
                vy_mps=0.0,
            ),
        ),
    )


def _step(
    runtime: R7Runtime,
    tick: int,
    *,
    observation: RuntimeObservation | None = None,
    robot: RuntimeRobotState | None = None,
):
    return runtime.step(
        RuntimeStepInput(
            control_tick=tick,
            robot=_robot() if robot is None else robot,
            observation=observation,
        )
    )


def _start_with_empty_frame(runtime: R7Runtime):
    initial = _step(runtime, 0)
    latency_hold = _step(runtime, 1)
    first_moving = _step(runtime, 2, observation=_empty_observation(0))
    return initial, latency_hold, first_moving


def test_a_empty_frames_run_through_one_persistent_runtime_instance() -> None:
    runtime = R7Runtime(_config())
    runtime.start_mission(_mission())

    initial, latency_hold, first_moving = _start_with_empty_frame(runtime)
    next_moving = _step(runtime, 3)
    fresh_empty = _step(runtime, 4, observation=_empty_observation(1))

    assert initial.motion_state is DynamicMotionState.HOLDING
    assert latency_hold.motion_state is DynamicMotionState.HOLDING
    assert first_moving.motion_state is DynamicMotionState.MOVING
    assert next_moving.motion_state is DynamicMotionState.MOVING
    assert fresh_empty.motion_state is DynamicMotionState.MOVING
    assert fresh_empty.linear_mps > 0.0
    assert runtime.diagnostics.mission_id == "runtime-public-mission"
    assert runtime.diagnostics.next_control_tick == 5
    assert runtime.diagnostics.predictor_history_counts == ()


def test_b_actor_frames_reach_existing_directional_predictor_without_bypass() -> None:
    runtime = R7Runtime(_config())
    runtime.start_mission(_mission())

    result = None
    for tick in range(41):
        observation = None
        if tick >= 2 and tick % 2 == 0:
            sequence = (tick - 2) // 2
            observation = _actor_observation(
                sequence,
                start_x_m=2.5,
                speed_mps=0.2,
            )
        result = _step(runtime, tick, observation=observation)

    assert result is not None
    assert result.motion_state is DynamicMotionState.MOVING
    assert runtime.diagnostics.predictor_status == "ready"
    assert runtime.diagnostics.predictor_history_counts == (("public-actor-binding", 20),)


def test_c_actor_clearance_hazard_brakes_then_holds_without_motion() -> None:
    runtime = R7Runtime(_config())
    runtime.start_mission(_mission())

    results = []
    for tick in range(43):
        observation = None
        if tick >= 2 and tick % 2 == 0:
            sequence = (tick - 2) // 2
            observation = _actor_observation(
                sequence,
                start_x_m=0.7,
                speed_mps=0.05,
            )
        results.append(_step(runtime, tick, observation=observation))

    braking = results[40]
    holding = results[42]
    assert braking.motion_state is DynamicMotionState.BRAKING
    assert "actor_clearance_below_minimum" in braking.failure_reasons
    assert holding.motion_state is DynamicMotionState.HOLDING
    assert holding.stop_epoch == 1
    assert all(result.linear_mps == 0.0 for result in results[40:])


def test_d_intermediate_20hz_ticks_keep_predictor_history_without_duplicate_frame() -> None:
    runtime = R7Runtime(_config())
    runtime.start_mission(_mission())

    _step(runtime, 0)
    _step(runtime, 1)
    _step(
        runtime,
        2,
        observation=_actor_observation(0, start_x_m=2.5, speed_mps=0.2),
    )
    first_count = runtime.diagnostics.predictor_history_counts
    _step(runtime, 3)
    duplicate_tick_count = runtime.diagnostics.predictor_history_counts
    _step(
        runtime,
        4,
        observation=_actor_observation(1, start_x_m=2.5, speed_mps=0.2),
    )

    assert first_count == (("public-actor-binding", 1),)
    assert duplicate_tick_count == first_count
    assert runtime.diagnostics.predictor_history_counts == (("public-actor-binding", 2),)


def test_e_reset_discards_mission_and_prediction_history_before_new_mission() -> None:
    runtime = R7Runtime(_config())
    runtime.start_mission(_mission())
    _step(runtime, 0)
    _step(runtime, 1)
    _step(
        runtime,
        2,
        observation=_actor_observation(0, start_x_m=2.5, speed_mps=0.2),
    )
    assert runtime.diagnostics.predictor_history_counts == (("public-actor-binding", 1),)

    runtime.reset()
    assert not runtime.mission_started
    assert runtime.diagnostics.mission_id is None

    runtime.start_mission(_mission(mission_id="runtime-second-mission", session_seed=202))
    _step(runtime, 0)
    _step(runtime, 1)
    _step(
        runtime,
        2,
        observation=_actor_observation(0, start_x_m=2.5, speed_mps=0.2),
    )

    assert runtime.diagnostics.mission_id == "runtime-second-mission"
    assert runtime.diagnostics.predictor_history_counts == (("public-actor-binding", 1),)


def test_f_stale_and_provenance_invalid_observations_fail_closed() -> None:
    runtime = R7Runtime(_config())
    runtime.start_mission(_mission())
    _, _, moving = _start_with_empty_frame(runtime)
    stale_results = [_step(runtime, tick) for tick in range(3, 10)]

    assert moving.motion_state is DynamicMotionState.MOVING
    assert stale_results[4].motion_state is DynamicMotionState.BRAKING
    assert stale_results[-1].motion_state is DynamicMotionState.HOLDING
    assert all(result.linear_mps == 0.0 for result in stale_results[4:])

    invalid_runtime = R7Runtime(_config())
    invalid_runtime.start_mission(_mission())
    _step(invalid_runtime, 0)
    _step(invalid_runtime, 1)
    invalid = _step(
        invalid_runtime,
        2,
        observation=RuntimeObservation(
            sequence=0,
            observation_revision=0,
            observed_at_s=0.0,
            map_id="wrong-map",
        ),
    )
    normalized_actor = RuntimeActorObservation("actor", "binding", 1, 2, 0, 0)

    assert invalid.motion_state is DynamicMotionState.HOLDING
    assert invalid.linear_mps == 0.0
    assert "map_id_mismatch" in invalid.failure_reasons
    assert normalized_actor.vx_mps == 0.0
    assert isinstance(normalized_actor.vy_mps, float)


def test_control_tick_gap_stops_the_mission_without_runtime_catch_up() -> None:
    runtime = R7Runtime(_config())
    runtime.start_mission(_mission())
    _start_with_empty_frame(runtime)

    # The façade must brake from the newest valid external twist, not from
    # the zero-velocity pose it saw at the prior valid tick.
    gap = _step(runtime, 5, robot=_robot(linear_mps=0.5))

    assert gap.motion_state is DynamicMotionState.BRAKING
    assert 0.0 < gap.linear_mps < 0.5
    with raises(RuntimeError, match="actual stop"):
        runtime.reset()

    # These are not controller catch-up commands. They are invalid-source
    # gate ticks that use the actual stopped chassis report to finish the
    # existing BRAKING -> HOLDING confirmation.
    braking = _step(runtime, 6, robot=_robot())
    still_braking = _step(runtime, 7, robot=_robot())
    confirmed = _step(runtime, 8, robot=_robot())

    assert braking.linear_mps == 0.0
    assert still_braking.motion_state is DynamicMotionState.BRAKING
    assert confirmed.motion_state is DynamicMotionState.HOLDING
    assert confirmed.linear_mps == 0.0
    assert confirmed.stop_epoch == 1
    assert confirmed.stop_reason == "runtime_control_tick_mismatch"
    runtime.reset()
    with raises(RuntimeError, match="new mission id or mission revision"):
        runtime.start_mission(_mission())
    runtime.start_mission(_mission(mission_id="runtime-post-stop-mission"))


def test_prestart_tick_gap_uses_shared_gate_until_actual_stop() -> None:
    runtime = R7Runtime(_config())
    runtime.start_mission(_mission())
    moving_robot = _robot(x_m=0.75, linear_mps=0.5)

    gap = _step(runtime, 1, robot=moving_robot)

    assert gap.motion_state is DynamicMotionState.BRAKING
    assert 0.0 < gap.linear_mps < 0.5
    with raises(RuntimeError, match="actual stop"):
        runtime.reset()

    stopped_robot = _robot(x_m=0.75)
    _step(runtime, 2, robot=stopped_robot)
    _step(runtime, 3, robot=stopped_robot)
    confirmed = _step(runtime, 4, robot=stopped_robot)

    assert confirmed.motion_state is DynamicMotionState.HOLDING
    assert confirmed.stop_epoch == 1
    runtime.reset()
    with raises(RuntimeError, match="new mission id or mission revision"):
        runtime.start_mission(_mission())


def test_missing_scheduled_observation_is_not_a_normal_intertick() -> None:
    runtime = R7Runtime(_config())
    runtime.start_mission(_mission())
    _start_with_empty_frame(runtime)

    intertick = _step(runtime, 3)
    due_slot_missing = _step(runtime, 4)

    assert intertick.motion_state is DynamicMotionState.MOVING
    assert due_slot_missing.motion_state is DynamicMotionState.MOVING
    assert due_slot_missing.observation_status == "fresh"
    assert due_slot_missing.prediction_status == "empty_frame"
    assert runtime.diagnostics.last_event_was_no_frame is True


def test_g_facade_matches_direct_persistent_pipeline_with_external_pose_sync() -> None:
    mission = _mission()
    config = _config()
    runtime = R7Runtime(config)
    runtime.start_mission(mission)
    _step(runtime, 0)
    _step(runtime, 1)
    facade_first = _step(runtime, 2, observation=_empty_observation(0))
    external_robot = _robot(x_m=0.65)
    facade_second = _step(runtime, 3, robot=external_robot)

    base_grid = build_runtime_grid_snapshot(mission)
    profile = observation_profile_for(config.observation_profile)
    validator = DynamicObservationValidator(build_observation_source(mission), profile)
    frame = to_observation_frame(
        _empty_observation(0),
        source=validator.source,
        profile=profile,
    )
    accepted = validator.accept(frame, received_at_s=0.1)
    assert accepted.accepted
    first_snapshot = validator.snapshot(control_time_s=0.1)
    predictor = DirectionalActorPredictor()
    first_prediction = predictor.update(first_snapshot)
    context, reference, validation = build_runtime_follow_reference(
        mission,
        grid_snapshot=base_grid,
        valid_from_tick=2,
    )
    direct = PersistentControllerPipeline(
        controller=PersistentRppController(),
        build_context=context,
        full_reference=reference,
        validation=validation,
        initial_robot_state=to_robot_state(_robot()),
        gate=DynamicSafetyGate(),
        authorization_revision=mission.authorization_revision,
        initial_tick=2,
    )
    direct.synchronize_external_robot_state(to_robot_state(_robot()))
    direct_first = direct.step(
        observation_snapshot=first_snapshot,
        prediction_set=first_prediction.prediction_set,
        computation_time_s=None,
        observation_safe=True,
        grid_snapshot=grid_snapshot_for_observation(base_grid, observation_revision=0),
    )
    second_snapshot = validator.snapshot(control_time_s=0.15)
    second_prediction = predictor.update(second_snapshot)
    direct.synchronize_external_robot_state(to_robot_state(external_robot))
    direct_second = direct.step(
        observation_snapshot=second_snapshot,
        prediction_set=second_prediction.prediction_set,
        computation_time_s=None,
        observation_safe=True,
        grid_snapshot=grid_snapshot_for_observation(base_grid, observation_revision=0),
    )

    assert facade_first.linear_mps == direct_first.safety_decision.command.linear
    assert facade_first.angular_radps == direct_first.safety_decision.command.angular
    assert facade_first.motion_state is direct_first.safety_decision.motion_state
    assert facade_second.linear_mps == direct_second.safety_decision.command.linear
    assert facade_second.angular_radps == direct_second.safety_decision.command.angular
    assert facade_second.motion_state is direct_second.safety_decision.motion_state
    assert facade_second.stop_epoch == direct_second.safety_decision.stop_epoch


def test_runtime_measures_controller_time_when_the_pipeline_requests_it() -> None:
    class SlowRppController(PersistentRppController):
        def step(self, tick_input):
            sleep(DYNAMIC_COMMAND_DEADLINE_S + 0.01)
            return super().step(tick_input)

    mission = _mission()
    base_grid = build_runtime_grid_snapshot(mission)
    profile = observation_profile_for(DynamicObservationProfileName.FUNCTIONAL_IDEAL)
    validator = DynamicObservationValidator(build_observation_source(mission), profile)
    frame = to_observation_frame(
        _empty_observation(0),
        source=validator.source,
        profile=profile,
    )
    assert validator.accept(frame, received_at_s=0.1).accepted
    snapshot = validator.snapshot(control_time_s=0.1)
    prediction = DirectionalActorPredictor().update(snapshot)
    context, reference, validation = build_runtime_follow_reference(
        mission,
        grid_snapshot=base_grid,
        valid_from_tick=2,
    )
    pipeline = PersistentControllerPipeline(
        controller=SlowRppController(),
        build_context=context,
        full_reference=reference,
        validation=validation,
        initial_robot_state=to_robot_state(_robot()),
        gate=DynamicSafetyGate(),
        initial_tick=2,
    )

    result = pipeline.step(
        observation_snapshot=snapshot,
        prediction_set=prediction.prediction_set,
        computation_time_s=None,
        observation_safe=True,
        grid_snapshot=grid_snapshot_for_observation(base_grid, observation_revision=0),
    )

    assert result.proposal.computation_time_s > DYNAMIC_COMMAND_DEADLINE_S
    assert result.safety_decision.motion_state is DynamicMotionState.BRAKING
    assert result.safety_decision.primary_hold_reason is DynamicHoldReason.DEADLINE


def test_runtime_import_does_not_load_hidden_or_corpus_modules() -> None:
    script = """
import sys
import hospital_path_lab.runtime
banned = (
    'hospital_path_lab.dynamic_corpus',
    'hospital_path_lab.dynamic_evaluation',
    'hospital_path_lab.dynamic_runner',
    'hospital_path_lab.r5b_temporal_evidence',
    'hospital_path_lab.r5b_temporal_authorization',
    'hospital_path_lab.r5c_observation_diagnostic',
    'hospital_path_lab.r7_',
)
loaded = [name for name in sys.modules if name.startswith(banned)]
if loaded:
    raise SystemExit(','.join(loaded))
"""
    completed = run([executable, "-c", script], capture_output=True, text=True, check=False)

    assert completed.returncode == 0, completed.stderr or completed.stdout
