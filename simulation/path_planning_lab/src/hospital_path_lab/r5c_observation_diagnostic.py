"""Limited public Normal/Stress diagnostic for the completed R5-B scenes.

This module deliberately does not close R2-B or issue an R5-C qualification
receipt.  It exercises only public scenes whose Actors exist from t=0 and
stops the run after the first loss of a usable directional prediction.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import StrEnum
from math import isfinite

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import GridSnapshot, Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicGroundTruthFrame,
    DynamicMotionState,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalActorPredictor,
    DirectionalPredictionResult,
    DirectionalPredictionStatus,
)
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationProfile,
    DynamicObservationSnapshot,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    generate_dynamic_observation_slots,
)
from hospital_path_lab.dynamic_prediction import ActorPredictionSet, build_actor_prediction_set
from hospital_path_lab.dynamic_safety import (
    DYNAMIC_SAFE_OBSERVATION_FRAMES,
    DynamicSafetyGate,
    build_resume_authorization,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.dynamic_witness_contracts import WitnessWorldSnapshot
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PersistentSourceDerivedDwbController,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.persistent_controller_contracts import PersistentControllerStatus
from hospital_path_lab.persistent_controller_pipeline import (
    PersistentControllerPipeline,
    PersistentPipelineStep,
    integrate_persistent_chassis_pose,
)
from hospital_path_lab.r5b_restop_execution import (
    R5B_REFERENCE_MISSION_ID,
    R5B_RESTOP_FIRST_RELEASE_TICK,
    _hold_step,
    build_r5b_follow_reference,
    build_r5b_restop_evidence,
)
from hospital_path_lab.r5b_temporal_authorization import R5BTemporalAuthorizationIssuer
from hospital_path_lab.r5b_temporal_execution import (
    _grid_snapshot_for_observation,
    _pre_release_hold_step,
)
from hospital_path_lab.r5b_temporal_reference import (
    build_r5b_crossing_reference_bundles,
    rebind_r5b_crossing_reference_bundle,
)

R5C_OBSERVATION_DIAGNOSTIC_VERSION = "r5c-public-observation-diagnostic-v1"
_TOLERANCE = 1e-12
_END_OF_WORLD_STOP_BUFFER_TICKS = 20


class R5CDiagnosticOutcome(StrEnum):
    COMPLETED = "completed"
    CONSERVATIVE_HOLD = "conservative_hold"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class R5CObservationDiagnosticResult:
    case_id: str
    profile_name: str
    planned_release_tick: int
    actual_release_tick: int | None
    initial_stop_confirmed_tick: int | None
    first_motion_tick: int | None
    first_prediction_loss_tick: int | None
    protective_stop_started_tick: int | None
    stop_confirmed_tick: int | None
    completion_tick: int | None
    outcome: R5CDiagnosticOutcome
    final_motion_state: DynamicMotionState
    final_stop_epoch: int
    controller_call_count: int
    controller_session_count: int
    release_ticks: tuple[int, ...]
    prediction_loss_ticks: tuple[int, ...]
    authorization_loss_ticks: tuple[int, ...]
    confirmed_stop_ticks: tuple[int, ...]
    session_stop_epochs: tuple[int, ...]
    observation_status_counts: tuple[tuple[str, int], ...]
    no_frame_tick_count: int
    maximum_consecutive_ready_frames: int
    minimum_actor_clearance_m: float | None
    minimum_static_clearance_m: float
    gate_override_count: int
    final_pose: Pose2D
    hard_failures: tuple[str, ...]
    trace_content_hash: str

    @property
    def passed_safety_boundary(self) -> bool:
        return not self.hard_failures and self.outcome is not R5CDiagnosticOutcome.FAILED


class _ProfileObservationStream:
    def __init__(
        self,
        world: WitnessWorldSnapshot,
        *,
        profile: DynamicObservationProfile,
        tick_limit: int,
        stream_id: str,
        mission_revision: int,
    ) -> None:
        self._source = DynamicObservationSourceIdentity(
            stream_id=stream_id,
            episode_id=world.world_id,
            episode_seed=world.seed,
            map_id=world.map_id,
            map_revision=world.map_revision,
        )
        frames = tuple(
            DynamicGroundTruthFrame(
                episode_id=world.world_id,
                seed=world.seed,
                tick_id=tick,
                simulation_time_s=tick * DYNAMIC_CONTROL_PERIOD_S,
                robot_state=world.initial_state,
                actors=world.actor_states_at(tick * DYNAMIC_CONTROL_PERIOD_S),
                map_revision=world.map_revision,
                mission_revision=mission_revision,
            )
            for tick in range(tick_limit + 1)
        )
        self._slots = generate_dynamic_observation_slots(
            frames,
            source=self._source,
            profile=profile,
        )
        self._validator = DynamicObservationValidator(self._source, profile)
        self._directional = DirectionalActorPredictor()
        self._next_slot = 0

    def tick(
        self,
        tick_id: int,
    ) -> tuple[
        DynamicObservationSnapshot,
        ActorPredictionSet | None,
        DirectionalPredictionResult,
    ]:
        time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        while (
            self._next_slot < len(self._slots)
            and self._slots[self._next_slot].scheduled_delivery_at_s <= time_s + _TOLERANCE
        ):
            slot = self._slots[self._next_slot]
            if slot.frame is None:
                self._validator.record_no_frame(
                    sequence=slot.sequence,
                    delivery_time_s=slot.scheduled_delivery_at_s,
                )
            else:
                accepted = self._validator.accept(
                    slot.frame,
                    received_at_s=slot.scheduled_delivery_at_s,
                )
                if not accepted.accepted:
                    raise RuntimeError("generated R5-C public frame failed validation")
            self._next_slot += 1
        snapshot = self._validator.snapshot(control_time_s=time_s)
        circular = build_actor_prediction_set(snapshot) if snapshot.usable else None
        return snapshot, circular, self._directional.update(snapshot)


@dataclass(slots=True)
class _Runtime:
    pipeline: PersistentControllerPipeline
    issuer: R5BTemporalAuthorizationIssuer | None
    resume_authorization: object | None


def run_r5c_crossing_diagnostic(
    *,
    side_index: int,
    profile: DynamicObservationProfile,
    tick_limit: int = 780,
    recover_after_loss: bool = False,
) -> R5CObservationDiagnosticResult:
    """Run one public crossing side until completion or the first safe hold."""

    if side_index not in (0, 1):
        raise ValueError("R5-C crossing side_index must be 0 or 1")
    source_bundle = build_r5b_crossing_reference_bundles()[side_index]
    active_bundle = source_bundle
    world = source_bundle.source.world
    gate = DynamicSafetyGate(
        profile=source_bundle.build_context.vehicle_profile,
        initial_stop_epoch=0,
    )

    def hold(
        state: RobotState,
        tick: int,
        snapshot: DynamicObservationSnapshot,
        circular: ActorPredictionSet | None,
    ):
        return _pre_release_hold_step(
            source_bundle,
            gate=gate,
            robot_state=state,
            tick_id=tick,
            snapshot=snapshot,
            prediction_set=circular,
        )

    def launch(state: RobotState, tick: int) -> _Runtime:
        nonlocal active_bundle
        if (
            gate.stop_epoch == source_bundle.reference.stop_epoch
            and tick == source_bundle.reference.validity.valid_from_control_tick
        ):
            active_bundle = source_bundle
        else:
            active_bundle = rebind_r5b_crossing_reference_bundle(
                source_bundle,
                current_pose=state.pose,
                stop_epoch=gate.stop_epoch,
                valid_from_tick=tick,
            )
        controller = PersistentSourceDerivedDwbController(
            use_cpp_safety_core=True,
            use_cpp_full_core=True,
        )
        pipeline = PersistentControllerPipeline(
            controller=controller,
            build_context=active_bundle.build_context,
            full_reference=active_bundle.reference,
            validation=active_bundle.validation,
            initial_robot_state=state,
            gate=gate,
            authorization_revision=gate.stop_epoch,
            initial_tick=tick,
        )
        resume = build_resume_authorization(
            mission_id=active_bundle.reference.mission_id,
            stop_epoch=gate.stop_epoch,
            issued_or_revalidated_at_s=tick * DYNAMIC_CONTROL_PERIOD_S,
            authorization_revision=gate.stop_epoch,
        )
        return _Runtime(pipeline, R5BTemporalAuthorizationIssuer(), resume)

    def controller_step(
        runtime: _Runtime,
        tick: int,
        snapshot: DynamicObservationSnapshot,
        directional: DirectionalPredictionResult,
    ) -> PersistentPipelineStep:
        assert runtime.issuer is not None and directional.prediction_set is not None
        temporal = runtime.issuer.issue(
            reference=active_bundle.reference,
            temporal_evidence=active_bundle.temporal_evidence,
            temporal_geometry=active_bundle.temporal_geometry,
            robot_state=runtime.pipeline.robot_state,
            vehicle_profile=active_bundle.build_context.vehicle_profile,
            observation_snapshot=snapshot,
            prediction_result=directional,
            controller_tick=tick,
            simulation_time_s=tick * DYNAMIC_CONTROL_PERIOD_S,
            gate_motion_state=gate.motion_state,
            gate_stop_epoch=gate.stop_epoch,
            resume_authorization_revision=(
                gate.stop_epoch
                if runtime.resume_authorization is not None
                else None
            ),
            actual_stop_confirmed=runtime.resume_authorization is not None,
            local_safety_recheck_passed=True,
        )
        record = runtime.pipeline.step(
            observation_snapshot=snapshot,
            prediction_set=directional.prediction_set,
            resume_authorization=runtime.resume_authorization,
            temporal_execution_authorization=temporal,
            grid_snapshot=_grid_snapshot_for_observation(active_bundle, snapshot),
        )
        runtime.resume_authorization = None
        return record

    return _run_profile_diagnostic(
        case_id=(
            f"crossing-{source_bundle.source.side.value}-recovery"
            if recover_after_loss
            else f"crossing-{source_bundle.source.side.value}"
        ),
        world=world,
        profile=profile,
        tick_limit=tick_limit,
        planned_release_tick=source_bundle.reference.validity.valid_from_control_tick,
        stream_id="r5c-crossing-public",
        mission_revision=source_bundle.reference.mission_revision,
        gate=gate,
        checker=CollisionChecker(
            source_bundle.build_context.static_grid_snapshot.grid,
            source_bundle.build_context.vehicle_profile,
            forbidden_cells=source_bundle.build_context.static_grid_snapshot.forbidden_cells,
        ),
        hold=hold,
        launch=launch,
        controller_step=controller_step,
        recover_after_loss=recover_after_loss,
        finish_with_confirmed_stop=recover_after_loss,
    )


def run_r5c_crossing_recovery_diagnostic(
    *,
    side_index: int,
    profile: DynamicObservationProfile,
    tick_limit: int = 780,
) -> R5CObservationDiagnosticResult:
    """Resume crossing only through new stop-bound reference sessions."""

    return run_r5c_crossing_diagnostic(
        side_index=side_index,
        profile=profile,
        tick_limit=tick_limit,
        recover_after_loss=True,
    )


def run_r5c_restop_diagnostic(
    *,
    profile: DynamicObservationProfile,
    tick_limit: int = 700,
) -> R5CObservationDiagnosticResult:
    """Run the public two-risk scene until completion or the first safe hold."""

    evidence = build_r5b_restop_evidence()
    world = evidence.controller_world
    gate = DynamicSafetyGate(
        profile=world.kinematic_contract.vehicle_profile,
        initial_stop_epoch=0,
    )

    def hold(
        state: RobotState,
        tick: int,
        snapshot: DynamicObservationSnapshot,
        circular: ActorPredictionSet | None,
    ):
        return _hold_step(
            evidence,
            gate=gate,
            robot_state=state,
            tick=tick,
            snapshot=snapshot,
            prediction_set=circular,
        )

    def launch(state: RobotState, tick: int) -> _Runtime:
        bundle = build_r5b_follow_reference(
            evidence,
            current_pose=state.pose,
            stop_epoch=gate.stop_epoch,
            valid_from_tick=tick,
        )
        controller = PersistentSourceDerivedDwbController(
            use_cpp_safety_core=True,
            use_cpp_full_core=True,
        )
        pipeline = PersistentControllerPipeline(
            controller=controller,
            build_context=bundle.build_context,
            full_reference=bundle.reference,
            validation=bundle.validation,
            initial_robot_state=state,
            gate=gate,
            authorization_revision=gate.stop_epoch,
            initial_tick=tick,
        )
        resume = build_resume_authorization(
            mission_id=R5B_REFERENCE_MISSION_ID,
            stop_epoch=gate.stop_epoch,
            issued_or_revalidated_at_s=tick * DYNAMIC_CONTROL_PERIOD_S,
            authorization_revision=gate.stop_epoch,
        )
        return _Runtime(pipeline, None, resume)

    def controller_step(
        runtime: _Runtime,
        _tick: int,
        snapshot: DynamicObservationSnapshot,
        directional: DirectionalPredictionResult,
    ) -> PersistentPipelineStep:
        assert directional.prediction_set is not None
        frozen_grid = runtime.pipeline.build_context.static_grid_snapshot
        observation_revision = (
            frozen_grid.metadata.observation_revision
            if snapshot.frame is None
            else snapshot.frame.observation_revision
        )
        current_grid = GridSnapshot(
            metadata=replace(
                frozen_grid.metadata,
                observation_revision=observation_revision,
            ),
            grid=frozen_grid.grid,
            forbidden_cells=frozen_grid.forbidden_cells,
        )
        record = runtime.pipeline.step(
            observation_snapshot=snapshot,
            prediction_set=directional.prediction_set,
            resume_authorization=runtime.resume_authorization,
            grid_snapshot=current_grid,
        )
        runtime.resume_authorization = None
        return record

    return _run_profile_diagnostic(
        case_id="restop-two-risk",
        world=world,
        profile=profile,
        tick_limit=tick_limit,
        planned_release_tick=R5B_RESTOP_FIRST_RELEASE_TICK,
        stream_id="r5c-restop-public",
        mission_revision=0,
        gate=gate,
        checker=CollisionChecker(
            world.grid.to_grid_map(),
            world.kinematic_contract.vehicle_profile,
            forbidden_cells=world.grid.forbidden_cells,
        ),
        hold=hold,
        launch=launch,
        controller_step=controller_step,
    )


def run_r5c_restop_recovery_diagnostic(
    *,
    profile: DynamicObservationProfile,
    tick_limit: int = 700,
) -> R5CObservationDiagnosticResult:
    """Run the public two-risk scene with stop-confirmed session recovery."""

    evidence = build_r5b_restop_evidence()
    world = evidence.controller_world
    gate = DynamicSafetyGate(
        profile=world.kinematic_contract.vehicle_profile,
        initial_stop_epoch=0,
    )

    def hold(
        state: RobotState,
        tick: int,
        snapshot: DynamicObservationSnapshot,
        circular: ActorPredictionSet | None,
    ):
        return _hold_step(
            evidence,
            gate=gate,
            robot_state=state,
            tick=tick,
            snapshot=snapshot,
            prediction_set=circular,
        )

    def launch(state: RobotState, tick: int) -> _Runtime:
        bundle = build_r5b_follow_reference(
            evidence,
            current_pose=state.pose,
            stop_epoch=gate.stop_epoch,
            valid_from_tick=tick,
        )
        controller = PersistentSourceDerivedDwbController(
            use_cpp_safety_core=True,
            use_cpp_full_core=True,
        )
        pipeline = PersistentControllerPipeline(
            controller=controller,
            build_context=bundle.build_context,
            full_reference=bundle.reference,
            validation=bundle.validation,
            initial_robot_state=state,
            gate=gate,
            authorization_revision=gate.stop_epoch,
            initial_tick=tick,
        )
        resume = build_resume_authorization(
            mission_id=R5B_REFERENCE_MISSION_ID,
            stop_epoch=gate.stop_epoch,
            issued_or_revalidated_at_s=tick * DYNAMIC_CONTROL_PERIOD_S,
            authorization_revision=gate.stop_epoch,
        )
        return _Runtime(pipeline, None, resume)

    def controller_step(
        runtime: _Runtime,
        _tick: int,
        snapshot: DynamicObservationSnapshot,
        directional: DirectionalPredictionResult,
    ) -> PersistentPipelineStep:
        assert directional.prediction_set is not None
        frozen_grid = runtime.pipeline.build_context.static_grid_snapshot
        observation_revision = (
            frozen_grid.metadata.observation_revision
            if snapshot.frame is None
            else snapshot.frame.observation_revision
        )
        current_grid = GridSnapshot(
            metadata=replace(
                frozen_grid.metadata,
                observation_revision=observation_revision,
            ),
            grid=frozen_grid.grid,
            forbidden_cells=frozen_grid.forbidden_cells,
        )
        record = runtime.pipeline.step(
            observation_snapshot=snapshot,
            prediction_set=directional.prediction_set,
            resume_authorization=runtime.resume_authorization,
            grid_snapshot=current_grid,
        )
        runtime.resume_authorization = None
        return record

    return _run_profile_diagnostic(
        case_id="restop-two-risk-recovery",
        world=world,
        profile=profile,
        tick_limit=tick_limit,
        planned_release_tick=R5B_RESTOP_FIRST_RELEASE_TICK,
        stream_id="r5c-restop-recovery-public",
        mission_revision=0,
        gate=gate,
        checker=CollisionChecker(
            world.grid.to_grid_map(),
            world.kinematic_contract.vehicle_profile,
            forbidden_cells=world.grid.forbidden_cells,
        ),
        hold=hold,
        launch=launch,
        controller_step=controller_step,
        recover_after_loss=True,
    )


def _run_profile_diagnostic(
    *,
    case_id: str,
    world: WitnessWorldSnapshot,
    profile: DynamicObservationProfile,
    tick_limit: int,
    planned_release_tick: int,
    stream_id: str,
    mission_revision: int,
    gate: DynamicSafetyGate,
    checker: CollisionChecker,
    hold: Callable,
    launch: Callable[[RobotState, int], _Runtime],
    controller_step: Callable,
    recover_after_loss: bool = False,
    finish_with_confirmed_stop: bool = False,
) -> R5CObservationDiagnosticResult:
    if profile not in (NORMAL_OBSERVATION_PROFILE, STRESS_OBSERVATION_PROFILE):
        raise ValueError("R5-C diagnostic accepts only frozen Normal or Stress")
    if tick_limit <= planned_release_tick:
        raise ValueError("R5-C tick limit must extend beyond planned release")
    stream = _ProfileObservationStream(
        world,
        profile=profile,
        tick_limit=tick_limit,
        stream_id=stream_id,
        mission_revision=mission_revision,
    )
    state = RobotState(world.initial_state.pose, Twist2D())
    runtime: _Runtime | None = None
    status_counts: Counter[str] = Counter()
    no_frame_tick_count = 0
    actual_release_tick: int | None = None
    initial_stop_confirmed_tick: int | None = None
    first_motion_tick: int | None = None
    first_prediction_loss_tick: int | None = None
    protective_stop_started_tick: int | None = None
    stop_confirmed_tick: int | None = None
    completion_tick: int | None = None
    minimum_actor: float | None = None
    minimum_static = float("inf")
    controller_calls = 0
    hard_failures: list[str] = []
    trace: list[object] = []
    stopping_after_loss = False
    confirmed_safe_frame_count = 0
    last_confirmed_safe_sequence: int | None = None
    release_ticks: list[int] = []
    prediction_loss_ticks: list[int] = []
    authorization_loss_ticks: list[int] = []
    confirmed_stop_ticks: list[int] = []
    session_stop_epochs: list[int] = []
    maximum_consecutive_ready_frames = 0
    consecutive_ready_frames = 0
    last_ready_sequence: int | None = None

    for tick in range(tick_limit):
        snapshot, circular, directional = stream.tick(tick)
        status_counts[directional.status.value] += 1
        no_frame_tick_count += int(snapshot.last_event_was_no_frame)
        current_sequence = None if snapshot.frame is None else snapshot.frame.sequence
        if directional.status is DirectionalPredictionStatus.READY:
            if current_sequence is not None and current_sequence != last_ready_sequence:
                consecutive_ready_frames += 1
                last_ready_sequence = current_sequence
        else:
            consecutive_ready_frames = 0
            last_ready_sequence = None
        maximum_consecutive_ready_frames = max(
            maximum_consecutive_ready_frames,
            consecutive_ready_frames,
        )

        if finish_with_confirmed_stop and tick >= (
            tick_limit - _END_OF_WORLD_STOP_BUFFER_TICKS
        ):
            if protective_stop_started_tick is None:
                protective_stop_started_tick = tick
            decision = hold(state, tick, snapshot, None)
            state = _advance_state(state, decision.command)
            trace.append(
                (
                    tick,
                    "end_of_world_protective_stop",
                    gate.motion_state,
                    gate.stop_epoch,
                    state,
                )
            )
            minimum_static, minimum_actor = _update_clearances(
                world,
                checker,
                state,
                tick,
                minimum_static,
                minimum_actor,
            )
            if gate.motion_state is DynamicMotionState.HOLDING:
                if not confirmed_stop_ticks or confirmed_stop_ticks[-1] != tick:
                    confirmed_stop_ticks.append(tick)
                if stop_confirmed_tick is None:
                    stop_confirmed_tick = tick
                runtime = None
                break
            continue

        if runtime is None:
            current_frame_adds_ready_evidence = int(
                directional.status is DirectionalPredictionStatus.READY
                and current_sequence is not None
                and current_sequence != last_confirmed_safe_sequence
            )
            projected_release_frames = (
                consecutive_ready_frames
                if recover_after_loss
                else confirmed_safe_frame_count + current_frame_adds_ready_evidence
            )
            can_release = all(
                (
                    tick >= planned_release_tick,
                    directional.status is DirectionalPredictionStatus.READY,
                    gate.motion_state is DynamicMotionState.HOLDING,
                    (
                        gate.stop_epoch >= 1
                        if recover_after_loss
                        else gate.stop_epoch == 1
                    ),
                    projected_release_frames >= DYNAMIC_SAFE_OBSERVATION_FRAMES,
                )
            )
            if can_release:
                runtime = launch(state, tick)
                release_ticks.append(tick)
                session_stop_epochs.append(gate.stop_epoch)
                if actual_release_tick is None:
                    actual_release_tick = tick
            else:
                decision = hold(state, tick, snapshot, circular)
                if recover_after_loss and (
                    directional.status is DirectionalPredictionStatus.READY
                    and decision.consecutive_safe_frames > 0
                ):
                    confirmed_safe_frame_count = min(
                        consecutive_ready_frames,
                        decision.consecutive_safe_frames,
                    )
                    last_confirmed_safe_sequence = current_sequence
                elif recover_after_loss:
                    confirmed_safe_frame_count = 0
                    last_confirmed_safe_sequence = None
                else:
                    confirmed_safe_frame_count = decision.consecutive_safe_frames
                    last_confirmed_safe_sequence = (
                        current_sequence if confirmed_safe_frame_count > 0 else None
                    )
                state = _advance_state(state, decision.command)
                if (
                    initial_stop_confirmed_tick is None
                    and gate.motion_state is DynamicMotionState.HOLDING
                ):
                    initial_stop_confirmed_tick = tick
                trace.append((tick, directional.status, gate.motion_state, gate.stop_epoch, state))
                minimum_static, minimum_actor = _update_clearances(
                    world,
                    checker,
                    state,
                    tick,
                    minimum_static,
                    minimum_actor,
                )
                continue

        if stopping_after_loss or directional.prediction_set is None:
            if not stopping_after_loss:
                prediction_loss_ticks.append(tick)
                if first_prediction_loss_tick is None:
                    first_prediction_loss_tick = tick
                if protective_stop_started_tick is None:
                    protective_stop_started_tick = tick
                stopping_after_loss = True
            # Directional prediction loss is the controller-facing failure.  Do
            # not substitute the circular fallback and accidentally keep moving.
            decision = hold(state, tick, snapshot, None)
            state = _advance_state(state, decision.command)
            trace.append((tick, directional.status, gate.motion_state, gate.stop_epoch, state))
            minimum_static, minimum_actor = _update_clearances(
                world,
                checker,
                state,
                tick,
                minimum_static,
                minimum_actor,
            )
            if gate.motion_state is DynamicMotionState.HOLDING:
                confirmed_stop_ticks.append(tick)
                if stop_confirmed_tick is None:
                    stop_confirmed_tick = tick
                if recover_after_loss:
                    runtime = None
                    stopping_after_loss = False
                    confirmed_safe_frame_count = 0
                    last_confirmed_safe_sequence = current_sequence
                    consecutive_ready_frames = 0
                    last_ready_sequence = current_sequence
                    continue
                break
            continue

        try:
            record = controller_step(runtime, tick, snapshot, directional)
        except (RuntimeError, TypeError, ValueError) as error:
            if recover_after_loss and str(error) == (
                "R5-B target is no longer conservatively behind the robot"
            ):
                authorization_loss_ticks.append(tick)
                if protective_stop_started_tick is None:
                    protective_stop_started_tick = tick
                stopping_after_loss = True
                decision = hold(state, tick, snapshot, None)
                state = _advance_state(state, decision.command)
                trace.append(
                    (
                        tick,
                        "temporal_authorization_revoked",
                        gate.motion_state,
                        gate.stop_epoch,
                        state,
                    )
                )
                minimum_static, minimum_actor = _update_clearances(
                    world,
                    checker,
                    state,
                    tick,
                    minimum_static,
                    minimum_actor,
                )
                if gate.motion_state is DynamicMotionState.HOLDING:
                    confirmed_stop_ticks.append(tick)
                    if stop_confirmed_tick is None:
                        stop_confirmed_tick = tick
                    runtime = None
                    stopping_after_loss = False
                    confirmed_safe_frame_count = 0
                    last_confirmed_safe_sequence = current_sequence
                    consecutive_ready_frames = 0
                    last_ready_sequence = current_sequence
                continue
            hard_failures.append(f"controller_exception:{tick}:{error}")
            break
        controller_calls += 1
        state = record.robot_state_after
        if first_motion_tick is None and record.safety_decision.command != Twist2D():
            first_motion_tick = tick
        result = record.controller_result
        if result is not None and result.status in {
            PersistentControllerStatus.INVALID_REFERENCE_INPUT,
            PersistentControllerStatus.STALE_REFERENCE_INPUT,
            PersistentControllerStatus.LATE_RESULT,
            PersistentControllerStatus.SECTION_EXECUTION_FAILED,
        }:
            hard_failures.append(
                f"controller:{tick}:{result.status.value}:{result.failure_reason}"
            )
            break
        if gate.motion_state is DynamicMotionState.COMPLETED:
            completion_tick = tick
        elif gate.motion_state is not DynamicMotionState.MOVING:
            protective_stop_started_tick = protective_stop_started_tick or tick
            if gate.motion_state is DynamicMotionState.HOLDING:
                if not confirmed_stop_ticks or confirmed_stop_ticks[-1] != tick:
                    confirmed_stop_ticks.append(tick)
                if stop_confirmed_tick is None:
                    stop_confirmed_tick = tick
        trace.append(
            (
                tick,
                directional.status,
                gate.motion_state,
                gate.stop_epoch,
                state,
                None if result is None else result.semantic_content_hash,
                record.safety_decision,
            )
        )
        minimum_static, minimum_actor = _update_clearances(
            world,
            checker,
            state,
            tick,
            minimum_static,
            minimum_actor,
        )
        if completion_tick is not None:
            break
        if gate.motion_state is DynamicMotionState.HOLDING:
            break

    minimum_required = world.kinematic_contract.vehicle_profile.minimum_clearance_m
    if minimum_static < minimum_required - _TOLERANCE:
        hard_failures.append("actual_static_clearance_below_minimum")
    if minimum_actor is not None and minimum_actor < minimum_required - _TOLERANCE:
        hard_failures.append("actual_actor_clearance_below_minimum")
    if runtime is not None and completion_tick is None and stop_confirmed_tick is None:
        hard_failures.append("protective_stop_not_confirmed")
    if hard_failures:
        outcome = R5CDiagnosticOutcome.FAILED
    elif completion_tick is not None:
        outcome = R5CDiagnosticOutcome.COMPLETED
    elif gate.motion_state is DynamicMotionState.HOLDING:
        outcome = R5CDiagnosticOutcome.CONSERVATIVE_HOLD
    else:
        outcome = R5CDiagnosticOutcome.FAILED
        hard_failures.append("final_state_not_conservative")
    if not isfinite(minimum_static):
        hard_failures.append("minimum_static_clearance_not_finite")
        outcome = R5CDiagnosticOutcome.FAILED

    return R5CObservationDiagnosticResult(
        case_id=case_id,
        profile_name=profile.name.value,
        planned_release_tick=planned_release_tick,
        actual_release_tick=actual_release_tick,
        initial_stop_confirmed_tick=initial_stop_confirmed_tick,
        first_motion_tick=first_motion_tick,
        first_prediction_loss_tick=first_prediction_loss_tick,
        protective_stop_started_tick=protective_stop_started_tick,
        stop_confirmed_tick=stop_confirmed_tick,
        completion_tick=completion_tick,
        outcome=outcome,
        final_motion_state=gate.motion_state,
        final_stop_epoch=gate.stop_epoch,
        controller_call_count=controller_calls,
        controller_session_count=len(release_ticks),
        release_ticks=tuple(release_ticks),
        prediction_loss_ticks=tuple(prediction_loss_ticks),
        authorization_loss_ticks=tuple(authorization_loss_ticks),
        confirmed_stop_ticks=tuple(confirmed_stop_ticks),
        session_stop_epochs=tuple(session_stop_epochs),
        observation_status_counts=tuple(sorted(status_counts.items())),
        no_frame_tick_count=no_frame_tick_count,
        maximum_consecutive_ready_frames=maximum_consecutive_ready_frames,
        minimum_actor_clearance_m=minimum_actor,
        minimum_static_clearance_m=minimum_static,
        gate_override_count=gate.counters.gate_overrides,
        final_pose=state.pose,
        hard_failures=tuple(dict.fromkeys(hard_failures)),
        trace_content_hash=canonical_content_hash(tuple(trace)),
    )


def _advance_state(state: RobotState, next_command: Twist2D) -> RobotState:
    pose = integrate_persistent_chassis_pose(
        state.pose,
        state.twist,
        DYNAMIC_CONTROL_PERIOD_S,
    )
    return RobotState(pose, next_command)


def _update_clearances(
    world: WitnessWorldSnapshot,
    checker: CollisionChecker,
    state: RobotState,
    tick: int,
    minimum_static: float,
    minimum_actor: float | None,
) -> tuple[float, float | None]:
    minimum_static = min(minimum_static, checker.clearance(state.pose))
    time_s = (tick + 1) * DYNAMIC_CONTROL_PERIOD_S
    for actor in world.actor_states_at(time_s):
        clearance = oriented_footprint_circle_surface_distance(
            state.pose,
            circle_center=(actor.position.x, actor.position.y),
            circle_radius_m=actor.radius_m,
            profile=world.kinematic_contract.vehicle_profile,
        )
        minimum_actor = clearance if minimum_actor is None else min(minimum_actor, clearance)
    return minimum_static, minimum_actor


__all__ = [
    "R5C_OBSERVATION_DIAGNOSTIC_VERSION",
    "R5CDiagnosticOutcome",
    "R5CObservationDiagnosticResult",
    "run_r5c_crossing_diagnostic",
    "run_r5c_crossing_recovery_diagnostic",
    "run_r5c_restop_diagnostic",
    "run_r5c_restop_recovery_diagnostic",
]
