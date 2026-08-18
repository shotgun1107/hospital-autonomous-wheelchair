"""Limited public Normal/Stress diagnostic for the completed R5-B scenes.

This module deliberately does not close R2-B or issue an R5-C qualification
receipt.  It exercises only public scenes whose Actors exist from t=0. A
single missing frame may reuse an already locked direction until TTL, while a
stale or otherwise unusable input still starts a protective stop.
"""

# The trace emitter is defined and consumed entirely within one loop iteration;
# it never escapes to a later iteration.
# ruff: noqa: B023

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
    ActorState,
    DynamicGroundTruthFrame,
    DynamicMotionState,
    Point2D,
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
from hospital_path_lab.local_reference_window import project_reference_cursor
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
    build_world_follow_reference,
)
from hospital_path_lab.r5b_temporal_authorization import (
    R5BTemporalAuthorizationIssuer,
    R5BTemporalAuthorizationPhase,
)
from hospital_path_lab.r5b_temporal_execution import (
    _grid_snapshot_for_observation,
    _pre_release_hold_step,
)
from hospital_path_lab.r5b_temporal_reference import (
    build_r5b_crossing_reference_bundles,
    rebind_r5b_crossing_reference_bundle,
)
from hospital_path_lab.r7_failure_trace import R7FailureTraceCollector

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
    post_pass_proof_tick: int | None
    follow_original_release_tick: int | None
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
        extend_terminal_actor_trajectory: bool = False,
        observation_seed: int | None = None,
    ) -> None:
        if observation_seed is None:
            observation_seed = world.seed
        if (
            isinstance(observation_seed, bool)
            or not isinstance(observation_seed, int)
            or observation_seed < 0
        ):
            raise ValueError("observation_seed must be a non-negative exact integer")
        self._source = DynamicObservationSourceIdentity(
            stream_id=stream_id,
            episode_id=world.world_id,
            episode_seed=observation_seed,
            map_id=world.map_id,
            map_revision=world.map_revision,
        )
        frames = tuple(
            DynamicGroundTruthFrame(
                episode_id=world.world_id,
                seed=observation_seed,
                tick_id=tick,
                simulation_time_s=tick * DYNAMIC_CONTROL_PERIOD_S,
                robot_state=world.initial_state,
                actors=_actor_states_at_observation_time(
                    world,
                    tick * DYNAMIC_CONTROL_PERIOD_S,
                    extend_terminal_actor_trajectory=extend_terminal_actor_trajectory,
                ),
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
    complete_after_post_pass: bool = False,
    observation_seed: int | None = None,
    failure_trace: R7FailureTraceCollector | None = None,
    observation_horizon_ticks: int | None = None,
) -> R5CObservationDiagnosticResult:
    """Run one public crossing side until completion or the first safe hold."""

    if side_index not in (0, 1):
        raise ValueError("R5-C crossing side_index must be 0 or 1")
    source_bundle = build_r5b_crossing_reference_bundles()[side_index]
    active_bundle = source_bundle
    world = source_bundle.source.world
    post_pass_authorization_hash: str | None = None
    post_pass_proof_tick: int | None = None
    follow_original_release_tick: int | None = None
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

    def protective_stop(
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
            controller_requested_stop=True,
        )

    def launch(state: RobotState, tick: int) -> _Runtime:
        nonlocal active_bundle, follow_original_release_tick
        if post_pass_authorization_hash is not None:
            follow_bundle = build_world_follow_reference(
                world,
                mission_id=R5B_REFERENCE_MISSION_ID,
                current_pose=state.pose,
                stop_epoch=gate.stop_epoch,
                valid_from_tick=tick,
                identity={
                    "version": R5C_OBSERVATION_DIAGNOSTIC_VERSION,
                    "source_bundle_hash": source_bundle.bundle_content_hash,
                    "post_pass_authorization_hash": post_pass_authorization_hash,
                    "stop_epoch": gate.stop_epoch,
                    "valid_from_tick": tick,
                    "start_pose": state.pose,
                    "goal_pose": source_bundle.reference.knots[-1].pose,
                },
                generation_reason_codes=("r5c_post_pass_follow_original",),
                goal_pose=source_bundle.reference.knots[-1].pose,
            )
            controller = PersistentSourceDerivedDwbController(
                use_cpp_safety_core=True,
                use_cpp_full_core=True,
            )
            pipeline = PersistentControllerPipeline(
                controller=controller,
                build_context=follow_bundle.build_context,
                full_reference=follow_bundle.reference,
                validation=follow_bundle.validation,
                initial_robot_state=state,
                gate=gate,
                authorization_revision=gate.stop_epoch,
                initial_tick=tick,
            )
            resume = build_resume_authorization(
                mission_id=follow_bundle.reference.mission_id,
                stop_epoch=gate.stop_epoch,
                issued_or_revalidated_at_s=tick * DYNAMIC_CONTROL_PERIOD_S,
                authorization_revision=gate.stop_epoch,
            )
            if follow_original_release_tick is None:
                follow_original_release_tick = tick
            return _Runtime(pipeline, None, resume)
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
        trace_detail: dict[str, object],
    ) -> PersistentPipelineStep:
        nonlocal post_pass_authorization_hash, post_pass_proof_tick
        assert directional.prediction_set is not None
        _capture_controller_before(runtime, trace_detail)
        if runtime.issuer is None:
            trace_detail["authorization_issue_attempted"] = False
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
            _capture_pipeline_trace(record, runtime, trace_detail)
            return record
        trace_detail["authorization_issue_attempted"] = True
        trace_detail["authorization_phase_requested"] = (
            R5BTemporalAuthorizationPhase.INITIAL_RELEASE.value
            if runtime.issuer.last_authorization is None
            else R5BTemporalAuthorizationPhase.CONTINUATION.value
        )
        trace_detail["prior_authorization_hash"] = (
            None
            if runtime.issuer.last_authorization is None
            else runtime.issuer.last_authorization.authorization_content_hash
        )
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
        trace_detail["authorization_issue_outcome"] = "issued"
        trace_detail["temporal_authorization_phase"] = temporal.phase.value
        if temporal.phase is R5BTemporalAuthorizationPhase.POST_PASS_COMPLETION:
            post_pass_authorization_hash = temporal.authorization_content_hash
            if post_pass_proof_tick is None:
                post_pass_proof_tick = tick
        record = runtime.pipeline.step(
            observation_snapshot=snapshot,
            prediction_set=directional.prediction_set,
            resume_authorization=runtime.resume_authorization,
            temporal_execution_authorization=temporal,
            grid_snapshot=_grid_snapshot_for_observation(active_bundle, snapshot),
        )
        runtime.resume_authorization = None
        _capture_pipeline_trace(record, runtime, trace_detail)
        return record

    result = _run_profile_diagnostic(
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
        protective_stop=protective_stop,
        launch=launch,
        controller_step=controller_step,
        recover_after_loss=recover_after_loss,
        finish_with_confirmed_stop=recover_after_loss,
        extend_terminal_actor_trajectory=complete_after_post_pass,
        empty_release_authorized=(
            (lambda: post_pass_authorization_hash is not None)
            if complete_after_post_pass
            else None
        ),
        planned_transition_stop_requested=(
            (lambda: post_pass_authorization_hash is not None)
            if complete_after_post_pass
            else None
        ),
        observation_seed=observation_seed,
        failure_trace=failure_trace,
        observation_horizon_ticks=observation_horizon_ticks,
    )
    return replace(
        result,
        post_pass_proof_tick=post_pass_proof_tick,
        follow_original_release_tick=follow_original_release_tick,
    )


def run_r5c_crossing_recovery_diagnostic(
    *,
    side_index: int,
    profile: DynamicObservationProfile,
    tick_limit: int = 780,
    observation_seed: int | None = None,
    failure_trace: R7FailureTraceCollector | None = None,
    observation_horizon_ticks: int | None = None,
) -> R5CObservationDiagnosticResult:
    """Resume crossing only through new stop-bound reference sessions."""

    return run_r5c_crossing_diagnostic(
        side_index=side_index,
        profile=profile,
        tick_limit=tick_limit,
        recover_after_loss=True,
        observation_seed=observation_seed,
        failure_trace=failure_trace,
        observation_horizon_ticks=observation_horizon_ticks,
    )


def run_r5c_crossing_completion_diagnostic(
    *,
    side_index: int,
    profile: DynamicObservationProfile,
    tick_limit: int = 1600,
    observation_seed: int | None = None,
    failure_trace: R7FailureTraceCollector | None = None,
    observation_horizon_ticks: int | None = None,
) -> R5CObservationDiagnosticResult:
    """Run the extended public scene through post-pass return and goal completion."""

    return run_r5c_crossing_diagnostic(
        side_index=side_index,
        profile=profile,
        tick_limit=tick_limit,
        recover_after_loss=True,
        complete_after_post_pass=True,
        observation_seed=observation_seed,
        failure_trace=failure_trace,
        observation_horizon_ticks=observation_horizon_ticks,
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
        trace_detail: dict[str, object],
    ) -> PersistentPipelineStep:
        trace_detail["authorization_issue_attempted"] = False
        assert directional.prediction_set is not None
        _capture_controller_before(runtime, trace_detail)
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
        _capture_pipeline_trace(record, runtime, trace_detail)
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
        trace_detail: dict[str, object],
    ) -> PersistentPipelineStep:
        trace_detail["authorization_issue_attempted"] = False
        assert directional.prediction_set is not None
        _capture_controller_before(runtime, trace_detail)
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
        _capture_pipeline_trace(record, runtime, trace_detail)
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
    protective_stop: Callable | None = None,
    recover_after_loss: bool = False,
    finish_with_confirmed_stop: bool = False,
    extend_terminal_actor_trajectory: bool = False,
    empty_release_authorized: Callable[[], bool] | None = None,
    planned_transition_stop_requested: Callable[[], bool] | None = None,
    observation_seed: int | None = None,
    failure_trace: R7FailureTraceCollector | None = None,
    observation_horizon_ticks: int | None = None,
) -> R5CObservationDiagnosticResult:
    if profile not in (NORMAL_OBSERVATION_PROFILE, STRESS_OBSERVATION_PROFILE):
        raise ValueError("R5-C diagnostic accepts only frozen Normal or Stress")
    if tick_limit <= planned_release_tick:
        raise ValueError("R5-C tick limit must extend beyond planned release")
    if observation_horizon_ticks is None:
        observation_horizon_ticks = tick_limit
    if (
        isinstance(observation_horizon_ticks, bool)
        or not isinstance(observation_horizon_ticks, int)
        or observation_horizon_ticks < tick_limit
    ):
        raise ValueError("observation_horizon_ticks must be an integer at least tick_limit")
    prefix_only = observation_horizon_ticks > tick_limit
    stream = _ProfileObservationStream(
        world,
        profile=profile,
        tick_limit=observation_horizon_ticks,
        stream_id=stream_id,
        mission_revision=mission_revision,
        extend_terminal_actor_trajectory=extend_terminal_actor_trajectory,
        observation_seed=observation_seed,
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
    stopping_after_loss_reason = "prediction_loss"
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
    gate_safe_frame_count = 0

    for tick in range(tick_limit):
        state_before_tick = state
        runtime_before_tick = runtime
        gate_state_before_tick = gate.motion_state
        stop_epoch_before_tick = gate.stop_epoch
        gate_safe_frames_before_tick = gate_safe_frame_count
        confirmed_safe_before_tick = confirmed_safe_frame_count
        gate_overrides_before_tick = gate.counters.gate_overrides
        trace_detail: dict[str, object] = {
            "recovery_reason": "none",
            "release_requested": False,
            "release_permitted": False,
            "release_denial_reasons": (),
            "authorization_issue_attempted": False,
            "authorization_phase_requested": None,
            "authorization_issue_outcome": "not_attempted",
            "authorization_issue_error": None,
            "temporal_authorization_phase": None,
            "prior_authorization_hash": None,
            "controller_called": False,
            "controller_exception_type": None,
            "controller_exception_message": None,
        }
        snapshot, circular, directional = stream.tick(tick)
        status_counts[directional.status.value] += 1
        no_frame_tick_count += int(snapshot.last_event_was_no_frame)
        current_sequence = None if snapshot.frame is None else snapshot.frame.sequence
        release_input_usable = (
            directional.status is DirectionalPredictionStatus.READY
            or (
                extend_terminal_actor_trajectory
                and directional.status is DirectionalPredictionStatus.EMPTY_FRAME
                and empty_release_authorized is not None
                and empty_release_authorized()
            )
        )
        if release_input_usable:
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

        def emit_failure_trace(
            decision=None,
            record: PersistentPipelineStep | None = None,
        ) -> None:
            if failure_trace is None:
                return
            _append_failure_trace_tick(
                failure_trace,
                tick=tick,
                snapshot=snapshot,
                directional=directional,
                release_input_usable=release_input_usable,
                consecutive_ready_frames=consecutive_ready_frames,
                last_ready_sequence=last_ready_sequence,
                gate_state_before=gate_state_before_tick,
                stop_epoch_before=stop_epoch_before_tick,
                gate_safe_frames_before=gate_safe_frames_before_tick,
                confirmed_safe_frames_before=confirmed_safe_before_tick,
                confirmed_safe_frames_after=confirmed_safe_frame_count,
                last_confirmed_safe_sequence=last_confirmed_safe_sequence,
                gate_overrides_before=gate_overrides_before_tick,
                state_before=state_before_tick,
                state_after=state,
                runtime_before=runtime_before_tick,
                runtime_after=runtime,
                decision=decision,
                record=record,
                detail=trace_detail,
            )

        if finish_with_confirmed_stop and tick >= (
            observation_horizon_ticks - _END_OF_WORLD_STOP_BUFFER_TICKS
        ):
            if protective_stop_started_tick is None:
                protective_stop_started_tick = tick
            decision = hold(state, tick, snapshot, None)
            gate_safe_frame_count = decision.consecutive_safe_frames
            trace_detail["recovery_reason"] = "end_of_world_stop"
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
                extend_terminal_actor_trajectory=extend_terminal_actor_trajectory,
            )
            if gate.motion_state is DynamicMotionState.COMPLETED:
                completion_tick = tick
                emit_failure_trace(decision)
                break
            if gate.motion_state is DynamicMotionState.HOLDING:
                if not confirmed_stop_ticks or confirmed_stop_ticks[-1] != tick:
                    confirmed_stop_ticks.append(tick)
                if stop_confirmed_tick is None:
                    stop_confirmed_tick = tick
                runtime = None
                emit_failure_trace(decision)
                break
            emit_failure_trace(decision)
            continue

        if runtime is None:
            current_frame_adds_ready_evidence = int(
                release_input_usable
                and current_sequence is not None
                and current_sequence != last_confirmed_safe_sequence
            )
            projected_release_frames = (
                confirmed_safe_frame_count + current_frame_adds_ready_evidence
            )
            gate_confirmed_release_ready = (
                confirmed_safe_frame_count >= DYNAMIC_SAFE_OBSERVATION_FRAMES
                and bool(current_frame_adds_ready_evidence)
            )
            can_release = all(
                (
                    tick >= planned_release_tick,
                    release_input_usable,
                    gate.motion_state is DynamicMotionState.HOLDING,
                    (
                        gate.stop_epoch >= 1
                        if recover_after_loss
                        else gate.stop_epoch == 1
                    ),
                    (
                        gate_confirmed_release_ready
                        if recover_after_loss
                        else projected_release_frames >= DYNAMIC_SAFE_OBSERVATION_FRAMES
                    ),
                )
            )
            trace_detail["release_requested"] = tick >= planned_release_tick
            trace_detail["release_permitted"] = can_release
            denial_reasons: list[str] = []
            if tick < planned_release_tick:
                denial_reasons.append("before_planned_release")
            if not release_input_usable:
                denial_reasons.append("observation_not_usable")
            if gate.motion_state is not DynamicMotionState.HOLDING:
                denial_reasons.append("actual_stop_not_confirmed")
            if gate.stop_epoch < 1:
                denial_reasons.append("stop_epoch_not_confirmed")
            if recover_after_loss and confirmed_safe_frame_count < (
                DYNAMIC_SAFE_OBSERVATION_FRAMES
            ):
                denial_reasons.append("insufficient_confirmed_safe_frames")
            elif recover_after_loss and not current_frame_adds_ready_evidence:
                denial_reasons.append("awaiting_new_safe_frame_for_release")
            elif (
                not recover_after_loss
                and projected_release_frames < DYNAMIC_SAFE_OBSERVATION_FRAMES
            ):
                denial_reasons.append("insufficient_confirmed_safe_frames")
            trace_detail["release_denial_reasons"] = tuple(denial_reasons)
            if can_release:
                runtime = launch(state, tick)
                release_ticks.append(tick)
                session_stop_epochs.append(gate.stop_epoch)
                if actual_release_tick is None:
                    actual_release_tick = tick
            else:
                decision = hold(state, tick, snapshot, circular)
                gate_safe_frame_count = decision.consecutive_safe_frames
                if recover_after_loss and release_input_usable and (
                    decision.consecutive_safe_frames > 0
                ):
                    if (
                        current_sequence is not None
                        and current_sequence != last_confirmed_safe_sequence
                    ):
                        confirmed_safe_frame_count += 1
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
                    extend_terminal_actor_trajectory=extend_terminal_actor_trajectory,
                )
                if gate.motion_state is DynamicMotionState.COMPLETED:
                    completion_tick = tick
                    emit_failure_trace(decision)
                    break
                emit_failure_trace(decision)
                continue

        if (
            runtime is not None
            and runtime.issuer is not None
            and planned_transition_stop_requested is not None
            and planned_transition_stop_requested()
            and not stopping_after_loss
        ):
            if protective_stop is None:
                raise RuntimeError("planned transition stop callback is missing")
            protective_stop_started_tick = protective_stop_started_tick or tick
            stopping_after_loss = True
            stopping_after_loss_reason = "post_pass_transition"

        if (
            stopping_after_loss
            or not release_input_usable
            or directional.prediction_set is None
        ):
            if not stopping_after_loss:
                prediction_loss_ticks.append(tick)
                if first_prediction_loss_tick is None:
                    first_prediction_loss_tick = tick
                if protective_stop_started_tick is None:
                    protective_stop_started_tick = tick
                stopping_after_loss = True
                stopping_after_loss_reason = "prediction_loss"
            trace_detail["recovery_reason"] = stopping_after_loss_reason
            # A prediction object is not sufficient authority to keep moving.
            # Fresh EMPTY deliberately carries an empty prediction set, but it
            # is controller-usable only after a conservative post-pass proof.
            # Treat every phase-ineligible directional result as input loss and
            # do not substitute the circular fallback.
            decision = (
                protective_stop(state, tick, snapshot, circular)
                if stopping_after_loss_reason == "post_pass_transition"
                and protective_stop is not None
                else hold(state, tick, snapshot, None)
            )
            gate_safe_frame_count = decision.consecutive_safe_frames
            state = _advance_state(state, decision.command)
            trace.append((tick, directional.status, gate.motion_state, gate.stop_epoch, state))
            minimum_static, minimum_actor = _update_clearances(
                world,
                checker,
                state,
                tick,
                minimum_static,
                minimum_actor,
                extend_terminal_actor_trajectory=extend_terminal_actor_trajectory,
            )
            if gate.motion_state is DynamicMotionState.COMPLETED:
                completion_tick = tick
                emit_failure_trace(decision)
                break
            if gate.motion_state is DynamicMotionState.HOLDING:
                confirmed_stop_ticks.append(tick)
                if stop_confirmed_tick is None:
                    stop_confirmed_tick = tick
                if recover_after_loss:
                    runtime = None
                    stopping_after_loss = False
                    stopping_after_loss_reason = "prediction_loss"
                    confirmed_safe_frame_count = 0
                    last_confirmed_safe_sequence = current_sequence
                    consecutive_ready_frames = 0
                    last_ready_sequence = current_sequence
                    emit_failure_trace(decision)
                    continue
                emit_failure_trace(decision)
                break
            emit_failure_trace(decision)
            continue

        try:
            trace_detail["controller_called"] = True
            record = controller_step(runtime, tick, snapshot, directional, trace_detail)
        except (RuntimeError, TypeError, ValueError) as error:
            trace_detail["controller_exception_type"] = type(error).__name__
            trace_detail["controller_exception_message"] = str(error)
            if trace_detail["authorization_issue_attempted"]:
                trace_detail["authorization_issue_outcome"] = "rejected"
                trace_detail["authorization_issue_error"] = str(error)
            if recover_after_loss and str(error) == (
                "R5-B target is no longer conservatively behind the robot"
            ):
                authorization_loss_ticks.append(tick)
                if protective_stop_started_tick is None:
                    protective_stop_started_tick = tick
                stopping_after_loss = True
                stopping_after_loss_reason = "authorization_loss"
                trace_detail["recovery_reason"] = "authorization_loss"
                decision = hold(state, tick, snapshot, None)
                gate_safe_frame_count = decision.consecutive_safe_frames
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
                    extend_terminal_actor_trajectory=extend_terminal_actor_trajectory,
                )
                if gate.motion_state is DynamicMotionState.COMPLETED:
                    completion_tick = tick
                    emit_failure_trace(decision)
                    break
                if gate.motion_state is DynamicMotionState.HOLDING:
                    confirmed_stop_ticks.append(tick)
                    if stop_confirmed_tick is None:
                        stop_confirmed_tick = tick
                    runtime = None
                    stopping_after_loss = False
                    stopping_after_loss_reason = "prediction_loss"
                    confirmed_safe_frame_count = 0
                    last_confirmed_safe_sequence = current_sequence
                    consecutive_ready_frames = 0
                    last_ready_sequence = current_sequence
                emit_failure_trace(decision)
                continue
            hard_failures.append(f"controller_exception:{tick}:{error}")
            emit_failure_trace()
            break
        controller_calls += 1
        gate_safe_frame_count = record.safety_decision.consecutive_safe_frames
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
            emit_failure_trace(record.safety_decision, record)
            break
        if gate.motion_state is DynamicMotionState.COMPLETED:
            completion_tick = tick
        elif gate.motion_state is not DynamicMotionState.MOVING:
            protective_stop_started_tick = protective_stop_started_tick or tick
            stopping_after_loss = True
            stopping_after_loss_reason = (
                "controller_protective_stop"
                if result is not None and result.controller_requested_protective_stop
                else "gate_protective_stop"
            )
            trace_detail["recovery_reason"] = stopping_after_loss_reason
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
            extend_terminal_actor_trajectory=extend_terminal_actor_trajectory,
        )
        if completion_tick is not None:
            emit_failure_trace(record.safety_decision, record)
            break
        if gate.motion_state is DynamicMotionState.HOLDING:
            if recover_after_loss:
                runtime = None
                stopping_after_loss = False
                stopping_after_loss_reason = "prediction_loss"
                confirmed_safe_frame_count = 0
                last_confirmed_safe_sequence = current_sequence
                consecutive_ready_frames = 0
                last_ready_sequence = current_sequence
                emit_failure_trace(record.safety_decision, record)
                continue
            emit_failure_trace(record.safety_decision, record)
            break
        emit_failure_trace(record.safety_decision, record)

    minimum_required = world.kinematic_contract.vehicle_profile.minimum_clearance_m
    if minimum_static < minimum_required - _TOLERANCE:
        hard_failures.append("actual_static_clearance_below_minimum")
    if minimum_actor is not None and minimum_actor < minimum_required - _TOLERANCE:
        hard_failures.append("actual_actor_clearance_below_minimum")
    if (
        not prefix_only
        and runtime is not None
        and completion_tick is None
        and stop_confirmed_tick is None
    ):
        hard_failures.append("protective_stop_not_confirmed")
    if hard_failures:
        outcome = R5CDiagnosticOutcome.FAILED
    elif completion_tick is not None:
        outcome = R5CDiagnosticOutcome.COMPLETED
    elif gate.motion_state is DynamicMotionState.HOLDING:
        outcome = R5CDiagnosticOutcome.CONSERVATIVE_HOLD
    elif prefix_only:
        outcome = R5CDiagnosticOutcome.FAILED
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
        post_pass_proof_tick=None,
        follow_original_release_tick=None,
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


def _capture_controller_before(
    runtime: _Runtime,
    detail: dict[str, object],
) -> None:
    controller = runtime.pipeline.controller
    detail["executor_active_before"] = getattr(controller, "active_section_index", None)


def _capture_pipeline_trace(
    record: PersistentPipelineStep,
    runtime: _Runtime,
    detail: dict[str, object],
) -> None:
    tick_input = record.tick_input
    result = record.controller_result
    controller = runtime.pipeline.controller
    detail["controller_called"] = True
    detail["controller_status"] = None if result is None else result.status.value
    detail["controller_failure_reason"] = None if result is None else result.failure_reason
    detail["controller_active_section"] = (
        None if result is None else result.active_section_index
    )
    detail["controller_result_hash"] = (
        None if result is None else result.semantic_content_hash
    )
    detail["executor_active_after"] = getattr(controller, "active_section_index", None)
    detail["catchup_attempted"] = getattr(
        controller,
        "last_catchup_attempted",
        None,
    )
    detail["catchup_succeeded"] = getattr(
        controller,
        "last_catchup_succeeded",
        None,
    )
    detail["catchup_failed_guard"] = getattr(
        controller,
        "last_catchup_failed_guard",
        None,
    )
    if tick_input is None:
        return
    window = tick_input.local_window
    detail["window_revision"] = window.window_content_hash
    detail["window_first_section"] = window.sections[0].section_index
    detail["window_last_section"] = window.sections[-1].section_index
    detail["window_source_control_tick"] = window.source_control_tick
    projection = project_reference_cursor(
        tick_input.full_reference,
        tick_input.robot_state.pose,
    )
    detail["projection_section"] = projection.source_section_index
    detail["projection_distance_m"] = projection.distance_to_reference_m
    detail["projection_ambiguous"] = projection.ambiguous
    detail["raw_reference_cursor_m"] = projection.cursor_arc_m
    detail["effective_reference_cursor_m"] = projection.cursor_arc_m
    active_before = detail.get("executor_active_before")
    first = window.sections[0].section_index
    if isinstance(active_before, int) and first > active_before:
        detail["intervening_section_kinds"] = tuple(
            section.section_kind.value
            for section in tick_input.full_reference.sections[active_before + 1 : first]
        )
    else:
        detail["intervening_section_kinds"] = ()


def _append_failure_trace_tick(
    collector: R7FailureTraceCollector,
    *,
    tick: int,
    snapshot: DynamicObservationSnapshot,
    directional: DirectionalPredictionResult,
    release_input_usable: bool,
    consecutive_ready_frames: int,
    last_ready_sequence: int | None,
    gate_state_before: DynamicMotionState,
    stop_epoch_before: int,
    gate_safe_frames_before: int,
    confirmed_safe_frames_before: int,
    confirmed_safe_frames_after: int,
    last_confirmed_safe_sequence: int | None,
    gate_overrides_before: int,
    state_before: RobotState,
    state_after: RobotState,
    runtime_before: _Runtime | None,
    runtime_after: _Runtime | None,
    decision,
    record: PersistentPipelineStep | None,
    detail: dict[str, object],
) -> None:
    frame = snapshot.frame
    result = None if record is None else record.controller_result
    after_state = gate_state_before if decision is None else decision.motion_state
    after_epoch = stop_epoch_before if decision is None else decision.stop_epoch
    safe_after = gate_safe_frames_before if decision is None else decision.consecutive_safe_frames
    gate_overrides_after = (
        gate_overrides_before if decision is None else decision.counters.gate_overrides
    )
    reference = None if runtime_after is None else runtime_after.pipeline.full_reference
    record_values: dict[str, object] = {
        "tick": tick,
        "simulation_time_s": tick * DYNAMIC_CONTROL_PERIOD_S,
        "robot_pose_before": _pose_payload(state_before.pose),
        "robot_twist_before": _twist_payload(state_before.twist),
        "robot_pose_after": _pose_payload(state_after.pose),
        "robot_twist_after": _twist_payload(state_after.twist),
        "observation_event": snapshot.availability.value,
        "observation_sequence": None if frame is None else frame.sequence,
        "observation_status": snapshot.availability.value,
        "observation_age_s": snapshot.age_s,
        "last_event_was_no_frame": snapshot.last_event_was_no_frame,
        "directional_status": directional.status.value,
        "prediction_present": directional.prediction_set is not None,
        "release_input_usable": release_input_usable,
        "consecutive_ready_frames": consecutive_ready_frames,
        "last_ready_sequence": last_ready_sequence,
        "gate_state_before": gate_state_before.value,
        "gate_state_after": after_state.value,
        "stop_epoch_before": stop_epoch_before,
        "stop_epoch_after": after_epoch,
        "gate_consecutive_safe_frames_before": gate_safe_frames_before,
        "gate_consecutive_safe_frames_after": safe_after,
        "confirmed_safe_frame_count_before": confirmed_safe_frames_before,
        "confirmed_safe_frame_count_after": confirmed_safe_frames_after,
        "last_confirmed_safe_sequence": last_confirmed_safe_sequence,
        "gate_override": gate_overrides_after > gate_overrides_before,
        "gate_failure_reasons": (
            () if decision is None else tuple(sorted(decision.failure_reasons))
        ),
        "runtime_present_before": runtime_before is not None,
        "runtime_present_after": runtime_after is not None,
        "actual_stop_confirmed": after_state is DynamicMotionState.HOLDING,
        "reference_session_id": (
            None if reference is None else reference.reference_session_id
        ),
        "reference_stop_epoch": None if reference is None else reference.stop_epoch,
        "resume_authorization_revision": (
            None
            if runtime_after is None or runtime_after.resume_authorization is None
            else runtime_after.resume_authorization.authorization_revision
        ),
        "controller_status": None if result is None else result.status.value,
        "controller_failure_reason": None if result is None else result.failure_reason,
        "controller_active_section": (
            None if result is None else result.active_section_index
        ),
        "controller_command_before_gate": (
            None if record is None else _twist_payload(record.proposal.command)
        ),
        "command_after_gate": (
            None if decision is None else _twist_payload(decision.command)
        ),
        "controller_result_hash": (
            None if result is None else result.semantic_content_hash
        ),
        "window_revision": None,
        "window_first_section": None,
        "window_last_section": None,
        "window_source_control_tick": None,
        "projection_section": None,
        "projection_distance_m": None,
        "projection_ambiguous": None,
        "raw_reference_cursor_m": None,
        "effective_reference_cursor_m": None,
        "executor_active_before": None,
        "executor_active_after": None,
        "catchup_attempted": None,
        "catchup_succeeded": None,
        "catchup_failed_guard": None,
        "intervening_section_kinds": (),
    }
    record_values.update(detail)
    collector.append(record_values)


def _pose_payload(pose: Pose2D) -> dict[str, float]:
    return {"x_m": pose.x, "y_m": pose.y, "yaw_rad": pose.yaw}


def _twist_payload(twist: Twist2D) -> dict[str, float]:
    return {"linear_mps": twist.linear, "angular_radps": twist.angular}


def _actor_states_at_observation_time(
    world: WitnessWorldSnapshot,
    simulation_time_s: float,
    *,
    extend_terminal_actor_trajectory: bool,
) -> tuple[ActorState, ...]:
    """Observe terminal Actors beyond the source world's short evidence horizon.

    The completion diagnostic extends a 39 s public world to an 80 s control
    horizon.  Deleting every Actor at the old boundary fabricated a fresh EMPTY
    observation even though the constant-velocity Actor had simply travelled
    beyond the mapped corridor.  Only trajectories that reach the original
    boundary are extended; Actors intentionally ending earlier stay ended.
    """

    if not isfinite(simulation_time_s) or simulation_time_s < 0.0:
        raise ValueError("extended Actor query time must be finite and non-negative")
    if simulation_time_s <= world.duration_s + _TOLERANCE:
        return world.actor_states_at(min(simulation_time_s, world.duration_s))
    if not extend_terminal_actor_trajectory:
        return ()
    states: list[ActorState] = []
    for actor in world.actors:
        if actor.active_until_s < world.duration_s - _TOLERANCE:
            continue
        elapsed_s = simulation_time_s - actor.active_from_s
        states.append(
            ActorState(
                actor_id=actor.actor_binding_id,
                position=Point2D(
                    actor.start_position.x + actor.velocity.x * elapsed_s,
                    actor.start_position.y + actor.velocity.y * elapsed_s,
                ),
                velocity=actor.velocity,
                radius_m=actor.radius_m,
                trajectory_revision=actor.trajectory_revision,
            )
        )
    return tuple(states)


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
    *,
    extend_terminal_actor_trajectory: bool = False,
) -> tuple[float, float | None]:
    minimum_static = min(minimum_static, checker.clearance(state.pose))
    time_s = (tick + 1) * DYNAMIC_CONTROL_PERIOD_S
    actors = _actor_states_at_observation_time(
        world,
        time_s,
        extend_terminal_actor_trajectory=extend_terminal_actor_trajectory,
    )
    for actor in actors:
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
    "run_r5c_crossing_completion_diagnostic",
    "run_r5c_crossing_diagnostic",
    "run_r5c_crossing_recovery_diagnostic",
    "run_r5c_restop_diagnostic",
    "run_r5c_restop_recovery_diagnostic",
]
