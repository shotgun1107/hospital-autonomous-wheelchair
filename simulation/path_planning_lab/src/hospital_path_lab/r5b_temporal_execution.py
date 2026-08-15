"""R5-B Ideal causal stream execution for persistent RPP and DWB.

This module is a public, simulation-only functional lane.  It holds the robot
stationary while the directional Actor history warms up, obtains the existing
shared-gate stop confirmation, and only then lets a persistent controller track
the independently validated temporal reference.  Ground truth is used to
generate the Ideal observation stream and to score the run; it is never passed
to either controller.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import atan2, hypot, isfinite, pi
from pathlib import Path

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import GridSnapshot, Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicCommandProposal,
    DynamicGroundTruthFrame,
    DynamicMotionState,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalActorPredictor,
    DirectionalPredictionResult,
    DirectionalPredictionStatus,
)
from hospital_path_lab.dynamic_observation import (
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    DynamicObservationSnapshot,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    generate_dynamic_observation_slots,
)
from hospital_path_lab.dynamic_prediction import (
    ActorPredictionSet,
    build_actor_prediction_set,
)
from hospital_path_lab.dynamic_safety import (
    DynamicSafetyContext,
    DynamicSafetyGate,
    build_resume_authorization,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PersistentSourceDerivedDwbController,
)
from hospital_path_lab.local_reference_contracts import LocalManeuverReference
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.persistent_controller_contracts import PersistentControllerStatus
from hospital_path_lab.persistent_controller_pipeline import (
    PersistentController,
    PersistentControllerPipeline,
    PersistentPipelineStep,
)
from hospital_path_lab.persistent_rpp_controller import PersistentRppController
from hospital_path_lab.r5b_temporal_authorization import (
    R5BTemporalAuthorizationIssuer,
)
from hospital_path_lab.r5b_temporal_evidence import (
    R5B_CAUSAL_RELEASE_TICK,
    frozen_r2_archive_path,
)
from hospital_path_lab.r5b_temporal_reference import (
    R5BTemporalReferenceBundle,
    build_r5b_temporal_reference_bundles,
)

R5B_TEMPORAL_EXECUTION_VERSION = "r5b-ideal-temporal-execution-v2"
R5B_AUTHORIZATION_REVISION = 1
R5B_REJOIN_DISTANCE_M = 0.10
R5B_REJOIN_HEADING_RAD = 10.0 * pi / 180.0
R5B_REJOIN_HOLD_TICKS = 10
_TOLERANCE = 1e-12


@dataclass(frozen=True, slots=True)
class R5BTemporalExecutionResult:
    controller_name: str
    public_id: str
    corpus_ordinal: int
    side: str
    completed: bool
    first_controller_tick: int | None
    first_motion_tick: int | None
    departure_tick: int | None
    overtake_tick: int | None
    rejoin_tick: int | None
    completion_tick: int | None
    maximum_lateral_deviation_m: float
    minimum_actor_clearance_m: float | None
    minimum_static_clearance_m: float
    gate_override_count: int
    controller_call_count: int
    final_pose: Pose2D
    final_section_index: int | None
    last_target_present_tick: int | None
    last_target_present_robot_pose: Pose2D | None
    last_target_progress_gap_m: float | None
    hard_failures: tuple[str, ...]
    trace_content_hash: str

    @property
    def passed(self) -> bool:
        return all(
            (
                self.completed,
                self.first_controller_tick == R5B_CAUSAL_RELEASE_TICK,
                self.first_motion_tick is not None,
                self.departure_tick is not None,
                self.overtake_tick is not None,
                self.rejoin_tick is not None,
                not self.hard_failures,
            )
        )


class _IdealCausalStream:
    def __init__(self, bundle: R5BTemporalReferenceBundle, *, tick_limit: int) -> None:
        world = bundle.source.world
        self._source = DynamicObservationSourceIdentity(
            stream_id="r5b-ideal-causal-public",
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
                mission_revision=bundle.reference.mission_revision,
            )
            for tick in range(tick_limit + 1)
        )
        self._slots = generate_dynamic_observation_slots(
            frames,
            source=self._source,
            profile=FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
        )
        self._validator = DynamicObservationValidator(
            self._source,
            FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
        )
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
            if slot.frame is None:  # Ideal has no dropout; keep fail-closed semantics.
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
                    raise RuntimeError("generated R5-B Ideal frame failed validation")
            self._next_slot += 1
        snapshot = self._validator.snapshot(control_time_s=time_s)
        circular = build_actor_prediction_set(snapshot) if snapshot.usable else None
        directional = self._directional.update(snapshot)
        return snapshot, circular, directional


def run_r5b_temporal_case(
    bundle: R5BTemporalReferenceBundle,
    *,
    controller: PersistentController,
    tick_limit: int = 1_800,
) -> R5BTemporalExecutionResult:
    """Run one public reference/controller pair without wall-clock qualification."""

    if tick_limit <= R5B_CAUSAL_RELEASE_TICK:
        raise ValueError("R5-B execution tick limit must extend past causal release")
    world = bundle.source.world
    stream = _IdealCausalStream(bundle, tick_limit=tick_limit)
    gate = DynamicSafetyGate(
        profile=bundle.build_context.vehicle_profile,
        initial_stop_epoch=0,
    )
    state = RobotState(world.initial_state.pose, Twist2D())
    pre_release_decisions = []
    release_snapshot: DynamicObservationSnapshot | None = None
    release_directional: DirectionalPredictionResult | None = None
    for tick in range(R5B_CAUSAL_RELEASE_TICK):
        snapshot, circular, directional = stream.tick(tick)
        decision = _pre_release_hold_step(
            bundle,
            gate=gate,
            robot_state=state,
            tick_id=tick,
            snapshot=snapshot,
            prediction_set=circular,
        )
        pre_release_decisions.append(decision)
        if decision.command != Twist2D():
            raise RuntimeError("R5-B pre-release gate emitted nonzero motion")
        state = RobotState(state.pose, decision.command)

    snapshot, _, directional = stream.tick(R5B_CAUSAL_RELEASE_TICK)
    release_snapshot = snapshot
    release_directional = directional
    if directional.status is not DirectionalPredictionStatus.READY:
        raise RuntimeError("R5-B directional prediction was not READY at frozen release")
    if gate.motion_state is not DynamicMotionState.HOLDING or gate.stop_epoch != 1:
        raise RuntimeError("R5-B shared gate did not establish the frozen stop epoch")

    resume = build_resume_authorization(
        mission_id=bundle.reference.mission_id,
        stop_epoch=gate.stop_epoch,
        issued_or_revalidated_at_s=R5B_CAUSAL_RELEASE_TICK * DYNAMIC_CONTROL_PERIOD_S,
        authorization_revision=R5B_AUTHORIZATION_REVISION,
    )
    issuer = R5BTemporalAuthorizationIssuer()
    pipeline = PersistentControllerPipeline(
        controller=controller,
        build_context=bundle.build_context,
        full_reference=bundle.reference,
        validation=bundle.validation,
        initial_robot_state=state,
        gate=gate,
        authorization_revision=R5B_AUTHORIZATION_REVISION,
        initial_tick=R5B_CAUSAL_RELEASE_TICK,
    )

    records: list[PersistentPipelineStep] = []
    failures: list[str] = []
    first_motion_tick: int | None = None
    departure_tick: int | None = None
    overtake_tick: int | None = None
    rejoin_tick: int | None = None
    completion_tick: int | None = None
    maximum_deviation = 0.0
    minimum_actor: float | None = None
    minimum_static = float("inf")
    rejoin_streak = 0
    passed_actor_once = False
    terminal_stop_started = False
    last_target_present_tick: int | None = None
    last_target_present_robot_pose: Pose2D | None = None
    last_target_progress_gap_m: float | None = None
    checker = CollisionChecker(
        bundle.build_context.static_grid_snapshot.grid,
        bundle.build_context.vehicle_profile,
        forbidden_cells=bundle.build_context.static_grid_snapshot.forbidden_cells,
    )

    for tick in range(R5B_CAUSAL_RELEASE_TICK, tick_limit):
        if tick == R5B_CAUSAL_RELEASE_TICK:
            snapshot = release_snapshot
            directional_result = release_directional
        else:
            snapshot, _, directional_result = stream.tick(tick)
        assert snapshot is not None and directional_result is not None
        if directional_result.prediction_set is None:
            failures.append(
                f"directional_prediction_not_ready:{tick}:{directional_result.status.value}"
            )
            break
        try:
            temporal_authorization = issuer.issue(
                reference=bundle.reference,
                temporal_evidence=bundle.temporal_evidence,
                temporal_geometry=bundle.temporal_geometry,
                robot_state=pipeline.robot_state,
                vehicle_profile=bundle.build_context.vehicle_profile,
                observation_snapshot=snapshot,
                prediction_result=directional_result,
                controller_tick=tick,
                simulation_time_s=tick * DYNAMIC_CONTROL_PERIOD_S,
                gate_motion_state=gate.motion_state,
                gate_stop_epoch=gate.stop_epoch,
                resume_authorization_revision=(
                    R5B_AUTHORIZATION_REVISION
                    if tick == R5B_CAUSAL_RELEASE_TICK
                    else None
                ),
                actual_stop_confirmed=(tick == R5B_CAUSAL_RELEASE_TICK),
                local_safety_recheck_passed=True,
            )
        except (TypeError, ValueError) as error:
            failures.append(f"temporal_authorization_failed:{tick}:{error}")
            break
        record = pipeline.step(
            observation_snapshot=snapshot,
            prediction_set=directional_result.prediction_set,
            resume_authorization=(
                resume if tick == R5B_CAUSAL_RELEASE_TICK else None
            ),
            temporal_execution_authorization=temporal_authorization,
            grid_snapshot=_grid_snapshot_for_observation(bundle, snapshot),
        )
        records.append(record)
        if record.safety_decision.command != Twist2D() and first_motion_tick is None:
            first_motion_tick = tick
        if record.safety_decision.failure_reasons:
            failures.extend(
                f"gate:{tick}:{reason}" for reason in record.safety_decision.failure_reasons
            )
        result = record.controller_result
        if result is None:
            failures.append(f"controller_not_called:{tick}")
            break
        if result.status in {
            PersistentControllerStatus.HOLD_REQUESTED,
            PersistentControllerStatus.NO_SAFE_COMMAND,
            PersistentControllerStatus.INVALID_REFERENCE_INPUT,
            PersistentControllerStatus.STALE_REFERENCE_INPUT,
            PersistentControllerStatus.LATE_RESULT,
            PersistentControllerStatus.SECTION_EXECUTION_FAILED,
        }:
            failures.append(f"controller:{tick}:{result.status.value}:{result.failure_reason}")
            break
        if result.status is PersistentControllerStatus.COMPLETED:
            terminal_stop_started = True

        pose = record.robot_state_after.pose
        deviation, heading_error = _original_reference_error(bundle.reference, pose)
        maximum_deviation = max(maximum_deviation, deviation)
        if departure_tick is None and deviation > R5B_REJOIN_DISTANCE_M + _TOLERANCE:
            departure_tick = tick
        static_clearance = checker.clearance(pose)
        minimum_static = min(minimum_static, static_clearance)
        actor = world.actors[0].state_at(
            (record.tick_id + 1) * DYNAMIC_CONTROL_PERIOD_S
        )
        if actor is not None:
            last_target_present_tick = tick
            last_target_present_robot_pose = pose
            actor_clearance = oriented_footprint_circle_surface_distance(
                pose,
                circle_center=(actor.position.x, actor.position.y),
                circle_radius_m=actor.radius_m,
                profile=bundle.build_context.vehicle_profile,
            )
            minimum_actor = (
                actor_clearance
                if minimum_actor is None
                else min(minimum_actor, actor_clearance)
            )
            robot_progress = _progress_along_original(bundle.reference, pose)
            actor_progress = _progress_along_original(
                bundle.reference,
                Pose2D(actor.position.x, actor.position.y, 0.0),
            )
            last_target_progress_gap_m = robot_progress - actor_progress
            overlap_extent = (
                bundle.build_context.vehicle_profile.collision_length_m / 2.0
                + actor.radius_m
            )
            if abs(robot_progress - actor_progress) <= overlap_extent:
                passed_actor_once = True
            if passed_actor_once and robot_progress > actor_progress + overlap_extent:
                overtake_tick = overtake_tick or tick
        if overtake_tick is not None and deviation <= R5B_REJOIN_DISTANCE_M + _TOLERANCE:
            if heading_error <= R5B_REJOIN_HEADING_RAD + _TOLERANCE:
                rejoin_streak += 1
                if rejoin_streak >= R5B_REJOIN_HOLD_TICKS and rejoin_tick is None:
                    rejoin_tick = tick
            else:
                rejoin_streak = 0
        elif overtake_tick is not None:
            rejoin_streak = 0

        if record.safety_decision.motion_state is DynamicMotionState.COMPLETED:
            completion_tick = tick
            break
        if (
            gate.motion_state is not DynamicMotionState.MOVING
            and not (
                terminal_stop_started
                and gate.motion_state is DynamicMotionState.BRAKING
            )
        ):
            failures.append(f"shared_gate_left_moving_state:{tick}:{gate.motion_state.value}")
            break

    completed = completion_tick is not None
    if not completed and not failures:
        failures.append("simulation_timeout")
    if departure_tick is None:
        failures.append("departure_not_observed")
    if overtake_tick is None:
        failures.append("ordered_overtake_not_observed")
    if rejoin_tick is None:
        failures.append("sustained_rejoin_not_observed")
    if minimum_static < bundle.build_context.vehicle_profile.minimum_clearance_m - _TOLERANCE:
        failures.append("actual_static_clearance_below_minimum")
    if (
        minimum_actor is not None
        and minimum_actor
        < bundle.build_context.vehicle_profile.minimum_clearance_m - _TOLERANCE
    ):
        failures.append("actual_actor_clearance_below_minimum")

    trace_payload = tuple(
        (
            item.tick_id,
            item.robot_state_before,
            item.robot_state_after,
            item.controller_result.semantic_content_hash if item.controller_result else None,
            item.safety_decision,
        )
        for item in records
    )
    return R5BTemporalExecutionResult(
        controller_name=controller.name,
        public_id=bundle.source.public_id,
        corpus_ordinal=bundle.source.corpus_ordinal,
        side=bundle.source.side.value,
        completed=completed,
        first_controller_tick=(records[0].tick_id if records else None),
        first_motion_tick=first_motion_tick,
        departure_tick=departure_tick,
        overtake_tick=overtake_tick,
        rejoin_tick=rejoin_tick,
        completion_tick=completion_tick,
        maximum_lateral_deviation_m=maximum_deviation,
        minimum_actor_clearance_m=minimum_actor,
        minimum_static_clearance_m=minimum_static,
        gate_override_count=gate.counters.gate_overrides,
        controller_call_count=len(records),
        final_pose=(records[-1].robot_state_after.pose if records else state.pose),
        final_section_index=(
            None
            if not records or records[-1].controller_result is None
            else records[-1].controller_result.active_section_index
        ),
        last_target_present_tick=last_target_present_tick,
        last_target_present_robot_pose=last_target_present_robot_pose,
        last_target_progress_gap_m=last_target_progress_gap_m,
        hard_failures=tuple(dict.fromkeys(failures)),
        trace_content_hash=canonical_content_hash(trace_payload),
    )


def run_first_r5b_pair(
    repository_root: Path,
    *,
    tick_limit: int = 1_800,
) -> tuple[R5BTemporalExecutionResult, R5BTemporalExecutionResult]:
    """Focused first public LEFT case used before the full public matrix."""

    bundle = build_r5b_temporal_reference_bundles(
        frozen_r2_archive_path(repository_root)
    )[0]
    return (
        run_r5b_temporal_case(
            bundle,
            controller=PersistentRppController(),
            tick_limit=tick_limit,
        ),
        run_r5b_temporal_case(
            bundle,
            controller=PersistentSourceDerivedDwbController(),
            tick_limit=tick_limit,
        ),
    )


def _pre_release_hold_step(
    bundle: R5BTemporalReferenceBundle,
    *,
    gate: DynamicSafetyGate,
    robot_state: RobotState,
    tick_id: int,
    snapshot: DynamicObservationSnapshot,
    prediction_set: ActorPredictionSet | None,
):
    current_grid_snapshot = _grid_snapshot_for_observation(bundle, snapshot)
    metadata = current_grid_snapshot.metadata
    frame = snapshot.frame
    proposal = DynamicCommandProposal(
        source_tick_id=tick_id,
        command=Twist2D(),
        computation_time_s=DYNAMIC_CONTROL_PERIOD_S,
        mission_id=bundle.reference.mission_id,
        map_id=metadata.map_id,
        map_revision=metadata.map_revision,
        mission_revision=metadata.mission_revision,
        observation_revision=metadata.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=(
            "observation-unavailable" if frame is None else frame.content_hash
        ),
    )
    context = DynamicSafetyContext(
        tick_id=tick_id,
        simulation_time_s=tick_id * DYNAMIC_CONTROL_PERIOD_S,
        mission_id=bundle.reference.mission_id,
        authorization_revision=R5B_AUTHORIZATION_REVISION,
        grid_snapshot=current_grid_snapshot,
        observation_snapshot=snapshot,
        prediction_set=prediction_set,
        path_still_valid=True,
        local_safety_recheck_passed=True,
        observation_safe=True,
    )
    return gate.step(proposal, robot_state=robot_state, context=context)


def _grid_snapshot_for_observation(
    bundle: R5BTemporalReferenceBundle,
    snapshot: DynamicObservationSnapshot,
) -> GridSnapshot:
    frozen = bundle.build_context.static_grid_snapshot
    frame = snapshot.frame
    revision = frozen.metadata.observation_revision if frame is None else frame.observation_revision
    return GridSnapshot(
        metadata=replace(frozen.metadata, observation_revision=revision),
        grid=frozen.grid,
        forbidden_cells=frozen.forbidden_cells,
    )


def _original_reference_error(
    reference: LocalManeuverReference,
    pose: Pose2D,
) -> tuple[float, float]:
    points = reference.original_reference if hasattr(reference, "original_reference") else ()
    # LocalManeuverReference intentionally stores only the alternate path.  Its
    # original route hash is bound through the build context, so execution uses
    # the source world's immutable reference below via the first/last rejoin line.
    del points
    start = reference.knots[0].pose
    end = reference.knots[-1].pose
    dx = end.x - start.x
    dy = end.y - start.y
    length = hypot(dx, dy)
    if length <= _TOLERANCE:
        return hypot(pose.x - start.x, pose.y - start.y), 0.0
    cross = abs((pose.x - start.x) * dy - (pose.y - start.y) * dx) / length
    heading = atan2(dy, dx)
    return cross, abs(_normalize_angle(pose.yaw - heading))


def _progress_along_original(reference: LocalManeuverReference, pose: Pose2D) -> float:
    start = reference.knots[0].pose
    end = reference.knots[-1].pose
    dx = end.x - start.x
    dy = end.y - start.y
    squared = dx * dx + dy * dy
    if squared <= _TOLERANCE:
        return 0.0
    return ((pose.x - start.x) * dx + (pose.y - start.y) * dy) / squared * hypot(dx, dy)


def _normalize_angle(value: float) -> float:
    while value > pi:
        value -= 2.0 * pi
    while value < -pi:
        value += 2.0 * pi
    return value


def assert_finite_r5b_result(result: R5BTemporalExecutionResult) -> None:
    values = (
        result.maximum_lateral_deviation_m,
        result.minimum_static_clearance_m,
    )
    if result.minimum_actor_clearance_m is not None:
        values = (*values, result.minimum_actor_clearance_m)
    if not all(isfinite(value) for value in values):
        raise ValueError("R5-B execution result contains non-finite metrics")


__all__ = [
    "R5B_TEMPORAL_EXECUTION_VERSION",
    "R5BTemporalExecutionResult",
    "assert_finite_r5b_result",
    "run_first_r5b_pair",
    "run_r5b_temporal_case",
]
