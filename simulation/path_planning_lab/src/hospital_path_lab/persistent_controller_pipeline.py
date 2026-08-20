"""R5 persistent controllers wired to the common dynamic safety gate.

This is the deterministic 20 Hz functional lane described by the R5 research
specification. Python wall-clock time never advances simulation time. The
historical ``computation_time_s`` fault input remains the default; a
same-process runtime may instead ask this module to measure controller-call
time for the existing deadline gate. The module does not run a corpus, inspect
evaluator labels, or grant resume authority.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import cos, isfinite, sin
from time import perf_counter_ns
from typing import TYPE_CHECKING, Protocol

from hospital_path_lab.contracts import GridSnapshot, Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicCommandProposal,
    DynamicSafetyDecision,
    ResumeAuthorization,
)
from hospital_path_lab.dynamic_directional_prediction import DirectionalPredictionSet
from hospital_path_lab.dynamic_observation import DynamicObservationSnapshot
from hospital_path_lab.dynamic_prediction import ActorPredictionSet
from hospital_path_lab.dynamic_safety import DynamicSafetyContext, DynamicSafetyGate
from hospital_path_lab.local_reference_contracts import (
    LocalManeuverReference,
    ReferenceBuildContext,
    ReferenceTravelDirection,
)
from hospital_path_lab.local_reference_validation import LocalReferenceValidation
from hospital_path_lab.local_reference_window import (
    LocalReferenceWindowManager,
    WindowUpdateStatus,
    project_reference_cursor,
)
from hospital_path_lab.persistent_controller_contracts import (
    PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
    PersistentControllerResult,
    PersistentControllerStatus,
    PersistentControllerTickInput,
    PersistentReferenceBinding,
    build_persistent_reference_binding,
)
from hospital_path_lab.reference_section_executor import R5_POSITION_TOLERANCE_M

if TYPE_CHECKING:
    from hospital_path_lab.r5b_temporal_authorization import R5BTemporalExecutionAuthorization

PERSISTENT_CONTROLLER_PIPELINE_VERSION = "persistent-controller-pipeline-v2"
_TOLERANCE = 1e-12


class PersistentController(Protocol):
    """Common callable boundary implemented by the RPP and DWB adapters."""

    name: str

    def step(self, tick_input: PersistentControllerTickInput) -> PersistentControllerResult: ...


@dataclass(frozen=True, slots=True)
class PersistentPipelineStep:
    """One deterministic controller→proposal→gate→chassis transition."""

    tick_id: int
    simulation_time_s: float
    robot_state_before: RobotState
    tick_input: PersistentControllerTickInput | None
    controller_result: PersistentControllerResult | None
    proposal: DynamicCommandProposal
    safety_context: DynamicSafetyContext
    safety_decision: DynamicSafetyDecision
    robot_state_after: RobotState

    def __post_init__(self) -> None:
        if self.tick_id < 0:
            raise ValueError("pipeline tick_id must not be negative")
        if not isfinite(self.simulation_time_s) or self.simulation_time_s < 0.0:
            raise ValueError("pipeline simulation time must be finite and non-negative")
        if (self.tick_input is None) != (self.controller_result is None):
            raise ValueError("pipeline tick input and controller result must be present together")
        if self.tick_input is not None and self.tick_input.controller_tick != self.tick_id:
            raise ValueError("pipeline tick input does not match its step")
        if self.safety_decision.tick_id != self.tick_id:
            raise ValueError("pipeline safety decision does not match its step")


class PersistentControllerPipeline:
    """Execute one R5-A reference with one persistent controller and shared gate.

    The pipeline owns mutable controller/window/gate/chassis state for exactly
    one reference session.  Observation validation and Actor prediction remain
    upstream responsibilities and are supplied as immutable inputs per tick.
    """

    def __init__(
        self,
        *,
        controller: PersistentController,
        build_context: ReferenceBuildContext,
        full_reference: LocalManeuverReference,
        validation: LocalReferenceValidation,
        initial_robot_state: RobotState,
        gate: DynamicSafetyGate | None = None,
        window_manager: LocalReferenceWindowManager | None = None,
        authorization_revision: int = 0,
        initial_tick: int = 0,
    ) -> None:
        if not hasattr(controller, "step") or not getattr(controller, "name", ""):
            raise TypeError("controller must implement the persistent controller protocol")
        if not isinstance(build_context, ReferenceBuildContext):
            raise TypeError("build_context must be a ReferenceBuildContext")
        if not isinstance(full_reference, LocalManeuverReference):
            raise TypeError("full_reference must be a LocalManeuverReference")
        if not isinstance(validation, LocalReferenceValidation):
            raise TypeError("validation must be a LocalReferenceValidation")
        if not isinstance(initial_robot_state, RobotState):
            raise TypeError("initial_robot_state must be a RobotState")
        if isinstance(initial_tick, bool) or not isinstance(initial_tick, int) or initial_tick < 0:
            raise ValueError("initial_tick must be a non-negative exact integer")
        if (
            isinstance(authorization_revision, bool)
            or not isinstance(authorization_revision, int)
            or authorization_revision < 0
        ):
            raise ValueError("authorization_revision must be a non-negative exact integer")
        self.controller = controller
        self.build_context = build_context
        self.full_reference = full_reference
        self.validation = validation
        self.gate = gate or DynamicSafetyGate(
            profile=build_context.vehicle_profile,
            initial_stop_epoch=build_context.stop_epoch,
        )
        if self.gate.stop_epoch != build_context.stop_epoch:
            raise ValueError("gate stop epoch must match the persistent reference context")
        self.window_manager = window_manager or LocalReferenceWindowManager()
        self.authorization_revision = authorization_revision
        self.tick_id = initial_tick
        self.robot_state = initial_robot_state

    def step(
        self,
        *,
        observation_snapshot: DynamicObservationSnapshot,
        prediction_set: ActorPredictionSet | DirectionalPredictionSet | None,
        computation_time_s: float | None = DYNAMIC_CONTROL_PERIOD_S,
        observation_safe: bool = True,
        path_still_valid: bool = True,
        local_safety_recheck_passed: bool = True,
        resume_authorization: ResumeAuthorization | None = None,
        temporal_execution_authorization: R5BTemporalExecutionAuthorization | None = None,
        grid_snapshot: GridSnapshot | None = None,
        mission_cancelled: bool = False,
    ) -> PersistentPipelineStep:
        """Run one exact 20 Hz step; current twist moves before gate output applies.

        The historical default remains the deterministic simulation fault value
        of ``0.05`` seconds.  A caller that passes ``None`` asks this adapter to
        measure just the controller invocation and send that measured value to
        the existing shared gate.  The latter is intended for a same-process
        runtime facade; it does not advance simulation time.
        """

        if computation_time_s is not None and (
            not isfinite(computation_time_s) or computation_time_s < 0.0
        ):
            raise ValueError("computation_time_s must be finite and non-negative")
        for flag in (
            observation_safe,
            path_still_valid,
            local_safety_recheck_passed,
            mission_cancelled,
        ):
            if not isinstance(flag, bool):
                raise TypeError("pipeline safety flags must be bool values")

        tick = self.tick_id
        simulation_time_s = tick * DYNAMIC_CONTROL_PERIOD_S
        state_before = self.robot_state
        current_grid_snapshot = grid_snapshot or self.build_context.static_grid_snapshot
        if not isinstance(current_grid_snapshot, GridSnapshot):
            raise TypeError("grid_snapshot must be a GridSnapshot when supplied")
        frozen_metadata = self.build_context.static_grid_snapshot.metadata
        current_metadata = current_grid_snapshot.metadata
        if (
            current_grid_snapshot.grid is not self.build_context.static_grid_snapshot.grid
            or current_grid_snapshot.forbidden_cells
            != self.build_context.static_grid_snapshot.forbidden_cells
            or current_metadata.map_id != frozen_metadata.map_id
            or current_metadata.map_revision != frozen_metadata.map_revision
            or current_metadata.mission_revision != frozen_metadata.mission_revision
            or current_metadata.content_hash != frozen_metadata.content_hash
        ):
            raise ValueError("current grid snapshot changed the frozen spatial source")
        current_build_context = replace(
            self.build_context,
            current_robot_pose=state_before.pose,
            control_tick=tick,
            simulation_time_s=simulation_time_s,
            static_grid_snapshot=current_grid_snapshot,
            context_content_hash="",
        )
        active_section_index = getattr(
            self.controller,
            "active_section_index",
            None,
        )
        preserve_section_index = None
        if active_section_index is not None:
            projection = project_reference_cursor(
                self.full_reference,
                state_before.pose,
            )
            if (
                not projection.ambiguous
                and projection.source_section_index > active_section_index
                and projection.distance_to_reference_m
                > R5_POSITION_TOLERANCE_M + _TOLERANCE
            ):
                preserve_section_index = active_section_index
        update = self.window_manager.update(
            current_build_context,
            self.full_reference,
            self.validation,
            preserve_section_index=preserve_section_index,
        )
        if update.status is not WindowUpdateStatus.WINDOW_READY or update.window is None:
            raise RuntimeError(f"persistent reference window unavailable:{update.reason_code}")

        binding = build_persistent_reference_binding(self.full_reference, update.window)
        if binding.stop_epoch != self.gate.stop_epoch:
            return self._hold_invalidated_reference(
                tick=tick,
                simulation_time_s=simulation_time_s,
                state_before=state_before,
                binding=binding,
                observation_snapshot=observation_snapshot,
                prediction_set=prediction_set,
                observation_safe=observation_safe,
                resume_authorization=resume_authorization,
                mission_cancelled=mission_cancelled,
            )
        tick_input = PersistentControllerTickInput(
            schema_version=PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
            controller_tick=tick,
            simulation_time_s=simulation_time_s,
            full_reference=self.full_reference,
            local_window=update.window,
            reference_binding=binding,
            robot_state=state_before,
            static_grid_snapshot=current_build_context.static_grid_snapshot,
            validated_observation=observation_snapshot,
            actor_prediction_set=prediction_set,
            vehicle_profile=current_build_context.vehicle_profile,
            current_gate_motion_state=self.gate.motion_state,
            current_gate_stop_epoch=self.gate.stop_epoch,
            current_resume_authorization_revision=(
                None
                if resume_authorization is None
                else resume_authorization.authorization_revision
            ),
            temporal_execution_authorization=temporal_execution_authorization,
        )
        controller_started_ns = perf_counter_ns()
        controller_result = self.controller.step(tick_input)
        measured_controller_time_s = (perf_counter_ns() - controller_started_ns) / 1_000_000_000
        gate_computation_time_s = (
            measured_controller_time_s if computation_time_s is None else computation_time_s
        )
        direction_failure = _section_bound_direction_failure(
            controller_result,
            tick_input,
        )
        if direction_failure is not None:
            controller_result = replace(
                controller_result,
                status=PersistentControllerStatus.SECTION_EXECUTION_FAILED,
                requested_twist=Twist2D(),
                predicted_trajectory=(),
                failure_reason=direction_failure,
                decision_trace=(
                    *controller_result.decision_trace,
                    f"pipeline_direction_failure={direction_failure}",
                ),
                planned_section_stop=False,
                controller_requested_protective_stop=True,
                no_safe_candidate=False,
                semantic_content_hash="",
            )
        proposal = persistent_result_to_dynamic_proposal(
            controller_result,
            tick_input=tick_input,
            computation_time_s=gate_computation_time_s,
        )
        context = DynamicSafetyContext(
            tick_id=tick,
            simulation_time_s=simulation_time_s,
            mission_id=binding.mission_id,
            authorization_revision=self.authorization_revision,
            grid_snapshot=tick_input.static_grid_snapshot,
            observation_snapshot=observation_snapshot,
            prediction_set=prediction_set,
            path_still_valid=path_still_valid,
            local_safety_recheck_passed=local_safety_recheck_passed,
            observation_safe=observation_safe,
            resume_authorization=resume_authorization,
            goal_reached=(controller_result.status is PersistentControllerStatus.COMPLETED),
            mission_cancelled=mission_cancelled,
            reference_binding=binding,
        )
        decision = self.gate.step(
            proposal,
            robot_state=state_before,
            context=context,
        )

        # R5 deterministic lane: motion during this interval uses the twist that
        # was already active at the snapshot.  The gate command becomes the next
        # tick's twist; Python computation wall time never advances T_sim.
        pose_after = integrate_persistent_chassis_pose(
            state_before.pose,
            state_before.twist,
            DYNAMIC_CONTROL_PERIOD_S,
        )
        state_after = RobotState(pose_after, decision.command)
        record = PersistentPipelineStep(
            tick_id=tick,
            simulation_time_s=simulation_time_s,
            robot_state_before=state_before,
            tick_input=tick_input,
            controller_result=controller_result,
            proposal=proposal,
            safety_context=context,
            safety_decision=decision,
            robot_state_after=state_after,
        )
        self.robot_state = state_after
        self.tick_id += 1
        return record

    def synchronize_external_robot_state(self, robot_state: RobotState) -> None:
        """Use an externally estimated robot state for the next controller tick.

        This is intentionally a narrow state-injection seam: it does not alter
        the controller, gate, reference session, or control tick.  A runtime
        integration calls it immediately before :meth:`step` so the gate checks
        the actual reported pose/twist instead of the pipeline's previous
        simulation integration result.
        """

        if not isinstance(robot_state, RobotState):
            raise TypeError("robot_state must be a RobotState")
        values = (
            robot_state.pose.x,
            robot_state.pose.y,
            robot_state.pose.yaw,
            robot_state.twist.linear,
            robot_state.twist.angular,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("external robot_state values must be finite")
        self.robot_state = robot_state

    def _hold_invalidated_reference(
        self,
        *,
        tick: int,
        simulation_time_s: float,
        state_before: RobotState,
        binding: PersistentReferenceBinding,
        observation_snapshot: DynamicObservationSnapshot,
        prediction_set: ActorPredictionSet | DirectionalPredictionSet | None,
        observation_safe: bool,
        resume_authorization: ResumeAuthorization | None,
        mission_cancelled: bool,
    ) -> PersistentPipelineStep:
        """Keep braking/holding after a protective stop invalidates the R4 reference."""

        grid_snapshot = self.build_context.static_grid_snapshot
        metadata = grid_snapshot.metadata
        frame = observation_snapshot.frame
        observation_hash = "observation-unavailable" if frame is None else frame.content_hash
        proposal = DynamicCommandProposal(
            source_tick_id=tick,
            command=Twist2D(),
            computation_time_s=DYNAMIC_CONTROL_PERIOD_S,
            mission_id=binding.mission_id,
            map_id=binding.map_id,
            map_revision=binding.map_revision,
            mission_revision=binding.mission_revision,
            observation_revision=metadata.observation_revision,
            grid_content_hash=metadata.content_hash,
            observation_content_hash=observation_hash,
            reference_binding=binding,
        )
        context = DynamicSafetyContext(
            tick_id=tick,
            simulation_time_s=simulation_time_s,
            mission_id=binding.mission_id,
            authorization_revision=self.authorization_revision,
            grid_snapshot=grid_snapshot,
            observation_snapshot=observation_snapshot,
            prediction_set=prediction_set,
            path_still_valid=False,
            local_safety_recheck_passed=False,
            observation_safe=observation_safe,
            resume_authorization=resume_authorization,
            goal_reached=False,
            mission_cancelled=mission_cancelled,
            reference_binding=binding,
        )
        decision = self.gate.step(proposal, robot_state=state_before, context=context)
        pose_after = integrate_persistent_chassis_pose(
            state_before.pose,
            state_before.twist,
            DYNAMIC_CONTROL_PERIOD_S,
        )
        state_after = RobotState(pose_after, decision.command)
        record = PersistentPipelineStep(
            tick_id=tick,
            simulation_time_s=simulation_time_s,
            robot_state_before=state_before,
            tick_input=None,
            controller_result=None,
            proposal=proposal,
            safety_context=context,
            safety_decision=decision,
            robot_state_after=state_after,
        )
        self.robot_state = state_after
        self.tick_id += 1
        return record


def persistent_result_to_dynamic_proposal(
    result: PersistentControllerResult,
    *,
    tick_input: PersistentControllerTickInput,
    computation_time_s: float,
) -> DynamicCommandProposal:
    """Preserve a result's echoed reference while adding gate provenance.

    This converter intentionally does not "repair" an old result with the
    current binding.  A delayed or reordered result therefore reaches the gate
    with its original capability and is rejected at the final boundary.
    """

    if not isinstance(result, PersistentControllerResult):
        raise TypeError("result must be a PersistentControllerResult")
    if not isinstance(tick_input, PersistentControllerTickInput):
        raise TypeError("tick_input must be a PersistentControllerTickInput")
    if not isfinite(computation_time_s) or computation_time_s < 0.0:
        raise ValueError("computation_time_s must be finite and non-negative")
    metadata = tick_input.static_grid_snapshot.metadata
    frame = tick_input.validated_observation.frame
    observation_hash = "observation-unavailable" if frame is None else frame.content_hash
    binding = result.reference_binding_echo
    return DynamicCommandProposal(
        source_tick_id=result.source_controller_tick,
        command=result.requested_twist,
        computation_time_s=computation_time_s,
        mission_id=binding.mission_id,
        map_id=binding.map_id,
        map_revision=binding.map_revision,
        mission_revision=binding.mission_revision,
        observation_revision=metadata.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=observation_hash,
        trajectory=result.predicted_trajectory,
        controller_requested_stop=result.controller_requested_protective_stop,
        no_safe_candidate=result.no_safe_candidate,
        reference_binding=binding,
    )


def _section_bound_direction_failure(
    result: PersistentControllerResult,
    tick_input: PersistentControllerTickInput,
) -> str | None:
    if result.status is not PersistentControllerStatus.COMMAND_FOUND:
        return None
    index = result.active_section_index
    if index is None or not 0 <= index < len(tick_input.full_reference.sections):
        return "active_section_direction_unavailable"
    direction = tick_input.full_reference.sections[index].travel_direction
    linear = result.requested_twist.linear
    if direction is ReferenceTravelDirection.FORWARD:
        return "forward_section_negative_command" if linear < -_TOLERANCE else None
    if direction is ReferenceTravelDirection.REVERSE:
        if linear > _TOLERANCE:
            return "reverse_section_positive_command"
        if linear < -min(0.10, tick_input.vehicle_profile.max_reverse_speed_mps) - _TOLERANCE:
            return "reverse_speed_limit_exceeded"
        return None
    return "non_command_section_translation" if abs(linear) > _TOLERANCE else None


def integrate_persistent_chassis_pose(
    pose: Pose2D,
    twist: Twist2D,
    duration_s: float,
) -> Pose2D:
    """Exact constant-twist differential-drive integration for the R5 chassis."""

    if not isfinite(duration_s) or duration_s < 0.0:
        raise ValueError("duration_s must be finite and non-negative")
    if abs(twist.angular) <= _TOLERANCE:
        return Pose2D(
            pose.x + twist.linear * cos(pose.yaw) * duration_s,
            pose.y + twist.linear * sin(pose.yaw) * duration_s,
            pose.yaw,
        )
    next_yaw = pose.yaw + twist.angular * duration_s
    radius = twist.linear / twist.angular
    return Pose2D(
        pose.x + radius * (sin(next_yaw) - sin(pose.yaw)),
        pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
        next_yaw,
    )
