"""Stateful, same-process facade over the existing R7 controller stack."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import isclose

from hospital_path_lab.contracts import GridSnapshot, RobotState, Twist2D
from hospital_path_lab.cpp_dwb_safety_core import CPP_DWB_SAFETY_CORE_AVAILABLE
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicCommandProposal,
    DynamicMotionState,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalActorPredictor,
    DirectionalPredictionResult,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
    DynamicObservationValidationReason,
    DynamicObservationValidator,
)
from hospital_path_lab.dynamic_safety import DynamicSafetyContext, DynamicSafetyGate
from hospital_path_lab.local_algorithms.dwb_reference.cpp_full_core import (
    CPP_DWB_FULL_CORE_AVAILABLE,
)
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PersistentSourceDerivedDwbController,
)
from hospital_path_lab.persistent_controller_pipeline import PersistentControllerPipeline
from hospital_path_lab.persistent_rpp_controller import PersistentRppController

from .adapters import (
    RuntimeAdapterError,
    build_observation_source,
    build_runtime_grid_snapshot,
    grid_snapshot_for_observation,
    observation_profile_for,
    to_observation_frame,
    to_resume_authorization,
    to_robot_state,
)
from .contracts import (
    RuntimeCommand,
    RuntimeConfig,
    RuntimeControllerKind,
    RuntimeDiagnostics,
    RuntimeMission,
    RuntimePose,
    RuntimeStepInput,
)
from .global_planning import plan_runtime_reference_path
from .reference import RuntimeReferenceError, build_runtime_follow_reference


class RuntimeStateError(RuntimeError):
    """The caller used the stateful facade outside its mission lifecycle."""


@dataclass(slots=True)
class _RuntimeSession:
    mission: RuntimeMission
    grid_snapshot: GridSnapshot
    validator: DynamicObservationValidator
    predictor: DirectionalActorPredictor
    gate: DynamicSafetyGate
    controller: PersistentSourceDerivedDwbController | PersistentRppController
    pipeline: PersistentControllerPipeline | None = None
    next_control_tick: int = 0
    last_robot_state: RobotState | None = None
    last_observation_snapshot: DynamicObservationSnapshot | None = None
    last_prediction: DirectionalPredictionResult | None = None
    next_observation_sequence: int = 0
    prestart_safety_active: bool = False
    blocked_reason: str | None = None


class R7Runtime:
    """One mission-scoped R7 runtime instance for a Python backend process.

    The facade owns only assembly and lifetime.  It delegates observation
    validation, directional prediction, persistent controller state, and the
    shared safety gate to their existing R7 modules.  It is deliberately not an
    HTTP server and remains limited to the frozen simulation-only vehicle
    profile used by R7.
    """

    def __init__(self, config: RuntimeConfig | None = None) -> None:
        if config is None:
            config = RuntimeConfig()
        if not isinstance(config, RuntimeConfig):
            raise TypeError("config must be a RuntimeConfig")
        self.config = config
        self._session: _RuntimeSession | None = None
        self._started_mission_keys: set[tuple[str, int]] = set()

    @property
    def mission_started(self) -> bool:
        return self._session is not None

    @property
    def reference_path(self) -> tuple[RuntimePose, ...] | None:
        """Return the resolved immutable path for integration diagnostics."""

        session = self._session
        return None if session is None else session.mission.reference_path

    @property
    def diagnostics(self) -> RuntimeDiagnostics:
        """Return a small read-only summary without exposing mutable R7 objects."""

        session = self._session
        if session is None:
            return RuntimeDiagnostics(
                mission_id=None,
                next_control_tick=None,
                motion_state=None,
                stop_epoch=None,
                predictor_status=None,
                predictor_history_counts=(),
                last_event_was_no_frame=None,
                controller_name=None,
                native_dwb_active=None,
            )
        prediction = session.last_prediction
        return RuntimeDiagnostics(
            mission_id=session.mission.mission_id,
            next_control_tick=session.next_control_tick,
            motion_state=(
                None if session.pipeline is None else session.gate.motion_state
            ),
            stop_epoch=None if session.pipeline is None else session.gate.stop_epoch,
            predictor_status=None if prediction is None else prediction.status.value,
            predictor_history_counts=(
                () if prediction is None else prediction.history_counts
            ),
            last_event_was_no_frame=(
                None
                if session.last_observation_snapshot is None
                else session.last_observation_snapshot.last_event_was_no_frame
            ),
            controller_name=session.controller.name,
            native_dwb_active=(
                session.controller.native_full_core_used
                if isinstance(session.controller, PersistentSourceDerivedDwbController)
                else None
            ),
        )

    def start_mission(self, mission: RuntimeMission) -> None:
        """Prepare one known-map mission without creating an HTTP/service layer.

        The actual pipeline begins only after the first fresh observation with a
        usable prediction arrives.  Until then :meth:`step` returns a zero
        HOLDING command.  This avoids treating the initial camera-latency gap or
        a directional-prediction warm-up as permission to move.
        """

        if not isinstance(mission, RuntimeMission):
            raise TypeError("mission must be a RuntimeMission")
        if self._session is not None:
            raise RuntimeStateError("reset is required before starting another mission")
        mission_key = (mission.mission_id, mission.mission_revision)
        if mission_key in self._started_mission_keys:
            raise RuntimeStateError(
                "a reset runtime requires a new mission id or mission revision; "
                "it cannot reuse a stopped mission's reference or authority"
            )
        grid_snapshot = build_runtime_grid_snapshot(mission)
        if mission.reference_path is None:
            mission = replace(
                mission,
                reference_path=plan_runtime_reference_path(
                    mission,
                    grid_snapshot=grid_snapshot,
                    planner_kind=self.config.global_planner_kind,
                ),
            )
        controller = self._build_controller()
        profile = observation_profile_for(self.config.observation_profile)
        source = build_observation_source(mission)
        self._session = _RuntimeSession(
            mission=mission,
            grid_snapshot=grid_snapshot,
            validator=DynamicObservationValidator(source, profile),
            predictor=DirectionalActorPredictor(),
            gate=DynamicSafetyGate(),
            controller=controller,
        )
        self._started_mission_keys.add(mission_key)

    def step(self, value: RuntimeStepInput) -> RuntimeCommand:
        """Consume one exact 20 Hz tick and return the shared-gate command."""

        if not isinstance(value, RuntimeStepInput):
            raise TypeError("value must be a RuntimeStepInput")
        session = self._require_session()
        try:
            robot_state = to_robot_state(value.robot)
        except (TypeError, ValueError) as error:
            return self._fail_closed(
                session,
                value,
                f"runtime_robot_input_invalid:{type(error).__name__}",
            )
        # This is the newest trustworthy chassis state for every later
        # fail-closed branch of this tick, including a sequence mismatch.
        session.last_robot_state = robot_state
        if session.blocked_reason is not None:
            return self._continue_blocked_stop(
                session,
                value,
                robot_state,
                session.blocked_reason,
            )
        if value.control_tick != session.next_control_tick:
            # A skipped/reordered tick is still a safety event at the newest
            # trustworthy robot pose.  Do not calculate braking from the
            # previous tick's pose/twist when the caller supplied a valid
            # current state.
            session.blocked_reason = "runtime_control_tick_mismatch"
            return self._fail_closed(
                session,
                value,
                session.blocked_reason,
            )
        control_time_s = value.control_tick * DYNAMIC_CONTROL_PERIOD_S

        snapshot = self._consume_observation(session, value, control_time_s)
        prediction = session.predictor.update(snapshot)
        session.last_observation_snapshot = snapshot
        session.last_prediction = prediction

        if session.pipeline is None:
            if not snapshot.usable or prediction.hold_required or prediction.prediction_set is None:
                session.next_control_tick += 1
                return RuntimeCommand(
                    linear_mps=0.0,
                    angular_radps=0.0,
                    motion_state=DynamicMotionState.HOLDING,
                    stop_reason=_startup_hold_reason(snapshot, prediction),
                    control_tick=value.control_tick,
                    stop_epoch=0,
                    failure_reasons=tuple(reason.value for reason in snapshot.failures),
                    observation_status=snapshot.availability.value,
                    prediction_status=prediction.status.value,
                )
            try:
                self._start_pipeline(session, robot_state, value.control_tick)
            except RuntimeReferenceError as error:
                session.next_control_tick += 1
                return RuntimeCommand(
                    linear_mps=0.0,
                    angular_radps=0.0,
                    motion_state=DynamicMotionState.HOLDING,
                    stop_reason=str(error),
                    control_tick=value.control_tick,
                    stop_epoch=0,
                    failure_reasons=("runtime_reference_unavailable",),
                    observation_status=snapshot.availability.value,
                    prediction_status=prediction.status.value,
                )

        assert session.pipeline is not None
        try:
            resume_authorization = to_resume_authorization(value.resume_authorization)
        except (TypeError, ValueError) as error:
            return self._fail_closed(
                session,
                value,
                f"runtime_resume_authorization_invalid:{type(error).__name__}",
            )
        session.pipeline.synchronize_external_robot_state(robot_state)
        grid_snapshot = _grid_for_snapshot(session.grid_snapshot, snapshot)
        pipeline_step = session.pipeline.step(
            observation_snapshot=snapshot,
            prediction_set=prediction.prediction_set,
            computation_time_s=None,
            observation_safe=not prediction.hold_required,
            path_still_valid=value.path_still_valid,
            local_safety_recheck_passed=value.local_safety_recheck_passed,
            resume_authorization=resume_authorization,
            grid_snapshot=grid_snapshot,
            mission_cancelled=value.mission_cancelled,
        )
        session.next_control_tick = session.pipeline.tick_id
        decision = pipeline_step.safety_decision
        return RuntimeCommand(
            linear_mps=decision.command.linear,
            angular_radps=decision.command.angular,
            motion_state=decision.motion_state,
            stop_reason=(
                None if decision.primary_hold_reason is None else decision.primary_hold_reason.value
            ),
            control_tick=decision.tick_id,
            stop_epoch=decision.stop_epoch,
            failure_reasons=decision.failure_reasons,
            observation_status=snapshot.availability.value,
            prediction_status=prediction.status.value,
        )

    def reset(self) -> None:
        """Discard only a stopped/completed mission; never mint restart authority."""

        session = self._session
        if session is None:
            return
        requires_confirmed_stop = session.pipeline is not None or session.prestart_safety_active
        if requires_confirmed_stop and session.gate.motion_state not in {
            DynamicMotionState.HOLDING,
            DynamicMotionState.COMPLETED,
        }:
            raise RuntimeStateError(
                "reset is blocked until the shared gate confirms an actual stop"
            )
        self._session = None

    def _build_controller(self) -> PersistentSourceDerivedDwbController | PersistentRppController:
        if self.config.controller_kind is RuntimeControllerKind.RPP:
            return PersistentRppController()
        if self.config.require_native_dwb and not (
            CPP_DWB_FULL_CORE_AVAILABLE and CPP_DWB_SAFETY_CORE_AVAILABLE
        ):
            raise RuntimeStateError(
                "native DWB libraries are required; install/build both R7 native cores "
                "or explicitly use the research-only Python fallback configuration"
            )
        return PersistentSourceDerivedDwbController(
            use_cpp_full_core=CPP_DWB_FULL_CORE_AVAILABLE,
            use_cpp_safety_core=CPP_DWB_SAFETY_CORE_AVAILABLE,
        )

    def _consume_observation(
        self,
        session: _RuntimeSession,
        value: RuntimeStepInput,
        control_time_s: float,
    ) -> DynamicObservationSnapshot:
        if value.observation is None:
            profile = session.validator.profile
            sequence = session.next_observation_sequence
            due_at_s = sequence * profile.observation_period_s + profile.latency_s
            # A None value on the 20 Hz inter-tick is normal. A None value at
            # the next scheduled 10 Hz delivery is an explicit camera/dropout
            # event and must reset existing predictor/gate evidence.
            if control_time_s >= due_at_s - 1e-12:
                session.validator.record_no_frame(
                    sequence=sequence,
                    delivery_time_s=due_at_s,
                )
                session.next_observation_sequence += 1
            return session.validator.snapshot(control_time_s=control_time_s)
        try:
            frame = to_observation_frame(
                value.observation,
                source=session.validator.source,
                profile=session.validator.profile,
            )
        except (TypeError, ValueError, RuntimeAdapterError):
            return _invalid_snapshot(session.last_observation_snapshot)
        result = session.validator.accept(frame, received_at_s=control_time_s)
        if result.accepted:
            session.next_observation_sequence = value.observation.sequence + 1
        return session.validator.snapshot(control_time_s=control_time_s)

    def _start_pipeline(
        self,
        session: _RuntimeSession,
        robot_state: RobotState,
        control_tick: int,
    ) -> None:
        expected_start = session.mission.start_pose
        if not (
            isclose(robot_state.pose.x, expected_start.x_m, rel_tol=0.0, abs_tol=1e-6)
            and isclose(robot_state.pose.y, expected_start.y_m, rel_tol=0.0, abs_tol=1e-6)
        ):
            raise RuntimeReferenceError("runtime_start_pose_changed_before_pipeline_start")
        context, reference, validation = build_runtime_follow_reference(
            session.mission,
            grid_snapshot=session.grid_snapshot,
            valid_from_tick=control_tick,
        )
        session.pipeline = PersistentControllerPipeline(
            controller=session.controller,
            build_context=context,
            full_reference=reference,
            validation=validation,
            initial_robot_state=robot_state,
            gate=session.gate,
            authorization_revision=session.mission.authorization_revision,
            initial_tick=control_tick,
        )

    def _fail_closed(
        self,
        session: _RuntimeSession,
        value: RuntimeStepInput,
        reason: str,
    ) -> RuntimeCommand:
        """Advance an active R7 gate through an explicit invalid-source tick.

        This branch is intentionally narrow.  It handles only facade-level
        conversion/order errors and never catches controller or gate failures as
        if they were successful movement.
        """

        session.blocked_reason = reason
        snapshot = _invalid_snapshot(session.last_observation_snapshot)
        prediction = session.predictor.update(snapshot)
        session.last_observation_snapshot = snapshot
        session.last_prediction = prediction
        if session.pipeline is None:
            session.prestart_safety_active = True
            robot_state = session.last_robot_state
            if robot_state is not None:
                return self._advance_prestart_stop(
                    session,
                    value,
                    robot_state,
                    reason,
                )
            session.next_control_tick += 1
            return RuntimeCommand(
                linear_mps=0.0,
                angular_radps=0.0,
                motion_state=DynamicMotionState.HOLDING,
                stop_reason=reason,
                control_tick=value.control_tick,
                stop_epoch=0,
                failure_reasons=(reason,),
                observation_status=snapshot.availability.value,
                prediction_status=prediction.status.value,
            )
        robot_state = session.last_robot_state
        if robot_state is None:
            raise RuntimeStateError("active runtime has no last known robot state")
        session.pipeline.synchronize_external_robot_state(robot_state)
        pipeline_step = session.pipeline.step(
            observation_snapshot=snapshot,
            prediction_set=None,
            computation_time_s=0.0,
            observation_safe=False,
            path_still_valid=False,
            local_safety_recheck_passed=False,
            grid_snapshot=_grid_for_snapshot(session.grid_snapshot, snapshot),
        )
        session.next_control_tick = session.pipeline.tick_id
        decision = pipeline_step.safety_decision
        return RuntimeCommand(
            linear_mps=decision.command.linear,
            angular_radps=decision.command.angular,
            motion_state=decision.motion_state,
            stop_reason=reason,
            control_tick=decision.tick_id,
            stop_epoch=decision.stop_epoch,
            failure_reasons=tuple((*decision.failure_reasons, reason)),
            observation_status=snapshot.availability.value,
            prediction_status=prediction.status.value,
        )

    def _continue_blocked_stop(
        self,
        session: _RuntimeSession,
        value: RuntimeStepInput,
        robot_state: RobotState,
        reason: str,
    ) -> RuntimeCommand:
        """Keep a faulted mission in the shared gate until it confirms a stop.

        A dropped or reordered control tick has no trustworthy way to replay
        the missing controller transitions. The facade therefore never catches
        up with a valid controller command. It does keep feeding the latest
        chassis state through an invalid-source gate tick so BRAKING can become
        a confirmed HOLDING stop.
        """

        if session.pipeline is None:
            return self._advance_prestart_stop(session, value, robot_state, reason)

        session.pipeline.synchronize_external_robot_state(robot_state)
        snapshot = _invalid_snapshot(session.last_observation_snapshot)
        prediction = session.predictor.update(snapshot)
        session.last_observation_snapshot = snapshot
        session.last_prediction = prediction
        pipeline_step = session.pipeline.step(
            observation_snapshot=snapshot,
            prediction_set=None,
            computation_time_s=0.0,
            observation_safe=False,
            path_still_valid=False,
            local_safety_recheck_passed=False,
            grid_snapshot=_grid_for_snapshot(session.grid_snapshot, snapshot),
        )
        session.next_control_tick = session.pipeline.tick_id
        decision = pipeline_step.safety_decision
        return RuntimeCommand(
            linear_mps=decision.command.linear,
            angular_radps=decision.command.angular,
            motion_state=decision.motion_state,
            stop_reason=reason,
            control_tick=value.control_tick,
            stop_epoch=decision.stop_epoch,
            failure_reasons=tuple((*decision.failure_reasons, reason)),
            observation_status=snapshot.availability.value,
            prediction_status=prediction.status.value,
        )

    def _advance_prestart_stop(
        self,
        session: _RuntimeSession,
        value: RuntimeStepInput,
        robot_state: RobotState,
        reason: str,
    ) -> RuntimeCommand:
        """Use the existing shared gate before a usable controller frame exists.

        There is intentionally no path/controller command here.  A malformed
        startup tick must still decelerate and wait for the gate's physical-stop
        confirmation, including when the reported pose is no longer the planned
        reference start pose.
        """

        snapshot = _invalid_snapshot(session.last_observation_snapshot)
        prediction = session.predictor.update(snapshot)
        session.last_observation_snapshot = snapshot
        session.last_prediction = prediction
        grid_snapshot = _grid_for_snapshot(session.grid_snapshot, snapshot)
        metadata = grid_snapshot.metadata
        internal_tick = session.next_control_tick
        observation_hash = (
            "observation-unavailable"
            if snapshot.frame is None
            else snapshot.frame.content_hash
        )
        proposal = DynamicCommandProposal(
            source_tick_id=internal_tick,
            command=Twist2D(),
            computation_time_s=0.0,
            mission_id=session.mission.mission_id,
            map_id=metadata.map_id,
            map_revision=metadata.map_revision,
            mission_revision=metadata.mission_revision,
            observation_revision=metadata.observation_revision,
            grid_content_hash=metadata.content_hash,
            observation_content_hash=observation_hash,
            controller_requested_stop=True,
        )
        decision = session.gate.step(
            proposal,
            robot_state=robot_state,
            context=DynamicSafetyContext(
                tick_id=internal_tick,
                simulation_time_s=internal_tick * DYNAMIC_CONTROL_PERIOD_S,
                mission_id=session.mission.mission_id,
                authorization_revision=session.mission.authorization_revision,
                grid_snapshot=grid_snapshot,
                observation_snapshot=snapshot,
                prediction_set=None,
                observation_safe=False,
                path_still_valid=False,
                local_safety_recheck_passed=False,
            ),
        )
        session.next_control_tick += 1
        return RuntimeCommand(
            linear_mps=decision.command.linear,
            angular_radps=decision.command.angular,
            motion_state=decision.motion_state,
            stop_reason=reason,
            control_tick=value.control_tick,
            stop_epoch=decision.stop_epoch,
            failure_reasons=tuple((*decision.failure_reasons, reason)),
            observation_status=snapshot.availability.value,
            prediction_status=prediction.status.value,
        )

    def _require_session(self) -> _RuntimeSession:
        if self._session is None:
            raise RuntimeStateError("start_mission must be called before step")
        return self._session


def _invalid_snapshot(
    previous: DynamicObservationSnapshot | None,
) -> DynamicObservationSnapshot:
    return DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.INVALID,
        frame=None if previous is None else previous.frame,
        age_s=None,
        failures=(DynamicObservationValidationReason.CONTENT_HASH_MISMATCH,),
        last_event_was_no_frame=False,
    )


def _grid_for_snapshot(source: GridSnapshot, snapshot: DynamicObservationSnapshot) -> GridSnapshot:
    frame = snapshot.frame
    observation_revision = 0 if frame is None else frame.observation_revision
    return grid_snapshot_for_observation(source, observation_revision=observation_revision)


def _startup_hold_reason(
    snapshot: DynamicObservationSnapshot,
    prediction: DirectionalPredictionResult,
) -> str:
    if snapshot.availability is not DynamicObservationAvailability.FRESH:
        return "runtime_waiting_for_fresh_observation"
    if prediction.hold_required:
        return f"runtime_waiting_for_prediction:{prediction.reason_code}"
    return "runtime_waiting_for_pipeline_start"
