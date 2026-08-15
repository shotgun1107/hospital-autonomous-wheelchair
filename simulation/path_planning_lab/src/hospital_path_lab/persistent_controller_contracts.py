"""R5 persistent reference controller의 immutable 계약과 session guard.

이 모듈은 R4 full reference·sliding window를 한 control tick의 controller 입력과
결박한다. 실제 RPP/DWB 계산, section 실행, shared safety gate와 이동 허가는 수행하지
않는다. 모든 수치와 상태는 Python ``simulation_only`` 연구 범위다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from hashlib import sha256
from math import isclose, isfinite
from re import fullmatch

from hospital_path_lab.contracts import (
    GridSnapshot,
    RobotState,
    TrajectoryPoint,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import DynamicMotionState
from hospital_path_lab.dynamic_directional_prediction import DirectionalPredictionSet
from hospital_path_lab.dynamic_observation import DynamicObservationSnapshot
from hospital_path_lab.dynamic_prediction import ActorPredictionSet
from hospital_path_lab.local_reference_contracts import (
    REFERENCE_SESSION_BINDING_VERSION,
    LocalManeuverReference,
    LocalReferenceWindow,
    ReferenceEvidenceLevel,
    ReferenceLifecycleStatus,
    ReferenceRevisionBinding,
    ReferenceSectionKind,
    evaluate_reference_revision_update,
    reference_revision_binding,
)
from hospital_path_lab.local_reference_window import window_is_exact_slice
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r5b_temporal_authorization import (
    R5BTemporalExecutionAuthorization,
    validate_r5b_temporal_authorization_for_tick,
)
from hospital_path_lab.spatial_oracle_contracts import spatial_grid_content_hash
from hospital_path_lab.vehicle import VehicleProfile

PERSISTENT_REFERENCE_BINDING_SCHEMA_VERSION = "persistent-reference-binding-v1"
PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION = "persistent-controller-input-v1"
PERSISTENT_CONTROLLER_RESULT_SCHEMA_VERSION = "persistent-controller-result-v1"
PERSISTENT_CONTROLLER_CONTRACT_VERSION = "persistent-reference-controller-v1"
PERSISTENT_CONTROL_PERIOD_S = 0.05
PERSISTENT_CONTROLLER_EVIDENCE_LEVEL = ReferenceEvidenceLevel.SPATIAL_ONLY


class PersistentControllerStatus(StrEnum):
    COMMAND_FOUND = "command_found"
    PLANNED_STOP = "planned_stop"
    HOLD_REQUESTED = "hold_requested"
    NO_SAFE_COMMAND = "no_safe_command"
    INVALID_REFERENCE_INPUT = "invalid_reference_input"
    STALE_REFERENCE_INPUT = "stale_reference_input"
    LATE_RESULT = "late_result"
    SECTION_EXECUTION_FAILED = "section_execution_failed"
    COMPLETED = "completed"


class PersistentControllerSessionTransition(StrEnum):
    INITIAL_BIND = "initial_bind"
    DUPLICATE_TICK = "duplicate_tick"
    WINDOW_UNCHANGED = "window_unchanged"
    WINDOW_ADVANCED = "window_advanced"
    SESSION_RESET = "session_reset"
    STATE_PRESERVED = "state_preserved"
    INVALIDATED = "invalidated"
    NONE = "none"


class ReferenceExecutorState(StrEnum):
    UNBOUND = "unbound"
    TRACK_TRANSLATION = "track_translation"
    APPROACH_PLANNED_STOP = "approach_planned_stop"
    CONFIRM_PLANNED_STOP = "confirm_planned_stop"
    ROTATE_IN_PLACE = "rotate_in_place"
    CONFIRM_ROTATION_STOP = "confirm_rotation_stop"
    HOLD_REQUESTED = "hold_requested"
    TERMINAL_STOP = "terminal_stop"
    TERMINAL_DWELL = "terminal_dwell"
    COMPLETED = "completed"
    INVALIDATED = "invalidated"


@dataclass(frozen=True, slots=True)
class PersistentReferenceBinding:
    """현재 controller tick에 전달되는 R4 reference identity 전체."""

    schema_version: str
    candidate_id: str
    reference_session_id: str
    mission_id: str
    stop_epoch: int
    map_id: str
    map_revision: int
    mission_revision: int
    maneuver_revision: int
    path_revision: int
    subgoal_revision: int
    full_reference_hash: str
    window_content_hash: str
    source_window_control_tick: int
    lifecycle: ReferenceLifecycleStatus
    binding_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != PERSISTENT_REFERENCE_BINDING_SCHEMA_VERSION:
            raise ValueError("unsupported persistent reference binding schema")
        for name in ("candidate_id", "reference_session_id"):
            _require_sha256(getattr(self, name), name)
        for name in ("mission_id", "map_id"):
            _require_nonempty(getattr(self, name), name)
        for name in (
            "stop_epoch",
            "map_revision",
            "mission_revision",
            "maneuver_revision",
            "path_revision",
            "subgoal_revision",
            "source_window_control_tick",
        ):
            _require_exact_nonnegative_int(getattr(self, name), name)
        for name in ("full_reference_hash", "window_content_hash"):
            _require_sha256(getattr(self, name), name)
        if not isinstance(self.lifecycle, ReferenceLifecycleStatus):
            raise TypeError("lifecycle must be a ReferenceLifecycleStatus")
        _bind_or_check_hash(self, "binding_content_hash", self.expected_content_hash)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "candidate_id": self.candidate_id,
                "reference_session_id": self.reference_session_id,
                "mission_id": self.mission_id,
                "stop_epoch": self.stop_epoch,
                "map_id": self.map_id,
                "map_revision": self.map_revision,
                "mission_revision": self.mission_revision,
                "maneuver_revision": self.maneuver_revision,
                "path_revision": self.path_revision,
                "subgoal_revision": self.subgoal_revision,
                "full_reference_hash": self.full_reference_hash,
                "window_content_hash": self.window_content_hash,
                "source_window_control_tick": self.source_window_control_tick,
                "lifecycle": self.lifecycle,
            }
        )


def build_persistent_reference_binding(
    reference: LocalManeuverReference,
    window: LocalReferenceWindow,
    *,
    lifecycle: ReferenceLifecycleStatus = ReferenceLifecycleStatus.AVAILABLE,
) -> PersistentReferenceBinding:
    """R4 reference/window 검증 뒤 R5 current-delivery binding을 만든다."""

    revision = reference_revision_binding(reference, window)
    if not isinstance(lifecycle, ReferenceLifecycleStatus):
        raise TypeError("lifecycle must be a ReferenceLifecycleStatus")
    return PersistentReferenceBinding(
        schema_version=PERSISTENT_REFERENCE_BINDING_SCHEMA_VERSION,
        candidate_id=revision.candidate_id,
        reference_session_id=revision.reference_session_id,
        mission_id=revision.mission_id,
        stop_epoch=revision.stop_epoch,
        map_id=reference.map_id,
        map_revision=reference.map_revision,
        mission_revision=reference.mission_revision,
        maneuver_revision=revision.maneuver_revision,
        path_revision=revision.path_revision,
        subgoal_revision=revision.subgoal_revision,
        full_reference_hash=revision.full_reference_hash,
        window_content_hash=revision.window_content_hash,
        source_window_control_tick=window.source_control_tick,
        lifecycle=lifecycle,
    )


@dataclass(frozen=True, slots=True)
class PersistentControllerTickInput:
    """R5 controller가 한 tick에서 볼 수 있는 ground-truth 비포함 snapshot."""

    schema_version: str
    controller_tick: int
    simulation_time_s: float
    full_reference: LocalManeuverReference
    local_window: LocalReferenceWindow
    reference_binding: PersistentReferenceBinding
    robot_state: RobotState
    static_grid_snapshot: GridSnapshot
    validated_observation: DynamicObservationSnapshot
    actor_prediction_set: ActorPredictionSet | DirectionalPredictionSet | None
    vehicle_profile: VehicleProfile
    current_gate_motion_state: DynamicMotionState
    current_gate_stop_epoch: int
    current_resume_authorization_revision: int | None
    temporal_execution_authorization: R5BTemporalExecutionAuthorization | None = None
    tick_input_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION:
            raise ValueError("unsupported persistent controller input schema")
        _require_exact_nonnegative_int(self.controller_tick, "controller_tick")
        _require_finite_nonnegative(self.simulation_time_s, "simulation_time_s")
        expected_time = self.controller_tick * PERSISTENT_CONTROL_PERIOD_S
        if not isclose(self.simulation_time_s, expected_time, rel_tol=0.0, abs_tol=1e-12):
            raise ValueError("simulation_time_s must derive from the 20 Hz controller tick")
        if not isinstance(self.full_reference, LocalManeuverReference):
            raise TypeError("full_reference must be a LocalManeuverReference")
        if not isinstance(self.local_window, LocalReferenceWindow):
            raise TypeError("local_window must be a LocalReferenceWindow")
        if not isinstance(self.reference_binding, PersistentReferenceBinding):
            raise TypeError("reference_binding must be a PersistentReferenceBinding")
        if not isinstance(self.robot_state, RobotState):
            raise TypeError("robot_state must be a RobotState")
        _validate_robot_state(self.robot_state)
        if not isinstance(self.static_grid_snapshot, GridSnapshot):
            raise TypeError("static_grid_snapshot must be a GridSnapshot")
        if not isinstance(self.validated_observation, DynamicObservationSnapshot):
            raise TypeError("validated_observation must be a DynamicObservationSnapshot")
        if self.actor_prediction_set is not None and not isinstance(
            self.actor_prediction_set,
            (ActorPredictionSet, DirectionalPredictionSet),
        ):
            raise TypeError("actor_prediction_set has an unsupported type")
        if not isinstance(self.vehicle_profile, VehicleProfile):
            raise TypeError("vehicle_profile must be a VehicleProfile")
        if not self.vehicle_profile.simulation_only:
            raise ValueError("persistent controller input requires a simulation-only profile")
        if not isinstance(self.current_gate_motion_state, DynamicMotionState):
            raise TypeError("current_gate_motion_state must be a DynamicMotionState")
        _require_exact_nonnegative_int(self.current_gate_stop_epoch, "current_gate_stop_epoch")
        if self.current_resume_authorization_revision is not None:
            _require_exact_nonnegative_int(
                self.current_resume_authorization_revision,
                "current_resume_authorization_revision",
            )
        if self.temporal_execution_authorization is not None and not isinstance(
            self.temporal_execution_authorization,
            R5BTemporalExecutionAuthorization,
        ):
            raise TypeError(
                "temporal_execution_authorization has an unsupported type"
            )
        _validate_reference_input_identity(self)
        _bind_or_check_hash(self, "tick_input_content_hash", self.expected_content_hash)

    @property
    def expected_content_hash(self) -> str:
        payload = {
                "schema_version": self.schema_version,
                "contract_version": PERSISTENT_CONTROLLER_CONTRACT_VERSION,
                "controller_tick": self.controller_tick,
                "simulation_time_s": self.simulation_time_s,
                "full_reference_hash": self.full_reference.reference_content_hash,
                "window_content_hash": self.local_window.window_content_hash,
                "reference_binding_hash": self.reference_binding.binding_content_hash,
                "robot_state": self.robot_state,
                "grid_commitment": _grid_commitment(self.static_grid_snapshot),
                "observation_commitment": _observation_commitment(
                    self.validated_observation
                ),
                "prediction_commitment": _prediction_commitment(
                    self.actor_prediction_set
                ),
                "vehicle_profile_hash": canonical_content_hash(self.vehicle_profile),
                "current_gate_motion_state": self.current_gate_motion_state,
                "current_gate_stop_epoch": self.current_gate_stop_epoch,
                "current_resume_authorization_revision": (
                    self.current_resume_authorization_revision
                ),
            }
        if self.temporal_execution_authorization is not None:
            payload["temporal_execution_authorization_hash"] = (
                self.temporal_execution_authorization.authorization_content_hash
            )
        return canonical_content_hash(payload)


@dataclass(frozen=True, slots=True)
class PersistentReferenceAcceptance:
    """Session guard가 한 immutable tick input을 수용했는지 나타낸다."""

    accepted: bool
    duplicate_tick: bool
    state_reset_required: bool
    transition: PersistentControllerSessionTransition
    reason_code: str
    next_binding: PersistentReferenceBinding | None

    def __post_init__(self) -> None:
        for name in ("accepted", "duplicate_tick", "state_reset_required"):
            if not isinstance(getattr(self, name), bool):
                raise TypeError(f"{name} must be a bool")
        if not isinstance(self.transition, PersistentControllerSessionTransition):
            raise TypeError("transition must be a PersistentControllerSessionTransition")
        _require_nonempty(self.reason_code, "reason_code")
        if self.accepted != (self.next_binding is not None):
            raise ValueError("accepted decision must carry the next binding")
        if self.duplicate_tick and not self.accepted:
            raise ValueError("duplicate tick must be accepted")
        if self.state_reset_required and not self.accepted:
            raise ValueError("rejected input cannot require a state reset")


class PersistentReferenceSessionGuard:
    """R4 revision 규칙에 현재 controller tick idempotence를 더한 guard."""

    def __init__(self) -> None:
        self._current_binding: PersistentReferenceBinding | None = None
        self._last_tick: int | None = None
        self._last_tick_input_hash: str | None = None

    @property
    def current_binding(self) -> PersistentReferenceBinding | None:
        return self._current_binding

    def evaluate(self, tick_input: PersistentControllerTickInput) -> PersistentReferenceAcceptance:
        if not isinstance(tick_input, PersistentControllerTickInput):
            raise TypeError("tick_input must be a PersistentControllerTickInput")
        tick = tick_input.controller_tick
        digest = tick_input.tick_input_content_hash
        if self._last_tick is not None:
            if tick < self._last_tick:
                return _rejected_acceptance("controller_tick_regression")
            if tick == self._last_tick:
                if digest == self._last_tick_input_hash:
                    assert self._current_binding is not None
                    return PersistentReferenceAcceptance(
                        accepted=True,
                        duplicate_tick=True,
                        state_reset_required=False,
                        transition=PersistentControllerSessionTransition.DUPLICATE_TICK,
                        reason_code="duplicate_tick_input",
                        next_binding=self._current_binding,
                    )
                return _rejected_acceptance("same_tick_input_changed")

        current_revision = (
            None
            if self._current_binding is None
            else _r4_revision_binding(self._current_binding)
        )
        incoming_revision = _r4_revision_binding(tick_input.reference_binding)
        revision_decision = evaluate_reference_revision_update(
            current_revision,
            incoming_revision,
        )
        if not revision_decision.accepted:
            return _rejected_acceptance(revision_decision.reason_code)

        if self._current_binding is None:
            transition = PersistentControllerSessionTransition.INITIAL_BIND
            reason = "initial_binding_accepted"
            reset = True
        elif revision_decision.duplicate:
            transition = PersistentControllerSessionTransition.WINDOW_UNCHANGED
            reason = "same_window_current_delivery"
            reset = False
        elif revision_decision.reason_code == "subgoal_revision_advanced":
            transition = PersistentControllerSessionTransition.WINDOW_ADVANCED
            reason = revision_decision.reason_code
            reset = False
        else:
            transition = PersistentControllerSessionTransition.SESSION_RESET
            reason = revision_decision.reason_code
            reset = True

        accepted = PersistentReferenceAcceptance(
            accepted=True,
            duplicate_tick=False,
            state_reset_required=reset,
            transition=transition,
            reason_code=reason,
            next_binding=tick_input.reference_binding,
        )
        self._current_binding = tick_input.reference_binding
        self._last_tick = tick
        self._last_tick_input_hash = digest
        return accepted


@dataclass(frozen=True, slots=True)
class PersistentControllerResult:
    """R5 adapter가 shared gate 앞에서 반환하는 reference-bound 결과."""

    schema_version: str
    controller_name: str
    source_controller_tick: int
    status: PersistentControllerStatus
    requested_twist: Twist2D
    predicted_trajectory: tuple[TrajectoryPoint, ...]
    failure_reason: str | None
    decision_trace: tuple[str, ...]
    reference_binding_echo: PersistentReferenceBinding
    tick_input_content_hash: str
    controller_session_transition: PersistentControllerSessionTransition
    executor_state: ReferenceExecutorState
    active_section_index: int | None
    active_section_kind: ReferenceSectionKind | None
    tracking_error_m: float | None
    candidate_diagnostics: tuple[str, ...]
    planned_section_stop: bool
    controller_requested_protective_stop: bool
    no_safe_candidate: bool
    elapsed_nonqualification_ns: int
    semantic_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != PERSISTENT_CONTROLLER_RESULT_SCHEMA_VERSION:
            raise ValueError("unsupported persistent controller result schema")
        _require_nonempty(self.controller_name, "controller_name")
        _require_exact_nonnegative_int(self.source_controller_tick, "source_controller_tick")
        if not isinstance(self.status, PersistentControllerStatus):
            raise TypeError("status must be a PersistentControllerStatus")
        _validate_twist(self.requested_twist, "requested_twist")
        trajectory = tuple(self.predicted_trajectory)
        _validate_trajectory(trajectory)
        object.__setattr__(self, "predicted_trajectory", trajectory)
        if self.failure_reason is not None:
            _require_nonempty(self.failure_reason, "failure_reason")
        object.__setattr__(self, "decision_trace", tuple(self.decision_trace))
        if not isinstance(self.reference_binding_echo, PersistentReferenceBinding):
            raise TypeError("reference_binding_echo must be a PersistentReferenceBinding")
        if self.reference_binding_echo.source_window_control_tick != self.source_controller_tick:
            raise ValueError("result tick must match the echoed current-delivery binding")
        _require_sha256(self.tick_input_content_hash, "tick_input_content_hash")
        if not isinstance(
            self.controller_session_transition,
            PersistentControllerSessionTransition,
        ):
            raise TypeError("controller_session_transition has an unsupported type")
        if not isinstance(self.executor_state, ReferenceExecutorState):
            raise TypeError("executor_state must be a ReferenceExecutorState")
        if self.active_section_index is not None:
            _require_exact_nonnegative_int(self.active_section_index, "active_section_index")
        if self.active_section_kind is not None and not isinstance(
            self.active_section_kind,
            ReferenceSectionKind,
        ):
            raise TypeError("active_section_kind must be a ReferenceSectionKind")
        if (self.active_section_index is None) != (self.active_section_kind is None):
            raise ValueError("active section index and kind must be present together")
        if self.tracking_error_m is not None:
            _require_finite_nonnegative(self.tracking_error_m, "tracking_error_m")
        diagnostics = tuple(self.candidate_diagnostics)
        if tuple(sorted(set(diagnostics))) != diagnostics:
            raise ValueError("candidate_diagnostics must be sorted and unique")
        object.__setattr__(self, "candidate_diagnostics", diagnostics)
        for name in (
            "planned_section_stop",
            "controller_requested_protective_stop",
            "no_safe_candidate",
        ):
            if not isinstance(getattr(self, name), bool):
                raise TypeError(f"{name} must be a bool")
        _require_exact_nonnegative_int(
            self.elapsed_nonqualification_ns,
            "elapsed_nonqualification_ns",
        )
        self._validate_status_semantics()
        _bind_or_check_hash(self, "semantic_content_hash", self.expected_semantic_hash)

    def _validate_status_semantics(self) -> None:
        zero_required = self.status in {
            PersistentControllerStatus.HOLD_REQUESTED,
            PersistentControllerStatus.NO_SAFE_COMMAND,
            PersistentControllerStatus.INVALID_REFERENCE_INPUT,
            PersistentControllerStatus.STALE_REFERENCE_INPUT,
            PersistentControllerStatus.LATE_RESULT,
            PersistentControllerStatus.SECTION_EXECUTION_FAILED,
            PersistentControllerStatus.COMPLETED,
        }
        if zero_required and self.requested_twist != Twist2D():
            raise ValueError("non-motion result status requires a zero requested twist")
        if self.status is PersistentControllerStatus.PLANNED_STOP:
            if not self.planned_section_stop or self.controller_requested_protective_stop:
                raise ValueError("planned stop must remain separate from protective stop")
        elif self.planned_section_stop:
            raise ValueError("planned_section_stop is only valid for PLANNED_STOP")
        if self.status is PersistentControllerStatus.HOLD_REQUESTED and not (
            self.controller_requested_protective_stop
        ):
            raise ValueError("HOLD_REQUESTED must request a protective stop")
        protective_required = self.status in {
            PersistentControllerStatus.HOLD_REQUESTED,
            PersistentControllerStatus.NO_SAFE_COMMAND,
            PersistentControllerStatus.INVALID_REFERENCE_INPUT,
            PersistentControllerStatus.STALE_REFERENCE_INPUT,
            PersistentControllerStatus.LATE_RESULT,
            PersistentControllerStatus.SECTION_EXECUTION_FAILED,
        }
        if protective_required and not self.controller_requested_protective_stop:
            raise ValueError("failure or hold status must request a protective stop")
        if self.status is PersistentControllerStatus.NO_SAFE_COMMAND and not self.no_safe_candidate:
            raise ValueError("NO_SAFE_COMMAND must set no_safe_candidate")
        if self.no_safe_candidate and self.status is not PersistentControllerStatus.NO_SAFE_COMMAND:
            raise ValueError("no_safe_candidate is only valid for NO_SAFE_COMMAND")
        if self.status is PersistentControllerStatus.COMMAND_FOUND and (
            self.planned_section_stop
            or self.controller_requested_protective_stop
            or self.no_safe_candidate
        ):
            raise ValueError("COMMAND_FOUND cannot carry stop flags")
        successful = self.status in {
            PersistentControllerStatus.COMMAND_FOUND,
            PersistentControllerStatus.PLANNED_STOP,
            PersistentControllerStatus.COMPLETED,
        }
        if successful and self.failure_reason is not None:
            raise ValueError("successful result status cannot carry a failure_reason")
        if not successful and self.failure_reason is None:
            raise ValueError("non-success result status requires a failure_reason")

    @property
    def expected_semantic_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "contract_version": PERSISTENT_CONTROLLER_CONTRACT_VERSION,
                "controller_name": self.controller_name,
                "source_controller_tick": self.source_controller_tick,
                "status": self.status,
                "requested_twist": self.requested_twist,
                "predicted_trajectory": self.predicted_trajectory,
                "failure_reason": self.failure_reason,
                "decision_trace": self.decision_trace,
                "reference_binding_echo": self.reference_binding_echo,
                "tick_input_content_hash": self.tick_input_content_hash,
                "controller_session_transition": self.controller_session_transition,
                "executor_state": self.executor_state,
                "active_section_index": self.active_section_index,
                "active_section_kind": self.active_section_kind,
                "tracking_error_m": self.tracking_error_m,
                "candidate_diagnostics": self.candidate_diagnostics,
                "planned_section_stop": self.planned_section_stop,
                "controller_requested_protective_stop": (
                    self.controller_requested_protective_stop
                ),
                "no_safe_candidate": self.no_safe_candidate,
            }
        )


def _validate_reference_input_identity(value: PersistentControllerTickInput) -> None:
    reference = value.full_reference
    window = value.local_window
    binding = value.reference_binding
    if reference.reference_content_hash != reference.expected_content_hash:
        raise ValueError("full reference semantic hash mismatch")
    if window.window_content_hash != window.expected_content_hash:
        raise ValueError("local window semantic hash mismatch")
    if not window_is_exact_slice(reference, window):
        raise ValueError("local window is not an exact full-reference slice")
    if reference.evidence_level is PERSISTENT_CONTROLLER_EVIDENCE_LEVEL:
        if value.temporal_execution_authorization is not None:
            raise ValueError("R5-A spatial input cannot claim temporal authorization")
    elif reference.evidence_level is ReferenceEvidenceLevel.GROUND_TRUTH_TEMPORAL:
        authorization = value.temporal_execution_authorization
        if authorization is None:
            raise ValueError("R5-B temporal input requires tick-bound authorization")
        prediction = value.actor_prediction_set
        if not isinstance(prediction, DirectionalPredictionSet):
            raise ValueError("R5-B temporal input requires directional prediction")
        validate_r5b_temporal_authorization_for_tick(
            authorization,
            reference=reference,
            observation_snapshot=value.validated_observation,
            prediction_set=prediction,
            controller_tick=value.controller_tick,
            simulation_time_s=value.simulation_time_s,
            gate_motion_state=value.current_gate_motion_state,
            gate_stop_epoch=value.current_gate_stop_epoch,
            resume_authorization_revision=value.current_resume_authorization_revision,
        )
    else:
        raise ValueError("persistent controller input rejects observation-integrated evidence")
    expected = build_persistent_reference_binding(
        reference,
        window,
        lifecycle=binding.lifecycle,
    )
    if binding != expected:
        raise ValueError("persistent reference binding does not match reference and window")
    if binding.lifecycle is not ReferenceLifecycleStatus.AVAILABLE:
        raise ValueError("persistent controller input requires an AVAILABLE reference")
    if binding.source_window_control_tick != value.controller_tick:
        raise ValueError("window must be freshly delivered for the current controller tick")
    validity = reference.validity
    if value.controller_tick < validity.valid_from_control_tick or (
        validity.valid_until_control_tick is not None
        and value.controller_tick > validity.valid_until_control_tick
    ):
        raise ValueError("reference is outside its valid control-tick interval")
    if value.current_gate_stop_epoch != binding.stop_epoch:
        raise ValueError("current gate stop epoch does not match the reference")
    metadata = value.static_grid_snapshot.metadata
    if (
        metadata.map_id,
        metadata.map_revision,
        metadata.mission_revision,
    ) != (
        binding.map_id,
        binding.map_revision,
        binding.mission_revision,
    ):
        raise ValueError("grid provenance does not match the persistent reference binding")
    if spatial_grid_content_hash(value.static_grid_snapshot.grid) != reference.grid_content_hash:
        raise ValueError("grid content does not match the full reference")
    if canonical_content_hash(value.vehicle_profile) != reference.vehicle_profile_hash:
        raise ValueError("vehicle profile does not match the full reference")


def _r4_revision_binding(binding: PersistentReferenceBinding) -> ReferenceRevisionBinding:
    return ReferenceRevisionBinding(
        binding_version=REFERENCE_SESSION_BINDING_VERSION,
        mission_id=binding.mission_id,
        stop_epoch=binding.stop_epoch,
        maneuver_revision=binding.maneuver_revision,
        path_revision=binding.path_revision,
        subgoal_revision=binding.subgoal_revision,
        candidate_id=binding.candidate_id,
        reference_session_id=binding.reference_session_id,
        full_reference_hash=binding.full_reference_hash,
        window_content_hash=binding.window_content_hash,
        lifecycle=binding.lifecycle,
    )


def _grid_commitment(snapshot: GridSnapshot) -> str:
    grid = snapshot.grid
    return canonical_content_hash(
        {
            "metadata": snapshot.metadata,
            "shape": tuple(int(value) for value in grid.occupancy.shape),
            "occupancy_sha256": sha256(grid.occupancy.tobytes()).hexdigest(),
            "resolution_m": grid.resolution_m,
            "origin_x_m": grid.origin_x_m,
            "origin_y_m": grid.origin_y_m,
            "forbidden_cells": tuple(sorted(snapshot.forbidden_cells)),
            "input_valid": snapshot.input_valid,
        }
    )


def _observation_commitment(snapshot: DynamicObservationSnapshot) -> str:
    return canonical_content_hash(
        {
            "availability": snapshot.availability,
            "frame_content_hash": (
                None if snapshot.frame is None else snapshot.frame.content_hash
            ),
            "age_s": snapshot.age_s,
            "failures": snapshot.failures,
            "last_event_was_no_frame": snapshot.last_event_was_no_frame,
        }
    )


def _prediction_commitment(
    prediction: ActorPredictionSet | DirectionalPredictionSet | None,
) -> str | None:
    if prediction is None:
        return None
    if isinstance(prediction, DirectionalPredictionSet):
        return prediction.content_hash
    if isinstance(prediction, ActorPredictionSet):
        return canonical_content_hash(prediction)
    raise TypeError("unsupported Actor prediction type")


def _validate_robot_state(state: RobotState) -> None:
    values = (
        state.pose.x,
        state.pose.y,
        state.pose.yaw,
        state.twist.linear,
        state.twist.angular,
    )
    if not all(isfinite(value) for value in values):
        raise ValueError("robot_state must contain finite values")


def _validate_twist(twist: Twist2D, name: str) -> None:
    if not isinstance(twist, Twist2D):
        raise TypeError(f"{name} must be a Twist2D")
    if not all(isfinite(value) for value in (twist.linear, twist.angular)):
        raise ValueError(f"{name} must contain finite values")


def _validate_trajectory(trajectory: tuple[TrajectoryPoint, ...]) -> None:
    previous = -1.0
    for point in trajectory:
        if not isinstance(point, TrajectoryPoint):
            raise TypeError("predicted_trajectory must contain TrajectoryPoint values")
        values = (
            point.time_s,
            point.pose.x,
            point.pose.y,
            point.pose.yaw,
            point.twist.linear,
            point.twist.angular,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("predicted_trajectory must contain finite values")
        if point.time_s <= previous:
            raise ValueError("predicted_trajectory time must strictly increase")
        previous = point.time_s
    if trajectory and trajectory[0].time_s != 0.0:
        raise ValueError("predicted_trajectory must start at time zero")


def _rejected_acceptance(reason: str) -> PersistentReferenceAcceptance:
    return PersistentReferenceAcceptance(
        accepted=False,
        duplicate_tick=False,
        state_reset_required=False,
        transition=PersistentControllerSessionTransition.INVALIDATED,
        reason_code=reason,
        next_binding=None,
    )


def _bind_or_check_hash(value: object, field_name: str, expected: str) -> None:
    current = getattr(value, field_name)
    if current:
        _require_sha256(current, field_name)
        if current != expected:
            raise ValueError(f"{field_name} mismatch")
    else:
        object.__setattr__(value, field_name, expected)


def _require_sha256(value: str, name: str) -> None:
    if not isinstance(value, str) or fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{name} must be a lowercase SHA-256")


def _require_nonempty(value: str, name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must not be empty")


def _require_exact_nonnegative_int(value: int, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative exact integer")


def _require_finite_nonnegative(value: float, name: str) -> None:
    if not isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
