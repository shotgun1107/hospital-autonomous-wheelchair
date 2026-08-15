"""R5-B Ideal causal stream의 tick-bound temporal 실행 허가."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from math import isclose
from re import fullmatch

from hospital_path_lab.dynamic_contracts import DynamicMotionState
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalPredictionResult,
    DirectionalPredictionSet,
    DirectionalPredictionStatus,
    validate_directional_prediction_set,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationFrameKind,
    DynamicObservationSnapshot,
    dynamic_observation_content_hash,
)
from hospital_path_lab.local_reference_contracts import (
    LocalManeuverReference,
    ReferenceEvidenceLevel,
    TemporalReferenceEvidence,
    TemporalReferenceGeometryEvidence,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r5b_temporal_evidence import R5B_CAUSAL_RELEASE_TICK

R5B_TEMPORAL_AUTHORIZATION_SCHEMA_VERSION = "r5b-temporal-execution-authorization-v1"
R5B_TEMPORAL_AUTHORIZATION_ISSUER_VERSION = "r5b-temporal-authorization-issuer-v1"
R5B_CONTROL_PERIOD_S = 0.05


class R5BTemporalAuthorizationPhase(StrEnum):
    INITIAL_RELEASE = "initial_release"
    CONTINUATION = "continuation"


@dataclass(frozen=True, slots=True)
class R5BTemporalExecutionAuthorization:
    schema_version: str
    issuer_version: str
    phase: R5BTemporalAuthorizationPhase
    reference_content_hash: str
    temporal_evidence_hash: str
    temporal_geometry_hash: str
    mission_id: str
    stop_epoch: int
    map_id: str
    map_revision: int
    mission_revision: int
    reference_session_id: str
    initial_release_tick: int
    controller_tick: int
    simulation_time_s: float
    gate_motion_state: DynamicMotionState
    resume_authorization_revision: int | None
    observation_content_hash: str
    observation_revision: int
    observation_sequence: int
    prediction_content_hash: str
    prediction_model_version: str
    target_actor_binding_id: str
    target_track_id: str
    prior_authorization_hash: str | None
    local_safety_recheck_passed: bool
    actual_stop_confirmed_for_release: bool
    authorization_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != R5B_TEMPORAL_AUTHORIZATION_SCHEMA_VERSION:
            raise ValueError("unsupported R5-B temporal authorization schema")
        if self.issuer_version != R5B_TEMPORAL_AUTHORIZATION_ISSUER_VERSION:
            raise ValueError("unsupported R5-B temporal authorization issuer")
        if not isinstance(self.phase, R5BTemporalAuthorizationPhase):
            raise TypeError("phase must be an R5BTemporalAuthorizationPhase")
        for name in (
            "reference_content_hash",
            "temporal_evidence_hash",
            "temporal_geometry_hash",
            "reference_session_id",
            "observation_content_hash",
            "prediction_content_hash",
        ):
            _require_sha256(getattr(self, name), name)
        if self.prior_authorization_hash is not None:
            _require_sha256(self.prior_authorization_hash, "prior_authorization_hash")
        for name in ("mission_id", "map_id", "prediction_model_version"):
            _require_nonempty(getattr(self, name), name)
        for name in (
            "stop_epoch",
            "map_revision",
            "mission_revision",
            "initial_release_tick",
            "controller_tick",
            "observation_revision",
            "observation_sequence",
        ):
            _require_nonnegative_int(getattr(self, name), name)
        if self.resume_authorization_revision is not None:
            _require_nonnegative_int(
                self.resume_authorization_revision,
                "resume_authorization_revision",
            )
        if not isclose(
            self.simulation_time_s,
            self.controller_tick * R5B_CONTROL_PERIOD_S,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("authorization time must derive from the 20 Hz tick")
        if not isinstance(self.gate_motion_state, DynamicMotionState):
            raise TypeError("gate_motion_state must be a DynamicMotionState")
        for name in ("target_actor_binding_id", "target_track_id"):
            _require_nonempty(getattr(self, name), name)
        if self.local_safety_recheck_passed is not True:
            raise ValueError("R5-B authorization requires a passed local safety recheck")
        if not isinstance(self.actual_stop_confirmed_for_release, bool):
            raise TypeError("actual_stop_confirmed_for_release must be bool")
        if self.phase is R5BTemporalAuthorizationPhase.INITIAL_RELEASE:
            if (
                self.controller_tick != self.initial_release_tick
                or self.gate_motion_state is not DynamicMotionState.HOLDING
                or self.resume_authorization_revision is None
                or not self.actual_stop_confirmed_for_release
                or self.prior_authorization_hash is not None
            ):
                raise ValueError("initial R5-B release authorization is incomplete")
        elif (
            self.controller_tick <= self.initial_release_tick
            or self.gate_motion_state is not DynamicMotionState.MOVING
            or self.resume_authorization_revision is not None
            or self.actual_stop_confirmed_for_release
            or self.prior_authorization_hash is None
        ):
            raise ValueError("R5-B continuation authorization is incomplete")
        expected = self.expected_content_hash
        if self.authorization_content_hash:
            _require_sha256(self.authorization_content_hash, "authorization_content_hash")
            if self.authorization_content_hash != expected:
                raise ValueError("R5-B temporal authorization hash mismatch")
        else:
            object.__setattr__(self, "authorization_content_hash", expected)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                name: getattr(self, name)
                for name in self.__dataclass_fields__
                if name != "authorization_content_hash"
            }
        )


class R5BTemporalAuthorizationIssuer:
    """한 reference session에서 최초 release와 연속 tick 허가를 발행한다."""

    def __init__(self) -> None:
        self._last: R5BTemporalExecutionAuthorization | None = None

    @property
    def last_authorization(self) -> R5BTemporalExecutionAuthorization | None:
        return self._last

    def invalidate(self) -> None:
        self._last = None

    def issue(
        self,
        *,
        reference: LocalManeuverReference,
        temporal_evidence: TemporalReferenceEvidence,
        temporal_geometry: TemporalReferenceGeometryEvidence,
        observation_snapshot: DynamicObservationSnapshot,
        prediction_result: DirectionalPredictionResult,
        controller_tick: int,
        simulation_time_s: float,
        gate_motion_state: DynamicMotionState,
        gate_stop_epoch: int,
        resume_authorization_revision: int | None,
        actual_stop_confirmed: bool,
        local_safety_recheck_passed: bool,
    ) -> R5BTemporalExecutionAuthorization:
        _validate_sources(reference, temporal_evidence, temporal_geometry)
        _require_nonnegative_int(controller_tick, "controller_tick")
        _require_nonnegative_int(gate_stop_epoch, "gate_stop_epoch")
        if gate_stop_epoch != reference.stop_epoch:
            raise ValueError("R5-B authorization stop epoch does not match the reference")
        if controller_tick < R5B_CAUSAL_RELEASE_TICK:
            raise ValueError("R5-B authorization cannot be issued before causal release")
        if not isclose(
            simulation_time_s,
            controller_tick * R5B_CONTROL_PERIOD_S,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("R5-B authorization time does not match its tick")

        frame, prediction = _validated_live_inputs(
            reference,
            temporal_evidence,
            observation_snapshot,
            prediction_result,
        )
        if self._last is None:
            if controller_tick != R5B_CAUSAL_RELEASE_TICK:
                raise ValueError("R5-B first authorization must occur at frozen tick 40")
            phase = R5BTemporalAuthorizationPhase.INITIAL_RELEASE
            prior_hash = None
            initial_release_tick = controller_tick
        else:
            previous = self._last
            if controller_tick != previous.controller_tick + 1:
                raise ValueError("R5-B continuation authorization tick must increase by one")
            if (
                previous.reference_content_hash != reference.reference_content_hash
                or previous.stop_epoch != reference.stop_epoch
                or previous.reference_session_id != reference.reference_session_id
            ):
                raise ValueError("R5-B authorization session changed")
            phase = R5BTemporalAuthorizationPhase.CONTINUATION
            prior_hash = previous.authorization_content_hash
            initial_release_tick = previous.initial_release_tick

        tube = next(
            tube
            for tube in prediction.tubes
            if tube.actor_binding_id == temporal_evidence.target_actor_binding_ids[0]
        )
        if self._last is not None and (
            self._last.target_actor_binding_id != tube.actor_binding_id
            or self._last.target_track_id != tube.track_id
        ):
            raise ValueError("R5-B continuation target track identity changed")
        authorization = R5BTemporalExecutionAuthorization(
            schema_version=R5B_TEMPORAL_AUTHORIZATION_SCHEMA_VERSION,
            issuer_version=R5B_TEMPORAL_AUTHORIZATION_ISSUER_VERSION,
            phase=phase,
            reference_content_hash=reference.reference_content_hash,
            temporal_evidence_hash=temporal_evidence.evidence_content_hash,
            temporal_geometry_hash=temporal_geometry.geometry_content_hash,
            mission_id=reference.mission_id,
            stop_epoch=reference.stop_epoch,
            map_id=reference.map_id,
            map_revision=reference.map_revision,
            mission_revision=reference.mission_revision,
            reference_session_id=reference.reference_session_id,
            initial_release_tick=initial_release_tick,
            controller_tick=controller_tick,
            simulation_time_s=simulation_time_s,
            gate_motion_state=gate_motion_state,
            resume_authorization_revision=resume_authorization_revision,
            observation_content_hash=frame.content_hash,
            observation_revision=frame.observation_revision,
            observation_sequence=frame.sequence,
            prediction_content_hash=prediction.content_hash,
            prediction_model_version=prediction.model_version,
            target_actor_binding_id=tube.actor_binding_id,
            target_track_id=tube.track_id,
            prior_authorization_hash=prior_hash,
            local_safety_recheck_passed=local_safety_recheck_passed,
            actual_stop_confirmed_for_release=actual_stop_confirmed,
        )
        self._last = authorization
        return authorization


def validate_r5b_temporal_authorization_for_tick(
    authorization: R5BTemporalExecutionAuthorization,
    *,
    reference: LocalManeuverReference,
    observation_snapshot: DynamicObservationSnapshot,
    prediction_set: DirectionalPredictionSet | None,
    controller_tick: int,
    simulation_time_s: float,
    gate_motion_state: DynamicMotionState,
    gate_stop_epoch: int,
    resume_authorization_revision: int | None,
) -> None:
    if not isinstance(authorization, R5BTemporalExecutionAuthorization):
        raise TypeError("authorization must be an R5BTemporalExecutionAuthorization")
    if authorization.authorization_content_hash != authorization.expected_content_hash:
        raise ValueError("R5-B temporal authorization semantic hash mismatch")
    frame = observation_snapshot.frame
    if prediction_set is None or frame is None:
        raise ValueError("R5-B temporal authorization needs current observation and prediction")
    validate_directional_prediction_set(prediction_set, current_frame=frame)
    if (
        authorization.temporal_evidence_hash
        != reference.source_temporal_evidence_hash
        or authorization.temporal_geometry_hash
        != reference.source_temporal_geometry_hash
    ):
        raise ValueError("R5-B temporal authorization source evidence changed")
    if authorization.prediction_model_version != prediction_set.model_version:
        raise ValueError("R5-B temporal authorization prediction model changed")
    matching_tubes = tuple(
        tube
        for tube in prediction_set.tubes
        if (
            tube.actor_binding_id == authorization.target_actor_binding_id
            and tube.track_id == authorization.target_track_id
        )
    )
    if len(matching_tubes) != 1:
        raise ValueError("R5-B temporal authorization target track changed")
    expected = (
        reference.reference_content_hash,
        reference.mission_id,
        reference.stop_epoch,
        reference.map_id,
        reference.map_revision,
        reference.mission_revision,
        reference.reference_session_id,
        controller_tick,
        simulation_time_s,
        gate_motion_state,
        gate_stop_epoch,
        resume_authorization_revision,
        frame.content_hash,
        frame.observation_revision,
        frame.sequence,
        prediction_set.content_hash,
    )
    actual = (
        authorization.reference_content_hash,
        authorization.mission_id,
        authorization.stop_epoch,
        authorization.map_id,
        authorization.map_revision,
        authorization.mission_revision,
        authorization.reference_session_id,
        authorization.controller_tick,
        authorization.simulation_time_s,
        authorization.gate_motion_state,
        authorization.stop_epoch,
        authorization.resume_authorization_revision,
        authorization.observation_content_hash,
        authorization.observation_revision,
        authorization.observation_sequence,
        authorization.prediction_content_hash,
    )
    if actual != expected:
        raise ValueError("R5-B temporal authorization does not match the current tick")
    if observation_snapshot.availability is not DynamicObservationAvailability.FRESH:
        raise ValueError("R5-B temporal authorization observation is not fresh")
    if observation_snapshot.last_event_was_no_frame or observation_snapshot.failures:
        raise ValueError("R5-B temporal authorization observation is not usable")


def _validate_sources(
    reference: LocalManeuverReference,
    temporal_evidence: TemporalReferenceEvidence,
    temporal_geometry: TemporalReferenceGeometryEvidence,
) -> None:
    if reference.evidence_level is not ReferenceEvidenceLevel.GROUND_TRUTH_TEMPORAL:
        raise ValueError("R5-B authorization requires GROUND_TRUTH_TEMPORAL evidence")
    if reference.source_temporal_evidence_hash != temporal_evidence.evidence_content_hash:
        raise ValueError("R5-B authorization temporal evidence mismatch")
    if reference.source_temporal_geometry_hash != temporal_geometry.geometry_content_hash:
        raise ValueError("R5-B authorization temporal geometry mismatch")
    if (
        temporal_evidence.maneuver_kind is not reference.maneuver_kind
        or temporal_geometry.maneuver_kind is not reference.maneuver_kind
        or temporal_evidence.target_actor_binding_ids
        != temporal_geometry.target_actor_binding_ids
    ):
        raise ValueError("R5-B authorization source semantics mismatch")


def _validated_live_inputs(
    reference: LocalManeuverReference,
    temporal_evidence: TemporalReferenceEvidence,
    snapshot: DynamicObservationSnapshot,
    result: DirectionalPredictionResult,
) -> tuple[object, DirectionalPredictionSet]:
    if snapshot.availability is not DynamicObservationAvailability.FRESH:
        raise ValueError("R5-B authorization requires a fresh observation")
    if snapshot.last_event_was_no_frame or snapshot.failures or snapshot.frame is None:
        raise ValueError("R5-B authorization rejects missing or invalid observations")
    frame = snapshot.frame
    if frame.frame_kind is not DynamicObservationFrameKind.TRACKS or not frame.tracks:
        raise ValueError("R5-B authorization requires a non-empty Actor frame")
    if dynamic_observation_content_hash(frame) != frame.content_hash:
        raise ValueError("R5-B authorization observation hash mismatch")
    if result.status is not DirectionalPredictionStatus.READY or result.hold_required:
        raise ValueError("R5-B authorization requires a READY directional prediction")
    prediction = result.prediction_set
    if prediction is None:
        raise ValueError("R5-B READY result is missing its prediction set")
    validate_directional_prediction_set(prediction, current_frame=frame)
    actor_id = temporal_evidence.target_actor_binding_ids[0]
    matching_tracks = tuple(
        track for track in frame.tracks if track.actor_binding_id == actor_id
    )
    matching_tubes = tuple(
        tube for tube in prediction.tubes if tube.actor_binding_id == actor_id
    )
    if len(matching_tracks) != 1 or len(matching_tubes) != 1:
        raise ValueError("R5-B target Actor is absent or ambiguous")
    if matching_tracks[0].track_id != matching_tubes[0].track_id:
        raise ValueError("R5-B target track identity changed")
    if (frame.map_id, frame.map_revision) != (reference.map_id, reference.map_revision):
        raise ValueError("R5-B live observation map provenance mismatch")
    return frame, prediction


def _require_sha256(value: str, name: str) -> None:
    if not isinstance(value, str) or fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")


def _require_nonempty(value: str, name: str) -> None:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must not be empty")


def _require_nonnegative_int(value: int, name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative exact integer")


__all__ = [
    "R5B_TEMPORAL_AUTHORIZATION_ISSUER_VERSION",
    "R5B_TEMPORAL_AUTHORIZATION_SCHEMA_VERSION",
    "R5BTemporalAuthorizationIssuer",
    "R5BTemporalAuthorizationPhase",
    "R5BTemporalExecutionAuthorization",
    "validate_r5b_temporal_authorization_for_tick",
]
