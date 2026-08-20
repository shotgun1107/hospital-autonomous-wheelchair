"""Stage 3 controller-independent dynamic safety, authority and timing gate."""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from json import dumps
from math import ceil, cos, isclose, isfinite, pi, sin
from typing import TYPE_CHECKING
from weakref import WeakSet

from hospital_path_lab.collision import (
    CollisionChecker,
    oriented_footprint_capsule_surface_distance,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.contracts import GridSnapshot, Pose2D, RobotState, TrajectoryPoint, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_COMMAND_APPLY_LATENCY_S,
    DYNAMIC_CONTROL_PERIOD_S,
    DYNAMIC_OBSERVATION_TTL_S,
    MAX_ACTOR_SPEED_MPS,
    DynamicCommandProposal,
    DynamicHoldReason,
    DynamicMotionState,
    DynamicSafetyDecision,
    DynamicSafetyEventCounters,
    ResumeAuthorization,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalCapsuleSample,
    DirectionalPredictionSet,
    sample_directional_capsules,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import (
    ActorPredictionSet,
    ActorTubeCircle,
    sample_actor_tubes,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile

if TYPE_CHECKING:
    from hospital_path_lab.persistent_controller_contracts import PersistentReferenceBinding

DYNAMIC_ANGULAR_DECELERATION_RADPS2 = 1.60
DYNAMIC_STOP_LINEAR_THRESHOLD_MPS = 0.01
DYNAMIC_STOP_ANGULAR_THRESHOLD_RADPS = 0.02
DYNAMIC_STOP_CONFIRMATION_TICKS = 3
DYNAMIC_SAFE_OBSERVATION_FRAMES = 11
DYNAMIC_COMMAND_DEADLINE_S = 0.050
DYNAMIC_SWEEP_SAMPLE_PERIOD_S = 0.005
_GEOMETRY_TOLERANCE = 1e-12
_COLLISION_CHECKER_TYPE = CollisionChecker
_DYNAMIC_CHECKER_FACTORY_CAPABILITY = object()


@dataclass(frozen=True, slots=True)
class DynamicSafetyContext:
    tick_id: int
    simulation_time_s: float
    mission_id: str
    authorization_revision: int
    grid_snapshot: GridSnapshot
    observation_snapshot: DynamicObservationSnapshot
    prediction_set: ActorPredictionSet | DirectionalPredictionSet | None
    path_still_valid: bool
    local_safety_recheck_passed: bool
    observation_safe: bool
    resume_authorization: ResumeAuthorization | None = None
    goal_reached: bool = False
    mission_cancelled: bool = False
    reference_binding: PersistentReferenceBinding | None = None

    def __post_init__(self) -> None:
        if self.tick_id < 0 or self.authorization_revision < 0:
            raise ValueError("dynamic safety context counters must not be negative")
        if not self.mission_id:
            raise ValueError("dynamic safety context mission_id must not be empty")
        if not isfinite(self.simulation_time_s) or self.simulation_time_s < 0.0:
            raise ValueError("dynamic safety context time must be finite and non-negative")
        for value in (
            self.path_still_valid,
            self.local_safety_recheck_passed,
            self.observation_safe,
            self.goal_reached,
            self.mission_cancelled,
        ):
            if not isinstance(value, bool):
                raise TypeError("dynamic safety context flags must be bool values")
        if self.reference_binding is not None:
            from hospital_path_lab.persistent_controller_contracts import (
                PersistentReferenceBinding,
            )

            if not isinstance(self.reference_binding, PersistentReferenceBinding):
                raise TypeError(
                    "reference_binding must be a PersistentReferenceBinding when present"
                )


@dataclass(frozen=True, slots=True)
class DynamicTrajectorySafetyEvidence:
    """Controller와 gate가 공유하는 stateless predicted-trajectory 판정."""

    safe: bool
    actor_hazard: bool
    forbidden_entry: bool
    minimum_static_clearance_m: float | None
    minimum_actor_clearance_m: float | None
    failures: tuple[str, ...]


@dataclass(frozen=True, slots=True, init=False, eq=False, weakref_slot=True)
class DynamicTrajectorySafetyCheckers:
    """Collision checkers bound to one exact grid snapshot and profile.

    ``CollisionChecker`` lazily builds geometry caches.  DWB evaluates many
    trajectories against the same immutable snapshot, so rebuilding both
    checkers per candidate discards those caches without changing the safety
    question.  This object permits reuse while retaining an identity check that
    prevents a checker from crossing snapshot boundaries.
    """

    grid_snapshot: GridSnapshot
    grid_source: object
    profile: VehicleProfile
    forbidden_cells_source: frozenset[tuple[int, int]]
    physical_checker: CollisionChecker
    combined_checker: CollisionChecker
    _factory_capability: object

    def __init__(
        self,
        grid_snapshot: GridSnapshot,
        profile: VehicleProfile,
        physical_checker: CollisionChecker,
        combined_checker: CollisionChecker,
        *,
        _factory_capability: object | None = None,
    ) -> None:
        if _factory_capability is not _DYNAMIC_CHECKER_FACTORY_CAPABILITY:
            raise TypeError(
                "DynamicTrajectorySafetyCheckers must be created by "
                "build_dynamic_trajectory_safety_checkers"
            )
        object.__setattr__(self, "grid_snapshot", grid_snapshot)
        object.__setattr__(self, "grid_source", grid_snapshot.grid)
        object.__setattr__(self, "profile", profile)
        object.__setattr__(
            self,
            "forbidden_cells_source",
            grid_snapshot.forbidden_cells,
        )
        object.__setattr__(self, "physical_checker", physical_checker)
        object.__setattr__(self, "combined_checker", combined_checker)
        object.__setattr__(self, "_factory_capability", _factory_capability)


_FACTORY_ISSUED_DYNAMIC_CHECKERS: WeakSet[DynamicTrajectorySafetyCheckers] = WeakSet()


def build_dynamic_trajectory_safety_checkers(
    *,
    grid_snapshot: GridSnapshot,
    profile: VehicleProfile,
) -> DynamicTrajectorySafetyCheckers:
    """Build the two shared-safety checkers once for one immutable snapshot."""

    checkers = DynamicTrajectorySafetyCheckers(
        grid_snapshot=grid_snapshot,
        profile=profile,
        physical_checker=CollisionChecker(grid_snapshot.grid, profile),
        combined_checker=CollisionChecker(
            grid_snapshot.grid,
            profile,
            forbidden_cells=grid_snapshot.forbidden_cells,
        ),
        _factory_capability=_DYNAMIC_CHECKER_FACTORY_CAPABILITY,
    )
    _validate_checker_pair_sources(
        checkers,
        grid_snapshot=grid_snapshot,
        profile=profile,
    )
    _FACTORY_ISSUED_DYNAMIC_CHECKERS.add(checkers)
    return checkers


class DynamicSafetyGate:
    """공통 online command filter와 보호정지 권한 상태기계."""

    def __init__(
        self,
        *,
        profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        initial_stop_epoch: int = 0,
    ) -> None:
        if not profile.simulation_only:
            raise ValueError("dynamic research gate requires a simulation-only vehicle profile")
        if (
            isinstance(initial_stop_epoch, bool)
            or not isinstance(initial_stop_epoch, int)
            or initial_stop_epoch < 0
        ):
            raise ValueError("initial_stop_epoch must be a non-negative exact integer")
        self.profile = profile
        self.motion_state = DynamicMotionState.MOVING
        self.stop_epoch = initial_stop_epoch
        self.stop_confirmed_at_s: float | None = None
        self._consecutive_stop_ticks = 0
        self._consecutive_safe_frames = 0
        self._last_safe_sequence: int | None = None
        self._last_tick_id: int | None = None
        self._mission_end_requested = False
        self._controller_stop_requests = 0
        self._gate_overrides = 0
        self._candidate_rejected_by_gate = 0
        self._late_results_discarded = 0
        self._resume_authorizations_rejected = 0
        # A valid authorization becomes spent only when it actually releases
        # this gate from HOLDING.  Keep the exact content hash for the gate
        # lifetime so a replay cannot turn the same protective-stop state
        # into motion twice.
        self._consumed_resume_authorization_hashes: set[str] = set()

    def step(
        self,
        proposal: DynamicCommandProposal,
        *,
        robot_state: RobotState,
        context: DynamicSafetyContext,
    ) -> DynamicSafetyDecision:
        self._validate_tick_order(context.tick_id)
        reference_failures = _reference_binding_failures(
            proposal,
            context,
            expected_stop_epoch=self.stop_epoch,
        )
        self._mission_end_requested |= not reference_failures and (
            context.goal_reached or context.mission_cancelled
        )
        if self._mission_end_requested:
            if self.motion_state is DynamicMotionState.HOLDING:
                self.motion_state = DynamicMotionState.COMPLETED
                command = Twist2D()
            else:
                self.motion_state = DynamicMotionState.BRAKING
                command = _limited_deceleration(robot_state.twist, self.profile)
                self._record_completion_confirmation(robot_state)
            if command != proposal.command:
                self._gate_overrides += 1
            self._last_tick_id = context.tick_id
            return self._decision(
                proposal,
                context,
                command=command,
                proposal_accepted=False,
                resume_allowed=False,
                reason=None,
                evidence=DynamicTrajectorySafetyEvidence(True, False, False, None, None, ()),
            )
        if self.motion_state is DynamicMotionState.COMPLETED:
            raise RuntimeError("completed dynamic safety gate cannot accept another mission tick")

        source_reason, source_failures = _source_reason(context)
        proposal_failures = _proposal_provenance_failures(proposal, context)
        if proposal_failures:
            source_reason = DynamicHoldReason.INVALID_SOURCE
            source_failures = tuple((*source_failures, *proposal_failures))
        if reference_failures:
            source_reason = DynamicHoldReason.INVALID_REFERENCE
            source_failures = tuple((*source_failures, *reference_failures))
        deadline_failed = (
            proposal.source_tick_id != context.tick_id
            or proposal.computation_time_s > DYNAMIC_COMMAND_DEADLINE_S
        )
        if deadline_failed:
            self._late_results_discarded += 1

        evidence = DynamicTrajectorySafetyEvidence(True, False, False, None, None, ())
        if source_reason is None and not deadline_failed:
            evidence = evaluate_dynamic_trajectory_safety(
                proposal,
                robot_state=robot_state,
                grid_snapshot=context.grid_snapshot,
                prediction_set=context.prediction_set,
                profile=self.profile,
            )

        reason = source_reason
        if reason is None and deadline_failed:
            reason = DynamicHoldReason.DEADLINE
        static_gate_rejection = any(
            failure != "actor_clearance_below_minimum" for failure in evidence.failures
        )
        if reason is None and static_gate_rejection:
            self._candidate_rejected_by_gate += 1
            reason = DynamicHoldReason.GATE_REJECTION
        if reason is None and proposal.no_safe_candidate:
            reason = DynamicHoldReason.NO_SAFE_CANDIDATE
        if reason is None and (proposal.controller_requested_stop or evidence.actor_hazard):
            if not evidence.safe:
                self._candidate_rejected_by_gate += 1
            if proposal.controller_requested_stop:
                self._controller_stop_requests += 1
            reason = DynamicHoldReason.TRAFFIC
        elif proposal.controller_requested_stop:
            self._controller_stop_requests += 1

        failures = tuple((*source_failures, *evidence.failures))
        if deadline_failed:
            failures = (*failures, "late_or_wrong_tick_result")

        if self.motion_state is DynamicMotionState.MOVING and reason is not None:
            self.motion_state = DynamicMotionState.BRAKING
            self._consecutive_stop_ticks = 0
            self._consecutive_safe_frames = 0
            self._last_safe_sequence = None

        resume_allowed = False
        proposal_accepted = False
        if self.motion_state is DynamicMotionState.BRAKING:
            command = _limited_deceleration(robot_state.twist, self.profile)
            self._record_stop_confirmation(robot_state, context.simulation_time_s)
            if reason is None:
                reason = DynamicHoldReason.UNAUTHORIZED
        elif self.motion_state is DynamicMotionState.HOLDING:
            self._record_safe_observation(context, evidence)
            authorization_failures = self._authorization_failures(context)
            authorization_valid = not authorization_failures
            if context.resume_authorization is not None and not authorization_valid:
                self._resume_authorizations_rejected += 1
            authority_failures: list[str] = []
            if not authorization_valid:
                authority_failures.append("resume_authorization_invalid")
                authority_failures.extend(authorization_failures)
                if reason not in {
                    DynamicHoldReason.INVALID_SOURCE,
                    DynamicHoldReason.INVALID_REFERENCE,
                    DynamicHoldReason.STALE,
                    DynamicHoldReason.DEADLINE,
                }:
                    reason = DynamicHoldReason.UNAUTHORIZED
            if not context.path_still_valid:
                authority_failures.append("path_not_valid")
            if not context.local_safety_recheck_passed:
                authority_failures.append("local_safety_recheck_failed")
            if self._consecutive_safe_frames < DYNAMIC_SAFE_OBSERVATION_FRAMES:
                authority_failures.append("continuous_safe_frames_incomplete")
            failures = tuple((*failures, *authority_failures))
            resume_allowed = all(
                (
                    authorization_valid,
                    context.path_still_valid,
                    context.local_safety_recheck_passed,
                    self._consecutive_safe_frames >= DYNAMIC_SAFE_OBSERVATION_FRAMES,
                    source_reason is None,
                    not deadline_failed,
                    reason is None,
                    evidence.safe,
                )
            )
            if resume_allowed:
                authorization = context.resume_authorization
                if authorization is None:
                    raise AssertionError(
                        "a successful dynamic safety resume requires an authorization"
                    )
                self._consumed_resume_authorization_hashes.add(authorization.content_hash)
                self.motion_state = DynamicMotionState.MOVING
                self._consecutive_stop_ticks = 0
                command = proposal.command
                proposal_accepted = True
                reason = None
            else:
                command = Twist2D()
                reason = reason or DynamicHoldReason.UNAUTHORIZED
        else:
            command = proposal.command
            proposal_accepted = True

        if command != proposal.command:
            self._gate_overrides += 1
        self._last_tick_id = context.tick_id
        return self._decision(
            proposal,
            context,
            command=command,
            proposal_accepted=proposal_accepted,
            resume_allowed=resume_allowed,
            reason=reason,
            evidence=DynamicTrajectorySafetyEvidence(
                evidence.safe,
                evidence.actor_hazard,
                evidence.forbidden_entry,
                evidence.minimum_static_clearance_m,
                evidence.minimum_actor_clearance_m,
                failures,
            ),
        )

    def _validate_tick_order(self, tick_id: int) -> None:
        if self._last_tick_id is not None and tick_id <= self._last_tick_id:
            raise ValueError("dynamic safety context tick_id must increase")

    def _record_stop_confirmation(
        self,
        robot_state: RobotState,
        simulation_time_s: float,
    ) -> None:
        stopped = (
            abs(robot_state.twist.linear) <= DYNAMIC_STOP_LINEAR_THRESHOLD_MPS
            and abs(robot_state.twist.angular) <= DYNAMIC_STOP_ANGULAR_THRESHOLD_RADPS
        )
        self._consecutive_stop_ticks = self._consecutive_stop_ticks + 1 if stopped else 0
        if self._consecutive_stop_ticks < DYNAMIC_STOP_CONFIRMATION_TICKS:
            return
        self.motion_state = DynamicMotionState.HOLDING
        self.stop_epoch += 1
        self.stop_confirmed_at_s = simulation_time_s
        self._consecutive_safe_frames = 0
        self._last_safe_sequence = None

    def _record_completion_confirmation(self, robot_state: RobotState) -> None:
        stopped = (
            abs(robot_state.twist.linear) <= DYNAMIC_STOP_LINEAR_THRESHOLD_MPS
            and abs(robot_state.twist.angular) <= DYNAMIC_STOP_ANGULAR_THRESHOLD_RADPS
        )
        self._consecutive_stop_ticks = self._consecutive_stop_ticks + 1 if stopped else 0
        if self._consecutive_stop_ticks >= DYNAMIC_STOP_CONFIRMATION_TICKS:
            self.motion_state = DynamicMotionState.COMPLETED

    def _record_safe_observation(
        self,
        context: DynamicSafetyContext,
        evidence: DynamicTrajectorySafetyEvidence,
    ) -> None:
        snapshot = context.observation_snapshot
        frame = snapshot.frame
        safe = (
            snapshot.availability is DynamicObservationAvailability.FRESH
            and not snapshot.last_event_was_no_frame
            and frame is not None
            and context.observation_safe
            and evidence.safe
        )
        if not safe:
            self._consecutive_safe_frames = 0
            self._last_safe_sequence = None
            return
        if frame.sequence == self._last_safe_sequence:
            return
        self._consecutive_safe_frames += 1
        self._last_safe_sequence = frame.sequence

    def _authorization_is_valid(self, context: DynamicSafetyContext) -> bool:
        return not self._authorization_failures(context)

    def _authorization_failures(self, context: DynamicSafetyContext) -> tuple[str, ...]:
        authorization = context.resume_authorization
        if authorization is None:
            return ("resume_authorization_missing",)
        if self.stop_confirmed_at_s is None:
            return ("stop_confirmation_missing",)
        expected_hash = resume_authorization_content_hash(
            mission_id=authorization.mission_id,
            stop_epoch=authorization.stop_epoch,
            issued_or_revalidated_at_s=authorization.issued_or_revalidated_at_s,
            authorization_revision=authorization.authorization_revision,
        )
        failures: list[str] = []
        if authorization.content_hash != expected_hash:
            failures.append("resume_authorization_hash_mismatch")
        elif authorization.content_hash in self._consumed_resume_authorization_hashes:
            failures.append("resume_authorization_already_consumed")
        if authorization.mission_id != context.mission_id:
            failures.append("resume_authorization_mission_mismatch")
        if authorization.stop_epoch != self.stop_epoch:
            failures.append("resume_authorization_stop_epoch_mismatch")
        if authorization.issued_or_revalidated_at_s < self.stop_confirmed_at_s:
            failures.append("resume_authorization_predates_stop")
        if authorization.authorization_revision != context.authorization_revision:
            failures.append("resume_authorization_revision_mismatch")
        return tuple(failures)

    def _decision(
        self,
        proposal: DynamicCommandProposal,
        context: DynamicSafetyContext,
        *,
        command: Twist2D,
        proposal_accepted: bool,
        resume_allowed: bool,
        reason: DynamicHoldReason | None,
        evidence: DynamicTrajectorySafetyEvidence,
    ) -> DynamicSafetyDecision:
        return DynamicSafetyDecision(
            tick_id=context.tick_id,
            source_tick_id=proposal.source_tick_id,
            motion_state=self.motion_state,
            stop_epoch=self.stop_epoch,
            command=command,
            proposal_accepted=proposal_accepted,
            resume_allowed=resume_allowed,
            primary_hold_reason=reason,
            consecutive_stop_ticks=self._consecutive_stop_ticks,
            consecutive_safe_frames=self._consecutive_safe_frames,
            minimum_static_clearance_m=evidence.minimum_static_clearance_m,
            minimum_actor_clearance_m=evidence.minimum_actor_clearance_m,
            counters=self.counters,
            failure_reasons=evidence.failures,
        )

    @property
    def counters(self) -> DynamicSafetyEventCounters:
        return DynamicSafetyEventCounters(
            controller_stop_requests=self._controller_stop_requests,
            gate_overrides=self._gate_overrides,
            candidate_rejected_by_gate=self._candidate_rejected_by_gate,
            late_results_discarded=self._late_results_discarded,
            resume_authorizations_rejected=self._resume_authorizations_rejected,
        )


def build_resume_authorization(
    *,
    mission_id: str,
    stop_epoch: int,
    issued_or_revalidated_at_s: float,
    authorization_revision: int,
) -> ResumeAuthorization:
    return ResumeAuthorization(
        mission_id=mission_id,
        stop_epoch=stop_epoch,
        issued_or_revalidated_at_s=issued_or_revalidated_at_s,
        authorization_revision=authorization_revision,
        content_hash=resume_authorization_content_hash(
            mission_id=mission_id,
            stop_epoch=stop_epoch,
            issued_or_revalidated_at_s=issued_or_revalidated_at_s,
            authorization_revision=authorization_revision,
        ),
    )


def build_dynamic_command_proposal(
    context: DynamicSafetyContext,
    *,
    command: Twist2D,
    computation_time_s: float,
    source_tick_id: int | None = None,
    trajectory: tuple[TrajectoryPoint, ...] = (),
    controller_requested_stop: bool = False,
    no_safe_candidate: bool = False,
) -> DynamicCommandProposal:
    """현재 immutable context에 명령 후보 provenance를 결합한다."""

    metadata = context.grid_snapshot.metadata
    frame = context.observation_snapshot.frame
    observation_content_hash = (
        frame.content_hash if frame is not None else "observation-unavailable"
    )
    return DynamicCommandProposal(
        source_tick_id=context.tick_id if source_tick_id is None else source_tick_id,
        command=command,
        computation_time_s=computation_time_s,
        mission_id=context.mission_id,
        map_id=metadata.map_id,
        map_revision=metadata.map_revision,
        mission_revision=metadata.mission_revision,
        observation_revision=metadata.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=observation_content_hash,
        trajectory=trajectory,
        controller_requested_stop=controller_requested_stop,
        no_safe_candidate=no_safe_candidate,
        reference_binding=context.reference_binding,
    )


def resume_authorization_content_hash(
    *,
    mission_id: str,
    stop_epoch: int,
    issued_or_revalidated_at_s: float,
    authorization_revision: int,
) -> str:
    payload = {
        "authorization_revision": authorization_revision,
        "issued_or_revalidated_at_s": issued_or_revalidated_at_s,
        "mission_id": mission_id,
        "stop_epoch": stop_epoch,
    }
    serialized = dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return sha256(serialized.encode("utf-8")).hexdigest()


def _source_reason(
    context: DynamicSafetyContext,
) -> tuple[DynamicHoldReason | None, tuple[str, ...]]:
    snapshot = context.observation_snapshot
    if snapshot.availability is DynamicObservationAvailability.STALE:
        return DynamicHoldReason.STALE, tuple(reason.value for reason in snapshot.failures)
    if snapshot.availability is not DynamicObservationAvailability.FRESH:
        failures = tuple(reason.value for reason in snapshot.failures) or (
            "observation_source_unavailable",
        )
        return DynamicHoldReason.INVALID_SOURCE, failures
    if snapshot.frame is None or context.prediction_set is None:
        return DynamicHoldReason.INVALID_SOURCE, ("fresh_prediction_missing",)
    frame = snapshot.frame
    prediction = context.prediction_set
    if type(prediction) not in (ActorPredictionSet, DirectionalPredictionSet):
        return DynamicHoldReason.INVALID_SOURCE, ("prediction_type_invalid",)
    time_reason = _source_time_reason(context, frame, prediction)
    if time_reason is not None:
        return time_reason
    metadata = context.grid_snapshot.metadata
    identity_matches = all(
        (
            prediction.stream_id == frame.stream_id,
            prediction.episode_id == frame.episode_id,
            prediction.map_id == frame.map_id,
            prediction.map_revision == frame.map_revision,
            prediction.observation_revision == frame.observation_revision,
            prediction.sequence == frame.sequence,
            prediction.source_content_hash == frame.content_hash,
            metadata.map_id == frame.map_id,
            metadata.map_revision == frame.map_revision,
            metadata.observation_revision == frame.observation_revision,
        )
    )
    if not identity_matches:
        return DynamicHoldReason.INVALID_SOURCE, ("prediction_source_mismatch",)
    return None, ()


def _source_time_reason(
    context: DynamicSafetyContext,
    frame,
    prediction: ActorPredictionSet | DirectionalPredictionSet,
) -> tuple[DynamicHoldReason, tuple[str, ...]] | None:
    """Bind a fresh snapshot and prediction to this exact simulation tick.

    A validated snapshot is immutable.  Replaying it later while leaving its
    cached ``FRESH`` availability unchanged must therefore not extend its life.
    The gate derives age again from the current simulation clock before trusting
    either the snapshot age or the prediction's controller-time commitment.
    """

    snapshot = context.observation_snapshot
    values = (
        snapshot.age_s,
        frame.observed_at_s,
        frame.delivered_at_s,
        prediction.observed_at_s,
        prediction.controller_time_s,
        prediction.snapshot_age_s,
    )
    if any(value is None or not isfinite(value) for value in values):
        return DynamicHoldReason.INVALID_SOURCE, ("observation_time_invalid",)
    assert snapshot.age_s is not None
    actual_age_s = context.simulation_time_s - frame.observed_at_s
    if not isfinite(actual_age_s) or actual_age_s < -_GEOMETRY_TOLERANCE:
        return DynamicHoldReason.INVALID_SOURCE, ("observation_time_invalid",)
    if actual_age_s > DYNAMIC_OBSERVATION_TTL_S + _GEOMETRY_TOLERANCE:
        return DynamicHoldReason.STALE, ("observation_replay_stale",)
    time_matches = all(
        (
            frame.delivered_at_s <= context.simulation_time_s + _GEOMETRY_TOLERANCE,
            isclose(
                snapshot.age_s,
                actual_age_s,
                rel_tol=0.0,
                abs_tol=_GEOMETRY_TOLERANCE,
            ),
            isclose(
                prediction.observed_at_s,
                frame.observed_at_s,
                rel_tol=0.0,
                abs_tol=_GEOMETRY_TOLERANCE,
            ),
            isclose(
                prediction.snapshot_age_s,
                actual_age_s,
                rel_tol=0.0,
                abs_tol=_GEOMETRY_TOLERANCE,
            ),
            isclose(
                prediction.controller_time_s,
                context.simulation_time_s,
                rel_tol=0.0,
                abs_tol=_GEOMETRY_TOLERANCE,
            ),
        )
    )
    if not time_matches:
        return DynamicHoldReason.INVALID_SOURCE, ("observation_prediction_time_mismatch",)
    return None


def _proposal_provenance_failures(
    proposal: DynamicCommandProposal,
    context: DynamicSafetyContext,
) -> tuple[str, ...]:
    metadata = context.grid_snapshot.metadata
    frame = context.observation_snapshot.frame
    matches = all(
        (
            proposal.mission_id == context.mission_id,
            proposal.map_id == metadata.map_id,
            proposal.map_revision == metadata.map_revision,
            proposal.mission_revision == metadata.mission_revision,
            proposal.observation_revision == metadata.observation_revision,
            proposal.grid_content_hash == metadata.content_hash,
            frame is None or proposal.observation_content_hash == frame.content_hash,
        )
    )
    return () if matches else ("proposal_provenance_mismatch",)


def _reference_binding_failures(
    proposal: DynamicCommandProposal,
    context: DynamicSafetyContext,
    *,
    expected_stop_epoch: int,
) -> tuple[str, ...]:
    """Validate the optional R5 reference capability at the final command boundary.

    Legacy dynamic lanes deliberately carry ``None`` on both sides.  Once either
    side opts into R5, both immutable bindings must be the exact current delivery;
    a previous session/window/tick/stop epoch is never reusable.
    """

    proposal_binding = proposal.reference_binding
    context_binding = context.reference_binding
    if proposal_binding is None and context_binding is None:
        return ()
    if proposal_binding is None or context_binding is None:
        return ("reference_binding_mismatch",)
    try:
        from hospital_path_lab.local_reference_contracts import ReferenceLifecycleStatus
        from hospital_path_lab.persistent_controller_contracts import (
            PersistentReferenceBinding,
        )

        bindings_valid = all(
            (
                type(proposal_binding) is PersistentReferenceBinding,
                type(context_binding) is PersistentReferenceBinding,
                proposal_binding.binding_content_hash == proposal_binding.expected_content_hash,
                context_binding.binding_content_hash == context_binding.expected_content_hash,
                proposal_binding.lifecycle is ReferenceLifecycleStatus.AVAILABLE,
                context_binding.lifecycle is ReferenceLifecycleStatus.AVAILABLE,
                proposal_binding == context_binding,
                proposal_binding.source_window_control_tick == proposal.source_tick_id,
                context_binding.source_window_control_tick == context.tick_id,
                proposal_binding.stop_epoch == expected_stop_epoch,
                context_binding.stop_epoch == expected_stop_epoch,
            )
        )
    except (AttributeError, TypeError, ValueError):
        bindings_valid = False
    return () if bindings_valid else ("reference_binding_mismatch",)


def evaluate_dynamic_trajectory_safety(
    proposal: DynamicCommandProposal,
    *,
    robot_state: RobotState,
    grid_snapshot: GridSnapshot,
    prediction_set: ActorPredictionSet | DirectionalPredictionSet | None,
    profile: VehicleProfile,
    checkers: DynamicTrajectorySafetyCheckers | None = None,
) -> DynamicTrajectorySafetyEvidence:
    """현재 운동·post-apply rollout·terminal stopping을 같은 계약으로 검사한다."""

    if not _robot_state_is_finite(robot_state):
        return _unsafe_evidence("robot_state_non_finite")
    if not _command_inside_limits(proposal.command, profile):
        return _unsafe_evidence("proposal_command_outside_vehicle_limits")
    if not grid_snapshot.input_valid:
        return _unsafe_evidence("grid_snapshot_invalid")
    prediction = prediction_set
    if prediction is None:
        return _unsafe_evidence("prediction_set_missing")
    if type(prediction) not in (ActorPredictionSet, DirectionalPredictionSet):
        return _unsafe_evidence("prediction_set_type_invalid")

    if checkers is None:
        checkers = build_dynamic_trajectory_safety_checkers(
            grid_snapshot=grid_snapshot,
            profile=profile,
        )
    else:
        _validate_dynamic_trajectory_safety_checkers(
            checkers,
            grid_snapshot=grid_snapshot,
            profile=profile,
        )
    physical_checker = checkers.physical_checker
    combined_checker = checkers.combined_checker
    static_clearances: list[float] = []
    actor_clearances: list[float] = []
    failures: list[str] = []
    actor_hazard = False
    forbidden_entry = False

    apply_samples = _constant_command_samples(
        robot_state.pose,
        robot_state.twist,
        duration_s=DYNAMIC_COMMAND_APPLY_LATENCY_S,
    )
    apply_end = apply_samples[-1].pose
    for point in apply_samples:
        remaining_s = DYNAMIC_COMMAND_APPLY_LATENCY_S - point.time_s
        try:
            sample = _sample_actor_safety_shapes(prediction, rollout_time_s=0.0)
            result = _pose_safety(
                point.pose,
                physical_checker=physical_checker,
                combined_checker=combined_checker,
                actor_shapes=sample,
                actor_radius_expansion_m=MAX_ACTOR_SPEED_MPS * remaining_s,
                profile=profile,
            )
        except (AttributeError, OverflowError, TypeError, ValueError):
            return _unsafe_evidence("prediction_set_malformed")
        _merge_pose_result(
            result,
            static_clearances,
            actor_clearances,
            failures,
        )
        actor_hazard |= result.actor_hazard
        forbidden_entry |= result.forbidden_entry

    try:
        rollout = _normalized_rollout(proposal, apply_end, profile)
    except (TypeError, ValueError) as error:
        return _unsafe_evidence(f"proposal_trajectory_invalid:{error}")
    for point in rollout:
        try:
            result = _pose_safety(
                point.pose,
                physical_checker=physical_checker,
                combined_checker=combined_checker,
                actor_shapes=_sample_actor_safety_shapes(
                    prediction,
                    rollout_time_s=point.time_s,
                ),
                profile=profile,
            )
        except (AttributeError, OverflowError, TypeError, ValueError):
            return _unsafe_evidence("prediction_set_malformed")
        _merge_pose_result(result, static_clearances, actor_clearances, failures)
        actor_hazard |= result.actor_hazard
        forbidden_entry |= result.forbidden_entry

    terminal = _terminal_stopping_samples(rollout[-1], profile)
    terminal_offset_s = rollout[-1].time_s
    for point in terminal[1:]:
        try:
            result = _pose_safety(
                point.pose,
                physical_checker=physical_checker,
                combined_checker=combined_checker,
                actor_shapes=_sample_actor_safety_shapes(
                    prediction,
                    rollout_time_s=terminal_offset_s + point.time_s,
                ),
                profile=profile,
            )
        except (AttributeError, OverflowError, TypeError, ValueError):
            return _unsafe_evidence("prediction_set_malformed")
        _merge_pose_result(result, static_clearances, actor_clearances, failures)
        actor_hazard |= result.actor_hazard
        forbidden_entry |= result.forbidden_entry

    unique_failures = tuple(dict.fromkeys(failures))
    return DynamicTrajectorySafetyEvidence(
        safe=not unique_failures,
        actor_hazard=actor_hazard,
        forbidden_entry=forbidden_entry,
        minimum_static_clearance_m=min(static_clearances) if static_clearances else None,
        minimum_actor_clearance_m=min(actor_clearances) if actor_clearances else None,
        failures=unique_failures,
    )


def _validate_dynamic_trajectory_safety_checkers(
    checkers: DynamicTrajectorySafetyCheckers,
    *,
    grid_snapshot: GridSnapshot,
    profile: VehicleProfile,
) -> None:
    """Reject stale, foreign, or differently configured prebuilt checkers."""

    if type(checkers) is not DynamicTrajectorySafetyCheckers:
        raise TypeError("checkers must be DynamicTrajectorySafetyCheckers")
    if (
        checkers._factory_capability is not _DYNAMIC_CHECKER_FACTORY_CAPABILITY
        or checkers not in _FACTORY_ISSUED_DYNAMIC_CHECKERS
    ):
        raise ValueError("prebuilt checkers were not issued by the checker factory")
    if checkers.grid_snapshot is not grid_snapshot:
        raise ValueError("prebuilt checkers belong to a different grid snapshot")
    if checkers.grid_source is not grid_snapshot.grid:
        raise ValueError("prebuilt checkers belong to a different grid source")
    if checkers.profile is not profile:
        raise ValueError("prebuilt checkers belong to a different vehicle profile")
    if checkers.forbidden_cells_source is not grid_snapshot.forbidden_cells:
        raise ValueError("prebuilt checkers belong to different forbidden cells")

    _validate_checker_pair_sources(
        checkers,
        grid_snapshot=grid_snapshot,
        profile=profile,
    )


def _validate_checker_pair_sources(
    checkers: DynamicTrajectorySafetyCheckers,
    *,
    grid_snapshot: GridSnapshot,
    profile: VehicleProfile,
) -> None:
    """Require the exact checker implementation and its exact immutable sources."""

    physical = checkers.physical_checker
    combined = checkers.combined_checker
    if (
        type(physical) is not _COLLISION_CHECKER_TYPE
        or type(combined) is not _COLLISION_CHECKER_TYPE
        or physical.grid is not grid_snapshot.grid
        or combined.grid is not grid_snapshot.grid
        or physical.profile is not profile
        or combined.profile is not profile
        or physical.forbidden_cells
        or combined.forbidden_cells != grid_snapshot.forbidden_cells
        or not physical.use_optimized_geometry
        or not combined.use_optimized_geometry
    ):
        raise ValueError("prebuilt checkers do not match the safety inputs")


def _unsafe_evidence(reason: str) -> DynamicTrajectorySafetyEvidence:
    return DynamicTrajectorySafetyEvidence(False, False, False, None, None, (reason,))


def _sample_actor_safety_shapes(
    prediction_set: ActorPredictionSet | DirectionalPredictionSet,
    *,
    rollout_time_s: float,
) -> tuple[ActorTubeCircle | DirectionalCapsuleSample, ...]:
    """Sample the exact geometry frozen by the selected prediction contract."""

    if type(prediction_set) is ActorPredictionSet:
        return sample_actor_tubes(prediction_set, rollout_time_s=rollout_time_s)
    if type(prediction_set) is DirectionalPredictionSet:
        return sample_directional_capsules(
            prediction_set,
            rollout_time_s=rollout_time_s,
        )
    raise TypeError("unsupported Actor prediction set type")


def _pose_safety(
    pose: Pose2D,
    *,
    physical_checker: CollisionChecker,
    combined_checker: CollisionChecker,
    actor_shapes: tuple[ActorTubeCircle | DirectionalCapsuleSample, ...],
    profile: VehicleProfile,
    actor_radius_expansion_m: float = 0.0,
) -> DynamicTrajectorySafetyEvidence:
    if not isfinite(actor_radius_expansion_m) or actor_radius_expansion_m < 0.0:
        raise ValueError("Actor radius expansion must be finite and non-negative")
    physical_clearance = physical_checker.clearance(pose)
    combined_clearance = combined_checker.clearance(pose)
    forbidden_entry = combined_checker.pose_enters_forbidden(pose)
    static_clearance = min(physical_clearance, combined_clearance)
    actor_clearances: list[float] = []
    for shape in actor_shapes:
        if type(shape) is ActorTubeCircle:
            actor_clearances.append(
                oriented_footprint_circle_surface_distance(
                    pose,
                    circle_center=(shape.center.x, shape.center.y),
                    circle_radius_m=shape.radius_m + actor_radius_expansion_m,
                    profile=profile,
                )
            )
        elif type(shape) is DirectionalCapsuleSample:
            actor_clearances.append(
                oriented_footprint_capsule_surface_distance(
                    pose,
                    segment_start=(shape.start.x, shape.start.y),
                    segment_end=(shape.end.x, shape.end.y),
                    capsule_radius_m=(shape.base_radius_m + actor_radius_expansion_m),
                    profile=profile,
                )
            )
        else:
            raise TypeError("unsupported Actor safety geometry")
    actor_clearance = min(actor_clearances) if actor_clearances else None
    failures: list[str] = []
    if forbidden_entry:
        failures.append("forbidden_zone_entry")
    elif static_clearance < profile.minimum_clearance_m - _GEOMETRY_TOLERANCE:
        failures.append("static_clearance_below_minimum")
    if actor_clearance is not None and actor_clearance < (
        profile.minimum_clearance_m - _GEOMETRY_TOLERANCE
    ):
        failures.append("actor_clearance_below_minimum")
    return DynamicTrajectorySafetyEvidence(
        safe=not failures,
        actor_hazard="actor_clearance_below_minimum" in failures,
        forbidden_entry=forbidden_entry,
        minimum_static_clearance_m=static_clearance,
        minimum_actor_clearance_m=actor_clearance,
        failures=tuple(failures),
    )


def _merge_pose_result(
    result: DynamicTrajectorySafetyEvidence,
    static_clearances: list[float],
    actor_clearances: list[float],
    failures: list[str],
) -> None:
    if result.minimum_static_clearance_m is not None:
        static_clearances.append(result.minimum_static_clearance_m)
    if result.minimum_actor_clearance_m is not None:
        actor_clearances.append(result.minimum_actor_clearance_m)
    failures.extend(result.failures)


def _normalized_rollout(
    proposal: DynamicCommandProposal,
    apply_end: Pose2D,
    profile: VehicleProfile,
) -> tuple[TrajectoryPoint, ...]:
    if not proposal.trajectory:
        return _constant_command_samples(
            apply_end,
            proposal.command,
            duration_s=profile.control_period_s,
        )
    trajectory = proposal.trajectory
    previous_time = -1.0
    for point in trajectory:
        values = (
            point.time_s,
            point.pose.x,
            point.pose.y,
            point.pose.yaw,
            point.twist.linear,
            point.twist.angular,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("proposal trajectory must be finite")
        if point.time_s <= previous_time:
            raise ValueError("proposal trajectory time must strictly increase")
        previous_time = point.time_s
    first = trajectory[0]
    if not isclose(first.time_s, 0.0, abs_tol=1e-12):
        raise ValueError("proposal trajectory must start at time zero")
    if not _poses_close(first.pose, apply_end):
        raise ValueError("proposal trajectory must start at the post-apply pose")
    if any(not _command_inside_limits(point.twist, profile) for point in trajectory):
        raise ValueError("proposal trajectory twist exceeds vehicle limits")

    samples: list[TrajectoryPoint] = [first]
    for source, target in zip(trajectory, trajectory[1:], strict=False):
        duration = target.time_s - source.time_s
        steps = max(1, ceil(duration / DYNAMIC_SWEEP_SAMPLE_PERIOD_S))
        for step in range(1, steps + 1):
            fraction = step / steps
            samples.append(
                TrajectoryPoint(
                    time_s=source.time_s + fraction * duration,
                    pose=_interpolate_pose(source.pose, target.pose, fraction),
                    twist=target.twist,
                )
            )
    return tuple(samples)


def _terminal_stopping_samples(
    start: TrajectoryPoint,
    profile: VehicleProfile,
) -> tuple[TrajectoryPoint, ...]:
    samples = [TrajectoryPoint(time_s=0.0, pose=start.pose, twist=start.twist)]
    pose = start.pose
    twist = start.twist
    elapsed_s = 0.0
    while abs(twist.linear) > 1e-12 or abs(twist.angular) > 1e-12:
        pose = _integrate(pose, twist, DYNAMIC_SWEEP_SAMPLE_PERIOD_S)
        twist = _decelerate(
            twist,
            linear_delta=profile.max_deceleration_mps2 * DYNAMIC_SWEEP_SAMPLE_PERIOD_S,
            angular_delta=(DYNAMIC_ANGULAR_DECELERATION_RADPS2 * DYNAMIC_SWEEP_SAMPLE_PERIOD_S),
        )
        elapsed_s += DYNAMIC_SWEEP_SAMPLE_PERIOD_S
        samples.append(TrajectoryPoint(elapsed_s, pose, twist))
    return tuple(samples)


def _constant_command_samples(
    start: Pose2D,
    command: Twist2D,
    *,
    duration_s: float,
) -> tuple[TrajectoryPoint, ...]:
    steps = max(1, int(round(duration_s / DYNAMIC_SWEEP_SAMPLE_PERIOD_S)))
    actual_dt_s = duration_s / steps
    samples = [TrajectoryPoint(0.0, start, command)]
    pose = start
    for step in range(1, steps + 1):
        pose = _integrate(pose, command, actual_dt_s)
        samples.append(TrajectoryPoint(step * actual_dt_s, pose, command))
    return tuple(samples)


def _limited_deceleration(twist: Twist2D, profile: VehicleProfile) -> Twist2D:
    return _decelerate(
        twist,
        linear_delta=profile.max_deceleration_mps2 * DYNAMIC_CONTROL_PERIOD_S,
        angular_delta=DYNAMIC_ANGULAR_DECELERATION_RADPS2 * DYNAMIC_CONTROL_PERIOD_S,
    )


def _decelerate(
    twist: Twist2D,
    *,
    linear_delta: float,
    angular_delta: float,
) -> Twist2D:
    return Twist2D(
        linear=_toward_zero(twist.linear, linear_delta),
        angular=_toward_zero(twist.angular, angular_delta),
    )


def _toward_zero(value: float, delta: float) -> float:
    if value > 0.0:
        return max(0.0, value - delta)
    if value < 0.0:
        return min(0.0, value + delta)
    return 0.0


def _integrate(pose: Pose2D, command: Twist2D, dt_s: float) -> Pose2D:
    if abs(command.angular) <= 1e-12:
        return Pose2D(
            x=pose.x + command.linear * cos(pose.yaw) * dt_s,
            y=pose.y + command.linear * sin(pose.yaw) * dt_s,
            yaw=pose.yaw,
        )
    next_yaw = pose.yaw + command.angular * dt_s
    radius = command.linear / command.angular
    return Pose2D(
        x=pose.x + radius * (sin(next_yaw) - sin(pose.yaw)),
        y=pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
        yaw=_normalize_angle(next_yaw),
    )


def _interpolate_pose(source: Pose2D, target: Pose2D, fraction: float) -> Pose2D:
    delta_yaw = _normalize_angle(target.yaw - source.yaw)
    return Pose2D(
        x=source.x + (target.x - source.x) * fraction,
        y=source.y + (target.y - source.y) * fraction,
        yaw=_normalize_angle(source.yaw + delta_yaw * fraction),
    )


def _poses_close(first: Pose2D, second: Pose2D) -> bool:
    return all(
        isclose(left, right, rel_tol=0.0, abs_tol=1e-9)
        for left, right in (
            (first.x, second.x),
            (first.y, second.y),
            (_normalize_angle(first.yaw), _normalize_angle(second.yaw)),
        )
    )


def _command_inside_limits(command: Twist2D, profile: VehicleProfile) -> bool:
    return all(isfinite(value) for value in (command.linear, command.angular)) and (
        -profile.max_reverse_speed_mps <= command.linear <= profile.max_forward_speed_mps
        and abs(command.angular) <= profile.max_angular_speed_radps
    )


def _robot_state_is_finite(state: RobotState) -> bool:
    return all(
        isfinite(value)
        for value in (
            state.pose.x,
            state.pose.y,
            state.pose.yaw,
            state.twist.linear,
            state.twist.angular,
        )
    )


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi
