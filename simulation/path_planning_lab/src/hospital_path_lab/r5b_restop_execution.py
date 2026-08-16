"""Public Ideal two-hazard stop -> resume -> restop controller execution."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import atan2, ceil, hypot, pi

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import (
    GridSnapshot,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicCommandProposal,
    DynamicGroundTruthFrame,
    DynamicMotionState,
    Point2D,
)
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    generate_dynamic_corpus,
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
from hospital_path_lab.dynamic_prediction import ActorPredictionSet, build_actor_prediction_set
from hospital_path_lab.dynamic_safety import (
    DynamicSafetyContext,
    DynamicSafetyGate,
    build_resume_authorization,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.dynamic_witness_contracts import (
    WitnessWorldSnapshot,
    project_public_witness_world,
)
from hospital_path_lab.dynamic_witness_events import ground_truth_hazard_intervals
from hospital_path_lab.dynamic_witness_restop import (
    RestopEvidenceLevel,
    search_multi_hazard_restop,
)
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PersistentSourceDerivedDwbController,
)
from hospital_path_lab.local_reference_contracts import (
    LOCAL_REFERENCE_CONTRACT_VERSION,
    LOCAL_REFERENCE_SCHEMA_VERSION,
    REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
    LocalManeuverKind,
    LocalManeuverReference,
    ObservationDependency,
    ReferenceBuildContext,
    ReferenceEvidenceLevel,
    ReferenceKnot,
    ReferenceKnotRole,
    ReferenceSection,
    ReferenceSectionKind,
    ReferenceTravelDirection,
    ReferenceValidity,
)
from hospital_path_lab.local_reference_validation import (
    LocalReferenceValidation,
    validate_local_maneuver_reference,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.persistent_controller_contracts import PersistentControllerStatus
from hospital_path_lab.persistent_controller_pipeline import PersistentControllerPipeline
from hospital_path_lab.r5b_temporal_reference import R5B_REFERENCE_MISSION_ID
from hospital_path_lab.reference_section_executor import R5_YAW_TOLERANCE_RAD
from hospital_path_lab.spatial_oracle_contracts import (
    SpatialAllowedRegion,
    spatial_grid_content_hash,
)

R5B_RESTOP_EXECUTION_VERSION = "r5b-restop-controller-execution-v1"
R5B_RESTOP_SECOND_HAZARD_DELAY_S = 7.0
R5B_RESTOP_FIRST_RELEASE_TICK = 44
R5B_RESTOP_MINIMUM_INTERMEDIATE_PROGRESS_M = 0.10
_TOLERANCE = 1e-12


@dataclass(frozen=True, slots=True)
class R5BRestopEvidence:
    source_world_hash: str
    source_witness_hash: str
    source_validation_hash: str
    controller_world: WitnessWorldSnapshot
    first_hazard_end_s: float
    second_hazard_start_s: float
    second_hazard_end_s: float
    evidence_content_hash: str = ""

    def __post_init__(self) -> None:
        if not (
            self.first_hazard_end_s
            < self.second_hazard_start_s
            < self.second_hazard_end_s
        ):
            raise ValueError("R5-B restop hazard order is invalid")
        expected = self.expected_content_hash
        if self.evidence_content_hash and self.evidence_content_hash != expected:
            raise ValueError("R5-B restop evidence hash mismatch")
        object.__setattr__(self, "evidence_content_hash", expected)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "version": R5B_RESTOP_EXECUTION_VERSION,
                "source_world_hash": self.source_world_hash,
                "source_witness_hash": self.source_witness_hash,
                "source_validation_hash": self.source_validation_hash,
                "controller_world_hash": self.controller_world.content_hash,
                "first_hazard_end_s": self.first_hazard_end_s,
                "second_hazard_start_s": self.second_hazard_start_s,
                "second_hazard_end_s": self.second_hazard_end_s,
            }
        )


@dataclass(frozen=True, slots=True)
class R5BFollowReferenceBundle:
    build_context: ReferenceBuildContext
    reference: LocalManeuverReference
    validation: LocalReferenceValidation

    def __post_init__(self) -> None:
        if not self.validation.passed:
            raise ValueError("R5-B follow reference requires passing validation")


@dataclass(frozen=True, slots=True)
class R5BRestopExecutionResult:
    completed: bool
    first_release_tick: int | None
    first_motion_tick: int | None
    second_stop_tick: int | None
    second_stop_epoch: int | None
    second_release_tick: int | None
    completion_tick: int | None
    intermediate_progress_m: float
    minimum_actor_clearance_m: float | None
    minimum_actor_clearance_tick: int | None
    minimum_actor_binding_id: str | None
    minimum_actor_robot_pose: Pose2D | None
    minimum_static_clearance_m: float
    gate_override_count: int
    controller_session_count: int
    native_full_core_used: bool
    hard_failures: tuple[str, ...]
    trace_content_hash: str

    @property
    def passed(self) -> bool:
        return all(
            (
                self.completed,
                self.first_release_tick == R5B_RESTOP_FIRST_RELEASE_TICK,
                self.first_motion_tick is not None,
                self.second_stop_tick is not None,
                self.second_stop_epoch == 2,
                self.second_release_tick is not None,
                self.completion_tick is not None,
                self.first_motion_tick is not None
                and self.first_release_tick is not None
                and self.first_motion_tick > self.first_release_tick,
                self.second_stop_tick is not None
                and self.first_motion_tick is not None
                and self.second_stop_tick > self.first_motion_tick,
                self.second_release_tick is not None
                and self.second_stop_tick is not None
                and self.second_release_tick > self.second_stop_tick,
                self.completion_tick is not None
                and self.second_release_tick is not None
                and self.completion_tick > self.second_release_tick,
                self.intermediate_progress_m
                >= R5B_RESTOP_MINIMUM_INTERMEDIATE_PROGRESS_M - _TOLERANCE,
                self.gate_override_count == 0,
                self.controller_session_count == 2,
                self.native_full_core_used,
                not self.hard_failures,
            )
        )


def build_r5b_restop_evidence() -> R5BRestopEvidence:
    episode = next(
        item
        for item in generate_dynamic_corpus()
        if item.split is DynamicCorpusSplit.GOLDEN
        and item.expectation_category is DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP
    )
    source_world = project_public_witness_world(episode)
    source = search_multi_hazard_restop(source_world)
    if (
        source.witness is None
        or source.validation is None
        or source.validation.evidence_level
        is not RestopEvidenceLevel.RESTOP_AND_RECOVERY_PROVEN
    ):
        raise RuntimeError("frozen R2-A restop evidence is unavailable")
    hazards = ground_truth_hazard_intervals(source_world)
    first, second = hazards[:2]
    first_actor, second_actor = episode.actors[:2]
    causal_actors = (
        replace(first_actor, active_until_s=episode.duration_s),
        replace(
            second_actor,
            active_from_s=0.0,
            active_until_s=episode.duration_s,
            start_position=Point2D(
                second_actor.start_position.x
                - second_actor.velocity.x
                * (second_actor.active_from_s + R5B_RESTOP_SECOND_HAZARD_DELAY_S),
                second_actor.start_position.y
                - second_actor.velocity.y
                * (second_actor.active_from_s + R5B_RESTOP_SECOND_HAZARD_DELAY_S),
            ),
        ),
    )
    causal_episode = replace(
        episode,
        episode_id=f"{episode.episode_id}-r5b-causal-restop-v1",
        actors=causal_actors,
    )
    return R5BRestopEvidence(
        source_world_hash=source_world.content_hash,
        source_witness_hash=source.witness.semantic_content_hash,
        source_validation_hash=source.validation.content_hash,
        controller_world=project_public_witness_world(causal_episode),
        first_hazard_end_s=first.ends_at_s,
        second_hazard_start_s=(
            second.starts_at_s + R5B_RESTOP_SECOND_HAZARD_DELAY_S
        ),
        second_hazard_end_s=second.ends_at_s + R5B_RESTOP_SECOND_HAZARD_DELAY_S,
    )


def build_r5b_follow_reference(
    evidence: R5BRestopEvidence,
    *,
    current_pose: Pose2D,
    stop_epoch: int,
    valid_from_tick: int,
) -> R5BFollowReferenceBundle:
    world = evidence.controller_world
    identity = {
        "version": R5B_RESTOP_EXECUTION_VERSION,
        "evidence_hash": evidence.evidence_content_hash,
        "stop_epoch": stop_epoch,
        "valid_from_tick": valid_from_tick,
        "start_pose": current_pose,
        "goal_pose": world.goal_pose,
    }
    return build_world_follow_reference(
        world,
        mission_id=R5B_REFERENCE_MISSION_ID,
        current_pose=current_pose,
        stop_epoch=stop_epoch,
        valid_from_tick=valid_from_tick,
        identity=identity,
        generation_reason_codes=("r5b_restop_follow_original",),
    )


def build_world_follow_reference(
    world: WitnessWorldSnapshot,
    *,
    mission_id: str,
    current_pose: Pose2D,
    stop_epoch: int,
    valid_from_tick: int,
    identity: dict[str, object],
    generation_reason_codes: tuple[str, ...],
    goal_pose: Pose2D | None = None,
) -> R5BFollowReferenceBundle:
    """Build a new stop-bound spatial reference from the current pose to the goal."""

    goal = world.goal_pose if goal_pose is None else goal_pose
    grid = world.grid.to_grid_map()
    forbidden = tuple(sorted(world.grid.forbidden_cells))
    snapshot = GridSnapshot(
        metadata=world_grid_metadata(world),
        grid=grid,
        forbidden_cells=frozenset(forbidden),
    )
    allowed = SpatialAllowedRegion()
    context = ReferenceBuildContext(
        schema_version=REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
        mission_id=mission_id,
        stop_epoch=stop_epoch,
        map_id=world.map_id,
        map_revision=world.map_revision,
        mission_revision=0,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        static_grid_snapshot=snapshot,
        grid_content_hash=spatial_grid_content_hash(grid),
        allowed_region=allowed,
        allowed_region_hash=allowed.content_hash,
        forbidden_cells=forbidden,
        forbidden_region_hash=canonical_content_hash(forbidden),
        vehicle_profile=world.kinematic_contract.vehicle_profile,
        vehicle_profile_hash=canonical_content_hash(world.kinematic_contract.vehicle_profile),
        original_reference=(current_pose, goal),
        original_reference_hash=canonical_content_hash((current_pose, goal)),
        current_robot_pose=current_pose,
        control_tick=valid_from_tick,
        simulation_time_s=valid_from_tick * DYNAMIC_CONTROL_PERIOD_S,
    )
    delta_x = goal.x - current_pose.x
    delta_y = goal.y - current_pose.y
    length = hypot(delta_x, delta_y)
    travel_yaw = atan2(delta_y, delta_x) if length > 1e-12 else current_pose.yaw
    terminal_yaw_error = abs((goal.yaw - travel_yaw + pi) % (2.0 * pi) - pi)
    needs_terminal_rotation = (
        length > 1e-12 and terminal_yaw_error > R5_YAW_TOLERANCE_RAD
    )
    translation_goal = (
        Pose2D(goal.x, goal.y, travel_yaw) if needs_terminal_rotation else goal
    )
    translation_terminal_roles = (
        (
            ReferenceKnotRole.ANCHOR,
            ReferenceKnotRole.TRANSLATION,
            ReferenceKnotRole.STOP_MARKER,
        )
        if needs_terminal_rotation
        else (
            ReferenceKnotRole.ANCHOR,
            ReferenceKnotRole.TRANSLATION,
            ReferenceKnotRole.REJOIN,
            ReferenceKnotRole.STOP_MARKER,
        )
    )
    knots: tuple[ReferenceKnot, ...] = (
        ReferenceKnot(
            knot_index=0,
            pose=current_pose,
            tangent_yaw=current_pose.yaw,
            cumulative_translation_arc_m=0.0,
            source_path_index=0,
            section_index=0,
            knot_roles=(ReferenceKnotRole.ANCHOR, ReferenceKnotRole.TRANSLATION),
        ),
        ReferenceKnot(
            knot_index=1,
            pose=translation_goal,
            tangent_yaw=translation_goal.yaw,
            cumulative_translation_arc_m=length,
            source_path_index=1,
            section_index=0,
            knot_roles=translation_terminal_roles,
        ),
    )
    sections: tuple[ReferenceSection, ...] = (
        ReferenceSection(
            section_index=0,
            section_kind=ReferenceSectionKind.FOLLOW_ORIGINAL,
            travel_direction=ReferenceTravelDirection.FORWARD,
            first_knot_index=0,
            last_knot_index=1,
            entry_requires_stopped=False,
            exit_requires_stopped=needs_terminal_rotation,
            source_primitive_indices=(),
        ),
    )
    if needs_terminal_rotation:
        knots += (
            ReferenceKnot(
                knot_index=2,
                pose=translation_goal,
                tangent_yaw=translation_goal.yaw,
                cumulative_translation_arc_m=length,
                source_path_index=2,
                section_index=1,
                knot_roles=(
                    ReferenceKnotRole.ANCHOR,
                    ReferenceKnotRole.ROTATION_ENTRY,
                    ReferenceKnotRole.STOP_MARKER,
                ),
            ),
            ReferenceKnot(
                knot_index=3,
                pose=goal,
                tangent_yaw=goal.yaw,
                cumulative_translation_arc_m=length,
                source_path_index=3,
                section_index=1,
                knot_roles=(
                    ReferenceKnotRole.ANCHOR,
                    ReferenceKnotRole.ROTATION_EXIT,
                    ReferenceKnotRole.REJOIN,
                    ReferenceKnotRole.STOP_MARKER,
                ),
            ),
        )
        sections += (
            ReferenceSection(
                section_index=1,
                section_kind=ReferenceSectionKind.ROTATE,
                travel_direction=ReferenceTravelDirection.NONE,
                first_knot_index=2,
                last_knot_index=3,
                entry_requires_stopped=True,
                exit_requires_stopped=True,
                source_primitive_indices=(),
            ),
        )
    reference = LocalManeuverReference(
        schema_version=LOCAL_REFERENCE_SCHEMA_VERSION,
        reference_contract_version=LOCAL_REFERENCE_CONTRACT_VERSION,
        candidate_id=canonical_content_hash({"r5b_follow_candidate": identity}),
        maneuver_kind=LocalManeuverKind.FOLLOW_ORIGINAL,
        evidence_level=ReferenceEvidenceLevel.SPATIAL_ONLY,
        mission_id=context.mission_id,
        stop_epoch=stop_epoch,
        map_id=context.map_id,
        map_revision=context.map_revision,
        mission_revision=context.mission_revision,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        maneuver_revision=stop_epoch,
        path_revision=1,
        reference_session_id=canonical_content_hash({"r5b_follow_session": identity}),
        source_spatial_seed_hash=None,
        source_temporal_evidence_hash=None,
        original_reference_hash=context.original_reference_hash,
        grid_content_hash=context.grid_content_hash,
        vehicle_profile_hash=context.vehicle_profile_hash,
        allowed_region_hash=context.allowed_region_hash,
        forbidden_region_hash=context.forbidden_region_hash,
        knots=knots,
        sections=sections,
        departure_knot_index=None,
        pass_section_index=None,
        rejoin_knot_index=len(knots) - 1,
        minimum_validated_static_clearance_m=0.08,
        validity=ReferenceValidity(
            required_mission_id=context.mission_id,
            required_stop_epoch=stop_epoch,
            required_map_revision=context.map_revision,
            required_mission_revision=context.mission_revision,
            required_observation_revision=None,
            valid_from_control_tick=valid_from_tick,
            valid_until_control_tick=None,
        ),
        generation_reason_codes=generation_reason_codes,
        limitations=("ideal_path_only", "public_simulation_only", "no_perception_claim"),
    )
    validation = validate_local_maneuver_reference(context, reference)
    return R5BFollowReferenceBundle(context, reference, validation)


class _RestopIdealStream:
    def __init__(self, evidence: R5BRestopEvidence, *, tick_limit: int) -> None:
        world = evidence.controller_world
        self._source = DynamicObservationSourceIdentity(
            stream_id="r5b-restop-ideal-public",
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
                mission_revision=0,
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
    ) -> tuple[DynamicObservationSnapshot, ActorPredictionSet | None, DirectionalPredictionResult]:
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
                result = self._validator.accept(
                    slot.frame,
                    received_at_s=slot.scheduled_delivery_at_s,
                )
                if not result.accepted:
                    raise RuntimeError("R5-B restop Ideal frame failed validation")
            self._next_slot += 1
        snapshot = self._validator.snapshot(control_time_s=time_s)
        circular = build_actor_prediction_set(snapshot) if snapshot.usable else None
        return snapshot, circular, self._directional.update(snapshot)


def run_r5b_restop_case(*, tick_limit: int = 700) -> R5BRestopExecutionResult:
    evidence = build_r5b_restop_evidence()
    world = evidence.controller_world
    if tick_limit > round(world.duration_s / DYNAMIC_CONTROL_PERIOD_S):
        raise ValueError("R5-B restop tick limit exceeds the public world duration")
    stream = _RestopIdealStream(evidence, tick_limit=tick_limit)
    gate = DynamicSafetyGate(
        profile=world.kinematic_contract.vehicle_profile,
        initial_stop_epoch=0,
    )
    state = world.initial_state
    first_release_tick: int | None = None
    first_motion_tick: int | None = None
    second_stop_tick: int | None = None
    second_release_tick: int | None = None
    completion_tick: int | None = None
    second_stop_pose: Pose2D | None = None
    minimum_actor: float | None = None
    minimum_actor_tick: int | None = None
    minimum_actor_binding_id: str | None = None
    minimum_actor_robot_pose: Pose2D | None = None
    minimum_static = float("inf")
    failures: list[str] = []
    trace: list[object] = []
    controllers: list[PersistentSourceDerivedDwbController] = []
    pipeline: PersistentControllerPipeline | None = None
    resume = None
    checker = CollisionChecker(
        world.grid.to_grid_map(),
        world.kinematic_contract.vehicle_profile,
        forbidden_cells=world.grid.forbidden_cells,
    )
    second_release_not_before = ceil(
        evidence.second_hazard_end_s / DYNAMIC_CONTROL_PERIOD_S
    ) + 1

    for tick in range(tick_limit):
        snapshot, circular, directional = stream.tick(tick)
        if pipeline is None:
            if tick < R5B_RESTOP_FIRST_RELEASE_TICK:
                decision = _hold_step(
                    evidence,
                    gate=gate,
                    robot_state=state,
                    tick=tick,
                    snapshot=snapshot,
                    prediction_set=circular,
                )
                state = RobotState(state.pose, decision.command)
                trace.append((tick, gate.motion_state, gate.stop_epoch, state))
                continue
            if directional.status is not DirectionalPredictionStatus.READY:
                failures.append(f"directional_prediction_not_ready:{tick}")
                break
            if gate.motion_state is not DynamicMotionState.HOLDING or gate.stop_epoch != 1:
                failures.append("first_stop_not_confirmed")
                break
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
            controllers.append(controller)
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
                mission_id=bundle.reference.mission_id,
                stop_epoch=gate.stop_epoch,
                issued_or_revalidated_at_s=tick * DYNAMIC_CONTROL_PERIOD_S,
                authorization_revision=gate.stop_epoch,
            )
            first_release_tick = tick

        if (
            gate.motion_state is DynamicMotionState.HOLDING
            and gate.stop_epoch == 2
            and tick >= second_release_not_before
        ):
            state = pipeline.robot_state
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
            controllers.append(controller)
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
                mission_id=bundle.reference.mission_id,
                stop_epoch=gate.stop_epoch,
                issued_or_revalidated_at_s=tick * DYNAMIC_CONTROL_PERIOD_S,
                authorization_revision=gate.stop_epoch,
            )
            second_release_tick = tick

        if directional.prediction_set is None:
            failures.append(f"directional_prediction_not_ready:{tick}")
            break
        current_bundle = bundle
        record = pipeline.step(
            observation_snapshot=snapshot,
            prediction_set=directional.prediction_set,
            resume_authorization=resume,
            grid_snapshot=_grid_for_observation(current_bundle, snapshot),
        )
        resume = None
        state = record.robot_state_after
        result = record.controller_result
        if result is not None and result.status in {
            PersistentControllerStatus.INVALID_REFERENCE_INPUT,
            PersistentControllerStatus.STALE_REFERENCE_INPUT,
            PersistentControllerStatus.LATE_RESULT,
            PersistentControllerStatus.SECTION_EXECUTION_FAILED,
        }:
            failures.append(f"controller:{tick}:{result.status.value}:{result.failure_reason}")
            break
        if first_motion_tick is None and record.safety_decision.command != Twist2D():
            first_motion_tick = tick
        if (
            gate.motion_state is DynamicMotionState.HOLDING
            and gate.stop_epoch == 2
            and second_stop_tick is None
        ):
            second_stop_tick = tick
            second_stop_pose = state.pose
        minimum_static = min(minimum_static, checker.clearance(state.pose))
        for actor in world.actor_states_at((tick + 1) * DYNAMIC_CONTROL_PERIOD_S):
            clearance = oriented_footprint_circle_surface_distance(
                state.pose,
                circle_center=(actor.position.x, actor.position.y),
                circle_radius_m=actor.radius_m,
                profile=world.kinematic_contract.vehicle_profile,
            )
            if minimum_actor is None or clearance < minimum_actor:
                minimum_actor = clearance
                minimum_actor_tick = tick
                minimum_actor_binding_id = actor.actor_id
                minimum_actor_robot_pose = state.pose
        trace.append(
            (
                tick,
                gate.motion_state,
                gate.stop_epoch,
                state,
                None if result is None else result.semantic_content_hash,
                record.safety_decision,
            )
        )
        if record.safety_decision.motion_state is DynamicMotionState.COMPLETED:
            completion_tick = tick
            break

    if pipeline is not None:
        state = pipeline.robot_state
    intermediate_progress = (
        0.0
        if second_stop_pose is None
        else hypot(
            second_stop_pose.x - world.initial_state.pose.x,
            second_stop_pose.y - world.initial_state.pose.y,
        )
    )
    if second_stop_tick is None:
        failures.append("second_distinct_stop_not_observed")
    if second_release_tick is None:
        failures.append("second_release_not_observed")
    if intermediate_progress < R5B_RESTOP_MINIMUM_INTERMEDIATE_PROGRESS_M - _TOLERANCE:
        failures.append("intermediate_progress_insufficient")
    if completion_tick is None:
        failures.append("mission_completion_not_observed")
    if minimum_static < world.kinematic_contract.vehicle_profile.minimum_clearance_m - _TOLERANCE:
        failures.append("actual_static_clearance_below_minimum")
    if (
        minimum_actor is not None
        and minimum_actor
        < world.kinematic_contract.vehicle_profile.minimum_clearance_m - _TOLERANCE
    ):
        failures.append("actual_actor_clearance_below_minimum")
    return R5BRestopExecutionResult(
        completed=completion_tick is not None,
        first_release_tick=first_release_tick,
        first_motion_tick=first_motion_tick,
        second_stop_tick=second_stop_tick,
        second_stop_epoch=(2 if second_stop_tick is not None else None),
        second_release_tick=second_release_tick,
        completion_tick=completion_tick,
        intermediate_progress_m=intermediate_progress,
        minimum_actor_clearance_m=minimum_actor,
        minimum_actor_clearance_tick=minimum_actor_tick,
        minimum_actor_binding_id=minimum_actor_binding_id,
        minimum_actor_robot_pose=minimum_actor_robot_pose,
        minimum_static_clearance_m=minimum_static,
        gate_override_count=gate.counters.gate_overrides,
        controller_session_count=len(controllers),
        native_full_core_used=bool(controllers)
        and all(controller.native_full_core_used for controller in controllers),
        hard_failures=tuple(dict.fromkeys(failures)),
        trace_content_hash=canonical_content_hash(tuple(trace)),
    )


def world_grid_metadata(world: WitnessWorldSnapshot) -> SnapshotMetadata:
    return SnapshotMetadata(
        map_id=world.map_id,
        map_revision=world.map_revision,
        mission_revision=0,
        observation_revision=0,
        seed=world.seed,
        content_hash=world.grid_content_hash,
        input_valid=True,
    )


def _grid_for_observation(
    bundle: R5BFollowReferenceBundle,
    snapshot: DynamicObservationSnapshot,
) -> GridSnapshot:
    frozen = bundle.build_context.static_grid_snapshot
    revision = (
        frozen.metadata.observation_revision
        if snapshot.frame is None
        else snapshot.frame.observation_revision
    )
    return GridSnapshot(
        metadata=replace(frozen.metadata, observation_revision=revision),
        grid=frozen.grid,
        forbidden_cells=frozen.forbidden_cells,
    )


def _hold_step(
    evidence: R5BRestopEvidence,
    *,
    gate: DynamicSafetyGate,
    robot_state: RobotState,
    tick: int,
    snapshot: DynamicObservationSnapshot,
    prediction_set: ActorPredictionSet | None,
):
    world = evidence.controller_world
    grid = GridSnapshot(
        metadata=replace(
            world_grid_metadata(world),
            observation_revision=(
                0 if snapshot.frame is None else snapshot.frame.observation_revision
            ),
        ),
        grid=world.grid.to_grid_map(),
        forbidden_cells=world.grid.forbidden_cells,
    )
    frame = snapshot.frame
    proposal = DynamicCommandProposal(
        source_tick_id=tick,
        command=Twist2D(),
        computation_time_s=DYNAMIC_CONTROL_PERIOD_S,
        mission_id=R5B_REFERENCE_MISSION_ID,
        map_id=world.map_id,
        map_revision=world.map_revision,
        mission_revision=0,
        observation_revision=grid.metadata.observation_revision,
        grid_content_hash=grid.metadata.content_hash,
        observation_content_hash=(
            "observation-unavailable" if frame is None else frame.content_hash
        ),
    )
    context = DynamicSafetyContext(
        tick_id=tick,
        simulation_time_s=tick * DYNAMIC_CONTROL_PERIOD_S,
        mission_id=R5B_REFERENCE_MISSION_ID,
        authorization_revision=1,
        grid_snapshot=grid,
        observation_snapshot=snapshot,
        prediction_set=prediction_set,
        path_still_valid=True,
        local_safety_recheck_passed=True,
        observation_safe=True,
    )
    return gate.step(proposal, robot_state=robot_state, context=context)


__all__ = [
    "R5B_RESTOP_EXECUTION_VERSION",
    "R5B_RESTOP_FIRST_RELEASE_TICK",
    "R5BRestopEvidence",
    "R5BRestopExecutionResult",
    "build_r5b_follow_reference",
    "build_world_follow_reference",
    "build_r5b_restop_evidence",
    "run_r5b_restop_case",
]
