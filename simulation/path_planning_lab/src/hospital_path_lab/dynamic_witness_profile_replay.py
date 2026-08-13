"""R2 public witness replay across Ideal, Normal and Stress observations.

This module is an offline, label-free research harness.  It does not run a
controller, issue motion authority, or consume evaluator categories/oracles.
"""

from __future__ import annotations

from bisect import bisect_right
from collections import Counter
from dataclasses import dataclass
from math import ceil, cos, hypot, isclose, isfinite, sin

from hospital_path_lab.collision import oriented_footprint_capsule_surface_distance
from hospital_path_lab.contracts import Pose2D, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_COMMAND_APPLY_LATENCY_S,
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicGroundTruthFrame,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DIRECTIONAL_PREDICTION_VERSION,
    FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS,
    DirectionalActorPredictor,
    DirectionalPredictionParameters,
    DirectionalPredictionResult,
    DirectionalPredictionStatus,
    sample_directional_capsules,
)
from hospital_path_lab.dynamic_observation import (
    DYNAMIC_OBSERVATION_GENERATOR_VERSION,
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationDropKind,
    DynamicObservationProfile,
    DynamicObservationProfileName,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    generate_dynamic_observation_slots,
)
from hospital_path_lab.dynamic_witness_contracts import (
    AutomatedWitness,
    WitnessKind,
    WitnessPhase,
    WitnessPoint,
    WitnessWorldSnapshot,
    build_automated_witness,
)
from hospital_path_lab.dynamic_witness_validation import (
    GroundTruthWitnessValidation,
    validate_ground_truth_witness,
)
from hospital_path_lab.map_factory import canonical_content_hash

WITNESS_PROFILE_REPLAY_SCHEMA_VERSION = "witness-profile-replay-v1"
WITNESS_PROFILE_REPLAY_VERSION = "witness-profile-replay-2026-08-13"
_REPLAY_STREAM_ID = "dynamic-witness-profile-replay"
_MISSION_REVISION = 0
_TIME_TOLERANCE_S = 1e-12
_GEOMETRY_TOLERANCE_M = 1e-9
_SUPPORTED_PROFILES = (
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
)
_USABLE_STATUSES = frozenset(
    (DirectionalPredictionStatus.READY, DirectionalPredictionStatus.EMPTY_FRAME)
)
_PASS_KINDS = frozenset((WitnessKind.PASS_LEFT, WitnessKind.PASS_RIGHT))


@dataclass(frozen=True, slots=True)
class PredictionStatusInterval:
    status: DirectionalPredictionStatus
    start_tick: int
    end_tick: int
    hold_required: bool

    def __post_init__(self) -> None:
        if not isinstance(self.status, DirectionalPredictionStatus):
            raise TypeError("interval status must be a DirectionalPredictionStatus")
        if type(self.start_tick) is not int or type(self.end_tick) is not int:
            raise TypeError("interval ticks must be exact integers")
        if self.start_tick < 0 or self.end_tick < self.start_tick:
            raise ValueError("interval ticks must be non-negative and ordered")
        if type(self.hold_required) is not bool:
            raise TypeError("interval hold_required must be bool")

    @property
    def tick_count(self) -> int:
        return self.end_tick - self.start_tick + 1

    @property
    def start_time_s(self) -> float:
        return self.start_tick * DYNAMIC_CONTROL_PERIOD_S

    @property
    def end_time_s(self) -> float:
        return self.end_tick * DYNAMIC_CONTROL_PERIOD_S


@dataclass(frozen=True, slots=True)
class WitnessProfileReplayResult:
    schema_version: str
    replay_version: str
    source_projection_hash: str
    world_content_hash: str
    witness_content_hash: str
    ground_truth_validation_hash: str
    observation_profile_name: DynamicObservationProfileName
    observation_profile_hash: str
    observation_generator_version: str
    prediction_model_version: str
    prediction_parameter_hash: str
    slot_count: int
    delivered_frame_count: int
    dropout_count: int
    status_counts: tuple[tuple[str, int], ...]
    status_intervals: tuple[PredictionStatusInterval, ...]
    first_ready_tick: int | None
    first_ready_time_s: float | None
    observation_decidable: bool
    delayed_witness: AutomatedWitness | None
    delayed_validation_hash: str | None
    delayed_validation_failures: tuple[str, ...]
    shifted_completion_within_episode: bool
    delayed_ground_truth_valid: bool
    observation_continuous_for_witness: bool
    evaluated_motion_tick_count: int
    unavailable_motion_tick_count: int
    capsule_sample_count: int
    predicted_clearance_violation_count: int
    minimum_predicted_clearance_m: float | None
    actual_actor_containment_sample_count: int
    actual_actor_containment_miss_count: int
    maximum_actor_containment_miss_m: float
    capsule_geometry_admissible_when_observed: bool
    prediction_admissible: bool
    hard_failures: tuple[str, ...]
    limitations: tuple[str, ...]

    def __post_init__(self) -> None:
        if self.schema_version != WITNESS_PROFILE_REPLAY_SCHEMA_VERSION:
            raise ValueError("unsupported witness profile replay schema")
        if self.replay_version != WITNESS_PROFILE_REPLAY_VERSION:
            raise ValueError("unsupported witness profile replay version")
        identity = (
            self.source_projection_hash,
            self.world_content_hash,
            self.witness_content_hash,
            self.ground_truth_validation_hash,
            self.observation_profile_hash,
            self.observation_generator_version,
            self.prediction_model_version,
            self.prediction_parameter_hash,
        )
        if not all(identity):
            raise ValueError("profile replay identity fields must not be empty")
        counts = (
            self.slot_count,
            self.delivered_frame_count,
            self.dropout_count,
            self.evaluated_motion_tick_count,
            self.unavailable_motion_tick_count,
            self.capsule_sample_count,
            self.predicted_clearance_violation_count,
            self.actual_actor_containment_sample_count,
            self.actual_actor_containment_miss_count,
        )
        if any(type(value) is not int or value < 0 for value in counts):
            raise ValueError("profile replay counts must be non-negative integers")
        if self.delivered_frame_count + self.dropout_count != self.slot_count:
            raise ValueError("profile replay slot counts are inconsistent")
        if self.first_ready_tick is None:
            if self.first_ready_time_s is not None or self.observation_decidable:
                raise ValueError("missing READY tick must be observation-undecidable")
        else:
            if type(self.first_ready_tick) is not int or self.first_ready_tick < 0:
                raise ValueError("first_ready_tick must be a non-negative integer")
            expected = self.first_ready_tick * DYNAMIC_CONTROL_PERIOD_S
            if self.first_ready_time_s is None or not isclose(
                self.first_ready_time_s,
                expected,
                rel_tol=0.0,
                abs_tol=_TIME_TOLERANCE_S,
            ):
                raise ValueError("first READY time must derive from its 20 Hz tick")
            if not self.observation_decidable:
                raise ValueError("a READY tick must make direction observation decidable")
        if (self.delayed_witness is None) != (self.delayed_validation_hash is None):
            raise ValueError("delayed witness and validation provenance are inconsistent")
        if self.delayed_ground_truth_valid and self.delayed_witness is None:
            raise ValueError("valid delayed ground truth requires a delayed witness")
        for value in (
            self.maximum_actor_containment_miss_m,
            self.minimum_predicted_clearance_m,
        ):
            if value is not None and not isfinite(value):
                raise ValueError("profile replay geometry metrics must be finite")
        if self.maximum_actor_containment_miss_m < 0.0:
            raise ValueError("containment miss distance must not be negative")
        object.__setattr__(self, "status_counts", tuple(self.status_counts))
        object.__setattr__(self, "status_intervals", tuple(self.status_intervals))
        object.__setattr__(
            self,
            "delayed_validation_failures",
            tuple(dict.fromkeys(self.delayed_validation_failures)),
        )
        object.__setattr__(self, "hard_failures", tuple(sorted(set(self.hard_failures))))
        object.__setattr__(self, "limitations", tuple(sorted(set(self.limitations))))

    @property
    def semantic_content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class WitnessProfileReplayBundle:
    schema_version: str
    replay_version: str
    source_projection_hash: str
    world_content_hash: str
    witness_content_hash: str
    ground_truth_validation_hash: str
    results: tuple[WitnessProfileReplayResult, ...]
    limitations: tuple[str, ...]

    def __post_init__(self) -> None:
        if self.schema_version != WITNESS_PROFILE_REPLAY_SCHEMA_VERSION:
            raise ValueError("unsupported witness profile replay bundle schema")
        if self.replay_version != WITNESS_PROFILE_REPLAY_VERSION:
            raise ValueError("unsupported witness profile replay bundle version")
        expected = tuple(profile.name for profile in _SUPPORTED_PROFILES)
        actual = tuple(result.observation_profile_name for result in self.results)
        if actual != expected:
            raise ValueError("profile replay bundle order must be Ideal, Normal, Stress")
        if any(
            (
                result.source_projection_hash != self.source_projection_hash
                or result.world_content_hash != self.world_content_hash
                or result.witness_content_hash != self.witness_content_hash
                or result.ground_truth_validation_hash != self.ground_truth_validation_hash
            )
            for result in self.results
        ):
            raise ValueError("profile replay bundle provenance does not match its results")
        object.__setattr__(self, "results", tuple(self.results))
        object.__setattr__(self, "limitations", tuple(sorted(set(self.limitations))))

    @property
    def semantic_content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class _PredictionTick:
    tick_id: int
    result: DirectionalPredictionResult


@dataclass(frozen=True, slots=True)
class _CapsuleReplayMetrics:
    evaluated_motion_ticks: int = 0
    unavailable_motion_ticks: int = 0
    capsule_samples: int = 0
    clearance_violations: int = 0
    minimum_clearance_m: float | None = None
    containment_samples: int = 0
    containment_misses: int = 0
    maximum_containment_miss_m: float = 0.0


def replay_witness_profiles(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    ground_truth_validation: GroundTruthWitnessValidation,
    *,
    prediction_parameters: DirectionalPredictionParameters = (
        FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS
    ),
) -> WitnessProfileReplayBundle:
    """Replay one validated public witness in the frozen three-profile order."""

    _validate_replay_input(
        world,
        witness,
        ground_truth_validation,
        prediction_parameters,
    )
    results = tuple(
        replay_witness_profile(
            world,
            witness,
            ground_truth_validation,
            profile,
            prediction_parameters=prediction_parameters,
        )
        for profile in _SUPPORTED_PROFILES
    )
    limitations = {
        "online_controller_and_gate_not_evaluated",
        "simulation_only_open_loop_circular_actor",
    }
    limitations.update(reason for result in results for reason in result.limitations)
    return WitnessProfileReplayBundle(
        schema_version=WITNESS_PROFILE_REPLAY_SCHEMA_VERSION,
        replay_version=WITNESS_PROFILE_REPLAY_VERSION,
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        witness_content_hash=witness.semantic_content_hash,
        ground_truth_validation_hash=ground_truth_validation.content_hash,
        results=results,
        limitations=tuple(sorted(limitations)),
    )


def replay_witness_profile(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    ground_truth_validation: GroundTruthWitnessValidation,
    profile: DynamicObservationProfile,
    *,
    prediction_parameters: DirectionalPredictionParameters = (
        FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS
    ),
) -> WitnessProfileReplayResult:
    """Replay one ground-truth witness under one frozen observation profile."""

    _validate_replay_input(
        world,
        witness,
        ground_truth_validation,
        prediction_parameters,
    )
    if profile not in _SUPPORTED_PROFILES:
        raise ValueError("witness profile replay accepts only frozen Ideal/Normal/Stress")

    source = _source_identity(world)
    slots = generate_dynamic_observation_slots(
        _ground_truth_frames(world),
        source=source,
        profile=profile,
    )
    ticks = _prediction_ticks(world, source, profile, slots)
    first_ready = next(
        (
            tick.tick_id
            for tick in ticks
            if tick.result.status is DirectionalPredictionStatus.READY
        ),
        None,
    )
    observation_decidable = first_ready is not None and bool(world.actors)
    first_ready_time_s = (
        first_ready * DYNAMIC_CONTROL_PERIOD_S if observation_decidable else None
    )

    limitations = {
        "online_controller_and_gate_not_evaluated",
        "simulation_only_open_loop_circular_actor",
    }
    hard_failures: set[str] = set()
    delayed_witness: AutomatedWitness | None = None
    delayed_validation: GroundTruthWitnessValidation | None = None
    shifted_completion = False
    delayed_ground_truth_valid = False
    if observation_decidable:
        assert first_ready is not None
        delayed_witness, shifted_completion = _delayed_witness(
            world,
            witness,
            profile,
            first_ready,
        )
        if delayed_witness is None:
            limitations.add(
                "delayed_witness_exceeds_episode"
                if not shifted_completion
                else "delayed_witness_unsupported_nonzero_initial_twist"
            )
        else:
            delayed_validation = validate_ground_truth_witness(
                world,
                delayed_witness,
                strict_declarations=delayed_witness.kind in _PASS_KINDS,
            )
            delayed_ground_truth_valid = delayed_validation.passed
            if not delayed_ground_truth_valid:
                limitations.add("delayed_ground_truth_invalid")
    else:
        limitations.add("no_ready_prediction")

    metrics = _CapsuleReplayMetrics()
    if delayed_witness is not None and shifted_completion:
        metrics = _replay_capsules(world, delayed_witness, ticks)
    observation_continuous = (
        metrics.evaluated_motion_ticks > 0 and metrics.unavailable_motion_ticks == 0
    )
    if metrics.unavailable_motion_ticks:
        limitations.add("observation_interrupted_during_witness")
    capsule_admissible = (
        metrics.capsule_samples > 0 and metrics.clearance_violations == 0
    )
    if metrics.clearance_violations:
        limitations.add("predicted_clearance_rejected")
    if metrics.containment_misses:
        if profile.name is DynamicObservationProfileName.FUNCTIONAL_IDEAL:
            hard_failures.add("ideal_capsule_ground_truth_miss")
        else:
            limitations.add("normal_or_stress_capsule_containment_miss")
    if profile.name is DynamicObservationProfileName.FUNCTIONAL_IDEAL and any(
        slot.drop_kind is not DynamicObservationDropKind.NONE for slot in slots
    ):
        hard_failures.add("ideal_observation_dropout")

    prediction_admissible = all(
        (
            observation_decidable,
            shifted_completion,
            delayed_ground_truth_valid,
            observation_continuous,
            capsule_admissible,
            not hard_failures,
        )
    )
    counts = Counter(tick.result.status.value for tick in ticks)
    status_counts = tuple(sorted(counts.items()))
    return WitnessProfileReplayResult(
        schema_version=WITNESS_PROFILE_REPLAY_SCHEMA_VERSION,
        replay_version=WITNESS_PROFILE_REPLAY_VERSION,
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        witness_content_hash=witness.semantic_content_hash,
        ground_truth_validation_hash=ground_truth_validation.content_hash,
        observation_profile_name=profile.name,
        observation_profile_hash=canonical_content_hash(profile),
        observation_generator_version=DYNAMIC_OBSERVATION_GENERATOR_VERSION,
        prediction_model_version=DIRECTIONAL_PREDICTION_VERSION,
        prediction_parameter_hash=canonical_content_hash(prediction_parameters),
        slot_count=len(slots),
        delivered_frame_count=sum(slot.delivered for slot in slots),
        dropout_count=sum(not slot.delivered for slot in slots),
        status_counts=status_counts,
        status_intervals=_status_intervals(ticks),
        first_ready_tick=first_ready if observation_decidable else None,
        first_ready_time_s=first_ready_time_s,
        observation_decidable=observation_decidable,
        delayed_witness=delayed_witness,
        delayed_validation_hash=(
            delayed_validation.content_hash if delayed_validation is not None else None
        ),
        delayed_validation_failures=(
            delayed_validation.failures if delayed_validation is not None else ()
        ),
        shifted_completion_within_episode=shifted_completion,
        delayed_ground_truth_valid=delayed_ground_truth_valid,
        observation_continuous_for_witness=observation_continuous,
        evaluated_motion_tick_count=metrics.evaluated_motion_ticks,
        unavailable_motion_tick_count=metrics.unavailable_motion_ticks,
        capsule_sample_count=metrics.capsule_samples,
        predicted_clearance_violation_count=metrics.clearance_violations,
        minimum_predicted_clearance_m=metrics.minimum_clearance_m,
        actual_actor_containment_sample_count=metrics.containment_samples,
        actual_actor_containment_miss_count=metrics.containment_misses,
        maximum_actor_containment_miss_m=metrics.maximum_containment_miss_m,
        capsule_geometry_admissible_when_observed=capsule_admissible,
        prediction_admissible=prediction_admissible,
        hard_failures=tuple(sorted(hard_failures)),
        limitations=tuple(sorted(limitations)),
    )


def _validate_replay_input(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    validation: GroundTruthWitnessValidation,
    parameters: DirectionalPredictionParameters,
) -> None:
    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(witness, AutomatedWitness):
        raise TypeError("witness must be an AutomatedWitness")
    if not isinstance(validation, GroundTruthWitnessValidation):
        raise TypeError("ground_truth_validation must be a GroundTruthWitnessValidation")
    if parameters != FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS:
        raise ValueError("witness replay requires frozen directional prediction parameters")
    expected = validate_ground_truth_witness(
        world,
        witness,
        strict_declarations=witness.kind in _PASS_KINDS,
    )
    if not validation.passed or validation.content_hash != expected.content_hash:
        raise ValueError("witness replay requires the exact passing ground-truth validation")


def _source_identity(world: WitnessWorldSnapshot) -> DynamicObservationSourceIdentity:
    return DynamicObservationSourceIdentity(
        stream_id=_REPLAY_STREAM_ID,
        episode_id=world.world_id,
        episode_seed=world.seed,
        map_id=world.map_id,
        map_revision=world.map_revision,
    )


def _ground_truth_frames(
    world: WitnessWorldSnapshot,
) -> tuple[DynamicGroundTruthFrame, ...]:
    tick_count = round(world.duration_s / DYNAMIC_CONTROL_PERIOD_S)
    if not isclose(
        tick_count * DYNAMIC_CONTROL_PERIOD_S,
        world.duration_s,
        rel_tol=0.0,
        abs_tol=_TIME_TOLERANCE_S,
    ):
        raise ValueError("witness world duration must align with the 20 Hz clock")
    return tuple(
        DynamicGroundTruthFrame(
            episode_id=world.world_id,
            seed=world.seed,
            tick_id=tick_id,
            simulation_time_s=tick_id * DYNAMIC_CONTROL_PERIOD_S,
            robot_state=world.initial_state,
            actors=world.actor_states_at(tick_id * DYNAMIC_CONTROL_PERIOD_S),
            map_revision=world.map_revision,
            mission_revision=_MISSION_REVISION,
        )
        for tick_id in range(tick_count + 1)
    )


def _prediction_ticks(
    world: WitnessWorldSnapshot,
    source: DynamicObservationSourceIdentity,
    profile: DynamicObservationProfile,
    slots,
) -> tuple[_PredictionTick, ...]:
    validator = DynamicObservationValidator(source, profile)
    predictor = DirectionalActorPredictor()
    next_slot = 0
    tick_count = round(world.duration_s / DYNAMIC_CONTROL_PERIOD_S)
    results: list[_PredictionTick] = []
    for tick_id in range(tick_count + 1):
        control_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        while (
            next_slot < len(slots)
            and slots[next_slot].scheduled_delivery_at_s
            <= control_time_s + _TIME_TOLERANCE_S
        ):
            slot = slots[next_slot]
            if slot.frame is None:
                validator.record_no_frame(
                    sequence=slot.sequence,
                    delivery_time_s=slot.scheduled_delivery_at_s,
                )
            else:
                accepted = validator.accept(
                    slot.frame,
                    received_at_s=slot.scheduled_delivery_at_s,
                )
                if not accepted.accepted:
                    raise ValueError("generated replay observation failed source validation")
            next_slot += 1
        prediction = predictor.update(validator.snapshot(control_time_s=control_time_s))
        results.append(_PredictionTick(tick_id=tick_id, result=prediction))
    return tuple(results)


def _status_intervals(
    ticks: tuple[_PredictionTick, ...],
) -> tuple[PredictionStatusInterval, ...]:
    if not ticks:
        return ()
    intervals: list[PredictionStatusInterval] = []
    start = ticks[0]
    previous = ticks[0]
    for tick in ticks[1:]:
        if (
            tick.result.status is previous.result.status
            and tick.result.hold_required is previous.result.hold_required
        ):
            previous = tick
            continue
        intervals.append(
            PredictionStatusInterval(
                status=start.result.status,
                start_tick=start.tick_id,
                end_tick=previous.tick_id,
                hold_required=start.result.hold_required,
            )
        )
        start = previous = tick
    intervals.append(
        PredictionStatusInterval(
            status=start.result.status,
            start_tick=start.tick_id,
            end_tick=previous.tick_id,
            hold_required=start.result.hold_required,
        )
    )
    return tuple(intervals)


def _delayed_witness(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    profile: DynamicObservationProfile,
    delay_tick: int,
) -> tuple[AutomatedWitness | None, bool]:
    delay_s = delay_tick * DYNAMIC_CONTROL_PERIOD_S
    finish_s = delay_s + witness.points[-1].time_s
    within_episode = finish_s <= world.duration_s + _TIME_TOLERANCE_S
    if not within_episode:
        return None, False
    initial_twist = world.initial_state.twist
    if delay_tick and not _twist_stopped(initial_twist):
        return None, True
    leading = tuple(
        WitnessPoint(
            time_s=tick * DYNAMIC_CONTROL_PERIOD_S,
            pose=world.initial_state.pose,
            twist=Twist2D(),
            phase=WitnessPhase.HOLD,
            source_primitive_id="profile-replay-ready-hold",
        )
        for tick in range(delay_tick)
    )
    shifted = tuple(
        WitnessPoint(
            time_s=point.time_s + delay_s,
            pose=point.pose,
            twist=point.twist,
            phase=point.phase,
            source_primitive_id=point.source_primitive_id,
        )
        for point in witness.points
    )
    points = leading + shifted
    delayed = build_automated_witness(
        world,
        witness_id=(
            f"profile-replay-{profile.name.value}-{delay_tick:04d}-"
            f"{witness.semantic_content_hash[:16]}"
        ),
        kind=witness.kind,
        terminal_mode=witness.terminal_mode,
        points=points,
        required_pass_actor_ids=witness.required_pass_actor_ids,
        departure_time_s=_shift_optional_time(witness.departure_time_s, delay_s),
        pass_times_by_actor=tuple(
            (actor_id, time_s + delay_s)
            for actor_id, time_s in witness.pass_times_by_actor
        ),
        rejoin_started_at_s=_shift_optional_time(witness.rejoin_started_at_s, delay_s),
        rejoin_confirmed_at_s=_shift_optional_time(
            witness.rejoin_confirmed_at_s,
            delay_s,
        ),
        terminal_dwell_s=witness.terminal_dwell_s,
    )
    return delayed, True


def _replay_capsules(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    ticks: tuple[_PredictionTick, ...],
) -> _CapsuleReplayMetrics:
    points = witness.points
    point_times = tuple(point.time_s for point in points)
    end_time_s = point_times[-1]
    evaluated_ticks = unavailable_ticks = capsule_samples = violations = 0
    containment_samples = containment_misses = 0
    minimum_clearance: float | None = None
    maximum_miss = 0.0

    for tick in ticks:
        control_time_s = tick.tick_id * DYNAMIC_CONTROL_PERIOD_S
        post_apply_start_s = control_time_s + DYNAMIC_COMMAND_APPLY_LATENCY_S
        if post_apply_start_s >= end_time_s - _TIME_TOLERANCE_S:
            break
        post_apply_end_s = min(
            end_time_s,
            post_apply_start_s + DYNAMIC_CONTROL_PERIOD_S,
        )
        if not _witness_interval_moves(points, point_times, post_apply_start_s, post_apply_end_s):
            continue
        evaluated_ticks += 1
        result = tick.result
        if result.status not in _USABLE_STATUSES or result.prediction_set is None:
            unavailable_ticks += 1
            continue
        subdivisions = max(
            1,
            ceil(
                (post_apply_end_s - post_apply_start_s)
                / world.kinematic_contract.evaluator_period_s
            ),
        )
        for index in range(subdivisions + 1):
            rollout_time_s = min(
                post_apply_end_s - post_apply_start_s,
                index * world.kinematic_contract.evaluator_period_s,
            )
            absolute_time_s = post_apply_start_s + rollout_time_s
            pose, _ = _sample_witness(points, point_times, absolute_time_s)
            capsules = sample_directional_capsules(
                result.prediction_set,
                rollout_time_s=rollout_time_s,
            )
            by_actor = {capsule.actor_binding_id: capsule for capsule in capsules}
            for capsule in capsules:
                clearance = oriented_footprint_capsule_surface_distance(
                    pose,
                    segment_start=(capsule.start.x, capsule.start.y),
                    segment_end=(capsule.end.x, capsule.end.y),
                    capsule_radius_m=capsule.base_radius_m,
                    profile=world.kinematic_contract.vehicle_profile,
                )
                capsule_samples += 1
                minimum_clearance = (
                    clearance
                    if minimum_clearance is None
                    else min(minimum_clearance, clearance)
                )
                if (
                    clearance
                    < world.kinematic_contract.vehicle_profile.minimum_clearance_m
                    - _GEOMETRY_TOLERANCE_M
                ):
                    violations += 1
            for actor in world.actor_states_at(absolute_time_s):
                containment_samples += 1
                capsule = by_actor.get(actor.actor_id)
                if capsule is None:
                    containment_misses += 1
                    maximum_miss = max(maximum_miss, actor.radius_m)
                    continue
                center_distance = _point_segment_distance(
                    actor.position.x,
                    actor.position.y,
                    capsule.start.x,
                    capsule.start.y,
                    capsule.end.x,
                    capsule.end.y,
                )
                allowed = capsule.base_radius_m - actor.radius_m
                miss = center_distance - allowed
                if miss > _GEOMETRY_TOLERANCE_M:
                    containment_misses += 1
                    maximum_miss = max(maximum_miss, miss)

    return _CapsuleReplayMetrics(
        evaluated_motion_ticks=evaluated_ticks,
        unavailable_motion_ticks=unavailable_ticks,
        capsule_samples=capsule_samples,
        clearance_violations=violations,
        minimum_clearance_m=minimum_clearance,
        containment_samples=containment_samples,
        containment_misses=containment_misses,
        maximum_containment_miss_m=maximum_miss,
    )


def _witness_interval_moves(
    points: tuple[WitnessPoint, ...],
    point_times: tuple[float, ...],
    start_time_s: float,
    end_time_s: float,
) -> bool:
    sample_times = (start_time_s, (start_time_s + end_time_s) / 2.0, end_time_s)
    return any(
        not _twist_stopped(_sample_witness(points, point_times, time_s)[1])
        for time_s in sample_times
    )


def _sample_witness(
    points: tuple[WitnessPoint, ...],
    point_times: tuple[float, ...],
    time_s: float,
) -> tuple[Pose2D, Twist2D]:
    if time_s >= point_times[-1] - _TIME_TOLERANCE_S:
        point = points[-1]
        return point.pose, point.twist
    index = max(0, min(len(points) - 2, bisect_right(point_times, time_s) - 1))
    point = points[index]
    offset_s = max(0.0, time_s - point.time_s)
    return _integrate_pose(point.pose, point.twist, offset_s), point.twist


def _integrate_pose(pose: Pose2D, twist: Twist2D, dt_s: float) -> Pose2D:
    return Pose2D(
        pose.x + twist.linear * cos(pose.yaw) * dt_s,
        pose.y + twist.linear * sin(pose.yaw) * dt_s,
        pose.yaw + twist.angular * dt_s,
    )


def _point_segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    dx = bx - ax
    dy = by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= _GEOMETRY_TOLERANCE_M * _GEOMETRY_TOLERANCE_M:
        return hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    return hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def _shift_optional_time(value: float | None, delay_s: float) -> float | None:
    return None if value is None else value + delay_s


def _twist_stopped(twist: Twist2D) -> bool:
    return abs(twist.linear) <= 1e-12 and abs(twist.angular) <= 1e-12


__all__ = [
    "PredictionStatusInterval",
    "WITNESS_PROFILE_REPLAY_SCHEMA_VERSION",
    "WITNESS_PROFILE_REPLAY_VERSION",
    "WitnessProfileReplayBundle",
    "WitnessProfileReplayResult",
    "replay_witness_profile",
    "replay_witness_profiles",
]
