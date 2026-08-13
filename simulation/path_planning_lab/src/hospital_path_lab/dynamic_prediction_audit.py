"""Public-only audit for directional Actor prediction contracts.

The audit deliberately separates deterministic ground-truth motion bounds from
statistical Gaussian observation coverage.  It is an offline simulation tool:
ground truth used here must never be routed into an online controller or gate.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from json import dumps
from math import acos, erf, exp, hypot, isfinite, sqrt
from pathlib import Path

from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    ActorState,
    Point2D,
)
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    V6DynamicCorpusEpisode,
    controller_episode_id,
    generate_dynamic_v6_public_corpus,
    generate_episode_observation_slots,
)
from hospital_path_lab.dynamic_directional_prediction import (
    FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS,
    DirectionalActorPredictor,
    DirectionalPredictionParameters,
    DirectionalPredictionStatus,
    sample_directional_capsules,
)
from hospital_path_lab.dynamic_observation import (
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationProfile,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
)
from hospital_path_lab.map_factory import canonical_content_hash

PREDICTION_CONTRACT_AUDIT_VERSION = "prediction_contract_audit_v1"
PREDICTION_CONTRACT_AUDIT_SCHEMA_VERSION = "prediction-contract-audit-v1"
PREDICTION_AUDIT_ROLLOUT_TIMES_S = (0.0, 0.5, 1.0, 1.5, 2.0, 2.4)
_PUBLIC_SPLITS = frozenset(
    (DynamicCorpusSplit.GOLDEN, DynamicCorpusSplit.DEVELOPMENT)
)
_EPSILON = 1e-9
_GAUSSIAN_COMPONENT_2SIGMA_PROBABILITY = erf(2.0 / sqrt(2.0))
_GAUSSIAN_RADIAL_2SIGMA_PROBABILITY = 1.0 - exp(-2.0)


@dataclass(frozen=True, slots=True)
class MotionAuditSample:
    episode_id: str
    actor_id: str
    time_s: float
    state: ActorState

    def __post_init__(self) -> None:
        if not self.episode_id or not self.actor_id:
            raise ValueError("motion audit sample identities must not be empty")
        if not isfinite(self.time_s) or self.time_s < 0.0:
            raise ValueError("motion audit sample time must be finite and non-negative")
        if self.state.actor_id != self.actor_id:
            raise ValueError("motion audit sample actor identity mismatch")


@dataclass(frozen=True, slots=True)
class MotionContractViolation:
    episode_id: str
    actor_id: str
    time_s: float
    reason_code: str
    measured_value: float
    allowed_value: float


@dataclass(frozen=True, slots=True)
class MotionContractAudit:
    actor_count: int
    sample_count: int
    transition_count: int
    acceleration_transition_count: int
    deceleration_transition_count: int
    stop_transition_count: int
    turn_transition_count: int
    maximum_speed_mps: float
    maximum_acceleration_mps2: float
    maximum_heading_rate_radps: float
    maximum_lateral_displacement_m: float
    violations: tuple[MotionContractViolation, ...]

    @property
    def passed(self) -> bool:
        return not self.violations


@dataclass(frozen=True, slots=True)
class ObservationCoverageAudit:
    profile_name: str
    slot_count: int
    delivered_frame_count: int
    dropout_count: int
    track_count: int
    exact_position_error_count: int
    exact_velocity_error_count: int
    component_position_sample_count: int
    component_position_inside_2sigma_count: int
    component_position_coverage: float | None
    radial_position_sample_count: int
    radial_position_inside_2sigma_count: int
    radial_position_coverage: float | None
    component_velocity_sample_count: int
    component_velocity_inside_2sigma_count: int
    component_velocity_coverage: float | None
    radial_velocity_sample_count: int
    radial_velocity_inside_2sigma_count: int
    radial_velocity_coverage: float | None
    maximum_position_component_z: float | None
    maximum_velocity_component_z: float | None
    expected_component_2sigma_probability: float
    expected_radial_2sigma_probability: float
    interpretation: str


@dataclass(frozen=True, slots=True)
class CapsuleRolloutCoverage:
    rollout_time_s: float
    sample_count: int
    contained_count: int
    coverage: float | None


@dataclass(frozen=True, slots=True)
class CapsuleCoverageAudit:
    profile_name: str
    unique_ready_prediction_count: int
    sample_count: int
    contained_count: int
    coverage: float | None
    miss_count: int
    maximum_miss_distance_m: float
    rollout_coverage: tuple[CapsuleRolloutCoverage, ...]
    interpretation: str


@dataclass(frozen=True, slots=True)
class PublicPredictionContractAudit:
    schema_version: str
    audit_version: str
    simulation_only: bool
    public_episode_count: int
    public_corpus_content_hash: str
    motion_contract: MotionContractAudit
    observation_coverage: tuple[ObservationCoverageAudit, ...]
    capsule_coverage: tuple[CapsuleCoverageAudit, ...]
    hard_failures: tuple[str, ...]
    limitations: tuple[str, ...]
    content_hash: str

    @property
    def passed(self) -> bool:
        return not self.hard_failures


def audit_directional_motion_samples(
    samples: tuple[MotionAuditSample, ...],
    *,
    parameters: DirectionalPredictionParameters = (
        FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS
    ),
) -> MotionContractAudit:
    """Audit ordered Actor samples against the frozen directional motion bound."""

    if not isinstance(samples, tuple):
        raise TypeError("motion audit samples must be a tuple")
    ordered = tuple(sorted(samples, key=lambda item: (item.episode_id, item.actor_id, item.time_s)))
    if ordered != samples:
        raise ValueError("motion audit samples must be deterministically ordered")

    violations: list[MotionContractViolation] = []
    previous_by_actor: dict[tuple[str, str], MotionAuditSample] = {}
    actor_keys: set[tuple[str, str]] = set()
    transition_count = 0
    acceleration_count = 0
    deceleration_count = 0
    stop_count = 0
    turn_count = 0
    maximum_speed = 0.0
    maximum_acceleration = 0.0
    maximum_heading_rate = 0.0
    maximum_lateral_displacement = 0.0

    for sample in samples:
        key = (sample.episode_id, sample.actor_id)
        actor_keys.add(key)
        state = sample.state
        values = (
            state.position.x,
            state.position.y,
            state.velocity.x,
            state.velocity.y,
            state.radius_m,
        )
        if not all(isfinite(value) for value in values):
            violations.append(
                _motion_violation(sample, "non_finite_motion_sample", float("inf"), 0.0)
            )
            previous_by_actor[key] = sample
            continue
        speed = state.velocity.magnitude
        maximum_speed = max(maximum_speed, speed)
        if speed > parameters.maximum_speed_mps + _EPSILON:
            violations.append(
                _motion_violation(
                    sample,
                    "speed_limit_exceeded",
                    speed,
                    parameters.maximum_speed_mps,
                )
            )

        previous = previous_by_actor.get(key)
        previous_by_actor[key] = sample
        if previous is None:
            continue
        dt_s = sample.time_s - previous.time_s
        if dt_s <= 0.0:
            violations.append(
                _motion_violation(sample, "non_increasing_motion_time", dt_s, 0.0)
            )
            continue
        transition_count += 1
        previous_speed = previous.state.velocity.magnitude
        delta_velocity = hypot(
            state.velocity.x - previous.state.velocity.x,
            state.velocity.y - previous.state.velocity.y,
        )
        acceleration = delta_velocity / dt_s
        maximum_acceleration = max(maximum_acceleration, acceleration)
        if speed > previous_speed + _EPSILON:
            acceleration_count += 1
            acceleration_limit = parameters.maximum_longitudinal_acceleration_mps2
        elif speed < previous_speed - _EPSILON:
            deceleration_count += 1
            acceleration_limit = parameters.maximum_longitudinal_deceleration_mps2
        else:
            acceleration_limit = parameters.maximum_longitudinal_acceleration_mps2
        if previous_speed > _EPSILON and speed <= _EPSILON:
            stop_count += 1
        if acceleration > acceleration_limit + _EPSILON:
            violations.append(
                _motion_violation(
                    sample,
                    "longitudinal_acceleration_limit_exceeded",
                    acceleration,
                    acceleration_limit,
                )
            )

        dx = state.position.x - previous.state.position.x
        dy = state.position.y - previous.state.position.y
        if previous_speed <= _EPSILON:
            maximum_departure = (
                0.5 * parameters.maximum_longitudinal_acceleration_mps2 * dt_s**2
            )
            displacement = hypot(dx, dy)
            if displacement > maximum_departure + _EPSILON:
                violations.append(
                    _motion_violation(
                        sample,
                        "departure_from_rest_exceeds_bound",
                        displacement,
                        maximum_departure,
                    )
                )
            continue

        heading_x = previous.state.velocity.x / previous_speed
        heading_y = previous.state.velocity.y / previous_speed
        longitudinal_displacement = dx * heading_x + dy * heading_y
        lateral_displacement = abs(-dx * heading_y + dy * heading_x)
        maximum_lateral_displacement = max(
            maximum_lateral_displacement,
            lateral_displacement,
        )
        if lateral_displacement > parameters.lateral_turn_bound_m + _EPSILON:
            violations.append(
                _motion_violation(
                    sample,
                    "lateral_motion_outside_contract",
                    lateral_displacement,
                    parameters.lateral_turn_bound_m,
                )
            )

        minimum_longitudinal = _limited_braking_distance(
            previous_speed,
            dt_s,
            parameters.maximum_longitudinal_deceleration_mps2,
        )
        maximum_longitudinal = _limited_acceleration_distance(
            previous_speed,
            dt_s,
            parameters.maximum_speed_mps,
            parameters.maximum_longitudinal_acceleration_mps2,
        )
        if longitudinal_displacement < minimum_longitudinal - _EPSILON:
            violations.append(
                _motion_violation(
                    sample,
                    "reverse_or_excessive_braking_motion",
                    longitudinal_displacement,
                    minimum_longitudinal,
                )
            )
        if longitudinal_displacement > maximum_longitudinal + _EPSILON:
            violations.append(
                _motion_violation(
                    sample,
                    "forward_motion_exceeds_acceleration_bound",
                    longitudinal_displacement,
                    maximum_longitudinal,
                )
            )

        if speed > _EPSILON:
            cosine = (
                previous.state.velocity.x * state.velocity.x
                + previous.state.velocity.y * state.velocity.y
            ) / (previous_speed * speed)
            heading_change = acos(max(-1.0, min(1.0, cosine)))
            heading_rate = heading_change / dt_s
            maximum_heading_rate = max(maximum_heading_rate, heading_rate)
            if heading_change > _EPSILON:
                turn_count += 1
                violations.append(
                    _motion_violation(
                        sample,
                        "heading_change_outside_constant_heading_contract",
                        heading_change,
                        0.0,
                    )
                )

    return MotionContractAudit(
        actor_count=len(actor_keys),
        sample_count=len(samples),
        transition_count=transition_count,
        acceleration_transition_count=acceleration_count,
        deceleration_transition_count=deceleration_count,
        stop_transition_count=stop_count,
        turn_transition_count=turn_count,
        maximum_speed_mps=maximum_speed,
        maximum_acceleration_mps2=maximum_acceleration,
        maximum_heading_rate_radps=maximum_heading_rate,
        maximum_lateral_displacement_m=maximum_lateral_displacement,
        violations=tuple(violations),
    )


def audit_public_prediction_contract(
    episodes: tuple[V6DynamicCorpusEpisode, ...] | None = None,
) -> PublicPredictionContractAudit:
    """Run the frozen Stage-1 audit on the complete v6 public corpus."""

    public = generate_dynamic_v6_public_corpus() if episodes is None else episodes
    _validate_public_episodes(public)
    motion = audit_directional_motion_samples(_public_motion_samples(public))
    profiles = (
        FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
        NORMAL_OBSERVATION_PROFILE,
        STRESS_OBSERVATION_PROFILE,
    )
    observation = tuple(_audit_observation_coverage(public, profile) for profile in profiles)
    capsule = tuple(_audit_capsule_coverage(public, profile) for profile in profiles)

    hard_failures: list[str] = []
    if motion.violations:
        hard_failures.append("public_motion_contract_violation")
    ideal_observation = observation[0]
    if ideal_observation.dropout_count:
        hard_failures.append("ideal_observation_dropout")
    if (
        ideal_observation.exact_position_error_count
        or ideal_observation.exact_velocity_error_count
    ):
        hard_failures.append("ideal_observation_not_exact")
    ideal_capsule = capsule[0]
    if ideal_capsule.miss_count:
        hard_failures.append("ideal_capsule_ground_truth_miss")

    limitations = [
        "simulation_only_open_loop_circular_actor",
        "gaussian_2sigma_is_statistical_not_hard_safety",
    ]
    feature_limits = (
        (motion.acceleration_transition_count, "public_corpus_has_no_acceleration_transition"),
        (motion.deceleration_transition_count, "public_corpus_has_no_deceleration_transition"),
        (motion.stop_transition_count, "public_corpus_has_no_stop_transition"),
        (motion.turn_transition_count, "public_corpus_has_no_turn_transition"),
    )
    limitations.extend(reason for count, reason in feature_limits if count == 0)
    for item in observation[1:]:
        if (
            item.component_position_inside_2sigma_count
            != item.component_position_sample_count
            or item.component_velocity_inside_2sigma_count
            != item.component_velocity_sample_count
        ):
            limitations.append(f"{item.profile_name}_observation_2sigma_misses_present")
    for item in capsule[1:]:
        if item.sample_count == 0:
            limitations.append(f"{item.profile_name}_has_no_ready_capsule_samples")
        elif item.miss_count:
            limitations.append(f"{item.profile_name}_capsule_coverage_misses_present")

    draft = PublicPredictionContractAudit(
        schema_version=PREDICTION_CONTRACT_AUDIT_SCHEMA_VERSION,
        audit_version=PREDICTION_CONTRACT_AUDIT_VERSION,
        simulation_only=True,
        public_episode_count=len(public),
        public_corpus_content_hash=canonical_content_hash(public),
        motion_contract=motion,
        observation_coverage=observation,
        capsule_coverage=capsule,
        hard_failures=tuple(hard_failures),
        limitations=tuple(sorted(set(limitations))),
        content_hash="pending",
    )
    return replace(draft, content_hash=canonical_content_hash(_audit_hash_payload(draft)))


def write_prediction_contract_audit(
    audit: PublicPredictionContractAudit,
    output_dir: Path,
) -> tuple[Path, Path]:
    """Write deterministic JSON and Markdown without overwriting existing evidence."""

    if not isinstance(audit, PublicPredictionContractAudit):
        raise TypeError("audit must be a PublicPredictionContractAudit")
    output_dir = Path(output_dir)
    if output_dir.exists():
        raise FileExistsError(f"prediction audit output already exists: {output_dir}")
    output_dir.mkdir(parents=True)
    json_path = output_dir / "prediction_contract_audit.json"
    summary_path = output_dir / "summary.md"
    json_path.write_text(
        dumps(asdict(audit), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    summary_path.write_text(_audit_summary(audit), encoding="utf-8")
    return json_path, summary_path


def _validate_public_episodes(episodes: tuple[V6DynamicCorpusEpisode, ...]) -> None:
    if not isinstance(episodes, tuple) or not episodes:
        raise ValueError("prediction audit requires a non-empty public tuple")
    if any(type(episode) is not V6DynamicCorpusEpisode for episode in episodes):
        raise ValueError("prediction audit accepts only v6 public episodes")
    if any(episode.split not in _PUBLIC_SPLITS for episode in episodes):
        raise ValueError("prediction audit rejects hidden or unsupported splits")
    if len({episode.episode_id for episode in episodes}) != len(episodes):
        raise ValueError("prediction audit episode identities must be unique")


def _public_motion_samples(
    episodes: tuple[V6DynamicCorpusEpisode, ...],
) -> tuple[MotionAuditSample, ...]:
    samples: list[MotionAuditSample] = []
    for episode in episodes:
        for actor in episode.actors:
            first_tick = round(actor.active_from_s / DYNAMIC_CONTROL_PERIOD_S)
            last_tick = round(actor.active_until_s / DYNAMIC_CONTROL_PERIOD_S)
            for tick_id in range(first_tick, last_tick + 1):
                time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
                state = actor.state_at(time_s)
                if state is None:
                    raise ValueError("public Actor active interval is not tick aligned")
                samples.append(
                    MotionAuditSample(
                        episode_id=episode.episode_id,
                        actor_id=actor.actor_id,
                        time_s=time_s,
                        state=state,
                    )
                )
    return tuple(sorted(samples, key=lambda item: (item.episode_id, item.actor_id, item.time_s)))


def _audit_observation_coverage(
    episodes: tuple[V6DynamicCorpusEpisode, ...],
    profile: DynamicObservationProfile,
) -> ObservationCoverageAudit:
    slot_count = 0
    delivered_count = 0
    dropout_count = 0
    track_count = 0
    exact_position_errors = 0
    exact_velocity_errors = 0
    position_component_count = 0
    position_component_inside = 0
    position_radial_count = 0
    position_radial_inside = 0
    velocity_component_count = 0
    velocity_component_inside = 0
    velocity_radial_count = 0
    velocity_radial_inside = 0
    maximum_position_z: float | None = None
    maximum_velocity_z: float | None = None

    for episode in episodes:
        for slot in generate_episode_observation_slots(episode, profile=profile):
            slot_count += 1
            if slot.frame is None:
                dropout_count += 1
                continue
            delivered_count += 1
            truth_by_id = {
                actor.actor_id: actor
                for actor in episode.actor_states_at(slot.observed_at_s)
            }
            for track in slot.frame.tracks:
                truth = truth_by_id.get(track.actor_binding_id)
                if truth is None:
                    raise ValueError("observation track has no ground-truth Actor binding")
                track_count += 1
                position_errors = (
                    track.observed_position.x - truth.position.x,
                    track.observed_position.y - truth.position.y,
                )
                velocity_errors = (
                    track.observed_velocity.x - truth.velocity.x,
                    track.observed_velocity.y - truth.velocity.y,
                )
                if profile.position_sigma_m == 0.0:
                    exact_position_errors += sum(abs(value) > _EPSILON for value in position_errors)
                else:
                    z_values = tuple(
                        abs(value) / profile.position_sigma_m for value in position_errors
                    )
                    position_component_count += len(z_values)
                    position_component_inside += sum(value <= 2.0 for value in z_values)
                    position_radial_count += 1
                    position_radial_inside += hypot(*z_values) <= 2.0
                    maximum_position_z = max(maximum_position_z or 0.0, *z_values)
                if profile.velocity_sigma_mps == 0.0:
                    exact_velocity_errors += sum(abs(value) > _EPSILON for value in velocity_errors)
                else:
                    z_values = tuple(
                        abs(value) / profile.velocity_sigma_mps for value in velocity_errors
                    )
                    velocity_component_count += len(z_values)
                    velocity_component_inside += sum(value <= 2.0 for value in z_values)
                    velocity_radial_count += 1
                    velocity_radial_inside += hypot(*z_values) <= 2.0
                    maximum_velocity_z = max(maximum_velocity_z or 0.0, *z_values)

    return ObservationCoverageAudit(
        profile_name=profile.name.value,
        slot_count=slot_count,
        delivered_frame_count=delivered_count,
        dropout_count=dropout_count,
        track_count=track_count,
        exact_position_error_count=exact_position_errors,
        exact_velocity_error_count=exact_velocity_errors,
        component_position_sample_count=position_component_count,
        component_position_inside_2sigma_count=position_component_inside,
        component_position_coverage=_coverage(position_component_inside, position_component_count),
        radial_position_sample_count=position_radial_count,
        radial_position_inside_2sigma_count=position_radial_inside,
        radial_position_coverage=_coverage(position_radial_inside, position_radial_count),
        component_velocity_sample_count=velocity_component_count,
        component_velocity_inside_2sigma_count=velocity_component_inside,
        component_velocity_coverage=_coverage(velocity_component_inside, velocity_component_count),
        radial_velocity_sample_count=velocity_radial_count,
        radial_velocity_inside_2sigma_count=velocity_radial_inside,
        radial_velocity_coverage=_coverage(velocity_radial_inside, velocity_radial_count),
        maximum_position_component_z=maximum_position_z,
        maximum_velocity_component_z=maximum_velocity_z,
        expected_component_2sigma_probability=_GAUSSIAN_COMPONENT_2SIGMA_PROBABILITY,
        expected_radial_2sigma_probability=_GAUSSIAN_RADIAL_2SIGMA_PROBABILITY,
        interpretation=(
            "deterministic_exact_input"
            if profile.position_sigma_m == profile.velocity_sigma_mps == 0.0
            else "statistical_coverage_not_hard_safety"
        ),
    )


def _audit_capsule_coverage(
    episodes: tuple[V6DynamicCorpusEpisode, ...],
    profile: DynamicObservationProfile,
) -> CapsuleCoverageAudit:
    by_rollout = {time_s: [0, 0] for time_s in PREDICTION_AUDIT_ROLLOUT_TIMES_S}
    unique_ready = 0
    sample_count = 0
    contained_count = 0
    maximum_miss = 0.0

    for episode in episodes:
        source = DynamicObservationSourceIdentity(
            stream_id="dynamic-stage5-stream",
            episode_id=controller_episode_id(episode),
            episode_seed=episode.seed,
            map_id=episode.map_id,
            map_revision=1,
        )
        slots = generate_episode_observation_slots(episode, profile=profile)
        validator = DynamicObservationValidator(source, profile)
        predictor = DirectionalActorPredictor()
        next_slot = 0
        seen_predictions: set[str] = set()
        for tick_id in range(episode.tick_count + 1):
            control_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
            while (
                next_slot < len(slots)
                and slots[next_slot].scheduled_delivery_at_s <= control_time_s + _EPSILON
            ):
                slot = slots[next_slot]
                if slot.frame is None:
                    validator.record_no_frame(
                        sequence=slot.sequence,
                        delivery_time_s=slot.scheduled_delivery_at_s,
                    )
                else:
                    validation = validator.accept(
                        slot.frame,
                        received_at_s=slot.scheduled_delivery_at_s,
                    )
                    if not validation.accepted:
                        raise ValueError(
                            "generated public observation failed validation during audit"
                        )
                next_slot += 1
            result = predictor.update(validator.snapshot(control_time_s=control_time_s))
            if (
                result.status is not DirectionalPredictionStatus.READY
                or result.prediction_set is None
                or result.prediction_set.content_hash in seen_predictions
            ):
                continue
            prediction = result.prediction_set
            seen_predictions.add(prediction.content_hash)
            unique_ready += 1
            for rollout_time_s in PREDICTION_AUDIT_ROLLOUT_TIMES_S:
                truth_time_s = (
                    control_time_s
                    + FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS.command_apply_latency_s
                    + rollout_time_s
                )
                if truth_time_s > episode.duration_s + _EPSILON:
                    continue
                truth_by_id = {
                    actor.actor_id: actor
                    for actor in episode.actor_states_at(truth_time_s)
                }
                for capsule in sample_directional_capsules(
                    prediction,
                    rollout_time_s=rollout_time_s,
                ):
                    truth = truth_by_id.get(capsule.actor_binding_id)
                    if truth is None:
                        continue
                    allowed_center_distance = capsule.base_radius_m - truth.radius_m
                    center_distance = _point_segment_distance(
                        truth.position,
                        capsule.start,
                        capsule.end,
                    )
                    contained = center_distance <= allowed_center_distance + _EPSILON
                    sample_count += 1
                    contained_count += contained
                    by_rollout[rollout_time_s][0] += 1
                    by_rollout[rollout_time_s][1] += contained
                    if not contained:
                        maximum_miss = max(
                            maximum_miss,
                            center_distance - allowed_center_distance,
                        )

    rollout_coverage = tuple(
        CapsuleRolloutCoverage(
            rollout_time_s=time_s,
            sample_count=counts[0],
            contained_count=counts[1],
            coverage=_coverage(counts[1], counts[0]),
        )
        for time_s, counts in by_rollout.items()
    )
    return CapsuleCoverageAudit(
        profile_name=profile.name.value,
        unique_ready_prediction_count=unique_ready,
        sample_count=sample_count,
        contained_count=contained_count,
        coverage=_coverage(contained_count, sample_count),
        miss_count=sample_count - contained_count,
        maximum_miss_distance_m=maximum_miss,
        rollout_coverage=rollout_coverage,
        interpretation=(
            "deterministic_ideal_containment"
            if profile.position_sigma_m == profile.velocity_sigma_mps == 0.0
            else "empirical_statistical_capsule_coverage"
        ),
    )


def _motion_violation(
    sample: MotionAuditSample,
    reason_code: str,
    measured_value: float,
    allowed_value: float,
) -> MotionContractViolation:
    return MotionContractViolation(
        episode_id=sample.episode_id,
        actor_id=sample.actor_id,
        time_s=sample.time_s,
        reason_code=reason_code,
        measured_value=measured_value,
        allowed_value=allowed_value,
    )


def _limited_braking_distance(speed_mps: float, time_s: float, deceleration: float) -> float:
    stop_time_s = speed_mps / deceleration
    if time_s <= stop_time_s:
        return speed_mps * time_s - 0.5 * deceleration * time_s**2
    return speed_mps**2 / (2.0 * deceleration)


def _limited_acceleration_distance(
    speed_mps: float,
    time_s: float,
    maximum_speed_mps: float,
    acceleration: float,
) -> float:
    saturation_time_s = max(0.0, maximum_speed_mps - speed_mps) / acceleration
    if time_s <= saturation_time_s:
        return speed_mps * time_s + 0.5 * acceleration * time_s**2
    return (
        speed_mps * saturation_time_s
        + 0.5 * acceleration * saturation_time_s**2
        + maximum_speed_mps * (time_s - saturation_time_s)
    )


def _point_segment_distance(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    denominator = dx * dx + dy * dy
    if denominator <= _EPSILON**2:
        return hypot(point.x - start.x, point.y - start.y)
    fraction = max(
        0.0,
        min(
            1.0,
            ((point.x - start.x) * dx + (point.y - start.y) * dy) / denominator,
        ),
    )
    nearest_x = start.x + fraction * dx
    nearest_y = start.y + fraction * dy
    return hypot(point.x - nearest_x, point.y - nearest_y)


def _coverage(inside_count: int, sample_count: int) -> float | None:
    return inside_count / sample_count if sample_count else None


def _audit_hash_payload(audit: PublicPredictionContractAudit) -> dict[str, object]:
    payload = asdict(audit)
    payload.pop("content_hash")
    return payload


def _audit_summary(audit: PublicPredictionContractAudit) -> str:
    lines = [
        "# Actor prediction 계약 감사 결과",
        "",
        f"- 판정: `{'PASS' if audit.passed else 'FAIL'}`",
        f"- 공개 episode: `{audit.public_episode_count}`",
        f"- motion transition: `{audit.motion_contract.transition_count}`",
        f"- motion violation: `{len(audit.motion_contract.violations)}`",
        f"- content hash: `{audit.content_hash}`",
        "",
        "## 관측 2σ coverage",
        "",
        "| profile | tracks | dropout | position component | position radial "
        "| velocity component | velocity radial |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for item in audit.observation_coverage:
        lines.append(
            "| "
            + " | ".join(
                (
                    item.profile_name,
                    str(item.track_count),
                    str(item.dropout_count),
                    _format_optional_ratio(item.component_position_coverage),
                    _format_optional_ratio(item.radial_position_coverage),
                    _format_optional_ratio(item.component_velocity_coverage),
                    _format_optional_ratio(item.radial_velocity_coverage),
                )
            )
            + " |"
        )
    lines.extend(
        (
            "",
            "## 방향성 Capsule coverage",
            "",
            "| profile | unique READY | samples | contained | coverage | max miss |",
            "|---|---:|---:|---:|---:|---:|",
        )
    )
    for item in audit.capsule_coverage:
        lines.append(
            "| "
            + " | ".join(
                (
                    item.profile_name,
                    str(item.unique_ready_prediction_count),
                    str(item.sample_count),
                    str(item.contained_count),
                    _format_optional_ratio(item.coverage),
                    f"{item.maximum_miss_distance_m:.9f} m",
                )
            )
            + " |"
        )
    lines.extend(("", "## Hard failures", ""))
    lines.extend(f"- `{item}`" for item in audit.hard_failures)
    if not audit.hard_failures:
        lines.append("- 없음")
    lines.extend(("", "## Limitations", ""))
    lines.extend(f"- `{item}`" for item in audit.limitations)
    lines.extend(
        (
            "",
            "> Normal·Stress의 2σ 및 Capsule miss는 통계적 coverage 결과이며,",
            "> 실제 사람 안전이나 제품 안전보장을 의미하지 않는다.",
            "",
        )
    )
    return "\n".join(lines)


def _format_optional_ratio(value: float | None) -> str:
    return "N/A" if value is None else f"{value:.6f}"


__all__ = [
    "CapsuleCoverageAudit",
    "MotionAuditSample",
    "MotionContractAudit",
    "ObservationCoverageAudit",
    "PREDICTION_AUDIT_ROLLOUT_TIMES_S",
    "PublicPredictionContractAudit",
    "audit_directional_motion_samples",
    "audit_public_prediction_contract",
    "write_prediction_contract_audit",
]
