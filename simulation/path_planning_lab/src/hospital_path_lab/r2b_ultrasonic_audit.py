"""HC-SR04 7개 임시 배치로 R2-B 관측 가능성을 감사한다.

Actor identity는 독립 simulation oracle에서만 사용한다. controller-facing 초음파
frame에는 거리와 센서 상태만 남으며 Actor ID·속도·정답 분류를 넣지 않는다.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from enum import StrEnum
from math import isfinite, pi

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.dynamic_contracts import DYNAMIC_OBSERVATION_TTL_S
from hospital_path_lab.dynamic_directional_prediction import (
    FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS,
)
from hospital_path_lab.dynamic_witness_contracts import (
    AutomatedWitness,
    WitnessPoint,
    WitnessWorldSnapshot,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r2b_entry_coverage import (
    R2BEntryCoverageContract,
    derive_r2b_covered_world,
)
from hospital_path_lab.ultrasonic_observation import (
    PROVISIONAL_HC_SR04_SEVEN_SENSOR_RIG,
    UltrasonicAvailability,
    UltrasonicFrameValidator,
    UltrasonicObstacle,
    UltrasonicRigSpec,
    UltrasonicScanSampleState,
    UltrasonicValidationPolicy,
    generate_dynamic_ultrasonic_frame,
    simulated_ultrasonic_cone_range,
    ultrasonic_sensor_world_pose,
)

R2B_ULTRASONIC_AUDIT_VERSION = "r2b-ultrasonic-audit-v1"
_TIME_TOLERANCE_S = 1e-12
_RANGE_TOLERANCE_M = 1e-12


class R2BUltrasonicCoverageStatus(StrEnum):
    PREENTRY_DETECTED = "preentry_detected"
    DETECTED_AFTER_ENTRY = "detected_after_entry"
    NEVER_DETECTED = "never_detected"


@dataclass(frozen=True, slots=True)
class R2BUltrasonicDetectionEvent:
    actor_binding_id: str
    sensor_id: str
    sample_time_s: float
    frame_delivery_time_s: float
    range_m: float
    frame_sequence: int
    frame_accepted: bool


@dataclass(frozen=True, slots=True)
class R2BUltrasonicActorCoverage:
    actor_binding_id: str
    entry_time_s: float
    status: R2BUltrasonicCoverageStatus
    raw_detection_count: int
    raw_preentry_detection_count: int
    accepted_preentry_detection_count: int
    first_raw_sample_time_s: float | None
    first_raw_delivery_time_s: float | None
    first_accepted_delivery_time_s: float | None
    maximum_raw_preentry_lead_s: float | None
    detection_sensor_ids: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class R2BUltrasonicWorldAudit:
    world_content_hash: str
    scan_count: int
    accepted_frame_count: int
    stale_frame_count: int
    invalid_frame_count: int
    actor_coverage: tuple[R2BUltrasonicActorCoverage, ...]
    detection_events: tuple[R2BUltrasonicDetectionEvent, ...]

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class R2BUltrasonicAuditResult:
    schema_version: str
    source_projection_hash: str
    source_world_content_hash: str
    covered_world_content_hash: str
    witness_content_hash: str
    entry_contract_hash: str
    rig_hash: str
    per_sensor_repeat_period_s: float
    frozen_history_frame_count: int
    frozen_history_span_s: float
    sequential_history_span_s: float
    source: R2BUltrasonicWorldAudit
    covered: R2BUltrasonicWorldAudit
    range_only_track_contract_supported: bool
    r2b_observation_qualified: bool
    failures: tuple[str, ...]
    limitations: tuple[str, ...]

    def __post_init__(self) -> None:
        if self.schema_version != R2B_ULTRASONIC_AUDIT_VERSION:
            raise ValueError("unsupported R2-B ultrasonic audit version")
        if self.source.world_content_hash != self.source_world_content_hash:
            raise ValueError("source audit identity mismatch")
        if self.covered.world_content_hash != self.covered_world_content_hash:
            raise ValueError("covered audit identity mismatch")
        if self.r2b_observation_qualified and self.failures:
            raise ValueError("qualified audit must not contain failures")
        if self.r2b_observation_qualified and not self.range_only_track_contract_supported:
            raise ValueError("qualified audit requires a supported observation contract")

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


def audit_r2b_ultrasonic_entry_coverage(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    entry_contract: R2BEntryCoverageContract,
    *,
    rig: UltrasonicRigSpec = PROVISIONAL_HC_SR04_SEVEN_SENSOR_RIG,
) -> R2BUltrasonicAuditResult:
    """원본 R2-B world와 감시 진입 파생 world를 동일 7센서로 비교한다."""

    if witness.world_content_hash != world.content_hash:
        raise ValueError("witness is not bound to the source world")
    if entry_contract.source_world_content_hash != world.content_hash:
        raise ValueError("entry contract is not bound to the source world")
    covered_world = derive_r2b_covered_world(world, entry_contract)
    entry_times = {
        approach.actor_binding_id: approach.entry_time_s
        for approach in entry_contract.approaches
    }
    source = _audit_world(world, witness, entry_times=entry_times, rig=rig)
    covered = _audit_world(covered_world, witness, entry_times=entry_times, rig=rig)
    repeat_period_s = len(rig.mounts) * rig.trigger_spacing_s
    history_count = FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS.history_frame_count
    frozen_history_span_s = (history_count - 1) / 10.0
    sequential_history_span_s = (history_count - 1) * repeat_period_s

    failures: list[str] = []
    if covered.accepted_frame_count == 0:
        failures.append("full_seven_sensor_frame_exceeds_frozen_300ms_ttl")
    if any(
        item.accepted_preentry_detection_count == 0
        for item in covered.actor_coverage
    ):
        failures.append("no_accepted_preentry_detection_for_every_delayed_actor")
    if sequential_history_span_s > frozen_history_span_s + _TIME_TOLERANCE_S:
        failures.append("per_sensor_sampling_cannot_supply_frozen_10hz_history")
    failures.append("range_only_frame_has_no_actor_identity_position_or_velocity")

    return R2BUltrasonicAuditResult(
        schema_version=R2B_ULTRASONIC_AUDIT_VERSION,
        source_projection_hash=world.source_projection_hash,
        source_world_content_hash=world.content_hash,
        covered_world_content_hash=covered_world.content_hash,
        witness_content_hash=witness.semantic_content_hash,
        entry_contract_hash=entry_contract.content_hash,
        rig_hash=canonical_content_hash(rig),
        per_sensor_repeat_period_s=repeat_period_s,
        frozen_history_frame_count=history_count,
        frozen_history_span_s=frozen_history_span_s,
        sequential_history_span_s=sequential_history_span_s,
        source=source,
        covered=covered,
        range_only_track_contract_supported=False,
        r2b_observation_qualified=False,
        failures=tuple(sorted(set(failures))),
        limitations=(
            "controller_facing_frame_contains_no_actor_identity_or_ground_truth",
            "full_frame_delivery_is_assumed_after_the_last_of_seven_sequential_samples",
            "ideal_circular_reflector_without_material_temperature_or_crosstalk",
            "static_map_occlusion_and_non_actor_echoes_are_not_simulated",
            "witness_pose_is_interpolated_between_frozen_20hz_points",
        ),
    )


def _audit_world(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    *,
    entry_times: dict[str, float],
    rig: UltrasonicRigSpec,
) -> R2BUltrasonicWorldAudit:
    source_id = f"r2b-ultrasonic-{world.content_hash[:24]}"
    validator = UltrasonicFrameValidator(
        expected_source_id=source_id,
        rig=rig,
        policy=UltrasonicValidationPolicy(ttl_s=DYNAMIC_OBSERVATION_TTL_S),
    )
    repeat_period_s = len(rig.mounts) * rig.trigger_spacing_s
    events: list[R2BUltrasonicDetectionEvent] = []
    accepted_count = 0
    stale_count = 0
    invalid_count = 0
    sequence = 0
    scan_start_s = 0.0
    while scan_start_s + rig.scan_duration_s <= world.duration_s + _TIME_TOLERANCE_S:
        states = _scan_states(world, witness, scan_start_s=scan_start_s, rig=rig)
        frame = generate_dynamic_ultrasonic_frame(
            source_id=source_id,
            sequence=sequence,
            scan_started_at_s=scan_start_s,
            scan_states=states,
            rig=rig,
        )
        validation = validator.accept(frame, controller_time_s=frame.delivered_at_s)
        if validation.accepted:
            accepted_count += 1
        elif validation.availability is UltrasonicAvailability.STALE:
            stale_count += 1
        else:
            invalid_count += 1
        events.extend(
            _oracle_detection_events(
                world,
                witness,
                frame_sequence=sequence,
                frame_delivery_time_s=frame.delivered_at_s,
                frame_accepted=validation.accepted,
                scan_start_s=scan_start_s,
                rig=rig,
            )
        )
        sequence += 1
        scan_start_s = round(sequence * repeat_period_s, 12)

    coverage = tuple(
        _actor_coverage(actor_id, entry_time_s, tuple(events))
        for actor_id, entry_time_s in sorted(entry_times.items())
    )
    return R2BUltrasonicWorldAudit(
        world_content_hash=world.content_hash,
        scan_count=sequence,
        accepted_frame_count=accepted_count,
        stale_frame_count=stale_count,
        invalid_frame_count=invalid_count,
        actor_coverage=coverage,
        detection_events=tuple(events),
    )


def _scan_states(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    *,
    scan_start_s: float,
    rig: UltrasonicRigSpec,
) -> tuple[UltrasonicScanSampleState, ...]:
    states: list[UltrasonicScanSampleState] = []
    for index, mount in enumerate(rig.mounts):
        sample_time_s = scan_start_s + index * rig.trigger_spacing_s
        actor_states = world.actor_states_at(sample_time_s)
        states.append(
            UltrasonicScanSampleState(
                sensor_id=mount.sensor_id,
                measured_at_s=sample_time_s,
                robot_pose=_witness_pose_at(witness.points, sample_time_s),
                obstacles=tuple(
                    UltrasonicObstacle(
                        obstacle_id=actor.actor_id,
                        x_m=actor.position.x,
                        y_m=actor.position.y,
                        radius_m=actor.radius_m,
                    )
                    for actor in actor_states
                ),
            )
        )
    return tuple(states)


def _oracle_detection_events(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    *,
    frame_sequence: int,
    frame_delivery_time_s: float,
    frame_accepted: bool,
    scan_start_s: float,
    rig: UltrasonicRigSpec,
) -> tuple[R2BUltrasonicDetectionEvent, ...]:
    events: list[R2BUltrasonicDetectionEvent] = []
    for index, mount in enumerate(rig.mounts):
        sample_time_s = scan_start_s + index * rig.trigger_spacing_s
        robot_pose = _witness_pose_at(witness.points, sample_time_s)
        sensor_pose = ultrasonic_sensor_world_pose(robot_pose, mount)
        candidates: list[tuple[str, float]] = []
        for actor in world.actor_states_at(sample_time_s):
            distance = simulated_ultrasonic_cone_range(
                sensor_pose,
                UltrasonicObstacle(
                    obstacle_id=actor.actor_id,
                    x_m=actor.position.x,
                    y_m=actor.position.y,
                    radius_m=actor.radius_m,
                ),
                rig,
            )
            if distance is not None:
                candidates.append((actor.actor_id, distance))
        if not candidates:
            continue
        nearest = min(distance for _, distance in candidates)
        for actor_id, distance in candidates:
            if abs(distance - nearest) <= _RANGE_TOLERANCE_M:
                events.append(
                    R2BUltrasonicDetectionEvent(
                        actor_binding_id=actor_id,
                        sensor_id=mount.sensor_id,
                        sample_time_s=sample_time_s,
                        frame_delivery_time_s=frame_delivery_time_s,
                        range_m=distance,
                        frame_sequence=frame_sequence,
                        frame_accepted=frame_accepted,
                    )
                )
    return tuple(events)


def _actor_coverage(
    actor_id: str,
    entry_time_s: float,
    all_events: tuple[R2BUltrasonicDetectionEvent, ...],
) -> R2BUltrasonicActorCoverage:
    events = tuple(event for event in all_events if event.actor_binding_id == actor_id)
    preentry = tuple(
        event
        for event in events
        if event.frame_delivery_time_s <= entry_time_s + _TIME_TOLERANCE_S
    )
    accepted_preentry = tuple(event for event in preentry if event.frame_accepted)
    if preentry:
        status = R2BUltrasonicCoverageStatus.PREENTRY_DETECTED
    elif events:
        status = R2BUltrasonicCoverageStatus.DETECTED_AFTER_ENTRY
    else:
        status = R2BUltrasonicCoverageStatus.NEVER_DETECTED
    first = min(events, key=lambda item: (item.sample_time_s, item.sensor_id)) if events else None
    first_accepted = (
        min(accepted_preentry, key=lambda item: item.frame_delivery_time_s)
        if accepted_preentry
        else None
    )
    return R2BUltrasonicActorCoverage(
        actor_binding_id=actor_id,
        entry_time_s=entry_time_s,
        status=status,
        raw_detection_count=len(events),
        raw_preentry_detection_count=len(preentry),
        accepted_preentry_detection_count=len(accepted_preentry),
        first_raw_sample_time_s=first.sample_time_s if first else None,
        first_raw_delivery_time_s=first.frame_delivery_time_s if first else None,
        first_accepted_delivery_time_s=(
            first_accepted.frame_delivery_time_s if first_accepted else None
        ),
        maximum_raw_preentry_lead_s=(
            max(entry_time_s - event.frame_delivery_time_s for event in preentry)
            if preentry
            else None
        ),
        detection_sensor_ids=tuple(sorted({event.sensor_id for event in events})),
    )


def _witness_pose_at(points: tuple[WitnessPoint, ...], time_s: float) -> Pose2D:
    if not isfinite(time_s) or time_s < 0.0:
        raise ValueError("witness pose query time must be finite and non-negative")
    if time_s <= points[0].time_s:
        return points[0].pose
    if time_s >= points[-1].time_s:
        return points[-1].pose
    times = tuple(point.time_s for point in points)
    right_index = bisect_right(times, time_s)
    left = points[right_index - 1]
    right = points[right_index]
    ratio = (time_s - left.time_s) / (right.time_s - left.time_s)
    yaw_delta = (right.pose.yaw - left.pose.yaw + pi) % (2.0 * pi) - pi
    return Pose2D(
        x=left.pose.x + (right.pose.x - left.pose.x) * ratio,
        y=left.pose.y + (right.pose.y - left.pose.y) * ratio,
        yaw=left.pose.yaw + yaw_delta * ratio,
    )


__all__ = [
    "R2B_ULTRASONIC_AUDIT_VERSION",
    "R2BUltrasonicActorCoverage",
    "R2BUltrasonicAuditResult",
    "R2BUltrasonicCoverageStatus",
    "R2BUltrasonicDetectionEvent",
    "R2BUltrasonicWorldAudit",
    "audit_r2b_ultrasonic_entry_coverage",
]
