"""R2-A ordered ground-truth stop -> resume -> restop evidence."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from math import cos, hypot, sin

from hospital_path_lab.contracts import Pose2D, Twist2D
from hospital_path_lab.dynamic_witness_contracts import (
    FROZEN_WITNESS_SEARCH_CONFIG,
    AutomatedWitness,
    WitnessKind,
    WitnessPhase,
    WitnessPoint,
    WitnessSearchConfig,
    WitnessTerminalMode,
    WitnessWorldSnapshot,
    build_automated_witness,
)
from hospital_path_lab.dynamic_witness_events import (
    GroundTruthHazardInterval,
    ground_truth_hazard_intervals,
    straight_reference_segments,
)
from hospital_path_lab.dynamic_witness_validation import (
    GroundTruthWitnessValidation,
    validate_ground_truth_witness,
)
from hospital_path_lab.map_factory import canonical_content_hash

RESTOP_SEARCH_VERSION = "multi-hazard-restop-search-v1"
RESTOP_VALIDATOR_VERSION = "multi-hazard-restop-validator-v1"
_STOP_TOLERANCE = 1e-12
_TIME_TOLERANCE = 1e-12
_MINIMUM_INTERMEDIATE_PROGRESS_M = 0.10


class RestopEvidenceLevel(StrEnum):
    NONE = "none"
    RESTOP_CORE_PROVEN = "restop_core_proven"
    RESTOP_AND_RECOVERY_PROVEN = "restop_and_recovery_proven"


@dataclass(frozen=True, slots=True)
class KinematicStopInterval:
    stopped_from_s: float
    stopped_until_s: float
    pose: Pose2D
    preceding_motion_observed: bool
    following_motion_observed: bool
    bound_hazard_ids: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class RestopValidation:
    validator_version: str
    witness_content_hash: str
    base_validation: GroundTruthWitnessValidation
    hazards: tuple[GroundTruthHazardInterval, ...]
    stop_intervals: tuple[KinematicStopInterval, ...]
    evidence_level: RestopEvidenceLevel
    core_passed: bool
    recovery_passed: bool
    core_failures: tuple[str, ...]
    recovery_failures: tuple[str, ...]
    intermediate_progress_m: float

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class RestopSearchResult:
    witness: AutomatedWitness | None
    validation: RestopValidation | None
    generated_count: int
    validated_count: int
    termination_reason: str

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


def search_multi_hazard_restop(
    world: WitnessWorldSnapshot,
    *,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
) -> RestopSearchResult:
    """Search the frozen two-hazard event template on one straight path."""

    if world.search_config_hash != search_config.content_hash:
        return RestopSearchResult(None, None, 0, 0, "invalid_provenance")
    hazards = ground_truth_hazard_intervals(world)
    if len(hazards) < 2:
        return RestopSearchResult(
            None,
            None,
            0,
            0,
            "restop_sequence_not_applicable",
        )
    generated = validated = 0
    best: tuple[AutomatedWitness, RestopValidation] | None = None
    for speed in reversed(search_config.linear_targets_mps):
        generated += 1
        witness = _build_restop_candidate(world, hazards[:2], speed)
        if witness is None:
            continue
        validation = validate_multi_hazard_restop(world, witness)
        if not validation.core_passed:
            continue
        validated += 1
        if best is None or (
            not validation.recovery_passed,
            witness.points[-1].time_s,
            witness.semantic_content_hash,
        ) < (
            not best[1].recovery_passed,
            best[0].points[-1].time_s,
            best[0].semantic_content_hash,
        ):
            best = (witness, validation)
    if best is None:
        return RestopSearchResult(
            None,
            None,
            generated,
            validated,
            "no_witness_in_restop_template",
        )
    return RestopSearchResult(
        best[0],
        best[1],
        generated,
        validated,
        "restop_and_recovery_found"
        if best[1].recovery_passed
        else "restop_core_found",
    )


def validate_multi_hazard_restop(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
) -> RestopValidation:
    base = validate_ground_truth_witness(world, witness)
    hazards = ground_truth_hazard_intervals(world)
    raw_stops = _kinematic_stop_intervals(witness)
    stops = tuple(_bind_stop(interval, hazards) for interval in raw_stops)
    core_failures: list[str] = []
    recovery_failures: list[str] = []
    if not base.passed:
        core_failures.append("invalid_provenance_or_ground_truth_motion")
    if len(hazards) < 2:
        core_failures.append("restop_sequence_not_applicable")
        return _validation(
            witness,
            base,
            hazards,
            stops,
            core_failures,
            recovery_failures,
            0.0,
        )
    first_hazard, second_hazard = hazards[:2]
    first_stop = next(
        (
            interval
            for interval in stops
            if first_hazard.hazard_id in interval.bound_hazard_ids
        ),
        None,
    )
    if first_stop is None:
        core_failures.append("first_hazard_stop_missing")
    second_stop = next(
        (
            interval
            for interval in stops
            if second_hazard.hazard_id in interval.bound_hazard_ids
            and (first_stop is None or interval.stopped_from_s > first_stop.stopped_until_s)
        ),
        None,
    )
    if second_stop is None:
        core_failures.append("second_hazard_stop_missing")
    if first_stop is not None and second_hazard.hazard_id in first_stop.bound_hazard_ids:
        core_failures.extend(
            ("continuous_hold_misclassified_as_restop", "second_stop_not_distinct")
        )

    resume_points = tuple(
        point
        for point in witness.points
        if point.time_s > first_hazard.ends_at_s + _TIME_TOLERANCE
        and point.time_s < second_hazard.starts_at_s - _TIME_TOLERANCE
        and not _stopped(point.twist)
    )
    if not resume_points:
        core_failures.append("intermediate_resume_missing")
        intermediate_progress = 0.0
    else:
        progress_at_clear = _progress_at_time(
            world,
            witness,
            first_hazard.ends_at_s,
        )
        intermediate_progress = max(
            0.0,
            max(_reference_progress(world, point.pose) for point in resume_points)
            - progress_at_clear,
        )
        if intermediate_progress < _MINIMUM_INTERMEDIATE_PROGRESS_M - 1e-12:
            core_failures.append("intermediate_progress_insufficient")
    if first_stop is not None and second_stop is not None:
        if not (
            first_hazard.starts_at_s
            <= first_stop.stopped_until_s + _TIME_TOLERANCE
            and first_stop.stopped_from_s
            <= first_hazard.ends_at_s + _TIME_TOLERANCE
            and second_hazard.starts_at_s
            <= second_stop.stopped_until_s + _TIME_TOLERANCE
            and second_stop.stopped_from_s
            <= second_hazard.ends_at_s + _TIME_TOLERANCE
        ):
            core_failures.append("hazard_order_invalid")

    resumed_after_second = any(
        point.time_s > second_hazard.ends_at_s + _TIME_TOLERANCE
        and not _stopped(point.twist)
        for point in witness.points
    )
    if not resumed_after_second:
        recovery_failures.append("post_second_hazard_recovery_missing")
    if not base.passed:
        recovery_failures.append("terminal_recovery_invalid")
    return _validation(
        witness,
        base,
        hazards,
        stops,
        core_failures,
        recovery_failures,
        intermediate_progress,
    )


def _build_restop_candidate(
    world: WitnessWorldSnapshot,
    hazards: tuple[GroundTruthHazardInterval, GroundTruthHazardInterval],
    linear_target_mps: float,
) -> AutomatedWitness | None:
    segments = straight_reference_segments(world)
    if len(segments) != 1:
        return None
    segment = segments[0]
    if abs(_angle(world.initial_state.pose.yaw - segment.tangent_yaw_rad)) > 1e-9:
        return None
    first, second = hazards
    period = world.kinematic_contract.control_period_s
    maximum_tick = round(world.duration_s / period)
    profile = world.kinematic_contract.vehicle_profile
    points = [
        WitnessPoint(
            0.0,
            world.initial_state.pose,
            world.initial_state.twist,
            WitnessPhase.START,
            "restop-start",
        )
    ]
    terminal_started = False
    for tick in range(1, maximum_tick + 1):
        current = points[-1]
        time_s = current.time_s
        remaining = hypot(
            world.goal_pose.x - current.pose.x,
            world.goal_pose.y - current.pose.y,
        )
        stop_distance = _linear_stop_distance(current.twist.linear, world)
        in_first = time_s <= first.ends_at_s + _TIME_TOLERANCE
        in_second = (
            time_s >= second.starts_at_s - _TIME_TOLERANCE
            and time_s <= second.ends_at_s + _TIME_TOLERANCE
        )
        target_linear = (
            0.0
            if in_first or in_second or remaining <= stop_distance + 0.005
            else linear_target_mps
        )
        phase = (
            WitnessPhase.WAIT
            if in_first
            else WitnessPhase.BRAKE_TO_STOP
            if in_second and not _stopped(current.twist)
            else WitnessPhase.WAIT
            if in_second
            else WitnessPhase.FOLLOW_REFERENCE
        )
        next_pose = Pose2D(
            current.pose.x + current.twist.linear * cos(current.pose.yaw) * period,
            current.pose.y + current.twist.linear * sin(current.pose.yaw) * period,
            current.pose.yaw,
        )
        increasing = abs(target_linear) > abs(current.twist.linear) + 1e-12
        rate = (
            profile.max_acceleration_mps2
            if increasing
            else profile.max_deceleration_mps2
        )
        next_twist = Twist2D(
            _slew(current.twist.linear, target_linear, rate * period),
            0.0,
        )
        points.append(
            WitnessPoint(
                round(tick * period, 12),
                next_pose,
                next_twist,
                phase,
                "restop-event-template",
            )
        )
        if (
            time_s > second.ends_at_s
            and remaining <= 0.05
            and _stopped(next_twist)
        ):
            terminal_started = True
            dwell_ticks = round(0.50 / period)
            if tick + dwell_ticks > maximum_tick:
                return None
            for dwell in range(1, dwell_ticks + 1):
                points.append(
                    WitnessPoint(
                        round((tick + dwell) * period, 12),
                        next_pose,
                        Twist2D(),
                        WitnessPhase.TERMINAL_DWELL,
                        "restop-terminal-dwell",
                    )
                )
            break
    if not terminal_started:
        return None
    payload = {
        "search_version": RESTOP_SEARCH_VERSION,
        "source_projection_hash": world.source_projection_hash,
        "hazards": tuple(item.blocking_geometry_hash for item in hazards),
        "linear_target_mps": linear_target_mps,
    }
    return build_automated_witness(
        world,
        witness_id=f"restop-witness-{canonical_content_hash(payload)[:24]}",
        kind=WitnessKind.WAIT_AND_FOLLOW,
        terminal_mode=WitnessTerminalMode.GOAL_DWELL,
        points=tuple(points),
        terminal_dwell_s=0.50,
    )


def _kinematic_stop_intervals(
    witness: AutomatedWitness,
) -> tuple[KinematicStopInterval, ...]:
    result: list[KinematicStopInterval] = []
    start_index: int | None = None
    for index, point in enumerate(witness.points):
        if _stopped(point.twist) and start_index is None:
            start_index = index
        next_moves = index + 1 == len(witness.points) or not _stopped(
            witness.points[index + 1].twist
        )
        if start_index is not None and next_moves:
            start = witness.points[start_index]
            end = point
            result.append(
                KinematicStopInterval(
                    start.time_s,
                    end.time_s,
                    start.pose,
                    any(not _stopped(item.twist) for item in witness.points[:start_index]),
                    any(not _stopped(item.twist) for item in witness.points[index + 1 :]),
                    (),
                )
            )
            start_index = None
    return tuple(result)


def _bind_stop(
    interval: KinematicStopInterval,
    hazards: tuple[GroundTruthHazardInterval, ...],
) -> KinematicStopInterval:
    bound = tuple(
        item.hazard_id
        for item in hazards
        if interval.stopped_from_s <= item.ends_at_s + _TIME_TOLERANCE
        and interval.stopped_until_s >= item.starts_at_s - _TIME_TOLERANCE
    )
    return KinematicStopInterval(
        interval.stopped_from_s,
        interval.stopped_until_s,
        interval.pose,
        interval.preceding_motion_observed,
        interval.following_motion_observed,
        bound,
    )


def _validation(
    witness: AutomatedWitness,
    base: GroundTruthWitnessValidation,
    hazards: tuple[GroundTruthHazardInterval, ...],
    stops: tuple[KinematicStopInterval, ...],
    core_failures: list[str],
    recovery_failures: list[str],
    progress: float,
) -> RestopValidation:
    core = not core_failures
    recovery = core and not recovery_failures
    level = (
        RestopEvidenceLevel.RESTOP_AND_RECOVERY_PROVEN
        if recovery
        else RestopEvidenceLevel.RESTOP_CORE_PROVEN
        if core
        else RestopEvidenceLevel.NONE
    )
    return RestopValidation(
        RESTOP_VALIDATOR_VERSION,
        witness.semantic_content_hash,
        base,
        hazards,
        stops,
        level,
        core,
        recovery,
        tuple(dict.fromkeys(core_failures)),
        tuple(dict.fromkeys(recovery_failures)),
        progress,
    )


def _progress_at_time(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    time_s: float,
) -> float:
    point = min(witness.points, key=lambda item: abs(item.time_s - time_s))
    return _reference_progress(world, point.pose)


def _reference_progress(world: WitnessWorldSnapshot, pose: Pose2D) -> float:
    segment = straight_reference_segments(world)[0]
    return max(
        0.0,
        min(
            segment.length_m,
            (pose.x - segment.source_x_m) * segment.tangent_x
            + (pose.y - segment.source_y_m) * segment.tangent_y,
        ),
    )


def _linear_stop_distance(linear_mps: float, world: WitnessWorldSnapshot) -> float:
    period = world.kinematic_contract.control_period_s
    decrement = (
        world.kinematic_contract.vehicle_profile.max_deceleration_mps2 * period
    )
    speed = abs(linear_mps)
    distance = 0.0
    while speed > _STOP_TOLERANCE:
        distance += speed * period
        speed = max(0.0, speed - decrement)
    return distance


def _slew(current: float, target: float, maximum_delta: float) -> float:
    delta = target - current
    if abs(delta) <= maximum_delta:
        return target
    return current + (maximum_delta if delta > 0.0 else -maximum_delta)


def _stopped(twist: Twist2D) -> bool:
    return abs(twist.linear) <= _STOP_TOLERANCE and abs(twist.angular) <= _STOP_TOLERANCE


def _angle(value: float) -> float:
    from math import pi

    return (value + pi) % (2.0 * pi) - pi


__all__ = [
    "KinematicStopInterval",
    "RESTOP_SEARCH_VERSION",
    "RESTOP_VALIDATOR_VERSION",
    "RestopEvidenceLevel",
    "RestopSearchResult",
    "RestopValidation",
    "search_multi_hazard_restop",
    "validate_multi_hazard_restop",
]
