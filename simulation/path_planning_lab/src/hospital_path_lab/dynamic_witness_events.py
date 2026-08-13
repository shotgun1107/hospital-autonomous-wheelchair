"""Label-free exact geometry events shared by R2-A witness tools."""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, hypot

from hospital_path_lab.dynamic_witness_contracts import (
    WitnessActorTrajectory,
    WitnessWorldSnapshot,
)
from hospital_path_lab.map_factory import canonical_content_hash

_TOLERANCE = 1e-12


@dataclass(frozen=True, slots=True)
class ReferenceSegmentGeometry:
    index: int
    source_x_m: float
    source_y_m: float
    length_m: float
    tangent_x: float
    tangent_y: float
    tangent_yaw_rad: float
    cumulative_start_m: float


@dataclass(frozen=True, slots=True)
class CrossingTargetGeometry:
    actor_binding_id: str
    segment_index: int
    crossing_station_progress_m: float
    blocking_starts_at_s: float
    blocking_ends_at_s: float
    normal_speed_mps: float
    blocking_geometry_hash: str


@dataclass(frozen=True, slots=True)
class GroundTruthHazardInterval:
    hazard_id: str
    actor_binding_ids: tuple[str, ...]
    starts_at_s: float
    ends_at_s: float
    blocking_geometry_hash: str


def straight_reference_segments(
    world: WitnessWorldSnapshot,
) -> tuple[ReferenceSegmentGeometry, ...]:
    result: list[ReferenceSegmentGeometry] = []
    cumulative = 0.0
    for index, (source, target) in enumerate(
        zip(world.reference_path, world.reference_path[1:], strict=False)
    ):
        dx = target.x - source.x
        dy = target.y - source.y
        length = hypot(dx, dy)
        if length <= _TOLERANCE:
            continue
        result.append(
            ReferenceSegmentGeometry(
                index=index,
                source_x_m=source.x,
                source_y_m=source.y,
                length_m=length,
                tangent_x=dx / length,
                tangent_y=dy / length,
                tangent_yaw_rad=atan2(dy, dx),
                cumulative_start_m=cumulative,
            )
        )
        cumulative += length
    return tuple(result)


def crossing_targets(
    world: WitnessWorldSnapshot,
    *,
    minimum_normal_speed_mps: float = 1e-6,
) -> tuple[CrossingTargetGeometry, ...]:
    """Return exact straight-segment lane crossings without evaluator labels."""

    profile = world.kinematic_contract.vehicle_profile
    lateral_margin = (
        profile.collision_width_m / 2.0
        + profile.minimum_clearance_m
    )
    longitudinal_margin = (
        profile.collision_length_m / 2.0
        + profile.minimum_clearance_m
    )
    result: list[CrossingTargetGeometry] = []
    for actor in sorted(world.actors, key=lambda item: item.actor_binding_id):
        for segment in straight_reference_segments(world):
            target = _crossing_target(
                actor,
                segment,
                lateral_margin_m=lateral_margin + actor.radius_m,
                longitudinal_margin_m=longitudinal_margin + actor.radius_m,
                minimum_normal_speed_mps=minimum_normal_speed_mps,
            )
            if target is not None:
                result.append(target)
    return tuple(
        sorted(
            result,
            key=lambda item: (
                item.actor_binding_id,
                item.segment_index,
                item.blocking_starts_at_s,
            ),
        )
    )


def ground_truth_hazard_intervals(
    world: WitnessWorldSnapshot,
) -> tuple[GroundTruthHazardInterval, ...]:
    """Merge overlapping exact reference-lane blocking intervals."""

    raw = list(crossing_targets(world))
    if not raw:
        return ()
    raw.sort(key=lambda item: (item.blocking_starts_at_s, item.blocking_ends_at_s))
    groups: list[list[CrossingTargetGeometry]] = []
    for item in raw:
        if not groups or item.blocking_starts_at_s > max(
            member.blocking_ends_at_s for member in groups[-1]
        ) + _TOLERANCE:
            groups.append([item])
        else:
            groups[-1].append(item)
    intervals: list[GroundTruthHazardInterval] = []
    for group in groups:
        payload = {
            "actors": tuple(sorted(item.actor_binding_id for item in group)),
            "segments": tuple(sorted(item.segment_index for item in group)),
            "starts_at_s": min(item.blocking_starts_at_s for item in group),
            "ends_at_s": max(item.blocking_ends_at_s for item in group),
            "geometry": tuple(item.blocking_geometry_hash for item in group),
        }
        geometry_hash = canonical_content_hash(payload)
        intervals.append(
            GroundTruthHazardInterval(
                hazard_id=f"hazard-{geometry_hash[:16]}",
                actor_binding_ids=payload["actors"],
                starts_at_s=payload["starts_at_s"],
                ends_at_s=payload["ends_at_s"],
                blocking_geometry_hash=geometry_hash,
            )
        )
    return tuple(intervals)


def _crossing_target(
    actor: WitnessActorTrajectory,
    segment: ReferenceSegmentGeometry,
    *,
    lateral_margin_m: float,
    longitudinal_margin_m: float,
    minimum_normal_speed_mps: float,
) -> CrossingTargetGeometry | None:
    relative_x = actor.start_position.x - segment.source_x_m
    relative_y = actor.start_position.y - segment.source_y_m
    along_start = relative_x * segment.tangent_x + relative_y * segment.tangent_y
    normal_start = -segment.tangent_y * relative_x + segment.tangent_x * relative_y
    along_rate = actor.velocity.x * segment.tangent_x + actor.velocity.y * segment.tangent_y
    normal_rate = -segment.tangent_y * actor.velocity.x + segment.tangent_x * actor.velocity.y
    if abs(normal_rate) <= minimum_normal_speed_mps:
        return None
    duration = actor.active_until_s - actor.active_from_s
    lateral = _linear_band_interval(
        normal_start,
        normal_rate,
        -lateral_margin_m,
        lateral_margin_m,
        duration,
    )
    longitudinal = _linear_band_interval(
        along_start,
        along_rate,
        -longitudinal_margin_m,
        segment.length_m + longitudinal_margin_m,
        duration,
    )
    if lateral is None or longitudinal is None:
        return None
    entry = max(lateral[0], longitudinal[0])
    exit_ = min(lateral[1], longitudinal[1])
    if entry > exit_ + _TOLERANCE:
        return None
    crossing_offset_s = -normal_start / normal_rate
    station = along_start + along_rate * min(duration, max(0.0, crossing_offset_s))
    if not (0.0 - _TOLERANCE <= station <= segment.length_m + _TOLERANCE):
        return None
    payload = {
        "actor_binding_id": actor.actor_binding_id,
        "segment_index": segment.index,
        "crossing_station_progress_m": station,
        "blocking_starts_at_s": actor.active_from_s + entry,
        "blocking_ends_at_s": actor.active_from_s + exit_,
        "normal_speed_mps": normal_rate,
        "lateral_margin_m": lateral_margin_m,
        "longitudinal_margin_m": longitudinal_margin_m,
    }
    return CrossingTargetGeometry(
        actor_binding_id=actor.actor_binding_id,
        segment_index=segment.index,
        crossing_station_progress_m=station,
        blocking_starts_at_s=actor.active_from_s + entry,
        blocking_ends_at_s=actor.active_from_s + exit_,
        normal_speed_mps=normal_rate,
        blocking_geometry_hash=canonical_content_hash(payload),
    )


def _linear_band_interval(
    initial: float,
    rate: float,
    lower: float,
    upper: float,
    duration: float,
) -> tuple[float, float] | None:
    if abs(rate) <= 1e-15:
        return (0.0, duration) if lower - _TOLERANCE <= initial <= upper + _TOLERANCE else None
    first = (lower - initial) / rate
    second = (upper - initial) / rate
    entry = max(0.0, min(first, second))
    exit_ = min(duration, max(first, second))
    return (entry, exit_) if entry <= exit_ + _TOLERANCE else None


__all__ = [
    "CrossingTargetGeometry",
    "GroundTruthHazardInterval",
    "ReferenceSegmentGeometry",
    "crossing_targets",
    "ground_truth_hazard_intervals",
    "straight_reference_segments",
]
