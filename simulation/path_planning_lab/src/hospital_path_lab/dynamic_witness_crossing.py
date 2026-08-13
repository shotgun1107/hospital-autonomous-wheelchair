"""R2-A structured crossing-Actor bypass search.

This is an offline ground-truth research tool.  It does not grant online motion
authority and does not reuse evaluator labels as search input.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin
from time import perf_counter_ns

from hospital_path_lab.contracts import Pose2D, Twist2D
from hospital_path_lab.dynamic_witness_contracts import (
    FROZEN_WITNESS_SEARCH_CONFIG,
    AutomatedWitness,
    PassSide,
    WitnessKind,
    WitnessObjective,
    WitnessPhase,
    WitnessPoint,
    WitnessSearchConfig,
    WitnessSearchResult,
    WitnessSearchStatus,
    WitnessTerminalMode,
    WitnessWorldSnapshot,
    build_automated_witness,
)
from hospital_path_lab.dynamic_witness_events import (
    CrossingTargetGeometry,
    crossing_targets,
    straight_reference_segments,
)
from hospital_path_lab.dynamic_witness_validation import (
    GroundTruthWitnessValidation,
    canonicalize_and_validate_ground_truth_crossing_bypass,
)
from hospital_path_lab.map_factory import canonical_content_hash

CROSSING_BYPASS_SEARCH_VERSION = "crossing-bypass-structured-search-v1"
_WAYPOINT_REACHED_M = 0.10
_GOAL_TOLERANCE_M = 0.05
_HEADING_TOLERANCE_RAD = 0.10
_STEERING_GAIN = 2.0


@dataclass(frozen=True, slots=True)
class CrossingBypassSearchResult:
    left: WitnessSearchResult
    right: WitnessSearchResult
    limitations: tuple[str, ...]
    elapsed_nonqualification_ns: int

    @property
    def semantic_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "search_version": CROSSING_BYPASS_SEARCH_VERSION,
                "left": self.left,
                "right": self.right,
                "limitations": self.limitations,
            }
        )


@dataclass(frozen=True, slots=True)
class _Candidate:
    witness: AutomatedWitness
    validation: GroundTruthWitnessValidation
    objective: WitnessObjective


def search_crossing_bypass(
    world: WitnessWorldSnapshot,
    *,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
) -> CrossingBypassSearchResult:
    """Search both signed sides of one-segment crossing targets."""

    started = perf_counter_ns()
    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(search_config, WitnessSearchConfig):
        raise TypeError("search_config must be a WitnessSearchConfig")
    if world.search_config_hash != search_config.content_hash:
        empty = _empty(world, search_config, "search_config_hash_mismatch")
        return CrossingBypassSearchResult(
            empty,
            empty,
            ("crossing_search_invalid_input",),
            perf_counter_ns() - started,
        )
    targets = crossing_targets(world)
    if not targets:
        empty = _empty(world, search_config, "no_eligible_crossing_target")
        return CrossingBypassSearchResult(
            empty,
            empty,
            ("structured_crossing_template_is_not_pose_space_complete",),
            perf_counter_ns() - started,
        )
    results = {
        side: _search_side(world, targets, side, search_config)
        for side in (PassSide.LEFT, PassSide.RIGHT)
    }
    return CrossingBypassSearchResult(
        results[PassSide.LEFT],
        results[PassSide.RIGHT],
        ("structured_crossing_template_is_not_pose_space_complete",),
        perf_counter_ns() - started,
    )


def _search_side(
    world: WitnessWorldSnapshot,
    targets: tuple[CrossingTargetGeometry, ...],
    side: PassSide,
    config: WitnessSearchConfig,
) -> WitnessSearchResult:
    generated = geometry_rejected = dynamic_rejected = validated = 0
    best: _Candidate | None = None
    profile = world.kinematic_contract.vehicle_profile
    base_offset = (
        profile.collision_width_m / 2.0
        + max(actor.radius_m for actor in world.actors)
        + profile.minimum_clearance_m
        + 3.0 * world.grid.resolution_m
    )
    offsets = tuple(round(base_offset + index * 0.10, 12) for index in range(4))
    distances = (0.30, 0.50, 0.70)
    speeds = tuple(reversed(config.linear_targets_mps[-2:]))
    for target in targets:
        for offset in offsets:
            for before_m in distances:
                for after_m in distances:
                    for speed in speeds:
                        generated += 1
                        if generated > config.max_geometry_candidates_per_episode:
                            return _result(
                                world,
                                config,
                                generated - 1,
                                geometry_rejected,
                                dynamic_rejected,
                                validated,
                                best,
                                WitnessSearchStatus.RESOURCE_LIMIT,
                                "crossing_geometry_candidate_limit_reached",
                            )
                        draft = _build_candidate(
                            world,
                            target=target,
                            side=side,
                            lateral_offset_m=offset,
                            before_station_m=before_m,
                            after_station_m=after_m,
                            linear_target_mps=speed,
                        )
                        if draft is None:
                            geometry_rejected += 1
                            continue
                        canonical, validation = (
                            canonicalize_and_validate_ground_truth_crossing_bypass(
                                world,
                                draft,
                            )
                        )
                        if canonical is None:
                            if any("actor" in failure for failure in validation.failures):
                                dynamic_rejected += 1
                            else:
                                geometry_rejected += 1
                            continue
                        validated += 1
                        objective = WitnessObjective(
                            hard_failure_count=0,
                            terminal_completion_time_s=canonical.points[-1].time_s,
                            actual_path_length_m=validation.metrics.actual_path_length_m,
                            maximum_reference_deviation_m=(
                                validation.metrics.maximum_reference_deviation_m
                            ),
                            full_stop_count=validation.metrics.full_stop_count,
                            absolute_angular_travel_rad=(
                                validation.metrics.absolute_angular_travel_rad
                            ),
                            kind_rank=0 if side is PassSide.LEFT else 1,
                            frozen_parameter_tuple=(
                                offset,
                                before_m,
                                after_m,
                                speed,
                            ),
                        )
                        candidate = _Candidate(canonical, validation, objective)
                        if best is None or (
                            candidate.objective.sort_key,
                            candidate.witness.semantic_content_hash,
                        ) < (
                            best.objective.sort_key,
                            best.witness.semantic_content_hash,
                        ):
                            best = candidate
    status = (
        WitnessSearchStatus.WITNESS_FOUND
        if best is not None
        else WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE
    )
    reason = (
        "crossing_bypass_found"
        if best is not None
        else "no_witness_in_crossing_template"
    )
    return _result(
        world,
        config,
        generated,
        geometry_rejected,
        dynamic_rejected,
        validated,
        best,
        status,
        reason,
    )


def _build_candidate(
    world: WitnessWorldSnapshot,
    *,
    target: CrossingTargetGeometry,
    side: PassSide,
    lateral_offset_m: float,
    before_station_m: float,
    after_station_m: float,
    linear_target_mps: float,
) -> AutomatedWitness | None:
    segments = {item.index: item for item in straight_reference_segments(world)}
    segment = segments[target.segment_index]
    departure_progress = target.crossing_station_progress_m - before_station_m
    bypass_progress = target.crossing_station_progress_m + after_station_m
    if departure_progress <= 0.0 or bypass_progress >= segment.length_m:
        return None
    side_sign = 1.0 if side is PassSide.LEFT else -1.0

    def point(progress: float, offset: float) -> tuple[float, float]:
        return (
            segment.source_x_m
            + segment.tangent_x * progress
            - segment.tangent_y * offset,
            segment.source_y_m
            + segment.tangent_y * progress
            + segment.tangent_x * offset,
        )

    waypoints = (
        point(departure_progress, side_sign * lateral_offset_m),
        point(bypass_progress, side_sign * lateral_offset_m),
        (world.goal_pose.x, world.goal_pose.y),
    )
    points = [
        WitnessPoint(
            0.0,
            world.initial_state.pose,
            world.initial_state.twist,
            WitnessPhase.START,
            "crossing-start",
        )
    ]
    waypoint_index = 0
    maximum_tick = round(world.duration_s / world.kinematic_contract.control_period_s)
    profile = world.kinematic_contract.vehicle_profile
    period = world.kinematic_contract.control_period_s
    for tick in range(1, maximum_tick + 1):
        current = points[-1]
        target_x, target_y = waypoints[waypoint_index]
        distance = hypot(target_x - current.pose.x, target_y - current.pose.y)
        if distance < _WAYPOINT_REACHED_M and waypoint_index < len(waypoints) - 1:
            waypoint_index += 1
            target_x, target_y = waypoints[waypoint_index]
            distance = hypot(target_x - current.pose.x, target_y - current.pose.y)
        desired_yaw = (
            atan2(target_y - current.pose.y, target_x - current.pose.x)
            if distance > 0.01
            else world.goal_pose.yaw
        )
        heading_error = _angle(desired_yaw - current.pose.yaw)
        stop_distance = _linear_stop_distance(current.twist.linear, world)
        at_final = waypoint_index == len(waypoints) - 1
        target_linear = (
            0.0
            if at_final and distance <= stop_distance + 0.04
            else linear_target_mps
            * max(0.20, min(1.0, 1.0 - abs(heading_error) / 1.30))
        )
        target_angular = max(
            -profile.max_angular_speed_radps,
            min(profile.max_angular_speed_radps, _STEERING_GAIN * heading_error),
        )
        if at_final and distance < _GOAL_TOLERANCE_M:
            target_linear = 0.0
            target_angular = max(
                -profile.max_angular_speed_radps,
                min(
                    profile.max_angular_speed_radps,
                    _STEERING_GAIN * _angle(world.goal_pose.yaw - current.pose.yaw),
                ),
            )
        next_pose = Pose2D(
            current.pose.x + current.twist.linear * cos(current.pose.yaw) * period,
            current.pose.y + current.twist.linear * sin(current.pose.yaw) * period,
            _angle(current.pose.yaw + current.twist.angular * period),
        )
        increasing = abs(target_linear) > abs(current.twist.linear) + 1e-12
        linear_rate = (
            profile.max_acceleration_mps2
            if increasing
            else profile.max_deceleration_mps2
        )
        next_twist = Twist2D(
            _slew(current.twist.linear, target_linear, linear_rate * period),
            _slew(
                current.twist.angular,
                target_angular,
                world.kinematic_contract.maximum_angular_acceleration_radps2
                * period,
            ),
        )
        phase = (
            WitnessPhase.CROSSING_BYPASS
            if waypoint_index < 2
            else WitnessPhase.REJOIN
        )
        points.append(
            WitnessPoint(
                round(tick * period, 12),
                next_pose,
                next_twist,
                phase,
                f"crossing-waypoint-{waypoint_index}",
            )
        )
        stopped_at_goal = (
            waypoint_index == len(waypoints) - 1
            and hypot(next_pose.x - world.goal_pose.x, next_pose.y - world.goal_pose.y)
            <= _GOAL_TOLERANCE_M
            and abs(_angle(next_pose.yaw - world.goal_pose.yaw))
            <= _HEADING_TOLERANCE_RAD
            and _stopped(next_twist)
        )
        if stopped_at_goal:
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
                        "crossing-terminal-dwell",
                    )
                )
            payload = {
                "search_version": CROSSING_BYPASS_SEARCH_VERSION,
                "source_projection_hash": world.source_projection_hash,
                "target": target,
                "side": side,
                "lateral_offset_m": lateral_offset_m,
                "before_station_m": before_station_m,
                "after_station_m": after_station_m,
                "linear_target_mps": linear_target_mps,
            }
            return build_automated_witness(
                world,
                witness_id=f"crossing-witness-{canonical_content_hash(payload)[:24]}",
                kind=(
                    WitnessKind.CROSSING_BYPASS_LEFT
                    if side is PassSide.LEFT
                    else WitnessKind.CROSSING_BYPASS_RIGHT
                ),
                terminal_mode=WitnessTerminalMode.GOAL_DWELL,
                points=tuple(points),
                required_pass_actor_ids=(target.actor_binding_id,),
                terminal_dwell_s=0.50,
            )
    return None


def _result(
    world: WitnessWorldSnapshot,
    config: WitnessSearchConfig,
    generated: int,
    geometry: int,
    dynamic: int,
    validated: int,
    best: _Candidate | None,
    status: WitnessSearchStatus,
    reason: str,
) -> WitnessSearchResult:
    return WitnessSearchResult(
        status=status,
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        search_config_hash=config.content_hash,
        generated_count=generated,
        geometry_pruned_count=geometry,
        dynamic_rejected_count=dynamic,
        validated_count=validated,
        selected_witness=None if best is None else best.witness,
        termination_reason=reason,
        deterministic_objective=None if best is None else best.objective,
        elapsed_nonqualification_ns=0,
        selected_validation_hash=(
            None if best is None else best.validation.content_hash
        ),
    )


def _empty(
    world: WitnessWorldSnapshot,
    config: WitnessSearchConfig,
    reason: str,
) -> WitnessSearchResult:
    status = (
        WitnessSearchStatus.INVALID_INPUT
        if reason == "search_config_hash_mismatch"
        else WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE
    )
    return _result(world, config, 0, 0, 0, 0, None, status, reason)


def _linear_stop_distance(linear_mps: float, world: WitnessWorldSnapshot) -> float:
    period = world.kinematic_contract.control_period_s
    decrement = (
        world.kinematic_contract.vehicle_profile.max_deceleration_mps2 * period
    )
    speed = abs(linear_mps)
    distance = 0.0
    while speed > 1e-12:
        distance += speed * period
        speed = max(0.0, speed - decrement)
    return distance


def _slew(current: float, target: float, maximum_delta: float) -> float:
    delta = target - current
    if abs(delta) <= maximum_delta:
        return target
    return current + (maximum_delta if delta > 0.0 else -maximum_delta)


def _angle(value: float) -> float:
    return (value + pi) % (2.0 * pi) - pi


def _stopped(twist: Twist2D) -> bool:
    return abs(twist.linear) <= 1e-12 and abs(twist.angular) <= 1e-12


__all__ = [
    "CROSSING_BYPASS_SEARCH_VERSION",
    "CrossingBypassSearchResult",
    "search_crossing_bypass",
]
