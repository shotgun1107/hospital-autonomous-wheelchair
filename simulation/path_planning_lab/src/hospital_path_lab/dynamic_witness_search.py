"""R2의 결정론적 HOLD_ONLY·WAIT_AND_FOLLOW structured search.

이 모듈은 label, category, oracle과 기존 feasible witness를 알지 못한다. 검색 후보는
``WitnessWorldSnapshot``의 공개 기하·ground-truth Actor trajectory만 사용하고, 최종
선택 전에는 독립 ``validate_ground_truth_witness``를 반드시 통과한다.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from enum import Enum
from math import atan2, ceil, cos, hypot, isfinite, pi, sin
from time import perf_counter_ns

from hospital_path_lab.collision import oriented_footprint_circle_surface_distance
from hospital_path_lab.contracts import Pose2D, Twist2D
from hospital_path_lab.dynamic_witness_contracts import (
    FROZEN_WITNESS_SEARCH_CONFIG,
    AutomatedWitness,
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
from hospital_path_lab.dynamic_witness_validation import (
    GroundTruthWitnessValidation,
    validate_ground_truth_witness,
)
from hospital_path_lab.map_factory import canonical_content_hash

WAIT_HOLD_SEARCH_VERSION = "wait-hold-structured-search-v1"
_GOAL_POSITION_TOLERANCE_M = 0.05
_TURN_POSITION_TOLERANCE_M = 0.025
_HEADING_TOLERANCE_RAD = 0.025


@dataclass(frozen=True, slots=True)
class _CandidateSpec:
    kind: WitnessKind
    departure_tick: int
    linear_target_mps: float

    @property
    def frozen_parameter_tuple(self) -> tuple[float, ...]:
        return (float(self.departure_tick), self.linear_target_mps)


@dataclass(frozen=True, slots=True)
class _ValidatedCandidate:
    witness: AutomatedWitness
    validation: GroundTruthWitnessValidation
    objective: WitnessObjective


class _CandidateRejectionKind(Enum):
    GEOMETRY_OR_DURATION = "geometry_or_duration"
    DYNAMIC_UNSAFE = "dynamic_unsafe"


@dataclass(slots=True)
class _CandidateBuildDiagnostics:
    rejection_kind: _CandidateRejectionKind | None = None

    def reject(self, kind: _CandidateRejectionKind) -> None:
        if self.rejection_kind is None:
            self.rejection_kind = kind


def generate_hold_only_witness(world: WitnessWorldSnapshot) -> AutomatedWitness:
    """Generate a full-episode stationary witness; validation decides safety."""

    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    period_s = world.kinematic_contract.control_period_s
    tick_count = round(world.duration_s / period_s)
    if abs(tick_count * period_s - world.duration_s) > 1e-12:
        raise ValueError("world duration must align with the witness control period")
    if tick_count < round(0.50 / period_s):
        raise ValueError("HOLD_ONLY world must cover the terminal dwell")
    if not _twist_stopped(world.initial_state.twist):
        raise ValueError("HOLD_ONLY requires an actually stopped initial state")
    points = tuple(
        WitnessPoint(
            time_s=round(tick * period_s, 12),
            pose=world.initial_state.pose,
            twist=Twist2D(),
            phase=(WitnessPhase.START if tick == 0 else WitnessPhase.HOLD),
            source_primitive_id="hold-full-episode",
        )
        for tick in range(tick_count + 1)
    )
    witness_id = _candidate_id(
        world,
        kind=WitnessKind.HOLD_ONLY,
        departure_tick=tick_count,
        linear_target_mps=0.0,
    )
    return build_automated_witness(
        world,
        witness_id=witness_id,
        kind=WitnessKind.HOLD_ONLY,
        terminal_mode=WitnessTerminalMode.SAFE_HOLD,
        points=points,
        terminal_dwell_s=0.50,
    )


def generate_wait_and_follow_witness(
    world: WitnessWorldSnapshot,
    *,
    departure_tick: int,
    linear_target_mps: float,
) -> AutomatedWitness | None:
    """Build one public WAIT candidate without exposing search diagnostics."""

    return _generate_wait_and_follow_witness(
        world,
        departure_tick=departure_tick,
        linear_target_mps=linear_target_mps,
        diagnostics=_CandidateBuildDiagnostics(),
    )


def _generate_wait_and_follow_witness(
    world: WitnessWorldSnapshot,
    *,
    departure_tick: int,
    linear_target_mps: float,
    diagnostics: _CandidateBuildDiagnostics,
) -> AutomatedWitness | None:
    """Synthesize one stop/wait then piecewise-reference follow candidate.

    ``None`` means this frozen primitive sequence cannot finish inside the episode.
    It does not mean that a general dynamically feasible path is absent.
    """

    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(departure_tick, int) or departure_tick < 0:
        raise ValueError("departure_tick must be a non-negative integer")
    if not isfinite(linear_target_mps) or linear_target_mps <= 0.0:
        raise ValueError("linear_target_mps must be finite and positive")
    profile = world.kinematic_contract.vehicle_profile
    if linear_target_mps > profile.max_forward_speed_mps + 1e-12:
        raise ValueError("linear target exceeds the vehicle profile")

    maximum_tick = round(
        world.duration_s / world.kinematic_contract.control_period_s
    )
    if departure_tick > maximum_tick:
        diagnostics.reject(_CandidateRejectionKind.GEOMETRY_OR_DURATION)
        return None
    points = [
        WitnessPoint(
            time_s=0.0,
            pose=world.initial_state.pose,
            twist=world.initial_state.twist,
            phase=WitnessPhase.START,
            source_primitive_id="initial-state",
        )
    ]
    while not _twist_stopped(points[-1].twist):
        if not _append_tick(
            points,
            target=Twist2D(),
            phase=WitnessPhase.BRAKE_TO_STOP,
            source_primitive_id="initial-brake",
            world=world,
            maximum_tick=maximum_tick,
            diagnostics=diagnostics,
        ):
            return None
    minimum_wait_ticks = round(0.50 / world.kinematic_contract.control_period_s)
    departure_tick = max(
        departure_tick,
        _point_tick(points[-1], world) + minimum_wait_ticks,
    )
    if departure_tick > maximum_tick:
        diagnostics.reject(_CandidateRejectionKind.GEOMETRY_OR_DURATION)
        return None
    while _point_tick(points[-1], world) < departure_tick:
        if not _append_tick(
            points,
            target=Twist2D(),
            phase=WitnessPhase.WAIT,
            source_primitive_id="initial-wait",
            world=world,
            maximum_tick=maximum_tick,
            diagnostics=diagnostics,
        ):
            return None

    for segment_index, target_pose in enumerate(world.reference_path[1:]):
        if not _move_to_pose(
            points,
            target_pose=target_pose,
            linear_target_mps=linear_target_mps,
            segment_index=segment_index,
            world=world,
            maximum_tick=maximum_tick,
            diagnostics=diagnostics,
        ):
            return None
    if not _brake_to_stop(
        points,
        phase=WitnessPhase.FOLLOW_REFERENCE,
        primitive_id="goal-linear-stop",
        world=world,
        maximum_tick=maximum_tick,
        diagnostics=diagnostics,
    ):
        return None
    if not _rotate_to_heading(
        points,
        target_yaw=world.goal_pose.yaw,
        phase=WitnessPhase.FOLLOW_REFERENCE,
        primitive_id="goal-yaw-align",
        world=world,
        maximum_tick=maximum_tick,
        diagnostics=diagnostics,
    ):
        return None
    if hypot(
        points[-1].pose.x - world.goal_pose.x,
        points[-1].pose.y - world.goal_pose.y,
    ) > _GOAL_POSITION_TOLERANCE_M:
        diagnostics.reject(_CandidateRejectionKind.GEOMETRY_OR_DURATION)
        return None
    dwell_ticks = round(0.50 / world.kinematic_contract.control_period_s)
    for _ in range(dwell_ticks):
        if not _append_tick(
            points,
            target=Twist2D(),
            phase=WitnessPhase.TERMINAL_DWELL,
            source_primitive_id="goal-terminal-dwell",
            world=world,
            maximum_tick=maximum_tick,
            diagnostics=diagnostics,
        ):
            return None

    witness_id = _candidate_id(
        world,
        kind=WitnessKind.WAIT_AND_FOLLOW,
        departure_tick=departure_tick,
        linear_target_mps=linear_target_mps,
    )
    return build_automated_witness(
        world,
        witness_id=witness_id,
        kind=WitnessKind.WAIT_AND_FOLLOW,
        terminal_mode=WitnessTerminalMode.GOAL_DWELL,
        points=tuple(points),
        terminal_dwell_s=0.50,
    )


def search_wait_and_hold(
    world: WitnessWorldSnapshot,
    *,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
) -> WitnessSearchResult:
    """Evaluate deterministic event-anchored WAIT candidates and a safe HOLD.

    The event-anchor set is a first R2 structured template, not a complete temporal
    planner. Exhausting it yields ``NO_WITNESS_IN_STRUCTURED_TEMPLATE`` rather than
    a spatial or temporal infeasibility claim.
    """

    started_ns = perf_counter_ns()
    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(search_config, WitnessSearchConfig):
        raise TypeError("search_config must be a WitnessSearchConfig")
    input_reason = _search_input_reason(world, search_config)
    if input_reason is not None:
        return _empty_result(
            world,
            status=WitnessSearchStatus.INVALID_INPUT,
            reason=input_reason,
            elapsed_ns=perf_counter_ns() - started_ns,
            search_config=search_config,
        )

    minimum_departure_tick = _minimum_wait_departure_tick(world)
    departure_ticks = tuple(
        sorted(
            {
                max(anchor_tick, minimum_departure_tick)
                for anchor_tick in _event_anchor_departure_ticks(world)
            }
        )
    )
    if len(departure_ticks) > search_config.max_geometry_candidates_per_episode:
        return _empty_result(
            world,
            status=WitnessSearchStatus.RESOURCE_LIMIT,
            reason="geometry_candidate_limit_reached",
            elapsed_ns=perf_counter_ns() - started_ns,
            search_config=search_config,
        )
    timed_candidate_count = (
        len(departure_ticks) * len(search_config.linear_targets_mps) + 1
    )

    generated_count = 0
    geometry_pruned_count = 0
    dynamic_rejected_count = 0
    validated_count = 0
    best_mission: _ValidatedCandidate | None = None
    best_hold: _ValidatedCandidate | None = None
    if timed_candidate_count > search_config.max_timed_candidates_per_episode:
        return WitnessSearchResult(
            status=WitnessSearchStatus.RESOURCE_LIMIT,
            source_projection_hash=world.source_projection_hash,
            world_content_hash=world.content_hash,
            search_config_hash=search_config.content_hash,
            generated_count=0,
            geometry_pruned_count=0,
            dynamic_rejected_count=0,
            validated_count=0,
            selected_witness=None,
            termination_reason="timed_candidate_limit_reached",
            deterministic_objective=None,
            elapsed_nonqualification_ns=perf_counter_ns() - started_ns,
        )
    for spec in _iter_candidate_specs(world, departure_ticks, search_config):
        generated_count += 1
        diagnostics = _CandidateBuildDiagnostics()
        if spec.kind is WitnessKind.HOLD_ONLY:
            if _twist_stopped(world.initial_state.twist):
                candidate = generate_hold_only_witness(world)
            else:
                diagnostics.reject(_CandidateRejectionKind.GEOMETRY_OR_DURATION)
                candidate = None
        else:
            candidate = _generate_wait_and_follow_witness(
                world,
                departure_tick=spec.departure_tick,
                linear_target_mps=spec.linear_target_mps,
                diagnostics=diagnostics,
            )
        if candidate is None:
            if diagnostics.rejection_kind is _CandidateRejectionKind.DYNAMIC_UNSAFE:
                dynamic_rejected_count += 1
            else:
                geometry_pruned_count += 1
            continue
        validation = validate_ground_truth_witness(world, candidate)
        if not validation.passed:
            if _validation_is_dynamic_rejection(validation.failures):
                dynamic_rejected_count += 1
            else:
                geometry_pruned_count += 1
            continue
        validated_count += 1
        validated_candidate = _ValidatedCandidate(
            witness=candidate,
            validation=validation,
            objective=_objective(candidate, validation, spec),
        )
        if validated_candidate.witness.kind is WitnessKind.WAIT_AND_FOLLOW:
            best_mission = _better_candidate(best_mission, validated_candidate)
        else:
            best_hold = _better_candidate(best_hold, validated_candidate)

    selected = best_mission or best_hold
    if selected is None:
        return WitnessSearchResult(
            status=WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE,
            source_projection_hash=world.source_projection_hash,
            world_content_hash=world.content_hash,
            search_config_hash=search_config.content_hash,
            generated_count=generated_count,
            geometry_pruned_count=geometry_pruned_count,
            dynamic_rejected_count=dynamic_rejected_count,
            validated_count=validated_count,
            selected_witness=None,
            termination_reason="wait_hold_template_exhausted",
            deterministic_objective=None,
            elapsed_nonqualification_ns=perf_counter_ns() - started_ns,
        )
    return WitnessSearchResult(
        status=WitnessSearchStatus.WITNESS_FOUND,
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        search_config_hash=search_config.content_hash,
        generated_count=generated_count,
        geometry_pruned_count=geometry_pruned_count,
        dynamic_rejected_count=dynamic_rejected_count,
        validated_count=validated_count,
        selected_witness=selected.witness,
        termination_reason=(
            "validated_wait_and_follow"
            if selected.witness.kind is WitnessKind.WAIT_AND_FOLLOW
            else "validated_safe_hold_only"
        ),
        deterministic_objective=selected.objective,
        elapsed_nonqualification_ns=perf_counter_ns() - started_ns,
        selected_validation_hash=selected.validation.content_hash,
    )


def _event_anchor_departure_ticks(world: WitnessWorldSnapshot) -> tuple[int, ...]:
    period_s = world.kinematic_contract.control_period_s
    maximum_tick = round(world.duration_s / period_s)
    ticks = {0}
    ticks.add(min(maximum_tick, round(0.50 / period_s)))
    for actor in world.actors:
        for event_time_s in (actor.active_from_s, actor.active_until_s):
            event_tick = round(event_time_s / period_s)
            ticks.add(max(0, min(maximum_tick, event_tick)))
            ticks.add(max(0, min(maximum_tick, event_tick + 1)))
    return tuple(sorted(ticks))


def _minimum_wait_departure_tick(world: WitnessWorldSnapshot) -> int:
    contract = world.kinematic_contract
    profile = contract.vehicle_profile
    period_s = contract.control_period_s
    linear_stop_ticks = ceil(
        abs(world.initial_state.twist.linear)
        / (profile.max_deceleration_mps2 * period_s)
    )
    angular_stop_ticks = ceil(
        abs(world.initial_state.twist.angular)
        / (contract.maximum_angular_acceleration_radps2 * period_s)
    )
    dwell_ticks = round(0.50 / period_s)
    return max(linear_stop_ticks, angular_stop_ticks) + dwell_ticks


def _iter_candidate_specs(
    world: WitnessWorldSnapshot,
    departure_ticks: tuple[int, ...],
    search_config: WitnessSearchConfig,
) -> Iterator[_CandidateSpec]:
    for departure_tick in departure_ticks:
        for linear_target in search_config.linear_targets_mps:
            yield _CandidateSpec(
                kind=WitnessKind.WAIT_AND_FOLLOW,
                departure_tick=departure_tick,
                linear_target_mps=linear_target,
            )
    yield _CandidateSpec(
        kind=WitnessKind.HOLD_ONLY,
        departure_tick=round(
            world.duration_s / world.kinematic_contract.control_period_s
        ),
        linear_target_mps=0.0,
    )


def _better_candidate(
    current: _ValidatedCandidate | None,
    candidate: _ValidatedCandidate,
) -> _ValidatedCandidate:
    if current is None:
        return candidate
    candidate_key = (
        candidate.objective.sort_key,
        candidate.witness.semantic_content_hash,
    )
    current_key = (
        current.objective.sort_key,
        current.witness.semantic_content_hash,
    )
    return candidate if candidate_key < current_key else current


def _search_input_reason(
    world: WitnessWorldSnapshot,
    search_config: WitnessSearchConfig,
) -> str | None:
    if world.search_config_hash != search_config.content_hash:
        return "search_config_hash_mismatch"
    if world.duration_s < 0.50:
        return "world_too_short_for_terminal_dwell"
    period_s = world.kinematic_contract.control_period_s
    if abs(world.duration_s / period_s - round(world.duration_s / period_s)) > 1e-12:
        return "world_duration_not_on_control_grid"
    profile = world.kinematic_contract.vehicle_profile
    initial_twist = world.initial_state.twist
    if not all((isfinite(initial_twist.linear), isfinite(initial_twist.angular))):
        return "initial_twist_not_finite"
    if not (
        -profile.max_reverse_speed_mps - 1e-12
        <= initial_twist.linear
        <= profile.max_forward_speed_mps + 1e-12
    ):
        return "initial_linear_speed_exceeds_vehicle_profile"
    if abs(initial_twist.angular) > profile.max_angular_speed_radps + 1e-12:
        return "initial_angular_speed_exceeds_vehicle_profile"
    if any(
        target > profile.max_forward_speed_mps + 1e-12
        for target in search_config.linear_targets_mps
    ):
        return "linear_target_exceeds_vehicle_profile"
    if search_config.reverse_enabled:
        return "wait_hold_v1_reverse_must_remain_disabled"
    return None


def _objective(
    witness: AutomatedWitness,
    validation: GroundTruthWitnessValidation,
    spec: _CandidateSpec,
) -> WitnessObjective:
    kind_rank = {
        WitnessKind.WAIT_AND_FOLLOW: 0,
        WitnessKind.PASS_LEFT: 1,
        WitnessKind.PASS_RIGHT: 2,
        WitnessKind.HOLD_ONLY: 3,
    }[witness.kind]
    return WitnessObjective(
        hard_failure_count=len(validation.failures),
        terminal_completion_time_s=witness.points[-1].time_s,
        actual_path_length_m=validation.metrics.actual_path_length_m,
        maximum_reference_deviation_m=(
            validation.metrics.maximum_reference_deviation_m
        ),
        full_stop_count=validation.metrics.full_stop_count,
        absolute_angular_travel_rad=validation.metrics.absolute_angular_travel_rad,
        kind_rank=kind_rank,
        frozen_parameter_tuple=spec.frozen_parameter_tuple,
    )


def _empty_result(
    world: WitnessWorldSnapshot,
    *,
    status: WitnessSearchStatus,
    reason: str,
    elapsed_ns: int,
    search_config: WitnessSearchConfig,
) -> WitnessSearchResult:
    return WitnessSearchResult(
        status=status,
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        search_config_hash=search_config.content_hash,
        generated_count=0,
        geometry_pruned_count=0,
        dynamic_rejected_count=0,
        validated_count=0,
        selected_witness=None,
        termination_reason=reason,
        deterministic_objective=None,
        elapsed_nonqualification_ns=elapsed_ns,
    )


def _move_to_pose(
    points: list[WitnessPoint],
    *,
    target_pose: Pose2D,
    linear_target_mps: float,
    segment_index: int,
    world: WitnessWorldSnapshot,
    maximum_tick: int,
    diagnostics: _CandidateBuildDiagnostics,
) -> bool:
    phase_ticks = 0
    while hypot(
        target_pose.x - points[-1].pose.x,
        target_pose.y - points[-1].pose.y,
    ) > _TURN_POSITION_TOLERANCE_M:
        if phase_ticks > maximum_tick + 1:
            diagnostics.reject(_CandidateRejectionKind.GEOMETRY_OR_DURATION)
            return False
        if not _brake_to_stop(
            points,
            phase=WitnessPhase.FOLLOW_REFERENCE,
            primitive_id=f"segment-{segment_index}-pre-turn-stop",
            world=world,
            maximum_tick=maximum_tick,
            diagnostics=diagnostics,
        ):
            return False
        target_yaw = atan2(
            target_pose.y - points[-1].pose.y,
            target_pose.x - points[-1].pose.x,
        )
        if not _rotate_to_heading(
            points,
            target_yaw=target_yaw,
            phase=WitnessPhase.FOLLOW_REFERENCE,
            primitive_id=f"segment-{segment_index}-align",
            world=world,
            maximum_tick=maximum_tick,
            diagnostics=diagnostics,
        ):
            return False
        advance_unit_x = cos(target_yaw)
        advance_unit_y = sin(target_yaw)
        while True:
            current = points[-1]
            remaining_m = hypot(
                target_pose.x - current.pose.x,
                target_pose.y - current.pose.y,
            )
            if _twist_stopped(current.twist) and remaining_m <= (
                _TURN_POSITION_TOLERANCE_M
            ):
                return True
            stop_distance_m = _discrete_linear_stop_distance(
                current.twist.linear,
                world,
            )
            signed_remaining_m = (
                (target_pose.x - current.pose.x) * advance_unit_x
                + (target_pose.y - current.pose.y) * advance_unit_y
            )
            target_linear = (
                0.0
                if signed_remaining_m <= stop_distance_m + 0.005
                else linear_target_mps
            )
            if not _append_tick(
                points,
                target=Twist2D(linear=target_linear, angular=0.0),
                phase=WitnessPhase.FOLLOW_REFERENCE,
                source_primitive_id=f"segment-{segment_index}-advance",
                world=world,
                maximum_tick=maximum_tick,
                ground_truth_guard=True,
                diagnostics=diagnostics,
            ):
                return False
            phase_ticks += 1
            if phase_ticks > maximum_tick + 1:
                return False
            if _twist_stopped(points[-1].twist):
                break
    return True


def _brake_to_stop(
    points: list[WitnessPoint],
    *,
    phase: WitnessPhase,
    primitive_id: str,
    world: WitnessWorldSnapshot,
    maximum_tick: int,
    diagnostics: _CandidateBuildDiagnostics,
) -> bool:
    count = 0
    while not _twist_stopped(points[-1].twist):
        if count > maximum_tick + 1:
            diagnostics.reject(_CandidateRejectionKind.GEOMETRY_OR_DURATION)
            return False
        if not _append_tick(
            points,
            target=Twist2D(),
            phase=phase,
            source_primitive_id=primitive_id,
            world=world,
            maximum_tick=maximum_tick,
            diagnostics=diagnostics,
        ):
            return False
        count += 1
    return True


def _rotate_to_heading(
    points: list[WitnessPoint],
    *,
    target_yaw: float,
    phase: WitnessPhase,
    primitive_id: str,
    world: WitnessWorldSnapshot,
    maximum_tick: int,
    diagnostics: _CandidateBuildDiagnostics,
) -> bool:
    if abs(points[-1].twist.linear) > 1e-12:
        diagnostics.reject(_CandidateRejectionKind.GEOMETRY_OR_DURATION)
        return False
    acceleration = world.kinematic_contract.maximum_angular_acceleration_radps2
    maximum_angular = world.kinematic_contract.vehicle_profile.max_angular_speed_radps
    period_s = world.kinematic_contract.control_period_s
    for _ in range(maximum_tick + 2):
        current = points[-1]
        error = _normalize_angle(target_yaw - current.pose.yaw)
        if abs(error) <= _HEADING_TOLERANCE_RAD and abs(current.twist.angular) <= 1e-12:
            return True
        direction = 1.0 if error >= 0.0 else -1.0
        angular = current.twist.angular
        stop_angle = _discrete_angular_stop_angle(angular, acceleration, period_s)
        target_angular = (
            0.0
            if angular * direction < -1e-12
            or abs(error) <= abs(stop_angle) + 0.002
            else direction * maximum_angular
        )
        if not _append_tick(
            points,
            target=Twist2D(0.0, target_angular),
            phase=phase,
            source_primitive_id=primitive_id,
            world=world,
            maximum_tick=maximum_tick,
            ground_truth_guard=True,
            diagnostics=diagnostics,
        ):
            return False
    diagnostics.reject(_CandidateRejectionKind.GEOMETRY_OR_DURATION)
    return False


def _append_tick(
    points: list[WitnessPoint],
    *,
    target: Twist2D,
    phase: WitnessPhase,
    source_primitive_id: str,
    world: WitnessWorldSnapshot,
    maximum_tick: int,
    ground_truth_guard: bool = False,
    diagnostics: _CandidateBuildDiagnostics,
) -> bool:
    current = points[-1]
    if _point_tick(current, world) >= maximum_tick:
        diagnostics.reject(_CandidateRejectionKind.GEOMETRY_OR_DURATION)
        return False
    period_s = world.kinematic_contract.control_period_s
    if ground_truth_guard and not _target_is_safely_stoppable(
        current,
        target,
        world,
    ):
        braking_target = Twist2D()
        if not _target_is_safely_stoppable(current, braking_target, world):
            diagnostics.reject(_CandidateRejectionKind.DYNAMIC_UNSAFE)
            return False
        target = braking_target
        phase = WitnessPhase.WAIT
        source_primitive_id = "ground-truth-actor-wait"
    next_pose = _integrate_pose(current.pose, current.twist, period_s)
    next_twist = Twist2D(
        linear=_slew_linear(current.twist.linear, target.linear, world),
        angular=_slew_scalar(
            current.twist.angular,
            target.angular,
            world.kinematic_contract.maximum_angular_acceleration_radps2 * period_s,
        ),
    )
    points.append(
        WitnessPoint(
            time_s=round(current.time_s + period_s, 12),
            pose=next_pose,
            twist=next_twist,
            phase=phase,
            source_primitive_id=source_primitive_id,
        )
    )
    return True


def _validation_is_dynamic_rejection(failures: tuple[str, ...]) -> bool:
    return any("actor" in failure for failure in failures)


def _slew_linear(
    current: float,
    target: float,
    world: WitnessWorldSnapshot,
) -> float:
    profile = world.kinematic_contract.vehicle_profile
    period_s = world.kinematic_contract.control_period_s
    if current * target < -1e-12:
        target = 0.0
    increasing = abs(target) > abs(current) + 1e-12
    rate = profile.max_acceleration_mps2 if increasing else profile.max_deceleration_mps2
    return _slew_scalar(current, target, rate * period_s)


def _slew_scalar(current: float, target: float, maximum_delta: float) -> float:
    delta = target - current
    if abs(delta) <= maximum_delta:
        return target
    return current + (maximum_delta if delta > 0.0 else -maximum_delta)


def _discrete_linear_stop_distance(
    linear_mps: float,
    world: WitnessWorldSnapshot,
) -> float:
    period_s = world.kinematic_contract.control_period_s
    decrement = (
        world.kinematic_contract.vehicle_profile.max_deceleration_mps2 * period_s
    )
    speed = abs(linear_mps)
    distance_m = 0.0
    while speed > 1e-12:
        distance_m += speed * period_s
        speed = max(0.0, speed - decrement)
    return distance_m


def _discrete_angular_stop_angle(
    angular_radps: float,
    acceleration_radps2: float,
    period_s: float,
) -> float:
    direction = 1.0 if angular_radps >= 0.0 else -1.0
    speed = abs(angular_radps)
    angle = 0.0
    decrement = acceleration_radps2 * period_s
    while speed > 1e-12:
        angle += speed * period_s
        speed = max(0.0, speed - decrement)
    return direction * angle


def _point_tick(point: WitnessPoint, world: WitnessWorldSnapshot) -> int:
    return round(point.time_s / world.kinematic_contract.control_period_s)


def _integrate_pose(pose: Pose2D, twist: Twist2D, dt_s: float) -> Pose2D:
    return Pose2D(
        pose.x + twist.linear * cos(pose.yaw) * dt_s,
        pose.y + twist.linear * sin(pose.yaw) * dt_s,
        _normalize_angle(pose.yaw + twist.angular * dt_s),
    )


def _candidate_id(
    world: WitnessWorldSnapshot,
    *,
    kind: WitnessKind,
    departure_tick: int,
    linear_target_mps: float,
) -> str:
    payload = {
        "search_version": WAIT_HOLD_SEARCH_VERSION,
        "source_projection_hash": world.source_projection_hash,
        "kind": kind,
        "departure_tick": departure_tick,
        "linear_target_mps": linear_target_mps,
    }
    return f"witness-{canonical_content_hash(payload)[:24]}"


def _target_is_safely_stoppable(
    point: WitnessPoint,
    target: Twist2D,
    world: WitnessWorldSnapshot,
) -> bool:
    if not world.actors:
        return True
    profile = world.kinematic_contract.vehicle_profile
    evaluator_period_s = world.kinematic_contract.evaluator_period_s
    control_period_s = world.kinematic_contract.control_period_s
    half_diagonal_m = hypot(
        profile.collision_length_m / 2.0,
        profile.collision_width_m / 2.0,
    )
    pose = point.pose
    twist = point.twist
    time_s = point.time_s
    next_target = target
    linear_stop_ticks = ceil(
        abs(twist.linear)
        / (profile.max_deceleration_mps2 * control_period_s)
    )
    angular_stop_ticks = ceil(
        abs(twist.angular)
        / (
            world.kinematic_contract.maximum_angular_acceleration_radps2
            * control_period_s
        )
    )
    for _ in range(max(linear_stop_ticks, angular_stop_ticks) + 2):
        subdivisions = round(control_period_s / evaluator_period_s)
        robot_speed_bound = abs(twist.linear) + abs(twist.angular) * half_diagonal_m
        for subdivision in range(subdivisions):
            offset_s = subdivision * evaluator_period_s
            sample_time_s = time_s + offset_s
            if sample_time_s > world.duration_s + 1e-12:
                return False
            sample_pose = _integrate_pose(pose, twist, offset_s)
            for actor in world.actor_states_at(min(sample_time_s, world.duration_s)):
                clearance_m = oriented_footprint_circle_surface_distance(
                    sample_pose,
                    circle_center=(actor.position.x, actor.position.y),
                    circle_radius_m=actor.radius_m,
                    profile=profile,
                ) - (
                    (robot_speed_bound + actor.velocity.magnitude)
                    * evaluator_period_s
                    / 2.0
                )
                if clearance_m < profile.minimum_clearance_m - 1e-9:
                    return False
        pose = _integrate_pose(pose, twist, control_period_s)
        time_s = round(time_s + control_period_s, 12)
        twist = Twist2D(
            linear=_slew_linear(twist.linear, next_target.linear, world),
            angular=_slew_scalar(
                twist.angular,
                next_target.angular,
                world.kinematic_contract.maximum_angular_acceleration_radps2
                * control_period_s,
            ),
        )
        next_target = Twist2D()
        if _twist_stopped(twist):
            return True
    raise RuntimeError("derived terminal stopping bound was insufficient")


def _twist_stopped(twist: Twist2D) -> bool:
    return abs(twist.linear) <= 1e-12 and abs(twist.angular) <= 1e-12


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


__all__ = [
    "WAIT_HOLD_SEARCH_VERSION",
    "generate_hold_only_witness",
    "generate_wait_and_follow_witness",
    "search_wait_and_hold",
]
