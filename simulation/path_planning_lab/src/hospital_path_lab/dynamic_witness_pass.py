"""결정론적 좌·우 PASS structured witness 탐색.

이 모듈은 ``WitnessWorldSnapshot``의 공개 기하와 offline ground-truth Actor 궤적만
사용한다. corpus label, evaluator oracle과 기존 수동 feasible witness는 입력도 import도
하지 않는다. 검색 중 만든 draft는 독립 validator로 사건을 측정한 뒤 canonical 선언을
채우고 strict validator를 한 번 더 통과해야만 결과에 들어간다.
"""

from __future__ import annotations

from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from dataclasses import dataclass
from functools import lru_cache
from itertools import islice
from math import atan2, ceil, cos, floor, hypot, isfinite, pi, sin
from multiprocessing import get_context
from time import perf_counter_ns

from hospital_path_lab.collision import oriented_footprint_circle_surface_distance
from hospital_path_lab.contracts import Pose2D, Twist2D
from hospital_path_lab.dynamic_witness_contracts import (
    FROZEN_WITNESS_SEARCH_CONFIG,
    PASS_STRUCTURED_SEARCH_VERSION,
    WITNESS_SEARCH_CONFIG_VERSION,
    AutomatedWitness,
    PassCandidateCounts,
    PassingPolicy,
    PassSide,
    PassSideSearchResult,
    PassSideWaitPolicy,
    PassStructuredSearchResult,
    WitnessActorTrajectory,
    WitnessKind,
    WitnessObjective,
    WitnessPhase,
    WitnessPoint,
    WitnessSearchConfig,
    WitnessSearchStatus,
    WitnessTerminalMode,
    WitnessWorldSnapshot,
    build_automated_witness,
    build_pass_candidate_parameter_hash,
)
from hospital_path_lab.dynamic_witness_validation import (
    GroundTruthWitnessValidation,
    canonicalize_and_validate_ground_truth_pass,
    validate_ground_truth_witness,
)
from hospital_path_lab.map_factory import canonical_content_hash


@dataclass(frozen=True, slots=True)
class PassCandidateRequest:
    """한 frozen grid 후보의 label-free 외부 요청."""

    actor_binding_id: str
    side: PassSide
    departure_progress_m: float
    lateral_offset_m: float
    release_tick: int
    linear_target_mps: float
    angular_magnitude_radps: float
    wait_policy: PassSideWaitPolicy

    def __post_init__(self) -> None:
        if not self.actor_binding_id:
            raise ValueError("PASS Actor binding must not be empty")
        if not isinstance(self.side, PassSide):
            raise TypeError("side must be a PassSide")
        if not isinstance(self.wait_policy, PassSideWaitPolicy):
            raise TypeError("wait_policy must be a PassSideWaitPolicy")
        if type(self.release_tick) is not int or self.release_tick < 0:
            raise ValueError("release_tick must be a non-negative integer")
        values = (
            self.departure_progress_m,
            self.lateral_offset_m,
            self.linear_target_mps,
            self.angular_magnitude_radps,
        )
        if not all(type(value) is float and isfinite(value) for value in values):
            raise TypeError("PASS numeric request fields must be finite exact floats")
        if min(values[1:]) <= 0.0:
            raise ValueError("PASS offset and speed magnitudes must be positive")


@dataclass(frozen=True, slots=True)
class _ReferenceSegment:
    index: int
    source: Pose2D
    target: Pose2D
    length_m: float
    tangent_x: float
    tangent_y: float
    tangent_yaw: float
    cumulative_start_m: float

    def pose(self, progress_on_segment_m: float, offset_m: float = 0.0) -> Pose2D:
        return Pose2D(
            self.source.x + self.tangent_x * progress_on_segment_m - self.tangent_y * offset_m,
            self.source.y + self.tangent_y * progress_on_segment_m + self.tangent_x * offset_m,
            self.tangent_yaw,
        )


@dataclass(frozen=True, slots=True)
class _Projection:
    segment: _ReferenceSegment
    progress_on_segment_m: float
    total_progress_m: float
    signed_offset_m: float
    distance_m: float


@dataclass(frozen=True, slots=True)
class _Target:
    actor: WitnessActorTrajectory
    segment: _ReferenceSegment
    initial_progress_m: float
    tangent_speed_mps: float
    signed_offset_m: float


@dataclass(frozen=True, slots=True)
class _CandidateSpec:
    target: _Target
    side: PassSide
    departure_progress_m: float
    lateral_offset_m: float
    release_tick: int
    linear_target_mps: float
    angular_magnitude_radps: float
    wait_policy: PassSideWaitPolicy

    @property
    def frozen_parameter_tuple(self) -> tuple[float, ...]:
        return (
            float(self.target.segment.index),
            0.0 if self.side is PassSide.LEFT else 1.0,
            self.departure_progress_m,
            self.lateral_offset_m,
            float(self.release_tick),
            self.linear_target_mps,
            self.angular_magnitude_radps,
            0.0 if self.wait_policy is PassSideWaitPolicy.IMMEDIATE else 1.0,
        )


@dataclass(frozen=True, slots=True)
class _Validated:
    witness: AutomatedWitness
    validation: GroundTruthWitnessValidation
    objective: WitnessObjective


@dataclass(frozen=True, slots=True)
class _PreparedPassSearch:
    targets: tuple[_Target, ...]
    geometry_axes: dict[
        tuple[str, PassSide],
        tuple[tuple[float, ...], tuple[float, ...]],
    ]
    eligible_speeds: dict[str, tuple[float, ...]]
    limitations: tuple[str, ...]
    timed_count: int


@dataclass(frozen=True, slots=True)
class _PassShardResult:
    first_ordinal: int
    candidate_count: int
    left_counts: PassCandidateCounts
    right_counts: PassCandidateCounts
    left_best: _Validated | None
    right_best: _Validated | None


class _DynamicGuardRejected(Exception):
    """Internal streaming signal for an exact-Actor stopping-guard failure."""


@dataclass(slots=True)
class _MutableCounts:
    generated: int = 0
    geometry: int = 0
    dynamic: int = 0
    validated: int = 0

    def freeze(self) -> PassCandidateCounts:
        return PassCandidateCounts(
            self.generated,
            self.geometry,
            self.dynamic,
            self.validated,
        )


_DYNAMIC_FAILURES = frozenset(
    {
        "actor_clearance_violation",
        "target_inactive_at_departure",
        "target_not_ahead_at_departure",
        "target_not_lane_overlapping_at_departure",
        "target_not_same_direction_at_departure",
        "ordered_overtake_robot_progress_missing",
        "ordered_overtake_missing",
        "pass_departure_missing",
        "pass_wrong_side",
        "post_pass_reversal",
        "sustained_rejoin_missing",
        "overtake_before_departure",
        "rejoin_before_overtake",
        "multi_actor_pass_out_of_scope",
    }
)

_GEOMETRY_FAILURES = frozenset(
    {
        "required_pass_actor_missing",
        "passing_policy_prohibited",
        "witness_must_start_at_zero",
        "witness_start_pose_mismatch",
        "witness_start_twist_mismatch",
        "witness_exceeds_world_duration",
        "witness_not_20hz",
        "linear_speed_exceeded",
        "angular_speed_exceeded",
        "reverse_without_stop",
        "linear_acceleration_exceeded",
        "angular_acceleration_exceeded",
        "kinematic_pose_mismatch",
        "static_clearance_violation",
        "forbidden_region_entry",
        "ambiguous_reference_projection",
        "pass_reference_segment_mismatch",
        "terminal_dwell_missing",
        "goal_position_not_reached",
        "goal_heading_not_reached",
        "terminal_rejoin_distance_exceeded",
        "terminal_rejoin_heading_exceeded",
    }
)


def search_pass_structured(
    world: WitnessWorldSnapshot,
    *,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
) -> PassStructuredSearchResult:
    """좌·우 PASS 증인을 독립적으로 보존하는 결정론적 공개 검색."""

    started_ns = perf_counter_ns()
    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(search_config, WitnessSearchConfig):
        raise TypeError("search_config must be a WitnessSearchConfig")

    invalid_reason = _input_reason(world, search_config)
    if invalid_reason is not None:
        return _empty_result(
            world,
            status=WitnessSearchStatus.INVALID_INPUT,
            reason=invalid_reason,
            started_ns=started_ns,
            search_config=search_config,
        )
    if world.maneuver_constraints.passing_policy is PassingPolicy.PROHIBITED:
        return _empty_result(
            world,
            status=WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE,
            reason="passing_policy_prohibited",
            started_ns=started_ns,
            search_config=search_config,
        )

    segments = _straight_segments(world)
    targets = _eligible_targets(world, segments, search_config)
    limitations = (
        ("passing_policy_unspecified",)
        if world.maneuver_constraints.passing_policy is PassingPolicy.UNSPECIFIED
        else ()
    )
    if not targets:
        return _empty_result(
            world,
            status=WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE,
            reason="no_eligible_same_direction_target",
            started_ns=started_ns,
            search_config=search_config,
            limitations=limitations,
        )

    geometry_axes: dict[tuple[str, PassSide], tuple[tuple[float, ...], tuple[float, ...]]] = {}
    geometry_count = 0
    for target in targets:
        departures = _departure_progresses(world, target, search_config)
        for side in search_config.pass_side_order:
            offsets = _lateral_offsets(world, target, side, search_config)
            geometry_axes[(target.actor.actor_binding_id, side)] = (departures, offsets)
            geometry_count += len(departures) * len(offsets)
    if geometry_count > search_config.max_geometry_candidates_per_episode:
        return _empty_result(
            world,
            status=WitnessSearchStatus.RESOURCE_LIMIT,
            reason="geometry_candidate_preflight_limit",
            started_ns=started_ns,
            search_config=search_config,
            limitations=limitations,
        )

    eligible_speeds = {
        target.actor.actor_binding_id: tuple(
            speed
            for speed in search_config.linear_targets_mps
            if speed > target.tangent_speed_mps + search_config.pass_speed_advantage_epsilon_mps
        )
        for target in targets
    }
    timed_count = sum(
        len(geometry_axes[(target.actor.actor_binding_id, side)][0])
        * len(geometry_axes[(target.actor.actor_binding_id, side)][1])
        * len(_release_ticks(world, target))
        * len(eligible_speeds[target.actor.actor_binding_id])
        * len(search_config.pass_angular_magnitudes_radps)
        * len(search_config.pass_side_wait_policies)
        for target in targets
        for side in search_config.pass_side_order
    )
    if timed_count > search_config.max_timed_candidates_per_episode:
        return _empty_result(
            world,
            status=WitnessSearchStatus.RESOURCE_LIMIT,
            reason="timed_candidate_preflight_limit",
            started_ns=started_ns,
            search_config=search_config,
            limitations=limitations,
        )

    # A timed candidate that survives synthesis still needs one strict 200 Hz
    # pass. The dedicated hashed budget keeps that work explicit rather than
    # silently evaluating an objective frontier and calling it exhaustive.
    if timed_count > search_config.max_pass_evaluated_candidates_per_episode:
        return _empty_result(
            world,
            status=WitnessSearchStatus.RESOURCE_LIMIT,
            reason="exhaustive_validation_work_limit",
            started_ns=started_ns,
            search_config=search_config,
            limitations=(*limitations, "exhaustive_validation_not_run"),
        )

    side_counts = {side: _MutableCounts() for side in search_config.pass_side_order}
    best: dict[PassSide, _Validated | None] = {side: None for side in search_config.pass_side_order}

    # This loop is reached only for a bounded exhaustive candidate space. Every
    # fully specified candidate receives exactly one terminal bucket.
    for target in targets:
        speeds = eligible_speeds[target.actor.actor_binding_id]
        release_ticks = _release_ticks(world, target)
        if not speeds:
            continue
        for side in search_config.pass_side_order:
            departures, offsets = geometry_axes[(target.actor.actor_binding_id, side)]
            if not departures or not offsets:
                continue
            for departure in departures:
                for offset in offsets:
                    for release_tick in release_ticks:
                        for speed in speeds:
                            for angular in search_config.pass_angular_magnitudes_radps:
                                for wait_policy in search_config.pass_side_wait_policies:
                                    spec = _CandidateSpec(
                                        target=target,
                                        side=side,
                                        departure_progress_m=departure,
                                        lateral_offset_m=offset,
                                        release_tick=release_tick,
                                        linear_target_mps=speed,
                                        angular_magnitude_radps=angular,
                                        wait_policy=wait_policy,
                                    )
                                    counts = side_counts[side]
                                    counts.generated += 1
                                    validated, rejection = _evaluate_candidate(
                                        world,
                                        spec,
                                        search_config,
                                    )
                                    if validated is None:
                                        if rejection == "dynamic":
                                            counts.dynamic += 1
                                        else:
                                            counts.geometry += 1
                                        continue
                                    counts.validated += 1
                                    best[side] = _better(best[side], validated)

    left = _side_result(PassSide.LEFT, side_counts[PassSide.LEFT], best[PassSide.LEFT])
    right = _side_result(
        PassSide.RIGHT,
        side_counts[PassSide.RIGHT],
        best[PassSide.RIGHT],
    )
    return PassStructuredSearchResult(
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        vehicle_profile_hash=world.vehicle_profile_hash,
        maneuver_policy_hash=world.maneuver_constraints.content_hash,
        maneuver_policy_revision=world.maneuver_constraints.policy_revision,
        search_config_hash=search_config.content_hash,
        search_config_version=WITNESS_SEARCH_CONFIG_VERSION,
        left=left,
        right=right,
        limitations=limitations,
        elapsed_nonqualification_ns=perf_counter_ns() - started_ns,
        pass_search_version=PASS_STRUCTURED_SEARCH_VERSION,
    )


def search_pass_structured_parallel(
    world: WitnessWorldSnapshot,
    *,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
    max_workers: int = 14,
    shard_size: int = 2_048,
) -> PassStructuredSearchResult:
    """Evaluate deterministic contiguous candidate shards in processes.

    Worker count, shard size and completion order are operational only. Every
    shard preserves the frozen serial candidate order, and the parent verifies
    exact ordinal coverage before reducing with the same total objective key.
    This function is not a wall-clock qualification benchmark.
    """

    if type(max_workers) is not int or max_workers <= 0:
        raise ValueError("max_workers must be a positive exact integer")
    if type(shard_size) is not int or shard_size <= 0:
        raise ValueError("shard_size must be a positive exact integer")
    started_ns = perf_counter_ns()
    prepared = _prepare_parallel_pass_search(
        world,
        search_config,
        started_ns=started_ns,
    )
    if isinstance(prepared, PassStructuredSearchResult):
        return prepared

    iterator = iter(_iter_candidate_specs(world, prepared, search_config))
    pending = set()
    outcomes: list[_PassShardResult] = []
    next_ordinal = 0
    with ProcessPoolExecutor(
        max_workers=max_workers,
        mp_context=get_context("spawn"),
    ) as executor:
        while True:
            chunk = tuple(islice(iterator, shard_size))
            if not chunk:
                break
            pending.add(
                executor.submit(
                    _evaluate_pass_candidate_shard,
                    world,
                    search_config,
                    next_ordinal,
                    chunk,
                )
            )
            next_ordinal += len(chunk)
            if len(pending) >= max_workers * 2:
                completed, pending = wait(pending, return_when=FIRST_COMPLETED)
                outcomes.extend(future.result() for future in completed)
        if pending:
            completed, _ = wait(pending)
            outcomes.extend(future.result() for future in completed)

    counts = {side: _MutableCounts() for side in search_config.pass_side_order}
    best: dict[PassSide, _Validated | None] = {
        side: None for side in search_config.pass_side_order
    }
    expected_ordinal = 0
    for outcome in sorted(outcomes, key=lambda item: item.first_ordinal):
        if outcome.first_ordinal != expected_ordinal:
            raise RuntimeError("PASS shard coverage has a gap or overlap")
        expected_ordinal += outcome.candidate_count
        _merge_counts(counts[PassSide.LEFT], outcome.left_counts)
        _merge_counts(counts[PassSide.RIGHT], outcome.right_counts)
        if outcome.left_best is not None:
            best[PassSide.LEFT] = _better(best[PassSide.LEFT], outcome.left_best)
        if outcome.right_best is not None:
            best[PassSide.RIGHT] = _better(best[PassSide.RIGHT], outcome.right_best)
    if expected_ordinal != prepared.timed_count:
        raise RuntimeError("PASS shard coverage does not match preflight count")
    _verify_parallel_best(world, best)
    return _complete_result(
        world,
        search_config,
        limitations=prepared.limitations,
        counts=counts,
        best=best,
        started_ns=started_ns,
    )


def generate_pass_candidate(
    world: WitnessWorldSnapshot,
    request: PassCandidateRequest,
    *,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
) -> AutomatedWitness | None:
    """한 label-free PASS 파라미터를 합성하고 canonical strict 검증한다.

    이 API는 대표 positive와 기동 원인 분리용이다. ``None``은 이 파라미터가
    실패했다는 뜻이며 일반적인 PASS 부재 증거가 아니다.
    """

    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(search_config, WitnessSearchConfig):
        raise TypeError("search_config must be a WitnessSearchConfig")
    if not isinstance(request, PassCandidateRequest):
        raise TypeError("request must be a PassCandidateRequest")
    if _input_reason(world, search_config) is not None:
        return None
    if world.maneuver_constraints.passing_policy is PassingPolicy.PROHIBITED:
        return None
    targets = _eligible_targets(world, _straight_segments(world), search_config)
    target = next(
        (item for item in targets if item.actor.actor_binding_id == request.actor_binding_id),
        None,
    )
    if target is None:
        return None
    if request.departure_progress_m not in _departure_progresses(world, target, search_config):
        return None
    if request.lateral_offset_m not in _lateral_offsets(world, target, request.side, search_config):
        return None
    if request.release_tick not in _release_ticks(world, target):
        return None
    if (
        request.linear_target_mps
        <= target.tangent_speed_mps + search_config.pass_speed_advantage_epsilon_mps
    ):
        return None
    if request.linear_target_mps not in search_config.linear_targets_mps:
        return None
    if request.angular_magnitude_radps not in search_config.pass_angular_magnitudes_radps:
        return None
    if request.wait_policy not in search_config.pass_side_wait_policies:
        return None
    spec = _CandidateSpec(
        target,
        request.side,
        request.departure_progress_m,
        request.lateral_offset_m,
        request.release_tick,
        request.linear_target_mps,
        request.angular_magnitude_radps,
        request.wait_policy,
    )
    validated, _rejection = _evaluate_candidate(world, spec, search_config)
    return None if validated is None else validated.witness


def generate_frozen_frontier_pass_candidate(
    world: WitnessWorldSnapshot,
    *,
    actor_binding_id: str,
    side: PassSide,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
) -> AutomatedWitness | None:
    """Build one label-free diagnostic candidate from frozen search axes.

    This helper is deliberately not a proof of search completeness.  It chooses
    the first geometry-derived departure and lateral offset, release tick zero,
    and the largest eligible frozen linear and angular targets.  The resulting
    draft still has to pass both the measurement and strict validators through
    :func:`generate_pass_candidate`.
    """

    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(search_config, WitnessSearchConfig):
        raise TypeError("search_config must be a WitnessSearchConfig")
    if not actor_binding_id:
        raise ValueError("actor_binding_id must not be empty")
    if not isinstance(side, PassSide):
        raise TypeError("side must be a PassSide")
    if _input_reason(world, search_config) is not None:
        return None
    targets = _eligible_targets(world, _straight_segments(world), search_config)
    target = next(
        (item for item in targets if item.actor.actor_binding_id == actor_binding_id),
        None,
    )
    if target is None:
        return None
    departures = _departure_progresses(world, target, search_config)
    offsets = _lateral_offsets(world, target, side, search_config)
    eligible_speeds = tuple(
        speed
        for speed in search_config.linear_targets_mps
        if speed > target.tangent_speed_mps + search_config.pass_speed_advantage_epsilon_mps
    )
    if (
        not departures
        or not offsets
        or 0 not in _release_ticks(world, target)
        or not eligible_speeds
        or not search_config.pass_angular_magnitudes_radps
        or PassSideWaitPolicy.IMMEDIATE not in search_config.pass_side_wait_policies
    ):
        return None
    return generate_pass_candidate(
        world,
        PassCandidateRequest(
            actor_binding_id=actor_binding_id,
            side=side,
            departure_progress_m=departures[0],
            lateral_offset_m=offsets[0],
            release_tick=0,
            linear_target_mps=max(eligible_speeds),
            angular_magnitude_radps=max(search_config.pass_angular_magnitudes_radps),
            wait_policy=PassSideWaitPolicy.IMMEDIATE,
        ),
        search_config=search_config,
    )


def _evaluate_candidate(
    world: WitnessWorldSnapshot,
    spec: _CandidateSpec,
    config: WitnessSearchConfig,
) -> tuple[_Validated | None, str]:
    try:
        candidate = _build_candidate(world, spec, config)
    except _DynamicGuardRejected:
        return None, "dynamic"
    if candidate is None:
        return None, "geometry"
    canonical, strict = canonicalize_and_validate_ground_truth_pass(world, candidate)
    if not strict.passed:
        return None, _rejection_bucket(strict.failures)
    if canonical is None:
        raise RuntimeError("passed canonical PASS validation returned no witness")
    return _Validated(canonical, strict, _objective(canonical, strict, spec)), "validated"


def _prepare_parallel_pass_search(
    world: WitnessWorldSnapshot,
    search_config: WitnessSearchConfig,
    *,
    started_ns: int,
) -> _PreparedPassSearch | PassStructuredSearchResult:
    """Repeat the serial preflight without evaluating or materializing candidates."""

    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(search_config, WitnessSearchConfig):
        raise TypeError("search_config must be a WitnessSearchConfig")
    invalid_reason = _input_reason(world, search_config)
    if invalid_reason is not None:
        return _empty_result(
            world,
            status=WitnessSearchStatus.INVALID_INPUT,
            reason=invalid_reason,
            started_ns=started_ns,
            search_config=search_config,
        )
    if world.maneuver_constraints.passing_policy is PassingPolicy.PROHIBITED:
        return _empty_result(
            world,
            status=WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE,
            reason="passing_policy_prohibited",
            started_ns=started_ns,
            search_config=search_config,
        )
    targets = _eligible_targets(world, _straight_segments(world), search_config)
    limitations = (
        ("passing_policy_unspecified",)
        if world.maneuver_constraints.passing_policy is PassingPolicy.UNSPECIFIED
        else ()
    )
    if not targets:
        return _empty_result(
            world,
            status=WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE,
            reason="no_eligible_same_direction_target",
            started_ns=started_ns,
            search_config=search_config,
            limitations=limitations,
        )

    geometry_axes: dict[
        tuple[str, PassSide],
        tuple[tuple[float, ...], tuple[float, ...]],
    ] = {}
    geometry_count = 0
    for target in targets:
        departures = _departure_progresses(world, target, search_config)
        for side in search_config.pass_side_order:
            offsets = _lateral_offsets(world, target, side, search_config)
            geometry_axes[(target.actor.actor_binding_id, side)] = (
                departures,
                offsets,
            )
            geometry_count += len(departures) * len(offsets)
    if geometry_count > search_config.max_geometry_candidates_per_episode:
        return _empty_result(
            world,
            status=WitnessSearchStatus.RESOURCE_LIMIT,
            reason="geometry_candidate_preflight_limit",
            started_ns=started_ns,
            search_config=search_config,
            limitations=limitations,
        )
    eligible_speeds = {
        target.actor.actor_binding_id: tuple(
            speed
            for speed in search_config.linear_targets_mps
            if speed
            > target.tangent_speed_mps
            + search_config.pass_speed_advantage_epsilon_mps
        )
        for target in targets
    }
    timed_count = sum(
        len(geometry_axes[(target.actor.actor_binding_id, side)][0])
        * len(geometry_axes[(target.actor.actor_binding_id, side)][1])
        * len(_release_ticks(world, target))
        * len(eligible_speeds[target.actor.actor_binding_id])
        * len(search_config.pass_angular_magnitudes_radps)
        * len(search_config.pass_side_wait_policies)
        for target in targets
        for side in search_config.pass_side_order
    )
    if timed_count > search_config.max_timed_candidates_per_episode:
        return _empty_result(
            world,
            status=WitnessSearchStatus.RESOURCE_LIMIT,
            reason="timed_candidate_preflight_limit",
            started_ns=started_ns,
            search_config=search_config,
            limitations=limitations,
        )
    if timed_count > search_config.max_pass_evaluated_candidates_per_episode:
        return _empty_result(
            world,
            status=WitnessSearchStatus.RESOURCE_LIMIT,
            reason="exhaustive_validation_work_limit",
            started_ns=started_ns,
            search_config=search_config,
            limitations=(*limitations, "exhaustive_validation_not_run"),
        )
    return _PreparedPassSearch(
        targets=targets,
        geometry_axes=geometry_axes,
        eligible_speeds=eligible_speeds,
        limitations=limitations,
        timed_count=timed_count,
    )


def _iter_candidate_specs(
    world: WitnessWorldSnapshot,
    prepared: _PreparedPassSearch,
    search_config: WitnessSearchConfig,
):
    """Yield exactly the serial frozen order without retaining the full space."""

    for target in prepared.targets:
        speeds = prepared.eligible_speeds[target.actor.actor_binding_id]
        release_ticks = _release_ticks(world, target)
        if not speeds:
            continue
        for side in search_config.pass_side_order:
            departures, offsets = prepared.geometry_axes[
                (target.actor.actor_binding_id, side)
            ]
            for departure in departures:
                for offset in offsets:
                    for release_tick in release_ticks:
                        for speed in speeds:
                            for angular in search_config.pass_angular_magnitudes_radps:
                                for wait_policy in search_config.pass_side_wait_policies:
                                    yield _CandidateSpec(
                                        target=target,
                                        side=side,
                                        departure_progress_m=departure,
                                        lateral_offset_m=offset,
                                        release_tick=release_tick,
                                        linear_target_mps=speed,
                                        angular_magnitude_radps=angular,
                                        wait_policy=wait_policy,
                                    )


def _evaluate_pass_candidate_shard(
    world: WitnessWorldSnapshot,
    search_config: WitnessSearchConfig,
    first_ordinal: int,
    specs: tuple[_CandidateSpec, ...],
) -> _PassShardResult:
    """Worker entrypoint; no files, corpus labels or shared mutable state."""

    _target_is_safely_stoppable.cache_clear()
    counts = {
        PassSide.LEFT: _MutableCounts(),
        PassSide.RIGHT: _MutableCounts(),
    }
    best: dict[PassSide, _Validated | None] = {
        PassSide.LEFT: None,
        PassSide.RIGHT: None,
    }
    for spec in specs:
        current = counts[spec.side]
        current.generated += 1
        validated, rejection = _evaluate_candidate(world, spec, search_config)
        if validated is None:
            if rejection == "dynamic":
                current.dynamic += 1
            else:
                current.geometry += 1
            continue
        current.validated += 1
        best[spec.side] = _better(best[spec.side], validated)
    return _PassShardResult(
        first_ordinal=first_ordinal,
        candidate_count=len(specs),
        left_counts=counts[PassSide.LEFT].freeze(),
        right_counts=counts[PassSide.RIGHT].freeze(),
        left_best=best[PassSide.LEFT],
        right_best=best[PassSide.RIGHT],
    )


def _merge_counts(target: _MutableCounts, source: PassCandidateCounts) -> None:
    target.generated += source.generated_count
    target.geometry += source.geometry_pruned_count
    target.dynamic += source.dynamic_rejected_count
    target.validated += source.validated_count


def _verify_parallel_best(
    world: WitnessWorldSnapshot,
    best: dict[PassSide, _Validated | None],
) -> None:
    """Revalidate only merged winners in the parent process."""

    for candidate in best.values():
        if candidate is None:
            continue
        validation = validate_ground_truth_witness(
            world,
            candidate.witness,
            strict_declarations=True,
        )
        if not validation.passed:
            raise RuntimeError("merged PASS winner failed parent strict validation")
        if validation.content_hash != candidate.validation.content_hash:
            raise RuntimeError("merged PASS winner validation hash changed in parent")


def _complete_result(
    world: WitnessWorldSnapshot,
    search_config: WitnessSearchConfig,
    *,
    limitations: tuple[str, ...],
    counts: dict[PassSide, _MutableCounts],
    best: dict[PassSide, _Validated | None],
    started_ns: int,
) -> PassStructuredSearchResult:
    return PassStructuredSearchResult(
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        vehicle_profile_hash=world.vehicle_profile_hash,
        maneuver_policy_hash=world.maneuver_constraints.content_hash,
        maneuver_policy_revision=world.maneuver_constraints.policy_revision,
        search_config_hash=search_config.content_hash,
        search_config_version=WITNESS_SEARCH_CONFIG_VERSION,
        left=_side_result(PassSide.LEFT, counts[PassSide.LEFT], best[PassSide.LEFT]),
        right=_side_result(
            PassSide.RIGHT,
            counts[PassSide.RIGHT],
            best[PassSide.RIGHT],
        ),
        limitations=limitations,
        elapsed_nonqualification_ns=perf_counter_ns() - started_ns,
        pass_search_version=PASS_STRUCTURED_SEARCH_VERSION,
    )


def _input_reason(
    world: WitnessWorldSnapshot,
    config: WitnessSearchConfig,
) -> str | None:
    if world.search_config_hash != config.content_hash:
        return "search_config_hash_mismatch"
    if config.reverse_enabled:
        return "pass_reverse_must_remain_disabled"
    period = world.kinematic_contract.control_period_s
    if abs(world.duration_s / period - round(world.duration_s / period)) > 1e-12:
        return "world_duration_not_on_control_grid"
    twist = world.initial_state.twist
    if not isfinite(twist.linear) or not isfinite(twist.angular):
        return "initial_twist_not_finite"
    return None


def _straight_segments(world: WitnessWorldSnapshot) -> tuple[_ReferenceSegment, ...]:
    segments: list[_ReferenceSegment] = []
    cumulative = 0.0
    for index, (source, target) in enumerate(
        zip(world.reference_path, world.reference_path[1:], strict=False)
    ):
        dx = target.x - source.x
        dy = target.y - source.y
        length = hypot(dx, dy)
        if length <= 1e-12:
            continue
        segments.append(
            _ReferenceSegment(
                index=index,
                source=source,
                target=target,
                length_m=length,
                tangent_x=dx / length,
                tangent_y=dy / length,
                tangent_yaw=atan2(dy, dx),
                cumulative_start_m=cumulative,
            )
        )
        cumulative += length
    return tuple(segments)


def _project(
    pose: Pose2D,
    segments: tuple[_ReferenceSegment, ...],
) -> _Projection | None:
    candidates: list[_Projection] = []
    for segment in segments:
        along = (pose.x - segment.source.x) * segment.tangent_x + (
            pose.y - segment.source.y
        ) * segment.tangent_y
        clipped = min(segment.length_m, max(0.0, along))
        projected = segment.pose(clipped)
        dx = pose.x - projected.x
        dy = pose.y - projected.y
        candidates.append(
            _Projection(
                segment=segment,
                progress_on_segment_m=clipped,
                total_progress_m=segment.cumulative_start_m + clipped,
                signed_offset_m=-segment.tangent_y * dx + segment.tangent_x * dy,
                distance_m=hypot(dx, dy),
            )
        )
    if not candidates:
        return None
    candidates.sort(key=lambda item: (item.distance_m, item.segment.index))
    best = candidates[0]
    tied = tuple(item for item in candidates[1:] if abs(item.distance_m - best.distance_m) <= 1e-9)
    for item in tied:
        adjacent = abs(item.segment.index - best.segment.index) == 1
        tangent_equal = abs(_angle(item.segment.tangent_yaw - best.segment.tangent_yaw)) <= 1e-9
        if not adjacent or not tangent_equal:
            return None
    return best


def _eligible_targets(
    world: WitnessWorldSnapshot,
    segments: tuple[_ReferenceSegment, ...],
    config: WitnessSearchConfig,
) -> tuple[_Target, ...]:
    result: list[_Target] = []
    profile = world.kinematic_contract.vehicle_profile
    for actor in sorted(world.actors, key=lambda item: item.actor_binding_id):
        state = actor.state_at(actor.active_from_s)
        if state is None or state.velocity.magnitude <= config.pass_minimum_actor_speed_mps:
            continue
        projection = _project(
            Pose2D(state.position.x, state.position.y, 0.0),
            segments,
        )
        if projection is None:
            continue
        segment = projection.segment
        tangent_speed = state.velocity.x * segment.tangent_x + state.velocity.y * segment.tangent_y
        direction_cosine = tangent_speed / state.velocity.magnitude
        if (
            tangent_speed <= 0.0
            or direction_cosine < cos(config.pass_same_direction_heading_tolerance_rad) - 1e-12
        ):
            continue
        lane_limit = profile.collision_width_m / 2.0 + state.radius_m + profile.minimum_clearance_m
        if abs(projection.signed_offset_m) > lane_limit + 1e-12:
            continue
        result.append(
            _Target(
                actor=actor,
                segment=segment,
                initial_progress_m=projection.progress_on_segment_m,
                tangent_speed_mps=tangent_speed,
                signed_offset_m=projection.signed_offset_m,
            )
        )
    return tuple(result)


def _departure_progresses(
    world: WitnessWorldSnapshot,
    target: _Target,
    config: WitnessSearchConfig,
) -> tuple[float, ...]:
    profile = world.kinematic_contract.vehicle_profile
    initial = _project(world.initial_state.pose, (target.segment,))
    if initial is None:
        return ()
    upper = target.initial_progress_m - (profile.collision_length_m / 2.0 + target.actor.radius_m)
    step = max(world.grid.resolution_m, config.geometry_progress_step_m)
    if upper < initial.progress_on_segment_m - 1e-12:
        return ()
    count = floor((upper - initial.progress_on_segment_m + 1e-12) / step) + 1
    return tuple(round(initial.progress_on_segment_m + index * step, 12) for index in range(count))


def _lateral_offsets(
    world: WitnessWorldSnapshot,
    target: _Target,
    side: PassSide,
    config: WitnessSearchConfig,
) -> tuple[float, ...]:
    profile = world.kinematic_contract.vehicle_profile
    sign = 1.0 if side is PassSide.LEFT else -1.0
    minimum_signed = target.signed_offset_m + sign * (
        target.actor.radius_m + profile.collision_width_m / 2.0 + profile.minimum_clearance_m
    )
    minimum_magnitude = sign * minimum_signed
    if minimum_magnitude <= 0.0:
        minimum_magnitude = world.grid.resolution_m / 2.0
    step = world.grid.resolution_m * config.pass_lateral_step_resolution_multiplier
    # Offset grid is tied to world cell centers. ``coordinate_factor`` maps the
    # selected world axis back to signed LEFT/RIGHT reference offset, including
    # vertical and reverse-directed segments.
    if abs(target.segment.tangent_y) > 1e-9 and abs(target.segment.tangent_x) > 1e-9:
        return ()  # v1 cell-center template is axis-aligned only
    if abs(target.segment.tangent_x) >= abs(target.segment.tangent_y):
        origin = world.grid.origin_y_m
        reference_coordinate = target.segment.source.y
        coordinate_factor = target.segment.tangent_x
        boundary_min = origin
        boundary_max = origin + world.grid.height * world.grid.resolution_m
    else:
        origin = world.grid.origin_x_m
        reference_coordinate = target.segment.source.x
        coordinate_factor = -target.segment.tangent_y
        boundary_min = origin
        boundary_max = origin + world.grid.width * world.grid.resolution_m
    desired_signed_offset = sign * minimum_magnitude
    desired_coordinate = reference_coordinate + desired_signed_offset / coordinate_factor
    coordinate_direction = sign * coordinate_factor
    cell_index = (
        ceil((desired_coordinate - origin) / step - 0.5)
        if coordinate_direction > 0.0
        else floor((desired_coordinate - origin) / step - 0.5)
    )
    first_coordinate = origin + (cell_index + 0.5) * step
    rotation_margin = (
        hypot(
            profile.collision_length_m / 2.0,
            profile.collision_width_m / 2.0,
        )
        + profile.minimum_clearance_m
    )
    values: list[float] = []
    coordinate = first_coordinate
    while (
        boundary_min + rotation_margin - 1e-12
        <= coordinate
        <= boundary_max - rotation_margin + 1e-12
    ):
        signed = coordinate_factor * (coordinate - reference_coordinate)
        magnitude = sign * signed
        if magnitude >= minimum_magnitude - 1e-12:
            values.append(round(magnitude, 12))
        coordinate += coordinate_direction * step
    return tuple(sorted(set(values)))


def _release_ticks(
    world: WitnessWorldSnapshot,
    target: _Target,
) -> tuple[int, ...]:
    """Return exact event anchors that can still produce an active-target pass.

    Actor event times remain continuous.  Only the command release anchor is
    rounded upward to the 20 Hz command grid.  ``active_until_s`` is inclusive,
    so a release at that same instant cannot create a later active departure and
    is removed before candidate counting.
    """

    period = world.kinematic_contract.control_period_s
    maximum = round(world.duration_s / period)
    ticks = {0}
    for actor in world.actors:
        ticks.add(ceil(actor.active_from_s / period - 1e-12))
        ticks.add(floor(actor.active_until_s / period + 1e-12) + 1)
        for entry_s, exit_s in _exact_lane_presence_intervals(world, actor):
            ticks.add(ceil(entry_s / period - 1e-12))
            ticks.add(ceil(exit_s / period - 1e-12))
    return tuple(
        sorted(
            tick
            for tick in ticks
            if 0 <= tick <= maximum and tick * period < target.actor.active_until_s - 1e-12
        )
    )


def _exact_lane_presence_intervals(
    world: WitnessWorldSnapshot,
    actor: WitnessActorTrajectory,
) -> tuple[tuple[float, float], ...]:
    """Find exact continuous-time intersections with the finite reference lane.

    The lane is the union of segment-aligned rectangles in which a wheelchair
    center could violate the frozen surface-clearance contract with the Actor.
    Segment ends are expanded by the longitudinal collision extent.  This is an
    event-anchor calculation only; static geometry and final safety remain the
    independent validator's responsibility.
    """

    profile = world.kinematic_contract.vehicle_profile
    longitudinal_margin = (
        profile.collision_length_m / 2.0 + actor.radius_m + profile.minimum_clearance_m
    )
    lateral_margin = profile.collision_width_m / 2.0 + actor.radius_m + profile.minimum_clearance_m
    duration = actor.active_until_s - actor.active_from_s
    intervals: list[tuple[float, float]] = []
    for segment in _straight_segments(world):
        relative_x = actor.start_position.x - segment.source.x
        relative_y = actor.start_position.y - segment.source.y
        along_start = relative_x * segment.tangent_x + relative_y * segment.tangent_y
        along_rate = actor.velocity.x * segment.tangent_x + actor.velocity.y * segment.tangent_y
        lateral_start = -segment.tangent_y * relative_x + segment.tangent_x * relative_y
        lateral_rate = -segment.tangent_y * actor.velocity.x + segment.tangent_x * actor.velocity.y
        along_interval = _linear_band_interval(
            along_start,
            along_rate,
            -longitudinal_margin,
            segment.length_m + longitudinal_margin,
            duration,
        )
        lateral_interval = _linear_band_interval(
            lateral_start,
            lateral_rate,
            -lateral_margin,
            lateral_margin,
            duration,
        )
        if along_interval is None or lateral_interval is None:
            continue
        entry = max(along_interval[0], lateral_interval[0])
        exit_ = min(along_interval[1], lateral_interval[1])
        if entry <= exit_ + 1e-12:
            intervals.append((actor.active_from_s + entry, actor.active_from_s + exit_))
    if not intervals:
        return ()
    intervals.sort()
    merged: list[list[float]] = []
    for entry, exit_ in intervals:
        if not merged or entry > merged[-1][1] + 1e-12:
            merged.append([entry, exit_])
        else:
            merged[-1][1] = max(merged[-1][1], exit_)
    return tuple((entry, exit_) for entry, exit_ in merged)


def _linear_band_interval(
    initial: float,
    rate: float,
    lower: float,
    upper: float,
    duration: float,
) -> tuple[float, float] | None:
    if abs(rate) <= 1e-15:
        return (0.0, duration) if lower - 1e-12 <= initial <= upper + 1e-12 else None
    first = (lower - initial) / rate
    second = (upper - initial) / rate
    entry = max(0.0, min(first, second))
    exit_ = min(duration, max(first, second))
    return (entry, exit_) if entry <= exit_ + 1e-12 else None


def _build_candidate(
    world: WitnessWorldSnapshot,
    spec: _CandidateSpec,
    config: WitnessSearchConfig,
) -> AutomatedWitness | None:
    maximum_tick = round(world.duration_s / world.kinematic_contract.control_period_s)
    points = [
        WitnessPoint(
            0.0,
            world.initial_state.pose,
            world.initial_state.twist,
            WitnessPhase.START,
            "initial_state",
        )
    ]
    segment = spec.target.segment
    side_sign = 1.0 if spec.side is PassSide.LEFT else -1.0
    departure_pose = segment.pose(spec.departure_progress_m)
    lane_pose = segment.pose(spec.departure_progress_m, side_sign * spec.lateral_offset_m)

    if not _move_to_pose(
        points,
        departure_pose,
        spec.linear_target_mps,
        spec.angular_magnitude_radps,
        WitnessPhase.FOLLOW_REFERENCE,
        "reference_prefix",
        world,
        config,
        maximum_tick,
    ):
        return None
    if not _brake(points, WitnessPhase.BRAKE_TO_STOP, "departure_stop", world, maximum_tick):
        return None
    while _tick(points[-1], world) < spec.release_tick:
        if not _append(
            points, Twist2D(), WitnessPhase.WAIT, "departure_release_wait", world, maximum_tick
        ):
            return None

    if not _move_to_pose(
        points,
        lane_pose,
        spec.linear_target_mps,
        spec.angular_magnitude_radps,
        WitnessPhase.MOVE_LATERAL,
        "move_lateral",
        world,
        config,
        maximum_tick,
    ):
        return None
    if not _brake(points, WitnessPhase.BRAKE_TO_STOP, "lateral_stop", world, maximum_tick):
        return None
    if not _turn(
        points,
        segment.tangent_yaw,
        spec.angular_magnitude_radps,
        WitnessPhase.PASS,
        "align_reference_tangent",
        world,
        config,
        maximum_tick,
    ):
        return None

    actor_end = spec.target.actor.state_at(spec.target.actor.active_until_s)
    if actor_end is None:
        return None
    actor_end_projection = _project(
        Pose2D(actor_end.position.x, actor_end.position.y, 0.0),
        (segment,),
    )
    if actor_end_projection is None:
        return None
    profile = world.kinematic_contract.vehicle_profile
    longitudinal_extent_m = profile.collision_length_m / 2.0 + spec.target.actor.radius_m
    pass_stop_progress = min(
        segment.length_m - profile.collision_length_m / 2.0 - profile.minimum_clearance_m,
        actor_end_projection.progress_on_segment_m
        + longitudinal_extent_m
        + max(world.grid.resolution_m, config.pass_synthesis_pose_tolerance_m),
    )
    if pass_stop_progress <= spec.departure_progress_m + 0.10:
        return None
    pass_pose = segment.pose(pass_stop_progress, side_sign * spec.lateral_offset_m)
    if not _move_to_pose(
        points,
        pass_pose,
        spec.linear_target_mps,
        spec.angular_magnitude_radps,
        WitnessPhase.PASS,
        "move_past",
        world,
        config,
        maximum_tick,
    ):
        return None
    if not _brake(points, WitnessPhase.BRAKE_TO_STOP, "suffix_safe_stop", world, maximum_tick):
        return None
    if spec.wait_policy is PassSideWaitPolicy.UNTIL_TARGET_INACTIVE:
        inactive_tick = (
            floor(
                spec.target.actor.active_until_s / world.kinematic_contract.control_period_s + 1e-12
            )
            + 1
        )
        while _tick(points[-1], world) < inactive_tick:
            if not _append(points, Twist2D(), WitnessPhase.WAIT, "side_wait", world, maximum_tick):
                return None

    rejoin_pose = segment.pose(pass_stop_progress)
    if not _move_to_pose(
        points,
        rejoin_pose,
        spec.linear_target_mps,
        spec.angular_magnitude_radps,
        WitnessPhase.REJOIN,
        "move_to_reference",
        world,
        config,
        maximum_tick,
    ):
        return None
    if not _brake(points, WitnessPhase.BRAKE_TO_STOP, "rejoin_stop", world, maximum_tick):
        return None
    if not _turn(
        points,
        segment.tangent_yaw,
        spec.angular_magnitude_radps,
        WitnessPhase.REJOIN,
        "align_reference",
        world,
        config,
        maximum_tick,
    ):
        return None
    for _ in range(round(0.50 / world.kinematic_contract.control_period_s)):
        if not _append(
            points, Twist2D(), WitnessPhase.TERMINAL_DWELL, "rejoin_dwell", world, maximum_tick
        ):
            return None

    identifier = canonical_content_hash(
        {
            "pass_search_version": PASS_STRUCTURED_SEARCH_VERSION,
            "source_projection_hash": world.source_projection_hash,
            "world_content_hash": world.content_hash,
            "search_config_hash": config.content_hash,
            "parameters": spec.frozen_parameter_tuple,
            "required_actor": spec.target.actor.actor_binding_id,
        }
    )
    return build_automated_witness(
        world,
        witness_id=f"pass-witness-{identifier[:24]}",
        kind=(WitnessKind.PASS_LEFT if spec.side is PassSide.LEFT else WitnessKind.PASS_RIGHT),
        terminal_mode=WitnessTerminalMode.REJOIN_DWELL,
        points=tuple(points),
        required_pass_actor_ids=(spec.target.actor.actor_binding_id,),
        terminal_dwell_s=0.50,
    )


def _move_to_pose(
    points: list[WitnessPoint],
    target: Pose2D,
    linear_target: float,
    angular_magnitude: float,
    phase: WitnessPhase,
    primitive: str,
    world: WitnessWorldSnapshot,
    config: WitnessSearchConfig,
    maximum_tick: int,
) -> bool:
    if not _brake(points, WitnessPhase.BRAKE_TO_STOP, f"{primitive}_pre_stop", world, maximum_tick):
        return False
    distance = hypot(target.x - points[-1].pose.x, target.y - points[-1].pose.y)
    if distance <= config.pass_synthesis_pose_tolerance_m:
        return True
    yaw = atan2(target.y - points[-1].pose.y, target.x - points[-1].pose.x)
    if not _turn(
        points, yaw, angular_magnitude, phase, f"{primitive}_turn", world, config, maximum_tick
    ):
        return False
    unit_x, unit_y = cos(yaw), sin(yaw)
    for _ in range(maximum_tick + 2):
        current = points[-1]
        dx, dy = target.x - current.pose.x, target.y - current.pose.y
        remaining = dx * unit_x + dy * unit_y
        cross = abs(-unit_y * dx + unit_x * dy)
        if _stopped(current.twist) and hypot(dx, dy) <= config.pass_synthesis_pose_tolerance_m:
            return True
        if (
            remaining < -config.pass_synthesis_pose_tolerance_m
            or cross > config.pass_synthesis_pose_tolerance_m
        ):
            return False
        stop_distance = _linear_stop_distance(current.twist.linear, world)
        command = 0.0 if remaining <= stop_distance + 0.005 else linear_target
        if not _append(points, Twist2D(command, 0.0), phase, primitive, world, maximum_tick):
            return False
    return False


def _brake(
    points: list[WitnessPoint],
    phase: WitnessPhase,
    primitive: str,
    world: WitnessWorldSnapshot,
    maximum_tick: int,
) -> bool:
    while not _stopped(points[-1].twist):
        if not _append(points, Twist2D(), phase, primitive, world, maximum_tick):
            return False
    return True


def _turn(
    points: list[WitnessPoint],
    target_yaw: float,
    angular_magnitude: float,
    phase: WitnessPhase,
    primitive: str,
    world: WitnessWorldSnapshot,
    config: WitnessSearchConfig,
    maximum_tick: int,
) -> bool:
    if abs(points[-1].twist.linear) > 1e-12:
        return False
    period = world.kinematic_contract.control_period_s
    acceleration = world.kinematic_contract.maximum_angular_acceleration_radps2
    for _ in range(maximum_tick + 2):
        current = points[-1]
        error = _angle(target_yaw - current.pose.yaw)
        if (
            abs(error) <= config.pass_synthesis_heading_tolerance_rad
            and abs(current.twist.angular) <= 1e-12
        ):
            return True
        direction = 1.0 if error >= 0.0 else -1.0
        stop_angle = _angular_stop_angle(current.twist.angular, acceleration, period)
        target = (
            0.0
            if current.twist.angular * direction < -1e-12 or abs(error) <= abs(stop_angle) + 0.002
            else direction * angular_magnitude
        )
        if not _append(points, Twist2D(0.0, target), phase, primitive, world, maximum_tick):
            return False
    return False


def _append(
    points: list[WitnessPoint],
    target: Twist2D,
    phase: WitnessPhase,
    primitive: str,
    world: WitnessWorldSnapshot,
    maximum_tick: int,
) -> bool:
    current = points[-1]
    if _tick(current, world) >= maximum_tick:
        return False
    if (not _stopped(current.twist) or not _stopped(target)) and not _target_is_safely_stoppable(
        current,
        target,
        world,
    ):
        raise _DynamicGuardRejected
    period = world.kinematic_contract.control_period_s
    profile = world.kinematic_contract.vehicle_profile
    next_pose = Pose2D(
        current.pose.x + current.twist.linear * cos(current.pose.yaw) * period,
        current.pose.y + current.twist.linear * sin(current.pose.yaw) * period,
        _angle(current.pose.yaw + current.twist.angular * period),
    )
    linear_target = target.linear
    if current.twist.linear * linear_target < -1e-12:
        linear_target = 0.0
    increasing = abs(linear_target) > abs(current.twist.linear) + 1e-12
    linear_rate = profile.max_acceleration_mps2 if increasing else profile.max_deceleration_mps2
    next_twist = Twist2D(
        _slew(current.twist.linear, linear_target, linear_rate * period),
        _slew(
            current.twist.angular,
            target.angular,
            world.kinematic_contract.maximum_angular_acceleration_radps2 * period,
        ),
    )
    points.append(
        WitnessPoint(
            round(current.time_s + period, 12),
            next_pose,
            next_twist,
            phase,
            primitive,
        )
    )
    return True


@lru_cache(maxsize=131_072)
def _target_is_safely_stoppable(
    point: WitnessPoint,
    target: Twist2D,
    world: WitnessWorldSnapshot,
) -> bool:
    """Conservatively test a limited stop against exact Actor ground truth.

    The proposed command is applied for at most the next control interval; all
    later targets are zero. Actor circles use exact ground-truth states, not an
    observation tube. The stopping rollout uses the frozen 5 ms grid and inserts
    raw Actor activation/deactivation instants. Static and policy geometry remain
    the independent final validator's responsibility.
    """

    if not world.actors:
        return True
    profile = world.kinematic_contract.vehicle_profile
    period = world.kinematic_contract.control_period_s
    evaluator_period = world.kinematic_contract.evaluator_period_s
    half_diagonal = hypot(
        profile.collision_length_m / 2.0,
        profile.collision_width_m / 2.0,
    )
    pose = point.pose
    twist = point.twist
    time_s = point.time_s
    next_target = target
    linear_stop_ticks = ceil(abs(twist.linear) / (profile.max_deceleration_mps2 * period))
    angular_stop_ticks = ceil(
        abs(twist.angular) / (world.kinematic_contract.maximum_angular_acceleration_radps2 * period)
    )
    subdivisions = round(period / evaluator_period)
    for _ in range(max(linear_stop_ticks, angular_stop_ticks) + 3):
        if time_s > world.duration_s + 1e-12:
            return False
        if _stopped(twist) and _stopped(next_target):
            if not _stopping_segment_is_safe(
                pose=pose,
                twist=twist,
                start_time_s=time_s,
                end_time_s=time_s,
                subdivisions=subdivisions,
                evaluator_period_s=evaluator_period,
                half_diagonal_m=half_diagonal,
                world=world,
            ):
                return False
            break
        right_time_s = round(time_s + period, 12)
        if not _stopping_segment_is_safe(
            pose=pose,
            twist=twist,
            start_time_s=time_s,
            end_time_s=right_time_s,
            subdivisions=subdivisions,
            evaluator_period_s=evaluator_period,
            half_diagonal_m=half_diagonal,
            world=world,
        ):
            return False
        pose = Pose2D(
            pose.x + twist.linear * cos(pose.yaw) * period,
            pose.y + twist.linear * sin(pose.yaw) * period,
            _angle(pose.yaw + twist.angular * period),
        )
        time_s = right_time_s
        linear_target = next_target.linear
        if twist.linear * linear_target < -1e-12:
            linear_target = 0.0
        increasing = abs(linear_target) > abs(twist.linear) + 1e-12
        linear_rate = profile.max_acceleration_mps2 if increasing else profile.max_deceleration_mps2
        twist = Twist2D(
            _slew(twist.linear, linear_target, linear_rate * period),
            _slew(
                twist.angular,
                next_target.angular,
                world.kinematic_contract.maximum_angular_acceleration_radps2 * period,
            ),
        )
        next_target = Twist2D()
    else:
        raise RuntimeError("derived PASS terminal stopping bound was insufficient")
    return True


def _stopping_segment_is_safe(
    *,
    pose: Pose2D,
    twist: Twist2D,
    start_time_s: float,
    end_time_s: float,
    subdivisions: int,
    evaluator_period_s: float,
    half_diagonal_m: float,
    world: WitnessWorldSnapshot,
) -> bool:
    """Use a conservative circle proof, then fall back to the exact 5 ms sweep."""

    profile = world.kinematic_contract.vehicle_profile
    robot_velocity_x = twist.linear * cos(pose.yaw)
    robot_velocity_y = twist.linear * sin(pose.yaw)
    fast_safe = True
    for actor in world.actors:
        active_start_s = max(start_time_s, actor.active_from_s)
        active_end_s = min(end_time_s, actor.active_until_s)
        if active_end_s < active_start_s - 1e-15:
            continue
        actor_state = actor.state_at(active_start_s)
        if actor_state is None:
            fast_safe = False
            break
        offset_s = active_start_s - start_time_s
        robot_x = pose.x + robot_velocity_x * offset_s
        robot_y = pose.y + robot_velocity_y * offset_s
        relative_x = actor_state.position.x - robot_x
        relative_y = actor_state.position.y - robot_y
        relative_velocity_x = actor_state.velocity.x - robot_velocity_x
        relative_velocity_y = actor_state.velocity.y - robot_velocity_y
        duration_s = max(0.0, active_end_s - active_start_s)
        relative_speed_squared = (
            relative_velocity_x * relative_velocity_x + relative_velocity_y * relative_velocity_y
        )
        closest_offset_s = 0.0
        if relative_speed_squared > 1e-24:
            closest_offset_s = min(
                duration_s,
                max(
                    0.0,
                    -(relative_x * relative_velocity_x + relative_y * relative_velocity_y)
                    / relative_speed_squared,
                ),
            )
        closest_x = relative_x + relative_velocity_x * closest_offset_s
        closest_y = relative_y + relative_velocity_y * closest_offset_s
        required_center_distance_m = half_diagonal_m + actor.radius_m + profile.minimum_clearance_m
        if hypot(closest_x, closest_y) < required_center_distance_m + 1e-12:
            fast_safe = False
            break
    if fast_safe:
        return True

    evaluation_times = {
        start_time_s + subdivision * evaluator_period_s
        for subdivision in range(subdivisions)
        if start_time_s + subdivision * evaluator_period_s <= end_time_s + 1e-15
    }
    evaluation_times.add(end_time_s)
    for actor in world.actors:
        for event_time_s in (actor.active_from_s, actor.active_until_s):
            if start_time_s <= event_time_s <= end_time_s:
                evaluation_times.add(event_time_s)
    ordered_times = tuple(sorted(evaluation_times))
    robot_speed_bound = abs(twist.linear) + abs(twist.angular) * half_diagonal_m
    for sample_index, sample_time_s in enumerate(ordered_times):
        next_time_s = (
            ordered_times[sample_index + 1]
            if sample_index + 1 < len(ordered_times)
            else sample_time_s
        )
        local_step_s = min(
            evaluator_period_s,
            max(0.0, next_time_s - sample_time_s),
        )
        offset_s = sample_time_s - start_time_s
        sample_pose = Pose2D(
            pose.x + robot_velocity_x * offset_s,
            pose.y + robot_velocity_y * offset_s,
            _angle(pose.yaw + twist.angular * offset_s),
        )
        for actor in world.actors:
            state = actor.state_at(min(sample_time_s, world.duration_s))
            if state is None:
                continue
            clearance_m = (
                oriented_footprint_circle_surface_distance(
                    sample_pose,
                    circle_center=(state.position.x, state.position.y),
                    circle_radius_m=state.radius_m,
                    profile=profile,
                )
                - (robot_speed_bound + state.velocity.magnitude) * local_step_s / 2.0
            )
            if clearance_m < profile.minimum_clearance_m - 1e-9:
                return False
    return True


def _linear_stop_distance(speed: float, world: WitnessWorldSnapshot) -> float:
    period = world.kinematic_contract.control_period_s
    decrement = world.kinematic_contract.vehicle_profile.max_deceleration_mps2 * period
    remaining = abs(speed)
    distance = 0.0
    while remaining > 1e-12:
        distance += remaining * period
        remaining = max(0.0, remaining - decrement)
    return distance


def _angular_stop_angle(speed: float, acceleration: float, period: float) -> float:
    direction = 1.0 if speed >= 0.0 else -1.0
    remaining = abs(speed)
    angle = 0.0
    while remaining > 1e-12:
        angle += remaining * period
        remaining = max(0.0, remaining - acceleration * period)
    return direction * angle


def _slew(current: float, target: float, maximum_delta: float) -> float:
    delta = target - current
    if abs(delta) <= maximum_delta:
        return target
    return current + (maximum_delta if delta > 0.0 else -maximum_delta)


def _tick(point: WitnessPoint, world: WitnessWorldSnapshot) -> int:
    return round(point.time_s / world.kinematic_contract.control_period_s)


def _stopped(twist: Twist2D) -> bool:
    return abs(twist.linear) <= 1e-12 and abs(twist.angular) <= 1e-12


def _angle(value: float) -> float:
    return (value + pi) % (2.0 * pi) - pi


def _objective(
    witness: AutomatedWitness,
    validation: GroundTruthWitnessValidation,
    spec: _CandidateSpec,
) -> WitnessObjective:
    return WitnessObjective(
        hard_failure_count=0,
        terminal_completion_time_s=witness.points[-1].time_s,
        actual_path_length_m=validation.metrics.actual_path_length_m,
        maximum_reference_deviation_m=validation.metrics.maximum_reference_deviation_m,
        full_stop_count=validation.metrics.full_stop_count,
        absolute_angular_travel_rad=validation.metrics.absolute_angular_travel_rad,
        kind_rank=0,
        frozen_parameter_tuple=spec.frozen_parameter_tuple,
    )


def _better(current: _Validated | None, candidate: _Validated) -> _Validated:
    if current is None:
        return candidate
    candidate_key = (candidate.objective.sort_key, candidate.witness.semantic_content_hash)
    current_key = (current.objective.sort_key, current.witness.semantic_content_hash)
    return candidate if candidate_key < current_key else current


def _rejection_bucket(failures: tuple[str, ...]) -> str:
    codes = frozenset(failures)
    unknown = codes - _DYNAMIC_FAILURES - _GEOMETRY_FAILURES
    if unknown:
        raise RuntimeError(f"unmapped PASS validator failure code(s): {sorted(unknown)!r}")
    return "geometry" if codes & _GEOMETRY_FAILURES else "dynamic"


def _side_result(
    side: PassSide,
    counts: _MutableCounts,
    best: _Validated | None,
) -> PassSideSearchResult:
    if best is None:
        return PassSideSearchResult(
            side=side,
            status=WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE,
            reason="structured_pass_template_exhausted",
            counts=counts.freeze(),
            best_witness=None,
            objective=None,
            selected_validation_hash=None,
            selected_candidate_parameter_hash=None,
        )
    return PassSideSearchResult(
        side=side,
        status=WitnessSearchStatus.WITNESS_FOUND,
        reason=f"validated_pass_{side.value}",
        counts=counts.freeze(),
        best_witness=best.witness,
        objective=best.objective,
        selected_validation_hash=best.validation.content_hash,
        selected_candidate_parameter_hash=build_pass_candidate_parameter_hash(
            side=side,
            witness=best.witness,
            objective=best.objective,
        ),
    )


def _empty_result(
    world: WitnessWorldSnapshot,
    *,
    status: WitnessSearchStatus,
    reason: str,
    started_ns: int,
    search_config: WitnessSearchConfig,
    limitations: tuple[str, ...] = (),
) -> PassStructuredSearchResult:
    zero = PassCandidateCounts(0, 0, 0, 0)
    left = PassSideSearchResult(PassSide.LEFT, status, reason, zero, None, None, None, None)
    right = PassSideSearchResult(PassSide.RIGHT, status, reason, zero, None, None, None, None)
    return PassStructuredSearchResult(
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        vehicle_profile_hash=world.vehicle_profile_hash,
        maneuver_policy_hash=world.maneuver_constraints.content_hash,
        maneuver_policy_revision=world.maneuver_constraints.policy_revision,
        search_config_hash=search_config.content_hash,
        search_config_version=WITNESS_SEARCH_CONFIG_VERSION,
        left=left,
        right=right,
        limitations=limitations,
        elapsed_nonqualification_ns=perf_counter_ns() - started_ns,
    )


__all__ = [
    "PassCandidateRequest",
    "generate_frozen_frontier_pass_candidate",
    "generate_pass_candidate",
    "search_pass_structured",
    "search_pass_structured_parallel",
]
