"""R3 v1 결정론적 bounded pose-heading lattice search."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import StrEnum
from heapq import heappop, heappush
from math import atan2, cos, hypot, pi, sin
from time import perf_counter_ns

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.spatial_oracle_contracts import (
    SPATIAL_COMPARISON_TOLERANCE_M,
    SPATIAL_ORACLE_RESULT_SCHEMA_VERSION,
    SPATIAL_ORACLE_VERSION,
    BoundedSpatialOracleRequest,
    BoundedSpatialOracleResult,
    ManeuverSide,
    SpatialLatticeState,
    SpatialOracleStatus,
    SpatialPrimitive,
    SpatialPrimitiveKind,
)
from hospital_path_lab.spatial_oracle_validation import (
    SpatialGeometryEvaluator,
    spatial_pose_is_safe,
    spatial_primitive_is_safe,
    validate_spatial_oracle_path,
)

_FORWARD_OFFSETS = (
    (5, 0),
    (4, 4),
    (0, 5),
    (-4, 4),
    (-5, 0),
    (-4, -4),
    (0, -5),
    (4, -4),
)
_PRIMITIVE_ORDER = (
    SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION,
    SpatialPrimitiveKind.REVERSE_ONE_TRANSLATION,
    SpatialPrimitiveKind.ROTATE_LEFT_45,
    SpatialPrimitiveKind.ROTATE_RIGHT_45,
)
_LIMITATIONS = (
    "abstract_anchor_connector",
    "abstract_terminal_stop_only",
    "orthogonal_lattice_motion_only",
    "simulation_only",
)


class _LaneStatus(StrEnum):
    FOUND = "found"
    EXHAUSTED = "exhausted"
    RESOURCE_LIMIT = "resource_limit"


@dataclass(frozen=True, slots=True)
class _SearchRecord:
    cost: float
    reverse_distance_m: float
    rotation_count: int
    primitive_lexical_sequence: tuple[str, ...]

    @property
    def dominance_key(self) -> tuple[float, float, int, tuple[str, ...]]:
        return (
            self.cost,
            self.reverse_distance_m,
            self.rotation_count,
            self.primitive_lexical_sequence,
        )


@dataclass(frozen=True, slots=True)
class _LaneOutcome:
    status: _LaneStatus
    side: ManeuverSide
    path: tuple[Pose2D, ...] = ()
    primitives: tuple[SpatialPrimitive, ...] = ()
    record: _SearchRecord | None = None
    generated_edges: int = 0
    expanded_states: int = 0
    peak_open_states: int = 0
    termination_reason: str = ""


def search_bounded_spatial_oracle(
    request: BoundedSpatialOracleRequest,
) -> BoundedSpatialOracleResult:
    """동결된 R3 v1 lattice를 탐색하고 성공 path를 독립 검증한다."""

    started = perf_counter_ns()
    integrity_failure = request.integrity_failure()
    if integrity_failure is not None:
        return _empty_result(
            request,
            status=SpatialOracleStatus.INVALID_INPUT,
            reason=integrity_failure,
            exhaustive=False,
            elapsed_ns=perf_counter_ns() - started,
        )
    if not spatial_pose_is_safe(request, request.start_pose):
        return _empty_result(
            request,
            status=SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            reason="start_footprint_unsafe",
            exhaustive=False,
            elapsed_ns=perf_counter_ns() - started,
        )
    if not spatial_pose_is_safe(request, request.rejoin_goal.pose):
        return _empty_result(
            request,
            status=SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            reason="goal_footprint_unsafe",
            exhaustive=False,
            elapsed_ns=perf_counter_ns() - started,
        )

    sides = (
        (ManeuverSide.LEFT, ManeuverSide.RIGHT)
        if request.maneuver_side is ManeuverSide.UNSPECIFIED
        else (request.maneuver_side,)
    )
    outcomes = tuple(_search_lane(request, side) for side in sides)
    generated_edges = sum(outcome.generated_edges for outcome in outcomes)
    expanded_states = sum(outcome.expanded_states for outcome in outcomes)
    peak_open_states = max((outcome.peak_open_states for outcome in outcomes), default=0)
    found = tuple(outcome for outcome in outcomes if outcome.status is _LaneStatus.FOUND)
    if found:
        selected = min(found, key=_outcome_selection_key)
        limitations = _LIMITATIONS + (
            ("unselected_side_resource_limited",)
            if any(outcome.status is _LaneStatus.RESOURCE_LIMIT for outcome in outcomes)
            else ()
        )
        validation = validate_spatial_oracle_path(
            request, selected.path, selected.primitives
        )
        if not validation.passed:
            raise RuntimeError(
                "independent spatial validator rejected selected path: "
                + ",".join(validation.failure_codes)
            )
        return BoundedSpatialOracleResult(
            schema_version=SPATIAL_ORACLE_RESULT_SCHEMA_VERSION,
            oracle_version=SPATIAL_ORACLE_VERSION,
            status=SpatialOracleStatus.SPATIALLY_FEASIBLE,
            termination_reason=f"goal_reached_{selected.side.value}",
            request_content_hash=request.request_content_hash,
            map_id=request.map_id,
            map_revision=request.map_revision,
            mission_revision=request.mission_revision,
            grid_content_hash=request.grid_content_hash,
            vehicle_profile_hash=request.vehicle_profile_hash,
            search_region_hash=request.search_region.content_hash,
            lattice_config_hash=request.lattice_config.content_hash,
            path=selected.path,
            primitive_sequence=selected.primitives,
            path_length_m=validation.path_length_m,
            reverse_length_m=validation.reverse_length_m,
            rotation_count=validation.rotation_count,
            minimum_clearance_m=validation.minimum_clearance_m,
            minimum_physical_clearance_m=validation.minimum_physical_clearance_m,
            minimum_forbidden_clearance_m=validation.minimum_forbidden_clearance_m,
            minimum_allowed_boundary_clearance_m=(
                validation.minimum_allowed_boundary_clearance_m
            ),
            generated_edges=generated_edges,
            expanded_states=expanded_states,
            peak_open_states=peak_open_states,
            exhaustive=False,
            validation=validation,
            limitations=limitations,
            elapsed_nonqualification_ns=perf_counter_ns() - started,
        )

    resource = tuple(
        outcome for outcome in outcomes if outcome.status is _LaneStatus.RESOURCE_LIMIT
    )
    if resource:
        reason = "+".join(
            sorted({outcome.termination_reason for outcome in resource})
        )
        return _empty_result(
            request,
            status=SpatialOracleStatus.RESOURCE_LIMIT,
            reason=reason,
            exhaustive=False,
            elapsed_ns=perf_counter_ns() - started,
            generated_edges=generated_edges,
            expanded_states=expanded_states,
            peak_open_states=peak_open_states,
        )
    return _empty_result(
        request,
        status=SpatialOracleStatus.SPATIALLY_INFEASIBLE,
        reason="bounded_lattice_exhausted",
        exhaustive=True,
        elapsed_ns=perf_counter_ns() - started,
        generated_edges=generated_edges,
        expanded_states=expanded_states,
        peak_open_states=peak_open_states,
    )


def _search_lane(request: BoundedSpatialOracleRequest, side: ManeuverSide) -> _LaneOutcome:
    config = request.lattice_config
    geometry = SpatialGeometryEvaluator(request)
    region_cells = frozenset(request.search_region.cells)
    start_entries, start_attempts = _anchor_entries(
        request,
        request.start_pose,
        side,
        is_start=True,
        required_excursion_reached=False,
        evaluator=geometry,
    )
    goal_entries, goal_attempts = _anchor_entries(
        request,
        request.rejoin_goal.pose,
        side,
        is_start=False,
        required_excursion_reached=True,
        evaluator=geometry,
    )
    generated_edges = start_attempts + goal_attempts
    if generated_edges > config.max_generated_edges:
        return _resource_outcome(
            side,
            "max_generated_edges",
            config.max_generated_edges,
            0,
            0,
        )
    if not start_entries or not goal_entries:
        return _LaneOutcome(
            status=_LaneStatus.EXHAUSTED,
            side=side,
            generated_edges=generated_edges,
            termination_reason=(
                "start_anchor_unreachable" if not start_entries else "goal_anchor_unreachable"
            ),
        )
    if len(start_entries) > config.max_open_states:
        return _resource_outcome(
            side,
            "max_open_states",
            generated_edges,
            0,
            config.max_open_states,
        )

    records: dict[SpatialLatticeState, _SearchRecord] = {}
    parents: dict[SpatialLatticeState, tuple[SpatialLatticeState, SpatialPrimitive]] = {}
    heap: list[tuple[object, ...]] = []
    open_members: set[SpatialLatticeState] = set()
    closed: set[SpatialLatticeState] = set()
    for state in start_entries:
        state_pose = _state_pose(request, state)
        has_anchor = not _poses_close(request.start_pose, state_pose)
        initial = _SearchRecord(
            cost=_anchor_cost(request, request.start_pose, state_pose),
            reverse_distance_m=0.0,
            rotation_count=0,
            primitive_lexical_sequence=(
                (SpatialPrimitiveKind.ANCHOR_CONNECTOR.value,) if has_anchor else ()
            ),
        )
        records[state] = initial
        open_members.add(state)
        _push(heap, request, state, initial)
    expanded_states = 0
    peak_open_states = len(open_members)
    primitive_cache: dict[tuple[int, int, int, SpatialPrimitiveKind], bool] = {}
    best_goal: _LaneOutcome | None = None

    while heap:
        item = heappop(heap)
        state = item[-1]
        if not isinstance(state, SpatialLatticeState):  # pragma: no cover - internal invariant
            raise RuntimeError("invalid spatial heap state")
        record = records.get(state)
        stale = (
            record is None
            or abs(float(item[0]) - float(item[1]) - record.cost) > 1e-12
            or item[2] != record.reverse_distance_m
            or item[3] != record.rotation_count
            or item[4] != record.primitive_lexical_sequence
        )
        if stale or state in closed:
            continue
        open_members.discard(state)
        if (
            best_goal is not None
            and best_goal.record is not None
            and float(item[0]) > best_goal.record.cost + 1e-12
        ):
            return replace(
                best_goal,
                generated_edges=generated_edges,
                expanded_states=expanded_states,
                peak_open_states=peak_open_states,
            )
        if state in goal_entries:
            path, primitives = _reconstruct(
                request, state, parents, start_entries
            )
            goal_anchor = goal_entries[state]
            if goal_anchor is not None:
                if not spatial_primitive_is_safe(
                    request, goal_anchor, evaluator=geometry
                ):  # pragma: no cover - prevalidated by _anchor_entries
                    closed.add(state)
                    continue
                path += (goal_anchor.end_pose,)
                primitives += (goal_anchor,)
                record = replace(
                    record,
                    cost=record.cost
                    + _anchor_cost(request, goal_anchor.start_pose, goal_anchor.end_pose),
                    primitive_lexical_sequence=(
                        record.primitive_lexical_sequence
                        + (SpatialPrimitiveKind.ANCHOR_CONNECTOR.value,)
                    ),
                )
            candidate_goal = _LaneOutcome(
                status=_LaneStatus.FOUND,
                side=side,
                path=path,
                primitives=primitives,
                record=record,
                generated_edges=generated_edges,
                expanded_states=expanded_states,
                peak_open_states=peak_open_states,
                termination_reason="goal_reached",
            )
            if best_goal is None or _outcome_selection_key(
                candidate_goal
            ) < _outcome_selection_key(best_goal):
                best_goal = candidate_goal
            closed.add(state)
            continue
        if expanded_states >= config.max_expanded_states:
            return _resource_outcome(
                side,
                "max_expanded_states",
                generated_edges,
                expanded_states,
                peak_open_states,
            )
        closed.add(state)
        expanded_states += 1

        for kind in _PRIMITIVE_ORDER:
            if generated_edges >= config.max_generated_edges:
                return _resource_outcome(
                    side,
                    "max_generated_edges",
                    generated_edges,
                    expanded_states,
                    peak_open_states,
                )
            generated_edges += 1
            neighbor, primitive = _neighbor(
                request, state, kind, side, region_cells=region_cells
            )
            if neighbor is None or primitive is None or neighbor in closed:
                continue
            cache_key = (state.x_cell, state.y_cell, state.heading_index, kind)
            safe = primitive_cache.get(cache_key)
            if safe is None:
                safe = spatial_primitive_is_safe(
                    request, primitive, evaluator=geometry
                )
                primitive_cache[cache_key] = safe
            if not safe:
                continue
            candidate = _extend_record(request, record, primitive)
            current = records.get(neighbor)
            if current is not None and current.dominance_key <= candidate.dominance_key:
                continue
            is_new_open = neighbor not in open_members
            if is_new_open and len(open_members) >= config.max_open_states:
                return _resource_outcome(
                    side,
                    "max_open_states",
                    generated_edges,
                    expanded_states,
                    peak_open_states,
                )
            records[neighbor] = candidate
            parents[neighbor] = (state, primitive)
            open_members.add(neighbor)
            peak_open_states = max(peak_open_states, len(open_members))
            _push(heap, request, neighbor, candidate)

    if best_goal is not None:
        return replace(
            best_goal,
            generated_edges=generated_edges,
            expanded_states=expanded_states,
            peak_open_states=peak_open_states,
        )
    return _LaneOutcome(
        status=_LaneStatus.EXHAUSTED,
        side=side,
        generated_edges=generated_edges,
        expanded_states=expanded_states,
        peak_open_states=peak_open_states,
        termination_reason="bounded_lattice_exhausted",
    )


def _neighbor(
    request: BoundedSpatialOracleRequest,
    state: SpatialLatticeState,
    kind: SpatialPrimitiveKind,
    side: ManeuverSide,
    *,
    region_cells: frozenset[tuple[int, int]],
) -> tuple[SpatialLatticeState | None, SpatialPrimitive | None]:
    x_cell = state.x_cell
    y_cell = state.y_cell
    heading = state.heading_index
    if kind is SpatialPrimitiveKind.FORWARD_ONE_TRANSLATION:
        dx, dy = _FORWARD_OFFSETS[heading]
        next_heading = heading
    elif kind is SpatialPrimitiveKind.REVERSE_ONE_TRANSLATION:
        dx, dy = _FORWARD_OFFSETS[heading]
        dx, dy = -dx, -dy
        next_heading = heading
    elif kind is SpatialPrimitiveKind.ROTATE_LEFT_45:
        dx, dy = 0, 0
        next_heading = (heading + 1) % 8
    else:
        dx, dy = 0, 0
        next_heading = (heading - 1) % 8
    cell = (x_cell + dx, y_cell + dy)
    if cell not in region_cells or not request.static_grid.in_bounds(cell):
        return None, None
    base_state = SpatialLatticeState(
        cell[0],
        cell[1],
        next_heading,
        state.required_excursion_reached,
    )
    pose = _state_pose(request, base_state)
    signed = request.reference_segment.signed_offset(pose)
    if side is ManeuverSide.LEFT:
        if signed < -SPATIAL_COMPARISON_TOLERANCE_M:
            return None, None
        reached = signed + SPATIAL_COMPARISON_TOLERANCE_M >= (
            request.rejoin_goal.minimum_side_excursion_m
        )
    else:
        if signed > SPATIAL_COMPARISON_TOLERANCE_M:
            return None, None
        reached = -signed + SPATIAL_COMPARISON_TOLERANCE_M >= (
            request.rejoin_goal.minimum_side_excursion_m
        )
    neighbor = replace(
        base_state,
        required_excursion_reached=state.required_excursion_reached or reached,
    )
    end_pose = _state_pose(request, neighbor)
    primitive = SpatialPrimitive(
        kind=kind,
        start_pose=_state_pose(request, state),
        end_pose=end_pose,
        start_state=state,
        end_state=neighbor,
    )
    return neighbor, primitive


def _anchor_entries(
    request: BoundedSpatialOracleRequest,
    pose: Pose2D,
    side: ManeuverSide,
    *,
    is_start: bool,
    required_excursion_reached: bool,
    evaluator: SpatialGeometryEvaluator,
) -> tuple[dict[SpatialLatticeState, SpatialPrimitive | None], int]:
    containing = request.static_grid.world_to_cell(pose)
    entries: dict[SpatialLatticeState, SpatialPrimitive | None] = {}
    attempts = 0
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            cell = (containing[0] + dx, containing[1] + dy)
            if (
                not request.static_grid.in_bounds(cell)
                or cell not in request.search_region.cells
            ):
                continue
            center = request.static_grid.cell_to_pose(cell)
            signed = request.reference_segment.signed_offset(center)
            if side is ManeuverSide.LEFT and signed < -SPATIAL_COMPARISON_TOLERANCE_M:
                continue
            if side is ManeuverSide.RIGHT and signed > SPATIAL_COMPARISON_TOLERANCE_M:
                continue
            for heading_index in range(8):
                attempts += 1
                state = SpatialLatticeState(
                    cell[0],
                    cell[1],
                    heading_index,
                    required_excursion_reached,
                )
                state_pose = _state_pose(request, state)
                if is_start:
                    primitive = _anchor_primitive(
                        pose,
                        state_pose,
                        start_state=None,
                        end_state=state,
                    )
                else:
                    primitive = _anchor_primitive(
                        state_pose,
                        pose,
                        start_state=state,
                        end_state=None,
                    )
                if _poses_close(primitive.start_pose, primitive.end_pose):
                    entries[state] = None
                elif spatial_primitive_is_safe(
                    request, primitive, evaluator=evaluator
                ):
                    entries[state] = primitive
    return entries, attempts


def _state_pose(
    request: BoundedSpatialOracleRequest, state: SpatialLatticeState
) -> Pose2D:
    center = request.static_grid.cell_to_pose((state.x_cell, state.y_cell))
    return Pose2D(center.x, center.y, state.heading_index * pi / 4.0)


def _anchor_primitive(
    start_pose: Pose2D,
    end_pose: Pose2D,
    *,
    start_state: SpatialLatticeState | None,
    end_state: SpatialLatticeState | None,
) -> SpatialPrimitive:
    return SpatialPrimitive(
        kind=SpatialPrimitiveKind.ANCHOR_CONNECTOR,
        start_pose=start_pose,
        end_pose=end_pose,
        start_state=start_state,
        end_state=end_state,
    )


def _extend_record(
    request: BoundedSpatialOracleRequest,
    record: _SearchRecord,
    primitive: SpatialPrimitive,
) -> _SearchRecord:
    distance = hypot(
        primitive.end_pose.x - primitive.start_pose.x,
        primitive.end_pose.y - primitive.start_pose.y,
    )
    reverse = primitive.kind is SpatialPrimitiveKind.REVERSE_ONE_TRANSLATION
    rotation = primitive.kind in {
        SpatialPrimitiveKind.ROTATE_LEFT_45,
        SpatialPrimitiveKind.ROTATE_RIGHT_45,
    }
    if reverse:
        cost = distance * request.lattice_config.reverse_cost_multiplier
    elif rotation:
        radius = hypot(
            request.vehicle_profile.collision_length_m / 2.0,
            request.vehicle_profile.collision_width_m / 2.0,
        )
        cost = radius * (pi / 4.0)
    else:
        cost = distance
    return _SearchRecord(
        cost=record.cost + cost,
        reverse_distance_m=record.reverse_distance_m + (distance if reverse else 0.0),
        rotation_count=record.rotation_count + int(rotation),
        primitive_lexical_sequence=(
            record.primitive_lexical_sequence + (primitive.kind.value,)
        ),
    )


def _push(
    heap: list[tuple[object, ...]],
    request: BoundedSpatialOracleRequest,
    state: SpatialLatticeState,
    record: _SearchRecord,
) -> None:
    pose = _state_pose(request, state)
    goal = request.rejoin_goal.pose
    heuristic = hypot(goal.x - pose.x, goal.y - pose.y)
    heappush(
        heap,
        (
            record.cost + heuristic,
            heuristic,
            record.reverse_distance_m,
            record.rotation_count,
            record.primitive_lexical_sequence,
            state,
        ),
    )


def _reconstruct(
    request: BoundedSpatialOracleRequest,
    goal_state: SpatialLatticeState,
    parents: dict[SpatialLatticeState, tuple[SpatialLatticeState, SpatialPrimitive]],
    start_entries: dict[SpatialLatticeState, SpatialPrimitive | None],
) -> tuple[tuple[Pose2D, ...], tuple[SpatialPrimitive, ...]]:
    reverse_primitives: list[SpatialPrimitive] = []
    state = goal_state
    while state not in start_entries:
        parent, primitive = parents[state]
        reverse_primitives.append(primitive)
        state = parent
    primitives = tuple(reversed(reverse_primitives))
    path = (_state_pose(request, state),) + tuple(
        primitive.end_pose for primitive in primitives
    )
    start_anchor = start_entries[state]
    if start_anchor is not None:
        path = (start_anchor.start_pose,) + path
        primitives = (start_anchor,) + primitives
    return path, primitives


def _anchor_cost(
    request: BoundedSpatialOracleRequest, start: Pose2D, end: Pose2D
) -> float:
    distance = hypot(end.x - start.x, end.y - start.y)
    radius = hypot(
        request.vehicle_profile.collision_length_m / 2.0,
        request.vehicle_profile.collision_width_m / 2.0,
    )
    return distance + radius * abs(_shortest_angle(end.yaw - start.yaw))


def _outcome_selection_key(outcome: _LaneOutcome) -> tuple[object, ...]:
    if outcome.record is None:  # pragma: no cover - guarded by caller
        raise RuntimeError("found outcome must have a search record")
    return (*outcome.record.dominance_key, outcome.side.value)


def _resource_outcome(
    side: ManeuverSide,
    reason: str,
    generated_edges: int,
    expanded_states: int,
    peak_open_states: int,
) -> _LaneOutcome:
    return _LaneOutcome(
        status=_LaneStatus.RESOURCE_LIMIT,
        side=side,
        generated_edges=generated_edges,
        expanded_states=expanded_states,
        peak_open_states=peak_open_states,
        termination_reason=reason,
    )


def _empty_result(
    request: BoundedSpatialOracleRequest,
    *,
    status: SpatialOracleStatus,
    reason: str,
    exhaustive: bool,
    elapsed_ns: int,
    generated_edges: int = 0,
    expanded_states: int = 0,
    peak_open_states: int = 0,
) -> BoundedSpatialOracleResult:
    return BoundedSpatialOracleResult(
        schema_version=SPATIAL_ORACLE_RESULT_SCHEMA_VERSION,
        oracle_version=SPATIAL_ORACLE_VERSION,
        status=status,
        termination_reason=reason,
        request_content_hash=request.request_content_hash,
        map_id=request.map_id,
        map_revision=request.map_revision,
        mission_revision=request.mission_revision,
        grid_content_hash=request.grid_content_hash,
        vehicle_profile_hash=request.vehicle_profile_hash,
        search_region_hash=request.search_region.content_hash,
        lattice_config_hash=request.lattice_config.content_hash,
        path=(),
        primitive_sequence=(),
        path_length_m=None,
        reverse_length_m=None,
        rotation_count=None,
        minimum_clearance_m=None,
        minimum_physical_clearance_m=None,
        minimum_forbidden_clearance_m=None,
        minimum_allowed_boundary_clearance_m=None,
        generated_edges=generated_edges,
        expanded_states=expanded_states,
        peak_open_states=peak_open_states,
        exhaustive=exhaustive,
        validation=None,
        limitations=_LIMITATIONS,
        elapsed_nonqualification_ns=elapsed_ns,
    )


def _poses_close(first: Pose2D, second: Pose2D) -> bool:
    return (
        hypot(first.x - second.x, first.y - second.y) <= 1e-9
        and abs(_shortest_angle(first.yaw - second.yaw)) <= 1e-9
    )


def _shortest_angle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))
