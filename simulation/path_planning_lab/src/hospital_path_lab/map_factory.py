"""재현 가능한 graph/grid 지도와 사건 코퍼스를 만드는 연구용 팩터리.

이 모듈의 지도와 차체 값은 ``simulation_only``인 Python 논리 시험 입력이다.
G1~G5가 확인되지 않았으므로 제품 알고리즘 채택이나 실물 안전성 근거로 쓰지 않는다.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass, fields, is_dataclass, replace
from enum import Enum, StrEnum
from functools import lru_cache
from hashlib import sha256
from json import dumps
from math import ceil, hypot, isfinite
from random import Random
from typing import Any

import numpy as np

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import GraphSnapshot, GridSnapshot, Pose2D, SnapshotMetadata
from hospital_path_lab.graph import Edge, GraphMap, Node, canonical_edge
from hospital_path_lab.grid import GridMap
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile

SCHEMA_VERSION = "1.1"
GRID_RESOLUTION_M = 0.02
GENERATOR_VERSION = "map_factory_v2"
DEFAULT_BATCH_SIZE = 10


class WorldFamily(StrEnum):
    CORRIDOR = "corridor"
    INTERSECTION = "intersection"
    DEAD_END = "dead_end"
    U_TRAP = "u_trap"


class CorpusSplit(StrEnum):
    GOLDEN = "golden"
    DEVELOPMENT = "development"
    HIDDEN = "hidden"
    REGRESSIONS = "regressions"


class EventKind(StrEnum):
    CLOSE_EDGE = "close_edge"
    OPEN_EDGE = "open_edge"
    CREATE_OBSTACLE = "create_obstacle"
    MOVE_OBSTACLE = "move_obstacle"
    REMOVE_OBSTACLE = "remove_obstacle"
    MOVE_START = "move_start"
    CHANGE_GOAL = "change_goal"
    INVALIDATE = "invalidate"


class GoldenScenario(StrEnum):
    SINGLE_ROUTE = "single_route"
    ALTERNATE_ROUTE = "alternate_route"
    EQUAL_COST = "equal_cost"
    DEAD_END = "dead_end"
    ISOLATED_GOAL = "isolated_goal"
    WIDE_CORRIDOR = "wide_corridor"
    NARROW_DOOR = "narrow_door"
    PARTIAL_OCCUPANCY = "partial_occupancy"
    FULL_BLOCK = "full_block"
    U_TRAP = "u_trap"
    SEQUENTIAL_CLOSE_OPEN = "sequential_close_open"
    STALE_INVALID_JUDGMENT = "stale_invalid_judgment"


@dataclass(frozen=True, slots=True)
class WorldSpec:
    schema_version: str
    generator_version: str
    world_id: str
    family: WorldFamily
    seed: int
    simulation_only: bool
    vehicle_profile_id: str
    resolution_m: float
    width_m: float
    height_m: float
    corridor_width_m: float
    directed: bool
    nodes: tuple[Node, ...]
    edges: tuple[Edge, ...]
    forbidden_zones: tuple[ForbiddenZone, ...] = ()
    scenario_id: str | None = None

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class DynamicObstacle:
    obstacle_id: str
    pose: Pose2D
    radius_m: float


@dataclass(frozen=True, slots=True)
class ForbiddenZone:
    """점유 장애물과 구분되는 승인 불가 직사각형 영역."""

    zone_id: str
    min_x_m: float
    min_y_m: float
    max_x_m: float
    max_y_m: float


@dataclass(frozen=True, slots=True)
class Event:
    step: int
    kind: EventKind
    map_revision: int
    mission_revision: int
    observation_revision: int
    expected_path_exists: bool
    edge: tuple[str, str] | None = None
    node_id: str | None = None
    input_valid: bool = True
    obstacle_id: str | None = None
    obstacle_pose: Pose2D | None = None
    obstacle_radius_m: float | None = None
    expected_grid_path_exists: bool | None = None


@dataclass(frozen=True, slots=True)
class EpisodeSpec:
    schema_version: str
    generator_version: str
    episode_id: str
    world_id: str
    world_content_hash: str
    seed: int
    split: CorpusSplit
    simulation_only: bool
    start: str
    goal: str
    initial_map_revision: int
    initial_mission_revision: int
    initial_observation_revision: int
    initial_path_exists: bool
    events: tuple[Event, ...]
    scenario_id: str | None = None

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class EpisodeState:
    start: str
    goal: str
    closed_edges: frozenset[tuple[str, str]]
    obstacles: tuple[DynamicObstacle, ...]
    input_valid: bool
    map_revision: int
    mission_revision: int
    observation_revision: int


@dataclass(frozen=True, slots=True)
class GeneratedCase:
    world: WorldSpec
    episode: EpisodeSpec


@dataclass(frozen=True, slots=True)
class FrozenCaseReference:
    world_id: str
    world_content_hash: str
    episode_id: str
    episode_content_hash: str
    world_seed: int
    episode_seed: int


@dataclass(frozen=True, slots=True)
class FrozenCorpus:
    schema_version: str
    generator_version: str
    corpus_id: str
    simulation_only: bool
    cases: tuple[FrozenCaseReference, ...]

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


def canonical_content_hash(value: object) -> str:
    """필드·dict 순서와 무관한 SHA-256을 반환한다."""

    encoded = dumps(
        _canonical_value(value),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return sha256(encoded).hexdigest()


def generate_world(seed: int, family: WorldFamily | str | None = None) -> WorldSpec:
    """seed와 계열로부터 같은 ``WorldSpec``을 반복 생성한다."""

    families = tuple(WorldFamily)
    selected = WorldFamily(family) if family is not None else families[seed % len(families)]
    rng = Random(seed)
    coordinates, connections = _family_template(selected)
    nodes = tuple(
        Node(
            node_id=node_id,
            x=round(x + rng.uniform(-0.04, 0.04), 4),
            y=round(y + rng.uniform(-0.04, 0.04), 4),
        )
        for node_id, (x, y) in coordinates
    )
    by_id = {node.node_id: node for node in nodes}
    edges = tuple(
        Edge(
            source=source,
            target=target,
            cost=round(
                hypot(by_id[source].x - by_id[target].x, by_id[source].y - by_id[target].y),
                10,
            ),
        )
        for source, target in connections
    )
    world = WorldSpec(
        schema_version=SCHEMA_VERSION,
        generator_version=GENERATOR_VERSION,
        world_id=f"{selected.value}_{seed:010d}",
        family=selected,
        seed=seed,
        simulation_only=True,
        vehicle_profile_id=VIRTUAL_DOLL_WHEELCHAIR_V0_1.profile_id,
        resolution_m=GRID_RESOLUTION_M,
        width_m=5.0,
        height_m=4.0,
        corridor_width_m=round(0.80 + rng.choice((0.0, 0.04, 0.08)), 2),
        directed=False,
        nodes=nodes,
        edges=edges,
    )
    validate_world(world)
    return world


def generate_episode(
    world: WorldSpec,
    *,
    seed: int,
    split: CorpusSplit | str = CorpusSplit.DEVELOPMENT,
) -> EpisodeSpec:
    """폐쇄 반복·동적 장애물·미션 변경·무효화를 포함한 episode를 만든다."""

    validate_world(world)
    selected_edge = canonical_edge(
        world.edges[0].source,
        world.edges[0].target,
        directed=world.directed,
    )
    obstacle_pose, moved_pose = _generic_obstacle_poses(world, selected_edge)
    actions = (
        _Action(EventKind.CLOSE_EDGE, edge=selected_edge),
        _Action(EventKind.OPEN_EDGE, edge=selected_edge),
        _Action(
            EventKind.CREATE_OBSTACLE,
            obstacle_id="dynamic_0",
            obstacle_pose=obstacle_pose,
            obstacle_radius_m=0.10,
        ),
        _Action(
            EventKind.MOVE_OBSTACLE,
            obstacle_id="dynamic_0",
            obstacle_pose=moved_pose,
        ),
        _Action(EventKind.REMOVE_OBSTACLE, obstacle_id="dynamic_0"),
        _Action(EventKind.CLOSE_EDGE, edge=selected_edge),
        _Action(EventKind.OPEN_EDGE, edge=selected_edge),
        _Action(EventKind.MOVE_START, node_id=_move_target(world)),
        _Action(EventKind.CHANGE_GOAL, node_id=_alternate_goal(world)),
        _Action(EventKind.INVALIDATE, input_valid=False),
    )
    return _build_episode(
        world,
        seed=seed,
        split=CorpusSplit(split),
        start="start",
        goal="goal",
        actions=actions,
    )


def generate_batch(*, base_seed: int, size: int = DEFAULT_BATCH_SIZE) -> tuple[GeneratedCase, ...]:
    """기본 10개 단위 split 일정을 고정한 generated batch를 만든다."""

    if size < 1:
        raise ValueError("batch size는 1 이상이어야 합니다.")
    split_schedule = (
        CorpusSplit.GOLDEN,
        CorpusSplit.GOLDEN,
        CorpusSplit.DEVELOPMENT,
        CorpusSplit.DEVELOPMENT,
        CorpusSplit.DEVELOPMENT,
        CorpusSplit.DEVELOPMENT,
        CorpusSplit.HIDDEN,
        CorpusSplit.HIDDEN,
        CorpusSplit.REGRESSIONS,
        CorpusSplit.REGRESSIONS,
    )
    families = tuple(WorldFamily)
    result: list[GeneratedCase] = []
    for index in range(size):
        world_seed = base_seed + index * 7_919
        episode_seed = world_seed ^ 0x5F37_59DF
        world = generate_world(world_seed, families[index % len(families)])
        episode = generate_episode(
            world,
            seed=episode_seed,
            split=split_schedule[index % len(split_schedule)],
        )
        result.append(GeneratedCase(world=world, episode=episode))
    batch = tuple(result)
    validate_batch(batch, expected_size=size)
    return batch


@lru_cache(maxsize=1)
def generate_golden_cases() -> tuple[GeneratedCase, ...]:
    """사람이 이름과 의도를 고정한 필수 회귀 12개를 생성한다."""

    cases = tuple(
        _make_golden_case(scenario, index)
        for index, scenario in enumerate(GoldenScenario)
    )
    validate_golden_cases(cases)
    return cases


def freeze_batch(batch: tuple[GeneratedCase, ...], *, corpus_id: str) -> FrozenCorpus:
    """현재 순서와 해시를 불변 참조로 동결한다."""

    if not corpus_id.strip():
        raise ValueError("corpus_id는 비어 있을 수 없습니다.")
    validate_batch(batch, expected_size=len(batch))
    return FrozenCorpus(
        schema_version=SCHEMA_VERSION,
        generator_version=GENERATOR_VERSION,
        corpus_id=corpus_id,
        simulation_only=True,
        cases=tuple(
            FrozenCaseReference(
                world_id=item.world.world_id,
                world_content_hash=item.world.content_hash,
                episode_id=item.episode.episode_id,
                episode_content_hash=item.episode.content_hash,
                world_seed=item.world.seed,
                episode_seed=item.episode.seed,
            )
            for item in batch
        ),
    )


def validate_frozen_batch(batch: tuple[GeneratedCase, ...], frozen: FrozenCorpus) -> None:
    """동결 후 내용·순서가 달라진 generated corpus를 거부한다."""

    if frozen.schema_version != SCHEMA_VERSION or frozen.generator_version != GENERATOR_VERSION:
        raise ValueError("frozen corpus schema/generator version이 현재 생성기와 다릅니다.")
    if not frozen.simulation_only:
        raise ValueError("frozen corpus는 simulation_only여야 합니다.")
    validate_batch(batch, expected_size=len(frozen.cases))
    actual = freeze_batch(batch, corpus_id=frozen.corpus_id)
    if actual != frozen or actual.content_hash != frozen.content_hash:
        raise ValueError("frozen corpus content hash 또는 순서가 변경됐습니다.")


def episode_state_at(episode: EpisodeSpec, *, through_step: int = 0) -> EpisodeState:
    """지정 step까지 사건을 적용한 불변 상태를 반환한다."""

    state = _initial_state(episode)
    for event in episode.events:
        if event.step > through_step:
            break
        _apply_event(state, event, directed=False)
    return state.freeze()


def build_graph_snapshot(
    world: WorldSpec,
    episode: EpisodeSpec,
    *,
    through_step: int = 0,
) -> GraphSnapshot:
    validate_episode(world, episode)
    state = episode_state_at(episode, through_step=through_step)
    graph = GraphMap(
        list(world.nodes),
        list(world.edges),
        directed=world.directed,
        closed_edges=set(state.closed_edges),
    )
    return GraphSnapshot(metadata=_snapshot_metadata(world, episode, state), graph=graph)


def build_grid_snapshot(
    world: WorldSpec,
    episode: EpisodeSpec,
    *,
    through_step: int = 0,
) -> GridSnapshot:
    validate_episode(world, episode)
    state = episode_state_at(episode, through_step=through_step)
    grid = _grid_for(world, state.closed_edges, state.obstacles)
    return GridSnapshot(
        metadata=_snapshot_metadata(world, episode, state),
        grid=grid,
        forbidden_cells=_forbidden_cells(world, grid),
    )


def validate_world(
    world: WorldSpec,
    *,
    vehicle_profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
) -> None:
    errors: list[str] = []
    if world.schema_version != SCHEMA_VERSION:
        errors.append("지원하지 않는 schema_version")
    if world.generator_version != GENERATOR_VERSION:
        errors.append("지원하지 않는 generator_version")
    if not world.simulation_only or not vehicle_profile.simulation_only:
        errors.append("합성 지도와 차체 profile은 simulation_only여야 함")
    if world.vehicle_profile_id != vehicle_profile.profile_id:
        errors.append("vehicle_profile_id 불일치")
    if world.resolution_m != GRID_RESOLUTION_M:
        errors.append("grid resolution은 0.02m여야 함")
    if world.directed:
        errors.append("map_factory_v2는 무방향 합성 지도만 지원함")
    if not all(isfinite(value) and value > 0 for value in (world.width_m, world.height_m)):
        errors.append("지도 크기는 유한한 양수여야 함")
    required_width = vehicle_profile.collision_width_m + 2 * vehicle_profile.minimum_clearance_m
    if not isfinite(world.corridor_width_m) or world.corridor_width_m < required_width:
        errors.append("corridor 폭이 가상 차체 폭과 최소 여유보다 작음")

    node_ids = [node.node_id for node in world.nodes]
    if len(node_ids) != len(set(node_ids)) or not node_ids:
        errors.append("node ID는 비어 있지 않고 유일해야 함")
    for node in world.nodes:
        if not node.node_id or not all(isfinite(value) for value in (node.x, node.y)):
            errors.append("node ID/좌표가 유효하지 않음")
        elif not (0.0 < node.x < world.width_m and 0.0 < node.y < world.height_m):
            errors.append(f"node가 지도 경계 밖임: {node.node_id}")

    known_nodes = set(node_ids)
    edge_keys: set[tuple[str, str]] = set()
    for edge in world.edges:
        key = canonical_edge(edge.source, edge.target, directed=world.directed)
        if edge.source not in known_nodes or edge.target not in known_nodes:
            errors.append(f"edge가 존재하지 않는 node를 참조함: {key}")
        if edge.source == edge.target:
            errors.append(f"self edge는 허용하지 않음: {key}")
        if not isfinite(edge.cost) or edge.cost <= 0:
            errors.append(f"edge cost는 유한한 양수여야 함: {key}")
        if key in edge_keys:
            errors.append(f"canonical edge 중복: {key}")
        edge_keys.add(key)
    if not world.edges:
        errors.append("edge는 하나 이상이어야 함")
    zone_ids: set[str] = set()
    for zone in world.forbidden_zones:
        if not zone.zone_id or zone.zone_id in zone_ids:
            errors.append("forbidden zone ID는 비어 있지 않고 유일해야 함")
        zone_ids.add(zone.zone_id)
        values = (zone.min_x_m, zone.min_y_m, zone.max_x_m, zone.max_y_m)
        if not all(isfinite(value) for value in values):
            errors.append(f"forbidden zone 좌표가 유한하지 않음: {zone.zone_id}")
        elif not (
            0.0 <= zone.min_x_m < zone.max_x_m <= world.width_m
            and 0.0 <= zone.min_y_m < zone.max_y_m <= world.height_m
        ):
            errors.append(f"forbidden zone 경계가 잘못됨: {zone.zone_id}")
    if errors:
        raise ValueError("WorldSpec 검증 실패: " + "; ".join(errors))


def validate_episode(world: WorldSpec, episode: EpisodeSpec) -> None:
    validate_world(world)
    errors: list[str] = []
    if episode.schema_version != SCHEMA_VERSION or episode.generator_version != GENERATOR_VERSION:
        errors.append("episode schema/generator version 불일치")
    if episode.world_id != world.world_id or episode.world_content_hash != world.content_hash:
        errors.append("episode가 현재 world ID/hash를 참조하지 않음")
    if not episode.simulation_only:
        errors.append("episode는 simulation_only여야 함")
    node_ids = {node.node_id for node in world.nodes}
    if episode.start not in node_ids or episode.goal not in node_ids:
        errors.append("start/goal이 world node에 없음")
    initial_exists = _graph_path_exists(world, episode.start, episode.goal, frozenset())
    if initial_exists != episode.initial_path_exists:
        errors.append("initial_path_exists가 graph oracle과 다름")

    state = _initial_state(episode)
    known_edges = {
        canonical_edge(edge.source, edge.target, directed=world.directed) for edge in world.edges
    }
    previous_step = -1
    for event in episode.events:
        if event.step <= previous_step:
            errors.append("event step은 엄격히 증가해야 함")
        previous_step = event.step
        before = state.freeze()
        if event.kind in (EventKind.CLOSE_EDGE, EventKind.OPEN_EDGE) and (
            event.edge is None
            or canonical_edge(*event.edge, directed=world.directed) not in known_edges
        ):
            errors.append(f"event가 world에 없는 edge를 참조함: step={event.step}")
            continue
        try:
            _apply_event(state, event, directed=world.directed)
        except ValueError as exc:
            errors.append(str(exc))
            continue
        expected_revisions = tuple(
            before_value + delta
            for before_value, delta in zip(
                (
                    before.map_revision,
                    before.mission_revision,
                    before.observation_revision,
                ),
                _revision_delta(event.kind),
                strict=True,
            )
        )
        actual_revisions = (
            event.map_revision,
            event.mission_revision,
            event.observation_revision,
        )
        if actual_revisions != expected_revisions:
            errors.append(f"event revision 전이가 잘못됨: step={event.step}")
        if state.start not in node_ids or state.goal not in node_ids:
            errors.append(f"event start/goal 참조가 잘못됨: step={event.step}")
        graph_exists = _graph_path_exists(
            world, state.start, state.goal, frozenset(state.closed_edges)
        )
        grid_exists = _world_grid_path_exists(
            world,
            state.start,
            state.goal,
            frozenset(state.closed_edges),
            tuple(sorted(state.obstacles.values(), key=lambda obstacle: obstacle.obstacle_id)),
        )
        if graph_exists != event.expected_path_exists:
            errors.append(f"expected_path_exists가 graph oracle과 다름: step={event.step}")
        expected_grid = event.expected_grid_path_exists
        if expected_grid is None:
            expected_grid = graph_exists
        if grid_exists != expected_grid:
            errors.append(f"expected_grid_path_exists가 grid oracle과 다름: step={event.step}")

    if not errors:
        for step in (0, *(event.step for event in episode.events)):
            snapshot_state = episode_state_at(episode, through_step=step)
            grid = _grid_for(
                world,
                snapshot_state.closed_edges,
                snapshot_state.obstacles,
            )
            start_pose = _node_pose(world, snapshot_state.start)
            goal_pose = _node_pose(world, snapshot_state.goal)
            if grid.is_occupied(grid.world_to_cell(start_pose)):
                errors.append(f"start가 충돌 cell에 있음: step={step}")
            if grid.is_occupied(grid.world_to_cell(goal_pose)):
                errors.append(f"goal이 충돌 cell에 있음: step={step}")
            graph_exists = _graph_path_exists(
                world,
                snapshot_state.start,
                snapshot_state.goal,
                snapshot_state.closed_edges,
            )
            grid_exists = _world_grid_path_exists(
                world,
                snapshot_state.start,
                snapshot_state.goal,
                snapshot_state.closed_edges,
                snapshot_state.obstacles,
            )
            if not snapshot_state.obstacles and graph_exists != grid_exists:
                errors.append(f"graph/grid 자유공간 연결 토폴로지 불일치: step={step}")
    if errors:
        raise ValueError("EpisodeSpec 검증 실패: " + "; ".join(errors))


def validate_batch(batch: tuple[GeneratedCase, ...], *, expected_size: int = 10) -> None:
    if len(batch) != expected_size:
        raise ValueError(f"batch 크기는 {expected_size}여야 합니다.")
    world_hashes: set[str] = set()
    episode_hashes: set[str] = set()
    for item in batch:
        validate_episode(item.world, item.episode)
        if item.world.content_hash in world_hashes or item.episode.content_hash in episode_hashes:
            raise ValueError("batch에는 중복 world/episode hash가 없어야 합니다.")
        world_hashes.add(item.world.content_hash)
        episode_hashes.add(item.episode.content_hash)


def validate_golden_cases(cases: tuple[GeneratedCase, ...]) -> None:
    expected_ids = tuple(scenario.value for scenario in GoldenScenario)
    actual_ids = tuple(item.episode.scenario_id for item in cases)
    if actual_ids != expected_ids:
        raise ValueError("golden corpus는 명시된 12개 시나리오 순서를 정확히 포함해야 합니다.")
    validate_batch(cases, expected_size=len(GoldenScenario))
    if any(item.episode.split is not CorpusSplit.GOLDEN for item in cases):
        raise ValueError("수동 golden case의 split은 모두 golden이어야 합니다.")

    by_id = {item.episode.scenario_id: item for item in cases}
    equal_case = by_id[GoldenScenario.EQUAL_COST.value]
    upper = _path_cost(equal_case.world, ("start", "upper", "goal"))
    lower = _path_cost(equal_case.world, ("start", "lower", "goal"))
    if upper != lower:
        raise ValueError("equal_cost golden의 두 경로 비용은 정확히 같아야 합니다.")

    for scenario in (GoldenScenario.NARROW_DOOR, GoldenScenario.WIDE_CORRIDOR):
        case = by_id[scenario.value]
        snapshot = build_grid_snapshot_unchecked(case.world, case.episode)
        checker = CollisionChecker(snapshot.grid)
        if not _configuration_path_exists(case.world, case.episode, checker):
            raise ValueError(
                f"{scenario.value}가 footprint configuration grid에서 통과 불가합니다."
            )


@dataclass(slots=True)
class _MutableEpisodeState:
    start: str
    goal: str
    closed_edges: set[tuple[str, str]]
    obstacles: dict[str, DynamicObstacle]
    input_valid: bool
    map_revision: int
    mission_revision: int
    observation_revision: int

    def freeze(self) -> EpisodeState:
        return EpisodeState(
            start=self.start,
            goal=self.goal,
            closed_edges=frozenset(self.closed_edges),
            obstacles=tuple(sorted(self.obstacles.values(), key=lambda item: item.obstacle_id)),
            input_valid=self.input_valid,
            map_revision=self.map_revision,
            mission_revision=self.mission_revision,
            observation_revision=self.observation_revision,
        )


@dataclass(frozen=True, slots=True)
class _Action:
    kind: EventKind
    edge: tuple[str, str] | None = None
    node_id: str | None = None
    input_valid: bool = True
    obstacle_id: str | None = None
    obstacle_pose: Pose2D | None = None
    obstacle_radius_m: float | None = None


def _initial_state(episode: EpisodeSpec) -> _MutableEpisodeState:
    return _MutableEpisodeState(
        start=episode.start,
        goal=episode.goal,
        closed_edges=set(),
        obstacles={},
        input_valid=True,
        map_revision=episode.initial_map_revision,
        mission_revision=episode.initial_mission_revision,
        observation_revision=episode.initial_observation_revision,
    )


def _apply_event(state: _MutableEpisodeState, event: Event, *, directed: bool) -> None:
    if event.kind is EventKind.CLOSE_EDGE:
        if event.edge is None:
            raise ValueError(f"close_edge에 edge가 없음: step={event.step}")
        edge = canonical_edge(*event.edge, directed=directed)
        if edge in state.closed_edges:
            raise ValueError(f"이미 닫힌 edge를 다시 닫음: step={event.step}")
        state.closed_edges.add(edge)
    elif event.kind is EventKind.OPEN_EDGE:
        if event.edge is None:
            raise ValueError(f"open_edge에 edge가 없음: step={event.step}")
        edge = canonical_edge(*event.edge, directed=directed)
        if edge not in state.closed_edges:
            raise ValueError(f"열려 있는 edge를 다시 엶: step={event.step}")
        state.closed_edges.remove(edge)
    elif event.kind is EventKind.CREATE_OBSTACLE:
        if event.obstacle_id is None or event.obstacle_pose is None:
            raise ValueError(f"create_obstacle 필드가 불완전함: step={event.step}")
        if event.obstacle_radius_m is None or event.obstacle_radius_m <= 0:
            raise ValueError(f"create_obstacle radius가 유효하지 않음: step={event.step}")
        if event.obstacle_id in state.obstacles:
            raise ValueError(f"이미 존재하는 obstacle을 생성함: step={event.step}")
        state.obstacles[event.obstacle_id] = DynamicObstacle(
            event.obstacle_id,
            event.obstacle_pose,
            event.obstacle_radius_m,
        )
    elif event.kind is EventKind.MOVE_OBSTACLE:
        if event.obstacle_id is None or event.obstacle_pose is None:
            raise ValueError(f"move_obstacle 필드가 불완전함: step={event.step}")
        previous = state.obstacles.get(event.obstacle_id)
        if previous is None:
            raise ValueError(f"존재하지 않는 obstacle을 이동함: step={event.step}")
        radius = previous.radius_m if event.obstacle_radius_m is None else event.obstacle_radius_m
        if radius <= 0:
            raise ValueError(f"move_obstacle radius가 유효하지 않음: step={event.step}")
        state.obstacles[event.obstacle_id] = DynamicObstacle(
            event.obstacle_id,
            event.obstacle_pose,
            radius,
        )
    elif event.kind is EventKind.REMOVE_OBSTACLE:
        if event.obstacle_id is None or event.obstacle_id not in state.obstacles:
            raise ValueError(f"존재하지 않는 obstacle을 제거함: step={event.step}")
        del state.obstacles[event.obstacle_id]
    elif event.kind is EventKind.MOVE_START:
        if event.node_id is None:
            raise ValueError(f"move_start에 node_id가 없음: step={event.step}")
        state.start = event.node_id
    elif event.kind is EventKind.CHANGE_GOAL:
        if event.node_id is None:
            raise ValueError(f"change_goal에 node_id가 없음: step={event.step}")
        state.goal = event.node_id
    elif event.kind is EventKind.INVALIDATE:
        if event.input_valid:
            raise ValueError(f"invalidate 사건은 input_valid=False여야 함: step={event.step}")
        state.input_valid = False
    else:  # pragma: no cover
        raise ValueError(f"알 수 없는 event kind: {event.kind}")
    state.map_revision = event.map_revision
    state.mission_revision = event.mission_revision
    state.observation_revision = event.observation_revision


def _revision_delta(kind: EventKind) -> tuple[int, int, int]:
    if kind in (EventKind.CLOSE_EDGE, EventKind.OPEN_EDGE):
        return 1, 0, 0
    if kind in (EventKind.MOVE_START, EventKind.CHANGE_GOAL):
        return 0, 1, 0
    return 0, 0, 1


def _build_episode(
    world: WorldSpec,
    *,
    seed: int,
    split: CorpusSplit,
    start: str,
    goal: str,
    actions: Iterable[_Action],
    scenario_id: str | None = None,
) -> EpisodeSpec:
    state = _MutableEpisodeState(start, goal, set(), {}, True, 0, 0, 0)
    events: list[Event] = []
    for step, action in enumerate(actions, start=1):
        map_delta, mission_delta, observation_delta = _revision_delta(action.kind)
        event = Event(
            step=step,
            kind=action.kind,
            map_revision=state.map_revision + map_delta,
            mission_revision=state.mission_revision + mission_delta,
            observation_revision=state.observation_revision + observation_delta,
            expected_path_exists=False,
            edge=action.edge,
            node_id=action.node_id,
            input_valid=action.input_valid,
            obstacle_id=action.obstacle_id,
            obstacle_pose=action.obstacle_pose,
            obstacle_radius_m=action.obstacle_radius_m,
        )
        _apply_event(state, event, directed=world.directed)
        frozen = state.freeze()
        graph_exists = _graph_path_exists(world, frozen.start, frozen.goal, frozen.closed_edges)
        grid_exists = _world_grid_path_exists(
            world,
            frozen.start,
            frozen.goal,
            frozen.closed_edges,
            frozen.obstacles,
        )
        events.append(
            replace(
                event,
                expected_path_exists=graph_exists,
                expected_grid_path_exists=grid_exists,
            )
        )
    episode = EpisodeSpec(
        schema_version=SCHEMA_VERSION,
        generator_version=GENERATOR_VERSION,
        episode_id=(
            f"{world.world_id}_{scenario_id}_{seed:010d}"
            if scenario_id is not None
            else f"{world.world_id}_{split.value}_{seed:010d}"
        ),
        world_id=world.world_id,
        world_content_hash=world.content_hash,
        seed=seed,
        split=split,
        simulation_only=True,
        start=start,
        goal=goal,
        initial_map_revision=0,
        initial_mission_revision=0,
        initial_observation_revision=0,
        initial_path_exists=_graph_path_exists(world, start, goal, frozenset()),
        events=tuple(events),
        scenario_id=scenario_id,
    )
    validate_episode(world, episode)
    return episode


def _snapshot_metadata(
    world: WorldSpec,
    episode: EpisodeSpec,
    state: EpisodeState,
) -> SnapshotMetadata:
    state_hash = canonical_content_hash(
        {
            "world_hash": world.content_hash,
            "episode_hash": episode.content_hash,
            "start": state.start,
            "goal": state.goal,
            "closed_edges": tuple(sorted(state.closed_edges)),
            "obstacles": state.obstacles,
            "input_valid": state.input_valid,
            "map_revision": state.map_revision,
            "mission_revision": state.mission_revision,
            "observation_revision": state.observation_revision,
        }
    )
    return SnapshotMetadata(
        map_id=world.world_id,
        map_revision=state.map_revision,
        mission_revision=state.mission_revision,
        observation_revision=state.observation_revision,
        seed=episode.seed,
        content_hash=state_hash,
        input_valid=state.input_valid,
    )


@lru_cache(maxsize=512)
def _grid_for(
    world: WorldSpec,
    closed_edges: frozenset[tuple[str, str]],
    obstacles: tuple[DynamicObstacle, ...] = (),
) -> GridMap:
    width = int(ceil(world.width_m / world.resolution_m))
    height = int(ceil(world.height_m / world.resolution_m))
    occupancy = np.ones((height, width), dtype=np.bool_)
    nodes = {node.node_id: node for node in world.nodes}
    radius = world.corridor_width_m / 2.0
    for edge in world.edges:
        key = canonical_edge(edge.source, edge.target, directed=world.directed)
        if key in closed_edges:
            continue
        _carve_segment(
            occupancy,
            world.resolution_m,
            nodes[edge.source],
            nodes[edge.target],
            radius,
        )
    for node in world.nodes:
        _carve_disc(occupancy, world.resolution_m, node.x, node.y, radius, occupied=False)
    for obstacle in obstacles:
        _carve_disc(
            occupancy,
            world.resolution_m,
            obstacle.pose.x,
            obstacle.pose.y,
            obstacle.radius_m,
            occupied=True,
        )
    return GridMap(occupancy=occupancy, resolution_m=world.resolution_m)


def _carve_segment(
    occupancy: np.ndarray,
    resolution: float,
    source: Node,
    target: Node,
    radius: float,
) -> None:
    min_x = max(0, int((min(source.x, target.x) - radius) // resolution))
    max_x = min(occupancy.shape[1] - 1, int((max(source.x, target.x) + radius) // resolution))
    min_y = max(0, int((min(source.y, target.y) - radius) // resolution))
    max_y = min(occupancy.shape[0] - 1, int((max(source.y, target.y) + radius) // resolution))
    dx = target.x - source.x
    dy = target.y - source.y
    length_sq = dx * dx + dy * dy
    x_values = (np.arange(min_x, max_x + 1, dtype=np.float64) + 0.5) * resolution
    y_values = (np.arange(min_y, max_y + 1, dtype=np.float64) + 0.5) * resolution
    cx, cy = np.meshgrid(x_values, y_values)
    projection = ((cx - source.x) * dx + (cy - source.y) * dy) / length_sq
    projection = np.clip(projection, 0.0, 1.0)
    nearest_x = source.x + projection * dx
    nearest_y = source.y + projection * dy
    mask = np.hypot(cx - nearest_x, cy - nearest_y) <= radius
    window = occupancy[min_y : max_y + 1, min_x : max_x + 1]
    window[mask] = False


def _carve_disc(
    occupancy: np.ndarray,
    resolution: float,
    center_x: float,
    center_y: float,
    radius: float,
    *,
    occupied: bool,
) -> None:
    min_x = max(0, int((center_x - radius) // resolution))
    max_x = min(occupancy.shape[1] - 1, int((center_x + radius) // resolution))
    min_y = max(0, int((center_y - radius) // resolution))
    max_y = min(occupancy.shape[0] - 1, int((center_y + radius) // resolution))
    x_values = (np.arange(min_x, max_x + 1, dtype=np.float64) + 0.5) * resolution
    y_values = (np.arange(min_y, max_y + 1, dtype=np.float64) + 0.5) * resolution
    cx, cy = np.meshgrid(x_values, y_values)
    mask = np.hypot(cx - center_x, cy - center_y) <= radius
    window = occupancy[min_y : max_y + 1, min_x : max_x + 1]
    window[mask] = occupied


def _graph_path_exists(
    world: WorldSpec,
    start: str,
    goal: str,
    closed_edges: frozenset[tuple[str, str]],
) -> bool:
    known = {node.node_id for node in world.nodes}
    if start not in known or goal not in known:
        return False
    adjacency: dict[str, list[str]] = {node_id: [] for node_id in known}
    for edge in world.edges:
        key = canonical_edge(edge.source, edge.target, directed=world.directed)
        if key in closed_edges:
            continue
        adjacency[edge.source].append(edge.target)
        if not world.directed:
            adjacency[edge.target].append(edge.source)
    pending = deque([start])
    visited = {start}
    while pending:
        current = pending.popleft()
        if current == goal:
            return True
        for neighbor in adjacency[current]:
            if neighbor not in visited:
                visited.add(neighbor)
                pending.append(neighbor)
    return False


def _grid_path_exists(grid: GridMap, start: Pose2D, goal: Pose2D) -> bool:
    start_cell = grid.world_to_cell(start)
    goal_cell = grid.world_to_cell(goal)
    if grid.is_occupied(start_cell) or grid.is_occupied(goal_cell):
        return False
    width = grid.width
    height = grid.height
    occupancy = grid.occupancy
    pending = deque([start_cell])
    visited = np.zeros((height, width), dtype=np.bool_)
    visited[start_cell[1], start_cell[0]] = True
    offsets = (
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
    )
    while pending:
        x, y = pending.popleft()
        if (x, y) == goal_cell:
            return True
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            if visited[ny, nx] or occupancy[ny, nx]:
                continue
            if dx and dy and (occupancy[y, nx] or occupancy[ny, x]):
                continue
            visited[ny, nx] = True
            pending.append((nx, ny))
    return False


@lru_cache(maxsize=1_024)
def _world_grid_path_exists(
    world: WorldSpec,
    start: str,
    goal: str,
    closed_edges: frozenset[tuple[str, str]],
    obstacles: tuple[DynamicObstacle, ...] = (),
) -> bool:
    grid = _grid_for(world, closed_edges, obstacles)
    return _grid_path_exists(grid, _node_pose(world, start), _node_pose(world, goal))


def _configuration_path_exists(
    world: WorldSpec,
    episode: EpisodeSpec,
    checker: CollisionChecker,
) -> bool:
    return _grid_path_exists(
        checker.configuration_grid,
        _node_pose(world, episode.start),
        _node_pose(world, episode.goal),
    )


def build_grid_snapshot_unchecked(world: WorldSpec, episode: EpisodeSpec) -> GridSnapshot:
    """golden 자체 검증에서 재귀 없이 초기 snapshot을 만드는 내부용 helper."""

    state = _initial_state(episode).freeze()
    grid = _grid_for(world, frozenset(), ())
    return GridSnapshot(
        metadata=_snapshot_metadata(world, episode, state),
        grid=grid,
        forbidden_cells=_forbidden_cells(world, grid),
    )


def _forbidden_cells(world: WorldSpec, grid: GridMap) -> frozenset[tuple[int, int]]:
    cells: set[tuple[int, int]] = set()
    for zone in world.forbidden_zones:
        min_x = max(0, int(zone.min_x_m // grid.resolution_m))
        max_x = min(grid.width - 1, int((zone.max_x_m - 1e-12) // grid.resolution_m))
        min_y = max(0, int(zone.min_y_m // grid.resolution_m))
        max_y = min(grid.height - 1, int((zone.max_y_m - 1e-12) // grid.resolution_m))
        for y in range(min_y, max_y + 1):
            for x in range(min_x, max_x + 1):
                cells.add((x, y))
    return frozenset(cells)


def _node_pose(world: WorldSpec, node_id: str) -> Pose2D:
    node = next(node for node in world.nodes if node.node_id == node_id)
    return Pose2D(node.x, node.y)


def _move_target(world: WorldSpec) -> str:
    return {
        WorldFamily.CORRIDOR: "mid_left",
        WorldFamily.INTERSECTION: "upper_left",
        WorldFamily.DEAD_END: "route_left",
        WorldFamily.U_TRAP: "left_top",
    }[world.family]


def _alternate_goal(world: WorldSpec) -> str:
    return {
        WorldFamily.CORRIDOR: "bay",
        WorldFamily.INTERSECTION: "lower_right",
        WorldFamily.DEAD_END: "dead_end_tip",
        WorldFamily.U_TRAP: "inner_dead",
    }[world.family]


def _generic_obstacle_poses(
    world: WorldSpec,
    edge_key: tuple[str, str],
) -> tuple[Pose2D, Pose2D]:
    nodes = {node.node_id: node for node in world.nodes}
    source, target = (nodes[edge_key[0]], nodes[edge_key[1]])
    dx = target.x - source.x
    dy = target.y - source.y
    length = hypot(dx, dy)
    normal_x, normal_y = (-dy / length, dx / length)
    first = Pose2D((source.x + target.x) / 2, (source.y + target.y) / 2)
    second = Pose2D(
        source.x + 0.60 * dx + 0.05 * normal_x,
        source.y + 0.60 * dy + 0.05 * normal_y,
    )
    return first, second


def _family_template(
    family: WorldFamily,
) -> tuple[tuple[tuple[str, tuple[float, float]], ...], tuple[tuple[str, str], ...]]:
    if family is WorldFamily.CORRIDOR:
        return (
            (
                ("start", (0.60, 2.00)),
                ("mid_left", (1.80, 2.00)),
                ("mid_right", (3.20, 2.00)),
                ("goal", (4.40, 2.00)),
                ("bay", (1.80, 3.20)),
            ),
            (
                ("mid_left", "mid_right"),
                ("start", "mid_left"),
                ("mid_right", "goal"),
                ("mid_left", "bay"),
            ),
        )
    if family is WorldFamily.INTERSECTION:
        return _alternate_template()
    if family is WorldFamily.DEAD_END:
        return (
            (
                ("start", (0.55, 2.00)),
                ("junction", (1.35, 2.00)),
                ("route_left", (2.15, 2.80)),
                ("route_right", (3.25, 2.80)),
                ("goal", (4.45, 2.00)),
                ("dead_end_left", (2.15, 1.20)),
                ("dead_end_tip", (3.45, 1.20)),
            ),
            (
                ("route_left", "route_right"),
                ("start", "junction"),
                ("junction", "route_left"),
                ("route_right", "goal"),
                ("junction", "dead_end_left"),
                ("dead_end_left", "dead_end_tip"),
            ),
        )
    return _u_trap_template()


def _alternate_template(
) -> tuple[tuple[tuple[str, tuple[float, float]], ...], tuple[tuple[str, str], ...]]:
    return (
        (
            ("start", (0.55, 2.00)),
            ("junction_left", (1.35, 2.00)),
            ("upper_left", (2.05, 2.80)),
            ("upper_right", (2.95, 2.80)),
            ("lower_left", (2.05, 1.20)),
            ("lower_right", (2.95, 1.20)),
            ("junction_right", (3.65, 2.00)),
            ("goal", (4.45, 2.00)),
        ),
        (
            ("upper_left", "upper_right"),
            ("start", "junction_left"),
            ("junction_left", "upper_left"),
            ("upper_right", "junction_right"),
            ("junction_left", "lower_left"),
            ("lower_left", "lower_right"),
            ("lower_right", "junction_right"),
            ("junction_right", "goal"),
        ),
    )


def _u_trap_template(
) -> tuple[tuple[tuple[str, tuple[float, float]], ...], tuple[tuple[str, str], ...]]:
    # 평행 통로 중심 간격은 1.20m다. 0.80m 통로끼리 0.40m 벽이 남아 grid 합류를 막는다.
    return (
        (
            ("start", (0.55, 0.55)),
            ("left_top", (0.55, 3.45)),
            ("center_top", (2.50, 3.45)),
            ("right_top", (4.45, 3.45)),
            ("goal", (4.45, 0.55)),
            ("inner_left", (1.75, 0.55)),
            ("inner_top_left", (1.75, 2.25)),
            ("inner_top_right", (3.25, 2.25)),
            ("inner_dead", (3.25, 1.35)),
        ),
        (
            ("center_top", "right_top"),
            ("start", "left_top"),
            ("left_top", "center_top"),
            ("right_top", "goal"),
            ("start", "inner_left"),
            ("inner_left", "inner_top_left"),
            ("inner_top_left", "inner_top_right"),
            ("inner_top_right", "inner_dead"),
        ),
    )


def _make_golden_case(scenario: GoldenScenario, index: int) -> GeneratedCase:
    seed = 8_100_000 + index
    actions: tuple[_Action, ...] = ()
    start = "start"
    goal = "goal"
    corridor_width = 0.80
    family = WorldFamily.CORRIDOR
    forbidden_zones: tuple[ForbiddenZone, ...] = ()

    if scenario in (
        GoldenScenario.ALTERNATE_ROUTE,
        GoldenScenario.SEQUENTIAL_CLOSE_OPEN,
    ):
        coordinates, connections = _alternate_template()
        family = WorldFamily.INTERSECTION
        edge = canonical_edge("upper_left", "upper_right", directed=False)
        if scenario is GoldenScenario.ALTERNATE_ROUTE:
            actions = (_Action(EventKind.CLOSE_EDGE, edge=edge),)
            forbidden_zones = (
                ForbiddenZone("unapproved_upper_route", 2.00, 2.55, 3.00, 3.05),
            )
        else:
            actions = (
                _Action(EventKind.CLOSE_EDGE, edge=edge),
                _Action(EventKind.OPEN_EDGE, edge=edge),
                _Action(EventKind.CLOSE_EDGE, edge=edge),
                _Action(EventKind.OPEN_EDGE, edge=edge),
            )
    elif scenario is GoldenScenario.EQUAL_COST:
        coordinates = (
            ("start", (0.50, 2.00)),
            ("upper", (2.50, 3.00)),
            ("lower", (2.50, 1.00)),
            ("goal", (4.50, 2.00)),
        )
        connections = (
            ("start", "upper"),
            ("upper", "goal"),
            ("start", "lower"),
            ("lower", "goal"),
        )
        family = WorldFamily.INTERSECTION
    elif scenario is GoldenScenario.DEAD_END:
        coordinates = (
            ("start", (0.55, 2.00)),
            ("junction", (1.50, 2.00)),
            ("route", (3.10, 2.80)),
            ("goal", (4.45, 2.00)),
            ("dead_tip", (3.10, 1.00)),
        )
        connections = (
            ("start", "junction"),
            ("junction", "route"),
            ("route", "goal"),
            ("junction", "dead_tip"),
        )
        family = WorldFamily.DEAD_END
    elif scenario is GoldenScenario.ISOLATED_GOAL:
        coordinates = (
            ("start", (0.60, 2.00)),
            ("mid", (2.00, 2.00)),
            ("goal", (4.40, 2.00)),
            ("spur", (2.00, 3.20)),
        )
        connections = (("start", "mid"), ("mid", "spur"))
    elif scenario in (
        GoldenScenario.WIDE_CORRIDOR,
        GoldenScenario.NARROW_DOOR,
        GoldenScenario.PARTIAL_OCCUPANCY,
        GoldenScenario.FULL_BLOCK,
        GoldenScenario.SINGLE_ROUTE,
        GoldenScenario.STALE_INVALID_JUDGMENT,
    ):
        coordinates = (
            ("start", (0.55, 2.00)),
            ("mid", (2.50, 2.00)),
            ("goal", (4.45, 2.00)),
        )
        connections = (("start", "mid"), ("mid", "goal"))
        if scenario is GoldenScenario.WIDE_CORRIDOR:
            corridor_width = 1.20
        elif scenario is GoldenScenario.NARROW_DOOR:
            corridor_width = 0.78
        elif scenario is GoldenScenario.PARTIAL_OCCUPANCY:
            corridor_width = 1.20
            actions = (
                _Action(
                    EventKind.CREATE_OBSTACLE,
                    obstacle_id="partial",
                    obstacle_pose=Pose2D(2.50, 2.38),
                    obstacle_radius_m=0.20,
                ),
            )
        elif scenario is GoldenScenario.FULL_BLOCK:
            actions = (
                _Action(
                    EventKind.CREATE_OBSTACLE,
                    obstacle_id="block",
                    obstacle_pose=Pose2D(2.50, 2.00),
                    obstacle_radius_m=0.50,
                ),
            )
        elif scenario is GoldenScenario.STALE_INVALID_JUDGMENT:
            actions = (
                _Action(EventKind.MOVE_START, node_id="mid"),
                _Action(EventKind.INVALIDATE, input_valid=False),
            )
    else:
        coordinates, connections = _u_trap_template()
        family = WorldFamily.U_TRAP
        start = "inner_dead"
        actions = (
            _Action(
                EventKind.CLOSE_EDGE,
                edge=canonical_edge("start", "inner_left", directed=False),
            ),
        )

    nodes = tuple(Node(node_id, x, y) for node_id, (x, y) in coordinates)
    by_id = {node.node_id: node for node in nodes}
    edges = tuple(
        Edge(
            source,
            target,
            2.2360679775
            if scenario is GoldenScenario.EQUAL_COST
            else round(
                hypot(
                    by_id[source].x - by_id[target].x,
                    by_id[source].y - by_id[target].y,
                ),
                10,
            ),
        )
        for source, target in connections
    )
    world = WorldSpec(
        schema_version=SCHEMA_VERSION,
        generator_version=GENERATOR_VERSION,
        world_id=f"golden_{scenario.value}",
        family=family,
        seed=seed,
        simulation_only=True,
        vehicle_profile_id=VIRTUAL_DOLL_WHEELCHAIR_V0_1.profile_id,
        resolution_m=GRID_RESOLUTION_M,
        width_m=5.0,
        height_m=4.0,
        corridor_width_m=corridor_width,
        directed=False,
        nodes=nodes,
        edges=edges,
        forbidden_zones=forbidden_zones,
        scenario_id=scenario.value,
    )
    episode = _build_episode(
        world,
        seed=seed ^ 0x5F37_59DF,
        split=CorpusSplit.GOLDEN,
        start=start,
        goal=goal,
        actions=actions,
        scenario_id=scenario.value,
    )
    return GeneratedCase(world, episode)


def _path_cost(world: WorldSpec, path: tuple[str, ...]) -> float:
    graph = GraphMap(list(world.nodes), list(world.edges), directed=world.directed)
    return graph.path_cost(path)


def _canonical_value(value: Any) -> Any:
    if is_dataclass(value) and not isinstance(value, type):
        return {field.name: _canonical_value(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(key): _canonical_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_canonical_value(item) for item in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    raise TypeError(f"canonical hash를 만들 수 없는 형식입니다: {type(value).__name__}")
