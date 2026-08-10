from collections import Counter, deque
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.graph import GraphMap
from hospital_path_lab.map_factory import (
    DEFAULT_BATCH_SIZE,
    GRID_RESOLUTION_M,
    CorpusSplit,
    Event,
    EventKind,
    GoldenScenario,
    WorldFamily,
    build_graph_snapshot,
    build_grid_snapshot,
    episode_state_at,
    freeze_batch,
    generate_batch,
    generate_episode,
    generate_golden_cases,
    generate_world,
    validate_episode,
    validate_frozen_batch,
)

CORPUS_DIRECTORY = Path(__file__).parents[1] / "corpus" / "map_factory"


def test_same_seed_produces_identical_world_episode_and_hashes() -> None:
    first_world = generate_world(1729, WorldFamily.INTERSECTION)
    second_world = generate_world(1729, WorldFamily.INTERSECTION)
    first_episode = generate_episode(first_world, seed=2718, split=CorpusSplit.GOLDEN)
    second_episode = generate_episode(second_world, seed=2718, split=CorpusSplit.GOLDEN)

    assert first_world == second_world
    assert first_world.content_hash == second_world.content_hash
    assert first_episode == second_episode
    assert first_episode.content_hash == second_episode.content_hash


def test_different_seed_changes_canonical_hash() -> None:
    first = generate_world(100, WorldFamily.CORRIDOR)
    second = generate_world(101, WorldFamily.CORRIDOR)
    assert first.content_hash != second.content_hash


def test_same_world_builds_graph_and_two_centimeter_grid_snapshots() -> None:
    world = generate_world(55, WorldFamily.U_TRAP)
    episode = generate_episode(world, seed=89, split=CorpusSplit.DEVELOPMENT)

    graph_snapshot = build_graph_snapshot(world, episode)
    grid_snapshot = build_grid_snapshot(world, episode)

    assert graph_snapshot.metadata == grid_snapshot.metadata
    assert graph_snapshot.metadata.map_id == world.world_id
    assert set(graph_snapshot.graph.nodes) == {node.node_id for node in world.nodes}
    assert grid_snapshot.grid.resolution_m == GRID_RESOLUTION_M

    state = episode_state_at(episode)
    nodes = {node.node_id: node for node in world.nodes}
    for node_id in (state.start, state.goal):
        node = nodes[node_id]
        assert not grid_snapshot.grid.is_occupied(
            grid_snapshot.grid.world_to_cell(Pose2D(node.x, node.y))
        )


def test_invalid_open_transition_is_rejected() -> None:
    world = generate_world(77, WorldFamily.DEAD_END)
    episode = generate_episode(world, seed=99)
    invalid_event = Event(
        step=1,
        kind=EventKind.OPEN_EDGE,
        edge=(world.edges[0].source, world.edges[0].target),
        map_revision=1,
        mission_revision=0,
        observation_revision=0,
        expected_path_exists=True,
    )
    invalid_episode = replace(episode, events=(invalid_event,))

    with pytest.raises(ValueError, match="열려 있는 edge를 다시 엶"):
        validate_episode(world, invalid_episode)


def test_default_batch_keeps_ten_case_split_and_freeze_semantics() -> None:
    first = generate_batch(base_seed=20260810)
    second = generate_batch(base_seed=20260810)

    assert len(first) == DEFAULT_BATCH_SIZE
    assert [item.world.content_hash for item in first] == [
        item.world.content_hash for item in second
    ]
    assert Counter(item.episode.split for item in first) == {
        CorpusSplit.GOLDEN: 2,
        CorpusSplit.DEVELOPMENT: 4,
        CorpusSplit.HIDDEN: 2,
        CorpusSplit.REGRESSIONS: 2,
    }
    assert {item.world.family for item in first} == set(WorldFamily)
    assert all(item.world.simulation_only and item.episode.simulation_only for item in first)
    assert all(len(item.episode.events) == 10 for item in first)
    assert {event.kind for item in first for event in item.episode.events} == set(EventKind)

    frozen = freeze_batch(first, corpus_id="development_batch_20260810")
    validate_frozen_batch(second, frozen)
    changed = list(second)
    changed[0] = replace(
        changed[0],
        episode=replace(changed[0].episode, seed=changed[0].episode.seed + 1),
    )
    with pytest.raises(ValueError, match="content hash"):
        validate_frozen_batch(tuple(changed), frozen)


def test_checked_in_frozen_manifest_matches_generated_content_hashes() -> None:
    manifest = yaml.safe_load(
        (CORPUS_DIRECTORY / "generated_frozen.yaml").read_text(encoding="utf-8")
    )
    batch = generate_batch(base_seed=manifest["base_seed"], size=manifest["batch_size"])
    frozen = freeze_batch(batch, corpus_id=manifest["corpus_id"])

    assert frozen.content_hash == manifest["frozen_content_hash"]
    assert [(case.world.content_hash, case.episode.content_hash) for case in batch] == [
        (item["world_content_hash"], item["episode_content_hash"])
        for item in manifest["cases"]
    ]


def test_obstacle_lifecycle_updates_ground_truth_and_observation_revision() -> None:
    world = generate_world(1001, WorldFamily.CORRIDOR)
    episode = generate_episode(world, seed=1002)
    create_event, move_event, remove_event = episode.events[2:5]

    assert [event.kind for event in (create_event, move_event, remove_event)] == [
        EventKind.CREATE_OBSTACLE,
        EventKind.MOVE_OBSTACLE,
        EventKind.REMOVE_OBSTACLE,
    ]
    assert [event.observation_revision for event in (create_event, move_event, remove_event)] == [
        1,
        2,
        3,
    ]
    assert all(
        event.map_revision == 2 and event.mission_revision == 0
        for event in episode.events[2:5]
    )

    created_state = episode_state_at(episode, through_step=create_event.step)
    moved_state = episode_state_at(episode, through_step=move_event.step)
    removed_state = episode_state_at(episode, through_step=remove_event.step)
    assert len(created_state.obstacles) == len(moved_state.obstacles) == 1
    assert created_state.obstacles[0].pose != moved_state.obstacles[0].pose
    assert removed_state.obstacles == ()

    before = build_grid_snapshot(world, episode, through_step=2)
    created = build_grid_snapshot(world, episode, through_step=create_event.step)
    obstacle_cell = created.grid.world_to_cell(created_state.obstacles[0].pose)
    assert not before.grid.is_occupied(obstacle_cell)
    assert created.grid.is_occupied(obstacle_cell)
    assert before.metadata.content_hash != created.metadata.content_hash


def test_revision_transitions_and_invalid_snapshot_metadata_are_explicit() -> None:
    world = generate_world(2001, WorldFamily.INTERSECTION)
    episode = generate_episode(world, seed=2002)

    assert (episode.events[6].map_revision, episode.events[6].mission_revision) == (4, 0)
    assert (episode.events[8].map_revision, episode.events[8].mission_revision) == (4, 2)
    invalid = build_grid_snapshot(world, episode, through_step=10)
    assert invalid.metadata.observation_revision == 4
    assert invalid.metadata.input_valid is False


def test_manual_golden_manifest_and_hashes_match_all_twelve_cases() -> None:
    cases = generate_golden_cases()
    manifest = yaml.safe_load((CORPUS_DIRECTORY / "golden.yaml").read_text(encoding="utf-8"))

    assert len(cases) == 12
    assert [item.episode.scenario_id for item in cases] == [
        scenario.value for scenario in GoldenScenario
    ]
    assert [item.episode.scenario_id for item in cases] == [
        item["scenario_id"] for item in manifest["cases"]
    ]
    assert [(item.world.content_hash, item.episode.content_hash) for item in cases] == [
        (item["world_content_hash"], item["episode_content_hash"])
        for item in manifest["cases"]
    ]
    assert manifest["simulation_only"] is True
    assert manifest["decision_boundary"]["g1_g5"] == "unconfirmed"
    assert manifest["decision_boundary"]["final_algorithm_selected"] is False


def test_equal_cost_golden_has_exactly_equal_branches_without_jitter() -> None:
    case = _golden(GoldenScenario.EQUAL_COST)
    graph = GraphMap(list(case.world.nodes), list(case.world.edges))

    assert graph.path_cost(("start", "upper", "goal")) == graph.path_cost(
        ("start", "lower", "goal")
    )
    assert {(node.x, node.y) for node in case.world.nodes} == {
        (0.5, 2.0),
        (2.5, 3.0),
        (2.5, 1.0),
        (4.5, 2.0),
    }


@pytest.mark.parametrize(
    "scenario",
    [GoldenScenario.NARROW_DOOR, GoldenScenario.WIDE_CORRIDOR],
)
def test_door_widths_are_checked_on_footprint_configuration_grid(
    scenario: GoldenScenario,
) -> None:
    case = _golden(scenario)
    snapshot = build_grid_snapshot(case.world, case.episode)
    checker = CollisionChecker(snapshot.grid, forbidden_cells=snapshot.forbidden_cells)
    start = _node_pose(case, case.episode.start)
    goal = _node_pose(case, case.episode.goal)

    assert _path_exists(checker.configuration_grid, start, goal)


def test_u_trap_has_no_grid_shortcut_across_separating_walls() -> None:
    case = _golden(GoldenScenario.U_TRAP)
    assert case.episode.events[0].expected_path_exists is False
    assert case.episode.events[0].expected_grid_path_exists is False

    snapshot = build_grid_snapshot(case.world, case.episode, through_step=1)
    assert not _path_exists(
        snapshot.grid,
        _node_pose(case, case.episode.start),
        _node_pose(case, case.episode.goal),
    )


def test_partial_and_full_obstacles_have_distinct_expected_grid_outcomes() -> None:
    partial = _golden(GoldenScenario.PARTIAL_OCCUPANCY)
    full = _golden(GoldenScenario.FULL_BLOCK)

    assert partial.episode.events[0].expected_grid_path_exists is True
    assert full.episode.events[0].expected_path_exists is True
    assert full.episode.events[0].expected_grid_path_exists is False


def test_forbidden_region_is_separate_from_physical_occupancy_and_hashed() -> None:
    case = _golden(GoldenScenario.ALTERNATE_ROUTE)
    snapshot = build_grid_snapshot(case.world, case.episode)

    free_forbidden = [
        cell for cell in snapshot.forbidden_cells if not snapshot.grid.is_occupied(cell)
    ]
    assert free_forbidden
    assert case.world.forbidden_zones[0].zone_id == "unapproved_upper_route"
    without_zone = replace(case.world, forbidden_zones=())
    assert without_zone.content_hash != case.world.content_hash


def _golden(scenario: GoldenScenario):
    return next(
        item for item in generate_golden_cases() if item.episode.scenario_id == scenario.value
    )


def _node_pose(case, node_id: str) -> Pose2D:
    node = next(node for node in case.world.nodes if node.node_id == node_id)
    return Pose2D(node.x, node.y)


def _path_exists(grid, start: Pose2D, goal: Pose2D) -> bool:
    start_cell = grid.world_to_cell(start)
    goal_cell = grid.world_to_cell(goal)
    if grid.is_occupied(start_cell) or grid.is_occupied(goal_cell):
        return False
    pending = deque([start_cell])
    visited = {start_cell}
    while pending:
        current = pending.popleft()
        if current == goal_cell:
            return True
        for neighbor, _ in grid.neighbors8(current):
            if neighbor not in visited:
                visited.add(neighbor)
                pending.append(neighbor)
    return False
