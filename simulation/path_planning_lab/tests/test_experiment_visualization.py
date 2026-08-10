from pathlib import Path

import matplotlib.pyplot as plt

from hospital_path_lab.contracts import PlanStatus, Pose2D, TrajectoryPoint, Twist2D
from hospital_path_lab.experiment_visualization import (
    save_graph_experiment_plot,
    save_grid_experiment_plot,
)
from hospital_path_lab.map_factory import (
    WorldFamily,
    build_graph_snapshot,
    build_grid_snapshot,
    episode_state_at,
    generate_episode,
    generate_world,
)
from hospital_path_lab.planners import AStarPlanner, DijkstraPlanner, SearchResult


def test_graph_plot_writes_png_with_closed_edge_and_no_figure_leak(tmp_path: Path) -> None:
    world = generate_world(301, WorldFamily.INTERSECTION)
    episode = generate_episode(world, seed=401)
    snapshot = build_graph_snapshot(world, episode, through_step=1)
    results = [
        DijkstraPlanner().plan(snapshot.graph, episode.start, episode.goal),
        AStarPlanner().plan(snapshot.graph, episode.start, episode.goal),
        SearchResult(
            planner="empty_candidate",
            status=PlanStatus.NO_PATH,
            path=(),
            cost=None,
            expanded_nodes=0,
            elapsed_ns=0,
        ),
    ]
    before = set(plt.get_fignums())
    output = save_graph_experiment_plot(snapshot, results, tmp_path / "graph.png")

    assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert output.stat().st_size > 0
    assert set(plt.get_fignums()) == before


def test_grid_plot_writes_overlays_and_empty_plot_without_figure_leak(tmp_path: Path) -> None:
    world = generate_world(302, WorldFamily.CORRIDOR)
    episode = generate_episode(world, seed=402)
    snapshot = build_grid_snapshot(world, episode)
    state = episode_state_at(episode)
    nodes = {node.node_id: node for node in world.nodes}
    start = Pose2D(nodes[state.start].x, nodes[state.start].y)
    middle = Pose2D(nodes["mid_left"].x, nodes["mid_left"].y)
    goal = Pose2D(nodes[state.goal].x, nodes[state.goal].y)
    trajectory = (
        TrajectoryPoint(0.0, start, Twist2D()),
        TrajectoryPoint(1.0, middle, Twist2D(linear=0.2)),
        TrajectoryPoint(2.0, goal, Twist2D()),
    )

    before = set(plt.get_fignums())
    overlay = save_grid_experiment_plot(
        snapshot,
        tmp_path / "nested" / "grid_overlay.png",
        reference_path=(start, middle, goal),
        path=(start, middle, goal),
        trajectory=trajectory,
        robot_trace=(start, middle),
    )
    empty = save_grid_experiment_plot(snapshot, tmp_path / "grid_empty.png")

    for output in (overlay, empty):
        assert output.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        assert output.stat().st_size > 0
    assert set(plt.get_fignums()) == before
