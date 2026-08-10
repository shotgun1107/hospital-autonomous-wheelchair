"""공통 snapshot과 실험 결과를 headless PNG 증거로 저장한다."""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

from hospital_path_lab.contracts import (  # noqa: E402
    GraphSnapshot,
    GridSnapshot,
    Pose2D,
    SnapshotMetadata,
    TrajectoryPoint,
)
from hospital_path_lab.graph import canonical_edge  # noqa: E402
from hospital_path_lab.planners import SearchResult, SearchStatus  # noqa: E402


def save_graph_experiment_plot(
    snapshot: GraphSnapshot,
    results: Sequence[SearchResult] | Iterable[SearchResult],
    output_path: str | Path,
    *,
    title: str = "Global path experiment",
) -> Path:
    """graph, 폐쇄 edge와 여러 전역 planner 결과를 한 PNG로 저장한다."""

    output = _prepare_output(output_path)
    figure, axis = plt.subplots(figsize=(9, 5.5))
    try:
        graph = snapshot.graph
        for edge in graph.edges:
            source = graph.nodes[edge.source]
            target = graph.nodes[edge.target]
            key = canonical_edge(edge.source, edge.target, directed=graph.directed)
            closed = key in graph.closed_edges
            axis.plot(
                (source.x, target.x),
                (source.y, target.y),
                color="tab:red" if closed else "0.72",
                linestyle="--" if closed else "-",
                linewidth=3.0 if closed else 1.5,
                label="closed edge" if closed else None,
                zorder=1,
            )

        colors = ("tab:blue", "tab:green", "tab:purple", "tab:orange", "tab:brown")
        for index, result in enumerate(tuple(results)):
            label = _search_result_label(result)
            points = [graph.nodes[node_id] for node_id in result.path if node_id in graph.nodes]
            if result.status is SearchStatus.FOUND and len(points) == len(result.path) and points:
                axis.plot(
                    [point.x for point in points],
                    [point.y for point in points],
                    marker="o",
                    linewidth=2.5,
                    color=colors[index % len(colors)],
                    label=label,
                    zorder=3,
                )
            else:
                # NO_PATH·빈 경로도 legend에 결과로 남기되 좌표를 요구하지 않는다.
                axis.plot([], [], color=colors[index % len(colors)], label=label)

        for node in graph.nodes.values():
            axis.scatter(node.x, node.y, color="black", s=25, zorder=4)
            axis.annotate(
                node.node_id,
                (node.x, node.y),
                xytext=(0, 7),
                textcoords="offset points",
                ha="center",
                fontsize=8,
            )

        axis.set_title(f"{title}\n{_metadata_text(snapshot.metadata)}")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.2)
        _deduplicated_legend(axis)
        figure.tight_layout()
        figure.savefig(output, dpi=160, format="png")
    finally:
        plt.close(figure)
    return output


def save_grid_experiment_plot(
    snapshot: GridSnapshot,
    output_path: str | Path,
    *,
    reference_path: Sequence[Pose2D] | Iterable[Pose2D] = (),
    path: Sequence[Pose2D] | Iterable[Pose2D] = (),
    trajectory: Sequence[TrajectoryPoint] | Iterable[TrajectoryPoint] = (),
    robot_trace: Sequence[Pose2D] | Iterable[Pose2D] = (),
    title: str = "Local path experiment",
) -> Path:
    """점유 grid와 기준·계획·trajectory·robot trace를 한 PNG로 저장한다."""

    output = _prepare_output(output_path)
    figure, axis = plt.subplots(figsize=(9, 6))
    try:
        grid = snapshot.grid
        extent = (
            grid.origin_x_m,
            grid.origin_x_m + grid.width * grid.resolution_m,
            grid.origin_y_m,
            grid.origin_y_m + grid.height * grid.resolution_m,
        )
        axis.imshow(
            grid.occupancy,
            origin="lower",
            extent=extent,
            interpolation="nearest",
            cmap=ListedColormap(("white", "0.18")),
            vmin=0,
            vmax=1,
            zorder=0,
        )

        _plot_pose_sequence(
            axis,
            tuple(reference_path),
            label="reference",
            color="tab:blue",
            linestyle="--",
            linewidth=1.8,
        )
        _plot_pose_sequence(
            axis,
            tuple(path),
            label="planned path",
            color="tab:green",
            linewidth=2.5,
        )
        trajectory_points = tuple(point.pose for point in trajectory)
        _plot_pose_sequence(
            axis,
            trajectory_points,
            label="trajectory",
            color="tab:orange",
            linewidth=2.0,
        )
        _plot_pose_sequence(
            axis,
            tuple(robot_trace),
            label="robot trace",
            color="tab:red",
            linewidth=1.8,
            marker=".",
        )

        axis.set_title(f"{title}\n{_metadata_text(snapshot.metadata)}")
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.12)
        _deduplicated_legend(axis)
        figure.tight_layout()
        figure.savefig(output, dpi=160, format="png")
    finally:
        plt.close(figure)
    return output


def _prepare_output(output_path: str | Path) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    return output


def _metadata_text(metadata: SnapshotMetadata) -> str:
    return (
        f"map={metadata.map_id} | "
        f"rev={metadata.map_revision}/{metadata.mission_revision}/"
        f"{metadata.observation_revision} | seed={metadata.seed}"
    )


def _search_result_label(result: SearchResult) -> str:
    if result.cost is None:
        return f"{result.planner}: {result.status.value}"
    return f"{result.planner}: {result.status.value}, cost={result.cost:.3f}"


def _plot_pose_sequence(
    axis: plt.Axes,
    poses: tuple[Pose2D, ...],
    *,
    label: str,
    color: str,
    linestyle: str = "-",
    linewidth: float,
    marker: str | None = None,
) -> None:
    if not poses:
        return
    axis.plot(
        [pose.x for pose in poses],
        [pose.y for pose in poses],
        label=label,
        color=color,
        linestyle=linestyle,
        linewidth=linewidth,
        marker=marker,
        zorder=3,
    )


def _deduplicated_legend(axis: plt.Axes) -> None:
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles, strict=False))
    if unique:
        axis.legend(unique.values(), unique.keys(), loc="best", fontsize=8)
