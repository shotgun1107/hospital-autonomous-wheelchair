"""그래프, 폐쇄 구간과 planner 경로의 정적 시각화."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from hospital_path_lab.graph import GraphMap, canonical_edge
from hospital_path_lab.planners import SearchResult, SearchStatus


def save_route_plot(
    graph: GraphMap,
    results: list[SearchResult],
    output_path: str | Path,
    *,
    title: str,
) -> None:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(8, 4.5))

    for edge in graph.edges:
        source = graph.nodes[edge.source]
        target = graph.nodes[edge.target]
        key = canonical_edge(edge.source, edge.target, directed=graph.directed)
        closed = key in graph.closed_edges
        axis.plot(
            [source.x, target.x],
            [source.y, target.y],
            color="tab:red" if closed else "0.75",
            linestyle="--" if closed else "-",
            linewidth=2.5 if closed else 1.5,
            zorder=1,
        )

    colors = ["tab:blue", "tab:green", "tab:purple", "tab:orange"]
    for index, result in enumerate(results):
        if result.status is not SearchStatus.FOUND:
            continue
        points = [graph.nodes[node_id] for node_id in result.path]
        axis.plot(
            [point.x for point in points],
            [point.y for point in points],
            marker="o",
            linewidth=3,
            color=colors[index % len(colors)],
            label=f"{result.planner}: cost={result.cost:.3f}",
            zorder=3,
        )

    for node in graph.nodes.values():
        axis.scatter(node.x, node.y, color="black", s=24, zorder=4)
        axis.text(node.x, node.y + 0.12, node.node_id, ha="center", fontsize=8)

    axis.set_title(title)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.2)
    if axis.get_legend_handles_labels()[0]:
        axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output, dpi=160)
    plt.close(figure)
