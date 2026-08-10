"""지도와 분리된 역할별 알고리즘 registry."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from hospital_path_lab.contracts import LocalPlanner, PathFollower
from hospital_path_lab.followers import PurePursuitFollower, RegulatedPurePursuitFollower
from hospital_path_lab.global_algorithms import DStarLitePlanner
from hospital_path_lab.local_algorithms import BoundedGridAStarPlanner, DynamicWindowPlanner
from hospital_path_lab.planners import AStarPlanner, DijkstraPlanner, Planner


@dataclass(frozen=True, slots=True)
class AlgorithmDescriptor:
    name: str
    role: str
    implementation_status: str
    factory: Callable[[], object] | None


GLOBAL_STATELESS: dict[str, Callable[[], Planner]] = {
    "dijkstra": DijkstraPlanner,
    "astar": AStarPlanner,
}
GLOBAL_INCREMENTAL: dict[str, Callable[[], DStarLitePlanner]] = {
    "dstar_lite": DStarLitePlanner,
}
LOCAL_PLANNERS: dict[str, Callable[[], LocalPlanner]] = {
    "grid_astar": BoundedGridAStarPlanner,
    "dwa": DynamicWindowPlanner,
}
PATH_FOLLOWERS: dict[str, Callable[[], PathFollower]] = {
    "pure_pursuit": PurePursuitFollower,
    "rpp": RegulatedPurePursuitFollower,
}


IMPLEMENTED_ALGORITHMS = (
    AlgorithmDescriptor("dijkstra", "global_oracle", "implemented", DijkstraPlanner),
    AlgorithmDescriptor("astar", "global", "implemented", AStarPlanner),
    AlgorithmDescriptor("dstar_lite", "global_incremental", "implemented", DStarLitePlanner),
    AlgorithmDescriptor("grid_astar", "local_path", "implemented", BoundedGridAStarPlanner),
    AlgorithmDescriptor("dwa", "local_trajectory", "implemented", DynamicWindowPlanner),
    AlgorithmDescriptor("pure_pursuit", "path_follower", "implemented", PurePursuitFollower),
    AlgorithmDescriptor("rpp", "path_follower", "implemented", RegulatedPurePursuitFollower),
)

DEFERRED_ALGORITHMS = (
    AlgorithmDescriptor("teb", "local_trajectory", "deferred", None),
    AlgorithmDescriptor("mppi", "local_trajectory", "deferred", None),
    AlgorithmDescriptor("state_lattice", "global_kinematic", "deferred", None),
    AlgorithmDescriptor("hybrid_astar", "global_kinematic", "deferred", None),
)


def algorithm_manifest() -> list[dict[str, str]]:
    return [
        {
            "name": descriptor.name,
            "role": descriptor.role,
            "implementation_status": descriptor.implementation_status,
        }
        for descriptor in IMPLEMENTED_ALGORITHMS + DEFERRED_ALGORITHMS
    ]
