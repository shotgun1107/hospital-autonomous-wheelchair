"""Seed 기반 open-loop 원형 Actor 시나리오와 궤적 보간."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
from math import hypot, isfinite
from random import Random

from hospital_path_lab.contracts import Pose2D, RobotState
from hospital_path_lab.dynamic_contracts import (
    ACTOR_RADIUS_M,
    DYNAMIC_ACTOR_GENERATOR_VERSION,
    DYNAMIC_CONTROL_PERIOD_S,
    DYNAMIC_SCHEMA_VERSION,
    MAX_ACTOR_SPEED_MPS,
    ActorState,
    ActorWaypoint,
    Point2D,
    Vector2D,
)
from hospital_path_lab.map_factory import canonical_content_hash


@dataclass(frozen=True, slots=True)
class DynamicActorScenario:
    schema_version: str
    generator_version: str
    episode_id: str
    seed: int
    simulation_only: bool
    duration_s: float
    robot_initial_state: RobotState
    reference_path: tuple[Pose2D, ...]
    actor_id: str
    actor_radius_m: float
    actor_waypoints: tuple[ActorWaypoint, ...]
    trajectory_revision: int
    map_revision: int
    mission_revision: int

    def __post_init__(self) -> None:
        if not self.schema_version or not self.generator_version or not self.episode_id:
            raise ValueError("scenario identity fields must not be empty")
        if not self.simulation_only:
            raise ValueError("dynamic Actor scenario must remain simulation_only")
        if not self.actor_id:
            raise ValueError("actor_id must not be empty")
        if not isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("duration_s must be finite and positive")
        if (
            abs(
                self.duration_s / DYNAMIC_CONTROL_PERIOD_S
                - round(self.duration_s / DYNAMIC_CONTROL_PERIOD_S)
            )
            > 1e-12
        ):
            raise ValueError("duration_s must align with the 20 Hz control period")
        if self.actor_radius_m != ACTOR_RADIUS_M:
            raise ValueError(f"actor radius must be {ACTOR_RADIUS_M:.2f} m")
        if min(self.trajectory_revision, self.map_revision, self.mission_revision) < 0:
            raise ValueError("scenario revisions must not be negative")
        robot_pose = self.robot_initial_state.pose
        robot_twist = self.robot_initial_state.twist
        if not all(
            isfinite(value)
            for value in (
                robot_pose.x,
                robot_pose.y,
                robot_pose.yaw,
                robot_twist.linear,
                robot_twist.angular,
            )
        ):
            raise ValueError("robot initial state must be finite")

        reference_path = tuple(self.reference_path)
        waypoints = tuple(self.actor_waypoints)
        if len(reference_path) < 2:
            raise ValueError("reference_path must contain at least two poses")
        if not all(
            isfinite(value) for pose in reference_path for value in (pose.x, pose.y, pose.yaw)
        ):
            raise ValueError("reference_path poses must be finite")
        if len(waypoints) < 2:
            raise ValueError("actor trajectory must contain at least two waypoints")
        if waypoints[0].simulation_time_s != 0.0:
            raise ValueError("actor trajectory must start at simulation time zero")
        if abs(waypoints[-1].simulation_time_s - self.duration_s) > 1e-12:
            raise ValueError("last actor waypoint must match duration_s")
        for source, target in zip(waypoints, waypoints[1:], strict=False):
            dt = target.simulation_time_s - source.simulation_time_s
            if dt <= 0.0:
                raise ValueError("actor waypoint times must be strictly increasing")
            distance = hypot(
                target.position.x - source.position.x,
                target.position.y - source.position.y,
            )
            if distance / dt > MAX_ACTOR_SPEED_MPS + 1e-12:
                raise ValueError("actor segment exceeds maximum speed")
        object.__setattr__(self, "reference_path", reference_path)
        object.__setattr__(self, "actor_waypoints", waypoints)

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)

    @property
    def tick_count(self) -> int:
        return round(self.duration_s / DYNAMIC_CONTROL_PERIOD_S)


def generate_corridor_crossing_scenario(seed: int) -> DynamicActorScenario:
    """복도 기준 경로를 한 명의 Actor가 횡단하는 재현 가능한 시나리오를 만든다."""

    rng = Random(seed)
    crossing_x = round(rng.uniform(1.80, 2.20), 6)
    start_tick = rng.randint(10, 20)
    start_time_s = start_tick * DYNAMIC_CONTROL_PERIOD_S
    crossing_speed_mps = rng.choice((0.35, 0.40, 0.45))
    start = Point2D(crossing_x, 0.25)
    end = Point2D(crossing_x, 1.75)
    crossing_duration_s = (end.y - start.y) / crossing_speed_mps
    crossing_end_s = start_time_s + crossing_duration_s
    duration_s = 6.50
    if crossing_end_s >= duration_s:
        raise AssertionError("corridor crossing template duration is too short")

    return DynamicActorScenario(
        schema_version=DYNAMIC_SCHEMA_VERSION,
        generator_version=DYNAMIC_ACTOR_GENERATOR_VERSION,
        episode_id="corridor_crossing_v1",
        seed=seed,
        simulation_only=True,
        duration_s=duration_s,
        robot_initial_state=RobotState(Pose2D(0.50, 1.00, 0.0)),
        reference_path=(Pose2D(0.50, 1.00, 0.0), Pose2D(3.50, 1.00, 0.0)),
        actor_id="actor_001",
        actor_radius_m=ACTOR_RADIUS_M,
        actor_waypoints=(
            ActorWaypoint(0.0, start),
            ActorWaypoint(start_time_s, start),
            ActorWaypoint(crossing_end_s, end),
            ActorWaypoint(duration_s, end),
        ),
        trajectory_revision=1,
        map_revision=1,
        mission_revision=1,
    )


def actor_state_at(scenario: DynamicActorScenario, simulation_time_s: float) -> ActorState:
    """Piecewise-linear trajectory를 right-continuous velocity로 보간한다."""

    if not isfinite(simulation_time_s):
        raise ValueError("simulation_time_s must be finite")
    if simulation_time_s < 0.0 or simulation_time_s > scenario.duration_s + 1e-12:
        raise ValueError("simulation_time_s is outside the scenario")

    waypoints = scenario.actor_waypoints
    if simulation_time_s >= waypoints[-1].simulation_time_s:
        return ActorState(
            actor_id=scenario.actor_id,
            position=waypoints[-1].position,
            velocity=Vector2D(0.0, 0.0),
            radius_m=scenario.actor_radius_m,
            trajectory_revision=scenario.trajectory_revision,
        )

    times = tuple(waypoint.simulation_time_s for waypoint in waypoints)
    index = min(bisect_right(times, simulation_time_s) - 1, len(waypoints) - 2)
    source = waypoints[index]
    target = waypoints[index + 1]
    duration = target.simulation_time_s - source.simulation_time_s
    fraction = (simulation_time_s - source.simulation_time_s) / duration
    dx = target.position.x - source.position.x
    dy = target.position.y - source.position.y
    return ActorState(
        actor_id=scenario.actor_id,
        position=Point2D(
            source.position.x + fraction * dx,
            source.position.y + fraction * dy,
        ),
        velocity=Vector2D(dx / duration, dy / duration),
        radius_m=scenario.actor_radius_m,
        trajectory_revision=scenario.trajectory_revision,
    )
