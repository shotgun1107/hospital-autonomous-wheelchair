from __future__ import annotations

from dataclasses import FrozenInstanceError, replace
from math import hypot

import pytest

from hospital_path_lab.dynamic_actor import (
    actor_state_at,
    generate_corridor_crossing_scenario,
)
from hospital_path_lab.dynamic_contracts import (
    ACTOR_RADIUS_M,
    MAX_ACTOR_SPEED_MPS,
    ActorWaypoint,
    DynamicGroundTruthFrame,
    Point2D,
)


def test_same_seed_reproduces_scenario_and_world_hash() -> None:
    first = generate_corridor_crossing_scenario(20260810)
    second = generate_corridor_crossing_scenario(20260810)

    assert first == second
    assert first.content_hash == second.content_hash
    assert first.simulation_only is True
    assert first.actor_radius_m == ACTOR_RADIUS_M
    assert first.episode_id == "corridor_crossing_v1"


def test_different_seed_changes_scenario_within_frozen_limits() -> None:
    first = generate_corridor_crossing_scenario(11)
    second = generate_corridor_crossing_scenario(12)

    assert first.content_hash != second.content_hash
    assert first.actor_waypoints != second.actor_waypoints
    for scenario in (first, second):
        crossing_x = scenario.actor_waypoints[0].position.x
        assert 1.80 <= crossing_x <= 2.20
        assert scenario.actor_waypoints[0].position.y == 0.25
        assert scenario.actor_waypoints[-1].position.y == 1.75
        for source, target in zip(
            scenario.actor_waypoints,
            scenario.actor_waypoints[1:],
            strict=False,
        ):
            distance = hypot(
                target.position.x - source.position.x,
                target.position.y - source.position.y,
            )
            speed = distance / (target.simulation_time_s - source.simulation_time_s)
            assert speed <= MAX_ACTOR_SPEED_MPS


def test_piecewise_linear_state_waits_moves_and_stops() -> None:
    scenario = generate_corridor_crossing_scenario(31)
    wait_end = scenario.actor_waypoints[1].simulation_time_s
    crossing_end = scenario.actor_waypoints[2].simulation_time_s

    waiting = actor_state_at(scenario, wait_end / 2.0)
    moving = actor_state_at(scenario, wait_end + 0.25)
    stopped = actor_state_at(scenario, scenario.duration_s)

    assert waiting.velocity.magnitude == 0.0
    assert moving.velocity.x == 0.0
    assert 0.0 < moving.velocity.y <= MAX_ACTOR_SPEED_MPS
    assert scenario.actor_waypoints[0].position.y < moving.position.y < 1.75
    assert crossing_end < scenario.duration_s
    assert stopped.position == scenario.actor_waypoints[-1].position
    assert stopped.velocity.magnitude == 0.0


def test_actor_contracts_reject_invalid_radius_speed_time_and_nonfinite_values() -> None:
    scenario = generate_corridor_crossing_scenario(41)

    with pytest.raises(ValueError, match="actor radius"):
        replace(scenario, actor_radius_m=0.20)
    with pytest.raises(ValueError, match="maximum speed"):
        replace(
            scenario,
            actor_waypoints=(
                ActorWaypoint(0.0, Point2D(0.0, 0.0)),
                ActorWaypoint(scenario.duration_s, Point2D(0.0, 4.0)),
            ),
        )
    with pytest.raises(ValueError, match="strictly increasing"):
        replace(
            scenario,
            actor_waypoints=(
                ActorWaypoint(0.0, Point2D(0.0, 0.0)),
                ActorWaypoint(1.0, Point2D(0.0, 0.1)),
                ActorWaypoint(1.0, Point2D(0.0, 0.2)),
                ActorWaypoint(scenario.duration_s, Point2D(0.0, 0.3)),
            ),
        )
    with pytest.raises(ValueError, match="finite"):
        Point2D(float("nan"), 0.0)
    with pytest.raises(ValueError, match="outside"):
        actor_state_at(scenario, scenario.duration_s + 1.0)


def test_actor_scenario_is_immutable() -> None:
    scenario = generate_corridor_crossing_scenario(51)

    with pytest.raises(FrozenInstanceError):
        scenario.seed = 99  # type: ignore[misc]


def test_ground_truth_frame_rejects_duplicate_actor_ids() -> None:
    scenario = generate_corridor_crossing_scenario(61)
    actor = actor_state_at(scenario, 0.0)

    with pytest.raises(ValueError, match="unique"):
        DynamicGroundTruthFrame(
            episode_id=scenario.episode_id,
            seed=scenario.seed,
            tick_id=0,
            simulation_time_s=0.0,
            robot_state=scenario.robot_initial_state,
            actors=(actor, actor),
            map_revision=scenario.map_revision,
            mission_revision=scenario.mission_revision,
        )
