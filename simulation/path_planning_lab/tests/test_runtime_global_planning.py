"""Public tests for automatic known-map planning at the R7 runtime boundary."""

from __future__ import annotations

from dataclasses import replace

import pytest

from hospital_path_lab.dynamic_contracts import DynamicMotionState
from hospital_path_lab.dynamic_observation import DynamicObservationProfileName
from hospital_path_lab.runtime import (
    R7Runtime,
    RuntimeConfig,
    RuntimeControllerKind,
    RuntimeGlobalPlannerKind,
    RuntimeMap,
    RuntimeMission,
    RuntimeObservation,
    RuntimePlanningError,
    RuntimePose,
    RuntimeRobotState,
    RuntimeStepInput,
)


def _config() -> RuntimeConfig:
    return RuntimeConfig(
        controller_kind=RuntimeControllerKind.RPP,
        global_planner_kind=RuntimeGlobalPlannerKind.GRID_ASTAR,
        observation_profile=DynamicObservationProfileName.FUNCTIONAL_IDEAL,
        require_native_dwb=False,
    )


def _map(
    *,
    width: int = 30,
    height: int = 30,
    occupied: frozenset[tuple[int, int]] = frozenset(),
    forbidden: tuple[tuple[int, int], ...] = (),
) -> RuntimeMap:
    return RuntimeMap(
        map_id="runtime-global-map",
        map_revision=1,
        occupancy_rows=tuple(
            tuple((x, y) in occupied for x in range(width)) for y in range(height)
        ),
        resolution_m=0.1,
        forbidden_cells=forbidden,
    )


def _mission(
    *,
    runtime_map: RuntimeMap | None = None,
    start: RuntimePose | None = None,
    goal: RuntimePose | None = None,
    reference_path: tuple[RuntimePose, ...] | None = None,
) -> RuntimeMission:
    start = RuntimePose(0.6, 1.5, 0.0) if start is None else start
    goal = RuntimePose(2.4, 1.5, 0.0) if goal is None else goal
    return RuntimeMission(
        mission_id="runtime-global-mission",
        mission_revision=1,
        runtime_map=_map() if runtime_map is None else runtime_map,
        start_pose=start,
        goal_pose=goal,
        observation_stream_id="runtime-global-camera",
        observation_session_seed=501,
        reference_path=reference_path,
    )


def _step_to_first_command(runtime: R7Runtime, start: RuntimePose):
    robot = RuntimeRobotState(start)
    runtime.step(RuntimeStepInput(control_tick=0, robot=robot))
    runtime.step(RuntimeStepInput(control_tick=1, robot=robot))
    return runtime.step(
        RuntimeStepInput(
            control_tick=2,
            robot=robot,
            observation=RuntimeObservation(
                sequence=0,
                observation_revision=0,
                observed_at_s=0.0,
            ),
        )
    )


def test_a_b_c_map_start_goal_create_exact_endpoint_reference() -> None:
    mission = _mission()
    runtime = R7Runtime(_config())

    runtime.start_mission(mission)

    assert mission.reference_path is None
    assert runtime.reference_path is not None
    assert len(runtime.reference_path) >= 2
    assert runtime.reference_path[0] == mission.start_pose
    assert runtime.reference_path[-1] == mission.goal_pose


def test_d_automatic_reference_routes_around_static_and_forbidden_cells() -> None:
    wall = frozenset((15, y) for y in range(19))
    runtime_map = _map(width=30, height=40, occupied=wall, forbidden=((15, 19),))
    mission = _mission(runtime_map=runtime_map)
    runtime = R7Runtime(_config())

    runtime.start_mission(mission)

    assert runtime.reference_path is not None
    cells = tuple(
        (
            int((pose.x_m - runtime_map.origin_x_m) // runtime_map.resolution_m),
            int((pose.y_m - runtime_map.origin_y_m) // runtime_map.resolution_m),
        )
        for pose in runtime.reference_path
    )
    assert not set(cells) & (set(wall) | set(runtime_map.forbidden_cells))
    assert max(pose.y_m for pose in runtime.reference_path) > mission.start_pose.y_m


def test_e_no_path_fails_mission_start_without_straight_fallback() -> None:
    blocking_wall = frozenset((15, y) for y in range(40))
    runtime = R7Runtime(_config())

    with pytest.raises(RuntimePlanningError, match="no_path"):
        runtime.start_mission(
            _mission(runtime_map=_map(width=30, height=40, occupied=blocking_wall))
        )

    assert not runtime.mission_started


@pytest.mark.parametrize(
    ("start", "goal", "reason"),
    [
        (RuntimePose(-0.1, 1.5), RuntimePose(2.4, 1.5), "start_out_of_bounds"),
        (RuntimePose(0.6, 1.5), RuntimePose(3.1, 1.5), "goal_out_of_bounds"),
    ],
)
def test_f_out_of_bounds_endpoint_is_rejected(
    start: RuntimePose,
    goal: RuntimePose,
    reason: str,
) -> None:
    runtime = R7Runtime(_config())

    with pytest.raises(RuntimePlanningError, match=reason):
        runtime.start_mission(_mission(start=start, goal=goal))


@pytest.mark.parametrize(
    ("occupied", "forbidden", "reason"),
    [
        (frozenset({(5, 15)}), (), "start_footprint_occupied"),
        (frozenset(), ((23, 15),), "goal_forbidden"),
    ],
)
def test_f_occupied_or_forbidden_endpoint_is_rejected(
    occupied: frozenset[tuple[int, int]],
    forbidden: tuple[tuple[int, int], ...],
    reason: str,
) -> None:
    runtime = R7Runtime(_config())
    runtime_map = _map(occupied=occupied, forbidden=forbidden)

    with pytest.raises(RuntimePlanningError, match=reason):
        runtime.start_mission(_mission(runtime_map=runtime_map))


def test_g_automatic_reference_runs_the_existing_r7_pipeline() -> None:
    mission = _mission()
    runtime = R7Runtime(_config())
    runtime.start_mission(mission)

    command = _step_to_first_command(runtime, mission.start_pose)

    assert command.motion_state is DynamicMotionState.MOVING
    assert command.linear_mps > 0.0


def test_h_explicit_reference_override_remains_supported() -> None:
    explicit = (
        RuntimePose(0.6, 1.5, 0.0),
        RuntimePose(1.5, 1.5, 0.0),
        RuntimePose(2.4, 1.5, 0.0),
    )
    runtime = R7Runtime(_config())

    runtime.start_mission(_mission(reference_path=explicit))

    assert runtime.reference_path == explicit


def test_i_automatic_and_same_explicit_reference_have_equal_first_command() -> None:
    automatic_mission = _mission()
    automatic = R7Runtime(_config())
    automatic.start_mission(automatic_mission)
    resolved = automatic.reference_path
    assert resolved is not None

    explicit_mission = replace(automatic_mission, reference_path=resolved)
    explicit = R7Runtime(_config())
    explicit.start_mission(explicit_mission)

    automatic_command = _step_to_first_command(automatic, automatic_mission.start_pose)
    explicit_command = _step_to_first_command(explicit, explicit_mission.start_pose)

    assert automatic_command == explicit_command
