"""Simulation-only local reference policy for directional Actor bypass research.

The source-derived DWB core is a trajectory controller: when its reference path
continues through a slower Actor, its path critics correctly prefer following
and stopping over an unrequested lane departure.  This module keeps that core
unchanged and supplies an explicit, controller-external local reference only
for a same-heading Actor that is ahead and close to the current reference.

It consumes no corpus label, evaluator oracle, hidden data, or ground truth.
The resulting path is still only a proposal; every generated DWB trajectory is
checked by the unchanged project constraint and shared online safety gate.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from math import atan2, ceil, hypot, isfinite

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.dynamic_contracts import (
    ControllerSnapshot,
    build_controller_snapshot,
)
from hospital_path_lab.dynamic_directional_prediction import DirectionalPredictionSet
from hospital_path_lab.local_algorithms.dwb_reference import (
    SourceDerivedDwbConfig,
    SourceDerivedDynamicDwbController,
)


class LocalDetourPolicyState(StrEnum):
    TRACK_REFERENCE = "track_reference"
    DETOUR_REFERENCE_ACTIVE = "detour_reference_active"


@dataclass(frozen=True, slots=True)
class LocalDetourPolicyConfig:
    trigger_distance_m: float = 2.0
    maximum_actor_cross_track_m: float = 0.40
    minimum_heading_alignment: float = 0.80
    lateral_offset_m: float = 0.70
    pass_lead_m: float = 1.80
    rejoin_lead_m: float = 0.50
    geometry_sample_step_m: float = 0.05
    waypoint_tolerance_m: float = 0.08

    def __post_init__(self) -> None:
        values = (
            self.trigger_distance_m,
            self.maximum_actor_cross_track_m,
            self.minimum_heading_alignment,
            self.lateral_offset_m,
            self.pass_lead_m,
            self.rejoin_lead_m,
            self.geometry_sample_step_m,
            self.waypoint_tolerance_m,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("local detour configuration must be finite")
        if min(values) <= 0.0 or self.minimum_heading_alignment > 1.0:
            raise ValueError("local detour configuration is outside its valid range")


class DirectionalLocalDetourPolicy:
    """Latch one conservative rectangular local reference from public input."""

    def __init__(self, config: LocalDetourPolicyConfig | None = None) -> None:
        self.config = config or LocalDetourPolicyConfig()
        self._reference_path: tuple[Pose2D, ...] | None = None

    @property
    def state(self) -> LocalDetourPolicyState:
        return (
            LocalDetourPolicyState.TRACK_REFERENCE
            if self._reference_path is None
            else LocalDetourPolicyState.DETOUR_REFERENCE_ACTIVE
        )

    @property
    def active_reference_path(self) -> tuple[Pose2D, ...] | None:
        return self._reference_path

    def reset(self) -> None:
        self._reference_path = None

    def restore_reference_path(self, path: tuple[Pose2D, ...]) -> None:
        if self._reference_path is not None:
            raise ValueError("local detour reference is already active")
        frozen = tuple(path)
        if len(frozen) != 5 or not all(isinstance(pose, Pose2D) for pose in frozen):
            raise ValueError("restored local detour reference must contain five poses")
        self._reference_path = frozen

    def apply(self, snapshot: ControllerSnapshot) -> ControllerSnapshot:
        if not isinstance(snapshot, ControllerSnapshot):
            raise TypeError("local detour policy input must be a ControllerSnapshot")
        if self._reference_path is None:
            self._reference_path = self._build_reference(snapshot)
        if self._reference_path is None:
            return snapshot
        return build_controller_snapshot(
            tick_id=snapshot.tick_id,
            simulation_time_s=snapshot.simulation_time_s,
            mission_id=snapshot.mission_id,
            robot_state=snapshot.robot_state,
            goal_pose=snapshot.goal_pose,
            reference_path=self._reference_path,
            static_grid_snapshot=snapshot.static_grid_snapshot,
            validated_observation=snapshot.validated_observation,
            actor_tubes=snapshot.actor_tubes,
            vehicle_profile=snapshot.vehicle_profile,
        )

    def _build_reference(
        self,
        snapshot: ControllerSnapshot,
    ) -> tuple[Pose2D, ...] | None:
        prediction = snapshot.actor_tubes
        if not isinstance(prediction, DirectionalPredictionSet) or not prediction.tubes:
            return None
        source = snapshot.reference_path[0]
        target = snapshot.reference_path[-1]
        direction_x = target.x - source.x
        direction_y = target.y - source.y
        length = hypot(direction_x, direction_y)
        if length <= 1e-12:
            return None
        direction_x /= length
        direction_y /= length
        normal_x, normal_y = -direction_y, direction_x
        robot = snapshot.robot_state.pose

        blockers: list[tuple[float, object]] = []
        for tube in prediction.tubes:
            relative_x = tube.anchor_position.x - robot.x
            relative_y = tube.anchor_position.y - robot.y
            along = relative_x * direction_x + relative_y * direction_y
            cross = relative_x * normal_x + relative_y * normal_y
            alignment = (
                tube.heading_unit.x * direction_x
                + tube.heading_unit.y * direction_y
            )
            if (
                0.0 < along <= self.config.trigger_distance_m
                and abs(cross) <= self.config.maximum_actor_cross_track_m
                and alignment >= self.config.minimum_heading_alignment
            ):
                blockers.append((along, tube))
        if not blockers:
            return None

        actor_along, _tube = min(blockers, key=lambda item: item[0])
        maximum_pass_along = length - self.config.rejoin_lead_m
        pass_along = min(actor_along + self.config.pass_lead_m, maximum_pass_along)
        if pass_along <= self.config.rejoin_lead_m:
            return None

        candidates = tuple(
            self._candidate_path(
                robot=robot,
                goal=target,
                direction=(direction_x, direction_y),
                normal=(normal_x, normal_y),
                pass_along=pass_along,
                side=side,
            )
            for side in (-1.0, 1.0)
        )
        checker = CollisionChecker(
            snapshot.static_grid_snapshot.grid,
            snapshot.vehicle_profile,
            forbidden_cells=snapshot.static_grid_snapshot.forbidden_cells,
        )
        valid = tuple(
            (self._path_minimum_clearance(path, checker), path)
            for path in candidates
            if self._path_is_valid(path, checker)
        )
        if not valid:
            return None
        if len(valid) == 2 and abs(valid[0][0] - valid[1][0]) <= 1e-9:
            # The generator's stable angular ordering starts on the negative
            # side.  Keep that deterministic side when geometry is equivalent.
            return valid[0][1]
        return max(valid, key=lambda item: item[0])[1]

    def _candidate_path(
        self,
        *,
        robot: Pose2D,
        goal: Pose2D,
        direction: tuple[float, float],
        normal: tuple[float, float],
        pass_along: float,
        side: float,
    ) -> tuple[Pose2D, ...]:
        dx, dy = direction
        nx, ny = normal
        offset_x = robot.x + side * nx * self.config.lateral_offset_m
        offset_y = robot.y + side * ny * self.config.lateral_offset_m
        pass_x = offset_x + dx * pass_along
        pass_y = offset_y + dy * pass_along
        rejoin_x = robot.x + dx * (pass_along + self.config.rejoin_lead_m)
        rejoin_y = robot.y + dy * (pass_along + self.config.rejoin_lead_m)
        lateral_yaw = atan2(side * ny, side * nx)
        forward_yaw = atan2(dy, dx)
        return (
            Pose2D(robot.x, robot.y, lateral_yaw),
            Pose2D(offset_x, offset_y, forward_yaw),
            Pose2D(pass_x, pass_y, forward_yaw),
            Pose2D(rejoin_x, rejoin_y, forward_yaw),
            goal,
        )

    def _path_is_valid(self, path, checker: CollisionChecker) -> bool:
        return all(
            checker.clearance(pose) > 0.0 and not checker.pose_enters_forbidden(pose)
            for pose in self._sample_path(path)
        )

    def _path_minimum_clearance(self, path, checker: CollisionChecker) -> float:
        return min(checker.clearance(pose) for pose in self._sample_path(path))

    def _sample_path(self, path: tuple[Pose2D, ...]) -> tuple[Pose2D, ...]:
        samples: list[Pose2D] = []
        for start, end in zip(path[:-1], path[1:], strict=True):
            distance = hypot(end.x - start.x, end.y - start.y)
            count = max(1, ceil(distance / self.config.geometry_sample_step_m))
            yaw = atan2(end.y - start.y, end.x - start.x)
            samples.extend(
                Pose2D(
                    start.x + (end.x - start.x) * index / count,
                    start.y + (end.y - start.y) * index / count,
                    yaw,
                )
                for index in range(count)
            )
        samples.append(path[-1])
        return tuple(samples)


class DirectionalLocalDetourDwbController:
    """Feed DWB one label-free local waypoint at a time.

    DWB's GoalDist critic scores the final cell of the supplied local path.  A
    full rectangular bypass path therefore keeps rewarding the distant mission
    goal and can prefer standing still over the initially lateral move.  This
    wrapper leaves the DWB core untouched and presents only the active segment.
    A fresh DWB instance is created when the segment goal changes so the source-
    derived latched goal contract is not bypassed.
    """

    name = "dynamic_dwb_reference_local_detour"

    def __init__(
        self,
        policy: DirectionalLocalDetourPolicy | None = None,
        *,
        dwb_config: SourceDerivedDwbConfig | None = None,
    ) -> None:
        self.policy = policy or DirectionalLocalDetourPolicy()
        self.dwb_config = dwb_config or SourceDerivedDwbConfig(
            goal_align_scale=0.0
        )
        self._controller = self._new_controller()
        self._active_waypoint_index: int | None = None

    @property
    def active_waypoint_index(self) -> int | None:
        return self._active_waypoint_index

    @property
    def active_local_goal(self) -> Pose2D | None:
        path = self.policy.active_reference_path
        if path is None or self._active_waypoint_index is None:
            return None
        return path[self._active_waypoint_index]

    def step(self, snapshot: ControllerSnapshot):
        transformed = self.policy.apply(snapshot)
        path = self.policy.active_reference_path
        if path is None:
            return self._controller.step(snapshot)

        if self._active_waypoint_index is None:
            self._active_waypoint_index = 1
            self._controller = self._new_controller()
        self._advance_waypoint(snapshot.robot_state.pose, path)
        target = path[self._active_waypoint_index]
        robot = snapshot.robot_state.pose
        segment_yaw = atan2(target.y - robot.y, target.x - robot.x)
        local_path = (
            Pose2D(robot.x, robot.y, segment_yaw),
            target,
        )
        controller_snapshot = build_controller_snapshot(
            tick_id=transformed.tick_id,
            simulation_time_s=transformed.simulation_time_s,
            mission_id=transformed.mission_id,
            robot_state=transformed.robot_state,
            goal_pose=target,
            reference_path=local_path,
            static_grid_snapshot=transformed.static_grid_snapshot,
            validated_observation=transformed.validated_observation,
            actor_tubes=transformed.actor_tubes,
            vehicle_profile=transformed.vehicle_profile,
        )
        return self._controller.step(controller_snapshot)

    def restore(
        self,
        path: tuple[Pose2D, ...],
        *,
        active_waypoint_index: int,
    ) -> None:
        if not isinstance(active_waypoint_index, int) or isinstance(
            active_waypoint_index, bool
        ):
            raise TypeError("active waypoint index must be an integer")
        if not 1 <= active_waypoint_index < len(path):
            raise ValueError("active waypoint index is outside the detour path")
        self.policy.restore_reference_path(path)
        self._active_waypoint_index = active_waypoint_index
        self._controller = self._new_controller()

    def _advance_waypoint(
        self,
        robot: Pose2D,
        path: tuple[Pose2D, ...],
    ) -> None:
        assert self._active_waypoint_index is not None
        changed = False
        while self._active_waypoint_index < len(path) - 1:
            target = path[self._active_waypoint_index]
            if hypot(target.x - robot.x, target.y - robot.y) > (
                self.policy.config.waypoint_tolerance_m
            ):
                break
            self._active_waypoint_index += 1
            changed = True
        if changed:
            self._controller = self._new_controller()

    def _new_controller(self) -> SourceDerivedDynamicDwbController:
        return SourceDerivedDynamicDwbController(config=self.dwb_config)


__all__ = [
    "DirectionalLocalDetourDwbController",
    "DirectionalLocalDetourPolicy",
    "LocalDetourPolicyConfig",
    "LocalDetourPolicyState",
]
