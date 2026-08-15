"""Runnable source-derived DWB controller for the dynamic research lane.

This module composes the independently tested upstream-style generator, core,
critics, and latched goal handler with the project's existing dynamic safety
contract.  It deliberately does not import corpus labels, evaluator oracles,
runner splits, or hidden-test data.

The project boundary remains a Python ``simulation_only`` reference and uses
the optional C++ numerical core when available.  It is neither a Nav2 plugin
nor evidence of product or human-rider safety.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields, is_dataclass, replace
from enum import Enum
from hashlib import sha256
from json import dumps
from math import cos, isfinite, pi, sin
from time import perf_counter_ns

import numpy as np

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import PlanStatus, Pose2D, TrajectoryPoint, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_COMMAND_APPLY_LATENCY_S,
    ControllerCommandResult,
    ControllerSnapshot,
    controller_snapshot_content_hash,
)
from hospital_path_lab.dynamic_trajectory_constraints import (
    ProjectDynamicSafetyConstraintCritic,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile

from .adapter import SourceDerivedDwbController
from .contracts import (
    NAV2_NAVIGATION_COMMIT,
    ROS1_NAVIGATION_COMMIT,
    DwbGeneratorConfig,
    DwbGeneratorRequest,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)
from .core import DwbCriticBinding, DwbReferenceCore, IllegalTrajectoryError
from .cpp_full_core import CppDwbReferenceCore
from .critics import (
    DwbCriticGrid,
    GoalAlignCritic,
    GoalDistCritic,
    OscillationCritic,
    PathAlignCritic,
    PathDistCritic,
    RotateToGoalCritic,
)
from .goal_controller import (
    DwbGoalControlRequest,
    DwbGoalControlState,
    DwbLatchedGoalController,
)
from .trajectory_generator import DwbReferenceTrajectoryGenerator

_EXPECTED_GOAL_CONTRACT_ERRORS = frozenset(
    {
        "the same session tick was reused with different input",
        "tick must increase monotonically within a session",
        "goal_pose changed without a new session_key or reset",
    }
)


@dataclass(frozen=True, slots=True)
class SourceDerivedDwbConfig:
    """Explicit v8 critic order, scales, and generator configuration."""

    generator: DwbGeneratorConfig = field(
        default_factory=lambda: DwbGeneratorConfig(maximum_forward_speed_mps=0.30)
    )
    safety_scale: float = 1.0
    rotate_to_goal_scale: float = 32.0
    oscillation_scale: float = 1.0
    goal_align_scale: float = 24.0
    path_align_scale: float = 32.0
    path_dist_scale: float = 32.0
    goal_dist_scale: float = 24.0
    forward_point_distance_m: float = 0.325

    def __post_init__(self) -> None:
        if not isfinite(self.safety_scale) or self.safety_scale <= 0.0:
            raise ValueError("project safety scale must be finite and positive")
        values = (
            self.rotate_to_goal_scale,
            self.oscillation_scale,
            self.goal_align_scale,
            self.path_align_scale,
            self.path_dist_scale,
            self.goal_dist_scale,
            self.forward_point_distance_m,
        )
        if any(not isfinite(value) or value < 0.0 for value in values):
            raise ValueError("DWB scales and distances must be finite and non-negative")


@dataclass(slots=True)
class _ControllerStack:
    geometry_signature: str
    adapter: SourceDerivedDwbController
    safety_critic: ProjectDynamicSafetyConstraintCritic
    generator: DwbReferenceTrajectoryGenerator


class SourceDerivedDynamicDwbController:
    """Source-derived DWB controller with optional full C++ numerical core."""

    name = "dynamic_dwb_reference"

    def __init__(
        self,
        vehicle_profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        *,
        config: SourceDerivedDwbConfig | None = None,
        use_cpp_full_core: bool = True,
    ) -> None:
        if not vehicle_profile.simulation_only:
            raise ValueError("source-derived DWB requires a simulation-only profile")
        if not isinstance(use_cpp_full_core, bool):
            raise TypeError("use_cpp_full_core must be bool")
        self.vehicle_profile = vehicle_profile
        self.config = config or SourceDerivedDwbConfig(
            generator=_generator_config_for(vehicle_profile)
        )
        self._use_cpp_full_core = use_cpp_full_core
        _validate_generator_profile(self.config.generator, vehicle_profile)
        self._goal_controller = DwbLatchedGoalController()
        self._stack: _ControllerStack | None = None
        self._stack_build_count = 0
        self._last_snapshot_identity: tuple[int, str] | None = None
        self._last_result: ControllerCommandResult | None = None

    @property
    def stack_build_count(self) -> int:
        return self._stack_build_count

    @property
    def selected_safety_evidence(self):
        stack = self._stack
        return None if stack is None else stack.safety_critic.selected_evidence

    @property
    def native_full_core_used(self) -> bool:
        stack = self._stack
        if stack is None:
            return False
        return bool(getattr(stack.adapter.core, "native_used", False))

    def step(self, snapshot: ControllerSnapshot) -> ControllerCommandResult:
        if not isinstance(snapshot, ControllerSnapshot):
            raise TypeError("source-derived DWB input must be a ControllerSnapshot")
        identity = snapshot.tick_id, _semantic_snapshot_digest(snapshot)
        if identity == self._last_snapshot_identity and self._last_result is not None:
            return self._last_result
        if (
            self._last_snapshot_identity is not None
            and snapshot.tick_id == self._last_snapshot_identity[0]
            and identity != self._last_snapshot_identity
        ):
            result = _fail_closed_result(snapshot, "same_tick_input_changed")
            self._remember(identity, result)
            return result

        geometry_failure = _stack_geometry_failure(snapshot)
        if geometry_failure is not None:
            result = _fail_closed_result(snapshot, geometry_failure)
            self._remember(identity, result)
            return result

        stack = self._ensure_stack(snapshot)
        preparation_failure = stack.adapter.prepare_snapshot(snapshot)
        if preparation_failure is not None:
            self._remember(identity, preparation_failure)
            return preparation_failure

        try:
            # The goal controller documents ValueError as its caller-contract
            # signal (for example, a regressed tick in one session).  Only that
            # narrow boundary is converted to a zero command.  Errors from stack
            # construction, the adapter, critics, or other internal code remain
            # visible so programming defects are not disguised as safe input.
            goal = self._goal_controller.update(
                DwbGoalControlRequest(
                    session_key=_goal_session_key(snapshot),
                    tick=snapshot.tick_id,
                    pose=_dwb_pose(snapshot.robot_state.pose),
                    actual_twist=DwbTwist2D(
                        snapshot.robot_state.twist.linear,
                        snapshot.robot_state.twist.angular,
                    ),
                    goal_pose=_dwb_pose(snapshot.goal_pose),
                )
            )
        except ValueError as error:
            if str(error) not in _EXPECTED_GOAL_CONTRACT_ERRORS:
                raise
            result = _fail_closed_result(
                snapshot,
                "goal_controller_contract_violation",
            )
            self._remember(identity, result)
            return result
        if goal.state is DwbGoalControlState.TRACK_PATH:
            result = stack.adapter.compute_prepared(snapshot)
        else:
            result = self._goal_override_result(snapshot, stack, goal)
        self._remember(identity, result)
        return result

    def _ensure_stack(self, snapshot: ControllerSnapshot) -> _ControllerStack:
        signature = _static_geometry_signature(snapshot, self.vehicle_profile)
        if self._stack is not None and self._stack.geometry_signature == signature:
            return self._stack

        grid = _critic_grid(snapshot, self.vehicle_profile)
        safety = ProjectDynamicSafetyConstraintCritic()
        rotate = RotateToGoalCritic(
            xy_goal_tolerance_m=0.05,
            path_length_tolerance_m=0.10,
            stopped_linear_velocity_mps=0.01,
        )
        oscillation = OscillationCritic()
        goal_align = GoalAlignCritic(
            grid,
            forward_point_distance_m=self.config.forward_point_distance_m,
        )
        path_align = PathAlignCritic(
            grid,
            forward_point_distance_m=self.config.forward_point_distance_m,
        )
        path_dist = PathDistCritic(grid)
        goal_dist = GoalDistCritic(grid)
        bindings = (
            DwbCriticBinding("project_safety", safety, self.config.safety_scale),
            DwbCriticBinding("rotate_to_goal", rotate, self.config.rotate_to_goal_scale),
            DwbCriticBinding("oscillation", oscillation, self.config.oscillation_scale),
            DwbCriticBinding(
                "goal_align",
                goal_align,
                _map_grid_scale(self.config.goal_align_scale, grid.resolution_m),
            ),
            DwbCriticBinding(
                "path_align",
                path_align,
                _map_grid_scale(self.config.path_align_scale, grid.resolution_m),
            ),
            DwbCriticBinding(
                "path_dist",
                path_dist,
                _map_grid_scale(self.config.path_dist_scale, grid.resolution_m),
            ),
            DwbCriticBinding(
                "goal_dist",
                goal_dist,
                _map_grid_scale(self.config.goal_dist_scale, grid.resolution_m),
            ),
        )
        generator = DwbReferenceTrajectoryGenerator(self.config.generator)
        core = (
            CppDwbReferenceCore(generator, bindings)
            if self._use_cpp_full_core
            else DwbReferenceCore(generator, bindings)
        )
        adapter = SourceDerivedDwbController(
            self.vehicle_profile,
            core=core,
            snapshot_binders=(safety,),
            generator_config=self.config.generator,
        )
        self._stack = _ControllerStack(signature, adapter, safety, generator)
        self._stack_build_count += 1
        self._goal_controller.reset()
        return self._stack

    def _goal_override_result(
        self,
        snapshot: ControllerSnapshot,
        stack: _ControllerStack,
        goal,
    ) -> ControllerCommandResult:
        started_at = perf_counter_ns()
        if goal.command is None:  # pragma: no cover - state contract invariant
            raise RuntimeError("goal override state did not provide a command")
        request = _generator_request(snapshot)
        trajectory = stack.generator.rollout(request.pose, goal.command)
        prepared = stack.safety_critic.prepare(request)
        if prepared is False:
            return _fail_closed_result(snapshot, "goal_safety_preparation_failed")
        try:
            stack.safety_critic.score(trajectory)
            stack.safety_critic.debrief(goal.command)
        except IllegalTrajectoryError as error:
            return _fail_closed_result(
                snapshot,
                f"goal_override_unsafe:{error.reason_code}",
            )
        evidence = stack.safety_critic.selected_evidence
        trace = (
            f"upstream_ros1_commit={ROS1_NAVIGATION_COMMIT}",
            f"upstream_nav2_commit={NAV2_NAVIGATION_COMMIT}",
            f"goal_state={goal.state.value}",
            f"goal_complete={str(goal.goal_complete).lower()}",
            f"goal_position_error_m={goal.position_error_m.hex()}",
            f"goal_yaw_error_rad={goal.yaw_error_rad.hex()}",
            (
                "goal_minimum_static_clearance_m="
                + (
                    "none"
                    if evidence is None or evidence.minimum_static_clearance_m is None
                    else evidence.minimum_static_clearance_m.hex()
                )
            ),
            (
                "goal_minimum_actor_clearance_m="
                + (
                    "none"
                    if evidence is None or evidence.minimum_actor_clearance_m is None
                    else evidence.minimum_actor_clearance_m.hex()
                )
            ),
        )
        return ControllerCommandResult(
            controller_name=self.name,
            source_tick_id=snapshot.tick_id,
            status=PlanStatus.FOUND,
            requested_twist=Twist2D(
                goal.command.linear_mps,
                goal.command.angular_radps,
            ),
            predicted_trajectory=_project_trajectory(trajectory),
            failure_reason=None,
            decision_trace=trace,
            mission_id=snapshot.mission_id,
            map_id=snapshot.map_id,
            map_revision=snapshot.map_revision,
            mission_revision=snapshot.mission_revision,
            observation_revision=snapshot.observation_revision,
            grid_content_hash=snapshot.static_grid_snapshot.metadata.content_hash,
            observation_content_hash=snapshot.observation_content_hash,
            input_content_hash=snapshot.input_content_hash,
            elapsed_ns=perf_counter_ns() - started_at,
            controller_requested_stop=False,
            no_safe_candidate=False,
        )

    def _remember(
        self,
        identity: tuple[int, str],
        result: ControllerCommandResult,
    ) -> None:
        self._last_snapshot_identity = identity
        self._last_result = result


def _critic_grid(
    snapshot: ControllerSnapshot,
    profile: VehicleProfile,
) -> DwbCriticGrid:
    checker = CollisionChecker(
        snapshot.static_grid_snapshot.grid,
        profile,
        forbidden_cells=snapshot.static_grid_snapshot.forbidden_cells,
    )
    configuration = checker.configuration_grid
    ys, xs = np.nonzero(configuration.occupancy)
    blocked = frozenset((int(x), int(y)) for y, x in zip(ys, xs, strict=True))
    return DwbCriticGrid(
        width=configuration.width,
        height=configuration.height,
        resolution_m=configuration.resolution_m,
        origin_x_m=configuration.origin_x_m,
        origin_y_m=configuration.origin_y_m,
        blocked_cells=blocked,
    )


def _stack_geometry_failure(snapshot: ControllerSnapshot) -> str | None:
    """Reject malformed geometry before hashing/building a reusable stack.

    The project adapter performs the complete provenance and observation
    validation after stack composition.  Geometry is the exceptional dependency
    needed *to create* that stack, so its narrow fail-closed preflight must run
    first.  This prevents non-finite resolution/origin values from reaching
    ``CollisionChecker`` or the geometry signature builder.
    """

    grid_snapshot = snapshot.static_grid_snapshot
    if not grid_snapshot.input_valid:
        return "grid_snapshot_invalid"
    grid = grid_snapshot.grid
    if not all(
        isfinite(value)
        for value in (grid.resolution_m, grid.origin_x_m, grid.origin_y_m)
    ) or grid.resolution_m <= 0.0:
        return "grid_geometry_invalid"
    occupancy = grid.occupancy
    if occupancy.ndim != 2 or not occupancy.size:
        return "grid_geometry_invalid"
    return None


def _static_geometry_signature(
    snapshot: ControllerSnapshot,
    profile: VehicleProfile,
) -> str:
    grid_snapshot = snapshot.static_grid_snapshot
    grid = grid_snapshot.grid
    payload = {
        "map_id": snapshot.map_id,
        "map_revision": snapshot.map_revision,
        "shape": grid.occupancy.shape,
        "resolution_m": grid.resolution_m.hex(),
        "origin": (grid.origin_x_m.hex(), grid.origin_y_m.hex()),
        "occupancy_sha256": sha256(grid.occupancy.tobytes()).hexdigest(),
        "forbidden_cells": tuple(sorted(grid_snapshot.forbidden_cells)),
        "vehicle_profile_id": profile.profile_id,
    }
    return sha256(
        dumps(payload, sort_keys=True, separators=(",", ":")).encode("ascii")
    ).hexdigest()


def _semantic_snapshot_digest(snapshot: ControllerSnapshot) -> str:
    """Hash every immutable controller input, not only provenance metadata.

    ``ControllerSnapshot.input_content_hash`` intentionally covers provenance,
    but it does not cover robot state, path geometry, goal pose, Actor tubes, or
    the grid bytes.  Same-tick command caching must include those values too.
    """

    payload = _semantic_digest_payload(snapshot)
    serialized = dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    return sha256(serialized.encode("utf-8")).hexdigest()


def _semantic_digest_payload(value):
    """Return a deterministic, type-tagged JSON value for snapshot contracts."""

    if isinstance(value, Enum):
        return {
            "__enum__": f"{type(value).__module__}.{type(value).__qualname__}",
            "value": _semantic_digest_payload(value.value),
        }
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return {"__float__": value.hex()}
    if isinstance(value, np.ndarray):
        contiguous = np.ascontiguousarray(value)
        return {
            "__ndarray__": True,
            "dtype": contiguous.dtype.str,
            "shape": list(contiguous.shape),
            "sha256": sha256(contiguous.tobytes()).hexdigest(),
        }
    if is_dataclass(value) and not isinstance(value, type):
        return {
            "__dataclass__": f"{type(value).__module__}.{type(value).__qualname__}",
            "fields": {
                item.name: _semantic_digest_payload(getattr(value, item.name))
                for item in fields(value)
            },
        }
    if isinstance(value, (tuple, list)):
        return {
            "__sequence__": type(value).__name__,
            "items": [_semantic_digest_payload(item) for item in value],
        }
    if isinstance(value, (set, frozenset)):
        items = [_semantic_digest_payload(item) for item in value]
        items.sort(
            key=lambda item: dumps(
                item,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
        )
        return {"__set__": type(value).__name__, "items": items}
    raise TypeError(f"unsupported snapshot digest value: {type(value).__qualname__}")


def _goal_session_key(snapshot: ControllerSnapshot) -> str:
    goal = snapshot.goal_pose
    payload = (
        snapshot.mission_id,
        snapshot.mission_revision,
        snapshot.map_id,
        snapshot.map_revision,
        goal.x.hex(),
        goal.y.hex(),
        goal.yaw.hex(),
    )
    return sha256(repr(payload).encode("ascii")).hexdigest()


def _generator_request(snapshot: ControllerSnapshot) -> DwbGeneratorRequest:
    pose = snapshot.robot_state.pose
    twist = snapshot.robot_state.twist
    duration = DYNAMIC_COMMAND_APPLY_LATENCY_S
    if abs(twist.angular) <= 1e-12:
        post_apply = Pose2D(
            pose.x + twist.linear * cos(pose.yaw) * duration,
            pose.y + twist.linear * sin(pose.yaw) * duration,
            pose.yaw,
        )
    else:
        next_yaw = pose.yaw + twist.angular * duration
        radius = twist.linear / twist.angular
        post_apply = Pose2D(
            pose.x + radius * (sin(next_yaw) - sin(pose.yaw)),
            pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
            _normalize_angle(next_yaw),
        )
    return DwbGeneratorRequest(
        pose=_dwb_pose(post_apply),
        current_twist=DwbTwist2D(twist.linear, twist.angular),
    )


def _project_trajectory(trajectory: DwbTrajectory) -> tuple[TrajectoryPoint, ...]:
    twist = Twist2D(
        trajectory.command.linear_mps,
        trajectory.command.angular_radps,
    )
    return tuple(
        TrajectoryPoint(
            index * trajectory.integration_step_s,
            Pose2D(pose.x_m, pose.y_m, pose.yaw_rad),
            twist,
        )
        for index, pose in enumerate(trajectory.poses)
    )


def _dwb_pose(pose: Pose2D) -> DwbPose2D:
    return DwbPose2D(pose.x, pose.y, pose.yaw)


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def _fail_closed_result(
    snapshot: ControllerSnapshot,
    reason: str,
) -> ControllerCommandResult:
    metadata = snapshot.static_grid_snapshot.metadata
    result_input_hash = controller_snapshot_content_hash(
        tick_id=snapshot.tick_id,
        mission_id=snapshot.mission_id,
        map_id=snapshot.map_id,
        map_revision=snapshot.map_revision,
        mission_revision=snapshot.mission_revision,
        observation_revision=snapshot.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=snapshot.observation_content_hash,
    )
    return ControllerCommandResult(
        controller_name=SourceDerivedDynamicDwbController.name,
        source_tick_id=snapshot.tick_id,
        status=PlanStatus.NO_PATH,
        requested_twist=Twist2D(),
        predicted_trajectory=(),
        failure_reason=reason,
        decision_trace=(f"composition_failure={reason}",),
        mission_id=snapshot.mission_id,
        map_id=snapshot.map_id,
        map_revision=snapshot.map_revision,
        mission_revision=snapshot.mission_revision,
        observation_revision=snapshot.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=snapshot.observation_content_hash,
        input_content_hash=result_input_hash,
        elapsed_ns=0,
        controller_requested_stop=True,
        no_safe_candidate=True,
    )


def _generator_config_for(profile: VehicleProfile) -> DwbGeneratorConfig:
    return DwbGeneratorConfig(
        control_period_s=profile.control_period_s,
        rollout_duration_s=2.0,
        integration_step_s=0.05,
        maximum_forward_speed_mps=profile.max_forward_speed_mps,
        maximum_reverse_speed_mps=profile.max_reverse_speed_mps,
        linear_acceleration_mps2=profile.max_acceleration_mps2,
        linear_deceleration_mps2=profile.max_deceleration_mps2,
        maximum_angular_speed_radps=profile.max_angular_speed_radps,
        angular_acceleration_radps2=1.60,
        angular_deceleration_radps2=1.60,
        linear_sample_count=7,
        angular_sample_count=31,
        allow_reverse=False,
    )


def _map_grid_scale(configured_scale: float, resolution_m: float) -> float:
    """Apply Nav2 MapGridCritic's frozen resolution-aware scale convention."""

    return configured_scale * resolution_m * 0.5


def _validate_generator_profile(
    config: DwbGeneratorConfig,
    profile: VehicleProfile,
    *,
    allow_reverse: bool = False,
) -> None:
    expected = replace(_generator_config_for(profile), allow_reverse=allow_reverse)
    if config != expected:
        raise ValueError("composition generator must match every frozen v8 parameter")


__all__ = [
    "SourceDerivedDynamicDwbController",
    "SourceDerivedDwbConfig",
]
