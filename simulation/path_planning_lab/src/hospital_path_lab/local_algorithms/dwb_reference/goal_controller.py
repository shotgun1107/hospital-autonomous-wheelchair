"""Latched stop-and-rotate goal handling for the DWB reference lane.

The state ordering follows the observable intent of ROS 1
``LatchedStopRotateController`` and Nav2 ``RotateToGoalCritic``: path tracking is
used until the position tolerance is latched, translation and rotation are then
stopped with bounded deceleration, and only an actually stopped chassis may begin
the shortest-angle in-place rotation.

This is a deterministic, simulation-only reconstruction.  It consumes only the
reference ``DwbPose2D`` and ``DwbTwist2D`` types and is not a product safety or
actuator controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from math import atan2, copysign, cos, hypot, isfinite, sin, sqrt

from .contracts import DwbPose2D, DwbTwist2D

_GEOMETRY_TOLERANCE_M = 1e-12


def _require_positive(name: str, value: float) -> None:
    if not isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True, slots=True)
class DwbGoalControllerConfig:
    """Simulation-only limits for the latched goal state machine."""

    control_period_s: float = 0.05
    xy_goal_tolerance_m: float = 0.05
    yaw_goal_tolerance_rad: float = 0.08
    stopped_linear_velocity_mps: float = 0.01
    stopped_angular_velocity_radps: float = 0.02
    linear_deceleration_mps2: float = 0.50
    angular_acceleration_radps2: float = 1.60
    angular_deceleration_radps2: float = 1.60
    maximum_angular_speed_radps: float = 0.80

    def __post_init__(self) -> None:
        for name in (
            "control_period_s",
            "xy_goal_tolerance_m",
            "yaw_goal_tolerance_rad",
            "stopped_linear_velocity_mps",
            "stopped_angular_velocity_radps",
            "linear_deceleration_mps2",
            "angular_acceleration_radps2",
            "angular_deceleration_radps2",
            "maximum_angular_speed_radps",
        ):
            _require_positive(name, getattr(self, name))


class DwbGoalControlState(StrEnum):
    """Mutually exclusive phases of the latched goal controller."""

    TRACK_PATH = "track_path"
    DECELERATE_TO_STOP = "decelerate_to_stop"
    ROTATE_TO_GOAL = "rotate_to_goal"
    ALIGNED_STOP = "aligned_stop"


@dataclass(frozen=True, slots=True)
class DwbGoalControlRequest:
    """One state-machine sample tied to a session and deterministic tick."""

    session_key: str
    tick: int
    pose: DwbPose2D
    actual_twist: DwbTwist2D
    goal_pose: DwbPose2D

    def __post_init__(self) -> None:
        if not self.session_key.strip():
            raise ValueError("session_key must not be blank")
        if isinstance(self.tick, bool) or not isinstance(self.tick, int) or self.tick < 0:
            raise ValueError("tick must be a non-negative integer")


@dataclass(frozen=True, slots=True)
class DwbGoalControlResult:
    """Goal-controller decision for a single tick."""

    state: DwbGoalControlState
    command: DwbTwist2D | None
    xy_tolerance_latched: bool
    goal_complete: bool
    position_error_m: float
    yaw_error_rad: float

    @property
    def overrides_path_tracking(self) -> bool:
        """Whether this result replaces the normal DWB path-tracking command."""

        return self.command is not None


class DwbLatchedGoalController:
    """Deterministic stop-then-rotate goal state machine."""

    def __init__(self, config: DwbGoalControllerConfig | None = None) -> None:
        self.config = config or DwbGoalControllerConfig()
        self._session_key: str | None = None
        self._goal_pose: DwbPose2D | None = None
        self._state = DwbGoalControlState.TRACK_PATH
        self._xy_tolerance_latched = False
        self._last_request: DwbGoalControlRequest | None = None
        self._last_result: DwbGoalControlResult | None = None

    @property
    def state(self) -> DwbGoalControlState:
        return self._state

    @property
    def xy_tolerance_latched(self) -> bool:
        return self._xy_tolerance_latched

    def reset(self, session_key: str | None = None) -> None:
        """Clear all latched state, optionally reserving a new session key."""

        if session_key is not None and not session_key.strip():
            raise ValueError("session_key must not be blank")
        self._session_key = session_key
        self._goal_pose = None
        self._state = DwbGoalControlState.TRACK_PATH
        self._xy_tolerance_latched = False
        self._last_request = None
        self._last_result = None

    def update(self, request: DwbGoalControlRequest) -> DwbGoalControlResult:
        """Advance at most once per tick and return the goal override decision."""

        self._start_or_validate_session(request)
        if self._last_request is not None and request.tick == self._last_request.tick:
            if request != self._last_request:
                raise ValueError("the same session tick was reused with different input")
            if self._last_result is None:  # pragma: no cover - internal invariant
                raise RuntimeError("cached request has no cached result")
            return self._last_result
        if self._last_request is not None and request.tick < self._last_request.tick:
            raise ValueError("tick must increase monotonically within a session")

        result = self._advance(request)
        self._last_request = request
        self._last_result = result
        return result

    def _start_or_validate_session(self, request: DwbGoalControlRequest) -> None:
        if request.session_key != self._session_key:
            self.reset(request.session_key)
        if self._goal_pose is None:
            self._goal_pose = request.goal_pose
        elif request.goal_pose != self._goal_pose:
            raise ValueError("goal_pose changed without a new session_key or reset")

    def _advance(self, request: DwbGoalControlRequest) -> DwbGoalControlResult:
        position_error = hypot(
            request.goal_pose.x_m - request.pose.x_m,
            request.goal_pose.y_m - request.pose.y_m,
        )
        yaw_error = shortest_angular_distance(request.pose.yaw_rad, request.goal_pose.yaw_rad)

        if self._state is DwbGoalControlState.ALIGNED_STOP:
            return self._result(
                DwbGoalControlState.ALIGNED_STOP,
                DwbTwist2D(0.0, 0.0),
                True,
                position_error,
                yaw_error,
            )

        if not self._xy_tolerance_latched:
            if position_error > self.config.xy_goal_tolerance_m + _GEOMETRY_TOLERANCE_M:
                self._state = DwbGoalControlState.TRACK_PATH
                return self._result(
                    self._state,
                    None,
                    False,
                    position_error,
                    yaw_error,
                )
            self._xy_tolerance_latched = True
            self._state = DwbGoalControlState.DECELERATE_TO_STOP

        if self._state is DwbGoalControlState.DECELERATE_TO_STOP:
            if not self._is_actually_stopped(request.actual_twist):
                return self._result(
                    self._state,
                    self._bounded_stop_command(request.actual_twist),
                    False,
                    position_error,
                    yaw_error,
                )
            if abs(yaw_error) <= self.config.yaw_goal_tolerance_rad:
                self._state = DwbGoalControlState.ALIGNED_STOP
                return self._result(
                    self._state,
                    DwbTwist2D(0.0, 0.0),
                    True,
                    position_error,
                    yaw_error,
                )
            self._state = DwbGoalControlState.ROTATE_TO_GOAL

        if self._state is DwbGoalControlState.ROTATE_TO_GOAL:
            # Translation must be observed stopped before any rotation command.
            if not self._is_linear_stopped(request.actual_twist):
                self._state = DwbGoalControlState.DECELERATE_TO_STOP
                return self._result(
                    self._state,
                    self._bounded_stop_command(request.actual_twist),
                    False,
                    position_error,
                    yaw_error,
                )
            if abs(yaw_error) <= self.config.yaw_goal_tolerance_rad:
                if self._is_actually_stopped(request.actual_twist):
                    self._state = DwbGoalControlState.ALIGNED_STOP
                    return self._result(
                        self._state,
                        DwbTwist2D(0.0, 0.0),
                        True,
                        position_error,
                        yaw_error,
                    )
                return self._result(
                    self._state,
                    self._bounded_stop_command(request.actual_twist),
                    False,
                    position_error,
                    yaw_error,
                )

            return self._result(
                self._state,
                DwbTwist2D(
                    0.0,
                    self._bounded_rotation_command(
                        request.actual_twist.angular_radps,
                        yaw_error,
                    ),
                ),
                False,
                position_error,
                yaw_error,
            )

        raise RuntimeError(f"unhandled goal-control state: {self._state}")

    def _bounded_stop_command(self, actual_twist: DwbTwist2D) -> DwbTwist2D:
        return DwbTwist2D(
            _approach_zero(
                actual_twist.linear_mps,
                self.config.linear_deceleration_mps2 * self.config.control_period_s,
            ),
            _approach_zero(
                actual_twist.angular_radps,
                self.config.angular_deceleration_radps2 * self.config.control_period_s,
            ),
        )

    def _bounded_rotation_command(self, actual_angular: float, yaw_error: float) -> float:
        distance_to_tolerance = max(
            0.0,
            abs(yaw_error) - self.config.yaw_goal_tolerance_rad,
        )
        stopping_limited_speed = sqrt(
            2.0 * self.config.angular_deceleration_radps2 * distance_to_tolerance
        )
        target_magnitude = min(
            self.config.maximum_angular_speed_radps,
            stopping_limited_speed,
        )
        target = copysign(target_magnitude, yaw_error)
        return _approach_velocity(
            actual_angular,
            target,
            acceleration_step=(
                self.config.angular_acceleration_radps2 * self.config.control_period_s
            ),
            deceleration_step=(
                self.config.angular_deceleration_radps2 * self.config.control_period_s
            ),
        )

    def _is_linear_stopped(self, twist: DwbTwist2D) -> bool:
        return abs(twist.linear_mps) <= self.config.stopped_linear_velocity_mps

    def _is_actually_stopped(self, twist: DwbTwist2D) -> bool:
        return self._is_linear_stopped(twist) and (
            abs(twist.angular_radps) <= self.config.stopped_angular_velocity_radps
        )

    def _result(
        self,
        state: DwbGoalControlState,
        command: DwbTwist2D | None,
        complete: bool,
        position_error: float,
        yaw_error: float,
    ) -> DwbGoalControlResult:
        return DwbGoalControlResult(
            state=state,
            command=command,
            xy_tolerance_latched=self._xy_tolerance_latched,
            goal_complete=complete,
            position_error_m=position_error,
            yaw_error_rad=yaw_error,
        )


def shortest_angular_distance(current_yaw_rad: float, target_yaw_rad: float) -> float:
    """Return the deterministic signed shortest rotation from current to target."""

    difference = target_yaw_rad - current_yaw_rad
    return atan2(sin(difference), cos(difference))


def _approach_zero(value: float, maximum_change: float) -> float:
    if abs(value) <= maximum_change:
        return 0.0
    return value - copysign(maximum_change, value)


def _approach_velocity(
    current: float,
    target: float,
    *,
    acceleration_step: float,
    deceleration_step: float,
) -> float:
    """Move toward a target without reversing before the current turn is stopped."""

    if current * target < 0.0:
        return _approach_zero(current, deceleration_step)
    maximum_change = deceleration_step if abs(target) < abs(current) else acceleration_step
    difference = target - current
    if abs(difference) <= maximum_change:
        return target
    return current + copysign(maximum_change, difference)
