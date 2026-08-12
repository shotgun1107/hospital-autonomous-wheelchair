"""Limited-acceleration differential-drive trajectory generation.

The behavior is reconstructed from the public ROS 1
``SimpleTrajectoryGenerator`` and Nav2 DWB ``LimitedAccelGenerator`` /
``OneDVelocityIterator`` at the source commits recorded in ``contracts``.
It is a clean Python implementation for simulation research, not a copy or a ROS
plugin and not evidence of real wheelchair safety.
"""

from __future__ import annotations

from math import cos, sin

from .contracts import (
    DwbGeneratorConfig,
    DwbGeneratorRequest,
    DwbGeneratorResult,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)

_ZERO_TOLERANCE = 1e-12


class DwbReferenceTrajectoryGenerator:
    """Generate reachable constant-twist trajectories in stable sample order."""

    def __init__(self, config: DwbGeneratorConfig | None = None) -> None:
        self.config = config or DwbGeneratorConfig()

    def generate(self, request: DwbGeneratorRequest) -> DwbGeneratorResult:
        """Generate every ``linear x angular`` candidate for one control tick."""

        self._validate_request(request)
        linear_window, angular_window = self.dynamic_window(request.current_twist)
        linear_samples = sample_velocity_axis(
            linear_window[0],
            linear_window[1],
            self.config.linear_sample_count,
        )
        angular_samples = sample_velocity_axis(
            angular_window[0],
            angular_window[1],
            self.config.angular_sample_count,
        )
        trajectories = tuple(
            self.rollout(request.pose, DwbTwist2D(linear, angular))
            for linear in linear_samples
            for angular in angular_samples
        )
        return DwbGeneratorResult(
            linear_window_mps=linear_window,
            angular_window_radps=angular_window,
            linear_samples_mps=linear_samples,
            angular_samples_radps=angular_samples,
            trajectories=trajectories,
        )

    def dynamic_window(
        self,
        current_twist: DwbTwist2D,
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        """Return velocities reachable by the end of the next control period."""

        self._validate_twist(current_twist)
        period = self.config.control_period_s
        minimum_linear_speed = (
            -self.config.maximum_reverse_speed_mps if self.config.allow_reverse else 0.0
        )
        linear_minimum = max(
            minimum_linear_speed,
            current_twist.linear_mps - self.config.linear_deceleration_mps2 * period,
        )
        linear_maximum = min(
            self.config.maximum_forward_speed_mps,
            current_twist.linear_mps + self.config.linear_acceleration_mps2 * period,
        )
        angular_minimum = max(
            -self.config.maximum_angular_speed_radps,
            current_twist.angular_radps
            - self.config.angular_deceleration_radps2 * period,
        )
        angular_maximum = min(
            self.config.maximum_angular_speed_radps,
            current_twist.angular_radps
            + self.config.angular_acceleration_radps2 * period,
        )
        if linear_minimum > linear_maximum or angular_minimum > angular_maximum:
            raise ValueError("current twist cannot produce a non-empty dynamic window")
        return (linear_minimum, linear_maximum), (angular_minimum, angular_maximum)

    def rollout(self, initial_pose: DwbPose2D, command: DwbTwist2D) -> DwbTrajectory:
        """Integrate a constant differential-drive twist for the frozen horizon."""

        self._validate_command(command)
        poses = [initial_pose]
        pose = initial_pose
        dt = self.config.integration_step_s
        for _ in range(self.config.rollout_step_count):
            pose = DwbPose2D(
                x_m=pose.x_m + command.linear_mps * cos(pose.yaw_rad) * dt,
                y_m=pose.y_m + command.linear_mps * sin(pose.yaw_rad) * dt,
                yaw_rad=pose.yaw_rad + command.angular_radps * dt,
            )
            poses.append(pose)
        return DwbTrajectory(
            command=command,
            poses=tuple(poses),
            integration_step_s=dt,
        )

    def _validate_request(self, request: DwbGeneratorRequest) -> None:
        self._validate_twist(request.current_twist)

    def _validate_twist(self, twist: DwbTwist2D) -> None:
        minimum_linear_speed = (
            -self.config.maximum_reverse_speed_mps if self.config.allow_reverse else 0.0
        )
        if not minimum_linear_speed <= twist.linear_mps <= self.config.maximum_forward_speed_mps:
            raise ValueError("current linear velocity is outside the configured limits")
        if abs(twist.angular_radps) > self.config.maximum_angular_speed_radps:
            raise ValueError("current angular velocity is outside the configured limits")

    def _validate_command(self, command: DwbTwist2D) -> None:
        minimum_linear_speed = (
            -self.config.maximum_reverse_speed_mps if self.config.allow_reverse else 0.0
        )
        if not minimum_linear_speed <= command.linear_mps <= self.config.maximum_forward_speed_mps:
            raise ValueError("command linear velocity is outside the configured limits")
        if abs(command.angular_radps) > self.config.maximum_angular_speed_radps:
            raise ValueError("command angular velocity is outside the configured limits")


def sample_velocity_axis(minimum: float, maximum: float, sample_count: int) -> tuple[float, ...]:
    """Return ordered uniform samples and insert an in-range zero when absent.

    This intentionally follows the upstream iterator meaning: adding zero may make
    the returned tuple one element larger than ``sample_count``.
    """

    if sample_count < 1:
        raise ValueError("sample_count must be positive")
    if minimum > maximum:
        raise ValueError("minimum must not exceed maximum")
    if minimum == maximum:
        return (0.0 if abs(minimum) <= _ZERO_TOLERANCE else minimum,)
    if sample_count < 2:
        raise ValueError("sample_count must be at least 2 for a non-zero range")

    step = (maximum - minimum) / (sample_count - 1)
    values = [minimum + index * step for index in range(sample_count)]
    values[-1] = maximum
    if minimum <= 0.0 <= maximum and not any(
        abs(value) <= _ZERO_TOLERANCE for value in values
    ):
        values.append(0.0)
        values.sort()

    normalized: list[float] = []
    for value in values:
        value = 0.0 if abs(value) <= _ZERO_TOLERANCE else value
        if not normalized or abs(value - normalized[-1]) > _ZERO_TOLERANCE:
            normalized.append(value)
    return tuple(normalized)
