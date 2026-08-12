"""Phase A-1 contracts for the source-derived DWA/DWB reference generator.

This module is an independent, behavior-level reconstruction.  It is informed by
the following frozen upstream revisions; no upstream implementation text is copied:

* ROS 1 ``ros-planning/navigation``
  ``f44bb1fc2810399165115cc98b530fe4b9397c18``
* ROS 2 Nav2 ``ros-navigation/navigation2``
  ``1e8afb17e2e09df443b1870ce0f4ecdee32207fd``

The types are intentionally independent from the existing experiment controller so
that later adapters must cross an explicit contract boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

ROS1_NAVIGATION_COMMIT = "f44bb1fc2810399165115cc98b530fe4b9397c18"
NAV2_NAVIGATION_COMMIT = "1e8afb17e2e09df443b1870ce0f4ecdee32207fd"


def _require_finite(name: str, value: float) -> None:
    if not isfinite(value):
        raise ValueError(f"{name} must be finite")


def _require_positive(name: str, value: float) -> None:
    _require_finite(name, value)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")


@dataclass(frozen=True, slots=True)
class DwbPose2D:
    """Planar pose used only by the DWB reference implementation."""

    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        _require_finite("x_m", self.x_m)
        _require_finite("y_m", self.y_m)
        _require_finite("yaw_rad", self.yaw_rad)


@dataclass(frozen=True, slots=True)
class DwbTwist2D:
    """Differential-drive command; lateral velocity is deliberately absent."""

    linear_mps: float
    angular_radps: float

    def __post_init__(self) -> None:
        _require_finite("linear_mps", self.linear_mps)
        _require_finite("angular_radps", self.angular_radps)


@dataclass(frozen=True, slots=True)
class DwbGeneratorConfig:
    """Frozen generator parameters for the simulation-only reference lane."""

    control_period_s: float = 0.05
    rollout_duration_s: float = 2.0
    integration_step_s: float = 0.05
    maximum_forward_speed_mps: float = 0.20
    maximum_reverse_speed_mps: float = 0.10
    linear_acceleration_mps2: float = 0.25
    linear_deceleration_mps2: float = 0.50
    maximum_angular_speed_radps: float = 0.80
    angular_acceleration_radps2: float = 1.60
    angular_deceleration_radps2: float = 1.60
    linear_sample_count: int = 7
    angular_sample_count: int = 31
    allow_reverse: bool = False

    def __post_init__(self) -> None:
        for name in (
            "control_period_s",
            "rollout_duration_s",
            "integration_step_s",
            "maximum_forward_speed_mps",
            "linear_acceleration_mps2",
            "linear_deceleration_mps2",
            "maximum_angular_speed_radps",
            "angular_acceleration_radps2",
            "angular_deceleration_radps2",
        ):
            _require_positive(name, getattr(self, name))
        _require_finite("maximum_reverse_speed_mps", self.maximum_reverse_speed_mps)
        if self.maximum_reverse_speed_mps < 0.0:
            raise ValueError("maximum_reverse_speed_mps must be non-negative")
        if self.linear_sample_count < 2:
            raise ValueError("linear_sample_count must be at least 2")
        if self.angular_sample_count < 2:
            raise ValueError("angular_sample_count must be at least 2")

        step_count = self.rollout_duration_s / self.integration_step_s
        if abs(step_count - round(step_count)) > 1e-12:
            raise ValueError("rollout_duration_s must be divisible by integration_step_s")

    @property
    def rollout_step_count(self) -> int:
        """Number of integration intervals, excluding the initial pose."""

        return round(self.rollout_duration_s / self.integration_step_s)


@dataclass(frozen=True, slots=True)
class DwbGeneratorRequest:
    """State snapshot used to generate one deterministic trajectory batch."""

    pose: DwbPose2D
    current_twist: DwbTwist2D


@dataclass(frozen=True, slots=True)
class DwbTrajectory:
    """One constant-twist rollout, including the initial pose."""

    command: DwbTwist2D
    poses: tuple[DwbPose2D, ...]
    integration_step_s: float

    def __post_init__(self) -> None:
        _require_positive("integration_step_s", self.integration_step_s)
        if not self.poses:
            raise ValueError("poses must not be empty")


@dataclass(frozen=True, slots=True)
class DwbGeneratorResult:
    """Dynamic window, samples, and rollout batch produced for one request."""

    linear_window_mps: tuple[float, float]
    angular_window_radps: tuple[float, float]
    linear_samples_mps: tuple[float, ...]
    angular_samples_radps: tuple[float, ...]
    trajectories: tuple[DwbTrajectory, ...]

    @property
    def candidate_count(self) -> int:
        return len(self.trajectories)
