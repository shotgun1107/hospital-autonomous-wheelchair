"""Source-derived DWA/DWB reference components for simulation research."""

from .adapter import SourceDerivedDwbController
from .composition import SourceDerivedDwbConfig, SourceDerivedDynamicDwbController
from .contracts import (
    NAV2_NAVIGATION_COMMIT,
    ROS1_NAVIGATION_COMMIT,
    DwbGeneratorConfig,
    DwbGeneratorRequest,
    DwbGeneratorResult,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)
from .core import DwbCriticBinding, DwbReferenceCore, IllegalTrajectoryError
from .goal_controller import DwbLatchedGoalController
from .trajectory_generator import DwbReferenceTrajectoryGenerator, sample_velocity_axis

__all__ = [
    "NAV2_NAVIGATION_COMMIT",
    "ROS1_NAVIGATION_COMMIT",
    "DwbCriticBinding",
    "DwbGeneratorConfig",
    "DwbGeneratorRequest",
    "DwbGeneratorResult",
    "DwbPose2D",
    "DwbReferenceTrajectoryGenerator",
    "DwbReferenceCore",
    "DwbTrajectory",
    "DwbTwist2D",
    "DwbLatchedGoalController",
    "IllegalTrajectoryError",
    "SourceDerivedDwbConfig",
    "SourceDerivedDwbController",
    "SourceDerivedDynamicDwbController",
    "sample_velocity_axis",
]
