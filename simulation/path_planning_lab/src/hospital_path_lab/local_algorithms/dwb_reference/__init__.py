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
from .core import (
    CriticBatchScore,
    DwbCriticBinding,
    DwbReferenceCore,
    IllegalTrajectoryError,
)
from .goal_controller import DwbLatchedGoalController
from .persistent_adapter import (
    PERSISTENT_DWB_ADAPTER_VERSION,
    PERSISTENT_DWB_CONTROLLER_NAME,
    PersistentDwbCoreSession,
    PersistentDwbSessionDiagnostics,
    PersistentSourceDerivedDwbController,
)
from .trajectory_generator import DwbReferenceTrajectoryGenerator, sample_velocity_axis

__all__ = [
    "NAV2_NAVIGATION_COMMIT",
    "PERSISTENT_DWB_ADAPTER_VERSION",
    "PERSISTENT_DWB_CONTROLLER_NAME",
    "ROS1_NAVIGATION_COMMIT",
    "DwbCriticBinding",
    "CriticBatchScore",
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
    "PersistentDwbCoreSession",
    "PersistentDwbSessionDiagnostics",
    "PersistentSourceDerivedDwbController",
    "SourceDerivedDwbConfig",
    "SourceDerivedDwbController",
    "SourceDerivedDynamicDwbController",
    "sample_velocity_axis",
]
