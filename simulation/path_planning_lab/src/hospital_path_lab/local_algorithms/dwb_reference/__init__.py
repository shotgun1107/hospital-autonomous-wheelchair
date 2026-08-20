"""Source-derived DWA/DWB reference components for simulation research.

The native safety wrapper imports the lightweight DWB contracts directly. Do
not eagerly import the composed controller here: doing so pulls the safety
wrapper back in while it is still initialising. Historical public names stay
available through :func:`__getattr__`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
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
    from .cpp_full_core import CPP_DWB_FULL_CORE_AVAILABLE, CppDwbReferenceCore
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
    "CriticBatchScore",
    "CPP_DWB_FULL_CORE_AVAILABLE",
    "CppDwbReferenceCore",
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
    "PersistentDwbCoreSession",
    "PersistentDwbSessionDiagnostics",
    "PersistentSourceDerivedDwbController",
    "SourceDerivedDwbConfig",
    "SourceDerivedDwbController",
    "SourceDerivedDynamicDwbController",
    "sample_velocity_axis",
]


def __getattr__(name: str):
    """Load a historical public symbol only when that symbol is requested."""

    module_by_name = {
        "SourceDerivedDwbController": ".adapter",
        "SourceDerivedDwbConfig": ".composition",
        "SourceDerivedDynamicDwbController": ".composition",
        "NAV2_NAVIGATION_COMMIT": ".contracts",
        "ROS1_NAVIGATION_COMMIT": ".contracts",
        "DwbGeneratorConfig": ".contracts",
        "DwbGeneratorRequest": ".contracts",
        "DwbGeneratorResult": ".contracts",
        "DwbPose2D": ".contracts",
        "DwbTrajectory": ".contracts",
        "DwbTwist2D": ".contracts",
        "CriticBatchScore": ".core",
        "DwbCriticBinding": ".core",
        "DwbReferenceCore": ".core",
        "IllegalTrajectoryError": ".core",
        "CPP_DWB_FULL_CORE_AVAILABLE": ".cpp_full_core",
        "CppDwbReferenceCore": ".cpp_full_core",
        "DwbLatchedGoalController": ".goal_controller",
        "PERSISTENT_DWB_ADAPTER_VERSION": ".persistent_adapter",
        "PERSISTENT_DWB_CONTROLLER_NAME": ".persistent_adapter",
        "PersistentDwbCoreSession": ".persistent_adapter",
        "PersistentDwbSessionDiagnostics": ".persistent_adapter",
        "PersistentSourceDerivedDwbController": ".persistent_adapter",
        "DwbReferenceTrajectoryGenerator": ".trajectory_generator",
        "sample_velocity_axis": ".trajectory_generator",
    }
    module_name = module_by_name.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module = __import__(f"{__name__}{module_name}", fromlist=[name])
    return getattr(module, name)
