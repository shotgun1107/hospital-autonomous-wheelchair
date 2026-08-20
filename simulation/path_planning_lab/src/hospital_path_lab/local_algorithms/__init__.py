"""공통 local planner 계약을 따르는 연구용 알고리즘.

The source-derived DWB implementation has a considerably larger dependency
tree than the original grid/DWA planners.  Keep it lazy so importing the base
package does not also import R5-B temporal-evidence helpers.  Direct imports
and the historical public names remain supported through ``__getattr__``.
"""

from typing import TYPE_CHECKING

from hospital_path_lab.local_algorithms.dwa import DynamicDwaController, DynamicWindowPlanner
from hospital_path_lab.local_algorithms.grid_astar import BoundedGridAStarPlanner

if TYPE_CHECKING:
    from hospital_path_lab.local_algorithms.dwb_reference import (
        PersistentSourceDerivedDwbController,
        SourceDerivedDynamicDwbController,
    )

__all__ = [
    "BoundedGridAStarPlanner",
    "DynamicDwaController",
    "DynamicWindowPlanner",
    "PersistentSourceDerivedDwbController",
    "SourceDerivedDynamicDwbController",
]


def __getattr__(name: str):
    """Load optional source-derived DWB symbols only when a caller asks for them."""

    if name in {"PersistentSourceDerivedDwbController", "SourceDerivedDynamicDwbController"}:
        from hospital_path_lab.local_algorithms.dwb_reference import (
            PersistentSourceDerivedDwbController,
            SourceDerivedDynamicDwbController,
        )

        values = {
            "PersistentSourceDerivedDwbController": PersistentSourceDerivedDwbController,
            "SourceDerivedDynamicDwbController": SourceDerivedDynamicDwbController,
        }
        return values[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
