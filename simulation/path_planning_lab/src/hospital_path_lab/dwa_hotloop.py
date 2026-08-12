"""Optional Cython dispatch for the DWA simulation hot loop.

The import is intentionally optional: source-only installs keep the frozen
Python implementation, while an in-place extension build activates the same
API without changing controller policy.
"""

from __future__ import annotations

from os import environ
from typing import Any

_DISABLED = environ.get("HOSPITAL_PATH_LAB_DISABLE_CYTHON_DWA") == "1"

try:
    if _DISABLED:
        raise ImportError("Cython DWA hot loop disabled by environment")
    from hospital_path_lab import _dwa_hotloop as _compiled
except ImportError:  # pragma: no cover - source-only or explicit fallback
    _compiled = None


CYTHON_DWA_HOTLOOP_AVAILABLE = _compiled is not None


def constant_rollout(
    start: Any,
    command: Any,
    *,
    horizon_s: float,
    step_s: float,
    pose_type: type[Any],
    trajectory_point_type: type[Any],
) -> tuple[Any, ...] | None:
    if _compiled is None:
        return None
    return _compiled.constant_rollout(
        start,
        command,
        horizon_s,
        step_s,
        pose_type,
        trajectory_point_type,
    )


def terminal_rollout(
    start: Any,
    *,
    linear_deceleration_mps2: float,
    angular_deceleration_radps2: float,
    step_s: float,
    pose_type: type[Any],
    twist_type: type[Any],
    trajectory_point_type: type[Any],
) -> tuple[Any, ...] | None:
    if _compiled is None:
        return None
    return _compiled.terminal_rollout(
        start,
        linear_deceleration_mps2,
        angular_deceleration_radps2,
        step_s,
        pose_type,
        twist_type,
        trajectory_point_type,
    )


def certified_actor_dominated_clearance(
    trajectory: tuple[Any, ...],
    **kwargs: Any,
) -> Any | None:
    if _compiled is None:
        return None
    return _compiled.certified_actor_dominated_clearance(trajectory, **kwargs)
