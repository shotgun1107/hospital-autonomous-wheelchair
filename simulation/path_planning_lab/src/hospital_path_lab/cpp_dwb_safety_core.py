"""ctypes adapter for the optional C++ DWB batch safety core."""

from __future__ import annotations

import ctypes
import os
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from enum import IntEnum
from math import ceil, isnan
from pathlib import Path

import numpy as np

from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_COMMAND_APPLY_LATENCY_S,
    MAX_ACTOR_SPEED_MPS,
    ControllerSnapshot,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalPredictionSet,
    sample_directional_capsules,
)
from hospital_path_lab.dynamic_prediction import ActorPredictionSet, sample_actor_tubes
from hospital_path_lab.dynamic_safety import (
    DYNAMIC_ANGULAR_DECELERATION_RADPS2,
    DYNAMIC_SWEEP_SAMPLE_PERIOD_S,
    DynamicTrajectorySafetyCheckers,
)
from hospital_path_lab.local_algorithms.dwb_reference.contracts import DwbTrajectory

_ABI_VERSION = 1
_DISABLED = os.environ.get("HOSPITAL_PATH_LAB_DISABLE_CPP_DWB") == "1"

_DOUBLE_P = ctypes.POINTER(ctypes.c_double)
_INT32_P = ctypes.POINTER(ctypes.c_int32)
_UINT8_P = ctypes.POINTER(ctypes.c_uint8)


class _Input(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_int32),
        ("width", ctypes.c_int32),
        ("height", ctypes.c_int32),
        ("physical_has_occupancy", ctypes.c_int32),
        ("combined_has_occupancy", ctypes.c_int32),
        ("forbidden_has_occupancy", ctypes.c_int32),
        ("resolution_m", ctypes.c_double),
        ("origin_x_m", ctypes.c_double),
        ("origin_y_m", ctypes.c_double),
        ("half_length_m", ctypes.c_double),
        ("half_width_m", ctypes.c_double),
        ("minimum_clearance_m", ctypes.c_double),
        ("linear_deceleration_mps2", ctypes.c_double),
        ("angular_deceleration_radps2", ctypes.c_double),
        ("sweep_step_s", ctypes.c_double),
        ("apply_duration_s", ctypes.c_double),
        ("maximum_actor_speed_mps", ctypes.c_double),
        ("robot_x", ctypes.c_double),
        ("robot_y", ctypes.c_double),
        ("robot_yaw", ctypes.c_double),
        ("robot_linear", ctypes.c_double),
        ("robot_angular", ctypes.c_double),
        ("candidate_count", ctypes.c_int32),
        ("pose_count", ctypes.c_int32),
        ("trajectory_step_s", ctypes.c_double),
        ("commands", _DOUBLE_P),
        ("trajectory_poses", _DOUBLE_P),
        ("physical_occupancy", _UINT8_P),
        ("combined_occupancy", _UINT8_P),
        ("forbidden_occupancy", _UINT8_P),
        ("combined_chebyshev_distance_m", _DOUBLE_P),
        ("actor_time_count", ctypes.c_int32),
        ("actor_capacity", ctypes.c_int32),
        ("actor_counts", _INT32_P),
        ("actor_time_valid", _UINT8_P),
        ("actor_capsules", _DOUBLE_P),
    ]


class _Result(ctypes.Structure):
    _fields_ = [
        ("failure", ctypes.c_int32),
        ("failure_time_s", ctypes.c_double),
        ("minimum_static_clearance_m", ctypes.c_double),
        ("minimum_actor_clearance_m", ctypes.c_double),
    ]


class CppDwbSafetyFailure(IntEnum):
    SAFE = 0
    FORBIDDEN_ZONE = 1
    STATIC_CLEARANCE = 2
    ACTOR_CLEARANCE = 3
    PREDICTION_INVALID = 4


@dataclass(frozen=True, slots=True)
class CppDwbSafetyResult:
    failure: CppDwbSafetyFailure
    failure_time_s: float | None
    minimum_static_clearance_m: float | None
    minimum_actor_clearance_m: float | None


@dataclass(frozen=True, slots=True)
class CppDwbSafetyStaticWorkspace:
    """Only the map arrays consumed by the native DWB safety ABI."""

    physical: np.ndarray
    combined: np.ndarray
    forbidden: np.ndarray
    chebyshev: np.ndarray


def _library_filename() -> str:
    if sys.platform == "win32":
        return "dwb_safety_core.dll"
    if sys.platform == "darwin":
        return "libdwb_safety_core.dylib"
    return "libdwb_safety_core.so"


def _load_library() -> ctypes.CDLL | None:
    if _DISABLED:
        return None
    configured = os.environ.get("HOSPITAL_PATH_LAB_CPP_DWB_LIBRARY")
    candidate = (
        Path(configured)
        if configured
        else Path(__file__).resolve().parent / "_native" / _library_filename()
    )
    if not candidate.is_file():
        return None
    library = ctypes.CDLL(str(candidate))
    library.dwb_safety_core_abi_version.argtypes = []
    library.dwb_safety_core_abi_version.restype = ctypes.c_int32
    library.dwb_safety_core_input_size.argtypes = []
    library.dwb_safety_core_input_size.restype = ctypes.c_int32
    library.dwb_safety_core_result_size.argtypes = []
    library.dwb_safety_core_result_size.restype = ctypes.c_int32
    if library.dwb_safety_core_abi_version() != _ABI_VERSION:
        raise RuntimeError("C++ DWB safety core ABI version mismatch")
    if library.dwb_safety_core_input_size() != ctypes.sizeof(_Input):
        raise RuntimeError("C++ DWB safety input layout mismatch")
    if library.dwb_safety_core_result_size() != ctypes.sizeof(_Result):
        raise RuntimeError("C++ DWB safety result layout mismatch")
    library.dwb_safety_core_evaluate.argtypes = [
        ctypes.POINTER(_Input),
        ctypes.POINTER(_Result),
        ctypes.c_int32,
    ]
    library.dwb_safety_core_evaluate.restype = ctypes.c_int32
    return library


try:
    _LIBRARY = _load_library()
except (OSError, RuntimeError):  # pragma: no cover - stale local binary
    _LIBRARY = None

CPP_DWB_SAFETY_CORE_AVAILABLE = _LIBRARY is not None


def _as_f64(value) -> np.ndarray:
    return np.ascontiguousarray(value, dtype=np.float64)


def _as_i32(value) -> np.ndarray:
    return np.ascontiguousarray(value, dtype=np.int32)


def _as_u8(value) -> np.ndarray:
    return np.ascontiguousarray(value, dtype=np.uint8)


def _double_pointer(array: np.ndarray) -> _DOUBLE_P:
    return array.ctypes.data_as(_DOUBLE_P)


def _int32_pointer(array: np.ndarray) -> _INT32_P:
    return array.ctypes.data_as(_INT32_P)


def _uint8_pointer(array: np.ndarray) -> _UINT8_P:
    return array.ctypes.data_as(_UINT8_P)


def _pack_actor_capsules(
    prediction: ActorPredictionSet | DirectionalPredictionSet,
    *,
    time_count: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    capacity = len(prediction.tubes)
    counts = np.zeros(time_count, dtype=np.int32)
    valid = np.ones(time_count, dtype=np.uint8)
    capsules = np.zeros((time_count, max(capacity, 1), 5), dtype=np.float64)
    for index in range(time_count):
        rollout_time_s = index * DYNAMIC_SWEEP_SAMPLE_PERIOD_S
        try:
            if isinstance(prediction, DirectionalPredictionSet):
                shapes = sample_directional_capsules(
                    prediction,
                    rollout_time_s=rollout_time_s,
                )
                packed = tuple(
                    (
                        shape.start.x,
                        shape.start.y,
                        shape.end.x,
                        shape.end.y,
                        shape.base_radius_m,
                    )
                    for shape in shapes
                )
            else:
                shapes = sample_actor_tubes(
                    prediction,
                    rollout_time_s=rollout_time_s,
                )
                packed = tuple(
                    (
                        shape.center.x,
                        shape.center.y,
                        shape.center.x,
                        shape.center.y,
                        shape.radius_m,
                    )
                    for shape in shapes
                )
        except (AttributeError, OverflowError, TypeError, ValueError):
            valid[index] = 0
            continue
        counts[index] = len(packed)
        for actor_index, shape in enumerate(packed):
            capsules[index, actor_index] = shape
    return _as_i32(counts), _as_u8(valid), _as_f64(capsules)


def prepare_dwb_safety_static_workspace(
    checkers: DynamicTrajectorySafetyCheckers,
) -> CppDwbSafetyStaticWorkspace:
    """Pack no configuration/collision grids that this core never reads."""

    return CppDwbSafetyStaticWorkspace(
        physical=_as_u8(checkers.physical_checker._effective_occupancy),
        combined=_as_u8(checkers.combined_checker._effective_occupancy),
        forbidden=_as_u8(checkers.combined_checker._forbidden_occupancy),
        chebyshev=_as_f64(
            checkers.combined_checker._center_chebyshev_distance_field_m
        ),
    )


def evaluate_dwb_safety_batch(
    *,
    trajectories: Sequence[DwbTrajectory],
    snapshot: ControllerSnapshot,
    checkers: DynamicTrajectorySafetyCheckers,
    static_workspace: CppDwbSafetyStaticWorkspace | None = None,
    native_commands: np.ndarray | None = None,
    native_poses: np.ndarray | None = None,
    native_integration_step_s: float | None = None,
) -> tuple[CppDwbSafetyResult, ...] | None:
    """Evaluate one frozen DWB trajectory batch, or return ``None`` for fallback."""

    if _LIBRARY is None:
        return None
    if not trajectories:
        raise ValueError("DWB safety batch must not be empty")
    if native_commands is not None or native_poses is not None:
        if (
            native_commands is None
            or native_poses is None
            or native_integration_step_s is None
        ):
            raise ValueError("native DWB safety buffers require one integration step")
        commands = native_commands
        poses = native_poses
        candidate_count = commands.shape[0]
        pose_count = poses.shape[1] if poses.ndim == 3 else 0
        trajectory_step_s = native_integration_step_s
    else:
        pose_count = len(trajectories[0].poses)
        trajectory_step_s = trajectories[0].integration_step_s
        if pose_count < 1 or any(
            len(trajectory.poses) != pose_count
            or trajectory.integration_step_s != trajectory_step_s
            for trajectory in trajectories
        ):
            raise ValueError(
                "DWB safety batch trajectories must share one shape and step"
            )
        candidate_count = len(trajectories)
        commands = _as_f64(
            [(item.command.linear_mps, item.command.angular_radps) for item in trajectories]
        )
        poses = _as_f64(
            [
                [(pose.x_m, pose.y_m, pose.yaw_rad) for pose in item.poses]
                for item in trajectories
            ]
        )
    if (
        commands.dtype != np.float64
        or not commands.flags.c_contiguous
        or commands.shape != (candidate_count, 2)
        or poses.dtype != np.float64
        or not poses.flags.c_contiguous
        or poses.shape != (candidate_count, pose_count, 3)
        or candidate_count != len(trajectories)
        or pose_count < 1
        or trajectory_step_s <= 0.0
    ):
        raise ValueError("native DWB safety buffers do not match trajectory batch")
    profile = snapshot.vehicle_profile
    maximum_stop_s = max(
        profile.max_forward_speed_mps / profile.max_deceleration_mps2,
        profile.max_reverse_speed_mps / profile.max_deceleration_mps2,
        profile.max_angular_speed_radps / DYNAMIC_ANGULAR_DECELERATION_RADPS2,
    )
    rollout_duration_s = (pose_count - 1) * trajectory_step_s
    actor_time_count = (
        int(
            ceil(
                (rollout_duration_s + maximum_stop_s)
                / DYNAMIC_SWEEP_SAMPLE_PERIOD_S
            )
        )
        + 1
    )
    counts, valid, capsules = _pack_actor_capsules(
        snapshot.actor_tubes,
        time_count=actor_time_count,
    )
    workspace = static_workspace or prepare_dwb_safety_static_workspace(checkers)
    grid = snapshot.static_grid_snapshot.grid
    robot = snapshot.robot_state
    core_input = _Input(
        abi_version=_ABI_VERSION,
        width=grid.width,
        height=grid.height,
        physical_has_occupancy=int(checkers.physical_checker._has_effective_occupancy),
        combined_has_occupancy=int(checkers.combined_checker._has_effective_occupancy),
        forbidden_has_occupancy=int(checkers.combined_checker._has_forbidden_occupancy),
        resolution_m=grid.resolution_m,
        origin_x_m=grid.origin_x_m,
        origin_y_m=grid.origin_y_m,
        half_length_m=profile.collision_length_m / 2.0,
        half_width_m=profile.collision_width_m / 2.0,
        minimum_clearance_m=profile.minimum_clearance_m,
        linear_deceleration_mps2=profile.max_deceleration_mps2,
        angular_deceleration_radps2=DYNAMIC_ANGULAR_DECELERATION_RADPS2,
        sweep_step_s=DYNAMIC_SWEEP_SAMPLE_PERIOD_S,
        apply_duration_s=DYNAMIC_COMMAND_APPLY_LATENCY_S,
        maximum_actor_speed_mps=MAX_ACTOR_SPEED_MPS,
        robot_x=robot.pose.x,
        robot_y=robot.pose.y,
        robot_yaw=robot.pose.yaw,
        robot_linear=robot.twist.linear,
        robot_angular=robot.twist.angular,
        candidate_count=candidate_count,
        pose_count=pose_count,
        trajectory_step_s=trajectory_step_s,
        commands=_double_pointer(commands),
        trajectory_poses=_double_pointer(poses),
        physical_occupancy=_uint8_pointer(workspace.physical),
        combined_occupancy=_uint8_pointer(workspace.combined),
        forbidden_occupancy=_uint8_pointer(workspace.forbidden),
        combined_chebyshev_distance_m=_double_pointer(workspace.chebyshev),
        actor_time_count=actor_time_count,
        actor_capacity=len(snapshot.actor_tubes.tubes),
        actor_counts=_int32_pointer(counts),
        actor_time_valid=_uint8_pointer(valid),
        actor_capsules=(
            _double_pointer(capsules)
            if snapshot.actor_tubes.tubes
            else ctypes.cast(None, _DOUBLE_P)
        ),
    )
    raw_results = (_Result * candidate_count)()
    status = _LIBRARY.dwb_safety_core_evaluate(
        ctypes.byref(core_input),
        raw_results,
        candidate_count,
    )
    if status != 0:
        raise RuntimeError(f"C++ DWB safety core rejected input with status {status}")
    return tuple(
        CppDwbSafetyResult(
            failure=CppDwbSafetyFailure(item.failure),
            failure_time_s=None if isnan(item.failure_time_s) else item.failure_time_s,
            minimum_static_clearance_m=(
                None
                if np.isinf(item.minimum_static_clearance_m)
                else item.minimum_static_clearance_m
            ),
            minimum_actor_clearance_m=(
                None
                if np.isinf(item.minimum_actor_clearance_m)
                else item.minimum_actor_clearance_m
            ),
        )
        for item in raw_results
    )
