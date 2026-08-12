"""ctypes adapter for the optional standalone C++ DWA numeric core."""

from __future__ import annotations

import ctypes
import os
import sys
from dataclasses import dataclass
from math import ceil, isnan
from pathlib import Path

import numpy as np

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.dynamic_prediction import ActorPredictionSet, sample_actor_tubes
from hospital_path_lab.vehicle import VehicleProfile

_ABI_VERSION = 2
_DISABLED = os.environ.get("HOSPITAL_PATH_LAB_DISABLE_CPP_DWA") == "1"

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
        ("start_x", ctypes.c_double),
        ("start_y", ctypes.c_double),
        ("start_yaw", ctypes.c_double),
        ("scoring_start_x", ctypes.c_double),
        ("scoring_start_y", ctypes.c_double),
        ("goal_x", ctypes.c_double),
        ("goal_y", ctypes.c_double),
        ("previous_angular", ctypes.c_double),
        ("horizon_s", ctypes.c_double),
        ("integration_step_s", ctypes.c_double),
        ("linear_deceleration_mps2", ctypes.c_double),
        ("angular_deceleration_radps2", ctypes.c_double),
        ("half_length_m", ctypes.c_double),
        ("half_width_m", ctypes.c_double),
        ("minimum_clearance_m", ctypes.c_double),
        ("linear_count", ctypes.c_int32),
        ("angular_count", ctypes.c_int32),
        ("linear_values", _DOUBLE_P),
        ("angular_values", _DOUBLE_P),
        ("physical_occupancy", _UINT8_P),
        ("combined_occupancy", _UINT8_P),
        ("configuration_occupancy", _UINT8_P),
        ("physical_collision_occupancy", _UINT8_P),
        ("combined_collision_occupancy", _UINT8_P),
        ("forbidden_occupancy", _UINT8_P),
        ("combined_chebyshev_distance_m", _DOUBLE_P),
        ("reference_count", ctypes.c_int32),
        ("reference_xy", _DOUBLE_P),
        ("actor_time_count", ctypes.c_int32),
        ("actor_capacity", ctypes.c_int32),
        ("actor_counts", _INT32_P),
        ("actor_time_valid", _UINT8_P),
        ("actor_circles", _DOUBLE_P),
    ]


class _Candidate(ctypes.Structure):
    _fields_ = [
        ("sample_index", ctypes.c_int32),
        ("state", ctypes.c_int32),
        ("phase", ctypes.c_int32),
        ("cause", ctypes.c_int32),
        ("underlying_terminal_cause", ctypes.c_int32),
        ("used_certified_actor_dominance", ctypes.c_int32),
        ("linear", ctypes.c_double),
        ("angular", ctypes.c_double),
        ("failure_time_s", ctypes.c_double),
        ("minimum_static_clearance_m", ctypes.c_double),
        ("minimum_actor_clearance_m", ctypes.c_double),
        ("minimum_clearance_m", ctypes.c_double),
        ("progress", ctypes.c_double),
        ("progress_cost", ctypes.c_double),
        ("reference_path_cost", ctypes.c_double),
        ("heading_cost", ctypes.c_double),
        ("clearance_cost", ctypes.c_double),
        ("speed_cost", ctypes.c_double),
        ("oscillation_cost", ctypes.c_double),
        ("score", ctypes.c_double),
        ("rank", ctypes.c_double * 9),
    ]


class _Summary(ctypes.Structure):
    _fields_ = [
        ("sampled_candidates", ctypes.c_int32),
        ("moving_candidates", ctypes.c_int32),
        ("accepted_candidates", ctypes.c_int32),
        ("nonmoving_samples", ctypes.c_int32),
        ("certified_actor_dominated_candidates", ctypes.c_int32),
        ("reference_geometry_candidates", ctypes.c_int32),
    ]


@dataclass(frozen=True, slots=True)
class CppDwaCandidate:
    sample_index: int
    state: int
    phase: int
    cause: int
    underlying_terminal_cause: int
    used_certified_actor_dominance: bool
    linear: float
    angular: float
    failure_time_s: float | None
    minimum_static_clearance_m: float
    minimum_actor_clearance_m: float
    minimum_clearance_m: float
    progress: float
    progress_cost: float
    reference_path_cost: float
    heading_cost: float
    clearance_cost: float
    speed_cost: float
    oscillation_cost: float
    score: float
    rank: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class CppDwaEvaluation:
    candidates: tuple[CppDwaCandidate, ...]
    ranked_sample_indices: tuple[int, ...]
    sampled_candidates: int
    moving_candidates: int
    accepted_candidates: int
    nonmoving_samples: int
    certified_actor_dominated_candidates: int
    reference_geometry_candidates: int


@dataclass(frozen=True, slots=True)
class CppDwaStaticWorkspace:
    physical: np.ndarray
    combined: np.ndarray
    configuration: np.ndarray
    physical_collision: np.ndarray
    combined_collision: np.ndarray
    forbidden: np.ndarray
    chebyshev: np.ndarray


def _library_filename() -> str:
    if sys.platform == "win32":
        return "dwa_core.dll"
    if sys.platform == "darwin":
        return "libdwa_core.dylib"
    return "libdwa_core.so"


def _load_library() -> ctypes.CDLL | None:
    if _DISABLED:
        return None
    configured = os.environ.get("HOSPITAL_PATH_LAB_CPP_DWA_LIBRARY")
    candidate = (
        Path(configured)
        if configured
        else Path(__file__).resolve().parent / "_native" / _library_filename()
    )
    if not candidate.is_file():
        return None
    library = ctypes.CDLL(str(candidate))
    library.dwa_core_abi_version.argtypes = []
    library.dwa_core_abi_version.restype = ctypes.c_int32
    if library.dwa_core_abi_version() != _ABI_VERSION:
        raise RuntimeError("C++ DWA core ABI version mismatch")
    library.dwa_core_input_size.argtypes = []
    library.dwa_core_input_size.restype = ctypes.c_int32
    library.dwa_core_candidate_result_size.argtypes = []
    library.dwa_core_candidate_result_size.restype = ctypes.c_int32
    if library.dwa_core_input_size() != ctypes.sizeof(_Input):
        raise RuntimeError("C++ DWA input layout mismatch")
    if library.dwa_core_candidate_result_size() != ctypes.sizeof(_Candidate):
        raise RuntimeError("C++ DWA candidate layout mismatch")
    library.dwa_core_evaluate.argtypes = [
        ctypes.POINTER(_Input),
        ctypes.POINTER(_Candidate),
        ctypes.c_int32,
        _INT32_P,
        ctypes.c_int32,
        ctypes.POINTER(_Summary),
    ]
    library.dwa_core_evaluate.restype = ctypes.c_int32
    return library


try:
    _LIBRARY = _load_library()
except (OSError, RuntimeError):  # pragma: no cover - stale/incompatible local binary
    _LIBRARY = None
CPP_DWA_CORE_AVAILABLE = _LIBRARY is not None


def _as_u8(array: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(array, dtype=np.uint8)


def _as_f64(array: np.ndarray | tuple[float, ...]) -> np.ndarray:
    return np.ascontiguousarray(array, dtype=np.float64)


def _as_i32(array: np.ndarray) -> np.ndarray:
    return np.ascontiguousarray(array, dtype=np.int32)


def _double_pointer(array: np.ndarray) -> _DOUBLE_P:
    return array.ctypes.data_as(_DOUBLE_P)


def _uint8_pointer(array: np.ndarray) -> _UINT8_P:
    return array.ctypes.data_as(_UINT8_P)


def _int32_pointer(array: np.ndarray) -> _INT32_P:
    return array.ctypes.data_as(_INT32_P)


def evaluate_candidates(
    *,
    apply_start: Pose2D,
    scoring_start: Pose2D,
    goal: Pose2D,
    previous_angular: float,
    linear_values: tuple[float, ...],
    angular_values: tuple[float, ...],
    physical_checker: CollisionChecker,
    combined_checker: CollisionChecker,
    reference_path: tuple[Pose2D, ...],
    prediction_set: ActorPredictionSet,
    vehicle: VehicleProfile,
    horizon_s: float,
    integration_step_s: float,
    angular_deceleration_radps2: float,
    static_workspace: CppDwaStaticWorkspace | None = None,
) -> CppDwaEvaluation | None:
    """Evaluate the frozen candidate set, or return ``None`` for Python fallback."""

    if _LIBRARY is None:
        return None
    grid = combined_checker.grid
    linear = _as_f64(linear_values)
    angular = _as_f64(angular_values)
    workspace = static_workspace or prepare_static_workspace(
        physical_checker=physical_checker,
        combined_checker=combined_checker,
    )
    physical = workspace.physical
    combined = workspace.combined
    configuration = workspace.configuration
    physical_collision = workspace.physical_collision
    combined_collision = workspace.combined_collision
    forbidden = workspace.forbidden
    chebyshev = workspace.chebyshev
    reference = _as_f64(
        np.asarray([(pose.x, pose.y) for pose in reference_path], dtype=np.float64)
    )

    stop_duration_s = max(
        vehicle.nominal_speed_mps / vehicle.max_deceleration_mps2,
        vehicle.max_angular_speed_radps / angular_deceleration_radps2,
    )
    actor_time_count = int(ceil((horizon_s + stop_duration_s) / integration_step_s)) + 1
    actor_capacity = len(prediction_set.tubes)
    actor_counts = np.zeros(actor_time_count, dtype=np.int32)
    actor_valid = np.ones(actor_time_count, dtype=np.uint8)
    actor_circles = np.zeros((actor_time_count, max(actor_capacity, 1), 3), dtype=np.float64)
    for index in range(actor_time_count):
        try:
            circles = sample_actor_tubes(
                prediction_set,
                rollout_time_s=index * integration_step_s,
            )
        except ValueError:
            actor_valid[index] = 0
            continue
        actor_counts[index] = len(circles)
        for actor_index, circle in enumerate(circles):
            actor_circles[index, actor_index] = (
                circle.center.x,
                circle.center.y,
                circle.radius_m,
            )
    actor_counts = _as_i32(actor_counts)
    actor_valid = _as_u8(actor_valid)
    actor_circles = _as_f64(actor_circles)

    core_input = _Input(
        abi_version=_ABI_VERSION,
        width=grid.width,
        height=grid.height,
        physical_has_occupancy=int(physical_checker._has_effective_occupancy),
        combined_has_occupancy=int(combined_checker._has_effective_occupancy),
        forbidden_has_occupancy=int(combined_checker._has_forbidden_occupancy),
        resolution_m=grid.resolution_m,
        origin_x_m=grid.origin_x_m,
        origin_y_m=grid.origin_y_m,
        start_x=apply_start.x,
        start_y=apply_start.y,
        start_yaw=apply_start.yaw,
        scoring_start_x=scoring_start.x,
        scoring_start_y=scoring_start.y,
        goal_x=goal.x,
        goal_y=goal.y,
        previous_angular=previous_angular,
        horizon_s=horizon_s,
        integration_step_s=integration_step_s,
        linear_deceleration_mps2=vehicle.max_deceleration_mps2,
        angular_deceleration_radps2=angular_deceleration_radps2,
        half_length_m=vehicle.collision_length_m / 2.0,
        half_width_m=vehicle.collision_width_m / 2.0,
        minimum_clearance_m=vehicle.minimum_clearance_m,
        linear_count=len(linear),
        angular_count=len(angular),
        linear_values=_double_pointer(linear),
        angular_values=_double_pointer(angular),
        physical_occupancy=_uint8_pointer(physical),
        combined_occupancy=_uint8_pointer(combined),
        configuration_occupancy=_uint8_pointer(configuration),
        physical_collision_occupancy=_uint8_pointer(physical_collision),
        combined_collision_occupancy=_uint8_pointer(combined_collision),
        forbidden_occupancy=_uint8_pointer(forbidden),
        combined_chebyshev_distance_m=_double_pointer(chebyshev),
        reference_count=len(reference_path),
        reference_xy=_double_pointer(reference),
        actor_time_count=actor_time_count,
        actor_capacity=actor_capacity,
        actor_counts=_int32_pointer(actor_counts),
        actor_time_valid=_uint8_pointer(actor_valid),
        actor_circles=(
            _double_pointer(actor_circles)
            if actor_capacity
            else ctypes.cast(None, _DOUBLE_P)
        ),
    )
    sample_count = len(linear) * len(angular)
    raw_candidates = (_Candidate * sample_count)()
    ranked = np.full(sample_count, -1, dtype=np.int32)
    summary = _Summary()
    status = _LIBRARY.dwa_core_evaluate(
        ctypes.byref(core_input),
        raw_candidates,
        sample_count,
        _int32_pointer(ranked),
        sample_count,
        ctypes.byref(summary),
    )
    if status != 0:
        raise RuntimeError(f"C++ DWA core rejected input with status {status}")

    candidates = tuple(
        CppDwaCandidate(
            sample_index=item.sample_index,
            state=item.state,
            phase=item.phase,
            cause=item.cause,
            underlying_terminal_cause=item.underlying_terminal_cause,
            used_certified_actor_dominance=bool(item.used_certified_actor_dominance),
            linear=item.linear,
            angular=item.angular,
            failure_time_s=None if isnan(item.failure_time_s) else item.failure_time_s,
            minimum_static_clearance_m=item.minimum_static_clearance_m,
            minimum_actor_clearance_m=item.minimum_actor_clearance_m,
            minimum_clearance_m=item.minimum_clearance_m,
            progress=item.progress,
            progress_cost=item.progress_cost,
            reference_path_cost=item.reference_path_cost,
            heading_cost=item.heading_cost,
            clearance_cost=item.clearance_cost,
            speed_cost=item.speed_cost,
            oscillation_cost=item.oscillation_cost,
            score=item.score,
            rank=tuple(item.rank),
        )
        for item in raw_candidates
    )
    return CppDwaEvaluation(
        candidates=candidates,
        ranked_sample_indices=tuple(
            int(value) for value in ranked[: summary.accepted_candidates]
        ),
        sampled_candidates=summary.sampled_candidates,
        moving_candidates=summary.moving_candidates,
        accepted_candidates=summary.accepted_candidates,
        nonmoving_samples=summary.nonmoving_samples,
        certified_actor_dominated_candidates=(
            summary.certified_actor_dominated_candidates
        ),
        reference_geometry_candidates=summary.reference_geometry_candidates,
    )


def prepare_static_workspace(
    *,
    physical_checker: CollisionChecker,
    combined_checker: CollisionChecker,
) -> CppDwaStaticWorkspace:
    """Pack immutable per-map arrays once, outside the 20 Hz candidate loop."""

    return CppDwaStaticWorkspace(
        physical=_as_u8(physical_checker._effective_occupancy),
        combined=_as_u8(combined_checker._effective_occupancy),
        configuration=_as_u8(combined_checker.configuration_grid.occupancy),
        physical_collision=_as_u8(physical_checker.collision_grid.occupancy),
        combined_collision=_as_u8(combined_checker.collision_grid.occupancy),
        forbidden=_as_u8(combined_checker._forbidden_occupancy),
        chebyshev=_as_f64(combined_checker._center_chebyshev_distance_field_m),
    )
