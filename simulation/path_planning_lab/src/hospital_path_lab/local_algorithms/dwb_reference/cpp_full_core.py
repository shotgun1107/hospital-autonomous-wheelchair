"""Optional C++ DWB generation, critic scoring, and selection core.

Python keeps project contracts, reference-session ownership, and the independent
shared safety gate.  The native core owns the local DWB numerical loop: dynamic
window sampling, constant-twist rollout, seven frozen critic scores, short-circuit
ordering, and strict lowest-cost selection.
"""

from __future__ import annotations

import ctypes
import os
import sys
from collections.abc import Sequence
from math import isnan
from pathlib import Path

import numpy as np

from hospital_path_lab.local_reference_contracts import ReferenceTravelDirection

from .contracts import (
    DwbGeneratorRequest,
    DwbGeneratorResult,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)
from .core import (
    CandidateEvaluationDiagnostic,
    CandidateEvaluationStatus,
    CandidateFailureDiagnostic,
    CandidateFailureKind,
    CriticScoreDiagnostic,
    DwbCoreResult,
    DwbCriticBinding,
    DwbPreparationError,
    DwbReferenceCore,
    NoLegalTrajectoryError,
)
from .critics import (
    GoalAlignCritic,
    GoalDistCritic,
    OscillationCritic,
    PathAlignCritic,
    PathDistCritic,
    RotateToGoalCritic,
)

_ABI_VERSION = 1
_CRITIC_COUNT = 7
_EXPECTED_CRITICS = (
    "project_safety",
    "rotate_to_goal",
    "oscillation",
    "goal_align",
    "path_align",
    "path_dist",
    "goal_dist",
)
_DISABLED = os.environ.get("HOSPITAL_PATH_LAB_DISABLE_CPP_DWB_FULL") == "1"

_DOUBLE_P = ctypes.POINTER(ctypes.c_double)
_INT32_P = ctypes.POINTER(ctypes.c_int32)
_UINT8_P = ctypes.POINTER(ctypes.c_uint8)


class _GeneratorInput(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_int32),
        ("control_period_s", ctypes.c_double),
        ("rollout_duration_s", ctypes.c_double),
        ("integration_step_s", ctypes.c_double),
        ("maximum_forward_speed_mps", ctypes.c_double),
        ("maximum_reverse_speed_mps", ctypes.c_double),
        ("linear_acceleration_mps2", ctypes.c_double),
        ("linear_deceleration_mps2", ctypes.c_double),
        ("maximum_angular_speed_radps", ctypes.c_double),
        ("angular_acceleration_radps2", ctypes.c_double),
        ("angular_deceleration_radps2", ctypes.c_double),
        ("linear_sample_count", ctypes.c_int32),
        ("angular_sample_count", ctypes.c_int32),
        ("allow_reverse", ctypes.c_int32),
        ("travel_direction", ctypes.c_int32),
        ("prefer_forward_progress", ctypes.c_int32),
        ("pose_x", ctypes.c_double),
        ("pose_y", ctypes.c_double),
        ("pose_yaw", ctypes.c_double),
        ("current_linear", ctypes.c_double),
        ("current_angular", ctypes.c_double),
    ]


class _GeneratorOutput(ctypes.Structure):
    _fields_ = [
        ("linear_minimum", ctypes.c_double),
        ("linear_maximum", ctypes.c_double),
        ("angular_minimum", ctypes.c_double),
        ("angular_maximum", ctypes.c_double),
        ("linear_count", ctypes.c_int32),
        ("angular_count", ctypes.c_int32),
        ("candidate_count", ctypes.c_int32),
        ("pose_count", ctypes.c_int32),
    ]


class _EvaluationInput(ctypes.Structure):
    _fields_ = [
        ("abi_version", ctypes.c_int32),
        ("candidate_count", ctypes.c_int32),
        ("pose_count", ctypes.c_int32),
        ("trajectory_step_s", ctypes.c_double),
        ("commands", _DOUBLE_P),
        ("poses", _DOUBLE_P),
        ("safety_failures", _INT32_P),
        ("critic_scales", _DOUBLE_P),
        ("short_circuit", ctypes.c_int32),
        ("width", ctypes.c_int32),
        ("height", ctypes.c_int32),
        ("resolution_m", ctypes.c_double),
        ("origin_x_m", ctypes.c_double),
        ("origin_y_m", ctypes.c_double),
        ("blocked_cells", _UINT8_P),
        ("goal_align_field", _INT32_P),
        ("path_align_field", _INT32_P),
        ("path_dist_field", _INT32_P),
        ("goal_dist_field", _INT32_P),
        ("forward_point_distance_m", ctypes.c_double),
        ("goal_align_projection_sign", ctypes.c_double),
        ("path_align_projection_sign", ctypes.c_double),
        ("goal_align_disabled", ctypes.c_int32),
        ("path_align_disabled", ctypes.c_int32),
        ("rotate_in_window", ctypes.c_int32),
        ("rotate_rotating", ctypes.c_int32),
        ("rotate_goal_yaw", ctypes.c_double),
        ("rotate_current_speed_sq", ctypes.c_double),
        ("rotate_slowing_factor", ctypes.c_double),
        ("rotate_lookahead_time_s", ctypes.c_double),
        ("oscillation_linear_positive_only", ctypes.c_int32),
        ("oscillation_linear_negative_only", ctypes.c_int32),
        ("oscillation_angular_positive_only", ctypes.c_int32),
        ("oscillation_angular_negative_only", ctypes.c_int32),
    ]


class _EvaluationOutput(ctypes.Structure):
    _fields_ = [
        ("selected_candidate_index", ctypes.c_int32),
        ("selected_total_score", ctypes.c_double),
        ("legal_count", ctypes.c_int32),
    ]


def _library_filename() -> str:
    if sys.platform == "win32":
        return "dwb_full_core.dll"
    if sys.platform == "darwin":
        return "libdwb_full_core.dylib"
    return "libdwb_full_core.so"


def _load_library() -> ctypes.CDLL | None:
    if _DISABLED:
        return None
    configured = os.environ.get("HOSPITAL_PATH_LAB_CPP_DWB_FULL_LIBRARY")
    candidate = (
        Path(configured)
        if configured
        else Path(__file__).resolve().parents[2] / "_native" / _library_filename()
    )
    if not candidate.is_file():
        return None
    library = ctypes.CDLL(str(candidate))
    for function_name in (
        "dwb_full_core_abi_version",
        "dwb_full_core_generator_input_size",
        "dwb_full_core_generator_output_size",
        "dwb_full_core_evaluation_input_size",
        "dwb_full_core_evaluation_output_size",
    ):
        function = getattr(library, function_name)
        function.argtypes = []
        function.restype = ctypes.c_int32
    if library.dwb_full_core_abi_version() != _ABI_VERSION:
        raise RuntimeError("C++ DWB full core ABI version mismatch")
    expected_sizes = (
        (library.dwb_full_core_generator_input_size(), ctypes.sizeof(_GeneratorInput)),
        (library.dwb_full_core_generator_output_size(), ctypes.sizeof(_GeneratorOutput)),
        (library.dwb_full_core_evaluation_input_size(), ctypes.sizeof(_EvaluationInput)),
        (library.dwb_full_core_evaluation_output_size(), ctypes.sizeof(_EvaluationOutput)),
    )
    if any(actual != expected for actual, expected in expected_sizes):
        raise RuntimeError("C++ DWB full core ABI layout mismatch")
    library.dwb_full_core_generate.argtypes = [
        ctypes.POINTER(_GeneratorInput),
        ctypes.POINTER(_GeneratorOutput),
        _DOUBLE_P,
        ctypes.c_int32,
        _DOUBLE_P,
        ctypes.c_int32,
        _DOUBLE_P,
        ctypes.c_int32,
        _DOUBLE_P,
        ctypes.c_int32,
    ]
    library.dwb_full_core_generate.restype = ctypes.c_int32
    library.dwb_full_core_evaluate.argtypes = [
        ctypes.POINTER(_EvaluationInput),
        ctypes.POINTER(_EvaluationOutput),
        _INT32_P,
        _DOUBLE_P,
        _DOUBLE_P,
        _INT32_P,
        ctypes.c_int32,
    ]
    library.dwb_full_core_evaluate.restype = ctypes.c_int32
    library.dwb_full_core_manhattan_field.argtypes = [
        ctypes.c_int32,
        ctypes.c_int32,
        _UINT8_P,
        _INT32_P,
        ctypes.c_int32,
        _INT32_P,
        ctypes.c_int32,
    ]
    library.dwb_full_core_manhattan_field.restype = ctypes.c_int32
    return library


try:
    _LIBRARY = _load_library()
except (OSError, RuntimeError):  # pragma: no cover - stale local binary
    _LIBRARY = None

CPP_DWB_FULL_CORE_AVAILABLE = _LIBRARY is not None


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


def _generator_modes(generator) -> tuple[int, int]:
    direction = getattr(generator, "_travel_direction", None)
    if direction is ReferenceTravelDirection.FORWARD:
        travel = 1
    elif direction is ReferenceTravelDirection.REVERSE:
        travel = 2
    else:
        travel = 0
    prefer = int(bool(getattr(generator, "_prefer_forward_progress_on_exact_ties", False)))
    return travel, prefer


def generate_dwb_full_batch(generator, request: DwbGeneratorRequest) -> DwbGeneratorResult | None:
    """Generate one exact native lattice, or return ``None`` for Python fallback."""

    if _LIBRARY is None:
        return None
    config = generator.config
    travel, prefer = _generator_modes(generator)
    linear_capacity = config.linear_sample_count + 1
    angular_capacity = config.angular_sample_count + 1
    candidate_capacity = linear_capacity * angular_capacity
    pose_count = config.rollout_step_count + 1
    linear = np.empty(linear_capacity, dtype=np.float64)
    angular = np.empty(angular_capacity, dtype=np.float64)
    commands = np.empty((candidate_capacity, 2), dtype=np.float64)
    poses = np.empty((candidate_capacity, pose_count, 3), dtype=np.float64)
    native_input = _GeneratorInput(
        _ABI_VERSION,
        config.control_period_s,
        config.rollout_duration_s,
        config.integration_step_s,
        config.maximum_forward_speed_mps,
        config.maximum_reverse_speed_mps,
        config.linear_acceleration_mps2,
        config.linear_deceleration_mps2,
        config.maximum_angular_speed_radps,
        config.angular_acceleration_radps2,
        config.angular_deceleration_radps2,
        config.linear_sample_count,
        config.angular_sample_count,
        int(config.allow_reverse),
        travel,
        prefer,
        request.pose.x_m,
        request.pose.y_m,
        request.pose.yaw_rad,
        request.current_twist.linear_mps,
        request.current_twist.angular_radps,
    )
    native_output = _GeneratorOutput()
    return_code = _LIBRARY.dwb_full_core_generate(
        ctypes.byref(native_input),
        ctypes.byref(native_output),
        _double_pointer(linear),
        linear.size,
        _double_pointer(angular),
        angular.size,
        _double_pointer(commands),
        commands.size,
        _double_pointer(poses),
        poses.size,
    )
    if return_code != 0:
        raise ValueError(f"C++ DWB generator rejected request: {return_code}")
    count = native_output.candidate_count
    native_trajectories = tuple(
        DwbTrajectory(
            command=DwbTwist2D(float(commands[index, 0]), float(commands[index, 1])),
            poses=tuple(
                DwbPose2D(float(x), float(y), float(yaw))
                for x, y, yaw in poses[index, : native_output.pose_count]
            ),
            integration_step_s=config.integration_step_s,
        )
        for index in range(count)
    )
    return DwbGeneratorResult(
        linear_window_mps=(native_output.linear_minimum, native_output.linear_maximum),
        angular_window_radps=(native_output.angular_minimum, native_output.angular_maximum),
        linear_samples_mps=tuple(float(value) for value in linear[: native_output.linear_count]),
        angular_samples_radps=tuple(
            float(value) for value in angular[: native_output.angular_count]
        ),
        trajectories=native_trajectories,
    )


def _distance_array(critic, *, allow_disabled: bool = False) -> np.ndarray:
    field = critic.distance_field
    if field is None:
        if not allow_disabled:
            raise RuntimeError("prepared native DWB distance field is missing")
        return np.zeros((critic.grid.height, critic.grid.width), dtype=np.int32)
    return _as_i32(
        [[-1 if value is None else value for value in row] for row in field.distances]
    )


def _blocked_array(grid) -> np.ndarray:
    blocked = np.zeros((grid.height, grid.width), dtype=np.uint8)
    for x, y in grid.blocked_cells:
        blocked[y, x] = 1
    return blocked


def build_native_manhattan_distances(
    grid,
    source_cells,
) -> tuple[tuple[int | None, ...], ...] | None:
    """Build the frozen four-neighbour field in C++, or request Python fallback."""

    if _LIBRARY is None:
        return None
    blocked = _blocked_array(grid)
    sources = _as_i32(source_cells)
    if sources.ndim != 2 or sources.shape[1] != 2 or sources.shape[0] == 0:
        raise ValueError("native distance field requires x/y source cells")
    distances = np.empty((grid.height, grid.width), dtype=np.int32)
    return_code = _LIBRARY.dwb_full_core_manhattan_field(
        grid.width,
        grid.height,
        _uint8_pointer(blocked),
        _int32_pointer(sources),
        sources.shape[0],
        _int32_pointer(distances),
        distances.size,
    )
    if return_code != 0:
        raise ValueError(f"C++ DWB distance field rejected input: {return_code}")
    return tuple(
        tuple(None if value < 0 else int(value) for value in row)
        for row in distances
    )


_FAILURE_REASON = {
    1: "forbidden_zone_entry",
    2: "static_clearance_below_minimum",
    3: "actor_clearance_below_minimum",
    4: "prediction_set_malformed",
    100: "not_slowing_near_goal",
    101: "translation_during_goal_rotation",
    200: "oscillation_sign_reversal",
    300: "off_grid",
    301: "blocked_grid_cell",
    302: "unreachable_grid_cell",
    900: "invalid_critic_score",
}


class CppDwbReferenceCore:
    """Drop-in DWB core with native generation, scoring, and selection."""

    def __init__(
        self,
        generator,
        critics: Sequence[DwbCriticBinding],
        *,
        short_circuit_trajectory_evaluation: bool = True,
    ) -> None:
        self._generator = generator
        self._critics = tuple(critics)
        self._short_circuit = short_circuit_trajectory_evaluation
        self._fallback = DwbReferenceCore(
            generator,
            critics,
            short_circuit_trajectory_evaluation=short_circuit_trajectory_evaluation,
        )
        for binding in self._critics:
            if isinstance(
                binding.critic,
                (GoalAlignCritic, PathAlignCritic, PathDistCritic, GoalDistCritic),
            ):
                binding.critic._use_cpp_distance_field = True
        self.native_used = False

    @property
    def path(self):
        return self._fallback.path

    @property
    def critic_names(self) -> tuple[str, ...]:
        return tuple(binding.name for binding in self._critics)

    def set_path(self, path) -> None:
        self._fallback.set_path(path)

    def reset(self) -> None:
        self._fallback.reset()
        self.native_used = False

    def compute(self, request: DwbGeneratorRequest) -> DwbCoreResult:
        self.native_used = False
        if _LIBRARY is None or self.critic_names != _EXPECTED_CRITICS:
            return self._fallback.compute(request)
        if not self._supported_critics():
            return self._fallback.compute(request)

        for binding in self._critics:
            if binding.critic.prepare(request) is False:
                raise DwbPreparationError(binding.name)
        generated = generate_dwb_full_batch(self._generator, request)
        if generated is None:  # pragma: no cover - checked above
            return self._fallback.compute(request)
        safety_batch = self._critics[0].critic.score_batch(generated.trajectories)
        if safety_batch is None:
            return self._fallback.compute(request)
        evaluations, selected_index, selected_score = self._evaluate_native(
            generated,
            safety_batch,
        )
        self.native_used = True
        if selected_index < 0:
            raise NoLegalTrajectoryError(evaluations)
        selected = generated.trajectories[selected_index]
        for binding in self._critics:
            binding.critic.debrief(selected.command)
        return DwbCoreResult(
            command=selected.command,
            trajectory=selected,
            total_score=selected_score,
            selected_candidate_index=selected_index,
            generator_result=generated,
            candidate_evaluations=evaluations,
        )

    def _supported_critics(self) -> bool:
        return (
            isinstance(self._critics[1].critic, RotateToGoalCritic)
            and isinstance(self._critics[2].critic, OscillationCritic)
            and isinstance(self._critics[3].critic, GoalAlignCritic)
            and isinstance(self._critics[4].critic, PathAlignCritic)
            and isinstance(self._critics[5].critic, PathDistCritic)
            and isinstance(self._critics[6].critic, GoalDistCritic)
            and hasattr(self._critics[0].critic, "score_batch")
        )

    def _evaluate_native(self, generated, safety_batch):
        trajectories = generated.trajectories
        commands = _as_f64(
            [(item.command.linear_mps, item.command.angular_radps) for item in trajectories]
        )
        poses = _as_f64(
            [[(pose.x_m, pose.y_m, pose.yaw_rad) for pose in item.poses] for item in trajectories]
        )
        safety_failures = _as_i32(
            [
                0
                if item.reason_code is None
                else {
                    "forbidden_zone_entry": 1,
                    "static_clearance_below_minimum": 2,
                    "actor_clearance_below_minimum": 3,
                    "prediction_set_malformed": 4,
                }[item.reason_code]
                for item in safety_batch
            ]
        )
        scales = _as_f64([binding.scale for binding in self._critics])
        rotate = self._critics[1].critic
        oscillation = self._critics[2].critic
        goal_align = self._critics[3].critic
        path_align = self._critics[4].critic
        path_dist = self._critics[5].critic
        goal_dist = self._critics[6].critic
        grid = path_dist.grid
        blocked = _blocked_array(grid)
        fields = (
            _distance_array(goal_align, allow_disabled=goal_align.disabled_near_goal),
            _distance_array(path_align, allow_disabled=path_align.disabled_near_goal),
            _distance_array(path_dist),
            _distance_array(goal_dist),
        )
        rotate_state = rotate.native_scoring_state
        oscillation_state = oscillation.native_restriction_flags
        native_input = _EvaluationInput(
            _ABI_VERSION,
            len(trajectories),
            len(trajectories[0].poses),
            trajectories[0].integration_step_s,
            _double_pointer(commands),
            _double_pointer(poses),
            _int32_pointer(safety_failures),
            _double_pointer(scales),
            int(self._short_circuit),
            grid.width,
            grid.height,
            grid.resolution_m,
            grid.origin_x_m,
            grid.origin_y_m,
            _uint8_pointer(blocked),
            _int32_pointer(fields[0]),
            _int32_pointer(fields[1]),
            _int32_pointer(fields[2]),
            _int32_pointer(fields[3]),
            goal_align.forward_point_distance_m,
            goal_align._projection_sign,
            path_align._projection_sign,
            int(goal_align.disabled_near_goal),
            int(path_align.disabled_near_goal),
            int(rotate_state[0]),
            int(rotate_state[1]),
            rotate_state[2],
            rotate_state[3],
            rotate_state[4],
            rotate_state[5],
            *(int(value) for value in oscillation_state),
        )
        count = len(trajectories)
        statuses = np.empty(count, dtype=np.int32)
        accumulated = np.empty(count, dtype=np.float64)
        raw = np.empty((count, _CRITIC_COUNT), dtype=np.float64)
        failures = np.empty(count, dtype=np.int32)
        native_output = _EvaluationOutput()
        return_code = _LIBRARY.dwb_full_core_evaluate(
            ctypes.byref(native_input),
            ctypes.byref(native_output),
            _int32_pointer(statuses),
            _double_pointer(accumulated),
            _double_pointer(raw),
            _int32_pointer(failures),
            count,
        )
        if return_code not in (0, 1):
            raise RuntimeError(f"C++ DWB evaluator failed: {return_code}")
        evaluations = tuple(
            self._evaluation_from_native(
                index,
                trajectories[index],
                int(statuses[index]),
                float(accumulated[index]),
                raw[index],
                int(failures[index]),
            )
            for index in range(count)
        )
        return (
            evaluations,
            int(native_output.selected_candidate_index),
            float(native_output.selected_total_score),
        )

    def _evaluation_from_native(
        self,
        index,
        trajectory,
        status_value,
        accumulated,
        raw_row,
        failure_code,
    ) -> CandidateEvaluationDiagnostic:
        status = (
            CandidateEvaluationStatus.LEGAL
            if status_value == 0
            else CandidateEvaluationStatus.ILLEGAL
            if status_value == 1
            else CandidateEvaluationStatus.SHORT_CIRCUITED
        )
        scores = tuple(
            CriticScoreDiagnostic(
                critic_name=binding.name,
                raw_score=float(raw_score),
                scale=binding.scale,
                weighted_score=float(raw_score) * binding.scale,
            )
            for binding, raw_score in zip(self._critics, raw_row, strict=True)
            if not isnan(raw_score)
        )
        failure = None
        if status is CandidateEvaluationStatus.ILLEGAL:
            reason = _FAILURE_REASON[failure_code]
            failure_index = next(
                index
                for index, (binding, raw_score) in enumerate(
                    zip(self._critics, raw_row, strict=True)
                )
                if binding.scale != 0.0 and isnan(raw_score)
            )
            failure = CandidateFailureDiagnostic(
                kind=(
                    CandidateFailureKind.INVALID_SCORE
                    if failure_code == 900
                    else CandidateFailureKind.CRITIC_REJECTION
                ),
                critic_name=self._critics[failure_index].name,
                reason_code=reason,
                message=reason,
            )
        return CandidateEvaluationDiagnostic(
            candidate_index=index,
            command=trajectory.command,
            status=status,
            accumulated_score=accumulated,
            critic_scores=scores,
            failure=failure,
        )


__all__ = [
    "CPP_DWB_FULL_CORE_AVAILABLE",
    "CppDwbReferenceCore",
    "build_native_manhattan_distances",
    "generate_dwb_full_batch",
]
