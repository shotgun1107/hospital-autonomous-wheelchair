"""R7 public-only native DWB parity and serial timing qualification."""

from __future__ import annotations

import ctypes
import json
import os
import platform
import re
import subprocess
import sys
from dataclasses import replace
from hashlib import sha256
from importlib.metadata import PackageNotFoundError, version
from importlib.util import find_spec
from math import ceil, floor
from multiprocessing import active_children
from pathlib import Path
from time import perf_counter_ns

from hospital_path_lab.contracts import GridSnapshot
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    ControllerCommandResult,
    ControllerSnapshot,
    build_controller_snapshot,
)
from hospital_path_lab.dynamic_corpus import (
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_observation import dynamic_observation_content_hash
from hospital_path_lab.dynamic_prediction import build_actor_prediction_set
from hospital_path_lab.dynamic_runner import _qualification_snapshot_cases
from hospital_path_lab.local_algorithms.dwb_reference import (
    SourceDerivedDynamicDwbController,
)
from hospital_path_lab.map_factory import canonical_content_hash

R7_NATIVE_QUALIFICATION_VERSION = "r7-native-dwb-qualification-v2"
R7_GATE_SCHEMA = "r7-native-release-gate-v2"
R7_RECEIPT_SCHEMA = "r7-native-qualification-receipt-v2"
R7_DEADLINE_NS = 50_000_000
R7_STANDARD_WARMUPS = 30
R7_STANDARD_REPEATS = 100
R7_SNAPSHOT_COUNT = 5

R6_RECEIPT_SCHEMA = "r6-public-end-to-end-receipt-v1"
R6_REQUIRED_CASE_COUNT = 17
R6_EXPECTED_RESULT_HASH = (
    "e1c086fc836c44d7b793aaccae1a834cff4bdb8b386f39a4f13af2b133168151"
)
R6_EXPECTED_RECEIPT_HASH = (
    "2d37f43b720ae1b6ed9050c4968c9a06e0123b8cfe0600ed88302c0f0452cbda"
)
R6_TRACKED_RECEIPT_RELATIVE_PATH = Path(
    "simulation/path_planning_lab/evidence/"
    "r6-public-end-to-end-qualification-receipt-20260817-64df95f.json"
)

_EXPLICIT_SOURCE_PATHS = (
    "docs/research/dynamic-actor-experiment/26-r7-native-release-gate.md",
    "docs/research/dynamic-actor-experiment/40-r7-stress-conditional-release-policy-2026-08-19.md",
    "docs/research/dynamic-actor-experiment/41-r7-hidden-v4-conditional-evaluator-2026-08-19.md",
    "docs/research/dynamic-actor-experiment/43-r7-hidden-v5-corrective-qualification-2026-08-19.md",
    "simulation/path_planning_lab/pyproject.toml",
    R6_TRACKED_RECEIPT_RELATIVE_PATH.as_posix(),
    "simulation/path_planning_lab/native/dwb_full_core.cpp",
    "simulation/path_planning_lab/native/dwb_full_core.h",
    "simulation/path_planning_lab/native/dwb_safety_core.cpp",
    "simulation/path_planning_lab/native/dwb_safety_core.h",
    "simulation/path_planning_lab/scripts/build_cpp_dwb_full_core.py",
    "simulation/path_planning_lab/scripts/build_cpp_dwb_safety_core.py",
    "simulation/path_planning_lab/scripts/run_r7_hidden_v4.py",
    "simulation/path_planning_lab/scripts/run_r7_hidden_v5.py",
    "simulation/path_planning_lab/scripts/run_r7_native_release_gate.py",
    "simulation/path_planning_lab/tests/test_cpp_dwb_safety_core.py",
    "simulation/path_planning_lab/tests/test_persistent_dwb_adapter.py",
)

_CONTRACT_PARITY_TESTS = (
    "tests/test_cpp_dwb_safety_core.py::test_python_and_cpp_share_the_forbidden_clearance_boundary",
    "tests/test_persistent_dwb_adapter.py::test_terminal_rotation_tie_only_applies_to_stopped_on_section_pre_endpoint_pose",
    "tests/test_persistent_dwb_adapter.py::test_public_goal_gap_stopped_state_still_selects_forward_command",
)
_CONTRACT_PARITY_EXPECTED_TEST_COUNT = 13


def r7_snapshot_cases() -> tuple[tuple[str, ControllerSnapshot, dict[str, object]], ...]:
    """Return the existing frozen five-case public timing set."""

    corpus = tuple((*generate_dynamic_corpus(), *generate_dynamic_v6_public_corpus()))
    cases = _qualification_snapshot_cases(corpus)
    if len(cases) != R7_SNAPSHOT_COUNT:
        raise RuntimeError("R7 snapshot catalog must contain exactly five cases")
    expected = (
        "actor-0-free",
        "actor-1-active",
        "actor-2-active",
        "corner-static-forbidden",
        "staggered-risk-multisegment",
    )
    if tuple(case_id for case_id, _, _ in cases) != expected:
        raise RuntimeError("R7 snapshot catalog identity changed")
    return cases


def retime_controller_snapshot(
    snapshot: ControllerSnapshot,
    *,
    offset: int,
) -> ControllerSnapshot:
    """Create a monotonic fresh-input variant without changing geometry or tracks."""

    if offset < 0:
        raise ValueError("R7 snapshot offset must not be negative")
    frame = snapshot.validated_observation.frame
    if frame is None:
        raise ValueError("R7 timing snapshot requires a delivered observation frame")
    delta_s = offset * DYNAMIC_CONTROL_PERIOD_S
    provisional = replace(
        frame,
        observation_revision=frame.observation_revision + offset,
        sequence=frame.sequence + offset,
        observed_at_s=frame.observed_at_s + delta_s,
        delivered_at_s=frame.delivered_at_s + delta_s,
        content_hash="r7-retime-pending",
    )
    current_frame = replace(
        provisional,
        content_hash=dynamic_observation_content_hash(provisional),
    )
    observation = replace(snapshot.validated_observation, frame=current_frame)
    prediction = build_actor_prediction_set(observation)
    metadata = replace(
        snapshot.static_grid_snapshot.metadata,
        observation_revision=current_frame.observation_revision,
    )
    grid = GridSnapshot(
        metadata=metadata,
        grid=snapshot.static_grid_snapshot.grid,
        forbidden_cells=snapshot.static_grid_snapshot.forbidden_cells,
    )
    return build_controller_snapshot(
        tick_id=snapshot.tick_id + offset,
        simulation_time_s=snapshot.simulation_time_s + delta_s,
        mission_id=snapshot.mission_id,
        robot_state=snapshot.robot_state,
        goal_pose=snapshot.goal_pose,
        reference_path=snapshot.reference_path,
        static_grid_snapshot=grid,
        validated_observation=observation,
        actor_tubes=prediction,
        vehicle_profile=snapshot.vehicle_profile,
    )


def controller_result_semantic_payload(result: ControllerCommandResult) -> dict[str, object]:
    return {
        field: getattr(result, field)
        for field in result.__dataclass_fields__
        if field != "elapsed_ns"
    }


def run_native_parity(
    cases: tuple[tuple[str, ControllerSnapshot, dict[str, object]], ...],
) -> dict[str, object]:
    records: list[dict[str, object]] = []
    for case_id, snapshot, metadata in cases:
        python_controller = SourceDerivedDynamicDwbController(use_cpp_full_core=False)
        native_controller = SourceDerivedDynamicDwbController(use_cpp_full_core=True)
        python_result = python_controller.step(snapshot)
        native_result = native_controller.step(snapshot)
        python_payload = controller_result_semantic_payload(python_result)
        native_payload = controller_result_semantic_payload(native_result)
        python_safety = python_controller.selected_safety_evidence
        native_safety = native_controller.selected_safety_evidence
        native_used = native_controller.native_full_core_used
        passed = bool(
            native_used
            and python_payload == native_payload
            and python_safety == native_safety
        )
        records.append(
            {
                "case_id": case_id,
                "passed": passed,
                "native_full_core_used": native_used,
                "snapshot_content_hash": metadata["snapshot_content_hash"],
                "python_result_hash": canonical_content_hash(python_payload),
                "native_result_hash": canonical_content_hash(native_payload),
                "python_safety_hash": canonical_content_hash(python_safety),
                "native_safety_hash": canonical_content_hash(native_safety),
            }
        )
    return {
        "schema": "r7-native-semantic-parity-v1",
        "passed": all(bool(record["passed"]) for record in records),
        "case_count": len(records),
        "records": records,
        "content_hash": canonical_content_hash(records),
    }


def run_native_timing(
    cases: tuple[tuple[str, ControllerSnapshot, dict[str, object]], ...],
    *,
    warmups: int,
    repeats: int,
) -> dict[str, object]:
    if warmups < 0 or repeats <= 0:
        raise ValueError("R7 timing counts must be positive")
    children_before = tuple(
        child.pid for child in active_children() if child.pid is not None
    )
    if children_before:
        raise RuntimeError("R7 timing requires all child workers to be stopped")

    memory_before = _process_memory_snapshot()
    cold_records: list[dict[str, object]] = []
    case_records: list[dict[str, object]] = []
    all_elapsed: list[int] = []
    for case_id, snapshot, metadata in cases:
        cold_controller = SourceDerivedDynamicDwbController(use_cpp_full_core=True)
        cold_started = perf_counter_ns()
        cold_result = cold_controller.step(snapshot)
        cold_elapsed = perf_counter_ns() - cold_started
        if not cold_controller.native_full_core_used:
            raise RuntimeError(f"R7 cold call did not use native core: {case_id}")
        cold_records.append(
            {
                "case_id": case_id,
                "elapsed_ns": cold_elapsed,
                "result_hash": canonical_content_hash(
                    controller_result_semantic_payload(cold_result)
                ),
            }
        )

        variants = tuple(
            retime_controller_snapshot(snapshot, offset=index)
            for index in range(warmups + repeats)
        )
        controller = SourceDerivedDynamicDwbController(use_cpp_full_core=True)
        for variant in variants[:warmups]:
            controller.step(variant)
            if not controller.native_full_core_used:
                raise RuntimeError(f"R7 warm-up did not use native core: {case_id}")
        elapsed: list[int] = []
        for variant in variants[warmups:]:
            started = perf_counter_ns()
            controller.step(variant)
            duration = perf_counter_ns() - started
            if not controller.native_full_core_used:
                raise RuntimeError(f"R7 measured call did not use native core: {case_id}")
            elapsed.append(duration)
        all_elapsed.extend(elapsed)
        case_records.append(
            {
                "case_id": case_id,
                "snapshot_content_hash": metadata["snapshot_content_hash"],
                **_timing_statistics(elapsed),
            }
        )

    children_after = tuple(
        child.pid for child in active_children() if child.pid is not None
    )
    if children_after:
        raise RuntimeError("R7 timing left active child workers")
    memory_after = _process_memory_snapshot()
    aggregate = _timing_statistics(all_elapsed)
    passed = bool(
        len(all_elapsed) == len(cases) * repeats
        and aggregate["deadline_miss_count"] == 0
        and aggregate["maximum_ns"] <= R7_DEADLINE_NS
    )
    return {
        "schema": R7_NATIVE_QUALIFICATION_VERSION,
        "passed": passed,
        "execution_mode": "serial_parent_no_worker",
        "parallelized": False,
        "clock": "time.perf_counter_ns",
        "deadline_ns": R7_DEADLINE_NS,
        "warmups_per_case": warmups,
        "repeats_per_case": repeats,
        "sample_count": len(all_elapsed),
        "active_child_process_ids_before": list(children_before),
        "active_child_process_ids_after": list(children_after),
        "process_affinity": list(_process_affinity()),
        "numeric_thread_environment": _numeric_thread_environment(),
        "cold_start_is_degradation_only": True,
        "cold_start_records": cold_records,
        "cold_start_maximum_ns": max(record["elapsed_ns"] for record in cold_records),
        "cases": case_records,
        "aggregate": aggregate,
        "memory_before": memory_before,
        "memory_after": memory_after,
        "power_scheme": _power_scheme(),
        "background_load_policy": (
            "runner starts no workers; operating-system background tasks are not controlled"
        ),
        "snapshot_set_hash": canonical_content_hash(
            tuple(
                (case_id, metadata["snapshot_content_hash"], metadata)
                for case_id, _, metadata in cases
            )
        ),
    }


def validate_r6_receipt(repository_root: Path, receipt_path: Path) -> dict[str, object]:
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    embedded_hash = receipt.get("receipt_content_hash")
    payload = {key: value for key, value in receipt.items() if key != "receipt_content_hash"}
    computed_hash = canonical_content_hash(payload)
    head = receipt.get("head")
    ancestry = False
    if isinstance(head, str) and head:
        ancestry = (
            subprocess.run(
                ["git", "merge-base", "--is-ancestor", head, "HEAD"],
                cwd=repository_root,
                check=False,
            ).returncode
            == 0
        )
    checks = {
        "schema": receipt.get("schema") == R6_RECEIPT_SCHEMA,
        "receipt_hash": embedded_hash == computed_hash == R6_EXPECTED_RECEIPT_HASH,
        "result_hash": receipt.get("result_set_hash") == R6_EXPECTED_RESULT_HASH,
        "case_count": receipt.get("required_case_count") == R6_REQUIRED_CASE_COUNT,
        "hard_failure_zero": receipt.get("hard_failure_count") == 0,
        "hidden_not_executed": receipt.get("hidden_executed") is False,
        "wall_clock_not_reused": receipt.get("wall_clock_is_qualification") is False,
        "head_is_ancestor": ancestry,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "receipt_path": str(receipt_path.resolve()),
        "receipt_content_hash": embedded_hash,
        "r6_head": head,
        "r6_tree": receipt.get("tree"),
        "result_set_hash": receipt.get("result_set_hash"),
    }


def source_freeze(repository_root: Path) -> dict[str, object]:
    records = []
    for relative in _source_paths(repository_root):
        path = repository_root / relative
        if not path.is_file():
            raise FileNotFoundError(f"R7 source freeze input missing: {relative}")
        records.append(
            {
                "path": relative,
                "size": path.stat().st_size,
                "sha256": sha256(path.read_bytes()).hexdigest(),
            }
        )
    return {
        "records": records,
        "content_hash": canonical_content_hash(records),
    }


def run_native_contract_parity(lab_root: Path) -> dict[str, object]:
    """Run the frozen forbidden-boundary and terminal-tie parity tests."""

    completed = subprocess.run(
        (
            sys.executable,
            "-m",
            "pytest",
            "-q",
            "--disable-warnings",
            *_CONTRACT_PARITY_TESTS,
        ),
        cwd=lab_root,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    combined = "\n".join(
        part.strip() for part in (completed.stdout, completed.stderr) if part.strip()
    )
    matches = re.findall(r"(\d+) passed", combined)
    passed_count = int(matches[-1]) if matches else 0
    payload = {
        "schema": "r7-native-contract-parity-v1",
        "passed": bool(
            completed.returncode == 0
            and passed_count == _CONTRACT_PARITY_EXPECTED_TEST_COUNT
        ),
        "return_code": completed.returncode,
        "expected_test_count": _CONTRACT_PARITY_EXPECTED_TEST_COUNT,
        "passed_test_count": passed_count,
        "test_node_ids": _CONTRACT_PARITY_TESTS,
        "output": combined,
    }
    return {**payload, "content_hash": canonical_content_hash(payload)}


def _source_paths(repository_root: Path) -> tuple[str, ...]:
    source_root = (
        repository_root / "simulation/path_planning_lab/src/hospital_path_lab"
    )
    python_sources = tuple(
        path.relative_to(repository_root).as_posix()
        for path in source_root.rglob("*.py")
        if "__pycache__" not in path.parts
    )
    return tuple(sorted(set((*_EXPLICIT_SOURCE_PATHS, *python_sources))))


def native_build_metadata(repository_root: Path) -> dict[str, object]:
    lab = repository_root / "simulation/path_planning_lab"
    zig_spec = find_spec("ziglang")
    zig_path: Path | None = None
    if zig_spec is not None and zig_spec.submodule_search_locations is not None:
        candidate = Path(next(iter(zig_spec.submodule_search_locations))) / (
            "zig.exe" if sys.platform == "win32" else "zig"
        )
        if candidate.is_file():
            zig_path = candidate
    compiler_version = None
    if zig_path is not None:
        result = subprocess.run(
            [str(zig_path), "version"],
            capture_output=True,
            text=True,
            check=False,
        )
        compiler_version = result.stdout.strip() or result.stderr.strip()
    try:
        package_version = version("ziglang")
    except PackageNotFoundError:
        package_version = None
    full_core = _native_component_metadata(
        lab,
        stem="dwb_full_core",
        flags=(
            "-std=c++20",
            "-O3",
            "-ffp-contract=off",
            "-fno-builtin-sin",
            "-fno-builtin-cos",
            "-Wno-nullability-completeness",
            "-shared",
        ),
    )
    safety_core = _native_component_metadata(
        lab,
        stem="dwb_safety_core",
        flags=(
            "-std=c++20",
            "-O3",
            "-Wno-nullability-completeness",
            "-shared",
        ),
    )
    return {
        "compiler": None if zig_path is None else str(zig_path.resolve()),
        "compiler_version": compiler_version,
        "ziglang_package_version": package_version,
        "language_standard": "c++20",
        "build_type": "O3",
        "full_core": full_core,
        "safety_core": safety_core,
    }


def _native_component_metadata(
    lab: Path,
    *,
    stem: str,
    flags: tuple[str, ...],
) -> dict[str, object]:
    source = lab / f"native/{stem}.cpp"
    header = lab / f"native/{stem}.h"
    suffix = ".dll" if sys.platform == "win32" else ".dylib" if sys.platform == "darwin" else ".so"
    prefix = "" if sys.platform == "win32" else "lib"
    library = lab / f"src/hospital_path_lab/_native/{prefix}{stem}{suffix}"
    return {
        "flags": list(flags),
        "source_sha256": _file_sha256(source),
        "header_sha256": _file_sha256(header),
        "library_path": str(library.resolve()),
        "library_size": library.stat().st_size,
        "library_sha256": _file_sha256(library),
    }


def machine_metadata() -> dict[str, object]:
    return {
        "machine_name": platform.node(),
        "platform": platform.platform(),
        "os_version": platform.version(),
        "python": sys.version,
        "processor": _processor_name(),
        "physical_core_count": _physical_core_count(),
        "logical_core_count": os.cpu_count(),
        "process_affinity": list(_process_affinity()),
    }


def git_metadata(repository_root: Path) -> dict[str, object]:
    return {
        "head": _git(repository_root, "rev-parse", "HEAD"),
        "tree": _git(repository_root, "rev-parse", "HEAD^{tree}"),
        "branch": _git(repository_root, "branch", "--show-current"),
        "status_porcelain": _git(
            repository_root,
            "status",
            "--porcelain=v1",
            "--untracked-files=all",
        ),
    }


def _timing_statistics(values: list[int]) -> dict[str, object]:
    if not values:
        raise ValueError("R7 timing statistics require samples")
    return {
        "sample_count": len(values),
        "p50_ns": _percentile(values, 0.50),
        "p95_ns": _percentile(values, 0.95),
        "p99_ns": _percentile(values, 0.99),
        "maximum_ns": max(values),
        "deadline_ns": R7_DEADLINE_NS,
        "deadline_miss_count": sum(value > R7_DEADLINE_NS for value in values),
    }


def _percentile(values: list[int], quantile: float) -> int:
    ordered = sorted(values)
    rank = (len(ordered) - 1) * quantile
    lower = floor(rank)
    upper = ceil(rank)
    if lower == upper:
        return ordered[lower]
    fraction = rank - lower
    return int(round(ordered[lower] + (ordered[upper] - ordered[lower]) * fraction))


def _file_sha256(path: Path) -> str:
    if not path.is_file():
        raise FileNotFoundError(path)
    return sha256(path.read_bytes()).hexdigest()


def _git(repository_root: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=repository_root,
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def _numeric_thread_environment() -> dict[str, str | None]:
    return {
        name: os.environ.get(name)
        for name in (
            "OMP_NUM_THREADS",
            "OPENBLAS_NUM_THREADS",
            "MKL_NUM_THREADS",
            "NUMEXPR_NUM_THREADS",
        )
    }


def _power_scheme() -> str | None:
    if sys.platform != "win32":
        return None
    result = subprocess.run(
        ["powercfg", "/GETACTIVESCHEME"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    value = (result.stdout or result.stderr).strip()
    return value or None


def _processor_name() -> str:
    if sys.platform == "win32":
        try:
            import winreg

            with winreg.OpenKey(
                winreg.HKEY_LOCAL_MACHINE,
                r"HARDWARE\DESCRIPTION\System\CentralProcessor\0",
            ) as key:
                return str(winreg.QueryValueEx(key, "ProcessorNameString")[0]).strip()
        except OSError:
            pass
    return platform.processor() or os.environ.get("PROCESSOR_IDENTIFIER", "unknown")


def _physical_core_count() -> int | None:
    if sys.platform != "win32":
        return None
    try:
        class _Core(ctypes.Structure):
            _fields_ = [("flags", ctypes.c_ubyte)]

        class _InfoUnion(ctypes.Union):
            _fields_ = [("core", _Core), ("reserved", ctypes.c_ulonglong * 2)]

        class _Info(ctypes.Structure):
            _anonymous_ = ("detail",)
            _fields_ = [
                ("processor_mask", ctypes.c_size_t),
                ("relationship", ctypes.c_int),
                ("detail", _InfoUnion),
            ]

        length = ctypes.c_ulong(0)
        kernel32 = ctypes.windll.kernel32
        kernel32.GetLogicalProcessorInformation(None, ctypes.byref(length))
        count = length.value // ctypes.sizeof(_Info)
        array = (_Info * count)()
        if not kernel32.GetLogicalProcessorInformation(array, ctypes.byref(length)):
            return None
        return sum(item.relationship == 0 for item in array)
    except (AttributeError, OSError, ValueError):
        return None


def _process_affinity() -> tuple[int, ...]:
    get_affinity = getattr(os, "sched_getaffinity", None)
    if get_affinity is not None:
        return tuple(sorted(get_affinity(0)))
    if sys.platform == "win32":
        try:
            process_mask = ctypes.c_size_t()
            system_mask = ctypes.c_size_t()
            kernel32 = ctypes.windll.kernel32
            handle = kernel32.GetCurrentProcess()
            if kernel32.GetProcessAffinityMask(
                handle,
                ctypes.byref(process_mask),
                ctypes.byref(system_mask),
            ):
                return tuple(
                    index
                    for index in range(ctypes.sizeof(ctypes.c_size_t) * 8)
                    if process_mask.value & (1 << index)
                )
        except (AttributeError, OSError):
            pass
    return tuple(range(os.cpu_count() or 1))


def _process_memory_snapshot() -> dict[str, int | None]:
    if sys.platform.startswith("linux"):
        try:
            import resource

            page_size = os.sysconf("SC_PAGE_SIZE")
            statm = Path("/proc/self/statm").read_text(encoding="ascii").split()
            usage = resource.getrusage(resource.RUSAGE_SELF)
            return {
                "working_set_bytes": int(statm[1]) * page_size,
                "peak_working_set_bytes": int(usage.ru_maxrss) * 1024,
                "private_usage_bytes": None,
                "page_fault_count": int(usage.ru_minflt + usage.ru_majflt),
            }
        except (IndexError, OSError, ValueError):
            pass
    if sys.platform != "win32":
        return {
            "working_set_bytes": None,
            "peak_working_set_bytes": None,
            "private_usage_bytes": None,
            "page_fault_count": None,
        }
    try:
        class _Counters(ctypes.Structure):
            _fields_ = [
                ("cb", ctypes.c_ulong),
                ("page_fault_count", ctypes.c_ulong),
                ("peak_working_set_size", ctypes.c_size_t),
                ("working_set_size", ctypes.c_size_t),
                ("quota_peak_paged_pool_usage", ctypes.c_size_t),
                ("quota_paged_pool_usage", ctypes.c_size_t),
                ("quota_peak_non_paged_pool_usage", ctypes.c_size_t),
                ("quota_non_paged_pool_usage", ctypes.c_size_t),
                ("pagefile_usage", ctypes.c_size_t),
                ("peak_pagefile_usage", ctypes.c_size_t),
                ("private_usage", ctypes.c_size_t),
            ]

        from ctypes import wintypes

        kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        psapi = ctypes.WinDLL("psapi", use_last_error=True)
        kernel32.GetCurrentProcess.restype = wintypes.HANDLE
        psapi.GetProcessMemoryInfo.argtypes = (
            wintypes.HANDLE,
            ctypes.POINTER(_Counters),
            wintypes.DWORD,
        )
        psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
        counters = _Counters()
        counters.cb = ctypes.sizeof(counters)
        handle = kernel32.GetCurrentProcess()
        if not psapi.GetProcessMemoryInfo(
            handle,
            ctypes.byref(counters),
            counters.cb,
        ):
            raise OSError("GetProcessMemoryInfo failed")
        return {
            "working_set_bytes": int(counters.working_set_size),
            "peak_working_set_bytes": int(counters.peak_working_set_size),
            "private_usage_bytes": int(counters.private_usage),
            "page_fault_count": int(counters.page_fault_count),
        }
    except (AttributeError, OSError):
        return {
            "working_set_bytes": None,
            "peak_working_set_bytes": None,
            "private_usage_bytes": None,
            "page_fault_count": None,
        }


__all__ = [
    "R7_DEADLINE_NS",
    "R7_GATE_SCHEMA",
    "R7_NATIVE_QUALIFICATION_VERSION",
    "R7_RECEIPT_SCHEMA",
    "R7_STANDARD_REPEATS",
    "R7_STANDARD_WARMUPS",
    "controller_result_semantic_payload",
    "git_metadata",
    "machine_metadata",
    "native_build_metadata",
    "r7_snapshot_cases",
    "retime_controller_snapshot",
    "run_native_contract_parity",
    "run_native_parity",
    "run_native_timing",
    "source_freeze",
    "validate_r6_receipt",
]
