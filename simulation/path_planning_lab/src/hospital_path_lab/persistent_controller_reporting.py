"""R5-A persistent RPP/DWB 공개 비교 runner와 증거 writer.

R4 public catalog 21개를 같은 순서로 소비한다. reference가 준비된 8개 case는
한 worker에서 fresh RPP와 fresh DWB를 같은 immutable 입력으로 순차 실행하고,
나머지 13개 case는 controller를 호출하지 않는다. Python wall-clock과 worker
정보는 비판정 metadata이며 hidden 또는 제품 알고리즘 선택은 이 모듈 범위가 아니다.
"""

from __future__ import annotations

import json
import os
import subprocess
from collections.abc import Callable, Sequence
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, is_dataclass
from enum import StrEnum
from hashlib import sha256
from math import atan2, ceil, cos, hypot, isfinite, sin, sqrt
from pathlib import Path
from time import perf_counter_ns

import numpy as np

from hospital_path_lab.contracts import Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicMotionState,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import ActorPredictionSet
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PERSISTENT_DWB_ADAPTER_VERSION,
    PersistentSourceDerivedDwbController,
)
from hospital_path_lab.local_reference_contracts import (
    LocalManeuverReference,
    ReferenceBuildStatus,
    ReferenceKnot,
    ReferenceSection,
    ReferenceSectionKind,
    ReferenceTravelDirection,
)
from hospital_path_lab.local_reference_reporting import (
    LOCAL_REFERENCE_PUBLIC_CASE_COUNT,
    LocalReferencePublicCase,
    LocalReferencePublicCaseResult,
    evaluate_local_reference_public_case,
    public_local_reference_cases,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.persistent_controller_contracts import (
    PersistentControllerSessionTransition,
    PersistentControllerStatus,
    ReferenceExecutorState,
)
from hospital_path_lab.persistent_controller_pipeline import (
    PersistentController,
    PersistentControllerPipeline,
    PersistentPipelineStep,
)
from hospital_path_lab.persistent_rpp_controller import (
    PERSISTENT_RPP_CONTROLLER_VERSION,
    PersistentRppController,
)

PERSISTENT_PUBLIC_CATALOG_VERSION = "persistent-controller-public-catalog-v2"
PERSISTENT_PUBLIC_REPORT_VERSION = "persistent-controller-public-report-v2"
PERSISTENT_PUBLIC_MANIFEST_VERSION = "persistent-controller-public-manifest-v2"
PERSISTENT_PUBLIC_RECEIPT_VERSION = "persistent-controller-public-receipt-v2"
PERSISTENT_PUBLIC_RUNNER_VERSION = "persistent-controller-public-runner-v2"
PERSISTENT_PUBLIC_READY_CASE_COUNT = 8
PERSISTENT_PUBLIC_CONTROLLER_COUNT = 2
R4_PUBLIC_AUDIT_SEMANTIC_HASH = (
    "0f7452784da87d6f308477ad7261dd4f0f674e64e031ecf192e72ce4211246ad"
)
R4_PUBLIC_RECEIPT_CONTENT_HASH = (
    "45934d93ce1b02db12ee5c5ba573b450813c0b46e604327245be778d9d51bc86"
)
PERSISTENT_PUBLIC_TARGET_SPEED_MPS = 0.20
PERSISTENT_PUBLIC_MINIMUM_TIMEOUT_S = 30.0
PERSISTENT_PUBLIC_TIMEOUT_MULTIPLIER = 2.5
PERSISTENT_PUBLIC_TIMEOUT_MARGIN_S = 10.0
PERSISTENT_PUBLIC_TRACKING_ERROR_LIMIT_M = 0.10
PERSISTENT_PUBLIC_DEADLOCK_WINDOW_S = 3.0
PERSISTENT_PUBLIC_DEADLOCK_PROGRESS_M = 0.02
_TOLERANCE = 1e-12
_SHA256_LENGTH = 64
_TRANSLATION_SECTION_KINDS = frozenset(
    {
        ReferenceSectionKind.FOLLOW_ORIGINAL,
        ReferenceSectionKind.DEPART,
        ReferenceSectionKind.BYPASS,
        ReferenceSectionKind.RETURN,
        ReferenceSectionKind.REJOIN,
    }
)


class PersistentPublicRunStatus(StrEnum):
    COMPLETED = "completed"
    TIMED_OUT = "timed_out"
    HARD_FAILED = "hard_failed"


@dataclass(frozen=True, slots=True)
class PersistentPublicTraceSample:
    tick_id: int
    simulation_time_s: float
    pose_before: Pose2D
    pose_after: Pose2D
    requested_twist: Twist2D
    applied_twist: Twist2D
    controller_status: PersistentControllerStatus
    motion_state: DynamicMotionState
    executor_state: str
    session_transition: PersistentControllerSessionTransition
    active_section_index: int | None
    active_section_kind: str | None
    active_travel_direction: str | None
    tracking_error_m: float | None
    minimum_static_clearance_m: float | None
    controller_failure_reason: str | None
    decision_trace: tuple[str, ...]
    candidate_diagnostics: tuple[str, ...]
    gate_failure_reasons: tuple[str, ...]
    semantic_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.tick_id < 0 or not isfinite(self.simulation_time_s):
            raise ValueError("trace tick/time must be finite and non-negative")
        object.__setattr__(self, "gate_failure_reasons", tuple(self.gate_failure_reasons))
        object.__setattr__(self, "decision_trace", tuple(self.decision_trace))
        object.__setattr__(self, "candidate_diagnostics", tuple(self.candidate_diagnostics))
        _bind_hash(self, "semantic_content_hash", self.expected_content_hash)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "tick_id": self.tick_id,
                "simulation_time_s": self.simulation_time_s,
                "pose_before": self.pose_before,
                "pose_after": self.pose_after,
                "requested_twist": self.requested_twist,
                "applied_twist": self.applied_twist,
                "controller_status": self.controller_status,
                "motion_state": self.motion_state,
                "executor_state": self.executor_state,
                "session_transition": self.session_transition,
                "active_section_index": self.active_section_index,
                "active_section_kind": self.active_section_kind,
                "active_travel_direction": self.active_travel_direction,
                "tracking_error_m": self.tracking_error_m,
                "minimum_static_clearance_m": self.minimum_static_clearance_m,
                "controller_failure_reason": self.controller_failure_reason,
                "decision_trace": self.decision_trace,
                "candidate_diagnostics": self.candidate_diagnostics,
                "gate_failure_reasons": self.gate_failure_reasons,
            }
        )


@dataclass(frozen=True, slots=True)
class PersistentPublicRunMetrics:
    tick_count: int
    completion_tick: int | None
    completion_simulation_time_s: float | None
    actual_path_length_m: float
    maximum_tracking_error_m: float
    rms_tracking_error_m: float
    minimum_static_clearance_m: float | None
    longitudinal_jerk_rms_mps3: float
    angular_acceleration_rms_radps2: float
    angular_jerk_rms_radps3: float
    peak_angular_velocity_radps: float
    direction_reversal_count: int
    initial_bind_count: int
    same_session_reset_count: int
    window_advance_count: int
    planned_stop_count: int
    controller_stop_request_count: int
    no_safe_command_count: int
    gate_override_count: int
    gate_rejection_count: int
    late_result_count: int
    deadlock_count: int

    def __post_init__(self) -> None:
        integer_fields = (
            self.tick_count,
            self.direction_reversal_count,
            self.initial_bind_count,
            self.same_session_reset_count,
            self.window_advance_count,
            self.planned_stop_count,
            self.controller_stop_request_count,
            self.no_safe_command_count,
            self.gate_override_count,
            self.gate_rejection_count,
            self.late_result_count,
            self.deadlock_count,
        )
        if any(value < 0 for value in integer_fields):
            raise ValueError("run metrics counters must be non-negative")
        if self.completion_tick is not None and self.completion_tick < 0:
            raise ValueError("completion tick must be non-negative")
        floats = (
            self.actual_path_length_m,
            self.maximum_tracking_error_m,
            self.rms_tracking_error_m,
            self.longitudinal_jerk_rms_mps3,
            self.angular_acceleration_rms_radps2,
            self.angular_jerk_rms_radps3,
            self.peak_angular_velocity_radps,
        )
        if any(not isfinite(value) or value < 0.0 for value in floats):
            raise ValueError("run metrics must be finite and non-negative")
        if self.minimum_static_clearance_m is not None and not isfinite(
            self.minimum_static_clearance_m
        ):
            raise ValueError("minimum clearance must be finite or None")


@dataclass(frozen=True, slots=True)
class PersistentPublicControllerRun:
    controller_name: str
    controller_version: str
    reference_session_id: str
    candidate_id: str
    paired_input_hash: str
    status: PersistentPublicRunStatus
    metrics: PersistentPublicRunMetrics
    samples: tuple[PersistentPublicTraceSample, ...]
    section_sequence: tuple[tuple[int, str], ...]
    hard_failures: tuple[str, ...]
    elapsed_nonqualification_ns: int
    semantic_content_hash: str = ""

    def __post_init__(self) -> None:
        for value in (
            self.controller_name,
            self.controller_version,
            self.reference_session_id,
            self.candidate_id,
        ):
            if not value:
                raise ValueError("controller run identity must not be empty")
        _require_sha256(self.paired_input_hash, "paired_input_hash")
        if not isinstance(self.status, PersistentPublicRunStatus):
            raise TypeError("status must be PersistentPublicRunStatus")
        object.__setattr__(self, "samples", tuple(self.samples))
        object.__setattr__(self, "section_sequence", tuple(self.section_sequence))
        object.__setattr__(self, "hard_failures", tuple(sorted(set(self.hard_failures))))
        if self.elapsed_nonqualification_ns < 0:
            raise ValueError("elapsed time must be non-negative")
        if self.metrics.tick_count != len(self.samples):
            raise ValueError("tick count must match trace sample count")
        _bind_hash(self, "semantic_content_hash", self.expected_content_hash)

    @property
    def hard_passed(self) -> bool:
        return self.status is PersistentPublicRunStatus.COMPLETED and not self.hard_failures

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "controller_name": self.controller_name,
                "controller_version": self.controller_version,
                "reference_session_id": self.reference_session_id,
                "candidate_id": self.candidate_id,
                "paired_input_hash": self.paired_input_hash,
                "status": self.status,
                "metrics": self.metrics,
                "sample_hashes": tuple(item.semantic_content_hash for item in self.samples),
                "section_sequence": self.section_sequence,
                "hard_failures": self.hard_failures,
            }
        )


@dataclass(frozen=True, slots=True)
class PersistentPublicPairDelta:
    completion_simulation_time_s: float | None
    maximum_tracking_error_m: float
    actual_path_length_m: float
    longitudinal_jerk_rms_mps3: float
    angular_acceleration_rms_radps2: float
    angular_jerk_rms_radps3: float
    gate_override_count: int
    no_safe_command_count: int


@dataclass(frozen=True, slots=True)
class PersistentPublicCaseResult:
    ordinal: int
    public_id: str
    case_content_hash: str
    r4_report_content_hash: str
    reference_status: ReferenceBuildStatus
    reference_path: tuple[Pose2D, ...]
    reference_sections: tuple[tuple[int, str], ...]
    paired_input_hash: str | None
    rpp_result: PersistentPublicControllerRun | None
    dwb_result: PersistentPublicControllerRun | None
    pair_delta_dwb_minus_rpp: PersistentPublicPairDelta | None
    controller_call_count: int
    worker_pid_nonqualification: int
    hard_failures: tuple[str, ...]
    semantic_content_hash: str = ""

    def __post_init__(self) -> None:
        _require_sha256(self.case_content_hash, "case_content_hash")
        _require_sha256(self.r4_report_content_hash, "r4_report_content_hash")
        if self.paired_input_hash is not None:
            _require_sha256(self.paired_input_hash, "paired_input_hash")
        object.__setattr__(self, "reference_path", tuple(self.reference_path))
        object.__setattr__(self, "reference_sections", tuple(self.reference_sections))
        object.__setattr__(self, "hard_failures", tuple(sorted(set(self.hard_failures))))
        if self.controller_call_count < 0 or self.worker_pid_nonqualification <= 0:
            raise ValueError("case counters/worker pid are invalid")
        ready = self.reference_status is ReferenceBuildStatus.REFERENCE_SET_READY
        if ready != (self.rpp_result is not None and self.dwb_result is not None):
            raise ValueError("ready status and paired run presence differ")
        if ready != (self.paired_input_hash is not None):
            raise ValueError("ready status and paired input presence differ")
        if ready != (self.pair_delta_dwb_minus_rpp is not None):
            raise ValueError("ready status and pair delta presence differ")
        if not ready and self.controller_call_count != 0:
            raise ValueError("non-ready case must not call a controller")
        _bind_hash(self, "semantic_content_hash", self.expected_content_hash)

    @property
    def hard_passed(self) -> bool:
        return not self.hard_failures

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "ordinal": self.ordinal,
                "public_id": self.public_id,
                "case_content_hash": self.case_content_hash,
                "r4_report_content_hash": self.r4_report_content_hash,
                "reference_status": self.reference_status,
                "reference_path": self.reference_path,
                "reference_sections": self.reference_sections,
                "paired_input_hash": self.paired_input_hash,
                "rpp_result_hash": (
                    None if self.rpp_result is None else self.rpp_result.semantic_content_hash
                ),
                "dwb_result_hash": (
                    None if self.dwb_result is None else self.dwb_result.semantic_content_hash
                ),
                "pair_delta_dwb_minus_rpp": self.pair_delta_dwb_minus_rpp,
                "controller_call_count": self.controller_call_count,
                "hard_failures": self.hard_failures,
            }
        )


@dataclass(frozen=True, slots=True)
class PersistentPublicAudit:
    report_version: str
    simulation_only: bool
    hidden_used: bool
    catalog_content_hash: str
    case_results: tuple[PersistentPublicCaseResult, ...]
    relation_failures: tuple[str, ...]
    parity_case_id: str
    serial_process_parity_passed: bool
    repeat_determinism_passed: bool
    hard_failures: tuple[str, ...]
    limitations: tuple[str, ...]
    semantic_content_hash: str
    elapsed_nonqualification_ns: int

    def __post_init__(self) -> None:
        if self.report_version != PERSISTENT_PUBLIC_REPORT_VERSION:
            raise ValueError("unsupported persistent public report version")
        if not self.simulation_only or self.hidden_used:
            raise ValueError("R5-A audit must remain simulation-only and hidden-free")
        _require_sha256(self.catalog_content_hash, "catalog_content_hash")
        _require_sha256(self.semantic_content_hash, "semantic_content_hash")
        object.__setattr__(self, "case_results", tuple(self.case_results))
        object.__setattr__(self, "relation_failures", tuple(sorted(set(self.relation_failures))))
        object.__setattr__(self, "hard_failures", tuple(sorted(set(self.hard_failures))))
        object.__setattr__(self, "limitations", tuple(self.limitations))
        if len(self.case_results) != LOCAL_REFERENCE_PUBLIC_CASE_COUNT:
            raise ValueError("R5-A audit requires all 21 R4 public cases")
        if tuple(item.ordinal for item in self.case_results) != tuple(
            range(LOCAL_REFERENCE_PUBLIC_CASE_COUNT)
        ):
            raise ValueError("R5-A case order must be complete and contiguous")
        if self.semantic_content_hash != self.expected_content_hash:
            raise ValueError("R5-A audit semantic hash mismatch")

    @property
    def hard_passed(self) -> bool:
        return not self.hard_failures

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "report_version": self.report_version,
                "simulation_only": self.simulation_only,
                "hidden_used": self.hidden_used,
                "catalog_content_hash": self.catalog_content_hash,
                "case_hashes": tuple(item.semantic_content_hash for item in self.case_results),
                "relation_failures": self.relation_failures,
                "parity_case_id": self.parity_case_id,
                "serial_process_parity_passed": self.serial_process_parity_passed,
                "repeat_determinism_passed": self.repeat_determinism_passed,
                "hard_failures": self.hard_failures,
                "limitations": self.limitations,
            }
        )


@dataclass(frozen=True, slots=True)
class PersistentPublicManifest:
    manifest_version: str
    simulation_only: bool
    hidden_used: bool
    git_head: str
    git_tree: str
    git_dirty: bool
    source_freeze_hash: str
    r4_catalog_content_hash: str
    r4_audit_semantic_hash: str
    r4_receipt_content_hash: str
    case_order: tuple[tuple[str, str], ...]
    contract_hash: str
    controller_config_hash: str
    public_case_limit: int | None
    tick_limit_override: int | None
    max_workers_nonsemantic: int
    logical_cpu_count_nonsemantic: int
    semantic_content_hash: str
    content_hash: str

    def __post_init__(self) -> None:
        if self.manifest_version != PERSISTENT_PUBLIC_MANIFEST_VERSION:
            raise ValueError("unsupported R5-A manifest version")
        for name in (
            "source_freeze_hash",
            "r4_catalog_content_hash",
            "r4_audit_semantic_hash",
            "r4_receipt_content_hash",
            "contract_hash",
            "controller_config_hash",
            "semantic_content_hash",
            "content_hash",
        ):
            _require_sha256(getattr(self, name), name)
        if self.public_case_limit is not None and self.public_case_limit <= 0:
            raise ValueError("public case limit must be positive")
        if self.tick_limit_override is not None and self.tick_limit_override <= 0:
            raise ValueError("tick limit override must be positive")
        if self.max_workers_nonsemantic <= 0 or self.logical_cpu_count_nonsemantic <= 0:
            raise ValueError("manifest worker counts must be positive")
        if self.semantic_content_hash != self.expected_semantic_hash:
            raise ValueError("manifest semantic hash mismatch")
        if self.content_hash != self.expected_content_hash:
            raise ValueError("manifest content hash mismatch")

    @property
    def sealing_run(self) -> bool:
        return self.public_case_limit is None and self.tick_limit_override is None

    @property
    def expected_semantic_hash(self) -> str:
        return canonical_content_hash(
            {
                "manifest_version": self.manifest_version,
                "simulation_only": self.simulation_only,
                "hidden_used": self.hidden_used,
                "source_freeze_hash": self.source_freeze_hash,
                "r4_catalog_content_hash": self.r4_catalog_content_hash,
                "r4_audit_semantic_hash": self.r4_audit_semantic_hash,
                "r4_receipt_content_hash": self.r4_receipt_content_hash,
                "case_order": self.case_order,
                "contract_hash": self.contract_hash,
                "controller_config_hash": self.controller_config_hash,
                "public_case_limit": self.public_case_limit,
                "tick_limit_override": self.tick_limit_override,
            }
        )

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "semantic_content_hash": self.semantic_content_hash,
                "git_head": self.git_head,
                "git_tree": self.git_tree,
                "git_dirty": self.git_dirty,
                "max_workers_nonsemantic": self.max_workers_nonsemantic,
                "logical_cpu_count_nonsemantic": self.logical_cpu_count_nonsemantic,
            }
        )


def evaluate_persistent_public_case(
    case: LocalReferencePublicCase,
    *,
    tick_limit_override: int | None = None,
) -> PersistentPublicCaseResult:
    """Evaluate one case; a ready pair always executes in this one process."""

    if tick_limit_override is not None and tick_limit_override <= 0:
        raise ValueError("tick_limit_override must be positive")
    started = perf_counter_ns()
    r4_result = evaluate_local_reference_public_case(case)
    failures = [f"r4:{item}" for item in r4_result.hard_failures]
    if r4_result.reference_set.status is not case.expected_build_status:
        failures.append("reference_status_mismatch")

    paired_input_hash: str | None = None
    reference_path: tuple[Pose2D, ...] = ()
    reference_sections: tuple[tuple[int, str], ...] = ()
    rpp: PersistentPublicControllerRun | None = None
    dwb: PersistentPublicControllerRun | None = None
    delta: PersistentPublicPairDelta | None = None
    controller_calls = 0
    if r4_result.reference_set.status is ReferenceBuildStatus.REFERENCE_SET_READY:
        if len(r4_result.reference_set.candidates) != 1 or len(r4_result.validations) != 1:
            failures.append("ready_case_requires_exactly_one_validated_reference")
        else:
            reference = r4_result.reference_set.candidates[0]
            validation = r4_result.validations[0]
            reference_path = tuple(knot.pose for knot in reference.knots)
            reference_sections = tuple(
                (section.section_index, section.section_kind.value)
                for section in reference.sections
            )
            paired_input_hash = canonical_content_hash(
                {
                    "case_hash": case.semantic_content_hash,
                    "build_context_hash": r4_result.build_context.context_content_hash,
                    "reference_hash": reference.reference_content_hash,
                    "validation_hash": validation.validation_content_hash,
                }
            )
            rpp = _run_persistent_controller(
                controller=PersistentRppController(),
                controller_version=PERSISTENT_RPP_CONTROLLER_VERSION,
                r4_result=r4_result,
                paired_input_hash=paired_input_hash,
                tick_limit_override=tick_limit_override,
            )
            dwb = _run_persistent_controller(
                controller=PersistentSourceDerivedDwbController(),
                controller_version=PERSISTENT_DWB_ADAPTER_VERSION,
                r4_result=r4_result,
                paired_input_hash=paired_input_hash,
                tick_limit_override=tick_limit_override,
            )
            controller_calls = rpp.metrics.tick_count + dwb.metrics.tick_count
            delta = _pair_delta(rpp.metrics, dwb.metrics)
            failures.extend(f"rpp:{item}" for item in rpp.hard_failures)
            failures.extend(f"dwb:{item}" for item in dwb.hard_failures)
            if rpp.paired_input_hash != dwb.paired_input_hash:
                failures.append("paired_input_hash_mismatch")
    elif r4_result.reference_set.candidates:
        failures.append("non_ready_case_exposed_reference_candidate")

    result = PersistentPublicCaseResult(
        ordinal=case.ordinal,
        public_id=case.public_id,
        case_content_hash=case.semantic_content_hash,
        r4_report_content_hash=r4_result.report_content_hash,
        reference_status=r4_result.reference_set.status,
        reference_path=reference_path,
        reference_sections=reference_sections,
        paired_input_hash=paired_input_hash,
        rpp_result=rpp,
        dwb_result=dwb,
        pair_delta_dwb_minus_rpp=delta,
        controller_call_count=controller_calls,
        worker_pid_nonqualification=os.getpid(),
        hard_failures=tuple(failures),
    )
    _ = perf_counter_ns() - started
    return result


def evaluate_persistent_public_cases(
    cases: Sequence[LocalReferencePublicCase],
    *,
    max_workers: int,
    tick_limit_override: int | None = None,
    on_case: Callable[[PersistentPublicCaseResult], None] | None = None,
) -> tuple[PersistentPublicCaseResult, ...]:
    if max_workers <= 0:
        raise ValueError("max_workers must be positive")
    materialized: list[PersistentPublicCaseResult] = []
    if max_workers == 1:
        for case in cases:
            result = evaluate_persistent_public_case(
                case,
                tick_limit_override=tick_limit_override,
            )
            materialized.append(result)
            if on_case is not None:
                on_case(result)
    else:
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            futures = {
                executor.submit(
                    evaluate_persistent_public_case,
                    case,
                    tick_limit_override=tick_limit_override,
                ): case.ordinal
                for case in cases
            }
            for future in as_completed(futures):
                result = future.result()
                materialized.append(result)
                if on_case is not None:
                    on_case(result)
    return tuple(sorted(materialized, key=lambda item: item.ordinal))


def audit_persistent_public_catalog(
    *,
    max_workers: int,
    on_case: Callable[[PersistentPublicCaseResult], None] | None = None,
) -> PersistentPublicAudit:
    """Run the complete sealing catalog, including repeat/process parity evidence."""

    cases = public_local_reference_cases()
    started = perf_counter_ns()
    results = evaluate_persistent_public_cases(
        cases,
        max_workers=max_workers,
        on_case=on_case,
    )
    parity_case = next(case for case in cases if case.expected_candidate_count == 1)
    serial_first = evaluate_persistent_public_case(parity_case)
    serial_second = evaluate_persistent_public_case(parity_case)
    process_result = results[parity_case.ordinal]
    parity_passed = serial_first.semantic_content_hash == process_result.semantic_content_hash
    repeat_passed = serial_first.semantic_content_hash == serial_second.semantic_content_hash
    relation_failures = _public_relation_failures(results)
    hard_failures = [
        f"{result.public_id}:{failure}"
        for result in results
        for failure in result.hard_failures
    ]
    hard_failures.extend(relation_failures)
    if not parity_passed:
        hard_failures.append("serial_process_semantic_parity_failed")
    if not repeat_passed:
        hard_failures.append("repeat_semantic_determinism_failed")
    ready_count = sum(
        result.reference_status is ReferenceBuildStatus.REFERENCE_SET_READY
        for result in results
    )
    if ready_count != PERSISTENT_PUBLIC_READY_CASE_COUNT:
        hard_failures.append("ready_case_count_mismatch")
    catalog_hash = _r4_catalog_content_hash(cases)
    limitations = (
        "offline_static_reference_tracking_only",
        "spatial_only_no_actor_temporal_execution",
        "python_wall_clock_is_nonqualification",
        "no_hidden_used",
        "no_product_controller_selection",
        "no_physical_or_human_safety_claim",
    )
    semantic_hash = canonical_content_hash(
        {
            "report_version": PERSISTENT_PUBLIC_REPORT_VERSION,
            "simulation_only": True,
            "hidden_used": False,
            "catalog_content_hash": catalog_hash,
            "case_hashes": tuple(result.semantic_content_hash for result in results),
            "relation_failures": tuple(sorted(set(relation_failures))),
            "parity_case_id": parity_case.public_id,
            "serial_process_parity_passed": parity_passed,
            "repeat_determinism_passed": repeat_passed,
            "hard_failures": tuple(sorted(set(hard_failures))),
            "limitations": limitations,
        }
    )
    return PersistentPublicAudit(
        report_version=PERSISTENT_PUBLIC_REPORT_VERSION,
        simulation_only=True,
        hidden_used=False,
        catalog_content_hash=catalog_hash,
        case_results=results,
        relation_failures=tuple(relation_failures),
        parity_case_id=parity_case.public_id,
        serial_process_parity_passed=parity_passed,
        repeat_determinism_passed=repeat_passed,
        hard_failures=tuple(hard_failures),
        limitations=limitations,
        semantic_content_hash=semantic_hash,
        elapsed_nonqualification_ns=perf_counter_ns() - started,
    )


def build_persistent_public_manifest(
    *,
    repository_root: Path,
    max_workers: int,
    public_case_limit: int | None = None,
    tick_limit_override: int | None = None,
) -> PersistentPublicManifest:
    if max_workers <= 0:
        raise ValueError("max_workers must be positive")
    cases = public_local_reference_cases()
    if public_case_limit is not None:
        if public_case_limit <= 0 or public_case_limit > len(cases):
            raise ValueError("public_case_limit is outside the frozen catalog")
        selected = cases[:public_case_limit]
    else:
        selected = cases
    order = tuple((case.public_id, case.semantic_content_hash) for case in selected)
    source_hash = _source_freeze_hash(repository_root)
    catalog_hash = _r4_catalog_content_hash(cases)
    contract_hash = canonical_content_hash(
        {
            "runner_version": PERSISTENT_PUBLIC_RUNNER_VERSION,
            "r4_catalog_hash": catalog_hash,
            "r4_audit_semantic_hash": R4_PUBLIC_AUDIT_SEMANTIC_HASH,
            "r4_receipt_content_hash": R4_PUBLIC_RECEIPT_CONTENT_HASH,
            "control_period_s": DYNAMIC_CONTROL_PERIOD_S,
            "timeout_formula": (
                PERSISTENT_PUBLIC_MINIMUM_TIMEOUT_S,
                PERSISTENT_PUBLIC_TARGET_SPEED_MPS,
                PERSISTENT_PUBLIC_TIMEOUT_MULTIPLIER,
                PERSISTENT_PUBLIC_TIMEOUT_MARGIN_S,
            ),
            "tracking_error_limit_m": PERSISTENT_PUBLIC_TRACKING_ERROR_LIMIT_M,
            "deadlock": (
                PERSISTENT_PUBLIC_DEADLOCK_WINDOW_S,
                PERSISTENT_PUBLIC_DEADLOCK_PROGRESS_M,
            ),
            "section_bound_translation_mps": {
                "forward": (0.0, 0.30),
                "reverse": (-0.10, 0.0),
                "none": (0.0, 0.0),
            },
            "signed_direction_change_stopped_confirmation_ticks": 3,
        }
    )
    controller_hash = canonical_content_hash(
        {
            "rpp_version": PERSISTENT_RPP_CONTROLLER_VERSION,
            "dwb_version": PERSISTENT_DWB_ADAPTER_VERSION,
            "paired_order": ("rpp", "dwb"),
        }
    )
    head = _git_output(repository_root, "rev-parse", "HEAD")
    tree = _git_output(repository_root, "rev-parse", "HEAD^{tree}")
    dirty = bool(_git_output(repository_root, "status", "--porcelain=v1"))
    semantic_hash = canonical_content_hash(
        {
            "manifest_version": PERSISTENT_PUBLIC_MANIFEST_VERSION,
            "simulation_only": True,
            "hidden_used": False,
            "source_freeze_hash": source_hash,
            "r4_catalog_content_hash": catalog_hash,
            "r4_audit_semantic_hash": R4_PUBLIC_AUDIT_SEMANTIC_HASH,
            "r4_receipt_content_hash": R4_PUBLIC_RECEIPT_CONTENT_HASH,
            "case_order": order,
            "contract_hash": contract_hash,
            "controller_config_hash": controller_hash,
            "public_case_limit": public_case_limit,
            "tick_limit_override": tick_limit_override,
        }
    )
    content_hash = canonical_content_hash(
        {
            "semantic_content_hash": semantic_hash,
            "git_head": head,
            "git_tree": tree,
            "git_dirty": dirty,
            "max_workers_nonsemantic": max_workers,
            "logical_cpu_count_nonsemantic": os.cpu_count() or 1,
        }
    )
    return PersistentPublicManifest(
        manifest_version=PERSISTENT_PUBLIC_MANIFEST_VERSION,
        simulation_only=True,
        hidden_used=False,
        git_head=head,
        git_tree=tree,
        git_dirty=dirty,
        source_freeze_hash=source_hash,
        r4_catalog_content_hash=catalog_hash,
        r4_audit_semantic_hash=R4_PUBLIC_AUDIT_SEMANTIC_HASH,
        r4_receipt_content_hash=R4_PUBLIC_RECEIPT_CONTENT_HASH,
        case_order=order,
        contract_hash=contract_hash,
        controller_config_hash=controller_hash,
        public_case_limit=public_case_limit,
        tick_limit_override=tick_limit_override,
        max_workers_nonsemantic=max_workers,
        logical_cpu_count_nonsemantic=os.cpu_count() or 1,
        semantic_content_hash=semantic_hash,
        content_hash=content_hash,
    )


class PersistentPublicOutputWriter:
    def __init__(
        self,
        output_dir: Path,
        manifest: PersistentPublicManifest,
        *,
        repository_root: Path,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.manifest = manifest
        self.repository_root = Path(repository_root)
        self._written: set[tuple[str, str]] = set()

    def start(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=False)
        _write_exclusive_json(self.output_dir / "run-manifest.json", self.manifest)
        _write_atomic_json(
            self.output_dir / "partial-state.json",
            {
                "partial": True,
                "manifest_content_hash": self.manifest.content_hash,
                "completed_cases": (),
            },
        )

    def write_case(self, result: PersistentPublicCaseResult) -> None:
        key = (result.public_id, result.case_content_hash)
        if key not in self.manifest.case_order:
            raise ValueError("case is outside the frozen R5-A manifest")
        if key in self._written:
            raise FileExistsError("case was already written")
        directory = self.output_dir / "cases" / f"{result.ordinal:02d}-{result.public_id}"
        directory.mkdir(parents=True, exist_ok=False)
        _write_exclusive_json(
            directory / "source-reference.json",
            {
                "public_id": result.public_id,
                "case_content_hash": result.case_content_hash,
                "r4_report_content_hash": result.r4_report_content_hash,
                "reference_status": result.reference_status,
                "reference_path": result.reference_path,
                "reference_sections": result.reference_sections,
                "paired_input_hash": result.paired_input_hash,
            },
        )
        if result.rpp_result is not None:
            _write_exclusive_json(directory / "rpp-result.json", result.rpp_result)
        if result.dwb_result is not None:
            _write_exclusive_json(directory / "dwb-result.json", result.dwb_result)
        _write_exclusive_json(directory / "paired-summary.json", result)
        _save_trajectory_plot(result, directory / "trajectories.png")
        self._written.add(key)
        _write_atomic_json(
            self.output_dir / "partial-state.json",
            {
                "partial": True,
                "manifest_content_hash": self.manifest.content_hash,
                "completed_cases": tuple(sorted(public_id for public_id, _ in self._written)),
            },
        )

    def complete(self, audit: PersistentPublicAudit) -> tuple[Path, Path, Path | None]:
        if not self.manifest.sealing_run:
            raise RuntimeError("limited R5-A runs remain partial/report-only")
        expected = self.manifest.case_order
        actual = tuple((item.public_id, item.case_content_hash) for item in audit.case_results)
        if actual != expected or self._written != set(expected):
            raise RuntimeError("cannot complete before all frozen R5-A cases are written")
        if audit.catalog_content_hash != self.manifest.r4_catalog_content_hash:
            raise ValueError("audit catalog differs from the frozen manifest")
        if _source_freeze_hash(self.repository_root) != self.manifest.source_freeze_hash:
            raise RuntimeError("source changed before R5-A completion")
        summary_json = self.output_dir / "summary.json"
        summary_md = self.output_dir / "summary.md"
        _write_exclusive_json(summary_json, audit)
        _write_exclusive_text(summary_md, _audit_summary(audit))
        _write_exclusive_json(
            self.output_dir / "complete-state.json",
            {
                "partial": False,
                "qualified": audit.hard_passed,
                "manifest_content_hash": self.manifest.content_hash,
                "audit_semantic_content_hash": audit.semantic_content_hash,
                "case_count": len(audit.case_results),
            },
        )
        (self.output_dir / "partial-state.json").unlink()
        receipt_path: Path | None = None
        if audit.hard_passed and not self.manifest.git_dirty:
            self._verify_git_state()
            if _source_freeze_hash(self.repository_root) != self.manifest.source_freeze_hash:
                raise RuntimeError("source changed immediately before R5-A receipt")
            receipt_path = self.output_dir / "qualification-receipt.json"
            receipt: dict[str, object] = {
                "receipt_version": PERSISTENT_PUBLIC_RECEIPT_VERSION,
                "qualified": True,
                "simulation_only": True,
                "hidden_used": False,
                "git_head": self.manifest.git_head,
                "git_tree": self.manifest.git_tree,
                "source_freeze_hash": self.manifest.source_freeze_hash,
                "manifest_content_hash": self.manifest.content_hash,
                "catalog_content_hash": audit.catalog_content_hash,
                "r4_audit_semantic_hash": self.manifest.r4_audit_semantic_hash,
                "r4_receipt_content_hash": self.manifest.r4_receipt_content_hash,
                "audit_semantic_content_hash": audit.semantic_content_hash,
                "case_count": len(audit.case_results),
                "limitations": audit.limitations,
            }
            receipt["receipt_content_hash"] = canonical_content_hash(receipt)
            _write_exclusive_json(receipt_path, receipt)
        return summary_json, summary_md, receipt_path

    def _verify_git_state(self) -> None:
        if _git_output(self.repository_root, "rev-parse", "HEAD") != self.manifest.git_head:
            raise RuntimeError("Git HEAD changed before R5-A receipt")
        if _git_output(self.repository_root, "rev-parse", "HEAD^{tree}") != self.manifest.git_tree:
            raise RuntimeError("Git tree changed before R5-A receipt")
        if _git_output(self.repository_root, "status", "--porcelain=v1"):
            raise RuntimeError("Git worktree became dirty before R5-A receipt")


def _run_persistent_controller(
    *,
    controller: PersistentController,
    controller_version: str,
    r4_result: LocalReferencePublicCaseResult,
    paired_input_hash: str,
    tick_limit_override: int | None,
) -> PersistentPublicControllerRun:
    reference = r4_result.reference_set.candidates[0]
    validation = r4_result.validations[0]
    tick_limit = (
        _episode_tick_limit(reference.knots[-1].cumulative_translation_arc_m)
        if tick_limit_override is None
        else tick_limit_override
    )
    pipeline = PersistentControllerPipeline(
        controller=controller,  # type: ignore[arg-type]
        build_context=r4_result.build_context,
        full_reference=reference,
        validation=validation,
        initial_robot_state=RobotState(reference.knots[0].pose, Twist2D()),
    )
    started = perf_counter_ns()
    records: list[PersistentPipelineStep] = []
    failures: list[str] = []
    deadlock_progress: list[tuple[int, float]] = []
    deadlock_section_index: int | None = None
    for _ in range(tick_limit):
        observation, prediction = _fresh_empty(r4_result, pipeline.tick_id)
        record = pipeline.step(
            observation_snapshot=observation,
            prediction_set=prediction,
        )
        records.append(record)
        result = record.controller_result
        if result is None:
            failures.append("controller_not_called_for_ready_reference")
            break
        section_index = result.active_section_index
        progress = 0.0
        if section_index is not None and 0 <= section_index < len(reference.sections):
            progress = _reference_section_progress_m(
                reference.knots,
                reference.sections[section_index],
                record.robot_state_after.pose,
            )
        if (
            result.status is PersistentControllerStatus.COMMAND_FOUND
            and result.active_section_kind in _TRANSLATION_SECTION_KINDS
            and result.executor_state is ReferenceExecutorState.TRACK_TRANSLATION
            and record.safety_decision.motion_state is DynamicMotionState.MOVING
            and record.safety_decision.primary_hold_reason is None
        ):
            if deadlock_section_index != section_index:
                deadlock_progress.clear()
                deadlock_section_index = section_index
            deadlock_progress.append((record.tick_id, progress))
            minimum_tick = record.tick_id - int(
                round(PERSISTENT_PUBLIC_DEADLOCK_WINDOW_S / DYNAMIC_CONTROL_PERIOD_S)
            )
            deadlock_progress = [item for item in deadlock_progress if item[0] >= minimum_tick]
            if (
                len(deadlock_progress) >= 2
                and deadlock_progress[-1][0] - deadlock_progress[0][0]
                >= int(round(PERSISTENT_PUBLIC_DEADLOCK_WINDOW_S / DYNAMIC_CONTROL_PERIOD_S))
                and _maximum_forward_progress_m(deadlock_progress)
                < PERSISTENT_PUBLIC_DEADLOCK_PROGRESS_M - _TOLERANCE
            ):
                failures.append("planner_deadlock")
                break
        else:
            deadlock_progress.clear()
            deadlock_section_index = None
        if result.tracking_error_m is not None and (
            result.tracking_error_m > PERSISTENT_PUBLIC_TRACKING_ERROR_LIMIT_M + _TOLERANCE
        ):
            failures.append("tracking_error_exceeded")
        if record.safety_decision.failure_reasons:
            failures.extend(
                f"gate:{reason}" for reason in record.safety_decision.failure_reasons
            )
        if record.safety_decision.motion_state is DynamicMotionState.COMPLETED:
            break
        if result.status in {
            PersistentControllerStatus.HOLD_REQUESTED,
            PersistentControllerStatus.NO_SAFE_COMMAND,
            PersistentControllerStatus.INVALID_REFERENCE_INPUT,
            PersistentControllerStatus.STALE_REFERENCE_INPUT,
            PersistentControllerStatus.LATE_RESULT,
            PersistentControllerStatus.SECTION_EXECUTION_FAILED,
        }:
            failures.append(f"controller:{result.status.value}:{result.failure_reason}")
            break

    samples = tuple(_trace_sample(record) for record in records)
    completed = bool(
        records and records[-1].safety_decision.motion_state is DynamicMotionState.COMPLETED
    )
    if not completed and not failures:
        failures.append("simulation_timeout")
    status = (
        PersistentPublicRunStatus.COMPLETED
        if completed and not failures
        else PersistentPublicRunStatus.HARD_FAILED
        if failures and failures != ["simulation_timeout"]
        else PersistentPublicRunStatus.TIMED_OUT
    )
    metrics = _run_metrics(records, deadlock_count=int("planner_deadlock" in failures))
    failures.extend(_run_functional_failures(reference, metrics, samples))
    return PersistentPublicControllerRun(
        controller_name=controller.name,
        controller_version=controller_version,
        reference_session_id=reference.reference_session_id,
        candidate_id=reference.candidate_id,
        paired_input_hash=paired_input_hash,
        status=status,
        metrics=metrics,
        samples=samples,
        section_sequence=_section_sequence(samples),
        hard_failures=tuple(failures),
        elapsed_nonqualification_ns=perf_counter_ns() - started,
    )


def _fresh_empty(
    r4_result: LocalReferencePublicCaseResult,
    tick: int,
) -> tuple[DynamicObservationSnapshot, ActorPredictionSet]:
    simulation_time_s = tick * DYNAMIC_CONTROL_PERIOD_S
    metadata = r4_result.build_context.static_grid_snapshot.metadata
    payload = {
        "stream_id": "r5-public-static-empty-v1",
        "episode_id": f"r5-public-{r4_result.public_id}",
        "episode_seed": 20260814 + r4_result.ordinal,
        "map_id": metadata.map_id,
        "map_revision": metadata.map_revision,
        "observation_revision": metadata.observation_revision,
        "sequence": tick,
        "observed_at_s": simulation_time_s,
        "delivered_at_s": simulation_time_s,
        "frame_kind": DynamicObservationFrameKind.EMPTY,
        "tracks": (),
    }
    frame = DynamicObservationFrame(**payload, content_hash=canonical_content_hash(payload))
    observation = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.FRESH,
        frame=frame,
        age_s=0.0,
        failures=(),
        last_event_was_no_frame=False,
    )
    prediction = ActorPredictionSet(
        stream_id=frame.stream_id,
        episode_id=frame.episode_id,
        map_id=frame.map_id,
        map_revision=frame.map_revision,
        observation_revision=frame.observation_revision,
        sequence=frame.sequence,
        source_content_hash=frame.content_hash,
        observed_at_s=simulation_time_s,
        controller_time_s=simulation_time_s,
        snapshot_age_s=0.0,
        tubes=(),
    )
    return observation, prediction


def _trace_sample(record: PersistentPipelineStep) -> PersistentPublicTraceSample:
    result = record.controller_result
    if result is None:
        raise ValueError("ready controller trace requires a controller result")
    return PersistentPublicTraceSample(
        tick_id=record.tick_id,
        simulation_time_s=record.simulation_time_s,
        pose_before=record.robot_state_before.pose,
        pose_after=record.robot_state_after.pose,
        requested_twist=result.requested_twist,
        applied_twist=record.safety_decision.command,
        controller_status=result.status,
        motion_state=record.safety_decision.motion_state,
        executor_state=result.executor_state.value,
        session_transition=result.controller_session_transition,
        active_section_index=result.active_section_index,
        active_section_kind=(
            None if result.active_section_kind is None else result.active_section_kind.value
        ),
        active_travel_direction=(
            None
            if result.active_section_index is None
            else record.tick_input.full_reference.sections[
                result.active_section_index
            ].travel_direction.value
        ),
        tracking_error_m=result.tracking_error_m,
        minimum_static_clearance_m=record.safety_decision.minimum_static_clearance_m,
        controller_failure_reason=result.failure_reason,
        decision_trace=result.decision_trace,
        candidate_diagnostics=result.candidate_diagnostics,
        gate_failure_reasons=record.safety_decision.failure_reasons,
    )


def _run_functional_failures(
    reference: LocalManeuverReference,
    metrics: PersistentPublicRunMetrics,
    samples: Sequence[PersistentPublicTraceSample],
) -> tuple[str, ...]:
    failures: list[str] = []
    if metrics.initial_bind_count != 1:
        failures.append(f"initial_bind_count:{metrics.initial_bind_count}")
    if metrics.same_session_reset_count != 0:
        failures.append(f"same_session_reset_count:{metrics.same_session_reset_count}")
    if metrics.deadlock_count != 0:
        failures.append("planner_deadlock")
    if metrics.maximum_tracking_error_m > PERSISTENT_PUBLIC_TRACKING_ERROR_LIMIT_M + _TOLERANCE:
        failures.append("tracking_error_exceeded")
    if metrics.gate_override_count != 0:
        failures.append(f"gate_override_count:{metrics.gate_override_count}")
    if metrics.gate_rejection_count != 0:
        failures.append(f"gate_rejection_count:{metrics.gate_rejection_count}")
    if metrics.late_result_count != 0:
        failures.append(f"late_result_count:{metrics.late_result_count}")
    reverse_sections_with_motion: set[int] = set()
    for sample in samples:
        linear = sample.requested_twist.linear
        if sample.controller_status is not PersistentControllerStatus.COMMAND_FOUND:
            continue
        if sample.active_travel_direction == ReferenceTravelDirection.FORWARD.value:
            if linear < -_TOLERANCE:
                failures.append("forward_section_negative_command")
        elif sample.active_travel_direction == ReferenceTravelDirection.REVERSE.value:
            if linear > _TOLERANCE:
                failures.append("reverse_section_positive_command")
            if linear < -0.10 - _TOLERANCE:
                failures.append("reverse_speed_limit_exceeded")
            if linear < -_TOLERANCE and sample.active_section_index is not None:
                reverse_sections_with_motion.add(sample.active_section_index)
        elif abs(linear) > _TOLERANCE:
            failures.append("non_translation_section_linear_command")
    indices = tuple(
        sample.active_section_index
        for sample in samples
        if sample.active_section_index is not None
    )
    if any(right < left for left, right in zip(indices, indices[1:], strict=False)):
        failures.append("section_index_regression")
    observed = set(indices)
    for section in reference.sections:
        if (
            section.travel_direction is ReferenceTravelDirection.REVERSE
            and section.section_index not in reverse_sections_with_motion
        ):
            failures.append(f"reverse_section_without_negative_motion:{section.section_index}")
        if section.section_index in observed:
            continue
        first = reference.knots[section.first_knot_index]
        last = reference.knots[section.last_knot_index]
        if (
            last.cumulative_translation_arc_m
            - first.cumulative_translation_arc_m
            > _TOLERANCE
            or section.section_kind is ReferenceSectionKind.ROTATE
        ):
            failures.append(f"section_not_observed:{section.section_index}")
    return tuple(sorted(set(failures)))


def _run_metrics(
    records: Sequence[PersistentPipelineStep],
    *,
    deadlock_count: int,
) -> PersistentPublicRunMetrics:
    samples = tuple(_trace_sample(record) for record in records)
    path_length = sum(
        hypot(
            right.pose_after.x - left.pose_after.x,
            right.pose_after.y - left.pose_after.y,
        )
        for left, right in zip(samples, samples[1:], strict=False)
    )
    tracking = tuple(
        sample.tracking_error_m
        for sample in samples
        if sample.tracking_error_m is not None
    )
    applied = tuple(sample.applied_twist for sample in samples)
    linear_accel = _differences(tuple(item.linear for item in applied), DYNAMIC_CONTROL_PERIOD_S)
    angular_accel = _differences(tuple(item.angular for item in applied), DYNAMIC_CONTROL_PERIOD_S)
    linear_jerk = _differences(linear_accel, DYNAMIC_CONTROL_PERIOD_S)
    angular_jerk = _differences(angular_accel, DYNAMIC_CONTROL_PERIOD_S)
    nonzero_signs = tuple(
        1 if item.linear > _TOLERANCE else -1
        for item in applied
        if abs(item.linear) > _TOLERANCE
    )
    reversals = sum(
        left != right
        for left, right in zip(nonzero_signs, nonzero_signs[1:], strict=False)
    )
    completion = next(
        (
            sample
            for sample in reversed(samples)
            if sample.motion_state is DynamicMotionState.COMPLETED
        ),
        None,
    )
    last_counters = records[-1].safety_decision.counters if records else None
    statuses = tuple(
        record.controller_result.status
        for record in records
        if record.controller_result is not None
    )
    transitions = tuple(
        record.controller_result.controller_session_transition
        for record in records
        if record.controller_result is not None
    )
    clearances = tuple(
        sample.minimum_static_clearance_m
        for sample in samples
        if sample.minimum_static_clearance_m is not None
    )
    return PersistentPublicRunMetrics(
        tick_count=len(samples),
        completion_tick=None if completion is None else completion.tick_id,
        completion_simulation_time_s=(
            None if completion is None else completion.simulation_time_s
        ),
        actual_path_length_m=path_length,
        maximum_tracking_error_m=max(tracking, default=0.0),
        rms_tracking_error_m=_rms(tracking),
        minimum_static_clearance_m=min(clearances) if clearances else None,
        longitudinal_jerk_rms_mps3=_rms(linear_jerk),
        angular_acceleration_rms_radps2=_rms(angular_accel),
        angular_jerk_rms_radps3=_rms(angular_jerk),
        peak_angular_velocity_radps=max((abs(item.angular) for item in applied), default=0.0),
        direction_reversal_count=reversals,
        initial_bind_count=sum(
            item is PersistentControllerSessionTransition.INITIAL_BIND for item in transitions
        ),
        same_session_reset_count=sum(
            item is PersistentControllerSessionTransition.SESSION_RESET for item in transitions[1:]
        ),
        window_advance_count=sum(
            item is PersistentControllerSessionTransition.WINDOW_ADVANCED for item in transitions
        ),
        planned_stop_count=sum(
            item is PersistentControllerStatus.PLANNED_STOP for item in statuses
        ),
        controller_stop_request_count=(
            0 if last_counters is None else last_counters.controller_stop_requests
        ),
        no_safe_command_count=sum(
            item is PersistentControllerStatus.NO_SAFE_COMMAND for item in statuses
        ),
        gate_override_count=0 if last_counters is None else last_counters.gate_overrides,
        gate_rejection_count=(
            0 if last_counters is None else last_counters.candidate_rejected_by_gate
        ),
        late_result_count=0 if last_counters is None else last_counters.late_results_discarded,
        deadlock_count=deadlock_count,
    )


def _pair_delta(
    rpp: PersistentPublicRunMetrics,
    dwb: PersistentPublicRunMetrics,
) -> PersistentPublicPairDelta:
    completion_delta = None
    if (
        rpp.completion_simulation_time_s is not None
        and dwb.completion_simulation_time_s is not None
    ):
        completion_delta = (
            dwb.completion_simulation_time_s - rpp.completion_simulation_time_s
        )
    return PersistentPublicPairDelta(
        completion_simulation_time_s=completion_delta,
        maximum_tracking_error_m=(
            dwb.maximum_tracking_error_m - rpp.maximum_tracking_error_m
        ),
        actual_path_length_m=dwb.actual_path_length_m - rpp.actual_path_length_m,
        longitudinal_jerk_rms_mps3=(
            dwb.longitudinal_jerk_rms_mps3 - rpp.longitudinal_jerk_rms_mps3
        ),
        angular_acceleration_rms_radps2=(
            dwb.angular_acceleration_rms_radps2 - rpp.angular_acceleration_rms_radps2
        ),
        angular_jerk_rms_radps3=(
            dwb.angular_jerk_rms_radps3 - rpp.angular_jerk_rms_radps3
        ),
        gate_override_count=dwb.gate_override_count - rpp.gate_override_count,
        no_safe_command_count=dwb.no_safe_command_count - rpp.no_safe_command_count,
    )


def _public_relation_failures(
    results: Sequence[PersistentPublicCaseResult],
) -> tuple[str, ...]:
    by_id = {result.public_id: result for result in results}
    failures: list[str] = []
    for first_id, second_id, label, mirrored in (
        (
            "wide-straight-left",
            "wide-straight-right",
            "wide_left_right_mirror",
            True,
        ),
        (
            "wide-mirror-left",
            "wide-mirror-right",
            "reverse_left_right_mirror",
            True,
        ),
        (
            "wide-straight-left",
            "wide-mirror-right",
            "wide_reverse_mirror_left",
            True,
        ),
        (
            "wide-straight-right",
            "wide-mirror-left",
            "wide_reverse_mirror_right",
            True,
        ),
        (
            "wide-straight-left",
            "vertical-left",
            "horizontal_vertical_left",
            False,
        ),
        (
            "wide-straight-right",
            "vertical-right",
            "horizontal_vertical_right",
            False,
        ),
    ):
        first = by_id[first_id]
        second = by_id[second_id]
        for controller_name in ("rpp_result", "dwb_result"):
            left = getattr(first, controller_name)
            right = getattr(second, controller_name)
            if left is None or right is None:
                failures.append(f"{label}:{controller_name}:missing")
                continue
            if left.status is not right.status:
                failures.append(f"{label}:{controller_name}:status")
            if left.section_sequence != right.section_sequence:
                failures.append(f"{label}:{controller_name}:section_sequence")
            if _normalized_run_signature(left) != _normalized_run_signature(
                right,
                mirror_lateral=mirrored,
            ):
                failures.append(f"{label}:{controller_name}:trajectory")
    for wide_id, crossing_id in (
        ("wide-straight-left", "crossing-static-left"),
        ("wide-straight-right", "crossing-static-right"),
    ):
        wide = by_id[wide_id]
        result = by_id[crossing_id]
        if not _is_ordered_subsequence(
            tuple(kind for _, kind in wide.reference_sections),
            tuple(kind for _, kind in result.reference_sections),
        ):
            failures.append(f"{crossing_id}:reference_section_order")
        for controller_name in ("rpp_result", "dwb_result"):
            controller = getattr(result, controller_name)
            if controller is None or not controller.section_sequence:
                failures.append(
                    f"{crossing_id}:{controller_name}:section_sequence_missing"
                )
    return tuple(sorted(set(failures)))


def _is_ordered_subsequence(
    expected: Sequence[str],
    observed: Sequence[str],
) -> bool:
    """Return whether every expected section occurs in order in observed."""

    cursor = iter(observed)
    return all(any(candidate == item for candidate in cursor) for item in expected)


def _normalized_run_signature(
    result: PersistentPublicControllerRun,
    *,
    mirror_lateral: bool = False,
) -> tuple[tuple[object, ...], ...]:
    if not result.samples:
        return ()
    origin = result.samples[0].pose_before
    cosine = cos(origin.yaw)
    sine = sin(origin.yaw)
    signature: list[tuple[object, ...]] = []
    for sample in result.samples:
        dx = sample.pose_after.x - origin.x
        dy = sample.pose_after.y - origin.y
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        local_yaw = atan2(
            sin(sample.pose_after.yaw - origin.yaw),
            cos(sample.pose_after.yaw - origin.yaw),
        )
        angular = sample.applied_twist.angular
        if mirror_lateral:
            local_y = -local_y
            local_yaw = -local_yaw
            angular = -angular
        signature.append(
            (
                sample.tick_id,
                round(local_x, 9),
                round(local_y, 9),
                round(local_yaw, 9),
                round(sample.applied_twist.linear, 9),
                round(angular, 9),
                sample.controller_status.value,
                sample.motion_state.value,
                sample.active_section_index,
                sample.active_section_kind,
                sample.active_travel_direction,
            )
        )
    return tuple(signature)


def _section_sequence(
    samples: Sequence[PersistentPublicTraceSample],
) -> tuple[tuple[int, str], ...]:
    sequence: list[tuple[int, str]] = []
    for sample in samples:
        if sample.active_section_index is None or sample.active_section_kind is None:
            continue
        item = (sample.active_section_index, sample.active_section_kind)
        if not sequence or sequence[-1] != item:
            sequence.append(item)
    return tuple(sequence)


def _episode_tick_limit(translation_arc_m: float) -> int:
    timeout_s = max(
        PERSISTENT_PUBLIC_MINIMUM_TIMEOUT_S,
        translation_arc_m
        / PERSISTENT_PUBLIC_TARGET_SPEED_MPS
        * PERSISTENT_PUBLIC_TIMEOUT_MULTIPLIER
        + PERSISTENT_PUBLIC_TIMEOUT_MARGIN_S,
    )
    return int(ceil(timeout_s / DYNAMIC_CONTROL_PERIOD_S))


def _reference_section_progress_m(
    knots: Sequence[ReferenceKnot],
    section: ReferenceSection,
    pose: Pose2D,
) -> float:
    best: tuple[float, float] | None = None
    section_knots = tuple(
        knot
        for knot in knots
        if section.first_knot_index <= knot.knot_index <= section.last_knot_index
    )
    for left, right in zip(section_knots, section_knots[1:], strict=False):
        left_pose = left.pose
        right_pose = right.pose
        dx = right_pose.x - left_pose.x
        dy = right_pose.y - left_pose.y
        length = hypot(dx, dy)
        if length <= _TOLERANCE:
            continue
        fraction = min(
            max(((pose.x - left_pose.x) * dx + (pose.y - left_pose.y) * dy) / length**2, 0.0),
            1.0,
        )
        x = left_pose.x + fraction * dx
        y = left_pose.y + fraction * dy
        progress = left.cumulative_translation_arc_m + fraction * length
        candidate = (hypot(pose.x - x, pose.y - y), -progress)
        if best is None or candidate < best:
            best = candidate
    if best is not None:
        return -best[1]
    if section_knots:
        return section_knots[0].cumulative_translation_arc_m
    raise ValueError("active reference section has no knots")


def _differences(values: Sequence[float], period_s: float) -> tuple[float, ...]:
    return tuple(
        (right - left) / period_s
        for left, right in zip(values, values[1:], strict=False)
    )


def _maximum_forward_progress_m(samples: Sequence[tuple[int, float]]) -> float:
    if not samples:
        return 0.0
    minimum = samples[0][1]
    maximum_increase = 0.0
    for _, progress in samples[1:]:
        maximum_increase = max(maximum_increase, progress - minimum)
        minimum = min(minimum, progress)
    return maximum_increase


def _rms(values: Sequence[float]) -> float:
    return 0.0 if not values else sqrt(sum(value * value for value in values) / len(values))


def _r4_catalog_content_hash(cases: Sequence[LocalReferencePublicCase]) -> str:
    return canonical_content_hash(
        {"case_hashes": tuple(case.semantic_content_hash for case in cases)}
    )


def _source_freeze_hash(repository_root: Path) -> str:
    lab = repository_root / "simulation" / "path_planning_lab"
    paths = tuple(sorted((lab / "src" / "hospital_path_lab").rglob("*.py"))) + (
        lab / "scripts" / "run_persistent_controller_public.py",
        lab / "pyproject.toml",
    )
    digest = sha256()
    for path in paths:
        digest.update(path.relative_to(repository_root).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def _save_trajectory_plot(result: PersistentPublicCaseResult, output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(7.0, 5.0))
    try:
        for controller, color, label in (
            (result.rpp_result, "tab:blue", "RPP"),
            (result.dwb_result, "tab:orange", "DWB"),
        ):
            if controller is None or not controller.samples:
                continue
            axis.plot(
                [sample.pose_after.x for sample in controller.samples],
                [sample.pose_after.y for sample in controller.samples],
                color=color,
                linewidth=1.5,
                label=f"{label} ({controller.status.value})",
            )
        if result.reference_path:
            axis.plot(
                [pose.x for pose in result.reference_path],
                [pose.y for pose in result.reference_path],
                "--",
                color="black",
                linewidth=1.0,
                label="R4 reference",
            )
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.2)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_title(f"R5-A persistent controllers\n{result.public_id}")
        if result.rpp_result is not None:
            axis.legend(loc="best")
        figure.tight_layout()
        figure.savefig(output_path, dpi=150, format="png")
    finally:
        plt.close(figure)


def _audit_summary(audit: PersistentPublicAudit) -> str:
    lines = [
        "# R5-A persistent controller 공개 qualification",
        "",
        f"- hard 판정: `{'PASS' if audit.hard_passed else 'FAIL'}`",
        f"- 공개 case: `{len(audit.case_results)}`",
        f"- hidden 사용: `{str(audit.hidden_used).lower()}`",
        f"- serial/process parity: `{'PASS' if audit.serial_process_parity_passed else 'FAIL'}`",
        f"- repeat determinism: `{'PASS' if audit.repeat_determinism_passed else 'FAIL'}`",
        f"- semantic hash: `{audit.semantic_content_hash}`",
        "- Python wall-clock과 worker 수는 판정 근거가 아니다.",
        "",
        "| id | reference | RPP | DWB | controller calls | hard |",
        "|---|---|---|---|---:|---|",
    ]
    for item in audit.case_results:
        lines.append(
            f"| `{item.public_id}` | {item.reference_status.value} | "
            f"{_run_status(item.rpp_result)} | {_run_status(item.dwb_result)} | "
            f"{item.controller_call_count} | {'PASS' if item.hard_passed else 'FAIL'} |"
        )
    lines.extend(("", "## 한계", ""))
    lines.extend(f"- `{value}`" for value in audit.limitations)
    return "\n".join(lines) + "\n"


def _run_status(result: PersistentPublicControllerRun | None) -> str:
    return "not_called" if result is None else result.status.value


def _git_output(repository_root: Path, *args: str) -> str:
    completed = subprocess.run(
        ("git", *args),
        cwd=repository_root,
        capture_output=True,
        text=True,
        check=True,
    )
    return completed.stdout.strip()


def _jsonable(value: object) -> object:
    if is_dataclass(value) and not isinstance(value, type):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    if isinstance(value, (set, frozenset)):
        return [_jsonable(item) for item in sorted(value, key=repr)]
    if isinstance(value, StrEnum):
        return value.value
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def _write_exclusive_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(
            _jsonable(value),
            stream,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        stream.write("\n")


def _write_atomic_json(path: Path, value: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(
            _jsonable(value),
            stream,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        stream.write("\n")
    temporary.replace(path)


def _write_exclusive_text(path: Path, value: str) -> None:
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(value)


def _bind_hash(value: object, field_name: str, expected: str) -> None:
    current = getattr(value, field_name)
    if current:
        _require_sha256(current, field_name)
        if current != expected:
            raise ValueError(f"{field_name} mismatch")
    else:
        object.__setattr__(value, field_name, expected)


def _require_sha256(value: str, field_name: str) -> None:
    if (
        not isinstance(value, str)
        or len(value) != _SHA256_LENGTH
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError(f"{field_name} must be a lowercase SHA-256 digest")


__all__ = [
    "PERSISTENT_PUBLIC_READY_CASE_COUNT",
    "PersistentPublicAudit",
    "PersistentPublicCaseResult",
    "PersistentPublicControllerRun",
    "PersistentPublicManifest",
    "PersistentPublicOutputWriter",
    "PersistentPublicRunMetrics",
    "PersistentPublicRunStatus",
    "PersistentPublicTraceSample",
    "audit_persistent_public_catalog",
    "build_persistent_public_manifest",
    "evaluate_persistent_public_case",
    "evaluate_persistent_public_cases",
]
