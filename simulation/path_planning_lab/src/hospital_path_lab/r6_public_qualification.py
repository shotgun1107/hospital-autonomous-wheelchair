"""R6 최신 R5-B/C 연속 공개 종단 자격 runner의 결정론적 핵심."""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from time import perf_counter

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.dynamic_contracts import DYNAMIC_CONTROL_PERIOD_S
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
)
from hospital_path_lab.dynamic_safety import DynamicMotionState
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PersistentSourceDerivedDwbController,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r5b_restop_execution import run_r5b_restop_case
from hospital_path_lab.r5b_temporal_evidence import frozen_r2_archive_path
from hospital_path_lab.r5b_temporal_execution import run_r5b_temporal_case
from hospital_path_lab.r5b_temporal_reference import (
    build_r5b_crossing_reference_bundles,
    build_r5b_temporal_reference_bundles,
)
from hospital_path_lab.r5c_observation_diagnostic import (
    R5CDiagnosticOutcome,
    run_r5c_crossing_completion_diagnostic,
)

R6_PUBLIC_QUALIFICATION_VERSION = "r6-public-end-to-end-v1"
R6_REQUIRED_CASE_COUNT = 17


class R6CaseKind(StrEnum):
    SAME_DIRECTION_IDEAL = "same_direction_ideal"
    CROSSING_IDEAL = "crossing_ideal"
    RESTOP_IDEAL = "restop_ideal"
    CROSSING_NORMAL = "crossing_normal"
    CROSSING_STRESS = "crossing_stress"


class R6ExpectedOutcome(StrEnum):
    COMPLETE = "complete"
    CONSERVATIVE_HOLD = "conservative_hold"


@dataclass(frozen=True, slots=True)
class R6PublicCaseSpec:
    ordinal: int
    case_id: str
    kind: R6CaseKind
    source_index: int | None
    profile_name: str
    tick_limit: int
    expected_outcome: R6ExpectedOutcome

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class R6PublicCaseResult:
    ordinal: int
    case_id: str
    kind: R6CaseKind
    profile_name: str
    expected_outcome: R6ExpectedOutcome
    passed: bool
    outcome: str
    completion_tick: int | None
    departure_tick: int | None
    pass_tick: int | None
    rejoin_tick: int | None
    second_stop_tick: int | None
    second_release_tick: int | None
    post_pass_proof_tick: int | None
    follow_original_release_tick: int | None
    first_motion_tick: int | None
    controller_call_count: int
    controller_session_count: int
    release_ticks: tuple[int, ...]
    final_motion_state: str
    final_stop_epoch: int
    final_pose: Pose2D | None
    minimum_actor_clearance_m: float | None
    minimum_static_clearance_m: float
    gate_override_count: int
    native_full_core_used: bool | None
    hard_failures: tuple[str, ...]
    trace_content_hash: str
    elapsed_s: float
    content_hash: str = ""

    def __post_init__(self) -> None:
        payload = {
            name: getattr(self, name)
            for name in self.__dataclass_fields__
            if name not in {"elapsed_s", "content_hash"}
        }
        expected = canonical_content_hash(payload)
        if self.content_hash and self.content_hash != expected:
            raise ValueError("R6 case result content hash mismatch")
        object.__setattr__(self, "content_hash", expected)


@dataclass(frozen=True, slots=True)
class R6PublicAudit:
    passed: bool
    required_case_count: int
    result_count: int
    failures: tuple[str, ...]
    results: tuple[R6PublicCaseResult, ...]
    result_set_hash: str


def public_r6_case_specs(repository_root: Path) -> tuple[R6PublicCaseSpec, ...]:
    """Return the exact ordered R6 public matrix without running controllers."""

    same_direction = build_r5b_temporal_reference_bundles(
        frozen_r2_archive_path(repository_root)
    )
    crossing = build_r5b_crossing_reference_bundles()
    specs: list[R6PublicCaseSpec] = []

    for index, bundle in enumerate(same_direction):
        specs.append(
            R6PublicCaseSpec(
                ordinal=len(specs),
                case_id=(
                    f"same-direction-{bundle.source.corpus_ordinal:02d}-"
                    f"{bundle.source.side.value}-ideal"
                ),
                kind=R6CaseKind.SAME_DIRECTION_IDEAL,
                source_index=index,
                profile_name="functional_ideal",
                tick_limit=int(
                    round(bundle.source.world.duration_s / DYNAMIC_CONTROL_PERIOD_S)
                ),
                expected_outcome=R6ExpectedOutcome.COMPLETE,
            )
        )
    for index, bundle in enumerate(crossing):
        specs.append(
            R6PublicCaseSpec(
                ordinal=len(specs),
                case_id=f"crossing-{bundle.source.side.value}-ideal",
                kind=R6CaseKind.CROSSING_IDEAL,
                source_index=index,
                profile_name="functional_ideal",
                tick_limit=int(
                    round(bundle.source.world.duration_s / DYNAMIC_CONTROL_PERIOD_S)
                ),
                expected_outcome=R6ExpectedOutcome.COMPLETE,
            )
        )
    specs.append(
        R6PublicCaseSpec(
            ordinal=len(specs),
            case_id="multi-hazard-restop-ideal",
            kind=R6CaseKind.RESTOP_IDEAL,
            source_index=None,
            profile_name="functional_ideal",
            tick_limit=700,
            expected_outcome=R6ExpectedOutcome.COMPLETE,
        )
    )
    for index, bundle in enumerate(crossing):
        specs.append(
            R6PublicCaseSpec(
                ordinal=len(specs),
                case_id=f"crossing-{bundle.source.side.value}-normal",
                kind=R6CaseKind.CROSSING_NORMAL,
                source_index=index,
                profile_name="normal",
                tick_limit=1_600,
                expected_outcome=R6ExpectedOutcome.COMPLETE,
            )
        )
    for index, bundle in enumerate(crossing):
        specs.append(
            R6PublicCaseSpec(
                ordinal=len(specs),
                case_id=f"crossing-{bundle.source.side.value}-stress",
                kind=R6CaseKind.CROSSING_STRESS,
                source_index=index,
                profile_name="stress",
                tick_limit=1_600,
                expected_outcome=R6ExpectedOutcome.CONSERVATIVE_HOLD,
            )
        )

    result = tuple(specs)
    if len(result) != R6_REQUIRED_CASE_COUNT:
        raise RuntimeError("R6 public catalog does not contain exactly 17 cases")
    if len({item.case_id for item in result}) != len(result):
        raise RuntimeError("R6 public catalog contains duplicate case IDs")
    if tuple(item.ordinal for item in result) != tuple(range(len(result))):
        raise RuntimeError("R6 public catalog ordinals are not contiguous")
    return result


def run_r6_public_case(
    repository_root: Path,
    spec: R6PublicCaseSpec,
) -> R6PublicCaseResult:
    """Run one case continuously; no checkpoint result is accepted as input."""

    started = perf_counter()
    if spec.kind in {R6CaseKind.SAME_DIRECTION_IDEAL, R6CaseKind.CROSSING_IDEAL}:
        bundles = (
            build_r5b_temporal_reference_bundles(frozen_r2_archive_path(repository_root))
            if spec.kind is R6CaseKind.SAME_DIRECTION_IDEAL
            else build_r5b_crossing_reference_bundles()
        )
        if spec.source_index is None:
            raise ValueError("R6 temporal case requires source_index")
        controller = PersistentSourceDerivedDwbController(
            use_cpp_safety_core=True,
            use_cpp_full_core=True,
        )
        result = run_r5b_temporal_case(
            bundles[spec.source_index],
            controller=controller,
            tick_limit=spec.tick_limit,
        )
        return R6PublicCaseResult(
            ordinal=spec.ordinal,
            case_id=spec.case_id,
            kind=spec.kind,
            profile_name=spec.profile_name,
            expected_outcome=spec.expected_outcome,
            passed=result.passed,
            outcome="completed" if result.completed else "failed",
            completion_tick=result.completion_tick,
            departure_tick=result.departure_tick,
            pass_tick=result.pass_event_tick,
            rejoin_tick=result.rejoin_tick,
            second_stop_tick=None,
            second_release_tick=None,
            post_pass_proof_tick=None,
            follow_original_release_tick=None,
            first_motion_tick=result.first_motion_tick,
            controller_call_count=result.controller_call_count,
            controller_session_count=1,
            release_ticks=(result.release_tick,),
            final_motion_state=("completed" if result.completed else "failed"),
            final_stop_epoch=1,
            final_pose=result.final_pose,
            minimum_actor_clearance_m=result.minimum_actor_clearance_m,
            minimum_static_clearance_m=result.minimum_static_clearance_m,
            gate_override_count=result.gate_override_count,
            native_full_core_used=controller.native_full_core_used,
            hard_failures=result.hard_failures,
            trace_content_hash=result.trace_content_hash,
            elapsed_s=perf_counter() - started,
        )

    if spec.kind is R6CaseKind.RESTOP_IDEAL:
        result = run_r5b_restop_case(tick_limit=spec.tick_limit)
        return R6PublicCaseResult(
            ordinal=spec.ordinal,
            case_id=spec.case_id,
            kind=spec.kind,
            profile_name=spec.profile_name,
            expected_outcome=spec.expected_outcome,
            passed=result.passed,
            outcome="completed" if result.completed else "failed",
            completion_tick=result.completion_tick,
            departure_tick=None,
            pass_tick=None,
            rejoin_tick=None,
            second_stop_tick=result.second_stop_tick,
            second_release_tick=result.second_release_tick,
            post_pass_proof_tick=None,
            follow_original_release_tick=None,
            first_motion_tick=result.first_motion_tick,
            controller_call_count=0,
            controller_session_count=result.controller_session_count,
            release_ticks=tuple(
                tick
                for tick in (result.first_release_tick, result.second_release_tick)
                if tick is not None
            ),
            final_motion_state=("completed" if result.completed else "failed"),
            final_stop_epoch=result.second_stop_epoch or 1,
            final_pose=None,
            minimum_actor_clearance_m=result.minimum_actor_clearance_m,
            minimum_static_clearance_m=result.minimum_static_clearance_m,
            gate_override_count=result.gate_override_count,
            native_full_core_used=result.native_full_core_used,
            hard_failures=result.hard_failures,
            trace_content_hash=result.trace_content_hash,
            elapsed_s=perf_counter() - started,
        )

    if spec.source_index is None:
        raise ValueError("R6 observation-integrated case requires source_index")
    profile = (
        NORMAL_OBSERVATION_PROFILE
        if spec.kind is R6CaseKind.CROSSING_NORMAL
        else STRESS_OBSERVATION_PROFILE
    )
    result = run_r5c_crossing_completion_diagnostic(
        side_index=spec.source_index,
        profile=profile,
        tick_limit=spec.tick_limit,
    )
    if spec.expected_outcome is R6ExpectedOutcome.COMPLETE:
        passed = all(
            (
                result.outcome is R5CDiagnosticOutcome.COMPLETED,
                result.completion_tick is not None,
                result.post_pass_proof_tick is not None,
                result.follow_original_release_tick is not None,
                result.first_motion_tick is not None,
                result.final_motion_state is DynamicMotionState.COMPLETED,
                not result.hard_failures,
            )
        )
    else:
        passed = all(
            (
                result.outcome is R5CDiagnosticOutcome.CONSERVATIVE_HOLD,
                result.actual_release_tick is None,
                result.first_motion_tick is None,
                result.controller_call_count == 0,
                not result.release_ticks,
                result.final_motion_state is DynamicMotionState.HOLDING,
                not result.hard_failures,
            )
        )
    return R6PublicCaseResult(
        ordinal=spec.ordinal,
        case_id=spec.case_id,
        kind=spec.kind,
        profile_name=spec.profile_name,
        expected_outcome=spec.expected_outcome,
        passed=passed,
        outcome=result.outcome.value,
        completion_tick=result.completion_tick,
        departure_tick=None,
        pass_tick=result.post_pass_proof_tick,
        rejoin_tick=None,
        second_stop_tick=None,
        second_release_tick=None,
        post_pass_proof_tick=result.post_pass_proof_tick,
        follow_original_release_tick=result.follow_original_release_tick,
        first_motion_tick=result.first_motion_tick,
        controller_call_count=result.controller_call_count,
        controller_session_count=result.controller_session_count,
        release_ticks=result.release_ticks,
        final_motion_state=result.final_motion_state.value,
        final_stop_epoch=result.final_stop_epoch,
        final_pose=result.final_pose,
        minimum_actor_clearance_m=result.minimum_actor_clearance_m,
        minimum_static_clearance_m=result.minimum_static_clearance_m,
        gate_override_count=result.gate_override_count,
        native_full_core_used=None,
        hard_failures=result.hard_failures,
        trace_content_hash=result.trace_content_hash,
        elapsed_s=perf_counter() - started,
    )


def evaluate_r6_public_cases(
    repository_root: Path,
    specs: tuple[R6PublicCaseSpec, ...],
    *,
    max_workers: int,
    on_case: Callable[[R6PublicCaseResult], None] | None = None,
) -> tuple[R6PublicCaseResult, ...]:
    if max_workers <= 0:
        raise ValueError("max_workers must be positive")
    if max_workers == 1:
        results = []
        for spec in specs:
            result = run_r6_public_case(repository_root, spec)
            results.append(result)
            if on_case is not None:
                on_case(result)
        return tuple(results)

    completed: dict[int, R6PublicCaseResult] = {}
    with ProcessPoolExecutor(max_workers=max_workers) as pool:
        futures = {
            pool.submit(run_r6_public_case, repository_root, spec): spec
            for spec in specs
        }
        for future in as_completed(futures):
            result = future.result()
            completed[result.ordinal] = result
            if on_case is not None:
                on_case(result)
    return tuple(completed[index] for index in sorted(completed))


def audit_r6_public_results(
    specs: tuple[R6PublicCaseSpec, ...],
    results: tuple[R6PublicCaseResult, ...],
) -> R6PublicAudit:
    failures: list[str] = []
    expected_ids = tuple(item.case_id for item in specs)
    result_ids = tuple(item.case_id for item in results)
    if len(specs) != R6_REQUIRED_CASE_COUNT:
        failures.append("required_case_catalog_incomplete")
    if len(results) != len(specs):
        failures.append("result_count_mismatch")
    if result_ids != expected_ids:
        failures.append("result_order_or_identity_mismatch")
    for spec, result in zip(specs, results, strict=False):
        if result.ordinal != spec.ordinal or result.kind is not spec.kind:
            failures.append(f"{spec.case_id}:spec_binding_mismatch")
        if not result.passed:
            failures.append(f"{spec.case_id}:case_failed")
        if result.hard_failures:
            failures.append(f"{spec.case_id}:hard_failure")
    result_set_hash = canonical_content_hash(
        tuple((item.case_id, item.content_hash) for item in results)
    )
    return R6PublicAudit(
        passed=not failures,
        required_case_count=R6_REQUIRED_CASE_COUNT,
        result_count=len(results),
        failures=tuple(dict.fromkeys(failures)),
        results=results,
        result_set_hash=result_set_hash,
    )
