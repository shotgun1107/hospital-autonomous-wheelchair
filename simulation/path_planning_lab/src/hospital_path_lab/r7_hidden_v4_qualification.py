"""R7 hidden-v4 evaluator for conditional Stress release and safe re-stop.

The historical hidden-v3 evaluator remains in :mod:`r7_hidden_qualification`.
This module uses a new seed-commitment namespace and does not change controller,
gate, observation, or restart behavior.
"""

from __future__ import annotations

from collections.abc import Callable
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
from time import perf_counter

from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
)
from hospital_path_lab.dynamic_safety import (
    DYNAMIC_SAFE_OBSERVATION_FRAMES,
    DynamicMotionState,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r5c_observation_diagnostic import (
    R5CDiagnosticOutcome,
    R5CObservationDiagnosticResult,
    run_r5c_crossing_completion_diagnostic,
)
from hospital_path_lab.r7_failure_trace import R7FailureTraceCollector

R7_HIDDEN_V4_OBSERVATION_VERSION = "r7-hidden-observation-v3"
R7_HIDDEN_V4_REPLICA_COUNT = 5
R7_HIDDEN_V4_REQUIRED_CASE_COUNT = 20
R7_HIDDEN_V4_TICK_LIMIT = 1_600
R7_HIDDEN_V4_MINIMUM_CLEARANCE_M = 0.08
_MAX_ROOT_SEED = (1 << 63) - 1


@dataclass(frozen=True, slots=True)
class R7HiddenV4CaseSpec:
    ordinal: int
    case_id: str
    replica: int
    side_index: int
    side_name: str
    profile_name: str
    observation_seed: int
    seed_tag: str
    expected_outcome: str
    tick_limit: int = R7_HIDDEN_V4_TICK_LIMIT

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class R7HiddenV4CaseResult:
    ordinal: int
    case_id: str
    replica: int
    side_name: str
    profile_name: str
    seed_tag: str
    expected_outcome: str
    passed: bool
    outcome: str
    completion_tick: int | None
    post_pass_proof_tick: int | None
    follow_original_release_tick: int | None
    actual_release_tick: int | None
    first_motion_tick: int | None
    protective_stop_started_tick: int | None
    stop_confirmed_tick: int | None
    controller_call_count: int
    release_ticks: tuple[int, ...]
    confirmed_stop_ticks: tuple[int, ...]
    session_stop_epochs: tuple[int, ...]
    final_motion_state: str
    final_stop_epoch: int
    minimum_actor_clearance_m: float | None
    minimum_static_clearance_m: float
    gate_override_count: int
    hard_failures: tuple[str, ...]
    trace_content_hash: str
    trace_file_sha256: str | None
    trace_record_count: int
    trace_last_record_hash: str | None
    minimum_release_confirmed_safe_frames: int | None
    release_contract_violation_count: int
    duplicate_safe_frame_violation_count: int
    stale_propulsion_violation_count: int
    unauthorized_restart_count: int
    actual_collision_count: int
    actual_forbidden_violation_count: int
    actual_clearance_violation_count: int
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
            raise ValueError("R7 hidden-v4 case result content hash mismatch")
        object.__setattr__(self, "content_hash", expected)


@dataclass(frozen=True, slots=True)
class R7HiddenV4Audit:
    passed: bool
    required_case_count: int
    result_count: int
    normal_completed_count: int
    stress_conditionally_safe_count: int
    stress_release_count: int
    hard_failure_count: int
    release_contract_violation_count: int
    duplicate_safe_frame_violation_count: int
    stale_propulsion_violation_count: int
    unauthorized_restart_count: int
    actual_collision_count: int
    actual_forbidden_violation_count: int
    actual_clearance_violation_count: int
    failures: tuple[str, ...]
    result_set_hash: str


def hidden_v4_seed_commitment(root_seed: int) -> str:
    """Return the commitment written before hidden-v4 cases are constructed."""

    _validate_root_seed(root_seed)
    return sha256(f"{R7_HIDDEN_V4_OBSERVATION_VERSION}:{root_seed}".encode()).hexdigest()


def build_hidden_v4_case_specs(root_seed: int) -> tuple[R7HiddenV4CaseSpec, ...]:
    _validate_root_seed(root_seed)
    specs: list[R7HiddenV4CaseSpec] = []
    for replica in range(R7_HIDDEN_V4_REPLICA_COUNT):
        for side_index, side_name in enumerate(("left", "right")):
            observation_seed = _derived_observation_seed(
                root_seed,
                replica=replica,
                side_name=side_name,
            )
            seed_tag = sha256(str(observation_seed).encode("ascii")).hexdigest()
            for profile_name, expected in (
                ("normal", "completed"),
                ("stress", "conditionally_safe_hold"),
            ):
                specs.append(
                    R7HiddenV4CaseSpec(
                        ordinal=len(specs),
                        case_id=f"hidden-v4-{replica:02d}-{side_name}-{profile_name}",
                        replica=replica,
                        side_index=side_index,
                        side_name=side_name,
                        profile_name=profile_name,
                        observation_seed=observation_seed,
                        seed_tag=seed_tag,
                        expected_outcome=expected,
                    )
                )

    result = tuple(specs)
    if len(result) != R7_HIDDEN_V4_REQUIRED_CASE_COUNT:
        raise RuntimeError("R7 hidden-v4 catalog size mismatch")
    if len({item.case_id for item in result}) != len(result):
        raise RuntimeError("R7 hidden-v4 catalog contains duplicate case IDs")
    if tuple(item.ordinal for item in result) != tuple(range(len(result))):
        raise RuntimeError("R7 hidden-v4 catalog ordinals are not contiguous")
    for replica in range(R7_HIDDEN_V4_REPLICA_COUNT):
        for side_name in ("left", "right"):
            paired = tuple(
                item
                for item in result
                if item.replica == replica and item.side_name == side_name
            )
            if len(paired) != 2 or paired[0].observation_seed != paired[1].observation_seed:
                raise RuntimeError("Normal and Stress must share one latent observation seed")
    return result


def run_hidden_v4_case(
    _repository_root: Path,
    spec: R7HiddenV4CaseSpec,
    failure_trace_root: Path | None = None,
) -> R7HiddenV4CaseResult:
    started = perf_counter()
    profile = (
        NORMAL_OBSERVATION_PROFILE
        if spec.profile_name == "normal"
        else STRESS_OBSERVATION_PROFILE
    )
    failure_trace = R7FailureTraceCollector()
    try:
        result = run_r5c_crossing_completion_diagnostic(
            side_index=spec.side_index,
            profile=profile,
            tick_limit=spec.tick_limit,
            observation_seed=spec.observation_seed,
            failure_trace=failure_trace,
        )
    except BaseException:
        if failure_trace.records and failure_trace_root is not None:
            failure_trace.write_jsonl(
                failure_trace_root / spec.case_id / "infrastructure-tick-trace.jsonl"
            )
        raise
    contract_proof = _trace_contract_proof(failure_trace.records, result)
    contract_passed = all(
        contract_proof[name] == 0
        for name in (
            "release_contract_violation_count",
            "duplicate_safe_frame_violation_count",
            "stale_propulsion_violation_count",
            "unauthorized_restart_count",
            "actual_collision_count",
            "actual_forbidden_violation_count",
            "actual_clearance_violation_count",
        )
    )
    passed = contract_passed and (
        _normal_result_passes(result)
        if spec.profile_name == "normal"
        else _stress_result_is_conditionally_safe(result)
    )
    trace_file_sha256 = None
    trace_last_record_hash = None
    if failure_trace_root is not None:
        trace_path = failure_trace_root / spec.case_id / "tick-trace.jsonl"
        failure_trace.write_jsonl(trace_path)
        trace_file_sha256 = sha256(trace_path.read_bytes()).hexdigest()
        trace_last_record_hash = (
            failure_trace.records[-1]["record_content_hash"]
            if failure_trace.records
            else None
        )
    case_result = _case_result(
        spec,
        result,
        passed=passed,
        elapsed_s=perf_counter() - started,
        contract_proof=contract_proof,
        trace_file_sha256=trace_file_sha256,
        trace_last_record_hash=trace_last_record_hash,
    )
    return case_result


def evaluate_hidden_v4_cases(
    repository_root: Path,
    specs: tuple[R7HiddenV4CaseSpec, ...],
    *,
    max_workers: int,
    on_case: Callable[[R7HiddenV4CaseResult], None] | None = None,
    failure_trace_root: Path | None = None,
) -> tuple[R7HiddenV4CaseResult, ...]:
    if max_workers <= 0:
        raise ValueError("max_workers must be positive")
    if max_workers == 1:
        results = []
        for spec in specs:
            result = run_hidden_v4_case(repository_root, spec, failure_trace_root)
            results.append(result)
            if on_case is not None:
                on_case(result)
        return tuple(results)

    completed: dict[int, R7HiddenV4CaseResult] = {}
    with ProcessPoolExecutor(max_workers=max_workers) as pool:
        futures = {
            pool.submit(
                run_hidden_v4_case,
                repository_root,
                spec,
                failure_trace_root,
            ): spec
            for spec in specs
        }
        for future in as_completed(futures):
            result = future.result()
            completed[result.ordinal] = result
            if on_case is not None:
                on_case(result)
    return tuple(completed[index] for index in sorted(completed))


def audit_hidden_v4_results(
    specs: tuple[R7HiddenV4CaseSpec, ...],
    results: tuple[R7HiddenV4CaseResult, ...],
) -> R7HiddenV4Audit:
    failures: list[str] = []
    if len(specs) != R7_HIDDEN_V4_REQUIRED_CASE_COUNT:
        failures.append("required_hidden_v4_catalog_incomplete")
    if len(results) != len(specs):
        failures.append("hidden_v4_result_count_mismatch")
    if tuple(item.case_id for item in results) != tuple(item.case_id for item in specs):
        failures.append("hidden_v4_result_order_or_identity_mismatch")
    for spec, result in zip(specs, results, strict=False):
        if result.ordinal != spec.ordinal or result.seed_tag != spec.seed_tag:
            failures.append(f"{spec.case_id}:spec_binding_mismatch")
        if result.hard_failures:
            failures.append(f"{spec.case_id}:hard_failure")
        if not result.passed:
            failures.append(f"{spec.case_id}:expected_outcome_failed")

    normal_completed = sum(
        item.profile_name == "normal" and item.outcome == "completed" for item in results
    )
    stress_safe = sum(
        item.profile_name == "stress"
        and item.passed
        and item.outcome == "conservative_hold"
        for item in results
    )
    stress_release_count = sum(
        item.profile_name == "stress" and bool(item.release_ticks) for item in results
    )
    hard_failure_count = sum(bool(item.hard_failures) for item in results)
    proof_totals = {
        name: sum(getattr(item, name) for item in results)
        for name in (
            "release_contract_violation_count",
            "duplicate_safe_frame_violation_count",
            "stale_propulsion_violation_count",
            "unauthorized_restart_count",
            "actual_collision_count",
            "actual_forbidden_violation_count",
            "actual_clearance_violation_count",
        )
    }
    if normal_completed != 10:
        failures.append("normal_completion_count_mismatch")
    if stress_safe != 10:
        failures.append("stress_conditionally_safe_count_mismatch")
    if hard_failure_count:
        failures.append("hidden_v4_hard_failure_nonzero")
    for name, count in proof_totals.items():
        if count:
            failures.append(f"hidden_v4_{name}_nonzero")

    return R7HiddenV4Audit(
        passed=not failures,
        required_case_count=R7_HIDDEN_V4_REQUIRED_CASE_COUNT,
        result_count=len(results),
        normal_completed_count=normal_completed,
        stress_conditionally_safe_count=stress_safe,
        stress_release_count=stress_release_count,
        hard_failure_count=hard_failure_count,
        **proof_totals,
        failures=tuple(dict.fromkeys(failures)),
        result_set_hash=canonical_content_hash(
            tuple((item.case_id, item.content_hash) for item in results)
        ),
    )


def _case_result(
    spec: R7HiddenV4CaseSpec,
    result: R5CObservationDiagnosticResult,
    *,
    passed: bool,
    elapsed_s: float,
    contract_proof: dict[str, int | None],
    trace_file_sha256: str | None,
    trace_last_record_hash: str | None,
) -> R7HiddenV4CaseResult:
    return R7HiddenV4CaseResult(
        ordinal=spec.ordinal,
        case_id=spec.case_id,
        replica=spec.replica,
        side_name=spec.side_name,
        profile_name=spec.profile_name,
        seed_tag=spec.seed_tag,
        expected_outcome=spec.expected_outcome,
        passed=passed,
        outcome=result.outcome.value,
        completion_tick=result.completion_tick,
        post_pass_proof_tick=result.post_pass_proof_tick,
        follow_original_release_tick=result.follow_original_release_tick,
        actual_release_tick=result.actual_release_tick,
        first_motion_tick=result.first_motion_tick,
        protective_stop_started_tick=result.protective_stop_started_tick,
        stop_confirmed_tick=result.stop_confirmed_tick,
        controller_call_count=result.controller_call_count,
        release_ticks=result.release_ticks,
        confirmed_stop_ticks=result.confirmed_stop_ticks,
        session_stop_epochs=result.session_stop_epochs,
        final_motion_state=result.final_motion_state.value,
        final_stop_epoch=result.final_stop_epoch,
        minimum_actor_clearance_m=result.minimum_actor_clearance_m,
        minimum_static_clearance_m=result.minimum_static_clearance_m,
        gate_override_count=result.gate_override_count,
        hard_failures=result.hard_failures,
        trace_content_hash=result.trace_content_hash,
        trace_file_sha256=trace_file_sha256,
        trace_record_count=int(contract_proof["trace_record_count"] or 0),
        trace_last_record_hash=trace_last_record_hash,
        minimum_release_confirmed_safe_frames=contract_proof[
            "minimum_release_confirmed_safe_frames"
        ],
        release_contract_violation_count=int(
            contract_proof["release_contract_violation_count"] or 0
        ),
        duplicate_safe_frame_violation_count=int(
            contract_proof["duplicate_safe_frame_violation_count"] or 0
        ),
        stale_propulsion_violation_count=int(
            contract_proof["stale_propulsion_violation_count"] or 0
        ),
        unauthorized_restart_count=int(
            contract_proof["unauthorized_restart_count"] or 0
        ),
        actual_collision_count=int(contract_proof["actual_collision_count"] or 0),
        actual_forbidden_violation_count=int(
            contract_proof["actual_forbidden_violation_count"] or 0
        ),
        actual_clearance_violation_count=int(
            contract_proof["actual_clearance_violation_count"] or 0
        ),
        elapsed_s=elapsed_s,
    )


def _trace_contract_proof(
    records: tuple[dict[str, object], ...],
    result: R5CObservationDiagnosticResult,
) -> dict[str, int | None]:
    release_records = tuple(
        record for record in records if record.get("release_permitted") is True
    )
    release_safe_counts = tuple(
        int(record.get("confirmed_safe_frame_count_after", 0))
        for record in release_records
    )
    release_violations = 0
    unauthorized_restarts = 0
    for record in release_records:
        stop_epoch = record.get("stop_epoch_after")
        if not all(
            (
                record.get("release_input_usable") is True,
                record.get("last_event_was_no_frame") is False,
                record.get("observation_status") == "fresh",
                int(record.get("confirmed_safe_frame_count_after", 0))
                >= DYNAMIC_SAFE_OBSERVATION_FRAMES,
                record.get("gate_state_before") == "holding",
                record.get("runtime_present_before") is False,
                record.get("runtime_present_after") is True,
                record.get("reference_stop_epoch") == stop_epoch,
                record.get("resume_authorization_revision") is not None,
            )
        ):
            release_violations += 1
        if (
            record.get("reference_stop_epoch") != stop_epoch
            or record.get("resume_authorization_revision") is None
        ):
            unauthorized_restarts += 1

    release_ticks = tuple(int(record["tick"]) for record in release_records)
    if release_ticks != result.release_ticks:
        release_violations += 1
    for record in records:
        if (
            record.get("runtime_present_before") is False
            and record.get("runtime_present_after") is True
            and record.get("release_permitted") is not True
        ):
            unauthorized_restarts += 1

    counted_frames: set[tuple[int, int]] = set()
    duplicate_safe_frame_violations = 0
    for record in records:
        before = int(record.get("confirmed_safe_frame_count_before", 0))
        after = int(record.get("confirmed_safe_frame_count_after", 0))
        if after <= before:
            continue
        sequence = record.get("observation_sequence")
        epoch = record.get("stop_epoch_after")
        if (
            after != before + 1
            or isinstance(sequence, bool)
            or not isinstance(sequence, int)
            or isinstance(epoch, bool)
            or not isinstance(epoch, int)
            or (epoch, sequence) in counted_frames
        ):
            duplicate_safe_frame_violations += 1
            continue
        counted_frames.add((epoch, sequence))

    stale_propulsion_violations = 0
    for record in records:
        if record.get("release_input_usable") is True:
            continue
        command = record.get("command_after_gate")
        before = record.get("robot_twist_before")
        if command is None or before is None:
            continue
        if record.get("controller_called") is True or not _is_limited_deceleration(
            before,
            command,
        ):
            stale_propulsion_violations += 1

    actor_clearance = result.minimum_actor_clearance_m
    minimum_clearance = (
        result.minimum_static_clearance_m
        if actor_clearance is None
        else min(result.minimum_static_clearance_m, actor_clearance)
    )
    return {
        "trace_record_count": len(records),
        "minimum_release_confirmed_safe_frames": (
            min(release_safe_counts) if release_safe_counts else None
        ),
        "release_contract_violation_count": release_violations,
        "duplicate_safe_frame_violation_count": duplicate_safe_frame_violations,
        "stale_propulsion_violation_count": stale_propulsion_violations,
        "unauthorized_restart_count": unauthorized_restarts,
        "actual_collision_count": int(minimum_clearance <= 0.0),
        "actual_forbidden_violation_count": int(
            result.minimum_static_clearance_m <= 0.0
        ),
        "actual_clearance_violation_count": int(
            minimum_clearance + 1e-12 < R7_HIDDEN_V4_MINIMUM_CLEARANCE_M
        ),
    }


def _is_limited_deceleration(before: object, after: object) -> bool:
    if not isinstance(before, dict) or not isinstance(after, dict):
        return False
    return all(
        _component_moves_toward_zero(
            float(before[before_name]),
            float(after[after_name]),
        )
        for before_name, after_name in (
            ("linear_mps", "linear_mps"),
            ("angular_radps", "angular_radps"),
        )
    )


def _component_moves_toward_zero(before: float, after: float) -> bool:
    tolerance = 1e-12
    if abs(after) <= tolerance:
        return True
    return before * after > 0.0 and abs(after) <= abs(before) + tolerance


def _normal_result_passes(result: R5CObservationDiagnosticResult) -> bool:
    follow_release = result.follow_original_release_tick
    return all(
        (
            result.outcome is R5CDiagnosticOutcome.COMPLETED,
            _normal_progress_is_ordered(result),
            bool(result.confirmed_stop_ticks),
            follow_release is not None,
            any(tick < follow_release for tick in result.confirmed_stop_ticks),
            len(result.release_ticks) == len(result.session_stop_epochs),
            len(result.session_stop_epochs) >= 2,
            result.session_stop_epochs[-1] > result.session_stop_epochs[0],
            result.final_motion_state is DynamicMotionState.COMPLETED,
            _clearances_pass(result),
            not result.hard_failures,
        )
    )


def _stress_result_is_conditionally_safe(
    result: R5CObservationDiagnosticResult | R7HiddenV4CaseResult,
) -> bool:
    if not all(
        (
            result.outcome
            in {R5CDiagnosticOutcome.CONSERVATIVE_HOLD, "conservative_hold"},
            result.final_motion_state
            in {DynamicMotionState.HOLDING, "holding"},
            _clearances_pass(result),
            not result.hard_failures,
        )
    ):
        return False

    if result.actual_release_tick is None:
        return all(
            (
                result.first_motion_tick is None,
                result.controller_call_count == 0,
                not result.release_ticks,
                not result.session_stop_epochs,
            )
        )

    if not all(
        (
            result.release_ticks,
            result.session_stop_epochs,
            len(result.release_ticks) == len(result.session_stop_epochs),
            result.release_ticks[0] == result.actual_release_tick,
            tuple(sorted(set(result.release_ticks))) == result.release_ticks,
            result.first_motion_tick is not None,
            result.protective_stop_started_tick is not None,
            result.stop_confirmed_tick is not None,
            result.controller_call_count > 0,
            result.confirmed_stop_ticks,
        )
    ):
        return False

    assert result.first_motion_tick is not None
    assert result.protective_stop_started_tick is not None
    assert result.stop_confirmed_tick is not None
    return all(
        (
            result.actual_release_tick < result.first_motion_tick,
            result.first_motion_tick < result.protective_stop_started_tick,
            result.protective_stop_started_tick <= result.stop_confirmed_tick,
            result.stop_confirmed_tick in result.confirmed_stop_ticks,
            result.final_stop_epoch > result.session_stop_epochs[-1],
        )
    )


def _validate_root_seed(root_seed: int) -> None:
    if (
        isinstance(root_seed, bool)
        or not isinstance(root_seed, int)
        or not 0 <= root_seed <= _MAX_ROOT_SEED
    ):
        raise ValueError("root_seed must be a non-negative signed 63-bit exact integer")


def _normal_progress_is_ordered(result) -> bool:
    ticks = (
        result.first_motion_tick,
        result.post_pass_proof_tick,
        result.follow_original_release_tick,
        result.completion_tick,
    )
    return all(tick is not None for tick in ticks) and all(
        left < right for left, right in zip(ticks[:-1], ticks[1:], strict=True)
    )


def _clearances_pass(result) -> bool:
    return (
        result.minimum_actor_clearance_m is not None
        and result.minimum_actor_clearance_m + 1e-12
        >= R7_HIDDEN_V4_MINIMUM_CLEARANCE_M
        and result.minimum_static_clearance_m + 1e-12
        >= R7_HIDDEN_V4_MINIMUM_CLEARANCE_M
    )


def _derived_observation_seed(root_seed: int, *, replica: int, side_name: str) -> int:
    encoded = (
        f"{R7_HIDDEN_V4_OBSERVATION_VERSION}:{root_seed}:{replica}:{side_name}"
    ).encode()
    return int.from_bytes(sha256(encoded).digest()[:8], byteorder="big") & _MAX_ROOT_SEED


__all__ = [
    "R7_HIDDEN_V4_MINIMUM_CLEARANCE_M",
    "R7_HIDDEN_V4_OBSERVATION_VERSION",
    "R7_HIDDEN_V4_REQUIRED_CASE_COUNT",
    "R7_HIDDEN_V4_TICK_LIMIT",
    "R7HiddenV4Audit",
    "R7HiddenV4CaseResult",
    "R7HiddenV4CaseSpec",
    "audit_hidden_v4_results",
    "build_hidden_v4_case_specs",
    "evaluate_hidden_v4_cases",
    "hidden_v4_seed_commitment",
    "run_hidden_v4_case",
]
