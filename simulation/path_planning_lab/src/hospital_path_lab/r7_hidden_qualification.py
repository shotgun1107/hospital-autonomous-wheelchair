"""새로운 관측 잡음·dropout 순서로 R6 횟단 완료 경계를 한 번 검증한다."""

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
from hospital_path_lab.dynamic_safety import DynamicMotionState
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r5c_observation_diagnostic import (
    R5CDiagnosticOutcome,
    run_r5c_crossing_completion_diagnostic,
)

R7_HIDDEN_OBSERVATION_VERSION = "r7-hidden-observation-v2"
R7_HIDDEN_REPLICA_COUNT = 5
R7_HIDDEN_REQUIRED_CASE_COUNT = 20
R7_HIDDEN_TICK_LIMIT = 1_600
R7_HIDDEN_MINIMUM_CLEARANCE_M = 0.08
_MAX_ROOT_SEED = (1 << 63) - 1


@dataclass(frozen=True, slots=True)
class R7HiddenCaseSpec:
    ordinal: int
    case_id: str
    replica: int
    side_index: int
    side_name: str
    profile_name: str
    observation_seed: int
    seed_tag: str
    expected_outcome: str
    tick_limit: int = R7_HIDDEN_TICK_LIMIT

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class R7HiddenCaseResult:
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
    first_motion_tick: int | None
    controller_call_count: int
    release_ticks: tuple[int, ...]
    final_motion_state: str
    final_stop_epoch: int
    minimum_actor_clearance_m: float | None
    minimum_static_clearance_m: float
    gate_override_count: int
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
            raise ValueError("R7 hidden case result content hash mismatch")
        object.__setattr__(self, "content_hash", expected)


@dataclass(frozen=True, slots=True)
class R7HiddenAudit:
    passed: bool
    required_case_count: int
    result_count: int
    normal_completed_count: int
    stress_holding_count: int
    hard_failure_count: int
    failures: tuple[str, ...]
    result_set_hash: str


def hidden_seed_commitment(root_seed: int) -> str:
    """Return the value written before any hidden case is constructed."""

    _validate_root_seed(root_seed)
    return sha256(f"{R7_HIDDEN_OBSERVATION_VERSION}:{root_seed}".encode()).hexdigest()


def build_hidden_case_specs(root_seed: int) -> tuple[R7HiddenCaseSpec, ...]:
    _validate_root_seed(root_seed)
    specs: list[R7HiddenCaseSpec] = []
    for replica in range(R7_HIDDEN_REPLICA_COUNT):
        for side_index, side_name in enumerate(("left", "right")):
            observation_seed = _derived_observation_seed(
                root_seed,
                replica=replica,
                side_name=side_name,
            )
            seed_tag = sha256(str(observation_seed).encode("ascii")).hexdigest()
            for profile_name, expected in (
                ("normal", "completed"),
                ("stress", "conservative_hold"),
            ):
                specs.append(
                    R7HiddenCaseSpec(
                        ordinal=len(specs),
                        case_id=(
                            f"hidden-v3-{replica:02d}-{side_name}-{profile_name}"
                        ),
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
    if len(result) != R7_HIDDEN_REQUIRED_CASE_COUNT:
        raise RuntimeError("R7 hidden catalog size mismatch")
    if len({item.case_id for item in result}) != len(result):
        raise RuntimeError("R7 hidden catalog contains duplicate case IDs")
    if tuple(item.ordinal for item in result) != tuple(range(len(result))):
        raise RuntimeError("R7 hidden catalog ordinals are not contiguous")
    for replica in range(R7_HIDDEN_REPLICA_COUNT):
        for side_name in ("left", "right"):
            paired = tuple(
                item
                for item in result
                if item.replica == replica and item.side_name == side_name
            )
            if len(paired) != 2 or paired[0].observation_seed != paired[1].observation_seed:
                raise RuntimeError("Normal and Stress must share one latent observation seed")
    return result


def run_hidden_case(
    _repository_root: Path,
    spec: R7HiddenCaseSpec,
) -> R7HiddenCaseResult:
    started = perf_counter()
    profile = (
        NORMAL_OBSERVATION_PROFILE
        if spec.profile_name == "normal"
        else STRESS_OBSERVATION_PROFILE
    )
    result = run_r5c_crossing_completion_diagnostic(
        side_index=spec.side_index,
        profile=profile,
        tick_limit=spec.tick_limit,
        observation_seed=spec.observation_seed,
    )
    if spec.expected_outcome == "completed":
        ordered_progress = _normal_progress_is_ordered(result)
        passed = all(
            (
                result.outcome is R5CDiagnosticOutcome.COMPLETED,
                ordered_progress,
                result.final_motion_state is DynamicMotionState.COMPLETED,
                _clearances_pass(result),
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
                _clearances_pass(result),
                not result.hard_failures,
            )
        )
    return R7HiddenCaseResult(
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
        first_motion_tick=result.first_motion_tick,
        controller_call_count=result.controller_call_count,
        release_ticks=result.release_ticks,
        final_motion_state=result.final_motion_state.value,
        final_stop_epoch=result.final_stop_epoch,
        minimum_actor_clearance_m=result.minimum_actor_clearance_m,
        minimum_static_clearance_m=result.minimum_static_clearance_m,
        gate_override_count=result.gate_override_count,
        hard_failures=result.hard_failures,
        trace_content_hash=result.trace_content_hash,
        elapsed_s=perf_counter() - started,
    )


def evaluate_hidden_cases(
    repository_root: Path,
    specs: tuple[R7HiddenCaseSpec, ...],
    *,
    max_workers: int,
    on_case: Callable[[R7HiddenCaseResult], None] | None = None,
) -> tuple[R7HiddenCaseResult, ...]:
    if max_workers <= 0:
        raise ValueError("max_workers must be positive")
    if max_workers == 1:
        results = []
        for spec in specs:
            result = run_hidden_case(repository_root, spec)
            results.append(result)
            if on_case is not None:
                on_case(result)
        return tuple(results)

    completed: dict[int, R7HiddenCaseResult] = {}
    with ProcessPoolExecutor(max_workers=max_workers) as pool:
        futures = {
            pool.submit(run_hidden_case, repository_root, spec): spec for spec in specs
        }
        for future in as_completed(futures):
            result = future.result()
            completed[result.ordinal] = result
            if on_case is not None:
                on_case(result)
    return tuple(completed[index] for index in sorted(completed))


def audit_hidden_results(
    specs: tuple[R7HiddenCaseSpec, ...],
    results: tuple[R7HiddenCaseResult, ...],
) -> R7HiddenAudit:
    failures: list[str] = []
    if len(specs) != R7_HIDDEN_REQUIRED_CASE_COUNT:
        failures.append("required_hidden_catalog_incomplete")
    if len(results) != len(specs):
        failures.append("hidden_result_count_mismatch")
    if tuple(item.case_id for item in results) != tuple(item.case_id for item in specs):
        failures.append("hidden_result_order_or_identity_mismatch")
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
    stress_holding = sum(
        item.profile_name == "stress" and item.outcome == "conservative_hold"
        for item in results
    )
    hard_failure_count = sum(bool(item.hard_failures) for item in results)
    if normal_completed != 10:
        failures.append("normal_completion_count_mismatch")
    if stress_holding != 10:
        failures.append("stress_holding_count_mismatch")
    if hard_failure_count:
        failures.append("hidden_hard_failure_nonzero")

    return R7HiddenAudit(
        passed=not failures,
        required_case_count=R7_HIDDEN_REQUIRED_CASE_COUNT,
        result_count=len(results),
        normal_completed_count=normal_completed,
        stress_holding_count=stress_holding,
        hard_failure_count=hard_failure_count,
        failures=tuple(dict.fromkeys(failures)),
        result_set_hash=canonical_content_hash(
            tuple((item.case_id, item.content_hash) for item in results)
        ),
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
        >= R7_HIDDEN_MINIMUM_CLEARANCE_M
        and result.minimum_static_clearance_m + 1e-12
        >= R7_HIDDEN_MINIMUM_CLEARANCE_M
    )


def _derived_observation_seed(root_seed: int, *, replica: int, side_name: str) -> int:
    encoded = (
        f"{R7_HIDDEN_OBSERVATION_VERSION}:{root_seed}:{replica}:{side_name}"
    ).encode()
    return int.from_bytes(sha256(encoded).digest()[:8], byteorder="big") & _MAX_ROOT_SEED
