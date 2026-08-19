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
from hospital_path_lab.dynamic_safety import DynamicMotionState
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r5c_observation_diagnostic import (
    R5CDiagnosticOutcome,
    R5CObservationDiagnosticResult,
    run_r5c_crossing_completion_diagnostic,
)

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
) -> R7HiddenV4CaseResult:
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
    passed = (
        _normal_result_passes(result)
        if spec.profile_name == "normal"
        else _stress_result_is_conditionally_safe(result)
    )
    return _case_result(spec, result, passed=passed, elapsed_s=perf_counter() - started)


def evaluate_hidden_v4_cases(
    repository_root: Path,
    specs: tuple[R7HiddenV4CaseSpec, ...],
    *,
    max_workers: int,
    on_case: Callable[[R7HiddenV4CaseResult], None] | None = None,
) -> tuple[R7HiddenV4CaseResult, ...]:
    if max_workers <= 0:
        raise ValueError("max_workers must be positive")
    if max_workers == 1:
        results = []
        for spec in specs:
            result = run_hidden_v4_case(repository_root, spec)
            results.append(result)
            if on_case is not None:
                on_case(result)
        return tuple(results)

    completed: dict[int, R7HiddenV4CaseResult] = {}
    with ProcessPoolExecutor(max_workers=max_workers) as pool:
        futures = {
            pool.submit(run_hidden_v4_case, repository_root, spec): spec for spec in specs
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
    if normal_completed != 10:
        failures.append("normal_completion_count_mismatch")
    if stress_safe != 10:
        failures.append("stress_conditionally_safe_count_mismatch")
    if hard_failure_count:
        failures.append("hidden_v4_hard_failure_nonzero")

    return R7HiddenV4Audit(
        passed=not failures,
        required_case_count=R7_HIDDEN_V4_REQUIRED_CASE_COUNT,
        result_count=len(results),
        normal_completed_count=normal_completed,
        stress_conditionally_safe_count=stress_safe,
        stress_release_count=stress_release_count,
        hard_failure_count=hard_failure_count,
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
        elapsed_s=elapsed_s,
    )


def _normal_result_passes(result: R5CObservationDiagnosticResult) -> bool:
    return all(
        (
            result.outcome is R5CDiagnosticOutcome.COMPLETED,
            _normal_progress_is_ordered(result),
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
