"""재현 가능한 지도 corpus 전체를 직렬 평가하고 증거 파일로 저장한다.

이 모듈의 결과는 ``simulation_only``인 Python 비교 증거다. 실제 차체의
안전성이나 최종 알고리즘 채택을 주장하지 않는다.
"""

from __future__ import annotations

import json
import tracemalloc
from collections import Counter, defaultdict
from collections.abc import Callable
from copy import deepcopy
from dataclasses import asdict, dataclass
from functools import partial
from hashlib import sha256
from heapq import heappop, heappush
from math import atan2, inf, isclose, isfinite
from pathlib import Path
from statistics import fmean, median
from time import perf_counter_ns
from typing import TypeVar

import networkx as nx
import numpy as np

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import (
    GridSnapshot,
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
)
from hospital_path_lab.evaluation import (
    authorize_after_protective_stop,
    route_churn,
    run_stateless_global,
    validate_follower_result,
    validate_global_result,
    validate_local_result,
    validate_result_provenance,
)
from hospital_path_lab.experiment_visualization import (
    save_graph_experiment_plot,
    save_grid_experiment_plot,
)
from hospital_path_lab.graph import canonical_edge
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_algorithms import DynamicWindowPlanner
from hospital_path_lab.local_algorithms.grid_astar import reference_search_bounds
from hospital_path_lab.map_factory import (
    CorpusSplit,
    GeneratedCase,
    build_graph_snapshot,
    build_grid_snapshot,
    episode_state_at,
    freeze_batch,
    generate_batch,
    generate_golden_cases,
    validate_frozen_batch,
)
from hospital_path_lab.planners import SearchResult
from hospital_path_lab.registry import (
    GLOBAL_INCREMENTAL,
    GLOBAL_STATELESS,
    LOCAL_PLANNERS,
    PATH_FOLLOWERS,
    algorithm_manifest,
)
from hospital_path_lab.safety import AutomaticResumeGate
from hospital_path_lab.simulation import (
    SimulationResult,
    simulate_dynamic_local_evidence,
    simulate_follower,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1

_T = TypeVar("_T")

_FOLLOWER_PATH_TIME_FACTOR = 2.5
_FOLLOWER_SETTLING_ALLOWANCE_S = 10.0


@dataclass(frozen=True, slots=True)
class ExperimentConfig:
    """고정 corpus 두 묶음과 직렬 시험 실행 설정."""

    base_seed: int = 20_260_810
    hidden_seed: int = 20_260_811
    batch_size: int = 10
    follower_max_time_s: float = 30.0
    regression_input_dir: str | None = None
    regression_input_limit: int | None = None
    # 아래 값은 이 Python 연구 실행기의 이상 감지 기준일 뿐 제품 deadline이 아니다.
    global_deadline_ns: int = 5_000_000_000
    local_deadline_ns: int = 30_000_000_000
    path_follower_deadline_ns: int = 60_000_000_000

    def __post_init__(self) -> None:
        if self.batch_size != 10:
            raise ValueError("experiment_runner v1의 batch_size는 10이어야 합니다.")
        if self.base_seed == self.hidden_seed:
            raise ValueError("hidden_seed는 base_seed와 분리해야 합니다.")
        if self.follower_max_time_s <= 0:
            raise ValueError("follower_max_time_s는 양수여야 합니다.")
        if self.regression_input_dir is not None and not self.regression_input_dir.strip():
            raise ValueError("regression_input_dir는 비어 있을 수 없습니다.")
        if self.regression_input_limit is not None and (
            isinstance(self.regression_input_limit, bool)
            or not isinstance(self.regression_input_limit, int)
            or self.regression_input_limit < 0
        ):
            raise ValueError("regression_input_limit은 0 이상의 정수 또는 None이어야 합니다.")
        if (
            min(
                self.global_deadline_ns,
                self.local_deadline_ns,
                self.path_follower_deadline_ns,
            )
            <= 0
        ):
            raise ValueError("simulation-only deadline_ns는 모두 양수여야 합니다.")


@dataclass(frozen=True, slots=True)
class ExperimentRunResult:
    output_dir: Path
    results_path: Path
    pareto_path: Path
    summary_path: Path
    visualization_paths: tuple[Path, ...]
    case_count: int
    split_counts: dict[str, int]
    hard_failures: tuple[dict[str, object], ...]


@dataclass(frozen=True, slots=True)
class _Measured:
    value: object
    elapsed_ns: int
    peak_memory_bytes: int
    memory_profiled: bool = True


@dataclass(frozen=True, slots=True)
class _Metric:
    elapsed_ns: int
    peak_memory_bytes: int
    passed: bool
    collision: bool
    success: bool
    deterministic: bool = True
    expanded_nodes: int | None = None
    route_churn: float | None = None
    minimum_clearance_m: float | None = None
    mean_tracking_error_m: float | None = None
    maximum_tracking_error_m: float | None = None
    jerk_rms_mps3: float | None = None
    additional_distance_m: float | None = None
    overshoot_m: float | None = None
    deadline_miss: bool = False


def run_experiment(
    output_dir: str | Path,
    config: ExperimentConfig | None = None,
) -> ExperimentRunResult:
    """10개 corpus를 실행하고 JSON, Pareto 요약, Markdown, PNG를 저장한다."""

    selected_config = config or ExperimentConfig()
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)

    corpus, freeze_evidence = _select_corpus(selected_config)
    split_counts = Counter(item.episode.split.value for item, _ in corpus)
    metrics: dict[str, list[_Metric]] = defaultdict(list)
    hard_failures: list[dict[str, object]] = []
    limitations: list[dict[str, object]] = _coverage_limitations()
    stale_checks: list[dict[str, object]] = []
    case_records: list[dict[str, object]] = []
    hidden_plots: list[Path] = []
    safety_source: tuple[object, str, str, SearchResult] | None = None
    memory_profiled: set[tuple[str, str]] = set()

    for case, source_batch_seed in corpus:
        case_record, plot_payloads, safety_candidate = _run_case(
            case,
            source_batch_seed=source_batch_seed,
            config=selected_config,
            metrics=metrics,
            hard_failures=hard_failures,
            limitations=limitations,
            stale_checks=stale_checks,
            memory_profiled=memory_profiled,
        )
        case_records.append(case_record)
        if safety_source is None and safety_candidate is not None:
            safety_source = safety_candidate
        if case.episode.split is CorpusSplit.HIDDEN:
            hidden_plots.extend(_save_hidden_plots(output, case, plot_payloads))

    dynamic_local_evidence = _dynamic_local_contract_evidence(hard_failures)
    safety_evidence = _protective_stop_evidence(safety_source, hard_failures)
    regression_candidates = _preserve_hidden_failures(
        output,
        corpus,
        hard_failures,
        limitations,
        freeze_sha256=str(freeze_evidence["freeze_sha256"]),
    )
    stale_evidence = {
        "checks": stale_checks,
        "check_count": len(stale_checks),
        "rejected_count": sum(bool(check["rejected"]) for check in stale_checks),
        "all_rejected": bool(stale_checks)
        and all(bool(check["rejected"]) for check in stale_checks),
    }

    manifest = algorithm_manifest()
    results_document = {
        "schema_version": "1.0",
        "evidence_scope": "simulation_only_python_algorithm_comparison",
        "config": asdict(selected_config),
        "vehicle_profile": asdict(VIRTUAL_DOLL_WHEELCHAIR_V0_1),
        "algorithm_manifest": manifest,
        "evaluation_coverage": _evaluation_coverage(
            case_records,
            hidden_visualization_count=len(hidden_plots),
            config=selected_config,
        ),
        "freeze_evidence": freeze_evidence,
        "split_counts": dict(sorted(split_counts.items())),
        "corpus": [_corpus_record(case, seed) for case, seed in corpus],
        "cases": case_records,
        "stale_result_evidence": stale_evidence,
        "dynamic_local_evidence": dynamic_local_evidence,
        "protective_stop_evidence": safety_evidence,
        "limitations": limitations,
        "hard_failures": hard_failures,
        "regression_candidates": regression_candidates,
        "visualizations": [str(path.relative_to(output)) for path in hidden_plots],
    }
    pareto_document = {
        "schema_version": "1.0",
        "comparison_policy": "same_role_only_no_composite_ranking",
        "roles": _aggregate_by_role(metrics, manifest),
        "pipelines": _aggregate_pipelines(case_records),
        "hard_failure_count": len(hard_failures),
        "limitation_count": len(limitations),
    }

    results_path = output / "experiment_results.json"
    pareto_path = output / "pareto.json"
    summary_path = output / "summary.md"
    _write_json(results_path, results_document)
    _write_json(pareto_path, pareto_document)
    summary_path.write_text(
        _markdown_summary(
            selected_config,
            split_counts,
            pareto_document,
            hard_failures,
            limitations,
            hidden_plots,
        ),
        encoding="utf-8",
    )
    return ExperimentRunResult(
        output_dir=output,
        results_path=results_path,
        pareto_path=pareto_path,
        summary_path=summary_path,
        visualization_paths=tuple(hidden_plots),
        case_count=len(corpus),
        split_counts=dict(sorted(split_counts.items())),
        hard_failures=tuple(hard_failures),
    )


def _select_corpus(
    config: ExperimentConfig,
) -> tuple[tuple[tuple[GeneratedCase, int], ...], dict[str, object]]:
    base_batch = generate_batch(base_seed=config.base_seed, size=config.batch_size)
    golden_cases = [(case, case.world.seed) for case in generate_golden_cases()]
    generated_public_cases = [
        (case, config.base_seed)
        for case in base_batch
        if case.episode.split in {CorpusSplit.DEVELOPMENT, CorpusSplit.REGRESSIONS}
    ]
    promoted_cases: list[tuple[GeneratedCase, int]] = []
    if config.regression_input_dir is not None:
        from hospital_path_lab.corpus_records import load_promoted_regressions

        promoted_cases = [
            (case, case.world.seed)
            for case in load_promoted_regressions(
                config.regression_input_dir,
                limit=config.regression_input_limit,
            )
        ]
    public_cases = golden_cases + generated_public_cases + promoted_cases

    # 알고리즘과 공개 corpus를 먼저 동결한 뒤에만 별도 seed의 hidden을 생성한다.
    # 이 순서 자체가 deterministic evidence에 포함되며 wall-clock 값은 넣지 않는다.
    freeze_evidence = _freeze_evidence(tuple(public_cases))
    hidden_batch = generate_batch(base_seed=config.hidden_seed, size=config.batch_size)
    hidden_cases = [
        (case, config.hidden_seed)
        for case in hidden_batch
        if case.episode.split is CorpusSplit.HIDDEN
    ]
    corpus = tuple(public_cases + hidden_cases)
    counts = Counter(case.episode.split for case, _ in corpus)
    expected = {
        CorpusSplit.GOLDEN: 12,
        CorpusSplit.DEVELOPMENT: 4,
        CorpusSplit.HIDDEN: 2,
        CorpusSplit.REGRESSIONS: 2 + len(promoted_cases),
    }
    if len(corpus) != 20 + len(promoted_cases) or counts != Counter(expected):
        raise RuntimeError(f"corpus split 선택이 잘못됐습니다: {dict(counts)}")
    hashes = [case.episode.content_hash for case, _ in corpus]
    if len(hashes) != len(set(hashes)):
        raise RuntimeError("base/hidden corpus의 episode hash가 중복됩니다.")
    freeze_evidence["hidden_selection"] = {
        "selected_after_freeze": True,
        "source_batch_seed": config.hidden_seed,
        "case_ids": [case.episode.episode_id for case, _ in hidden_cases],
        "episode_hashes": [case.episode.content_hash for case, _ in hidden_cases],
    }
    freeze_evidence["promoted_regressions"] = {
        "loaded_before_hidden_selection": True,
        "input_enabled": config.regression_input_dir is not None,
        "requested_limit": config.regression_input_limit,
        "case_count": len(promoted_cases),
        "case_ids": [case.episode.episode_id for case, _ in promoted_cases],
        "episode_hashes": [case.episode.content_hash for case, _ in promoted_cases],
    }
    return corpus, freeze_evidence


def _freeze_evidence(
    public_cases: tuple[tuple[GeneratedCase, int], ...],
) -> dict[str, object]:
    """hidden 생성 전에 알고리즘 소스와 공개 corpus 목록을 동결한다."""

    source_root = Path(__file__).resolve().parent
    source_files = []
    for path in sorted(source_root.rglob("*.py"), key=lambda item: item.as_posix()):
        source_files.append(
            {
                "path": path.relative_to(source_root).as_posix(),
                "sha256": sha256(path.read_bytes()).hexdigest(),
            }
        )
    source_tree_hash = _canonical_sha256(source_files)
    public_batch = tuple(case for case, _ in public_cases)
    frozen_contract = freeze_batch(
        public_batch,
        corpus_id="path_planning_public_pre_hidden_v1",
    )
    validate_frozen_batch(public_batch, frozen_contract)
    frozen_corpus = [_corpus_record(case, seed) for case, seed in public_cases]
    frozen_corpus_hash = _canonical_sha256(frozen_corpus)
    freeze_hash = _canonical_sha256(
        {
            "algorithm_source_tree_sha256": source_tree_hash,
            "frozen_public_corpus_sha256": frozen_corpus_hash,
        }
    )
    return {
        "schema_version": "1.0",
        "capture_order": [
            "algorithm_source_tree",
            "frozen_public_corpus",
            "freeze_hash",
            "hidden_generation_and_selection",
        ],
        "algorithm_source_tree": {
            "root": "hospital_path_lab",
            "sha256": source_tree_hash,
            "files": source_files,
        },
        "frozen_public_corpus": {
            "sha256": frozen_corpus_hash,
            "contract_content_hash": frozen_contract.content_hash,
            "contract_case_count": len(frozen_contract.cases),
            "cases": frozen_corpus,
        },
        "freeze_sha256": freeze_hash,
    }


def _canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return sha256(encoded).hexdigest()


def _preserve_hidden_failures(
    output: Path,
    corpus: tuple[tuple[GeneratedCase, int], ...],
    hard_failures: list[dict[str, object]],
    limitations: list[dict[str, object]],
    *,
    freeze_sha256: str,
) -> list[dict[str, object]]:
    """hidden hard failure를 현재 실행 중 튜닝하지 않고 회귀 후보로 보존한다."""

    hidden_by_id = {
        case.episode.episode_id: case
        for case, _ in corpus
        if case.episode.split is CorpusSplit.HIDDEN
    }
    hidden_failures = [failure for failure in hard_failures if failure["case_id"] in hidden_by_id]
    if not hidden_failures:
        return []
    try:
        from hospital_path_lab.corpus_records import preserve_hidden_failure
    except ImportError as exc:  # pragma: no cover - 선택 모듈 배포 누락 방어선
        limitations.append(
            {
                "case_id": "experiment",
                "step": -1,
                "algorithm": "regression_candidate_persistence",
                "reason": f"corpus_records_import_failed:{type(exc).__name__}:{exc}",
            }
        )
        return []

    manifest: list[dict[str, object]] = []
    for failure in hidden_failures:
        case = hidden_by_id[str(failure["case_id"])]
        failure_type = _safe_path_component(str(failure["type"]))
        algorithm = _safe_path_component(str(failure["algorithm"]))
        target = output / "regression_candidates" / failure_type / algorithm
        try:
            path = preserve_hidden_failure(
                case,
                reason=f"{failure['type']}:{failure['algorithm']}",
                failing_step=int(failure["step"]),
                output_directory=target,
                minimal_evidence={
                    "failure": failure,
                    "freeze_sha256": freeze_sha256,
                    "retuned_during_hidden_run": False,
                },
            )
            created = True
        except FileExistsError:
            # 기존 증거를 덮어쓰지 않는다. 동일 경로만 manifest에 재참조한다.
            path = target / (
                f"regression_{case.world.content_hash[:12]}_"
                f"{case.episode.content_hash[:12]}_step_{int(failure['step']):03d}.json"
            )
            created = False
        except Exception as exc:  # pragma: no cover - 증거 보존 실패를 limitation으로 노출
            limitations.append(
                _limitation(
                    case,
                    int(failure["step"]),
                    str(failure["algorithm"]),
                    f"regression_candidate_persistence_failed:{type(exc).__name__}:{exc}",
                )
            )
            continue
        if not path.is_file():
            limitations.append(
                _limitation(
                    case,
                    int(failure["step"]),
                    str(failure["algorithm"]),
                    "regression_candidate_existing_path_ambiguous",
                )
            )
            continue
        manifest.append(
            {
                "case_id": failure["case_id"],
                "step": failure["step"],
                "algorithm": failure["algorithm"],
                "failure_type": failure["type"],
                "path": path.relative_to(output).as_posix(),
                "sha256": sha256(path.read_bytes()).hexdigest(),
                "created": created,
                "retuned_during_hidden_run": False,
            }
        )
    return manifest


def _safe_path_component(value: str) -> str:
    sanitized = "".join(character if character.isalnum() else "_" for character in value)
    return sanitized.strip("_") or "unknown"


def _episode_step_range(episode: object) -> range:
    """initial step 0부터 episode가 가진 마지막 event step까지 평가한다."""

    final_step = max((event.step for event in episode.events), default=0)
    return range(final_step + 1)


def _run_case(
    case: GeneratedCase,
    *,
    source_batch_seed: int,
    config: ExperimentConfig,
    metrics: dict[str, list[_Metric]],
    hard_failures: list[dict[str, object]],
    limitations: list[dict[str, object]],
    stale_checks: list[dict[str, object]],
    memory_profiled: set[tuple[str, str]],
) -> tuple[
    dict[str, object],
    tuple[dict[str, object], ...],
    tuple[object, str, str, SearchResult] | None,
]:
    dstar = GLOBAL_INCREMENTAL["dstar_lite"]()
    previous: dict[str, tuple[object, str, str, SearchResult]] = {}
    previous_non_global: dict[str, tuple[int, str, object, object]] = {}
    steps: list[dict[str, object]] = []
    plot_payloads: list[dict[str, object]] = []
    safety_candidate: tuple[object, str, str, SearchResult] | None = None

    for step in _episode_step_range(case.episode):
        state = episode_state_at(case.episode, through_step=step)
        graph_snapshot = build_graph_snapshot(case.world, case.episode, through_step=step)
        grid_snapshot = build_grid_snapshot(case.world, case.episode, through_step=step)
        oracle_status, oracle_cost = _networkx_oracle(graph_snapshot, state.start, state.goal)
        metadata = _metadata_record(graph_snapshot.metadata)

        stale_for_step = _stale_checks_for_step(
            previous,
            current_metadata=graph_snapshot.metadata,
            case_id=case.episode.episode_id,
            step=step,
            hard_failures=hard_failures,
        )
        stale_for_step.extend(
            _stale_non_global_checks_for_step(
                previous_non_global,
                current_metadata=graph_snapshot.metadata,
                case_id=case.episode.episode_id,
                step=step,
                hard_failures=hard_failures,
            )
        )
        stale_checks.extend(stale_for_step)

        global_records: list[dict[str, object]] = []
        current_results: dict[str, SearchResult] = {}
        planners: tuple[tuple[str, Callable[[], SearchResult]], ...] = (
            (
                "dijkstra",
                partial(
                    run_stateless_global,
                    GLOBAL_STATELESS["dijkstra"](),
                    graph_snapshot,
                    state.start,
                    state.goal,
                ),
            ),
            (
                "astar",
                partial(
                    run_stateless_global,
                    GLOBAL_STATELESS["astar"](),
                    graph_snapshot,
                    state.start,
                    state.goal,
                ),
            ),
            (
                "dstar_lite",
                partial(
                    dstar.reset if step == 0 else dstar.replan,
                    graph_snapshot,
                    state.start,
                    state.goal,
                ),
            ),
        )
        if not state.input_valid:
            for name, invoke in planners:
                try:
                    measured = _measure(
                        invoke,
                        profile_memory=_claim_memory_profile(
                            memory_profiled, name, case
                        ),
                    )
                    result = measured.value
                    assert isinstance(result, SearchResult)
                    validation = validate_global_result(
                        graph_snapshot, state.start, state.goal, result
                    )
                    rejected = result.status is PlanStatus.INVALID_INPUT
                    deadline_miss = _record_deadline_miss(
                        measured,
                        config.global_deadline_ns,
                        hard_failures,
                        case,
                        step,
                        name,
                    )
                    if not rejected:
                        _hard_failure(
                            hard_failures,
                            "invalid_input_accepted",
                            case,
                            step,
                            name,
                            "input_valid=False snapshot을 planner가 직접 거부하지 않음",
                        )
                    global_records.append(
                        _global_record(
                            result,
                            peak_memory_bytes=measured.peak_memory_bytes,
                            validation=_validation_record(validation),
                            oracle_status=oracle_status,
                            oracle_cost=oracle_cost,
                            oracle_matched=rejected,
                            path_churn=None,
                            measured_elapsed_ns=measured.elapsed_ns,
                            deadline_ns=config.global_deadline_ns,
                            deadline_miss=deadline_miss,
                        )
                    )
                    metrics[name].append(
                        _Metric(
                            measured.elapsed_ns,
                            measured.peak_memory_bytes,
                            rejected
                            and _result_matches_metadata(result, graph_snapshot.metadata)
                            and not deadline_miss,
                            False,
                            False,
                            deadline_miss=deadline_miss,
                        )
                    )
                    current_results[name] = result
                except Exception as exc:  # pragma: no cover - 증거 보존용 방어선
                    _hard_failure(
                        hard_failures,
                        "exception",
                        case,
                        step,
                        name,
                        f"{type(exc).__name__}: {exc}",
                    )
                    global_records.append(_exception_record(name, exc, metadata))
                    metrics[name].append(_Metric(0, 0, False, False, False))
        else:
            for name, invoke in planners:
                try:
                    dstar_replay = deepcopy(dstar) if name == "dstar_lite" else None
                    measured = _measure(
                        invoke,
                        profile_memory=_claim_memory_profile(
                            memory_profiled, name, case
                        ),
                    )
                    result = measured.value
                    assert isinstance(result, SearchResult)
                    if dstar_replay is not None:
                        repeated = (
                            dstar_replay.reset(graph_snapshot, state.start, state.goal)
                            if step == 0
                            else dstar_replay.replan(
                                graph_snapshot,
                                state.start,
                                state.goal,
                            )
                        )
                    else:
                        repeated = run_stateless_global(
                            GLOBAL_STATELESS[name](),
                            graph_snapshot,
                            state.start,
                            state.goal,
                        )
                    deterministic = _search_signature(result) == _search_signature(repeated)
                    validation = validate_global_result(
                        graph_snapshot, state.start, state.goal, result
                    )
                    previous_path = previous[name][3].path if name in previous else ()
                    path_churn = route_churn(previous_path, result.path)
                    oracle_matched = result.status is oracle_status and (
                        result.cost is None
                        if oracle_cost is None
                        else result.cost is not None
                        and isclose(result.cost, oracle_cost, rel_tol=1e-9)
                    )
                    finite = _search_result_is_finite(result)
                    if not oracle_matched:
                        _hard_failure(
                            hard_failures,
                            "oracle_mismatch",
                            case,
                            step,
                            name,
                            f"{result.status.value}/{result.cost} != "
                            f"{oracle_status.value}/{oracle_cost}",
                        )
                    if not finite:
                        _hard_failure(
                            hard_failures,
                            "non_finite",
                            case,
                            step,
                            name,
                            "전역 결과에 NaN/inf가 포함됨",
                        )
                    if not deterministic:
                        _hard_failure(
                            hard_failures,
                            "non_deterministic",
                            case,
                            step,
                            name,
                            "동일 상태·seed 재실행 결과가 달라짐",
                        )
                    deadline_miss = _record_deadline_miss(
                        measured,
                        config.global_deadline_ns,
                        hard_failures,
                        case,
                        step,
                        name,
                    )
                    passed = (
                        validation.passed
                        and oracle_matched
                        and finite
                        and deterministic
                        and not deadline_miss
                    )
                    record = _global_record(
                        result,
                        peak_memory_bytes=measured.peak_memory_bytes,
                        validation=_validation_record(validation),
                        oracle_status=oracle_status,
                        oracle_cost=oracle_cost,
                        oracle_matched=oracle_matched,
                        measured_elapsed_ns=measured.elapsed_ns,
                        path_churn=path_churn,
                        deterministic=deterministic,
                        deadline_ns=config.global_deadline_ns,
                        deadline_miss=deadline_miss,
                    )
                    if name == "dstar_lite":
                        record["incremental_state"] = {
                            "reset_count": dstar.reset_count,
                            "state_reuse_count": dstar.state_reuse_count,
                            "total_expanded_nodes": dstar.total_expanded_nodes,
                            "changed_arcs": [list(arc) for arc in dstar.last_changed_arcs],
                        }
                    global_records.append(record)
                    metrics[name].append(
                        _Metric(
                            measured.elapsed_ns,
                            measured.peak_memory_bytes,
                            passed,
                            False,
                            result.status is PlanStatus.FOUND,
                            deterministic=deterministic,
                            expanded_nodes=result.expanded_nodes,
                            route_churn=path_churn,
                            deadline_miss=deadline_miss,
                        )
                    )
                    current_results[name] = result
                except Exception as exc:  # pragma: no cover - 증거 보존용 방어선
                    _hard_failure(
                        hard_failures,
                        "exception",
                        case,
                        step,
                        name,
                        f"{type(exc).__name__}: {exc}",
                    )
                    global_records.append(_exception_record(name, exc, metadata))
                    metrics[name].append(_Metric(0, 0, False, False, False))

        local_records: list[dict[str, object]] = []
        follower_records: list[dict[str, object]] = []
        follower_command_results: dict[str, object] = {}
        astar_result = current_results.get("astar")
        reference_path: tuple[Pose2D, ...] = ()
        grid_result = None
        dwa_result = None
        follower_trace: tuple[Pose2D, ...] = ()
        if (
            state.input_valid
            and astar_result is not None
            and astar_result.status is PlanStatus.FOUND
        ):
            reference_path = _node_path_to_poses(case, astar_result.path)
            initial_state = RobotState(pose=reference_path[0])
            goal = reference_path[-1]
            grid_oracle_status, grid_oracle_cost = _grid_dijkstra_oracle(
                grid_snapshot,
                reference_path,
                initial_state.pose,
                goal,
            )
            # 동일한 snapshot/reference/profile을 사용해 모든 호환 step에서 비교한다.
            local_names = ("grid_astar", "dwa")
            for name in local_names:
                planner = LOCAL_PLANNERS[name]()
                try:
                    measured = _measure(
                        partial(
                            planner.plan,
                            grid_snapshot,
                            reference_path,
                            initial_state,
                            goal,
                        ),
                        profile_memory=_claim_memory_profile(
                            memory_profiled, name, case
                        ),
                    )
                    result = measured.value
                    repeated = planner.plan(
                        grid_snapshot,
                        reference_path,
                        initial_state,
                        goal,
                    )
                    deterministic = _local_signature(result) == _local_signature(repeated)
                    validation = validate_local_result(
                        grid_snapshot,
                        initial_state.pose,
                        goal,
                        result,
                        require_goal=name == "grid_astar",
                    )
                    hard_safety_failures = _local_hard_safety_failures(
                        result_collision=result.collision,
                        validation_failures=validation.failures,
                    )
                    collision = "collision" in hard_safety_failures
                    forbidden_entry = "forbidden_zone_entry" in hard_safety_failures
                    finite = _local_result_is_finite(result)
                    oracle_matched = name != "grid_astar" or (
                        result.status is grid_oracle_status
                        and (
                            result.cost is None
                            if grid_oracle_cost is None
                            else result.cost is not None
                            and isclose(result.cost, grid_oracle_cost, rel_tol=1e-9)
                        )
                    )
                    if collision:
                        _hard_failure(
                            hard_failures,
                            "collision",
                            case,
                            step,
                            name,
                            "local 경로/궤적 충돌",
                        )
                    if forbidden_entry:
                        _hard_failure(
                            hard_failures,
                            "forbidden_zone_entry",
                            case,
                            step,
                            name,
                            "local 경로/궤적이 승인되지 않은 금지구역에 진입",
                        )
                    if name == "grid_astar" and not oracle_matched:
                        _hard_failure(
                            hard_failures,
                            "oracle_mismatch",
                            case,
                            step,
                            name,
                            f"grid_astar={result.status.value}/{result.cost}, "
                            f"grid_dijkstra={grid_oracle_status.value}/{grid_oracle_cost}",
                        )
                    if not finite:
                        _hard_failure(
                            hard_failures,
                            "non_finite",
                            case,
                            step,
                            name,
                            "local 결과에 NaN/inf가 포함됨",
                        )
                    if not deterministic:
                        _hard_failure(
                            hard_failures,
                            "non_deterministic",
                            case,
                            step,
                            name,
                            "동일 상태·seed 재실행 결과가 달라짐",
                        )
                    deadline_miss = _record_deadline_miss(
                        measured,
                        config.local_deadline_ns,
                        hard_failures,
                        case,
                        step,
                        name,
                    )
                    status_passed = (
                        result.status is grid_oracle_status
                        if name == "grid_astar"
                        else result.status is PlanStatus.FOUND
                    )
                    passed = all(
                        (
                            validation.passed,
                            oracle_matched,
                            finite,
                            not collision,
                            deterministic,
                            status_passed,
                            not deadline_miss,
                        )
                    )
                    local_records.append(
                        _local_record(
                            result,
                            measured,
                            validation=_validation_record(validation),
                            oracle_matched=oracle_matched if name == "grid_astar" else None,
                            grid_oracle=(grid_oracle_status, grid_oracle_cost)
                            if name == "grid_astar"
                            else None,
                            deterministic=deterministic,
                            deadline_ns=config.local_deadline_ns,
                            deadline_miss=deadline_miss,
                        )
                    )
                    metrics[name].append(
                        _Metric(
                            measured.elapsed_ns,
                            measured.peak_memory_bytes,
                            passed,
                            collision,
                            result.status is PlanStatus.FOUND,
                            deterministic=deterministic,
                            minimum_clearance_m=result.minimum_clearance,
                            deadline_miss=deadline_miss,
                        )
                    )
                    if name == "grid_astar":
                        grid_result = result
                    else:
                        dwa_result = result
                    if result.status is PlanStatus.NO_PATH:
                        limitations.append(
                            _limitation(case, step, name, "expected_or_conservative_no_path")
                        )
                except Exception as exc:  # pragma: no cover - 증거 보존용 방어선
                    _hard_failure(
                        hard_failures,
                        "exception",
                        case,
                        step,
                        name,
                        f"{type(exc).__name__}: {exc}",
                    )
                    local_records.append(_exception_record(name, exc, metadata))
                    metrics[name].append(_Metric(0, 0, False, False, False))

            if grid_result is not None and grid_result.status is PlanStatus.FOUND:
                follower_time_budget_s = _follower_time_budget_s(
                    grid_result.path,
                    floor_s=config.follower_max_time_s,
                )
                expected_class = _follower_expected_class(
                    case,
                    input_valid=state.input_valid,
                    grid_oracle_status=grid_oracle_status,
                    grid_result=grid_result,
                )
                for name in ("pure_pursuit", "rpp"):
                    follower = PATH_FOLLOWERS[name]()
                    try:
                        initial_command_result = follower.step(
                            grid_result.path,
                            RobotState(pose=grid_result.path[0]),
                            grid_snapshot.metadata,
                        )
                        initial_command_validation = validate_follower_result(
                            grid_snapshot.metadata,
                            initial_command_result,
                        )
                        follower_command_results[name] = initial_command_result
                        if not initial_command_validation.passed:
                            _hard_failure(
                                hard_failures,
                                "follower_validation_failure",
                                case,
                                step,
                                name,
                                ",".join(initial_command_validation.failures),
                            )
                        measured = _measure(
                            partial(
                                simulate_follower,
                                follower,
                                grid_result.path,
                                grid_snapshot,
                                RobotState(pose=grid_result.path[0]),
                                grid_result.path[-1],
                                max_time_s=follower_time_budget_s,
                            ),
                            profile_memory=_claim_memory_profile(
                                memory_profiled, name, case
                            ),
                        )
                        simulation = measured.value
                        assert isinstance(simulation, SimulationResult)
                        repeated = simulate_follower(
                            PATH_FOLLOWERS[name](),
                            grid_result.path,
                            grid_snapshot,
                            RobotState(pose=grid_result.path[0]),
                            grid_result.path[-1],
                            max_time_s=follower_time_budget_s,
                        )
                        deterministic = _simulation_signature(simulation) == _simulation_signature(
                            repeated
                        )
                        finite = _simulation_is_finite(simulation)
                        collision = simulation.collision
                        passed = finite and not collision and deterministic
                        if collision:
                            _hard_failure(
                                hard_failures,
                                "collision",
                                case,
                                step,
                                name,
                                "follower 종단 simulation 충돌",
                            )
                        if not finite:
                            _hard_failure(
                                hard_failures,
                                "non_finite",
                                case,
                                step,
                                name,
                                "follower 결과에 NaN/inf가 포함됨",
                            )
                        if not deterministic:
                            _hard_failure(
                                hard_failures,
                                "non_deterministic",
                                case,
                                step,
                                name,
                                "동일 상태·seed 재실행 결과가 달라짐",
                            )
                        deadline_miss = _record_deadline_miss(
                            measured,
                            config.path_follower_deadline_ns,
                            hard_failures,
                            case,
                            step,
                            name,
                        )
                        completion_failure = _follower_completion_failure(simulation)
                        if completion_failure is not None:
                            _hard_failure(
                                hard_failures,
                                completion_failure,
                                case,
                                step,
                                name,
                                simulation.failure_reason or "goal_timeout",
                            )
                        follower_record = _simulation_record(
                            simulation,
                            measured,
                            grid_snapshot.metadata,
                            grid_result.path,
                            deterministic,
                            expected_class=expected_class,
                            simulation_time_budget_s=follower_time_budget_s,
                            deadline_ns=config.path_follower_deadline_ns,
                            deadline_miss=deadline_miss,
                        )
                        follower_record["initial_command_validation"] = _validation_record(
                            initial_command_validation
                        )
                        follower_records.append(follower_record)
                        additional_distance = _additional_distance(
                            simulation,
                            grid_result.path,
                        )
                        overshoot = _overshoot(simulation.poses, grid_result.path)
                        metrics[name].append(
                            _Metric(
                                measured.elapsed_ns,
                                measured.peak_memory_bytes,
                                passed and simulation.goal_reached and not deadline_miss,
                                collision,
                                simulation.goal_reached,
                                deterministic=deterministic,
                                minimum_clearance_m=simulation.minimum_clearance_m,
                                mean_tracking_error_m=simulation.mean_tracking_error_m,
                                maximum_tracking_error_m=simulation.maximum_tracking_error_m,
                                jerk_rms_mps3=simulation.jerk_rms_mps3,
                                additional_distance_m=additional_distance,
                                overshoot_m=overshoot,
                                deadline_miss=deadline_miss,
                            )
                        )
                        if not follower_trace:
                            follower_trace = simulation.poses
                    except Exception as exc:  # pragma: no cover - 증거 보존용 방어선
                        _hard_failure(
                            hard_failures,
                            "exception",
                            case,
                            step,
                            name,
                            f"{type(exc).__name__}: {exc}",
                        )
                        follower_records.append(_exception_record(name, exc, metadata))
                        metrics[name].append(_Metric(0, 0, False, False, False))
        elif state.input_valid:
            limitations.append(
                _limitation(case, step, "local_pipeline", "global_reference_path_unavailable")
            )

        if current_results:
            previous = {
                name: (graph_snapshot, state.start, state.goal, result)
                for name, result in current_results.items()
            }
        if step == 0 and "astar" in current_results:
            safety_candidate = (
                graph_snapshot,
                state.start,
                state.goal,
                current_results["astar"],
            )
        plot_payloads.append(
            {
                "step": step,
                "graph_snapshot": graph_snapshot,
                "global_results": tuple(current_results.values()),
                "grid_snapshot": grid_snapshot,
                "reference_path": reference_path,
                "grid_result": grid_result,
                "dwa_result": dwa_result,
                "follower_trace": follower_trace,
            }
        )

        pipeline_records = _pipeline_records(
            global_records,
            local_records,
            follower_records,
        )

        steps.append(
            {
                "step": step,
                "input_valid": state.input_valid,
                "start": state.start,
                "goal": state.goal,
                "metadata": metadata,
                "oracle": {"status": oracle_status.value, "cost": oracle_cost},
                "global_results": global_records,
                "local_results": local_records,
                "follower_results": follower_records,
                "pipeline_results": pipeline_records,
                "stale_result_checks": stale_for_step,
            }
        )
        previous_non_global = {}
        for result in (grid_result, dwa_result):
            if result is not None:
                previous_non_global[f"local:{result.planner}"] = (
                    step,
                    "local",
                    grid_snapshot.metadata,
                    result,
                )
        for name, result in follower_command_results.items():
            previous_non_global[f"follower:{name}"] = (
                step,
                "follower",
                grid_snapshot.metadata,
                result,
            )

    return (
        {
            "case_id": case.episode.episode_id,
            "split": case.episode.split.value,
            "family": case.world.family.value,
            "source_batch_seed": source_batch_seed,
            "world_hash": case.world.content_hash,
            "episode_hash": case.episode.content_hash,
            "steps": steps,
        },
        tuple(plot_payloads),
        safety_candidate,
    )


def _stale_checks_for_step(
    previous: dict[str, tuple[object, str, str, SearchResult]],
    *,
    current_metadata: object,
    case_id: str,
    step: int,
    hard_failures: list[dict[str, object]],
) -> list[dict[str, object]]:
    checks: list[dict[str, object]] = []
    for name, (snapshot, start, goal, result) in previous.items():
        validation = validate_global_result(
            snapshot,
            start,
            goal,
            result,
            current_metadata=current_metadata,
        )
        rejected = not validation.executable and "stale_revision" in validation.failures
        record = {
            "case_id": case_id,
            "source_step": step - 1,
            "target_step": step,
            "algorithm": name,
            "role": "global",
            "rejected": rejected,
            "executable": validation.executable,
            "failures": list(validation.failures),
        }
        checks.append(record)
        if validation.executable:
            hard_failures.append(
                {
                    "type": "stale_executable",
                    "case_id": case_id,
                    "step": step,
                    "algorithm": name,
                    "detail": "이전 snapshot 결과가 최신 상태에서 실행 가능으로 판정됨",
                }
            )
    return checks


def _stale_non_global_checks_for_step(
    previous: dict[str, tuple[int, str, object, object]],
    *,
    current_metadata: object,
    case_id: str,
    step: int,
    hard_failures: list[dict[str, object]],
) -> list[dict[str, object]]:
    """이전 local/follower 결과도 최신 metadata에서 실행 불가인지 검증한다."""

    checks: list[dict[str, object]] = []
    stale_failure_names = {
        "input_invalidated",
        "stale_map_id",
        "stale_revision",
        "stale_content_hash",
    }
    for key, (source_step, role, source_metadata, result) in previous.items():
        validation = validate_result_provenance(
            source_metadata,
            result,
            current_metadata=current_metadata,
        )
        rejected = not validation.executable and bool(
            stale_failure_names.intersection(validation.failures)
        )
        algorithm = key.split(":", maxsplit=1)[1]
        record = {
            "case_id": case_id,
            "source_step": source_step,
            "target_step": step,
            "algorithm": algorithm,
            "role": role,
            "rejected": rejected,
            "executable": validation.executable,
            "failures": list(validation.failures),
        }
        checks.append(record)
        if validation.executable:
            hard_failures.append(
                {
                    "type": "stale_executable",
                    "case_id": case_id,
                    "step": step,
                    "algorithm": algorithm,
                    "detail": f"이전 {role} 결과가 최신 상태에서 실행 가능으로 판정됨",
                }
            )
    return checks


def _protective_stop_evidence(
    source: tuple[object, str, str, SearchResult] | None,
    hard_failures: list[dict[str, object]],
) -> dict[str, object]:
    if source is None:
        hard_failures.append(
            {
                "type": "exception",
                "case_id": "experiment",
                "step": 0,
                "algorithm": "safety_gate",
                "detail": "안전 게이트 검증에 사용할 유효한 A* 결과가 없음",
            }
        )
        return {"available": False}
    snapshot, start, goal, result = source
    validation = validate_global_result(snapshot, start, goal, result)
    gate = AutomaticResumeGate()
    gate.hazard_detected()
    gate.confirm_stop()
    gate.hazard_cleared()
    authorization_before = authorize_after_protective_stop(validation, gate)
    automatic_resume_before = gate.try_automatic_resume()
    gate.record_path_revalidation(original_path_safe=validation.executable)
    gate.revalidate_resume_instruction()
    gate.authorize_local_safety()
    automatic_resume_after = gate.try_automatic_resume()
    authorization_after = authorize_after_protective_stop(validation, gate)
    if authorization_before or automatic_resume_before:
        hard_failures.append(
            {
                "type": "unauthorized_resume",
                "case_id": snapshot.metadata.map_id,
                "step": 0,
                "algorithm": "safety_gate",
                "detail": "위험 해소만으로 재개가 허용됨",
            }
        )
    return {
        "available": True,
        "authorization_before_revalidation": authorization_before,
        "automatic_resume_before_revalidation": automatic_resume_before,
        "automatic_resume_after_all_gates": automatic_resume_after,
        "authorization_after_all_gates": authorization_after,
        "events": list(gate.events),
    }


def _dynamic_local_contract_evidence(
    hard_failures: list[dict[str, object]],
) -> dict[str, object]:
    """차단→유지→해제 sequence에서 DWA의 정지·교착·회복을 재현한다."""

    open_occupancy = np.zeros((60, 80), dtype=np.bool_)
    blocked_occupancy = open_occupancy.copy()
    blocked_occupancy[32, 27] = True

    def snapshot(
        occupancy: np.ndarray,
        *,
        observation_revision: int,
        content_hash: str,
    ) -> GridSnapshot:
        return GridSnapshot(
            metadata=SnapshotMetadata(
                map_id="dynamic_local_contract",
                map_revision=0,
                mission_revision=0,
                observation_revision=observation_revision,
                seed=31,
                content_hash=content_hash,
            ),
            grid=GridMap(occupancy, resolution_m=0.05),
        )

    blocked = snapshot(
        blocked_occupancy,
        observation_revision=10,
        content_hash="dynamic-local-obstacle-created-v1",
    )
    reopened = snapshot(
        open_occupancy,
        observation_revision=11,
        content_hash="dynamic-local-obstacle-removed-v1",
    )
    snapshots = (blocked, blocked, blocked) + (reopened,) * 60
    event_kinds = (
        "create_obstacle",
        "obstacle_hold",
        "obstacle_hold",
        "remove_obstacle",
    ) + ("obstacle_hold",) * 59
    reference_start = Pose2D(1.025, 1.525)
    initial_pose = Pose2D(reference_start.x, reference_start.y + 0.11)
    goal = Pose2D(3.025, 1.525)
    planner = DynamicWindowPlanner(
        horizon_s=0.4,
        integration_dt_s=0.1,
        linear_samples=3,
        angular_samples=5,
    )
    evidence = simulate_dynamic_local_evidence(
        planner,
        (reference_start, goal),
        snapshots,
        RobotState(initial_pose),
        goal,
        event_kinds=event_kinds,
        deadlock_threshold_steps=2,
    )
    contract_passed = all(
        (
            evidence.simulation_only,
            evidence.collision_count == 0,
            evidence.safe_stop_count >= 1,
            evidence.deadlock_observed,
            evidence.recovery_observed,
            evidence.path_deviation_observed,
            evidence.rejoin_observed,
            evidence.commands_finite,
            evidence.metrics_finite,
        )
    )
    if not contract_passed:
        hard_failures.append(
            {
                "type": "dynamic_local_contract_failed",
                "case_id": "dynamic_local_contract",
                "step": -1,
                "algorithm": "dwa",
                "detail": (
                    f"collision={evidence.collision_count}, "
                    f"stops={evidence.safe_stop_count}, "
                    f"deadlock={evidence.deadlock_observed}, "
                    f"recovery={evidence.recovery_observed}, "
                    f"deviation={evidence.path_deviation_observed}, "
                    f"rejoin={evidence.rejoin_observed}"
                ),
            }
        )
    record = asdict(evidence)
    record["contract_passed"] = contract_passed
    record["scenario"] = "synthetic_create_hold_remove_rejoin_simulation_only"
    return record


def _save_hidden_plots(
    output: Path,
    case: GeneratedCase,
    payloads: tuple[dict[str, object], ...],
) -> tuple[Path, ...]:
    target = output / "visualizations"
    stem = case.episode.episode_id
    paths: list[Path] = []
    for payload in payloads:
        step = int(payload["step"])
        graph_snapshot = payload["graph_snapshot"]
        grid_snapshot = payload["grid_snapshot"]
        graph_path = save_graph_experiment_plot(
            graph_snapshot,
            payload["global_results"],
            target / f"{stem}_step_{step:02d}_graph.png",
            title=(
                f"Hidden graph: {stem} step={step} map_rev={graph_snapshot.metadata.map_revision}"
            ),
        )
        grid_result = payload["grid_result"]
        dwa_result = payload["dwa_result"]
        grid_path = save_grid_experiment_plot(
            grid_snapshot,
            target / f"{stem}_step_{step:02d}_grid.png",
            reference_path=payload["reference_path"],
            path=grid_result.path if grid_result is not None else (),
            trajectory=dwa_result.trajectory if dwa_result is not None else (),
            robot_trace=payload["follower_trace"],
            title=(
                f"Hidden grid: {stem} step={step} map_rev={grid_snapshot.metadata.map_revision}"
            ),
        )
        paths.extend((graph_path, grid_path))
    return tuple(paths)


def _claim_memory_profile(
    profiled: set[tuple[str, str]],
    algorithm: str,
    case: GeneratedCase,
) -> bool:
    """알고리즘·지도 계열별 첫 표본만 tracemalloc로 측정한다."""

    key = algorithm, case.world.family.value
    if key in profiled:
        return False
    profiled.add(key)
    return True


def _measure(
    invoke: Callable[[], _T],
    *,
    profile_memory: bool = True,
) -> _Measured:
    if profile_memory:
        tracemalloc.start()
    started_at = perf_counter_ns()
    try:
        value = invoke()
        elapsed_ns = perf_counter_ns() - started_at
        peak = tracemalloc.get_traced_memory()[1] if profile_memory else 0
    finally:
        if profile_memory:
            tracemalloc.stop()
    return _Measured(
        value=value,
        elapsed_ns=elapsed_ns,
        peak_memory_bytes=int(peak),
        memory_profiled=profile_memory,
    )


def _networkx_oracle(snapshot: object, start: str, goal: str) -> tuple[PlanStatus, float | None]:
    if not getattr(snapshot.metadata, "input_valid", True):
        return PlanStatus.INVALID_INPUT, None
    graph = snapshot.graph
    if start not in graph.nodes or goal not in graph.nodes:
        return PlanStatus.INVALID_INPUT, None
    oracle = nx.DiGraph() if graph.directed else nx.Graph()
    oracle.add_nodes_from(graph.nodes)
    for edge in graph.edges:
        key = canonical_edge(edge.source, edge.target, directed=graph.directed)
        if key not in graph.closed_edges:
            oracle.add_edge(edge.source, edge.target, weight=edge.cost)
    try:
        cost = nx.shortest_path_length(oracle, start, goal, weight="weight")
    except nx.NetworkXNoPath:
        return PlanStatus.NO_PATH, None
    return PlanStatus.FOUND, float(cost)


def _grid_dijkstra_oracle(
    snapshot: object,
    reference_path: tuple[Pose2D, ...],
    start: Pose2D,
    goal: Pose2D,
) -> tuple[PlanStatus, float | None]:
    """공통 configuration grid를 휴리스틱 없이 탐색하는 독립 local oracle."""

    if not snapshot.input_valid or not reference_path:
        return PlanStatus.INVALID_INPUT, None
    collision_checker = CollisionChecker(
        snapshot.grid,
        forbidden_cells=snapshot.forbidden_cells,
    )
    grid = collision_checker.configuration_grid
    search_bounds = reference_search_bounds(grid, reference_path)
    start_cell = grid.world_to_cell(start)
    goal_cell = grid.world_to_cell(goal)
    if not grid.in_bounds(start_cell) or not grid.in_bounds(goal_cell):
        return PlanStatus.INVALID_INPUT, None
    if grid.is_occupied(start_cell) or grid.is_occupied(goal_cell):
        return PlanStatus.INVALID_INPUT, None
    if not search_bounds.contains(start_cell) or not search_bounds.contains(goal_cell):
        return PlanStatus.INVALID_INPUT, None

    frontier: list[tuple[float, int, int]] = [(0.0, start_cell[0], start_cell[1])]
    best_cost = {start_cell: 0.0}
    while frontier:
        current_cost, x, y = heappop(frontier)
        current = (x, y)
        if current_cost > best_cost.get(current, inf):
            continue
        if current == goal_cell:
            return PlanStatus.FOUND, current_cost
        for neighbor, edge_cost in grid.neighbors8(current):
            if not search_bounds.contains(neighbor):
                continue
            candidate = current_cost + edge_cost
            if candidate >= best_cost.get(neighbor, inf):
                continue
            best_cost[neighbor] = candidate
            heappush(frontier, (candidate, neighbor[0], neighbor[1]))
    return PlanStatus.NO_PATH, None


def _node_path_to_poses(case: GeneratedCase, path: tuple[str, ...]) -> tuple[Pose2D, ...]:
    nodes = {node.node_id: node for node in case.world.nodes}
    poses: list[Pose2D] = []
    for index, node_id in enumerate(path):
        node = nodes[node_id]
        if index + 1 < len(path):
            next_node = nodes[path[index + 1]]
            yaw = atan2(next_node.y - node.y, next_node.x - node.x)
        elif poses:
            yaw = poses[-1].yaw
        else:
            yaw = 0.0
        poses.append(Pose2D(node.x, node.y, yaw))
    return tuple(poses)


def _invalid_global_result(name: str, snapshot: object) -> SearchResult:
    metadata = snapshot.metadata
    return SearchResult(
        planner=name,
        status=PlanStatus.INVALID_INPUT,
        path=(),
        cost=None,
        expanded_nodes=0,
        elapsed_ns=0,
        map_revision=metadata.map_revision,
        mission_revision=metadata.mission_revision,
        observation_revision=metadata.observation_revision,
        map_id=metadata.map_id,
        input_content_hash=metadata.content_hash,
        failure_reason="episode_input_invalid",
    )


def _global_record(
    result: SearchResult,
    *,
    peak_memory_bytes: int,
    validation: dict[str, object],
    oracle_status: PlanStatus,
    oracle_cost: float | None,
    oracle_matched: bool | None,
    path_churn: float | None,
    deterministic: bool = True,
    measured_elapsed_ns: int | None = None,
    deadline_ns: int | None = None,
    deadline_miss: bool = False,
) -> dict[str, object]:
    provenance = _result_provenance(result)
    return {
        "algorithm": result.planner,
        "status": result.status.value,
        "path": list(result.path),
        "cost": result.cost,
        "expanded_nodes": result.expanded_nodes,
        "route_churn": path_churn,
        "deterministic": deterministic,
        "algorithm_elapsed_ns": result.elapsed_ns,
        "measured_elapsed_ns": measured_elapsed_ns or result.elapsed_ns,
        "peak_memory_bytes": peak_memory_bytes,
        **provenance,
        "provenance": provenance,
        "deadline_ns": deadline_ns,
        "deadline_miss": deadline_miss,
        "failure_reason": result.failure_reason,
        "oracle": {"status": oracle_status.value, "cost": oracle_cost},
        "oracle_matched": oracle_matched,
        "validation": validation,
    }


def _local_record(
    result: object,
    measured: _Measured,
    *,
    validation: dict[str, object],
    oracle_matched: bool | None,
    grid_oracle: tuple[PlanStatus, float | None] | None,
    deterministic: bool,
    deadline_ns: int,
    deadline_miss: bool,
) -> dict[str, object]:
    provenance = _result_provenance(result)
    return {
        "algorithm": result.planner,
        "status": result.status.value,
        "path": [_pose_record(pose) for pose in result.path],
        "trajectory": [
            {
                "time_s": point.time_s,
                "pose": _pose_record(point.pose),
                "twist": asdict(point.twist),
            }
            for point in result.trajectory
        ],
        "cost": result.cost,
        "expanded_nodes": result.expanded_nodes,
        "sampled_trajectories": result.sampled_trajectories,
        "algorithm_elapsed_ns": result.elapsed_ns,
        "measured_elapsed_ns": measured.elapsed_ns,
        "peak_memory_bytes": measured.peak_memory_bytes,
        **provenance,
        "provenance": provenance,
        "deadline_ns": deadline_ns,
        "deadline_miss": deadline_miss,
        "collision": result.collision,
        "minimum_clearance": result.minimum_clearance,
        "failure_reason": result.failure_reason,
        "deterministic": deterministic,
        "oracle_matched": oracle_matched,
        "grid_oracle": (
            {"algorithm": "grid_dijkstra", "status": grid_oracle[0].value, "cost": grid_oracle[1]}
            if grid_oracle is not None
            else None
        ),
        "validation": validation,
    }


def _simulation_record(
    result: SimulationResult,
    measured: _Measured,
    metadata: object,
    reference_path: tuple[Pose2D, ...],
    deterministic: bool,
    *,
    expected_class: str,
    simulation_time_budget_s: float,
    deadline_ns: int,
    deadline_miss: bool,
) -> dict[str, object]:
    travelled_distance = _path_length(result.poses)
    reference_distance = _path_length(reference_path)
    additional_distance = _additional_distance(result, reference_path)
    overshoot = _overshoot(result.poses, reference_path)
    provenance = _result_provenance(result, metadata)
    return {
        "algorithm": result.component,
        "status": result.status.value,
        "goal_reached": result.goal_reached,
        "collision": result.collision,
        "deterministic": deterministic,
        "pose_count": len(result.poses),
        "command_count": len(result.commands),
        "simulation_elapsed_s": result.elapsed_s,
        "simulation_time_budget_s": simulation_time_budget_s,
        "measured_elapsed_ns": measured.elapsed_ns,
        "peak_memory_bytes": measured.peak_memory_bytes,
        **provenance,
        "provenance": provenance,
        "expected_outcome_class": expected_class,
        "deadline_ns": deadline_ns,
        "deadline_miss": deadline_miss,
        "minimum_clearance_m": result.minimum_clearance_m,
        "mean_tracking_error_m": result.mean_tracking_error_m,
        "maximum_tracking_error_m": result.maximum_tracking_error_m,
        "jerk_rms_mps3": result.jerk_rms_mps3,
        "final_goal_distance_m": result.final_goal_distance_m,
        "travelled_distance_m": travelled_distance,
        "reference_distance_m": reference_distance,
        "additional_distance_m": additional_distance,
        "overshoot_m": overshoot,
        "failure_reason": result.failure_reason,
    }


def _pipeline_records(
    global_records: list[dict[str, object]],
    local_records: list[dict[str, object]],
    follower_records: list[dict[str, object]],
) -> list[dict[str, object]]:
    """A* → Grid A* → follower 종단 조합을 역할별 결과와 별도로 기록한다."""

    astar = next(
        (record for record in global_records if record.get("algorithm") == "astar"),
        None,
    )
    grid_astar = next(
        (record for record in local_records if record.get("algorithm") == "grid_astar"),
        None,
    )
    if astar is None or grid_astar is None:
        return []

    records: list[dict[str, object]] = []
    for follower in follower_records:
        follower_name = str(follower["algorithm"])
        collision = bool(grid_astar["collision"] or follower["collision"])
        deadline_miss = bool(
            astar["deadline_miss"]
            or grid_astar["deadline_miss"]
            or follower["deadline_miss"]
        )
        deterministic = bool(
            astar["deterministic"]
            and grid_astar["deterministic"]
            and follower["deterministic"]
        )
        component_validation_passed = bool(
            astar["validation"]["passed"]
            and astar["oracle_matched"]
            and grid_astar["validation"]["passed"]
            and grid_astar["oracle_matched"]
            and follower["initial_command_validation"]["passed"]
        )
        success = bool(
            astar["status"] == PlanStatus.FOUND.value
            and grid_astar["status"] == PlanStatus.FOUND.value
            and follower["goal_reached"]
            and not collision
            and not deadline_miss
            and deterministic
            and component_validation_passed
        )
        records.append(
            {
                "pipeline": f"astar_grid_astar_{follower_name}",
                "components": ["astar", "grid_astar", follower_name],
                "status": PlanStatus.FOUND.value if success else PlanStatus.NO_PATH.value,
                "success": success,
                "collision": collision,
                "deterministic": deterministic,
                "component_validation_passed": component_validation_passed,
                "deadline_miss": deadline_miss,
                "measured_elapsed_ns": sum(
                    int(record["measured_elapsed_ns"])
                    for record in (astar, grid_astar, follower)
                ),
                "peak_memory_bytes": max(
                    int(record["peak_memory_bytes"])
                    for record in (astar, grid_astar, follower)
                ),
                "peak_memory_policy": "maximum_profiled_stage_sample_or_zero",
                "minimum_clearance_m": min(
                    float(grid_astar["minimum_clearance"]),
                    float(follower["minimum_clearance_m"]),
                ),
                "mean_tracking_error_m": follower["mean_tracking_error_m"],
                "maximum_tracking_error_m": follower["maximum_tracking_error_m"],
                "jerk_rms_mps3": follower["jerk_rms_mps3"],
                "additional_distance_m": follower["additional_distance_m"],
                "overshoot_m": follower["overshoot_m"],
                "provenance": follower["provenance"],
            }
        )
    return records


def _aggregate_pipelines(
    case_records: list[dict[str, object]],
) -> dict[str, dict[str, object]]:
    """종단 조합은 알고리즘 역할 Pareto와 섞지 않고 별도 분포로 집계한다."""

    grouped: dict[str, list[dict[str, object]]] = defaultdict(list)
    for case in case_records:
        for step in case["steps"]:
            for record in step["pipeline_results"]:
                grouped[str(record["pipeline"])].append(record)

    aggregates: dict[str, dict[str, object]] = {}
    for name, records in sorted(grouped.items()):
        elapsed = [float(record["measured_elapsed_ns"]) for record in records]
        peak_memory = [int(record["peak_memory_bytes"]) for record in records]
        minimum_clearance = [float(record["minimum_clearance_m"]) for record in records]
        tracking_error = [float(record["maximum_tracking_error_m"]) for record in records]
        aggregate: dict[str, object] = {
            "samples": len(records),
            "success_count": sum(bool(record["success"]) for record in records),
            "collision_count": sum(bool(record["collision"]) for record in records),
            "deterministic_count": sum(
                bool(record["deterministic"]) for record in records
            ),
            "deadline_miss_count": sum(
                bool(record["deadline_miss"]) for record in records
            ),
            "component_validation_pass_count": sum(
                bool(record["component_validation_passed"]) for record in records
            ),
            "peak_memory_bytes": max(peak_memory),
            "peak_memory_policy": "maximum_profiled_stage_sample_or_zero",
            "minimum_clearance_m_min": min(minimum_clearance),
            "maximum_tracking_error_m_worst": max(tracking_error),
        }
        aggregate.update(_distribution(elapsed, "elapsed_ns"))
        aggregates[name] = aggregate
    return aggregates


def _aggregate_by_role(
    metrics: dict[str, list[_Metric]],
    manifest: list[dict[str, str]],
) -> dict[str, dict[str, dict[str, object]]]:
    roles: dict[str, dict[str, dict[str, object]]] = defaultdict(dict)
    for descriptor in manifest:
        name = descriptor["name"]
        if descriptor["implementation_status"] != "implemented":
            continue
        samples = metrics.get(name, [])
        elapsed = np.asarray([sample.elapsed_ns for sample in samples], dtype=np.int64)
        if samples:
            summary = {
                "samples": len(samples),
                "elapsed_ns_p50": int(median(sample.elapsed_ns for sample in samples)),
                "elapsed_ns_p95": int(np.percentile(elapsed, 95)),
                "elapsed_ns_p99": int(np.percentile(elapsed, 99)),
                "elapsed_ns_worst": int(elapsed.max()),
                "peak_memory_bytes": max(sample.peak_memory_bytes for sample in samples),
                "pass_count": sum(sample.passed for sample in samples),
                "pass_rate": sum(sample.passed for sample in samples) / len(samples),
                "collision_count": sum(sample.collision for sample in samples),
                "success_count": sum(sample.success for sample in samples),
                "deterministic_count": sum(sample.deterministic for sample in samples),
                "deterministic_rate": sum(sample.deterministic for sample in samples)
                / len(samples),
                "deadline_miss_count": sum(sample.deadline_miss for sample in samples),
            }
        else:
            summary = {
                "samples": 0,
                "elapsed_ns_p50": 0,
                "elapsed_ns_p95": 0,
                "elapsed_ns_p99": 0,
                "elapsed_ns_worst": 0,
                "peak_memory_bytes": 0,
                "pass_count": 0,
                "pass_rate": 0.0,
                "collision_count": 0,
                "success_count": 0,
                "deterministic_count": 0,
                "deterministic_rate": 0.0,
                "deadline_miss_count": 0,
            }
        if descriptor["role"] in {"global_oracle", "global", "global_incremental"}:
            expanded = [
                float(sample.expanded_nodes)
                for sample in samples
                if sample.expanded_nodes is not None
            ]
            churn = [sample.route_churn for sample in samples if sample.route_churn is not None]
            summary.update(_distribution(expanded, "expanded_nodes"))
            summary.update(_distribution(churn, "route_churn"))
        elif descriptor["role"] in {"local_path", "local_trajectory"}:
            clearances = [
                sample.minimum_clearance_m
                for sample in samples
                if sample.minimum_clearance_m is not None
            ]
            summary.update(_distribution(clearances, "minimum_clearance_m", include_min=True))
        elif descriptor["role"] == "path_follower":
            mean_errors = [
                sample.mean_tracking_error_m
                for sample in samples
                if sample.mean_tracking_error_m is not None
            ]
            maximum_errors = [
                sample.maximum_tracking_error_m
                for sample in samples
                if sample.maximum_tracking_error_m is not None
            ]
            jerks = [sample.jerk_rms_mps3 for sample in samples if sample.jerk_rms_mps3 is not None]
            extra_distance = [
                sample.additional_distance_m
                for sample in samples
                if sample.additional_distance_m is not None
            ]
            overshoots = [
                sample.overshoot_m for sample in samples if sample.overshoot_m is not None
            ]
            summary.update(_distribution(mean_errors, "mean_tracking_error_m"))
            summary.update(_distribution(maximum_errors, "maximum_tracking_error_m"))
            summary.update(_distribution(jerks, "jerk_rms_mps3"))
            summary.update(_distribution(extra_distance, "additional_distance_m"))
            summary.update(_distribution(overshoots, "overshoot_m"))
        roles[_comparison_role(descriptor["role"])][name] = summary
    return {role: dict(algorithms) for role, algorithms in sorted(roles.items())}


def _comparison_role(manifest_role: str) -> str:
    if manifest_role in {"global_oracle", "global", "global_incremental"}:
        return "global"
    if manifest_role in {"local_path", "local_trajectory"}:
        return "local"
    return manifest_role


def _distribution(
    values: list[float],
    prefix: str,
    *,
    include_min: bool = False,
) -> dict[str, float]:
    if not values:
        result = {
            f"{prefix}_mean": 0.0,
            f"{prefix}_p50": 0.0,
            f"{prefix}_p95": 0.0,
            f"{prefix}_p99": 0.0,
            f"{prefix}_worst": 0.0,
        }
        if include_min:
            result[f"{prefix}_min"] = 0.0
        return result
    array = np.asarray(values, dtype=np.float64)
    result = {
        f"{prefix}_mean": float(fmean(values)),
        f"{prefix}_p50": float(np.percentile(array, 50)),
        f"{prefix}_p95": float(np.percentile(array, 95)),
        f"{prefix}_p99": float(np.percentile(array, 99)),
        f"{prefix}_worst": float(array.max()),
    }
    if include_min:
        result[f"{prefix}_min"] = float(array.min())
    return result


def _markdown_summary(
    config: ExperimentConfig,
    split_counts: Counter[str],
    pareto: dict[str, object],
    hard_failures: list[dict[str, object]],
    limitations: list[dict[str, object]],
    plots: list[Path],
) -> str:
    lines = [
        "# 경로 알고리즘 실험 요약",
        "",
        "> 이 결과는 가상 차체와 합성 지도를 사용한 `simulation_only` Python 비교다.",
        "> 실제 사람 탑승 안전성 또는 최종 알고리즘 채택의 근거가 아니다.",
        "",
        "## 실행 구성",
        "",
        f"- 공개 corpus seed: `{config.base_seed}`",
        f"- 숨김 corpus seed: `{config.hidden_seed}`",
        f"- corpus: `{sum(split_counts.values())}`개 "
        f"(golden {split_counts['golden']}, development {split_counts['development']}, "
        f"hidden {split_counts['hidden']}, regressions {split_counts['regressions']})",
        f"- hard failure: `{len(hard_failures)}`건",
        f"- limitation: `{len(limitations)}`건",
        f"- hidden PNG: `{len(plots)}`개",
        "",
        "## 실행 범위 제한",
        "",
        "- DWA와 Grid A*는 전역 reference가 있는 모든 유효 step에서 실행했다.",
        "- PP/RPP는 Grid A* 경로가 생성된 모든 호환 step에서 실행했다.",
        "- 동적 local 재합류·교착·정지 동작은 이번 one-shot 실행에서 측정하지 않았다.",
        "- role별 deadline은 제품값이 아닌 simulation-only 연구 이상 감지값이다.",
        "- hidden PNG는 각 hidden case의 initial 및 모든 event step을 저장했다.",
        "",
        "## 역할별 결과",
        "",
        "| 역할 | 알고리즘 | 표본 | 통과 | 충돌 | p95(ns) | peak memory(bytes) |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for role, algorithms in pareto["roles"].items():
        for name, result in algorithms.items():
            lines.append(
                f"| {role} | {name} | {result['samples']} | {result['pass_count']} | "
                f"{result['collision_count']} | {result['elapsed_ns_p95']} | "
                f"{result['peak_memory_bytes']} |"
            )
    lines.extend(
        [
            "",
            "실행시간 하나로 종합 순위를 만들지 않았으며 같은 역할끼리만 비교한다.",
            "일반 case의 보수적 `NO_PATH`와 follower 미도착은 limitation으로 분리했다. ",
            "검증된 도달 가능 golden의 follower 미도착은 hard failure다.",
            "",
        ]
    )
    return "\n".join(lines)


def _corpus_record(case: GeneratedCase, source_batch_seed: int) -> dict[str, object]:
    return {
        "case_id": case.episode.episode_id,
        "split": case.episode.split.value,
        "family": case.world.family.value,
        "source_batch_seed": source_batch_seed,
        "world_seed": case.world.seed,
        "episode_seed": case.episode.seed,
        "world_hash": case.world.content_hash,
        "episode_hash": case.episode.content_hash,
        "generator_version": case.world.generator_version,
        "simulation_only": case.world.simulation_only and case.episode.simulation_only,
    }


def _evaluation_coverage(
    case_records: list[dict[str, object]],
    *,
    hidden_visualization_count: int,
    config: ExperimentConfig,
) -> dict[str, object]:
    steps = [step for case in case_records for step in case["steps"]]
    local_records = [record for step in steps for record in step["local_results"]]
    follower_records = [record for step in steps for record in step["follower_results"]]
    pipeline_records = [record for step in steps for record in step["pipeline_results"]]
    hidden_steps = sum(len(case["steps"]) for case in case_records if case["split"] == "hidden")
    return {
        "global": {
            "policy": "all_cases_initial_through_episode_max_event_step",
            "evaluated_steps": len(steps),
        },
        "grid_astar": {
            "policy": "all_valid_steps_with_reference_path",
            "result_count": sum(
                record.get("algorithm") == "grid_astar" for record in local_records
            ),
        },
        "dwa": {
            "policy": "all_valid_steps_with_reference_path",
            "result_count": sum(record.get("algorithm") == "dwa" for record in local_records),
        },
        "path_followers": {
            "policy": "all_steps_with_found_grid_astar_path",
            "compatible_step_count": sum(bool(step["follower_results"]) for step in steps),
            "result_count": len(follower_records),
        },
        "end_to_end_pipelines": {
            "policy": "astar_to_grid_astar_to_each_path_follower",
            "compatible_step_count": sum(bool(step["pipeline_results"]) for step in steps),
            "result_count": len(pipeline_records),
        },
        "dynamic_local_closed_loop": {
            "policy": "synthetic_create_hold_remove_stateful_dwa_rejoin_contract",
            "evaluated_steps": 63,
            "metrics": [
                "safe_stop",
                "deadlock",
                "recovery",
                "path_deviation",
                "rejoin",
                "collision",
                "clearance",
                "tracking_error",
            ],
        },
        "hidden_visualizations": {
            "policy": "graph_and_grid_for_every_evaluated_hidden_step",
            "evaluated_steps": hidden_steps,
            "png_count": hidden_visualization_count,
        },
        "deadline_policy": {
            "scope": "simulation_only_research_threshold_not_product_requirement",
            "comparison": "measured_elapsed_ns_strictly_greater_than_deadline_ns",
            "global_deadline_ns": config.global_deadline_ns,
            "local_deadline_ns": config.local_deadline_ns,
            "path_follower_deadline_ns": config.path_follower_deadline_ns,
        },
        "peak_memory_policy": {
            "method": "tracemalloc_first_sample_per_algorithm_and_world_family",
            "reason": "tracemalloc 자체 오버헤드를 전체 성능 분포에 섞지 않음",
            "unprofiled_sample_value_bytes": 0,
        },
        "not_measured": ["full_corpus_dynamic_local_closed_loop"],
    }


def _coverage_limitations() -> list[dict[str, object]]:
    return [
        {
            "case_id": "experiment",
            "step": -1,
            "algorithm": "local_pipeline",
            "reason": "full_corpus_dynamic_local_closed_loop_not_measured",
        },
    ]


def _metadata_record(metadata: object) -> dict[str, object]:
    return {
        "map_id": metadata.map_id,
        "map_revision": metadata.map_revision,
        "mission_revision": metadata.mission_revision,
        "observation_revision": metadata.observation_revision,
        "seed": metadata.seed,
        "content_hash": metadata.content_hash,
        "input_valid": getattr(metadata, "input_valid", True),
    }


def _validation_record(validation: object) -> dict[str, object]:
    return {
        "passed": validation.passed,
        "executable": validation.executable,
        "failures": list(validation.failures),
    }


def _exception_record(name: str, exc: Exception, metadata: dict[str, object]) -> dict[str, object]:
    provenance = {
        "map_id": metadata["map_id"],
        "map_revision": metadata["map_revision"],
        "mission_revision": metadata["mission_revision"],
        "observation_revision": metadata["observation_revision"],
        "input_content_hash": metadata["content_hash"],
    }
    return {
        "algorithm": name,
        "status": PlanStatus.INVALID_INPUT.value,
        "exception": f"{type(exc).__name__}: {exc}",
        **provenance,
        "provenance": provenance,
        "validation": {"passed": False, "executable": False, "failures": ["exception"]},
    }


def _hard_failure(
    target: list[dict[str, object]],
    failure_type: str,
    case: GeneratedCase,
    step: int,
    algorithm: str,
    detail: str,
) -> None:
    target.append(
        {
            "type": failure_type,
            "case_id": case.episode.episode_id,
            "step": step,
            "algorithm": algorithm,
            "detail": detail,
        }
    )


def _record_deadline_miss(
    measured: _Measured,
    deadline_ns: int,
    hard_failures: list[dict[str, object]],
    case: GeneratedCase,
    step: int,
    algorithm: str,
) -> bool:
    """simulation-only 연구 deadline 초과를 명시적 hard failure로 보존한다."""

    missed = measured.elapsed_ns > deadline_ns
    if missed:
        _hard_failure(
            hard_failures,
            "deadline_miss",
            case,
            step,
            algorithm,
            f"measured_elapsed_ns={measured.elapsed_ns} > deadline_ns={deadline_ns}",
        )
    return missed


def _follower_expected_class(
    case: GeneratedCase,
    *,
    input_valid: bool,
    grid_oracle_status: PlanStatus,
    grid_result: object,
) -> str:
    """미도착을 hard failure로 볼 검증된 golden과 일반 사례를 구분한다."""

    if (
        case.episode.split is CorpusSplit.GOLDEN
        and input_valid
        and grid_oracle_status is PlanStatus.FOUND
        and grid_result.status is PlanStatus.FOUND
    ):
        return "validated_reachable_golden"
    return "general_case"


def _local_hard_safety_failures(
    *,
    result_collision: bool,
    validation_failures: tuple[str, ...],
) -> frozenset[str]:
    """검증 결과에서 즉시 hard failure로 승격할 local 안전 위반을 분리한다."""

    failures: set[str] = set()
    if result_collision or "collision" in validation_failures:
        failures.add("collision")
    if "forbidden_zone_entry" in validation_failures:
        failures.add("forbidden_zone_entry")
    return frozenset(failures)


def _follower_completion_failure(result: SimulationResult) -> str | None:
    """충돌과 별개로 시간 예산 내 미도착을 모든 split에서 hard 처리한다."""

    if not result.goal_reached and not result.collision:
        return "follower_timeout"
    return None


def _result_matches_metadata(result: object, metadata: object) -> bool:
    """INVALID_INPUT도 올바른 snapshot provenance를 보존했는지 확인한다."""

    return (
        getattr(result, "map_id", None) == getattr(metadata, "map_id", None)
        and getattr(result, "map_revision", None)
        == getattr(metadata, "map_revision", None)
        and getattr(result, "mission_revision", None)
        == getattr(metadata, "mission_revision", None)
        and getattr(result, "observation_revision", None)
        == getattr(metadata, "observation_revision", None)
        and getattr(result, "input_content_hash", None)
        == getattr(metadata, "content_hash", None)
    )


def _follower_time_budget_s(
    reference_path: tuple[Pose2D, ...],
    *,
    floor_s: float,
) -> float:
    """경로 길이에 비례해 두 추종기에 동일한 폐루프 시뮬레이션 예산을 준다.

    ``floor_s``는 짧은 경로의 최소 예산이다. 길이 기반 항은 공통 nominal
    speed의 2.5배 시간과 정지·선회 여유를 더한다. 이는 실제 제품 deadline이
    아니라 simulation-only 회귀시험의 timeout 계약이다.
    """

    if floor_s <= 0.0:
        raise ValueError("follower simulation time floor는 양수여야 합니다.")
    reference_distance_m = _path_length(reference_path)
    nominal_speed_mps = VIRTUAL_DOLL_WHEELCHAIR_V0_1.nominal_speed_mps
    return max(
        floor_s,
        reference_distance_m
        / nominal_speed_mps
        * _FOLLOWER_PATH_TIME_FACTOR
        + _FOLLOWER_SETTLING_ALLOWANCE_S,
    )


def _result_provenance(result: object, metadata: object | None = None) -> dict[str, object]:
    """계약 전환 중에도 존재하는 provenance 필드를 손실 없이 기록한다."""

    def value(name: str, fallback: object) -> object:
        return getattr(result, name, fallback)

    return {
        "map_id": value("map_id", getattr(metadata, "map_id", "")),
        "map_revision": value("map_revision", getattr(metadata, "map_revision", 0)),
        "mission_revision": value("mission_revision", getattr(metadata, "mission_revision", 0)),
        "observation_revision": value(
            "observation_revision", getattr(metadata, "observation_revision", 0)
        ),
        "input_content_hash": value("input_content_hash", getattr(metadata, "content_hash", "")),
    }


def _limitation(
    case: GeneratedCase,
    step: int,
    algorithm: str,
    reason: str,
) -> dict[str, object]:
    return {
        "case_id": case.episode.episode_id,
        "step": step,
        "algorithm": algorithm,
        "reason": reason,
    }


def _pose_record(pose: Pose2D) -> dict[str, float]:
    return {"x": pose.x, "y": pose.y, "yaw": pose.yaw}


def _path_length(path: tuple[Pose2D, ...]) -> float:
    return sum(
        ((target.x - source.x) ** 2 + (target.y - source.y) ** 2) ** 0.5
        for source, target in zip(path, path[1:], strict=False)
    )


def _additional_distance(
    result: SimulationResult,
    reference_path: tuple[Pose2D, ...],
) -> float | None:
    if not result.goal_reached:
        return None
    return max(0.0, _path_length(result.poses) - _path_length(reference_path))


def _overshoot(
    poses: tuple[Pose2D, ...],
    reference_path: tuple[Pose2D, ...],
) -> float:
    if len(reference_path) < 2:
        return 0.0
    previous = reference_path[-2]
    goal = reference_path[-1]
    dx = goal.x - previous.x
    dy = goal.y - previous.y
    length = (dx * dx + dy * dy) ** 0.5
    if length == 0.0:
        return 0.0
    return max(
        0.0,
        max(((pose.x - goal.x) * dx + (pose.y - goal.y) * dy) / length for pose in poses),
    )


def _search_result_is_finite(result: SearchResult) -> bool:
    return (
        result.elapsed_ns >= 0
        and result.expanded_nodes >= 0
        and (result.cost is None or isfinite(result.cost))
    )


def _search_signature(result: SearchResult) -> tuple[object, ...]:
    return result.status, result.path, result.cost, result.expanded_nodes, result.failure_reason


def _local_signature(result: object) -> tuple[object, ...]:
    return (
        result.status,
        result.path,
        result.trajectory,
        result.cost,
        result.expanded_nodes,
        result.sampled_trajectories,
        result.collision,
        result.minimum_clearance,
        result.failure_reason,
    )


def _simulation_signature(result: SimulationResult) -> tuple[object, ...]:
    return (
        result.status,
        result.goal_reached,
        result.collision,
        result.poses,
        result.commands,
        result.elapsed_s,
        result.minimum_clearance_m,
        result.mean_tracking_error_m,
        result.maximum_tracking_error_m,
        result.jerk_rms_mps3,
        result.final_goal_distance_m,
        result.failure_reason,
    )


def _local_result_is_finite(result: object) -> bool:
    scalars = [result.cost, result.minimum_clearance]
    poses = list(result.path) + [point.pose for point in result.trajectory]
    return (
        result.elapsed_ns >= 0
        and all(value is None or isfinite(value) for value in scalars)
        and all(isfinite(value) for pose in poses for value in (pose.x, pose.y, pose.yaw))
        and all(
            isfinite(value)
            for point in result.trajectory
            for value in (point.time_s, point.twist.linear, point.twist.angular)
        )
    )


def _simulation_is_finite(result: SimulationResult) -> bool:
    scalars = (
        result.elapsed_s,
        result.mean_tracking_error_m,
        result.maximum_tracking_error_m,
        result.jerk_rms_mps3,
        result.final_goal_distance_m,
    )
    return (
        all(isfinite(value) for value in scalars)
        and (result.minimum_clearance_m is None or isfinite(result.minimum_clearance_m))
        and all(isfinite(value) for pose in result.poses for value in (pose.x, pose.y, pose.yaw))
        and all(
            isfinite(value)
            for command in result.commands
            for value in (command.linear, command.angular)
        )
    )


def _write_json(path: Path, value: object) -> None:
    path.write_text(
        json.dumps(
            value,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
