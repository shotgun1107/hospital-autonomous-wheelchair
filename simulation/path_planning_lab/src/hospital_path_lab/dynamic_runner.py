"""동결된 동적 Actor 비교의 공개·hidden paired runner.

결과는 open-loop 원형 Actor와 합성 관측을 사용하는 Python ``simulation_only``
연구 증거다. 제품 알고리즘이나 실제 사람 탑승 안전성을 결정하지 않는다.
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
import sys
import tracemalloc
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, replace
from hashlib import sha256
from math import hypot
from multiprocessing import get_context
from pathlib import Path
from random import Random
from statistics import median
from time import perf_counter_ns

import numpy as np

from hospital_path_lab.contracts import RobotState
from hospital_path_lab.corpus_records import preserve_dynamic_hidden_failure
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicMotionState,
)
from hospital_path_lab.dynamic_corpus import (
    DYNAMIC_CORPUS_GENERATOR_VERSION,
    DynamicCorpusEpisode,
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    build_dynamic_grid_snapshot,
    dynamic_contract_fault_cases,
    generate_dynamic_corpus,
    generate_dynamic_hidden_corpus,
    generate_episode_observation_slots,
    paired_controller_snapshots,
    validate_dynamic_corpus,
    validate_dynamic_hidden_corpus,
)
from hospital_path_lab.dynamic_evaluation import evaluate_dynamic_pipeline
from hospital_path_lab.dynamic_observation import (
    DYNAMIC_OBSERVATION_GENERATOR_VERSION,
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationAvailability,
    DynamicObservationFrameKind,
    DynamicObservationProfile,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    dynamic_observation_content_hash,
)
from hospital_path_lab.dynamic_prediction import build_actor_prediction_set
from hospital_path_lab.dynamic_safety import (
    DynamicSafetyContext,
    DynamicSafetyGate,
    build_resume_authorization,
)
from hospital_path_lab.experiment_visualization import save_dynamic_pipeline_plot
from hospital_path_lab.followers import DynamicPurePursuitController
from hospital_path_lab.local_algorithms import DynamicDwaController
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.simulation import (
    DynamicControllerPipelineResult,
    simulate_dynamic_controller_pipeline,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1

LAB_ROOT = Path(__file__).resolve().parents[2]
REPOSITORY_ROOT = LAB_ROOT.parents[1]
DYNAMIC_RUNNER_VERSION = "dynamic_runner_v1"
NUMERIC_TOLERANCE_VERSION = "dynamic_numeric_tolerance_v1"
_CONTROLLER_DEADLINE_NS = 50_000_000
_SCOPE_SENTENCE = (
    "이 결과는 open-loop 원형 Actor와 동결된 합성 관측을 사용하는 Python "
    "simulation_only 비교이며 제품 알고리즘 또는 실제 사람 탑승 안전성의 증거가 아니다."
)


@dataclass(frozen=True, slots=True)
class DynamicExperimentConfig:
    base_seed: int
    hidden_seed: int
    hidden_seed_commitment: str
    bootstrap_iterations: int = 10_000
    qualification_warmups: int = 30
    qualification_repeats: int = 100
    profiles: tuple[str, ...] = ("normal", "stress")
    public_episode_limit: int | None = None
    hidden_episode_limit: int | None = None
    evaluation_tick_limit: int | None = None
    simulation_workers: int | None = None
    contract_test_evidence: bool | None = None
    generate_visualizations: bool = True

    def __post_init__(self) -> None:
        if not self.hidden_seed_commitment:
            raise ValueError("hidden seed commitment must not be empty")
        if min(
            self.bootstrap_iterations,
            self.qualification_repeats,
        ) <= 0 or self.qualification_warmups < 0:
            raise ValueError("runner repeat counts must be positive")
        if not self.profiles or any(name not in {"normal", "stress"} for name in self.profiles):
            raise ValueError("profiles must contain normal and/or stress")
        for limit in (
            self.public_episode_limit,
            self.hidden_episode_limit,
            self.evaluation_tick_limit,
            self.simulation_workers,
        ):
            if limit is not None and limit <= 0:
                raise ValueError("episode limits must be positive or None")


@dataclass(frozen=True, slots=True)
class DynamicExperimentResult:
    output_directory: Path
    manifest_path: Path
    paired_results_path: Path
    statistics_path: Path
    promotion_path: Path
    summary_path: Path
    public_run_count: int
    hidden_run_count: int
    hard_failure_count: int
    promoted_dwa: bool
    simulation_worker_count: int


@dataclass(frozen=True, slots=True)
class _EpisodeProfileJob:
    order: int
    episode: DynamicCorpusEpisode
    profile_name: str
    output_directory: str
    hidden: bool
    generate_visualizations: bool


class _EpisodeContextFactory:
    def __init__(
        self,
        episode: DynamicCorpusEpisode,
        profile: DynamicObservationProfile,
    ) -> None:
        self.episode = episode
        self.profile = profile
        self.source = DynamicObservationSourceIdentity(
            stream_id="dynamic-stage5-stream",
            episode_id=episode.episode_id,
            episode_seed=episode.seed,
            map_id=episode.map_id,
            map_revision=1,
        )
        self.slots = generate_episode_observation_slots(episode, profile=profile)
        self.validator = DynamicObservationValidator(self.source, profile)
        self._next_slot = 0
        self._grid_by_tick: dict[int, object] = {}

    def __call__(
        self,
        tick_id: int,
        simulation_time_s: float,
        _state: RobotState,
        gate: DynamicSafetyGate,
    ) -> DynamicSafetyContext:
        self._deliver_available_slots(simulation_time_s)
        observation = self.validator.snapshot(control_time_s=simulation_time_s)
        prediction = (
            build_actor_prediction_set(observation) if observation.usable else None
        )
        observation_revision = (
            observation.frame.observation_revision
            if observation.frame is not None
            else 0
        )
        grid = build_dynamic_grid_snapshot(
            self.episode,
            observation_revision=observation_revision,
        )
        self._grid_by_tick[tick_id] = grid
        authorization = None
        if (
            gate.motion_state is DynamicMotionState.HOLDING
            and gate.stop_confirmed_at_s is not None
        ):
            authorization = build_resume_authorization(
                mission_id=self.episode.mission_id,
                stop_epoch=gate.stop_epoch,
                issued_or_revalidated_at_s=simulation_time_s,
                authorization_revision=1,
            )
        observation_safe = bool(
            observation.availability is DynamicObservationAvailability.FRESH
            and observation.frame is not None
            and observation.frame.frame_kind is DynamicObservationFrameKind.EMPTY
            and not observation.last_event_was_no_frame
        )
        return DynamicSafetyContext(
            tick_id=tick_id,
            simulation_time_s=simulation_time_s,
            mission_id=self.episode.mission_id,
            authorization_revision=1,
            grid_snapshot=grid,
            observation_snapshot=observation,
            prediction_set=prediction,
            path_still_valid=True,
            local_safety_recheck_passed=True,
            observation_safe=observation_safe,
            resume_authorization=authorization,
        )

    def grid_at(self, tick_id: int):
        return self._grid_by_tick[tick_id]

    def _deliver_available_slots(self, simulation_time_s: float) -> None:
        while self._next_slot < len(self.slots):
            slot = self.slots[self._next_slot]
            if slot.scheduled_delivery_at_s > simulation_time_s + 1e-12:
                break
            if slot.frame is None:
                self.validator.record_no_frame(
                    sequence=slot.sequence,
                    delivery_time_s=slot.scheduled_delivery_at_s,
                )
            else:
                frame = slot.frame
                if (
                    self.episode.observation_fault == "source_invalid_then_recovers"
                    and slot.scheduled_delivery_at_s < 2.0
                ):
                    frame = replace(
                        frame,
                        stream_id="fault-invalid-stream",
                        content_hash="pending",
                    )
                    frame = replace(
                        frame,
                        content_hash=dynamic_observation_content_hash(frame),
                    )
                self.validator.accept(
                    frame,
                    received_at_s=slot.scheduled_delivery_at_s,
                )
            self._next_slot += 1


def run_dynamic_experiment(
    output_directory: str | Path,
    config: DynamicExperimentConfig,
) -> DynamicExperimentResult:
    output = Path(output_directory)
    manifest_path = output / "experiment_manifest.json"
    if manifest_path.exists():
        raise FileExistsError("dynamic experiment output already contains a manifest")
    output.mkdir(parents=True, exist_ok=True)

    _configure_numeric_thread_environment()
    source_freeze_hash = _source_freeze_hash()
    simulation_workers = _resolved_simulation_workers(config.simulation_workers)
    _assert_hidden_commitment_unused(
        output.parent,
        config.hidden_seed_commitment,
        current_output=output,
    )
    public_corpus = generate_dynamic_corpus(base_seed=config.base_seed)
    public_validation = validate_dynamic_corpus(public_corpus)
    if not public_validation.passed:
        raise ValueError(f"public corpus validation failed: {public_validation.failures}")
    selected_public = _limited(public_corpus, config.public_episode_limit)
    contract_results = _contract_fault_qualification(config)

    public_records = _run_corpus(
        selected_public,
        config=config,
        output_directory=output,
        hidden=False,
        worker_count=simulation_workers,
    )
    public_prequalification = {
        "contract_fault_passed": bool(contract_results["passed"]),
        "hard_safety_failures": _hard_failure_records(public_records),
    }
    public_prequalification["passed"] = bool(
        public_prequalification["contract_fault_passed"]
        and not public_prequalification["hard_safety_failures"]
    )
    _write_json(output / "public_prequalification.json", public_prequalification)
    if not public_prequalification["passed"]:
        raise RuntimeError("public hard-safety or contract-fault qualification failed")

    qualification = _run_wall_clock_qualification(public_corpus, config)
    manifest = _build_manifest(
        config,
        public_validation=public_validation,
        public_corpus=public_corpus,
        source_freeze_hash=source_freeze_hash,
        qualification=qualification,
        simulation_workers=simulation_workers,
    )
    _write_hashed_json(manifest_path, manifest)

    # Manifest가 디스크에 봉인된 뒤에만 hidden seed를 해제한다.
    hidden_corpus = generate_dynamic_hidden_corpus(
        hidden_seed=config.hidden_seed,
        expected_commitment=config.hidden_seed_commitment,
    )
    hidden_validation = validate_dynamic_hidden_corpus(
        hidden_corpus,
        public_corpus=public_corpus,
    )
    if not hidden_validation.passed:
        raise ValueError(f"hidden corpus validation failed: {hidden_validation.failures}")
    receipt = {
        "hidden_seed_commitment": config.hidden_seed_commitment,
        "hidden_corpus_hash": hidden_validation.corpus_content_hash,
        "source_freeze_hash": source_freeze_hash,
    }
    receipt["receipt_content_hash"] = canonical_content_hash(receipt)
    _write_exclusive_json(output / "hidden_consumption_receipt.json", receipt)
    selected_hidden = _limited(hidden_corpus, config.hidden_episode_limit)
    hidden_records = _run_corpus(
        selected_hidden,
        config=config,
        output_directory=output,
        hidden=True,
        worker_count=simulation_workers,
    )
    if _source_freeze_hash() != source_freeze_hash:
        raise RuntimeError("source changed after hidden manifest freeze")

    all_records = public_records + hidden_records
    statistics = compute_paired_statistics(
        hidden_records,
        bootstrap_iterations=config.bootstrap_iterations,
        bootstrap_seed=config.hidden_seed,
    )
    promotion = _promotion_decision(
        all_records,
        hidden_records,
        statistics,
        qualification,
        contract_results,
        full_run=(
            config.public_episode_limit is None
            and config.hidden_episode_limit is None
            and config.evaluation_tick_limit is None
            and set(config.profiles) == {"normal", "stress"}
        ),
    )
    pareto = _pareto_summary(hidden_records)
    hard_failures = _hard_failure_records(all_records)
    regression_paths = _preserve_hidden_failures(
        selected_hidden,
        hidden_records,
        output / "regression_candidates",
    )

    paired_path = output / "paired_episode_results.json"
    statistics_path = output / "paired_statistics.json"
    promotion_path = output / "promotion_decision.json"
    summary_path = output / "summary.md"
    _write_json(
        output / "qualification_results.json",
        qualification,
    )
    _write_json(output / "contract_fault_results.json", contract_results)
    _write_json(output / "hard_safety_results.json", hard_failures)
    _write_json(paired_path, all_records)
    _write_json(statistics_path, statistics)
    _write_json(output / "pareto_summary.json", pareto)
    _write_json(promotion_path, promotion)
    summary_path.write_text(
        _summary_markdown(
            promotion,
            statistics,
            qualification,
            public_records,
            hidden_records,
            regression_paths,
        ),
        encoding="utf-8",
        newline="\n",
    )

    return DynamicExperimentResult(
        output_directory=output,
        manifest_path=manifest_path,
        paired_results_path=paired_path,
        statistics_path=statistics_path,
        promotion_path=promotion_path,
        summary_path=summary_path,
        public_run_count=len(public_records),
        hidden_run_count=len(hidden_records),
        hard_failure_count=len(hard_failures),
        promoted_dwa=bool(promotion["promote_dynamic_dwa"]),
        simulation_worker_count=simulation_workers,
    )


def compute_paired_statistics(
    records: list[dict[str, object]],
    *,
    bootstrap_iterations: int,
    bootstrap_seed: int,
) -> dict[str, object]:
    normal = [record for record in records if record["observation_profile"] == "normal"]
    index = {
        (record["episode_id"], record["controller_name"]): record for record in normal
    }
    pairs: list[tuple[dict[str, object], dict[str, object]]] = []
    for episode_id in sorted({record["episode_id"] for record in normal}):
        pp = index.get((episode_id, "dynamic_pure_pursuit"))
        dwa = index.get((episode_id, "dynamic_dwa"))
        if pp is not None and dwa is not None and bool(pp["progressable"]):
            pairs.append((pp, dwa))

    complete_pairs = [pair for pair in pairs if _completed_pair(pair)]
    pp_times = [float(pair[0]["metrics"]["completion_time_s"]) for pair in complete_pairs]
    dwa_times = [float(pair[1]["metrics"]["completion_time_s"]) for pair in complete_pairs]
    time_improvement = _improvement(pp_times, dwa_times)
    pp_holds = [float(pair[0]["metrics"]["safety_hold_duration_s"]) for pair in pairs]
    dwa_holds = [float(pair[1]["metrics"]["safety_hold_duration_s"]) for pair in pairs]
    hold_improvement = (
        _improvement(pp_holds, dwa_holds)
        if pp_holds and median(pp_holds) > 0.0
        else None
    )

    selected_metric: str | None = None
    selected_pairs: list[tuple[str, float]] = []
    if time_improvement is not None and time_improvement >= 0.15:
        selected_metric = "completion_time_s"
        selected_pairs = [
            (
                str(pp["expectation_category"]),
                float(dwa["metrics"]["completion_time_s"])
                - float(pp["metrics"]["completion_time_s"]),
            )
            for pp, dwa in complete_pairs
        ]
    elif hold_improvement is not None and hold_improvement >= 0.20:
        selected_metric = "safety_hold_duration_s"
        selected_pairs = [
            (
                str(pp["expectation_category"]),
                float(dwa["metrics"]["safety_hold_duration_s"])
                - float(pp["metrics"]["safety_hold_duration_s"]),
            )
            for pp, dwa in pairs
        ]
    confidence_interval = (
        stratified_paired_bootstrap_ci(
            selected_pairs,
            iterations=bootstrap_iterations,
            seed=bootstrap_seed,
        )
        if selected_pairs
        else None
    )

    comfort: dict[str, object] = {}
    for metric, floor in (
        ("longitudinal_jerk_rms_mps3", 0.10),
        ("angular_acceleration_rms_radps2", 0.10),
        ("angular_jerk_rms_radps3", 0.10),
    ):
        pp_values = [float(pp["metrics"][metric]) for pp, _ in pairs]
        dwa_values = [float(dwa["metrics"][metric]) for _, dwa in pairs]
        comfort[metric] = metric_worsening(pp_values, dwa_values, denominator_floor=floor)
    return {
        "population": "normal_hidden_progressable_paired_episode_ids",
        "paired_episode_count": len(pairs),
        "complete_paired_episode_count": len(complete_pairs),
        "time_improvement": time_improvement,
        "hold_improvement": hold_improvement,
        "selected_improvement_metric": selected_metric,
        "paired_delta_bootstrap_95ci": confidence_interval,
        "bootstrap_iterations": bootstrap_iterations,
        "comfort_worsening": comfort,
    }


def stratified_paired_bootstrap_ci(
    paired_deltas: list[tuple[str, float]],
    *,
    iterations: int,
    seed: int,
) -> dict[str, float]:
    if not paired_deltas or iterations <= 0:
        raise ValueError("bootstrap requires paired deltas and positive iterations")
    groups: dict[str, list[float]] = {}
    for category, delta in paired_deltas:
        groups.setdefault(category, []).append(delta)
    rng = Random(seed)
    samples: list[float] = []
    for _ in range(iterations):
        draw: list[float] = []
        for category in sorted(groups):
            values = groups[category]
            draw.extend(values[rng.randrange(len(values))] for _ in values)
        samples.append(float(median(draw)))
    return {
        "lower": float(np.percentile(samples, 2.5)),
        "upper": float(np.percentile(samples, 97.5)),
    }


def metric_worsening(
    pp_values: list[float],
    dwa_values: list[float],
    *,
    denominator_floor: float,
) -> float | None:
    if not pp_values or len(pp_values) != len(dwa_values) or denominator_floor <= 0.0:
        return None
    pp_median = float(median(pp_values))
    dwa_median = float(median(dwa_values))
    return (dwa_median - pp_median) / max(abs(pp_median), denominator_floor)


def _run_corpus(
    episodes: tuple[DynamicCorpusEpisode, ...],
    *,
    config: DynamicExperimentConfig,
    output_directory: Path,
    hidden: bool,
    worker_count: int,
) -> list[dict[str, object]]:
    jobs = tuple(
        _EpisodeProfileJob(
            order=order,
            episode=episode,
            profile_name=profile_name,
            output_directory=str(output_directory),
            hidden=hidden,
            generate_visualizations=config.generate_visualizations,
        )
        for order, (episode, profile_name) in enumerate(
            (episode, profile_name)
            for episode in episodes
            for profile_name in config.profiles
        )
    )
    tick_limit = config.evaluation_tick_limit
    if worker_count == 1:
        completed = [_run_episode_profile_job(job, tick_limit) for job in jobs]
    else:
        completed = []
        with ProcessPoolExecutor(
            max_workers=worker_count,
            mp_context=get_context("spawn"),
        ) as executor:
            futures = {
                executor.submit(_run_episode_profile_job, job, tick_limit): job.order
                for job in jobs
            }
            for future in as_completed(futures):
                completed.append(future.result())
    completed.sort(key=lambda item: item[0])
    return [record for _, job_records in completed for record in job_records]


def _run_episode_profile_job(
    job: _EpisodeProfileJob,
    tick_limit: int | None,
) -> tuple[int, tuple[dict[str, object], ...]]:
    episode = job.episode
    profile = _profile(job.profile_name)
    records: list[dict[str, object]] = []
    for controller in (
        DynamicPurePursuitController(),
        DynamicDwaController(),
    ):
        context = _EpisodeContextFactory(episode, profile)
        started = perf_counter_ns()
        pipeline = simulate_dynamic_controller_pipeline(
            controller,
            initial_state=episode.initial_state,
            reference_path=episode.reference_path,
            goal=episode.goal_pose,
            context_factory=context,
            max_ticks=(
                episode.tick_count
                if tick_limit is None
                else min(episode.tick_count, tick_limit)
            ),
        )
        worker_elapsed_ns = perf_counter_ns() - started
        evaluation = evaluate_dynamic_pipeline(
            pipeline,
            episode_id=episode.episode_id,
            expectation_category=episode.expectation_category.value,
            progressable=episode.progressable,
            reference_path=episode.reference_path,
            goal_pose=episode.goal_pose,
            actor_states_at=episode.actor_states_at,
            grid_snapshot_at=context.grid_at,
            blocking_cleared_at_s=episode.blocking_cleared_at_s,
        )
        records.append(
            _run_record(
                episode,
                job.profile_name,
                pipeline,
                evaluation,
                worker_elapsed_ns,
            )
        )
        if job.hidden and job.generate_visualizations:
            save_dynamic_pipeline_plot(
                episode,
                pipeline,
                Path(job.output_directory)
                / "visualizations"
                / episode.episode_id
                / f"{job.profile_name}_{controller.name}.png",
            )
    return job.order, tuple(records)


def _run_record(
    episode: DynamicCorpusEpisode,
    profile_name: str,
    pipeline: DynamicControllerPipelineResult,
    evaluation: object,
    worker_elapsed_ns: int,
) -> dict[str, object]:
    steps = pipeline.steps
    nonzero_proposals = sum(
        abs(step.controller_result.requested_twist.linear) > 1e-12
        or abs(step.controller_result.requested_twist.angular) > 1e-12
        for step in steps
    )
    overridden_nonzero = sum(
        step.gate_overrode_controller
        and (
            abs(step.controller_result.requested_twist.linear) > 1e-12
            or abs(step.controller_result.requested_twist.angular) > 1e-12
        )
        for step in steps
    )
    max_consecutive_override = 0
    streak = 0
    for step in steps:
        if step.gate_overrode_controller:
            streak += 1
            max_consecutive_override = max(max_consecutive_override, streak)
        else:
            streak = 0
    reference_length = sum(
        hypot(target.x - source.x, target.y - source.y)
        for source, target in zip(
            episode.reference_path,
            episode.reference_path[1:],
            strict=False,
        )
    )
    deterministic_signature = canonical_content_hash(
        tuple(
            (
                step.controller_result.requested_twist,
                step.safety_decision.command,
                step.safety_decision.motion_state,
                step.safety_decision.primary_hold_reason,
                step.robot_state_after,
            )
            for step in steps
        )
    )
    metrics = asdict(evaluation.metrics)
    return {
        "episode_id": episode.episode_id,
        "episode_content_hash": episode.content_hash,
        "split": episode.split.value,
        "expectation_category": episode.expectation_category.value,
        "seed": episode.seed,
        "progressable": episode.progressable,
        "observation_profile": profile_name,
        "controller_name": pipeline.controller_name,
        "hard_safety": asdict(evaluation.hard_safety),
        "functional_qualified": evaluation.functional_qualified,
        "functional_failures": list(evaluation.functional_failures),
        "metrics": metrics,
        "pipeline": {
            "status": pipeline.status.value,
            "completed": pipeline.completed,
            "expected_hold_reached": pipeline.expected_hold_reached,
            "tick_count": len(steps),
            "failure_reason": pipeline.failure_reason,
        },
        "worker_elapsed_ns_nonqualification": worker_elapsed_ns,
        "command_state_event_hash": deterministic_signature,
        "nonzero_controller_proposal_ticks": nonzero_proposals,
        "gate_override_on_nonzero_ticks": overridden_nonzero,
        "gate_override_ratio": (
            overridden_nonzero / nonzero_proposals if nonzero_proposals else 0.0
        ),
        "maximum_consecutive_gate_override_ticks": max_consecutive_override,
        "positive_detour_ratio": (
            float(metrics["positive_detour_length_m"]) / reference_length
            if reference_length > 0.0
            else 0.0
        ),
    }


def _run_wall_clock_qualification(
    public_corpus: tuple[DynamicCorpusEpisode, ...],
    config: DynamicExperimentConfig,
) -> dict[str, object]:
    selected_categories = (
        DynamicExpectationCategory.OBSERVATION_INVALID,
        DynamicExpectationCategory.WAIT_AND_RESUME,
        DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP,
        DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE,
    )
    episodes = tuple(
        next(
            episode
            for episode in public_corpus
            if episode.split is DynamicCorpusSplit.GOLDEN
            and episode.expectation_category is category
        )
        for category in selected_categories
    )
    snapshots = tuple(paired_controller_snapshots(episode)[0] for episode in episodes)
    records: dict[str, object] = {}
    for controller in (DynamicPurePursuitController(), DynamicDwaController()):
        for snapshot in snapshots:
            for _ in range(config.qualification_warmups):
                controller.step(snapshot)
        elapsed: list[int] = []
        for snapshot in snapshots:
            for _ in range(config.qualification_repeats):
                started = perf_counter_ns()
                controller.step(snapshot)
                elapsed.append(perf_counter_ns() - started)
        tracemalloc.start()
        try:
            controller.step(snapshots[-1])
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
        array = np.asarray(elapsed, dtype=np.int64)
        records[controller.name] = {
            "samples": len(elapsed),
            "p50_ns": int(np.percentile(array, 50)),
            "p95_ns": int(np.percentile(array, 95)),
            "p99_ns": int(np.percentile(array, 99)),
            "maximum_ns": max(elapsed),
            "deadline_ns": _CONTROLLER_DEADLINE_NS,
            "deadline_miss_count": sum(value > _CONTROLLER_DEADLINE_NS for value in elapsed),
            "peak_memory_bytes": int(peak),
        }
    return {
        "machine_identifier": _machine_identifier(),
        "execution_mode": "serial_parent_after_simulation_worker_pool_shutdown",
        "parallelized": False,
        "numeric_thread_environment": {
            variable: os.environ.get(variable)
            for variable in (
                "OMP_NUM_THREADS",
                "OPENBLAS_NUM_THREADS",
                "MKL_NUM_THREADS",
                "NUMEXPR_NUM_THREADS",
            )
        },
        "snapshot_set_hash": canonical_content_hash(
            tuple(snapshot.input_content_hash for snapshot in snapshots)
        ),
        "warmups_per_snapshot": config.qualification_warmups,
        "repeats_per_snapshot": config.qualification_repeats,
        "controllers": records,
    }


def _contract_fault_qualification(config: DynamicExperimentConfig) -> dict[str, object]:
    test_files = (
        "tests/test_dynamic_observation.py",
        "tests/test_dynamic_authority.py",
        "tests/test_dynamic_timing.py",
        "tests/test_dynamic_contract_faults.py",
    )
    evidence_hash = _files_hash(tuple(LAB_ROOT / path for path in test_files))
    if config.contract_test_evidence is None:
        completed = subprocess.run(
            [
                sys.executable,
                "-m",
                "pytest",
                "-c",
                str(LAB_ROOT / "pyproject.toml"),
                *(str(LAB_ROOT / path) for path in test_files),
                "-q",
            ],
            cwd=REPOSITORY_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        passed = completed.returncode == 0
        command_result = {
            "returncode": completed.returncode,
            "stdout_tail": completed.stdout[-2000:],
            "stderr_tail": completed.stderr[-2000:],
        }
    else:
        passed = config.contract_test_evidence
        command_result = {"injected_test_evidence": config.contract_test_evidence}
    return {
        "passed": passed,
        "test_source_hash": evidence_hash,
        "case_count": len(dynamic_contract_fault_cases()),
        "cases": [asdict(case) for case in dynamic_contract_fault_cases()],
        "pytest": command_result,
    }


def _build_manifest(
    config: DynamicExperimentConfig,
    *,
    public_validation: object,
    public_corpus: tuple[DynamicCorpusEpisode, ...],
    source_freeze_hash: str,
    qualification: dict[str, object],
    simulation_workers: int,
) -> dict[str, object]:
    commit_hash, dirty = _git_state()
    manifest = {
        "schema_version": "1.0",
        "runner_version": DYNAMIC_RUNNER_VERSION,
        "simulation_only": True,
        "code_commit_hash": commit_hash,
        "working_tree_dirty": dirty,
        "source_freeze_hash": source_freeze_hash,
        "map_corpus_hash": public_validation.corpus_content_hash,
        "public_episode_count": len(public_corpus),
        "pp_parameter_hash": canonical_content_hash(_pp_parameters()),
        "dwa_parameter_hash": canonical_content_hash(_dwa_parameters()),
        "safety_gate_parameter_hash": canonical_content_hash(_gate_parameters()),
        "observation_generator_hash": canonical_content_hash(
            {
                "version": DYNAMIC_OBSERVATION_GENERATOR_VERSION,
                "normal": NORMAL_OBSERVATION_PROFILE,
                "stress": STRESS_OBSERVATION_PROFILE,
            }
        ),
        "scenario_generator_hash": canonical_content_hash(
            {"version": DYNAMIC_CORPUS_GENERATOR_VERSION, "base_seed": config.base_seed}
        ),
        "simulator_version": DYNAMIC_RUNNER_VERSION,
        "hidden_seed_commitment": config.hidden_seed_commitment,
        "qualification_snapshot_set_hash": qualification["snapshot_set_hash"],
        "machine_identifier": qualification["machine_identifier"],
        "tuning_access_count_by_controller": {
            "dynamic_pure_pursuit": 1,
            "dynamic_dwa": 1,
        },
        "numeric_tolerance_version": NUMERIC_TOLERANCE_VERSION,
        "profiles": list(config.profiles),
        "simulation_execution": {
            "mode": "process_parallel_episode_profile_jobs",
            "worker_count": simulation_workers,
            "paired_unit": "same_episode_and_profile_pp_then_dwa",
            "result_order": "corpus_then_profile_then_controller",
        },
        "timing_qualification_execution": "serial_parent_after_worker_pool_shutdown",
    }
    manifest["manifest_content_hash"] = canonical_content_hash(manifest)
    return manifest


def _promotion_decision(
    all_records: list[dict[str, object]],
    hidden_records: list[dict[str, object]],
    statistics: dict[str, object],
    qualification: dict[str, object],
    contract_results: dict[str, object],
    *,
    full_run: bool,
) -> dict[str, object]:
    normal_hidden = [
        record
        for record in hidden_records
        if record["observation_profile"] == "normal"
    ]
    progress = [record for record in normal_hidden if record["progressable"]]
    feasible_dwa = [
        record
        for record in normal_hidden
        if record["controller_name"] == "dynamic_dwa"
        and record["expectation_category"]
        == DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE.value
    ]
    forbidden_dwa = [
        record
        for record in normal_hidden
        if record["controller_name"] == "dynamic_dwa"
        and record["expectation_category"]
        in {
            DynamicExpectationCategory.LOCAL_DETOUR_FORBIDDEN.value,
            DynamicExpectationCategory.NO_SAFE_SOLUTION.value,
        }
    ]
    detour_success_rate = (
        sum(
            bool(record["metrics"]["rejoin_observed"])
            and float(record["metrics"]["maximum_reference_deviation_m"]) > 0.10
            and bool(record["functional_qualified"])
            for record in feasible_dwa
        )
        / len(feasible_dwa)
        if feasible_dwa
        else 0.0
    )
    ci = statistics["paired_delta_bootstrap_95ci"]
    improvement = (
        (
            statistics["time_improvement"] is not None
            and float(statistics["time_improvement"]) >= 0.15
        )
        or (
            statistics["hold_improvement"] is not None
            and float(statistics["hold_improvement"]) >= 0.20
        )
    ) and ci is not None and float(ci["upper"]) < 0.0
    comfort_values = statistics["comfort_worsening"].values()
    controllers = qualification["controllers"]
    conditions = {
        "01_both_controllers_normal_stress_hard_safety": all(
            bool(record["hard_safety"]["passed"]) for record in hidden_records
        ),
        "02_common_contract_fault_qualification": bool(contract_results["passed"]),
        "03_progressable_functional_qualification": bool(progress)
        and all(bool(record["functional_qualified"]) for record in progress),
        "04_normal_deadline_and_late_application": all(
            int(item["deadline_miss_count"]) == 0 for item in controllers.values()
        )
        and all(
            int(record["hard_safety"]["late_command_applied_count"]) == 0
            for record in normal_hidden
        ),
        "05_dwa_feasible_detour_rejoin_rate": bool(feasible_dwa)
        and detour_success_rate >= 0.80,
        "06_forbidden_or_no_safe_overtaking_zero": all(
            not bool(record["metrics"]["overtaking_observed"])
            for record in forbidden_dwa
        ),
        "07_time_or_hold_improvement_with_ci": improvement,
        "08_comfort_worsening_within_25_percent": bool(comfort_values)
        and all(value is not None and float(value) <= 0.25 for value in comfort_values),
        "09_detour_and_deviation_bounds": bool(normal_hidden)
        and all(
            float(record["positive_detour_ratio"]) <= 0.30
            and float(record["metrics"]["maximum_reference_deviation_m"]) <= 0.50
            for record in normal_hidden
            if record["controller_name"] == "dynamic_dwa"
            and bool(record["pipeline"]["completed"])
        ),
        "10_gate_override_bounds": bool(normal_hidden)
        and all(
            float(record["gate_override_ratio"]) <= 0.05
            and int(record["maximum_consecutive_gate_override_ticks"]) <= 3
            for record in normal_hidden
            if record["controller_name"] == "dynamic_dwa"
        ),
    }
    promote = full_run and all(conditions.values())
    return {
        "scope": _SCOPE_SENTENCE,
        "full_frozen_run": full_run,
        "conditions": conditions,
        "evidence": {
            "dwa_feasible_detour_rejoin_rate": detour_success_rate,
            "statistics": statistics,
            "qualification": qualification,
        },
        "promote_dynamic_dwa": promote,
        "research_baseline": (
            "dynamic_dwa_plus_shared_gate"
            if promote
            else "pure_pursuit_plus_shared_gate"
        ),
    }


def _pareto_summary(records: list[dict[str, object]]) -> dict[str, object]:
    groups: dict[str, list[dict[str, object]]] = {}
    for record in records:
        key = f"{record['observation_profile']}:{record['controller_name']}"
        groups.setdefault(key, []).append(record)
    summary: dict[str, object] = {}
    for key, items in sorted(groups.items()):
        summary[key] = {
            "samples": len(items),
            "hard_safety_passes": sum(bool(item["hard_safety"]["passed"]) for item in items),
            "functional_passes": sum(bool(item["functional_qualified"]) for item in items),
            "median_completion_time_s": _median_optional(
                [item["metrics"]["completion_time_s"] for item in items]
            ),
            "median_hold_duration_s": float(
                median(float(item["metrics"]["safety_hold_duration_s"]) for item in items)
            ),
            "worst_minimum_clearance_m": min(
                float(item["metrics"]["minimum_surface_clearance_m"])
                for item in items
                if item["metrics"]["minimum_surface_clearance_m"] is not None
            ),
            "median_worker_elapsed_ns_nonqualification": int(
                median(
                    int(item["worker_elapsed_ns_nonqualification"])
                    for item in items
                )
            ),
        }
    return {"scope": _SCOPE_SENTENCE, "groups": summary}


def _preserve_hidden_failures(
    hidden_episodes: tuple[DynamicCorpusEpisode, ...],
    records: list[dict[str, object]],
    output_directory: Path,
) -> list[str]:
    by_id = {episode.episode_id: episode for episode in hidden_episodes}
    paths: list[str] = []
    for record in records:
        hard_passed = bool(record["hard_safety"]["passed"])
        functional_passed = bool(record["functional_qualified"])
        if hard_passed and functional_passed:
            continue
        first_failure = record["hard_safety"]["first_failure_time_s"]
        failing_tick = (
            round(float(first_failure) / DYNAMIC_CONTROL_PERIOD_S)
            if first_failure is not None
            else int(record["pipeline"]["tick_count"])
        )
        reasons = list(record["hard_safety"]["failures"]) + list(
            record["functional_failures"]
        )
        path = preserve_dynamic_hidden_failure(
            by_id[str(record["episode_id"])],
            observation_profile=str(record["observation_profile"]),
            controller_name=str(record["controller_name"]),
            failing_tick=failing_tick,
            reason=",".join(reasons) or "unknown_failure",
            output_directory=output_directory,
            minimal_evidence={
                "hard_safety": record["hard_safety"],
                "functional_failures": record["functional_failures"],
                "command_state_event_hash": record["command_state_event_hash"],
            },
        )
        paths.append(str(path))
    return paths


def _hard_failure_records(records: list[dict[str, object]]) -> list[dict[str, object]]:
    failures: list[dict[str, object]] = []
    for record in records:
        if not record["hard_safety"]["passed"]:
            failures.append(
                {
                    "episode_id": record["episode_id"],
                    "split": record["split"],
                    "profile": record["observation_profile"],
                    "controller": record["controller_name"],
                    "failures": record["hard_safety"]["failures"],
                    "first_failure_time_s": record["hard_safety"]["first_failure_time_s"],
                }
            )
    return failures


def _summary_markdown(
    promotion: dict[str, object],
    statistics: dict[str, object],
    qualification: dict[str, object],
    public_records: list[dict[str, object]],
    hidden_records: list[dict[str, object]],
    regression_paths: list[str],
) -> str:
    conditions = promotion["conditions"]
    lines = [
        _SCOPE_SENTENCE,
        "",
        "# 동적 Actor PP·DWA 비교 결과",
        "",
        f"- 공개 run: `{len(public_records)}`",
        f"- hidden run: `{len(hidden_records)}`",
        f"- DWA 연구 기준선 승격: `{promotion['promote_dynamic_dwa']}`",
        f"- 유지 기준선: `{promotion['research_baseline']}`",
        f"- regression 후보: `{len(regression_paths)}`",
        "",
        "## 승격 조건",
        "",
    ]
    lines.extend(f"- `{name}`: `{passed}`" for name, passed in conditions.items())
    lines.extend(
        (
            "",
            "## paired 통계",
            "",
            f"- time improvement: `{statistics['time_improvement']}`",
            f"- hold improvement: `{statistics['hold_improvement']}`",
            f"- bootstrap 95% CI: `{statistics['paired_delta_bootstrap_95ci']}`",
            "",
            "## wall-clock qualification",
            "",
            f"- machine: `{qualification['machine_identifier']}`",
            "",
            "최종 제품 알고리즘, G1~G5 또는 실제 사람 탑승 안전성은 결정하지 않았다.",
            "",
        )
    )
    return "\n".join(lines)


def _source_freeze_hash() -> str:
    paths = tuple(sorted((LAB_ROOT / "src" / "hospital_path_lab").rglob("*.py"))) + (
        LAB_ROOT / "pyproject.toml",
    )
    return _files_hash(paths)


def _files_hash(paths: tuple[Path, ...]) -> str:
    digest = sha256()
    for path in sorted(paths, key=lambda item: item.as_posix()):
        digest.update(path.relative_to(REPOSITORY_ROOT).as_posix().encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def _assert_hidden_commitment_unused(
    parent: Path,
    commitment: str,
    *,
    current_output: Path,
) -> None:
    if not parent.exists():
        return
    for path in parent.rglob("hidden_consumption_receipt.json"):
        if current_output in path.parents:
            continue
        try:
            receipt = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        recorded_hash = receipt.pop("receipt_content_hash", None)
        if recorded_hash is not None and (
            not isinstance(recorded_hash, str)
            or canonical_content_hash(receipt) != recorded_hash
        ):
            raise ValueError(f"hidden consumption receipt hash mismatch: {path}")
        if receipt.get("hidden_seed_commitment") == commitment:
            raise ValueError(f"hidden commitment was already consumed: {path}")


def _profile(name: str) -> DynamicObservationProfile:
    return (
        NORMAL_OBSERVATION_PROFILE
        if name == "normal"
        else STRESS_OBSERVATION_PROFILE
    )


def _resolved_simulation_workers(requested: int | None) -> int:
    if requested is not None:
        return requested
    logical_processors = os.cpu_count() or 1
    return max(1, min(6, logical_processors // 4))


def _configure_numeric_thread_environment() -> None:
    for variable in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ[variable] = "1"


def _limited(
    episodes: tuple[DynamicCorpusEpisode, ...],
    limit: int | None,
) -> tuple[DynamicCorpusEpisode, ...]:
    return episodes if limit is None else episodes[:limit]


def _improvement(pp_values: list[float], dwa_values: list[float]) -> float | None:
    if not pp_values or len(pp_values) != len(dwa_values):
        return None
    pp_median = float(median(pp_values))
    if pp_median <= 0.0:
        return None
    return 1.0 - float(median(dwa_values)) / pp_median


def _completed_pair(pair: tuple[dict[str, object], dict[str, object]]) -> bool:
    return all(item["metrics"]["completion_time_s"] is not None for item in pair)


def _median_optional(values: list[object]) -> float | None:
    finite = [float(value) for value in values if value is not None]
    return float(median(finite)) if finite else None


def _git_state() -> tuple[str, bool]:
    head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPOSITORY_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    status = subprocess.run(
        ["git", "status", "--porcelain"],
        cwd=REPOSITORY_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    return head.stdout.strip() or "unavailable", bool(status.stdout.strip())


def _machine_identifier() -> str:
    return canonical_content_hash(
        {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "processor": platform.processor(),
            "python": platform.python_version(),
        }
    )


def _pp_parameters() -> dict[str, object]:
    controller = DynamicPurePursuitController()
    return {
        "name": controller.name,
        "lookahead_m": 0.35,
        "goal_tolerance_m": 0.05,
        "nominal_speed_mps": 0.20,
        "lookahead_rule": "nearest_polyline_projection_plus_arc_length",
        "goal_speed_rule": "min(nominal,sqrt(2*deceleration*remaining_distance))",
        "curvature_rule": "2*y_local/lookahead_distance_squared",
    }


def _dwa_parameters() -> dict[str, object]:
    controller = DynamicDwaController()
    return {
        "name": controller.name,
        "horizon_s": controller.horizon_s,
        "integration_dt_s": controller.integration_dt_s,
        "linear_samples": controller.linear_sample_count,
        "angular_samples": controller.angular_sample_count,
        "reverse_enabled": False,
        "pose_samples_per_candidate": 41,
        "terminal_stopping_sweep": True,
        "cost_contract": {
            "progress": "1-clip(progress_m/0.40,0,1)",
            "reference": "clip(mean_polyline_distance_m/0.50,0,1)",
            "heading": "clip(abs_goal_heading_error_rad/pi,0,1)",
            "clearance": "1-clip((clearance_m-0.08)/(0.50-0.08),0,1)",
            "speed": "clip((0.20-linear_mps)/0.20,0,1)",
            "oscillation": "opposite_angular_sign_above_0.05",
            "weights": (1.0, 1.0, 0.5, 1.5, 0.2, 0.3),
            "tie_break": (
                "score_asc",
                "minimum_clearance_desc",
                "progress_desc",
                "reference_cost_asc",
                "heading_cost_asc",
                "oscillation_cost_asc",
                "abs_angular_asc",
                "linear_desc",
                "angular_asc",
            ),
        },
    }


def _gate_parameters() -> dict[str, object]:
    return {
        "control_period_s": DYNAMIC_CONTROL_PERIOD_S,
        "minimum_clearance_m": VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m,
        "command_deadline_ns": _CONTROLLER_DEADLINE_NS,
        "resume_safe_frames": 11,
        "actual_stop_linear_threshold_mps": 0.01,
        "actual_stop_angular_threshold_radps": 0.02,
        "actual_stop_consecutive_ticks": 3,
        "stop_epoch_rule": "increment_once_on_distinct_protective_stop_confirmation",
    }


def _write_hashed_json(path: Path, payload: dict[str, object]) -> None:
    _write_json(path, payload)
    loaded = json.loads(path.read_text(encoding="utf-8"))
    recorded = loaded.pop("manifest_content_hash")
    if canonical_content_hash(loaded) != recorded:
        raise RuntimeError("written manifest hash did not verify")


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        + "\n",
        encoding="utf-8",
        newline="\n",
    )


def _write_exclusive_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (
        json.dumps(value, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False)
        + "\n"
    )
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(payload)
