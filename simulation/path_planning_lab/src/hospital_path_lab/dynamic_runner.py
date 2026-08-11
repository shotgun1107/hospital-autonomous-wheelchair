"""동결된 동적 Actor 비교의 공개 qualification runner.

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
from multiprocessing import active_children, get_context
from pathlib import Path
from random import Random
from statistics import median
from tempfile import TemporaryDirectory
from time import perf_counter_ns

import numpy as np

from hospital_path_lab.contracts import RobotState
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    ControllerSnapshot,
    DynamicMotionState,
    build_controller_snapshot,
)
from hospital_path_lab.dynamic_corpus import (
    DYNAMIC_CORPUS_GENERATOR_VERSION,
    DYNAMIC_V6_CORPUS_GENERATOR_VERSION,
    DYNAMIC_V6_ORACLE_VERSION,
    DynamicCorpusEpisode,
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    DynamicScenarioFamily,
    V6DynamicCorpusEpisode,
    build_dynamic_grid_snapshot,
    controller_episode_id,
    dynamic_contract_fault_cases,
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
    generate_episode_observation_slots,
    validate_dynamic_corpus,
    validate_dynamic_v6_public_corpus,
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
DYNAMIC_RUNNER_VERSION = "dynamic_runner_v6"
NUMERIC_TOLERANCE_VERSION = "dynamic_numeric_tolerance_v1"
_CONTROLLER_DEADLINE_NS = 50_000_000
_CANONICAL_PUBLIC_BASE_SEED = 20_260_811
_STANDARD_QUALIFICATION_WARMUPS = 30
_STANDARD_QUALIFICATION_REPEATS = 100
_PUBLIC_REPORT_SCHEMA = "dynamic_public_qualification_report_v6"
_PUBLIC_RECEIPT_SCHEMA = "dynamic_public_qualification_receipt_v6"
_PUBLIC_GATE_SCHEMA = "dynamic_public_qualification_gate_v6"
_QUALIFICATION_SCHEMA = "dynamic_wall_clock_qualification_v6"
_RIGID_SIGNATURE_SCHEMA = "dynamic_rigid_metamorphic_signature_v6"
_RIGID_SIGNATURE_COMPONENT_FIELDS = (
    "controller_command_trace_hash",
    "shared_gate_trace_hash",
    "pipeline_result_hash",
    "hard_safety_result_hash",
    "category_result_hash",
    "functional_result_hash",
)
_RIGID_SIGNATURE_FIELDS = frozenset(
    {
        "schema_version",
        "numeric_tolerance_version",
        *_RIGID_SIGNATURE_COMPONENT_FIELDS,
        "content_hash",
    }
)
_NONDETERMINISTIC_PUBLIC_RECORD_FIELDS = frozenset(
    {
        "worker_elapsed_ns_nonqualification",
        "worker_process_id_nonqualification",
    }
)
_PUBLIC_PHASE_ARTIFACTS = (
    "public_qualification_report.json",
    "public_qualification_receipt.json",
    "public_qualification_gate.json",
    "public_prequalification.json",
    "qualification_results.json",
    "contract_fault_results.json",
    "hard_safety_results.json",
    "paired_episode_results.json",
)
@dataclass(frozen=True, slots=True)
class DynamicPublicQualificationConfig:
    base_seed: int
    qualification_warmups: int = 30
    qualification_repeats: int = 100
    profiles: tuple[str, ...] = ("normal", "stress")
    public_episode_limit: int | None = None
    evaluation_tick_limit: int | None = None
    simulation_workers: int | None = None
    contract_test_evidence: bool | None = None
    def __post_init__(self) -> None:
        if self.qualification_warmups < 0 or self.qualification_repeats <= 0:
            raise ValueError("runner repeat counts must be positive")
        if not self.profiles or any(
            name not in {"normal", "stress"} for name in self.profiles
        ):
            raise ValueError("profiles must contain normal and/or stress")
        for limit in (
            self.public_episode_limit,
            self.evaluation_tick_limit,
            self.simulation_workers,
        ):
            if limit is not None and limit <= 0:
                raise ValueError("episode limits must be positive or None")


@dataclass(frozen=True, slots=True)
class DynamicPublicQualificationResult:
    output_directory: Path
    report_path: Path
    gate_path: Path
    paired_results_path: Path
    receipt_path: Path | None
    passed: bool
    public_run_count: int
    simulation_worker_count: int


@dataclass(frozen=True, slots=True)
class _DynamicPublicPhase:
    source_freeze_hash_before: str
    source_freeze_hash_after: str
    source_freeze_hash_before_seal: str
    source_freeze_hash_at_receipt_write: str | None
    source_freeze_consistent: bool
    simulation_workers: int
    public_validation: object
    v6_public_validation: object
    public_corpus: tuple[DynamicCorpusEpisode, ...]
    public_records: list[dict[str, object]]
    contract_results: dict[str, object]
    qualification: dict[str, object]
    public_gate: dict[str, object]
    run_scope: dict[str, object]
    public_record_set_hash: str
    scenario_oracle_matrix_hash: str
    full_evidence_hash: str
    public_receipt: dict[str, object] | None


@dataclass(frozen=True, slots=True)
class _EpisodeProfileJob:
    order: int
    episode: DynamicCorpusEpisode
    profile_name: str


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
            episode_id=controller_episode_id(episode),
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


def _public_functional_qualification(
    records: list[dict[str, object]],
    *,
    public_corpus: tuple[DynamicCorpusEpisode, ...],
) -> dict[str, object]:
    """공개 receipt를 seal하기 전에 category 기능 자격을 보수적으로 판정한다."""

    generic_failures = [
        {
            "episode_id": record["episode_id"],
            "profile": record["observation_profile"],
            "controller": record["controller_name"],
            "failures": record["functional_failures"],
        }
        for record in records
        if not bool(record["functional_qualified"])
    ]
    feasible_dwa_normal = [
        record
        for record in records
        if record["controller_name"] == "dynamic_dwa"
        and record["observation_profile"] == "normal"
        and record["expectation_category"]
        == DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE.value
    ]
    feasible_passes = [
        record
        for record in feasible_dwa_normal
        if bool(record["pipeline"]["completed"])
        and float(record["metrics"]["maximum_reference_deviation_m"]) > 0.10
        and bool(record["metrics"]["rejoin_observed"])
    ]
    golden_feasible = [
        record for record in feasible_dwa_normal if record["split"] == "golden"
    ]
    development_feasible = [
        record for record in feasible_dwa_normal if record["split"] == "development"
    ]
    golden_passed = bool(golden_feasible) and all(
        record in feasible_passes for record in golden_feasible
    )
    development_pass_count = sum(
        record in feasible_passes for record in development_feasible
    )
    development_ratio = (
        development_pass_count / len(development_feasible)
        if development_feasible
        else None
    )
    detour_rejoin_passed = bool(
        golden_passed
        and development_ratio is not None
        and development_ratio >= 0.80
    )
    rigid_pair_metamorphic = _rigid_pair_metamorphic_qualification(
        records,
        public_corpus=public_corpus,
    )
    return {
        "passed": bool(
            not generic_failures
            and detour_rejoin_passed
            and rigid_pair_metamorphic["passed"]
        ),
        "generic_functional_failures": generic_failures,
        "local_detour_feasible": {
            "golden_count": len(golden_feasible),
            "golden_passed": golden_passed,
            "development_count": len(development_feasible),
            "development_pass_count": development_pass_count,
            "development_pass_ratio": development_ratio,
            "required_development_pass_ratio": 0.80,
            "passed": detour_rejoin_passed,
        },
        "rigid_pair_metamorphic": rigid_pair_metamorphic,
    }


def _rigid_pair_metamorphic_qualification(
    records: list[dict[str, object]],
    *,
    public_corpus: tuple[DynamicCorpusEpisode, ...],
) -> dict[str, object]:
    episode_groups: dict[str, list[V6DynamicCorpusEpisode]] = {}
    for episode in public_corpus:
        if isinstance(episode, V6DynamicCorpusEpisode):
            episode_groups.setdefault(episode.latent_case_id, []).append(episode)
    rigid_pairs = {
        latent_case_id: tuple(episodes)
        for latent_case_id, episodes in episode_groups.items()
        if len(episodes) > 1
    }
    failures: list[dict[str, object]] = []
    comparisons: list[dict[str, object]] = []
    if not rigid_pairs:
        failures.append(
            {
                "latent_case_id": None,
                "controller": None,
                "profile": None,
                "reasons": ["public_rigid_pair_missing"],
            }
        )

    for latent_case_id in sorted(rigid_pairs):
        episodes = rigid_pairs[latent_case_id]
        expected_by_episode = {
            episode.episode_id: episode.orientation.value for episode in episodes
        }
        pair_contract_failures: list[str] = []
        if len(episodes) != 2:
            pair_contract_failures.append("rigid_pair_episode_count_not_two")
        if len(set(expected_by_episode.values())) != len(episodes):
            pair_contract_failures.append("rigid_pair_orientations_not_distinct")
        if len({episode.seed for episode in episodes}) != 1:
            pair_contract_failures.append("rigid_pair_seed_mismatch")

        for profile_name in ("normal", "stress"):
            for controller_name in ("dynamic_pure_pursuit", "dynamic_dwa"):
                matching = [
                    record
                    for record in records
                    if record.get("episode_id") in expected_by_episode
                    and record.get("observation_profile") == profile_name
                    and record.get("controller_name") == controller_name
                ]
                reasons = list(pair_contract_failures)
                actual_episode_ids = [str(record.get("episode_id")) for record in matching]
                if len(matching) != len(episodes):
                    reasons.append("rigid_pair_record_count_mismatch")
                if set(actual_episode_ids) != set(expected_by_episode):
                    reasons.append("rigid_pair_episode_coverage_mismatch")

                signatures: list[dict[str, object]] = []
                for record in matching:
                    scenario = record.get("scenario")
                    signature = record.get("rigid_metamorphic_signature")
                    episode_id = str(record.get("episode_id"))
                    orientation = (
                        scenario.get("orientation") if isinstance(scenario, dict) else None
                    )
                    if (
                        not isinstance(scenario, dict)
                        or scenario.get("latent_case_id") != latent_case_id
                        or orientation != expected_by_episode.get(episode_id)
                    ):
                        reasons.append("rigid_pair_record_identity_mismatch")
                    content_hash, signature_failures = (
                        _validated_rigid_metamorphic_signature(signature)
                    )
                    reasons.extend(signature_failures)
                    signatures.append(
                        {
                            "episode_id": episode_id,
                            "orientation": orientation,
                            "content_hash": content_hash,
                        }
                    )
                content_hashes = {
                    item["content_hash"]
                    for item in signatures
                    if item["content_hash"] is not None
                }
                if len(signatures) == len(episodes) and len(content_hashes) != 1:
                    reasons.append("rigid_pair_orientation_signature_mismatch")
                reasons = list(dict.fromkeys(reasons))
                comparison = {
                    "latent_case_id": latent_case_id,
                    "controller": controller_name,
                    "profile": profile_name,
                    "expected_orientations": sorted(expected_by_episode.values()),
                    "signatures": sorted(
                        signatures,
                        key=lambda item: (str(item["orientation"]), str(item["episode_id"])),
                    ),
                    "passed": not reasons,
                    "reasons": reasons,
                }
                comparisons.append(comparison)
                if reasons:
                    failures.append(
                        {
                            "latent_case_id": latent_case_id,
                            "controller": controller_name,
                            "profile": profile_name,
                            "reasons": reasons,
                        }
                    )
    return {
        "schema_version": "dynamic_rigid_pair_metamorphic_qualification_v6",
        "passed": not failures,
        "rigid_pair_count": len(rigid_pairs),
        "comparison_count": len(comparisons),
        "comparisons": comparisons,
        "failures": failures,
    }


def _validated_rigid_metamorphic_signature(
    signature: object,
) -> tuple[str | None, tuple[str, ...]]:
    if not isinstance(signature, dict):
        return None, ("rigid_pair_signature_missing_or_invalid",)
    failures: list[str] = []
    if set(signature) != _RIGID_SIGNATURE_FIELDS:
        failures.append("rigid_pair_signature_key_set_invalid")
    if signature.get("schema_version") != _RIGID_SIGNATURE_SCHEMA:
        failures.append("rigid_pair_signature_schema_invalid")
    if signature.get("numeric_tolerance_version") != NUMERIC_TOLERANCE_VERSION:
        failures.append("rigid_pair_signature_numeric_tolerance_invalid")
    for field in (*_RIGID_SIGNATURE_COMPONENT_FIELDS, "content_hash"):
        value = signature.get(field)
        if not isinstance(value, str) or not value:
            failures.append(f"rigid_pair_signature_{field}_invalid")
    if failures:
        return None, tuple(failures)
    payload = {
        field: signature[field]
        for field in _RIGID_SIGNATURE_FIELDS
        if field != "content_hash"
    }
    recomputed_hash = canonical_content_hash(payload)
    if signature["content_hash"] != recomputed_hash:
        return None, ("rigid_pair_signature_content_hash_mismatch",)
    return recomputed_hash, ()


def _wall_clock_qualification_passed(
    qualification: dict[str, object],
) -> bool:
    controllers = qualification.get("controllers")
    if (
        qualification.get("schema_version") != _QUALIFICATION_SCHEMA
        or qualification.get("status") != "completed"
        or qualification.get("passed") is not True
        or not isinstance(controllers, dict)
        or set(controllers) != {"dynamic_pure_pursuit", "dynamic_dwa"}
    ):
        return False
    return all(
        isinstance(record, dict)
        and record.get("passed") is True
        and int(record.get("samples", 0)) > 0
        and int(record.get("deadline_miss_count", -1)) == 0
        and int(record.get("maximum_ns", _CONTROLLER_DEADLINE_NS + 1))
        <= int(record.get("deadline_ns", _CONTROLLER_DEADLINE_NS))
        for record in controllers.values()
    )


def _public_run_scope(config: DynamicPublicQualificationConfig) -> dict[str, object]:
    requirements = {
        "canonical_base_seed": config.base_seed == _CANONICAL_PUBLIC_BASE_SEED,
        "full_public_corpus": config.public_episode_limit is None,
        "full_normal_stress_profiles": config.profiles == ("normal", "stress"),
        "full_episode_duration": config.evaluation_tick_limit is None,
        "measured_contract_evidence": config.contract_test_evidence is None,
        "standard_qualification_warmups": (
            config.qualification_warmups == _STANDARD_QUALIFICATION_WARMUPS
        ),
        "standard_qualification_repeats": (
            config.qualification_repeats == _STANDARD_QUALIFICATION_REPEATS
        ),
    }
    reason_by_requirement = {
        "canonical_base_seed": "noncanonical_base_seed",
        "full_public_corpus": "public_episode_limit_set",
        "full_normal_stress_profiles": "profiles_not_full_normal_stress",
        "full_episode_duration": "evaluation_tick_limit_set",
        "measured_contract_evidence": "contract_evidence_injected",
        "standard_qualification_warmups": "qualification_warmups_not_30",
        "standard_qualification_repeats": "qualification_repeats_not_100",
    }
    reasons = tuple(
        reason_by_requirement[name]
        for name, passed in requirements.items()
        if not passed
    )
    return {
        "schema_version": "dynamic_public_run_scope_v6",
        "mode": "sealing_candidate" if not reasons else "non_sealing_report",
        "sealing_eligible": not reasons,
        "non_sealing_reasons": list(reasons),
        "requirements": requirements,
        "base_seed": config.base_seed,
        "profiles": list(config.profiles),
        "public_episode_limit": config.public_episode_limit,
        "evaluation_tick_limit": config.evaluation_tick_limit,
        "contract_evidence_injected": config.contract_test_evidence is not None,
        "qualification_warmups": config.qualification_warmups,
        "qualification_repeats": config.qualification_repeats,
    }


def _public_record_set_hash(records: list[dict[str, object]]) -> str:
    stable_records = tuple(
        {
            key: value
            for key, value in record.items()
            if key not in _NONDETERMINISTIC_PUBLIC_RECORD_FIELDS
        }
        for record in records
    )
    return canonical_content_hash(stable_records)


def _public_record_coverage(
    episodes: tuple[DynamicCorpusEpisode, ...],
    *,
    profiles: tuple[str, ...],
    records: list[dict[str, object]],
) -> dict[str, object]:
    controller_names = ("dynamic_pure_pursuit", "dynamic_dwa")
    expected: dict[tuple[str, str, str], dict[str, object]] = {}
    for episode in episodes:
        expected_scenario = (
            {
                "family": episode.scenario_family.value,
                "variant": episode.variant,
                "orientation": episode.orientation.value,
                "latent_case_id": episode.latent_case_id,
                "semantic_world_hash": episode.semantic_world_hash,
                "oracle_hash": episode.oracle_hash,
            }
            if isinstance(episode, V6DynamicCorpusEpisode)
            else None
        )
        for profile_name in profiles:
            for controller_name in controller_names:
                expected[(episode.episode_id, profile_name, controller_name)] = {
                    "episode_content_hash": episode.content_hash,
                    "split": episode.split.value,
                    "expectation_category": episode.expectation_category.value,
                    "seed": episode.seed,
                    "progressable": episode.progressable,
                    "scenario": expected_scenario,
                }

    actual: dict[tuple[str, str, str], list[dict[str, object]]] = {}
    malformed_records: list[int] = []
    for index, record in enumerate(records):
        identity = (
            record.get("episode_id"),
            record.get("observation_profile"),
            record.get("controller_name"),
        )
        if not all(isinstance(item, str) for item in identity):
            malformed_records.append(index)
            continue
        key = (str(identity[0]), str(identity[1]), str(identity[2]))
        actual.setdefault(key, []).append(record)

    missing = [
        {
            "episode_id": key[0],
            "profile": key[1],
            "controller": key[2],
        }
        for key in expected
        if key not in actual
    ]
    unexpected = [
        {
            "episode_id": key[0],
            "profile": key[1],
            "controller": key[2],
        }
        for key in actual
        if key not in expected
    ]
    duplicates = [
        {
            "episode_id": key[0],
            "profile": key[1],
            "controller": key[2],
            "count": len(items),
        }
        for key, items in actual.items()
        if key in expected and len(items) != 1
    ]
    identity_failures: list[dict[str, object]] = []
    for key, expected_identity in expected.items():
        items = actual.get(key, [])
        if len(items) != 1:
            continue
        record = items[0]
        mismatched_fields = [
            field
            for field, expected_value in expected_identity.items()
            if record.get(field) != expected_value
        ]
        if mismatched_fields:
            identity_failures.append(
                {
                    "episode_id": key[0],
                    "profile": key[1],
                    "controller": key[2],
                    "mismatched_fields": mismatched_fields,
                }
            )

    pairing_failures: list[dict[str, object]] = []
    for episode in episodes:
        for profile_name in profiles:
            pair = [
                actual.get((episode.episode_id, profile_name, controller_name), [])
                for controller_name in controller_names
            ]
            if any(len(items) != 1 for items in pair):
                continue
            pp_record, dwa_record = pair[0][0], pair[1][0]
            mismatched_fields = [
                field
                for field in (
                    "pair_id",
                    "observation_stream_hash",
                    "worker_process_id_nonqualification",
                    "seed",
                )
                if pp_record.get(field) != dwa_record.get(field)
            ]
            if mismatched_fields:
                pairing_failures.append(
                    {
                        "episode_id": episode.episode_id,
                        "profile": profile_name,
                        "mismatched_fields": mismatched_fields,
                    }
                )

    passed = not any(
        (
            malformed_records,
            missing,
            unexpected,
            duplicates,
            identity_failures,
            pairing_failures,
        )
    ) and len(records) == len(expected)
    return {
        "schema_version": "dynamic_public_record_coverage_v6",
        "passed": passed,
        "expected_record_count": len(expected),
        "actual_record_count": len(records),
        "malformed_record_indexes": malformed_records,
        "missing": missing,
        "unexpected": unexpected,
        "duplicates": duplicates,
        "identity_failures": identity_failures,
        "pairing_failures": pairing_failures,
    }


def _scenario_oracle_matrix_hash(
    public_corpus: tuple[DynamicCorpusEpisode, ...],
) -> str:
    matrix: list[dict[str, object]] = []
    for episode in public_corpus:
        entry: dict[str, object] = {
            "episode_id": episode.episode_id,
            "episode_content_hash": episode.content_hash,
            "split": episode.split.value,
            "expectation_category": episode.expectation_category.value,
            "seed": episode.seed,
            "scenario_family": "legacy_v1",
            "variant": None,
            "orientation": None,
            "semantic_world_hash": None,
            "oracle_hash": None,
        }
        if isinstance(episode, V6DynamicCorpusEpisode):
            entry.update(
                {
                    "scenario_family": episode.scenario_family.value,
                    "variant": episode.variant,
                    "orientation": episode.orientation.value,
                    "semantic_world_hash": episode.semantic_world_hash,
                    "oracle_hash": episode.oracle_hash,
                }
            )
        matrix.append(entry)
    return canonical_content_hash(
        {
            "schema_version": "dynamic_scenario_oracle_matrix_v6",
            "records": tuple(matrix),
        }
    )


def _full_public_evidence(
    *,
    public_records: list[dict[str, object]],
    contract_results: dict[str, object],
    public_functional: dict[str, object],
    qualification: dict[str, object],
) -> dict[str, object]:
    hard_safety = tuple(
        {
            "pair_id": record["pair_id"],
            "controller_name": record["controller_name"],
            "hard_safety": record["hard_safety"],
        }
        for record in public_records
    )
    functional = tuple(
        {
            "pair_id": record["pair_id"],
            "controller_name": record["controller_name"],
            "functional_qualified": record["functional_qualified"],
            "functional_failures": record["functional_failures"],
            "category_oracle_applied": record["category_oracle_applied"],
            "category_oracle_failures": record["category_oracle_failures"],
        }
        for record in public_records
    )
    return {
        "schema_version": "dynamic_public_evidence_v6",
        "contract_fault": contract_results,
        "hard_safety": hard_safety,
        "functional": {
            "summary": public_functional,
            "records": functional,
        },
        "qualification": qualification,
    }


def run_dynamic_public_qualification(
    output_directory: str | Path,
    config: DynamicPublicQualificationConfig,
) -> DynamicPublicQualificationResult:
    output = Path(output_directory)
    report_path = output / "public_qualification_report.json"
    _assert_public_output_unused(output)
    phase = _run_dynamic_public_phase(output, config)
    report = {
        "schema_version": _PUBLIC_REPORT_SCHEMA,
        "runner_version": DYNAMIC_RUNNER_VERSION,
        "passed": bool(phase.public_gate["passed"]),
        "report_only": phase.public_receipt is None,
        "source_freeze_hash_before": phase.source_freeze_hash_before,
        "source_freeze_hash_after": phase.source_freeze_hash_after,
        "source_freeze_hash_before_seal": phase.source_freeze_hash_before_seal,
        "source_freeze_hash_at_receipt_write": (
            phase.source_freeze_hash_at_receipt_write
        ),
        "source_freeze_consistent": phase.source_freeze_consistent,
        "public_corpus_hash": phase.public_validation.corpus_content_hash,
        "v6_public_corpus_hash": phase.v6_public_validation.corpus_content_hash,
        "public_run_count": len(phase.public_records),
        "simulation_workers": phase.simulation_workers,
        "run_scope": phase.run_scope,
        "public_record_set_hash": phase.public_record_set_hash,
        "scenario_oracle_matrix_hash": phase.scenario_oracle_matrix_hash,
        "full_evidence_hash": phase.full_evidence_hash,
        "gate": phase.public_gate,
        "receipt_content_hash": (
            phase.public_receipt["receipt_content_hash"]
            if phase.public_receipt is not None
            else None
        ),
    }
    report["report_content_hash"] = canonical_content_hash(report)
    _write_exclusive_json(report_path, report)
    receipt_path = (
        output / "public_qualification_receipt.json"
        if phase.public_receipt is not None
        else None
    )
    return DynamicPublicQualificationResult(
        output_directory=output,
        report_path=report_path,
        gate_path=output / "public_qualification_gate.json",
        paired_results_path=output / "paired_episode_results.json",
        receipt_path=receipt_path,
        passed=bool(phase.public_gate["passed"]),
        public_run_count=len(phase.public_records),
        simulation_worker_count=phase.simulation_workers,
    )


def _run_dynamic_public_phase(
    output: Path,
    config: DynamicPublicQualificationConfig,
) -> _DynamicPublicPhase:
    output.mkdir(parents=True, exist_ok=True)
    _configure_numeric_thread_environment()
    source_freeze_hash_before = _source_freeze_hash()
    run_scope = _public_run_scope(config)
    legacy_public_corpus = generate_dynamic_corpus(base_seed=config.base_seed)
    public_validation = validate_dynamic_corpus(legacy_public_corpus)
    if not public_validation.passed:
        raise ValueError(f"public corpus validation failed: {public_validation.failures}")
    v6_public_corpus = generate_dynamic_v6_public_corpus(base_seed=config.base_seed)
    v6_public_validation = validate_dynamic_v6_public_corpus(v6_public_corpus)
    if not v6_public_validation.passed:
        raise ValueError(
            f"v6 public corpus validation failed: {v6_public_validation.failures}"
        )
    public_corpus = legacy_public_corpus + v6_public_corpus
    selected_public = _limited(public_corpus, config.public_episode_limit)
    simulation_workers = _resolved_simulation_workers(
        config.simulation_workers,
        job_count=len(selected_public) * len(config.profiles),
    )
    contract_results = _contract_fault_qualification(
        config,
        workspace_basetemp_parent=output,
    )
    public_records = _run_corpus(
        selected_public,
        config=config,
        worker_count=simulation_workers,
    )
    public_record_coverage = _public_record_coverage(
        selected_public,
        profiles=config.profiles,
        records=public_records,
    )
    public_functional = _public_functional_qualification(
        public_records,
        public_corpus=public_corpus,
    )
    public_functional["record_coverage"] = public_record_coverage
    public_functional["passed"] = bool(
        public_functional["passed"] and public_record_coverage["passed"]
    )
    hard_safety_failures = _hard_failure_records(public_records)
    public_prequalification = {
        "contract_fault_passed": bool(contract_results["passed"]),
        "hard_safety_failures": hard_safety_failures,
        "record_coverage": public_record_coverage,
        "functional_qualification": public_functional,
    }
    public_prequalification["passed"] = bool(
        public_prequalification["contract_fault_passed"]
        and not hard_safety_failures
        and public_functional["passed"]
    )
    if public_prequalification["passed"]:
        try:
            qualification = _normalized_qualification_result(
                _run_wall_clock_qualification(public_corpus, config),
                config,
            )
        except Exception as error:  # qualification 오류는 seal 대신 구조화해 보존한다.
            qualification = _qualification_fallback(
                config,
                reason="wall_clock_qualification_failed",
                failure_detail=f"{type(error).__name__}: {error}",
            )
    else:
        qualification = _qualification_fallback(
            config,
            reason="public_contract_hard_safety_or_functional_failed",
        )
    timing_passed = _wall_clock_qualification_passed(qualification)

    source_freeze_hash_after = _source_freeze_hash()
    public_record_set_hash = _public_record_set_hash(public_records)
    scenario_oracle_matrix_hash = _scenario_oracle_matrix_hash(public_corpus)
    full_evidence = _full_public_evidence(
        public_records=public_records,
        contract_results=contract_results,
        public_functional=public_functional,
        qualification=qualification,
    )
    full_evidence_hash = canonical_content_hash(full_evidence)

    _write_json(output / "public_prequalification.json", public_prequalification)
    _write_json(output / "qualification_results.json", qualification)
    _write_json(output / "contract_fault_results.json", contract_results)
    _write_json(output / "hard_safety_results.json", hard_safety_failures)
    _write_json(output / "paired_episode_results.json", public_records)

    source_freeze_hash_before_seal = _source_freeze_hash()
    source_freeze_consistent = len(
        {
            source_freeze_hash_before,
            source_freeze_hash_after,
            source_freeze_hash_before_seal,
        }
    ) == 1
    evidence_passed = bool(public_prequalification["passed"] and timing_passed)
    public_gate_passed = bool(
        evidence_passed
        and run_scope["sealing_eligible"]
        and source_freeze_consistent
    )
    public_gate = {
        "schema_version": _PUBLIC_GATE_SCHEMA,
        "runner_version": DYNAMIC_RUNNER_VERSION,
        "passed": public_gate_passed,
        "evidence_passed": evidence_passed,
        "report_only": not public_gate_passed,
        "receipt_sealed": public_gate_passed,
        "sealing_eligible": bool(run_scope["sealing_eligible"]),
        "non_sealing_reasons": list(run_scope["non_sealing_reasons"]),
        "source_freeze_consistent": source_freeze_consistent,
        "contract_fault_passed": public_prequalification["contract_fault_passed"],
        "hard_safety_passed": not hard_safety_failures,
        "record_coverage_passed": bool(public_record_coverage["passed"]),
        "functional_passed": bool(public_functional["passed"]),
        "wall_clock_50ms_passed": timing_passed,
        "qualification_status": qualification["status"],
        "source_freeze_hash_at_receipt_write": None,
    }

    public_receipt: dict[str, object] | None = None
    source_freeze_hash_at_receipt_write: str | None = None
    if public_gate_passed:
        commit_hash, dirty = _git_state()
        controller_semantic_digest_set_hash = canonical_content_hash(
            tuple(
                (
                    record["pair_id"],
                    record["controller_name"],
                    record["controller_semantic_digest"],
                )
                for record in public_records
            )
        )
        parameter_hashes = {
            "pp": canonical_content_hash(_pp_parameters()),
            "dwa": canonical_content_hash(_dwa_parameters()),
            "vehicle": canonical_content_hash(VIRTUAL_DOLL_WHEELCHAIR_V0_1),
            "observation": canonical_content_hash(_observation_parameters()),
            "safety_gate": canonical_content_hash(_gate_parameters()),
        }
        source_freeze_hash_at_receipt_write = _source_freeze_hash()
        public_gate["source_freeze_hash_at_receipt_write"] = (
            source_freeze_hash_at_receipt_write
        )
        if source_freeze_hash_at_receipt_write != source_freeze_hash_before:
            source_freeze_consistent = False
            public_gate_passed = False
            public_gate.update(
                {
                    "passed": False,
                    "report_only": True,
                    "receipt_sealed": False,
                    "source_freeze_consistent": False,
                    "non_sealing_reasons": [
                        *public_gate["non_sealing_reasons"],
                        "source_changed_at_receipt_write",
                    ],
                }
            )
        else:
            public_receipt = {
                "schema_version": _PUBLIC_RECEIPT_SCHEMA,
                "source_freeze_hash": source_freeze_hash_before,
                "source_freeze_hash_after_run": source_freeze_hash_after,
                "source_freeze_hash_before_seal": source_freeze_hash_before_seal,
                "source_freeze_hash_at_receipt_write": (
                    source_freeze_hash_at_receipt_write
                ),
                "code_commit": commit_hash,
                "working_tree_dirty": dirty,
                "runner_version": DYNAMIC_RUNNER_VERSION,
                "run_scope_hash": canonical_content_hash(run_scope),
                "public_corpus_hash": public_validation.corpus_content_hash,
                "v6_public_corpus_hash": v6_public_validation.corpus_content_hash,
                "public_semantic_world_set_hash": (
                    v6_public_validation.semantic_world_set_hash
                ),
                "combined_public_corpus_hash": canonical_content_hash(public_corpus),
                "scenario_versions": {
                    "matrix": "dynamic_scenario_oracle_matrix_v6",
                    "legacy": DYNAMIC_CORPUS_GENERATOR_VERSION,
                    "v6": DYNAMIC_V6_CORPUS_GENERATOR_VERSION,
                    "oracle": DYNAMIC_V6_ORACLE_VERSION,
                },
                "numeric_tolerance_version": NUMERIC_TOLERANCE_VERSION,
                "parameter_hashes": parameter_hashes,
                "public_record_set_hash": public_record_set_hash,
                "controller_semantic_digest_set_hash": (
                    controller_semantic_digest_set_hash
                ),
                "scenario_oracle_matrix_hash": scenario_oracle_matrix_hash,
                "full_evidence_hash": full_evidence_hash,
                "contract_fault_result_hash": canonical_content_hash(contract_results),
                "hard_safety_evidence_hash": canonical_content_hash(
                    full_evidence["hard_safety"]
                ),
                "functional_result_hash": canonical_content_hash(public_functional),
                "qualification_result_hash": canonical_content_hash(qualification),
                "qualification_snapshot_set_hash": qualification.get(
                    "snapshot_set_hash"
                ),
                "qualification_snapshot_cases": qualification.get("snapshot_cases"),
                "qualification_execution": {
                    "execution_mode": qualification.get("execution_mode"),
                    "parallelized": qualification.get("parallelized"),
                    "active_worker_process_ids_before": qualification.get(
                        "active_worker_process_ids_before"
                    ),
                },
                "machine_identifier": qualification.get("machine_identifier"),
                "process_affinity": qualification.get("process_affinity"),
                "simulation_execution": {
                    "mode": "process_parallel_episode_profile_jobs",
                    "worker_count": simulation_workers,
                    "paired_unit": "same_episode_profile_seed_pp_then_dwa",
                    "result_order": "corpus_profile_controller",
                },
                "public_gate_hash": canonical_content_hash(public_gate),
            }
            public_receipt["receipt_content_hash"] = canonical_content_hash(public_receipt)

    _write_json(output / "public_qualification_gate.json", public_gate)
    if public_receipt is not None:
        source_freeze_hash_at_receipt_write = _source_freeze_hash()
        if source_freeze_hash_at_receipt_write != source_freeze_hash_before:
            source_freeze_consistent = False
            public_gate.update(
                {
                    "passed": False,
                    "report_only": True,
                    "receipt_sealed": False,
                    "source_freeze_consistent": False,
                    "source_freeze_hash_at_receipt_write": (
                        source_freeze_hash_at_receipt_write
                    ),
                    "non_sealing_reasons": [
                        *public_gate["non_sealing_reasons"],
                        "source_changed_at_receipt_write",
                    ],
                }
            )
            public_receipt = None
            _write_json(output / "public_qualification_gate.json", public_gate)
        else:
            _write_exclusive_json(
                output / "public_qualification_receipt.json",
                public_receipt,
            )
    return _DynamicPublicPhase(
        source_freeze_hash_before=source_freeze_hash_before,
        source_freeze_hash_after=source_freeze_hash_after,
        source_freeze_hash_before_seal=source_freeze_hash_before_seal,
        source_freeze_hash_at_receipt_write=source_freeze_hash_at_receipt_write,
        source_freeze_consistent=source_freeze_consistent,
        simulation_workers=simulation_workers,
        public_validation=public_validation,
        v6_public_validation=v6_public_validation,
        public_corpus=public_corpus,
        public_records=public_records,
        contract_results=contract_results,
        qualification=qualification,
        public_gate=public_gate,
        run_scope=run_scope,
        public_record_set_hash=public_record_set_hash,
        scenario_oracle_matrix_hash=scenario_oracle_matrix_hash,
        full_evidence_hash=full_evidence_hash,
        public_receipt=public_receipt,
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
        "population": "normal_progressable_paired_episode_ids",
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
    config: DynamicPublicQualificationConfig,
    worker_count: int,
) -> list[dict[str, object]]:
    jobs = tuple(
        _EpisodeProfileJob(
            order=order,
            episode=episode,
            profile_name=profile_name,
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
    pair_process_id = os.getpid()
    pair_id = canonical_content_hash(
        {
            "episode_content_hash": episode.content_hash,
            "profile": job.profile_name,
            "seed": episode.seed,
        }
    )
    observation_stream_hash = canonical_content_hash(
        generate_episode_observation_slots(episode, profile=profile)
    )
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
            oracle_spec=(
                episode.oracle_spec
                if isinstance(episode, V6DynamicCorpusEpisode)
                else None
            ),
        )
        records.append(
            _run_record(
                episode,
                job.profile_name,
                pipeline,
                evaluation,
                worker_elapsed_ns,
                pair_process_id=pair_process_id,
                pair_id=pair_id,
                observation_stream_hash=observation_stream_hash,
            )
        )
    return job.order, tuple(records)


def _rigid_metamorphic_signature(
    pipeline: DynamicControllerPipelineResult,
    evaluation: object,
) -> dict[str, object]:
    controller_command_trace = tuple(
        (
            step.tick_id,
            step.controller_result.status.value,
            _metamorphic_scalar(step.controller_result.requested_twist.linear),
            _metamorphic_scalar(step.controller_result.requested_twist.angular),
            step.controller_result.failure_reason,
            step.controller_result.controller_requested_stop,
            step.controller_result.no_safe_candidate,
        )
        for step in pipeline.steps
    )
    shared_gate_trace = tuple(
        (
            step.tick_id,
            _metamorphic_scalar(step.safety_decision.command.linear),
            _metamorphic_scalar(step.safety_decision.command.angular),
            step.safety_decision.motion_state.value,
            (
                step.safety_decision.primary_hold_reason.value
                if step.safety_decision.primary_hold_reason is not None
                else None
            ),
            step.safety_decision.resume_allowed,
            step.safety_decision.proposal_accepted,
            step.safety_decision.stop_epoch,
            tuple(step.safety_decision.failure_reasons),
            step.gate_overrode_controller,
        )
        for step in pipeline.steps
    )
    pipeline_result = {
        "status": pipeline.status.value,
        "completed": pipeline.completed,
        "expected_hold_reached": pipeline.expected_hold_reached,
        "tick_count": len(pipeline.steps),
        "failure_reason": pipeline.failure_reason,
    }
    hard_safety_result = asdict(evaluation.hard_safety)
    first_failure_time_s = hard_safety_result.get("first_failure_time_s")
    if isinstance(first_failure_time_s, float):
        hard_safety_result["first_failure_time_s"] = _metamorphic_scalar(
            first_failure_time_s
        )
    category_result = {
        "applied": evaluation.category_oracle_applied,
        "failures": tuple(evaluation.category_oracle_failures),
    }
    functional_result = {
        "qualified": evaluation.functional_qualified,
        "failures": tuple(evaluation.functional_failures),
    }
    signature = {
        "schema_version": _RIGID_SIGNATURE_SCHEMA,
        "numeric_tolerance_version": NUMERIC_TOLERANCE_VERSION,
        "controller_command_trace_hash": canonical_content_hash(
            controller_command_trace
        ),
        "shared_gate_trace_hash": canonical_content_hash(shared_gate_trace),
        "pipeline_result_hash": canonical_content_hash(pipeline_result),
        "hard_safety_result_hash": canonical_content_hash(hard_safety_result),
        "category_result_hash": canonical_content_hash(category_result),
        "functional_result_hash": canonical_content_hash(functional_result),
    }
    signature["content_hash"] = canonical_content_hash(signature)
    return signature


def _metamorphic_scalar(value: float) -> float:
    normalized = round(float(value), 12)
    return 0.0 if abs(normalized) <= 1e-12 else normalized


def _run_record(
    episode: DynamicCorpusEpisode,
    profile_name: str,
    pipeline: DynamicControllerPipelineResult,
    evaluation: object,
    worker_elapsed_ns: int,
    *,
    pair_process_id: int,
    pair_id: str,
    observation_stream_hash: str,
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
    controller_semantic_digest = canonical_content_hash(
        tuple(
            (
                step.controller_result.status,
                step.controller_result.requested_twist,
                step.controller_result.predicted_trajectory,
                step.controller_result.failure_reason,
                step.controller_result.decision_trace,
                step.controller_result.controller_requested_stop,
                step.controller_result.no_safe_candidate,
                step.safety_decision.command,
                step.safety_decision.motion_state,
                step.safety_decision.primary_hold_reason,
                step.safety_decision.resume_allowed,
                step.gate_overrode_controller,
                step.robot_state_after,
            )
            for step in steps
        )
    )
    post_controller_gate_events = tuple(
        {
            "tick_id": step.tick_id,
            "gate_overrode_controller": step.gate_overrode_controller,
            "proposal_accepted": step.safety_decision.proposal_accepted,
            "motion_state": step.safety_decision.motion_state.value,
            "primary_hold_reason": (
                step.safety_decision.primary_hold_reason.value
                if step.safety_decision.primary_hold_reason is not None
                else None
            ),
            "failure_reasons": list(step.safety_decision.failure_reasons),
            "controller_requested_stop": (
                step.controller_result.controller_requested_stop
            ),
            "controller_no_safe_candidate": step.controller_result.no_safe_candidate,
        }
        for step in steps
        if (
            step.gate_overrode_controller
            or step.safety_decision.primary_hold_reason is not None
            or step.safety_decision.failure_reasons
        )
    )
    hold_reason_counts: dict[str, int] = {}
    for event in post_controller_gate_events:
        reason = event["primary_hold_reason"]
        if reason is not None:
            hold_reason_counts[str(reason)] = hold_reason_counts.get(str(reason), 0) + 1
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
        "category_oracle_applied": evaluation.category_oracle_applied,
        "category_oracle_failures": list(evaluation.category_oracle_failures),
        "scenario": (
            {
                "family": episode.scenario_family.value,
                "variant": episode.variant,
                "orientation": episode.orientation.value,
                "latent_case_id": episode.latent_case_id,
                "semantic_world_hash": episode.semantic_world_hash,
                "oracle_hash": episode.oracle_hash,
            }
            if isinstance(episode, V6DynamicCorpusEpisode)
            else None
        ),
        "metrics": metrics,
        "pipeline": {
            "status": pipeline.status.value,
            "completed": pipeline.completed,
            "expected_hold_reached": pipeline.expected_hold_reached,
            "tick_count": len(steps),
            "failure_reason": pipeline.failure_reason,
        },
        "worker_elapsed_ns_nonqualification": worker_elapsed_ns,
        "worker_process_id_nonqualification": pair_process_id,
        "pair_id": pair_id,
        "observation_stream_hash": observation_stream_hash,
        "command_state_event_hash": deterministic_signature,
        "controller_semantic_digest": controller_semantic_digest,
        "rigid_metamorphic_signature": (
            _rigid_metamorphic_signature(pipeline, evaluation)
            if isinstance(episode, V6DynamicCorpusEpisode)
            else None
        ),
        "post_controller_gate_diagnostics": {
            "schema_version": "dynamic_post_controller_gate_diagnostics_v6",
            "stage": "POST_CONTROLLER_GATE",
            "event_count": len(post_controller_gate_events),
            "override_count": sum(
                bool(event["gate_overrode_controller"])
                for event in post_controller_gate_events
            ),
            "hold_reason_counts": dict(sorted(hold_reason_counts.items())),
            "events": list(post_controller_gate_events),
        },
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


def _empty_controller_qualification() -> dict[str, object]:
    return {
        "passed": False,
        "samples": 0,
        "cold_samples": 0,
        "cold_maximum_ns": None,
        "p50_ns": None,
        "p95_ns": None,
        "p99_ns": None,
        "maximum_ns": None,
        "deadline_ns": _CONTROLLER_DEADLINE_NS,
        "deadline_miss_count": None,
        "peak_memory_bytes": None,
    }


def _qualification_fallback(
    config: DynamicPublicQualificationConfig,
    *,
    reason: str,
    failure_detail: str | None = None,
    status: str | None = None,
) -> dict[str, object]:
    return {
        "schema_version": _QUALIFICATION_SCHEMA,
        "status": status or ("not_run" if failure_detail is None else "failed"),
        "passed": False,
        "not_run_reason": reason,
        "failure_detail": failure_detail,
        "machine_identifier": _machine_identifier(),
        "parent_process_id": os.getpid(),
        "process_affinity": list(_process_affinity()),
        "active_worker_process_ids_before": [],
        "execution_mode": "serial_parent_not_run_fail_closed",
        "parallelized": False,
        "numeric_thread_environment": _numeric_thread_environment(),
        "snapshot_cases": [],
        "snapshot_set_hash": None,
        "warmups_per_snapshot": config.qualification_warmups,
        "repeats_per_snapshot": config.qualification_repeats,
        "controllers": {
            "dynamic_pure_pursuit": _empty_controller_qualification(),
            "dynamic_dwa": _empty_controller_qualification(),
        },
    }


def _normalized_controller_qualification(value: object) -> dict[str, object]:
    if not isinstance(value, dict):
        return _empty_controller_qualification()
    result = _empty_controller_qualification()
    for key in result:
        if key in value:
            result[key] = value[key]
    required_integer_fields = (
        "samples",
        "cold_samples",
        "cold_maximum_ns",
        "p50_ns",
        "p95_ns",
        "p99_ns",
        "maximum_ns",
        "deadline_ns",
        "deadline_miss_count",
        "peak_memory_bytes",
    )
    structurally_complete = all(
        isinstance(result[field], int) and not isinstance(result[field], bool)
        for field in required_integer_fields
    )
    passed = bool(
        structurally_complete
        and int(result["samples"]) > 0
        and int(result["cold_samples"]) > 0
        and int(result["deadline_miss_count"]) == 0
        and int(result["maximum_ns"]) <= int(result["deadline_ns"])
    )
    result["passed"] = passed
    return result


def _normalized_qualification_result(
    value: object,
    config: DynamicPublicQualificationConfig,
) -> dict[str, object]:
    if not isinstance(value, dict):
        return _qualification_fallback(
            config,
            reason="wall_clock_qualification_invalid_schema",
        )
    controllers = value.get("controllers")
    controllers = controllers if isinstance(controllers, dict) else {}
    normalized_controllers = {
        name: _normalized_controller_qualification(controllers.get(name))
        for name in ("dynamic_pure_pursuit", "dynamic_dwa")
    }
    qualification_passed = all(
        record["passed"] for record in normalized_controllers.values()
    )
    snapshot_cases = value.get("snapshot_cases")
    process_affinity = value.get("process_affinity")
    numeric_environment = value.get("numeric_thread_environment")
    metadata_complete = bool(
        value.get("schema_version") == _QUALIFICATION_SCHEMA
        and value.get("status") == "completed"
        and value.get("passed") is qualification_passed
        and value.get("not_run_reason") is None
        and value.get("failure_detail") is None
        and isinstance(value.get("machine_identifier"), str)
        and bool(value.get("machine_identifier"))
        and isinstance(value.get("parent_process_id"), int)
        and not isinstance(value.get("parent_process_id"), bool)
        and int(value["parent_process_id"]) > 0
        and isinstance(process_affinity, (list, tuple))
        and bool(process_affinity)
        and all(
            isinstance(cpu, int) and not isinstance(cpu, bool) and cpu >= 0
            for cpu in process_affinity
        )
        and value.get("active_worker_process_ids_before") == []
        and value.get("execution_mode")
        == "serial_parent_after_simulation_worker_pool_shutdown"
        and value.get("parallelized") is False
        and numeric_environment
        == {
            "OMP_NUM_THREADS": "1",
            "OPENBLAS_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
            "NUMEXPR_NUM_THREADS": "1",
        }
        and isinstance(snapshot_cases, list)
        and bool(snapshot_cases)
        and all(
            isinstance(case, dict)
            and isinstance(case.get("case_id"), str)
            and bool(case.get("case_id"))
            and isinstance(case.get("input_content_hash"), str)
            and bool(case.get("input_content_hash"))
            and isinstance(case.get("snapshot_content_hash"), str)
            and bool(case.get("snapshot_content_hash"))
            for case in snapshot_cases
        )
        and isinstance(value.get("snapshot_set_hash"), str)
        and bool(value.get("snapshot_set_hash"))
        and value.get("warmups_per_snapshot") == config.qualification_warmups
        and value.get("repeats_per_snapshot") == config.qualification_repeats
        and set(controllers) == {"dynamic_pure_pursuit", "dynamic_dwa"}
        and all(
            isinstance(controllers[name], dict)
            and controllers[name].get("passed")
            is normalized_controllers[name]["passed"]
            and normalized_controllers[name]["samples"]
            == len(snapshot_cases) * config.qualification_repeats
            and normalized_controllers[name]["cold_samples"] == len(snapshot_cases)
            and normalized_controllers[name]["deadline_ns"] == _CONTROLLER_DEADLINE_NS
            for name in normalized_controllers
        )
    )
    if not metadata_complete:
        return _qualification_fallback(
            config,
            reason="wall_clock_qualification_invalid_schema",
            failure_detail="qualification evidence failed v6 structural validation",
            status="invalid_evidence",
        )
    passed = bool(
        qualification_passed
        and value.get("parallelized") is False
        and value.get("active_worker_process_ids_before") == []
    )
    return {
        "schema_version": _QUALIFICATION_SCHEMA,
        "status": "completed",
        "passed": passed,
        "not_run_reason": None,
        "failure_detail": None,
        "machine_identifier": value.get("machine_identifier") or _machine_identifier(),
        "parent_process_id": value.get("parent_process_id", os.getpid()),
        "process_affinity": list(value.get("process_affinity") or _process_affinity()),
        "active_worker_process_ids_before": list(
            value.get("active_worker_process_ids_before") or []
        ),
        "execution_mode": value.get(
            "execution_mode",
            "serial_parent_after_simulation_worker_pool_shutdown",
        ),
        "parallelized": bool(value.get("parallelized", False)),
        "numeric_thread_environment": value.get(
            "numeric_thread_environment",
            _numeric_thread_environment(),
        ),
        "snapshot_cases": list(value.get("snapshot_cases") or []),
        "snapshot_set_hash": value.get("snapshot_set_hash"),
        "warmups_per_snapshot": config.qualification_warmups,
        "repeats_per_snapshot": config.qualification_repeats,
        "controllers": normalized_controllers,
    }


def _run_wall_clock_qualification(
    public_corpus: tuple[DynamicCorpusEpisode, ...],
    config: DynamicPublicQualificationConfig,
) -> dict[str, object]:
    cases = _qualification_snapshot_cases(public_corpus)
    snapshots = tuple(snapshot for _, snapshot, _ in cases)
    children_before = tuple(child.pid for child in active_children())
    if children_before:
        raise RuntimeError("wall-clock qualification requires every worker to be stopped")
    records: dict[str, object] = {}
    for controller_type in (DynamicPurePursuitController, DynamicDwaController):
        cold_elapsed: list[int] = []
        for snapshot in snapshots:
            controller = controller_type()
            started = perf_counter_ns()
            controller.step(snapshot)
            cold_elapsed.append(perf_counter_ns() - started)
        controller = controller_type()
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
        deadline_miss_count = sum(value > _CONTROLLER_DEADLINE_NS for value in elapsed)
        maximum_ns = max(elapsed)
        records[controller.name] = {
            "passed": deadline_miss_count == 0 and maximum_ns <= _CONTROLLER_DEADLINE_NS,
            "samples": len(elapsed),
            "cold_samples": len(cold_elapsed),
            "cold_maximum_ns": max(cold_elapsed),
            "p50_ns": int(np.percentile(array, 50)),
            "p95_ns": int(np.percentile(array, 95)),
            "p99_ns": int(np.percentile(array, 99)),
            "maximum_ns": maximum_ns,
            "deadline_ns": _CONTROLLER_DEADLINE_NS,
            "deadline_miss_count": deadline_miss_count,
            "peak_memory_bytes": int(peak),
        }
    passed = all(bool(record["passed"]) for record in records.values())
    return {
        "schema_version": _QUALIFICATION_SCHEMA,
        "status": "completed",
        "passed": passed,
        "not_run_reason": None,
        "failure_detail": None,
        "machine_identifier": _machine_identifier(),
        "parent_process_id": os.getpid(),
        "process_affinity": _process_affinity(),
        "active_worker_process_ids_before": list(children_before),
        "execution_mode": "serial_parent_after_simulation_worker_pool_shutdown",
        "parallelized": False,
        "numeric_thread_environment": _numeric_thread_environment(),
        "snapshot_cases": [metadata for _, _, metadata in cases],
        "snapshot_set_hash": canonical_content_hash(
            tuple(
                (case_id, _controller_snapshot_semantic_hash(snapshot), metadata)
                for case_id, snapshot, metadata in cases
            )
        ),
        "warmups_per_snapshot": config.qualification_warmups,
        "repeats_per_snapshot": config.qualification_repeats,
        "controllers": records,
    }


def _qualification_snapshot_cases(
    public_corpus: tuple[DynamicCorpusEpisode, ...],
) -> tuple[tuple[str, ControllerSnapshot, dict[str, object]], ...]:
    legacy_empty = next(
        episode
        for episode in public_corpus
        if type(episode) is DynamicCorpusEpisode
        and episode.split is DynamicCorpusSplit.GOLDEN
        and episode.expectation_category
        is DynamicExpectationCategory.OBSERVATION_INVALID
    )
    legacy_actor = next(
        episode
        for episode in public_corpus
        if type(episode) is DynamicCorpusEpisode
        and episode.split is DynamicCorpusSplit.GOLDEN
        and episode.expectation_category is DynamicExpectationCategory.WAIT_AND_RESUME
    )
    simultaneous = next(
        episode
        for episode in public_corpus
        if isinstance(episode, V6DynamicCorpusEpisode)
        and episode.scenario_family is DynamicScenarioFamily.MULTI_ACTOR
        and episode.variant == "simultaneous-overlap"
    )
    corner = next(
        episode
        for episode in public_corpus
        if isinstance(episode, V6DynamicCorpusEpisode)
        and episode.scenario_family is DynamicScenarioFamily.CORNER_INTERSECTION
        and episode.variant == "left-turn-static-topology"
    )
    stress_corner = next(
        episode
        for episode in public_corpus
        if isinstance(episode, V6DynamicCorpusEpisode)
        and episode.scenario_family is DynamicScenarioFamily.CORNER_INTERSECTION
        and episode.variant == "second-risk-after-corner"
    )
    requested = (
        ("actor-0-free", legacy_empty, NORMAL_OBSERVATION_PROFILE, 0, False, False, 1),
        ("actor-1-active", legacy_actor, NORMAL_OBSERVATION_PROFILE, 1, False, False, 1),
        ("actor-2-active", simultaneous, NORMAL_OBSERVATION_PROFILE, 2, False, False, 1),
        ("corner-static-forbidden", corner, STRESS_OBSERVATION_PROFILE, 1, True, True, 2),
        (
            "staggered-risk-multisegment",
            stress_corner,
            STRESS_OBSERVATION_PROFILE,
            1,
            True,
            True,
            2,
        ),
    )
    return tuple(
        _qualification_snapshot_case(
            case_id,
            episode,
            profile,
            actor_count=actor_count,
            require_static=require_static,
            require_forbidden=require_forbidden,
            minimum_path_segments=minimum_path_segments,
        )
        for (
            case_id,
            episode,
            profile,
            actor_count,
            require_static,
            require_forbidden,
            minimum_path_segments,
        ) in requested
    )


def _qualification_snapshot_case(
    case_id: str,
    episode: DynamicCorpusEpisode,
    profile: DynamicObservationProfile,
    *,
    actor_count: int,
    require_static: bool,
    require_forbidden: bool,
    minimum_path_segments: int,
) -> tuple[str, ControllerSnapshot, dict[str, object]]:
    context_factory = _EpisodeContextFactory(episode, profile)
    gate = DynamicSafetyGate()
    for tick_id in range(episode.tick_count):
        simulation_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        context = context_factory(
            tick_id,
            simulation_time_s,
            episode.initial_state,
            gate,
        )
        prediction = context.prediction_set
        if prediction is None or len(prediction.tubes) != actor_count:
            continue
        has_static = bool(np.any(context.grid_snapshot.grid.occupancy))
        has_forbidden = bool(context.grid_snapshot.forbidden_cells)
        if require_static and not has_static:
            continue
        if require_forbidden and not has_forbidden:
            continue
        if len(episode.reference_path) - 1 < minimum_path_segments:
            continue
        snapshot = build_controller_snapshot(
            tick_id=tick_id,
            simulation_time_s=simulation_time_s,
            mission_id=episode.mission_id,
            robot_state=episode.initial_state,
            goal_pose=episode.goal_pose,
            reference_path=episode.reference_path,
            static_grid_snapshot=context.grid_snapshot,
            validated_observation=context.observation_snapshot,
            actor_tubes=prediction,
            vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        )
        metadata = {
            "case_id": case_id,
            "episode_id": episode.episode_id,
            "profile": profile.name.value,
            "tick_id": tick_id,
            "actor_tube_count": actor_count,
            "has_static_occupancy": has_static,
            "has_forbidden_cells": has_forbidden,
            "reference_path_segment_count": len(episode.reference_path) - 1,
            "input_content_hash": snapshot.input_content_hash,
            "snapshot_content_hash": _controller_snapshot_semantic_hash(snapshot),
        }
        return case_id, snapshot, metadata
    raise ValueError(f"qualification case did not satisfy its frozen contract: {case_id}")


def _controller_snapshot_semantic_hash(snapshot: ControllerSnapshot) -> str:
    return canonical_content_hash(
        {
            "tick_id": snapshot.tick_id,
            "simulation_time_s": snapshot.simulation_time_s,
            "mission_id": snapshot.mission_id,
            "robot_state": snapshot.robot_state,
            "goal_pose": snapshot.goal_pose,
            "reference_path": snapshot.reference_path,
            "grid_content_hash": snapshot.static_grid_snapshot.metadata.content_hash,
            "forbidden_cells": tuple(sorted(snapshot.static_grid_snapshot.forbidden_cells)),
            "validated_observation": snapshot.validated_observation,
            "actor_tubes": snapshot.actor_tubes,
            "vehicle_profile": snapshot.vehicle_profile,
            "map_id": snapshot.map_id,
            "map_revision": snapshot.map_revision,
            "mission_revision": snapshot.mission_revision,
            "observation_revision": snapshot.observation_revision,
            "input_content_hash": snapshot.input_content_hash,
        }
    )


def _contract_fault_qualification(
    config: DynamicPublicQualificationConfig,
    *,
    workspace_basetemp_parent: Path,
) -> dict[str, object]:
    test_files = (
        "tests/test_dynamic_observation.py",
        "tests/test_dynamic_authority.py",
        "tests/test_dynamic_timing.py",
        "tests/test_dynamic_contract_faults.py",
    )
    evidence_hash = _files_hash(tuple(LAB_ROOT / path for path in test_files))
    if config.contract_test_evidence is None:
        workspace_basetemp_parent.mkdir(parents=True, exist_ok=True)
        with TemporaryDirectory(
            prefix=".contract-pytest-",
            dir=workspace_basetemp_parent,
        ) as workspace_basetemp:
            completed = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pytest",
                    "-c",
                    str(LAB_ROOT / "pyproject.toml"),
                    *(str(LAB_ROOT / path) for path in test_files),
                    "--basetemp",
                    workspace_basetemp,
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


def _assert_public_output_unused(output: Path) -> None:
    existing = tuple(name for name in _PUBLIC_PHASE_ARTIFACTS if (output / name).exists())
    if existing:
        raise FileExistsError(
            "dynamic public qualification output already contains artifacts: "
            + ", ".join(existing)
        )


def _profile(name: str) -> DynamicObservationProfile:
    return (
        NORMAL_OBSERVATION_PROFILE
        if name == "normal"
        else STRESS_OBSERVATION_PROFILE
    )


def _resolved_simulation_workers(
    requested: int | None,
    *,
    job_count: int | None = None,
) -> int:
    if requested is not None:
        return requested
    usable_processors = len(_process_affinity())
    default_workers = max(1, (usable_processors + 1) // 2)
    return min(default_workers, job_count) if job_count is not None else default_workers


def _process_affinity() -> tuple[int, ...]:
    get_affinity = getattr(os, "sched_getaffinity", None)
    if get_affinity is not None:
        return tuple(sorted(get_affinity(0)))
    if sys.platform == "win32":
        try:
            from ctypes import byref, c_size_t, windll

            process_mask = c_size_t()
            system_mask = c_size_t()
            if windll.kernel32.GetProcessAffinityMask(
                windll.kernel32.GetCurrentProcess(),
                byref(process_mask),
                byref(system_mask),
            ):
                return tuple(
                    index
                    for index in range(process_mask.value.bit_length())
                    if process_mask.value & (1 << index)
                )
        except (AttributeError, OSError):
            pass
    return tuple(range(os.cpu_count() or 1))


def _configure_numeric_thread_environment() -> None:
    for variable in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ[variable] = "1"


def _numeric_thread_environment() -> dict[str, str | None]:
    return {
        variable: os.environ.get(variable)
        for variable in (
            "OMP_NUM_THREADS",
            "OPENBLAS_NUM_THREADS",
            "MKL_NUM_THREADS",
            "NUMEXPR_NUM_THREADS",
        )
    }


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


def _observation_parameters() -> dict[str, object]:
    return {
        "version": DYNAMIC_OBSERVATION_GENERATOR_VERSION,
        "normal": NORMAL_OBSERVATION_PROFILE,
        "stress": STRESS_OBSERVATION_PROFILE,
    }


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
