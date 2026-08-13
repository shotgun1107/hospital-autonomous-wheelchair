"""Public-only R2 witness audit, reporting, and non-overwriting artifacts.

The search boundary consumes only :class:`WitnessWorldSnapshot`.  Evaluator
expectations are attached after every search and are excluded from the search
semantic hash.  Python wall-clock values are operational diagnostics only.
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
from collections.abc import Callable
from contextlib import suppress
from dataclasses import asdict, dataclass, is_dataclass, replace
from enum import StrEnum
from hashlib import sha256
from math import hypot
from pathlib import Path
from time import perf_counter_ns

import numpy as np

from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusEpisode,
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
    generate_episode_observation_slots,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DIRECTIONAL_PREDICTION_VERSION,
    FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS,
)
from hospital_path_lab.dynamic_observation import (
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationAvailability,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    dynamic_observation_content_hash,
)
from hospital_path_lab.dynamic_witness_contracts import (
    FROZEN_WITNESS_SEARCH_CONFIG,
    WITNESS_VALIDATOR_VERSION,
    AutomatedWitness,
    PassStructuredSearchResult,
    WitnessKind,
    WitnessSearchConfig,
    WitnessSearchResult,
    WitnessSearchStatus,
    WitnessWorldSnapshot,
    project_public_witness_world,
)
from hospital_path_lab.dynamic_witness_crossing import search_crossing_bypass
from hospital_path_lab.dynamic_witness_pass import search_pass_structured_parallel
from hospital_path_lab.dynamic_witness_profile_replay import (
    WITNESS_PROFILE_REPLAY_VERSION,
    WitnessProfileReplayBundle,
    replay_witness_profiles,
)
from hospital_path_lab.dynamic_witness_search import (
    generate_hold_only_witness,
    generate_wait_and_follow_witness,
    search_wait_and_hold,
)
from hospital_path_lab.dynamic_witness_restop import (
    search_multi_hazard_restop,
    validate_multi_hazard_restop,
)
from hospital_path_lab.dynamic_witness_validation import (
    GroundTruthWitnessValidation,
    validate_ground_truth_witness,
)
from hospital_path_lab.map_factory import canonical_content_hash

WITNESS_PUBLIC_AUDIT_SCHEMA_VERSION = "dynamic-witness-public-audit-v1"
WITNESS_PUBLIC_AUDIT_VERSION = "dynamic-witness-public-audit-r2a-supplement-v1"
WITNESS_AUDIT_MANIFEST_SCHEMA_VERSION = "dynamic-witness-audit-manifest-v1"
WITNESS_AUDIT_OUTPUT_SCHEMA_VERSION = "dynamic-witness-audit-output-v1"
_PUBLIC_LANES = ("v6_primary", "legacy_mechanism")
_PASS_KINDS = frozenset((WitnessKind.PASS_LEFT, WitnessKind.PASS_RIGHT))
_CROSSING_KINDS = frozenset(
    (WitnessKind.CROSSING_BYPASS_LEFT, WitnessKind.CROSSING_BYPASS_RIGHT)
)
_FEASIBLE_KINDS = _PASS_KINDS | _CROSSING_KINDS
_SHA256_LENGTH = 64
_TIME_TOLERANCE_S = 1e-12


class WitnessEvidenceClass(StrEnum):
    FEASIBLE = "feasible"
    WAIT_ONLY = "wait_only"
    FORBIDDEN = "forbidden"
    NO_SAFE_SOLUTION = "no_safe_solution"
    OBSERVATION_UNDECIDABLE = "observation_undecidable"
    SEARCH_INCONCLUSIVE = "search_inconclusive"


class ExpectationAssessment(StrEnum):
    MATCHED = "matched"
    MISMATCHED = "mismatched"
    NOT_FULLY_COVERED = "not_fully_covered"


@dataclass(frozen=True, slots=True)
class WitnessAuditRecord:
    roles: tuple[str, ...]
    witness: AutomatedWitness
    validation: GroundTruthWitnessValidation
    profile_replay: WitnessProfileReplayBundle

    def __post_init__(self) -> None:
        roles = tuple(sorted(set(self.roles)))
        if not roles or any(not role for role in roles):
            raise ValueError("witness audit record requires stable roles")
        if not self.validation.passed:
            raise ValueError("witness audit record requires a passing validation")
        if self.validation.witness_content_hash != self.witness.semantic_content_hash:
            raise ValueError("witness audit validation is not bound to the witness")
        if self.profile_replay.witness_content_hash != self.witness.semantic_content_hash:
            raise ValueError("profile replay is not bound to the witness")
        object.__setattr__(self, "roles", roles)

    @property
    def semantic_content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class ObservationFaultReplay:
    fault_name: str
    invalid_until_s: float
    injected_invalid_frame_count: int
    observed_invalid_snapshot_count: int
    first_recovered_fresh_time_s: float | None
    hold_witness_covers_fault_interval: bool
    recovery_grants_motion_authority: bool
    passed: bool
    content_hash: str

    def __post_init__(self) -> None:
        if self.fault_name != "source_invalid_then_recovers":
            raise ValueError("unsupported public observation fault replay")
        if self.invalid_until_s != 2.0:
            raise ValueError("public observation fault recovery boundary must remain 2.0 s")
        if self.injected_invalid_frame_count <= 0:
            raise ValueError("fault replay must inject at least one invalid frame")
        if self.observed_invalid_snapshot_count != self.injected_invalid_frame_count:
            raise ValueError("every injected invalid frame must produce an invalid snapshot")
        if self.first_recovered_fresh_time_s is None:
            raise ValueError("fault replay must observe a fresh recovery frame")
        if self.recovery_grants_motion_authority:
            raise ValueError("observation recovery must never grant motion authority")
        _require_sha256("observation fault replay hash", self.content_hash)


@dataclass(frozen=True, slots=True)
class WitnessEpisodeAudit:
    schema_version: str
    audit_version: str
    public_id: str
    corpus_lane: str
    corpus_ordinal: int
    source_episode_id: str
    source_episode_content_hash: str
    expected_category: DynamicExpectationCategory
    world: WitnessWorldSnapshot
    wait_hold_search: WitnessSearchResult
    pass_search: PassStructuredSearchResult
    witness_records: tuple[WitnessAuditRecord, ...]
    observation_fault_replay: ObservationFaultReplay | None
    evidence_classes: tuple[WitnessEvidenceClass, ...]
    expectation_assessment: ExpectationAssessment
    assessment_reasons: tuple[str, ...]
    hard_failures: tuple[str, ...]
    limitations: tuple[str, ...]
    search_semantic_hash: str
    report_content_hash: str
    elapsed_nonqualification_ns: int

    def __post_init__(self) -> None:
        if self.schema_version != WITNESS_PUBLIC_AUDIT_SCHEMA_VERSION:
            raise ValueError("unsupported witness episode audit schema")
        if self.audit_version != WITNESS_PUBLIC_AUDIT_VERSION:
            raise ValueError("unsupported witness episode audit version")
        if self.corpus_lane not in _PUBLIC_LANES:
            raise ValueError("unsupported public witness corpus lane")
        if self.corpus_ordinal < 0 or self.elapsed_nonqualification_ns < 0:
            raise ValueError("episode ordinal and elapsed time must not be negative")
        if self.world.content_hash != self.wait_hold_search.world_content_hash:
            raise ValueError("WAIT/HOLD result does not match the projected world")
        if self.world.content_hash != self.pass_search.world_content_hash:
            raise ValueError("PASS result does not match the projected world")
        if len({record.witness.semantic_content_hash for record in self.witness_records}) != len(
            self.witness_records
        ):
            raise ValueError("episode witness records must be unique")
        _require_sha256("source episode content hash", self.source_episode_content_hash)
        _require_sha256("episode search semantic hash", self.search_semantic_hash)
        _require_sha256("episode report content hash", self.report_content_hash)
        object.__setattr__(
            self,
            "evidence_classes",
            tuple(sorted(set(self.evidence_classes), key=lambda item: item.value)),
        )
        object.__setattr__(self, "assessment_reasons", tuple(sorted(set(self.assessment_reasons))))
        object.__setattr__(self, "hard_failures", tuple(sorted(set(self.hard_failures))))
        object.__setattr__(self, "limitations", tuple(sorted(set(self.limitations))))


@dataclass(frozen=True, slots=True)
class WitnessPublicAudit:
    schema_version: str
    audit_version: str
    simulation_only: bool
    hidden_used: bool
    r1_audit_content_hash: str
    v6_public_corpus_content_hash: str
    legacy_golden_corpus_content_hash: str
    search_config_hash: str
    episode_results: tuple[WitnessEpisodeAudit, ...]
    hard_failures: tuple[str, ...]
    limitations: tuple[str, ...]
    semantic_content_hash: str
    report_content_hash: str
    elapsed_nonqualification_ns: int

    def __post_init__(self) -> None:
        if self.schema_version != WITNESS_PUBLIC_AUDIT_SCHEMA_VERSION:
            raise ValueError("unsupported witness public audit schema")
        if self.audit_version != WITNESS_PUBLIC_AUDIT_VERSION:
            raise ValueError("unsupported witness public audit version")
        if not self.simulation_only or self.hidden_used:
            raise ValueError("R2 audit must remain public and simulation-only")
        for name, value in (
            ("R1 audit content hash", self.r1_audit_content_hash),
            ("v6 corpus content hash", self.v6_public_corpus_content_hash),
            ("legacy corpus content hash", self.legacy_golden_corpus_content_hash),
            ("search config hash", self.search_config_hash),
            ("audit semantic content hash", self.semantic_content_hash),
            ("audit report content hash", self.report_content_hash),
        ):
            _require_sha256(name, value)
        if self.elapsed_nonqualification_ns < 0:
            raise ValueError("audit elapsed time must not be negative")
        if len({item.public_id for item in self.episode_results}) != len(self.episode_results):
            raise ValueError("public audit episode identities must be unique")
        object.__setattr__(self, "hard_failures", tuple(sorted(set(self.hard_failures))))
        object.__setattr__(self, "limitations", tuple(sorted(set(self.limitations))))

    @property
    def hard_passed(self) -> bool:
        return not self.hard_failures

    @property
    def r2_completion_qualified(self) -> bool:
        v6_count = sum(item.corpus_lane == "v6_primary" for item in self.episode_results)
        legacy_count = sum(item.corpus_lane == "legacy_mechanism" for item in self.episode_results)
        return (
            self.hard_passed
            and len(self.episode_results) == 19
            and v6_count == 13
            and legacy_count == 6
            and all(
                item.expectation_assessment is ExpectationAssessment.MATCHED
                for item in self.episode_results
            )
        )


@dataclass(frozen=True, slots=True)
class WitnessAuditManifest:
    schema_version: str
    output_schema_version: str
    audit_version: str
    code_commit: str
    code_tree_hash: str
    source_freeze_hash: str
    git_dirty: bool
    r1_audit_content_hash: str
    v6_public_corpus_content_hash: str
    legacy_golden_corpus_content_hash: str
    search_config_hash: str
    validator_version: str
    replay_version: str
    prediction_version: str
    prediction_parameter_hash: str
    observation_profile_hash: str
    vehicle_profile_hash: str
    episode_order: tuple[tuple[str, str], ...]
    process_workers: int
    candidate_shard_size: int
    host_identifier: str
    python_version: str
    hidden_used: bool
    semantic_content_hash: str
    content_hash: str

    def __post_init__(self) -> None:
        if self.schema_version != WITNESS_AUDIT_MANIFEST_SCHEMA_VERSION:
            raise ValueError("unsupported witness audit manifest schema")
        if self.output_schema_version != WITNESS_AUDIT_OUTPUT_SCHEMA_VERSION:
            raise ValueError("unsupported witness audit output schema")
        if self.hidden_used:
            raise ValueError("witness audit manifest must never include hidden input")
        if self.process_workers <= 0 or self.candidate_shard_size <= 0:
            raise ValueError("operational process settings must be positive")
        _require_git_object_id("code commit", self.code_commit)
        _require_git_object_id("code tree hash", self.code_tree_hash)
        for name, value in (
            ("source freeze hash", self.source_freeze_hash),
            ("R1 audit content hash", self.r1_audit_content_hash),
            ("v6 corpus hash", self.v6_public_corpus_content_hash),
            ("legacy corpus hash", self.legacy_golden_corpus_content_hash),
            ("search config hash", self.search_config_hash),
            ("prediction parameter hash", self.prediction_parameter_hash),
            ("observation profile hash", self.observation_profile_hash),
            ("vehicle profile hash", self.vehicle_profile_hash),
            ("manifest semantic hash", self.semantic_content_hash),
            ("manifest content hash", self.content_hash),
        ):
            _require_sha256(name, value)


PassSearch = Callable[[WitnessWorldSnapshot], PassStructuredSearchResult]
EpisodeProgress = Callable[[WitnessEpisodeAudit], None]


def public_witness_audit_episodes() -> tuple[tuple[str, DynamicCorpusEpisode], ...]:
    """Return the frozen v6 13 + legacy GOLDEN 6 public scope."""

    v6 = tuple(("v6_primary", episode) for episode in generate_dynamic_v6_public_corpus())
    legacy = tuple(
        ("legacy_mechanism", episode)
        for episode in generate_dynamic_corpus()
        if episode.split is DynamicCorpusSplit.GOLDEN
    )
    if len(v6) != 13 or len(legacy) != 6:
        raise RuntimeError("R2 public scope must remain v6 13 + legacy GOLDEN 6")
    return (*v6, *legacy)


def audit_public_witness_episode(
    episode: DynamicCorpusEpisode,
    *,
    corpus_lane: str,
    corpus_ordinal: int,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
    pass_search: PassSearch | None = None,
) -> WitnessEpisodeAudit:
    """Run one public episode without passing evaluator metadata into search."""

    started_ns = perf_counter_ns()
    if episode.split not in (DynamicCorpusSplit.GOLDEN, DynamicCorpusSplit.DEVELOPMENT):
        raise ValueError("witness public audit rejects hidden or unsupported split")
    if corpus_lane not in _PUBLIC_LANES:
        raise ValueError("unsupported witness public audit lane")
    if corpus_ordinal < 0:
        raise ValueError("corpus ordinal must not be negative")

    world = project_public_witness_world(episode, search_config=search_config)
    wait_result = search_wait_and_hold(world, search_config=search_config)
    execute_pass = pass_search or (
        lambda item: search_pass_structured_parallel(
            item,
            search_config=search_config,
        )
    )
    pass_result = execute_pass(world)
    crossing_result = search_crossing_bypass(world, search_config=search_config)
    restop_result = search_multi_hazard_restop(world, search_config=search_config)

    roles_by_hash: dict[str, set[str]] = {}
    witnesses_by_hash: dict[str, AutomatedWitness] = {}

    def add_witness(witness: AutomatedWitness | None, role: str) -> None:
        if witness is None:
            return
        key = witness.semantic_content_hash
        witnesses_by_hash[key] = witness
        roles_by_hash.setdefault(key, set()).add(role)

    add_witness(wait_result.selected_witness, "wait_hold_search_selected")
    add_witness(pass_result.best_pass_left, "pass_left_search_selected")
    add_witness(pass_result.best_pass_right, "pass_right_search_selected")
    add_witness(crossing_result.left.selected_witness, "crossing_left_search_selected")
    add_witness(crossing_result.right.selected_witness, "crossing_right_search_selected")
    add_witness(restop_result.witness, "multi_hazard_restop_selected")
    add_witness(
        _find_multi_hazard_wait_diagnostic(world, search_config),
        "multi_hazard_wait_diagnostic",
    )
    with suppress(ValueError):
        add_witness(generate_hold_only_witness(world), "full_episode_hold_diagnostic")

    records: list[WitnessAuditRecord] = []
    hard_failures: list[str] = []
    limitations: set[str] = {
        "offline_ground_truth_search_only",
        "online_controller_gate_and_authority_not_evaluated",
        "python_wall_clock_is_operational_only",
        "simulation_only_open_loop_circular_actor",
    }
    for key in sorted(witnesses_by_hash):
        witness = witnesses_by_hash[key]
        validation = validate_ground_truth_witness(
            world,
            witness,
            strict_declarations=witness.kind in _FEASIBLE_KINDS,
        )
        if not validation.passed:
            hard_failures.extend(
                f"selected_witness_validation_failed:{failure}" for failure in validation.failures
            )
            continue
        expected_validation_hash = _selected_validation_hash(
            wait_result,
            pass_result,
            crossing_result,
            restop_result,
            witness,
        )
        if (
            expected_validation_hash is not None
            and expected_validation_hash != validation.content_hash
        ):
            hard_failures.append("selected_validation_hash_mismatch")
        replay = replay_witness_profiles(world, witness, validation)
        ideal = replay.results[0]
        hard_failures.extend(f"ideal_profile:{failure}" for failure in ideal.hard_failures)
        limitations.update(replay.limitations)
        records.append(
            WitnessAuditRecord(
                roles=tuple(roles_by_hash[key]),
                witness=witness,
                validation=validation,
                profile_replay=replay,
            )
        )

    observation_fault_replay = _replay_observation_fault(
        episode,
        tuple(records),
    )

    if wait_result.status is WitnessSearchStatus.INVALID_INPUT:
        hard_failures.append("wait_hold_search_invalid_input")
    if any(
        side.status is WitnessSearchStatus.INVALID_INPUT
        for side in (pass_result.left, pass_result.right)
    ):
        hard_failures.append("pass_search_invalid_input")
    if any(
        side.status is WitnessSearchStatus.INVALID_INPUT
        for side in (crossing_result.left, crossing_result.right)
    ):
        hard_failures.append("crossing_search_invalid_input")
    limitations.update(pass_result.limitations)

    evidence = _evidence_classes(
        world,
        wait_result,
        pass_result,
        tuple(records),
        observation_fault_replay,
    )
    assessment, assessment_reasons = _assess_expectation(
        episode.expectation_category,
        world,
        tuple(records),
        evidence,
        observation_fault_replay,
    )
    if (
        episode.expectation_category is not DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
        and WitnessEvidenceClass.FEASIBLE in evidence
    ):
        hard_failures.append("unexpected_pass_for_expected_category")
    if assessment is not ExpectationAssessment.MATCHED:
        limitations.add("evaluator_expectation_not_fully_reproduced")
    if observation_fault_replay is not None:
        limitations.add("observation_fault_replay_is_evaluator_only_after_search")
    if corpus_lane == "legacy_mechanism":
        limitations.add("legacy_mechanism_not_v6_primary_evidence")

    public_id = f"{corpus_lane}-{corpus_ordinal:02d}-{world.content_hash[:12]}"
    search_payload = {
        "world_content_hash": world.content_hash,
        "source_projection_hash": world.source_projection_hash,
        "wait_hold_search_hash": wait_result.semantic_content_hash,
        "pass_search_hash": pass_result.semantic_content_hash,
        "crossing_search_hash": crossing_result.semantic_content_hash,
        "restop_search_hash": restop_result.content_hash,
        "witness_record_hashes": tuple(record.semantic_content_hash for record in records),
        "evidence_classes": evidence,
    }
    search_hash = canonical_content_hash(search_payload)
    source_episode_hash = canonical_content_hash(episode)
    report_payload = {
        "public_id": public_id,
        "corpus_lane": corpus_lane,
        "corpus_ordinal": corpus_ordinal,
        "source_episode_id": episode.episode_id,
        "source_episode_content_hash": source_episode_hash,
        "expected_category": episode.expectation_category,
        "search_semantic_hash": search_hash,
        "expectation_assessment": assessment,
        "assessment_reasons": assessment_reasons,
        "observation_fault_replay": observation_fault_replay,
        "hard_failures": tuple(sorted(set(hard_failures))),
        "limitations": tuple(sorted(limitations)),
    }
    return WitnessEpisodeAudit(
        schema_version=WITNESS_PUBLIC_AUDIT_SCHEMA_VERSION,
        audit_version=WITNESS_PUBLIC_AUDIT_VERSION,
        public_id=public_id,
        corpus_lane=corpus_lane,
        corpus_ordinal=corpus_ordinal,
        source_episode_id=episode.episode_id,
        source_episode_content_hash=source_episode_hash,
        expected_category=episode.expectation_category,
        world=world,
        wait_hold_search=wait_result,
        pass_search=pass_result,
        witness_records=tuple(records),
        observation_fault_replay=observation_fault_replay,
        evidence_classes=evidence,
        expectation_assessment=assessment,
        assessment_reasons=assessment_reasons,
        hard_failures=tuple(hard_failures),
        limitations=tuple(limitations),
        search_semantic_hash=search_hash,
        report_content_hash=canonical_content_hash(report_payload),
        elapsed_nonqualification_ns=perf_counter_ns() - started_ns,
    )


def audit_public_witness_corpus(
    *,
    r1_audit_content_hash: str,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
    max_workers: int = 14,
    shard_size: int = 2_048,
    on_episode: EpisodeProgress | None = None,
) -> WitnessPublicAudit:
    """Run the exact public 13+6 scope in deterministic corpus order."""

    _require_sha256("R1 audit content hash", r1_audit_content_hash)
    started_ns = perf_counter_ns()
    scope = public_witness_audit_episodes()
    results: list[WitnessEpisodeAudit] = []
    for ordinal, (lane, episode) in enumerate(scope):
        result = audit_public_witness_episode(
            episode,
            corpus_lane=lane,
            corpus_ordinal=ordinal,
            search_config=search_config,
            pass_search=lambda world: search_pass_structured_parallel(
                world,
                search_config=search_config,
                max_workers=max_workers,
                shard_size=shard_size,
            ),
        )
        results.append(result)
        if on_episode is not None:
            on_episode(result)

    v6 = tuple(episode for lane, episode in scope if lane == "v6_primary")
    legacy = tuple(episode for lane, episode in scope if lane == "legacy_mechanism")
    hard_failures = tuple(
        f"{item.public_id}:{failure}" for item in results for failure in item.hard_failures
    )
    limitations = {
        "r2_structured_templates_are_not_general_pose_space_complete",
        "legacy_mechanism_not_v6_primary_evidence",
        "python_wall_clock_is_operational_only",
        "online_controller_gate_and_authority_not_evaluated",
        "actual_person_and_product_safety_not_evaluated",
    }
    limitations.update(reason for item in results for reason in item.limitations)
    semantic_payload = {
        "schema_version": WITNESS_PUBLIC_AUDIT_SCHEMA_VERSION,
        "audit_version": WITNESS_PUBLIC_AUDIT_VERSION,
        "simulation_only": True,
        "hidden_used": False,
        "r1_audit_content_hash": r1_audit_content_hash,
        "v6_public_corpus_content_hash": canonical_content_hash(v6),
        "legacy_golden_corpus_content_hash": canonical_content_hash(legacy),
        "search_config_hash": search_config.content_hash,
        "episode_search_hashes": tuple(item.search_semantic_hash for item in results),
    }
    semantic_hash = canonical_content_hash(semantic_payload)
    report_payload = {
        **semantic_payload,
        "episode_report_hashes": tuple(item.report_content_hash for item in results),
        "hard_failures": hard_failures,
        "limitations": tuple(sorted(limitations)),
    }
    return WitnessPublicAudit(
        schema_version=WITNESS_PUBLIC_AUDIT_SCHEMA_VERSION,
        audit_version=WITNESS_PUBLIC_AUDIT_VERSION,
        simulation_only=True,
        hidden_used=False,
        r1_audit_content_hash=r1_audit_content_hash,
        v6_public_corpus_content_hash=canonical_content_hash(v6),
        legacy_golden_corpus_content_hash=canonical_content_hash(legacy),
        search_config_hash=search_config.content_hash,
        episode_results=tuple(results),
        hard_failures=hard_failures,
        limitations=tuple(limitations),
        semantic_content_hash=semantic_hash,
        report_content_hash=canonical_content_hash(report_payload),
        elapsed_nonqualification_ns=perf_counter_ns() - started_ns,
    )


def build_witness_audit_manifest(
    *,
    repository_root: Path,
    r1_audit_content_hash: str,
    max_workers: int,
    shard_size: int,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
) -> WitnessAuditManifest:
    """Build a source-bound manifest; worker settings remain operational only."""

    _require_sha256("R1 audit content hash", r1_audit_content_hash)
    repository_root = Path(repository_root).resolve()
    head = _git_output(repository_root, "rev-parse", "HEAD")
    tree = _git_output(repository_root, "rev-parse", "HEAD^{tree}")
    dirty = bool(_git_output(repository_root, "status", "--porcelain"))
    scope = public_witness_audit_episodes()
    worlds = tuple(project_public_witness_world(episode) for _, episode in scope)
    v6 = tuple(episode for lane, episode in scope if lane == "v6_primary")
    legacy = tuple(episode for lane, episode in scope if lane == "legacy_mechanism")
    profiles = (
        FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
        NORMAL_OBSERVATION_PROFILE,
        STRESS_OBSERVATION_PROFILE,
    )
    semantic_payload = {
        "schema_version": WITNESS_AUDIT_MANIFEST_SCHEMA_VERSION,
        "output_schema_version": WITNESS_AUDIT_OUTPUT_SCHEMA_VERSION,
        "audit_version": WITNESS_PUBLIC_AUDIT_VERSION,
        "code_commit": head,
        "code_tree_hash": tree,
        "source_freeze_hash": _source_freeze_hash(repository_root),
        "git_dirty": dirty,
        "r1_audit_content_hash": r1_audit_content_hash,
        "v6_public_corpus_content_hash": canonical_content_hash(v6),
        "legacy_golden_corpus_content_hash": canonical_content_hash(legacy),
        "search_config_hash": search_config.content_hash,
        "validator_version": WITNESS_VALIDATOR_VERSION,
        "replay_version": WITNESS_PROFILE_REPLAY_VERSION,
        "prediction_version": DIRECTIONAL_PREDICTION_VERSION,
        "prediction_parameter_hash": canonical_content_hash(
            FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS
        ),
        "observation_profile_hash": canonical_content_hash(profiles),
        "vehicle_profile_hash": worlds[0].vehicle_profile_hash,
        "episode_order": tuple(
            (f"{lane}-{index:02d}-{world.content_hash[:12]}", world.content_hash)
            for index, ((lane, _episode), world) in enumerate(zip(scope, worlds, strict=True))
        ),
        "hidden_used": False,
    }
    semantic_hash = canonical_content_hash(semantic_payload)
    operational = {
        "process_workers": max_workers,
        "candidate_shard_size": shard_size,
        "host_identifier": canonical_content_hash(
            {
                "platform": platform.platform(),
                "machine": platform.machine(),
                "processor": platform.processor(),
            }
        ),
        "python_version": platform.python_version(),
    }
    draft = WitnessAuditManifest(
        **semantic_payload,
        **operational,
        semantic_content_hash=semantic_hash,
        content_hash="0" * _SHA256_LENGTH,
    )
    return replace(
        draft,
        content_hash=canonical_content_hash(asdict(draft) | {"content_hash": None}),
    )


class WitnessAuditOutputWriter:
    """Write immutable episode evidence and an explicit partial lifecycle."""

    def __init__(self, output_dir: Path, manifest: WitnessAuditManifest) -> None:
        self.output_dir = Path(output_dir)
        self.manifest = manifest
        self._completed_ids: list[str] = []
        self._started = False

    def start(self) -> None:
        if self.output_dir.exists():
            raise FileExistsError(f"witness audit output already exists: {self.output_dir}")
        self.output_dir.mkdir(parents=True)
        _write_exclusive_json(self.output_dir / "witness_search_manifest.json", self.manifest)
        self._write_state(partial=True)
        self._started = True

    def write_episode(self, result: WitnessEpisodeAudit) -> None:
        if not self._started:
            raise RuntimeError("witness audit output writer has not started")
        if result.public_id in self._completed_ids:
            raise FileExistsError(f"episode output already written: {result.public_id}")
        episode_dir = self.output_dir / "episodes" / result.public_id
        episode_dir.mkdir(parents=True, exist_ok=False)
        _write_exclusive_json(
            episode_dir / "search_diagnostics.json",
            {
                "schema_version": WITNESS_AUDIT_OUTPUT_SCHEMA_VERSION,
                "public_id": result.public_id,
                "world": result.world,
                "wait_hold_search": result.wait_hold_search,
                "pass_search": result.pass_search,
                "observation_fault_replay": result.observation_fault_replay,
                "search_semantic_hash": result.search_semantic_hash,
                "elapsed_nonqualification_ns": result.elapsed_nonqualification_ns,
            },
        )
        _write_exclusive_json(
            episode_dir / "selected_witness.json",
            {
                "schema_version": WITNESS_AUDIT_OUTPUT_SCHEMA_VERSION,
                "public_id": result.public_id,
                "records": tuple(
                    {"roles": record.roles, "witness": record.witness}
                    for record in result.witness_records
                ),
            },
        )
        _write_exclusive_json(
            episode_dir / "ground_truth_validation.json",
            {
                "schema_version": WITNESS_AUDIT_OUTPUT_SCHEMA_VERSION,
                "public_id": result.public_id,
                "records": tuple(
                    {
                        "witness_content_hash": record.witness.semantic_content_hash,
                        "validation": record.validation,
                    }
                    for record in result.witness_records
                ),
            },
        )
        _write_exclusive_json(
            episode_dir / "profile_replay.json",
            {
                "schema_version": WITNESS_AUDIT_OUTPUT_SCHEMA_VERSION,
                "public_id": result.public_id,
                "records": tuple(
                    {
                        "witness_content_hash": record.witness.semantic_content_hash,
                        "profile_replay": record.profile_replay,
                    }
                    for record in result.witness_records
                ),
            },
        )
        _save_episode_plot(result, episode_dir / "trajectory.png")
        self._completed_ids.append(result.public_id)
        self._write_state(partial=True)

    def complete(self, audit: WitnessPublicAudit) -> tuple[Path, Path, Path]:
        if not self._started:
            raise RuntimeError("witness audit output writer has not started")
        expected = tuple(item.public_id for item in audit.episode_results)
        if tuple(self._completed_ids) != expected:
            raise RuntimeError("cannot finalize incomplete witness audit output")
        results_path = self.output_dir / "witness_search_results.json"
        summary_path = self.output_dir / "summary.md"
        receipt_path = self.output_dir / "witness_audit_completion.json"
        _write_exclusive_json(results_path, audit)
        summary_path.write_text(_audit_summary(audit), encoding="utf-8", newline="\n")
        incomplete = self.output_dir / "run_state.incomplete.json"
        self._write_state(partial=False)
        complete = self.output_dir / "run_state.complete.json"
        incomplete.rename(complete)
        _write_exclusive_json(
            receipt_path,
            {
                "schema_version": WITNESS_AUDIT_OUTPUT_SCHEMA_VERSION,
                "partial": False,
                "manifest_content_hash": self.manifest.content_hash,
                "audit_semantic_content_hash": audit.semantic_content_hash,
                "audit_report_content_hash": audit.report_content_hash,
                "hard_passed": audit.hard_passed,
                "r2_completion_qualified": audit.r2_completion_qualified,
                "episode_count": len(audit.episode_results),
                "hidden_used": False,
            },
        )
        return results_path, summary_path, receipt_path

    def _write_state(self, *, partial: bool) -> None:
        path = self.output_dir / "run_state.incomplete.json"
        value = {
            "schema_version": WITNESS_AUDIT_OUTPUT_SCHEMA_VERSION,
            "partial": partial,
            "completed_public_ids": tuple(self._completed_ids),
            "expected_public_ids": tuple(public_id for public_id, _ in self.manifest.episode_order),
            "final_evidence_eligible": not partial
            and tuple(self._completed_ids)
            == tuple(public_id for public_id, _ in self.manifest.episode_order),
        }
        _write_atomic_json(path, value)


def _selected_validation_hash(
    wait_result: WitnessSearchResult,
    pass_result: PassStructuredSearchResult,
    crossing_result,
    restop_result,
    witness: AutomatedWitness,
) -> str | None:
    if (
        wait_result.selected_witness is not None
        and wait_result.selected_witness.semantic_content_hash == witness.semantic_content_hash
    ):
        return wait_result.selected_validation_hash
    for side in (pass_result.left, pass_result.right):
        if (
            side.best_witness is not None
            and side.best_witness.semantic_content_hash == witness.semantic_content_hash
        ):
            return side.selected_validation_hash
    for side in (crossing_result.left, crossing_result.right):
        if (
            side.selected_witness is not None
            and side.selected_witness.semantic_content_hash
            == witness.semantic_content_hash
        ):
            return side.selected_validation_hash
    if (
        restop_result.witness is not None
        and restop_result.validation is not None
        and restop_result.witness.semantic_content_hash == witness.semantic_content_hash
    ):
        return restop_result.validation.base_validation.content_hash
    return None


def _evidence_classes(
    world: WitnessWorldSnapshot,
    wait_result: WitnessSearchResult,
    pass_result: PassStructuredSearchResult,
    records: tuple[WitnessAuditRecord, ...],
    observation_fault_replay: ObservationFaultReplay | None,
) -> tuple[WitnessEvidenceClass, ...]:
    classes: set[WitnessEvidenceClass] = set()
    pass_found = any(record.witness.kind in _FEASIBLE_KINDS for record in records)
    wait_found = any(record.witness.kind is WitnessKind.WAIT_AND_FOLLOW for record in records)
    hold_found = any(record.witness.kind is WitnessKind.HOLD_ONLY for record in records)
    narrow_proof = _straight_corridor_pass_forbidden(world)
    if pass_found:
        classes.add(WitnessEvidenceClass.FEASIBLE)
    if narrow_proof:
        classes.add(WitnessEvidenceClass.FORBIDDEN)
        if wait_found:
            classes.add(WitnessEvidenceClass.WAIT_ONLY)
    if _analytic_no_safe_solution(world, wait_found=wait_found, hold_found=hold_found):
        classes.add(WitnessEvidenceClass.NO_SAFE_SOLUTION)
    if (observation_fault_replay is not None and observation_fault_replay.passed) or any(
        not profile.observation_decidable
        for record in records
        for profile in record.profile_replay.results
    ):
        classes.add(WitnessEvidenceClass.OBSERVATION_UNDECIDABLE)
    pass_inconclusive = any(
        side.status
        in (
            WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE,
            WitnessSearchStatus.RESOURCE_LIMIT,
        )
        for side in (pass_result.left, pass_result.right)
    )
    wait_inconclusive = wait_result.status in (
        WitnessSearchStatus.NO_WITNESS_IN_STRUCTURED_TEMPLATE,
        WitnessSearchStatus.RESOURCE_LIMIT,
    )
    if (pass_inconclusive and not narrow_proof and not pass_found) or (
        wait_inconclusive and not hold_found
    ):
        classes.add(WitnessEvidenceClass.SEARCH_INCONCLUSIVE)
    return tuple(sorted(classes, key=lambda item: item.value))


def _assess_expectation(
    expectation: DynamicExpectationCategory,
    world: WitnessWorldSnapshot,
    records: tuple[WitnessAuditRecord, ...],
    evidence: tuple[WitnessEvidenceClass, ...],
    observation_fault_replay: ObservationFaultReplay | None,
) -> tuple[ExpectationAssessment, tuple[str, ...]]:
    evidence_set = set(evidence)
    wait_records = tuple(
        record for record in records if record.witness.kind is WitnessKind.WAIT_AND_FOLLOW
    )
    hold_found = any(record.witness.kind is WitnessKind.HOLD_ONLY for record in records)
    if expectation is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE:
        matched = WitnessEvidenceClass.FEASIBLE in evidence_set
        return (
            ExpectationAssessment.MATCHED if matched else ExpectationAssessment.MISMATCHED,
            () if matched else ("expected_feasible_pass_not_found",),
        )
    if expectation is DynamicExpectationCategory.LOCAL_DETOUR_FORBIDDEN:
        matched = (
            WitnessEvidenceClass.FORBIDDEN in evidence_set
            and WitnessEvidenceClass.FEASIBLE not in evidence_set
            and bool(wait_records)
        )
        return (
            ExpectationAssessment.MATCHED if matched else ExpectationAssessment.MISMATCHED,
            () if matched else ("forbidden_wait_only_mechanism_not_reproduced",),
        )
    if expectation is DynamicExpectationCategory.WAIT_AND_RESUME:
        matched = bool(wait_records)
        return (
            ExpectationAssessment.MATCHED if matched else ExpectationAssessment.MISMATCHED,
            () if matched else ("wait_and_follow_witness_not_found",),
        )
    if expectation is DynamicExpectationCategory.NO_SAFE_SOLUTION:
        matched = WitnessEvidenceClass.NO_SAFE_SOLUTION in evidence_set
        return (
            ExpectationAssessment.MATCHED if matched else ExpectationAssessment.MISMATCHED,
            () if matched else ("analytic_no_safe_solution_not_reproduced",),
        )
    if expectation is DynamicExpectationCategory.OBSERVATION_INVALID:
        undecidable = WitnessEvidenceClass.OBSERVATION_UNDECIDABLE in evidence_set
        if (
            undecidable
            and hold_found
            and observation_fault_replay is not None
            and observation_fault_replay.passed
        ):
            return (
                ExpectationAssessment.MATCHED,
                (),
            )
        return (
            ExpectationAssessment.MISMATCHED,
            ("observation_undecidable_hold_not_reproduced",),
        )
    if expectation is DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP:
        matched = any(
            validate_multi_hazard_restop(world, record.witness).core_passed
            for record in wait_records
        )
        return (
            ExpectationAssessment.MATCHED if matched else ExpectationAssessment.NOT_FULLY_COVERED,
            () if matched else ("two_distinct_hazard_restops_not_demonstrated",),
        )
    raise AssertionError(f"unsupported evaluator expectation: {expectation}")


def _replay_observation_fault(
    episode: DynamicCorpusEpisode,
    records: tuple[WitnessAuditRecord, ...],
) -> ObservationFaultReplay | None:
    if episode.observation_fault is None:
        return None
    if episode.observation_fault != "source_invalid_then_recovers":
        raise ValueError("unsupported public observation fault")
    slots = generate_episode_observation_slots(
        episode,
        profile=FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    )
    first_frame = next((slot.frame for slot in slots if slot.frame is not None), None)
    if first_frame is None:
        raise ValueError("observation fault replay requires at least one frame")
    source = DynamicObservationSourceIdentity(
        stream_id=first_frame.stream_id,
        episode_id=first_frame.episode_id,
        episode_seed=first_frame.episode_seed,
        map_id=first_frame.map_id,
        map_revision=first_frame.map_revision,
    )
    validator = DynamicObservationValidator(
        source,
        FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    )
    injected = 0
    invalid = 0
    recovered_at: float | None = None
    for slot in slots:
        if slot.frame is None:
            validator.record_no_frame(
                sequence=slot.sequence,
                delivery_time_s=slot.scheduled_delivery_at_s,
            )
        else:
            frame = slot.frame
            if slot.scheduled_delivery_at_s < 2.0:
                frame = replace(
                    frame,
                    stream_id="fault-invalid-stream",
                    content_hash="pending",
                )
                frame = replace(frame, content_hash=dynamic_observation_content_hash(frame))
                injected += 1
            validator.accept(frame, received_at_s=slot.scheduled_delivery_at_s)
        snapshot = validator.snapshot(control_time_s=slot.scheduled_delivery_at_s)
        if slot.scheduled_delivery_at_s < 2.0:
            invalid += snapshot.availability is DynamicObservationAvailability.INVALID
        elif recovered_at is None and snapshot.availability is DynamicObservationAvailability.FRESH:
            recovered_at = slot.scheduled_delivery_at_s
            break

    hold_record = next(
        (record for record in records if record.witness.kind is WitnessKind.HOLD_ONLY),
        None,
    )
    hold_covers = bool(
        hold_record is not None
        and hold_record.witness.points[0].time_s <= _TIME_TOLERANCE_S
        and hold_record.witness.points[-1].time_s >= 2.0 - _TIME_TOLERANCE_S
        and all(
            abs(point.twist.linear) <= _TIME_TOLERANCE_S
            and abs(point.twist.angular) <= _TIME_TOLERANCE_S
            for point in hold_record.witness.points
            if point.time_s <= 2.0 + _TIME_TOLERANCE_S
        )
    )
    passed = bool(injected > 0 and invalid == injected and recovered_at is not None and hold_covers)
    payload = {
        "fault_name": episode.observation_fault,
        "invalid_until_s": 2.0,
        "injected_invalid_frame_count": injected,
        "observed_invalid_snapshot_count": invalid,
        "first_recovered_fresh_time_s": recovered_at,
        "hold_witness_covers_fault_interval": hold_covers,
        "recovery_grants_motion_authority": False,
        "passed": passed,
    }
    return ObservationFaultReplay(**payload, content_hash=canonical_content_hash(payload))


def _straight_corridor_pass_forbidden(world: WitnessWorldSnapshot) -> bool:
    if not world.actors or len(world.reference_path) != 2:
        return False
    start, end = world.reference_path
    dx = end.x - start.x
    dy = end.y - start.y
    if abs(dx) > _TIME_TOLERANCE_S and abs(dy) <= _TIME_TOLERANCE_S:
        available_width = world.grid.height * world.grid.resolution_m
    elif abs(dy) > _TIME_TOLERANCE_S and abs(dx) <= _TIME_TOLERANCE_S:
        available_width = world.grid.width * world.grid.resolution_m
    else:
        return False
    largest_radius = max(actor.radius_m for actor in world.actors)
    vehicle = world.kinematic_contract.vehicle_profile
    required_width = (
        vehicle.collision_width_m + 2.0 * largest_radius + 3.0 * vehicle.minimum_clearance_m
    )
    return available_width + 1e-12 < required_width


def _analytic_no_safe_solution(
    world: WitnessWorldSnapshot,
    *,
    wait_found: bool,
    hold_found: bool,
) -> bool:
    if wait_found or not hold_found or not _straight_corridor_pass_forbidden(world):
        return False
    if len(world.reference_path) != 2 or len(world.actors) != 1:
        return False
    actor = world.actors[0]
    if (
        actor.active_from_s > _TIME_TOLERANCE_S
        or actor.active_until_s < world.duration_s - _TIME_TOLERANCE_S
        or actor.velocity.magnitude > _TIME_TOLERANCE_S
    ):
        return False
    start, end = world.reference_path
    return (
        _point_segment_distance(
            actor.start_position.x,
            actor.start_position.y,
            start.x,
            start.y,
            end.x,
            end.y,
        )
        <= actor.radius_m + world.kinematic_contract.vehicle_profile.collision_width_m / 2.0
    )


def _hazard_restop_count(world: WitnessWorldSnapshot, witness: AutomatedWitness) -> int:
    points = witness.points
    count = 0
    for actor in world.actors:
        active_points = tuple(
            point
            for point in points
            if actor.active_from_s - _TIME_TOLERANCE_S
            <= point.time_s
            <= actor.active_until_s + _TIME_TOLERANCE_S
        )
        for point in active_points:
            stopped_now = abs(point.twist.linear) <= 1e-12 and abs(point.twist.angular) <= 1e-12
            moved_before = actor.active_from_s <= _TIME_TOLERANCE_S or any(
                abs(previous.twist.linear) > 1e-12 or abs(previous.twist.angular) > 1e-12
                for previous in points
                if previous.time_s < point.time_s - _TIME_TOLERANCE_S
            )
            resumes_after = any(
                abs(point.twist.linear) > 1e-12 or abs(point.twist.angular) > 1e-12
                for point in points
                if point.time_s > actor.active_until_s + _TIME_TOLERANCE_S
            )
            if moved_before and stopped_now and resumes_after:
                count += 1
                break
    return count


def _find_multi_hazard_wait_diagnostic(
    world: WitnessWorldSnapshot,
    search_config: WitnessSearchConfig,
) -> AutomatedWitness | None:
    """Preserve a valid multi-hazard WAIT witness without evaluator labels.

    The primary WAIT search optimizes completion and may legitimately choose a
    schedule that passes before a later hazard.  R2 also needs a mechanism
    witness showing both distinct holds when such a frozen candidate exists.
    This diagnostic enumerates only the same public event ticks and speed axis.
    """

    if len(world.actors) < 2:
        return None
    period_s = world.kinematic_contract.control_period_s
    maximum_tick = round(world.duration_s / period_s)
    departure_ticks = {0, min(maximum_tick, round(0.50 / period_s))}
    for actor in world.actors:
        for event_time_s in (actor.active_from_s, actor.active_until_s):
            event_tick = round(event_time_s / period_s)
            departure_ticks.add(max(0, min(maximum_tick, event_tick)))
            departure_ticks.add(max(0, min(maximum_tick, event_tick + 1)))

    candidates: dict[str, tuple[AutomatedWitness, GroundTruthWitnessValidation]] = {}
    for departure_tick in sorted(departure_ticks):
        for speed in search_config.linear_targets_mps:
            witness = generate_wait_and_follow_witness(
                world,
                departure_tick=departure_tick,
                linear_target_mps=speed,
            )
            if witness is None or witness.semantic_content_hash in candidates:
                continue
            validation = validate_ground_truth_witness(world, witness)
            if validation.passed and _hazard_restop_count(world, witness) >= 2:
                candidates[witness.semantic_content_hash] = (witness, validation)
    if not candidates:
        return None
    return min(
        candidates.values(),
        key=lambda item: (
            item[0].points[-1].time_s,
            item[1].metrics.actual_path_length_m,
            item[0].semantic_content_hash,
        ),
    )[0]


def _point_segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    dx = bx - ax
    dy = by - ay
    length_squared = dx * dx + dy * dy
    if length_squared <= 0.0:
        return hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_squared))
    return hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def _save_episode_plot(result: WitnessEpisodeAudit, output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    world = result.world
    figure, axis = plt.subplots(figsize=(9.5, 6.0))
    try:
        rgba = np.zeros((world.grid.height, world.grid.width, 4), dtype=float)
        for x, y in world.grid.occupied_cells:
            rgba[y, x] = (0.20, 0.20, 0.20, 0.75)
        for x, y in world.grid.forbidden_cells:
            rgba[y, x] = (0.85, 0.15, 0.15, 0.45)
        extent = (
            world.grid.origin_x_m,
            world.grid.origin_x_m + world.grid.width * world.grid.resolution_m,
            world.grid.origin_y_m,
            world.grid.origin_y_m + world.grid.height * world.grid.resolution_m,
        )
        axis.imshow(rgba, origin="lower", extent=extent, interpolation="nearest")
        axis.plot(
            [pose.x for pose in world.reference_path],
            [pose.y for pose in world.reference_path],
            linestyle="--",
            color="tab:blue",
            linewidth=2.0,
            label="reference",
        )
        actor_colors = ("tab:orange", "tab:green", "tab:purple", "tab:brown")
        sample_count = round(world.duration_s / world.kinematic_contract.control_period_s)
        for index, actor in enumerate(world.actors):
            samples = tuple(
                actor.state_at(tick * world.kinematic_contract.control_period_s)
                for tick in range(sample_count + 1)
            )
            active = tuple(sample for sample in samples if sample is not None)
            if not active:
                continue
            axis.plot(
                [sample.position.x for sample in active],
                [sample.position.y for sample in active],
                color=actor_colors[index % len(actor_colors)],
                linewidth=1.8,
                label=f"actor-{index}",
            )
        witness_colors = ("black", "tab:red", "tab:cyan", "tab:pink")
        for index, record in enumerate(result.witness_records):
            points = record.witness.points
            axis.plot(
                [point.pose.x for point in points],
                [point.pose.y for point in points],
                color=witness_colors[index % len(witness_colors)],
                linewidth=1.5,
                alpha=0.85,
                label="/".join(record.roles),
            )
            for event_name, event_time, marker in (
                ("depart", record.witness.departure_time_s, "o"),
                (
                    "pass",
                    min((time_s for _, time_s in record.witness.pass_times_by_actor), default=None),
                    "X",
                ),
                ("rejoin", record.witness.rejoin_confirmed_at_s, "s"),
            ):
                if event_time is None:
                    continue
                point = min(points, key=lambda item: abs(item.time_s - event_time))
                axis.scatter(point.pose.x, point.pose.y, marker=marker, s=45, label=event_name)
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.18)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_title(
            f"R2 public witness audit\n{result.public_id} | "
            f"expected={result.expected_category.value} | "
            f"assessment={result.expectation_assessment.value}"
        )
        handles, labels = axis.get_legend_handles_labels()
        unique = dict(zip(labels, handles, strict=False))
        axis.legend(unique.values(), unique.keys(), fontsize=7, loc="best")
        figure.tight_layout()
        figure.savefig(output_path, dpi=160, format="png")
    finally:
        plt.close(figure)


def _audit_summary(audit: WitnessPublicAudit) -> str:
    lines = [
        "# R2 공개 Witness 감사 결과",
        "",
        f"- hard 판정: `{'PASS' if audit.hard_passed else 'FAIL'}`",
        f"- R2 완료 자격: `{'충족' if audit.r2_completion_qualified else '미충족'}`",
        f"- 공개 범위: `{len(audit.episode_results)}`개 (`v6 13 + legacy golden 6`)",
        f"- hidden 사용: `{str(audit.hidden_used).lower()}`",
        f"- semantic hash: `{audit.semantic_content_hash}`",
        "- Python wall-clock은 운영 진단일 뿐 witness·taxonomy·안전 판정에 사용하지 않았다.",
        "",
        "| public id | lane | 기대 | evidence | 평가 | hard failures |",
        "|---|---|---|---|---|---:|",
    ]
    for item in audit.episode_results:
        evidence = ", ".join(value.value for value in item.evidence_classes) or "none"
        lines.append(
            f"| `{item.public_id}` | {item.corpus_lane} | "
            f"{item.expected_category.value} | {evidence} | "
            f"{item.expectation_assessment.value} | {len(item.hard_failures)} |"
        )
    lines.extend(
        (
            "",
            "## 해석 제한",
            "",
            *[f"- `{reason}`" for reason in audit.limitations],
            "",
            "이 결과는 동결된 합성 open-loop 원형 Actor 시뮬레이션의 연구 증거다. "
            "제품 알고리즘 채택, 실제 사람 탑승 안전성, G1~G5 또는 "
            "경로 분석 7단계를 결정하지 않는다.",
            "",
        )
    )
    return "\n".join(lines)


def _source_freeze_hash(repository_root: Path) -> str:
    lab_root = repository_root / "simulation" / "path_planning_lab"
    paths = tuple(sorted((lab_root / "src" / "hospital_path_lab").rglob("*.py"))) + (
        lab_root / "pyproject.toml",
    )
    digest = sha256()
    for path in paths:
        digest.update(path.relative_to(repository_root).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def _git_output(repository_root: Path, *args: str) -> str:
    result = subprocess.run(
        ("git", *args),
        cwd=repository_root,
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout.strip()


def _jsonable(value: object) -> object:
    if is_dataclass(value) and not isinstance(value, type):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _write_exclusive_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = (
        json.dumps(
            _jsonable(value),
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    )
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(payload)


def _write_atomic_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    payload = (
        json.dumps(
            _jsonable(value),
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n"
    )
    temporary.write_text(payload, encoding="utf-8", newline="\n")
    os.replace(temporary, path)


def _require_sha256(name: str, value: str) -> None:
    if len(value) != _SHA256_LENGTH or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise ValueError(f"{name} must be a lowercase SHA-256")


def _require_git_object_id(name: str, value: str) -> None:
    if len(value) not in (40, 64) or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise ValueError(f"{name} must be a lowercase Git object id")


__all__ = [
    "ExpectationAssessment",
    "ObservationFaultReplay",
    "WitnessAuditManifest",
    "WitnessAuditOutputWriter",
    "WitnessAuditRecord",
    "WitnessEpisodeAudit",
    "WitnessEvidenceClass",
    "WitnessPublicAudit",
    "audit_public_witness_corpus",
    "audit_public_witness_episode",
    "build_witness_audit_manifest",
    "public_witness_audit_episodes",
]
