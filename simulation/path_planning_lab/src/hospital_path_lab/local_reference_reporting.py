"""R4 local reference의 public-only reporting과 process qualification 하네스.

R3의 동결 public case 순서를 그대로 소비하되, expectation은 이 reporting 계층에만 둔다.
각 case는 독립 process에서 처리할 수 있지만 하나의 reference window sequence는 동일 manager로
직렬 실행한다. wall-clock과 worker 수는 운영 진단이며 semantic 판정에 포함하지 않는다.
"""

from __future__ import annotations

import json
import os
import subprocess
from collections.abc import Callable
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, is_dataclass, replace
from hashlib import sha256
from pathlib import Path
from time import perf_counter_ns

import numpy as np

from hospital_path_lab.contracts import GridSnapshot, SnapshotMetadata
from hospital_path_lab.local_reference_builder import (
    SpatialReferenceSource,
    build_spatial_reference_set,
    project_validated_spatial_seed,
)
from hospital_path_lab.local_reference_contracts import (
    REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
    LocalManeuverReferenceSet,
    ObservationDependency,
    ReferenceBuildContext,
    ReferenceBuildStatus,
    ReferenceEvidenceLevel,
)
from hospital_path_lab.local_reference_validation import (
    LocalReferenceValidation,
    validate_local_maneuver_reference,
)
from hospital_path_lab.local_reference_window import (
    LocalReferenceWindowManager,
    LocalReferenceWindowUpdate,
    WindowUpdateStatus,
    window_is_exact_slice,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.spatial_oracle_contracts import (
    ManeuverSide,
    SpatialOracleStatus,
    spatial_grid_content_hash,
)
from hospital_path_lab.spatial_oracle_reporting import (
    SPATIAL_PUBLIC_CASE_COUNT,
    SpatialPublicCase,
    SpatialPublicCaseResult,
    evaluate_spatial_public_case,
    public_spatial_cases,
)

LOCAL_REFERENCE_PUBLIC_CATALOG_VERSION = "local-reference-public-catalog-v1"
LOCAL_REFERENCE_PUBLIC_REPORT_VERSION = "local-reference-public-report-v1"
LOCAL_REFERENCE_PUBLIC_MANIFEST_VERSION = "local-reference-public-manifest-v1"
LOCAL_REFERENCE_PUBLIC_RECEIPT_VERSION = "local-reference-public-receipt-v1"
LOCAL_REFERENCE_PUBLIC_CASE_COUNT = SPATIAL_PUBLIC_CASE_COUNT
LOCAL_REFERENCE_PUBLIC_MANEUVER_REVISION = 1
LOCAL_REFERENCE_PUBLIC_PATH_REVISION = 1
LOCAL_REFERENCE_PUBLIC_STOP_EPOCH = 1
_RELATION_TOLERANCE_M = 0.02 + 1e-9
_SHA256_LENGTH = 64


@dataclass(frozen=True, slots=True)
class LocalReferencePublicCase:
    ordinal: int
    public_id: str
    spatial_case: SpatialPublicCase
    expected_build_status: ReferenceBuildStatus
    expected_candidate_count: int
    limitations: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if isinstance(self.ordinal, bool) or not isinstance(self.ordinal, int) or self.ordinal < 0:
            raise ValueError("public ordinal must be a non-negative exact integer")
        if not self.public_id or self.public_id != self.spatial_case.public_id:
            raise ValueError("R4 public id must match its R3 source case")
        if self.ordinal != self.spatial_case.ordinal:
            raise ValueError("R4 public ordinal must match its R3 source case")
        if not isinstance(self.expected_build_status, ReferenceBuildStatus):
            raise TypeError("expected_build_status must be ReferenceBuildStatus")
        if (
            isinstance(self.expected_candidate_count, bool)
            or not isinstance(self.expected_candidate_count, int)
            or self.expected_candidate_count < 0
        ):
            raise ValueError("expected_candidate_count must be a non-negative exact integer")
        if self.expected_build_status is ReferenceBuildStatus.REFERENCE_SET_READY:
            if self.expected_candidate_count != 1:
                raise ValueError("R4 v1 ready public case requires exactly one candidate")
        elif self.expected_candidate_count != 0:
            raise ValueError("non-ready public case cannot expect a candidate")
        object.__setattr__(self, "limitations", tuple(sorted(set(self.limitations))))

    @property
    def semantic_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "catalog_version": LOCAL_REFERENCE_PUBLIC_CATALOG_VERSION,
                "ordinal": self.ordinal,
                "public_id": self.public_id,
                "spatial_case_hash": self.spatial_case.semantic_content_hash,
                "expected_build_status": self.expected_build_status,
                "expected_candidate_count": self.expected_candidate_count,
                "limitations": self.limitations,
            }
        )


@dataclass(frozen=True, slots=True)
class LocalReferencePublicWindowSequence:
    candidate_id: str
    reference_session_id: str
    validation_content_hash: str
    updates: tuple[LocalReferenceWindowUpdate, ...]
    semantic_content_hash: str = ""

    def __post_init__(self) -> None:
        for name in ("candidate_id", "reference_session_id", "validation_content_hash"):
            _require_sha256(getattr(self, name), name)
        updates = tuple(self.updates)
        if not updates:
            raise ValueError("window sequence must not be empty")
        object.__setattr__(self, "updates", updates)
        expected = self.expected_content_hash
        if self.semantic_content_hash:
            _require_sha256(self.semantic_content_hash, "semantic_content_hash")
            if self.semantic_content_hash != expected:
                raise ValueError("window sequence semantic_content_hash mismatch")
        else:
            object.__setattr__(self, "semantic_content_hash", expected)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "candidate_id": self.candidate_id,
                "reference_session_id": self.reference_session_id,
                "validation_content_hash": self.validation_content_hash,
                "update_hashes": tuple(update.semantic_content_hash for update in self.updates),
            }
        )

    @property
    def all_ready(self) -> bool:
        return all(update.status is WindowUpdateStatus.WINDOW_READY for update in self.updates)


@dataclass(frozen=True, slots=True)
class LocalReferencePublicCaseResult:
    ordinal: int
    public_id: str
    case_content_hash: str
    spatial_result: SpatialPublicCaseResult
    build_context: ReferenceBuildContext
    reference_set: LocalManeuverReferenceSet
    validations: tuple[LocalReferenceValidation, ...]
    window_sequences: tuple[LocalReferencePublicWindowSequence, ...]
    hard_failures: tuple[str, ...]
    report_content_hash: str

    def __post_init__(self) -> None:
        _require_sha256(self.case_content_hash, "case_content_hash")
        _require_sha256(self.report_content_hash, "report_content_hash")
        validations = tuple(self.validations)
        sequences = tuple(self.window_sequences)
        object.__setattr__(self, "validations", validations)
        object.__setattr__(self, "window_sequences", sequences)
        object.__setattr__(self, "hard_failures", tuple(sorted(set(self.hard_failures))))
        if (
            self.ordinal != self.spatial_result.ordinal
            or self.public_id != self.spatial_result.public_id
        ):
            raise ValueError("R4 case result must preserve R3 public identity")
        if len(validations) != len(self.reference_set.candidates):
            raise ValueError("every reference candidate requires one validation")
        if len(sequences) != len(self.reference_set.candidates):
            raise ValueError("every reference candidate requires one window sequence")
        if self.reference_set.build_context_hash != self.build_context.context_content_hash:
            raise ValueError("reference set and build context hash mismatch")
        if self.report_content_hash != self.expected_report_content_hash:
            raise ValueError("case report_content_hash mismatch")

    @property
    def hard_passed(self) -> bool:
        return not self.hard_failures

    @property
    def semantic_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "ordinal": self.ordinal,
                "public_id": self.public_id,
                "case_content_hash": self.case_content_hash,
                "spatial_result_hash": self.spatial_result.semantic_content_hash,
                "build_context_hash": self.build_context.context_content_hash,
                "reference_set_hash": self.reference_set.semantic_content_hash,
                "validation_hashes": tuple(
                    item.validation_content_hash for item in self.validations
                ),
                "window_sequence_hashes": tuple(
                    item.semantic_content_hash for item in self.window_sequences
                ),
                "hard_failures": self.hard_failures,
            }
        )

    @property
    def expected_report_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "case_hash": self.case_content_hash,
                "semantic_content_hash": self.semantic_content_hash,
                "hard_failures": self.hard_failures,
            }
        )


@dataclass(frozen=True, slots=True)
class LocalReferencePublicAudit:
    report_version: str
    simulation_only: bool
    hidden_used: bool
    catalog_content_hash: str
    case_results: tuple[LocalReferencePublicCaseResult, ...]
    relation_failures: tuple[str, ...]
    parity_case_id: str
    serial_process_parity_passed: bool
    repeat_determinism_passed: bool
    hard_failures: tuple[str, ...]
    limitations: tuple[str, ...]
    semantic_content_hash: str
    report_content_hash: str
    elapsed_nonqualification_ns: int

    def __post_init__(self) -> None:
        if self.report_version != LOCAL_REFERENCE_PUBLIC_REPORT_VERSION:
            raise ValueError("unsupported R4 public report version")
        if not self.simulation_only or self.hidden_used:
            raise ValueError("R4 public audit must remain simulation-only and hidden-free")
        for name in ("catalog_content_hash", "semantic_content_hash", "report_content_hash"):
            _require_sha256(getattr(self, name), name)
        if self.elapsed_nonqualification_ns < 0:
            raise ValueError("elapsed time must be non-negative")
        results = tuple(self.case_results)
        if len(results) != LOCAL_REFERENCE_PUBLIC_CASE_COUNT:
            raise ValueError("R4 audit requires the complete frozen public catalog")
        if tuple(item.ordinal for item in results) != tuple(
            range(LOCAL_REFERENCE_PUBLIC_CASE_COUNT)
        ):
            raise ValueError("R4 public case order must be complete and contiguous")
        expected_catalog = canonical_content_hash(
            {"case_hashes": tuple(item.case_content_hash for item in results)}
        )
        if self.catalog_content_hash != expected_catalog:
            raise ValueError("R4 audit catalog hash mismatch")
        object.__setattr__(self, "case_results", results)
        object.__setattr__(self, "relation_failures", tuple(sorted(set(self.relation_failures))))
        object.__setattr__(self, "hard_failures", tuple(sorted(set(self.hard_failures))))
        object.__setattr__(self, "limitations", tuple(sorted(set(self.limitations))))
        if self.semantic_content_hash != self.expected_semantic_content_hash:
            raise ValueError("R4 audit semantic_content_hash mismatch")
        if self.report_content_hash != self.expected_report_content_hash:
            raise ValueError("R4 audit report_content_hash mismatch")

    @property
    def hard_passed(self) -> bool:
        return (
            not self.hard_failures
            and not self.relation_failures
            and self.serial_process_parity_passed
            and self.repeat_determinism_passed
        )

    @property
    def expected_semantic_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "report_version": self.report_version,
                "catalog_content_hash": self.catalog_content_hash,
                "case_result_hashes": tuple(
                    item.semantic_content_hash for item in self.case_results
                ),
                "relation_failures": self.relation_failures,
                "parity_case_id": self.parity_case_id,
                "serial_process_parity_passed": self.serial_process_parity_passed,
                "repeat_determinism_passed": self.repeat_determinism_passed,
                "hard_failures": self.hard_failures,
            }
        )

    @property
    def expected_report_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "semantic_content_hash": self.semantic_content_hash,
                "case_report_hashes": tuple(
                    item.report_content_hash for item in self.case_results
                ),
            }
        )


@dataclass(frozen=True, slots=True)
class LocalReferencePublicManifest:
    manifest_version: str
    simulation_only: bool
    hidden_used: bool
    git_head: str
    git_tree: str
    git_dirty: bool
    source_freeze_hash: str
    catalog_content_hash: str
    case_order: tuple[tuple[str, str], ...]
    max_workers_nonsemantic: int
    logical_cpu_count_nonsemantic: int
    semantic_content_hash: str
    content_hash: str

    def __post_init__(self) -> None:
        if self.manifest_version != LOCAL_REFERENCE_PUBLIC_MANIFEST_VERSION:
            raise ValueError("unsupported R4 public manifest version")
        if not self.simulation_only or self.hidden_used:
            raise ValueError("R4 manifest must remain simulation-only and hidden-free")
        for name in (
            "source_freeze_hash",
            "catalog_content_hash",
            "semantic_content_hash",
            "content_hash",
        ):
            _require_sha256(getattr(self, name), name)
        _require_git_object_id(self.git_head, "git_head")
        _require_git_object_id(self.git_tree, "git_tree")
        if not isinstance(self.git_dirty, bool):
            raise TypeError("git_dirty must be a bool")
        if self.max_workers_nonsemantic <= 0 or self.logical_cpu_count_nonsemantic <= 0:
            raise ValueError("operational CPU values must be positive")
        if len(self.case_order) != LOCAL_REFERENCE_PUBLIC_CASE_COUNT:
            raise ValueError("manifest requires the complete R4 public order")
        for public_id, case_hash in self.case_order:
            if not public_id:
                raise ValueError("manifest public id must not be empty")
            _require_sha256(case_hash, "manifest case hash")
        if self.semantic_content_hash != self.expected_semantic_content_hash:
            raise ValueError("manifest semantic_content_hash mismatch")
        if self.content_hash != self.expected_content_hash:
            raise ValueError("manifest content_hash mismatch")

    @property
    def expected_semantic_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "manifest_version": self.manifest_version,
                "simulation_only": self.simulation_only,
                "hidden_used": self.hidden_used,
                "source_freeze_hash": self.source_freeze_hash,
                "catalog_content_hash": self.catalog_content_hash,
                "case_order": self.case_order,
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


def public_local_reference_cases() -> tuple[LocalReferencePublicCase, ...]:
    cases = tuple(
        LocalReferencePublicCase(
            ordinal=spatial.ordinal,
            public_id=spatial.public_id,
            spatial_case=spatial,
            expected_build_status=_expected_build_status(spatial),
            expected_candidate_count=(
                1
                if _expected_build_status(spatial) is ReferenceBuildStatus.REFERENCE_SET_READY
                else 0
            ),
            limitations=("r3_public_input_order_preserved",),
        )
        for spatial in public_spatial_cases()
    )
    if len(cases) != LOCAL_REFERENCE_PUBLIC_CASE_COUNT:
        raise RuntimeError("R4 public catalog count invariant failed")
    if tuple(case.ordinal for case in cases) != tuple(range(LOCAL_REFERENCE_PUBLIC_CASE_COUNT)):
        raise RuntimeError("R4 public catalog ordinal invariant failed")
    return cases


def build_public_reference_context(case: LocalReferencePublicCase) -> ReferenceBuildContext:
    request = case.spatial_case.request
    reference = (request.reference_segment.start, request.reference_segment.end)
    snapshot_hash = canonical_content_hash(
        {
            "catalog_version": LOCAL_REFERENCE_PUBLIC_CATALOG_VERSION,
            "case_hash": case.semantic_content_hash,
            "grid_content_hash": spatial_grid_content_hash(request.static_grid),
            "forbidden_cells": request.forbidden_cells,
        }
    )
    snapshot = GridSnapshot(
        SnapshotMetadata(
            map_id=request.map_id,
            map_revision=request.map_revision,
            mission_revision=request.mission_revision,
            observation_revision=0,
            seed=20260814 + case.ordinal,
            content_hash=snapshot_hash,
        ),
        request.static_grid,
        frozenset(request.forbidden_cells),
    )
    return ReferenceBuildContext(
        schema_version=REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION,
        mission_id=f"r4-public-mission-{case.ordinal:02d}",
        stop_epoch=LOCAL_REFERENCE_PUBLIC_STOP_EPOCH,
        map_id=request.map_id,
        map_revision=request.map_revision,
        mission_revision=request.mission_revision,
        observation_dependency=ObservationDependency.STATIC_ONLY,
        observation_revision=None,
        observation_content_hash=None,
        static_grid_snapshot=snapshot,
        grid_content_hash=spatial_grid_content_hash(request.static_grid),
        allowed_region=request.allowed_region,
        allowed_region_hash=request.allowed_region.content_hash,
        forbidden_cells=request.forbidden_cells,
        forbidden_region_hash=canonical_content_hash(request.forbidden_cells),
        vehicle_profile=request.vehicle_profile,
        vehicle_profile_hash=request.vehicle_profile_hash,
        original_reference=reference,
        original_reference_hash=canonical_content_hash(reference),
        current_robot_pose=request.start_pose,
        control_tick=0,
        simulation_time_s=0.0,
    )


def evaluate_local_reference_public_case(
    case: LocalReferencePublicCase,
) -> LocalReferencePublicCaseResult:
    if not isinstance(case, LocalReferencePublicCase):
        raise TypeError("case must be a LocalReferencePublicCase")
    spatial_result = evaluate_spatial_public_case(case.spatial_case)
    context = build_public_reference_context(case)
    source = SpatialReferenceSource(case.spatial_case.request, spatial_result.result)
    reference_set = build_spatial_reference_set(
        context,
        (source,),
        maneuver_revision=LOCAL_REFERENCE_PUBLIC_MANEUVER_REVISION,
        path_revision=LOCAL_REFERENCE_PUBLIC_PATH_REVISION,
        elapsed_nonqualification_ns=0,
    )
    validations: list[LocalReferenceValidation] = []
    sequences: list[LocalReferencePublicWindowSequence] = []
    if reference_set.candidates:
        seed = project_validated_spatial_seed(context, source)
        for reference in reference_set.candidates:
            validation = validate_local_maneuver_reference(
                context,
                reference,
                spatial_seed=seed,
            )
            validations.append(validation)
            manager = LocalReferenceWindowManager()
            updates: list[LocalReferenceWindowUpdate] = []
            for tick, knot in enumerate(reference.knots):
                tick_context = replace(
                    context,
                    current_robot_pose=knot.pose,
                    control_tick=tick,
                    simulation_time_s=tick * 0.05,
                    context_content_hash="",
                )
                updates.append(manager.update(tick_context, reference, validation))
            sequences.append(
                LocalReferencePublicWindowSequence(
                    candidate_id=reference.candidate_id,
                    reference_session_id=reference.reference_session_id,
                    validation_content_hash=validation.validation_content_hash,
                    updates=tuple(updates),
                )
            )

    failures = _case_failures(
        case,
        spatial_result,
        reference_set,
        tuple(validations),
        tuple(sequences),
    )
    semantic_payload = {
        "ordinal": case.ordinal,
        "public_id": case.public_id,
        "case_content_hash": case.semantic_content_hash,
        "spatial_result_hash": spatial_result.semantic_content_hash,
        "build_context_hash": context.context_content_hash,
        "reference_set_hash": reference_set.semantic_content_hash,
        "validation_hashes": tuple(item.validation_content_hash for item in validations),
        "window_sequence_hashes": tuple(item.semantic_content_hash for item in sequences),
        "hard_failures": tuple(sorted(failures)),
    }
    semantic_hash = canonical_content_hash(semantic_payload)
    report_hash = canonical_content_hash(
        {
            "case_hash": case.semantic_content_hash,
            "semantic_content_hash": semantic_hash,
            "hard_failures": tuple(sorted(failures)),
        }
    )
    return LocalReferencePublicCaseResult(
        ordinal=case.ordinal,
        public_id=case.public_id,
        case_content_hash=case.semantic_content_hash,
        spatial_result=spatial_result,
        build_context=context,
        reference_set=reference_set,
        validations=tuple(validations),
        window_sequences=tuple(sequences),
        hard_failures=tuple(failures),
        report_content_hash=report_hash,
    )


def evaluate_local_reference_public_cases(
    cases: tuple[LocalReferencePublicCase, ...],
    *,
    max_workers: int,
    on_case: Callable[[LocalReferencePublicCaseResult], None] | None = None,
) -> tuple[LocalReferencePublicCaseResult, ...]:
    if isinstance(max_workers, bool) or not isinstance(max_workers, int) or max_workers <= 0:
        raise ValueError("max_workers must be a positive exact integer")
    if len({case.ordinal for case in cases}) != len(cases):
        raise ValueError("case ordinals must be unique")
    materialized: list[LocalReferencePublicCaseResult] = []
    if max_workers == 1:
        for case in cases:
            result = evaluate_local_reference_public_case(case)
            materialized.append(result)
            if on_case is not None:
                on_case(result)
    else:
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            futures = {
                executor.submit(evaluate_local_reference_public_case, case): case.ordinal
                for case in cases
            }
            for future in as_completed(futures):
                result = future.result()
                materialized.append(result)
                if on_case is not None:
                    on_case(result)
    return tuple(sorted(materialized, key=lambda item: item.ordinal))


def audit_local_reference_public_catalog(
    *,
    max_workers: int,
    on_case: Callable[[LocalReferencePublicCaseResult], None] | None = None,
) -> LocalReferencePublicAudit:
    cases = public_local_reference_cases()
    started = perf_counter_ns()
    results = evaluate_local_reference_public_cases(
        cases,
        max_workers=max_workers,
        on_case=on_case,
    )
    parity_case = cases[0]
    serial_first = evaluate_local_reference_public_case(parity_case)
    serial_second = evaluate_local_reference_public_case(parity_case)
    process_hash = results[parity_case.ordinal].semantic_content_hash
    parity_passed = serial_first.semantic_content_hash == process_hash
    repeat_passed = serial_first.semantic_content_hash == serial_second.semantic_content_hash
    relation_failures = _relation_failures(results)
    hard_failures = [
        f"{item.public_id}:{failure}" for item in results for failure in item.hard_failures
    ]
    hard_failures.extend(relation_failures)
    if not parity_passed:
        hard_failures.append("serial_process_semantic_parity_failed")
    if not repeat_passed:
        hard_failures.append("repeat_determinism_failed")
    catalog_hash = canonical_content_hash(
        {"case_hashes": tuple(case.semantic_content_hash for case in cases)}
    )
    semantic_hash = canonical_content_hash(
        {
            "report_version": LOCAL_REFERENCE_PUBLIC_REPORT_VERSION,
            "catalog_content_hash": catalog_hash,
            "case_result_hashes": tuple(item.semantic_content_hash for item in results),
            "relation_failures": tuple(sorted(relation_failures)),
            "parity_case_id": parity_case.public_id,
            "serial_process_parity_passed": parity_passed,
            "repeat_determinism_passed": repeat_passed,
            "hard_failures": tuple(sorted(hard_failures)),
        }
    )
    report_hash = canonical_content_hash(
        {
            "semantic_content_hash": semantic_hash,
            "case_report_hashes": tuple(item.report_content_hash for item in results),
        }
    )
    return LocalReferencePublicAudit(
        report_version=LOCAL_REFERENCE_PUBLIC_REPORT_VERSION,
        simulation_only=True,
        hidden_used=False,
        catalog_content_hash=catalog_hash,
        case_results=results,
        relation_failures=tuple(relation_failures),
        parity_case_id=parity_case.public_id,
        serial_process_parity_passed=parity_passed,
        repeat_determinism_passed=repeat_passed,
        hard_failures=tuple(hard_failures),
        limitations=(
            "offline_static_reference_generation_only",
            "r3_public_source_recomputed",
            "spatial_only_no_dynamic_safety_claim",
            "python_wall_clock_is_nonqualification",
            "no_controller_or_movement_authority",
            "no_product_or_human_safety_claim",
        ),
        semantic_content_hash=semantic_hash,
        report_content_hash=report_hash,
        elapsed_nonqualification_ns=perf_counter_ns() - started,
    )


def build_local_reference_public_manifest(
    *, repository_root: Path, max_workers: int
) -> LocalReferencePublicManifest:
    if isinstance(max_workers, bool) or not isinstance(max_workers, int) or max_workers <= 0:
        raise ValueError("max_workers must be a positive exact integer")
    cases = public_local_reference_cases()
    catalog_hash = canonical_content_hash(
        {"case_hashes": tuple(case.semantic_content_hash for case in cases)}
    )
    head = _git_output(repository_root, "rev-parse", "HEAD")
    tree = _git_output(repository_root, "rev-parse", "HEAD^{tree}")
    dirty = bool(_git_output(repository_root, "status", "--porcelain=v1"))
    source_hash = _source_freeze_hash(repository_root)
    order = tuple((case.public_id, case.semantic_content_hash) for case in cases)
    semantic_hash = canonical_content_hash(
        {
            "manifest_version": LOCAL_REFERENCE_PUBLIC_MANIFEST_VERSION,
            "simulation_only": True,
            "hidden_used": False,
            "source_freeze_hash": source_hash,
            "catalog_content_hash": catalog_hash,
            "case_order": order,
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
    return LocalReferencePublicManifest(
        manifest_version=LOCAL_REFERENCE_PUBLIC_MANIFEST_VERSION,
        simulation_only=True,
        hidden_used=False,
        git_head=head,
        git_tree=tree,
        git_dirty=dirty,
        source_freeze_hash=source_hash,
        catalog_content_hash=catalog_hash,
        case_order=order,
        max_workers_nonsemantic=max_workers,
        logical_cpu_count_nonsemantic=os.cpu_count() or 1,
        semantic_content_hash=semantic_hash,
        content_hash=content_hash,
    )


class LocalReferencePublicOutputWriter:
    def __init__(
        self,
        output_dir: Path,
        manifest: LocalReferencePublicManifest,
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

    def write_case(self, result: LocalReferencePublicCaseResult) -> None:
        key = (result.public_id, result.case_content_hash)
        if key not in self.manifest.case_order:
            raise ValueError("case is not part of the frozen R4 manifest")
        if key in self._written:
            raise FileExistsError("case was already written")
        directory = self.output_dir / "cases" / f"{result.ordinal:02d}-{result.public_id}"
        directory.mkdir(parents=True, exist_ok=False)
        _write_exclusive_json(
            directory / "build-context.json",
            _context_payload(result.build_context),
        )
        _write_exclusive_json(directory / "source-evidence.json", result.spatial_result)
        _write_exclusive_json(directory / "reference-set.json", result.reference_set)
        _write_exclusive_json(directory / "validation.json", result.validations)
        _write_exclusive_json(directory / "window-sequence.json", result.window_sequences)
        _write_exclusive_json(directory / "assessment.json", result)
        _save_reference_plot(result, directory / "reference.png")
        self._written.add(key)
        _write_atomic_json(
            self.output_dir / "partial-state.json",
            {
                "partial": True,
                "manifest_content_hash": self.manifest.content_hash,
                "completed_cases": tuple(sorted(public_id for public_id, _ in self._written)),
            },
        )

    def complete(self, audit: LocalReferencePublicAudit) -> tuple[Path, Path, Path | None]:
        expected = self.manifest.case_order
        actual = tuple((item.public_id, item.case_content_hash) for item in audit.case_results)
        if actual != expected:
            raise ValueError("R4 audit does not match frozen manifest order")
        if self._written != set(expected):
            raise RuntimeError("cannot complete before every R4 public case is written")
        if audit.catalog_content_hash != self.manifest.catalog_content_hash:
            raise ValueError("R4 audit catalog hash mismatch")
        if _source_freeze_hash(self.repository_root) != self.manifest.source_freeze_hash:
            raise RuntimeError("source changed before R4 public completion")
        summary_json = self.output_dir / "summary.json"
        summary_md = self.output_dir / "summary.md"
        complete_state = self.output_dir / "complete-state.json"
        _write_exclusive_json(summary_json, audit)
        _write_exclusive_text(summary_md, _audit_summary(audit))
        _write_exclusive_json(
            complete_state,
            {
                "partial": False,
                "manifest_content_hash": self.manifest.content_hash,
                "audit_semantic_content_hash": audit.semantic_content_hash,
                "hard_passed": audit.hard_passed,
                "case_count": len(audit.case_results),
            },
        )
        (self.output_dir / "partial-state.json").unlink()
        receipt_path: Path | None = None
        if audit.hard_passed and not self.manifest.git_dirty:
            self._verify_git_state()
            receipt_path = self.output_dir / "qualification-receipt.json"
            receipt: dict[str, object] = {
                "receipt_version": LOCAL_REFERENCE_PUBLIC_RECEIPT_VERSION,
                "simulation_only": True,
                "hidden_used": False,
                "qualified": True,
                "git_head": self.manifest.git_head,
                "git_tree": self.manifest.git_tree,
                "source_freeze_hash": self.manifest.source_freeze_hash,
                "manifest_content_hash": self.manifest.content_hash,
                "catalog_content_hash": audit.catalog_content_hash,
                "audit_semantic_content_hash": audit.semantic_content_hash,
                "case_count": len(audit.case_results),
                "limitations": audit.limitations,
            }
            receipt["receipt_content_hash"] = canonical_content_hash(receipt)
            _write_exclusive_json(receipt_path, receipt)
        return summary_json, summary_md, receipt_path

    def _verify_git_state(self) -> None:
        if _git_output(self.repository_root, "rev-parse", "HEAD") != self.manifest.git_head:
            raise RuntimeError("Git HEAD changed before R4 receipt")
        if _git_output(self.repository_root, "rev-parse", "HEAD^{tree}") != self.manifest.git_tree:
            raise RuntimeError("Git tree changed before R4 receipt")
        if _git_output(self.repository_root, "status", "--porcelain=v1"):
            raise RuntimeError("Git worktree became dirty before R4 receipt")


def _expected_build_status(case: SpatialPublicCase) -> ReferenceBuildStatus:
    if case.expected_status is SpatialOracleStatus.RESOURCE_LIMIT:
        return ReferenceBuildStatus.SEARCH_INCONCLUSIVE
    if case.expected_status is SpatialOracleStatus.INVALID_INPUT:
        return ReferenceBuildStatus.INVALID_INPUT
    if case.expected_status is SpatialOracleStatus.SPATIALLY_INFEASIBLE:
        return ReferenceBuildStatus.NO_REFERENCE
    if case.request.maneuver_side in (ManeuverSide.LEFT, ManeuverSide.RIGHT):
        return ReferenceBuildStatus.REFERENCE_SET_READY
    return ReferenceBuildStatus.NO_REFERENCE


def _case_failures(
    case: LocalReferencePublicCase,
    spatial_result: SpatialPublicCaseResult,
    reference_set: LocalManeuverReferenceSet,
    validations: tuple[LocalReferenceValidation, ...],
    sequences: tuple[LocalReferencePublicWindowSequence, ...],
) -> tuple[str, ...]:
    failures = [f"r3_source:{failure}" for failure in spatial_result.hard_failures]
    if reference_set.status is not case.expected_build_status:
        failures.append(
            f"build_status_mismatch:{case.expected_build_status.value}:{reference_set.status.value}"
        )
    if len(reference_set.candidates) != case.expected_candidate_count:
        failures.append("candidate_count_mismatch")
    for index, (reference, validation, sequence) in enumerate(
        zip(reference_set.candidates, validations, sequences, strict=True)
    ):
        prefix = f"candidate_{index}"
        if reference.evidence_level is not ReferenceEvidenceLevel.SPATIAL_ONLY:
            failures.append(f"{prefix}:non_spatial_evidence_claim")
        if not validation.passed:
            failures.append(f"{prefix}:independent_validation_failed")
        if not sequence.all_ready:
            failures.append(f"{prefix}:window_sequence_not_ready")
            continue
        if any(
            update.window is None or not window_is_exact_slice(reference, update.window)
            for update in sequence.updates
        ):
            failures.append(f"{prefix}:window_not_exact_slice")
        windows = tuple(update.window for update in sequence.updates)
        if any(window is None for window in windows):
            failures.append(f"{prefix}:window_missing")
            continue
        ready_windows = tuple(window for window in windows if window is not None)
        if any(
            window.reference_session_id != reference.reference_session_id
            or window.full_reference_hash != reference.reference_content_hash
            or window.path_revision != reference.path_revision
            or window.maneuver_revision != reference.maneuver_revision
            for window in ready_windows
        ):
            failures.append(f"{prefix}:window_identity_mismatch")
        revisions = tuple(window.subgoal_revision for window in ready_windows)
        if any(right < left for left, right in zip(revisions, revisions[1:], strict=False)):
            failures.append(f"{prefix}:subgoal_revision_regression")
        if not ready_windows[-1].terminal_rejoin_included:
            failures.append(f"{prefix}:terminal_window_missing")
    if case.expected_build_status is ReferenceBuildStatus.SEARCH_INCONCLUSIVE and (
        "search_resource_limit_passthrough" not in reference_set.limitations
    ):
        failures.append("resource_limit_not_preserved")
    if case.expected_build_status is ReferenceBuildStatus.INVALID_INPUT and (
        reference_set.termination_reason != "invalid_spatial_source"
    ):
        failures.append("invalid_source_not_fail_closed")
    return tuple(sorted(set(failures)))


def _relation_failures(
    results: tuple[LocalReferencePublicCaseResult, ...],
) -> tuple[str, ...]:
    by_id = {item.public_id: item for item in results}
    failures: list[str] = []
    for first_id, second_id, label in (
        ("wide-straight-left", "wide-mirror-right", "wide_left_mirror"),
        ("wide-straight-right", "wide-mirror-left", "wide_right_mirror"),
        ("wide-straight-left", "vertical-left", "vertical_left"),
        ("wide-straight-right", "vertical-right", "vertical_right"),
    ):
        first = by_id[first_id]
        second = by_id[second_id]
        if first.reference_set.status is not second.reference_set.status:
            failures.append(f"{label}:status")
            continue
        if not first.reference_set.candidates or not second.reference_set.candidates:
            continue
        left = first.reference_set.candidates[0]
        right = second.reference_set.candidates[0]
        if tuple(section.section_kind for section in left.sections) != tuple(
            section.section_kind for section in right.sections
        ):
            failures.append(f"{label}:section_kinds")
        if len(left.knots) != len(right.knots):
            failures.append(f"{label}:knot_count")
        if (
            abs(
                left.knots[-1].cumulative_translation_arc_m
                - right.knots[-1].cumulative_translation_arc_m
            )
            > _RELATION_TOLERANCE_M
        ):
            failures.append(f"{label}:translation_arc")
        if (
            abs(
                left.minimum_validated_static_clearance_m
                - right.minimum_validated_static_clearance_m
            )
            > _RELATION_TOLERANCE_M
        ):
            failures.append(f"{label}:minimum_clearance")
    return tuple(sorted(set(failures)))


def _context_payload(context: ReferenceBuildContext) -> dict[str, object]:
    return {
        "schema_version": context.schema_version,
        "mission_id": context.mission_id,
        "stop_epoch": context.stop_epoch,
        "map_id": context.map_id,
        "map_revision": context.map_revision,
        "mission_revision": context.mission_revision,
        "observation_dependency": context.observation_dependency,
        "grid_content_hash": context.grid_content_hash,
        "allowed_region_hash": context.allowed_region_hash,
        "forbidden_region_hash": context.forbidden_region_hash,
        "vehicle_profile_hash": context.vehicle_profile_hash,
        "original_reference": context.original_reference,
        "original_reference_hash": context.original_reference_hash,
        "current_robot_pose": context.current_robot_pose,
        "control_tick": context.control_tick,
        "simulation_time_s": context.simulation_time_s,
        "context_content_hash": context.context_content_hash,
    }


def _save_reference_plot(result: LocalReferencePublicCaseResult, output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    request = result.spatial_result.request
    grid = request.static_grid
    figure, axis = plt.subplots(figsize=(8.0, 6.0))
    try:
        rgba = np.zeros((grid.height, grid.width, 4), dtype=float)
        rgba[grid.occupancy] = (0.20, 0.20, 0.20, 0.85)
        for x, y in request.forbidden_cells:
            rgba[y, x] = (0.90, 0.15, 0.15, 0.60)
        extent = (
            grid.origin_x_m,
            grid.origin_x_m + grid.width * grid.resolution_m,
            grid.origin_y_m,
            grid.origin_y_m + grid.height * grid.resolution_m,
        )
        axis.imshow(rgba, origin="lower", extent=extent, interpolation="nearest")
        base = request.reference_segment
        axis.plot(
            (base.start.x, base.end.x),
            (base.start.y, base.end.y),
            "--",
            color="tab:blue",
            linewidth=1.5,
            label="original reference",
        )
        for index, reference in enumerate(result.reference_set.candidates):
            axis.plot(
                [knot.pose.x for knot in reference.knots],
                [knot.pose.y for knot in reference.knots],
                linewidth=2.0,
                label=f"R4 candidate {index}",
            )
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        axis.grid(alpha=0.15)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_title(
            f"R4 local reference\n{result.public_id} | "
            f"{result.reference_set.status.value} | "
            f"hard={'PASS' if result.hard_passed else 'FAIL'}"
        )
        axis.legend(fontsize=8, loc="best")
        figure.tight_layout()
        figure.savefig(output_path, dpi=160, format="png")
    finally:
        plt.close(figure)


def _audit_summary(audit: LocalReferencePublicAudit) -> str:
    lines = [
        "# R4 local reference 공개 qualification",
        "",
        f"- hard 판정: `{'PASS' if audit.hard_passed else 'FAIL'}`",
        f"- 공개 case: `{len(audit.case_results)}`",
        f"- hidden 사용: `{str(audit.hidden_used).lower()}`",
        f"- serial/process parity: `{'PASS' if audit.serial_process_parity_passed else 'FAIL'}`",
        f"- repeat determinism: `{'PASS' if audit.repeat_determinism_passed else 'FAIL'}`",
        f"- semantic hash: `{audit.semantic_content_hash}`",
        "- Python wall-clock과 worker 수는 판정 근거가 아니다.",
        "",
        "| id | R3 status | R4 status | candidates | hard |",
        "|---|---|---|---:|---|",
    ]
    for item in audit.case_results:
        lines.append(
            f"| `{item.public_id}` | {item.spatial_result.result.status.value} | "
            f"{item.reference_set.status.value} | {len(item.reference_set.candidates)} | "
            f"{'PASS' if item.hard_passed else 'FAIL'} |"
        )
    lines.extend(("", "## 해석 제한", ""))
    lines.extend(f"- `{value}`" for value in audit.limitations)
    lines.extend(
        (
            "",
            "이 결과는 동결된 공개 정적 지도에서 R3 source를 immutable R4 reference와 "
            "sliding window로 손실 없이 변환하는 offline 연구 증거다. controller 추종, "
            "동적 Actor 안전, 이동 허가, 제품 알고리즘 또는 실제 사람 안전을 증명하지 않는다.",
            "",
        )
    )
    return "\n".join(lines)


def _source_freeze_hash(repository_root: Path) -> str:
    lab = repository_root / "simulation" / "path_planning_lab"
    paths = tuple(sorted((lab / "src" / "hospital_path_lab").rglob("*.py"))) + (
        lab / "scripts" / "run_local_reference_public.py",
        lab / "pyproject.toml",
    )
    digest = sha256()
    for path in paths:
        digest.update(path.relative_to(repository_root).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


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
        return [_jsonable(item) for item in sorted(value)]
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def _write_exclusive_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(
        _jsonable(value),
        ensure_ascii=False,
        sort_keys=True,
        indent=2,
        allow_nan=False,
    ) + "\n"
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(payload)


def _write_atomic_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(
            _jsonable(value),
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
        newline="\n",
    )
    os.replace(temporary, path)


def _write_exclusive_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(value)


def _require_git_object_id(value: str, field_name: str) -> None:
    if (
        not isinstance(value, str)
        or len(value) not in (40, 64)
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError(f"{field_name} must be a lowercase Git object id")


def _require_sha256(value: str, field_name: str) -> None:
    if (
        not isinstance(value, str)
        or len(value) != _SHA256_LENGTH
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError(f"{field_name} must be a lowercase SHA-256")


__all__ = [
    "LOCAL_REFERENCE_PUBLIC_CASE_COUNT",
    "LOCAL_REFERENCE_PUBLIC_CATALOG_VERSION",
    "LocalReferencePublicAudit",
    "LocalReferencePublicCase",
    "LocalReferencePublicCaseResult",
    "LocalReferencePublicManifest",
    "LocalReferencePublicOutputWriter",
    "LocalReferencePublicWindowSequence",
    "audit_local_reference_public_catalog",
    "build_local_reference_public_manifest",
    "build_public_reference_context",
    "evaluate_local_reference_public_case",
    "evaluate_local_reference_public_cases",
    "public_local_reference_cases",
]
