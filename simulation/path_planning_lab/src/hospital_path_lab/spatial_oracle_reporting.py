"""R3 bounded 공간 oracle의 public-only qualification과 불변 산출물.

기대 판정은 reporting 계층에만 존재한다. 검색 함수는 정적 request만 받고 Actor, corpus
category, observation, controller 또는 hidden 입력을 보지 않는다. Python wall-clock과 worker
수는 운영 진단이며 semantic 판정에서 제외한다.
"""

from __future__ import annotations

import json
import os
import subprocess
from collections.abc import Callable
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass, is_dataclass, replace
from enum import StrEnum
from hashlib import sha256
from math import pi
from pathlib import Path
from time import perf_counter_ns

import numpy as np

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    generate_dynamic_corpus,
)
from hospital_path_lab.dynamic_witness_contracts import project_public_witness_world
from hospital_path_lab.grid import GridMap
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.spatial_oracle_contracts import (
    SPATIAL_ORACLE_SCHEMA_VERSION,
    BoundedSpatialOracleRequest,
    ManeuverSide,
    SpatialAllowedRegion,
    SpatialLatticeConfig,
    SpatialOracleStatus,
    SpatialReferenceSegment,
    SpatialRejoinGoal,
    SpatialSearchRegion,
    build_bounded_spatial_request,
)
from hospital_path_lab.spatial_oracle_lattice import search_bounded_spatial_oracle
from hospital_path_lab.spatial_oracle_projection import (
    project_witness_world_to_spatial_request,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1

SPATIAL_PUBLIC_CATALOG_VERSION = "bounded-spatial-public-catalog-v1"
SPATIAL_PUBLIC_REPORT_VERSION = "bounded-spatial-public-report-v1"
SPATIAL_PUBLIC_MANIFEST_VERSION = "bounded-spatial-public-manifest-v1"
SPATIAL_PUBLIC_RECEIPT_VERSION = "bounded-spatial-public-receipt-v1"
SPATIAL_PUBLIC_CASE_COUNT = 21
SPATIAL_PUBLIC_RESOLUTION_M = 0.02
_RESOURCE_EXACT_EXPANDED = 1_056
_RESOURCE_EXACT_GENERATED = 4_320
_RESOURCE_EXACT_OPEN = 233
_RELATION_TOLERANCE = 1e-9
_SHA256_LENGTH = 64


class SpatialPublicRelation(StrEnum):
    WIDE_MIRROR = "wide_mirror"
    VERTICAL_ROTATION = "vertical_rotation"
    RESOURCE_BOUNDARY = "resource_boundary"


@dataclass(frozen=True, slots=True)
class SpatialPublicCase:
    ordinal: int
    public_id: str
    request: BoundedSpatialOracleRequest
    expected_status: SpatialOracleStatus
    expected_reason: str | None = None
    relation: SpatialPublicRelation | None = None
    limitations: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if isinstance(self.ordinal, bool) or not isinstance(self.ordinal, int) or self.ordinal < 0:
            raise ValueError("public ordinal must be a non-negative exact integer")
        if not self.public_id:
            raise ValueError("public_id must not be empty")
        if not isinstance(self.request, BoundedSpatialOracleRequest):
            raise TypeError("public case requires a bounded spatial request")
        if not isinstance(self.expected_status, SpatialOracleStatus):
            raise TypeError("expected_status must be SpatialOracleStatus")
        if self.relation is not None and not isinstance(self.relation, SpatialPublicRelation):
            raise TypeError("relation must be SpatialPublicRelation or None")
        object.__setattr__(self, "limitations", tuple(sorted(set(self.limitations))))

    @property
    def semantic_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "catalog_version": SPATIAL_PUBLIC_CATALOG_VERSION,
                "ordinal": self.ordinal,
                "public_id": self.public_id,
                "request_content_hash": self.request.request_content_hash,
                "expected_status": self.expected_status,
                "expected_reason": self.expected_reason,
                "relation": self.relation,
                "limitations": self.limitations,
            }
        )


@dataclass(frozen=True, slots=True)
class SpatialPublicCaseResult:
    ordinal: int
    public_id: str
    case_content_hash: str
    request: BoundedSpatialOracleRequest
    expected_status: SpatialOracleStatus
    expected_reason: str | None
    relation: SpatialPublicRelation | None
    result: object
    hard_failures: tuple[str, ...]
    report_content_hash: str

    def __post_init__(self) -> None:
        from hospital_path_lab.spatial_oracle_contracts import BoundedSpatialOracleResult

        if not isinstance(self.result, BoundedSpatialOracleResult):
            raise TypeError("case result requires BoundedSpatialOracleResult")
        _require_sha256(self.case_content_hash, "case_content_hash")
        _require_sha256(self.report_content_hash, "report_content_hash")
        object.__setattr__(self, "hard_failures", tuple(sorted(set(self.hard_failures))))
        if self.request.request_content_hash != self.result.request_content_hash:
            raise ValueError("case result request hash mismatch")
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
                "request_content_hash": self.request.request_content_hash,
                "expected_status": self.expected_status,
                "expected_reason": self.expected_reason,
                "relation": self.relation,
                "result_semantic_hash": self.result.semantic_content_hash,
                "hard_failures": self.hard_failures,
            }
        )

    @property
    def expected_report_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "case_hash": self.case_content_hash,
                "result_hash": self.result.semantic_content_hash,
                "hard_failures": self.hard_failures,
            }
        )


@dataclass(frozen=True, slots=True)
class SpatialPublicAudit:
    report_version: str
    simulation_only: bool
    hidden_used: bool
    catalog_content_hash: str
    case_results: tuple[SpatialPublicCaseResult, ...]
    relation_failures: tuple[str, ...]
    parity_case_id: str
    parity_passed: bool
    hard_failures: tuple[str, ...]
    limitations: tuple[str, ...]
    semantic_content_hash: str
    report_content_hash: str
    elapsed_nonqualification_ns: int

    def __post_init__(self) -> None:
        if self.report_version != SPATIAL_PUBLIC_REPORT_VERSION:
            raise ValueError("unsupported spatial public report version")
        if not self.simulation_only or self.hidden_used:
            raise ValueError("R3 public audit must remain simulation-only and hidden-free")
        for name in ("catalog_content_hash", "semantic_content_hash", "report_content_hash"):
            _require_sha256(getattr(self, name), name)
        if self.elapsed_nonqualification_ns < 0:
            raise ValueError("elapsed time must not be negative")
        if len(self.case_results) != SPATIAL_PUBLIC_CASE_COUNT:
            raise ValueError("R3 public audit requires the complete frozen catalog")
        if tuple(item.ordinal for item in self.case_results) != tuple(
            range(SPATIAL_PUBLIC_CASE_COUNT)
        ):
            raise ValueError("R3 public case order must be complete and contiguous")
        if len({item.public_id for item in self.case_results}) != len(self.case_results):
            raise ValueError("R3 public ids must be unique")
        expected_catalog_hash = canonical_content_hash(
            {"case_hashes": tuple(item.case_content_hash for item in self.case_results)}
        )
        if self.catalog_content_hash != expected_catalog_hash:
            raise ValueError("audit catalog hash does not match case result provenance")
        object.__setattr__(self, "relation_failures", tuple(sorted(set(self.relation_failures))))
        object.__setattr__(self, "hard_failures", tuple(sorted(set(self.hard_failures))))
        object.__setattr__(self, "limitations", tuple(sorted(set(self.limitations))))
        if self.semantic_content_hash != self.expected_semantic_content_hash:
            raise ValueError("audit semantic_content_hash mismatch")
        if self.report_content_hash != self.expected_report_content_hash:
            raise ValueError("audit report_content_hash mismatch")

    @property
    def hard_passed(self) -> bool:
        return not self.hard_failures and not self.relation_failures and self.parity_passed

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
                "parity_passed": self.parity_passed,
                "hard_failures": self.hard_failures,
            }
        )

    @property
    def expected_report_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "semantic_content_hash": self.semantic_content_hash,
                "case_report_hashes": tuple(item.report_content_hash for item in self.case_results),
            }
        )


@dataclass(frozen=True, slots=True)
class SpatialPublicManifest:
    manifest_version: str
    simulation_only: bool
    hidden_used: bool
    git_head: str
    git_tree: str
    git_dirty: bool
    source_freeze_hash: str
    catalog_content_hash: str
    request_order: tuple[tuple[str, str], ...]
    max_workers_nonsemantic: int
    logical_cpu_count_nonsemantic: int
    semantic_content_hash: str
    content_hash: str

    def __post_init__(self) -> None:
        if self.manifest_version != SPATIAL_PUBLIC_MANIFEST_VERSION:
            raise ValueError("unsupported spatial public manifest version")
        if not self.simulation_only or self.hidden_used:
            raise ValueError("manifest must remain simulation-only and hidden-free")
        for name in (
            "source_freeze_hash",
            "catalog_content_hash",
            "semantic_content_hash",
            "content_hash",
        ):
            _require_sha256(getattr(self, name), name)
        if self.max_workers_nonsemantic <= 0 or self.logical_cpu_count_nonsemantic <= 0:
            raise ValueError("operational CPU settings must be positive")
        if not isinstance(self.git_dirty, bool):
            raise TypeError("git_dirty must be a bool")
        if len(self.request_order) != SPATIAL_PUBLIC_CASE_COUNT:
            raise ValueError("manifest requires the complete frozen request order")
        if len({public_id for public_id, _ in self.request_order}) != len(self.request_order):
            raise ValueError("manifest public ids must be unique")
        for public_id, request_hash in self.request_order:
            if not public_id:
                raise ValueError("manifest public id must not be empty")
            _require_sha256(request_hash, "manifest request hash")
        _require_git_object_id(self.git_head, "git_head")
        _require_git_object_id(self.git_tree, "git_tree")
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
                "request_order": self.request_order,
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


def public_spatial_cases() -> tuple[SpatialPublicCase, ...]:
    """동결된 21개 public request와 evaluator-only 기대값을 만든다."""

    wide_left = _open_straight_request("wide-straight-left", ManeuverSide.LEFT)
    wide_right = _open_straight_request("wide-straight-right", ManeuverSide.RIGHT)
    mirror_left = _open_straight_request("wide-mirror-left", ManeuverSide.LEFT, mirrored=True)
    mirror_right = _open_straight_request("wide-mirror-right", ManeuverSide.RIGHT, mirrored=True)
    narrow = _corridor_request("narrow-corridor", half_width_cells=12)
    narrow_door = _door_request("narrow-door", gap_cells=24)
    wide_door = _door_request("just-wide-door", gap_cells=30)
    dead_end = _door_request("dead-end", gap_cells=0)
    corner_safe = _corner_request("corner-safe", blocked=False)
    corner_blocked = _corner_request("corner-rotation-blocked", blocked=True)
    vertical_left = _vertical_request("vertical-left", ManeuverSide.LEFT)
    vertical_right = _vertical_request("vertical-right", ManeuverSide.RIGHT)
    forbidden = _forbidden_block_request()
    allowed = _allowed_pinch_request()
    start_unsafe = _endpoint_unsafe_request("start-unsafe", start=True)
    goal_unsafe = _endpoint_unsafe_request("goal-unsafe", start=False)
    resource_exact, resource_plus_one = _resource_boundary_requests()
    invalid = replace(wide_left, mission_revision=wide_left.mission_revision + 1)
    crossing_left, crossing_right = _crossing_projection_requests()

    cases = (
        SpatialPublicCase(
            0, "wide-straight-left", wide_left, SpatialOracleStatus.SPATIALLY_FEASIBLE
        ),
        SpatialPublicCase(
            1, "wide-straight-right", wide_right, SpatialOracleStatus.SPATIALLY_FEASIBLE
        ),
        SpatialPublicCase(
            2,
            "wide-mirror-left",
            mirror_left,
            SpatialOracleStatus.SPATIALLY_FEASIBLE,
            relation=SpatialPublicRelation.WIDE_MIRROR,
        ),
        SpatialPublicCase(
            3,
            "wide-mirror-right",
            mirror_right,
            SpatialOracleStatus.SPATIALLY_FEASIBLE,
            relation=SpatialPublicRelation.WIDE_MIRROR,
        ),
        SpatialPublicCase(
            4,
            "narrow-corridor",
            narrow,
            SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            "start_footprint_unsafe",
        ),
        SpatialPublicCase(
            5,
            "narrow-door",
            narrow_door,
            SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            "bounded_lattice_exhausted",
        ),
        SpatialPublicCase(6, "just-wide-door", wide_door, SpatialOracleStatus.SPATIALLY_FEASIBLE),
        SpatialPublicCase(
            7,
            "dead-end",
            dead_end,
            SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            "bounded_lattice_exhausted",
        ),
        SpatialPublicCase(
            8,
            "corner-safe",
            corner_safe,
            SpatialOracleStatus.SPATIALLY_FEASIBLE,
            limitations=("single_segment_corner_proxy",),
        ),
        SpatialPublicCase(
            9,
            "corner-rotation-blocked",
            corner_blocked,
            SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            "bounded_lattice_exhausted",
            limitations=("single_segment_corner_proxy",),
        ),
        SpatialPublicCase(
            10,
            "vertical-left",
            vertical_left,
            SpatialOracleStatus.SPATIALLY_FEASIBLE,
            relation=SpatialPublicRelation.VERTICAL_ROTATION,
        ),
        SpatialPublicCase(
            11,
            "vertical-right",
            vertical_right,
            SpatialOracleStatus.SPATIALLY_FEASIBLE,
            relation=SpatialPublicRelation.VERTICAL_ROTATION,
        ),
        SpatialPublicCase(
            12,
            "forbidden-only-block",
            forbidden,
            SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            "bounded_lattice_exhausted",
        ),
        SpatialPublicCase(
            13,
            "allowed-region-pinch",
            allowed,
            SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            "bounded_lattice_exhausted",
        ),
        SpatialPublicCase(
            14,
            "start-unsafe",
            start_unsafe,
            SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            "start_footprint_unsafe",
        ),
        SpatialPublicCase(
            15,
            "goal-unsafe",
            goal_unsafe,
            SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            "goal_footprint_unsafe",
        ),
        SpatialPublicCase(
            16,
            "resource-exact",
            resource_exact,
            SpatialOracleStatus.SPATIALLY_INFEASIBLE,
            "bounded_lattice_exhausted",
            relation=SpatialPublicRelation.RESOURCE_BOUNDARY,
        ),
        SpatialPublicCase(
            17,
            "resource-plus-one",
            resource_plus_one,
            SpatialOracleStatus.RESOURCE_LIMIT,
            "max_expanded_states",
            relation=SpatialPublicRelation.RESOURCE_BOUNDARY,
        ),
        SpatialPublicCase(
            18,
            "invalid-provenance",
            invalid,
            SpatialOracleStatus.INVALID_INPUT,
            "request_content_hash_mismatch",
        ),
        SpatialPublicCase(
            19,
            "crossing-static-left",
            crossing_left,
            SpatialOracleStatus.SPATIALLY_FEASIBLE,
            limitations=("actor_removed_static_projection",),
        ),
        SpatialPublicCase(
            20,
            "crossing-static-right",
            crossing_right,
            SpatialOracleStatus.SPATIALLY_FEASIBLE,
            limitations=("actor_removed_static_projection",),
        ),
    )
    if tuple(case.ordinal for case in cases) != tuple(range(SPATIAL_PUBLIC_CASE_COUNT)):
        raise RuntimeError("spatial public catalog ordinal invariant failed")
    if len({case.public_id for case in cases}) != len(cases):
        raise RuntimeError("spatial public catalog id invariant failed")
    return cases


def evaluate_spatial_public_case(case: SpatialPublicCase) -> SpatialPublicCaseResult:
    result = search_bounded_spatial_oracle(case.request)
    failures: list[str] = []
    if result.status is not case.expected_status:
        failures.append(f"status_mismatch:{case.expected_status.value}:{result.status.value}")
    if case.expected_reason is not None and result.termination_reason != case.expected_reason:
        failures.append(f"reason_mismatch:{case.expected_reason}:{result.termination_reason}")
    if result.status is SpatialOracleStatus.SPATIALLY_FEASIBLE and (
        result.validation is None or not result.validation.passed
    ):
        failures.append("feasible_without_independent_validation")
    if result.status is SpatialOracleStatus.SPATIALLY_INFEASIBLE and (
        not result.exhaustive
        and result.termination_reason
        not in {"start_footprint_unsafe", "goal_footprint_unsafe", "analytic_cross_section_blocked"}
    ):
        failures.append("infeasible_without_exhaustive_or_analytic_proof")
    report_hash = canonical_content_hash(
        {
            "case_hash": case.semantic_content_hash,
            "result_hash": result.semantic_content_hash,
            "hard_failures": tuple(sorted(failures)),
        }
    )
    return SpatialPublicCaseResult(
        ordinal=case.ordinal,
        public_id=case.public_id,
        case_content_hash=case.semantic_content_hash,
        request=case.request,
        expected_status=case.expected_status,
        expected_reason=case.expected_reason,
        relation=case.relation,
        result=result,
        hard_failures=tuple(failures),
        report_content_hash=report_hash,
    )


def audit_spatial_public_catalog(
    *,
    max_workers: int,
    on_case: Callable[[SpatialPublicCaseResult], None] | None = None,
) -> SpatialPublicAudit:
    if isinstance(max_workers, bool) or not isinstance(max_workers, int) or max_workers <= 0:
        raise ValueError("max_workers must be a positive exact integer")
    cases = public_spatial_cases()
    started = perf_counter_ns()
    results = evaluate_spatial_public_cases(
        cases,
        max_workers=max_workers,
        on_case=on_case,
    )

    parity_case = cases[0]
    parity_result = evaluate_spatial_public_case(parity_case)
    parity_passed = (
        parity_result.result.semantic_content_hash
        == results[parity_case.ordinal].result.semantic_content_hash
    )
    relation_failures = _relation_failures(results)
    hard_failures = [
        f"{item.public_id}:{failure}" for item in results for failure in item.hard_failures
    ]
    hard_failures.extend(relation_failures)
    if not parity_passed:
        hard_failures.append("serial_process_semantic_parity_failed")
    catalog_hash = canonical_content_hash(
        {"case_hashes": tuple(case.semantic_content_hash for case in cases)}
    )
    semantic_hash = canonical_content_hash(
        {
            "report_version": SPATIAL_PUBLIC_REPORT_VERSION,
            "catalog_content_hash": catalog_hash,
            "case_result_hashes": tuple(item.semantic_content_hash for item in results),
            "relation_failures": tuple(sorted(relation_failures)),
            "parity_case_id": parity_case.public_id,
            "parity_passed": parity_passed,
            "hard_failures": tuple(sorted(hard_failures)),
        }
    )
    report_hash = canonical_content_hash(
        {
            "semantic_content_hash": semantic_hash,
            "case_report_hashes": tuple(item.report_content_hash for item in results),
        }
    )
    return SpatialPublicAudit(
        report_version=SPATIAL_PUBLIC_REPORT_VERSION,
        simulation_only=True,
        hidden_used=False,
        catalog_content_hash=catalog_hash,
        case_results=results,
        relation_failures=tuple(relation_failures),
        parity_case_id=parity_case.public_id,
        parity_passed=parity_passed,
        hard_failures=tuple(hard_failures),
        limitations=(
            "offline_static_bounded_lattice_only",
            "abstract_anchor_connector_not_a_chassis_primitive",
            "python_wall_clock_is_nonqualification",
            "multi_segment_corner_projection_not_implemented",
            "no_product_or_human_safety_claim",
        ),
        semantic_content_hash=semantic_hash,
        report_content_hash=report_hash,
        elapsed_nonqualification_ns=perf_counter_ns() - started,
    )


def evaluate_spatial_public_cases(
    cases: tuple[SpatialPublicCase, ...],
    *,
    max_workers: int,
    on_case: Callable[[SpatialPublicCaseResult], None] | None = None,
) -> tuple[SpatialPublicCaseResult, ...]:
    """독립 case를 process 병렬화하고 항상 input ordinal 순서로 반환한다."""

    if isinstance(max_workers, bool) or not isinstance(max_workers, int) or max_workers <= 0:
        raise ValueError("max_workers must be a positive exact integer")
    if len({case.ordinal for case in cases}) != len(cases):
        raise ValueError("case ordinals must be unique")
    if max_workers == 1:
        materialized: list[SpatialPublicCaseResult] = []
        for case in cases:
            result = evaluate_spatial_public_case(case)
            materialized.append(result)
            if on_case is not None:
                on_case(result)
        results = tuple(materialized)
    else:
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            futures = {
                executor.submit(evaluate_spatial_public_case, case): case.ordinal for case in cases
            }
            materialized = []
            for future in as_completed(futures):
                result = future.result()
                materialized.append(result)
                if on_case is not None:
                    on_case(result)
            results = tuple(materialized)
    return tuple(sorted(results, key=lambda item: item.ordinal))


def build_spatial_public_manifest(
    *, repository_root: Path, max_workers: int
) -> SpatialPublicManifest:
    cases = public_spatial_cases()
    catalog_hash = canonical_content_hash(
        {"case_hashes": tuple(case.semantic_content_hash for case in cases)}
    )
    head = _git_output(repository_root, "rev-parse", "HEAD")
    tree = _git_output(repository_root, "rev-parse", "HEAD^{tree}")
    dirty = bool(_git_output(repository_root, "status", "--porcelain=v1"))
    source_hash = _source_freeze_hash(repository_root)
    order = tuple((case.public_id, case.request.request_content_hash) for case in cases)
    semantic_hash = canonical_content_hash(
        {
            "manifest_version": SPATIAL_PUBLIC_MANIFEST_VERSION,
            "simulation_only": True,
            "hidden_used": False,
            "source_freeze_hash": source_hash,
            "catalog_content_hash": catalog_hash,
            "request_order": order,
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
    return SpatialPublicManifest(
        manifest_version=SPATIAL_PUBLIC_MANIFEST_VERSION,
        simulation_only=True,
        hidden_used=False,
        git_head=head,
        git_tree=tree,
        git_dirty=dirty,
        source_freeze_hash=source_hash,
        catalog_content_hash=catalog_hash,
        request_order=order,
        max_workers_nonsemantic=max_workers,
        logical_cpu_count_nonsemantic=os.cpu_count() or 1,
        semantic_content_hash=semantic_hash,
        content_hash=content_hash,
    )


class SpatialPublicOutputWriter:
    def __init__(
        self, output_dir: Path, manifest: SpatialPublicManifest, *, repository_root: Path
    ) -> None:
        self.output_dir = Path(output_dir)
        self.manifest = manifest
        self.repository_root = Path(repository_root)
        self._written: set[tuple[str, str]] = set()

    def start(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=False)
        _write_exclusive_json(self.output_dir / "run-manifest.json", self.manifest)
        _write_atomic_json(
            self.output_dir / "run_state.incomplete.json",
            {
                "partial": True,
                "manifest_content_hash": self.manifest.content_hash,
                "completed_public_ids": (),
            },
        )

    def write_case(self, result: SpatialPublicCaseResult) -> None:
        key = (result.public_id, result.request.request_content_hash)
        if key not in self.manifest.request_order:
            raise ValueError("case is not part of the frozen manifest")
        if key in self._written:
            raise FileExistsError("case was already written")
        directory = (
            self.output_dir
            / "requests"
            / f"{result.ordinal:02d}-{result.request.request_content_hash[:12]}"
        )
        directory.mkdir(parents=True, exist_ok=False)
        _write_exclusive_json(directory / "request.json", _request_payload(result.request))
        _write_exclusive_json(directory / "result.json", result.result)
        _write_exclusive_json(directory / "validation.json", result.result.validation)
        _write_exclusive_json(directory / "assessment.json", result)
        _save_spatial_plot(result, directory / "path.png")
        self._written.add(key)
        completed = tuple(
            sorted(
                path.parent.name
                for path in (self.output_dir / "requests").glob("*/assessment.json")
            )
        )
        _write_atomic_json(
            self.output_dir / "run_state.incomplete.json",
            {
                "partial": True,
                "manifest_content_hash": self.manifest.content_hash,
                "completed_request_directories": completed,
            },
        )

    def complete(self, audit: SpatialPublicAudit) -> tuple[Path, Path, Path | None]:
        expected = self.manifest.request_order
        actual = tuple(
            (item.public_id, item.request.request_content_hash) for item in audit.case_results
        )
        if actual != expected:
            raise ValueError("public audit does not match the frozen manifest request order")
        if self._written != set(expected):
            raise RuntimeError("cannot complete before every manifest case artifact is written")
        if audit.catalog_content_hash != self.manifest.catalog_content_hash:
            raise ValueError("public audit catalog hash mismatch")
        if _source_freeze_hash(self.repository_root) != self.manifest.source_freeze_hash:
            raise RuntimeError("source changed before spatial public completion")
        results_path = self.output_dir / "spatial-public-results.json"
        summary_path = self.output_dir / "summary.md"
        complete_path = self.output_dir / "run_state.complete.json"
        _write_exclusive_json(results_path, audit)
        _write_exclusive_text(summary_path, _audit_summary(audit))
        _write_exclusive_json(
            complete_path,
            {
                "partial": False,
                "manifest_content_hash": self.manifest.content_hash,
                "audit_semantic_content_hash": audit.semantic_content_hash,
                "hard_passed": audit.hard_passed,
                "case_count": len(audit.case_results),
            },
        )
        incomplete = self.output_dir / "run_state.incomplete.json"
        incomplete.unlink()
        receipt_path: Path | None = None
        if audit.hard_passed and not self.manifest.git_dirty:
            self._verify_git_state()
            receipt_path = self.output_dir / "qualification-receipt.json"
            receipt = {
                "receipt_version": SPATIAL_PUBLIC_RECEIPT_VERSION,
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
            _write_exclusive_json(
                receipt_path,
                receipt,
            )
        return results_path, summary_path, receipt_path

    def _verify_git_state(self) -> None:
        if _git_output(self.repository_root, "rev-parse", "HEAD") != self.manifest.git_head:
            raise RuntimeError("Git HEAD changed before spatial public receipt")
        if _git_output(self.repository_root, "rev-parse", "HEAD^{tree}") != self.manifest.git_tree:
            raise RuntimeError("Git tree changed before spatial public receipt")
        if _git_output(self.repository_root, "status", "--porcelain=v1"):
            raise RuntimeError("Git worktree became dirty before spatial public receipt")


def _relation_failures(results: tuple[SpatialPublicCaseResult, ...]) -> tuple[str, ...]:
    by_id = {item.public_id: item for item in results}
    failures: list[str] = []
    for left_id, right_id, label in (
        ("wide-straight-left", "wide-mirror-right", "wide_left_mirror"),
        ("wide-straight-right", "wide-mirror-left", "wide_right_mirror"),
        ("wide-straight-left", "vertical-left", "vertical_left"),
        ("wide-straight-right", "vertical-right", "vertical_right"),
    ):
        first = by_id[left_id].result
        second = by_id[right_id].result
        if first.status is not second.status:
            failures.append(f"{label}:status")
            continue
        if first.status is not SpatialOracleStatus.SPATIALLY_FEASIBLE:
            continue
        if first.rotation_count != second.rotation_count:
            failures.append(f"{label}:rotation_count")
        for field in ("path_length_m", "minimum_clearance_m"):
            a = getattr(first, field)
            b = getattr(second, field)
            if (
                a is None
                or b is None
                or abs(a - b) > SPATIAL_PUBLIC_RESOLUTION_M + _RELATION_TOLERANCE
            ):
                failures.append(f"{label}:{field}")
    exact = by_id["resource-exact"].result
    limited = by_id["resource-plus-one"].result
    if exact.status is not SpatialOracleStatus.SPATIALLY_INFEASIBLE or not exact.exhaustive:
        failures.append("resource_boundary:exact_not_exhaustive")
    if limited.status is not SpatialOracleStatus.RESOURCE_LIMIT or limited.exhaustive:
        failures.append("resource_boundary:plus_one_not_resource")
    return tuple(failures)


def _open_straight_request(
    case_id: str, side: ManeuverSide, *, mirrored: bool = False
) -> BoundedSpatialOracleRequest:
    grid = GridMap(np.zeros((88, 104), dtype=np.bool_), SPATIAL_PUBLIC_RESOLUTION_M)
    start_cell = (26, 44)
    goal_cell = (77, 44)
    if mirrored:
        start_cell = (grid.width - 1 - start_cell[0], start_cell[1])
        goal_cell = (grid.width - 1 - goal_cell[0], goal_cell[1])
    yaw = pi if mirrored else 0.0
    start_base = grid.cell_to_pose(start_cell)
    goal_base = grid.cell_to_pose(goal_cell)
    start = Pose2D(start_base.x, start_base.y, yaw)
    goal = Pose2D(goal_base.x, goal_base.y, yaw)
    return _make_request(case_id, grid, start, goal, side=side)


def _vertical_request(case_id: str, side: ManeuverSide) -> BoundedSpatialOracleRequest:
    grid = GridMap(np.zeros((104, 88), dtype=np.bool_), SPATIAL_PUBLIC_RESOLUTION_M)
    start_base = grid.cell_to_pose((44, 26))
    goal_base = grid.cell_to_pose((44, 77))
    return _make_request(
        case_id,
        grid,
        Pose2D(start_base.x, start_base.y, pi / 2.0),
        Pose2D(goal_base.x, goal_base.y, pi / 2.0),
        side=side,
    )


def _corridor_request(case_id: str, *, half_width_cells: int) -> BoundedSpatialOracleRequest:
    occupancy = np.zeros((88, 104), dtype=np.bool_)
    center_y = 44
    occupancy[: center_y - half_width_cells, :] = True
    occupancy[center_y + half_width_cells + 1 :, :] = True
    grid = GridMap(occupancy, SPATIAL_PUBLIC_RESOLUTION_M)
    start = grid.cell_to_pose((26, center_y))
    goal = grid.cell_to_pose((77, center_y))
    return _make_request(case_id, grid, start, goal, side=ManeuverSide.UNSPECIFIED)


def _door_request(case_id: str, *, gap_cells: int) -> BoundedSpatialOracleRequest:
    occupancy = np.zeros((88, 104), dtype=np.bool_)
    wall_x = 52
    occupancy[:, wall_x : wall_x + 2] = True
    if gap_cells:
        low = 44 - gap_cells // 2
        occupancy[low : low + gap_cells, wall_x : wall_x + 2] = False
    grid = GridMap(occupancy, SPATIAL_PUBLIC_RESOLUTION_M)
    start = grid.cell_to_pose((26, 44))
    goal = grid.cell_to_pose((77, 44))
    return _make_request(case_id, grid, start, goal, side=ManeuverSide.UNSPECIFIED)


def _corner_request(case_id: str, *, blocked: bool) -> BoundedSpatialOracleRequest:
    occupancy = np.zeros((96, 96), dtype=np.bool_)
    if blocked:
        occupancy[:, :] = True
        occupancy[22:49, 10:56] = False
        occupancy[35:86, 42:69] = False
    grid = GridMap(occupancy, SPATIAL_PUBLIC_RESOLUTION_M)
    start_cell = (28, 48) if not blocked else (28, 35)
    goal_cell = (52, 72) if not blocked else (55, 65)
    start_base = grid.cell_to_pose(start_cell)
    goal_base = grid.cell_to_pose(goal_cell)
    return _make_request(
        case_id,
        grid,
        Pose2D(start_base.x, start_base.y, 0.0),
        Pose2D(goal_base.x, goal_base.y, pi / 2.0),
        side=ManeuverSide.UNSPECIFIED,
    )


def _forbidden_block_request() -> BoundedSpatialOracleRequest:
    base = _open_straight_request("forbidden-only-block", ManeuverSide.UNSPECIFIED)
    forbidden = tuple((x, y) for y in range(8, 80) for x in range(50, 54))
    return _clone_request(base, forbidden_cells=forbidden, source_id="forbidden-only-block")


def _allowed_pinch_request() -> BoundedSpatialOracleRequest:
    base = _open_straight_request("allowed-region-pinch", ManeuverSide.UNSPECIFIED)
    center_y = base.static_grid.world_to_cell(base.start_pose)[1]
    left_room = {(x, y) for y in range(8, base.static_grid.height - 8) for x in range(8, 46)}
    right_room = {
        (x, y)
        for y in range(8, base.static_grid.height - 8)
        for x in range(59, base.static_grid.width - 8)
    }
    narrow_connector = {(x, y) for y in range(center_y - 12, center_y + 13) for x in range(46, 59)}
    allowed_cells = tuple(sorted(left_room | narrow_connector | right_room))
    return _clone_request(
        base,
        allowed_region=SpatialAllowedRegion(allowed_cells, unrestricted=False),
        source_id="allowed-region-pinch",
    )


def _endpoint_unsafe_request(case_id: str, *, start: bool) -> BoundedSpatialOracleRequest:
    base = _open_straight_request(case_id, ManeuverSide.LEFT)
    occupancy = base.static_grid.occupancy.copy()
    pose = base.start_pose if start else base.rejoin_goal.pose
    x, y = base.static_grid.world_to_cell(pose)
    occupancy[y, x] = True
    grid = GridMap(
        occupancy,
        base.static_grid.resolution_m,
        base.static_grid.origin_x_m,
        base.static_grid.origin_y_m,
    )
    return _clone_request(base, static_grid=grid, source_id=case_id)


def _resource_boundary_requests() -> tuple[
    BoundedSpatialOracleRequest, BoundedSpatialOracleRequest
]:
    exact_config = SpatialLatticeConfig(
        max_expanded_states=_RESOURCE_EXACT_EXPANDED,
        max_generated_edges=_RESOURCE_EXACT_GENERATED,
        max_open_states=_RESOURCE_EXACT_OPEN,
    )
    exact = _small_resource_request(exact_config)
    limited_config = replace(exact_config, max_expanded_states=exact_config.max_expanded_states - 1)
    limited = _small_resource_request(limited_config)
    return exact, limited


def _small_resource_request(config: SpatialLatticeConfig) -> BoundedSpatialOracleRequest:
    occupancy = np.zeros((70, 100), dtype=np.bool_)
    occupancy[:, 49:52] = True
    grid = GridMap(occupancy, SPATIAL_PUBLIC_RESOLUTION_M)
    return _make_request(
        "resource-boundary",
        grid,
        grid.cell_to_pose((24, 35)),
        grid.cell_to_pose((75, 35)),
        side=ManeuverSide.LEFT,
        config=config,
        region_margin_cells=20,
    )


def _crossing_projection_requests() -> tuple[
    BoundedSpatialOracleRequest, BoundedSpatialOracleRequest
]:
    episode = next(
        item
        for item in generate_dynamic_corpus()
        if item.split is DynamicCorpusSplit.GOLDEN
        and item.episode_id == "golden-local_detour_feasible-00-20260812"
    )
    world = project_public_witness_world(episode)
    return (
        project_witness_world_to_spatial_request(world, maneuver_side=ManeuverSide.LEFT),
        project_witness_world_to_spatial_request(world, maneuver_side=ManeuverSide.RIGHT),
    )


def _make_request(
    case_id: str,
    grid: GridMap,
    start: Pose2D,
    goal: Pose2D,
    *,
    side: ManeuverSide,
    config: SpatialLatticeConfig | None = None,
    forbidden_cells: tuple[tuple[int, int], ...] = (),
    allowed_region: SpatialAllowedRegion | None = None,
    region_margin_cells: int = 18,
) -> BoundedSpatialOracleRequest:
    start_cell = grid.world_to_cell(start)
    goal_cell = grid.world_to_cell(goal)
    min_x = max(0, min(start_cell[0], goal_cell[0]) - region_margin_cells)
    max_x = min(grid.width - 1, max(start_cell[0], goal_cell[0]) + region_margin_cells)
    min_y = max(0, min(start_cell[1], goal_cell[1]) - region_margin_cells)
    max_y = min(grid.height - 1, max(start_cell[1], goal_cell[1]) + region_margin_cells)
    region = SpatialSearchRegion(
        tuple((x, y) for y in range(min_y, max_y + 1) for x in range(min_x, max_x + 1))
    )
    projection_hash = canonical_content_hash(
        {
            "catalog_version": SPATIAL_PUBLIC_CATALOG_VERSION,
            "case_id": case_id,
            "grid_shape": (grid.height, grid.width),
            "grid_occupancy": np.argwhere(grid.occupancy).tolist(),
            "start": start,
            "goal": goal,
            "side": side,
            "forbidden_cells": forbidden_cells,
            "allowed_region": allowed_region or SpatialAllowedRegion(),
            "search_region": region,
        }
    )
    return build_bounded_spatial_request(
        schema_version=SPATIAL_ORACLE_SCHEMA_VERSION,
        map_id=f"r3-public-{case_id}",
        map_revision=1,
        mission_revision=1,
        static_grid=grid,
        forbidden_cells=forbidden_cells,
        allowed_region=allowed_region or SpatialAllowedRegion(),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        start_pose=start,
        rejoin_goal=SpatialRejoinGoal(goal),
        reference_segment=SpatialReferenceSegment(start, goal),
        maneuver_side=side,
        search_region=region,
        lattice_config=config or SpatialLatticeConfig(),
        source_projection_hash=projection_hash,
    )


def _clone_request(
    base: BoundedSpatialOracleRequest,
    *,
    static_grid: GridMap | None = None,
    forbidden_cells: tuple[tuple[int, int], ...] | None = None,
    allowed_region: SpatialAllowedRegion | None = None,
    source_id: str,
) -> BoundedSpatialOracleRequest:
    grid = static_grid or base.static_grid
    return build_bounded_spatial_request(
        schema_version=base.schema_version,
        map_id=f"r3-public-{source_id}",
        map_revision=base.map_revision,
        mission_revision=base.mission_revision,
        static_grid=grid,
        forbidden_cells=base.forbidden_cells if forbidden_cells is None else forbidden_cells,
        allowed_region=base.allowed_region if allowed_region is None else allowed_region,
        vehicle_profile=base.vehicle_profile,
        start_pose=base.start_pose,
        rejoin_goal=base.rejoin_goal,
        reference_segment=base.reference_segment,
        maneuver_side=base.maneuver_side,
        search_region=base.search_region,
        lattice_config=base.lattice_config,
        source_projection_hash=canonical_content_hash(
            {
                "catalog_version": SPATIAL_PUBLIC_CATALOG_VERSION,
                "source_id": source_id,
                "base_projection_hash": base.source_projection_hash,
                "grid_hash": canonical_content_hash(np.argwhere(grid.occupancy).tolist()),
                "forbidden_cells": base.forbidden_cells
                if forbidden_cells is None
                else forbidden_cells,
                "allowed_region": base.allowed_region if allowed_region is None else allowed_region,
            }
        ),
    )


def _request_payload(request: BoundedSpatialOracleRequest) -> dict[str, object]:
    return {
        "schema_version": request.schema_version,
        "map_id": request.map_id,
        "map_revision": request.map_revision,
        "mission_revision": request.mission_revision,
        "grid": {
            "width": request.static_grid.width,
            "height": request.static_grid.height,
            "resolution_m": request.static_grid.resolution_m,
            "origin_x_m": request.static_grid.origin_x_m,
            "origin_y_m": request.static_grid.origin_y_m,
            "occupied_cells": tuple(
                (int(x), int(y)) for y, x in np.argwhere(request.static_grid.occupancy)
            ),
            "content_hash": request.grid_content_hash,
        },
        "forbidden_cells": request.forbidden_cells,
        "allowed_region": request.allowed_region,
        "vehicle_profile": request.vehicle_profile,
        "start_pose": request.start_pose,
        "rejoin_goal": request.rejoin_goal,
        "reference_segment": request.reference_segment,
        "maneuver_side": request.maneuver_side,
        "search_region": request.search_region,
        "lattice_config": request.lattice_config,
        "source_projection_hash": request.source_projection_hash,
        "request_content_hash": request.request_content_hash,
    }


def _save_spatial_plot(result: SpatialPublicCaseResult, output_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    request = result.request
    grid = request.static_grid
    figure, axis = plt.subplots(figsize=(8.0, 6.0))
    try:
        rgba = np.zeros((grid.height, grid.width, 4), dtype=float)
        if not request.allowed_region.unrestricted:
            allowed = set(request.allowed_region.cells)
            for y in range(grid.height):
                for x in range(grid.width):
                    if (x, y) not in allowed:
                        rgba[y, x] = (0.95, 0.65, 0.10, 0.18)
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
        reference = request.reference_segment
        axis.plot(
            (reference.start.x, reference.end.x),
            (reference.start.y, reference.end.y),
            "--",
            color="tab:blue",
            linewidth=1.8,
            label="reference",
        )
        if result.result.path:
            axis.plot(
                [pose.x for pose in result.result.path],
                [pose.y for pose in result.result.path],
                color="tab:green",
                linewidth=2.0,
                label="validated spatial path",
            )
        axis.scatter(
            (request.start_pose.x, request.rejoin_goal.pose.x),
            (request.start_pose.y, request.rejoin_goal.pose.y),
            c=("black", "tab:red"),
            s=40,
            label="start / goal",
        )
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlim(extent[0], extent[1])
        axis.set_ylim(extent[2], extent[3])
        axis.grid(alpha=0.15)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        axis.set_title(
            f"R3 bounded spatial oracle\n{result.public_id} | "
            f"{result.result.status.value} | hard={'PASS' if result.hard_passed else 'FAIL'}"
        )
        axis.legend(fontsize=8, loc="best")
        figure.tight_layout()
        figure.savefig(output_path, dpi=160, format="png")
    finally:
        plt.close(figure)


def _audit_summary(audit: SpatialPublicAudit) -> str:
    lines = [
        "# R3 bounded 공간 Oracle 공개 qualification",
        "",
        f"- hard 판정: `{'PASS' if audit.hard_passed else 'FAIL'}`",
        f"- 공개 request: `{len(audit.case_results)}`",
        f"- hidden 사용: `{str(audit.hidden_used).lower()}`",
        f"- serial/process parity: `{'PASS' if audit.parity_passed else 'FAIL'}`",
        f"- semantic hash: `{audit.semantic_content_hash}`",
        "- Python wall-clock과 worker 수는 판정 근거가 아니다.",
        "",
        "| id | expected | actual | reason | expanded | hard |",
        "|---|---|---|---|---:|---|",
    ]
    for item in audit.case_results:
        lines.append(
            f"| `{item.public_id}` | {item.expected_status.value} | "
            f"{item.result.status.value} | {item.result.termination_reason} | "
            f"{item.result.expanded_states} | {'PASS' if item.hard_passed else 'FAIL'} |"
        )
    lines.extend(("", "## 해석 제한", ""))
    lines.extend(f"- `{value}`" for value in audit.limitations)
    lines.extend(
        (
            "",
            "이 결과는 Actor를 제거한 동결 정적 지도와 8-heading bounded lattice의 offline "
            "공간 연구 증거다. online controller, 제품 알고리즘 채택, 실제 사람 탑승 안전성, "
            "G1~G5 또는 경로 분석 7단계를 결정하지 않는다.",
            "",
        )
    )
    return "\n".join(lines)


def _source_freeze_hash(repository_root: Path) -> str:
    lab = repository_root / "simulation" / "path_planning_lab"
    paths = tuple(sorted((lab / "src" / "hospital_path_lab").rglob("*.py"))) + (
        lab / "scripts" / "run_spatial_oracle_public.py",
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
    if isinstance(value, np.ndarray):
        return value.tolist()
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
    "SPATIAL_PUBLIC_CASE_COUNT",
    "SPATIAL_PUBLIC_CATALOG_VERSION",
    "SpatialPublicAudit",
    "SpatialPublicCase",
    "SpatialPublicCaseResult",
    "SpatialPublicManifest",
    "SpatialPublicOutputWriter",
    "SpatialPublicRelation",
    "audit_spatial_public_catalog",
    "build_spatial_public_manifest",
    "evaluate_spatial_public_case",
    "evaluate_spatial_public_cases",
    "public_spatial_cases",
]
