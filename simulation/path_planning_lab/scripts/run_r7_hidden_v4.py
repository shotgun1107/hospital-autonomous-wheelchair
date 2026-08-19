"""Preflight and run the one-use R7 hidden-v4 observation study."""

from __future__ import annotations

import argparse
import json
import os
import platform
import secrets
import subprocess
from collections.abc import Sequence
from dataclasses import asdict
from datetime import UTC, datetime
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile

from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r7_hidden_v4_qualification import (
    R7_HIDDEN_V4_OBSERVATION_VERSION,
    audit_hidden_v4_results,
    build_hidden_v4_case_specs,
    evaluate_hidden_v4_cases,
    hidden_v4_seed_commitment,
)

_PREFLIGHT_SCHEMA = "r7-hidden-v4-preflight-v1"
_SEED_COMMITMENT_SCHEMA = "r7-hidden-v4-seed-commitment-v1"
_CONSUMED_SEED_SCHEMA = "r7-hidden-v4-consumed-seed-v1"
_PARTIAL_SCHEMA = "r7-hidden-v4-partial-v1"
_SUMMARY_SCHEMA = "r7-hidden-v4-summary-v1"
_RECEIPT_SCHEMA = "r7-hidden-v4-consumption-receipt-v1"
_REQUIRED_EVIDENCE_FILES = (
    "run-manifest.json",
    "semantic-parity.json",
    "contract-parity.json",
    "timing-qualification.json",
    "release-gate.json",
    "qualification-receipt.json",
    "summary.md",
)
_PACKAGING_PATHS = (
    "simulation/path_planning_lab/scripts/run_r7_hidden_v4.py",
    "simulation/path_planning_lab/src/hospital_path_lab/"
    "r7_hidden_v4_qualification.py",
)
_KNOWN_CONSUMED_ROOT_SEEDS = frozenset(
    {
        # The sealed 2026-08-19 FAIL_ANALYZED v4 execution.  Keep this guard
        # even though corrective work now uses the separate v5 runner.
        6_564_067_906_066_881_700,
        8_488_859_258_265_267_075,
        5_041_993_867_976_238_990,
        8_164_808_726_104_920_337,
    }
)
_KNOWN_PUBLIC_OBSERVATION_SEEDS = frozenset(
    {
        2_140_928_701_629_245_82,
        4_097_001_075_006_799_098,
        6_422_064_046_178_126_625,
        8_970_341_022_568_507_592,
        1_993_037_174_228_324_916,
        4_525_333_994_236_990_214,
    }
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--qualification-evidence", type=Path, required=True)
    parser.add_argument("--qualification-sha256", required=True)
    parser.add_argument(
        "--max-workers",
        type=int,
        default=min(14, max(1, (os.cpu_count() or 2) // 2)),
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Validate the release package without generating a hidden seed.",
    )
    parser.add_argument(
        "--execute-approved",
        action="store_true",
        help="Required acknowledgement for the one-use hidden execution.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.max_workers <= 0:
        parser.error("max-workers must be positive")
    expected_sha = args.qualification_sha256.lower()
    if len(expected_sha) != 64 or any(ch not in "0123456789abcdef" for ch in expected_sha):
        parser.error("qualification-sha256 must be a lowercase 64-character SHA-256")
    if not args.preflight_only and not args.execute_approved:
        parser.error("hidden execution requires --execute-approved")

    repository_root = _repository_root()
    output = args.output.resolve()
    if output.exists():
        parser.error("output path already exists; hidden outputs are never overwritten")
    if _git(repository_root, "status", "--porcelain=v1"):
        parser.error("hidden-v4 requires a clean Git working tree")

    preflight = _preflight(
        repository_root,
        args.qualification_evidence.resolve(),
        expected_sha,
    )
    if args.preflight_only:
        output.mkdir(parents=True)
        preflight_receipt = _build_preflight_receipt(
            preflight,
            max_workers=args.max_workers,
        )
        _write_json(output / "preflight-receipt.json", preflight_receipt)
        print("preflight_passed=true", flush=True)
        print("hidden_seed_generated=false", flush=True)
        return 0

    ledger_path = _acquire_consumption_ledger(repository_root, output, preflight)
    output.mkdir(parents=True)
    preflight_receipt = _build_preflight_receipt(
        preflight,
        max_workers=args.max_workers,
    )
    _write_json(output / "preflight-receipt.json", preflight_receipt)
    root_seed = secrets.randbits(63)
    if root_seed in _KNOWN_CONSUMED_ROOT_SEEDS:
        error = RuntimeError("generated root seed was already consumed")
        receipt_hash = _record_infrastructure_failure(output, None, 0, error)
        _seal_consumption_ledger(
            ledger_path,
            status="infrastructure_failure",
            receipt_content_hash=receipt_hash,
        )
        raise error
    commitment = hidden_v4_seed_commitment(root_seed)
    try:
        started_at, specs, pre_run = _prepare_committed_hidden_v4(
            output=output,
            ledger_path=ledger_path,
            root_seed=root_seed,
            commitment=commitment,
            preflight=preflight,
            max_workers=args.max_workers,
        )
    except BaseException as exc:
        receipt_hash = _record_infrastructure_failure(output, commitment, 0, exc)
        _seal_consumption_ledger(
            ledger_path,
            status="infrastructure_failure",
            seed_commitment=commitment,
            receipt_content_hash=receipt_hash,
        )
        raise
    partial = []

    def on_case(result) -> None:
        partial.append(result)
        _write_json(
            output / "partial-state.json",
            {
                "schema": _PARTIAL_SCHEMA,
                "seed_commitment": commitment,
                "completed_case_count": len(partial),
                "completed_case_ids": tuple(item.case_id for item in partial),
                "partial_is_final_evidence": False,
            },
        )
        print(
            f"case_complete={len(partial)}/{len(specs)}:{result.case_id}:"
            f"outcome={result.outcome}:passed={str(result.passed).lower()}",
            flush=True,
        )

    try:
        results = evaluate_hidden_v4_cases(
            repository_root,
            specs,
            max_workers=args.max_workers,
            on_case=on_case,
            failure_trace_root=output / "failure-traces",
        )
        postflight = _verify_execution_freeze(repository_root, preflight)
    except BaseException as exc:
        receipt_hash = _record_infrastructure_failure(
            output,
            commitment,
            len(partial),
            exc,
        )
        _seal_consumption_ledger(
            ledger_path,
            status="infrastructure_failure",
            seed_commitment=commitment,
            receipt_content_hash=receipt_hash,
        )
        raise

    try:
        return _finalize_hidden_v4(
            output=output,
            specs=specs,
            results=results,
            postflight=postflight,
            commitment=commitment,
            preflight=preflight,
            expected_sha=expected_sha,
            pre_run=pre_run,
            preflight_receipt=preflight_receipt,
            ledger_path=ledger_path,
        )
    except BaseException as exc:
        receipt_hash = _record_infrastructure_failure(
            output,
            commitment,
            len(partial),
            exc,
        )
        _seal_consumption_ledger(
            ledger_path,
            status="infrastructure_failure",
            seed_commitment=commitment,
            receipt_content_hash=receipt_hash,
        )
        raise


def _finalize_hidden_v4(
    *,
    output: Path,
    specs: Sequence[object],
    results: Sequence[object],
    postflight: dict[str, object],
    commitment: str,
    preflight: dict[str, object],
    expected_sha: str,
    pre_run: dict[str, object],
    preflight_receipt: dict[str, object],
    ledger_path: Path,
) -> int:
    audit = audit_hidden_v4_results(tuple(specs), tuple(results))
    failure_trace_manifest = _failure_trace_manifest(
        output / "failure-traces",
        results=results,
    )
    case_trace_set_hash = _case_trace_set_hash(results)
    _write_json(output / "case-results.json", results)
    _write_json(output / "failure-trace-manifest.json", failure_trace_manifest)
    summary = {
        "schema": _SUMMARY_SCHEMA,
        "final_status": "PASS_FINAL" if audit.passed else "FAIL_REQUIRES_ANALYSIS",
        "passed": audit.passed,
        "case_count": audit.result_count,
        "normal_completed_count": audit.normal_completed_count,
        "stress_conditionally_safe_count": audit.stress_conditionally_safe_count,
        "stress_release_count": audit.stress_release_count,
        "hard_failure_count": audit.hard_failure_count,
        "release_contract_violation_count": audit.release_contract_violation_count,
        "duplicate_safe_frame_violation_count": (
            audit.duplicate_safe_frame_violation_count
        ),
        "stale_propulsion_violation_count": audit.stale_propulsion_violation_count,
        "unauthorized_restart_count": audit.unauthorized_restart_count,
        "actual_collision_count": audit.actual_collision_count,
        "actual_forbidden_violation_count": audit.actual_forbidden_violation_count,
        "actual_clearance_violation_count": audit.actual_clearance_violation_count,
        "failures": audit.failures,
        "result_set_hash": audit.result_set_hash,
        "seed_commitment": commitment,
        "head": preflight["head"],
        "tree": preflight["tree"],
        "hidden_scope": "new_observation_noise_and_dropout_sequences_only",
        "product_or_human_safety_claim": False,
        "postflight": postflight,
        "failure_trace_manifest_hash": failure_trace_manifest["content_hash"],
        "case_trace_set_hash": case_trace_set_hash,
    }
    _write_json(output / "summary.json", summary)
    (output / "summary.md").write_text(
        _summary_markdown(summary, results), encoding="utf-8"
    )
    receipt = {
        "schema": _RECEIPT_SCHEMA,
        "completed": True,
        "passed": audit.passed,
        "head": preflight["head"],
        "tree": preflight["tree"],
        "seed_commitment": commitment,
        "case_catalog_hash": pre_run["case_catalog_hash"],
        "result_set_hash": audit.result_set_hash,
        "case_count": audit.result_count,
        "normal_completed_count": audit.normal_completed_count,
        "stress_conditionally_safe_count": audit.stress_conditionally_safe_count,
        "stress_release_count": audit.stress_release_count,
        "hard_failure_count": audit.hard_failure_count,
        "release_contract_violation_count": audit.release_contract_violation_count,
        "duplicate_safe_frame_violation_count": (
            audit.duplicate_safe_frame_violation_count
        ),
        "stale_propulsion_violation_count": audit.stale_propulsion_violation_count,
        "unauthorized_restart_count": audit.unauthorized_restart_count,
        "actual_collision_count": audit.actual_collision_count,
        "actual_forbidden_violation_count": audit.actual_forbidden_violation_count,
        "actual_clearance_violation_count": audit.actual_clearance_violation_count,
        "failure_trace_manifest_hash": failure_trace_manifest["content_hash"],
        "case_trace_set_hash": case_trace_set_hash,
        "release_receipt_content_hash": preflight["release_evidence"][
            "receipt_content_hash"
        ],
        "release_evidence_sha256": expected_sha,
        "packaging_source_freeze_hash": preflight["packaging_source_freeze"][
            "content_hash"
        ],
        "postflight_content_hash": canonical_content_hash(postflight),
        "preflight_receipt_content_hash": preflight_receipt[
            "receipt_content_hash"
        ],
        "reuse_as_final_hidden_after_code_change": False,
    }
    receipt["receipt_content_hash"] = canonical_content_hash(receipt)
    _write_json(output / "hidden-v4-consumption-receipt.json", receipt)
    _seal_consumption_ledger(
        ledger_path,
        status="completed_pass" if audit.passed else "completed_fail",
        seed_commitment=commitment,
        receipt_content_hash=receipt["receipt_content_hash"],
    )
    (output / "partial-state.json").unlink(missing_ok=True)
    print(f"hidden_v4_passed={str(audit.passed).lower()}", flush=True)
    print(f"receipt={output / 'hidden-v4-consumption-receipt.json'}", flush=True)
    return 0 if audit.passed else 1


def _prepare_committed_hidden_v4(
    *,
    output: Path,
    ledger_path: Path,
    root_seed: int,
    commitment: str,
    preflight: dict[str, object],
    max_workers: int,
) -> tuple[str, tuple[object, ...], dict[str, object]]:
    started_at = datetime.now(UTC).isoformat()
    _write_json(
        output / "seed-commitment.json",
        {
            "schema": _SEED_COMMITMENT_SCHEMA,
            "seed_commitment": commitment,
            "root_seed_disclosed_before_commitment": False,
            "commitment_written_before_seed_derivation": True,
            "created_at_utc": started_at,
        },
    )
    _write_json(
        output / "consumed-seed.json",
        {
            "schema": _CONSUMED_SEED_SCHEMA,
            "root_seed": root_seed,
            "seed_commitment": commitment,
            "consumed_at_utc": started_at,
            "reuse_as_final_hidden_after_code_change": False,
        },
    )
    _seal_consumption_ledger(
        ledger_path,
        status="seed_consumed",
        seed_commitment=commitment,
    )
    specs = build_hidden_v4_case_specs(root_seed)
    if any(spec.observation_seed in _KNOWN_PUBLIC_OBSERVATION_SEEDS for spec in specs):
        raise RuntimeError("generated observation seed was already public or consumed")
    pre_run = {
        "schema": R7_HIDDEN_V4_OBSERVATION_VERSION,
        "started_at_utc": started_at,
        "head": preflight["head"],
        "tree": preflight["tree"],
        "working_tree_clean": True,
        "seed_commitment": commitment,
        "root_seed_disclosed_before_commitment": False,
        "commitment_written_before_seed_derivation": True,
        "case_count": len(specs),
        "case_catalog_hash": canonical_content_hash(
            tuple(item.content_hash for item in specs)
        ),
        "normal_case_count": sum(item.profile_name == "normal" for item in specs),
        "stress_case_count": sum(item.profile_name == "stress" for item in specs),
        "max_workers": max_workers,
        "python_wall_clock_is_qualification": False,
        "release_evidence": preflight["release_evidence"],
        "packaging_source_freeze": preflight["packaging_source_freeze"],
        "hidden_seed_generated": True,
        "hidden_executed": True,
        "product_or_human_safety_claim": False,
    }
    _write_json(output / "pre-run-manifest.json", pre_run)
    return started_at, specs, pre_run


def _repository_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _failure_trace_manifest(
    root: Path,
    *,
    results: Sequence[object],
) -> dict[str, object]:
    paths = tuple(sorted(root.rglob("tick-trace.jsonl"))) if root.exists() else ()
    if len(paths) != len(results):
        raise RuntimeError("hidden-v4 case trace count does not match case results")
    results_by_id = {str(item.case_id): item for item in results}
    if len(results_by_id) != len(results):
        raise RuntimeError("hidden-v4 case results contain duplicate case IDs")
    records = []
    for path in paths:
        case_id = path.parent.name
        result = results_by_id.get(case_id)
        if result is None:
            raise RuntimeError(f"hidden-v4 case trace has no result: {case_id}")
        lines = path.read_text(encoding="utf-8").splitlines()
        if not lines:
            raise RuntimeError(f"hidden-v4 failure trace is empty: {path}")
        last_record = json.loads(lines[-1])
        record = {
            "case_id": case_id,
            "relative_path": path.relative_to(root.parent).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": _sha256(path),
            "record_count": len(lines),
            "last_record_hash": last_record["record_content_hash"],
            "trace_content_hash": result.trace_content_hash,
        }
        if (
            record["sha256"] != result.trace_file_sha256
            or record["record_count"] != result.trace_record_count
            or record["last_record_hash"] != result.trace_last_record_hash
        ):
            raise RuntimeError(f"hidden-v4 case trace binding mismatch: {case_id}")
        records.append(record)
    payload = {
        "schema": "r7-hidden-v4-case-trace-manifest-v1",
        "case_count": len(results),
        "records": records,
    }
    return {**payload, "content_hash": canonical_content_hash(payload)}


def _case_trace_set_hash(results: Sequence[object]) -> str:
    return canonical_content_hash(
        tuple(
            (
                item.case_id,
                item.trace_content_hash,
                item.trace_file_sha256,
                item.trace_record_count,
                item.trace_last_record_hash,
            )
            for item in results
        )
    )


def _build_preflight_receipt(
    preflight: dict[str, object],
    *,
    max_workers: int,
) -> dict[str, object]:
    payload = {
        "schema": _PREFLIGHT_SCHEMA,
        "checked_at_utc": datetime.now(UTC).isoformat(),
        **preflight,
        "max_workers_if_approved": max_workers,
        "hidden_seed_generated": False,
        "hidden_executed": False,
        "product_or_human_safety_claim": False,
    }
    return {**payload, "receipt_content_hash": canonical_content_hash(payload)}


def _consumption_ledger_path(repository_root: Path) -> Path:
    return (
        repository_root
        / "simulation/path_planning_lab/outputs"
        / f"r7-hidden-v4-{_git(repository_root, 'rev-parse', 'HEAD')}-consumption-ledger.json"
    )


def _acquire_consumption_ledger(
    repository_root: Path,
    output: Path,
    preflight: dict[str, object],
) -> Path:
    path = _consumption_ledger_path(repository_root)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema": "r7-hidden-v4-consumption-ledger-v1",
        "status": "reserved_before_seed",
        "head": preflight["head"],
        "tree": preflight["tree"],
        "output": str(output),
        "reserved_at_utc": datetime.now(UTC).isoformat(),
        "seed_commitment": None,
        "receipt_content_hash": None,
    }
    try:
        with path.open("x", encoding="utf-8", newline="\n") as handle:
            handle.write(
                json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
            )
    except FileExistsError as exc:
        raise RuntimeError(
            "hidden-v4 was already reserved or consumed for this commit"
        ) from exc
    return path


def _seal_consumption_ledger(
    path: Path,
    *,
    status: str,
    seed_commitment: str | None = None,
    receipt_content_hash: str | None = None,
) -> None:
    current = json.loads(path.read_text(encoding="utf-8"))
    current.update(
        {
            "status": status,
            "seed_commitment": seed_commitment,
            "receipt_content_hash": receipt_content_hash,
            "updated_at_utc": datetime.now(UTC).isoformat(),
        }
    )
    _write_json(path, current)


def _record_infrastructure_failure(
    output: Path,
    seed_commitment: str | None,
    completed_case_count: int,
    error: BaseException,
) -> str:
    trace_manifest = _partial_trace_manifest(output / "failure-traces")
    failure_payload = {
        "schema": "r7-hidden-v4-infrastructure-failure-v1",
        "final_status": "BLOCKED_INFRASTRUCTURE",
        "completed": False,
        "algorithm_verdict": None,
        "seed_commitment": seed_commitment,
        "completed_case_count": completed_case_count,
        "error_type": type(error).__name__,
        "error_message": str(error),
        "partial_is_final_evidence": False,
        "partial_trace_manifest_hash": trace_manifest["content_hash"],
    }
    failure = {
        **failure_payload,
        "content_hash": canonical_content_hash(failure_payload),
    }
    _write_json(output / "infrastructure-failure.json", failure)
    _write_json(output / "partial-trace-manifest.json", trace_manifest)
    summary_payload = {
        "schema": _SUMMARY_SCHEMA,
        "final_status": "BLOCKED_INFRASTRUCTURE",
        "completed": False,
        "passed": False,
        "algorithm_verdict": None,
        "seed_commitment": seed_commitment,
        "completed_case_count": completed_case_count,
        "infrastructure_failure_hash": failure["content_hash"],
        "partial_trace_manifest_hash": trace_manifest["content_hash"],
        "partial_is_final_evidence": False,
    }
    summary = {
        **summary_payload,
        "content_hash": canonical_content_hash(summary_payload),
    }
    _write_json(output / "summary.json", summary)
    receipt_payload = {
        "schema": _RECEIPT_SCHEMA,
        "final_status": "BLOCKED_INFRASTRUCTURE",
        "completed": False,
        "passed": False,
        "algorithm_verdict": None,
        "seed_commitment": seed_commitment,
        "completed_case_count": completed_case_count,
        "infrastructure_failure_hash": failure["content_hash"],
        "summary_content_hash": summary["content_hash"],
        "partial_trace_manifest_hash": trace_manifest["content_hash"],
        "reuse_as_final_hidden_after_code_change": False,
    }
    receipt = {
        **receipt_payload,
        "receipt_content_hash": canonical_content_hash(receipt_payload),
    }
    _write_json(output / "hidden-v4-consumption-receipt.json", receipt)
    return receipt["receipt_content_hash"]


def _partial_trace_manifest(root: Path) -> dict[str, object]:
    records = []
    paths = tuple(sorted(root.rglob("*.jsonl"))) if root.exists() else ()
    for path in paths:
        lines = path.read_text(encoding="utf-8").splitlines()
        last_record_hash = None
        integrity_readable = False
        if lines:
            try:
                last_record = json.loads(lines[-1])
                candidate = last_record.get("record_content_hash")
                if isinstance(candidate, str):
                    last_record_hash = candidate
                    integrity_readable = True
            except (json.JSONDecodeError, AttributeError):
                pass
        records.append(
            {
                "relative_path": path.relative_to(root.parent).as_posix(),
                "size_bytes": path.stat().st_size,
                "sha256": _sha256(path),
                "record_count": len(lines),
                "last_record_hash": last_record_hash,
                "integrity_readable": integrity_readable,
            }
        )
    payload = {
        "schema": "r7-hidden-v4-partial-trace-manifest-v1",
        "trace_file_count": len(records),
        "records": records,
    }
    return {**payload, "content_hash": canonical_content_hash(payload)}


def _preflight(
    repository_root: Path,
    evidence_path: Path,
    expected_evidence_sha256: str,
) -> dict[str, object]:
    release_evidence = _verify_release_evidence(
        repository_root,
        evidence_path,
        expected_evidence_sha256,
    )
    _verify_native_libraries(repository_root, release_evidence)
    return {
        "head": _git(repository_root, "rev-parse", "HEAD"),
        "tree": _git(repository_root, "rev-parse", "HEAD^{tree}"),
        "working_tree_clean": True,
        "release_evidence": release_evidence,
        "packaging_source_freeze": _packaging_source_freeze(repository_root),
        "machine": {
            "name": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "logical_cpu_count": os.cpu_count(),
        },
    }


def _verify_release_evidence(
    repository_root: Path,
    evidence_path: Path,
    expected_sha256: str,
) -> dict[str, object]:
    if not evidence_path.is_file():
        raise RuntimeError("R7 release evidence ZIP is missing")
    actual_sha = _sha256(evidence_path)
    if actual_sha != expected_sha256:
        raise RuntimeError("R7 release evidence ZIP hash mismatch")
    with ZipFile(evidence_path) as archive:
        names = archive.namelist()
        if len(names) != len(set(names)):
            raise RuntimeError("R7 release evidence ZIP contains duplicate names")
        missing = [name for name in _REQUIRED_EVIDENCE_FILES if name not in names]
        if missing:
            raise RuntimeError(f"R7 release evidence ZIP is incomplete: {missing}")
        manifest = json.loads(archive.read("run-manifest.json"))
        receipt = json.loads(archive.read("qualification-receipt.json"))
        parity = json.loads(archive.read("semantic-parity.json"))
        contract_parity = json.loads(archive.read("contract-parity.json"))
        timing = json.loads(archive.read("timing-qualification.json"))
        release_gate = json.loads(archive.read("release-gate.json"))

    receipt_payload = {
        key: value for key, value in receipt.items() if key != "receipt_content_hash"
    }
    if receipt.get("receipt_content_hash") != canonical_content_hash(receipt_payload):
        raise RuntimeError("R7 release receipt content hash mismatch")
    timing_cases = timing.get("cases", ())
    expected_case_ids = (
        "actor-0-free",
        "actor-1-active",
        "actor-2-active",
        "corner-static-forbidden",
        "staggered-risk-multisegment",
    )
    formal_cases_pass = bool(
        tuple(item.get("case_id") for item in timing_cases) == expected_case_ids
        and all(
            item.get("sample_count") == 100
            and item.get("deadline_miss_count") == 0
            and item.get("deadline_ns") == 50_000_000
            and item.get("maximum_ns", 50_000_001) <= 50_000_000
            for item in timing_cases
        )
    )
    if not all(
        (
            release_gate.get("qualified") is True,
            release_gate.get("eligible_for_user_hidden_approval") is True,
            release_gate.get("hidden_executed") is False,
            manifest.get("hidden_executed") is False,
            receipt.get("hidden_executed") is False,
            timing.get("passed") is True,
            timing.get("sample_count") == 500,
            timing.get("deadline_ns") == 50_000_000,
            timing.get("warmups_per_case") == 30,
            timing.get("repeats_per_case") == 100,
            timing.get("parallelized") is False,
            timing.get("execution_mode") == "serial_parent_no_worker",
            timing.get("aggregate", {}).get("deadline_miss_count") == 0,
            timing.get("aggregate", {}).get("maximum_ns", 50_000_001)
            <= 50_000_000,
            receipt.get("sample_count") == 500,
            receipt.get("deadline_miss_count") == 0,
            receipt.get("deadline_ns") == 50_000_000,
            manifest.get("warmups") == 30,
            manifest.get("repeats") == 100,
            manifest.get("build_executed") is True,
            all(release_gate.get("checks", {}).values()),
            formal_cases_pass,
        )
    ):
        raise RuntimeError("R7 release or timing gate is not passing")
    parity_records = parity.get("records", ())
    if (
        parity.get("passed") is not True
        or len(parity_records) != 5
        or not all(item.get("passed") is True for item in parity_records)
    ):
        raise RuntimeError("R7 semantic parity is not 5/5")
    if receipt.get("semantic_parity_hash") != parity.get("content_hash"):
        raise RuntimeError("R7 semantic parity receipt binding mismatch")
    if not all(
        (
            contract_parity.get("passed") is True,
            contract_parity.get("expected_test_count") == 13,
            contract_parity.get("passed_test_count") == 13,
            receipt.get("contract_parity_hash")
            == contract_parity.get("content_hash"),
        )
    ):
        raise RuntimeError("R7 boundary and terminal contract parity is not passing")
    if receipt.get("timing_result_hash") != canonical_content_hash(timing):
        raise RuntimeError("R7 timing receipt binding mismatch")

    source_before = manifest.get("source_freeze_before", {})
    source_after = manifest.get("source_freeze_after", {})
    if source_before != source_after:
        raise RuntimeError("R7 executable source changed during qualification")
    if receipt.get("source_freeze_hash") != source_after.get("content_hash"):
        raise RuntimeError("R7 source freeze receipt binding mismatch")
    if source_after.get("content_hash") != canonical_content_hash(
        source_after.get("records", ())
    ):
        raise RuntimeError("R7 source freeze content hash mismatch")
    evidence_head = manifest.get("git_after", {}).get("head")
    evidence_tree = manifest.get("git_after", {}).get("tree")
    if (
        not isinstance(evidence_head, str)
        or not isinstance(evidence_tree, str)
        or manifest.get("git_before", {}).get("head") != evidence_head
        or manifest.get("git_before", {}).get("tree") != evidence_tree
        or receipt.get("head") != evidence_head
        or receipt.get("tree") != evidence_tree
    ):
        raise RuntimeError("R7 release Git identity is inconsistent")
    current_head = _git(repository_root, "rev-parse", "HEAD")
    current_tree = _git(repository_root, "rev-parse", "HEAD^{tree}")
    if evidence_head != current_head or evidence_tree != current_tree:
        raise RuntimeError("R7 release evidence must match the hidden execution commit")
    records = source_after.get("records", ())
    paths = tuple(record.get("path") for record in records)
    if len(paths) != len(set(paths)):
        raise RuntimeError("R7 source freeze contains duplicate paths")
    for record in records:
        relative_path = record.get("path")
        if not isinstance(relative_path, str):
            raise RuntimeError("R7 source freeze record path is invalid")
        current = repository_root / relative_path
        expected_size = record.get("size_bytes", record.get("size"))
        if (
            not current.is_file()
            or current.stat().st_size != expected_size
            or _sha256(current) != record.get("sha256")
        ):
            raise RuntimeError(f"R7 frozen executable source changed: {relative_path}")

    return {
        "path": str(evidence_path),
        "size_bytes": evidence_path.stat().st_size,
        "sha256": actual_sha,
        "head": evidence_head,
        "tree": evidence_tree,
        "sample_count": 500,
        "deadline_miss_count": 0,
        "semantic_parity_case_count": 5,
        "contract_parity_test_count": 13,
        "receipt_content_hash": receipt["receipt_content_hash"],
        "source_freeze_hash": receipt["source_freeze_hash"],
        "native_full_library_sha256": receipt["native_full_library_sha256"],
        "native_safety_library_sha256": receipt["native_safety_library_sha256"],
    }


def _verify_execution_freeze(
    repository_root: Path,
    preflight: dict[str, object],
) -> dict[str, object]:
    if _git(repository_root, "status", "--porcelain=v1"):
        raise RuntimeError("hidden-v4 working tree changed during execution")
    head = _git(repository_root, "rev-parse", "HEAD")
    tree = _git(repository_root, "rev-parse", "HEAD^{tree}")
    if head != preflight["head"] or tree != preflight["tree"]:
        raise RuntimeError("hidden-v4 Git identity changed during execution")
    release_before = preflight["release_evidence"]
    release_after = _verify_release_evidence(
        repository_root,
        Path(str(release_before["path"])),
        str(release_before["sha256"]),
    )
    if release_after != release_before:
        raise RuntimeError("R7 release evidence changed during hidden-v4 execution")
    packaging_after = _packaging_source_freeze(repository_root)
    if packaging_after != preflight["packaging_source_freeze"]:
        raise RuntimeError("hidden-v4 packaging source changed during execution")
    _verify_native_libraries(repository_root, release_after)
    return {
        "head": head,
        "tree": tree,
        "working_tree_clean": True,
        "release_evidence_sha256": release_after["sha256"],
        "packaging_source_freeze_hash": packaging_after["content_hash"],
        "native_libraries_match_release": True,
    }


def _packaging_source_freeze(repository_root: Path) -> dict[str, object]:
    records = []
    for relative_path in _PACKAGING_PATHS:
        path = repository_root / relative_path
        if not path.is_file():
            raise RuntimeError(f"hidden-v4 packaging source is missing: {relative_path}")
        _git(repository_root, "ls-files", "--error-unmatch", relative_path)
        if subprocess.run(
            ("git", "diff", "--quiet", "HEAD", "--", relative_path),
            cwd=repository_root,
            check=False,
        ).returncode != 0:
            raise RuntimeError(f"hidden-v4 packaging source is not committed: {relative_path}")
        records.append(
            {
                "path": relative_path,
                "size_bytes": path.stat().st_size,
                "sha256": _sha256(path),
            }
        )
    payload = {"records": records}
    return {**payload, "content_hash": canonical_content_hash(payload)}


def _verify_native_libraries(
    repository_root: Path,
    release_evidence: dict[str, object],
) -> None:
    native = repository_root / "simulation/path_planning_lab/src/hospital_path_lab/_native"
    expected = {
        "dwb_full_core.dll": release_evidence["native_full_library_sha256"],
        "dwb_safety_core.dll": release_evidence["native_safety_library_sha256"],
    }
    for name, expected_hash in expected.items():
        path = native / name
        if not path.is_file():
            raise RuntimeError(f"required native library is missing: {name}")
        if _sha256(path) != expected_hash:
            raise RuntimeError(f"required native library hash mismatch: {name}")


def _git(root: Path, *args: str) -> str:
    return subprocess.run(
        ("git", *args),
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    ).stdout.strip()


def _git_bytes(root: Path, *args: str) -> bytes:
    return subprocess.run(
        ("git", *args), cwd=root, check=True, capture_output=True
    ).stdout


def _sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, value) -> None:
    temporary = path.with_name(f"{path.name}.tmp")
    temporary.write_text(
        json.dumps(_json_value(value), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _json_value(value):
    if hasattr(value, "__dataclass_fields__"):
        return _json_value(asdict(value))
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def _summary_markdown(summary, results) -> str:
    lines = [
        "# R7 hidden-v4 관측 시험 결과",
        "",
        f"- 판정: `{'PASS' if summary['passed'] else 'FAIL'}`",
        f"- case: `{summary['case_count']}/20`",
        f"- Normal 완료: `{summary['normal_completed_count']}/10`",
        "- Stress 조건부 안전: "
        f"`{summary['stress_conditionally_safe_count']}/10`",
        f"- Stress 출발 사례: `{summary['stress_release_count']}/10`",
        f"- hard failure: `{summary['hard_failure_count']}`",
        f"- seed commitment: `{summary['seed_commitment']}`",
        "- 범위: 새 합성 관측 순서만, 실제 카메라·사람 안전 증거 아님",
        "",
        "| case | profile | outcome | pass | hard |",
        "|---|---|---|---|---:|",
    ]
    lines.extend(
        f"| `{item.case_id}` | `{item.profile_name}` | `{item.outcome}` | "
        f"`{item.passed}` | `{len(item.hard_failures)}` |"
        for item in results
    )
    if summary["failures"]:
        lines.extend(("", "## 실패", ""))
        lines.extend(f"- `{item}`" for item in summary["failures"])
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    raise SystemExit(main())
