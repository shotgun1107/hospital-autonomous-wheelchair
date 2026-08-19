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
    "timing-qualification.json",
    "release-gate.json",
    "qualification-receipt.json",
)
_PACKAGING_PATHS = (
    "simulation/path_planning_lab/scripts/run_r7_hidden_v4.py",
    "simulation/path_planning_lab/src/hospital_path_lab/"
    "r7_hidden_v4_qualification.py",
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
        _write_json(
            output / "preflight-manifest.json",
            {
                "schema": _PREFLIGHT_SCHEMA,
                "checked_at_utc": datetime.now(UTC).isoformat(),
                **preflight,
                "max_workers_if_approved": args.max_workers,
                "hidden_seed_generated": False,
                "hidden_executed": False,
                "product_or_human_safety_claim": False,
            },
        )
        print("preflight_passed=true", flush=True)
        print("hidden_seed_generated=false", flush=True)
        return 0

    root_seed = secrets.randbits(63)
    commitment = hidden_v4_seed_commitment(root_seed)
    output.mkdir(parents=True)
    started_at = datetime.now(UTC).isoformat()
    _write_json(
        output / "seed-commitment.json",
        {
            "schema": _SEED_COMMITMENT_SCHEMA,
            "seed_commitment": commitment,
            "root_seed_disclosed_before_run": False,
            "created_at_utc": started_at,
        },
    )
    specs = build_hidden_v4_case_specs(root_seed)
    pre_run = {
        "schema": R7_HIDDEN_V4_OBSERVATION_VERSION,
        "started_at_utc": started_at,
        "head": preflight["head"],
        "tree": preflight["tree"],
        "working_tree_clean": True,
        "seed_commitment": commitment,
        "root_seed_disclosed_before_run": False,
        "case_count": len(specs),
        "case_catalog_hash": canonical_content_hash(
            tuple(item.content_hash for item in specs)
        ),
        "normal_case_count": sum(item.profile_name == "normal" for item in specs),
        "stress_case_count": sum(item.profile_name == "stress" for item in specs),
        "max_workers": args.max_workers,
        "python_wall_clock_is_qualification": False,
        "release_evidence": preflight["release_evidence"],
        "packaging_source_freeze": preflight["packaging_source_freeze"],
        "hidden_seed_generated": True,
        "hidden_executed": True,
        "product_or_human_safety_claim": False,
    }
    _write_json(output / "pre-run-manifest.json", pre_run)
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
        )
    except BaseException as exc:
        _write_json(
            output / "infrastructure-failure.json",
            {
                "schema": "r7-hidden-v4-infrastructure-failure-v1",
                "completed": False,
                "algorithm_verdict": None,
                "seed_commitment": commitment,
                "completed_case_count": len(partial),
                "error_type": type(exc).__name__,
                "error_message": str(exc),
                "partial_is_final_evidence": False,
            },
        )
        raise

    audit = audit_hidden_v4_results(specs, results)
    _write_json(output / "case-results.json", results)
    summary = {
        "schema": _SUMMARY_SCHEMA,
        "passed": audit.passed,
        "case_count": audit.result_count,
        "normal_completed_count": audit.normal_completed_count,
        "stress_conditionally_safe_count": audit.stress_conditionally_safe_count,
        "stress_release_count": audit.stress_release_count,
        "hard_failure_count": audit.hard_failure_count,
        "failures": audit.failures,
        "result_set_hash": audit.result_set_hash,
        "seed_commitment": commitment,
        "head": preflight["head"],
        "tree": preflight["tree"],
        "hidden_scope": "new_observation_noise_and_dropout_sequences_only",
        "product_or_human_safety_claim": False,
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
        "release_receipt_content_hash": preflight["release_evidence"][
            "receipt_content_hash"
        ],
        "release_evidence_sha256": expected_sha,
        "packaging_source_freeze_hash": preflight["packaging_source_freeze"][
            "content_hash"
        ],
        "reuse_as_final_hidden_after_code_change": False,
    }
    receipt["receipt_content_hash"] = canonical_content_hash(receipt)
    _write_json(output / "hidden-v4-consumption-receipt.json", receipt)
    (output / "partial-state.json").unlink(missing_ok=True)
    print(f"hidden_v4_passed={str(audit.passed).lower()}", flush=True)
    print(f"receipt={output / 'hidden-v4-consumption-receipt.json'}", flush=True)
    return 0 if audit.passed else 1


def _repository_root() -> Path:
    return Path(__file__).resolve().parents[3]


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
        timing = json.loads(archive.read("timing-qualification.json"))
        release_gate = json.loads(archive.read("release-gate.json"))

    receipt_payload = {
        key: value for key, value in receipt.items() if key != "receipt_content_hash"
    }
    if receipt.get("receipt_content_hash") != canonical_content_hash(receipt_payload):
        raise RuntimeError("R7 release receipt content hash mismatch")
    if not all(
        (
            release_gate.get("qualified") is True,
            release_gate.get("eligible_for_user_hidden_approval") is True,
            release_gate.get("hidden_executed") is False,
            manifest.get("hidden_executed") is False,
            receipt.get("hidden_executed") is False,
            timing.get("passed") is True,
            timing.get("sample_count") == 500,
            timing.get("aggregate", {}).get("deadline_miss_count") == 0,
            receipt.get("sample_count") == 500,
            receipt.get("deadline_miss_count") == 0,
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
    if receipt.get("timing_result_hash") != canonical_content_hash(timing):
        raise RuntimeError("R7 timing receipt binding mismatch")

    source_before = manifest.get("source_freeze_before", {})
    source_after = manifest.get("source_freeze_after", {})
    if source_before != source_after:
        raise RuntimeError("R7 executable source changed during qualification")
    if receipt.get("source_freeze_hash") != source_after.get("content_hash"):
        raise RuntimeError("R7 source freeze receipt binding mismatch")
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
    _git(repository_root, "cat-file", "-e", f"{evidence_head}^{{commit}}")
    _git(repository_root, "merge-base", "--is-ancestor", evidence_head, "HEAD")
    for record in source_after.get("records", ()):
        relative_path = record.get("path")
        if not isinstance(relative_path, str):
            raise RuntimeError("R7 source freeze record path is invalid")
        current = repository_root / relative_path
        if (
            not current.is_file()
            or current.stat().st_size != record.get("size_bytes")
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
        "receipt_content_hash": receipt["receipt_content_hash"],
        "source_freeze_hash": receipt["source_freeze_hash"],
        "native_full_library_sha256": receipt["native_full_library_sha256"],
        "native_safety_library_sha256": receipt["native_safety_library_sha256"],
    }


def _packaging_source_freeze(repository_root: Path) -> dict[str, object]:
    records = []
    for relative_path in _PACKAGING_PATHS:
        path = repository_root / relative_path
        if not path.is_file():
            raise RuntimeError(f"hidden-v4 packaging source is missing: {relative_path}")
        if _git_bytes(repository_root, "show", f"HEAD:{relative_path}") != path.read_bytes():
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
