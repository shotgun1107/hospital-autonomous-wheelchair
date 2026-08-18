"""Run the one-use R7 hidden observation sequence study."""

from __future__ import annotations

import argparse
import json
import os
import platform
import secrets
import subprocess
from dataclasses import asdict
from datetime import UTC, datetime
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile

from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r7_hidden_qualification import (
    R7_HIDDEN_OBSERVATION_VERSION,
    audit_hidden_results,
    build_hidden_case_specs,
    evaluate_hidden_cases,
    hidden_seed_commitment,
)

R7_EVIDENCE_RELATIVE_PATH = Path(
    "simulation/path_planning_lab/outputs/"
    "r7-native-v2-public-qualification-evidence-20260817-8c3b733.zip"
)
R7_EVIDENCE_SIZE = 7_667
R7_EVIDENCE_SHA256 = (
    "4f784e086a60d86e99be15a5a39f9589d51593458e065a05abc636d1a8c01d8a"
)
R7_IMPLEMENTATION_COMMIT = "8c3b733"
R7_RESULT_DOCUMENT = (
    "docs/research/dynamic-actor-experiment/26-r7-native-release-gate.md"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-workers", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    args = parser.parse_args()
    if args.max_workers <= 0:
        parser.error("max-workers must be positive")

    repository_root = Path(__file__).resolve().parents[3]
    output = args.output.resolve()
    if output.exists():
        parser.error("output path already exists; hidden outputs are never overwritten")
    if _git(repository_root, "status", "--porcelain=v1"):
        parser.error("hidden run requires a clean Git working tree")

    r7_gate = _verify_r7_evidence(repository_root)
    _verify_native_libraries(repository_root)
    output.mkdir(parents=True)

    root_seed = secrets.randbits(63)
    commitment = hidden_seed_commitment(root_seed)
    specs = build_hidden_case_specs(root_seed)
    head = _git(repository_root, "rev-parse", "HEAD")
    tree = _git(repository_root, "rev-parse", "HEAD^{tree}")
    started_at = datetime.now(UTC).isoformat()
    pre_run = {
        "schema": R7_HIDDEN_OBSERVATION_VERSION,
        "started_at_utc": started_at,
        "head": head,
        "tree": tree,
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
        "r7_gate": r7_gate,
        "machine": {
            "name": platform.node(),
            "platform": platform.platform(),
            "python": platform.python_version(),
            "logical_cpu_count": os.cpu_count(),
        },
    }
    _write_json(output / "pre-run-manifest.json", pre_run)
    _write_json(
        output / "consumed-seed.json",
        {
            "schema": "r7-hidden-consumed-seed-v1",
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
                "schema": "r7-hidden-partial-v1",
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

    results = evaluate_hidden_cases(
        repository_root,
        specs,
        max_workers=args.max_workers,
        on_case=on_case,
    )
    audit = audit_hidden_results(specs, results)
    _write_json(output / "case-results.json", results)
    summary = {
        "schema": "r7-hidden-observation-summary-v1",
        "passed": audit.passed,
        "case_count": audit.result_count,
        "normal_completed_count": audit.normal_completed_count,
        "stress_holding_count": audit.stress_holding_count,
        "hard_failure_count": audit.hard_failure_count,
        "failures": audit.failures,
        "result_set_hash": audit.result_set_hash,
        "seed_commitment": commitment,
        "head": head,
        "tree": tree,
        "hidden_scope": "new_observation_noise_and_dropout_sequences_only",
        "product_or_human_safety_claim": False,
    }
    _write_json(output / "summary.json", summary)
    (output / "summary.md").write_text(
        _summary_markdown(summary, results), encoding="utf-8"
    )
    receipt = {
        "schema": "r7-hidden-consumption-receipt-v1",
        "completed": True,
        "passed": audit.passed,
        "head": head,
        "tree": tree,
        "seed_commitment": commitment,
        "case_catalog_hash": pre_run["case_catalog_hash"],
        "result_set_hash": audit.result_set_hash,
        "case_count": audit.result_count,
        "normal_completed_count": audit.normal_completed_count,
        "stress_holding_count": audit.stress_holding_count,
        "hard_failure_count": audit.hard_failure_count,
        "r7_receipt_content_hash": r7_gate["receipt_content_hash"],
        "reuse_as_final_hidden_after_code_change": False,
    }
    receipt["receipt_content_hash"] = canonical_content_hash(receipt)
    _write_json(output / "hidden-consumption-receipt.json", receipt)
    (output / "partial-state.json").unlink(missing_ok=True)
    print(f"hidden_passed={str(audit.passed).lower()}", flush=True)
    print(f"receipt={output / 'hidden-consumption-receipt.json'}", flush=True)
    return 0 if audit.passed else 1


def _verify_r7_evidence(repository_root: Path) -> dict[str, object]:
    evidence = repository_root / R7_EVIDENCE_RELATIVE_PATH
    if not evidence.is_file() or evidence.stat().st_size != R7_EVIDENCE_SIZE:
        raise RuntimeError("R7 evidence ZIP size mismatch")
    if _sha256(evidence) != R7_EVIDENCE_SHA256:
        raise RuntimeError("R7 evidence ZIP hash mismatch")
    with ZipFile(evidence) as archive:
        manifest = json.loads(archive.read("run-manifest.json"))
        receipt = json.loads(archive.read("qualification-receipt.json"))
        parity = json.loads(archive.read("semantic-parity.json"))
    if receipt.get("deadline_miss_count") != 0 or receipt.get("sample_count") != 500:
        raise RuntimeError("R7 timing qualification is not passing")
    if manifest.get("hidden_executed") is not False:
        raise RuntimeError("R7 evidence already reports hidden execution")
    parity_cases = parity.get("records", ())
    if (
        parity.get("passed") is not True
        or len(parity_cases) != 5
        or not all(item.get("passed") for item in parity_cases)
    ):
        raise RuntimeError("R7 semantic parity is not 5/5")
    source_freeze = manifest.get("source_freeze_after", {})
    if source_freeze != manifest.get("source_freeze_before"):
        raise RuntimeError("R7 source changed during its qualification run")
    for record in source_freeze.get("records", ()):
        relative_path = record["path"]
        frozen_bytes = _git_bytes(
            repository_root,
            "show",
            f"{R7_IMPLEMENTATION_COMMIT}:{relative_path}",
        )
        if relative_path == R7_RESULT_DOCUMENT:
            continue
        current_bytes = _git_bytes(repository_root, "show", f"HEAD:{relative_path}")
        if current_bytes != frozen_bytes:
            raise RuntimeError(f"R7 frozen executable source changed: {relative_path}")
    return {
        "evidence_size": R7_EVIDENCE_SIZE,
        "evidence_sha256": R7_EVIDENCE_SHA256,
        "deadline_miss_count": receipt["deadline_miss_count"],
        "sample_count": receipt["sample_count"],
        "semantic_parity_case_count": len(parity_cases),
        "receipt_content_hash": receipt["receipt_content_hash"],
        "source_freeze_hash": receipt["source_freeze_hash"],
    }


def _verify_native_libraries(repository_root: Path) -> None:
    native = (
        repository_root
        / "simulation/path_planning_lab/src/hospital_path_lab/_native"
    )
    missing = [
        name
        for name in ("dwb_full_core.dll", "dwb_safety_core.dll")
        if not (native / name).is_file()
    ]
    if missing:
        raise RuntimeError(f"required native libraries are missing: {', '.join(missing)}")


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
        ("git", *args),
        cwd=root,
        check=True,
        capture_output=True,
    ).stdout


def _sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, value) -> None:
    path.write_text(
        json.dumps(_json_value(value), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


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
        "# R7 후속 비공개 관측 시험 결과",
        "",
        f"- 판정: `{'PASS' if summary['passed'] else 'FAIL'}`",
        f"- case: `{summary['case_count']}/20`",
        f"- Normal 완료: `{summary['normal_completed_count']}/10`",
        f"- Stress 보수적 정지: `{summary['stress_holding_count']}/10`",
        f"- hard failure: `{summary['hard_failure_count']}`",
        f"- seed commitment: `{summary['seed_commitment']}`",
        "- 범위: 새 관측 잡음·dropout 순서만, 실제 카메라·사람 안전 증거 아님",
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
