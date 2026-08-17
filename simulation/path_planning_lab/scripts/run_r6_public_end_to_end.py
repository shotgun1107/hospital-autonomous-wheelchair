"""최신 R5 실행을 사용하는 R6 연속 공개 종단 자격을 실행한다."""

from __future__ import annotations

import argparse
import json
import subprocess
from dataclasses import fields, is_dataclass
from enum import Enum
from hashlib import sha256
from pathlib import Path

from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.r6_public_qualification import (
    R6_PUBLIC_QUALIFICATION_VERSION,
    audit_r6_public_results,
    evaluate_r6_public_cases,
    public_r6_case_specs,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument("--case-limit", type=int)
    parser.add_argument("--case-id", action="append", default=[])
    args = parser.parse_args()
    if args.max_workers <= 0:
        parser.error("max-workers must be positive")
    if args.case_limit is not None and args.case_limit <= 0:
        parser.error("case-limit must be positive")
    if args.case_limit is not None and args.case_id:
        parser.error("case-limit and case-id cannot be used together")

    repository_root = Path(__file__).resolve().parents[3]
    output = args.output_dir.resolve()
    if output.exists():
        raise RuntimeError("R6 output directory must not already exist")
    output.mkdir(parents=True)

    all_specs = public_r6_case_specs(repository_root)
    if args.case_id:
        requested = tuple(dict.fromkeys(args.case_id))
        by_id = {item.case_id: item for item in all_specs}
        missing = tuple(item for item in requested if item not in by_id)
        if missing:
            parser.error(f"unknown case-id: {', '.join(missing)}")
        specs = tuple(by_id[item] for item in requested)
    else:
        specs = all_specs if args.case_limit is None else all_specs[: args.case_limit]
    source_before = _source_freeze_hash(repository_root)
    head = _git(repository_root, "rev-parse", "HEAD")
    tree = _git(repository_root, "rev-parse", "HEAD^{tree}")
    clean_before = not _git(repository_root, "status", "--porcelain=v1")
    sealing_run = args.case_limit is None and not args.case_id and clean_before
    manifest = {
        "version": R6_PUBLIC_QUALIFICATION_VERSION,
        "head": head,
        "tree": tree,
        "clean_before": clean_before,
        "sealing_run": sealing_run,
        "source_freeze_hash_before": source_before,
        "required_case_count": len(all_specs),
        "selected_case_count": len(specs),
        "case_catalog_hash": canonical_content_hash(all_specs),
        "selected_case_hash": canonical_content_hash(specs),
        "max_workers": args.max_workers,
        "wall_clock_is_qualification": False,
        "hidden_executed": False,
        "native_dwb_full_core_sha256": _optional_file_sha256(
            repository_root
            / "simulation/path_planning_lab/src/hospital_path_lab/_native/dwb_full_core.dll"
        ),
    }
    _write_json(output / "run-manifest.json", manifest)
    partial = []

    def on_case(result) -> None:
        partial.append(result)
        partial.sort(key=lambda item: item.ordinal)
        _write_json(output / "partial-results.json", partial)
        print(
            f"case={len(partial)}/{len(specs)}:{result.case_id}:"
            f"{'PASS' if result.passed else 'FAIL'}:{result.elapsed_s:.2f}s",
            flush=True,
        )

    results = evaluate_r6_public_cases(
        repository_root,
        specs,
        max_workers=args.max_workers,
        on_case=on_case,
    )
    audit = audit_r6_public_results(specs, results)
    source_after = _source_freeze_hash(repository_root)
    clean_after = not _git(repository_root, "status", "--porcelain=v1")
    source_stable = source_after == source_before
    qualified = bool(
        sealing_run and audit.passed and source_stable and clean_after
    )
    summary = {
        "version": R6_PUBLIC_QUALIFICATION_VERSION,
        "qualified": qualified,
        "report_only": not sealing_run,
        "audit_passed": audit.passed,
        "source_stable": source_stable,
        "clean_after": clean_after,
        "passed_case_count": sum(item.passed for item in results),
        "result_count": len(results),
        "required_case_count": audit.required_case_count,
        "failures": audit.failures,
        "result_set_hash": audit.result_set_hash,
        "source_freeze_hash_before": source_before,
        "source_freeze_hash_after": source_after,
        "hidden_executed": False,
        "wall_clock_is_qualification": False,
    }
    _write_json(output / "case-results.json", results)
    _write_json(output / "summary.json", summary)
    (output / "summary.md").write_text(_summary_markdown(summary, results), encoding="utf-8")
    if qualified:
        receipt = {
            "schema": "r6-public-end-to-end-receipt-v1",
            "head": head,
            "tree": tree,
            "source_freeze_hash": source_after,
            "case_catalog_hash": manifest["case_catalog_hash"],
            "result_set_hash": audit.result_set_hash,
            "required_case_count": audit.required_case_count,
            "hard_failure_count": sum(bool(item.hard_failures) for item in results),
            "hidden_executed": False,
            "wall_clock_is_qualification": False,
            "native_dwb_full_core_sha256": manifest["native_dwb_full_core_sha256"],
        }
        receipt["receipt_content_hash"] = canonical_content_hash(receipt)
        _write_json(output / "qualification-receipt.json", receipt)
    print(f"qualified={qualified}")
    print(f"result_set_hash={audit.result_set_hash}")
    print(
        "receipt="
        + (
            str(output / "qualification-receipt.json")
            if qualified
            else "NOT_CREATED"
        )
    )
    return 0 if qualified or not sealing_run else 1


def _git(root: Path, *args: str) -> str:
    return subprocess.run(
        ("git", *args),
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _source_freeze_hash(root: Path) -> str:
    roots = (
        root / "simulation/path_planning_lab/src/hospital_path_lab",
        root / "simulation/path_planning_lab/native",
        root / "simulation/path_planning_lab/scripts/run_r6_public_end_to_end.py",
        root / "docs/research/dynamic-actor-experiment/25-r6-public-end-to-end-qualification.md",
    )
    files = []
    for item in roots:
        files.extend(
            (
                path
                for path in item.rglob("*")
                if path.is_file() and path.suffix in {".py", ".cpp", ".h", ".txt"}
            )
            if item.is_dir()
            else (item,)
        )
    digest = sha256()
    for path in sorted(set(files)):
        relative = path.relative_to(root).as_posix().encode("utf-8")
        data = path.read_bytes()
        digest.update(len(relative).to_bytes(8, "big"))
        digest.update(relative)
        digest.update(len(data).to_bytes(8, "big"))
        digest.update(data)
    return digest.hexdigest()


def _optional_file_sha256(path: Path) -> str | None:
    return sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def _json_value(value):
    if is_dataclass(value):
        return {field.name: _json_value(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    return value


def _write_json(path: Path, value) -> None:
    path.write_text(
        json.dumps(_json_value(value), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _summary_markdown(summary, results) -> str:
    rows = [
        "# R6 연속 공개 종단 실행 결과",
        "",
        f"- 자격: `{'PASS' if summary['qualified'] else 'FAIL/REPORT-ONLY'}`",
        f"- 사례: `{summary['passed_case_count']}/{summary['result_count']}`",
        f"- 결과 hash: `{summary['result_set_hash']}`",
        "- hidden: 실행하지 않음",
        "- wall-clock: 관측값일 뿐 자격조건 아님",
        "",
        "| 사례 | profile | 기대 | 결과 | 완료 tick | hard failure |",
        "|---|---|---|---|---:|---:|",
    ]
    rows.extend(
        "| "
        + " | ".join(
            (
                item.case_id,
                item.profile_name,
                item.expected_outcome.value,
                "PASS" if item.passed else "FAIL",
                "-" if item.completion_tick is None else str(item.completion_tick),
                str(len(item.hard_failures)),
            )
        )
        + " |"
        for item in results
    )
    rows.extend(("", "## 제한", "", "실제 카메라·사람·실물·제품 안전 증거가 아니다.", ""))
    return "\n".join(rows)


if __name__ == "__main__":
    raise SystemExit(main())
