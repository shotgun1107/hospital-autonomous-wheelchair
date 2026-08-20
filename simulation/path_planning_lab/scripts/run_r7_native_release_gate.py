"""Build and run the public-only R7 native DWB release gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import zipfile
from pathlib import Path

_EVIDENCE_FILES = (
    "run-manifest.json",
    "semantic-parity.json",
    "contract-parity.json",
    "timing-qualification.json",
    "release-gate.json",
    "qualification-receipt.json",
    "summary.md",
)


def _write_json(path: Path, payload: object) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_evidence_zip(output_dir: Path, destination: Path) -> dict[str, object]:
    """Package a qualified release directory without changing its evidence."""

    if destination.exists():
        raise RuntimeError("R7 evidence ZIP destination must not already exist")
    missing = [name for name in _EVIDENCE_FILES if not (output_dir / name).is_file()]
    if missing:
        raise RuntimeError(f"R7 evidence files are incomplete: {missing}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.tmp")
    if temporary.exists():
        raise RuntimeError("R7 evidence ZIP temporary path already exists")
    try:
        with zipfile.ZipFile(
            temporary,
            mode="x",
            compression=zipfile.ZIP_DEFLATED,
            compresslevel=9,
        ) as archive:
            for name in _EVIDENCE_FILES:
                info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
                info.compress_type = zipfile.ZIP_DEFLATED
                info.external_attr = 0o100644 << 16
                archive.writestr(info, (output_dir / name).read_bytes())
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return {
        "path": str(destination),
        "size_bytes": destination.stat().st_size,
        "sha256": _sha256(destination),
        "files": list(_EVIDENCE_FILES),
    }


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    r6_source = parser.add_mutually_exclusive_group()
    r6_source.add_argument(
        "--r6-receipt",
        type=Path,
        help="Use an explicit immutable R6 qualification receipt.",
    )
    r6_source.add_argument(
        "--r6-output",
        type=Path,
        help="Legacy compatibility: read qualification-receipt.json from this output.",
    )
    parser.add_argument("--warmups", type=int, default=30)
    parser.add_argument("--repeats", type=int, default=100)
    parser.add_argument("--skip-rebuild", action="store_true")
    parser.add_argument(
        "--evidence-zip",
        type=Path,
        help="Write a deterministic ZIP only when the formal release gate passes.",
    )
    args = parser.parse_args()

    repository_root = Path(__file__).resolve().parents[3]
    lab_root = repository_root / "simulation/path_planning_lab"
    output_dir = args.output_dir.resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError("R7 output directory must be new or empty")
    output_dir.mkdir(parents=True, exist_ok=True)
    evidence_zip = args.evidence_zip.resolve() if args.evidence_zip is not None else None
    if evidence_zip is not None and evidence_zip.exists():
        raise RuntimeError("R7 evidence ZIP destination must not already exist")

    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ[name] = "1"

    build_executed = not args.skip_rebuild
    if build_executed:
        subprocess.run(
            [
                sys.executable,
                str(lab_root / "scripts/build_cpp_dwb_safety_core.py"),
            ],
            cwd=repository_root,
            env=os.environ.copy(),
            check=True,
        )
        subprocess.run(
            [
                sys.executable,
                str(lab_root / "scripts/build_cpp_dwb_full_core.py"),
            ],
            cwd=repository_root,
            env=os.environ.copy(),
            check=True,
        )

    sys.path.insert(0, str(lab_root / "src"))
    from hospital_path_lab.map_factory import canonical_content_hash
    from hospital_path_lab.r7_native_qualification import (
        R6_TRACKED_RECEIPT_RELATIVE_PATH,
        R7_GATE_SCHEMA,
        R7_RECEIPT_SCHEMA,
        R7_STANDARD_REPEATS,
        R7_STANDARD_WARMUPS,
        git_metadata,
        machine_metadata,
        native_build_metadata,
        r7_snapshot_cases,
        run_native_contract_parity,
        run_native_parity,
        run_native_timing,
        source_freeze,
        validate_r6_receipt,
    )

    r6_receipt_path = (
        args.r6_receipt.resolve()
        if args.r6_receipt is not None
        else (
            args.r6_output.resolve() / "qualification-receipt.json"
            if args.r6_output is not None
            else repository_root / R6_TRACKED_RECEIPT_RELATIVE_PATH
        )
    )

    git_before = git_metadata(repository_root)
    source_before = source_freeze(repository_root)
    r6_receipt = validate_r6_receipt(
        repository_root,
        r6_receipt_path,
    )
    cases = r7_snapshot_cases()
    case_catalog = [metadata for _, _, metadata in cases]
    parity = run_native_parity(cases)
    contract_parity = run_native_contract_parity(lab_root)
    if parity["passed"] and contract_parity["passed"]:
        timing = run_native_timing(
            cases,
            warmups=args.warmups,
            repeats=args.repeats,
        )
    else:
        timing = {
            "schema": "r7-native-dwb-qualification-v2",
            "passed": False,
            "status": "not_run",
            "reason": "native_semantic_parity_failed",
        }
    source_after = source_freeze(repository_root)
    git_after = git_metadata(repository_root)
    build = native_build_metadata(repository_root)
    machine = machine_metadata()
    formal_counts = bool(
        args.warmups == R7_STANDARD_WARMUPS
        and args.repeats == R7_STANDARD_REPEATS
    )
    clean_before = git_before["status_porcelain"] == ""
    clean_after = git_after["status_porcelain"] == ""
    checks = {
        "r6_receipt": r6_receipt["passed"] is True,
        "native_rebuilt": build_executed,
        "compiler_recorded": bool(build.get("compiler") and build.get("compiler_version")),
        "formal_counts": formal_counts,
        "clean_before": clean_before,
        "clean_after": clean_after,
        "source_stable": source_before["content_hash"] == source_after["content_hash"],
        "head_stable": git_before["head"] == git_after["head"],
        "tree_stable": git_before["tree"] == git_after["tree"],
        "parity": parity["passed"] is True,
        "contract_parity": contract_parity["passed"] is True,
        "timing": timing["passed"] is True,
        "hidden_not_executed": True,
    }
    qualified = all(checks.values())
    gate = {
        "schema": R7_GATE_SCHEMA,
        "qualified": qualified,
        "checks": checks,
        "eligible_for_user_hidden_approval": qualified,
        "hidden_executed": False,
        "product_algorithm_adopted": False,
        "limitations": [
            "simulation_only",
            "synthetic_actor_tracks",
            "single_recorded_machine",
            "operating_system_background_tasks_not_fully_controlled",
            "cold_start_is_degradation_only",
        ],
    }
    manifest = {
        "schema": "r7-native-release-manifest-v2",
        "git_before": git_before,
        "git_after": git_after,
        "source_freeze_before": source_before,
        "source_freeze_after": source_after,
        "r6_receipt": r6_receipt,
        "case_catalog": case_catalog,
        "case_catalog_hash": canonical_content_hash(case_catalog),
        "build": build,
        "build_executed": build_executed,
        "machine": machine,
        "warmups": args.warmups,
        "repeats": args.repeats,
        "hidden_executed": False,
    }
    _write_json(output_dir / "run-manifest.json", manifest)
    _write_json(output_dir / "semantic-parity.json", parity)
    _write_json(output_dir / "contract-parity.json", contract_parity)
    _write_json(output_dir / "timing-qualification.json", timing)
    _write_json(output_dir / "release-gate.json", gate)

    receipt = None
    if qualified:
        receipt_payload = {
            "schema": R7_RECEIPT_SCHEMA,
            "head": git_after["head"],
            "tree": git_after["tree"],
            "source_freeze_hash": source_after["content_hash"],
            "r6_receipt_hash": r6_receipt["receipt_content_hash"],
            "case_catalog_hash": manifest["case_catalog_hash"],
            "snapshot_set_hash": timing["snapshot_set_hash"],
            "semantic_parity_hash": parity["content_hash"],
            "contract_parity_hash": contract_parity["content_hash"],
            "timing_result_hash": canonical_content_hash(timing),
            "native_full_library_sha256": build["full_core"]["library_sha256"],
            "native_safety_library_sha256": build["safety_core"]["library_sha256"],
            "deadline_ns": timing["deadline_ns"],
            "sample_count": timing["sample_count"],
            "deadline_miss_count": timing["aggregate"]["deadline_miss_count"],
            "hidden_executed": False,
        }
        receipt = {
            **receipt_payload,
            "receipt_content_hash": canonical_content_hash(receipt_payload),
        }
        _write_json(output_dir / "qualification-receipt.json", receipt)

    summary_lines = [
        "# R7 C++ DWB 시간 자격 결과",
        "",
        f"- 판정: `{'PASS' if qualified else 'FAIL'}`",
        f"- Python↔C++ 동일성: `{'PASS' if parity['passed'] else 'FAIL'}`",
        "- 안전 경계·terminal tie 동일성: "
        f"`{'PASS' if contract_parity['passed'] else 'FAIL'}`",
        f"- 시간 측정: `{'PASS' if timing['passed'] else 'FAIL'}`",
        "- hidden: `미실행`",
    ]
    if timing.get("aggregate"):
        aggregate = timing["aggregate"]
        summary_lines.extend(
            [
                f"- 표본: `{timing['sample_count']}`",
                f"- p50: `{aggregate['p50_ns'] / 1_000_000:.3f}ms`",
                f"- p95: `{aggregate['p95_ns'] / 1_000_000:.3f}ms`",
                f"- p99: `{aggregate['p99_ns'] / 1_000_000:.3f}ms`",
                f"- 최대: `{aggregate['maximum_ns'] / 1_000_000:.3f}ms`",
                f"- 50ms 초과: `{aggregate['deadline_miss_count']}/{timing['sample_count']}`",
            ]
        )
    if receipt is not None:
        summary_lines.append(f"- receipt: `{receipt['receipt_content_hash']}`")
    summary_lines.extend(
        [
            "",
            "실제 카메라·실물·사람 안전 또는 제품 알고리즘 채택 결과가 아니다.",
        ]
    )
    (output_dir / "summary.md").write_text(
        "\n".join(summary_lines) + "\n",
        encoding="utf-8",
    )
    evidence = None
    if qualified and evidence_zip is not None:
        evidence = _write_evidence_zip(output_dir, evidence_zip)
        _write_json(output_dir / "evidence-package.json", evidence)
    print(
        json.dumps(
            {
                "qualified": qualified,
                "output": str(output_dir),
                "evidence_zip": evidence,
            },
            ensure_ascii=False,
        )
    )
    return 0 if qualified else 2


if __name__ == "__main__":
    raise SystemExit(_main())
