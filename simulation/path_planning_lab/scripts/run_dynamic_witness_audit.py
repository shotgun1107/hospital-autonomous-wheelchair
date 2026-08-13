"""Run the frozen public-only R2 witness audit and write immutable artifacts."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from hospital_path_lab.dynamic_witness_reporting import (
    WitnessAuditOutputWriter,
    WitnessEpisodeAudit,
    audit_public_witness_corpus,
    build_witness_audit_manifest,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--r1-audit-json", type=Path, required=True)
    parser.add_argument("--max-workers", type=int, default=14)
    parser.add_argument("--shard-size", type=int, default=2_048)
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="development-only: permit a dirty tree; never use for final R2 evidence",
    )
    args = parser.parse_args()
    if args.max_workers <= 0 or args.shard_size <= 0:
        parser.error("worker and shard settings must be positive")

    repository_root = Path(__file__).resolve().parents[3]
    r1_content_hash = _load_r1_audit_hash(args.r1_audit_json)
    _configure_numeric_threads()
    manifest = build_witness_audit_manifest(
        repository_root=repository_root,
        r1_audit_content_hash=r1_content_hash,
        max_workers=args.max_workers,
        shard_size=args.shard_size,
    )
    if manifest.git_dirty and not args.allow_dirty:
        raise RuntimeError(
            "final witness audit requires a clean Git tree; commit the implementation first"
        )

    writer = WitnessAuditOutputWriter(args.output_dir, manifest)
    writer.start()
    print(f"output={args.output_dir}", flush=True)
    print(f"manifest={manifest.content_hash}", flush=True)
    try:
        audit = audit_public_witness_corpus(
            r1_audit_content_hash=r1_content_hash,
            max_workers=args.max_workers,
            shard_size=args.shard_size,
            on_episode=lambda result: _record_episode(writer, result),
        )
    except KeyboardInterrupt:
        print("status=INFRASTRUCTURE_INCOMPLETE:user_interrupted", flush=True)
        return 130

    results_path, summary_path, receipt_path = writer.complete(audit)
    print(f"hard={'PASS' if audit.hard_passed else 'FAIL'}")
    print(f"r2_completion={'PASS' if audit.r2_completion_qualified else 'FAIL'}")
    print(f"results={results_path}")
    print(f"summary={summary_path}")
    print(f"completion={receipt_path}")
    return 0 if audit.r2_completion_qualified else 1


def _record_episode(
    writer: WitnessAuditOutputWriter,
    result: WitnessEpisodeAudit,
) -> None:
    writer.write_episode(result)
    print(f"episode_complete={result.public_id}", flush=True)


def _load_r1_audit_hash(path: Path) -> str:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    content_hash = payload.get("content_hash")
    hard_failures = payload.get("hard_failures")
    if not isinstance(content_hash, str) or len(content_hash) != 64:
        raise ValueError("R1 audit JSON does not contain a valid content hash")
    if hard_failures:
        raise ValueError("R1 audit must have zero hard failures before R2 final audit")
    return content_hash


def _configure_numeric_threads() -> None:
    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ[name] = "1"


if __name__ == "__main__":
    raise SystemExit(main())
