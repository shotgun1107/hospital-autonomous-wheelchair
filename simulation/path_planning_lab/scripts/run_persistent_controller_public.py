"""동결된 R5-A persistent RPP/DWB public qualification을 실행한다."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from hospital_path_lab.local_reference_reporting import public_local_reference_cases
from hospital_path_lab.persistent_controller_reporting import (
    PersistentPublicCaseResult,
    PersistentPublicOutputWriter,
    audit_persistent_public_catalog,
    build_persistent_public_manifest,
    evaluate_persistent_public_cases,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-workers", type=int, default=8)
    parser.add_argument(
        "--public-case-limit",
        type=int,
        help="development-only: catalog prefix만 실행하며 receipt를 만들지 않는다",
    )
    parser.add_argument(
        "--tick-limit",
        type=int,
        help="development-only: episode tick을 줄이며 receipt를 만들지 않는다",
    )
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="development-only: dirty tree에서는 receipt를 만들지 않는다",
    )
    args = parser.parse_args()
    if args.max_workers <= 0:
        parser.error("max-workers must be positive")
    if args.public_case_limit is not None and args.public_case_limit <= 0:
        parser.error("public-case-limit must be positive")
    if args.tick_limit is not None and args.tick_limit <= 0:
        parser.error("tick-limit must be positive")

    _configure_numeric_threads()
    repository_root = Path(__file__).resolve().parents[3]
    manifest = build_persistent_public_manifest(
        repository_root=repository_root,
        max_workers=args.max_workers,
        public_case_limit=args.public_case_limit,
        tick_limit_override=args.tick_limit,
    )
    if manifest.git_dirty and not args.allow_dirty:
        raise RuntimeError(
            "final R5-A public qualification requires a clean Git tree; "
            "commit the implementation first"
        )
    writer = PersistentPublicOutputWriter(
        args.output_dir,
        manifest,
        repository_root=repository_root,
    )
    writer.start()
    print(f"output={args.output_dir}", flush=True)
    print(f"manifest={manifest.content_hash}", flush=True)

    if not manifest.sealing_run:
        cases = public_local_reference_cases()[: args.public_case_limit]
        results = evaluate_persistent_public_cases(
            cases,
            max_workers=args.max_workers,
            tick_limit_override=args.tick_limit,
            on_case=lambda result: _record_case(writer, result),
        )
        print(f"status=PARTIAL_REPORT_ONLY:{len(results)}", flush=True)
        print("receipt=NOT_CREATED", flush=True)
        return 0

    try:
        audit = audit_persistent_public_catalog(
            max_workers=args.max_workers,
            on_case=lambda result: _record_case(writer, result),
        )
    except KeyboardInterrupt:
        print("status=INFRASTRUCTURE_INCOMPLETE:user_interrupted", flush=True)
        return 130
    summary_json, summary_md, receipt = writer.complete(audit)
    print(f"hard={'PASS' if audit.hard_passed else 'FAIL'}")
    print(f"summary_json={summary_json}")
    print(f"summary_md={summary_md}")
    print(f"receipt={receipt if receipt is not None else 'NOT_CREATED'}")
    return 0 if audit.hard_passed else 1


def _configure_numeric_threads() -> None:
    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ[name] = "1"


def _record_case(
    writer: PersistentPublicOutputWriter,
    result: PersistentPublicCaseResult,
) -> None:
    writer.write_case(result)
    print(
        f"case_complete={result.ordinal:02d}:{result.public_id}:"
        f"{result.reference_status.value}",
        flush=True,
    )


if __name__ == "__main__":
    raise SystemExit(main())
