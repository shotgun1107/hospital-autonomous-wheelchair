"""동결된 R3 bounded 공간 oracle public qualification을 실행한다."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from hospital_path_lab.spatial_oracle_reporting import (
    SpatialPublicOutputWriter,
    audit_spatial_public_catalog,
    build_spatial_public_manifest,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-workers", type=int, default=14)
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="development-only: dirty tree에서는 qualification receipt를 만들지 않는다",
    )
    args = parser.parse_args()
    if args.max_workers <= 0:
        parser.error("max-workers must be positive")

    _configure_numeric_threads()
    repository_root = Path(__file__).resolve().parents[3]
    manifest = build_spatial_public_manifest(
        repository_root=repository_root,
        max_workers=args.max_workers,
    )
    if manifest.git_dirty and not args.allow_dirty:
        raise RuntimeError(
            "final R3 public qualification requires a clean Git tree; "
            "commit the implementation first"
        )

    writer = SpatialPublicOutputWriter(
        args.output_dir,
        manifest,
        repository_root=repository_root,
    )
    writer.start()
    print(f"output={args.output_dir}", flush=True)
    print(f"manifest={manifest.content_hash}", flush=True)
    try:
        audit = audit_spatial_public_catalog(
            max_workers=args.max_workers,
            on_case=lambda result: _record_case(writer, result),
        )
    except KeyboardInterrupt:
        print("status=INFRASTRUCTURE_INCOMPLETE:user_interrupted", flush=True)
        return 130

    results_path, summary_path, receipt_path = writer.complete(audit)
    print(f"hard={'PASS' if audit.hard_passed else 'FAIL'}")
    print(f"results={results_path}")
    print(f"summary={summary_path}")
    print(f"receipt={receipt_path if receipt_path is not None else 'NOT_CREATED'}")
    return 0 if audit.hard_passed else 1


def _configure_numeric_threads() -> None:
    for name in (
        "OMP_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "MKL_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ[name] = "1"


def _record_case(writer: SpatialPublicOutputWriter, result: object) -> None:
    from hospital_path_lab.spatial_oracle_reporting import SpatialPublicCaseResult

    if not isinstance(result, SpatialPublicCaseResult):
        raise TypeError("runner callback requires SpatialPublicCaseResult")
    writer.write_case(result)
    print(
        f"case_complete={result.ordinal:02d}:{result.public_id}:{result.result.status.value}",
        flush=True,
    )


if __name__ == "__main__":
    raise SystemExit(main())
