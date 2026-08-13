"""Run the public-only directional prediction contract audit."""

from __future__ import annotations

import argparse
from pathlib import Path

from hospital_path_lab.dynamic_prediction_audit import (
    audit_public_prediction_contract,
    write_prediction_contract_audit,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="new directory for prediction_contract_audit.json and summary.md",
    )
    args = parser.parse_args()
    audit = audit_public_prediction_contract()
    json_path, summary_path = write_prediction_contract_audit(audit, args.output_dir)
    print(f"audit={'PASS' if audit.passed else 'FAIL'}")
    print(f"hard_failures={len(audit.hard_failures)}")
    print(f"json={json_path}")
    print(f"summary={summary_path}")
    return 0 if audit.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
