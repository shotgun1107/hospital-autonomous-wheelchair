"""Deterministic tick trace used to diagnose the public R7 failure seeds.

The collector is deliberately out-of-band: it observes already-computed values,
never calls a controller or safety gate, and excludes wall-clock data from every
semantic hash.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from math import isfinite
from pathlib import Path
from typing import Any

from hospital_path_lab.map_factory import canonical_content_hash

# v2 records the immutable authorization input used by the gate separately
# from the intentionally cleared one-shot runtime slot.
R7_FAILURE_TRACE_SCHEMA_VERSION = "r7-failure-tick-trace-v2"
R7_FAILURE_RUN_MANIFEST_SCHEMA_VERSION = "r7-failure-run-manifest-v1"
_TRACE_START = "TRACE_START"


class R7FailureTraceCollector:
    """Collect exactly one hash-chained semantic record for every control tick."""

    def __init__(self) -> None:
        self._records: list[dict[str, Any]] = []

    @property
    def records(self) -> tuple[dict[str, Any], ...]:
        return tuple(dict(record) for record in self._records)

    @property
    def semantic_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema": R7_FAILURE_TRACE_SCHEMA_VERSION,
                "record_hashes": tuple(
                    record["record_content_hash"] for record in self._records
                ),
            }
        )

    def append(self, record: Mapping[str, Any]) -> dict[str, Any]:
        frozen = dict(record)
        if "record_content_hash" in frozen or "previous_record_hash" in frozen:
            raise ValueError("trace record hash fields are collector-owned")
        tick = frozen.get("tick")
        if isinstance(tick, bool) or not isinstance(tick, int) or tick < 0:
            raise ValueError("trace tick must be a non-negative exact integer")
        if tick != len(self._records):
            raise ValueError("trace ticks must be contiguous and start at zero")
        frozen["schema"] = R7_FAILURE_TRACE_SCHEMA_VERSION
        frozen["previous_record_hash"] = (
            _TRACE_START
            if not self._records
            else self._records[-1]["record_content_hash"]
        )
        frozen["record_content_hash"] = canonical_content_hash(frozen)
        self._records.append(frozen)
        return dict(frozen)

    def write_jsonl(self, path: str | Path) -> None:
        output = Path(path)
        if output.exists():
            raise FileExistsError(f"R7 trace output already exists: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
        payload = "".join(
            json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
            + "\n"
            for record in self._records
        )
        output.write_text(payload, encoding="utf-8", newline="\n")


def write_r7_failure_run_manifest(
    path: str | Path,
    *,
    git_head: str,
    git_tree: str,
    working_tree_clean: bool,
    public_case_id: str,
    side: str,
    profile_name: str,
    observation_seed: int,
    tick_limit: int,
    control_period_s: float,
    observation_period_s: float,
    source_file_hashes: Mapping[str, str],
) -> dict[str, Any]:
    """Write the immutable companion manifest for one public trace run."""

    output = Path(path)
    if output.exists():
        raise FileExistsError(f"R7 manifest output already exists: {output}")
    for name, value in (
        ("git_head", git_head),
        ("git_tree", git_tree),
        ("public_case_id", public_case_id),
        ("side", side),
        ("profile_name", profile_name),
    ):
        if not isinstance(value, str) or not value:
            raise ValueError(f"{name} must be a non-empty string")
    if not isinstance(working_tree_clean, bool):
        raise TypeError("working_tree_clean must be bool")
    for name, value in (("observation_seed", observation_seed), ("tick_limit", tick_limit)):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"{name} must be a non-negative exact integer")
    if tick_limit == 0:
        raise ValueError("tick_limit must be positive")
    for name, value in (
        ("control_period_s", control_period_s),
        ("observation_period_s", observation_period_s),
    ):
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise TypeError(f"{name} must be numeric")
        if not isfinite(float(value)) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
    if not isinstance(source_file_hashes, Mapping) or not source_file_hashes:
        raise ValueError("source_file_hashes must be a non-empty mapping")
    frozen_hashes: dict[str, str] = {}
    for source_name, source_hash in sorted(source_file_hashes.items()):
        if not isinstance(source_name, str) or not source_name:
            raise ValueError("source file names must be non-empty strings")
        if (
            not isinstance(source_hash, str)
            or len(source_hash) != 64
            or any(character not in "0123456789abcdef" for character in source_hash)
        ):
            raise ValueError("source file hashes must be lowercase SHA-256 strings")
        frozen_hashes[source_name] = source_hash
    payload: dict[str, Any] = {
        "schema": R7_FAILURE_RUN_MANIFEST_SCHEMA_VERSION,
        "git_head": git_head,
        "git_tree": git_tree,
        "working_tree_clean": working_tree_clean,
        "public_case_id": public_case_id,
        "side": side,
        "profile_name": profile_name,
        "observation_seed": observation_seed,
        "tick_limit": tick_limit,
        "control_period_s": float(control_period_s),
        "observation_period_s": float(observation_period_s),
        "source_file_hashes": frozen_hashes,
        "trace_schema_version": R7_FAILURE_TRACE_SCHEMA_VERSION,
    }
    payload["manifest_content_hash"] = canonical_content_hash(payload)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    return dict(payload)


__all__ = [
    "R7_FAILURE_RUN_MANIFEST_SCHEMA_VERSION",
    "R7_FAILURE_TRACE_SCHEMA_VERSION",
    "R7FailureTraceCollector",
    "write_r7_failure_run_manifest",
]
