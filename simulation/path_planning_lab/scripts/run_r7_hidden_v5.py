"""Reserve and run the one-use R7 hidden-v5 corrective qualification.

The historical hidden-v4 run is sealed as ``FAIL_ANALYZED``.  This runner has
its own execution, catalog, ledger, and receipt namespace.  It never rewrites
or invokes the historical v4 runner.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import platform
import re
import secrets
import subprocess
import tempfile
from collections.abc import Mapping, Sequence
from dataclasses import asdict
from datetime import UTC, datetime
from hashlib import sha256
from pathlib import Path
from types import ModuleType

from hospital_path_lab.r7_hidden_v4_qualification import (
    R7_HIDDEN_V4_REPLICA_COUNT,
    R7HiddenV4CaseSpec,
    audit_hidden_v4_results,
    evaluate_hidden_v4_cases,
)


def _load_v4_support() -> ModuleType:
    """Load only stable release-evidence helpers; do not execute hidden-v4."""

    path = Path(__file__).with_name("run_r7_hidden_v4.py")
    spec = importlib.util.spec_from_file_location("r7_hidden_v4_support", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("R7 hidden-v5 support runner could not be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_V4_SUPPORT = _load_v4_support()
canonical_content_hash = _V4_SUPPORT.canonical_content_hash

_EXECUTION_NAMESPACE = "r7-hidden-v5-execution-v1"
_OBSERVATION_NAMESPACE = "r7-hidden-observation-v5"
_CATALOG_SCHEMA = "r7-hidden-v5-case-catalog-v1"
_PREFLIGHT_SCHEMA = "r7-hidden-v5-preflight-v1"
_SEED_COMMITMENT_SCHEMA = "r7-hidden-v5-seed-commitment-v1"
_CONSUMED_SEED_SCHEMA = "r7-hidden-v5-consumed-seed-v1"
_PARTIAL_SCHEMA = "r7-hidden-v5-partial-v1"
_SUMMARY_SCHEMA = "r7-hidden-v5-summary-v1"
_RECEIPT_SCHEMA = "r7-hidden-v5-consumption-receipt-v1"
_LOCAL_LEDGER_SCHEMA = "r7-hidden-v5-execution-ledger-v1"
_REMOTE_RESERVATION_SCHEMA = "r7-hidden-v5-remote-reservation-v1"
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
    "simulation/path_planning_lab/scripts/run_r7_hidden_v5.py",
    "simulation/path_planning_lab/src/hospital_path_lab/"
    "r7_hidden_v4_qualification.py",
)
_MAX_ROOT_SEED = (1 << 63) - 1
_HISTORICAL_V4_CONSUMED_ROOT_SEED = 6_564_067_906_066_881_700
_KNOWN_CONSUMED_ROOT_SEEDS = frozenset(
    {
        _HISTORICAL_V4_CONSUMED_ROOT_SEED,
        *tuple(_V4_SUPPORT._KNOWN_CONSUMED_ROOT_SEEDS),
    }
)
_KNOWN_PUBLIC_OBSERVATION_SEEDS = frozenset(_V4_SUPPORT._KNOWN_PUBLIC_OBSERVATION_SEEDS)


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
        help="Validate without generating a seed or running a hidden case.",
    )
    parser.add_argument(
        "--reserve-remote",
        action="store_true",
        help="Atomically reserve this exact release on origin during preflight.",
    )
    parser.add_argument(
        "--designated-executor",
        help="Stable approved executor label, for example company-pc-r7.",
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
    expected_sha = _normalize_sha256(parser, args.qualification_sha256)
    designated_executor = _validate_designated_executor(parser, args.designated_executor)
    if args.reserve_remote and not args.preflight_only:
        parser.error("--reserve-remote is only allowed with --preflight-only")
    if args.reserve_remote and designated_executor is None:
        parser.error("--reserve-remote requires --designated-executor")
    if not args.preflight_only:
        if not args.execute_approved:
            parser.error("hidden execution requires --execute-approved")
        if designated_executor is None:
            parser.error("hidden execution requires --designated-executor")

    repository_root = _repository_root()
    output = args.output.resolve()
    if output.exists():
        parser.error("output path already exists; hidden outputs are never overwritten")
    if _git(repository_root, "status", "--porcelain=v1"):
        parser.error("hidden-v5 requires a clean Git working tree")

    preflight = _preflight(
        repository_root,
        args.qualification_evidence.resolve(),
        expected_sha,
    )
    identity = _execution_identity(preflight)
    if args.preflight_only:
        reservation = None
        if args.reserve_remote:
            assert designated_executor is not None
            reservation = _reserve_remote(
                repository_root,
                preflight,
                identity,
                designated_executor,
            )
        output.mkdir(parents=True)
        _write_json(
            output / "preflight-receipt.json",
            _build_preflight_receipt(
                preflight,
                identity,
                max_workers=args.max_workers,
                reservation=reservation,
            ),
        )
        print("preflight_passed=true", flush=True)
        print("hidden_seed_generated=false", flush=True)
        print(f"reservation_ref={identity['reservation_ref']}", flush=True)
        return 0

    assert designated_executor is not None
    claimed_reservation = _claim_remote_reservation(
        repository_root,
        preflight,
        identity,
        designated_executor,
    )
    output.mkdir(parents=True)
    _write_json(
        output / "preflight-receipt.json",
        _build_preflight_receipt(
            preflight,
            identity,
            max_workers=args.max_workers,
            reservation=claimed_reservation,
        ),
    )
    _write_json(output / "remote-reservation.json", claimed_reservation["record"])
    ledger_path = output / "hidden-v5-execution-ledger.json"
    _write_local_ledger(
        ledger_path,
        state="remote_claimed_before_seed",
        identity=identity,
        remote_commit=claimed_reservation["commit"],
    )

    commitment: str | None = None
    partial: list[object] = []
    try:
        root_seed = secrets.randbits(63)
        _validate_fresh_root_seed(root_seed)
        commitment = hidden_v5_seed_commitment(root_seed)
        specs, pre_run = _prepare_committed_hidden_v5(
            output=output,
            ledger_path=ledger_path,
            root_seed=root_seed,
            commitment=commitment,
            preflight=preflight,
            identity=identity,
            remote_commit=claimed_reservation["commit"],
            max_workers=args.max_workers,
        )

        def on_case(result: object) -> None:
            partial.append(result)
            _write_json(
                output / "partial-state.json",
                {
                    "schema": _PARTIAL_SCHEMA,
                    "execution_namespace": _EXECUTION_NAMESPACE,
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

        results = evaluate_hidden_v4_cases(
            repository_root,
            specs,
            max_workers=args.max_workers,
            on_case=on_case,
            failure_trace_root=output / "failure-traces",
        )
        postflight = _verify_execution_freeze(repository_root, preflight)
        return_code, receipt_hash, terminal_state = _finalize_hidden_v5(
            output=output,
            specs=specs,
            results=results,
            postflight=postflight,
            commitment=commitment,
            preflight=preflight,
            identity=identity,
            expected_sha=expected_sha,
            pre_run=pre_run,
            ledger_path=ledger_path,
        )
    except BaseException as exc:
        receipt_hash = _record_infrastructure_failure(
            output,
            commitment,
            len(partial),
            exc,
            identity,
        )
        _seal_local_ledger(
            ledger_path,
            state="infrastructure_failure",
            seed_commitment=commitment,
            receipt_content_hash=receipt_hash,
        )
        _finalize_remote_reservation(
            repository_root,
            preflight,
            identity,
            designated_executor,
            claimed_reservation["commit"],
            state="infrastructure_failure",
            seed_commitment=commitment,
            receipt_content_hash=receipt_hash,
        )
        raise

    _finalize_remote_reservation(
        repository_root,
        preflight,
        identity,
        designated_executor,
        claimed_reservation["commit"],
        state=terminal_state,
        seed_commitment=commitment,
        receipt_content_hash=receipt_hash,
    )
    print(f"hidden_v5_passed={str(return_code == 0).lower()}", flush=True)
    print(f"receipt={output / 'hidden-v5-consumption-receipt.json'}", flush=True)
    return return_code


def _normalize_sha256(parser: argparse.ArgumentParser, value: str) -> str:
    normalized = value.lower()
    if len(normalized) != 64 or any(ch not in "0123456789abcdef" for ch in normalized):
        parser.error("qualification-sha256 must be a lowercase 64-character SHA-256")
    return normalized


def _validate_designated_executor(
    parser: argparse.ArgumentParser,
    value: str | None,
) -> str | None:
    if value is None:
        return None
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{2,63}", value):
        parser.error("designated-executor must be 3-64 safe identifier characters")
    return value


def hidden_v5_seed_commitment(root_seed: int) -> str:
    _validate_fresh_root_seed(root_seed)
    return sha256(f"{_OBSERVATION_NAMESPACE}:{root_seed}".encode()).hexdigest()


def _validate_fresh_root_seed(root_seed: int) -> None:
    if (
        isinstance(root_seed, bool)
        or not isinstance(root_seed, int)
        or not 0 <= root_seed <= _MAX_ROOT_SEED
    ):
        raise ValueError("root_seed must be a non-negative signed 63-bit exact integer")
    if root_seed in _KNOWN_CONSUMED_ROOT_SEEDS:
        raise ValueError("generated root seed was already consumed and is permanently rejected")


def build_hidden_v5_case_specs(root_seed: int) -> tuple[R7HiddenV4CaseSpec, ...]:
    """Build a v5-labelled catalog with a new observation derivation namespace."""

    _validate_fresh_root_seed(root_seed)
    specs: list[R7HiddenV4CaseSpec] = []
    for replica in range(R7_HIDDEN_V4_REPLICA_COUNT):
        for side_index, side_name in enumerate(("left", "right")):
            observation_seed = _derived_v5_observation_seed(
                root_seed,
                replica=replica,
                side_name=side_name,
            )
            seed_tag = sha256(str(observation_seed).encode("ascii")).hexdigest()
            for profile_name, expected_outcome in (
                ("normal", "completed"),
                ("stress", "conditionally_safe_hold"),
            ):
                specs.append(
                    R7HiddenV4CaseSpec(
                        ordinal=len(specs),
                        case_id=f"hidden-v5-{replica:02d}-{side_name}-{profile_name}",
                        replica=replica,
                        side_index=side_index,
                        side_name=side_name,
                        profile_name=profile_name,
                        observation_seed=observation_seed,
                        seed_tag=seed_tag,
                        expected_outcome=expected_outcome,
                    )
                )
    result = tuple(specs)
    if len(result) != 20:
        raise RuntimeError("R7 hidden-v5 catalog size mismatch")
    if len({item.case_id for item in result}) != len(result):
        raise RuntimeError("R7 hidden-v5 catalog contains duplicate case IDs")
    if tuple(item.ordinal for item in result) != tuple(range(len(result))):
        raise RuntimeError("R7 hidden-v5 catalog ordinals are not contiguous")
    for replica in range(R7_HIDDEN_V4_REPLICA_COUNT):
        for side_name in ("left", "right"):
            paired = tuple(
                item
                for item in result
                if item.replica == replica and item.side_name == side_name
            )
            if len(paired) != 2 or paired[0].observation_seed != paired[1].observation_seed:
                raise RuntimeError("Normal and Stress must share one latent observation seed")
    return result


def _derived_v5_observation_seed(root_seed: int, *, replica: int, side_name: str) -> int:
    encoded = f"{_OBSERVATION_NAMESPACE}:{root_seed}:{replica}:{side_name}".encode()
    return int.from_bytes(sha256(encoded).digest()[:8], byteorder="big") & _MAX_ROOT_SEED


def _repository_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _preflight(
    repository_root: Path,
    evidence_path: Path,
    expected_evidence_sha256: str,
) -> dict[str, object]:
    release_evidence = _V4_SUPPORT._verify_release_evidence(
        repository_root,
        evidence_path,
        expected_evidence_sha256,
    )
    _V4_SUPPORT._verify_native_libraries(repository_root, release_evidence)
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


def _verify_execution_freeze(
    repository_root: Path,
    preflight: dict[str, object],
) -> dict[str, object]:
    if _git(repository_root, "status", "--porcelain=v1"):
        raise RuntimeError("hidden-v5 working tree changed during execution")
    head = _git(repository_root, "rev-parse", "HEAD")
    tree = _git(repository_root, "rev-parse", "HEAD^{tree}")
    if head != preflight["head"] or tree != preflight["tree"]:
        raise RuntimeError("hidden-v5 Git identity changed during execution")
    release_before = preflight["release_evidence"]
    release_after = _V4_SUPPORT._verify_release_evidence(
        repository_root,
        Path(str(release_before["path"])),
        str(release_before["sha256"]),
    )
    if release_after != release_before:
        raise RuntimeError("R7 release evidence changed during hidden-v5 execution")
    packaging_after = _packaging_source_freeze(repository_root)
    if packaging_after != preflight["packaging_source_freeze"]:
        raise RuntimeError("hidden-v5 packaging source changed during execution")
    _V4_SUPPORT._verify_native_libraries(repository_root, release_after)
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
            raise RuntimeError(f"hidden-v5 packaging source is missing: {relative_path}")
        _git(repository_root, "ls-files", "--error-unmatch", relative_path)
        if _run_git(
            repository_root,
            "diff",
            "--quiet",
            "HEAD",
            "--",
            relative_path,
            check=False,
        ).returncode != 0:
            raise RuntimeError(f"hidden-v5 packaging source is not committed: {relative_path}")
        records.append(
            {
                "path": relative_path,
                "size_bytes": path.stat().st_size,
                "sha256": _sha256(path),
            }
        )
    payload = {"records": records}
    return {**payload, "content_hash": canonical_content_hash(payload)}


def _execution_identity(preflight: Mapping[str, object]) -> dict[str, str]:
    release = preflight["release_evidence"]
    packaging = preflight["packaging_source_freeze"]
    if not isinstance(release, Mapping) or not isinstance(packaging, Mapping):
        raise RuntimeError("hidden-v5 preflight identity is incomplete")
    payload = {
        "execution_namespace": _EXECUTION_NAMESPACE,
        "observation_namespace": _OBSERVATION_NAMESPACE,
        "head": preflight["head"],
        "tree": preflight["tree"],
        "release_evidence_sha256": release["sha256"],
        "release_receipt_content_hash": release["receipt_content_hash"],
        "packaging_source_freeze_hash": packaging["content_hash"],
    }
    execution_id = canonical_content_hash(payload)
    return {
        **{key: str(value) for key, value in payload.items()},
        "execution_id": execution_id,
        # One fixed remote ref makes v5 globally one-use.  A later source or
        # document commit must not silently create a second v5 execution ID;
        # any genuinely new study needs a new versioned runner/ref.
        "reservation_ref": "refs/heads/codex/r7-hidden-v5-reservation",
        "artifact_path": f".r7-hidden-reservations/r7-hidden-v5/{execution_id}.json",
    }


def _build_preflight_receipt(
    preflight: Mapping[str, object],
    identity: Mapping[str, str],
    *,
    max_workers: int,
    reservation: Mapping[str, object] | None,
) -> dict[str, object]:
    payload = {
        "schema": _PREFLIGHT_SCHEMA,
        "checked_at_utc": datetime.now(UTC).isoformat(),
        "execution_namespace": _EXECUTION_NAMESPACE,
        "observation_namespace": _OBSERVATION_NAMESPACE,
        **preflight,
        "execution_id": identity["execution_id"],
        "reservation_ref": identity["reservation_ref"],
        "max_workers_if_approved": max_workers,
        "remote_reservation": reservation,
        "hidden_seed_generated": False,
        "hidden_executed": False,
        "product_or_human_safety_claim": False,
    }
    return {**payload, "receipt_content_hash": canonical_content_hash(payload)}


def _reservation_record(
    identity: Mapping[str, str],
    *,
    designated_executor: str,
    state: str,
    reserved_at_utc: str | None = None,
    receipt_content_hash: str | None = None,
    seed_commitment: str | None = None,
) -> dict[str, object]:
    payload = {
        "schema": _REMOTE_RESERVATION_SCHEMA,
        "execution_namespace": _EXECUTION_NAMESPACE,
        "observation_namespace": _OBSERVATION_NAMESPACE,
        "catalog_schema": _CATALOG_SCHEMA,
        "execution_id": identity["execution_id"],
        "reservation_ref": identity["reservation_ref"],
        "artifact_path": identity["artifact_path"],
        "head": identity["head"],
        "tree": identity["tree"],
        "release_evidence_sha256": identity["release_evidence_sha256"],
        "release_receipt_content_hash": identity["release_receipt_content_hash"],
        "packaging_source_freeze_hash": identity["packaging_source_freeze_hash"],
        "designated_executor": designated_executor,
        "executor_machine": platform.node(),
        "state": state,
        "reserved_at_utc": reserved_at_utc or datetime.now(UTC).isoformat(),
        "updated_at_utc": datetime.now(UTC).isoformat(),
        "seed_commitment": seed_commitment,
        "receipt_content_hash": receipt_content_hash,
        "raw_root_seed_in_remote_artifact": False,
    }
    return {**payload, "content_hash": canonical_content_hash(payload)}


def _reserve_remote(
    repository_root: Path,
    preflight: Mapping[str, object],
    identity: Mapping[str, str],
    designated_executor: str,
) -> dict[str, object]:
    _require_origin_contains_head(repository_root, str(preflight["head"]))
    if _remote_ref_hash(repository_root, identity["reservation_ref"]) is not None:
        raise RuntimeError("hidden-v5 is already reserved or consumed for this exact release")
    record = _reservation_record(
        identity,
        designated_executor=designated_executor,
        state="reserved_before_seed",
    )
    commit = _commit_reservation_artifact(
        repository_root,
        parent=str(preflight["head"]),
        artifact_path=identity["artifact_path"],
        record=record,
        message="reserve R7 hidden-v5 execution before seed",
    )
    _push_reservation_commit(repository_root, commit, identity["reservation_ref"])
    if _remote_ref_hash(repository_root, identity["reservation_ref"]) != commit:
        raise RuntimeError("hidden-v5 remote reservation could not be verified")
    return {"commit": commit, "record": record}


def _require_origin_contains_head(repository_root: Path, head: str) -> None:
    refs = _git(repository_root, "ls-remote", "--heads", "origin")
    remote_heads = {
        line.split(maxsplit=1)[0]
        for line in refs.splitlines()
        if len(line.split(maxsplit=1)) == 2
    }
    if head not in remote_heads:
        raise RuntimeError(
            "hidden-v5 source commit is not published on origin; push the reviewed source first"
        )


def _claim_remote_reservation(
    repository_root: Path,
    preflight: Mapping[str, object],
    identity: Mapping[str, str],
    designated_executor: str,
) -> dict[str, object]:
    remote = _load_remote_reservation(repository_root, identity)
    _validate_remote_reservation(
        remote["record"],
        identity,
        designated_executor=designated_executor,
        required_state="reserved_before_seed",
    )
    record = _reservation_record(
        identity,
        designated_executor=designated_executor,
        state="execution_started_before_seed",
        reserved_at_utc=str(remote["record"]["reserved_at_utc"]),
    )
    commit = _commit_reservation_artifact(
        repository_root,
        parent=str(remote["commit"]),
        artifact_path=identity["artifact_path"],
        record=record,
        message="claim R7 hidden-v5 execution before seed",
    )
    _push_reservation_commit(repository_root, commit, identity["reservation_ref"])
    if _remote_ref_hash(repository_root, identity["reservation_ref"]) != commit:
        raise RuntimeError("hidden-v5 remote claim could not be verified")
    if str(preflight["head"]) != identity["head"]:
        raise RuntimeError("hidden-v5 preflight identity changed before seed")
    return {"commit": commit, "record": record}


def _finalize_remote_reservation(
    repository_root: Path,
    preflight: Mapping[str, object],
    identity: Mapping[str, str],
    designated_executor: str,
    claimed_commit: str,
    *,
    state: str,
    seed_commitment: str | None,
    receipt_content_hash: str,
) -> None:
    remote = _load_remote_reservation(repository_root, identity)
    if remote["commit"] != claimed_commit:
        raise RuntimeError("hidden-v5 remote reservation changed after execution started")
    _validate_remote_reservation(
        remote["record"],
        identity,
        designated_executor=designated_executor,
        required_state="execution_started_before_seed",
    )
    record = _reservation_record(
        identity,
        designated_executor=designated_executor,
        state=state,
        reserved_at_utc=str(remote["record"]["reserved_at_utc"]),
        seed_commitment=seed_commitment,
        receipt_content_hash=receipt_content_hash,
    )
    commit = _commit_reservation_artifact(
        repository_root,
        parent=claimed_commit,
        artifact_path=identity["artifact_path"],
        record=record,
        message="seal R7 hidden-v5 execution receipt",
    )
    _push_reservation_commit(repository_root, commit, identity["reservation_ref"])
    if _remote_ref_hash(repository_root, identity["reservation_ref"]) != commit:
        raise RuntimeError("hidden-v5 remote final receipt could not be verified")
    if str(preflight["tree"]) != identity["tree"]:
        raise RuntimeError("hidden-v5 preflight tree changed while sealing receipt")


def _load_remote_reservation(
    repository_root: Path,
    identity: Mapping[str, str],
) -> dict[str, object]:
    expected_commit = _remote_ref_hash(repository_root, identity["reservation_ref"])
    if expected_commit is None:
        raise RuntimeError("hidden-v5 has no remote reservation; preflight reserve it first")
    _git(repository_root, "fetch", "--no-tags", "origin", identity["reservation_ref"])
    fetched_commit = _git(repository_root, "rev-parse", "FETCH_HEAD")
    if fetched_commit != expected_commit:
        raise RuntimeError("hidden-v5 reservation changed while it was being fetched")
    raw = _git(
        repository_root,
        "cat-file",
        "blob",
        f"{fetched_commit}:./{identity['artifact_path']}",
    )
    try:
        record = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise RuntimeError("hidden-v5 remote reservation JSON is invalid") from exc
    if not isinstance(record, dict):
        raise RuntimeError("hidden-v5 remote reservation record is invalid")
    _validate_record_content_hash(record)
    return {"commit": fetched_commit, "record": record}


def _validate_remote_reservation(
    record: Mapping[str, object],
    identity: Mapping[str, str],
    *,
    designated_executor: str,
    required_state: str,
) -> None:
    expected = {
        "schema": _REMOTE_RESERVATION_SCHEMA,
        "execution_namespace": _EXECUTION_NAMESPACE,
        "observation_namespace": _OBSERVATION_NAMESPACE,
        "catalog_schema": _CATALOG_SCHEMA,
        "execution_id": identity["execution_id"],
        "reservation_ref": identity["reservation_ref"],
        "artifact_path": identity["artifact_path"],
        "head": identity["head"],
        "tree": identity["tree"],
        "release_evidence_sha256": identity["release_evidence_sha256"],
        "release_receipt_content_hash": identity["release_receipt_content_hash"],
        "packaging_source_freeze_hash": identity["packaging_source_freeze_hash"],
        "designated_executor": designated_executor,
        "executor_machine": platform.node(),
        "state": required_state,
        "raw_root_seed_in_remote_artifact": False,
    }
    if any(record.get(key) != value for key, value in expected.items()):
        raise RuntimeError("hidden-v5 remote reservation does not match this executor/release")


def _validate_record_content_hash(record: Mapping[str, object]) -> None:
    payload = {key: value for key, value in record.items() if key != "content_hash"}
    if record.get("content_hash") != canonical_content_hash(payload):
        raise RuntimeError("hidden-v5 remote reservation content hash mismatch")


def _remote_ref_hash(repository_root: Path, ref: str) -> str | None:
    output = _git(repository_root, "ls-remote", "--refs", "origin", ref)
    if not output:
        return None
    lines = output.splitlines()
    if len(lines) != 1:
        raise RuntimeError("hidden-v5 remote reservation ref is ambiguous")
    try:
        commit, actual_ref = lines[0].split(maxsplit=1)
    except ValueError as exc:
        raise RuntimeError("hidden-v5 remote reservation ref is malformed") from exc
    if actual_ref != ref or not re.fullmatch(r"[0-9a-f]{40,64}", commit):
        raise RuntimeError("hidden-v5 remote reservation ref is malformed")
    return commit


def _commit_reservation_artifact(
    repository_root: Path,
    *,
    parent: str,
    artifact_path: str,
    record: Mapping[str, object],
    message: str,
) -> str:
    serialized = json.dumps(record, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    with tempfile.TemporaryDirectory(prefix="r7-hidden-v5-index-") as temporary_directory:
        index_path = Path(temporary_directory) / "index"
        environment = {**os.environ, "GIT_INDEX_FILE": str(index_path)}
        _git(repository_root, "read-tree", parent, environment=environment)
        blob = _git(
            repository_root,
            "hash-object",
            "-w",
            "--stdin",
            input_text=serialized,
            environment=environment,
        )
        _git(
            repository_root,
            "update-index",
            "--add",
            "--cacheinfo",
            f"100644,{blob},{artifact_path}",
            environment=environment,
        )
        tree = _git(repository_root, "write-tree", environment=environment)
        return _git(
            repository_root,
            "commit-tree",
            tree,
            "-p",
            parent,
            "-m",
            message,
            environment=environment,
        )


def _push_reservation_commit(repository_root: Path, commit: str, ref: str) -> None:
    try:
        _git(repository_root, "push", "origin", f"{commit}:{ref}")
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            "hidden-v5 remote reservation was changed or rejected; no seed was generated"
        ) from exc


def _prepare_committed_hidden_v5(
    *,
    output: Path,
    ledger_path: Path,
    root_seed: int,
    commitment: str,
    preflight: Mapping[str, object],
    identity: Mapping[str, str],
    remote_commit: str,
    max_workers: int,
) -> tuple[tuple[R7HiddenV4CaseSpec, ...], dict[str, object]]:
    started_at = datetime.now(UTC).isoformat()
    _write_json(
        output / "seed-commitment.json",
        {
            "schema": _SEED_COMMITMENT_SCHEMA,
            "execution_namespace": _EXECUTION_NAMESPACE,
            "observation_namespace": _OBSERVATION_NAMESPACE,
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
    _seal_local_ledger(
        ledger_path,
        state="seed_consumed",
        seed_commitment=commitment,
    )
    specs = build_hidden_v5_case_specs(root_seed)
    if any(item.observation_seed in _KNOWN_PUBLIC_OBSERVATION_SEEDS for item in specs):
        raise RuntimeError("generated observation seed was already public or consumed")
    case_catalog_hash = canonical_content_hash(tuple(item.content_hash for item in specs))
    pre_run = {
        "schema": _CATALOG_SCHEMA,
        "execution_namespace": _EXECUTION_NAMESPACE,
        "observation_namespace": _OBSERVATION_NAMESPACE,
        "started_at_utc": started_at,
        "head": preflight["head"],
        "tree": preflight["tree"],
        "working_tree_clean": True,
        "seed_commitment": commitment,
        "root_seed_disclosed_before_commitment": False,
        "commitment_written_before_seed_derivation": True,
        "case_count": len(specs),
        "case_catalog_hash": case_catalog_hash,
        "case_id_prefix": "hidden-v5-",
        "normal_case_count": sum(item.profile_name == "normal" for item in specs),
        "stress_case_count": sum(item.profile_name == "stress" for item in specs),
        "max_workers": max_workers,
        "python_wall_clock_is_qualification": False,
        "release_evidence": preflight["release_evidence"],
        "packaging_source_freeze": preflight["packaging_source_freeze"],
        "execution_id": identity["execution_id"],
        "reservation_ref": identity["reservation_ref"],
        "remote_claim_commit": remote_commit,
        "hidden_seed_generated": True,
        "hidden_executed": True,
        "product_or_human_safety_claim": False,
    }
    _write_json(output / "pre-run-manifest.json", pre_run)
    return specs, pre_run


def _finalize_hidden_v5(
    *,
    output: Path,
    specs: Sequence[R7HiddenV4CaseSpec],
    results: Sequence[object],
    postflight: Mapping[str, object],
    commitment: str,
    preflight: Mapping[str, object],
    identity: Mapping[str, str],
    expected_sha: str,
    pre_run: Mapping[str, object],
    ledger_path: Path,
) -> tuple[int, str, str]:
    audit = audit_hidden_v4_results(tuple(specs), tuple(results))
    failure_trace_manifest = _failure_trace_manifest(output / "failure-traces", results=results)
    case_trace_set_hash = _case_trace_set_hash(results)
    _write_json(output / "case-results.json", results)
    _write_json(output / "failure-trace-manifest.json", failure_trace_manifest)
    summary = {
        "schema": _SUMMARY_SCHEMA,
        "execution_namespace": _EXECUTION_NAMESPACE,
        "observation_namespace": _OBSERVATION_NAMESPACE,
        "catalog_schema": _CATALOG_SCHEMA,
        "final_status": "PASS_FINAL" if audit.passed else "FAIL_REQUIRES_ANALYSIS",
        "passed": audit.passed,
        "case_count": audit.result_count,
        "normal_completed_count": audit.normal_completed_count,
        "stress_conditionally_safe_count": audit.stress_conditionally_safe_count,
        "stress_release_count": audit.stress_release_count,
        "hard_failure_count": audit.hard_failure_count,
        "release_contract_violation_count": audit.release_contract_violation_count,
        "duplicate_safe_frame_violation_count": audit.duplicate_safe_frame_violation_count,
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
        "execution_id": identity["execution_id"],
        "reservation_ref": identity["reservation_ref"],
        "case_catalog_hash": pre_run["case_catalog_hash"],
        "hidden_scope": "new_observation_noise_and_dropout_sequences_only",
        "product_or_human_safety_claim": False,
        "postflight": postflight,
        "failure_trace_manifest_hash": failure_trace_manifest["content_hash"],
        "case_trace_set_hash": case_trace_set_hash,
    }
    _write_json(output / "summary.json", summary)
    (output / "summary.md").write_text(_summary_markdown(summary, results), encoding="utf-8")
    receipt_payload = {
        "schema": _RECEIPT_SCHEMA,
        "execution_namespace": _EXECUTION_NAMESPACE,
        "observation_namespace": _OBSERVATION_NAMESPACE,
        "catalog_schema": _CATALOG_SCHEMA,
        "completed": True,
        "passed": audit.passed,
        "head": preflight["head"],
        "tree": preflight["tree"],
        "execution_id": identity["execution_id"],
        "reservation_ref": identity["reservation_ref"],
        "seed_commitment": commitment,
        "case_catalog_hash": pre_run["case_catalog_hash"],
        "result_set_hash": audit.result_set_hash,
        "case_count": audit.result_count,
        "normal_completed_count": audit.normal_completed_count,
        "stress_conditionally_safe_count": audit.stress_conditionally_safe_count,
        "stress_release_count": audit.stress_release_count,
        "hard_failure_count": audit.hard_failure_count,
        "release_contract_violation_count": audit.release_contract_violation_count,
        "duplicate_safe_frame_violation_count": audit.duplicate_safe_frame_violation_count,
        "stale_propulsion_violation_count": audit.stale_propulsion_violation_count,
        "unauthorized_restart_count": audit.unauthorized_restart_count,
        "actual_collision_count": audit.actual_collision_count,
        "actual_forbidden_violation_count": audit.actual_forbidden_violation_count,
        "actual_clearance_violation_count": audit.actual_clearance_violation_count,
        "failure_trace_manifest_hash": failure_trace_manifest["content_hash"],
        "case_trace_set_hash": case_trace_set_hash,
        "release_receipt_content_hash": preflight["release_evidence"]["receipt_content_hash"],
        "release_evidence_sha256": expected_sha,
        "packaging_source_freeze_hash": preflight["packaging_source_freeze"]["content_hash"],
        "postflight_content_hash": canonical_content_hash(postflight),
        "reuse_as_final_hidden_after_code_change": False,
    }
    receipt = {
        **receipt_payload,
        "receipt_content_hash": canonical_content_hash(receipt_payload),
    }
    _write_json(output / "hidden-v5-consumption-receipt.json", receipt)
    _seal_local_ledger(
        ledger_path,
        state="completed_pass" if audit.passed else "completed_fail",
        seed_commitment=commitment,
        receipt_content_hash=receipt["receipt_content_hash"],
    )
    (output / "partial-state.json").unlink(missing_ok=True)
    return (
        0 if audit.passed else 1,
        str(receipt["receipt_content_hash"]),
        "completed_pass" if audit.passed else "completed_fail",
    )


def _failure_trace_manifest(root: Path, *, results: Sequence[object]) -> dict[str, object]:
    paths = tuple(sorted(root.rglob("tick-trace.jsonl"))) if root.exists() else ()
    if len(paths) != len(results):
        raise RuntimeError("hidden-v5 case trace count does not match case results")
    results_by_id = {str(item.case_id): item for item in results}
    if len(results_by_id) != len(results):
        raise RuntimeError("hidden-v5 case results contain duplicate case IDs")
    records = []
    for path in paths:
        case_id = path.parent.name
        result = results_by_id.get(case_id)
        if result is None:
            raise RuntimeError(f"hidden-v5 case trace has no result: {case_id}")
        lines = path.read_text(encoding="utf-8").splitlines()
        if not lines:
            raise RuntimeError(f"hidden-v5 failure trace is empty: {path}")
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
            raise RuntimeError(f"hidden-v5 case trace binding mismatch: {case_id}")
        records.append(record)
    payload = {
        "schema": "r7-hidden-v5-case-trace-manifest-v1",
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


def _record_infrastructure_failure(
    output: Path,
    seed_commitment: str | None,
    completed_case_count: int,
    error: BaseException,
    identity: Mapping[str, str],
) -> str:
    trace_manifest = _partial_trace_manifest(output / "failure-traces")
    failure_payload = {
        "schema": "r7-hidden-v5-infrastructure-failure-v1",
        "execution_namespace": _EXECUTION_NAMESPACE,
        "final_status": "BLOCKED_INFRASTRUCTURE",
        "completed": False,
        "algorithm_verdict": None,
        "seed_commitment": seed_commitment,
        "completed_case_count": completed_case_count,
        "execution_id": identity["execution_id"],
        "error_type": type(error).__name__,
        "error_message": str(error),
        "partial_is_final_evidence": False,
        "partial_trace_manifest_hash": trace_manifest["content_hash"],
    }
    failure = {**failure_payload, "content_hash": canonical_content_hash(failure_payload)}
    _write_json(output / "infrastructure-failure.json", failure)
    _write_json(output / "partial-trace-manifest.json", trace_manifest)
    summary_payload = {
        "schema": _SUMMARY_SCHEMA,
        "execution_namespace": _EXECUTION_NAMESPACE,
        "final_status": "BLOCKED_INFRASTRUCTURE",
        "completed": False,
        "passed": False,
        "algorithm_verdict": None,
        "seed_commitment": seed_commitment,
        "completed_case_count": completed_case_count,
        "execution_id": identity["execution_id"],
        "infrastructure_failure_hash": failure["content_hash"],
        "partial_trace_manifest_hash": trace_manifest["content_hash"],
        "partial_is_final_evidence": False,
    }
    summary = {**summary_payload, "content_hash": canonical_content_hash(summary_payload)}
    _write_json(output / "summary.json", summary)
    receipt_payload = {
        "schema": _RECEIPT_SCHEMA,
        "execution_namespace": _EXECUTION_NAMESPACE,
        "final_status": "BLOCKED_INFRASTRUCTURE",
        "completed": False,
        "passed": False,
        "algorithm_verdict": None,
        "seed_commitment": seed_commitment,
        "completed_case_count": completed_case_count,
        "execution_id": identity["execution_id"],
        "infrastructure_failure_hash": failure["content_hash"],
        "summary_content_hash": summary["content_hash"],
        "partial_trace_manifest_hash": trace_manifest["content_hash"],
        "reuse_as_final_hidden_after_code_change": False,
    }
    receipt = {
        **receipt_payload,
        "receipt_content_hash": canonical_content_hash(receipt_payload),
    }
    _write_json(output / "hidden-v5-consumption-receipt.json", receipt)
    return str(receipt["receipt_content_hash"])


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
        "schema": "r7-hidden-v5-partial-trace-manifest-v1",
        "trace_file_count": len(records),
        "records": records,
    }
    return {**payload, "content_hash": canonical_content_hash(payload)}


def _write_local_ledger(
    path: Path,
    *,
    state: str,
    identity: Mapping[str, str],
    remote_commit: str,
) -> None:
    payload = {
        "schema": _LOCAL_LEDGER_SCHEMA,
        "state": state,
        "execution_id": identity["execution_id"],
        "reservation_ref": identity["reservation_ref"],
        "remote_claim_commit": remote_commit,
        "seed_commitment": None,
        "receipt_content_hash": None,
        "updated_at_utc": datetime.now(UTC).isoformat(),
    }
    _write_json(path, {**payload, "content_hash": canonical_content_hash(payload)})


def _seal_local_ledger(
    path: Path,
    *,
    state: str,
    seed_commitment: str | None = None,
    receipt_content_hash: str | None = None,
) -> None:
    current = json.loads(path.read_text(encoding="utf-8"))
    current.update(
        {
            "state": state,
            "seed_commitment": seed_commitment,
            "receipt_content_hash": receipt_content_hash,
            "updated_at_utc": datetime.now(UTC).isoformat(),
        }
    )
    payload = {key: value for key, value in current.items() if key != "content_hash"}
    _write_json(path, {**payload, "content_hash": canonical_content_hash(payload)})


def _run_git(
    repository_root: Path,
    *args: str,
    input_text: str | None = None,
    environment: Mapping[str, str] | None = None,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ("git", *args),
        cwd=repository_root,
        check=check,
        capture_output=True,
        text=True,
        encoding="utf-8",
        input=input_text,
        env=dict(environment) if environment is not None else None,
    )


def _git(
    repository_root: Path,
    *args: str,
    input_text: str | None = None,
    environment: Mapping[str, str] | None = None,
) -> str:
    return _run_git(
        repository_root,
        *args,
        input_text=input_text,
        environment=environment,
    ).stdout.strip()


def _sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, value: object) -> None:
    temporary = path.with_name(f"{path.name}.tmp")
    temporary.write_text(
        json.dumps(_json_value(value), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _json_value(value: object) -> object:
    if hasattr(value, "__dataclass_fields__"):
        return _json_value(asdict(value))
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_json_value(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def _summary_markdown(summary: Mapping[str, object], results: Sequence[object]) -> str:
    lines = [
        "# R7 hidden-v5 관측 시험 결과",
        "",
        f"- 판정: `{'PASS' if summary['passed'] else 'FAIL'}`",
        f"- case: `{summary['case_count']}/20`",
        f"- Normal 완료: `{summary['normal_completed_count']}/10`",
        "- Stress 조건부 안전: " f"`{summary['stress_conditionally_safe_count']}/10`",
        f"- Stress 출발 사례: `{summary['stress_release_count']}/10`",
        f"- hard failure: `{summary['hard_failure_count']}`",
        f"- seed commitment: `{summary['seed_commitment']}`",
        "- 범위: 새 합성 관측 순서만, 실제 카메라·사람 안전 증거 아님",
        "",
        "| case | profile | outcome | pass | hard |",
        "|---|---|---|---:|---:|",
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
