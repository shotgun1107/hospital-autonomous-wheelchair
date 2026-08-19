from __future__ import annotations

import importlib.util
import json
from collections.abc import Callable
from pathlib import Path
from types import ModuleType, SimpleNamespace
from zipfile import ZIP_DEFLATED, ZipFile

import pytest

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
LAB_ROOT = REPOSITORY_ROOT / "simulation/path_planning_lab"
RELEASE_GATE_PATH = LAB_ROOT / "scripts/run_r7_native_release_gate.py"
HIDDEN_V4_PATH = LAB_ROOT / "scripts/run_r7_hidden_v4.py"
HISTORICAL_HIDDEN_PATH = LAB_ROOT / "scripts/run_r7_hidden_observation.py"


def _load_script(module_name: str, path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_zip(path: Path, payloads: dict[str, bytes]) -> None:
    with ZipFile(path, mode="w", compression=ZIP_DEFLATED) as archive:
        for name, payload in payloads.items():
            archive.writestr(name, payload)


def _json_bytes(value: object) -> bytes:
    return (json.dumps(value, sort_keys=True) + "\n").encode()


def _read_zip(path: Path) -> dict[str, bytes]:
    with ZipFile(path) as archive:
        return {name: archive.read(name) for name in archive.namelist()}


def _mutate_json_member(
    path: Path,
    member: str,
    mutate: Callable[[dict[str, object]], None],
) -> None:
    payloads = _read_zip(path)
    value = json.loads(payloads[member])
    mutate(value)
    payloads[member] = _json_bytes(value)
    _write_zip(path, payloads)


def _make_valid_release_evidence(
    tmp_path: Path,
    runner: ModuleType,
) -> tuple[Path, Path, dict[str, object]]:
    repository_root = tmp_path / "repository"
    repository_root.mkdir()
    source = repository_root / "frozen.py"
    source.write_bytes(b"print('frozen')\n")
    native = (
        repository_root
        / "simulation/path_planning_lab/src/hospital_path_lab/_native"
    )
    native.mkdir(parents=True)
    full = native / "dwb_full_core.dll"
    safety = native / "dwb_safety_core.dll"
    full.write_bytes(b"full-native-library")
    safety.write_bytes(b"safety-native-library")

    source_record = {
        "path": "frozen.py",
        "size": source.stat().st_size,
        "sha256": runner._sha256(source),
    }
    source_freeze = {
        "records": [source_record],
        "content_hash": runner.canonical_content_hash([source_record]),
    }
    head = "a" * 40
    tree = "b" * 40
    timing = {
        "passed": True,
        "sample_count": 500,
        "deadline_ns": 50_000_000,
        "warmups_per_case": 30,
        "repeats_per_case": 100,
        "parallelized": False,
        "execution_mode": "serial_parent_no_worker",
        "aggregate": {
            "deadline_miss_count": 0,
            "p50_ns": 12_000_000,
            "p95_ns": 28_000_000,
            "maximum_ns": 38_000_000,
        },
        "cases": [
            {
                "case_id": case_id,
                "sample_count": 100,
                "deadline_miss_count": 0,
                "deadline_ns": 50_000_000,
                "maximum_ns": 38_000_000,
            }
            for case_id in (
                "actor-0-free",
                "actor-1-active",
                "actor-2-active",
                "corner-static-forbidden",
                "staggered-risk-multisegment",
            )
        ],
    }
    parity = {
        "passed": True,
        "records": [{"passed": True, "case_id": str(index)} for index in range(5)],
        "content_hash": runner.canonical_content_hash(
            [{"passed": True, "case_id": str(index)} for index in range(5)]
        ),
    }
    contract_payload = {
        "schema": "r7-native-contract-parity-v1",
        "passed": True,
        "return_code": 0,
        "expected_test_count": 13,
        "passed_test_count": 13,
        "test_node_ids": ("frozen-contract-tests",),
        "output": "13 passed",
    }
    contract_parity = {
        **contract_payload,
        "content_hash": runner.canonical_content_hash(contract_payload),
    }
    manifest = {
        "git_before": {"head": head, "tree": tree},
        "git_after": {"head": head, "tree": tree},
        "source_freeze_before": source_freeze,
        "source_freeze_after": source_freeze,
        "hidden_executed": False,
        "warmups": 30,
        "repeats": 100,
        "build_executed": True,
    }
    release_gate = {
        "qualified": True,
        "eligible_for_user_hidden_approval": True,
        "hidden_executed": False,
        "checks": {"all_required_checks": True},
    }
    receipt_payload = {
        "head": head,
        "tree": tree,
        "source_freeze_hash": source_freeze["content_hash"],
        "semantic_parity_hash": parity["content_hash"],
        "contract_parity_hash": contract_parity["content_hash"],
        "timing_result_hash": runner.canonical_content_hash(timing),
        "native_full_library_sha256": runner._sha256(full),
        "native_safety_library_sha256": runner._sha256(safety),
        "sample_count": 500,
        "deadline_miss_count": 0,
        "deadline_ns": 50_000_000,
        "hidden_executed": False,
    }
    receipt = {
        **receipt_payload,
        "receipt_content_hash": runner.canonical_content_hash(receipt_payload),
    }
    evidence_path = tmp_path / "release-evidence.zip"
    _write_zip(
        evidence_path,
        {
            "run-manifest.json": _json_bytes(manifest),
            "semantic-parity.json": _json_bytes(parity),
            "contract-parity.json": _json_bytes(contract_parity),
            "timing-qualification.json": _json_bytes(timing),
            "release-gate.json": _json_bytes(release_gate),
            "qualification-receipt.json": _json_bytes(receipt),
            "summary.md": b"qualified\n",
        },
    )
    return repository_root, evidence_path, {
        "source": source,
        "full": full,
        "safety": safety,
    }


def test_release_evidence_zip_is_byte_deterministic(tmp_path: Path) -> None:
    release_gate = _load_script("r7_release_gate_zip_test", RELEASE_GATE_PATH)
    output = tmp_path / "qualified"
    output.mkdir()
    for index, name in enumerate(release_gate._EVIDENCE_FILES):
        (output / name).write_bytes(f"evidence-{index}\n".encode())
    first = tmp_path / "first.zip"
    second = tmp_path / "second.zip"

    first_record = release_gate._write_evidence_zip(output, first)
    second_record = release_gate._write_evidence_zip(output, second)

    assert first.read_bytes() == second.read_bytes()
    assert first_record["sha256"] == second_record["sha256"]
    assert first_record["size_bytes"] == second_record["size_bytes"]
    with ZipFile(first) as archive:
        assert tuple(archive.namelist()) == release_gate._EVIDENCE_FILES
        assert all(info.date_time == (1980, 1, 1, 0, 0, 0) for info in archive.infolist())


def test_release_evidence_zip_rejects_missing_input_and_overwrite(
    tmp_path: Path,
) -> None:
    release_gate = _load_script("r7_release_gate_errors_test", RELEASE_GATE_PATH)
    incomplete = tmp_path / "incomplete"
    incomplete.mkdir()
    for name in release_gate._EVIDENCE_FILES[:-1]:
        (incomplete / name).write_bytes(b"evidence")

    with pytest.raises(RuntimeError, match="incomplete"):
        release_gate._write_evidence_zip(incomplete, tmp_path / "missing.zip")

    complete = tmp_path / "complete"
    complete.mkdir()
    for name in release_gate._EVIDENCE_FILES:
        (complete / name).write_bytes(b"evidence")
    destination = tmp_path / "existing.zip"
    destination.write_bytes(b"do-not-overwrite")

    with pytest.raises(RuntimeError, match="must not already exist"):
        release_gate._write_evidence_zip(complete, destination)
    assert destination.read_bytes() == b"do-not-overwrite"


def test_hidden_v4_parser_has_no_manual_seed_and_requires_execution_approval(
    tmp_path: Path,
) -> None:
    runner = _load_script("r7_hidden_v4_parser_test", HIDDEN_V4_PATH)
    option_strings = {
        option
        for action in runner._build_parser()._actions
        for option in action.option_strings
    }

    assert not any("seed" in option for option in option_strings)
    assert "--execute-approved" in option_strings
    output = tmp_path / "must-not-exist"
    with pytest.raises(SystemExit) as exc_info:
        runner.main(
            [
                "--output",
                str(output),
                "--qualification-evidence",
                str(tmp_path / "evidence.zip"),
                "--qualification-sha256",
                "a" * 64,
            ]
        )
    assert exc_info.value.code == 2
    assert not output.exists()


def test_hidden_v4_preflight_only_never_generates_seed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_script("r7_hidden_v4_preflight_test", HIDDEN_V4_PATH)
    output = tmp_path / "preflight"
    monkeypatch.setattr(runner, "_repository_root", lambda: tmp_path)
    monkeypatch.setattr(runner, "_git", lambda *_args: "")
    monkeypatch.setattr(
        runner,
        "_preflight",
        lambda *_args: {
            "head": "a" * 40,
            "tree": "b" * 40,
            "working_tree_clean": True,
            "release_evidence": {"receipt_content_hash": "c" * 64},
            "packaging_source_freeze": {"content_hash": "d" * 64},
            "machine": {"logical_cpu_count": 8},
        },
    )
    monkeypatch.setattr(
        runner.secrets,
        "randbits",
        lambda _bits: (_ for _ in ()).throw(AssertionError("seed must not be created")),
    )

    assert (
        runner.main(
            [
                "--output",
                str(output),
                "--qualification-evidence",
                str(tmp_path / "evidence.zip"),
                "--qualification-sha256",
                "a" * 64,
                "--preflight-only",
            ]
        )
        == 0
    )
    assert tuple(path.name for path in output.iterdir()) == ("preflight-receipt.json",)
    manifest = json.loads((output / "preflight-receipt.json").read_text())
    assert manifest["schema"] == runner._PREFLIGHT_SCHEMA
    assert manifest["hidden_seed_generated"] is False
    assert manifest["hidden_executed"] is False


def test_hidden_v4_rejects_receipt_and_timing_tampering(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_script("r7_hidden_v4_evidence_tamper_test", HIDDEN_V4_PATH)
    repository_root, evidence, _ = _make_valid_release_evidence(tmp_path, runner)
    monkeypatch.setattr(
        runner,
        "_git",
        lambda _root, *args: "a" * 40
        if args == ("rev-parse", "HEAD")
        else "b" * 40,
    )
    expected_sha = runner._sha256(evidence)

    accepted = runner._verify_release_evidence(repository_root, evidence, expected_sha)
    assert accepted["sample_count"] == 500

    receipt_tampered = tmp_path / "receipt-tampered.zip"
    receipt_tampered.write_bytes(evidence.read_bytes())
    _mutate_json_member(
        receipt_tampered,
        "qualification-receipt.json",
        lambda value: value.__setitem__("sample_count", 499),
    )
    with pytest.raises(RuntimeError, match="receipt content hash mismatch"):
        runner._verify_release_evidence(
            repository_root,
            receipt_tampered,
            runner._sha256(receipt_tampered),
        )

    timing_tampered = tmp_path / "timing-tampered.zip"
    timing_tampered.write_bytes(evidence.read_bytes())
    _mutate_json_member(
        timing_tampered,
        "timing-qualification.json",
        lambda value: value.__setitem__("unbound_note", "tampered"),
    )
    with pytest.raises(RuntimeError, match="timing receipt binding mismatch"):
        runner._verify_release_evidence(
            repository_root,
            timing_tampered,
            runner._sha256(timing_tampered),
        )


def test_hidden_v4_rejects_source_freeze_and_native_hash_tampering(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_script("r7_hidden_v4_source_native_tamper_test", HIDDEN_V4_PATH)
    repository_root, evidence, files = _make_valid_release_evidence(tmp_path, runner)
    monkeypatch.setattr(
        runner,
        "_git",
        lambda _root, *args: "a" * 40
        if args == ("rev-parse", "HEAD")
        else "b" * 40,
    )
    expected_sha = runner._sha256(evidence)
    accepted = runner._verify_release_evidence(repository_root, evidence, expected_sha)
    runner._verify_native_libraries(repository_root, accepted)

    files["source"].write_bytes(b"print('changed')\n")
    with pytest.raises(RuntimeError, match="frozen executable source changed"):
        runner._verify_release_evidence(repository_root, evidence, expected_sha)
    files["source"].write_bytes(b"print('frozen')\n")

    files["full"].write_bytes(b"tampered-native-library")
    with pytest.raises(RuntimeError, match="dwb_full_core.dll"):
        runner._verify_native_libraries(repository_root, accepted)


def test_hidden_v4_schemas_are_separate_from_historical_runner() -> None:
    runner = _load_script("r7_hidden_v4_schema_test", HIDDEN_V4_PATH)
    historical = _load_script("r7_hidden_historical_schema_test", HISTORICAL_HIDDEN_PATH)
    v4_schemas = {
        runner._PREFLIGHT_SCHEMA,
        runner._SEED_COMMITMENT_SCHEMA,
        runner._CONSUMED_SEED_SCHEMA,
        runner._PARTIAL_SCHEMA,
        runner._SUMMARY_SCHEMA,
        runner._RECEIPT_SCHEMA,
        runner.R7_HIDDEN_V4_OBSERVATION_VERSION,
    }

    assert len(v4_schemas) == 7
    assert all("v4" in schema or schema == "r7-hidden-observation-v3" for schema in v4_schemas)
    assert "r7-hidden-preflight-v2" not in v4_schemas
    assert runner.R7_HIDDEN_V4_OBSERVATION_VERSION != historical.R7_HIDDEN_OBSERVATION_VERSION


def test_hidden_v4_infrastructure_failure_writes_terminal_receipt_and_trace_manifest(
    tmp_path: Path,
) -> None:
    runner = _load_script("r7_hidden_v4_infrastructure_test", HIDDEN_V4_PATH)
    output = tmp_path / "blocked"
    trace = output / "failure-traces/case-00/infrastructure-tick-trace.jsonl"
    trace.parent.mkdir(parents=True)
    trace.write_text(
        json.dumps({"record_content_hash": "a" * 64}, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    receipt_hash = runner._record_infrastructure_failure(
        output,
        "b" * 64,
        0,
        RuntimeError("worker failed"),
    )

    failure = json.loads((output / "infrastructure-failure.json").read_text())
    summary = json.loads((output / "summary.json").read_text())
    receipt = json.loads(
        (output / "hidden-v4-consumption-receipt.json").read_text()
    )
    manifest = json.loads((output / "partial-trace-manifest.json").read_text())
    assert failure["final_status"] == "BLOCKED_INFRASTRUCTURE"
    assert summary["final_status"] == "BLOCKED_INFRASTRUCTURE"
    assert receipt["final_status"] == "BLOCKED_INFRASTRUCTURE"
    assert receipt["completed"] is False
    assert receipt["algorithm_verdict"] is None
    assert receipt["receipt_content_hash"] == receipt_hash
    assert manifest["trace_file_count"] == 1
    assert manifest["records"][0]["sha256"] == runner._sha256(trace)
    assert manifest["records"][0]["record_count"] == 1
    assert manifest["records"][0]["last_record_hash"] == "a" * 64


def test_hidden_v4_case_trace_manifest_rejects_stale_result_binding(
    tmp_path: Path,
) -> None:
    runner = _load_script("r7_hidden_v4_trace_binding_test", HIDDEN_V4_PATH)
    root = tmp_path / "failure-traces"
    trace = root / "case-00/tick-trace.jsonl"
    trace.parent.mkdir(parents=True)
    trace.write_text(
        json.dumps({"record_content_hash": "c" * 64}, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    result = SimpleNamespace(
        case_id="case-00",
        trace_content_hash="d" * 64,
        trace_file_sha256=runner._sha256(trace),
        trace_record_count=1,
        trace_last_record_hash="c" * 64,
    )

    manifest = runner._failure_trace_manifest(root, results=(result,))
    assert manifest["case_count"] == 1
    assert manifest["records"][0]["case_id"] == "case-00"
    assert len(runner._case_trace_set_hash((result,))) == 64

    trace.write_text(
        json.dumps({"record_content_hash": "e" * 64}, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="case trace binding mismatch"):
        runner._failure_trace_manifest(root, results=(result,))


@pytest.mark.parametrize("failure_stage", ("prepare", "evaluate", "finalize"))
def test_hidden_v4_post_commitment_errors_are_sealed_as_infrastructure_blocks(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure_stage: str,
) -> None:
    runner = _load_script(f"r7_hidden_v4_{failure_stage}_failure_test", HIDDEN_V4_PATH)
    repository_root = tmp_path / "repository"
    repository_root.mkdir()
    output = tmp_path / f"blocked-{failure_stage}"

    def fake_git(_root: Path, *args: str) -> str:
        if args == ("status", "--porcelain=v1"):
            return ""
        if args == ("rev-parse", "HEAD"):
            return "a" * 40
        return "b" * 40

    monkeypatch.setattr(runner, "_repository_root", lambda: repository_root)
    monkeypatch.setattr(runner, "_git", fake_git)
    monkeypatch.setattr(
        runner,
        "_preflight",
        lambda *_args: {
            "head": "a" * 40,
            "tree": "b" * 40,
            "working_tree_clean": True,
            "release_evidence": {"receipt_content_hash": "c" * 64},
            "packaging_source_freeze": {"content_hash": "d" * 64},
            "machine": {"logical_cpu_count": 8},
        },
    )
    monkeypatch.setattr(runner.secrets, "randbits", lambda _bits: 123_456)
    if failure_stage == "prepare":
        monkeypatch.setattr(
            runner,
            "build_hidden_v4_case_specs",
            lambda _seed: (_ for _ in ()).throw(RuntimeError("prepare failed")),
        )
    elif failure_stage == "evaluate":
        monkeypatch.setattr(
            runner,
            "evaluate_hidden_v4_cases",
            lambda *_args, **_kwargs: (_ for _ in ()).throw(
                RuntimeError("evaluate failed")
            ),
        )
    else:
        monkeypatch.setattr(runner, "evaluate_hidden_v4_cases", lambda *_a, **_k: ())
        monkeypatch.setattr(
            runner,
            "_verify_execution_freeze",
            lambda *_args: {"content_hash": "e" * 64},
        )
        monkeypatch.setattr(
            runner,
            "_finalize_hidden_v4",
            lambda **_kwargs: (_ for _ in ()).throw(RuntimeError("finalize failed")),
        )

    with pytest.raises(RuntimeError, match=f"{failure_stage} failed"):
        runner.main(
            [
                "--output",
                str(output),
                "--qualification-evidence",
                str(tmp_path / "evidence.zip"),
                "--qualification-sha256",
                "f" * 64,
                "--execute-approved",
            ]
        )

    assert (output / "seed-commitment.json").is_file()
    receipt = json.loads(
        (output / "hidden-v4-consumption-receipt.json").read_text()
    )
    assert receipt["final_status"] == "BLOCKED_INFRASTRUCTURE"
    ledger_path = next(
        (repository_root / "simulation/path_planning_lab/outputs").glob("*ledger.json")
    )
    ledger = json.loads(ledger_path.read_text(encoding="utf-8"))
    assert ledger["status"] == "infrastructure_failure"
    assert ledger["receipt_content_hash"] == receipt["receipt_content_hash"]
