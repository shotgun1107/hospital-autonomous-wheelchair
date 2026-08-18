from __future__ import annotations

import importlib.util
import shutil
from pathlib import Path

import pytest

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
RUNNER_PATH = REPOSITORY_ROOT / "simulation/path_planning_lab/scripts/run_r7_hidden_observation.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("r7_hidden_runner_under_test", RUNNER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_runner_is_bound_to_v4_evidence_and_has_no_manual_seed_option() -> None:
    runner = _load_runner()

    assert runner.R7_EVIDENCE_SIZE == 7_773
    assert runner.R7_EVIDENCE_SHA256 == (
        "3829e14dcf5e548210cdc181bde5dc913743f4f211ca25c3f28c15e2a7016183"
    )
    assert runner.R7_IMPLEMENTATION_COMMIT == (
        "8a6275c874ec060c0b268d4f56ee7205ab9f7266"
    )
    option_strings = {
        option
        for action in runner._build_parser()._actions
        for option in action.option_strings
    }
    assert "--preflight-only" in option_strings
    assert not any("seed" in option for option in option_strings)


def test_v4_evidence_zip_and_receipt_are_accepted() -> None:
    runner = _load_runner()

    gate = runner._verify_r7_evidence(REPOSITORY_ROOT)

    assert gate["release_gate_qualified"] is True
    assert gate["deadline_miss_count"] == 0
    assert gate["sample_count"] == 500
    assert gate["semantic_parity_case_count"] == 5
    assert gate["receipt_content_hash"] == runner.R7_RECEIPT_CONTENT_HASH


def test_tampered_evidence_zip_is_rejected_before_reading_cases(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    source = REPOSITORY_ROOT / runner.R7_EVIDENCE_RELATIVE_PATH
    tampered = tmp_path / "tampered.zip"
    shutil.copyfile(source, tampered)
    payload = bytearray(tampered.read_bytes())
    payload[len(payload) // 2] ^= 0x01
    tampered.write_bytes(payload)
    monkeypatch.setattr(runner, "R7_EVIDENCE_RELATIVE_PATH", Path("tampered.zip"))

    with pytest.raises(RuntimeError, match="ZIP hash mismatch"):
        runner._verify_r7_evidence(tmp_path)


def test_preflight_only_writes_no_seed_or_case_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    output = tmp_path / "preflight"
    monkeypatch.setattr(runner, "_repository_root", lambda: tmp_path)
    monkeypatch.setattr(runner, "_git", lambda *_args: "")
    monkeypatch.setattr(
        runner,
        "_preflight",
        lambda _root: {
            "head": "a" * 40,
            "tree": "b" * 40,
            "working_tree_clean": True,
            "r7_implementation_commit": runner.R7_IMPLEMENTATION_COMMIT,
            "r7_gate": {"receipt_content_hash": runner.R7_RECEIPT_CONTENT_HASH},
            "machine": {"logical_cpu_count": 28},
        },
    )
    monkeypatch.setattr(
        runner.secrets,
        "randbits",
        lambda _bits: (_ for _ in ()).throw(AssertionError("seed must not be created")),
    )

    assert runner.main(["--output", str(output), "--preflight-only"]) == 0
    assert tuple(path.name for path in output.iterdir()) == ("preflight-manifest.json",)
    manifest = runner.json.loads((output / "preflight-manifest.json").read_text())
    assert manifest["hidden_seed_generated"] is False
    assert manifest["hidden_executed"] is False


def test_dirty_tree_stops_before_output_or_seed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runner = _load_runner()
    output = tmp_path / "must-not-exist"
    monkeypatch.setattr(runner, "_repository_root", lambda: tmp_path)
    monkeypatch.setattr(runner, "_git", lambda *_args: "dirty-file")
    monkeypatch.setattr(
        runner.secrets,
        "randbits",
        lambda _bits: (_ for _ in ()).throw(AssertionError("seed must not be created")),
    )

    with pytest.raises(SystemExit):
        runner.main(["--output", str(output), "--preflight-only"])
    assert not output.exists()


def test_native_library_hash_mismatch_is_rejected(
    tmp_path: Path,
) -> None:
    runner = _load_runner()
    native = tmp_path / "simulation/path_planning_lab/src/hospital_path_lab/_native"
    native.mkdir(parents=True)
    full = native / "dwb_full_core.dll"
    safety = native / "dwb_safety_core.dll"
    full.write_bytes(b"full")
    safety.write_bytes(b"safety")
    gate = {
        "native_full_library_sha256": runner._sha256(full),
        "native_safety_library_sha256": "0" * 64,
    }

    with pytest.raises(RuntimeError, match="dwb_safety_core.dll"):
        runner._verify_native_libraries(tmp_path, gate)
