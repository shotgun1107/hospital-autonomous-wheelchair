from __future__ import annotations

import importlib.util
import json
import subprocess
from pathlib import Path
from types import ModuleType

import pytest

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
HIDDEN_V5_PATH = REPOSITORY_ROOT / "simulation/path_planning_lab/scripts/run_r7_hidden_v5.py"


def _load_runner(module_name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(module_name, HIDDEN_V5_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _git(root: Path, *args: str) -> str:
    return subprocess.run(
        ("git", *args),
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    ).stdout.strip()


def _make_remote_clones(tmp_path: Path) -> tuple[Path, Path, Path]:
    origin = tmp_path / "origin.git"
    subprocess.run(("git", "init", "--bare", str(origin)), check=True, capture_output=True)

    source = tmp_path / "source"
    source.mkdir()
    _git(source, "init")
    _git(source, "config", "user.name", "R7 lifecycle test")
    _git(source, "config", "user.email", "r7-lifecycle@example.invalid")
    (source / "README.md").write_text("reservation fixture\n", encoding="utf-8")
    _git(source, "add", "README.md")
    _git(source, "commit", "-m", "fixture")
    _git(source, "branch", "-M", "main")
    _git(source, "remote", "add", "origin", str(origin))
    _git(source, "push", "-u", "origin", "main")
    subprocess.run(
        ("git", "--git-dir", str(origin), "symbolic-ref", "HEAD", "refs/heads/main"),
        check=True,
        capture_output=True,
    )

    first = tmp_path / "first"
    second = tmp_path / "second"
    subprocess.run(("git", "clone", str(origin), str(first)), check=True, capture_output=True)
    subprocess.run(("git", "clone", str(origin), str(second)), check=True, capture_output=True)
    for clone in (first, second):
        _git(clone, "config", "user.name", "R7 lifecycle test")
        _git(clone, "config", "user.email", "r7-lifecycle@example.invalid")
    return origin, first, second


def _preflight_for(runner: ModuleType, repository_root: Path) -> dict[str, object]:
    return {
        "head": _git(repository_root, "rev-parse", "HEAD"),
        "tree": _git(repository_root, "rev-parse", "HEAD^{tree}"),
        "working_tree_clean": True,
        "release_evidence": {
            "sha256": "1" * 64,
            "receipt_content_hash": "2" * 64,
        },
        "packaging_source_freeze": {"content_hash": "3" * 64},
        "machine": {"logical_cpu_count": 1},
    }


def test_hidden_v5_uses_a_new_catalog_and_permanently_rejects_historic_v4_seed() -> None:
    runner = _load_runner("r7_hidden_v5_catalog_test")

    root_seed = 123_456_789
    specs = runner.build_hidden_v5_case_specs(root_seed)

    assert runner._EXECUTION_NAMESPACE == "r7-hidden-v5-execution-v1"
    assert runner._OBSERVATION_NAMESPACE == "r7-hidden-observation-v5"
    assert (
        runner._execution_identity(
            {
                "head": "a" * 40,
                "tree": "b" * 40,
                "release_evidence": {
                    "sha256": "c" * 64,
                    "receipt_content_hash": "d" * 64,
                },
                "packaging_source_freeze": {"content_hash": "e" * 64},
            }
        )["reservation_ref"]
        == "refs/heads/codex/r7-hidden-v5-reservation"
    )
    assert len(specs) == 20
    assert all(item.case_id.startswith("hidden-v5-") for item in specs)
    v4_commitment = runner._V4_SUPPORT.hidden_v4_seed_commitment(root_seed)
    assert runner.hidden_v5_seed_commitment(root_seed) != v4_commitment
    assert 6_564_067_906_066_881_700 in runner._V4_SUPPORT._KNOWN_CONSUMED_ROOT_SEEDS
    with pytest.raises(ValueError, match="permanently rejected"):
        runner.hidden_v5_seed_commitment(6_564_067_906_066_881_700)


def test_remote_preflight_reservation_never_generates_a_seed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _origin, first, _second = _make_remote_clones(tmp_path)
    runner = _load_runner("r7_hidden_v5_preflight_reservation_test")
    preflight = _preflight_for(runner, first)
    output = tmp_path / "preflight-output"
    monkeypatch.setattr(runner, "_repository_root", lambda: first)
    monkeypatch.setattr(runner, "_preflight", lambda *_args: preflight)
    monkeypatch.setattr(
        runner.secrets,
        "randbits",
        lambda _bits: (_ for _ in ()).throw(AssertionError("preflight generated a seed")),
    )

    assert (
        runner.main(
            [
                "--output",
                str(output),
                "--qualification-evidence",
                str(tmp_path / "release.zip"),
                "--qualification-sha256",
                "a" * 64,
                "--preflight-only",
                "--reserve-remote",
                "--designated-executor",
                "company-pc-r7",
            ]
        )
        == 0
    )

    receipt = json.loads((output / "preflight-receipt.json").read_text(encoding="utf-8"))
    assert receipt["schema"] == runner._PREFLIGHT_SCHEMA
    assert receipt["hidden_seed_generated"] is False
    assert receipt["hidden_executed"] is False
    assert receipt["remote_reservation"]["record"]["state"] == "reserved_before_seed"
    assert not (output / "consumed-seed.json").exists()
    assert _git(first, "status", "--porcelain=v1") == ""


def test_remote_reservation_and_claim_allow_only_one_clone(tmp_path: Path) -> None:
    _origin, first, second = _make_remote_clones(tmp_path)
    runner = _load_runner("r7_hidden_v5_remote_claim_test")
    preflight = _preflight_for(runner, first)
    identity = runner._execution_identity(preflight)

    reservation = runner._reserve_remote(first, preflight, identity, "company-pc-r7")
    assert reservation["record"]["state"] == "reserved_before_seed"
    with pytest.raises(RuntimeError, match="already reserved or consumed"):
        runner._reserve_remote(second, preflight, identity, "company-pc-r7")

    claim = runner._claim_remote_reservation(first, preflight, identity, "company-pc-r7")
    assert claim["record"]["state"] == "execution_started_before_seed"
    with pytest.raises(RuntimeError, match="does not match"):
        runner._claim_remote_reservation(second, preflight, identity, "company-pc-r7")
    assert _git(first, "status", "--porcelain=v1") == ""
    assert _git(second, "status", "--porcelain=v1") == ""
