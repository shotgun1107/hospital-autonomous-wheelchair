import json
from pathlib import Path

import pytest

from hospital_path_lab.cli import LAB_ROOT, main


def test_safety_demo_command(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["safety-demo"]) == 0
    output = capsys.readouterr().out
    assert "denied_before_checks=True" in output
    assert "resumed_after_checks=True" in output


def test_list_algorithms_reports_implemented_and_deferred(
    capsys: pytest.CaptureFixture[str],
) -> None:
    assert main(["list-algorithms"]) == 0
    output = capsys.readouterr().out
    assert "dstar_lite\timplemented" in output
    assert "grid_astar\timplemented" in output
    assert "dwa\timplemented" in output
    assert "rpp\timplemented" in output
    assert "teb\tdeferred" in output


def test_benchmark_command_writes_json_and_plots(tmp_path: Path) -> None:
    scenario = LAB_ROOT / "scenarios" / "hospital_corridors.yaml"
    assert (
        main(
            [
                "benchmark",
                "--scenario",
                str(scenario),
                "--repeats",
                "3",
                "--output-dir",
                str(tmp_path),
            ]
        )
        == 0
    )
    records = json.loads((tmp_path / "global_benchmark.json").read_text(encoding="utf-8"))
    assert len(records) == 10
    assert all(record["expected_result_matched"] for record in records)
    assert len(list(tmp_path.glob("*.png"))) == 5
