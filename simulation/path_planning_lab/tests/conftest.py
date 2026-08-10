from pathlib import Path

import pytest

from hospital_path_lab.scenario import ScenarioSuite, load_scenario_suite

LAB_ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="session")
def suite() -> ScenarioSuite:
    return load_scenario_suite(LAB_ROOT / "scenarios" / "hospital_corridors.yaml")
