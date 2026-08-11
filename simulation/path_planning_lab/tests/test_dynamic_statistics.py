from __future__ import annotations

import pytest

from hospital_path_lab.dynamic_runner import (
    compute_paired_statistics,
    metric_worsening,
    stratified_paired_bootstrap_ci,
)


def _record(
    episode_id: str,
    controller_name: str,
    *,
    completion_time_s: float,
    hold_s: float,
    comfort: float,
    category: str,
) -> dict[str, object]:
    return {
        "episode_id": episode_id,
        "controller_name": controller_name,
        "observation_profile": "normal",
        "progressable": True,
        "expectation_category": category,
        "metrics": {
            "completion_time_s": completion_time_s,
            "safety_hold_duration_s": hold_s,
            "longitudinal_jerk_rms_mps3": comfort,
            "angular_acceleration_rms_radps2": comfort,
            "angular_jerk_rms_radps3": comfort,
        },
    }


def test_median_improvement_and_stratified_bootstrap_match_oracle() -> None:
    records: list[dict[str, object]] = []
    for index, category in enumerate(("wait_and_resume", "local_detour_feasible")):
        episode_id = f"hidden_{index}"
        records.extend(
            (
                _record(
                    episode_id,
                    "dynamic_pure_pursuit",
                    completion_time_s=10.0,
                    hold_s=5.0,
                    comfort=1.0,
                    category=category,
                ),
                _record(
                    episode_id,
                    "dynamic_dwa",
                    completion_time_s=8.0,
                    hold_s=4.0,
                    comfort=1.2,
                    category=category,
                ),
            )
        )

    result = compute_paired_statistics(
        records,
        bootstrap_iterations=100,
        bootstrap_seed=77,
    )

    assert result["time_improvement"] == pytest.approx(0.20)
    assert result["hold_improvement"] == pytest.approx(0.20)
    assert result["selected_improvement_metric"] == "completion_time_s"
    assert result["paired_delta_bootstrap_95ci"] == {"lower": -2.0, "upper": -2.0}
    assert all(
        value == pytest.approx(0.20)
        for value in result["comfort_worsening"].values()
    )


def test_bootstrap_is_reproducible_and_preserves_category_sample_counts() -> None:
    paired = [("a", -1.0), ("a", -3.0), ("b", -2.0), ("b", -4.0)]

    first = stratified_paired_bootstrap_ci(paired, iterations=500, seed=19)
    second = stratified_paired_bootstrap_ci(paired, iterations=500, seed=19)

    assert first == second
    assert first["upper"] < 0.0


def test_metric_worsening_uses_denominator_floor() -> None:
    assert metric_worsening([0.0, 0.0], [0.02, 0.02], denominator_floor=0.10) == pytest.approx(
        0.20
    )
    assert metric_worsening([], [], denominator_floor=0.10) is None
    assert metric_worsening([1.0], [1.0, 2.0], denominator_floor=0.10) is None
