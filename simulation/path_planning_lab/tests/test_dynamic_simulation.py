from __future__ import annotations

import json
from dataclasses import fields
from pathlib import Path

import matplotlib.pyplot as plt

from hospital_path_lab.dynamic_actor import generate_corridor_crossing_scenario
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    ActorState,
    DynamicControllerInputFrame,
)
from hospital_path_lab.experiment_visualization import save_dynamic_actor_artifacts
from hospital_path_lab.simulation import (
    dynamic_artifact_stem,
    dynamic_trace_content_hash,
    simulate_dynamic_actor_scenario,
)


def test_dynamic_simulation_uses_exact_20hz_tick_time_and_stationary_robot() -> None:
    scenario = generate_corridor_crossing_scenario(20260810)
    trace = simulate_dynamic_actor_scenario(scenario)

    assert trace.metadata.control_frequency_hz == 20.0
    assert trace.metadata.tick_count == 130
    assert len(trace.ground_truth_frames) == 131
    assert len(trace.controller_input_frames) == 131
    assert len(trace.accepted_commands) == 130
    assert all(
        frame.tick_id == tick_id and frame.simulation_time_s == tick_id * DYNAMIC_CONTROL_PERIOD_S
        for tick_id, frame in enumerate(trace.ground_truth_frames)
    )
    assert all(
        frame.robot_state == scenario.robot_initial_state for frame in trace.ground_truth_frames
    )
    assert all(
        command.command.linear == command.command.angular == 0.0
        for command in trace.accepted_commands
    )
    assert tuple(event.kind for event in trace.state_events) == (
        "episode_started",
        "actor_crossed_reference",
        "episode_finished",
    )


def test_dynamic_trace_is_fully_reproducible_for_same_seed() -> None:
    first = simulate_dynamic_actor_scenario(generate_corridor_crossing_scenario(71))
    second = simulate_dynamic_actor_scenario(generate_corridor_crossing_scenario(71))

    assert first == second
    assert dynamic_trace_content_hash(first) == dynamic_trace_content_hash(second)
    assert first.metadata.world_content_hash == second.metadata.world_content_hash


def test_controller_frames_do_not_expose_actor_ground_truth_or_seed() -> None:
    trace = simulate_dynamic_actor_scenario(generate_corridor_crossing_scenario(81))
    field_names = {field.name for field in fields(DynamicControllerInputFrame)}

    assert "actors" not in field_names
    assert "ground_truth" not in field_names
    assert "seed" not in field_names
    assert all(
        not any(isinstance(getattr(frame, field.name), ActorState) for field in fields(frame))
        for frame in trace.controller_input_frames
    )
    assert trace.controller_input_frames[0].robot_state == trace.ground_truth_frames[0].robot_state


def test_json_and_png_artifacts_are_deterministic_and_close_figures(tmp_path: Path) -> None:
    trace = simulate_dynamic_actor_scenario(generate_corridor_crossing_scenario(91))
    before = set(plt.get_fignums())

    first_json, first_png = save_dynamic_actor_artifacts(trace, tmp_path / "first")
    second_json, second_png = save_dynamic_actor_artifacts(trace, tmp_path / "second")

    stem = dynamic_artifact_stem(trace)
    assert first_json.name == f"{stem}.json"
    assert first_png.name == f"{stem}.png"
    assert first_json.read_bytes() == second_json.read_bytes()
    assert first_png.read_bytes() == second_png.read_bytes()
    assert first_png.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert first_png.stat().st_size > 0
    payload = json.loads(first_json.read_text(encoding="utf-8"))
    assert payload["metadata"]["seed"] == 91
    assert payload["metadata"]["generator_version"] == "dynamic_actor_v1"
    assert payload["content_hash"] == dynamic_trace_content_hash(trace)
    assert len(payload["ground_truth_frames"]) == 131
    assert set(plt.get_fignums()) == before
