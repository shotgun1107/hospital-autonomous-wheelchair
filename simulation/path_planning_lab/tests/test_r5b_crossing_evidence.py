from __future__ import annotations

import pytest

from hospital_path_lab.dynamic_witness_contracts import WitnessKind
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    PersistentSourceDerivedDwbController,
)
from hospital_path_lab.local_reference_contracts import LocalManeuverKind
from hospital_path_lab.r5b_crossing_evidence import (
    R5B_CROSSING_RELEASE_TICK,
    build_causal_r5b_crossing_evidence,
)
from hospital_path_lab.r5b_temporal_execution import run_r5b_temporal_case
from hospital_path_lab.r5b_temporal_reference import (
    build_r5b_crossing_reference_bundles,
)


def test_causal_crossing_evidence_preserves_both_validated_sides() -> None:
    evidence = build_causal_r5b_crossing_evidence()

    assert len(evidence) == 2
    assert tuple(item.witness.kind for item in evidence) == (
        WitnessKind.CROSSING_BYPASS_LEFT,
        WitnessKind.CROSSING_BYPASS_RIGHT,
    )
    for item in evidence:
        assert item.release_tick == R5B_CROSSING_RELEASE_TICK
        assert item.world.actors[0].active_from_s == 0.0
        assert item.world.duration_s == 39.0
        assert item.world.actors[0].active_until_s == item.world.duration_s
        assert item.world.actors[0].start_position.y < item.world.grid.origin_y_m
        assert item.validation.passed
        assert item.witness.points[R5B_CROSSING_RELEASE_TICK].time_s == 4.0
        assert item.witness.points[R5B_CROSSING_RELEASE_TICK].pose == (
            item.world.initial_state.pose
        )


def test_causal_crossing_reference_preserves_distinct_crossing_kind() -> None:
    bundles = build_r5b_crossing_reference_bundles()

    assert tuple(item.reference.maneuver_kind for item in bundles) == (
        LocalManeuverKind.CROSSING_BYPASS_LEFT,
        LocalManeuverKind.CROSSING_BYPASS_RIGHT,
    )
    for item in bundles:
        assert item.validation.passed
        assert item.reference.validity.valid_from_control_tick == R5B_CROSSING_RELEASE_TICK
        assert item.reference.departure_knot_index is not None
        assert item.reference.pass_section_index == 1


def test_cpp_dwb_crossing_holds_until_frozen_release() -> None:
    bundle = build_r5b_crossing_reference_bundles()[0]
    controller = PersistentSourceDerivedDwbController(
        use_cpp_safety_core=True,
        use_cpp_full_core=True,
    )

    result = run_r5b_temporal_case(bundle, controller=controller, tick_limit=85)

    assert controller.native_full_core_used
    assert result.release_tick == R5B_CROSSING_RELEASE_TICK
    assert result.first_controller_tick == R5B_CROSSING_RELEASE_TICK
    assert result.first_motion_tick == 81
    assert result.gate_override_count == 0


@pytest.mark.parametrize(
    ("bundle_index", "expected"),
    (
        (0, (163, 370)),
        (1, (161, 295)),
    ),
    ids=("left", "right"),
)
def test_cpp_dwb_crossing_bypasses_rejoins_and_completes(
    bundle_index: int,
    expected: tuple[int, int],
) -> None:
    bundle = build_r5b_crossing_reference_bundles()[bundle_index]
    controller = PersistentSourceDerivedDwbController(
        use_cpp_safety_core=True,
        use_cpp_full_core=True,
    )

    result = run_r5b_temporal_case(bundle, controller=controller, tick_limit=780)

    departure, pass_event = expected
    assert result.completed
    assert result.release_tick == R5B_CROSSING_RELEASE_TICK
    assert result.first_motion_tick == 81
    assert result.departure_tick == departure
    assert result.pass_event_tick == pass_event
    assert result.rejoin_tick is not None
    assert result.completion_tick is not None
    assert result.rejoin_tick < result.completion_tick
    assert result.maximum_lateral_deviation_m > 0.10
    assert result.minimum_actor_clearance_m is not None
    assert result.minimum_actor_clearance_m >= 0.08
    assert result.gate_override_count == 0
    assert result.hard_failures == ()
    assert controller.native_full_core_used
