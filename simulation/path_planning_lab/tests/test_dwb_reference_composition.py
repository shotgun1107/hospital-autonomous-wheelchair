from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from hospital_path_lab.contracts import (
    GridSnapshot,
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import (
    ActorTrack,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    Point2D,
    Vector2D,
    build_controller_snapshot,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import build_actor_prediction_set
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_algorithms.dwb_reference.composition import (
    SourceDerivedDwbConfig,
    SourceDerivedDynamicDwbController,
    _map_grid_scale,
)
from hospital_path_lab.local_algorithms.dwb_reference.contracts import DwbGeneratorConfig
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _fast_config() -> SourceDerivedDwbConfig:
    # Composition intentionally has no reduced-candidate test configuration:
    # integration exercises the same frozen v7 generator contract as research.
    return SourceDerivedDwbConfig()


def test_map_grid_critic_scale_matches_frozen_nav2_resolution_convention() -> None:
    assert _map_grid_scale(32.0, 0.02) == pytest.approx(0.32)
    assert _map_grid_scale(24.0, 0.02) == pytest.approx(0.24)


def _snapshot(
    *,
    tick: int = 0,
    state: RobotState | None = None,
    goal: Pose2D | None = None,
    actor: ActorTrack | None = None,
    occupancy: np.ndarray | None = None,
    mission_revision: int = 1,
):
    simulation_time_s = tick * 0.05
    tracks = () if actor is None else (actor,)
    frame = DynamicObservationFrame(
        stream_id="stream-source-derived",
        episode_id="episode-source-derived",
        episode_seed=17,
        map_id="map-source-derived",
        map_revision=1,
        observation_revision=tick,
        sequence=tick,
        observed_at_s=simulation_time_s,
        delivered_at_s=simulation_time_s,
        frame_kind=(
            DynamicObservationFrameKind.EMPTY
            if actor is None
            else DynamicObservationFrameKind.TRACKS
        ),
        tracks=tracks,
        content_hash=f"observation-{tick}",
    )
    observation = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.FRESH,
        frame=frame,
        age_s=0.0,
        failures=(),
        last_event_was_no_frame=False,
    )
    if occupancy is None:
        occupancy = np.zeros((100, 100), dtype=np.bool_)
    grid = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="map-source-derived",
            map_revision=1,
            mission_revision=mission_revision,
            observation_revision=tick,
            seed=17,
            content_hash=f"grid-{tick}",
        ),
        grid=GridMap(occupancy, resolution_m=0.05),
    )
    return build_controller_snapshot(
        tick_id=tick,
        simulation_time_s=simulation_time_s,
        mission_id="mission-source-derived",
        robot_state=state or RobotState(Pose2D(0.60, 2.50), Twist2D()),
        goal_pose=goal or Pose2D(4.20, 2.50),
        reference_path=(Pose2D(0.60, 2.50), goal or Pose2D(4.20, 2.50)),
        static_grid_snapshot=grid,
        validated_observation=observation,
        actor_tubes=build_actor_prediction_set(observation),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )


def test_composed_controller_selects_a_safe_source_derived_candidate() -> None:
    controller = SourceDerivedDynamicDwbController(config=_fast_config())

    result = controller.step(_snapshot())

    assert result.status is PlanStatus.FOUND
    assert result.controller_name == "dynamic_dwb_reference"
    assert len(result.predicted_trajectory) == 41
    assert controller.selected_safety_evidence is not None
    assert controller.selected_safety_evidence.safe


def test_same_tick_is_idempotent_and_does_not_rebuild_the_stack() -> None:
    controller = SourceDerivedDynamicDwbController(config=_fast_config())
    snapshot = _snapshot()

    first = controller.step(snapshot)
    second = controller.step(snapshot)

    assert second is first
    assert controller.stack_build_count == 1


def test_observation_revision_does_not_rebuild_unchanged_static_geometry() -> None:
    controller = SourceDerivedDynamicDwbController(config=_fast_config())

    first = controller.step(_snapshot(tick=0))
    second = controller.step(_snapshot(tick=1))

    assert first.status is PlanStatus.FOUND
    assert second.status is PlanStatus.FOUND
    assert controller.stack_build_count == 1


def test_near_actor_rejects_all_candidates_without_weakening_shared_safety() -> None:
    actor = ActorTrack(
        track_id="track-1",
        actor_binding_id="actor-1",
        observed_position=Point2D(0.70, 2.50),
        observed_velocity=Vector2D(0.0, 0.0),
        position_sigma_m=0.0,
        velocity_sigma_mps=0.0,
    )
    controller = SourceDerivedDynamicDwbController(config=_fast_config())

    result = controller.step(_snapshot(actor=actor))

    assert result.status is PlanStatus.NO_PATH
    assert result.controller_requested_stop
    assert result.no_safe_candidate
    assert result.failure_reason == "no_legal_dwb_trajectory"


def test_goal_override_decelerates_without_claiming_a_protective_stop() -> None:
    state = RobotState(Pose2D(4.16, 2.50, 0.0), Twist2D(0.20, 0.0))
    controller = SourceDerivedDynamicDwbController(config=_fast_config())

    result = controller.step(_snapshot(state=state))

    assert result.status is PlanStatus.FOUND
    assert result.requested_twist.linear == pytest.approx(0.175)
    assert result.requested_twist.angular == 0.0
    assert not result.controller_requested_stop
    assert "goal_state=decelerate_to_stop" in result.decision_trace


def test_same_tick_changed_input_fails_closed() -> None:
    controller = SourceDerivedDynamicDwbController(config=_fast_config())
    first = _snapshot()
    controller.step(first)
    changed = replace(first, input_content_hash="different-same-tick-input")

    result = controller.step(changed)

    assert result.status is PlanStatus.NO_PATH
    assert result.failure_reason == "same_tick_input_changed"
    assert result.controller_requested_stop


def test_same_tick_cache_covers_all_semantic_motion_inputs() -> None:
    actor = ActorTrack(
        track_id="track-cache",
        actor_binding_id="actor-cache",
        observed_position=Point2D(4.60, 4.60),
        observed_velocity=Vector2D(0.0, 0.0),
        position_sigma_m=0.0,
        velocity_sigma_mps=0.0,
    )
    baseline = _snapshot(actor=actor)
    assert baseline.actor_tubes is not None
    tube = baseline.actor_tubes.tubes[0]
    changed_tubes = replace(
        baseline.actor_tubes,
        tubes=(replace(tube, observed_position=Point2D(4.40, 4.60)),),
    )
    changed_occupancy = baseline.static_grid_snapshot.grid.occupancy.copy()
    changed_occupancy[90, 90] = True
    changed_grid = replace(
        baseline.static_grid_snapshot,
        grid=GridMap(changed_occupancy, resolution_m=0.05),
    )
    mutations = (
        replace(
            baseline,
            robot_state=RobotState(Pose2D(0.65, 2.50), Twist2D()),
        ),
        replace(baseline, goal_pose=Pose2D(4.10, 2.50)),
        replace(
            baseline,
            reference_path=(Pose2D(0.60, 2.50), Pose2D(3.00, 2.70), baseline.goal_pose),
        ),
        replace(baseline, actor_tubes=changed_tubes),
        replace(baseline, static_grid_snapshot=changed_grid),
    )

    for changed in mutations:
        controller = SourceDerivedDynamicDwbController(config=_fast_config())
        cached = controller.step(baseline)

        result = controller.step(changed)

        assert result is not cached
        assert result.status is PlanStatus.NO_PATH
        assert result.requested_twist == Twist2D()
        assert result.failure_reason == "same_tick_input_changed"
        assert result.controller_requested_stop
        assert result.input_content_hash == baseline.input_content_hash


def test_zero_safety_scale_is_rejected() -> None:
    with pytest.raises(ValueError, match="project safety scale"):
        SourceDerivedDwbConfig(safety_scale=0.0)


def test_regressed_goal_tick_fails_closed_with_current_provenance() -> None:
    controller = SourceDerivedDynamicDwbController(config=_fast_config())
    controller.step(_snapshot(tick=2))
    regressed = _snapshot(tick=1)

    result = controller.step(regressed)

    assert result.status is PlanStatus.NO_PATH
    assert result.requested_twist == Twist2D()
    assert result.failure_reason == "goal_controller_contract_violation"
    assert result.controller_requested_stop
    assert result.no_safe_candidate
    assert result.source_tick_id == regressed.tick_id
    assert result.mission_id == regressed.mission_id
    assert result.map_revision == regressed.map_revision
    assert result.observation_revision == regressed.observation_revision
    assert result.input_content_hash == regressed.input_content_hash


def test_unexpected_goal_controller_value_error_is_not_hidden(monkeypatch) -> None:
    controller = SourceDerivedDynamicDwbController(config=_fast_config())

    def _unexpected_error(_goal_controller, _request):
        raise ValueError("programming defect sentinel")

    monkeypatch.setattr(type(controller._goal_controller), "update", _unexpected_error)

    with pytest.raises(ValueError, match="programming defect sentinel"):
        controller.step(_snapshot())


@pytest.mark.parametrize(
    ("geometry_field", "invalid_value"),
    [
        ("resolution_m", float("nan")),
        ("resolution_m", float("inf")),
        ("origin_x_m", float("nan")),
        ("origin_y_m", float("inf")),
    ],
)
def test_non_finite_grid_geometry_fails_closed_before_stack_build(
    geometry_field: str,
    invalid_value: float,
) -> None:
    baseline = _snapshot()
    old_grid = baseline.static_grid_snapshot.grid
    grid_arguments = {
        "occupancy": old_grid.occupancy,
        "resolution_m": old_grid.resolution_m,
        "origin_x_m": old_grid.origin_x_m,
        "origin_y_m": old_grid.origin_y_m,
    }
    grid_arguments[geometry_field] = invalid_value
    malformed = replace(
        baseline,
        static_grid_snapshot=replace(
            baseline.static_grid_snapshot,
            grid=GridMap(**grid_arguments),
        ),
    )
    controller = SourceDerivedDynamicDwbController()

    result = controller.step(malformed)

    assert result.status is PlanStatus.NO_PATH
    assert result.requested_twist == Twist2D()
    assert result.failure_reason == "grid_geometry_invalid"
    assert result.controller_requested_stop
    assert result.no_safe_candidate
    assert controller.stack_build_count == 0


def test_invalid_grid_snapshot_fails_closed_before_stack_build() -> None:
    baseline = _snapshot()
    malformed = replace(
        baseline,
        static_grid_snapshot=replace(
            baseline.static_grid_snapshot,
            metadata=replace(
                baseline.static_grid_snapshot.metadata,
                input_valid=False,
            ),
        ),
    )
    controller = SourceDerivedDynamicDwbController()

    result = controller.step(malformed)

    assert result.failure_reason == "grid_snapshot_invalid"
    assert result.requested_twist == Twist2D()
    assert controller.stack_build_count == 0


@pytest.mark.parametrize(
    ("field_name", "replacement"),
    [
        ("control_period_s", 0.10),
        ("rollout_duration_s", 1.0),
        ("integration_step_s", 0.10),
        ("maximum_forward_speed_mps", 0.19),
        ("maximum_reverse_speed_mps", 0.09),
        ("linear_acceleration_mps2", 0.20),
        ("linear_deceleration_mps2", 0.40),
        ("maximum_angular_speed_radps", 0.70),
        ("angular_acceleration_radps2", 1.50),
        ("angular_deceleration_radps2", 1.50),
        ("linear_sample_count", 6),
        ("angular_sample_count", 29),
        ("allow_reverse", True),
    ],
)
def test_every_frozen_v8_generator_parameter_is_enforced(
    field_name: str,
    replacement,
) -> None:
    generator = replace(DwbGeneratorConfig(), **{field_name: replacement})
    config = SourceDerivedDwbConfig(generator=generator)

    with pytest.raises(ValueError, match="every frozen v8 parameter"):
        SourceDerivedDynamicDwbController(config=config)
