from __future__ import annotations

from dataclasses import replace
from hashlib import sha256
from json import dumps
from math import inf, isclose, pi
from random import Random
from unittest.mock import patch

import numpy as np

import hospital_path_lab.collision as collision_module
from hospital_path_lab.collision import (
    CollisionChecker,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.contracts import (
    GridSnapshot,
    PlanStatus,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    TrajectoryPoint,
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
from hospital_path_lab.dynamic_corpus import (
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
)
from hospital_path_lab.dynamic_prediction import build_actor_prediction_set
from hospital_path_lab.dynamic_runner import (
    _EpisodeContextFactory,
    _qualification_snapshot_cases,
)
from hospital_path_lab.dynamic_safety import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicSafetyGate,
)
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_algorithms.dwa import (
    DynamicDwaCandidateCause,
    DynamicDwaCandidatePhase,
    DynamicDwaController,
    _coarse_dynamic_candidate_clearance,
    _dynamic_candidate,
    _dynamic_constant_rollout,
    _shared_gate_failure_cause,
    _StepActorTubeSampler,
    dynamic_dwa_controller_semantic_digest,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _controller_snapshot(
    *,
    tick_id: int = 0,
    state: RobotState | None = None,
    occupancy: np.ndarray | None = None,
    actor: ActorTrack | None = None,
):
    state = state or RobotState(Pose2D(1.0, 1.0), Twist2D(0.20, 0.0))
    simulation_time_s = tick_id * 0.05
    tracks = () if actor is None else (actor,)
    frame = DynamicObservationFrame(
        stream_id="stream-v1",
        episode_id="episode-v1",
        episode_seed=1,
        map_id="map-v1",
        map_revision=1,
        observation_revision=tick_id,
        sequence=tick_id,
        observed_at_s=simulation_time_s,
        delivered_at_s=simulation_time_s,
        frame_kind=(
            DynamicObservationFrameKind.EMPTY
            if actor is None
            else DynamicObservationFrameKind.TRACKS
        ),
        tracks=tracks,
        content_hash=f"observation-{tick_id}",
    )
    observation = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.FRESH,
        frame=frame,
        age_s=0.0,
        failures=(),
        last_event_was_no_frame=False,
    )
    if occupancy is None:
        occupancy = np.zeros((180, 180), dtype=np.bool_)
    grid = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="map-v1",
            map_revision=1,
            mission_revision=1,
            observation_revision=tick_id,
            seed=1,
            content_hash=f"grid-{tick_id}",
        ),
        grid=GridMap(occupancy, resolution_m=0.02),
    )
    return build_controller_snapshot(
        tick_id=tick_id,
        simulation_time_s=simulation_time_s,
        mission_id="mission-v1",
        robot_state=state,
        goal_pose=Pose2D(2.4, 1.0),
        reference_path=(Pose2D(1.0, 1.0), Pose2D(2.4, 1.0)),
        static_grid_snapshot=grid,
        validated_observation=observation,
        actor_tubes=build_actor_prediction_set(observation),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )


def test_dynamic_dwa_uses_217_candidates_and_41_post_apply_poses() -> None:
    snapshot = _controller_snapshot()

    result = DynamicDwaController().step(snapshot)

    assert result.status is PlanStatus.FOUND
    assert result.requested_twist.linear == 0.20
    assert result.requested_twist.linear >= 0.0
    assert len(result.predicted_trajectory) == 41
    assert result.predicted_trajectory[0].time_s == 0.0
    assert "sampled_candidates=217" in result.decision_trace
    assert "pose_samples=41" in result.decision_trace


def test_dynamic_dwa_cost_equations_match_the_frozen_oracle() -> None:
    trajectory = tuple(
        TrajectoryPoint(
            time_s=index * 0.05,
            pose=Pose2D(index * 0.01, 0.0),
            twist=Twist2D(0.20, 0.0),
        )
        for index in range(41)
    )

    candidate = _dynamic_candidate(
        Twist2D(0.20, 0.0),
        trajectory,
        start=Pose2D(0.0, 0.0),
        goal=Pose2D(1.0, 0.0),
        reference_path=(Pose2D(0.0, 0.0), Pose2D(1.0, 0.0)),
        minimum_clearance=inf,
        previous_angular=0.0,
    )

    assert candidate.progress_cost == 0.0
    assert candidate.reference_path_cost == 0.0
    assert candidate.heading_cost == 0.0
    assert candidate.clearance_cost == 0.0
    assert candidate.speed_cost == 0.0
    assert candidate.oscillation_cost == 0.0
    assert candidate.score == 0.0


def test_dynamic_dwa_nontrivial_cost_and_rank_match_the_pre_v6_oracle() -> None:
    trajectory = _dynamic_constant_rollout(
        Pose2D(1.01, 1.0),
        Twist2D(0.195, 0.04),
        horizon_s=2.0,
        step_s=0.05,
    )

    candidate = _dynamic_candidate(
        Twist2D(0.195, 0.04),
        trajectory,
        start=Pose2D(1.0, 1.0),
        goal=Pose2D(2.4, 1.0),
        reference_path=(
            Pose2D(1.0, 1.0),
            Pose2D(1.7, 1.1),
            Pose2D(2.4, 1.0),
        ),
        minimum_clearance=0.19,
        previous_angular=-0.20,
    )

    assert tuple(
        getattr(candidate, field).hex()
        for field in (
            "progress",
            "progress_cost",
            "reference_path_cost",
            "heading_cost",
            "clearance_cost",
            "speed_cost",
            "oscillation_cost",
            "score",
        )
    ) == (
        "0x1.990cbc07b5d10p-2",
        "0x1.6029ecb975800p-10",
        "0x1.856189bfca809p-5",
        "0x1.f27d02b458df7p-6",
        "0x1.79e79e79e79e8p-1",
        "0x1.99999999999a0p-6",
        "0x0.0p+0",
        "0x1.2d1d75be7dfb4p+0",
    )
    assert tuple(value.hex() for value in candidate.rank) == (
        "0x1.2d1d75be7dfb4p+0",
        "-0x1.851eb851eb852p-3",
        "-0x1.990cbc07b5d10p-2",
        "0x1.856189bfca809p-5",
        "0x1.f27d02b458df7p-6",
        "0x0.0p+0",
        "0x1.47ae147ae147bp-5",
        "-0x1.8f5c28f5c28f6p-3",
        "0x1.47ae147ae147bp-5",
    )


def test_dynamic_dwa_reverse_is_disabled_and_zero_does_not_add_a_sample() -> None:
    controller = DynamicDwaController()
    linear, angular = controller._dynamic_window(RobotState(Pose2D(1.0, 1.0), Twist2D(0.0, 0.03)))

    assert len(linear) == 7
    assert len(angular) == 31
    assert min(linear) == 0.0
    assert max(linear) <= 0.20
    assert 0.0 in angular


def test_dynamic_dwa_rejects_every_candidate_when_terminal_stop_is_blocked() -> None:
    occupancy = np.ones((120, 180), dtype=np.bool_)
    occupancy[30:80, 10:72] = False
    snapshot = _controller_snapshot(
        state=RobotState(Pose2D(1.0, 1.0), Twist2D(0.20, 0.0)),
        occupancy=occupancy,
    )

    result = DynamicDwaController().step(snapshot)

    assert result.status is PlanStatus.NO_PATH
    assert result.requested_twist == Twist2D()
    assert result.failure_reason == "no_safe_candidate"
    assert result.no_safe_candidate
    assert result.controller_requested_stop


def test_dynamic_dwa_is_deterministic_except_for_elapsed_time() -> None:
    snapshot = _controller_snapshot()
    results = [DynamicDwaController().step(snapshot) for _ in range(2)]
    signatures = {
        (
            result.status,
            result.requested_twist,
            result.predicted_trajectory,
            result.decision_trace,
            result.failure_reason,
        )
        for result in results
    }

    assert len(signatures) == 1
    assert isclose(results[0].requested_twist.linear, 0.20)


def test_dynamic_dwa_can_receive_a_moving_actor_without_ground_truth() -> None:
    actor = ActorTrack(
        track_id="track-1",
        actor_binding_id="actor-1",
        observed_position=Point2D(1.6, 1.15),
        observed_velocity=Vector2D(0.0, 0.50),
        position_sigma_m=0.0,
        velocity_sigma_mps=0.0,
    )
    snapshot = _controller_snapshot(actor=actor)

    result = DynamicDwaController().step(snapshot)

    assert result.status in {PlanStatus.FOUND, PlanStatus.NO_PATH}
    assert result.observation_content_hash == snapshot.observation_content_hash
    assert not hasattr(snapshot, "ground_truth_actors")


def test_dynamic_dwa_v6_taxonomy_separates_selected_from_unchecked_candidates() -> None:
    controller = DynamicDwaController()

    result = controller.step(_controller_snapshot())

    summary = controller.last_diagnostics
    assert summary is not None
    assert summary.sampled_candidates == 217
    assert summary.moving_candidates == 217
    assert summary.coarse_admissible_candidates == 217
    assert summary.selected_sample_index == 201
    assert summary.selected_rank == 0
    assert (
        _taxonomy_count(
            summary,
            DynamicDwaCandidatePhase.RANKING,
            DynamicDwaCandidateCause.SELECTED,
        )
        == 1
    )
    assert (
        _taxonomy_count(
            summary,
            DynamicDwaCandidatePhase.RANKING,
            DynamicDwaCandidateCause.ADMISSIBLE_NOT_SELECTED,
        )
        == 0
    )
    assert (
        _taxonomy_count(
            summary,
            DynamicDwaCandidatePhase.RANKING,
            DynamicDwaCandidateCause.NOT_EVALUATED_AFTER_SELECTION,
        )
        == 216
    )
    assert "ranking_admissibility_scope=exact_checked_only" in result.decision_trace


def test_dynamic_dwa_full_diagnostic_mode_proves_admissible_not_selected() -> None:
    controller = DynamicDwaController(verify_all_ranked_candidates=True)
    controller.linear_sample_count = 2
    controller.angular_sample_count = 3

    result = controller.step(_controller_snapshot())

    summary = controller.last_diagnostics
    assert summary is not None
    assert result.status is PlanStatus.FOUND
    assert summary.sampled_candidates == 6
    assert (
        _taxonomy_count(
            summary,
            DynamicDwaCandidatePhase.RANKING,
            DynamicDwaCandidateCause.SELECTED,
        )
        == 1
    )
    assert (
        _taxonomy_count(
            summary,
            DynamicDwaCandidatePhase.RANKING,
            DynamicDwaCandidateCause.ADMISSIBLE_NOT_SELECTED,
        )
        == 5
    )
    assert (
        _taxonomy_count(
            summary,
            DynamicDwaCandidatePhase.RANKING,
            DynamicDwaCandidateCause.NOT_EVALUATED_AFTER_SELECTION,
        )
        == 0
    )


def test_dynamic_dwa_v6_actor_rejections_are_counted_and_details_are_bounded() -> None:
    actor = ActorTrack(
        track_id="track-1",
        actor_binding_id="actor-1",
        observed_position=Point2D(1.6, 1.15),
        observed_velocity=Vector2D(0.0, 0.50),
        position_sigma_m=0.0,
        velocity_sigma_mps=0.0,
    )
    controller = DynamicDwaController()

    result = controller.step(_controller_snapshot(actor=actor))

    summary = controller.last_diagnostics
    assert summary is not None
    assert result.status is PlanStatus.NO_PATH
    assert (
        _taxonomy_count(
            summary,
            DynamicDwaCandidatePhase.COARSE_ROLLOUT,
            DynamicDwaCandidateCause.ACTOR_TUBE,
        )
        == 217
    )
    assert len(summary.details) == 8
    assert tuple(detail.sample_index for detail in summary.details) == tuple(range(8))
    assert all(
        detail.phase is DynamicDwaCandidatePhase.COARSE_ROLLOUT
        and detail.cause is DynamicDwaCandidateCause.ACTOR_TUBE
        for detail in summary.details
    )


def test_dynamic_dwa_v6_terminal_failure_keeps_underlying_cause() -> None:
    occupancy = np.zeros((180, 180), dtype=np.bool_)
    occupancy[:, 89] = True
    snapshot = _controller_snapshot(occupancy=occupancy)
    checker = CollisionChecker(
        snapshot.static_grid_snapshot.grid,
        VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )
    trajectory = _dynamic_constant_rollout(
        Pose2D(1.01, 1.0),
        Twist2D(0.20, 0.0),
        horizon_s=2.0,
        step_s=0.05,
    )

    outcome = _coarse_dynamic_candidate_clearance(
        trajectory,
        snapshot=snapshot,
        physical_checker=checker,
        combined_checker=checker,
        vehicle=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        actor_sampler=_StepActorTubeSampler(snapshot.actor_tubes, enabled=True),
    )

    assert not outcome.accepted
    assert outcome.failure_phase is DynamicDwaCandidatePhase.COARSE_TERMINAL
    assert outcome.failure_cause is DynamicDwaCandidateCause.TERMINAL_STOPPING
    assert outcome.underlying_terminal_cause is DynamicDwaCandidateCause.STATIC_CLEARANCE
    assert outcome.failure_time_s == 2.05


def test_dynamic_dwa_step_local_workspace_preserves_all_non_timing_semantics() -> None:
    actor = ActorTrack(
        track_id="track-1",
        actor_binding_id="actor-1",
        observed_position=Point2D(1.6, 1.15),
        observed_velocity=Vector2D(0.0, 0.50),
        position_sigma_m=0.0,
        velocity_sigma_mps=0.0,
    )
    for snapshot in (_controller_snapshot(), _controller_snapshot(actor=actor)):
        reference = DynamicDwaController(use_step_local_workspace=False)
        optimized = DynamicDwaController(use_step_local_workspace=True)

        reference_result = reference.step(snapshot)
        optimized_result = optimized.step(snapshot)

        assert dynamic_dwa_controller_semantic_digest(
            reference_result
        ) == dynamic_dwa_controller_semantic_digest(optimized_result)
        assert reference.last_diagnostics is not None
        assert optimized.last_diagnostics is not None
        assert (
            reference.last_diagnostics.semantic_digest == optimized.last_diagnostics.semantic_digest
        )


def test_dynamic_dwa_boundary_precedes_actor_for_all_217_candidates() -> None:
    actor = ActorTrack(
        track_id="boundary-track",
        actor_binding_id="boundary-actor",
        observed_position=Point2D(0.55, 1.0),
        observed_velocity=Vector2D(0.0, 0.0),
        position_sigma_m=0.0,
        velocity_sigma_mps=0.0,
    )
    snapshot = _controller_snapshot(
        state=RobotState(Pose2D(0.35, 1.0), Twist2D(0.20, 0.0)),
        actor=actor,
    )
    reference = DynamicDwaController(use_step_local_workspace=False)
    optimized = DynamicDwaController(use_step_local_workspace=True)

    reference_result = reference.step(snapshot)
    optimized_result = optimized.step(snapshot)

    assert reference.last_diagnostics is not None
    assert optimized.last_diagnostics is not None
    for summary in (reference.last_diagnostics, optimized.last_diagnostics):
        assert summary.sampled_candidates == 217
        assert summary.moving_candidates == 217
        assert (
            _taxonomy_count(
                summary,
                DynamicDwaCandidatePhase.COARSE_ROLLOUT,
                DynamicDwaCandidateCause.STATIC_CLEARANCE,
            )
            == 217
        )
        assert (
            _taxonomy_count(
                summary,
                DynamicDwaCandidatePhase.COARSE_ROLLOUT,
                DynamicDwaCandidateCause.ACTOR_TUBE,
            )
            == 0
        )
        assert summary.semantic_digest == (
            "5725b8dfe5303af3084acb1afffaf63b0ebebdaa33b840a7b4629e0309426303"
        )
    assert optimized.last_workspace_metrics.reference_geometry_candidates == 217
    assert optimized.last_workspace_metrics.certified_actor_dominated_candidates == 0
    assert dynamic_dwa_controller_semantic_digest(reference_result) == (
        "9f7fa4689357d8e738fc314c2dea50dc35fd2842ea68bd80d21a24337801b5b0"
    )
    assert dynamic_dwa_controller_semantic_digest(optimized_result) == (
        dynamic_dwa_controller_semantic_digest(reference_result)
    )


def test_dynamic_dwa_v6_controller_digest_is_embedded_and_stable() -> None:
    result = DynamicDwaController().step(_controller_snapshot())
    expected = dynamic_dwa_controller_semantic_digest(result)

    assert f"dwa_controller_semantic_digest={expected}" in result.decision_trace
    assert len(expected) == 64
    assert dynamic_dwa_controller_semantic_digest(result) == expected


def test_dynamic_dwa_optimized_motion_matches_the_pre_v6_frozen_signature() -> None:
    result = DynamicDwaController().step(_controller_snapshot())

    assert _legacy_motion_digest(result) == (
        "058bdbfa989293ca2ea69a1ddc0cd82388fd636a882d3657a6a2b2671ee5ed66"
    )


def test_dynamic_dwa_invalid_prediction_has_structured_input_diagnostic() -> None:
    snapshot = replace(_controller_snapshot(), actor_tubes=None)
    controller = DynamicDwaController()

    result = controller.step(snapshot)

    summary = controller.last_diagnostics
    assert summary is not None
    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "actor_prediction_missing"
    assert (
        _taxonomy_count(
            summary,
            DynamicDwaCandidatePhase.INPUT,
            DynamicDwaCandidateCause.PREDICTION_INVALID,
        )
        == 1
    )
    assert summary.details[0].shared_gate_failures == ("actor_prediction_missing",)


def test_dynamic_dwa_exact_shared_gate_failures_have_deterministic_precedence() -> None:
    assert _shared_gate_failure_cause(
        ("forbidden_zone_entry", "actor_clearance_below_minimum")
    ) is DynamicDwaCandidateCause.FORBIDDEN_ZONE
    assert _shared_gate_failure_cause(
        ("static_clearance_below_minimum", "actor_clearance_below_minimum")
    ) is DynamicDwaCandidateCause.STATIC_CLEARANCE
    assert _shared_gate_failure_cause(
        ("actor_clearance_below_minimum",)
    ) is DynamicDwaCandidateCause.ACTOR_TUBE
    assert _shared_gate_failure_cause(
        ("proposal_trajectory_invalid:bad sample",)
    ) is DynamicDwaCandidateCause.PREDICTION_INVALID
    assert _shared_gate_failure_cause(("unknown_shared_failure",)) is (
        DynamicDwaCandidateCause.SHARED_GATE
    )


def test_oriented_circle_fast_geometry_matches_reference_exactly() -> None:
    random = Random(0xD6A)
    for _ in range(4_000):
        pose = Pose2D(
            random.uniform(-20.0, 20.0),
            random.uniform(-20.0, 20.0),
            random.uniform(-pi, pi),
        )
        circle_center = (
            pose.x + random.uniform(-3.0, 3.0),
            pose.y + random.uniform(-3.0, 3.0),
        )
        circle_radius_m = random.uniform(0.0, 2.0)
        common = {
            "circle_center": circle_center,
            "circle_radius_m": circle_radius_m,
            "profile": VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        }

        reference = oriented_footprint_circle_surface_distance(
            pose,
            **common,
            use_optimized_geometry=False,
        )
        optimized = oriented_footprint_circle_surface_distance(
            pose,
            **common,
            use_optimized_geometry=True,
        )
        validated = oriented_footprint_circle_surface_distance(
            pose,
            **common,
            use_optimized_geometry=True,
            inputs_validated=True,
        )

        assert optimized == reference
        assert validated == reference


def test_chebyshev_clearance_field_matches_brute_force_grid_distance() -> None:
    random = np.random.default_rng(0xC4EB)
    for _ in range(8):
        occupancy = random.random((17, 19)) < 0.12
        occupancy[8, 9] = True
        checker = CollisionChecker(GridMap(occupancy, resolution_m=0.02))
        occupied_y, occupied_x = np.nonzero(occupancy)
        expected_cells = np.empty(occupancy.shape, dtype=np.int32)
        for y in range(occupancy.shape[0]):
            for x in range(occupancy.shape[1]):
                expected_cells[y, x] = min(
                    max(abs(x - int(cell_x)), abs(y - int(cell_y)))
                    for cell_y, cell_x in zip(
                        occupied_y,
                        occupied_x,
                        strict=True,
                    )
                )

        expected = expected_cells.astype(np.float64) * 0.02
        assert np.array_equal(
            checker._center_chebyshev_distance_field_m,
            expected,
        )


def test_collision_step_geometry_preserves_exact_static_and_forbidden_results_with_less_cell_work(
) -> None:
    occupancy = np.zeros((240, 250), dtype=np.bool_)
    occupancy[:20, :] = True
    occupancy[-20:, :] = True
    occupancy[:, :20] = True
    occupancy[:, -20:] = True
    occupancy[70:170, 105:125] = True
    forbidden = frozenset(
        (x, y)
        for y in range(75, 165)
        for x in range(145, 165)
    )
    grid = GridMap(occupancy, resolution_m=0.02)
    poses = tuple(
        Pose2D(0.70 + index * 0.025, 1.10, 0.0)
        for index in range(50)
    ) + tuple(
        Pose2D(1.95, 1.10 + index * 0.025, 1.5707963267948966)
        for index in range(70)
    )
    reference = CollisionChecker(
        grid,
        forbidden_cells=forbidden,
        use_optimized_geometry=False,
    )
    optimized = CollisionChecker(
        grid,
        forbidden_cells=forbidden,
        use_optimized_geometry=True,
    )

    original_reference_distance = collision_module._convex_polygon_distance
    with patch.object(
        collision_module,
        "_convex_polygon_distance",
        wraps=original_reference_distance,
    ) as reference_distance:
        reference_results = tuple(
            (
                reference.clearance(pose),
                reference.forbidden_clearance(pose),
                reference.pose_enters_forbidden(pose),
            )
            for pose in poses
        )
    original_optimized_distance = collision_module._oriented_footprint_cell_distance
    with patch.object(
        collision_module,
        "_oriented_footprint_cell_distance",
        wraps=original_optimized_distance,
    ) as optimized_distance:
        optimized_results = tuple(
            (
                optimized.clearance(pose),
                optimized.forbidden_clearance(pose),
                optimized.pose_enters_forbidden(pose),
            )
            for pose in poses
        )

    assert optimized_results == reference_results
    assert all(
        optimized.certified_clearance_lower_bound(pose)
        <= optimized.clearance(pose) + 1e-12
        for pose in poses
    )
    assert optimized.certified_minimum_clearance_lower_bound(poses) == min(
        optimized.certified_clearance_lower_bound(pose) for pose in poses
    )
    assert optimized_distance.call_count * 4 < reference_distance.call_count


def test_dynamic_dwa_step_workspace_matches_all_public_episode_profile_representatives() -> None:
    """One usable tick per 49 episode x 2 profiles; not a 900-tick exhaustive proof."""

    episodes = (*generate_dynamic_corpus(), *generate_dynamic_v6_public_corpus())
    profiles = (NORMAL_OBSERVATION_PROFILE, STRESS_OBSERVATION_PROFILE)
    compared = 0
    coverage = {
        "static": False,
        "forbidden": False,
        "multisegment": False,
        "two_actor": False,
    }
    assert len(episodes) == 49

    for episode in episodes:
        for profile in profiles:
            snapshot = _representative_controller_snapshot(episode, profile)
            coverage["static"] |= bool(
                np.any(snapshot.static_grid_snapshot.grid.occupancy)
            )
            coverage["forbidden"] |= bool(
                snapshot.static_grid_snapshot.forbidden_cells
            )
            coverage["multisegment"] |= len(snapshot.reference_path) > 2
            coverage["two_actor"] |= bool(
                snapshot.actor_tubes is not None
                and len(snapshot.actor_tubes.tubes) >= 2
            )

            reference = DynamicDwaController(use_step_local_workspace=False)
            optimized = DynamicDwaController(use_step_local_workspace=True)
            for controller in (reference, optimized):
                # Preserve the full 41-pose/2 s candidate semantics while
                # bounding this cross-corpus regression to one moving sample.
                controller.linear_sample_count = 2
                controller.angular_sample_count = 1

            reference_result = reference.step(snapshot)
            optimized_result = optimized.step(snapshot)

            assert dynamic_dwa_controller_semantic_digest(
                optimized_result
            ) == dynamic_dwa_controller_semantic_digest(reference_result), (
                episode.episode_id,
                profile.name.value,
            )
            assert reference.last_diagnostics is not None
            assert optimized.last_diagnostics is not None
            assert optimized.last_diagnostics.semantic_digest == (
                reference.last_diagnostics.semantic_digest
            )
            compared += 1

    assert compared == 98
    assert all(coverage.values())


def test_dynamic_dwa_real_corner_and_multisegment_reduce_exact_geometry_by_work_count() -> None:
    corpus = (*generate_dynamic_corpus(), *generate_dynamic_v6_public_corpus())
    cases = {
        case_id: (snapshot, metadata)
        for case_id, snapshot, metadata in _qualification_snapshot_cases(corpus)
        if case_id in {
            "corner-static-forbidden",
            "staggered-risk-multisegment",
        }
    }

    assert set(cases) == {
        "corner-static-forbidden",
        "staggered-risk-multisegment",
    }
    for snapshot, metadata in cases.values():
        controller = DynamicDwaController()
        controller.step(snapshot)

        metrics = controller.last_workspace_metrics
        assert metadata["has_static_occupancy"] is True
        assert metadata["has_forbidden_cells"] is True
        assert metadata["reference_path_segment_count"] >= 2
        assert metrics.coarse_candidates > 0
        assert metrics.certified_actor_dominated_candidates > (
            5 * metrics.reference_geometry_candidates
        )


def _representative_controller_snapshot(episode, profile):
    context_factory = _EpisodeContextFactory(episode, profile)
    gate = DynamicSafetyGate()
    for tick_id in range(episode.tick_count):
        simulation_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        context = context_factory(
            tick_id,
            simulation_time_s,
            episode.initial_state,
            gate,
        )
        if context.prediction_set is None:
            continue
        return build_controller_snapshot(
            tick_id=tick_id,
            simulation_time_s=simulation_time_s,
            mission_id=episode.mission_id,
            robot_state=episode.initial_state,
            goal_pose=episode.goal_pose,
            reference_path=episode.reference_path,
            static_grid_snapshot=context.grid_snapshot,
            validated_observation=context.observation_snapshot,
            actor_tubes=context.prediction_set,
            vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        )
    raise AssertionError(f"no usable representative observation: {episode.episode_id}")


def _taxonomy_count(summary, phase, cause) -> int:
    return next(
        count
        for recorded_phase, recorded_cause, count in summary.ordered_counts
        if recorded_phase == phase.value and recorded_cause == cause.value
    )


def _legacy_motion_digest(result) -> str:
    payload = {
        "status": result.status.value,
        "twist": [result.requested_twist.linear, result.requested_twist.angular],
        "trajectory": [
            [
                point.time_s,
                point.pose.x,
                point.pose.y,
                point.pose.yaw,
                point.twist.linear,
                point.twist.angular,
            ]
            for point in result.predicted_trajectory
        ],
        "failure": result.failure_reason,
        "flags": [result.controller_requested_stop, result.no_safe_candidate],
    }
    return sha256(
        dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()
