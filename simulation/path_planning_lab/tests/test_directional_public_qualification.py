from __future__ import annotations

from dataclasses import dataclass, replace
from enum import StrEnum
from math import cos, isfinite, sin

import pytest

from hospital_path_lab.collision import (
    oriented_footprint_capsule_surface_distance,
)
from hospital_path_lab.contracts import Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicMotionState,
)
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    V6DynamicCorpusEpisode,
    build_dynamic_grid_snapshot,
    controller_episode_id,
    generate_dynamic_v6_public_corpus,
    generate_episode_observation_slots,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalActorPredictor,
    DirectionalPredictionResult,
    DirectionalPredictionSet,
    DirectionalPredictionStatus,
    sample_directional_capsules,
)
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationProfile,
    DynamicObservationSlot,
    DynamicObservationSnapshot,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
)
from hospital_path_lab.dynamic_safety import (
    DynamicSafetyContext,
    DynamicSafetyGate,
    build_dynamic_command_proposal,
    build_resume_authorization,
)
from hospital_path_lab.local_algorithms.dwb_reference.contracts import (
    DwbGeneratorRequest,
    DwbPose2D,
    DwbTwist2D,
)
from hospital_path_lab.local_algorithms.dwb_reference.trajectory_generator import (
    DwbReferenceTrajectoryGenerator,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1

_TERMINAL_SWEEP_PERIOD_S = 0.005
_ANGULAR_DECELERATION_RADPS2 = 1.60
_FLOAT_TOLERANCE = 1e-12


class DirectionalPublicQualificationStatus(StrEnum):
    PREDICTION_GEOMETRY_QUALIFIED = "prediction_geometry_qualified"
    ONLINE_DWB_BYPASS_UNPROVEN = "online_dwb_bypass_unproven"


@dataclass(frozen=True, slots=True)
class _GeometryQualificationReport:
    episode_id: str
    candidate_count: int
    rollout_pose_count_per_candidate: int
    exact_capsule_clearance_count: int
    legacy_witness_horizon_s: float
    maximum_verified_prediction_horizon_s: float
    clearance_digest: tuple[str, ...]
    prediction_status: DirectionalPublicQualificationStatus
    online_dwb_status: DirectionalPublicQualificationStatus


def _same_direction_public_episodes() -> tuple[V6DynamicCorpusEpisode, ...]:
    public = generate_dynamic_v6_public_corpus()
    assert len(public) == 13
    assert all(episode.split is not DynamicCorpusSplit.HIDDEN for episode in public)
    selected = tuple(
        episode
        for episode in public
        if episode.latent_case_id.startswith("same-direction-wide")
    )
    assert len(selected) == 5
    return selected


def _prediction_results(
    episode: V6DynamicCorpusEpisode,
    profile: DynamicObservationProfile,
):
    source = DynamicObservationSourceIdentity(
        stream_id="dynamic-stage5-stream",
        episode_id=controller_episode_id(episode),
        episode_seed=episode.seed,
        map_id=episode.map_id,
        map_revision=1,
    )
    validator = DynamicObservationValidator(source, profile)
    predictor = DirectionalActorPredictor()
    results = []
    for slot in generate_episode_observation_slots(episode, profile=profile):
        if slot.frame is None:
            validator.record_no_frame(
                sequence=slot.sequence,
                delivery_time_s=slot.scheduled_delivery_at_s,
            )
        else:
            acceptance = validator.accept(
                slot.frame,
                received_at_s=slot.scheduled_delivery_at_s,
            )
            assert acceptance.accepted
        snapshot = validator.snapshot(control_time_s=slot.scheduled_delivery_at_s)
        results.append((slot, snapshot, predictor.update(snapshot)))
    return tuple(results)


def _first_ready_prediction(
    episode: V6DynamicCorpusEpisode,
) -> DirectionalPredictionSet:
    for _slot, _snapshot, result in _prediction_results(
        episode,
        NORMAL_OBSERVATION_PROFILE,
    ):
        if result.status is DirectionalPredictionStatus.READY:
            assert result.prediction_set is not None
            return result.prediction_set
    raise AssertionError(f"{episode.episode_id}: no READY directional prediction")


def _directional_observation_safe_candidate(
    result: DirectionalPredictionResult,
) -> bool:
    """v7 candidate flag; authority and trajectory safety remain separate AND gates."""

    return result.status in {
        DirectionalPredictionStatus.READY,
        DirectionalPredictionStatus.EMPTY_FRAME,
    }


def _first_consecutive_ready_run(
    episode: V6DynamicCorpusEpisode,
    *,
    required_count: int,
) -> tuple[
    tuple[
        DynamicObservationSlot,
        DynamicObservationSnapshot,
        DirectionalPredictionResult,
    ],
    ...,
]:
    run = []
    previous_sequence: int | None = None
    for item in _prediction_results(episode, NORMAL_OBSERVATION_PROFILE):
        slot, _snapshot, result = item
        if (
            result.status is DirectionalPredictionStatus.READY
            and slot.frame is not None
            and (
                previous_sequence is None
                or slot.sequence == previous_sequence + 1
            )
        ):
            run.append(item)
            previous_sequence = slot.sequence
            if len(run) == required_count:
                return tuple(run)
        else:
            run = []
            previous_sequence = None
    raise AssertionError(
        f"{episode.episode_id}: no {required_count}-frame READY run"
    )


def _directional_context(
    episode: V6DynamicCorpusEpisode,
    *,
    tick_id: int,
    simulation_time_s: float,
    snapshot: DynamicObservationSnapshot,
    prediction_result: DirectionalPredictionResult,
    gate: DynamicSafetyGate,
    authorize: bool,
) -> DynamicSafetyContext:
    assert snapshot.frame is not None
    authorization = None
    if authorize:
        authorization = build_resume_authorization(
            mission_id=episode.mission_id,
            stop_epoch=gate.stop_epoch,
            issued_or_revalidated_at_s=simulation_time_s,
            authorization_revision=7,
        )
    return DynamicSafetyContext(
        tick_id=tick_id,
        simulation_time_s=simulation_time_s,
        mission_id=episode.mission_id,
        authorization_revision=7,
        grid_snapshot=build_dynamic_grid_snapshot(
            episode,
            observation_revision=snapshot.frame.observation_revision,
        ),
        observation_snapshot=snapshot,
        prediction_set=prediction_result.prediction_set,
        path_still_valid=True,
        local_safety_recheck_passed=True,
        observation_safe=_directional_observation_safe_candidate(prediction_result),
        resume_authorization=authorization,
    )


def _drive_directional_gate_to_holding(
    episode: V6DynamicCorpusEpisode,
    gate: DynamicSafetyGate,
) -> None:
    warming = tuple(
        item
        for item in _prediction_results(episode, NORMAL_OBSERVATION_PROFILE)
        if item[0].frame is not None
        and item[1].usable
        and item[2].status is DirectionalPredictionStatus.WARMING_UP
    )[:3]
    assert len(warming) == 3
    stopped = RobotState(episode.initial_state.pose, Twist2D())
    for tick_id, (slot, snapshot, result) in enumerate(warming):
        assert not _directional_observation_safe_candidate(result)
        context = _directional_context(
            episode,
            tick_id=tick_id,
            simulation_time_s=slot.scheduled_delivery_at_s,
            snapshot=snapshot,
            prediction_result=result,
            gate=gate,
            authorize=False,
        )
        decision = gate.step(
            build_dynamic_command_proposal(
                context,
                command=Twist2D(),
                computation_time_s=0.001,
            ),
            robot_state=stopped,
            context=context,
        )
    assert decision.motion_state is DynamicMotionState.HOLDING
    assert decision.stop_epoch == 1


def _integrate_pose(
    pose: DwbPose2D,
    twist: DwbTwist2D,
    duration_s: float,
) -> DwbPose2D:
    if abs(twist.angular_radps) <= _FLOAT_TOLERANCE:
        return DwbPose2D(
            pose.x_m + twist.linear_mps * cos(pose.yaw_rad) * duration_s,
            pose.y_m + twist.linear_mps * sin(pose.yaw_rad) * duration_s,
            pose.yaw_rad,
        )
    next_yaw = pose.yaw_rad + twist.angular_radps * duration_s
    radius_m = twist.linear_mps / twist.angular_radps
    return DwbPose2D(
        pose.x_m + radius_m * (sin(next_yaw) - sin(pose.yaw_rad)),
        pose.y_m - radius_m * (cos(next_yaw) - cos(pose.yaw_rad)),
        next_yaw,
    )


def _toward_zero(value: float, delta: float) -> float:
    if value > 0.0:
        return max(0.0, value - delta)
    if value < 0.0:
        return min(0.0, value + delta)
    return 0.0


def _terminal_stopping_poses(
    pose: DwbPose2D,
    twist: DwbTwist2D,
) -> tuple[tuple[float, DwbPose2D], ...]:
    profile = VIRTUAL_DOLL_WHEELCHAIR_V0_1
    samples = [(0.0, pose)]
    elapsed_s = 0.0
    while (
        abs(twist.linear_mps) > _FLOAT_TOLERANCE
        or abs(twist.angular_radps) > _FLOAT_TOLERANCE
    ):
        pose = _integrate_pose(pose, twist, _TERMINAL_SWEEP_PERIOD_S)
        twist = DwbTwist2D(
            _toward_zero(
                twist.linear_mps,
                profile.max_deceleration_mps2 * _TERMINAL_SWEEP_PERIOD_S,
            ),
            _toward_zero(
                twist.angular_radps,
                _ANGULAR_DECELERATION_RADPS2 * _TERMINAL_SWEEP_PERIOD_S,
            ),
        )
        elapsed_s += _TERMINAL_SWEEP_PERIOD_S
        samples.append((elapsed_s, pose))
    return tuple(samples)


def _exact_geometry_report(
    episode: V6DynamicCorpusEpisode,
    prediction_set: DirectionalPredictionSet,
) -> _GeometryQualificationReport:
    generator = DwbReferenceTrajectoryGenerator()
    initial = episode.initial_state
    generated = generator.generate(
        DwbGeneratorRequest(
            pose=DwbPose2D(
                initial.pose.x,
                initial.pose.y,
                initial.pose.yaw,
            ),
            current_twist=DwbTwist2D(
                initial.twist.linear,
                initial.twist.angular,
            ),
        )
    )
    digest: list[str] = []
    maximum_horizon_s = 0.0
    for trajectory in generated.trajectories:
        timed_poses = [
            (index * trajectory.integration_step_s, pose)
            for index, pose in enumerate(trajectory.poses)
        ]
        terminal = _terminal_stopping_poses(
            trajectory.poses[-1],
            trajectory.command,
        )
        timed_poses.extend(
            (generator.config.rollout_duration_s + offset_s, pose)
            for offset_s, pose in terminal[1:]
        )
        for rollout_time_s, robot_pose in timed_poses:
            capsules = sample_directional_capsules(
                prediction_set,
                rollout_time_s=rollout_time_s,
            )
            for capsule in capsules:
                clearance_m = oriented_footprint_capsule_surface_distance(
                    Pose2D(robot_pose.x_m, robot_pose.y_m, robot_pose.yaw_rad),
                    segment_start=(capsule.start.x, capsule.start.y),
                    segment_end=(capsule.end.x, capsule.end.y),
                    capsule_radius_m=capsule.base_radius_m,
                )
                assert isfinite(clearance_m)
                maximum_horizon_s = max(
                    maximum_horizon_s,
                    capsule.prediction_horizon_s,
                )
                digest.append(clearance_m.hex())

    legacy_horizon_s = (
        NORMAL_OBSERVATION_PROFILE.ttl_s + DYNAMIC_CONTROL_PERIOD_S
    )
    return _GeometryQualificationReport(
        episode_id=episode.episode_id,
        candidate_count=generated.candidate_count,
        rollout_pose_count_per_candidate=(
            generator.config.rollout_step_count + 1
        ),
        exact_capsule_clearance_count=len(digest),
        legacy_witness_horizon_s=legacy_horizon_s,
        maximum_verified_prediction_horizon_s=maximum_horizon_s,
        clearance_digest=tuple(digest),
        prediction_status=(
            DirectionalPublicQualificationStatus.PREDICTION_GEOMETRY_QUALIFIED
        ),
        online_dwb_status=(
            DirectionalPublicQualificationStatus.ONLINE_DWB_BYPASS_UNPROVEN
        ),
    )


def test_normal_same_direction_public_cases_lock_after_twenty_unique_frames() -> None:
    for episode in _same_direction_public_episodes():
        results = _prediction_results(episode, NORMAL_OBSERVATION_PROFILE)
        ready = tuple(
            result
            for _slot, _snapshot, result in results
            if result.status is DirectionalPredictionStatus.READY
        )

        assert ready, f"{episode.latent_case_id}: direction never became READY"
        assert ready[0].history_counts == ((episode.actors[0].actor_id, 20),)
        assert ready[0].prediction_set is not None
        assert ready[0].prediction_set.tubes[0].history_count == 20


def test_stress_same_direction_low_speed_remains_fail_closed() -> None:
    unexpected_ready: list[tuple[str, int, int, int]] = []
    low_confidence_count = 0
    for episode in _same_direction_public_episodes():
        results = _prediction_results(episode, STRESS_OBSERVATION_PROFILE)
        non_ready_tracks = tuple(
            result
            for slot, _snapshot, result in results
            if slot.frame is not None
            and slot.frame.tracks
            and result.status is not DirectionalPredictionStatus.READY
        )

        assert non_ready_tracks
        assert all(result.hold_required for result in non_ready_tracks)
        assert all(result.prediction_set is None for result in non_ready_tracks)
        ready_sequences = tuple(
            slot.sequence
            for slot, _snapshot, result in results
            if result.status is DirectionalPredictionStatus.READY
        )
        if ready_sequences:
            unexpected_ready.append(
                (
                    episode.latent_case_id,
                    len(ready_sequences),
                    ready_sequences[0],
                    ready_sequences[-1],
                )
            )
        low_confidence_count += sum(
            result.status is DirectionalPredictionStatus.LOW_CONFIDENCE
            for result in non_ready_tracks
        )

    assert not unexpected_ready, (
        "Stress low-speed direction must stay fail-closed; unexpected READY "
        f"sequences={unexpected_ready}"
    )
    assert low_confidence_count > 0


@pytest.mark.parametrize(
    ("authorize", "path_valid", "local_recheck", "expected_resume"),
    (
        (True, True, True, True),
        (False, True, True, False),
        (True, False, True, False),
        (True, True, False, False),
    ),
)
def test_directional_lane_warmup_hold_then_ready_requires_every_resume_gate(
    authorize: bool,
    path_valid: bool,
    local_recheck: bool,
    expected_resume: bool,
) -> None:
    episode = _same_direction_public_episodes()[0]
    gate = DynamicSafetyGate()
    _drive_directional_gate_to_holding(episode, gate)
    ready_run = _first_consecutive_ready_run(episode, required_count=11)
    stopped = RobotState(episode.initial_state.pose, Twist2D())

    for offset, (slot, snapshot, result) in enumerate(ready_run, start=3):
        assert result.status is DirectionalPredictionStatus.READY
        assert _directional_observation_safe_candidate(result)
        context = _directional_context(
            episode,
            tick_id=offset,
            simulation_time_s=slot.scheduled_delivery_at_s,
            snapshot=snapshot,
            prediction_result=result,
            gate=gate,
            authorize=authorize,
        )
        context = replace(
            context,
            path_still_valid=path_valid,
            local_safety_recheck_passed=local_recheck,
        )
        decision = gate.step(
            build_dynamic_command_proposal(
                context,
                command=Twist2D(),
                computation_time_s=0.001,
            ),
            robot_state=stopped,
            context=context,
        )

    assert decision.consecutive_safe_frames == 11
    assert decision.resume_allowed is expected_resume
    assert (decision.motion_state is DynamicMotionState.MOVING) is expected_resume
    assert decision.minimum_actor_clearance_m is not None
    assert decision.minimum_actor_clearance_m >= (
        VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m
    )
    assert "actor_clearance_below_minimum" not in decision.failure_reasons


@pytest.mark.parametrize(
    "episode",
    _same_direction_public_episodes(),
    ids=lambda episode: episode.latent_case_id,
)
def test_ready_capsules_cover_full_rollout_and_terminal_with_exact_geometry(
    episode: V6DynamicCorpusEpisode,
) -> None:
    prediction_set = _first_ready_prediction(episode)

    first = _exact_geometry_report(episode, prediction_set)
    second = _exact_geometry_report(episode, prediction_set)

    assert first == second
    assert first.candidate_count == 217
    assert first.rollout_pose_count_per_candidate == 41
    assert first.exact_capsule_clearance_count > 217 * 41
    assert first.legacy_witness_horizon_s == pytest.approx(0.35)
    assert first.maximum_verified_prediction_horizon_s > 2.0
    assert (
        first.maximum_verified_prediction_horizon_s
        > first.legacy_witness_horizon_s
    ), "the old 0.35 s rollout-zero witness does not cover rollout+terminal stopping"
    assert (
        first.prediction_status
        is DirectionalPublicQualificationStatus.PREDICTION_GEOMETRY_QUALIFIED
    )
    assert (
        first.online_dwb_status
        is DirectionalPublicQualificationStatus.ONLINE_DWB_BYPASS_UNPROVEN
    ), "exact capsule arithmetic is not proof of a closed-loop DWB bypass"
