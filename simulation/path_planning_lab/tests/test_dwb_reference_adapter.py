from __future__ import annotations

from dataclasses import dataclass, field, replace
from math import isclose

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
from hospital_path_lab.dynamic_directional_prediction import DirectionalActorPredictor
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    dynamic_observation_content_hash,
)
from hospital_path_lab.dynamic_prediction import build_actor_prediction_set
from hospital_path_lab.grid import GridMap
from hospital_path_lab.local_algorithms.dwb_reference.adapter import (
    SourceDerivedDwbController,
    source_derived_dwb_semantic_digest,
)
from hospital_path_lab.local_algorithms.dwb_reference.contracts import (
    DwbGeneratorRequest,
    DwbGeneratorResult,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)
from hospital_path_lab.local_algorithms.dwb_reference.core import (
    CandidateEvaluationDiagnostic,
    CandidateEvaluationStatus,
    CandidateFailureDiagnostic,
    CandidateFailureKind,
    DwbCoreResult,
    DwbPreparationError,
    NoLegalTrajectoryError,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1


def _snapshot(
    *,
    reference_path: tuple[Pose2D, ...] | None = None,
    robot_state: RobotState | None = None,
    mission_revision: int = 4,
    tracks: tuple[ActorTrack, ...] = (),
):
    frame = DynamicObservationFrame(
        stream_id="stream-v7",
        episode_id="episode-v7",
        episode_seed=7,
        map_id="map-v7",
        map_revision=3,
        observation_revision=5,
        sequence=5,
        observed_at_s=1.0,
        delivered_at_s=1.0,
        frame_kind=(
            DynamicObservationFrameKind.TRACKS
            if tracks
            else DynamicObservationFrameKind.EMPTY
        ),
        tracks=tracks,
        content_hash="observation-v7",
    )
    observation = DynamicObservationSnapshot(
        availability=DynamicObservationAvailability.FRESH,
        frame=frame,
        age_s=0.0,
        failures=(),
        last_event_was_no_frame=False,
    )
    grid = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="map-v7",
            map_revision=3,
            mission_revision=mission_revision,
            observation_revision=5,
            seed=7,
            content_hash="grid-v7",
        ),
        grid=GridMap(np.zeros((180, 180), dtype=np.bool_), resolution_m=0.02),
    )
    return build_controller_snapshot(
        tick_id=20,
        simulation_time_s=1.0,
        mission_id="mission-v7",
        robot_state=robot_state
        or RobotState(Pose2D(1.0, 1.0, 0.0), Twist2D(0.20, 0.0)),
        goal_pose=Pose2D(2.4, 1.0, 0.0),
        reference_path=reference_path
        or (Pose2D(1.0, 1.0, 0.0), Pose2D(2.4, 1.0, 0.0)),
        static_grid_snapshot=grid,
        validated_observation=observation,
        actor_tubes=build_actor_prediction_set(observation),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )


def _trajectory(request: DwbGeneratorRequest) -> DwbTrajectory:
    command = DwbTwist2D(0.20, 0.0)
    return DwbTrajectory(
        command=command,
        poses=tuple(
            DwbPose2D(
                request.pose.x_m + index * 0.01,
                request.pose.y_m,
                request.pose.yaw_rad,
            )
            for index in range(41)
        ),
        integration_step_s=0.05,
    )


def _directional_snapshot(*, empty: bool = False):
    source = DynamicObservationSourceIdentity(
        stream_id="stream-v7",
        episode_id="episode-v7",
        episode_seed=7,
        map_id="map-v7",
        map_revision=3,
    )
    validator = DynamicObservationValidator(source, NORMAL_OBSERVATION_PROFILE)
    predictor = DirectionalActorPredictor()
    result = None
    frame = None
    count = 1 if empty else 20
    for sequence in range(count):
        observed_at_s = sequence * 0.1
        tracks = () if empty else (
            ActorTrack(
                track_id="person-1",
                actor_binding_id="actor-1",
                observed_position=Point2D(2.0 + 0.20 * observed_at_s, 1.5),
                observed_velocity=Vector2D(0.20, 0.0),
                position_sigma_m=0.03,
                velocity_sigma_mps=0.05,
            ),
        )
        frame = DynamicObservationFrame(
            stream_id=source.stream_id,
            episode_id=source.episode_id,
            episode_seed=source.episode_seed,
            map_id=source.map_id,
            map_revision=source.map_revision,
            observation_revision=sequence,
            sequence=sequence,
            observed_at_s=observed_at_s,
            delivered_at_s=observed_at_s + NORMAL_OBSERVATION_PROFILE.latency_s,
            frame_kind=(
                DynamicObservationFrameKind.EMPTY
                if empty
                else DynamicObservationFrameKind.TRACKS
            ),
            tracks=tracks,
            content_hash="pending",
        )
        frame = replace(frame, content_hash=dynamic_observation_content_hash(frame))
        assert validator.accept(frame, received_at_s=frame.delivered_at_s).accepted
        result = predictor.update(
            validator.snapshot(control_time_s=frame.delivered_at_s)
        )

    assert result is not None and result.prediction_set is not None and frame is not None
    observation = validator.snapshot(control_time_s=frame.delivered_at_s)
    grid = GridSnapshot(
        metadata=SnapshotMetadata(
            map_id="map-v7",
            map_revision=3,
            mission_revision=4,
            observation_revision=frame.observation_revision,
            seed=7,
            content_hash="grid-v7-directional",
        ),
        grid=GridMap(np.zeros((180, 180), dtype=np.bool_), resolution_m=0.02),
    )
    snapshot = build_controller_snapshot(
        tick_id=40,
        simulation_time_s=frame.delivered_at_s,
        mission_id="mission-v7",
        robot_state=RobotState(Pose2D(1.0, 1.0, 0.0), Twist2D(0.20, 0.0)),
        goal_pose=Pose2D(2.4, 1.0, 0.0),
        reference_path=(Pose2D(1.0, 1.0, 0.0), Pose2D(2.4, 1.0, 0.0)),
        static_grid_snapshot=grid,
        validated_observation=observation,
        actor_tubes=result.prediction_set,
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )
    return snapshot


@dataclass
class FakeCore:
    mode: str = "success"
    set_path_calls: list[tuple[DwbPose2D, ...]] = field(default_factory=list)
    requests: list[DwbGeneratorRequest] = field(default_factory=list)

    def set_path(self, path) -> None:
        self.set_path_calls.append(tuple(path))

    def compute(self, request: DwbGeneratorRequest) -> DwbCoreResult:
        self.requests.append(request)
        if self.mode == "prepare_failure":
            raise DwbPreparationError("path_dist")

        trajectory = _trajectory(request)
        generator_result = DwbGeneratorResult(
            linear_window_mps=(0.175, 0.20),
            angular_window_radps=(-0.08, 0.08),
            linear_samples_mps=(0.20,),
            angular_samples_radps=(0.0,),
            trajectories=(trajectory,),
        )
        if self.mode == "no_legal":
            rejected = CandidateEvaluationDiagnostic(
                candidate_index=0,
                command=trajectory.command,
                status=CandidateEvaluationStatus.ILLEGAL,
                accumulated_score=0.0,
                critic_scores=(),
                failure=CandidateFailureDiagnostic(
                    kind=CandidateFailureKind.CRITIC_REJECTION,
                    critic_name="actor_constraint",
                    reason_code="actor_tube_collision",
                    message="blocked",
                ),
            )
            raise NoLegalTrajectoryError((rejected,))

        selected = CandidateEvaluationDiagnostic(
            candidate_index=0,
            command=trajectory.command,
            status=CandidateEvaluationStatus.LEGAL,
            accumulated_score=1.25,
            critic_scores=(),
        )
        return DwbCoreResult(
            command=trajectory.command,
            trajectory=trajectory,
            total_score=1.25,
            selected_candidate_index=0,
            generator_result=generator_result,
            candidate_evaluations=(selected,),
        )


@dataclass
class RecordingBinder:
    events: list[str]

    def bind_snapshot(self, snapshot) -> None:
        self.events.append(f"bind:{snapshot.tick_id}")


def _invalid_grid(snapshot):
    metadata = replace(snapshot.static_grid_snapshot.metadata, input_valid=False)
    return replace(
        snapshot,
        static_grid_snapshot=replace(snapshot.static_grid_snapshot, metadata=metadata),
    )


def _stale_observation(snapshot):
    return replace(
        snapshot,
        validated_observation=replace(
            snapshot.validated_observation,
            availability=DynamicObservationAvailability.STALE,
        ),
    )


def _wrong_actor_identity(snapshot):
    return replace(
        snapshot,
        actor_tubes=replace(snapshot.actor_tubes, stream_id="wrong-stream"),
    )


def _out_of_range_speed(snapshot):
    return replace(
        snapshot,
        robot_state=replace(snapshot.robot_state, twist=Twist2D(0.201, 0.0)),
    )


def _wrong_snapshot_hash(snapshot):
    return replace(snapshot, input_content_hash="wrong-input-hash")


@pytest.mark.parametrize(
    ("mutation", "reason"),
    [
        (_invalid_grid, "grid_snapshot_invalid"),
        (_stale_observation, "fresh_observation_required"),
        (_wrong_actor_identity, "actor_prediction_provenance_mismatch"),
        (_out_of_range_speed, "linear_state_outside_frozen_range"),
        (_wrong_snapshot_hash, "snapshot_content_hash_mismatch"),
    ],
)
def test_adapter_rejects_invalid_project_inputs_without_calling_core(
    mutation,
    reason: str,
) -> None:
    core = FakeCore()
    result = SourceDerivedDwbController(core=core).step(mutation(_snapshot()))

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == reason
    assert result.requested_twist == Twist2D()
    assert result.controller_requested_stop
    assert not core.requests


def test_adapter_rejects_malformed_actor_tubes_without_raising() -> None:
    core = FakeCore()
    malformed = replace(_snapshot(), actor_tubes=object())

    result = SourceDerivedDwbController(core=core).step(malformed)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "actor_prediction_content_mismatch"
    assert result.requested_twist == Twist2D()
    assert result.controller_requested_stop
    assert not core.requests


def test_adapter_rejects_issued_directional_prediction_after_no_frame_event() -> None:
    core = FakeCore()
    snapshot = _directional_snapshot()
    dropped = replace(
        snapshot,
        validated_observation=replace(
            snapshot.validated_observation,
            last_event_was_no_frame=True,
        ),
    )

    result = SourceDerivedDwbController(core=core).step(dropped)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "fresh_observation_required"
    assert result.requested_twist == Twist2D()
    assert result.controller_requested_stop
    assert not core.requests


@pytest.mark.parametrize(
    "tube_change",
    [
        {"observed_position": Point2D(2.25, 1.5)},
        {"capped_velocity": Vector2D(0.05, -0.10)},
        {"position_sigma_m": 0.08},
        {"velocity_sigma_mps": 0.15},
    ],
)
def test_adapter_rejects_actor_prediction_geometry_tampering(
    tube_change: dict[str, object],
) -> None:
    core = FakeCore()
    snapshot = _snapshot(
        tracks=(
            ActorTrack(
                track_id="person-1",
                actor_binding_id="actor-1",
                observed_position=Point2D(2.0, 1.5),
                observed_velocity=Vector2D(0.10, 0.0),
                position_sigma_m=0.03,
                velocity_sigma_mps=0.05,
            ),
        )
    )
    original_tube = snapshot.actor_tubes.tubes[0]
    tampered_tube = replace(original_tube, **tube_change)
    snapshot = replace(
        snapshot,
        actor_tubes=replace(snapshot.actor_tubes, tubes=(tampered_tube,)),
    )

    result = SourceDerivedDwbController(core=core).step(snapshot)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "actor_prediction_content_mismatch"
    assert result.requested_twist == Twist2D()
    assert result.controller_requested_stop
    assert not core.requests


@pytest.mark.parametrize("empty", [False, True])
def test_adapter_accepts_issued_directional_prediction_and_fresh_empty(
    empty: bool,
) -> None:
    core = FakeCore()
    snapshot = _directional_snapshot(empty=empty)

    result = SourceDerivedDwbController(core=core).step(snapshot)

    assert result.status is PlanStatus.FOUND
    assert len(core.requests) == 1


@pytest.mark.parametrize(
    "mutation",
    [
        lambda prediction: replace(prediction),
        lambda prediction: replace(
            prediction,
            tubes=(
                replace(
                    prediction.tubes[0],
                    anchor_position=Point2D(9.0, 9.0),
                ),
            ),
        ),
        lambda prediction: replace(
            prediction,
            history_content_hash="forged-history",
        ),
        lambda prediction: replace(
            prediction,
            parameter_content_hash="forged-parameters",
        ),
    ],
)
def test_adapter_rejects_unissued_or_tampered_directional_prediction(
    mutation,
) -> None:
    core = FakeCore()
    snapshot = _directional_snapshot()
    changed = replace(snapshot, actor_tubes=mutation(snapshot.actor_tubes))

    result = SourceDerivedDwbController(core=core).step(changed)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "actor_prediction_content_mismatch"
    assert result.controller_requested_stop
    assert not core.requests


def test_adapter_rejects_in_place_directional_geometry_tampering() -> None:
    core = FakeCore()
    snapshot = _directional_snapshot()
    prediction = snapshot.actor_tubes
    tube = prediction.tubes[0]
    original = tube.anchor_position
    object.__setattr__(tube, "anchor_position", Point2D(9.0, 9.0))
    try:
        result = SourceDerivedDwbController(core=core).step(snapshot)
    finally:
        object.__setattr__(tube, "anchor_position", original)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "actor_prediction_content_mismatch"
    assert result.controller_requested_stop
    assert not core.requests


def test_compute_prepared_revalidates_directional_issuance_capability() -> None:
    core = FakeCore()
    controller = SourceDerivedDwbController(core=core)
    prepared = _directional_snapshot()
    assert controller.prepare_snapshot(prepared) is None
    unissued_equal_copy = replace(
        prepared,
        actor_tubes=replace(prepared.actor_tubes),
    )

    result = controller.compute_prepared(unissued_equal_copy)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "actor_prediction_content_mismatch"
    assert result.controller_requested_stop
    assert not core.requests
    with pytest.raises(ValueError, match="must be prepared"):
        controller.compute_prepared(prepared)


def test_adapter_uses_post_apply_pose_and_converts_41_pose_rollout() -> None:
    core = FakeCore()
    snapshot = _snapshot()

    result = SourceDerivedDwbController(core=core).step(snapshot)

    assert result.status is PlanStatus.FOUND
    assert len(core.requests) == 1
    assert isclose(core.requests[0].pose.x_m, 1.01, abs_tol=1e-12)
    assert core.requests[0].pose.y_m == 1.0
    assert core.requests[0].current_twist == DwbTwist2D(0.20, 0.0)
    assert len(result.predicted_trajectory) == 41
    assert result.predicted_trajectory[0].time_s == 0.0
    assert result.predicted_trajectory[0].pose == Pose2D(1.01, 1.0, 0.0)
    assert result.predicted_trajectory[-1].time_s == 2.0


def test_adapter_reuses_last_fresh_frame_during_single_dropout_until_ttl() -> None:
    core = FakeCore()
    snapshot = _snapshot()
    snapshot = replace(
        snapshot,
        validated_observation=replace(
            snapshot.validated_observation,
            last_event_was_no_frame=True,
        ),
    )

    result = SourceDerivedDwbController(core=core).step(snapshot)

    assert result.status is PlanStatus.FOUND
    assert len(core.requests) == 1


def test_adapter_post_apply_rotation_uses_shared_gate_exact_arc_geometry() -> None:
    core = FakeCore()
    snapshot = _snapshot(
        robot_state=RobotState(Pose2D(1.0, 1.0, 0.0), Twist2D(0.20, 0.40))
    )

    result = SourceDerivedDwbController(core=core).step(snapshot)

    request_pose = core.requests[0].pose
    assert result.status is PlanStatus.FOUND
    assert request_pose.x_m == pytest.approx(1.0099993333466666)
    assert request_pose.y_m == pytest.approx(1.000099996666711)
    assert request_pose.yaw_rad == pytest.approx(0.02)
    assert result.predicted_trajectory[0].pose == Pose2D(
        request_pose.x_m,
        request_pose.y_m,
        request_pose.yaw_rad,
    )


def test_adapter_preserves_every_result_provenance_field() -> None:
    snapshot = _snapshot()

    result = SourceDerivedDwbController(core=FakeCore()).step(snapshot)

    assert result.controller_name == "dynamic_dwb_reference"
    assert result.source_tick_id == snapshot.tick_id
    assert result.mission_id == snapshot.mission_id
    assert result.map_id == snapshot.map_id
    assert result.map_revision == snapshot.map_revision
    assert result.mission_revision == snapshot.mission_revision
    assert result.observation_revision == snapshot.observation_revision
    assert result.grid_content_hash == snapshot.static_grid_snapshot.metadata.content_hash
    assert result.observation_content_hash == snapshot.observation_content_hash
    assert result.input_content_hash == snapshot.input_content_hash


def test_adapter_maps_no_legal_trajectory_to_fail_closed_no_path() -> None:
    result = SourceDerivedDwbController(core=FakeCore(mode="no_legal")).step(_snapshot())

    assert result.status is PlanStatus.NO_PATH
    assert result.failure_reason == "no_legal_dwb_trajectory"
    assert result.controller_requested_stop
    assert result.no_safe_candidate
    assert "rejection.actor_tube_collision=1" in result.decision_trace


def test_adapter_maps_critic_preparation_failure_to_invalid_input() -> None:
    result = SourceDerivedDwbController(core=FakeCore(mode="prepare_failure")).step(
        _snapshot()
    )

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "dwb_critic_preparation_failed"
    assert result.controller_requested_stop
    assert "critic=path_dist" in result.decision_trace


def test_repeat_result_is_identical_when_elapsed_is_excluded() -> None:
    core = FakeCore()
    controller = SourceDerivedDwbController(core=core)
    snapshot = _snapshot()

    first = controller.step(snapshot)
    second = controller.step(snapshot)

    assert replace(first, elapsed_ns=0) == replace(second, elapsed_ns=0)
    assert source_derived_dwb_semantic_digest(first) == source_derived_dwb_semantic_digest(
        second
    )
    assert len(core.set_path_calls) == 1


def test_core_path_is_reset_only_when_reference_signature_changes() -> None:
    core = FakeCore()
    controller = SourceDerivedDwbController(core=core)
    first = _snapshot()
    changed = _snapshot(
        reference_path=(
            Pose2D(1.0, 1.0, 0.0),
            Pose2D(1.8, 1.2, 0.1),
            Pose2D(2.4, 1.0, 0.0),
        )
    )

    controller.step(first)
    controller.step(first)
    controller.step(changed)
    controller.step(changed)

    assert len(core.set_path_calls) == 2
    assert core.set_path_calls[0] != core.set_path_calls[1]


def test_new_mission_revision_resets_path_bound_critic_state() -> None:
    core = FakeCore()
    controller = SourceDerivedDwbController(core=core)

    controller.step(_snapshot(mission_revision=4))
    controller.step(_snapshot(mission_revision=5))

    assert len(core.set_path_calls) == 2


def test_snapshot_extension_is_rebound_after_path_install_on_every_tick() -> None:
    events: list[str] = []

    @dataclass
    class OrderedCore(FakeCore):
        def set_path(self, path) -> None:
            events.append("set_path")
            super().set_path(path)

        def compute(self, request: DwbGeneratorRequest) -> DwbCoreResult:
            events.append("compute")
            return super().compute(request)

    controller = SourceDerivedDwbController(
        core=OrderedCore(),
        snapshot_binders=(RecordingBinder(events),),
    )
    snapshot = _snapshot()

    controller.step(snapshot)
    controller.step(snapshot)

    assert events == ["set_path", "bind:20", "compute", "bind:20", "compute"]


def test_prepare_snapshot_binds_without_computing_or_debriefing() -> None:
    events: list[str] = []

    @dataclass
    class OrderedCore(FakeCore):
        def set_path(self, path) -> None:
            events.append("set_path")
            super().set_path(path)

        def compute(self, request: DwbGeneratorRequest) -> DwbCoreResult:
            events.append("compute")
            return super().compute(request)

    core = OrderedCore()
    controller = SourceDerivedDwbController(
        core=core,
        snapshot_binders=(RecordingBinder(events),),
    )

    failure = controller.prepare_snapshot(_snapshot())

    assert failure is None
    assert events == ["set_path", "bind:20"]
    assert not core.requests

    result = controller.compute_prepared(_snapshot())
    assert result.status is PlanStatus.FOUND
    assert events == ["set_path", "bind:20", "compute"]


def test_compute_prepared_rejects_unprepared_and_fail_closes_different_tick() -> None:
    controller = SourceDerivedDwbController(core=FakeCore())

    with pytest.raises(ValueError, match="must be prepared"):
        controller.compute_prepared(_snapshot())

    controller.prepare_snapshot(_snapshot())
    result = controller.compute_prepared(replace(_snapshot(), tick_id=21))

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "prepared_snapshot_semantic_mismatch"
    assert result.requested_twist == Twist2D()
    assert result.controller_requested_stop


def _change_prepared_goal(snapshot):
    return replace(snapshot, goal_pose=Pose2D(2.5, 1.0, 0.2))


def _change_prepared_path(snapshot):
    return replace(
        snapshot,
        reference_path=(
            Pose2D(1.0, 1.0, 0.0),
            Pose2D(1.8, 1.3, 0.2),
            Pose2D(2.4, 1.0, 0.0),
        ),
    )


def _change_prepared_robot_state(snapshot):
    return replace(
        snapshot,
        robot_state=RobotState(Pose2D(1.05, 1.0, 0.1), Twist2D(0.15, 0.2)),
    )


def _change_prepared_actor_tube(snapshot):
    tube = snapshot.actor_tubes.tubes[0]
    changed_tube = replace(tube, observed_position=Point2D(2.1, 1.5))
    return replace(
        snapshot,
        actor_tubes=replace(snapshot.actor_tubes, tubes=(changed_tube,)),
    )


@pytest.mark.parametrize(
    "semantic_change",
    [
        _change_prepared_goal,
        _change_prepared_path,
        _change_prepared_robot_state,
        _change_prepared_actor_tube,
    ],
)
def test_compute_prepared_fail_closes_if_same_hash_semantics_change(
    semantic_change,
) -> None:
    core = FakeCore()
    controller = SourceDerivedDwbController(core=core)
    prepared = _snapshot(
        tracks=(
            ActorTrack(
                track_id="person-1",
                actor_binding_id="actor-1",
                observed_position=Point2D(2.0, 1.5),
                observed_velocity=Vector2D(0.10, 0.0),
                position_sigma_m=0.03,
                velocity_sigma_mps=0.05,
            ),
        )
    )
    assert controller.prepare_snapshot(prepared) is None
    changed = semantic_change(prepared)

    assert changed.tick_id == prepared.tick_id
    assert changed.input_content_hash == prepared.input_content_hash

    result = controller.compute_prepared(changed)

    assert result.status is PlanStatus.INVALID_INPUT
    assert result.failure_reason == "prepared_snapshot_semantic_mismatch"
    assert result.requested_twist == Twist2D()
    assert result.controller_requested_stop
    assert not core.requests

    # A mismatch consumes the prepared token; previously bound state cannot be
    # recovered and executed after the caller has mixed snapshot semantics.
    with pytest.raises(ValueError, match="must be prepared"):
        controller.compute_prepared(prepared)
