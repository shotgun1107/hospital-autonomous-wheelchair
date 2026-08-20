"""Conversions from backend-friendly runtime DTOs to existing R7 contracts."""

from __future__ import annotations

from dataclasses import replace
from math import isclose

import numpy as np

from hospital_path_lab.contracts import (
    GridSnapshot,
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
    ResumeAuthorization,
    Vector2D,
)
from hospital_path_lab.dynamic_observation import (
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationProfile,
    DynamicObservationProfileName,
    DynamicObservationSourceIdentity,
    dynamic_observation_content_hash,
)
from hospital_path_lab.grid import GridMap
from hospital_path_lab.spatial_oracle_contracts import spatial_grid_content_hash

from .contracts import (
    RuntimeActorObservation,
    RuntimeMission,
    RuntimeObservation,
    RuntimePose,
    RuntimeResumeAuthorization,
    RuntimeRobotState,
)

_TIME_TOLERANCE_S = 1e-12


class RuntimeAdapterError(ValueError):
    """A caller-visible DTO-to-R7 conversion rejection."""


def observation_profile_for(name: DynamicObservationProfileName) -> DynamicObservationProfile:
    """Return only an existing frozen R7 observation profile."""

    profiles = {
        DynamicObservationProfileName.NORMAL: NORMAL_OBSERVATION_PROFILE,
        DynamicObservationProfileName.STRESS: STRESS_OBSERVATION_PROFILE,
        DynamicObservationProfileName.FUNCTIONAL_IDEAL: FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    }
    try:
        return profiles[name]
    except KeyError as error:
        raise RuntimeAdapterError(f"runtime_observation_profile_unsupported:{name}") from error


def to_pose(value: RuntimePose) -> Pose2D:
    if not isinstance(value, RuntimePose):
        raise TypeError("value must be a RuntimePose")
    return Pose2D(value.x_m, value.y_m, value.yaw_rad)


def to_robot_state(value: RuntimeRobotState) -> RobotState:
    if not isinstance(value, RuntimeRobotState):
        raise TypeError("value must be a RuntimeRobotState")
    return RobotState(
        pose=to_pose(value.pose),
        twist=Twist2D(value.linear_mps, value.angular_radps),
    )


def build_runtime_grid_snapshot(mission: RuntimeMission) -> GridSnapshot:
    """Create the immutable spatial source used for an entire runtime mission."""

    if not isinstance(mission, RuntimeMission):
        raise TypeError("mission must be a RuntimeMission")
    runtime_map = mission.runtime_map
    grid = GridMap(
        occupancy=np.asarray(runtime_map.occupancy_rows, dtype=np.bool_),
        resolution_m=runtime_map.resolution_m,
        origin_x_m=runtime_map.origin_x_m,
        origin_y_m=runtime_map.origin_y_m,
    )
    grid_hash = spatial_grid_content_hash(grid)
    metadata = SnapshotMetadata(
        map_id=runtime_map.map_id,
        map_revision=runtime_map.map_revision,
        mission_revision=mission.mission_revision,
        observation_revision=0,
        seed=mission.observation_session_seed,
        content_hash=grid_hash,
    )
    return GridSnapshot(
        metadata=metadata,
        grid=grid,
        forbidden_cells=frozenset(runtime_map.forbidden_cells),
    )


def grid_snapshot_for_observation(
    source: GridSnapshot,
    *,
    observation_revision: int,
) -> GridSnapshot:
    """Preserve spatial identity while binding one observation revision to a tick."""

    if not isinstance(source, GridSnapshot):
        raise TypeError("source must be a GridSnapshot")
    if isinstance(observation_revision, bool) or not isinstance(observation_revision, int):
        raise RuntimeAdapterError("observation_revision_invalid")
    if observation_revision < 0:
        raise RuntimeAdapterError("observation_revision_invalid")
    return GridSnapshot(
        metadata=replace(source.metadata, observation_revision=observation_revision),
        grid=source.grid,
        forbidden_cells=source.forbidden_cells,
    )


def build_observation_source(mission: RuntimeMission) -> DynamicObservationSourceIdentity:
    if not isinstance(mission, RuntimeMission):
        raise TypeError("mission must be a RuntimeMission")
    return DynamicObservationSourceIdentity(
        stream_id=mission.observation_stream_id,
        episode_id=mission.mission_id,
        episode_seed=mission.observation_session_seed,
        map_id=mission.runtime_map.map_id,
        map_revision=mission.runtime_map.map_revision,
    )


def to_observation_frame(
    value: RuntimeObservation,
    *,
    source: DynamicObservationSourceIdentity,
    profile: DynamicObservationProfile,
) -> DynamicObservationFrame:
    """Build a canonical R7 observation frame from one processed camera result."""

    if not isinstance(value, RuntimeObservation):
        raise TypeError("value must be a RuntimeObservation")
    if not isinstance(source, DynamicObservationSourceIdentity):
        raise TypeError("source must be a DynamicObservationSourceIdentity")
    expected_observed_at_s = value.sequence * profile.observation_period_s
    if not isclose(
        value.observed_at_s,
        expected_observed_at_s,
        rel_tol=0.0,
        abs_tol=_TIME_TOLERANCE_S,
    ):
        raise RuntimeAdapterError("observation_timestamp_not_aligned_to_sequence")
    map_id = source.map_id if value.map_id is None else value.map_id
    map_revision = source.map_revision if value.map_revision is None else value.map_revision
    tracks = tuple(_to_actor_track(actor, profile=profile) for actor in value.actors)
    payload = {
        "stream_id": source.stream_id,
        "episode_id": source.episode_id,
        "episode_seed": source.episode_seed,
        "map_id": map_id,
        "map_revision": map_revision,
        "observation_revision": value.observation_revision,
        "sequence": value.sequence,
        "observed_at_s": value.observed_at_s,
        "delivered_at_s": value.observed_at_s + profile.latency_s,
        "frame_kind": (
            DynamicObservationFrameKind.TRACKS if tracks else DynamicObservationFrameKind.EMPTY
        ),
        "tracks": tracks,
    }
    provisional = DynamicObservationFrame(**payload, content_hash="runtime-pending-hash")
    return replace(provisional, content_hash=dynamic_observation_content_hash(provisional))


def to_resume_authorization(value: RuntimeResumeAuthorization | None) -> ResumeAuthorization | None:
    """Pass through a backend-issued authorization without minting a new one."""

    if value is None:
        return None
    if not isinstance(value, RuntimeResumeAuthorization):
        raise TypeError("value must be a RuntimeResumeAuthorization or None")
    return ResumeAuthorization(
        mission_id=value.mission_id,
        stop_epoch=value.stop_epoch,
        issued_or_revalidated_at_s=value.issued_or_revalidated_at_s,
        authorization_revision=value.authorization_revision,
        content_hash=value.content_hash,
    )


def _to_actor_track(
    value: RuntimeActorObservation,
    *,
    profile: DynamicObservationProfile,
) -> ActorTrack:
    return ActorTrack(
        track_id=value.track_id,
        actor_binding_id=value.actor_binding_id,
        observed_position=Point2D(value.x_m, value.y_m),
        observed_velocity=Vector2D(value.vx_mps, value.vy_mps),
        position_sigma_m=profile.position_sigma_m,
        velocity_sigma_mps=profile.velocity_sigma_mps,
    )
