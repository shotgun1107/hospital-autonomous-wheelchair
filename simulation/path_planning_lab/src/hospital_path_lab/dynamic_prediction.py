"""검증된 Actor 관측으로부터 controller 비종속 예측 tube를 만든다.

이 모듈은 PP, DWA 또는 safety gate를 import하지 않는다. 세 소비자는 동일한
``ActorPredictionSet``과 ``sample_actor_tubes`` API를 사용해야 한다.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot, isclose, isfinite

from hospital_path_lab.dynamic_contracts import (
    ACTOR_RADIUS_M,
    DYNAMIC_COMMAND_APPLY_LATENCY_S,
    MAX_ACTOR_ACCELERATION_MPS2,
    MAX_ACTOR_SPEED_MPS,
    ActorTrack,
    DynamicObservationFrameKind,
    Point2D,
    Vector2D,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationSnapshot,
    dynamic_observation_age_is_stale,
)

_TIME_ABS_TOLERANCE_S = 1e-12


@dataclass(frozen=True, slots=True)
class ActorPredictionTube:
    """한 accepted track에서 동결한 time-indexed tube의 계산 기준."""

    track_id: str
    actor_binding_id: str
    observed_position: Point2D
    capped_velocity: Vector2D
    position_sigma_m: float
    velocity_sigma_mps: float

    def __post_init__(self) -> None:
        if not self.track_id or not self.actor_binding_id:
            raise ValueError("prediction tube identity fields must not be empty")
        _require_finite(
            "prediction tube uncertainty",
            self.position_sigma_m,
            self.velocity_sigma_mps,
        )
        if min(self.position_sigma_m, self.velocity_sigma_mps) < 0.0:
            raise ValueError("prediction tube uncertainty must not be negative")
        if self.capped_velocity.magnitude > MAX_ACTOR_SPEED_MPS + 1e-12:
            raise ValueError("prediction tube velocity exceeds the Actor speed cap")


@dataclass(frozen=True, slots=True)
class ActorPredictionSet:
    """한 controller snapshot에서 PP·DWA·gate가 공유할 immutable tube 집합."""

    stream_id: str
    episode_id: str
    map_id: str
    map_revision: int
    observation_revision: int
    sequence: int
    source_content_hash: str
    observed_at_s: float
    controller_time_s: float
    snapshot_age_s: float
    tubes: tuple[ActorPredictionTube, ...]

    def __post_init__(self) -> None:
        if not self.stream_id or not self.episode_id or not self.map_id:
            raise ValueError("prediction set identity fields must not be empty")
        if not self.source_content_hash:
            raise ValueError("prediction set source_content_hash must not be empty")
        if min(self.map_revision, self.observation_revision, self.sequence) < 0:
            raise ValueError("prediction set revisions and sequence must not be negative")
        _require_finite(
            "prediction set time",
            self.observed_at_s,
            self.controller_time_s,
            self.snapshot_age_s,
        )
        if min(self.observed_at_s, self.controller_time_s, self.snapshot_age_s) < 0.0:
            raise ValueError("prediction set times must not be negative")
        derived_age_s = self.controller_time_s - self.observed_at_s
        if derived_age_s < -_TIME_ABS_TOLERANCE_S:
            raise ValueError("prediction set observation must not be in the future")
        if not isclose(
            self.snapshot_age_s,
            max(0.0, derived_age_s),
            rel_tol=0.0,
            abs_tol=_TIME_ABS_TOLERANCE_S,
        ):
            raise ValueError("prediction set snapshot_age_s must match its timestamps")
        if dynamic_observation_age_is_stale(self.snapshot_age_s):
            raise ValueError("prediction set observation must not be stale")
        object.__setattr__(self, "tubes", tuple(self.tubes))


@dataclass(frozen=True, slots=True)
class ActorTubeCircle:
    """특정 post-apply rollout 시각의 원형 Actor tube sample."""

    track_id: str
    actor_binding_id: str
    rollout_time_s: float
    prediction_horizon_s: float
    center: Point2D
    radius_m: float
    position_sigma_m: float
    acceleration_bound_m: float

    def __post_init__(self) -> None:
        if not self.track_id or not self.actor_binding_id:
            raise ValueError("Actor tube sample identity fields must not be empty")
        _require_finite(
            "Actor tube sample",
            self.rollout_time_s,
            self.prediction_horizon_s,
            self.radius_m,
            self.position_sigma_m,
            self.acceleration_bound_m,
        )
        if min(
            self.rollout_time_s,
            self.prediction_horizon_s,
            self.position_sigma_m,
            self.acceleration_bound_m,
        ) < 0.0:
            raise ValueError("Actor tube sample values must not be negative")
        if self.radius_m < ACTOR_RADIUS_M:
            raise ValueError("Actor tube radius must include the Actor radius")


def build_actor_prediction_set(
    snapshot: DynamicObservationSnapshot,
) -> ActorPredictionSet:
    """검증 완료된 fresh observation snapshot을 prediction input으로 동결한다.

    Raw frame은 받지 않는다. source·revision·hash·TTL을 검증한
    ``DynamicObservationValidator.snapshot`` 결과만 이 API를 통과할 수 있다.
    """

    if not isinstance(snapshot, DynamicObservationSnapshot):
        raise TypeError("prediction input must be a validated observation snapshot")
    if not snapshot.usable or snapshot.frame is None or snapshot.age_s is None:
        raise ValueError("prediction input must be a fresh validated observation snapshot")
    if snapshot.failures:
        raise ValueError("fresh prediction input must not contain validation failures")

    frame = snapshot.frame
    snapshot_age_s = _normalized_nonnegative_duration(snapshot.age_s, label="snapshot age")
    if dynamic_observation_age_is_stale(snapshot_age_s):
        raise ValueError("observation is stale")
    controller_time_s = frame.observed_at_s + snapshot_age_s
    _require_finite("controller time", controller_time_s)

    if frame.frame_kind is DynamicObservationFrameKind.EMPTY and frame.tracks:
        raise ValueError("empty observation frame must not contain tracks")
    if frame.frame_kind is DynamicObservationFrameKind.TRACKS and not frame.tracks:
        raise ValueError("tracks observation frame must contain at least one track")

    tubes = tuple(_prediction_tube(track) for track in frame.tracks)
    return ActorPredictionSet(
        stream_id=frame.stream_id,
        episode_id=frame.episode_id,
        map_id=frame.map_id,
        map_revision=frame.map_revision,
        observation_revision=frame.observation_revision,
        sequence=frame.sequence,
        source_content_hash=frame.content_hash,
        observed_at_s=frame.observed_at_s,
        controller_time_s=controller_time_s,
        snapshot_age_s=snapshot_age_s,
        tubes=tubes,
    )


def sample_actor_tubes(
    prediction_set: ActorPredictionSet,
    *,
    rollout_time_s: float,
) -> tuple[ActorTubeCircle, ...]:
    """고정 v5 수식으로 모든 Actor tube를 같은 rollout 시각에 sampling한다."""

    _require_finite("rollout time", rollout_time_s)
    if rollout_time_s < 0.0:
        raise ValueError("rollout_time_s must not be negative")
    prediction_horizon_s = (
        prediction_set.snapshot_age_s
        + DYNAMIC_COMMAND_APPLY_LATENCY_S
        + rollout_time_s
    )
    if not isfinite(prediction_horizon_s):
        raise ValueError("prediction horizon must be finite")

    return tuple(
        _sample_tube(tube, rollout_time_s, prediction_horizon_s)
        for tube in prediction_set.tubes
    )


def _prediction_tube(track: ActorTrack) -> ActorPredictionTube:
    velocity_x = track.observed_velocity.x
    velocity_y = track.observed_velocity.y
    largest_component = max(abs(velocity_x), abs(velocity_y))
    if largest_component == 0.0:
        capped_velocity = track.observed_velocity
    else:
        scaled_x = velocity_x / largest_component
        scaled_y = velocity_y / largest_component
        scaled_magnitude = hypot(scaled_x, scaled_y)
        speed_exceeds_cap = largest_component > MAX_ACTOR_SPEED_MPS / scaled_magnitude
        if speed_exceeds_cap:
            capped_velocity = Vector2D(
                MAX_ACTOR_SPEED_MPS * scaled_x / scaled_magnitude,
                MAX_ACTOR_SPEED_MPS * scaled_y / scaled_magnitude,
            )
        else:
            capped_velocity = track.observed_velocity
    return ActorPredictionTube(
        track_id=track.track_id,
        actor_binding_id=track.actor_binding_id,
        observed_position=track.observed_position,
        capped_velocity=capped_velocity,
        position_sigma_m=track.position_sigma_m,
        velocity_sigma_mps=track.velocity_sigma_mps,
    )


def _sample_tube(
    tube: ActorPredictionTube,
    rollout_time_s: float,
    prediction_horizon_s: float,
) -> ActorTubeCircle:
    center_x = (
        tube.observed_position.x + tube.capped_velocity.x * prediction_horizon_s
    )
    center_y = (
        tube.observed_position.y + tube.capped_velocity.y * prediction_horizon_s
    )
    propagated_velocity_sigma = prediction_horizon_s * tube.velocity_sigma_mps
    position_sigma_m = hypot(tube.position_sigma_m, propagated_velocity_sigma)

    velocity_delta_cap_mps = MAX_ACTOR_SPEED_MPS + tube.capped_velocity.magnitude
    velocity_delta_time_s = velocity_delta_cap_mps / MAX_ACTOR_ACCELERATION_MPS2
    if prediction_horizon_s <= velocity_delta_time_s:
        acceleration_bound_m = (
            0.5 * MAX_ACTOR_ACCELERATION_MPS2 * prediction_horizon_s**2
        )
    else:
        acceleration_bound_m = (
            0.5 * MAX_ACTOR_ACCELERATION_MPS2 * velocity_delta_time_s**2
            + velocity_delta_cap_mps
            * (prediction_horizon_s - velocity_delta_time_s)
        )
    radius_m = ACTOR_RADIUS_M + 2.0 * position_sigma_m + acceleration_bound_m

    if not all(
        isfinite(value)
        for value in (
            center_x,
            center_y,
            position_sigma_m,
            acceleration_bound_m,
            radius_m,
        )
    ):
        raise ValueError("Actor prediction produced a non-finite output")

    return ActorTubeCircle(
        track_id=tube.track_id,
        actor_binding_id=tube.actor_binding_id,
        rollout_time_s=rollout_time_s,
        prediction_horizon_s=prediction_horizon_s,
        center=Point2D(center_x, center_y),
        radius_m=radius_m,
        position_sigma_m=position_sigma_m,
        acceleration_bound_m=acceleration_bound_m,
    )


def _normalized_nonnegative_duration(value: float, *, label: str) -> float:
    _require_finite(label, value)
    if value < -_TIME_ABS_TOLERANCE_S:
        raise ValueError(f"{label} must not be negative")
    return 0.0 if value < 0.0 else value


def _require_finite(label: str, *values: float) -> None:
    if not all(isfinite(value) for value in values):
        raise ValueError(f"{label} values must be finite")
