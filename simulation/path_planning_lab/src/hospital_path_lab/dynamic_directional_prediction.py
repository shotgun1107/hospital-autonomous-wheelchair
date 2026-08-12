"""Directional, history-derived Actor prediction for the v7 research lane.

This module is deliberately separate from :mod:`dynamic_prediction`.  It models
the current v6/v7 public corpus only: an open-loop circular Actor keeps a constant
heading while its longitudinal speed may change within frozen bounds.  It is a
simulation research contract, not a general pedestrian reachable-set claim.

The predictor consumes only validated controller-facing observations.  Until a
direction is supported by twenty unique accepted TRACKS frames it returns an
explicit hold result instead of silently falling back to an unsafe prediction.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from enum import StrEnum
from hashlib import sha256
from json import dumps
from math import ceil, hypot, isclose, isfinite, sqrt
from threading import RLock
from weakref import ref

from hospital_path_lab.dynamic_contracts import (
    ACTOR_RADIUS_M,
    DYNAMIC_COMMAND_APPLY_LATENCY_S,
    MAX_ACTOR_ACCELERATION_MPS2,
    MAX_ACTOR_SPEED_MPS,
    ActorTrack,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    Point2D,
    Vector2D,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationSnapshot,
    dynamic_observation_content_hash,
)
from hospital_path_lab.dynamic_prediction import ActorTubeCircle

DIRECTIONAL_PREDICTION_VERSION = "directional_constant_heading_actor_v7"
_TIME_TOLERANCE_S = 1e-12
_GEOMETRY_TOLERANCE_M = 1e-12
_ISSUED_PREDICTIONS_LOCK = RLock()
_ISSUED_PREDICTIONS: dict[
    int,
    tuple[ref[DirectionalPredictionSet], str],
] = {}


@dataclass(frozen=True, slots=True)
class DirectionalPredictionParameters:
    """Frozen v7 parameters for a constant-heading, forward-only Actor."""

    model_version: str = DIRECTIONAL_PREDICTION_VERSION
    history_frame_count: int = 20
    minimum_history_span_s: float = 1.9
    sigma_multiplier: float = 2.0
    minimum_directional_speed_mps: float = 0.03
    maximum_speed_mps: float = MAX_ACTOR_SPEED_MPS
    maximum_longitudinal_acceleration_mps2: float = MAX_ACTOR_ACCELERATION_MPS2
    maximum_longitudinal_deceleration_mps2: float = MAX_ACTOR_ACCELERATION_MPS2
    lateral_turn_bound_m: float = 0.0
    maximum_fit_rms_sigma_multiplier: float = 3.0
    maximum_circle_spacing_m: float = 0.10
    command_apply_latency_s: float = DYNAMIC_COMMAND_APPLY_LATENCY_S

    def __post_init__(self) -> None:
        if self.model_version != DIRECTIONAL_PREDICTION_VERSION:
            raise ValueError("directional prediction model_version is frozen")
        if self.history_frame_count != 20:
            raise ValueError("directional prediction requires exactly 20 frames")
        values = (
            self.minimum_history_span_s,
            self.sigma_multiplier,
            self.minimum_directional_speed_mps,
            self.maximum_speed_mps,
            self.maximum_longitudinal_acceleration_mps2,
            self.maximum_longitudinal_deceleration_mps2,
            self.lateral_turn_bound_m,
            self.maximum_fit_rms_sigma_multiplier,
            self.maximum_circle_spacing_m,
            self.command_apply_latency_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("directional prediction parameters must be finite")
        if min(values) < 0.0:
            raise ValueError("directional prediction parameters must not be negative")
        if self.minimum_history_span_s <= 0.0:
            raise ValueError("minimum_history_span_s must be positive")
        if self.sigma_multiplier != 2.0:
            raise ValueError("the v7 research contract keeps the 2-sigma heuristic")
        if self.maximum_speed_mps <= 0.0:
            raise ValueError("maximum_speed_mps must be positive")
        if min(
            self.maximum_longitudinal_acceleration_mps2,
            self.maximum_longitudinal_deceleration_mps2,
            self.maximum_circle_spacing_m,
        ) <= 0.0:
            raise ValueError("motion and discretization limits must be positive")
        if self.lateral_turn_bound_m != 0.0:
            raise ValueError("the current open-loop corpus freezes lateral turn to zero")


FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS = DirectionalPredictionParameters()


class DirectionalPredictionStatus(StrEnum):
    READY = "ready"
    WARMING_UP = "warming_up"
    LOW_SPEED = "low_speed"
    LOW_CONFIDENCE = "low_confidence"
    EMPTY_FRAME = "empty_frame"
    DROPOUT = "dropout"
    STALE = "stale"
    INVALID = "invalid"
    UNAVAILABLE = "unavailable"
    ORDER_VIOLATION = "order_violation"


@dataclass(frozen=True, slots=True)
class DirectionalPredictionTube:
    """One constant-heading Actor estimate derived from causal history."""

    track_id: str
    actor_binding_id: str
    anchor_position: Point2D
    heading_unit: Vector2D
    estimated_speed_mps: float
    position_sigma_m: float
    velocity_sigma_mps: float
    history_count: int
    history_span_s: float
    fit_rms_m: float
    history_content_hash: str

    def __post_init__(self) -> None:
        if not self.track_id or not self.actor_binding_id or not self.history_content_hash:
            raise ValueError("directional tube identity fields must not be empty")
        values = (
            self.estimated_speed_mps,
            self.position_sigma_m,
            self.velocity_sigma_mps,
            self.history_span_s,
            self.fit_rms_m,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("directional tube values must be finite")
        if min(values) < 0.0:
            raise ValueError("directional tube values must not be negative")
        if self.history_count != 20:
            raise ValueError("a directional tube requires 20 observations")
        if self.estimated_speed_mps > MAX_ACTOR_SPEED_MPS + 1e-12:
            raise ValueError("directional tube exceeds the Actor speed cap")
        if not isclose(
            self.heading_unit.magnitude,
            1.0,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("directional tube heading must be a unit vector")


@dataclass(frozen=True, slots=True, weakref_slot=True)
class DirectionalPredictionSet:
    """Immutable controller-facing set with observation provenance."""

    model_version: str
    stream_id: str
    episode_id: str
    episode_seed: int
    map_id: str
    map_revision: int
    observation_revision: int
    sequence: int
    source_content_hash: str
    observed_at_s: float
    controller_time_s: float
    snapshot_age_s: float
    tubes: tuple[DirectionalPredictionTube, ...]
    parameter_content_hash: str
    history_content_hash: str
    content_hash: str

    def __post_init__(self) -> None:
        if self.model_version != DIRECTIONAL_PREDICTION_VERSION:
            raise ValueError("directional prediction set version mismatch")
        if not all(
            (
                self.stream_id,
                self.episode_id,
                self.map_id,
                self.source_content_hash,
                self.parameter_content_hash,
                self.history_content_hash,
                self.content_hash,
            )
        ):
            raise ValueError("directional prediction identity fields must not be empty")
        if min(
            self.episode_seed,
            self.map_revision,
            self.observation_revision,
            self.sequence,
        ) < 0:
            raise ValueError("directional prediction revisions must not be negative")
        times = (self.observed_at_s, self.controller_time_s, self.snapshot_age_s)
        if not all(isfinite(value) for value in times) or min(times) < 0.0:
            raise ValueError("directional prediction times must be finite and non-negative")
        if not isclose(
            self.controller_time_s - self.observed_at_s,
            self.snapshot_age_s,
            rel_tol=0.0,
            abs_tol=_TIME_TOLERANCE_S,
        ):
            raise ValueError("snapshot age must match prediction timestamps")
        object.__setattr__(self, "tubes", tuple(self.tubes))


@dataclass(frozen=True, slots=True)
class DirectionalCapsuleSample:
    """A conservative circle-chain cover of one longitudinal capsule."""

    track_id: str
    actor_binding_id: str
    rollout_time_s: float
    prediction_horizon_s: float
    start: Point2D
    end: Point2D
    longitudinal_min_m: float
    longitudinal_max_m: float
    measurement_sigma_m: float
    base_radius_m: float
    covering_circle_radius_m: float
    circles: tuple[ActorTubeCircle, ...]

    def __post_init__(self) -> None:
        values = (
            self.rollout_time_s,
            self.prediction_horizon_s,
            self.longitudinal_min_m,
            self.longitudinal_max_m,
            self.measurement_sigma_m,
            self.base_radius_m,
            self.covering_circle_radius_m,
        )
        if not all(isfinite(value) for value in values) or min(values) < 0.0:
            raise ValueError("directional capsule values must be finite and non-negative")
        if self.longitudinal_min_m > self.longitudinal_max_m + 1e-12:
            raise ValueError("directional capsule distance bounds are reversed")
        if self.base_radius_m < ACTOR_RADIUS_M:
            raise ValueError("directional capsule must include the Actor radius")
        if self.covering_circle_radius_m < self.base_radius_m:
            raise ValueError("circle cover must not be smaller than the capsule")
        if not self.circles:
            raise ValueError("directional capsule must contain a circle cover")
        object.__setattr__(self, "circles", tuple(self.circles))


@dataclass(frozen=True, slots=True)
class DirectionalPredictionResult:
    status: DirectionalPredictionStatus
    prediction_set: DirectionalPredictionSet | None
    hold_required: bool
    reason_code: str
    history_counts: tuple[tuple[str, int], ...]
    duplicate_observation: bool = False
    session_reset: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "history_counts", tuple(self.history_counts))
        if self.status is DirectionalPredictionStatus.READY:
            if self.prediction_set is None or not self.prediction_set.tubes:
                raise ValueError("READY result requires directional tubes")
            if self.hold_required:
                raise ValueError("READY result must not request hold")
        if self.status is DirectionalPredictionStatus.EMPTY_FRAME:
            if self.prediction_set is None or self.prediction_set.tubes:
                raise ValueError("EMPTY_FRAME result requires an empty prediction set")
            if self.hold_required:
                raise ValueError("a fresh empty frame is not a dropout")
        if self.hold_required and self.prediction_set is not None:
            raise ValueError("hold result must not expose a directional prediction")


@dataclass(frozen=True, slots=True)
class _HistorySample:
    sequence: int
    observation_revision: int
    content_hash: str
    observed_at_s: float
    track_id: str
    actor_binding_id: str
    position: Point2D
    velocity: Vector2D
    position_sigma_m: float
    velocity_sigma_mps: float


@dataclass(frozen=True, slots=True)
class _FitOutcome:
    status: DirectionalPredictionStatus
    tube: DirectionalPredictionTube | None


class DirectionalActorPredictor:
    """Stateful, deterministic 20-frame direction estimator."""

    def __init__(
        self,
        parameters: DirectionalPredictionParameters = (
            FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS
        ),
    ) -> None:
        self.parameters = parameters
        self._session: tuple[str, str, int, str, int] | None = None
        self._last_frame_key: tuple[int, int, str] | None = None
        self._last_observed_at_s: float | None = None
        self._histories: dict[str, list[_HistorySample]] = {}

    def reset(self) -> None:
        self._session = None
        self._clear_observation_state()

    def update(self, snapshot: DynamicObservationSnapshot) -> DirectionalPredictionResult:
        """Consume one validated snapshot and return prediction or an explicit hold."""

        if not isinstance(snapshot, DynamicObservationSnapshot):
            raise TypeError("directional prediction input must be an observation snapshot")
        if snapshot.availability is DynamicObservationAvailability.INVALID:
            self.reset()
            return self._hold(DirectionalPredictionStatus.INVALID, "invalid_observation")
        if snapshot.availability is DynamicObservationAvailability.UNAVAILABLE:
            self.reset()
            return self._hold(
                DirectionalPredictionStatus.UNAVAILABLE,
                "observation_unavailable",
            )
        if snapshot.availability is DynamicObservationAvailability.STALE:
            self.reset()
            return self._hold(DirectionalPredictionStatus.STALE, "observation_stale")
        if not snapshot.usable or snapshot.frame is None or snapshot.age_s is None:
            self.reset()
            return self._hold(DirectionalPredictionStatus.INVALID, "malformed_fresh_snapshot")
        if not isfinite(snapshot.age_s) or snapshot.age_s < 0.0:
            self.reset()
            return self._hold(DirectionalPredictionStatus.INVALID, "invalid_snapshot_age")
        if snapshot.failures:
            self.reset()
            return self._hold(DirectionalPredictionStatus.INVALID, "snapshot_has_failures")
        if snapshot.last_event_was_no_frame:
            return self._hold(DirectionalPredictionStatus.DROPOUT, "frame_dropout")

        frame = snapshot.frame
        if dynamic_observation_content_hash(frame) != frame.content_hash:
            self.reset()
            return self._hold(
                DirectionalPredictionStatus.INVALID,
                "observation_content_hash_mismatch",
            )
        session = _session_identity(frame)
        session_reset = self._session is not None and self._session != session
        if self._session != session:
            self._clear_observation_state()
            self._session = session

        frame_key = (frame.sequence, frame.observation_revision, frame.content_hash)
        duplicate = frame_key == self._last_frame_key
        if not duplicate and self._last_frame_key is not None:
            order_valid = (
                frame.sequence > self._last_frame_key[0]
                and frame.observation_revision > self._last_frame_key[1]
                and self._last_observed_at_s is not None
                and frame.observed_at_s > self._last_observed_at_s + _TIME_TOLERANCE_S
            )
            if not order_valid:
                self._clear_observation_state()
                self._session = session
                return self._hold(
                    DirectionalPredictionStatus.ORDER_VIOLATION,
                    "observation_order_violation",
                    session_reset=session_reset,
                )

        if frame.frame_kind is DynamicObservationFrameKind.EMPTY:
            if frame.tracks:
                self.reset()
                return self._hold(DirectionalPredictionStatus.INVALID, "empty_frame_has_tracks")
            if not duplicate:
                self._histories.clear()
                self._remember_frame(frame, frame_key)
            prediction_set = _prediction_set(
                frame,
                snapshot,
                (),
                histories={},
                parameters=self.parameters,
            )
            return DirectionalPredictionResult(
                status=DirectionalPredictionStatus.EMPTY_FRAME,
                prediction_set=prediction_set,
                hold_required=False,
                reason_code="fresh_empty_frame",
                history_counts=(),
                duplicate_observation=duplicate,
                session_reset=session_reset,
            )
        if frame.frame_kind is not DynamicObservationFrameKind.TRACKS or not frame.tracks:
            self.reset()
            return self._hold(DirectionalPredictionStatus.INVALID, "tracks_frame_malformed")

        bindings = tuple(track.actor_binding_id for track in frame.tracks)
        if len(bindings) != len(set(bindings)):
            self.reset()
            return self._hold(DirectionalPredictionStatus.INVALID, "duplicate_actor_binding")
        if not duplicate:
            active_bindings = set(bindings)
            self._histories = {
                binding: history
                for binding, history in self._histories.items()
                if binding in active_bindings
            }
            for track in sorted(frame.tracks, key=lambda item: item.actor_binding_id):
                history = self._histories.get(track.actor_binding_id)
                if history and history[-1].track_id != track.track_id:
                    # A stable binding may not silently inherit a previous track
                    # identity's direction evidence.
                    self._histories.pop(track.actor_binding_id, None)
                self._append(frame, track)
            self._remember_frame(frame, frame_key)

        outcomes = tuple(
            self._fit(binding)
            for binding in sorted(bindings)
        )
        status = _combined_status(outcome.status for outcome in outcomes)
        counts = self._history_counts()
        if status is not DirectionalPredictionStatus.READY:
            if status in (
                DirectionalPredictionStatus.LOW_SPEED,
                DirectionalPredictionStatus.LOW_CONFIDENCE,
            ):
                # Once the confidence contract is violated, old evidence cannot
                # be recycled into a later direction lock.
                self._histories.clear()
            return DirectionalPredictionResult(
                status=status,
                prediction_set=None,
                hold_required=True,
                reason_code=status.value,
                history_counts=counts,
                duplicate_observation=duplicate,
                session_reset=session_reset,
            )
        tubes = tuple(outcome.tube for outcome in outcomes if outcome.tube is not None)
        return DirectionalPredictionResult(
            status=DirectionalPredictionStatus.READY,
            prediction_set=_prediction_set(
                frame,
                snapshot,
                tubes,
                histories=self._histories,
                parameters=self.parameters,
            ),
            hold_required=False,
            reason_code="direction_locked",
            history_counts=counts,
            duplicate_observation=duplicate,
            session_reset=session_reset,
        )

    def _append(self, frame: DynamicObservationFrame, track: ActorTrack) -> None:
        history = self._histories.setdefault(track.actor_binding_id, [])
        history.append(
            _HistorySample(
                sequence=frame.sequence,
                observation_revision=frame.observation_revision,
                content_hash=frame.content_hash,
                observed_at_s=frame.observed_at_s,
                track_id=track.track_id,
                actor_binding_id=track.actor_binding_id,
                position=track.observed_position,
                velocity=track.observed_velocity,
                position_sigma_m=track.position_sigma_m,
                velocity_sigma_mps=track.velocity_sigma_mps,
            )
        )
        del history[: -self.parameters.history_frame_count]

    def _fit(self, actor_binding_id: str) -> _FitOutcome:
        history = self._histories.get(actor_binding_id, [])
        if len(history) < self.parameters.history_frame_count:
            return _FitOutcome(DirectionalPredictionStatus.WARMING_UP, None)
        first_time_s = history[0].observed_at_s
        times = tuple(sample.observed_at_s - first_time_s for sample in history)
        span_s = times[-1] - times[0]
        if span_s + _TIME_TOLERANCE_S < self.parameters.minimum_history_span_s:
            return _FitOutcome(DirectionalPredictionStatus.LOW_CONFIDENCE, None)
        mean_velocity_x = sum(sample.velocity.x for sample in history) / len(history)
        mean_velocity_y = sum(sample.velocity.y for sample in history) / len(history)
        raw_speed_mps = hypot(mean_velocity_x, mean_velocity_y)
        if raw_speed_mps <= _GEOMETRY_TOLERANCE_M:
            return _FitOutcome(DirectionalPredictionStatus.LOW_SPEED, None)

        maximum_position_sigma_m = max(sample.position_sigma_m for sample in history)
        maximum_velocity_sigma_mps = max(sample.velocity_sigma_mps for sample in history)
        estimate_velocity_sigma_mps = maximum_velocity_sigma_mps / sqrt(len(history))
        lower_speed_mps = raw_speed_mps - (
            self.parameters.sigma_multiplier * estimate_velocity_sigma_mps
        )

        latest = history[-1]
        fitted_positions = tuple(
            Point2D(
                latest.position.x
                - mean_velocity_x * (times[-1] - time_s),
                latest.position.y
                - mean_velocity_y * (times[-1] - time_s),
            )
            for time_s in times
        )
        fit_rms_m = sqrt(
            sum(
                (sample.position.x - fitted.x) ** 2
                + (sample.position.y - fitted.y) ** 2
                for sample, fitted in zip(history, fitted_positions, strict=True)
            )
            / len(history)
        )
        allowed_fit_rms_m = (
            self.parameters.maximum_fit_rms_sigma_multiplier
            * sqrt(2.0)
            * maximum_position_sigma_m
        )
        if (
            lower_speed_mps < self.parameters.minimum_directional_speed_mps
            or fit_rms_m > allowed_fit_rms_m + _GEOMETRY_TOLERANCE_M
        ):
            return _FitOutcome(DirectionalPredictionStatus.LOW_CONFIDENCE, None)
        estimated_speed_mps = min(raw_speed_mps, self.parameters.maximum_speed_mps)
        heading_unit = Vector2D(
            mean_velocity_x / raw_speed_mps,
            mean_velocity_y / raw_speed_mps,
        )
        return _FitOutcome(
            DirectionalPredictionStatus.READY,
            DirectionalPredictionTube(
                track_id=history[-1].track_id,
                actor_binding_id=actor_binding_id,
                anchor_position=latest.position,
                heading_unit=heading_unit,
                estimated_speed_mps=estimated_speed_mps,
                position_sigma_m=latest.position_sigma_m,
                velocity_sigma_mps=estimate_velocity_sigma_mps,
                history_count=len(history),
                history_span_s=span_s,
                fit_rms_m=fit_rms_m,
                history_content_hash=_history_content_hash(history),
            ),
        )

    def _remember_frame(
        self,
        frame: DynamicObservationFrame,
        frame_key: tuple[int, int, str],
    ) -> None:
        self._last_frame_key = frame_key
        self._last_observed_at_s = frame.observed_at_s

    def _clear_observation_state(self) -> None:
        self._last_frame_key = None
        self._last_observed_at_s = None
        self._histories.clear()

    def _history_counts(self) -> tuple[tuple[str, int], ...]:
        return tuple(
            (binding, len(history))
            for binding, history in sorted(self._histories.items())
        )

    def _hold(
        self,
        status: DirectionalPredictionStatus,
        reason_code: str,
        *,
        session_reset: bool = False,
    ) -> DirectionalPredictionResult:
        return DirectionalPredictionResult(
            status=status,
            prediction_set=None,
            hold_required=True,
            reason_code=reason_code,
            history_counts=self._history_counts(),
            session_reset=session_reset,
        )


def sample_directional_capsules(
    prediction_set: DirectionalPredictionSet,
    *,
    rollout_time_s: float,
    parameters: DirectionalPredictionParameters = (
        FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS
    ),
) -> tuple[DirectionalCapsuleSample, ...]:
    """Sample forward-only longitudinal reachable capsules at one rollout time."""

    validate_directional_prediction_set(prediction_set)
    if parameters != FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS:
        raise ValueError("directional capsule sampling requires frozen parameters")
    if prediction_set.model_version != parameters.model_version:
        raise ValueError("directional prediction model and parameter versions differ")
    if not isfinite(rollout_time_s) or rollout_time_s < 0.0:
        raise ValueError("rollout_time_s must be finite and non-negative")
    horizon_s = (
        prediction_set.snapshot_age_s
        + parameters.command_apply_latency_s
        + rollout_time_s
    )
    if not isfinite(horizon_s):
        raise ValueError("directional prediction horizon must be finite")
    return tuple(
        _sample_capsule(tube, rollout_time_s, horizon_s, parameters)
        for tube in prediction_set.tubes
    )


def sample_directional_actor_circles(
    prediction_set: DirectionalPredictionSet,
    *,
    rollout_time_s: float,
    parameters: DirectionalPredictionParameters = (
        FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS
    ),
) -> tuple[ActorTubeCircle, ...]:
    """Flatten capsules into the existing circle-safety API representation."""

    return tuple(
        circle
        for capsule in sample_directional_capsules(
            prediction_set,
            rollout_time_s=rollout_time_s,
            parameters=parameters,
        )
        for circle in capsule.circles
    )


def _sample_capsule(
    tube: DirectionalPredictionTube,
    rollout_time_s: float,
    horizon_s: float,
    parameters: DirectionalPredictionParameters,
) -> DirectionalCapsuleSample:
    longitudinal_min_m = _limited_braking_distance_over_time(
        tube.estimated_speed_mps,
        horizon_s,
        parameters.maximum_longitudinal_deceleration_mps2,
    )
    longitudinal_max_m = _limited_acceleration_distance_over_time(
        tube.estimated_speed_mps,
        horizon_s,
        parameters.maximum_speed_mps,
        parameters.maximum_longitudinal_acceleration_mps2,
    )
    measurement_sigma_m = hypot(
        tube.position_sigma_m,
        horizon_s * tube.velocity_sigma_mps,
    )
    base_radius_m = ACTOR_RADIUS_M + (
        parameters.sigma_multiplier * measurement_sigma_m
    )
    start = _advance(tube.anchor_position, tube.heading_unit, longitudinal_min_m)
    end = _advance(tube.anchor_position, tube.heading_unit, longitudinal_max_m)
    length_m = max(0.0, longitudinal_max_m - longitudinal_min_m)
    segment_count = (
        0
        if length_m <= _GEOMETRY_TOLERANCE_M
        else max(1, ceil(length_m / parameters.maximum_circle_spacing_m))
    )
    actual_spacing_m = length_m / segment_count if segment_count else 0.0
    covering_radius_m = hypot(base_radius_m, actual_spacing_m / 2.0)
    sample_count = segment_count + 1
    circles = tuple(
        ActorTubeCircle(
            track_id=tube.track_id,
            actor_binding_id=tube.actor_binding_id,
            rollout_time_s=rollout_time_s,
            prediction_horizon_s=horizon_s,
            center=_advance(
                start,
                tube.heading_unit,
                actual_spacing_m * index,
            ),
            radius_m=covering_radius_m,
            position_sigma_m=measurement_sigma_m,
            acceleration_bound_m=length_m,
        )
        for index in range(sample_count)
    )
    return DirectionalCapsuleSample(
        track_id=tube.track_id,
        actor_binding_id=tube.actor_binding_id,
        rollout_time_s=rollout_time_s,
        prediction_horizon_s=horizon_s,
        start=start,
        end=end,
        longitudinal_min_m=longitudinal_min_m,
        longitudinal_max_m=longitudinal_max_m,
        measurement_sigma_m=measurement_sigma_m,
        base_radius_m=base_radius_m,
        covering_circle_radius_m=covering_radius_m,
        circles=circles,
    )


def _limited_braking_distance_over_time(
    speed_mps: float,
    horizon_s: float,
    deceleration_mps2: float,
) -> float:
    stop_time_s = speed_mps / deceleration_mps2
    active_time_s = min(horizon_s, stop_time_s)
    distance_m = (
        speed_mps * active_time_s
        - 0.5 * deceleration_mps2 * active_time_s**2
    )
    return max(0.0, distance_m)


def _limited_acceleration_distance_over_time(
    speed_mps: float,
    horizon_s: float,
    maximum_speed_mps: float,
    acceleration_mps2: float,
) -> float:
    acceleration_time_s = max(0.0, maximum_speed_mps - speed_mps) / acceleration_mps2
    active_time_s = min(horizon_s, acceleration_time_s)
    return (
        speed_mps * active_time_s
        + 0.5 * acceleration_mps2 * active_time_s**2
        + maximum_speed_mps * (horizon_s - active_time_s)
    )


def _advance(position: Point2D, heading: Vector2D, distance_m: float) -> Point2D:
    return Point2D(
        position.x + heading.x * distance_m,
        position.y + heading.y * distance_m,
    )


def _session_identity(frame: DynamicObservationFrame) -> tuple[str, str, int, str, int]:
    return (
        frame.stream_id,
        frame.episode_id,
        frame.episode_seed,
        frame.map_id,
        frame.map_revision,
    )


def _prediction_set(
    frame: DynamicObservationFrame,
    snapshot: DynamicObservationSnapshot,
    tubes: tuple[DirectionalPredictionTube, ...],
    *,
    histories: dict[str, list[_HistorySample]],
    parameters: DirectionalPredictionParameters,
) -> DirectionalPredictionSet:
    assert snapshot.age_s is not None
    ordered_history_hashes = tuple(
        (
            tube.actor_binding_id,
            tube.history_content_hash,
        )
        for tube in tubes
    )
    history_content_hash = _hash_payload(
        {
            "current_frame_content_hash": frame.content_hash,
            "current_frame_kind": frame.frame_kind.value,
            "tube_history_hashes": ordered_history_hashes,
            "history_sizes": tuple(
                (binding, len(history))
                for binding, history in sorted(histories.items())
            ),
        }
    )
    parameter_content_hash = _parameter_content_hash(parameters)
    fields = {
        "model_version": DIRECTIONAL_PREDICTION_VERSION,
        "stream_id": frame.stream_id,
        "episode_id": frame.episode_id,
        "episode_seed": frame.episode_seed,
        "map_id": frame.map_id,
        "map_revision": frame.map_revision,
        "observation_revision": frame.observation_revision,
        "sequence": frame.sequence,
        "source_content_hash": frame.content_hash,
        "observed_at_s": frame.observed_at_s,
        "controller_time_s": frame.observed_at_s + snapshot.age_s,
        "snapshot_age_s": snapshot.age_s,
        "tubes": tubes,
        "parameter_content_hash": parameter_content_hash,
        "history_content_hash": history_content_hash,
    }
    content_hash = _prediction_content_hash(**fields)
    prediction = DirectionalPredictionSet(**fields, content_hash=content_hash)
    return _issue_directional_prediction(prediction)


def validate_directional_prediction_set(
    prediction_set: DirectionalPredictionSet,
    *,
    current_frame: DynamicObservationFrame | None = None,
) -> None:
    """Validate an issued immutable prediction and, optionally, its live frame.

    The twenty-frame history cannot be reconstructed from one controller frame.
    The stateful predictor therefore issues an identity-bound capability and
    commits the complete accepted history into immutable hashes.  Copying with
    ``dataclasses.replace`` is not issuance; in-place field tampering invalidates
    the registered semantic commitment.
    """

    if not isinstance(prediction_set, DirectionalPredictionSet):
        raise TypeError("directional prediction must be a DirectionalPredictionSet")
    with _ISSUED_PREDICTIONS_LOCK:
        issued = _ISSUED_PREDICTIONS.get(id(prediction_set))
    if issued is None or issued[0]() is not prediction_set:
        raise ValueError("directional prediction was not issued by the predictor")
    if issued[1] != prediction_set.content_hash:
        raise ValueError("directional prediction issuance commitment mismatch")
    if prediction_set.model_version != DIRECTIONAL_PREDICTION_VERSION:
        raise ValueError("directional prediction model version mismatch")
    if prediction_set.parameter_content_hash != _parameter_content_hash(
        FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS
    ):
        raise ValueError("directional prediction parameter commitment mismatch")
    expected_history_hash = _hash_payload(
        {
            "current_frame_content_hash": prediction_set.source_content_hash,
            "current_frame_kind": (
                DynamicObservationFrameKind.TRACKS.value
                if prediction_set.tubes
                else DynamicObservationFrameKind.EMPTY.value
            ),
            "tube_history_hashes": tuple(
                (tube.actor_binding_id, tube.history_content_hash)
                for tube in prediction_set.tubes
            ),
            "history_sizes": tuple(
                (tube.actor_binding_id, tube.history_count)
                for tube in prediction_set.tubes
            ),
        }
    )
    if prediction_set.history_content_hash != expected_history_hash:
        raise ValueError("directional prediction history commitment mismatch")
    expected_content_hash = _prediction_content_hash(
        model_version=prediction_set.model_version,
        stream_id=prediction_set.stream_id,
        episode_id=prediction_set.episode_id,
        episode_seed=prediction_set.episode_seed,
        map_id=prediction_set.map_id,
        map_revision=prediction_set.map_revision,
        observation_revision=prediction_set.observation_revision,
        sequence=prediction_set.sequence,
        source_content_hash=prediction_set.source_content_hash,
        observed_at_s=prediction_set.observed_at_s,
        controller_time_s=prediction_set.controller_time_s,
        snapshot_age_s=prediction_set.snapshot_age_s,
        tubes=prediction_set.tubes,
        parameter_content_hash=prediction_set.parameter_content_hash,
        history_content_hash=prediction_set.history_content_hash,
    )
    if prediction_set.content_hash != expected_content_hash:
        raise ValueError("directional prediction semantic commitment mismatch")

    if current_frame is None:
        return
    if (
        prediction_set.stream_id,
        prediction_set.episode_id,
        prediction_set.episode_seed,
        prediction_set.map_id,
        prediction_set.map_revision,
        prediction_set.observation_revision,
        prediction_set.sequence,
        prediction_set.source_content_hash,
        prediction_set.observed_at_s,
    ) != (
        current_frame.stream_id,
        current_frame.episode_id,
        current_frame.episode_seed,
        current_frame.map_id,
        current_frame.map_revision,
        current_frame.observation_revision,
        current_frame.sequence,
        current_frame.content_hash,
        current_frame.observed_at_s,
    ):
        raise ValueError("directional prediction current-frame provenance mismatch")

    if current_frame.frame_kind is DynamicObservationFrameKind.EMPTY:
        if current_frame.tracks or prediction_set.tubes:
            raise ValueError("directional empty-frame binding mismatch")
        return
    if current_frame.frame_kind is not DynamicObservationFrameKind.TRACKS:
        raise ValueError("directional prediction frame kind is unsupported")
    tracks = {
        (track.track_id, track.actor_binding_id): track
        for track in current_frame.tracks
    }
    if len(tracks) != len(current_frame.tracks) or len(tracks) != len(
        prediction_set.tubes
    ):
        raise ValueError("directional prediction current Actor binding mismatch")
    for tube in prediction_set.tubes:
        track = tracks.get((tube.track_id, tube.actor_binding_id))
        if track is None:
            raise ValueError("directional prediction current Actor binding mismatch")
        if (
            tube.anchor_position != track.observed_position
            or tube.position_sigma_m != track.position_sigma_m
        ):
            raise ValueError("directional prediction current Actor anchor mismatch")


def _issue_directional_prediction(
    prediction: DirectionalPredictionSet,
) -> DirectionalPredictionSet:
    identifier = id(prediction)

    def _discard(dead_ref: ref[DirectionalPredictionSet]) -> None:
        with _ISSUED_PREDICTIONS_LOCK:
            current = _ISSUED_PREDICTIONS.get(identifier)
            if current is not None and current[0] is dead_ref:
                _ISSUED_PREDICTIONS.pop(identifier, None)

    weak_prediction = ref(prediction, _discard)
    with _ISSUED_PREDICTIONS_LOCK:
        _ISSUED_PREDICTIONS[identifier] = (
            weak_prediction,
            prediction.content_hash,
        )
    return prediction


def _history_content_hash(history: list[_HistorySample]) -> str:
    return _hash_payload(
        tuple(
            {
                "sequence": sample.sequence,
                "observation_revision": sample.observation_revision,
                "content_hash": sample.content_hash,
                "observed_at_s": sample.observed_at_s.hex(),
                "track_id": sample.track_id,
                "actor_binding_id": sample.actor_binding_id,
                "position": (sample.position.x.hex(), sample.position.y.hex()),
                "velocity": (sample.velocity.x.hex(), sample.velocity.y.hex()),
                "position_sigma_m": sample.position_sigma_m.hex(),
                "velocity_sigma_mps": sample.velocity_sigma_mps.hex(),
            }
            for sample in history
        )
    )


def _parameter_content_hash(parameters: DirectionalPredictionParameters) -> str:
    return _hash_payload(
        {
            name: value.hex() if isinstance(value, float) else value
            for name, value in (
                ("model_version", parameters.model_version),
                ("history_frame_count", parameters.history_frame_count),
                ("minimum_history_span_s", parameters.minimum_history_span_s),
                ("sigma_multiplier", parameters.sigma_multiplier),
                (
                    "minimum_directional_speed_mps",
                    parameters.minimum_directional_speed_mps,
                ),
                ("maximum_speed_mps", parameters.maximum_speed_mps),
                (
                    "maximum_longitudinal_acceleration_mps2",
                    parameters.maximum_longitudinal_acceleration_mps2,
                ),
                (
                    "maximum_longitudinal_deceleration_mps2",
                    parameters.maximum_longitudinal_deceleration_mps2,
                ),
                ("lateral_turn_bound_m", parameters.lateral_turn_bound_m),
                (
                    "maximum_fit_rms_sigma_multiplier",
                    parameters.maximum_fit_rms_sigma_multiplier,
                ),
                ("maximum_circle_spacing_m", parameters.maximum_circle_spacing_m),
                ("command_apply_latency_s", parameters.command_apply_latency_s),
            )
        }
    )


def _prediction_content_hash(**fields: object) -> str:
    payload = {
        name: _semantic_value(value)
        for name, value in fields.items()
    }
    return _hash_payload(payload)


def _semantic_value(value: object) -> object:
    if isinstance(value, float):
        return value.hex()
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    if isinstance(value, Point2D):
        return {"x": value.x.hex(), "y": value.y.hex()}
    if isinstance(value, Vector2D):
        return {"x": value.x.hex(), "y": value.y.hex()}
    if isinstance(value, DirectionalPredictionTube):
        return {
            name: _semantic_value(getattr(value, name))
            for name in value.__dataclass_fields__
        }
    if isinstance(value, tuple):
        return tuple(_semantic_value(item) for item in value)
    raise TypeError(f"unsupported directional commitment value: {type(value).__name__}")


def _hash_payload(payload: object) -> str:
    serialized = dumps(
        payload,
        ensure_ascii=True,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    return sha256(serialized.encode("ascii")).hexdigest()


def _combined_status(
    statuses: Iterable[DirectionalPredictionStatus],
) -> DirectionalPredictionStatus:
    normalized = tuple(statuses)
    for status in (
        DirectionalPredictionStatus.WARMING_UP,
        DirectionalPredictionStatus.LOW_SPEED,
        DirectionalPredictionStatus.LOW_CONFIDENCE,
    ):
        if status in normalized:
            return status
    return DirectionalPredictionStatus.READY
