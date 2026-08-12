"""Deterministic 10 Hz Actor observation generation and source validation."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import StrEnum
from hashlib import sha256
from json import dumps
from math import isclose, isfinite
from random import Random
from typing import Any

from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DYNAMIC_OBSERVATION_PERIOD_S,
    DYNAMIC_OBSERVATION_TTL_S,
    ActorTrack,
    DynamicGroundTruthFrame,
    DynamicObservationFrame,
    DynamicObservationFrameKind,
    Point2D,
    Vector2D,
)
from hospital_path_lab.map_factory import canonical_content_hash

DYNAMIC_OBSERVATION_GENERATOR_VERSION = "dynamic_observation_v1"
_NANOSECONDS_PER_SECOND = 1_000_000_000
_SHA256_HEX_LENGTH = 64
_TIME_ABS_TOLERANCE_S = 1e-12


def _seconds_to_ns(seconds: float) -> int:
    return round(seconds * _NANOSECONDS_PER_SECOND)


class DynamicObservationProfileName(StrEnum):
    NORMAL = "normal"
    FUNCTIONAL_NO_DROPOUT = "functional_no_dropout"
    FUNCTIONAL_IDEAL = "functional_ideal"
    STRESS = "stress"
    BOUNDARY_300 = "boundary_300"
    BOUNDARY_350 = "boundary_350"


@dataclass(frozen=True, slots=True)
class DynamicObservationProfile:
    name: DynamicObservationProfileName
    latency_s: float
    ttl_s: float
    position_sigma_m: float
    velocity_sigma_mps: float
    dropout_probability: float
    observation_period_s: float = DYNAMIC_OBSERVATION_PERIOD_S

    def __post_init__(self) -> None:
        values = (
            self.observation_period_s,
            self.latency_s,
            self.ttl_s,
            self.position_sigma_m,
            self.velocity_sigma_mps,
            self.dropout_probability,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("observation profile values must be finite")
        if self.observation_period_s <= 0.0 or self.latency_s < 0.0 or self.ttl_s < 0.0:
            raise ValueError("observation period must be positive and times must not be negative")
        if min(self.position_sigma_m, self.velocity_sigma_mps) < 0.0:
            raise ValueError("observation uncertainty must not be negative")
        if not 0.0 <= self.dropout_probability <= 1.0:
            raise ValueError("dropout_probability must be between zero and one")
        if _seconds_to_ns(self.observation_period_s) != _seconds_to_ns(
            DYNAMIC_OBSERVATION_PERIOD_S
        ):
            raise ValueError("dynamic observation profile must remain at 10 Hz")


NORMAL_OBSERVATION_PROFILE = DynamicObservationProfile(
    name=DynamicObservationProfileName.NORMAL,
    latency_s=0.100,
    ttl_s=DYNAMIC_OBSERVATION_TTL_S,
    position_sigma_m=0.03,
    velocity_sigma_mps=0.05,
    dropout_probability=0.05,
)
FUNCTIONAL_NO_DROPOUT_OBSERVATION_PROFILE = DynamicObservationProfile(
    name=DynamicObservationProfileName.FUNCTIONAL_NO_DROPOUT,
    latency_s=NORMAL_OBSERVATION_PROFILE.latency_s,
    ttl_s=NORMAL_OBSERVATION_PROFILE.ttl_s,
    position_sigma_m=NORMAL_OBSERVATION_PROFILE.position_sigma_m,
    velocity_sigma_mps=NORMAL_OBSERVATION_PROFILE.velocity_sigma_mps,
    dropout_probability=0.0,
)
FUNCTIONAL_IDEAL_OBSERVATION_PROFILE = DynamicObservationProfile(
    name=DynamicObservationProfileName.FUNCTIONAL_IDEAL,
    latency_s=NORMAL_OBSERVATION_PROFILE.latency_s,
    ttl_s=NORMAL_OBSERVATION_PROFILE.ttl_s,
    position_sigma_m=0.0,
    velocity_sigma_mps=0.0,
    dropout_probability=0.0,
)
STRESS_OBSERVATION_PROFILE = DynamicObservationProfile(
    name=DynamicObservationProfileName.STRESS,
    latency_s=0.250,
    ttl_s=DYNAMIC_OBSERVATION_TTL_S,
    position_sigma_m=0.08,
    velocity_sigma_mps=0.15,
    dropout_probability=0.20,
)
BOUNDARY_300_OBSERVATION_PROFILE = DynamicObservationProfile(
    name=DynamicObservationProfileName.BOUNDARY_300,
    latency_s=0.300,
    ttl_s=DYNAMIC_OBSERVATION_TTL_S,
    position_sigma_m=0.0,
    velocity_sigma_mps=0.0,
    dropout_probability=0.0,
)
BOUNDARY_350_OBSERVATION_PROFILE = DynamicObservationProfile(
    name=DynamicObservationProfileName.BOUNDARY_350,
    latency_s=0.350,
    ttl_s=DYNAMIC_OBSERVATION_TTL_S,
    position_sigma_m=0.0,
    velocity_sigma_mps=0.0,
    dropout_probability=0.0,
)


@dataclass(frozen=True, slots=True)
class DynamicObservationSourceIdentity:
    """Stage 2 source identity injected without modifying the Stage 1 trace."""

    stream_id: str
    episode_id: str
    episode_seed: int
    map_id: str
    map_revision: int

    def __post_init__(self) -> None:
        if not self.stream_id or not self.episode_id or not self.map_id:
            raise ValueError("observation source identity fields must not be empty")
        if self.map_revision < 0:
            raise ValueError("map_revision must not be negative")


@dataclass(frozen=True, slots=True)
class FourFrameBurstDropout:
    start_sequence: int
    frame_count: int = 4

    def __post_init__(self) -> None:
        if self.start_sequence < 0:
            raise ValueError("burst start_sequence must not be negative")
        if self.frame_count != 4:
            raise ValueError("the frozen burst fault must drop exactly four frames")

    def contains(self, sequence: int) -> bool:
        return self.start_sequence <= sequence < self.start_sequence + self.frame_count


class DynamicObservationDropKind(StrEnum):
    NONE = "none"
    INDEPENDENT = "independent"
    FORCED_BURST = "forced_burst"


@dataclass(frozen=True, slots=True)
class DynamicObservationSlot:
    """Generator record; only ``frame`` is exposed to a controller at delivery."""

    sequence: int
    observed_at_s: float
    scheduled_delivery_at_s: float
    frame: DynamicObservationFrame | None
    drop_kind: DynamicObservationDropKind

    @property
    def delivered(self) -> bool:
        return self.frame is not None


class DynamicObservationValidationReason(StrEnum):
    STREAM_ID_MISMATCH = "stream_id_mismatch"
    EPISODE_ID_MISMATCH = "episode_id_mismatch"
    EPISODE_SEED_MISMATCH = "episode_seed_mismatch"
    MAP_ID_MISMATCH = "map_id_mismatch"
    MAP_REVISION_MISMATCH = "map_revision_mismatch"
    INVALID_SEQUENCE = "invalid_sequence"
    INVALID_OBSERVATION_REVISION = "invalid_observation_revision"
    SEQUENCE_NOT_INCREASING = "sequence_not_increasing"
    OBSERVATION_REVISION_REGRESSED = "observation_revision_regressed"
    OBSERVATION_TIME_REGRESSED = "observation_time_regressed"
    DUPLICATE_TRACK_ID = "duplicate_track_id"
    ACTOR_BINDING_CHANGED = "actor_binding_changed"
    FRAME_KIND_TRACK_MISMATCH = "frame_kind_track_mismatch"
    NON_FINITE_TRACK = "non_finite_track"
    NON_FINITE_UNCERTAINTY = "non_finite_uncertainty"
    NEGATIVE_UNCERTAINTY = "negative_uncertainty"
    UNCERTAINTY_PROFILE_MISMATCH = "uncertainty_profile_mismatch"
    NON_FINITE_TIMESTAMP = "non_finite_timestamp"
    NEGATIVE_TIMESTAMP = "negative_timestamp"
    OBSERVATION_AFTER_DELIVERY = "observation_after_delivery"
    DELIVERY_IN_FUTURE = "delivery_in_future"
    DELIVERY_TIME_REGRESSED = "delivery_time_regressed"
    LATENCY_MISMATCH = "latency_mismatch"
    CONTENT_HASH_MALFORMED = "content_hash_malformed"
    CONTENT_HASH_MISMATCH = "content_hash_mismatch"
    STALE = "stale"
    NO_VALID_FRAME = "no_valid_frame"


@dataclass(frozen=True, slots=True)
class DynamicObservationValidationResult:
    accepted: bool
    source_valid: bool
    failures: tuple[DynamicObservationValidationReason, ...]


class DynamicObservationAvailability(StrEnum):
    FRESH = "fresh"
    STALE = "stale"
    UNAVAILABLE = "unavailable"
    INVALID = "invalid"


@dataclass(frozen=True, slots=True)
class DynamicObservationSnapshot:
    availability: DynamicObservationAvailability
    frame: DynamicObservationFrame | None
    age_s: float | None
    failures: tuple[DynamicObservationValidationReason, ...]
    last_event_was_no_frame: bool

    @property
    def usable(self) -> bool:
        return self.availability is DynamicObservationAvailability.FRESH


def generate_dynamic_observation_slots(
    ground_truth_frames: Sequence[DynamicGroundTruthFrame],
    *,
    source: DynamicObservationSourceIdentity,
    profile: DynamicObservationProfile,
    burst_dropout: FourFrameBurstDropout | None = None,
) -> tuple[DynamicObservationSlot, ...]:
    """Generate deterministic 10 Hz observations from a 20 Hz Stage 1 trace."""

    slots: list[DynamicObservationSlot] = []
    for truth in ground_truth_frames:
        _validate_truth_source(truth, source)
        if truth.tick_id % 2:
            continue
        expected_time_s = truth.tick_id * DYNAMIC_CONTROL_PERIOD_S
        if _seconds_to_ns(truth.simulation_time_s) != _seconds_to_ns(expected_time_s):
            raise ValueError("ground truth timestamp must derive from its 20 Hz tick")

        sequence = truth.tick_id // 2
        observed_at_s = sequence * profile.observation_period_s
        delivered_at_s = observed_at_s + profile.latency_s
        forced = burst_dropout is not None and burst_dropout.contains(sequence)
        independent = _uniform_draw(
            source,
            sequence,
            component="dropout",
        ) < profile.dropout_probability
        if forced or independent:
            slots.append(
                DynamicObservationSlot(
                    sequence=sequence,
                    observed_at_s=observed_at_s,
                    scheduled_delivery_at_s=delivered_at_s,
                    frame=None,
                    drop_kind=(
                        DynamicObservationDropKind.FORCED_BURST
                        if forced
                        else DynamicObservationDropKind.INDEPENDENT
                    ),
                )
            )
            continue

        tracks = tuple(
            _observed_track(actor, source=source, profile=profile, sequence=sequence)
            for actor in sorted(truth.actors, key=lambda item: item.actor_id)
        )
        frame = _build_observation_frame(
            source=source,
            map_revision=truth.map_revision,
            observation_revision=sequence,
            sequence=sequence,
            observed_at_s=observed_at_s,
            delivered_at_s=delivered_at_s,
            tracks=tracks,
        )
        slots.append(
            DynamicObservationSlot(
                sequence=sequence,
                observed_at_s=observed_at_s,
                scheduled_delivery_at_s=delivered_at_s,
                frame=frame,
                drop_kind=DynamicObservationDropKind.NONE,
            )
        )
    return tuple(slots)


def dynamic_observation_content_hash(frame: DynamicObservationFrame) -> str:
    """Hash every semantic frame field except the hash field itself."""

    return canonical_content_hash(_observation_payload(frame))


class DynamicObservationValidator:
    """Stateful, transactional source validator and TTL holder."""

    def __init__(
        self,
        source: DynamicObservationSourceIdentity,
        profile: DynamicObservationProfile,
    ) -> None:
        self.source = source
        self.profile = profile
        self._last_sequence: int | None = None
        self._last_observation_revision: int | None = None
        self._last_observed_ns: int | None = None
        self._last_event_sequence: int | None = None
        self._last_event_delivery_ns: int | None = None
        self._last_valid_frame: DynamicObservationFrame | None = None
        self._bindings: dict[str, str] = {}
        self._latest_failures: tuple[DynamicObservationValidationReason, ...] = ()
        self._last_event_was_no_frame = False

    def accept(
        self,
        frame: DynamicObservationFrame,
        *,
        received_at_s: float,
    ) -> DynamicObservationValidationResult:
        failures = self._collect_failures(frame, received_at_s=received_at_s)
        if failures:
            self._latest_failures = tuple(failures)
            self._last_event_was_no_frame = False
            return DynamicObservationValidationResult(
                accepted=False,
                source_valid=False,
                failures=tuple(failures),
            )

        # All state changes occur only after every check passes.
        self._last_sequence = frame.sequence
        self._last_observation_revision = frame.observation_revision
        self._last_observed_ns = _seconds_to_ns(frame.observed_at_s)
        self._last_event_sequence = frame.sequence
        self._last_event_delivery_ns = _seconds_to_ns(frame.delivered_at_s)
        for track in frame.tracks:
            self._bindings[track.track_id] = track.actor_binding_id
        self._last_valid_frame = frame
        self._latest_failures = ()
        self._last_event_was_no_frame = False
        return DynamicObservationValidationResult(accepted=True, source_valid=True, failures=())

    def record_no_frame(self, *, sequence: int, delivery_time_s: float) -> None:
        if not _is_non_negative_int(sequence):
            raise ValueError("no-frame sequence must be a non-negative integer")
        if not isfinite(delivery_time_s) or delivery_time_s < 0.0:
            raise ValueError("no-frame delivery time must be finite and non-negative")
        delivery_ns = _seconds_to_ns(delivery_time_s)
        if self._last_event_sequence is not None and sequence <= self._last_event_sequence:
            raise ValueError("no-frame sequence must increase")
        expected_delivery_s = (
            sequence * self.profile.observation_period_s + self.profile.latency_s
        )
        if not isclose(
            delivery_time_s,
            expected_delivery_s,
            rel_tol=0.0,
            abs_tol=_TIME_ABS_TOLERANCE_S,
        ):
            raise ValueError("no-frame delivery time must match its 10 Hz sequence")
        if (
            self._last_event_delivery_ns is not None
            and delivery_ns < self._last_event_delivery_ns
        ):
            raise ValueError("no-frame delivery time must not regress")
        self._last_event_sequence = sequence
        self._last_event_delivery_ns = delivery_ns
        self._last_event_was_no_frame = True

    def snapshot(self, *, control_time_s: float) -> DynamicObservationSnapshot:
        if not isfinite(control_time_s):
            return DynamicObservationSnapshot(
                availability=DynamicObservationAvailability.INVALID,
                frame=self._last_valid_frame,
                age_s=None,
                failures=(DynamicObservationValidationReason.NON_FINITE_TIMESTAMP,),
                last_event_was_no_frame=self._last_event_was_no_frame,
            )
        if control_time_s < 0.0:
            return DynamicObservationSnapshot(
                availability=DynamicObservationAvailability.INVALID,
                frame=self._last_valid_frame,
                age_s=None,
                failures=(DynamicObservationValidationReason.NEGATIVE_TIMESTAMP,),
                last_event_was_no_frame=self._last_event_was_no_frame,
            )
        if self._latest_failures:
            return DynamicObservationSnapshot(
                availability=DynamicObservationAvailability.INVALID,
                frame=self._last_valid_frame,
                age_s=self._age_s(control_time_s),
                failures=self._latest_failures,
                last_event_was_no_frame=self._last_event_was_no_frame,
            )
        if self._last_valid_frame is None:
            return DynamicObservationSnapshot(
                availability=DynamicObservationAvailability.UNAVAILABLE,
                frame=None,
                age_s=None,
                failures=(DynamicObservationValidationReason.NO_VALID_FRAME,),
                last_event_was_no_frame=self._last_event_was_no_frame,
            )

        if (
            self._last_valid_frame.delivered_at_s - control_time_s
            > _TIME_ABS_TOLERANCE_S
        ):
            return DynamicObservationSnapshot(
                availability=DynamicObservationAvailability.INVALID,
                frame=self._last_valid_frame,
                age_s=None,
                failures=(DynamicObservationValidationReason.DELIVERY_IN_FUTURE,),
                last_event_was_no_frame=self._last_event_was_no_frame,
            )
        age_s = self._age_s(control_time_s)
        if age_s is None or age_s < -_TIME_ABS_TOLERANCE_S:
            return DynamicObservationSnapshot(
                availability=DynamicObservationAvailability.INVALID,
                frame=self._last_valid_frame,
                age_s=age_s,
                failures=(DynamicObservationValidationReason.OBSERVATION_AFTER_DELIVERY,),
                last_event_was_no_frame=self._last_event_was_no_frame,
            )
        age_s = 0.0 if age_s < 0.0 else age_s
        if isclose(
            age_s,
            self.profile.ttl_s,
            rel_tol=0.0,
            abs_tol=_TIME_ABS_TOLERANCE_S,
        ):
            age_s = self.profile.ttl_s
        stale = dynamic_observation_age_is_stale(age_s, ttl_s=self.profile.ttl_s)
        return DynamicObservationSnapshot(
            availability=(
                DynamicObservationAvailability.STALE
                if stale
                else DynamicObservationAvailability.FRESH
            ),
            frame=self._last_valid_frame,
            age_s=age_s,
            failures=(DynamicObservationValidationReason.STALE,) if stale else (),
            last_event_was_no_frame=self._last_event_was_no_frame,
        )

    def _age_s(self, control_time_s: float) -> float | None:
        if self._last_observed_ns is None:
            return None
        return control_time_s - self._last_observed_ns / _NANOSECONDS_PER_SECOND

    def _collect_failures(
        self,
        frame: DynamicObservationFrame,
        *,
        received_at_s: float,
    ) -> list[DynamicObservationValidationReason]:
        failures: list[DynamicObservationValidationReason] = []

        _append_if(
            failures,
            frame.stream_id != self.source.stream_id,
            DynamicObservationValidationReason.STREAM_ID_MISMATCH,
        )
        _append_if(
            failures,
            frame.episode_id != self.source.episode_id,
            DynamicObservationValidationReason.EPISODE_ID_MISMATCH,
        )
        _append_if(
            failures,
            frame.episode_seed != self.source.episode_seed,
            DynamicObservationValidationReason.EPISODE_SEED_MISMATCH,
        )
        _append_if(
            failures,
            frame.map_id != self.source.map_id,
            DynamicObservationValidationReason.MAP_ID_MISMATCH,
        )
        _append_if(
            failures,
            frame.map_revision != self.source.map_revision,
            DynamicObservationValidationReason.MAP_REVISION_MISMATCH,
        )

        sequence_valid = _is_non_negative_int(frame.sequence)
        revision_valid = _is_non_negative_int(frame.observation_revision)
        _append_if(
            failures,
            not sequence_valid,
            DynamicObservationValidationReason.INVALID_SEQUENCE,
        )
        _append_if(
            failures,
            not revision_valid,
            DynamicObservationValidationReason.INVALID_OBSERVATION_REVISION,
        )
        if sequence_valid and self._last_event_sequence is not None:
            _append_if(
                failures,
                frame.sequence <= self._last_event_sequence,
                DynamicObservationValidationReason.SEQUENCE_NOT_INCREASING,
            )
        if revision_valid and self._last_observation_revision is not None:
            _append_if(
                failures,
                frame.observation_revision < self._last_observation_revision,
                DynamicObservationValidationReason.OBSERVATION_REVISION_REGRESSED,
            )

        timestamps = (frame.observed_at_s, frame.delivered_at_s, received_at_s)
        timestamps_finite = all(isfinite(value) for value in timestamps)
        _append_if(
            failures,
            not timestamps_finite,
            DynamicObservationValidationReason.NON_FINITE_TIMESTAMP,
        )
        if timestamps_finite:
            _append_if(
                failures,
                min(timestamps) < 0.0,
                DynamicObservationValidationReason.NEGATIVE_TIMESTAMP,
            )
            observed_ns = _seconds_to_ns(frame.observed_at_s)
            delivered_ns = _seconds_to_ns(frame.delivered_at_s)
            received_ns = _seconds_to_ns(received_at_s)
            _append_if(
                failures,
                observed_ns > delivered_ns,
                DynamicObservationValidationReason.OBSERVATION_AFTER_DELIVERY,
            )
            _append_if(
                failures,
                delivered_ns > received_ns,
                DynamicObservationValidationReason.DELIVERY_IN_FUTURE,
            )
            if self._last_event_delivery_ns is not None:
                _append_if(
                    failures,
                    delivered_ns < self._last_event_delivery_ns,
                    DynamicObservationValidationReason.DELIVERY_TIME_REGRESSED,
                )
            _append_if(
                failures,
                delivered_ns - observed_ns != _seconds_to_ns(self.profile.latency_s),
                DynamicObservationValidationReason.LATENCY_MISMATCH,
            )
            if self._last_observed_ns is not None:
                _append_if(
                    failures,
                    observed_ns < self._last_observed_ns,
                    DynamicObservationValidationReason.OBSERVATION_TIME_REGRESSED,
                )

        tracks = tuple(frame.tracks)
        track_ids = tuple(getattr(track, "track_id", "") for track in tracks)
        _append_if(
            failures,
            len(track_ids) != len(set(track_ids)),
            DynamicObservationValidationReason.DUPLICATE_TRACK_ID,
        )
        if frame.frame_kind is DynamicObservationFrameKind.TRACKS:
            kind_mismatch = not tracks
        elif frame.frame_kind is DynamicObservationFrameKind.EMPTY:
            kind_mismatch = bool(tracks)
        else:
            kind_mismatch = True
        _append_if(
            failures,
            kind_mismatch,
            DynamicObservationValidationReason.FRAME_KIND_TRACK_MISMATCH,
        )

        for track in tracks:
            position = track.observed_position
            velocity = track.observed_velocity
            finite_track = all(
                isfinite(value)
                for value in (position.x, position.y, velocity.x, velocity.y)
            )
            _append_if(
                failures,
                not finite_track,
                DynamicObservationValidationReason.NON_FINITE_TRACK,
            )
            uncertainties = (track.position_sigma_m, track.velocity_sigma_mps)
            finite_uncertainty = all(isfinite(value) for value in uncertainties)
            _append_if(
                failures,
                not finite_uncertainty,
                DynamicObservationValidationReason.NON_FINITE_UNCERTAINTY,
            )
            if finite_uncertainty:
                _append_if(
                    failures,
                    min(uncertainties) < 0.0,
                    DynamicObservationValidationReason.NEGATIVE_UNCERTAINTY,
                )
                _append_if(
                    failures,
                    not isclose(
                        track.position_sigma_m,
                        self.profile.position_sigma_m,
                        rel_tol=0.0,
                        abs_tol=_TIME_ABS_TOLERANCE_S,
                    )
                    or not isclose(
                        track.velocity_sigma_mps,
                        self.profile.velocity_sigma_mps,
                        rel_tol=0.0,
                        abs_tol=_TIME_ABS_TOLERANCE_S,
                    ),
                    DynamicObservationValidationReason.UNCERTAINTY_PROFILE_MISMATCH,
                )
            previous_binding = self._bindings.get(track.track_id)
            _append_if(
                failures,
                previous_binding is not None and previous_binding != track.actor_binding_id,
                DynamicObservationValidationReason.ACTOR_BINDING_CHANGED,
            )

        content_hash = frame.content_hash
        hash_well_formed = (
            isinstance(content_hash, str)
            and len(content_hash) == _SHA256_HEX_LENGTH
            and content_hash == content_hash.lower()
            and all(character in "0123456789abcdef" for character in content_hash)
        )
        _append_if(
            failures,
            not hash_well_formed,
            DynamicObservationValidationReason.CONTENT_HASH_MALFORMED,
        )
        if hash_well_formed and timestamps_finite:
            try:
                expected_hash = dynamic_observation_content_hash(frame)
            except (TypeError, ValueError, AttributeError):
                expected_hash = None
            _append_if(
                failures,
                expected_hash is None or expected_hash != content_hash,
                DynamicObservationValidationReason.CONTENT_HASH_MISMATCH,
            )
        return _deduplicate(failures)


def _build_observation_frame(
    *,
    source: DynamicObservationSourceIdentity,
    map_revision: int,
    observation_revision: int,
    sequence: int,
    observed_at_s: float,
    delivered_at_s: float,
    tracks: tuple[ActorTrack, ...],
) -> DynamicObservationFrame:
    frame_kind = (
        DynamicObservationFrameKind.TRACKS if tracks else DynamicObservationFrameKind.EMPTY
    )
    payload = {
        "stream_id": source.stream_id,
        "episode_id": source.episode_id,
        "episode_seed": source.episode_seed,
        "map_id": source.map_id,
        "map_revision": map_revision,
        "observation_revision": observation_revision,
        "sequence": sequence,
        "observed_at_s": observed_at_s,
        "delivered_at_s": delivered_at_s,
        "frame_kind": frame_kind,
        "tracks": tracks,
    }
    return DynamicObservationFrame(
        **payload,
        content_hash=canonical_content_hash(payload),
    )


def _observed_track(
    actor: Any,
    *,
    source: DynamicObservationSourceIdentity,
    profile: DynamicObservationProfile,
    sequence: int,
) -> ActorTrack:
    actor_id = actor.actor_id
    position = Point2D(
        actor.position.x
        + profile.position_sigma_m
        * _normal_draw(source, sequence, actor_id, component="position", axis="x"),
        actor.position.y
        + profile.position_sigma_m
        * _normal_draw(source, sequence, actor_id, component="position", axis="y"),
    )
    velocity = Vector2D(
        actor.velocity.x
        + profile.velocity_sigma_mps
        * _normal_draw(source, sequence, actor_id, component="velocity", axis="x"),
        actor.velocity.y
        + profile.velocity_sigma_mps
        * _normal_draw(source, sequence, actor_id, component="velocity", axis="y"),
    )
    return ActorTrack(
        track_id=actor_id,
        actor_binding_id=actor_id,
        observed_position=position,
        observed_velocity=velocity,
        position_sigma_m=profile.position_sigma_m,
        velocity_sigma_mps=profile.velocity_sigma_mps,
    )


def _normal_draw(
    source: DynamicObservationSourceIdentity,
    sequence: int,
    actor_binding_id: str,
    *,
    component: str,
    axis: str,
) -> float:
    rng = Random(
        _derived_seed(
            source,
            sequence,
            component=component,
            actor_binding_id=actor_binding_id,
            axis=axis,
        )
    )
    return rng.gauss(0.0, 1.0)


def _uniform_draw(
    source: DynamicObservationSourceIdentity,
    sequence: int,
    *,
    component: str,
) -> float:
    return Random(_derived_seed(source, sequence, component=component)).random()


def _derived_seed(
    source: DynamicObservationSourceIdentity,
    sequence: int,
    *,
    component: str,
    actor_binding_id: str = "",
    axis: str = "",
) -> int:
    # Profile is deliberately absent: Normal and Stress share latent z/u draws.
    encoded = dumps(
        {
            "namespace": DYNAMIC_OBSERVATION_GENERATOR_VERSION,
            "episode_seed": source.episode_seed,
            "stream_id": source.stream_id,
            "sequence": sequence,
            "component": component,
            "actor_binding_id": actor_binding_id,
            "axis": axis,
        },
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return int.from_bytes(sha256(encoded).digest(), byteorder="big")


def _validate_truth_source(
    truth: DynamicGroundTruthFrame,
    source: DynamicObservationSourceIdentity,
) -> None:
    if truth.episode_id != source.episode_id or truth.seed != source.episode_seed:
        raise ValueError("ground truth episode identity does not match observation source")
    if truth.map_revision != source.map_revision:
        raise ValueError("ground truth map revision does not match observation source")


def _observation_payload(frame: DynamicObservationFrame) -> dict[str, object]:
    return {
        "stream_id": frame.stream_id,
        "episode_id": frame.episode_id,
        "episode_seed": frame.episode_seed,
        "map_id": frame.map_id,
        "map_revision": frame.map_revision,
        "observation_revision": frame.observation_revision,
        "sequence": frame.sequence,
        "observed_at_s": frame.observed_at_s,
        "delivered_at_s": frame.delivered_at_s,
        "frame_kind": frame.frame_kind,
        "tracks": frame.tracks,
    }


def _is_non_negative_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def dynamic_observation_age_is_stale(
    age_s: float,
    *,
    ttl_s: float = DYNAMIC_OBSERVATION_TTL_S,
) -> bool:
    """Apply the single Stage 2 TTL boundary: equality is fresh, greater is stale."""

    if not isfinite(age_s) or not isfinite(ttl_s):
        raise ValueError("observation age and TTL must be finite")
    if age_s < -_TIME_ABS_TOLERANCE_S or ttl_s < 0.0:
        raise ValueError("observation age and TTL must not be negative")
    return age_s > ttl_s + _TIME_ABS_TOLERANCE_S


def _append_if(
    failures: list[DynamicObservationValidationReason],
    condition: bool,
    reason: DynamicObservationValidationReason,
) -> None:
    if condition:
        failures.append(reason)


def _deduplicate(
    failures: list[DynamicObservationValidationReason],
) -> list[DynamicObservationValidationReason]:
    return list(dict.fromkeys(failures))
