"""Stage 5 공개 동적 Actor corpus와 contract-fault 목록.

expectation label은 evaluator와 corpus validator만 소유한다. controller용 paired 입력에는
label, split, oracle을 포함하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from enum import StrEnum
from hashlib import sha256
from math import atan2, ceil, cos, floor, hypot, isfinite, pi, sin, sqrt
from random import Random

import numpy as np

from hospital_path_lab.collision import (
    CollisionChecker,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.contracts import (
    GridSnapshot,
    Pose2D,
    RobotState,
    SnapshotMetadata,
    Twist2D,
)
from hospital_path_lab.dynamic_contracts import (
    ACTOR_RADIUS_M,
    DYNAMIC_CONTROL_PERIOD_S,
    MAX_ACTOR_ACCELERATION_MPS2,
    MAX_ACTOR_SPEED_MPS,
    ActorState,
    ActorTrack,
    ControllerSnapshot,
    DynamicGroundTruthFrame,
    DynamicObservationFrame,
    Point2D,
    Vector2D,
    build_controller_snapshot,
)
from hospital_path_lab.dynamic_observation import (
    NORMAL_OBSERVATION_PROFILE,
    STRESS_OBSERVATION_PROFILE,
    DynamicObservationProfile,
    DynamicObservationSlot,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    generate_dynamic_observation_slots,
)
from hospital_path_lab.dynamic_prediction import build_actor_prediction_set
from hospital_path_lab.grid import GridMap
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1

DYNAMIC_CORPUS_SCHEMA_VERSION = "1.0"
DYNAMIC_CORPUS_GENERATOR_VERSION = "dynamic_corpus_v1"
LEGACY_V1_PUBLIC_CORPUS_HASH = (
    "f7c7a5635458daad4233d8b2b067d27b014619a655a9f020e039ba77c4018abd"
)
DYNAMIC_V6_CORPUS_SCHEMA_VERSION = "2.0"
DYNAMIC_V6_CORPUS_GENERATOR_VERSION = "dynamic_corpus_v6_public_1"
DYNAMIC_V6_ORACLE_VERSION = "dynamic_category_oracle_v6_1"
DYNAMIC_V6_SEED_NAMESPACE = "hospital-path-lab/dynamic-v6/public/1"
_DURATION_S = 35.0
_MAP_LENGTH_M = 5.0
_GRID_RESOLUTION_M = 0.02
_MISSION_ID = "dynamic-stage5-mission"
_V6_DURATION_S = 45.0
_V6_WITNESS_EVALUATOR_PERIOD_S = 0.005
_V6_WITNESS_MAX_ANGULAR_ACCELERATION_RADPS2 = 1.60
_V6_PUBLIC_CASE_KEYS = (
    "same-direction-wide",
    "same-direction-narrow",
    "offset-head-on",
    "diagonal-crossing",
    "corner-intersection",
    "second-risk-intersection",
    "vertical-diagonal",
    "simultaneous-two-actor",
    "staggered-two-actor",
)


class DynamicCorpusSplit(StrEnum):
    GOLDEN = "golden"
    DEVELOPMENT = "development"
    HIDDEN = "hidden"


class DynamicExpectationCategory(StrEnum):
    WAIT_AND_RESUME = "wait_and_resume"
    LOCAL_DETOUR_FEASIBLE = "local_detour_feasible"
    LOCAL_DETOUR_FORBIDDEN = "local_detour_forbidden"
    NO_SAFE_SOLUTION = "no_safe_solution"
    OBSERVATION_INVALID = "observation_invalid"
    DYNAMIC_CHANGE_RESTOP = "dynamic_change_restop"


class DynamicScenarioFamily(StrEnum):
    """v6 evaluator 전용 시나리오 계열.

    이 값은 controller 입력으로 직렬화하지 않는다.
    """

    SAME_DIRECTION = "same_direction"
    HEAD_ON = "head_on"
    DIAGONAL_CROSSING = "diagonal_crossing"
    CORNER_INTERSECTION = "corner_intersection"
    VERTICAL_PATH = "vertical_path"
    MULTI_ACTOR = "multi_actor"


class DynamicScenarioOrientation(StrEnum):
    HORIZONTAL = "horizontal"
    VERTICAL = "vertical"
    DIAGONAL = "diagonal"
    CORNER = "corner"
    INTERSECTION = "intersection"


@dataclass(frozen=True, slots=True)
class DynamicAxisAlignedRegion:
    min_x_m: float
    min_y_m: float
    max_x_m: float
    max_y_m: float

    def __post_init__(self) -> None:
        values = (self.min_x_m, self.min_y_m, self.max_x_m, self.max_y_m)
        if not all(isfinite(value) for value in values):
            raise ValueError("static-layout region must be finite")
        if self.max_x_m <= self.min_x_m or self.max_y_m <= self.min_y_m:
            raise ValueError("static-layout region must have positive area")


@dataclass(frozen=True, slots=True)
class DynamicStaticLayoutSpec:
    occupied_regions: tuple[DynamicAxisAlignedRegion, ...] = ()
    forbidden_regions: tuple[DynamicAxisAlignedRegion, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "occupied_regions", tuple(self.occupied_regions))
        object.__setattr__(self, "forbidden_regions", tuple(self.forbidden_regions))


@dataclass(frozen=True, slots=True)
class DynamicFeasibleWitnessPoint:
    time_s: float
    pose: Pose2D
    twist: Twist2D = Twist2D()

    def __post_init__(self) -> None:
        if not all(
            isfinite(value)
            for value in (
                self.time_s,
                self.pose.x,
                self.pose.y,
                self.pose.yaw,
                self.twist.linear,
                self.twist.angular,
            )
        ):
            raise ValueError("feasible-witness point must be finite")
        if self.time_s < 0.0:
            raise ValueError("feasible-witness time must not be negative")


@dataclass(frozen=True, slots=True)
class DynamicFeasibleWitness:
    """controller와 분리된 evaluator-only 안전 우회 증인."""

    witness_id: str
    points: tuple[DynamicFeasibleWitnessPoint, ...]
    terminal_dwell_s: float = 0.50

    def __post_init__(self) -> None:
        if not self.witness_id:
            raise ValueError("feasible-witness id must not be empty")
        points = tuple(self.points)
        if len(points) < 3:
            raise ValueError("feasible witness requires at least three points")
        if any(
            right.time_s <= left.time_s
            for left, right in zip(points, points[1:], strict=False)
        ):
            raise ValueError("feasible-witness times must be strictly increasing")
        if self.terminal_dwell_s < 0.50:
            raise ValueError("terminal dwell must cover at least 0.5 s")
        object.__setattr__(self, "points", points)


@dataclass(frozen=True, slots=True)
class DynamicOracleSpec:
    """evaluator 전용 category oracle. controller 경계 밖에만 존재한다."""

    oracle_version: str
    expectation_category: DynamicExpectationCategory
    hazard_intervals_s: tuple[tuple[float, float], ...]
    same_direction_actor_ids: tuple[str, ...] = ()
    required_protective_stop_epochs: int = 0
    feasible_witness: DynamicFeasibleWitness | None = None
    departure_threshold_m: float = 0.10
    rejoin_distance_m: float = 0.10
    rejoin_heading_tolerance_deg: float = 10.0
    rejoin_hold_s: float = 0.50

    def __post_init__(self) -> None:
        if self.oracle_version != DYNAMIC_V6_ORACLE_VERSION:
            raise ValueError("unsupported dynamic v6 oracle version")
        intervals = tuple(tuple(interval) for interval in self.hazard_intervals_s)
        if any(
            len(interval) != 2
            or not all(isfinite(value) for value in interval)
            or interval[0] < 0.0
            or interval[1] <= interval[0]
            for interval in intervals
        ):
            raise ValueError("hazard intervals must be finite, positive and ordered")
        if self.required_protective_stop_epochs < 0:
            raise ValueError("required stop epochs must not be negative")
        if (
            self.departure_threshold_m <= 0.0
            or self.rejoin_distance_m <= 0.0
            or self.rejoin_heading_tolerance_deg <= 0.0
            or self.rejoin_hold_s < 0.50
        ):
            raise ValueError("rejoin oracle thresholds must remain positive")
        actor_ids = tuple(self.same_direction_actor_ids)
        if len(actor_ids) != len(set(actor_ids)):
            raise ValueError("same-direction actor ids must be unique")
        if (
            self.expectation_category
            is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
            and self.feasible_witness is None
        ):
            raise ValueError("feasible category requires an evaluator-only witness")
        object.__setattr__(self, "hazard_intervals_s", intervals)
        object.__setattr__(self, "same_direction_actor_ids", actor_ids)


class DynamicContractFaultDomain(StrEnum):
    OBSERVATION = "observation"
    AUTHORITY = "authority"
    DEADLINE = "deadline"


class DynamicContractFaultResponse(StrEnum):
    BRAKE_AND_HOLD = "brake_and_hold"
    CONTINUE_WITH_FRESH_EMPTY = "continue_with_fresh_empty"
    HOLD_LAST_VALID_FRAME = "hold_last_valid_frame"
    ACCEPT_TTL_BOUNDARY = "accept_ttl_boundary"
    ACCEPT_CURRENT_TICK = "accept_current_tick"
    REJECT_AUTHORIZATION = "reject_authorization"
    DISCARD_RESULT = "discard_result"


@dataclass(frozen=True, slots=True)
class DynamicCorpusActor:
    actor_id: str
    active_from_s: float
    active_until_s: float
    start_position: Point2D
    velocity: Vector2D
    radius_m: float = ACTOR_RADIUS_M
    trajectory_revision: int = 1

    def __post_init__(self) -> None:
        if not self.actor_id:
            raise ValueError("actor_id must not be empty")
        if not all(isfinite(value) for value in (self.active_from_s, self.active_until_s)):
            raise ValueError("actor active interval must be finite")
        if self.active_from_s < 0.0 or self.active_until_s <= self.active_from_s:
            raise ValueError("actor active interval must be positive and ordered")
        if self.radius_m != ACTOR_RADIUS_M:
            raise ValueError("dynamic corpus actor radius is frozen at 0.18 m")
        if self.velocity.magnitude > MAX_ACTOR_SPEED_MPS + 1e-12:
            raise ValueError("actor velocity exceeds frozen maximum")

    def state_at(self, simulation_time_s: float) -> ActorState | None:
        if not self.active_from_s <= simulation_time_s <= self.active_until_s:
            return None
        elapsed_s = simulation_time_s - self.active_from_s
        return ActorState(
            actor_id=self.actor_id,
            position=Point2D(
                self.start_position.x + self.velocity.x * elapsed_s,
                self.start_position.y + self.velocity.y * elapsed_s,
            ),
            velocity=self.velocity,
            radius_m=self.radius_m,
            trajectory_revision=self.trajectory_revision,
        )


@dataclass(frozen=True, slots=True)
class DynamicCorpusEpisode:
    schema_version: str
    generator_version: str
    episode_id: str
    split: DynamicCorpusSplit
    expectation_category: DynamicExpectationCategory
    seed: int
    simulation_only: bool
    map_id: str
    mission_id: str
    duration_s: float
    corridor_width_m: float
    map_length_m: float
    grid_resolution_m: float
    initial_state: RobotState
    goal_pose: Pose2D
    reference_path: tuple[Pose2D, ...]
    actors: tuple[DynamicCorpusActor, ...]
    progressable: bool
    blocking_cleared_at_s: float | None
    observation_fault: str | None = None

    def __post_init__(self) -> None:
        if not all(
            (self.schema_version, self.generator_version, self.episode_id, self.map_id)
        ):
            raise ValueError("episode identity fields must not be empty")
        if not self.simulation_only:
            raise ValueError("dynamic corpus must remain simulation_only")
        if self.mission_id != _MISSION_ID:
            raise ValueError("dynamic corpus mission contract changed")
        if self.duration_s <= 0.0 or self.corridor_width_m <= 0.0:
            raise ValueError("episode geometry and duration must be positive")
        if abs(self.duration_s / DYNAMIC_CONTROL_PERIOD_S - round(
            self.duration_s / DYNAMIC_CONTROL_PERIOD_S
        )) > 1e-12:
            raise ValueError("episode duration must align with 20 Hz")
        if self.grid_resolution_m != _GRID_RESOLUTION_M:
            raise ValueError("dynamic corpus grid resolution is frozen at 0.02 m")
        if len(self.reference_path) < 2:
            raise ValueError("reference_path must contain at least two poses")
        object.__setattr__(self, "reference_path", tuple(self.reference_path))
        object.__setattr__(self, "actors", tuple(self.actors))

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)

    @property
    def tick_count(self) -> int:
        return round(self.duration_s / DYNAMIC_CONTROL_PERIOD_S)

    def actor_states_at(self, simulation_time_s: float) -> tuple[ActorState, ...]:
        if (
            not isfinite(simulation_time_s)
            or simulation_time_s < 0.0
            or simulation_time_s > self.duration_s + 1e-9
        ):
            raise ValueError("ground-truth query time is outside the episode")
        simulation_time_s = min(simulation_time_s, self.duration_s)
        states = tuple(actor.state_at(simulation_time_s) for actor in self.actors)
        return tuple(state for state in states if state is not None)


@dataclass(frozen=True, slots=True, kw_only=True)
class V6DynamicCorpusEpisode(DynamicCorpusEpisode):
    """legacy-v1 content hash와 분리된 v6 공개 episode schema."""

    scenario_family: DynamicScenarioFamily
    variant: str
    orientation: DynamicScenarioOrientation
    latent_case_id: str
    static_layout_spec: DynamicStaticLayoutSpec
    oracle_spec: DynamicOracleSpec
    semantic_world_hash: str
    oracle_hash: str

    def __post_init__(self) -> None:
        super(V6DynamicCorpusEpisode, self).__post_init__()
        if self.schema_version != DYNAMIC_V6_CORPUS_SCHEMA_VERSION:
            raise ValueError("v6 episode schema version mismatch")
        if self.generator_version != DYNAMIC_V6_CORPUS_GENERATOR_VERSION:
            raise ValueError("v6 episode generator version mismatch")
        if not all(
            (
                self.variant,
                self.latent_case_id,
                self.semantic_world_hash,
                self.oracle_hash,
            )
        ):
            raise ValueError("v6 evaluator metadata must not be empty")
        if self.oracle_spec.expectation_category is not self.expectation_category:
            raise ValueError("episode category and oracle category must match")


@dataclass(frozen=True, slots=True)
class DynamicControllerCorpusInput:
    """PP와 DWA에 똑같이 전달되는 label-free 공개 입력."""

    episode_id: str
    seed: int
    mission_id: str
    initial_state: RobotState
    goal_pose: Pose2D
    reference_path: tuple[Pose2D, ...]
    grid_snapshot: GridSnapshot
    observation_slots: tuple[DynamicObservationSlot, ...]
    observation_stream_hash: str


@dataclass(frozen=True, slots=True)
class DynamicContractFaultCase:
    case_id: str
    domain: DynamicContractFaultDomain
    injected_fault: str
    expected_response: DynamicContractFaultResponse


@dataclass(frozen=True, slots=True)
class DynamicCorpusValidation:
    passed: bool
    golden_count: int
    development_count: int
    category_counts: tuple[tuple[str, int], ...]
    failures: tuple[str, ...]
    corpus_content_hash: str


@dataclass(frozen=True, slots=True)
class DynamicHiddenCorpusValidation:
    passed: bool
    hidden_count: int
    category_counts: tuple[tuple[str, int], ...]
    failures: tuple[str, ...]
    corpus_content_hash: str


@dataclass(frozen=True, slots=True)
class DynamicV6CorpusValidation:
    passed: bool
    episode_count: int
    golden_count: int
    development_count: int
    family_counts: tuple[tuple[str, int], ...]
    orientation_counts: tuple[tuple[str, int], ...]
    failures: tuple[str, ...]
    corpus_content_hash: str
    semantic_world_set_hash: str


def generate_dynamic_corpus(*, base_seed: int = 20260811) -> tuple[DynamicCorpusEpisode, ...]:
    episodes: list[DynamicCorpusEpisode] = []
    categories = tuple(DynamicExpectationCategory)
    for index, category in enumerate(categories):
        episodes.append(
            _generate_episode(
                seed=base_seed + index,
                split=DynamicCorpusSplit.GOLDEN,
                category=category,
                replica=0,
            )
        )
    for category_index, category in enumerate(categories):
        for replica in range(5):
            episodes.append(
                _generate_episode(
                    seed=base_seed + 100 + category_index * 10 + replica,
                    split=DynamicCorpusSplit.DEVELOPMENT,
                    category=category,
                    replica=replica,
                )
            )
    return tuple(episodes)


def generate_dynamic_v6_public_corpus(
    *,
    base_seed: int = 20260811,
) -> tuple[V6DynamicCorpusEpisode, ...]:
    """legacy-v1 lane과 분리된 v6 공개 시나리오를 생성한다.

    seed는 template의 안정된 namespace key에서 유도하므로 template 순서 변경이나 다른
    family 추가가 기존 episode의 외생 입력을 바꾸지 않는다.
    """

    episodes: list[V6DynamicCorpusEpisode] = []
    for case_key in _V6_PUBLIC_CASE_KEYS:
        replica_count = 5 if case_key == "same-direction-wide" else 1
        for replica in range(replica_count):
            split = (
                DynamicCorpusSplit.GOLDEN
                if replica == 0
                else DynamicCorpusSplit.DEVELOPMENT
            )
            seed_case_key = (
                "diagonal-rigid-pair"
                if case_key in {"diagonal-crossing", "vertical-diagonal"}
                else case_key
            )
            namespace_key = f"{seed_case_key}/{split.value}/{replica:02d}"
            episodes.append(
                _generate_v6_public_episode(
                    case_key=case_key,
                    split=split,
                    replica=replica,
                    seed=_derive_v6_seed(base_seed, namespace_key),
                )
            )
    return tuple(episodes)


def hidden_seed_commitment(hidden_seed: int) -> str:
    """seed를 공개하지 않고 freeze 전에 고정하기 위한 SHA-256 commitment."""

    payload = f"dynamic-hidden-v1:{hidden_seed}".encode()
    return sha256(payload).hexdigest()


def generate_dynamic_hidden_corpus(
    *,
    hidden_seed: int,
    expected_commitment: str,
) -> tuple[DynamicCorpusEpisode, ...]:
    if hidden_seed_commitment(hidden_seed) != expected_commitment:
        raise ValueError("hidden seed commitment mismatch")
    episodes = tuple(
        _generate_episode(
            seed=hidden_seed + category_index * 10 + replica,
            split=DynamicCorpusSplit.HIDDEN,
            category=category,
            replica=replica,
        )
        for category_index, category in enumerate(DynamicExpectationCategory)
        for replica in range(5)
    )
    return episodes


def validate_dynamic_hidden_corpus(
    episodes: tuple[DynamicCorpusEpisode, ...],
    *,
    public_corpus: tuple[DynamicCorpusEpisode, ...],
) -> DynamicHiddenCorpusValidation:
    failures: list[str] = []
    if len(episodes) != 30 or any(
        episode.split is not DynamicCorpusSplit.HIDDEN for episode in episodes
    ):
        failures.append("hidden_count_or_split_mismatch")
    if len({episode.episode_id for episode in episodes}) != len(episodes):
        failures.append("duplicate_hidden_episode_id")
    if len({episode.content_hash for episode in episodes}) != len(episodes):
        failures.append("duplicate_hidden_episode_hash")
    public_hashes = {episode.content_hash for episode in public_corpus}
    if public_hashes.intersection(episode.content_hash for episode in episodes):
        failures.append("public_hidden_hash_overlap")
    category_counts = {
        category.value: sum(item.expectation_category is category for item in episodes)
        for category in DynamicExpectationCategory
    }
    if any(count != 5 for count in category_counts.values()):
        failures.append("hidden_category_balance_mismatch")
    for episode in episodes:
        failures.extend(_episode_validation_failures(episode))
    return DynamicHiddenCorpusValidation(
        passed=not failures,
        hidden_count=len(episodes),
        category_counts=tuple(sorted(category_counts.items())),
        failures=tuple(failures),
        corpus_content_hash=canonical_content_hash(episodes),
    )


def build_dynamic_grid_snapshot(
    episode: DynamicCorpusEpisode,
    *,
    observation_revision: int = 0,
) -> GridSnapshot:
    if isinstance(episode, V6DynamicCorpusEpisode):
        width_cells = ceil(episode.map_length_m / episode.grid_resolution_m)
        height_cells = ceil(episode.corridor_width_m / episode.grid_resolution_m)
    else:
        width_cells = round(episode.map_length_m / episode.grid_resolution_m)
        height_cells = round(episode.corridor_width_m / episode.grid_resolution_m)
    occupancy = np.zeros((height_cells, width_cells), dtype=np.bool_)
    forbidden_cells: set[tuple[int, int]] = set()
    if isinstance(episode, V6DynamicCorpusEpisode):
        _rasterize_regions(
            occupancy,
            episode.static_layout_spec.occupied_regions,
            resolution_m=episode.grid_resolution_m,
        )
        forbidden_mask = np.zeros_like(occupancy)
        _rasterize_regions(
            forbidden_mask,
            episode.static_layout_spec.forbidden_regions,
            resolution_m=episode.grid_resolution_m,
        )
        occupied_y, occupied_x = np.nonzero(forbidden_mask)
        forbidden_cells.update(
            (int(x), int(y)) for y, x in zip(occupied_y, occupied_x, strict=True)
        )
    content_payload: dict[str, object]
    if isinstance(episode, V6DynamicCorpusEpisode):
        content_payload = {
            "world_hash": episode.semantic_world_hash,
            "map_id": episode.map_id,
            "map_revision": 1,
            "observation_revision": observation_revision,
            "grid_shape": (height_cells, width_cells),
            "grid_resolution_m": episode.grid_resolution_m,
            "occupancy_sha256": sha256(occupancy.tobytes()).hexdigest(),
            "forbidden_cells": tuple(sorted(forbidden_cells)),
        }
    else:
        content_payload = {
            "episode_hash": episode.content_hash,
            "map_id": episode.map_id,
            "map_revision": 1,
            "observation_revision": observation_revision,
        }
    return GridSnapshot(
        metadata=SnapshotMetadata(
            map_id=episode.map_id,
            map_revision=1,
            mission_revision=1,
            observation_revision=observation_revision,
            seed=episode.seed,
            content_hash=canonical_content_hash(content_payload),
        ),
        grid=GridMap(occupancy, resolution_m=episode.grid_resolution_m),
        forbidden_cells=frozenset(forbidden_cells),
    )


def generate_episode_ground_truth_frames(
    episode: DynamicCorpusEpisode,
) -> tuple[DynamicGroundTruthFrame, ...]:
    return tuple(
        DynamicGroundTruthFrame(
            episode_id=controller_episode_id(episode),
            seed=episode.seed,
            tick_id=tick_id,
            simulation_time_s=tick_id * DYNAMIC_CONTROL_PERIOD_S,
            robot_state=episode.initial_state,
            actors=episode.actor_states_at(tick_id * DYNAMIC_CONTROL_PERIOD_S),
            map_revision=1,
            mission_revision=1,
        )
        for tick_id in range(episode.tick_count + 1)
    )


def generate_episode_observation_slots(
    episode: DynamicCorpusEpisode,
    *,
    profile: DynamicObservationProfile,
) -> tuple[DynamicObservationSlot, ...]:
    slots = generate_dynamic_observation_slots(
        generate_episode_ground_truth_frames(episode),
        source=DynamicObservationSourceIdentity(
            stream_id="dynamic-stage5-stream",
            episode_id=controller_episode_id(episode),
            episode_seed=episode.seed,
            map_id=episode.map_id,
            map_revision=1,
        ),
        profile=profile,
    )
    if (
        isinstance(episode, V6DynamicCorpusEpisode)
        and episode.latent_case_id == "diagonal-rigid-pair-v6"
        and episode.orientation is DynamicScenarioOrientation.VERTICAL
    ):
        return _rotate_rigid_pair_observation_noise(episode, slots)
    return slots


def _rotate_rigid_pair_observation_noise(
    episode: V6DynamicCorpusEpisode,
    slots: tuple[DynamicObservationSlot, ...],
) -> tuple[DynamicObservationSlot, ...]:
    rotated: list[DynamicObservationSlot] = []
    for slot in slots:
        frame = slot.frame
        if frame is None:
            rotated.append(slot)
            continue
        truth_by_id = {
            actor.actor_id: actor
            for actor in episode.actor_states_at(frame.observed_at_s)
        }
        tracks: list[ActorTrack] = []
        for track in frame.tracks:
            truth = truth_by_id.get(track.actor_binding_id)
            if truth is None:
                raise ValueError("rigid-pair observation track has no ground truth binding")
            position_noise_x = track.observed_position.x - truth.position.x
            position_noise_y = track.observed_position.y - truth.position.y
            velocity_noise_x = track.observed_velocity.x - truth.velocity.x
            velocity_noise_y = track.observed_velocity.y - truth.velocity.y
            tracks.append(
                replace(
                    track,
                    observed_position=Point2D(
                        truth.position.x - position_noise_y,
                        truth.position.y + position_noise_x,
                    ),
                    observed_velocity=Vector2D(
                        truth.velocity.x - velocity_noise_y,
                        truth.velocity.y + velocity_noise_x,
                    ),
                )
            )
        rotated_frame = _replace_observation_frame_tracks(frame, tuple(tracks))
        rotated.append(replace(slot, frame=rotated_frame))
    return tuple(rotated)


def _replace_observation_frame_tracks(
    frame: DynamicObservationFrame,
    tracks: tuple[ActorTrack, ...],
) -> DynamicObservationFrame:
    payload = {
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
        "tracks": tracks,
    }
    return DynamicObservationFrame(
        **payload,
        content_hash=canonical_content_hash(payload),
    )


def paired_controller_inputs(
    episode: DynamicCorpusEpisode,
    *,
    profile: DynamicObservationProfile = NORMAL_OBSERVATION_PROFILE,
) -> tuple[DynamicControllerCorpusInput, DynamicControllerCorpusInput]:
    slots = generate_episode_observation_slots(episode, profile=profile)
    stream_hash = canonical_content_hash(slots)
    shared = DynamicControllerCorpusInput(
        episode_id=controller_episode_id(episode),
        seed=episode.seed,
        mission_id=episode.mission_id,
        initial_state=episode.initial_state,
        goal_pose=episode.goal_pose,
        reference_path=episode.reference_path,
        grid_snapshot=build_dynamic_grid_snapshot(episode),
        observation_slots=slots,
        observation_stream_hash=stream_hash,
    )
    return shared, shared


def paired_controller_snapshots(
    episode: DynamicCorpusEpisode,
    *,
    profile: DynamicObservationProfile = NORMAL_OBSERVATION_PROFILE,
) -> tuple[ControllerSnapshot, ControllerSnapshot]:
    """공개 stream의 첫 유효 frame을 같은 PP/DWA snapshot으로 만든다."""

    source = DynamicObservationSourceIdentity(
        stream_id="dynamic-stage5-stream",
        episode_id=controller_episode_id(episode),
        episode_seed=episode.seed,
        map_id=episode.map_id,
        map_revision=1,
    )
    slots = generate_episode_observation_slots(episode, profile=profile)
    slot = next((item for item in slots if item.frame is not None), None)
    if slot is None or slot.frame is None:
        raise ValueError("paired corpus requires at least one delivered observation frame")
    validator = DynamicObservationValidator(source, profile)
    validation = validator.accept(
        slot.frame,
        received_at_s=slot.scheduled_delivery_at_s,
    )
    if not validation.accepted:
        raise ValueError(f"generated observation did not validate: {validation.failures}")
    observation = validator.snapshot(control_time_s=slot.scheduled_delivery_at_s)
    grid = build_dynamic_grid_snapshot(
        episode,
        observation_revision=slot.frame.observation_revision,
    )
    shared = build_controller_snapshot(
        tick_id=round(slot.scheduled_delivery_at_s / DYNAMIC_CONTROL_PERIOD_S),
        simulation_time_s=slot.scheduled_delivery_at_s,
        mission_id=episode.mission_id,
        robot_state=episode.initial_state,
        goal_pose=episode.goal_pose,
        reference_path=episode.reference_path,
        static_grid_snapshot=grid,
        validated_observation=observation,
        actor_tubes=build_actor_prediction_set(observation),
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    )
    return shared, shared


def validate_dynamic_corpus(
    episodes: tuple[DynamicCorpusEpisode, ...],
) -> DynamicCorpusValidation:
    failures: list[str] = []
    golden = tuple(item for item in episodes if item.split is DynamicCorpusSplit.GOLDEN)
    development = tuple(
        item for item in episodes if item.split is DynamicCorpusSplit.DEVELOPMENT
    )
    if len(golden) != 6:
        failures.append("golden_count_mismatch")
    if len(development) != 30:
        failures.append("development_count_mismatch")
    if len({item.episode_id for item in episodes}) != len(episodes):
        failures.append("duplicate_episode_id")
    if len({item.content_hash for item in episodes}) != len(episodes):
        failures.append("duplicate_episode_hash")

    category_counts = {
        category.value: sum(item.expectation_category is category for item in episodes)
        for category in DynamicExpectationCategory
    }
    if any(count != 6 for count in category_counts.values()):
        failures.append("category_balance_mismatch")

    for episode in episodes:
        failures.extend(_episode_validation_failures(episode))
        first_normal = generate_episode_observation_slots(
            episode,
            profile=NORMAL_OBSERVATION_PROFILE,
        )
        second_normal = generate_episode_observation_slots(
            episode,
            profile=NORMAL_OBSERVATION_PROFILE,
        )
        first_stress = generate_episode_observation_slots(
            episode,
            profile=STRESS_OBSERVATION_PROFILE,
        )
        second_stress = generate_episode_observation_slots(
            episode,
            profile=STRESS_OBSERVATION_PROFILE,
        )
        if canonical_content_hash(first_normal) != canonical_content_hash(second_normal):
            failures.append(f"{episode.episode_id}:normal_stream_nondeterministic")
        if canonical_content_hash(first_stress) != canonical_content_hash(second_stress):
            failures.append(f"{episode.episode_id}:stress_stream_nondeterministic")
        pp_input, dwa_input = paired_controller_inputs(episode)
        if pp_input != dwa_input:
            failures.append(f"{episode.episode_id}:paired_input_mismatch")

    return DynamicCorpusValidation(
        passed=not failures,
        golden_count=len(golden),
        development_count=len(development),
        category_counts=tuple(sorted(category_counts.items())),
        failures=tuple(failures),
        corpus_content_hash=canonical_content_hash(episodes),
    )


def validate_dynamic_v6_public_corpus(
    episodes: tuple[V6DynamicCorpusEpisode, ...],
) -> DynamicV6CorpusValidation:
    """v6 공개 schema, 물리 world, oracle와 누출 경계를 독립 검증한다."""

    failures: list[str] = []
    golden = tuple(item for item in episodes if item.split is DynamicCorpusSplit.GOLDEN)
    development = tuple(
        item for item in episodes if item.split is DynamicCorpusSplit.DEVELOPMENT
    )
    if len(episodes) != 13 or len(golden) != 9 or len(development) != 4:
        failures.append("v6_public_count_or_split_mismatch")
    if any(not isinstance(item, V6DynamicCorpusEpisode) for item in episodes):
        failures.append("v6_episode_type_mismatch")
    if len({item.episode_id for item in episodes}) != len(episodes):
        failures.append("duplicate_v6_episode_id")
    if len({item.content_hash for item in episodes}) != len(episodes):
        failures.append("duplicate_v6_episode_hash")
    if len({item.semantic_world_hash for item in episodes}) != len(episodes):
        failures.append("duplicate_v6_semantic_world")

    required_families = set(DynamicScenarioFamily)
    present_families = {item.scenario_family for item in episodes}
    if present_families != required_families:
        failures.append("v6_scenario_family_matrix_incomplete")
    required_orientations = set(DynamicScenarioOrientation)
    present_orientations = {item.orientation for item in episodes}
    if present_orientations != required_orientations:
        failures.append("v6_orientation_matrix_incomplete")

    for episode in episodes:
        prefix = f"{episode.episode_id}:"
        failures.extend(_episode_validation_failures(episode))
        failures.extend(_v6_episode_validation_failures(episode))
        if any(
            abs(dimension_m / episode.grid_resolution_m - round(
                dimension_m / episode.grid_resolution_m
            ))
            > 1e-9
            for dimension_m in (episode.map_length_m, episode.corridor_width_m)
        ):
            failures.append(f"{prefix}grid_dimension_not_resolution_aligned")
        if episode.semantic_world_hash != _semantic_world_hash(episode):
            failures.append(f"{prefix}semantic_world_hash_mismatch")
        if episode.oracle_hash != canonical_content_hash(episode.oracle_spec):
            failures.append(f"{prefix}oracle_hash_mismatch")
        pp_input, dwa_input = paired_controller_inputs(episode)
        if pp_input is not dwa_input:
            failures.append(f"{prefix}paired_input_not_shared")
        controller_fields = {field.name for field in fields(pp_input)}
        if controller_fields.intersection(
            {
                "split",
                "expectation_category",
                "scenario_family",
                "variant",
                "orientation",
                "latent_case_id",
                "static_layout_spec",
                "oracle_spec",
                "semantic_world_hash",
                "oracle_hash",
            }
        ):
            failures.append(f"{prefix}evaluator_label_leaked_to_controller")

    diagonal_pair = tuple(
        item for item in episodes if item.latent_case_id == "diagonal-rigid-pair-v6"
    )
    if len(diagonal_pair) != 2 or {
        item.orientation for item in diagonal_pair
    } != {DynamicScenarioOrientation.DIAGONAL, DynamicScenarioOrientation.VERTICAL}:
        failures.append("rigid_transform_pair_missing")
    else:
        horizontal = next(
            item
            for item in diagonal_pair
            if item.orientation is DynamicScenarioOrientation.DIAGONAL
        )
        vertical = next(
            item
            for item in diagonal_pair
            if item.orientation is DynamicScenarioOrientation.VERTICAL
        )
        if not _is_exact_rigid_rotation_pair(horizontal, vertical):
            failures.append("rigid_transform_geometry_mismatch")

    family_counts = {
        family.value: sum(item.scenario_family is family for item in episodes)
        for family in DynamicScenarioFamily
    }
    orientation_counts = {
        orientation.value: sum(item.orientation is orientation for item in episodes)
        for orientation in DynamicScenarioOrientation
    }
    return DynamicV6CorpusValidation(
        passed=not failures,
        episode_count=len(episodes),
        golden_count=len(golden),
        development_count=len(development),
        family_counts=tuple(sorted(family_counts.items())),
        orientation_counts=tuple(sorted(orientation_counts.items())),
        failures=tuple(failures),
        corpus_content_hash=canonical_content_hash(episodes),
        semantic_world_set_hash=canonical_content_hash(
            tuple(sorted(item.semantic_world_hash for item in episodes))
        ),
    )


def dynamic_contract_fault_cases() -> tuple[DynamicContractFaultCase, ...]:
    observation_faults = (
        "stream_id_mismatch",
        "episode_seed_mismatch",
        "map_id_mismatch",
        "sequence_regressed",
        "revision_regressed",
        "content_hash_mismatch",
        "duplicate_track_id",
        "actor_binding_changed",
        "fresh_empty_frame",
        "single_dropout",
        "four_frame_burst_dropout",
        "age_equal_ttl",
        "age_greater_than_ttl",
    )
    authority_faults = (
        "previous_stop_epoch",
        "different_mission_id",
        "authorization_before_stop_confirmation",
        "authorization_revision_mismatch",
        "authorization_missing",
        "new_stop_after_authorization",
        "hazard_clear_without_authorization",
    )
    deadline_faults = (
        "result_49ms",
        "result_50ms",
        "result_51ms",
        "late_result_next_tick",
        "late_and_current_result_reordered",
    )
    cases = [
        DynamicContractFaultCase(
            f"observation-{index:02d}",
            DynamicContractFaultDomain.OBSERVATION,
            name,
            {
                "fresh_empty_frame": (
                    DynamicContractFaultResponse.CONTINUE_WITH_FRESH_EMPTY
                ),
                "single_dropout": DynamicContractFaultResponse.HOLD_LAST_VALID_FRAME,
                "age_equal_ttl": DynamicContractFaultResponse.ACCEPT_TTL_BOUNDARY,
            }.get(name, DynamicContractFaultResponse.BRAKE_AND_HOLD),
        )
        for index, name in enumerate(observation_faults, start=1)
    ]
    cases.extend(
        DynamicContractFaultCase(
            f"authority-{index:02d}",
            DynamicContractFaultDomain.AUTHORITY,
            name,
            DynamicContractFaultResponse.REJECT_AUTHORIZATION,
        )
        for index, name in enumerate(authority_faults, start=1)
    )
    cases.extend(
        DynamicContractFaultCase(
            f"deadline-{index:02d}",
            DynamicContractFaultDomain.DEADLINE,
            name,
            (
                DynamicContractFaultResponse.ACCEPT_CURRENT_TICK
                if name in {"result_49ms", "result_50ms"}
                else DynamicContractFaultResponse.DISCARD_RESULT
            ),
        )
        for index, name in enumerate(deadline_faults, start=1)
    )
    return tuple(cases)


def _derive_v6_seed(base_seed: int, namespace_key: str) -> int:
    if not namespace_key:
        raise ValueError("v6 seed namespace key must not be empty")
    payload = f"{DYNAMIC_V6_SEED_NAMESPACE}:{base_seed}:{namespace_key}".encode()
    return int.from_bytes(sha256(payload).digest()[:8], "big") & 0x7FFF_FFFF_FFFF_FFFF


def _generate_v6_public_episode(
    *,
    case_key: str,
    split: DynamicCorpusSplit,
    replica: int,
    seed: int,
) -> V6DynamicCorpusEpisode:
    rng = Random(seed)
    empty_layout = DynamicStaticLayoutSpec()
    duration_s = _V6_DURATION_S
    map_length_m = 5.0
    corridor_width_m = 2.4
    center_y = corridor_width_m / 2.0
    initial_state = RobotState(Pose2D(0.60, center_y, 0.0))
    goal_pose = Pose2D(4.40, center_y, 0.0)
    reference_path = (initial_state.pose, goal_pose)
    layout = empty_layout
    observation_fault: str | None = None
    progressable = True
    blocking_cleared_at_s: float | None
    witness: DynamicFeasibleWitness | None = None

    if case_key == "same-direction-wide":
        corridor_width_m = _grid_aligned_v6_dimension(
            4.62 + rng.uniform(-0.04, 0.04)
        )
        center_y = corridor_width_m / 2.0
        speed = round(rng.uniform(0.06, 0.07), 3)
        actor_start_x = round(1.50 + rng.uniform(0.0, 0.04), 3)
        initial_state = RobotState(Pose2D(0.60, center_y, 0.0))
        actors = (
            DynamicCorpusActor(
                "same-direction-lead",
                0.0,
                min(30.0, (map_length_m - ACTOR_RADIUS_M - actor_start_x) / speed),
                Point2D(actor_start_x, center_y),
                Vector2D(speed, 0.0),
            ),
        )
        witness = _same_direction_feasible_witness(center_y)
        goal_pose = witness.points[-1].pose
        reference_path = (initial_state.pose, goal_pose)
        category = DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
        family = DynamicScenarioFamily.SAME_DIRECTION
        orientation = DynamicScenarioOrientation.HORIZONTAL
        variant = f"wide-feasible-r{replica:02d}"
        latent_case_id = f"same-direction-wide-r{replica:02d}"
        blocking_cleared_at_s = actors[0].active_until_s
        same_direction_ids = (actors[0].actor_id,)
        required_stops = 1
    elif case_key == "same-direction-narrow":
        corridor_width_m = 0.92
        center_y = corridor_width_m / 2.0
        initial_state = RobotState(Pose2D(0.60, center_y, 0.0))
        goal_pose = Pose2D(4.40, center_y, 0.0)
        reference_path = (initial_state.pose, goal_pose)
        actors = (
            DynamicCorpusActor(
                "same-direction-blocking",
                0.0,
                22.0,
                Point2D(1.50, center_y),
                Vector2D(0.10, 0.0),
            ),
        )
        category = DynamicExpectationCategory.LOCAL_DETOUR_FORBIDDEN
        family = DynamicScenarioFamily.SAME_DIRECTION
        orientation = DynamicScenarioOrientation.HORIZONTAL
        variant = "narrow-forbidden"
        latent_case_id = "same-direction-narrow-v6"
        blocking_cleared_at_s = actors[0].active_until_s
        same_direction_ids = (actors[0].actor_id,)
        required_stops = 0
    elif case_key == "offset-head-on":
        corridor_width_m = 2.4
        robot_y = 0.98
        actor_y = robot_y + 0.44
        initial_state = RobotState(Pose2D(0.60, robot_y, 0.0))
        goal_pose = Pose2D(4.40, robot_y, 0.0)
        reference_path = (initial_state.pose, goal_pose)
        actors = (
            DynamicCorpusActor(
                "offset-counterflow",
                0.0,
                18.0,
                Point2D(4.10, actor_y),
                Vector2D(-0.18, 0.0),
            ),
        )
        category = DynamicExpectationCategory.WAIT_AND_RESUME
        family = DynamicScenarioFamily.HEAD_ON
        orientation = DynamicScenarioOrientation.HORIZONTAL
        variant = "offset-counterflow"
        latent_case_id = "offset-head-on-v6"
        blocking_cleared_at_s = actors[0].active_until_s
        same_direction_ids = ()
        required_stops = 1
    elif case_key in {"diagonal-crossing", "vertical-diagonal"}:
        map_length_m = 4.8
        corridor_width_m = 4.8
        horizontal_start = Pose2D(0.60, 2.40, 0.0)
        horizontal_goal = Pose2D(4.20, 2.40, 0.0)
        horizontal_actor = DynamicCorpusActor(
            "diagonal-crossing",
            0.0,
            8.0,
            Point2D(2.00, 0.30),
            Vector2D(0.08, 0.28),
        )
        if case_key == "diagonal-crossing":
            initial_state = RobotState(horizontal_start)
            goal_pose = horizontal_goal
            reference_path = (horizontal_start, horizontal_goal)
            actors = (horizontal_actor,)
            family = DynamicScenarioFamily.DIAGONAL_CROSSING
            orientation = DynamicScenarioOrientation.DIAGONAL
            variant = "xy-nonzero"
        else:
            initial_state = RobotState(_rotate_pose_90(horizontal_start, map_length_m))
            goal_pose = _rotate_pose_90(horizontal_goal, map_length_m)
            reference_path = (initial_state.pose, goal_pose)
            actors = (_rotate_actor_90(horizontal_actor, map_length_m),)
            family = DynamicScenarioFamily.VERTICAL_PATH
            orientation = DynamicScenarioOrientation.VERTICAL
            variant = "rigid-rotation-of-diagonal"
        category = DynamicExpectationCategory.WAIT_AND_RESUME
        latent_case_id = "diagonal-rigid-pair-v6"
        blocking_cleared_at_s = actors[0].active_until_s
        same_direction_ids = ()
        required_stops = 1
    elif case_key in {"corner-intersection", "second-risk-intersection"}:
        map_length_m = 5.0
        corridor_width_m = 4.8
        initial_state = RobotState(Pose2D(0.60, 0.80, 0.0))
        goal_pose = Pose2D(2.50, 4.20, pi / 2.0)
        reference_path = (
            initial_state.pose,
            Pose2D(2.50, 0.80, 0.0),
            goal_pose,
        )
        layout = DynamicStaticLayoutSpec(
            occupied_regions=(
                DynamicAxisAlignedRegion(0.0, 1.50, 1.20, 4.80),
                DynamicAxisAlignedRegion(3.80, 1.50, 5.00, 4.80),
            ),
            forbidden_regions=(
                DynamicAxisAlignedRegion(0.0, 1.50, 0.40, 4.80),
                DynamicAxisAlignedRegion(4.60, 1.50, 5.00, 4.80),
            ),
        )
        corner_actor = DynamicCorpusActor(
            "corner-crossing",
            13.0,
            23.0,
            Point2D(1.50, 2.60),
            Vector2D(0.20, 0.0),
        )
        if case_key == "corner-intersection":
            actors = (corner_actor,)
            category = DynamicExpectationCategory.WAIT_AND_RESUME
            orientation = DynamicScenarioOrientation.CORNER
            variant = "left-turn-static-topology"
            latent_case_id = "corner-intersection-v6"
            required_stops = 1
        else:
            actors = (
                DynamicCorpusActor(
                    "first-horizontal-risk",
                    2.0,
                    7.0,
                    Point2D(1.50, ACTOR_RADIUS_M),
                    Vector2D(0.0, 0.25),
                ),
                corner_actor,
            )
            category = DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP
            orientation = DynamicScenarioOrientation.INTERSECTION
            variant = "second-risk-after-corner"
            latent_case_id = "second-risk-intersection-v6"
            required_stops = 2
        family = DynamicScenarioFamily.CORNER_INTERSECTION
        blocking_cleared_at_s = max(actor.active_until_s for actor in actors)
        same_direction_ids = ()
    elif case_key in {"simultaneous-two-actor", "staggered-two-actor"}:
        corridor_width_m = 3.0
        center_y = corridor_width_m / 2.0
        initial_state = RobotState(Pose2D(0.60, center_y, 0.0))
        goal_pose = Pose2D(4.40, center_y, 0.0)
        reference_path = (initial_state.pose, goal_pose)
        second_start = 0.0 if case_key == "simultaneous-two-actor" else 10.50
        actors = (
            DynamicCorpusActor(
                "multi-bottom-up",
                0.0,
                10.0,
                Point2D(1.80, ACTOR_RADIUS_M),
                Vector2D(0.0, 0.25),
            ),
            DynamicCorpusActor(
                "multi-top-down",
                second_start,
                second_start + 10.0,
                Point2D(2.60, corridor_width_m - ACTOR_RADIUS_M),
                Vector2D(0.0, -0.25),
            ),
        )
        family = DynamicScenarioFamily.MULTI_ACTOR
        orientation = DynamicScenarioOrientation.HORIZONTAL
        same_direction_ids = ()
        blocking_cleared_at_s = max(actor.active_until_s for actor in actors)
        if case_key == "simultaneous-two-actor":
            category = DynamicExpectationCategory.WAIT_AND_RESUME
            variant = "simultaneous-overlap"
            latent_case_id = "simultaneous-two-actor-v6"
            required_stops = 1
        else:
            category = DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP
            variant = "staggered-two-stop"
            latent_case_id = "staggered-two-actor-v6"
            required_stops = 2
    else:
        raise ValueError(f"unknown v6 public case key: {case_key}")

    oracle = DynamicOracleSpec(
        oracle_version=DYNAMIC_V6_ORACLE_VERSION,
        expectation_category=category,
        hazard_intervals_s=tuple(
            (actor.active_from_s, actor.active_until_s) for actor in actors
        ),
        same_direction_actor_ids=same_direction_ids,
        required_protective_stop_epochs=required_stops,
        feasible_witness=witness,
    )
    return _make_v6_episode(
        seed=seed,
        split=split,
        replica=replica,
        case_key=case_key,
        category=category,
        family=family,
        variant=variant,
        orientation=orientation,
        latent_case_id=latent_case_id,
        duration_s=duration_s,
        corridor_width_m=corridor_width_m,
        map_length_m=map_length_m,
        initial_state=initial_state,
        goal_pose=goal_pose,
        reference_path=reference_path,
        actors=actors,
        layout=layout,
        oracle=oracle,
        progressable=progressable,
        blocking_cleared_at_s=blocking_cleared_at_s,
        observation_fault=observation_fault,
    )


def _make_v6_episode(
    *,
    seed: int,
    split: DynamicCorpusSplit,
    replica: int,
    case_key: str,
    category: DynamicExpectationCategory,
    family: DynamicScenarioFamily,
    variant: str,
    orientation: DynamicScenarioOrientation,
    latent_case_id: str,
    duration_s: float,
    corridor_width_m: float,
    map_length_m: float,
    initial_state: RobotState,
    goal_pose: Pose2D,
    reference_path: tuple[Pose2D, ...],
    actors: tuple[DynamicCorpusActor, ...],
    layout: DynamicStaticLayoutSpec,
    oracle: DynamicOracleSpec,
    progressable: bool,
    blocking_cleared_at_s: float | None,
    observation_fault: str | None,
) -> V6DynamicCorpusEpisode:
    actor_id_map = {
        actor.actor_id: f"dynamic-v6-actor-{index:03d}"
        for index, actor in enumerate(actors)
    }
    if len(actor_id_map) != len(actors):
        raise ValueError("v6 Actor administrative ids must be unique")
    actors = tuple(
        replace(actor, actor_id=actor_id_map[actor.actor_id]) for actor in actors
    )
    oracle = replace(
        oracle,
        same_direction_actor_ids=tuple(
            actor_id_map[actor_id]
            for actor_id in oracle.same_direction_actor_ids
        ),
    )
    episode_id = f"v6-{split.value}-{case_key}-{replica:02d}-{seed}"
    draft = V6DynamicCorpusEpisode(
        schema_version=DYNAMIC_V6_CORPUS_SCHEMA_VERSION,
        generator_version=DYNAMIC_V6_CORPUS_GENERATOR_VERSION,
        episode_id=episode_id,
        split=split,
        expectation_category=category,
        seed=seed,
        simulation_only=True,
        map_id="dynamic-v6-map-pending",
        mission_id=_MISSION_ID,
        duration_s=duration_s,
        corridor_width_m=corridor_width_m,
        map_length_m=map_length_m,
        grid_resolution_m=_GRID_RESOLUTION_M,
        initial_state=initial_state,
        goal_pose=goal_pose,
        reference_path=reference_path,
        actors=actors,
        progressable=progressable,
        blocking_cleared_at_s=blocking_cleared_at_s,
        observation_fault=observation_fault,
        scenario_family=family,
        variant=variant,
        orientation=orientation,
        latent_case_id=latent_case_id,
        static_layout_spec=layout,
        oracle_spec=oracle,
        semantic_world_hash="pending",
        oracle_hash=canonical_content_hash(oracle),
    )
    semantic_world_hash = _semantic_world_hash(draft)
    return replace(
        draft,
        semantic_world_hash=semantic_world_hash,
        map_id=f"dynamic-v6-map-{semantic_world_hash[:24]}",
    )


def _same_direction_feasible_witness(center_y: float) -> DynamicFeasibleWitness:
    points = [
        DynamicFeasibleWitnessPoint(
            0.0,
            Pose2D(0.60, center_y, 0.0),
            Twist2D(),
        )
    ]
    # DWA의 비용·tie-break를 사용하지 않는 evaluator-only 직각 우회 증인이다.
    # 각 maneuver 사이를 완전히 정지해 differential-drive 운동학과 가감속
    # 경계를 독립적으로 검증할 수 있게 한다.
    maneuvers = (
        (Twist2D(0.0, -0.80), 39),
        (Twist2D(0.25, 0.0), 69),
        (Twist2D(0.0, 0.80), 39),
        (Twist2D(0.25, 0.0), 229),
        (Twist2D(0.0, 0.80), 39),
        (Twist2D(0.25, 0.0), 69),
        (Twist2D(0.0, -0.80), 39),
        (Twist2D(0.25, 0.0), 83),
    )
    for maneuver_index, (target, command_ticks) in enumerate(maneuvers):
        _append_witness_maneuver(points, target, command_ticks=command_ticks)
        if maneuver_index == 3:
            # 빠른 Actor replica까지 추월한 뒤 time-indexed tube와 겹치지 않는
            # 측면 위치에서 Actor 활성 구간이 끝날 때까지 기다린다.
            _append_witness_dwell(points, duration_s=9.25)
    _append_witness_dwell(points, duration_s=0.50)
    return DynamicFeasibleWitness(
        witness_id="same-direction-wide-independent-v2",
        points=tuple(points),
    )


def _append_witness_maneuver(
    points: list[DynamicFeasibleWitnessPoint],
    target: Twist2D,
    *,
    command_ticks: int,
) -> None:
    if command_ticks <= 0:
        raise ValueError("witness maneuver must contain at least one tick")
    for _ in range(command_ticks):
        _append_witness_tick(points, target)
    while not _twist_stopped(points[-1].twist):
        _append_witness_tick(points, Twist2D())


def _append_witness_dwell(
    points: list[DynamicFeasibleWitnessPoint],
    *,
    duration_s: float,
) -> None:
    tick_count = round(duration_s / DYNAMIC_CONTROL_PERIOD_S)
    if abs(tick_count * DYNAMIC_CONTROL_PERIOD_S - duration_s) > 1e-12:
        raise ValueError("witness dwell must align to the 20 Hz control period")
    if not _twist_stopped(points[-1].twist):
        raise ValueError("witness dwell must start from an actual stop")
    for _ in range(tick_count):
        _append_witness_tick(points, Twist2D())


def _append_witness_tick(
    points: list[DynamicFeasibleWitnessPoint],
    target: Twist2D,
) -> None:
    current = points[-1]
    next_pose = _integrate_witness_pose(
        current.pose,
        current.twist,
        DYNAMIC_CONTROL_PERIOD_S,
    )
    next_twist = Twist2D(
        linear=_slew_witness_linear(current.twist.linear, target.linear),
        angular=_slew_witness_angular(current.twist.angular, target.angular),
    )
    points.append(
        DynamicFeasibleWitnessPoint(
            round(current.time_s + DYNAMIC_CONTROL_PERIOD_S, 12),
            next_pose,
            next_twist,
        )
    )


def _slew_witness_linear(current: float, target: float) -> float:
    increasing_magnitude = abs(target) > abs(current) + 1e-12
    rate = (
        VIRTUAL_DOLL_WHEELCHAIR_V0_1.max_acceleration_mps2
        if increasing_magnitude
        else VIRTUAL_DOLL_WHEELCHAIR_V0_1.max_deceleration_mps2
    )
    return _slew_witness_scalar(current, target, rate * DYNAMIC_CONTROL_PERIOD_S)


def _slew_witness_angular(current: float, target: float) -> float:
    return _slew_witness_scalar(
        current,
        target,
        _V6_WITNESS_MAX_ANGULAR_ACCELERATION_RADPS2 * DYNAMIC_CONTROL_PERIOD_S,
    )


def _slew_witness_scalar(current: float, target: float, maximum_delta: float) -> float:
    delta = target - current
    if abs(delta) <= maximum_delta:
        return target
    return current + (maximum_delta if delta > 0.0 else -maximum_delta)


def _integrate_witness_pose(pose: Pose2D, twist: Twist2D, dt_s: float) -> Pose2D:
    return Pose2D(
        pose.x + twist.linear * cos(pose.yaw) * dt_s,
        pose.y + twist.linear * sin(pose.yaw) * dt_s,
        _normalize_angle(pose.yaw + twist.angular * dt_s),
    )


def _twist_stopped(twist: Twist2D) -> bool:
    return abs(twist.linear) <= 1e-12 and abs(twist.angular) <= 1e-12


def _rotate_pose_90(pose: Pose2D, square_size_m: float) -> Pose2D:
    return Pose2D(
        square_size_m - pose.y,
        pose.x,
        (pose.yaw + pi / 2.0 + pi) % (2.0 * pi) - pi,
    )


def _rotate_actor_90(
    actor: DynamicCorpusActor,
    square_size_m: float,
) -> DynamicCorpusActor:
    return DynamicCorpusActor(
        actor_id=actor.actor_id,
        active_from_s=actor.active_from_s,
        active_until_s=actor.active_until_s,
        start_position=Point2D(
            square_size_m - actor.start_position.y,
            actor.start_position.x,
        ),
        velocity=Vector2D(-actor.velocity.y, actor.velocity.x),
        radius_m=actor.radius_m,
        trajectory_revision=actor.trajectory_revision,
    )


def _is_exact_rigid_rotation_pair(
    horizontal: V6DynamicCorpusEpisode,
    vertical: V6DynamicCorpusEpisode,
) -> bool:
    square_size_m = horizontal.map_length_m
    return all(
        (
            horizontal.seed == vertical.seed,
            horizontal.schema_version == vertical.schema_version,
            horizontal.generator_version == vertical.generator_version,
            abs(horizontal.duration_s - vertical.duration_s) <= 1e-12,
            abs(horizontal.grid_resolution_m - vertical.grid_resolution_m) <= 1e-12,
            abs(horizontal.map_length_m - horizontal.corridor_width_m) <= 1e-12,
            abs(vertical.map_length_m - square_size_m) <= 1e-12,
            abs(vertical.corridor_width_m - square_size_m) <= 1e-12,
            vertical.initial_state.pose
            == _rotate_pose_90(horizontal.initial_state.pose, square_size_m),
            vertical.initial_state.twist == horizontal.initial_state.twist,
            vertical.goal_pose == _rotate_pose_90(horizontal.goal_pose, square_size_m),
            vertical.reference_path
            == tuple(
                _rotate_pose_90(pose, square_size_m)
                for pose in horizontal.reference_path
            ),
            vertical.actors
            == tuple(
                _rotate_actor_90(actor, square_size_m)
                for actor in horizontal.actors
            ),
            vertical.progressable == horizontal.progressable,
            vertical.blocking_cleared_at_s == horizontal.blocking_cleared_at_s,
            vertical.observation_fault == horizontal.observation_fault,
            vertical.expectation_category == horizontal.expectation_category,
            vertical.static_layout_spec == horizontal.static_layout_spec,
            vertical.oracle_spec == horizontal.oracle_spec,
        )
    )


def _grid_aligned_v6_dimension(value_m: float) -> float:
    cell_count = round(value_m / _GRID_RESOLUTION_M)
    if cell_count <= 0:
        raise ValueError("v6 grid dimension must contain at least one cell")
    return round(cell_count * _GRID_RESOLUTION_M, 12)


def _semantic_world_hash(episode: V6DynamicCorpusEpisode) -> str:
    actor_worlds = tuple(
        sorted(
            (
                {
                    "active_from_s": actor.active_from_s,
                    "active_until_s": actor.active_until_s,
                    "start_position": actor.start_position,
                    "velocity": actor.velocity,
                    "radius_m": actor.radius_m,
                }
                for actor in episode.actors
            ),
            key=canonical_content_hash,
        )
    )
    return canonical_content_hash(
        {
            "duration_s": episode.duration_s,
            "corridor_width_m": episode.corridor_width_m,
            "map_length_m": episode.map_length_m,
            "grid_resolution_m": episode.grid_resolution_m,
            "initial_state": episode.initial_state,
            "goal_pose": episode.goal_pose,
            "reference_path": episode.reference_path,
            "actors": actor_worlds,
            "static_layout_spec": episode.static_layout_spec,
        }
    )


def controller_episode_id(episode: DynamicCorpusEpisode) -> str:
    if isinstance(episode, V6DynamicCorpusEpisode):
        return f"dynamic-v6-source-{episode.semantic_world_hash[:24]}"
    return episode.episode_id


def _rasterize_regions(
    occupancy: np.ndarray,
    regions: tuple[DynamicAxisAlignedRegion, ...],
    *,
    resolution_m: float,
) -> None:
    height, width = occupancy.shape
    for region in regions:
        min_x = max(0, floor(region.min_x_m / resolution_m))
        min_y = max(0, floor(region.min_y_m / resolution_m))
        max_x = min(width, ceil(region.max_x_m / resolution_m))
        max_y = min(height, ceil(region.max_y_m / resolution_m))
        if min_x < max_x and min_y < max_y:
            occupancy[min_y:max_y, min_x:max_x] = True


def _v6_episode_validation_failures(
    episode: V6DynamicCorpusEpisode,
) -> list[str]:
    failures: list[str] = []
    prefix = f"{episode.episode_id}:"
    layout_regions = (
        *episode.static_layout_spec.occupied_regions,
        *episode.static_layout_spec.forbidden_regions,
    )
    for region in layout_regions:
        if (
            region.min_x_m < 0.0
            or region.min_y_m < 0.0
            or region.max_x_m > episode.map_length_m
            or region.max_y_m > episode.corridor_width_m
        ):
            failures.append(f"{prefix}static_layout_out_of_bounds")

    snapshot = build_dynamic_grid_snapshot(episode)
    checker = CollisionChecker(
        snapshot.grid,
        VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        forbidden_cells=snapshot.forbidden_cells,
    )
    for pose in _sample_polyline(episode.reference_path, spacing_m=0.02):
        if checker.clearance(pose) < (
            VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m - 1e-9
        ):
            failures.append(f"{prefix}reference_path_not_statically_clear")
            break

    for actor in episode.actors:
        failures.extend(_actor_trajectory_failures(episode, actor))
    for index, first in enumerate(episode.actors):
        for second in episode.actors[index + 1 :]:
            if _actors_overlap_over_active_trajectory(first, second):
                failures.append(
                    f"{prefix}actor_trajectory_overlap:{first.actor_id}:{second.actor_id}"
                )

    oracle_actor_ids = set(episode.oracle_spec.same_direction_actor_ids)
    episode_actor_ids = {actor.actor_id for actor in episode.actors}
    if not oracle_actor_ids.issubset(episode_actor_ids):
        failures.append(f"{prefix}oracle_actor_binding_unknown")

    if episode.scenario_family is DynamicScenarioFamily.SAME_DIRECTION:
        for actor in episode.actors:
            if not (
                0.0 < actor.velocity.x < 0.20
                and abs(actor.velocity.y) <= 1e-12
            ):
                failures.append(f"{prefix}same_direction_velocity_invalid")
        initial_actor = episode.actors[0].state_at(episode.actors[0].active_from_s)
        if initial_actor is not None:
            initial_gap = (
                initial_actor.position.x
                - episode.initial_state.pose.x
                - VIRTUAL_DOLL_WHEELCHAIR_V0_1.collision_length_m / 2.0
                - initial_actor.radius_m
            )
            if initial_gap < 0.48 - 1e-9:
                failures.append(f"{prefix}same_direction_initial_gap_below_0_48m")

    if episode.scenario_family is DynamicScenarioFamily.HEAD_ON:
        actor = episode.actors[0]
        if actor.velocity.x >= 0.0 or abs(actor.velocity.y) > 1e-12:
            failures.append(f"{prefix}head_on_velocity_invalid")
        lateral_offset = abs(actor.start_position.y - episode.initial_state.pose.y)
        if lateral_offset < 0.44 - 1e-9:
            failures.append(f"{prefix}head_on_offset_below_0_44m")

    if episode.scenario_family in {
        DynamicScenarioFamily.DIAGONAL_CROSSING,
        DynamicScenarioFamily.VERTICAL_PATH,
    }:
        velocity = episode.actors[0].velocity
        if abs(velocity.x) <= 1e-12 or abs(velocity.y) <= 1e-12:
            failures.append(f"{prefix}diagonal_velocity_component_missing")

    if episode.scenario_family is DynamicScenarioFamily.CORNER_INTERSECTION:
        if len(episode.reference_path) < 3:
            failures.append(f"{prefix}corner_reference_not_multisegment")
        if not episode.static_layout_spec.occupied_regions:
            failures.append(f"{prefix}corner_static_topology_missing")

    if episode.scenario_family is DynamicScenarioFamily.MULTI_ACTOR:
        if len(episode.actors) < 2:
            failures.append(f"{prefix}multi_actor_count_below_two")
        if episode.variant == "simultaneous-overlap":
            overlap_start = max(actor.active_from_s for actor in episode.actors)
            overlap_end = min(actor.active_until_s for actor in episode.actors)
            if overlap_end <= overlap_start:
                failures.append(f"{prefix}multi_actor_active_interval_not_overlapping")
            elif len(episode.actor_states_at((overlap_start + overlap_end) / 2.0)) < 2:
                failures.append(f"{prefix}multi_actor_ground_truth_not_simultaneous")

    if episode.oracle_spec.feasible_witness is not None:
        failures.extend(_feasible_witness_failures(episode))
    return failures


def _actor_trajectory_failures(
    episode: V6DynamicCorpusEpisode,
    actor: DynamicCorpusActor,
) -> list[str]:
    failures: list[str] = []
    prefix = f"{episode.episode_id}:{actor.actor_id}:"
    if actor.active_until_s > episode.duration_s + 1e-9:
        failures.append(f"{prefix}active_interval_exceeds_episode")
    duration = actor.active_until_s - actor.active_from_s
    for elapsed_s in (0.0, duration):
        x = actor.start_position.x + actor.velocity.x * elapsed_s
        y = actor.start_position.y + actor.velocity.y * elapsed_s
        if (
            x < actor.radius_m - 1e-9
            or x > episode.map_length_m - actor.radius_m + 1e-9
            or y < actor.radius_m - 1e-9
            or y > episode.corridor_width_m - actor.radius_m + 1e-9
        ):
            failures.append(f"{prefix}trajectory_out_of_map")
            break
    for region in (
        *episode.static_layout_spec.occupied_regions,
        *episode.static_layout_spec.forbidden_regions,
    ):
        rasterized_region = _rasterized_region_extent(
            region,
            resolution_m=episode.grid_resolution_m,
            map_length_m=episode.map_length_m,
            corridor_width_m=episode.corridor_width_m,
        )
        if rasterized_region is not None and (
            _moving_point_region_minimum_distance(actor, rasterized_region)
            < actor.radius_m - 1e-9
        ):
            failures.append(f"{prefix}trajectory_intersects_static_layout")
            break
    return failures


def _rasterized_region_extent(
    region: DynamicAxisAlignedRegion,
    *,
    resolution_m: float,
    map_length_m: float,
    corridor_width_m: float,
) -> DynamicAxisAlignedRegion | None:
    """Return the continuous AABB occupied by the same cells as rasterization."""

    min_x_m = max(0.0, floor(region.min_x_m / resolution_m) * resolution_m)
    min_y_m = max(0.0, floor(region.min_y_m / resolution_m) * resolution_m)
    max_x_m = min(
        map_length_m,
        ceil(region.max_x_m / resolution_m) * resolution_m,
    )
    max_y_m = min(
        corridor_width_m,
        ceil(region.max_y_m / resolution_m) * resolution_m,
    )
    if max_x_m <= min_x_m or max_y_m <= min_y_m:
        return None
    return DynamicAxisAlignedRegion(
        min_x_m=round(min_x_m, 12),
        min_y_m=round(min_y_m, 12),
        max_x_m=round(max_x_m, 12),
        max_y_m=round(max_y_m, 12),
    )


def _moving_point_region_minimum_distance(
    actor: DynamicCorpusActor,
    region: DynamicAxisAlignedRegion,
) -> float:
    duration = actor.active_until_s - actor.active_from_s
    candidates = {0.0, duration}
    if abs(actor.velocity.x) > 1e-12:
        for boundary in (region.min_x_m, region.max_x_m):
            candidates.add(
                min(
                    duration,
                    max(0.0, (boundary - actor.start_position.x) / actor.velocity.x),
                )
            )
    if abs(actor.velocity.y) > 1e-12:
        for boundary in (region.min_y_m, region.max_y_m):
            candidates.add(
                min(
                    duration,
                    max(0.0, (boundary - actor.start_position.y) / actor.velocity.y),
                )
            )
    speed_squared = actor.velocity.x**2 + actor.velocity.y**2
    if speed_squared > 1e-18:
        for corner_x, corner_y in (
            (region.min_x_m, region.min_y_m),
            (region.min_x_m, region.max_y_m),
            (region.max_x_m, region.min_y_m),
            (region.max_x_m, region.max_y_m),
        ):
            projection_s = -(
                (actor.start_position.x - corner_x) * actor.velocity.x
                + (actor.start_position.y - corner_y) * actor.velocity.y
            ) / speed_squared
            candidates.add(min(duration, max(0.0, projection_s)))
    return min(
        _point_region_distance(
            actor.start_position.x + actor.velocity.x * elapsed_s,
            actor.start_position.y + actor.velocity.y * elapsed_s,
            region,
        )
        for elapsed_s in candidates
    )


def _point_region_distance(
    x: float,
    y: float,
    region: DynamicAxisAlignedRegion,
) -> float:
    dx = max(region.min_x_m - x, 0.0, x - region.max_x_m)
    dy = max(region.min_y_m - y, 0.0, y - region.max_y_m)
    return hypot(dx, dy)


def _actors_overlap_over_active_trajectory(
    first: DynamicCorpusActor,
    second: DynamicCorpusActor,
) -> bool:
    overlap_start = max(first.active_from_s, second.active_from_s)
    overlap_end = min(first.active_until_s, second.active_until_s)
    if overlap_end < overlap_start:
        return False
    first_elapsed = overlap_start - first.active_from_s
    second_elapsed = overlap_start - second.active_from_s
    dx = (
        first.start_position.x
        + first.velocity.x * first_elapsed
        - second.start_position.x
        - second.velocity.x * second_elapsed
    )
    dy = (
        first.start_position.y
        + first.velocity.y * first_elapsed
        - second.start_position.y
        - second.velocity.y * second_elapsed
    )
    dvx = first.velocity.x - second.velocity.x
    dvy = first.velocity.y - second.velocity.y
    duration = overlap_end - overlap_start
    speed_squared = dvx * dvx + dvy * dvy
    closest_t = (
        0.0
        if speed_squared <= 1e-18
        else min(duration, max(0.0, -(dx * dvx + dy * dvy) / speed_squared))
    )
    distance = hypot(dx + dvx * closest_t, dy + dvy * closest_t)
    return distance < first.radius_m + second.radius_m - 1e-9


def _feasible_witness_failures(episode: V6DynamicCorpusEpisode) -> list[str]:
    witness = episode.oracle_spec.feasible_witness
    if witness is None:
        return [f"{episode.episode_id}:feasible_witness_missing"]
    failures: list[str] = []
    prefix = f"{episode.episode_id}:"
    if abs(witness.points[0].time_s) > 1e-12:
        failures.append(f"{prefix}feasible_witness_must_start_at_zero")
    if hypot(
        witness.points[0].pose.x - episode.initial_state.pose.x,
        witness.points[0].pose.y - episode.initial_state.pose.y,
    ) > 1e-9 or abs(
        _normalize_angle(
            witness.points[0].pose.yaw - episode.initial_state.pose.yaw
        )
    ) > 1e-9:
        failures.append(f"{prefix}feasible_witness_start_mismatch")
    if witness.points[0].twist != episode.initial_state.twist:
        failures.append(f"{prefix}feasible_witness_start_twist_mismatch")
    if hypot(
        witness.points[-1].pose.x - episode.goal_pose.x,
        witness.points[-1].pose.y - episode.goal_pose.y,
    ) > 1e-9 or abs(
        _normalize_angle(witness.points[-1].pose.yaw - episode.goal_pose.yaw)
    ) > 1e-9:
        failures.append(f"{prefix}feasible_witness_goal_mismatch")

    snapshot = build_dynamic_grid_snapshot(episode)
    checker = CollisionChecker(
        snapshot.grid,
        VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        forbidden_cells=snapshot.forbidden_cells,
    )
    samples: list[tuple[float, Pose2D, Twist2D]] = []
    for left, right in zip(witness.points, witness.points[1:], strict=False):
        duration = right.time_s - left.time_s
        if abs(duration - DYNAMIC_CONTROL_PERIOD_S) > 1e-12:
            failures.append(f"{prefix}feasible_witness_not_20hz")
            continue
        if not (
            -VIRTUAL_DOLL_WHEELCHAIR_V0_1.max_reverse_speed_mps - 1e-9
            <= left.twist.linear
            <= VIRTUAL_DOLL_WHEELCHAIR_V0_1.max_forward_speed_mps + 1e-9
        ):
            failures.append(f"{prefix}feasible_witness_speed_exceeded")
        if abs(left.twist.angular) > (
            VIRTUAL_DOLL_WHEELCHAIR_V0_1.max_angular_speed_radps + 1e-9
        ):
            failures.append(f"{prefix}feasible_witness_angular_speed_exceeded")
        expected_pose = _integrate_witness_pose(left.pose, left.twist, duration)
        if not _witness_poses_close(expected_pose, right.pose):
            failures.append(f"{prefix}feasible_witness_kinematic_mismatch")

        linear_sign_flip = left.twist.linear * right.twist.linear < -1e-12
        if linear_sign_flip:
            failures.append(f"{prefix}feasible_witness_reverse_without_stop")
        linear_rate = abs(right.twist.linear - left.twist.linear) / duration
        increasing_magnitude = abs(right.twist.linear) > abs(left.twist.linear) + 1e-12
        linear_limit = (
            VIRTUAL_DOLL_WHEELCHAIR_V0_1.max_acceleration_mps2
            if increasing_magnitude
            else VIRTUAL_DOLL_WHEELCHAIR_V0_1.max_deceleration_mps2
        )
        if linear_rate > linear_limit + 1e-9:
            failures.append(f"{prefix}feasible_witness_acceleration_exceeded")
        angular_rate = abs(right.twist.angular - left.twist.angular) / duration
        if angular_rate > _V6_WITNESS_MAX_ANGULAR_ACCELERATION_RADPS2 + 1e-9:
            failures.append(f"{prefix}feasible_witness_angular_acceleration_exceeded")

        subdivision_count = round(duration / _V6_WITNESS_EVALUATOR_PERIOD_S)
        for subdivision in range(subdivision_count):
            offset_s = subdivision * _V6_WITNESS_EVALUATOR_PERIOD_S
            samples.append(
                (
                    left.time_s + offset_s,
                    _integrate_witness_pose(left.pose, left.twist, offset_s),
                    left.twist,
                )
            )
    samples.append(
        (
            witness.points[-1].time_s,
            witness.points[-1].pose,
            witness.points[-1].twist,
        )
    )

    dwell_started_at_s = witness.points[-1].time_s - witness.terminal_dwell_s
    dwell_points = tuple(
        point
        for point in witness.points
        if point.time_s >= dwell_started_at_s - 1e-12
    )
    if (
        not dwell_points
        or dwell_points[0].time_s > dwell_started_at_s + 1e-12
        or any(
            not _twist_stopped(point.twist)
            or not _witness_poses_close(point.pose, witness.points[-1].pose)
            for point in dwell_points
        )
    ):
        failures.append(f"{prefix}feasible_witness_terminal_dwell_missing")
    if witness.points[-1].time_s > episode.duration_s + 1e-12:
        failures.append(f"{prefix}feasible_witness_exceeds_episode_window")

    half_diagonal_m = hypot(
        VIRTUAL_DOLL_WHEELCHAIR_V0_1.collision_length_m / 2.0,
        VIRTUAL_DOLL_WHEELCHAIR_V0_1.collision_width_m / 2.0,
    )
    expected_overtakes = set(episode.oracle_spec.same_direction_actor_ids)
    initially_ahead: set[str] = set()
    witnessed_overtakes: set[str] = set()
    departure_time_s: float | None = None
    first_overtake_time_s: float | None = None
    for time_s, pose, twist in samples:
        robot_speed_bound = abs(twist.linear) + abs(twist.angular) * half_diagonal_m
        swept_static_clearance = checker.clearance(pose) - (
            robot_speed_bound * _V6_WITNESS_EVALUATOR_PERIOD_S / 2.0
        )
        swept_forbidden_clearance = checker.forbidden_clearance(pose) - (
            robot_speed_bound * _V6_WITNESS_EVALUATOR_PERIOD_S / 2.0
        )
        if swept_static_clearance < (
            VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m - 1e-9
        ) or swept_forbidden_clearance < -1e-9:
            failures.append(f"{prefix}feasible_witness_static_or_forbidden_failure")
            break
        deviation = _point_path_distance(pose.x, pose.y, episode.reference_path)
        if (
            departure_time_s is None
            and deviation > episode.oracle_spec.departure_threshold_m
        ):
            departure_time_s = time_s
        robot_progress = _point_path_progress(pose.x, pose.y, episode.reference_path)
        for actor in episode.actor_states_at(min(time_s, episode.duration_s)):
            actor_progress = _point_path_progress(
                actor.position.x,
                actor.position.y,
                episode.reference_path,
            )
            order = actor_progress - robot_progress
            longitudinal_extent = (
                VIRTUAL_DOLL_WHEELCHAIR_V0_1.collision_length_m / 2.0
                + actor.radius_m
            )
            if actor.actor_id in expected_overtakes:
                if order > longitudinal_extent:
                    initially_ahead.add(actor.actor_id)
                elif (
                    actor.actor_id in initially_ahead
                    and order < -longitudinal_extent
                    and departure_time_s is not None
                ):
                    witnessed_overtakes.add(actor.actor_id)
                    if first_overtake_time_s is None:
                        first_overtake_time_s = time_s
            for profile in (
                NORMAL_OBSERVATION_PROFILE,
                STRESS_OBSERVATION_PROFILE,
            ):
                tube_radius = _v6_witness_tube_radius(
                    actor.velocity.magnitude,
                    profile=profile,
                )
                clearance = oriented_footprint_circle_surface_distance(
                    pose,
                    circle_center=(actor.position.x, actor.position.y),
                    circle_radius_m=tube_radius,
                ) - (
                    (robot_speed_bound + actor.velocity.magnitude)
                    * _V6_WITNESS_EVALUATOR_PERIOD_S
                    / 2.0
                )
                if clearance < (
                    VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m - 1e-9
                ):
                    failures.append(
                        f"{prefix}feasible_witness_actor_tube_failure_"
                        f"{profile.name.value}"
                    )
                    return failures
    deviations = tuple(
        _point_path_distance(point.pose.x, point.pose.y, episode.reference_path)
        for point in witness.points
    )
    if max(deviations, default=0.0) <= episode.oracle_spec.departure_threshold_m:
        failures.append(f"{prefix}feasible_witness_has_no_detour")
    if expected_overtakes and not expected_overtakes.issubset(witnessed_overtakes):
        failures.append(f"{prefix}feasible_witness_has_no_ordered_overtake")
    if (
        expected_overtakes
        and departure_time_s is not None
        and first_overtake_time_s is not None
        and departure_time_s >= first_overtake_time_s
    ):
        failures.append(f"{prefix}feasible_witness_overtake_before_departure")
    return failures


def _v6_witness_tube_radius(
    actor_speed_mps: float,
    *,
    profile: DynamicObservationProfile,
) -> float:
    # 각 profile의 delivery age와 50 ms 적용 지연을 포함한 rollout-zero
    # 2σ/reachable envelope. 200 Hz witness는 Normal과 Stress를 모두 검사한다.
    # Dropout 직전 마지막 valid frame은 age==TTL까지 fresh다. 여기에 한 control
    # tick의 적용 지연을 더한 0.35 s가 두 공개 profile의 worst fresh horizon이다.
    tau_s = profile.ttl_s + DYNAMIC_CONTROL_PERIOD_S
    sigma_m = sqrt(
        profile.position_sigma_m**2
        + (tau_s * profile.velocity_sigma_mps) ** 2
    )
    bounded_speed = min(actor_speed_mps, MAX_ACTOR_SPEED_MPS)
    velocity_delta = MAX_ACTOR_SPEED_MPS + bounded_speed
    acceleration_saturation_s = velocity_delta / MAX_ACTOR_ACCELERATION_MPS2
    acceleration_bound_m = (
        0.5 * MAX_ACTOR_ACCELERATION_MPS2 * tau_s**2
        if tau_s <= acceleration_saturation_s
        else 0.5
        * MAX_ACTOR_ACCELERATION_MPS2
        * acceleration_saturation_s**2
        + velocity_delta * (tau_s - acceleration_saturation_s)
    )
    return ACTOR_RADIUS_M + 2.0 * sigma_m + acceleration_bound_m


def _witness_poses_close(first: Pose2D, second: Pose2D) -> bool:
    return (
        abs(first.x - second.x) <= 1e-9
        and abs(first.y - second.y) <= 1e-9
        and abs(_normalize_angle(first.yaw - second.yaw)) <= 1e-9
    )


def _sample_polyline(
    path: tuple[Pose2D, ...],
    *,
    spacing_m: float,
) -> tuple[Pose2D, ...]:
    samples: list[Pose2D] = []
    for source, target in zip(path, path[1:], strict=False):
        dx = target.x - source.x
        dy = target.y - source.y
        distance = hypot(dx, dy)
        count = max(1, round(distance / spacing_m))
        tangent = atan2(dy, dx)
        samples.extend(
            Pose2D(
                source.x + dx * index / count,
                source.y + dy * index / count,
                tangent,
            )
            for index in range(count)
        )
    samples.append(path[-1])
    return tuple(samples)


def _point_path_distance(
    x: float,
    y: float,
    path: tuple[Pose2D, ...],
) -> float:
    best = float("inf")
    for source, target in zip(path, path[1:], strict=False):
        dx = target.x - source.x
        dy = target.y - source.y
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-18:
            continue
        fraction = min(
            1.0,
            max(0.0, ((x - source.x) * dx + (y - source.y) * dy) / length_squared),
        )
        best = min(
            best,
            hypot(x - source.x - fraction * dx, y - source.y - fraction * dy),
        )
    return best


def _point_path_progress(
    x: float,
    y: float,
    path: tuple[Pose2D, ...],
) -> float:
    best_distance = float("inf")
    best_progress = 0.0
    cumulative = 0.0
    for source, target in zip(path, path[1:], strict=False):
        dx = target.x - source.x
        dy = target.y - source.y
        length = hypot(dx, dy)
        if length <= 1e-18:
            continue
        fraction = min(
            1.0,
            max(0.0, ((x - source.x) * dx + (y - source.y) * dy) / length**2),
        )
        projected_x = source.x + fraction * dx
        projected_y = source.y + fraction * dy
        distance = hypot(x - projected_x, y - projected_y)
        if distance < best_distance:
            best_distance = distance
            best_progress = cumulative + fraction * length
        cumulative += length
    return best_progress


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def _generate_episode(
    *,
    seed: int,
    split: DynamicCorpusSplit,
    category: DynamicExpectationCategory,
    replica: int,
) -> DynamicCorpusEpisode:
    rng = Random(seed)
    narrow_width = round(rng.uniform(0.84, 0.92), 2)
    medium_width = round(rng.uniform(1.15, 1.35), 2)
    wide_width = round(rng.uniform(4.40, 4.80), 2)
    corridor_width = (
        wide_width
        if category is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
        else narrow_width
        if category
        in {
            DynamicExpectationCategory.LOCAL_DETOUR_FORBIDDEN,
            DynamicExpectationCategory.NO_SAFE_SOLUTION,
        }
        else medium_width
    )
    center_y = corridor_width / 2.0
    speed = round(rng.uniform(0.30, 0.45), 3)
    crossing_x = round(rng.uniform(1.30, 2.10), 3)
    crossing_duration = (corridor_width - 2.0 * ACTOR_RADIUS_M) / speed

    actors: tuple[DynamicCorpusActor, ...]
    fault: str | None = None
    progressable = True
    clear_at: float | None
    if category is DynamicExpectationCategory.NO_SAFE_SOLUTION:
        actors = (
            DynamicCorpusActor(
                actor_id="actor-static-block",
                active_from_s=0.0,
                active_until_s=_DURATION_S,
                start_position=Point2D(2.0, center_y),
                velocity=Vector2D(0.0, 0.0),
            ),
        )
        progressable = False
        clear_at = None
    elif category is DynamicExpectationCategory.OBSERVATION_INVALID:
        actors = ()
        fault = "source_invalid_then_recovers"
        clear_at = 2.0
    elif category is DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP:
        first_start = 0.0
        second_start = min(9.0, crossing_duration + 2.0)
        actors = (
            DynamicCorpusActor(
                "actor-first",
                first_start,
                first_start + crossing_duration,
                Point2D(crossing_x, ACTOR_RADIUS_M),
                Vector2D(0.0, speed),
            ),
            DynamicCorpusActor(
                "actor-second",
                second_start,
                second_start + crossing_duration,
                Point2D(min(3.2, crossing_x + 1.3), ACTOR_RADIUS_M),
                Vector2D(0.0, speed),
            ),
        )
        clear_at = second_start + crossing_duration
    else:
        start_time = 0.0 if category is DynamicExpectationCategory.WAIT_AND_RESUME else 2.0
        actors = (
            DynamicCorpusActor(
                actor_id="actor-crossing",
                active_from_s=start_time,
                active_until_s=start_time + crossing_duration,
                start_position=Point2D(crossing_x, ACTOR_RADIUS_M),
                velocity=Vector2D(0.0, speed),
            ),
        )
        clear_at = start_time + crossing_duration

    identifier = f"{split.value}-{category.value}-{replica:02d}-{seed}"
    return DynamicCorpusEpisode(
        schema_version=DYNAMIC_CORPUS_SCHEMA_VERSION,
        generator_version=DYNAMIC_CORPUS_GENERATOR_VERSION,
        episode_id=identifier,
        split=split,
        expectation_category=category,
        seed=seed,
        simulation_only=True,
        map_id=f"dynamic-map-{identifier}",
        mission_id=_MISSION_ID,
        duration_s=_DURATION_S,
        corridor_width_m=corridor_width,
        map_length_m=_MAP_LENGTH_M,
        grid_resolution_m=_GRID_RESOLUTION_M,
        initial_state=RobotState(Pose2D(0.60, center_y, 0.0)),
        goal_pose=Pose2D(4.40, center_y, 0.0),
        reference_path=(
            Pose2D(0.60, center_y, 0.0),
            Pose2D(4.40, center_y, 0.0),
        ),
        actors=actors,
        progressable=progressable,
        blocking_cleared_at_s=clear_at,
        observation_fault=fault,
    )


def _episode_validation_failures(episode: DynamicCorpusEpisode) -> list[str]:
    failures: list[str] = []
    snapshot = build_dynamic_grid_snapshot(episode)
    checker = CollisionChecker(snapshot.grid)
    if checker.clearance(episode.initial_state.pose) < (
        VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m
    ):
        failures.append(f"{episode.episode_id}:initial_state_not_clear")
    if checker.clearance(episode.goal_pose) < (
        VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m
    ):
        failures.append(f"{episode.episode_id}:goal_not_clear")
    for actor in episode.actors:
        if actor.velocity.magnitude > MAX_ACTOR_SPEED_MPS + 1e-12:
            failures.append(f"{episode.episode_id}:actor_speed_exceeded")
        # Corpus actor는 활성 구간 동안 constant velocity이므로 가속도는 0이다.
        state = actor.state_at(actor.active_from_s)
        if state is None:
            failures.append(f"{episode.episode_id}:actor_start_missing")
            continue
        clearance = oriented_footprint_circle_surface_distance(
            episode.initial_state.pose,
            circle_center=(state.position.x, state.position.y),
            circle_radius_m=state.radius_m,
        )
        if clearance < VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m:
            failures.append(f"{episode.episode_id}:actor_initial_collision")

    physical_pass_width = (
        VIRTUAL_DOLL_WHEELCHAIR_V0_1.collision_width_m
        + 2.0 * ACTOR_RADIUS_M
        + 3.0 * VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m
    )
    if (
        episode.expectation_category
        in {
            DynamicExpectationCategory.LOCAL_DETOUR_FORBIDDEN,
            DynamicExpectationCategory.NO_SAFE_SOLUTION,
        }
        and episode.corridor_width_m >= physical_pass_width
    ):
        failures.append(f"{episode.episode_id}:forbidden_geometry_too_wide")
    if episode.expectation_category is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE:
        required = _normal_prediction_pass_width(
            max((actor.velocity.magnitude for actor in episode.actors), default=0.0)
        )
        if episode.corridor_width_m < required:
            failures.append(f"{episode.episode_id}:detour_geometry_too_narrow")
    if (
        episode.expectation_category is DynamicExpectationCategory.NO_SAFE_SOLUTION
        and oriented_footprint_circle_surface_distance(
            episode.initial_state.pose,
            circle_center=(episode.actors[0].start_position.x, episode.actors[0].start_position.y),
            circle_radius_m=ACTOR_RADIUS_M,
        )
        < VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m
    ):
        failures.append(f"{episode.episode_id}:no_safe_solution_has_no_safe_hold_space")
    if (
        episode.expectation_category is DynamicExpectationCategory.OBSERVATION_INVALID
        and episode.observation_fault is None
    ):
        failures.append(f"{episode.episode_id}:observation_fault_missing")
    if (
        episode.expectation_category is DynamicExpectationCategory.DYNAMIC_CHANGE_RESTOP
        and len(episode.actors) < 2
    ):
        failures.append(f"{episode.episode_id}:second_actor_missing")
    return failures


def _normal_prediction_pass_width(actor_speed_mps: float) -> float:
    tau_s = 0.100 + 0.050 + 2.0
    sigma_m = sqrt(0.03**2 + (tau_s * 0.05) ** 2)
    bounded_speed = min(actor_speed_mps, MAX_ACTOR_SPEED_MPS)
    velocity_delta = MAX_ACTOR_SPEED_MPS + bounded_speed
    acceleration_saturation_s = velocity_delta / MAX_ACTOR_ACCELERATION_MPS2
    if tau_s <= acceleration_saturation_s:
        acceleration_bound_m = 0.5 * MAX_ACTOR_ACCELERATION_MPS2 * tau_s**2
    else:
        acceleration_bound_m = (
            0.5
            * MAX_ACTOR_ACCELERATION_MPS2
            * acceleration_saturation_s**2
            + velocity_delta * (tau_s - acceleration_saturation_s)
        )
    tube_radius_m = ACTOR_RADIUS_M + 2.0 * sigma_m + acceleration_bound_m
    return (
        VIRTUAL_DOLL_WHEELCHAIR_V0_1.collision_width_m
        + 2.0 * tube_radius_m
        + 3.0 * VIRTUAL_DOLL_WHEELCHAIR_V0_1.minimum_clearance_m
    )
