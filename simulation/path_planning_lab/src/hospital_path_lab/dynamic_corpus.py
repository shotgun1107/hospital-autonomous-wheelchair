"""Stage 5 공개 동적 Actor corpus와 contract-fault 목록.

expectation label은 evaluator와 corpus validator만 소유한다. controller용 paired 입력에는
label, split, oracle을 포함하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from hashlib import sha256
from math import isfinite, sqrt
from random import Random

import numpy as np

from hospital_path_lab.collision import (
    CollisionChecker,
    oriented_footprint_circle_surface_distance,
)
from hospital_path_lab.contracts import GridSnapshot, Pose2D, RobotState, SnapshotMetadata
from hospital_path_lab.dynamic_contracts import (
    ACTOR_RADIUS_M,
    DYNAMIC_CONTROL_PERIOD_S,
    MAX_ACTOR_ACCELERATION_MPS2,
    MAX_ACTOR_SPEED_MPS,
    ActorState,
    ControllerSnapshot,
    DynamicGroundTruthFrame,
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
_DURATION_S = 35.0
_MAP_LENGTH_M = 5.0
_GRID_RESOLUTION_M = 0.02
_MISSION_ID = "dynamic-stage5-mission"


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
    width_cells = round(episode.map_length_m / episode.grid_resolution_m)
    height_cells = round(episode.corridor_width_m / episode.grid_resolution_m)
    occupancy = np.zeros((height_cells, width_cells), dtype=np.bool_)
    return GridSnapshot(
        metadata=SnapshotMetadata(
            map_id=episode.map_id,
            map_revision=1,
            mission_revision=1,
            observation_revision=observation_revision,
            seed=episode.seed,
            content_hash=canonical_content_hash(
                {
                    "episode_hash": episode.content_hash,
                    "map_id": episode.map_id,
                    "map_revision": 1,
                    "observation_revision": observation_revision,
                }
            ),
        ),
        grid=GridMap(occupancy, resolution_m=episode.grid_resolution_m),
    )


def generate_episode_ground_truth_frames(
    episode: DynamicCorpusEpisode,
) -> tuple[DynamicGroundTruthFrame, ...]:
    return tuple(
        DynamicGroundTruthFrame(
            episode_id=episode.episode_id,
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
    return generate_dynamic_observation_slots(
        generate_episode_ground_truth_frames(episode),
        source=DynamicObservationSourceIdentity(
            stream_id="dynamic-stage5-stream",
            episode_id=episode.episode_id,
            episode_seed=episode.seed,
            map_id=episode.map_id,
            map_revision=1,
        ),
        profile=profile,
    )


def paired_controller_inputs(
    episode: DynamicCorpusEpisode,
    *,
    profile: DynamicObservationProfile = NORMAL_OBSERVATION_PROFILE,
) -> tuple[DynamicControllerCorpusInput, DynamicControllerCorpusInput]:
    slots = generate_episode_observation_slots(episode, profile=profile)
    stream_hash = canonical_content_hash(slots)
    shared = DynamicControllerCorpusInput(
        episode_id=episode.episode_id,
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
        episode_id=episode.episode_id,
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
