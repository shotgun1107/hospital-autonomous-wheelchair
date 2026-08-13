"""R2 공개 feasible-witness 탐색의 label-free 자료 계약.

이 모듈은 evaluator 정답을 검색 입력에서 제거한다. ground truth Actor trajectory는
offline oracle 전용이며 online controller 또는 shared gate 입력으로 재사용하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from math import isfinite

import numpy as np

from hospital_path_lab.contracts import Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import ActorState, Point2D, Vector2D
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusActor,
    DynamicCorpusEpisode,
    DynamicCorpusSplit,
    build_dynamic_grid_snapshot,
)
from hospital_path_lab.grid import GridMap
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.vehicle import (
    VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    VehicleProfile,
)

WITNESS_WORLD_SCHEMA_VERSION = "dynamic-witness-world-v1"
WITNESS_SCHEMA_VERSION = "automated-dynamic-witness-v1"
WITNESS_SEARCH_CONFIG_VERSION = "structured-witness-search-v1"
WITNESS_VALIDATOR_VERSION = "ground-truth-witness-validator-v1"
WITNESS_CONTROL_PERIOD_S = 0.05
WITNESS_EVALUATOR_PERIOD_S = 0.005
WITNESS_MAX_ANGULAR_ACCELERATION_RADPS2 = 1.60
_PUBLIC_SPLITS = frozenset(
    (DynamicCorpusSplit.GOLDEN, DynamicCorpusSplit.DEVELOPMENT)
)


class PassingPolicy(StrEnum):
    UNSPECIFIED = "unspecified"
    ALLOWED = "allowed"
    PROHIBITED = "prohibited"


class WitnessSearchStatus(StrEnum):
    WITNESS_FOUND = "witness_found"
    NO_WITNESS_IN_STRUCTURED_TEMPLATE = "no_witness_in_structured_template"
    RESOURCE_LIMIT = "resource_limit"
    INVALID_INPUT = "invalid_input"


class WitnessKind(StrEnum):
    PASS_LEFT = "pass_left"
    PASS_RIGHT = "pass_right"
    WAIT_AND_FOLLOW = "wait_and_follow"
    HOLD_ONLY = "hold_only"


class WitnessTerminalMode(StrEnum):
    REJOIN_DWELL = "rejoin_dwell"
    GOAL_DWELL = "goal_dwell"
    SAFE_HOLD = "safe_hold"


class WitnessPhase(StrEnum):
    UNSPECIFIED = "unspecified"
    START = "start"
    BRAKE_TO_STOP = "brake_to_stop"
    WAIT = "wait"
    TURN_OUT = "turn_out"
    MOVE_LATERAL = "move_lateral"
    PASS = "pass"
    TURN_RETURN = "turn_return"
    REJOIN = "rejoin"
    FOLLOW_REFERENCE = "follow_reference"
    TERMINAL_DWELL = "terminal_dwell"
    HOLD = "hold"


@dataclass(frozen=True, slots=True)
class WitnessGridSpec:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    occupied_cells: tuple[tuple[int, int], ...]
    forbidden_cells: tuple[tuple[int, int], ...]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("witness grid dimensions must be positive")
        if not all(
            isfinite(value)
            for value in (self.resolution_m, self.origin_x_m, self.origin_y_m)
        ):
            raise ValueError("witness grid geometry must be finite")
        if self.resolution_m <= 0.0:
            raise ValueError("witness grid resolution must be positive")
        occupied = _normalize_cells(
            self.occupied_cells,
            width=self.width,
            height=self.height,
            field_name="occupied_cells",
        )
        forbidden = _normalize_cells(
            self.forbidden_cells,
            width=self.width,
            height=self.height,
            field_name="forbidden_cells",
        )
        object.__setattr__(self, "occupied_cells", occupied)
        object.__setattr__(self, "forbidden_cells", forbidden)

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)

    def to_grid_map(self) -> GridMap:
        occupancy = np.zeros((self.height, self.width), dtype=np.bool_)
        for x, y in self.occupied_cells:
            occupancy[y, x] = True
        return GridMap(
            occupancy,
            resolution_m=self.resolution_m,
            origin_x_m=self.origin_x_m,
            origin_y_m=self.origin_y_m,
        )


@dataclass(frozen=True, slots=True)
class ManeuverConstraintSpec:
    policy_revision: int = 1
    passing_policy: PassingPolicy = PassingPolicy.UNSPECIFIED
    allowed_cells: tuple[tuple[int, int], ...] = ()

    def __post_init__(self) -> None:
        if self.policy_revision < 0:
            raise ValueError("maneuver policy revision must not be negative")
        if not isinstance(self.passing_policy, PassingPolicy):
            raise TypeError("passing_policy must be a PassingPolicy")
        normalized = tuple(sorted(set(self.allowed_cells)))
        if any(
            not isinstance(cell, tuple)
            or len(cell) != 2
            or any(not isinstance(value, int) for value in cell)
            for cell in normalized
        ):
            raise TypeError("allowed_cells must contain integer (x, y) tuples")
        object.__setattr__(self, "allowed_cells", normalized)

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class WitnessSearchConfig:
    config_version: str = WITNESS_SEARCH_CONFIG_VERSION
    geometry_progress_step_m: float = 0.10
    linear_targets_mps: tuple[float, ...] = (0.10, 0.15, 0.20, 0.25, 0.30)
    angular_targets_radps: tuple[float, ...] = (
        -0.80,
        -0.60,
        -0.40,
        0.0,
        0.40,
        0.60,
        0.80,
    )
    control_period_s: float = WITNESS_CONTROL_PERIOD_S
    evaluator_period_s: float = WITNESS_EVALUATOR_PERIOD_S
    maximum_angular_acceleration_radps2: float = (
        WITNESS_MAX_ANGULAR_ACCELERATION_RADPS2
    )
    reverse_enabled: bool = False
    max_geometry_candidates_per_episode: int = 50_000
    max_timed_candidates_per_episode: int = 250_000

    def __post_init__(self) -> None:
        if self.config_version != WITNESS_SEARCH_CONFIG_VERSION:
            raise ValueError("unsupported witness search config version")
        numeric = (
            self.geometry_progress_step_m,
            self.control_period_s,
            self.evaluator_period_s,
            self.maximum_angular_acceleration_radps2,
            *self.linear_targets_mps,
            *self.angular_targets_radps,
        )
        if not all(isfinite(value) for value in numeric):
            raise ValueError("witness search config must be finite")
        if min(
            self.geometry_progress_step_m,
            self.control_period_s,
            self.evaluator_period_s,
            self.maximum_angular_acceleration_radps2,
        ) <= 0.0:
            raise ValueError("witness search steps and limits must be positive")
        if self.max_geometry_candidates_per_episode <= 0:
            raise ValueError("geometry candidate limit must be positive")
        if self.max_timed_candidates_per_episode <= 0:
            raise ValueError("timed candidate limit must be positive")
        if tuple(sorted(set(self.linear_targets_mps))) != self.linear_targets_mps:
            raise ValueError("linear targets must be unique and sorted")
        if tuple(sorted(set(self.angular_targets_radps))) != self.angular_targets_radps:
            raise ValueError("angular targets must be unique and sorted")
        if not self.linear_targets_mps or self.linear_targets_mps[0] <= 0.0:
            raise ValueError("linear targets must be non-empty and positive")
        if not self.angular_targets_radps or 0.0 not in self.angular_targets_radps:
            raise ValueError("angular targets must include zero")

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


FROZEN_WITNESS_SEARCH_CONFIG = WitnessSearchConfig()


@dataclass(frozen=True, slots=True)
class WitnessActorTrajectory:
    actor_binding_id: str
    active_from_s: float
    active_until_s: float
    start_position: Point2D
    velocity: Vector2D
    radius_m: float
    trajectory_revision: int

    def __post_init__(self) -> None:
        if not self.actor_binding_id:
            raise ValueError("witness Actor binding must not be empty")
        if not all(
            isfinite(value)
            for value in (
                self.active_from_s,
                self.active_until_s,
                self.start_position.x,
                self.start_position.y,
                self.velocity.x,
                self.velocity.y,
                self.radius_m,
            )
        ):
            raise ValueError("witness Actor trajectory must be finite")
        if self.active_from_s < 0.0 or self.active_until_s <= self.active_from_s:
            raise ValueError("witness Actor interval must be positive and ordered")
        if self.radius_m <= 0.0:
            raise ValueError("witness Actor radius must be positive")
        if self.trajectory_revision < 0:
            raise ValueError("witness Actor revision must not be negative")

    def state_at(self, simulation_time_s: float) -> ActorState | None:
        if not isfinite(simulation_time_s):
            raise ValueError("Actor query time must be finite")
        if not self.active_from_s <= simulation_time_s <= self.active_until_s:
            return None
        elapsed_s = simulation_time_s - self.active_from_s
        return ActorState(
            actor_id=self.actor_binding_id,
            position=Point2D(
                self.start_position.x + self.velocity.x * elapsed_s,
                self.start_position.y + self.velocity.y * elapsed_s,
            ),
            velocity=self.velocity,
            radius_m=self.radius_m,
            trajectory_revision=self.trajectory_revision,
        )


@dataclass(frozen=True, slots=True)
class WitnessKinematicContract:
    vehicle_profile: VehicleProfile
    maximum_angular_acceleration_radps2: float
    control_period_s: float
    evaluator_period_s: float

    def __post_init__(self) -> None:
        if not self.vehicle_profile.simulation_only:
            raise ValueError("witness vehicle profile must remain simulation_only")
        values = (
            self.maximum_angular_acceleration_radps2,
            self.control_period_s,
            self.evaluator_period_s,
        )
        if not all(isfinite(value) and value > 0.0 for value in values):
            raise ValueError("witness kinematic contract must be finite and positive")
        subdivisions = self.control_period_s / self.evaluator_period_s
        if abs(subdivisions - round(subdivisions)) > 1e-12:
            raise ValueError("evaluator period must divide the control period")

    @property
    def vehicle_profile_hash(self) -> str:
        return canonical_content_hash(self.vehicle_profile)


@dataclass(frozen=True, slots=True)
class WitnessWorldSnapshot:
    schema_version: str
    source_schema_version: str
    source_generator_version: str
    source_projection_hash: str
    world_id: str
    seed: int
    simulation_only: bool
    map_id: str
    map_revision: int
    grid_content_hash: str
    grid: WitnessGridSpec
    reference_path: tuple[Pose2D, ...]
    initial_state: RobotState
    goal_pose: Pose2D
    duration_s: float
    actors: tuple[WitnessActorTrajectory, ...]
    maneuver_constraints: ManeuverConstraintSpec
    kinematic_contract: WitnessKinematicContract
    search_config_hash: str

    def __post_init__(self) -> None:
        if self.schema_version != WITNESS_WORLD_SCHEMA_VERSION:
            raise ValueError("unsupported witness world schema")
        if not all(
            (
                self.source_schema_version,
                self.source_generator_version,
                self.source_projection_hash,
                self.world_id,
                self.map_id,
                self.grid_content_hash,
                self.search_config_hash,
            )
        ):
            raise ValueError("witness world identity fields must not be empty")
        if not self.simulation_only:
            raise ValueError("witness world must remain simulation_only")
        if self.map_revision < 0:
            raise ValueError("witness map revision must not be negative")
        if not isfinite(self.duration_s) or self.duration_s <= 0.0:
            raise ValueError("witness duration must be finite and positive")
        reference = tuple(self.reference_path)
        actors = tuple(self.actors)
        if len(reference) < 2:
            raise ValueError("witness reference path requires at least two poses")
        if len({actor.actor_binding_id for actor in actors}) != len(actors):
            raise ValueError("witness Actor bindings must be unique")
        _require_finite_pose("initial pose", self.initial_state.pose)
        _require_finite_twist("initial twist", self.initial_state.twist)
        _require_finite_pose("goal pose", self.goal_pose)
        for pose in reference:
            _require_finite_pose("reference pose", pose)
        if self.grid_content_hash != self.grid.content_hash:
            raise ValueError("witness grid content hash mismatch")
        expected_map_id = (
            "witness-map-"
            f"{canonical_content_hash({'grid': self.grid, 'map_revision': self.map_revision})[:24]}"
        )
        if self.map_id != expected_map_id:
            raise ValueError("witness map id is not bound to grid content")
        expected_source_hash = canonical_content_hash(
            _source_projection_payload(
                source_schema_version=self.source_schema_version,
                source_generator_version=self.source_generator_version,
                seed=self.seed,
                map_id=self.map_id,
                map_revision=self.map_revision,
                grid=self.grid,
                reference_path=reference,
                initial_state=self.initial_state,
                goal_pose=self.goal_pose,
                duration_s=self.duration_s,
                actors=actors,
                maneuver_constraints=self.maneuver_constraints,
                kinematic_contract=self.kinematic_contract,
                search_config_hash=self.search_config_hash,
            )
        )
        if self.source_projection_hash != expected_source_hash:
            raise ValueError("witness source projection hash mismatch")
        if self.world_id != f"witness-world-{expected_source_hash[:24]}":
            raise ValueError("witness world id is not bound to projected content")
        allowed = self.maneuver_constraints.allowed_cells
        if any(
            not (0 <= x < self.grid.width and 0 <= y < self.grid.height)
            for x, y in allowed
        ):
            raise ValueError("allowed maneuver cells must be inside the grid")
        object.__setattr__(self, "reference_path", reference)
        object.__setattr__(self, "actors", actors)

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)

    @property
    def vehicle_profile_hash(self) -> str:
        return self.kinematic_contract.vehicle_profile_hash

    def actor_states_at(self, simulation_time_s: float) -> tuple[ActorState, ...]:
        if (
            not isfinite(simulation_time_s)
            or simulation_time_s < 0.0
            or simulation_time_s > self.duration_s + 1e-9
        ):
            raise ValueError("witness Actor query is outside the world duration")
        query_time_s = min(simulation_time_s, self.duration_s)
        states = tuple(actor.state_at(query_time_s) for actor in self.actors)
        return tuple(state for state in states if state is not None)


@dataclass(frozen=True, slots=True)
class WitnessPoint:
    time_s: float
    pose: Pose2D
    twist: Twist2D
    phase: WitnessPhase = WitnessPhase.UNSPECIFIED
    source_primitive_id: str = "unspecified"

    def __post_init__(self) -> None:
        if not isfinite(self.time_s) or self.time_s < 0.0:
            raise ValueError("witness point time must be finite and non-negative")
        _require_finite_pose("witness pose", self.pose)
        _require_finite_twist("witness twist", self.twist)
        if not isinstance(self.phase, WitnessPhase):
            raise TypeError("witness phase must be a WitnessPhase")
        if not self.source_primitive_id:
            raise ValueError("witness source primitive id must not be empty")


@dataclass(frozen=True, slots=True)
class AutomatedWitness:
    schema_version: str
    witness_id: str
    source_projection_hash: str
    world_content_hash: str
    vehicle_profile_hash: str
    search_config_hash: str
    kind: WitnessKind
    terminal_mode: WitnessTerminalMode
    points: tuple[WitnessPoint, ...]
    required_pass_actor_ids: tuple[str, ...] = ()
    departure_time_s: float | None = None
    pass_times_by_actor: tuple[tuple[str, float], ...] = ()
    rejoin_started_at_s: float | None = None
    rejoin_confirmed_at_s: float | None = None
    terminal_dwell_s: float = 0.50

    def __post_init__(self) -> None:
        if self.schema_version != WITNESS_SCHEMA_VERSION:
            raise ValueError("unsupported automated witness schema")
        if not all(
            (
                self.witness_id,
                self.source_projection_hash,
                self.world_content_hash,
                self.vehicle_profile_hash,
                self.search_config_hash,
            )
        ):
            raise ValueError("automated witness identity fields must not be empty")
        points = tuple(self.points)
        actor_ids = tuple(self.required_pass_actor_ids)
        pass_times = tuple(self.pass_times_by_actor)
        if not isinstance(self.kind, WitnessKind):
            raise TypeError("witness kind must be a WitnessKind")
        if not isinstance(self.terminal_mode, WitnessTerminalMode):
            raise TypeError("terminal_mode must be a WitnessTerminalMode")
        if len(points) < 3:
            raise ValueError("automated witness requires at least three points")
        if any(
            right.time_s <= left.time_s
            for left, right in zip(points, points[1:], strict=False)
        ):
            raise ValueError("automated witness times must be strictly increasing")
        if len(actor_ids) != len(set(actor_ids)):
            raise ValueError("required pass Actor ids must be unique")
        if self.kind in (WitnessKind.PASS_LEFT, WitnessKind.PASS_RIGHT) and not actor_ids:
            raise ValueError("pass witness requires at least one Actor binding")
        if self.kind not in (WitnessKind.PASS_LEFT, WitnessKind.PASS_RIGHT) and actor_ids:
            raise ValueError("non-pass witness must not declare pass Actor ids")
        if (self.kind is WitnessKind.HOLD_ONLY) != (
            self.terminal_mode is WitnessTerminalMode.SAFE_HOLD
        ):
            raise ValueError("HOLD_ONLY and SAFE_HOLD must be used together")
        if len({actor_id for actor_id, _ in pass_times}) != len(pass_times):
            raise ValueError("pass times must have unique Actor ids")
        if any(
            not actor_id or not isfinite(time_s) or time_s < 0.0
            for actor_id, time_s in pass_times
        ):
            raise ValueError("pass times must contain valid Actor ids and times")
        if any(actor_id not in actor_ids for actor_id, _ in pass_times):
            raise ValueError("declared pass times must reference required Actor ids")
        optional_times = (
            self.departure_time_s,
            self.rejoin_started_at_s,
            self.rejoin_confirmed_at_s,
        )
        if any(
            value is not None and (not isfinite(value) or value < 0.0)
            for value in optional_times
        ):
            raise ValueError("witness event times must be finite and non-negative")
        if (
            self.departure_time_s is not None
            and pass_times
            and self.departure_time_s > min(time_s for _, time_s in pass_times)
        ):
            raise ValueError("departure must not follow a declared pass")
        if (
            self.rejoin_started_at_s is not None
            and self.rejoin_confirmed_at_s is not None
            and self.rejoin_started_at_s > self.rejoin_confirmed_at_s
        ):
            raise ValueError("rejoin confirmation must not precede rejoin start")
        if not isfinite(self.terminal_dwell_s) or self.terminal_dwell_s < 0.50:
            raise ValueError("witness terminal dwell must cover at least 0.5 s")
        object.__setattr__(self, "points", points)
        object.__setattr__(self, "required_pass_actor_ids", actor_ids)
        object.__setattr__(self, "pass_times_by_actor", pass_times)

    @property
    def semantic_content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class WitnessObjective:
    hard_failure_count: int
    terminal_completion_time_s: float
    actual_path_length_m: float
    maximum_reference_deviation_m: float
    full_stop_count: int
    absolute_angular_travel_rad: float
    kind_rank: int
    frozen_parameter_tuple: tuple[float, ...]

    def __post_init__(self) -> None:
        if self.hard_failure_count < 0 or self.full_stop_count < 0:
            raise ValueError("witness objective counts must not be negative")
        numeric = (
            self.terminal_completion_time_s,
            self.actual_path_length_m,
            self.maximum_reference_deviation_m,
            self.absolute_angular_travel_rad,
            *self.frozen_parameter_tuple,
        )
        if not all(isfinite(value) and value >= 0.0 for value in numeric):
            raise ValueError("witness objective values must be finite and non-negative")
        if self.kind_rank < 0:
            raise ValueError("witness kind rank must not be negative")

    @property
    def sort_key(self) -> tuple[object, ...]:
        return (
            self.hard_failure_count,
            self.terminal_completion_time_s,
            self.actual_path_length_m,
            self.maximum_reference_deviation_m,
            self.full_stop_count,
            self.absolute_angular_travel_rad,
            self.kind_rank,
            self.frozen_parameter_tuple,
        )


@dataclass(frozen=True, slots=True)
class WitnessSearchResult:
    status: WitnessSearchStatus
    source_projection_hash: str
    world_content_hash: str
    search_config_hash: str
    generated_count: int
    geometry_pruned_count: int
    dynamic_rejected_count: int
    validated_count: int
    selected_witness: AutomatedWitness | None
    termination_reason: str
    deterministic_objective: WitnessObjective | None
    elapsed_nonqualification_ns: int
    validator_version: str = WITNESS_VALIDATOR_VERSION
    selected_validation_hash: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.status, WitnessSearchStatus):
            raise TypeError("status must be a WitnessSearchStatus")
        if not all(
            (
                self.source_projection_hash,
                self.world_content_hash,
                self.search_config_hash,
                self.termination_reason,
            )
        ):
            raise ValueError("witness search result identity must not be empty")
        if self.validator_version != WITNESS_VALIDATOR_VERSION:
            raise ValueError("unsupported witness validator version")
        counts = (
            self.generated_count,
            self.geometry_pruned_count,
            self.dynamic_rejected_count,
            self.validated_count,
            self.elapsed_nonqualification_ns,
        )
        if any(value < 0 for value in counts):
            raise ValueError("witness search result counts must not be negative")
        found = self.status is WitnessSearchStatus.WITNESS_FOUND
        if found != (self.selected_witness is not None):
            raise ValueError("WITNESS_FOUND must carry exactly one selected witness")
        if found != (self.deterministic_objective is not None):
            raise ValueError("WITNESS_FOUND must carry one deterministic objective")
        if found != (self.selected_validation_hash is not None):
            raise ValueError("WITNESS_FOUND must carry one validation hash")
        if found:
            assert self.selected_witness is not None
            assert self.selected_validation_hash is not None
            if len(self.selected_validation_hash) != 64 or any(
                character not in "0123456789abcdef"
                for character in self.selected_validation_hash
            ):
                raise ValueError("selected validation hash must be lowercase SHA-256")
            if any(
                (
                    self.selected_witness.source_projection_hash
                    != self.source_projection_hash,
                    self.selected_witness.world_content_hash
                    != self.world_content_hash,
                    self.selected_witness.search_config_hash
                    != self.search_config_hash,
                )
            ):
                raise ValueError("selected witness provenance does not match result")
            if self.validated_count <= 0:
                raise ValueError("WITNESS_FOUND requires a validated candidate")
        if (
            self.geometry_pruned_count
            + self.dynamic_rejected_count
            + self.validated_count
            != self.generated_count
        ):
            raise ValueError("witness search result counts are inconsistent")

    @property
    def semantic_content_hash(self) -> str:
        payload = {
            "status": self.status,
            "source_projection_hash": self.source_projection_hash,
            "world_content_hash": self.world_content_hash,
            "search_config_hash": self.search_config_hash,
            "generated_count": self.generated_count,
            "geometry_pruned_count": self.geometry_pruned_count,
            "dynamic_rejected_count": self.dynamic_rejected_count,
            "validated_count": self.validated_count,
            "selected_witness": self.selected_witness,
            "termination_reason": self.termination_reason,
            "deterministic_objective": self.deterministic_objective,
            "validator_version": self.validator_version,
            "selected_validation_hash": self.selected_validation_hash,
        }
        return canonical_content_hash(payload)


def project_public_witness_world(
    episode: DynamicCorpusEpisode,
    *,
    maneuver_constraints: ManeuverConstraintSpec | None = None,
    search_config: WitnessSearchConfig = FROZEN_WITNESS_SEARCH_CONFIG,
) -> WitnessWorldSnapshot:
    """Create a label-free offline world from an allowed public episode."""

    if not isinstance(episode, DynamicCorpusEpisode):
        raise TypeError("witness projection requires a dynamic corpus episode")
    if episode.split not in _PUBLIC_SPLITS:
        raise ValueError("witness projection rejects hidden or unsupported splits")
    if not episode.simulation_only:
        raise ValueError("witness projection requires simulation_only input")
    if not isinstance(search_config, WitnessSearchConfig):
        raise TypeError("search_config must be a WitnessSearchConfig")
    constraints = maneuver_constraints or ManeuverConstraintSpec()
    if not isinstance(constraints, ManeuverConstraintSpec):
        raise TypeError("maneuver_constraints must be a ManeuverConstraintSpec")

    snapshot = build_dynamic_grid_snapshot(episode)
    occupied_y, occupied_x = np.nonzero(snapshot.grid.occupancy)
    grid = WitnessGridSpec(
        width=snapshot.grid.width,
        height=snapshot.grid.height,
        resolution_m=snapshot.grid.resolution_m,
        origin_x_m=snapshot.grid.origin_x_m,
        origin_y_m=snapshot.grid.origin_y_m,
        occupied_cells=tuple(
            (int(x), int(y))
            for y, x in zip(occupied_y, occupied_x, strict=True)
        ),
        forbidden_cells=tuple(sorted(snapshot.forbidden_cells)),
    )
    actors = tuple(
        _project_actor(index, actor)
        for index, actor in enumerate(episode.actors)
    )
    kinematics = WitnessKinematicContract(
        vehicle_profile=VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        maximum_angular_acceleration_radps2=(
            search_config.maximum_angular_acceleration_radps2
        ),
        control_period_s=search_config.control_period_s,
        evaluator_period_s=search_config.evaluator_period_s,
    )
    map_payload = {
        "grid": grid,
        "map_revision": 1,
    }
    map_id = f"witness-map-{canonical_content_hash(map_payload)[:24]}"
    world_payload = _source_projection_payload(
        source_schema_version=episode.schema_version,
        source_generator_version=episode.generator_version,
        seed=episode.seed,
        map_id=map_id,
        map_revision=1,
        grid=grid,
        reference_path=episode.reference_path,
        initial_state=episode.initial_state,
        goal_pose=episode.goal_pose,
        duration_s=episode.duration_s,
        actors=actors,
        maneuver_constraints=constraints,
        kinematic_contract=kinematics,
        search_config_hash=search_config.content_hash,
    )
    source_projection_hash = canonical_content_hash(world_payload)
    world_id = f"witness-world-{source_projection_hash[:24]}"
    return WitnessWorldSnapshot(
        schema_version=WITNESS_WORLD_SCHEMA_VERSION,
        source_schema_version=episode.schema_version,
        source_generator_version=episode.generator_version,
        source_projection_hash=source_projection_hash,
        world_id=world_id,
        seed=episode.seed,
        simulation_only=True,
        map_id=map_id,
        map_revision=1,
        grid_content_hash=grid.content_hash,
        grid=grid,
        reference_path=episode.reference_path,
        initial_state=episode.initial_state,
        goal_pose=episode.goal_pose,
        duration_s=episode.duration_s,
        actors=actors,
        maneuver_constraints=constraints,
        kinematic_contract=kinematics,
        search_config_hash=search_config.content_hash,
    )


def build_automated_witness(
    world: WitnessWorldSnapshot,
    *,
    witness_id: str,
    kind: WitnessKind,
    terminal_mode: WitnessTerminalMode,
    points: tuple[WitnessPoint, ...],
    required_pass_actor_ids: tuple[str, ...] = (),
    departure_time_s: float | None = None,
    pass_times_by_actor: tuple[tuple[str, float], ...] = (),
    rejoin_started_at_s: float | None = None,
    rejoin_confirmed_at_s: float | None = None,
    terminal_dwell_s: float = 0.50,
) -> AutomatedWitness:
    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    return AutomatedWitness(
        schema_version=WITNESS_SCHEMA_VERSION,
        witness_id=witness_id,
        source_projection_hash=world.source_projection_hash,
        world_content_hash=world.content_hash,
        vehicle_profile_hash=world.vehicle_profile_hash,
        search_config_hash=world.search_config_hash,
        kind=kind,
        terminal_mode=terminal_mode,
        points=points,
        required_pass_actor_ids=required_pass_actor_ids,
        departure_time_s=departure_time_s,
        pass_times_by_actor=pass_times_by_actor,
        rejoin_started_at_s=rejoin_started_at_s,
        rejoin_confirmed_at_s=rejoin_confirmed_at_s,
        terminal_dwell_s=terminal_dwell_s,
    )


def _project_actor(index: int, actor: DynamicCorpusActor) -> WitnessActorTrajectory:
    geometry = {
        "index": index,
        "active_from_s": actor.active_from_s,
        "active_until_s": actor.active_until_s,
        "start_position": actor.start_position,
        "velocity": actor.velocity,
        "radius_m": actor.radius_m,
        "trajectory_revision": actor.trajectory_revision,
    }
    opaque_hash = canonical_content_hash(geometry)[:12]
    return WitnessActorTrajectory(
        actor_binding_id=f"actor-{index:03d}-{opaque_hash}",
        active_from_s=actor.active_from_s,
        active_until_s=actor.active_until_s,
        start_position=actor.start_position,
        velocity=actor.velocity,
        radius_m=actor.radius_m,
        trajectory_revision=actor.trajectory_revision,
    )


def _source_projection_payload(
    *,
    source_schema_version: str,
    source_generator_version: str,
    seed: int,
    map_id: str,
    map_revision: int,
    grid: WitnessGridSpec,
    reference_path: tuple[Pose2D, ...],
    initial_state: RobotState,
    goal_pose: Pose2D,
    duration_s: float,
    actors: tuple[WitnessActorTrajectory, ...],
    maneuver_constraints: ManeuverConstraintSpec,
    kinematic_contract: WitnessKinematicContract,
    search_config_hash: str,
) -> dict[str, object]:
    return {
        "source_schema_version": source_schema_version,
        "source_generator_version": source_generator_version,
        "seed": seed,
        "map_id": map_id,
        "map_revision": map_revision,
        "grid": grid,
        "reference_path": reference_path,
        "initial_state": initial_state,
        "goal_pose": goal_pose,
        "duration_s": duration_s,
        "actors": actors,
        "maneuver_constraints": maneuver_constraints,
        "kinematic_contract": kinematic_contract,
        "search_config_hash": search_config_hash,
    }


def _normalize_cells(
    cells: tuple[tuple[int, int], ...],
    *,
    width: int,
    height: int,
    field_name: str,
) -> tuple[tuple[int, int], ...]:
    normalized = tuple(sorted(set(cells)))
    if any(
        not isinstance(cell, tuple)
        or len(cell) != 2
        or any(not isinstance(value, int) for value in cell)
        for cell in normalized
    ):
        raise TypeError(f"{field_name} must contain integer (x, y) tuples")
    if any(not (0 <= x < width and 0 <= y < height) for x, y in normalized):
        raise ValueError(f"{field_name} must remain inside the grid")
    return normalized


def _require_finite_pose(name: str, pose: Pose2D) -> None:
    if not all(isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
        raise ValueError(f"{name} must be finite")


def _require_finite_twist(name: str, twist: Twist2D) -> None:
    if not all(isfinite(value) for value in (twist.linear, twist.angular)):
        raise ValueError(f"{name} must be finite")


__all__ = [
    "AutomatedWitness",
    "FROZEN_WITNESS_SEARCH_CONFIG",
    "ManeuverConstraintSpec",
    "PassingPolicy",
    "WITNESS_CONTROL_PERIOD_S",
    "WITNESS_EVALUATOR_PERIOD_S",
    "WITNESS_SCHEMA_VERSION",
    "WITNESS_VALIDATOR_VERSION",
    "WITNESS_WORLD_SCHEMA_VERSION",
    "WitnessActorTrajectory",
    "WitnessGridSpec",
    "WitnessKind",
    "WitnessKinematicContract",
    "WitnessPhase",
    "WitnessPoint",
    "WitnessObjective",
    "WitnessSearchConfig",
    "WitnessSearchResult",
    "WitnessSearchStatus",
    "WitnessTerminalMode",
    "WitnessWorldSnapshot",
    "build_automated_witness",
    "project_public_witness_world",
]
