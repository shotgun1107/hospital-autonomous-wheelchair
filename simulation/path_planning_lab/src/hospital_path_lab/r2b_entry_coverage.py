"""Simulation-only R2-B monitored-entry coverage research contract.

The source world remains the frozen negative regression.  A derived observation
world extends delayed Actor trajectories backwards into an abstract monitored
approach zone.  No entry time, oracle label, or ground truth is exposed to a
controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isclose, isfinite

from hospital_path_lab.dynamic_contracts import Point2D
from hospital_path_lab.dynamic_directional_prediction import (
    FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS,
)
from hospital_path_lab.dynamic_observation import (
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
)
from hospital_path_lab.dynamic_witness_contracts import (
    AutomatedWitness,
    WitnessActorTrajectory,
    WitnessKind,
    WitnessWorldSnapshot,
    build_automated_witness,
    derive_witness_world_with_actors,
)
from hospital_path_lab.dynamic_witness_profile_replay import (
    WitnessProfileReplayBundle,
    replay_witness_profiles,
)
from hospital_path_lab.dynamic_witness_validation import (
    GroundTruthWitnessValidation,
    validate_ground_truth_witness,
)
from hospital_path_lab.map_factory import canonical_content_hash

R2B_ENTRY_COVERAGE_SCHEMA_VERSION = "r2b-entry-coverage-v1"
_OBSERVATION_RATE_HZ = 10.0
_TIME_TOLERANCE_S = 1e-12
_POSITION_TOLERANCE_M = 1e-12
_PASS_KINDS = frozenset(
    (
        WitnessKind.PASS_LEFT,
        WitnessKind.PASS_RIGHT,
        WitnessKind.CROSSING_BYPASS_LEFT,
        WitnessKind.CROSSING_BYPASS_RIGHT,
    )
)


@dataclass(frozen=True, slots=True)
class R2BMonitoredEntryApproach:
    actor_binding_id: str
    monitored_from_s: float
    entry_time_s: float
    monitored_start_x_m: float
    monitored_start_y_m: float

    def __post_init__(self) -> None:
        if not self.actor_binding_id:
            raise ValueError("entry approach Actor binding must not be empty")
        values = (
            self.monitored_from_s,
            self.entry_time_s,
            self.monitored_start_x_m,
            self.monitored_start_y_m,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("entry approach values must be finite")
        if self.monitored_from_s < 0.0:
            raise ValueError("monitored entry must start inside the episode")
        if self.entry_time_s <= self.monitored_from_s:
            raise ValueError("entry must follow monitored approach start")


@dataclass(frozen=True, slots=True)
class R2BEntryCoverageContract:
    schema_version: str
    source_projection_hash: str
    source_world_content_hash: str
    history_frame_count: int
    observation_rate_hz: float
    ideal_latency_s: float
    required_lead_time_s: float
    approaches: tuple[R2BMonitoredEntryApproach, ...]

    def __post_init__(self) -> None:
        if self.schema_version != R2B_ENTRY_COVERAGE_SCHEMA_VERSION:
            raise ValueError("unsupported R2-B entry coverage schema")
        if not self.source_projection_hash or not self.source_world_content_hash:
            raise ValueError("entry coverage source identity must not be empty")
        expected_history = FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS.history_frame_count
        if self.history_frame_count != expected_history:
            raise ValueError("entry coverage must preserve the frozen 20-frame history")
        if self.observation_rate_hz != _OBSERVATION_RATE_HZ:
            raise ValueError("entry coverage observation rate must remain 10 Hz")
        if self.ideal_latency_s != FUNCTIONAL_IDEAL_OBSERVATION_PROFILE.latency_s:
            raise ValueError("entry coverage must preserve the Ideal latency")
        expected_lead = (
            (self.history_frame_count - 1) / self.observation_rate_hz
            + self.ideal_latency_s
        )
        if not isclose(
            self.required_lead_time_s,
            expected_lead,
            rel_tol=0.0,
            abs_tol=_TIME_TOLERANCE_S,
        ):
            raise ValueError("entry coverage lead time does not cover history and latency")
        approaches = tuple(self.approaches)
        actor_ids = tuple(item.actor_binding_id for item in approaches)
        if not approaches:
            raise ValueError("entry coverage requires at least one delayed Actor")
        if len(actor_ids) != len(set(actor_ids)):
            raise ValueError("entry coverage Actor bindings must be unique")
        if any(
            item.entry_time_s - item.monitored_from_s
            < self.required_lead_time_s - _TIME_TOLERANCE_S
            for item in approaches
        ):
            raise ValueError("entry approach does not provide enough prediction lead time")
        object.__setattr__(self, "approaches", approaches)

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class R2BEntryCoverageReplay:
    schema_version: str
    contract: R2BEntryCoverageContract
    source_world_content_hash: str
    covered_world_content_hash: str
    source_witness_content_hash: str
    covered_witness_content_hash: str
    source_validation_hash: str
    covered_validation_hash: str
    source_profiles: WitnessProfileReplayBundle
    covered_profiles: WitnessProfileReplayBundle
    limitations: tuple[str, ...]

    def __post_init__(self) -> None:
        if self.schema_version != R2B_ENTRY_COVERAGE_SCHEMA_VERSION:
            raise ValueError("unsupported R2-B entry coverage replay schema")
        if self.contract.source_world_content_hash != self.source_world_content_hash:
            raise ValueError("entry coverage contract is not bound to the source world")
        if self.source_profiles.world_content_hash != self.source_world_content_hash:
            raise ValueError("source profile replay is not bound to the source world")
        if self.covered_profiles.world_content_hash != self.covered_world_content_hash:
            raise ValueError("covered profile replay is not bound to the covered world")
        if self.source_profiles.witness_content_hash != self.source_witness_content_hash:
            raise ValueError("source profile replay is not bound to the source witness")
        if self.covered_profiles.witness_content_hash != self.covered_witness_content_hash:
            raise ValueError("covered profile replay is not bound to the covered witness")
        if self.source_profiles.ground_truth_validation_hash != self.source_validation_hash:
            raise ValueError("source profile replay validation identity mismatch")
        if self.covered_profiles.ground_truth_validation_hash != self.covered_validation_hash:
            raise ValueError("covered profile replay validation identity mismatch")
        object.__setattr__(self, "limitations", tuple(sorted(set(self.limitations))))

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


def build_r2b_entry_coverage_contract(
    world: WitnessWorldSnapshot,
) -> R2BEntryCoverageContract:
    """Bind every delayed Actor to a deterministic monitored approach."""

    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    history_count = FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS.history_frame_count
    latency_s = FUNCTIONAL_IDEAL_OBSERVATION_PROFILE.latency_s
    required_lead_s = (history_count - 1) / _OBSERVATION_RATE_HZ + latency_s
    approaches: list[R2BMonitoredEntryApproach] = []
    for actor in world.actors:
        if actor.active_from_s <= _TIME_TOLERANCE_S:
            continue
        if actor.velocity.magnitude <= 1e-12:
            raise ValueError("delayed Actor cannot establish a moving entry approach")
        monitored_from_s = 0.0
        elapsed_s = actor.active_from_s - monitored_from_s
        approaches.append(
            R2BMonitoredEntryApproach(
                actor_binding_id=actor.actor_binding_id,
                monitored_from_s=monitored_from_s,
                entry_time_s=actor.active_from_s,
                monitored_start_x_m=(
                    actor.start_position.x - actor.velocity.x * elapsed_s
                ),
                monitored_start_y_m=(
                    actor.start_position.y - actor.velocity.y * elapsed_s
                ),
            )
        )
    return R2BEntryCoverageContract(
        schema_version=R2B_ENTRY_COVERAGE_SCHEMA_VERSION,
        source_projection_hash=world.source_projection_hash,
        source_world_content_hash=world.content_hash,
        history_frame_count=history_count,
        observation_rate_hz=_OBSERVATION_RATE_HZ,
        ideal_latency_s=latency_s,
        required_lead_time_s=required_lead_s,
        approaches=tuple(approaches),
    )


def derive_r2b_covered_world(
    world: WitnessWorldSnapshot,
    contract: R2BEntryCoverageContract,
) -> WitnessWorldSnapshot:
    """Derive an observation world while preserving every post-entry state."""

    if not isinstance(world, WitnessWorldSnapshot):
        raise TypeError("world must be a WitnessWorldSnapshot")
    if not isinstance(contract, R2BEntryCoverageContract):
        raise TypeError("contract must be an R2BEntryCoverageContract")
    if (
        contract.source_projection_hash != world.source_projection_hash
        or contract.source_world_content_hash != world.content_hash
    ):
        raise ValueError("entry coverage contract source identity mismatch")
    approach_by_actor = {item.actor_binding_id: item for item in contract.approaches}
    delayed_ids = {
        actor.actor_binding_id
        for actor in world.actors
        if actor.active_from_s > _TIME_TOLERANCE_S
    }
    if set(approach_by_actor) != delayed_ids:
        raise ValueError("entry coverage must bind every and only delayed Actor")

    covered_actors: list[WitnessActorTrajectory] = []
    for actor in world.actors:
        approach = approach_by_actor.get(actor.actor_binding_id)
        if approach is None:
            covered_actors.append(actor)
            continue
        if not isclose(
            approach.entry_time_s,
            actor.active_from_s,
            rel_tol=0.0,
            abs_tol=_TIME_TOLERANCE_S,
        ):
            raise ValueError("entry coverage time does not match source Actor")
        covered_actor = WitnessActorTrajectory(
            actor_binding_id=actor.actor_binding_id,
            active_from_s=approach.monitored_from_s,
            active_until_s=actor.active_until_s,
            start_position=Point2D(
                approach.monitored_start_x_m,
                approach.monitored_start_y_m,
            ),
            velocity=actor.velocity,
            radius_m=actor.radius_m,
            trajectory_revision=actor.trajectory_revision,
        )
        _require_entry_continuity(actor, covered_actor, approach.entry_time_s)
        covered_actors.append(covered_actor)
    covered_world = derive_witness_world_with_actors(world, tuple(covered_actors))
    if covered_world.content_hash == world.content_hash:
        raise ValueError("entry coverage must create a distinct derived world")
    return covered_world


def replay_r2b_entry_coverage(
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
    source_validation: GroundTruthWitnessValidation,
) -> R2BEntryCoverageReplay:
    """Compare the frozen negative world with its monitored-entry derivation."""

    if not source_validation.passed:
        raise ValueError("R2-B entry coverage requires a valid source witness")
    if (
        source_validation.source_projection_hash != world.source_projection_hash
        or source_validation.world_content_hash != world.content_hash
        or source_validation.witness_content_hash != witness.semantic_content_hash
    ):
        raise ValueError("source validation provenance mismatch")
    contract = build_r2b_entry_coverage_contract(world)
    covered_world = derive_r2b_covered_world(world, contract)
    covered_witness = build_automated_witness(
        covered_world,
        witness_id=f"{witness.witness_id}-monitored-entry",
        kind=witness.kind,
        terminal_mode=witness.terminal_mode,
        points=witness.points,
        required_pass_actor_ids=witness.required_pass_actor_ids,
        departure_time_s=witness.departure_time_s,
        pass_times_by_actor=witness.pass_times_by_actor,
        rejoin_started_at_s=witness.rejoin_started_at_s,
        rejoin_confirmed_at_s=witness.rejoin_confirmed_at_s,
        terminal_dwell_s=witness.terminal_dwell_s,
    )
    covered_validation = validate_ground_truth_witness(
        covered_world,
        covered_witness,
        strict_declarations=witness.kind in _PASS_KINDS,
    )
    if not covered_validation.passed:
        raise ValueError(
            "covered witness no longer passes ground-truth validation: "
            + ",".join(covered_validation.failures)
        )
    source_profiles = replay_witness_profiles(world, witness, source_validation)
    covered_profiles = replay_witness_profiles(
        covered_world,
        covered_witness,
        covered_validation,
    )
    return R2BEntryCoverageReplay(
        schema_version=R2B_ENTRY_COVERAGE_SCHEMA_VERSION,
        contract=contract,
        source_world_content_hash=world.content_hash,
        covered_world_content_hash=covered_world.content_hash,
        source_witness_content_hash=witness.semantic_content_hash,
        covered_witness_content_hash=covered_witness.semantic_content_hash,
        source_validation_hash=source_validation.content_hash,
        covered_validation_hash=covered_validation.content_hash,
        source_profiles=source_profiles,
        covered_profiles=covered_profiles,
        limitations=(
            "abstract_monitored_approach_not_camera_fov_evidence",
            "controller_and_motion_authority_not_evaluated",
            "original_r2b_negative_world_preserved",
            "simulation_only_open_loop_circular_actor",
        ),
    )


def _require_entry_continuity(
    source: WitnessActorTrajectory,
    covered: WitnessActorTrajectory,
    entry_time_s: float,
) -> None:
    source_state = source.state_at(entry_time_s)
    covered_state = covered.state_at(entry_time_s)
    if source_state is None or covered_state is None:
        raise ValueError("entry coverage must contain both source and covered Actor")
    if (
        not isclose(
            source_state.position.x,
            covered_state.position.x,
            rel_tol=0.0,
            abs_tol=_POSITION_TOLERANCE_M,
        )
        or not isclose(
            source_state.position.y,
            covered_state.position.y,
            rel_tol=0.0,
            abs_tol=_POSITION_TOLERANCE_M,
        )
        or source_state.velocity != covered_state.velocity
        or source_state.radius_m != covered_state.radius_m
        or source_state.trajectory_revision != covered_state.trajectory_revision
    ):
        raise ValueError("covered Actor is discontinuous at the entry boundary")


__all__ = [
    "R2B_ENTRY_COVERAGE_SCHEMA_VERSION",
    "R2BEntryCoverageContract",
    "R2BEntryCoverageReplay",
    "R2BMonitoredEntryApproach",
    "build_r2b_entry_coverage_contract",
    "derive_r2b_covered_world",
    "replay_r2b_entry_coverage",
]
