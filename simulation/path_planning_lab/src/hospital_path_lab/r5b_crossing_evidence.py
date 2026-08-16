"""Causal public crossing evidence for the R5-B path-only controller lane."""

from __future__ import annotations

from dataclasses import dataclass, replace

from hospital_path_lab.contracts import Twist2D
from hospital_path_lab.dynamic_contracts import Point2D
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    DynamicExpectationCategory,
    generate_dynamic_corpus,
)
from hospital_path_lab.dynamic_witness_contracts import (
    AutomatedWitness,
    PassSide,
    WitnessKind,
    WitnessPhase,
    WitnessPoint,
    WitnessWorldSnapshot,
    build_automated_witness,
    project_public_witness_world,
)
from hospital_path_lab.dynamic_witness_crossing import search_crossing_bypass
from hospital_path_lab.dynamic_witness_validation import (
    GroundTruthWitnessValidation,
    canonicalize_and_validate_ground_truth_crossing_bypass,
)
from hospital_path_lab.map_factory import canonical_content_hash

R5B_CROSSING_EVIDENCE_SCHEMA_VERSION = "r5b-causal-crossing-evidence-v1"
R5B_CROSSING_RELEASE_TICK = 80
R5B_CROSSING_RELEASE_TIME_S = 4.0
_CONTROL_PERIOD_S = 0.05


@dataclass(frozen=True, slots=True)
class CausalR5BCrossingEvidence:
    schema_version: str
    source_world_hash: str
    public_id: str
    corpus_ordinal: int
    side: PassSide
    release_tick: int
    time_shift_s: float
    world: WitnessWorldSnapshot
    witness: AutomatedWitness
    validation: GroundTruthWitnessValidation
    evidence_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != R5B_CROSSING_EVIDENCE_SCHEMA_VERSION:
            raise ValueError("unsupported R5-B crossing evidence schema")
        if self.corpus_ordinal != 0 or self.release_tick != R5B_CROSSING_RELEASE_TICK:
            raise ValueError("R5-B crossing evidence identity changed")
        if self.time_shift_s != R5B_CROSSING_RELEASE_TIME_S:
            raise ValueError("R5-B crossing evidence time shift changed")
        expected_kind = (
            WitnessKind.CROSSING_BYPASS_LEFT
            if self.side is PassSide.LEFT
            else WitnessKind.CROSSING_BYPASS_RIGHT
        )
        if self.witness.kind is not expected_kind or not self.validation.passed:
            raise ValueError("R5-B crossing evidence requires a passing side witness")
        if self.witness.world_content_hash != self.world.content_hash:
            raise ValueError("R5-B crossing witness is not bound to its world")
        if self.validation.witness_content_hash != self.witness.semantic_content_hash:
            raise ValueError("R5-B crossing validation is not bound to its witness")
        expected = self.expected_content_hash
        if self.evidence_content_hash and self.evidence_content_hash != expected:
            raise ValueError("R5-B crossing evidence hash mismatch")
        object.__setattr__(self, "evidence_content_hash", expected)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "source_world_hash": self.source_world_hash,
                "public_id": self.public_id,
                "corpus_ordinal": self.corpus_ordinal,
                "side": self.side,
                "release_tick": self.release_tick,
                "time_shift_s": self.time_shift_s,
                "world_content_hash": self.world.content_hash,
                "witness_content_hash": self.witness.semantic_content_hash,
                "validation_content_hash": self.validation.content_hash,
            }
        )


def build_causal_r5b_crossing_evidence() -> tuple[CausalR5BCrossingEvidence, ...]:
    """Build left/right public evidence without weakening observation or safety limits."""

    episode = next(
        item
        for item in generate_dynamic_corpus()
        if item.split is DynamicCorpusSplit.GOLDEN
        and item.expectation_category is DynamicExpectationCategory.LOCAL_DETOUR_FEASIBLE
    )
    source_world = project_public_witness_world(episode)
    source_search = search_crossing_bypass(source_world)
    source_witnesses = {
        PassSide.LEFT: source_search.left.selected_witness,
        PassSide.RIGHT: source_search.right.selected_witness,
    }
    actor = episode.actors[0]
    backward_s = actor.active_from_s + R5B_CROSSING_RELEASE_TIME_S
    causal_duration_s = episode.duration_s + R5B_CROSSING_RELEASE_TIME_S
    causal_actor = replace(
        actor,
        active_from_s=0.0,
        active_until_s=causal_duration_s,
        start_position=Point2D(
            actor.start_position.x - actor.velocity.x * backward_s,
            actor.start_position.y - actor.velocity.y * backward_s,
        ),
    )
    causal_episode = replace(
        episode,
        episode_id=f"{episode.episode_id}-r5b-causal-crossing-v1",
        duration_s=causal_duration_s,
        actors=(causal_actor,),
    )
    world = project_public_witness_world(causal_episode)
    results: list[CausalR5BCrossingEvidence] = []
    for side in (PassSide.LEFT, PassSide.RIGHT):
        source = source_witnesses[side]
        if source is None:
            raise RuntimeError("frozen crossing source witness is missing")
        draft = _shift_witness(world, source, side)
        witness, validation = canonicalize_and_validate_ground_truth_crossing_bypass(
            world,
            draft,
        )
        if witness is None or not validation.passed:
            raise RuntimeError(
                "causal crossing witness failed independent validation:"
                + ",".join(validation.failures)
            )
        results.append(
            CausalR5BCrossingEvidence(
                schema_version=R5B_CROSSING_EVIDENCE_SCHEMA_VERSION,
                source_world_hash=source_world.content_hash,
                public_id=f"r5b-crossing-causal-{side.value}",
                corpus_ordinal=0,
                side=side,
                release_tick=R5B_CROSSING_RELEASE_TICK,
                time_shift_s=R5B_CROSSING_RELEASE_TIME_S,
                world=world,
                witness=witness,
                validation=validation,
            )
        )
    return tuple(results)


def _shift_witness(
    world: WitnessWorldSnapshot,
    source: AutomatedWitness,
    side: PassSide,
) -> AutomatedWitness:
    hold = tuple(
        WitnessPoint(
            time_s=tick * _CONTROL_PERIOD_S,
            pose=world.initial_state.pose,
            twist=Twist2D(),
            phase=(WitnessPhase.START if tick == 0 else WitnessPhase.WAIT),
            source_primitive_id="r5b-crossing-causal-hold",
        )
        for tick in range(R5B_CROSSING_RELEASE_TICK)
    )
    shifted = tuple(
        replace(point, time_s=point.time_s + R5B_CROSSING_RELEASE_TIME_S)
        for point in source.points
    )
    kind = (
        WitnessKind.CROSSING_BYPASS_LEFT
        if side is PassSide.LEFT
        else WitnessKind.CROSSING_BYPASS_RIGHT
    )
    return build_automated_witness(
        world,
        witness_id=f"r5b-crossing-causal-{side.value}-{world.content_hash[:16]}",
        kind=kind,
        terminal_mode=source.terminal_mode,
        points=(*hold, *shifted),
        required_pass_actor_ids=(world.actors[0].actor_binding_id,),
        terminal_dwell_s=source.terminal_dwell_s,
    )


__all__ = [
    "CausalR5BCrossingEvidence",
    "R5B_CROSSING_EVIDENCE_SCHEMA_VERSION",
    "R5B_CROSSING_RELEASE_TICK",
    "R5B_CROSSING_RELEASE_TIME_S",
    "build_causal_r5b_crossing_evidence",
]
