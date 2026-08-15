from __future__ import annotations

from dataclasses import dataclass, field

import pytest

from hospital_path_lab.local_algorithms.dwb_reference.contracts import (
    DwbGeneratorRequest,
    DwbGeneratorResult,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)
from hospital_path_lab.local_algorithms.dwb_reference.core import (
    CandidateEvaluationStatus,
    CandidateFailureKind,
    CriticBatchScore,
    DwbCriticBinding,
    DwbPreparationError,
    DwbReferenceCore,
    IllegalTrajectoryError,
    NoLegalTrajectoryError,
)


def _trajectory(linear: float, angular: float = 0.0) -> DwbTrajectory:
    pose = DwbPose2D(linear, angular, 0.0)
    return DwbTrajectory(
        command=DwbTwist2D(linear, angular),
        poses=(DwbPose2D(0.0, 0.0, 0.0), pose),
        integration_step_s=0.05,
    )


def _request() -> DwbGeneratorRequest:
    return DwbGeneratorRequest(
        pose=DwbPose2D(0.0, 0.0, 0.0),
        current_twist=DwbTwist2D(0.0, 0.0),
    )


@dataclass
class FakeGenerator:
    trajectories: tuple[DwbTrajectory, ...]
    events: list[str] = field(default_factory=list)

    def generate(self, request: DwbGeneratorRequest) -> DwbGeneratorResult:
        self.events.append("generate")
        return DwbGeneratorResult(
            linear_window_mps=(0.0, 0.2),
            angular_window_radps=(-0.8, 0.8),
            linear_samples_mps=tuple(
                trajectory.command.linear_mps for trajectory in self.trajectories
            ),
            angular_samples_radps=tuple(
                trajectory.command.angular_radps for trajectory in self.trajectories
            ),
            trajectories=self.trajectories,
        )


@dataclass
class RecordingCritic:
    name: str
    events: list[str]
    values: dict[float, float] = field(default_factory=dict)
    illegal_commands: set[float] = field(default_factory=set)
    prepare_result: bool | None = True
    paths: list[tuple[DwbPose2D, ...]] = field(default_factory=list)

    def prepare(self, request: DwbGeneratorRequest) -> bool | None:
        self.events.append(f"prepare:{self.name}")
        return self.prepare_result

    def score(self, trajectory: DwbTrajectory) -> float:
        linear = trajectory.command.linear_mps
        self.events.append(f"score:{self.name}:{linear}")
        if linear in self.illegal_commands:
            raise IllegalTrajectoryError("blocked", f"{linear} rejected")
        return self.values.get(linear, 0.0)

    def debrief(self, selected_command: DwbTwist2D) -> None:
        self.events.append(f"debrief:{self.name}:{selected_command.linear_mps}")

    def reset(self) -> None:
        self.events.append(f"reset:{self.name}")

    def set_path(self, path: tuple[DwbPose2D, ...]) -> None:
        self.events.append(f"set_path:{self.name}")
        self.paths.append(path)


@dataclass
class BatchRecordingCritic(RecordingCritic):
    batch: tuple[CriticBatchScore, ...] | None = None

    def score_batch(
        self,
        trajectories: tuple[DwbTrajectory, ...],
    ) -> tuple[CriticBatchScore, ...] | None:
        self.events.append(f"score_batch:{self.name}:{len(trajectories)}")
        return self.batch


def _core(
    trajectories: tuple[DwbTrajectory, ...],
    critics: tuple[tuple[RecordingCritic, float], ...],
    *,
    short_circuit: bool = False,
) -> tuple[DwbReferenceCore, FakeGenerator, list[str]]:
    events = critics[0][0].events if critics else []
    generator = FakeGenerator(trajectories, events)
    core = DwbReferenceCore(
        generator,
        tuple(DwbCriticBinding(critic.name, critic, scale) for critic, scale in critics),
        short_circuit_trajectory_evaluation=short_circuit,
    )
    return core, generator, events


def test_lifecycle_uses_configured_critic_and_candidate_order() -> None:
    events: list[str] = []
    first = RecordingCritic("first", events, {0.1: 1.0, 0.2: 2.0})
    second = RecordingCritic("second", events, {0.1: 3.0, 0.2: 4.0})
    core, _, _ = _core((_trajectory(0.1), _trajectory(0.2)), ((first, 1.0), (second, 1.0)))

    result = core.compute(_request())

    assert result.command == DwbTwist2D(0.1, 0.0)
    assert events == [
        "prepare:first",
        "prepare:second",
        "generate",
        "score:first:0.1",
        "score:second:0.1",
        "score:first:0.2",
        "score:second:0.2",
        "debrief:first:0.1",
        "debrief:second:0.1",
    ]


def test_illegal_candidate_is_removed_immediately_and_diagnosed() -> None:
    events: list[str] = []
    obstacle = RecordingCritic("obstacle", events, illegal_commands={0.1})
    later = RecordingCritic("later", events, {0.2: 2.0})
    core, _, _ = _core(
        (_trajectory(0.1), _trajectory(0.2)),
        ((obstacle, 1.0), (later, 1.0)),
    )

    result = core.compute(_request())

    rejected = result.candidate_evaluations[0]
    assert result.selected_candidate_index == 1
    assert rejected.status is CandidateEvaluationStatus.ILLEGAL
    assert rejected.failure is not None
    assert rejected.failure.kind is CandidateFailureKind.CRITIC_REJECTION
    assert rejected.failure.critic_name == "obstacle"
    assert rejected.failure.reason_code == "blocked"
    assert "score:later:0.1" not in events


def test_optional_batch_critic_preserves_order_scores_and_rejections() -> None:
    events: list[str] = []
    critic = BatchRecordingCritic(
        "native",
        events,
        batch=(
            CriticBatchScore(reason_code="native_blocked", message="blocked"),
            CriticBatchScore(raw_score=2.0),
        ),
    )
    core, _, _ = _core(
        (_trajectory(0.1), _trajectory(0.2)),
        ((critic, 3.0),),
    )

    result = core.compute(_request())

    assert result.selected_candidate_index == 1
    assert result.total_score == 6.0
    assert result.candidate_evaluations[0].failure is not None
    assert result.candidate_evaluations[0].failure.reason_code == "native_blocked"
    assert not any(item.startswith("score:native") for item in events)
    assert events == [
        "prepare:native",
        "generate",
        "score_batch:native:2",
        "debrief:native:0.2",
    ]


def test_weighted_sum_and_strict_less_keep_first_exact_tie() -> None:
    events: list[str] = []
    first = RecordingCritic("distance", events, {0.1: 2.0, 0.2: 1.0})
    second = RecordingCritic("heading", events, {0.1: 0.0, 0.2: 1.0})
    core, _, _ = _core(
        (_trajectory(0.1), _trajectory(0.2)),
        ((first, 2.0), (second, 2.0)),
    )

    result = core.compute(_request())

    assert result.selected_candidate_index == 0
    assert result.total_score == pytest.approx(4.0)
    assert [score.weighted_score for score in result.candidate_evaluations[1].critic_scores] == [
        2.0,
        2.0,
    ]


def test_short_circuit_retains_partial_score_diagnostics() -> None:
    events: list[str] = []
    first = RecordingCritic("first", events, {0.1: 1.0, 0.2: 2.0})
    second = RecordingCritic("second", events, {0.1: 0.0, 0.2: 50.0})
    core, _, _ = _core(
        (_trajectory(0.1), _trajectory(0.2)),
        ((first, 1.0), (second, 1.0)),
        short_circuit=True,
    )

    result = core.compute(_request())

    pruned = result.candidate_evaluations[1]
    assert result.selected_candidate_index == 0
    assert pruned.status is CandidateEvaluationStatus.SHORT_CIRCUITED
    assert pruned.accumulated_score == pytest.approx(2.0)
    assert [score.critic_name for score in pruned.critic_scores] == ["first"]
    assert "score:second:0.2" not in events


def test_all_illegal_candidates_raise_with_failure_taxonomy() -> None:
    events: list[str] = []
    critic = RecordingCritic("obstacle", events, illegal_commands={0.1, 0.2})
    core, _, _ = _core(
        (_trajectory(0.1), _trajectory(0.2)),
        ((critic, 1.0),),
    )

    with pytest.raises(NoLegalTrajectoryError) as caught:
        core.compute(_request())

    assert caught.value.failure_counts == {"blocked": 2}
    assert [evaluation.candidate_index for evaluation in caught.value.evaluations] == [0, 1]
    assert not any(event.startswith("debrief") for event in events)


@pytest.mark.parametrize("bad_score", [float("nan"), float("inf"), -1.0])
def test_invalid_critic_score_rejects_candidate_with_diagnostic(bad_score: float) -> None:
    events: list[str] = []
    critic = RecordingCritic("broken", events, {0.1: bad_score})
    core, _, _ = _core((_trajectory(0.1),), ((critic, 1.0),))

    with pytest.raises(NoLegalTrajectoryError) as caught:
        core.compute(_request())

    failure = caught.value.evaluations[0].failure
    assert failure is not None
    assert failure.kind is CandidateFailureKind.INVALID_SCORE
    assert failure.reason_code == "invalid_critic_score"


def test_prepare_failure_stops_before_generation() -> None:
    events: list[str] = []
    critic = RecordingCritic("path", events, prepare_result=False)
    core, _, _ = _core((_trajectory(0.1),), ((critic, 1.0),))

    with pytest.raises(DwbPreparationError, match="path"):
        core.compute(_request())

    assert events == ["prepare:path"]


def test_zero_scale_critic_is_not_scored_but_is_prepared_and_debriefed() -> None:
    events: list[str] = []
    disabled = RecordingCritic("disabled", events, illegal_commands={0.1})
    core, _, _ = _core((_trajectory(0.1),), ((disabled, 0.0),))

    result = core.compute(_request())

    assert result.total_score == 0.0
    assert events == ["prepare:disabled", "generate", "debrief:disabled:0.1"]


def test_new_path_resets_state_once_and_forwards_frozen_path() -> None:
    events: list[str] = []
    critic = RecordingCritic("path", events)
    core, _, _ = _core((_trajectory(0.1),), ((critic, 1.0),))
    path = [DwbPose2D(0.0, 0.0, 0.0), DwbPose2D(1.0, 0.0, 0.0)]

    core.set_path(path)
    path.append(DwbPose2D(2.0, 0.0, 0.0))

    assert core.path == tuple(path[:2])
    assert critic.paths == [tuple(path[:2])]
    assert events == ["reset:path", "set_path:path"]


def test_repeated_compute_is_deterministic_for_stateless_critics() -> None:
    events: list[str] = []
    critic = RecordingCritic("cost", events, {0.1: 1.0, 0.2: 2.0})
    core, _, _ = _core(
        (_trajectory(0.1), _trajectory(0.2)),
        ((critic, 1.0),),
        short_circuit=False,
    )

    first = core.compute(_request())
    second = core.compute(_request())

    assert first.command == second.command
    assert first.total_score == second.total_score
    assert first.candidate_evaluations == second.candidate_evaluations


def test_duplicate_names_and_invalid_scales_are_rejected() -> None:
    events: list[str] = []
    critic = RecordingCritic("same", events)
    generator = FakeGenerator((_trajectory(0.1),), events)

    with pytest.raises(ValueError, match="unique"):
        DwbReferenceCore(
            generator,
            (DwbCriticBinding("same", critic), DwbCriticBinding("same", critic)),
        )
    with pytest.raises(ValueError, match="non-negative"):
        DwbCriticBinding("bad", critic, -1.0)
