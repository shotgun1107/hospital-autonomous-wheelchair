"""Deterministic DWB-style candidate evaluation core.

This module reconstructs the observable evaluation lifecycle of Nav2 DWB at the
source revision frozen in :mod:`contracts`.  It deliberately owns no path, actor,
footprint, or safety-gate policy.  Those concerns enter through structurally typed
critics and later project adapters.

The core contract is intentionally strict:

* critics are prepared and scored in configured order;
* a critic rejects a candidate by raising :class:`IllegalTrajectoryError`;
* legal critic costs are finite and non-negative, and lower is better;
* equal totals preserve the first generated candidate;
* every rejection and every computed raw/weighted critic score is retained.

It is a simulation-only research reference, not a ROS plugin or a wheelchair
safety component.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass
from enum import StrEnum
from math import isfinite
from typing import Protocol, runtime_checkable

from .contracts import (
    DwbGeneratorRequest,
    DwbGeneratorResult,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)


@runtime_checkable
class TrajectoryGenerator(Protocol):
    """Minimum generator surface consumed by :class:`DwbReferenceCore`."""

    def generate(self, request: DwbGeneratorRequest) -> DwbGeneratorResult:
        """Return one stable-order batch for the current control snapshot."""


@runtime_checkable
class TrajectoryCritic(Protocol):
    """DWB critic lifecycle used without importing concrete critic classes."""

    def prepare(self, request: DwbGeneratorRequest) -> bool | None:
        """Prepare per-control-tick state; ``False`` rejects the whole request."""

    def score(self, trajectory: DwbTrajectory) -> float:
        """Return a finite, non-negative raw cost or reject the trajectory."""

    def debrief(self, selected_command: DwbTwist2D) -> None:
        """Observe the selected command after the complete batch is evaluated."""

    def reset(self) -> None:
        """Clear state that must not cross a reset or path boundary."""


@dataclass(frozen=True, slots=True)
class DwbCriticBinding:
    """Configured critic identity, implementation, and non-negative weight."""

    name: str
    critic: TrajectoryCritic
    scale: float = 1.0

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("critic name must not be blank")
        if not isfinite(self.scale) or self.scale < 0.0:
            raise ValueError("critic scale must be finite and non-negative")


class CandidateEvaluationStatus(StrEnum):
    """Why candidate evaluation finished."""

    LEGAL = "legal"
    ILLEGAL = "illegal"
    SHORT_CIRCUITED = "short_circuited"


class CandidateFailureKind(StrEnum):
    """Stable top-level failure taxonomy independent of critic-specific reasons."""

    CRITIC_REJECTION = "critic_rejection"
    INVALID_SCORE = "invalid_score"


@dataclass(frozen=True, slots=True)
class CriticBatchScore:
    """Optional one-candidate result produced by a critic's native batch path."""

    raw_score: float | None = None
    reason_code: str | None = None
    message: str = ""

    def __post_init__(self) -> None:
        scored = self.raw_score is not None
        rejected = self.reason_code is not None
        if scored == rejected:
            raise ValueError("batch critic outcome must be exactly scored or rejected")
        if rejected and not self.reason_code.strip():
            raise ValueError("batch critic reason_code must not be blank")


class IllegalTrajectoryError(Exception):
    """Expected critic signal that one trajectory violates a hard condition."""

    def __init__(self, reason_code: str, message: str = "") -> None:
        if not reason_code.strip():
            raise ValueError("reason_code must not be blank")
        self.reason_code = reason_code
        self.detail = message
        super().__init__(message or reason_code)


@dataclass(frozen=True, slots=True)
class CriticScoreDiagnostic:
    """One evaluated critic's contribution to a candidate total."""

    critic_name: str
    raw_score: float
    scale: float
    weighted_score: float


@dataclass(frozen=True, slots=True)
class CandidateFailureDiagnostic:
    """Structured explanation for a candidate rejected during critic scoring."""

    kind: CandidateFailureKind
    critic_name: str
    reason_code: str
    message: str


@dataclass(frozen=True, slots=True)
class CandidateEvaluationDiagnostic:
    """Complete observable result for one generator-order candidate."""

    candidate_index: int
    command: DwbTwist2D
    status: CandidateEvaluationStatus
    accumulated_score: float
    critic_scores: tuple[CriticScoreDiagnostic, ...]
    failure: CandidateFailureDiagnostic | None = None

    @property
    def fully_scored(self) -> bool:
        return self.status is CandidateEvaluationStatus.LEGAL


@dataclass(frozen=True, slots=True)
class DwbCoreResult:
    """Selected candidate plus batch-wide reproducibility diagnostics."""

    command: DwbTwist2D
    trajectory: DwbTrajectory
    total_score: float
    selected_candidate_index: int
    generator_result: DwbGeneratorResult
    candidate_evaluations: tuple[CandidateEvaluationDiagnostic, ...]


class DwbPreparationError(RuntimeError):
    """A critic could not prepare the current control snapshot."""

    def __init__(self, critic_name: str) -> None:
        self.critic_name = critic_name
        super().__init__(f"critic preparation failed: {critic_name}")


class NoLegalTrajectoryError(RuntimeError):
    """No candidate survived all configured critics."""

    def __init__(
        self,
        evaluations: tuple[CandidateEvaluationDiagnostic, ...],
    ) -> None:
        self.evaluations = evaluations
        self.failure_counts = dict(
            Counter(
                evaluation.failure.reason_code
                for evaluation in evaluations
                if evaluation.failure is not None
            )
        )
        super().__init__(
            "no legal trajectory"
            if not self.failure_counts
            else f"no legal trajectory: {self.failure_counts}"
        )


class DwbReferenceCore:
    """Prepare, generate, score, select, and debrief one DWB candidate batch."""

    def __init__(
        self,
        generator: TrajectoryGenerator,
        critics: Sequence[DwbCriticBinding],
        *,
        short_circuit_trajectory_evaluation: bool = True,
    ) -> None:
        self._generator = generator
        self._critics = tuple(critics)
        self._short_circuit = short_circuit_trajectory_evaluation
        self._path: tuple[DwbPose2D, ...] | None = None
        names = [binding.name for binding in self._critics]
        if len(names) != len(set(names)):
            raise ValueError("critic names must be unique")

    @property
    def path(self) -> tuple[DwbPose2D, ...] | None:
        """Currently installed path, if the adapter has supplied one."""

        return self._path

    @property
    def critic_names(self) -> tuple[str, ...]:
        """Configured critic order used for every candidate."""

        return tuple(binding.name for binding in self._critics)

    def set_path(self, path: Sequence[DwbPose2D]) -> None:
        """Install a new path and clear all state tied to the previous path.

        Concrete critics may optionally expose ``set_path(path)``.  The optional
        hook is deliberately discovered structurally so this core does not import
        or depend on the concurrently developed critic module.
        """

        frozen_path = tuple(path)
        if not frozen_path:
            raise ValueError("path must not be empty")
        self.reset()
        self._path = frozen_path
        for binding in self._critics:
            set_path = getattr(binding.critic, "set_path", None)
            if set_path is not None:
                set_path(frozen_path)

    def reset(self) -> None:
        """Clear stateful critic state without changing critic configuration."""

        for binding in self._critics:
            binding.critic.reset()

    def compute(self, request: DwbGeneratorRequest) -> DwbCoreResult:
        """Evaluate one generator batch and return the strict lowest-cost candidate."""

        self._prepare_critics(request)
        generator_result = self._generator.generate(request)
        batch_scores = self._batch_critic_scores(generator_result.trajectories)
        evaluations: list[CandidateEvaluationDiagnostic] = []
        best_trajectory: DwbTrajectory | None = None
        best_score: float | None = None
        best_index: int | None = None

        for candidate_index, trajectory in enumerate(generator_result.trajectories):
            evaluation = self._evaluate_candidate(
                candidate_index,
                trajectory,
                best_score,
                batch_scores,
            )
            evaluations.append(evaluation)
            if evaluation.status is not CandidateEvaluationStatus.LEGAL:
                continue
            # Strictly lower is intentional: generator order resolves exact ties.
            if best_score is None or evaluation.accumulated_score < best_score:
                best_trajectory = trajectory
                best_score = evaluation.accumulated_score
                best_index = candidate_index

        frozen_evaluations = tuple(evaluations)
        if best_trajectory is None or best_score is None or best_index is None:
            raise NoLegalTrajectoryError(frozen_evaluations)

        for binding in self._critics:
            binding.critic.debrief(best_trajectory.command)

        return DwbCoreResult(
            command=best_trajectory.command,
            trajectory=best_trajectory,
            total_score=best_score,
            selected_candidate_index=best_index,
            generator_result=generator_result,
            candidate_evaluations=frozen_evaluations,
        )

    def _prepare_critics(self, request: DwbGeneratorRequest) -> None:
        for binding in self._critics:
            prepared = binding.critic.prepare(request)
            if prepared is False:
                raise DwbPreparationError(binding.name)

    def _batch_critic_scores(
        self,
        trajectories: tuple[DwbTrajectory, ...],
    ) -> dict[str, tuple[CriticBatchScore, ...]]:
        outcomes: dict[str, tuple[CriticBatchScore, ...]] = {}
        for binding in self._critics:
            if binding.scale == 0.0:
                continue
            score_batch = getattr(binding.critic, "score_batch", None)
            if score_batch is None:
                continue
            candidate_scores = score_batch(trajectories)
            if candidate_scores is None:
                continue
            frozen = tuple(candidate_scores)
            if len(frozen) != len(trajectories) or any(
                not isinstance(item, CriticBatchScore) for item in frozen
            ):
                raise RuntimeError(
                    f"critic batch result shape is invalid: {binding.name}"
                )
            outcomes[binding.name] = frozen
        return outcomes

    def _evaluate_candidate(
        self,
        candidate_index: int,
        trajectory: DwbTrajectory,
        best_score: float | None,
        batch_scores: dict[str, tuple[CriticBatchScore, ...]],
    ) -> CandidateEvaluationDiagnostic:
        accumulated_score = 0.0
        scores: list[CriticScoreDiagnostic] = []

        for binding in self._critics:
            if binding.scale == 0.0:
                continue
            batch = batch_scores.get(binding.name)
            batch_score = None if batch is None else batch[candidate_index]
            if batch_score is not None and batch_score.reason_code is not None:
                return CandidateEvaluationDiagnostic(
                    candidate_index=candidate_index,
                    command=trajectory.command,
                    status=CandidateEvaluationStatus.ILLEGAL,
                    accumulated_score=accumulated_score,
                    critic_scores=tuple(scores),
                    failure=CandidateFailureDiagnostic(
                        kind=CandidateFailureKind.CRITIC_REJECTION,
                        critic_name=binding.name,
                        reason_code=batch_score.reason_code,
                        message=batch_score.message,
                    ),
                )
            if batch_score is not None:
                raw_score = batch_score.raw_score
                assert raw_score is not None
            else:
                try:
                    raw_score = binding.critic.score(trajectory)
                except IllegalTrajectoryError as error:
                    return CandidateEvaluationDiagnostic(
                        candidate_index=candidate_index,
                        command=trajectory.command,
                        status=CandidateEvaluationStatus.ILLEGAL,
                        accumulated_score=accumulated_score,
                        critic_scores=tuple(scores),
                        failure=CandidateFailureDiagnostic(
                            kind=CandidateFailureKind.CRITIC_REJECTION,
                            critic_name=binding.name,
                            reason_code=error.reason_code,
                            message=error.detail,
                        ),
                    )

            weighted_score = raw_score * binding.scale
            if (
                not isfinite(raw_score)
                or raw_score < 0.0
                or not isfinite(weighted_score)
                or weighted_score < 0.0
            ):
                return CandidateEvaluationDiagnostic(
                    candidate_index=candidate_index,
                    command=trajectory.command,
                    status=CandidateEvaluationStatus.ILLEGAL,
                    accumulated_score=accumulated_score,
                    critic_scores=tuple(scores),
                    failure=CandidateFailureDiagnostic(
                        kind=CandidateFailureKind.INVALID_SCORE,
                        critic_name=binding.name,
                        reason_code="invalid_critic_score",
                        message=f"raw={raw_score!r}, weighted={weighted_score!r}",
                    ),
                )

            accumulated_score += weighted_score
            scores.append(
                CriticScoreDiagnostic(
                    critic_name=binding.name,
                    raw_score=raw_score,
                    scale=binding.scale,
                    weighted_score=weighted_score,
                )
            )
            if (
                self._short_circuit
                and best_score is not None
                and accumulated_score > best_score
            ):
                return CandidateEvaluationDiagnostic(
                    candidate_index=candidate_index,
                    command=trajectory.command,
                    status=CandidateEvaluationStatus.SHORT_CIRCUITED,
                    accumulated_score=accumulated_score,
                    critic_scores=tuple(scores),
                )

        return CandidateEvaluationDiagnostic(
            candidate_index=candidate_index,
            command=trajectory.command,
            status=CandidateEvaluationStatus.LEGAL,
            accumulated_score=accumulated_score,
            critic_scores=tuple(scores),
        )
