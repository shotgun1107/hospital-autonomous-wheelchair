"""Public-only v7 context lane for directional Actor prediction research.

The factory in this module converts the frozen 10 Hz synthetic observation
stream into the 20 Hz :class:`~hospital_path_lab.dynamic_safety.DynamicSafetyContext`
used by the shared online gate.  It intentionally accepts public corpus splits
only and never puts evaluator-only metadata into the controller-facing context.

This remains a Python simulation harness for constant-heading, open-loop circular
Actors.  It is not product code and is not evidence for real-person safety.
"""

from __future__ import annotations

from math import isclose, isfinite

from hospital_path_lab.contracts import GridSnapshot, RobotState
from hospital_path_lab.dynamic_contracts import DYNAMIC_CONTROL_PERIOD_S, DynamicMotionState
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusEpisode,
    DynamicCorpusSplit,
    build_dynamic_grid_snapshot,
    controller_episode_id,
    generate_episode_observation_slots,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalActorPredictor,
    DirectionalPredictionResult,
    DirectionalPredictionStatus,
)
from hospital_path_lab.dynamic_observation import (
    DynamicObservationAvailability,
    DynamicObservationProfile,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
)
from hospital_path_lab.dynamic_safety import (
    DynamicSafetyContext,
    DynamicSafetyGate,
    build_resume_authorization,
)

_TIME_TOLERANCE_S = 1e-12
_PUBLIC_SPLITS = frozenset(
    (DynamicCorpusSplit.GOLDEN, DynamicCorpusSplit.DEVELOPMENT)
)
_SAFE_DIRECTIONAL_STATUSES = frozenset(
    (DirectionalPredictionStatus.READY, DirectionalPredictionStatus.EMPTY_FRAME)
)


class DirectionalPublicEpisodeContextFactory:
    """Create one deterministic, public-only v7 safety context per 20 Hz tick.

    The synthetic authority emitter issues one authorization for each newly
    confirmed protective-stop epoch.  That authorization is still only one term
    of the shared gate's AND condition: the current path, local safety recheck,
    fresh directional observation, eleven unique safe frames and trajectory
    safety must also pass before motion can resume.
    """

    def __init__(
        self,
        episode: DynamicCorpusEpisode,
        profile: DynamicObservationProfile,
        *,
        authorization_revision: int = 1,
        path_still_valid: bool = True,
        local_safety_recheck_passed: bool = True,
        emit_resume_authorization: bool = True,
    ) -> None:
        if not isinstance(episode, DynamicCorpusEpisode):
            raise TypeError("directional public context requires a corpus episode")
        if episode.split not in _PUBLIC_SPLITS:
            raise ValueError("directional v7 context rejects non-public corpus splits")
        if not isinstance(profile, DynamicObservationProfile):
            raise TypeError("directional public context requires an observation profile")
        if not isinstance(authorization_revision, int) or isinstance(
            authorization_revision, bool
        ):
            raise TypeError("authorization_revision must be an integer")
        if authorization_revision < 0:
            raise ValueError("authorization_revision must not be negative")
        for value in (
            path_still_valid,
            local_safety_recheck_passed,
            emit_resume_authorization,
        ):
            if not isinstance(value, bool):
                raise TypeError("directional context policy flags must be bool values")

        source = DynamicObservationSourceIdentity(
            stream_id="dynamic-stage5-stream",
            episode_id=controller_episode_id(episode),
            episode_seed=episode.seed,
            map_id=episode.map_id,
            map_revision=1,
        )
        self._episode = episode
        self._profile = profile
        self._source = source
        self._slots = generate_episode_observation_slots(episode, profile=profile)
        self._validator = DynamicObservationValidator(source, profile)
        self._predictor = DirectionalActorPredictor()
        self._authorization_revision = authorization_revision
        self._path_still_valid = path_still_valid
        self._local_safety_recheck_passed = local_safety_recheck_passed
        self._emit_resume_authorization = emit_resume_authorization
        self._next_slot = 0
        self._last_tick_id: int | None = None
        self._last_simulation_time_s: float | None = None
        self._last_prediction_result: DirectionalPredictionResult | None = None
        self._authorization_epoch: int | None = None
        self._authorization = None
        self._grid_by_tick: dict[int, GridSnapshot] = {}

    @property
    def source(self) -> DynamicObservationSourceIdentity:
        """Return the label-free identity expected by the observation validator."""

        return self._source

    @property
    def last_prediction_result(self) -> DirectionalPredictionResult | None:
        """Expose predictor status for public diagnostics, never evaluator labels."""

        return self._last_prediction_result

    def grid_at(self, tick_id: int) -> GridSnapshot:
        """Return the exact grid snapshot bound to a previously built tick."""

        return self._grid_by_tick[tick_id]

    def __call__(
        self,
        tick_id: int,
        simulation_time_s: float,
        _robot_state: RobotState,
        gate: DynamicSafetyGate,
    ) -> DynamicSafetyContext:
        self._validate_tick(tick_id, simulation_time_s)
        self._deliver_available_slots(simulation_time_s)

        observation = self._validator.snapshot(control_time_s=simulation_time_s)
        prediction_result = self._predictor.update(observation)
        self._last_prediction_result = prediction_result
        usable_directional_result = all(
            (
                observation.availability is DynamicObservationAvailability.FRESH,
                observation.frame is not None,
                prediction_result.status in _SAFE_DIRECTIONAL_STATUSES,
                prediction_result.prediction_set is not None,
            )
        )
        prediction_set = (
            prediction_result.prediction_set if usable_directional_result else None
        )

        observation_revision = (
            observation.frame.observation_revision
            if observation.frame is not None
            else 0
        )
        grid = build_dynamic_grid_snapshot(
            self._episode,
            observation_revision=observation_revision,
        )
        self._grid_by_tick[tick_id] = grid

        authorization = self._resume_authorization(gate, simulation_time_s)
        context = DynamicSafetyContext(
            tick_id=tick_id,
            simulation_time_s=simulation_time_s,
            mission_id=self._episode.mission_id,
            authorization_revision=self._authorization_revision,
            grid_snapshot=grid,
            observation_snapshot=observation,
            prediction_set=prediction_set,
            path_still_valid=self._path_still_valid,
            local_safety_recheck_passed=self._local_safety_recheck_passed,
            observation_safe=usable_directional_result,
            resume_authorization=authorization,
        )
        self._last_tick_id = tick_id
        self._last_simulation_time_s = simulation_time_s
        return context

    def _validate_tick(self, tick_id: int, simulation_time_s: float) -> None:
        if not isinstance(tick_id, int) or isinstance(tick_id, bool) or tick_id < 0:
            raise ValueError("tick_id must be a non-negative integer")
        if not isfinite(simulation_time_s) or simulation_time_s < 0.0:
            raise ValueError("simulation_time_s must be finite and non-negative")
        expected_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        if not isclose(
            simulation_time_s,
            expected_time_s,
            rel_tol=0.0,
            abs_tol=_TIME_TOLERANCE_S,
        ):
            raise ValueError("directional context tick must align with the 20 Hz clock")
        if self._last_tick_id is not None and tick_id <= self._last_tick_id:
            raise ValueError("directional context tick_id must increase")
        if (
            self._last_simulation_time_s is not None
            and simulation_time_s <= self._last_simulation_time_s
        ):
            raise ValueError("directional context simulation time must increase")

    def _deliver_available_slots(self, simulation_time_s: float) -> None:
        while self._next_slot < len(self._slots):
            slot = self._slots[self._next_slot]
            if slot.scheduled_delivery_at_s > simulation_time_s + _TIME_TOLERANCE_S:
                break
            if slot.frame is None:
                self._validator.record_no_frame(
                    sequence=slot.sequence,
                    delivery_time_s=slot.scheduled_delivery_at_s,
                )
            else:
                accepted = self._validator.accept(
                    slot.frame,
                    received_at_s=slot.scheduled_delivery_at_s,
                )
                if not accepted.accepted:
                    raise ValueError(
                        "generated public observation failed validation: "
                        f"{accepted.failures}"
                    )
            self._next_slot += 1

    def _resume_authorization(
        self,
        gate: DynamicSafetyGate,
        simulation_time_s: float,
    ):
        if (
            not self._emit_resume_authorization
            or gate.motion_state is not DynamicMotionState.HOLDING
            or gate.stop_confirmed_at_s is None
        ):
            return None
        if self._authorization_epoch != gate.stop_epoch:
            self._authorization_epoch = gate.stop_epoch
            self._authorization = build_resume_authorization(
                mission_id=self._episode.mission_id,
                stop_epoch=gate.stop_epoch,
                issued_or_revalidated_at_s=simulation_time_s,
                authorization_revision=self._authorization_revision,
            )
        return self._authorization


__all__ = ["DirectionalPublicEpisodeContextFactory"]
