"""Project-contract adapter for the source-derived DWB reference core.

The core intentionally knows nothing about the hospital experiment's provenance
or dynamic-observation contracts.  This adapter is the only Phase B-2 boundary:
it accepts one :class:`ControllerSnapshot`, validates it fail-closed, advances the
chassis through the frozen 50 ms command-application delay, and converts the DWB
result back to a provenance-preserving :class:`ControllerCommandResult`.

This stage covers path tracking only.  Goal approach / stop / final rotation is a
separate integration step and must not be inferred from a successful result here.
Dynamic Actor, footprint, forbidden-region, and terminal-stopping constraints are
also injected as critic bindings (or through an already composed core); they are
not silently reimplemented in this adapter.

The implementation is a simulation-only behavioral reconstruction informed by
the frozen ROS 1 Navigation and Nav2 revisions recorded in ``contracts.py``.  It
is not a ROS plugin and is not evidence of real-wheelchair or human safety.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Sequence
from dataclasses import fields, is_dataclass, replace
from enum import Enum
from hashlib import sha256
from json import dumps
from math import atan2, cos, isclose, isfinite, sin
from time import perf_counter_ns
from typing import Protocol

import numpy as np

from hospital_path_lab.contracts import PlanStatus, Pose2D, TrajectoryPoint, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_COMMAND_APPLY_LATENCY_S,
    ControllerCommandResult,
    ControllerSnapshot,
    controller_snapshot_content_hash,
)
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalPredictionSet,
    validate_directional_prediction_set,
)
from hospital_path_lab.dynamic_observation import DynamicObservationAvailability
from hospital_path_lab.dynamic_prediction import (
    ActorPredictionSet,
    build_actor_prediction_set,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile

from .contracts import (
    DwbGeneratorConfig,
    DwbGeneratorRequest,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)
from .core import (
    CandidateEvaluationStatus,
    DwbCoreResult,
    DwbCriticBinding,
    DwbPreparationError,
    DwbReferenceCore,
    NoLegalTrajectoryError,
)
from .trajectory_generator import DwbReferenceTrajectoryGenerator

_TIME_TOLERANCE_S = 1e-12
_FLOAT_TOLERANCE = 1e-12
_SEMANTIC_DIGEST_PREFIX = "semantic_digest="


class DwbCore(Protocol):
    """Narrow injection boundary used by the project adapter."""

    def set_path(self, path: Sequence[DwbPose2D]) -> None:
        """Install a path and reset path-bound critic state."""

    def compute(self, request: DwbGeneratorRequest) -> DwbCoreResult:
        """Return the selected candidate or raise a documented core error."""


class SnapshotBinder(Protocol):
    """Per-tick project extension that must be rebound after path resets."""

    def bind_snapshot(self, snapshot: ControllerSnapshot) -> None:
        """Bind the immutable project input used by the next core evaluation."""


class SourceDerivedDwbController:
    """Translate one validated project snapshot to and from a DWB reference core.

    ``core`` allows tests or a later constraint composition layer to inject a
    complete implementation.  Otherwise a reference generator and the supplied
    ``critics`` are composed locally.  Passing no critics is permitted for narrow
    adapter/generator tests, but is not a public-corpus safety configuration.
    """

    name = "dynamic_dwb_reference"

    def __init__(
        self,
        vehicle_profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
        *,
        core: DwbCore | None = None,
        critics: Sequence[DwbCriticBinding] = (),
        snapshot_binders: Sequence[SnapshotBinder] = (),
        generator_config: DwbGeneratorConfig | None = None,
        allow_reverse_generator: bool = False,
    ) -> None:
        if not vehicle_profile.simulation_only:
            raise ValueError("source-derived DWB requires a simulation-only profile")
        if core is not None and critics:
            raise ValueError("inject either a composed core or critic bindings, not both")

        config = generator_config or _generator_config_for(vehicle_profile)
        _validate_generator_profile(
            config,
            vehicle_profile,
            allow_reverse=allow_reverse_generator,
        )
        self.vehicle_profile = vehicle_profile
        self.generator_config = config
        self._core: DwbCore = core or DwbReferenceCore(
            DwbReferenceTrajectoryGenerator(config),
            tuple(critics),
        )
        self._snapshot_binders = tuple(snapshot_binders)
        self._installed_path_signature: str | None = None
        self._prepared_snapshot_identity: tuple[int, str] | None = None
        self._command_linear_bounds = (0.0, vehicle_profile.max_forward_speed_mps)

    def set_command_linear_bounds(self, minimum_mps: float, maximum_mps: float) -> None:
        """Set section-bound output limits without changing generator ordering."""

        configured_minimum = (
            -self.generator_config.maximum_reverse_speed_mps
            if self.generator_config.allow_reverse
            else 0.0
        )
        configured_maximum = self.generator_config.maximum_forward_speed_mps
        if not (
            isfinite(minimum_mps)
            and isfinite(maximum_mps)
            and configured_minimum <= minimum_mps <= maximum_mps <= configured_maximum
        ):
            raise ValueError("command linear bounds exceed the configured generator range")
        self._command_linear_bounds = (minimum_mps, maximum_mps)

    @property
    def installed_path_signature(self) -> str | None:
        """Signature of the most recent path successfully installed in the core."""

        return self._installed_path_signature

    def step(self, snapshot: ControllerSnapshot) -> ControllerCommandResult:
        """Compute one command while preserving the snapshot's exact provenance."""

        if not isinstance(snapshot, ControllerSnapshot):
            raise TypeError("source-derived DWB input must be a ControllerSnapshot")
        started_at = perf_counter_ns()
        preparation_failure = self._prepare_snapshot(snapshot, started_at)
        if preparation_failure is not None:
            return preparation_failure

        return self._compute_prepared(snapshot, started_at)

    def prepare_snapshot(
        self,
        snapshot: ControllerSnapshot,
    ) -> ControllerCommandResult | None:
        """Validate, install the path, and bind per-tick project extensions.

        Goal-handling composition uses this boundary when it must validate a
        stop/rotate override without running or debriefing the normal DWB batch.
        ``None`` means preparation succeeded; a result is a fail-closed response.
        """

        if not isinstance(snapshot, ControllerSnapshot):
            raise TypeError("source-derived DWB input must be a ControllerSnapshot")
        return self._prepare_snapshot(snapshot, perf_counter_ns())

    def compute_prepared(self, snapshot: ControllerSnapshot) -> ControllerCommandResult:
        """Run the core once for the exact snapshot prepared by the caller."""

        if not isinstance(snapshot, ControllerSnapshot):
            raise TypeError("source-derived DWB input must be a ControllerSnapshot")
        prepared_identity = self._prepared_snapshot_identity
        if prepared_identity is None:
            raise ValueError("snapshot must be prepared before core computation")
        started_at = perf_counter_ns()
        if prepared_identity != _snapshot_identity(snapshot):
            # A binder has already consumed the prepared snapshot.  Never run
            # the core with different semantic input merely because the narrow
            # provenance hash (tick/revisions/content hashes) stayed unchanged.
            # Invalidate the token as well so the previously bound state cannot
            # be reused after a mismatched compute attempt.
            self._prepared_snapshot_identity = None
            return _result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.INVALID_INPUT,
                failure_reason="prepared_snapshot_semantic_mismatch",
                decision_trace=(
                    "adapter_input=invalid",
                    "reason=prepared_snapshot_semantic_mismatch",
                ),
                controller_requested_stop=True,
            )
        invalid_reason = self._invalid_reason(snapshot)
        if invalid_reason is not None:
            self._prepared_snapshot_identity = None
            return _result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.INVALID_INPUT,
                failure_reason=invalid_reason,
                decision_trace=(
                    "adapter_input=invalid_after_prepare",
                    f"reason={invalid_reason}",
                ),
                controller_requested_stop=True,
            )
        return self._compute_prepared(snapshot, started_at)

    def _prepare_snapshot(
        self,
        snapshot: ControllerSnapshot,
        started_at: int,
    ) -> ControllerCommandResult | None:
        self._prepared_snapshot_identity = None
        invalid_reason = self._invalid_reason(snapshot)
        if invalid_reason is not None:
            return _result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.INVALID_INPUT,
                failure_reason=invalid_reason,
                decision_trace=("adapter_input=invalid", f"reason={invalid_reason}"),
                controller_requested_stop=True,
            )

        path = tuple(_to_dwb_pose(pose) for pose in snapshot.reference_path)
        signature = _controller_path_signature(snapshot, path)
        if signature != self._installed_path_signature:
            try:
                self._core.set_path(path)
            except ValueError:
                return _result(
                    self.name,
                    snapshot,
                    started_at,
                    status=PlanStatus.INVALID_INPUT,
                    failure_reason="dwb_path_rejected",
                    decision_trace=(
                        "adapter_input=invalid",
                        "reason=dwb_path_rejected",
                        f"path_signature={signature}",
                    ),
                    controller_requested_stop=True,
                )
            self._installed_path_signature = signature

        try:
            # ``set_path`` resets stateful critics.  Project extensions therefore
            # bind the current snapshot only after a possible path reset.
            for binder in self._snapshot_binders:
                binder.bind_snapshot(snapshot)
        except (TypeError, ValueError):
            return _result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.INVALID_INPUT,
                failure_reason="dwb_snapshot_binding_failed",
                decision_trace=(
                    f"path_signature={signature}",
                    "core_status=snapshot_binding_failed",
                ),
                controller_requested_stop=True,
            )

        self._prepared_snapshot_identity = _snapshot_identity(snapshot)
        return None

    def _compute_prepared(
        self,
        snapshot: ControllerSnapshot,
        started_at: int,
    ) -> ControllerCommandResult:
        signature = self._installed_path_signature
        if signature is None:  # pragma: no cover - preparation invariant
            raise RuntimeError("DWB snapshot was not prepared")

        request = _generator_request(snapshot)
        try:
            core_result = self._core.compute(request)
        except DwbPreparationError as error:
            return _result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.INVALID_INPUT,
                failure_reason="dwb_critic_preparation_failed",
                decision_trace=(
                    f"path_signature={signature}",
                    "core_status=preparation_failed",
                    f"critic={error.critic_name}",
                ),
                controller_requested_stop=True,
            )
        except NoLegalTrajectoryError as error:
            trace = [
                f"path_signature={signature}",
                "core_status=no_legal_trajectory",
                f"candidate_count={len(error.evaluations)}",
            ]
            trace.extend(
                f"rejection.{reason}={count}"
                for reason, count in sorted(error.failure_counts.items())
            )
            return _result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.NO_PATH,
                failure_reason="no_legal_dwb_trajectory",
                decision_trace=tuple(trace),
                controller_requested_stop=True,
                no_safe_candidate=True,
            )
        except ValueError:
            # A generator or injected core rejected the already frozen request.
            # Do not leak implementation-specific exception text into evidence.
            return _result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.INVALID_INPUT,
                failure_reason="dwb_core_request_rejected",
                decision_trace=(
                    f"path_signature={signature}",
                    "core_status=request_rejected",
                ),
                controller_requested_stop=True,
            )

        output_reason = self._invalid_output_reason(core_result)
        if output_reason is not None:
            return _result(
                self.name,
                snapshot,
                started_at,
                status=PlanStatus.INVALID_INPUT,
                failure_reason=output_reason,
                decision_trace=(
                    f"path_signature={signature}",
                    "core_status=invalid_output",
                    f"reason={output_reason}",
                ),
                controller_requested_stop=True,
            )

        evaluations = core_result.candidate_evaluations
        counts = Counter(evaluation.status for evaluation in evaluations)
        selected = evaluations[core_result.selected_candidate_index]
        trace = [
            f"path_signature={signature}",
            "core_status=success",
            f"candidate_count={len(evaluations)}",
            f"legal_candidates={counts[CandidateEvaluationStatus.LEGAL]}",
            f"illegal_candidates={counts[CandidateEvaluationStatus.ILLEGAL]}",
            (
                "short_circuited_candidates="
                f"{counts[CandidateEvaluationStatus.SHORT_CIRCUITED]}"
            ),
            f"selected_candidate_index={core_result.selected_candidate_index}",
            f"total_score={_float_token(core_result.total_score)}",
        ]
        trace.extend(
            (
                f"selected_critic.{score.critic_name}="
                f"{_float_token(score.raw_score)}*{_float_token(score.scale)}"
            )
            for score in selected.critic_scores
        )
        command = Twist2D(
            linear=core_result.command.linear_mps,
            angular=core_result.command.angular_radps,
        )
        return _result(
            self.name,
            snapshot,
            started_at,
            status=PlanStatus.FOUND,
            requested_twist=command,
            predicted_trajectory=_to_project_trajectory(core_result.trajectory),
            decision_trace=tuple(trace),
        )

    def _invalid_reason(self, snapshot: ControllerSnapshot) -> str | None:
        if snapshot.vehicle_profile != self.vehicle_profile:
            return "vehicle_profile_mismatch"
        if not (
            snapshot.vehicle_profile.simulation_only
            and snapshot.vehicle_profile.differential_drive
        ):
            return "vehicle_profile_not_supported"

        grid_snapshot = snapshot.static_grid_snapshot
        metadata = grid_snapshot.metadata
        if not grid_snapshot.input_valid:
            return "grid_snapshot_invalid"
        if (
            snapshot.map_id,
            snapshot.map_revision,
            snapshot.mission_revision,
            snapshot.observation_revision,
        ) != (
            metadata.map_id,
            metadata.map_revision,
            metadata.mission_revision,
            metadata.observation_revision,
        ):
            return "grid_provenance_mismatch"
        grid = grid_snapshot.grid
        if not all(
            isfinite(value)
            for value in (grid.resolution_m, grid.origin_x_m, grid.origin_y_m)
        ) or grid.resolution_m <= 0.0:
            return "grid_geometry_invalid"

        expected_hash = controller_snapshot_content_hash(
            tick_id=snapshot.tick_id,
            mission_id=snapshot.mission_id,
            map_id=snapshot.map_id,
            map_revision=snapshot.map_revision,
            mission_revision=snapshot.mission_revision,
            observation_revision=snapshot.observation_revision,
            grid_content_hash=metadata.content_hash,
            observation_content_hash=snapshot.observation_content_hash,
        )
        if snapshot.input_content_hash != expected_hash:
            return "snapshot_content_hash_mismatch"

        observation = snapshot.validated_observation
        frame = observation.frame
        if (
            observation.availability is not DynamicObservationAvailability.FRESH
            or not observation.usable
            or frame is None
            or observation.age_s is None
            or observation.failures
            or not isfinite(observation.age_s)
            or observation.age_s < 0.0
        ):
            return "fresh_observation_required"

        prediction = snapshot.actor_tubes
        if prediction is None:
            return "actor_prediction_missing"
        if type(prediction) not in (ActorPredictionSet, DirectionalPredictionSet):
            return "actor_prediction_content_mismatch"
        if (
            isinstance(prediction, DirectionalPredictionSet)
            and observation.last_event_was_no_frame
        ):
            return "fresh_observation_required"
        if (
            prediction.stream_id,
            prediction.episode_id,
            prediction.map_id,
            prediction.map_revision,
            prediction.observation_revision,
            prediction.sequence,
            prediction.source_content_hash,
        ) != (
            frame.stream_id,
            frame.episode_id,
            frame.map_id,
            frame.map_revision,
            frame.observation_revision,
            frame.sequence,
            frame.content_hash,
        ):
            return "actor_prediction_provenance_mismatch"
        if not (
            isclose(
                prediction.snapshot_age_s,
                observation.age_s,
                rel_tol=0.0,
                abs_tol=_TIME_TOLERANCE_S,
            )
            and isclose(
                prediction.controller_time_s,
                snapshot.simulation_time_s,
                rel_tol=0.0,
                abs_tol=_TIME_TOLERANCE_S,
            )
        ):
            return "actor_prediction_time_mismatch"
        frame_actor_ids = tuple(
            sorted((track.track_id, track.actor_binding_id) for track in frame.tracks)
        )
        tube_actor_ids = tuple(
            sorted((tube.track_id, tube.actor_binding_id) for tube in prediction.tubes)
        )
        if (
            len(frame_actor_ids) != len(set(frame_actor_ids))
            or len(tube_actor_ids) != len(set(tube_actor_ids))
            or frame_actor_ids != tube_actor_ids
        ):
            return "actor_prediction_identity_mismatch"
        if isinstance(prediction, DirectionalPredictionSet):
            try:
                validate_directional_prediction_set(
                    prediction,
                    current_frame=frame,
                )
            except (TypeError, ValueError):
                return "actor_prediction_content_mismatch"
        elif isinstance(prediction, ActorPredictionSet):
            # The legacy one-frame circular model remains exactly rebuildable
            # from the current validated observation.
            try:
                expected_prediction = build_actor_prediction_set(observation)
            except (TypeError, ValueError):
                return "actor_prediction_content_mismatch"
            if prediction != expected_prediction:
                return "actor_prediction_content_mismatch"
        else:
            return "actor_prediction_content_mismatch"

        twist = snapshot.robot_state.twist
        if not isfinite(twist.linear) or not isfinite(twist.angular):
            return "robot_twist_non_finite"
        configured_minimum = (
            -self.generator_config.maximum_reverse_speed_mps
            if self.generator_config.allow_reverse
            else 0.0
        )
        if not configured_minimum <= twist.linear <= self.vehicle_profile.max_forward_speed_mps:
            return "linear_state_outside_frozen_range"
        if abs(twist.angular) > self.vehicle_profile.max_angular_speed_radps:
            return "angular_state_outside_vehicle_limits"
        return None

    def _invalid_output_reason(self, result: DwbCoreResult) -> str | None:
        command = result.command
        minimum_linear, maximum_linear = self._command_linear_bounds
        if not minimum_linear <= command.linear_mps <= maximum_linear:
            return "core_linear_command_outside_frozen_range"
        if abs(command.angular_radps) > self.vehicle_profile.max_angular_speed_radps:
            return "core_angular_command_outside_vehicle_limits"
        trajectory = result.trajectory
        if trajectory.command != command:
            return "core_command_trajectory_mismatch"
        if len(trajectory.poses) != self.generator_config.rollout_step_count + 1:
            return "core_rollout_pose_count_mismatch"
        if not isclose(
            trajectory.integration_step_s,
            self.generator_config.integration_step_s,
            rel_tol=0.0,
            abs_tol=_TIME_TOLERANCE_S,
        ):
            return "core_rollout_step_mismatch"
        if not 0 <= result.selected_candidate_index < len(result.candidate_evaluations):
            return "core_selected_index_invalid"
        selected = result.candidate_evaluations[result.selected_candidate_index]
        if (
            selected.status is not CandidateEvaluationStatus.LEGAL
            or selected.command != command
        ):
            return "core_selected_diagnostic_mismatch"
        return None


def reference_path_signature(path: Sequence[DwbPose2D]) -> str:
    """Return a stable, float-exact signature used only for path reset lifetime."""

    frozen = tuple(path)
    if not frozen:
        raise ValueError("path must not be empty")
    payload = [
        (pose.x_m.hex(), pose.y_m.hex(), pose.yaw_rad.hex())
        for pose in frozen
    ]
    return sha256(
        dumps(payload, ensure_ascii=True, separators=(",", ":")).encode("ascii")
    ).hexdigest()


def _controller_path_signature(
    snapshot: ControllerSnapshot,
    path: Sequence[DwbPose2D],
) -> str:
    """Reset path-bound state across mission/map/goal changes, not observations."""

    goal = snapshot.goal_pose
    payload = {
        "map_id": snapshot.map_id,
        "map_revision": snapshot.map_revision,
        "mission_id": snapshot.mission_id,
        "mission_revision": snapshot.mission_revision,
        "goal": (goal.x.hex(), goal.y.hex(), goal.yaw.hex()),
        "reference_path_signature": reference_path_signature(path),
    }
    return sha256(
        dumps(payload, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode(
            "ascii"
        )
    ).hexdigest()


def source_derived_dwb_semantic_digest(result: ControllerCommandResult) -> str:
    """Hash controller semantics while deliberately excluding wall-clock elapsed."""

    payload = {
        "controller_name": result.controller_name,
        "source_tick_id": result.source_tick_id,
        "status": result.status.value,
        "requested_twist": (
            _float_token(result.requested_twist.linear),
            _float_token(result.requested_twist.angular),
        ),
        "predicted_trajectory": [
            (
                _float_token(point.time_s),
                _float_token(point.pose.x),
                _float_token(point.pose.y),
                _float_token(point.pose.yaw),
                _float_token(point.twist.linear),
                _float_token(point.twist.angular),
            )
            for point in result.predicted_trajectory
        ],
        "failure_reason": result.failure_reason,
        "decision_trace": [
            item
            for item in result.decision_trace
            if not item.startswith(_SEMANTIC_DIGEST_PREFIX)
        ],
        "mission_id": result.mission_id,
        "map_id": result.map_id,
        "map_revision": result.map_revision,
        "mission_revision": result.mission_revision,
        "observation_revision": result.observation_revision,
        "grid_content_hash": result.grid_content_hash,
        "observation_content_hash": result.observation_content_hash,
        "input_content_hash": result.input_content_hash,
        "controller_requested_stop": result.controller_requested_stop,
        "no_safe_candidate": result.no_safe_candidate,
    }
    return sha256(
        dumps(
            payload,
            ensure_ascii=True,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("ascii")
    ).hexdigest()


def _generator_config_for(profile: VehicleProfile) -> DwbGeneratorConfig:
    return DwbGeneratorConfig(
        control_period_s=profile.control_period_s,
        maximum_forward_speed_mps=profile.max_forward_speed_mps,
        maximum_reverse_speed_mps=profile.max_reverse_speed_mps,
        linear_acceleration_mps2=profile.max_acceleration_mps2,
        linear_deceleration_mps2=profile.max_deceleration_mps2,
        maximum_angular_speed_radps=profile.max_angular_speed_radps,
        allow_reverse=False,
    )


def _validate_generator_profile(
    config: DwbGeneratorConfig,
    profile: VehicleProfile,
    *,
    allow_reverse: bool = False,
) -> None:
    expected = (
        (config.control_period_s, profile.control_period_s),
        (config.maximum_forward_speed_mps, profile.max_forward_speed_mps),
        (config.maximum_reverse_speed_mps, profile.max_reverse_speed_mps),
        (config.linear_acceleration_mps2, profile.max_acceleration_mps2),
        (config.linear_deceleration_mps2, profile.max_deceleration_mps2),
        (config.maximum_angular_speed_radps, profile.max_angular_speed_radps),
    )
    if config.allow_reverse is not allow_reverse or any(
        not isclose(left, right, rel_tol=0.0, abs_tol=_FLOAT_TOLERANCE)
        for left, right in expected
    ):
        direction = "signed" if allow_reverse else "forward-only"
        raise ValueError(f"generator config must match the frozen {direction} profile")


def _generator_request(snapshot: ControllerSnapshot) -> DwbGeneratorRequest:
    pose = snapshot.robot_state.pose
    twist = snapshot.robot_state.twist
    post_apply = _integrate_pose(
        pose,
        twist,
        DYNAMIC_COMMAND_APPLY_LATENCY_S,
    )
    return DwbGeneratorRequest(
        pose=_to_dwb_pose(post_apply),
        current_twist=DwbTwist2D(twist.linear, twist.angular),
    )


def _integrate_pose(pose: Pose2D, command: Twist2D, duration_s: float) -> Pose2D:
    if abs(command.angular) <= _FLOAT_TOLERANCE:
        return Pose2D(
            x=pose.x + command.linear * cos(pose.yaw) * duration_s,
            y=pose.y + command.linear * sin(pose.yaw) * duration_s,
            yaw=pose.yaw,
        )
    next_yaw = pose.yaw + command.angular * duration_s
    radius = command.linear / command.angular
    return Pose2D(
        x=pose.x + radius * (sin(next_yaw) - sin(pose.yaw)),
        y=pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
        yaw=atan2(sin(next_yaw), cos(next_yaw)),
    )


def _to_dwb_pose(pose: Pose2D) -> DwbPose2D:
    return DwbPose2D(pose.x, pose.y, pose.yaw)


def _to_project_trajectory(trajectory: DwbTrajectory) -> tuple[TrajectoryPoint, ...]:
    command = Twist2D(
        trajectory.command.linear_mps,
        trajectory.command.angular_radps,
    )
    return tuple(
        TrajectoryPoint(
            time_s=index * trajectory.integration_step_s,
            pose=Pose2D(pose.x_m, pose.y_m, pose.yaw_rad),
            twist=command,
        )
        for index, pose in enumerate(trajectory.poses)
    )


def _result(
    controller_name: str,
    snapshot: ControllerSnapshot,
    started_at: int,
    *,
    status: PlanStatus,
    requested_twist: Twist2D | None = None,
    predicted_trajectory: tuple[TrajectoryPoint, ...] = (),
    failure_reason: str | None = None,
    decision_trace: tuple[str, ...] = (),
    controller_requested_stop: bool = False,
    no_safe_candidate: bool = False,
) -> ControllerCommandResult:
    metadata = snapshot.static_grid_snapshot.metadata
    # A malformed self-declared snapshot hash cannot be copied into the result:
    # ControllerCommandResult correctly requires a hash derived from the immutable
    # provenance fields.  Re-deriving it records the rejected input identity
    # without laundering the malformed claim as valid output provenance.
    result_input_hash = controller_snapshot_content_hash(
        tick_id=snapshot.tick_id,
        mission_id=snapshot.mission_id,
        map_id=snapshot.map_id,
        map_revision=snapshot.map_revision,
        mission_revision=snapshot.mission_revision,
        observation_revision=snapshot.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=snapshot.observation_content_hash,
    )
    provisional = ControllerCommandResult(
        controller_name=controller_name,
        source_tick_id=snapshot.tick_id,
        status=status,
        requested_twist=requested_twist or Twist2D(),
        predicted_trajectory=predicted_trajectory,
        failure_reason=failure_reason,
        decision_trace=decision_trace,
        mission_id=snapshot.mission_id,
        map_id=snapshot.map_id,
        map_revision=snapshot.map_revision,
        mission_revision=snapshot.mission_revision,
        observation_revision=snapshot.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=snapshot.observation_content_hash,
        input_content_hash=result_input_hash,
        elapsed_ns=perf_counter_ns() - started_at,
        controller_requested_stop=controller_requested_stop,
        no_safe_candidate=no_safe_candidate,
    )
    digest = source_derived_dwb_semantic_digest(provisional)
    return replace(
        provisional,
        elapsed_ns=perf_counter_ns() - started_at,
        decision_trace=(*decision_trace, f"{_SEMANTIC_DIGEST_PREFIX}{digest}"),
    )


def _float_token(value: float) -> str:
    return "0x0.0p+0" if value == 0.0 else value.hex()


def _snapshot_identity(snapshot: ControllerSnapshot) -> tuple[int, str]:
    return snapshot.tick_id, _controller_snapshot_semantic_digest(snapshot)


def _controller_snapshot_semantic_digest(snapshot: ControllerSnapshot) -> str:
    """Hash the complete immutable controller input for prepare/compute binding.

    ``input_content_hash`` intentionally identifies provenance only.  It omits
    command-relevant values including robot state, goal/path geometry, Actor
    tubes, and the occupancy bytes.  The prepared-core boundary therefore uses
    this private full semantic digest rather than treating that narrow hash as
    an equality contract.
    """

    payload = _semantic_digest_payload(snapshot)
    serialized = dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    return sha256(serialized.encode("utf-8")).hexdigest()


def _semantic_digest_payload(value: object) -> object:
    """Return a deterministic, type-tagged JSON value for frozen contracts."""

    if isinstance(value, Enum):
        return {
            "__enum__": f"{type(value).__module__}.{type(value).__qualname__}",
            "value": _semantic_digest_payload(value.value),
        }
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return {"__float__": value.hex()}
    if isinstance(value, np.ndarray):
        contiguous = np.ascontiguousarray(value)
        return {
            "__ndarray__": True,
            "dtype": contiguous.dtype.str,
            "shape": list(contiguous.shape),
            "sha256": sha256(contiguous.tobytes()).hexdigest(),
        }
    if is_dataclass(value) and not isinstance(value, type):
        return {
            "__dataclass__": f"{type(value).__module__}.{type(value).__qualname__}",
            "fields": {
                item.name: _semantic_digest_payload(getattr(value, item.name))
                for item in fields(value)
            },
        }
    if isinstance(value, (tuple, list)):
        return {
            "__sequence__": type(value).__name__,
            "items": [_semantic_digest_payload(item) for item in value],
        }
    if isinstance(value, (set, frozenset)):
        items = [_semantic_digest_payload(item) for item in value]
        items.sort(
            key=lambda item: dumps(
                item,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
        )
        return {"__set__": type(value).__name__, "items": items}
    raise TypeError(f"unsupported snapshot digest value: {type(value).__qualname__}")


__all__ = [
    "DwbCore",
    "SnapshotBinder",
    "SourceDerivedDwbController",
    "reference_path_signature",
    "source_derived_dwb_semantic_digest",
]
