"""Project hard-safety constraint for source-derived DWB trajectories.

The upstream-derived DWB core deliberately knows nothing about this project's
static map, forbidden areas, Actor prediction tubes, terminal stopping sweep, or
snapshot provenance.  This critic is the explicit boundary between those two
layers.  It delegates the actual geometry decision to the already shared
``evaluate_dynamic_trajectory_safety`` function, so candidate filtering and the
online safety gate use the same simulation-only contract.

This module does not validate observation freshness or authorization.  Those are
adapter and online-gate responsibilities; callers must bind a fresh,
provenance-checked :class:`ControllerSnapshot` for each control tick.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isclose, pi, sin

from hospital_path_lab.contracts import Pose2D, TrajectoryPoint, Twist2D
from hospital_path_lab.cpp_dwb_safety_core import (
    CppDwbSafetyFailure,
    evaluate_dwb_safety_batch,
)
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_COMMAND_APPLY_LATENCY_S,
    ControllerSnapshot,
    DynamicCommandProposal,
)
from hospital_path_lab.dynamic_safety import (
    DynamicTrajectorySafetyCheckers,
    DynamicTrajectorySafetyEvidence,
    build_dynamic_trajectory_safety_checkers,
    evaluate_dynamic_trajectory_safety,
)
from hospital_path_lab.local_algorithms.dwb_reference.contracts import (
    DwbGeneratorRequest,
    DwbPose2D,
    DwbTrajectory,
    DwbTwist2D,
)
from hospital_path_lab.local_algorithms.dwb_reference.core import (
    CriticBatchScore,
    IllegalTrajectoryError,
)

_POSE_ABSOLUTE_TOLERANCE = 1e-9


@dataclass(frozen=True, slots=True)
class DynamicTrajectoryConstraintRecord:
    """One candidate's exact project proposal and shared-safety evidence."""

    proposal: DynamicCommandProposal
    evidence: DynamicTrajectorySafetyEvidence


class ProjectDynamicSafetyConstraintCritic:
    """Reject DWB candidates that violate the existing project safety contract.

    Lifecycle is intentionally explicit:

    ``bind_snapshot`` -> ``prepare`` -> one or more ``score`` calls -> ``debrief``.

    Binding a new snapshot or resetting the critic discards all candidate evidence
    from the previous tick.  ``debrief`` retains only the record belonging to the
    command selected by the DWB core.
    """

    name = "project_dynamic_safety_constraint"

    def __init__(self, *, use_cpp_batch: bool = True) -> None:
        if not isinstance(use_cpp_batch, bool):
            raise TypeError("use_cpp_batch must be bool")
        self._use_cpp_batch = use_cpp_batch
        self._snapshot: ControllerSnapshot | None = None
        self._checkers: DynamicTrajectorySafetyCheckers | None = None
        self._prepared = False
        self._records: dict[DwbTrajectory, DynamicTrajectoryConstraintRecord] = {}
        self._batch_trajectories: tuple[DwbTrajectory, ...] = ()
        self._selected_record: DynamicTrajectoryConstraintRecord | None = None
        self._native_batch_used = False

    @property
    def bound_snapshot(self) -> ControllerSnapshot | None:
        """The immutable controller input currently bound to this critic."""

        return self._snapshot

    @property
    def selected_record(self) -> DynamicTrajectoryConstraintRecord | None:
        """Safety record for the command selected during the latest debrief."""

        return self._selected_record

    @property
    def selected_evidence(self) -> DynamicTrajectorySafetyEvidence | None:
        """Convenience view of the selected candidate's safety evidence."""

        record = self._selected_record
        return None if record is None else record.evidence

    @property
    def cached_candidate_count(self) -> int:
        """Number of candidates evaluated in the current prepared tick."""

        return len(self._records)

    @property
    def native_batch_used(self) -> bool:
        """Whether the most recently prepared tick used the optional C++ batch."""

        return self._native_batch_used

    def bind_snapshot(self, snapshot: ControllerSnapshot) -> None:
        """Bind one already validated controller snapshot and start a new tick."""

        if not isinstance(snapshot, ControllerSnapshot):
            raise TypeError("snapshot must be a ControllerSnapshot")
        if not snapshot.vehicle_profile.simulation_only:
            raise ValueError("dynamic constraint requires a simulation-only profile")
        self._snapshot = snapshot
        self._checkers = build_dynamic_trajectory_safety_checkers(
            grid_snapshot=snapshot.static_grid_snapshot,
            profile=snapshot.vehicle_profile,
        )
        self._prepared = False
        self._records.clear()
        self._batch_trajectories = ()
        self._selected_record = None
        self._native_batch_used = False

    def prepare(self, request: DwbGeneratorRequest) -> bool:
        """Verify that DWB starts at the shared contract's 50 ms apply pose."""

        snapshot = self._require_snapshot()
        self._records.clear()
        self._batch_trajectories = ()
        self._selected_record = None
        self._native_batch_used = False
        self._prepared = False

        expected_pose = _post_apply_pose(snapshot)
        if not _dwb_pose_matches_pose(request.pose, expected_pose):
            return False
        if not _dwb_twist_matches_twist(request.current_twist, snapshot.robot_state.twist):
            return False

        self._prepared = True
        return True

    def score_batch(
        self,
        trajectories: tuple[DwbTrajectory, ...],
    ) -> tuple[CriticBatchScore, ...] | None:
        """Use the optional C++ core for one complete candidate safety batch."""

        snapshot = self._require_prepared_snapshot()
        checkers = self._require_checkers()
        if not self._use_cpp_batch:
            return None
        results = evaluate_dwb_safety_batch(
            trajectories=trajectories,
            snapshot=snapshot,
            checkers=checkers,
        )
        if results is None:
            return None
        self._native_batch_used = True
        self._batch_trajectories = tuple(trajectories)
        reason_codes = {
            CppDwbSafetyFailure.FORBIDDEN_ZONE: "forbidden_zone_entry",
            CppDwbSafetyFailure.STATIC_CLEARANCE: "static_clearance_below_minimum",
            CppDwbSafetyFailure.ACTOR_CLEARANCE: "actor_clearance_below_minimum",
            CppDwbSafetyFailure.PREDICTION_INVALID: "prediction_set_malformed",
        }
        return tuple(
            CriticBatchScore(raw_score=0.0)
            if result.failure is CppDwbSafetyFailure.SAFE
            else CriticBatchScore(
                reason_code=reason_codes[result.failure],
                message=reason_codes[result.failure],
            )
            for result in results
        )

    def score(self, trajectory: DwbTrajectory) -> float:
        """Return zero for a safe candidate or raise with all failure evidence."""

        snapshot = self._require_prepared_snapshot()
        checkers = self._require_checkers()
        record = self._records.get(trajectory)
        if record is None:
            proposal = _proposal_from_trajectory(snapshot, trajectory)
            evidence = evaluate_dynamic_trajectory_safety(
                proposal,
                robot_state=snapshot.robot_state,
                grid_snapshot=snapshot.static_grid_snapshot,
                prediction_set=snapshot.actor_tubes,
                profile=snapshot.vehicle_profile,
                checkers=checkers,
            )
            record = DynamicTrajectoryConstraintRecord(proposal, evidence)
            self._records[trajectory] = record

        if record.evidence.safe:
            return 0.0

        failures = record.evidence.failures or ("dynamic_safety_rejected",)
        raise IllegalTrajectoryError(
            reason_code=failures[0],
            message="; ".join(failures),
        )

    def debrief(self, selected_command: DwbTwist2D) -> None:
        """Retain the selected command's safe record for reporting."""

        self._require_prepared_snapshot()
        matches = tuple(
            record
            for trajectory, record in self._records.items()
            if trajectory.command == selected_command
        )
        if not matches and self._batch_trajectories:
            trajectories = tuple(
                trajectory
                for trajectory in self._batch_trajectories
                if trajectory.command == selected_command
            )
            if len(trajectories) != 1:
                raise RuntimeError(
                    "selected command must identify exactly one native batch trajectory"
                )
            self.score(trajectories[0])
            matches = tuple(
                record
                for trajectory, record in self._records.items()
                if trajectory.command == selected_command
            )
        if len(matches) != 1:
            raise RuntimeError(
                "selected command must identify exactly one scored safety record"
            )
        if not matches[0].evidence.safe:
            raise RuntimeError("an unsafe candidate cannot be selected")
        self._selected_record = matches[0]

    def reset(self) -> None:
        """Discard snapshot, preparation state, and all per-tick evidence."""

        self._snapshot = None
        self._checkers = None
        self._prepared = False
        self._records.clear()
        self._batch_trajectories = ()
        self._selected_record = None
        self._native_batch_used = False

    def evidence_for(
        self,
        trajectory: DwbTrajectory,
    ) -> DynamicTrajectorySafetyEvidence | None:
        """Return cached evidence without evaluating or mutating the candidate."""

        record = self._records.get(trajectory)
        return None if record is None else record.evidence

    def proposal_for(self, trajectory: DwbTrajectory) -> DynamicCommandProposal | None:
        """Return the exact provenance-bearing proposal sent to shared safety."""

        record = self._records.get(trajectory)
        return None if record is None else record.proposal

    def _require_snapshot(self) -> ControllerSnapshot:
        if self._snapshot is None:
            raise RuntimeError("bind_snapshot must be called before prepare")
        return self._snapshot

    def _require_prepared_snapshot(self) -> ControllerSnapshot:
        snapshot = self._require_snapshot()
        if not self._prepared:
            raise RuntimeError("prepare must succeed before candidate scoring")
        return snapshot

    def _require_checkers(self) -> DynamicTrajectorySafetyCheckers:
        if self._checkers is None:
            raise RuntimeError("bind_snapshot must build safety checkers before scoring")
        return self._checkers


def _proposal_from_trajectory(
    snapshot: ControllerSnapshot,
    trajectory: DwbTrajectory,
) -> DynamicCommandProposal:
    points = tuple(
        TrajectoryPoint(
            time_s=index * trajectory.integration_step_s,
            pose=Pose2D(pose.x_m, pose.y_m, pose.yaw_rad),
            twist=Twist2D(
                trajectory.command.linear_mps,
                trajectory.command.angular_radps,
            ),
        )
        for index, pose in enumerate(trajectory.poses)
    )
    metadata = snapshot.static_grid_snapshot.metadata
    return DynamicCommandProposal(
        source_tick_id=snapshot.tick_id,
        command=Twist2D(
            trajectory.command.linear_mps,
            trajectory.command.angular_radps,
        ),
        computation_time_s=0.0,
        mission_id=snapshot.mission_id,
        map_id=snapshot.map_id,
        map_revision=snapshot.map_revision,
        mission_revision=snapshot.mission_revision,
        observation_revision=snapshot.observation_revision,
        grid_content_hash=metadata.content_hash,
        observation_content_hash=snapshot.observation_content_hash,
        trajectory=points,
    )


def _post_apply_pose(snapshot: ControllerSnapshot) -> Pose2D:
    pose = snapshot.robot_state.pose
    twist = snapshot.robot_state.twist
    duration_s = DYNAMIC_COMMAND_APPLY_LATENCY_S
    if abs(twist.angular) <= 1e-12:
        return Pose2D(
            x=pose.x + twist.linear * cos(pose.yaw) * duration_s,
            y=pose.y + twist.linear * sin(pose.yaw) * duration_s,
            yaw=pose.yaw,
        )
    next_yaw = pose.yaw + twist.angular * duration_s
    radius = twist.linear / twist.angular
    return Pose2D(
        x=pose.x + radius * (sin(next_yaw) - sin(pose.yaw)),
        y=pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
        yaw=_normalize_angle(next_yaw),
    )


def _dwb_pose_matches_pose(dwb: DwbPose2D, project: Pose2D) -> bool:
    return all(
        isclose(left, right, rel_tol=0.0, abs_tol=_POSE_ABSOLUTE_TOLERANCE)
        for left, right in (
            (dwb.x_m, project.x),
            (dwb.y_m, project.y),
            (_normalize_angle(dwb.yaw_rad), _normalize_angle(project.yaw)),
        )
    )


def _dwb_twist_matches_twist(dwb: DwbTwist2D, project: Twist2D) -> bool:
    return all(
        isclose(left, right, rel_tol=0.0, abs_tol=_POSE_ABSOLUTE_TOLERANCE)
        for left, right in (
            (dwb.linear_mps, project.linear),
            (dwb.angular_radps, project.angular),
        )
    )


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi
