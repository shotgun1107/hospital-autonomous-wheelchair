"""R4 immutable full reference의 stateful sliding-window manager.

동일 full reference 안의 window 이동만 같은 controller session으로 유지한다. 새 maneuver,
path, stop epoch 또는 session은 이 manager가 자동 수용하지 않으며 상위 계층이 revision
판정 뒤 새 manager를 만들어야 한다. 이 모듈은 controller 명령이나 이동 허가를 만들지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from math import atan2, cos, hypot, isfinite, sin
from re import fullmatch

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.local_reference_contracts import (
    LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION,
    LocalManeuverReference,
    LocalReferenceWindow,
    ReferenceBuildContext,
    ReferenceKnotRole,
    ReferenceSectionKind,
)
from hospital_path_lab.local_reference_validation import LocalReferenceValidation
from hospital_path_lab.map_factory import canonical_content_hash

LOCAL_REFERENCE_WINDOW_MANAGER_VERSION = "local-reference-window-manager-v1"
LOCAL_REFERENCE_WINDOW_UPDATE_SCHEMA_VERSION = "local-reference-window-update-v1"
R4_WINDOW_CONTROL_PERIOD_S = 0.05
R4_REAR_CONTEXT_ARC_M = 0.10
R4_MINIMUM_FORWARD_WINDOW_ARC_M = 0.60
R4_WINDOW_ADVANCE_QUANTUM_M = 0.10
R4_PROJECTION_TIE_TOLERANCE_M = 1e-9
R4_MAXIMUM_CURSOR_REGRESSION_M = 0.05
_TOLERANCE = 1e-9


class WindowUpdateStatus(StrEnum):
    WINDOW_READY = "window_ready"
    INVALID_INPUT = "invalid_input"
    STALE_INPUT = "stale_input"


@dataclass(frozen=True, slots=True)
class ReferenceCursorProjection:
    cursor_arc_m: float
    distance_to_reference_m: float
    source_section_index: int
    source_edge_start_knot_index: int
    ambiguous: bool
    ambiguity_reason: str | None

    def __post_init__(self) -> None:
        for name in ("cursor_arc_m", "distance_to_reference_m"):
            value = getattr(self, name)
            if not isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        for name in ("source_section_index", "source_edge_start_knot_index"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a non-negative exact integer")
        if not isinstance(self.ambiguous, bool):
            raise TypeError("ambiguous must be a bool")
        if self.ambiguous != (self.ambiguity_reason is not None):
            raise ValueError("ambiguous projection must carry exactly one reason state")


@dataclass(frozen=True, slots=True)
class LocalReferenceWindowUpdate:
    schema_version: str
    manager_version: str
    status: WindowUpdateStatus
    reason_code: str
    build_context_hash: str
    reference_content_hash: str
    validation_content_hash: str
    source_control_tick: int
    raw_cursor_arc_m: float | None
    effective_cursor_arc_m: float | None
    projection_distance_m: float | None
    window: LocalReferenceWindow | None
    semantic_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != LOCAL_REFERENCE_WINDOW_UPDATE_SCHEMA_VERSION:
            raise ValueError("unsupported local reference window update schema")
        if self.manager_version != LOCAL_REFERENCE_WINDOW_MANAGER_VERSION:
            raise ValueError("unsupported local reference window manager version")
        if not isinstance(self.status, WindowUpdateStatus):
            raise TypeError("status must be a WindowUpdateStatus")
        if not isinstance(self.reason_code, str) or not self.reason_code:
            raise ValueError("reason_code must not be empty")
        for name in (
            "build_context_hash",
            "reference_content_hash",
            "validation_content_hash",
        ):
            _require_sha256(getattr(self, name), name)
        if (
            isinstance(self.source_control_tick, bool)
            or not isinstance(self.source_control_tick, int)
            or self.source_control_tick < 0
        ):
            raise ValueError("source_control_tick must be a non-negative exact integer")
        for name in (
            "raw_cursor_arc_m",
            "effective_cursor_arc_m",
            "projection_distance_m",
        ):
            value = getattr(self, name)
            if value is not None and (not isfinite(value) or value < 0.0):
                raise ValueError(f"{name} must be finite and non-negative when present")
        ready = self.status is WindowUpdateStatus.WINDOW_READY
        if ready != (self.window is not None):
            raise ValueError("only WINDOW_READY may carry a window")
        if ready and any(
            value is None
            for value in (
                self.raw_cursor_arc_m,
                self.effective_cursor_arc_m,
                self.projection_distance_m,
            )
        ):
            raise ValueError("ready window update requires projection metrics")
        expected = self.expected_content_hash
        if self.semantic_content_hash:
            _require_sha256(self.semantic_content_hash, "semantic_content_hash")
            if self.semantic_content_hash != expected:
                raise ValueError("semantic_content_hash mismatch")
        else:
            object.__setattr__(self, "semantic_content_hash", expected)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "manager_version": self.manager_version,
                "status": self.status,
                "reason_code": self.reason_code,
                "build_context_hash": self.build_context_hash,
                "reference_content_hash": self.reference_content_hash,
                "validation_content_hash": self.validation_content_hash,
                "source_control_tick": self.source_control_tick,
                "raw_cursor_arc_m": self.raw_cursor_arc_m,
                "effective_cursor_arc_m": self.effective_cursor_arc_m,
                "projection_distance_m": self.projection_distance_m,
                "window": self.window,
            }
        )


class LocalReferenceWindowManager:
    """하나의 immutable full reference session에만 결박되는 window manager."""

    def __init__(self) -> None:
        self._identity: tuple[object, ...] | None = None
        self._last_control_tick: int | None = None
        self._last_input_digest: str | None = None
        self._last_cursor_arc_m: float | None = None
        self._last_window: LocalReferenceWindow | None = None
        self._last_update: LocalReferenceWindowUpdate | None = None

    def update(
        self,
        context: ReferenceBuildContext,
        reference: LocalManeuverReference,
        validation: LocalReferenceValidation,
    ) -> LocalReferenceWindowUpdate:
        if not isinstance(context, ReferenceBuildContext):
            raise TypeError("context must be a ReferenceBuildContext")
        if not isinstance(reference, LocalManeuverReference):
            raise TypeError("reference must be a LocalManeuverReference")
        if not isinstance(validation, LocalReferenceValidation):
            raise TypeError("validation must be a LocalReferenceValidation")

        reason = _input_failure(context, reference, validation)
        if reason is not None:
            return _failure_update(
                WindowUpdateStatus.INVALID_INPUT,
                reason,
                context,
                reference,
                validation,
            )
        identity = _reference_identity(reference)
        if self._identity is not None and identity != self._identity:
            return _failure_update(
                WindowUpdateStatus.STALE_INPUT,
                "reference_session_or_path_changed",
                context,
                reference,
                validation,
            )
        input_digest = _window_input_digest(context, reference, validation)
        if self._last_control_tick is not None:
            if context.control_tick < self._last_control_tick:
                return _failure_update(
                    WindowUpdateStatus.STALE_INPUT,
                    "source_control_tick_regression",
                    context,
                    reference,
                    validation,
                )
            if context.control_tick == self._last_control_tick:
                if input_digest == self._last_input_digest:
                    assert self._last_update is not None
                    return self._last_update
                return _failure_update(
                    WindowUpdateStatus.INVALID_INPUT,
                    "same_tick_different_input",
                    context,
                    reference,
                    validation,
                )

        projection = project_reference_cursor(
            reference,
            context.current_robot_pose,
            cursor_hint_m=self._last_cursor_arc_m,
        )
        if projection.ambiguous:
            return _failure_update(
                WindowUpdateStatus.INVALID_INPUT,
                projection.ambiguity_reason or "ambiguous_reference_projection",
                context,
                reference,
                validation,
                raw_cursor_arc_m=projection.cursor_arc_m,
                projection_distance_m=projection.distance_to_reference_m,
            )
        raw_cursor = projection.cursor_arc_m
        if self._last_cursor_arc_m is not None and raw_cursor < (
            self._last_cursor_arc_m - R4_MAXIMUM_CURSOR_REGRESSION_M - _TOLERANCE
        ):
            return _failure_update(
                WindowUpdateStatus.STALE_INPUT,
                "cursor_regression_exceeded",
                context,
                reference,
                validation,
                raw_cursor_arc_m=raw_cursor,
                projection_distance_m=projection.distance_to_reference_m,
            )
        effective_cursor = (
            raw_cursor
            if self._last_cursor_arc_m is None
            else max(raw_cursor, self._last_cursor_arc_m)
        )
        start_section, end_section = _window_section_range(
            reference,
            projection.source_section_index,
            effective_cursor,
        )
        bounds = (
            reference.sections[start_section].first_knot_index,
            reference.sections[end_section].last_knot_index,
        )
        previous = self._last_window
        # R4 v1 window는 whole-section slice이므로 bounds 변경 자체가 명세의
        # "atomic section changed" 조건이다.  같은 section 안에서 cursor가
        # R4_WINDOW_ADVANCE_QUANTUM_M만큼 움직여도 slice가 같으면 revision을 올리지 않는다.
        same_slice = previous is not None and bounds == (
            previous.start_knot_index,
            previous.end_knot_index,
        )
        subgoal_revision = (
            0 if previous is None else previous.subgoal_revision + (0 if same_slice else 1)
        )
        window = _build_window(
            reference,
            start_section,
            end_section,
            source_control_tick=context.control_tick,
            subgoal_revision=subgoal_revision,
        )
        window_reason = (
            "initial_window"
            if previous is None
            else ("window_unchanged" if same_slice else "window_advanced")
        )
        if not window_is_exact_slice(reference, window):
            return _failure_update(
                WindowUpdateStatus.INVALID_INPUT,
                "window_not_contiguous",
                context,
                reference,
                validation,
                raw_cursor_arc_m=raw_cursor,
                projection_distance_m=projection.distance_to_reference_m,
            )
        update = LocalReferenceWindowUpdate(
            schema_version=LOCAL_REFERENCE_WINDOW_UPDATE_SCHEMA_VERSION,
            manager_version=LOCAL_REFERENCE_WINDOW_MANAGER_VERSION,
            status=WindowUpdateStatus.WINDOW_READY,
            reason_code=window_reason,
            build_context_hash=context.context_content_hash,
            reference_content_hash=reference.reference_content_hash,
            validation_content_hash=validation.validation_content_hash,
            source_control_tick=context.control_tick,
            raw_cursor_arc_m=raw_cursor,
            effective_cursor_arc_m=effective_cursor,
            projection_distance_m=projection.distance_to_reference_m,
            window=window,
        )
        self._identity = identity
        self._last_control_tick = context.control_tick
        self._last_input_digest = input_digest
        self._last_cursor_arc_m = effective_cursor
        self._last_window = window
        self._last_update = update
        return update


def project_reference_cursor(
    reference: LocalManeuverReference,
    robot_pose: Pose2D,
    *,
    cursor_hint_m: float | None = None,
) -> ReferenceCursorProjection:
    """translation edges에 투영하고 self-overlap ambiguity를 fail-closed 표시한다."""

    if not isinstance(reference, LocalManeuverReference):
        raise TypeError("reference must be a LocalManeuverReference")
    if not isinstance(robot_pose, Pose2D) or not all(
        isfinite(value) for value in (robot_pose.x, robot_pose.y, robot_pose.yaw)
    ):
        raise ValueError("robot_pose must be a finite Pose2D")
    if cursor_hint_m is not None and (not isfinite(cursor_hint_m) or cursor_hint_m < 0.0):
        raise ValueError("cursor_hint_m must be finite and non-negative when present")
    candidates: list[tuple[float, float, int, int, float, float, float, float, float]] = []
    knots = reference.knots
    for edge_index, (left, right) in enumerate(zip(knots, knots[1:], strict=False)):
        dx = right.pose.x - left.pose.x
        dy = right.pose.y - left.pose.y
        length = hypot(dx, dy)
        if length <= _TOLERANCE:
            continue
        fraction = max(
            0.0,
            min(
                1.0,
                ((robot_pose.x - left.pose.x) * dx + (robot_pose.y - left.pose.y) * dy)
                / (length * length),
            ),
        )
        projected_x = left.pose.x + fraction * dx
        projected_y = left.pose.y + fraction * dy
        distance = hypot(robot_pose.x - projected_x, robot_pose.y - projected_y)
        cursor = left.cumulative_translation_arc_m + fraction * length
        section_index = left.section_index
        candidates.append(
            (
                distance,
                cursor,
                section_index,
                edge_index,
                dx / length,
                dy / length,
                projected_x,
                projected_y,
                abs(_angle_delta(robot_pose.yaw, atan2(dy, dx))),
            )
        )
    for knot in knots:
        distance = hypot(robot_pose.x - knot.pose.x, robot_pose.y - knot.pose.y)
        candidates.append(
            (
                distance,
                knot.cumulative_translation_arc_m,
                knot.section_index,
                knot.knot_index,
                cos(knot.pose.yaw),
                sin(knot.pose.yaw),
                knot.pose.x,
                knot.pose.y,
                abs(_angle_delta(robot_pose.yaw, knot.pose.yaw)),
            )
        )
    if not candidates:
        return ReferenceCursorProjection(0.0, 0.0, 0, 0, True, "no_translational_edge")
    minimum_distance = min(candidate[0] for candidate in candidates)
    geometrically_tied = tuple(
        candidate
        for candidate in candidates
        if abs(candidate[0] - minimum_distance) <= R4_PROJECTION_TIE_TOLERANCE_M
    )
    tied = geometrically_tied
    # 동일 위치 회전 중에는 robot yaw가 translation tangent 사이를 지나간다. 이때
    # heading을 먼저 고르면 이미 통과한 비인접 section으로 cursor가 되돌아갈 수 있다.
    # 같은 위치의 기하 동률은 이전 cursor의 monotonic locality를 먼저 적용하고, 그
    # 범위 안에서만 heading으로 결정한다.
    if cursor_hint_m is not None and len(tied) > 1:
        monotonic_candidates = tuple(
            candidate
            for candidate in tied
            if candidate[1] + R4_MAXIMUM_CURSOR_REGRESSION_M + R4_PROJECTION_TIE_TOLERANCE_M
            >= cursor_hint_m
        )
        if monotonic_candidates:
            tied = monotonic_candidates
        minimum_cursor_distance = min(abs(candidate[1] - cursor_hint_m) for candidate in tied)
        tied = tuple(
            candidate
            for candidate in tied
            if abs(abs(candidate[1] - cursor_hint_m) - minimum_cursor_distance)
            <= R4_PROJECTION_TIE_TOLERANCE_M
        )
    minimum_heading_error = min(candidate[8] for candidate in tied)
    tied = tuple(
        candidate
        for candidate in tied
        if abs(candidate[8] - minimum_heading_error) <= R4_PROJECTION_TIE_TOLERANCE_M
    )
    for index, first in enumerate(tied):
        for second in tied[index + 1 :]:
            dot = first[4] * second[4] + first[5] * second[5]
            same_projection = hypot(first[6] - second[6], first[7] - second[7]) <= (
                R4_PROJECTION_TIE_TOLERANCE_M
            )
            same_arc = abs(first[1] - second[1]) <= R4_PROJECTION_TIE_TOLERANCE_M
            if dot < -R4_PROJECTION_TIE_TOLERANCE_M and not (same_projection and same_arc):
                chosen = max(tied, key=lambda item: (item[1], item[2], item[3]))
                return ReferenceCursorProjection(
                    chosen[1], chosen[0], chosen[2], chosen[3], True, "opposite_tangent_projection"
                )
            nonadjacent_topology = abs(first[2] - second[2]) > 1 or abs(first[3] - second[3]) > 1
            if nonadjacent_topology and not (same_projection and same_arc):
                chosen = max(tied, key=lambda item: (item[1], item[2], item[3]))
                return ReferenceCursorProjection(
                    chosen[1],
                    chosen[0],
                    chosen[2],
                    chosen[3],
                    True,
                    "ambiguous_reference_projection",
                )
    chosen = max(tied, key=lambda item: (item[1], item[2], item[3]))
    return ReferenceCursorProjection(chosen[1], chosen[0], chosen[2], chosen[3], False, None)


def window_is_exact_slice(
    reference: LocalManeuverReference,
    window: LocalReferenceWindow,
) -> bool:
    """window가 full reference의 수정 없는 whole-section slice인지 검사한다."""

    if not isinstance(reference, LocalManeuverReference) or not isinstance(
        window, LocalReferenceWindow
    ):
        return False
    if (
        window.reference_session_id != reference.reference_session_id
        or window.maneuver_revision != reference.maneuver_revision
        or window.path_revision != reference.path_revision
        or window.full_reference_hash != reference.reference_content_hash
        or window.window_content_hash != window.expected_content_hash
        or not 0 <= window.start_knot_index <= window.end_knot_index < len(reference.knots)
        or window.knots != reference.knots[window.start_knot_index : window.end_knot_index + 1]
    ):
        return False
    if not window.sections:
        return False
    first_section = window.sections[0].section_index
    last_section = window.sections[-1].section_index
    if not 0 <= first_section <= last_section < len(reference.sections):
        return False
    if window.sections != reference.sections[first_section : last_section + 1]:
        return False
    if (
        window.start_knot_index != reference.sections[first_section].first_knot_index
        or window.end_knot_index != reference.sections[last_section].last_knot_index
    ):
        return False
    for section in window.sections:
        if section.section_kind is ReferenceSectionKind.ROTATE and (
            section.first_knot_index < window.start_knot_index
            or section.last_knot_index > window.end_knot_index
        ):
            return False
    return True


def _input_failure(
    context: ReferenceBuildContext,
    reference: LocalManeuverReference,
    validation: LocalReferenceValidation,
) -> str | None:
    try:
        if context.context_content_hash != context.expected_content_hash:
            return "build_context_hash_mismatch"
        if reference.reference_content_hash != reference.expected_content_hash:
            return "reference_hash_mismatch"
        if validation.validation_content_hash != validation.expected_content_hash:
            return "validation_hash_mismatch"
    except (AttributeError, TypeError, ValueError):
        return "input_hash_recalculation_failed"
    if not validation.passed:
        return "reference_validation_failed"
    if validation.reference_content_hash != reference.reference_content_hash:
        return "validation_provenance_mismatch"
    if (
        abs(context.simulation_time_s - context.control_tick * R4_WINDOW_CONTROL_PERIOD_S)
        > _TOLERANCE
    ):
        return "control_tick_time_mismatch"
    if (
        reference.mission_id != context.mission_id
        or reference.stop_epoch != context.stop_epoch
        or reference.map_id != context.map_id
        or reference.map_revision != context.map_revision
        or reference.mission_revision != context.mission_revision
        or reference.grid_content_hash != context.grid_content_hash
        or reference.vehicle_profile_hash != context.vehicle_profile_hash
        or reference.allowed_region_hash != context.allowed_region_hash
        or reference.forbidden_region_hash != context.forbidden_region_hash
        or reference.original_reference_hash != context.original_reference_hash
    ):
        return "reference_context_provenance_mismatch"
    validity = reference.validity
    if context.control_tick < validity.valid_from_control_tick or (
        validity.valid_until_control_tick is not None
        and context.control_tick > validity.valid_until_control_tick
    ):
        return "reference_outside_validity_window"
    return None


def _reference_identity(reference: LocalManeuverReference) -> tuple[object, ...]:
    return (
        reference.mission_id,
        reference.stop_epoch,
        reference.maneuver_revision,
        reference.path_revision,
        reference.candidate_id,
        reference.reference_session_id,
        reference.reference_content_hash,
    )


def _window_input_digest(
    context: ReferenceBuildContext,
    reference: LocalManeuverReference,
    validation: LocalReferenceValidation,
) -> str:
    return canonical_content_hash(
        {
            "context_hash": context.context_content_hash,
            "current_robot_pose": context.current_robot_pose,
            "control_tick": context.control_tick,
            "simulation_time_s": context.simulation_time_s,
            "reference_hash": reference.reference_content_hash,
            "validation_hash": validation.validation_content_hash,
        }
    )


def _window_section_range(
    reference: LocalManeuverReference,
    projected_section_index: int,
    cursor_arc_m: float,
) -> tuple[int, int]:
    section_count = len(reference.sections)
    current = min(max(projected_section_index, 0), section_count - 1)
    rear_target = max(0.0, cursor_arc_m - R4_REAR_CONTEXT_ARC_M)
    start = current
    while start > 0:
        previous_end_arc = reference.knots[
            reference.sections[start - 1].last_knot_index
        ].cumulative_translation_arc_m
        if previous_end_arc + _TOLERANCE < rear_target:
            break
        start -= 1
    forward_target = min(
        reference.knots[-1].cumulative_translation_arc_m,
        cursor_arc_m + R4_MINIMUM_FORWARD_WINDOW_ARC_M,
    )
    end = current
    while end + 1 < section_count:
        end_arc = reference.knots[
            reference.sections[end].last_knot_index
        ].cumulative_translation_arc_m
        if end_arc + _TOLERANCE >= forward_target:
            break
        end += 1
    while end + 1 < section_count:
        next_section = reference.sections[end + 1]
        next_end_arc = reference.knots[next_section.last_knot_index].cumulative_translation_arc_m
        if next_end_arc > forward_target + _TOLERANCE:
            break
        end += 1
    return start, end


def _build_window(
    reference: LocalManeuverReference,
    start_section_index: int,
    end_section_index: int,
    *,
    source_control_tick: int,
    subgoal_revision: int,
) -> LocalReferenceWindow:
    sections = reference.sections[start_section_index : end_section_index + 1]
    first_knot = sections[0].first_knot_index
    last_knot = sections[-1].last_knot_index
    knots = reference.knots[first_knot : last_knot + 1]
    terminal = {
        ReferenceKnotRole.REJOIN,
        ReferenceKnotRole.STOP_MARKER,
    } <= set(knots[-1].knot_roles)
    return LocalReferenceWindow(
        schema_version=LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION,
        reference_session_id=reference.reference_session_id,
        maneuver_revision=reference.maneuver_revision,
        path_revision=reference.path_revision,
        subgoal_revision=subgoal_revision,
        full_reference_hash=reference.reference_content_hash,
        source_control_tick=source_control_tick,
        start_knot_index=first_knot,
        end_knot_index=last_knot,
        knots=knots,
        sections=sections,
        terminal_rejoin_included=terminal,
    )


def _failure_update(
    status: WindowUpdateStatus,
    reason_code: str,
    context: ReferenceBuildContext,
    reference: LocalManeuverReference,
    validation: LocalReferenceValidation,
    *,
    raw_cursor_arc_m: float | None = None,
    projection_distance_m: float | None = None,
) -> LocalReferenceWindowUpdate:
    return LocalReferenceWindowUpdate(
        schema_version=LOCAL_REFERENCE_WINDOW_UPDATE_SCHEMA_VERSION,
        manager_version=LOCAL_REFERENCE_WINDOW_MANAGER_VERSION,
        status=status,
        reason_code=reason_code,
        build_context_hash=_safe_hash(context.context_content_hash, "context"),
        reference_content_hash=_safe_hash(reference.reference_content_hash, "reference"),
        validation_content_hash=_safe_hash(
            validation.validation_content_hash,
            "validation",
        ),
        source_control_tick=context.control_tick,
        raw_cursor_arc_m=raw_cursor_arc_m,
        effective_cursor_arc_m=None,
        projection_distance_m=projection_distance_m,
        window=None,
    )


def _safe_hash(value: object, label: str) -> str:
    if isinstance(value, str) and fullmatch(r"[0-9a-f]{64}", value) is not None:
        return value
    return canonical_content_hash({"invalid_hash": label, "value": repr(value)})


def _require_sha256(value: object, field_name: str) -> None:
    if not isinstance(value, str) or fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{field_name} must be a lowercase SHA-256 hex digest")


def _angle_delta(left: float, right: float) -> float:
    return atan2(sin(left - right), cos(left - right))


__all__ = [
    "LOCAL_REFERENCE_WINDOW_MANAGER_VERSION",
    "LOCAL_REFERENCE_WINDOW_UPDATE_SCHEMA_VERSION",
    "LocalReferenceWindowManager",
    "LocalReferenceWindowUpdate",
    "ReferenceCursorProjection",
    "WindowUpdateStatus",
    "project_reference_cursor",
    "window_is_exact_slice",
]
