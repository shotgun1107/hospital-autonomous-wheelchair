"""HC-SR04 7개 임시 배치를 위한 simulation-only 거리 관측 하네스.

이 모듈은 실제 센서·펌웨어 적합성을 증명하지 않는다. 합성 원형 장애물로부터
결정론적인 거리 frame을 만들고, 무응답·오래된 값·출처 불일치를 이동 허가로
오인하지 않는 최소 계약만 검증한다.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import StrEnum
from math import asin, atan2, cos, hypot, isfinite, pi, sin

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile

ULTRASONIC_OBSERVATION_VERSION = "ultrasonic_observation_v1"
HC_SR04_MODEL = "HC-SR04"
HC_SR04_MIN_RANGE_M = 0.02
HC_SR04_MAX_RANGE_M = 4.0
HC_SR04_HALF_BEAM_ANGLE_RAD = 7.5 * pi / 180.0
HC_SR04_MIN_TRIGGER_SPACING_S = 0.061
_TIME_TOLERANCE_S = 1e-12


class UltrasonicSampleStatus(StrEnum):
    VALID = "valid"
    NO_ECHO = "no_echo"
    TIMEOUT = "timeout"
    DEVICE_ERROR = "device_error"


class UltrasonicMotionIntent(StrEnum):
    STOPPED = "stopped"
    FORWARD = "forward"
    REVERSE = "reverse"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"


class UltrasonicStopOutcome(StrEnum):
    CLEAR = "clear"
    STOP_OBSTACLE = "stop_obstacle"
    STOP_UNCERTAIN = "stop_uncertain"


class UltrasonicAvailability(StrEnum):
    FRESH = "fresh"
    STALE = "stale"
    INVALID = "invalid"
    NO_FRAME = "no_frame"


class UltrasonicValidationReason(StrEnum):
    NO_FRAME = "no_frame"
    SCHEMA_MISMATCH = "schema_mismatch"
    SOURCE_MISMATCH = "source_mismatch"
    RIG_REVISION_MISMATCH = "rig_revision_mismatch"
    INVALID_SEQUENCE = "invalid_sequence"
    SEQUENCE_NOT_INCREASING = "sequence_not_increasing"
    SENSOR_SET_MISMATCH = "sensor_set_mismatch"
    SAMPLE_TIME_MISMATCH = "sample_time_mismatch"
    SAMPLE_TIME_AFTER_DELIVERY = "sample_time_after_delivery"
    DELIVERY_TIME_MISMATCH = "delivery_time_mismatch"
    DELIVERY_IN_FUTURE = "delivery_in_future"
    NON_FINITE_VALUE = "non_finite_value"
    INVALID_RANGE_STATUS = "invalid_range_status"
    RANGE_OUTSIDE_MODEL = "range_outside_model"
    CONTENT_HASH_MISMATCH = "content_hash_mismatch"
    STALE = "stale"


@dataclass(frozen=True, slots=True)
class UltrasonicSensorMount:
    sensor_id: str
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True, slots=True)
class UltrasonicRigSpec:
    rig_id: str
    rig_revision: int
    sensor_model: str
    simulation_only: bool
    min_range_m: float
    max_range_m: float
    half_beam_angle_rad: float
    trigger_spacing_s: float
    mounts: tuple[UltrasonicSensorMount, ...]

    def __post_init__(self) -> None:
        if not self.rig_id or not self.sensor_model or self.rig_revision < 0:
            raise ValueError("rig identity must be present and revision must not be negative")
        values = (
            self.min_range_m,
            self.max_range_m,
            self.half_beam_angle_rad,
            self.trigger_spacing_s,
        )
        if not all(isfinite(value) for value in values):
            raise ValueError("rig numeric values must be finite")
        if not 0.0 < self.min_range_m < self.max_range_m:
            raise ValueError("rig range bounds are invalid")
        if self.half_beam_angle_rad <= 0.0 or self.trigger_spacing_s <= 0.0:
            raise ValueError("beam angle and trigger spacing must be positive")
        sensor_ids = tuple(mount.sensor_id for mount in self.mounts)
        if len(sensor_ids) != len(set(sensor_ids)) or not sensor_ids:
            raise ValueError("sensor ids must be non-empty and unique")

    @property
    def scan_duration_s(self) -> float:
        return (len(self.mounts) - 1) * self.trigger_spacing_s


def provisional_hc_sr04_seven_sensor_rig(
    vehicle: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
) -> UltrasonicRigSpec:
    """가상 차체 외곽에 배치한 7개 센서 임시 연구값을 반환한다."""

    half_length = vehicle.body_length_m / 2.0
    half_width = vehicle.body_width_m / 2.0
    diagonal = 25.0 * pi / 180.0
    mounts = (
        UltrasonicSensorMount("front_center", half_length, 0.0, 0.0),
        UltrasonicSensorMount("front_left", half_length * 0.9, half_width * 0.75, diagonal),
        UltrasonicSensorMount("front_right", half_length * 0.9, -half_width * 0.75, -diagonal),
        UltrasonicSensorMount("side_left", 0.0, half_width, pi / 2.0),
        UltrasonicSensorMount("side_right", 0.0, -half_width, -pi / 2.0),
        UltrasonicSensorMount("rear_left", -half_length, half_width * 0.625, pi - diagonal),
        UltrasonicSensorMount("rear_right", -half_length, -half_width * 0.625, -pi + diagonal),
    )
    return UltrasonicRigSpec(
        rig_id="virtual_hc_sr04_seven_v1",
        rig_revision=1,
        sensor_model=HC_SR04_MODEL,
        simulation_only=True,
        min_range_m=HC_SR04_MIN_RANGE_M,
        max_range_m=HC_SR04_MAX_RANGE_M,
        half_beam_angle_rad=HC_SR04_HALF_BEAM_ANGLE_RAD,
        trigger_spacing_s=HC_SR04_MIN_TRIGGER_SPACING_S,
        mounts=mounts,
    )


PROVISIONAL_HC_SR04_SEVEN_SENSOR_RIG = provisional_hc_sr04_seven_sensor_rig()


@dataclass(frozen=True, slots=True)
class UltrasonicObstacle:
    obstacle_id: str
    x_m: float
    y_m: float
    radius_m: float

    def __post_init__(self) -> None:
        if not self.obstacle_id or not all(
            isfinite(value) for value in (self.x_m, self.y_m, self.radius_m)
        ):
            raise ValueError("obstacle values must be finite and id must not be empty")
        if self.radius_m <= 0.0:
            raise ValueError("obstacle radius must be positive")


@dataclass(frozen=True, slots=True)
class UltrasonicScanSampleState:
    """한 센서를 trigger한 시점의 simulation-only world 상태."""

    sensor_id: str
    measured_at_s: float
    robot_pose: Pose2D
    obstacles: tuple[UltrasonicObstacle, ...]

    def __post_init__(self) -> None:
        if not self.sensor_id or not isfinite(self.measured_at_s) or self.measured_at_s < 0.0:
            raise ValueError("scan sample identity and time are invalid")
        values = (self.robot_pose.x, self.robot_pose.y, self.robot_pose.yaw)
        if not all(isfinite(value) for value in values):
            raise ValueError("scan sample robot pose must be finite")
        object.__setattr__(self, "obstacles", tuple(self.obstacles))


@dataclass(frozen=True, slots=True)
class UltrasonicSample:
    sensor_id: str
    measured_at_s: float
    status: UltrasonicSampleStatus
    range_m: float | None


@dataclass(frozen=True, slots=True)
class UltrasonicFrame:
    schema_version: str
    source_id: str
    rig_id: str
    rig_revision: int
    sequence: int
    scan_started_at_s: float
    delivered_at_s: float
    samples: tuple[UltrasonicSample, ...]
    content_hash: str = ""


def ultrasonic_frame_content_hash(frame: UltrasonicFrame) -> str:
    return canonical_content_hash(replace(frame, content_hash=""))


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def ultrasonic_sensor_world_pose(
    robot_pose: Pose2D,
    mount: UltrasonicSensorMount,
) -> Pose2D:
    cos_yaw = cos(robot_pose.yaw)
    sin_yaw = sin(robot_pose.yaw)
    return Pose2D(
        x=robot_pose.x + cos_yaw * mount.x_m - sin_yaw * mount.y_m,
        y=robot_pose.y + sin_yaw * mount.x_m + cos_yaw * mount.y_m,
        yaw=_normalize_angle(robot_pose.yaw + mount.yaw_rad),
    )


def simulated_ultrasonic_cone_range(
    sensor_pose: Pose2D,
    obstacle: UltrasonicObstacle,
    rig: UltrasonicRigSpec,
) -> float | None:
    dx = obstacle.x_m - sensor_pose.x
    dy = obstacle.y_m - sensor_pose.y
    center_distance = hypot(dx, dy)
    if center_distance <= obstacle.radius_m:
        return rig.min_range_m
    center_angle = atan2(dy, dx)
    angular_radius = asin(min(1.0, obstacle.radius_m / center_distance))
    if abs(_normalize_angle(center_angle - sensor_pose.yaw)) > (
        rig.half_beam_angle_rad + angular_radius
    ):
        return None
    surface_distance = center_distance - obstacle.radius_m
    if surface_distance > rig.max_range_m:
        return None
    return max(rig.min_range_m, surface_distance)


def generate_ultrasonic_frame(
    *,
    source_id: str,
    sequence: int,
    scan_started_at_s: float,
    robot_pose: Pose2D,
    obstacles: tuple[UltrasonicObstacle, ...],
    rig: UltrasonicRigSpec = PROVISIONAL_HC_SR04_SEVEN_SENSOR_RIG,
    fault_status_by_sensor: dict[str, UltrasonicSampleStatus] | None = None,
) -> UltrasonicFrame:
    """같은 입력에서 같은 7개 거리 frame을 만든다."""

    states = tuple(
        UltrasonicScanSampleState(
            sensor_id=mount.sensor_id,
            measured_at_s=scan_started_at_s + index * rig.trigger_spacing_s,
            robot_pose=robot_pose,
            obstacles=obstacles,
        )
        for index, mount in enumerate(rig.mounts)
    )
    return generate_dynamic_ultrasonic_frame(
        source_id=source_id,
        sequence=sequence,
        scan_started_at_s=scan_started_at_s,
        scan_states=states,
        rig=rig,
        fault_status_by_sensor=fault_status_by_sensor,
    )


def generate_dynamic_ultrasonic_frame(
    *,
    source_id: str,
    sequence: int,
    scan_started_at_s: float,
    scan_states: tuple[UltrasonicScanSampleState, ...],
    rig: UltrasonicRigSpec = PROVISIONAL_HC_SR04_SEVEN_SENSOR_RIG,
    fault_status_by_sensor: dict[str, UltrasonicSampleStatus] | None = None,
) -> UltrasonicFrame:
    """센서별 측정 시점의 움직이는 차체·장애물을 반영한 frame을 만든다."""

    if not source_id or sequence < 0 or not isfinite(scan_started_at_s) or scan_started_at_s < 0:
        raise ValueError("source, sequence and scan start are invalid")
    expected_ids = tuple(mount.sensor_id for mount in rig.mounts)
    if tuple(state.sensor_id for state in scan_states) != expected_ids:
        raise ValueError("scan states must match the rig sensor order")
    expected_times = tuple(
        scan_started_at_s + index * rig.trigger_spacing_s
        for index in range(len(rig.mounts))
    )
    if any(
        abs(state.measured_at_s - expected_time) > _TIME_TOLERANCE_S
        for state, expected_time in zip(scan_states, expected_times, strict=True)
    ):
        raise ValueError("scan state times must match the sequential trigger schedule")
    faults = fault_status_by_sensor or {}
    unknown_fault_ids = set(faults) - {mount.sensor_id for mount in rig.mounts}
    if unknown_fault_ids:
        raise ValueError("fault map contains an unknown sensor id")

    samples: list[UltrasonicSample] = []
    for mount, state in zip(rig.mounts, scan_states, strict=True):
        measured_at_s = state.measured_at_s
        forced_status = faults.get(mount.sensor_id)
        if forced_status is UltrasonicSampleStatus.VALID:
            raise ValueError("fault injection cannot force a value without a range")
        if forced_status is not None:
            samples.append(UltrasonicSample(mount.sensor_id, measured_at_s, forced_status, None))
            continue

        sensor_pose = ultrasonic_sensor_world_pose(state.robot_pose, mount)
        ranges = tuple(
            detected
            for obstacle in state.obstacles
            if (
                detected := simulated_ultrasonic_cone_range(sensor_pose, obstacle, rig)
            )
            is not None
        )
        if ranges:
            samples.append(
                UltrasonicSample(
                    mount.sensor_id,
                    measured_at_s,
                    UltrasonicSampleStatus.VALID,
                    min(ranges),
                )
            )
        else:
            samples.append(
                UltrasonicSample(
                    mount.sensor_id,
                    measured_at_s,
                    UltrasonicSampleStatus.NO_ECHO,
                    None,
                )
            )

    frame = UltrasonicFrame(
        schema_version=ULTRASONIC_OBSERVATION_VERSION,
        source_id=source_id,
        rig_id=rig.rig_id,
        rig_revision=rig.rig_revision,
        sequence=sequence,
        scan_started_at_s=scan_started_at_s,
        delivered_at_s=scan_started_at_s + rig.scan_duration_s,
        samples=tuple(samples),
    )
    return replace(frame, content_hash=ultrasonic_frame_content_hash(frame))


@dataclass(frozen=True, slots=True)
class UltrasonicValidationPolicy:
    ttl_s: float

    def __post_init__(self) -> None:
        if not isfinite(self.ttl_s) or self.ttl_s < 0.0:
            raise ValueError("ttl_s must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class UltrasonicValidationResult:
    accepted: bool
    availability: UltrasonicAvailability
    failures: tuple[UltrasonicValidationReason, ...]
    frame: UltrasonicFrame | None


class UltrasonicFrameValidator:
    def __init__(
        self,
        *,
        expected_source_id: str,
        rig: UltrasonicRigSpec,
        policy: UltrasonicValidationPolicy,
    ) -> None:
        if not expected_source_id:
            raise ValueError("expected_source_id must not be empty")
        self._source_id = expected_source_id
        self._rig = rig
        self._policy = policy
        self._last_sequence: int | None = None

    def accept(
        self,
        frame: UltrasonicFrame | None,
        *,
        controller_time_s: float,
    ) -> UltrasonicValidationResult:
        if frame is None:
            return UltrasonicValidationResult(
                False,
                UltrasonicAvailability.NO_FRAME,
                (UltrasonicValidationReason.NO_FRAME,),
                None,
            )
        failures = self._collect_failures(frame, controller_time_s=controller_time_s)
        if failures:
            availability = (
                UltrasonicAvailability.STALE
                if failures == [UltrasonicValidationReason.STALE]
                else UltrasonicAvailability.INVALID
            )
            return UltrasonicValidationResult(False, availability, tuple(failures), None)
        self._last_sequence = frame.sequence
        return UltrasonicValidationResult(True, UltrasonicAvailability.FRESH, (), frame)

    def _collect_failures(
        self,
        frame: UltrasonicFrame,
        *,
        controller_time_s: float,
    ) -> list[UltrasonicValidationReason]:
        failures: list[UltrasonicValidationReason] = []
        if frame.schema_version != ULTRASONIC_OBSERVATION_VERSION:
            failures.append(UltrasonicValidationReason.SCHEMA_MISMATCH)
        if frame.source_id != self._source_id:
            failures.append(UltrasonicValidationReason.SOURCE_MISMATCH)
        if frame.rig_id != self._rig.rig_id or frame.rig_revision != self._rig.rig_revision:
            failures.append(UltrasonicValidationReason.RIG_REVISION_MISMATCH)
        if frame.sequence < 0:
            failures.append(UltrasonicValidationReason.INVALID_SEQUENCE)
        if self._last_sequence is not None and frame.sequence <= self._last_sequence:
            failures.append(UltrasonicValidationReason.SEQUENCE_NOT_INCREASING)
        expected_ids = tuple(mount.sensor_id for mount in self._rig.mounts)
        if tuple(sample.sensor_id for sample in frame.samples) != expected_ids:
            failures.append(UltrasonicValidationReason.SENSOR_SET_MISMATCH)
        expected_times = tuple(
            frame.scan_started_at_s + index * self._rig.trigger_spacing_s
            for index in range(len(frame.samples))
        )
        if any(
            abs(sample.measured_at_s - expected_time) > _TIME_TOLERANCE_S
            for sample, expected_time in zip(frame.samples, expected_times, strict=True)
        ):
            failures.append(UltrasonicValidationReason.SAMPLE_TIME_MISMATCH)
        expected_delivery_s = frame.scan_started_at_s + self._rig.scan_duration_s
        if abs(frame.delivered_at_s - expected_delivery_s) > _TIME_TOLERANCE_S:
            failures.append(UltrasonicValidationReason.DELIVERY_TIME_MISMATCH)
        numeric_values = (frame.scan_started_at_s, frame.delivered_at_s, controller_time_s)
        numeric_values += tuple(sample.measured_at_s for sample in frame.samples)
        if not all(isfinite(value) and value >= 0.0 for value in numeric_values):
            failures.append(UltrasonicValidationReason.NON_FINITE_VALUE)
        elif any(sample.measured_at_s > frame.delivered_at_s for sample in frame.samples):
            failures.append(UltrasonicValidationReason.SAMPLE_TIME_AFTER_DELIVERY)
        elif frame.delivered_at_s > controller_time_s + _TIME_TOLERANCE_S:
            failures.append(UltrasonicValidationReason.DELIVERY_IN_FUTURE)
        for sample in frame.samples:
            if not isinstance(sample.status, UltrasonicSampleStatus):
                failures.append(UltrasonicValidationReason.INVALID_RANGE_STATUS)
                continue
            if sample.status is UltrasonicSampleStatus.VALID:
                if sample.range_m is None or not isfinite(sample.range_m):
                    failures.append(UltrasonicValidationReason.INVALID_RANGE_STATUS)
                elif not self._rig.min_range_m <= sample.range_m <= self._rig.max_range_m:
                    failures.append(UltrasonicValidationReason.RANGE_OUTSIDE_MODEL)
            elif sample.range_m is not None:
                failures.append(UltrasonicValidationReason.INVALID_RANGE_STATUS)
        if frame.content_hash != ultrasonic_frame_content_hash(frame):
            failures.append(UltrasonicValidationReason.CONTENT_HASH_MISMATCH)
        if not failures and frame.samples:
            oldest_sample_s = min(sample.measured_at_s for sample in frame.samples)
            if controller_time_s - oldest_sample_s > self._policy.ttl_s + _TIME_TOLERANCE_S:
                failures.append(UltrasonicValidationReason.STALE)
        return list(dict.fromkeys(failures))


@dataclass(frozen=True, slots=True)
class UltrasonicStopPolicy:
    stop_distance_m: float

    def __post_init__(self) -> None:
        if not isfinite(self.stop_distance_m) or self.stop_distance_m <= 0.0:
            raise ValueError("stop_distance_m must be finite and positive")


@dataclass(frozen=True, slots=True)
class UltrasonicStopDecision:
    outcome: UltrasonicStopOutcome
    sensor_ids: tuple[str, ...]
    minimum_range_m: float | None
    reason: str


_RELEVANT_SENSORS = {
    UltrasonicMotionIntent.STOPPED: (),
    UltrasonicMotionIntent.FORWARD: ("front_center", "front_left", "front_right"),
    UltrasonicMotionIntent.REVERSE: ("rear_left", "rear_right"),
    UltrasonicMotionIntent.ROTATE_LEFT: (
        "front_center",
        "front_left",
        "side_left",
        "rear_left",
    ),
    UltrasonicMotionIntent.ROTATE_RIGHT: (
        "front_center",
        "front_right",
        "side_right",
        "rear_right",
    ),
}


def evaluate_ultrasonic_stop(
    validation: UltrasonicValidationResult,
    *,
    intent: UltrasonicMotionIntent,
    policy: UltrasonicStopPolicy,
) -> UltrasonicStopDecision:
    """거리 frame만으로 정지 여부를 판정하며 모터 명령은 만들지 않는다."""

    if intent is UltrasonicMotionIntent.STOPPED:
        return UltrasonicStopDecision(UltrasonicStopOutcome.CLEAR, (), None, "already_stopped")
    if not validation.accepted or validation.frame is None:
        return UltrasonicStopDecision(
            UltrasonicStopOutcome.STOP_UNCERTAIN,
            (),
            None,
            f"observation_{validation.availability.value}",
        )
    by_id = {sample.sensor_id: sample for sample in validation.frame.samples}
    relevant = _RELEVANT_SENSORS[intent]
    uncertain = tuple(
        sensor_id
        for sensor_id in relevant
        if by_id[sensor_id].status is not UltrasonicSampleStatus.VALID
    )
    if uncertain:
        return UltrasonicStopDecision(
            UltrasonicStopOutcome.STOP_UNCERTAIN,
            uncertain,
            None,
            "relevant_sensor_has_no_trustworthy_range",
        )
    ranges = tuple((sensor_id, by_id[sensor_id].range_m) for sensor_id in relevant)
    blocked = tuple(
        sensor_id
        for sensor_id, range_m in ranges
        if range_m is not None and range_m <= policy.stop_distance_m
    )
    minimum_range = min(range_m for _, range_m in ranges if range_m is not None)
    if blocked:
        return UltrasonicStopDecision(
            UltrasonicStopOutcome.STOP_OBSTACLE,
            blocked,
            minimum_range,
            "obstacle_inside_simulation_stop_distance",
        )
    return UltrasonicStopDecision(
        UltrasonicStopOutcome.CLEAR,
        relevant,
        minimum_range,
        "all_relevant_ranges_beyond_simulation_stop_distance",
    )
