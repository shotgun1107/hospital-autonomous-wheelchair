from __future__ import annotations

from dataclasses import replace
from math import cos, isclose, pi, sin

from hospital_path_lab.contracts import Pose2D
from hospital_path_lab.ultrasonic_observation import (
    PROVISIONAL_HC_SR04_SEVEN_SENSOR_RIG,
    UltrasonicAvailability,
    UltrasonicFrame,
    UltrasonicFrameValidator,
    UltrasonicMotionIntent,
    UltrasonicObstacle,
    UltrasonicSample,
    UltrasonicSampleStatus,
    UltrasonicStopOutcome,
    UltrasonicStopPolicy,
    UltrasonicValidationPolicy,
    UltrasonicValidationReason,
    evaluate_ultrasonic_stop,
    generate_ultrasonic_frame,
    ultrasonic_frame_content_hash,
)

SOURCE_ID = "synthetic_ultrasonic_source_v1"
RIG = PROVISIONAL_HC_SR04_SEVEN_SENSOR_RIG


def _rehash(frame: UltrasonicFrame) -> UltrasonicFrame:
    return replace(frame, content_hash=ultrasonic_frame_content_hash(frame))


def _all_valid_frame(*, sequence: int = 0, scan_started_at_s: float = 1.0) -> UltrasonicFrame:
    samples = tuple(
        UltrasonicSample(
            sensor_id=mount.sensor_id,
            measured_at_s=scan_started_at_s + index * RIG.trigger_spacing_s,
            status=UltrasonicSampleStatus.VALID,
            range_m=1.0,
        )
        for index, mount in enumerate(RIG.mounts)
    )
    frame = UltrasonicFrame(
        schema_version="ultrasonic_observation_v1",
        source_id=SOURCE_ID,
        rig_id=RIG.rig_id,
        rig_revision=RIG.rig_revision,
        sequence=sequence,
        scan_started_at_s=scan_started_at_s,
        delivered_at_s=scan_started_at_s + RIG.scan_duration_s,
        samples=samples,
    )
    return _rehash(frame)


def _validator(*, ttl_s: float = 0.5) -> UltrasonicFrameValidator:
    return UltrasonicFrameValidator(
        expected_source_id=SOURCE_ID,
        rig=RIG,
        policy=UltrasonicValidationPolicy(ttl_s=ttl_s),
    )


def _accepted(frame: UltrasonicFrame, *, ttl_s: float = 0.5):
    return _validator(ttl_s=ttl_s).accept(frame, controller_time_s=frame.delivered_at_s)


def _with_ranges(frame: UltrasonicFrame, **ranges: float | None) -> UltrasonicFrame:
    samples = tuple(
        replace(
            sample,
            status=(
                UltrasonicSampleStatus.VALID
                if ranges.get(sample.sensor_id, sample.range_m) is not None
                else UltrasonicSampleStatus.NO_ECHO
            ),
            range_m=ranges.get(sample.sensor_id, sample.range_m),
        )
        for sample in frame.samples
    )
    return _rehash(replace(frame, samples=samples, content_hash=""))


def test_provisional_hc_sr04_rig_has_seven_directional_mounts_and_scan_skew() -> None:
    assert RIG.sensor_model == "HC-SR04"
    assert RIG.simulation_only is True
    assert tuple(mount.sensor_id for mount in RIG.mounts) == (
        "front_center",
        "front_left",
        "front_right",
        "side_left",
        "side_right",
        "rear_left",
        "rear_right",
    )
    assert isclose(RIG.scan_duration_s, 0.366)
    assert RIG.min_range_m == 0.02
    assert RIG.max_range_m == 4.0


def test_synthetic_cone_frame_is_deterministic_and_contains_no_obstacle_identity() -> None:
    obstacle = UltrasonicObstacle("person-a", 0.70, 0.0, 0.18)
    first = generate_ultrasonic_frame(
        source_id=SOURCE_ID,
        sequence=4,
        scan_started_at_s=2.0,
        robot_pose=Pose2D(0.0, 0.0, 0.0),
        obstacles=(obstacle,),
    )
    second = generate_ultrasonic_frame(
        source_id=SOURCE_ID,
        sequence=4,
        scan_started_at_s=2.0,
        robot_pose=Pose2D(0.0, 0.0, 0.0),
        obstacles=(obstacle,),
    )
    assert first == second
    assert first.content_hash == ultrasonic_frame_content_hash(first)
    front = first.samples[0]
    assert front.status is UltrasonicSampleStatus.VALID
    assert isclose(front.range_m or 0.0, 0.32)
    assert "person-a" not in repr(first)


def test_rotated_robot_rotates_sensor_detection_with_it() -> None:
    mount = RIG.mounts[0]
    sensor_x = 1.0 + cos(pi / 2.0) * mount.x_m - sin(pi / 2.0) * mount.y_m
    sensor_y = 2.0 + sin(pi / 2.0) * mount.x_m + cos(pi / 2.0) * mount.y_m
    obstacle = UltrasonicObstacle("wall", sensor_x, sensor_y + 0.70, 0.18)
    frame = generate_ultrasonic_frame(
        source_id=SOURCE_ID,
        sequence=0,
        scan_started_at_s=0.0,
        robot_pose=Pose2D(1.0, 2.0, pi / 2.0),
        obstacles=(obstacle,),
    )
    assert frame.samples[0].status is UltrasonicSampleStatus.VALID


def test_no_echo_timeout_and_device_error_remain_distinct_from_valid_range() -> None:
    frame = generate_ultrasonic_frame(
        source_id=SOURCE_ID,
        sequence=0,
        scan_started_at_s=0.0,
        robot_pose=Pose2D(0.0, 0.0, 0.0),
        obstacles=(),
        fault_status_by_sensor={
            "front_left": UltrasonicSampleStatus.TIMEOUT,
            "front_right": UltrasonicSampleStatus.DEVICE_ERROR,
        },
    )
    by_id = {sample.sensor_id: sample for sample in frame.samples}
    assert by_id["front_center"].status is UltrasonicSampleStatus.NO_ECHO
    assert by_id["front_left"].status is UltrasonicSampleStatus.TIMEOUT
    assert by_id["front_right"].status is UltrasonicSampleStatus.DEVICE_ERROR
    assert all(sample.range_m is None for sample in frame.samples)


def test_validator_rejects_no_frame_wrong_source_revision_sequence_and_hash() -> None:
    frame = _all_valid_frame()
    validator = _validator()
    no_frame = validator.accept(None, controller_time_s=frame.delivered_at_s)
    assert no_frame.availability is UltrasonicAvailability.NO_FRAME
    assert no_frame.failures == (UltrasonicValidationReason.NO_FRAME,)

    wrong = replace(frame, source_id="other", rig_revision=99, content_hash="bad")
    result = validator.accept(wrong, controller_time_s=frame.delivered_at_s)
    assert result.accepted is False
    assert set(result.failures) >= {
        UltrasonicValidationReason.SOURCE_MISMATCH,
        UltrasonicValidationReason.RIG_REVISION_MISMATCH,
        UltrasonicValidationReason.CONTENT_HASH_MISMATCH,
    }

    accepted = validator.accept(frame, controller_time_s=frame.delivered_at_s)
    assert accepted.accepted is True
    replay = validator.accept(frame, controller_time_s=frame.delivered_at_s)
    assert replay.failures == (UltrasonicValidationReason.SEQUENCE_NOT_INCREASING,)


def test_ttl_boundary_is_fresh_and_value_above_boundary_is_stale() -> None:
    frame = _all_valid_frame(scan_started_at_s=1.0)
    at_boundary = _validator(ttl_s=0.5).accept(frame, controller_time_s=1.5)
    above_boundary = _validator(ttl_s=0.5).accept(frame, controller_time_s=1.500_001)
    assert at_boundary.availability is UltrasonicAvailability.FRESH
    assert above_boundary.availability is UltrasonicAvailability.STALE
    assert above_boundary.failures == (UltrasonicValidationReason.STALE,)


def test_future_delivery_and_sensor_set_tamper_are_invalid() -> None:
    frame = _all_valid_frame()
    future = _validator().accept(frame, controller_time_s=1.30)
    assert UltrasonicValidationReason.DELIVERY_IN_FUTURE in future.failures
    missing = _rehash(replace(frame, samples=frame.samples[:-1], content_hash=""))
    result = _validator().accept(missing, controller_time_s=frame.delivered_at_s)
    assert result.failures == (UltrasonicValidationReason.SENSOR_SET_MISMATCH,)


def test_schema_sample_timing_delivery_timing_and_status_tamper_are_invalid() -> None:
    frame = _all_valid_frame()
    bad_sample = replace(frame.samples[0], measured_at_s=1.01, status="made_up")
    tampered = _rehash(
        replace(
            frame,
            schema_version="unknown",
            delivered_at_s=1.40,
            samples=(bad_sample, *frame.samples[1:]),
            content_hash="",
        )
    )
    result = _validator().accept(tampered, controller_time_s=1.40)
    assert set(result.failures) >= {
        UltrasonicValidationReason.SCHEMA_MISMATCH,
        UltrasonicValidationReason.SAMPLE_TIME_MISMATCH,
        UltrasonicValidationReason.DELIVERY_TIME_MISMATCH,
        UltrasonicValidationReason.INVALID_RANGE_STATUS,
    }


def test_forward_rear_and_rotation_obstacles_stop_in_the_relevant_direction() -> None:
    policy = UltrasonicStopPolicy(stop_distance_m=0.40)
    base = _all_valid_frame()
    cases = (
        (UltrasonicMotionIntent.FORWARD, {"front_center": 0.25}, "front_center"),
        (UltrasonicMotionIntent.REVERSE, {"rear_left": 0.30}, "rear_left"),
        (UltrasonicMotionIntent.ROTATE_LEFT, {"side_left": 0.20}, "side_left"),
        (UltrasonicMotionIntent.ROTATE_RIGHT, {"side_right": 0.20}, "side_right"),
    )
    for intent, ranges, expected_sensor in cases:
        frame = _with_ranges(base, **ranges)
        decision = evaluate_ultrasonic_stop(_accepted(frame), intent=intent, policy=policy)
        assert decision.outcome is UltrasonicStopOutcome.STOP_OBSTACLE
        assert expected_sensor in decision.sensor_ids


def test_no_echo_and_stale_are_conservative_stop_not_clear() -> None:
    policy = UltrasonicStopPolicy(stop_distance_m=0.40)
    no_echo = _with_ranges(_all_valid_frame(), front_left=None)
    uncertain = evaluate_ultrasonic_stop(
        _accepted(no_echo),
        intent=UltrasonicMotionIntent.FORWARD,
        policy=policy,
    )
    stale_validation = _validator(ttl_s=0.1).accept(
        _all_valid_frame(),
        controller_time_s=_all_valid_frame().delivered_at_s,
    )
    stale = evaluate_ultrasonic_stop(
        stale_validation,
        intent=UltrasonicMotionIntent.FORWARD,
        policy=policy,
    )
    assert uncertain.outcome is UltrasonicStopOutcome.STOP_UNCERTAIN
    assert uncertain.sensor_ids == ("front_left",)
    assert stale.outcome is UltrasonicStopOutcome.STOP_UNCERTAIN


def test_all_relevant_finite_ranges_beyond_threshold_are_clear_only_for_that_intent() -> None:
    frame = _with_ranges(_all_valid_frame(), rear_left=None)
    validation = _accepted(frame)
    policy = UltrasonicStopPolicy(stop_distance_m=0.40)
    forward = evaluate_ultrasonic_stop(
        validation,
        intent=UltrasonicMotionIntent.FORWARD,
        policy=policy,
    )
    reverse = evaluate_ultrasonic_stop(
        validation,
        intent=UltrasonicMotionIntent.REVERSE,
        policy=policy,
    )
    assert forward.outcome is UltrasonicStopOutcome.CLEAR
    assert reverse.outcome is UltrasonicStopOutcome.STOP_UNCERTAIN
