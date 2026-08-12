from __future__ import annotations

from dataclasses import FrozenInstanceError
from math import isfinite

import pytest

from hospital_path_lab.local_algorithms.dwb_reference import (
    NAV2_NAVIGATION_COMMIT,
    ROS1_NAVIGATION_COMMIT,
    DwbGeneratorConfig,
    DwbGeneratorRequest,
    DwbPose2D,
    DwbReferenceTrajectoryGenerator,
    DwbTwist2D,
    sample_velocity_axis,
)


def _request(*, linear: float = 0.0, angular: float = 0.0) -> DwbGeneratorRequest:
    return DwbGeneratorRequest(
        pose=DwbPose2D(1.0, 2.0, 0.0),
        current_twist=DwbTwist2D(linear, angular),
    )


def test_upstream_source_commits_are_frozen() -> None:
    assert ROS1_NAVIGATION_COMMIT == "f44bb1fc2810399165115cc98b530fe4b9397c18"
    assert NAV2_NAVIGATION_COMMIT == "1e8afb17e2e09df443b1870ce0f4ecdee32207fd"


def test_contracts_are_frozen() -> None:
    pose = DwbPose2D(0.0, 0.0, 0.0)
    with pytest.raises(FrozenInstanceError):
        pose.x_m = 1.0  # type: ignore[misc]


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_pose_and_twist_reject_non_finite_values(value: float) -> None:
    with pytest.raises(ValueError, match="finite"):
        DwbPose2D(value, 0.0, 0.0)
    with pytest.raises(ValueError, match="finite"):
        DwbTwist2D(0.0, value)


def test_config_rejects_invalid_rollout_partition() -> None:
    with pytest.raises(ValueError, match="divisible"):
        DwbGeneratorConfig(rollout_duration_s=2.0, integration_step_s=0.03)


def test_dynamic_window_uses_next_control_period_and_asymmetric_limits() -> None:
    generator = DwbReferenceTrajectoryGenerator()

    linear_window, angular_window = generator.dynamic_window(DwbTwist2D(0.10, 0.20))

    assert linear_window == pytest.approx((0.075, 0.1125))
    assert angular_window == pytest.approx((0.12, 0.28))


def test_dynamic_window_clamps_to_vehicle_limits_and_disables_reverse() -> None:
    generator = DwbReferenceTrajectoryGenerator()

    stopped_linear, _ = generator.dynamic_window(DwbTwist2D(0.0, 0.0))
    maximum_linear, maximum_angular = generator.dynamic_window(DwbTwist2D(0.20, 0.80))

    assert stopped_linear == pytest.approx((0.0, 0.0125))
    assert maximum_linear == pytest.approx((0.175, 0.20))
    assert maximum_angular == pytest.approx((0.72, 0.80))


def test_velocity_axis_inserts_zero_without_duplicate() -> None:
    samples = sample_velocity_axis(-0.9, 0.8, 4)

    assert len(samples) == 5
    assert samples == tuple(sorted(samples))
    assert samples.count(0.0) == 1
    assert samples[0] == -0.9
    assert samples[-1] == 0.8


def test_velocity_axis_keeps_nominal_count_when_zero_is_uniform_sample() -> None:
    samples = sample_velocity_axis(-0.8, 0.8, 5)

    assert len(samples) == 5
    assert samples.count(0.0) == 1


def test_default_batch_is_vx_times_w_in_stable_order() -> None:
    generator = DwbReferenceTrajectoryGenerator()

    first = generator.generate(_request())
    second = generator.generate(_request())

    assert first == second
    assert len(first.linear_samples_mps) == 7
    assert len(first.angular_samples_radps) == 31
    assert first.candidate_count == 217
    assert tuple(trajectory.command for trajectory in first.trajectories[:31]) == tuple(
        DwbTwist2D(first.linear_samples_mps[0], angular)
        for angular in first.angular_samples_radps
    )


def test_constant_twist_rollout_has_initial_pose_plus_forty_intervals() -> None:
    generator = DwbReferenceTrajectoryGenerator()

    trajectory = generator.rollout(
        DwbPose2D(1.0, 2.0, 0.0),
        DwbTwist2D(0.20, 0.0),
    )

    assert len(trajectory.poses) == 41
    assert trajectory.poses[0] == DwbPose2D(1.0, 2.0, 0.0)
    assert trajectory.poses[-1].x_m == pytest.approx(1.40)
    assert trajectory.poses[-1].y_m == pytest.approx(2.0)
    assert trajectory.poses[-1].yaw_rad == pytest.approx(0.0)


def test_turning_rollout_keeps_command_constant() -> None:
    generator = DwbReferenceTrajectoryGenerator()
    command = DwbTwist2D(0.10, 0.20)

    trajectory = generator.rollout(DwbPose2D(0.0, 0.0, 0.0), command)

    assert trajectory.command == command
    assert trajectory.poses[-1].yaw_rad == pytest.approx(0.40)
    assert trajectory.poses[-1].x_m > 0.19
    assert trajectory.poses[-1].y_m > 0.0


def test_reverse_samples_are_absent_by_default_and_available_when_enabled() -> None:
    default_generator = DwbReferenceTrajectoryGenerator()
    reverse_generator = DwbReferenceTrajectoryGenerator(
        DwbGeneratorConfig(allow_reverse=True)
    )

    default_result = default_generator.generate(_request())
    reverse_window, _ = reverse_generator.dynamic_window(DwbTwist2D(-0.05, 0.0))

    assert all(value >= 0.0 for value in default_result.linear_samples_mps)
    assert reverse_window == pytest.approx((-0.075, -0.0375))


def test_request_velocity_outside_configuration_is_rejected() -> None:
    generator = DwbReferenceTrajectoryGenerator()

    with pytest.raises(ValueError, match="outside"):
        generator.generate(_request(linear=-0.01))
    with pytest.raises(ValueError, match="outside"):
        generator.generate(_request(angular=0.81))


def test_all_generated_values_and_poses_are_finite() -> None:
    result = DwbReferenceTrajectoryGenerator().generate(_request(linear=0.10, angular=0.20))

    values = [
        coordinate
        for trajectory in result.trajectories
        for pose in trajectory.poses
        for coordinate in (pose.x_m, pose.y_m, pose.yaw_rad)
    ]
    assert all(isfinite(value) for value in values)
