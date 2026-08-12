"""Equivalence contract for the optional Cython DWA hot loop."""

from __future__ import annotations

from math import cos, pi, sin
from unittest.mock import patch

import pytest

import hospital_path_lab.local_algorithms.dwa as dwa_module
from hospital_path_lab.contracts import Pose2D, TrajectoryPoint, Twist2D
from hospital_path_lab.dwa_hotloop import CYTHON_DWA_HOTLOOP_AVAILABLE
from hospital_path_lab.dynamic_corpus import (
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_runner import _qualification_snapshot_cases
from hospital_path_lab.local_algorithms.dwa import (
    DynamicDwaController,
    _dynamic_constant_rollout,
    dynamic_dwa_controller_semantic_digest,
)

pytestmark = pytest.mark.skipif(
    not CYTHON_DWA_HOTLOOP_AVAILABLE,
    reason="optional Cython DWA extension is not built",
)


def test_cython_rollout_matches_the_frozen_python_oracle_exactly() -> None:
    starts = (
        Pose2D(1.01, 1.0, 0.0),
        Pose2D(-2.0, 3.0, -1.25),
    )
    commands = (
        Twist2D(0.20, 0.0),
        Twist2D(0.195, 0.04),
        Twist2D(0.05, -0.32),
    )

    for start in starts:
        for command in commands:
            assert _dynamic_constant_rollout(
                start,
                command,
                horizon_s=2.0,
                step_s=0.05,
            ) == _python_rollout_oracle(start, command)


def test_cython_and_python_fallback_match_all_qualification_snapshots() -> None:
    corpus = (*generate_dynamic_corpus(), *generate_dynamic_v6_public_corpus())
    cases = tuple(_qualification_snapshot_cases(corpus))

    assert len(cases) == 5
    for _case_id, snapshot, _metadata in cases:
        accelerated = DynamicDwaController()
        accelerated_result = accelerated.step(snapshot)

        with (
            patch.object(dwa_module, "_cython_constant_rollout", return_value=None),
            patch.object(dwa_module, "_cython_terminal_rollout", return_value=None),
            patch.object(
                dwa_module,
                "_cython_certified_actor_clearance",
                return_value=None,
            ),
        ):
            fallback = DynamicDwaController()
            fallback_result = fallback.step(snapshot)

        assert dynamic_dwa_controller_semantic_digest(accelerated_result) == (
            dynamic_dwa_controller_semantic_digest(fallback_result)
        )
        assert accelerated.last_diagnostics is not None
        assert fallback.last_diagnostics is not None
        assert accelerated.last_diagnostics.semantic_digest == (
            fallback.last_diagnostics.semantic_digest
        )


def _python_rollout_oracle(
    start: Pose2D,
    command: Twist2D,
) -> tuple[TrajectoryPoint, ...]:
    pose = start
    points = [TrajectoryPoint(0.0, pose, command)]
    if abs(command.angular) <= 1e-12:
        delta_x = command.linear * cos(pose.yaw) * 0.05
        delta_y = command.linear * sin(pose.yaw) * 0.05
        for step in range(1, 41):
            pose = Pose2D(pose.x + delta_x, pose.y + delta_y, pose.yaw)
            points.append(TrajectoryPoint(step * 0.05, pose, command))
        return tuple(points)

    delta_yaw = command.angular * 0.05
    radius = command.linear / command.angular
    for step in range(1, 41):
        next_yaw = pose.yaw + delta_yaw
        pose = Pose2D(
            pose.x + radius * (sin(next_yaw) - sin(pose.yaw)),
            pose.y - radius * (cos(next_yaw) - cos(pose.yaw)),
            (next_yaw + pi) % (2.0 * pi) - pi,
        )
        points.append(TrajectoryPoint(step * 0.05, pose, command))
    return tuple(points)
