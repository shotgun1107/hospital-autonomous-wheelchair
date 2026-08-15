from __future__ import annotations

import pytest

from hospital_path_lab.local_algorithms.dwb_reference.contracts import (
    DwbGeneratorConfig,
    DwbGeneratorRequest,
    DwbPose2D,
    DwbTwist2D,
)
from hospital_path_lab.local_algorithms.dwb_reference.cpp_full_core import (
    CPP_DWB_FULL_CORE_AVAILABLE,
    build_native_manhattan_distances,
    generate_dwb_full_batch,
)
from hospital_path_lab.local_algorithms.dwb_reference.critics import (
    DwbCriticGrid,
    build_manhattan_distance_field,
)
from hospital_path_lab.local_algorithms.dwb_reference.persistent_adapter import (
    SectionBoundDwbReferenceTrajectoryGenerator,
)
from hospital_path_lab.local_algorithms.dwb_reference.trajectory_generator import (
    DwbReferenceTrajectoryGenerator,
)
from hospital_path_lab.local_reference_contracts import ReferenceTravelDirection

pytestmark = pytest.mark.skipif(
    not CPP_DWB_FULL_CORE_AVAILABLE,
    reason="optional C++ full DWB core has not been built",
)


def test_cpp_manhattan_field_exactly_matches_python_oracle() -> None:
    grid = DwbCriticGrid(
        width=9,
        height=7,
        resolution_m=0.05,
        blocked_cells=frozenset({(2, 1), (2, 2), (5, 4), (6, 4)}),
    )
    sources = ((0, 0), (8, 6))

    python_field = build_manhattan_distance_field(grid, sources)
    native_distances = build_native_manhattan_distances(grid, sources)

    assert native_distances == python_field.distances


def _assert_generator_parity(generator, request: DwbGeneratorRequest) -> None:
    python_result = generator.generate(request)
    cpp_result = generate_dwb_full_batch(generator, request)

    assert cpp_result is not None
    assert cpp_result.linear_window_mps == python_result.linear_window_mps
    assert cpp_result.angular_window_radps == python_result.angular_window_radps
    assert cpp_result.linear_samples_mps == python_result.linear_samples_mps
    assert cpp_result.angular_samples_radps == python_result.angular_samples_radps
    assert tuple(item.command for item in cpp_result.trajectories) == tuple(
        item.command for item in python_result.trajectories
    )
    assert len(cpp_result.trajectories) == len(python_result.trajectories)
    for cpp_trajectory, python_trajectory in zip(
        cpp_result.trajectories,
        python_result.trajectories,
        strict=True,
    ):
        assert len(cpp_trajectory.poses) == 41
        for cpp_pose, python_pose in zip(
            cpp_trajectory.poses,
            python_trajectory.poses,
            strict=True,
        ):
            assert cpp_pose.x_m == pytest.approx(python_pose.x_m, abs=1e-15)
            assert cpp_pose.y_m == pytest.approx(python_pose.y_m, abs=1e-15)
            assert cpp_pose.yaw_rad == pytest.approx(python_pose.yaw_rad, abs=1e-15)


def test_cpp_generator_preserves_frozen_217_candidate_lattice() -> None:
    generator = DwbReferenceTrajectoryGenerator()
    request = DwbGeneratorRequest(
        pose=DwbPose2D(0.30, 0.70, 0.20),
        current_twist=DwbTwist2D(0.0, 0.0),
    )

    _assert_generator_parity(generator, request)

    result = generate_dwb_full_batch(generator, request)
    assert result is not None
    assert len(result.trajectories) == 217
    assert all(len(item.poses) == 41 for item in result.trajectories)


@pytest.mark.parametrize(
    ("direction", "prefer_forward"),
    (
        (ReferenceTravelDirection.FORWARD, False),
        (ReferenceTravelDirection.FORWARD, True),
        (ReferenceTravelDirection.REVERSE, False),
    ),
)
def test_cpp_generator_preserves_signed_section_and_tie_order(
    direction: ReferenceTravelDirection,
    prefer_forward: bool,
) -> None:
    generator = SectionBoundDwbReferenceTrajectoryGenerator(
        DwbGeneratorConfig(allow_reverse=True)
    )
    generator.set_travel_direction(direction)
    generator.set_prefer_forward_progress_on_exact_ties(prefer_forward)
    request = DwbGeneratorRequest(
        pose=DwbPose2D(0.60, 2.50, 0.0),
        current_twist=DwbTwist2D(0.0, 0.0),
    )

    _assert_generator_parity(generator, request)
