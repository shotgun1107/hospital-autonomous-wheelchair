# cython: language_level=3
# cython: boundscheck=False
# cython: wraparound=False
# cython: initializedcheck=False

"""Cython-only numeric loops for the simulation DWA controller.

This module deliberately owns only rollout integration and the repeated
coarse collision scan.  Mission authority, candidate costs, tie-breaking,
the shared safety gate, and result construction remain in Python.
"""

from libc.math cimport copysign, cos, fabs, floor, fmod, hypot, signbit, sin
import numpy as np
cimport numpy as cnp
import math as _python_math


cdef double _PI = 3.141592653589793238462643383279502884
cdef double _TWO_PI = 6.283185307179586476925286766559005768


cdef inline double _normalize_angle(double angle) noexcept:
    cdef double normalized = fmod(angle + _PI, _TWO_PI)
    if normalized < 0.0:
        normalized += _TWO_PI
    return normalized - _PI


cdef inline double _toward_zero(double value, double delta) noexcept:
    if value > 0.0:
        return 0.0 if value <= delta else value - delta
    if value < 0.0:
        return 0.0 if -value <= delta else value + delta
    return 0.0


cdef inline Py_ssize_t _python_floor_div_cell(
    double offset,
    double resolution,
) noexcept:
    """Match CPython float ``//`` used by ``GridMap.world_to_cell``."""

    cdef double remainder = fmod(offset, resolution)
    cdef double divided = (offset - remainder) / resolution
    cdef double floored
    if remainder != 0.0:
        if signbit(remainder) != signbit(resolution):
            remainder += resolution
            divided -= 1.0
    else:
        remainder = copysign(0.0, resolution)
    if divided != 0.0:
        floored = floor(divided)
        if divided - floored > 0.5:
            floored += 1.0
    else:
        floored = copysign(0.0, offset / resolution)
    return <Py_ssize_t>floored


cdef double _minimum_static_lower_bound(
    object poses,
    const double[:, ::1] field,
    double resolution_m,
    double origin_x_m,
    double origin_y_m,
    Py_ssize_t width,
    Py_ssize_t height,
    double half_length,
    double half_width,
    double half_diagonal,
) except *:
    cdef double max_x_m = origin_x_m + width * resolution_m
    cdef double max_y_m = origin_y_m + height * resolution_m
    cdef double minimum = 1.0
    cdef double cosine
    cdef double sine
    cdef double extent_x
    cdef double extent_y
    cdef double boundary
    cdef double candidate
    cdef double obstacle_lower_bound
    cdef Py_ssize_t cell_x
    cdef Py_ssize_t cell_y
    cdef object pose

    for pose in poses:
        cell_x = _python_floor_div_cell(pose.x - origin_x_m, resolution_m)
        cell_y = _python_floor_div_cell(pose.y - origin_y_m, resolution_m)
        if cell_x < 0 or cell_x >= width or cell_y < 0 or cell_y >= height:
            return 0.0
        cosine = fabs(_python_math.cos(pose.yaw))
        sine = fabs(_python_math.sin(pose.yaw))
        extent_x = cosine * half_length + sine * half_width
        extent_y = sine * half_length + cosine * half_width
        boundary = pose.x - extent_x - origin_x_m
        candidate = max_x_m - pose.x - extent_x
        if candidate < boundary:
            boundary = candidate
        candidate = pose.y - extent_y - origin_y_m
        if candidate < boundary:
            boundary = candidate
        candidate = max_y_m - pose.y - extent_y
        if candidate < boundary:
            boundary = candidate
        obstacle_lower_bound = field[cell_y, cell_x] - resolution_m - half_diagonal
        if obstacle_lower_bound < 0.0:
            obstacle_lower_bound = 0.0
        if boundary < minimum:
            minimum = boundary
        if obstacle_lower_bound < minimum:
            minimum = obstacle_lower_bound
        if minimum <= 0.0:
            return minimum
    return minimum


def constant_rollout(
    start,
    command,
    double horizon_s,
    double step_s,
    pose_type,
    trajectory_point_type,
):
    """Integrate one constant command while preserving the Python object API."""

    cdef int steps = <int>round(horizon_s / step_s)
    cdef int step
    cdef double x = start.x
    cdef double y = start.y
    cdef double yaw = start.yaw
    cdef double linear = command.linear
    cdef double angular = command.angular
    cdef double delta_x
    cdef double delta_y
    cdef double delta_yaw
    cdef double next_yaw
    cdef double radius
    cdef object pose = start
    cdef list points = [trajectory_point_type(0.0, pose, command)]

    if fabs(angular) <= 1e-12:
        # The frozen oracle records exact float hex values.  CPython's math
        # wrappers are therefore retained at the transcendental boundary even
        # though the surrounding 41-pose loop is compiled.
        delta_x = linear * _python_math.cos(yaw) * step_s
        delta_y = linear * _python_math.sin(yaw) * step_s
        for step in range(1, steps + 1):
            x += delta_x
            y += delta_y
            pose = pose_type(x=x, y=y, yaw=yaw)
            points.append(trajectory_point_type(step * step_s, pose, command))
        return tuple(points)

    delta_yaw = angular * step_s
    radius = linear / angular
    for step in range(1, steps + 1):
        next_yaw = yaw + delta_yaw
        x += radius * (_python_math.sin(next_yaw) - _python_math.sin(yaw))
        y -= radius * (_python_math.cos(next_yaw) - _python_math.cos(yaw))
        yaw = _normalize_angle(next_yaw)
        pose = pose_type(x=x, y=y, yaw=yaw)
        points.append(trajectory_point_type(step * step_s, pose, command))
    return tuple(points)


def terminal_rollout(
    start,
    double linear_deceleration_mps2,
    double angular_deceleration_radps2,
    double step_s,
    pose_type,
    twist_type,
    trajectory_point_type,
):
    """Integrate the frozen limited-deceleration terminal stopping tail."""

    cdef object pose = start.pose
    cdef object twist = start.twist
    cdef double x = pose.x
    cdef double y = pose.y
    cdef double yaw = pose.yaw
    cdef double linear = twist.linear
    cdef double angular = twist.angular
    cdef double next_yaw
    cdef double radius
    cdef double elapsed_s = 0.0
    cdef list points = [trajectory_point_type(0.0, pose, twist)]

    while fabs(linear) > 1e-12 or fabs(angular) > 1e-12:
        if fabs(angular) <= 1e-12:
            x += linear * _python_math.cos(yaw) * step_s
            y += linear * _python_math.sin(yaw) * step_s
        else:
            next_yaw = yaw + angular * step_s
            radius = linear / angular
            x += radius * (_python_math.sin(next_yaw) - _python_math.sin(yaw))
            y -= radius * (_python_math.cos(next_yaw) - _python_math.cos(yaw))
            yaw = _normalize_angle(next_yaw)
        pose = pose_type(x=x, y=y, yaw=yaw)
        linear = _toward_zero(linear, linear_deceleration_mps2 * step_s)
        angular = _toward_zero(angular, angular_deceleration_radps2 * step_s)
        twist = twist_type(linear=linear, angular=angular)
        elapsed_s += step_s
        points.append(trajectory_point_type(elapsed_s, pose, twist))
    return tuple(points)


def certified_actor_dominated_clearance(
    trajectory,
    combined_checker,
    vehicle,
    actor_sampler,
    bint preserve_rejection_detail,
    evaluation_type,
    phase_type,
    cause_type,
    pose_type,
    twist_type,
    trajectory_point_type,
    oriented_circle_distance,
    double angular_deceleration_radps2,
):
    """Compiled form of the proof-safe Actor-dominated coarse collision scan.

    ``None`` means that Python must execute the historical exact-geometry
    fallback.  A returned evaluation has exactly the same meaning as the
    Python implementation and is still rechecked by the shared safety gate.
    """

    cdef double minimum_actor_clearance = float("inf")
    cdef double point_actor_clearance
    cdef double actor_clearance
    cdef double minimum_static_lower_bound
    cdef double half_length = vehicle.collision_length_m / 2.0
    cdef double half_width = vehicle.collision_width_m / 2.0
    cdef double delta_x
    cdef double delta_y
    cdef double cosine
    cdef double sine
    cdef double local_x
    cdef double local_y
    cdef double outside_x
    cdef double outside_y
    cdef double witness_tolerance
    cdef double threshold_guard
    cdef double actor_threshold = vehicle.minimum_clearance_m - 1e-12
    cdef object point
    cdef object circle
    cdef object phase
    cdef object actor_circles
    cdef list minimum_actor_witnesses = []
    cdef list point_actor_circles
    cdef list evaluated_poses = []

    cdef int last_trajectory_index = len(trajectory) - 1
    cdef object terminal = terminal_rollout(
        trajectory[last_trajectory_index],
        vehicle.max_deceleration_mps2,
        angular_deceleration_radps2,
        0.05,
        pose_type,
        twist_type,
        trajectory_point_type,
    )
    cdef tuple terminal_points = tuple(
        trajectory_point_type(
            trajectory[last_trajectory_index].time_s + point.time_s,
            point.pose,
            point.twist,
        )
        for point in terminal[1:]
    )
    cdef tuple phased_points = (
        (phase_type.COARSE_ROLLOUT, trajectory),
        (phase_type.COARSE_TERMINAL, terminal_points),
    )

    configuration_grid = combined_checker.configuration_grid
    cdef const cnp.npy_bool[:, ::1] configuration_occupancy = (
        configuration_grid.occupancy
    )
    cdef double resolution_m = configuration_grid.resolution_m
    cdef double origin_x_m = configuration_grid.origin_x_m
    cdef double origin_y_m = configuration_grid.origin_y_m
    cdef Py_ssize_t width = configuration_grid.width
    cdef Py_ssize_t height = configuration_grid.height
    cdef Py_ssize_t cell_x
    cdef Py_ssize_t cell_y
    cdef const double[:, ::1] static_field = (
        combined_checker._center_chebyshev_distance_field_m
    )
    cdef double half_diagonal = combined_checker._half_diagonal
    for phase, points in phased_points:
        for point in points:
            cell_x = _python_floor_div_cell(
                point.pose.x - origin_x_m,
                resolution_m,
            )
            cell_y = _python_floor_div_cell(
                point.pose.y - origin_y_m,
                resolution_m,
            )
            if (
                cell_x < 0
                or cell_x >= width
                or cell_y < 0
                or cell_y >= height
                or configuration_occupancy[cell_y, cell_x]
            ):
                return None

    for phase, points in phased_points:
        for point in points:
            evaluated_poses.append(point.pose)
            try:
                actor_circles = actor_sampler.sample(point.time_s)
            except ValueError:
                minimum_static_lower_bound = _minimum_static_lower_bound(
                    evaluated_poses,
                    static_field,
                    resolution_m,
                    origin_x_m,
                    origin_y_m,
                    width,
                    height,
                    half_length,
                    half_width,
                    half_diagonal,
                )
                if minimum_static_lower_bound < vehicle.minimum_clearance_m:
                    return None
                if preserve_rejection_detail:
                    return None
                return evaluation_type(
                    None,
                    failure_phase=phase,
                    failure_cause=(
                        cause_type.TERMINAL_STOPPING
                        if phase is phase_type.COARSE_TERMINAL
                        else cause_type.PREDICTION_INVALID
                    ),
                    failure_time_s=point.time_s,
                    minimum_actor_clearance_m=(
                        None
                        if minimum_actor_clearance == float("inf")
                        else minimum_actor_clearance
                    ),
                    underlying_terminal_cause=(
                        cause_type.PREDICTION_INVALID
                        if phase is phase_type.COARSE_TERMINAL
                        else None
                    ),
                    used_certified_actor_dominance=True,
                )

            point_actor_clearance = float("inf")
            point_actor_circles = []
            threshold_guard = 0.0
            for circle in actor_circles:
                delta_x = circle.center.x - point.pose.x
                delta_y = circle.center.y - point.pose.y
                cosine = cos(point.pose.yaw)
                sine = sin(point.pose.yaw)
                local_x = cosine * delta_x + sine * delta_y
                local_y = -sine * delta_x + cosine * delta_y
                outside_x = max(fabs(local_x) - half_length, 0.0)
                outside_y = max(fabs(local_y) - half_width, 0.0)
                actor_clearance = (
                    -circle.radius_m
                    if outside_x == 0.0 and outside_y == 0.0
                    else hypot(outside_x, outside_y) - circle.radius_m
                )
                witness_tolerance = 1e-9 * max(
                    1.0,
                    fabs(point.pose.x),
                    fabs(point.pose.y),
                    fabs(circle.center.x),
                    fabs(circle.center.y),
                    circle.radius_m,
                )
                threshold_guard = max(threshold_guard, witness_tolerance)
                if actor_clearance < point_actor_clearance - witness_tolerance:
                    point_actor_clearance = actor_clearance
                    point_actor_circles = [circle]
                elif actor_clearance <= point_actor_clearance + witness_tolerance:
                    point_actor_circles.append(circle)

                if actor_clearance < minimum_actor_clearance - witness_tolerance:
                    minimum_actor_clearance = actor_clearance
                    minimum_actor_witnesses = [(point.pose, circle)]
                elif actor_clearance <= minimum_actor_clearance + witness_tolerance:
                    minimum_actor_witnesses.append((point.pose, circle))

            if point_actor_circles and (
                fabs(point_actor_clearance - actor_threshold) <= threshold_guard
            ):
                point_actor_clearance = min(
                    oriented_circle_distance(
                        point.pose,
                        circle_center=(circle.center.x, circle.center.y),
                        circle_radius_m=circle.radius_m,
                        profile=vehicle,
                        inputs_validated=True,
                    )
                    for circle in actor_circles
                )
            if point_actor_clearance < vehicle.minimum_clearance_m - 1e-12:
                minimum_static_lower_bound = _minimum_static_lower_bound(
                    evaluated_poses,
                    static_field,
                    resolution_m,
                    origin_x_m,
                    origin_y_m,
                    width,
                    height,
                    half_length,
                    half_width,
                    half_diagonal,
                )
                if minimum_static_lower_bound < vehicle.minimum_clearance_m:
                    return None
                if preserve_rejection_detail:
                    return None
                return evaluation_type(
                    None,
                    failure_phase=phase,
                    failure_cause=(
                        cause_type.TERMINAL_STOPPING
                        if phase is phase_type.COARSE_TERMINAL
                        else cause_type.ACTOR_TUBE
                    ),
                    failure_time_s=point.time_s,
                    minimum_actor_clearance_m=minimum_actor_clearance,
                    underlying_terminal_cause=(
                        cause_type.ACTOR_TUBE
                        if phase is phase_type.COARSE_TERMINAL
                        else None
                    ),
                    used_certified_actor_dominance=True,
                )

    if minimum_actor_clearance == float("inf"):
        return None
    minimum_actor_clearance = float("inf")
    for pose, circle in minimum_actor_witnesses:
        actor_clearance = oriented_circle_distance(
            pose,
            circle_center=(circle.center.x, circle.center.y),
            circle_radius_m=circle.radius_m,
            profile=vehicle,
            inputs_validated=True,
        )
        if actor_clearance < minimum_actor_clearance:
            minimum_actor_clearance = actor_clearance
    minimum_static_lower_bound = _minimum_static_lower_bound(
        evaluated_poses,
        static_field,
        resolution_m,
        origin_x_m,
        origin_y_m,
        width,
        height,
        half_length,
        half_width,
        half_diagonal,
    )
    if minimum_static_lower_bound < minimum_actor_clearance + 1e-12:
        return None
    return evaluation_type(
        minimum_actor_clearance,
        minimum_actor_clearance_m=minimum_actor_clearance,
        used_certified_actor_dominance=True,
    )
