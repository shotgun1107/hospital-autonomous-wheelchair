#include "dwb_safety_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace {

constexpr std::int32_t kAbiVersion = 1;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kGeometryTolerance = 1e-12;
constexpr double kClearanceLimit = 1.0;

struct Point {
  double x;
  double y;
};

struct Pose {
  double x;
  double y;
  double yaw;
};

double normalize_angle(double angle) {
  double normalized = std::fmod(angle + kPi, 2.0 * kPi);
  if (normalized < 0.0) {
    normalized += 2.0 * kPi;
  }
  return normalized - kPi;
}

double toward_zero(double value, double delta) {
  if (value > 0.0) {
    return std::max(0.0, value - delta);
  }
  if (value < 0.0) {
    return std::min(0.0, value + delta);
  }
  return 0.0;
}

Pose integrate(const Pose& source, double linear, double angular, double dt_s) {
  if (std::abs(angular) <= kGeometryTolerance) {
    return {
        source.x + linear * std::cos(source.yaw) * dt_s,
        source.y + linear * std::sin(source.yaw) * dt_s,
        source.yaw,
    };
  }
  const double next_yaw = source.yaw + angular * dt_s;
  const double radius = linear / angular;
  return {
      source.x + radius * (std::sin(next_yaw) - std::sin(source.yaw)),
      source.y - radius * (std::cos(next_yaw) - std::cos(source.yaw)),
      normalize_angle(next_yaw),
  };
}

Pose interpolate(const Pose& source, const Pose& target, double fraction) {
  const double yaw_delta = normalize_angle(target.yaw - source.yaw);
  return {
      source.x + (target.x - source.x) * fraction,
      source.y + (target.y - source.y) * fraction,
      normalize_angle(source.yaw + yaw_delta * fraction),
  };
}

std::int32_t floor_cell(double offset, double resolution) {
  return static_cast<std::int32_t>(std::floor(offset / resolution));
}

std::pair<std::int32_t, std::int32_t> cell_for(
    const DwbSafetyInput& input, const Pose& pose) {
  return {
      floor_cell(pose.x - input.origin_x_m, input.resolution_m),
      floor_cell(pose.y - input.origin_y_m, input.resolution_m),
  };
}

double point_segment_distance(
    const Point& point, const Point& source, const Point& target) {
  const double dx = target.x - source.x;
  const double dy = target.y - source.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared == 0.0) {
    return std::hypot(point.x - source.x, point.y - source.y);
  }
  const double fraction = std::clamp(
      ((point.x - source.x) * dx + (point.y - source.y) * dy) /
          length_squared,
      0.0,
      1.0);
  return std::hypot(
      point.x - (source.x + fraction * dx),
      point.y - (source.y + fraction * dy));
}

double orientation_cross(const Point& source, const Point& target, const Point& point) {
  return (target.x - source.x) * (point.y - source.y) -
         (target.y - source.y) * (point.x - source.x);
}

bool point_on_segment(
    const Point& point,
    const Point& source,
    const Point& target,
    double cross_tolerance,
    double coordinate_tolerance) {
  if (std::abs(orientation_cross(source, target, point)) > cross_tolerance) {
    return false;
  }
  return point.x >= std::min(source.x, target.x) - coordinate_tolerance &&
         point.x <= std::max(source.x, target.x) + coordinate_tolerance &&
         point.y >= std::min(source.y, target.y) - coordinate_tolerance &&
         point.y <= std::max(source.y, target.y) + coordinate_tolerance;
}

bool segments_intersect(
    const Point& first_start,
    const Point& first_end,
    const Point& second_start,
    const Point& second_end) {
  const double scale = std::max({
      1.0,
      std::abs(first_start.x),
      std::abs(first_start.y),
      std::abs(first_end.x),
      std::abs(first_end.y),
      std::abs(second_start.x),
      std::abs(second_start.y),
      std::abs(second_end.x),
      std::abs(second_end.y),
  });
  const double cross_tolerance = 1e-15 * scale * scale;
  const double coordinate_tolerance = 1e-15 * scale;
  const double a = orientation_cross(first_start, first_end, second_start);
  const double b = orientation_cross(first_start, first_end, second_end);
  const double c = orientation_cross(second_start, second_end, first_start);
  const double d = orientation_cross(second_start, second_end, first_end);
  if (((a > cross_tolerance && b < -cross_tolerance) ||
       (a < -cross_tolerance && b > cross_tolerance)) &&
      ((c > cross_tolerance && d < -cross_tolerance) ||
       (c < -cross_tolerance && d > cross_tolerance))) {
    return true;
  }
  return point_on_segment(
             second_start, first_start, first_end, cross_tolerance, coordinate_tolerance) ||
         point_on_segment(
             second_end, first_start, first_end, cross_tolerance, coordinate_tolerance) ||
         point_on_segment(
             first_start, second_start, second_end, cross_tolerance, coordinate_tolerance) ||
         point_on_segment(
             first_end, second_start, second_end, cross_tolerance, coordinate_tolerance);
}

std::array<Point, 4> local_rectangle(const DwbSafetyInput& input) {
  return {{
      {-input.half_length_m, -input.half_width_m},
      {input.half_length_m, -input.half_width_m},
      {input.half_length_m, input.half_width_m},
      {-input.half_length_m, input.half_width_m},
  }};
}

Point to_local(const Pose& pose, const Point& point) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const double dx = point.x - pose.x;
  const double dy = point.y - pose.y;
  return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

bool point_inside_rectangle(
    const DwbSafetyInput& input, const Point& point) {
  return point.x >= -input.half_length_m && point.x <= input.half_length_m &&
         point.y >= -input.half_width_m && point.y <= input.half_width_m;
}

double rectangle_segment_distance(
    const DwbSafetyInput& input,
    const Point& segment_start,
    const Point& segment_end) {
  const auto rectangle = local_rectangle(input);
  if (point_inside_rectangle(input, segment_start) ||
      point_inside_rectangle(input, segment_end)) {
    return 0.0;
  }
  for (std::size_t index = 0; index < rectangle.size(); ++index) {
    if (segments_intersect(
            segment_start,
            segment_end,
            rectangle[index],
            rectangle[(index + 1) % rectangle.size()])) {
      return 0.0;
    }
  }
  double best = std::numeric_limits<double>::infinity();
  for (const Point& corner : rectangle) {
    best = std::min(best, point_segment_distance(corner, segment_start, segment_end));
  }
  for (std::size_t index = 0; index < rectangle.size(); ++index) {
    const Point& edge_start = rectangle[index];
    const Point& edge_end = rectangle[(index + 1) % rectangle.size()];
    best = std::min(best, point_segment_distance(segment_start, edge_start, edge_end));
    best = std::min(best, point_segment_distance(segment_end, edge_start, edge_end));
  }
  return best;
}

std::array<Point, 4> footprint(const DwbSafetyInput& input, const Pose& pose) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const auto local = local_rectangle(input);
  std::array<Point, 4> world{};
  for (std::size_t index = 0; index < local.size(); ++index) {
    world[index] = {
        pose.x + cosine * local[index].x - sine * local[index].y,
        pose.y + sine * local[index].x + cosine * local[index].y,
    };
  }
  return world;
}

template <typename Polygon>
bool polygons_overlap(const Polygon& first, const Polygon& second) {
  const auto separated = [&](const auto& polygon) {
    for (std::size_t index = 0; index < polygon.size(); ++index) {
      const Point& source = polygon[index];
      const Point& target = polygon[(index + 1) % polygon.size()];
      const double axis_x = -(target.y - source.y);
      const double axis_y = target.x - source.x;
      double first_min = std::numeric_limits<double>::infinity();
      double first_max = -std::numeric_limits<double>::infinity();
      double second_min = std::numeric_limits<double>::infinity();
      double second_max = -std::numeric_limits<double>::infinity();
      for (const Point& point : first) {
        const double value = point.x * axis_x + point.y * axis_y;
        first_min = std::min(first_min, value);
        first_max = std::max(first_max, value);
      }
      for (const Point& point : second) {
        const double value = point.x * axis_x + point.y * axis_y;
        second_min = std::min(second_min, value);
        second_max = std::max(second_max, value);
      }
      if (first_max < second_min || second_max < first_min) {
        return true;
      }
    }
    return false;
  };
  return !separated(first) && !separated(second);
}

double footprint_cell_distance(
    const DwbSafetyInput& input,
    const Pose& pose,
    double center_x,
    double center_y) {
  const auto robot = footprint(input, pose);
  const double half_cell = input.resolution_m / 2.0;
  const std::array<Point, 4> cell{{
      {center_x - half_cell, center_y - half_cell},
      {center_x + half_cell, center_y - half_cell},
      {center_x + half_cell, center_y + half_cell},
      {center_x - half_cell, center_y + half_cell},
  }};
  if (polygons_overlap(robot, cell)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  for (const Point& point : robot) {
    for (std::size_t index = 0; index < cell.size(); ++index) {
      best = std::min(
          best,
          point_segment_distance(point, cell[index], cell[(index + 1) % cell.size()]));
    }
  }
  for (const Point& point : cell) {
    for (std::size_t index = 0; index < robot.size(); ++index) {
      best = std::min(
          best,
          point_segment_distance(point, robot[index], robot[(index + 1) % robot.size()]));
    }
  }
  return best;
}

double boundary_clearance(const DwbSafetyInput& input, const Pose& pose) {
  const double cosine = std::abs(std::cos(pose.yaw));
  const double sine = std::abs(std::sin(pose.yaw));
  const double extent_x = cosine * input.half_length_m + sine * input.half_width_m;
  const double extent_y = sine * input.half_length_m + cosine * input.half_width_m;
  const double maximum_x = input.origin_x_m + input.width * input.resolution_m;
  const double maximum_y = input.origin_y_m + input.height * input.resolution_m;
  return std::min({
      pose.x - extent_x - input.origin_x_m,
      maximum_x - pose.x - extent_x,
      pose.y - extent_y - input.origin_y_m,
      maximum_y - pose.y - extent_y,
  });
}

double occupancy_clearance(
    const DwbSafetyInput& input,
    const Pose& pose,
    const std::uint8_t* occupancy,
    bool has_occupancy) {
  double best = std::min(boundary_clearance(input, pose), kClearanceLimit);
  if (best <= 0.0 || !has_occupancy) {
    return std::max(0.0, best);
  }
  const double half_diagonal = std::hypot(input.half_length_m, input.half_width_m);
  const auto [cell_x, cell_y] = cell_for(input, pose);
  if (cell_x >= 0 && cell_y >= 0 && cell_x < input.width && cell_y < input.height) {
    const double certified = std::max(
        0.0,
        input.combined_chebyshev_distance_m[
            static_cast<std::size_t>(cell_y * input.width + cell_x)] -
            input.resolution_m - half_diagonal);
    if (certified >= best) {
      return best;
    }
  }
  const double cell_half_diagonal = input.resolution_m / std::sqrt(2.0);
  const double radius = half_diagonal + kClearanceLimit + cell_half_diagonal;
  const std::int32_t minimum_x = std::max(
      0, floor_cell(pose.x - radius - input.origin_x_m, input.resolution_m));
  const std::int32_t minimum_y = std::max(
      0, floor_cell(pose.y - radius - input.origin_y_m, input.resolution_m));
  const std::int32_t maximum_x = std::min(
      input.width - 1,
      floor_cell(pose.x + radius - input.origin_x_m, input.resolution_m));
  const std::int32_t maximum_y = std::min(
      input.height - 1,
      floor_cell(pose.y + radius - input.origin_y_m, input.resolution_m));
  for (std::int32_t y = minimum_y; y <= maximum_y; ++y) {
    for (std::int32_t x = minimum_x; x <= maximum_x; ++x) {
      if (occupancy[static_cast<std::size_t>(y * input.width + x)] == 0) {
        continue;
      }
      const double center_x = input.origin_x_m + (x + 0.5) * input.resolution_m;
      const double center_y = input.origin_y_m + (y + 0.5) * input.resolution_m;
      const double lower_bound =
          std::hypot(center_x - pose.x, center_y - pose.y) - half_diagonal -
          cell_half_diagonal;
      if (lower_bound >= best) {
        continue;
      }
      best = std::min(best, footprint_cell_distance(input, pose, center_x, center_y));
      if (best <= 0.0) {
        return 0.0;
      }
    }
  }
  return best;
}

double actor_clearance(
    const DwbSafetyInput& input,
    const Pose& pose,
    std::int32_t time_index,
    double radius_expansion_m) {
  if (time_index < 0 || time_index >= input.actor_time_count ||
      input.actor_time_valid[time_index] == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double best = std::numeric_limits<double>::infinity();
  const std::int32_t count = input.actor_counts[time_index];
  for (std::int32_t actor = 0; actor < count; ++actor) {
    const std::size_t offset = static_cast<std::size_t>(
        (time_index * input.actor_capacity + actor) * 5);
    const Point start = to_local(
        pose, {input.actor_capsules[offset], input.actor_capsules[offset + 1]});
    const Point end = to_local(
        pose, {input.actor_capsules[offset + 2], input.actor_capsules[offset + 3]});
    const double radius = input.actor_capsules[offset + 4] + radius_expansion_m;
    best = std::min(best, rectangle_segment_distance(input, start, end) - radius);
  }
  return best;
}

std::int32_t actor_time_index(const DwbSafetyInput& input, double time_s) {
  return static_cast<std::int32_t>(std::llround(time_s / input.sweep_step_s));
}

void merge_pose(
    const DwbSafetyInput& input,
    const Pose& pose,
    double time_s,
    double actor_radius_expansion_m,
    DwbSafetyCandidateResult& result) {
  const double physical = occupancy_clearance(
      input, pose, input.physical_occupancy, input.physical_has_occupancy != 0);
  const double combined = occupancy_clearance(
      input, pose, input.combined_occupancy, input.combined_has_occupancy != 0);
  const double static_clearance = std::min(physical, combined);
  result.minimum_static_clearance_m = std::min(
      result.minimum_static_clearance_m, static_clearance);
  bool forbidden = false;
  if (input.forbidden_has_occupancy != 0 && input.forbidden_occupancy != nullptr) {
    forbidden = occupancy_clearance(
                    input, pose, input.forbidden_occupancy, true) <= 0.0;
  }
  const double actor = actor_clearance(
      input,
      pose,
      actor_time_index(input, time_s),
      actor_radius_expansion_m);
  if (std::isfinite(actor)) {
    result.minimum_actor_clearance_m = std::min(
        result.minimum_actor_clearance_m, actor);
  }
  if (result.failure != DWB_SAFETY_SAFE) {
    return;
  }
  if (forbidden) {
    result.failure = DWB_SAFETY_FORBIDDEN_ZONE;
  } else if (static_clearance < input.minimum_clearance_m - kGeometryTolerance) {
    result.failure = DWB_SAFETY_STATIC_CLEARANCE;
  } else if (std::isnan(actor)) {
    result.failure = DWB_SAFETY_PREDICTION_INVALID;
  } else if (actor < input.minimum_clearance_m - kGeometryTolerance) {
    result.failure = DWB_SAFETY_ACTOR_CLEARANCE;
  }
  if (result.failure != DWB_SAFETY_SAFE) {
    result.failure_time_s = time_s;
  }
}

DwbSafetyCandidateResult empty_result() {
  return {
      DWB_SAFETY_SAFE,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
  };
}

DwbSafetyCandidateResult evaluate_apply(const DwbSafetyInput& input) {
  DwbSafetyCandidateResult result = empty_result();
  Pose pose{input.robot_x, input.robot_y, input.robot_yaw};
  const auto steps = static_cast<std::int32_t>(
      std::llround(input.apply_duration_s / input.sweep_step_s));
  for (std::int32_t step = 0; step <= steps; ++step) {
    const double elapsed_s = step * input.sweep_step_s;
    const double remaining_s = input.apply_duration_s - elapsed_s;
    merge_pose(
        input,
        pose,
        0.0,
        input.maximum_actor_speed_mps * std::max(0.0, remaining_s),
        result);
    if (result.failure != DWB_SAFETY_SAFE) {
      return result;
    }
    if (step < steps) {
      pose = integrate(
          pose, input.robot_linear, input.robot_angular, input.sweep_step_s);
    }
  }
  return result;
}

Pose candidate_pose(
    const DwbSafetyInput& input,
    std::int32_t candidate,
    std::int32_t pose_index) {
  const std::size_t offset = static_cast<std::size_t>(
      (candidate * input.pose_count + pose_index) * 3);
  return {
      input.trajectory_poses[offset],
      input.trajectory_poses[offset + 1],
      input.trajectory_poses[offset + 2],
  };
}

DwbSafetyCandidateResult evaluate_candidate(
    const DwbSafetyInput& input,
    std::int32_t candidate,
    const DwbSafetyCandidateResult& apply) {
  if (apply.failure != DWB_SAFETY_SAFE) {
    return apply;
  }
  DwbSafetyCandidateResult result = apply;
  for (std::int32_t pose_index = 0; pose_index < input.pose_count; ++pose_index) {
    const Pose target = candidate_pose(input, candidate, pose_index);
    if (pose_index == 0) {
      merge_pose(input, target, 0.0, 0.0, result);
      if (result.failure != DWB_SAFETY_SAFE) {
        return result;
      }
      continue;
    }
    const Pose source = candidate_pose(input, candidate, pose_index - 1);
    const auto subdivisions = static_cast<std::int32_t>(
        std::ceil(input.trajectory_step_s / input.sweep_step_s));
    for (std::int32_t step = 1; step <= subdivisions; ++step) {
      const double fraction = static_cast<double>(step) / subdivisions;
      const Pose pose = interpolate(source, target, fraction);
      const double time_s =
          (pose_index - 1) * input.trajectory_step_s +
          fraction * input.trajectory_step_s;
      merge_pose(input, pose, time_s, 0.0, result);
      if (result.failure != DWB_SAFETY_SAFE) {
        return result;
      }
    }
  }

  const std::size_t command_offset = static_cast<std::size_t>(candidate * 2);
  double linear = input.commands[command_offset];
  double angular = input.commands[command_offset + 1];
  Pose pose = candidate_pose(input, candidate, input.pose_count - 1);
  double time_s = (input.pose_count - 1) * input.trajectory_step_s;
  while (std::abs(linear) > kGeometryTolerance ||
         std::abs(angular) > kGeometryTolerance) {
    pose = integrate(pose, linear, angular, input.sweep_step_s);
    time_s += input.sweep_step_s;
    linear = toward_zero(
        linear, input.linear_deceleration_mps2 * input.sweep_step_s);
    angular = toward_zero(
        angular, input.angular_deceleration_radps2 * input.sweep_step_s);
    merge_pose(input, pose, time_s, 0.0, result);
    if (result.failure != DWB_SAFETY_SAFE) {
      return result;
    }
  }
  return result;
}

bool finite_input(const DwbSafetyInput& input) {
  const std::array<double, 20> scalars{{
      input.resolution_m,
      input.origin_x_m,
      input.origin_y_m,
      input.half_length_m,
      input.half_width_m,
      input.minimum_clearance_m,
      input.linear_deceleration_mps2,
      input.angular_deceleration_radps2,
      input.sweep_step_s,
      input.apply_duration_s,
      input.maximum_actor_speed_mps,
      input.robot_x,
      input.robot_y,
      input.robot_yaw,
      input.robot_linear,
      input.robot_angular,
      input.trajectory_step_s,
      static_cast<double>(input.width),
      static_cast<double>(input.height),
      static_cast<double>(input.pose_count),
  }};
  return std::all_of(scalars.begin(), scalars.end(), [](double value) {
    return std::isfinite(value);
  });
}

}  // namespace

extern "C" {

std::int32_t dwb_safety_core_abi_version() { return kAbiVersion; }

std::int32_t dwb_safety_core_input_size() {
  return static_cast<std::int32_t>(sizeof(DwbSafetyInput));
}

std::int32_t dwb_safety_core_result_size() {
  return static_cast<std::int32_t>(sizeof(DwbSafetyCandidateResult));
}

std::int32_t dwb_safety_core_evaluate(
    const DwbSafetyInput* input,
    DwbSafetyCandidateResult* results,
    std::int32_t result_capacity) {
  if (input == nullptr || results == nullptr || input->abi_version != kAbiVersion) {
    return 1;
  }
  if (!finite_input(*input) || input->width <= 0 || input->height <= 0 ||
      input->candidate_count <= 0 || input->pose_count <= 0 ||
      input->actor_time_count <= 0 || input->actor_capacity < 0 ||
      result_capacity < input->candidate_count || input->commands == nullptr ||
      input->trajectory_poses == nullptr || input->physical_occupancy == nullptr ||
      input->combined_occupancy == nullptr || input->forbidden_occupancy == nullptr ||
      input->combined_chebyshev_distance_m == nullptr ||
      input->actor_counts == nullptr || input->actor_time_valid == nullptr ||
      (input->actor_capacity > 0 && input->actor_capsules == nullptr)) {
    return 2;
  }
  if (input->resolution_m <= 0.0 || input->half_length_m <= 0.0 ||
      input->half_width_m <= 0.0 || input->minimum_clearance_m < 0.0 ||
      input->linear_deceleration_mps2 <= 0.0 ||
      input->angular_deceleration_radps2 <= 0.0 ||
      input->sweep_step_s <= 0.0 || input->trajectory_step_s <= 0.0 ||
      input->apply_duration_s < 0.0 || input->maximum_actor_speed_mps < 0.0) {
    return 3;
  }
  const DwbSafetyCandidateResult apply = evaluate_apply(*input);
  for (std::int32_t candidate = 0; candidate < input->candidate_count; ++candidate) {
    results[candidate] = evaluate_candidate(*input, candidate, apply);
  }
  return 0;
}

}  // extern "C"
