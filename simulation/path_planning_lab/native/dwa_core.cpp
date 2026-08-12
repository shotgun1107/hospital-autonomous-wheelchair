#include "dwa_core.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <tuple>
#include <vector>

namespace {

constexpr std::int32_t kAbiVersion = 2;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kEpsilon = 1e-12;
constexpr double kClearanceLimit = 1.0;

struct Pose {
  double x;
  double y;
  double yaw;
  double time_s;
};

struct Point {
  double x;
  double y;
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

Pose integrate(Pose pose, double linear, double angular, double dt_s) {
  if (std::abs(angular) <= kEpsilon) {
    pose.x += linear * std::cos(pose.yaw) * dt_s;
    pose.y += linear * std::sin(pose.yaw) * dt_s;
  } else {
    const double next_yaw = pose.yaw + angular * dt_s;
    const double radius = linear / angular;
    pose.x += radius * (std::sin(next_yaw) - std::sin(pose.yaw));
    pose.y -= radius * (std::cos(next_yaw) - std::cos(pose.yaw));
    pose.yaw = normalize_angle(next_yaw);
  }
  pose.time_s += dt_s;
  return pose;
}

std::vector<Pose> constant_rollout(
    const DwaCoreInput& input, double linear, double angular) {
  const auto steps = static_cast<std::int32_t>(
      std::llround(input.horizon_s / input.integration_step_s));
  std::vector<Pose> poses;
  poses.reserve(static_cast<std::size_t>(steps + 1));
  Pose pose{input.start_x, input.start_y, input.start_yaw, 0.0};
  poses.push_back(pose);
  for (std::int32_t step = 1; step <= steps; ++step) {
    pose = integrate(pose, linear, angular, input.integration_step_s);
    pose.time_s = step * input.integration_step_s;
    poses.push_back(pose);
  }
  return poses;
}

std::vector<Pose> terminal_rollout(
    const DwaCoreInput& input,
    const Pose& start,
    double linear,
    double angular) {
  std::vector<Pose> poses;
  Pose pose = start;
  double elapsed_s = 0.0;
  while (std::abs(linear) > kEpsilon || std::abs(angular) > kEpsilon) {
    pose = integrate(pose, linear, angular, input.integration_step_s);
    elapsed_s += input.integration_step_s;
    pose.time_s = start.time_s + elapsed_s;
    linear = toward_zero(
        linear, input.linear_deceleration_mps2 * input.integration_step_s);
    angular = toward_zero(
        angular, input.angular_deceleration_radps2 * input.integration_step_s);
    poses.push_back(pose);
  }
  return poses;
}

std::int32_t floor_cell(double offset, double resolution) {
  double remainder = std::fmod(offset, resolution);
  double divided = (offset - remainder) / resolution;
  double floored = 0.0;
  if (remainder != 0.0) {
    if (std::signbit(remainder) != std::signbit(resolution)) {
      remainder += resolution;
      divided -= 1.0;
    }
  } else {
    remainder = std::copysign(0.0, resolution);
  }
  if (divided != 0.0) {
    floored = std::floor(divided);
    if (divided - floored > 0.5) {
      floored += 1.0;
    }
  } else {
    floored = std::copysign(0.0, offset / resolution);
  }
  return static_cast<std::int32_t>(floored);
}

std::pair<std::int32_t, std::int32_t> cell_for(
    const DwaCoreInput& input, const Pose& pose) {
  return {
      floor_cell(pose.x - input.origin_x_m, input.resolution_m),
      floor_cell(pose.y - input.origin_y_m, input.resolution_m),
  };
}

bool occupied(
    const DwaCoreInput& input,
    const std::uint8_t* occupancy,
    const Pose& pose) {
  const auto [x, y] = cell_for(input, pose);
  if (x < 0 || y < 0 || x >= input.width || y >= input.height) {
    return true;
  }
  return occupancy[static_cast<std::size_t>(y * input.width + x)] != 0;
}

std::array<Point, 4> footprint(const DwaCoreInput& input, const Pose& pose) {
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const std::array<Point, 4> local{{
      {-input.half_length_m, -input.half_width_m},
      {input.half_length_m, -input.half_width_m},
      {input.half_length_m, input.half_width_m},
      {-input.half_length_m, input.half_width_m},
  }};
  std::array<Point, 4> world{};
  for (std::size_t index = 0; index < local.size(); ++index) {
    world[index] = {
        pose.x + cosine * local[index].x - sine * local[index].y,
        pose.y + sine * local[index].x + cosine * local[index].y,
    };
  }
  return world;
}

double point_segment_distance(const Point& point, const Point& source, const Point& target) {
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
    const DwaCoreInput& input,
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

double boundary_clearance(const DwaCoreInput& input, const Pose& pose) {
  const double cosine = std::abs(std::cos(pose.yaw));
  const double sine = std::abs(std::sin(pose.yaw));
  const double extent_x = cosine * input.half_length_m + sine * input.half_width_m;
  const double extent_y = sine * input.half_length_m + cosine * input.half_width_m;
  const double max_x = input.origin_x_m + input.width * input.resolution_m;
  const double max_y = input.origin_y_m + input.height * input.resolution_m;
  return std::min({
      pose.x - extent_x - input.origin_x_m,
      max_x - pose.x - extent_x,
      pose.y - extent_y - input.origin_y_m,
      max_y - pose.y - extent_y,
  });
}

double occupancy_clearance(
    const DwaCoreInput& input,
    const Pose& pose,
    const std::uint8_t* occupancy_grid,
    bool has_occupancy) {
  double best = std::min(boundary_clearance(input, pose), kClearanceLimit);
  if (best <= 0.0) {
    return 0.0;
  }
  if (!has_occupancy) {
    return best;
  }
  const double half_diagonal = std::hypot(input.half_length_m, input.half_width_m);
  const auto [cell_x, cell_y] = cell_for(input, pose);
  if (cell_x >= 0 && cell_y >= 0 && cell_x < input.width && cell_y < input.height) {
    const double certified_lower_bound = std::max(
        0.0,
        input.combined_chebyshev_distance_m[
            static_cast<std::size_t>(cell_y * input.width + cell_x)] -
            input.resolution_m - half_diagonal);
    // The combined field contains every physical and forbidden cell.  Its
    // lower bound is therefore safe for each subset queried here.  The exact
    // polygon scan remains mandatory whenever the proof cannot reach the
    // current upper bound.
    if (certified_lower_bound >= best) {
      return best;
    }
  }
  const double cell_half_diagonal = input.resolution_m / std::sqrt(2.0);
  const double radius = half_diagonal + kClearanceLimit + cell_half_diagonal;
  const std::int32_t min_x = std::max(
      0, floor_cell(pose.x - radius - input.origin_x_m, input.resolution_m));
  const std::int32_t min_y = std::max(
      0, floor_cell(pose.y - radius - input.origin_y_m, input.resolution_m));
  const std::int32_t max_x = std::min(
      input.width - 1,
      floor_cell(pose.x + radius - input.origin_x_m, input.resolution_m));
  const std::int32_t max_y = std::min(
      input.height - 1,
      floor_cell(pose.y + radius - input.origin_y_m, input.resolution_m));
  for (std::int32_t y = min_y; y <= max_y; ++y) {
    for (std::int32_t x = min_x; x <= max_x; ++x) {
      if (occupancy_grid[static_cast<std::size_t>(y * input.width + x)] == 0) {
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
    const DwaCoreInput& input,
    const Pose& pose,
    std::int32_t time_index) {
  if (time_index < 0 || time_index >= input.actor_time_count ||
      input.actor_time_valid[time_index] == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double best = std::numeric_limits<double>::infinity();
  const std::int32_t count = input.actor_counts[time_index];
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  for (std::int32_t actor = 0; actor < count; ++actor) {
    const std::size_t offset = static_cast<std::size_t>(
        (time_index * input.actor_capacity + actor) * 3);
    const double center_x = input.actor_circles[offset];
    const double center_y = input.actor_circles[offset + 1];
    const double radius = input.actor_circles[offset + 2];
    const double dx = center_x - pose.x;
    const double dy = center_y - pose.y;
    const double local_x = cosine * dx + sine * dy;
    const double local_y = -sine * dx + cosine * dy;
    const double outside_x = std::max(std::abs(local_x) - input.half_length_m, 0.0);
    const double outside_y = std::max(std::abs(local_y) - input.half_width_m, 0.0);
    const double distance =
        outside_x == 0.0 && outside_y == 0.0
            ? -radius
            : std::hypot(outside_x, outside_y) - radius;
    best = std::min(best, distance);
  }
  return best;
}

double certified_static_lower_bound(
    const DwaCoreInput& input, const std::vector<Pose>& poses) {
  const double half_diagonal = std::hypot(input.half_length_m, input.half_width_m);
  double minimum = kClearanceLimit;
  for (const Pose& pose : poses) {
    const auto [x, y] = cell_for(input, pose);
    if (x < 0 || y < 0 || x >= input.width || y >= input.height) {
      return 0.0;
    }
    const double obstacle = std::max(
        0.0,
        input.combined_chebyshev_distance_m[
            static_cast<std::size_t>(y * input.width + x)] -
            input.resolution_m - half_diagonal);
    minimum = std::min({minimum, boundary_clearance(input, pose), obstacle});
    if (minimum <= 0.0) {
      return minimum;
    }
  }
  return minimum;
}

struct Evaluation {
  bool accepted = false;
  std::int32_t phase = DWA_CORE_PHASE_NONE;
  std::int32_t cause = DWA_CORE_CAUSE_NONE;
  std::int32_t underlying = DWA_CORE_CAUSE_NONE;
  bool certified = false;
  double failure_time = std::numeric_limits<double>::quiet_NaN();
  double minimum_static = std::numeric_limits<double>::infinity();
  double minimum_actor = std::numeric_limits<double>::infinity();
  double minimum = std::numeric_limits<double>::infinity();
};

std::int32_t time_index(const DwaCoreInput& input, double time_s) {
  return static_cast<std::int32_t>(std::llround(time_s / input.integration_step_s));
}

std::int32_t configuration_failure(
    const DwaCoreInput& input, const Pose& pose) {
  if (!occupied(input, input.configuration_occupancy, pose)) {
    return DWA_CORE_CAUSE_NONE;
  }
  const bool physical = occupied(input, input.physical_collision_occupancy, pose);
  const bool combined = occupied(input, input.combined_collision_occupancy, pose);
  if (combined && !physical) {
    return DWA_CORE_CAUSE_FORBIDDEN_ZONE;
  }
  if (physical) {
    return DWA_CORE_CAUSE_STATIC_OCCUPANCY;
  }
  return DWA_CORE_CAUSE_STATIC_CLEARANCE;
}

Evaluation exact_evaluation(
    const DwaCoreInput& input,
    const std::vector<Pose>& rollout,
    const std::vector<Pose>& terminal) {
  Evaluation result;
  const std::array<const std::vector<Pose>*, 2> phases{{&rollout, &terminal}};
  for (std::size_t phase_index = 0; phase_index < phases.size(); ++phase_index) {
    const std::int32_t phase =
        phase_index == 0 ? DWA_CORE_PHASE_ROLLOUT : DWA_CORE_PHASE_TERMINAL;
    for (const Pose& pose : *phases[phase_index]) {
      std::int32_t cause = configuration_failure(input, pose);
      if (cause != DWA_CORE_CAUSE_NONE) {
        result.phase = phase;
        result.cause =
            phase == DWA_CORE_PHASE_TERMINAL ? DWA_CORE_CAUSE_TERMINAL_STOPPING : cause;
        result.underlying = phase == DWA_CORE_PHASE_TERMINAL ? cause : DWA_CORE_CAUSE_NONE;
        result.failure_time = pose.time_s;
        return result;
      }

      const double physical = occupancy_clearance(
          input, pose, input.physical_occupancy, input.physical_has_occupancy != 0);
      const double combined = occupancy_clearance(
          input, pose, input.combined_occupancy, input.combined_has_occupancy != 0);
      const double static_clearance = std::min(physical, combined);
      result.minimum_static = std::min(result.minimum_static, static_clearance);
      result.minimum = std::min(result.minimum, static_clearance);
      if (static_clearance < input.minimum_clearance_m - kEpsilon) {
        cause = physical <= kEpsilon ? DWA_CORE_CAUSE_STATIC_OCCUPANCY
                                    : DWA_CORE_CAUSE_STATIC_CLEARANCE;
      } else if (
          input.forbidden_has_occupancy != 0 && input.forbidden_occupancy != nullptr &&
          occupancy_clearance(input, pose, input.forbidden_occupancy, true) <= 0.0) {
        cause = DWA_CORE_CAUSE_FORBIDDEN_ZONE;
      }
      if (cause != DWA_CORE_CAUSE_NONE) {
        result.phase = phase;
        result.cause =
            phase == DWA_CORE_PHASE_TERMINAL ? DWA_CORE_CAUSE_TERMINAL_STOPPING : cause;
        result.underlying = phase == DWA_CORE_PHASE_TERMINAL ? cause : DWA_CORE_CAUSE_NONE;
        result.failure_time = pose.time_s;
        return result;
      }

      const double actor = actor_clearance(input, pose, time_index(input, pose.time_s));
      if (std::isnan(actor)) {
        cause = DWA_CORE_CAUSE_PREDICTION_INVALID;
      } else {
        result.minimum_actor = std::min(result.minimum_actor, actor);
        result.minimum = std::min(result.minimum, actor);
        if (actor < input.minimum_clearance_m - kEpsilon) {
          cause = DWA_CORE_CAUSE_ACTOR_TUBE;
        }
      }
      if (cause != DWA_CORE_CAUSE_NONE) {
        result.phase = phase;
        result.cause =
            phase == DWA_CORE_PHASE_TERMINAL ? DWA_CORE_CAUSE_TERMINAL_STOPPING : cause;
        result.underlying = phase == DWA_CORE_PHASE_TERMINAL ? cause : DWA_CORE_CAUSE_NONE;
        result.failure_time = pose.time_s;
        return result;
      }
    }
  }
  result.accepted = true;
  return result;
}

Evaluation evaluate_candidate(
    const DwaCoreInput& input,
    const std::vector<Pose>& rollout,
    const std::vector<Pose>& terminal) {
  std::vector<Pose> all;
  all.reserve(rollout.size() + terminal.size());
  all.insert(all.end(), rollout.begin(), rollout.end());
  all.insert(all.end(), terminal.begin(), terminal.end());
  for (const Pose& pose : all) {
    if (occupied(input, input.configuration_occupancy, pose)) {
      return exact_evaluation(input, rollout, terminal);
    }
  }

  double minimum_actor = std::numeric_limits<double>::infinity();
  std::int32_t first_actor_phase = DWA_CORE_PHASE_NONE;
  double first_actor_time = std::numeric_limits<double>::quiet_NaN();
  std::int32_t first_invalid_phase = DWA_CORE_PHASE_NONE;
  double first_invalid_time = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t index = 0; index < all.size(); ++index) {
    const Pose& pose = all[index];
    const std::int32_t phase =
        index < rollout.size() ? DWA_CORE_PHASE_ROLLOUT : DWA_CORE_PHASE_TERMINAL;
    const double actor = actor_clearance(input, pose, time_index(input, pose.time_s));
    if (std::isnan(actor)) {
      if (first_invalid_phase == DWA_CORE_PHASE_NONE) {
        first_invalid_phase = phase;
        first_invalid_time = pose.time_s;
      }
      continue;
    }
    minimum_actor = std::min(minimum_actor, actor);
    if (actor < input.minimum_clearance_m - kEpsilon &&
        first_actor_phase == DWA_CORE_PHASE_NONE) {
      first_actor_phase = phase;
      first_actor_time = pose.time_s;
    }
  }

  const double static_lower = certified_static_lower_bound(input, all);
  const bool actor_dominates =
      std::isfinite(minimum_actor) && static_lower >= minimum_actor + kEpsilon;
  const bool invalid_is_safe_from_static =
      first_invalid_phase != DWA_CORE_PHASE_NONE &&
      static_lower >= input.minimum_clearance_m;

  if (invalid_is_safe_from_static) {
    Evaluation result;
    result.certified = true;
    result.phase = first_invalid_phase;
    result.cause = first_invalid_phase == DWA_CORE_PHASE_TERMINAL
                       ? DWA_CORE_CAUSE_TERMINAL_STOPPING
                       : DWA_CORE_CAUSE_PREDICTION_INVALID;
    result.underlying = first_invalid_phase == DWA_CORE_PHASE_TERMINAL
                            ? DWA_CORE_CAUSE_PREDICTION_INVALID
                            : DWA_CORE_CAUSE_NONE;
    result.failure_time = first_invalid_time;
    result.minimum_actor = minimum_actor;
    return result;
  }
  if (first_actor_phase != DWA_CORE_PHASE_NONE &&
      static_lower >= input.minimum_clearance_m) {
    Evaluation result;
    result.certified = true;
    result.phase = first_actor_phase;
    result.cause = first_actor_phase == DWA_CORE_PHASE_TERMINAL
                       ? DWA_CORE_CAUSE_TERMINAL_STOPPING
                       : DWA_CORE_CAUSE_ACTOR_TUBE;
    result.underlying = first_actor_phase == DWA_CORE_PHASE_TERMINAL
                            ? DWA_CORE_CAUSE_ACTOR_TUBE
                            : DWA_CORE_CAUSE_NONE;
    result.failure_time = first_actor_time;
    result.minimum_actor = minimum_actor;
    return result;
  }
  if (actor_dominates) {
    Evaluation result;
    result.accepted = true;
    result.certified = true;
    result.minimum_actor = minimum_actor;
    result.minimum = minimum_actor;
    return result;
  }
  return exact_evaluation(input, rollout, terminal);
}

double point_segment_distance(
    const Pose& point,
    double source_x,
    double source_y,
    double target_x,
    double target_y) {
  return point_segment_distance(
      Point{point.x, point.y}, Point{source_x, source_y}, Point{target_x, target_y});
}

double mean_reference_distance(
    const DwaCoreInput& input, const std::vector<Pose>& rollout) {
  double total = 0.0;
  for (const Pose& pose : rollout) {
    double best = std::numeric_limits<double>::infinity();
    for (std::int32_t index = 0; index + 1 < input.reference_count; ++index) {
      const std::size_t source = static_cast<std::size_t>(index * 2);
      const std::size_t target = static_cast<std::size_t>((index + 1) * 2);
      best = std::min(
          best,
          point_segment_distance(
              pose,
              input.reference_xy[source],
              input.reference_xy[source + 1],
              input.reference_xy[target],
              input.reference_xy[target + 1]));
    }
    total += best;
  }
  return total / rollout.size();
}

double clip(double value, double lower, double upper) {
  return std::min(std::max(value, lower), upper);
}

void fill_costs(
    const DwaCoreInput& input,
    const std::vector<Pose>& rollout,
    DwaCoreCandidateResult& output) {
  const Pose& end = rollout.back();
  const double start_distance =
      std::hypot(input.scoring_start_x - input.goal_x,
                 input.scoring_start_y - input.goal_y);
  const double end_distance = std::hypot(end.x - input.goal_x, end.y - input.goal_y);
  output.progress = start_distance - end_distance;
  output.progress_cost = 1.0 - clip(output.progress / 0.40, 0.0, 1.0);
  output.reference_path_cost = clip(mean_reference_distance(input, rollout) / 0.50, 0.0, 1.0);
  const double desired = std::atan2(input.goal_y - end.y, input.goal_x - end.x);
  output.heading_cost = clip(std::abs(normalize_angle(desired - end.yaw)) / kPi, 0.0, 1.0);
  output.clearance_cost =
      std::isinf(output.minimum_clearance_m)
          ? 0.0
          : 1.0 - clip(
                      (output.minimum_clearance_m - 0.08) / (0.50 - 0.08),
                      0.0,
                      1.0);
  output.speed_cost = clip((0.20 - output.linear) / 0.20, 0.0, 1.0);
  output.oscillation_cost =
      std::abs(input.previous_angular) > 0.05 && std::abs(output.angular) > 0.05 &&
              input.previous_angular * output.angular < 0.0
          ? 1.0
          : 0.0;
  output.score = output.progress_cost + output.reference_path_cost +
                 0.5 * output.heading_cost + 1.5 * output.clearance_cost +
                 0.2 * output.speed_cost + 0.3 * output.oscillation_cost;
  output.rank[0] = output.score;
  output.rank[1] = -output.minimum_clearance_m;
  output.rank[2] = -output.progress;
  output.rank[3] = output.reference_path_cost;
  output.rank[4] = output.heading_cost;
  output.rank[5] = output.oscillation_cost;
  output.rank[6] = std::abs(output.angular);
  output.rank[7] = -output.linear;
  output.rank[8] = output.angular;
}

bool finite_input(const DwaCoreInput& input) {
  const std::array<double, 18> scalars{{
      input.resolution_m,
      input.origin_x_m,
      input.origin_y_m,
      input.start_x,
      input.start_y,
      input.start_yaw,
      input.scoring_start_x,
      input.scoring_start_y,
      input.goal_x,
      input.goal_y,
      input.previous_angular,
      input.horizon_s,
      input.integration_step_s,
      input.linear_deceleration_mps2,
      input.angular_deceleration_radps2,
      input.half_length_m,
      input.half_width_m,
      input.minimum_clearance_m,
  }};
  return std::all_of(scalars.begin(), scalars.end(), [](double value) {
    return std::isfinite(value);
  });
}

}  // namespace

extern "C" {

std::int32_t dwa_core_abi_version() { return kAbiVersion; }

std::int32_t dwa_core_input_size() {
  return static_cast<std::int32_t>(sizeof(DwaCoreInput));
}

std::int32_t dwa_core_candidate_result_size() {
  return static_cast<std::int32_t>(sizeof(DwaCoreCandidateResult));
}

std::int32_t dwa_core_evaluate(
    const DwaCoreInput* input,
    DwaCoreCandidateResult* candidate_results,
    std::int32_t candidate_capacity,
    std::int32_t* ranked_sample_indices,
    std::int32_t ranked_capacity,
    DwaCoreSummary* summary) {
  if (input == nullptr || candidate_results == nullptr || ranked_sample_indices == nullptr ||
      summary == nullptr) {
    return -1;
  }
  if (input->abi_version != kAbiVersion || input->width <= 0 || input->height <= 0 ||
      input->resolution_m <= 0.0 || input->linear_count <= 0 ||
      input->angular_count <= 0 || input->reference_count < 2 ||
      input->actor_time_count <= 0 || input->actor_capacity < 0 ||
      !finite_input(*input)) {
    return -2;
  }
  const std::int32_t sampled = input->linear_count * input->angular_count;
  if (candidate_capacity < sampled || ranked_capacity < sampled ||
      input->linear_values == nullptr || input->angular_values == nullptr ||
      input->physical_occupancy == nullptr || input->combined_occupancy == nullptr ||
      input->configuration_occupancy == nullptr ||
      input->physical_collision_occupancy == nullptr ||
      input->combined_collision_occupancy == nullptr ||
      input->combined_chebyshev_distance_m == nullptr ||
      input->reference_xy == nullptr || input->actor_counts == nullptr ||
      input->actor_time_valid == nullptr ||
      (input->actor_capacity > 0 && input->actor_circles == nullptr)) {
    return -3;
  }

  *summary = {};
  summary->sampled_candidates = sampled;
  std::vector<std::int32_t> accepted;
  accepted.reserve(static_cast<std::size_t>(sampled));
  std::int32_t sample_index = -1;
  for (std::int32_t linear_index = 0; linear_index < input->linear_count; ++linear_index) {
    const double linear = input->linear_values[linear_index];
    for (std::int32_t angular_index = 0; angular_index < input->angular_count; ++angular_index) {
      const double angular = input->angular_values[angular_index];
      ++sample_index;
      DwaCoreCandidateResult& output = candidate_results[sample_index];
      output = {};
      output.sample_index = sample_index;
      output.linear = linear;
      output.angular = angular;
      output.failure_time_s = std::numeric_limits<double>::quiet_NaN();
      output.minimum_static_clearance_m = std::numeric_limits<double>::infinity();
      output.minimum_actor_clearance_m = std::numeric_limits<double>::infinity();
      output.minimum_clearance_m = std::numeric_limits<double>::infinity();
      if (linear <= kEpsilon) {
        output.state = DWA_CORE_NONMOVING;
        ++summary->nonmoving_samples;
        continue;
      }
      ++summary->moving_candidates;
      const std::vector<Pose> rollout = constant_rollout(*input, linear, angular);
      const std::vector<Pose> terminal = terminal_rollout(
          *input, rollout.back(), linear, angular);
      const Evaluation evaluation = evaluate_candidate(*input, rollout, terminal);
      output.used_certified_actor_dominance = evaluation.certified ? 1 : 0;
      if (evaluation.certified) {
        ++summary->certified_actor_dominated_candidates;
      } else {
        ++summary->reference_geometry_candidates;
      }
      output.minimum_static_clearance_m = evaluation.minimum_static;
      output.minimum_actor_clearance_m = evaluation.minimum_actor;
      output.minimum_clearance_m = evaluation.minimum;
      if (!evaluation.accepted) {
        output.state = DWA_CORE_REJECTED;
        output.phase = evaluation.phase;
        output.cause = evaluation.cause;
        output.underlying_terminal_cause = evaluation.underlying;
        output.failure_time_s = evaluation.failure_time;
        continue;
      }
      output.state = DWA_CORE_ACCEPTED;
      fill_costs(*input, rollout, output);
      accepted.push_back(sample_index);
    }
  }

  std::stable_sort(accepted.begin(), accepted.end(), [&](std::int32_t left, std::int32_t right) {
    for (std::size_t index = 0; index < 9; ++index) {
      const double lhs = candidate_results[left].rank[index];
      const double rhs = candidate_results[right].rank[index];
      if (lhs < rhs) {
        return true;
      }
      if (rhs < lhs) {
        return false;
      }
    }
    return false;
  });
  summary->accepted_candidates = static_cast<std::int32_t>(accepted.size());
  std::copy(accepted.begin(), accepted.end(), ranked_sample_indices);
  return 0;
}

}  // extern "C"
