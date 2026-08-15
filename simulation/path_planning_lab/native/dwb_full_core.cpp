#include "dwb_full_core.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace {

constexpr std::int32_t kAbiVersion = 1;
constexpr std::int32_t kCriticCount = 7;
constexpr double kTolerance = 1e-12;
constexpr double kPi = 3.141592653589793238462643383279502884;

bool finite(const double value) { return std::isfinite(value); }

double python_mod(const double value, const double modulus) {
  double result = std::fmod(value, modulus);
  if (result < 0.0) {
    result += modulus;
  }
  return result;
}

double shortest_angle(const double from, const double to) {
  return python_mod(to - from + kPi, 2.0 * kPi) - kPi;
}

std::vector<double> sample_axis(
    const double minimum, const double maximum, const std::int32_t count) {
  std::vector<double> values;
  if (minimum == maximum) {
    values.push_back(std::abs(minimum) <= kTolerance ? 0.0 : minimum);
    return values;
  }
  const double step = (maximum - minimum) / static_cast<double>(count - 1);
  values.reserve(static_cast<std::size_t>(count + 1));
  for (std::int32_t index = 0; index < count; ++index) {
    values.push_back(minimum + static_cast<double>(index) * step);
  }
  values.back() = maximum;
  bool has_zero = false;
  for (const double value : values) {
    has_zero = has_zero || std::abs(value) <= kTolerance;
  }
  if (minimum <= 0.0 && maximum >= 0.0 && !has_zero) {
    values.push_back(0.0);
    std::sort(values.begin(), values.end());
  }
  std::vector<double> normalized;
  normalized.reserve(values.size());
  for (double value : values) {
    if (std::abs(value) <= kTolerance) {
      value = 0.0;
    }
    if (normalized.empty() || std::abs(value - normalized.back()) > kTolerance) {
      normalized.push_back(value);
    }
  }
  return normalized;
}

bool valid_generator(const DwbFullGeneratorInput& input) {
  const double values[] = {
      input.control_period_s,
      input.rollout_duration_s,
      input.integration_step_s,
      input.maximum_forward_speed_mps,
      input.maximum_reverse_speed_mps,
      input.linear_acceleration_mps2,
      input.linear_deceleration_mps2,
      input.maximum_angular_speed_radps,
      input.angular_acceleration_radps2,
      input.angular_deceleration_radps2,
      input.pose_x,
      input.pose_y,
      input.pose_yaw,
      input.current_linear,
      input.current_angular,
  };
  for (const double value : values) {
    if (!finite(value)) {
      return false;
    }
  }
  return input.control_period_s > 0.0 && input.rollout_duration_s > 0.0 &&
         input.integration_step_s > 0.0 && input.maximum_forward_speed_mps > 0.0 &&
         input.maximum_reverse_speed_mps >= 0.0 && input.linear_sample_count >= 2 &&
         input.angular_sample_count >= 2 && input.travel_direction >= 0 &&
         input.travel_direction <= 2;
}

std::int32_t score_field(
    const DwbFullEvaluationInput& input,
    const std::int32_t* field,
    const double x,
    const double y,
    const bool stop_on_failure,
    double* score) {
  const auto cell_x = static_cast<std::int32_t>(
      std::floor((x - input.origin_x_m) / input.resolution_m));
  const auto cell_y = static_cast<std::int32_t>(
      std::floor((y - input.origin_y_m) / input.resolution_m));
  if (cell_x < 0 || cell_y < 0 || cell_x >= input.width || cell_y >= input.height) {
    return DWB_FULL_OFF_GRID;
  }
  const std::int32_t index = cell_y * input.width + cell_x;
  const double obstacle_score = static_cast<double>(input.width * input.height);
  if (input.blocked_cells[index] != 0U) {
    if (stop_on_failure) {
      return DWB_FULL_BLOCKED_CELL;
    }
    *score = obstacle_score;
    return DWB_FULL_NO_FAILURE;
  }
  if (field[index] < 0) {
    if (stop_on_failure) {
      return DWB_FULL_UNREACHABLE_CELL;
    }
    *score = obstacle_score + 1.0;
    return DWB_FULL_NO_FAILURE;
  }
  *score = static_cast<double>(field[index]);
  return DWB_FULL_NO_FAILURE;
}

void trajectory_pose_at(
    const DwbFullEvaluationInput& input,
    const std::int32_t candidate,
    const double time_s,
    double* x,
    double* y,
    double* yaw) {
  const auto base = static_cast<std::size_t>(candidate) *
                    static_cast<std::size_t>(input.pose_count) * 3U;
  if (time_s <= 0.0) {
    *x = input.poses[base];
    *y = input.poses[base + 1U];
    *yaw = input.poses[base + 2U];
    return;
  }
  const double maximum_time =
      static_cast<double>(input.pose_count - 1) * input.trajectory_step_s;
  if (time_s >= maximum_time) {
    const auto offset = base + static_cast<std::size_t>(input.pose_count - 1) * 3U;
    *x = input.poses[offset];
    *y = input.poses[offset + 1U];
    *yaw = input.poses[offset + 2U];
    return;
  }
  const double position = time_s / input.trajectory_step_s;
  const auto lower = static_cast<std::int32_t>(std::floor(position));
  const double ratio = position - static_cast<double>(lower);
  const auto lower_offset = base + static_cast<std::size_t>(lower) * 3U;
  const auto upper_offset = lower_offset + 3U;
  *x = input.poses[lower_offset] +
       ratio * (input.poses[upper_offset] - input.poses[lower_offset]);
  *y = input.poses[lower_offset + 1U] +
       ratio * (input.poses[upper_offset + 1U] - input.poses[lower_offset + 1U]);
  *yaw = input.poses[lower_offset + 2U] +
         ratio * shortest_angle(input.poses[lower_offset + 2U],
                                input.poses[upper_offset + 2U]);
}

std::int32_t raw_score(
    const DwbFullEvaluationInput& input,
    const std::int32_t candidate,
    const std::int32_t critic,
    double* score) {
  const double linear = input.commands[candidate * 2];
  const double angular = input.commands[candidate * 2 + 1];
  const auto final_offset =
      (static_cast<std::size_t>(candidate) * static_cast<std::size_t>(input.pose_count) +
       static_cast<std::size_t>(input.pose_count - 1)) *
      3U;
  const double final_x = input.poses[final_offset];
  const double final_y = input.poses[final_offset + 1U];
  const double final_yaw = input.poses[final_offset + 2U];

  if (critic == 0) {
    const std::int32_t failure = input.safety_failures[candidate];
    *score = 0.0;
    return failure;
  }
  if (critic == 1) {
    if (input.rotate_in_window == 0) {
      *score = 0.0;
      return DWB_FULL_NO_FAILURE;
    }
    const double speed_sq = linear * linear;
    if (input.rotate_rotating == 0 && speed_sq >= input.rotate_current_speed_sq) {
      return DWB_FULL_NOT_SLOWING;
    }
    if (input.rotate_rotating != 0 && std::abs(linear) > kTolerance) {
      return DWB_FULL_TRANSLATION_DURING_ROTATION;
    }
    double rotation_x = final_x;
    double rotation_y = final_y;
    double rotation_yaw = final_yaw;
    if (input.rotate_lookahead_time_s >= 0.0) {
      trajectory_pose_at(input, candidate, input.rotate_lookahead_time_s,
                         &rotation_x, &rotation_y, &rotation_yaw);
    }
    const double rotation_score =
        std::abs(shortest_angle(rotation_yaw, input.rotate_goal_yaw));
    *score = input.rotate_rotating != 0
                 ? rotation_score
                 : speed_sq * input.rotate_slowing_factor + rotation_score;
    return DWB_FULL_NO_FAILURE;
  }
  if (critic == 2) {
    const bool linear_bad =
        (input.oscillation_linear_positive_only != 0 && linear < 0.0) ||
        (input.oscillation_linear_negative_only != 0 && linear > 0.0);
    const bool angular_bad =
        (input.oscillation_angular_positive_only != 0 && angular < 0.0) ||
        (input.oscillation_angular_negative_only != 0 && angular > 0.0);
    if (linear_bad || angular_bad) {
      return DWB_FULL_OSCILLATION;
    }
    *score = 0.0;
    return DWB_FULL_NO_FAILURE;
  }

  if (critic == 3 || critic == 4) {
    const bool disabled = critic == 3 ? input.goal_align_disabled != 0
                                      : input.path_align_disabled != 0;
    if (disabled) {
      *score = 0.0;
      return DWB_FULL_NO_FAILURE;
    }
    const double sign = critic == 3 ? input.goal_align_projection_sign
                                    : input.path_align_projection_sign;
    const double x = final_x + sign * input.forward_point_distance_m * std::cos(final_yaw);
    const double y = final_y + sign * input.forward_point_distance_m * std::sin(final_yaw);
    const std::int32_t* field =
        critic == 3 ? input.goal_align_field : input.path_align_field;
    return score_field(input, field, x, y, false, score);
  }
  if (critic == 5) {
    return score_field(input, input.path_dist_field, final_x, final_y, true, score);
  }
  return score_field(input, input.goal_dist_field, final_x, final_y, true, score);
}

}  // namespace

extern "C" {

std::int32_t dwb_full_core_abi_version() { return kAbiVersion; }
std::int32_t dwb_full_core_generator_input_size() {
  return static_cast<std::int32_t>(sizeof(DwbFullGeneratorInput));
}
std::int32_t dwb_full_core_generator_output_size() {
  return static_cast<std::int32_t>(sizeof(DwbFullGeneratorOutput));
}
std::int32_t dwb_full_core_evaluation_input_size() {
  return static_cast<std::int32_t>(sizeof(DwbFullEvaluationInput));
}
std::int32_t dwb_full_core_evaluation_output_size() {
  return static_cast<std::int32_t>(sizeof(DwbFullEvaluationOutput));
}

std::int32_t dwb_full_core_generate(
    const DwbFullGeneratorInput* input,
    DwbFullGeneratorOutput* output,
    double* linear_samples,
    const std::int32_t linear_capacity,
    double* angular_samples,
    const std::int32_t angular_capacity,
    double* commands,
    const std::int32_t command_capacity,
    double* poses,
    const std::int32_t pose_capacity) {
  if (input == nullptr || output == nullptr || linear_samples == nullptr ||
      angular_samples == nullptr || commands == nullptr || poses == nullptr ||
      input->abi_version != kAbiVersion || !valid_generator(*input)) {
    return -1;
  }
  const double minimum_linear = input->allow_reverse != 0
                                    ? -input->maximum_reverse_speed_mps
                                    : 0.0;
  double linear_min = std::max(
      minimum_linear,
      input->current_linear - input->linear_deceleration_mps2 * input->control_period_s);
  double linear_max = std::min(
      input->maximum_forward_speed_mps,
      input->current_linear + input->linear_acceleration_mps2 * input->control_period_s);
  if (input->travel_direction == 1) {
    linear_min = std::max(0.0, linear_min);
    linear_max = std::max(0.0, linear_max);
  } else if (input->travel_direction == 2) {
    linear_min = std::min(0.0, linear_min);
    linear_max = std::min(0.0, linear_max);
  }
  const double angular_min = std::max(
      -input->maximum_angular_speed_radps,
      input->current_angular -
          input->angular_deceleration_radps2 * input->control_period_s);
  const double angular_max = std::min(
      input->maximum_angular_speed_radps,
      input->current_angular +
          input->angular_acceleration_radps2 * input->control_period_s);
  if (linear_min > linear_max + kTolerance || angular_min > angular_max) {
    return -2;
  }
  std::vector<double> linear =
      sample_axis(linear_min, linear_max, input->linear_sample_count);
  const std::vector<double> angular =
      sample_axis(angular_min, angular_max, input->angular_sample_count);
  if (input->prefer_forward_progress != 0) {
    if (input->travel_direction != 1) {
      return -3;
    }
    std::reverse(linear.begin(), linear.end());
  }
  const auto pose_count = static_cast<std::int32_t>(
                              std::llround(input->rollout_duration_s /
                                           input->integration_step_s)) +
                          1;
  const auto candidate_count = static_cast<std::int32_t>(
      linear.size() * angular.size());
  if (linear_capacity < static_cast<std::int32_t>(linear.size()) ||
      angular_capacity < static_cast<std::int32_t>(angular.size()) ||
      command_capacity < candidate_count * 2 ||
      pose_capacity < candidate_count * pose_count * 3) {
    return -4;
  }
  std::copy(linear.begin(), linear.end(), linear_samples);
  std::copy(angular.begin(), angular.end(), angular_samples);
  std::int32_t candidate = 0;
  for (const double linear_value : linear) {
    for (const double angular_value : angular) {
      commands[candidate * 2] = linear_value;
      commands[candidate * 2 + 1] = angular_value;
      double x = input->pose_x;
      double y = input->pose_y;
      double yaw = input->pose_yaw;
      for (std::int32_t pose_index = 0; pose_index < pose_count; ++pose_index) {
        const auto offset =
            (static_cast<std::size_t>(candidate) * static_cast<std::size_t>(pose_count) +
             static_cast<std::size_t>(pose_index)) *
            3U;
        poses[offset] = x;
        poses[offset + 1U] = y;
        poses[offset + 2U] = yaw;
        if (pose_index + 1 < pose_count) {
          x += linear_value * std::cos(yaw) * input->integration_step_s;
          y += linear_value * std::sin(yaw) * input->integration_step_s;
          yaw += angular_value * input->integration_step_s;
        }
      }
      ++candidate;
    }
  }
  *output = DwbFullGeneratorOutput{
      linear_min,
      linear_max,
      angular_min,
      angular_max,
      static_cast<std::int32_t>(linear.size()),
      static_cast<std::int32_t>(angular.size()),
      candidate_count,
      pose_count,
  };
  return 0;
}

std::int32_t dwb_full_core_evaluate(
    const DwbFullEvaluationInput* input,
    DwbFullEvaluationOutput* output,
    std::int32_t* candidate_status,
    double* accumulated_scores,
    double* critic_raw_scores,
    std::int32_t* failure_codes,
    const std::int32_t result_capacity) {
  if (input == nullptr || output == nullptr || candidate_status == nullptr ||
      accumulated_scores == nullptr || critic_raw_scores == nullptr ||
      failure_codes == nullptr || input->abi_version != kAbiVersion ||
      input->candidate_count <= 0 || input->pose_count <= 0 ||
      input->width <= 0 || input->height <= 0 || input->resolution_m <= 0.0 ||
      result_capacity < input->candidate_count) {
    return -1;
  }
  const double nan = std::numeric_limits<double>::quiet_NaN();
  double best_score = 0.0;
  std::int32_t best_index = -1;
  std::int32_t legal_count = 0;
  for (std::int32_t candidate = 0; candidate < input->candidate_count; ++candidate) {
    candidate_status[candidate] = DWB_FULL_LEGAL;
    accumulated_scores[candidate] = 0.0;
    failure_codes[candidate] = DWB_FULL_NO_FAILURE;
    for (std::int32_t critic = 0; critic < kCriticCount; ++critic) {
      critic_raw_scores[candidate * kCriticCount + critic] = nan;
    }
    for (std::int32_t critic = 0; critic < kCriticCount; ++critic) {
      const double scale = input->critic_scales[critic];
      if (scale == 0.0) {
        continue;
      }
      double raw = 0.0;
      const std::int32_t failure = raw_score(*input, candidate, critic, &raw);
      if (failure != DWB_FULL_NO_FAILURE) {
        candidate_status[candidate] = DWB_FULL_ILLEGAL;
        failure_codes[candidate] = failure;
        break;
      }
      const double weighted = raw * scale;
      if (!finite(raw) || raw < 0.0 || !finite(weighted) || weighted < 0.0) {
        candidate_status[candidate] = DWB_FULL_ILLEGAL;
        failure_codes[candidate] = DWB_FULL_INVALID_SCORE;
        break;
      }
      critic_raw_scores[candidate * kCriticCount + critic] = raw;
      accumulated_scores[candidate] += weighted;
      if (input->short_circuit != 0 && best_index >= 0 &&
          accumulated_scores[candidate] > best_score) {
        candidate_status[candidate] = DWB_FULL_SHORT_CIRCUITED;
        break;
      }
    }
    if (candidate_status[candidate] != DWB_FULL_LEGAL) {
      continue;
    }
    ++legal_count;
    if (best_index < 0 || accumulated_scores[candidate] < best_score) {
      best_index = candidate;
      best_score = accumulated_scores[candidate];
    }
  }
  *output = DwbFullEvaluationOutput{best_index, best_score, legal_count};
  return best_index < 0 ? 1 : 0;
}

std::int32_t dwb_full_core_manhattan_field(
    const std::int32_t width,
    const std::int32_t height,
    const std::uint8_t* blocked_cells,
    const std::int32_t* source_xy,
    const std::int32_t source_count,
    std::int32_t* distances,
    const std::int32_t distance_capacity) {
  if (width <= 0 || height <= 0 || blocked_cells == nullptr ||
      source_xy == nullptr || source_count <= 0 || distances == nullptr ||
      distance_capacity < width * height) {
    return -1;
  }
  const std::int32_t cell_count = width * height;
  std::fill(distances, distances + cell_count, -1);
  std::vector<std::int32_t> queue;
  queue.reserve(static_cast<std::size_t>(cell_count));
  for (std::int32_t source = 0; source < source_count; ++source) {
    const std::int32_t x = source_xy[source * 2];
    const std::int32_t y = source_xy[source * 2 + 1];
    if (x < 0 || y < 0 || x >= width || y >= height) {
      return -2;
    }
    const std::int32_t index = y * width + x;
    if (blocked_cells[index] != 0U) {
      return -3;
    }
    if (distances[index] < 0) {
      distances[index] = 0;
      queue.push_back(index);
    }
  }
  constexpr std::int32_t dx[] = {-1, 0, 0, 1};
  constexpr std::int32_t dy[] = {0, -1, 1, 0};
  std::size_t head = 0;
  while (head < queue.size()) {
    const std::int32_t index = queue[head++];
    const std::int32_t x = index % width;
    const std::int32_t y = index / width;
    for (std::int32_t direction = 0; direction < 4; ++direction) {
      const std::int32_t adjacent_x = x + dx[direction];
      const std::int32_t adjacent_y = y + dy[direction];
      if (adjacent_x < 0 || adjacent_y < 0 || adjacent_x >= width ||
          adjacent_y >= height) {
        continue;
      }
      const std::int32_t adjacent = adjacent_y * width + adjacent_x;
      if (blocked_cells[adjacent] != 0U || distances[adjacent] >= 0) {
        continue;
      }
      distances[adjacent] = distances[index] + 1;
      queue.push_back(adjacent);
    }
  }
  return 0;
}

}  // extern "C"
