#ifndef HOSPITAL_PATH_LAB_DWB_FULL_CORE_H
#define HOSPITAL_PATH_LAB_DWB_FULL_CORE_H

#include <cstdint>

#if defined(_WIN32)
#define DWB_FULL_CORE_EXPORT __declspec(dllexport)
#else
#define DWB_FULL_CORE_EXPORT __attribute__((visibility("default")))
#endif

extern "C" {

enum DwbFullCandidateStatus : std::int32_t {
  DWB_FULL_LEGAL = 0,
  DWB_FULL_ILLEGAL = 1,
  DWB_FULL_SHORT_CIRCUITED = 2,
};

enum DwbFullFailure : std::int32_t {
  DWB_FULL_NO_FAILURE = 0,
  DWB_FULL_SAFETY_FORBIDDEN = 1,
  DWB_FULL_SAFETY_STATIC = 2,
  DWB_FULL_SAFETY_ACTOR = 3,
  DWB_FULL_SAFETY_PREDICTION = 4,
  DWB_FULL_NOT_SLOWING = 100,
  DWB_FULL_TRANSLATION_DURING_ROTATION = 101,
  DWB_FULL_OSCILLATION = 200,
  DWB_FULL_OFF_GRID = 300,
  DWB_FULL_BLOCKED_CELL = 301,
  DWB_FULL_UNREACHABLE_CELL = 302,
  DWB_FULL_INVALID_SCORE = 900,
};

struct DwbFullGeneratorInput {
  std::int32_t abi_version;
  double control_period_s;
  double rollout_duration_s;
  double integration_step_s;
  double maximum_forward_speed_mps;
  double maximum_reverse_speed_mps;
  double linear_acceleration_mps2;
  double linear_deceleration_mps2;
  double maximum_angular_speed_radps;
  double angular_acceleration_radps2;
  double angular_deceleration_radps2;
  std::int32_t linear_sample_count;
  std::int32_t angular_sample_count;
  std::int32_t allow_reverse;
  std::int32_t travel_direction;
  std::int32_t prefer_forward_progress;
  double pose_x;
  double pose_y;
  double pose_yaw;
  double current_linear;
  double current_angular;
};

struct DwbFullGeneratorOutput {
  double linear_minimum;
  double linear_maximum;
  double angular_minimum;
  double angular_maximum;
  std::int32_t linear_count;
  std::int32_t angular_count;
  std::int32_t candidate_count;
  std::int32_t pose_count;
};

struct DwbFullEvaluationInput {
  std::int32_t abi_version;
  std::int32_t candidate_count;
  std::int32_t pose_count;
  double trajectory_step_s;
  const double* commands;
  const double* poses;
  const std::int32_t* safety_failures;
  const double* critic_scales;
  std::int32_t short_circuit;

  std::int32_t width;
  std::int32_t height;
  double resolution_m;
  double origin_x_m;
  double origin_y_m;
  const std::uint8_t* blocked_cells;
  const std::int32_t* goal_align_field;
  const std::int32_t* path_align_field;
  const std::int32_t* path_dist_field;
  const std::int32_t* goal_dist_field;
  double forward_point_distance_m;
  double goal_align_projection_sign;
  double path_align_projection_sign;
  std::int32_t goal_align_disabled;
  std::int32_t path_align_disabled;

  std::int32_t rotate_in_window;
  std::int32_t rotate_rotating;
  double rotate_goal_yaw;
  double rotate_current_speed_sq;
  double rotate_slowing_factor;
  double rotate_lookahead_time_s;
  std::int32_t oscillation_linear_positive_only;
  std::int32_t oscillation_linear_negative_only;
  std::int32_t oscillation_angular_positive_only;
  std::int32_t oscillation_angular_negative_only;
};

struct DwbFullEvaluationOutput {
  std::int32_t selected_candidate_index;
  double selected_total_score;
  std::int32_t legal_count;
};

DWB_FULL_CORE_EXPORT std::int32_t dwb_full_core_abi_version();
DWB_FULL_CORE_EXPORT std::int32_t dwb_full_core_generator_input_size();
DWB_FULL_CORE_EXPORT std::int32_t dwb_full_core_generator_output_size();
DWB_FULL_CORE_EXPORT std::int32_t dwb_full_core_evaluation_input_size();
DWB_FULL_CORE_EXPORT std::int32_t dwb_full_core_evaluation_output_size();

DWB_FULL_CORE_EXPORT std::int32_t dwb_full_core_generate(
    const DwbFullGeneratorInput* input,
    DwbFullGeneratorOutput* output,
    double* linear_samples,
    std::int32_t linear_capacity,
    double* angular_samples,
    std::int32_t angular_capacity,
    double* commands,
    std::int32_t command_capacity,
    double* poses,
    std::int32_t pose_capacity);

DWB_FULL_CORE_EXPORT std::int32_t dwb_full_core_evaluate(
    const DwbFullEvaluationInput* input,
    DwbFullEvaluationOutput* output,
    std::int32_t* candidate_status,
    double* accumulated_scores,
    double* critic_raw_scores,
    std::int32_t* failure_codes,
    std::int32_t result_capacity);

DWB_FULL_CORE_EXPORT std::int32_t dwb_full_core_manhattan_field(
    std::int32_t width,
    std::int32_t height,
    const std::uint8_t* blocked_cells,
    const std::int32_t* source_xy,
    std::int32_t source_count,
    std::int32_t* distances,
    std::int32_t distance_capacity);

}  // extern "C"

#endif
