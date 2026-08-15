#ifndef HOSPITAL_PATH_LAB_DWB_SAFETY_CORE_H
#define HOSPITAL_PATH_LAB_DWB_SAFETY_CORE_H

#include <cstdint>

#if defined(_WIN32)
#define DWB_SAFETY_CORE_EXPORT __declspec(dllexport)
#else
#define DWB_SAFETY_CORE_EXPORT __attribute__((visibility("default")))
#endif

extern "C" {

enum DwbSafetyFailure : std::int32_t {
  DWB_SAFETY_SAFE = 0,
  DWB_SAFETY_FORBIDDEN_ZONE = 1,
  DWB_SAFETY_STATIC_CLEARANCE = 2,
  DWB_SAFETY_ACTOR_CLEARANCE = 3,
  DWB_SAFETY_PREDICTION_INVALID = 4,
};

struct DwbSafetyInput {
  std::int32_t abi_version;
  std::int32_t width;
  std::int32_t height;
  std::int32_t physical_has_occupancy;
  std::int32_t combined_has_occupancy;
  std::int32_t forbidden_has_occupancy;
  double resolution_m;
  double origin_x_m;
  double origin_y_m;
  double half_length_m;
  double half_width_m;
  double minimum_clearance_m;
  double linear_deceleration_mps2;
  double angular_deceleration_radps2;
  double sweep_step_s;
  double apply_duration_s;
  double maximum_actor_speed_mps;
  double robot_x;
  double robot_y;
  double robot_yaw;
  double robot_linear;
  double robot_angular;
  std::int32_t candidate_count;
  std::int32_t pose_count;
  double trajectory_step_s;
  const double* commands;
  const double* trajectory_poses;
  const std::uint8_t* physical_occupancy;
  const std::uint8_t* combined_occupancy;
  const std::uint8_t* forbidden_occupancy;
  const double* combined_chebyshev_distance_m;
  std::int32_t actor_time_count;
  std::int32_t actor_capacity;
  const std::int32_t* actor_counts;
  const std::uint8_t* actor_time_valid;
  const double* actor_capsules;
};

struct DwbSafetyCandidateResult {
  std::int32_t failure;
  double failure_time_s;
  double minimum_static_clearance_m;
  double minimum_actor_clearance_m;
};

DWB_SAFETY_CORE_EXPORT std::int32_t dwb_safety_core_abi_version();
DWB_SAFETY_CORE_EXPORT std::int32_t dwb_safety_core_input_size();
DWB_SAFETY_CORE_EXPORT std::int32_t dwb_safety_core_result_size();
DWB_SAFETY_CORE_EXPORT std::int32_t dwb_safety_core_evaluate(
    const DwbSafetyInput* input,
    DwbSafetyCandidateResult* results,
    std::int32_t result_capacity);

}  // extern "C"

#endif
