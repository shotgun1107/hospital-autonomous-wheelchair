#ifndef HOSPITAL_PATH_LAB_DWA_CORE_H
#define HOSPITAL_PATH_LAB_DWA_CORE_H

#include <cstdint>

#if defined(_WIN32)
#define DWA_CORE_EXPORT __declspec(dllexport)
#else
#define DWA_CORE_EXPORT __attribute__((visibility("default")))
#endif

extern "C" {

enum DwaCoreCandidateState : std::int32_t {
  DWA_CORE_NONMOVING = 0,
  DWA_CORE_ACCEPTED = 1,
  DWA_CORE_REJECTED = 2,
};

enum DwaCorePhase : std::int32_t {
  DWA_CORE_PHASE_NONE = 0,
  DWA_CORE_PHASE_ROLLOUT = 1,
  DWA_CORE_PHASE_TERMINAL = 2,
};

enum DwaCoreCause : std::int32_t {
  DWA_CORE_CAUSE_NONE = 0,
  DWA_CORE_CAUSE_STATIC_OCCUPANCY = 1,
  DWA_CORE_CAUSE_STATIC_CLEARANCE = 2,
  DWA_CORE_CAUSE_FORBIDDEN_ZONE = 3,
  DWA_CORE_CAUSE_ACTOR_TUBE = 4,
  DWA_CORE_CAUSE_PREDICTION_INVALID = 5,
  DWA_CORE_CAUSE_TERMINAL_STOPPING = 6,
};

struct DwaCoreInput {
  std::int32_t abi_version;
  std::int32_t width;
  std::int32_t height;
  std::int32_t physical_has_occupancy;
  std::int32_t combined_has_occupancy;
  std::int32_t forbidden_has_occupancy;
  double resolution_m;
  double origin_x_m;
  double origin_y_m;

  double start_x;
  double start_y;
  double start_yaw;
  double scoring_start_x;
  double scoring_start_y;
  double goal_x;
  double goal_y;
  double previous_angular;

  double horizon_s;
  double integration_step_s;
  double linear_deceleration_mps2;
  double angular_deceleration_radps2;
  double half_length_m;
  double half_width_m;
  double minimum_clearance_m;

  std::int32_t linear_count;
  std::int32_t angular_count;
  const double* linear_values;
  const double* angular_values;

  const std::uint8_t* physical_occupancy;
  const std::uint8_t* combined_occupancy;
  const std::uint8_t* configuration_occupancy;
  const std::uint8_t* physical_collision_occupancy;
  const std::uint8_t* combined_collision_occupancy;
  const std::uint8_t* forbidden_occupancy;
  const double* combined_chebyshev_distance_m;

  std::int32_t reference_count;
  const double* reference_xy;

  std::int32_t actor_time_count;
  std::int32_t actor_capacity;
  const std::int32_t* actor_counts;
  const std::uint8_t* actor_time_valid;
  const double* actor_circles;
};

struct DwaCoreCandidateResult {
  std::int32_t sample_index;
  std::int32_t state;
  std::int32_t phase;
  std::int32_t cause;
  std::int32_t underlying_terminal_cause;
  std::int32_t used_certified_actor_dominance;
  double linear;
  double angular;
  double failure_time_s;
  double minimum_static_clearance_m;
  double minimum_actor_clearance_m;
  double minimum_clearance_m;
  double progress;
  double progress_cost;
  double reference_path_cost;
  double heading_cost;
  double clearance_cost;
  double speed_cost;
  double oscillation_cost;
  double score;
  double rank[9];
};

struct DwaCoreSummary {
  std::int32_t sampled_candidates;
  std::int32_t moving_candidates;
  std::int32_t accepted_candidates;
  std::int32_t nonmoving_samples;
  std::int32_t certified_actor_dominated_candidates;
  std::int32_t reference_geometry_candidates;
};

DWA_CORE_EXPORT std::int32_t dwa_core_abi_version();
DWA_CORE_EXPORT std::int32_t dwa_core_input_size();
DWA_CORE_EXPORT std::int32_t dwa_core_candidate_result_size();

DWA_CORE_EXPORT std::int32_t dwa_core_evaluate(
    const DwaCoreInput* input,
    DwaCoreCandidateResult* candidate_results,
    std::int32_t candidate_capacity,
    std::int32_t* ranked_sample_indices,
    std::int32_t ranked_capacity,
    DwaCoreSummary* summary);

}  // extern "C"

#endif
