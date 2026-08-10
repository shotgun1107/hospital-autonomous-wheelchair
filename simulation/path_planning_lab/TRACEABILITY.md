# 경로 알고리즘 실험실 요구사항·시험 추적성

이 문서는 `simulation/path_planning_lab`의 연구용 요구사항을 현재 구현 파일과 pytest
식별자에 연결한다. 제품 요구사항, 팀 합의 또는 알고리즘 채택 문서가 아니다.

## 범위와 상태 해석

- 합성 지도와 완전한 ground truth를 사용하는 Python 논리·기하·폐루프 추종 시험이다.
- `G1~G5`는 팀에서 확인되지 않았고 7단계 결정 게이트는 시작되지 않았다.
- 모든 지도·차체·속도·해상도·deadline 값은 `simulation_only` 가설이다.
- 실제 센서, ROS 2 scheduling, 통신 지연, 구동계, 축소 실물과 실제 사람 탑승은
  증거 범위에 없다.
- `pytest ID`는 요구사항에 연결된 시험 위치다. 이 문서만으로 최신 실행 통과를
  주장하지 않으며, 새 output 디렉터리에서 회귀시험과 통합 실행을 확인해야 한다.

| 상태 | 의미 |
|---|---|
| 연결됨 | 구현과 직접 회귀시험이 존재한다. |
| 제한적 연결 | 구현과 시험은 있으나 증거 범위가 계획의 일부에 한정된다. |
| 미측정 | 현재 runner가 명시적으로 측정하지 않는다. |
| 후속 후보 | registry에만 표시하며 현재 구현·성능·채택을 주장하지 않는다. |

증거 수준은 `L1=단위·속성 pytest`, `L2=20-case 통합 실행 산출물`로 구분한다. 어느
수준도 물리 안전성이나 의료기기 인증의 증거가 아니다.

## 현재 통합 실행 계약

| 항목 | 현재 값 | 근거 |
|---|---|---|
| 평가 corpus | 기본 20개: `golden 12 / development 4 / hidden 2 / regressions 2`; 이전 후보 `N`개 재투입 시 `regressions 2+N`, 총 `20+N` | [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_experiment_runner.py::test_full_twenty_case_experiment_writes_reproducible_evidence`, `::test_previous_hidden_failure_can_feed_the_next_regression_corpus` |
| 공개 동결 | 기본 공개 18개와 알고리즘 소스를 hidden 선택 전에 SHA-256으로 동결; 재투입 후보 `N`개도 공개 corpus에 넣어 `18+N`개 동결 | `tests/test_experiment_runner.py::test_corpus_is_frozen_before_separate_hidden_selection`, `::test_previous_hidden_failure_can_feed_the_next_regression_corpus` |
| 전역 실행 | 모든 case의 step 0부터 마지막 사건 step, 무효 입력 포함 | `tests/test_experiment_runner.py::test_episode_steps_follow_last_event_instead_of_fixed_six` |
| local 실행 | 입력이 유효하고 전역 reference path가 있는 모든 step에서 Grid A*·DWA | `tests/test_experiment_runner.py::test_full_twenty_case_experiment_writes_reproducible_evidence` |
| 추종 실행 | Grid A*가 `FOUND`인 모든 step에서 Pure Pursuit·RPP | 같은 통합 시험 |
| 종단 pipeline | 모든 추종 호환 step에서 `A* → Grid A* → Pure Pursuit/RPP`를 별도 결과·Pareto로 집계하며 global/local validation·oracle과 follower 초기 명령 validation을 모두 성공 조건에 포함 | `tests/test_experiment_runner.py::test_pipeline_success_requires_every_component_validation_and_oracle`와 통합 시험의 `pipeline_results`, `pareto.pipelines` 검증 |
| stale 증거 | 이전 snapshot 결과를 최신 metadata에 대입해 global·local·follower 모두 실행 불가인지 확인 | 같은 통합 시험의 `stale_result_evidence` 역할 집합 검증 |
| 상태 유지 DWA | 초기 path deviation `0.11m`, 차단 3 + 해제 60 = synthetic 63-step에서 정지·교착·회복·재합류를 측정 | `tests/test_simulation.py::test_dynamic_local_evidence_records_stop_deadlock_recovery_and_rejoin`와 통합 runner 시험 |
| 추종 시간 예산 | 두 후보 공통 `max(30s, path_length / 0.20m/s × 2.5 + 10s)` | `tests/test_experiment_runner.py::test_follower_time_budget_is_shared_and_scales_with_path_length` |
| 계산시간 deadline | 전역 5초, local 30초, 추종 60초; 반환한 호출의 측정시간을 사후 비교해 초과를 hard failure로 기록 | `tests/test_experiment_runner.py::test_tiny_simulation_deadline_is_classified_as_hard_failure` |
| deadline 범위 | 제품 요구가 아닌 `simulation_only` 실행 이상 감지값; hang 강제 종료 watchdog은 미구현 | [experiment_runner.py](src/hospital_path_lab/experiment_runner.py) |
| peak memory | 알고리즘×지도 계열의 첫 표본만 `tracemalloc`; 미측정 표본은 0 | 같은 통합 시험의 `peak_memory_policy` 검증 |
| hidden 그림 | 평가한 모든 hidden step마다 graph 1장과 grid/path/trajectory 1장 | 같은 통합 시험과 [experiment_visualization.py](src/hospital_path_lab/experiment_visualization.py) |

생성기 자체의 10개 batch 분배는 `golden 2 / development 4 / hidden 2 /
regressions 2`다. 기본 통합 실행의 20개 분배와 혼동하지 않는다. 통합 실행은 생성
batch의 golden 2를 쓰는 대신 수동으로 고정한 golden 12를 사용하고, 별도 hidden seed
batch의 hidden 2를 동결 뒤 선택한다. `--regression-input-dir`을 사용한 회차는 검증·중복
제거된 과거 후보가 추가되므로 20개보다 많아질 수 있다.

## DYN-STAGE1 — 동적 원형 Actor 시뮬레이션 기반

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| `DYN-ARCH-001` | controller-facing frame은 Actor ground truth와 seed를 포함하지 않는다. | [dynamic_contracts.py](src/hospital_path_lab/dynamic_contracts.py), `tests/test_dynamic_simulation.py::test_controller_frames_do_not_expose_actor_ground_truth_or_seed` | 연결됨, L1. 관측 frame은 2단계 미구현이다. |
| `DYN-ARCH-002` | 같은 seed는 같은 open-loop Actor scenario, 20 Hz frame, 사건과 SHA-256을 만든다. | [dynamic_actor.py](src/hospital_path_lab/dynamic_actor.py), [simulation.py](src/hospital_path_lab/simulation.py), `tests/test_dynamic_actor.py::test_same_seed_reproduces_scenario_and_world_hash`, `tests/test_dynamic_simulation.py::test_dynamic_trace_is_fully_reproducible_for_same_seed` | 연결됨, L1. 단일 원형 Actor 합성 궤적이다. |
| `DYN-SIM-001` | Actor는 반지름 `0.18m`, 최대속도 `0.50m/s`의 piecewise-linear open-loop waypoint를 따른다. | [dynamic_actor.py](src/hospital_path_lab/dynamic_actor.py), `tests/test_dynamic_actor.py::test_actor_contracts_reject_invalid_radius_speed_time_and_nonfinite_values`, `::test_piecewise_linear_state_waits_moves_and_stops` | 연결됨, L1. 실제 사람 운동 모델이 아니다. |
| `DYN-SIM-002` | tick 시각은 누적 덧셈이 아니라 `tick_id × 0.05s`로 만들고 로봇은 1단계에서 정지한다. | [simulation.py](src/hospital_path_lab/simulation.py), `tests/test_dynamic_simulation.py::test_dynamic_simulation_uses_exact_20hz_tick_time_and_stationary_robot` | 연결됨, L1. controller closed loop는 4단계 대상이다. |
| `DYN-OUTPUT-001` | episode ID·seed 파일명의 결정론적 JSON과 reference·robot·Actor trace PNG를 생성한다. | [experiment_visualization.py](src/hospital_path_lab/experiment_visualization.py), `tests/test_dynamic_simulation.py::test_json_and_png_artifacts_are_deterministic_and_close_figures` | 연결됨, L1. 산출물은 ignored outputs에 저장한다. |

## PLAB-MAP — 지도·사건·corpus

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| PLAB-MAP-001 | 같은 seed·계열은 같은 world, episode와 SHA-256 hash를 만들고 하나의 world에서 graph와 `0.02m` grid를 만든다. | [map_factory.py](src/hospital_path_lab/map_factory.py), `tests/test_map_factory.py::test_same_seed_produces_identical_world_episode_and_hashes`, `::test_same_world_builds_graph_and_two_centimeter_grid_snapshots` | 연결됨, L1. 합성 변환 증거다. |
| PLAB-MAP-002 | 생성기는 지도 전용 알고리즘 대신 `corridor`, `intersection`, `dead_end`, `u_trap` 계열을 사용하고 10개 batch를 동결 검증한다. | [manifest.yaml](corpus/map_factory/manifest.yaml), [generated_frozen.yaml](corpus/map_factory/generated_frozen.yaml), `tests/test_map_factory.py::test_default_batch_keeps_ten_case_split_and_freeze_semantics`, `::test_checked_in_frozen_manifest_matches_generated_content_hashes` | 연결됨, L1. 생성 batch와 통합 20-case 선택은 별도다. |
| PLAB-MAP-003 | 필수 golden 12개와 의도·world/episode hash를 사람이 읽을 수 있는 manifest로 고정한다. | [golden.yaml](corpus/map_factory/golden.yaml), `tests/test_map_factory.py::test_manual_golden_manifest_and_hashes_match_all_twelve_cases`, `::test_equal_cost_golden_has_exactly_equal_branches_without_jitter`, `::test_door_widths_are_checked_on_footprint_configuration_grid`, `::test_u_trap_has_no_grid_shortcut_across_separating_walls` | 연결됨, L1. 사례가 실제 병원 ODD를 대표한다는 증거는 아니다. |
| PLAB-MAP-004 | step 사건으로 폐쇄·해제, 장애물 생성·이동·제거, 시작점 이동, 목적지 변경, 판단 무효화와 revision 전이를 재현한다. | [map_factory.py](src/hospital_path_lab/map_factory.py), `tests/test_map_factory.py::test_obstacle_lifecycle_updates_ground_truth_and_observation_revision`, `::test_revision_transitions_and_invalid_snapshot_metadata_are_explicit`, `tests/test_dstar_lite.py::test_sequential_close_open_move_start_and_no_path_reuses_state` | 연결됨, L1. 센서 관측 오차나 지연은 없다. |
| PLAB-MAP-005 | 물리 점유와 승인되지 않은 금지 영역을 분리하고 둘 다 hash·경로 검증에 반영한다. | [contracts.py](src/hospital_path_lab/contracts.py), `tests/test_map_factory.py::test_forbidden_region_is_separate_from_physical_occupancy_and_hashed`, `tests/test_provenance_safety_contract.py::test_grid_astar_routes_around_forbidden_cells` | 연결됨, L1. 제품 구역 승인 정책을 확정하지 않는다. |
| PLAB-MAP-006 | 알고리즘 소스와 공개 18개를 동결한 뒤 별도 seed에서 hidden 2개를 선택한다. | [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_experiment_runner.py::test_corpus_is_frozen_before_separate_hidden_selection` | 연결됨, L1/L2. hidden은 보안상 비밀 데이터가 아니다. |
| PLAB-MAP-007 | hidden hard failure를 같은 실행 중 튜닝하지 않고 원본 provenance·실패 step prefix와 함께 새 regression 후보로만 보존한다. 다음 회차에는 모든 record를 먼저 검증한 뒤 같은 원본 world·episode의 여러 실패를 가장 큰 failing-step prefix 하나로 합쳐 재투입한다. | [corpus_records.py](src/hospital_path_lab/corpus_records.py), [cli.py](src/hospital_path_lab/cli.py), `tests/test_corpus_records.py::test_hidden_failure_is_preserved_as_minimal_hashed_regression`, `::test_nested_experiment_candidates_are_sorted_deduplicated_and_limited`, `tests/test_experiment_runner.py::test_previous_hidden_failure_can_feed_the_next_regression_corpus` | 연결됨, L1/L2. world 중복 없이 선택한 prefix의 step 0부터 마지막 step까지 모두 재실행하고, 그 뒤 `--regression-limit`을 적용한다. event prefix 축소는 일반 delta debugging 최소화가 아니며 기존 record를 덮어쓰지 않는다. |

## PLAB-CONTRACT — 타입·provenance·가상 차체

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| PLAB-CONTRACT-001 | 전역, local, 추종을 `Planner`, `LocalPlanner`, `PathFollower` 역할 계약으로 분리하고 registry에서 생성한다. | [planners.py](src/hospital_path_lab/planners.py), [contracts.py](src/hospital_path_lab/contracts.py), [registry.py](src/hospital_path_lab/registry.py), `tests/test_cli.py::test_list_algorithms_reports_implemented_and_deferred` | 연결됨, L1. Python 계약이며 ROS 2·제품 인터페이스 확정이 아니다. |
| PLAB-CONTRACT-002 | 결과에 map ID, map·mission·observation revision, 입력 hash, 상태, 비용·시간과 역할별 지표·실패 이유를 보존한다. | [contracts.py](src/hospital_path_lab/contracts.py), [planners.py](src/hospital_path_lab/planners.py), `tests/test_provenance_safety_contract.py::test_global_result_carries_map_identity_and_input_hash`, `::test_local_and_follower_results_use_the_same_provenance_gate`, 통합 runner 시험 | 연결됨, L1/L2. |
| PLAB-CONTRACT-003 | local 계획기와 추종기는 같은 `virtual_doll_wheelchair_v0_1`을 사용하고 실제 값으로 오인되지 않게 표시한다. | [vehicle.py](src/hospital_path_lab/vehicle.py), `tests/test_collision.py::test_virtual_doll_profile_is_explicitly_simulation_only`, `tests/test_dwa.py::test_dwa_command_stays_inside_one_period_dynamic_window` | 연결됨, L1. 가상 차체 가설이다. |

## PLAB-GLOBAL — Dijkstra·A*·D* Lite

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| PLAB-GLOBAL-001 | Dijkstra와 A*는 snapshot마다 전체 재계산하고 상태·최적 비용을 Dijkstra와 독립 NetworkX oracle로 검산한다. | [planners.py](src/hospital_path_lab/planners.py), [evaluation.py](src/hospital_path_lab/evaluation.py), `tests/test_global_planners.py::test_planners_match_networkx_oracle`, `::test_astar_expands_no_more_nodes_than_dijkstra_on_normal_case` | 연결됨, L1. 정확히 같은 경로 모양은 요구하지 않는다. |
| PLAB-GLOBAL-002 | D* Lite는 `g/rhs/km` 상태를 재사용하며 폐쇄·해제·시작점 이동을 증분 처리하고 목적지 변경에는 안전하게 reset한다. | [dstar_lite.py](src/hospital_path_lab/global_algorithms/dstar_lite.py), `tests/test_dstar_lite.py::test_sequential_close_open_move_start_and_no_path_reuses_state`, `::test_goal_change_resets_state_and_matches_oracles` | 연결됨, L1. 실제 지도 갱신 latency 증거는 아니다. |
| PLAB-GLOBAL-003 | revision 퇴행과 같은 revision의 내용·hash 변경을 거부해 증분 상태를 오염시키지 않는다. | [dstar_lite.py](src/hospital_path_lab/global_algorithms/dstar_lite.py), `tests/test_dstar_lite.py::test_map_change_without_map_revision_is_rejected`, `::test_regressed_snapshot_is_rejected_without_mutating_incremental_state`, `tests/test_provenance_safety_contract.py::test_dstar_rejects_hash_change_when_all_revisions_are_unchanged` | 연결됨, L1. 최신성을 판단할 수 없으면 실행 불가다. |
| PLAB-GLOBAL-004 | 비용, 확장 노드, 계산시간, route churn, 결정성과 deadline miss를 모든 step에서 역할별 기록한다. | [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_evaluation.py::test_route_churn_uses_path_edges`, 통합 runner 시험 | 연결됨, L1/L2. 다른 실행 환경의 절대시간을 직접 비교하지 않는다. |

## PLAB-LOCAL — Grid A*·DWA

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| PLAB-LOCAL-001 | Grid A*와 DWA는 같은 footprint·여유·금지영역 계약의 충돌 검사를 사용한다. | [collision.py](src/hospital_path_lab/collision.py), `tests/test_collision.py::test_configuration_grid_uses_half_diagonal_plus_clearance_without_mutation`, `::test_map_factory_grid_astar_path_is_footprint_collision_free`, `tests/test_provenance_safety_contract.py::test_exact_cell_aabb_check_detects_one_millimetre_corner_overlap` | 연결됨, L1. Grid A* 구성공간과 DWA 연속 pose 검사의 내부 계산은 다르다. |
| PLAB-LOCAL-002 | bounded Grid A*는 8방향, corner-cut 금지와 결정적 tie-break를 사용한다. 독립 Grid Dijkstra oracle도 같은 footprint·금지영역 구성공간과 같은 reference search bounds를 사용해 상태·최적 비용을 검산한다. | [grid_astar.py](src/hospital_path_lab/local_algorithms/grid_astar.py), [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_grid_astar.py::test_grid_astar_matches_independent_dijkstra_oracle`, `::test_grid_astar_does_not_escape_reference_search_bounds`, `tests/test_experiment_runner.py::test_grid_dijkstra_oracle_uses_same_reference_bounds_as_grid_astar`, `::test_grid_dijkstra_oracle_treats_forbidden_cells_as_non_traversable` | 연결됨, L1/L2. reference path가 없는 step에는 실행하지 않는다. |
| PLAB-LOCAL-003 | DWA는 속도·가속 제한 안에서 전진·후진·제자리 회전 후보를 만들고 안전 후보가 없으면 보수적으로 `NO_PATH`를 반환한다. | [dwa.py](src/hospital_path_lab/local_algorithms/dwa.py), `tests/test_dwa.py::test_dwa_stops_when_obstacle_is_inside_stopping_margin`, `::test_dwa_dynamic_window_contains_reverse_and_in_place_rotation_from_rest`, `::test_reverse_stopping_sweep_extends_behind_robot` | 연결됨, L1. 2초 가상 궤적이며 센서·구동 지연은 없다. |
| PLAB-LOCAL-004 | Grid A*와 DWA를 모든 호환 step에서 동일 snapshot·reference·차체 조건으로 실행하고 충돌, 최소 여유, 상태, 결정성과 deadline을 기록한다. | [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), 통합 runner 시험 | 제한적 연결, L2. DWA는 각 step의 one-shot 판정이다. |
| PLAB-LOCAL-005 | reference path에서 `0.11m` 벗어난 초기 pose로 시작해 차단 `create/hold/hold` 3-step과 해제 `remove/hold×59` 60-step, 총 63-step에서 같은 DWA 상태를 유지한다. 안전정지·교착·회복과 실제 이탈 뒤 재합류를 측정한다. | [simulation.py](src/hospital_path_lab/simulation.py), [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_simulation.py::test_dynamic_local_evidence_records_stop_deadlock_recovery_and_rejoin`, 통합 runner 시험의 `dynamic_local_evidence` | 제한적 연결, L1/L2. 계약 기대값은 충돌 0, 안전정지 3회, 교착·회복, `>0.10m` deviation 관측 후 진행하며 `≤0.10m`가 된 재합류, finite command/metric이다. 전체 corpus 동적 local 폐루프는 `not_measured`다. |

## PLAB-FOLLOWER — Pure Pursuit·RPP

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| PLAB-FOLLOWER-001 | Pure Pursuit는 고정 lookahead 대조군, RPP는 adaptive lookahead와 곡률 기반 감속 후보를 제공한다. | [pure_pursuit.py](src/hospital_path_lab/followers/pure_pursuit.py), `tests/test_followers.py::test_straight_path_selects_lookahead_after_current_position`, `::test_rpp_slows_down_for_high_curvature`, `::test_rpp_adaptive_lookahead_stays_inside_profile_range` | 연결됨, L1. Nav2 제품 구현과 동일하다는 주장이 아니다. |
| PLAB-FOLLOWER-002 | 두 추종기는 한 제어 주기의 가속·감속 제한을 지키고, 도착은 위치뿐 아니라 실제 정지까지 요구한다. | `tests/test_followers.py::test_follower_linear_command_obeys_one_period_acceleration_limits`, `tests/test_followers.py::test_goal_within_tolerance_decelerates_instead_of_jumping_to_zero`, `tests/test_simulation.py::test_goal_reached_requires_position_and_actual_stop` | 연결됨, L1. 이상적 pose feedback이다. |
| PLAB-FOLLOWER-003 | 모든 Grid A* `FOUND` step에서 두 후보를 같은 길이 기반 시간 예산으로 폐루프 적분하고 도착, 횡·최대 오차, overshoot, jerk와 추가 이동거리를 기록한다. | [simulation.py](src/hospital_path_lab/simulation.py), [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_experiment_runner.py::test_follower_time_budget_is_shared_and_scales_with_path_length`, 통합 runner 시험 | 연결됨, L1/L2. 종단 경로는 Grid A* 출력이며 DWA 출력 연결 시험은 아니다. |
| PLAB-FOLLOWER-004 | 공통 길이 기반 시간 예산 안에 도착하지 못하면 split이나 기대 분류와 무관하게 `follower_timeout` hard failure로 처리한다. | [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_experiment_runner.py::test_forbidden_entry_and_general_follower_timeout_are_hard_failures`, `::test_follower_expectation_and_hidden_failure_preservation` | 연결됨, L1/L2. golden·development·hidden·regressions 모두 같은 P0 gate를 사용하며 실물 도착 증거가 아니다. |

## PLAB-SAFETY — provenance·보호정지·hard gate

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| PLAB-SAFETY-001 | map ID, 세 revision, 입력 hash 또는 `input_valid`가 맞지 않으면 전역·local·추종 결과를 실행 불가로 거부하고 통합 stale 증거에 세 역할을 모두 기록한다. | [evaluation.py](src/hospital_path_lab/evaluation.py), [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_provenance_safety_contract.py::test_same_revisions_with_different_map_id_or_hash_are_rejected`, `::test_invalidated_snapshot_is_refused_before_planning_or_following`, 통합 runner 시험의 stale 역할 집합 검증 | 연결됨, L1/L2. |
| PLAB-SAFETY-002 | 위험 해소·경로 생성·통신 복구만으로 재출발하지 않고 실제 정지, 경로 재검증, 재개 지시와 local 안전 허가를 모두 요구한다. | [safety.py](src/hospital_path_lab/safety.py), `tests/test_safety.py::test_hazard_clear_alone_does_not_resume`, `::test_automatic_resume_requires_every_gate`, `tests/test_evaluation.py::test_protective_stop_requires_validation_and_every_gate` | 연결됨, L1/L2. 상태 논리이며 물리 제동 증거가 아니다. |
| PLAB-SAFETY-003 | oracle 불일치, 충돌·금지영역 진입, 모든 split의 follower timeout, stale 실행, 무단 재개, 비결정성, 예외·non-finite와 deadline miss를 P0 hard failure로 보존한다. | [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_provenance_safety_contract.py::test_forbidden_zone_entry_has_a_distinct_validation_reason`, `tests/test_experiment_runner.py::test_forbidden_entry_and_general_follower_timeout_are_hard_failures`, `::test_tiny_simulation_deadline_is_classified_as_hard_failure`, 통합 runner 시험 | 연결됨, L1/L2. simulation-only 입력과 기준에 한정된다. |

## PLAB-EVAL·OUTPUT — 공정 비교와 재현 산출물

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| PLAB-EVAL-001 | 전역 3종, local 2종, 추종 2종을 같은 역할끼리 비교하고 `A* → Grid A* → PP/RPP` 두 종단 pipeline은 별도 Pareto에 집계한다. pipeline 성공은 planner 상태, global/local validation·oracle, follower 초기 명령 validation, 도착, 무충돌, deadline, 결정성을 모두 요구한다. | [registry.py](src/hospital_path_lab/registry.py), [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_experiment_runner.py::test_pipeline_success_requires_every_component_validation_and_oracle`, 통합 runner 시험의 `pareto.roles`와 `pareto.pipelines` 검증 | 연결됨, L2. 단일 종합 순위를 만들지 않으며 Pareto는 채택 결과가 아니다. |
| PLAB-EVAL-002 | NetworkX와 Grid Dijkstra를 독립 oracle로 사용하고 정확한 경로 문자열 대신 상태·유효성·최적 비용을 검사한다. Grid oracle은 Grid A*와 금지영역 및 search bounds를 공유한다. | [evaluation.py](src/hospital_path_lab/evaluation.py), [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_global_planners.py::test_planners_match_networkx_oracle`, `tests/test_grid_astar.py::test_grid_astar_matches_independent_dijkstra_oracle`, `tests/test_experiment_runner.py::test_grid_dijkstra_oracle_uses_same_reference_bounds_as_grid_astar`, `::test_grid_dijkstra_oracle_treats_forbidden_cells_as_non_traversable` | 연결됨, L1/L2. |
| PLAB-EVAL-003 | 역할별 및 별도 pipeline 계산시간 p50·p95·p99·최악값, 제한적 peak memory와 역할별 기하·추종 지표를 저장한다. pipeline 표본의 memory는 세 stage 기록값의 최댓값이다. | [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_evaluation.py::test_benchmark_global_records_distribution_and_oracle`, 통합 runner 시험의 역할·pipeline 분포 검증 | 제한적 연결, L1/L2. 역할 표본은 알고리즘×계열 첫 표본만 profile한다. pipeline 정책은 `maximum_profiled_stage_sample_or_zero`이며 동시 종단 peak가 아니다. |
| PLAB-EVAL-004 | simulation-only 계산시간 deadline을 결과에 기록하고 반환한 호출의 `measured_elapsed_ns`가 기준을 엄격 초과하면 hard failure로 처리한다. | [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_experiment_runner.py::test_tiny_simulation_deadline_is_classified_as_hard_failure` | 제한적 연결, L1/L2. 제품 실시간 요구사항이 아니며 실행 중 hang을 제한시간에 중단하는 watchdog은 구현하지 않았다. |
| PLAB-OUTPUT-001 | `experiment_results.json`, `pareto.json`, `summary.md`에 config, manifest, corpus, 모든 step·pipeline 결과, 세 역할 stale 증거, synthetic DWA 증거, provenance, 한계와 실패를 보존한다. | [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), `tests/test_experiment_runner.py::test_full_twenty_case_experiment_writes_reproducible_evidence` | 연결됨, L2. 생성 결과는 기본적으로 Git에 커밋하지 않는다. |
| PLAB-OUTPUT-002 | 모든 hidden step의 graph와 grid/path/DWA trajectory/follower trace를 headless PNG로 저장한다. | [experiment_visualization.py](src/hospital_path_lab/experiment_visualization.py), `tests/test_experiment_visualization.py::test_graph_plot_writes_png_with_closed_edge_and_no_figure_leak`, `::test_grid_plot_writes_overlays_and_empty_plot_without_figure_leak`, 통합 runner 시험 | 연결됨, L1/L2. 정적 step별 그림이며 연속 애니메이션이 아니다. |
| PLAB-OUTPUT-003 | hidden hard failure가 있을 때 `regression_candidates/`에 비덮어쓰기·자체 hash JSON을 저장하고 원본 freeze hash를 연결한다. 다음 실행은 모든 후보를 재귀 탐색·검증한 뒤 같은 원본 world·episode를 가장 큰 failing-step prefix 하나로 합치고, 마지막에 limit을 적용해 regression 입력으로 동결한다. | [corpus_records.py](src/hospital_path_lab/corpus_records.py), [experiment_runner.py](src/hospital_path_lab/experiment_runner.py), [cli.py](src/hospital_path_lab/cli.py), `tests/test_corpus_records.py::test_nested_experiment_candidates_are_sorted_deduplicated_and_limited`, `::test_recursive_loader_rejects_tamper_and_non_hidden_provenance`, `tests/test_experiment_runner.py::test_previous_hidden_failure_can_feed_the_next_regression_corpus` | 연결됨, L1/L2. world가 중복되지 않고 선택된 prefix의 앞선 step도 모두 실행한다. 사용자 명시 옵션으로만 추가하며 같은 hidden 실행 중 재튜닝하지 않는다. |

## 명시적 한계와 후속 후보

현재 결과를 해석할 때 다음을 함께 보존한다.

1. DWA는 호환되는 모든 corpus step에서 실행되지만 각 step은 one-shot 2초 궤적
   평가다. 별도 synthetic 63-step 시험은 초기 `0.11m` 이탈과 상태를 유지해
   정지·교착·회복·실제 재합류를 검증하지만, 전체 corpus의 동적 장애물 sequence를
   같은 방식으로 폐루프 재생하지는 않는다.
2. `A* → Grid A* → Pure Pursuit/RPP` 두 종단 조합은 모든 호환 step에서 별도
   pipeline 결과와 Pareto로 집계된다. DWA 출력이나 D* Lite 출력까지 결합한 전체
   navigation stack 비교는 아니다.
3. 추종 시간 예산은 경로 길이에 따라 늘어나는 simulation timeout이고, 계산시간
   deadline은 호출 반환 뒤의 실행 이상 감지값이다. 둘 다 실제 운행시간·실시간
   요구가 아니며, hang을 강제 종료하는 watchdog은 없다.
4. hidden 실패의 “최소 증거”는 실패 step까지의 event prefix다. 일반 목적의 입력
   최소화나 자동 알고리즘 튜닝을 수행하지 않는다. 다음 회차 재투입도 사용자가 CLI
   옵션으로 명시해야 한다. 같은 원본 world·episode의 여러 record는 모두 검증한 뒤
   가장 큰 failing-step prefix 하나로 합치며, 그 prefix의 앞선 step도 다시 실행한다.
5. peak memory의 0은 0 byte 사용을 뜻하지 않고 해당 표본을 profile하지 않았다는
   뜻이다. pipeline memory도 동시에 잰 종단 peak가 아니라 profile된 stage 표본의
   최댓값 또는 0이므로 `peak_memory_policy`와 함께 해석한다.
6. TEB, MPPI, State Lattice, Hybrid A*는 [registry.py](src/hospital_path_lab/registry.py)의
   `deferred` 후속 후보다. 구현·성능·적합성을 주장하지 않는다.
7. 합성 Python 실험의 재현성은 `G1~G5` 답변, 팀 합의, 7단계 결정, 최종 알고리즘
   채택, 실제 차체·사람 탑승 안전성을 대신하지 않는다.

## 검증 명령과 산출물 확인

저장소 루트 PowerShell에서 실행한다.

```powershell
New-Item -ItemType Directory -Force .\simulation\path_planning_lab\outputs\test-runs | Out-Null
$testRun = ".\simulation\path_planning_lab\outputs\test-runs\$([guid]::NewGuid())"
.\.venv\Scripts\python -m pytest -c .\simulation\path_planning_lab\pyproject.toml .\simulation\path_planning_lab\tests -p no:cacheprovider --basetemp=$testRun
.\.venv\Scripts\python -m ruff check .\simulation\path_planning_lab
```

통합 산출물은 실행별 고유 디렉터리를 권장한다.

```powershell
$runId = Get-Date -Format "yyyyMMdd-HHmmss"
$outputDir = ".\simulation\path_planning_lab\outputs\experiment-$runId"
.\.venv\Scripts\hospital-path-lab.exe experiment --base-seed 20260810 --hidden-seed 91260810 --output-dir $outputDir
```

이전 실행의 검증된 후보를 다음 회차에 최대 5개 추가하려면 다음처럼 실행한다.

```powershell
$previousOutput = ".\simulation\path_planning_lab\outputs\experiment-20260810-120000"
$nextOutput = ".\simulation\path_planning_lab\outputs\experiment-regression-replay"
.\.venv\Scripts\hospital-path-lab.exe experiment --base-seed 20260810 --hidden-seed 91260810 --regression-input-dir $previousOutput --regression-limit 5 --output-dir $nextOutput
```

성공 여부는 프로세스 종료 코드와 함께 `experiment_results.json`의
`hard_failures`, `limitations`, `evaluation_coverage`, `stale_result_evidence`,
`dynamic_local_evidence`, `freeze_evidence`, `regression_candidates`를 확인하고,
`pareto.json`의 `roles`와 `pipelines`를 분리해 읽는다. hard failure가 0이어도
`full_corpus_dynamic_local_closed_loop` limitation은 별도 해석해야 한다.
