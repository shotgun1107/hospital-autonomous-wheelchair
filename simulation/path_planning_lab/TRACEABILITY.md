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
| `DYN-ARCH-001` | controller-facing frame은 Actor ground truth와 seed를 포함하지 않는다. | [dynamic_contracts.py](src/hospital_path_lab/dynamic_contracts.py), `tests/test_dynamic_simulation.py::test_controller_frames_do_not_expose_actor_ground_truth_or_seed`, `tests/test_dynamic_observation.py::test_controller_observation_contract_leaks_no_ground_truth_or_labels`, `tests/test_dynamic_prediction.py::test_prediction_contracts_do_not_expose_ground_truth_or_expectation_fields` | 연결됨, L1. controller용 관측·예측 계약까지 확인하며 evaluator ground truth와는 분리돼 있다. |
| `DYN-ARCH-002` | 같은 seed는 같은 open-loop Actor scenario, 20 Hz frame, 사건과 SHA-256을 만들고 별도 RNG namespace의 관측 stream도 재현한다. | [dynamic_actor.py](src/hospital_path_lab/dynamic_actor.py), [simulation.py](src/hospital_path_lab/simulation.py), [dynamic_observation.py](src/hospital_path_lab/dynamic_observation.py), `tests/test_dynamic_actor.py::test_same_seed_reproduces_scenario_and_world_hash`, `tests/test_dynamic_simulation.py::test_dynamic_trace_is_fully_reproducible_for_same_seed`, `tests/test_dynamic_observation.py::test_normal_and_stress_are_reproducible_and_share_latent_noise_draws` | 연결됨, L1. 단일 원형 Actor 합성 궤적과 합성 Gaussian/dropout 관측이다. |
| `DYN-SIM-001` | Actor는 반지름 `0.18m`, 최대속도 `0.50m/s`의 piecewise-linear open-loop waypoint를 따른다. | [dynamic_actor.py](src/hospital_path_lab/dynamic_actor.py), `tests/test_dynamic_actor.py::test_actor_contracts_reject_invalid_radius_speed_time_and_nonfinite_values`, `::test_piecewise_linear_state_waits_moves_and_stops` | 연결됨, L1. 실제 사람 운동 모델이 아니다. |
| `DYN-SIM-002` | tick 시각은 누적 덧셈이 아니라 `tick_id × 0.05s`로 만든다. 4단계 pipeline은 현재 twist로 한 tick 적분해 gate 명령을 다음 tick에 적용하고, 5단계 evaluator는 각 구간을 200 Hz로 독립 재적분한다. | [simulation.py](src/hospital_path_lab/simulation.py), [dynamic_evaluation.py](src/hospital_path_lab/dynamic_evaluation.py), `tests/test_dynamic_simulation.py::test_dynamic_simulation_uses_exact_20hz_tick_time_and_stationary_robot`, `tests/test_dynamic_evaluation.py::test_200hz_ground_truth_evaluator_passes_empty_stationary_trace` | 연결됨, L1. Python 합성 차체와 open-loop Actor에 한정된다. |
| `DYN-OUTPUT-001` | episode ID·seed 파일명의 결정론적 JSON과 reference·robot·Actor trace PNG를 생성한다. | [experiment_visualization.py](src/hospital_path_lab/experiment_visualization.py), `tests/test_dynamic_simulation.py::test_json_and_png_artifacts_are_deterministic_and_close_figures` | 연결됨, L1. 산출물은 ignored outputs에 저장한다. |

## DYN-STAGE2 — 열화 관측과 Actor prediction tube

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| `DYN-OBS-001` | 20 Hz ground truth에서 seed 기반 10 Hz Normal·Stress·Boundary 관측을 만들고, frame dropout·4-frame burst·fresh `EMPTY`·no-frame을 서로 구분한다. | [dynamic_observation.py](src/hospital_path_lab/dynamic_observation.py), `tests/test_dynamic_observation.py::test_twenty_hz_truth_is_sampled_at_exact_ten_hz_with_frozen_latency`, `::test_normal_and_stress_are_reproducible_and_share_latent_noise_draws`, `::test_dropout_removes_whole_frame_and_preserves_sequence_gaps`, `::test_forced_four_frame_burst_does_not_shift_following_frames`, `::test_fresh_empty_frame_and_no_frame_have_distinct_state_effects` | 연결됨, L1. 실제 센서나 실제 사람 관측이 아닌 합성 Gaussian·Bernoulli 열화다. |
| `DYN-OBS-002` | source·episode·map·sequence·revision·hash·binding·profile σ·시간 순서를 transactional하게 검증하고 `age == 300ms`는 fresh, 초과는 stale로 둔다. prediction은 validator가 만든 fresh snapshot만 받는다. | [dynamic_observation.py](src/hospital_path_lab/dynamic_observation.py), [dynamic_prediction.py](src/hospital_path_lab/dynamic_prediction.py), [dynamic_safety.py](src/hospital_path_lab/dynamic_safety.py), `tests/test_dynamic_observation.py::test_each_source_identity_mismatch_returns_a_structured_reason`, `::test_sequence_revision_hash_duplicate_and_binding_faults_are_transactional`, `::test_ttl_exactly_300ms_is_fresh`, `::test_ttl_any_later_nanosecond_is_stale`, `tests/test_dynamic_safety.py::test_stale_and_invalid_sources_only_allow_limited_deceleration`, `::test_prediction_identity_mismatch_is_invalid_source` | 연결됨, L1. source invalid·stale에서 새 명령을 거부하고 제한 감속한다. 실제 센서 입력 증거는 아니다. |
| `DYN-SAFE-001` | 관측 age, 50 ms 적용 지연, vector `0.50m/s` clamp, 2σ와 임의방향 가속 reachable bound로 controller 비종속 time-indexed 원형 tube를 만들고 online gate가 동일 tube를 사용한다. | [dynamic_prediction.py](src/hospital_path_lab/dynamic_prediction.py), [dynamic_safety.py](src/hospital_path_lab/dynamic_safety.py), [dynamic_evaluation.py](src/hospital_path_lab/dynamic_evaluation.py), `tests/test_dynamic_prediction.py::test_actor_tube_matches_independent_center_sigma_and_acceleration_oracle`, `tests/test_dynamic_safety.py::test_actor_tube_static_obstacle_and_forbidden_cell_are_all_rejected`, `tests/test_dynamic_evaluation.py::test_exact_actor_surface_clearance_threshold_is_a_hard_pass` | 연결됨, L1. online prediction tube와 독립 ground-truth 판정을 모두 시험하지만 실제 센서·사람 증거는 아니다. |

### DYN-DIR-v7 — 방향 고정 합성 Actor 공개-only lane

후속 지역 기동 연구의 전체 단계·gate·hidden 수명주기는
[`R1~R7 master specification`](../../docs/research/dynamic-actor-experiment/10-dynamic-local-maneuver-research-master-spec.md)을
따른다. 이 번호는 제품 경로분석 단계나 기존 동적 Actor 구현 단계와 다른 연구 단계다.
R2의 상세 계약은
[`witness 자동화·일반화 명세`](../../docs/research/dynamic-actor-experiment/11-witness-automation-and-generalization.md)에
있다. 현재는 label-free 계약·projection, 독립 ground-truth validator, WAIT/HOLD·PASS
structured search, 대표 profile replay와 공개 13+6 영구 audit/reporting runner까지 L1으로
연결됐다. 최신 전체 실행은 완료됐지만 Ideal coverage hard failure 2건 때문에 결합 R2 자격은
통과하지 못했다. 후속 [`ADR 0011`](../../docs/decisions/0011-separate-path-and-perception-research-gates.md)에
따라 이를 R2-B failure로 보존하고 R2-A 공간 미해결 사례의 R3 진입은 분리했다.

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| `DYN-DIR-001` | 동일 stable identity·binding의 최신 20개 unique accepted `observed_velocity`를 평균하고 `norm(v_mean)-2·(max(velocity_sigma)/√20) >= 0.03m/s`일 때만 direction을 lock한다. anchor는 최신 `observed_position`, 위치 sigma는 최신 frame 값이다. stale·invalid·track/binding 변경과 순서 위반은 이전 이력을 폐기한다. | [dynamic_directional_prediction.py](src/hospital_path_lab/dynamic_directional_prediction.py), `tests/test_dynamic_directional_prediction.py::test_direction_locks_only_after_twenty_unique_accepted_tracks_frames`, `::test_duplicate_observation_is_idempotent_and_does_not_grow_history`, `::test_session_identity_change_resets_history_instead_of_mixing_actors`, `::test_stale_invalid_and_unavailable_states_are_explicit_holds` | 연결됨, L1. v6 공개 constant-heading open-loop 원형 Actor만 다루며 실제 사람 방향추정이 아니다. |
| `DYN-DIR-002` | 최신 위치에서 heading 축의 제한 감속 `d_min`과 제한 가속 `d_max`를 endpoint로 삼고, 속도 평균 불확실성은 radius에 한 번만 반영한다. exact Capsule과 oriented wheelchair footprint의 표면거리가 canonical이며 circle-chain은 legacy 호환 표본이다. | [dynamic_directional_prediction.py](src/hospital_path_lab/dynamic_directional_prediction.py), [collision.py](src/hospital_path_lab/collision.py), [dynamic_safety.py](src/hospital_path_lab/dynamic_safety.py), `tests/test_dynamic_directional_prediction.py::test_capsule_uses_forward_only_braking_and_bounded_acceleration`, `tests/test_collision.py::test_optimized_capsule_distance_matches_world_polygon_reference_randomly` | 연결됨, L1. `lateral_turn_bound=0`이고 방향전환 Actor는 범위 밖이다. `2σ`는 연구용 휴리스틱이지 확률적 안전 보장이 아니다. |
| `DYN-DIR-003` | 공개 same-direction-wide 5개에서 Normal은 20-frame 뒤 eventual READY, Stress 저속은 READY 0이고 모든 non-READY TRACKS에서 hold·prediction 미노출이며 corpus 전체에 LOW_CONFIDENCE가 존재한다. 공개-only exact geometry는 217후보×후보당 41 pose의 2초 rollout과 terminal stopping까지 결정론적으로 계산한다. | `tests/test_directional_public_qualification.py::test_normal_same_direction_public_cases_lock_after_twenty_unique_frames`, `::test_stress_same_direction_low_speed_remains_fail_closed`, `::test_ready_capsules_cover_full_rollout_and_terminal_with_exact_geometry` | 연결됨, L1. targeted 합계 `33 passed`. 기하 계산 자격은 실제 DWB 이탈·추월·재합류를 증명하지 않으며 online bypass는 `ONLINE_DWB_BYPASS_UNPROVEN`이다. |
| `DYN-DIR-004` | READY 또는 fresh EMPTY는 안전 관측 후보가 될 수 있지만, warmup hold 뒤 재개에는 11개 unique safe frame, current `stop_epoch` authorization, path valid, local recheck와 shared evidence safe가 모두 필요하다. | [dynamic_safety.py](src/hospital_path_lab/dynamic_safety.py), `tests/test_directional_public_qualification.py::test_directional_lane_warmup_hold_then_ready_requires_every_resume_gate` | 연결됨, L1. confidence 획득만으로 자동 재출발하지 않는다. |
| `DYN-DIR-005` | v7 구현·공개 자격·manifest 동결 전에는 새 hidden을 생성·열람·실행하지 않는다. | [08-directional-actor-prediction-v7.md](../../docs/research/dynamic-actor-experiment/08-directional-actor-prediction-v7.md), [0010-directional-actor-prediction.md](../../docs/decisions/0010-directional-actor-prediction.md) | 미측정. v7 hidden 미실행이며 제품 채택·실제 사람 증거가 아니다. |
| `DYN-PRED-AUDIT-001` | 결정론적 Actor 운동 계약과 Gaussian 관측 coverage를 별도 판정한다. 공개 motion 위반과 Ideal 관측·Capsule miss만 hard failure이고 Normal·Stress의 `2σ` miss는 통계적 limitation이다. | [09-prediction-contract-audit.md](../../docs/research/dynamic-actor-experiment/09-prediction-contract-audit.md), [dynamic_prediction_audit.py](src/hospital_path_lab/dynamic_prediction_audit.py), `tests/test_dynamic_prediction_audit.py::test_public_audit_separates_motion_and_statistical_coverage` | 연결됨, L1. 실제 사람 운동·센서·확률적 안전 보장이 아니다. |
| `DYN-PRED-AUDIT-002` | 공개 v6 `GOLDEN`·`DEVELOPMENT` 13개만 허용하고 hidden 입력을 거부한다. 같은 입력은 같은 semantic 결과 hash를 만들며 기존 output을 덮어쓰지 않는다. | [run_prediction_contract_audit.py](scripts/run_prediction_contract_audit.py), `tests/test_dynamic_prediction_audit.py::test_public_audit_is_deterministic`, `::test_public_audit_rejects_hidden_input`, `::test_writer_preserves_evidence_and_refuses_overwrite` | 연결됨, L1. 최신 실측은 별도 고유 output에 보존하며 저장소에 대용량 결과를 커밋하지 않는다. |
| `DYN-PRED-AUDIT-003` | 공개 motion에 나타나지 않은 가속·감속·정지·회전은 미검증 limitation으로 명시하고, synthetic auditor에서 제한 감속·정지는 허용하되 순간 회전·측면 이탈은 거부한다. | [dynamic_prediction_audit.py](src/hospital_path_lab/dynamic_prediction_audit.py), `tests/test_dynamic_prediction_audit.py::test_motion_contract_accepts_bounded_deceleration_and_stop`, `::test_motion_contract_rejects_heading_change_and_lateral_motion` | 연결됨, L1. public corpus 일반화를 주장하지 않으며 해당 장면은 후속 공개 corpus 명세 대상이다. |

### DYN-WIT-R2 — 공개 feasible-witness 자동화 기반

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| `DYN-WIT-001` | 검색 입력은 공개 `GOLDEN`·`DEVELOPMENT`만 허용하고 category, family·orientation label, oracle, 기존 witness, progressable·blocking 정보와 controller 결과를 제외한 `WitnessWorldSnapshot`만 사용한다. 지도·Actor ID도 원본 관리 ID가 아닌 projected content에 결박한다. | [dynamic_witness_contracts.py](src/hospital_path_lab/dynamic_witness_contracts.py), `tests/test_dynamic_witness_contracts.py::test_public_projection_is_label_and_oracle_free`, `::test_legacy_label_changes_do_not_change_projection_semantics`, `::test_hidden_projection_is_rejected_before_content_is_exposed` | 연결됨, L1. ground-truth Actor trajectory는 offline search·oracle 전용이며 online controller 입력이 아니다. |
| `DYN-WIT-002` | no-passing과 allowed-region은 category에서 추론하지 않고 별도 `ManeuverConstraintSpec`과 hash로 입력한다. 검색 상태는 found, structured-template no-witness, resource-limit, invalid-input을 구분한다. 후보 bucket 합계, validator version·선택 validation hash를 결박하고 wall-clock은 semantic hash에서 제외한다. | [dynamic_witness_contracts.py](src/hospital_path_lab/dynamic_witness_contracts.py), [dynamic_witness_search.py](src/hospital_path_lab/dynamic_witness_search.py), `tests/test_dynamic_witness_contracts.py::test_explicit_policy_is_hashed_without_category_inference`, `::test_search_result_semantic_hash_excludes_wall_clock`, `tests/test_dynamic_witness_search.py::test_result_counts_validation_hash_and_semantics_are_deterministic`, `::test_small_timed_candidate_limit_is_inconclusive_resource_limit` | 연결됨, L1. structured subset의 resource 결과이며 일반 공간·시간 불가능 판정이 아니다. |
| `DYN-WIT-003` | 최종 validator는 검색 pruning·objective와 기존 corpus private validator를 호출하지 않고 20 Hz 운동학·가감속, 200 Hz static·forbidden·exact Actor 원 clearance, ordered departure→overtake→재합류와 terminal dwell을 재검증한다. 5 ms grid 밖 Actor 활성 사건도 exact sample로 검사한다. HOLD는 episode 전체 정지, WAIT는 terminal dwell 밖의 실제 wait 뒤 `0.10m` 이상 progress를 추가로 요구한다. | [dynamic_witness_validation.py](src/hospital_path_lab/dynamic_witness_validation.py), `tests/test_dynamic_witness_validation.py::test_legacy_positive_is_reproduced_by_independent_validator`, `::test_all_five_public_feasible_replicas_pass_ground_truth_validation`, `::test_200hz_validator_catches_between_tick_actor_clearance`, `::test_validator_samples_exact_off_grid_actor_appearance_time`, `tests/test_dynamic_witness_search.py::test_full_duration_hold_passes_but_short_hold_is_rejected`, `::test_terminal_dwell_alone_does_not_count_as_actual_wait`, `::test_wait_after_all_progress_does_not_satisfy_wait_then_follow_order` | 연결됨, L1. exact ground-truth offline 검증이며 online prediction·controller·권한 실행 증거가 아니다. |
| `DYN-WIT-004` | label·oracle-free world에서 Actor 활성 event anchor, 20 Hz reference-follow와 exact Actor terminal-stopping guard로 `WAIT_AND_FOLLOW`를 생성하고, 안전한 mission witness가 없을 때만 full-duration `HOLD_ONLY`를 선택한다. 초기 제동·최소 wait를 반영한 effective departure를 중복 제거하고 best WAIT·HOLD만 streaming 보존한다. 모든 선택 후보는 독립 validator를 통과해야 한다. | [dynamic_witness_search.py](src/hospital_path_lab/dynamic_witness_search.py), `tests/test_dynamic_witness_search.py`의 14개 pytest case | 연결됨, L1. WAIT/HOLD template 밖 해의 부재를 뜻하지 않는다. |
| `DYN-WIT-005` | 공개 입력 전체에서 WAIT/HOLD selected witness와 독립 validator의 연결을 점검한다. | 2026-08-13 읽기 전용 수동 감사: v6 공개 `13/13`, legacy golden `6/6` selected witness validator pass | 수동 확인, L1 한정. 체크인된 전체 audit runner·영구 CI나 taxonomy 정답 일치가 아니며, profile replay·online 실행·제품 채택 증거가 아니다. hidden은 사용하지 않았다. |
| `DYN-WIT-006` | `PASS_LEFT/RIGHT`는 같은 방향·앞선 단일 Actor만 target으로 삼고, departure→Actor active 중 ordered overtake→동일 segment 재합류→0.50 s dwell을 strict ground-truth로 검증한다. 매 이동 tick은 제한감속 terminal-stopping guard를 거치며 allowed/prohibited policy와 non-target Actor를 무시하지 않는다. | [dynamic_witness_pass.py](src/hospital_path_lab/dynamic_witness_pass.py), [dynamic_witness_validation.py](src/hospital_path_lab/dynamic_witness_validation.py), `tests/test_dynamic_witness_pass.py`, `tests/test_dynamic_witness_validation.py` | 연결됨, L1. 축 정렬 직선·단일 target structured template이며 일반 pose-space 해 존재나 online 실행 가능성을 뜻하지 않는다. |
| `DYN-WIT-007` | PASS 후보는 frozen ordinal을 연속 shard로 나눠 process 병렬 평가할 수 있다. parent는 range 전체 coverage와 count를 확인하고 동일 total objective key로 best를 선택한 뒤 strict validator/hash를 재확인한다. worker 수·shard 크기·wall-clock은 semantic 결과에서 제외한다. | [dynamic_witness_pass.py](src/hospital_path_lab/dynamic_witness_pass.py), `tests/test_dynamic_witness_pass.py::test_parallel_shards_match_serial_semantics_and_counts` | 연결됨, L1. 2026-08-13 회사 PC 20 physical/28 logical CPU에서 14 worker로 공개 wide 5개 `135,360`후보를 완주했다. 총 wall-clock 약 `29분 29초`는 운영 진단이며 timing 자격이 아니다. |
| `DYN-WIT-008` | 공개 same-direction-wide 5개에서 좌·우 structured PASS 자동 발견과 strict validator 통과를 확인한다. | 2026-08-13 public-only 완전탐색: validated `38,660`, dynamic reject `70,318`, geometry reject `26,382`; 총 `135,360`. 직접 영향권 `106 passed`, 전체 회귀 `668 passed`. | 수동 실행 L1. profile replay·공개 13+6 영구 audit·JSON/PNG·online controller·제품 채택 증거가 아니며 hidden은 사용하지 않았다. |
| `DYN-WIT-009` | 자동 witness를 Ideal·Normal·Stress 관측 stream과 방향성 predictor에 재생하고 최초 READY, 상태 interval, 지연 witness ground-truth 재검증, post-apply 5 ms Capsule clearance와 actual Actor containment를 분리한다. | [dynamic_witness_profile_replay.py](src/hospital_path_lab/dynamic_witness_profile_replay.py), `tests/test_dynamic_witness_profile_replay.py`의 11개 pytest case | 연결됨, L1 대표 checkpoint. r00 RIGHT에서 Ideal READY `2.00s`, Normal `2.10s`, Stress READY 없음. Ideal·Normal 지연 witness는 ground-truth 재검증 실패했고 Ideal predicted minimum clearance 약 `0.07427m`; 직접 영향권 `163 passed`, 전체 회귀 `679 passed`. controller·gate는 실행하지 않았다. |
| `DYN-WIT-010` | Gaussian `2σ` Capsule의 actual Actor miss를 exact ground-truth hard clearance와 분리한다. Ideal miss만 hard replay failure이며 Normal·Stress miss는 통계 limitation이다. | [13-witness-profile-replay.md](../../docs/research/dynamic-actor-experiment/13-witness-profile-replay.md), [dynamic_witness_profile_replay.py](src/hospital_path_lab/dynamic_witness_profile_replay.py), `tests/test_dynamic_witness_profile_replay.py::test_normal_and_stress_keep_degradation_separate_from_ground_truth` | 연결됨, L1. 대표 Normal containment miss `354/3,285`; 이 수치로 safety margin이나 prediction envelope를 완화하지 않는다. |
| `DYN-WIT-011` | 공개 v6 13개와 legacy mechanism golden 6개를 label-free search 뒤 evaluator taxonomy와 결합하고, WAIT/HOLD·PASS·profile replay·독립 검증·JSON/Markdown/PNG·partial/complete 수명주기를 한 audit에서 봉인한다. | [dynamic_witness_reporting.py](src/hospital_path_lab/dynamic_witness_reporting.py), [run_dynamic_witness_audit.py](scripts/run_dynamic_witness_audit.py), `tests/test_dynamic_witness_public_audit.py`, [실행 결과](../../docs/research/dynamic-actor-experiment/r2-public-witness-audit-result-2026-08-13.md) | 연결됨, L1. 2026-08-13 `19/19` 실행은 complete이나 v6 second-risk와 legacy dynamic-change의 Actor 출현 전 관측 공백에서 Ideal containment hard failure 2건이 발생해 R2 자격 fail. 전체 회귀 `688 passed`. hidden·controller·제품 증거가 아니다. |
| `DYN-WIT-012` | Ground-truth 시간 경로 R2-A와 observation/prediction R2-B를 별도 gate로 유지한다. R2-B failure는 online 통합 자격을 막지만 observation을 입력으로 받지 않는 R3 static 공간 oracle을 막지 않는다. | [ADR 0011](../../docs/decisions/0011-separate-path-and-perception-research-gates.md), [Pro 반영 판정](../../docs/reviews/pro-r2-actor-appearance-review-disposition-2026-08-13.md), [R1~R7 master](../../docs/research/dynamic-actor-experiment/10-dynamic-local-maneuver-research-master-spec.md) | 문서 연결, L1. R2-A 공개 audit·legacy 표적 보완과 R2-B hard failure를 제품·카메라 통합 완료로 합치지 않는다. |
| `DYN-R3-001` | Actor·관측·controller를 제거한 bounded 정적 공간에서 oriented footprint의 통과·회전·재합류 가능성을 판정하고, exhaustive 음성·resource limit·invalid input·infrastructure incomplete를 분리한다. | [R3 상세 명세](../../docs/research/dynamic-actor-experiment/14-bounded-spatial-oracle.md), [contracts](src/hospital_path_lab/spatial_oracle_contracts.py), [lattice](src/hospital_path_lab/spatial_oracle_lattice.py), [validator](src/hospital_path_lab/spatial_oracle_validation.py), [projection](src/hospital_path_lab/spatial_oracle_projection.py), [reporting](src/hospital_path_lab/spatial_oracle_reporting.py), [runner](scripts/run_spatial_oracle_public.py), `tests/test_spatial_oracle_*.py`, [실행 결과](../../docs/research/dynamic-actor-experiment/r3-public-spatial-qualification-result-2026-08-14.md) | core·reporting L1 연결. clean commit `53fd9f8`에서 public `21/21`, 관계 오류 0, parity PASS, receipt 생성, 전체 회귀 `729 passed`. R3 path는 online 이동 허가·제품 planner·실제 사람 안전 증거가 아니며 multi-segment corner는 미구현이다. |
| `DYN-R4-001` | 검증된 R2/R3 source를 WAIT/LEFT/RIGHT immutable full reference와 revision-bound sliding subpath로 변환하고, 동일 path의 window 이동은 controller session을 유지하며 stale revision·과거 session 결과는 거부한다. | [R4 상세 명세](../../docs/research/dynamic-actor-experiment/15-local-maneuver-reference-contract.md), [ADR 0012](../../docs/decisions/0012-persistent-controller-session-for-sliding-subpaths.md), [contracts](src/hospital_path_lab/local_reference_contracts.py), [builder](src/hospital_path_lab/local_reference_builder.py), [validator](src/hospital_path_lab/local_reference_validation.py), [window manager](src/hospital_path_lab/local_reference_window.py), [reporting](src/hospital_path_lab/local_reference_reporting.py), [runner](scripts/run_local_reference_public.py), `tests/test_local_reference_*.py`, [실행 결과](../../docs/research/dynamic-actor-experiment/r4-public-local-reference-qualification-result-2026-08-14.md) | R4-1~R4-6 L1 완료. clean commit `f43fbbf`에서 public `21/21`, reference ready 8·no-reference 11·inconclusive 1·invalid 1, hard·relation failure 0, parity·repeat determinism PASS, receipt 생성과 전체 회귀 `794 passed`. 대표 `wide-straight-left`는 validation, 23-knot window, subgoal revision `0→4`와 terminal window를 통과했다. R2 temporal evidence·R5 연결은 미수행이며 이동 허가·동적 안전·controller 추종·제품 알고리즘 증거가 아니다. |

## DYN-STAGE3 — 동적 safety gate·권한·시간

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| `DYN-SAFE-001` | 현재 운동의 50 ms 적용 구간, post-apply rollout과 terminal stopping을 5 ms 간격으로 검사하고 static grid·금지 cell·Actor tube 표면 여유 미달을 거부한다. | [dynamic_safety.py](src/hospital_path_lab/dynamic_safety.py), [collision.py](src/hospital_path_lab/collision.py), `tests/test_dynamic_safety.py::test_actor_tube_static_obstacle_and_forbidden_cell_are_all_rejected`, `::test_malformed_rollout_is_rejected_instead_of_raising` | 제한적 연결, L1. online prediction 안전필터이며 ground-truth evaluator는 아니다. |
| `DYN-SAFE-002` | 49·50 ms의 현재 tick 결과만 허용하고 51 ms, 과거·미래 tick 결과를 폐기하며 제한 감속한다. 제안 명령은 mission·map·세 revision·grid/observation hash에 결합한다. | [dynamic_safety.py](src/hospital_path_lab/dynamic_safety.py), `tests/test_dynamic_timing.py::test_49_and_50_millisecond_results_are_valid_for_current_tick`, `::test_51_millisecond_result_is_discarded_and_braking_is_applied`, `::test_past_or_future_tick_result_is_never_applied`, `::test_late_old_result_cannot_replace_a_previously_accepted_newer_result`, `::test_proposal_provenance_mismatch_is_rejected` | 연결됨, L1. 주입한 computation time 계약이며 고정 머신 wall-clock qualification은 6단계 대상이다. |
| `DYN-AUTH-001` | 선속도·각속도가 임계값 이하로 3 tick 연속 확인될 때 서로 다른 보호정지당 `stop_epoch`를 1회 증가시킨다. 이동 중 goal·cancel도 실제 정지 뒤 완료하지만 epoch는 증가시키지 않고, 이미 확인된 hold에서 cancel하면 새 epoch 없이 완료한다. | [dynamic_safety.py](src/hospital_path_lab/dynamic_safety.py), `tests/test_dynamic_authority.py::test_stop_epoch_increments_once_per_distinct_protective_stop`, `::test_stop_confirmation_requires_both_thresholds_for_three_consecutive_ticks`, `::test_normal_goal_completion_does_not_create_stop_epoch`, `::test_mission_cancel_from_confirmed_hold_completes_without_new_epoch` | 연결됨, L1. 합성 RobotState feedback이다. |
| `DYN-AUTH-002` | 현재 mission·epoch·정지시각·authorization revision이 일치하고 새로운 safe frame 11개, path 유효성과 local 재검사를 모두 만족할 때만 재개한다. no-frame은 safe frame 누적을 초기화한다. | [dynamic_safety.py](src/hospital_path_lab/dynamic_safety.py), `tests/test_dynamic_authority.py::test_eleven_new_safe_frames_and_current_epoch_authorization_resume`, `::test_hazard_clear_and_safe_frames_without_new_authorization_do_not_resume`, `::test_no_frame_resets_continuous_safe_frame_count`, `::test_wrong_or_pre_stop_authorization_is_rejected` | 연결됨, L1. 재개 요청 주체와 제품 권한 정책은 확정하지 않는다. |

## DYN-STAGE4 — PP·DWA 동적 controller 통합

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| `DYN-CTRL-001` | PP와 DWA는 동일 `ControllerSnapshot` provenance와 `0.20m/s` 자유주행 목표속도를 사용한다. | [dynamic_contracts.py](src/hospital_path_lab/dynamic_contracts.py), [registry.py](src/hospital_path_lab/registry.py), `tests/test_dynamic_controller_parity.py::test_dynamic_controllers_share_input_hash_and_free_space_target_speed`, `tests/test_dynamic_pp_pipeline.py::test_dynamic_pp_result_preserves_snapshot_provenance` | 연결됨, L1. `0.30m/s`는 가상 차체 물리 상한일 뿐 동적 비교 목표속도가 아니다. |
| `DYN-CTRL-002` | PP는 polyline projection·0.35m lookahead·remaining arc goal 감속을 사용하며 자체 detour를 생성하지 않는다. PP rollout은 post-apply pose부터 2초·41 pose다. | [pure_pursuit.py](src/hospital_path_lab/followers/pure_pursuit.py), `tests/test_dynamic_pp_pipeline.py::test_dynamic_pp_tracks_reference_without_creating_a_detour`, `::test_dynamic_pp_uses_remaining_arc_goal_speed_and_angular_rate_limit` | 연결됨, L1. rollout은 현재 명령 검사용 상수 명령 예측이며 제품 차체 예측이 아니다. |
| `DYN-CTRL-003` | 동적 DWA는 후진 없이 최대 217후보·후보당 41 pose, v5 절대 비용과 결정론 tie-break를 유지한다. v6는 후보 탈락 phase/cause와 선택·미선택·미평가를 분리하고 step-local 정적·금지구역 기하를 재사용하되 요청 명령과 비시간 semantic digest를 보존한다. | [dwa.py](src/hospital_path_lab/local_algorithms/dwa.py), [collision.py](src/hospital_path_lab/collision.py), [dynamic_safety.py](src/hospital_path_lab/dynamic_safety.py), `tests/test_dynamic_dwa_pipeline.py::test_dynamic_dwa_uses_217_candidates_and_41_post_apply_poses`, `::test_dynamic_dwa_nontrivial_cost_and_rank_match_the_pre_v6_oracle`, `::test_dynamic_dwa_boundary_precedes_actor_for_all_217_candidates`, `::test_dynamic_dwa_step_workspace_matches_all_public_episode_profile_representatives`, `::test_dynamic_dwa_real_corner_and_multisegment_reduce_exact_geometry_by_work_count`, `tests/test_collision.py::test_forbidden_certification_small_non_square_grid_falls_back_exactly` | 연결됨, L1. Python+NumPy standalone 500회의 miss `100/500`은 역사적 실패다. 선택적 C++ 경로는 동일 timing 하위 자격을 통과했지만 full 공개 기능 자격이나 제품 적합성을 주장하지 않는다. |
| `DYN-PIPE-001` | 두 controller 결과는 우회 경로 없이 actuator로 가지 않고 동일 동적 gate를 거친다. trace는 controller 요청, gate 명령·override, hold reason과 전후 RobotState를 분리한다. | [simulation.py](src/hospital_path_lab/simulation.py), `tests/test_dynamic_controller_parity.py::test_pp_crossing_actor_stops_holds_and_resumes_with_current_epoch_authority`, `::test_new_actor_risk_restarts_braking_for_both_pipelines` | 연결됨, L1. online Actor tube 판정이며 독립 중복 안전채널이 아니다. |
| `DYN-PIPE-002` | no-Actor 완료, 횡단 stop/hold/resume, 넓은 공간 DWA detour/rejoin, 좁은 차단 hold, no-candidate와 새 위험 재정지를 mechanism golden으로 고정하고 같은 seed의 명령·상태·사건열을 재현한다. | `tests/test_dynamic_controller_parity.py`, `tests/test_dynamic_dwa_pipeline.py::test_dynamic_dwa_is_deterministic_except_for_elapsed_time`, `tests/test_dynamic_evaluation.py::test_authorized_safety_hold_is_not_planner_deadlock` | 연결됨, L1. whole-corpus paired closed loop 통계는 6단계 대상이다. |

## DYN-STAGE5 — 독립 ground-truth evaluator와 공개 corpus

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| `DYN-EVAL-001` | controller가 보지 못한 실제 Actor 상태와 연속적인 20 Hz chassis trace를 200 Hz로 재적분하고 sample 사이 상대이동 상한까지 차감해 collision·`0.08m` clearance·금지구역을 hard 판정한다. step 시간·전후 state·final state와 재개 stop epoch 불일치도 hard failure다. | [dynamic_evaluation.py](src/hospital_path_lab/dynamic_evaluation.py), [collision.py](src/hospital_path_lab/collision.py), `tests/test_dynamic_evaluation.py::test_swept_margin_detects_collision_between_5ms_samples`, `::test_forbidden_entry_is_a_hard_failure_not_a_metric_only_warning`, `::test_hard_safety_rejects_resume_with_a_different_stop_epoch`, `::test_hard_safety_rejects_discontinuous_state_time_and_final_state` | 연결됨, L1. 보수적 수치기하 평가이며 연속 실물 swept-volume 증명은 아니다. |
| `DYN-EVAL-002` | hold와 deadlock을 분리하고 completion·hold 원인·path length·deviation·jerk·각운동·clearance·TTC·rejoin·overtaking을 limiter 적용 뒤 실제 simulated state에서 계산한다. | [dynamic_evaluation.py](src/hospital_path_lab/dynamic_evaluation.py), `tests/test_dynamic_evaluation.py::test_authorized_safety_hold_is_not_planner_deadlock`, `::test_rejoin_and_reference_projection_overtaking_have_explicit_oracles` | 연결됨, L1. 전체 36 episode 결과 집계와 controller 우열 판정은 아직 수행하지 않았다. |
| `DYN-CORP-001` | 공개 corpus는 범주별 golden 1개와 development 5개, 총 `6+30`개이며 같은 seed에서 episode·map·Normal/Stress stream hash를 재현한다. | [dynamic_corpus.py](src/hospital_path_lab/dynamic_corpus.py), `tests/test_dynamic_corpus.py::test_golden_and_development_corpus_are_balanced_and_valid`, `::test_same_seed_reproduces_corpus_map_and_observation_streams` | 연결됨, L1. 실제 병원 사람 행동 분포를 대표하지 않는다. |
| `DYN-CORP-002` | expectation label·split·oracle을 controller 입력에서 제외하고 PP와 DWA에 같은 paired snapshot·observation hash를 제공한다. | [dynamic_corpus.py](src/hospital_path_lab/dynamic_corpus.py), `tests/test_dynamic_corpus.py::test_pp_and_dwa_receive_the_exact_same_label_free_paired_input`, `::test_both_controllers_replay_each_golden_first_snapshot_with_same_provenance` | 제한적 연결, L1. golden 첫 유효 snapshot 재생까지이며 whole-episode paired runner는 6단계 대상이다. |
| `DYN-CORP-003` | legacy-v1 공개 36개와 exact hash를 보존하면서 v6 공개 13개를 별도 생성한다. v6는 opaque source·map·Actor ID, semantic-world/oracle hash, 실제 90° rigid pair, rasterized static·forbidden·다중 Actor와 20 Hz 비홀로노믹 feasible witness를 검증한다. | [dynamic_corpus.py](src/hospital_path_lab/dynamic_corpus.py), `tests/test_dynamic_corpus.py::test_legacy_v1_public_lane_keeps_exact_36_episode_hash`, `::test_v6_public_matrix_is_deterministic_separate_and_valid`, `::test_v6_evaluator_metadata_relabel_does_not_change_controller_world`, `::test_v6_actor_trajectory_uses_the_rasterized_static_cell_extent`, `::test_v6_feasible_witness_is_dense_and_rejects_nonholonomic_tampering`, `::test_v6_witness_time_grid_boundary_and_rigid_duration_are_fail_closed` | 연결됨, L1. witness는 200 Hz에서 Normal·Stress 연구용 tube와 합성 차체·지도만 검증하며 실제 사람의 회피 가능성을 보장하지 않는다. |
| `DYN-EVAL-003` | v6 category oracle은 정확한 controller ID를 fail-closed로 판별하고, hazard interval별 별도 stop epoch·허가된 중간 재출발과 같은 방향 Actor의 이탈·추월·추월 뒤 0.5초 재합류 순서를 검증한다. | [dynamic_evaluation.py](src/hospital_path_lab/dynamic_evaluation.py), `tests/test_dynamic_evaluation.py::test_v6_category_oracle_requires_sustained_rejoin_and_same_direction_pass`, `::test_dynamic_change_oracle_does_not_count_initial_invalid_stop_as_hazard`, `::test_dynamic_change_oracle_requires_authorized_resume_between_hazards` | 연결됨, L1. 공개 기능 oracle이며 알고리즘 승격 판정은 정식 public-only runner 실행 뒤에만 가능하다. |
| `DYN-FAULT-001` | observation 13종, authority 7종, deadline 5종을 성능 corpus와 분리하고 fresh empty·single dropout·TTL boundary와 stale를 서로 다른 기대 응답으로 고정한다. | [dynamic_corpus.py](src/hospital_path_lab/dynamic_corpus.py), `tests/test_dynamic_contract_faults.py`, `tests/test_dynamic_observation.py`, `tests/test_dynamic_authority.py`, `tests/test_dynamic_timing.py` | 연결됨, L1. Stage 2·3의 validator/gate 시험을 공통 자격 근거로 재사용한다. |

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

## DYN-RUN — paired runner·hidden·판정

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| DYN-R2A-001 | 같은 방향 PASS와 횡단 Actor 우회를 분리하고, Actor가 direct lane을 실제로 막는 동안 station을 signed side로 통과한 뒤 0.50 s 재합류한 사건을 200 Hz ground truth로 검증한다. | [dynamic_witness_events.py](src/hospital_path_lab/dynamic_witness_events.py), [dynamic_witness_crossing.py](src/hospital_path_lab/dynamic_witness_crossing.py), [dynamic_witness_validation.py](src/hospital_path_lab/dynamic_witness_validation.py), `tests/test_dynamic_witness_r2a_supplement.py::test_legacy_crossing_has_distinct_bypass_witnesses_on_both_sides` | 연결됨, L1. legacy crossing 표적에서 좌·우를 증명했지만 일반 pose-space 완전성·R2-B 관측 가능성·online 실행을 뜻하지 않는다. |
| DYN-R2A-002 | 두 exact blocking hazard에 결박된 서로 다른 정지 사이에 실제 이동과 0.10 m 이상 progress를 요구하고, 두 번째 위험 뒤 회복을 별도 증거로 분리한다. | [dynamic_witness_restop.py](src/hospital_path_lab/dynamic_witness_restop.py), `tests/test_dynamic_witness_r2a_supplement.py::test_legacy_two_hazard_search_proves_ordered_restop_and_recovery`, `::test_continuous_hold_is_not_misclassified_as_two_restops` | 연결됨, L1. legacy restop 표적은 통과했으나 기존 R2-B `ideal_capsule_ground_truth_miss`는 보류다. |

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| DYN-RUN-001 | v6 public-only runner는 전체 legacy+v6 공개 corpus·Normal/Stress·실제 contract·동결된 30/100 qualification만 정식 봉인 대상으로 인정한다. 축소·주입 실행은 report-only이며 전체 record 조합·rigid 결과·evidence·scenario·source hash가 실제 exclusive write 직전까지 일치할 때만 receipt를 만든다. | [dynamic_runner.py](src/hospital_path_lab/dynamic_runner.py), `tests/test_dynamic_runner.py::test_full_public_run_seals_v6_receipt_with_complete_hashes`, `::test_limited_or_injected_run_is_non_sealing`, `::test_incomplete_public_cross_product_never_seals_receipt`, `::test_rigid_pair_metamorphic_gate_rejects_stale_component_hash`, `::test_source_change_immediately_before_receipt_file_write_fails_closed` | 연결됨, L1. 정식 expanded-public 실측은 아직 수행하지 않았으므로 L2 자격 통과를 주장하지 않는다. |
| DYN-RUN-002 | 독립 episode·profile은 process 병렬화하되 같은 pair의 PP·DWA를 한 worker에서 실행하고, 결과를 입력 순서로 재정렬하며 wall-clock qualification은 pool 종료 뒤 직렬 실행한다. | [dynamic_runner.py](src/hospital_path_lab/dynamic_runner.py), `tests/test_dynamic_runner.py::test_process_parallel_results_match_serial_order_and_semantics`, `dynamic-experiment-20260811-final-v4` manifest의 worker 14·serial qualification 증거 | 연결됨, L2. worker 경과시간은 nonqualification이며 50 ms 판정에 쓰지 않는다. |
| DYN-HID-001 | v5 hidden helper와 `final-v4`는 회귀 이력으로만 남긴다. v6 공개 기능·안전·50 ms 자격이 모두 통과하고 별도 단계가 승인되기 전에는 runner·CLI에서 hidden 실행이나 test override를 노출하지 않는다. | [dynamic_runner.py](src/hospital_path_lab/dynamic_runner.py), [cli.py](src/hospital_path_lab/cli.py), `tests/test_dynamic_runner.py::test_public_runner_exports_no_dynamic_hidden_execution_or_override` | 연결됨, L1. 새 v6 hidden은 생성·소비하지 않았고 기존 hidden을 새 최종평가에 재사용하지 않는다. |
| DYN-STAT-001 | Normal hidden progressable paired 집합에서 median 개선율, class-stratified bootstrap 95% CI와 denominator floor 승차감 악화율을 계산한다. | [dynamic_runner.py](src/hospital_path_lab/dynamic_runner.py), `tests/test_dynamic_statistics.py` | 연결됨, L2. 단일 종합점수나 제품 채택 판정이 아니다. |
| DYN-OUTPUT-001 | paired 결과·hard safety·qualification·Pareto·10개 승격 조건·PNG를 저장하고 hidden 실패를 자체 hash·비덮어쓰기 회귀 후보로 남긴다. | [dynamic_runner.py](src/hospital_path_lab/dynamic_runner.py), [corpus_records.py](src/hospital_path_lab/corpus_records.py), [experiment_visualization.py](src/hospital_path_lab/experiment_visualization.py), `tests/test_dynamic_runner.py`, `tests/test_dynamic_hidden_lifecycle.py::test_hidden_failure_is_exclusive_and_tamper_evident`, final-v4의 264 records·120 PNG·67 regression candidates | 연결됨, L2. 생성 output은 기본 Git 대상이 아니다. |

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

## C++ DWA 가속 경로 추적

| ID | 연구 요구사항 | 구현·시험 근거 | 상태·증거 한계 |
|---|---|---|---|
| DYN-CPP-001 | 217개 후보, 후보당 41 pose, terminal stopping, oriented footprint, Actor tube, 비용·tie-break를 바꾸지 않고 반복 수치 계산을 C++로 옮긴다. | [dwa_core.cpp](native/dwa_core.cpp), [cpp_dwa_core.py](src/hospital_path_lab/cpp_dwa_core.py), `tests/test_cpp_dwa_core.py::test_cpp_core_preserves_frozen_candidate_and_pose_counts` | 연결됨, L1. C++ 라이브러리가 없으면 시험은 skip되고 Python fallback을 사용한다. |
| DYN-CPP-002 | C++ 결과도 기존 shared safety gate를 통과해야 하며 Python 기준선과 같은 공개 입력 결과를 내야 한다. | [dwa.py](src/hospital_path_lab/local_algorithms/dwa.py), `tests/test_cpp_dwa_core.py::test_cpp_core_matches_python_on_all_frozen_qualification_snapshots`, `tests/test_dynamic_dwa_pipeline.py` | 연결됨, L1. 98개 공개 대표 snapshot의 별도 one-tick 대조는 구현 세션 진단이며 정식 public qualification이 아니다. |
| DYN-CPP-003 | 정적 지도 보조 배열은 동일 map revision에서 재사용하고 ABI·공유 라이브러리 문제에서는 Python으로 fallback한다. | [cpp_dwa_core.py](src/hospital_path_lab/cpp_dwa_core.py), [build_cpp_dwa_core.py](scripts/build_cpp_dwa_core.py), `tests/test_cpp_dwa_core.py::test_explicit_python_fallback_does_not_use_native_core` | 연결됨, L1. 2026-08-12 동일 5-case×100 직렬 timing에서 PP·C++ DWA 모두 miss `0/500`, C++ DWA p50 `3.770 ms`, p95 `15.459 ms`, 최대 `35.576 ms`였다. expanded public·receipt·hidden은 미실행이다. |
