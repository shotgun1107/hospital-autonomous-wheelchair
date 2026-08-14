# R4 — 지역 기동 Reference·Sliding Subpath 계약

## 1. 문서 상태와 목적

- 작성일: `2026-08-14`
- 상태: 구현 전 상세 명세
- 상위 기준:
  - [`R1~R7 master specification`](10-dynamic-local-maneuver-research-master-spec.md)
  - [`R3 bounded spatial oracle`](14-bounded-spatial-oracle.md)
  - [`ADR 0011`](../../decisions/0011-separate-path-and-perception-research-gates.md)
  - [`ADR 0012`](../../decisions/0012-persistent-controller-session-for-sliding-subpaths.md)
  - [`경로 안전·권한 흐름`](../../safety/path-safety-authority-flow.md)
- 실행 범위: Python `simulation_only`, 공개 합성 지도, 가상 차체
- 팀 합의·제품 알고리즘 채택·`G1~G5`: 미수행
- ROS 2·실차·실제 센서·사람 탑승: 범위 밖

R3는 Actor와 시간을 제거한 정적 공간에서 pose·heading 경로의 존재를 판정했다. R4는
독립 validator를 통과한 R3 경로와 검증된 R2 WAIT/PASS 근거를 persistent controller가
소비할 수 있는 방향 있는 지역 reference로 변환한다.

핵심 질문은 다음이다.

> 검증된 지역 기동의 전체 기하·회전·재합류 의미와 provenance를 잃지 않으면서, 같은
> controller session에 안정적인 sliding local subpath를 제공할 수 있는가?

reference의 존재·유효성·선택 가능성은 이동 허가가 아니다.

## 2. 책임 경계

R3 path를 controller에 직접 넘길 수 없는 이유:

- 동일 위치 회전 pose와 translation pose가 하나의 tuple에 섞여 있다.
- R3 primitive는 공간 연결 증거이지 시간화된 차체 명령이 아니다.
- immutable full path와 현재 controller가 보는 local window가 구분되지 않는다.
- path·window·controller session의 revision과 stale 폐기 규칙이 없다.
- R3는 static geometry만 보므로 Actor·시간·재출발 권한을 증명하지 않는다.

```text
R2 ground-truth time witness ─┐
                              ├─> R4 reference candidate set
R3 validated spatial path ────┘       ├─ immutable full reference
                                      └─ revision-bound sliding subpath
                                                    │
                                                    v
                                           R5 persistent controller
                                                    │
                                                    v
                                           shared online safety gate
```

R4가 수행하는 작업:

- source result의 identity·content hash·validation을 확인한다.
- pose·heading·primitive를 canonical knot와 section으로 변환한다.
- WAIT/LEFT/RIGHT를 하나의 schema로 표현한다.
- full reference·sliding window·session의 revision 수명주기를 정의한다.
- rotation·translation·stop marker를 controller adapter가 구분하게 한다.
- stale·변조·모순 입력을 fail-closed한다.

R4가 수행하지 않는 작업:

- 최적 후보·제품 알고리즘 선택
- 속도 profile·가속도·차체 command 생성
- Actor 예측·online collision 판정·이동 허가
- 보호정지 해제·자동 재출발
- RPP·DWB 실행 비교
- hidden 생성·열람·실행

## 3. 입력과 source 자격

### 3.1 공통 문맥

```text
ReferenceBuildContext
  schema_version
  mission_id
  stop_epoch
  map_id
  map_revision
  mission_revision
  observation_dependency
  observation_revision | null
  observation_content_hash | null
  static_grid_snapshot
  grid_content_hash
  allowed_region
  allowed_region_hash
  forbidden_cells
  forbidden_region_hash
  vehicle_profile_hash
  original_reference_hash
  original_reference
  current_robot_pose
  control_tick
  simulation_time_s
  context_content_hash
```

`observation_dependency`는 다음 둘만 허용한다.

| 값 | 의미 |
|---|---|
| `STATIC_ONLY` | R3 공간 변환이며 observation을 안전 근거로 사용하지 않음 |
| `REQUIRED` | 특정 validated observation에 의존함 |

`STATIC_ONLY`이면 observation revision/hash는 `null`이어야 한다. 이를 fresh·safe·Actor 없음으로
해석하지 않는다. `REQUIRED`이면 둘 다 존재해야 하며 현재 observation과 다르면 거부한다.

### 3.2 R3 source

다음을 모두 만족하는 결과만 `SpatialReferenceSeed`로 투영한다.

```text
status == SPATIALLY_FEASIBLE
validation.passed == true
result.request_content_hash == source request hash
validation.request_content_hash == result.request_content_hash
validation.path_content_hash == recomputed path hash
result semantic hash == recomputed semantic hash
map/mission/grid/profile provenance == current context
```

```text
SpatialReferenceSeed
  source_spatial_result_hash
  source_spatial_request_hash
  source_validation_hash
  map_id
  map_revision
  mission_revision
  grid_content_hash
  vehicle_profile_hash
  side
  start_pose
  rejoin_goal
  pose_heading_path
  primitive_sequence
  minimum_clearance_m
  limitations
  seed_content_hash
```

`SPATIALLY_INFEASIBLE`, `RESOURCE_LIMIT`, `INVALID_INPUT`, validation failure와 hash mismatch는
reference로 변환하지 않는다. `RESOURCE_LIMIT`을 경로 없음으로 바꾸지 않는다.

### 3.3 R2 source

WAIT 또는 동적 PASS 의미를 붙이려면 R2 canonical strict validation을 통과한 witness를
다음 evidence로 받는다. R2 trajectory를 차체 명령으로 복사하지 않는다.

```text
TemporalReferenceEvidence
  source_witness_hash
  source_validation_hash
  maneuver_kind
  target_actor_binding_ids
  departure_progress_m | null
  pass_progress_m | null
  rejoin_progress_m | null
  ground_truth_only
  limitations
  evidence_content_hash
```

category·oracle·episode label·split·family·hidden identity는 입력으로 받지 않는다.

## 4. Evidence level

| 값 | 확보 근거 | 허용되는 후속 사용 |
|---|---|---|
| `SPATIAL_ONLY` | R3 static path+independent validation | R4 schema·geometry·window 시험 |
| `GROUND_TRUTH_TEMPORAL` | R3+R2-A exact Actor witness | R5 Ideal path-only 연구 후보 |
| `OBSERVATION_INTEGRATED` | 위 근거+R2-B observation/prediction | perception-integrated R5/R6 후보 |

현재 R2-B hard failure가 남아 있으므로 최고 수준은 원칙적으로
`GROUND_TRUTH_TEMPORAL`이다. `SPATIAL_ONLY`를 동적 통과 성공 후보로 승격하지 않는다.

## 5. Canonical reference

### 5.1 종류와 상위 결과

```text
LocalManeuverKind
  WAIT_OR_FOLLOW
  PASS_LEFT
  PASS_RIGHT

ReferenceBuildStatus
  REFERENCE_SET_READY
  WAIT_ONLY
  NO_REFERENCE
  SEARCH_INCONCLUSIVE
  INVALID_INPUT
```

`GLOBAL_REROUTE_REQUEST`, `SUPPORT_REQUEST`는 local path가 아니므로 별도 상위 disposition으로
유지한다.

### 5.2 Knot와 section

```text
ReferenceKnot
  knot_index
  pose
  tangent_yaw
  cumulative_translation_arc_m
  source_path_index
  section_index
  knot_roles

knot_roles의 원소
  ANCHOR | TRANSLATION | ROTATION_ENTRY | ROTATION_EXIT | STOP_MARKER | REJOIN
```

불변조건:

- knot index는 `0`부터 연속 증가하고 모든 수치는 finite다.
- translation arc는 감소하지 않는다.
- 위치가 변하면 arc 증가는 metric 거리와 일치한다.
- 동일 위치에서 heading만 바뀌면 arc는 같고 rotation marker를 갖는다.
- tangent는 위치 차분으로 추측하지 않고 source heading·section에서 결정한다.
- 첫 knot는 source start, 마지막 knot는 rejoin tolerance 안에 있다.
- terminal knot는 `REJOIN`과 `STOP_MARKER` 두 role을 함께 가진다.

R3의 in-place rotation을 중복 polyline 점으로 조용히 제거하지 않는다.

```text
ReferenceSection
  section_index
  section_kind
  first_knot_index
  last_knot_index
  entry_requires_stopped
  exit_requires_stopped
  source_primitive_indices
  section_content_hash

section_kind
  FOLLOW_ORIGINAL | DEPART | ROTATE | BYPASS | RETURN | REJOIN | HOLD
```

`ROTATE` section은 entry·exit knot와 stop marker를 하나의 atomic section으로 유지한다.

### 5.3 Full reference

```text
LocalManeuverReference
  schema_version
  reference_contract_version
  candidate_id
  maneuver_kind
  evidence_level
  mission_id
  stop_epoch
  map_id
  map_revision
  mission_revision
  observation_dependency
  observation_revision | null
  observation_content_hash | null
  maneuver_revision
  path_revision
  reference_session_id
  source_spatial_seed_hash | null
  source_temporal_evidence_hash | null
  original_reference_hash
  grid_content_hash
  vehicle_profile_hash
  allowed_region_hash
  forbidden_region_hash
  knots
  sections
  departure_knot_index | null
  pass_section_index | null
  rejoin_knot_index
  minimum_validated_static_clearance_m
  validity
  generation_reason_codes
  limitations
  reference_content_hash
```

`WAIT_OR_FOLLOW`는 원 reference를 유지하고 hold 뒤 재검토한다는 의미만 담는다. 자동 release
시각을 담지 않는다. `PASS_LEFT/RIGHT`는 R3 side와 일치해야 한다.

### 5.4 Candidate set

```text
LocalManeuverReferenceSet
  schema_version
  build_context_hash
  maneuver_revision
  candidates
  upper_dispositions
  rejected_sources
  limitations
  semantic_content_hash
  elapsed_nonqualification_ns
```

정렬은 `WAIT_OR_FOLLOW`, `PASS_LEFT`, `PASS_RIGHT`, `candidate_id` 오름차순으로 고정한다.
이는 추천 순위가 아니다. R4 v1은 candidate를 선택하지 않고 R5에서 후보별 paired 실행한다.
한 set의 후보는 같은 generation `maneuver_revision`을 공유하지만 `candidate_id`와
`reference_session_id`는 서로 다르다. R5에서 한 후보를 active binding한 뒤 다른 후보로
바꾸려면 같은 revision의 sibling을 즉시 실행하지 않고 새 maneuver revision으로 다시 발행한다.

## 6. Revision·session·lifecycle

| 필드 | 증가 조건 | 유지 조건 |
|---|---|---|
| `maneuver_revision` | 새 candidate set, active kind·side·source evidence·stop epoch·기동 의미 교체 | 동일 active 기동 window 이동 |
| `path_revision` | full knots·sections·rejoin·geometry 변경 | 동일 full path window 이동 |
| `subgoal_revision` | controller window 시작·끝·terminal marker 변경 | 동일 window 재전달 |

- revision은 non-negative exact integer이며 같은 mission에서 감소하지 않는다.
- 같은 revision에 다른 content hash가 오면 `INVALID_INPUT`이다.
- 같은 입력 duplicate는 idempotent하게 같은 결과/hash를 반환한다.
- 새 maneuver/path를 수용할 때 새 `reference_session_id`를 만든다.
- 동일 maneuver/path의 window 이동은 session을 유지한다.
- 이전 session의 늦은 command/result는 실행하지 않는다.
- `stop_epoch`가 바뀌면 기존 reference를 이동 근거로 재사용하지 않는다.
- 기존 기하를 재사용하려면 최신 context로 재검증해 새 maneuver revision으로 발행한다.

```text
AVAILABLE
  ├─ same-path window update ─> AVAILABLE
  ├─ newer maneuver/path ─────> SUPERSEDED
  ├─ provenance invalid ──────> STALE
  ├─ explicit cancellation ───> WITHDRAWN
  └─ evaluator terminal proof ─> COMPLETED
```

`AVAILABLE`은 controller 입력 후보일 뿐 이동 허가가 아니다. `COMPLETED`는 R5 이후 실제 chassis
state와 evaluator가 확인한다.

## 7. Validity envelope

```text
ReferenceValidity
  required_mission_id
  required_stop_epoch
  required_map_revision
  required_mission_revision
  required_observation_revision | null
  valid_from_control_tick
  valid_until_control_tick | null
  requires_actual_stop_confirmation
  requires_resume_authorization
  requires_local_safety_recheck
```

- static path라도 map·mission·stop epoch가 바뀌면 재검증한다.
- `valid_until=null`은 무기한 안전이 아니라 revision 기반 static 유효성만 뜻한다.
- 마지막 세 boolean은 현재 안전 흐름에서 항상 `true`다.
- R4는 실제 정지·재개 승인·local safety 통과를 선언하지 않는다.

## 8. Sliding local subpath

```text
LocalReferenceWindow
  reference_session_id
  maneuver_revision
  path_revision
  subgoal_revision
  full_reference_hash
  source_control_tick
  start_knot_index
  end_knot_index
  knots
  sections
  terminal_rejoin_included
  window_content_hash
```

R4 v1 simulation-only config:

```text
control_period_s = 0.05
rear_context_arc_m = 0.10
minimum_forward_window_arc_m = 0.60
window_advance_quantum_m = 0.10
projection_tie_tolerance_m = 1e-9
maximum_cursor_regression_m = 0.05
```

`0.60m`는 현재 연구 controller의 `0.35m` lookahead와 `0.20m/s × 2.0s = 0.40m`
rollout보다 긴 geometric window다. 제품 수치가 아니다.

생성 규칙:

1. robot pose를 full reference translational section에 투영한다.
2. 비인접 section 또는 반대 tangent의 동률 projection은 fail-closed한다.
3. cursor는 이전 값보다 `0.05m` 초과 후퇴하지 않는다.
4. start는 cursor에서 최대 `0.10m` 뒤 atomic section 경계다.
5. end는 cursor에서 최소 `0.60m` 앞이며 atomic section 끝으로 올림한다.
6. rejoin이 window 안이면 terminal marker까지 포함한다.
7. 동일 slice면 subgoal revision을 올리지 않는다.
8. end가 `0.10m` 이상 전진하거나 atomic section이 바뀔 때만 올린다.

window는 full reference의 수정 없는 contiguous slice다. controller별 resampling은 R5 adapter의
별도 hash-bound 파생 결과로 다룬다.

rotation section 진입 전·exit knot를 같은 window에 둔다. rotation 중 translation arc만으로
다음 section으로 건너뛰지 않으며 실제 회전 완료는 R5가 pose·yaw·twist로 확인한다.

## 9. 변환 규칙

### R3

- path/primitive 길이 관계를 재검사한다.
- anchor connector는 source anchor 의미를 유지한다.
- translation primitive는 translation knot/section이 된다.
- rotation primitive는 동일 위치 entry/exit knot와 `ROTATE` section이 된다.
- reverse는 limitation·metadata에 남긴다. 실제 후진 허용을 결정하지 않는다.
- start·rejoin·clearance는 independent R4 validator가 다시 확인한다.

### WAIT

- 검증된 R2 WAIT/HOLD 또는 original reference의 별도 static validation이 필요하다.
- HOLD와 FOLLOW를 같은 motion state로 표현하지 않는다.
- terminal dwell을 traffic wait로 오인하지 않는다.
- Actor 해소·EMPTY·path 존재만으로 release 조건을 만들지 않는다.
- actual stop·현재 epoch용 resume authorization·local recheck 요구를 유지한다.

### PASS

- PASS kind와 R3 side가 일치해야 한다.
- `DEPART→BYPASS→RETURN→REJOIN` 순서를 갖는다.
- R2 evidence가 있으면 progress anchor가 R4 projection과 tolerance 안에서 일치해야 한다.
- R2 evidence가 없으면 `SPATIAL_ONLY`이며 ordered overtake를 주장하지 않는다.
- static R3 변환에 Actor 위치를 주입하지 않는다.

## 10. Independent validator

converter와 별도 모듈이 검사한다.

### Provenance·hash

- source R2/R3, context, reference, section, window hash 재계산
- map·mission·grid·profile·stop epoch 일치
- 같은 revision/다른 content, 이전 revision/session 결과 거부

### 구조

- knot index·section range 연속성
- finite pose·heading·arc와 translation arc monotonicity
- path/primitive/source index 대응
- rotation atomicity·stop marker 보존
- depart→bypass→return→rejoin 순서
- side sign·minimum excursion·rejoin pose/heading tolerance

### 기하

- R3와 동일 grid·allowed·forbidden·vehicle profile
- 모든 knot와 인접 구간의 oriented swept-footprint
- source R3 validation과 minimum clearance 일치
- window가 full reference의 contiguous slice인지 확인

R4 validator는 Actor ground truth·category·oracle을 읽지 않는다. 시간·Actor 안전은 R2/R5
evaluator의 별도 책임이다.

## 11. 결과 taxonomy

Hard invalid:

```text
unsupported_schema
source_hash_mismatch
source_validation_missing
source_validation_failed
source_status_not_feasible
map_or_mission_provenance_mismatch
stop_epoch_mismatch
observation_dependency_mismatch
same_revision_different_content
revision_regression
stale_reference_session
non_finite_reference
path_primitive_length_mismatch
reference_structure_invalid
rotation_marker_lost
window_not_contiguous
window_hash_mismatch
ambiguous_reference_projection
independent_geometry_validation_failed
```

정상 음성·limitation:

```text
no_spatial_candidate
wait_source_unavailable
temporal_evidence_missing
observation_evidence_unavailable
multi_segment_projection_unsupported
reverse_primitive_simulation_only
search_resource_limit_passthrough
```

`temporal_evidence_missing`은 `SPATIAL_ONLY`로 남을 수 있지만 R5 동적 성공 자격에는 쓰지 않는다.

## 12. 원인 로그

```text
reference_generation_reason
reference_candidate_rejection
reference_lifecycle_transition
window_update_reason
controller_result
shared_gate_override
authority_hold_reason
ground_truth_evaluator_verdict
```

candidate 없음, stale 거부, controller 추종 실패와 gate override를 서로 다른 필드로 기록한다.

## 13. 공개 시험

### 계약·변환

- exact enum/int/finite/hash/schema
- feasible R3 result만 seed로 변환
- validation/hash/path tamper와 resource/infeasible 비변환
- 같은 revision/다른 path, revision regression, 이전 session 거부
- elapsed wall-clock semantic hash 제외
- straight/mirror/vertical LEFT·RIGHT 관계
- just-wide door clearance 보존
- crossing-static의 section 순서
- in-place rotation·reverse limitation 보존
- independent swept geometry 재검증
- multi-segment corner는 명시적 unsupported/limitation

### WAIT/PASS evidence

- WAIT에 자동 resume 조건이 없음
- spatial-only와 ground-truth-temporal 분리
- R2 validation tamper·progress projection mismatch 거부
- category·oracle·split·hidden identifier AST 누출 검사

### Sliding window

- duplicate idempotence
- 20Hz cursor/window 단조 전진
- 작은 이동은 revision 유지, quantum 이상은 subgoal revision만 증가
- same path에서 maneuver/path/session 유지
- rotation section 미절단
- ambiguous projection·cursor regression fail-closed
- path 교체 때 새 path revision/session
- 과거 window·늦은 controller result 거부

### Public matrix

R3 public 21-case를 입력 순서대로 사용한다.

- feasible: conversion+R4 validation
- infeasible: lateral reference 생성 금지
- resource limit: `SEARCH_INCONCLUSIVE` passthrough
- invalid: fail-closed passthrough
- mirror·vertical 관계 보존

별도로 R2 공개 WAIT/PASS 대표 source를 붙여 evidence level을 검사한다. expectation label은 builder
입력에 넣지 않는다.

## 14. 실행·병렬화·산출물

- 독립 candidate/public case는 process 병렬화한다.
- 한 session의 window sequence는 상태 의존이므로 직렬 실행한다.
- 결과는 input ordinal로 재정렬한다.
- Python wall-clock·CPU·RSS·cache는 운영 진단이며 합격 기준이 아니다.
- timeout·crash·OOM은 `INFRASTRUCTURE_INCOMPLETE`이며 `NO_REFERENCE`로 바꾸지 않는다.

```text
outputs/local-reference-public-<UTC>-<HEAD>/
  run-manifest.json
  partial-state.json
  cases/<ordinal>-<case>/
    build-context.json
    source-evidence.json
    reference-set.json
    validation.json
    reference.png
  summary.json
  summary.md
  complete-state.json
  qualification-receipt.json
```

partial은 보존하지만 final evidence로 사용하지 않는다. receipt는 clean source·전체 public·hard
failure 0·결정론·serial/process parity·마지막 source 재확인을 통과할 때만 만든다. output은
기본적으로 Git에 커밋하지 않는다.

## 15. 구현 파일과 순서

```text
src/hospital_path_lab/local_reference_contracts.py
src/hospital_path_lab/local_reference_builder.py
src/hospital_path_lab/local_reference_validation.py
src/hospital_path_lab/local_reference_window.py
src/hospital_path_lab/local_reference_reporting.py
scripts/run_local_reference_public.py
tests/test_local_reference_contracts.py
tests/test_local_reference_builder.py
tests/test_local_reference_validation.py
tests/test_local_reference_window.py
tests/test_local_reference_public.py
```

validator는 builder를 import하지 않는다. builder는 corpus category·oracle·evaluator를 import하지
않는다. 기존 `ControllerSnapshot`은 바로 수정하지 않고 R4 계약을 닫은 뒤 R5 adapter에서 연결한다.

```text
R4-1 contracts·hash·revision lifecycle       20~30분
R4-2 R3 seed→canonical builder               30~45분
R4-3 independent validator                   30~45분
R4-4 sliding window/session                  30~45분
R4-5 public reporting·process runner         30~45분
R4-6 대표 public→감사→전체 public→회귀       실행 전 재산정
```

이는 작업시간 추정이며 기능 판정이 아니다. 장시간 full은 최종 감사 전 시작하지 않는다.

## 16. 완료조건

- WAIT/LEFT/RIGHT가 하나의 immutable schema에 있다.
- R3 pose·heading·rotation·rejoin 의미가 보존된다.
- R2/R3·map·mission·stop epoch와 hash가 결박된다.
- full reference와 window hash/revision이 분리된다.
- same-path window 이동이 controller session을 재생성하지 않는다.
- stale revision·다른 session 결과가 fail-closed된다.
- independent validator가 구조와 static swept geometry를 재검증한다.
- public matrix·serial/process parity를 통과한다.
- category·oracle·ground truth·hidden이 builder/controller 입력에 누출되지 않는다.
- `SPATIAL_ONLY`, `GROUND_TRUTH_TEMPORAL`, observation 미완료를 구분한다.
- 영향권 시험과 구현 뒤 마지막 전체 회귀가 통과한다.

## 17. R5 전달과 중단조건

```text
ControllerReferenceInput
  current ReferenceBuildContext identity
  LocalManeuverReference
  LocalReferenceWindow
  reference_session_id
  maneuver_revision
  path_revision
  subgoal_revision
  full_reference_hash
  window_content_hash
```

R5 controller result는 위 identity를 그대로 반환해야 한다. shared gate는 현재 context와 하나라도
다르면 command를 적용하지 않는다.

다음 중 하나면 R5로 넘어가지 않는다.

- R3 path를 손실 없이 변환하지 못함
- rotation marker·rejoin heading 소실
- same revision/different content 수용
- window 갱신마다 controller session 초기화
- 과거 session command/result 수용
- independent geometry hard failure
- category·oracle·hidden 정보 누출
- source와 R4 결과 hash/provenance 불일치
- static-only reference를 observation-integrated 또는 이동 허가로 표시

R4 reference는 동적 안전, 현재 이동 허가, controller 추종 성공, 제품 planner 채택, 실제 센서·
차체·사람 탑승 안전을 의미하지 않는다.
