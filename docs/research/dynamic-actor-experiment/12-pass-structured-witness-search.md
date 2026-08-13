# R2-PASS — 좌·우 통과 Witness 자동 탐색 상세 명세

## 1. 문서 상태와 목적

- 작성일: `2026-08-13`
- 상태: 구현 전 동결 후보, 사용자 검토 대기
- 상위 단계: [R1~R7 Master Specification](10-dynamic-local-maneuver-research-master-spec.md)의 `R2`
- 상위 상세 명세: [R2 Witness 자동화·일반화](11-witness-automation-and-generalization.md)
- 선행 구현: label-free projection, 독립 ground-truth validator, `WAIT_AND_FOLLOW`,
  `HOLD_ONLY`
- 실행 범위: Python `simulation_only`, 공개 `GOLDEN`·`DEVELOPMENT`만 사용
- hidden: 생성·열람·실행 금지
- 제품 알고리즘 채택, `G1~G5`, 제품 경로분석 7단계: 미수행

이 문서는 R2의 세 번째 구현 묶음인 `PASS_LEFT`·`PASS_RIGHT` structured witness 자동
탐색을 구현하기 전에 후보 공간, 선택 규칙, 독립 검증과 시험 순서를 고정한다.

질문은 다음 하나다.

> 정확한 공개 ground truth와 동결한 가상 차체·지도·안전조건 아래에서, Actor가 실제로
> 존재하는 동안 기준 경로를 벗어나 Actor를 추월하고 원 경로로 재합류하는 안전한
> 시간 궤적이 structured template 안에 존재하는가?

이 단계는 DWB·DWA·PP를 실행하거나 튜닝하지 않는다. witness가 존재해도 online controller가
그 기동을 만들 수 있다는 뜻이 아니며 실제 통행 허가나 이동 명령도 아니다.

## 2. 선행 계약과 이번 변경 이유

현재 구현은 `WAIT_AND_FOLLOW`와 `HOLD_ONLY`의 최적 후보 한 개를 반환한다. PASS를 같은 단일
objective에 추가하면 더 짧거나 덜 움직이는 WAIT가 선택되어, 유효한 PASS가 존재해도 결과에서
사라질 수 있다.

따라서 이번 단계는 아래 두 사실을 분리한다.

```text
각 witness kind에 안전한 해가 존재하는가
!=
그중 어떤 기동을 운용 정책으로 선택할 것인가
```

R2는 첫 번째만 다룬다. 종류 간 운용 우선순위나 제품 선택은 이 단계에서 결정하지 않는다.

## 3. 범위와 비범위

### 3.1 포함

- 기준 경로의 한 직선 segment 안에서 수행되는 정지·제자리회전·측면이동 기반 통과
- `PASS_LEFT`, `PASS_RIGHT` 양쪽 후보
- 시작 후 기준 경로를 따라 departure 지점까지 이동하는 prefix
- 정확한 static·forbidden·allowed-region 기하
- 정확한 open-loop Actor 원과 20 Hz 시간 후보
- Actor 존재 중 ordered overtake
- 원 경로 재합류와 연속 `0.50 s` dwell
- episode 종료 전 `REJOIN_DWELL` 또는 가능한 경우 `GOAL_DWELL`
- 종류별 최적 witness, 후보 수와 거부 원인

### 3.2 제외

- 일반 pose-space, State Lattice, Hybrid A*, SIPP 탐색
- 곡선 주행으로 수행하는 smooth pass
- 코너를 가로지르거나 여러 reference segment에 걸친 pass
- 후진, 3점 회전, 복수 Actor를 순차·동시에 모두 추월하는 기동
- Actor 반응, 사람 행동 예측 또는 실제 사람 안전 주장
- noisy observation에 맞춘 geometry 튜닝
- persistent controller, shared gate, `stop_epoch` 권한 실행
- wall-clock 50 ms controller 자격

제외 범위에서 witness를 못 찾은 결과는 `NO_WITNESS_IN_STRUCTURED_TEMPLATE` 또는 limitation이며
`SPATIALLY_INFEASIBLE`, `TEMPORALLY_INFEASIBLE`이나 `NO_SAFE_SOLUTION`의 일반 증명이 아니다.

## 4. 입력과 정보 격리

PASS 검색이 받을 수 있는 입력은 다음뿐이다.

```text
WitnessWorldSnapshot
WitnessSearchConfig
ManeuverConstraintSpec  # world 안에 content-hash로 결박
```

허용 정보:

- grid, static·forbidden cells, 선택적으로 명시된 allowed cells
- reference polyline, 시작 pose·twist, goal, episode duration
- 가상 차체 profile과 운동학 계약
- opaque Actor binding, radius와 exact time-indexed ground-truth trajectory
- `passing_policy`

금지 정보:

- episode category, family, orientation label과 progressable label
- evaluator oracle·기대 결과·기존 수동 witness
- DWA·DWB·PP command, critic score, controller result
- hidden split·seed·ID 또는 hidden 결과에서 유도한 수치

검색 모듈은 `dynamic_corpus`, `dynamic_evaluation`, controller와 hidden runner를 import하지
않는다. 원본 관리 ID 대신 projection에서 생성한 opaque binding과 content hash만 사용한다.

## 5. 좌표계와 사건의 정확한 정의

### 5.1 Reference 좌표

reference segment의 시작점과 끝점을 `q0`, `q1`이라 한다.

```text
t = (q1-q0) / norm(q1-q0)
n_left = (-t.y, t.x)
s = segment 시작부터 t 방향 progress
d = dot(robot_position-reference_projection, n_left)
```

- `d > 0`: `PASS_LEFT`
- `d < 0`: `PASS_RIGHT`
- 같은 최소거리 projection이 인접한 collinear segment에 있고 tangent가 같으면 작은 segment
  index, 작은 progress 순으로 선택한다. 비인접 segment 또는 tangent가 다른 동률은
  `ambiguous_reference_projection`으로 거부한다.
- departure·pass·rejoin anchor는 모두 같은 non-zero reference segment에 있어야 한다.
- 코너나 다른 segment로 넘어가는 pass는 생성하지 않는다.

### 5.2 Departure

실제 pose의 reference distance가 처음 `> 0.10 m`가 된 sample을 departure로 판정한다.
`0.10 m`와 같은 값은 departure가 아니다. phase 문자열이나 선언 시각만으로 인정하지 않는다.

### 5.3 Target Actor

R2-PASS v1 후보 하나는 target Actor 한 명만 결박한다. 후보 target은 정답 label이 아니라
다음 ground-truth 기하조건으로 생성한다.

1. 실제 candidate departure tick에 Actor가 active다.
2. Actor reference progress가 로봇보다 앞에 있다.
3. 종방향 간격이
   `wheelchair_length/2 + actor_radius`보다 크다.
4. Actor의 signed reference offset 절댓값이
   `wheelchair_width/2 + actor_radius + 0.08 m` 이하라 기준 lane과 겹친다.
5. Actor 속도 크기가 `> 1e-6 m/s`이고 local tangent 방향 성분이 양수다.
6. Actor 속도와 local tangent의 방향 오차가 `<= 10 deg`다.
7. candidate의 공통 linear target이 Actor의 tangent 속도보다 `> 1e-9 m/s` 크다.
8. Actor가 ordered pass 판정 시각까지 active 상태를 유지한다.

target 후보는 opaque Actor binding 순으로 생성한다. target이 아닌 다른 Actor도 모든 충돌검사에
포함하며, 다른 Actor 때문에 unsafe한 후보는 거부한다. departure부터 planned rejoin 구간에
같은 방향으로 기준 lane을 막는 Actor가 추가로 들어오면 한 명만 임의로 선택하지 않고
`multi_actor_pass_out_of_scope`로 거부한다. 복수 Actor를 모두 추월해야만 완료되는 기동은
R2-PASS v1 범위 밖이다. 정지·정면·횡단·대각 Actor가 단순 order 역전을 만들더라도 target으로
선택하지 않는다.

### 5.4 Ordered overtake

각 200 Hz ground-truth sample에서 다음을 계산한다.

```text
order = actor_reference_progress - robot_reference_progress
longitudinal_extent = wheelchair_length/2 + actor_radius
```

target Actor가 active인 동안:

```text
order > +longitudinal_extent
→ departure 발생
→ order < -longitudinal_extent
```

순서가 관측돼야 pass로 인정한다. Actor 소멸, ID 변경, declaration만으로 pass를 만들지 않는다.
또한 pass 시점의 로봇 progress는 departure 시점보다 최소 `0.10 m` 증가해야 한다. order 변화가
Actor의 이동만으로 발생한 경우는 추월이 아니다. pass 뒤 Actor가 여전히 active라면 rejoin
confirmation 또는 Actor 종료 중 먼저 발생하는 시각까지 로봇이 앞선 순서를 유지해야 한다.

### 5.5 Side 유지

- departure를 처음 만든 signed offset의 부호가 witness kind와 일치해야 한다.
- departure부터 마지막 ordered pass까지 선택 side의 signed offset 절댓값은 계속
  `> 0.10 m`여야 한다.
- 그 구간의 반대쪽 excursion은 numeric tolerance `1e-9 m`만 허용한다.
- 실제 pass event sample의 부호도 witness kind와 일치해야 한다.
- pass 뒤 rejoin 과정에서는 중심선으로 접근할 수 있지만 반대쪽으로 `1e-9 m`보다 넘어가면
  wrong-side candidate다.

따라서 잠깐 왼쪽으로 departure한 뒤 실제로 오른쪽에서 추월한 궤적은 `PASS_LEFT`로 인정하지
않는다. 경로 전체의 `maximum_left/right_offset`만으로 side를 판정하지 않는다.

### 5.6 Rejoin

ordered pass 이후 다음을 모두 연속 `>= 0.50 s` 만족해야 한다.

```text
reference distance <= 0.10 m
reference tangent heading error <= 10 deg
```

rejoin은 pass 이전에 시작할 수 없다. terminal pose에서는 실제 선·각속도가 정지 상태여야 하며
같은 pose의 `0.50 s` dwell을 별도로 확인한다.

## 6. Passing policy와 허용 영역

- `PROHIBITED`: PASS geometry 후보를 만들지 않는다. 정상 음성 결과다.
- `ALLOWED`: PASS 후보를 생성하고 ground-truth feasibility를 판정한다.
- `UNSPECIFIED`: offline 기하·시간 후보는 생성할 수 있지만 결과에
  `passing_policy_unspecified` limitation을 남긴다. 이는 online 통행 허가가 아니다.
- `allowed_cells`가 비어 있지 않으면 모든 footprint와 swept footprint가 해당 영역 안에 있어야
  한다.
- static free이더라도 forbidden cell은 절대 통과할 수 없다.
- category나 복도 이름으로 passing policy를 추론하지 않는다.

## 7. Structured geometry 후보

후보 parameter는 다음과 같다.

```text
target_actor_binding
side                    # LEFT / RIGHT
reference_segment_index
departure_release_tick
departure_progress_m
lateral_offset_m
common_linear_target_mps
common_angular_magnitude_radps
side_wait_policy        # IMMEDIATE / UNTIL_TARGET_INACTIVE
```

`pass_target_progress_m`와 `rejoin_progress_m`는 전조합 축으로 만들지 않는다. ordered pass는
처음 `order < -longitudinal_extent`가 된 시각에 기록하되 그 즉시 감속하지 않는다. 매 이후
20 Hz candidate state에서 제한감속, 선택 side wait, `TURN_RETURN`, `MOVE_TO_REFERENCE`,
`ALIGN_REFERENCE`, `REJOIN_DWELL` suffix를 결정론적으로 합성·예측한다. rejoin confirmation까지
target Actor가 active인 모든 sample에서 로봇의 앞선 순서와 hard clearance가 유지되는 첫
suffix-safe stop point의 다음 command부터 제한감속을 시작한다. 완전 정지한 pose의 reference
projection을 같은 segment 안의 rejoin progress로 사용한다. suffix-safe stop point가 없으면
해당 후보는 dynamic reject다. 이렇게 해야
`departure × offset × pass × rejoin` 조합 폭발로 공개 positive가 평가 전에 resource limit에
걸리는 것을 막을 수 있다.

### 7.1 결정론적 grid

- progress step: `max(grid_resolution, 0.10 m)`
- lateral step: `grid_resolution`
- departure progress 범위: 시작 robot progress부터 target Actor active interval의 최소 progress에서
  `wheelchair_length/2 + actor_radius`를 뺀 지점까지
- lateral offset 시작값: Actor 중심의 signed offset에 선택 side 방향으로
  `actor_radius + wheelchair_width/2 + 0.08 m`를 더한 robot center offset 이후의 첫
  cell-center. centered Actor와 차체에서는 `0.18 + 0.18 + 0.08 = 0.44 m`다.
- lateral offset 끝값: oriented footprint와 `0.08 m` static clearance를 유지하는
  map·allowed-region 경계
- 시작·끝 여유: oriented footprint와 `0.08 m` clearance가 segment 끝을 넘지 않는 범위
- side 순회: `LEFT`, `RIGHT`
- 모든 수치는 작은 값에서 큰 값, Actor binding과 segment index는 정렬 순서로 순회한다.

현재 차체 폭 `0.36 m`, Actor 지름 `0.36 m`, 세 clearance `0.08 m`를 모두 요구하는 centered
straight corridor의 cross-section sanity bound는 `0.96 m`다. v6 narrow `0.92 m`는 이
필요조건보다 `0.04 m` 좁다. 90° 제자리회전에서는 차체 길이 절반과 clearance를 합한
`0.30 m`의 중심-벽 여유가 필요하다. 이 analytic bound는 빠른 음성 pruning 근거일 뿐,
`width >= 0.96 m`를 충분조건이나 PASS 정답으로 사용하지 않는다.

기존 수동 witness의 `0.70 m`, 약 `0.80 m`, 특정 tick 또는 command sequence를 정답으로
주입하지 않는다. 이 값들은 grid가 자연스럽게 포함할 수 있는 후보일 뿐이다.

### 7.2 Static geometry pruning과 합성 뒤 geometry rejection

timing 후보를 만들기 전에는 endpoint가 이미 정해진 다음 항목만 oriented swept footprint로
검사한다.

```text
reference prefix
TURN_OUT
MOVE_LATERAL
TURN_ALONG_REFERENCE
선택 lateral lane의 straight envelope
```

timed candidate 합성 뒤 실제 endpoint가 정해지면 다음을 별도로 검사한다.

```text
MOVE_PAST
제한감속 후 stop pose
TURN_RETURN
MOVE_TO_REFERENCE
ALIGN_REFERENCE
REJOIN_DWELL
```

어느 단계든 static·forbidden·allowed-region을 위반하면 해당 timed candidate를
`geometry_pruned`로 분류한다. pre-timing candidate count에는 raw geometry 후보를, 최종
`generated_count`에는 fully specified timed candidate를 사용한다. fast pruning과 post-synthesis
검사는 final validator를 대체하지 않으며 통과 후보는 반드시 독립 validator에서 다시 검사한다.

departure·pass·rejoin projection이 하나의 같은 straight segment에 결박되지 않거나 corner,
self-intersection, equidistant projection 때문에 segment 선택이 모호하면
`ambiguous_reference_projection`으로 제거한다.

## 8. 20 Hz 운동학 합성

### 8.1 Phase 순서

```text
optional FOLLOW_REFERENCE prefix
→ BRAKE_TO_STOP
→ TURN_OUT
→ MOVE_LATERAL
→ BRAKE_TO_STOP
→ TURN_ALONG_REFERENCE
→ ordered pass 뒤 suffix-safe stop point까지 MOVE_PAST
→ BRAKE_TO_STOP
→ IMMEDIATE 또는 UNTIL_TARGET_INACTIVE SIDE_WAIT
→ TURN_RETURN
→ MOVE_TO_REFERENCE
→ BRAKE_TO_STOP
→ ALIGN_REFERENCE
→ REJOIN_DWELL
→ optional FOLLOW_REFERENCE + GOAL_DWELL
```

- 방향을 바꾸기 전에는 실제 `v=0`, `w=0`을 거친다.
- in-place turn은 `v=0`, `w!=0`으로 허용한다.
- reverse는 비활성이다.
- 현재 twist로 pose를 적분한 뒤 다음 tick twist를 갱신하는 기존 20 Hz convention을 유지한다.
- 각 phase의 overshoot, phase cycle, non-finite와 episode duration 초과는 후보 무효다.
- `REJOIN_DWELL`까지가 PASS feasibility witness다. goal까지의 suffix는 best PASS를 고른 뒤
  별도 diagnostic replay로 만들 수 있지만 PASS objective와 존재 판정을 바꾸지 않는다.

### 8.2 동결 target

```text
linear target [m/s] : 0.10, 0.15, 0.20, 0.25, 0.30
angular target [rad/s]: -0.80, -0.60, -0.40, 0, 0.40, 0.60, 0.80
control period       : 0.05 s
maximum angular accel: 1.60 rad/s^2
reverse              : false
synthesis pose tolerance: 0.025 m
synthesis heading tolerance: 0.025 rad
goal tolerance       : 0.05 m
```

선속도·선가감속과 최대 각속도는 `virtual_doll_wheelchair_v0_1` profile을 따른다. 이 수치는
simulation 연구값이며 실제 휠체어 차체값이 아니다.

candidate 하나는 모든 translation phase에 공통 linear target 하나를 사용한다. 모든 turn은
공통 angular magnitude `{0.40, 0.60, 0.80}` 중 하나를 사용하며 부호는 목표 heading의
shortest turn 방향으로 geometry가 결정한다. 양·음 각속도를 phase마다 독립 조합하지 않는다.

## 9. 시간 후보와 안전 guard

command와 wait parameter는 `0.05 s` tick으로만 생성한다.

후보 anchor:

```text
release_ticks = sorted(unique(
  0,
  ceil(active_from_s / 0.05),
  floor(active_until_s / 0.05) + 1,
  ceil(exact_lane_entry_time_s / 0.05),
  ceil(exact_lane_exit_time_s / 0.05)
))
```

위 식은 모든 target·non-target Actor에 적용하고 episode 범위 밖 tick을 제거한다. reference
prefix와 departure pose의 실제 정지를 먼저 완료한 뒤 release tick까지 그 pose에서 `v=w=0`으로
hold하고, release tick 이후 `TURN_OUT`을 시작한다. 실제 departure event는 이후 측면 이동에서
reference distance가 `>0.10 m`가 된 순간이다. side-wait 시작·종료는 합성 결과이며 release tick
집합에 넣지 않는다.

raw Actor event time 자체를 command timestamp로 사용하지 않는다. command anchor는 위 규칙으로
control tick에 올림한다. 반대로 독립 validator는 raw event를 반올림하지 않고 원래 float
시각 그대로 evaluation sample에 추가한다. 따라서 50 ms command grid와 5 ms grid 밖의 정확한
Actor 출현·소멸 판정을 섞지 않는다. `active_until_s`는 inclusive이므로 그 시각 이후에만
발생한 order 역전은 pass가 아니다.

side wait policy는 두 개로 제한한다.

- `IMMEDIATE`: suffix-safe stop point에서 완전제동 뒤 wait `0` tick, 즉시 rejoin
- `UNTIL_TARGET_INACTIVE`: suffix-safe stop point에서 완전제동 뒤 target의 inclusive
  `active_until_s`보다 뒤인 첫 control tick까지 `v=w=0` hold

suffix-safe 판정은 명시된 wait 시간을 포함한다. side wait 중 Actor가 다시 앞서거나 clearance가
깨지는 후보는 `UNTIL_TARGET_INACTIVE`라는 이유로 허용하지 않는다.

각 이동 tick 전 fast guard는 현재 운동, 제한감속과 정확한 Actor 원을 사용해 terminal stopping이
가능한지 보수적으로 확인한다. ordered pass 전 guard 실패는 제한제동 가능 여부만 확인한 뒤
candidate를 dynamic reject한다. ordered pass 뒤 `IMMEDIATE`의 return guard가 실패해도 암묵적
wait로 바꾸지 않고 reject한다. `UNTIL_TARGET_INACTIVE`만 명시된 tick까지 hold한 뒤 return을
검사한다. guard와 final validator는 별도 구현이어야 한다. 검색기는 20 Hz candidate state만
사용하고 200 Hz ordered-event 판정은 독립 validator에서만 수행한다.

## 10. 후보 수, resource와 실행 방식

구현은 전체 candidate·witness를 메모리에 쌓지 않는다.

1. geometry 축의 곱으로 geometry candidate 수를 먼저 계산한다.
2. fully specified candidate 수를 다음 축의 곱으로 계산한다.

```text
target × side × departure_progress × lateral_offset
× departure_release_tick × common_linear_target
× common_angular_magnitude × side_wait_policy
```

3. target보다 느리거나 같은 linear target과 중복 effective tick은 count 전에 제거한다.
4. 사용자에게 대표 episode의 예상 후보 수와 대략적인 실행시간을 먼저 알린다.
5. limit을 넘으면 witness materialization 전에 `RESOURCE_LIMIT`을 반환한다.
6. 후보는 frozen 순서로 streaming 생성한다.
7. 종류별 best와 count·reason diagnostics만 보존한다.

동결 limit:

```text
max_geometry_candidates_per_episode = 50_000
max_timed_candidates_per_episode    = 250_000
max_points_per_candidate            = episode_tick_count + 1
```

episode 하나 안의 후보 평가는 결정론적 직렬 순서를 유지한다. 서로 독립적인 공개 episode는
process 기반으로 병렬 실행할 수 있으며 결과를 corpus 입력 순서로 재정렬한다. wall-clock은
semantic hash와 합격 판정에서 제외한다. timing benchmark가 아니므로 CPU contention 시간으로
알고리즘 자격을 판단하지 않는다.

limit으로 전체 objective를 끝까지 평가하지 못하면 그 전에 valid candidate를 찾았더라도 final
selected PASS로 봉인하지 않는다. `generated_count`는 fully specified timed candidate 단위로
유지하고 geometry raw count와 세부 reason histogram은 별도 diagnostics로 둔다. watchdog,
worker crash와 I/O 실패는 `RESOURCE_LIMIT`이 아닌 infrastructure failure다.

## 11. 결과 계약

### 11.1 종류별 결과 보존

기존 단일 `WitnessSearchResult.selected_witness`를 억지로 확장하지 않는다. PASS는 별도
`search_pass_structured()` API와 다음 의미의 결과를 추가한다.

```text
PassStructuredSearchResult
  source_projection_hash
  world_content_hash
  vehicle_profile_hash
  maneuver_policy_hash
  maneuver_policy_revision
  search_config_hash
  pass_search_version
  validator_version
  best_pass_left: AutomatedWitness | null
  best_pass_right: AutomatedWitness | null
  objective_by_side: LEFT/RIGHT -> WitnessObjective | null
  validation_hash_by_side
  status_by_side: LEFT/RIGHT -> WitnessSearchStatus
  reason_by_side: LEFT/RIGHT -> stable reason code
  count_by_side: LEFT/RIGHT -> generated/geometry_pruned/dynamic_rejected/validated
  limitations: sorted unique tuple[str, ...]
  semantic_content_hash
```

기존 `search_wait_and_hold()`와 `WitnessSearchResult`는 회귀 API로 보존한다. 후속 공개
audit은 다음처럼 두 결과를 합성하되 원본을 잃지 않는다.

```text
StructuredWitnessAuditBundle
  pass_result
  wait_hold_result
  semantic_content_hash
```

PASS 존재 판정은 `best_pass_left` 또는 `best_pass_right`의 독립 validator 통과 여부로 한다.
전역으로 한 개를 고르는 convenience field가 있더라도 taxonomy·완료조건에는 사용하지 않는다.

### 11.2 종류 내부 objective

같은 kind 안에서만 다음 오름차순 objective를 적용한다.

```text
1. hard validation failure count
2. terminal completion time
3. actual path length
4. maximum reference deviation
5. full-stop count
6. absolute angular travel
7. frozen parameter tuple
8. witness semantic content hash
```

LEFT와 RIGHT 사이를 제품 우선순위로 비교하지 않는다. 정확히 대칭인 analytic world에서는
valid candidate set이 `LEFT↔RIGHT`로 대응하고 metric이 같음을 확인한다. semantic hash가
물리적 선호를 뜻하지 않으므로 단일 selected side의 mirror-equivariance는 요구하지 않는다.

### 11.3 Config와 provenance

PASS에서 새로 사용하는 모든 threshold, phase tolerance, candidate 축과 limit은 hashed config에
반드시 포함한다. 기존 `WitnessSearchConfig`를 확장하고 config version을 올리며 과거 result
hash를 재사용하지 않는다. 구현 코드의 private 상수로 search 의미를 숨기지 않는다.

단, search 효율과 합성 방법을 결정하는 값과 독립 hard validator의 의미를 섞지 않는다.

```text
WitnessSearchConfig에 저장·hash:
- progress/lateral grid
- linear/angular target
- synthesis pose/heading tolerance
- release/wait policy와 candidate/resource limit

validator-v2의 불변 hard semantic:
- minimum ground-truth clearance 0.08 m
- departure threshold > 0.10 m
- rejoin distance <= 0.10 m
- rejoin heading error <= 10 deg
- rejoin/terminal dwell >= 0.50 s
- exact Actor event와 ordered pass 의미
```

hard semantic은 search config에 중복 저장하지 않고 `WITNESS_VALIDATOR_VERSION`으로 결박한다.
따라서 현재 `validate_ground_truth_witness(world, witness)` API는 search config 값을 읽지 않고도
독립성을 유지한다. hard semantic을 바꾸려면 validator version과 상위 명세를 먼저 개정해야
하며, 개발 episode별 custom threshold는 허용하지 않는다.

최소 version 변경:

```text
WITNESS_SEARCH_CONFIG_VERSION bump
WITNESS_VALIDATOR_VERSION bump
PASS_STRUCTURED_SEARCH_VERSION 신규
```

각 result는 최소한 다음을 결박한다.

```text
source projection hash
world hash
vehicle profile hash
maneuver policy hash/revision
search config hash/version
validator version
selected validation hash by side
candidate count and termination reason by side
```

elapsed time, worker 번호와 process 완료 순서는 semantic hash에서 제외한다.
`limitations`는 정렬·중복 제거한 뒤 semantic hash에 포함한다.

episode 전체 geometry/timed limit은 LEFT·RIGHT 합산 fully specified candidate에 적용한다.
preflight limit을 넘으면 양쪽 status를 모두 `RESOURCE_LIMIT`, count를 모두 `0`, best와
objective·validation hash를 `null`로 기록한다. `objective_by_side`는 선택된 canonical witness를
만든 frozen parameter tuple과 독립 validator metric을 포함하며 semantic hash에 들어간다.

candidate ID에는 source/world/config hash, 전체 frozen parameter tuple과 정렬된 required Actor
binding을 포함한다. canonical event 절차는 다음 두 단계로 고정한다.

1. measurement validation은 draft witness의 event declaration 부재를 허용하고 물리·운동학·
   clearance·ordered event를 측정한다.
2. 측정한 departure/pass/rejoin event를 채운 canonical witness를 생성한다.
3. strict validation은 선언값을 필수로 요구하고 측정값과 `0.005 s` 이내 일치를 검사한다.
4. strict 결과만 `validated_count`와 최종 validation hash에 사용한다.

mission, `stop_epoch`, path revision은 offline R2 결과에 임의로 추가하지 않는다.

### 11.4 Exhaustive rejection taxonomy

failure 문자열 포함 여부로 bucket을 고르지 않는다. 모든 known failure code를 다음처럼
명시적으로 exhaustive mapping한다.

- `INVALID_INPUT`: config/source/world hash, non-finite, profile limit, degenerate reference
- `GEOMETRY_PRUNED`: prohibited policy, ambiguous projection, invalid progress·offset, eligible
  same-direction target 없음, static·forbidden·allowed-region, duration·kinematic synthesis 실패
- `DYNAMIC_REJECTED`: Actor clearance, terminal stopping, target inactive/not-ahead,
  `ordered_overtake_missing`, wrong-side, event ordering, pass 뒤 재역전, rejoin 실패
- `VALIDATED`: canonical PASS witness의 두 번째 독립 validation 통과
- `RESOURCE_LIMIT`: geometry 또는 fully specified timed candidate preflight 초과
- `INFRASTRUCTURE_FAILURE`: watchdog, worker crash, I/O. 불가능 search status로 바꾸지 않음

각 generated timed candidate는 정확히 하나의
`geometry_pruned | dynamic_rejected | validated` bucket에 들어가며 합은 `generated_count`와
같아야 한다. preflight `RESOURCE_LIMIT`에서는 후보를 일부 생성한 것처럼 count하지 않는다.

## 12. 독립 검증 계약

모든 PASS candidate는 검색 code path와 독립된 `validate_ground_truth_witness()`를 통과해야
한다. validator는 candidate grid·pruning·objective를 알지 못한다.

필수 hard validation:

1. schema·hash·finite·시작 state
2. 20 Hz timestamp와 current-twist 운동학
3. 선·각속도, 선·각가감속과 정지 뒤 방향 변경
4. 200 Hz oriented footprint static·forbidden·allowed-region
5. raw Actor event 시각을 포함한 exact Actor circle clearance `>= 0.08 m`
6. 선언이 아닌 실제 departure `> 0.10 m`
7. departure 시 target Actor active·ahead·same-direction·lane-overlap
8. target Actor가 active인 동안 ordered overtake와 robot progress `>= 0.10 m`
9. departure부터 pass까지 PASS kind와 실제 signed side가 계속 일치
10. pass 뒤 Actor가 active한 동안 재역전 없음
11. 모든 required pass Actor 뒤에 rejoin 시작
12. reference 거리·heading 조건의 연속 `>= 0.50 s`
13. terminal 실제 정지와 같은 pose dwell `>= 0.50 s`
14. episode duration 안 종료. `end == duration`은 허용하고 `end > duration`은 실패
15. strict validation에서 선언 event 시각과 측정 event 시각 `<= 0.005 s` 일치

하나라도 실패한 candidate는 best에 들어갈 수 없다.

같은 stationary aligned interval이 sustained rejoin `0.50 s`와 terminal dwell `0.50 s`를 동시에
만족할 수 있다. 두 predicate는 독립 확인하지만 시간을 합산해 별도 `1.0 s`로 요구하지 않는다.

## 13. 판정 규칙

### PASS evidence found

```text
best_pass_left 또는 best_pass_right 존재
AND 해당 독립 validation passed
AND departure < ordered overtake < rejoin confirmation
```

이는 ground-truth structured-template positive다. online 실행 가능이나 제품 채택이 아니다.

### 정상 음성 결과

- `passing_policy=PROHIBITED`여서 후보를 생성하지 않음
- 해당 side의 static·forbidden·allowed-region geometry가 모두 막힘
- target Actor가 active인 동안 ordered pass를 완료할 timing 후보가 없음
- episode duration 안 rejoin·dwell을 완료하지 못함
- non-target Actor 때문에 모든 후보가 dynamic reject됨

### `SEARCH_INCONCLUSIVE`

- candidate/resource limit 도달
- target을 한 명으로 제한한 template 밖 multi-Actor pass가 필요함
- corner·multi-segment·reverse·smooth curve pass만 가능한 것으로 보임
- structured template에서 PASS가 없지만 일반 공간 불가능 증명은 없음

### Hard failure

- label·oracle·기존 witness·controller·hidden 누출
- selected PASS의 independent validator 실패
- prohibited/forbidden에서 PASS false positive
- Actor 소멸을 overtake로 기록
- side, event 순서, declaration과 측정값 불일치
- collision, clearance, kinematic, provenance 또는 hash 위반
- 같은 input/config의 semantic 결과 비결정성
- partial 실행을 final evidence로 사용하거나 기존 output 덮어쓰기

## 14. 공개 시험 matrix

### 14.1 첫 표적시험

첫 구현시험은 v6 public `same-direction-wide-r00`, Ideal ground truth 한 건으로 제한한다.

합격조건:

- label·oracle 없이 PASS 후보 생성
- 좌·우 중 적어도 하나가 independent validator 통과
- 실제 departure `>0.10 m`
- Actor active 중 ordered overtake
- overtake 뒤 sustained rejoin `>=0.50 s`
- collision·forbidden·clearance·kinematic failure `0`
- 같은 입력을 두 번 실행한 semantic result 동일

### 14.2 적대·음성시험

1. 좌우 mirror analytic world에서 후보 집합·종류별 objective mirror 동등
2. 한쪽만 static wall로 막으면 반대쪽만 valid
3. 양쪽이 막히면 PASS 없음, 일반 infeasible로 오분류하지 않음
4. `passing_policy=PROHIBITED`에서 candidate `0`
5. allowed-region 밖 측면 통과 거부
6. `0.10 m`와 같은 이탈은 departure로 인정하지 않음
7. 정지·정면·횡단·대각·뒤에서 접근하는 Actor를 same-direction target으로 선택하지 않음
8. Actor가 order 역전 전에 사라지면 pass 거부
9. overtake 전 rejoin, rejoin 없는 pass, dwell 부족과 pass 뒤 재역전 거부
10. 잠깐 왼쪽 departure 뒤 오른쪽 통과 같은 side-label 위조 거부
11. 20 Hz endpoint 사이와 raw off-grid Actor event 충돌 거부
12. target binding 변경·중복·tamper 거부
13. non-target 두 번째 Actor와 충돌하거나 추가 co-direction pass가 필요한 후보 거부
14. corner·multi-segment만 가능한 입력은 structured no-witness
15. `end == duration` 통과와 `end == duration + 0.05 s` 실패
16. geometry·timed resource의 limit-1, exact-limit, limit+1 구분
17. candidate 순서·process 완료 순서가 종류별 best를 바꾸지 않음
18. WAIT가 더 짧아도 valid PASS evidence가 합성 audit bundle에 보존됨

### 14.3 공개 확대시험

- same-direction-wide 5개: 적어도 한 side의 PASS 자동 발견과 validator pass
- same-direction-narrow: PASS false positive `0`
- offset head-on·diagonal·vertical·corner: 우연한 side deviation을 ordered pass로 오인하지 않음
- simultaneous·staggered·second-risk: non-target Actor 위험을 무시하지 않음
- legacy mechanism golden: 기존 WAIT/HOLD 결과를 PASS 추가가 손상하지 않음

legacy의 `LOCAL_DETOUR_FEASIBLE` crossing Actor는 R2-PASS v1의 same-direction positive가 아니다.
해당 사례에서 PASS를 찾지 못하는 것은 v1 scope limitation이며, 기존 WAIT/HOLD 회귀만 검사한다.

공개 결과를 본 뒤 clearance, witness threshold, grid step, target Actor 선택 규칙을 episode별로
바꾸지 않는다.

## 15. 구현 파일 경계

권장 변경 범위:

```text
src/hospital_path_lab/dynamic_witness_contracts.py
src/hospital_path_lab/dynamic_witness_pass.py          # 신규 권장
src/hospital_path_lab/dynamic_witness_search.py
src/hospital_path_lab/dynamic_witness_validation.py    # 필요한 독립 검증만
tests/test_dynamic_witness_pass.py                     # 신규 권장
tests/test_dynamic_witness_search.py
tests/test_dynamic_witness_validation.py
```

- `dynamic_witness_pass.py`는 `WitnessWorldSnapshot + frozen config`만 받는다.
- 기존 WAIT/HOLD API와 회귀 의미를 유지한다.
- corpus·evaluator·controller·runner를 PASS module에 연결하지 않는다.
- reporting·profile replay·full runner는 이번 구현 묶음에서 시작하지 않는다.

## 16. 실행 순서와 중단 조건

새 [임시 연구 하네스 실행 규칙](../../../AGENTS.md)에 따라 다음 순서를 사용한다.

1. 이 명세 사용자 검토·동결
2. 계약·projection·종류별 bundle 단위시험
3. `same-direction-wide-r00` 표적 positive
4. mirror·narrow·prohibited 적대시험
5. 읽기 전용 독립 감사와 수정
6. 공개 same-direction-wide 5개 process 병렬시험
7. 직접 영향권 회귀
8. R2의 PASS·WAIT/HOLD 공개 확대시험
9. R2 코드 동결 뒤 마지막 전체 회귀 한 번

처음부터 전체 공개나 전체 회귀를 돌리지 않는다. timeout 또는 장기 실행 뒤 같은 명령을 그대로
재실행하지 않고 후보 수·병목을 먼저 확인한다. wall-clock timing qualification은 이 단계의
판정 대상이 아니다.

다음 상황에서는 구현을 중단하고 보고한다.

- parent R2 contract와 종류별 결과 보존을 함께 만족할 수 없음
- public positive가 current hard validator에서 재현되지 않음
- policy/allowed-region 없이 category에서 통과 허용을 추론해야 함
- pass target을 label·oracle·기존 witness에서 가져와야만 함
- candidate limit을 지키면서 streaming 평가를 구성할 수 없음
- final validator를 search fast guard와 공유해야만 성능이 나옴

## 17. 이 구현 묶음의 완료조건

- 사용자 승인된 frozen 상세 명세와 구현·시험 연결
- PASS module의 label·oracle·controller·hidden 비의존성
- kind별 best 결과를 보존하고 WAIT가 PASS evidence를 지우지 않음
- same-direction-wide 5개에서 ground-truth PASS 자동 발견
- narrow·prohibited·forbidden false-positive `0`
- ordered departure→overtake→rejoin→dwell hard validation failure `0`
- mirror·determinism·resource·tamper 적대시험 통과
- 기존 WAIT/HOLD 회귀 유지
- 직접 영향권, Ruff, compileall 통과
- 독립 읽기 전용 감사에서 P0/P1 없음
- 코드 동결 뒤 마지막 전체 회귀 통과
- hidden·profile replay·controller·gate·제품 결정을 변경하지 않음

이 완료조건을 충족해도 R2 전체는 끝나지 않는다. 다음 구현 묶음은 current R1
`FUNCTIONAL_IDEAL/NORMAL/STRESS` profile replay이며, 그 뒤 공개 13+6 영구 audit,
JSON·PNG·process-parallel runner를 수행한다.
