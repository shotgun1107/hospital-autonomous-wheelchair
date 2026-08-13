# R2 — Feasible Witness 자동화·일반화 상세 명세

## 1. 상태와 목적

- 작성일: `2026-08-13`
- 상태: 상세 명세 동결, 2차 구현 checkpoint(WAIT/HOLD structured search),
  R2-PASS 구현 전 상세 명세 작성, R2 미완료
- 상위 단계: [R1~R7 Master Specification](10-dynamic-local-maneuver-research-master-spec.md)
- 선행 gate: `R1 완료`
- 실행 범위: Python `simulation_only`, 공개 corpus만 사용
- hidden: 생성·열람·실행 금지
- 제품 알고리즘 채택, `G1~G5`, 제품 경로분석 7단계: 미수행

R2의 목적은 기존에 사람이 command tick을 직접 적어 만든 feasible witness를 자동으로
탐색·재생·검증 가능한 공개 offline oracle로 바꾸는 것이다.

R2는 controller를 개선하거나 DWB의 점수를 튜닝하는 단계가 아니다. 먼저 다음 질문에
답한다.

> 동결한 가상 차체·지도·Actor 운동·안전조건에서 시간에 따라 안전한 통과 또는 대기
> witness가 실제로 존재하는가?

그리고 다음 세 사실을 분리한다.

```text
ground_truth_feasible
!=
prediction_or_observation_decidable
!=
online_controller_executable
```

- `ground_truth_feasible`: offline evaluator가 정확한 Actor 상태를 사용해 안전한 시간 궤적의
  존재를 확인함
- `prediction_or_observation_decidable`: 동결한 합성 관측과 prediction 계약에서 그 기동을
  판단·검증할 정보가 있음
- `online_controller_executable`: 실제 persistent controller와 shared gate가 해당 기동을
  연속 실행함

R2는 첫째와 둘째를 분리해 기록한다. 셋째는 `R5~R6` 대상이다.

`PASS_LEFT`·`PASS_RIGHT`의 구현 전 세부 계약은
[`R2-PASS 좌·우 통과 Witness 자동 탐색 상세 명세`](12-pass-structured-witness-search.md)를
따른다. 후보 종류별 결과 보존, target Actor 결박, 직선 segment template와 구현 순서는 해당
문서가 이 상위 명세를 구체화한다.

## 2. 현재 자료의 정확한 상태

### 2.1 기존 수동 witness

v6 공개 `same-direction-wide` 5개에는
`same-direction-wide-independent-v2` witness가 evaluator-only 자료로 들어 있다.

대표 witness의 현재 특성:

```text
duration: 44.05 s
points: 882
control grid: 20 Hz
maximum lateral deviation: 약 0.799953 m
maximum linear speed: 0.25 m/s
maximum angular speed: 0.80 rad/s
terminal dwell: 0.50 s
```

구조는 다음과 같다.

```text
정지 상태에서 90° 회전
→ 측면 이동
→ 원 경로 방향으로 회전
→ Actor 추월 방향으로 전진
→ Actor 활성 종료까지 측면에서 대기
→ 반대 회전·측면 복귀
→ 원 경로 방향 정렬
→ 목표 부근 전진·정지·terminal dwell
```

현재 validator는 다음을 검사한다.

- 시작·목표 pose와 시작 twist
- 20Hz timestamp
- differential-drive Euler 운동학
- 선·각속도와 선·각가감속
- 선속도 부호 반전 전 실제 정지
- 200Hz static·forbidden·Actor clearance 표본
- `0.10m` 초과 이탈
- 같은 방향 Actor의 ordered overtake
- 마지막 `0.50s` 정지 dwell
- episode duration 안의 종료

### 2.2 기존 witness가 증명하지 못하는 것

기존 witness는 다음 이유로 현재 R1 prediction이나 online controller의 증거가 아니다.

1. command sequence와 phase tick 수가 corpus generator 안에 수동으로 작성돼 있다.
2. same-direction 수평 직선 장면 전용이며 left/right·회전된 장면·다른 topology를 자동
   탐색하지 않는다.
3. Actor 검사는 당시 Normal·Stress의 `TTL + 1 tick = 0.35s` rollout-zero 원형 tube를
   ground-truth Actor 중심마다 배치한 방식이다.
4. 현재 R1 방향성 predictor의 20-frame warmup, confidence, exact Capsule과 전체 rollout
   시간축을 재생하지 않는다.
5. witness는 `t=0`부터 움직이지만 현재 online lane은 방향 확정과 보호정지 뒤 11개 새
   safe frame·현재 `stop_epoch` 재승인을 기다린다.
6. search space, resource limit와 `찾지 못함`의 의미가 없다.
7. 같은 장면에서 witness가 없을 때 공간 불가·시간 불가·관측 불충분을 구분하지 못한다.

따라서 R2는 기존 witness를 삭제하지 않고 `legacy evaluator witness`로 보존한다. 새 자동
탐색 결과와 의미·hash를 별도로 저장한다.

## 3. 입력과 정보 경계

## 3.1 공개 입력 집합

R2 primary evidence는 v6 공개 13개다.

| category | 개수 | R2 역할 |
|---|---:|---|
| `LOCAL_DETOUR_FEASIBLE` | 5 | 기존 positive witness 자동 재현·replica 일반화 |
| `LOCAL_DETOUR_FORBIDDEN` | 1 | 좁은 통로에서 false-positive pass 금지 |
| `WAIT_AND_RESUME` | 5 | crossing·head-on·corner·vertical·multi-Actor 대기 witness |
| `DYNAMIC_CHANGE_RESTOP` | 2 | 두 번째 위험 뒤 재정지·대기 sequence |

v6 공개에는 `NO_SAFE_SOLUTION`과 `OBSERVATION_INVALID`가 없다. taxonomy mechanism test에는
기존 legacy-v1 공개 `GOLDEN` 6개만 별도 사용한다.

- `NO_SAFE_SOLUTION`: 통로를 계속 막는 정적 Actor와 안전한 hold 공간
- `OBSERVATION_INVALID`: source invalid 뒤 복구
- 나머지 4개 category의 legacy golden: schema·분류 회귀

legacy-v1 자료는 R2 계약 시험용이며 v6 primary 성능 집계나 최신 제품 증거로 합치지 않는다.
전체 legacy development 30개는 R2 v1 완료조건에 포함하지 않는다.

## 3.2 Search 입력 projection

탐색기는 원본 episode를 직접 받지 않는다. 다음처럼 정답 정보를 제거한
`WitnessWorldSnapshot` projection만 받는다.

```text
schema_version
world_id
seed
simulation_only
map_id + map_revision + grid_content_hash
static occupancy + forbidden cells + allowed maneuver region
reference_path
initial robot state
goal pose
episode duration
vehicle_profile_ref + profile_content_hash
ground-truth Actor trajectories
public maneuver policy constraints
search_config_hash
```

ground-truth Actor trajectory는 offline search·oracle에만 허용한다. 이 projection은 online
controller 입력으로 재사용하지 않는다.

다음 필드는 projection에 포함하지 않는다.

```text
split
expectation_category
scenario_family
orientation label
variant
latent_case_id
oracle_spec
existing feasible_witness
progressable
blocking_cleared_at_s
controller ID 또는 critic 결과
hidden seed·commitment·결과
```

`allowed maneuver region`과 `no-passing` 같은 정책은 expectation label이 아니라 지도·운용의
명시적 공개 constraint여야 한다. 현재 corpus가 이를 label로만 표현한다면 구현 전에
research-only `ManeuverConstraintSpec`을 추가하고 source hash에 포함한다. category를 보고
constraint를 사후 생성해서는 안 된다.

## 3.3 Evaluator-only 비교 입력

search가 끝난 뒤 evaluator wrapper만 원본의 다음 정보를 사용한다.

- expectation category
- same-direction Actor ID
- hazard interval
- 기존 수동 witness
- departure·rejoin threshold
- corpus split과 mechanism-test 구분

이 정보는 검색 순서, 후보 점수나 종료조건에 영향을 주지 않는다. label을 바꿔도 search
semantic 결과가 같다는 적대 시험을 둔다.

## 4. 자료 계약

### 4.1 Search status

```text
WITNESS_FOUND
NO_WITNESS_IN_STRUCTURED_TEMPLATE
RESOURCE_LIMIT
INVALID_INPUT
```

- `WITNESS_FOUND`: 독립 validator까지 통과한 witness가 하나 이상 있음
- `NO_WITNESS_IN_STRUCTURED_TEMPLATE`: 동결한 R2 template 후보 전체를 검사했지만 없음
- `RESOURCE_LIMIT`: 동결 expansion·candidate limit에 도달해 결론을 못 냄
- `INVALID_INPUT`: schema·provenance·finite·map·profile 계약 위반

`NO_WITNESS_IN_STRUCTURED_TEMPLATE`과 `RESOURCE_LIMIT`을
`SPATIALLY_INFEASIBLE` 또는 `TEMPORALLY_INFEASIBLE`로 바꾸지 않는다. 일반 공간 완전성은 R3
bounded oracle 대상이다.

### 4.2 Witness kind

```text
PASS_LEFT
PASS_RIGHT
WAIT_AND_FOLLOW
HOLD_ONLY
```

- `PASS_LEFT/RIGHT`: reference 진행방향 기준 측면 이탈·통과·재합류
- `WAIT_AND_FOLLOW`: 실제 정지·대기 뒤 원 reference를 따라 진행
- `HOLD_ONLY`: episode 안에서 안전한 이동 해가 없거나 판단이 불충분해 정지를 유지

`HOLD_ONLY`는 임무 성공이 아니다. 안전하게 정지해 있을 수 있다는 증거다.

### 4.3 Witness point와 결과

```text
WitnessPoint
- time_s
- pose(x, y, yaw)
- twist(v, w)
- phase
- source_primitive_id

AutomatedWitness
- witness_version
- witness_id
- kind
- terminal_mode
- points
- departure_time_s
- pass_times_by_actor
- rejoin_started_at_s
- rejoin_confirmed_at_s
- terminal_dwell_s
- semantic_content_hash

WitnessSearchResult
- status
- world/source/search_config hash
- generated/geometry_pruned/dynamic_rejected/validated counts
- selected_witness or null
- termination_reason
- deterministic objective tuple
- validator version + selected validation hash
- elapsed_nonqualification_ns
- content_hash excluding wall-clock
```

`terminal_mode`은 다음을 구분한다.

```text
REJOIN_DWELL
GOAL_DWELL
SAFE_HOLD
```

R2의 local maneuver 증거에는 `REJOIN_DWELL`이면 충분하다. 기존 수동 witness 재현 회귀는
`GOAL_DWELL`도 검사한다. `REJOIN_DWELL`을 목적지 도착으로 기록하지 않는다.

## 5. R2 v1 자동 탐색 범위

### 5.1 Structured template를 사용하는 이유

R2 v1은 일반 State Lattice planner를 구현하지 않는다. 현재 수동 witness가 표현한 기동을
자동화하는 최소 구조를 먼저 사용한다.

```text
PASS template
TURN_OUT
→ MOVE_LATERAL
→ TURN_ALONG_REFERENCE
→ MOVE_PAST
→ optional DWELL
→ TURN_RETURN
→ MOVE_TO_REFERENCE
→ ALIGN_REFERENCE
→ REJOIN_DWELL
→ optional MOVE_TO_GOAL + GOAL_DWELL

WAIT template
BRAKE_TO_STOP
→ DWELL_UNTIL_CLEAR
→ ALIGN_REFERENCE
→ FOLLOW_REFERENCE
→ REJOIN_OR_GOAL_DWELL
```

이 template 밖의 curved pass, 복잡한 reverse, 여러 topology와 좁은 공간 pose search는
R3 또는 R4 대상이다. R2 template에서 찾지 못했다는 사실은 일반 해 부재가 아니다.

### 5.2 기하 후보 생성

모든 좌표는 reference polyline의 progress·tangent·normal 좌표계로 생성한다.

후보 축:

- side: `LEFT`, `RIGHT`
- departure progress
- lateral lane offset
- pass-complete progress
- rejoin progress
- terminal mode

생성 규칙:

1. departure·pass·rejoin progress는 reference 길이와 Actor 활성 구간의 progress 범위에서
   생성한다.
2. progress grid 기본 간격은 `max(grid_resolution, 0.10m)`다.
3. lateral offset은 grid cell 중심을 따라 `grid_resolution` 간격으로 생성한다.
4. candidate footprint가 static·forbidden·allowed-region 경계를 침범하면 timing 생성 전에
   제거한다.
5. 이탈은 departure threshold `0.10m`를 넘을 수 있는 offset만 pass 후보로 인정한다.
6. 재합류 pose는 reference까지 `0.10m`, tangent heading 오차 `10°` 안으로 들어와야 한다.
7. goal까지 남은 길이가 부족하면 `GOAL_DWELL` 후보를 만들지 않고 `REJOIN_DWELL`만
   평가한다.
8. left/right 모두 생성하고 첫 성공에서 중단하지 않는다.

`0.70m` 또는 기존 witness의 약 `0.80m`를 고정 offset으로 사용하지 않는다. 두 값은 search
grid 안의 후보일 수 있지만 정답으로 주입하지 않는다.

### 5.3 운동학 합성

각 기하 후보는 다음 동결 target 집합에서 command sequence를 합성한다.

```text
linear target [m/s]: 0.10, 0.15, 0.20, 0.25, 0.30
angular target [rad/s]: -0.80, -0.60, -0.40, 0, 0.40, 0.60, 0.80
control period: 0.05 s
reverse target: R2 v1 pass template에서 비활성
```

- 선속도는 가상 profile의 가속·감속 제한으로 target에 접근한다.
- 각속도는 기존 witness의 simulation-only `1.60rad/s²` 제한을 사용한다.
- 선속도 방향을 바꾸기 전에는 실제 `v=0`을 거친다.
- in-place turn은 허용하되 `v=0`, `w!=0`으로 명시한다.
- 각 phase는 target pose·heading tolerance 또는 episode 시간에 도달할 때 종료한다.
- current twist로 pose를 적분한 다음 다음 tick twist를 갱신하는 기존 20Hz convention을
  유지한다.
- non-finite, tolerance 밖 overshoot와 phase cycle은 후보 무효다.

이 수치는 모두 `virtual_doll_wheelchair_v0_1` 연구 profile 전용이며 실제 차체값이 아니다.

### 5.4 시간 후보와 Actor 사건

시간 탐색은 `0.05s` tick 단위다.

- 초기 hold tick
- 각 phase 사이의 실제 정지 tick
- 측면 pass lane에서의 dwell tick
- hazard 해소 뒤 follow 재개 tick
- multi-Actor 위험 사이의 re-stop tick

Actor active 시작·종료, reference 투영 순서가 바뀔 수 있는 시각과 observation readiness
변화는 event anchor로 기록하지만, 정답 category는 사용하지 않는다.

현재 구현된 WAIT/HOLD subset은 `t=0`, 실제 비terminal wait 최소값, 각 Actor의 활성 시작·종료와
그 다음 20 Hz tick을 결정론적 departure anchor로 사용한다. reference polyline의 각 segment를
20 Hz에서 가감속·정지·제자리 정렬하며 따라가고, 매 이동 후보에는 정확한 ground-truth Actor
원에 대한 terminal-stopping guard를 먼저 적용한다. 이 guard는 최종 판정기가 아니며, 생성된
witness는 아래 독립 200 Hz validator를 다시 통과해야 한다.

`HOLD_ONLY`는 짧은 terminal dwell이 아니라 `t=0`부터 episode 종료까지 같은 초기 pose에서
실제 정지를 유지해야 한다. `WAIT_AND_FOLLOW`는 terminal dwell과 별개의 실제 비terminal wait
뒤 reference progress가 최소 `0.10m` 더 증가해야 한다. 단순히 먼저 이동한 뒤 terminal 직전에
멈추는 궤적은 wait→follow witness로 인정하지 않는다.

ground-truth search는 Actor의 정확한 trajectory를 사용한다. observation-conditioned replay는
탐색 뒤 별도 단계에서 수행한다. noisy observation seed에 맞춰 ground-truth witness geometry를
튜닝하지 않는다.

### 5.5 결정론적 탐색·선택 순서

현재 WAIT/HOLD checkpoint는 후보를 모두 평가한 뒤 아래 objective를 오름차순으로 비교한다.
PASS를 추가할 때는 종류 간 단일 최종 1개만 남기지 않고 `PASS_LEFT`, `PASS_RIGHT`,
`WAIT_AND_FOLLOW`, `HOLD_ONLY`별 best를 각각 보존한다. 그렇지 않으면 짧은 WAIT가 안전한
PASS의 존재 증거를 지울 수 있다.

같은 kind 안에서는 다음 objective를 오름차순으로 비교한다.

```text
1. hard validation failure count
2. terminal completion time
3. actual path length
4. maximum reference deviation
5. full-stop count
6. absolute angular travel
7. frozen parameter tuple의 lexicographic order
8. witness semantic content hash
```

hard failure가 하나라도 있는 후보는 선택 대상이 아니다. 종류 간 운용 우선순위와 제품 선택은
R2가 결정하지 않는다. 기존 `WitnessObjective.kind_rank`는 WAIT/HOLD 회귀 호환 필드로 남길 수
있지만 PASS 존재·taxonomy 판정에는 사용하지 않는다.

같은 입력의 thread/process 완료 순서가 selected witness를 바꾸면 실패다.

## 6. Resource limit와 완전성 경계

R2 v1 기본 search config:

```text
max_geometry_candidates_per_episode = 50_000
max_timed_candidates_per_episode    = 250_000
max_points_per_candidate            = episode_tick_count + 1
maximum_episode_duration            = corpus에 동결된 값
candidate_storage                   = streaming, best+diagnostics만 보존
wall_clock_timeout                  = 판정에 사용하지 않음
```

- limit은 manifest와 result hash에 포함한다.
- limit 도달 시 `RESOURCE_LIMIT`이며 해 부재가 아니다.
- 실행시간이 길면 episode별 process 병렬화를 사용할 수 있지만 candidate 순서는 바꾸지
  않는다.
- 한 episode 내부 search·validation은 결정론적 직렬 순서다.
- wall-clock hang 방지 watchdog을 추가하더라도 `RESOURCE_LIMIT`과 별도 infrastructure failure로
  기록한다.
- public development 결과를 본 뒤 limit을 바꾸면 새 audit version·config hash로 전체 public을
  다시 실행한다. 좋은 episode에만 다른 limit을 적용하지 않는다.

R2 template는 완전한 pose-space 탐색이 아니다. `NO_WITNESS_IN_STRUCTURED_TEMPLATE`은 R3로
넘길 근거이지 일반적인 불가능 증명은 아니다.

## 7. 독립 검증

검색 중 빠른 충돌검사와 최종 validator를 같은 함수로 구현하지 않는다. 최종 validator는
검색 parameter와 objective를 알지 못하고 `AutomatedWitness + WitnessWorldSnapshot`만 받는다.

### 7.1 Ground-truth hard validator

모든 witness를 다음 기준으로 독립 재검증한다.

1. schema·source·world·vehicle·search hash와 finite
2. 시작 state와 첫 witness point 일치
3. timestamp가 `0.05s` 간격이고 엄격 증가
4. current twist 기반 differential-drive pose 적분
5. 선·각속도, 선·각가감속과 정지 뒤 방향 반전
6. 200Hz의 oriented footprint static·forbidden swept clearance
7. 200Hz의 실제 Actor circle surface clearance `>=0.08m`
8. departure `>0.10m`
9. Actor 존재 중 reference 투영 순서의 앞→뒤 역전
10. pass 이후 reference 거리 `<=0.10m`, heading 오차 `<=10°`
11. rejoin 상태가 연속 `>=0.50s`
12. terminal 실제 정지와 `>=0.50s` 동일 pose dwell
13. episode duration 안의 종료
14. `HOLD_ONLY`의 episode 전체 정지 유지
15. `WAIT_AND_FOLLOW`의 terminal dwell 제외 실제 wait와 그 뒤 `>=0.10m` 후속 progress

평가기 hard clearance는 online prediction shape가 아니라 정확한 ground-truth Actor circle을
사용한다. online predictor와 독립성을 유지한다.

### 7.2 Prediction·observation replay

ground-truth validator 통과 뒤 같은 witness를 profile별로 재생한다.

```text
FUNCTIONAL_IDEAL
NORMAL
STRESS
```

profile replay는 다음을 별도 기록한다.

- direction READY 최초 시각
- warmup·low-confidence·dropout·stale·invalid hold 구간
- exact directional Capsule과 witness footprint clearance
- profile stream에서 기동 판단이 가능한 최초 시각
- witness 시작을 그 시각까지 미뤘을 때 episode 안 완료 가능 여부
- actual Actor가 Capsule 밖에 있었던 경험적 sample
- `prediction_admissible`, `observation_decidable`과 limitation

Gaussian sample의 Capsule miss를 ground-truth witness hard failure로 바꾸지 않는다. 반대로
prediction이 ground truth를 포함하지 못해도 실제 clearance 위반을 무시하지 않는다.

current `stop_epoch` authorization과 shared gate의 실제 재개는 R5~R6에서 검증한다. R2가
관측 READY만으로 `online executable`을 주장하지 않는다.

### 7.3 Existing witness 비교

same-direction-wide 5개에서는 새 결과와 legacy witness를 다음 의미로 비교한다.

- 둘 다 ground-truth hard validator 통과 여부
- duration, path length, maximum deviation와 stop count
- ordered overtake·rejoin·terminal 결과
- current R1 profile replay 결과

새 witness가 기존 882개 point와 byte-identical일 필요는 없다. 안전·운동학·순서 의미가 같아야
한다. 기존 witness를 새 결과로 덮어쓰지 않는다.

## 8. 상위 Evidence 분류 규칙

search status와 독립 validator를 조합해 다음처럼 판정한다.

### `FEASIBLE`

```text
PASS_LEFT 또는 PASS_RIGHT witness found
AND ground-truth hard validator pass
AND ordered departure→overtake→rejoin→dwell pass
```

profile replay가 판단 불가여도 ground-truth `FEASIBLE`은 유지하고 별도
`OBSERVATION_UNDECIDABLE` limitation을 병기할 수 있다.

### `WAIT_ONLY`

다음 두 조건을 모두 요구한다.

- 안전한 `WAIT_AND_FOLLOW` witness가 존재함
- explicit no-passing policy 또는 독립적인 좁은 통로 analytic proof가 pass를 금지함

단순히 structured pass를 못 찾았다는 이유만으로 `WAIT_ONLY`를 부여하지 않는다.

### `FORBIDDEN`

- public maneuver policy constraint가 pass를 금지하거나
- static/forbidden geometry가 허용영역 밖 이탈을 요구함

category label만으로 부여하지 않는다. 안전한 wait witness가 있으면 `WAIT_ONLY`를 함께
기록할 수 있다.

### `NO_SAFE_SOLUTION`

R2에서는 다음처럼 단순하고 완결된 analytic golden만 확정할 수 있다.

- Actor가 episode 전체 동안 reference를 막음
- 명시적 no-passing 또는 corridor cross-section proof로 좌·우 통과 불가
- goal까지 다른 허용 route가 없음
- 초기 위치의 `HOLD_ONLY`는 충돌 없이 유지 가능

일반 지도에서의 `NO_SAFE_SOLUTION` 확정은 R3 이후로 미룬다.

### `OBSERVATION_UNDECIDABLE`

- ground-truth witness는 valid지만
- source invalid·stale·confidence 부족 또는 prediction admissibility 때문에 profile lane에서
  기동 시작을 정당화할 수 없음

이는 ground-truth 공간·시간 불가가 아니다. fail-closed 정지가 정상 결과다.

### `SEARCH_INCONCLUSIVE`

- `NO_WITNESS_IN_STRUCTURED_TEMPLATE`
- `RESOURCE_LIMIT`
- 현재 R2로 공간·시간 완전성을 증명할 수 없음

`SEARCH_INCONCLUSIVE`은 실패를 숨기는 이름이 아니다. R3가 필요한 정확한 분기다.

## 9. 결과 산출물

실행별 새 output 디렉터리에 다음을 생성한다.

```text
witness_search_manifest.json
witness_search_results.json
summary.md
episodes/<public_id>/search_diagnostics.json
episodes/<public_id>/selected_witness.json       # 있을 때만
episodes/<public_id>/ground_truth_validation.json
episodes/<public_id>/profile_replay.json
episodes/<public_id>/trajectory.png
```

manifest 필수 항목:

- code commit·source tree hash
- R1 audit content hash
- v6 public corpus hash
- legacy mechanism golden hash
- map·vehicle·search config·validator version hash
- profile·prediction parameter hash
- episode 순서와 process worker 수
- output schema version
- hidden 사용 `false`

PNG에는 static·forbidden·reference·Actor ground-truth trajectory·witness·departure·overtake·
rejoin을 표시한다. PNG는 사람이 확인하는 보조물이며 JSON 판정을 대체하지 않는다.

기존 output 디렉터리가 있으면 실행을 거부한다. partial result는 `partial=true`와 완료 episode
목록을 기록하고 R2 final summary로 사용하지 않는다.

## 10. Hard failure, 정상 음성 결과와 limitation

### Hard failure

- hidden 또는 허용되지 않은 split 입력
- search projection에 label·oracle·기존 witness 누출
- 같은 seed·config의 semantic 결과 비결정성
- search가 선택한 witness의 독립 validator 실패
- 충돌·금지구역·실제 clearance 위반
- 운동학·속도·가감속·terminal dwell 위반
- ordered pass라고 기록했지만 departure·overtake·rejoin 순서 불일치
- provenance·hash·revision 불일치
- non-finite·예외·schema 불일치
- 기존 output 덮어쓰기
- category label 변경이 search 결과를 바꿈
- known forbidden·wait-only golden에서 pass false positive

### 정상 음성 결과

- explicit forbidden에서 pass 미생성
- wait-only에서 wait witness 생성
- observation invalid에서 `OBSERVATION_UNDECIDABLE`과 hold
- no-safe-solution analytic golden에서 movement witness 없음·safe hold
- Stress confidence 부족으로 fail-closed

### Limitation

- R2 structured template 밖 path 존재 가능성
- resource limit 또는 template exhaust
- legacy-v1 mechanism test가 v6 성능 증거가 아님
- current public에 방향전환·정지 후 재출발 Actor가 없음
- Gaussian coverage가 확률적 안전 보장이 아님
- profile replay가 authority·controller 실행 증거가 아님
- 가상 차체와 open-loop Actor만 사용

## 11. 시험 명세

### 11.1 계약·탐색 단위시험

1. search projection이 금지 필드를 포함하지 않음
2. hidden·invalid provenance·non-finite 입력 거부
3. 같은 input/config의 결과와 content hash 결정론
4. label·oracle·기존 witness를 바꿔도 search 결과 불변
5. resource limit은 `RESOURCE_LIMIT`, infeasible로 오분류하지 않음
6. candidate 순서와 process completion 순서가 selected witness를 바꾸지 않음
7. left/right mirror에서 rigid-transform 동등 결과
8. `0.70m` 또는 legacy command ticks가 search 정답으로 하드코딩되지 않음

### 11.2 Witness·validator 적대시험

9. 기존 same-direction-wide positive 자동 witness 통과
10. pose 한 점 측면 변조의 운동학 실패
11. 20Hz timestamp 한 점 변조 실패
12. 선·각속도와 가감속 초과 실패
13. static·forbidden 침범 실패
14. 200Hz 사이 충돌을 20Hz endpoint만으로 놓치지 않음
15. 실제 Actor clearance `0.08m` 경계 통과·미달 실패
16. overtake 전 rejoin 또는 Actor 소멸 뒤 잘못된 pass 판정 실패
17. rejoin `0.50s` 미만 실패
18. terminal 실제 정지·dwell 누락 실패

### 11.3 공개 corpus 시험

19. v6 feasible replica 5개에서 ground-truth pass witness 자동 발견
20. same-direction-narrow에서 pass false positive `0`
21. v6 wait 5개에서 wait·follow 또는 정당한 hold
22. v6 dynamic-change 2개에서 두 위험의 순서와 재정지 evidence
23. legacy no-safe golden analytic 음성 판정
24. legacy observation-invalid golden의 `OBSERVATION_UNDECIDABLE`
25. Ideal replay는 deterministic exact input
26. Normal replay의 READY·Capsule·limitation 기록
27. Stress low-confidence·dropout을 우회 성공으로 가장하지 않음
28. serial과 process-parallel episode 결과 동일

### 11.4 출력·회귀

29. JSON schema·self hash·manifest hash 검증
30. 기존 output 비덮어쓰기
31. partial 결과가 final evidence로 봉인되지 않음
32. graphically empty/negative 결과도 PNG 생성
33. 표적 시험, Ruff, compileall과 최신 전체 회귀 통과

## 12. R2 완료조건

다음이 모두 충족돼야 `R2 완료`다.

- 상세 명세와 구현·시험·TRACEABILITY 연결
- 기존 same-direction-wide 수동 positive 의미를 자동으로 재현
- v6 public 13개와 legacy mechanism golden 6개를 모두 실행
- positive·wait·forbidden·no-solution·observation-invalid mechanism을 구분
- 독립 validator hard failure `0`
- known negative의 pass false positive `0`
- `NO_WITNESS`, resource limit과 일반 infeasible을 구분
- ground-truth feasible과 profile observation decidable을 분리
- serial/process 결과 결정론
- JSON·Markdown·PNG 비덮어쓰기 산출물 생성
- 표적·전체 회귀와 코드 품질 검사 통과
- hidden·controller·gate·안전 수치·제품 결정을 변경하지 않음

R2가 완료돼도 `SPATIALLY_INFEASIBLE` 일반 판정은 완료되지 않는다. R3가 그 역할을 맡는다.

## 13. R3 진입 Gate

R2 결과를 다음처럼 전달한다.

```text
자동 witness found + validator pass
→ R4 reference 후보의 독립 positive 근거

wait/forbidden analytic proof
→ WAIT_OR_FOLLOW 후보와 false-positive 회귀

ground-truth feasible + observation undecidable
→ prediction·운영 범위 limitation, controller 튜닝 금지

NO_WITNESS_IN_STRUCTURED_TEMPLATE
→ R3 bounded 공간 oracle 입력

RESOURCE_LIMIT
→ search config·성능 문제, 불가능 판정 금지
```

다음 상황에서는 R3 구현 전에 중단하고 보고한다.

- legacy positive witness도 current hard validator에서 재현되지 않음
- search projection에서 정답 누출을 제거할 수 없음
- ManeuverConstraintSpec 없이 forbidden을 category label로만 판정해야 함
- ground-truth evaluator와 online prediction 검사가 서로 재사용돼 독립성을 잃음
- R1 prediction 계약을 바꾸지 않고 profile replay를 구성할 수 없음

## 14. 구현 예정 경계

R2 구현 시 권장 신규 모듈은 다음과 같다.

```text
src/hospital_path_lab/dynamic_witness_contracts.py
src/hospital_path_lab/dynamic_witness_search.py
src/hospital_path_lab/dynamic_witness_validation.py
src/hospital_path_lab/dynamic_witness_reporting.py
scripts/run_dynamic_witness_audit.py
tests/test_dynamic_witness_contracts.py
tests/test_dynamic_witness_search.py
tests/test_dynamic_witness_validation.py
tests/test_dynamic_witness_public_audit.py
```

기존 `dynamic_corpus.py`의 수동 witness와 private validator는 회귀 기준으로 유지한다. 새
search가 기존 helper를 호출해 정답을 가져오거나 private validator를 그대로 최종 validator로
사용하지 않는다.

구현은 다음 순서로 진행한다.

1. 계약·projection·금지 필드 적대시험
2. 독립 ground-truth validator
3. WAIT/HOLD template
4. PASS structured geometry·kinematics template
5. current R1 profile replay
6. 공개 13+6 mechanism audit
7. JSON·PNG·병렬 episode runner
8. 전체 회귀와 독립 읽기 전용 감사

이 순서는 연구 구현 순서이며 최종 제품 경로 알고리즘을 확정하지 않는다.

### 14.1 현재 구현 checkpoint — 2026-08-13

두 번째 구현 묶음까지 다음을 완료했다.

- `dynamic_witness_contracts.py`: label-free `WitnessWorldSnapshot`, 명시적
  `ManeuverConstraintSpec`, witness·검색 상태·objective·result 계약
- `project_public_witness_world()`: `GOLDEN`·`DEVELOPMENT`만 허용하고 원본 episode ID,
  category, family·orientation label, oracle, 기존 witness와 controller 자료를 제외한 projection
- `dynamic_witness_validation.py`: 검색 코드와 기존 private corpus validator를 호출하지 않는
  exact ground-truth Actor 원·200 Hz hard validator
- 기존 same-direction-wide 공개 positive 5개를 새 독립 validator로 재검증
- pose·timestamp·가속·terminal dwell·provenance·no-passing·Actor clearance 변조와
  20 Hz endpoint 사이 위험을 거부하는 적대 시험
- `dynamic_witness_search.py`: label·oracle을 받지 않는 `HOLD_ONLY`와
  `WAIT_AND_FOLLOW` structured search
- Actor 활성 사건 기반 departure anchor, 20 Hz reference-follow 합성, 정확한 Actor 원의
  terminal-stopping guard와 최종 독립 validator 재검증
- episode 전체를 덮는 `HOLD_ONLY`, terminal dwell과 분리한 실제 비terminal wait, wait 뒤
  `0.10m` 이상 후속 progress 순서 검증
- `RESOURCE_LIMIT`과 template no-witness 분리, 후보 수 bucket 합계, validator version과 선택
  validation hash 결박, wall-clock 제외 semantic result hash
- 초기 제동과 최소 wait를 반영한 effective departure tick 정규화·중복 제거, 전체 후보를
  보관하지 않고 best WAIT·best HOLD만 유지하는 streaming 평가
- 5 ms grid 밖 Actor 활성 시작·종료 시각도 exact evaluation sample로 삽입해 순간 출현
  clearance 위반을 놓치지 않는 검증
- 세 번째 구현 묶음의 구현 전 기준으로
  [`R2-PASS 상세 명세`](12-pass-structured-witness-search.md)를 작성했다. 현재는 동결 후보이며
  PASS 코드·시험을 구현했다는 뜻이 아니다.

`tests/test_dynamic_witness_search.py`의 현재 14개 pytest case는 full-duration hold, wait→follow
순서, label·oracle 비누출, 결정론적 hash·count, resource limit, nonzero initial twist와 공개
직선·코너·다중 Actor 대표 사례를 검사한다. 또한 2026-08-13 읽기 전용 수동 감사에서 v6 공개
`13/13`과 legacy golden `6/6`의 선택 witness가 독립 validator를 통과했다. 이 `13+6` 결과는
아직 체크인된 전체 공개 audit runner나 영구 CI 시험이 아니며, taxonomy 정답 일치·online
controller 실행·제품 알고리즘 채택의 증거로 사용하지 않는다. hidden은 생성·열람·실행하지
않았다.

아직 구현하지 않은 범위는 다음과 같다.

- `PASS_LEFT/PASS_RIGHT` structured geometry·kinematics 자동 탐색
- profile별 prediction·observation replay
- 공개 13+6 taxonomy 판정과 영구 자동 audit
- JSON·PNG·process-parallel runner

따라서 현재 checkpoint는 `R2 완료`가 아니다. 수동 witness를 새 검색 결과로 가장하거나
`NO_WITNESS`를 공간·시간 불가능으로 해석하지 않는다.
