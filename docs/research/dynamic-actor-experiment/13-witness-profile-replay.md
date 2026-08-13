# R2 — Witness 관측·예측 프로필 재생 상세 명세

## 1. 상태와 목적

- 작성일: `2026-08-13`
- 상태: 대표 public 구현 checkpoint 완료, R2 전체 미완료
- 상위 단계: `R2 — 기존 Witness 자동화·일반화`
- 선행 입력: 독립 ground-truth validator를 통과한 `AutomatedWitness`
- 실행 범위: Python `simulation_only`, 공개 `GOLDEN`·`DEVELOPMENT`
- hidden: 생성·열람·실행 금지
- 제품 알고리즘 채택, `G1~G5`, 제품 경로분석 7단계: 미수행

이 단계는 자동 탐색된 witness가 정확한 Actor 위치에서는 안전하다는 사실과, 동결된 합성
관측·방향성 prediction으로 그 기동을 판단할 수 있다는 사실을 분리한다.

```text
ground-truth witness 존재
→ FUNCTIONAL_IDEAL 관측 재생
→ NORMAL 관측 재생
→ STRESS 관측 재생
→ 판단 가능성·중단 구간·Capsule 관계 기록
```

이 단계는 controller 또는 shared safety gate를 실행하지 않는다. `READY`가 나온 사실이나
prediction Capsule과 witness가 떨어져 있다는 사실을 이동 허가 또는 online 실행 성공으로
해석하지 않는다.

## 2. 질문과 비질문

### 2.1 답하는 질문

1. profile별 방향 prediction이 최초로 `READY`가 되는 시각은 언제인가?
2. warmup·low confidence·dropout·stale·invalid 때문에 정지가 필요한 구간은 어디인가?
3. 최초 `READY`까지 초기 정지를 유지한 뒤 witness를 시작해도 episode 안에 끝나는가?
4. 지연된 witness가 바뀐 Actor 시간관계에서도 ground-truth hard validator를 통과하는가?
5. 관측이 유효한 tick에서 방향성 Capsule과 witness footprint clearance가 `0.08m` 이상인가?
6. 실제 Actor 원이 경험적으로 Capsule 안에 들어왔는가?

### 2.2 답하지 않는 질문

- controller가 witness를 추종할 수 있는가
- shared gate의 `stop_epoch`·11개 safe frame·재승인 조건을 통과하는가
- Normal·Stress에서 임무를 연속 완료하는가
- 실제 센서·사람·차체에서도 같은 prediction이 성립하는가
- 제품 local planner 또는 controller를 채택해야 하는가

위 항목은 각각 `R5~R7` 또는 축소 실물·종단 시험 대상이다.

## 3. 입력과 provenance

### 3.1 필수 입력

```text
WitnessWorldSnapshot
AutomatedWitness
GroundTruthWitnessValidation
DynamicObservationProfile
DirectionalPredictionParameters
```

- world와 witness의 source·world·vehicle·search hash가 일치해야 한다.
- 원 witness는 kind가 PASS이면 strict declaration 검증까지 통과해야 한다.
- profile은 아래 동결 3개만 허용한다.
- prediction parameter는 `FROZEN_DIRECTIONAL_PREDICTION_PARAMETERS`와 정확히 같아야 한다.
- corpus label, expectation category, oracle, 기존 수동 witness, controller ID는 입력하지 않는다.

### 3.2 동결 profile

| profile | 주기 | 지연 | TTL | 위치 σ | 속도 σ | dropout |
|---|---:|---:|---:|---:|---:|---:|
| `FUNCTIONAL_IDEAL` | `10Hz` | `100ms` | `300ms` | `0` | `0` | `0` |
| `NORMAL` | `10Hz` | `100ms` | `300ms` | `0.03m` | `0.05m/s` | `5%` |
| `STRESS` | `10Hz` | `250ms` | `300ms` | `0.08m` | `0.15m/s` | `20%` |

Normal·Stress는 같은 latent Gaussian·dropout draw namespace를 사용한다. profile 결과를 보고
seed, noise, dropout 또는 prediction parameter를 바꾸지 않는다.

### 3.3 공개 split

profile replay API는 `WitnessWorldSnapshot`만 받는다. 해당 world는 기존 public projection에서
만들어져야 하며 hidden을 복원하거나 원본 episode label을 조회하지 않는다.

## 4. observation trace 재구성

world의 exact Actor trajectory에서 20Hz `DynamicGroundTruthFrame`을 만들고 기존
`generate_dynamic_observation_slots()`를 호출한다.

```text
stream_id = "dynamic-witness-profile-replay"
episode_id = world.world_id
episode_seed = world.seed
map_id/revision = world.map_id/revision
mission_revision = 0
```

10Hz slot은 scheduled delivery 시각에 validator에 전달한다. dropout은 `record_no_frame()`으로
기록한다. 매 20Hz control tick에서 다음 순서로 처리한다.

```text
도착 slot 전달
→ DynamicObservationValidator.snapshot(control_time)
→ DirectionalActorPredictor.update(snapshot)
→ status·hold_required·prediction hash 기록
```

같은 10Hz frame을 두 20Hz tick에서 보더라도 predictor의 duplicate 처리에 맡긴다. 임의로
history count를 두 번 늘리지 않는다.

## 5. profile 상태 trace

각 20Hz tick의 prediction 상태를 다음 값으로 기록하고, 연속된 같은 상태는 interval로
압축한다.

```text
READY
WARMING_UP
LOW_SPEED
LOW_CONFIDENCE
EMPTY_FRAME
DROPOUT
STALE
INVALID
UNAVAILABLE
ORDER_VIOLATION
```

interval은 `start_tick`, `end_tick`을 모두 포함하며 각 tick의 시각은 `tick*0.05s`다.

- `READY`, `EMPTY_FRAME`: predictor 자체가 hold를 요구하지 않는 상태
- 나머지: 판단 불충분 또는 계약 실패로 hold가 필요한 상태
- `EMPTY_FRAME`은 fresh empty이며 dropout과 다르다.
- `READY`가 한 번도 없으면 `observation_decidable=false`다.
- Actor가 처음부터 끝까지 없는 world는 별도 EMPTY-only mechanism으로 기록하며, 방향 판단
  성공으로 가장하지 않는다.

## 6. 최초 READY와 지연 witness

### 6.1 시작 시각

Actor가 있는 witness의 profile 시작 후보는 최초 `READY` control tick이다.

```text
delayed_start_tick = first_ready_tick
delayed_start_time = delayed_start_tick * 0.05s
```

원 witness를 단순히 시간 이동했다고 가정하지 않는다. 다음처럼 초기 정지 구간을 실제로
붙인다.

```text
t=0 .. delayed_start_time: initial pose + zero twist HOLD
delayed_start_time 이후: 원 witness point·event time을 같은 양만큼 이동
```

초기 twist가 0이 아니면서 start delay가 필요한 witness는 v1 범위 밖이며
`delayed_witness_unsupported_nonzero_initial_twist`로 거부한다.

### 6.2 재검증

지연 witness는 다음을 다시 통과해야 한다.

- episode duration 안 종료
- 기존 20Hz 운동학과 속도·가감속
- 200Hz static·forbidden·exact Actor clearance
- PASS의 Actor 존재 중 ordered overtake·side·재합류 순서
- terminal dwell

즉 최초 READY 뒤 사람이 더 이동해 pass 순서가 깨지거나, warmup 중 Actor가 정지 위치를
침범하거나, 종료시간이 넘으면 `delayed_ground_truth_valid=false`다.

원 witness와 지연 witness의 validation hash를 모두 결과에 결박한다.

## 7. Capsule replay

### 7.1 평가 시간축

control time `t`의 prediction은 동결 command-apply 지연 `0.05s` 뒤의 witness 구간
`[t+0.05,t+0.10]`에 적용한다. post-apply 구간은 최대 `5ms` 간격으로 나누고, 각 rollout
offset `u`에서:

```text
robot_pose = witness의 절대시각 t + 0.05 + u pose
capsule = sample_directional_capsules(prediction, rollout_time_s=u)
clearance = surface_distance(oriented wheelchair footprint, capsule)
```

최소 predicted surface clearance와 `<0.08m` 위반 수를 기록한다. `EMPTY_FRAME`이면 Capsule이
없는 것이 정상이나, 같은 절대시각에 actual Actor가 존재하면 별도 coverage miss로 기록한다.

### 7.2 관측 중단

motion tick이 READY 또는 valid EMPTY가 아니면 해당 tick은 `unavailable_motion_tick`이다.
그 구간의 Capsule clearance를 추측하지 않는다.

```text
observation_continuous_for_witness = unavailable_motion_tick_count == 0
```

이 값이 false여도 ground-truth witness가 무효가 되는 것은 아니다. online에서는 gate가
감속·정지를 개시해야 하므로 원 witness의 연속 실행 증거가 없다는 뜻이다.

### 7.3 actual Actor containment

각 Capsule sample 시각에 exact Actor 중심이 같은 binding의 Capsule 선분으로부터
`capsule_radius - actor_radius` 안에 있는지 측정한다.

- Ideal miss는 hard replay failure다.
- Normal·Stress miss는 경험적 통계 limitation이다.
- Gaussian `2σ`는 확률적 안전 보장이 아니므로 Normal·Stress miss `0`을 hard criterion으로
  요구하지 않는다.
- actual Actor clearance는 독립 ground-truth validator 결과를 사용하며 Capsule 결과로
  대체하지 않는다.

## 8. 출력 계약

### 8.1 profile 결과

```text
WitnessProfileReplayResult
- schema/replay version
- source/world/witness/profile/prediction hashes
- ground_truth_validation_hash
- profile_name
- slot/delivery/dropout counts
- status counts + compressed intervals
- first_ready_tick/time
- observation_decidable
- delayed_witness + delayed validation hash
- delayed validation failure codes
- shifted_completion_within_episode
- delayed_ground_truth_valid
- observation_continuous_for_witness
- evaluated/unavailable motion tick counts
- Capsule sample/violation/minimum-clearance
- actual Actor containment sample/miss/maximum miss
- capsule_geometry_admissible_when_observed
- prediction_admissible
- limitations
- semantic content hash
```

wall-clock 시간은 semantic hash에 포함하지 않는다.

### 8.2 bundle

```text
WitnessProfileReplayBundle
- FUNCTIONAL_IDEAL result
- NORMAL result
- STRESS result
- source/world/witness hashes
- ground-truth validation hash
- limitations
- semantic content hash
```

profile 순서는 항상 `FUNCTIONAL_IDEAL, NORMAL, STRESS`다. process 완료 순서로 바꾸지 않는다.

## 9. 판정 규칙

### `observation_decidable`

Actor가 있는 world에서 `READY`가 한 번 이상 존재한다.

### `capsule_geometry_admissible_when_observed`

평가한 Capsule sample이 하나 이상이고 predicted clearance 위반이 0이다.

### `prediction_admissible`

다음을 모두 만족한다.

```text
observation_decidable
AND shifted_completion_within_episode
AND delayed_ground_truth_valid
AND observation_continuous_for_witness
AND capsule_geometry_admissible_when_observed
AND Ideal이면 actual Actor containment miss == 0
```

Normal·Stress의 containment miss는 hard predicate에서 제외하고 limitation으로 남긴다.

`prediction_admissible`도 online 이동 허가가 아니다. stop_epoch authorization, 11개 safe frame,
path/local recheck와 shared gate는 이 단계에서 실행하지 않는다.

## 10. hard failure와 정상 음성 결과

### hard failure

- world·witness·validation hash 불일치
- 원 witness ground-truth validation 실패
- 지원하지 않는 profile 또는 prediction parameter
- observation generator/validator가 동결 public 입력을 거부
- non-finite Capsule geometry
- Ideal 관측 오차·dropout·containment miss
- 같은 입력의 semantic 결과 비결정성

### 정상 음성·limitation

- `no_ready_prediction`
- `delayed_witness_exceeds_episode`
- `delayed_ground_truth_invalid`
- `observation_interrupted_during_witness`
- `predicted_clearance_rejected`
- `normal_or_stress_capsule_containment_miss`
- `online_controller_and_gate_not_evaluated`
- `simulation_only_open_loop_circular_actor`

정상 음성 결과를 예외나 hard safety 실패로 바꾸지 않는다.

## 11. 시험

### 계약·결정론

- 세 profile 순서와 hash 결정론
- elapsed·실행 순서가 semantic hash에 영향 없음
- world/witness/validation/profile/prediction tamper 거부
- hidden·label·oracle import가 profile replay 모듈에 없음

### observation

- Ideal: dropout 0, exact observation, READY 도달
- Normal: 동결 noise/dropout 재현
- Stress: low confidence·dropout·stale를 hold로 기록
- fresh EMPTY와 no-frame 분리
- duplicate 20Hz tick이 history를 증가시키지 않음

### 지연 witness

- 선행 hold와 event time shift
- start delay 뒤 200Hz ground-truth 재검증
- duration exact boundary pass, `+0.05s` fail
- nonzero initial twist delay 거부
- Actor가 hold pose를 침범하면 delayed validation 실패

### Capsule

- 5ms 이하 subdivision
- oriented footprint·Capsule `0.08m` 경계
- Ideal actual Actor containment miss 0
- Normal·Stress miss를 hard safety로 오분류하지 않음
- READY가 아닌 tick에서 Capsule을 추측하지 않음

### 대표 public

- `same-direction-wide-r00`의 자동 strict PASS witness
- Ideal에서 최초 READY만큼 지연한 witness를 ground-truth로 재검증하고, 통과 또는 정확한
  실패 원인을 보존
- profile별 first READY·중단·clearance·containment 결과 결정론
- 결과가 controller 실행·목적지 완료로 표시되지 않음

### 2026-08-13 구현 checkpoint

- 표적 profile replay: `11 passed`
- PASS·WAIT/HOLD·관측·prediction을 포함한 직접 영향권: `163 passed`
- 8개 독립 pytest process 전체 회귀: `679 passed`
- 대표 `same-direction-wide-r00` RIGHT witness:
  - Ideal 최초 READY `2.00s`, Normal `2.10s`, Stress READY 없음
  - Ideal·Normal 지연 witness는 `actor_clearance_violation`과
    `declared_pass_time_mismatch`로 ground-truth 재검증 실패
  - Ideal predicted minimum clearance 약 `0.07427m`
  - Normal actual Actor containment miss `354/3,285`
- 위 수치는 online controller·gate 실행이나 목적지 완료 증거가 아니다.
- hidden은 생성·열람·실행하지 않았다.

## 12. 완료조건과 다음 단계

R2 profile replay checkpoint 완료조건:

1. 이 문서의 계약과 표적 시험 통과
2. 공개 same-direction-wide 대표 witness의 Ideal·Normal·Stress bundle 생성
3. Ideal hard replay failure 0
4. Normal·Stress의 판단 불가·중단·coverage miss를 limitation으로 보존
5. 원·지연 witness ground-truth validation hash 결박
6. hidden 접근 0

`delayed_ground_truth_valid`나 `prediction_admissible` 자체를 checkpoint 구현 합격조건으로
두지 않는다. 두 값이 false이면 현재 ground-truth search witness와 관측 준비시간이 결합되지
않았다는 연구 결과이며, 안전 수치나 prediction을 완화하지 않고 다음 R2 search 입력에
반영한다.

그다음 공개 13개+legacy mechanism 6개의 영구 audit·JSON/PNG reporting으로 확장한다.
R2 전체가 닫히기 전에는 R3 bounded 공간 oracle, R4 reference, R5 controller 비교로 결과를
승격하지 않는다.

## 13. 증거 한계

이 단계의 결과는 합성 관측과 open-loop 원형 Actor를 사용한 offline 연구 증거다. 실제 사람
운동, 실제 센서, ROS 2 지연, 실제 차체 제동, 사람 탑승 안전, 의료기기 인증 또는 제품
알고리즘 채택을 증명하지 않는다.
