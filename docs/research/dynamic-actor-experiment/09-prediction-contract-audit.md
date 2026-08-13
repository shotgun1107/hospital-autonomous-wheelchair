# 1단계 — Actor prediction 계약 감사 명세

## 1. 상태와 목적

- 날짜: `2026-08-13`
- 상태: 사용자 개인 연구 방향에 따른 공개-only 구현 명세
- 범위: Python `simulation_only`
- 대상: v6 공개 13개 episode와 Ideal·Normal·Stress 합성 관측
- hidden: 생성·열람·실행 금지

이 단계는 controller를 튜닝하거나 우회 기능을 개선하지 않는다. 목적은 현재 방향성 Actor
예측에서 서로 다른 의미로 사용되던 다음 두 계약을 분리해 측정하는 것이다.

```text
deterministic_motion_containment
!=
statistical_observation_coverage
```

결과는 실제 사람 운동, 실제 센서 또는 실제 탑승 안전성의 증거가 아니다.

## 2. 감사 질문

### Q1. 결정론적 운동 계약

현재 v7 방향성 모델이 가정하는 constant-heading·forward-only Actor 운동 범위 안에 공개
corpus의 실제 ground-truth 운동이 존재하는가?

검사 항목:

- 속도 상한
- 제한 가속·감속
- 즉시 후진·방향반전 금지
- `lateral_turn_bound=0`에서의 측면 이동·heading 변화
- 연속 sample 사이의 종방향 이동 하한·상한
- 모든 파생값의 finite 여부

현재 v6 공개 Actor는 활성 구간 동안 일정한 속도 벡터를 사용한다. 따라서 공개 audit은
constant-heading 구간의 계약만 자격화한다. 정지·재출발, 비영점 가속·감속과 `30°/45°/90°`
방향전환은 별도 synthetic auditor 회귀로 탐지 능력만 확인하며, 공개 corpus 기능 증거로
계산하지 않는다.

### Q2. Gaussian 관측 coverage

Normal·Stress의 독립 Gaussian x/y 위치·속도 잡음이 동결한 profile과 일치하는지 측정한다.

다음 두 coverage를 별도로 기록한다.

```text
component_2sigma
= |error_x| <= 2 sigma, |error_y| <= 2 sigma를 축별로 계산

radial_2sigma
= sqrt((error_x/sigma)^2 + (error_y/sigma)^2) <= 2
```

독립 표준정규 2축에서 이론값은 서로 다르다.

```text
component_2sigma_probability ~= 0.954499736
radial_2sigma_probability    ~= 0.864664717
```

따라서 Normal·Stress에서 `2σ 위반 0회`를 요구하지 않는다. 표본 수, 포함 수, 비율,
최대 정규화 오차와 dropout 수를 그대로 보존한다. Ideal은 sigma와 dropout이 0이므로 관측과
ground truth가 정확히 같아야 한다.

### Q3. 방향성 Capsule의 경험적 coverage

방향 추정기가 `READY`를 반환한 고유 prediction에 대해 다음 rollout 시각을 검사한다.

```text
0.0 s / 0.5 s / 1.0 s / 1.5 s / 2.0 s / 2.4 s
```

ground-truth Actor 원 전체가 예측 Capsule 안에 포함되려면 다음을 만족해야 한다.

```text
distance(actor_center, capsule_center_segment)
<= capsule_base_radius - actor_radius
```

Ideal의 누락은 결정론적 계약 실패다. Normal·Stress의 누락은 통계적 coverage 결과이며
그 자체를 hard failure로 승격하지 않는다. 결과에 맞춰 `2σ`, Actor 반경, clearance 또는
운동 bound를 바꾸지 않는다.

## 3. 입력과 공개 경계

입력은 다음 함수가 생성한 공개 자료로 제한한다.

```text
generate_dynamic_v6_public_corpus()
generate_episode_ground_truth_frames()
generate_episode_observation_slots()
```

허용 split:

```text
GOLDEN
DEVELOPMENT
```

다음 정보는 predictor나 controller 입력으로 전달하지 않는다.

- expectation category
- oracle spec
- feasible witness
- scenario label
- evaluator ground truth
- hidden seed 또는 hidden episode

감사기는 offline evaluator이므로 ground truth를 사용할 수 있지만, 결과는 online prediction과
분리해 저장한다.

## 4. 출력 계약

`prediction_contract_audit.json`에 다음을 저장한다.

- schema·audit version
- public corpus hash와 episode 수
- 결정론적 motion sample·transition·feature 수
- motion violation의 episode·Actor·시각·reason
- profile별 slot·dropout·track 수
- 위치·속도의 component/radial 2σ coverage
- 최대 정규화 관측오차
- profile·rollout별 Capsule coverage와 최대 miss distance
- hard failure
- 명시적 limitation
- 전체 결과 content hash

`summary.md`에는 hard failure와 coverage를 사람이 읽을 수 있는 표로 저장한다.

기존 output을 덮어쓰지 않는다. output 디렉터리가 이미 존재하면 실행을 거부한다.

## 5. Hard failure와 limitation

### Hard failure

- 공개 corpus에서 speed·longitudinal acceleration·no-reverse·constant-heading 운동 계약 위반
- non-finite ground truth·관측·coverage 결과
- Ideal 관측오차 또는 dropout 발생
- Ideal `READY` Capsule이 ground-truth Actor 원을 포함하지 못함
- source·binding·timestamp가 맞지 않아 비교 대상을 연결할 수 없음
- 같은 seed 실행의 semantic 결과 비결정성
- hidden 또는 허용되지 않은 split 입력

### Limitation

- Normal·Stress의 component/radial `2σ` 바깥 표본
- Normal·Stress의 방향성 Capsule coverage miss
- confidence 미성립으로 Capsule 표본이 없는 profile·episode
- 공개 corpus에 비영점 가속·감속·정지·방향전환 전이가 없음
- open-loop constant-heading 원형 Actor만 사용

limitation은 숨기지 않지만 hard failure와 혼합하지 않는다.

## 6. 시험

최소 회귀시험:

1. constant-heading·bounded-speed trace 통과
2. 제한 안의 종방향 감속·정지 통과
3. 즉시 방향반전 거부
4. `lateral_turn_bound=0`에서의 90° 방향전환 거부
5. Ideal 관측·Capsule coverage `100%`
6. Normal·Stress의 실제 `2σ` 바깥 표본을 limitation으로 기록
7. component와 radial 이론값을 혼동하지 않음
8. 공개 13개 외 split 거부
9. 같은 입력의 JSON semantic 결과 결정론
10. existing output 덮어쓰기 거부

## 7. 완료조건

- 세 coverage 의미가 JSON과 문서에서 분리된다.
- 공개 v6 motion contract의 hard failure가 0이다.
- Ideal 관측 및 Capsule hard failure가 0이다.
- Normal·Stress의 `2σ` miss가 숨겨지지 않고 hard failure로 오분류되지 않는다.
- 현재 공개 corpus가 검증하지 못하는 motion feature가 limitation으로 명시된다.
- 표적 시험, Ruff, compileall과 최신 전체 회귀가 통과한다.
- hidden, controller·gate·corpus 수치와 제품 결정을 변경하지 않는다.

## 8. 다음 분기

```text
motion hard failure
→ controller 수정 금지
→ Actor generator와 prediction motion contract를 먼저 재검토

Ideal coverage hard failure
→ 기하·시간축·prediction 구현 결함 역추적

Normal·Stress statistical miss only
→ 현재 coverage를 결과로 보존
→ safety 수치 완화 금지

1단계 완료
→ 기존 feasible witness 자동화·일반화 명세로 이동
```
