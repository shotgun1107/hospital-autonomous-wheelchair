# ADR 0010: 방향성 Actor 예측과 confidence gate

- 상태: 사용자 개인 승인, 팀 합의 전
- 날짜: 2026-08-12
- 범위: Python `simulation_only` source-derived v7 연구실험

## 배경

기존 Actor reachable tube는 관측 속도 방향으로 중심을 이동시키면서도 가속 불확실성을 모든
방향으로 원형 팽창한다. 이 모델에서는 앞으로 가는 Actor가 짧은 rollout 안에 정지하고 옆으로
움직이며 반대 방향으로 되돌아오는 경우까지 한꺼번에 포함된다. 그 결과 공개
`same-direction-wide-r00` 진단에서 source-derived DWB 후보 `217/217`개가 Actor clearance
constraint로 제거됐다.

반면 v6 공개 corpus의 Actor는 활성 구간 동안 일정한 속도와 heading을 유지한다. 현재
`LOCAL_DETOUR_FEASIBLE` 분류와 임의 방향 원형 reachable set은 서로 다른 운동 가정을 사용한다.

단일 관측 속도를 그대로 방향으로 쓰는 것도 허용할 수 없다. 느린 같은 방향 Actor의 속도보다
Normal·Stress 속도 noise가 비슷하거나 더 크므로, 공개 stream에서도 관측 방향이 반대로 나온
frame이 존재한다.

## 결정

source-derived v7의 현재 공개 비교에서는 Actor를 **실제 사람이 아닌 constant-heading
open-loop synthetic actor**로 한정하고 다음 구조를 사용한다.

1. 동일한 stable identity·binding의 최신 unique accepted frame 20개만 인과적으로
   모은다.
2. 최신 20개 `observed_velocity` 평균에서 `2·(velocity_sigma/√20)`를 뺀 하한이
   `0.03 m/s` 이상이고, 1.9초 이상 이력과 최신 위치 anchor 기준의 position-fit RMS gate까지
   통과할 때만 direction confidence를 인정한다.
3. confidence 전, 상실 뒤 또는 이력 reset 뒤에는 새 비영점 이동을 금지하고 제한 감속·정지를
   수행한다.
4. confidence 뒤에는 최신 `observed_position`을 anchor로 하고 평균 속도를 heading과 `s0`로
   사용하는 종방향 reachable capsule을 사용한다.
5. capsule 중심선은 제한 감속 정지거리부터 제한 가속 최대거리까지이며 뒤쪽 이동과 진행방향
   반전을 포함하지 않는다.
6. 현재 corpus의 측면 방향전환 상한은 `0`이다. 비영점 turn bound는 새 공개 방향전환 corpus가
   생기기 전까지 도입하지 않는다.
7. endpoint에는 `s0`의 제한 감속·가속만 반영하고 속도 불확실성은 Capsule 반경에 한 번만
   반영한다. 위치는 최신 frame의 현재 sigma를 사용하며 `2σ`를 확률적 안전 보장으로
   주장하지 않는다.
8. 기존 shared safety gate, terminal stopping, `stop_epoch`, provenance와 200 Hz ground-truth
   evaluator를 유지한다.
9. exact Capsule을 canonical online geometry로 사용한다. circle-chain은 legacy API 비교·호환
   표본일 뿐 안전 판정의 정본이 아니다.

구체 계산식·reset·시험·hidden 계약은
[`08-directional-actor-prediction-v7.md`](../research/dynamic-actor-experiment/08-directional-actor-prediction-v7.md)를 따른다.

이 승인은 Software A의 합성 Python 비교실험 진행 승인이다. 팀 전체의 제품 알고리즘 채택,
실제 사람 운동 모델 승인, `G1~G5` 또는 경로 분석 7단계 결정이 아니다.

## 고려한 대안

### 기존 임의 방향 원형 tube 유지

보수적 정지 기준선으로는 유지할 수 있지만 현재 공개 corpus의 우회 가능 질문을 평가하지 못한다.
현 진단에서는 정지 후보를 포함한 모든 DWB 후보를 제거했다.

### 원형 tube 반경만 축소

채택하지 않는다. 방향·속도 운동 계약을 고치지 않고 public 결과에 맞춰 반경만 줄이면 안전조건
튜닝과 corpus 과적합을 구분할 수 없다.

### 단일 frame의 observed velocity를 heading으로 사용

채택하지 않는다. 느린 Actor와 Stress noise에서 방향 반전 관측이 발생하므로 방향 근거가
불충분하다.

### 즉시 실제 사람의 비영점 turn bound 도입

현재는 채택하지 않는다. 공개 corpus에 방향을 바꾸는 Actor가 없어 해당 수치를 검증하거나
정당화할 수 없다. `30°/45°/90°` 공개 turn corpus와 경계시험을 먼저 만든다.

## 결과

장점:

- 현재 공개 Actor의 고정 heading 계약과 online predictor의 운동 가정을 일치시킨다.
- 정지·감속은 허용하면서 즉시 후진하는 원형 과팽창을 제거한다.
- 관측 근거가 부족할 때 우회하지 않는 fail-closed 경계를 유지한다.
- Actor별 estimator와 constraint를 DWB critic 점수에서 분리해 실패 원인을 추적할 수 있다.

비용과 제한:

- 최소 20개의 accepted frame이 필요하므로 우회 전 대기시간이 생긴다.
- Stress에서는 confidence가 늦게 생기거나 끝까지 생기지 않을 수 있다.
- 무클리핑 Gaussian과 `2σ`는 모든 실제 표본을 포함하지 않는다.
- 현재 모델은 방향전환 사람을 평가하지 않으며 실제 사람 행동의 근거가 아니다.
- 기존 feasible witness는 전체 rollout과 terminal stopping 조건으로 다시 검증해야 한다.
- estimator·capsule·corpus가 바뀌므로 새 manifest와 공개 자격시험이 필요하다.

## 안전 및 증거 경계

- confidence 미성립은 경로 생성 실패가 아니라 판단 불충분에 따른 안전정지 원인이다.
- confidence 획득, Actor 소멸 또는 위험 해소만으로 자동 재출발하지 않는다.
- shared gate가 실제 정지를 확인하고 현재 `stop_epoch`용 재승인과 기존 재개조건을 모두
  만족해야 한다.
- predictor가 안전하다고 판단해도 200 Hz evaluator의 실제 clearance 위반은 hard failure다.
- 방향 추정과 capsule 구현은 evaluator label·witness·ground truth를 입력받지 않는다.
- 결과를 실제 사람 탑승 안전성이나 의료기기·제품 안전 증거로 확대하지 않는다.

## 공개·hidden 운영

현재 공개-only 시험은 217개 action primitive의 후보당 41 pose, 2.0초 rollout과 terminal
stopping 구간에서 exact Capsule 계산과 결정론을 검증한다. 이 결과는 기존 `0.35 s` witness의
시간 공백을 드러내고 기하 계산 범위를 넓히지만, 실제 closed-loop DWB의 legal bypass를
증명하지 않는다. 그 상태는 `ONLINE_DWB_BYPASS_UNPROVEN`으로 유지한다. Stress 저속 Actor의
READY 0, non-READY TRACKS hold·prediction 미노출과 corpus-level LOW_CONFIDENCE 발생을 포함한
targeted 공개 자격은 최종 통과했다. 이는 estimator·기하 하위 자격이며 전체 폐루프 자격이 아니다.

구현 뒤에는 우선 v6 공개 13개를 Normal·Stress로 다시 검증한다. 같은 방향 wide 5개는 전체
rollout·terminal stopping witness와 실제 추월·재합류를 확인하고, narrow·head-on·diagonal·
corner·multi-Actor는 기존 보수 동작과 회전 동등성을 확인한다.

새 hidden은 생성·열람·실행하지 않았다. 생성·열람·실행은 이 공개 재검증과 manifest 동결
전까지 금지한다. 공개 결과 뒤
계약을 바꾸면 결과를 regression으로만 보존한다. 이후 hidden을 확인하고 변경하면 해당 hidden도
regression으로 전환하고 새 seed commitment를 만든다.

## 재검토 조건

다음 중 하나가 필요하면 이 ADR을 새 결정으로 대체한다.

- 실제 방향전환 Actor 또는 반응형 사람 모델 도입
- 20-frame window·confidence 식·2σ 계약 변경
- 비영점 lateral turn bound 도입
- controller-facing motion-segment revision 추가
- bounded-noise 관측으로 변경
- 실제 센서·ROS 2·축소 실물에서 motion estimator 사용

그 전까지 이 결정은 방향이 고정된 합성 Actor의 Python 연구실험에만 유효하다.
