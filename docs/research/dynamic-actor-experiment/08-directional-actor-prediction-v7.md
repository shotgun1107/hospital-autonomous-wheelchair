# 방향 관성을 반영한 Actor 예측 v7 명세

## 1. 상태와 적용 범위

- 상태: **사용자 개인 승인, 팀 합의 전**
- 날짜: 2026-08-12
- 범위: Python `simulation_only` 공개 연구실험
- 대상: 방향이 고정된 open-loop 원형 Actor

이 명세는 실제 사람의 보행을 모델링하거나 사람 탑승 안전을 증명하지 않는다. 현재 v6 공개
corpus의 Actor는 활성 구간 동안 일정한 속도 벡터를 유지하며 방향을 바꾸지 않는다. 따라서 v7의
첫 방향 모델도 이 합성 조건만 다룬다.

다음은 이 명세의 범위가 아니다.

- 실제 사람의 방향 전환, 반응, 양보와 회피 행동
- 병원 환경의 보행 분포 또는 임상·제품 안전성
- 제품 알고리즘 채택, `G1~G5` 결정 또는 경로 분석 7단계
- 새 hidden corpus의 생성·열람·실행

관련 결정은
[`ADR 0010`](../../decisions/0010-directional-actor-prediction.md)에 기록한다.

## 2. 변경 이유

기존 예측은 현재 관측 속도를 중심으로 사용하지만 가속 불확실성을 모든 방향의 원으로 팽창한다.
그 결과 앞으로 이동하는 Actor도 rollout 동안 감속·정지·옆 이동·즉시 반대 방향 이동을 모두 할
수 있는 것으로 취급한다. v6 공개 `same-direction-wide-r00`의 첫 공개 진단에서는 이 hard
constraint가 DWB 후보 `217/217`개를 점수 계산 전에 제거했다.

공개 corpus 감사 결과는 다음과 같다.

- v6 공개 13개 Actor의 활성 구간 내 실제 가속도와 방향 변화는 모두 0이다.
- 실제 속도 범위는 `0.062~0.2912 m/s`다.
- `LOCAL_DETOUR_FEASIBLE`은 `same-direction-wide-r00~r04` 5개다.
- 기존 feasible witness는 `0.35 s`의 동시각 tube만 확인하며, 매 제어 시점의 전체 rollout과
  terminal stopping 예측을 증명하지 않는다.

따라서 임의 방향 원형 팽창을 조용히 줄이지 않고, 현재 공개 Actor의 운동 계약을 명시한 별도
v7 예측 모델을 만든다.

## 3. 입력 경계

방향 추정기는 검증이 끝난 `DynamicObservationSnapshot`만 사용한다. 다음 정보는 입력으로
사용하지 않는다.

- expectation category
- scenario family 또는 latent case ID
- feasible witness
- evaluator ground truth
- public·development 구분

각 추적 이력의 키는 최소한 다음을 포함한다.

```text
stream_id
episode_id
map_id / map_revision
track_id
actor_binding_id
```

source·revision·hash·TTL 검증은 기존 계약을 유지한다. 검증 실패를 방향 추정기로 보완하거나
무시하지 않는다.

현재 controller-facing `ActorTrack`에는 trajectory 또는 motion-segment revision이 없다. 따라서
현재 constant-heading corpus에서는 Actor 소멸·재출현과 binding 변경을 segment 경계로 사용한다.
향후 방향전환·정지 후 재출발 corpus를 만들기 전에는 `motion_segment_revision` 같은 명시적
controller-facing 필드를 먼저 계약에 추가해야 한다.

## 4. 인과적인 20-frame 방향 추정

### 4.1 유일한 accepted frame

하나의 Actor 방향을 계산하려면 동일한 추적 키에 속한 **서로 다른 20개의 accepted frame**이
필요하다.

- source validator가 승인한 frame만 기록한다.
- 현재 control snapshot 시각보다 늦게 도착하는 frame은 사용할 수 없다.
- sequence와 observation revision이 서로 다른 frame만 한 번씩 센다.
- dropout 동안 유지되는 마지막 frame을 여러 번 세지 않는다.
- 최신 20개 accepted velocity vector의 산술평균을 사용한다.

```text
v_mean = sum(v_i for the latest 20 unique accepted frames) / 20
heading = normalize(v_mean)
sigma_v_mean = max(velocity_sigma_i for the latest 20 frames) / sqrt(20)
```

현재 연구실험의 방향 confidence는 다음을 모두 만족할 때만 성립한다.

```text
direction_confident =
    unique_accepted_frame_count >= 20
    AND current_observation_is_fresh
    AND source_and_binding_are_valid
    AND all_20_samples_belong_to_the_current_stable_identity_and_binding
    AND their_sequence_and_observation_revision_are_strictly_increasing
    AND history_span >= 1.9 s
    AND norm(v_mean) - 2 * sigma_v_mean >= minimum_directional_speed
    AND fit_rms <= 3 * sqrt(2) * max(last_20_position_sigma)
    AND every_derived_value_is_finite
```

현재 `minimum_directional_speed=0.03 m/s`다. 따라서 단순히 정지가 2σ 범위 밖이라는 것만으로
충분하지 않고, 2σ를 뺀 속도 하한이 `0.03 m/s` 이상이어야 한다. 이 식은 Noise가 큰 Stress의
느린 Actor가 우연한 표본 평균으로 간헐적으로 lock되는 것을 막고, Normal 공개 same-direction
wide 5개가 eventual `READY`가 되는지를 공개 gate로 확인한다.

`fit_rms`는 최신 `observed_position`을 anchor로 `v_mean`을 과거 시각까지 역투영한 20개 예측
위치와 실제 관측 위치의 RMS 오차다. 이 잔차 gate는 평균 속도만 우연히 통과한 비일관 이력을
`LOW_CONFIDENCE`로 거부한다.

### 4.2 confidence 전과 상실 뒤 동작

활성 Actor가 하나라도 방향 confidence를 얻지 못했다면 directional capsule을 만들지 않는다.

```text
direction_unconfirmed
→ non-zero local motion proposal 금지
→ 제한 감속
→ 실제 정지 뒤 hold
```

confidence를 얻기 전의 정지는 planner deadlock이 아니라 판단 불충분에 따른 safety hold다.
confidence를 잃거나 추적 이력이 reset되면 같은 fail-closed 절차를 다시 적용한다. 이전 방향을
관성이라는 이유로 무기한 재사용하지 않는다.

## 5. 방향성 reachable capsule

### 5.1 시간축

```text
A_snapshot = control_snapshot_time - observation_timestamp
L_apply = 0.05 s
u = post-apply rollout time
tau = A_snapshot + L_apply + u
```

rollout과 terminal stopping의 모든 sample은 각자의 `tau`를 사용한다.

### 5.2 종방향 이동 구간

방향 confidence가 성립하면 `h = v_mean / norm(v_mean)`, `s0 = norm(v_mean)`으로 둔다.
여기서 `v_mean`은 최신 20개 unique accepted `observed_velocity`의 산술평균이다. capsule의
anchor는 회귀선 위치가 아니라 **최신 accepted frame의 `observed_position`** 이다.
현재 합성 Actor의 최대속도와 가속도 상한은 기존 값인 `v_max=0.50 m/s`,
`a_max=0.50 m/s²`를 유지한다.

최소 이동은 현재 진행방향을 유지한 채 제한 감속해 정지한 경우다.

```text
t_stop = s0 / a_max

if tau <= t_stop:
    d_min(tau) = s0*tau - 0.5*a_max*tau²
else:
    d_min(tau) = s0² / (2*a_max)
```

최대 이동은 제한 가속해 최대속도에 도달한 경우다.

```text
t_max = max(0, (v_max-s0)/a_max)

if tau <= t_max:
    d_max(tau) = s0*tau + 0.5*a_max*tau²
else:
    d_max(tau) =
        s0*t_max + 0.5*a_max*t_max²
        + v_max*(tau-t_max)
```

입력 `s0`는 기존 속도 상한 규칙으로 `v_max` 이하로 제한한다. 모든 경우에
`0 <= d_min <= d_max`여야 한다. 정지한 Actor가 같은 heading 축의 뒤쪽으로 자동 이동하거나,
한 rollout 안에서 진행방향을 뒤집는 경우는 현재 corpus 계약에 포함하지 않는다.

### 5.3 측면 범위와 2σ 팽창

현재 공개 corpus는 Actor 방향 전환을 포함하지 않으므로 다음 값을 사용한다.

```text
lateral_turn_bound(tau) = 0
```

이는 실제 사람이 방향을 바꾸지 못한다는 뜻이 아니다. 현 공개 corpus로 검증 가능한 합성
가정만 명시한 것이다.

최신 위치관측을 `p_obs`라고 하면 capsule 중심선은 다음 선분이다.

```text
p_obs + h*d_min(tau)
→ p_obs + h*d_max(tau)
```

20개 속도표본이 독립이라는 현재 합성 관측 계약에서 속도 불확실성은 평균에 한 번만 반영한다.
위치 불확실성은 최신 frame의 현재 `position_sigma`를 사용한다.

```text
sigma_p(tau) = sqrt(
    current_position_sigma²
    + (tau * max(last_20_velocity_sigma) / sqrt(20))²
)

capsule_radius(tau) =
    actor_radius
    + 2*sigma_p(tau)
    + lateral_turn_bound(tau)
```

`d_min`과 `d_max`에는 `s0`의 제한 감속·가속만 사용한다. 속도 불확실성을 endpoint 속도에도
더하고 반경에도 다시 더하는 이중 계산은 하지 않는다. 속도 불확실성은 위 radius에 한 번만
들어간다. Actor의 종방향 가감속 범위도 capsule 중심선 길이에 반영하므로 같은 이동을 원형
반경에 다시 더하지 않는다.

wheelchair oriented footprint와 **정확한 Capsule** 사이의 표면거리가 canonical 안전 기하다.
기존 원형 API와의 비교·호환을 위한 circle-chain 표본은 보조 표현일 뿐, online 판정이나 v7
자격의 정본 기하가 아니다.

```text
surface_distance(
    wheelchair_oriented_footprint(tau),
    actor_reachable_capsule(tau)
) >= 0.08 m
```

capsule 계산이 non-finite이거나 기하 판정이 정의되지 않으면 해당 trajectory는 불법이다.

## 6. 2σ의 의미와 관측 한계

`2σ`는 이 합성 비교실험의 휴리스틱 팽창값이다. 확률적 안전 보장, 실제 사람 reachable set
또는 모든 Gaussian 표본의 포함을 뜻하지 않는다.

- Normal: 위치 `σ=0.03 m`, 속도 `σ=0.05 m/s`, 지연 `0.10 s`, dropout `5%`
- Stress: 위치 `σ=0.08 m`, 속도 `σ=0.15 m/s`, 지연 `0.25 s`, dropout `20%`
- x/y noise는 서로 독립인 무클리핑 Gaussian이다.

2차원 Gaussian에서 반지름 `2σ`는 흔히 오해하는 1차원 95% 경계와 동일하지 않으며, 무클리핑
Gaussian은 어떤 유한한 σ 배수로도 모든 미래 seed를 보장할 수 없다. 공개 감사에서 20-frame
평균 constant-velocity 중심에 대한 `2σ` 포함률도 Normal·Stress 각각 약 85.6%와 85.8%였다.
따라서 다음을 지킨다.

- online capsule과 독립적인 200 Hz ground-truth evaluator를 유지한다.
- Stress에서 confidence가 늦어지거나 사라지면 우회 성공률을 높이기 위해 기준을 낮추지 않는다.
- 실제 clearance `0.08 m` 미만은 예측이 통과했더라도 hard failure다.
- public 결과를 본 뒤 σ 배수나 noise를 맞추지 않는다.

모든 관측 표본을 수학적으로 포함해야 하는 새 실험을 원한다면, 무클리핑 Gaussian 대신 사전에
동결한 bounded-noise 계약과 새 corpus·manifest를 사용해야 한다.

## 7. 이력 reset 계약

다음 사건에서는 해당 Actor의 20-frame 이력과 direction confidence를 즉시 폐기한다.

- stream·episode·map identity 또는 map revision 변경
- track ID의 actor binding 변경
- sequence·observation revision 역행
- hash 불일치, invalid source 또는 stale observation
- fresh empty frame으로 Actor 소멸 확인
- Actor가 사라졌다가 같은 ID로 다시 나타남
- 속도 불확실성에 정지가 포함되거나 유효 heading을 만들 수 없음
- 정지 뒤 재출발, 방향반전 또는 새 motion segment가 명시된 future corpus 사건

향후 `motion_segment_revision`을 controller 입력에 추가하면 그 값의 변경·역행도 reset 원인에
포함한다. 현재 존재하지 않는 revision을 ground truth나 evaluator에서 몰래 읽어 reset하지 않는다.

`track_id`가 같은 `actor_binding_id` 안에서 바뀌거나 binding 관계가 바뀌어도 이전 방향 증거를
즉시 폐기한다. invalid와 stale은 predictor 전체 session state를 reset한다.

단일 dropout은 accepted frame 수를 늘리지 않는다. 마지막 snapshot이 TTL 안에서 fresh더라도
새 방향 증거로 세지 않으며, TTL을 넘으면 반드시 reset·정지한다. reset 뒤에는 과거 20개를
재사용하지 않고 새로 수집한다.

## 8. 공개 검증 순서

### 8.1 단위·계약시험

- 같은 frame을 여러 control tick에서 재사용해도 accepted count가 증가하지 않음
- 19개 frame에서는 항상 fail-closed, 20번째 unique frame 뒤에만 confidence 평가
- future frame이나 아직 도착하지 않은 frame을 사용하지 않음
- confidence 미성립·상실·reset에서 새 비영점 명령 0
- 감속·정지 종방향 하한과 가속·최대속도 상한의 해석적 검증
- 모든 `tau`에서 뒤쪽 이동과 방향반전 없음
- capsule 회전·병진 metamorphic 동등성
- Actor별 이력이 다중 Actor 사이에서 섞이지 않음

### 8.2 v6 공개 13개 회귀

Normal과 Stress에서 다음을 모두 확인한다.

- `same-direction-wide-r00~r04`: confidence 뒤 실제 이탈·안전 통과·추월·재합류 후보 존재
- `same-direction-narrow`: 좁은 공간에서 측면 통과 0
- `offset-head-on`: 정면 Actor를 같은 방향으로 잘못 분류하지 않고 대기
- diagonal horizontal·vertical rigid pair: 90° 회전 뒤 동일한 판정
- corner·second-risk: 로봇 경로 회전과 Actor 방향을 혼동하지 않음
- simultaneous·staggered multi-Actor: track별 이력과 reset 독립성

기존 public label과 witness를 자동 정답으로 간주하지 않는다. 특히
`LOCAL_DETOUR_FEASIBLE` 5개 witness는 다음 전체 구간으로 다시 검증한다.

```text
20-frame confidence 전 fail-closed 구간
+ command apply delay
+ 매 control tick의 2.0 s rollout
+ 각 후보의 terminal stopping sweep
+ 200 Hz actual ground-truth clearance
```

하나라도 통과하지 못하면 해당 witness를 근거로 우회 가능이라고 주장하지 않는다. 안전식을
줄이는 대신 공개 corpus의 분류·witness 또는 합성 운동 계약을 새 버전에서 다시 결정한다.

## 9. 미래 방향전환 corpus

현재 corpus는 `lateral_turn_bound > 0`을 검증할 수 없다. 실제 방향전환 연구는 hidden 전에 새
public mechanism corpus로 다음을 추가한다.

- 현재 heading에서 완만하게 `30°`, `45°`, `90°` 회전
- 동결된 heading-rate·lateral-acceleration 경계 바로 안과 밖의 사례
- 정지·재출발
- 동일 진행축에서의 방향반전
- 회전 도중 dropout·stale·identity 변경
- 방향전환 중 새 위험 발생과 재정지

각 사례는 waypoint 사이의 속도 연속성, 최대 heading rate, 종·횡가속도를 validator가 직접
검사해야 한다. 경계를 넘는 움직임은 예측 범위를 조용히 넓히지 않고 confidence reset과
fail-closed 정지를 유발해야 한다.

비영점 `lateral_turn_bound`, 방향 estimator window 또는 confidence 기준을 바꾸면 새 manifest와
public corpus hash를 만든다.

## 10. hidden lifecycle

- 이 명세 구현·단위시험·전체 공개 재검증이 끝나기 전에는 새 hidden을 생성하거나 실행하지 않는다.
- 기존 v5/v6 hidden과 과거 결과를 v7 최종 증거로 재사용하지 않는다.
- public 결과를 보고 estimator, capsule, σ, Actor motion 또는 witness를 바꾸면 기존 결과는
  regression으로만 보존한다.
- 코드·파라미터·public corpus·witness·manifest를 다시 동결한 뒤에만 새 hidden seed commitment를
  만들 수 있다.
- hidden을 확인한 뒤 변경하면 해당 hidden은 즉시 regression으로 전환하고 새 commitment가
  필요하다.

## 11. 완료조건과 결론 제한

### 현재 공개-only 구현·시험 상태

- 방향 예측과 공개-only 자격의 targeted 시험은 `33 passed`로 완료했다.
- Normal `same-direction-wide-r00~r04`는 최신 20개 unique accepted frame 뒤 eventual
  `READY`를 확인했다.
- Stress의 같은 저속 Actor 5개는 `READY`가 0이고, 모든 non-READY TRACKS 결과가 hold이며
  prediction을 노출하지 않았다. stale reset 때문에 각 episode가 반드시 20개를 모으는 것은
  요구하지 않으며, 공개 corpus 전체에서 `LOW_CONFIDENCE`가 실제 발생하는 것도 확인했다.
- 공개-only 기하시험은 각 사례에서 217개 DWB action primitive의 2.0초 rollout 41 pose와
  terminal stopping을 exact Capsule로 계산하고 결정론을 확인한다. 이는 기존 `0.35 s` witness
  공백을 닫는 **기하 계산 자격**이다.
- 실제 closed-loop DWB가 이탈·통과·추월·재합류하는 legal bypass는 아직
  `ONLINE_DWB_BYPASS_UNPROVEN`이다. 기하 계산 자격을 우회 기능 합격으로 바꾸어 쓰지 않는다.
- v7 hidden은 생성·열람·실행하지 않았다.

이는 estimator와 exact Capsule의 공개-only 하위 자격 완료다. 아래의 공개 13개 전체 폐루프,
200 Hz 실제 clearance, 실제 detour·overtake·rejoin과 manifest 자격은 아직 남아 있다.

다음을 모두 만족해야 directional prediction을 공개 비교에 사용할 수 있다.

- 20-frame estimator의 인과성·유일성·reset 시험 통과
- confidence 전·상실 뒤 비영점 추진 0
- 종방향 no-reverse capsule oracle 통과
- 공개 13개×Normal·Stress hard safety와 기능 oracle 통과
- 공개-only exact Capsule full rollout·terminal 계산 자격 통과
- feasible witness와 online DWB의 실제 detour·overtake·rejoin 자격 통과
- 200 Hz 실제 clearance 위반 0
- 결정론·paired stream·manifest hash 유지

통과 결과의 의미는 다음으로 제한한다.

> 방향이 고정된 open-loop 합성 Actor와 동결된 관측조건에서, 방향 confidence 뒤 종방향
> no-reverse capsule을 사용한 국소 우회의 연구 가능성을 확인했다.

이 결과를 실제 사람의 관성, 실제 병원 안전성, 최종 제품 알고리즘 또는 사람 탑승 허가로
확대하지 않는다.
