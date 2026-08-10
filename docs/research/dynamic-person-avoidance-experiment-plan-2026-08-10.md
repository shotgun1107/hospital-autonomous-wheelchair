# 움직이는 원형 Actor 회피 비교실험 v5

## 문서 상태

- 상태: **동결 승인 — 개인 연구용 비교실험 명세**
- 작성 기준일: 2026-08-10
- 구현 상태: 1단계 동적 Actor·20 Hz trace·JSON/PNG 기반 구현 완료, 2~6단계 미구현
- 증거 범위: `simulation_only`
- 비교 대상: PP 경로추종 기준선과 사용자 정의 DWA 국소 우회
- 제품 결정: 아님
- 팀 합의: 아님
- 경로 분석 7단계 및 G1~G5 결정: 수행하지 않음

이 문서는 움직이는 사람을 단순화한 open-loop 원형 Actor를 이용해 두 local 주행
방식의 trade-off를 비교하기 위한 연구 계획이다. 실제 사람 행동, 실제 센서 지연,
ROS 2 종단 통합, 실차 제동, 사람 탑승 안전성을 증명하지 않는다. 제품 경로 전략의
현재 결정 상태는
[조건부 권고 문서](../product/path-planning-conditional-recommendation.md)를 우선한다.

이 동결 요구사항을 코드로 옮기는 전체 구조와 단계별 완료조건은
[동적 원형 Actor 비교실험 설계 명세](dynamic-actor-experiment/README.md)에서 관리한다.

---

## 먼저 할 일: 1시간 컷 1차 작업

첫 작업에서는 알고리즘을 수정하지 않고 **동적 시험환경의 최소 골격만 구현**한다.

### 포함

- `MovingActor`의 위치, 속도, 반지름 모델
- 20 Hz 고정 simulation tick
- seed 기반 결정론적 Actor 궤적
- 복도를 가로지르는 단일 Actor 시나리오 1개
- evaluator용 ground truth와 controller 입력의 자료 구조 분리
- 로봇·Actor 시간 이력 기록
- JSON 결과와 경로 PNG 생성
- 같은 seed에서 궤적·사건·지표가 재현되는 시험
- PP와 DWA를 나중에 연결할 adapter 자리
- 관련 단위시험, 문서, 커밋과 원격 push

### 제외

- PP와 DWA의 실제 closed-loop 비교
- 관측 지연·noise·dropout
- safety gate와 `stop_epoch` 전체 구현
- contract-fault corpus
- 200 Hz swept evaluator
- development·hidden 대량 corpus
- 통계·승격 판정
- 50 ms 성능 최적화
- ROS 2·Unity·실물 연동

### 1시간 종료 조건

```text
같은 seed
→ 같은 Actor ground-truth trajectory
→ 같은 controller-input frame sequence
→ 같은 상태·사건 결과
→ JSON 및 PNG 생성
→ 자동시험 통과
→ 원격 브랜치에 push
```

시간이 부족하면 시나리오 수를 늘리지 않는다. 재현성 시험과 ground truth/입력 분리는
삭제하지 않는다.

### 후속 분할

1. 1차: 동적 시험환경 골격
2. 2차: PP·DWA closed loop와 공통 safety gate
3. 3차: 관측 열화, 권한·deadline fault, 200 Hz evaluator
4. 4차: development·hidden corpus, 성능 qualification, 승격 판정

---

## 1. 실험 질문

정확한 비교 질문은 다음과 같다.

> 같은 A*·Grid A* 기준 경로, 같은 차체, 같은 관측 stream, 같은 online safety
> gate에서 PP 경로추종+정지·대기 기준선과 DWA 국소 우회+정지·대기 비교군은
> 안전성, 완료시간, 대기시간, 경로 이탈, 승차감과 계산비용에서 어떤 차이를 보이는가?

```text
PP 기준선
A* → Grid A* → PP path tracking → shared safety-gate stop/hold

DWA 비교군
A* → Grid A* reference → custom DWA local detour
   → shared safety-gate stop/hold
```

- PP는 동적 Actor를 피해 스스로 기준 경로에서 벗어나지 않는다.
- DWA는 같은 기준 경로 주위에서 제한적인 local detour를 시도한다.
- 두 방식 모두 최종 명령을 같은 safety gate에 통과시킨다.
- safety gate는 공통 online command filter이지 독립적인 중복 안전채널이 아니다.
- 실제 안전 결과는 controller 입력에 없는 ground-truth evaluator가 별도로 판정한다.
- Actor는 로봇을 보고 양보하거나 회피하지 않는 사전 정의 open-loop 궤적을 따른다.

이 결과로 제품 알고리즘의 최종 우열이나 실제 사람의 안전·승차감을 주장하지 않는다.

## 2. 비교 공정성

- PP와 DWA의 자유주행 목표속도는 모두 `0.20 m/s`이다.
- `0.30 m/s`는 가상 차체의 물리적 전진 상한일 뿐 비교 목표속도가 아니다.
- 동일 map, mission, reference path, Actor ground truth, 관측 seed를 사용한다.
- expectation category와 scenario label은 evaluator와 runner만 사용한다.
- PP, DWA, safety gate 입력에는 정답 category를 넣지 않는다.
- Normal·Stress는 같은 scenario seed를 controller별로 paired 실행한다.

## 3. 공통 가상 차체와 Actor

| 항목 | 동결값 |
|---|---:|
| collision footprint | 폭 `0.36 m`, 길이 `0.44 m` oriented rectangle |
| controller 목표·최대속도 | `0.20 m/s` |
| 물리 전진 상한 | `0.30 m/s` |
| 물리 후진 상한 | `0.10 m/s` |
| 선형 가속 | `0.25 m/s²` |
| 선형 감속 | `0.50 m/s²` |
| 각속도 상한 | `0.80 rad/s` |
| 각가속·각감속 | `1.60 rad/s²` |
| 제어주기 | `20 Hz`, `dt=0.05 s` |
| 실제 최소 표면 clearance | `0.08 m` |
| Actor 반지름 | `0.18 m` |
| Actor 최대속도 | `0.50 m/s` |
| Actor 최대가속도 | `0.50 m/s²` |
| DWA 후진 | 비활성화 |

모든 값은 가상 비교실험용이며 실제 차체 또는 제품 임계값이 아니다.

## 4. PP 동결 계약

- lookahead: `0.35 m`
- 현재 pose를 reference polyline의 가장 가까운 선분에 투영한다.
- 동률이면 누적 arc length가 작은 투영점을 선택한다.
- 투영점에서 polyline을 따라 `0.35 m` 앞의 점을 lookahead로 선택한다.
- 남은 경로가 더 짧으면 goal을 선택한다.
- goal tolerance: `0.05 m`

```text
v_goal = min(
    0.20,
    sqrt(2 · 0.50 · max(0, remaining_arc_length - 0.05))
)

kappa = 2 · y_local / lookahead_distance²
omega_raw = clip(v_goal · kappa, -0.80, +0.80)
```

최종 명령에는 공통 선형·각속도 acceleration limiter를 적용한다.

## 5. DWA 동결 계약

### 5.1 동적 window와 sampling

```text
v_min = max(0, current_v - 0.50 · 0.05)
v_max = min(0.20, current_v + 0.25 · 0.05)
w_min = max(-0.80, current_w - 1.60 · 0.05)
w_max = min(+0.80, current_w + 1.60 · 0.05)
```

- 선속도 sample: 7개
- 각속도 sample: 31개
- 최대 후보: `7 × 31 = 217`개 `(v, w)` 명령쌍
- rollout horizon: `2.0 s`
- 적분 간격: `0.05 s`
- 후보 하나는 rollout 동안 같은 `(v, w)`를 유지한다.
- 후보당 40개 적분 구간, 초기상태 포함 41개 pose sample을 사용한다.
- rollout 뒤 terminal stopping sweep를 추가한다.
- `0`이 구간 안에 있지만 sample에 정확히 없으면 가장 가까운 내부 sample 하나를
  `0`으로 교체한다. sample을 추가하지 않는다.

### 5.2 비용함수

```text
raw_progress = distance(start, goal) - distance(terminal, goal)
progress_cost = 1 - clip(raw_progress / (0.20 · 2.0), 0, 1)

reference_path_cost = clip(
    mean(distance(each rollout pose, nearest polyline segment)) / 0.50,
    0,
    1
)

heading_cost = clip(
    abs(normalize(atan2(goal-terminal) - terminal_yaw)) / pi,
    0,
    1
)

clearance_cost = 1 - clip(
    (minimum_surface_clearance - 0.08) / (0.50 - 0.08),
    0,
    1
)

speed_cost = clip((0.20 - v_candidate) / 0.20, 0, 1)

oscillation_cost = 1
    if previous |w| > 0.05
    and candidate |w| > 0.05
    and signs are opposite
    else 0

score =
    1.0 · progress_cost
  + 1.0 · reference_path_cost
  + 0.5 · heading_cost
  + 1.5 · clearance_cost
  + 0.2 · speed_cost
  + 0.3 · oscillation_cost
```

- 낮은 score가 우선이다.
- 장애물이 없어 `minimum_surface_clearance=+∞`이면 `clearance_cost=0`이다.
- NaN 또는 정의되지 않은 geometry 결과는 후보 무효다.
- 동률 순서: score 오름차순 → clearance 내림차순 → progress 내림차순 →
  reference·heading·oscillation cost 오름차순 → `abs(w)` 오름차순 →
  `v` 내림차순 → signed `w` 오름차순.
- rollout과 terminal stopping sweep가 static grid, forbidden zone, Actor tube에 대해
  모두 안전한 후보만 admissible하다.
- admissible 후보가 없으면 `NO_PATH/no_safe_candidate`를 반환한다.

## 6. 관측 프로필

| 프로필 | 관측 주기 | 지연 | TTL | 위치 `σp0` | 속도 `σv` | frame dropout |
|---|---:|---:|---:|---:|---:|---:|
| Normal | `10 Hz` | `100 ms` | `300 ms` | `0.03 m` | `0.05 m/s` | `5%` |
| Stress | `10 Hz` | `250 ms` | `300 ms` | `0.08 m` | `0.15 m/s` | `20%` |
| Boundary | `10 Hz` | `300/350 ms` | `300 ms` | `0` | `0` | `0` |

- Normal과 Stress의 위치 x/y, 속도 x/y 오차는 서로 독립인 Gaussian이다.
- noise를 clipping하지 않는다.
- dropout은 frame별 독립 Bernoulli이다.
- 4-frame burst dropout은 별도의 contract-fault case다.
- `delay == TTL`과 `delay > TTL`은 성능 Stress가 아니라 watchdog boundary 시험이다.
- 마지막 유효 관측은 `age > TTL`이 될 때까지 timestamp와 함께 유지한다.
- fresh empty frame과 no-frame/dropout을 구분한다.

## 7. Actor 예측과 online clearance

```text
t_control = controller snapshot simulation time
t_obs = observation timestamp
A_snapshot = t_control - t_obs
L_apply = 0.05
u = post-apply rollout time
tau = A_snapshot + L_apply + u

if norm(observed_velocity) <= 0.50:
    v_hat = observed_velocity
else:
    v_hat = 0.50 · observed_velocity / norm(observed_velocity)

predicted_center(u) = observed_position + v_hat · tau
sigma_p(u) = sqrt(sigma_p0² + (tau · sigma_v)²)

delta_v_cap = 0.50 + norm(v_hat)
t_delta = delta_v_cap / 0.50

if tau <= t_delta:
    d_accel(tau) = 0.5 · 0.50 · tau²
else:
    d_accel(tau) =
        0.5 · 0.50 · t_delta²
        + delta_v_cap · (tau - t_delta)

actor_tube_radius(u) =
    0.18 + 2 · sigma_p(u) + d_accel(tau)
```

online 계약은 다음 하나로 통일한다.

```text
surface_distance(
    wheelchair_oriented_footprint(u),
    predicted_actor_tube_circle(u)
) >= 0.08 m
```

현재 운동, 명령 적용 지연, 후보 rollout, terminal stopping 전 구간을 time-indexed swept
geometry로 검사한다. 별도의 scalar `required_clearance`를 같은 Actor 이동에 중복 적용하지
않는다. `2σ`와 reachable tube는 연구용 보수 모델이며 확률적 안전 보장이나 실제 사람의
완전한 reachable set을 뜻하지 않는다.

## 8. 공통 safety gate와 재출발 권한

### 8.1 제한 감속

```text
v_next = sign(v) · max(0, abs(v) - 0.50 · 0.05)
w_next = sign(w) · max(0, abs(w) - 1.60 · 0.05)
```

stale, invalid source, deadline 초과에서는 새로운 비영점 선속도·각속도 명령, 속도
크기 증가, 방향 반전을 금지한다. 현재 속도를 0으로 줄이는 제한 감속만 허용한다.

### 8.2 실제 정지와 상태

- 정지 완료: `|v| <= 0.01 m/s`와 `|w| <= 0.02 rad/s`가 3 tick 연속 유지
- `motion_state`: `MOVING / BRAKING / HOLDING / COMPLETED`
- `stop_epoch`: 서로 다른 보호정지가 처음 `STOP_CONFIRMED`로 전이할 때 1회 증가
- 같은 hold가 지속되는 동안 증가하지 않는다.
- 정상 목적지 도착은 보호정지 epoch를 만들지 않는다.

`primary_hold_reason` 우선순위:

```text
INVALID_SOURCE > STALE > DEADLINE > UNAUTHORIZED
> GATE_REJECTION > NO_SAFE_CANDIDATE > TRAFFIC
```

controller stop request, gate override, candidate rejection은 상태와 섞지 않고 별도 event
counter로 기록한다. `planner_deadlock`은 evaluator verdict로 기록한다.

### 8.3 재출발

```text
resume_authorization_valid =
    command.mission_id == current_mission_id
    AND command.stop_epoch == current_stop_epoch
    AND command.issued_or_revalidated_at >= actual_stop_confirmed_at
    AND command.authorization_revision == current_authorization_revision

resume_allowed =
    resume_authorization_valid
    AND path_still_valid
    AND local_safety_recheck_passed
    AND continuous_safe_duration >= 1.0 s
    AND observation_is_fresh
    AND observation_source_valid
```

1초 안전관측은 10 Hz에서 새로운 safe frame 11개가 연속으로 들어온 것으로 정의한다.
dropout, invalid, unsafe frame이 오면 누적을 초기화한다. 위험 해소, 경로 생성, 과거 이동
허가만으로 자동 재출발하지 않는다.

## 9. 시간 의미와 deadline

### 9.1 결정론적 기능·안전 lane

각 simulation timestamp의 순서는 다음으로 고정한다.

1. 이전 명령으로 ground truth 적분
2. 도착한 observation 전달
3. source 계약 검증
4. controller snapshot 생성
5. controller 실행
6. safety gate 검사
7. 다음 tick actuator queue 반영

- accepted command에는 simulation-time `L_apply=50 ms`를 주입한다.
- 모든 결과에 tick ID를 붙인다.
- 늦은 과거 tick 결과는 폐기하며 다음 tick에 재사용하지 않는다.
- boundary corpus에서는 observation과 control 순서를 뒤집어도 안전하게 stale 처리되는지
  별도로 시험한다.

### 9.2 실제 연산시간 qualification lane

- frozen snapshot에서 physics clock을 멈추고 측정한다.
- 포함: frame 검증, track 변환, Actor 예측, PP/DWA, gate, 최종 명령
- 제외: ground truth 적분, evaluator, global replanning, 시각화, 파일 I/O
- 한 process, 고정 CPU affinity, numeric thread 1개
- warm-up 30회, 측정 100회
- `<=50 ms` 결과만 현재 tick에 유효하며 `>50 ms` 결과는 폐기한다.
- 49/50/51 ms 강제 결과와 old/new 결과 순서 역전을 contract-fault로 시험한다.

qualification manifest에는 `qualification_snapshot_set_hash`를 넣고 Actor 0명, 1명,
2명, 최대 Actor tube와 static geometry가 함께 있는 development 최악 사례를 포함한다.

## 10. evaluator와 corpus

### 10.1 ground-truth evaluator

- 평가 주기: `200 Hz`, `5 ms`
- 매 구간에서 보수적인 oriented swept footprint를 사용한다.
- Normal과 Stress 모두 실제 표면 clearance `>=0.08 m`를 hard criterion으로 둔다.
- stale·invalid에서는 즉시 물리속도 0을 요구하지 않고 새 추진·회전 금지, 제한 감속,
  정지 뒤 hold를 요구한다.
- 재합류: 먼저 reference path에서 `>0.10 m` 이탈한 뒤 path distance `<=0.10 m`,
  heading error `<=10°`를 `0.5 s` 유지한다.
- 추월: Actor와 로봇의 reference-path 투영 순서가 뒤바뀌고 통과 중 두 footprint의
  종방향 투영 범위가 겹친 경우로 판정한다.

### 10.2 기대 범주

| 범주 | 기대 동작 |
|---|---|
| `WAIT_AND_RESUME` | 양쪽 모두 안전정지 후 원 경로 재개 |
| `LOCAL_DETOUR_FEASIBLE` | DWA 우회 가능, PP는 대기 |
| `LOCAL_DETOUR_FORBIDDEN` | 양쪽 모두 추월·우회 금지, 해소 뒤 재개 가능 |
| `NO_SAFE_SOLUTION` | 양쪽 모두 정지 유지 |
| `OBSERVATION_INVALID` | 양쪽 모두 stale·invalid 정지, 복구 뒤 재검증 |
| `DYNAMIC_CHANGE_RESTOP` | 우회 또는 재개 중 새 위험에 다시 정지 |

`NO_SAFE_SOLUTION`에서도 open-loop Actor가 정지한 로봇과 최소 clearance를 침범하지
않도록 corpus를 검증한다.

### 10.3 corpus 규모

- golden: 6개 고정 mechanism test
- development: 6개 범주 × paired seed 5개 = 30개
- hidden: 6개 범주 × paired seed 5개 = 30개
- Normal·Stress는 같은 scenario seed 쌍을 사용한다.
- Boundary와 contract-fault는 성능통계에서 제외한다.

Normal의 progressable episode는 마지막 유효 차단 조건이 해소된 뒤 30초 안에 목적지에
도달해야 하며 `planner_deadlock=0`이어야 한다. 적용 대상은 `NO_SAFE_SOLUTION`을 제외한
복구 가능한 범주다. 정당한 traffic/safety hold는 deadlock으로 세지 않는다.

## 11. contract-fault 자격시험

성능 corpus와 분리하여 PP와 DWA 시스템 모두에 다음 음성시험을 적용한다.

### 관측 계약

- 잘못된 stream·episode seed·map ID
- sequence·revision 역행
- hash 불일치
- 중복 track ID와 actor binding 변경
- fresh empty frame과 no-frame 구분
- 단일 dropout과 4-frame burst
- `age == TTL`, `age > TTL`

### 권한 계약

- 이전 `stop_epoch`
- 다른 mission ID
- 실제 정지 전 발행 authorization
- 잘못된 authorization revision
- authorization 없음
- 승인 뒤 새 보호정지
- 위험 해소만 있고 새 승인 없음

### deadline 계약

- 강제 49 ms, 50 ms, 51 ms
- 늦은 결과의 다음 tick 도착
- 늦은 과거 결과와 최신 결과의 순서 역전

공통 hard safety 또는 fault 시험이 하나라도 실패하면 PP/DWA 성능 우열을 해석하지
않는다. 원인을 수정한 뒤 이미 본 hidden은 regression으로 전환한다.

## 12. manifest와 hidden 운영

실행 전 다음 항목을 hash로 동결한다.

- code commit
- map·corpus
- PP parameter
- DWA parameter와 비용함수
- safety gate
- observation generator
- scenario generator
- simulator version
- hidden seed commitment
- 실행 machine identifier
- controller별 tuning 및 development 전체 접근 횟수
- `qualification_snapshot_set_hash`

controller별 development 전체 평가는 최대 3회다. hidden을 확인한 뒤 code, parameter,
corpus를 바꾸면 기존 hidden은 regression으로 바꾸고 새 비공개 seed commitment로
hidden-v2를 만든다. 같은 hidden을 최종 성능 증거로 재사용하지 않는다.

## 13. 통계와 연구 기준선 승격

```text
S_progress =
    Normal hidden 중 사전에 progressable로 지정된
    모든 paired episode ID
```

`NO_SAFE_SOLUTION`은 완료시간 모집단에서 제외한다.

```text
time_improvement =
    1 - median(T_DWA over S_progress) / median(T_PP over S_progress)

hold_improvement =
    1 - median(Hold_DWA over S_progress) / median(Hold_PP over S_progress)
```

- `time_improvement >= 0.15` 또는 `hold_improvement >= 0.20`을 요구한다.
- PP median hold가 0이면 hold 개선율은 정의하지 않는다.
- 선택된 동일 지표와 동일 episode 집합에서 범주별 paired seed를 유지하는 10,000회
  stratified bootstrap을 사용하며 paired delta 95% CI upper bound가 0보다 작아야 한다.

승차감 지표 악화율:

```text
worsening(m) =
    (median(m_DWA) - median(m_PP))
    / max(abs(median(m_PP)), denominator_floor(m))
```

| 지표 | denominator floor |
|---|---:|
| longitudinal jerk RMS | `0.10 m/s³` |
| angular acceleration RMS | `0.10 rad/s²` |
| angular jerk RMS | `0.10 rad/s³` |

세 지표가 각각 `worsening <= 0.25`여야 한다.

### DWA 연구 기준선 승격 조건

1. PP와 DWA 모두 Normal·Stress hard safety 통과
2. 공통 contract-fault corpus 통과
3. 두 방식 모두 `S_progress` 기능 자격 통과
4. Normal wall-clock deadline miss 0, late command 적용 0
5. DWA가 `LOCAL_DETOUR_FEASIBLE`의 80% 이상에서 우회·재합류
6. forbidden·no-safe-solution 추월 0
7. 완료시간 15% 또는 hold 20% 개선과 paired CI 조건 충족
8. 세 승차감 악화율 각각 25% 이하
9. positive detour length가 reference의 30% 이하, 최대 이탈 `0.50 m` 이하
10. DWA 비영점 이동 제안 tick을 분모로 gate override 5% 이하,
    최대 연속 override 3 tick 이하

모두 통과하면 DWA를 **동결된 시뮬레이션 조건의 다음 연구 기준선**으로만 승격한다.
하나라도 미달하면 PP+gate를 연구 기준선으로 유지하고 DWA는 비교·개선 후보로 남긴다.

## 14. 결과 해석 금지선

이 실험은 다음을 확정하지 않는다.

- 실제 환자 탑승 안전성
- 실제 병원 사람 행동과 상호작용
- 실제 센서, 네트워크, actuator 지연
- ROS 2와 하드웨어 gateway 종단 안전성
- 제품 알고리즘 최종 채택
- G1~G5 또는 경로 분석 7단계 팀 결정
- DWA 일반 또는 Nav2 DWB 전체의 우월성

Actor tube가 보수적이어서 우회가 거의 나오지 않으면 현재 관측·운동 불확실성 아래에서
국소 우회의 근거가 부족하다는 결과로 기록한다. hidden 결과를 보고 tube, clearance,
가속 상한을 낮추지 않는다.

## 15. 검토 근거

- [Nav2 Tuning Guide](https://docs.nav2.org/tuning/index.html)
- [The Dynamic Window Approach to Collision Avoidance](https://publications.ri.cmu.edu/storage/publications/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf)
- [Nav2 Collision Monitor](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)

외부 자료는 알고리즘 성격과 시험 구조의 참고 근거다. 이 저장소의 사용자 정의 DWA가
원 논문 또는 Nav2 DWB 구현과 같다는 뜻은 아니다.

## 16. 집 PC 재동기화

집 PC에서 기존 변경을 먼저 확인하고, 깨끗한 경우에만 fast-forward pull한다.

```powershell
git status --short
git fetch origin
git switch codex/path-planning-python-lab
git pull --ff-only
```

`git status --short`에 출력이 있으면 로컬 변경을 덮어쓰지 말고 먼저 commit 또는 stash로
보존한다. 이 문서와 1차 시험환경 작업의 정본 브랜치는
`origin/codex/path-planning-python-lab`이다.
