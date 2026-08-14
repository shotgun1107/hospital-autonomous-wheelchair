# 공개 DWA·DWB 소스 분석과 프로젝트 적용 설계

## 1. 목적과 판정 범위

직접 만든 동적 DWA에 예외 처리를 계속 덧붙이는 대신, 실제 ROS에서 공개·사용된 DWA와
그 ROS 2 후속 구조인 DWB를 소스 수준에서 분석하고 우리 Python 실험환경에 맞게 재구현한다.

- 공개 구현을 정답지이자 구조적 기준으로 사용한다.
- 공개 코드를 통째로 복사하거나 현재 실험환경을 즉시 ROS 2로 교체하지 않는다.
- 먼저 Python reference 구현으로 동작을 재현하고 이후 ROS 2 연동 또는 C++ 이식을 검토한다.
- 기존 v6 사용자 정의 DWA 결과는 실패 회귀자료로 보존한다.
- 새 구현은 기존 v6의 단순 수정이 아닌 `source-derived v7` 연구 트랙으로 구분한다.
- 제품 알고리즘 채택, `G1~G5`, 경로 분석 7단계와 실제 사람 탑승 안전성을 결정하지 않는다.

## 2. 동결한 공개 기준 소스

### 2.1 ROS 1 DWA

| 항목 | 값 |
|---|---|
| 저장소 | [`ros-planning/navigation`](https://github.com/ros-planning/navigation) |
| 브랜치 | `noetic-devel` |
| 고정 커밋 | [`f44bb1fc2810399165115cc98b530fe4b9397c18`](https://github.com/ros-planning/navigation/commit/f44bb1fc2810399165115cc98b530fe4b9397c18) |
| 패키지·버전 | `dwa_local_planner`, `base_local_planner` 1.17.3 |
| 라이선스 | BSD |

ROS 1 구현은 전통적인 DWA의 실제 동작과 목표 도착 처리 기준으로 사용한다.

### 2.2 ROS 2 DWB

| 항목 | 값 |
|---|---|
| 저장소 | [`ros-navigation/navigation2`](https://github.com/ros-navigation/navigation2) |
| 브랜치 | `main` |
| 고정 커밋 | [`1e8afb17e2e09df443b1870ce0f4ecdee32207fd`](https://github.com/ros-navigation/navigation2/commit/1e8afb17e2e09df443b1870ce0f4ecdee32207fd) |
| 패키지·버전 | `dwb_core`, `dwb_plugins`, `dwb_critics` 1.5.0 |
| 라이선스 | BSD-3-Clause |

Nav2 DWB는 후보 생성기와 점수 평가기를 분리하므로 Python reference 구현의 주 구조로 사용한다.

## 3. 공개 구현의 실제 흐름

```text
현재 위치·속도와 local reference path
→ 이번 제어주기 안에 도달 가능한 속도 범위 계산
→ (v, w) 후보 생성
→ 후보별 일정 속도 rollout
→ hard constraint 또는 critic으로 불법 후보 제거
→ critic 비용 합산
→ 가장 낮은 비용 후보 선택
→ 선택 명령을 상태형 critic에 통지
→ 목표 근처에서는 별도 감속·정지·회전 처리
```

Nav2 DWB의 수명주기는 다음과 같다.

```text
critic.prepare(...)
→ generator.startNewIteration(current_velocity)
→ nextTwist() / generateTrajectory()
→ critic.scoreTrajectory(...)
→ 최저 점수 후보 선택
→ critic.debrief(selected_command)
```

불법 후보는 낮은 점수를 받는 것이 아니라 즉시 제외된다. 합법 후보의 비용은 critic별
`raw_score × scale` 합이다. 누적 비용이 현재 최선보다 커지면 남은 계산을 조기에 끝낼 수 있다.

## 4. 후보 속도창과 rollout

공개 구현은 전체 rollout 시간 뒤가 아니라 다음 제어주기까지 도달 가능한 속도만 표본으로 만든다.

```text
v_min = max(vehicle_min_v, current_v - deceleration_limit × control_period)
v_max = min(vehicle_max_v, current_v + acceleration_limit × control_period)
w_min = max(vehicle_min_w, current_w - angular_deceleration × control_period)
w_max = min(vehicle_max_w, current_w + angular_acceleration × control_period)
```

우리 differential-drive 차체에서는 `vy=0`으로 고정한다. 0이 범위 안에 있지만 균등 표본에
없다면 공개 iterator처럼 0을 포함할 수 있다. 따라서 v7은 기존 v6의 고정 `7×31=217`을
무조건 유지하지 않고, 고정된 iterator 규칙과 실제 생성 후보 수를 기록한다.

이 규칙에서는 0이 기존 균등 표본에 없을 때 표본 하나를 교체하지 않고 추가한다. 따라서
축별 기본 표본 수는 `7`, `31`이지만 특정 속도창에서는 후보 수가 217개보다 많아질 수 있다.
이는 v6의 최대 217개 계약을 조용히 바꾸는 것이 아니라, 공개 DWB iterator 의미를 따르는 v7의
명시적 차이다. manifest에는 축별 실제 표본과 최종 후보 수를 모두 남긴다.

선택한 `(v, w)`를 rollout 동안 일정하게 유지하고 differential-drive 운동식을 적분한다.
rollout pose에는 상대 시간이 포함돼야 하며, 동적 Actor 제약은 같은 시점의 Actor tube를 검사한다.
generator와 기존 20 Hz simulator는 공개 구현과 같은 순차 Euler 적분을 기준으로 한다. 기존
safety gate의 5 ms 보간·terminal stopping 검사는 후보 생성기의 대체 운동모델이 아니라 최종 명령을
더 촘촘히 제한하는 별도 안전 검사로 유지한다. 회전 궤적 비교에서는 이 두 역할을 섞어
동등하다고 주장하지 않는다.

초기 reference 값:

- 제어 주기 `0.05 s`
- rollout `2.0 s`
- 적분 간격 `0.05 s`
- 초기 pose 포함 후보당 `41 pose`
- 후진 후보 비활성

기준 소스:

이 값은 source-derived v7의 기존 동적 Actor 비교 기준선이다. 2026-08-14의
[`ADR 0014`](../../decisions/0014-section-bound-bounded-reverse-translation.md)는 별도 R5 persistent
reference v2에서 active reverse section에만 제한 후진을 허용한다. v7 결과를 소급 변경하거나 모든
상태에서 자유 후진을 허용하는 결정이 아니다.

- [ROS 1 `SimpleTrajectoryGenerator`](https://github.com/ros-planning/navigation/blob/f44bb1fc2810399165115cc98b530fe4b9397c18/base_local_planner/src/simple_trajectory_generator.cpp)
- [Nav2 `LimitedAccelGenerator`](https://github.com/ros-navigation/navigation2/blob/1e8afb17e2e09df443b1870ce0f4ecdee32207fd/nav2_dwb_controller/dwb_plugins/src/limited_accel_generator.cpp)
- [Nav2 `OneDVelocityIterator`](https://github.com/ros-navigation/navigation2/blob/1e8afb17e2e09df443b1870ce0f4ecdee32207fd/nav2_dwb_controller/dwb_plugins/include/dwb_plugins/one_d_velocity_iterator.hpp)

## 5. Critic과 constraint 구조

### 5.1 점수로 타협하지 않고 제거할 후보

- static occupancy 또는 지도 밖 진입
- oriented footprint 충돌
- 금지구역 진입
- 시간 대응 Actor tube와의 표면거리 `0.08 m` 미만
- rollout 종료 후 제한 감속 정지 궤적이 안전하지 않음
- non-finite 또는 유효하지 않은 trajectory
- stateful oscillation 금지 상태 위반

### 5.2 합법 후보의 점수

| Critic | v7에서의 책임 |
|---|---|
| `PathDist` | 후보 종점에서 reference path까지의 grid 거리 |
| `GoalDist` | 후보 종점에서 local goal까지의 grid 거리 |
| `PathAlign` | 앞쪽 평가점이 reference path와 정렬되는 정도 |
| `GoalAlign` | 앞쪽 평가점이 local goal 방향과 정렬되는 정도 |
| `ObstacleFootprint` | 모든 pose의 회전된 footprint 정적 충돌 검사 |
| `Oscillation` | 충분한 이동 전 반복적인 방향 반전 후보 제거 |
| `PreferForward` | 후진 허용 후속 실험에서 전진 선호 |
| `RotateToGoal` | 목표 근처 감속 후 선이동 금지와 목표 yaw 정렬 |

`PathDist`와 `GoalDist`는 평균 직선거리가 아니라 장애물을 반영한 local grid 거리장으로
구현한다. 점수가 같으면 후보 생성 순서를 유지하며 임의의 후처리 정렬로 의미를 바꾸지 않는다.

기준 소스:

- [DWB core 후보 평가](https://github.com/ros-navigation/navigation2/blob/1e8afb17e2e09df443b1870ce0f4ecdee32207fd/nav2_dwb_controller/dwb_core/src/dwb_local_planner.cpp)
- [DWB critics](https://github.com/ros-navigation/navigation2/tree/1e8afb17e2e09df443b1870ce0f4ecdee32207fd/nav2_dwb_controller/dwb_critics)
- [ROS 1 DWA critic 구성](https://github.com/ros-planning/navigation/blob/f44bb1fc2810399165115cc98b530fe4b9397c18/dwa_local_planner/src/dwa_planner.cpp)

## 6. 목표 접근과 완료 처리

기존 사용자 정의 DWA는 목표 근처에서도 같은 점수식으로 전진·회전을 골라 장애물 없는 목표
앞에서 제자리 회전을 선택했다. 공개 구현을 따라 목표 처리를 분리한다.

```text
목표 위치 허용범위 밖 → 일반 DWB 후보 평가
목표 위치 허용범위 진입 → 제한 감속으로 병진 정지
병진 정지 확인 → 제자리 회전
위치·방향 허용오차와 실제 정지 만족 → 완료
```

목표 접근은 점수 가중치를 임시로 덮어쓰는 방식으로 구현하지 않는다. 같은 tick의 반복 호출로
상태가 두 번 전이되지 않아야 하며 새 mission 또는 새 path에서 reset한다.

- [ROS 1 `LatchedStopRotateController`](https://github.com/ros-planning/navigation/blob/f44bb1fc2810399165115cc98b530fe4b9397c18/base_local_planner/src/latched_stop_rotate_controller.cpp)
- [Nav2 `RotateToGoalCritic`](https://github.com/ros-navigation/navigation2/blob/1e8afb17e2e09df443b1870ce0f4ecdee32207fd/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp)

## 7. 공개 구현에서 가져오지 않는 계약

다음은 공개 구현에 없거나 충분하지 않으므로 프로젝트 계약을 유지한다.

- mission/map/observation revision과 content hash
- 검증된 `DynamicObservationFrame`만 사용하는 경계
- observation age·noise·속도 불확실성을 반영한 Actor reachable tube
- trajectory 시간과 Actor tube 시간을 맞춘 동적 충돌 검사
- pose 사이 swept footprint와 `0.08 m` 표면거리
- rollout 뒤 terminal stopping sweep
- 외부 `DynamicSafetyGate`의 최종 명령 재검사
- stale·invalid·late result에서 제한 감속·정지
- `stop_epoch`와 새로운 재개 승인
- 200 Hz ground-truth evaluator
- paired corpus, source hash, hidden lifecycle

ROS 1 `stop_time_buffer`는 분석한 버전에서 값을 저장하지만 실제 후보 판정에는 사용되지 않는다.
공개 코드로 전환한다는 이유로 terminal stopping을 삭제하지 않는다.

## 8. 프로젝트 적용 구조

```text
ControllerSnapshot
→ DynamicDwbReferenceController adapter
→ DwbReferenceCore
   ├─ LimitedAccelTrajectoryGenerator
   ├─ static / forbidden / Actor / terminal constraints
   └─ path / goal / alignment / oscillation critics
→ ControllerCommandResult
→ 기존 DynamicSafetyGate
→ 20 Hz chassis simulation
→ 기존 200 Hz ground-truth evaluator
```

권장 파일 구조:

```text
src/hospital_path_lab/local_algorithms/dwb_reference/
  __init__.py
  contracts.py
  trajectory_generator.py
  critics.py
  core.py
  adapter.py

src/hospital_path_lab/dynamic_trajectory_constraints.py
```

핵심 계약:

```python
class TrajectoryGenerator(Protocol):
    def velocity_samples(self, request): ...
    def generate(self, request, target_twist): ...

class TrajectoryConstraint(Protocol):
    def evaluate(self, request, trajectory): ...

class TrajectoryCritic(Protocol):
    def prepare(self, request): ...
    def score(self, trajectory): ...
    def debrief(self, selected_command): ...
    def reset(self): ...

class DwbReferenceCore:
    def compute(self, request): ...
```

## 9. 기존 임시 DWA의 폐기·격리 범위

다음은 DWA core가 아니라 특정 추월 시험에 맞춘 행동 상태기계이므로 새 core에 넣지 않는다.

- `DEPART → PASS → REJOIN`을 DWA 내부에 직접 넣는 상태기계
- evaluator의 추월 성공 조건을 controller 내부에서 흉내 내는 전이
- Actor를 잃어도 만료 없이 마지막 위치를 사용하는 로직
- 한 점만 검사해 우회 방향을 고르는 로직
- 목표 근처에서 후보 순서를 별도로 재정렬하는 임시 처리
- Python backend에만 적용되는 점수·정렬 변경
- 같은 tick의 반복 호출로 상태 count가 증가하는 처리

우회 행동 정책이 필요하다고 확인되면 DWB core 밖의 `LocalDetourPolicy` 또는 local reference
생성 단계로 분리한다. 이 정책은 hidden label이나 evaluator 결과를 입력으로 받지 않는다.

## 10. 구현과 검증 순서

### Phase A — upstream 동작 재현

1. v7 내부 계약과 generator/critic 수명주기 구현
2. dynamic window와 0 포함 iterator 시험
3. differential-drive rollout 시험
4. critic별 독립 oracle 시험
5. 불법 후보 제거와 동점 결정론 시험
6. 목표 감속·정지·회전 완료 시험

이 단계에서는 Actor 우회를 넣지 않는다. 정적 환경에서 공개 구현 구조를 먼저 재현한다.

2026-08-12 구현 상태:

- limited-acceleration generator, 0 포함 iterator와 41-pose rollout 구현
- critic `prepare → score → debrief → reset` 수명주기 구현
- `PathDist`, `GoalDist`, `PathAlign`, `GoalAlign`, `Oscillation`, `RotateToGoal` 기준 구현
- 불법 후보 즉시 제거, 가중합, strict `<` 동률 결정과 실패 진단 구현
- generator·core·critic·goal controller 전용 시험 `54 passed`

이 통과는 정적 Python reference의 구조 시험이며 동적 사람 우회 성공이나 ROS 2 plugin 동등성을
의미하지 않는다.

### Phase B — 프로젝트 안전 계약 연결

1. `ControllerSnapshot` adapter와 provenance 검증
2. static footprint와 금지구역 constraint
3. Actor tube 시간 대응 constraint
4. terminal stopping constraint
5. 기존 shared safety gate 연결
6. stale·invalid·late result와 `stop_epoch` 회귀시험

2026-08-12 구현 상태:

- `ControllerSnapshot` 검증과 source-derived DWB adapter 구현
- 기존 `evaluate_dynamic_trajectory_safety()`를 재사용하는 hard-constraint critic 구현
- 정적 장애물·금지구역·Actor tube·terminal stopping 후보 제거 연결
- 목표 감속·정지·제자리회전을 core 바깥 goal controller로 분리
- generator·core·critics·goal·constraint·adapter·composition 시험 `129 passed`
- 관측 frame과 Actor prediction geometry 전체 내용 결박, 동일 tick 전체 입력 digest와
  알려진 계약 위반 fail-closed 처리 구현
- `PathAlign`·`GoalAlign`의 finite failure cost와 MapGrid resolution scale을 고정 Nav2 의미에
  맞춤
- 전체 경로 알고리즘 실험실 회귀시험 `467 passed`

아직 기존 20 Hz 동적 pipeline의 goal 완료 판정과 evaluator controller-role 매핑은 v7에 연결하지
않았다. 따라서 이 상태는 단위 구성요소 연결 완료이며, 공개 종단 qualification 완료가 아니다.

### Phase B 최초 공개 진단

v6 공개 `same-direction-wide-r00` Normal 사례의 첫 usable observation에서 source-derived controller를
한 tick 실행했다. 이는 전체 episode 성능시험이나 timing qualification이 아니라 후보 탈락 원인을
확인하기 위한 공개 진단이다.

| 항목 | 결과 |
|---|---:|
| 생성 후보 | `217` |
| 합법 후보 | `0` |
| `actor_clearance_below_minimum` | `217` |
| controller 결과 | `NO_PATH`·정지 요청 |

검사기 생성은 snapshot당 한 번으로 줄였지만 같은 입력의 실제 1-tick 실행은 약 `1.40 s`였다.
주된 비용은 후보마다 5 ms 간격으로 rollout과 terminal stopping을 다시 검사해 총 약 `90,696`회의
pose safety 판정을 수행하는 부분이다. 따라서 현재 Python reference는 50 ms timing 자격을
통과하지 않았으며, 이 진단은 기능 계약을 약화할 근거가 아니다.

정지한 차체 기준으로도 현재 Actor reachable tube 반경은 rollout `2.0 s`에서 약 `1.365 m`, terminal
stopping을 포함한 `2.4 s`에서 약 `1.658 m`까지 증가했다. 그 결과 Actor와 멀어지거나 정지하는
후보까지 모두 불법으로 판정됐다.

이 결과는 critic 가중치나 DWB 후보 생성기의 실패가 아니다. hard constraint가 점수 계산 전에 모든
후보를 제거했기 때문이다. 현재 다음 두 계약이 동시에 참일 수 없다는 뜻이다.

1. corpus는 이 사례를 `LOCAL_DETOUR_FEASIBLE`로 분류한다.
2. online safety는 Actor가 2초 동안 임의 방향으로 크게 가속·감속·반전할 수 있다고 팽창한다.

따라서 공개 full episode나 새 hidden을 실행하기 전에 다음 중 하나를 별도 실험 버전으로 결정해야
한다.

- 보수적 Actor tube를 유지하고 해당 사례를 국소 우회 불가·정지 적합으로 재분류한다.
- 예측 구간의 Actor 운동 가정을 제한하고 corpus·oracle·manifest를 새로 동결한다.

hidden 결과를 본 뒤 tube 수치만 줄이거나, 이 충돌을 critic 튜닝으로 우회하지 않는다.

### Phase C — 공개 기능시험

1. Actor 없는 직선·곡선·세로 경로
2. 정적 장애물과 좁은 복도
3. 정지 사람과 횡단 사람
4. 같은 방향 사람의 실제 추월·재합류
5. 정면·대각선·코너·교차로·다중 Actor
6. 실패 후보 taxonomy와 critic별 점수 보존

공개 development 시험을 통과하기 전에는 새 hidden을 만들거나 열지 않는다.

### Phase D — 성능과 ROS 2 연결

1. Python reference로 기능 의미 동결
2. 단독 직렬 timing qualification
3. 필요 시 C++ core 또는 실제 Nav2 DWB plugin adapter 구현
4. Python과 native/ROS 2의 command·trajectory·critic 결과 동등성 확인

## 11. 합격 기준

- 공개 source-derived 단위시험이 결정론적으로 통과
- 정적 환경에서 후보·critic·goal 처리 oracle 일치
- Actor 우회 시험에서 충돌·금지구역·clearance 위반 0
- 실제 departure, Actor 존재 중 ordered overtake, sustained rejoin, goal 완료
- stale·invalid·late command와 무단 재개 0
- 후보 탈락 원인과 critic별 비용을 결과에서 역추적 가능
- Python reference 통과 전 native/C++ 의미 변경 금지

시간 기준은 기능 자격과 분리한다. 기능 통과 뒤 CPU contention이 없는 직렬 lane에서 50 ms를
측정한다.

## 12. 라이선스

우선 공개 소스의 동작과 구조를 분석한 독립 Python 재구현으로 진행하고 기준 저장소·커밋·파일을
명시한다. 코드를 직접 복사하거나 변형해 포함하면 해당 BSD 저작권 고지, 재배포 조건과 면책
문구를 보존하고 `THIRD_PARTY_NOTICES`에 기록한다. Navigation2 전체를 단일 Apache 라이선스로
간주하지 않고 실제 사용 package와 파일의 라이선스를 확인한다.

## 13. 결론

현재 실패가 DWA 알고리즘 자체의 실패라는 근거는 없다. 기존 Python 코드는 DWA core에 추월
상태기계·목표 접근·안전검사·평가 기준 대응을 섞었고, 우회·재합류 뒤에도 목표 근처에서 정체했다.

> ROS 1 DWA의 실제 동작과 Nav2 DWB의 모듈 구조를 고정 커밋으로 분석한 뒤, 공개 구현의
> generator·critic·goal 처리 구조를 Python으로 재현하고, 프로젝트 고유의 Actor tube·terminal
> stopping·shared safety gate·권한 계약만 명시적으로 확장한다.
