# 집 PC DWB 지역 우회 연구 및 Pro 검토 인수인계

- 작성일: 2026-08-12
- 작업 위치: 집 PC 로컬
- 기준 브랜치: `main`
- 작업 시작 HEAD: `9256b88532635a271440265366f11c2e04162826`
- 작업 시작 tree: `3399e3bf7c7b36c883f1639f7aa1476f9824cf0b`
- 상태: 탐색 구현과 부분 검증 완료, 전체 기능 성공 미증명
- 제품 알고리즘·G1~G5·경로 분석 7단계: 미결정

## 1. 이번 집 PC 작업의 목적

회사 PC에서 구현한 source-derived DWB와 방향성 Actor Capsule을 이어받아 공개
`same-direction-wide-r00`에서 다음 순서가 실제로 가능한지 확인하려 했다.

```text
정지·재판단
→ 실제 측면 이탈
→ 같은 방향 Actor 통과
→ 원 경로 재합류
→ 목적지 도착
```

50ms 시간 자격이나 제품 채택보다 Python 기능 원인을 먼저 분리하는 작업이었다. hidden,
full runner, ROS 2, Unity와 실물 연동은 수행하지 않았다.

## 2. 관측 원인 분리

동결된 Normal·Stress 프로필을 바꾸지 않고 기능 원인 분리용 프로필을 별도로 추가했다.

### FUNCTIONAL_NO_DROPOUT

- Normal의 10Hz, 100ms 지연, TTL, 위치·속도 Gaussian 잡음을 유지한다.
- 독립 frame dropout만 0으로 둔다.
- Normal 안전 증거가 아니라 dropout 영향 분리용이다.

### FUNCTIONAL_IDEAL

- Normal의 10Hz, 100ms 지연과 TTL을 유지한다.
- 위치·속도 잡음과 dropout을 0으로 둔다.
- controller 구조 자체를 보는 mechanism-only 입력이다.

No-dropout 공개 장면을 220 tick 실행한 결과:

- wall time 약 639초
- 실제 이동 42 tick
- 최대 원 경로 이탈 약 `0.0048619m`
- 추월, 재합류, 완료 없음

dropout만 제거해도 Normal 잡음으로 방향 confidence가 다시 쌓이는 시간이 반복돼 충분한
우회 동작으로 이어지지 않았다.

## 3. 기존 직선 reference에서 확인한 DWB 문제

Ideal 입력에서는 217개 중 여러 안전 후보가 남았다. 일부 시점에는 약 45개의 legal 후보와
더 큰 회전 후보가 존재했지만, Path·Goal critic은 다음을 선호했다.

- 직선 reference 유지
- 선행 Actor 뒤에서 감속
- 정지

Actor에서 멀어지는 단순 보상도 시험했지만 옆으로 추월하기보다 뒤에서 느려지는 행동을
강화했다. 해당 비용 변경은 채택하지 않았다.

현재 제한적인 해석은 다음과 같다.

> DWB는 안전한 짧은 속도·궤적 후보를 고를 수 있지만, 직선 reference만 받은 상태에서
> 좌·우 통과 topology를 반드시 새로 생성하는 지역 경로 생성기로 볼 수 없다.

## 4. 외부 지역 reference 탐색

`DirectionalLocalDetourPolicy`를 DWB core 밖에 추가했다.

- corpus label, evaluator oracle, hidden, Actor ground truth를 입력받지 않는다.
- 방향성 Actor prediction, 지도, 현재 로봇 상태와 기존 reference만 사용한다.
- 좌우 후보를 기존 충돌·금지구역 geometry로 확인한다.
- 임시 연구 reference는 다음 5개 pose로 구성한다.

```text
현재 pose
→ 측면 이탈 pose
→ Actor 앞 통과 pose
→ 원 경로 재합류 pose
→ 기존 목적지
```

이 경로는 일반 지역 planner가 아니라 `same-direction-wide` 원인 분리용 직사각형 후보다.
고정 `0.70m` offset도 제품 수치가 아니다.

### 전체 경로를 한 번에 준 실패

- DWB가 회전은 했지만 전진하지 않았다.
- GoalAlign만 0으로 만들어도 distant final goal 영향이 남았다.
- GoalDist까지 없애면 정지 후보와 tie되는 다른 문제가 발생했다.
- 대각선 진입만 바꿔도 해결되지 않았다.

### 다음 waypoint만 준 부분 성공

외부 정책은 전체 지역 경로를 보존하고, DWB에는 현재 구간의 다음 waypoint만 제공했다.
waypoint가 바뀌면 goal-latch를 억지로 우회하지 않기 위해 새 DWB instance를 만들었다.

이 방식은 실제 이탈을 만들었지만 다음 문제가 남는다.

- singleton waypoint는 다음 구간 접선과 전체 재합류 방향을 잃는다.
- DWB instance 재생성은 Oscillation·goal latch 상태를 초기화한다.
- 따라서 persistent DWB와 동일한 조건의 비교가 아니다.
- 향후 비동기 실행 전에는 path·maneuver·controller revision 계약이 별도로 필요하다.

## 5. 180-tick 연속 Ideal 실행 결과

공개 `same-direction-wide-r00`을 episode 시작부터 연속 실행했다.

| 항목 | 결과 |
|---|---|
| profile | `FUNCTIONAL_IDEAL` |
| wall time | 약 `891.37초` |
| 최초 실제 전진 적용 | tick `77` |
| 최대 적용 선속도 | 약 `0.153125m/s` |
| 최종 pose | `x≈0.631029, y≈2.098274, yaw≈-1.569707` |
| 기존 reference | `y=2.32` |
| 최대 reference 이탈 | `0.221726m` |
| hard-safety | 통과 |
| no-safe-candidate | `0` |
| gate override | `20` |
| 충돌·금지구역 진입 | `0` |
| 종료 waypoint | `1` |
| Actor 추월 | 미증명 |
| 원 경로 재합류 | 미증명 |
| 목적지 도착 | 미증명 |

이 결과가 증명한 것은 다음 하나다.

> 외부 지역 reference와 단기 목표를 주면 현재 DWB가 기존 직선 reference에서 `0.10m`
> 이상 실제로 이탈하는 명령을 만들 수 있다.

전체 우회 기능 성공, Normal 성공, 제품 적합성 또는 DWB 채택을 증명하지 않는다.

## 6. checkpoint/resume 탐색

순수 Python DWB가 tick당 약 6~7초 걸려 장시간 기능 탐색을 위해 다음을 추가했다.

- simulator의 `start_tick_id`
- 기존 `DynamicSafetyGate` 전달
- 기능 실행 스크립트의 checkpoint JSON 입력
- 지역 reference와 현재 waypoint 복원

한 tick resume smoke에서 pose, twist와 waypoint는 이어졌다. 그러나 현재 저장 상태에는 RNG,
observation queue, tracker posterior, DWB critic 상태, gate 전체 상태와 evaluator 누적 상태가 모두
포함되지 않는다.

따라서 resume 결과는 다음으로 제한했다.

- 기능 탐색 segment: 사용 가능
- episode 시작부터의 연속 hard-safety 증거: 사용 금지

180→300 tick resume 실행은 시작했지만 사용자 중단 지시에 따라 종료했다. 현재 실행 중인
Python 프로세스는 없다.

## 7. 테스트와 검증 상태

회사 정본 `9256b885...`에서는 전체 회귀 `549 passed`와 Ruff가 통과했다.

집 PC 변경 후 확인한 범위:

- 관측·방향·DWB 집중 범위: `118 passed`
- 지역 우회 정책: `5 passed`
- 시뮬레이션·안전 영향권: `32 passed`
- 관련 Ruff: 통과
- 인수인계 직전 변경 직접 영향권 재검증: `28 passed in 20.40s`
- 인수인계 직전 Ruff, compileall, `git diff --check`: 통과

중요한 한계:

- 최신 집 PC 변경 뒤 전체 549개 회귀는 다시 실행하지 않았다.
- 180-tick 연속 실행은 Ideal mechanism evidence다.
- Normal 종단 실행, 추월, 재합류와 목적지 도착은 미완료다.
- output JSON은 `simulation/path_planning_lab/outputs/` 아래 로컬 ignored 산출물이며 Git으로
  전달되지 않는다. 수치는 이 문서에 보존했다.

## 8. Pro 연구 요청과 검토 결과

다음 Pro 입력 프롬프트를 작성했다.

```text
docs/reviews/pro-path-algorithm-research-prompt-2026-08-12.md
```

사용자는 Claude Opus 5 Max의 연구 결과를 Pro에 다시 입력해 보강된 답변을 받았다. 해당
원문은 현재 Codex 첨부 파일로만 제공됐으며 저장소에는 포함되지 않았다. 내일 사용자가 다음
자료를 회사 PC 세션에 제공할 수 있다.

1. 원래 Pro 질문 프롬프트
2. Claude 원문 조사 결과
3. Claude 자료를 받은 Pro의 실제 프롬프트
4. Pro 최종 답변

이 네 자료를 함께 받아야 결론의 출처와 변형 과정을 정확히 추적할 수 있다.

### Pro 답변에서 유지할 내용

- 상위 계층 구조는 전면 폐기보다 수정 후 유지한다.
- 지역 기동 후보 생성과 follower·velocity selector를 분리한다.
- `WAIT/FOLLOW`, `PASS_LEFT`, `PASS_RIGHT`를 명시적인 후보로 비교한다.
- persistent controller와 방향 정보를 가진 sliding subpath를 검토한다.
- 2초 rollout은 전체 추월 경로가 아니라 짧은 receding execution에 사용한다.
- State Lattice와 SIPP는 제품 확정이 아니라 offline oracle 후보로만 검토한다.
- Ideal·no-dropout 결과를 Normal 안전 증거로 승격하지 않는다.
- 공개 기능·안전·시간 자격 전에는 hidden을 생성하지 않는다.

### 그대로 적용하면 안 되는 부분

#### 1. `2σ envelope 위반 0회`를 일반 안전 보장으로 사용하면 안 됨

현재 잡음은 Gaussian이고 방향성 Capsule은 명시적인 2σ 연구 휴리스틱이다. 다음을 분리해야
한다.

- 동결한 Actor 운동학이 결정론적 reachable bound 안에 있는가: 위반 0회
- noisy observation 기반 2σ envelope의 coverage가 calibration되는가: 통계 판정
- 실제 사람 안전 보장: 주장 금지

현재 `same-direction-wide` Actor는 일정한 +x 속도의 open-loop Actor다. 방향전환 Actor의
일반 containment는 공개 corpus 확대 전 차단조건이지만 현재 Ideal 한 장면의 mechanism
실험을 무효화하는 근거는 아니다.

#### 2. command revision 문제는 현재 동기식 simulator에서 즉시 P0가 아님

현재 controller와 gate는 같은 tick에서 순차 실행되고 `source_tick_id` 불일치를 거부한다.
따라서 현재 문제는 늦은 이전 명령 실행보다 instance 초기화로 비교가 달라지는 P1이다.
path·maneuver·controller revision은 비동기·ROS2·분산 실행 전에 P0 계약이 된다.

#### 3. 기존 시간 포함 witness를 누락된 것으로 취급하지 않음

현재 `dynamic_corpus.py`의 feasible witness validator는 이미 다음을 검사한다.

- 20Hz 차체 운동학과 가감속
- exact footprint와 금지구역
- Normal·Stress time-indexed Actor tube
- ordered overtake
- terminal dwell

부족한 것은 시간 검사가 전혀 없는 것이 아니라, 수작업 witness를 자동 탐색하고 다양한
장면에 일반화하는 oracle이다.

#### 4. `DWPP`는 정의되지 않음

Pro 답변의 `RPP/DWPP`에서 DWPP가 무엇인지 확인되기 전에는 새 알고리즘 후보로 쓰지 않는다.

## 9. 현재 결론

현재 결과로 DWA나 DWB를 폐기하거나 채택할 수 없다.

```text
직선 reference의 topology 부족
+ DWB path·goal critic의 직진·감속 선호
+ Normal 방향 confidence의 반복 초기화
+ 단일 waypoint와 controller 재생성의 비교 왜곡
```

위 원인이 섞여 있다.

현재 가장 비용이 낮은 다음 연구 순서는 다음과 같다. 이 순서는 제품 채택이나 구현 승인이
아니다.

1. 현재 고정 방향 Actor의 prediction containment와 2σ calibration을 분리 감사
2. 기존 시간 포함 witness를 persistent RPP와 persistent DWB에 동일하게 제공
3. `전체 local path / sliding subpath / singleton waypoint`만 바꾸는 ablation
4. 기존 witness가 여러 장면에 일반화되지 않을 때 bounded State Lattice 검토
5. 기다림·출발 시각 자동 탐색이 필요할 때 Kinodynamic SIPP 검토

## 10. 변경 파일

### 수정

```text
simulation/path_planning_lab/src/hospital_path_lab/dynamic_observation.py
simulation/path_planning_lab/src/hospital_path_lab/simulation.py
simulation/path_planning_lab/tests/test_directional_dwb_focused_public.py
simulation/path_planning_lab/tests/test_dynamic_observation.py
```

### 추가

```text
simulation/path_planning_lab/src/hospital_path_lab/local_detour_policy.py
simulation/path_planning_lab/scripts/run_directional_dwb_functional.py
simulation/path_planning_lab/tests/test_local_detour_policy.py
docs/reviews/pro-path-algorithm-research-prompt-2026-08-12.md
docs/reviews/home-local-dwb-research-handoff-2026-08-12.md
```

## 11. 회사 PC에서 먼저 할 일

1. Git 동기화와 작업트리 보존을 완료한다.
2. `AGENTS.md`, `인수인계.md`, 이 보고서를 완전히 읽는다.
3. 전체 회귀를 아직 통과했다고 가정하지 않는다.
4. 사용자에게 Pro·Claude 원문 자료를 받을 수 있는지 확인한다.
5. 원문을 받으면 이 보고서 8절의 네 보정이 실제 Pro 입력·답변에도 맞는지 재검토한다.
6. 사용자 승인 전 State Lattice, SIPP, persistent DWB, sliding subpath를 구현하지 않는다.
7. hidden, 제품 알고리즘 채택, G1~G5와 경로 분석 7단계는 시작하지 않는다.
