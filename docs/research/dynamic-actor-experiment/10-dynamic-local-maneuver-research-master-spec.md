# 동적 지역 기동 연구 R1~R7 Master Specification

## 1. 문서 상태와 목적

- 작성일: `2026-08-13`
- 상태: 사용자 개인 연구 방향을 실행 가능한 단계 계약으로 구체화한 기준선
- 증거 범위: Python `simulation_only`, 합성 지도, 가상 차체, open-loop 원형 Actor
- 팀 전체 합의: 아님
- 제품 알고리즘 채택: 아님
- `G1~G5` 결정과 제품 경로분석 7단계: 미수행
- 현재 단계: `R1 완료`, `R2 계약·projection·독립 validator 구현`, 자동 탐색 미구현

이 문서는
[`동적 지역 기동 연구 방향 판정과 자료 출처`](../../reviews/dynamic-local-maneuver-research-direction-2026-08-13.md)의
공개 연구 순서 1~7을 하나의 실행 기준으로 묶는다. 각 단계에서 무엇을 입력으로 받고,
무엇을 증명하며, 어떤 결과를 남긴 뒤 다음 단계로 이동할지를 정의한다.

이 문서의 `R1~R7`은 다음 둘과 다른 번호 체계다.

- 제품 기획의 경로 알고리즘 분석·설계 `1~8단계`
- 기존 동적 Actor 비교실험의 구현 `1~6단계`

혼동을 막기 위해 이 문서에서는 모든 후속 연구 단계에 `R` 접두사를 붙인다.

## 2. 문서 우선순위와 변경 규칙

충돌 시 우선순위는 다음과 같다.

1. `AGENTS.md`와 제품·안전 기준 문서
2. [경로 기능 6단계 조건부 추천안](../../product/path-planning-conditional-recommendation.md)과
   [경로 안전·권한 흐름](../../safety/path-safety-authority-flow.md)
3. 승인된 동적 Actor 실험 계약과 ADR
4. 이 master specification
5. `R1`, `R2`처럼 개별 단계의 상세 명세
6. 구현 코드, 시험과 실행 산출물

하위 문서나 코드가 상위 기준과 충돌하면 임의로 상위 기준을 고치지 않는다. 충돌 위치,
영향받는 단계와 현재 증거를 먼저 보고하고, 사용자 승인 뒤 명세와 구현을 함께 갱신한다.

각 단계 구현 전 상세 명세에는 최소한 다음이 있어야 한다.

- 상태와 범위
- 입력, provenance와 허용 split
- 추상 인터페이스와 출력
- hard failure, 정상 음성 결과와 limitation
- 시험 목록과 독립 oracle
- 완료조건과 다음 분기
- 생성 산출물과 비덮어쓰기 규칙
- 제품·실물·사람 탑승으로 확대할 수 없는 증거 한계

단계별 명세는 미리 작성할 수 있지만, 앞 단계의 gate를 통과하지 않은 뒤 단계의 실행
결과를 최종 연구 증거로 승격하지 않는다.

## 3. 전체 연구 질문

현재 연구는 다음 실패 원인을 서로 분리한다.

```text
관측·예측 계약이 장면을 표현하지 못함
vs
공간적으로 통과할 수 없음
vs
시간적으로 안전한 통과 순서가 없음
vs
지역 reference가 기동 구조를 제공하지 못함
vs
controller가 주어진 reference를 실행하지 못함
vs
shared safety gate 또는 권한 계약이 실행을 거부함
vs
계산시간이 실행 주기를 만족하지 못함
```

상위 구조는 다음으로 고정한다.

```text
전역 통로·경로 선택
→ 지역 기동 후보 판단
→ bounded 지역 경로·reference 생성
→ persistent controller
→ shared online safety gate
→ 제한 감속·실제 정지·hold

offline:
공간 oracle + 시간 witness/oracle + ground-truth evaluator
```

State Lattice, SIPP, RPP와 DWB는 이 연구에서 제품 채택안이 아니다. 특정 실패가 어느
계층에 속하는지 분리하기 위한 연구 도구 또는 비교 대상이다.

## 4. 전 단계 공통 불변조건

### 4.1 안전과 권한

- 충돌 가능성 또는 필수 판단 불충분에서는 경로 변경보다 로봇 로컬 안전정지를 먼저
  개시한다.
- 정지 요청, 감속 중 상태와 실제 정지 완료를 구분한다.
- 경로·기동·reference 존재는 이동 허가가 아니다.
- 실제 정지 완료 전 원 경로 재개나 대체 기동을 실행하지 않는다.
- 보호정지 이전의 이동 허가와 재개 지시를 다음 `stop_epoch`에서 재사용하지 않는다.
- 위험 해소, 입력 복구, 통신 복구와 비상정지 해제만으로 자동 재출발하지 않는다.
- 현재 미션·지도·경로·관측·기동 revision과 일치하지 않는 결과를 실행하지 않는다.
- 충돌·금지구역·stale·invalid·late·무단 재개를 통과시키기 위해 안전 수치를 완화하지
  않는다.
- shared gate가 위험 명령을 막은 사실을 controller 자체의 안전성으로 주장하지 않는다.

현재 실험의 Actor 반경 `0.18m`, 실제 표면 clearance `0.08m`, oriented footprint,
terminal stopping과 5ms swept 검사는 상위 동결 계약을 따른다. 이 master 문서는 수치를
재정의하지 않는다.

### 4.2 정보 분리

online controller와 gate에는 다음을 전달하지 않는다.

- ground-truth Actor 위치
- expectation category와 정답 label
- oracle 결과와 feasible witness
- split·family·variant를 드러내는 식별자
- hidden seed 또는 hidden 결과

ground truth는 offline oracle과 evaluator만 사용한다.

### 4.3 공개·hidden 수명주기

- `R1~R6`은 `GOLDEN`과 `DEVELOPMENT` 공개 자료만 사용한다.
- 공개 기능·안전·연산 자격이 닫히기 전에는 새 hidden을 생성·열람·실행하지 않는다.
- hidden 결과를 본 뒤 코드·parameter·corpus를 바꾸면 해당 hidden은 regression으로
  전환하고 최종 증거로 다시 사용하지 않는다.
- 새 hidden은 `R7`에서 별도 사용자 승인과 새 seed commitment 뒤 한 번만 실행한다.
- 기존에 소비한 v5 hidden은 새 연구의 최종 증거로 재사용하지 않는다.

### 4.4 결정론과 실행 자원

- 한 episode 안의 tick은 상태가 이어지므로 직렬로 실행한다.
- 서로 독립적인 public episode는 process 기반으로 병렬 실행할 수 있다.
- paired controller 비교는 같은 worker에서 같은 seed·관측 stream을 사용한다.
- 병렬 결과는 corpus 입력 순서로 다시 정렬해 worker 완료 순서의 영향을 없앤다.
- wall-clock timing qualification은 worker pool 종료 뒤 CPU 간섭 없이 직렬 실행한다.
- wall-clock 시간 자체는 결정론 대상이 아니며, 명령·상태·event·metric은 동결 tolerance
  안에서 결정론을 만족해야 한다.
- 장기 실행의 checkpoint는 진단용이며, 여러 checkpoint를 이어 붙여 연속 hard-safety
  증거로 사용하지 않는다.

### 4.5 공통 provenance

비동기·ROS 2·별도 process로 이동하기 전까지 다음 문맥을 결과와 명령에 결박한다.

```text
mission_id
stop_epoch
map_revision
observation_revision
maneuver_revision
path_revision
subgoal_revision
controller_session_id
control_tick
input_content_hash
```

현재 동기식 simulator에서 누락된 revision은 연속 controller 비교의 P1이다. 비동기 실행
전에는 stale 명령을 실제로 적용할 수 있으므로 P0 실행 차단 계약이다.

## 5. 공통 결과 분류

공간·시간·관측과 controller 결과를 하나의 `성공/실패`로 합치지 않는다.

| 분류 | 의미 | 기본 처리 |
|---|---|---|
| `FEASIBLE` | 동결한 범위에서 검증 가능한 안전 기동 witness가 있음 | 다음 계층 입력 후보 |
| `WAIT_ONLY` | 국소 통과보다 정지·대기·원 경로 재개가 적절함 | 안전정지·대기 |
| `FORBIDDEN` | 허용구역·통행 규칙 또는 안전 계약상 기동 금지 | 기동 생성 금지 |
| `SPATIALLY_INFEASIBLE` | 차체와 정적 공간만으로 통과·재합류 불가 | 정지·상위 재경로 검토 |
| `TEMPORALLY_INFEASIBLE` | 공간 path는 있으나 Actor와 시간 순서를 만족하는 witness가 없음 | 정지·대기 |
| `OBSERVATION_UNDECIDABLE` | ground truth에서는 가능성을 논할 수 있으나 online 관측으로 판단 불가 | fail-closed 정지 |
| `NO_SAFE_SOLUTION` | 현재 범위에서 기다림을 포함해 안전한 이동 해가 없음 | 정지·지원 |
| `INVALID_INPUT` | source·revision·hash·finite·schema 계약 위반 | 실행 거부 |
| `SEARCH_INCONCLUSIVE` | 구조화된 template에서 witness를 못 찾았거나 resource limit으로 완전 판정 불가 | R3 또는 search 진단으로 전달 |

이 분류는 연구 판정을 위한 상위 taxonomy다. 구체 enum과 직렬화 형식은 `R2` 상세 명세에서
정의하고 이후 단계에서 의미를 바꾸지 않는다.

## 6. 단계별 계약

## R1 — Prediction 계약 감사

### 목적

Actor generator의 결정론적 운동 범위와 predictor envelope의 포함 관계를 검사하고,
Gaussian 관측 coverage를 hard safety와 분리한다.

### 입력

- v6 공개 13개 episode
- Ideal·Normal·Stress 합성 관측
- 동결 방향성 prediction parameter
- evaluator 전용 ground truth

### 출력

- 운동 sample·transition·위반 목록
- profile별 dropout과 component/radial `2σ` coverage
- rollout 시각별 exact Capsule 경험적 coverage
- hard failure와 limitation
- 자체 content hash를 가진 JSON과 사람이 읽는 요약

### 완료조건

- 공개 Actor motion contract hard failure `0`
- Ideal 관측 오차·dropout `0`
- Ideal Capsule miss `0`
- Normal·Stress의 Gaussian·Capsule miss를 통계 결과로 보존
- 공개 corpus에 없는 가속·감속·정지·회전을 미검증 limitation으로 명시
- 같은 입력의 semantic 결과 결정론과 hidden 입력 거부

### 현재 상태

`완료`. 상세 명세는
[`09-prediction-contract-audit.md`](09-prediction-contract-audit.md)를 따른다. 최신 공개 감사의
motion transition은 `5,420`, 위반은 `0`, Ideal Capsule은 `26,257/26,257`, Normal
Capsule은 `19,170/20,118`이다. Normal miss `948`개는 안전 수치 완화 근거가 아니다.

## R2 — 기존 Witness 자동화·일반화

상세 구현 계약은
[`11-witness-automation-and-generalization.md`](11-witness-automation-and-generalization.md)를
따른다.

### 목적

현재 수동 feasible witness를 자동 실행 가능한 공개 time-indexed oracle로 만들고, 여러
장면에서 공간·시간·관측 실패를 분리한다.

### 입력

- `R1`을 통과한 prediction contract와 공개 corpus
- 기존 수동 feasible witness와 ground-truth evaluator
- 20Hz 가상 차체 운동학, 선·각속도와 가감속
- exact oriented footprint, 허용·금지구역과 time-indexed Actor shape
- terminal stopping과 ordered overtake·rejoin 계약

### 필수 동작

- 기존 witness를 동일 안전조건으로 자동 재생·검증한다.
- `positive`, `wait-only`, `forbidden`, `no-solution` 공개 장면을 포함한다.
- 출발·이탈·통과·투영 순서 역전·재합류·terminal dwell의 시각 순서를 보존한다.
- 탐색기는 controller ID, critic 점수, category 정답과 hidden 정보를 읽지 않는다.
- 같은 seed에서 같은 witness 또는 같은 음성 판정을 반환한다.
- witness가 없을 때 탐색 제한, 공간 불가, 시간 불가와 관측 불충분을 구분한다.

### 출력

- episode별 상위 결과 분류와 실패 taxonomy
- time-indexed pose·twist witness 또는 음성 판정 근거
- 최소 ground-truth clearance, 금지구역·운동학·terminal 검증 결과
- 탐색 seed·parameter·corpus·source hash
- JSON 요약과 경로·Actor·시간을 보여주는 PNG

### 완료조건

- 기존 positive witness를 자동 경로로 재현한다.
- 알려진 wait-only·forbidden·no-solution 사례를 positive로 오분류하지 않는다.
- exact evaluator와 독립 validator가 witness를 재검증한다.
- 실패가 search budget 초과인지 검증된 음성 결과인지 구분된다.
- 공개 corpus 전체의 분류와 limitation이 비덮어쓰기 산출물로 보존된다.

### 중단·분기

- 기존 positive witness를 동일 계약에서 재현하지 못하면 `R3~R6` controller 비교를
  시작하지 않고 witness·corpus·prediction 계약 충돌을 보고한다.
- `SPATIALLY_INFEASIBLE`과 `TEMPORALLY_INFEASIBLE`을 자동 witness만으로 구분할 수 없으면
  `R3` 공간 oracle이 해당 판정을 분해한다.
- `R2` 결과에 맞춰 safety, Actor radius, clearance와 prediction envelope를 낮추지 않는다.

## R3 — Bounded 공간 Oracle 연구

### 목적

동적 시간과 controller 성능을 제거한 상태에서, 가상 차체가 허용된 bounded 공간 안에서
출발 pose부터 재합류 pose까지 물리적으로 이어질 수 있는지 독립 판정한다.

### 후보와 범위

- 첫 후보: bounded State Lattice 또는 동등한 pose·heading 탐색 oracle
- 역할: offline 공간 수행 가능성 판정
- 범위: exact footprint, static obstacle, forbidden zone, 시작·종료 pose와 heading
- 제외: 제품 local planner 채택, 실제 motion primitive 확정, online 이동 허가

### 입력

- static grid·허용영역·금지영역과 map provenance
- 가상 footprint와 추상 motion primitive profile
- 시작 pose, 기동 방향과 재합류 pose·heading tolerance
- bounded search region과 명시적 resource limit

### 출력

- collision-free pose·heading path 또는 공간 음성 판정
- expanded state, search bound, resolution과 termination reason
- 최소 clearance, path length, curvature·primitive sequence
- 독립 exact-footprint validation 결과

### 완료조건

- 넓은 통로, 좁은 문, 막다른 구간, 좌·우 mirror와 재합류 방향 시험을 통과한다.
- 반환 path는 exact footprint·forbidden·종료 heading 검사를 통과한다.
- 알려진 공간 불가 사례는 `SPATIALLY_INFEASIBLE`로 닫힌다.
- resource limit 초과를 `SPATIALLY_INFEASIBLE`로 가장하지 않는다.
- 실제 primitive 수치는 `G2` 전까지 `simulation_only` 가상 profile로 표시한다.

### 다음 분기

```text
공간 불가
→ controller 비교 대상 아님

공간 가능 + R2 시간 불가
→ temporal oracle 또는 WAIT_ONLY 검토

공간·시간 가능
→ R4 지역 기동 reference 후보 생성
```

## R4 — 복수 기동 후보와 Reference 계약

### 목적

단일 `0.70m` offset과 singleton waypoint 대신, 방향과 재합류 구조를 가진 복수 지역 기동
후보를 동일 계약으로 표현한다.

### 최소 후보

```text
WAIT_OR_FOLLOW
PASS_LEFT
PASS_RIGHT
```

`GLOBAL_REROUTE_REQUEST`와 `SUPPORT_REQUEST`는 지역 통과 path가 아니라 상위 결과이므로
별도 결과로 유지한다.

### 후보 필수 의미

- 현재 미션·map·observation provenance
- `maneuver_revision`, `path_revision`, `subgoal_revision`
- 기동 종류와 선택 이유
- pose와 tangent를 가진 local path
- 이탈 시작, Actor 통과 구간과 재합류 pose·heading
- 허용구역·금지구역과 유효시간
- path를 자르는 sliding local subpath 규칙
- 취소·교체·stale·완료 상태

후보의 `valid` 또는 `feasible`은 이동 허가가 아니다.

### 완료조건

- left/right mirror와 wait 후보가 하나의 schema에서 결정론적으로 생성된다.
- point 하나가 아니라 연속 pose·tangent·재합류 방향이 controller에 전달된다.
- 같은 maneuver 안에서 sliding subpath 갱신이 controller session을 재생성하지 않는다.
- 다른 revision의 이전 명령·subpath·결과를 실행하지 않는다.
- candidate와 gate/controller/evaluator의 원인 로그를 서로 구분한다.
- category·oracle·ground truth가 online 후보 생성 입력으로 누출되지 않는다.

## R5 — 동일 Witness Controller 비교

### 목적

같은 지역 path를 받았을 때 persistent RPP와 persistent source-derived DWB가 기동을 어떻게
실행하는지 비교해 path 생성 실패와 controller 실행 실패를 분리한다.

### 공정 비교 조건

- 같은 start state, local path, 차체 profile과 free-space target speed
- 같은 observation stream, Actor prediction과 shared gate
- 같은 stop·resume authority와 simulation-time 적용 지연
- 같은 episode 안에서 controller instance와 내부 상태 유지
- waypoint 또는 subpath 갱신마다 controller를 재생성하지 않음
- `Ideal → no-dropout → Normal public → Stress degradation` 순서

### 출력 지표

- 기동 시작, 실제 측면 이탈과 path tracking error
- ordered overtake와 rejoin 여부·지속시간
- completion, traffic wait와 planner deadlock
- controller stop request, no-safe candidate와 gate override 원인
- 최소 ground-truth clearance, path length, jerk와 각운동 지표
- 후보 수·illegal taxonomy·계산시간 진단

### 완료조건

- controller별 명령·상태·event가 repeated public run에서 결정론적이다.
- 기능상 progressable인 Ideal·no-dropout 사례에서 기대 기동을 실행한다.
- hard safety 실패가 있는 비교 결과로 우열을 계산하지 않는다.
- Normal에서 기능 실패 시 prediction, reference, controller와 gate 원인을 분리한다.
- Stress는 안전 열화 시험이며 무조건 임무 완료를 요구하지 않는다.
- RPP 또는 DWB 한 구현의 실패를 알고리즘 계열 전체의 실패로 확대하지 않는다.

## R6 — 연속 공개 종단 자격

### 목적

조각난 checkpoint가 아니라 하나의 연속 public episode에서 지역 기동 전체와 안전·권한
계약을 증명한다.

### 필수 연속 순서

```text
측면 이탈
→ Actor 존재 중 안전한 통과
→ reference 투영 순서 역전
→ 원 경로 재합류
→ 0.5초 이상 재합류 유지
→ 지역 기동 종료 또는 목적지 도착
```

### Hard gate

- 충돌 `0`
- 금지구역 진입 `0`
- 실제 표면 clearance 위반 `0`
- stale·invalid·다른 revision 명령 적용 `0`
- late command 적용 `0`
- 무단 자동 재개 `0`
- provenance 불일치 실행 `0`
- non-finite·예외·설명되지 않는 비결정성 `0`

### Functional gate

- progressable Normal episode의 완료와 planner deadlock `0`
- wait-only·forbidden·no-solution의 보수적 정지
- controller stop과 gate override의 원인별 귀속
- full-state digest와 연속 event trace
- public corpus·source·parameter·code hash 동결

### 실행과 timing

- 독립 episode의 기능·안전 평가는 process 병렬화할 수 있다.
- paired 조건은 같은 worker에 유지한다.
- wall-clock qualification은 별도 직렬 lane에서 측정한다.
- Python 기능이 통과했지만 timing이 실패하면 기능 증거와 성능 미달을 별도로 기록한다.
- 기능 미달을 C++ 이식으로 숨기지 않는다.

### 완료조건

- 모든 요구 public episode가 중간 checkpoint 없이 완료 또는 정당한 보수적 종료된다.
- 최신 전체 회귀, Ruff, source freeze와 결과 hash가 통과한다.
- 부분·축소 실행은 report-only이며 정식 public qualification receipt를 만들지 않는다.
- 기능·안전·timing 중 미완료 항목을 숨기지 않는다.

## R7 — 후속 실행·Native·Hidden 진입 Gate

### 성격

`R7`은 새로운 planner 구현 단계가 아니다. `R1~R6`의 증거를 보고 다음 연구 실행을 허용할지
판정하는 release gate다.

### 순서

1. Python에서 prediction → oracle → reference → persistent controller → gate의 기능 구조를
   먼저 증명한다.
2. 기능은 통과했으나 계산시간만 미달하면 병목을 측정한다.
3. 의미를 바꾸지 않고 필요한 계산 kernel만 C++로 이전한다.
4. Python과 native의 controller·diagnostic·safety semantic parity를 확인한다.
5. CPU 간섭 없는 직렬 wall-clock qualification을 실행한다.
6. 공개 기능·안전·연산 자격과 manifest를 동결한다.
7. 사용자 별도 승인 뒤 새 hidden seed commitment를 생성한다.
8. 새 hidden을 한 번 실행하고 결과를 변경 없이 보존한다.

### Hidden 전 필수 입력

- `R1~R6` 완료 상태와 산출물 hash
- code commit과 source tree hash
- corpus·map·vehicle·prediction·controller·gate parameter hash
- qualification snapshot set hash와 실행 머신 식별
- public receipt와 limitation
- 새 hidden seed commitment

### 판정

```text
공개 기능 미달
→ Python 공개 연구로 복귀, hidden 금지

공개 기능 통과 + timing 미달
→ native 최적화 후보, hidden 금지

공개 기능·안전·timing 통과
→ 사용자 승인 시 새 hidden 자격

hidden hard failure
→ 해당 hidden은 regression 전환, 같은 hidden 재튜닝 금지

hidden까지 연구 조건 충족
→ 동결 simulation 연구 기준선 후보
```

마지막 결과도 제품 알고리즘 채택, 실제 사람 탑승 안전성, `G1~G5` 또는 제품 경로분석
7단계 완료를 의미하지 않는다.

## 7. 단계 Gate 표

| 단계 | 핵심 질문 | 완료 증거 | 현재 상태 | 다음 진입 조건 |
|---|---|---|---|---|
| `R1` | prediction 계약이 generator와 맞는가? | 공개 motion·관측·Capsule audit | 완료 | hard failure 0 |
| `R2` | 안전한 time-indexed witness를 자동화할 수 있는가? | 자동 witness·음성 판정·taxonomy | 부분 구현: label-free 계약·독립 validator | R1 완료 |
| `R3` | 정적 공간에서 차체가 통과·재합류할 수 있는가? | bounded 공간 oracle | 미시작 | R2 분류 가능 |
| `R4` | WAIT/LEFT/RIGHT를 방향 있는 reference로 표현하는가? | revision 결박 local path·subpath | 미시작 | R2·R3 계약 정리 |
| `R5` | 같은 reference에서 controller 차이가 무엇인가? | persistent RPP·DWB paired 결과 | 미시작 | 검증된 witness·reference |
| `R6` | 연속 공개 episode 전체가 닫히는가? | public 종단 report·receipt·회귀 | 미시작 | R5 공개 기능 통과 |
| `R7` | native 또는 새 hidden으로 넘어갈 자격이 있는가? | freeze manifest·직렬 timing·승인 | 미시작 | R1~R6 증거 완결 |

## 8. 산출물과 보존

각 단계는 최소한 다음을 남긴다.

- 단계별 Markdown 상세 명세
- 입력·parameter·source·corpus hash
- machine-readable JSON 결과
- 사람이 읽는 summary
- 실패 taxonomy와 limitation
- relevant pytest·Ruff·전체 회귀 결과
- 필요한 경우 지도·path·trajectory PNG

규칙:

- 기존 output 경로를 덮어쓰지 않는다.
- partial·checkpoint를 final evidence로 승격하지 않는다.
- 생성 로그와 대용량 산출물은 기본적으로 Git에 커밋하지 않는다.
- 명세·코드·작은 golden fixture·회귀시험은 검증 뒤 커밋한다.
- 실패 결과를 삭제하거나 좋은 결과만 선택하지 않는다.

단계별 상세 명세 권장 파일명은 다음과 같다.

```text
09-prediction-contract-audit.md                  # R1, 완료
11-witness-automation-and-generalization.md      # R2
12-bounded-spatial-oracle.md                     # R3
13-local-maneuver-reference-contract.md          # R4
14-persistent-controller-comparison.md           # R5
15-public-end-to-end-qualification.md             # R6
16-native-and-hidden-entry-gate.md                # R7
```

## 9. 전체 중단조건

다음 중 하나가 발생하면 뒤 단계로 진행하지 않는다.

- 상위 제품·안전 문서와 충돌
- public과 hidden 또는 evaluator와 controller 사이 정보 누출
- 안전 수치·Actor model·corpus label을 결과에 맞춰 사후 완화
- 기존 positive witness를 동일 계약에서 재현하지 못함
- stale revision 또는 과거 command가 실행될 수 있음
- ground-truth evaluator가 online prediction을 재사용해 독립성을 잃음
- partial checkpoint를 연속 종단 증거로 사용
- public 기능 실패를 native 성능 최적화로 우회
- 실제 사람·센서·차체 증거로 확대 해석

중단은 연구 실패를 숨기는 절차가 아니라 실패 계층을 다시 분리하는 gate다.

## 10. 현재 결론과 바로 다음 작업

현재 `R1`은 완료됐다. 이 결과는 public constant-heading Actor와 합성 관측의 prediction
계약이 감사 가능하고, Ideal 입력에서 결정론적 포함이 성립한다는 뜻이다. Normal·Stress의
통계적 miss와 public corpus에 없는 가속·감속·정지·회전은 그대로 limitation이다.

`R2` 상세 명세는
[`11-witness-automation-and-generalization.md`](11-witness-automation-and-generalization.md)로
완료했다. 바로 다음 작업은 해당 문서 순서에 따라 계약·projection·독립 validator를 먼저
구현하는 것이다. 자동 search는 그 경계가 시험으로 닫힌 뒤 추가한다.

이 master specification 자체는 지역 수정이나 DWB를 제품 기능으로 채택하지 않는다.
