# 동적 지역 기동 연구 R1~R7 Master Specification

## 1. 문서 상태와 목적

- 작성일: `2026-08-13`
- 상태: 사용자 개인 연구 방향을 실행 가능한 단계 계약으로 구체화한 기준선
- 기능·안전 증거 범위: `R1~R6` Python `simulation_only`, 합성 지도, 가상 차체,
  open-loop 원형 Actor
- 연산 성능 증거 범위: `R7`의 frozen native(C++) 구현·고정 머신 qualification만 해당
- 팀 전체 합의: 아님
- 제품 알고리즘 채택: 아님
- `G1~G5` 결정과 제품 경로분석 7단계: 미수행
- 현재 단계: `R1 prediction 계약 감사 완료`,
  `R2-A 공개 ground-truth witness 감사와 좌·우 PASS 탐색 완료`,
  `R2-B 관측·prediction hard failure 2건으로 후속 보류`,
  `R3 public 21/21·receipt·729 전체 회귀 완료`,
  `R4 v1 public 21/21·receipt·794 전체 회귀 완료`,
  `R5-A v3 signed static public 21/21·receipt 완료`,
  `R5-B 공개 Ideal same-direction 10/10·횡단 좌우·다중 위험 기능 완료`,
  `R5-C Normal 횡단 좌·우 복구·원 경로 복귀·도착 완료, Stress 좌·우 보수 정지 완료`,
  `R6 공개 연속 종단 17/17·hard failure 0·receipt·945 전체 회귀 완료`,
  `R7 Python↔C++ 동일성 5/5 통과·50ms 301/500 초과로 자격 실패·hidden 차단`

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
동결 native 구현의 실제 계산시간이 실행 주기를 만족하지 못함
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

### 4.4 시간 영역, 결정론과 실행 자원

시간은 다음 네 영역으로 분리한다.

| 시간 영역 | 의미 | 판정 사용 범위 |
|---|---|---|
| `T_sim` | Actor·차체 운동, 관측 timestamp·latency·TTL, 제어 주기, 고정 적용 지연, 제동과 episode duration | `R1~R6` 기능·안전 판정에 사용 |
| `T_fault` | stale·late result·통신 지연을 재현하기 위해 명시적으로 주입한 결정론적 simulation-time delay | 계약·fail-closed 시험에 사용 |
| `T_wall_python` | Python 하네스·oracle·controller·시험이 실제 PC에서 소비한 시간 | 운영·병목 진단만, 합격·탈락 금지 |
| `T_wall_native` | semantic parity를 통과한 frozen native(C++) 구현의 고정 머신 실행시간 | `R7` 연산 자격에서만 사용 |

- 문서와 결과에서 단순히 `time`이라고 쓰지 않고 가능한 한 위 시간 영역을 명시한다.
- `R1~R6`의 `time_s`, 관측 준비 `2.00s`, `20Hz`, `50ms`, Actor 활성시각과 episode
  duration은 모두 `T_sim`이다. Python 함수가 실제로 소비한 시간이 아니다.
- Python 함수·후보 하나·episode·전체 회귀의 wall-clock, CPU 사용률, 프로세스 수, RSS와
  cache 상태는 알고리즘의 `FEASIBLE`, `NO_WITNESS`, hard safety 또는 functional gate를
  변경하지 않는다.
- Python timeout, OOM, worker crash, 사용자의 중단과 머신 오류는
  `INFRASTRUCTURE_INCOMPLETE`로 기록한다. 완료되지 않은 결과를 `NO_WITNESS`,
  `RESOURCE_LIMIT`, timing failure 또는 알고리즘 실패로 바꾸지 않는다.
- 동결 candidate·expansion limit은 search가 실제로 검사한 유한 범위를 정의하는 semantic
  config다. 이 count limit 도달은 `SEARCH_INCONCLUSIVE/RESOURCE_LIMIT`이며 Python이 느리다는
  판정이나 일반 해 부재가 아니다.
- Python의 late-command 계약시험은 측정된 함수 실행시간이 아니라 `T_fault`로 늦은 결과를
  주입해 검증한다. 실제 연산 deadline miss는 `R7`의 `T_wall_native`에서만 판정한다.
- 서로 독립적인 public episode와 독립 후보 shard는 process 기반으로 병렬 실행할 수 있다.
  shard 내부 frozen 순서와 ordinal을 유지하고, parent는 gap·overlap 없이 결정론적으로 환원한다.
- paired controller 비교는 같은 worker에서 같은 seed·관측 stream을 사용한다.
- 병렬 결과는 corpus 입력 순서로 다시 정렬해 worker 완료 순서의 영향을 없앤다.
- 한 episode의 상태 의존 20Hz tick은 직렬로 실행한다. 독립 후보 평가는 위 shard 계약 아래에서만
  병렬화한다.
- 명령·상태·event·metric은 동결 tolerance 안에서 결정론을 만족한다. wall-clock, worker 번호와
  완료 순서는 semantic hash에서 제외한다.
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

`INFRASTRUCTURE_INCOMPLETE`는 위 evidence taxonomy가 아니라 실행 완료 상태다. timeout·OOM·
worker crash·I/O 실패·사용자 중단으로 semantic 평가가 끝나지 않았음을 뜻하며, 이 상태에서는
episode evidence를 생성하지 않고 partial 진단만 보존한다.

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

[`ADR 0011`](../../decisions/0011-separate-path-and-perception-research-gates.md)에 따라 R2
완료판정은 다음 두 lane으로 분리한다.

```text
R2-A: exact ground-truth Actor 기반 시간 경로 존재성
R2-B: 관측·prediction으로 해당 기동을 판단할 수 있는지
```

R2-B 실패는 perception-integrated 종단 자격을 막지만, observation을 입력으로 받지 않는
R3 static 공간 oracle을 자동으로 막지 않는다.

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
- R2-A ground-truth hard failure는 해당 경로 후보를 R3·R4에 전달하지 않는다.
- R2-A의 `SEARCH_INCONCLUSIVE`는 불가능 판정이 아니라 R3 공간 oracle 입력이다.
- R2-B hard failure는 관측 통합 R5~R7과 hidden을 막지만 R3 공간 연구를 막지 않는다.

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

상세 구현 계약은
[`15-local-maneuver-reference-contract.md`](15-local-maneuver-reference-contract.md)를 따른다.

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
- 같은 R4 v2 `travel_direction`을 사용하고 방향 전환 전 같은 실제 정지 확인 적용
- reverse section은 두 controller 모두 `-0.10m/s <= v <= 0`, forward section은 `v >= 0`
- `Ideal → no-dropout → Normal public → Stress degradation` 순서

### 출력 지표

- 기동 시작, 실제 측면 이탈과 path tracking error
- ordered overtake와 rejoin 여부·지속시간
- completion, traffic wait와 planner deadlock
- controller stop request, no-safe candidate와 gate override 원인
- 최소 ground-truth clearance, path length, jerk와 각운동 지표
- 후보 수·illegal taxonomy
- Python wall-clock·CPU·memory 관측값은 별도 non-qualification 병목 진단

### 완료조건

- controller별 명령·상태·event가 repeated public run에서 결정론적이다.
- 기능상 progressable인 Ideal·no-dropout 사례에서 기대 기동을 실행한다.
- hard safety 실패가 있는 비교 결과로 우열을 계산하지 않는다.
- Normal에서 기능 실패 시 prediction, reference, controller와 gate 원인을 분리한다.
- Stress는 안전 열화 시험이며 무조건 임무 완료를 요구하지 않는다.
- RPP 또는 DWB 한 구현의 실패를 알고리즘 계열 전체의 실패로 확대하지 않는다.
- Python wall-clock을 controller 기능·안전 또는 알고리즘 계열의 합격·탈락에 사용하지 않는다.
- signed translation 방향 위반, 무정지 방향 전환과 reverse 속도 초과가 `0`이다.
- reverse rollout·terminal stopping의 뒤쪽 swept safety가 shared gate에서 동일하게 검증된다.

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

### 실행과 기능 자격

- 독립 episode의 기능·안전 평가는 process 병렬화할 수 있다.
- paired 조건은 같은 worker에 유지한다.
- Python wall-clock은 운영 metadata와 병목 후보로만 기록하고 R6 자격조건에 포함하지 않는다.
- stale·late-command 적용 금지는 결정론적으로 주입한 `T_fault`로 검사한다.
- native wall-clock qualification은 R6 결과·source·parameter를 동결한 뒤 R7에서 수행한다.
- 기능 미달을 C++ 이식으로 숨기지 않는다.

### 완료조건

- 모든 요구 public episode가 중간 checkpoint 없이 완료 또는 정당한 보수적 종료된다.
- 최신 전체 회귀, Ruff, source freeze와 결과 hash가 통과한다.
- 부분·축소 실행은 report-only이며 정식 public qualification receipt를 만들지 않는다.
- 기능·안전 중 미완료 항목을 숨기지 않는다. 연산 자격은 R7 진입 전까지 `미측정`으로 둔다.

## R7 — 후속 실행·Native·Hidden 진입 Gate

### 성격

`R7`은 새로운 planner 구현 단계가 아니다. `R1~R6`의 증거를 보고 다음 연구 실행을 허용할지
판정하는 release gate다.

### 순서

1. Python에서 prediction → oracle → reference → persistent controller → gate의 기능·안전 구조를
   먼저 증명한다.
2. Python profiler는 native 이식 범위를 찾는 진단에만 사용하고 deadline 합격·실패를 판정하지
   않는다.
3. 동결된 의미·후보·안전·평가 기준을 바꾸지 않고 실제 실행 대상 kernel을 C++로 이전한다.
4. Python과 native의 controller·diagnostic·safety semantic parity를 확인한다.
5. parity를 통과한 native build만 고정 머신에서 직렬 wall-clock qualification한다.
6. 공개 기능·안전·native 연산 자격과 manifest를 동결한다.
7. 사용자 별도 승인 뒤 새 hidden seed commitment를 생성한다.
8. 새 hidden을 한 번 실행하고 결과를 변경 없이 보존한다.

### Native 연산 자격 조건

native timing manifest에는 최소한 다음을 동결한다.

- compiler·build type·optimization flag·native source hash
- 실행 머신·CPU model·physical/logical core·OS·전원 정책
- process/thread 수, CPU affinity와 background-load 정책
- warm-up 횟수, warm-cache와 cold-start 측정 구분
- allocator·동적 할당 허용 범위, peak RSS와 page-fault 관측
- 후보 수·Actor 수·map geometry를 결박한 qualification snapshot set
- monotonic high-resolution clock, p50·p95·p99·maximum과 deadline miss 수

주 자격은 CPU contention이 없는 직렬 lane에서 수행한다. 별도의 contention·cold-cache lane은
degradation 자료이며 주 자격 결과를 대체하거나 유리한 결과만 선택하는 데 사용하지 않는다.

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

공개 기능 통과 + native timing 미달
→ native 최적화 후보, hidden 금지

공개 기능·안전·native timing 통과
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
| `R1` | prediction 계약이 generator와 맞는가? | 공개 motion·관측·Capsule audit | 기존 narrow claim 완료, total Actor coverage는 R2-B 후속 | hard failure 0 |
| `R2-A` | exact ground truth에서 안전한 time-indexed witness가 있는가? | 자동 witness·음성 판정·taxonomy | 공개 19개 audit와 좌·우 PASS 탐색 완료, legacy 횡단·재정지 표적 보완 완료 | 검증된 source만 R3·R4 전달 |
| `R2-B` | 관측·prediction으로 기동을 판단할 수 있는가? | profile replay·역방향 coverage | hard failure 2건, 카메라와 함께 후속 보류 | 관측 통합 전 필수 |
| `R3` | 정적 공간에서 차체가 통과·재합류할 수 있는가? | bounded 공간 oracle | public `21/21`, clean-source receipt, 729 전체 회귀 완료 | 검증된 feasible 결과만 R4 전달 |
| `R4` | WAIT/LEFT/RIGHT와 signed travel direction을 reference로 표현하는가? | revision 결박 local path·subpath | v1 public `21/21`, ready 8, clean receipt, 794 전체 회귀 완료. v2 `travel_direction` clean public `21/21`, ready 8, relation failure 0, receipt 완료 | R5 v2 signed controller 전달 |
| `R5` | 같은 signed reference에서 controller 차이가 무엇인가? | persistent RPP·DWB paired 결과 | R5-A signed static public 완료. C++ DWB Ideal same-direction `10/10`, 횡단 좌·우와 다중 위험 완료. R5-C Normal 횡단 좌·우 도착, Stress 좌·우 출발 없는 보수 정지 완료 | 최신 R5-B/C 결과를 R6 연속 실행에 전달 |
| `R6` | 연속 공개 episode의 기능·안전 계약이 닫히는가? | public 종단 report·receipt·회귀 | 완료. 공개 `17/17`, hard failure `0`, 전체 회귀 `945 passed`, receipt 생성 | R7은 별도 시작 지시 뒤 검토 |
| `R7` | native 연산 자격과 새 hidden 진입 자격이 있는가? | semantic parity·native timing manifest·승인 | 측정 완료·자격 실패. 동일성 `5/5`, 50ms 초과 `301/500`, hidden 차단 | 동작 보존형 native 최적화는 별도 시작 지시 필요 |

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
12-pass-structured-witness-search.md              # R2-PASS
13-witness-profile-replay.md                     # R2 profile replay
14-bounded-spatial-oracle.md                     # R3
15-local-maneuver-reference-contract.md          # R4
16-persistent-controller-comparison.md           # R5
25-r6-public-end-to-end-qualification.md          # R6
26-r7-native-release-gate.md                      # R7
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
- Python wall-clock·CPU 사용률·메모리·cache 상태를 기능 또는 알고리즘 가능성 판정에 사용
- timeout·OOM·worker crash·사용자 중단을 `NO_WITNESS`, `RESOURCE_LIMIT` 또는 알고리즘 실패로
  오분류
- 실제 사람·센서·차체 증거로 확대 해석

중단은 연구 실패를 숨기는 절차가 아니라 실패 계층을 다시 분리하는 gate다.

## 10. 현재 결론과 바로 다음 작업

현재 `R1`은 완료됐다. 이 결과는 public constant-heading Actor와 합성 관측의 prediction
계약이 감사 가능하고, Ideal 입력에서 결정론적 포함이 성립한다는 뜻이다. Normal·Stress의
통계적 miss와 public corpus에 없는 가속·감속·정지·회전은 그대로 limitation이다.

`R2`의 WAIT/HOLD·PASS 구조화 search, Ideal·Normal·Stress replay와 공개 13+6 영구
audit·JSON/Markdown/PNG reporting을 구현하고 전체 `19/19`를 실행했다. 그러나 v6
second-risk와 legacy dynamic-change에서 episode 중간에 새 Actor가 생성된 뒤 관측 latency
동안 fresh EMPTY가 유지돼, 실제 Actor는 존재하지만 Ideal Capsule은 없는 hard failure 2건이
발생했다. 자세한 실행 정본과 수치는
[`R2 공개 Witness 감사 결과`](r2-public-witness-audit-result-2026-08-13.md)에 보존한다.

따라서 기존 결합 R2는 `실행 완료`이지 전체 `자격 완료`가 아니다. 다만 사용자 결정과
[`ADR 0011`](../../decisions/0011-separate-path-and-perception-research-gates.md)에 따라 경로
연구와 카메라·관측 연구를 분리한다. R2-A에서 확인한 WAIT/HOLD·same-direction PASS와
legacy 횡단·재정지 표적 보완 결과를 입력으로 R3 명세를 시작할 수 있다. R2-B의 Actor
entry·visibility·fresh EMPTY와 역방향 coverage는 hard failure 2건을 보존한 채 카메라 통합
후속으로 둔다.

R3의 public `21/21` 공간 자격 뒤 R4 v1은 검증된 LEFT/RIGHT source 8개를 immutable full
reference와 same-session sliding window로 변환했다. clean commit `f43fbbf`에서 R4 public
`21/21`, hard·relation failure 0, parity·repeat determinism, receipt와 전체 회귀 `794 passed`를
확인했다. 이 결과는 online 이동 허가나 perception 통합 완료가 아니다.

R5 상세 구현 계약은
[`16-persistent-controller-comparison.md`](16-persistent-controller-comparison.md)에 둔다. R5는
공통 section executor 위에서 persistent RPP와 source-derived DWB를 비교하며, local window
끝과 full terminal을 분리하고 same-session subgoal update에서 controller state를 보존한다.
R5-A 첫 clean public 실행에서 ready 8개 모두 reverse edge를 포함하지만 R5 v1 controller가
음의 선속도를 지원하지 않는 계약 충돌이 확인됐다. 사용자 승인과
[`ADR 0014`](../../decisions/0014-section-bound-bounded-reverse-translation.md)에 따라 R4 v2가
source primitive의 signed `travel_direction`을 명시하고 R5 v2가 해당 reverse section에서만
최대 `0.10m/s` 후진하도록 보정한다. 기존 v1 결과와 receipt는 보존한다.

R5 v2 signed static reference tracking은 완료됐고, R5-B 공개 Ideal에서 same-direction 좌·우
10-case, 별도 횡단 좌·우와 다중 위험 재정지·복구를 완료했다. source-derived DWB의 후보 생성·
41-pose 적분·7개 critic·선택·Manhattan 거리 지도도 C++20 수치 코어로 옮겨 Python 의미를
보존했다. 이후 R5-C Normal 횡단 좌·우는 입력 상실마다 실제 정지와 새 stop epoch session을
사용하고, 통과 증거 보존 뒤 원 경로 복귀 reference로 각각 tick `1328`, `1432`에 도착했다.
Stress 좌·우는 11개 READY를 연속 확보하지 못해 release·controller call·실제 이동 `0`으로
정지를 유지했다. R2-B 원본 내부 Actor 출현 hard failure는 음성 회귀로 보존한다. 별도 추상
감시 접근 world에서는 지연 Actor를 원래 진입 전부터 같은 track으로 관측해 대표 Ideal miss를
`38/22 → 0/0`으로 제거했다. 그러나 실제 카메라·FOV·가림·검출 증거가 아니므로
perception-integrated R6, 정식 R7 50ms 자격, hidden과 제품 안전 주장을 허용하지 않는다.

R6 공개 연속 종단은 기준 코드 `64df95f`에서 `17/17`, hard failure `0`으로 완료했고 전체
회귀 `945 passed`와 receipt를 남겼다. 세부 결과와 hash는
[`R6 공개 연속 종단 자격 결과`](r6-public-end-to-end-qualification-result-2026-08-17.md)에
보존한다. 이 완료는 합성 `ActorTrack` 기능·안전 근거이며 실제 카메라 통합, R7 시간 자격,
hidden 또는 제품 채택을 자동으로 허용하지 않는다.

R7은 집 PC의 고정 공개 5개 입력을 C++ DWB로 사례당 100회 직렬 측정했다. Python↔C++
결과 동일성은 `5/5`였지만 50ms 초과가 `301/500`, 최대 `321.12ms`여서 시간 자격은
실패했다. qualification receipt와 hidden은 생성하지 않았다. 상세는
[`R7 C++ DWB 시간 자격 결과`](r7-native-release-gate-result-2026-08-17.md)에 보존한다.
다음 native 최적화는 별도 작업이며 이 실패를 후보 수·안전거리·평가 기준 완화로 숨기지 않는다.

이후 `R3~R6` Python 단계는 기능·안전 semantic만 판정한다. Python wall-clock은 병목 진단일
뿐이며, 실제 계산 deadline·CPU·memory·cache 자격은 semantic parity를 통과한 native(C++)
구현으로 `R7`에서만 판정한다.

이 master specification 자체는 지역 수정이나 DWB를 제품 기능으로 채택하지 않는다.
