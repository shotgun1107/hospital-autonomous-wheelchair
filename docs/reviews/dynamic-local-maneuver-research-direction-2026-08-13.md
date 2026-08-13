# 동적 지역 기동 연구 방향 판정과 자료 출처

## 1. 문서 상태

- 작성일: `2026-08-13`
- 상태: **사용자 개인 연구 방향 승인**
- 팀 전체 합의: **아님**
- 제품 알고리즘 채택: **아님**
- `G1~G5` 결정 및 경로 분석 7단계: **미수행**
- 증거 범위: 합성 지도·가상 차체·open-loop 원형 Actor를 사용하는
  `simulation_only` Python 연구

이 문서는 기존 DWA/DWB 기능 실패와 집 PC의 지역 우회 탐색을 바탕으로, 다음 공개
연구에서 어떤 계층과 실험을 먼저 검증할지 정리한다. 실제 사람 탑승 안전성, 의료기기
적합성, 실제 차체 수치 또는 최종 MVP 구성을 확정하지 않는다.

## 2. 검토 자료와 출처 사슬

이번 방향은 다음 순서의 자료를 기반으로 한다.

1. 최초 Pro 검토 입력
   - 저장소 문서:
     [pro-path-algorithm-research-prompt-2026-08-12.md](pro-path-algorithm-research-prompt-2026-08-12.md)
2. 최초 Pro 판정문
   - 시작 문구: `# A. 최종 판정`
   - 원문 SHA-256:
     `ab04d1f972b5160d6e79e343b6b784f23030f7df7e88b2fedac9884f60153049`
3. Claude Opus 5 Max 문헌조사 원문
   - 시작 문구: `문헌 조사를 마쳤습니다.`
   - 원문 SHA-256:
     `f668ad8e105ec3006539b8b55a7cf887357ecde36963c7f64271ff6a17a9564e`
4. Claude 자료를 첨부해 Pro에 보낸 실제 후속 프롬프트

   > 자 너가 생각한 기반 새로운 자료느낌으로 받아들여 아래는 클로드 opus 5 max의
   > 자료조사이다 해당 자료기반을 재추론하여 코덱스가 요청했더 형식으로 추론해뵈

5. Claude 자료를 받은 Pro의 재판정문
   - 시작 문구: `아래 판정은 첨부된 Claude Opus 5 Max 자료를 새로운 연구 입력으로`
   - 원문 SHA-256:
     `8518ffdb928e97a3f45940e174c0ec4a8534144f7ec1d98e91a912c6c8e6c791`

위 해시는 제공된 원문의 식별값이다. 해시만으로 원문 내용이 저장소에 보존되는 것은
아니므로, 이 문서는 연구 방향에 영향을 준 결론과 보정사항을 함께 기록한다.

## 3. 최종 판정

판정은 **수정 후 유지**다.

현재 상위 계층은 유지한다.

```text
전역 통로·경로 선택
→ 지역 기동 판단과 후보 생성
→ 지역 우회 경로·reference 생성
→ 경로 추종 또는 국소 궤적·속도 선택
→ 공통 online safety gate
→ 제한 감속·실제 정지·hold
```

현재 실패를 `DWA/DWB 알고리즘 계열 전체의 실패`로 해석하지 않는다. 더 정확한 해석은
다음 요소가 결합했다는 것이다.

```text
의미 있는 지역 우회 path 부족
+ 단일 직사각형·singleton waypoint 자극
+ DWB의 짧은 horizon과 path·goal critic 선호
+ 방향 관측의 불확실성과 보수적 hold
+ waypoint별 controller 재생성에 따른 상태 초기화
```

집 PC에서 확인한 최대 측면 이탈 `0.221726m`는 외부 지역 reference가 DWB의 실제
측면 이탈을 유도할 수 있다는 메커니즘 증거다. Actor 추월, 원 경로 재합류, 재합류
유지와 목적지 도착은 아직 증명하지 못했다.

## 4. 교정된 연구 구조

### 4.1 Online 실행 계층

```text
A* / D* Lite
→ 기동 후보 관리자
   - WAIT_OR_FOLLOW
   - PASS_LEFT
   - PASS_RIGHT
   - GLOBAL_REROUTE_REQUEST
   - SUPPORT_REQUEST
→ bounded 지역 경로 생성기
→ sliding local subpath
→ persistent RPP 기준선 / persistent DWB 비교군
→ shared safety monitor·arbitrator
→ 현재 검증된 recovery: braking → actual stop → hold
→ 구동 limiter·로컬 정지·물리 비상정지
```

DWB는 경로에서 일부 이탈한 속도 궤적을 고를 수 있지만, 안전한 좌·우 통과 topology를
항상 새로 생성하는 planner로 취급하지 않는다. 별도 지역 경로 생성기가 기동 구조와
재합류 방향을 제공하고, DWB는 이를 지속 상태로 실행하는 비교 controller로 둔다.

### 4.2 Offline 증거 계층

```text
정확한 지도·차체·Actor ground truth
├─ 공간 수행 가능성 oracle
├─ 시간 수행 가능성 oracle
└─ ground-truth evaluator
```

- 공간 oracle 후보: bounded State Lattice
- 시간 oracle 후보: 기존 time-indexed witness의 자동화·일반화, 필요한 경우
  Kinodynamic SIPP 또는 time-indexed primitive search
- evaluator: 실제 swept clearance, 충돌, 금지구역, 추월, 재합류와 도착 판정

State Lattice와 SIPP는 제품 채택 후보가 아니라 controller 실패와 장면 비성립을
분리하기 위한 offline 연구 도구다.

## 5. Pro·Claude 판정에서 반드시 보정할 사항

### 5.1 Gaussian `2σ`와 결정론적 포함을 분리한다

Gaussian 잡음은 이론적으로 무한한 꼬리를 가지므로 Normal·Stress의 일반 관측에서
다음 조건을 hard criterion으로 요구하지 않는다.

```text
모든 ground-truth sample이 2σ Capsule 안에 존재
```

대신 다음을 분리한다.

```text
deterministic_motion_containment
= 동결한 Actor 속도·가속·방향변화 범위를 운동 envelope가 포함

statistical_observation_coverage
= Gaussian 관측오차가 2σ 범위에 포함된 경험적 비율
```

`2σ`는 연구용 보수 모델이지 확률적 안전보장이나 실제 사람의 완전한 reachable set이
아니다. 실제 hard-safety 결과는 독립 ground-truth evaluator로 판정한다.

### 5.2 revision 문제의 현재 심각도를 구분한다

`path_revision`, `maneuver_revision`, `subgoal_revision`과
`controller_session_id`가 없는 문제는 현재 동기식 simulator에서는 controller instance
재생성에 따른 비교·연속성 **P1**이다.

다만 비동기 계산, ROS 2, 별도 process 또는 실제 command queue로 넘어가기 전에는
이전 결과가 새 경로에서 실행될 수 있으므로 **P0 실행 차단 계약**으로 승격한다.

최종 명령은 최소한 다음 문맥에 결박한다.

```text
mission_id
stop_epoch
maneuver_revision
path_revision
subgoal_revision
controller_session_id
observation_revision
control_tick
```

### 5.3 기존 feasible witness의 증거를 축소하지 않는다

현재 feasible witness는 이미 다음을 검사한다.

- `20Hz` 차체 운동학
- 선·각속도와 가감속
- exact oriented footprint
- Normal·Stress time-indexed Actor tube
- ordered overtake
- 재합류와 terminal dwell

따라서 누락된 것은 단순한 시간 검사가 아니다. 현재 부족한 것은 다음이다.

- 자동 경로 탐색
- 여러 장면으로의 일반화
- positive·wait-only·forbidden·no-solution의 독립 판정
- ground-truth feasible과 observation-limited decidable의 구분

SIPP는 기존 witness를 대체하기보다 시간 oracle을 자동화·확장할 필요가 확인됐을 때
추가한다.

### 5.4 `DWPP`는 미정의 상태로 유지한다

제공된 Claude·Pro 자료에는 `DWPP`의 정확한 정의, 버전, 구현 경계와 현재 프로젝트
인터페이스가 확정돼 있지 않다. 의미와 출처를 별도로 검증하기 전에는 구현 후보,
기준선 또는 Nav2 채택 사실로 사용하지 않는다.

## 6. 문제 우선순위

### 다음 공식 연구 결과 전에 닫을 계약

1. Actor generator의 결정론적 운동 범위와 predictor 운동 envelope의 일치
2. Gaussian 관측 coverage와 deterministic motion bound의 분리
3. persistent controller 도입 전 revision·command 폐기 계약
4. controller stop과 gate override의 원인별 귀속

### 현재 기능 해석을 왜곡하는 P1

- 단일 `0.70m` 직사각형 기동만 생성
- 다음 waypoint 한 점만 전달해 접선과 재합류 방향 손실
- waypoint마다 DWB instance 재생성
- 2초 controller rollout에 전체 추월 기동을 기대
- 단일 dropout·stale에서 방향 상태를 과도하게 초기화
- Ideal 부분 성공을 Normal 기능 증거로 확대할 위험
- checkpoint metadata와 연속 상태 동등성 혼동

## 7. 다음 공개 연구 순서

이 절의 실행 계약은
[`동적 지역 기동 연구 R1~R7 Master Specification`](../research/dynamic-actor-experiment/10-dynamic-local-maneuver-research-master-spec.md)으로
구체화한다. 제품 경로분석 단계와 기존 동적 Actor 구현 단계 번호와의 혼동을 막기 위해
master 명세에서는 아래 단계를 `R1~R7`로 표기한다. 이 문서는 방향과 근거의 정본이고,
master 명세는 단계별 입력·출력·완료 gate의 정본이다.

### 1단계 — prediction 계약 감사

- deterministic Actor motion containment와 statistical observation coverage를 분리한다.
- Ideal·Normal·Stress, waypoint 방향전환, 감속·정지, dropout·TTL 경계를 검사한다.
- 안전 수치나 Capsule을 결과에 맞춰 축소하지 않는다.

### 2단계 — 기존 witness 자동화·일반화

- 상세 구현 계약은
  [`R2 witness 자동화·일반화 명세`](../research/dynamic-actor-experiment/11-witness-automation-and-generalization.md)를
  따른다.
- 수동 witness를 자동 실행 가능한 공개 oracle로 정리한다.
- positive, wait-only, forbidden, no-solution 장면을 포함한다.
- 공간 불가, 시간 불가, 관측상 판단 불가를 서로 다른 결과로 기록한다.

### 3단계 — bounded 공간 oracle 연구

- bounded State Lattice를 첫 공간 수행 가능성 후보로 검토한다.
- exact footprint, 금지구역, 추상 motion primitive, 재합류 pose·heading과 terminal
  stopping을 검사한다.
- 실제 motion primitive 수치는 `G2` 전에는 가상 프로필로만 둔다.

### 4단계 — 복수 기동 후보와 reference 계약

```text
WAIT_OR_FOLLOW
PASS_LEFT
PASS_RIGHT
```

- 단일 `0.70m` offset은 공개 비교 자극 하나로만 유지한다.
- 각 후보는 pose와 접선, 재합류 방향, path·maneuver revision을 포함한다.
- 단일 점 대신 방향 정보를 가진 sliding local subpath를 사용한다.

### 5단계 — 동일 witness controller 비교

```text
persistent RPP
vs
persistent DWB
```

- 같은 local path, 관측, gate, 차체, 속도와 시작 상태를 사용한다.
- controller instance를 waypoint마다 새로 만들지 않는다.
- Ideal → no-dropout → Normal public → Stress degradation 순서로 실행한다.

### 6단계 — 연속 공개 종단 자격

다음 순서를 하나의 연속 episode에서 증명한다.

```text
측면 이탈
→ Actor 존재 중 안전한 통과
→ 투영 순서 역전
→ 원 경로 재합류
→ 0.5초 이상 재합류 유지
→ 지역 기동 종료 또는 목적지 도착
```

함께 확인한다.

- 충돌·금지구역 진입 `0`
- stale·다른 revision 명령 적용 `0`
- 무단 자동 재개 `0`
- gate override 원인과 최대 연속 길이
- full-state 연속 실행 증거
- 최신 전체 회귀

### 7단계 — 후속 실행 순서

- 1단계 예측 계약 감사는
  [`09-prediction-contract-audit.md`](../research/dynamic-actor-experiment/09-prediction-contract-audit.md)에
  따라 결정론적 운동 포함과 통계적 coverage를 분리한다.
- Python에서 위 기능 구조를 먼저 증명한다.
- 기능이 닫힌 뒤에만 계산 kernel의 C++ 이전을 검토한다.
- 공개 기능·안전·연산 자격이 닫힌 뒤 별도 승인으로 새 hidden을 생성한다.
- 기존에 소비한 hidden은 최종 증거로 재사용하지 않는다.

## 8. 하지 말아야 할 수정

1. Actor 반경, `0.08m` clearance와 terminal stopping 완화
2. hidden·실행 결과에 맞춘 prediction envelope 축소
3. `0.70m`를 통과할 때까지 조정해 일반 planner로 선언
4. singleton waypoint 성공을 추월·재합류 성공으로 기록
5. controller instance 재생성으로 상태 문제를 해결했다고 간주
6. DWB critic 여러 개를 동시에 제거
7. ground truth 또는 expectation label을 online controller·gate에 전달
8. Ideal·no-dropout 결과를 Normal·Stress 증거로 승격
9. checkpoint 조각을 연속 hard-safety 증거로 연결
10. 기능 구조가 닫히기 전 전체 C++·ROS 2 이식
11. MPPI·TEB·MPC를 최신이라는 이유만으로 우선 구현
12. shared gate가 막았다는 이유로 controller 자체가 안전하다고 주장
13. State Lattice·SIPP·RPP·DWB 중 하나를 제품 알고리즘으로 채택
14. `G1~G5` 또는 경로 분석 7단계를 완료했다고 선언

## 9. 결정 권한 구분

### Software A가 연구 명세로 정리 가능

- 기동 후보 taxonomy
- spatial/time/observation feasibility 구분
- local path·sliding subpath 계약
- path·maneuver·subgoal revision 관계
- persistent controller reset 정책
- critic·gate override 귀속 로그
- 기존 witness 자동화·일반화 계획
- State Lattice·SIPP 연구용 추상 입출력
- checkpoint full-state digest 계약

### 팀과 합의 필요

- 실제 footprint와 방향별 돌출부
- 제자리 회전·후진 허용 여부
- 실제 선·각 가감속과 정지거리
- 센서 주기·지연·covariance
- 실행 보드와 계산 자원
- 실제 정지 완료의 권위 있는 신호
- path revision 전달 책임
- 서버·로봇·MCU의 안전 권한 경계
- 동적 사람 추월의 MVP 포함 여부

## 10. 최종 연구 방향

Claude 조사에서 채택할 핵심은 특정 최신 알고리즘이 아니다. 현재 실패를 다음 계층으로
분해해 검증해야 한다는 점이다.

```text
기동 선택
→ 지역 경로 생성
→ persistent controller
→ shared safety gate
→ offline feasibility oracle와 ground-truth evaluator
```

현재의 다음 연구 기본 순서는 다음과 같다.

```text
prediction 계약 감사
→ 기존 witness 자동화·일반화
→ bounded State Lattice 공간 oracle 검토
→ 필요할 때 시간 oracle 확장
→ WAIT/LEFT/RIGHT 지역 기동 생성
→ persistent RPP 기준선
→ persistent DWB 비교
→ 공개 Normal 자격
→ 이후에만 새 hidden
```

이 순서는 제품 알고리즘 순위나 팀 결정이 아니라, 현재 실패 원인을 가장 적은 혼합으로
분리하기 위한 개인 연구 순서다.
