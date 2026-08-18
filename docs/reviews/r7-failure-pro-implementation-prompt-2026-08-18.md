# Pro 구현 요청 — R7 실패 공개 재현과 Python 수정

## 역할

당신은 이전에 R7 비공개 관측 시험 실패 5건을 분석한 수석 Python 로봇 simulation
개발자다. 이번에는 첨부 코드에 실제 수정안을 구현하고, 표적시험을 실행한 뒤 수정 파일과
결과를 반환하라.

코드를 쓰기 전에 반드시 다음 문서를 먼저 읽어라.

1. `AGENTS.md`
2. `docs/research/dynamic-actor-experiment/28-r7-hidden-observation-result-2026-08-18.md`
3. `docs/research/dynamic-actor-experiment/29-r7-failure-trace-and-public-regression-spec.md`

29번 문서가 이번 구현의 최우선 계약이다. 코드와 문서가 충돌하면 임의로 고치지 말고 충돌
위치와 영향을 먼저 보고하라.

## 현재 상태

- 기준 commit: `a10d1338a3b1b121cdd22090fc6e8663f70c0436`
- R6 공개 종단시험: `17/17`
- R7 Python↔C++ 일치: `5/5`
- R7 50ms 초과: `0/500`
- 비공개시험 전 전체 회귀: `958 passed`
- 비공개 관측 시험: `FAIL`
- Normal 완료: `6/10`
- Stress 최종 정지: `10/10`, 그중 한 건은 중간 출발로 기준 위반
- 실제 충돌·금지구역·0.08m 여유 위반: `0`

## 수정 대상

### 확정 P0-A: BRAKING 중 기존 continuation 발행

공개 입력:

```text
side = LEFT
profile = Normal
observation_seed = 1993037174228324916
prefix tick_limit = 260
현재 실패 = tick 259 R5-B continuation authorization is incomplete
```

의도:

```text
MOVING → BRAKING
→ 기존 runtime에서 controller/continuation 호출 중단
→ 제한감속과 hold만 수행
→ 실제 HOLDING 확인
→ 이전 runtime/issuer/reference/authorization 폐기
→ 새 stop_epoch에 묶인 재출발 대기
```

예외를 `try/except`로 숨기지 말고 불법 continuation 호출 자체를 막아라.

### 확정 P0-B: Stress 재출발 증거 연결 오류

공개 입력:

```text
side = LEFT
profile = Stress
observation_seed = 6422064046178126625
prefix tick_limit = 533
현재 release = 531
현재 first motion = 532
```

재출발은 predictor READY 수가 아니라 shared safety gate가 서로 다른 frame에서 확인한 안전
증거 `11개`를 기준으로 해야 한다. 같은 10Hz frame을 20Hz tick에서 두 번 세면 안 된다.

다음에는 누적 증거를 초기화한다.

```text
stale
dropout/no-frame
prediction loss
unsafe gate decision
stop_epoch 변경
새 reference/session
```

### 확정 P0-C: active section과 local window 불일치

공개 입력:

```text
side = RIGHT
profile = Normal
observation_seed = 4525333994236990214
prefix tick_limit = 504
현재 실패 = tick 503 active_section_not_in_current_window
```

먼저 29번 명세의 trace로 정확한 `catchup_failed_guard`를 확보하라. 그 결과 없이 허용범위를
넓히지 마라.

- 동일 방향의 연속 section을 실제로 통과한 경우라면 각 중간 경계를 증명하는 제한적
  N-step catch-up을 구현할 수 있다.
- required stop, rotate, hold 또는 방향 전환 경계를 local window가 제거한 경우라면 catch-up
  완화가 아니라 active section을 window에 보존하도록 수정한다.

기존 `0.05m` 허용오차를 늘리지 마라.

### P1: Normal 미완료 두 건

```text
RIGHT Normal seed 8970341022568507592
LEFT Normal seed 6422064046178126625
tick_limit 1600
```

상세 trace에서 다음이 확인된 경우에만 복구 코드를 수정한다.

```text
실제 HOLDING 확인
+ 현재 stop_epoch
+ gate-confirmed distinct safe frame 11개
그런데도 새 session을 만들지 못함
```

안전 frame이 부족했다면 정상 fail-closed이므로 무조건 완료시키지 마라. 특히 통과 전 EMPTY
frame을 재출발 증거로 사용하지 마라.

## 구현 순서

### 1단계: 동작 불변 trace

29번 명세에 따라 tick trace 자료형과 선택 가능한 JSONL writer를 구현한다.

- trace on/off 최종 결과 동일
- wall-clock 제외 두 번 실행 지문 동일
- tick 누락·중복 없음
- expectation label을 controller에 전달하지 않음

기존 명령과 상태 전이를 이 단계에서 바꾸지 마라.

### 2단계: 공개 prefix 재현시험

다음 세 시험을 추가하고 수정 전 실패 signature가 실제로 재현되는지 확인한다.

```text
test_normal_left_seed_1993037174228324916_does_not_continue_while_braking
test_normal_right_seed_4525333994236990214_keeps_active_section_representable
test_stress_left_seed_6422064046178126625_requires_gate_confirmed_frames
```

최종 브랜치에는 수정 후 기대조건을 검사하는 시험을 남긴다. 과거 오류를 기대하는 시험을
최종 통과 suite에 남기지 않는다.

### 3단계: P0-A와 P0-B 수정

각각 독립적인 표적시험으로 고친다. 하나의 profile·seed 전용 분기를 만들지 마라.

### 4단계: P0-C 수정

tick 503 trace로 실패 guard를 보고 수정 방법을 선택한다. required-stop·direction-change·
rotate·hold·projection ambiguity 적대시험을 함께 추가한다.

### 5단계: P1 판정과 전체 공개 재생

Normal 미완료 두 건의 마지막 HOLD 원인을 trace로 판정한다. 코드 결함으로 확인된 것만 고친
뒤 공개된 5개 입력을 원래 tick limit으로 실행한다.

## 최종 합격 기준

### Normal 네 건

```text
hard failure 없음
실제 clearance >= 0.08m
post-pass proof 존재
follow-original release 존재
completion 존재
final state COMPLETED
```

단, `6422064046178126625` Normal의 마지막 HOLD가 실제 안전 증거 부족으로 확인되면 억지로
완료시키지 말고 `정상 fail-closed`로 별도 보고한다.

### Stress 한 건

```text
tick 532까지 release 없음
controller call 0
first motion 없음
최종 전체 replay에서도 release·motion 없음
final HOLDING
```

### 공통

```text
충돌 0
금지구역 진입 0
실제 clearance 0.08m 미만 0
무단 재출발 0
stale/invalid 추진 0
```

## 금지

- 안전거리·stale·dropout·재출발 기준 완화
- episode tick 증가로 통과시키기
- 무조건 재출발
- profile, seed, case ID 전용 동작 분기
- 예외 숨기기
- `active_section_not_in_current_window`를 단순 HOLD로 바꿔 증상만 숨기기
- required stop·rotate·hold·방향 전환 건너뛰기
- C++ 코어 수정
- R7 500회 timing 재실행
- 새 hidden 생성·실행
- 제품 알고리즘 선택, `G1~G5`, 실제 사람 안전 주장
- commit, push, PR

## 시험 실행 원칙

1. 정적 검사와 새 단위시험
2. 세 prefix 시험
3. 공개 5건 full replay
4. 관련 기존 시험
5. 모든 수정이 끝난 뒤 전체 회귀 한 번

장시간 시험 전 예상 시간을 계산해 보고하라. 독립 시험은 process로 병렬화하되 wall-clock
성능 판정으로 사용하지 마라.

## 반환물

다음을 하나의 ZIP으로 반환하라.

```text
modified-files/
  실제 수정한 소스·시험 파일, 저장소 상대경로 유지
reports/root-cause.md
reports/changes.md
reports/test-results.md
outputs/trace-prefix-*.jsonl
outputs/public-five-summary.json
SHA256SUMS.txt
```

보고서에는 반드시 다음을 포함한다.

- tick 259의 상태 전이
- tick 503의 정확한 catch-up 실패 guard
- Stress tick 531에서 READY 수와 gate-confirmed 수
- Normal 미완료 두 건의 마지막 HOLD 판정
- 변경 파일 목록
- 실행한 시험과 정확한 통과 수
- 미실행 시험과 이유
- 남은 위험

첨부 ZIP에 필요한 코드가 없으면 구현을 추측하지 말고 정확한 저장소 경로를 요청하라.
