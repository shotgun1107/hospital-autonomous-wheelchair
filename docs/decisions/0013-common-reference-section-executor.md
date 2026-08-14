# ADR 0013: R5 Controller 비교의 공통 Reference Section Executor

- 상태: 사용자 개인 연구 방향, 팀 합의 전
- 날짜: `2026-08-14`
- 범위: 동적 지역 기동 연구 R5, Python `simulation_only`

## 배경

R4의 `LocalManeuverReference`는 단순 polyline이 아니다. 위치를 따라가는 translation과 같은
위치에서 heading만 바꾸는 `ROTATE`, 실제 정지를 요구하는 `STOP_MARKER`, terminal
`REJOIN`을 함께 보존한다.

기존 RPP와 source-derived DWB에 이 reference를 그대로 넣으면 비교 의미가 깨진다.

- RPP는 local window의 마지막 점을 실제 goal로 보아 window마다 감속할 수 있다.
- DWB core의 `set_path()`는 모든 critic을 reset하므로 window 이동마다 oscillation 상태가
  사라질 수 있다.
- DWB의 path-scoring target과 goal-rotation target이 한 path에 묶여 있어 local window 끝을
  최종 goal처럼 취급할 수 있다.
- 일반 polyline 추종은 동일 위치의 entry·exit knot로 표현된 제자리회전을 실행하지 못한다.
- controller마다 정지·회전 완료 의미가 다르면 RPP와 DWB의 paired 비교가 오염된다.

## 결정

R5에는 RPP와 DWB 위에 공통 `ReferenceSectionExecutor`를 둔다.

```text
R4 full reference + current sliding window
                 ↓
       ReferenceSectionExecutor
         ├─ translation → selected controller
         ├─ planned stop → common bounded stop
         ├─ ROTATE       → common stop-then-rotate
         └─ terminal     → common stopped dwell
                 ↓
        reference-bound result
                 ↓
          shared safety gate
```

다음 경계를 고정한다.

- RPP와 DWB는 translation section만 서로 다른 방식으로 추종한다.
- `ROTATE`, planned stop, terminal stop·dwell은 같은 executor가 같은 수치로 실행한다.
- planned section stop은 정상 경로 실행이며 보호정지나 `stop_epoch` 증가로 오인하지 않는다.
- `HOLD`는 자동 release하지 않는다. 위험 해소·빈 관측·경로 존재만으로 다음 section으로
  넘어가지 않는다.
- full reference terminal과 current window endpoint를 구분한다.
- 같은 full reference의 window 이동은 controller session과 stateful critic을 reset하지 않는다.
- 새 maneuver/path/session/stop epoch는 기존 executor와 controller state를 폐기한다.
- controller result와 shared gate context는 reference session·revision·hash를 왕복 검증한다.

## 필요한 API 분리

기존 controller API를 그대로 재사용하지 않는다. R5 adapter는 최소한 다음 의미를 분리해야 한다.

```text
begin_reference_session(full_reference, session_binding)
update_scoring_window(local_window)
step(current_tick_input)
invalidate(reason)
```

DWB에서는 `set_path()`의 현재 reset 의미를 다음 두 동작으로 분리한다.

```text
new full reference/session
→ stateful critic·goal·oscillation reset

same-session local window update
→ path/goal distance field만 갱신
→ oscillation·section progress·session state 유지
```

RPP에서는 local window를 lookahead 검색에 사용하되 속도 감속과 완료 판정은 active section의
명시적 stop marker 또는 full reference terminal에만 결박한다.

## 이유

공통 executor가 없으면 비교 결과에 controller 알고리즘 외의 차이가 섞인다.

- 한 controller는 rotation marker를 무시하고 다른 controller는 정지·회전할 수 있다.
- 한 controller는 window마다 reset되고 다른 controller는 state를 유지할 수 있다.
- local window 끝에서 발생한 가짜 감속을 경로추종 성능으로 오인할 수 있다.
- checkpoint별 성공을 연속 closed-loop 성공처럼 이어 붙일 수 있다.

공통 executor는 이 차이를 제거하고 R5가 translation tracking 차이를 비교하도록 한다.

## 고려한 대안

### 각 controller가 section 의미를 자체 구현

controller 고유 동작과 section 실행 정책을 분리할 수 없고 paired 비교가 불공정해진다.

### rotation knot를 제거하고 polyline만 전달

R3·R4가 보존한 제자리회전과 stop marker를 잃으므로 허용하지 않는다.

### 매 window마다 새 controller 생성

구현은 단순하지만 [`ADR 0012`](0012-persistent-controller-session-for-sliding-subpaths.md)의
persistent session 계약을 위반한다.

### full reference 전체만 항상 전달

window reset 문제는 피할 수 있지만 R4 sliding subpath의 입력 크기·현재 subgoal·stale result
계약을 검증하지 못한다.

## 결과와 비용

장점:

- RPP·DWB에 동일한 planned stop·rotation·completion 의미를 준다.
- local window와 실제 terminal goal을 구분한다.
- same-session state 유지와 stale result 폐기를 시험할 수 있다.
- path 생성 실패, section 실행 실패, controller 추종 실패와 gate 거부를 분리할 수 있다.

비용:

- 기존 RPP와 DWB를 직접 호출하지 않고 R5 adapter를 새로 작성해야 한다.
- DWB critic의 session reset과 scoring-path update API를 분리해야 한다.
- shared gate에 optional reference binding 검증을 추가해야 한다.
- planned stop과 protective stop의 로그·상태를 별도로 보존해야 한다.

## 안전·증거 경계

- executor command도 shared safety gate를 우회하지 않는다.
- planned stop 완료는 보호정지 해제나 이동 권한이 아니다.
- `HOLD` section은 현재 epoch용 재승인과 새 reference 재검증 없이 자동 해제하지 않는다.
- R5 path-only 통과는 Actor online 안전, 카메라 통합, 제품 controller 채택 또는 실제 사람
  탑승 안전의 증거가 아니다.

## 연결 문서

- [`R5 Persistent Controller 비교 상세 명세`](../research/dynamic-actor-experiment/16-persistent-controller-comparison.md)
- [`ADR 0012`](0012-persistent-controller-session-for-sliding-subpaths.md)
- [`R4 Reference·Sliding Subpath 상세 명세`](../research/dynamic-actor-experiment/15-local-maneuver-reference-contract.md)
- [`경로 안전·권한 흐름`](../safety/path-safety-authority-flow.md)
