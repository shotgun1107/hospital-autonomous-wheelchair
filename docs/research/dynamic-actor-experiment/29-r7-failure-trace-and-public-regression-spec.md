# R7 실패 추적·공개 재현시험 명세

- 상태: 명세 완료, 공개 회귀 수정·검증 완료(개인 연구 승인)
- 작성일: `2026-08-18`
- 근거 결과: [28-r7-hidden-observation-result-2026-08-18.md](./28-r7-hidden-observation-result-2026-08-18.md)
- 구현 결과: [30-r7-failure-fix-and-public-regression-result-2026-08-18.md](./30-r7-failure-fix-and-public-regression-result-2026-08-18.md)
- 적용 범위: R7 비공개 관측 시험에서 실패한 5건의 원인 확정
- 비적용 범위: 동작 수정, 안전 기준 변경, 새 비공개 시험, 제품 알고리즘 채택

## 1. 목적

현재 결과에는 tick별 상세 기록이 없고 마지막 요약과 기록 지문만 있다. 이 때문에 다음 두
Normal 미완료가 코드 오류인지 올바른 안전정지인지 확정할 수 없고, 경로 구간 오류도 어느
검사에서 발생했는지 알 수 없다.

이 단계의 목적은 다음 세 가지뿐이다.

1. 소비된 비공개 입력 5건을 공개 회귀 입력으로 전환한다.
2. 한 control tick 안에서 관측·안전정지·재출발 권한·경로 구간이 바뀌는 순서를 기록한다.
3. 이후 수정할 정확한 조건을 확정한다.

이 단계에서는 controller, safety gate, 재출발 판단과 경로 구간 동작을 바꾸지 않는다.

## 2. 현재 판정

### 확정된 문제

| 입력 | 확정된 문제 | 현재 우선순위 |
|---|---|---:|
| Normal left `1993037174228324916` | `BRAKING` 상태에서 기존 temporal continuation 발행 시도 | P0 |
| Normal right `4525333994236990214` | executor의 active section이 현재 local window에서 탈락 | P0 |
| Stress left `6422064046178126625` | gate가 확인한 안전 frame이 아닌 predictor READY 수로 재출발 | P0 |

여기서 P0는 제품 전체의 P0가 아니라 **현재 simulation 비교 자격을 막는 연구 하네스 P0**를
의미한다.

### 아직 확정되지 않은 문제

| 입력 | 현재 사실 | 추가로 확인할 것 |
|---|---|---|
| Normal right `8970341022568507592` | 통과·원 경로 복귀 뒤 반복 정지, 최종 HOLDING | 마지막 정지가 복구 누락인지 관측 증거 부족인지 |
| Normal left `6422064046178126625` | 통과 전 반복 정지, 최종 HOLDING | 일반 gate stop 복구 누락인지 정상 fail-closed인지 |

## 3. 공개 입력 전환 규칙

비공개 시험은 이미 완료됐으므로 아래 seed는 더 이상 비공개가 아니다. 공개 회귀시험에서는
기존 `hidden-*` 이름을 쓰지 않고 동작과 seed로 식별한다.

| 공개 ID | side | profile | seed | 진단 종료 tick |
|---|---|---|---:|---:|
| `normal-left-continuation-braking` | LEFT | Normal | `1993037174228324916` | `260` |
| `normal-right-section-window` | RIGHT | Normal | `4525333994236990214` | `504` |
| `stress-left-confirmed-safe-release` | LEFT | Stress | `6422064046178126625` | `533` |
| `normal-right-post-pass-recovery` | RIGHT | Normal | `8970341022568507592` | `1600` |
| `normal-left-pre-pass-recovery` | LEFT | Normal | `6422064046178126625` | `1600` |

다음 정보는 controller 입력에 넣지 않는다.

- 공개 ID
- 과거 비공개 case 이름
- 기대 outcome
- 실패 tick
- Normal 완료 또는 Stress 무출발 판정

controller는 기존과 같이 현재 미션·지도·로봇 상태·관측·예측·reference만 받는다.

## 4. 추적 기록 단위

### 4.1 파일 형식

각 실행은 다음 두 파일을 별도 output 경로에 쓴다.

```text
run-manifest.json
tick-trace.jsonl
```

- `tick-trace.jsonl`은 control tick당 정확히 한 줄을 쓴다.
- 줄 순서는 `tick=0..N` 오름차순이다.
- 중단된 실행의 trace는 원인 조사에는 사용할 수 있지만 최종 수정 자격에는 사용하지 않는다.
- 출력 경로를 덮어쓰지 않는다.
- wall-clock 시간은 기록할 수 있으나 의미 지문과 합격 판정에서는 제외한다.

### 4.2 실행 manifest

`run-manifest.json`에는 최소 다음을 기록한다.

```text
schema
git_head
git_tree
working_tree_clean
public_case_id
side
profile_name
observation_seed
tick_limit
control_period_s
observation_period_s
source_file_hashes
trace_schema_version
```

`public_case_id`와 seed는 runner·시험만 사용하며 controller snapshot에는 넣지 않는다.

### 4.3 tick 공통 필드

모든 tick record에는 다음 필드가 있어야 한다.

```text
tick
simulation_time_s
robot_pose_before
robot_twist_before
robot_pose_after
robot_twist_after
previous_record_hash
record_content_hash
```

pose는 `x_m`, `y_m`, `yaw_rad`, twist는 `linear_mps`, `angular_radps`를 가진다.
첫 record의 `previous_record_hash`는 고정 문자열 `TRACE_START`다. record 지문은 자기
`record_content_hash`와 wall-clock 값을 제외한 내용으로 계산한다. 실행 전체 지문은 tick
순서의 record 지문 목록으로 계산한다.

## 5. 필수 추적 필드

### 5.1 관측과 예측

```text
observation_event
observation_sequence
observation_status
observation_age_s
last_event_was_no_frame
directional_status
prediction_present
release_input_usable
consecutive_ready_frames
last_ready_sequence
```

관측 frame이 없으면 sequence와 age는 `null`로 기록한다. 같은 10Hz frame을 두 번 처리한
20Hz tick에서는 sequence가 같아야 하며 READY 수가 증가하면 안 된다.

### 5.2 gate의 안전 확인

```text
gate_state_before
gate_state_after
stop_epoch_before
stop_epoch_after
gate_consecutive_safe_frames_before
gate_consecutive_safe_frames_after
confirmed_safe_frame_count_before
confirmed_safe_frame_count_after
last_confirmed_safe_sequence
gate_override
gate_failure_reasons
```

`gate_failure_reasons`는 정렬된 문자열 목록으로 기록한다. reason이 없으면 빈 목록을 쓴다.

### 5.3 복구와 재출발

```text
runtime_present_before
runtime_present_after
recovery_reason
release_requested
release_permitted
release_denial_reasons
actual_stop_confirmed
reference_session_id
reference_stop_epoch
resume_authorization_revision
authorization_issue_attempted
authorization_phase_requested
authorization_issue_outcome
authorization_issue_error
temporal_authorization_phase
prior_authorization_hash
```

`authorization_issue_outcome`은 `not_attempted`, `issued`, `rejected` 중 하나다. 발행 함수가
예외를 내면 예외를 숨기지 않고 `rejected`와 정확한 error를 기록한 뒤 현재 진단 결과의 기존
실패 처리로 전달한다.

`recovery_reason`은 최소 다음 중 하나다.

```text
none
prediction_loss
authorization_loss
controller_protective_stop
gate_protective_stop
end_of_world_stop
```

새 이유가 필요하면 enum과 명세를 먼저 갱신한다. 자유문자열로 임의 추가하지 않는다.

### 5.4 local window와 section executor

```text
window_revision
window_first_section
window_last_section
window_source_control_tick
projection_section
projection_distance_m
projection_ambiguous
raw_reference_cursor_m
effective_reference_cursor_m
executor_active_before
executor_active_after
catchup_attempted
catchup_succeeded
catchup_failed_guard
intervening_section_kinds
```

section이 없으면 `null`을 쓴다. catch-up을 시도하지 않았으면 성공 여부와 실패 guard도
`null`이다. `catchup_failed_guard`는 다음처럼 한 가지 정확한 이유를 기록한다.

```text
non_contiguous_section
direction_changed
required_stop_boundary
rotate_or_hold_boundary
projection_ambiguous
projection_too_far
cursor_not_past_section_end
unsupported_section_kind
```

### 5.5 controller 호출 결과

```text
controller_called
controller_status
controller_failure_reason
controller_exception_type
controller_exception_message
controller_active_section
controller_command_before_gate
command_after_gate
controller_result_hash
```

controller가 호출되지 않았으면 status·reason·예외·section·command·hash는 `null`이다.
호출 중 예외가 발생하면 exception type/message를 기록하고 기존 예외 처리를 그대로 수행한다.

## 6. 기록 불변조건

trace 구현은 다음 조건을 만족해야 한다.

1. 기록 on/off가 command, pose, gate 상태와 최종 결과를 바꾸지 않는다.
2. 같은 입력을 두 번 실행하면 wall-clock을 제외한 trace 지문이 같다.
3. 한 tick에서 record가 빠지거나 중복되지 않는다.
4. `gate_state_before`는 이전 tick의 `gate_state_after`와 같다.
5. `stop_epoch`는 줄어들지 않는다.
6. release가 있으면 현재 reference와 authorization의 stop epoch가 gate stop epoch와 같다.
7. CONTINUATION은 gate before/after 중 계약에서 정한 시점의 상태가 `MOVING`일 때만 존재한다.
8. controller가 호출되지 않은 tick에는 controller result를 만들지 않는다.
9. active section이 window 밖이면 catch-up 시도와 정확한 실패 guard를 남긴다.
10. trace에 기대 결과나 evaluator 전용 label이 들어가지 않는다.
11. trace writer 실패를 controller 성공이나 안전 통과로 바꾸지 않는다. 실행 기반시설 실패로
    별도 중단한다.

## 7. 공개 prefix 재현시험

### 7.1 BRAKING continuation

시험 이름:

```text
test_normal_left_seed_1993037174228324916_does_not_continue_while_braking
```

입력:

```text
side = LEFT
profile = Normal
seed = 1993037174228324916
tick_limit = 260
```

수정 전 재현 확인:

```text
tick 259
controller_exception = R5-B continuation authorization is incomplete
gate state = BRAKING
```

향후 수정 합격 기준:

```text
tick 259까지 hard failure 없음
BRAKING/HOLDING 중 CONTINUATION 없음
실제 HOLDING 전에 새 session 없음
stop_epoch 증가 뒤에만 새 stop-bound session 가능
```

### 7.2 active section과 local window

시험 이름:

```text
test_normal_right_seed_4525333994236990214_keeps_active_section_representable
```

입력:

```text
side = RIGHT
profile = Normal
seed = 4525333994236990214
tick_limit = 504
```

수정 전 재현 확인:

```text
tick 503
controller status = section_execution_failed
reason = active_section_not_in_current_window
```

이 단계의 필수 산출물은 tick 503의 다음 값이다.

```text
active section before
window first/last section
projection section/distance/ambiguity
catch-up attempted
catch-up failed guard
intervening section kinds
```

향후 수정 방법은 이 결과를 보기 전에는 선택하지 않는다.

### 7.3 Stress의 잘못된 재출발

시험 이름:

```text
test_stress_left_seed_6422064046178126625_requires_gate_confirmed_frames
```

입력:

```text
side = LEFT
profile = Stress
seed = 6422064046178126625
tick_limit = 533
```

수정 전 재현 확인:

```text
release tick = 531
first motion tick = 532
```

향후 수정 합격 기준:

```text
tick 532까지 release 없음
controller call = 0
first motion = null
release가 있다면 그 직전 gate-confirmed distinct safe frame >= 11
```

단순히 해당 seed에서만 움직이지 않는 것으로 끝내지 않고 `release ⇒ gate-confirmed 11`을
독립 불변조건으로 시험한다.

## 8. Normal 미완료 두 건의 판정 절차

두 건은 먼저 전체 `1600 ticks` trace를 한 번 생성한다. 수정 전 반복 실행은 하지 않는다.

### `normal-right-post-pass-recovery`

다음 사건을 찾는다.

```text
post_pass_proof tick 582
follow_original release tick 620
마지막 정상 MOVING tick
마지막 protective stop 시작 tick
HOLDING 확인 tick
새 session을 만들지 못한 최초 tick
종료 이유
```

HOLDING 뒤 안전 frame 11개가 실제로 확인됐는데도 session이 없으면 복구 코드 결함으로
확정한다. 안전 frame이 부족하면 해당 마지막 HOLD는 정상 fail-closed로 분류한다.

### `normal-left-pre-pass-recovery`

다음 사건을 찾는다.

```text
마지막 release tick 694
마지막 정상 MOVING tick
protective stop 시작 tick
HOLDING 확인 tick
이후 distinct READY와 gate-confirmed safe frame 수
종료 이유
```

통과 전이므로 EMPTY frame이나 단순 Actor 소멸을 재출발 증거로 인정하지 않는다. 안전 근거가
부족했다면 미완료는 정상적인 보수 정지로 남긴다.

## 9. Stage 1 완료 조건

다음을 모두 만족하면 추적·재현 단계가 완료된다.

- trace on/off 결과 동일성 시험 통과
- trace 결정론 시험 통과
- 세 prefix 입력에서 현재 실패 signature 재현
- tick 259의 불법 continuation 호출 순서 확정
- tick 503의 catch-up 실패 guard 확정
- Stress tick 531에서 READY 수와 gate-confirmed 수 차이 확정
- Normal 미완료 두 건의 마지막 HOLD 원인 분류
- 충돌·금지구역·0.08m 여유 기준 변경 없음

Stage 1 결과는 원인 확정 자료일 뿐 수정 완료나 알고리즘 자격을 의미하지 않는다.

## 10. 구현 순서 — 향후 승인용

이 문서는 구현을 승인하지 않는다. 향후 사용자가 구현을 시작하면 다음 순서만 허용한다.

1. trace 자료형과 JSONL writer 추가
2. 기존 동작 불변·결정론 단위시험
3. 세 prefix 공개 재현시험
4. Normal 미완료 두 건 trace 1회 생성과 원인 분류
5. 결과 문서화 후 코드 수정 단계의 별도 승인 요청

전체 회귀는 trace 코드와 표적시험이 동결된 뒤 한 번만 실행한다. C++ timing 500회, 새 hidden,
제품 알고리즘 선택과 `G1~G5`는 이 단계에서 실행하지 않는다.
