# R5-B 1차 공개 temporal tracking 결과

- 상태: **FAIL — 원인 고정 중, qualification receipt 0**
- 작성일: 2026-08-15
- 범위: R2-A same-direction 공개 PASS 5 episode × 좌·우, Ideal causal 관측, persistent RPP·source-derived DWB
- 비범위: hidden, 실제 초음파 거리·배치 coverage·반사·무응답, 실제 사람, 제품 controller 채택, G1~G5, 경로 분석 7단계

## 1. 결론

현재 R5-B는 통과하지 못했다. 경로가 없어서 실패한 것은 아니다. `2.0 s / tick 40`까지
정지한 현재 상태에서 다시 만든 좌·우 경로 10개는 모두 현재 ground-truth validator를
통과한다. 그러나 R5-A에서 동결한 `0.20 m/s` persistent controller가 첫 공개 temporal
reference를 실제로 추종하면, 목표 Actor가 ground truth에서 사라지는 `30.0 s`까지 완전한
추월을 끝내지 못한다.

안전 기준이나 Actor 반경을 줄이지 않았고, Actor가 사라진 fresh-empty frame을 이전
temporal 허가의 연장으로 해석하지 않았다. 따라서 해당 시점에 실행은 fail-closed로
중단된다.

## 2. 이번에 구현한 범위

1. 추적 R2 ZIP의 크기·SHA-256·entry·world/witness/validation provenance를 엄격히 확인한다.
2. 과거 선택 경로를 단순히 2초 이동하는 잘못된 방식은 거부한다. 이 방식은 10개 모두
   Actor clearance를 위반한다.
3. 동결된 측면 offset 축만 사용해 tick 40 현재 상태에서 최초 안전 경로를 다시 찾는다.
4. 10개 경로를 `GROUND_TRUTH_TEMPORAL` R4 reference로 변환하고 현재 공간 validator로
   다시 검증한다.
5. 관측·prediction·reference·stop epoch·tick을 묶는 일회성 temporal 실행 허가를 발행한다.
6. 관측 warming-up 동안 shared gate가 실제 정지를 확인하고 0 command를 유지한다.
7. 10 Hz 관측 revision을 매 tick의 grid metadata에 연결한다. R5-A의 고정 revision `0`을
   그대로 쓰면 최신 관측을 `prediction_source_mismatch`로 거부하므로, 공간 grid의 동일성은
   유지하면서 현재 observation revision만 전달한다.
8. Ideal causal stream에서 persistent RPP·DWB를 실제 chassis 적분과 shared gate에 연결하는
   공개 실행기를 추가한다.

## 3. 확정된 선행 결과

### 경로·증거

- 입력 archive: `witness-audit-public-20260813-r2-v2-4e4ba0f.zip`
- size: `3,657,108 bytes`
- SHA-256: `50567b093082a57232e668ef89c0316a426cd936496e465b943fe57efa894266`
- 복원한 PASS 증거: `10/10`
- tick 40 causal 재탐색 성공: `10/10`
- 현재 strict ground-truth validation: `10/10 PASS`
- R4 temporal reference 변환·공간 validation: `10/10 PASS`

동결 offset 축에서 선택된 최초 안전 측면 offset은 다음과 같다.

```text
episode 0: left 1.11 m / right 1.11 m
episode 1: left 1.02 m / right 1.02 m
episode 2: left 1.76 m / right 1.76 m
episode 3: left 1.14 m / right 1.14 m
episode 4: left 1.00 m / right 1.00 m
```

### 최초 출발 계약

- tick `0~39`: controller 호출 0, command 0
- 최초 controller 호출: tick `40`
- 최초 실제 이동: tick `44`
- shared gate override: 0
- release 이전 stale·empty·not-ready 이동: 0
- gate stop epoch: `1`

## 4. 첫 공개 RPP 실패 수치

대상은 `v6_primary-00-bd4637cb3cb1 / PASS_LEFT`다.

```text
Actor active interval                         0.0 ~ 30.0 s
마지막 시간 정합 state-after tick             599
그 시점 휠체어 중심의 Actor 대비 진행 차이     +0.18533733027495725 m
완전 추월에 필요한 중심 분리                  0.40000000000000000 m
부족한 중심 분리                              0.21466266972504275 m
최대 원 경로 이탈                             1.1115848601134344 m
최소 실제 Actor clearance                    0.644 m
최소 static/forbidden clearance              0.31577553509462813 m
Actor disappearance가 관측에 도달한 tick      604
```

휠체어 중심이 Actor보다 앞서기는 했지만 두 footprint의 종방향 범위가 아직 겹친다. 따라서
이를 추월 완료로 세지 않는다. tick 604의 fresh-empty frame에서는 directional prediction이
`EMPTY_FRAME`으로 전환되며, R5-B 명세가 요구하는 target-bound `READY` 허가를 더 이상
발행하지 않는다.

### 공개 10개 RPP 확인

동일 실행기로 same-direction 공개 5 episode의 좌·우를 모두 `tick 610`까지 확인했다.
10개 모두 tick `115`에 원 경로에서 이탈했고 shared gate override는 `0`이었지만,
Actor가 마지막으로 존재하는 tick `599`에 필요한 `+0.400m` 중심 분리를 만들지 못했다.

| episode | LEFT 진행 차이 | RIGHT 진행 차이 | 완전 추월 |
|---:|---:|---:|---|
| 0 | `+0.18533733027495725m` | `+0.18533733026963840m` | 실패 |
| 1 | `+0.08567801359725902m` | `+0.08567801360220617m` | 실패 |
| 2 | `-0.62075261854577150m` | `-0.62075261855756160m` | 실패 |
| 3 | `+0.18299319060445507m` | `+0.18299319060463537m` | 실패 |
| 4 | `+0.06461878291852896m` | `+0.06461878292450240m` | 실패 |

따라서 첫 사례나 LEFT 방향만의 우연한 실패는 아니다. 양쪽 대칭 실행도 같은 판정으로
닫혔다. 이 수치는 qualification receipt가 아니라 현재 실패를 재현하는 공개 회귀값이다.

## 5. DWB 상태

DWB도 tick `40`에 최초 호출되고 tick `44`에 실제 움직임을 시작했다. 첫 공개 LEFT 사례를
같은 `tick 610` 경계까지 끝까지 실행한 결과는 다음과 같다.

```text
실행 wall-clock                               3,074.4671572 s
controller call                              564
마지막 시간 정합 state-after tick             599
그 시점 휠체어 중심의 Actor 대비 진행 차이     -0.08294597353371147 m
완전 추월에 필요한 중심 분리                  +0.40000000000000000 m
최대 원 경로 이탈                             1.1052723922549408 m
최소 실제 Actor clearance                    0.6471500000000001 m
최소 static/forbidden clearance              0.31577553509462813 m
shared gate override                         0
trace hash                                   e46737ee4214bd15f36371f8d0e158d24b64cf57c9423b5551057e285f152d22
```

DWB도 안전 위반 없이 옆 경로에 진입했지만 Actor보다 완전히 앞서지 못했고, tick `604`의
fresh-empty에서 fail-closed했다. 따라서 첫 사례에서는 RPP와 DWB가 같은 기능 판정으로
닫혔다. Python source-derived DWB의 51분 wall-clock은 운영 병목으로 별도 기록하되 이번
lane의 기능 실패 근거로 사용하지 않으며, 50 ms qualification도 적용하지 않는다. 이 장시간
실행을 일반 회귀에 반복 포함하지 않는다.

### C++ 안전 배치 가속 재실행

후속 구현에서 후보 `217개`, 후보당 `41 pose`, rollout·terminal stopping, 안전 수치,
critic·tie-break와 최종 shared gate를 유지한 채 후보별 동적 안전 판정만 C++20 배치
코어로 옮겼다. 같은 첫 LEFT 610틱은 `109.597374s`에 끝났고, 최종 pose·clearance·
실패 이유와 trace hash가 위 순수 Python 결과와 정확히 일치했다. 전체 사례 wall-clock은
약 `28.05배` 줄었지만 추월·재합류가 생기지는 않았다. 따라서 이는 기능 수정이나 DWB
채택 근거가 아니라 동작 보존형 계산 가속이다. 상세 내용은
[R5-B C++ DWB 안전 배치 가속 결과](r5b-cpp-dwb-safety-acceleration-result-2026-08-15.md)에
기록한다.

## 6. 판정

현재 상태에서 다음을 주장할 수 있다.

- `2.0 s` causal release 뒤에도 안전한 ground-truth 경로는 존재한다.
- temporal reference·현재 관측·prediction·stop epoch·tick을 묶는 실행 경계가 작동한다.
- RPP와 DWB는 release 이전에 움직이지 않고 정확한 시점에 동일 입력으로 시작한다.
- 첫 RPP는 경로를 따라 실제로 이탈했으며 static/Actor clearance를 위반하지 않았다.
- 첫 DWB도 경로를 따라 실제로 이탈했으며 static/Actor clearance를 위반하지 않았다.

다음은 아직 주장할 수 없다.

- Actor가 존재하는 동안의 완전 추월
- ordered `departure → overtake → sustained rejoin → terminal stop`
- R5-B 통과 또는 qualification receipt
- RPP 또는 DWB의 R5-B 기능 성공
- 실제 perception 또는 사람 안전

## 7. 다음 허용 작업

실패를 없애기 위해 안전 margin, Actor 반경, 20-frame history, latency, shared gate를 줄이지
않는다. 다음 비교는 별도 공개 연구 변경으로만 수행한다.

1. 현재 `0.20 m/s` controller와 `0.30 m/s` witness timing의 불일치를 명시적으로 해결한다.
2. 시간 목표를 controller 입력에 숨겨 넣지 않고, speed profile을 reference 계약으로 만들지
   또는 현재 controller 속도에 맞는 causal witness를 다시 합성할지 비교한다.
3. Actor active interval을 사후 연장하거나 empty frame을 자동 허가로 바꾸지 않는다.
4. 공개 기능시험이 통과하기 전 hidden, 제품 채택, R5-C/R6/R7로 진행하지 않는다.

## 8. 검증

- R5-B·직접 영향권: `95 passed`
- 전체 회귀: `884 passed, 3 skipped`
- skip 3건: 선택적 C++ DWA core 미빌드
- Ruff: 통과
- `compileall`: 통과
- `git diff --check`: 통과

전체 회귀는 테스트 파일 `73`개를 `19/18/18/18`로 분할해 4개 독립 process로 실행했다.
할당 수와 unique 수는 모두 `73`이었다. 새 51분 DWB 전체 사례는 반복 suite에 넣지 않고,
짧은 release 경계와 RPP 실패 회귀만 자동시험으로 보존했다.
