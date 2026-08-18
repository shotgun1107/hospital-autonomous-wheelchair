# R7 완료 연장 구간 Actor 삭제 정정

## 판정

R7 완료 진단은 원래 `39 s`인 공개 world를 `80 s`까지 실행하면서, 원본 경계 뒤의
Actor를 빈 목록으로 바꾸고 있었다. 이는 사람의 운동 결과가 아니라 진단 하네스가 만든
인공적인 `EMPTY` 관측이다.

이번 정정은 제품 제어기에 "사람이 사라지면 재개" 규칙을 추가하지 않는다. 원본 world와
그 content hash도 변경하지 않는다. 완료 진단의 추가 관측 구간에서만 다음을 적용한다.

- 원본 world 종료 시각까지 active인 constant-velocity Actor는 같은 속도로 계속 이동한다.
- 원본 종료 전에 의도적으로 끝난 Actor는 다시 생성하지 않는다.
- 관측 상태가 현재 제어 단계에서 사용할 수 없으면 기존처럼 제한 제동·정지를 수행한다.
- clearance, 후보 수, 관측 잡음, dropout, TTL과 재출발 확인 수는 변경하지 않는다.

## 기존 문제

횡단 Actor의 동결 운동은 다음과 같다.

```text
start y = -2.388 m
velocity y = 0.428 m/s
active interval = 0..39 s
position at 39 s = 14.304 m
```

Actor는 39초에 이미 복도와 지도에서 멀리 벗어나 있다. 그런데 완료 진단은 39초 이후
Actor를 계속 외삽하지 않고 `actors=()`로 바꿨다. Normal 관측 지연이 반영된 tick 784에서
세 사례가 갑자기 `EMPTY_FRAME`을 받았고, 수정 전 코드는 이를 제어기로 넘겨 예외를 냈다.

P0 예외 처리는 별도 커밋 `ae1721c`에서 fail-closed 제동으로 고쳤다. 이번 정정은 그
방어코드를 제거하지 않고, 원인이었던 인공적인 Actor 삭제도 함께 제거한다.

## 구현 경계

`r5c_observation_diagnostic.py`의 완료 연장 lane은 원본 `WitnessWorldSnapshot`을 `replace`하지
않는다. world의 source projection hash는 원본 39초 corpus에 결박돼 있기 때문이다.

대신 추가 관측 시각에 다음처럼 상태만 계산한다.

```text
if t <= source world duration:
    source world actor_states_at(t)
elif completion extension AND actor active at source boundary:
    start_position + velocity * (t - active_from)
else:
    no Actor
```

같은 상태 provider를 합성 관측과 ground-truth clearance 기록에 모두 사용한다.

## 검증 결과

- 새 경계시험:
  - `39 s` 이후에도 Actor identity·velocity가 유지된다.
  - tick 784가 인공적인 `EMPTY_FRAME`이 아니다.
- 직접 영향권:
  - `test_r7_failure_public_regression.py`: `9 passed`
  - `test_r5c_observation_diagnostic.py`: `12 passed`
  - Ruff, compileall, `git diff --check`: 통과
- 소비된 hidden-v2 Normal 실패 6건의 읽기 전용 회귀 재생:
  - 3건 완료
  - 3건 안전정지
  - 예외 0건
  - 최소 Actor clearance `0.5549 m` 이상

이 결과는 기존 hidden-v2의 판정을 PASS로 바꾸지 않는다. 해당 hidden은 이미 소비됐고
regression 자료일 뿐이다.

## 남은 문제

남은 Normal 3건은 Actor 삭제 문제가 아니라 5% 독립 frame dropout마다 제동하고, 재출발 전
서로 다른 안전 frame을 다시 확인하는 과정 때문에 80초 안에 완료하지 못했다. Stress에서
간헐적으로 `READY`가 발생한 문제도 이번 변경 범위가 아니다.

다음 단계에서는 단일 dropout과 `300 ms` TTL의 관계를 명세에서 먼저 다시 검토해야 한다.
실제 카메라·FOV·가림 모델이나 제품 재출발 정책의 증거로 확대하지 않는다.
