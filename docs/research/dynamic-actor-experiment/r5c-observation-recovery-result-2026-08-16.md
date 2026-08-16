# R5-C 공개 관측 복구 제한 결과

## 1. 판정

- 상태: 제한 복구 진단 완료
- 범위: Actor가 `t=0`부터 존재하는 공개 다중 위험 장면
- Normal 결과: 관측 상실 뒤 정지·새 session 재출발 반복 확인, 장면 종료 시 정지 유지
- Stress 결과: 연속 READY 부족으로 출발하지 않음
- hard failure: `0`
- 임무 완료: `0`
- R2-B Actor 내부 출현 문제: 미해결 유지

이번 결과는 이미 추적하던 Actor의 관측이 끊겼다가 회복되는 경우만 다룬다. 관측 전에 지도
내부에 새 Actor가 생기는 기존 R2-B 실패 2건, 실제 카메라·FOV·가림·검출은 다루지 않았다.
따라서 formal R5-C/R6 자격이나 제품 안전 결과가 아니다.

## 2. Normal 복구 결과

동결 Normal profile과 공개 다중 위험 장면을 `700` control tick까지 실행했다.

| 항목 | 결과 |
|---|---|
| 첫 계획 release | tick `44` |
| 실제 session release | `82, 138, 220, 356, 424, 618, 650` |
| 방향 예측 상실 | `100, 184, 306, 384, 494, 624, 666` |
| 실제 정지 확인 | `111, 198, 320, 398, 508, 629, 675` |
| session이 사용한 stop epoch | `1, 2, 3, 4, 5, 6, 7` |
| 최종 stop epoch | `8` |
| controller session | `7` |
| controller call | `270` |
| 최소 Actor 여유 | `0.5290m` |
| 최소 정적 여유 | `0.3800m` |
| 최종 pose | `(3.875336, 0.620000, 0.0000)` |
| 최종 상태 | `HOLDING` |
| hard failure | `0` |

각 관측 상실에서 다음 순서가 지켜졌다.

```text
prediction loss
< 실제 정지 확인
< 다음 새 session release
```

이전 controller, 이전 reference session과 이전 resume authorization은 재사용하지 않았다. 실제
정지 뒤 새 stop epoch에 맞는 현재 pose 기반 `FOLLOW_ORIGINAL` reference와 새 controller를
생성했다. 마지막 일곱 번째 누락은 tick `675`에 정지를 확인한 뒤 공개 world의 `35s` 종료가
먼저 와서 새 session을 만들지 않았다.

반복 정지 때문에 목표까지 갈 시간이 부족했으므로 결과는 `COMPLETED`가 아니라
`CONSERVATIVE_HOLD`다. 이를 기능 PASS로 승격하지 않는다.

## 3. Stress 결과

Stress에서는 전체 `700` tick 중 READY 상태가 합계 `32` tick 있었지만, 고유 fresh READY
frame은 최대 `10`개까지만 연속됐다. 재출발에 필요한 `11`개를 채우지 못했으므로:

- release `0`
- controller call `0`
- 비영점 이동 `0`
- 최종 `stop_epoch=1`, `HOLDING`
- hard failure `0`

일시적인 READY 한 번이나 fresh `EMPTY`를 이동 허가로 사용하지 않았다.

## 4. R2-B 재현 경계

v6 `second-risk-after-corner` 공개 장면을 별도 단위시험으로 고정했다.

```text
t=12.9s: 두 번째 Actor 없음
t=13.0s: 두 번째 Actor 실제 존재
t=13.0s에 도착한 최신 Ideal frame: observed_at=12.9s, EMPTY
```

즉 fresh `EMPTY`는 미래에 Actor가 생기지 않는다는 보장이 아니다. 이 시험은 기존 실패를
없애는 시험이 아니라 false-safe 조건을 다시 놓치지 않기 위한 회귀 경계다.

## 5. 검증

- R2-B 출현 경계: `2 passed`
- R5-C 전체 전용시험: `8 passed`
- 결합 R5-C·R2-B 경계: `10 passed in 128.61s`
- 관측·방향예측·shared gate: `66 passed`
- 기존 R5-B 재정지·시간 허가·pipeline: `28 passed in 400.31s`
- 영향권 합계: `104 passed`, 실패 `0`
- 전체 실험실 회귀: 4개 process shard 합계 `926 passed`, 실패·건너뜀 `0`
  - shard 결과: `165 / 232 / 260 / 269 passed`
- Ruff·compileall·`git diff --check`: 통과

## 6. 남은 일

1. 횡단 PASS 중 관측 상실 뒤 사용할 새 시간·공간 reference 계약을 별도로 설계한다.
2. R2-B를 재개할 때 Actor 진입·가시영역·빈 공간 확인 증거를 먼저 정한다.
3. 새 entry/visibility 계약 없이 기존 hard failure를 삭제하거나 완화하지 않는다.
4. hidden, formal receipt, 제품 알고리즘 채택은 시작하지 않는다.
