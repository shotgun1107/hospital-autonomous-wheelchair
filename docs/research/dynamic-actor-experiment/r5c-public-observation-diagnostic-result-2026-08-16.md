# R5-C 공개 Normal·Stress 제한 진단 결과

- 실행일: 2026-08-16
- 대상: 횡단 LEFT·RIGHT, 다중 위험 재정지 공개 장면
- 상태: 제한 진단 완료, 정식 R5-C 자격 미발급
- 기준: [`19-r5c-public-observation-diagnostic.md`](19-r5c-public-observation-diagnostic.md)

## 1. 결론

현재 관측 통합은 안전하게 멈추지만 임무를 이어가지는 못한다.

- Normal 횡단 좌·우는 tick `80`에 허가되고 tick `81`에 움직였다. 두 방향 모두 첫 frame
  누락이 반영된 tick `128`에 새 이동 명령을 중단했고 tick `133`에 실제 정지했다.
- Normal 다중 위험은 계획 tick `44`에 바로 출발하지 않았다. 연속 안전 frame `11`개를
  다시 확보한 tick `68`에 허가되고 tick `69`에 움직였지만, tick `70`의 입력 상실 뒤
  tick `73`에 정지했다.
- Stress는 횡단·다중 위험 모두 방향 예측 `READY`가 한 번도 나오지 않았다. controller call과
  실제 움직임은 `0`이고 gate는 초기 `stop_epoch=1` 정지를 유지했다.
- 다섯 실행 모두 Actor·정적 최소 여유 `0.08m`를 지켰고 hard failure는 `0`이다.
- 어느 실행도 목표까지 완료하지 못했다. 따라서 Normal·Stress 기능 PASS, R2-B 해결 또는
  정식 R5-C 자격을 주장할 수 없다.

## 2. 실행 결과

| 장면 | profile | 계획/실제 허가 | 첫 움직임 | 입력 상실 | 실제 정지 | controller call | 최소 Actor 여유 | 결과 |
|---|---|---:|---:|---:|---:|---:|---:|---|
| 횡단 LEFT | Normal | 80 / 80 | 81 | 128 | 133 | 48 | 1.8133m | 보수적 정지 |
| 횡단 RIGHT | Normal | 80 / 80 | 81 | 128 | 133 | 48 | 1.6643m | 보수적 정지 |
| 다중 위험 | Normal | 44 / 68 | 69 | 70 | 73 | 2 | 0.5290m | 보수적 정지 |
| 횡단 LEFT | Stress | 80 / 없음 | 없음 | 해당 없음 | 초기 정지 유지 | 0 | 0.7900m | 출발 보류 |
| 다중 위험 | Stress | 44 / 없음 | 없음 | 해당 없음 | 초기 정지 유지 | 0 | 0.5290m | 출발 보류 |

Normal 횡단의 최소 정적 여유는 LEFT `0.3793m`, RIGHT `0.3786m`이고, 다중 위험은
`0.3800m`다. 횡단 정지 과정의 gate override `2`는 controller의 이전 비영점 명령 대신
제한 감속 명령을 적용한 횟수다. 안전 위반이 아니다. 다중 위험과 Stress의 override는 `0`이다.

## 3. 관측 상태

| 장면 | profile | READY tick | warming-up | dropout/stale tick | no-frame tick |
|---|---|---:|---:|---:|---:|
| 횡단 좌·우 공통 | Normal | 88 | 38 | dropout 6 | 6 |
| 다중 위험 | Normal | 22 | 38 | dropout 12 | 12 |
| 횡단 | Stress | 0 | 609 | stale 166 | 166 |
| 다중 위험 | Stress | 0 | 547 | stale 148 | 148 |

Normal은 방향 판단이 가능해도 누락 frame이 오면 이전 방향 예측과 허가를 재사용하지 않는다.
Stress는 잡음과 누락 아래 20-frame 방향 신뢰 조건을 만족하지 못했으며, 성공률을 높이기 위해
신뢰 기준·안전거리·TTL을 낮추지 않았다.

## 4. 구현 경계

추가한 전용 실행기는 다음만 수행한다.

1. 공개 world의 ground truth로 Normal·Stress 합성 관측을 만든다.
2. controller에는 검증된 관측과 방향 예측만 전달한다.
3. 현재 accepted·fresh `READY`, 실제 정지, 연속 안전 frame `11`, 현재 stop epoch용 새 허가가
   함께 있을 때만 출발을 요청한다.
4. 이동 중 방향 예측이 없어지면 원형 fallback 예측으로 계속 이동하지 않고 prediction을
   `None`으로 gate에 전달해 감속·정지를 강제한다.
5. 입력 상실 뒤 자동 재출발이나 새 reference session은 만들지 않는다.

마지막 규칙 때문에 이번 결과는 recovery 구현 결과가 아니다. 다음 기능 후보는 실제 정지 완료
뒤 새 fresh `READY` 구간, 새 stop epoch reference와 새 허가를 결합해 임무를 재개하는 것이다.
다만 이 작업은 R2-B hard failure와 정식 R5-C 진입 조건을 함께 다시 검토한 뒤 별도로 승인해야
한다.

## 5. 검증

- R5-C 표적시험: `6 passed in 45.71s`
- 관측·방향예측·안전 gate 계약: `66 passed in 0.90s`
- 기존 Ideal 횡단·다중 위험: `8 passed in 315.61s`
- 전체 회귀: 4개 process shard 합계 `922 passed`, 실패·건너뜀 `0`
  (`176 + 228 + 262 + 256`, 최장 shard `743.30s`)
- Ruff·compile·`git diff --check`: 통과

## 6. 말할 수 없는 것

- R2-B Actor 출현/fresh EMPTY hard failure 해결
- 카메라·FOV·가림·검출·추적 성능
- Normal·Stress 임무 완료
- formal R5-C/R6 receipt
- hidden 결과
- 실제 사람·환자 탑승 안전 또는 제품 DWB 채택
