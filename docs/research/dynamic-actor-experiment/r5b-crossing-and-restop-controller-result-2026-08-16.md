# R5-B 횡단·다중 위험 C++ DWB 실행 결과

- 날짜: 2026-08-16
- 판정: 공개 Ideal 경로 기능 통과
- 범위: 횡단 Actor 좌·우, 두 위험의 정지→재개→재정지→재개→도착
- 비범위: Normal·Stress, 실제 perception, hidden, 제품 알고리즘 채택

## 결론

C++ DWB는 공개 합성 Ideal 장면에서 다음 두 미완료 항목을 모두 끝냈다.

1. 옆에서 가로지르는 Actor를 좌·우로 우회하고 원 경로에 돌아와 목적지에 도착했다.
2. 첫 위험 뒤 출발하고 두 번째 위험에서 다른 정지 번호로 다시 멈춘 뒤, 새 허가와 새
   reference session으로 재출발해 목적지에 도착했다.

안전거리 `0.08m`, Actor 반경, 후보 `217개`, 후보당 `41 pose`, 2초 rollout, terminal stopping,
방향성 Capsule과 shared gate는 완화하지 않았다.

## 구현 중 확인한 문제와 수정

### 횡단

- 곡선 구간의 끝점을 정확히 밟지 않고 다음 연속 구간으로 안전하게 진입하면 현재 구간 번호가
  뒤처졌다.
- 같은 진행방향의 바로 다음 구간, 경로 오차 `0.05m` 이내, 끝점 진행량 통과, 정지·회전 경계
  없음일 때만 한 구간 따라잡도록 했다.
- 최종 위치에 도착했지만 yaw가 남은 경우 공통 executor가 실제 정지 확인 뒤 제자리 정렬하고
  terminal dwell을 수행하도록 했다.
- 마지막 직선의 GoalAlign이 5cm 부근에서 영점 명령을 고르던 문제는 남은 구간이 모두
  무이동 종단 구간일 때만 기존 near-goal 비활성을 적용했다.

### 다중 위험

- 첫 시도는 두 번째 Actor가 휠체어보다 너무 일찍 지나가 재정지가 발생하지 않았다.
- 두 번째 위험만 `7.0s` 늦춰 첫 출발 뒤 휠체어가 실제로 접근하는 시각에 맞췄다.
- 각 보호정지마다 현재 pose→기존 목적지의 `FOLLOW_ORIGINAL` reference를 새 stop epoch와 새
  session ID로 다시 발급했다. 이전 허가는 재사용하지 않았다.

## 결과

| 사례 | 주요 tick | 최소 Actor 여유 | gate override | hard failure | 완료 |
|---|---|---:|---:|---:|---|
| 횡단 왼쪽 | release 80, move 81, pass 370, rejoin 610, goal 625 | 0.08m 이상 | 0 | 0 | PASS |
| 횡단 오른쪽 | release 80, move 81, pass 295, rejoin 541, goal 556 | 0.08m 이상 | 0 | 0 | PASS |
| 다중 위험 | release 44, restop 232(epoch 2), release 264, goal 490 | 0.1002m | 0 | 0 | PASS |

다중 위험의 두 정지 사이 진행거리는 `1.7283m`이고 서로 다른 controller/reference session
`2개`가 사용됐다. 하나의 긴 정지를 두 번 센 결과가 아니다.

## 검증

- 새 다중 위험 전용시험: `3 passed`
- 공통 reference/executor/DWB 빠른 영향권: `79 passed`
- 전체 실험실 회귀: `916 passed`, 실패·건너뜀 `0`, `1357.18s`
- Ruff와 `git diff --check`: 통과

## 증거 파일

- `r5b_crossing_evidence.py`
- `r5b_restop_execution.py`
- `r5b_temporal_reference.py`
- `r5b_temporal_execution.py`
- `reference_section_executor.py`
- `tests/test_r5b_crossing_evidence.py`
- `tests/test_r5b_restop_execution.py`

## 한계

이는 합성 Ideal 관측과 공개 장면의 경로 기능 결과다. 실제 초음파 센서가 지도 밖 Actor를 미리
관측할 수 있다는 증거가 아니며, 보류 중인 R2-B 출현 관측 문제를 닫지 않는다. 실제 사람·환자
탑승 안전, 실물 휠체어, 50ms 실시간성, Normal·Stress, hidden과 제품 알고리즘 결정도 별도다.
