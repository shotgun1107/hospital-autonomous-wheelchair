# R5-C 횡단 경로 복구 제한 결과

## 결론

카메라 등 상위 관측 영역이 기존 `ActorTrack` 계약을 제공한다고 둔 공개 Normal 좌·우 장면에서,
관측 상실 뒤 실제 정지와 새 stop-bound reference/controller session을 반복하는 흐름이
안전 경계를 통과했다.

이번 결과는 횡단 경로의 **중단 후 복구가 가능함**을 확인한 것이다. 반복 관측 상실이 있는
Normal 장면에서 항상 제한 시간 안에 도착한다고 확정한 결과는 아니며, world 종료 전에는
새 출발을 막고 실제 정지로 닫는다.

## 수정한 문제

1. 새 세션의 local window는 현재 위치의 뒤쪽 section부터 열리지만 executor가 항상 0번
   section에서 시작해 실패하던 문제를 고쳤다.
2. 카메라 잡음에 해당하는 합성 관측 변화로 추월 완료 판정이 되돌아갈 때 실행 오류로 끝내지
   않고 보호정지와 새 세션으로 연결했다.
3. 장면 종료 시 움직이는 상태로 끝나지 않도록 마지막 20 tick을 실제 정지 확인에 남겼다.

경로 형상, 안전거리 `0.08m`, Actor 예측관, 후보 수, C++ DWB와 shared gate는 완화하지 않았다.

## 검증

- reference 재결박·공통 executor 영향권: `22 passed`
- Normal 횡단 복구 왼쪽: `1 passed in 70.48s`
- Normal 횡단 복구 오른쪽: `1 passed in 67.86s`
- Ruff: 통과

긴 전체 회귀, hidden, formal receipt와 실제 카메라 시험은 이번 범위에서 실행하지 않았다.

