# R6 연속 공개 종단 자격 명세

- 상태: 구현·공개 실행 준비
- 작성일: 2026-08-17
- 입력 가정: 카메라 등 상위 영역이 기존 `ActorTrack` 계약을 제공함
- 대상: 최신 R5-B/C C++ DWB 경로 실행 흐름
- 비범위: 실제 카메라·사람 검출, 50ms 자격, hidden, 제품 알고리즘 채택

## 1. 목적

기존 R1~R5 결과를 다시 시작하지 않는다. R6는 최신 R5에서 각각 확인한 경로 실행을 하나의
공개 자격표로 묶고, 각 사례를 중간 checkpoint 없이 처음부터 끝까지 다시 실행한다.

예전 `dynamic-public-qualification`은 현재 R5의 temporal reference, stop-bound session 복구와
통과 후 원 경로 복귀를 사용하지 않는다. 따라서 예전 runner를 R6 증거로 재사용하지 않고,
최신 R5-B/C 공개 함수만 호출하는 별도 R6 runner를 사용한다.

## 2. 고정 공개 사례

| 묶음 | profile | 사례 수 | 요구 결과 |
|---|---|---:|---|
| 같은 방향 Actor 좌·우 | Ideal | 10 | 이탈→추월→재합류 유지→도착 |
| 횡단 Actor 좌·우 | Ideal | 2 | 이탈→횡단 지점 통과→재합류 유지→도착 |
| 두 위험 순차 대응 | Ideal | 1 | 정지→재개→두 번째 정지→새 재개→도착 |
| 횡단 Actor 좌·우 | Normal | 2 | 관측 상실 때마다 실제 정지·새 session 후 통과→원 경로 복귀→도착 |
| 횡단 Actor 좌·우 | Stress | 2 | 판단 근거 부족으로 출발하지 않고 정지 유지 |

총 요구 사례는 `17개`다. Normal 다중 위험은 현재 공개 world 시간 안에 임무를 끝내지 못한
R5-C 제한 진단이므로 이번 R6 완료 사례에 넣지 않는다. 누락을 숨기지 않고 제한사항으로 남긴다.

## 3. 공통 Hard Gate

- 충돌·금지구역 진입·최소 여유 위반 `0`
- stale·invalid·다른 revision·late command 적용 `0`
- 이전 stop epoch의 reference, controller session 또는 이동 허가 재사용 `0`
- 위험 해소·fresh EMPTY·통신 복구만으로 자동 재출발 `0`
- non-finite, 예외, provenance 불일치와 설명되지 않는 비결정성 `0`
- controller 결과와 shared gate의 최종 이동 허가를 같은 것으로 취급하지 않음

## 4. 사례별 기능 Gate

### Ideal 같은 방향·횡단

실제 측면 이탈, Actor 존재 중 통과, 원 reference 진행 순서상 Actor 통과, 원 경로 재합류,
`0.5초` 이상 재합류 유지와 목적지 완료가 모두 있어야 한다.

### Ideal 두 위험

첫 이동 뒤 다른 `stop_epoch`의 두 번째 실제 정지, 두 정지 사이 진행, 새 reference·새 허가,
두 번째 이동과 목적지 완료가 순서대로 있어야 한다.

### Normal 횡단

관측 상실 때 기존 명령·허가·session을 폐기하고 실제 정지한다. 현재 위치와 새 `stop_epoch`에
묶인 새 reference와 새 허가로만 재개한다. 통과 증거를 보존한 뒤 다시 정지하고 원 경로 복귀
reference로 도착해야 한다.

### Stress 횡단

현재 동결 입력에서는 방향 예측이 `READY`가 되지 않는다. 따라서 완료를 강제하지 않는다.
release, controller call과 실제 이동이 모두 `0`이고 `HOLDING`으로 끝나며 hard failure가 없어야
정당한 보수적 종료로 통과한다.

## 5. 실행 규칙

1. 정적 검사와 R5 직접 영향권 시험을 먼저 실행한다.
2. 대표 Stress 정지와 Normal 한쪽 완료를 확인한다.
3. 읽기 전용 결과 감사를 거친 뒤 독립 사례를 process로 병렬 실행한다.
4. 각 worker는 한 사례를 처음부터 끝까지 실행하며 checkpoint 결과를 합치지 않는다.
5. 결과는 고정된 사례 순서로 다시 정렬하고 사례별 trace hash를 기록한다.
6. Python wall-clock은 기록만 하며 R6 기능 합격조건으로 쓰지 않는다.
7. 부분 실행, dirty source, 요구 사례 누락 또는 실패가 있으면 receipt를 만들지 않는다.
8. 결과·source·parameter hash를 봉인하기 직전에 source 변경 여부를 다시 확인한다.

## 6. 산출물

```text
outputs/r6-public-end-to-end-<UTC>-<HEAD>/
  run-manifest.json
  partial-results.json
  case-results.json
  summary.json
  summary.md
  qualification-receipt.json  # 모든 조건 통과 때만
```

## 7. 완료와 다음 Gate

R6 완료는 위 `17개` 공개 사례, hard gate, 전체 회귀, Ruff, source freeze와 결과 hash가 모두
통과하고 receipt가 생성된 경우에만 선언한다. R6가 통과해도 제품 DWB 채택이 아니다.

R7은 별도 사용자 지시 뒤 native 50ms 자격과 hidden 진입 가능 여부를 판단한다. R6에서는
hidden seed 생성·실행과 50ms 판정을 하지 않는다.

