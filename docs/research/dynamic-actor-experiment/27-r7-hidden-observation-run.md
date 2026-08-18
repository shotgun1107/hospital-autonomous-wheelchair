# R7 후속 비공개 관측 시험

- 상태: 실행 전 기준 고정
- 사용자 승인: `2026-08-18`
- 전제: R6 공개 `17/17`, R7 동일성 `5/5`, 50ms 초과 `0/500`
- 범위: 합성 `ActorTrack` 관측 순서만 비공개로 바꾸는 simulation 연구
- 비범위: 새 지도·새 사람 운동, 실제 카메라, 제품 알고리즘 채택, `G1~G5`

## 1. 질문

공개 시험에서 사용하지 않은 새로운 관측 잡음·frame dropout 순서를 주었을 때도:

- Normal에서 좌·우 횟단 우회를 완료하는가?
- Stress에서 판단 근거가 부족하면 출발하지 않는가?
- 두 경우 모두 충돌·금지구역·잘못된 재출발이 없는가?

이 시험은 기존 좌·우 횟단 지도와 Actor 운동을 그대로 쓴다. 따라서 “모든 지도와
사람 움직임을 검증했다”는 시험이 아니다.

## 2. 시험 수

```text
5개의 새 관측 순서
× LEFT / RIGHT
× Normal / Stress
= 20 cases
```

같은 replica·side의 Normal과 Stress는 같은 기본 난수를 쓴다. 두 profile의 차이는 위치·속도
오차와 dropout 비율이지, 운 좋은 순서를 각각 고르는 것이 아니다.

## 3. 시험 값을 미리 보지 않는 방법

1. 이 문서·실행기·판정 코드를 먼저 commit한다.
2. 작업트리가 clean인 상태에서 OS 난수로 root seed를 한 번 만든다.
3. 실행 전에는 root seed를 화면·manifest·Git에 기록하지 않고 SHA-256 commitment만 기록한다.
4. 실행이 시작되면 이 seed는 소비된다. 결과를 본 뒤 코드를 바꾸면 동일 seed를 최종
   비공개 근거로 다시 쓰지 않는다.
5. 실행 중 중단되면 partial을 보존하고 최종 결과로 쓰지 않는다.

seed commitment:

```text
SHA256("r7-hidden-observation-v1:" + decimal_root_seed)
```

각 replica·side의 실제 관측 seed는 root seed에서 SHA-256로 결정론적으로 만든다. 생성 순서나
worker 완료 순서가 결과를 바꾸지 않는다.

## 4. 고정하는 것

다음은 R6·R7에서 쓴 값을 그대로 유지한다.

- 좌·우 횟단 world, reference, Actor trajectory
- Normal·Stress profile 수치와 10Hz observation / 20Hz control
- 후보 `217개`, 후보당 `41 pose`, `2s` rollout, terminal stopping
- Actor prediction, wheelchair footprint, `0.08m` 여유, forbidden 판정
- critic·점수·tie-break와 external shared gate
- Normal·Stress episode `1600 ticks`
- 출발 전 11개의 서로 다른 안전 frame과 현재 `stop_epoch` 허가

속도 측정은 이 시험의 합격 기준이 아니다. R7의 500회 시간 자격을 재사용하고 돌리지
않는다.

## 5. 합격 기준

### 모든 20 case

- `hard_failures == ()`
- 충돌, 금지구역 진입, `0.08m` 여유 위반, 늦거나 잘못된 명령 적용, 허가 없는
  재출발이 모두 `0`

### Normal 10 case

모두 다음을 만족해야 한다.

- outcome `completed`
- 통과 증거와 원 경로 복귀 허가가 있음
- 첫 실제 이동과 목표 도착 tick이 있음
- 최종 상태 `COMPLETED`

### Stress 10 case

모두 다음을 만족해야 한다.

- outcome `conservative_hold`
- release tick 없음
- controller call `0`
- 첫 실제 이동 없음
- 최종 상태 `HOLDING`

한 건이라도 다르면 전체 판정은 FAIL이다. 실패를 본 뒤 profile·안전 값·episode 길이·
평가 기준을 낮추지 않는다.

## 6. 실행 전 확인

runner는 다음이 맞지 않으면 seed를 만들지 않고 중단한다.

- Git working tree clean
- R7 증거 ZIP 크기·SHA-256 일치
- R7 receipt의 `0/500`, Python↔C++ `5/5`, hidden 미실행 표시
- R7이 고정한 native/controller 파일 hash 일치
- C++ full core·safety core 사용 가능
- 공개 default seed 동작이 기존과 같음

## 7. 실행과 보존

독립 case는 process로 병렬 실행하고 최종 결과는 ordinal 순서로 합친다. 같은 replica·side의
Normal·Stress는 같은 derived seed를 쓴다. worker 수는 결과 의미에 포함하지 않는다.

```text
outputs/r7-hidden-observation-<UTC>-<HEAD>/
  pre-run-manifest.json
  consumed-seed.json
  case-results.json
  summary.json
  summary.md
  hidden-consumption-receipt.json
```

- `consumed-seed.json`은 실행이 시작된 뒤의 정확한 회귀 보존용이다.
- 산출물은 기존 경로를 덮어쓰지 않는다.
- 모든 case가 완주한 뒤에만 consumption receipt를 쓴다.
- worker crash·timeout·사용자 중단은 기능 실패로 바꾸지 않는다.

## 8. 결과 해석

- PASS: 이 20개 합성 관측 순서에서 현재 연구 기준선이 유지됐다.
- FAIL: 해당 입력은 회귀 자료로 바꾸고 현재 연구 기준선의 한계로 기록한다.
- 어느 결과도 실제 카메라·실물 휠체어·사람 탑승 안전과 제품 알고리즘 채택을 의미하지 않는다.
