# ADR 0012: Sliding Subpath 갱신에서 Persistent Controller Session 유지

- 상태: 사용자 개인 연구 방향, 팀 합의 전
- 날짜: `2026-08-14`
- 범위: 동적 지역 기동 연구 R4·R5, Python `simulation_only`

## 배경

R3는 정적 공간에서 통과·회전·재합류 가능한 전체 pose·heading path를 반환한다. R4는 이
경로를 controller가 소비할 local reference와 sliding window로 바꾼다.

매 window 갱신마다 controller instance를 다시 만들면 다음 문제가 생긴다.

- oscillation·goal phase·progress와 같은 controller 내부 상태가 사라진다.
- 동일 경로인데도 초기화 횟수에 따라 명령과 결과가 달라진다.
- RPP와 DWB의 상태 수명이 달라 paired 비교가 오염된다.
- waypoint 단위 성공을 이어 붙인 결과를 연속 closed-loop 성공으로 오인할 수 있다.
- 이전 window·session의 늦은 결과를 현재 command와 구분하기 어렵다.

반대로 모든 변경을 같은 session에 넣으면 전체 path가 교체됐는데도 이전 controller 상태를
잘못 재사용할 수 있다. 따라서 full reference 교체와 같은 reference 안의 window 이동을
revision으로 구분해야 한다.

## 결정

다음 identity를 분리한다.

```text
maneuver_revision
path_revision
subgoal_revision
reference_session_id
full_reference_hash
window_content_hash
```

- 새 active maneuver 의미 또는 stop epoch가 바뀌면 `maneuver_revision`을 올린다.
- full knots·sections·rejoin geometry가 바뀌면 `path_revision`을 올린다.
- 같은 full path에서 controller가 보는 contiguous window만 바뀌면 `subgoal_revision`만
  올린다.
- 동일 maneuver/path의 window 갱신에서는 `reference_session_id`와 controller instance를
  유지한다.
- 새 maneuver/path를 수용하면 새 session과 controller state를 시작한다.
- 같은 revision에 다른 content hash가 오면 invalid로 거부한다.
- 이전 session 또는 이전 revision의 늦은 command/result는 적용하지 않는다.
- rotation section은 atomic하게 window에 포함해 window 경계가 회전 동작을 절단하지 않게
  한다.
- 보호정지로 `stop_epoch`가 바뀌면 기존 reference/session을 재출발 근거로 재사용하지 않는다.

R4 candidate set의 LEFT·RIGHT·WAIT는 같은 generation revision에 함께 존재할 수 있다. 한 후보를
active binding한 뒤 sibling 후보로 전환하려면 같은 revision에서 즉시 바꾸지 않고 새 maneuver
revision과 session으로 발행한다.

## 이유

이 결정은 다음 두 요구를 동시에 만족한다.

```text
같은 경로의 local window 이동
→ controller state 보존

기동 의미·전체 경로·stop epoch 교체
→ 이전 state와 늦은 결과 폐기
```

따라서 R5에서 RPP와 DWB를 동일한 session 수명으로 비교할 수 있고, sliding window 구현 세부가
알고리즘 비교 결과를 임의로 바꾸는 것을 줄인다.

## 고려한 대안

### 매 window마다 controller 재생성

구현은 단순하지만 상태가 계속 초기화되어 persistent controller 비교가 아니다. 기존 집 PC
기능 탐색에서 waypoint마다 새 DWB를 만든 결과도 연속 실행 증거로 인정하지 않았다.

### 모든 path 교체를 한 session에서 처리

이전 oscillation·goal·progress 상태가 새 geometry에 남을 수 있고 stale 결과 폐기가 어렵다.

### full path만 controller에 항상 전달

revision 문제 일부는 줄지만 local planner 입력 크기와 현재 subgoal 의미가 불명확하며, R4의
sliding subpath 연구 질문을 검증하지 못한다.

## 결과와 비용

장점:

- 동일 path의 controller 상태 수명을 보존한다.
- path 교체와 window 이동을 로그·시험에서 구분한다.
- stale command/result를 revision과 hash로 fail-closed할 수 있다.
- RPP·DWB paired 비교의 session 조건을 동일화한다.

비용:

- window manager가 cursor·revision·session 상태를 보존해야 한다.
- same-tick duplicate, revision regression, path replacement와 late result 적대시험이 필요하다.
- R5 adapter와 shared gate가 identity 전체를 왕복·재검증해야 한다.

## 안전·증거 경계

- session 유지가 이동 허가를 유지한다는 뜻은 아니다.
- reference가 같아도 shared gate와 모든 거부 조건은 매 tick 다시 검사한다.
- 보호정지 전 허가와 이전 stop epoch는 재사용하지 않는다.
- 이 결정은 Python simulation 연구 공정성 계약이며 ROS 2 lifecycle·제품 interface 확정이 아니다.
- 실제 사람 탑승 안전이나 제품 controller 채택의 증거가 아니다.

## 연결 문서

- [`R4 Reference·Sliding Subpath 상세 명세`](../research/dynamic-actor-experiment/15-local-maneuver-reference-contract.md)
- [`R1~R7 master specification`](../research/dynamic-actor-experiment/10-dynamic-local-maneuver-research-master-spec.md)
- [`경로 안전·권한 흐름`](../safety/path-safety-authority-flow.md)
