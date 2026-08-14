# 4단계 — PP·DWA closed-loop 통합

## 구현 상태

- 상태: **구현·전용시험·전체 회귀 완료**
- 완료일: 2026-08-11
- 전용시험: PP·DWA·공통 pipeline 합계 `16 passed`
- 동적 controller 영향권 시험: `177 passed`
- 전체 회귀: `249 passed`
- 선행 단계: 1~3단계 완료
- 다음 단계 선행 구현: 없음

### v6 보정 메모 — 2026-08-11

v5 완료 이력과 별개로 현재 Python DWA 구현의 실패 원인과 실행시간을 다시 검증한다.
후보 수 `217`, 후보당 `41` pose, 2초 horizon, 비용·tie-break, reverse 금지와 외부
shared safety gate 권한은 바꾸지 않는다.

현재 v6 변경 범위는 후보 탈락 phase/cause 집계, 선택·미선택·미평가 후보의 구분,
controller semantic digest, 동일 step의 Actor tube·immutable grid 기하 재사용이다. 최적화
전후 command·trajectory·상태·사건열이 같아야 하며 시간값만 비교에서 제외한다. 내부
exact safety API가 apply·rollout·terminal 최초 실패를 구분해 주지 않는 경우 이를
`EXACT_SHARED_GATE`로 합쳐 기록하고, 세부 phase를 구현한 것처럼 주장하지 않는다.

이 보정이 공개 기능·직렬 50 ms 자격을 통과하기 전에는 DWA를 승격하지 않는다.

2026-08-14의 [`ADR 0014`](../../decisions/0014-section-bound-bounded-reverse-translation.md)는
후속 R4/R5 persistent reference 연구에서 section-bound 제한 후진을 허용한다. 이 문서의
v5/v6 동적 controller `reverse 비활성` 결과는 역사적 회귀 기준선으로 유지하며 소급 변경하지 않는다.

최종 동작보존 회귀에서 실제 corner는 reference `38.998 s` 대비 optimized `0.075 s`,
multisegment는 `15.474 s` 대비 `0.034 s`였고 controller·진단 digest가 일치했다. 그러나
기존 Python+NumPy 경로의 5-case×100 직렬 측정은 DWA miss `100/500`, p50
`27.506 ms`, p95 `58.033 ms`, 최대 `75.957 ms`로 실패했다. 후보 수·horizon·비용·
tie-break·외부 gate를 완화하지 않고 선택적 C++ 반복 계산 코어를 추가한 뒤, 2026-08-12
동일 timing 재자격은 PP·C++ DWA 모두 miss `0/500`이었다. C++ DWA는 p50 `3.770 ms`,
p95 `15.459 ms`, 최대 `35.576 ms`였다. 이 결과는 controller timing 하위 자격이며
expanded public 기능 자격이나 DWA 승격을 뜻하지 않는다.

## 목표

같은 reference path와 `ControllerSnapshot`을 PP와 DWA에 제공하고, 두 결과를 같은
safety gate와 20 Hz 차체 적분에 연결한다.

## 진입조건

- 3단계 safety·authority·timing fault 시험이 통과한다.
- gate 없이 controller 출력을 actuator에 적용하는 경로가 없다.

## 수정·추가 대상

```text
src/hospital_path_lab/dynamic_contracts.py
src/hospital_path_lab/followers/pure_pursuit.py
src/hospital_path_lab/local_algorithms/dwa.py
src/hospital_path_lab/simulation.py
src/hospital_path_lab/registry.py
tests/test_dynamic_pp_pipeline.py
tests/test_dynamic_dwa_pipeline.py
tests/test_dynamic_controller_parity.py
```

## 공통 controller 계약

```text
ControllerSnapshot
- tick_id
- simulation_time_s
- mission_id
- robot_state
- goal_pose
- reference_path
- static_grid_snapshot
- forbidden_cells
- validated_observation
- actor_tubes
- vehicle_profile
- map/mission/observation revisions
- input_content_hash

ControllerCommandResult
- controller_name
- source_tick_id
- status
- requested_twist
- predicted_trajectory
- failure_reason
- decision_trace
- input provenance
- elapsed_ns
```

두 controller는 expectation category, ground truth Actor, evaluator 결과를 받지 않는다.

`ControllerSnapshot.input_content_hash`는 tick·mission·map/mission/observation revision,
grid content hash와 observation content hash를 canonical JSON으로 직렬화한 SHA-256이다.
snapshot의 명시 필드와 grid·observation·prediction provenance가 다르면 controller 입력은
`INVALID_INPUT`이다. 관측이 stale·invalid이거나 prediction이 없을 때도 임의의 빈 Actor
관측으로 바꾸지 않는다. PP가 경로 명령을 계산할 수 있더라도 최종 gate가 해당 source
상태를 거부한다.

`ControllerCommandResult`는 snapshot의 mission·map·세 revision·grid/observation hash와
`input_content_hash`를 그대로 복사한다. gate용 proposal 변환에서는 이 provenance를
다시 계산하거나 현재값으로 덮어쓰지 않는다. 따라서 계산 도중 입력이 바뀐 결과는
3단계 gate에서 거부된다.

## PP 기준선

- 기존 Pure Pursuit polyline projection을 재사용한다.
- lookahead `0.35 m`, goal tolerance `0.05 m`를 사용한다.
- remaining arc length 기반 감속 목표속도를 적용한다.
- PP는 Actor를 피해 reference에서 이탈하는 경로를 만들지 않는다.
- PP가 낸 추종 명령을 gate가 위험하면 braking/hold로 바꾼다.

PP의 `predicted_trajectory`는 현재 chassis twist로 50 ms 진행한 post-apply pose를
`time_s=0`으로 두고, PP가 요청한 `(v,w)`를 2.0초 동안 유지한 0.05초 간격 41 pose다.
이는 PP가 2초 동안 명령을 고정한다는 제품 주장이 아니라 공통 gate가 현재 명령의
보수적 결과를 검사하기 위한 Stage 4 adapter 계약이다.

PP adapter는 controller 명령과 gate override를 구분해 기록해야 한다.

## DWA 비교군

- 최대 217개 `(v,w)` 후보
- 후보당 2.0초, 0.05초 간격 40구간, 41 pose
- reverse 비활성
- `v=0` 후보는 정지 fallback 판정에만 사용하고 local detour 후보로 선택하지 않는다.
  양의 선속도를 가진 admissible 후보가 없으면 제자리 회전으로 시간을 끌지 않고
  `NO_SAFE_CANDIDATE`를 반환한다.
- 각 후보 뒤 terminal stopping sweep
- v5의 여섯 비용과 가중치, tie-break를 그대로 사용
- 같은 collision checker와 Actor tube를 사용
- admissible 후보가 없으면 정지 명령과 `NO_SAFE_CANDIDATE`를 반환

DWA의 동적 adapter는 기존 정적 planner의 후보 상대 정규화를 재사용하지 않고 v5의
절대 비용식·가중치·동률 규칙을 사용한다. 후보 pose 41개는 PP와 마찬가지로 post-apply
pose의 `time_s=0`부터 시작한다. 각 후보의 rollout과 terminal stopping은 3단계 gate가
사용하는 동일한 static·forbidden·Actor tube 안전평가 함수를 통과해야 한다. DWA 내부
후진 후보만 비활성화하며 기존 정적 DWA 연구시험의 reverse 동작은 변경하지 않는다.

최적화 때문에 후보 순서나 tie-break 결과가 바뀌면 안 된다. caching은 입력 provenance를
key에 포함하고 결정론 시험으로 검증한다.

## closed-loop 처리

```text
reference path
→ controller snapshot
→ PP 또는 DWA command result
→ deadline/tick 검증
→ shared safety gate
→ acceleration limiter
→ next-tick chassis state
→ trace/evaluator
```

두 pipeline의 chassis integration과 goal 판정 코드는 공유한다.

tick의 물리·명령 순서는 다음으로 고정한다.

```text
tick t의 RobotState snapshot
→ controller와 gate 계산
→ 현재 state.twist로 [t,t+0.05] pose 적분
→ gate command를 t+1의 chassis twist로 저장
```

즉 현재 tick에서 계산한 새 명령으로 같은 50 ms 구간을 다시 적분하지 않는다. 이는
3단계의 current-motion apply sweep와 일치한다. controller 계산시간은 Stage 4 결정론
lane에서 고정 주입하고 실제 wall-clock qualification은 6단계로 남긴다.

Stage 4의 closed-loop trace에는 controller 요청, gate 적용 명령, gate override 여부,
motion state, primary hold reason, robot state before/after와 입력 provenance를 모두 남긴다.
이 단계의 `collision=0`은 static grid·forbidden cell과 online Actor prediction tube 계약에
대한 L1 증거다. ground-truth Actor의 200 Hz swept 판정은 5단계 대상이다.

## mechanism golden

최소한 다음을 고정한다.

1. Actor 없음: 둘 다 `0.20 m/s` 정책으로 완료
2. 횡단 후 해소: PP stop/hold/resume
3. 넓은 공간: DWA local detour/rejoin
4. 좁은 복도: DWA도 추월하지 않고 hold
5. 후보 없음: DWA `NO_SAFE_CANDIDATE`
6. 우회 중 새 Actor 위험: 양쪽 모두 gate 재정지

이 단계에서는 noise profile을 최소화한 golden으로 기능을 먼저 검증한다.
`끝까지 실행`은 progressable 사례에서는 goal 도착과 실제 정지 완료, 의도적으로 진행할
수 없는 사례에서는 예상된 `HOLDING/NO_SAFE_CANDIDATE` 도달을 뜻한다. 후자의 정지를
goal 완료로 집계하지 않는다.

## 시험

| 시험 ID | 내용 | 연결 계약 |
|---|---|---|
| `DYN-T-CTRL-001` | PP·DWA 자유주행 목표속도 동일 | `DYN-CTRL-001` |
| `DYN-T-CTRL-002` | 두 pipeline 입력 observation hash 동일 | `DYN-ARCH-001` |
| `DYN-T-PP-001` | PP가 자체 detour하지 않음 | 실험 질문 |
| `DYN-T-PP-002` | goal 근처 감속·도착 | `DYN-CTRL-001` |
| `DYN-T-DWA-001` | 최대 217후보·41 pose 계약 | 재현성 |
| `DYN-T-DWA-002` | 비용·tie-break oracle | 재현성 |
| `DYN-T-DWA-003` | terminal stopping 불가 후보 거부 | `DYN-SAFE-001` |
| `DYN-T-PIPE-001` | 모든 명령이 gate를 통과 | `DYN-SAFE-001` |
| `DYN-T-PIPE-002` | 같은 seed command sequence 결정론 | `DYN-ARCH-002` |

## 완료조건

- golden 6개에서 PP와 DWA pipeline이 끝까지 실행된다.
- 실제 충돌과 forbidden entry가 0이다.
- PP stop과 gate override, DWA no-candidate와 gate rejection을 구분한다.
- 같은 seed의 command·state·event sequence가 동일하다.
- 아직 hidden 통계로 승자를 결정하지 않는다.

위 완료조건은 2026-08-11 기준 충족했다. 넓은 공간 golden에서 DWA의 제한적 이탈과
reference 복귀를 확인했고, 좁은 차단·terminal stop 불가 사례에서는
`NO_SAFE_CANDIDATE`와 보수적 hold를 확인했다. 이는 합성 prediction tube와 static
grid를 사용한 L1 결과이며 실제 Actor ground truth의 200 Hz hard 판정은 아니다.

## 커밋 경계

```text
integrate pp and dwa dynamic closed-loop pipelines
```
