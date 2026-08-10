# 4단계 — PP·DWA closed-loop 통합

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

## PP 기준선

- 기존 Pure Pursuit polyline projection을 재사용한다.
- lookahead `0.35 m`, goal tolerance `0.05 m`를 사용한다.
- remaining arc length 기반 감속 목표속도를 적용한다.
- PP는 Actor를 피해 reference에서 이탈하는 경로를 만들지 않는다.
- PP가 낸 추종 명령을 gate가 위험하면 braking/hold로 바꾼다.

PP adapter는 controller 명령과 gate override를 구분해 기록해야 한다.

## DWA 비교군

- 최대 217개 `(v,w)` 후보
- 후보당 2.0초, 0.05초 간격 40구간, 41 pose
- reverse 비활성
- 각 후보 뒤 terminal stopping sweep
- v5의 여섯 비용과 가중치, tie-break를 그대로 사용
- 같은 collision checker와 Actor tube를 사용
- admissible 후보가 없으면 정지 명령과 `NO_SAFE_CANDIDATE`를 반환

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

## mechanism golden

최소한 다음을 고정한다.

1. Actor 없음: 둘 다 `0.20 m/s` 정책으로 완료
2. 횡단 후 해소: PP stop/hold/resume
3. 넓은 공간: DWA local detour/rejoin
4. 좁은 복도: DWA도 추월하지 않고 hold
5. 후보 없음: DWA `NO_SAFE_CANDIDATE`
6. 우회 중 새 Actor 위험: 양쪽 모두 gate 재정지

이 단계에서는 noise profile을 최소화한 golden으로 기능을 먼저 검증한다.

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

## 커밋 경계

```text
integrate pp and dwa dynamic closed-loop pipelines
```
