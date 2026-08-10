# 1단계 — 동적 시뮬레이션 기반

## 목표

controller 구현을 건드리지 않고 움직이는 원형 Actor와 로봇의 결정론적 ground-truth
시간축, trace, 단일 시나리오를 만든다.

## 진입조건

- 기존 정적 경로 실험실 전체 시험이 통과한다.
- v5 가상 차체 profile을 읽을 수 있다.
- 출력 디렉터리는 Git 추적 대상과 분리되어 있다.

## 수정·추가 대상

```text
src/hospital_path_lab/dynamic_contracts.py
src/hospital_path_lab/dynamic_actor.py
src/hospital_path_lab/simulation.py
src/hospital_path_lab/experiment_visualization.py
tests/test_dynamic_actor.py
tests/test_dynamic_simulation.py
```

기존 `simulation.py`의 정적 follower 시험 API는 깨지지 않게 유지한다.

## 자료형

```text
ActorState
- actor_id: str
- position: Point2D
- velocity: Vector2D
- radius_m: float
- trajectory_revision: int

ActorWaypoint
- simulation_time_s: float
- position: Point2D

DynamicGroundTruthFrame
- episode_id: str
- seed: int
- tick_id: int
- simulation_time_s: float
- robot_state: RobotState
- actors: tuple[ActorState, ...]
- map_revision: int
- mission_revision: int

DynamicTrace
- metadata
- ground_truth_frames
- accepted_commands
- state_events
```

모든 결과 자료형은 immutable하게 만들고 position·velocity·time은 finite인지 생성 시
검사한다. Actor ID는 episode 안에서 유일해야 한다.

## Actor 운동

- Actor는 waypoint 사이를 piecewise-linear하게 이동한다.
- waypoint 시각은 엄격히 증가한다.
- segment 안에서는 속도가 일정하다.
- segment 경계에서 방향이 바뀔 수 있다.
- Actor는 로봇 상태에 반응하지 않는다.
- 같은 seed와 generator version은 같은 waypoint와 hash를 만든다.
- 최대속도 `0.50 m/s`, 반지름 `0.18 m`를 validator가 검사한다.

## 첫 golden 시나리오

```text
이름: corridor_crossing_v1
로봇: 직선 reference path 시작점에서 정지
Actor: 로봇 경로의 측면에서 출발해 복도를 횡단
목적: 동적 ground truth와 trace 재현성 확인
```

첫 단계에서는 회피 성공을 판정하지 않는다. Actor와 로봇 trace가 올바른 시간축으로
생성되는지만 본다.

## 20 Hz tick

```text
dt = 0.05 s
tick 0은 초기상태
tick n의 simulation_time = n · dt
```

부동소수점 누적 덧셈 대신 `tick_id * dt`로 시각을 계산한다. 종료 시각은 tick 수로
동결한다.

## 출력

- JSON: seed, generator version, world hash, 모든 ground-truth frame
- PNG: reference path, 로봇 trace, Actor trace, 시작·종료 위치
- 출력 파일명에는 episode ID와 seed를 포함한다.
- wall-clock timestamp를 결정론적 content hash에 넣지 않는다.

## 시험

| 시험 ID | 내용 | 연결 계약 |
|---|---|---|
| `DYN-T-SIM-001` | 같은 seed의 Actor waypoint와 hash가 동일 | `DYN-ARCH-002` |
| `DYN-T-SIM-002` | 다른 seed는 시나리오 허용범위 안에서 다른 trace 생성 | `DYN-ARCH-002` |
| `DYN-T-SIM-003` | tick 시각이 정확히 `tick_id*0.05` | `DYN-ARCH-002` |
| `DYN-T-SIM-004` | 속도·반지름·waypoint validation | `DYN-ARCH-002` |
| `DYN-T-SIM-005` | JSON 재실행 결과가 동일 | `DYN-ARCH-002` |
| `DYN-T-SIM-006` | PNG가 생성되고 figure가 누수되지 않음 | 추적성 |
| `DYN-T-SIM-007` | controller-facing 자료형에 ground truth 객체가 없음 | `DYN-ARCH-001` |

## 완료조건

- 단일 corridor crossing episode가 20 Hz로 실행된다.
- 같은 seed에서 ground-truth JSON의 의미 내용과 hash가 동일하다.
- JSON과 PNG가 모두 생성된다.
- 기존 정적 실험실 시험이 깨지지 않는다.
- 아직 PP·DWA, noise, gate, hidden을 실행하지 않는다.

## 커밋 경계

```text
implement deterministic dynamic actor simulation core
```
