# R7 경로 runtime 함수 호출·반환 명세

## 1. 목적과 현재 범위

이 문서는 서버 담당 Python 코드가 R7 경로 runtime을 직접 import해 호출할 때 사용하는
함수와 자료형을 고정한다. HTTP/FastAPI, DB, 카메라 처리와 모터 통신은 이 명세의 범위가
아니다.

현재 구현이 제공하는 기능은 다음과 같다.

```text
알려진 지도 + 시작 자세 + 목적지 자세
  → 기존 Grid A*가 전역 reference path 생성
  → 기존 reference builder/validator
+ 현재 차체 위치·속도
+ 처리된 사람 위치·속도
  → R7Runtime.step(...)
  → 최종 승인 선속도·각속도와 이동/제동/정지 상태
```

현재 구현이 제공하지 않는 기능은 다음과 같다.

- raw 카메라 영상에서 사람을 검출·추적하는 함수
- HTTP API, 사용자 인증과 DB 저장
- 실제 모터 드라이버로 명령을 전송하는 함수
- 보호정지 뒤 재출발 권한을 자동 생성하는 함수

서버의 기본 입력은 지도, 시작 자세와 목적지 자세다. runtime은 저장소의 기존
`BoundedGridAStarPlanner`를 지도 전체 범위에서 실행해 reference path를 만든다. 이 선택은
simulation runtime 연결 기본값이며 제품 알고리즘 채택이나 `G1~G5` 결정을 뜻하지 않는다.

이 계약은 현재 개인 승인된 simulation-only 연결 계약이며 팀 전체 합의나 제품 알고리즘
채택을 뜻하지 않는다.

## 2. 설치와 import

설치:

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".\simulation\path_planning_lab[dev,native]"
```

서버 코드의 import:

```python
from hospital_path_lab.runtime import (
    R7Runtime,
    RuntimeActorObservation,
    RuntimeConfig,
    RuntimeMap,
    RuntimeMission,
    RuntimeObservation,
    RuntimePlanningError,
    RuntimePose,
    RuntimeRobotState,
    RuntimeStepInput,
)
```

정본 구현 경로:

```text
simulation/path_planning_lab/src/hospital_path_lab/runtime/
```

서버는 hidden runner, corpus, evaluator나 연구 case ID를 import하지 않는다.

### 2.1 전체 데이터 흐름

```text
Backend: mission_id + map + start_pose + goal_pose
  → R7Runtime.start_mission()
  → 기존 Grid A*
  → reference path
  → 기존 reference builder / validator
  → persistent R7 pipeline

매 50ms: 최신 robot state + 새로 도착한 processed Actor observation
  → R7Runtime.step()
  → observation validator
  → direction predictor
  → DWB
  → shared safety gate
  → RuntimeCommand
```

## 3. 공개 함수

### 3.1 생성

```python
runtime = R7Runtime(config: RuntimeConfig | None = None)
```

기본 `RuntimeConfig()`는 다음을 뜻한다.

| 필드 | 기본값 | 뜻 |
|---|---|---|
| `controller_kind` | `RuntimeControllerKind.DWB` | 기존 persistent DWB controller 사용 |
| `global_planner_kind` | `RuntimeGlobalPlannerKind.GRID_ASTAR` | 기존 점유격자 A*로 전역 reference 생성 |
| `observation_profile` | `NORMAL` | 동결된 Normal 관측 규칙 사용 |
| `require_native_dwb` | `True` | C++ full core와 safety core가 없으면 시작 거부 |

RPP와 Ideal 관측은 연결 시험에서만 명시적으로 주입한다. 제품 채택값으로 사용하지 않는다.

### 3.2 미션 시작

```python
runtime.start_mission(mission: RuntimeMission) -> None
```

한 runtime 인스턴스에는 동시에 한 미션만 존재한다. 이미 미션이 있으면
`RuntimeStateError`를 발생시킨다.

이 함수는 지도 변환 → 기존 Grid A* → endpoint 복원 → 기존 reference builder/validator →
persistent controller 준비 순서로 실행한다. 경로를 찾지 못하면 `RuntimePlanningError`를
발생시키며 직선 fallback을 만들지 않는다.

### 3.3 한 control tick 실행

```python
command: RuntimeCommand = runtime.step(value: RuntimeStepInput)
```

호출 한 번은 정확히 한 20Hz control tick이다. 동일 runtime의 `step()`을 여러 thread에서
동시에 호출하면 안 된다. 서버 담당자는 미션별 lock 또는 단일 queue로 직렬 호출한다.

### 3.4 상태 조회

```python
diagnostics: RuntimeDiagnostics = runtime.diagnostics
```

이 값은 상태 표시와 로그를 위한 읽기 전용 요약이다. 이동 허가로 사용하지 않는다.

### 3.5 미션 폐기

```python
runtime.reset() -> None
```

제동 요청만 발생한 상태에서는 reset할 수 없다. shared safety gate가 실제 정지를
`HOLDING`으로 확인했거나 미션이 `COMPLETED`일 때만 가능하다. reset 뒤 같은
`mission_id + mission_revision`을 같은 runtime에 다시 사용할 수 없다.

## 4. 미션 입력 `RuntimeMission`

```python
RuntimeMission(
    mission_id: str,
    mission_revision: int,
    runtime_map: RuntimeMap,
    start_pose: RuntimePose,
    goal_pose: RuntimePose,
    observation_stream_id: str,
    observation_session_seed: int,
    authorization_revision: int = 0,
    reference_path: tuple[RuntimePose, ...] | None = None,
)
```

| 필드 | 형식 | 뜻 |
|---|---|---|
| `mission_id` | non-empty `str` | 서버가 발급한 미션 식별자 |
| `mission_revision` | `int >= 0` | 미션과 reference 변경 세대 |
| `runtime_map` | `RuntimeMap` | 알려진 정적 지도와 금지 cell |
| `start_pose` | `RuntimePose` | 미션 시작 자세 |
| `goal_pose` | `RuntimePose` | 목적지 자세 |
| `reference_path` | `tuple[RuntimePose, ...] | None` | 기본 `None`: 자동 계획, 명시값: 연구·시험 override |
| `observation_stream_id` | non-empty `str` | 처리된 카메라 관측 stream 식별자 |
| `observation_session_seed` | `int >= 0` | 관측 session 결박값 |
| `authorization_revision` | `int >= 0` | 현재 이동 권한 revision |

일반 backend는 `reference_path`를 생략한다. 명시할 경우 첫 위치는 `start_pose`, 마지막
위치는 `goal_pose`와 같아야 하며 연속 duplicate가 없어야 한다.

### 4.1 자동 전역 경로 생성

기본 `GRID_ASTAR`를 선택한 이유는 `RuntimeMap` 자체가 점유격자이고, 기존 구현이 차체 크기,
0.08m 최소 여유, 금지 cell과 대각선 코너 절단 금지를 이미 함께 처리하기 때문이다.
통로 그래프용 A*/Dijkstra/D* Lite는 별도 `GraphMap`이 필요한 반면 backend 기본 입력에는
그 그래프가 없다.

좌표 변환은 기존 `GridMap.world_to_cell()`과 `cell_to_pose()`를 사용한다. 변환식은
`floor((world-origin)/resolution)`이며 planner가 반환한 cell center 중 첫점과 끝점은 서버가
준 정확한 `start_pose`와 `goal_pose`로 복원한다. 중간 yaw는 다음 구간 방향, 마지막 yaw는
`goal_pose.yaw_rad`를 따른다.

### 4.2 지도 `RuntimeMap`

```python
RuntimeMap(
    map_id: str,
    map_revision: int,
    occupancy_rows: tuple[tuple[bool, ...], ...],
    resolution_m: float = 0.02,
    origin_x_m: float = 0.0,
    origin_y_m: float = 0.0,
    forbidden_cells: tuple[tuple[int, int], ...] = (),
)
```

- `occupancy_rows[y][x] == True`: 정적 장애물 cell
- 모든 행 길이는 같아야 한다.
- `forbidden_cells`: 진입 금지 `(x, y)` 정수 cell
- `resolution_m`: cell 한 칸의 실제 길이(m)
- 지도·reference·차체·사람 위치는 같은 좌표계를 사용한다.

### 4.3 자세 `RuntimePose`

```python
RuntimePose(x_m: float, y_m: float, yaw_rad: float = 0.0)
```

| 값 | 단위 |
|---|---:|
| `x_m`, `y_m` | m |
| `yaw_rad` | rad |

모든 수치는 finite여야 한다. `NaN`, `inf`는 거부한다.

## 5. 매 tick 입력 `RuntimeStepInput`

```python
RuntimeStepInput(
    control_tick: int,
    robot: RuntimeRobotState,
    observation: RuntimeObservation | None = None,
    path_still_valid: bool = True,
    local_safety_recheck_passed: bool = True,
    resume_authorization: RuntimeResumeAuthorization | None = None,
    mission_cancelled: bool = False,
)
```

| 필드 | 뜻 |
|---|---|
| `control_tick` | 0부터 1씩 증가하는 20Hz 순번 |
| `robot` | localization과 차체에서 받은 현재 pose·twist |
| `observation` | 이번 tick에 전달되는 새 10Hz 처리 frame, 없으면 `None` |
| `path_still_valid` | 서버/경로 상위 계층의 현재 path 유효 여부 |
| `local_safety_recheck_passed` | 로봇 로컬 재검사 통과 여부 |
| `resume_authorization` | 별도 권한 계층이 발급한 재개 권한, 없으면 `None` |
| `mission_cancelled` | 미션 취소 요청 여부 |

### 5.1 차체 상태 `RuntimeRobotState`

```python
RuntimeRobotState(
    pose: RuntimePose,
    linear_mps: float = 0.0,
    angular_radps: float = 0.0,
)
```

`step()`마다 최신 측정값을 전달한다. runtime 내부의 이전 시뮬레이션 위치보다 이 입력이
우선한다. tick 오류나 관측 오류가 발생해도 최신 유효 차체 속도를 기준으로 제한감속한다.

### 5.2 처리된 사람 관측 `RuntimeObservation`

```python
RuntimeObservation(
    sequence: int,
    observation_revision: int,
    observed_at_s: float,
    actors: tuple[RuntimeActorObservation, ...] = (),
    map_id: str | None = None,
    map_revision: int | None = None,
)
```

```python
RuntimeActorObservation(
    track_id: str,
    actor_binding_id: str,
    x_m: float,
    y_m: float,
    vx_mps: float,
    vy_mps: float,
)
```

이 입력은 raw 카메라 image나 bounding box가 아니다. 카메라/인지 담당이 지도 좌표로
변환한 사람 track의 위치와 속도다.

| 상황 | 전달값 |
|---|---|
| 새 frame에 사람이 있음 | `RuntimeObservation(actors=(actor, ...))` |
| 새 frame에 사람이 없음 | `RuntimeObservation(actors=())` |
| 이번 20Hz 중간 tick에 새 frame이 없음 | `observation=None` |
| 예정된 10Hz frame 시각에도 frame이 없음 | `observation=None`, runtime이 dropout으로 기록 |

제어 시간은 `control_tick × 0.05초`다. Normal/Ideal에서 10Hz sequence 0의 첫 전달 시각은
control tick 2다. 동일 sequence를 매 20Hz tick에 반복 제출하면 안 된다.

사람 방향 예측에는 서로 다른 TRACKS frame 20개와 최소 1.9초가 필요하다. 그 전의
`WARMING_UP` 정지는 정상 동작이다.

## 6. 반환값 `RuntimeCommand`

```python
RuntimeCommand(
    linear_mps: float,
    angular_radps: float,
    motion_state: DynamicMotionState,
    stop_reason: str | None,
    control_tick: int,
    stop_epoch: int,
    failure_reasons: tuple[str, ...] = (),
    observation_status: str | None = None,
    prediction_status: str | None = None,
)
```

| 필드 | 뜻 |
|---|---|
| `linear_mps` | shared safety gate가 최종 승인한 선속도(m/s) |
| `angular_radps` | 최종 승인한 각속도(rad/s) |
| `motion_state` | `MOVING`, `BRAKING`, `HOLDING`, `COMPLETED` |
| `stop_reason` | 주된 정지 이유, 없으면 `None` |
| `control_tick` | 이 결과의 20Hz tick |
| `stop_epoch` | 보호정지 세대 |
| `failure_reasons` | provenance·관측·시간·안전 실패 코드 |
| `observation_status` | `fresh`, `stale`, `invalid` 등 입력 상태 |
| `prediction_status` | `empty_frame`, `warming_up`, `ready` 등 예측 상태 |

서버는 `RuntimeCommand`를 새 이동 허가로 바꾸거나 속도를 키우지 않는다. 이 값은 실제
차체의 저수준 watchdog, 물리 E-stop과 최종 구동 차단을 대체하지 않는다.

`BRAKING`은 정지 요청·제동 중 상태이고 실제 정지 완료가 아니다. `HOLDING`이 실제 정지
확인 상태다.

## 7. 최소 호출 예

```python
from hospital_path_lab.runtime import (
    R7Runtime,
    RuntimeMap,
    RuntimeMission,
    RuntimeObservation,
    RuntimePose,
    RuntimeRobotState,
    RuntimeStepInput,
)

runtime = R7Runtime()

start = RuntimePose(0.5, 0.5, 0.0)
goal = RuntimePose(2.0, 0.5, 0.0)
runtime.start_mission(
    RuntimeMission(
        mission_id="mission-123",
        mission_revision=1,
        runtime_map=RuntimeMap(
            map_id="floor-1",
            map_revision=7,
            occupancy_rows=known_occupancy_rows,
        ),
        start_pose=start,
        goal_pose=goal,
        observation_stream_id="camera-pipeline-1",
        observation_session_seed=101,
    )
)

# tick 0: 아직 첫 10Hz frame 전달 시각 전
command0 = runtime.step(
    RuntimeStepInput(
        control_tick=0,
        robot=RuntimeRobotState(pose=start),
        observation=None,
    )
)

# tick 1: 정상적인 20Hz 중간 tick, 새 10Hz frame 없음
command1 = runtime.step(
    RuntimeStepInput(
        control_tick=1,
        robot=RuntimeRobotState(pose=start),
        observation=None,
    )
)

# tick 2: 새 frame은 도착했지만 사람이 없음
command2 = runtime.step(
    RuntimeStepInput(
        control_tick=2,
        robot=RuntimeRobotState(pose=start),
        observation=RuntimeObservation(
            sequence=0,
            observation_revision=0,
            observed_at_s=0.0,
            actors=(),
        ),
    )
)
```

## 8. 오류와 fail-closed 동작

| 상황 | 결과 |
|---|---|
| `start_mission()` 전 `step()` | `RuntimeStateError` |
| 미션이 있는데 다시 `start_mission()` | `RuntimeStateError` |
| native DWB가 필요한데 DLL 없음 | `RuntimeStateError` |
| start/goal이 지도 밖·장애물·금지구역 | `RuntimePlanningError` |
| 전역 경로가 없음 | `RuntimePlanningError` |
| 자동 경로가 기존 reference validator를 통과하지 못함 | `RuntimeReferenceError` |
| 자료형·finite·지도 구조 오류 | `TypeError` 또는 `ValueError` |
| control tick 누락·역행 | 기존 controller를 catch-up하지 않고 안전정지 진행 |
| 관측 sequence·map·hash 오류 | invalid 입력으로 gate에 전달해 제동·정지 |
| 예정 frame 장기 누락 | stale/dropout으로 제동·정지 |
| controller 계산 50ms 초과 | deadline으로 제동 |
| BRAKING 중 `reset()` | `RuntimeStateError` |

예상하지 못한 controller·gate 예외를 정상 이동 명령으로 바꾸거나 숨기지 않는다.

## 9. 서버 담당의 최소 책임

서버 또는 robot gateway 담당은 다음을 지킨다.

1. 미션 하나당 `R7Runtime` 인스턴스 하나를 유지한다.
2. 같은 미션의 `step()`을 lock/queue로 직렬 호출한다.
3. `control_tick`을 0부터 1씩 증가시킨다.
4. 매 tick 최신 차체 pose·twist를 전달한다.
5. 10Hz 새 관측이 도착한 tick에만 `RuntimeObservation`을 전달한다.
6. 보호정지 뒤 기존 reference·session·권한을 재사용하지 않는다.
7. runtime이 반환한 속도를 상향하거나 실패 상태를 이동으로 바꾸지 않는다.
8. 예외와 `failure_reasons`를 미션·tick과 함께 기록한다.

## 10. 담당 영역 구분

| 담당 | 만들어야 하는 값 | 이 runtime이 하는 일 |
|---|---|---|
| 서버/미션 | mission ID·지도·시작·목적지·권한 | 값 검증과 session 결박 |
| R7 navigation/runtime | 해당 없음 | 기존 Grid A*로 reference 생성·검증 |
| localization/차체 | 현재 pose·linear/angular velocity | 최신 상태로 controller와 gate 계산 |
| 카메라/인지 | Actor track ID·지도 위치·속도 | 관측 검증·방향 예측·충돌 안전검사 |
| R7 runtime | 해당 없음 | 최종 선속도·각속도·상태 반환 |
| MCU/저수준 안전 | 실제 모터 적용·watchdog·E-stop | 이 Python 모듈 범위 밖 |

## 11. 검증 명령

```powershell
.\.venv\Scripts\python.exe -m pytest -q `
  .\simulation\path_planning_lab\tests\test_runtime_global_planning.py `
  .\simulation\path_planning_lab\tests\test_runtime_r7_runtime.py
.\.venv\Scripts\python.exe -m ruff check `
  .\simulation\path_planning_lab\src\hospital_path_lab\runtime
```

현재 runtime 공개 계약시험은 빈 관측, Actor warm-up, 위험 시 제동·실제 정지, 상태 지속,
reset, stale·잘못된 입력, 기존 pipeline과 함수 facade 결과 일치를 검사한다.
