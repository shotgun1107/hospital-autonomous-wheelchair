# R7 Python runtime 연결 안내

## 무엇인가

`hospital_path_lab.runtime.R7Runtime`은 서버 Python 코드가 기존 R7 경로 제어 묶음을
같은 프로세스 안에서 호출할 수 있게 하는 얇은 연결층이다.

```text
서버의 미션·지도·처리된 사람 관측
  → R7Runtime
  → 기존 관측 검증 → 사람 이동 예측 → persistent controller → shared safety gate
  → 선속도·각속도 명령
```

새 경로 알고리즘, 새 안전 수치, 새 재출발 규칙은 만들지 않는다. HTTP/FastAPI 서버도 이
패키지 안에 넣지 않는다. 이후 서버 담당자가 자신의 FastAPI 코드에서 이 모듈을 import해
사용한다.

이 인터페이스는 R7의 `simulation_only` 차량 프로필과 알려진 지도 가정에만 묶여 있다.
실제 카메라, 모터, 사람 탑승 안전성 또는 의료기기 인증을 뜻하지 않는다.

## 설치와 native 준비

프로젝트 루트에서 다음을 실행한다.

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".\simulation\path_planning_lab[dev,native]"
.\.venv\Scripts\python.exe .\simulation\path_planning_lab\scripts\build_cpp_dwb_safety_core.py
.\.venv\Scripts\python.exe .\simulation\path_planning_lab\scripts\build_cpp_dwb_full_core.py
```

기본 `RuntimeConfig()`는 C++ DWB DLL 두 개를 요구한다. DLL이 없으면 미션 시작을 거부한다.
Python fallback은 연구용 명시 설정에서만 쓸 수 있으며, R7 native 50ms 자격 결과를 이어받지
않는다. RPP는 이 runtime의 가벼운 연결 시험용 선택지이고, 제품 알고리즘 채택을 뜻하지
않는다.

## 가장 작은 사용 예

```python
from hospital_path_lab.runtime import (
    R7Runtime,
    RuntimeMap,
    RuntimeMission,
    RuntimePose,
    RuntimeRobotState,
    RuntimeStepInput,
)

runtime = R7Runtime()

runtime_map = RuntimeMap(
    map_id="floor-1",
    map_revision=7,
    occupancy_rows=known_occupancy_rows,
    resolution_m=0.02,
)
mission = RuntimeMission(
    mission_id="mission-123",
    mission_revision=1,
    runtime_map=runtime_map,
    start_pose=RuntimePose(0.5, 0.5, 0.0),
    goal_pose=RuntimePose(2.0, 0.5, 0.0),
    reference_path=(
        RuntimePose(0.5, 0.5, 0.0),
        RuntimePose(2.0, 0.5, 0.0),
    ),
    observation_stream_id="camera-pipeline-1",
    observation_session_seed=101,
)
runtime.start_mission(mission)

command = runtime.step(
    RuntimeStepInput(
        control_tick=0,
        robot=RuntimeRobotState(RuntimePose(0.5, 0.5, 0.0)),
        observation=None,
    )
)
```

`reference_path`는 서버가 이미 선택한 알려진 지도 위 경로다. 이 runtime은 전역 경로를
새로 찾지 않는다. 입력 경로를 기존 R7 `FOLLOW_ORIGINAL` reference로 바꾸고 기존 독립
validator를 통과한 경우에만 controller를 시작한다.

## 입력 규칙

| 값 | 단위·뜻 |
|---|---|
| `x_m`, `y_m` | 지도/world 좌표계의 m |
| `yaw_rad` | rad |
| `linear_mps`, `angular_radps` | m/s, rad/s |
| `control_tick` | 미션 시작부터 20Hz 정수 tick. 시간은 `tick × 0.05초` |
| `RuntimeObservation.sequence` | 10Hz 관측 순번. 관측 시간은 `sequence × 0.1초` |
| `occupancy_rows[y][x]` | `True`면 정적 장애물 |
| `forbidden_cells` | 들어가면 안 되는 `(x, y)` grid cell |

각 사람 관측은 이미 카메라 처리 단계가 만든 지도 좌표의 `track_id`, 안정적인
`actor_binding_id`, 위치와 속도다. raw RGB frame, YOLO 결과, Arduino 패킷은 이 입력이
아니다.

`actors=()`는 **새로운 빈 관측 frame**이다. 사람을 보지 못했다는 뜻이 아니라, 처리된 최신
frame이 사람 없음이라고 말한 경우다. 반대로 `observation=None`은 새 frame이 도착하지 않은
것이다. 20 Hz 중간 tick의 `None`은 정상적인 frame 사이 간격이지만, 다음 10 Hz 전달 시각의
`None`은 명시적인 no-frame/dropout으로 기록된다. 이 둘을 섞으면 안 된다.

관측 frame은 해당 프로필의 지연 시간이 지난 뒤에만 전달한다. 예를 들어 Normal/Ideal은
sequence `0`을 `control_tick=2`에서 전달한다. sequence를 반복 제출하지 말고, 그 사이 20Hz
tick에는 `observation=None`을 넣는다.

## 상태와 안전 동작

`R7Runtime` 인스턴스 하나는 미션 하나에만 쓴다. 매 tick 새 인스턴스를 만들면 안 된다.
다음 상태는 같은 인스턴스 안에 남는다.

- 관측 sequence·최신성 검증 상태
- 사람 방향을 결정하기 위한 20개 frame 이력
- controller session과 reference window
- shared safety gate의 제동·정지·`stop_epoch`·safe frame 수

사람이 보이면 방향 예측에는 서로 다른 TRACKS frame 20개와 최소 1.9초가 필요하다. 그
전에는 정상적으로 정지 명령이 나온다. 관측이 0.3초보다 오래되거나 형식·지도·순번이 맞지
않으면 기존 gate가 `BRAKING` 후 실제 정지 확인을 거쳐 `HOLDING`으로 간다.

runtime은 재출발 권한을 만들지 않는다. backend/authority 계층에서 만든
`RuntimeResumeAuthorization`을 전달할 수는 있지만, 보호정지 후 기존 reference는
`stop_epoch`가 달라져 무효다. 이 v1 runtime은 자동 재계획·자동 rebind·자동 재출발을 하지
않는다. 새로 검증된 reference와 새 미션/session은 backend가 명시적으로 시작해야 한다.

## 출력

`RuntimeCommand`에는 다음이 들어 있다.

- `linear_mps`, `angular_radps`: shared safety gate가 최종 승인한 명령
- `motion_state`: `moving`, `braking`, `holding`, `completed`
- `stop_reason`, `failure_reasons`: 멈춘 이유
- `control_tick`, `stop_epoch`: 미션 순서와 보호정지 세대
- `observation_status`, `prediction_status`: 관측·예측 입력 상태

서버는 이 명령을 바로 모터 안전 보장으로 해석하면 안 된다. 실제 차체에는 별도의 최종
저수준 안전 차단과 통신 계약이 필요하다.

## 시간 측정 범위

runtime이 controller를 호출할 때는 그 호출 시간을 실제로 재서 기존 shared gate에 전달한다.
50ms를 넘으면 gate는 기존 deadline 규칙으로 제동한다. 다만 runtime 연결 시험 자체는 R7
native 500회 시간 자격을 새로 증명하지 않는다. 성능 자격은 별도 native release gate에서
CPU 경쟁이 없는 상태로 직렬 측정해야 한다.

## 서버 연결 시 주의

- 미션마다 `R7Runtime`을 하나만 두고, 같은 미션의 `step()` 호출은 순서대로 처리한다.
- FastAPI route나 worker가 병렬로 같은 runtime을 호출하지 않도록 미션 단위 lock/queue를 둔다.
- tick이 빠지거나 순서가 바뀌면 runtime은 기존 controller 명령을 catch-up하지 않는다. 대신
  shared gate가 최신 차체 상태로 실제 정지를 확인할 때까지 invalid-source 정지 tick만 처리한다.
- `reset()`은 BRAKING 중에는 거부된다. reset 뒤 같은 `mission_id + mission_revision`은 다시
  시작할 수 없으며, backend가 새 mission 또는 새 revision·reference/session·재개 권한을
  제공해야 한다. runtime은 재출발 권한이나 경로를 만들지 않는다.
- 이 모듈은 hidden runner, corpus, evaluator를 import하지 않는다.

## 확인 명령

```powershell
.\.venv\Scripts\python.exe -m pytest -q simulation\path_planning_lab\tests\test_runtime_r7_runtime.py
.\.venv\Scripts\python.exe -m ruff check simulation\path_planning_lab\src\hospital_path_lab\runtime
```

이 시험은 빈 관측, 사람 방향 warm-up, 가까운 사람에 대한 제동·정지, 상태 유지, reset,
stale·잘못된 지도 입력, 기존 pipeline과 facade 결과 일치를 확인한다.
