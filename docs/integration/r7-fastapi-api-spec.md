# R7 FastAPI 송수신 명세

## 1. 문서 목적과 상태

이 문서는 Python 서버가 HTTP로 R7 runtime을 호출할 때 무엇을 보내고 무엇을 받는지
정리한 현재 구현 명세다.

- 구현 위치: `server/src/hospital_server/`
- 기본 주소: `http://127.0.0.1:8000`
- 본문 형식: `application/json`
- 대화형 문서: `GET /docs`
- 기계가 읽는 OpenAPI: `GET /openapi.json`
- 적용 범위: 알려진 지도와 처리된 사람 관측을 사용하는 simulation-only POC

이 명세는 DB, 인증, raw 카메라 영상, 모터 통신, 실제 사람 탑승 안전을 포함하지 않는다.
RPP와 Ideal 관측을 사용하는 `/demo/*`는 통신 확인용이며 제품 알고리즘 선택이 아니다.

## 2. 전체 통신 흐름

```text
웹 또는 서버 호출자
  → JSON HTTP 요청
  → FastAPI 입력 검사
  → 미션별 R7Runtime
  → 관측 검증 → 사람 이동 예측 → 경로 controller → shared safety gate
  → 선속도·각속도·상태 JSON 응답
```

같은 미션의 `step` 요청은 한 번에 하나씩 순서대로 처리한다. 요청이 동시에 도착해도
미션별 lock이 직렬화한다. 서버는 누락된 tick을 임의 계산하거나 이전 명령으로 채우지 않는다.

## 3. 빠른 대화형 데모 API

### 3.1 `POST /demo/start`

고정된 4m × 4m 빈 지도와 시작 위치 `(0.5m, 0.5m)`로 데모 미션을 시작한다.

요청:

```json
{
  "goal_x_m": 2.0
}
```

| 필드 | 형식 | 범위·뜻 |
|---|---|---|
| `goal_x_m` | number | `0.8 < 값 <= 3.5`, 목적지 x 좌표(m) |

응답:

```json
{
  "mission_id": "interactive-demo",
  "message": "Call /demo/step repeatedly and change observation_mode.",
  "next_control_tick": 0
}
```

한 Uvicorn 프로세스에서 한 번만 시작할 수 있다. 처음부터 다시 하려면 서버를 재시작한다.

### 3.2 `POST /demo/step`

사용자가 입력한 차체 상태와 사람 관측을 현재 20Hz control tick에 전달한다.

요청 기본값:

```json
{
  "robot_x_m": 0.5,
  "robot_y_m": 0.5,
  "robot_yaw_rad": 0.0,
  "robot_linear_mps": 0.0,
  "robot_angular_radps": 0.0,
  "observation_mode": "empty",
  "actor_x_m": 1.2,
  "actor_y_m": 0.5,
  "actor_vx_mps": 0.1,
  "actor_vy_mps": 0.0
}
```

| 필드 | 단위 | 뜻 |
|---|---:|---|
| `robot_x_m`, `robot_y_m` | m | 지도 좌표계의 차체 중심 위치 |
| `robot_yaw_rad` | rad | 차체 방향 |
| `robot_linear_mps` | m/s | 현재 측정 선속도 |
| `robot_angular_radps` | rad/s | 현재 측정 각속도 |
| `observation_mode` | enum | `empty`, `actor`, `missing` 중 하나 |
| `actor_x_m`, `actor_y_m` | m | 처리된 사람 중심 위치 |
| `actor_vx_mps`, `actor_vy_mps` | m/s | 처리된 사람 속도 |

`observation_mode` 의미:

| 값 | 의미 |
|---|---|
| `empty` | 새 카메라 처리 frame이 도착했고 사람이 없음 |
| `actor` | 새 frame이 도착했고 위 Actor 위치·속도가 검출됨 |
| `missing` | 예정된 frame이 도착하지 않음. due tick이면 dropout으로 기록 |

제어는 20Hz, 사람 관측은 10Hz다. 따라서 `/demo/step`을 호출해도 해당 tick이 10Hz frame
전달 시각이 아니면 `observation_frame_sent=false`가 반환된다. 입력값을 버린 것이 아니라
아직 관측 전달 시각이 아닌 것이다.

응답 예:

```json
{
  "input_control_tick": 4,
  "input_observation_mode": "actor",
  "observation_frame_sent": true,
  "observation_sequence": 1,
  "command": {
    "linear_mps": 0.0,
    "angular_radps": 0.0,
    "motion_state": "braking",
    "stop_reason": "invalid_source",
    "control_tick": 4,
    "stop_epoch": 0,
    "failure_reasons": ["fresh_prediction_missing"],
    "observation_status": "fresh",
    "prediction_status": "warming_up"
  },
  "next_control_tick": 5
}
```

`command` 필드:

| 필드 | 뜻 |
|---|---|
| `linear_mps` | shared safety gate가 최종 승인한 선속도 |
| `angular_radps` | shared safety gate가 최종 승인한 각속도 |
| `motion_state` | `moving`, `braking`, `holding`, `completed` |
| `stop_reason` | 정지 이유. 없으면 `null` |
| `control_tick` | 이 응답을 계산한 20Hz tick |
| `stop_epoch` | 보호정지 세대. 실제 정지가 확인되면 증가 가능 |
| `failure_reasons` | gate가 기록한 실패 코드 목록 |
| `observation_status` | 관측의 `fresh`, `stale`, `invalid` 등 상태 |
| `prediction_status` | 사람 예측의 `empty_frame`, `warming_up`, `ready` 등 상태 |

확인 순서:

1. `POST /demo/start`를 한 번 호출한다.
2. `observation_mode=empty`로 `/demo/step`을 세 번 호출한다.
3. 응답 상태가 `holding → holding → moving`으로 변하는지 본다.
4. `observation_mode=actor`, `actor_x_m=1.2`, `actor_y_m=0.5`로 두 번 호출한다.
5. 두 번째 Actor 호출에서 `observation_frame_sent=true`, `motion_state=braking`인지 본다.

### 3.3 `GET /demo/status`

현재 데모 runtime의 상태를 읽는다. 입력 본문은 없다.

```json
{
  "mission_id": "interactive-demo",
  "next_control_tick": 5,
  "motion_state": "braking",
  "stop_epoch": 0,
  "predictor_status": "warming_up",
  "predictor_history_counts": [
    ["interactive-demo-actor", 1]
  ],
  "last_event_was_no_frame": false,
  "controller_name": "persistent_rpp_reference",
  "native_dwb_active": null
}
```

## 4. 일반 미션 API

### 4.1 `POST /v1/missions`

서버가 이미 알고 있는 지도·경로로 미션별 runtime을 만든다. 이 API는 전역 경로를 찾지
않는다. `reference_path`는 호출자가 제공해야 한다.

요청 구조:

```json
{
  "mission_id": "mission-123",
  "mission_revision": 1,
  "runtime_map": {
    "map_id": "floor-1",
    "map_revision": 7,
    "occupancy_rows": [[false, false], [false, true]],
    "resolution_m": 0.02,
    "origin_x_m": 0.0,
    "origin_y_m": 0.0,
    "forbidden_cells": [[1, 1]]
  },
  "start_pose": {"x_m": 0.5, "y_m": 0.5, "yaw_rad": 0.0},
  "goal_pose": {"x_m": 2.0, "y_m": 0.5, "yaw_rad": 0.0},
  "reference_path": [
    {"x_m": 0.5, "y_m": 0.5, "yaw_rad": 0.0},
    {"x_m": 2.0, "y_m": 0.5, "yaw_rad": 0.0}
  ],
  "observation_stream_id": "camera-pipeline-1",
  "observation_session_seed": 101,
  "authorization_revision": 0
}
```

`occupancy_rows[y][x]`에서 `true`는 정적 장애물이다. 모든 행 길이는 같아야 하고
`forbidden_cells`는 지도 안의 `(x, y)` 정수 cell이다. 위 2×2 값은 구조 설명용이며 실제
휠체어 reference 검증에는 차체가 들어갈 수 있는 충분한 크기의 지도가 필요하다.

응답:

```json
{
  "mission_id": "mission-123",
  "state": "started"
}
```

같은 `mission_id`를 동시에 두 번 만들 수 없다. reset 후에도 같은
`mission_id + mission_revision`은 다시 사용할 수 없으며 새 revision이 필요하다.

### 4.2 `POST /v1/missions/{mission_id}/steps`

한 번 호출할 때 정확히 한 control tick을 처리한다.

```json
{
  "control_tick": 2,
  "robot": {
    "pose": {"x_m": 0.5, "y_m": 0.5, "yaw_rad": 0.0},
    "linear_mps": 0.0,
    "angular_radps": 0.0
  },
  "observation": {
    "sequence": 0,
    "observation_revision": 0,
    "observed_at_s": 0.0,
    "actors": [
      {
        "track_id": "track-7",
        "actor_binding_id": "person-7",
        "x_m": 1.2,
        "y_m": 0.5,
        "vx_mps": 0.1,
        "vy_mps": 0.0
      }
    ],
    "map_id": "floor-1",
    "map_revision": 7
  },
  "path_still_valid": true,
  "local_safety_recheck_passed": true,
  "resume_authorization": null,
  "mission_cancelled": false
}
```

중요 규칙:

- `control_tick`은 0부터 시작하는 20Hz 순번이며 한 번에 1씩 증가해야 한다.
- 시간은 `control_tick × 0.05초`다. Unix 시간을 넣지 않는다.
- 10Hz 새 frame이 없는 중간 tick에는 `observation=null`을 보낸다.
- 예정된 frame 시각에도 `null`이면 runtime은 no-frame/dropout으로 기록한다.
- `actors=[]`는 frame 누락이 아니라 정상적으로 사람이 없는 새 frame이다.
- 처리된 Actor의 위치·속도는 지도 좌표계여야 한다. raw 이미지나 bounding box는 받지 않는다.
- `resume_authorization`은 runtime이 만들지 않는다. 별도 권한 계층이 발급한 경우에만 전달한다.

응답 형식은 `/demo/step`의 `command`와 같은 `CommandResponse`다.

### 4.3 `GET /v1/missions/{mission_id}`

미션의 다음 tick, 정지 상태, predictor 이력과 native DWB 사용 여부를 읽는다. 응답 형식은
`GET /demo/status`와 같다.

### 4.4 `DELETE /v1/missions/{mission_id}`

미션 runtime을 제거한다. gate가 `braking`인 동안에는 거부한다. 실제 정지가 확인된
`holding` 또는 완료 상태에서만 제거할 수 있다. 성공하면 HTTP `204`이며 본문은 없다.

## 5. HTTP 오류 코드

| 코드 | 뜻 |
|---:|---|
| `200` | 정상 조회·step |
| `201` | 일반 미션 생성 성공 |
| `204` | 미션 제거 성공 |
| `400` | JSON 형식은 맞지만 runtime 값 계약 위반 |
| `404` | 해당 미션 없음 |
| `409` | 미션 중복·재사용, 시작 전 step, 정지 전 reset 등 상태 충돌 |
| `422` | 필수 JSON 필드 누락, 잘못된 enum·자료형, 허용하지 않은 추가 필드 |
| `503` | 기본 native DWB가 설치되지 않아 미션을 시작할 수 없음 |

예상하지 못한 내부 예외를 정상 응답이나 이동 명령으로 바꾸지 않는다.

## 6. 자동 생성 명세 읽기

실행 중인 서버의 실제 코드와 맞는 OpenAPI JSON은 다음 주소에서 읽는다.

```text
http://127.0.0.1:8000/openapi.json
```

Python, TypeScript 또는 다른 서버는 이 JSON을 이용해 client 코드를 생성할 수 있다.
사람이 직접 시험할 때는 `http://127.0.0.1:8000/docs`를 사용한다.

Markdown과 OpenAPI가 충돌하면 현재 호출 형식은 실행 중인 `/openapi.json`을 먼저 확인하고,
코드·문서 충돌로 기록한 뒤 둘 중 하나를 임의로 조용히 바꾸지 않는다.

## 7. 현재 한계

- 메모리 저장만 사용하므로 서버 재시작 시 미션 상태가 사라진다.
- 로그인, 권한 인증, TLS, DB, 여러 서버 프로세스 간 상태 공유가 없다.
- raw 카메라 영상과 detection 모델은 연결하지 않았다.
- 실제 로봇 위치·속도 수집과 모터 명령 전송은 연결하지 않았다.
- 기본 native DWB 앱의 새 프로세스 첫 호출은 50ms deadline을 넘을 수 있으며, 이 경우 기존
  safety gate가 제동한다. 기준을 완화하지 않았다.
- 현재 HTTP 통과는 시뮬레이션 연결 증거이며 실제 사람 탑승 안전 증거가 아니다.
