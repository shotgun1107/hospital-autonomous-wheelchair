# 최소 FastAPI 검증 서버

이 폴더는 `hospital_path_lab.runtime.R7Runtime`을 HTTP로 호출해 보는 작은 메모리 서버다.
DB, 로그인, 배차, raw 카메라 처리와 모터 통신은 포함하지 않는다.

필드별 요청·응답, 단위, 호출 순서와 오류 코드는
[R7 FastAPI 송수신 명세](../docs/integration/r7-fastapi-api-spec.md)를 따른다. 실행 중인
서버가 제공하는 기계 판독 명세는 `http://127.0.0.1:8000/openapi.json`이다.

## 설치

프로젝트 루트의 Python 3.12 가상환경에서 실행한다.

```powershell
.\.venv\Scripts\python.exe -m pip install -e ".\simulation\path_planning_lab[dev,native]"
.\.venv\Scripts\python.exe -m pip install -e ".\server[dev]"
```

기본 앱은 native DWB를 요구한다. 먼저 기존 native build 명령을 실행해야 한다. 단순 HTTP
기능 확인용 `hospital_server.demo:app`은 RPP와 Ideal 관측 프로필을 명시적으로 주입한다.
이는 제품 알고리즘 선택이나 native DWB 자격을 대신하지 않는다.

## 실행

```powershell
.\.venv\Scripts\python.exe -m uvicorn hospital_server.demo:app --host 127.0.0.1 --port 8000
```

- Swagger UI: `http://127.0.0.1:8000/docs`
- 상태 확인: `GET http://127.0.0.1:8000/health`
- 쉬운 데모 시작: `POST /demo/start`
- 값 변경 데모: `POST /demo/step`
- 데모 상태: `GET /demo/status`
- 미션 시작: `POST /v1/missions`
- 제어 tick: `POST /v1/missions/{mission_id}/steps`
- 상태 조회: `GET /v1/missions/{mission_id}`
- 정지 완료 미션 제거: `DELETE /v1/missions/{mission_id}`

같은 미션의 `steps` 요청은 서버 내부 lock으로 직렬 처리한다. tick을 건너뛰거나 순서를
바꾸면 runtime의 기존 안전정지 흐름으로 들어간다. reset 뒤에도 같은
`mission_id + mission_revision`은 다시 사용할 수 없으며 새 revision이 필요하다.

Swagger에서 직접 값 변화를 보려면 다음 순서로 누른다.

1. `POST /demo/start`에서 기본값 `goal_x_m=2.0`으로 실행한다.
2. `POST /demo/step`을 기본값 `observation_mode=empty`로 세 번 실행한다.
   결과의 `command.motion_state`가 `holding`, `holding`, `moving`으로 바뀐다.
3. 같은 `/demo/step`에서 `observation_mode=actor`, `actor_x_m=1.2`,
   `actor_y_m=0.5`로 바꾸어 두 번 실행한다.
4. 두 번째 Actor 요청은 실제 10Hz frame 전달 tick이며 `observation_frame_sent=true`,
   `command.motion_state=braking`으로 바뀐다.

각 응답에는 실제로 사용한 control tick, 관측 frame 전송 여부, 입력 mode, 최종 선속도·
각속도와 정지 이유가 함께 나온다. 데모를 처음부터 다시 하려면 Uvicorn을 재시작한다.

## 시험

```powershell
.\.venv\Scripts\python.exe -m pytest -q .\server\tests
.\.venv\Scripts\python.exe -m ruff check .\server
```

이 서버는 시뮬레이션 연결 시험용이다. 실제 사람 탑승 안전, 실제 카메라 인식 성능이나
실제 모터 명령 전달을 증명하지 않는다.

## 현재 확인된 native cold-start 제한

`hospital_server.app:app`은 기본 native DWB 설정이다. 새 Uvicorn 프로세스에서 첫 DWB
controller 호출이 50ms를 넘으면 기존 shared gate가 `deadline`으로 제동한다. 2026-08-20
로컬 smoke에서도 이 보수정지가 확인됐다. 이 서버는 안전 시간 기준을 완화하거나 첫 명령을
우회하지 않는다. 실제 서버 연결 전에는 native 초기화·warm-up을 어느 lifecycle 단계에서
수행하고 그 결과를 어떻게 자격화할지 별도로 정해야 한다.
