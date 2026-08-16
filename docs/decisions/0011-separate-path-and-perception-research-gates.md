# ADR 0011: 경로 가능성 연구와 관측·센서 통합 Gate 분리

- 상태: 사용자 개인 승인, 팀 합의 전
- 날짜: `2026-08-13`
- 범위: 동적 지역 기동 연구 `R1~R7`, Python `simulation_only`

## 배경

기존 R2는 다음 두 질문을 하나의 완료조건으로 묶었다.

```text
정확한 Actor ground truth 아래 안전한 시간 경로가 존재하는가?
관측·prediction으로 그 Actor를 빠짐없이 표현할 수 있는가?
```

공개 13+6 감사에서는 모든 episode 실행과 ground-truth witness 검증을 마쳤지만, episode
중간에 지도 내부에서 즉시 생성된 Actor가 `100ms` 관측 latency 동안 fresh EMPTY 뒤에
숨는 두 장면에서 `ideal_capsule_ground_truth_miss`가 발생했다.

외부 Pro 검토는 다음을 올바르게 지적했다.

- fresh EMPTY와 빈 prediction만으로 비영점 이동을 허가할 수 있는 false-safe 경로
- `active_from_s`가 물리 존재·진입·가시성·관측·track 생성을 혼합하는 corpus 계약
- R1이 생성된 shape에서 Actor 방향으로만 검사해 `Actor 있음 + shape 없음`을 누락한 증거 공백

그러나 사용자는 실제 카메라 영상·FOV·가림·검출·추적은 경로 가능성을 먼저 확인한 뒤
후속으로 다루기로 결정했다. 또한 R3는 관측이나 online gate를 입력으로 받지 않고 static
grid·차체 footprint·시작과 재합류 자세만 검사하는 offline 공간 oracle이다. 따라서 관측
false-safe가 R3의 공간 연구까지 자동으로 막는다는 판정은 범위가 과도하다.

## 결정

R2를 다음 두 evidence lane으로 분리한다.

### R2-A — Ground-truth 시간 경로 연구

질문:

> 정확한 Actor trajectory를 evaluator-only로 사용할 때 안전한 WAIT·PASS·재정지 witness가
> 존재하는가?

입력과 판정:

- exact ground-truth Actor trajectory
- 20Hz 가상 차체 운동학과 200Hz 독립 validator
- exact footprint·static·forbidden·`0.08m` Actor clearance
- ordered wait·departure·overtake·rejoin·restop
- observation, prediction, camera, fresh EMPTY와 online authorization은 판정에서 제외

R2-A의 `SEARCH_INCONCLUSIVE`는 R3의 입력이다. 현재 structured template가 경로를 찾지
못했다는 이유만으로 공간 불가능을 선언하지 않는다.

### R2-B — 관측·Prediction 판단 가능성 연구

질문:

> 추상 `ActorTrack` 또는 후속 실제 센서 관측 결과로 R2-A 기동을 판단할 수 있는가?

입력과 판정:

- observed/delivered time, latency, dropout과 track lifecycle
- fresh EMPTY의 공간·시간 의미
- Actor-present-without-hazard-shape 역방향 coverage
- 방향성 prediction, warmup과 shared safety gate
- 후속 실제 센서 coverage·무응답·검출·추적 계약

현재 hard failure 2건은 R2-B failure로 보존한다. 기존 output을 삭제하거나 limitation으로
낮추지 않는다.

## 단계 Gate

```text
R2-A ground-truth hard safety 검증
→ unresolved spatial/search cases를 R3에 전달 가능
→ R3 공간 oracle
→ R4 local reference 계약
→ R5 path/controller 기능 lane

R2-B 관측·prediction 계약 보정
→ perception-integrated R5/R6 전에 반드시 통과
→ 최종 R6/R7/hidden 자격 전에 다시 결합
```

R2-B 실패는 R3의 정적 공간 탐색을 막지 않는다. 다만 다음에는 사용할 수 없다.

- fresh EMPTY를 이동 안전 근거로 하는 online controller 자격
- 실제 센서 통합 완료 주장
- perception-integrated R6 종단 자격
- 제품 또는 사람 탑승 안전 주장

## 현재 판정

> 2026-08-16 최신 해석: 아래의 `카메라 모델 미구현`은 당시 기록이다. 팀에서 Arduino 계열
> MCU와 초음파 거리 센서를 사용한다는 방향이 전달됐으므로 실제 통합 gate의 다음 입력은
> 초음파 거리·무응답·stale·배치 coverage다. 감시 진입 파생 world는 대표 Ideal miss를
> `38/22 → 0/0`으로 줄였지만 추상 상위 계약일 뿐 실제 센서 gate를 통과한 것은 아니다.
> 세부 경계는 [ADR 0015](0015-ultrasonic-observation-boundary.md)를 따른다.

### R2-A

- 공개 19개 search·ground-truth 검증 실행 완료
- WAIT/HOLD와 same-direction-wide 5개의 좌·우 PASS 확인
- selected witness의 ground-truth 충돌·금지구역·clearance hard failure 없음
- 기대 결과 `17 matched`
- 횡단 Actor detour 1건은 현재 PASS-v1 범위 밖 `SEARCH_INCONCLUSIVE`
- legacy dynamic-change 1건은 두 위험의 재정지 순서가 `not fully covered`

따라서 R2-A는 현재 template 범위에서 부분 완료이며, 미해결 공간 분류를 위한 R3 명세·연구를
시작할 수 있다. 횡단·다중 위험의 시간 witness 보완은 R3와 병행하되 일반 불가능 판정으로
바꾸지 않는다.

### R2-B

- 공개 audit 실행 완료
- 원본 Ideal Actor 출현 coverage hard failure 2건은 음성 회귀로 보존
- 감시 진입 파생 world의 대표 Ideal containment miss `38/22 → 0/0`
- fresh EMPTY는 이동 허가로 사용하지 않도록 보수 경계를 유지
- Arduino·초음파 실측 모델, 배치 coverage와 거리→track 변환 미구현

따라서 추상 감시 진입 계약은 통과했지만 실제 센서 통합 R2-B는 미완료·후속 보류다.

## 안전 및 증거 경계

- R2-A에서 evaluator ground truth를 사용한 사실을 online controller 입력 허가로 확대하지 않는다.
- R3·R4 결과는 공간·reference 연구 근거일 뿐 이동 허가가 아니다.
- R5를 먼저 수행하더라도 `Ideal ActorTrack path/controller lane`으로 표시하고 실제 센서 통합
  결과로 부르지 않는다.
- R2-B를 통과하기 전 perception-integrated R6와 hidden을 실행하지 않는다.
- 실제 초음파 센서·배치·반사 환경·사람 탑승 안전성은 후속 팀 합의와 별도 증거가 필요하다.
- Python wall-clock·CPU·memory·cache는 R2-A/R2-B 기능 판정에 사용하지 않는다.

## 결과

장점:

- 경로 생성 문제와 센서·관측 문제를 서로의 실패로 오인하지 않는다.
- 현재 안전한 ground-truth 경로 연구를 R3·R4로 이어갈 수 있다.
- Pro가 발견한 online false-safe 문제는 독립 hard failure로 보존된다.
- 실제 초음파 관측 구현 전에 DWB·RPP 또는 경로 구조를 검증할 수 있다.

비용과 제한:

- R5·R6 결과를 path-only와 perception-integrated로 이중 표기해야 한다.
- 최종 결합 전에는 제품 수준 종단 성공을 주장할 수 없다.
- R2-A 횡단·다중 위험 범위는 별도로 보완해야 한다.

