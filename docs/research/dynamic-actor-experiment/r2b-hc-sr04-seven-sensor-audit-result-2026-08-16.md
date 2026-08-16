# R2-B HC-SR04 7개 순차 관측 감사 결과

- 실행일: `2026-08-16`
- 범위: 공개 simulation-only R2-B 원본·감시 진입 파생 world
- 센서 가정: HC-SR04 7개, 전방 3·후방 2·좌우 1씩
- 순차 trigger 간격: `61ms`
- 판정: `R2-B 관측 자격 보류`
- 비범위: 실제 Arduino, 센서 반사·간섭·온도, 제품 센서 선정, DWB 변경, hidden

## 1. 질문

기존 R2-B hard failure 두 장면에서 HC-SR04 7개 임시 배치가 다음을 제공할 수 있는지
분리해서 확인했다.

1. Actor가 기존 지도 내부 진입점에 도달하기 전에 원시 echo를 얻는가?
2. 그 echo가 기존 `300ms` TTL 안의 유효한 7센서 frame이 되는가?
3. 기존 방향 예측기의 `10Hz × 20개` 동일 Actor 이력을 만들 수 있는가?

## 2. 동결한 입력

- 가상 차체: `virtual_doll_wheelchair_v0_1`
- 센서 순서: 전방 중앙, 전방 좌, 전방 우, 좌측, 우측, 후방 좌, 후방 우
- 첫 센서와 마지막 센서 측정 차이: `0.366s`
- 같은 센서가 다시 측정되는 주기: `0.427s`
- 기존 R2-B 관측 TTL: `0.300s`
- 기존 20-frame 10Hz 이력 시간폭: `1.900s`
- 순차 7센서에서 같은 센서 20회 이력 시간폭: `8.113s`

각 sensor trigger 시점의 실제 witness 차체 pose와 Actor 위치를 따로 사용했다. Actor ID는
독립 감사기에서 어떤 원형 장애물이 echo를 만들었는지 검산할 때만 사용하며, controller용
거리 frame에는 넣지 않았다.

## 3. 결과

### v6 `second-risk-after-corner`

| 대상 | 원본 world 진입 전 감지 | 감시 진입 파생 world 원시 감지 | 유효 frame 감지 |
|---|---:|---:|---:|
| 첫 번째 지연 Actor | `0회` | `1회`, 최대 `0.353s` 선행 | `0회` |
| 두 번째 지연 Actor | `0회` | `0회` | `0회` |

- 원본·파생 world 각각 scan `105개`
- 기존 TTL을 통과한 full frame `0/105`
- stale full frame `105/105`
- 감사 결과 hash: `fe3e3be951f9b838a99d7396a87045d69a8ef639c75601f88f583b96224bea23`

두 번째 Actor는 추상 감시 접근 world에 `t=0`부터 존재하지만 현재 7개 좁은 방향 cone과
차체 궤적의 조합에서 진입 전에 잡히지 않았다. 첫 번째 Actor도 원시 echo 선행시간이
`0.353s`라 기존 20-frame 방향 이력을 만들 수 없다.

### legacy `dynamic-change-restop`

| 대상 | 원본 world 진입 전 감지 | 감시 진입 파생 world 원시 감지 | 유효 frame 감지 |
|---|---:|---:|---:|
| 두 번째 지연 Actor | `0회` | `6회`, 최대 `2.404282...s` 선행 | `0회` |

- 원본·파생 world 각각 scan `82개`
- 기존 TTL을 통과한 full frame `0/82`
- stale full frame `82/82`
- 감사 결과 hash: `a7cb7ca163acca9fd84b66c03613e310d3b1927dbdaf1812eee12f80b0f9909f`

원시 echo 선행시간은 기존 2초 준비시간을 넘지만, full frame이 완성될 때 첫 센서 표본은 이미
300ms TTL을 넘는다. 또한 거리 frame에는 같은 Actor임을 결박할 ID·2차원 위치·속도가 없다.

## 4. 판정

현재 방식은 다음 네 이유로 R2-B를 통과하지 못한다.

1. 7개 full frame 전달시간 `0.366s`가 기존 TTL `0.300s`보다 길다.
2. 같은 센서의 반복 주기 `0.427s`는 기존 10Hz 이력을 제공하지 못한다.
3. v6 두 번째 진입 Actor는 현재 배치에서 진입 전 원시 감지가 없다.
4. 원시 거리만으로 동일 Actor의 ID·2차원 위치·속도를 만들 수 없다.

이는 DWB 실패가 아니고 HC-SR04 자체의 최종 탈락 판정도 아니다. 다음 중 어느 입력 방식을
채택하느냐에 따라 다시 감사해야 한다.

- 센서별 표본을 full scan 완료 전 즉시 전송
- 전방·후방·측면을 서로 다른 주기로 읽는 우선순위 schedule
- 상호 간섭을 검증한 병렬 sensor group
- 거리 이력만 사용하는 정지 전용 계약
- 별도 센서와 결합한 위치·속도 추정 계약

TTL·안전거리·Actor 반경·Capsule을 결과에 맞춰 완화하지 않았다.

## 5. 증거와 한계

- 구현: `simulation/path_planning_lab/src/hospital_path_lab/r2b_ultrasonic_audit.py`
- 거리 frame: `simulation/path_planning_lab/src/hospital_path_lab/ultrasonic_observation.py`
- 시험: `simulation/path_planning_lab/tests/test_r2b_ultrasonic_audit.py`
- 관련 표적·회귀: `33 passed`

원형 반사체와 이상적 cone만 사용했다. 벽·정적 장애물 echo, 가림, 재질, 온도, 다중경로,
센서 간 간섭과 실제 장착 오차는 포함하지 않았다. 따라서 실제 성능은 이 결과보다 좋아질 수도,
나빠질 수도 있으며 실측 없이 제품 안전성을 주장할 수 없다.
