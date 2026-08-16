# R2-B 감시 진입 구간 공개 보정 결과

## 1. 판정

- 원본 R2-B 실패 장면: 변경·삭제하지 않음
- 추상 감시 접근 계약: 구현 완료
- v6 `second-risk` 파생 Ideal containment miss: `0`
- legacy `dynamic-change` 파생 Ideal containment miss: `0`
- 실제 카메라·FOV·가림·검출 계약: 미구현
- formal R5-C/R6·hidden·제품 자격: 미완료

이번 결과는 사람이 위험 관련 구역에 들어오기 전부터 같은 track으로 감시되는 추상 공개
시뮬레이션 조건만 검증한다. 실제 병원에서 카메라가 그 접근 구간을 볼 수 있다는 뜻이 아니다.

## 2. 구현

지연 출현 Actor마다 원래 진입 상태에서 같은 속도로 `t=0`까지 접근 궤적을 역산했다. 원본
world는 음성 회귀로 유지하고, 감시 접근 Actor를 가진 파생 world는 새 hash로 분리했다.

다음은 바꾸지 않았다.

- 원본 witness 경로와 시간
- Actor의 원래 진입 시각 이후 위치·속도·반경·revision
- Ideal `100ms` 지연과 10Hz 관측
- 20-frame 방향 예측
- Capsule과 ground-truth containment 판정
- 차체·안전거리·shared gate

경로 제어기에는 진입 시각, 정답 궤적, 평가 label을 전달하지 않는다.

## 3. 결과

| 항목 | v6 `second-risk` | legacy `dynamic-change` |
|---|---:|---:|
| 원본 대표 replay miss | `38` | `22` |
| 과거 전체 audit miss | `38` | `43` |
| 원본 최대 miss | `0.18m` | `0.18m` |
| 파생 replay miss | `0` | `0` |
| 파생 Ideal hard failure | `0` | `0` |
| 파생 최초 READY tick | `40` | `40` |
| 감시 접근 Actor 수 | `2` | `1` |

legacy의 과거 전체 audit `43`과 이번 대표 replay `22`는 서로 다른 선택 witness 묶음에서 나온
수치다. 둘 다 같은 episode-level `ideal_capsule_ground_truth_miss`를 재현한다. 과거 수치를
새 수치로 덮어쓰지 않는다.

### 결정론적 식별자

| 항목 | v6 | legacy |
|---|---|---|
| 계약 hash | `da7bf95fec01b7f0f7cd07b9f3791e6e61c8a4952a92019dfea521afacc94528` | `d05aa05cee5cfa8001081d37f87ef288840e8f60937a0d2d65333d8809319b2a` |
| 파생 world hash | `f01a2c75497f3b78f011664c002fd81700b80633b21414d83a44991f35d41350` | `ca78f731678bff8498aad56e0c9a2da73d6c8df480516ef505a0b4f090ff8c70` |
| replay 결과 hash | `cd4d011a8f4b22aeb34e91ba67f4ad79f475845373793d660558b005a64ec657` | `2b94c019779f07bf17f22ba437351b3ab93d33cd7fc3087128a9c2d2e449e848` |

## 4. 안전 경계

감시 접근 시작 위치는 로컬 휠체어 지도 밖일 수 있다. 이는 “외부 접근 구간도 관측할 수
있다”는 추상 입력이며 실제 카메라 설치 위치나 FOV를 확정하지 않는다.

계약이 없거나, 지연 Actor가 누락되거나, 진입 전 20-frame과 Ideal 지연을 포함한 `2.0s`
여유가 없으면 파생 world를 만들지 않는다. fresh `EMPTY`는 여전히 출발·재출발 근거가 아니다.

## 5. 검증

- R2-B 감시 진입 전용: `9 passed in 20.75s`
- 기존 계약·corpus·ground-truth validator: `57 passed in 35.03s`
- profile replay·출현 경계·공개 audit: `22 passed in 80.28s`
- 영향권 합계: `88 passed`, 실패 `0`
- 전체 회귀: 4개 process shard 합계 `935 passed`, 실패·건너뜀 `0`
  - shard 결과: `185 / 221 / 262 / 267 passed`
- Ruff·compileall·`git diff --check`: 통과

## 6. 남은 일

1. 실제 카메라 연구를 시작할 때 접근 구간 FOV·가림·검출 누락을 별도 계약으로 만든다.
2. Normal·Stress online controller 종단 성공은 별도 R5-C/R6에서 검증한다.
3. hidden, 제품 알고리즘·센서 채택과 실제 사람 안전 주장을 시작하지 않는다.
