# 5단계 — ground-truth 평가기와 corpus

## 구현 상태

- 상태: **구현·전용시험·전체 회귀 완료**
- 완료일: 2026-08-11
- 5단계 전용시험: `15 passed`
- 1~5단계 동적 영향권: `133 passed`
- 실험실 전체 회귀: `264 passed`
- 선행 단계: 1~4단계 완료
- 다음 단계 선행 구현: 없음

### v6 공개 corpus·oracle 보정 — 2026-08-11

legacy-v1 공개 36개와 그 exact hash는 그대로 둔 채 v6 공개 13개를 별도 generator로
추가했다. v6는 같은 방향의 넓은·좁은 통로, offset 정면 접근, 대각선 횡단, 코너·교차로,
90도 회전 세로 경로, 동시·시차 다중 Actor를 포함한다. 모든 값은 합성
`simulation_only` calibration이며 병원 사람 행동 분포의 근거가 아니다.

v6 evaluator 전용 metadata와 oracle은 controller 입력과 분리한다. source·map·Actor ID는
평가 label을 드러내지 않는 불투명 ID를 사용하고, grid provenance는 label이 아니라
`semantic_world_hash`에 결합한다. evaluator metadata를 바꿔도 같은 물리 world의
controller input·관측 stream·grid가 바뀌지 않는 회귀시험을 둔다.

기능 oracle은 다음을 추가로 확인한다.

- reference에서 `>0.10 m` 이탈한 뒤 거리 `<=0.10 m`, heading error `<=10°`를
  `0.5 s` 이상 유지한 rejoin
- 같은 방향 Actor에 한정한 path 순서 반전과 종방향 footprint 중첩
- 각 hazard interval에 귀속된 서로 다른 `stop_epoch`; 초기 source-invalid 정지를 두 번째
  동적 위험 정지로 세지 않음
- `LOCAL_DETOUR_FEASIBLE` label에 대한 controller 비종속 witness 존재
- witness 시작 시각 `0`, 마지막 fresh 관측의 TTL `0.30 s`와 apply `0.05 s`를 합친
  `0.35 s` tube 상한
- pipeline step 시간·state 연속성, final state, 재출발 `stop_epoch` 동일성
- Actor 원 궤적과 실제 rasterized occupied·forbidden cell 경계의 비중첩
- 90° rigid pair의 관측 noise와 controller·gate·평가 결과 metamorphic 일치

이 공개 corpus·oracle 구현만으로 DWA 기능 자격이나 제품 안전을 통과한 것은 아니다.

## 목표

controller 입력과 독립된 200 Hz evaluator를 만들고, 기대 행동 범주·관측 fault·권한
fault를 재현하는 golden/development corpus를 생성한다.

## 진입조건

- 4단계 mechanism golden 6개가 통과한다.
- 두 pipeline이 같은 ground truth와 열화 관측 seed를 공유한다.

## 수정·추가 대상

```text
src/hospital_path_lab/dynamic_evaluation.py
src/hospital_path_lab/dynamic_corpus.py
src/hospital_path_lab/map_factory.py
src/hospital_path_lab/experiment_visualization.py
tests/test_dynamic_evaluation.py
tests/test_dynamic_corpus.py
tests/test_dynamic_contract_faults.py
```

## evaluator 시간축

- 200 Hz, `dt=0.005 s`
- 20 Hz chassis pose 사이를 보수적으로 보간한다.
- 이동량이 큰 경우 oriented swept footprint 또는 adaptive subdivision을 사용한다.
- Actor ground truth도 같은 evaluator 시각으로 보간한다.
- 두 sample 사이 충돌을 놓치지 않는 보수성 시험을 둔다.

구현은 20 Hz 각 구간을 5 ms 간격으로 다시 적분한다. sample 사이에서는 로봇 선속도,
회전에 따른 footprint 꼭짓점 속도와 Actor 실제 속도의 반 구간 이동 상한을 surface
clearance에서 빼 보수 판정한다. 이 방식은 controller prediction tube와 독립이며 evaluator
전용 Actor ground truth만 사용한다.

## hard safety 판정

Normal·Stress 공통:

- collision 0
- actual surface clearance `>=0.08 m`
- forbidden zone entry 0
- stale·invalid 이후 새 추진·회전 명령 0
- unauthorized resume 0
- late command applied 0
- NaN·예외·provenance mismatch 0

공통 gate나 PP hard safety가 실패하면 DWA와의 성능 비교 자격 자체를 실패로 처리한다.

## 기능·승차감 지표

- completion time
- safety hold duration과 primary reason별 duration
- controller stop request, gate override, no-candidate counter
- path length와 signed path length delta
- positive detour length
- maximum/RMS reference deviation
- actual longitudinal jerk RMS
- angular acceleration·jerk RMS
- peak angular velocity와 방향전환 횟수
- minimum clearance와 TTC
- rejoin, overtaking, planner deadlock

jerk는 limiter 적용 뒤 simulated chassis velocity로 계산한다.

## 기대 범주와 oracle

| 범주 | corpus oracle |
|---|---|
| `WAIT_AND_RESUME` | Actor 해소 뒤 원 경로 진행 가능 |
| `LOCAL_DETOUR_FEASIBLE` | 동결 tube로도 측면 통과·재합류 가능 |
| `LOCAL_DETOUR_FORBIDDEN` | 폭·forbidden geometry상 추월 불가 |
| `NO_SAFE_SOLUTION` | 진행 불가지만 정지 로봇은 Actor에 침범되지 않음 |
| `OBSERVATION_INVALID` | source 복구 전 정지, 복구 뒤 재검증 가능 |
| `DYNAMIC_CHANGE_RESTOP` | 첫 진행 뒤 두 번째 위험에서 재정지 필요 |

generator가 붙인 label을 controller가 보지 못하도록 직렬화 경계를 검사한다.

## corpus 구성

```text
golden: 범주별 1개, 총 6개
development: 범주별 paired seed 5개, 총 30개
contract-fault: 성능 통계와 별도
```

hidden 30개는 6단계의 동결 뒤 생성한다.

## contract-fault corpus

### observation

- stream·episode·map identity 불일치
- sequence·revision 역행
- hash mismatch
- duplicate track와 actor binding 변경
- fresh empty, single dropout, 4-frame burst
- age 300 ms와 350 ms

### authority

- 이전 epoch, 다른 mission, 잘못된 revision
- 정지 확인 전 authorization
- authorization 없음
- 승인 뒤 새 보호정지
- 위험 해소만 발생

### deadline

- 49/50/51 ms
- 늦은 결과의 이후 tick 도착
- 과거 결과와 최신 결과 순서 역전

## corpus validator

- seed와 generator version에서 content hash 재현
- start·goal·Actor 초기상태 collision-free
- Actor 속도·가속·반지름 제한
- `NO_SAFE_SOLUTION`의 정지 공간 보장
- 범주 oracle과 geometry 일치
- PP와 DWA에 동일 ground truth·observation stream 제공
- corpus label이 controller snapshot에 없음
- split 간 world·episode hash 중복 없음

## 시험

| 시험 ID | 내용 | 연결 계약 |
|---|---|---|
| `DYN-T-EVAL-001` | 5 ms 사이 swept collision 검출 | `DYN-EVAL-001` |
| `DYN-T-EVAL-002` | 실제 surface clearance oracle | `DYN-SAFE-001` |
| `DYN-T-EVAL-003` | hold/deadlock 분리 | 기능 자격 |
| `DYN-T-EVAL-004` | rejoin·overtake 정의 | 기능 자격 |
| `DYN-T-CORP-001` | golden/dev hash 결정론 | `DYN-ARCH-002` |
| `DYN-T-CORP-002` | 범주별 geometry oracle | 기능 자격 |
| `DYN-T-CORP-003` | controller label 누출 없음 | `DYN-ARCH-001` |
| `DYN-T-FAULT-001` | 모든 observation fault 보수정지 | `DYN-OBS-002` |
| `DYN-T-FAULT-002` | 모든 authorization fault 거부 | `DYN-AUTH-001` |
| `DYN-T-FAULT-003` | 모든 late result 폐기 | `DYN-SAFE-002` |

## 완료조건

- golden 6개와 development 30개가 validator를 통과한다.
- PP와 DWA가 paired stream에서 반복 실행된다.
- contract-fault 전체가 보수적으로 거부된다.
- hard safety false-pass 회귀시험이 존재한다.
- hidden seed는 아직 runner에 공개하지 않는다.

2026-08-11 기준 다음 범위로 완료했다.

- 범주별 golden 1개와 development 5개, 총 `6 + 30` episode를 seed 기반으로 생성하고
  동일 seed의 map·Normal/Stress observation hash 재현을 검증했다.
- expectation category, split과 oracle이 `DynamicControllerCorpusInput` 및
  `ControllerSnapshot`에 포함되지 않음을 검사했다.
- golden 6개의 첫 유효 paired snapshot을 PP와 DWA에 실제 재생해 동일 input/observation
  hash를 보존함을 확인했다. 36개 whole-episode 반복 실행과 통계 집계는 6단계 runner
  책임이며 이 단계에서 수행하지 않았다.
- observation·authority·deadline fault 25종을 별도 catalog로 고정했다. Stage 2·3의 실제
  validator/gate 시험과 Stage 5 boundary·dropout 회귀를 함께 자격 근거로 사용한다.
- evaluator는 actual collision·`0.08m` clearance·forbidden·stale/invalid propulsion·
  unauthorized resume·late application·provenance를 hard failure로 분류한다.
- hold와 planner deadlock, 실제 이탈 뒤 rejoin, reference 투영 순서 변화 기반 overtaking,
  path·deviation·jerk·각운동·TTC 지표를 구현했다.
- hidden seed 생성, 전체 paired 실행, 통계와 승격 판정은 시작하지 않았다.

## 커밋 경계

```text
add dynamic evaluator development corpus and fault contracts
```
