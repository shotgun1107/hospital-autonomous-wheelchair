# 동적 원형 Actor 비교실험 v6 보정·재자격 명세

## 1. 상태와 목적

- 상태: **v6 보정 구현·회귀 완료, 공개 재자격은 50 ms 미달로 보류**
- 기준 소스: `main@fe383b010164e0fc35c932460a6c3200f80d9fd7`
- 기준 tree: `5fb734678b437cc773343058484abb55b472fa98`
- Stage 3~6 구현 기준 commit: `f89713c72c130c89b6d095a77ef90540aa5768f5`
- 증거 범위: Python `simulation_only`
- 제품 알고리즘 채택: 아님
- `G1~G5`와 경로 분석 7단계: 수행하지 않음

이 문서는 v5 실험의 수치·안전 불변조건을 완화하지 않고, `final-v4`에서 드러난
관측성·기능 oracle·실행시간·hidden lifecycle 결함을 고치는 v6 보정 기준선이다.
동적 실험의 v6는 제품 경로 분석의 6단계나 7단계와 다른 연구실 내부 판본명이다.

코드 변경은 이 문서가 먼저 작성·제시된 뒤 시작한다. 1~3단계의 관측·예측·안전·권한
계약은 유지하며, 4단계 controller 진단부터 보정한다.

### 1.1 2026-08-11 현재 구현 스냅샷

- legacy-v1 공개 36개와 exact corpus hash를 그대로 유지한다.
- 방향·코너·교차로·세로 경로·다중 Actor를 포함한 v6 공개 13개를 별도 lane으로
  생성하며 label·oracle·split은 controller 입력에 넣지 않는다.
- `LOCAL_DETOUR_FEASIBLE`의 독립 witness는 20 Hz 차체 적분과 가감속 제한, 200 Hz
  static·forbidden·Normal/Stress tube 검사를 통과해야 한다.
- category oracle은 hazard별 stop epoch와 허가된 중간 재출발, 같은 방향 Actor의
  `이탈 → 추월 → 0.5초 재합류` 순서를 검사한다.
- DWA에는 원인별 후보 진단과 step-local 기하 workspace가 추가됐지만 후보 수·rollout·
  비용·tie-break·외부 shared gate는 바꾸지 않는다.
- runner는 public-only 진입점만 노출한다. 축소·주입 실행은 report-only이고 정식 receipt를
  만들 수 없으며, 새 hidden 생성·실행 경로는 닫혀 있다.
- evaluator는 step 시간축·상태 연속성·최종 상태와 `stop_epoch` 동일성을 검증하고,
  corpus는 Actor 궤적을 연속 사각형이 아니라 실제 raster cell 경계와 대조한다.
- public gate는 전체 episode×Normal/Stress×PP/DWA 결과 조합과 90° rigid pair 결과를
  검증하며, stale signature·소스 변경·결과 누락이 있으면 receipt를 만들지 않는다.

동작보존 최적화 전후 98개 공개 episode/profile 대표 snapshot의 semantic digest는
일치했고, 전체 pytest는 동적 `187`개와 기존 실험실 `148`개, 합계 `335 passed`다.
하지만 동결된 5-case×100 직렬 측정에서 PP는 miss `0/500`, DWA는 miss `100/500`
(`p50 27.506 ms`, `p95 58.033 ms`, 최대 `75.957 ms`)였다. 따라서 expanded public의
정식 full qualification과 receipt 봉인은 수행하지 않았고, v6 기능 자격·승격·제품
알고리즘 결론은 없으며 새 hidden도 생성하지 않는다.

## 2. v5와 `final-v4`의 해석

`final-v4`에서 확인된 값은 다음과 같다.

- hard safety `264/264`, contract fault `25/25`
- hidden Normal 기능 자격: PP `27/30`, DWA `16/30`
- hidden Stress 기능 자격: PP `5/30`, DWA `5/30`
- DWA feasible detour·rejoin `0%`
- DWA 최대 기준경로 이탈 약 `0.00585 m`
- 직렬 50 ms qualification: PP miss `0/400`, DWA miss `324/400`
- DWA p50 약 `65.6 ms`, p95 약 `81.6 ms`
- 승격 조건 3·4·5·10 미달

이 결과는 DWA라는 알고리즘 계열의 일반 판정이 아니다. 현재 사용자 정의 Python DWA
구현과 현재 corpus가 기능·실행시간·gate override 자격을 통과하지 못했다는 regression
자료다. hard safety 통과도 DWA 단독의 안전 성공이 아니라 공통 safety gate를 포함한
pipeline 결과다.

회사 PC의 ignored output
`simulation/path_planning_lab/outputs/dynamic-experiment-20260811-final-v4`는 기존 hidden을
이미 소비한 원본 증거다. 집 PC에는 이 디렉터리가 없으므로 다음 원칙을 적용한다.

1. 기존 hidden seed를 집에서 다시 생성해 최종평가에 사용하지 않는다.
2. 회사 원본 전체를 별도 전달받기 전에는 `final-v4` 전체 동작보존 회귀를 통과했다고
   주장하지 않는다.
3. 전달할 경우 manifest·receipt·paired 120 records·PNG 120개·67개 regression record를
   포함한 전체 output을 hash와 함께 보존한다.
4. 67개 후보만으로 전체 실패를 대표하지 않는다. detour/rejoin·gate override·timing
   실패는 기존 후보 보존 조건에서 누락될 수 있다.

## 3. 유지하는 동결 계약

다음은 이번 동작보존 보정에서 바꾸지 않는다.

- 가상 footprint `0.36 × 0.44 m`, 최소 표면 clearance `0.08 m`
- Actor 반지름 `0.18 m`, 최대속도·최대가속도 각 `0.50`
- 자유주행 목표속도 `0.20 m/s`, 20 Hz control, 50 ms apply latency
- DWA 최대 `217` 후보, 2.0초 rollout, 0.05초 적분, 후보당 41 pose
- DWA 후진 비활성, 비용식·가중치·동률 규칙
- 5 ms 공통 safety sweep와 terminal stopping 검사
- 10 Hz 열화 관측, TTL·noise·dropout, Actor prediction tube
- 실제 정지 확인, `stop_epoch`, 새 authorization과 11 safe frame
- 200 Hz ground-truth evaluator와 `0.08 m` hard criterion
- PP와 DWA의 동일 map·mission·reference·ground truth·observation stream
- controller에 expectation label·oracle·ground truth를 전달하지 않는 경계

결과가 나쁘거나 느리다는 이유로 Actor tube, clearance, safety sweep, 후보 수, horizon,
비용 또는 동률 규칙을 완화하지 않는다. 운동모델이나 후보 정책을 바꿔야 한다면 먼저
별도 실험 판본과 적용조건을 작성하고, 동작보존 최적화와 같은 변경으로 섞지 않는다.

## 4. v6 전체 게이트 순서

```text
legacy-v1 공개 regression
→ observation·authority·deadline fault
→ v6 공개 hard safety
→ 시나리오별 공개 기능 oracle
→ 공개 DWA detour·rejoin 자격
→ process worker 완전 종료
→ 단독 직렬 50 ms qualification
→ public qualification receipt 동결
→ 새 hidden commitment 생성
→ 새 hidden 생성·소비·paired 평가
```

앞 단계가 실패하거나 미실행이면 다음 단계로 가지 않는다. 특히 공개 기능과 직렬
50 ms 자격이 모두 통과하기 전에는 hidden seed·corpus·소비 영수증을 생성하지 않는다.
실패 산출물은 public regression으로 남기되 hidden lifecycle을 시작하지 않는다.

현재 `DynamicExperimentConfig`처럼 시작부터 hidden seed와 commitment를 요구하는 단일
실행 계약은 v6에 맞지 않는다. 실행을 최소 두 상태로 분리한다.

### 4.1 `PUBLIC_QUALIFICATION`

- hidden seed·commitment를 입력받지 않는다.
- public regression, hard safety, fault, 기능, 직렬 timing을 수행한다.
- 성공 시 source·parameter·corpus·oracle·qualification snapshot hash가 결합된
  `public_qualification_receipt.json`을 무덮어쓰기로 남긴다.
- 실패 시 구조화된 gate 결과를 남기고 종료한다.

### 4.2 `HIDDEN_EVALUATION`

- 통과한 public receipt와 동일 source freeze를 재검증한다.
- receipt 이후 코드·수치·corpus·oracle이 바뀌면 실행을 거부한다.
- 새 v6 commitment만 허용한다.
- 기존 `final-v4` commitment와 hidden은 regression 이외 용도로 재사용하지 않는다.

## 5. 4단계 v6-A — DWA 후보 탈락 진단

현재 coarse filter는 서로 다른 탈락 원인을 모두 `None`으로 합치고, exact shared-safety
실패 이유도 버린다. v6는 선택 결과를 바꾸기 전에 후보 진단을 먼저 추가한다.

### 5.1 후보 판정 단계

| 단계 | 의미 |
|---|---|
| `INPUT` | snapshot·prediction·vehicle 계약 검증 |
| `COARSE_ROLLOUT` | 50 ms rollout pose의 빠른 사전검사 |
| `COARSE_TERMINAL` | terminal stopping 사전검사 |
| `EXACT_APPLY` | 현재 운동 50 ms의 공통 5 ms 검사 |
| `EXACT_ROLLOUT` | 후보 rollout의 공통 5 ms 검사 |
| `EXACT_TERMINAL` | terminal stopping의 공통 5 ms 검사 |
| `POST_CONTROLLER_GATE` | controller 밖 최종 shared gate 재검사 |
| `RANKING` | 안전 후보 중 비용·동률 규칙에 따른 선택 |

### 5.2 원인 taxonomy

| 원인 | 의미 |
|---|---|
| `STATIC_OCCUPANCY` | 구성공간 점유 또는 footprint 충돌 |
| `STATIC_CLEARANCE` | 정적 표면 여유 부족 |
| `FORBIDDEN_ZONE` | 금지영역 접촉·진입 |
| `ACTOR_TUBE` | time-indexed Actor tube 여유 부족 |
| `PREDICTION_INVALID` | tube 생성·sampling 입력 무효 |
| `TERMINAL_STOPPING` | 정지 sweep에서만 처음 실패 |
| `SHARED_GATE` | coarse를 통과했지만 exact/shared gate에서 실패 |
| `ADMISSIBLE_NOT_SELECTED` | 안전하지만 score·tie-break 순위에서 선택되지 않음 |
| `SELECTED` | 최종 선택 |

`ADMISSIBLE_NOT_SELECTED`는 안전 탈락과 합치지 않는다. 각 후보에는 sample index,
`(v,w)`, 최초 실패 단계·원인·시간, 가능한 경우 최소 static/Actor clearance를 기록한다.
일반 실행 결과에는 고정 순서 원인별 count와 선택 후보 근거를 남긴다. 전 후보 상세는
공개 실패·regression 진단에서만 제한적으로 보존해 process IPC와 JSON 크기를 통제한다.

동일 입력에서 taxonomy count, 선택 후보, score, tie-break와 최종 command가 결정론적으로
같아야 한다. 진단 추가 자체가 command·trajectory·gate 결과를 바꾸면 실패다.

### 5.3 원인 가설과 금지선

현재 정지 또는 직진 상태의 한 tick angular window와 2초 상수 명령 rollout이 충분한
측면 이탈을 만들지 못해 hold로 빠지는 bootstrap 문제가 있을 수 있다. 이는 아직 후보별
로그로 확증되지 않은 가설이다. taxonomy로 먼저 확인하고, 확인 전에는 운동모델·window·
rollout 정책을 원인으로 단정하거나 바꾸지 않는다.

## 6. 4단계 v6-B — 동작보존형 50 ms 최적화

### 6.1 허용 범위

- 한 `step()` 안에서 immutable grid·forbidden geometry 재사용
- 동일 rollout 시각의 Actor tube sampling 재사용
- 점유 cell 좌표·distance 관련 step-local workspace 사전계산
- rollout과 terminal 구간에서 동일한 불변 계산 제거
- 결과를 바꾸지 않는 자료구조·loop 최적화

공통 safety gate의 최종 권한은 유지한다. DWA 내부 exact 판정을 했다는 이유로 외부 gate를
건너뛰지 않는다. persistent full-result cache로 같은 qualification snapshot의 반복 측정을
속이는 것도 금지한다. cache key에 `input_content_hash`만 쓰지 않는다. 현재 그 hash에는
robot state·goal·reference path·vehicle 전체가 포함되지 않기 때문이다.

### 6.2 동작보존 oracle

최적화 전·후 같은 공개 입력에서 timing을 제외한 다음이 같아야 한다.

- status, failure reason과 stop/no-safe flag
- requested `(v,w)`
- 41-pose trajectory 전체
- 후보별 단계·원인 count와 선택 순위
- 비용 6종, score와 tie-break 결과
- controller decision trace
- shared gate command·override·motion state·hold reason
- robot state·event sequence

기존 `command_state_event_hash`는 trajectory와 decision trace를 포함하지 않으므로 v6에는
별도 `controller_semantic_digest`를 추가한다. serial과 process 실행에서도 같은 digest를
요구한다.

회사 `final-v4` 전체 output을 전달받지 못한 동안에는 legacy public corpus와 고정 공개
regression으로 이 oracle을 수행하고, 결과에 `final_v4_full_artifact_available=false`를
명시한다.

## 7. 직렬 wall-clock qualification

qualification은 모든 episode worker가 종료된 뒤 부모 process에서 단독 직렬 실행한다.
worker 내부 elapsed는 `nonqualification` 진단값일 뿐 자격 판정에 사용하지 않는다.

### 7.1 동결 snapshot set

현재처럼 네 golden의 첫 frame만 고르지 않는다. 다음 case가 실제로 포함됐음을 validator가
검사하고 case ID·input hash·ordered suite hash를 manifest에 남긴다.

1. Actor 0명, free-space
2. 활성 Actor 1명
3. 같은 snapshot에 활성 Actor 2명
4. 최대 tube와 static·forbidden geometry가 동시에 있는 all-candidate stress
5. 다중 segment reference path가 있는 corner/intersection

각 case는 첫 frame이 아니라 지정된 tick에서 뽑는다. Actor 수, tube 수, static·forbidden
존재, path segment 수가 기대와 다르면 qualification을 시작하지 않는다.

### 7.2 측정 계약

- numeric thread 1
- active simulation worker 0
- parent process PID·CPU affinity·machine ID 기록
- case별 warm-up 30회, 측정 100회
- PP와 DWA 각각 `deadline_miss_count == 0`
- 유효 기준은 `elapsed <= 50 ms`
- p50·p95·p99·maximum·peak memory 기록
- cold-case와 steady 반복을 구분해 persistent-cache 착시를 확인

기존 네 case 기준이면 controller당 400회였지만, v6 snapshot set 수가 바뀌면 총 sample 수는
manifest의 case 수×100으로 계산한다. pass 기준은 sample 수와 무관하게 miss 0이다.

## 8. 5단계 v6 — 공개 시나리오 확장

legacy generator v1의 공개 36개와 public corpus hash
`f7c7a5635458daad4233d8b2b067d27b014619a655a9f020e039ba77c4018abd`는 exact regression
lane으로 유지한다. v6 scenario는 별도 generator version과 안정된 seed namespace를 쓴다.

### 8.1 episode 메타데이터

evaluator 전용으로 다음을 추가한다.

- `scenario_family`
- `variant`
- `orientation`
- `latent_case_id`
- `static_layout_spec`
- `oracle_spec`
- `semantic_world_hash`
- `oracle_hash`

controller snapshot과 observation에는 위 label·oracle을 넣지 않는다. full content hash와
별도로 ID·split·label을 제외한 `semantic_world_hash`를 사용해 public·regression·hidden의
물리적 중복을 거부한다.

### 8.2 필수 공개 scenario matrix

| family | 대표 기대 범주 | 필수 검증 |
|---|---|---|
| 기존 직선 횡단·정지 | 기존 6개 유지 | legacy 결과와 hash regression |
| 같은 방향, 넓은 공간 | `LOCAL_DETOUR_FEASIBLE` | PP 대기, DWA 실제 이탈·추월·재합류 |
| 같은 방향, 좁은 공간 | `LOCAL_DETOUR_FORBIDDEN` | 양쪽 추월 0, 해소 뒤 완료 |
| offset 정면 접근 | `WAIT_AND_RESUME` | 정지·대기·재개, 추월 판정 적용 제외 |
| 대각선 횡단 | `WAIT_AND_RESUME` | x/y 속도 모두 있는 tube와 해소 전 정지 |
| 코너·교차로 | `WAIT_AND_RESUME` | 다중 segment tangent·회전 footprint·static topology |
| 두 번째 위험 교차로 | `DYNAMIC_CHANGE_RESTOP` | 정지→재개→별도 보호정지 |
| 세로 경로 | 원본과 동일 | 수평 scene의 90° rigid-transform metamorphic oracle |
| 동시 2 Actor | `WAIT_AND_RESUME` | 활성구간 중첩, Normal frame 2 track, 모두 해소 뒤 재개 |
| 시차 2 Actor | `DYNAMIC_CHANGE_RESTOP` | 서로 다른 두 보호정지 epoch |

코너·교차로는 occlusion이나 실제 병원 보행 규칙을 주장하지 않는다. 이 실험에서 검증하는
것은 polyline 방향, footprint 회전과 static topology뿐이다.

수직 회전본은 기능 gate에는 포함하지만 원본과 같은 `latent_case_id`이므로 통계에서
독립 표본으로 두 번 세지 않는다.

### 8.3 신규 scenario 값의 상태

기존 차체·Actor·주기·clearance 값은 동결이다. 다음은 v6 공개 calibration 값이며 기존
제품값이나 v5 동결값으로 가장하지 않는다.

- 같은 방향: `0 < actor_speed < 0.20 m/s`
- 초기 종방향 표면 안전간격은 최소 `0.48 m`
- offset counterflow의 횡방향 중심간격은 최소 `0.44 m`
- 대각선은 x/y 속도가 모두 0이 아니고 벡터 크기 `<=0.50 m/s`
- 신규 offset·교차각·corner 치수·replica 수는 public generator와 oracle test에서 먼저
  고정하고 manifest에 기록한다.

Actor의 전체 활성 궤적에 대해 map boundary·static obstacle·Actor 간 초기/시간축 겹침을
검증한다. 여러 Actor는 frozen dataclass/enum/spec으로 표현하고 worker에 lambda·closure·
lock·generator를 전달하지 않는다.

## 9. category별 기능 oracle

`functional_qualified`를 단순 완료·deadlock 검사로 끝내지 않는다.

| 범주 | PP 기대 | DWA 기대 | 공통 실패조건 |
|---|---|---|---|
| `WAIT_AND_RESUME` | 정지·대기·재개·완료 | 정지 또는 안전한 진행 뒤 완료 | 해소 전 위험 진행, 미완료, 무단 재개 |
| `LOCAL_DETOUR_FEASIBLE` | 대기 뒤 완료 | `>0.10m` 이탈, 안전 통과, 재합류, 완료 | witness 없음, 실제 이탈 없음, 재합류 없음 |
| `LOCAL_DETOUR_FORBIDDEN` | 추월 0, 해소 뒤 완료 | 추월 0, 해소 뒤 완료 | 금지된 통과·추월 |
| `NO_SAFE_SOLUTION` | 안전정지 유지 | 안전정지 유지 | 진행·추월·충돌 |
| `OBSERVATION_INVALID` | 복구 전 제동·hold | 복구 전 제동·hold | invalid 입력 추진·과거 결과 재사용 |
| `DYNAMIC_CHANGE_RESTOP` | 두 위험에 각각 정지 | 두 위험에 각각 정지 | 두 번째 위험 무시·epoch 혼합 |

### 9.1 feasible label witness

`LOCAL_DETOUR_FEASIBLE`는 통로 폭만으로 붙이지 않는다. evaluator 전용 독립 witness가
같은 차체 제한, time-indexed tube, static·forbidden geometry, terminal stopping과 episode
시간창에서 safe detour·rejoin이 가능함을 보여야 한다. witness는 DWA 비용·tie-break를
사용하지 않고 controller 입력에도 들어가지 않는다. witness가 없으면 feasible label을
거부하고 scenario geometry/timing을 고친다. safety 조건을 완화하지 않는다.

### 9.2 rejoin과 overtaking

- rejoin: 먼저 reference에서 `>0.10 m` 이탈한 뒤 distance `<=0.10 m`, heading error
  `<=10°`를 `0.5 s` 연속 유지
- overtaking: 같은 방향 Actor에만 적용하며 path 투영 순서 반전과 통과 중 종방향
  footprint 투영 중첩을 함께 요구
- 정면 접근·횡단에는 overtaking을 적용하지 않는다.

공개 golden은 category별 oracle을 모두 통과해야 한다. 공개 development의
`LOCAL_DETOUR_FEASIBLE` DWA detour·rejoin은 최소 기존 승격 기준인 80%를 충족해야
public 기능 gate를 통과한다. 어떤 public hard-safety 실패도 허용하지 않는다.

## 10. process 병렬 공정성

```text
job = episode × observation_profile
same worker = 같은 외생 입력으로 PP 실행 후 DWA 실행
parent = corpus → profile → controller 순 결정론적 재정렬
qualification = worker pool 종료 후 부모에서 직렬
```

- map·Actor ground truth·observation stream은 job당 한 번 결정되어 불변 공유한다.
- controller state와 context state는 PP·DWA마다 새로 초기화한다.
- 각 record에 worker PID, episode seed, stream hash와 pair ID를 남긴다.
- serial 1-worker와 process N-worker의 semantic digest·결과 순서가 같아야 한다.
- 기본 worker 수는 process affinity의 논리 CPU 약 50%와 job 수 중 작은 값으로 하며,
  실제 값과 계산 근거를 manifest에 기록한다. 명시 override를 허용한다.
- 상세 후보 로그는 고정 순서 집계와 제한된 failure detail만 부모로 보내 IPC 폭증을 막는다.

## 11. public receipt와 새 hidden commitment

public receipt에는 최소 다음을 포함한다.

- source freeze와 code commit
- legacy·v6 public corpus hash와 semantic world set hash
- scenario matrix·generator·oracle version/hash
- PP·DWA·vehicle·observation·safety parameter hash
- fault·hard-safety·category 기능 결과
- controller semantic digest set
- qualification case 목록·ordered set hash·timing 결과
- machine·affinity·worker 0 확인

새 hidden commitment는 단순 `sha256(seed)`가 아니라 다음 freeze에 결합한다.

```text
public receipt hash
+ source/parameter hash
+ public semantic-world set hash
+ scenario matrix version
+ qualification snapshot set hash
+ new hidden seed
```

receipt 뒤 하나라도 바뀌면 commitment를 폐기한다. hidden을 한 번 소비한 뒤 변경하면
같은 hidden은 regression으로 전환하고 새 commitment를 만든다.

## 12. 시험과 완료조건

### 12.1 진단·동작보존

- 모든 rejection phase·cause가 구조화 reason으로 집계된다.
- selected와 admissible-not-selected가 안전 탈락과 분리된다.
- 최적화 전후 controller semantic digest가 timing 제외 완전히 같다.
- 기존 217후보·41 pose·비용·tie-break·terminal·5 ms gate 시험이 유지된다.

### 12.2 corpus·oracle

- legacy public 36개 hash regression이 통과한다.
- 신규 family·orientation·multi-Actor가 모두 생성·검증된다.
- label·oracle·split은 controller에 누출되지 않는다.
- feasible witness, rejoin 0.5초+heading, 같은 방향 overtaking oracle이 독립시험을 갖는다.
- semantic world 중복과 rigid-transform 통계 중복이 거부된다.

### 12.3 실행·lifecycle

- serial과 process paired 결과·hash·순서가 같다.
- 같은 job의 PP·DWA가 같은 worker PID·seed·stream hash를 가진다.
- public hard safety·fault·기능 중 하나를 강제 실패시키면 hidden generator와 receipt가
  호출되지 않는다.
- timing miss를 강제하면 hidden lifecycle이 시작되지 않는다.
- qualification은 active worker 0, serial parent에서 수행된다.
- PP와 DWA의 모든 qualification sample이 `<=50 ms`, miss 0이다.

## 13. 구현 순서와 중단조건

1. 이 문서와 v6 계약 ID를 문서 인덱스에 연결한다.
2. 후보 탈락 taxonomy와 동작보존 digest를 구현한다.
3. 최적화 전 공개 semantic baseline을 저장한다.
4. step-local 동작보존 최적화를 한 종류씩 적용하고 매번 oracle을 재검증한다.
5. qualification snapshot validator와 직렬 50 ms 측정을 고친다.
6. legacy-v1 regression lane과 신규 scenario schema를 구현한다.
7. 독립 feasible witness·category oracle·rejoin/overtaking gap을 고친다.
8. public-only lifecycle과 public receipt를 구현한다.
9. expanded public을 process-paired 실행하고 public gate를 판정한다.
10. 모든 public gate가 통과할 때만 별도 작업으로 새 hidden을 준비한다.

다음 경우에는 hidden으로 가지 않고 public regression 보고에서 멈춘다.

- DWA가 동작보존 최적화 뒤에도 50 ms miss를 낸다.
- feasible witness가 있지만 DWA가 공개 detour·rejoin 자격을 통과하지 못한다.
- category oracle, hard safety 또는 fault가 하나라도 실패한다.
- 최적화 전후 semantic digest가 다르다.
- 회사 `final-v4` 원본이 필요한 주장을 하면서 원본을 전달받지 못했다.

이 중단은 제품 알고리즘의 기각이나 경로 분석 7단계 결정이 아니다. 현재 연구 구현의
미충족 조건을 기록하고 다음 보정 범위를 다시 명세한다는 뜻이다.
