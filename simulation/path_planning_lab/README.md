# Python 경로 알고리즘 실험실

합성 병원 지도에서 역할이 같은 경로 후보를 동일 조건으로 비교하는
`simulation_only` 연구 환경이다. 센서 입력 대신 완전한 ground truth를 사용하며,
제품 코드·실물 주행·실제 사람 탑승 안전성의 증거가 아니다.

구현과 시험의 연결 및 현재 한계는 [TRACEABILITY.md](TRACEABILITY.md)에서 추적한다.
움직이는 원형 Actor를 이용한 다음 비교 단계와 1시간 단위 구현 순서는
[동적 사람 회피 비교실험 v5](../../docs/research/dynamic-person-avoidance-experiment-plan-2026-08-10.md)에
동결했다.
전체 모듈 구조와 6단계 입력·출력·시험·완료조건은
[동적 원형 Actor 비교실험 설계 명세](../../docs/research/dynamic-actor-experiment/README.md)를
따른다.
현재는 1단계의 seed 기반 원형 Actor·20 Hz ground-truth trace·controller 입력 분리·
결정론적 JSON/PNG, 2단계의 10 Hz 열화 관측·source validation·공통 Actor prediction
tube, 3단계의 공통 동적 safety gate·제한 감속·`stop_epoch`·재개 권한·deadline
폐기, 4단계의 PP·DWA adapter·동일 gate·20 Hz 동적 closed loop, 5단계의 controller와
독립된 200 Hz ground-truth evaluator·golden 6개·development 30개·contract-fault
catalog와 6단계 paired runner·hidden commitment·통계·승격 판정·PNG·회귀 후보 보존을
구현했다. 독립 episode·profile 결과는 process 기반으로 병렬 계산하되 PP·DWA pair는
같은 worker에서 실행하고, 50 ms wall-clock qualification은 worker pool 종료 뒤 직렬로
분리한다. 최종 full hidden 실행은 코드와 manifest를 동결한 고유 output에서 한 번 수행한다.
2026-08-11 `final-v4`에서는 공개 144·hidden 120 runs와 hard-safety `264/264`를
완료했다. DWA는 기능·50 ms·실제 detour/rejoin·gate override 조건이 미달해 승격하지
않았고, 이 합성 실험의 연구 기준선은 `PP + shared gate`로 유지한다.
이 실험실은 `G1~G5` 확인, 7단계 팀 결정, 최종 경로 전략 또는 제품 알고리즘 채택을
수행하지 않는다.

### 동적 지역 기동 연구 R2 실행 결과 — 2026-08-13

R1 prediction 계약 감사 뒤 R2의 두 번째 구현 묶음까지 공개 episode용 label-free
`WitnessWorldSnapshot`, 명시적 maneuver constraint, 검색 결과 계약, 독립 ground-truth
witness validator와 `HOLD_ONLY`·`WAIT_AND_FOLLOW` structured search를 추가했다. WAIT/HOLD
search는 category·oracle 없이 Actor 활성 사건의 deterministic anchor, 20 Hz reference follow와
정확한 Actor 원의 terminal-stopping guard를 사용하고, 모든 선택 후보를 별도의 200 Hz
validator로 다시 검사한다. `HOLD_ONLY`는 episode 전체 정지를 요구하며,
`WAIT_AND_FOLLOW`는 terminal dwell을 제외한 실제 wait 뒤 `0.10m` 이상 후속 progress를
요구한다. resource limit, 후보 count bucket, validator version·validation hash와 wall-clock을
제외한 semantic hash도 결과 계약에 포함한다.

`test_dynamic_witness_search.py`의 현재 14개 pytest case가 이 subset을 검사한다. 후보는 초기
제동과 최소 wait를 반영한 effective departure tick으로 중복 제거하며, 전체 witness를 쌓지
않고 최적 WAIT·HOLD만 유지한다. validator는 5 ms grid 밖 Actor 활성 사건도 exact sample로
추가한다. 별도의
2026-08-13 읽기 전용 수동 감사에서는 v6 공개 `13/13`과 legacy golden `6/6`의 selected
witness가 독립 validator를 통과했다. 이는 체크인된 전체 taxonomy audit나 영구 CI 근거가
아니며 category 정답, online controller 실행 또는 제품 채택을 뜻하지 않는다.
`PASS_LEFT/PASS_RIGHT`는
[`R2-PASS 상세 명세`](../../docs/research/dynamic-actor-experiment/12-pass-structured-witness-search.md)에
따라 label-free 후보 생성, 종류별 best, 매 이동 tick terminal-stopping guard, 단일 200 Hz
엄격 검증과 결정론적 process-shard 검색까지 구현했다. 2026-08-13 공개
`same-direction-wide-r00~r04`의 총 `135,360`후보를 14 process로 완전탐색해 5개 모두 좌·우
PASS witness를 찾았다. 후보 bucket은 validated `38,660`, dynamic reject `70,318`, geometry
reject `26,382`로 전체 합계와 일치했다. 같은 작은 공개 파생 입력의 serial·parallel 결과는
후보 수·선택 witness·validation hash·semantic hash가 동일했다. 기능 실행 wall-clock은 총 약
`29분 29초`였으며 timing qualification이 아니다.

[`R2 profile replay 상세 명세`](../../docs/research/dynamic-actor-experiment/13-witness-profile-replay.md)에
따라 대표 `same-direction-wide-r00` PASS witness를 Ideal·Normal·Stress 관측에 다시 연결했다.
Ideal 최초 READY는 `2.00s`, Normal은 `2.10s`, Stress는 READY 없음이었다. 최초 READY만큼
기존 witness를 지연한 뒤 200 Hz ground-truth로 다시 검사하자 Ideal·Normal 모두 Actor
clearance와 선언된 pass time이 달라져 무효였다. Ideal Capsule containment miss는 `0`이었지만
predicted minimum clearance는 약 `0.07427m`로 `0.08m` 계약에 미달했다. Normal은 dropout
`16/451`, Stress는 `89/451`이었고, Stress는 방향 판단 불가로 보수 종료됐다. 이는 자동
witness가 존재하더라도 관측 준비시간을 포함한 time-aware search가 아직 필요하다는 결과이며,
controller·gate 실행 실패를 뜻하지 않는다.

공개 13+6 영구 audit와 JSON·Markdown·PNG reporting runner를 구현하고 `19/19`를 끝까지
실행했다. PASS 후보 `135,360`개와 WAIT/HOLD 후보 `389`개가 모두 정직한 count bucket으로
집계됐고, 산출물 coverage도 `19/19`였다. 그러나 v6 second-risk와 legacy dynamic-change에서
episode 중간 Actor 출현 뒤 관측 latency 동안 fresh EMPTY가 유지되어 실제 Actor를 Ideal
Capsule이 포함하지 못하는 hard failure 2건이 발생했다. 따라서 실행은 complete지만 결합 R2
자격은 fail이다. 이후 [`ADR 0011`](../../docs/decisions/0011-separate-path-and-perception-research-gates.md)에
따라 R2-A ground-truth path와 R2-B observation/prediction을 분리했다. 두 hard failure는
R2-B에 보존하고, R2-A 미해결 공간 분류를 위한 R3 명세 진입은 허용한다. 상세 수치와 다음
수정 경계는
[`R2 공개 Witness 감사 결과`](../../docs/research/dynamic-actor-experiment/r2-public-witness-audit-result-2026-08-13.md)에
기록했다. 구현 뒤 저장소 전체 회귀는 `688 passed`, Ruff·compileall·diff 검사는 통과했다.
hidden은 생성·열람·실행하지 않았다.

R3의 static grid·oriented footprint·bounded lattice·resource taxonomy·독립 validator 계약은
[`R3 Bounded 공간 Oracle 상세 명세`](../../docs/research/dynamic-actor-experiment/14-bounded-spatial-oracle.md)에
작성했다. contract·독립 swept validator·8-heading lattice·label-free straight projection core는
L1으로 구현했고 public 21-case catalog·process runner·JSON/Markdown/PNG·partial/complete
수명주기와 clean-source receipt까지 코드로 연결했다. R3 직접 영향권은 `38 passed`다.
clean commit `53fd9f8` 대상 14-process 공개 실행은 `21/21`, 관계 오류 `0`, serial/process
parity PASS로 receipt를 생성했고 전체 회귀는 `729 passed`였다. 상세 결과는
[`R3 공개 qualification 결과`](../../docs/research/dynamic-actor-experiment/r3-public-spatial-qualification-result-2026-08-14.md)에
기록했다. 기존 registry의 `state_lattice=deferred` 상태는 변경하지 않는다.

R4 상세 계약, R4-1 immutable reference·sliding window·revision 수명주기와 R4-2 R3 source
변환 구현은
[`R4 지역 기동 Reference·Sliding Subpath 명세`](../../docs/research/dynamic-actor-experiment/15-local-maneuver-reference-contract.md)에
정의했다. 구현 코드는 [`local_reference_contracts.py`](src/hospital_path_lab/local_reference_contracts.py),
[`local_reference_builder.py`](src/hospital_path_lab/local_reference_builder.py), 시험은
[`test_local_reference_contracts.py`](tests/test_local_reference_contracts.py)와
[`test_local_reference_builder.py`](tests/test_local_reference_builder.py)에 있다.
R3 pose·heading·rotation·rejoin 의미를 immutable full reference로 보존하고,
같은 maneuver/path에서는 `subgoal_revision`만 바뀌는 sliding window를 제공한다. 다른
`maneuver_revision`·`path_revision`·`stop_epoch` 또는 session 결과는 거부한다. R4-2 builder는
R3의 feasible·독립 검증·hash/provenance가 일치한 결과만 `SPATIAL_ONLY` LEFT/RIGHT reference로
바꾸며 resource limit·invalid·infeasible을 경로로 승격하지 않는다. R2 temporal evidence 결합,
독립 R4 validator, window manager, public runner와 R5 연결은 미구현이다.

### v6 재자격 진행 상태 — 2026-08-11

현재는 [v6 보정·재자격 명세](../../docs/research/dynamic-actor-experiment/v6-correction-and-requalification.md)에
따라 공개 단계만 보정하고 있다. legacy 36개 corpus는 exact regression으로 유지하고,
방향·코너·세로 경로·다중 Actor를 포함한 v6 공개 13개와 category oracle을 별도로
추가했다. DWA 진단과 동작보존형 계산 재사용은 구현·회귀 검증했으며 98개 공개 대표
snapshot의 최적화 전후 semantic digest가 일치한다. 기존 Python+NumPy 경로는 동결된
직렬 500회에서 DWA deadline miss `100/500`으로 실패했다. 2026-08-12 선택적 C++ 코어와
보수적 정적 geometry lower-bound 최적화를 적용한 독립 timing 재자격에서는 PP와 C++ DWA가
각각 `0/500` miss였고, C++ DWA는 p50 `3.770 ms`, p95 `15.459 ms`, 최대 `35.576 ms`였다.
이는 timing 하위 자격 통과다. 2026-08-12 expanded public 1차 실행은 49 episode × 2 profile
× 2 controller=`196` run의 범위·contract·hard safety를 통과했지만 기능 자격에서 실패했다.
PP/C++ DWA 기능 통과는 Normal `35/49`·`16/49`, Stress `6/49`·`6/49`였고, DWA의
`LOCAL_DETOUR_FEASIBLE` 통과는 `0/11`, 양의 detour 관측은 `0/98`이었다. 이 때문에
직렬 timing은 fail-closed로 실행하지 않았고 receipt·새 hidden도 만들지 않았다.

기존 `dynamic-experiment` public+hidden 일괄 경로는 v6 재자격 동안 차단된다. 부분·축소
실행은 진단용 report만 만들 수 있고 정식 receipt나 새 hidden을 만들 수 없다. 회사 PC의
ignored `final-v4` output도 이 집 PC에는 없으므로 전체 artifact 회귀를 완료했다고 보지
않는다.

현재 pytest는 동적 `187 passed`와 기존 실험실 `148 passed`, 합계 `335 passed`다. 이는
코드 회귀 증거이며 정식 expanded-public qualification receipt나 제품 안전 증거가 아니다.

```text
seed 기반 지도 생성 → graph/grid·step 사건 검증 → 역할별 알고리즘 실행
→ 독립 oracle·안전 검증 → hidden 실패 보존 → 다음 회차 regression 후보
```

## 구현된 후보

| 역할 | 구현 | 비교 목적 |
|---|---|---|
| 전역 기준선·oracle | Dijkstra, NetworkX | 상태와 최단 비용 검산 |
| 전역 경로 | A* | snapshot마다 전체 재탐색 |
| 증분 전역 경로 | D* Lite | 폐쇄·해제와 시작점 이동 때 탐색 상태 재사용 |
| 제한 영역 local 경로 | Grid A* | footprint가 반영된 구성공간의 최적 경로 |
| local 속도 궤적 | DWA | 가상 differential-drive의 2초 후보 궤적 |
| 경로 추종 | Pure Pursuit, RPP | 같은 차체 적분기에서 폐루프 추종 비교 |
| 동적 controller | Dynamic PP, Dynamic DWA | 동일 관측·prediction tube·gate의 stop/hold 대 local detour |

TEB, MPPI, State Lattice, Hybrid A*는 registry에 `deferred`로만 기록한다. 구현 또는
채택된 후보가 아니다.

## 지도 corpus와 통합 실행 범위

`corridor`, `intersection`, `dead_end`, `u_trap` 계열 생성기가 seed로 지도를 만들고,
하나의 `WorldSpec`에서 전역용 graph와 local용 `0.02m` 점유 grid를 만든다. snapshot과
step 기록에는 seed를 포함하고, 각 알고리즘 결과에는 map ID, map·mission·observation
revision과 입력 SHA-256 content hash를 연결한다. 물리 점유와 승인되지 않은 금지
영역은 별도 자료로 유지한다.

생성 batch 하나는 10개이며 분배는 `golden 2 / development 4 / hidden 2 /
regressions 2`다. 통합 `experiment` 명령은 이 batch를 그대로 10개만 쓰지 않고 다음
20개를 평가한다. 이전 실행의 검증된 regression 후보를 명시적으로 재투입하면 그
수만큼 평가 corpus가 늘어난다.

| split | 사례 수 | 선택 방식 |
|---|---:|---|
| `golden` | 12 | 사람이 의도와 hash를 고정한 필수 회귀사례 |
| `development` | 4 | 공개 base seed batch에서 선택 |
| `regressions` | 2 | 공개 base seed batch에서 선택 |
| `hidden` | 2 | 알고리즘 소스와 공개 18개를 동결한 뒤 별도 hidden seed batch에서 선택 |

기본 실행은 20개이고, 이전 후보를 `N`개 재투입한 실행은 `regressions 2+N`, 전체
`20+N`개다. 승격한 후보도 공개 corpus에 포함해 hidden 선택 전에 함께 동결한다.

고정 golden은 단일·대체 경로, 동일 비용 분기, 막다른 길, 고립 목적지, 넓은 복도,
좁은 문, 부분 점유, 전체 차단, U자 함정, 반복 폐쇄·해제, stale·판단 무효화의 12개다.
생성 사건은 wall clock이 아닌 step으로 통로 폐쇄·해제, 장애물 생성·이동·제거,
시작점 이동, 목적지 변경과 입력 무효화를 재현한다.

통합 실행의 호환 범위는 다음과 같다.

- 전역 3종: 모든 case의 step 0부터 마지막 사건 step까지 실행한다. 무효 입력 step은
  세 알고리즘이 보수적으로 거부하는지 검사한다.
- Grid A*와 DWA: 입력이 유효하고 전역 reference path가 있는 모든 step에서 실행한다.
- Grid A*의 Grid Dijkstra oracle은 Grid A*와 동일하게 footprint·금지영역을 반영한
  구성공간과 동일한 reference search bounds 안에서만 탐색한다.
- Pure Pursuit와 RPP: Grid A*가 `FOUND`를 반환한 모든 step에서 같은 경로와 같은
  시뮬레이션 시간 예산으로 실행한다.
- 이전 step 결과의 stale 거부 증거는 전역뿐 아니라 local·follower 역할에도 남긴다.
- `A* → Grid A* → Pure Pursuit/RPP` 두 종단 조합은 역할별 비교와 섞지 않고 별도
  pipeline 결과와 Pareto 집계로 저장한다. pipeline 성공에는 A*·Grid A*의 결과
  validation과 oracle 일치, follower 초기 명령 validation까지 모두 필요하다.
- hidden 시각화: 평가한 모든 hidden step마다 graph PNG와 grid/path/trajectory PNG를
  각각 만든다.

저장소의 hidden은 비밀 데이터가 아니라 과적합을 줄이기 위한 절차적 구분이다.
hidden hard failure는 같은 실행에서 재튜닝하지 않고, 실패 step까지의 event prefix와
원본 provenance를 새 JSON 회귀 후보로 보존한다. 다음 실행에서 사용자가
`--regression-input-dir`과 `--regression-limit`을 지정하면 먼저 발견한 모든 record의
무결성과 provenance를 검증한다. 같은 원본 world·episode에서 여러 실패 step이 나오면
가장 큰 failing-step prefix 하나만 남겨 world 중복 없이 regression split에 추가한다.
runner는 그 prefix의 step 0부터 마지막 step까지 모두 실행하므로 앞선 실패도 다시
검사한다. 이후에만 `--regression-limit`을 적용한다. 이는 일반적인 delta debugging
최소화와는 다르다.

## 가상 차체와 추종 시간 예산

모든 local 후보와 추종기는 `virtual_doll_wheelchair_v0_1`을 공유한다.

- 본체 `0.32 × 0.40m`, collision footprint `0.36 × 0.44m`
- differential drive, 제자리 회전 가능
- 전진 `0.30m/s`, 일반 주행 `0.20m/s`, 후진 `0.10m/s`
- 각속도 `0.80rad/s`, 가속 `0.25m/s²`, 감속 `0.50m/s²`
- 제어 `20Hz`, 최소 장애물 여유 `0.08m`, 정지 여유 `0.15m`

추종 시뮬레이션 시간 예산은 두 추종기에 공통으로 아래처럼 계산한다.

```text
max(30초, reference_path_length / 0.20m/s × 2.5 + 10초)
```

이 값은 긴 경로를 고정 30초로 잘라내지 않기 위한 simulation timeout이다. 아래의
계산시간 deadline과도, 실제 제품의 주행 deadline과도 다르다. 차체 프로필 전체는
`simulation_only: true`이며 실제 차체·제어 수치로 확정하지 않는다.

## 환경 구성

Python `3.11` 또는 `3.12`가 필요하다. 저장소 루트의 PowerShell에서 실행한다.

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python -m pip install --upgrade pip
.\.venv\Scripts\python -m pip install -e ".\simulation\path_planning_lab[dev]"
```

## 실행 명령

방향성 Actor 예측 계약 1단계 감사는 공개 v6 13개만 사용하며, 기존 경로를 덮지 않는 새
output 디렉터리를 요구한다.

```powershell
$runId = Get-Date -Format "yyyyMMdd-HHmmss"
$outputDir = ".\simulation\path_planning_lab\outputs\prediction-contract-audit-$runId"
.\.venv\Scripts\python.exe .\simulation\path_planning_lab\scripts\run_prediction_contract_audit.py --output-dir $outputDir
```

출력은 `prediction_contract_audit.json`과 `summary.md`다. 공개 motion bound와 Ideal
입력·Capsule 위반만 hard failure이며, Normal·Stress의 Gaussian `2σ` 및 Capsule miss는
통계 coverage와 limitation으로 보존한다. 이 명령은 hidden을 만들거나 실행하지 않는다.

구현·후속 후보 목록:

```powershell
.\.venv\Scripts\hospital-path-lab.exe list-algorithms
```

v6에서는 먼저 public-only 재자격만 실행한다. 매번 존재하지 않는 새 output 경로를
사용하며, 부분·축소 실행은 진단 report만 만들고 정식 receipt를 만들지 않는다.

```powershell
$runId = Get-Date -Format "yyyyMMdd-HHmmss"
$outputDir = ".\simulation\path_planning_lab\outputs\dynamic-public-v6-$runId"
.\.venv\Scripts\hospital-path-lab.exe dynamic-public-qualification --base-seed 20260811 --simulation-workers 6 --output-dir $outputDir
```

출력은 public report·gate와 paired 결과, contract·hard-safety·category 기능·직렬 50 ms
qualification 증거를 포함한다. 전체 canonical 범위와 동결 횟수를 모두 실행하고 모든
gate가 통과한 경우에만 `public_qualification_receipt.json`이 생긴다. 현재 CLI에는 v6
hidden 생성·실행 명령이 없다. `final-v4`의 legacy hidden 명령과 산출물 설명은 과거
회귀 이력이며 새 최종평가 절차가 아니다.

worker 내부 경과시간은 병렬 contention이 섞인 nonqualification 값이고, 50 ms 판정에는
worker pool 종료 뒤 부모 프로세스의 직렬 qualification만 사용한다.

기본 20개 corpus 직렬 평가. 이전 산출물을 덮지 않도록 실행별 output 디렉터리를
권장한다.

```powershell
$runId = Get-Date -Format "yyyyMMdd-HHmmss"
$outputDir = ".\simulation\path_planning_lab\outputs\experiment-$runId"
.\.venv\Scripts\hospital-path-lab.exe experiment --base-seed 20260810 --hidden-seed 91260810 --output-dir $outputDir
```

이전 실행의 `regression_candidates/`를 다음 회차에 최대 5개 재투입하는 예시다.
`--regression-input-dir`에는 이전 experiment output 루트나
`regression_candidates` 디렉터리를 지정할 수 있다.

```powershell
$previousOutput = ".\simulation\path_planning_lab\outputs\experiment-20260810-120000"
$nextOutput = ".\simulation\path_planning_lab\outputs\experiment-regression-replay"
.\.venv\Scripts\hospital-path-lab.exe experiment --base-seed 20260810 --hidden-seed 91260810 --regression-input-dir $previousOutput --regression-limit 5 --output-dir $nextOutput
```

기존 소형 정적 graph 기준선과 보호정지 재개 게이트 데모:

```powershell
.\.venv\Scripts\hospital-path-lab.exe benchmark --repeats 100
.\.venv\Scripts\hospital-path-lab.exe safety-demo
```

전체 회귀시험과 lint:

```powershell
New-Item -ItemType Directory -Force .\simulation\path_planning_lab\outputs\test-runs | Out-Null
$testRun = ".\simulation\path_planning_lab\outputs\test-runs\$([guid]::NewGuid())"
.\.venv\Scripts\python -m pytest -c .\simulation\path_planning_lab\pyproject.toml .\simulation\path_planning_lab\tests -p no:cacheprovider --basetemp=$testRun
.\.venv\Scripts\python -m ruff check .\simulation\path_planning_lab
```

## 판정과 산출물

R2-A 표적 보완으로 같은 방향 추월과 별개인 횡단 Actor 좌·우 우회, 그리고 두 위험의
`실제 정지 → 0.10m 이상 이동 → 별도 재정지 → 회복` offline ground-truth 도구가 추가됐다.
legacy 표적 2건은 맞춰졌고 전체 코드 회귀는 `691 passed`였지만, 공개 19개 R2 audit은
보완 뒤 다시 실행하지 않았다. 다중 위험 사례의 R2-B `ideal_capsule_ground_truth_miss`도
남아 있다. 이는 online controller나 제품 알고리즘
채택 결과가 아니다. 상세 범위는
[R2-A 보완 문서](../../docs/research/dynamic-actor-experiment/r2a-crossing-and-restop-supplement-2026-08-13.md)를
따른다.

전역·local·추종 결과를 하나의 종합 점수로 섞지 않는다. `pareto.json`에 역할별
원시 집계를 저장하며, 실행시간은 p50·p95·p99·최악값을 기록한다. peak memory는
`tracemalloc` 오버헤드를 전체 시간 분포에 섞지 않기 위해 알고리즘×지도 계열의 첫
표본만 측정하고 나머지 표본은 `0`으로 기록한다.

별도 pipeline Pareto도 합산 stage 실행시간의 p50·p95·p99·최악값을 기록한다. 각
pipeline 표본의 memory는 A*·Grid A*·follower stage에 기록된 값 중 최댓값이며 정책명은
`maximum_profiled_stage_sample_or_zero`다. 따라서 동시에 측정한 종단 프로세스 peak가
아니며, 어느 stage도 해당 표본에서 profile되지 않았다면 `0`일 수 있다. pipeline
성공은 두 planner의 `FOUND`, follower 도착, 무충돌·deadline 준수·결정성에 더해 위
세 component validation/oracle 조건을 모두 만족해야 한다.

연구 실행기의 계산시간 이상 감지 기준은 전역 `5초`, local `30초`, 추종 `60초`다.
`measured_elapsed_ns > deadline_ns`이면 hard failure다. 이 값은 모두
`simulation_only` 연구 기준이며 제품 실시간 요구사항이 아니다. 현재 구현은 호출이
반환된 뒤 측정시간을 비교하는 사후 판정이다. 멈춘 호출을 제한시간에 강제 종료하는
watchdog은 구현하지 않았다.

실행 디렉터리에는 다음 산출물이 생긴다.

- `experiment_results.json`: config, 알고리즘 manifest, 기본 20개와 재투입 regression
  corpus의 모든 step 결과, 전역·local·follower stale 증거, 63-step 동적 local 증거,
  보호정지, hard failure와 명시적 한계
- `pareto.json`: 역할별 성능·정확성·안전 지표 및 별도 종단 pipeline 집계와 deadline
  miss 수
- `summary.md`: 실행 조건과 결과 요약
- `visualizations/*.png`: 모든 hidden step의 graph와 grid/path/trajectory 그림
- `regression_candidates/`: hidden hard failure가 있을 때만 생성되는 비덮어쓰기 JSON

oracle 불일치, 충돌·금지영역 진입, stale 결과 실행, 무단 재개, 비결정성,
예외·NaN/inf와 simulation-only deadline 초과는 hard failure다. follower가 공통 시간
예산 안에 도착하지 못하면 golden·development·hidden·regressions 구분 없이 모두
`follower_timeout` hard failure다. local 계획기의 보수적 `NO_PATH`는 별도 기능 한계로
기록될 수 있다.

각 corpus step의 DWA 평가는 one-shot local 궤적 판정이다. 이와 별도로 reference
path에서 `0.11m` 벗어난 초기 pose로 시작하는 synthetic 63-step 상태 유지 계약을
실행한다. 처음 3 step은 `create → hold → hold` 차단, 다음 60 step은
`remove → hold × 59` 해제 상태다. 이 계약은 안전정지 3회와 교착을 관측한 뒤 회복을
확인하며, 먼저 `0.10m` 초과 path deviation을 실제 관측하고 장애물 제거 뒤 다시
진행해 tracking error가 `0.10m` 이하가 된 경우에만 재합류로 판정한다. 충돌·여유·
추종 오차와 finite command도 기록한다. 다만 전체 corpus의 동적 장애물을 상태 유지
DWA 폐루프로 재생하는 시험은 아직 미측정이다. 종단 pipeline은 Grid A* 경로를
사용하며 DWA 출력을 추종기에 연결한 종단 시험이 아니다.

JSON·PNG·Markdown 결과는 `outputs/`에 저장되고 기본적으로 Git에서 제외된다. Python
시험이 성공해도 센서 오차, 통신 지연, 실제 회전·제동, 축소 실물 또는 사람 탑승
안전성을 입증하지 않는다. 경로가 `FOUND`여도 이동 허가로 간주하지 않는다.

## 선택적 C++ DWA 코어

동적 DWA의 217개 후보 생성·41 pose rollout·terminal stopping·정적/금지구역/Actor
충돌검사·비용·정렬은 선택적 C++20 공유 라이브러리로 실행할 수 있다. Python은
시나리오, 관측, 권한, 결과 조립과 기존 shared safety gate를 계속 담당한다. 라이브러리가
없거나 ABI가 맞지 않으면 기존 Python 경로로 보수적으로 fallback한다.

```powershell
.\.venv\Scripts\python.exe .\simulation\path_planning_lab\scripts\build_cpp_dwa_core.py
```

빌드 산출물은 `src/hospital_path_lab/_native/`에 생성되며 Git에 포함하지 않는다. C++
경로의 강제 비활성화는 `HOSPITAL_PATH_LAB_DISABLE_CPP_DWA=1`을 사용한다. 상세 계약과
현재 검증 범위는
[v6 C++ DWA·충돌 코어 구현 명세](../../docs/research/dynamic-actor-experiment/v6-cpp-dwa-core.md)를
따른다. 공식 5-case×100 timing은 `0/500` miss로 통과했지만 expanded public qualification,
receipt, hidden 또는 제품 채택 근거가 아니다.
