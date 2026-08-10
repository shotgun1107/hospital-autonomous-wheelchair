# Python 경로 알고리즘 실험실

합성 병원 지도에서 역할이 같은 경로 후보를 동일 조건으로 비교하는
`simulation_only` 연구 환경이다. 센서 입력 대신 완전한 ground truth를 사용하며,
제품 코드·실물 주행·실제 사람 탑승 안전성의 증거가 아니다.

구현과 시험의 연결 및 현재 한계는 [TRACEABILITY.md](TRACEABILITY.md)에서 추적한다.
움직이는 원형 Actor를 이용한 다음 비교 단계와 1시간 단위 구현 순서는
[동적 사람 회피 비교실험 v5](../../docs/research/dynamic-person-avoidance-experiment-plan-2026-08-10.md)에
동결했다.
이 실험실은 `G1~G5` 확인, 7단계 팀 결정, 최종 경로 전략 또는 제품 알고리즘 채택을
수행하지 않는다.

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

구현·후속 후보 목록:

```powershell
.\.venv\Scripts\hospital-path-lab.exe list-algorithms
```

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
