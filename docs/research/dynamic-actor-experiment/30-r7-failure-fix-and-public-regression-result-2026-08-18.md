# R7 실패 수정·공개 회귀시험 결과

- 상태: 공개 입력 수정·회귀시험 완료
- 작성일: `2026-08-18`
- 기준 명세: [29-r7-failure-trace-and-public-regression-spec.md](./29-r7-failure-trace-and-public-regression-spec.md)
- 수정 전 기준 커밋: `fd7576baaf6ac1c33ed89dd4fcf280d8c917d29d`
- 증거 범위: Python·native 혼합 시뮬레이션 연구 하네스의 공개 회귀
- 비적용 범위: 새 hidden 실행, 제품 알고리즘 채택, 실제 사람 탑승 안전성, G1~G5 결정

## 1. 최종 결론

29번 명세에서 공개 회귀로 전환한 세 오류를 수정했다. 안전 거리, 관측 TTL, 후보 수,
차체 제한과 평가 기준은 완화하지 않았다.

| 항목 | 실제 원인 | 수정 결과 |
|---|---|---|
| P0-A | gate가 `BRAKING`으로 바뀐 뒤에도 이전 runtime을 호출해 과거 `CONTINUATION` 권한을 재사용하려 함 | 제동·정지 중 이전 runtime 호출 금지, 실제 `HOLDING` 확인 뒤 폐기, 현재 `stop_epoch`에 묶인 새 runtime만 생성 |
| P0-B | 재출발 조건이 gate가 확인한 frame 수가 아니라 predictor의 READY 수에 연결됨 | usable directional 입력과 safe gate 결과를 모두 만족한 서로 다른 10 Hz frame만 별도 계수하고, 같은 frame을 승인 발행에 재사용하지 않음 |
| P0-C | tick 503에서 executor active section은 0인데 local window는 1부터 시작했고, 자동 catch-up은 기준선 거리 `0.064553 m > 0.05 m` 때문에 거부됨 | active section이 실제로 창에서 빠질 예정이며 catch-up 거리 조건도 실패할 때만 그 구간을 임시 보존 |

P0-C의 첫 구현은 active section을 항상 보존해 기존 왼쪽 우회가 멈추는 회귀를 만들었다.
이 회귀는 최종 반영 전에 발견했으며, 보존 조건을 위의 두 조건이 동시에 성립할 때로
좁혀 새 tick 503 사례와 기존 왼쪽 우회 사례를 모두 통과시켰다.

## 2. 추적 기록

`r7_failure_trace.py`에 다음을 구현했다.

- control tick당 한 줄의 결정론적 JSONL 기록
- `TRACE_START`에서 시작하는 record hash chain
- wall-clock 값을 제외한 실행 전체 의미 지문
- 기존 파일을 덮어쓰지 않는 `tick-trace.jsonl` writer
- 기준 commit/tree, 공개 사례, side, profile, seed, tick 수, 주기와 source SHA-256을 묶는
  `run-manifest.json` writer

추적기는 이미 계산된 값을 읽기만 한다. controller나 gate를 추가 호출하지 않으며, 추적
on/off 실행 결과가 같은 시험으로 이 경계를 고정했다.

## 3. 공개 입력 결과

### 짧은 원인 재현

| 공개 입력 | 종료 tick | 결과 |
|---|---:|---|
| Normal LEFT `1993037174228324916` | 259 | `BRAKING/HOLDING` 중 `CONTINUATION` 호출 없음 |
| Normal RIGHT `4525333994236990214` | 503 | executor active section이 local window 안에 유지됨 |
| Stress LEFT `6422064046178126625` | 532 | gate-confirmed 11개 frame 전에는 release·첫 이동·controller 호출 없음 |

### Normal 전체 실행

| 공개 입력 | 결과 | 해석 |
|---|---|---|
| RIGHT `8970341022568507592` | `completed`, completion tick `1178` | 정지 뒤 현재 권한으로 복구하고 완료 |
| LEFT `6422064046178126625` | `conservative_hold` | Actor 통과 증거가 생기기 전에 관측이 empty가 되어 자동 재출발하지 않음 |

두 번째 LEFT 결과는 미수정 실패가 아니다. 통과 증거가 없는 상태에서 정지를 유지한다는
동결된 안전 계약의 정상 결과다. 완료시키기 위해 empty frame, 안전 frame 수 또는 권한
조건을 완화하지 않았다.

## 4. 검증 결과

코드 동결 뒤 마지막 회귀는 실제 시간 측정과 일반 기능 시험을 분리했다.

| 검증 | 결과 |
|---|---:|
| 일반 기능·안전 시험 82개 파일, 10 process 분할 | `948 passed` |
| 실제 시간 영향을 받는 experiment runner 단독 실행 | `11 passed` |
| native short timing·source freeze 단독 실행 | `5 passed` |
| 최종 합계 | `964 passed` |
| Ruff | 통과 |
| `compileall` | 통과 |
| `git diff --check` | 오류 없음 |

병렬 실행 중 experiment runner 한 건이 CPU contention으로 `60.661 s > 60.000 s`를
기록했다. 이는 AGENTS.md 규칙대로 합격 근거에서 제외하고 CPU가 비어 있는 상태에서 단독
재실행했으며 통과했다. 기능 회귀로 발견된 기존 LEFT 우회 실패는 위 P0-C 범위 축소 뒤
단독·영향권·전체 회귀에서 모두 통과했다.

R7 C++ core와 timing 알고리즘 소스는 변경하지 않았다. 기존 `0/500` 50 ms 자격시험은
재실행하지 않았고, 이번에는 짧은 native parity/timing 시험만 단독 실행했다.

## 5. 변경 경계

변경한 범위는 다음뿐이다.

- 공개 실패 원인 기록과 manifest
- R5C 진단 runtime의 제동·정지 수명주기
- gate-confirmed distinct-frame 재출발 계수
- local window와 section executor의 catch-up 진단·조건부 active section 보존
- RPP/DWB adapter의 executor 상태 읽기 전용 노출
- 공개 회귀시험

다음은 수행하지 않았다.

- hidden 생성 또는 실행
- safety clearance·TTL·stop 확인 수·재출발 권한 완화
- DWB 후보·critic·native core 변경
- 제품 알고리즘 채택
- G1~G5 또는 경로 분석 7단계 결정

## 6. 다음 단계

이 결과는 공개 회귀 오류가 닫혔다는 뜻이다. 다음 선택은 새 hidden 연구를 시작할지 여부이며,
사용자 명시 승인 전에는 실행하지 않는다. 새 hidden을 시작하더라도 기존 hidden 결과를
재사용하거나 이번 공개 seed에 맞춰 파라미터를 조정해서는 안 된다.
