# R5-A v2 구간 제한 후진 구현 중간 결과

- 작업일: `2026-08-15`
- 범위: Python `simulation_only` R5-A static reference tracking
- 상태: **대표 case 성공 — RPP·DWB 실제 후진과 종단 완료, 전체 21-case 자격 미실행**
- hidden: 미사용
- 제품 controller 채택·G1~G5·제품 경로분석 7단계: 미수행

## 1. 구현한 것

- 공통 reference executor가 R4 v2의 `FORWARD|REVERSE|NONE`을 직접 소비한다.
- `FORWARD↔REVERSE` 전환 전에 제한 감속과 실제 정지 3 tick을 확인한다.
- `NONE` connector를 임의 translation으로 실행하지 않는다.
- RPP는 `FORWARD`에서 양수, `REVERSE`에서 `-0.10~0m/s`만 생성한다.
- source-derived DWB generator도 active section 밖의 부호 후보를 만들지 않는다.
- DWB의 GoalAlign은 reverse에서 rear projection을 사용하고, PathAlign은 짧은 section 종점의
  upstream near-goal 안정화를 유지한다.
- self-near reference에서 완료한 과거 edge가 수 mm 더 가깝더라도 기존 `0.05m` 진행 회귀 한계
  안의 비회귀 후보를 우선해 현재 signed section을 유지한다.
- controller 결과 뒤 공통 pipeline이 방향·후진 속도 상한을 다시 검사하고 위반 시 정지한다.
- v2 reporting은 tick별 active travel direction과 signed command를 기록하고, reverse section에서
  실제 음수 이동이 없으면 hard failure로 남긴다.

## 2. 검증 결과

- 이번 수정 직접·영향권: `27 passed`와 `99 passed`
- 전체 Python 회귀: `859 passed, 3 skipped`, 실패 `0`
- skip 3건은 선택적 C++ DWA core가 현재 빌드되지 않은 경우이며 Python fallback 범위는 통과했다.
- Ruff·compileall·`git diff --check`: 통과
- Windows 기본 pytest 임시 폴더 권한 오류는 저장소 내부 `--basetemp`로 재실행해 해소했다.

clean full public 21-case qualification은 아직 실행하지 않았다.

## 3. 공개 대표 case 결과

입력: `wide-straight-left`, fresh-empty, 동일 R4 v2 reference, 공통 shared gate.

### RPP

- 상태: `COMPLETED`
- completion tick: `296`
- tick 수: `297`
- 실제 음수 후진: `12 tick`
- 최소 선속도: `-0.05m/s`
- 최대 추적오차: `0.0351668523m`
- gate override/rejection: `0/0`
- hard failure: `0`

### DWB

- 상태: `COMPLETED`
- completion tick: `378`
- tick 수: `379`
- 실제 음수 후진: `24 tick`
- 최소 선속도: `-0.03125m/s`
- 방향 전환: `1`
- 최대 추적오차: `0.0480769401m`
- gate override/rejection: `0/0`
- planner deadlock/hard failure: `0/0`

## 4. 원인과 최소 수정

첫 진단의 `RotateToGoalCritic` 추정은 후보 전체 점검으로 기각했다. 이전 deadlock pose에서
nonzero reverse 후보 `186`개는 모두 안전상 허용됐고 `RotateToGoal` 점수도 `0`이었다. 실제 원인은
짧은 reverse 종점에서 PathAlign grid cost가 안전한 음수 후보보다 `v=0`을 더 낮게 평가한
local minimum이었다. GoalAlign의 reverse rear projection은 유지하고 PathAlign만 기존 upstream
near-goal 비활성 규칙을 적용해 이 정체를 닫았다. critic weight·tie-break·후보 수·안전 기준은
바꾸지 않았다.

그 뒤 실제 후진 중 reference cursor가 과거 forward edge로 약 `9.6cm` 튀는 별도 결함이 드러났다.
실패 pose에서 과거 edge와 현재 reverse edge의 공간거리 차이는 약 `0.00039m`뿐이었다. 기존
`0.05m` 회귀 한계를 새 수치 없이 재사용해, 과거 투영이 그 한계를 넘길 때만 같은 공간 범위의
비회귀 후보를 우선한다. 실제로 크게 뒤로 이동한 입력은 기존처럼
`cursor_regression_exceeded`로 닫힌다. reverse edge의 chassis heading도 이동 tangent의 반대
방향으로 비교한다.

대표 재실행 output은
`outputs/persistent-controller-public-20260815-r5a-v2-representative-fix3-dirty`에 남겼다.
dirty·1-case 실행이므로 `PARTIAL_REPORT_ONLY`이며 receipt는 만들지 않았다.

## 5. 다음 작업

1. 현재 구현과 문서를 commit·push한다.
2. clean commit에서 R5-A public 21-case를 새 output 경로로 한 번 실행한다.
3. ready 8개 paired 완료, signed 방향·정지·reverse sweep, hard failure·deadlock `0`, repeat·
   serial/process parity를 모두 확인한다.
4. 모든 자격이 통과할 때만 R5-A receipt를 생성한다. 실패하면 output을 보존하고 해당 원인만
   다시 진단한다.

하지 말 것: hidden, 기존 output 덮어쓰기, 제품 알고리즘 채택, G1~G5 결정, 제품 경로분석 7단계,
실제 사람 탑승 안전 주장.
