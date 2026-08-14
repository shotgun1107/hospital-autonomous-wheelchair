# R5-A v2 구간 제한 후진 구현 중간 결과

- 작업일: `2026-08-15`
- 범위: Python `simulation_only` R5-A static reference tracking
- 상태: **부분 성공 — RPP 완료, DWB 후진 진입 성공 후 deadlock 미해결**
- hidden: 미사용
- 제품 controller 채택·G1~G5·제품 경로분석 7단계: 미수행

## 1. 구현한 것

- 공통 reference executor가 R4 v2의 `FORWARD|REVERSE|NONE`을 직접 소비한다.
- `FORWARD↔REVERSE` 전환 전에 제한 감속과 실제 정지 3 tick을 확인한다.
- `NONE` connector를 임의 translation으로 실행하지 않는다.
- RPP는 `FORWARD`에서 양수, `REVERSE`에서 `-0.10~0m/s`만 생성한다.
- source-derived DWB generator도 active section 밖의 부호 후보를 만들지 않는다.
- DWB의 PathAlign·GoalAlign은 reverse에서 rear projection을 사용한다.
- controller 결과 뒤 공통 pipeline이 방향·후진 속도 상한을 다시 검사하고 위반 시 정지한다.
- v2 reporting은 tick별 active travel direction과 signed command를 기록하고, reverse section에서
  실제 음수 이동이 없으면 hard failure로 남긴다.

## 2. 검증 결과

- 공통 pipeline: `11 passed`
- reference executor·RPP·DWB·공통 safety 영향권: 기능 기준 `160 passed`, 오류 문구 회귀 수정 후
  composition `29 passed`
- reporting: `7 passed`
- reverse alignment critic·persistent DWB: `24 passed`
- Ruff: 변경 영향권 통과
- Windows 기본 pytest 임시 폴더 권한 오류는 저장소 내부 `--basetemp`로 재실행해 해소했다.

전체 pytest와 clean full public 21-case qualification은 아직 실행하지 않았다.

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

- section-bound 음수 후보 생성과 실제 후진은 확인했다.
- 잘못된 방향 회전은 reverse rear-alignment 활성화로 고쳤다.
- 그러나 reverse section 종점 약 `5.7cm` 앞에서 정지해 `planner_deadlock`이 발생했다.
- 대표 partial 결과: tick `341`, 음수 후진 `45 tick`, 최소 선속도
  `-0.0116666667m/s`, 최대 추적오차 `0.0480769401m`.
- gate override/rejection: `0/0`이므로 shared gate가 막은 것은 아니다.
- deadlock 판정을 제외한 650-tick 진단에서도 `v=0,w=0`으로 남아 실제 미완료가 확인됐다.

현재 유력 원인은 짧은 최종 reverse translation에서 DWB의 full-terminal
`RotateToGoalCritic`과 공통 executor의 section stop 책임이 중복되는 것이다. reverse 방향 정렬 뒤
가속 후보가 terminal tolerance에 진입하면 DWB 내부 critic이 탈락시키는지 후보별 reason으로
확정해야 한다. deadlock threshold나 safety margin을 완화해 성공으로 만들면 안 된다.

## 4. 다음 작업

1. `wide-straight-left` reverse 종점의 DWB 후보별 rejection reason을 기록한다.
2. `RotateToGoalCritic`이 가속 reverse 후보를 거부하는지 표적시험으로 확정한다.
3. 공통 executor가 intermediate/final section stop을 소유한다는 기존 명세와 맞춰 책임 중복만 최소
   수정한다. critic weight·tie-break·안전 threshold·R4 reference는 바꾸지 않는다.
4. 같은 대표 공개 case에서 DWB가 reverse→planned stop→rotation→completion까지 가는지 재검증한다.
5. 통과 뒤 영향권·전체 pytest·Ruff를 실행하고, clean commit에서만 21-case public
   qualification과 receipt를 검토한다.

하지 말 것: hidden, 기존 output 덮어쓰기, 제품 알고리즘 채택, G1~G5 결정, 제품 경로분석 7단계,
실제 사람 탑승 안전 주장.
