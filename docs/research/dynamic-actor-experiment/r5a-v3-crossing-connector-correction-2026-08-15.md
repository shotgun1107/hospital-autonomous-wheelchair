# R5-A v3 교차 경로 연결 교착 보정 결과

- 작업일: `2026-08-15`
- 범위: Python `simulation_only` R5-A static reference tracking
- 상태: **좌·우 교차 DWB 종단 통과, 전체 21-case clean 재qualification 대기**
- hidden: 미사용
- 제품 controller 채택·G1~G5·제품 경로분석 7단계: 미수행

## 1. 직전 clean 실행 결과

commit `7ef755d`의
`outputs/persistent-controller-public-20260815-r5a-v2-7ef755d`는 public `21/21`,
serial/process parity와 repeat determinism을 끝까지 실행했지만 hard `FAIL`로 receipt를 만들지
않았다. ready 8개 중 wide·vertical 6개는 RPP·DWB 모두 완료했고, 교차 2개와 관계 검사가
실패했다.

확인된 실패는 다음 두 묶음이다.

1. `crossing-static-left/right`의 마지막 translation 뒤에 약 `0.02236068m`의 stopped
   `NONE` connector가 있었다. connector는 비구동인데 executor가 앞 translation의 일반
   `0.05m` 완료를 먼저 인정해 connector 끝이 tolerance 밖에 남거나, 반대로 지나치게
   보수적인 `0.05m - connector displacement` 조건과 DWB의 기존 `0.05m` goal window가
   서로 교착했다.
2. relation audit가 좌우 reference의 실제 중심 경로가 아니라 매 tick pose·yaw·command까지
   정확히 mirror여야 한다고 요구했다. forward/reverse 속도 상한과 section별 tick 수가 다른
   signed reference에서는 성립하지 않는 판정이었다.

원 output과 실패 semantic hash는 변경하지 않았다.

## 2. 최소 보정

### 2.1 stopped connector

- moving `NONE` connector는 계속 비구동이며 직접 command를 만들지 않는다.
- 감속에는 기존 `0.05m`에서 connector displacement를 뺀 보수적 scalar를 유지한다.
- 완료 판정은 새 tolerance를 만들지 않고, 현재 chassis center가 앞 translation 끝과 이어지는
  stopped connector 끝 각각의 기존 `0.05m` 안에 있는지 직접 검사한다.
- RPP stop limiting도 같은 보수적 scalar를 사용한다.

### 2.2 DWB 최종 짧은 전진

- connector가 completion budget을 줄이는 forward section에서만 exact-score tie의 linear
  candidate block을 큰 전진 속도부터 평가한다.
- candidate 집합·개수, angular 순서, critic score, 안전 constraint와 strict lower-score core는
  바꾸지 않는다.
- heading이 아직 맞지 않을 때만 GoalAlign·PathAlign을 유지한다. heading이 기존 `0.08rad`
  안에 들어오면 upstream near-goal 비활성 동작으로 돌아가, forward projection score가 빠른
  전진을 계속 억제하지 않게 한다.

### 2.3 relation audit

- status와 section sequence는 그대로 exact 비교한다.
- 매 tick 동일성 대신 section별 시작·끝·최소·최대 중심 위치를 기존 `0.05m`, footprint axis
  yaw를 기존 `0.08rad` 안에서 비교한다.
- 5cm를 넘는 합성 geometry 변조는 계속 실패한다.

## 3. 중간 실패와 원인 추적

수정 과정의 실패도 보존한다.

1. connector budget만 적용했을 때 DWB left는 약 `x=4.297m`에서 최저속을 반복했다.
2. align critic을 끝까지 유지하자 yaw는 정렬됐지만 약 `x=4.306m`에서 다시 최저속을 반복했다.
3. heading 정렬 뒤 align critic을 끄자 `x=4.354m`까지 갔지만, executor의 보수 scalar
   `0.02763932m`와 DWB의 기존 terminal `0.05m`가 충돌해 영명령 교착했다.
4. 두 알려진 endpoint에 기존 `0.05m`를 각각 직접 적용한 뒤 교착이 해소됐다.

deadlock threshold, safety clearance, candidate 수, critic weight, 일반 DWB core tie-break와
shared gate는 완화하지 않았다.

## 4. 현재 검증 결과

### crossing-static-left DWB

- 상태: `COMPLETED`
- tick 수: `667`, completion tick `666`
- deadlock: `0`
- gate override: `0`
- hard failure: `0`
- 최종 pose: `(4.3523698786, 2.3608515845, -0.0128000270)`

### crossing-static-right DWB

- 상태: `COMPLETED`
- tick 수: `655`, completion tick `654`
- deadlock: `0`
- gate override: `0`
- hard failure: `0`
- 최종 pose: `(4.3601483097, 2.3213904356, -0.0325761164)`

### 자동 시험

- persistent 영향권: `66 passed`
- 전체 Python 회귀: `864 passed, 3 skipped`
- skip 3건: 빌드하지 않은 선택적 C++ DWA core
- Ruff·compileall·`git diff --check`: 통과

위 좌·우 실행은 dirty source에서 수행한 표적 기능시험이므로 qualification receipt가 아니다.

## 5. 다음 작업

1. 코드·시험·이 문서를 commit·push한다.
2. clean commit에서 새 output 경로로 public 21-case를 한 번 실행한다.
3. ready 8개 paired 완료, non-ready 무호출, hard failure·deadlock·gate override `0`, 관계 오류
   `0`, repeat·serial/process parity를 확인한다.
4. 모두 통과할 때만 receipt와 최종 결과 문서를 만든다.

하지 말 것: hidden, 기존 output 덮어쓰기, 제품 알고리즘 채택, G1~G5 결정, 제품 경로분석
7단계, 실제 사람 탑승 안전 주장.
