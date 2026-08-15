# R5-A v3 교차 경로 연결 교착 보정 결과

- 작업일: `2026-08-15`
- 범위: Python `simulation_only` R5-A static reference tracking
- 상태: **R5-A v3 clean public qualification PASS·receipt 생성**
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
- 매 tick 동일성 대신 section별 시작·끝·최소·최대 중심 위치를 기존 `0.05m` 안에서 비교한다.
- 좌우 signed mirror는 한쪽이 전진하고 다른 쪽이 같은 chassis yaw로 후진하므로 중심 경로만
  mirror 비교한다. 각 실행의 oriented footprint 안전은 shared gate가 독립 검사한다.
- travel direction을 보존하는 수평·수직 rigid relation만 footprint axis yaw를 기존 `0.08rad`
  안에서 비교한다.
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

### 첫 clean v3 실행

commit `5400000`의
`outputs/persistent-controller-public-20260815-r5a-v3-5400000`은 public `21/21`, ready 8개
RPP·DWB 종단, non-ready 무호출, serial/process parity, repeat determinism과 모든 case별 hard
safety를 통과했다. semantic hash는
`a793efc1c738603c18ac3b898a852218bd9efe07da4b5b139e909e805063369f`다.

최종 hard 판정만 좌우 signed relation의 DWB trajectory 4건으로 실패했다. 원인은 좌우 reference가
중심 경로는 mirror지만, 한쪽은 전진하고 다른 쪽은 **같은 `+90°` chassis yaw를 유지한 채 후진**하도록
동결돼 있는데 audit가 chassis yaw까지 mirror했기 때문이다. 위치 관계는 최대 약 `0.0173m`로 기존
`0.05m` 안이었다. 따라서 이 output은 그대로 실패 증거로 보존하고 receipt를 만들지 않았다.

관계 판정은 좌우 signed mirror에서 중심 geometry만 비교하고, travel direction을 보존하는
수평·수직 rigid relation에서는 footprint axis 비교를 유지하도록 보정했다. 보고 모듈 `9 passed`,
기존 clean output의 여섯 관계 재계산 `6/6 PASS`, Ruff·diff check를 통과했다.

commit `7810432`의 새 clean full 실행은 ready 8개 RPP·DWB 종단, non-ready 무호출,
hard·relation failure `0`, deadlock·gate override `0`, serial/process parity·repeat determinism
`PASS`로 qualification receipt를 생성했다. 최종 상세와 전달 ZIP은
[`R5-A v3 공개 qualification 결과`](r5a-v3-public-persistent-controller-qualification-result-2026-08-15.md)에
기록한다.

## 5. 다음 작업

R5-A 정적 reference tracking은 완료됐다. 다음 작업은 사용자 승인과 해당 단계 명세에 따라
R5-B Actor temporal execution 또는 보류된 Actor 출현 입력 문제를 다루는 것이다. R5-A output을
동적 Actor 성공이나 제품 DWB 채택 근거로 재사용하지 않는다.

하지 말 것: hidden, 기존 output 덮어쓰기, 제품 알고리즘 채택, G1~G5 결정, 제품 경로분석
7단계, 실제 사람 탑승 안전 주장.
