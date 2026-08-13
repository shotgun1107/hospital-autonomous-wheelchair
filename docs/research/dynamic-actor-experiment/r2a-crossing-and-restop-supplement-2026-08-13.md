# R2-A 보완 — 횡단 장애물 우회와 다중 위험 재정지

## 1. 문서 상태와 범위

- 작성일: `2026-08-13`
- 상태: 사용자 검토 대기, 구현 전
- 대상: R2-A exact ground-truth path lane의 미해결 2건
- 선행 증거: `witness-audit-public-20260813-r2-v2-4e4ba0f.zip`
- 구현·실험 재실행: 미수행
- R2-B 관측·prediction hard failure 2건: 이 문서에서 수정하지 않음
- R3 static 공간 oracle: 시작하지 않음
- hidden, 제품 알고리즘, `G1~G5`, 제품 경로분석 7단계: 시작하지 않음

이 문서는 다음 세 작업까지만 고정한다.

1. R2-A 미해결 사례의 보완 원칙
2. 횡단 장애물용 structured 탐색 범위
3. 다중 위험의 `정지 → 재개 → 재정지` ground-truth 검증 방식

코드 구조, 실제 함수명과 최종 schema version은 후속 구현 승인 뒤 정한다.

## 2. 현재 증거에서 확인된 문제

### 2.1 횡단 장애물 우회

대상 공개 결과:

```text
public id: legacy_mechanism-14-d796ebd8ba71
expected: local_detour_feasible
assessment: mismatched
reason: expected_feasible_pass_not_found
```

실제 결과는 다음과 같다.

- 기존 `PASS_LEFT`와 `PASS_RIGHT`는 모두 후보를 한 개도 만들지 않았다.
- 양쪽 종료 이유는 `no_eligible_same_direction_target`이었다.
- exact ground truth에서 안전한 `WAIT_AND_FOLLOW` witness는 존재했다.
- 따라서 이 결과는 우회 불가능 판정이 아니다.
- 원인은 현재 R2-PASS가 기준 경로와 같은 방향으로 움직이는 Actor의 추월만 정의하고,
  기준 경로를 가로지르는 Actor의 우회를 정의하지 않았기 때문이다.

### 2.2 다중 위험 재정지

대상 공개 결과:

```text
public id: legacy_mechanism-18-28f0a990202f
expected: dynamic_change_restop
assessment: not_fully_covered
reason: two_distinct_hazard_restops_not_demonstrated
```

두 Actor의 활성 구간은 서로 분리돼 있다.

```text
first:  0.0000000000s .. 2.0512820513s
second: 4.0512820513s .. 6.1025641026s
```

현재 선택 witness는 `start → wait → follow_reference → terminal_dwell` 구조다. 안전한 경로인
것은 확인됐지만 다음 순서를 하나의 witness에서 입증하지 못했다.

```text
첫 위험에서 실제 정지
→ 첫 위험 뒤 실제 이동 재개
→ 두 번째 위험에서 별도의 실제 재정지
```

기존 `_hazard_restop_count()` 형태의 사후 개수 세기만으로는 충분하지 않다. 정지 시점이 각
위험에 속하는지, 두 정지가 실제 이동으로 분리됐는지, 두 번째 정지가 첫 정지의 연장이 아닌지
순서대로 검증해야 한다.

## 3. R2-A 보완 공통 원칙

### 3.1 유지할 불변조건

- search 입력은 공개 `GOLDEN`·`DEVELOPMENT`의 label-free `WitnessWorldSnapshot`만 사용한다.
- expectation category, oracle 정답, 기존 수동 witness, controller 결과와 hidden 정보는 search
  입력에 넣지 않는다.
- Actor의 exact ground-truth trajectory는 offline R2-A search와 validator에서만 사용한다.
- 20 Hz 차체 운동학과 독립 200 Hz ground-truth validator를 유지한다.
- 실제 Actor 원과 oriented wheelchair footprint의 surface clearance `0.08m`를 완화하지 않는다.
- static·forbidden·allowed-region, 속도·가감속, terminal stopping과 provenance 검사를 유지한다.
- 검색 중 fast guard와 최종 validator를 같은 판정 함수로 만들지 않는다.
- Python wall-clock, worker 수와 완료 순서는 witness 의미와 합격 판정에 사용하지 않는다.
- 기존 R2 ZIP과 결과 hash를 수정하거나 성공 결과로 다시 표시하지 않는다.

### 3.2 결과 분리

보완 뒤에도 다음 결과를 합치지 않는다.

```text
횡단 Actor를 기다렸다가 직진함
!= 횡단 Actor가 막고 있는 동안 공간적으로 우회함

첫 위험부터 두 번째 위험까지 계속 정지함
!= 첫 정지 뒤 움직였다가 두 번째 위험에서 다시 정지함

R2-A ground-truth witness 존재
!= R2-B 관측으로 판단 가능
!= online controller가 실행 가능
```

### 3.3 기존 evidence 보존

- 기존 결과는 `R2-A pre-supplement regression`으로 그대로 보존한다.
- 보완 결과는 새 audit version, 새 output 경로와 새 content hash를 사용한다.
- 최초 대상은 위 두 legacy 공개 사례다.
- 두 사례가 통과한 뒤에만 기존 공개 19개에 대한 회귀 범위를 별도로 승인받는다.
- 이전 ZIP에 없는 135,360개 탈락 후보를 이 보완 때문에 다시 계산하지 않는다.

## 4. 횡단 장애물용 탐색 범위

## 4.1 새 witness 의미

같은 방향 추월과 횡단 우회를 구분하기 위해 횡단용 witness 종류를 별도로 둔다.

```text
CROSSING_BYPASS_LEFT
CROSSING_BYPASS_RIGHT
```

`LEFT/RIGHT`는 기준 경로 tangent에서 본 signed normal 방향이다. 이는 Actor의 왼쪽·오른쪽이나
통행 우선순위를 뜻하지 않는다.

기존 `PASS_LEFT/RIGHT`의 ordered overtake 의미는 변경하지 않는다. 횡단 Actor를 기존 PASS의
same-direction target 조건에 억지로 포함하지 않는다.

## 4.2 횡단 target의 label-free 정의

Actor는 다음 조건을 모두 만족할 때만 횡단 target 후보가 된다.

1. Actor trajectory가 하나의 non-zero straight reference segment의 안전 통로를 가로지른다.
2. Actor 속도의 reference normal 성분 절댓값이 명시적 최소값보다 크다.
3. 같은 방향 PASS target 조건은 만족하지 않는다.
4. exact trajectory에서 Actor 원과 기준 차선의 차체 footprint가 겹칠 수 있는 연속
   `blocking interval`이 존재한다.
5. blocking interval과 교차 progress는 expectation label이 아니라 geometry와 Actor
   trajectory에서 계산한다.
6. projection이 코너·self-intersection 때문에 모호하면 해당 structured 후보를 만들지 않는다.

속도 성분의 최소값과 projection tolerance는 구현 전 hashed search config에 명시한다. 현재
문서에서는 실제 제품 수치로 확정하지 않는다.

## 4.3 structured 기동 template

첫 보완 범위는 기존 PASS처럼 하나의 직선 reference segment 안에서 끝나는 다음 기동이다.

```text
optional FOLLOW_REFERENCE prefix
→ BRAKE_TO_STOP
→ optional PRE_DEPARTURE_HOLD
→ TURN_OUT
→ MOVE_LATERAL
→ BRAKE_TO_STOP
→ TURN_ALONG_REFERENCE
→ MOVE_ACROSS_CROSSING_STATION
→ BRAKE_TO_STOP
→ optional SIDE_HOLD
→ TURN_RETURN
→ MOVE_TO_REFERENCE
→ BRAKE_TO_STOP
→ ALIGN_REFERENCE
→ REJOIN_DWELL
```

포함 범위:

- 한 명의 횡단 target Actor
- target 외 Actor 전체에 대한 exact clearance 검사
- 좌·우 양쪽 후보
- Actor가 direct reference lane을 막는 동안 crossing station을 우회해 통과
- 우회 뒤 같은 segment의 원 reference로 재합류
- 실제 이탈, 우회 통과와 재합류 사건의 측정·선언·strict 재검증

제외 범위:

- 코너나 여러 reference segment에 걸친 우회
- 후진·3점 회전·곡선 최적화
- 여러 Actor를 동시에 우회해야만 하는 기동
- Actor 반응과 통행권 협상
- 관측·prediction을 이용한 online 판단
- 일반 pose-space 완전성 또는 공간 불가능 판정

## 4.4 후보 축

후보는 다음 축을 frozen 순서로 생성한다.

```text
target_actor_binding
side
reference_segment_index
crossing_station_progress
departure_progress
lateral_offset
release_tick
common_linear_target
common_angular_magnitude
side_hold_policy
```

규칙:

- crossing station은 Actor trajectory와 reference segment의 기하 교차에서 계산한다.
- departure는 crossing station보다 앞에 있어야 한다.
- lateral offset은 실제 reference distance가 `>0.10m`가 될 수 있어야 한다.
- release tick은 Actor exact blocking interval의 시작·끝과 20 Hz 인접 tick에서 생성한다.
- 좌·우 모두 끝까지 평가하고 첫 성공에서 중단하지 않는다.
- 후보 수와 resource limit은 manifest에 고정한다.
- limit 도달은 `RESOURCE_LIMIT`이며 우회 불가능이 아니다.

## 4.5 횡단 우회 사건의 strict 정의

최종 validator는 phase 문자열이 아니라 200 Hz 실제 pose와 Actor 상태로 다음 순서를 측정한다.

```text
departure
< active_blocking_bypass
< rejoin_start
<= rejoin_confirmed
```

각 사건의 의미:

- `departure`: reference distance가 처음 `>0.10m`가 된 시각
- `active_blocking_bypass`: target Actor가 direct lane의 blocking interval 안에 있을 때,
  로봇 중심의 reference progress가 crossing station 앞에서 뒤로 넘어간 최초 시각
- `rejoin_start`: bypass 뒤 reference distance와 heading이 재합류 범위에 처음 들어온 시각
- `rejoin_confirmed`: 위 조건을 연속 `0.50s` 만족한 시각

추가 hard 조건:

- departure부터 bypass까지 signed side가 witness kind와 계속 일치해야 한다.
- bypass 시점에도 reference distance가 `>0.10m`여야 한다.
- bypass는 target Actor가 active하기만 한 것이 아니라 direct lane을 실제로 막는 interval 안에서
  발생해야 한다.
- target Actor 소멸 뒤 직선으로 진행한 결과를 bypass로 세지 않는다.
- target 외 모든 Actor, static·forbidden·allowed-region clearance를 만족해야 한다.
- bypass 뒤 반대편으로 넘어가지 않고 원 reference에 재합류해야 한다.
- terminal pose는 실제 정지와 `0.50s` dwell을 만족해야 한다.

## 4.6 횡단 결과 taxonomy

```text
CROSSING_BYPASS_FOUND
WAIT_AND_FOLLOW_FOUND
NO_WITNESS_IN_CROSSING_TEMPLATE
RESOURCE_LIMIT
INVALID_INPUT
```

- `CROSSING_BYPASS_FOUND`와 `WAIT_AND_FOLLOW_FOUND`는 동시에 존재할 수 있다.
- 둘 중 어떤 행동을 실제로 선택할지는 R2-A가 결정하지 않는다.
- `NO_WITNESS_IN_CROSSING_TEMPLATE`은 R3 입력이며 `SPATIALLY_INFEASIBLE`이 아니다.
- 기존 `LOCAL_DETOUR_FEASIBLE` 기대와의 비교는 evaluator-only 단계에서 수행한다.

## 5. 다중 위험 `정지 → 재개 → 재정지` 검증 방식

## 5.1 R2-A에서 검증할 것

R2-A는 물리적인 시간 경로만 검증한다. online 안전 권한의 `stop_epoch`, 재개 승인과 11개 safe
frame은 R5~R6 범위이므로 R2-A witness에 주입하지 않는다.

대신 이름이 겹치지 않는 `KinematicStopInterval`을 사용한다.

```text
KinematicStopInterval
- stopped_from_s
- stopped_until_s
- pose
- preceding_motion_observed
- following_motion_observed
- bound_hazard_ids
```

실제 정지는 선속도와 각속도가 모두 정지 tolerance 안에 있는 연속 interval이다. 한 번의 긴
정지 interval을 두 개로 잘라 서로 다른 정지로 세지 않는다.

## 5.2 hazard interval 생성

각 Actor의 전체 active interval을 그대로 위험으로 간주하지 않는다. exact Actor trajectory가
reference의 차체 통행 band 또는 현재 witness의 terminal-stopping envelope를 침범하는 시각을
200 Hz와 raw Actor event 시각에서 계산해 `GroundTruthHazardInterval`을 만든다.

```text
GroundTruthHazardInterval
- hazard_id
- actor_binding_ids
- starts_at_s
- ends_at_s
- blocking_geometry_hash
```

- 같은 원인의 겹치는 interval은 합친다.
- 서로 분리된 위험은 별도 interval로 유지한다.
- 두 Actor가 동시에 하나의 blocking interval을 만들면 하나의 위험으로 결합할 수 있다.
- 두 위험 사이에 실제 이동 가능한 시간창이 없으면 `RESTOP_SEQUENCE_NOT_APPLICABLE` 또는
  structured-search limitation으로 남기며, 재개를 강제로 만들지 않는다.

## 5.3 필수 순서

두 개의 분리된 위험 `H1`, `H2`에 대해 strict validator는 다음 순서를 요구한다.

```text
H1 시작
→ S1: H1에 결박된 실제 정지 interval
→ H1 해소
→ R1: S1 뒤 실제 이동 재개
→ P1: H2 시작 전 의미 있는 후속 progress
→ H2 시작
→ S2: H2에 결박된 별도의 실제 정지 interval
```

`S1`과 `S2` 사이에는 반드시 실제 비정지 motion sample이 있어야 한다. 따라서 다음은 실패다.

- 처음부터 두 번째 위험이 끝날 때까지 계속 정지
- 하나의 긴 hold를 두 정지로 중복 계수
- 두 번째 위험 전에 이미 정지했지만 그 정지가 두 번째 위험과 무관함
- 첫 정지 뒤 속도 명령만 바뀌고 실제 pose·twist는 움직이지 않음
- 첫 정지 뒤 움직였지만 두 번째 위험 전에 의미 있는 progress가 없음

`P1`의 최소 progress는 기존 WAIT witness와 일관되게 reference progress 또는 실제 path length
`>=0.10m`를 첫 후보로 둔다. 최종 값은 구현 전 hashed validator version에서 고정한다.

## 5.4 두 가지 증거 수준

재정지 자체와 임무 회복을 분리한다.

```text
RESTOP_CORE_PROVEN
S1 → R1 → S2 순서와 두 위험 결박을 증명

RESTOP_AND_RECOVERY_PROVEN
RESTOP_CORE_PROVEN
+ H2 해소 뒤 두 번째 실제 재개
+ reference follow 또는 안전한 종단 상태
+ 필요한 경우 목표 도착과 terminal dwell
```

R2-A의 `DYNAMIC_CHANGE_RESTOP` 핵심 판정에는 `RESTOP_CORE_PROVEN`을 요구한다. 공개 mission
완료 증거로 사용할 때는 `RESTOP_AND_RECOVERY_PROVEN`을 별도로 요구한다. 핵심 재정지는
증명했지만 episode 시간이 부족해 최종 목표에 못 간 결과를 재정지 실패로 섞지 않는다.

## 5.5 다중 위험 search 범위

기존 하나의 초기 departure tick만 고르는 WAIT search로는 중간 재정지를 만들 수 없다. 후속
구현에서는 다음 event-anchored template를 별도로 둔다.

```text
WAIT_H1
→ FOLLOW_REFERENCE
→ BRAKE_FOR_H2
→ WAIT_H2
→ optional FOLLOW_REFERENCE
→ terminal state
```

후보 축:

```text
H1 release tick
linear target before H2
H2 braking anchor
H2 release tick
linear target after H2
terminal mode
```

각 tick은 exact ground-truth terminal-stopping guard를 거치고, 최종 candidate는 독립 200 Hz
validator가 다시 검사한다. expectation category나 `required_protective_stop_epochs`를 search에
입력하지 않는다.

## 5.6 strict failure code

최소 failure code는 다음처럼 분리한다.

```text
first_hazard_stop_missing
first_stop_not_bound_to_hazard
intermediate_resume_missing
intermediate_progress_insufficient
second_hazard_stop_missing
second_stop_not_distinct
hazard_order_invalid
continuous_hold_misclassified_as_restop
post_second_hazard_recovery_missing
resource_limit
invalid_provenance
```

`post_second_hazard_recovery_missing`은 `RESTOP_CORE_PROVEN`을 취소하지 않고 full recovery만
실패시킨다.

## 5.7 적대 검증 사례

후속 구현 전 다음 시험을 고정해야 한다.

1. H1 정지 → 실제 이동 → H2 정지: core 통과
2. H1부터 H2까지 계속 정지: core 실패
3. 하나의 hold를 두 hazard에 중복 결박: 실패
4. H1 정지 뒤 zero command만 바뀌고 실제 이동 없음: 실패
5. H1 뒤 `0.10m` 미만 움직인 뒤 H2 정지: progress 실패
6. H2 전에 멈췄지만 H2 위험과 시간·기하 결박이 없음: 실패
7. H1/H2 순서를 바꿈: 실패
8. core 뒤 H2 해소 후 재개 없음: core 통과, full recovery 실패
9. H2 해소 뒤 재개·목표 도착·dwell: full recovery 통과
10. Actor active interval은 둘이지만 실제 blocking interval은 하나: 두 정지 요구 금지
11. 두 위험 사이 재개 가능 시간창이 없음: 적용 불가 또는 limitation, false pass 금지
12. 같은 input/config의 결과와 event hash 결정론

## 6. 1~3단계 완료 산출물과 다음 gate

이 문서로 완료된 범위:

- R2-A 미해결 2건의 원인과 보완 경계
- 같은 방향 추월과 분리된 횡단 장애물 우회 탐색 범위
- 다중 위험의 ordered `정지 → 재개 → 재정지` ground-truth 검증 방식

아직 완료되지 않은 범위:

- 새 자료형·검색기·validator 구현
- 표적·공개 회귀시험
- 기존 R2 결과 재실행 또는 교체
- R2-B Actor 출현 관측 문제 해결
- R3 static 공간 oracle

후속 구현을 시작하려면 사용자가 이 보완 명세를 승인하고 별도로 구현 시작을 지시해야 한다.
구현 순서는 다음 후보로만 남긴다.

```text
계약·failure code 단위시험
→ crossing target·event validator
→ crossing structured search
→ multi-hazard ordered validator
→ multi-hazard event-anchored search
→ legacy 2건 표적시험
→ 독립 읽기 전용 감사
→ 사용자 승인 뒤 공개 회귀 범위 결정
```
