# R3 Bounded 공간 Oracle 상세 명세

- 문서 상태: 사용자 개인 연구 입력, core L1 구현·public qualification 미완료
- 작성일: `2026-08-14`
- 적용 범위: Python `simulation_only`, 공개 synthetic map, 실제 사람 미탑승
- 선행 단계: R2-A ground-truth 시간 경로 연구
- 후속 단계: R4 방향 있는 지역 reference 계약
- 제외: 제품 알고리즘 채택, online 이동 허가, 카메라·관측 통합, 실제 차체 안전 증거

## 1. 목적

R2-A의 structured search가 경로를 찾지 못했을 때 다음 두 원인이 섞여 있다.

```text
현재 search template가 해당 모양의 경로를 표현하지 못함
실제로 휠체어 차체가 정적 공간을 통과할 수 없음
```

R3는 Actor 시간, observation, prediction과 controller를 제거하고 다음 질문만 독립적으로
판정한다.

> 동결된 가상 휠체어 footprint가 지정된 bounded 정적 공간 안에서 출발 pose부터 목표
> 재합류 pose·heading까지 collision-free로 연결될 수 있는가?

R3의 결과는 offline 공간 가능성 근거다. 반환 경로가 존재해도 실제 이동 명령, 재개 승인,
제품 local planner 채택이나 실제 사람 안전을 의미하지 않는다.

## 2. R1~R7에서의 위치

```text
R2-A ground-truth 시간 witness
  ├─ witness found + 독립 validator pass → R3 positive 회귀 및 R4 후보 근거
  └─ structured no-witness/resource → R3 공간 원인 분리 입력

R3 bounded 정적 공간 oracle
  ├─ SPATIALLY_FEASIBLE → R4 reference 표현 후보
  ├─ SPATIALLY_INFEASIBLE → 현재 동결 공간·격자에서는 controller 비교 제외
  ├─ RESOURCE_LIMIT → 결론 보류, 불가능으로 해석 금지
  └─ INVALID_INPUT → provenance·계약 수정 전 결론 금지
```

R2-B의 `ideal_capsule_ground_truth_miss`와 fresh EMPTY 문제는 R3 입력에 observation을 넣지
않으므로 R3 자체를 막지 않는다. 다만 R2-B 통과 전에는 perception-integrated R6, hidden,
제품 안전 주장을 할 수 없다.

## 3. 증거 범위와 비목표

### 3.1 R3가 증명할 수 있는 것

- 동결된 static grid, forbidden·allowed region과 가상 footprint 조건에서 pose·heading path가
  존재하는지
- 반환 path가 정적 장애물·금지영역·허용영역 경계와 최소 `0.08m` clearance를 만족하는지
- 지정한 side와 재합류 pose·heading을 만족하는지
- 동결한 finite lattice 전체를 완전히 소진한 음성 결과인지, resource limit에 막힌 것인지

### 3.2 R3가 증명하지 않는 것

- 움직이는 Actor와의 시간 충돌 회피
- 카메라 FOV, 가림, 검출, 추적과 prediction 정확도
- controller가 경로를 실제로 추종할 수 있는지
- stop epoch, 재개 승인, shared safety gate의 online 상태 전이
- Python wall-clock 또는 실제 20Hz 실시간 충족
- 실제 휠체어 motion primitive, 모터 성능이나 승차감
- 제품 알고리즘 우열, `G1~G5`, 경로분석 7단계 결정
- 실제 사람 탑승 안전성 또는 의료기기 인증

## 4. 용어

### `bounded spatial oracle`

명시된 search region, 해상도, heading bin, primitive와 resource 계약 안에서 pose·heading graph를
결정론적으로 탐색하는 offline 연구 도구다.

### `lattice-complete negative`

동결된 bounded lattice의 reachable state를 resource limit 없이 모두 소진한 뒤에도 목표 상태를
찾지 못한 결과다. 이는 연속 공간 전체의 수학적 불가능이 아니라 **현재 동결 lattice 안의
공간 불가능**을 의미한다.

### `independent validator`

search의 occupancy shortcut이나 node validity cache를 신뢰하지 않고, 반환된 primitive와 pose를
원본 static geometry와 oriented footprint로 다시 검사하는 별도 판정기다.

### `rejoin goal`

R4에 전달할 원 reference 상의 목표 pose와 heading이다. 단순히 goal cell에 중심점이 들어오는
것이 아니라 위치·heading·최종 정지 조건을 각각 만족해야 한다.

## 5. 입력 계약

R3 top-level API는 corpus episode, expectation category, oracle label, Actor ground truth 또는
hidden split을 직접 받지 않는다. 다음 정본 입력만 받는다.

```text
BoundedSpatialOracleRequest
  schema_version
  map_id
  map_revision
  mission_revision
  static_grid
  forbidden_cells
  allowed_region
  vehicle_profile
  start_pose
  rejoin_goal
  reference_segment
  maneuver_side
  search_region
  lattice_config
  source_projection_hash
  request_content_hash
```

### 5.1 정적 지도와 provenance

- `static_grid`는 immutable `GridMap`과 같은 좌표계·단위·resolution을 사용한다.
- `forbidden_cells`는 물리 occupancy와 별도 의미를 유지하지만 탐색·검증에서는 둘 다 통과
  금지다.
- `allowed_region` 밖은 지도 내부 free cell이어도 통과 금지다.
- `map_id`, map·mission revision, grid content hash와 projection hash를 결과에 그대로 결박한다.
- non-finite origin·resolution, 빈 occupancy, 범위 밖 forbidden cell과 hash 불일치는
  `INVALID_INPUT`이다.

### 5.2 가상 차체 프로필

R3 v1은 기존 `VIRTUAL_DOLL_WHEELCHAIR_V0_1`만 사용한다.

```text
profile_id:             virtual_doll_wheelchair_v0_1
simulation_only:        true
collision_width:        0.36m
collision_length:       0.44m
differential_drive:     true
in_place_rotation:      true
minimum_clearance:      0.08m
max_forward_speed:      0.30m/s   # 공간 oracle 비용·자격에는 사용하지 않음
max_reverse_speed:      0.10m/s   # 공간 primitive 방향 허용 근거만 제공
max_angular_speed:      0.80rad/s # 공간 oracle timing 자격에는 사용하지 않음
```

body 크기가 아니라 collision footprint를 사용한다. 해당 값은 연구용 가상 프로필이며 실제
차체·모터 수치로 확정하지 않는다. profile ID 또는 semantic hash가 다르면 묵시 변환하지 않고
`INVALID_INPUT`으로 처리한다.

### 5.3 시작과 재합류 목표

```text
SpatialRejoinGoal
  pose
  position_tolerance_m = 0.05
  heading_tolerance_rad = 10deg
  require_stopped = true
  minimum_side_excursion_m = 0.10
```

`require_stopped`는 R3의 추상 terminal state에 선속도·각속도 `0` 표식을 요구한다는 뜻이다.
R3에는 시간·가감속·actuator가 없으므로 실제 제동 완료를 증명하지 않는다. 실제 정지 가능성과
정지 완료는 R4 이후의 시간 경로 및 공통 safety gate가 별도로 검증한다.

- start와 goal footprint가 처음부터 static·forbidden·allowed 경계를 위반하면
  `SPATIALLY_INFEASIBLE`의 명시적 termination reason으로 기록한다. pose 자체가 non-finite이거나
  map frame이 다르면 `INVALID_INPUT`이다.
- start와 goal이 tolerance를 이미 만족해도 LEFT/RIGHT 공간 기동은 최소 side excursion을
  증명해야 하므로 reference 위 1-point path를 `SPATIALLY_FEASIBLE`로 허용하지 않는다.
- goal heading은 request의 명시적 `rejoin_goal.pose.yaw`를 사용한다.
- LEFT/RIGHT request는 departure 뒤 지정 side의 signed lateral offset이 한 번 이상
  `minimum_side_excursion_m`에 도달해야 한다. 이를 만족하지 않은 직진 path는 feasible이 아니다.
- `UNSPECIFIED`은 LEFT·RIGHT lane 각각에서 같은 최소 이탈을 요구한다. Actor를 제거했다는 이유로
  reference 직진 path를 좌·우 우회 가능성 근거로 사용하지 않는다.

### 5.4 기동 side

```text
ManeuverSide
  LEFT
  RIGHT
  UNSPECIFIED
```

- `reference_segment`는 map frame의 start·end pose와 누적 progress 범위를 가진다. 길이 0,
  non-finite 또는 search region과 분리된 segment는 `INVALID_INPUT`이다.
- LEFT/RIGHT는 해당 reference segment tangent와 signed lateral offset으로 정의한다.
- side가 지정되면 departure 뒤 rejoin 전까지 반대 side로 넘어가는 state를 생성하지 않는다.
- `UNSPECIFIED`은 좌·우를 별도 search lane으로 실행한 뒤 같은 규칙으로 결과를 합친다.
- corpus의 expectation category로 side를 결정하지 않는다.

### 5.5 bounded search region

`SpatialSearchRegion`은 map frame의 closed polygon 또는 grid-cell mask로 정본화한다.

- 모든 state footprint는 region 안에 있어야 한다.
- region은 static map을 넘어갈 수 없다.
- start와 goal을 모두 포함해야 한다.
- R2 projection이 region을 만들 때 Actor 위치·category를 정답 누출로 사용하지 않는다.
- region semantic hash를 request와 result에 포함한다.
- 너무 작은 region에서 no-path가 나와도 그 결과는 해당 region에만 한정된다.

## 6. R3 v1 동결 lattice

R3는 제품 State Lattice planner를 채택하는 단계가 아니다. v1은 finite 공간 가능성을
검사하기 위한 다음 **가상 연구 lattice**를 사용한다.

```text
grid_resolution_m:          source static grid resolution, 현재 public 0.02m
axis_translation_cells:     5
diagonal_translation_cells: 4
heading_bin_count:          8
heading_quantum:            45deg
allow_forward:              true
allow_reverse:              true
allow_in_place_rotation:    true
allow_combined_arc:         false in v1
```

### 6.1 state

```text
SpatialLatticeState
  x_cell
  y_cell
  heading_index
  required_excursion_reached
```

- 위치 index는 source grid cell center를 기준으로 한다.
- heading index는 `[0, 2π)`를 8개 bin으로 정규화한다.
- `required_excursion_reached`는 지정 side offset이 `minimum_side_excursion_m` 이상이 된 뒤에만
  `true`가 되고 같은 search lane 안에서 다시 `false`로 돌아가지 않는다. 이 phase bit가 다른
  state는 pose·heading이 같아도 별도 state다.
- start와 goal이 lattice center에 정확히 놓이지 않아도 exact anchor connector로 해당 cell과
  Chebyshev 1-cell 이웃의 8 heading state를 결정론적 cell·heading 순서로 평가한다.
- connector도 일반 primitive와 같은 swept-footprint validator를 통과해야 한다.
- anchor connector 후보도 `generated_edges`와 resource limit에 포함하며, 안전한 connector가
  하나도 없으면 해당 bounded lattice lane은 exhaustive no-entry/no-exit로 종료한다.
- 동일 state의 tie는 `(x_cell, y_cell, heading_index)` 순으로 고정한다.

### 6.2 primitive

v1 primitive는 다음 4개 계열만 허용한다.

```text
FORWARD_ONE_TRANSLATION
REVERSE_ONE_TRANSLATION
ROTATE_LEFT_45
ROTATE_RIGHT_45
```

- translation은 현재 heading을 일정하게 유지하고 아래 정수 cell offset 표를 사용한다.
- rotation은 위치를 고정하고 정확히 한 heading bin(`45deg`)만 회전한다.
- forward·reverse와 회전을 조합한 곡선 primitive는 v1에서 사용하지 않는다.
- start·goal connector는 별도 `ANCHOR_CONNECTOR`로 기록한다.
- 이 primitive는 실제 차체 명령이나 승차감 모델이 아니다.

heading별 forward offset은 다음 정본 표를 사용한다. reverse는 부호를 반전한다.

```text
heading index 0..7 forward (dx,dy)
0:(5,0)   1:(4,4)   2:(0,5)   3:(-4,4)
4:(-5,0)  5:(-4,-4) 6:(0,-5)  7:(4,-4)
```

축 방향 실제 translation 길이는 `0.10m`, 대각선은 약 `0.113137m`이며 cost와 swept 검사는
정수 offset의 실제 metric 길이를 사용한다. 이동 벡터와 차체 heading은 정확히 평행하므로
differential-drive에 없는 횡미끄러짐을 도입하지 않는다. 모든 state가 source grid cell center에
남고 좌우·상하 mirror가 보존된다. 이를 임의로 항상 `0.10m`라고 기록하지 않는다.

곡선 primitive가 없다는 한계는 `orthogonal_lattice_motion_only`로 기록한다. v1에서
`SPATIALLY_INFEASIBLE`이 나온 사례는 richer lattice와 대조하기 전 제품 불가능으로 확대하지
않는다.

### 6.3 deterministic 비용과 tie-break

검색은 A* 또는 동일한 deterministic best-first를 사용한다.

```text
translation cost = 이동거리
reverse multiplier = 1.25
rotation cost = collision circumscribed radius × |Δyaw|
anchor connector cost = connector translation + rotation equivalent
heuristic = Euclidean position distance
```

heuristic은 실제 남은 비용을 초과하지 않도록 rotation과 장애물을 무시한다. 우선순위가 같으면
다음 순서를 사용한다.

```text
1. f_cost 오름차순
2. h_cost 오름차순
3. reverse distance 오름차순
4. rotation count 오름차순
5. primitive sequence lexical order
6. state tuple lexical order
```

Python set·dict iteration이나 process 완료 순서가 선택 결과를 바꾸면 실패다.

## 7. 공간 안전 계약

### 7.1 pose 안전

각 pose에서 다음을 모두 만족해야 한다.

```text
oriented collision footprint가 map 안에 있음
AND physical occupancy와 겹치지 않음
AND forbidden cell과 겹치지 않음
AND allowed region 밖으로 나가지 않음
AND static·forbidden·allowed boundary surface clearance >= 0.08m
```

중심 cell만 free인 것은 충분하지 않다. `0.36m × 0.44m` oriented rectangle 전체를 검사한다.

### 7.2 primitive swept 안전

두 endpoint만 안전한 primitive는 허용하지 않는다. primitive 전체 이동 동안 swept footprint를
검사한다.

- translation은 start와 end footprint의 swept polygon을 검사한다.
- in-place rotation은 rectangle corner가 그리는 회전 sweep을 보수적으로 검사한다.
- analytic sweep이 없으면 adaptive subdivision을 사용하되, 인접 sample 사이 footprint 최대
  이동량 기반 보수 팽창으로 sample 사이 충돌 누락이 없음을 증명한다.
- 기본 subdivision 상한은 translation `0.005m`, rotation `0.5deg`다.
- raw map·allowed boundary event가 regular sample 사이에 있으면 exact event 위치를 추가한다.

`0.005m`는 validator 공간 해상도이며 Python wall-clock deadline이 아니다.

### 7.3 최소 clearance

- physical, forbidden, allowed boundary clearance를 별도 기록한다.
- 최종 `minimum_clearance_m`은 세 값의 최솟값이다.
- 접촉 또는 `0.08m` 미만은 hard reject다.
- tolerance는 부동소수점 비교용 `1e-9m`만 허용한다.

## 8. 검색·검증 분리

### 8.1 search-side fast rejection

search는 cell occupancy, footprint cache, primitive cache와 conservative analytic prune을 사용할
수 있다. 근사 기하만으로 최종 path를 승인할 수 없다.

### 8.2 independent final validator

`validate_spatial_oracle_path(request, result_path)`는 다음을 독립적으로 다시 계산한다.

- request·map·profile·config semantic hash
- start와 goal pose·heading
- primitive endpoint 연속성
- 각 primitive의 허용 종류와 길이·각도
- swept oriented footprint
- static·forbidden·allowed clearance
- side 유지와 rejoin
- path length, reverse length, rotation count

validator는 search의 closed set, parent pointer, collision cache와 success flag를 입력으로 받지
않는다. validator 실패 path는 `SPATIALLY_FEASIBLE`로 반환할 수 없다.

## 9. 결과 taxonomy

```text
SpatialOracleStatus
  SPATIALLY_FEASIBLE
  SPATIALLY_INFEASIBLE
  RESOURCE_LIMIT
  INVALID_INPUT
```

### `SPATIALLY_FEASIBLE`

- start부터 rejoin goal까지 path 존재
- independent validator pass
- minimum clearance `>=0.08m`
- path·primitive·validation hash 존재

### `SPATIALLY_INFEASIBLE`

```text
start_footprint_unsafe
goal_footprint_unsafe
bounded_lattice_exhausted
analytic_cross_section_blocked
```

`bounded_lattice_exhausted`는 frozen finite graph를 resource limit 없이 끝까지 소진한 경우에만
허용한다. timeout, Python 예외나 worker 종료를 infeasible로 바꾸지 않는다.

`analytic_cross_section_blocked`는 문·직선 통로 단면 전체가 footprint+clearance보다 좁다는
별도 보수 proof가 있을 때만 사용한다. 단순 corridor width로 코너 회전까지 일반화하지 않는다.

### `RESOURCE_LIMIT`

```text
max_expanded_states = 250,000
max_generated_edges = 2,000,000
max_open_states = 250,000
```

- limit은 config hash에 포함한다.
- `max_open_states`는 아직 확정되지 않은 서로 다른 state 수를 센다. heap에 남은 stale entry나
  같은 state의 더 나쁜 중복 entry는 다시 세지 않는다.
- `count == limit`에서 더 탐색할 state가 없으면 정상 exhaustive 종료다.
- 실제 N+1 state가 필요한 시점에만 resource limit으로 판정한다.
- elapsed time, CPU 사용률, cache hit ratio는 taxonomy에 넣지 않는다.
- `UNSPECIFIED`은 LEFT·RIGHT를 서로 독립된 bounded lane으로 실행하므로 위 limit은 side lane별로
  적용한다. 합친 result의 count는 두 lane 합계여서 개별 limit의 2배까지 될 수 있다.

### `INVALID_INPUT`

- provenance/hash mismatch
- unsupported vehicle or lattice profile
- non-finite pose·resolution·tolerance
- start·goal frame mismatch
- search region이 start·goal을 포함하지 않음
- allowed·forbidden source 계약 불일치

## 10. infrastructure 상태 분리

다음은 위 네 알고리즘 결과가 아니다.

```text
INFRASTRUCTURE_INCOMPLETE
  process crash
  OOM
  사용자 중단
  output write 실패
  validator 예외
  예상하지 못한 Python 예외
```

partial output은 보존하지만 `RESOURCE_LIMIT`, `SPATIALLY_INFEASIBLE` 또는 성공 근거로 승격하지
않는다. 재개는 같은 request/config/source hash와 누락 없는 shard receipt가 검증될 때만 허용한다.

## 11. 결과 계약

```text
BoundedSpatialOracleResult
  schema_version
  oracle_version
  status
  termination_reason
  request_content_hash
  map_id
  map_revision
  mission_revision
  grid_content_hash
  vehicle_profile_hash
  search_region_hash
  lattice_config_hash
  path
  primitive_sequence
  path_length_m
  reverse_length_m
  rotation_count
  minimum_clearance_m
  minimum_physical_clearance_m
  minimum_forbidden_clearance_m
  minimum_allowed_boundary_clearance_m
  generated_edges
  expanded_states
  peak_open_states
  exhaustive
  validation:
    validator_version
    passed
    failure_codes
    validation_content_hash
  limitations
  semantic_content_hash
  elapsed_nonqualification_ns
```

### 결과 불변식

- feasible에서만 non-empty path와 validation pass를 허용한다.
- infeasible은 `exhaustive=true` 또는 독립 analytic proof를 요구한다.
- resource limit은 `exhaustive=false`, path·validation `null`이다.
- invalid input은 search count가 모두 0이다.
- `elapsed_nonqualification_ns`와 worker·shard 운영정보는 semantic hash에서 제외한다.
- limitations는 정렬·중복 제거 후 semantic hash에 포함한다.

## 12. R2-A 입력 projection

R2-A episode를 R3 request로 바꾸는 projection은 다음만 사용한다.

```text
static grid
forbidden·allowed region
vehicle profile
start pose
명시적으로 선택한 reference rejoin pose·heading
해당 rejoin이 속한 reference segment
bounded search region
side
map provenance
```

다음은 projection 입력으로 금지한다.

```text
expectation_category
oracle_spec
feasible_witness
수동 witness waypoint
selected R2 path
Actor trajectory·속도·active interval
hidden split·seed
evaluator verdict
```

Actor를 지운 static projection은 “사람을 피해 지나갈 수 있다”를 증명하지 않는다. 그 장면의
정적 차체 공간이 우회 모양을 수용할 수 있는지만 판정한다.

legacy crossing 장면은 같은 static world와 rejoin goal에 대해 LEFT·RIGHT request를 각각
만든다. 기존 crossing witness는 search 입력이 아니라 결과 뒤 evaluator-only 회귀 비교에만
사용한다.

## 13. 공개 시험 matrix

| 시험군 | 입력 | 기대 판정 |
|---|---|---|
| wide straight LEFT | 충분한 좌측 공간 | `SPATIALLY_FEASIBLE` |
| wide straight RIGHT | 충분한 우측 공간 | `SPATIALLY_FEASIBLE` |
| left-right mirror | geometry mirror | status·비용·clearance 대칭 |
| narrow corridor | footprint+clearance보다 좁은 단면 | `SPATIALLY_INFEASIBLE` |
| narrow door | 중심점은 통과하나 oriented footprint 불가 | `SPATIALLY_INFEASIBLE` |
| just-wide door | 경계보다 충분히 넓음 | `SPATIALLY_FEASIBLE` |
| dead end | 진입 가능하나 rejoin 불가 | exhaustive `SPATIALLY_INFEASIBLE` |
| corner | 제자리회전 sweep이 필요한 90° 경로 | safe/unsafe 분리 |
| vertical rotation | 수평 사례의 90° 회전 | metamorphic 동일 판정 |
| forbidden-only block | occupancy free, forbidden이 차단 | `SPATIALLY_INFEASIBLE` |
| allowed-region pinch | map free, allowed boundary가 차단 | `SPATIALLY_INFEASIBLE` |
| start/goal unsafe | endpoint footprint 위반 | 명시적 infeasible reason |
| exact resource boundary | 마지막 state가 limit에서 소진 | 정상 exhaustive 결과 |
| limit plus one | 다음 state가 필요 | `RESOURCE_LIMIT` |
| invalid provenance | hash·revision 변조 | `INVALID_INPUT` |
| crossing static projection | legacy 횡단 장면 좌·우 | 기존 positive와 모순 없는 feasible |

### 13.1 동결 public request catalog

runner가 실행하는 순서와 ID는 다음으로 고정한다. `expected_status`와 관계 검사는 reporting에만
존재하며 search request에는 들어가지 않는다.

```text
00 wide-straight-left
01 wide-straight-right
02 wide-mirror-left
03 wide-mirror-right
04 narrow-corridor
05 narrow-door
06 just-wide-door
07 dead-end
08 corner-safe
09 corner-rotation-blocked
10 vertical-left
11 vertical-right
12 forbidden-only-block
13 allowed-region-pinch
14 start-unsafe
15 goal-unsafe
16 resource-exact
17 resource-plus-one
18 invalid-provenance
19 crossing-static-left
20 crossing-static-right
```

- synthetic case는 같은 `0.02m` grid와 가상 차체를 사용하며 public catalog 생성 함수가
  request와 evaluator-only 기대값을 함께 만든다.
- crossing 두 건만 legacy golden `local_detour_feasible`의 공개 world를 static projection한다.
- corner는 v1의 단일 reference segment 안에서 시작 yaw와 목표 yaw가 90도 다른 synthetic
  회전 기하 시험이다. multi-segment public corner projection 완료를 뜻하지 않는다.
- `resource-exact`와 `resource-plus-one`은 같은 static request에서 동결된 exact count와 그보다
  하나 작은 한계를 사용한다.
- mirror와 vertical 관계는 status뿐 아니라 rotation count를 정확히, path length·clearance를
  public grid 한 칸인 `0.02m` 허용오차 안에서 비교한다.

### 13.2 public qualification 판정

```text
case hard pass =
    actual status == evaluator-only expected status
    AND feasible이면 independent validation pass
    AND infeasible이면 exhaustive 또는 허용된 analytic reason
    AND resource/invalid taxonomy가 기대 reason과 일치

run hard pass =
    catalog ID·request hash 누락/중복 0
    AND 모든 case hard pass
    AND mirror·vertical·resource 관계 pass
    AND serial/process semantic parity pass
```

wall-clock, worker 수와 case 완료 순서는 운영 진단이며 semantic hash와 합격 판정에서 제외한다.

### 필수 적대시험

- center cell만 free지만 footprint corner가 벽과 겹치는 path 거부
- endpoint는 안전하지만 translation 중간이 금지영역을 통과하는 primitive 거부
- endpoint는 안전하지만 in-place rotation corner가 벽을 스치는 primitive 거부
- `0.08m - ε` 거부, `0.08m + ε` 허용
- search success flag만 변조한 path를 independent validator가 거부
- map revision·grid byte·allowed region·profile hash 변조 거부
- worker 완료 순서를 바꿔도 semantic result 동일
- elapsed time을 바꿔도 semantic hash 동일
- resource limit을 infeasible로 직렬화하는 결과 생성자 거부
- category·oracle·Actor·hidden import가 search package에 들어오면 AST 시험 실패

## 14. 결정론·병렬 실행·cache

### 14.1 결정론

- 같은 request/config/source hash는 path, primitive, count, validation과 semantic hash가 같다.
- heap tie-break는 명시적 tuple만 사용한다.
- NumPy·set·dict iteration 순서에 결과를 의존하지 않는다.
- 좌·우 결과 합치기는 side enum 순서로 고정한다.

### 14.2 병렬 실행

- 독립 request·side·public case는 process 기반 병렬 실행할 수 있다.
- 하나의 request 내부 state expansion은 v1에서 결정론적 직렬 순서를 사용한다.
- 병렬화를 이유로 grid, heading bin, primitive, state 수, clearance나 판정 기준을 바꾸지 않는다.
- worker 수는 실제 CPU·메모리 확인 후 대략 절반 CPU 활용을 운영 목표로 한다.
- wall-clock은 판정 근거가 아니며 native timing은 R7에서 별도 측정한다.

### 14.3 cache

- static footprint validity와 primitive sweep 결과는 `(map/profile/config/state/primitive)`로 cache할
  수 있다.
- cache hit·miss가 의미 결과를 바꾸면 실패다.
- source hash가 다르면 cache를 재사용하지 않는다.
- cache는 운영 최적화이며 semantic output에 포함하지 않는다.

## 15. 산출물과 수명주기

```text
outputs/spatial-oracle-public-<UTC>-<HEAD>/
  run-manifest.json
  run_state.incomplete.json
  requests/<ordinal>-<request-hash>/request.json
  requests/<ordinal>-<request-hash>/result.json
  requests/<ordinal>-<request-hash>/path.png
  requests/<ordinal>-<request-hash>/validation.json
  summary.md
  run_state.complete.json
  qualification-receipt.json
```

- output 경로를 덮어쓰지 않는다.
- 시작 시 incomplete state를 먼저 쓴다.
- 모든 public request와 독립 validator가 완료된 뒤에만 complete·receipt를 쓴다.
- partial은 삭제하지 않지만 최종 근거로 사용하지 않는다.
- 생성 output은 기본 Git 대상이 아니다.
- R3에서 hidden을 생성·조회·실행하지 않는다.
- `qualification-receipt.json`은 catalog 21건, request/result/validation hash, source freeze,
  process 결과의 input-ordinal 재정렬과 관계 검사가 모두 확인된 경우에만 생성한다.
- runner의 독립 case 계산은 process 병렬화하지만, 같은 request 내부 state expansion은 기존
  직렬 순서를 유지한다.

## 16. 구현 경계와 현재 상태

2026-08-14 core L1에서 다음을 구현했다.

```text
src/hospital_path_lab/spatial_oracle_contracts.py
src/hospital_path_lab/spatial_oracle_lattice.py
src/hospital_path_lab/spatial_oracle_validation.py
src/hospital_path_lab/spatial_oracle_projection.py
tests/test_spatial_oracle_contracts.py
tests/test_spatial_oracle_lattice.py
tests/test_spatial_oracle_validation.py
tests/test_spatial_oracle_projection.py
```

core 표적 `28 passed`에는 hash·taxonomy·resource boundary·회전 sweep·LEFT/RIGHT·공개
`same-direction-wide-r00` static projection이 포함된다. 구현 중 직진 path가 LEFT/RIGHT를
거짓 통과하지 않도록 최소 side excursion `0.10m`와 phase bit를 추가했다. search의 빠른
clearance lower bound는 후보 거부·승인 전처리일 뿐이며 선택 path는 새 geometry evaluator를
사용하는 독립 validator로 다시 검사한다. 14-process 파일 분할 전체 회귀는 `719 passed`,
failure·error·skip `0`으로 완료했다. 해당 병렬 wall-clock은 timing 자격 근거가 아니다.

2026-08-14 public qualification 계층으로 다음을 추가했다.

```text
src/hospital_path_lab/spatial_oracle_reporting.py
scripts/run_spatial_oracle_public.py
tests/test_spatial_oracle_public.py
```

동결 21-case catalog, evaluator-only 기대값, 관계·serial/process parity, process 병렬 실행,
non-overwrite partial/complete 수명주기, JSON·Markdown·PNG와 clean-source receipt를 구현했다.
전체 R3 직접 영향권은 `38 passed`다. clean commit `53fd9f8` 대상 14-process 실행에서
21개가 모두 기대 판정을 통과했고 receipt를 생성했다. 구현 뒤 전체 회귀는 `729 passed`였다.
상세 hash와 case 결과는
[`R3 공개 qualification 결과`](r3-public-spatial-qualification-result-2026-08-14.md)에 기록했다.

의존 방향은 다음으로 고정한다.

```text
contracts
  ↑
lattice search ← static collision geometry
  ↓
independent validator
  ↓
reporting

R2 public episode → projection → request
evaluator label ────────────────→ reporting only
```

- validator는 lattice search를 import하지 않는다.
- search는 `dynamic_corpus`, evaluator taxonomy, Actor prediction과 hidden helper를 import하지
  않는다.
- projection만 공개 episode 구조를 알 수 있고 request를 만든 뒤 category·Actor를 버린다.
- registry의 `state_lattice=deferred`는 제품 planner로 채택하기 전까지 그대로 유지한다.

## 17. 구현·검증 순서

```text
1. contracts와 생성자 불변식
2. oriented pose·primitive sweep validator
3. 작은 analytic map의 exhaustive lattice
4. start·goal anchor connector
5. side·allowed·forbidden 계약
6. R2-A static projection
7. 대표 public/golden 사례
8. 읽기 전용 최종 감사
9. 공개 request process 병렬 실행
10. 전체 회귀
```

예상 시간을 각 단계 전에 시험 수·state bound로 계산한다. 장시간 전체 suite는 파일과 request를
분할하며, timeout 뒤 같은 명령을 그대로 재실행하지 않는다.

## 18. R3 완료조건

- contract·hash·taxonomy 적대시험 통과
- wide LEFT/RIGHT와 mirror positive 통과
- narrow door·dead end·forbidden·allowed negative가 exhaustive 또는 analytic proof로 닫힘
- corner와 vertical metamorphic 시험 통과
- crossing static projection이 기존 R2-A positive와 모순되지 않음
- 모든 feasible path가 independent swept-footprint validator를 통과
- resource boundary에서 infeasible 오분류 0
- invalid provenance fail-closed
- serial·process batch의 semantic result 동일
- public output lifecycle과 receipt 완주
- 관련 영향권과 전체 회귀 통과
- R2-B·hidden·제품 결정에 손대지 않음

현재 core·reporting L1, 전체 public matrix, clean-source receipt와 구현 뒤 마지막 전체 회귀를
완료했다. 이 완료는 offline static R3 범위에만 적용되며 multi-segment corner projection과
R4 이후 시간·controller 통합은 포함하지 않는다.

## 19. R4 전달 계약

R3 feasible 결과 중 independent validator를 통과한 것만 R4 후보로 전달한다.

```text
SpatialReferenceSeed
  source_spatial_result_hash
  map_id
  map_revision
  mission_revision
  side
  start_pose
  rejoin_goal
  pose_heading_path
  primitive_sequence
  minimum_clearance_m
  validation_hash
  limitations
```

R4는 이를 controller가 소비할 local reference·subpath revision 계약으로 변환한다. R3 path
자체를 chassis command로 실행하지 않으며, 경로 존재를 재개 승인으로 해석하지 않는다.

## 20. 동결 전 명시적 한계

- v1 lattice는 translation+in-place rotation만 사용하므로 smooth curvature 가능성을 일반화하지
  못한다.
- start·goal anchor connector는 1-cell 이웃 안의 추상 swept connector이며 실제 차체 motion
  primitive가 아니다. R4 이후 시간 경로가 추종 가능성을 별도로 검증해야 한다.
- `SPATIALLY_INFEASIBLE`은 bounded region·8 heading bins·v1 primitive 안의 음성 판정이다.
- R3의 simulation-only reverse primitive 자체는 실제 차체 후진 허용을 결정하지 않는다.
  후속 R5 Python 연구에서는 [`ADR 0014`](../../decisions/0014-section-bound-bounded-reverse-translation.md)에
  따라 R4가 명시한 reverse section에만 제한 후진을 허용하며, 제품·실물 결정은 여전히 별도다.
- static projection은 Actor와 시간 충돌을 제거하므로 temporal feasibility를 증명하지 않는다.
- Python 실행시간·CPU·memory·cache는 기능 판정에서 제외한다.
- 실제 차체·Unity·ROS 2·센서·사람 탑승 증거가 아니다.

이 한계를 줄이려면 별도 버전에서 richer lattice 또는 Hybrid A* 대조 oracle을 추가하고 기존
v1 결과를 회귀 자료로 유지한다. hidden이나 현재 결과를 본 뒤 v1 lattice 수치를 조정하지
않는다.
