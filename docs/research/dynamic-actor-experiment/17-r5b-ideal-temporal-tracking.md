# R5-B Ideal temporal tracking 상세 명세

- 상태: 구현 기준선
- 작성일: 2026-08-15
- 범위: Python 공개 연구실, 합성 ground truth Actor, R2-A exact witness
- 비범위: 카메라·FOV·가림·검출·추적, R2-B 판정 변경, hidden, 제품 알고리즘 채택

## 1. 목적

R5-A는 정적인 full reference와 section을 persistent RPP·source-derived DWB가 끝까지
추종할 수 있음을 확인했다. R5-B는 그 controller를 새로 고르는 단계가 아니다. R2-A에서
이미 검증한 시간축 PASS witness를 R4 reference와 실제로 결박하고, 합성 Actor가 처음부터
존재하는 공개 장면에서 다음 순서를 지키는지 확인한다.

```text
실제 정지 유지
→ Ideal causal ActorTrack이 READY가 됨
→ 동결한 2.0 s / tick 40 인과 대기 도달
→ 별도 temporal 실행 허가 발행
→ 현재 상태에서 다시 검증한 우회 reference 추종
→ Actor 통과
→ 원 경로 재합류
→ 목표 정지
```

R5-B 성공은 실제 perception 성공, 사람 안전 보장 또는 제품 controller 선정이 아니다.

## 2. 입력 증거

정본 R2 증거는 다음 추적 ZIP이다.

```text
simulation/path_planning_lab/outputs/
witness-audit-public-20260813-r2-v2-4e4ba0f.zip
```

- 크기: `3,657,108 bytes`
- SHA-256: `50567b093082a57232e668ef89c0316a426cd936496e465b943fe57efa894266`
- source audit commit: `4e4ba0fb91d67498fe163aca99ff1ab647224f08`
- 대상: `v6_primary` same-direction `wide-feasible-r00~r04`
- side: 각 episode의 `PASS_LEFT`, `PASS_RIGHT`
- 총 temporal case: `5 × 2 = 10`

135,360개 전체를 다시 탐색하지 않는다. ZIP의 world, selected witness와 validation을 읽되
현재 코드에서 immutable 계약 객체로 복원하고 strict ground-truth validator를 다시 실행한다.
파일 크기, ZIP digest, 안전한 entry 이름, public id, world/witness hash 또는 기록된 validation
provenance 중 하나라도 다르면 fail-closed한다. 기록은 validator v2 hash를 그대로 보존하고 현재
validator v3 재검증 hash를 별도로 결박한다. 같은 validator 판본에서 두 hash가 다르면
fail-closed한다.

기존 R2 selected witness를 시간축으로 2.0 s 단순 이동하는 방식은 10개 모두 Actor
clearance를 깨뜨리므로 금지한다. R5-B는 복원한 world와 코드에 동결된 측면 offset
축은 보존하되, tick 40에서 시작하는 후보를 현재 strict validator로 다시 검증하여
첫 통과 후보를 파생한다. 이것은 R2 결과를 바꾸는 것이 아니며 R5-B 전용 causal
reference를 만드는 것이다.

## 3. R2-B와의 경계

R5-B는 R2-B hard failure를 없애거나 우회해서 관측 통합을 주장하지 않는다.

- 대상 Actor는 모든 10개 case에서 `t=0`부터 존재한다.
- observation은 Ideal 10 Hz, latency 100 ms, noise/dropout 0이다.
- controller는 ground truth를 직접 받지 않고 `DynamicObservationFrame → validated snapshot →
  DirectionalPredictionSet` 경로만 사용한다.
- fresh empty, no-frame, stale, prediction-not-ready일 때 temporal release를 발행하지 않는다.
- interior instant appearance, 카메라 가림과 R2-B의 기존 두 failure는 변경하지 않는다.
- R5-C는 여전히 R2-B 통과 전 차단한다.

## 4. 시간 증거와 reference 결박

각 case는 다음 immutable 체인을 가진다.

```text
archive digest
→ frozen R2 world hash
→ selected witness hash
→ archived strict validation v2 hash + current strict validation v3 hash
→ TemporalReferenceEvidence hash
→ GROUND_TRUTH_TEMPORAL LocalManeuverReference hash
→ R5-B execution authorization hash
→ controller tick input hash
```

`TemporalReferenceEvidence`는 정확히 한 Actor binding, ordered departure/pass/rejoin progress,
ground-truth-only 한계와 source witness/validation hash를 가진다. reference의 공간 geometry도
별도로 다시 검증한다. witness hash를 검증하지 않은 채 임의의 spatial seed hash로 가장하지
않는다.

## 5. 실행 허가

GROUND_TRUTH_TEMPORAL reference라는 사실만으로 출발할 수 없다. tick마다 다음을 모두 만족한
경우에만 `R5-B temporal execution authorization`을 발행한다.

1. reference·temporal evidence·world·witness·validation hash가 일치한다.
2. 현재 mission, map, stop epoch, reference session과 Actor binding이 일치한다.
3. 실제 정지 상태가 유지되고 있다.
4. current tick의 observation snapshot이 accepted/fresh이며 empty가 아니다.
5. directional prediction 상태가 `READY`이고 target Actor capsule이 존재한다.
6. current simulation time이 `2.0 s` 이상이고 controller tick이 `40` 이상이다.
7. Actor revision/source identity가 이전 valid state보다 역행하지 않는다.
8. shared local safety gate의 독립 허가가 별도로 성립한다.

허가 발행 전에는 0 command와 HOLD를 유지한다. 한 번 발행된 허가는 다른 mission, stop epoch,
reference revision, session, Actor revision 또는 tick에서 재사용하지 않는다. 입력이 stale·empty·
invalid로 바뀌면 이전 허가를 폐기하고 다시 정지한다.

## 6. reference 변환

PASS witness를 그대로 controller command로 재생하지 않는다. witness pose를 R4 full reference로
변환하고 persistent controller가 그 reference를 추종하게 한다.

- tick 40까지의 정지 pose는 실행 path에서 제거하고 tick 40 pose를 첫 anchor로 삼는다.
- translation·rotation geometry와 signed travel direction은 witness 연속 pose에서 유도한다.
- phase는 ordered `DEPART → BYPASS → RETURN → REJOIN` section으로 매핑한다.
- causal release tick과 실제 출발 순서는 geometry가 아니라 temporal evidence와 실행
  허가에 남긴다.
- 마지막 reference knot는 `REJOIN + STOP_MARKER`다.
- 변환된 reference는 current static grid·forbidden region·차체로 독립 재검증한다.

## 7. 공개 판정

각 case를 RPP와 DWB에 동일한 world, observation stream, temporal reference와 release tick으로
paired 실행한다. controller 내부 상태만 분리한다.

필수 성공 조건:

- 허가 전 비영점 command 0회
- fresh empty/no-frame/stale/NOT_READY 상태에서 release 0회
- release 뒤 실제 departure 1회
- target Actor와의 ground-truth clearance 위반 0회
- static·forbidden·shared gate hard failure 0회
- departure → pass → sustained rejoin → terminal stop 순서 일치
- 목표 도달 및 deadlock 0회
- RPP/DWB가 같은 외생 입력 digest 사용
- repeat determinism과 serial/process semantic parity 통과

R5-A static 21개와 R2-B failure regression은 그대로 통과해야 한다. R5-B를 통과해도 R5-C,
R6, R7 또는 hidden으로 자동 진입하지 않는다.

## 8. 중단 조건

다음이면 결과를 성공으로 만들기 위해 기준을 완화하지 않고 원인과 증거를 남긴다.

- R2 ZIP 또는 기록 hash 불일치
- 현재 validator가 frozen witness를 더 이상 승인하지 않음
- 변환 reference가 R4 구조·공간 validator를 통과하지 않음
- Ideal causal stream이 departure 전 READY가 되지 않음
- 허가 전 이동 또는 stale/empty에서 이동
- Actor/static/forbidden clearance 위반
- controller가 reference를 추종하지 못함

안전 margin, Actor radius, latency, TTL, predictor tube, controller 기준, witness 선언 또는
shared gate를 통과시키기 위해 줄이지 않는다.

## 9. 2026-08-15 1차 실행 판정

구현 뒤 공개 RPP 5 episode × 좌·우 10개 실행은 R5-B를 통과하지 못했다. `2.0 s` causal
release에서 새로 만든 경로는 ground truth 안전 검증을 통과했지만, R5-A의 `0.20 m/s`
controller는 Actor가 존재하는 마지막 시간까지 완전한 종방향 분리를 만들지 못했다. 첫
LEFT 사례의 마지막 정합 진행 차이는 `+0.18533733027495725 m`이고 필요한 분리는
`0.400 m`다. 10개 모두 실제 이탈은 관측됐지만 ordered overtake·sustained rejoin은 없었다.
첫 LEFT DWB도 마지막 진행 차이 `-0.08294597353371147 m`, gate override·clearance 위반
`0`으로 같은 기능 실패를 보였다. 약 51분의 Python wall-clock은 기능 판정과 분리한다.

따라서 이 명세의 `fresh empty` 차단과 target-bound READY 조건을 유지한다. Actor active
interval을 늘리거나 empty frame을 이전 허가의 자동 연장으로 바꾸지 않는다. 상세 결과는
`r5b-initial-public-temporal-tracking-result-2026-08-15.md`에 기록한다.
