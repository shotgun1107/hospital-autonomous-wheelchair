# R7 hidden-v4 조건부 Stress evaluator

- 상태: evaluator·공개시험 구현, 실제 hidden 미실행
- 기준 정책: [40-r7-stress-conditional-release-policy-2026-08-19.md](./40-r7-stress-conditional-release-policy-2026-08-19.md)
- 적용 범위: offline·simulation 관측 순서 평가
- 비적용 범위: 실제 카메라·사람 안전, 제품 알고리즘 채택

## 1. 새 namespace

기존 hidden-v3 결과와 seed를 재사용하지 않도록 다음 값을 별도로 사용한다.

```text
commitment/evaluator version = r7-hidden-observation-v3
case prefix = hidden-v4-
Stress expected outcome = conditionally_safe_hold
```

구현은 `r7_hidden_v4_qualification.py`에 분리했다. 기존
`r7_hidden_qualification.py`와 hidden-v3 runner의 무출발 판정은 역사적 결과 재현을 위해
변경하지 않는다.

## 2. Normal 판정

Normal 10건은 기존과 같은 순서를 요구한다.

```text
첫 이동
< Actor 통과 증거
< 원 경로용 새 session release
< 목적지 완료
```

최종 상태는 `COMPLETED`, hard failure는 0이고 Actor·정적 여유는 모두 `0.08m` 이상이어야
한다.

## 3. Stress 판정

Stress는 다음 두 종료 형태를 허용한다.

### 출발하지 않은 보수 정지

```text
release 없음
first motion 없음
controller call 0
최종 HOLDING
```

### 조건부 출발 뒤 재정지

```text
release
< first motion
< protective stop 시작
<= 실제 stop 확인

최종 HOLDING
마지막 session stop_epoch보다 final stop_epoch가 큼
hard failure 0
실제 clearance >= 0.08m
```

Evaluator는 실행 결과의 사건 순서와 stop epoch를 검사한다. 재출발 직전 gate-confirmed distinct
safe frame 11개 조건은 공개 trace 회귀
`test_stress_left_seed_214092870162924582_conditionally_releases_then_restops`에서 직접 검사한다.
같은 10Hz frame을 두 번 세는 동작, stale release와 profile 이름 전용 controller 분기는 허용하지
않는다.

## 4. 공개시험

```text
tests/test_r7_hidden_v4_qualification.py
```

다음을 고정한다.

- 새 commitment namespace와 `hidden-v4-*` catalog
- 같은 replica·side의 Normal/Stress seed pairing
- 무출발 HOLD와 조건부 release→re-stop 두 형태
- stop 확인 누락, epoch 미증가, clearance 위반과 hard failure 거부
- 기존 hidden-v3 namespace와 evaluator 불변

## 5. 실제 hidden 전 차단 조건

이 evaluator 구현만으로 actual hidden을 실행하지 않는다. 다음을 먼저 완료해야 한다.

1. 전체 공개 회귀 완료
2. 현재 source와 clean native library의 Python↔C++ parity
3. CPU contention 없는 직렬 `5×100=500` timing에서 `50ms` 초과 0
4. 실행 commit·tree·source freeze·native hash가 결박된 새 qualification evidence
5. 새 evidence hash와 실행 commit을 검증하는 hidden-v4 runner freeze
6. seed 생성 전에 commitment 기록
7. 이전 hidden-v1/v2/v3와 공개 회귀 seed 미사용

이 조건이 하나라도 미완료면 actual hidden-v4는 실행하지 않는다.

## 6. 실행 증거와 기반시설 실패

- 각 case는 첫 실행의 JSONL trace를 남긴다.
- trace SHA-256, record count, 마지막 record hash와 semantic hash를 case result에 넣고,
  case trace set과 manifest hash를 summary·consumption receipt까지 연결한다.
- result에 기록된 trace 정보와 실제 파일이 다르면 최종 포장을 거부한다.
- commitment 뒤 준비, case 실행 또는 최종 포장에서 기반시설 오류가 나면 알고리즘 FAIL로
  판정하지 않는다. `BLOCKED_INFRASTRUCTURE` summary·receipt를 쓰고 소비 ledger에 그 receipt
  hash를 기록한다.
- 중단 trace는 원인 분석 자료일 뿐 `PASS_FINAL` 증거가 아니다.
