# R2-B Actor 출현과 관측 복구 명세

- 상태: 공개 시뮬레이션 연구 범위 고정
- 작성일: 2026-08-16
- 대상: 기존 R2-B 실패 2건과 R5-C 제한 진단의 관측 상실 뒤 복구
- 비범위: 실제 초음파 센서·배치·반사·무응답 성능, hidden, 제품 알고리즘 채택

> 최신 센서 입력 — 2026-08-16: 당시 카메라 가시영역을 예로 든 부분은 현재 Arduino·초음파
> 거리 센서 방향에 맞춰 `실제 센서 coverage`로 일반화한다. 이 문서는 합성 `ActorTrack`
> 복구 계약이며 거리 측정에서 track을 만드는 방법은
> [초음파 관측 전환 명세](22-ultrasonic-observation-transition.md)에서 별도로 다룬다.

## 1. 확인된 문제

기존 R2 공개 감사의 두 실패는 예측 수식이나 DWA/DWB 때문에 발생하지 않았다.

| 장면 | Actor 출현 | 관측 공백 | 결과 |
|---|---:|---:|---|
| v6 `second-risk` | `13.000s` | `13.000~13.150s` fresh `EMPTY` | containment miss `38` |
| legacy `dynamic-change` | `4.051282...s` | 약 `4.055~4.250s` fresh `EMPTY` | containment miss `43` |

현재 합성 world는 Actor를 지도 내부에 순간 생성한다. 관측에는 지연이 있으므로 생성 직후에는
실제 Actor가 있어도 직전 빈 frame만 도착한다. 관측 전에 존재를 알아내는 것은 불가능하다.
따라서 Actor 반경, Capsule, 지연, 안전 여유 또는 hard criterion을 낮춰 이 실패를 없애지
않는다. 기존 R2 output과 실패 판정도 역사적 회귀 자료로 유지한다.

## 2. 서로 다른 두 문제

### 2.1 미관측 Actor의 새 출현

```text
직전 fresh EMPTY
→ 지도 내부에서 새 Actor 생성
→ 아직 새 관측이 도착하지 않음
```

현재 하네스에는 실제 초음파 센서의 배치 coverage나 진입구 감시 증거가 없다. 따라서 fresh `EMPTY`는
“이 frame에 track이 없다”는 뜻일 뿐, 안전 관련 영역 전체에 사람이 없다는 증거가 아니다.

현재 단계에서는 다음을 금지한다.

- fresh `EMPTY`만으로 비영점 이동 시작 또는 재개
- corpus의 Actor 출현 시각을 controller에 전달
- ground truth로 관측 공백을 미리 알려 정지
- 과거 빈 frame으로 미래의 무출현을 보증

R2-B의 이 부분을 닫으려면 후속 단계에서 최소한 다음 중 하나를 공개 계약으로 정해야 한다.

1. Actor가 감시된 진입 경계를 통해서만 들어오는 world와 그 진입 여유
2. 안전 관련 전 영역을 포함하는 유효한 빈 공간 확인 증거
3. 실제 센서 coverage·무응답·오검출 실패를 포함한 보수적 관측 계약

후속으로 1번의 추상 감시 진입 world를 구현해 대표 Ideal miss를 `38/22 → 0/0`으로
줄였다. 그 뒤 HC-SR04 7개 순차 simulation 감사를 추가했으나 full frame은 기존 300ms TTL을
`0개` 통과했고 v6 지연 Actor 하나는 진입 전 원시 감지도 `0회`였다. 원본 hard failure는
음성 회귀로 유지하며 실제 센서 coverage 계약은 아직 닫히지 않았다.

### 2.2 이미 추적하던 Actor의 관측 상실과 회복

```text
READY 상태로 이동 중
→ dropout·stale·low confidence·track 상실
→ 로컬 보호정지
→ 실제 정지 완료
→ 새 관측으로 READY 재성립
```

이 문제는 새 Actor 출현 문제와 다르다. Actor가 처음부터 존재하는 공개 장면에서 기존
관측·예측·정지 계약을 그대로 사용해 제한적으로 시험할 수 있다. 이 복구가 성공해도 R2-B
출현 문제나 실제 perception을 해결한 것으로 보지 않는다.

## 3. 복구 상태 흐름

관측 상실 뒤에는 다음 순서를 강제한다.

```text
이동 중 방향 예측 상실
→ 기존 controller 결과·reference session·이동 허가 폐기
→ 감속
→ 실제 정지 완료와 새 stop epoch 확인
→ 고유한 fresh READY frame 11개 재수집
→ 현재 정지 pose에서 새 reference 생성·검증
→ 새 stop epoch에 결박된 새 resume authorization 생성
→ 새 controller session 시작
```

다음은 복구 조건이 아니다.

- 관측 한 frame의 복귀
- 장애물 또는 Actor 소실
- 이전 reference·이동 허가·controller 상태의 재사용
- fresh `EMPTY`

새 reference를 안전하게 만들 수 없거나 READY 11개가 모이지 않으면 정지를 유지한다.

## 4. 이번 구현 범위

### 허용

- Actor가 `t=0`부터 존재하는 공개 Normal·Stress 장면
- 입력 상실 시 방향 예측을 `None`으로 gate에 전달해 보호정지
- 실제 정지 뒤 새 stop epoch와 새 session을 사용하는 복구
- 현재 pose에서 기존 목적지까지 이미 검증 가능한 `FOLLOW_ORIGINAL` reference를 새로 만들 수
  있는 다중 위험 장면
- Stress에서 READY가 성립하지 않으면 계속 정지하는 음성 결과

### 보류

- 횡단 PASS reference의 중간 지점에서 새 시간·공간 증거를 만드는 절차
- fresh `EMPTY`를 이용한 post-pass 계속 이동
- 두 R2-B 내부 순간 출현 장면의 완료 판정
- 실제 센서나 사람 안전 주장

횡단 장면은 입력 상실 뒤 새 reference를 만들 공개 계약이 아직 없으므로 이번에는 안전정지
결과를 유지한다. 이를 억지로 원래 reference에 다시 붙이지 않는다.

## 5. 완료조건

1. 기존 R2-B 실패 2건과 원인이 문서·시험에서 보존된다.
2. fresh `EMPTY` 단독으로 새 이동이나 재출발을 허가하지 않는다.
3. 관측 상실 뒤 실제 정지가 확인되기 전에는 복구를 시작하지 않는다.
4. 복구는 이전 stop epoch·reference session·허가를 재사용하지 않는다.
5. 고유한 fresh READY frame `11`개를 다시 확인한다.
6. Normal 다중 위험 장면에서 새 session을 사용한 재출발 또는 보수적 정지의 원인을 기록한다.
7. Stress는 판단이 불충분하면 비영점 명령 없이 정지를 유지한다.
8. 안전 여유·Actor 반경·Capsule·관측 profile·shared gate를 완화하지 않는다.
9. 결과는 공개 시뮬레이션 진단으로만 보고하며 formal R5-C/R6 receipt를 발급하지 않는다.

## 6. 다음 분기

```text
제한 복구 성공
→ 이미 추적하던 Actor의 dropout 회복 증거만 추가
→ R2-B 출현 실패는 유지

새 reference 생성 불가
→ 정지 유지
→ reference 복구 계약을 별도 설계

R2-B 출현 문제를 다시 시작
→ entry·visibility·coverage 계약을 먼저 승인
→ 새 corpus/hash/output에서 공개 감사
```

## 7. 후속 진행

사용자 지시에 따라 추상 감시 접근 구간을 별도 계약으로 구현했다. 원본 실패 world는 그대로
두고, 지연 출현 Actor를 원래 진입 상태에서 `t=0`까지 역산한 새 관측 world와 hash를 사용한다.
두 대표 Ideal replay에서 원본 miss `38/22`를 재현한 뒤 파생 miss `0/0`을 확인했다.

상세 기준과 결과:

- [R2-B 감시 진입 명세](21-r2b-monitored-entry-coverage.md)
- [R2-B 감시 진입 결과](r2b-monitored-entry-coverage-result-2026-08-16.md)

이는 실제 초음파 거리·배치 coverage·반사·무응답 증거가 아니므로 R2-B 전체나 formal
R5-C/R6를 완료 처리하지 않는다.

HC-SR04 7개 임시 배치를 적용한 후속 결과는
[R2-B HC-SR04 7개 감사 결과](r2b-hc-sr04-seven-sensor-audit-result-2026-08-16.md)에
기록했다. 이 감사도 simulation-only이며 R2-B 판정은 계속 보류다.
