# R5-C 공개 관측 열화 진단

- 상태: 공개 진단 범위 고정
- 작성일: 2026-08-16
- 대상: R5-B에서 완료한 횡단 좌·우와 다중 위험 재정지 사례
- 비범위: R2-B 완료 판정, 카메라·FOV·가림, hidden, 제품 알고리즘 채택

## 1. 목적과 경계

R5-B는 잡음과 누락이 없는 Ideal 합성 관측에서 횡단 좌·우와 다중 위험 재정지를
완료했다. 이번 작업은 같은 공개 world에 동결된 Normal·Stress 관측을 넣었을 때
관측·방향 예측·정지 권한·controller 중 어디에서 진행이 멈추는지 분리해 기록한다.

기존 R2-B에는 내부 시점 Actor 출현과 fresh EMPTY에 관한 hard failure 2건이 남아 있다.
따라서 이 문서의 실행은 정식 `R5-C OBSERVATION_INTEGRATED` 자격이나 R2-B 통과가 아니라,
Actor가 `t=0`부터 존재하는 제한된 공개 사례의 진단이다. 결과가 좋아도 R6·hidden으로
자동 전환하지 않는다.

## 2. 동결 입력

| profile | 관측률 | 지연 | TTL | 위치 잡음 | 속도 잡음 | frame 누락 |
|---|---:|---:|---:|---:|---:|---:|
| Normal | 10 Hz | 100 ms | 300 ms | 0.03 m | 0.05 m/s | 5% |
| Stress | 10 Hz | 250 ms | 300 ms | 0.08 m | 0.15 m/s | 20% |

다음은 R5-B와 동일하게 유지한다.

- 공개 world·seed·지도·차체·안전거리
- 횡단 LEFT·RIGHT reference와 다중 위험 원 경로 reference
- C++ 전체 DWB core, 후보 217개, rollout 41 pose와 shared gate
- ground truth는 관측 생성과 사후 평가에만 사용하고 controller 입력으로 전달하지 않음

## 3. 실행 규칙

1. 기존 최소 release tick 전에는 0 명령과 실제 정지 확인을 유지한다.
2. release는 현재 accepted·fresh 관측과 `READY` 방향 예측, 현재 stop epoch용 새 이동 허가,
   shared gate의 독립 허가가 모두 있을 때만 가능하다.
3. no-frame·stale·invalid·NOT_READY가 발생하면 이전 이동 허가를 재사용하지 않는다.
4. 이동 중 판단 입력이 사라지면 비영점 새 명령을 만들지 않고 shared gate를 통해 감속·정지한다.
5. 실제 정지 완료 전에는 새 reference session이나 재개 허가를 만들지 않는다.
6. 재개를 시험하려면 현재 fresh `READY`, 연속 안전 frame, 새 stop epoch에 결박된 reference와
   새 이동 허가가 모두 필요하다. 이 조건을 구현하지 않은 진단은 정지 상태에서 끝낸다.
7. Stress는 임무 완료를 강제하지 않는다. 판단 불충분으로 안전하게 정지한 결과와 충돌·무단
   재개·안전거리 위반을 서로 다른 결과로 기록한다.

## 4. 기록 항목

- profile별 최초 `READY` tick과 `READY` 구간
- 상태별 tick 수, no-frame·stale 횟수
- 계획 release tick과 실제 release tick
- 이동 후 최초 판단 입력 상실 tick, 보호정지 개시와 실제 정지 완료 tick
- 출발·이탈·통과·재합류·도착 여부
- 최소 Actor·정적 clearance
- gate override, stop epoch, controller session과 hard failure

## 5. 판정

### 완료

현재 profile에서 별도 허가 경계를 지키며 출발부터 도착까지 완료하고 hard failure가 없다.

### 보수적 중단

관측 또는 예측이 불충분해 비영점 명령을 중단하고 실제 정지를 유지한다. 이는 충돌 실패가
아니지만 해당 profile의 임무 완료 증거도 아니다.

### 실패

안전거리·금지구역·출처·최신성·stop epoch·허가 경계를 위반하거나, 판단 입력이 없는데
계속 이동하거나, 예외·비결정성이 발생한다.

## 6. 현재 진입 제한

이 진단만으로 다음을 수행하지 않는다.

- R2-B hard failure 2건 종료
- 카메라·검출·추적 성능 주장
- formal R5-C/R6 receipt 발급
- hidden 생성·실행
- 제품 controller 또는 경로 알고리즘 채택
