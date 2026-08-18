# R7 새 hidden-v3 실행 명세

## 1. 목적

단일 camera frame 누락 처리와 원 경로 복귀를 고친 뒤, 개발 중 보지 않은 새 관측 잡음과
dropout 순서에서도 같은 결과가 나오는지 한 번 확인한다.

이 시험이 확인하는 범위는 다음뿐이다.

- 동결된 합성 지도와 원형 Actor
- 동결된 Normal·Stress 관측 생성기
- 동결된 DWB·shared safety gate·정지·재개 흐름
- 동결된 회사 PC simulation 환경

실제 카메라, 실제 사람, 실물 휠체어, 제품 알고리즘 채택이나 사람 탑승 안전을 확인하는
시험이 아니다.

## 2. 이름과 과거 결과 처리

- 새 실행 이름: `r7-hidden-v3`
- 새 실행 schema: `r7-hidden-observation-v2`
- 과거 hidden-v2: 실패 입력이 공개된 회귀자료로만 유지
- 과거 hidden-v2 결과는 수정 뒤 최종 합격 근거로 재사용하지 않음
- 새 hidden-v3 결과를 본 뒤 코드를 바꾸면 hidden-v3도 즉시 회귀자료로 전환

## 3. 실행 전 고정값

| 항목 | 고정값 |
|---|---|
| 실행 코드 기준 commit | `8a6275c` |
| 현재 결과·증거 commit | `6b16b32` |
| R7 증거 ZIP | `r7-native-v4-public-qualification-evidence-20260818-8a6275c.zip` |
| ZIP 크기 | `7,773 bytes` |
| ZIP SHA-256 | `3829e14dcf5e548210cdc181bde5dc913743f4f211ca25c3f28c15e2a7016183` |
| R7 receipt | `a971ffeefa83edfd430600261f866d6482467289414d3059592f4f1a95e4ef64` |
| Python↔C++ 동일성 | `5/5` |
| 50 ms 초과 | `0/500` |
| 전체 회귀 | `969 passed`, 실패·건너뜀 `0` |
| 사례 시간 | `1,600 tick = 80 s` |

hidden 실행 전에 다음을 다시 확인한다.

1. 전역·지역 경로, controller, shared gate, 관측·예측 코드가 `8a6275c`의 동결 소스와 같다.
2. 증거 ZIP의 크기와 SHA-256이 위 값과 같다.
3. ZIP 안의 `release-gate.json`이 PASS이고 `qualification-receipt.json`이 `0/500`이다.
4. native DLL의 SHA-256이 receipt와 같다.
5. 실행 전용 복제본의 작업트리가 깨끗하다.
6. 다른 simulation worker와 timing 측정이 실행 중이지 않다.

`run_r7_hidden_observation.py`는 commit `6a272ff`에서 새 v4 증거 ZIP과 실행 commit을
검증하도록 수정됐다. 깨끗한 복제본에서 seed 없는 preflight도 통과했다. 실제 hidden 실행은
아직 승인되지 않았다.

## 4. hidden 사례

총 `20개`를 사용한다.

```text
5개 replica
× LEFT / RIGHT
× Normal / Stress
= 20개
```

같은 `replica + side`의 Normal과 Stress는 같은 기본 관측 seed를 사용한다. profile에 따른
지연·잡음·dropout 차이만 달라진다.

지도, Actor 운동, 경로, 차체, 속도, 안전거리와 tick 수는 공개시험과 같다. hidden으로 새로
숨기는 것은 관측 잡음과 dropout 순서뿐이다. 새로운 병원 구조나 실제 사람 행동을 숨겨
시험하는 것이 아니다.

## 5. seed 생성과 한 번만 실행하는 규칙

사용자가 root seed를 직접 입력하는 옵션은 두지 않는다.

실행기는 한 process 안에서 다음 순서를 지킨다.

```text
코드·증거·DLL·clean 상태 확인
→ 운영체제 난수로 63-bit root seed 생성
→ seed commitment SHA-256 계산
→ case를 만들기 전에 seed-commitment.json 기록
→ pre-run-manifest.json 기록
→ consumed-seed.json에 사용된 seed 기록
→ 20개 사례 실행
```

- root seed는 실행 시작 전에 개발자나 모델에게 보여주지 않는다.
- commitment를 기록한 뒤 코드·수치·판정 기준을 바꾸지 않는다.
- `consumed-seed.json`이 만들어진 seed는 성공·실패·중단과 관계없이 소비된 것으로 본다.
- 중단된 결과는 삭제하지 않고 `partial`로 보존하며 최종 근거로 사용하지 않는다.
- 같은 seed로 수정 코드를 다시 최종 판정하지 않는다.
- 실행 결과를 본 뒤 수치를 조정하려면 새 코드 버전·새 공개 회귀·새 seed가 필요하다.

## 6. 실행 방식

- 각 hidden 사례는 서로 독립된 process에서 실행할 수 있다.
- Normal·Stress pair의 seed, 지도, 경로와 profile은 바꾸지 않는다.
- 결과는 process 완료 순서가 아니라 frozen ordinal `0..19` 순서로 다시 정렬한다.
- 이 회사 PC에서는 최대 worker를 `min(14, logical_cpu/2)`로 둔다.
- worker 수는 결과 의미나 합격 기준에 포함하지 않는다.
- hidden 실행 중에는 50 ms timing 시험을 같이 돌리지 않는다.
- output은 새 경로만 허용하고 기존 결과를 덮어쓰지 않는다.

예상 실행시간은 현재 공개·회귀 실행을 기준으로 약 `3~8분`이다. 이는 운영 예상일 뿐 합격
기준이 아니다.

## 7. 합격 조건

### 공통 20개

- hard failure `0`
- 충돌 `0`
- 금지구역 진입 `0`
- 실제 Actor·정적 장애물 최소 여유 `0.08 m` 이상
- stale·invalid 입력의 새 추진 명령 `0`
- 이전 stop epoch 재사용 `0`
- 늦은 명령 적용 `0`
- 결과 누락·중복·순서 불일치 `0`

### Normal 10개

모두 다음을 만족해야 한다.

- 목적지 도착
- 실제 이동 발생
- Actor 통과 확인 존재
- 통과 뒤 계획 정지와 실제 정지 확인
- 새 stop epoch와 새 안전 frame 11개 확인
- 원 경로용 새 session 시작
- 원 경로 복귀 뒤 목적지 도착
- 최종 상태 `COMPLETED`

### Stress 10개

모두 다음을 만족해야 한다.

- 출발 허가 `0`
- controller call `0`
- 실제 이동 `0`
- release tick `0`
- 최종 상태 `HOLDING`

20개 중 하나라도 위 조건을 만족하지 못하면 hidden-v3 전체는 FAIL이다. 일부 성공률이나 평균으로
대체하지 않는다.

## 8. 판정

### PASS

```text
Normal 10/10 완료
AND Stress 10/10 무출발 정지
AND hard failure 0
```

의미:

> 동결된 합성 관측 조건에서 현재 DWB·shared gate·재개 흐름을 연구 기준선으로 유지할 수 있다.

제품 채택이나 실제 사람 안전을 뜻하지 않는다.

### FAIL

- 결과와 최소 실패 prefix를 무덮어쓰기 방식으로 보존한다.
- 같은 실행에서 코드나 수치를 조정하지 않는다.
- 실패 seed는 공개 회귀자료로 전환한다.
- 원인을 수정하려면 공개시험·전체 회귀·50 ms 자격을 다시 통과한 뒤 새 hidden 버전을 만든다.

### 실행 오류·중단

- 알고리즘 FAIL과 구분해 `infrastructure_failure`로 기록한다.
- 완료된 일부 결과를 최종 판정에 사용하지 않는다.
- partial 산출물을 삭제하지 않는다.
- 소비된 seed는 다시 최종 hidden에 사용하지 않는다.

## 9. 필수 산출물

```text
preflight-manifest.json   # seed 없는 사전검사 때만 생성
seed-commitment.json
pre-run-manifest.json
consumed-seed.json
case-results.json
summary.json
summary.md
hidden-consumption-receipt.json
partial-state.json        # 중단됐을 때만 남음
infrastructure-failure.json # 실행 오류·중단 때만 남음
```

manifest와 receipt에는 최소한 다음을 기록한다.

- HEAD와 tree
- 동결 실행 commit
- R7 증거 ZIP 크기·SHA-256·receipt
- seed commitment
- 사례 catalog hash
- Normal·Stress 사례 수
- worker 수
- 결과 hash
- hard failure 수
- hidden 재사용 금지 표시
- 실제 카메라·제품·사람 안전 증거가 아니라는 제한

## 10. 실제 실행 전 구현 순서

1. [완료] hidden schema를 `r7-hidden-observation-v2`로 올린다.
2. [완료] runner가 새 v4 증거 ZIP과 실행 commit `8a6275c`를 검증하도록 고친다.
3. [완료] runner에 수동 root seed 입력 경로가 없음을 시험한다.
4. [완료] ZIP·DLL·commit·source hash가 맞지 않으면 실행 전에 거부한다.
5. [완료] 20개 catalog·Normal/Stress pair·ordinal·판정 규칙 시험을 통과시킨다.
6. [완료] 고정된 가짜 seed를 사용하는 단위시험만 실행했다. 실제 hidden seed는 만들지 않았다.
7. [완료] Ruff·compileall·직접 영향시험과 전체 회귀를 통과했다.
8. [완료] 구현을 commit `6a272ff`로 푸시하고 전용 clean 복제본에서 preflight만 실행했다.
9. [대기] 사용자에게 실제 hidden-v3 실행 승인을 다시 받는다.
10. [미실행] 승인 뒤 한 번 실행한다.

이 문서 작성만으로 hidden 실행이 승인된 것은 아니다.
