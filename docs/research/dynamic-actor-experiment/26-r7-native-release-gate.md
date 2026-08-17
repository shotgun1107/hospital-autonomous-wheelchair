# R7 C++ DWB 시간 자격·후속 진입 Gate 명세

- 상태: v1 측정 실패 뒤 동작 보존형 v2 구현·개발 측정 완료, clean 재자격 대기
- 작성일: `2026-08-17`
- 선행 입력: R6 공개 연속 종단 `17/17`, hard failure `0`, receipt 완료
- 대상: 현재 R5/R6에서 사용하는 C++ 전체 DWB 수치 코어와 C++ 안전 배치 코어
- 비범위: 새 planner, 후보·점수·안전 기준 변경, hidden 실행, 제품 알고리즘 채택

## 1. 목적

R7은 새로운 경로 기능을 만드는 단계가 아니다. R6에서 기능·안전 결과가 닫힌 현재 C++
DWB가 Python 기준과 같은 결과를 내는지 확인하고, 고정된 집 PC에서 한 제어 주기 `50ms`
안에 계산되는지를 별도로 측정한다.

R7 통과는 새 hidden을 자동 실행하는 허가가 아니다. 통과하면 사용자에게 다음 연구 진입을
승인받을 자격만 생긴다.

## 2. 선행 R6 동결값

- R6 기준 commit: `64df95f91e1c514e9407b1eac772afaf697359d6`
- R6 result hash: `e1c086fc836c44d7b793aaccae1a834cff4bdb8b386f39a4f13af2b133168151`
- R6 receipt hash: `2d37f43b720ae1b6ed9050c4968c9a06e0123b8cfe0600ed88302c0f0452cbda`
- R6 case catalog hash: `c284005f40683904f2cedecfddd5b9d74edabe5116ae0025e8cfa9264201fd5a`
- native DLL SHA-256: `dfa167abd8294f6a4ad0e74ce7208cc046a786db39596f55d6461a040cba6bbe`

R7 실행기는 R6 receipt의 자체 hash, 사례 수, hard failure, hidden 미실행, 기준 commit의 조상
관계를 확인한다. R6 output을 찾지 못하거나 변조됐으면 시간 측정을 시작하지 않는다.

## 3. 고정 측정 입력

기존 공개 corpus에서 다음 5개 입력을 그대로 만든다.

| ID | 핵심 부담 |
|---|---|
| `actor-0-free` | Actor 없음, 빈 공간 |
| `actor-1-active` | Actor tube 1개 |
| `actor-2-active` | Actor tube 2개 |
| `corner-static-forbidden` | 정적 장애물·금지 cell·코너 경로 |
| `staggered-risk-multisegment` | 정적 장애물·금지 cell·여러 경로 구간 |

각 입력은 사례 ID, 지도·관측·prediction·차체·경로와 input hash를 묶어 snapshot set hash로
남긴다. 후보 `217개`, 후보당 `41` pose, `2s` rollout, terminal stopping, 비용·tie-break와
shared safety 기준은 바꾸지 않는다.

## 4. Python↔C++ 결과 동일성

각 입력에서 다음 두 controller를 새로 만든다.

- Python DWB 수치 코어
- C++ 전체 DWB 수치 코어

다음 값은 실행시간을 제외하고 정확히 같아야 한다.

- 상태와 선택 명령
- 선택 trajectory
- 실패 이유와 정지 요청
- 후보 진단과 decision trace
- 선택 후보의 안전 근거

C++ 경로를 요청했지만 실제 native core를 사용하지 않은 경우도 실패다.

## 5. 시간 측정

- clock: Python `perf_counter_ns`, monotonic 고해상도 clock
- 실행: worker 없는 부모 process에서 직렬
- controller: C++ 전체 DWB만 측정
- warm-up: 사례당 `30회`
- 본 측정: 사례당 `100회`, 합계 `500회`
- deadline: `50,000,000ns`
- 통과: 초과 `0/500`, maximum `≤50ms`

동일 tick 결과 cache로 시간을 줄이지 않는다. 같은 공간·Actor 수를 유지하면서 tick·관측
revision과 timestamp를 단조 증가시킨 검증 가능한 입력을 사용한다. 각 호출 뒤 native core
사용 여부를 다시 확인한다.

새 controller의 첫 호출은 사례당 별도 cold-start 자료로 기록한다. cold-start는 주 자격을
대체하지 않으며, 50ms를 넘으면 실제 운용에서 첫 명령 전 보호정지가 필요할 수 있다는 한계로
남긴다.

## 6. 실행 환경 기록

- OS·Python·machine ID·CPU 식별자·논리 core 수
- process affinity와 active child process
- 전원 정책
- compiler 경로·버전·C++20·`-O3`·부동소수점 관련 flag
- native source·header·DLL hash와 크기
- warm-up·반복 수, snapshot set hash
- p50·p95·p99·maximum·deadline miss
- peak working set·page fault 관측 가능 범위

runner가 다른 worker를 띄우지는 않는다. 운영체제의 모든 background process를 완전히
제거했다고 주장하지 않으며, 해당 한계는 결과에 남긴다.

## 7. 실행 순서

1. R6 receipt와 Git 상태 확인
2. C++ DWB 전체 코어·안전 배치 코어 DLL 재빌드 및 compiler 정보 기록
3. 5개 snapshot 계약 시험
4. Python↔C++ 결과 동일성 시험
5. worker가 없는 상태에서 직렬 5×100 측정
6. 결과·manifest·Gate 기록
7. 영향권 시험과 전체 회귀
8. 결과 문서와 인수인계 갱신

## 8. 판정

```text
R6 또는 결과 동일성 실패
→ R7 실패, 시간 결과로 덮지 않음, hidden 금지

결과 동일성 통과 + 50ms 미달
→ native 최적화 후보, hidden 금지

결과 동일성 통과 + 0/500 초과
→ R7 시간 자격 통과, 사용자에게 다음 연구 진입 승인 요청 가능
```

어떤 결과도 실제 카메라·실물 휠체어·사람 탑승 안전, 제품 DWB 채택, `G1~G5` 또는 제품
경로분석 7단계 결정을 의미하지 않는다.

## 9. v2 동작 보존형 최적화 경계

v1의 `301/500` 초과 뒤 다음 큰 병목만 측정 근거를 가지고 제거한다.

- 선택된 후보를 Python 안전 검사로 다시 계산하던 중복 평가 제거
- 동일 tick의 controller 입력 전체 hash 중복 계산 제거
- 같은 지도·경로에서 변하지 않는 Manhattan 거리장과 native 입력 배열 재사용
- C++ 생성 배열을 안전 코어와 평가 코어에 복사 없이 전달
- online native 경로에서는 217×41 Python pose를 모두 만들지 않고 선택된 41 pose만 생성
- C++ 안전 코어에서 합집합 occupancy와 중복되는 physical pass 제거
- pose의 sin·cos·footprint를 한 번 계산하고 static·forbidden·Actor 검사에서 공유
- forbidden cell은 합집합 clearance가 0일 때만 별도 원인 분류
- DWB 안전 코어가 읽지 않는 DWA configuration·collision grid는 생성·보관하지 않음

다음 값은 바꾸지 않는다.

- 후보 `217개`, 후보당 `41` pose, `2s` rollout과 terminal stopping
- Actor prediction, 차체 footprint, `0.08m` 여유와 forbidden 판정
- critic 순서·비용·scale·short-circuit·strict tie-break
- 선택 명령·trajectory·실패 이유·후보 진단·안전 근거
- 외부 shared safety gate

공개 eager generator API는 기존처럼 217개 전체 trajectory를 반환한다. 지연 생성은 native
online 내부에만 적용하며 선택 trajectory 수와 전체 후보 진단 수는 그대로다. v2 재자격은
두 native DLL, 두 build script와 위 Python 접착 코드를 모두 source freeze에 포함한다.

`-march=native`와 LTO는 별도 시험에서 이 Windows/Zig 조합의 속도를 개선하지 못하거나
링크에 실패했으므로 적용하지 않는다. 수치 동일성을 흔들 수 있는 fast-math도 사용하지 않는다.
