# v6 C++ DWA·충돌 코어 구현 명세

## 1. 목적

Python+NumPy DWA는 동결된 5-case×100 직렬 측정에서 `100/500`회의 50 ms 초과를
기록했다. 별도 Cython 실험에서는 짧은 5-case×10 진단의 중앙값은 줄었지만 초과 횟수는
줄지 않았다. 따라서 다음 재자격 후보는 반복 계산을 Python 객체 단위로 호출하지 않는
독립 C++ 수치 코어다.

이 작업은 DWA 알고리즘 변경이나 제품 채택이 아니다. 기존 v6 공개 계약을 그대로 계산하는
`simulation_only` 동작보존 최적화다.

## 2. 유지하는 계약

- 최대 `7×31=217` 후보
- 후보당 `2.0 s / 0.05 s=40` 구간과 초기 pose를 포함한 `41` pose
- 후진 비활성, `0.20 m/s` 자유주행 목표속도
- 선속도·각속도 dynamic window와 zero sample 교체 규칙
- terminal stopping과 `0.08 m` 최소 표면 여유
- oriented `0.36×0.44 m` footprint
- time-indexed Actor tube와 static·forbidden geometry
- 비용 6종, 가중치, 정렬 방향과 결정론 tie-break
- 외부 shared safety gate와 5 ms safety sweep
- 진단 taxonomy, provenance, `stop_epoch`, deadline과 evaluator

## 3. 책임 경계

```text
Python ControllerSnapshot
  → 한 번의 배열 변환
  → C ABI 한 번 호출
      후보 217개 생성
      후보당 41 pose와 terminal stopping
      coarse static/forbidden/Actor 충돌
      비용·rank 계산과 정렬
  → ranked candidate index와 원인별 결과
  → Python에서 선택 후보만 동결 방식으로 재구성
  → 기존 exact shared safety gate
  → 기존 ControllerCommandResult
```

C++는 Python 객체를 후보·pose마다 호출하지 않는다. 입력은 contiguous numeric buffer와
read-only grid view로 전달하고, 출력은 고정 크기 candidate result 배열이다. Python/C++
경계는 control tick당 한 번만 넘는다.

## 4. C ABI

ABI는 `dwa_core_abi_version()==2`로 시작한다. 입력은 다음을 포함한다.

- 시작 pose, goal과 이전 각속도
- linear·angular sample 배열
- physical·combined·configuration·collision·forbidden occupancy
- combined Chebyshev distance field
- reference polyline
- rollout/terminal 시각별 Actor circle 배열
- 차체 footprint·감속·clearance 수치

후보별 출력은 다음을 포함한다.

- sample index와 `(v,w)`
- nonmoving/accepted/rejected 상태
- 최초 실패 phase·cause·terminal underlying cause
- 실패 시간
- static·Actor·전체 최소 clearance
- 비용 6종, score와 rank key

모든 배열 크기와 포인터는 호출 전에 Python adapter와 C++ 양쪽에서 검증한다. 잘못된 ABI,
크기, non-finite 입력 또는 출력 capacity 부족은 C++ 결과를 적용하지 않고 Python fallback을
사용한다.

## 5. 동작보존 방식

C++는 전체 후보를 평가하고 정렬하지만 최종 출력 궤적을 직접 확정하지 않는다. Python은
C++가 반환한 순서에서 shared gate를 확인할 후보만 기존 `_dynamic_constant_rollout`,
`_coarse_dynamic_candidate_clearance`, `_dynamic_candidate`로 다시 구성한다.

이 방식의 목적은 다음과 같다.

- 최종 궤적 float와 decision trace는 기존 Python oracle을 유지한다.
- 첫 8개 rejection detail도 기존 Python 함수로 다시 계산해 diagnostic digest를 유지한다.
- shared gate를 C++ 내부 결과로 대체하거나 건너뛰지 않는다.
- 일반 free-space에서는 217개 Python trajectory 대신 선택 후보 한 개만 만든다.

공개 대표 입력에서 timing을 제외한 controller semantic digest와 diagnostic digest가 Python
기준선과 같지 않으면 C++ 경로는 자격 실패다.

## 6. 빌드와 fallback

코어는 Python 헤더나 ROS 2에 의존하지 않는 C++20 shared library로 만든다. Python은
표준 `ctypes`로 C ABI를 호출한다.

- Windows 개발: MSVC Build Tools 우선
- 현재 회사 PC 검증: 가상환경에 설치된 Zig C++ compiler 사용 가능
- Linux/ROS 2: GCC 또는 Clang
- native library 부재: 기존 Python DWA로 fallback
- 환경변수 `HOSPITAL_PATH_LAB_DISABLE_CPP_DWA=1`: C++ 경로 강제 비활성

생성 DLL·SO·LIB·PDB와 build cache는 Git에 포함하지 않는다. 저장소에는 C++ source,
header, build script와 동등성 시험만 보존한다.

## 7. 검증 순서

1. 독립 rollout·terminal·geometry 단위 oracle
2. 공개 qualification 대표 5-case의 controller·diagnostic digest 동등성
3. 기존 DWA·충돌 영향권 회귀
4. 전체 Python 실험실 회귀
5. 짧은 5-case 직렬 진단

이번 구현에서는 expanded public, receipt, 5-case×100 최종 자격과 새 hidden을 실행하지
않는다. 짧은 진단에서 50 ms 초과가 남으면 C++ 구현 완료와 공개 자격 통과를 구분한다.

## 8. 중단조건

- 후보 수·rollout·비용·tie-break·clearance를 줄여야만 빨라지는 경우
- Python 기준선과 semantic 또는 diagnostic digest가 다른 경우
- shared safety gate를 건너뛰어야만 50 ms를 만족하는 경우
- 공개 hard-safety 또는 fault 계약이 깨지는 경우

중단은 DWA 알고리즘 계열이나 제품 기능의 기각을 뜻하지 않는다.

## 9. 2026-08-12 구현·진단 결과

구현된 범위는 다음과 같다.

- C++20 코어와 ABI 2, CMake 및 로컬 build script
- `ctypes` 단일 호출 adapter와 ABI version·구조체 크기 검사
- 동일 map revision의 occupancy·configuration·distance field 재사용
- 공유 라이브러리 부재·비활성화·ABI 불일치 시 Python fallback
- C++ ranking 뒤 선택 후보의 Python trajectory 복원과 기존 exact shared safety gate 적용
- 첫 8개 rejection detail과 최종 선택 결과의 기존 Python diagnostic 보존

공개 49 episode × Normal/Stress 대표 1 tick, 총 98개 snapshot을 full 217 후보로
대조했으며 controller semantic digest와 diagnostic digest 불일치는 `0/98`이었다. C++
전용·DWA 영향권 시험은 `25 passed`, 장시간 20-case experiment 통합시험을 제외한 전체
회귀는 `327 passed`, 제외한 runner 단위시험은 `10 passed`였다. 전체 338개 묶음은 기존
장시간 20-case 통합시험에 진입한 뒤 5분 제한으로 중단했으며 중단 전 실패 출력은 없었다.
이 partial 실행을 최종 실험 근거로 사용하지 않는다.

동일 프로세스에서 2회 warm-up 뒤 5개 공개 qualification snapshot을 각각 10회 직렬로
호출한 짧은 진단은 다음과 같았다.

| 경로 | p50 | p95 | 최대 | 50 ms 초과 |
|---|---:|---:|---:|---:|
| Python fallback | `24.630 ms` | `45.657 ms` | `49.285 ms` | `0/50` |
| C++ core | `3.839 ms` | `32.989 ms` | `34.228 ms` | `0/50` |

이 값은 cache warm-up을 포함한 반복 진단이며 머신 상태에 따라 달라진다. 특히 기존
공식 Python 5-case×100의 `100/500` 초과와 직접 교체되는 자격증명이 아니다. expanded
public, receipt, 공식 5-case×100, 새 hidden은 생성하거나 실행하지 않았다. 다음 판단은
별도 직렬 5-case×100 qualification을 통과한 뒤에만 한다.
