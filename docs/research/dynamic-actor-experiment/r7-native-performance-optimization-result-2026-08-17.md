# R7 C++ DWB 성능 최적화·재자격 결과

## 판정

- 실행일: `2026-08-17`
- 구현 commit: `8c3b7339144ee6e9d796d34eb45c411e6c1d2654`
- 구현 tree: `d540f85f6041710931dc263cb511168f17f71500`
- Python↔C++ 결과 동일성: `5/5` 통과
- C++ DWB 50ms 시간 자격: **통과**
- 정식 측정: `0/500` 초과, 최대 `35.190ms`
- 전체 회귀: `951 passed`, 실패·건너뜀 `0`
- hidden: 미실행
- 제품 알고리즘: 미채택

R7 v1은 `301/500`회가 50ms를 넘었다. 병목 측정 뒤 후보 수·trajectory·안전 기준을
줄이지 않고 hot path의 중복 연산·Python 객체 생성·배열 복사를 제거했고, 같은 공개 5개
입력에서 R7 v2 시간 자격을 통과했다.

## 변경 전후 정식 500회 측정

측정 조건은 사례당 warm-up `30회`, 본 측정 `100회`, 합계 `500회`, worker 없는 부모
process 직렬 실행으로 동일하다.

| 지표 | v1 | v2 | 변화 |
|---|---:|---:|---:|
| 전체 p50 | `58.269ms` | `10.847ms` | `5.37배` 빠름 |
| 전체 p95 | `250.321ms` | `25.412ms` | `9.85배` 빠름 |
| 최대 | `321.115ms` | `35.190ms` | `9.13배` 빠름 |
| 50ms 초과 | `301/500` | `0/500` | `301회 → 0회` |

| 입력 | v2 p50 | v2 p95 | v2 최대 | 50ms 초과 |
|---|---:|---:|---:|---:|
| Actor 없음 | `5.012ms` | `6.034ms` | `10.507ms` | `0/100` |
| Actor 1개 | `7.401ms` | `11.225ms` | `13.155ms` | `0/100` |
| Actor 2개 | `14.687ms` | `18.943ms` | `23.328ms` | `0/100` |
| 코너·정적 장애물·금지 cell | `24.857ms` | `29.025ms` | `35.190ms` | `0/100` |
| 여러 경로 구간·정적 장애물 | `10.832ms` | `12.901ms` | `17.422ms` | `0/100` |

## 실제 병목과 제거한 일

변경 전 대표 코너 입력의 Python profiler는 약 `2.0M` call과 `0.516s`를 기록했다. 최적화
후 같은 종류의 steady-state 입력은 `202,189` call과 `0.050s`였다. profiler 자체의 계측
부하가 있으므로 이는 정식 wall-clock 자격값이 아니라 호출 경로 감소 근거다.

측정으로 확인한 큰 병목부터 다음 순서로 제거했다.

1. C++에서 이미 판정한 선택 후보를 Python 안전 함수로 다시 계산하던 중복 평가 제거
2. 같은 tick의 controller snapshot 전체 hash를 여러 계층에서 반복 생성하던 작업을 1회로 통합
3. C++ 생성 결과를 Python tuple/object로 전부 바꾼 뒤 다시 C 배열로 포장하던 왕복 제거
4. online 경로에서는 선택된 trajectory만 Python 객체로 만들고 공개 eager API는 그대로 유지
5. 같은 지도·경로의 Manhattan 거리장과 native 배열을 재사용
6. C++ 안전 코어에서 합집합 occupancy와 중복되는 physical clearance pass 제거
7. pose별 sin·cos·footprint를 한 번 계산해 static·forbidden·Actor 검사에서 공유
8. DWB 안전 ABI가 읽지 않는 DWA configuration·collision grid 생성·보관 제거

online Python 객체 생성은 후보 `217개 × 41 pose = 8,897 pose + 217 trajectory`에서
`41 pose + 1 trajectory`로 줄었다. pose와 trajectory 객체 모두 약 `99.54%` 감소했다.
명령 배열 `217×2`와 pose 배열 `217×41×3`은 contiguous `float64`로 한 번 만들고 C++ 안전
코어와 평가 코어에 복사 없이 전달한다.

## 메모리 관측

Windows process 수준 측정에서 500회 구간의 working-set 증가는
`14,639,104 → 13,004,800 bytes`로 약 `11.16%` 감소했고, private usage 증가는
`13,438,976 → 12,464,128 bytes`로 약 `7.25%` 감소했다. page fault 증분은
`253,133 → 11,229`로 약 `95.56%` 감소했다.

이는 Python runtime 전체가 포함된 process 관측값이며 MCU RAM 상한을 증명하지 않는다.
고정 지도에서 속도를 얻기 위해 거리장과 작은 native 작업공간을 유지하는 RAM 교환은 남아
있다. 다만 DWB 안전 코어가 사용하지 않는 격자 3장은 더 이상 만들거나 보관하지 않는다.

## 컴파일러 검토

- 사용: C++20, Zig `0.16.0`, `-O3`
- full core: 수치 재현을 위해 `-ffp-contract=off`, `-fno-builtin-sin/cos`
- `-march=native`: 이 Windows/Zig 측정에서 오히려 느려져 폐기
- LTO: 이 Windows/Zig 공유 라이브러리 링크에서 실패해 폐기
- fast-math: 수치 동일성을 바꿀 수 있어 미적용
- C++ `-Wall -Wextra -Wpedantic`: 경고 없음

근거 없이 flag를 추가하지 않았고, 실제로 빨라지지 않은 변경은 최종 코드에 남기지 않았다.

## 동일성·회귀·산출물

- 상태·선택 명령·trajectory·실패 이유·후보 진단·안전 근거: Python 기준과 `5/5` 일치
- 후보 `217개`, 후보당 `41 pose`, `2s` rollout, terminal stopping: 유지
- Actor prediction·차체 footprint·`0.08m` 여유·forbidden·critic·tie-break: 유지
- 전체 회귀: `951 passed`
- Ruff·compileall·`git diff --check`: 통과
- output: `simulation/path_planning_lab/outputs/r7-native-v2-release-20260817-8c3b733/`
- timing SHA-256: `a88024077f3587e7cd2ccc8fc98bf93ba249602d3579bd1e47e45fc24d4f3e89`
- gate SHA-256: `1fc8ff96d54a454d85eb5d352a140236802b14d54b9e70bc8a35ac545d0822e6`
- receipt SHA-256: `3901a9fbec1b9e538529dcbff36f044f24a261b974411065ba1d24ff1df7136d`
- receipt content hash: `6c5243c7062137bf2cd05345f76a15b8ebb4401d99145d450cf34353d8744575`

## 결론과 경계

현재 집 PC의 동결 공개 5개 입력에서는 C++ DWB가 20Hz 계산시간 자격을 통과했다. 따라서
v1에서 확인된 **현재 구현의 속도 문제는 해결**됐다. 이는 DWB의 제품 채택, 실제 카메라,
실물 휠체어, 환자 탑승 안전 또는 모든 병원 상황의 실시간 보장을 뜻하지 않는다.

R7 통과는 다음 연구를 사용자에게 요청할 자격만 만든다. hidden은 실행하지 않았고,
`G1~G5`와 제품 경로분석 7단계도 시작하지 않았다.
