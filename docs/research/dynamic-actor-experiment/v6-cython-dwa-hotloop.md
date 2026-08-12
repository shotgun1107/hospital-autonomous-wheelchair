# v6 DWA Cython hot-loop 이전 기록

## 1. 목적과 범위

2026-08-12 회사 PC에서 동결된 Python DWA의 병목을 다시 측정하고, 다음 두 부분만
선택형 Cython 확장으로 이전했다.

- 최대 `217`개 후보의 후보당 `41` pose 상수 명령 rollout과 terminal stopping 적분
- 반복되는 구성공간 점유·Actor tube 표면거리·정적 여유 하한의 coarse 충돌 검사

다음 항목은 Python에 그대로 남겼다.

- 후보 수, 시간 horizon, 적분 간격
- 비용식, 가중치, 정렬 방향과 tie-break
- 진단 taxonomy와 결과 provenance
- exact shared safety gate와 5 ms safety sweep
- 권한, `stop_epoch`, deadline, evaluator와 runner

따라서 이 작업은 DWA 정책 변경이나 새 알고리즘 채택이 아니라 같은 Python 실험의
계산 경계만 좁게 옮긴 것이다. Cython 확장을 빌드하지 못하면 동일한 Python 구현으로
자동 fallback한다.

## 2. 이전 전 병목 측정

고정된 공개 qualification 입력 5개를 warm-up한 뒤 각 5회, 총 25회의 `step()`을
`cProfile`로 측정했다. 이 측정은 병목 진단이며 500회 직렬 자격시험이 아니다.

| 구간 | 호출 수 | 누적시간 |
|---|---:|---:|
| `DynamicDwaController.step` | 25 | `2.478 s` |
| coarse 후보 검사 | 4,650 | `1.607 s` |
| Actor 우세 충돌 검사 | 4,650 | `1.201 s` |
| 41-pose rollout | 4,650 | `0.485 s` |
| 개별 coarse point 검사 | 44,945 | `0.354 s` |
| 정적 `clearance()` | 56,925 | `0.213 s` |

결론은 비용 정렬보다 `217 × 41` 궤적 생성과 그 궤적의 반복 충돌 검사가 주 병목이라는
것이다.

## 3. 구현 경계

| 파일 | 책임 |
|---|---|
| `src/hospital_path_lab/_dwa_hotloop.pyx` | rollout, terminal stopping, coarse Actor·구성공간 충돌 반복문 |
| `src/hospital_path_lab/dwa_hotloop.py` | 선택형 확장 import와 source-only fallback |
| `src/hospital_path_lab/local_algorithms/dwa.py` | 기존 정책·비용·gate를 유지하며 Cython 함수 호출 |
| `setup_cython.py` | 표준 C/C++ build toolchain을 사용하는 in-place 확장 빌드 |
| `scripts/profile_dwa_hotloop.py` | 공개 5-case 병목·짧은 성능 진단 |
| `tests/test_dwa_cython_hotloop.py` | Cython과 Python fallback의 비시간 의미 동등성 |

CPython `math` 결과의 마지막 비트까지 동결 oracle과 같게 유지하기 위해 rollout의
삼각함수 경계는 CPython `math`를 호출하고, 41-step 반복·상태 갱신·객체 조립을 Cython
루프에서 수행한다.

## 4. 빌드와 fallback

Windows에서 Visual Studio C++ Build Tools가 있는 표준 개발환경은 다음처럼 빌드한다.

```powershell
cd .\simulation\path_planning_lab
..\..\.venv\Scripts\python.exe -m pip install -e ".[acceleration]"
..\..\.venv\Scripts\python.exe .\setup_cython.py build_ext --inplace
```

회사 PC에는 MSVC가 없어 검증 시 가상환경 전용 `ziglang 0.15.2`로 동일 Cython 생성 C를
컴파일했다. Zig는 프로젝트 runtime 의존성이 아니며 컴파일된 `.pyd`, 생성 `.c`, build
cache는 Git에 포함하지 않는다.

확장을 강제로 끄고 fallback을 확인하려면 새 프로세스에서 다음 환경변수를 사용한다.

```powershell
$env:HOSPITAL_PATH_LAB_DISABLE_CYTHON_DWA = "1"
```

## 5. 동등성과 짧은 진단 결과

- Cython rollout은 직선·좌회전·우회전 표본에서 Python 동결 oracle과 객체·float가
  완전히 같았다.
- 공개 qualification 대표 입력 5개에서 controller semantic digest와 candidate
  diagnostic digest가 Python fallback과 모두 같았다.
- Cython 적용 뒤 같은 25-call `cProfile`의 전체 누적은 `2.316 s → 1.502 s`, coarse
  검사는 `1.607 s → 1.000 s`, Actor 우세 검사는 `1.201 s → 0.580 s`, rollout은
  `0.485 s → 0.286 s`로 감소했다.
- 별도 5-case×10 짧은 진단에서 중앙값은 Python `30.303 ms`, Cython `26.223 ms`,
  p95는 각각 `59.705 ms`, `56.437 ms`였다. 50 ms 초과는 양쪽 모두 `10/50`으로
  남았다.

짧은 실행은 표본이 작고 실행 순서·OS 부하의 영향을 받으므로 자격 판정에 사용하지
않는다. 현재 결론은 **요청한 hot loop 이전과 동작보존 검증은 완료했지만 DWA 50 ms
무실패 자격은 아직 통과하지 못했다**이다.

## 6. 이번 작업에서 수행하지 않은 것

- expanded public full qualification과 receipt 생성
- 5-case×100 최종 직렬 timing 자격 재실행
- 새 hidden seed 생성·commitment·실행
- 비용·Actor tube·clearance·후보 수 완화
- C++ core 이전
- 제품 알고리즘 채택, G1~G5 또는 경로 분석 7단계 결정

다음 단계로 진행하려면 공개 자격을 다시 열기 전에 Cython 이후 남은 `clearance()`,
reference-path 비용과 exact gate 병목을 같은 동작보존 원칙으로 별도 검토해야 한다.
