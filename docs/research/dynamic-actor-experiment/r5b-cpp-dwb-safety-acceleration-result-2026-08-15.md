# R5-B C++ DWB 안전 배치 가속 결과

- 상태: **구현·동작 보존 확인 완료, R5-B 기능 판정은 계속 FAIL**
- 작성일: 2026-08-15
- 범위: source-derived Python DWB의 후보별 동적 안전 판정만 C++20 배치 코어로 가속
- 비범위: 후보·점수·tie-break·안전 기준 변경, hidden, 제품 controller 채택, G1~G5, 경로 분석 7단계

## 1. 결론

순수 Python DWB의 가장 큰 병목은 217개 후보의 41-pose rollout과 terminal stopping을
후보마다 Python으로 반복 검사하는 부분이었다. 이 안전 판정을 C++20 공유 라이브러리로
옮기되 다음 항목은 바꾸지 않았다.

- 후보 `217개`
- 후보당 pose `41개`
- 2초 rollout과 5ms sweep
- terminal stopping과 50ms command apply 검사
- 차체 외곽, static·forbidden clearance, 원형 Actor tube와 방향성 Capsule
- critic 순서·scale·score·tie-break
- 최종 선택 후보의 기존 Python authoritative safety 재검사
- controller 뒤의 기존 shared safety gate

C++ 라이브러리가 없거나 입력·ABI가 맞지 않으면 기존 Python 경로로 돌아간다. C++ 결과만
믿고 모터 명령을 허가하지 않는다.

## 2. 구현

- C++ core: `simulation/path_planning_lab/native/dwb_safety_core.cpp`
- C ABI: `simulation/path_planning_lab/native/dwb_safety_core.h`
- Python adapter: `simulation/path_planning_lab/src/hospital_path_lab/cpp_dwb_safety_core.py`
- DWB batch critic 연결:
  `simulation/path_planning_lab/src/hospital_path_lab/dynamic_trajectory_constraints.py`
- build script: `simulation/path_planning_lab/scripts/build_cpp_dwb_safety_core.py`

플랫폼별 DLL·SO·dylib는 Git에 넣지 않는다. 선택 의존성은 `.[native]`이며 Zig가 없으면
환경의 `clang++`, `g++` 또는 `CXX`를 사용한다.

```powershell
python -m pip install -e ".\simulation\path_planning_lab[native]"
python .\simulation\path_planning_lab\scripts\build_cpp_dwb_safety_core.py
```

환경변수 `HOSPITAL_PATH_LAB_DISABLE_CPP_DWB=1`은 native 경로를 강제로 끄고 Python
기준선을 실행한다.

## 3. 동작 보존 검증

원형 Actor prediction과 방향성 Capsule 각각에서 동일한 217개 trajectory를 C++과 기존
Python shared-safety evaluator에 넣고 모든 safe/failure 판정을 후보 순서대로 비교했다.
안전 후보는 최소 static·Actor clearance도 `2e-12m` 절대오차 안에서 일치했다.

또한 다음을 확인했다.

- 공개 첫 DWB tick의 command·trajectory·후보 진단·선택 evidence exact 일치
- 실제 R5-B 첫 LEFT의 최초 native batch까지 Python/C++ result exact 일치
- 같은 R5-B 입력 97틱 trace hash exact 일치
- C++이 선택한 최종 후보는 기존 Python 안전 평가를 다시 통과해야만 채택
- 최종 shared gate는 그대로 유지

영향권 1차 실행은 과도하게 넣은 시험 가정 2건을 제외한 실제 구현 시험이 모두
통과했다. 그 두 건은 C++/Python 불일치가 아니라 모든 후보가 동일 실패인 장면에
`SAFE 후보도 반드시 존재`한다고 잘못 요구한 시험 문제였으며, 후보별 exact parity 검사는
유지한 채 해당 가정만 제거했다.

## 4. 성능과 공개 R5-B 결과

### 짧은 구간

- 정적 공개 첫 DWB call: 약 `1.55s → 0.13s` (`약 12배`)
- 실제 R5-B 97틱 구간: `8.245s → 2.685s` (`약 3.07배`)
- 위 97틱 trace hash:
  `4d483244d20e1f1892a2f5c832b05494f985518caf6ca5341bf2cbf19a18ffd3`

### 실제 첫 LEFT 610틱

동일 공개 사례를 Actor disappearance가 반영되는 tick 610까지 C++ 경로로 실행했다.

```text
wall-clock                                    109.597374 s
기존 Python wall-clock                       3,074.4671572 s
대략적인 전체 사례 가속                      28.05배
controller call                              564
최초 controller / 최초 이동                  tick 40 / tick 44
departure                                    tick 115
overtake / sustained rejoin                   없음 / 없음
마지막 Actor 대비 진행 차이                   -0.08294597353371147 m
최대 원 경로 이탈                            1.1052723922549408 m
최소 Actor / static clearance                0.6471500000000001 / 0.31577553509462813 m
shared gate override                         0
trace hash                                   e46737ee4214bd15f36371f8d0e158d24b64cf57c9423b5551057e285f152d22
```

위 위치·거리·실패·trace hash는 이전 순수 Python 610틱 결과와 정확히 같다. 따라서 C++
이식은 계산시간을 줄였지만 R5-B 기능 실패를 고치지는 않았다.

### 최종 회귀

- C++ core 재빌드: PASS
- 원형·방향성 217개 후보 parity: PASS
- R5-B 97틱 Python/C++ result exact parity: PASS
- Ruff·compileall·`git diff --check`: PASS
- 전체 pytest: `892 passed` (`811.74s`), failure·skip `0`

## 5. 현재 판정

- C++ DWB 안전 배치 경로: 동작 보존 L1 회귀 근거 확보
- R5-B: 계속 `FAIL`, receipt `0`
- 실패 의미: 안전 위반이나 경로 부재가 아니라 Actor가 존재하는 동안 추월·재합류 미완료
- 다음 기능 연구: 안전 수치를 완화하지 않고 `0.30m/s` witness timing과 `0.20m/s`
  controller 계약 불일치를 해결
- 아직 아님: 50ms native 자격, 전체 공개 R5-B 성공, hidden, 제품 DWB 채택
