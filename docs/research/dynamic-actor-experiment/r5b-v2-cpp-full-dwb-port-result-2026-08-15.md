# R5-B v2 C++ 전체 DWB 수치 코어 이식 결과

- 상태: 공개 첫 LEFT 기능 보존 PASS
- 작성일: 2026-08-15
- 범위: source-derived DWB의 후보 생성·궤적 적분·점수 계산·선택·거리 지도
- 비범위: 공개 10-case qualification, 50ms 자격, hidden, 제품 알고리즘 채택

## 1. 옮긴 범위

기존에는 후보별 동적 안전 검사만 C++였고, 후보 217개 생성·후보당 41개 자세 적분·나머지
점수 계산·최종 선택은 Python이었다. 이번 이식으로 다음 수치 계산을 C++20 코어가 담당한다.

1. 현재 속도에서 가능한 선속도·각속도 축과 217개 후보 생성
2. 각 후보의 2초·41개 자세 constant-twist 적분
3. ProjectSafety, RotateToGoal, Oscillation, GoalAlign, PathAlign, PathDist,
   GoalDist 순서의 7개 critic 계산
4. 기존 short-circuit와 생성 순서 기반 exact tie 처리
5. 최저 점수 후보 선택
6. 4방향 Manhattan 거리 지도 생성

동적 안전 수치 배치는 기존 별도 C++ 코어를 그대로 사용한다. Python은 프로젝트 계약 검증,
reference section·session 관리, 관측·예측 입력 결박, C++ 입력·출력 변환, 선택 결과 기록과
독립 shared safety gate를 담당한다. 따라서 여기서 말하는 `전체 DWB C++`는 지역 DWB 수치
알고리즘 전체를 뜻하며 휠체어 시스템 전체를 C++로 바꿨다는 뜻이 아니다.

native 라이브러리가 없거나 ABI가 맞지 않으면 기존 Python reference로 되돌아간다. 생성된
DLL·SO·dylib는 로컬 빌드 산출물이며 Git에 넣지 않는다.

## 2. 동작 보존 결과

- 일반 217후보 격자의 선속도·각속도·명령 순서는 Python과 정확히 일치했다.
- 자세 적분은 C/Python 삼각함수의 극미한 반올림 차이만 허용하고 `1e-15m/rad` 안에서
  일치했다.
- 정방향, 역방향, 정방향 exact-score tie 우선순위가 모두 Python과 일치했다.
- C++ Manhattan 거리 지도는 장애물이 있는 독립 Python oracle과 정확히 일치했다.
- 정상 명령과 전 후보 탈락 사례에서 command·trajectory·failure·decision trace가 Python과
  일치했다.
- 첫 LEFT 97틱 release trace가 Python reference와 정확히 일치했다.
- 첫 LEFT 900틱은 추월 `459`, 재합류 `779`, 완료 `797`, gate override `0`, hard failure `0`을
  유지했다.

## 3. 실행시간

후보 루프만 C++로 옮긴 첫 중간판은 거리 지도를 Python이 매번 다시 만들어 900틱이
`170.91s`로 끝났다. 이 상태는 기존 C++ 안전 배치판보다 빠르다고 볼 수 없어 완료로 처리하지
않았다. Manhattan 거리 지도까지 C++로 옮긴 최종판은 같은 900틱 시험을 `108.37s`에
완료했다. 같은 코드에서 97틱 전체 pipeline 단순 비교는 Python DWB `3.2046s`, C++ 전체 DWB
`3.0433s`였다.

이 시간에는 DWB 밖의 reference·관측·prediction·shared gate 계산도 포함된다. 고정 머신
반복 측정이 아니므로 50ms 자격 결과로 사용하지 않는다.

## 4. 검증

- C++ 전체 DWB 전용: `5 passed`
- DWB·persistent adapter·R5-B 직접 영향권: `84 passed`
- 변경 뒤 첫 LEFT 900틱 독립 회귀: `1 passed in 108.37s`
- 전체 75개 테스트 파일 4개 process shard: 합계 `905 passed`, failure·skip `0`
- Ruff·compileall·C++ 재빌드·`git diff --check` 통과

## 5. 결론과 남은 범위

첫 LEFT에서 기존 Python DWB의 기능 결과를 유지한 채 지역 DWB 수치 계산 전체를 C++로
이식했다. 이전의 “C++는 안전 배치뿐”이라는 제한은 해소됐다. 다만 이것은 첫 LEFT와 직접
영향권의 공개 연구 증거다. 좌·우 공개 10-case, Normal·Stress, 고정 머신 50ms, receipt와
hidden은 아직 수행하지 않았다. 제품 알고리즘 채택, G1~G5와 제품 경로분석 7단계도
결정하지 않았다.
