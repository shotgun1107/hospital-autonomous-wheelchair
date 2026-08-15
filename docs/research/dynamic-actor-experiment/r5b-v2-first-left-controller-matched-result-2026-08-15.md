# R5-B v2 공개 첫 LEFT 컨트롤러 정합 결과

- 상태: 제한적 공개 기능 PASS
- 작성일: 2026-08-15
- 범위: `same-direction-wide-r00 LEFT`, Ideal 관측, RPP와 source-derived DWB
- 비범위: 공개 10-case qualification, 50ms 자격, hidden, 실제 perception, 제품 알고리즘 채택

## 1. 해결한 문제

1차 R5-B는 `0.30m/s` witness를 시간 정보 없는 경로로 바꾼 뒤 `0.20m/s` controller가
Actor 활성 시간 안에 완전 추월하지 못했다. Actor가 사라진 fresh empty frame도 이전 허가를
무조건 폐기해, 안전하게 추월했더라도 재합류와 도착을 끝낼 수 없었다.

v2는 공개 연구용 경로를 `0.20m/s`, 최소 측면 offset `0.65m`, 종방향 completion buffer
`0.20m`로 다시 합성했다. 로봇 뒤 외곽이 현재 directional capsule의 앞 외곽보다 진행 방향
앞에 있고 기존 최소 clearance 이상 떨어진 경우에만 `POST_PASS_COMPLETION`으로 전환했다.
그 증명 뒤의 같은 source fresh empty만 재합류·도착에 사용할 수 있으며, no-frame·stale·invalid·
Actor 진행 회귀는 계속 fail-closed다.

source-derived DWB는 forward section에서 차량 프로필의 기존 최대 전진속도 `0.30m/s`까지
후보를 생성·적용하도록 내부 범위를 일치시켰다. exact-score 동률에서는 heading-aligned forward
section에 한해 더 큰 전진 후보를 먼저 평가한다. 후보 수 `217`, 후보당 `41 pose`, rollout,
terminal stopping, critic 점수, 안전 수치, 최종 Python 재검사와 shared gate는 그대로다.

## 2. 결과

| 항목 | RPP | source-derived DWB + C++ safety batch |
|---|---:|---:|
| ordered overtake tick | 566 | 459 |
| sustained rejoin tick | 788 | 779 |
| terminal completion tick | 806 | 797 |
| Actor 마지막 정합 tick | 599 | 599 |
| 마지막 진행 차이 | 0.591413977186829m | 0.578492559223498m |
| 최대 측면 이탈 | 0.651630615m | 0.6463410207507176m |
| 최소 Actor clearance | 0.289896m | 0.2863172779913767m |
| 최소 static clearance | 0.31577553509462813m | 0.31577553509462813m |
| shared gate override | 0 | 0 |
| hard failure | 0 | 0 |

DWB 독립 900-tick 회귀는 `1 passed`였고 약 `160.18s`가 걸렸다. 영향권은 `52 passed`,
전체 74개 테스트 파일은 4개 process shard에서 합계 `897 passed`, failure·skip `0`이었다.
Ruff·compileall·`git diff --check`도 통과했다. 이는 행동 검증 시간이며
50ms 실시간 자격 통과를 뜻하지 않는다. 사용된 C++는 후보별 동적 안전 배치 코어이고,
경로 section 관리·critic 조합·권한 흐름 전체가 C++로 이식된 것은 아니다.

## 3. 판정과 남은 범위

공개 첫 LEFT에 한해 RPP와 C++ safety batch를 사용한 DWB 모두
`departure → overtake → rejoin → completion`을 실제 closed loop에서 완료했다. 따라서 과거
“DWB는 이 경로에서 거의 못 쓴다”는 1차 실패는 이 사례에는 더 이상 해당하지 않는다.

다만 좌·우 10개 전체, Normal·Stress, 반복·관계·직렬/병렬 결정론, 50ms, full C++ 이식은
아직 검증하지 않았다. receipt는 만들지 않았고 hidden을 실행하지 않았다. 이 결과는 제품 DWB
채택, 실제 사람 안전, G1~G5 또는 제품 경로분석 7단계의 결정 근거가 아니다.
