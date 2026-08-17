# R6 공개 연속 종단 자격 결과

## 판정

- 실행일: `2026-08-17`
- 기준 코드: `64df95f91e1c514e9407b1eac772afaf697359d6`
- 공개 사례: `17/17` 통과
- 안전 위반: `0`
- 전체 회귀: `945 passed`, 실패 `0`
- Ruff·compileall·`git diff --check`: 통과
- R6 판정: **완료**

이 결과는 검증된 합성 `ActorTrack`과 가상 차체를 사용한 공개 simulation 증거다. 실제
카메라·사람·실물 휠체어 안전, 제품 알고리즘 채택이나 hidden 결과를 뜻하지 않는다.

## 공개 사례 결과

| 사례 | 결과 | 주요 사건 tick |
|---|---|---|
| same-direction 00 LEFT | 완료 | 통과 `459`, 재합류 `757`, 완료 `775` |
| same-direction 00 RIGHT | 완료 | 통과 `461`, 재합류 `767`, 완료 `785` |
| same-direction 01 LEFT | 완료 | 통과 `474`, 재합류 `764`, 완료 `782` |
| same-direction 01 RIGHT | 완료 | 통과 `476`, 재합류 `762`, 완료 `780` |
| same-direction 02 LEFT | 완료 | 통과 `471`, 재합류 `764`, 완료 `782` |
| same-direction 02 RIGHT | 완료 | 통과 `473`, 재합류 `762`, 완료 `780` |
| same-direction 03 LEFT | 완료 | 통과 `460`, 재합류 `763`, 완료 `781` |
| same-direction 03 RIGHT | 완료 | 통과 `461`, 재합류 `760`, 완료 `779` |
| same-direction 04 LEFT | 완료 | 통과 `477`, 재합류 `764`, 완료 `782` |
| same-direction 04 RIGHT | 완료 | 통과 `478`, 재합류 `762`, 완료 `780` |
| crossing Ideal LEFT | 완료 | 통과 `370`, 재합류 `610`, 완료 `625` |
| crossing Ideal RIGHT | 완료 | 통과 `295`, 재합류 `541`, 완료 `556` |
| two-risk restop Ideal | 완료 | 두 번째 정지 `232`, 재개 `264`, 완료 `490` |
| crossing Normal LEFT | 완료 | post-pass `632`, 원 경로 복귀 허가 `680`, 완료 `1328` |
| crossing Normal RIGHT | 완료 | post-pass `633`, 원 경로 복귀 허가 `680`, 완료 `1432` |
| crossing Stress LEFT | 보수 정지 통과 | release·controller call·이동 `0`, `HOLDING` |
| crossing Stress RIGHT | 보수 정지 통과 | release·controller call·이동 `0`, `HOLDING` |

모든 사례의 hard failure는 `0`이다. Stress는 도착 성공으로 세지 않고, 현재 정보로 안전한
출발 조건을 만들 수 없을 때 움직이지 않는 요구를 충족한 결과로 판정했다.

## 봉인 정보

- output: `simulation/path_planning_lab/outputs/r6-public-end-to-end-20260817-64df95f/`
- result hash: `e1c086fc836c44d7b793aaccae1a834cff4bdb8b386f39a4f13af2b133168151`
- receipt hash: `2d37f43b720ae1b6ed9050c4968c9a06e0123b8cfe0600ed88302c0f0452cbda`
- source freeze hash: `fcbc6ce253c4c15b7a071ad9b651b9148b4074ca97992b0faef514ed49bc169d`
- case catalog hash: `c284005f40683904f2cedecfddd5b9d74edabe5116ae0025e8cfa9264201fd5a`
- native DWB DLL SHA-256: `dfa167abd8294f6a4ad0e74ce7208cc046a786db39596f55d6461a040cba6bbe`
- 기준 tree: `759cec40c5bc9cd406aaba5987efb6c7eaafc526`

생성 output은 Git에 넣지 않았다. 위 경로의 완주 결과만 R6 근거이며, 중단된 부분 실행은
최종 근거로 승격하지 않았다.

## 진행 중 발견·수정한 문제

1. 첫 실행에서 Ideal 사례의 tick 한도를 world 길이보다 길게 잡아 범위를 벗어났다. 실제
   world duration에서 tick 한도를 계산하도록 고쳤고 회귀시험을 추가했다.
2. 첫 완주본 `cf3b26b`도 `17/17`이었지만 전체 회귀에서 과거 reference가 임의의 중간
   위치에서 새 session처럼 시작할 수 있는 안전 회귀 1건을 발견했다.
3. 중간 위치 재시작은 현재 stop epoch와 현재 tick에 새로 결박된 stop-bound session에서만
   허용하도록 `64df95f`에서 막았다.
4. 수정 전·후 R6 result hash는 동일했다. 정상 공개 동작을 바꾸지 않고 잘못된 우회 진입만
   차단했음을 확인했다.

## 검증

전체 회귀를 네 묶음으로 병렬 실행했다.

- `178 passed`
- `237 passed`
- `251 passed`
- `279 passed`
- 합계: `945 passed`, 실패·건너뜀 `0`

그 뒤 Ruff, Python compileall과 `git diff --check`를 통과했다.

## 남은 범위

- 실제 카메라·FOV·가림·검출·track 생성은 검증하지 않았다.
- Normal 다중 위험 종단은 R6 필수 17개에 포함하지 않았다.
- wall-clock은 R6 합격조건이 아니다.
- hidden은 생성하거나 실행하지 않았다.
- R7 native 시간 자격과 다음 연구 진입 판정은 시작하지 않았다.
- 제품 경로분석 7단계, G1~G5와 제품 알고리즘 채택은 그대로 미수행이다.
