# R5-A 공개 Persistent Controller 1차 Qualification 결과

- 실행일: 2026-08-14
- 실행 source commit: `7e2264281cff67db054baafd0a04965b75324f8c`
- 실행 source tree: `beb6faf08d3fc8abb73b3304704005f73f084973`
- output: `simulation/path_planning_lab/outputs/persistent-controller-public-20260814-r5a-v1-7e22642/`
- 상태: **완료했으나 qualification 실패**
- receipt: **미생성**
- hidden: **미사용**

## 1. 한 줄 결론

R5-A runner와 증거 수명주기는 정상 동작했지만, R4의 8개 `REFERENCE_SET_READY` 경로가 모두
정확히 한 개의 reverse translation edge를 포함하는 반면 R5 DWB는 reverse를 금지하므로
R4 reference 계약과 R5 실행 profile이 호환되지 않았다. 이 실행은 RPP 또는 DWB의 제품 적합성
판정이 아니며, 현 상태에서 어느 controller도 채택하지 않는다.

## 2. 실행 무결성

- public case: `21/21`
- ready case: `8`, 각 case에서 fresh RPP 뒤 fresh DWB paired 실행
- non-ready case: `13`, controller call `0`
- repeat determinism: PASS
- serial/process parity: PASS
- complete state: 생성
- qualification receipt: 미생성
- summary semantic hash:
  `2e22bdb01a066b2ebef1e8279201404f2ee5ecbda9e6e5deda05a788e067429a`
- manifest content hash:
  `807da2a8d45b732b7560e892595eb30b84905b1fbd4665ed76e100051f2028bf`

runner는 실패를 성공으로 바꾸지 않았고, hard failure 때문에 fail-closed로 receipt를 만들지 않았다.

## 3. ready 8개 결과

| case | RPP | DWB | 핵심 관측 |
|---|---|---|---|
| `wide-straight-left` | 완료, tick 417 | 완료, tick 393 | 위치 경로는 완료했지만 reverse edge를 큰 방향 전환으로 우회 실행 |
| `wide-straight-right` | deadlock·tracking error | deadlock | reverse edge가 첫 lateral departure에 나타남 |
| `wide-mirror-left` | 완료, tick 417 | 완료, tick 392 | 위치 경로는 완료했지만 reverse edge를 큰 방향 전환으로 우회 실행 |
| `wide-mirror-right` | deadlock·tracking error | deadlock | 오른쪽과 같은 구조적 실패 |
| `vertical-left` | 완료, tick 417 | 완료, tick 393 | 회전 관계에서도 왼쪽과 같은 우회 실행 |
| `vertical-right` | deadlock·tracking error | deadlock | 오른쪽과 같은 구조적 실패 |
| `crossing-static-left` | active section/window 불일치 뒤 gate override | deadlock | reverse edge 뒤 crossing window 문제가 연쇄 발생 |
| `crossing-static-right` | active section/window 불일치 뒤 gate override | deadlock | mirror와 같은 연쇄 실패 |

완료 3건은 reverse primitive의 heading을 그대로 실행한 성공이 아니다. 두 controller 모두 음수
선속도를 사용하지 않았고, 위치 polyline을 따라가기 위해 차체 방향을 크게 바꿨다. 따라서 이 3건을
reverse reference 실행 성공으로 해석하지 않는다.

## 4. 확인된 공통 원인

R3/R4는 `REVERSE_ONE_TRANSLATION`을 simulation-only primitive로 허용한다. R4 명세도 reverse를
limitation·metadata로 보존할 뿐 실제 후진 허용을 결정하지 않는다고 명시한다. 실제 ready 8개
source-reference를 검사하면 모두 displacement와 저장 heading의 내적이 정확히 `-1.0`인 edge를
한 개씩 포함한다.

| case 계열 | reverse edge 위치 |
|---|---|
| wide/vertical left | return section |
| wide/vertical right | first lateral depart section |
| crossing-static left/right | diagonal depart section |

반면 R5 DWB v1은 `reverse = disabled`이고 RPP translation도 양의 목표 선속도만 만든다. 따라서
현재 frozen R5 실행 profile은 R4 ready 8개를 의미 그대로 실행할 수 없다. 왼쪽 성공·오른쪽 실패
비대칭은 controller 우열이 아니라 reverse edge가 어느 section에 배치됐는지에 따른 결과다.

## 5. 별도 runner 판정 오류

원본 실행은 crossing-static reference가 wide reference와 section 종류 목록이 완전히 같아야 한다고
비교해 `crossing-static-*:reference_section_order` 2건을 기록했다. crossing reference에는 원 경로를
계속 따라가는 추가 `RETURN` section이 있으므로 exact equality는 잘못된 판정이다.

실행 결과는 보존하고, 후속 source에서는 wide section 순서가 crossing section 안에 순서대로
포함되는지만 검사하도록 수정했다. 이 수정은 reverse 불일치, controller deadlock, window failure 또는
qualification 실패를 성공으로 바꾸지 않는다.

## 6. 후속 연구 선택

첫 실패 뒤 다음 세 방향을 분리했다.

1. R5에서 reverse를 허용하고 RPP·DWB 모두 signed translation을 구현한다.
2. reverse primitive를 명시적 rotate→forward→rotate 기동으로 변환하는 새 R4/R5 계약을 만든다.
3. R3/R4에 forward-only 별도 lattice lane을 만들고 새 public reference set으로 재qualification한다.

사용자는 2026-08-14에 1번을 **Python simulation 연구 방향**으로 승인했다. 이에
[`ADR 0014`](../../decisions/0014-section-bound-bounded-reverse-translation.md)는 R4가 명시한 reverse
section에서만 최대 `0.10m/s` 제한 후진을 허용한다. 자유 후진, 제품 후진 정책, 실제 사람 탑승
후진을 승인한 것은 아니다. 기존 R4 receipt와 이번 실패 output은 덮어쓰지 않는다.

## 7. 다음 게이트

- R4 v2 `travel_direction`과 R5 v2 section-bound signed translation을 구현한다.
- 새 version·새 source hash·새 output 경로를 사용한다.
- R4/R5 contract·validator·controller·관계시험을 함께 갱신한다.
- 변경 뒤 ready 전체를 다시 실행한다.
- hard failure 0과 모든 기능·관계·repeat·process parity를 통과하기 전 receipt를 만들지 않는다.
- R5-B/C, hidden, 제품 controller 채택, G1~G5와 경로 분석 7단계는 시작하지 않는다.
