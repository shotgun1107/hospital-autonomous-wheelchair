# ADR 0014: R5 Section-bound 제한 후진 Translation

- 상태: 사용자 승인 — 개인 연구 명세, 팀·제품 합의 전
- 날짜: `2026-08-14`
- 범위: R3→R4→R5 Python `simulation_only` 연구 계약
- 구현 상태: R4 v2 signed reference 구현·clean public qualification 완료. R5 v2 공통
  executor·RPP·DWB signed 실행은 대표 `wide-straight-left`에서 실제 후진과 종단 완료, hard
  failure·deadlock·gate rejection `0`을 확인했다. 전체 public 21-case qualification과 receipt는
  아직 미완료

## 배경

R3 bounded lattice는 `REVERSE_ONE_TRANSLATION`을 허용하고, R4 v1 public ready 8개는 모두
reverse translation edge를 정확히 한 개 포함했다. 그러나 R4 v1은 reverse를 limitation으로만
남겼고 R5 v1 persistent RPP·DWB는 음의 차체 선속도를 생성하지 않았다. 그 결과 R5-A 첫 clean
public 실행은 `21/21`을 끝냈지만 ready 8개 중 5개가 실패했고 receipt를 만들지 못했다.

여기서 다음 두 의미를 구분한다.

- 휠 단위 역회전: differential drive 제자리회전에 필요할 수 있는 저수준 모터 방향이다.
- 차체 후진 translation: 차체 기준 signed linear velocity `v < 0`으로 뒤로 이동하는 R5 명령이다.

제자리회전 지원이 차체 후진을 자동 허용하는 것은 아니며, 차체 후진 허용도 실제 모터·센서·제품
정책을 확정하는 것은 아니다.

## 결정

R5의 다음 연구 계약은 **R4가 명시한 reverse translation section에서만** 제한 후진을 허용한다.

```text
FORWARD section → 0 <= v <= 0.30m/s
REVERSE section → -0.10m/s <= v <= 0
ROTATE/HOLD     → translation v = 0
```

다음 조건을 함께 고정한다.

1. R4 v2는 각 translation section에 `travel_direction=FORWARD|REVERSE`를 명시한다.
2. 한 section 안에 forward와 reverse를 섞지 않는다. 방향이 바뀌면 section을 분리한다.
3. 방향 source는 R3 primitive metadata이며 pose 기하에서 사후 추측하지 않는다.
   `FORWARD_ONE_TRANSLATION`과 `REVERSE_ONE_TRANSLATION`만 signed translation이다.
   R3의 `ANCHOR_CONNECTOR`는 격자 시작·종료를 잇는 추상 연결 증거이므로 방향을 추론하지 않고
   `NONE`인 비실행 연결 구간으로 보존한다. 실제 변위가 있는 connector는 양 끝 정지 표식이
   있을 때만 R4 reference에 남긴다.
4. RPP와 DWB는 active section의 방향 밖에 있는 nonzero 선속도 후보를 만들거나 선택하지 않는다.
5. forward↔reverse 전환 전 common executor가 제한 감속과 실제 정지 3 tick을 확인한다.
6. reverse는 기존 `0.10m/s` 상한, 선가속·감속 한계와 50ms 적용 지연을 그대로 사용한다.
7. reverse rollout과 terminal stopping은 차체 뒤쪽 swept footprint까지 static·forbidden·Actor
   clearance를 검사하며 최종 shared gate를 우회하지 않는다.
8. 필수 후방 판단정보가 없거나 stale·invalid이면 nonzero reverse를 허용하지 않고 정지한다.
9. 경로에 reverse section이 없는 상태에서 controller가 임의로 후진해 새 경로를 만들지 않는다.
10. R3의 reverse multiplier와 결정론 tie-break는 유지해 forward 해가 같거나 더 낫다면 forward를
    우선한다.

## 버전과 증거 보존

- R3/R4 v1 결과·receipt와 R5-A 첫 실패 output은 변경하지 않는다.
- `travel_direction` 추가는 R4 schema·builder·validator·hash를 바꾸므로 R4 v2로 재qualification한다.
- R5 signed controller·executor·runner도 새 version과 새 output 경로를 사용한다.
- 기존 no-reverse v5/v6/v7 동적 controller 결과는 역사적 회귀자료로 유지한다.

## 필수 검증

- forward/reverse section metadata와 source primitive exact 대응
- 방향 metadata tamper·혼합 방향 section 거부
- RPP와 DWB의 section-bound signed candidate 범위
- forward→stop→reverse와 reverse→stop→forward 전환
- reverse 중 뒤쪽 collision·forbidden·Actor·terminal stopping
- stale/no-frame/invalid source에서 nonzero reverse 금지
- left/right mirror와 horizontal/vertical 관계
- R4 ready 8개의 reverse edge 의미 보존 paired 완료
- 반복 결정론·serial/process parity·hard failure 0 뒤에만 새 receipt 생성

## 범위 밖

- 실제 모터 드라이버 방향·배선·토크 결정
- 후방 카메라·라이다·초음파의 종류와 배치
- 실제 사람 탑승 자동 후진
- 공공 병원 공간에서의 제품 후진 정책
- G1~G5 또는 제품 경로 알고리즘 채택

실제 장치에서 후진을 허용하려면 차체·센서·정지거리·후방 관측·승차감과 운영정책을 별도로
합의하고 축소 실물에서 검증해야 한다.
