# R5-B v2 컨트롤러 속도 정합 및 추월 후 재합류 명세

- 상태: C++ DWB 공개 Ideal `10/10`, RPP 공개 Ideal `4/10` 비교 완료, Normal·Stress 대기
- 작성일: 2026-08-15
- 범위: 공개 Ideal same-direction PASS 5개 × 좌·우 2개, RPP 및 source-derived DWB
- 비범위: hidden, 실제 perception, 안전 임계값 변경, 후보 수·rollout 변경, 제품 알고리즘 채택

## 1. 문제

기존 R5-B 경로 증거는 `0.30 m/s`로 만들었지만 현재 persistent controller의 병진 목표는
`0.20 m/s`다. 공개 10개 실행에서 컨트롤러는 Actor가 존재하는 동안 검증 경로의 추월·재합류
순서를 끝내지 못했다.

추가 진단 결과, 기존 `0.30 m/s` 증거와 새 `0.20 m/s` 증거 모두 재합류 시각은 Actor의
활성 종료 `30.0 s` 이후다. 현재 계약은 fresh empty frame에서도 무조건 이전 허가를 폐기하므로,
경로 속도만 바꿔도 완주할 수 없다.

## 2. 채택할 최소 수정

1. R2 ZIP과 기존 `0.30 m/s` 파생 증거는 회귀 자료로 보존한다.
2. R5-B 실행용 경로만 현재 controller 병진 목표와 같은 `0.20 m/s`로 다시 만든다.
   공개 첫 사례에서 확인된 prediction rollout 여유를 반영해 lateral offset은 최소 `0.65 m`,
   Actor 종료점 뒤 종방향 여유는 `0.20 m`로 둔다. 이 값은 공개 연구용 보정값이지 제품 수치가 아니다.
3. Actor가 보이는 동안 로봇 전체 외곽이 보수적 Actor capsule 전체보다 진행 방향 앞에 있고,
   그 사이 여유가 동결 최소 안전거리 이상임을 현재 관측과 현재 로봇 pose로 증명한다.
4. 이 증명이 성공한 tick부터만 `POST_PASS_COMPLETION` 단계로 전환한다.
5. 이 단계에 들어간 뒤의 fresh empty frame은 자동 허가가 아니다. 같은 mission·stop epoch·map·
   reference session·연속 tick·이전 허가 hash를 유지한 경우에만 RETURN·REJOIN 완료를 계속 허용한다.
6. Actor가 계속 보이거나 다시 보이면 매 tick 같은 추월 여유를 다시 계산한다. Actor가 다시 앞에
   있거나 여유가 부족하면 정지한다.
7. no-frame, stale, invalid, 순서 역행, source 변경은 추월 증명 뒤에도 계속 fail-closed다.

## 3. 추월 완료 증명

원 경로의 단위 진행 벡터를 `u`, 로봇 중심 진행량을 `s_r`, 로봇 외곽의 진행 방향 반폭을
`h_r`, 현재 directional capsule의 두 끝 진행량 최댓값을 `s_a`, capsule cover 반경을 `r_a`라
한다.

```text
robot_rear = s_r - h_r
actor_front = s_a + r_a
pass_margin = robot_rear - actor_front
```

`pass_margin >= minimum_clearance_m`일 때만 완전 추월로 인정한다. 로봇의 yaw가 원 경로와 다르면
충돌 직사각형의 길이와 폭을 원 경로 축으로 투영해 `h_r`을 계산한다. Actor 중심점이나 ground
truth event label만으로는 이 증명을 만들지 않는다.

## 4. 공개 사전 확인 결과

- `0.20 m/s` strict causal PASS: 10/10 존재
- 선택 lateral offset: `0.65~0.66 m`
- 종방향 completion buffer: `0.20 m`
- strict pass 시각: 공개 재검증 결과에 기록
- Actor 활성 종료: `30.0 s`
- strict rejoin 시각: `35.315~36.565 s`

따라서 추월은 Actor가 보이는 동안 검증할 수 있고, 재합류만 명시적인 추월 완료 권한 체인 뒤에
수행해야 한다.

## 5. 필수 악조건 시험

- 추월 증명 전 fresh empty로 전환하면 거부한다.
- 로봇 중심만 앞이고 로봇 뒤 외곽이 capsule을 지나지 않았으면 거부한다.
- 저장된 margin·pose·proof tick·hash를 바꾸면 거부한다.
- 추월 증명 뒤 no-frame·stale·invalid frame은 거부한다.
- 추월 증명 뒤 Actor가 다시 앞에 나타나면 거부한다.
- 추월 증명 뒤 같은 source의 fresh empty와 빈 directional set만 연속 완료 단계로 인정한다.
- 기존 pre-release hold, 실제 정지 확인, 별도 재출발 허가, shared safety gate를 우회하지 않는다.

## 6. 완료 기준

먼저 작은 계약 시험과 공개 첫 LEFT를 통과시킨다. 그다음 공개 10개에서 RPP·C++ DWB 각각에 대해
departure → ordered overtake → sustained rejoin → terminal completion을 확인한다. 실패 시 결과를
FAIL로 기록하며 Actor 활성 시간을 늘리거나 안전거리·prediction tube·shared gate를 약화하지 않는다.

## 7. 1차 구현 결과

- RPP 첫 LEFT: 추월 tick `566`, 재합류 tick `788`, 완료 tick `806`
- C++ safety batch를 사용하는 source-derived DWB 첫 LEFT: 추월 tick `459`, 재합류 tick `779`,
  완료 tick `797`
- 두 실행 모두 shared gate override `0`, hard failure `0`
- fresh empty는 추월 완료 증명 뒤에만 허용하며 no-frame·stale·Actor 진행 회귀는 계속 거부한다.
- DWB는 forward section에서 차량의 기존 `max_forward_speed_mps=0.30` 범위까지 사용한다.
  후보 수·rollout·terminal stopping·critic 점수·shared gate는 바꾸지 않았다.

이는 공개 첫 LEFT의 기능 검증일 뿐 공개 10-case qualification, 50ms 자격, full C++ DWB,
hidden 또는 제품 controller 채택이 아니다.

## 8. C++ DWB 공개 10개 후속 결과

첫 구현 그대로는 공개 `4/10`만 도착했다. 열 사례 모두 추월했지만, 더 빠른 Actor가 있는
여섯 사례에서 DWB가 우회 종점 앞에서 너무 일찍 감속했고 복귀 회전 중 Actor가 다시 따라와
공통 실행 허가가 정지시켰다.

DWB의 BYPASS 점수 경로에만 종점 앞 `0.30m` 가상 앞보기 점을 추가하고, DWB 실행기의 BYPASS
완료 오차만 `0.02m`로 제한했다. 실제 reference, 안전거리, Actor 예측관, 후보 `217`개,
후보당 `41`개 자세, rollout, terminal stopping, critic 가중치와 shared gate는 변경하지 않았다.

정식 소스로 공개 5개 × 좌·우 2개를 다시 실행한 결과 `10/10`이 추월→재합류→도착했고,
gate override와 hard failure는 모두 `0`이었다. 상세 결과는
[C++ DWB 공개 10개 결과](r5b-v2-cpp-dwb-public-10case-result-2026-08-16.md)를 따른다.
이는 Ideal 공개 기능 결과이며 Normal·Stress, 50ms, hidden, 실물 안전과 제품 채택 증거가 아니다.

같은 공개 10개 RPP 비교는 `4/10`만 도착했다. 순번 1·2는 추월 사건 뒤에도 post-pass
completion 허가를 만들지 못했고, 순번 4는 ordered overtake도 만들지 못했다. 공통 안전
장치는 모두 fail-closed로 정지했으며 약화하지 않았다. 상세 결과는
[RPP 공개 10개 비교 결과](r5b-v2-rpp-public-10case-comparison-2026-08-16.md)를 따른다.
