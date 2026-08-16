# 카메라 ActorTrack 가정 R단계 재검증

## 범위

- 시작일: `2026-08-16`
- 입력 가정: 카메라 등 상위 영역이 기존 `ActorTrack`을 제공함
- 대상: 동적 지역 기동 연구 `R1~R7`
- 실제 카메라·검출 성능: 별도 Gate
- 제품 알고리즘·G1~G5·제품 경로분석 7단계: 미수행
- hidden: R1~R6 공개 Gate와 별도 사용자 승인 전 금지

이 재검증은 기존 결과를 삭제하지 않는다. 공개 합성 ActorTrack을 사용해 prediction, witness,
공간 경로, reference, controller와 종단 Gate를 다시 순서대로 확인한다. 실제 카메라가 있다는
가정은 track을 controller 입력으로 사용할 수 있다는 뜻이며, ground truth·scenario label·
미래 Actor 궤적을 controller에 넘긴다는 뜻이 아니다.

## 진행표

| 단계 | 상태 | 현재 결과 |
|---|---|---|
| R1 prediction 계약 | 완료 | 공개 13개 PASS, hard failure 0 |
| R2-A witness | 대기 | 기존 공개 witness를 새 순서에서 재검증 예정 |
| R2-B observation 판단 | 대기 | 실제 카메라 대신 기존 ActorTrack profile 계약 사용 |
| R3 정적 공간 oracle | 대기 | R2 검증 source만 입력 |
| R4 local reference | 대기 | revision·signed direction 재검증 |
| R5 controller | 대기 | RPP·DWB 공개 기능 재검증 |
| R6 공개 종단 | 대기 | 기능·안전 report·receipt |
| R7 native 자격 | 대기 | R1~R6 통과 뒤에만 검토 |

## R1 실행 결과

새 output:

`simulation/path_planning_lab/outputs/r1-camera-assumed-rerun-20260816/`

결과:

- 공개 episode: `13`
- motion transition: `5420`
- motion violation: `0`
- Ideal Capsule coverage: `26257/26257`, `100%`
- Normal Capsule coverage: `19170/20118`, `95.2878%`
- Stress Capsule coverage: `145/145`, `100%`
- hard failure: `0`
- 결과 hash: `53d485de797880ba22ea1dc0e8eddb96c67efa662eeca71f1ae4e37359da0cdc`
- 표적 회귀: `52 passed`
- Ruff: 통과

Normal의 Capsule 누락과 Normal·Stress 2σ 밖 관측은 통계적 한계로 유지한다. 수치를 결과에
맞춰 완화하지 않았다. 공개 corpus에는 가속·감속·정지·회전 Actor 운동이 없어 해당 일반화는
증명되지 않았다.

## 다음 단계

R2-A의 공개 witness·음성 판정과 R2-B의 profile replay를 기존 source 그대로 다시 실행한다.
실제 카메라 FOV·가림은 별도 lane이며, 합성 ActorTrack 경로 연구 결과와 합치지 않는다.
