# 카메라 ActorTrack 가정 R단계 재검증

## 범위

- 시작일: `2026-08-16`
- 입력 가정: 카메라 등 상위 영역이 기존 `ActorTrack`을 제공함
- 대상: 동적 지역 기동 연구 `R1~R7`
- 실제 카메라·검출 성능: 별도 Gate
- 제품 알고리즘·G1~G5·제품 경로분석 7단계: 미수행
- hidden: R1~R6 공개 Gate와 별도 사용자 승인 전 금지

이 재검증은 기존 결과를 삭제하거나 R1부터 다시 시작하지 않는다. `2026-08-16`까지 완료한
R1~R5 증거를 유지한 채, 공개 합성 ActorTrack을 사용하는 R6 연속 종단 Gate로 이어간다.
실제 카메라가 있다는 가정은 track을 controller 입력으로 사용할 수 있다는 뜻이며, ground
truth·scenario label·미래 Actor 궤적을 controller에 넘긴다는 뜻이 아니다.

## 진행표

| 단계 | 상태 | 현재 결과 |
|---|---|---|
| R1 prediction 계약 | 완료 | 공개 13개 PASS, hard failure 0 |
| R2-A witness | 완료 | same-direction·횡단·다중 위험 공개 witness 유지 |
| R2-B observation 판단 | 제한 완료 | 기존 ActorTrack profile 계약 사용, 실제 perception은 별도 |
| R3 정적 공간 oracle | 완료 | bounded spatial oracle 공개 자격 유지 |
| R4 local reference | 완료 | signed direction·revision 공개 자격 유지 |
| R5 controller | 완료 | Ideal same-direction·횡단·restop, Normal 횡단 완료, Stress 정지 유지 |
| R6 공개 종단 | 완료 | 공개 `17/17`, hard failure `0`, 전체 회귀 `945 passed`, receipt 생성 |
| R7 native 자격 | v2 완료·통과 | 동일성 `5/5`, 50ms 초과 `0/500`, 최대 `35.190ms`, receipt 생성, hidden 미실행 |

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

## 현재 결과와 다음 후보

[`R6 연속 공개 종단 자격 결과`](r6-public-end-to-end-qualification-result-2026-08-17.md)에
따라 최신 R5-B/C 실행을 중간 checkpoint 없이 묶은 공개 `17/17`을 완료했다. 실제 카메라
FOV·가림은 별도 lane이며 합성 ActorTrack 결과와 합치지 않는다. R7 v1은 50ms 초과
`301/500`으로 실패했지만, 사용자 지시로 동작 보존형 native 최적화를 수행한 v2는 동일성
`5/5`, 50ms 초과 `0/500`, 최대 `35.190ms`로 통과했다. 결과는
[`R7 v1 실패`](r7-native-release-gate-result-2026-08-17.md)와
[`R7 v2 최적화·통과`](r7-native-performance-optimization-result-2026-08-17.md)에 구분해
보존한다. hidden과 제품 채택은 자동으로 시작하지 않는다.
