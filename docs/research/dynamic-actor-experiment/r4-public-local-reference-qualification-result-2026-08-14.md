# R4 지역 Reference·Sliding Window 공개 qualification 결과

- 실행일: 2026-08-14
- 판정: **PASS — 동결된 offline static reference 연구 범위**
- 평가 commit: `f43fbbf80c1659372c86f7c48f752fde075bed24`
- 평가 tree: `13acb52f8e24952a0afb6fbb0aa7177b6872e3c7`
- hidden 사용: 없음
- 제품 알고리즘 채택: 하지 않음

## 1. 실행 범위

R3의 동결 public 21-case를 같은 입력 순서로 사용했다. 각 case의 R3 검색과 R4 변환·독립
검증은 14개 process로 병렬 계산했고, 하나의 reference 안에서 20 Hz knot cursor와 sliding
window sequence는 같은 manager로 직렬 실행했다. process 완료 순서, worker 수와 Python
wall-clock은 semantic 판정에서 제외했다.

```text
R3 feasible LEFT/RIGHT
  → R4 SPATIAL_ONLY immutable reference
  → independent static swept validation
  → same-session sliding window sequence

R3 infeasible
  → NO_REFERENCE

R3 resource limit
  → SEARCH_INCONCLUSIVE

R3 invalid input
  → INVALID_INPUT
```

builder·validator·window 입력에는 expectation category, oracle label, Actor, observation, controller와
hidden 정보를 넣지 않았다. R3의 `UNSPECIFIED` feasible proxy는 R4 v1 LEFT/RIGHT reference로
승격하지 않고 `NO_REFERENCE`와 limitation으로 보존했다.

## 2. 최종 판정

| 항목 | 결과 |
|---|---:|
| public case 완료 | `21/21` |
| `REFERENCE_SET_READY` | `8` |
| `NO_REFERENCE` | `11` |
| `SEARCH_INCONCLUSIVE` | `1` |
| `INVALID_INPUT` | `1` |
| hard failure | `0` |
| relation failure | `0` |
| serial/process semantic parity | `PASS` |
| repeat determinism | `PASS` |
| window update | `242` |
| PNG | `21` |
| partial state 잔존 | `0` |
| clean-source qualification receipt | 생성 |

reference가 생성된 8개 case는 다음과 같다.

```text
wide-straight-left / right
wide-mirror-left / right
vertical-left / right
crossing-static-left / right
```

대표 `wide-straight-left`는 독립 reference validation을 통과했고, 23개 knot를 같은
`reference_session_id`로 순회하면서 `subgoal_revision 0→4`와 terminal window까지 통과했다.
mirror·vertical 관계에서 section 종류, knot 수, translation arc와 minimum clearance 불일치는
발생하지 않았다.

## 3. 증거 결박

```text
manifest:
28ae600ff6a340454ffa0ae9ec7b421a0c8ed935938f9f2dd4ae0af559729202

source freeze:
26dea2032a9b258680130d384aa0305874776c3bc99f0d5d5f0aed2fe6fccc80

catalog:
f95e8fd81b9de1a4cd312d739216b87aaceca1beca7e68613413619acf37e907

audit semantic:
0f7452784da87d6f308477ad7261dd4f0f674e64e031ecf192e72ce4211246ad

audit report:
9abd603ecc5708010ac8ce71b264b581185fd0aaf9810437bbfa6fb2ca143274

receipt:
45934d93ce1b02db12ee5c5ba573b450813c0b46e604327245be778d9d51bc86
```

생성 산출물은 Git에 커밋하지 않고 다음 로컬 경로에 보존한다.

```text
simulation/path_planning_lab/outputs/
  local-reference-public-20260814T060604Z-f43fbbf/
```

## 4. 회귀 검증

- R4-5 전용: `9 passed`
- R3/R4 직접 영향권: `103 passed`
- 전체 회귀: `794 passed`
- 전체 회귀 분할: `63`개 test file, 최대 `12` process
- failed file·non-empty stderr: `0 / 0`
- 전체 회귀 완료 시간: `367.583s`
- 전체 회귀 output:
  `simulation/path_planning_lab/outputs/test-runs/r4-final-v2-f43fbbf/`

첫 전체 회귀 오케스트레이션은 Windows `Start-Process`가 한글이 포함된 절대 test 경로를
손상시켜 63개 파일 모두 수집 전에 종료했다. 테스트 실행 수는 `0`이었으며 해당 output은
`r4-final-f43fbbf`에 보존하지만 최종 근거로 사용하지 않는다. 같은 명령을 반복하지 않고
저장소 상대경로를 전달하는 별도 `r4-final-v2-f43fbbf` 실행으로 전체 회귀를 완주했다. 시험
파일·시험 수·알고리즘·안전 기준과 판정은 변경하지 않았다.

## 5. 결론과 한계

이번 결과로 말할 수 있는 것은 다음뿐이다.

> 동결된 공개 정적 grid와 가상 차체 조건에서 검증된 R3 LEFT/RIGHT 공간 경로를 R4 immutable
> full reference와 revision-bound sliding window로 변환하면서 pose·heading·rotation·rejoin,
> source taxonomy와 session identity를 보존했다.

다음은 증명하지 않았다.

- Actor가 있는 시간 경로의 online 안전성
- observation/prediction·권한·shared safety gate 통합
- persistent controller의 reference 추종 성공
- R2 temporal evidence와 R4 reference의 결합
- 제품 알고리즘 채택, G1~G5 또는 경로 분석 7단계 결정
- 실제 센서·실차·사람 탑승 안전성 또는 의료기기 인증

따라서 R4 결과는 R5 persistent controller 입력 계약의 연구 후보로만 전달한다. reference 또는
window가 존재한다는 사실을 이동 허가, 자동 재출발이나 제품 기능 채택으로 해석하지 않는다.
