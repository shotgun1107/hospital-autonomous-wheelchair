# R4 v2 Signed Reference 공개 qualification 결과

- 실행일: `2026-08-14`
- 판정: **PASS — 동결된 offline static reference 연구 범위**
- 평가 commit: `33a357dceba1c08d6c16db076878d7241d12fdcc`
- 평가 tree: `d7bf278dd5dcd6af19eacfa62ff7725b63b459f7`
- hidden 사용: 없음
- R5 v2 실행: 하지 않음
- 제품 알고리즘·G1~G5·제품 경로분석 7단계: 결정하지 않음

## 1. 이번 변경

R4 v1 결과와 receipt는 그대로 보존하고 새 v2 계약으로 다음만 추가했다.

- 각 section의 `travel_direction=FORWARD|REVERSE|NONE`
- 한 section 안의 signed direction 혼합 금지와 방향 변경 시 section 분리
- forward↔reverse 경계의 stopped entry/exit와 `STOP_MARKER`
- R3 `ANCHOR_CONNECTOR`를 방향 추론 없이 `NONE`인 비실행 연결 구간으로 보존
- 변위가 있는 abstract connector의 양 끝 정지 표식
- builder와 독립적인 source direction·전환 정지 검증
- schema·contract·set·window·builder·validator·reporting v2와 `path_revision=2`

R3의 `FORWARD_ONE_TRANSLATION`과 `REVERSE_ONE_TRANSLATION`만 signed translation으로
해석한다. 시작·종료 격자를 연결하는 `ANCHOR_CONNECTOR`의 작은 변위를 전진 또는 후진 명령으로
추측하지 않는다.

## 2. 공개 결과

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
| window update | `250` |
| PNG | `21` |
| clean-source receipt | 생성 |

ready 8개는 다음과 같다.

```text
wide-straight-left / right
wide-mirror-left / right
vertical-left / right
crossing-static-left / right
```

대표 `wide-straight-left`는 24개 knot, subgoal revision `0→5`, terminal window까지 같은
reference session으로 통과했다. section 방향은 다음과 같다.

```text
DEPART:NONE
→ DEPART:FORWARD
→ ROTATE:NONE
→ DEPART:FORWARD
→ BYPASS:NONE
→ RETURN:REVERSE
→ ROTATE:NONE
→ REJOIN:NONE
```

crossing-static 2건의 시작·종료 connector도 `NONE`으로 보존되며, signed translation 구간과
구분된 stopped abstract connector로 검증됐다.

## 3. 증거 결박

```text
manifest:
9830df6be77b51feea39e8761b844f6719f460e6b9082d7f4bf06651ae912fe6

source freeze:
ffb500e0b3fc43fc4a5527442a04c45de5baee4e4da127e21b411ed7489aeb9a

catalog:
6f57b29396247a009c78ab5948d345521aad15df4ec1d80aaf65541cff1ee881

audit semantic:
cdd8910954c29a5468dc3a3248146e3e88d12dfd5f68216b433fc013a07bd304

audit report:
1e1bcb00943852ab0fabe1932f8288a0193d868b9fb36810e89e45f17abfc8fc

receipt:
c496b5c2e9a813effca109b54aa20004d35c12414e90a2214d51fd1a0d4a758c
```

로컬 산출물은 Git에 커밋하지 않고 다음 경로에 보존한다.

```text
simulation/path_planning_lab/outputs/
  r4v2-local-reference-public-20260814-33a357d/
```

해당 폴더는 `152`개 파일, `64,567,309` bytes다.

## 4. 시험과 개발 중 발견한 문제

- R4 contracts·builder·validator·window·public 집중 시험: `69 passed`
- R5 v1 영향 확인: `13 passed, 1 xfailed`
- Ruff·compileall·`git diff --check`: 통과

첫 draft에서는 crossing-static의 `ANCHOR_CONNECTOR`를 signed direction으로 추론해 2건을
잘못 거부했다. 이를 비실행 `NONE` connector로 수정했다. 두 번째 draft에서는 mirror 관계가
section별 방향 순서까지 같아야 한다고 과도하게 가정해 relation 2건을 잘못 실패시켰다. source
primitive 독립 검증은 유지하고 mirror relation은 section kind와 방향 구성 보존을 검사하도록
수정했다. 세 번째 draft와 clean 실행은 모두 `21/21 PASS`였다.

기존 R5 v1 RPP는 R4 v2 signed reference 실행 자격이 없다. 대표 입력은 terminal까지 도달했지만
최대 tracking error가 `0.100510m`로 기존 `0.100000m` 기준을 넘었다. 기준을 완화하지 않았고,
R5 v2 구현 전까지 이 시험을 명시적 `xfail` 진단으로 보존한다.

## 5. 결론과 다음 단계

이번 결과로 말할 수 있는 것은 다음뿐이다.

> 동결된 공개 정적 grid와 가상 차체 조건에서 R3 primitive의 signed 이동 방향과 추상 connector를
> 구분해 immutable R4 v2 reference와 sliding window로 변환하고 독립 검증할 수 있다.

다음 구현 후보는 R5 v2다.

1. common executor가 forward↔reverse 전환 전 실제 정지 3 tick 확인
2. RPP와 DWB가 active `REVERSE` section에서만 최대 `0.10m/s` 후진 후보 사용
3. reverse rollout·terminal stopping을 동일 shared gate로 검사
4. signed static tracking 공개 재qualification

이번 결과는 controller 추종 성공, 동적 Actor 안전, 이동 허가, 실제 모터·후방 센서, 사람 탑승,
제품 알고리즘 채택이나 인증을 증명하지 않는다. R5 v2는 별도 구현·시험으로 시작해야 한다.
