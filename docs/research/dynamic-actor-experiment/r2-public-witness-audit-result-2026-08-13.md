# R2 공개 Witness 감사 결과 — 2026-08-13

## 1. 판정

- 실행 상태: `COMPLETE`
- R2 완료 자격: `FAIL`
- hard failure: `2`
- 공개 episode: `19/19` 실행 완료
  - v6 공개: `13/13`
  - legacy mechanism golden: `6/6`
- hidden: 생성·열람·실행하지 않음
- 제품 알고리즘, `G1~G5`, 제품 경로분석 7단계: 결정하지 않음

후속 범위 보정: 이 문서의 `R2 자격 FAIL`은 당시 결합돼 있던 ground-truth path와
observation/prediction 전체 자격에 대한 역사적 판정으로 유지한다. 이후 사용자 결정과
[`ADR 0011`](../../decisions/0011-separate-path-and-perception-research-gates.md)에 따라
`R2-A ground-truth path`와 `R2-B observation/prediction`을 분리했다. hard failure 2건은
R2-B에 귀속하며, R2-A 미해결 공간 분류를 위한 R3 명세 진입은 허용한다.

이번 결과는 검색이나 보고 runner가 중단된 partial 결과가 아니다. 전체 19개 episode의
검색·독립 검증·Ideal/Normal/Stress replay·JSON/Markdown/PNG 생성과 completion receipt
봉인까지 끝났다. 그러나 Ideal profile hard failure가 2건이므로 `R2 완료`로 승격하지 않는다.

## 2. 실행 정본

```text
source commit: 4e4ba0fb91d67498fe163aca99ff1ab647224f08
source tree:   b5df045fcf7f03269cf3f6056c93bb9a2b091432
workers:       14 process
PASS shard:    2,048 candidates
manifest hash: bd25b0348fd729c3df7aba49bc3c9c5ef50dfcbf6386b7197a0aab6de94f495f
semantic hash: 13448a876803d2d0631a549a4e6c9c451c76fc7571a6c0e0be820ef7a14ee9d0
report hash:   2aec4da6d96967ef3ca24b910c01b8ca31aeae18a52dc3162202aa7f0e0ab431
```

완료 산출물은 다음 비덮어쓰기 경로에 보존했다.

```text
simulation/path_planning_lab/outputs/
  witness-audit-public-20260813-r2-v2-4e4ba0f/
```

이 경로의 정본 파일은 `witness_search_manifest.json`, `witness_search_results.json`,
`summary.md`, `run_state.complete.json`, `witness_audit_completion.json`과 episode별
하위 산출물이다.

첫 실행은 도구 timeout으로 약 10초 뒤 중단됐다. 해당 경로는 삭제하거나 덮어쓰지 않았고
`run_state.incomplete.json`과 manifest만 가진 infrastructure evidence로 보존했다.

```text
simulation/path_planning_lab/outputs/
  witness-audit-public-20260813-r2-v1-4e4ba0f/
```

`v1`에는 `witness_search_manifest.json`과 `run_state.incomplete.json`만 남아 있다.
`INFRASTRUCTURE_INCOMPLETE`이며 최종 실험 근거로 사용하지 않는다.

전체 완료 wall-clock은 약 `1,943.59초`다. 이는 Python 운영 진단일 뿐 witness·taxonomy·
안전·R2 자격 판정에는 사용하지 않는다.

구현 뒤 저장소 전체 Python 회귀는 `688 passed in 587.64s`, Ruff·compileall·diff 검사는
통과했다. pytest wall-clock도 기능·알고리즘 또는 native timing 자격이 아니라 회귀 운영
정보다.

## 3. 검색 결과

### PASS

```text
generated:        135,360
geometry reject:   26,382
dynamic reject:    70,318
validated:         38,660
```

### WAIT/HOLD

```text
generated:       389
geometry reject: 127
dynamic reject:   28
validated:       234
```

사전 기대와 결과의 episode-level 비교는 다음과 같다.

```text
matched:             17
mismatched:           1
not fully covered:    1
```

- legacy crossing `LOCAL_DETOUR_FEASIBLE` 1건은 R2-PASS v1의 같은 방향 Actor 범위 밖이라
  `SEARCH_INCONCLUSIVE`이다. DWA나 일반 지역우회 불가능 판정이 아니다.
- legacy dynamic-change 1건은 두 위험에 대한 재정지 순서를 완전히 증명하지 못해
  `not fully covered`다.

## 4. hard failure 2건

실패 episode는 다음 둘이다.

1. v6 `second-risk` 장면
   - 새 Actor 활성 시작: `T_sim=13.000s`
   - Ideal 관측은 `13.000~13.150s` 동안 fresh `EMPTY_FRAME`
   - 실제 Actor는 존재하지만 예측 Capsule은 없음
   - containment miss: `38`, 최대 miss `0.18m`
2. legacy `dynamic-change` 장면
   - 두 번째 Actor 활성 시작: `T_sim=4.051282...s`
   - Ideal 관측은 약 `4.055~4.250s` 동안 fresh `EMPTY_FRAME`
   - 실제 Actor는 존재하지만 예측 Capsule은 없음
   - containment miss: `43`, 최대 miss `0.18m`

두 witness의 exact ground-truth 충돌·clearance 검증이 실패한 것은 아니다. 실패 원인은
다음 계약 조합이다.

```text
episode 중간에 Actor가 즉시 생성됨
+ 관측 latency가 존재함
+ fresh EMPTY를 사용 가능한 관측으로 취급함
+ Ideal에서는 actual Actor가 prediction shape에 항상 포함돼야 함
= 생성 직후 관측 전 구간에서 필연적인 Ideal coverage miss
```

R1 기존 감사가 이 문제를 보고하지 않은 이유도 확인했다. R1 coverage 집계는 생성된 Capsule을
순회했기 때문에, 실제 Actor가 존재하지만 Capsule 자체가 없는 구간을 분모에 넣지 않았다.
R2의 stricter replay는 실제 Actor 기준으로 반대 방향도 검사해 이 공백을 드러냈다.

## 5. 결론과 다음 작업

현재 결론은 다음과 같다.

> 공개 19개 R2 감사 runner와 산출물 수명주기는 완성됐지만, 공개 corpus의 Actor 출현 모델과
> Ideal 관측·prediction 계약이 서로 충돌하므로 R2 자격은 실패했다.

결합 R2를 그대로 재실행한다면 새로운 공개 버전에서 다음을 먼저 명세해야 한다.

- Actor가 episode 중간에 어디에서 어떻게 진입하는가
- 지도 밖·가림·미관측 영역을 표현할 것인가
- Actor 활성 시작과 센서 가시성 시작을 같은 사건으로 볼 것인가
- fresh `EMPTY_FRAME`이 보장하는 범위와 의미
- 새 Actor 출현 직후 latency 구간에서 online 영역이 정지해야 하는가
- R1 audit가 actual Actor without prediction shape를 반드시 세는가

안전 여유, Actor radius, Capsule, latency나 hard criterion을 이번 결과에 맞춰 낮추지 않는다.
계약을 수정하면 기존 완료 output은 실패 회귀 자료로 유지하고, 새 corpus·manifest hash로 공개
감사를 다시 실행한다. 이 조건은 perception-integrated R5~R7과 hidden 진입을 막는다. 후속
gate 분리 뒤에는 observation을 입력으로 받지 않는 R3 static 공간 oracle까지 막지는 않는다.

## 6. 증거 한계

이 결과는 Python 합성 시뮬레이션의 공개 open-loop 원형 Actor에 한정한다. 실제 사람 운동,
실제 센서, 축소 실물, 제품 알고리즘, 실제 사람 탑승 안전성 또는 의료기기 인증의 증거가
아니다. Python wall-clock·CPU·memory·cache는 R2 의미 판정에서 제외하며, native 연산 자격은
semantic parity 뒤 R7에서 별도로 측정한다.
