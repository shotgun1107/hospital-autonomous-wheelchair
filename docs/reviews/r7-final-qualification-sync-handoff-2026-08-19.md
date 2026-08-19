# R7 최종 자격 작업 동기화 인수인계

> 갱신일: 2026-08-19 (Asia/Seoul)
> 작업 브랜치: `codex/r7-final-qualification-20260819`
> hidden 실행 commit: `1eb5011e84caffa346abd40aeda711fadfe169f7`
> hidden 실행 tree: `d6edeed84652f53aabe1432bd7683ff5bb0c8e31`

## 0. 2026-08-19 교정 작업 최신 상태

- hidden-v4 `FAIL_ANALYZED` 증거와 root seed는 그대로 보존했다.
- one-shot 재출발 권한을 실제 gate 입력에서 기록하도록 trace/evaluator를 교정했다.
- 정지 확인 이전 발행, mission·epoch·revision·hash 불일치, temporal session/reference/tick/phase/hash
  불일치와 같은 권한 hash 재사용을 모두 fail-closed로 거부한다.
- 저장 JSONL 재읽기 회귀를 포함한 표적시험 `52 passed`, v5 lifecycle·포장시험
  `21 passed`, 공개 안전 묶음 `131 passed`다.
- 전체 회귀는 4개 독립 process에서 `206 + 255 + 283 + 296 = 1,040 passed`이며 실패·건너뜀은
  없다. Ruff·compileall·`git diff --check`도 통과했다.
- 교정 hidden은 `r7-hidden-observation-v5`, `hidden-v5-*`로 분리했고, GitHub의 고정 원격
  reservation ref로 집·회사 clone 사이 두 번째 v5 실행을 막는다.
- 교정 변경은 현재 작업 브랜치에 commit·push해 회사 PC에서 이어받는다. clean committed
  source가 필수인 native parity·직렬 500회·v5 hidden은 아직 실행하지 않았다.

### 정확한 재개점

1. 회사 PC에서 해당 branch를 fast-forward pull하고 `HEAD == origin/<branch>`, clean tree를 확인한다.
2. current HEAD에서 `run_r7_native_release_gate.py`를 rebuild 포함으로 실행한다.
3. parity·계약시험·직렬 500회에서 50ms 초과 0이면 v5 preflight remote reservation을 만든다.
4. 같은 지정 실행 PC에서 reservation을 claim한 뒤 v5 hidden을 정확히 한 번 실행한다.
5. PASS/FAIL/BLOCKED 결과를 재실행 없이 문서·receipt·증거 ZIP으로 동결한다.

교정 명세는
[43-r7-hidden-v5-corrective-qualification-2026-08-19.md](../research/dynamic-actor-experiment/43-r7-hidden-v5-corrective-qualification-2026-08-19.md)를
따른다.

## 1. 현재 결론

- R7 포장 차단점 3개를 고쳤고 `41 passed`를 확인했다.
- R7 공개 회귀 `65 passed`, 전체 회귀 `1,035 passed`다.
- native parity `5 case`, 계약시험 `13`, 직렬 `500회`의 50ms 초과 `0`으로 통과했다.
- hidden-v4 20개를 정확히 한 번 실행했다.
- Normal 10개는 모두 완료했고 Stress는 8개 무출발 정지, 2개 출발 뒤 안전 재정지다.
- 실제 충돌·금지구역·`0.08m` 거리 위반·stale 추진·hard failure는 0이다.
- 공식 결과는 `PASS_FINAL`이 아니라 `FAIL_ANALYZED`다.

실패 원인은 재출발 승인 객체를 pipeline이 사용하고 지운 다음 trace를 만들지만 evaluator가
지워진 사후 `resume_authorization_revision`을 요구한 것이다. release 39건의 다른 조건은 모두
통과했고 이 필드만 39/39 누락됐다. 같은 누락을 release 계약 위반과 unauthorized restart로
각각 세어 두 수치가 모두 39가 됐다.

상세 근거는
[42-r7-hidden-v4-fail-analyzed-result-2026-08-19.md](../research/dynamic-actor-experiment/42-r7-hidden-v4-fail-analyzed-result-2026-08-19.md)를
정본으로 사용한다.

## 2. 이번 작업에서 완료한 변경

commit `1eb5011e84caffa346abd40aeda711fadfe169f7`:

- R6 receipt를 tracked immutable evidence로 연결
- 과거 hidden evidence를 과거 source에서만 검증하고 현재 source에서는 mismatch 거부
- 20개 모든 case의 JSONL trace와 SHA·record count·마지막 record hash 결박
- commitment 이후 준비·실행·포장 오류의 `BLOCKED_INFRASTRUCTURE` receipt/ledger 결박

숨은 실행 뒤에는 코드를 수정하거나 시험을 다시 실행하지 않았다. 결과 분석과 문서·증거 포장만
수행했다.

## 3. 보존 증거

```text
simulation/path_planning_lab/outputs/r7-hidden-v4-fail-analyzed-evidence-20260819-1eb5011.zip
size: 9,835,156 bytes
SHA-256: 0df82eb0f5eb184c1f2e65190361d917a4b8ba9c2fb04ed675a4be15eab538c5
entries: 318
```

포함 범위는 hidden 20개 전체 trace·case 결과·summary·receipt, one-use ledger, native release
evidence, 전체 회귀 shard 로그다. ZIP 내부 manifest 317개는 payload와 exact-set·SHA-256가
일치한다.

주요 hash:

```text
seed commitment: 24c8345635d953356e5b3a980f1803200aa9d87471f7419285e47afcf551e2cc
result set:      381fe8f2ccdb6b63b139864ecc4110c26df697cc627501c9895bc47f5d83ae19
case trace set:  49e61b598e556570580317226e0aef73f10e44167773e55967353e25290bd23c
trace manifest:  8a85d91f86a6dd03d96d4c3f09b2bfc89152207a3f0dd2bb902a84548d027302
receipt:         5596c93d3a279f2a5ef4b5832e99276c818890a07eedade11b8aa6d8acbca348
```

## 4. 다음 작업의 정확한 시작점

1. `test_r7_hidden_v4_qualification.py`에 실제 one-shot authorization 소비 순서를 재현하는 공개
   회귀를 추가한다.
2. `r5c_observation_diagnostic.py`에서 authorization revision·hash·epoch를 소비 전에 별도 trace
   evidence로 저장한다.
3. `r7_hidden_v4_qualification.py`가 사후 runtime 객체 대신 그 불변 evidence를 검증하게 한다.
4. 무단 runtime 생성, stale authorization과 epoch mismatch mutation은 계속 거부한다.
5. 수정 뒤 공개시험 → 전체 회귀 → native parity·500회 자격 순서로 다시 검증한다.

새 hidden namespace/seed 설계와 실행은 위 공개 수정이 끝나도 자동으로 시작하지 않는다. 별도
사용자 승인이 필요하다.

## 5. 금지선

- 소비된 hidden-v4 seed 재실행 금지
- hidden 결과를 사후 `PASS_FINAL`로 변경 금지
- `0.08m`, goal `0.05m`, distinct safe frame `11`, stale/dropout, stop epoch/session 기준 완화 금지
- 과거 hidden-v1/v2/v3/v4 FAIL과 evidence 삭제·덮어쓰기 금지
- 제품 알고리즘, `G1~G5`, 제품 경로분석 7단계 자동 시작 금지
- 실제 카메라·사람·환자·제품 안전 주장 금지

## 6. 환경·동기화

Python 정본은 `simulation/path_planning_lab/pyproject.toml`이다. `.venv`, DLL, pytest temp는
Git에 넣지 않는다. Windows 공용 temp가 막히면 프로젝트의 ignored `outputs/test-runs` 아래에
고유 `--basetemp`를 사용한다.

다른 PC에서는 기존 변경을 삭제하지 말고 다음만 확인한다.

```powershell
git status -sb
git fetch origin --prune
git switch codex/r7-final-qualification-20260819
git pull --ff-only origin codex/r7-final-qualification-20260819
git rev-parse HEAD
git rev-parse origin/codex/r7-final-qualification-20260819
git status --porcelain=v1
```

## 7. 새 세션 시작 프롬프트

```text
hospital-autonomous-wheelchair의 R7 hidden-v5 교정 자격을 이어서 끝낸다.

먼저 AGENTS.md, 인수인계.md 최상단,
docs/reviews/r7-final-qualification-sync-handoff-2026-08-19.md의 0절,
docs/research/dynamic-actor-experiment/43-r7-hidden-v5-corrective-qualification-2026-08-19.md만 읽어라.

현재 집 로컬에는 전체 회귀 1,040 passed까지 검증된 미커밋 변경이 있다. 이를 pull/reset/restore로
덮어쓰지 말고 git status와 git diff --check로 확인한 뒤 한 commit으로 보존하고 현재 branch를
origin에 push하라. commit 전후에 소스가 바뀌지 않았다면 전체 회귀를 다시 반복하지 않는다.

clean committed HEAD에서 native release gate를 rebuild 포함으로 실행한다. parity·계약시험과 직렬
500회에서 50ms 초과가 0이면, run_r7_hidden_v5.py의 고정 원격 reservation ref를 지정 실행 PC에서
reserve하고 claim한 뒤 v5 hidden을 정확히 한 번 실행한다. 이 작업은 이미 사용자 승인됐으므로
중간 재승인을 요구하지 않는다. 다른 clone이 reservation을 선점했거나 native gate가 실패하면
우회하지 말고 BLOCKED로 닫는다.

hidden-v4와 그 seed·FAIL_ANALYZED 증거는 수정하거나 재실행하지 않는다. v5 결과는 PASS/FAIL/BLOCKED
중 실제 결과 그대로 receipt·증거 ZIP·상태 문서에 동결하고 재실행하지 않는다. 제품 알고리즘,
G1~G5와 제품 경로분석 7단계는 시작하지 않는다.
```
