# R7 최종 자격 작업 동기화 인수인계

> 갱신일: 2026-08-19 (Asia/Seoul)
> 작업 브랜치: `codex/r7-final-qualification-20260819`
> hidden 실행 commit: `1eb5011e84caffa346abd40aeda711fadfe169f7`
> hidden 실행 tree: `d6edeed84652f53aabe1432bd7683ff5bb0c8e31`

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
hospital-autonomous-wheelchair의 R7 FAIL_ANALYZED 후속 작업을 이어간다.

git pull이 끝났다는 전제에서 AGENTS.md, 인수인계.md 최상단과 다음 두 문서만 먼저 읽어라.
1. docs/research/dynamic-actor-experiment/42-r7-hidden-v4-fail-analyzed-result-2026-08-19.md
2. docs/reviews/r7-final-qualification-sync-handoff-2026-08-19.md

git status -sb, git rev-parse HEAD, git rev-parse origin/codex/r7-final-qualification-20260819로
동기화와 clean 상태를 간단히 확인한다. 기존 내용을 장황하게 재보고하지 마라.

hidden-v4는 이미 한 번 소비됐고 공식 결과는 FAIL_ANALYZED다. 같은 seed를 다시 실행하거나
결과를 PASS로 바꾸지 마라. 새 hidden, 제품 알고리즘, G1~G5, 제품 경로분석 7단계도 시작하지
마라.

바로 공개 회귀에서 one-shot resume authorization을 소비 전에 trace evidence로 남기고 evaluator가
그 evidence를 검증하도록 고쳐라. 무단 runtime 생성, stale authorization, epoch mismatch 거부는
유지한다. 표적시험 뒤 공개시험 → 전체 회귀 → native parity·500회 자격 순서로 검증하되,
새 hidden 실행은 별도 사용자 승인 전 금지한다.
```
