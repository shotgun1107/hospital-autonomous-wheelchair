# R7 최종 자격 작업 동기화 인수인계

> 작성일: 2026-08-19 (Asia/Seoul)
> 저장소: `https://github.com/shotgun1107/hospital-autonomous-wheelchair.git`
> 작업 브랜치: `codex/r7-final-qualification-20260819`
> 기준 commit: `54b4f04f06a22f6eebc05228a4f80abdfdd42615`
> 동기화 WIP checkpoint: `e6723e614d565594e10664af031ec8d958fa1c2f`
> 이전 보존 checkpoint: `ee08150` (`wip: preserve r7 final logic and packaging`)

이 문서는 회사 PC의 미완료 R7 최종 자격 작업을 다른 로컬 PC에서 그대로 이어가기 위한
인수인계다. 이 문서는 `PASS_FINAL` 결과가 아니며, 새 hidden을 실행하거나 새 seed를 만든
기록도 아니다.

## 1. 지금까지 확인된 사실

이미 보존된 경로·안전 로직은 다시 설계하지 않는다.

- forbidden 구역을 포함한 실제 clearance가 `0.08m` 미만이면 거부하고, 정확히 `0.08m`는
  허용하는 P0 수정이 있다.
- 목표 앞 `0.05m` 초과·`0.10m` 이하에서만 허용되는 terminal forward tie P1 수정이 있다.
- Stress라는 이름만으로 출발을 막지 않는다. fresh safe frame 11개를 gate가 확인한 뒤에만
  조건부 출발하고, dropout/stale 뒤에는 감속·실제 정지·새 epoch·새 허가가 필요하다.
- historical hidden-v1/v2/v3와 그 FAIL 기록은 보존한다. 이미 알려진 공개 seed는 새 hidden에
  재사용하지 않는다.

이번 WIP는 위 경로 로직을 바꾸지 않았다. 남은 것은 clean checkout에서 재현 가능한
qualification/hidden 포장·검증 연결이다.

## 2. 이번 동기화에 포함한 WIP

### 새로 보강한 부분

- native qualification source freeze를 현재 Python 실행 경로와 R7 실행 스크립트까지 넓혔다.
- native release evidence ZIP에 contract parity 증거와 deterministic ZIP 생성 규칙을 추가했다.
- 새 hidden-v4 runner가 release evidence의 HEAD/tree, source hash, native DLL hash, timing
  `500회·0 miss`를 확인하도록 보강했다.
- hidden 실행 전 preflight receipt, one-use 소비 ledger, 새 seed commitment 순서를 추가했다.
- hidden 첫 실행부터 case별 JSONL trace를 남기고, release 11-frame·중복 frame·stale propulsion·
  unauthorized restart·collision·forbidden·clearance 위반 수를 결과/receipt에 묶기 시작했다.
- Python/native forbidden 경계 시험에 `0.10m` 안전 사례를 추가했다.
- 새 packaging 단위시험을 추가했다.

### 이번 동기화 전 직접 확인한 검증

```text
Ruff                                               PASS
compileall                                         PASS
git diff --check                                   PASS
R7 native qualification + hidden-v4 + packaging
  + C++ safety 직접 영향권                         35 passed, 21.05s
```

첫 pytest 호출은 Windows 공용 임시폴더 권한 때문에 7개 setup error가 났다. 같은 시험을
`simulation/path_planning_lab/outputs/test-runs/` 아래의 프로젝트 전용 `--basetemp`로
다시 실행해 `35 passed`를 확인했다. 이는 코드 실패가 아니라 PC 임시폴더 권한 문제다.

## 3. 아직 하지 않은 것

다음은 **미실행**이며 PASS라고 부르면 안 된다.

- R7 관련 기존 public regression 전체
- clean 환경 전체 pytest 1회
- clean native build
- Python↔native parity와 계약 parity의 최종 증거 생성
- 직렬 `5 case × 100 = 500` timing qualification
- 새 release evidence ZIP 생성
- 새 hidden-v4 preflight-only
- 새 root seed commitment와 hidden-v4 실제 1회 실행
- `PASS_FINAL` 또는 `FAIL_ANALYZED` 결과 문서

## 4. 다음 PC에서 먼저 해결할 일

1. R6 선행 receipt가 Git에서 재현 가능한지 정리한다.
   - 기존 runner 기본 경로는 clean checkout에 없는 ignored output을 가리킬 수 있다.
   - 현재 유효한 recovered receipt를 immutable tracked artifact로 둘지, 명시 CLI 입력으로
     받을지를 결정·시험해야 한다.
2. historical hidden runner 시험을 바로잡는다.
   - 현 source가 바뀌었으므로 과거 evidence를 현 source에서 허용하면 안 된다.
   - 과거 commit에서는 검증 가능하고, 현 HEAD에서는 source mismatch로 거부되는지를 분리해
     시험해야 한다.
3. 새 runner의 실패 처리 범위를 마지막까지 점검한다.
   - commitment 이후 어떤 infrastructure 오류가 나도 `BLOCKED_INFRASTRUCTURE` 기록이 남아야
     한다.
   - JSONL trace SHA·record count·마지막 record hash가 case result → summary → receipt까지
     실제로 결박되는지 시험한다.
4. 위를 끝내고 코드 동결 전 읽기 전용 감사를 한다.
5. 그 다음에만 공개 관련 시험 → 전체 회귀 1회 → native qualification → preflight-only →
   hidden-v4 1회 순서로 진행한다.

hidden 실패 뒤에는 코드를 바로 고치지 않는다. 이미 만든 trace로 먼저 root-cause report를
작성하고 `FAIL_ANALYZED`로 끝낸다.

## 5. 금지선

- `0.08m`, goal `0.05m`, distinct safe frame `11`, stale/dropout, stop epoch, session,
  authorization 조건을 완화하지 않는다.
- hidden-v1/v2/v3, historical FAIL, 기존 evidence를 삭제하거나 PASS로 바꾸지 않는다.
- 새 hidden을 두 번 실행하거나 seed를 다시 뽑지 않는다.
- main에 직접 병합하거나 force push, reset --hard, clean, rebase, 무단 stash를 하지 않는다.
- 사용자 제공 ZIP은 stage/commit/delete하지 않는다.

## 6. 주요 경로

```text
simulation/path_planning_lab/scripts/run_r7_native_release_gate.py
simulation/path_planning_lab/scripts/run_r7_hidden_v4.py
simulation/path_planning_lab/src/hospital_path_lab/r7_native_qualification.py
simulation/path_planning_lab/src/hospital_path_lab/r7_hidden_v4_qualification.py
simulation/path_planning_lab/tests/test_r7_native_qualification.py
simulation/path_planning_lab/tests/test_r7_hidden_v4_qualification.py
simulation/path_planning_lab/tests/test_r7_final_packaging.py
simulation/path_planning_lab/tests/test_r7_hidden_runner.py
docs/research/dynamic-actor-experiment/26-r7-native-release-gate.md
docs/research/dynamic-actor-experiment/29-r7-failure-trace-and-public-regression-spec.md
docs/research/dynamic-actor-experiment/40-r7-stress-conditional-release-policy-2026-08-19.md
docs/research/dynamic-actor-experiment/41-r7-hidden-v4-conditional-evaluator-2026-08-19.md
```

## 7. 환경 복원

확인한 Python은 `3.12`다. 다른 PC에서 먼저:

```powershell
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -e ".\simulation\path_planning_lab[dev]"
```

native build가 필요할 때만:

```powershell
.\.venv\Scripts\python.exe -m pip install ziglang
.\.venv\Scripts\python.exe .\simulation\path_planning_lab\scripts\build_cpp_dwb_safety_core.py
.\.venv\Scripts\python.exe .\simulation\path_planning_lab\scripts\build_cpp_dwb_full_core.py
```

Windows 공용 temp 접근이 막히면 pytest에 프로젝트 내부 basetemp를 준다.

```powershell
$testRun = ".\simulation\path_planning_lab\outputs\test-runs\handoff-$([guid]::NewGuid())"
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider `
  --basetemp=$testRun `
  -c .\simulation\path_planning_lab\pyproject.toml `
  .\simulation\path_planning_lab\tests\test_r7_native_qualification.py `
  .\simulation\path_planning_lab\tests\test_r7_hidden_v4_qualification.py `
  .\simulation\path_planning_lab\tests\test_r7_final_packaging.py
```

Docker, VM, DB, 외부 서비스, 비밀 환경변수는 이 R7 연구 실험에 없다. `.venv`, native DLL,
pytest temp/output은 PC별 산출물이라 Git에 넣지 않는다.

## 8. 동기화 방법

집 PC에 먼저 미보존 변경이 있으면 민감 파일을 제외한 안전한 파일만 local backup branch에
commit하고 원격에는 push하지 않는다. 그 뒤 작업 브랜치를 fast-forward한다.

```powershell
git status -sb
git status --short
git remote -v
git branch -vv
git stash list
git log --oneline --branches --not --remotes

git fetch origin --prune
git switch codex/r7-final-qualification-20260819
git pull --ff-only origin codex/r7-final-qualification-20260819
git rev-parse HEAD
git rev-parse origin/codex/r7-final-qualification-20260819
git merge-base --is-ancestor e6723e614d565594e10664af031ec8d958fa1c2f HEAD
git status --porcelain=v1
```

fast-forward가 실패하거나 local branch와 remote branch가 갈라지면 reset/rebase/merge로
임의 해결하지 말고 local/remote HEAD와 차이 파일을 보고한다.

## 9. 새 세션 시작 프롬프트

```text
hospital-autonomous-wheelchair의 R7 최종 자격 작업을 다른 로컬 PC에서 이어간다.

저장소: https://github.com/shotgun1107/hospital-autonomous-wheelchair.git
작업 브랜치: codex/r7-final-qualification-20260819
기준 baseline: 54b4f04f06a22f6eebc05228a4f80abdfdd42615
필수 WIP ancestor: e6723e614d565594e10664af031ec8d958fa1c2f

먼저 AGENTS.md와 다음 문서만 완전히 읽어라.
1. 인수인계.md 최상단의 최신 R7 상태
2. docs/reviews/r7-final-qualification-sync-handoff-2026-08-19.md
3. docs/research/dynamic-actor-experiment/26-r7-native-release-gate.md
4. docs/research/dynamic-actor-experiment/29-r7-failure-trace-and-public-regression-spec.md
5. docs/research/dynamic-actor-experiment/40-r7-stress-conditional-release-policy-2026-08-19.md
6. docs/research/dynamic-actor-experiment/41-r7-hidden-v4-conditional-evaluator-2026-08-19.md

현재 로컬 상태를 읽기 전용으로 확인한다.
git status -sb
git status --short
git remote -v
git branch -vv
git stash list
git log --oneline --branches --not --remotes

미보존 변경은 삭제·덮어쓰기 하지 말고, 민감 파일이 아닌 경우에만 local backup branch와
backup commit으로 보존한다. backup branch는 push하지 않는다.
reset --hard, clean, rebase, force push, 무단 stash는 금지한다.

그 다음:
git fetch origin --prune
git switch codex/r7-final-qualification-20260819
git pull --ff-only origin codex/r7-final-qualification-20260819
git rev-parse HEAD
git rev-parse origin/codex/r7-final-qualification-20260819
git merge-base --is-ancestor e6723e614d565594e10664af031ec8d958fa1c2f HEAD
git status --porcelain=v1

branch/head가 일치하고 e6723e6이 ancestor이며 작업트리가 clean인지 확인한다.
갈라짐 또는 fast-forward 실패 시 임의로 해결하지 말고 보고한다.

현재 상태는 R7 final packaging/runner WIP다. hidden, 새 seed, 전체 회귀, 500회 timing,
native release evidence는 아직 실행하지 않았다. 먼저 R6 tracked prerequisite, historical
runner test, failure-trace/receipt binding을 고치고 표적시험과 읽기 전용 감사를 통과시켜라.
그 뒤에만 전체 회귀 1회 → native qualification → preflight-only → hidden-v4 1회 순서로
진행한다. hidden을 두 번 실행하거나 실패 뒤 즉시 튜닝하지 마라.
```
