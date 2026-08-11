# 6단계 — runner, hidden, 통계와 보고

## 구현 상태

- 상태: runner·CLI·hidden lifecycle·통계·산출물 구현 및 full hidden 실행 완료
- 공개 corpus hard-safety와 contract-fault 자격 실패 시 hidden 생성 전에 중단한다.
- manifest를 먼저 기록한 뒤 hidden 30개를 생성하고, 해시가 붙은 소비 영수증을
  무덮어쓰기로 남긴다.
- `evaluation_tick_limit`은 전용시험용 축소 실행에만 사용하며 이 값이 있으면 full frozen
  run 또는 DWA 승격으로 판정하지 않는다.
- 최종 full hidden 실행 전에는 코드·파라미터를 고정하며, 실행 뒤 변경 시 같은 commitment를
  재사용하지 않는다.
- 독립적인 `episode × observation profile`을 process worker로 병렬 실행한다. 같은 job
  안에서는 PP와 DWA를 같은 입력으로 순서대로 실행하고, 부모가 corpus·profile·controller
  순서로 결과를 재정렬한다.
- 기본 worker 수는 실제 process affinity의 논리 CPU 수를 기준으로 `min(6, logical/4)`로
  제한한다. 현재 회사 PC는 논리 CPU 28개이므로 기본값은 6이다.
- worker 내부 경과시간은 contention의 영향을 받으므로 `nonqualification` 진단값으로만
  기록한다. 50 ms wall-clock qualification은 모든 worker가 종료된 뒤 부모 프로세스에서
  단독 직렬 실행한다.

## 2026-08-11 full 실행 결과

- output: `simulation/path_planning_lab/outputs/dynamic-experiment-20260811-final-v4`
- worker: 14개, 28 logical CPU의 약 50%; 결과 계산만 병렬화
- 실행량: 공개 144 runs, hidden 120 runs
- hard-safety: 전체 `264/264`, hidden `120/120` 통과
- contract-fault: `25/25` 통과
- hidden Normal 기능 자격: PP `27/30`, DWA `16/30`
- hidden Stress 기능 자격: PP `5/30`, DWA `5/30`
- DWA feasible detour·rejoin: `0%`, 최대 기준경로 이탈 약 `0.00585 m`
- 직렬 timing: PP deadline miss `0/400`, DWA `324/400`
- 통계: DWA hold median 개선 `25.17%`, paired delta 95% CI
  `[-1.75 s, -1.60 s]`; 완료시간은 `6.31%` 악화
- 판정: 승격 조건 3·4·5·10 미달, DWA 승격 안 함, `PP + shared gate` 유지

결과를 보고 DWA·Actor tube·기준을 수정하지 않았다. 이후 DWA를 변경하면 이 hidden은
regression으로만 사용하고 새 commitment와 새 hidden으로 다시 평가한다.

## 목표

development 튜닝을 종료하고 code·parameter·corpus를 동결한 뒤 새 hidden paired corpus를
한 번 실행해 연구 기준선 승격 여부와 한계를 재현 가능한 산출물로 남긴다.

## 진입조건

- 5단계 golden·development·fault 시험이 통과한다.
- controller별 development 전체 평가 횟수가 3회 이하다.
- hard safety 또는 공통 fault 실패가 0이다.
- 동결할 모든 parameter가 코드 또는 manifest 입력으로 표현되어 있다.

## 수정·추가 대상

```text
src/hospital_path_lab/dynamic_runner.py
src/hospital_path_lab/experiment_runner.py
src/hospital_path_lab/cli.py
src/hospital_path_lab/corpus_records.py
tests/test_dynamic_runner.py
tests/test_dynamic_statistics.py
tests/test_dynamic_hidden_lifecycle.py
```

## 실행 명령 후보

```powershell
hospital-path-lab dynamic-experiment `
  --base-seed <public-seed> `
  --hidden-seed <hidden-seed> `
  --hidden-commitment <sha256-commitment> `
  --simulation-workers 6 `
  --output-dir <output-dir>
```

실제 옵션 이름은 CLI 구현 시 고정하며 `--help`와 README를 함께 갱신한다.

## 동결 manifest

```text
experiment_manifest.json
- code_commit_hash
- map_corpus_hash
- pp_parameter_hash
- dwa_parameter_hash
- safety_gate_parameter_hash
- observation_generator_hash
- scenario_generator_hash
- simulator_version
- hidden_seed_commitment
- qualification_snapshot_set_hash
- machine_identifier
- tuning_access_count_by_controller
- numeric_tolerance_version
```

qualification snapshot에는 Actor 0명, 1명, 2명, 최대 Actor tube와 static geometry가 함께
있는 development 최악 사례를 포함한다.

## hidden lifecycle

```text
public code·parameter·development freeze
→ public corpus hash 확정
→ hidden seed commitment 검증
→ hidden 30개 생성
→ PP·DWA paired Normal·Stress 실행
→ 결과 판정
```

hidden을 본 뒤 변경하면:

```text
hidden-v1 → regression 전환
코드·parameter 수정
새 commitment → hidden-v2
```

동일 hidden을 수정 후 최종 증거로 재사용하지 않는다. 실패 episode는 provenance와 최소
failing prefix를 무덮어쓰기 방식으로 보존한다.

## 통계 모집단

`S_progress`는 Normal hidden 중 사전에 progressable로 지정한 paired episode다.
`NO_SAFE_SOLUTION`은 완료시간 모집단에서 제외한다.

```text
time_improvement =
    1 - median(T_DWA) / median(T_PP)

hold_improvement =
    1 - median(Hold_DWA) / median(Hold_PP)
```

- time 15% 또는 hold 20% 개선
- PP median hold가 0이면 hold 개선은 undefined
- 같은 지표·episode 집합의 class-stratified paired bootstrap 10,000회
- paired delta 95% CI upper bound `<0`

승차감 악화율은 v5의 denominator floor를 사용하며 세 지표 각각 25% 이하여야 한다.

## 연구 기준선 승격 판정

runner는 다음을 개별 boolean과 근거 수치로 저장한다.

1. PP·DWA Normal·Stress hard safety
2. 공통 fault 자격
3. `S_progress` 기능 자격
4. Normal deadline miss 0, late applied 0
5. DWA feasible detour·rejoin 80% 이상
6. forbidden/no-safe overtaking 0
7. time 또는 hold 개선과 CI
8. comfort 악화 25% 이하
9. detour 30%, 최대 이탈 0.50 m 이하
10. gate override 5%, 연속 3 tick 이하

하나라도 false면 DWA를 승격하지 않는다. 결과를 단일 실행시간 순위로 축약하지 않고
안전·기능·효율·승차감·연산비용 Pareto를 함께 저장한다.

## wall-clock qualification

- episode 결과 계산용 process pool을 완전히 종료한 뒤 실행한다.
- qualification 자체는 병렬화하지 않는다.
- 고정 machine ID와 CPU affinity
- numeric thread 1
- snapshot별 warm-up 30회, 측정 100회
- p50, p95, p99, maximum, miss count, peak memory
- wall-clock 값은 결정론 비교 대상에서 제외
- 명령 sequence와 상태·사건·metric은 frozen tolerance 안에서 결정론 검증

## 산출물

```text
experiment_manifest.json
hidden_consumption_receipt.json
public_prequalification.json
qualification_results.json
hard_safety_results.json
contract_fault_results.json
paired_episode_results.json
paired_statistics.json
pareto_summary.json
promotion_decision.json
summary.md
visualizations/<episode>/<controller>.png
regression_candidates/<record>.json
```

`summary.md` 첫 문장은 반드시 다음 범위를 밝힌다.

> 이 결과는 open-loop 원형 Actor와 동결된 합성 관측을 사용하는 Python
> `simulation_only` 비교이며 제품 알고리즘 또는 실제 사람 탑승 안전성의 증거가 아니다.

## 시험

| 시험 ID | 내용 | 연결 계약 |
|---|---|---|
| `DYN-T-RUN-001` | manifest hash와 source freeze 검증 | `DYN-HID-001` |
| `DYN-T-RUN-002` | hidden 생성 전 commitment 검증 | `DYN-HID-001` |
| `DYN-T-RUN-003` | hidden 본 뒤 변경 시 재사용 거부 | `DYN-HID-001` |
| `DYN-T-STAT-001` | median improvement 식 oracle | 통계 계약 |
| `DYN-T-STAT-002` | paired stratified bootstrap 재현 | 통계 계약 |
| `DYN-T-STAT-003` | denominator floor 악화율 | 통계 계약 |
| `DYN-T-RUN-004` | hard failure가 promotion false로 전파 | 안전 자격 |
| `DYN-T-RUN-005` | hidden 실패 record 무덮어쓰기 보존 | 추적성 |
| `DYN-T-RUN-006` | JSON·Markdown·PNG 산출물 존재 | 추적성 |

## 완료조건

- 모든 hash가 manifest에 기록되고 재검증된다.
- hidden 30개가 PP·DWA paired Normal·Stress로 실행된다.
- hard safety와 fault 결과가 성능통계보다 먼저 판정된다.
- promotion decision의 각 조건에 근거 수치가 연결된다.
- 전체 시험, 문서 링크, `git diff --check`가 통과한다.
- 커밋·push 후 집 PC에서 `git pull --ff-only`로 동일 상태를 받을 수 있다.

## 집 PC 동기화

```powershell
git status --short
git fetch origin
git switch codex/path-planning-python-lab
git pull --ff-only
```

로컬 변경이 있으면 먼저 commit 또는 stash로 보존한다.

## 커밋 경계

```text
run frozen dynamic actor comparison experiment
```
