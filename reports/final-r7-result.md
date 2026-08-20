# R7 최종 자격 결과

## 상태

`PASS_FINAL`

R7 hidden-v5는 한 번만 실행됐고, 실행 전 native 자격과 실행 뒤의 결과·trace·원격 reservation 기록이 서로 맞는다. 이 결과는 동결된 synthetic Actor·관측 시뮬레이션에서의 R7 자격 결과다. 제품 알고리즘 채택, `G1~G5` 결정, 실제 카메라 성능, 실제 사람 또는 사람 탑승 안전을 뜻하지 않는다.

## 실행 기준

| 항목 | 값 |
|---|---|
| 코드 HEAD | `3c8eb5f48478ae9ab80e7c19c3720684189d9e1c` |
| 코드 tree | `65928c4c3b2ca19d8ae1278d345c882b2dcbd2f0` |
| 실행 위치 | 별도 clean worktree |
| hidden 상태 | `PASS_FINAL` |
| hidden 실행 횟수 | 정확히 1회 |

이 문서와 evidence ZIP을 추가하는 후속 commit은 실행 코드 기준을 바꾸지 않는다. 실행의 정본은 위 HEAD/tree다.

## 결과 요약

- 공개 전체 회귀: `1,040 passed`, 실패·skip `0` — 실행 기준 commit에 대해 앞선 자격 단계에서 4개 독립 shard로 완료됐으며, 이번 세션에서는 사용자 지시에 따라 다시 실행하지 않았다. 자세한 출처는 [전체 회귀 기록](./full-regression.md)에 남겼다.
- native: C++/Python 동일성 `5/5`, 계약시험 `13/13`, 직렬 `500/500`에서 `50ms` 초과 `0`.
- hidden-v5: `20/20` 통과. Normal 완료 `10/10`, Stress 조건부 보수 정지 `10/10`이며 실제 조건부 release는 `2`회였다.
- 안전 결과: 충돌·금지구역 진입·`0.08m` clearance 위반·stale 중 추진·권한 없는 재출발·release 계약 위반·같은 safe frame 중복 누적은 모두 `0`회다.

## 증거

| 산출물 | SHA-256 | 크기 |
|---|---|---:|
| [hidden-v5 최종 evidence ZIP](../simulation/path_planning_lab/outputs/r7-hidden-v5-pass-evidence-20260820-3c8eb5f.zip) | `7a1c1e0d8c757707769485ba190f1f86335c76d63cb62bbff0dd96dab7bb7115` | `2,885,909` bytes |
| ZIP 안 native release evidence | `6f0e8f5652792555c4d9fa9b6dcfcf44e618a2455201f5949e1c26821fc2f0c2` | `13,613` bytes |

외부 ZIP은 20개 JSONL trace, hidden receipt, native 결과, 원격 reservation 최종 기록과 각 payload SHA-256 목록 `EVIDENCE_MANIFEST.sha256`를 담는다. ZIP 자체와 manifest를 각각 검증했다.

## 원격 1회 실행 기록

- reservation ref: `refs/heads/codex/r7-hidden-v5-reservation`
- 최종 ref commit: `ea950dac4d91439e93162eda5a29682af5514a29`
- 원격 상태: `completed_pass`
- seed commitment: `1a84727b65a829f2a4492370655e255b39b386a8281f5ee14db539271bde3586`
- hidden receipt content hash: `69e4e4ca9a63e95babc357ce881c33a6512bf80cbe90c80bdf94b4550c616ff2`

원격 reservation에는 원본 root seed를 기록하지 않았다. 최종 원격 record의 복사본은 [여기](./evidence/r7-hidden-v5-remote-reservation-final.json)에 있다.

## 보존한 역사 결과

hidden-v4의 공식 결과 `FAIL_ANALYZED`와 그 seed는 수정하거나 재실행하지 않았다. v5는 교정된 새 namespace와 새 seed commitment를 사용한 별도 단 한 번의 실행이다.
