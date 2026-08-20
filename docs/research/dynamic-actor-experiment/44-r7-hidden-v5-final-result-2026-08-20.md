# R7 hidden-v5 최종 결과

- 상태: `PASS_FINAL`
- 실행 기준 HEAD: `3c8eb5f48478ae9ab80e7c19c3720684189d9e1c`
- 실행 기준 tree: `65928c4c3b2ca19d8ae1278d345c882b2dcbd2f0`
- 범위: offline·synthetic observation simulation
- 비범위: 제품 알고리즘 채택, `G1~G5`, 실제 카메라·사람·사람 탑승 안전

## 결과

R7 native release gate를 clean worktree에서 다시 build해 통과한 뒤, 새 v5 hidden을 원격 reservation으로 정확히 한 번 실행했다.

| 단계 | 결과 |
|---|---|
| 선행 전체 공개 회귀 | `1,040 passed`, 실패·skip `0` — 이미 완료된 동결 결과, 이번 세션 재실행 없음 |
| C++/Python parity | `5/5` |
| native contract parity | `13/13` |
| native timing | 직렬 `500/500`, `50ms` 초과 `0`, 최대 `49.960700ms` |
| hidden-v5 | `20/20 PASS_FINAL` |
| Normal | 완료 `10/10` |
| Stress | 조건부 보수 정지 `10/10`, 실제 release `2` |
| safety/authority 위반 | collision·forbidden·clearance·stale propulsion·unauthorized restart 모두 `0` |

hidden 결과의 최소 Actor clearance는 `0.228898261m`, 최소 static clearance는 `0.378628910m`이다. 동결된 `0.08m` 안전 기준은 바꾸지 않았다.

## 1회 실행 기록

- reservation ref: `refs/heads/codex/r7-hidden-v5-reservation`
- reserve → claim → final commit:
  `5c00491b2a1eab448b0bb1c12d4bf827bb0d6f91` →
  `28f2e52b944b16331cc74f5807abcc3385cd50a1` →
  `ea950dac4d91439e93162eda5a29682af5514a29`
- remote final state: `completed_pass`
- seed commitment: `1a84727b65a829f2a4492370655e255b39b386a8281f5ee14db539271bde3586`
- hidden receipt content hash: `69e4e4ca9a63e95babc357ce881c33a6512bf80cbe90c80bdf94b4550c616ff2`

v4의 `FAIL_ANALYZED` 결과와 seed는 그대로 보존했다. v5 결과로 과거 v4를 바꾸거나 재실행하지 않았다.

## 증거

최종 evidence ZIP: [r7-hidden-v5-pass-evidence-20260820-3c8eb5f.zip](../../../simulation/path_planning_lab/outputs/r7-hidden-v5-pass-evidence-20260820-3c8eb5f.zip)

- ZIP SHA-256: `7a1c1e0d8c757707769485ba190f1f86335c76d63cb62bbff0dd96dab7bb7115`
- ZIP 크기: `2,885,909` bytes
- 내용: 20개 JSONL trace, receipt, commitment, native release 결과, native evidence ZIP, 원격 reservation 최종 record, payload SHA-256 manifest

20개 trace는 총 `23,444` records이고 각 파일의 SHA-256, record 수, 마지막 hash, tick 연속성, hash chain을 다시 확인했다.

## 결론 범위

이는 교정된 R7 권한 추적과 조건부 Stress 정책이 이 동결 simulation 조건에서 실패하지 않았다는 증거다. 실제 센서·카메라 입력, 로봇 차체, 네트워크, 실제 사람 운동 또는 사람 탑승 안전에 대한 증거는 아니다.
