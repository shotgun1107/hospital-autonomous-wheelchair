# R7 hidden-v5 결과

## 최종 상태

`PASS_FINAL`

hidden-v5는 v4와 다른 namespace·seed commitment를 사용했고, 원격 reservation을 reserve → claim → completed_pass 순서로 바꾼 뒤 정확히 한 번만 실행했다.

| 항목 | 값 |
|---|---:|
| 실행 ID | `e1b12bdea3f60917ff9a24b84f5b5ac879006f1c840d470b216d29e34752fc94` |
| case 수 | 20 |
| Normal 완료 | `10/10` |
| Stress 조건부 보수 정지 | `10/10` |
| Stress 실제 release | 2 |
| hard failure | 0 |
| 충돌 / 금지구역 / clearance 위반 | `0 / 0 / 0` |
| stale 중 추진 / 권한 없는 재출발 | `0 / 0` |
| release 계약 / 같은 frame 중복 누적 위반 | `0 / 0` |

최소 Actor clearance는 `0.228898261m`, 최소 static clearance는 `0.378628910m`으로, 동결된 `0.08m` 기준보다 크다.

## trace 검증

20개 case의 tick JSONL trace를 보존했다. 총 `23,444` record, `88,899,152` bytes이며, 각 파일의 SHA-256, record 수, 마지막 record hash, tick 연속성, record hash chain을 독립 재검증했다.

- result set hash: `1097c63dc8be281035570807c4f14d8d50c471fd156a973d3b4ea39c94aa4743`
- trace manifest hash: `94e3ad18286ca4a5190fae8054bc9287a3e754c235234369b014351862588086`
- trace set hash: `aa2227edcd1dee034c5ef8ff0ac1b59296b67dc2e616e4d9fdfa139c2362d900`
- hidden receipt content hash: `69e4e4ca9a63e95babc357ce881c33a6512bf80cbe90c80bdf94b4550c616ff2`

## 1회 실행 보장

| 항목 | 값 |
|---|---|
| reservation ref | `refs/heads/codex/r7-hidden-v5-reservation` |
| reserve commit | `5c00491b2a1eab448b0bb1c12d4bf827bb0d6f91` |
| claim commit | `28f2e52b944b16331cc74f5807abcc3385cd50a1` |
| final commit | `ea950dac4d91439e93162eda5a29682af5514a29` |
| final remote state | `completed_pass` |
| root seed commitment | `1a84727b65a829f2a4492370655e255b39b386a8281f5ee14db539271bde3586` |

원격 record에는 root seed를 넣지 않았다. [원격 최종 record](./evidence/r7-hidden-v5-remote-reservation-final.json)는 receipt와 commitment를 연결한다.

## 범위 제한

이 hidden은 새 synthetic observation noise·dropout sequence에 대한 offline simulation 결과다. 이 결과만으로 제품 알고리즘 선택, 실제 카메라 인식, 실제 환자·사람 탑승 안전, `G1~G5`, 제품 경로분석 7단계를 결정하지 않는다.
