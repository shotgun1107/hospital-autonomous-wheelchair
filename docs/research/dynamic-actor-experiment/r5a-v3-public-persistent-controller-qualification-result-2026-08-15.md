# R5-A v3 persistent controller 공개 qualification 결과

- 실행일: `2026-08-15`
- 실행 commit: `7810432abbcb58534cf2a7b808553765d7396ce7`
- 실행 tree: `d711f4c1fe78525ab7a47fe94d827ed8d015f502`
- 범위: Python `simulation_only`, 정적 R4 `SPATIAL_ONLY` reference tracking
- 최종 판정: **PASS**
- receipt: **생성됨**
- hidden: **미사용**

## 1. 결론

R5-A의 공통 section executor 아래에서 persistent RPP와 source-derived DWB가 공개 21개 case를
모두 계약대로 처리했다. reference가 준비된 8개 case는 두 controller가 모두 종단 완료했고,
준비되지 않은 13개 case는 controller를 한 번도 호출하지 않았다.

hard failure, relation failure, planner deadlock, shared gate override는 모두 `0`이다.
serial/process parity와 repeat determinism도 통과했다. 따라서 이 실행은 R5-A
`STATIC_REFERENCE_TRACKING_QUALIFIED`의 공개 Python 증거다.

이 결론은 **정적 reference 추종 기능**에만 해당한다. Actor temporal execution, perception,
50ms 성능 자격, 실제 차체, 사람 탑승 안전, 제품 controller 선택은 증명하지 않는다.

## 2. 완료 tick

| 공개 case | RPP completion tick | DWB completion tick |
|---|---:|---:|
| `wide-straight-left` | 296 | 378 |
| `wide-straight-right` | 295 | 376 |
| `wide-mirror-left` | 296 | 380 |
| `wide-mirror-right` | 295 | 377 |
| `vertical-left` | 296 | 378 |
| `vertical-right` | 295 | 376 |
| `crossing-static-left` | 557 | 666 |
| `crossing-static-right` | 557 | 654 |

모든 ready case에서 RPP·DWB의 deadlock과 gate override는 각각 `0`이다.

## 3. 판정 증거

- public case: `21`
- ready paired case: `8`
- non-ready controller call: `0`
- hard failures: `()`
- relation failures: `()`
- serial/process parity: `PASS`
- repeat determinism: `PASS`
- audit semantic hash:
  `aef2ff56060762bab95198345c469f551aa05732891038e05b84c7d940e756e0`
- manifest content hash:
  `716c133fcd896c4cd8d8bcf667c05746bbeb733782097b2670821ee0e9dc845c`
- receipt content hash:
  `b6fcdfbed362bf887200dff817377ebe22ea91ae87338a9ee34d04038bcc1211`
- source freeze hash:
  `ee31ef846b4e06a1bd3421570b1091c0956132389ba87d327c11b83980467320`

파일 해시:

- `summary.json` SHA-256:
  `79601d51e1b7bc20503aa9a11390d1a6a3e3efcdb3c490ab70c6ba6aac5ee9d9`
- `qualification-receipt.json` SHA-256:
  `c9b09933f6237af06702888e8387f8e06fefe07d2e1631c4ecbe8d6cc7779c70`

## 4. 증거 보존

원본 output:

`simulation/path_planning_lab/outputs/persistent-controller-public-20260815-r5a-v3-7810432/`

Git 추적 전달용 ZIP:

`simulation/path_planning_lab/outputs/persistent-controller-public-20260815-r5a-v3-7810432.zip`

- 파일 수: `84`
- ZIP 크기: `3,462,064 bytes`
- ZIP SHA-256:
  `fe26d4143d98347d78b4416d7586f8c57e8b3d10ed96b3748040691d35486aa7`

ZIP에는 공개 21개 case의 source reference, RPP·DWB 결과, paired summary, PNG, run manifest,
complete state, 최종 summary와 qualification receipt가 들어 있다. hidden 결과는 없다.

## 5. 직전 실패와 해소

commit `5400000`의 첫 clean v3 실행은 개별 주행·parity·repeat를 모두 통과했지만 좌우 signed
relation에서 footprint yaw까지 mirror한 잘못된 판정 4건으로 receipt 없이 종료됐다. 좌우
reference는 중심 경로만 mirror이고, 한쪽은 전진하고 다른 쪽은 같은 chassis yaw로 후진한다.

최종 판정은 좌우 signed relation에서 중심 geometry를 비교하고, travel direction을 보존하는
수평·수직 rigid relation에서만 footprint axis를 추가 비교한다. 각 실행의 실제 oriented
footprint 안전은 기존 shared gate가 독립적으로 검사하며 완화하지 않았다.

## 6. 남은 범위

- R5-B Actor temporal execution과 기존 Actor 출현 관련 보류 항목
- R5-C observation/perception 입력 통합
- Python wall-clock과 분리된 native 50ms 자격
- 실제 차체·후방 센서·사람 탑승 검증
- hidden 실행
- 제품 controller 선택, G1~G5, 제품 경로분석 7단계

R5-A PASS를 위 항목의 통과나 DWB 제품 채택으로 확대 해석하지 않는다.
