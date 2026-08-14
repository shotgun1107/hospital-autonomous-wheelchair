# R3 bounded 공간 Oracle 공개 qualification 결과

- 실행일: 2026-08-14
- 판정: **PASS — 동결된 offline static bounded-lattice 연구 범위**
- 평가 commit: `53fd9f863761f7ca850ea21b085cb9357351b2f6`
- 평가 tree: `fda5c6607efab13bbcc7a96d465a1aa424e2ef13`
- hidden 사용: 없음
- 제품 알고리즘 채택: 하지 않음

## 1. 실행 범위

동결된 21개 public request를 14개 process로 병렬 계산했다. 각 request 내부 state expansion은
기존 결정론적 직렬 순서를 유지했고, 완료 순서와 worker 수는 semantic 결과에서 제외했다.
검색 입력에는 Actor, 관측, controller, expectation category, oracle label과 hidden 정보를 넣지
않았다. crossing 두 건은 공개 legacy 장면에서 Actor와 시간축을 제거한 static projection이다.

```text
wide·mirror·vertical feasible
narrow corridor·door·dead-end negative
corner safe·rotation-blocked
forbidden-only·allowed-region pinch
start·goal unsafe
exact resource boundary·limit plus one
invalid provenance
legacy crossing static LEFT·RIGHT
```

## 2. 최종 판정

| 항목 | 결과 |
|---|---:|
| public request 완료 | `21/21` |
| case 기대 status·reason 불일치 | `0` |
| 관계 검사 오류 | `0` |
| serial/process semantic parity | `PASS` |
| feasible path independent validation 실패 | `0` |
| PNG | `21` |
| partial state 잔존 | `0` |
| clean-source qualification receipt | 생성 |

주요 양성 결과는 다음과 같다.

| case | path length | minimum clearance | rotation count |
|---|---:|---:|---:|
| wide LEFT/RIGHT·mirror·vertical | `1.220m` | `0.244509m` | `2` |
| just-wide door | `1.220m` | `0.107500m` | `2` |
| corner-safe proxy | `0.795980m` | `0.247643m` | `2` |
| crossing static LEFT/RIGHT | `4.070996m` | `0.367764m` | `2` |

negative는 `narrow-door`, `dead-end`, `corner-rotation-blocked`, `forbidden-only-block`,
`allowed-region-pinch`, `resource-exact`에서 bounded lattice를 소진했다. `resource-exact`는
expanded `1,056`, generated `4,320`, peak open `233`에서 정상 exhaustive였고,
expanded 한계를 `1,055`로 낮춘 `resource-plus-one`은 `RESOURCE_LIMIT`로 분리됐다.

## 3. 증거 결박

```text
manifest:
f94c720ea865b072d92c9d1d79a19677e28667b18093d9cea632da0a0a4bf5ba

source freeze:
4513b30163e1046a703eac7d588346ef779fc2ce24d68b76f06b12d6123ab9c6

catalog:
cfe21dd32199012546b94f52adcd78c82625bd7570d1878631270f2cfd9df126

audit semantic:
01b13fbeb83c7a6c8c25ce9ea6b726bb36112ad3c22c79fc72aa2d780ab4cfe7

receipt:
79514fda8ae8677cbc2bababd56fe37cf60e389dd7882dd20138dad751f09c01
```

생성 산출물은 Git에 커밋하지 않고 다음 로컬 경로에 보존한다.

```text
simulation/path_planning_lab/outputs/
  spatial-oracle-public-20260814-53fd9f8/
```

## 4. 회귀 검증

- R3 직접 영향권: `38 passed`
- 전체 회귀: `729 passed`
- 전체 회귀 분할: `58`개 test file, `14` process
- failures·errors·skips: `0 / 0 / 0`
- 전체 회귀 output:
  `simulation/path_planning_lab/outputs/test-runs/20260814-r3-public-final2-53fd9f8/`

첫 오케스트레이션은 Windows 환경의 `Path`/`PATH` 중복으로 worker 생성 전에 실패했다.
테스트는 `0`건 실행됐고 해당 빈 partial output을 보존했다. 같은 명령을 반복하지 않고 .NET
process API로 전환해 두 번째 실행을 완료했다. 이 운영 우회는 시험 파일, case, state bound,
안전 기준과 판정을 변경하지 않았다.

## 5. 결론과 한계

이번 결과로 말할 수 있는 것은 다음뿐이다.

> 동결된 정적 grid, 가상 차체, 8-heading translation+in-place-rotation bounded lattice 안에서
> 지정 public 공간 사례의 양성·음성·resource·invalid 분류가 독립 validator와 함께 일관되게
> 동작했다.

다음은 증명하지 않았다.

- Actor가 있는 시간 경로의 실행 가능성
- 관측·prediction·권한·shared safety gate 통합
- multi-segment public corner projection
- smooth curvature 또는 실제 차체 추종 가능성
- online controller 동작, 제품 알고리즘 채택, G1~G5
- 실제 사람 탑승 안전성 또는 의료기기 인증

따라서 R3 positive path는 R4의 local reference/subpath 후보 입력으로만 전달한다. R3 path 자체를
구동 명령이나 재개 허가로 해석하지 않는다.
