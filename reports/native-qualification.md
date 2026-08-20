# R7 native 자격 결과

## 판정

`qualified = true`

실행 기준 HEAD/tree에서 C++ native core를 다시 build한 뒤 Python과 C++ 결과를 비교하고, 계약시험과 직렬 timing을 실행했다. release gate의 12개 check는 모두 true다.

## 결과

| 항목 | 결과 |
|---|---:|
| C++ 재build | 통과 |
| Python↔C++ semantic parity | `5/5` |
| native contract parity | `13/13`, `12.10s` |
| timing 구성 | 5 사례 × warm-up 30회 + 측정 100회 |
| 직렬 측정 수 | `500` |
| `50ms` 초과 | `0/500` |
| p50 | `12.945200ms` |
| p95 | `29.909960ms` |
| p99 | `35.084894ms` |
| 최대 | `49.960700ms` |

timing은 다른 process worker 없이 직렬로 측정했다. cold-start 최대 `94.520400ms`는 별도 성능 저하 관측값이며 formal `500`회 자격 판정에는 넣지 않았다.

## 증거 결박

- native receipt content hash: `7002afcd55d204027e962c7c0edb01a01a8fdefc3bd3dd113b4e82098b504a5f`
- native evidence ZIP SHA-256: `6f0e8f5652792555c4d9fa9b6dcfcf44e618a2455201f5949e1c26821fc2f0c2`
- evidence ZIP 크기: `13,613` bytes

native evidence는 [최종 hidden-v5 evidence ZIP](../simulation/path_planning_lab/outputs/r7-hidden-v5-pass-evidence-20260820-3c8eb5f.zip)의 `native-release/` 안에 그대로 들어 있다.

## 범위 제한

이 수치는 이 PC·이 source·이 synthetic snapshot set의 Python/C++ 결과 일치와 실행시간을 보인 것뿐이다. 실제 차체의 실시간 제어 성능, 카메라 입력 성능 또는 실제 사람 안전을 보장하지 않는다.
