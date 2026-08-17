# R7 C++ DWB 시간 자격·후속 진입 Gate 결과

## 판정

- 실행일: `2026-08-17`
- 기준 commit: `12e4a7dbd0485c233363485b9b97d693d5938a3e`
- 기준 tree: `cae4c18d9a60a87a6c1b7a7ecf95458e9dbffa8e`
- R6 receipt 검증: 통과
- Python↔C++ 결과 동일성: `5/5` 통과
- C++ DWB 50ms 시간 자격: **실패**
- hidden: 미실행·진입 차단 유지
- R7 상태: 측정 완료, 자격 실패

R6의 공개 기능·안전 결과는 유지된다. 이번 실패는 C++ DWB가 선택된 집 PC와 복잡한 공개
입력에서 `20Hz` 계산시간을 보장하지 못했다는 뜻이다. DWB 기능 자체가 전부 무효이거나
제품 알고리즘으로 기각됐다는 뜻은 아니다.

## Python↔C++ 결과 동일성

다음 5개 공개 입력에서 상태, 선택 명령, trajectory, 실패 이유, 후보 진단, decision trace와
선택 안전 근거가 정확히 일치했다.

- Actor 없음
- Actor 1개
- Actor 2개
- 정적 장애물·금지 cell·코너
- 정적 장애물·금지 cell·여러 경로 구간

모든 사례에서 C++ 전체 DWB 코어가 실제로 사용됐다.

- parity content hash: `0659f4cc3c6bf90f27f9cb5ea3bcab8290c5254fea34554ec77e48a874d61ecf`
- parity file SHA-256: `70a28f3fd4d13e92a330fcd876ebd0cecf2a5ad549f2b8ef06d13c8c3f31fdc4`

## 정식 500회 시간 측정

- warm-up: 사례당 `30회`
- 측정: 사례당 `100회`, 합계 `500회`
- 방식: worker 없는 부모 process에서 직렬
- deadline: `50ms`
- 같은 tick cache: 사용하지 않음

| 입력 | p50 | p95 | 최대 | 50ms 초과 |
|---|---:|---:|---:|---:|
| Actor 없음 | `34.99ms` | `42.43ms` | `44.07ms` | `0/100` |
| Actor 1개 | `35.22ms` | `44.30ms` | `56.90ms` | `1/100` |
| Actor 2개 | `58.27ms` | `66.91ms` | `75.93ms` | `100/100` |
| 코너·정적 장애물·금지 cell | `246.25ms` | `270.56ms` | `321.12ms` | `100/100` |
| 여러 경로 구간·정적 장애물 | `139.06ms` | `157.34ms` | `171.52ms` | `100/100` |
| 전체 | `58.27ms` | `250.32ms` | `321.12ms` | `301/500` |

Actor가 없고 단순한 경우는 100회 모두 50ms 안이었다. 그러나 Actor 1개에서도 1회 초과가
있었고 Actor 2개, 코너와 여러 경로 구간은 지속적으로 초과했다. 따라서 좋은 사례만 골라
R7 통과로 처리하지 않았다.

새 controller의 첫 호출도 별도로 측정했다. 최대값은 코너 입력의 `258.53ms`였다. 이 값은
주 자격을 대체하지 않는 cold-start 자료다.

## 실행 환경

- CPU: `AMD Ryzen 7 9800X3D 8-Core Processor`
- core: 물리 `8`, 논리 `16`
- OS: `Windows 11 10.0.26200`
- Python: `3.12.13`, 64-bit
- compiler: Zig `0.16.0`, C++20, `-O3`
- 전원 정책: 최고의 성능
- process affinity: 논리 core `0~15`
- 병렬 worker: `0`
- native DLL SHA-256: `dfa167abd8294f6a4ad0e74ce7208cc046a786db39596f55d6461a040cba6bbe`
- snapshot set hash: `77f876702b4db90ea729740b3eb96187ae14fba26f045c2ea63c96dd678bb8af`
- source freeze hash: `c365c984b54a258a9bb0c98c700402e2caacfbc205157d67edecc41d8b923991`

운영체제 background process를 완전히 제거했다고 주장하지 않는다. 다만 runner가 별도 worker를
실행하지 않았고 측정 전후 source·HEAD·tree·작업트리가 동일했다.

## 산출물과 검증

- output: `simulation/path_planning_lab/outputs/r7-native-release-20260817-12e4a7d/`
- timing file SHA-256: `921db742a353bda545c4d21ab677837147835a5d60af53e073226c68a973173b`
- Gate file SHA-256: `fbca3105e1ba31e2ad1b5fe514be133ed2e674a4150c74fa48cf274a67044cc6`
- qualification receipt: 생성하지 않음
- 전체 회귀: `950 passed`, 실패·건너뜀 `0`
- Ruff·compileall·`git diff --check`: 통과

생성 output은 Git에 넣지 않았다. 축소 진단은 최종 근거로 승격하지 않았고 위 500회 완주
결과만 정식 R7 측정 근거로 사용한다.

## 결론과 다음 후보

1. C++ 이식의 계산 결과는 Python과 같다.
2. 현재 C++ 구현은 단순 입력에서는 빠르지만 복잡한 입력에서 50ms를 크게 넘는다.
3. 따라서 새 hidden 진입은 금지한다.
4. 다음 후보는 코너·정적 지도·다중 Actor에서 반복되는 거리 지도·안전 검사·경로 구간 계산을
   대상으로 한 **동작 보존형 native 최적화**다.
5. 후보 수, rollout, 안전거리, prediction, 비용과 shared gate를 줄이는 방식은 허용하지 않는다.

실제 카메라·실물 휠체어·사람 탑승 안전, 제품 DWB 채택, `G1~G5`와 제품 경로분석 7단계는
여전히 별도이며 이 결과로 결정하지 않는다.
