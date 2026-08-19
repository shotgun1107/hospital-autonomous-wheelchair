# R7 hidden-v4 최종 자격 실패 분석 결과

## 1. 최종 판정

actual hidden-v4 20개를 정확히 한 번 실행했다. 공식 판정은 `FAIL_ANALYZED`다.

```text
Normal 동작 완료                         10/10
Stress 보수 정지                         8/10
Stress 조건부 출발 뒤 안전 재정지         2/10
hard failure                              0
충돌·금지구역·0.08m 거리 위반             0
형식상 release 계약 위반                  39
형식상 unauthorized restart               39
```

마지막 두 수치는 39번의 실제 무단 재출발을 뜻하지 않는다. 같은 39개 release record에서
승인 객체를 사용한 뒤 지운 사후 상태를 evaluator가 승인 증거로 요구해 같은 누락을 두 항목으로
센 결과다. 이 결함을 hidden 결과를 본 뒤 고쳐 PASS로 바꾸지 않는다. 현재 실행은 실패로
보존하고, 같은 seed를 최종 hidden으로 다시 사용하지 않는다.

## 2. 실행·증거 정보

| 항목 | 값 |
|---|---|
| 실행 HEAD | `1eb5011e84caffa346abd40aeda711fadfe169f7` |
| 실행 tree | `d6edeed84652f53aabe1432bd7683ff5bb0c8e31` |
| case | `20/20` |
| seed commitment | `24c8345635d953356e5b3a980f1803200aa9d87471f7419285e47afcf551e2cc` |
| result set | `381fe8f2ccdb6b63b139864ecc4110c26df697cc627501c9895bc47f5d83ae19` |
| case trace set | `49e61b598e556570580317226e0aef73f10e44167773e55967353e25290bd23c` |
| trace manifest | `8a85d91f86a6dd03d96d4c3f09b2bfc89152207a3f0dd2bb902a84548d027302` |
| consumption receipt | `5596c93d3a279f2a5ef4b5832e99276c818890a07eedade11b8aa6d8acbca348` |
| packaging source freeze | `d13d1b28ca764fc112312f3b36867e83b0fee2275718d67ad4c872c78a44e982` |

원본 증거 ZIP:

```text
simulation/path_planning_lab/outputs/r7-hidden-v4-fail-analyzed-evidence-20260819-1eb5011.zip
size: 9,835,156 bytes
SHA-256: 0df82eb0f5eb184c1f2e65190361d917a4b8ba9c2fb04ed675a4be15eab538c5
entries: 318
```

ZIP은 hidden 20개 전체 JSONL trace, case 결과, manifest·receipt, one-use ledger, native release
evidence와 전체 회귀 로그를 포함한다. 내부 manifest 317개와 payload exact set이 일치하며 ZIP
CRC·payload SHA-256·중복·위험 경로 검사를 통과했다.

## 3. hidden 전 자격

- 포장 차단점 수정 영향권: `41 passed`
- R7 공개 회귀: `65 passed`
- 전체 회귀: 4개 고정 shard 합계 `1,035 passed`, 실패·건너뜀 `0`
- native semantic parity: `5 case`
- native contract parity: `13 tests`
- 직렬 시간 자격: `500 samples`, `50ms` 초과 `0`
- native release evidence: `13,403 bytes`, SHA-256
  `1b40727df3ab6dce66d951d1bc132f9a21c2e39cdf4bff8fbb39211cb469dcda`

Windows Unicode 작업 경로에서 pytest-xdist 초기화가 실패해 전체 회귀는 각 시험이 정확히 한
shard에만 들어가도록 4개 process로 나눴다. 네 shard는 각각 `190`, `267`, `278`, `300`
passed다. xdist와 첫 잘못된 분할 시도는 시험 0개를 실행했으므로 결과에 세지 않았다.

## 4. 실제 동작 관찰

- Normal 10개는 모두 Actor 통과, 원 경로 복귀와 목적지 완료까지 갔다.
- Stress 8개는 출발하지 않고 정지를 유지했다.
- `hidden-v4-01-right-stress`, `hidden-v4-03-right-stress`는 각각 한 번 출발한 뒤 관측 조건이
  무너지자 안전하게 다시 멈췄다.
- 최소 Actor 여유는 `0.2191699586m`, 최소 정적 여유는 `0.3751054425m`다.
- 중복 safe frame, stale 상태 추진, 실제 충돌, 금지구역 진입과 `0.08m` 미만 여유는 모두 0이다.

이는 simulation 20개에서 관찰한 결과일 뿐 실제 카메라·사람·환자·제품 안전 증거가 아니다.

## 5. 실패 원인

release record는 총 39개다. 37개는 Normal, 2개는 Stress다. 39개 모두 다음 조건을 만족했다.

- fresh usable frame
- no-frame 아님
- 서로 다른 safe frame 11개 이상
- release 직전 gate `holding`
- runtime 없음 → 있음
- reference stop epoch와 실제 stop epoch 일치
- controller 호출과 gate의 `moving` 전환
- release 없이 runtime이 생긴 record 0

유일하게 39/39에서 빠진 값은 trace의 `resume_authorization_revision`이다.

실행 흐름은 release 때 `build_resume_authorization(...)`으로 새 승인 객체를 만들고
`PersistentControllerPipeline.step(...)`에 전달한다. pipeline 호출 직후 one-shot 승인 객체를
`None`으로 지운 다음 trace를 만들기 때문에 trace에는 revision이 남지 않는다. 그러나
`_trace_contract_proof(...)`는 그 사후 필드가 반드시 남아 있어야 한다고 검사한다. 따라서
release가 발생하면 정상 여부와 관계없이 contract violation과 unauthorized restart를 각각 1개씩
만드는 구조였다.

20개 release에서는 temporal issuer가 `issued`를 기록했고 오류가 없었다. 나머지 19개는 해당
경로가 temporal issuer를 사용하지 않지만 launch가 직접 만든 resume authorization을 pipeline에
전달했다. 두 경로 모두 승인 객체를 소비한 뒤 trace가 만들어져 revision은 `None`이었다.

즉 이번 실패의 직접 원인은 경로 제어·충돌 회피가 아니라 **one-shot 승인 증거를 소비 전에
기록하지 않은 trace 계약과, 소비 뒤 상태를 요구한 evaluator의 불일치**다. 다만 증거 계약이
실패했으므로 공식 PASS로 승격할 수 없다.

## 6. 다음 재개 지점

1. public fixture에서 실제 release 흐름을 그대로 실행해 현재 evaluator가 반드시 실패하는
   회귀시험을 먼저 고정한다.
2. 승인 객체의 revision·hash·epoch를 pipeline 소비 전에 별도 trace evidence로 캡처하고,
   evaluator는 그 불변 증거를 검증하게 한다.
3. 실제 무단 runtime 생성 mutation과 stale/epoch mismatch가 계속 거부되는지 확인한다.
4. 공개시험·전체 회귀·native parity·500회 시간 자격을 새 코드에서 다시 통과한다.
5. 새 source namespace와 새 seed를 사용할 다음 hidden은 별도 사용자 승인 뒤에만 설계한다.

현재 hidden-v4 seed는 소비됐다. hidden-v4 재실행, 수치 완화, 결과 재분류, 제품 알고리즘 채택,
`G1~G5` 또는 제품 경로분석 7단계는 시작하지 않는다.
