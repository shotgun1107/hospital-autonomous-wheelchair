# R7 hidden-v5 교정 자격 명세

- 상태: 공개 수정·검증 진행 중, hidden 미실행
- 기준 역사 결과: [42-r7-hidden-v4-fail-analyzed-result-2026-08-19.md](./42-r7-hidden-v4-fail-analyzed-result-2026-08-19.md)
- 범위: offline·simulation 관측 순서 자격
- 비범위: 제품 알고리즘 채택, `G1~G5`, 실제 카메라·사람 안전

## 1. 역사 결과 보존

2026-08-19 hidden-v4는 이미 한 번 소비됐고 공식 결과는 `FAIL_ANALYZED`다. 같은 root seed
`6564067906066881700`은 영구 거부하며, 기존 ZIP·receipt·문서와 v4 runner의 역사 산출물은
수정하거나 PASS로 재분류하지 않는다.

## 2. 공개 교정 범위

거짓 실패의 원인은 pipeline이 실제로 사용한 one-shot 재출발 권한을 cleanup한 뒤 trace가
사후 runtime 슬롯을 읽은 데 있다. 교정은 다음만 수행한다.

- gate에 전달된 불변 `ResumeAuthorization`을 cleanup 전에 trace evidence로 기록
- mission, stop epoch, revision, hash와 정지 확인 이후 발행 시간 검증
- R5-B temporal authorization의 reference session·reference hash·tick·phase·hash 검증
- 실제 release에 성공한 authorization hash의 재사용을 gate 수명 동안 거부
- JSONL 저장·재읽기 뒤에도 같은 판정 유지

안전거리 `0.08m`, distinct safe frame `11`, stale/dropout, stop epoch/session, shared gate와
controller 기준은 완화하지 않는다.

## 3. v5 분리와 1회 실행

교정 후 새 hidden은 다음 식별자를 사용한다.

```text
execution namespace = r7-hidden-v5-execution-v1
observation namespace = r7-hidden-observation-v5
case prefix = hidden-v5-
```

집·회사 clone에서 중복 실행하지 못하도록 seed 생성 전에 GitHub 원격 reservation ref를
원자적으로 선점한다. 지정 실행자가 reservation을 `execution_started_before_seed`로 전환한 뒤에만
seed를 생성할 수 있다. 다른 clone의 reserve·claim은 non-fast-forward push로 실패한다. 원격
reservation에는 root seed를 기록하지 않고 commitment와 최종 receipt hash만 남긴다.

## 4. 실행 게이트

다음이 모두 통과한 단일 clean commit에서만 v5를 정확히 한 번 실행한다.

1. 정적 검사와 표적 authorization/public 회귀
2. 전체 공개 회귀
3. 현재 source로 새 native build와 Python↔C++ parity
4. 직렬 `5×100=500` timing의 `50ms` 초과 0
5. source freeze·native hash·qualification receipt 결박
6. 원격 reservation과 지정 실행자 일치

하나라도 실패하면 seed를 생성하지 않는다. 실행 시작 뒤 기반시설 오류가 발생하면
`BLOCKED_INFRASTRUCTURE`, 기능·안전 판정 실패면 `FAIL_REQUIRES_ANALYSIS`로 동결하고 같은
execution을 다시 실행하지 않는다.
