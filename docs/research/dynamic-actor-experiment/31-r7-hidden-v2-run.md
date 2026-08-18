# R7 새 hidden 관측 시험 v2

- 상태: 실행 승인, seed 생성 전
- 승인일: `2026-08-18`
- 실행 기준 코드: `2642965611a27c11111cdef2829d8d46cfed367b`
- 수정 결과: [30-r7-failure-fix-and-public-regression-result-2026-08-18.md](./30-r7-failure-fix-and-public-regression-result-2026-08-18.md)
- 이전 hidden 결과: [28-r7-hidden-observation-result-2026-08-18.md](./28-r7-hidden-observation-result-2026-08-18.md)

## 1. 목적

이전 hidden에서 발견해 공개 회귀로 옮긴 세 코드 오류를 수정한 뒤, 공개시험에 사용하지 않은
새 관측 잡음·dropout 순서에서도 같은 연구 기준이 유지되는지 한 번 확인한다.

이 시험은 기존 v1 seed를 재사용하지 않는다. 새 seed 생성 전에 실행기, 판정, 코드와 새 R7
공개 자격 증거를 먼저 commit·push한다.

## 2. 새 공개 자격

R7 frozen source 중 `persistent_adapter.py`가 공개 오류 추적을 위한 읽기 전용 상태 노출로
변경됐으므로, 기존 500회 시간 증거를 그대로 사용하지 않고 현재 코드에서 다시 측정했다.

- 실행 commit: `2642965611a27c11111cdef2829d8d46cfed367b`
- Python↔C++ 동일성: 통과
- 표본: `500`
- 50 ms 초과: `0/500`
- p50: `12.651 ms`
- p95: `29.161 ms`
- 최대: `36.654 ms`
- receipt: `35601ac0f51c3072cf36cf8b1282b709b1dc67af0f94804c10663c67407ba7be`
- evidence ZIP: `simulation/path_planning_lab/outputs/r7-native-v3-public-qualification-evidence-20260818-2642965.zip`
- ZIP 크기: `7,771 bytes`
- ZIP SHA-256: `81bed89b078c77f964cac56a9da33979fc86b9bd8b7600b06824cfe0c8297c42`

hidden 실행기는 위 ZIP의 크기·해시·source freeze·receipt뿐 아니라 실제 실행에 로드하는 두
native DLL의 SHA-256도 receipt와 일치하는지 확인한 뒤에만 seed를 만든다.

## 3. 동결 조건

v1과 동일하게 다음을 유지한다.

- 5개 새 관측 순서 × LEFT/RIGHT × Normal/Stress = 20 cases
- 같은 replica·side의 Normal과 Stress는 같은 derived seed 사용
- 기존 지도·reference·Actor trajectory와 Normal·Stress profile
- 20 Hz control, 10 Hz observation, 1600 ticks
- 0.08 m clearance, 공통 safety gate와 현재 stop_epoch 재승인
- Normal 10건은 모두 완료, Stress 10건은 모두 무출발 보수정지
- hard failure 0

독립 case는 10 process로 병렬 실행하고 ordinal 순서로 다시 합친다. wall-clock은 합격 기준이
아니다.

## 4. seed 수명주기

1. 이 문서·실행기·증거 ZIP을 먼저 commit·push한다.
2. 깨끗한 detached worktree에서 OS 난수로 root seed를 한 번 만든다.
3. 실행 전 기록에는 commitment만 남기고 seed 원문은 노출하지 않는다.
4. 실행 시작 뒤 `consumed-seed.json`에 seed를 보존한다.
5. 결과를 본 뒤 코드를 바꾸면 이 seed는 회귀자료가 되며 최종 hidden으로 재사용하지 않는다.
6. partial 결과는 보존하되 최종 증거로 사용하지 않는다.

## 5. 증거 범위

PASS여도 동결된 합성 관측 순서에서 simulation 연구 기준이 유지됐다는 뜻뿐이다. 실제 카메라,
실물 휠체어, 실제 사람 안전, 제품 알고리즘 채택과 G1~G5의 증거가 아니다.
