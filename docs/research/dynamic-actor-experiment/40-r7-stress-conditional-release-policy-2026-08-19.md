# R7 Stress 조건부 재출발 공개 정책

- 상태: 공개 회귀 정책 확정, 새 hidden 미실행
- 기준 코드: `54b4f04f06a22f6eebc05228a4f80abdfdd42615` 이후 Python 수정안
- 적용 범위: R7 연구용 시뮬레이션의 Stress 관측 profile 평가
- 비적용 범위: 제품 알고리즘 채택, 실제 카메라·사람 안전 주장

## 1. 결정

Stress profile이라는 이름만으로 운행을 영구 금지하지 않는다. 다음 조건을 모두 만족할 때만
조건부 재출발을 허용한다.

1. shared safety gate가 서로 다른 fresh observation frame에서 안전을 11회 확인한다.
2. 같은 10Hz observation frame을 20Hz control tick에서 두 번 세지 않는다.
3. release tick의 입력이 usable하고 stale·dropout·invalid 상태가 아니다.
4. reference, resume authorization과 shared gate의 `stop_epoch`가 일치한다.

출발 뒤 dropout, stale, invalid observation 또는 prediction loss가 발생하면 새 추진 명령을
계속 만들지 않고 제한감속을 시작한다. 실제 `HOLDING`을 확인한 뒤에만 `stop_epoch`를
증가시키며, 이전 session·reference·authorization은 재사용하지 않는다.

## 2. hidden-v3 결과 보존

기존 hidden-v3는 Stress의 80초 무출발을 요구했으므로 `FAIL` 기록을 그대로 보존한다. 이번
정책 결정으로 과거 결과를 소급해 PASS로 바꾸지 않는다. 소비된 root seed와 observation
seed도 새 최종 hidden에 재사용하지 않는다.

다음 hidden은 새 commitment namespace와 새 evaluator version을 공개한 뒤에만 실행한다.
기존 `r7_hidden_qualification.py`의 hidden-v3 evaluator는 역사적 결과 재현용이므로 이번
수정에서 변경하지 않는다.

구현된 후속 evaluator는 `r7_hidden_v4_qualification.py`이며 namespace는
`r7-hidden-observation-v3`, case prefix는 `hidden-v4-`다. 실제 실행 전 qualification과
runner freeze 조건은 [41번 문서](./41-r7-hidden-v4-conditional-evaluator-2026-08-19.md)를 따른다.

## 3. 공개 회귀

다음 소비 입력을 공개 회귀로 고정한다.

```text
side = LEFT
profile = Stress
observation_seed = 214092870162924582
tick_limit = 503
```

기대 상태 전이는 다음과 같다.

```text
tick 497: gate-confirmed safe evidence 11개 이상, 조건부 release
tick 498: 첫 실제 이동
tick 499: stale/no-frame, controller 미호출, MOVING → BRAKING
tick 502: 실제 정지 확인, BRAKING → HOLDING, stop_epoch 증가
```

합격 조건은 hard failure 0, 실제 clearance 0.08m 이상, stale 추진 0, 최종 `HOLDING`이다.
