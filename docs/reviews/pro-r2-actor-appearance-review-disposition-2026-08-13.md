# Pro R2 Actor 출현 조사 반영 판정

## 1. 입력

사용자가 제공한 Pro 답변은 `2026-08-13` R2 Actor 출현 조사 ZIP의 외부·내부 SHA-256을
확인하고, 명세·코드·시험·실패 episode를 대조한 결과다.

이 문서는 Pro 답변을 그대로 승인하지 않고, 사용자의 최신 결정인 다음 범위와 대조한다.

> 실제 카메라 영상·FOV·가림·검출·추적은 후속으로 두고, 먼저 경로가 제대로 성립하는지
> 확인한다.

## 2. 승인하는 지적

다음은 코드·증거와 일치하므로 승인한다.

1. `fresh EMPTY → 빈 prediction → 비영점 이동 가능`은 online false-safe 경로다.
2. 지도 내부 Actor 순간 생성과 `100ms` latency를 동시에 두면 최초 관측 전 containment는
   수학적으로 불가능하다.
3. R1은 생성된 shape에서 Actor로 검사해 `Actor 있음 + shape 없음`을 놓쳤다.
4. 기존 hard failure 2건과 완료 receipt를 삭제·덮어쓰면 안 된다.
5. DWA/DWB, Python 성능과 방향성 Capsule 크기는 이 두 failure의 직접 원인이 아니다.
6. 향후 관측 연구에는 physical presence·visibility·observation·track·prediction lifecycle과
   EMPTY coverage 의미가 필요하다.

## 3. 범위를 보정하는 판정

Pro의 다음 판정은 기존 R2가 경로와 관측을 결합한 상태에서는 타당하지만, 경로 우선 lane을
분리한 뒤에는 과도하다.

> 관측 계약 수정 전에는 R3로 넘어가면 안 된다.

R3는 다음만 입력으로 받는다.

```text
static grid
allowed/forbidden region
wheelchair footprint
start pose
rejoin pose·heading tolerance
bounded motion primitives
```

R3는 카메라 frame, fresh EMPTY, ActorTrack, prediction shape, safety gate 또는 motion
authorization을 사용하지 않는다. 따라서 관측 false-safe는 R3가 답하려는 “차체가 공간적으로
지나갈 수 있는가”의 논리적 선행조건이 아니다.

보정 판정:

- Pro P0는 `R2-B`와 perception-integrated `R5~R7`의 P0로 유지
- `R2-A`와 `R3`에는 직접 P0로 적용하지 않음
- R3 결과를 online 이동 허가로 사용하는 것은 계속 금지

## 4. 현재 연구 진행 판정

```text
R2-A ground-truth path:
  부분 완료
  same-direction PASS·WAIT/HOLD 확인
  횡단 Actor 1건 SEARCH_INCONCLUSIVE
  다중 위험 재정지 1건 not fully covered

R2-B observation/prediction:
  실행 완료, 자격 실패
  hard failure 2건 보존
  카메라 통합 후속

R3 bounded spatial oracle:
  명세·구현 시작 가능
  R2-A unresolved spatial cases를 입력으로 받음
```

## 5. 바로 다음 작업

1. R1~R7 master와 R2 상세 명세에 R2-A/R2-B gate를 반영한다.
2. R2-A 미해결 사례를 R3 입력과 시간 witness 보완 대상으로 분리한다.
3. R3 상세 명세를 먼저 작성한다.
4. R3는 static 공간·차체 자세만 평가하며 관측 수치를 포함하지 않는다.
5. 카메라를 시작할 때 Pro의 coverage certificate·entry portal·occlusion 제안을 R2-B 신규
   명세 입력으로 다시 검토한다.

## 6. 최종 판정문

> Pro가 발견한 fresh EMPTY false-safe, 지도 내부 순간 생성과 R1 역방향 coverage 누락은
> 유효한 관측·online 통합 문제이며 기존 hard failure 2건을 그대로 보존한다. 다만 이 문제는
> exact ground truth로 시간 경로를 찾는 R2-A와 관측을 사용하지 않는 R3 공간 oracle을 막는
> 직접 선행조건이 아니다. 연구 gate를 R2-A 경로 lane과 R2-B 관측·카메라 lane으로 분리하고,
> R2-A의 횡단·다중 위험 미완료를 R3 입력으로 전달해 경로 연구를 계속한다. R2-B는 후속
> 미완료로 유지하며, 이를 통과하기 전에는 perception-integrated 종단 자격·hidden·제품 안전을
> 주장하지 않는다.

