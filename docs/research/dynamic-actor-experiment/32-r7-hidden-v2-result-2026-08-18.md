# R7 새 hidden 관측 시험 v2 결과

- 판정: `FAIL`
- 실행일: `2026-08-18`
- 실행 commit: `7af54976cc776f40ed6467f69bcdd7e0ca948f63`
- 실행 tree: `ed7058969ec3833abce8c6bd94bfac23e4045ac0`
- seed commitment: `c7db071bbcc1866bbcbd8d9219a7e6ea13ff394c7e0f9235b41f7ea8733e49be`
- 결과 집합 지문: `5bc46a04810d6778d54f128961bf600b27eb99e15f691bc577cf6de11038a42a`
- consumption receipt: `152c273c2fc104a24ccb2f2c7bb461d34051cd2bbd3eb9e7621478b8edd9587e`
- 공개 시간 자격 receipt: `35601ac0f51c3072cf36cf8b1282b709b1dc67af0f94804c10663c67407ba7be`
- 공개용 증거 ZIP: `simulation/path_planning_lab/outputs/r7-hidden-observation-v2-fail-public-evidence-20260818-7af5497.zip`
- 공개용 ZIP 크기: `6,706 bytes`
- 공개용 ZIP SHA-256: `13b1a2e971113353273b10c7ae94d58c2a5ea73889ef7b795fe6919a3cf09a94`
- root seed 포함 전체 ZIP: 로컬 수동보존, Git 커밋·원격 push 금지

## 1. 한 줄 결론

이전 hidden에서 공개 회귀로 옮긴 세 오류는 수정됐지만, 새로운 관측 순서 20건에서는 Normal
완료가 `4/10`에 그쳤고 Stress 한 건이 실제로 출발했으며, Normal 세 건에서 empty 관측 처리
예외가 발생했다. 따라서 현재 연구 기준선은 새 관측 누락 순서에 안정적이라고 판정할 수 없다.

## 2. 실행 조건

seed 생성 전에 다음을 완료했다.

- 수정 코드·판정·실행기를 commit·push
- 새 R7 공개 500회 시간 자격 `0/500`
- Python↔C++ 동일성 통과
- 실행 DLL SHA-256과 자격 receipt 결박
- 깨끗한 detached worktree 확인

OS 난수로 root seed를 한 번 만들고 20개 case를 10 process로 실행했다. 같은 replica·side의
Normal과 Stress는 같은 derived seed를 사용했다. 완료 순서와 무관하게 ordinal 순서로 다시
합쳤으며 wall-clock은 판정에 사용하지 않았다.

Git에 보존한 공개용 ZIP에서는 `consumed-seed.json`을 제외했다. commitment, case 결과,
summary와 receipt만 포함하므로 결과 검증은 가능하지만 root seed 원문은 공개되지 않는다.
원본 seed가 든 전체 ZIP과 실행 폴더는 이 회사 PC 로컬에만 남긴다.

## 3. 전체 결과

| 항목 | 결과 | 요구 | 판정 |
|---|---:|---:|---|
| 전체 case 완주 | `20/20` | `20/20` | 통과 |
| 개별 판정 통과 | `13/20` | `20/20` | 실패 |
| Normal 목적지 완료 | `4/10` | `10/10` | 실패 |
| Stress 최종 보수정지 | `10/10` | `10/10` | 결과만 통과 |
| Stress 무출발 | `9/10` | `10/10` | 실패 |
| hard failure | `3` | `0` | 실패 |

전체 최소 실제 Actor 여유는 약 `0.3551 m`, 최소 정적 장애물 여유는 약 `0.3786 m`였다.
동결된 `0.08 m`보다 크므로 이번 실패는 충돌이나 벽 접촉이 아니라 관측 누락 뒤 상태 처리와
완료 신뢰성 문제다.

## 4. 실패한 7건

### Normal 코드 예외 3건

다음 세 건은 모두 tick `784`에서 같은 예외로 중단됐다.

```text
R5-B empty observation is not post-pass completion input
```

- `hidden-00-left-normal`
- `hidden-01-left-normal`
- `hidden-03-right-normal`

세 사례 모두 여러 번 정지·재출발한 뒤 최종 `MOVING` 상태에서 empty 관측이 들어왔다. 기존
공개시험은 이 순서를 충분히 포함하지 못했다.

### Normal 보수정지 3건

- `hidden-02-left-normal`: 통과 증거 없이 `HOLDING`
- `hidden-04-left-normal`: 통과 증거 없이 `HOLDING`
- `hidden-04-right-normal`: post-pass 증거와 원 경로 복귀 허가는 있었지만 1600 tick 안에
  완료하지 못하고 `HOLDING`

이 세 건에는 코드 예외나 clearance 위반은 없었다. 그러나 Normal의 사전 합격조건은 목적지
완료이므로 모두 실패다.

### Stress 무출발 위반 1건

`hidden-03-left-stress`는 최종적으로 `conservative_hold`였지만:

- 첫 이동 tick: `222`
- controller 호출: `22`
- release tick: `221`, `333`
- gate override: `5`

따라서 “Stress에서는 처음부터 끝까지 움직이지 않음” 조건을 위반했다.

## 5. 통과한 Normal

다음 네 건만 목적지까지 완료했다.

- `hidden-00-right-normal`
- `hidden-01-right-normal`
- `hidden-02-right-normal`
- `hidden-03-left-normal`

이 결과는 특정 side가 항상 성공한다는 뜻이 아니다. RIGHT도 한 건은 코드 예외, 한 건은
미완료였고 LEFT도 한 건만 완료했다.

## 6. 판정과 다음 처리

이 hidden seed는 소비 완료됐으며 앞으로 공개 회귀자료로만 사용한다. 결과를 보고 코드를
고친 뒤 같은 seed를 최종 hidden으로 다시 사용하지 않는다.

다음 구현 순서는 공개 입력에서만 진행한다.

1. tick 784의 empty 관측과 post-pass 상태 전이를 공개 회귀시험으로 만든다.
2. Stress의 release tick 221 조건을 공개 회귀시험으로 만든다.
3. 코드 예외 없이 `HOLDING`으로 끝난 Normal 세 건은 통과 증거 부족과 반복 정지 원인을
   분리한다.
4. 공개 회귀를 수정한 뒤 전체 회귀와 현재 R7 공개 자격을 다시 확인한다.
5. 이후 또 다른 최종 hidden이 필요하면 새 명세·새 commitment·새 seed로 별도 승인한다.

안전 수치, observation profile, tick 수와 합격 기준은 이번 실패를 이유로 완화하지 않는다.
이 결과는 합성 관측 simulation 연구의 실패 기록이며 실제 카메라·실물 휠체어·실제 사람
안전 또는 제품 알고리즘 채택 증거가 아니다.
