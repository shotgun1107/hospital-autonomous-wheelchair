# Pro 시작 프롬프트 — R7 전체 경로 로직 최종 검토

당신은 `hospital-autonomous-wheelchair` 프로젝트의 R7 경로 로직을 최종 검토한다.
첨부 ZIP 안의 코드와 문서를 직접 읽고 판단하라. 이번 작업은 코드 수정이 아니라 읽기 전용
검토다. 확인하지 않은 내용을 추측으로 확정하지 마라.

## 기준

- repository: `https://github.com/shotgun1107/hospital-autonomous-wheelchair.git`
- branch: `main`
- HEAD: `54b4f04f06a22f6eebc05228a4f80abdfdd42615`
- tree: `25a373fc3332758fdd4d116221c26f7dc6766a9f`
- 현재 범위: 축소 POC용 offline·simulation 경로 연구
- 실제 사람 탑승 안전이나 제품 알고리즘 채택을 판단하지 않는다.

먼저 `AGENTS.md`를 완전히 읽고 안전 불변조건을 지켜라.

## 현재 확인된 결과

hidden-v3 20개를 한 번 실행한 결과는 다음과 같다.

```text
Normal 완료: 9/10
Stress 무출발 기준 통과: 9/10
hard failure: 0
충돌·금지구역·안전거리 실패: 0
```

두 실패는 원인이 달랐다.

1. Stress 1건은 정상 관측 11개가 쌓여 기존 규칙대로 출발한 뒤 frame 누락에서 다시
   안전하게 멈췄다. 구현은 조건부 출발을 허용하지만 시험은 80초간 무출발을 요구했다.
2. Normal 1건은 목적지 6.56cm 앞에서 정지·전진 후보 점수가 같아 정지를 선택했다.
   완료 제어는 5cm 안에서만 시작하므로 1.56cm의 정지 구간이 생겼다.

Normal 문제는 commit `54b4f04`에서 다음처럼 수정됐다.

- 원 경로의 마지막 전진 구간이다.
- 다음 구간이 목적지 방향 회전이다.
- 구간 끝점 0.10m 이내다.
- 위 조건에서만 같은 점수의 후보 중 전진 후보를 먼저 고른다.

실패했던 공개 회귀 입력은 수정 뒤 완료됐고 hard failure는 0이었다. 관련 시험은 `39 passed`다.
전체 회귀와 R7 50ms 자격은 이 수정 뒤 아직 다시 실행하지 않았다. 새 hidden도 실행하지 않았다.

## 검토할 코드 범위

ZIP에는 아래 전체가 들어 있다.

- `simulation/path_planning_lab/src/`
- `simulation/path_planning_lab/tests/`
- `simulation/path_planning_lab/native/`
- `simulation/path_planning_lab/scripts/`
- R7 문서 26~39
- lab `README.md`, `TRACEABILITY.md`, `pyproject.toml`
- hidden-v3 실패 증거 ZIP

특히 다음 흐름을 처음부터 끝까지 연결해서 읽어라.

```text
관측 검증
→ Actor 방향·위치 예측
→ local reference 생성·구간 전환
→ persistent DWB/RPP 명령 생성
→ shared safety gate
→ 감속·실제 정지 확인
→ stop_epoch와 재출발 승인
→ Actor 통과·원 경로 복귀
→ 목적지 정지·방향 정렬·완료
```

## 반드시 답할 질문

1. 충돌, stale 입력, 잘못된 재출발 또는 오래된 명령을 통과시키는 P0 문제가 남아 있는가?
2. 정지 후 다시 못 움직이거나 목적지 앞에서 영구 정지하는 P1 문제가 더 있는가?
3. `54b4f04`의 0.10m 동률 수정이 다른 구간에서 불필요한 전진을 만들 수 있는가?
4. Python과 native C++의 후보 생성·점수·동률·충돌검사 의미가 달라지는 곳이 있는가?
5. 동일 tick 재호출, frame dropout, stale 복구, 새 stop_epoch, Actor 소멸·재등장과 track 교체에서
   상태가 잘못 이어지는가?
6. Stress는 아래 어느 규칙이 현재 코드와 연구 목적에 맞는가?
   - 항상 운행 금지
   - 서로 다른 정상 관측 11개 뒤 조건부 출발, 이후 누락 시 즉시 안전정지
7. 새 hidden 전에 반드시 추가하거나 다시 실행할 공개시험은 무엇인가?

## 금지

- safety clearance, 정지 조건, 목적지 기준 또는 시험 기준을 결과에 맞춰 낮추지 마라.
- hidden-v3 실패를 합격으로 바꾸지 마라.
- 소비된 hidden seed를 새 최종 hidden으로 재사용하라고 제안하지 마라.
- 실제 카메라, 실제 사람 또는 제품 안전을 증명했다고 확대하지 마라.
- 단순 코드 스타일이나 성능 개선을 P0/P1처럼 과장하지 마라.

## 답변 형식

어려운 표현을 피하고 한국어로 작성한다.

1. 최종 판정: `승인 / 수정 후 승인 / 차단`
2. P0 문제: 파일·함수·줄 근거, 재현 조건, 실제 영향
3. P1 문제: 파일·함수·줄 근거, 재현 조건, 실제 영향
4. 기존 수정 `54b4f04`의 적절성 판정
5. Stress 규칙 권고와 이유
6. 필요한 최소 수정 순서
7. 수정마다 필요한 공개 회귀시험
8. 새 hidden 실행 전 체크리스트

문제를 발견하면 가능한 최소 수정 방향을 제안하되, 근거 없는 대규모 재작성은 제안하지 마라.
문제가 없다면 없다고 명확히 쓰고, 확인하지 못한 부분은 `미검증`으로 표시하라.
