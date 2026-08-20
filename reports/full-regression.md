# R7 전체 회귀 기록

## 판정

실행 기준 `3c8eb5f48478ae9ab80e7c19c3720684189d9e1c`에 대해 이미 완료된 전체 회귀는 `1,040 passed`, 실패 `0`, skip `0`이다.

| shard | 통과 수 |
|---|---:|
| 0 | 206 |
| 1 | 255 |
| 2 | 283 |
| 3 | 296 |
| 합계 | 1,040 |

이 결과는 실행 전에 작성된 [R7 동기화 인수인계](../docs/reviews/r7-final-qualification-sync-handoff-2026-08-19.md)의 0절에 기록돼 있다. 같은 source HEAD의 native 재자격과 hidden-v5 실행 뒤에는 코드가 변경되지 않았다.

## 이번 세션의 처리

사용자는 이미 통과한 전체 회귀 `1,040`개와 `500`회 timing을 다시 돌리지 말라고 지시했다. 그래서 이번 세션에서는 전체 회귀를 재실행하지 않고, clean worktree에서 native release gate와 hidden-v5만 실행했다. 이는 실행 시간을 줄이기 위해 사례·안전 기준·seed를 줄인 것이 아니다.

Ruff, compileall, `git diff --check`도 위 선행 자격 단계에서 통과한 것으로 기록돼 있다. 이번 native release gate는 source/tree가 실행 전후 동일하고 clean임을 다시 확인했다.

## 한계

이번 회사 PC clean worktree에는 선행 전체 회귀의 원본 shard stdout/stderr 파일이 남아 있지 않아, 그 로그를 새 evidence ZIP에 사후 복사하지 않았다. 따라서 이 문서는 선행 동결 자격의 정확한 shard 합계와 source provenance를 보존한 기록이며, 새 전체 회귀 실행 로그는 아니다.
