# ADR 0009: 공개 DWA·DWB 기반 reference 구현

- 상태: 사용자 개인 승인, 팀 합의 전
- 날짜: 2026-08-12
- 범위: Python `simulation_only` 연구 실험

## 배경

기존 사용자 정의 DWA는 실제 추월과 재합류 일부를 만들었지만 우회 상태·점수·목표 접근과
안전검사가 한 구현에 섞였다. 공개 개발시험에 맞춘 예외가 늘었고 목표 근처 정체와 Python/C++
의미 불일치가 발생했다.

## 결정

ROS 1 `dwa_local_planner`와 ROS 2 Nav2 DWB의 고정 커밋을 소스 기준으로 분석하고, Nav2 DWB의
generator·constraint·critic 구조를 따르는 별도 Python reference 구현을 만든다.

- ROS 1 기준: `f44bb1fc2810399165115cc98b530fe4b9397c18`
- Nav2 기준: `1e8afb17e2e09df443b1870ce0f4ecdee32207fd`
- 기존 v6 사용자 정의 DWA는 legacy 회귀자료로 보존한다.
- 새 구현은 `source-derived v7` 연구 트랙으로 관리한다.
- Actor tube, terminal stopping, shared safety gate, provenance와 `stop_epoch`는 유지한다.
- Python 기능 의미가 통과하기 전 C++ 또는 ROS 2 adapter로 이식하지 않는다.

이 승인은 Software A의 Python 비교실험 진행 승인이며, 팀 전체의 제품 알고리즘 채택이나
경로 분석 7단계 결정이 아니다.

상세 분석과 구현 계약은
[`07-open-source-dwa-dwb-analysis-and-adaptation.md`](../research/dynamic-actor-experiment/07-open-source-dwa-dwb-analysis-and-adaptation.md)를 따른다.

## 결과

장점:

- 공개 현장에서 사용된 구조를 기준으로 후보 생성·점수·목표 접근을 재현할 수 있다.
- DWA core와 프로젝트 고유 안전 계약을 분리해 실패 원인을 추적할 수 있다.
- Python과 향후 native/ROS 2 구현의 동등성 기준이 생긴다.

비용과 제한:

- 기존 v6의 `217개×41 pose`와 사용자 정의 비용식은 새 구현의 정답이 아니다.
- v7은 새 manifest, 공개 qualification과 새 hidden commitment가 필요하다.
- 최초 공개 진단에서 현 Actor reachable tube가 `217/217` 후보를 제거했다. 공개 full 실행 전
  corpus의 우회 가능 분류와 Actor 운동 불확실성 계약 중 무엇을 유지할지 별도 결정해야 한다.
- 이 결정은 최종 제품 알고리즘 채택이나 실제 사람 탑승 안전 승인이 아니다.
