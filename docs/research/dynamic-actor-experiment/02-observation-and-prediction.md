# 2단계 — 관측 생성과 Actor 예측

## 목표

정확한 Actor ground truth에서 controller용 열화 관측을 생성하고, source 계약을 검증한
뒤 관측 age·noise·가속 편차를 반영한 time-indexed Actor tube를 만든다.

## 진입조건

- 1단계 결정론적 Actor trace 시험이 통과한다.
- controller 입력과 ground truth 자료형이 분리되어 있다.

## 수정·추가 대상

```text
src/hospital_path_lab/dynamic_contracts.py
src/hospital_path_lab/dynamic_observation.py
src/hospital_path_lab/dynamic_prediction.py
tests/test_dynamic_observation.py
tests/test_dynamic_prediction.py
```

## controller-facing frame

```text
DynamicObservationFrame
- stream_id
- episode_id
- episode_seed
- map_id
- map_revision
- observation_revision
- sequence
- observed_at_s
- delivered_at_s
- frame_kind: TRACKS | EMPTY
- tracks: tuple[ActorTrack, ...]
- content_hash

ActorTrack
- track_id
- actor_binding_id
- observed_position
- observed_velocity
- position_sigma_m
- velocity_sigma_mps
```

`ActorTrack`에는 future ground truth waypoint나 expectation category를 넣지 않는다.

## 관측 프로필

| 프로필 | 주기 | 지연 | TTL | 위치 σ | 속도 σ | 독립 dropout |
|---|---:|---:|---:|---:|---:|---:|
| Normal | 10 Hz | 100 ms | 300 ms | 0.03 m | 0.05 m/s | 5% |
| Stress | 10 Hz | 250 ms | 300 ms | 0.08 m | 0.15 m/s | 20% |
| Boundary | 10 Hz | 300/350 ms | 300 ms | 0 | 0 | 0 |

- x/y 위치·속도 noise는 서로 독립인 Gaussian이며 clipping하지 않는다.
- random stream은 episode seed에서 actor motion과 별도 namespace로 파생한다.
- dropout은 frame 자체를 전달하지 않는 사건이다.
- Actor가 없지만 source가 정상인 경우 `EMPTY` frame을 전달한다.
- 4-frame burst dropout은 일반 profile이 아니라 fault case로 생성한다.

## frame validation

다음은 즉시 invalid다.

- stream, episode seed, map ID 불일치
- sequence 또는 revision 역행
- content hash 불일치
- 같은 frame의 중복 track ID
- 기존 track ID의 actor binding 변경
- non-finite position·velocity·timestamp
- delivery 시각보다 미래인 observation timestamp

마지막 valid frame은 `age > 0.300 s`가 될 때 stale이다. `age == TTL` 경계는 fresh로
판정하되 사건 순서 변형 시험을 별도로 둔다.

## Actor tube

v5의 벡터식과 가속 편차식을 그대로 구현한다.

```text
tau = snapshot_age + 0.05 + rollout_time
predicted_center = observed_position + capped_velocity · tau
sigma_p = sqrt(sigma_p0² + (tau·sigma_v)²)
actor_tube_radius = 0.18 + 2·sigma_p + d_accel(tau)
```

`d_accel`은 임의 방향 속도 변화의 보수 상한을 사용한다. tube API는 특정 controller에
종속되지 않고 PP, DWA, gate가 같은 결과를 공유한다.

## oracle

- zero noise·zero acceleration case는 해석식과 일치해야 한다.
- 속도 상한을 넘는 관측은 방향을 유지한 채 `0.50 m/s`로 clamp한다.
- tube radius는 rollout time에 따라 감소하지 않아야 한다.
- 같은 frame과 같은 query time은 byte-equivalent 직렬화 결과를 만든다.

## 시험

| 시험 ID | 내용 | 연결 계약 |
|---|---|---|
| `DYN-T-OBS-001` | Normal·Stress 같은 seed 재현 | `DYN-ARCH-002` |
| `DYN-T-OBS-002` | fresh empty와 dropout 구분 | `DYN-OBS-001` |
| `DYN-T-OBS-003` | TTL 300/350 ms 경계 | `DYN-OBS-002` |
| `DYN-T-OBS-004` | source·revision·hash 음성시험 | `DYN-OBS-002` |
| `DYN-T-OBS-005` | ground truth 필드 누출 없음 | `DYN-ARCH-001` |
| `DYN-T-PRED-001` | 중심·σ·가속 반경 oracle 일치 | `DYN-SAFE-001` |
| `DYN-T-PRED-002` | radius 단조 비감소·finite | `DYN-SAFE-001` |
| `DYN-T-PRED-003` | 최대속도 vector clamp | `DYN-SAFE-001` |

## 완료조건

- Normal·Stress·Boundary frame stream을 seed로 재현할 수 있다.
- controller는 ground truth 없이 Actor tube를 계산한다.
- 모든 invalid/stale 원인이 구조화된 reason code로 반환된다.
- PP와 DWA는 아직 closed loop로 연결하지 않는다.

## 커밋 경계

```text
add deterministic dynamic observation and actor prediction
```
