from __future__ import annotations

from dataclasses import replace
from json import loads

import pytest

from hospital_path_lab.dynamic_contracts import (
    ACTOR_RADIUS_M,
    ActorState,
    Point2D,
    Vector2D,
)
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_prediction_audit import (
    MotionAuditSample,
    audit_directional_motion_samples,
    audit_public_prediction_contract,
    write_prediction_contract_audit,
)


@pytest.fixture(scope="module")
def public_audit():
    return audit_public_prediction_contract()


def _state(
    *,
    actor_id: str = "actor",
    x: float,
    y: float,
    vx: float,
    vy: float,
) -> ActorState:
    return ActorState(
        actor_id=actor_id,
        position=Point2D(x, y),
        velocity=Vector2D(vx, vy),
        radius_m=ACTOR_RADIUS_M,
        trajectory_revision=1,
    )


def test_motion_contract_accepts_bounded_deceleration_and_stop() -> None:
    samples = (
        MotionAuditSample(
            episode_id="bounded-stop",
            actor_id="actor",
            time_s=0.0,
            state=_state(x=0.0, y=0.0, vx=0.10, vy=0.0),
        ),
        MotionAuditSample(
            episode_id="bounded-stop",
            actor_id="actor",
            time_s=0.2,
            state=_state(x=0.01, y=0.0, vx=0.0, vy=0.0),
        ),
    )

    result = audit_directional_motion_samples(samples)

    assert result.passed
    assert result.stop_transition_count == 1
    assert result.deceleration_transition_count == 1
    assert result.maximum_acceleration_mps2 == pytest.approx(0.5)


def test_motion_contract_rejects_heading_change_and_lateral_motion() -> None:
    samples = (
        MotionAuditSample(
            episode_id="instant-turn",
            actor_id="actor",
            time_s=0.0,
            state=_state(x=0.0, y=0.0, vx=0.10, vy=0.0),
        ),
        MotionAuditSample(
            episode_id="instant-turn",
            actor_id="actor",
            time_s=0.2,
            state=_state(x=0.01, y=0.01, vx=0.0, vy=0.10),
        ),
    )

    result = audit_directional_motion_samples(samples)
    reasons = {item.reason_code for item in result.violations}

    assert not result.passed
    assert "lateral_motion_outside_contract" in reasons
    assert "heading_change_outside_constant_heading_contract" in reasons


def test_public_audit_separates_motion_and_statistical_coverage(public_audit) -> None:
    assert public_audit.passed
    assert public_audit.public_episode_count == 13
    assert public_audit.motion_contract.transition_count == 5_420
    assert public_audit.motion_contract.violations == ()

    ideal, normal, stress = public_audit.observation_coverage
    assert ideal.profile_name == "functional_ideal"
    assert ideal.dropout_count == 0
    assert ideal.exact_position_error_count == 0
    assert ideal.exact_velocity_error_count == 0
    assert ideal.component_position_coverage is None

    for measured in (normal, stress):
        assert measured.dropout_count > 0
        assert 0.0 < measured.radial_position_coverage < 1.0
        assert 0.0 < measured.component_position_coverage < 1.0
        assert measured.component_position_coverage > measured.radial_position_coverage
        assert measured.component_velocity_coverage > measured.radial_velocity_coverage
        assert measured.maximum_position_component_z > 2.0
        assert measured.maximum_velocity_component_z > 2.0

    ideal_capsule, normal_capsule, stress_capsule = public_audit.capsule_coverage
    assert ideal_capsule.coverage == 1.0
    assert ideal_capsule.miss_count == 0
    assert normal_capsule.miss_count > 0
    assert stress_capsule.unique_ready_prediction_count > 0
    assert "normal_capsule_coverage_misses_present" in public_audit.limitations
    assert public_audit.hard_failures == ()


def test_public_audit_is_deterministic(public_audit) -> None:
    repeated = audit_public_prediction_contract()

    assert repeated == public_audit
    assert repeated.content_hash == public_audit.content_hash


def test_public_audit_rejects_hidden_input() -> None:
    public = generate_dynamic_v6_public_corpus()
    hidden = replace(public[0], split=DynamicCorpusSplit.HIDDEN)

    with pytest.raises(ValueError, match="hidden or unsupported"):
        audit_public_prediction_contract((hidden,))


def test_writer_preserves_evidence_and_refuses_overwrite(tmp_path, public_audit) -> None:
    output_dir = tmp_path / "prediction-audit"

    json_path, summary_path = write_prediction_contract_audit(public_audit, output_dir)

    payload = loads(json_path.read_text(encoding="utf-8"))
    assert payload["content_hash"] == public_audit.content_hash
    assert payload["hard_failures"] == []
    summary = summary_path.read_text(encoding="utf-8")
    assert "Actor prediction 계약 감사 결과" in summary
    assert "Normal·Stress" in summary
    with pytest.raises(FileExistsError, match="already exists"):
        write_prediction_contract_audit(public_audit, output_dir)
