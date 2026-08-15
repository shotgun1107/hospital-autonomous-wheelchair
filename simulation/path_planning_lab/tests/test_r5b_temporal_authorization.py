from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from hospital_path_lab.dynamic_contracts import DynamicGroundTruthFrame, DynamicMotionState
from hospital_path_lab.dynamic_directional_prediction import (
    DirectionalActorPredictor,
    DirectionalPredictionStatus,
)
from hospital_path_lab.dynamic_observation import (
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    DynamicObservationSourceIdentity,
    DynamicObservationValidator,
    generate_dynamic_observation_slots,
)
from hospital_path_lab.local_reference_window import LocalReferenceWindowManager
from hospital_path_lab.persistent_controller_contracts import (
    PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
    PersistentControllerTickInput,
    build_persistent_reference_binding,
)
from hospital_path_lab.r5b_temporal_authorization import (
    R5BTemporalAuthorizationIssuer,
    R5BTemporalAuthorizationPhase,
)
from hospital_path_lab.r5b_temporal_evidence import frozen_r2_archive_path
from hospital_path_lab.r5b_temporal_reference import (
    build_r5b_temporal_reference_bundles,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def bundle():
    return build_r5b_temporal_reference_bundles(
        frozen_r2_archive_path(REPOSITORY_ROOT)
    )[0]


def _ideal_tick(bundle, target_tick: int = 40):
    world = bundle.source.world
    source = DynamicObservationSourceIdentity(
        stream_id="r5b-ideal-causal-public",
        episode_id=world.world_id,
        episode_seed=world.seed,
        map_id=world.map_id,
        map_revision=world.map_revision,
    )
    frames = tuple(
        DynamicGroundTruthFrame(
            episode_id=world.world_id,
            seed=world.seed,
            tick_id=tick,
            simulation_time_s=tick * 0.05,
            robot_state=world.initial_state,
            actors=world.actor_states_at(tick * 0.05),
            map_revision=world.map_revision,
            mission_revision=0,
        )
        for tick in range(target_tick + 1)
    )
    slots = generate_dynamic_observation_slots(
        frames,
        source=source,
        profile=FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    )
    validator = DynamicObservationValidator(
        source,
        FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    )
    predictor = DirectionalActorPredictor()
    next_slot = 0
    results = []
    for tick in range(target_tick + 1):
        time_s = tick * 0.05
        while (
            next_slot < len(slots)
            and slots[next_slot].scheduled_delivery_at_s <= time_s + 1e-12
        ):
            slot = slots[next_slot]
            assert slot.frame is not None
            assert validator.accept(
                slot.frame,
                received_at_s=slot.scheduled_delivery_at_s,
            ).accepted
            next_slot += 1
        snapshot = validator.snapshot(control_time_s=time_s)
        results.append((snapshot, predictor.update(snapshot)))
    return validator, predictor, tuple(results)


def test_ideal_causal_stream_is_not_ready_before_frozen_release(bundle) -> None:
    _, _, results = _ideal_tick(bundle)
    assert all(result.status is not DirectionalPredictionStatus.READY for _, result in results[:40])
    assert results[40][1].status is DirectionalPredictionStatus.READY


def test_issuer_rejects_early_release_and_issues_tick_bound_chain(bundle) -> None:
    validator, predictor, results = _ideal_tick(bundle)
    issuer = R5BTemporalAuthorizationIssuer()
    snapshot39, prediction39 = results[39]
    with pytest.raises(ValueError, match="causal release"):
        issuer.issue(
            reference=bundle.reference,
            temporal_evidence=bundle.temporal_evidence,
            temporal_geometry=bundle.temporal_geometry,
            observation_snapshot=snapshot39,
            prediction_result=prediction39,
            controller_tick=39,
            simulation_time_s=1.95,
            gate_motion_state=DynamicMotionState.HOLDING,
            gate_stop_epoch=bundle.reference.stop_epoch,
            resume_authorization_revision=7,
            actual_stop_confirmed=True,
            local_safety_recheck_passed=True,
        )

    snapshot40, prediction40 = results[40]
    initial = issuer.issue(
        reference=bundle.reference,
        temporal_evidence=bundle.temporal_evidence,
        temporal_geometry=bundle.temporal_geometry,
        observation_snapshot=snapshot40,
        prediction_result=prediction40,
        controller_tick=40,
        simulation_time_s=2.0,
        gate_motion_state=DynamicMotionState.HOLDING,
        gate_stop_epoch=bundle.reference.stop_epoch,
        resume_authorization_revision=7,
        actual_stop_confirmed=True,
        local_safety_recheck_passed=True,
    )
    snapshot41 = validator.snapshot(control_time_s=2.05)
    prediction41 = predictor.update(snapshot41)
    continuation = issuer.issue(
        reference=bundle.reference,
        temporal_evidence=bundle.temporal_evidence,
        temporal_geometry=bundle.temporal_geometry,
        observation_snapshot=snapshot41,
        prediction_result=prediction41,
        controller_tick=41,
        simulation_time_s=2.05,
        gate_motion_state=DynamicMotionState.MOVING,
        gate_stop_epoch=bundle.reference.stop_epoch,
        resume_authorization_revision=None,
        actual_stop_confirmed=False,
        local_safety_recheck_passed=True,
    )
    assert initial.phase is R5BTemporalAuthorizationPhase.INITIAL_RELEASE
    assert continuation.phase is R5BTemporalAuthorizationPhase.CONTINUATION
    assert continuation.prior_authorization_hash == initial.authorization_content_hash


def test_persistent_tick_input_requires_and_accepts_temporal_authorization(bundle) -> None:
    _, _, results = _ideal_tick(bundle)
    snapshot, prediction_result = results[40]
    issuer = R5BTemporalAuthorizationIssuer()
    authorization = issuer.issue(
        reference=bundle.reference,
        temporal_evidence=bundle.temporal_evidence,
        temporal_geometry=bundle.temporal_geometry,
        observation_snapshot=snapshot,
        prediction_result=prediction_result,
        controller_tick=40,
        simulation_time_s=2.0,
        gate_motion_state=DynamicMotionState.HOLDING,
        gate_stop_epoch=bundle.reference.stop_epoch,
        resume_authorization_revision=7,
        actual_stop_confirmed=True,
        local_safety_recheck_passed=True,
    )
    context = replace(
        bundle.build_context,
        current_robot_pose=bundle.source.world.initial_state.pose,
        control_tick=40,
        simulation_time_s=2.0,
        context_content_hash="",
    )
    update = LocalReferenceWindowManager().update(
        context,
        bundle.reference,
        bundle.validation,
    )
    assert update.window is not None
    binding = build_persistent_reference_binding(bundle.reference, update.window)
    common = dict(
        schema_version=PERSISTENT_CONTROLLER_INPUT_SCHEMA_VERSION,
        controller_tick=40,
        simulation_time_s=2.0,
        full_reference=bundle.reference,
        local_window=update.window,
        reference_binding=binding,
        robot_state=bundle.source.world.initial_state,
        static_grid_snapshot=context.static_grid_snapshot,
        validated_observation=snapshot,
        actor_prediction_set=prediction_result.prediction_set,
        vehicle_profile=context.vehicle_profile,
        current_gate_motion_state=DynamicMotionState.HOLDING,
        current_gate_stop_epoch=bundle.reference.stop_epoch,
        current_resume_authorization_revision=7,
    )
    with pytest.raises(ValueError, match="tick-bound authorization"):
        PersistentControllerTickInput(**common)
    tick_input = PersistentControllerTickInput(
        **common,
        temporal_execution_authorization=authorization,
    )
    assert tick_input.temporal_execution_authorization == authorization

    tampered_cases = (
        (
            replace(
                authorization,
                target_actor_binding_id="forged-actor",
                authorization_content_hash="",
            ),
            "target track changed",
        ),
        (
            replace(
                authorization,
                temporal_evidence_hash="f" * 64,
                authorization_content_hash="",
            ),
            "source evidence changed",
        ),
        (
            replace(
                authorization,
                prediction_model_version="forged-model",
                authorization_content_hash="",
            ),
            "prediction model changed",
        ),
    )
    for tampered, failure in tampered_cases:
        with pytest.raises(ValueError, match=failure):
            PersistentControllerTickInput(
                **common,
                temporal_execution_authorization=tampered,
            )


def test_authorization_rejects_not_ready_and_wrong_gate_state(bundle) -> None:
    _, _, results = _ideal_tick(bundle)
    snapshot, prediction = results[20]
    assert prediction.status is not DirectionalPredictionStatus.READY
    issuer = R5BTemporalAuthorizationIssuer()
    with pytest.raises(ValueError, match="READY"):
        issuer.issue(
            reference=bundle.reference,
            temporal_evidence=bundle.temporal_evidence,
            temporal_geometry=bundle.temporal_geometry,
            observation_snapshot=snapshot,
            prediction_result=prediction,
            controller_tick=40,
            simulation_time_s=2.0,
            gate_motion_state=DynamicMotionState.HOLDING,
            gate_stop_epoch=bundle.reference.stop_epoch,
            resume_authorization_revision=7,
            actual_stop_confirmed=True,
            local_safety_recheck_passed=True,
        )
