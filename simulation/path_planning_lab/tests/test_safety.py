from hospital_path_lab.safety import AutomaticResumeGate, MotionState


def test_hazard_clear_alone_does_not_resume() -> None:
    gate = AutomaticResumeGate()
    gate.hazard_detected()
    gate.confirm_stop()
    gate.hazard_cleared()
    assert gate.try_automatic_resume() is False
    assert gate.state is MotionState.STOPPED


def test_automatic_resume_requires_every_gate() -> None:
    gate = AutomaticResumeGate()
    gate.hazard_detected()
    gate.confirm_stop()
    gate.hazard_cleared()
    gate.record_path_revalidation(original_path_safe=True)
    gate.revalidate_resume_instruction()
    gate.authorize_local_safety()
    assert gate.try_automatic_resume() is True
    assert gate.state is MotionState.MOVING


def test_unsafe_original_path_keeps_stop() -> None:
    gate = AutomaticResumeGate()
    gate.hazard_detected()
    gate.confirm_stop()
    gate.hazard_cleared()
    gate.record_path_revalidation(original_path_safe=False)
    gate.revalidate_resume_instruction()
    gate.authorize_local_safety()
    assert gate.try_automatic_resume() is False
    assert gate.state is MotionState.STOPPED
