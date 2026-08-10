"""경로 판단과 실제 이동 재개를 분리하는 연구용 안전 게이트."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum


class MotionState(StrEnum):
    MOVING = "moving"
    STOPPING = "stopping"
    STOPPED = "stopped"


@dataclass(slots=True)
class AutomaticResumeGate:
    """장애물 해소만으로는 재출발하지 않는 자동 재개 체크리스트."""

    state: MotionState = MotionState.MOVING
    hazard_active: bool = False
    stop_confirmed: bool = False
    path_revalidated: bool = False
    resume_instruction_revalidated: bool = False
    local_safety_authorized: bool = False
    events: list[str] = field(default_factory=list)

    def hazard_detected(self) -> None:
        self.hazard_active = True
        self.state = MotionState.STOPPING
        self.stop_confirmed = False
        self.path_revalidated = False
        self.resume_instruction_revalidated = False
        self.local_safety_authorized = False
        self.events.append("hazard_detected:stop_started")

    def confirm_stop(self) -> None:
        if self.state is not MotionState.STOPPING:
            raise RuntimeError("정지 동작을 시작한 뒤에만 실제 정지를 확인할 수 있습니다.")
        self.stop_confirmed = True
        self.state = MotionState.STOPPED
        self.events.append("physical_stop_confirmed")

    def hazard_cleared(self) -> None:
        self.hazard_active = False
        self.events.append("hazard_cleared:resume_not_yet_allowed")

    def record_path_revalidation(self, *, original_path_safe: bool) -> None:
        self.path_revalidated = bool(
            original_path_safe and self.stop_confirmed and not self.hazard_active
        )
        self.events.append(f"path_revalidated:{self.path_revalidated}")

    def revalidate_resume_instruction(self) -> None:
        self.resume_instruction_revalidated = True
        self.events.append("resume_instruction_revalidated")

    def authorize_local_safety(self) -> None:
        self.local_safety_authorized = True
        self.events.append("local_safety_authorized")

    def try_automatic_resume(self) -> bool:
        allowed = all(
            (
                self.state is MotionState.STOPPED,
                self.stop_confirmed,
                not self.hazard_active,
                self.path_revalidated,
                self.resume_instruction_revalidated,
                self.local_safety_authorized,
            )
        )
        if allowed:
            self.state = MotionState.MOVING
            self.events.append("automatic_resume_allowed")
        else:
            self.events.append("automatic_resume_denied")
        return allowed
