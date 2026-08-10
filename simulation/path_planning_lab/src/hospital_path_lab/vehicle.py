"""실물 결정과 분리된 연구용 가상 차체 프로필."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class VehicleProfile:
    profile_id: str
    simulation_only: bool
    body_width_m: float
    body_length_m: float
    collision_width_m: float
    collision_length_m: float
    differential_drive: bool
    in_place_rotation: bool
    max_forward_speed_mps: float
    nominal_speed_mps: float
    max_reverse_speed_mps: float
    max_angular_speed_radps: float
    max_acceleration_mps2: float
    max_deceleration_mps2: float
    control_frequency_hz: float
    minimum_clearance_m: float
    stopping_margin_m: float

    @property
    def control_period_s(self) -> float:
        return 1.0 / self.control_frequency_hz


VIRTUAL_DOLL_WHEELCHAIR_V0_1 = VehicleProfile(
    profile_id="virtual_doll_wheelchair_v0_1",
    simulation_only=True,
    body_width_m=0.32,
    body_length_m=0.40,
    collision_width_m=0.36,
    collision_length_m=0.44,
    differential_drive=True,
    in_place_rotation=True,
    max_forward_speed_mps=0.30,
    nominal_speed_mps=0.20,
    max_reverse_speed_mps=0.10,
    max_angular_speed_radps=0.80,
    max_acceleration_mps2=0.25,
    max_deceleration_mps2=0.50,
    control_frequency_hz=20.0,
    minimum_clearance_m=0.08,
    stopping_margin_m=0.15,
)
