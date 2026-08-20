"""Explicit functional demo app; not the native R7 release configuration."""

from hospital_path_lab.dynamic_observation import DynamicObservationProfileName
from hospital_path_lab.runtime import RuntimeConfig, RuntimeControllerKind

from .app import create_app

app = create_app(
    RuntimeConfig(
        controller_kind=RuntimeControllerKind.RPP,
        observation_profile=DynamicObservationProfileName.FUNCTIONAL_IDEAL,
        require_native_dwb=False,
    )
)
