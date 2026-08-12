from __future__ import annotations

import pytest

from hospital_path_lab.cpp_dwa_core import CPP_DWA_CORE_AVAILABLE
from hospital_path_lab.dynamic_corpus import (
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_runner import _qualification_snapshot_cases
from hospital_path_lab.local_algorithms.dwa import (
    DynamicDwaController,
    dynamic_dwa_controller_semantic_digest,
)

pytestmark = pytest.mark.skipif(
    not CPP_DWA_CORE_AVAILABLE,
    reason="optional C++ DWA core has not been built",
)


def test_cpp_core_matches_python_on_all_frozen_qualification_snapshots() -> None:
    corpus = (*generate_dynamic_corpus(), *generate_dynamic_v6_public_corpus())

    for case_id, snapshot, _metadata in _qualification_snapshot_cases(corpus):
        python_controller = DynamicDwaController(use_cpp_core=False)
        cpp_controller = DynamicDwaController(use_cpp_core=True)

        python_result = python_controller.step(snapshot)
        cpp_result = cpp_controller.step(snapshot)

        assert cpp_controller.last_workspace_metrics.native_core_used is True, case_id
        assert dynamic_dwa_controller_semantic_digest(cpp_result) == (
            dynamic_dwa_controller_semantic_digest(python_result)
        ), case_id
        assert python_controller.last_diagnostics is not None
        assert cpp_controller.last_diagnostics is not None
        assert cpp_controller.last_diagnostics.semantic_digest == (
            python_controller.last_diagnostics.semantic_digest
        ), case_id


def test_cpp_core_preserves_frozen_candidate_and_pose_counts() -> None:
    corpus = (*generate_dynamic_corpus(), *generate_dynamic_v6_public_corpus())
    _case_id, snapshot, _metadata = _qualification_snapshot_cases(corpus)[0]
    controller = DynamicDwaController(use_cpp_core=True)

    result = controller.step(snapshot)

    assert controller.last_diagnostics is not None
    assert controller.last_diagnostics.sampled_candidates == 217
    assert "pose_samples=41" in result.decision_trace
    assert controller.last_workspace_metrics.native_core_used is True


def test_explicit_python_fallback_does_not_use_native_core() -> None:
    corpus = (*generate_dynamic_corpus(), *generate_dynamic_v6_public_corpus())
    _case_id, snapshot, _metadata = _qualification_snapshot_cases(corpus)[0]
    controller = DynamicDwaController(use_cpp_core=False)

    controller.step(snapshot)

    assert controller.last_workspace_metrics.native_core_used is False
