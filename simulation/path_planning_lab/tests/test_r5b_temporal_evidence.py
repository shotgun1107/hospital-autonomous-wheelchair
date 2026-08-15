from __future__ import annotations

import shutil
from dataclasses import replace
from pathlib import Path

import pytest

from hospital_path_lab.dynamic_witness_contracts import PassSide, WitnessKind
from hospital_path_lab.r5b_temporal_evidence import (
    R5B_CAUSAL_RELEASE_TICK,
    R5B_CONTROLLER_COMPLETION_BUFFER_M,
    R5B_CONTROLLER_MATCHED_LINEAR_TARGET_MPS,
    R5B_CONTROLLER_MINIMUM_LATERAL_OFFSET_M,
    R5B_EXPECTED_PASS_EVIDENCE_COUNT,
    R5B_R2_ARCHIVE_SHA256,
    build_causal_r5b_pass_evidence,
    frozen_r2_archive_path,
    load_frozen_r2_pass_evidence,
)

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def frozen_evidence():
    load_frozen_r2_pass_evidence.cache_clear()
    return load_frozen_r2_pass_evidence(frozen_r2_archive_path(REPOSITORY_ROOT))


def test_frozen_r2_archive_restores_ten_strict_pass_records(frozen_evidence) -> None:
    assert len(frozen_evidence) == R5B_EXPECTED_PASS_EVIDENCE_COUNT
    assert tuple(item.corpus_ordinal for item in frozen_evidence) == tuple(
        ordinal for ordinal in range(5) for _ in range(2)
    )
    assert tuple(item.side for item in frozen_evidence) == (
        PassSide.LEFT,
        PassSide.RIGHT,
    ) * 5
    assert len({item.evidence_content_hash for item in frozen_evidence}) == 10
    assert all(item.validation.passed for item in frozen_evidence)
    assert all(item.archive_sha256 == R5B_R2_ARCHIVE_SHA256 for item in frozen_evidence)
    assert {item.archived_validator_version for item in frozen_evidence} == {
        "ground-truth-witness-validator-v2"
    }
    assert {item.validation.validator_version for item in frozen_evidence} == {
        "ground-truth-witness-validator-v3"
    }


def test_frozen_r2_pass_records_keep_actor_and_time_order(frozen_evidence) -> None:
    for item in frozen_evidence:
        expected_kind = (
            WitnessKind.PASS_LEFT if item.side is PassSide.LEFT else WitnessKind.PASS_RIGHT
        )
        witness = item.witness
        assert witness.kind is expected_kind
        assert len(witness.required_pass_actor_ids) == 1
        assert witness.departure_time_s is not None
        assert witness.rejoin_confirmed_at_s is not None
        assert witness.departure_time_s < witness.pass_times_by_actor[0][1]
        assert witness.pass_times_by_actor[0][1] < witness.rejoin_confirmed_at_s
        actor = item.world.actors[0]
        assert actor.active_from_s == 0.0
        assert actor.active_until_s >= witness.rejoin_confirmed_at_s
        # 20 Ideal 10 Hz frames plus 100 ms latency are available before departure.
        assert witness.departure_time_s >= 2.0


def test_frozen_evidence_hash_rejects_provenance_tampering(frozen_evidence) -> None:
    item = frozen_evidence[0]
    with pytest.raises(ValueError, match="content hash mismatch"):
        replace(item, evidence_content_hash="0" * 64)


def test_archive_digest_mismatch_fails_closed(tmp_path: Path) -> None:
    source = frozen_r2_archive_path(REPOSITORY_ROOT)
    changed = tmp_path / source.name
    shutil.copyfile(source, changed)
    with changed.open("r+b") as stream:
        stream.seek(128)
        original = stream.read(1)
        stream.seek(128)
        stream.write(bytes((original[0] ^ 0x01,)))
    load_frozen_r2_pass_evidence.cache_clear()
    with pytest.raises(ValueError, match="SHA-256 mismatch"):
        load_frozen_r2_pass_evidence(changed)


def test_causal_r5b_search_finds_strict_left_and_right_passes() -> None:
    build_causal_r5b_pass_evidence.cache_clear()
    evidence = build_causal_r5b_pass_evidence(frozen_r2_archive_path(REPOSITORY_ROOT))
    assert len(evidence) == 10
    assert tuple(item.side for item in evidence) == (PassSide.LEFT, PassSide.RIGHT) * 5
    assert all(item.release_tick == R5B_CAUSAL_RELEASE_TICK for item in evidence)
    assert all(item.validation.passed for item in evidence)
    assert all(item.validation.metrics.minimum_actor_clearance_m >= 0.08 for item in evidence)
    assert tuple(item.selected_lateral_offset_m for item in evidence) == (
        1.11,
        1.11,
        1.02,
        1.02,
        1.76,
        1.76,
        1.14,
        1.14,
        1.00,
        1.00,
    )


def test_causal_release_api_does_not_change_frozen_r2_witnesses(
    frozen_evidence,
) -> None:
    causal = build_causal_r5b_pass_evidence(frozen_r2_archive_path(REPOSITORY_ROOT))
    assert all(
        source.witness.semantic_content_hash != derived.witness.semantic_content_hash
        for source, derived in zip(frozen_evidence, causal, strict=True)
    )
    assert all(derived.witness.points[40].time_s == 2.0 for derived in causal)
    assert all(
        derived.witness.points[40].twist == derived.witness.points[0].twist
        for derived in causal
    )


def test_controller_matched_causal_passes_finish_overtake_before_actor_disappears() -> None:
    evidence = build_causal_r5b_pass_evidence(
        frozen_r2_archive_path(REPOSITORY_ROOT),
        linear_target_mps=R5B_CONTROLLER_MATCHED_LINEAR_TARGET_MPS,
        longitudinal_pass_buffer_m=R5B_CONTROLLER_COMPLETION_BUFFER_M,
        minimum_lateral_offset_m=R5B_CONTROLLER_MINIMUM_LATERAL_OFFSET_M,
    )
    assert len(evidence) == 10
    for item in evidence:
        actor = item.world.actors[0]
        pass_time_s = item.witness.pass_times_by_actor[0][1]
        rejoin_time_s = item.witness.rejoin_confirmed_at_s
        assert rejoin_time_s is not None
        assert pass_time_s < actor.active_until_s
        assert rejoin_time_s > actor.active_until_s
        assert item.selected_lateral_offset_m in (0.65, 0.66)
