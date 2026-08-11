from __future__ import annotations

import json

import pytest

from hospital_path_lab.corpus_records import (
    load_dynamic_regression_record,
    preserve_dynamic_hidden_failure,
)
from hospital_path_lab.dynamic_corpus import (
    DynamicCorpusSplit,
    generate_dynamic_corpus,
    generate_dynamic_hidden_corpus,
    hidden_seed_commitment,
    validate_dynamic_hidden_corpus,
)


def test_hidden_commitment_balance_and_public_separation() -> None:
    public = generate_dynamic_corpus(base_seed=1000)
    commitment = hidden_seed_commitment(9000)
    hidden = generate_dynamic_hidden_corpus(
        hidden_seed=9000,
        expected_commitment=commitment,
    )
    validation = validate_dynamic_hidden_corpus(hidden, public_corpus=public)

    assert validation.passed
    assert validation.hidden_count == 30
    assert set(count for _, count in validation.category_counts) == {5}
    assert all(episode.split is DynamicCorpusSplit.HIDDEN for episode in hidden)
    assert not ({item.content_hash for item in public} & {item.content_hash for item in hidden})


def test_hidden_seed_must_match_frozen_commitment() -> None:
    with pytest.raises(ValueError, match="commitment mismatch"):
        generate_dynamic_hidden_corpus(
            hidden_seed=101,
            expected_commitment=hidden_seed_commitment(102),
        )


def test_hidden_failure_is_exclusive_and_tamper_evident(tmp_path) -> None:
    seed = 8080
    episode = generate_dynamic_hidden_corpus(
        hidden_seed=seed,
        expected_commitment=hidden_seed_commitment(seed),
    )[0]
    path = preserve_dynamic_hidden_failure(
        episode,
        observation_profile="normal",
        controller_name="dynamic_dwa",
        failing_tick=12,
        reason="clearance_violation",
        output_directory=tmp_path,
        minimal_evidence={"minimum_clearance_m": 0.07},
    )

    record = load_dynamic_regression_record(path)
    assert record.episode_content_hash == episode.content_hash
    assert record.failing_tick == 12
    with pytest.raises(FileExistsError):
        preserve_dynamic_hidden_failure(
            episode,
            observation_profile="normal",
            controller_name="dynamic_dwa",
            failing_tick=12,
            reason="clearance_violation",
            output_directory=tmp_path,
        )

    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["reason"] = "tampered"
    path.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="content hash mismatch"):
        load_dynamic_regression_record(path)
