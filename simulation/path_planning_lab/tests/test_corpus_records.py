from json import dumps, loads

import pytest

from hospital_path_lab.corpus_records import (
    load_promoted_regressions,
    load_regression_record,
    preserve_hidden_failure,
    replay_regression_record,
)
from hospital_path_lab.map_factory import (
    CorpusSplit,
    canonical_content_hash,
    generate_batch,
)


def test_hidden_failure_is_preserved_as_minimal_hashed_regression(tmp_path) -> None:
    case = next(
        item
        for item in generate_batch(base_seed=777)
        if item.episode.split is CorpusSplit.HIDDEN
    )

    path = preserve_hidden_failure(
        case,
        reason="grid_astar_no_path_vs_oracle",
        failing_step=3,
        output_directory=tmp_path,
        minimal_evidence={"planner": "grid_astar"},
    )
    loaded = load_regression_record(path)

    assert loaded.world_content_hash == case.world.content_hash
    assert loaded.episode_content_hash == case.episode.content_hash
    assert loaded.world_seed == case.world.seed
    assert loaded.episode_seed == case.episode.seed
    assert loaded.reason == "grid_astar_no_path_vs_oracle"
    assert loaded.minimal_evidence["event_kind"] == "create_obstacle"
    assert loaded.target_split == CorpusSplit.REGRESSIONS.value

    replayed = replay_regression_record(loaded)
    assert replayed.world.content_hash == loaded.world_content_hash
    assert replayed.episode.split is CorpusSplit.REGRESSIONS
    assert len(replayed.episode.events) == 3
    assert replayed.episode.events[-1].kind.value == loaded.minimal_evidence["event_kind"]
    assert load_promoted_regressions(tmp_path) == (replayed,)

    with pytest.raises(FileExistsError):
        preserve_hidden_failure(
            case,
            reason="grid_astar_no_path_vs_oracle",
            failing_step=3,
            output_directory=tmp_path,
        )


def test_non_hidden_case_and_tampered_record_are_rejected(tmp_path) -> None:
    development = next(
        item
        for item in generate_batch(base_seed=888)
        if item.episode.split is CorpusSplit.DEVELOPMENT
    )
    with pytest.raises(ValueError, match="hidden split"):
        preserve_hidden_failure(
            development,
            reason="not_hidden",
            failing_step=0,
            output_directory=tmp_path,
        )

    hidden = next(
        item
        for item in generate_batch(base_seed=888)
        if item.episode.split is CorpusSplit.HIDDEN
    )
    path = preserve_hidden_failure(
        hidden,
        reason="original_reason",
        failing_step=0,
        output_directory=tmp_path,
    )
    raw = loads(path.read_text(encoding="utf-8"))
    raw["reason"] = "tampered_reason"
    tampered = tmp_path / "tampered.json"
    tampered.write_text(dumps(raw), encoding="utf-8")

    with pytest.raises(ValueError, match="content hash"):
        load_regression_record(tampered)


def test_nested_experiment_candidates_are_sorted_deduplicated_and_limited(
    tmp_path,
) -> None:
    first_case = next(
        item
        for item in generate_batch(base_seed=991)
        if item.episode.split is CorpusSplit.HIDDEN
    )
    second_case = next(
        item
        for item in generate_batch(base_seed=992)
        if item.episode.split is CorpusSplit.HIDDEN
    )

    first_path = preserve_hidden_failure(
        first_case,
        reason="first_failure",
        failing_step=0,
        output_directory=tmp_path / "a_run" / "regression_candidates" / "batch_02",
    )
    preserve_hidden_failure(
        second_case,
        reason="second_failure",
        failing_step=0,
        output_directory=tmp_path / "z_run" / "regression_candidates" / "batch_01",
    )
    duplicate = (
        tmp_path
        / "m_run"
        / "regression_candidates"
        / "duplicates"
        / first_path.name
    )
    duplicate.parent.mkdir(parents=True)
    duplicate.write_bytes(first_path.read_bytes())
    preserve_hidden_failure(
        first_case,
        reason="later_failure_same_world",
        failing_step=2,
        output_directory=tmp_path
        / "n_run"
        / "regression_candidates"
        / "later_step",
    )

    ignored = tmp_path / "experiment_results.json"
    ignored.write_text('{"not": "a regression record"}', encoding="utf-8")

    promoted = load_promoted_regressions(tmp_path)
    assert [item.world.content_hash for item in promoted] == [
        first_case.world.content_hash,
        second_case.world.content_hash,
    ]
    assert max(event.step for event in promoted[0].episode.events) == 2
    assert len({item.world.content_hash for item in promoted}) == len(promoted)
    assert load_promoted_regressions(tmp_path, limit=1) == (promoted[0],)
    assert load_promoted_regressions(tmp_path, limit=0) == ()

    with pytest.raises(ValueError, match="limit"):
        load_promoted_regressions(tmp_path, limit=-1)
    with pytest.raises(ValueError, match="limit"):
        load_promoted_regressions(tmp_path, limit=True)
    with pytest.raises(ValueError, match="limit"):
        load_promoted_regressions(tmp_path, limit=1.5)


def test_recursive_loader_rejects_tamper_and_non_hidden_provenance(tmp_path) -> None:
    hidden = next(
        item
        for item in generate_batch(base_seed=993)
        if item.episode.split is CorpusSplit.HIDDEN
    )
    path = preserve_hidden_failure(
        hidden,
        reason="original_reason",
        failing_step=0,
        output_directory=tmp_path / "run" / "regression_candidates" / "batch",
    )

    raw = loads(path.read_text(encoding="utf-8"))
    raw["reason"] = "tampered_reason"
    path.write_text(dumps(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="content hash"):
        load_promoted_regressions(tmp_path, limit=0)

    raw["source_split"] = CorpusSplit.DEVELOPMENT.value
    raw_without_hash = {
        key: value for key, value in raw.items() if key != "record_content_hash"
    }
    raw["record_content_hash"] = canonical_content_hash(raw_without_hash)
    path.write_text(dumps(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="provenance"):
        load_promoted_regressions(tmp_path)
