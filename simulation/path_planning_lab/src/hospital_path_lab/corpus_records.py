"""숨김 평가 실패를 덮어쓰기 없이 회귀 입력으로 보존한다."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from json import dumps, loads
from pathlib import Path
from typing import Any

from hospital_path_lab.map_factory import (
    GENERATOR_VERSION,
    SCHEMA_VERSION,
    CorpusSplit,
    GeneratedCase,
    canonical_content_hash,
    generate_episode,
    generate_world,
    validate_episode,
)


@dataclass(frozen=True, slots=True)
class DynamicRegressionRecord:
    schema_version: str
    generator_version: str
    record_id: str
    source_split: str
    episode_id: str
    episode_content_hash: str
    episode_seed: int
    expectation_category: str
    observation_profile: str
    controller_name: str
    failing_tick: int
    reason: str
    minimal_evidence: dict[str, Any]
    record_content_hash: str


@dataclass(frozen=True, slots=True)
class RegressionRecord:
    schema_version: str
    generator_version: str
    record_id: str
    source_split: str
    target_split: str
    world_id: str
    world_content_hash: str
    episode_id: str
    episode_content_hash: str
    world_seed: int
    episode_seed: int
    world_family: str
    failing_step: int
    reason: str
    minimal_evidence: dict[str, Any]
    record_content_hash: str


def preserve_dynamic_hidden_failure(
    episode: object,
    *,
    observation_profile: str,
    controller_name: str,
    failing_tick: int,
    reason: str,
    output_directory: str | Path,
    minimal_evidence: dict[str, Any] | None = None,
) -> Path:
    """동적 hidden 실패를 기존 파일을 덮어쓰지 않고 보존한다."""

    from hospital_path_lab.dynamic_corpus import (
        DYNAMIC_CORPUS_GENERATOR_VERSION,
        DYNAMIC_CORPUS_SCHEMA_VERSION,
        DynamicCorpusEpisode,
        DynamicCorpusSplit,
    )

    if not isinstance(episode, DynamicCorpusEpisode):
        raise TypeError("dynamic regression source must be a DynamicCorpusEpisode")
    if episode.split is not DynamicCorpusSplit.HIDDEN:
        raise ValueError("dynamic regression source must be hidden")
    if not observation_profile or not controller_name or not reason.strip():
        raise ValueError("dynamic regression identity and reason must not be empty")
    if failing_tick < 0 or failing_tick > episode.tick_count:
        raise ValueError("dynamic failing_tick is outside the episode")
    payload: dict[str, Any] = {
        "schema_version": DYNAMIC_CORPUS_SCHEMA_VERSION,
        "generator_version": DYNAMIC_CORPUS_GENERATOR_VERSION,
        "record_id": (
            f"dynamic_{episode.content_hash[:12]}_{observation_profile}_"
            f"{controller_name}_tick_{failing_tick:04d}"
        ),
        "source_split": DynamicCorpusSplit.HIDDEN.value,
        "episode_id": episode.episode_id,
        "episode_content_hash": episode.content_hash,
        "episode_seed": episode.seed,
        "expectation_category": episode.expectation_category.value,
        "observation_profile": observation_profile,
        "controller_name": controller_name,
        "failing_tick": failing_tick,
        "reason": reason,
        "minimal_evidence": dict(minimal_evidence or {}),
    }
    payload["record_content_hash"] = canonical_content_hash(payload)
    record = DynamicRegressionRecord(**payload)
    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    path = destination / f"{record.record_id}.json"
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(
            dumps(
                asdict(record),
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
        )
        stream.write("\n")
    return path


def load_dynamic_regression_record(path: str | Path) -> DynamicRegressionRecord:
    from hospital_path_lab.dynamic_corpus import (
        DYNAMIC_CORPUS_GENERATOR_VERSION,
        DYNAMIC_CORPUS_SCHEMA_VERSION,
        DynamicCorpusSplit,
    )

    raw = loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("dynamic regression record must be an object")
    recorded_hash = raw.pop("record_content_hash", None)
    if not isinstance(recorded_hash, str) or canonical_content_hash(raw) != recorded_hash:
        raise ValueError("dynamic regression record content hash mismatch")
    raw["record_content_hash"] = recorded_hash
    record = DynamicRegressionRecord(**raw)
    if (
        record.schema_version != DYNAMIC_CORPUS_SCHEMA_VERSION
        or record.generator_version != DYNAMIC_CORPUS_GENERATOR_VERSION
        or record.source_split != DynamicCorpusSplit.HIDDEN.value
    ):
        raise ValueError("dynamic regression provenance is invalid")
    return record


def preserve_hidden_failure(
    case: GeneratedCase,
    *,
    reason: str,
    failing_step: int,
    output_directory: str | Path,
    minimal_evidence: dict[str, Any] | None = None,
) -> Path:
    """hidden 실패의 최소 재생 정보를 새 JSON 파일로만 기록한다.

    같은 실패 파일이 이미 있으면 ``FileExistsError``를 내며 원본을 절대 덮어쓰지 않는다.
    """

    validate_episode(case.world, case.episode)
    if case.episode.split is not CorpusSplit.HIDDEN:
        raise ValueError("회귀 기록의 source episode는 hidden split이어야 합니다.")
    if not reason.strip():
        raise ValueError("실패 reason은 비어 있을 수 없습니다.")
    max_step = max((event.step for event in case.episode.events), default=0)
    if failing_step < 0 or failing_step > max_step:
        raise ValueError("failing_step이 episode 범위를 벗어났습니다.")

    evidence = dict(minimal_evidence or {})
    event = next((item for item in case.episode.events if item.step == failing_step), None)
    if event is not None:
        evidence.setdefault("event_kind", event.kind.value)
        evidence.setdefault(
            "input_revisions",
            {
                "map": event.map_revision,
                "mission": event.mission_revision,
                "observation": event.observation_revision,
            },
        )
    payload: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "generator_version": GENERATOR_VERSION,
        "record_id": (
            f"regression_{case.world.content_hash[:12]}_"
            f"{case.episode.content_hash[:12]}_step_{failing_step:03d}"
        ),
        "source_split": CorpusSplit.HIDDEN.value,
        "target_split": CorpusSplit.REGRESSIONS.value,
        "world_id": case.world.world_id,
        "world_content_hash": case.world.content_hash,
        "episode_id": case.episode.episode_id,
        "episode_content_hash": case.episode.content_hash,
        "world_seed": case.world.seed,
        "episode_seed": case.episode.seed,
        "world_family": case.world.family.value,
        "failing_step": failing_step,
        "reason": reason,
        "minimal_evidence": evidence,
    }
    payload["record_content_hash"] = canonical_content_hash(payload)
    record = RegressionRecord(**payload)

    destination = Path(output_directory)
    destination.mkdir(parents=True, exist_ok=True)
    path = destination / f"{record.record_id}.json"
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(
            dumps(
                asdict(record),
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
        )
        stream.write("\n")
    return path


def load_regression_record(path: str | Path) -> RegressionRecord:
    """보존 record를 읽고 자체 hash가 맞지 않으면 거부한다."""

    raw = loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("regression record 최상위 값은 object여야 합니다.")
    recorded_hash = raw.pop("record_content_hash", None)
    if not isinstance(recorded_hash, str) or canonical_content_hash(raw) != recorded_hash:
        raise ValueError("regression record content hash가 일치하지 않습니다.")
    raw["record_content_hash"] = recorded_hash
    record = RegressionRecord(**raw)
    if (
        record.schema_version != SCHEMA_VERSION
        or record.generator_version != GENERATOR_VERSION
        or record.source_split != CorpusSplit.HIDDEN.value
        or record.target_split != CorpusSplit.REGRESSIONS.value
    ):
        raise ValueError("regression record provenance가 유효하지 않습니다.")
    return record


def replay_regression_record(record: RegressionRecord) -> GeneratedCase:
    """기록의 원본 hash를 검증한 뒤 failing step prefix를 regression으로 승격한다."""

    world = generate_world(record.world_seed, record.world_family)
    source_episode = generate_episode(
        world,
        seed=record.episode_seed,
        split=CorpusSplit.HIDDEN,
    )
    if (
        world.world_id != record.world_id
        or world.content_hash != record.world_content_hash
        or source_episode.episode_id != record.episode_id
        or source_episode.content_hash != record.episode_content_hash
    ):
        raise ValueError("regression source world/episode hash가 기록과 일치하지 않습니다.")

    event_prefix = tuple(
        event for event in source_episode.events if event.step <= record.failing_step
    )
    promoted_episode = replace(
        source_episode,
        episode_id=f"{record.record_id}_promoted",
        split=CorpusSplit.REGRESSIONS,
        events=event_prefix,
    )
    validate_episode(world, promoted_episode)
    return GeneratedCase(world=world, episode=promoted_episode)


def load_promoted_regressions(
    directory: str | Path,
    *,
    limit: int | None = None,
) -> tuple[GeneratedCase, ...]:
    """검증된 회귀 레코드를 결정적인 파일 경로 순서로 재생한다.

    과거 실험 출력 루트를 넘길 수 있도록 모든 중첩 ``regression_candidates``
    디렉터리를 탐색한다. 후보 디렉터리가 없을 때만 기존처럼 ``directory``
    바로 아래의 JSON을 읽는다.
    동일한 원본 world/episode의 여러 실패는 가장 뒤의 failing step prefix 하나로
    합친다. runner가 prefix의 모든 이전 step도 재실행하므로 앞선 실패를 함께
    재현하면서 public corpus의 world hash 중복을 피한다.
    ``limit``은 발견한 모든 레코드의 무결성과 provenance를 검증한 뒤 적용한다.
    """

    if limit is not None and (
        isinstance(limit, bool) or not isinstance(limit, int) or limit < 0
    ):
        raise ValueError("limit은 0 이상의 정수 또는 None이어야 합니다.")

    root = Path(directory)
    if not root.exists():
        return ()
    if not root.is_dir():
        raise ValueError("regression record 경로는 디렉터리여야 합니다.")

    selected: dict[
        tuple[str, str],
        tuple[int, RegressionRecord, GeneratedCase],
    ] = {}
    for path_index, path in enumerate(_discover_regression_record_paths(root)):
        record = load_regression_record(path)
        replayed = replay_regression_record(record)
        source_identity = (
            record.world_content_hash,
            record.episode_content_hash,
        )
        previous = selected.get(source_identity)
        if previous is None or record.failing_step > previous[1].failing_step:
            first_index = path_index if previous is None else previous[0]
            selected[source_identity] = (first_index, record, replayed)

    promoted = [
        replayed
        for _, _, replayed in sorted(selected.values(), key=lambda item: item[0])
    ]

    if limit is None:
        return tuple(promoted)
    return tuple(promoted[:limit])


def _discover_regression_record_paths(root: Path) -> tuple[Path, ...]:
    """기존 flat 레코드와 중첩 실험 출력의 후보 레코드만 찾는다."""

    candidate_directories = (
        (root,)
        if root.name == "regression_candidates"
        else tuple(
            path
            for path in root.rglob("regression_candidates")
            if path.is_dir()
        )
    )
    paths = {
        path
        for candidate_directory in candidate_directories
        for path in candidate_directory.rglob("*.json")
    }
    if not candidate_directories:
        paths.update(root.glob("*.json"))

    def relative_sort_key(path: Path) -> tuple[str, str]:
        relative = path.relative_to(root).as_posix()
        return relative.casefold(), relative

    return tuple(sorted(paths, key=relative_sort_key))
