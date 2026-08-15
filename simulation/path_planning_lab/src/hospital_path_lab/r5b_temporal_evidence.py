"""Frozen R2-A evidence intake for the R5-B path-only temporal lane.

This module never runs the structured witness search.  It verifies and restores the
tracked R2 archive, then re-runs the independent ground-truth validator using the
current immutable contracts.  Camera/perception claims remain outside this lane.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from functools import lru_cache
from hashlib import sha256
from pathlib import Path, PurePosixPath
from typing import Any
from zipfile import ZipFile, ZipInfo

from hospital_path_lab.contracts import Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import Point2D, Vector2D
from hospital_path_lab.dynamic_witness_contracts import (
    AutomatedWitness,
    ManeuverConstraintSpec,
    PassingPolicy,
    PassSide,
    PassSideWaitPolicy,
    WitnessActorTrajectory,
    WitnessGridSpec,
    WitnessKind,
    WitnessKinematicContract,
    WitnessPhase,
    WitnessPoint,
    WitnessTerminalMode,
    WitnessWorldSnapshot,
)
from hospital_path_lab.dynamic_witness_pass import (
    PassCandidateRequest,
    generate_causal_release_pass_candidate,
    pass_candidate_lateral_offsets,
)
from hospital_path_lab.dynamic_witness_validation import (
    GroundTruthWitnessValidation,
    validate_ground_truth_witness,
)
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.vehicle import VehicleProfile

R5B_TEMPORAL_EVIDENCE_SCHEMA_VERSION = "r5b-frozen-r2-pass-evidence-v1"
R5B_R2_ARCHIVE_RELATIVE_PATH = Path(
    "simulation/path_planning_lab/outputs/"
    "witness-audit-public-20260813-r2-v2-4e4ba0f.zip"
)
R5B_R2_ARCHIVE_SIZE_BYTES = 3_657_108
R5B_R2_ARCHIVE_SHA256 = (
    "50567b093082a57232e668ef89c0316a426cd936496e465b943fe57efa894266"
)
R5B_R2_ARCHIVE_ROOT = "witness-audit-public-20260813-r2-v2-4e4ba0f"
R5B_R2_SOURCE_AUDIT_COMMIT = "4e4ba0fb91d67498fe163aca99ff1ab647224f08"
R5B_EXPECTED_EPISODE_COUNT = 5
R5B_EXPECTED_PASS_EVIDENCE_COUNT = 10
R5B_CAUSAL_RELEASE_TICK = 40
R5B_CAUSAL_RELEASE_TIME_S = 2.0
R5B_CAUSAL_DEPARTURE_PROGRESS_M = 0.0
R5B_CAUSAL_LINEAR_TARGET_MPS = 0.30
R5B_CAUSAL_ANGULAR_MAGNITUDE_RADPS = 0.80
_EXPECTED_VARIANT_PREFIXES = tuple(f"v6_primary-{index:02d}-" for index in range(5))


@dataclass(frozen=True, slots=True)
class FrozenR2PassEvidence:
    schema_version: str
    public_id: str
    corpus_ordinal: int
    side: PassSide
    archive_sha256: str
    source_audit_commit: str
    world: WitnessWorldSnapshot
    witness: AutomatedWitness
    archived_validator_version: str
    archived_validation_hash: str
    validation: GroundTruthWitnessValidation
    evidence_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != R5B_TEMPORAL_EVIDENCE_SCHEMA_VERSION:
            raise ValueError("unsupported R5-B frozen evidence schema")
        if not 0 <= self.corpus_ordinal < R5B_EXPECTED_EPISODE_COUNT:
            raise ValueError("R5-B frozen corpus ordinal is outside the selected scope")
        if not self.public_id.startswith(_EXPECTED_VARIANT_PREFIXES[self.corpus_ordinal]):
            raise ValueError("R5-B public id does not match its frozen ordinal")
        expected_kind = (
            WitnessKind.PASS_LEFT if self.side is PassSide.LEFT else WitnessKind.PASS_RIGHT
        )
        if self.witness.kind is not expected_kind:
            raise ValueError("R5-B witness kind does not match its pass side")
        if self.archive_sha256 != R5B_R2_ARCHIVE_SHA256:
            raise ValueError("R5-B archive digest is not the frozen R2 digest")
        if self.source_audit_commit != R5B_R2_SOURCE_AUDIT_COMMIT:
            raise ValueError("R5-B source audit commit is not frozen")
        if not self.archived_validator_version:
            raise ValueError("R5-B archived validator version must not be empty")
        if len(self.archived_validation_hash) != 64:
            raise ValueError("R5-B archived validation hash must be SHA-256")
        if self.witness.world_content_hash != self.world.content_hash:
            raise ValueError("R5-B witness is not bound to the restored world")
        if self.validation.world_content_hash != self.world.content_hash:
            raise ValueError("R5-B validation is not bound to the restored world")
        if self.validation.witness_content_hash != self.witness.semantic_content_hash:
            raise ValueError("R5-B validation is not bound to the restored witness")
        if not self.validation.passed:
            raise ValueError("R5-B only accepts strictly validated R2 PASS evidence")
        expected_hash = self.expected_content_hash
        if self.evidence_content_hash and self.evidence_content_hash != expected_hash:
            raise ValueError("R5-B frozen evidence content hash mismatch")
        object.__setattr__(self, "evidence_content_hash", expected_hash)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "public_id": self.public_id,
                "corpus_ordinal": self.corpus_ordinal,
                "side": self.side,
                "archive_sha256": self.archive_sha256,
                "source_audit_commit": self.source_audit_commit,
                "world_content_hash": self.world.content_hash,
                "witness_content_hash": self.witness.semantic_content_hash,
                "archived_validator_version": self.archived_validator_version,
                "archived_validation_hash": self.archived_validation_hash,
                "current_validation_content_hash": self.validation.content_hash,
            }
        )


@dataclass(frozen=True, slots=True)
class CausalR5BPassEvidence:
    schema_version: str
    source_frozen_evidence_hash: str
    public_id: str
    corpus_ordinal: int
    side: PassSide
    release_tick: int
    rejected_lateral_offset_count: int
    selected_lateral_offset_m: float
    world: WitnessWorldSnapshot
    witness: AutomatedWitness
    validation: GroundTruthWitnessValidation
    evidence_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != R5B_TEMPORAL_EVIDENCE_SCHEMA_VERSION:
            raise ValueError("unsupported R5-B causal evidence schema")
        if self.release_tick != R5B_CAUSAL_RELEASE_TICK:
            raise ValueError("R5-B causal release tick is not frozen")
        if self.rejected_lateral_offset_count < 0:
            raise ValueError("R5-B rejected offset count must not be negative")
        if self.selected_lateral_offset_m <= 0.0:
            raise ValueError("R5-B selected lateral offset must be positive")
        expected_kind = (
            WitnessKind.PASS_LEFT if self.side is PassSide.LEFT else WitnessKind.PASS_RIGHT
        )
        if self.witness.kind is not expected_kind or not self.validation.passed:
            raise ValueError("R5-B causal evidence requires a passing side-matched witness")
        if self.witness.world_content_hash != self.world.content_hash:
            raise ValueError("R5-B causal witness is not bound to its frozen world")
        if self.validation.world_content_hash != self.world.content_hash:
            raise ValueError("R5-B causal validation is not bound to its frozen world")
        if self.validation.witness_content_hash != self.witness.semantic_content_hash:
            raise ValueError("R5-B causal validation is not bound to its witness")
        expected_hash = self.expected_content_hash
        if self.evidence_content_hash and self.evidence_content_hash != expected_hash:
            raise ValueError("R5-B causal evidence content hash mismatch")
        object.__setattr__(self, "evidence_content_hash", expected_hash)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "source_frozen_evidence_hash": self.source_frozen_evidence_hash,
                "public_id": self.public_id,
                "corpus_ordinal": self.corpus_ordinal,
                "side": self.side,
                "release_tick": self.release_tick,
                "rejected_lateral_offset_count": self.rejected_lateral_offset_count,
                "selected_lateral_offset_m": self.selected_lateral_offset_m,
                "world_content_hash": self.world.content_hash,
                "witness_content_hash": self.witness.semantic_content_hash,
                "validation_content_hash": self.validation.content_hash,
            }
        )


def frozen_r2_archive_path(repository_root: Path) -> Path:
    return Path(repository_root) / R5B_R2_ARCHIVE_RELATIVE_PATH


@lru_cache(maxsize=4)
def load_frozen_r2_pass_evidence(
    archive_path: Path,
) -> tuple[FrozenR2PassEvidence, ...]:
    """Verify one tracked R2 archive and restore the ten same-direction PASS records."""

    path = Path(archive_path).resolve()
    _verify_archive_file(path)
    with ZipFile(path) as archive:
        names = _validated_archive_names(archive.infolist())
        episode_dirs = _selected_episode_directories(names)
        restored: list[FrozenR2PassEvidence] = []
        for ordinal, directory in enumerate(episode_dirs):
            diagnostics = _read_json_member(archive, f"{directory}/search_diagnostics.json")
            selected = _read_json_member(archive, f"{directory}/selected_witness.json")
            validations = _read_json_member(
                archive,
                f"{directory}/ground_truth_validation.json",
            )
            public_id = _required_string(selected, "public_id")
            if public_id != directory.rsplit("/", 1)[-1]:
                raise ValueError("R5-B archive public id and directory disagree")
            if (
                diagnostics.get("public_id") != public_id
                or validations.get("public_id") != public_id
            ):
                raise ValueError("R5-B archive record public ids disagree")
            world_payload = _required_mapping(diagnostics, "world")
            world = _restore_world(world_payload)
            if canonical_content_hash(world_payload) != world.content_hash:
                raise ValueError("R5-B restored world differs from the archived world")
            selected_records = _record_by_witness_hash(selected)
            validation_records = _validation_by_witness_hash(validations)
            for side in (PassSide.LEFT, PassSide.RIGHT):
                role = f"pass_{side.value}_search_selected"
                witness_payload = _single_role_witness(selected_records, role)
                witness = _restore_witness(witness_payload)
                archived_witness_hash = canonical_content_hash(witness_payload)
                if archived_witness_hash != witness.semantic_content_hash:
                    raise ValueError("R5-B restored witness differs from the archived witness")
                validation_payload = validation_records.get(witness.semantic_content_hash)
                if validation_payload is None:
                    raise ValueError("R5-B archive is missing the selected witness validation")
                _validate_archived_validation_payload(
                    validation_payload,
                    world=world,
                    witness=witness,
                )
                strict_validation = validate_ground_truth_witness(
                    world,
                    witness,
                    strict_declarations=True,
                )
                archived_validation_hash = canonical_content_hash(validation_payload)
                archived_validator_version = _required_string(
                    validation_payload,
                    "validator_version",
                )
                if (
                    archived_validator_version == strict_validation.validator_version
                    and archived_validation_hash != strict_validation.content_hash
                ):
                    raise ValueError("same-version strict validation changed from the archive")
                if not strict_validation.passed:
                    raise ValueError("current strict validator rejects the frozen R2 witness")
                restored.append(
                    FrozenR2PassEvidence(
                        schema_version=R5B_TEMPORAL_EVIDENCE_SCHEMA_VERSION,
                        public_id=public_id,
                        corpus_ordinal=ordinal,
                        side=side,
                        archive_sha256=R5B_R2_ARCHIVE_SHA256,
                        source_audit_commit=R5B_R2_SOURCE_AUDIT_COMMIT,
                        world=world,
                        witness=witness,
                        archived_validator_version=archived_validator_version,
                        archived_validation_hash=archived_validation_hash,
                        validation=strict_validation,
                    )
                )
    result = tuple(restored)
    if len(result) != R5B_EXPECTED_PASS_EVIDENCE_COUNT:
        raise ValueError("R5-B archive did not produce the frozen ten PASS records")
    if len({item.evidence_content_hash for item in result}) != len(result):
        raise ValueError("R5-B frozen PASS evidence hashes must be unique")
    return result


@lru_cache(maxsize=2)
def build_causal_r5b_pass_evidence(
    archive_path: Path,
) -> tuple[CausalR5BPassEvidence, ...]:
    """Find the first strict PASS per frozen side after the Ideal warm-up hold."""

    result: list[CausalR5BPassEvidence] = []
    for source in load_frozen_r2_pass_evidence(Path(archive_path)):
        actor_id = source.witness.required_pass_actor_ids[0]
        offsets = pass_candidate_lateral_offsets(
            source.world,
            actor_binding_id=actor_id,
            side=source.side,
        )
        selected: AutomatedWitness | None = None
        selected_offset: float | None = None
        rejected_count = 0
        for offset in offsets:
            selected = generate_causal_release_pass_candidate(
                source.world,
                PassCandidateRequest(
                    actor_binding_id=actor_id,
                    side=source.side,
                    departure_progress_m=R5B_CAUSAL_DEPARTURE_PROGRESS_M,
                    lateral_offset_m=offset,
                    release_tick=R5B_CAUSAL_RELEASE_TICK,
                    linear_target_mps=R5B_CAUSAL_LINEAR_TARGET_MPS,
                    angular_magnitude_radps=R5B_CAUSAL_ANGULAR_MAGNITUDE_RADPS,
                    wait_policy=PassSideWaitPolicy.IMMEDIATE,
                ),
            )
            if selected is not None:
                selected_offset = offset
                break
            rejected_count += 1
        if selected is None or selected_offset is None:
            raise RuntimeError(
                f"R5-B causal PASS search found no strict witness for {source.public_id} "
                f"{source.side.value}"
            )
        validation = validate_ground_truth_witness(
            source.world,
            selected,
            strict_declarations=True,
        )
        result.append(
            CausalR5BPassEvidence(
                schema_version=R5B_TEMPORAL_EVIDENCE_SCHEMA_VERSION,
                source_frozen_evidence_hash=source.evidence_content_hash,
                public_id=source.public_id,
                corpus_ordinal=source.corpus_ordinal,
                side=source.side,
                release_tick=R5B_CAUSAL_RELEASE_TICK,
                rejected_lateral_offset_count=rejected_count,
                selected_lateral_offset_m=selected_offset,
                world=source.world,
                witness=selected,
                validation=validation,
            )
        )
    causal = tuple(result)
    if len(causal) != R5B_EXPECTED_PASS_EVIDENCE_COUNT:
        raise RuntimeError("R5-B causal search did not return the frozen ten cases")
    return causal


def _verify_archive_file(path: Path) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"R5-B R2 archive is missing: {path}")
    if path.stat().st_size != R5B_R2_ARCHIVE_SIZE_BYTES:
        raise ValueError("R5-B R2 archive size mismatch")
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    if digest.hexdigest() != R5B_R2_ARCHIVE_SHA256:
        raise ValueError("R5-B R2 archive SHA-256 mismatch")


def _validated_archive_names(infos: list[ZipInfo]) -> frozenset[str]:
    names: list[str] = []
    for info in infos:
        name = info.filename
        pure = PurePosixPath(name)
        if "\\" in name or pure.is_absolute() or ".." in pure.parts:
            raise ValueError("R5-B archive contains an unsafe member path")
        if not pure.parts or pure.parts[0] != R5B_R2_ARCHIVE_ROOT:
            raise ValueError("R5-B archive member is outside the frozen root")
        names.append(name.rstrip("/"))
    if len(names) != len(set(names)) or len(names) != len({name.casefold() for name in names}):
        raise ValueError("R5-B archive contains duplicate member paths")
    return frozenset(names)


def _selected_episode_directories(names: frozenset[str]) -> tuple[str, ...]:
    selected: list[str] = []
    for prefix in _EXPECTED_VARIANT_PREFIXES:
        matches = sorted(
            name.rsplit("/", 1)[0]
            for name in names
            if name.endswith("/selected_witness.json")
            and name.rsplit("/", 2)[-2].startswith(prefix)
        )
        if len(matches) != 1:
            raise ValueError(f"R5-B archive needs exactly one selected episode for {prefix}")
        selected.append(matches[0])
    return tuple(selected)


def _read_json_member(archive: ZipFile, member: str) -> dict[str, Any]:
    try:
        raw = archive.read(member)
    except KeyError as error:
        raise ValueError(f"R5-B archive member is missing: {member}") from error
    value = json.loads(raw)
    if not isinstance(value, dict):
        raise TypeError(f"R5-B archive member must contain an object: {member}")
    return value


def _restore_world(payload: dict[str, Any]) -> WitnessWorldSnapshot:
    grid_data = _required_mapping(payload, "grid")
    initial = _required_mapping(payload, "initial_state")
    kinematics = _required_mapping(payload, "kinematic_contract")
    constraints = _required_mapping(payload, "maneuver_constraints")
    grid = WitnessGridSpec(
        width=_required_int(grid_data, "width"),
        height=_required_int(grid_data, "height"),
        resolution_m=_required_float(grid_data, "resolution_m"),
        origin_x_m=_required_float(grid_data, "origin_x_m"),
        origin_y_m=_required_float(grid_data, "origin_y_m"),
        occupied_cells=_restore_cells(grid_data.get("occupied_cells")),
        forbidden_cells=_restore_cells(grid_data.get("forbidden_cells")),
    )
    return WitnessWorldSnapshot(
        schema_version=_required_string(payload, "schema_version"),
        source_schema_version=_required_string(payload, "source_schema_version"),
        source_generator_version=_required_string(payload, "source_generator_version"),
        source_projection_hash=_required_string(payload, "source_projection_hash"),
        world_id=_required_string(payload, "world_id"),
        seed=_required_int(payload, "seed"),
        simulation_only=_required_bool(payload, "simulation_only"),
        map_id=_required_string(payload, "map_id"),
        map_revision=_required_int(payload, "map_revision"),
        grid_content_hash=_required_string(payload, "grid_content_hash"),
        grid=grid,
        reference_path=tuple(
            _restore_pose(item) for item in _required_list(payload, "reference_path")
        ),
        initial_state=RobotState(
            pose=_restore_pose(_required_mapping(initial, "pose")),
            twist=_restore_twist(_required_mapping(initial, "twist")),
        ),
        goal_pose=_restore_pose(_required_mapping(payload, "goal_pose")),
        duration_s=_required_float(payload, "duration_s"),
        actors=tuple(
            _restore_actor(item) for item in _required_list(payload, "actors")
        ),
        maneuver_constraints=ManeuverConstraintSpec(
            policy_revision=_required_int(constraints, "policy_revision"),
            passing_policy=PassingPolicy(_required_string(constraints, "passing_policy")),
            allowed_cells=_restore_cells(constraints.get("allowed_cells")),
        ),
        kinematic_contract=WitnessKinematicContract(
            vehicle_profile=VehicleProfile(**_required_mapping(kinematics, "vehicle_profile")),
            maximum_angular_acceleration_radps2=_required_float(
                kinematics,
                "maximum_angular_acceleration_radps2",
            ),
            control_period_s=_required_float(kinematics, "control_period_s"),
            evaluator_period_s=_required_float(kinematics, "evaluator_period_s"),
        ),
        search_config_hash=_required_string(payload, "search_config_hash"),
    )


def _restore_actor(payload: Any) -> WitnessActorTrajectory:
    data = _require_mapping_value(payload, "actor")
    position = _required_mapping(data, "start_position")
    velocity = _required_mapping(data, "velocity")
    return WitnessActorTrajectory(
        actor_binding_id=_required_string(data, "actor_binding_id"),
        active_from_s=_required_float(data, "active_from_s"),
        active_until_s=_required_float(data, "active_until_s"),
        start_position=Point2D(
            x=_required_float(position, "x"),
            y=_required_float(position, "y"),
        ),
        velocity=Vector2D(
            x=_required_float(velocity, "x"),
            y=_required_float(velocity, "y"),
        ),
        radius_m=_required_float(data, "radius_m"),
        trajectory_revision=_required_int(data, "trajectory_revision"),
    )


def _restore_witness(payload: dict[str, Any]) -> AutomatedWitness:
    return AutomatedWitness(
        schema_version=_required_string(payload, "schema_version"),
        witness_id=_required_string(payload, "witness_id"),
        source_projection_hash=_required_string(payload, "source_projection_hash"),
        world_content_hash=_required_string(payload, "world_content_hash"),
        vehicle_profile_hash=_required_string(payload, "vehicle_profile_hash"),
        search_config_hash=_required_string(payload, "search_config_hash"),
        kind=WitnessKind(_required_string(payload, "kind")),
        terminal_mode=WitnessTerminalMode(_required_string(payload, "terminal_mode")),
        points=tuple(
            _restore_witness_point(item) for item in _required_list(payload, "points")
        ),
        required_pass_actor_ids=tuple(
            _require_string_value(value, "required_pass_actor_ids")
            for value in _required_list(payload, "required_pass_actor_ids")
        ),
        departure_time_s=_optional_float(payload.get("departure_time_s")),
        pass_times_by_actor=_restore_pass_times(
            _required_list(payload, "pass_times_by_actor")
        ),
        rejoin_started_at_s=_optional_float(payload.get("rejoin_started_at_s")),
        rejoin_confirmed_at_s=_optional_float(payload.get("rejoin_confirmed_at_s")),
        terminal_dwell_s=_required_float(payload, "terminal_dwell_s"),
    )


def _restore_witness_point(payload: Any) -> WitnessPoint:
    data = _require_mapping_value(payload, "witness point")
    return WitnessPoint(
        time_s=_required_float(data, "time_s"),
        pose=_restore_pose(_required_mapping(data, "pose")),
        twist=_restore_twist(_required_mapping(data, "twist")),
        phase=WitnessPhase(_required_string(data, "phase")),
        source_primitive_id=_required_string(data, "source_primitive_id"),
    )


def _restore_pose(payload: Any) -> Pose2D:
    data = _require_mapping_value(payload, "pose")
    return Pose2D(
        x=_required_float(data, "x"),
        y=_required_float(data, "y"),
        yaw=_required_float(data, "yaw"),
    )


def _restore_twist(payload: Any) -> Twist2D:
    data = _require_mapping_value(payload, "twist")
    return Twist2D(
        linear=_required_float(data, "linear"),
        angular=_required_float(data, "angular"),
    )


def _record_by_witness_hash(payload: dict[str, Any]) -> tuple[dict[str, Any], ...]:
    return tuple(
        _require_mapping_value(item, "selected witness record")
        for item in _required_list(payload, "records")
    )


def _validation_by_witness_hash(payload: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for item in _required_list(payload, "records"):
        record = _require_mapping_value(item, "validation record")
        witness_hash = _required_string(record, "witness_content_hash")
        if witness_hash in result:
            raise ValueError("R5-B archive repeats a validation witness hash")
        result[witness_hash] = _required_mapping(record, "validation")
    return result


def _validate_archived_validation_payload(
    payload: dict[str, Any],
    *,
    world: WitnessWorldSnapshot,
    witness: AutomatedWitness,
) -> None:
    if payload.get("passed") is not True or payload.get("failures") != []:
        raise ValueError("R5-B archived validation must be a passing result")
    if payload.get("source_projection_hash") != world.source_projection_hash:
        raise ValueError("R5-B archived validation source projection mismatch")
    if payload.get("world_content_hash") != world.content_hash:
        raise ValueError("R5-B archived validation world mismatch")
    if payload.get("witness_content_hash") != witness.semantic_content_hash:
        raise ValueError("R5-B archived validation witness mismatch")
    if not isinstance(payload.get("metrics"), dict):
        raise TypeError("R5-B archived validation metrics must be an object")


def _single_role_witness(records: tuple[dict[str, Any], ...], role: str) -> dict[str, Any]:
    matches = []
    for record in records:
        roles = tuple(
            _require_string_value(item, "record role")
            for item in _required_list(record, "roles")
        )
        if role in roles:
            matches.append(_required_mapping(record, "witness"))
    if len(matches) != 1:
        raise ValueError(f"R5-B archive needs exactly one witness with role {role}")
    return matches[0]


def _restore_cells(value: Any) -> tuple[tuple[int, int], ...]:
    if not isinstance(value, list):
        raise TypeError("grid cells must be a list")
    result: list[tuple[int, int]] = []
    for item in value:
        if not isinstance(item, list) or len(item) != 2:
            raise TypeError("grid cell must be a two-item list")
        result.append(
            (
                _require_int_value(item[0], "grid x"),
                _require_int_value(item[1], "grid y"),
            )
        )
    return tuple(result)


def _restore_pass_times(value: list[Any]) -> tuple[tuple[str, float], ...]:
    result: list[tuple[str, float]] = []
    for item in value:
        if not isinstance(item, list) or len(item) != 2:
            raise TypeError("pass time must be a two-item list")
        result.append(
            (
                _require_string_value(item[0], "pass actor id"),
                _require_float_value(item[1], "pass time"),
            )
        )
    return tuple(result)


def _required_mapping(payload: dict[str, Any], key: str) -> dict[str, Any]:
    return _require_mapping_value(payload.get(key), key)


def _require_mapping_value(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise TypeError(f"{name} must be an object")
    return value


def _required_list(payload: dict[str, Any], key: str) -> list[Any]:
    value = payload.get(key)
    if not isinstance(value, list):
        raise TypeError(f"{key} must be a list")
    return value


def _required_string(payload: dict[str, Any], key: str) -> str:
    return _require_string_value(payload.get(key), key)


def _require_string_value(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise TypeError(f"{name} must be a non-empty string")
    return value


def _required_int(payload: dict[str, Any], key: str) -> int:
    return _require_int_value(payload.get(key), key)


def _require_int_value(value: Any, name: str) -> int:
    if type(value) is not int:
        raise TypeError(f"{name} must be an exact integer")
    return value


def _required_bool(payload: dict[str, Any], key: str) -> bool:
    value = payload.get(key)
    if type(value) is not bool:
        raise TypeError(f"{key} must be a bool")
    return value


def _required_float(payload: dict[str, Any], key: str) -> float:
    return _require_float_value(payload.get(key), key)


def _require_float_value(value: Any, name: str) -> float:
    if type(value) not in (int, float):
        raise TypeError(f"{name} must be numeric")
    return float(value)


def _optional_float(value: Any) -> float | None:
    return None if value is None else _require_float_value(value, "optional time")


__all__ = [
    "CausalR5BPassEvidence",
    "FrozenR2PassEvidence",
    "R5B_CAUSAL_RELEASE_TICK",
    "R5B_CAUSAL_RELEASE_TIME_S",
    "R5B_EXPECTED_PASS_EVIDENCE_COUNT",
    "R5B_R2_ARCHIVE_RELATIVE_PATH",
    "R5B_R2_ARCHIVE_SHA256",
    "R5B_R2_ARCHIVE_SIZE_BYTES",
    "R5B_TEMPORAL_EVIDENCE_SCHEMA_VERSION",
    "build_causal_r5b_pass_evidence",
    "frozen_r2_archive_path",
    "load_frozen_r2_pass_evidence",
]
