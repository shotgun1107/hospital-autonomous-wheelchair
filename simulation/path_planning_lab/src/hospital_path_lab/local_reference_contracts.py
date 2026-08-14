"""R4 지역 기동 reference의 label-free 계약과 revision 수명주기.

이 모듈은 Python ``simulation_only`` 연구 계약만 정의한다. 경로 생성, Actor 판단,
controller 실행, shared safety gate와 이동 허가는 수행하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import StrEnum
from math import atan2, cos, isfinite, sin
from re import fullmatch

from hospital_path_lab.contracts import GridSnapshot, Pose2D
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.spatial_oracle_contracts import (
    ManeuverSide,
    SpatialAllowedRegion,
    SpatialPrimitive,
    SpatialRejoinGoal,
    spatial_grid_content_hash,
    spatial_path_content_hash,
)
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile

LOCAL_REFERENCE_SCHEMA_VERSION = "local-maneuver-reference-v2"
LOCAL_REFERENCE_CONTRACT_VERSION = "local-maneuver-reference-contract-v2"
LOCAL_REFERENCE_SET_SCHEMA_VERSION = "local-maneuver-reference-set-v2"
LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION = "local-reference-window-v2"
REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION = "reference-build-context-v1"
SPATIAL_REFERENCE_SEED_SCHEMA_VERSION = "spatial-reference-seed-v1"
TEMPORAL_REFERENCE_EVIDENCE_SCHEMA_VERSION = "temporal-reference-evidence-v1"
REFERENCE_SESSION_BINDING_VERSION = "reference-session-binding-v1"
R4_MINIMUM_CLEARANCE_M = 0.08
R4_COMPARISON_TOLERANCE = 1e-9


class ObservationDependency(StrEnum):
    STATIC_ONLY = "static_only"
    REQUIRED = "required"


class LocalManeuverKind(StrEnum):
    WAIT_OR_FOLLOW = "wait_or_follow"
    PASS_LEFT = "pass_left"
    PASS_RIGHT = "pass_right"


class ReferenceEvidenceLevel(StrEnum):
    SPATIAL_ONLY = "spatial_only"
    GROUND_TRUTH_TEMPORAL = "ground_truth_temporal"
    OBSERVATION_INTEGRATED = "observation_integrated"


class ReferenceBuildStatus(StrEnum):
    REFERENCE_SET_READY = "reference_set_ready"
    WAIT_ONLY = "wait_only"
    NO_REFERENCE = "no_reference"
    SEARCH_INCONCLUSIVE = "search_inconclusive"
    INVALID_INPUT = "invalid_input"


class ReferenceUpperDisposition(StrEnum):
    GLOBAL_REROUTE_REQUEST = "global_reroute_request"
    SUPPORT_REQUEST = "support_request"


class ReferenceKnotRole(StrEnum):
    ANCHOR = "anchor"
    TRANSLATION = "translation"
    ROTATION_ENTRY = "rotation_entry"
    ROTATION_EXIT = "rotation_exit"
    STOP_MARKER = "stop_marker"
    REJOIN = "rejoin"


class ReferenceSectionKind(StrEnum):
    FOLLOW_ORIGINAL = "follow_original"
    DEPART = "depart"
    ROTATE = "rotate"
    BYPASS = "bypass"
    RETURN = "return"
    REJOIN = "rejoin"
    HOLD = "hold"


class ReferenceTravelDirection(StrEnum):
    FORWARD = "forward"
    REVERSE = "reverse"
    NONE = "none"


class ReferenceLifecycleStatus(StrEnum):
    AVAILABLE = "available"
    SUPERSEDED = "superseded"
    STALE = "stale"
    WITHDRAWN = "withdrawn"
    COMPLETED = "completed"


@dataclass(frozen=True, slots=True)
class ReferenceBuildContext:
    schema_version: str
    mission_id: str
    stop_epoch: int
    map_id: str
    map_revision: int
    mission_revision: int
    observation_dependency: ObservationDependency
    observation_revision: int | None
    observation_content_hash: str | None
    static_grid_snapshot: GridSnapshot
    grid_content_hash: str
    allowed_region: SpatialAllowedRegion
    allowed_region_hash: str
    forbidden_cells: tuple[tuple[int, int], ...]
    forbidden_region_hash: str
    vehicle_profile: VehicleProfile
    vehicle_profile_hash: str
    original_reference: tuple[Pose2D, ...]
    original_reference_hash: str
    current_robot_pose: Pose2D
    control_tick: int
    simulation_time_s: float
    context_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != REFERENCE_BUILD_CONTEXT_SCHEMA_VERSION:
            raise ValueError("unsupported reference build context schema")
        _require_nonempty(self.mission_id, "mission_id")
        _require_nonempty(self.map_id, "map_id")
        for name in ("stop_epoch", "map_revision", "mission_revision", "control_tick"):
            _require_exact_nonnegative_int(getattr(self, name), name)
        if not isinstance(self.observation_dependency, ObservationDependency):
            raise TypeError("observation_dependency must be an ObservationDependency")
        _require_finite_nonnegative(self.simulation_time_s, "simulation_time_s")
        _require_finite_pose(self.current_robot_pose, "current_robot_pose")
        if not isinstance(self.static_grid_snapshot, GridSnapshot):
            raise TypeError("static_grid_snapshot must be a GridSnapshot")
        if not self.static_grid_snapshot.input_valid:
            raise ValueError("reference build context requires an input-valid grid snapshot")
        metadata = self.static_grid_snapshot.metadata
        if (self.map_id, self.map_revision, self.mission_revision) != (
            metadata.map_id,
            metadata.map_revision,
            metadata.mission_revision,
        ):
            raise ValueError("reference build context grid provenance mismatch")
        if self.observation_dependency is ObservationDependency.STATIC_ONLY:
            if self.observation_revision is not None or self.observation_content_hash is not None:
                raise ValueError("static-only context cannot claim observation provenance")
        else:
            _require_exact_nonnegative_int(self.observation_revision, "observation_revision")
            assert self.observation_revision is not None
            if self.observation_revision != metadata.observation_revision:
                raise ValueError("required observation revision must match the grid snapshot")
            _require_sha256(self.observation_content_hash, "observation_content_hash")
        if not isinstance(self.allowed_region, SpatialAllowedRegion):
            raise TypeError("allowed_region must be a SpatialAllowedRegion")
        if not self.allowed_region.unrestricted and any(
            not self.static_grid_snapshot.grid.in_bounds(cell) for cell in self.allowed_region.cells
        ):
            raise ValueError("restricted allowed region contains an out-of-bounds cell")
        if not isinstance(self.vehicle_profile, VehicleProfile):
            raise TypeError("vehicle_profile must be a VehicleProfile")
        if self.vehicle_profile != VIRTUAL_DOLL_WHEELCHAIR_V0_1:
            raise ValueError("R4 context requires the frozen virtual wheelchair profile")
        forbidden = _normalize_cells(self.forbidden_cells, "forbidden_cells")
        if forbidden != tuple(sorted(self.static_grid_snapshot.forbidden_cells)):
            raise ValueError("forbidden cells must exactly match the grid snapshot")
        object.__setattr__(self, "forbidden_cells", forbidden)
        reference = tuple(self.original_reference)
        if len(reference) < 2:
            raise ValueError("original_reference must contain at least two poses")
        for pose in reference:
            _require_finite_pose(pose, "original_reference pose")
        object.__setattr__(self, "original_reference", reference)
        expected_hashes = {
            "grid_content_hash": spatial_grid_content_hash(self.static_grid_snapshot.grid),
            "allowed_region_hash": self.allowed_region.content_hash,
            "forbidden_region_hash": canonical_content_hash(forbidden),
            "vehicle_profile_hash": canonical_content_hash(self.vehicle_profile),
            "original_reference_hash": canonical_content_hash(reference),
        }
        for name, expected in expected_hashes.items():
            _require_sha256(getattr(self, name), name)
            if getattr(self, name) != expected:
                raise ValueError(f"{name} mismatch")
        _bind_or_check_hash(self, "context_content_hash", self.expected_content_hash)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "mission_id": self.mission_id,
                "stop_epoch": self.stop_epoch,
                "map_id": self.map_id,
                "map_revision": self.map_revision,
                "mission_revision": self.mission_revision,
                "observation_dependency": self.observation_dependency,
                "observation_revision": self.observation_revision,
                "observation_content_hash": self.observation_content_hash,
                "grid_content_hash": self.grid_content_hash,
                "allowed_region_hash": self.allowed_region_hash,
                "forbidden_region_hash": self.forbidden_region_hash,
                "vehicle_profile_hash": self.vehicle_profile_hash,
                "original_reference_hash": self.original_reference_hash,
                "current_robot_pose": self.current_robot_pose,
                "control_tick": self.control_tick,
                "simulation_time_s": self.simulation_time_s,
            }
        )


@dataclass(frozen=True, slots=True)
class SpatialReferenceSeed:
    schema_version: str
    source_spatial_result_hash: str
    source_spatial_request_hash: str
    source_validation_hash: str
    map_id: str
    map_revision: int
    mission_revision: int
    grid_content_hash: str
    vehicle_profile_hash: str
    side: ManeuverSide
    start_pose: Pose2D
    rejoin_goal: SpatialRejoinGoal
    pose_heading_path: tuple[Pose2D, ...]
    primitive_sequence: tuple[SpatialPrimitive, ...]
    minimum_clearance_m: float
    limitations: tuple[str, ...]
    seed_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != SPATIAL_REFERENCE_SEED_SCHEMA_VERSION:
            raise ValueError("unsupported spatial reference seed schema")
        _require_nonempty(self.map_id, "map_id")
        for name in ("map_revision", "mission_revision"):
            _require_exact_nonnegative_int(getattr(self, name), name)
        for name in (
            "source_spatial_result_hash",
            "source_spatial_request_hash",
            "source_validation_hash",
            "grid_content_hash",
            "vehicle_profile_hash",
        ):
            _require_sha256(getattr(self, name), name)
        if not isinstance(self.side, ManeuverSide):
            raise TypeError("side must be a ManeuverSide")
        _require_finite_pose(self.start_pose, "start_pose")
        if not isinstance(self.rejoin_goal, SpatialRejoinGoal):
            raise TypeError("rejoin_goal must be a SpatialRejoinGoal")
        path = tuple(self.pose_heading_path)
        primitives = tuple(self.primitive_sequence)
        if not path:
            raise ValueError("pose_heading_path must not be empty")
        if len(primitives) != len(path) - 1:
            raise ValueError("spatial seed path and primitive sequence length mismatch")
        for pose in path:
            _require_finite_pose(pose, "pose_heading_path pose")
        if path[0] != self.start_pose:
            raise ValueError("spatial seed path must start at start_pose")
        if not _pose_within_goal(path[-1], self.rejoin_goal):
            raise ValueError("spatial seed path must finish inside the rejoin goal tolerance")
        object.__setattr__(self, "pose_heading_path", path)
        object.__setattr__(self, "primitive_sequence", primitives)
        _require_finite_nonnegative(self.minimum_clearance_m, "minimum_clearance_m")
        if self.minimum_clearance_m + R4_COMPARISON_TOLERANCE < R4_MINIMUM_CLEARANCE_M:
            raise ValueError("spatial seed cannot violate the frozen minimum clearance")
        object.__setattr__(self, "limitations", _normalize_codes(self.limitations, "limitations"))
        _bind_or_check_hash(self, "seed_content_hash", self.expected_content_hash)

    @property
    def source_path_content_hash(self) -> str:
        return spatial_path_content_hash(self.pose_heading_path, self.primitive_sequence)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "source_spatial_result_hash": self.source_spatial_result_hash,
                "source_spatial_request_hash": self.source_spatial_request_hash,
                "source_validation_hash": self.source_validation_hash,
                "map_id": self.map_id,
                "map_revision": self.map_revision,
                "mission_revision": self.mission_revision,
                "grid_content_hash": self.grid_content_hash,
                "vehicle_profile_hash": self.vehicle_profile_hash,
                "side": self.side,
                "start_pose": self.start_pose,
                "rejoin_goal": self.rejoin_goal,
                "source_path_content_hash": self.source_path_content_hash,
                "minimum_clearance_m": self.minimum_clearance_m,
                "limitations": self.limitations,
            }
        )


@dataclass(frozen=True, slots=True)
class TemporalReferenceEvidence:
    schema_version: str
    source_witness_hash: str
    source_validation_hash: str
    maneuver_kind: LocalManeuverKind
    target_actor_binding_ids: tuple[str, ...]
    departure_progress_m: float | None
    pass_progress_m: float | None
    rejoin_progress_m: float | None
    ground_truth_only: bool
    limitations: tuple[str, ...]
    evidence_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != TEMPORAL_REFERENCE_EVIDENCE_SCHEMA_VERSION:
            raise ValueError("unsupported temporal reference evidence schema")
        _require_sha256(self.source_witness_hash, "source_witness_hash")
        _require_sha256(self.source_validation_hash, "source_validation_hash")
        if not isinstance(self.maneuver_kind, LocalManeuverKind):
            raise TypeError("maneuver_kind must be a LocalManeuverKind")
        actors = _normalize_codes(self.target_actor_binding_ids, "target_actor_binding_ids")
        object.__setattr__(self, "target_actor_binding_ids", actors)
        if not isinstance(self.ground_truth_only, bool):
            raise TypeError("ground_truth_only must be a bool")
        progress = (
            self.departure_progress_m,
            self.pass_progress_m,
            self.rejoin_progress_m,
        )
        for value in progress:
            if value is not None:
                _require_finite_nonnegative(value, "temporal progress")
        if self.maneuver_kind in (LocalManeuverKind.PASS_LEFT, LocalManeuverKind.PASS_RIGHT):
            if len(actors) != 1 or any(value is None for value in progress):
                raise ValueError("R4 v1 PASS evidence requires one Actor and all progress anchors")
            assert all(value is not None for value in progress)
            if not (self.departure_progress_m <= self.pass_progress_m <= self.rejoin_progress_m):
                raise ValueError("PASS evidence progress anchors must be ordered")
        elif self.departure_progress_m is not None or self.pass_progress_m is not None:
            raise ValueError("WAIT evidence cannot declare departure or pass progress")
        object.__setattr__(self, "limitations", _normalize_codes(self.limitations, "limitations"))
        _bind_or_check_hash(self, "evidence_content_hash", self.expected_content_hash)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "source_witness_hash": self.source_witness_hash,
                "source_validation_hash": self.source_validation_hash,
                "maneuver_kind": self.maneuver_kind,
                "target_actor_binding_ids": self.target_actor_binding_ids,
                "departure_progress_m": self.departure_progress_m,
                "pass_progress_m": self.pass_progress_m,
                "rejoin_progress_m": self.rejoin_progress_m,
                "ground_truth_only": self.ground_truth_only,
                "limitations": self.limitations,
            }
        )


@dataclass(frozen=True, slots=True)
class ReferenceKnot:
    knot_index: int
    pose: Pose2D
    tangent_yaw: float
    cumulative_translation_arc_m: float
    source_path_index: int
    section_index: int
    knot_roles: tuple[ReferenceKnotRole, ...]

    def __post_init__(self) -> None:
        for name in ("knot_index", "source_path_index", "section_index"):
            _require_exact_nonnegative_int(getattr(self, name), name)
        _require_finite_pose(self.pose, "reference knot pose")
        if not isfinite(self.tangent_yaw):
            raise ValueError("tangent_yaw must be finite")
        _require_finite_nonnegative(
            self.cumulative_translation_arc_m, "cumulative_translation_arc_m"
        )
        raw_roles = tuple(self.knot_roles)
        if not raw_roles or any(not isinstance(role, ReferenceKnotRole) for role in raw_roles):
            raise ValueError("knot_roles must contain ReferenceKnotRole values")
        roles = tuple(sorted(set(raw_roles), key=_enum_order))
        object.__setattr__(self, "knot_roles", roles)


@dataclass(frozen=True, slots=True)
class ReferenceSection:
    section_index: int
    section_kind: ReferenceSectionKind
    travel_direction: ReferenceTravelDirection
    first_knot_index: int
    last_knot_index: int
    entry_requires_stopped: bool
    exit_requires_stopped: bool
    source_primitive_indices: tuple[int, ...]
    section_content_hash: str = ""

    def __post_init__(self) -> None:
        for name in ("section_index", "first_knot_index", "last_knot_index"):
            _require_exact_nonnegative_int(getattr(self, name), name)
        if self.last_knot_index < self.first_knot_index:
            raise ValueError("section knot range must not be reversed")
        if not isinstance(self.section_kind, ReferenceSectionKind):
            raise TypeError("section_kind must be a ReferenceSectionKind")
        if not isinstance(self.travel_direction, ReferenceTravelDirection):
            raise TypeError("travel_direction must be a ReferenceTravelDirection")
        for name in ("entry_requires_stopped", "exit_requires_stopped"):
            if not isinstance(getattr(self, name), bool):
                raise TypeError(f"{name} must be a bool")
        indices = tuple(self.source_primitive_indices)
        if any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0 for value in indices
        ):
            raise ValueError("source_primitive_indices must be non-negative exact integers")
        if tuple(sorted(set(indices))) != indices:
            raise ValueError("source_primitive_indices must be sorted and unique")
        object.__setattr__(self, "source_primitive_indices", indices)
        if self.section_kind in (ReferenceSectionKind.ROTATE, ReferenceSectionKind.HOLD) and not (
            self.entry_requires_stopped and self.exit_requires_stopped
        ):
            raise ValueError("ROTATE and HOLD sections require stopped entry and exit")
        if (
            self.section_kind in (ReferenceSectionKind.ROTATE, ReferenceSectionKind.HOLD)
            and self.travel_direction is not ReferenceTravelDirection.NONE
        ):
            raise ValueError("ROTATE and HOLD sections require NONE travel direction")
        if (
            self.section_kind is ReferenceSectionKind.ROTATE
            and self.first_knot_index == self.last_knot_index
        ):
            raise ValueError("ROTATE sections require distinct entry and exit knots")
        _bind_or_check_hash(self, "section_content_hash", self.expected_content_hash)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "section_index": self.section_index,
                "section_kind": self.section_kind,
                "travel_direction": self.travel_direction,
                "first_knot_index": self.first_knot_index,
                "last_knot_index": self.last_knot_index,
                "entry_requires_stopped": self.entry_requires_stopped,
                "exit_requires_stopped": self.exit_requires_stopped,
                "source_primitive_indices": self.source_primitive_indices,
            }
        )


@dataclass(frozen=True, slots=True)
class ReferenceValidity:
    required_mission_id: str
    required_stop_epoch: int
    required_map_revision: int
    required_mission_revision: int
    required_observation_revision: int | None
    valid_from_control_tick: int
    valid_until_control_tick: int | None
    requires_actual_stop_confirmation: bool = True
    requires_resume_authorization: bool = True
    requires_local_safety_recheck: bool = True

    def __post_init__(self) -> None:
        _require_nonempty(self.required_mission_id, "required_mission_id")
        for name in (
            "required_stop_epoch",
            "required_map_revision",
            "required_mission_revision",
            "valid_from_control_tick",
        ):
            _require_exact_nonnegative_int(getattr(self, name), name)
        if self.required_observation_revision is not None:
            _require_exact_nonnegative_int(
                self.required_observation_revision, "required_observation_revision"
            )
        if self.valid_until_control_tick is not None:
            _require_exact_nonnegative_int(
                self.valid_until_control_tick, "valid_until_control_tick"
            )
            if self.valid_until_control_tick < self.valid_from_control_tick:
                raise ValueError("validity tick range must not be reversed")
        for name in (
            "requires_actual_stop_confirmation",
            "requires_resume_authorization",
            "requires_local_safety_recheck",
        ):
            if getattr(self, name) is not True:
                raise ValueError(f"{name} must remain true in the R4 safety contract")

    @property
    def content_hash(self) -> str:
        return canonical_content_hash(self)


@dataclass(frozen=True, slots=True)
class LocalManeuverReference:
    schema_version: str
    reference_contract_version: str
    candidate_id: str
    maneuver_kind: LocalManeuverKind
    evidence_level: ReferenceEvidenceLevel
    mission_id: str
    stop_epoch: int
    map_id: str
    map_revision: int
    mission_revision: int
    observation_dependency: ObservationDependency
    observation_revision: int | None
    observation_content_hash: str | None
    maneuver_revision: int
    path_revision: int
    reference_session_id: str
    source_spatial_seed_hash: str | None
    source_temporal_evidence_hash: str | None
    original_reference_hash: str
    grid_content_hash: str
    vehicle_profile_hash: str
    allowed_region_hash: str
    forbidden_region_hash: str
    knots: tuple[ReferenceKnot, ...]
    sections: tuple[ReferenceSection, ...]
    departure_knot_index: int | None
    pass_section_index: int | None
    rejoin_knot_index: int
    minimum_validated_static_clearance_m: float
    validity: ReferenceValidity
    generation_reason_codes: tuple[str, ...]
    limitations: tuple[str, ...]
    reference_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != LOCAL_REFERENCE_SCHEMA_VERSION:
            raise ValueError("unsupported local reference schema")
        if self.reference_contract_version != LOCAL_REFERENCE_CONTRACT_VERSION:
            raise ValueError("unsupported local reference contract version")
        for name in ("candidate_id", "reference_session_id"):
            _require_sha256(getattr(self, name), name)
        if not isinstance(self.maneuver_kind, LocalManeuverKind):
            raise TypeError("maneuver_kind must be a LocalManeuverKind")
        if not isinstance(self.evidence_level, ReferenceEvidenceLevel):
            raise TypeError("evidence_level must be a ReferenceEvidenceLevel")
        if not isinstance(self.observation_dependency, ObservationDependency):
            raise TypeError("observation_dependency must be an ObservationDependency")
        _require_nonempty(self.mission_id, "mission_id")
        _require_nonempty(self.map_id, "map_id")
        for name in (
            "stop_epoch",
            "map_revision",
            "mission_revision",
            "maneuver_revision",
            "path_revision",
            "rejoin_knot_index",
        ):
            _require_exact_nonnegative_int(getattr(self, name), name)
        _validate_observation_claim(
            self.observation_dependency,
            self.observation_revision,
            self.observation_content_hash,
        )
        for name in (
            "source_spatial_seed_hash",
            "source_temporal_evidence_hash",
            "original_reference_hash",
            "grid_content_hash",
            "vehicle_profile_hash",
            "allowed_region_hash",
            "forbidden_region_hash",
        ):
            value = getattr(self, name)
            if value is not None:
                _require_sha256(value, name)
        knots = tuple(self.knots)
        sections = tuple(self.sections)
        _validate_reference_structure(knots, sections)
        if knots[0].knot_index != 0 or sections[0].section_index != 0:
            raise ValueError("full reference knot and section indices must start at zero")
        if abs(knots[0].cumulative_translation_arc_m) > R4_COMPARISON_TOLERANCE:
            raise ValueError("full reference translation arc must start at zero")
        object.__setattr__(self, "knots", knots)
        object.__setattr__(self, "sections", sections)
        if self.rejoin_knot_index != len(knots) - 1:
            raise ValueError("rejoin_knot_index must identify the terminal knot")
        terminal_roles = set(knots[-1].knot_roles)
        if not {ReferenceKnotRole.REJOIN, ReferenceKnotRole.STOP_MARKER} <= terminal_roles:
            raise ValueError("terminal knot must carry REJOIN and STOP_MARKER roles")
        _require_finite_nonnegative(
            self.minimum_validated_static_clearance_m,
            "minimum_validated_static_clearance_m",
        )
        if (
            self.minimum_validated_static_clearance_m + R4_COMPARISON_TOLERANCE
            < R4_MINIMUM_CLEARANCE_M
        ):
            raise ValueError("reference cannot violate the frozen minimum clearance")
        if not isinstance(self.validity, ReferenceValidity):
            raise TypeError("validity must be a ReferenceValidity")
        if (
            self.validity.required_mission_id != self.mission_id
            or self.validity.required_stop_epoch != self.stop_epoch
            or self.validity.required_map_revision != self.map_revision
            or self.validity.required_mission_revision != self.mission_revision
            or self.validity.required_observation_revision != self.observation_revision
        ):
            raise ValueError("reference validity provenance mismatch")
        reasons = _normalize_codes(self.generation_reason_codes, "generation_reason_codes")
        if not reasons:
            raise ValueError("reference needs at least one generation reason")
        object.__setattr__(self, "generation_reason_codes", reasons)
        object.__setattr__(self, "limitations", _normalize_codes(self.limitations, "limitations"))
        self._validate_kind_and_evidence()
        _bind_or_check_hash(self, "reference_content_hash", self.expected_content_hash)

    def _validate_kind_and_evidence(self) -> None:
        is_pass = self.maneuver_kind in (
            LocalManeuverKind.PASS_LEFT,
            LocalManeuverKind.PASS_RIGHT,
        )
        if is_pass:
            if self.source_spatial_seed_hash is None:
                raise ValueError("PASS reference requires a spatial seed")
            if self.departure_knot_index is None or self.pass_section_index is None:
                raise ValueError("PASS reference requires departure and pass anchors")
            _require_exact_nonnegative_int(self.departure_knot_index, "departure_knot_index")
            _require_exact_nonnegative_int(self.pass_section_index, "pass_section_index")
            if self.departure_knot_index >= len(self.knots):
                raise ValueError("departure_knot_index is outside the reference")
            if self.pass_section_index >= len(self.sections):
                raise ValueError("pass_section_index is outside the reference")
            if (
                self.sections[self.pass_section_index].section_kind
                is not ReferenceSectionKind.BYPASS
            ):
                raise ValueError("pass_section_index must identify a BYPASS section")
            departure_section = self.sections[self.knots[self.departure_knot_index].section_index]
            if departure_section.section_kind is not ReferenceSectionKind.DEPART:
                raise ValueError("departure_knot_index must identify a DEPART section knot")
            kinds = tuple(section.section_kind for section in self.sections)
            positions = []
            for expected in (
                ReferenceSectionKind.DEPART,
                ReferenceSectionKind.BYPASS,
                ReferenceSectionKind.RETURN,
                ReferenceSectionKind.REJOIN,
            ):
                try:
                    positions.append(kinds.index(expected))
                except ValueError as error:
                    raise ValueError("PASS reference is missing a required section") from error
            if positions != sorted(positions):
                raise ValueError("PASS reference section order is invalid")
            if (
                self.sections[0].section_kind is not ReferenceSectionKind.DEPART
                or self.sections[-1].section_kind is not ReferenceSectionKind.REJOIN
            ):
                raise ValueError("PASS reference must start with DEPART and end with REJOIN")
        elif self.departure_knot_index is not None or self.pass_section_index is not None:
            raise ValueError("WAIT reference cannot declare PASS anchors")
        else:
            kinds = tuple(section.section_kind for section in self.sections)
            if (
                not kinds
                or kinds[0] is not ReferenceSectionKind.HOLD
                or ReferenceSectionKind.FOLLOW_ORIGINAL not in kinds[1:]
            ):
                raise ValueError("WAIT reference must hold before following the original path")
        if self.evidence_level is ReferenceEvidenceLevel.SPATIAL_ONLY:
            if self.observation_dependency is not ObservationDependency.STATIC_ONLY:
                raise ValueError("spatial-only reference cannot depend on observation")
            if self.source_temporal_evidence_hash is not None:
                raise ValueError("spatial-only reference cannot claim temporal evidence")
        elif self.evidence_level is ReferenceEvidenceLevel.GROUND_TRUTH_TEMPORAL:
            if self.source_temporal_evidence_hash is None:
                raise ValueError("ground-truth temporal reference needs temporal evidence")
            if self.observation_dependency is not ObservationDependency.STATIC_ONLY:
                raise ValueError(
                    "ground-truth temporal reference cannot claim observation integration"
                )
        else:
            if self.source_temporal_evidence_hash is None:
                raise ValueError("observation-integrated reference needs temporal evidence")
            if self.observation_dependency is not ObservationDependency.REQUIRED:
                raise ValueError("observation-integrated reference requires observation provenance")

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "reference_contract_version": self.reference_contract_version,
                "candidate_id": self.candidate_id,
                "maneuver_kind": self.maneuver_kind,
                "evidence_level": self.evidence_level,
                "mission_id": self.mission_id,
                "stop_epoch": self.stop_epoch,
                "map_id": self.map_id,
                "map_revision": self.map_revision,
                "mission_revision": self.mission_revision,
                "observation_dependency": self.observation_dependency,
                "observation_revision": self.observation_revision,
                "observation_content_hash": self.observation_content_hash,
                "maneuver_revision": self.maneuver_revision,
                "path_revision": self.path_revision,
                "reference_session_id": self.reference_session_id,
                "source_spatial_seed_hash": self.source_spatial_seed_hash,
                "source_temporal_evidence_hash": self.source_temporal_evidence_hash,
                "original_reference_hash": self.original_reference_hash,
                "grid_content_hash": self.grid_content_hash,
                "vehicle_profile_hash": self.vehicle_profile_hash,
                "allowed_region_hash": self.allowed_region_hash,
                "forbidden_region_hash": self.forbidden_region_hash,
                "knots": self.knots,
                "sections": self.sections,
                "departure_knot_index": self.departure_knot_index,
                "pass_section_index": self.pass_section_index,
                "rejoin_knot_index": self.rejoin_knot_index,
                "minimum_validated_static_clearance_m": (self.minimum_validated_static_clearance_m),
                "validity": self.validity,
                "generation_reason_codes": self.generation_reason_codes,
                "limitations": self.limitations,
            }
        )


@dataclass(frozen=True, slots=True)
class ReferenceSourceRejection:
    source_content_hash: str
    reason_codes: tuple[str, ...]

    def __post_init__(self) -> None:
        _require_sha256(self.source_content_hash, "source_content_hash")
        reasons = _normalize_codes(self.reason_codes, "reason_codes")
        if not reasons:
            raise ValueError("source rejection requires a reason")
        object.__setattr__(self, "reason_codes", reasons)


@dataclass(frozen=True, slots=True)
class LocalManeuverReferenceSet:
    schema_version: str
    status: ReferenceBuildStatus
    termination_reason: str
    build_context_hash: str
    maneuver_revision: int
    candidates: tuple[LocalManeuverReference, ...]
    upper_dispositions: tuple[ReferenceUpperDisposition, ...]
    rejected_sources: tuple[ReferenceSourceRejection, ...]
    limitations: tuple[str, ...]
    elapsed_nonqualification_ns: int
    semantic_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != LOCAL_REFERENCE_SET_SCHEMA_VERSION:
            raise ValueError("unsupported local reference set schema")
        if not isinstance(self.status, ReferenceBuildStatus):
            raise TypeError("status must be a ReferenceBuildStatus")
        _require_nonempty(self.termination_reason, "termination_reason")
        _require_sha256(self.build_context_hash, "build_context_hash")
        _require_exact_nonnegative_int(self.maneuver_revision, "maneuver_revision")
        _require_exact_nonnegative_int(
            self.elapsed_nonqualification_ns, "elapsed_nonqualification_ns"
        )
        candidates = tuple(sorted(self.candidates, key=_candidate_sort_key))
        if len({candidate.candidate_id for candidate in candidates}) != len(candidates):
            raise ValueError("reference candidates must have unique candidate IDs")
        if len({candidate.reference_session_id for candidate in candidates}) != len(candidates):
            raise ValueError("reference candidates must have unique reference sessions")
        if any(candidate.maneuver_revision != self.maneuver_revision for candidate in candidates):
            raise ValueError("candidate maneuver revision must match the reference set")
        object.__setattr__(self, "candidates", candidates)
        raw_dispositions = tuple(self.upper_dispositions)
        if any(not isinstance(item, ReferenceUpperDisposition) for item in raw_dispositions):
            raise TypeError("upper_dispositions must contain ReferenceUpperDisposition values")
        dispositions = tuple(sorted(set(raw_dispositions), key=_enum_order))
        object.__setattr__(self, "upper_dispositions", dispositions)
        rejections = tuple(sorted(self.rejected_sources, key=lambda item: item.source_content_hash))
        if len({item.source_content_hash for item in rejections}) != len(rejections):
            raise ValueError("rejected source hashes must be unique")
        object.__setattr__(self, "rejected_sources", rejections)
        object.__setattr__(self, "limitations", _normalize_codes(self.limitations, "limitations"))
        if self.status is ReferenceBuildStatus.REFERENCE_SET_READY and not candidates:
            raise ValueError("ready reference set requires at least one candidate")
        if self.status is ReferenceBuildStatus.WAIT_ONLY and (
            len(candidates) != 1
            or candidates[0].maneuver_kind is not LocalManeuverKind.WAIT_OR_FOLLOW
        ):
            raise ValueError("WAIT_ONLY requires exactly one WAIT_OR_FOLLOW candidate")
        if (
            self.status
            in (
                ReferenceBuildStatus.NO_REFERENCE,
                ReferenceBuildStatus.SEARCH_INCONCLUSIVE,
                ReferenceBuildStatus.INVALID_INPUT,
            )
            and candidates
        ):
            raise ValueError("non-reference result cannot carry candidates")
        _bind_or_check_hash(self, "semantic_content_hash", self.expected_semantic_hash)

    @property
    def expected_semantic_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "status": self.status,
                "termination_reason": self.termination_reason,
                "build_context_hash": self.build_context_hash,
                "maneuver_revision": self.maneuver_revision,
                "candidates": self.candidates,
                "upper_dispositions": self.upper_dispositions,
                "rejected_sources": self.rejected_sources,
                "limitations": self.limitations,
            }
        )


@dataclass(frozen=True, slots=True)
class LocalReferenceWindow:
    schema_version: str
    reference_session_id: str
    maneuver_revision: int
    path_revision: int
    subgoal_revision: int
    full_reference_hash: str
    source_control_tick: int
    start_knot_index: int
    end_knot_index: int
    knots: tuple[ReferenceKnot, ...]
    sections: tuple[ReferenceSection, ...]
    terminal_rejoin_included: bool
    window_content_hash: str = ""

    def __post_init__(self) -> None:
        if self.schema_version != LOCAL_REFERENCE_WINDOW_SCHEMA_VERSION:
            raise ValueError("unsupported local reference window schema")
        _require_sha256(self.reference_session_id, "reference_session_id")
        _require_sha256(self.full_reference_hash, "full_reference_hash")
        for name in (
            "maneuver_revision",
            "path_revision",
            "subgoal_revision",
            "source_control_tick",
            "start_knot_index",
            "end_knot_index",
        ):
            _require_exact_nonnegative_int(getattr(self, name), name)
        if self.end_knot_index < self.start_knot_index:
            raise ValueError("window knot range must not be reversed")
        if not isinstance(self.terminal_rejoin_included, bool):
            raise TypeError("terminal_rejoin_included must be a bool")
        knots = tuple(self.knots)
        sections = tuple(self.sections)
        _validate_reference_structure(knots, sections)
        if (
            knots[0].knot_index != self.start_knot_index
            or knots[-1].knot_index != self.end_knot_index
        ):
            raise ValueError("window range must match its knot indices")
        if tuple(knot.knot_index for knot in knots) != tuple(
            range(self.start_knot_index, self.end_knot_index + 1)
        ):
            raise ValueError("window knots must be a contiguous full-reference slice")
        has_terminal = {
            ReferenceKnotRole.REJOIN,
            ReferenceKnotRole.STOP_MARKER,
        } <= set(knots[-1].knot_roles)
        if self.terminal_rejoin_included != has_terminal:
            raise ValueError("terminal_rejoin_included does not match the terminal knot")
        object.__setattr__(self, "knots", knots)
        object.__setattr__(self, "sections", sections)
        _bind_or_check_hash(self, "window_content_hash", self.expected_content_hash)

    @property
    def expected_content_hash(self) -> str:
        return canonical_content_hash(
            {
                "schema_version": self.schema_version,
                "reference_session_id": self.reference_session_id,
                "maneuver_revision": self.maneuver_revision,
                "path_revision": self.path_revision,
                "subgoal_revision": self.subgoal_revision,
                "full_reference_hash": self.full_reference_hash,
                "start_knot_index": self.start_knot_index,
                "end_knot_index": self.end_knot_index,
                "knots": self.knots,
                "sections": self.sections,
                "terminal_rejoin_included": self.terminal_rejoin_included,
            }
        )


@dataclass(frozen=True, slots=True)
class ReferenceRevisionBinding:
    binding_version: str
    mission_id: str
    stop_epoch: int
    maneuver_revision: int
    path_revision: int
    subgoal_revision: int
    candidate_id: str
    reference_session_id: str
    full_reference_hash: str
    window_content_hash: str
    lifecycle: ReferenceLifecycleStatus = ReferenceLifecycleStatus.AVAILABLE

    def __post_init__(self) -> None:
        if self.binding_version != REFERENCE_SESSION_BINDING_VERSION:
            raise ValueError("unsupported reference session binding version")
        _require_nonempty(self.mission_id, "mission_id")
        for name in ("stop_epoch", "maneuver_revision", "path_revision", "subgoal_revision"):
            _require_exact_nonnegative_int(getattr(self, name), name)
        for name in (
            "candidate_id",
            "reference_session_id",
            "full_reference_hash",
            "window_content_hash",
        ):
            _require_sha256(getattr(self, name), name)
        if not isinstance(self.lifecycle, ReferenceLifecycleStatus):
            raise TypeError("lifecycle must be a ReferenceLifecycleStatus")


@dataclass(frozen=True, slots=True)
class ReferenceRevisionDecision:
    accepted: bool
    duplicate: bool
    reason_code: str
    next_binding: ReferenceRevisionBinding | None

    def __post_init__(self) -> None:
        if not isinstance(self.accepted, bool) or not isinstance(self.duplicate, bool):
            raise TypeError("revision decision flags must be bool values")
        _require_nonempty(self.reason_code, "reason_code")
        if self.duplicate and not self.accepted:
            raise ValueError("duplicate revision decision must be accepted")
        if self.accepted != (self.next_binding is not None):
            raise ValueError("accepted revision decision must carry the next binding")


def reference_revision_binding(
    reference: LocalManeuverReference,
    window: LocalReferenceWindow,
) -> ReferenceRevisionBinding:
    """같은 full reference와 window를 하나의 revision binding으로 결박한다."""

    if (
        window.reference_session_id != reference.reference_session_id
        or window.maneuver_revision != reference.maneuver_revision
        or window.path_revision != reference.path_revision
        or window.full_reference_hash != reference.reference_content_hash
    ):
        raise ValueError("reference and window identity mismatch")
    return ReferenceRevisionBinding(
        binding_version=REFERENCE_SESSION_BINDING_VERSION,
        mission_id=reference.mission_id,
        stop_epoch=reference.stop_epoch,
        maneuver_revision=reference.maneuver_revision,
        path_revision=reference.path_revision,
        subgoal_revision=window.subgoal_revision,
        candidate_id=reference.candidate_id,
        reference_session_id=reference.reference_session_id,
        full_reference_hash=reference.reference_content_hash,
        window_content_hash=window.window_content_hash,
    )


def evaluate_reference_revision_update(
    current: ReferenceRevisionBinding | None,
    incoming: ReferenceRevisionBinding,
) -> ReferenceRevisionDecision:
    """현재 binding에 대한 incoming update를 mutation 없이 판정한다."""

    if not isinstance(incoming, ReferenceRevisionBinding):
        raise TypeError("incoming must be a ReferenceRevisionBinding")
    if current is None:
        if incoming.lifecycle is not ReferenceLifecycleStatus.AVAILABLE:
            return _revision_rejection("initial_binding_not_available")
        return ReferenceRevisionDecision(True, False, "initial_binding_accepted", incoming)
    if not isinstance(current, ReferenceRevisionBinding):
        raise TypeError("current must be a ReferenceRevisionBinding or None")
    if current.lifecycle is not ReferenceLifecycleStatus.AVAILABLE:
        return _revision_rejection("current_binding_is_terminal")
    if incoming.lifecycle is not ReferenceLifecycleStatus.AVAILABLE:
        return _revision_rejection("incoming_binding_not_available")
    if incoming.mission_id != current.mission_id:
        return _revision_rejection("mission_id_mismatch")
    if incoming.stop_epoch < current.stop_epoch:
        return _revision_rejection("stop_epoch_regression")
    current_revisions = (
        current.maneuver_revision,
        current.path_revision,
        current.subgoal_revision,
    )
    incoming_revisions = (
        incoming.maneuver_revision,
        incoming.path_revision,
        incoming.subgoal_revision,
    )
    if any(new < old for old, new in zip(current_revisions, incoming_revisions, strict=True)):
        return _revision_rejection("revision_regression")
    if incoming_revisions == current_revisions and incoming.stop_epoch == current.stop_epoch:
        if incoming == current:
            return ReferenceRevisionDecision(True, True, "duplicate_binding", current)
        return _revision_rejection("same_revision_different_content")
    if incoming.stop_epoch > current.stop_epoch:
        if incoming.maneuver_revision <= current.maneuver_revision:
            return _revision_rejection("stop_epoch_requires_new_maneuver_revision")
        if incoming.reference_session_id == current.reference_session_id:
            return _revision_rejection("stop_epoch_requires_new_session")
    if incoming.maneuver_revision == current.maneuver_revision:
        if incoming.candidate_id != current.candidate_id:
            return _revision_rejection("candidate_changed_without_maneuver_revision")
        if incoming.path_revision == current.path_revision:
            if incoming.reference_session_id != current.reference_session_id:
                return _revision_rejection("window_update_changed_session")
            if incoming.full_reference_hash != current.full_reference_hash:
                return _revision_rejection("same_path_revision_different_reference")
            if incoming.subgoal_revision == current.subgoal_revision:
                return _revision_rejection("same_revision_different_content")
            if incoming.window_content_hash == current.window_content_hash:
                return _revision_rejection("subgoal_revision_without_window_change")
            return ReferenceRevisionDecision(True, False, "subgoal_revision_advanced", incoming)
        if incoming.reference_session_id == current.reference_session_id:
            return _revision_rejection("path_revision_requires_new_session")
        if incoming.full_reference_hash == current.full_reference_hash:
            return _revision_rejection("path_revision_without_content_change")
        return ReferenceRevisionDecision(True, False, "path_revision_advanced", incoming)
    if incoming.reference_session_id == current.reference_session_id:
        return _revision_rejection("maneuver_revision_requires_new_session")
    return ReferenceRevisionDecision(True, False, "maneuver_revision_advanced", incoming)


def transition_reference_lifecycle(
    binding: ReferenceRevisionBinding,
    target: ReferenceLifecycleStatus,
) -> ReferenceRevisionBinding:
    """AVAILABLE binding을 terminal lifecycle로 단방향 전이한다."""

    if not isinstance(binding, ReferenceRevisionBinding):
        raise TypeError("binding must be a ReferenceRevisionBinding")
    if not isinstance(target, ReferenceLifecycleStatus):
        raise TypeError("target must be a ReferenceLifecycleStatus")
    if target is binding.lifecycle:
        return binding
    if binding.lifecycle is not ReferenceLifecycleStatus.AVAILABLE:
        raise ValueError("terminal reference lifecycle cannot transition again")
    if target is ReferenceLifecycleStatus.AVAILABLE:
        return binding
    return replace(binding, lifecycle=target)


def _validate_reference_structure(
    knots: tuple[ReferenceKnot, ...],
    sections: tuple[ReferenceSection, ...],
) -> None:
    if not knots or not sections:
        raise ValueError("reference requires knots and sections")
    if tuple(knot.knot_index for knot in knots) != tuple(
        range(knots[0].knot_index, knots[0].knot_index + len(knots))
    ):
        raise ValueError("reference knot indices must be contiguous")
    if tuple(section.section_index for section in sections) != tuple(
        range(sections[0].section_index, sections[0].section_index + len(sections))
    ):
        raise ValueError("reference section indices must be contiguous")
    if sections[0].first_knot_index != knots[0].knot_index:
        raise ValueError("first section must start at the first knot")
    if sections[-1].last_knot_index != knots[-1].knot_index:
        raise ValueError("last section must end at the last knot")
    expected_first = knots[0].knot_index
    by_knot_index = {knot.knot_index: knot for knot in knots}
    previous_arc = knots[0].cumulative_translation_arc_m
    previous_travel_section: ReferenceSection | None = None
    sections_since_travel: list[ReferenceSection] = []
    for section in sections:
        if section.first_knot_index != expected_first:
            raise ValueError("reference sections must cover knots without gaps or overlap")
        for knot_index in range(section.first_knot_index, section.last_knot_index + 1):
            knot = by_knot_index.get(knot_index)
            if knot is None or knot.section_index != section.section_index:
                raise ValueError("reference knot section binding mismatch")
        section_knots = tuple(
            by_knot_index[knot_index]
            for knot_index in range(section.first_knot_index, section.last_knot_index + 1)
        )
        movement_present = any(
            _pose_distance(left.pose, right.pose) > R4_COMPARISON_TOLERANCE
            for left, right in zip(section_knots, section_knots[1:], strict=False)
        )
        if (
            section.travel_direction is ReferenceTravelDirection.NONE
            and movement_present
            and section.section_kind is not ReferenceSectionKind.HOLD
            and (
                not section.entry_requires_stopped
                or not section.exit_requires_stopped
                or ReferenceKnotRole.STOP_MARKER not in section_knots[0].knot_roles
                or ReferenceKnotRole.STOP_MARKER not in section_knots[-1].knot_roles
                or any(ReferenceKnotRole.ANCHOR not in knot.knot_roles for knot in section_knots)
            )
        ):
            raise ValueError("moving NONE section must be a stopped abstract anchor connector")
        if section.section_kind is ReferenceSectionKind.ROTATE:
            if ReferenceKnotRole.ROTATION_ENTRY not in section_knots[0].knot_roles:
                raise ValueError("ROTATE section entry knot requires ROTATION_ENTRY role")
            if ReferenceKnotRole.ROTATION_EXIT not in section_knots[-1].knot_roles:
                raise ValueError("ROTATE section exit knot requires ROTATION_EXIT role")
        if section.section_kind is ReferenceSectionKind.HOLD:
            anchor = section_knots[0]
            if any(
                _pose_distance(anchor.pose, knot.pose) > R4_COMPARISON_TOLERANCE
                or abs(knot.cumulative_translation_arc_m - anchor.cumulative_translation_arc_m)
                > R4_COMPARISON_TOLERANCE
                for knot in section_knots[1:]
            ):
                raise ValueError("HOLD section knots must remain at one stopped pose")
        if section.travel_direction is ReferenceTravelDirection.NONE:
            if previous_travel_section is not None:
                sections_since_travel.append(section)
        else:
            if (
                previous_travel_section is not None
                and previous_travel_section.travel_direction is not section.travel_direction
            ):
                if not previous_travel_section.exit_requires_stopped:
                    raise ValueError("travel direction change requires stopped exit")
                if not section.entry_requires_stopped:
                    raise ValueError("travel direction change requires stopped entry")
                previous_last = by_knot_index[previous_travel_section.last_knot_index]
                current_first = by_knot_index[section.first_knot_index]
                if (
                    ReferenceKnotRole.STOP_MARKER not in previous_last.knot_roles
                    or ReferenceKnotRole.STOP_MARKER not in current_first.knot_roles
                ):
                    raise ValueError("travel direction change requires stop markers")
                for transition in sections_since_travel:
                    first = by_knot_index[transition.first_knot_index]
                    last = by_knot_index[transition.last_knot_index]
                    if (
                        not transition.entry_requires_stopped
                        or not transition.exit_requires_stopped
                        or ReferenceKnotRole.STOP_MARKER not in first.knot_roles
                        or ReferenceKnotRole.STOP_MARKER not in last.knot_roles
                    ):
                        raise ValueError("travel direction change requires stopped intermediary")
            previous_travel_section = section
            sections_since_travel = []
        expected_first = section.last_knot_index + 1
    for left, right in zip(knots, knots[1:], strict=False):
        if right.cumulative_translation_arc_m + R4_COMPARISON_TOLERANCE < previous_arc:
            raise ValueError("reference translation arc must not regress")
        distance = _pose_distance(left.pose, right.pose)
        arc_delta = right.cumulative_translation_arc_m - left.cumulative_translation_arc_m
        if distance > R4_COMPARISON_TOLERANCE:
            if abs(arc_delta - distance) > R4_COMPARISON_TOLERANCE:
                raise ValueError("translation arc must equal adjacent metric distance")
        elif abs(arc_delta) > R4_COMPARISON_TOLERANCE:
            raise ValueError("same-position knots cannot advance translation arc")
        previous_arc = right.cumulative_translation_arc_m


def _validate_observation_claim(
    dependency: ObservationDependency,
    revision: int | None,
    content_hash: str | None,
) -> None:
    if dependency is ObservationDependency.STATIC_ONLY:
        if revision is not None or content_hash is not None:
            raise ValueError("static-only reference cannot claim observation provenance")
        return
    _require_exact_nonnegative_int(revision, "observation_revision")
    _require_sha256(content_hash, "observation_content_hash")


def _pose_within_goal(pose: Pose2D, goal: SpatialRejoinGoal) -> bool:
    return (
        _pose_distance(pose, goal.pose) <= goal.position_tolerance_m + R4_COMPARISON_TOLERANCE
        and abs(_angle_delta(pose.yaw, goal.pose.yaw))
        <= goal.heading_tolerance_rad + R4_COMPARISON_TOLERANCE
    )


def _angle_delta(left: float, right: float) -> float:
    return atan2(sin(left - right), cos(left - right))


def _pose_distance(left: Pose2D, right: Pose2D) -> float:
    return ((right.x - left.x) ** 2 + (right.y - left.y) ** 2) ** 0.5


def _candidate_sort_key(reference: LocalManeuverReference) -> tuple[int, str]:
    order = {
        LocalManeuverKind.WAIT_OR_FOLLOW: 0,
        LocalManeuverKind.PASS_LEFT: 1,
        LocalManeuverKind.PASS_RIGHT: 2,
    }
    return order[reference.maneuver_kind], reference.candidate_id


def _revision_rejection(reason_code: str) -> ReferenceRevisionDecision:
    return ReferenceRevisionDecision(False, False, reason_code, None)


def _enum_order(value: StrEnum) -> str:
    return value.value


def _normalize_cells(
    cells: tuple[tuple[int, int], ...], field_name: str
) -> tuple[tuple[int, int], ...]:
    normalized = tuple(sorted(set(cells)))
    if any(
        not isinstance(cell, tuple)
        or len(cell) != 2
        or any(isinstance(value, bool) or not isinstance(value, int) for value in cell)
        for cell in normalized
    ):
        raise TypeError(f"{field_name} must contain exact integer (x, y) tuples")
    return normalized


def _normalize_codes(values: tuple[str, ...], field_name: str) -> tuple[str, ...]:
    normalized = tuple(sorted(set(values)))
    if any(not isinstance(value, str) or not value for value in normalized):
        raise ValueError(f"{field_name} must contain non-empty strings")
    return normalized


def _bind_or_check_hash(instance: object, field_name: str, expected: str) -> None:
    value = getattr(instance, field_name)
    if value:
        _require_sha256(value, field_name)
        if value != expected:
            raise ValueError(f"{field_name} mismatch")
    else:
        object.__setattr__(instance, field_name, expected)


def _require_nonempty(value: str, field_name: str) -> None:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field_name} must not be empty")


def _require_exact_nonnegative_int(value: int | None, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field_name} must be a non-negative exact integer")


def _require_finite_nonnegative(value: float, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise ValueError(f"{field_name} must be finite")
    if value < 0.0:
        raise ValueError(f"{field_name} must be non-negative")


def _require_finite_pose(pose: Pose2D, field_name: str) -> None:
    if not isinstance(pose, Pose2D) or not all(
        isfinite(value) for value in (pose.x, pose.y, pose.yaw)
    ):
        raise ValueError(f"{field_name} must be a finite Pose2D")


def _require_sha256(value: str | None, field_name: str) -> None:
    if not isinstance(value, str) or fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{field_name} must be a lowercase SHA-256 hex digest")
