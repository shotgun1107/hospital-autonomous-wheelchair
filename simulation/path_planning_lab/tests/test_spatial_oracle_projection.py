from __future__ import annotations

from dataclasses import fields, replace

import numpy as np
import pytest

from hospital_path_lab.dynamic_corpus import (
    DynamicExpectationCategory,
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_witness_contracts import project_public_witness_world
from hospital_path_lab.spatial_oracle_contracts import ManeuverSide, SpatialOracleStatus
from hospital_path_lab.spatial_oracle_lattice import search_bounded_spatial_oracle
from hospital_path_lab.spatial_oracle_projection import (
    project_witness_world_to_spatial_request,
)


def _v6_case(case_id: str):
    return next(
        episode
        for episode in generate_dynamic_v6_public_corpus()
        if episode.latent_case_id == case_id
    )


def test_r3_projection_drops_actor_time_and_evaluator_labels() -> None:
    world = project_public_witness_world(_v6_case("same-direction-wide-r00"))

    request = project_witness_world_to_spatial_request(
        world, maneuver_side=ManeuverSide.LEFT
    )

    names = {field.name for field in fields(request)}
    assert names.isdisjoint(
        {
            "actors",
            "duration_s",
            "expectation_category",
            "oracle_spec",
            "latent_case_id",
            "split",
            "hidden_seed",
        }
    )
    assert all(actor.actor_binding_id not in repr(request) for actor in world.actors)
    assert request.integrity_failure() is None


def test_legacy_evaluator_label_changes_do_not_change_r3_projection() -> None:
    episode = generate_dynamic_corpus()[0]
    relabeled = replace(
        episode,
        expectation_category=DynamicExpectationCategory.NO_SAFE_SOLUTION,
        progressable=not episode.progressable,
        blocking_cleared_at_s=None,
        observation_fault="evaluator-only-change",
    )
    original_world = project_public_witness_world(episode)
    relabeled_world = project_public_witness_world(relabeled)

    original = project_witness_world_to_spatial_request(
        original_world, maneuver_side=ManeuverSide.RIGHT
    )
    changed = project_witness_world_to_spatial_request(
        relabeled_world, maneuver_side=ManeuverSide.RIGHT
    )

    assert changed.request_content_hash == original.request_content_hash
    assert changed.expected_content_hash == original.expected_content_hash
    assert np.array_equal(changed.static_grid.occupancy, original.static_grid.occupancy)


@pytest.mark.parametrize("side", (ManeuverSide.LEFT, ManeuverSide.RIGHT))
def test_public_same_direction_wide_static_projection_is_spatially_feasible(
    side: ManeuverSide,
) -> None:
    world = project_public_witness_world(_v6_case("same-direction-wide-r00"))
    request = project_witness_world_to_spatial_request(world, maneuver_side=side)

    result = search_bounded_spatial_oracle(request)

    assert result.status is SpatialOracleStatus.SPATIALLY_FEASIBLE
    assert result.validation is not None and result.validation.passed
    assert result.validation.maximum_signed_side_excursion_m >= 0.10 - 1e-9


def test_multi_segment_corner_requires_explicit_future_segment_projection() -> None:
    world = project_public_witness_world(_v6_case("corner-intersection-v6"))

    with pytest.raises(ValueError, match="one explicit straight segment"):
        project_witness_world_to_spatial_request(
            world, maneuver_side=ManeuverSide.LEFT
        )
