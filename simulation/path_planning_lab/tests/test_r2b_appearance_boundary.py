from __future__ import annotations

import pytest

from hospital_path_lab.dynamic_contracts import DynamicObservationFrameKind
from hospital_path_lab.dynamic_corpus import (
    generate_dynamic_v6_public_corpus,
    generate_episode_observation_slots,
)
from hospital_path_lab.dynamic_observation import FUNCTIONAL_IDEAL_OBSERVATION_PROFILE


def test_second_risk_appearance_precedes_the_first_nonempty_ideal_delivery() -> None:
    episode = next(
        item
        for item in generate_dynamic_v6_public_corpus()
        if item.variant == "second-risk-after-corner"
    )
    second_actor = max(episode.actors, key=lambda actor: actor.active_from_s)
    appearance_time_s = second_actor.active_from_s
    assert appearance_time_s == pytest.approx(13.0)
    assert episode.actor_states_at(appearance_time_s)

    slots = generate_episode_observation_slots(
        episode,
        profile=FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    )
    latest_delivered = max(
        (
            slot
            for slot in slots
            if slot.frame is not None
            and slot.scheduled_delivery_at_s <= appearance_time_s + 1e-12
        ),
        key=lambda slot: slot.scheduled_delivery_at_s,
    )
    frame = latest_delivered.frame
    assert frame is not None
    assert frame.delivered_at_s == pytest.approx(appearance_time_s)
    assert frame.observed_at_s == pytest.approx(appearance_time_s - 0.1)
    assert frame.frame_kind is DynamicObservationFrameKind.EMPTY
    assert frame.tracks == ()


def test_fresh_empty_is_not_a_future_no_actor_guarantee() -> None:
    episode = next(
        item
        for item in generate_dynamic_v6_public_corpus()
        if item.variant == "second-risk-after-corner"
    )
    second_actor = max(episode.actors, key=lambda actor: actor.active_from_s)
    before = second_actor.active_from_s - 0.1
    after = second_actor.active_from_s

    assert not any(
        actor.actor_id == second_actor.actor_id
        for actor in episode.actor_states_at(before)
    )
    assert any(
        actor.actor_id == second_actor.actor_id
        for actor in episode.actor_states_at(after)
    )
