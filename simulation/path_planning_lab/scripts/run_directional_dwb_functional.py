"""Run a PUBLIC-only directional DWB functional isolation lane.

This diagnostic keeps Normal latency and Gaussian noise while removing only
independent frame dropout.  It never runs hidden data or the promotion runner.
Progress and final evidence are written atomically so a long pure-Python run
can be monitored without relying on terminal buffering.
"""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import replace
from pathlib import Path
from time import perf_counter

from hospital_path_lab.contracts import Pose2D, RobotState, Twist2D
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_PERIOD_S,
    DynamicMotionState,
)
from hospital_path_lab.dynamic_corpus import generate_dynamic_v6_public_corpus
from hospital_path_lab.dynamic_directional_experiment import (
    DirectionalPublicEpisodeContextFactory,
)
from hospital_path_lab.dynamic_evaluation import evaluate_dynamic_pipeline
from hospital_path_lab.dynamic_observation import (
    FUNCTIONAL_IDEAL_OBSERVATION_PROFILE,
    FUNCTIONAL_NO_DROPOUT_OBSERVATION_PROFILE,
)
from hospital_path_lab.dynamic_safety import DynamicSafetyGate
from hospital_path_lab.local_algorithms.dwb_reference import SourceDerivedDynamicDwbController
from hospital_path_lab.local_detour_policy import (
    DirectionalLocalDetourDwbController,
)
from hospital_path_lab.simulation import simulate_dynamic_controller_pipeline


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-ticks", type=int, default=220)
    parser.add_argument("--progress-every", type=int, default=5)
    parser.add_argument(
        "--profile",
        choices=("no_dropout", "ideal"),
        default="no_dropout",
    )
    parser.add_argument("--local-detour", action="store_true")
    parser.add_argument("--resume-from", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.max_ticks <= 0 or args.progress_every <= 0:
        parser.error("tick values must be positive")

    episode = next(
        item
        for item in generate_dynamic_v6_public_corpus()
        if item.latent_case_id == "same-direction-wide-r00"
    )
    profile = (
        FUNCTIONAL_IDEAL_OBSERVATION_PROFILE
        if args.profile == "ideal"
        else FUNCTIONAL_NO_DROPOUT_OBSERVATION_PROFILE
    )
    factory = DirectionalPublicEpisodeContextFactory(episode, profile)
    if args.local_detour:
        detour_controller = DirectionalLocalDetourDwbController()
        controller = detour_controller
        detour_policy = detour_controller.policy
    else:
        detour_controller = None
        controller = SourceDerivedDynamicDwbController()
        detour_policy = None
    start_tick_id = 0
    initial_state = episode.initial_state
    resume_gate = None
    if args.resume_from is not None:
        if not args.local_detour:
            parser.error("resume currently requires --local-detour")
        previous = json.loads(args.resume_from.read_text(encoding="utf-8"))
        if previous.get("state") != "complete":
            parser.error("resume input must be a completed diagnostic result")
        if previous.get("episode_id") != episode.episode_id:
            parser.error("resume input episode does not match the public case")
        if previous.get("profile") != profile.name.value:
            parser.error("resume input observation profile does not match")
        trace = previous.get("tick_trace")
        if not isinstance(trace, list) or not trace:
            parser.error("resume input does not contain a tick trace")
        start_tick_id = int(trace[-1]["tick_id"]) + 1
        final_pose = previous["final_pose"]
        final_twist = previous["final_twist"]
        initial_state = RobotState(
            Pose2D(
                float(final_pose["x"]),
                float(final_pose["y"]),
                float(final_pose["yaw"]),
            ),
            Twist2D(
                float(final_twist["linear"]),
                float(final_twist["angular"]),
            ),
        )
        restored_path = tuple(
            Pose2D(float(item["x"]), float(item["y"]), float(item["yaw"]))
            for item in previous["local_reference_path"]
        )
        assert detour_controller is not None
        detour_controller.restore(
            restored_path,
            active_waypoint_index=int(previous["local_waypoint_index"]),
        )
        resume_gate = DynamicSafetyGate()
        for tick_id in range(start_tick_id):
            factory(
                tick_id,
                tick_id * DYNAMIC_CONTROL_PERIOD_S,
                episode.initial_state,
                resume_gate,
            )
    if start_tick_id >= args.max_ticks:
        parser.error("--max-ticks must be greater than the resume tick")
    started = perf_counter()
    controller_timings_s: list[float] = []

    class ProgressController:
        name = controller.name

        def step(self, snapshot):
            step_started = perf_counter()
            result = controller.step(snapshot)
            elapsed_s = perf_counter() - step_started
            controller_timings_s.append(elapsed_s)
            if snapshot.tick_id % args.progress_every == 0:
                pose = snapshot.robot_state.pose
                _write_json(
                    args.output,
                    {
                        "state": "running",
                        "episode_id": episode.episode_id,
                        "profile": profile.name.value,
                        "local_detour": args.local_detour,
                        "dwb_goal_align_scale": (
                            detour_controller.dwb_config.goal_align_scale
                            if detour_controller is not None
                            else controller.config.goal_align_scale
                        ),
                        "local_detour_state": (
                            detour_policy.state.value
                            if detour_policy is not None
                            else "disabled"
                        ),
                        "local_waypoint_index": (
                            detour_controller.active_waypoint_index
                            if detour_controller is not None
                            else None
                        ),
                        "tick_id": snapshot.tick_id,
                        "max_ticks": args.max_ticks,
                        "elapsed_wall_s": perf_counter() - started,
                        "last_controller_elapsed_s": elapsed_s,
                        "pose": {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
                        "requested_twist": {
                            "linear": result.requested_twist.linear,
                            "angular": result.requested_twist.angular,
                        },
                        "status": result.status.value,
                        "no_safe_candidate": result.no_safe_candidate,
                    },
                )
            return result

    pipeline = simulate_dynamic_controller_pipeline(
        ProgressController(),
        initial_state=initial_state,
        reference_path=episode.reference_path,
        goal=episode.goal_pose,
        context_factory=factory,
        max_ticks=min(args.max_ticks, episode.tick_count),
        start_tick_id=start_tick_id,
        gate=resume_gate,
        simulated_computation_time_s=0.001,
    )
    evaluation = evaluate_dynamic_pipeline(
        replace(pipeline, controller_name="dynamic_dwa"),
        episode_id=episode.episode_id,
        expectation_category=episode.expectation_category.value,
        progressable=episode.progressable,
        reference_path=episode.reference_path,
        goal_pose=episode.goal_pose,
        actor_states_at=episode.actor_states_at,
        grid_snapshot_at=factory.grid_at,
        blocking_cleared_at_s=episode.blocking_cleared_at_s,
        oracle_spec=episode.oracle_spec,
    )
    moving_tick_ids = tuple(
        step.tick_id
        for step in pipeline.steps
        if step.safety_decision.motion_state is DynamicMotionState.MOVING
        and (
            abs(step.safety_decision.command.linear) > 1e-12
            or abs(step.safety_decision.command.angular) > 1e-12
        )
    )
    pose = pipeline.final_state.pose
    tick_trace = [
        {
            "tick_id": step.tick_id,
            "pose_before": {
                "x": step.robot_state_before.pose.x,
                "y": step.robot_state_before.pose.y,
                "yaw": step.robot_state_before.pose.yaw,
            },
            "requested_twist": {
                "linear": step.controller_result.requested_twist.linear,
                "angular": step.controller_result.requested_twist.angular,
            },
            "applied_twist": {
                "linear": step.safety_decision.command.linear,
                "angular": step.safety_decision.command.angular,
            },
            "controller_status": step.controller_result.status.value,
            "no_safe_candidate": step.controller_result.no_safe_candidate,
            "gate_overrode_controller": step.gate_overrode_controller,
            "motion_state": step.safety_decision.motion_state.value,
            "hold_reason": (
                None
                if step.safety_decision.primary_hold_reason is None
                else step.safety_decision.primary_hold_reason.value
            ),
        }
        for step in pipeline.steps
    ]
    _write_json(
        args.output,
        {
            "state": "complete",
            "episode_id": episode.episode_id,
            "profile": profile.name.value,
            "local_detour": args.local_detour,
            "dwb_goal_align_scale": (
                detour_controller.dwb_config.goal_align_scale
                if detour_controller is not None
                else controller.config.goal_align_scale
            ),
            "local_detour_state": (
                detour_policy.state.value if detour_policy is not None else "disabled"
            ),
            "local_waypoint_index": (
                detour_controller.active_waypoint_index
                if detour_controller is not None
                else None
            ),
            "local_reference_path": (
                [
                    {"x": item.x, "y": item.y, "yaw": item.yaw}
                    for item in detour_policy.active_reference_path
                ]
                if detour_policy is not None
                and detour_policy.active_reference_path is not None
                else None
            ),
            "max_ticks": args.max_ticks,
            "start_tick_id": start_tick_id,
            "resumed_from": (
                args.resume_from.name if args.resume_from is not None else None
            ),
            "executed_ticks": len(pipeline.steps),
            "elapsed_wall_s": perf_counter() - started,
            "controller_elapsed_s": {
                "minimum": min(controller_timings_s),
                "maximum": max(controller_timings_s),
                "mean": sum(controller_timings_s) / len(controller_timings_s),
            },
            "final_pose": {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
            "final_twist": {
                "linear": pipeline.final_state.twist.linear,
                "angular": pipeline.final_state.twist.angular,
            },
            "first_moving_tick": moving_tick_ids[0] if moving_tick_ids else None,
            "last_moving_tick": moving_tick_ids[-1] if moving_tick_ids else None,
            "moving_tick_count": len(moving_tick_ids),
            "completed": pipeline.completed,
            "pipeline_failure_reason": pipeline.failure_reason,
            "evidence_scope": (
                "continuous_from_episode_start"
                if start_tick_id == 0
                else "resumed_functional_segment"
            ),
            "hard_safety_passed": (
                evaluation.hard_safety.passed if start_tick_id == 0 else None
            ),
            "hard_safety_failures": (
                evaluation.hard_safety.failures if start_tick_id == 0 else ()
            ),
            "resumed_segment_evaluator_failures": (
                evaluation.hard_safety.failures if start_tick_id > 0 else ()
            ),
            "segment_static_collision_count": pipeline.static_collision_count,
            "segment_forbidden_entry_count": pipeline.forbidden_entry_count,
            "functional_qualified": evaluation.functional_qualified,
            "functional_failures": evaluation.functional_failures,
            "maximum_reference_deviation_m": (
                evaluation.metrics.maximum_reference_deviation_m
            ),
            "overtaking_observed": evaluation.metrics.overtaking_observed,
            "same_direction_overtaking_actor_ids": (
                evaluation.metrics.same_direction_overtaking_actor_ids
            ),
            "rejoin_observed": evaluation.metrics.rejoin_observed,
            "no_safe_candidate_count": pipeline.no_safe_candidate_count,
            "gate_override_count": pipeline.gate_override_count,
            "controller_stop_request_count": pipeline.controller_stop_request_count,
            "tick_trace": tick_trace,
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
