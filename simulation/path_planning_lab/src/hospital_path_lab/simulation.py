"""공통 가상 차체로 follower와 local controller를 직렬 실행한다."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import asdict, dataclass, replace
from json import dumps
from math import cos, hypot, isfinite, pi, sin, sqrt
from pathlib import Path
from statistics import fmean

from hospital_path_lab.collision import CollisionChecker
from hospital_path_lab.contracts import (
    GridSnapshot,
    LocalPlanner,
    PathFollower,
    PlanStatus,
    Pose2D,
    RobotState,
    Twist2D,
)
from hospital_path_lab.dynamic_actor import DynamicActorScenario, actor_state_at
from hospital_path_lab.dynamic_contracts import (
    DYNAMIC_CONTROL_FREQUENCY_HZ,
    DYNAMIC_CONTROL_PERIOD_S,
    ControllerCommandResult,
    DynamicAcceptedCommand,
    DynamicController,
    DynamicControllerInputFrame,
    DynamicGroundTruthFrame,
    DynamicMotionState,
    DynamicSafetyDecision,
    DynamicStateEvent,
    DynamicTrace,
    DynamicTraceMetadata,
    build_controller_snapshot,
    controller_result_to_proposal,
)
from hospital_path_lab.dynamic_safety import DynamicSafetyContext, DynamicSafetyGate
from hospital_path_lab.map_factory import canonical_content_hash
from hospital_path_lab.vehicle import VIRTUAL_DOLL_WHEELCHAIR_V0_1, VehicleProfile


@dataclass(frozen=True, slots=True)
class SimulationResult:
    component: str
    status: PlanStatus
    goal_reached: bool
    collision: bool
    poses: tuple[Pose2D, ...]
    commands: tuple[Twist2D, ...]
    elapsed_s: float
    minimum_clearance_m: float | None
    mean_tracking_error_m: float
    maximum_tracking_error_m: float
    jerk_rms_mps3: float
    final_goal_distance_m: float
    failure_reason: str | None = None


@dataclass(frozen=True, slots=True)
class DynamicLocalStepEvidence:
    """한 스냅샷에서 관측한 local planner의 simulation-only 증거."""

    step_index: int
    event_kind: str | None
    map_id: str
    map_revision: int
    mission_revision: int
    observation_revision: int
    input_content_hash: str
    planner_status: PlanStatus
    command: Twist2D
    pose_before: Pose2D
    pose_after: Pose2D
    collision: bool
    command_rejected: bool
    safe_stop_observed: bool
    recovery_observed: bool
    rejoin_observed: bool
    minimum_clearance_m: float
    tracking_error_m: float
    goal_progress_m: float
    no_path_streak: int
    no_progress_streak: int
    deadlock_observed: bool
    failure_reason: str | None = None


@dataclass(frozen=True, slots=True)
class DynamicLocalEvidence:
    """동적 스냅샷 열에 대한 local planner의 집계 증거.

    이 결과는 축소 Python 시뮬레이션의 관측일 뿐 제품 안전성의 근거가 아니다.
    """

    component: str
    simulation_only: bool
    steps: tuple[DynamicLocalStepEvidence, ...]
    final_state: RobotState
    collision_count: int
    rejected_command_count: int
    safe_stop_count: int
    no_path_count: int
    recovery_observed: bool
    path_deviation_observed: bool
    rejoin_observed: bool
    deadlock_observed: bool
    maximum_no_path_streak: int
    maximum_no_progress_streak: int
    minimum_clearance_m: float | None
    mean_tracking_error_m: float
    maximum_tracking_error_m: float
    commands_finite: bool
    metrics_finite: bool


@dataclass(frozen=True, slots=True)
class DynamicControllerPipelineStep:
    tick_id: int
    simulation_time_s: float
    controller_result: ControllerCommandResult
    safety_decision: DynamicSafetyDecision
    robot_state_before: RobotState
    robot_state_after: RobotState
    gate_overrode_controller: bool
    static_collision: bool
    forbidden_entry: bool


@dataclass(frozen=True, slots=True)
class DynamicControllerPipelineResult:
    controller_name: str
    simulation_only: bool
    status: PlanStatus
    completed: bool
    expected_hold_reached: bool
    final_state: RobotState
    steps: tuple[DynamicControllerPipelineStep, ...]
    static_collision_count: int
    forbidden_entry_count: int
    gate_override_count: int
    controller_stop_request_count: int
    no_safe_candidate_count: int
    failure_reason: str | None = None


def simulate_dynamic_actor_scenario(scenario: DynamicActorScenario) -> DynamicTrace:
    """20 Hz에서 open-loop Actor와 정지 로봇의 ground-truth trace를 만든다.

    Stage 1에서는 controller를 실행하지 않는다. controller-facing frame에는 로봇과
    reference path만 복사하며 Actor ground truth를 넣지 않는다.
    """

    truth_frames: list[DynamicGroundTruthFrame] = []
    controller_frames: list[DynamicControllerInputFrame] = []
    for tick_id in range(scenario.tick_count + 1):
        simulation_time_s = tick_id * DYNAMIC_CONTROL_PERIOD_S
        truth = DynamicGroundTruthFrame(
            episode_id=scenario.episode_id,
            seed=scenario.seed,
            tick_id=tick_id,
            simulation_time_s=simulation_time_s,
            robot_state=scenario.robot_initial_state,
            actors=(actor_state_at(scenario, simulation_time_s),),
            map_revision=scenario.map_revision,
            mission_revision=scenario.mission_revision,
        )
        truth_frames.append(truth)
        controller_frames.append(
            DynamicControllerInputFrame(
                tick_id=tick_id,
                simulation_time_s=simulation_time_s,
                robot_state=truth.robot_state,
                reference_path=scenario.reference_path,
                map_revision=scenario.map_revision,
                mission_revision=scenario.mission_revision,
            )
        )

    accepted_commands = tuple(
        DynamicAcceptedCommand(
            source_tick_id=tick_id,
            applied_tick_id=tick_id + 1,
            command=Twist2D(),
        )
        for tick_id in range(scenario.tick_count)
    )
    crossing_y = scenario.reference_path[0].y
    crossing_tick = next(
        frame.tick_id for frame in truth_frames if frame.actors[0].position.y >= crossing_y
    )
    events = (
        DynamicStateEvent(0, 0.0, "episode_started", scenario.actor_id),
        DynamicStateEvent(
            crossing_tick,
            crossing_tick * DYNAMIC_CONTROL_PERIOD_S,
            "actor_crossed_reference",
            scenario.actor_id,
        ),
        DynamicStateEvent(
            scenario.tick_count,
            scenario.tick_count * DYNAMIC_CONTROL_PERIOD_S,
            "episode_finished",
            scenario.actor_id,
        ),
    )
    return DynamicTrace(
        metadata=DynamicTraceMetadata(
            schema_version=scenario.schema_version,
            generator_version=scenario.generator_version,
            episode_id=scenario.episode_id,
            seed=scenario.seed,
            simulation_only=True,
            world_content_hash=scenario.content_hash,
            control_frequency_hz=DYNAMIC_CONTROL_FREQUENCY_HZ,
            tick_count=scenario.tick_count,
            map_revision=scenario.map_revision,
            mission_revision=scenario.mission_revision,
        ),
        reference_path=scenario.reference_path,
        ground_truth_frames=tuple(truth_frames),
        controller_input_frames=tuple(controller_frames),
        accepted_commands=accepted_commands,
        state_events=events,
    )


def dynamic_trace_content_hash(trace: DynamicTrace) -> str:
    """Wall-clock 값을 포함하지 않는 trace 의미 내용의 SHA-256."""

    return canonical_content_hash(trace)


def dynamic_artifact_stem(trace: DynamicTrace) -> str:
    """episode ID와 seed를 포함한 안전한 산출물 파일 stem을 반환한다."""

    safe_episode_id = "".join(
        character if character.isalnum() or character in {"-", "_"} else "_"
        for character in trace.metadata.episode_id
    )
    return f"{safe_episode_id}_seed_{trace.metadata.seed}"


def save_dynamic_trace_json(trace: DynamicTrace, output_path: str | Path) -> Path:
    """Trace와 의미 content hash를 결정론적 UTF-8 JSON으로 저장한다."""

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = asdict(trace)
    payload["content_hash"] = dynamic_trace_content_hash(trace)
    output.write_text(
        dumps(
            payload,
            ensure_ascii=False,
            sort_keys=True,
            indent=2,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    return output


def simulate_dynamic_controller_pipeline(
    controller: DynamicController,
    *,
    initial_state: RobotState,
    reference_path: tuple[Pose2D, ...],
    goal: Pose2D,
    context_factory: Callable[
        [int, float, RobotState, DynamicSafetyGate],
        DynamicSafetyContext,
    ],
    profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    max_ticks: int = 600,
    simulated_computation_time_s: float = 0.001,
    stop_when_holding: bool = False,
    goal_tolerance_m: float = 0.05,
) -> DynamicControllerPipelineResult:
    """동일 gate·20 Hz 적분기로 PP 또는 DWA의 Stage 4 폐루프를 실행한다."""

    if max_ticks <= 0 or not reference_path:
        raise ValueError("dynamic pipeline requires positive ticks and a reference path")
    if not profile.simulation_only:
        raise ValueError("dynamic pipeline requires a simulation-only vehicle profile")
    if not 0.0 <= simulated_computation_time_s <= 0.050:
        raise ValueError("deterministic computation time must be inside the 50 ms deadline")

    gate = DynamicSafetyGate(profile=profile)
    state = initial_state
    steps: list[DynamicControllerPipelineStep] = []
    static_collision_count = 0
    forbidden_entry_count = 0
    no_safe_candidate_count = 0
    expected_hold_reached = False

    for tick_id in range(max_ticks):
        simulation_time_s = tick_id * profile.control_period_s
        context = context_factory(tick_id, simulation_time_s, state, gate)
        if context.tick_id != tick_id or not isfinite(context.simulation_time_s):
            raise ValueError("context factory returned a mismatched tick")
        if abs(context.simulation_time_s - simulation_time_s) > 1e-12:
            raise ValueError("context factory returned a mismatched simulation time")
        goal_reached = _distance(state.pose, goal) <= goal_tolerance_m
        if goal_reached and not context.goal_reached:
            context = replace(context, goal_reached=True)

        controller_snapshot = build_controller_snapshot(
            tick_id=tick_id,
            simulation_time_s=simulation_time_s,
            mission_id=context.mission_id,
            robot_state=state,
            goal_pose=goal,
            reference_path=reference_path,
            static_grid_snapshot=context.grid_snapshot,
            validated_observation=context.observation_snapshot,
            actor_tubes=context.prediction_set,
            vehicle_profile=profile,
        )
        controller_result = controller.step(controller_snapshot)
        if controller_result.source_tick_id != tick_id:
            raise ValueError("controller returned a result for a different tick")
        proposal = controller_result_to_proposal(
            controller_result,
            computation_time_s=simulated_computation_time_s,
        )
        decision = gate.step(proposal, robot_state=state, context=context)

        # Stage 3 current-motion sweep와 동일하게 현재 twist로 한 tick의 pose를 적분한 뒤
        # gate 출력을 다음 tick의 twist로 저장한다.
        next_pose = _integrate(state.pose, state.twist, profile.control_period_s)
        next_state = RobotState(next_pose, decision.command)
        checker = CollisionChecker(
            context.grid_snapshot.grid,
            profile,
            forbidden_cells=context.grid_snapshot.forbidden_cells,
        )
        static_collision = checker.clearance(next_pose) <= 0.0
        forbidden_entry = checker.pose_enters_forbidden(next_pose)
        static_collision_count += int(static_collision)
        forbidden_entry_count += int(forbidden_entry)
        no_safe_candidate_count += int(controller_result.no_safe_candidate)
        gate_overrode = decision.command != controller_result.requested_twist
        steps.append(
            DynamicControllerPipelineStep(
                tick_id=tick_id,
                simulation_time_s=simulation_time_s,
                controller_result=controller_result,
                safety_decision=decision,
                robot_state_before=state,
                robot_state_after=next_state,
                gate_overrode_controller=gate_overrode,
                static_collision=static_collision,
                forbidden_entry=forbidden_entry,
            )
        )
        state = next_state

        if static_collision or forbidden_entry:
            break
        if decision.motion_state is DynamicMotionState.COMPLETED:
            break
        if stop_when_holding and decision.motion_state is DynamicMotionState.HOLDING:
            expected_hold_reached = True
            break

    completed = bool(
        steps and steps[-1].safety_decision.motion_state is DynamicMotionState.COMPLETED
    )
    if static_collision_count:
        failure_reason = "static_collision"
    elif forbidden_entry_count:
        failure_reason = "forbidden_entry"
    elif completed or expected_hold_reached:
        failure_reason = None
    else:
        failure_reason = "pipeline_timeout"
    return DynamicControllerPipelineResult(
        controller_name=controller.name,
        simulation_only=True,
        status=(
            PlanStatus.FOUND
            if completed
            else PlanStatus.NO_PATH
        ),
        completed=completed,
        expected_hold_reached=expected_hold_reached,
        final_state=state,
        steps=tuple(steps),
        static_collision_count=static_collision_count,
        forbidden_entry_count=forbidden_entry_count,
        gate_override_count=gate.counters.gate_overrides,
        controller_stop_request_count=gate.counters.controller_stop_requests,
        no_safe_candidate_count=no_safe_candidate_count,
        failure_reason=failure_reason,
    )


def simulate_follower(
    follower: PathFollower,
    path: tuple[Pose2D, ...],
    snapshot: GridSnapshot,
    initial_state: RobotState,
    goal: Pose2D,
    *,
    profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    max_time_s: float = 30.0,
    goal_tolerance_m: float = 0.05,
) -> SimulationResult:
    if not path or max_time_s <= 0:
        return _failed_simulation(
            follower.name, initial_state.pose, goal, "invalid_simulation_input"
        )

    state = initial_state
    collision_checker = CollisionChecker(
        snapshot.grid,
        profile,
        forbidden_cells=snapshot.forbidden_cells,
    )
    poses = [state.pose]
    commands: list[Twist2D] = []
    clearances = [collision_checker.conservative_clearance(state.pose)]
    tracking_errors = [_nearest_path_distance(state.pose, path)]
    step_count = max(1, int(max_time_s / profile.control_period_s))
    failure_reason: str | None = None
    collision = clearances[-1] <= 0.0

    for _ in range(step_count):
        if collision or (
            _distance(state.pose, goal) <= goal_tolerance_m
            and _twist_is_stopped(state.twist)
        ):
            break
        output = follower.step(path, state, snapshot.metadata)
        if output.status is not PlanStatus.FOUND:
            failure_reason = output.failure_reason or output.status.value
            break
        command = _bounded_command(output.command, profile)
        state = RobotState(
            pose=_integrate(state.pose, command, profile.control_period_s),
            twist=command,
        )
        commands.append(command)
        poses.append(state.pose)
        clearances.append(collision_checker.conservative_clearance(state.pose))
        tracking_errors.append(_nearest_path_distance(state.pose, path))
        collision = clearances[-1] <= 0.0

    goal_distance = _distance(state.pose, goal)
    goal_reached = (
        goal_distance <= goal_tolerance_m
        and _twist_is_stopped(state.twist)
        and not collision
    )
    status = PlanStatus.FOUND if goal_reached else PlanStatus.NO_PATH
    if collision:
        failure_reason = "collision"
    elif not goal_reached and failure_reason is None:
        failure_reason = "goal_not_reached_before_timeout"
    return SimulationResult(
        component=follower.name,
        status=status,
        goal_reached=goal_reached,
        collision=collision,
        poses=tuple(poses),
        commands=tuple(commands),
        elapsed_s=len(commands) * profile.control_period_s,
        minimum_clearance_m=min(clearances) if clearances else None,
        mean_tracking_error_m=fmean(tracking_errors),
        maximum_tracking_error_m=max(tracking_errors),
        jerk_rms_mps3=_jerk_rms(commands, profile.control_period_s),
        final_goal_distance_m=goal_distance,
        failure_reason=failure_reason,
    )


def simulate_local_controller(
    planner: LocalPlanner,
    reference_path: tuple[Pose2D, ...],
    snapshot: GridSnapshot,
    initial_state: RobotState,
    goal: Pose2D,
    *,
    profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    max_time_s: float = 30.0,
    goal_tolerance_m: float = 0.05,
) -> SimulationResult:
    state = initial_state
    collision_checker = CollisionChecker(
        snapshot.grid,
        profile,
        forbidden_cells=snapshot.forbidden_cells,
    )
    poses = [state.pose]
    commands: list[Twist2D] = []
    clearances = [collision_checker.conservative_clearance(state.pose)]
    tracking_errors = [_nearest_path_distance(state.pose, reference_path)]
    collision = clearances[-1] <= 0.0
    failure_reason: str | None = None
    step_count = max(1, int(max_time_s / profile.control_period_s))

    for _ in range(step_count):
        if collision or (
            _distance(state.pose, goal) <= goal_tolerance_m
            and _twist_is_stopped(state.twist)
        ):
            break
        result = planner.plan(snapshot, reference_path, state, goal)
        if result.status is not PlanStatus.FOUND or len(result.trajectory) < 2:
            failure_reason = result.failure_reason or "no_safe_local_trajectory"
            break
        command = _bounded_command(result.trajectory[1].twist, profile)
        state = RobotState(
            pose=_integrate(state.pose, command, profile.control_period_s),
            twist=command,
        )
        commands.append(command)
        poses.append(state.pose)
        clearances.append(collision_checker.conservative_clearance(state.pose))
        tracking_errors.append(_nearest_path_distance(state.pose, reference_path))
        collision = clearances[-1] <= 0.0

    goal_distance = _distance(state.pose, goal)
    goal_reached = (
        goal_distance <= goal_tolerance_m
        and _twist_is_stopped(state.twist)
        and not collision
    )
    status = PlanStatus.FOUND if goal_reached else PlanStatus.NO_PATH
    if collision:
        failure_reason = "collision"
    elif not goal_reached and failure_reason is None:
        failure_reason = "goal_not_reached_before_timeout"
    return SimulationResult(
        component=planner.name,
        status=status,
        goal_reached=goal_reached,
        collision=collision,
        poses=tuple(poses),
        commands=tuple(commands),
        elapsed_s=len(commands) * profile.control_period_s,
        minimum_clearance_m=min(clearances) if clearances else None,
        mean_tracking_error_m=fmean(tracking_errors),
        maximum_tracking_error_m=max(tracking_errors),
        jerk_rms_mps3=_jerk_rms(commands, profile.control_period_s),
        final_goal_distance_m=goal_distance,
        failure_reason=failure_reason,
    )


def simulate_dynamic_local_evidence(
    planner: LocalPlanner,
    reference_path: tuple[Pose2D, ...],
    snapshots: tuple[GridSnapshot, ...],
    initial_state: RobotState,
    goal: Pose2D,
    *,
    event_kinds: tuple[str | None, ...] | None = None,
    profile: VehicleProfile = VIRTUAL_DOLL_WHEELCHAIR_V0_1,
    deadlock_threshold_steps: int = 3,
    progress_tolerance_m: float = 1e-6,
    rejoin_tolerance_m: float = 0.10,
) -> DynamicLocalEvidence:
    """상태를 유지하며 순차 grid snapshot마다 local planner를 한 번 실행한다.

    실행할 수 없는 결과나 충돌 가능한 첫 명령은 0속도 명령으로 바꾼다. 장애물
    제거는 ``event_kinds``의 ``obstacle_remove`` 계열 값 또는 점유 cell 감소로
    식별한다. 반환값은 오직 ``simulation_only`` 회귀시험 증거다.
    """

    if not reference_path:
        raise ValueError("reference_path must not be empty")
    if not snapshots:
        raise ValueError("snapshots must not be empty")
    if event_kinds is not None and len(event_kinds) != len(snapshots):
        raise ValueError("event_kinds must have the same length as snapshots")
    if deadlock_threshold_steps <= 0:
        raise ValueError("deadlock_threshold_steps must be positive")
    if progress_tolerance_m < 0 or rejoin_tolerance_m < 0:
        raise ValueError("simulation tolerances must not be negative")

    labels = event_kinds or (None,) * len(snapshots)
    state = initial_state
    evidence_steps: list[DynamicLocalStepEvidence] = []
    no_path_streak = 0
    no_progress_streak = 0
    maximum_no_path_streak = 0
    maximum_no_progress_streak = 0
    recovery_armed = False
    removal_seen_after_stop = False
    recovery_observed = False
    path_deviation_observed = False
    rejoin_observed = False
    previous_occupied_count: int | None = None

    for step_index, (snapshot, event_kind) in enumerate(
        zip(snapshots, labels, strict=True)
    ):
        checker = CollisionChecker(
            snapshot.grid,
            profile,
            forbidden_cells=snapshot.forbidden_cells,
        )
        pose_before = state.pose
        clearance_before = checker.conservative_clearance(pose_before)
        collision_before = clearance_before <= 0.0
        goal_distance_before = _distance(pose_before, goal)
        plan = planner.plan(snapshot, reference_path, state, goal)

        provenance_matches = (
            plan.map_id == snapshot.metadata.map_id
            and plan.map_revision == snapshot.metadata.map_revision
            and plan.mission_revision == snapshot.metadata.mission_revision
            and plan.observation_revision == snapshot.metadata.observation_revision
            and plan.input_content_hash == snapshot.metadata.content_hash
        )
        command_rejected = not provenance_matches
        rejection_reason = "planner_result_provenance_mismatch" if command_rejected else None
        proposed_command = Twist2D()
        if plan.status is PlanStatus.FOUND and len(plan.trajectory) >= 2:
            proposed_command = plan.trajectory[1].twist
        elif plan.status is PlanStatus.FOUND:
            command_rejected = True
            rejection_reason = "found_result_has_no_executable_command"

        if not all(
            isfinite(value) for value in (proposed_command.linear, proposed_command.angular)
        ):
            command_rejected = True
            rejection_reason = "planner_command_non_finite"

        bounded_command = (
            _bounded_command(proposed_command, profile)
            if not command_rejected and plan.status is PlanStatus.FOUND
            else Twist2D()
        )
        proposed_pose = _integrate(pose_before, bounded_command, profile.control_period_s)
        if (
            collision_before
            or not checker.conservative_path_is_collision_free(
                (pose_before, proposed_pose)
            )
        ):
            command_rejected = True
            rejection_reason = (
                "initial_pose_in_collision"
                if collision_before
                else "planner_command_would_collide"
            )

        command = (
            Twist2D()
            if command_rejected or plan.status is not PlanStatus.FOUND
            else bounded_command
        )
        pose_after = _integrate(pose_before, command, profile.control_period_s)
        state = RobotState(pose=pose_after, twist=command)
        clearance_after = checker.conservative_clearance(pose_after)
        collision = clearance_after <= 0.0
        tracking_error = _nearest_path_distance(pose_after, reference_path)
        if tracking_error > rejoin_tolerance_m:
            path_deviation_observed = True
        goal_progress = goal_distance_before - _distance(pose_after, goal)
        made_progress = _distance(pose_before, pose_after) > progress_tolerance_m
        no_executable_path = plan.status is not PlanStatus.FOUND or command_rejected
        no_path_streak = no_path_streak + 1 if no_executable_path else 0
        no_progress_streak = no_progress_streak + 1 if not made_progress else 0
        maximum_no_path_streak = max(maximum_no_path_streak, no_path_streak)
        maximum_no_progress_streak = max(maximum_no_progress_streak, no_progress_streak)
        deadlock = (
            no_path_streak >= deadlock_threshold_steps
            or no_progress_streak >= deadlock_threshold_steps
        )
        safe_stop = no_executable_path and _twist_is_stopped(command)
        if safe_stop:
            recovery_armed = True

        occupied_count = int(snapshot.grid.occupancy.sum()) + len(snapshot.forbidden_cells)
        normalized_event = (event_kind or "").strip().lower()
        removal_event = normalized_event in {
            "obstacle_remove",
            "remove_obstacle",
            "obstacle_removed",
            "remove",
        }
        occupancy_decreased = (
            previous_occupied_count is not None
            and occupied_count < previous_occupied_count
        )
        if recovery_armed and (removal_event or occupancy_decreased):
            removal_seen_after_stop = True
        previous_occupied_count = occupied_count

        step_recovery = (
            recovery_armed
            and removal_seen_after_stop
            and not no_executable_path
            and command.linear > progress_tolerance_m
        )
        if step_recovery:
            recovery_observed = True
        step_rejoin = (
            path_deviation_observed
            and
            (recovery_observed or step_recovery)
            and made_progress
            and tracking_error <= rejoin_tolerance_m
        )
        if step_rejoin:
            rejoin_observed = True

        failure_reason = rejection_reason or plan.failure_reason
        evidence_steps.append(
            DynamicLocalStepEvidence(
                step_index=step_index,
                event_kind=event_kind,
                map_id=snapshot.metadata.map_id,
                map_revision=snapshot.metadata.map_revision,
                mission_revision=snapshot.metadata.mission_revision,
                observation_revision=snapshot.metadata.observation_revision,
                input_content_hash=snapshot.metadata.content_hash,
                planner_status=plan.status,
                command=command,
                pose_before=pose_before,
                pose_after=pose_after,
                collision=collision,
                command_rejected=command_rejected,
                safe_stop_observed=safe_stop,
                recovery_observed=step_recovery,
                rejoin_observed=step_rejoin,
                minimum_clearance_m=clearance_after,
                tracking_error_m=tracking_error,
                goal_progress_m=goal_progress,
                no_path_streak=no_path_streak,
                no_progress_streak=no_progress_streak,
                deadlock_observed=deadlock,
                failure_reason=failure_reason,
            )
        )

    clearances = tuple(step.minimum_clearance_m for step in evidence_steps)
    tracking_errors = tuple(step.tracking_error_m for step in evidence_steps)
    commands_finite = all(
        isfinite(value)
        for step in evidence_steps
        for value in (step.command.linear, step.command.angular)
    )
    metrics_finite = all(
        isfinite(value)
        for step in evidence_steps
        for value in (
            step.minimum_clearance_m,
            step.tracking_error_m,
            step.goal_progress_m,
            step.pose_after.x,
            step.pose_after.y,
            step.pose_after.yaw,
        )
    )
    return DynamicLocalEvidence(
        component=planner.name,
        simulation_only=True,
        steps=tuple(evidence_steps),
        final_state=state,
        collision_count=sum(step.collision for step in evidence_steps),
        rejected_command_count=sum(step.command_rejected for step in evidence_steps),
        safe_stop_count=sum(step.safe_stop_observed for step in evidence_steps),
        no_path_count=sum(
            step.planner_status is not PlanStatus.FOUND for step in evidence_steps
        ),
        recovery_observed=recovery_observed,
        path_deviation_observed=path_deviation_observed,
        rejoin_observed=rejoin_observed,
        deadlock_observed=any(step.deadlock_observed for step in evidence_steps),
        maximum_no_path_streak=maximum_no_path_streak,
        maximum_no_progress_streak=maximum_no_progress_streak,
        minimum_clearance_m=min(clearances) if clearances else None,
        mean_tracking_error_m=fmean(tracking_errors),
        maximum_tracking_error_m=max(tracking_errors),
        commands_finite=commands_finite,
        metrics_finite=metrics_finite,
    )


def _integrate(pose: Pose2D, command: Twist2D, dt: float) -> Pose2D:
    yaw = _normalize_angle(pose.yaw + command.angular * dt)
    return Pose2D(
        x=pose.x + command.linear * cos(pose.yaw) * dt,
        y=pose.y + command.linear * sin(pose.yaw) * dt,
        yaw=yaw,
    )


def _bounded_command(command: Twist2D, profile: VehicleProfile) -> Twist2D:
    linear = max(
        -profile.max_reverse_speed_mps,
        min(profile.max_forward_speed_mps, command.linear),
    )
    angular = max(
        -profile.max_angular_speed_radps,
        min(profile.max_angular_speed_radps, command.angular),
    )
    return Twist2D(
        linear=0.0 if abs(linear) <= 1e-12 else linear,
        angular=0.0 if abs(angular) <= 1e-12 else angular,
    )


def _nearest_path_distance(pose: Pose2D, path: tuple[Pose2D, ...]) -> float:
    if not path:
        return float("inf")
    if len(path) == 1:
        return _distance(pose, path[0])
    return min(
        _point_to_segment_distance(pose, source, target)
        for source, target in zip(path[:-1], path[1:], strict=True)
    )


def _point_to_segment_distance(pose: Pose2D, source: Pose2D, target: Pose2D) -> float:
    dx = target.x - source.x
    dy = target.y - source.y
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return _distance(pose, source)
    fraction = min(
        1.0,
        max(
            0.0,
            ((pose.x - source.x) * dx + (pose.y - source.y) * dy) / length_squared,
        ),
    )
    closest = Pose2D(source.x + fraction * dx, source.y + fraction * dy)
    return _distance(pose, closest)


def _distance(a: Pose2D, b: Pose2D) -> float:
    return hypot(a.x - b.x, a.y - b.y)


def _normalize_angle(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


def _twist_is_stopped(twist: Twist2D, *, tolerance: float = 1e-9) -> bool:
    return abs(twist.linear) <= tolerance and abs(twist.angular) <= tolerance


def _jerk_rms(commands: list[Twist2D], dt: float) -> float:
    if len(commands) < 3:
        return 0.0
    accelerations = [
        (current.linear - previous.linear) / dt
        for previous, current in zip(commands, commands[1:], strict=False)
    ]
    jerks = [
        (current - previous) / dt
        for previous, current in zip(accelerations, accelerations[1:], strict=False)
    ]
    return sqrt(fmean(value * value for value in jerks)) if jerks else 0.0


def _failed_simulation(
    component: str,
    start: Pose2D,
    goal: Pose2D,
    reason: str,
) -> SimulationResult:
    return SimulationResult(
        component=component,
        status=PlanStatus.INVALID_INPUT,
        goal_reached=False,
        collision=False,
        poses=(start,),
        commands=(),
        elapsed_s=0.0,
        minimum_clearance_m=None,
        mean_tracking_error_m=0.0,
        maximum_tracking_error_m=0.0,
        jerk_rms_mps3=0.0,
        final_goal_distance_m=_distance(start, goal),
        failure_reason=reason,
    )
