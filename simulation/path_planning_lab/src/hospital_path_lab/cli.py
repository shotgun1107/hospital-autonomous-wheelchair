"""경로 실험실 명령행 진입점."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from hospital_path_lab.benchmark import benchmark_suite
from hospital_path_lab.planners import AStarPlanner, DijkstraPlanner
from hospital_path_lab.registry import algorithm_manifest
from hospital_path_lab.safety import AutomaticResumeGate
from hospital_path_lab.scenario import load_scenario_suite
from hospital_path_lab.visualization import save_route_plot

LAB_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENARIO = LAB_ROOT / "scenarios" / "hospital_corridors.yaml"
DEFAULT_OUTPUT = LAB_ROOT / "outputs"


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.command == "benchmark":
        return _run_benchmark(args)
    if args.command == "safety-demo":
        return _run_safety_demo()
    if args.command == "list-algorithms":
        return _run_list_algorithms()
    if args.command == "experiment":
        return _run_experiment(args)
    if args.command == "dynamic-public-qualification":
        return _run_dynamic_public_qualification(args)
    parser.error("알 수 없는 명령입니다.")
    return 2


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="병원 자율휠체어 경로 알고리즘 실험실")
    subparsers = parser.add_subparsers(dest="command", required=True)

    benchmark = subparsers.add_parser("benchmark", help="기준 시나리오 반복 비교")
    benchmark.add_argument("--scenario", type=Path, default=DEFAULT_SCENARIO)
    benchmark.add_argument("--repeats", type=int, default=100)
    benchmark.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    subparsers.add_parser("safety-demo", help="안전정지 뒤 자동 재개 게이트 데모")
    subparsers.add_parser("list-algorithms", help="역할별 구현·후속 알고리즘 목록")
    experiment = subparsers.add_parser(
        "experiment", help="생성 corpus 전체와 숨김 seed를 직렬 평가"
    )
    experiment.add_argument("--base-seed", type=int, default=20_260_810)
    experiment.add_argument("--hidden-seed", type=int, default=91_260_810)
    experiment.add_argument(
        "--regression-input-dir",
        type=Path,
        default=None,
        help="이전 실행의 regression_candidates를 다음 회차 입력으로 승격",
    )
    experiment.add_argument(
        "--regression-limit",
        type=int,
        default=None,
        help="승격할 검증된 회귀 후보 수 상한",
    )
    experiment.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT / "experiment")
    public_dynamic = subparsers.add_parser(
        "dynamic-public-qualification",
        help="run the v6 public gates without accepting or generating a hidden seed",
    )
    public_dynamic.add_argument("--base-seed", type=int, default=20_260_811)
    public_dynamic.add_argument("--simulation-workers", type=int, default=None)
    public_dynamic.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT / "dynamic-public-qualification",
    )
    return parser


def _run_benchmark(args: argparse.Namespace) -> int:
    suite = load_scenario_suite(args.scenario)
    planners = [DijkstraPlanner(), AStarPlanner()]
    records = benchmark_suite(suite, planners, repeats=args.repeats)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "global_benchmark.json"
    json_path.write_text(
        json.dumps([record.as_json_dict() for record in records], ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    for case in suite.cases:
        graph = suite.graph_for(case)
        results = [planner.plan(graph, case.start, case.goal) for planner in planners]
        save_route_plot(
            graph,
            results,
            args.output_dir / f"{case.name}.png",
            title=case.name,
        )

    failed = [record for record in records if not record.expected_result_matched]
    print(f"scenario={len(suite.cases)}, records={len(records)}, failed={len(failed)}")
    print(f"json={json_path}")
    return 1 if failed else 0


def _run_safety_demo() -> int:
    gate = AutomaticResumeGate()
    gate.hazard_detected()
    gate.confirm_stop()
    gate.hazard_cleared()
    denied_before_checks = not gate.try_automatic_resume()
    gate.record_path_revalidation(original_path_safe=True)
    gate.revalidate_resume_instruction()
    gate.authorize_local_safety()
    resumed_after_checks = gate.try_automatic_resume()

    print(f"denied_before_checks={denied_before_checks}")
    print(f"resumed_after_checks={resumed_after_checks}")
    for event in gate.events:
        print(f"- {event}")
    return 0 if denied_before_checks and resumed_after_checks else 1


def _run_list_algorithms() -> int:
    for item in algorithm_manifest():
        print(f"{item['role']}\t{item['name']}\t{item['implementation_status']}")
    return 0


def _run_experiment(args: argparse.Namespace) -> int:
    from hospital_path_lab.experiment_runner import ExperimentConfig, run_experiment

    result = run_experiment(
        args.output_dir,
        ExperimentConfig(
            base_seed=args.base_seed,
            hidden_seed=args.hidden_seed,
            regression_input_dir=(
                str(args.regression_input_dir)
                if args.regression_input_dir is not None
                else None
            ),
            regression_input_limit=args.regression_limit,
        ),
    )
    print(f"cases={result.case_count}, hard_failures={len(result.hard_failures)}")
    print(f"results={result.results_path}")
    print(f"pareto={result.pareto_path}")
    print(f"summary={result.summary_path}")
    return 1 if result.hard_failures else 0


def _run_dynamic_public_qualification(args: argparse.Namespace) -> int:
    from hospital_path_lab.dynamic_runner import (
        DynamicPublicQualificationConfig,
        run_dynamic_public_qualification,
    )

    result = run_dynamic_public_qualification(
        args.output_dir,
        DynamicPublicQualificationConfig(
            base_seed=args.base_seed,
            simulation_workers=args.simulation_workers,
        ),
    )
    print(
        f"public_runs={result.public_run_count}, passed={result.passed}, "
        f"simulation_workers={result.simulation_worker_count}"
    )
    print(f"gate={result.gate_path}")
    print(f"report={result.report_path}")
    if result.receipt_path is not None:
        print(f"receipt={result.receipt_path}")
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
