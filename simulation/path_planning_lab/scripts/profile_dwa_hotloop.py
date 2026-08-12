"""Profile or micro-benchmark the frozen five-case DWA qualification input set.

This utility is diagnostic only.  Its short default run is not the 500-call
wall-clock qualification and does not create a receipt or consume hidden data.
"""

from __future__ import annotations

import argparse
import cProfile
import json
from io import StringIO
from pstats import SortKey, Stats
from statistics import median
from time import perf_counter_ns

from hospital_path_lab.dwa_hotloop import CYTHON_DWA_HOTLOOP_AVAILABLE
from hospital_path_lab.dynamic_corpus import (
    generate_dynamic_corpus,
    generate_dynamic_v6_public_corpus,
)
from hospital_path_lab.dynamic_runner import _qualification_snapshot_cases
from hospital_path_lab.local_algorithms.dwa import DynamicDwaController


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--profile", action="store_true")
    args = parser.parse_args()
    if args.repetitions <= 0:
        parser.error("--repetitions must be positive")

    corpus = (*generate_dynamic_corpus(), *generate_dynamic_v6_public_corpus())
    cases = tuple(_qualification_snapshot_cases(corpus))
    if len(cases) != 5:
        raise RuntimeError(f"expected five qualification cases, received {len(cases)}")

    controller = DynamicDwaController()
    for _case_id, snapshot, _metadata in cases:
        controller.step(snapshot)

    timings_ms: list[float] = []

    def run() -> None:
        for _ in range(args.repetitions):
            for _case_id, snapshot, _metadata in cases:
                started_at = perf_counter_ns()
                controller.step(snapshot)
                timings_ms.append((perf_counter_ns() - started_at) / 1_000_000.0)

    profile_text: str | None = None
    if args.profile:
        profiler = cProfile.Profile()
        profiler.enable()
        run()
        profiler.disable()
        stream = StringIO()
        Stats(profiler, stream=stream).strip_dirs().sort_stats(SortKey.CUMULATIVE).print_stats(25)
        profile_text = stream.getvalue()
    else:
        run()

    ordered = sorted(timings_ms)
    payload = {
        "schema_version": "dwa-hotloop-diagnostic-v1",
        "cython_hotloop_available": CYTHON_DWA_HOTLOOP_AVAILABLE,
        "qualification_claim": False,
        "case_count": len(cases),
        "repetitions": args.repetitions,
        "sample_count": len(ordered),
        "deadline_ms": 50.0,
        "deadline_misses": sum(value > 50.0 for value in ordered),
        "p50_ms": median(ordered),
        "p95_ms": _percentile(ordered, 0.95),
        "maximum_ms": ordered[-1],
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    if profile_text is not None:
        print(profile_text)
    return 0


def _percentile(ordered: list[float], fraction: float) -> float:
    if not ordered:
        raise ValueError("percentile input must not be empty")
    index = fraction * (len(ordered) - 1)
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


if __name__ == "__main__":
    raise SystemExit(main())
