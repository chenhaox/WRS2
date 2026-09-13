"""Compare readability refactors with unchanged physical inputs and solver budgets.

Run directly, or set LABEL to "before"/"after". Reports live outside examples.
Repeated calls create fresh evaluators; imported libraries stay warm.
"""

from pathlib import Path
import json
import sys
from time import perf_counter
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import check_equilibrium, save_report
from examples.assembly._shared.quality import compute as quality_case
from examples.assembly._shared.sequence import compute_plan
from examples.assembly._shared.stability_cases import make_case, case_config
from examples.assembly._shared.robot import compute as robot_case

LABEL = "current"
BASELINE_LABEL = "before"
REPEATS = 3
ROBOT = True
OUTPUT_DIR = ROOT / "benchmark_results" / "assembly" / "readability"


def measure(query, summarize, repeats=REPEATS):
    times = []
    summaries = []
    for _ in range(repeats):
        start = perf_counter()
        result = query()
        times.append(perf_counter() - start)
        summaries.append(summarize(result))
    assert all(item == summaries[0] for item in summaries), "Nondeterministic result"
    return dict(seconds=times, median_s=float(np.median(times)), outcome=summaries[0])


def sequence_summary(data):
    plan, replay = data[3:]
    return dict(
        status=plan.status,
        replay=replay["status"],
        order=[step.part_id for step in plan.assembly_steps],
        supports=[(step.supports_before, step.supports_after) for step in plan.assembly_steps],
        frames=[len(step.assembly_poses) for step in plan.assembly_steps],
        expansions=plan.diagnostics["expansions"],
    )


def quality_summary(data):
    result = data[3]
    return dict(
        status=result.status,
        score=round(result.score, 10),
        order=[step.part_id for step in result.plan.assembly_steps],
        supports=[
            (step.supports_before, step.supports_after) for step in result.plan.assembly_steps
        ],
        qualities=[
            (round(q.stability, 9), q.graspability, q.assemblability) for q in result.qualities
        ],
        optimality=result.diagnostics["optimality"],
        expansions=result.diagnostics["expansions"],
        pruned=result.diagnostics["pruned"],
        stability_states=result.diagnostics["stability_states"],
        transition_cache_size=result.diagnostics["transition_cache_size"],
    )


def compare_reports(before, after):
    """Require identical physical results/work counts before comparing elapsed time."""
    if set(before["rows"]) != set(after["rows"]):
        raise ValueError("Use the same benchmark cases and ROBOT setting for both reports")
    comparison = {}
    for name, row in after["rows"].items():
        baseline = before["rows"][name]
        if baseline["outcome"] != row["outcome"]:
            raise AssertionError(f"Results or search/trajectory work changed: {name}")
        comparison[name] = dict(
            outcome_matches=True,
            before_s=baseline["median_s"],
            after_s=row["median_s"],
            time_ratio=row["median_s"] / baseline["median_s"],
        )
    return comparison


def run(label=LABEL):
    rows = {}
    assembly, state, graph = make_case("bridge")
    rows["equilibrium_bridge"] = measure(
        lambda: check_equilibrium(assembly, state, graph, config=case_config("bridge")),
        lambda r: dict(
            status=r.status,
            robust=r.robustness_status,
            sites=len(r.force_sites),
            variables=r.diagnostics["force_variables"],
        ),
        repeats=15,
    )
    for name in ("stack", "gantry"):
        rows["sequence_" + name] = measure(lambda: compute_plan(name, "dfs", 100), sequence_summary)
    for name in ("counterweight", "bridge"):
        rows["quality_" + name] = measure(
            lambda: quality_case(name, "cuda", 300, True), quality_summary
        )
    if ROBOT:
        for auxiliary in (False, True):
            rows["robot_dual" if auxiliary else "robot_single"] = measure(
                lambda: robot_case(auxiliary),
                lambda data: dict(
                    status=data[2].status,
                    frames=len(data[2].frames),
                    events=[e["kind"] for e in data[2].events],
                    collision_queries=data[2].diagnostics["collision_queries"],
                ),
                repeats=1,
            )
    report = dict(
        label=label,
        python=sys.executable,
        rows=rows,
        scope="fresh evaluators, fixed inputs; imports excluded; first CUDA use may initialize runtime",
    )
    baseline_path = OUTPUT_DIR / (BASELINE_LABEL + ".json")
    if label != BASELINE_LABEL and baseline_path.exists():
        before = json.loads(baseline_path.read_text(encoding="utf-8"))
        report["comparison"] = compare_reports(before, report)
    save_report(report, OUTPUT_DIR / (label + ".json"))
    for name, row in rows.items():
        print(name, round(row["median_s"], 4), "s", row["outcome"]["status"], flush=True)
    return report


if __name__ == "__main__":
    run()
