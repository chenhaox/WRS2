"""九类真实网格接触、方向和可装配性分数的批量检查。"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import save_report, score_assemblability
from examples.assembly._shared.direction_cases import CASES
from examples.assembly._shared.directions import compute

SAMPLES = 6500
OUTPUT_DIR = ROOT / "benchmark_results" / "assembly" / "nine_directions"


def main():
    summary = []
    for key in CASES:
        assembly, state, contacts, results = compute(key, SAMPLES)
        record = dict(case=key, title=CASES[key][0], contacts=contacts, results=results,
                      scores={m: score_assemblability(r) for m, r in results.items()})
        save_report(record, OUTPUT_DIR / f"{key}.json")
        row = dict(case=key, score=record["scores"]["socp"].score, dimension=results["socp"].dimension,
                   active_area_m2=sum(p.area_m2 for p in contacts.patches if p.classification == "active"),
                   socp_ms=1000*results["socp"].diagnostics["elapsed_s"],
                   fibonacci_ms=1000*results["fibonacci"].diagnostics["elapsed_s"])
        summary.append(row)
        print(row, flush=True)
    save_report(summary, OUTPUT_DIR / "summary.json")


if __name__ == "__main__":
    main()
