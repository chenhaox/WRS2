"""批量检查论文完整装配 / 中间步骤，保存证据；不启动窗口。"""
from pathlib import Path
import sys
from collections import Counter

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import save_report
from examples.assembly._shared.paper import compute, summary_record
from examples.assembly._shared.paper_cases import CATALOG, make_case

CASES = tuple(CATALOG)  # 或 ("fig08_soma3",)
PREFIXES = False  # True 同时检查每个中间装配
NOMINAL = True  # False：原始网格
SAMPLES = 6500
OUTPUT_DIR = ROOT / "benchmark_results" / "assembly" / "paper2021"


def main():
    output = OUTPUT_DIR / ("nominal" if NOMINAL else "raw")
    summary = []
    for key in CASES:
        try:
            count = len(make_case(key, nominal=NOMINAL).provenance["order"])
        except FileNotFoundError as exc:
            summary.append(dict(case=key, status="missing_source", reason=str(exc)))
            print(f"{key}: missing_source", flush=True)
            continue
        stages = range(1, count + 1) if PREFIXES else (0,)
        for stage in stages:
            data = compute(key, stage, SAMPLES, NOMINAL)
            record = summary_record(data)
            summary.append(record)
            save_report(dict(summary=record, metadata=data[4], contacts=data[2], directions=data[3]),
                        output / f"{key}_{stage}.json")
            status = dict(Counter(r["socp"].status for r in data[3].values()))
            print(f"{key} stage={stage}/{count}: {status}, contact={record['contact_ms']:.1f} ms", flush=True)
    save_report(summary, output / ("prefix_summary.json" if PREFIXES else "summary.json"))


if __name__ == "__main__":
    main()
