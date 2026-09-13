"""Warm API wall times; contact extraction and visualization are separate."""
from dataclasses import replace
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import check_equilibrium
from examples.assembly._shared.stability_cases import DESCRIPTIONS,make_case,case_config,case_supports


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
REPEAT = 31
COMPARE_REDUCTION = False
OUTPUT_DIR = ROOT / "benchmark_results" / "assembly" / "stability_cases"


def main():
    if REPEAT<1: raise ValueError('REPEAT must be positive')
    rows = []
    for name in DESCRIPTIONS:
        started = perf_counter()
        a,s,g = make_case(name)
        extraction_ms = (perf_counter()-started)*1000
        supports = case_supports(name)
        for reduce in (False,True) if COMPARE_REDUCTION else (True,):
            cfg = replace(case_config(name),reduce_contact_points=reduce)
            started = perf_counter()
            r = check_equilibrium(a,s,g,config=cfg,supports=supports)
            first_ms = (perf_counter()-started)*1000
            times = []
            for _ in range(REPEAT):
                started = perf_counter()
                r = check_equilibrium(a,s,g,config=cfg,supports=supports)
                times.append((perf_counter()-started)*1000)
            row = dict(case=name,reduce_contact_points=reduce,status=r.status,force_sites=len(r.force_sites),
                       variables=r.diagnostics['force_variables'],load_cases=1+len(cfg.disturbances),
                       median_ms=float(np.median(times)),p95_ms=float(np.percentile(times,95)),
                       first_call_ms=first_ms,setup_and_contact_extraction_ms=extraction_ms,calls_ms=times)
            rows.append(row)
            print(f'{name:10} reduce={str(reduce):5} {row["median_ms"]:7.3f} ms  P95 {row["p95_ms"]:7.3f} ms '
                  f'  {row["load_cases"]} load(s), {len(r.force_sites)} sites',flush=True)
    filename = 'point-reduction-benchmark.json' if COMPARE_REDUCTION else 'cases-benchmark.json'
    output = OUTPUT_DIR / filename
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(interpreter=sys.executable,repeat=REPEAT,
        scope='warm complete API including result freeze; excludes contact extraction and rendering; '
              'floating uses both declared finite supports, excludes subset search; '
              'first call may share cleanup cached by the previous policy',rows=rows),indent=2),encoding='utf-8')


if __name__=='__main__': main()
