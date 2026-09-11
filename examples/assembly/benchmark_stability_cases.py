"""Warm API wall times; contact extraction and visualization are separate."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from wrs.assembly import check_equilibrium
from _stability_cases import DESCRIPTIONS,make_case,case_config,case_supports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repeat',type=int,default=31)
    parser.add_argument('--compare-reduction',action='store_true',
                        help='compare complete cleaned points with default reduction on the same geometry')
    args = parser.parse_args()
    if args.repeat<1: parser.error('--repeat must be positive')
    rows = []
    for name in DESCRIPTIONS:
        started = perf_counter()
        a,s,g = make_case(name)
        extraction_ms = (perf_counter()-started)*1000
        supports = case_supports(name)
        for reduce in (False,True) if args.compare_reduction else (True,):
            cfg = replace(case_config(name),reduce_contact_points=reduce)
            started = perf_counter()
            r = check_equilibrium(a,s,g,config=cfg,supports=supports)
            first_ms = (perf_counter()-started)*1000
            times = []
            for _ in range(args.repeat):
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
    filename = 'point-reduction-benchmark.json' if args.compare_reduction else 'cases-benchmark.json'
    output = Path(__file__).with_name('output')/'stability'/filename
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(interpreter=sys.executable,repeat=args.repeat,
        scope='warm complete API including result freeze; excludes contact extraction and rendering; '
              'floating uses both declared finite supports, excludes subset search; '
              'first call may share cleanup cached by the previous policy',rows=rows),indent=2),encoding='utf-8')


if __name__=='__main__': main()
