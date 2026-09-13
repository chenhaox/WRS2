"""Time the current static API and its stages, without changing its implementation."""
from collections import defaultdict
from contextlib import ExitStack
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from time import perf_counter
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import (StabilityConfig, ExternalWrench, LoadCase, SupportCandidate,
                          check_equilibrium, find_support_requirements)
import wrs.assembly.mechanics.equilibrium as stability
from wrs.assembly.contact.graph import ContactGraph
from examples.assembly._shared.stability_cases import make_case


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
REPEAT = 51
OUTPUT = ROOT / "benchmark_results" / "assembly" / "stability.json"


def measure(call, repeat):
    call()  # imports and allocator warmup; results and LP bases are not cached
    times = []
    for _ in range(repeat):
        start = perf_counter()
        result = call()
        times.append((perf_counter()-start)*1000)
    return result, {'median_ms':float(np.median(times)), 'p95_ms':float(np.percentile(times,95)),
                    'individual_calls_ms':times}


def profile_stages(call, repeat):
    elapsed, counts = defaultdict(float), defaultdict(int)

    def timed(name, function):
        def wrapped(*args, **kwargs):
            start = perf_counter()
            try:
                return function(*args, **kwargs)
            finally:
                elapsed[name] += perf_counter()-start
                counts[name] += 1
        return wrapped

    # These stages do not call one another, so the remainder is meaningful.
    targets = [(ContactGraph,'assert_matches','state_check'),
               (stability,'digest','input_digest'),
               (stability,'prepare_force_points','contact_points'),
               (stability,'_generators','friction_generators'),
               (stability,'linprog','scipy_highs')]
    with ExitStack() as stack:
        for owner, attr, name in targets:
            stack.enter_context(patch.object(owner,attr,timed(name,getattr(owner,attr))))
        start = perf_counter()
        for _ in range(repeat):
            call()
        total = perf_counter()-start
    stages = {name:value*1000/repeat for name,value in elapsed.items()}
    stages['assembly_validation_output_and_instrumentation'] = (total-sum(elapsed.values()))*1000/repeat
    return {'mean_stage_ms':stages, 'instrumented_total_mean_ms':total*1000/repeat,
            'stage_calls_per_api_call':{k:v/repeat for k,v in counts.items()}}


def main():
    if REPEAT < 1:
        raise ValueError('REPEAT must be positive')
    assembly, state, graph = make_case('stack')
    floating, floating_state, floating_graph = make_case('floating')
    loads = tuple(LoadCase(f'push_{i}',(ExternalWrench('upper',tuple(v)),))
                  for i,v in enumerate(np.vstack((np.eye(3),-np.eye(3)))))
    circle = tuple(LoadCase(f'push_angle_{i}',(ExternalWrench('upper',(np.cos(a),np.sin(a),0)),))
                   for i,a in enumerate(2*np.pi*np.arange(24)/24))
    cases = []
    for sides in (4,16,32):
        cfg = StabilityConfig(friction_sides=sides)
        cases.append((f'stack_{sides}_sides',lambda cfg=cfg:check_equilibrium(assembly,state,graph,config=cfg)))
    for name,disturbances in (('stack_plus_6_loads',loads),('stack_plus_24_loads',circle)):
        cfg = StabilityConfig(disturbances=disturbances)
        cases.append((name,lambda cfg=cfg:check_equilibrium(assembly,state,graph,config=cfg)))
    cases.append(('floating',lambda:check_equilibrium(floating,floating_state,floating_graph)))
    supports = (SupportCandidate('left','lower',(-.03,0,0),(0,0,1),20.),
                SupportCandidate('right','lower',(.03,0,0),(0,0,1),20.))
    cases.append(('search_two_supports',lambda:find_support_requirements(floating,floating_state,floating_graph,supports)))
    report = {'timestamp':datetime.now(timezone.utc).isoformat(), 'python':sys.executable,
              'repeat':REPEAT, 'scope':'Warm API calls; contact extraction, imports and viewer excluded. '
              'Stage measurements are a separate instrumented run, not an exact partition of latency medians.', 'cases':[]}
    for name,call in cases:
        result, timing = measure(call,REPEAT)
        row = dict(case=name,status=result.status,**timing,**profile_stages(call,REPEAT))
        equilibrium = result.equilibrium if hasattr(result,'equilibrium') else result
        row.update(force_points=equilibrium.diagnostics['force_points'],
                   force_variables=equilibrium.diagnostics['force_variables'],
                   robustness_status=equilibrium.robustness_status)
        if hasattr(result,'tested_subsets'):
            row['tested_subsets'] = result.tested_subsets
        report['cases'].append(row)
        print(f'{name}: {result.status}, median={timing["median_ms"]:.3f} ms, p95={timing["p95_ms"]:.3f} ms')
        print(json.dumps(row['mean_stage_ms']))
    OUTPUT.parent.mkdir(parents=True,exist_ok=True)
    OUTPUT.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print('Report:',OUTPUT)


if __name__ == '__main__':
    main()
