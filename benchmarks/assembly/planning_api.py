"""Measure the production direction and equilibrium APIs; geometry is prepared first."""
import json
from pathlib import Path
from time import perf_counter
import sys
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import DirectionConfig, solve_directions, check_equilibrium
from benchmarks.assembly._direction_fixtures import make_case
from examples.assembly._shared.stability_cases import make_case as make_stack


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
REPEAT = 31
SAMPLES = 6500
OUTPUT = ROOT / "benchmark_results" / "assembly" / "planning_api.json"


def measure(call, repeat):
    start = perf_counter()
    result = call()
    first = (perf_counter()-start)*1000
    call()  # Warm imports/allocator; no solver or answer cache.
    times = []
    for _ in range(repeat):
        start = perf_counter()
        result = call()
        times.append((perf_counter()-start)*1000)
    return result, {'first_call_ms':first, 'median_ms':float(np.median(times)),
                    'p95_ms':float(np.percentile(times,95)), 'individual_calls_ms':times}


def main():
    if REPEAT < 1:
        raise ValueError('REPEAT must be positive')
    report = {'python':sys.executable, 'repeat':REPEAT, 'samples':SAMPLES,
              'scope':'Full matrix direction API and equilibrium API; excludes contact extraction/imports/viewer',
              'directions':[], 'stability':[]}
    cases = {name:make_case(name)[2] for name in ('plane','corner','channel','shaft','blind_shaft','blocked')}
    angles = np.arange(32)*2*np.pi/32
    R = Rotation.from_rotvec([.2,.7,-.4]).as_matrix()
    cases['rotated_32_radials'] = np.column_stack((np.cos(angles),np.sin(angles),np.zeros(32))) @ R.T
    for name, normals in cases.items():
        for method in ('fibonacci','socp'):
            cfg = DirectionConfig(method=method, sample_count=SAMPLES)
            result, timing = measure(lambda:solve_directions(normals,config=cfg),REPEAT)
            row = dict(case=name,method=method,status=result.status,dimension=result.dimension,
                       directions=len(result.directions),roundoff_reduction=result.diagnostics.get('roundoff_reduction',False),
                       dimension_lps=result.diagnostics['dimension_lps'],**timing)
            report['directions'].append(row)
            print(f'{name:20s} {method:10s} {result.status:12s} dim={result.dimension} '
                  f'points={len(result.directions):4d} median={timing["median_ms"]:.3f} ms p95={timing["p95_ms"]:.3f} ms')
    for name in ('stack','floating'):
        assembly, state, graph = make_stack(name)
        result, timing = measure(lambda:check_equilibrium(assembly,state,graph),REPEAT)
        report['stability'].append(dict(case=name,status=result.status,**timing))
        print(f'{name:20s} equilibrium {result.status:12s} median={timing["median_ms"]:.3f} ms')
    OUTPUT.parent.mkdir(parents=True,exist_ok=True)
    OUTPUT.write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    print('Report:',OUTPUT)


if __name__ == '__main__':
    main()
