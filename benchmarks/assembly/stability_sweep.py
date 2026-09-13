"""Reproducible CPU/GPU directional LP benchmark (no viewer).

Set COUNTS / REPEATS below; CPU-only machines can set BACKENDS = ['highs', 'numpy'].
"""
import json
import platform
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import DirectionalStabilityAnalyzer,StabilitySweepConfig,disturbance_directions
from examples.assembly._shared.stability_cases import make_case


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
CASES = ['stack','bridge']
COUNTS = [300,6500]
BACKENDS = ['highs','numpy','cuda']
MODE = 'wrench'
REPEATS = 3
LARGE_REPEATS = None
BATCH_SIZE = 256
OUTPUT_DIR = ROOT / "benchmark_results" / "assembly" / "stability_sweep"


def main():
    if REPEATS<1: raise ValueError('positive repeats required')
    report=dict(python=sys.executable,platform=platform.platform(),mode=MODE,rows=[])
    out=OUTPUT_DIR; out.mkdir(parents=True,exist_ok=True)
    for name in CASES:
        start=perf_counter(); a,s,g=make_case(name); geometry_s=perf_counter()-start
        analyzers={}; references={}
        for backend in BACKENDS:
            cfg=StabilitySweepConfig(backend=backend,mode=MODE,batch_size=BATCH_SIZE)
            start=perf_counter(); analyzer=DirectionalStabilityAnalyzer(a,s,g,config=cfg)
            setup=perf_counter()-start; analyzers[backend]=analyzer
            for count in COUNTS:
                directions=disturbance_directions(count,MODE)
                cold=analyzer.analyze(directions)
                # Results are copied to CPU and certified before analyze returns;
                # these timings include CUDA completion and transfer, not enqueue.
                repeats=LARGE_REPEATS if count>=1000 and LARGE_REPEATS is not None else REPEATS
                if repeats<1: raise ValueError('positive large-repeats required')
                results=[analyzer.analyze(directions) for _ in range(repeats)]
                scores=np.array([c.score_n if c.score_n is not None else np.nan for c in cold.cases])
                ref=references.setdefault(count,scores)
                row=dict(case=name,backend=backend,directions_per_body=count,problems=len(scores),
                         geometry_s=geometry_s,prepare_s=setup,first_query_s=cold.diagnostics['query_s'],
                         warm_median_s=float(np.median([r.diagnostics['query_s'] for r in results])),
                         warm_queries_s=[r.diagnostics['query_s'] for r in results],
                         reference_backend=BACKENDS[0],
                         max_score_difference_n=float(np.max(abs(scores-ref))) if np.all(np.isfinite(scores-ref)) else None,
                         sampled_minimum_n=cold.sampled_minimum_n,status=cold.status,
                         diagnostics=dict(results[-1].diagnostics))
                report['rows'].append(row)
                print(json.dumps(row,ensure_ascii=False),flush=True)
                (out/f'benchmark_{MODE}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


if __name__=='__main__': main()
