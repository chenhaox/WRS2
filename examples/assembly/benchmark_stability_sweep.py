"""Reproducible CPU/GPU directional LP benchmark (no viewer).

python examples/assembly/benchmark_stability_sweep.py --counts 300 6500 --repeats 3
CUDA is optional: pass --backends highs numpy on a CPU-only machine.
"""
import argparse
import json
import platform
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from wrs.assembly import DirectionalStabilityAnalyzer,StabilitySweepConfig,disturbance_directions
from _stability_cases import make_case


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases',nargs='+',default=['stack','bridge'])
    parser.add_argument('--counts',nargs='+',type=int,default=[300,6500])
    parser.add_argument('--backends',nargs='+',default=['highs','numpy','cuda'])
    parser.add_argument('--mode',choices=['force','torque','wrench','legacy_coupled'],default='legacy_coupled')
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--large-repeats',type=int,default=None,help='optional repeat count for >=1000 directions')
    parser.add_argument('--batch-size',type=int,default=256)
    args=parser.parse_args()
    if args.repeats<1: parser.error('positive repeats required')
    report=dict(python=sys.executable,platform=platform.platform(),mode=args.mode,rows=[])
    out=Path(__file__).parent/'output'/'stability_sweep'; out.mkdir(parents=True,exist_ok=True)
    for name in args.cases:
        start=perf_counter(); a,s,g=make_case(name); geometry_s=perf_counter()-start
        analyzers={}; references={}
        for backend in args.backends:
            cfg=StabilitySweepConfig(backend=backend,mode=args.mode,batch_size=args.batch_size)
            start=perf_counter(); analyzer=DirectionalStabilityAnalyzer(a,s,g,config=cfg)
            setup=perf_counter()-start; analyzers[backend]=analyzer
            for count in args.counts:
                directions=disturbance_directions(count,args.mode)
                cold=analyzer.analyze(directions)
                # Results are copied to CPU and certified before analyze returns;
                # these timings include CUDA completion and transfer, not enqueue.
                repeats=args.large_repeats if count>=1000 and args.large_repeats is not None else args.repeats
                if repeats<1: parser.error('positive large-repeats required')
                results=[analyzer.analyze(directions) for _ in range(repeats)]
                scores=np.array([c.score_n if c.score_n is not None else np.nan for c in cold.cases])
                ref=references.setdefault(count,scores)
                row=dict(case=name,backend=backend,directions_per_body=count,problems=len(scores),
                         geometry_s=geometry_s,prepare_s=setup,first_query_s=cold.diagnostics['query_s'],
                         warm_median_s=float(np.median([r.diagnostics['query_s'] for r in results])),
                         warm_queries_s=[r.diagnostics['query_s'] for r in results],
                         reference_backend=args.backends[0],
                         max_score_difference_n=float(np.max(abs(scores-ref))) if np.all(np.isfinite(scores-ref)) else None,
                         sampled_minimum_n=cold.sampled_minimum_n,status=cold.status,
                         diagnostics=dict(results[-1].diagnostics))
                report['rows'].append(row)
                print(json.dumps(row,ensure_ascii=False),flush=True)
                (out/f'benchmark_{args.mode}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


if __name__=='__main__': main()
