"""Contact regions and local directions for every paper example/declared replacement.

python examples/assembly/paper_contact_directions.py --all --headless
python examples/assembly/paper_contact_directions.py --case fig08_soma3 --port 8900
"""
import argparse
from collections import Counter
from functools import lru_cache
from pathlib import Path
import sys
from time import perf_counter
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
from wrs.assembly import (analyze_contacts,build_contact_graph,assembly_directions,
    DirectionConfig,score_assemblability,save_report)
from wrs.assembly.geometry.proximity import MeshProximity
from _paper_cases import CATALOG,make_case,prefix_state


@lru_cache(maxsize=48)
def compute(key, stage=0, samples=6500, nominal=True):
    start = perf_counter()
    assembly = make_case(key,nominal=nominal)
    state = prefix_state(assembly,stage)
    analysis = analyze_contacts(assembly,state,backend=MeshProximity())
    contact_s = perf_counter()-start
    graph = build_contact_graph(assembly,state,analysis)
    results = {}
    for pid in assembly.provenance['order']:
        if pid not in state.poses: continue
        results[pid] = {method:assembly_directions(graph,(pid,),config=DirectionConfig(method=method,sample_count=samples))
                        for method in ('socp','fibonacci')}
        for result in results[pid].values():
            if result.best_direction is not None:
                residual = result.constraints.matrix_world[:,:3]@result.best_direction
                if len(residual) and residual.min() < -1e-9:
                    raise AssertionError((key,pid,'infeasible returned direction'))
    metadata = dict(assembly.provenance)
    metadata.update(contact_s=contact_s,total_s=perf_counter()-start,stage=stage,
                    issues={p:tuple(r['socp'].issues) for p,r in results.items()})
    return assembly,state,analysis,results,metadata


def summary_record(data):
    assembly,state,analysis,results,meta = data
    return dict(case=meta['figure_key'],stage=meta['stage'],source=meta['source'],reproduction=meta['reproduction'],
        mode=meta['mode'],parts=len(results),contact_ms=1000*meta['contact_s'],total_ms=1000*meta['total_s'],
        patches=dict(Counter(p.classification for p in analysis.patches)),
        active_dimensions=dict(Counter(str(p.dimension) for p in analysis.patches if p.classification=='active')),
        overlap=dict(Counter(d['overlap']['status'] for d in analysis.pair_diagnostics)),
        directions={pid:dict(status=r['socp'].status,dimension=r['socp'].dimension,
            score=score_assemblability(r['socp']),issues=r['socp'].issues,
            socp_ms=1000*r['socp'].diagnostics.get('elapsed_s',0),
            fibonacci_ms=1000*r['fibonacci'].diagnostics.get('elapsed_s',0)) for pid,r in results.items()})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=tuple(CATALOG),default='fig08_soma3')
    parser.add_argument('--all',action='store_true',help='Check every catalog entry')
    parser.add_argument('--prefixes',action='store_true',help='Also inspect each intermediate prefix')
    parser.add_argument('--stage',type=int,default=0,help='0 = complete; 1..N = first N parts')
    parser.add_argument('--samples',type=int,default=6500)
    parser.add_argument('--raw',action='store_true',help='Original exports without nominal corrections')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--port',type=int,default=8900)
    args = parser.parse_args()
    if args.headless:
        output = Path(__file__).parent/'output'/'paper2021'/('raw' if args.raw else 'nominal')
        summary = []
        for key in (CATALOG if args.all else (args.case,)):
            try:
                count = len(make_case(key,nominal=not args.raw).provenance['order'])
            except FileNotFoundError as exc:
                summary.append(dict(case=key,status='missing_source',reason=str(exc)))
                print(f'{key}: missing_source',flush=True); continue
            stages = range(1,count+1) if args.prefixes else (args.stage,)
            for stage in stages:
                data = compute(key,stage,args.samples,not args.raw)
                record = summary_record(data); summary.append(record)
                save_report(dict(summary=record,metadata=data[4],contacts=data[2],directions=data[3]),output/f'{key}_{stage}.json')
                status = dict(Counter(r['socp'].status for r in data[3].values()))
                print(f'{key} stage={stage}/{count}: {status}, patches={record["patches"]}, '
                      f'contact={record["contact_ms"]:.1f} ms',flush=True)
        save_report(summary,output/('prefix_summary.json' if args.prefixes else 'summary.json'))
        return
    from _paper_contact_viewer import show_gallery
    show_gallery(tuple(CATALOG),lambda key,stage:compute(key,stage,args.samples,not args.raw),
                 initial=args.case,stage=args.stage,port=args.port,title='论文装配：接触面与方向')


if __name__=='__main__': main()
