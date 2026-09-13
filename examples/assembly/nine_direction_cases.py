"""Actual contact extraction -> SOCP/Fibonacci -> nine assemblability classes.

python examples/assembly/nine_direction_cases.py --headless
python examples/assembly/nine_direction_cases.py --case g --port 8899
"""
import argparse
from pathlib import Path
import sys
from dataclasses import replace
from time import perf_counter
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
from wrs.assembly import (analyze_contacts,build_contact_graph,assembly_directions,DirectionConfig,
                         score_assemblability,save_report)
from _nine_direction_cases import CASES,make_case


def compute(key,count=6500):
    assembly = make_case(key); state = assembly.initial_state()
    start = perf_counter()
    analysis = analyze_contacts(assembly,state)
    analysis = replace(analysis,statistics={**analysis.statistics,'contact_s':perf_counter()-start})
    graph = build_contact_graph(assembly,state,analysis)
    results = {method:assembly_directions(graph,('moving',),config=DirectionConfig(method=method,sample_count=count))
               for method in ('socp','fibonacci')}
    expected = CASES[key]
    for method,result in results.items():
        score = score_assemblability(result)
        if (score.case,score.score,result.dimension)!=(key,expected[2],expected[3]):
            raise AssertionError((key,method,score,result.dimension,result.issues))
        if result.best_direction is not None:
            residual = result.constraints.matrix_world[:,:3]@result.best_direction
            if np.min(residual)<-1e-9: raise AssertionError('Direction violates an extracted contact')
    active = [p for p in analysis.patches if p.classification=='active' and 'moving' in (p.part_a,p.part_b)]
    area = sum(p.area_m2 for p in active)
    if not np.isclose(area,assembly.provenance['expected_active_area_m2'],atol=1e-12):
        raise AssertionError(('unexpected physical contact area',key,area))
    return assembly,state,analysis,results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=tuple(CASES),default='a')
    parser.add_argument('--samples',type=int,default=6500)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--port',type=int,default=8899)
    args = parser.parse_args()
    output = Path(__file__).parent/'output'/'nine_directions'
    if args.headless:
        summary = []
        for key in CASES:
            a,s,contacts,results = compute(key,args.samples)
            record = dict(case=key,title=CASES[key][0],contacts=contacts,results=results,
                          scores={m:score_assemblability(r) for m,r in results.items()})
            save_report(record,output/f'{key}.json')
            summary.append(dict(case=key,score=record['scores']['socp'].score,dimension=results['socp'].dimension,
                active_area_m2=sum(p.area_m2 for p in contacts.patches if p.classification=='active'),
                socp_ms=1000*results['socp'].diagnostics['elapsed_s'],
                fibonacci_ms=1000*results['fibonacci'].diagnostics['elapsed_s']))
            print(summary[-1],flush=True)
        save_report(summary,output/'summary.json'); return
    from _paper_contact_viewer import show_gallery
    def load(key,stage):
        a,s,c,r = compute(key,args.samples)
        return a,s,c,dict(moving=r),dict(title=CASES[key][0],source='九类实际网格接触验证',
            note='蓝：拆出方向；紫：反向插入。局部方向不等于完整可执行路径。',order=('moving',),
            contact_s=c.statistics['contact_s'])
    show_gallery(tuple(CASES),load,initial=args.case,port=args.port,title='九类可装配方向')


if __name__=='__main__': main()
