"""Contact/direction calculations and report records for the paper catalog."""
from collections import Counter
from functools import lru_cache
from time import perf_counter
import numpy as np
from wrs.assembly import (analyze_contacts,build_contact_graph,assembly_directions,
    DirectionConfig,score_assemblability)
from wrs.assembly.geometry.proximity import MeshProximity
from .paper_cases import CATALOG,make_case,prefix_state


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
