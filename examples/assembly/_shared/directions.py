"""Actual contact extraction and independent checks of the nine direction classes.
"""
from dataclasses import replace
from time import perf_counter
import numpy as np
from wrs.assembly import (analyze_contacts,build_contact_graph,assembly_directions,DirectionConfig,
                         score_assemblability)
from .direction_cases import CASES,make_case


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


def load_case(key, samples=6500):
    assembly, state, contacts, results = compute(key, samples)
    metadata = dict(title=CASES[key][0], source='九类实际网格接触验证',
        note='蓝：拆出；紫：反向插入；局部方向仍需验证有限路径。', order=('moving',),
        contact_s=contacts.statistics['contact_s'])
    return assembly, state, contacts, {'moving': results}, metadata
