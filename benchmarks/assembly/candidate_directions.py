"""Measure existing local direction generation separately from contact extraction.

Whole-call timings run the production function unchanged. A separate in-memory
AST copy adds four timestamp boundaries for phase attribution; its outputs must
match production exactly. No planner code or optimization policy is changed.
"""
import ast
import hashlib
import inspect
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from importlib.metadata import version
from pathlib import Path
from time import perf_counter_ns

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from benchmarks.assembly._candidate_fixtures import fixtures
from wrs.assembly import ContactConfig, analyze_contacts, build_contact_graph
from wrs.assembly.motion import constraints as module
from wrs.assembly.model import to_dict


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
REPEAT = 31
PHASE_REPEAT = 15
OUT = ROOT / "benchmark_results" / "assembly" / "candidate_directions.json"


def timed_copy():
    """Insert timestamps at structural boundaries; fail on unsupported source."""
    source = inspect.getsource(module.candidate_motions)
    tree = ast.parse(source)
    function = tree.body[0]
    counts = dict(pre=0, lp=0, seeds=0, check=0)

    def stamp(name):
        return ast.parse(f"_phase_stamp('{name}')").body[0]

    body = []
    for node in function.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name == 'seeds':
                body.append(stamp('constraints_and_matrix'))
                counts['pre'] += 1
            elif name == 'candidates':
                body.append(stamp('extra_seeds_normalize_dedup'))
                counts['seeds'] += 1
            elif name == 'status':
                body.append(stamp('validation_and_records'))
                counts['check'] += 1
        if isinstance(node, ast.If) and ast.unparse(node.test) == 'len(matrix)':
            assert isinstance(node.body[0], ast.For)
            assert 'linprog' in ast.unparse(node.body[0])
            node.body.insert(1, stamp('coordinate_lp'))
            assert not node.orelse
            node.orelse = [stamp('coordinate_lp')]
            counts['lp'] += 1
        body.append(node)
    assert set(counts.values()) == {1}, counts
    function.body = body
    ast.fix_missing_locations(tree)
    state = {}

    def record(name):
        now = perf_counter_ns()
        state['phases'][name] = (now-state['last'])/1e6
        state['last'] = now

    namespace = dict(module.__dict__, _phase_stamp=record)
    exec(compile(tree, '<candidate_motions_phase_timing>', 'exec'), namespace)
    return namespace['candidate_motions'], state, source


def canonical(result):
    return json.dumps(to_dict(result), sort_keys=True, allow_nan=False)


def main():
    if min(REPEAT, PHASE_REPEAT) < 3:
        raise ValueError('Use at least three repeats')
    instrumented, clock, source = timed_copy()
    rows = []
    for key, description, assembly, backend, cfg in fixtures():
        state = assembly.initial_state()
        start = perf_counter_ns()
        contacts = analyze_contacts(assembly, state, backend=backend,
                                    config=ContactConfig(surface_resolution_m=.001))
        contact_ms = (perf_counter_ns()-start)/1e6
        start = perf_counter_ns()
        graph = build_contact_graph(assembly, state, contacts)
        graph_ms = (perf_counter_ns()-start)/1e6
        start = perf_counter_ns()
        result = module.candidate_motions(graph, ['part'], config=cfg)
        first_ms = (perf_counter_ns()-start)/1e6
        baseline = canonical(result)
        for _ in range(5):
            module.candidate_motions(graph, ['part'], config=cfg)
        times = []
        for _ in range(REPEAT):
            start = perf_counter_ns()
            result = module.candidate_motions(graph, ['part'], config=cfg)
            times.append((perf_counter_ns()-start)/1e6)
        assert canonical(result) == baseline
        phases = []
        for _ in range(PHASE_REPEAT):
            clock['last'] = perf_counter_ns()
            clock['phases'] = {}
            result = instrumented(graph, ['part'], config=cfg)
            end = perf_counter_ns()
            clock['phases']['status_and_freeze'] = (end-clock['last'])/1e6
            phases.append(dict(clock['phases']))
            assert canonical(result) == baseline
        row = dict(case=key, mode=cfg.mode, description=description,
                   contact_extraction_once_ms=contact_ms, graph_once_ms=graph_ms,
                   first_call_for_case_ms=first_ms, repeat=REPEAT,
                   warm_median_ms=float(np.median(times)), warm_p95_ms=float(np.percentile(times, 95)),
                   raw_warm_ms=times, phase_repeat=PHASE_REPEAT,
                   phase_median_ms={k:float(np.median([p[k] for p in phases])) for k in phases[0]},
                   raw_phase_ms=phases, raw_rows=len(result.constraints.matrix_world),
                   unique_solver_rows=result.diagnostics['row_count'],
                   seeds=result.diagnostics['seed_count'], candidates=len(result.candidates),
                   status=result.status, issues=list(result.constraints.issues),
                   phase_copy_matches_production=True)
        rows.append(row)
        print(json.dumps({k:row[k] for k in ('case','warm_median_ms','phase_median_ms',
                         'unique_solver_rows','candidates','status')}, ensure_ascii=False), flush=True)
    report = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(), python=sys.executable,
                  python_version=platform.python_version(), platform=platform.platform(),
                  processor=platform.processor(),
                  versions={k:version(k) for k in ('numpy','scipy','open3d')},
                  git_head=subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT, text=True).strip(),
                  candidate_source_sha256=hashlib.sha256(source.encode()).hexdigest(),
                  scope='warm full candidate call; excludes imports/contact extraction/graph/render; no best-direction solve',
                  phase_scope='separate instrumented calls; phase medians need not sum to full-call median',
                  cases=rows)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(OUT)


if __name__ == '__main__':
    main()
