"""Compare deterministic sphere filtering, coordinate LPs and margin solvers.

This is an experiment, not a production direction API. Empty samples do not
prove blockage. SOCP zero margin requires a separate feasibility/dimension
check. Contact extraction, rendering and finite paths are outside timings.
"""
from contextlib import nullcontext
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import importlib.metadata as metadata
import json
import platform
from pathlib import Path
import sys
from time import perf_counter_ns

import numpy as np
from scipy import sparse
from scipy.optimize import linprog
from threadpoolctl import threadpool_info, threadpool_limits

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from benchmarks.assembly._candidate_fixtures import fixtures
from wrs.assembly import ContactConfig, analyze_contacts, build_contact_graph
from wrs.assembly.motion.constraints import candidate_motions

TOL = 1e-9


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
SAMPLES = 6500
REPEAT = 31
OUT = ROOT / "benchmark_results" / "assembly" / "direction_methods.json"


def fibonacci_sphere(count=6500):
    """Fixed midpoint-z Fibonacci bank, float64 (count, 3), excluding poles."""
    if not isinstance(count, int) or isinstance(count, bool) or count < 1:
        raise ValueError('count must be a positive integer')
    i = np.arange(count, dtype=np.float64)
    z = 1 - 2 * (i + .5) / count
    angle = i * (np.pi * (3 - np.sqrt(5)))
    radius = np.sqrt(np.maximum(0, 1 - z*z))
    return np.column_stack((radius*np.cos(angle), radius*np.sin(angle), z))


def margins(normals, directions, method='matmul', scratch_bytes=8*1024**2):
    """Minimum unit-normal dot product for each unit direction.

    Inputs are already normalized. Chunking caps the dot-product scratch
    matrix; input/output arrays and BLAS internal workspace are additional.
    """
    m, k = len(normals), len(directions)
    if m == 0:
        return np.full(k, np.inf)
    if method == 'matmul':
        return np.min(normals @ directions.T, axis=0)
    if method == 'einsum':
        return np.min(np.einsum('ij,kj->ik', normals, directions,
                                optimize=False), axis=0)
    if method != 'blocked':
        raise ValueError(method)
    capacity = max(1, scratch_bytes//8)
    columns = min(k, 4096, capacity)
    rows = min(m, 128, max(1, capacity//columns))
    scratch = np.empty((rows, columns), dtype=np.float64)
    minimum = np.full(k, np.inf)
    for start in range(0, k, columns):
        block = directions[start:start+columns]
        target = minimum[start:start+len(block)]
        for offset in range(0, m, rows):
            active = normals[offset:offset+rows]
            out = scratch[:len(active), :len(block)]
            np.matmul(active, block.T, out=out)
            np.minimum(target, out.min(axis=0), out=target)
    return minimum


def sample_result(normals, directions, method='matmul'):
    if method == 'progressive':
        return progressive_result(normals, directions)
    low = margins(normals, directions, method)
    feasible = np.flatnonzero(low >= -TOL)
    best = int(feasible[np.argmax(low[feasible])]) if len(feasible) else None
    return dict(count=len(feasible), best_index=best,
                best_margin=float(low[best]) if best is not None and len(normals) else None)


def progressive_result(normals, directions, row_batch=32):
    """Keep only feasible samples; never materialize the full M x K table.

    Deterministic extrema-first ordering helps reject directions early. This
    may help some constraint distributions and hurt others; it is benchmarked,
    not assumed universally fastest. Ordering and gather costs are included.
    """
    ids = np.arange(len(directions))
    low = np.full(len(ids), np.inf)
    if len(normals):
        first = np.unique(np.r_[np.argmin(normals, axis=0), np.argmax(normals, axis=0)])
        used = np.zeros(len(normals), dtype=bool)
        used[first] = True
        order = np.r_[first, np.flatnonzero(~used)]
        # Use a tiny first batch, then check every remaining row in blocks.
        batches = [first]
        batches.extend(order[i:i+row_batch] for i in range(len(first), len(order), row_batch))
        for batch in batches:
            selected = directions[ids]
            rows = normals[batch]
            if len(rows) <= 3:
                dots = np.einsum('ij,kj->ik', rows, selected, optimize=False)
            else:
                dots = rows @ selected.T
            np.minimum(low, dots.min(axis=0), out=low)
            keep = low >= -TOL
            ids, low = ids[keep], low[keep]
            if not len(ids):
                break
    best = int(np.argmax(low)) if len(ids) else None
    return dict(count=len(ids), best_index=int(ids[best]) if best is not None else None,
                best_margin=float(low[best]) if best is not None and len(normals) else None)


def six_lp(normals):
    found = []
    if not len(normals):
        return np.vstack((np.eye(3), -np.eye(3)))
    for axis in np.vstack((np.eye(3), -np.eye(3))):
        result = linprog(-axis, A_ub=-normals, b_ub=np.zeros(len(normals)),
                         bounds=[(-1, 1)]*3, method='highs')
        if not result.success:
            raise RuntimeError(result.message)
        size = np.linalg.norm(result.x)
        if size > TOL:
            direction = result.x / size
            if np.min(normals @ direction) < -TOL:
                raise AssertionError('LP direction violates original rows')
            found.append(direction)
    return np.asarray(found).reshape(-1, 3)


def solve_margin(normals, solver_module, formulation='socp'):
    """Construct + solve + residual check, on an already prepared matrix.

    SOCP: maximize t, N*d>=t, ||d||<=1.
    QP: minimize ||x||^2/2, N*x>=1; equivalent only if strict feasibility
    exists. QP infeasible does not imply the original cone is blocked.
    """
    m = len(normals)
    if not m:
        return dict(status='unconstrained', direction=None, margin=None)
    settings = solver_module.DefaultSettings()
    settings.verbose = False
    settings.tol_gap_abs = settings.tol_gap_rel = settings.tol_feas = 1e-10
    if formulation == 'socp':
        P = sparse.csc_matrix((4, 4))
        q = np.array([0., 0., 0., -1.])
        A = sparse.csc_matrix(np.vstack((
            np.column_stack((-normals, np.ones(m))),
            np.zeros((1, 4)), np.column_stack((-np.eye(3), np.zeros(3))))))
        b = np.r_[np.zeros(m), 1., np.zeros(3)]
        cones = [solver_module.NonnegativeConeT(m), solver_module.SecondOrderConeT(4)]
    else:
        P = sparse.eye(3, format='csc')
        q = np.zeros(3)
        A, b = sparse.csc_matrix(-normals), -np.ones(m)
        cones = [solver_module.NonnegativeConeT(m)]
    solver = solver_module.DefaultSolver(P, q, A, b, cones, settings)
    result = solver.solve()
    answer = dict(status=str(result.status), iterations=result.iterations,
                  primal_residual=float(result.r_prim), dual_residual=float(result.r_dual),
                  direction=None, margin=None)
    if str(result.status) != 'Solved':
        return answer
    x = np.array(result.x[:3])
    size = float(np.linalg.norm(x))
    answer['raw_norm'] = size
    if formulation == 'socp':
        answer['objective_t'] = float(result.x[3])
        if size > 1+1e-7 or np.min(normals @ x-result.x[3]) < -1e-7:
            raise AssertionError('SOCP primal residual')
        if result.x[3] <= 1e-8:
            answer['classification'] = 'zero_margin_needs_feasibility_and_dimension_check'
            return answer
    elif np.min(normals @ x) < 1-1e-7:
        raise AssertionError('QP primal residual')
    if size > TOL:
        direction = x/size
        low = float(np.min(normals @ direction))
        if low < -TOL:
            raise AssertionError('Returned unit direction violates constraints')
        answer.update(direction=direction.tolist(), margin=low,
                      classification='positive_margin')
    return answer


def timing(fn, repeat):
    """Warm timing with amortized clock overhead; retain every batch mean."""
    start = perf_counter_ns()
    fn()
    first_ns = perf_counter_ns()-start
    for _ in range(3):
        fn()
    inner = min(1000, max(1, int(2_000_000/max(first_ns, 1))))
    times = []
    for _ in range(repeat):
        start = perf_counter_ns()
        for _ in range(inner):
            fn()
        times.append((perf_counter_ns()-start)/inner/1e6)
    return dict(median_ms=float(np.median(times)), p95_batch_mean_ms=float(np.percentile(times, 95)),
                first_ms=first_ns/1e6, calls_per_batch=inner, raw_batch_mean_ms=times)


def unit_rows(matrix):
    matrix = np.asarray(matrix, dtype=np.float64).reshape(-1, 3)
    return np.unique(matrix/np.linalg.norm(matrix, axis=1)[:, None], axis=0)


def main():
    if SAMPLES < 1 or REPEAT < 3:
        raise ValueError('Use positive samples and at least 3 repeats')
    try:
        import clarabel
    except ImportError:
        clarabel = None
    directions = fibonacci_sphere(SAMPLES)
    np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1, rtol=0, atol=1e-14)
    np.testing.assert_array_equal(directions, fibonacci_sphere(SAMPLES))
    cases, production = {}, {}
    # Same physical M2 fixtures; the corner is deliberately translation-only.
    for key, description, assembly, backend, cfg in fixtures()[:4]:
        cfg = replace(cfg, mode='translation')
        state = assembly.initial_state()
        contacts = analyze_contacts(assembly, state, backend=backend,
                                    config=ContactConfig(surface_resolution_m=.001))
        graph = build_contact_graph(assembly, state, contacts)
        baseline = candidate_motions(graph, ['part'], config=cfg)
        name = key.replace('_twist', '_translation')
        cases[name] = unit_rows(baseline.constraints.matrix_world[:, :3])
        production[name] = dict(timing=timing(lambda: candidate_motions(graph, ['part'], config=cfg), REPEAT),
                                candidate_count=len(baseline.candidates), status=baseline.status,
                                raw_rows=len(baseline.constraints.matrix_world), issues=list(baseline.constraints.issues))
    cases['shaft_ideal'] = unit_rows([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0]])
    # A rotated ideal shaft prevents axis-specific heuristics from hiding misses.
    axis = np.array([1., 2., 3.])/np.sqrt(14)
    a = np.cross(axis, [0., 0., 1.]); a /= np.linalg.norm(a)
    b = np.cross(axis, a)
    cases['shaft_tilted'] = unit_rows([a, -a, b, -b])
    epsilon = np.tan(np.deg2rad(.1))
    cases['narrow_0p1deg'] = unit_rows([[1,0,epsilon],[-1,0,epsilon],[0,1,epsilon],[0,-1,epsilon]])
    rows = []
    for name, normals in cases.items():
        sampled = sample_result(normals, directions)
        found = six_lp(normals)
        row = dict(case=name, normalized_rows=normals.tolist(), rows=len(normals),
                   sample=sampled, six_lp_nonzero=bool(len(found)), six_lp=timing(lambda: six_lp(normals), REPEAT))
        row['sample_cached'] = timing(lambda: sample_result(normals, directions), REPEAT)
        row['sample_generate_and_filter'] = timing(
            lambda: sample_result(normals, fibonacci_sphere(SAMPLES)), REPEAT)
        if clarabel:
            for formulation in ('socp', 'qp'):
                result = solve_margin(normals, clarabel, formulation)
                row[formulation] = dict(result=result, timing=timing(
                    lambda: solve_margin(normals, clarabel, formulation), REPEAT))
            if row['socp']['result']['margin'] is not None:
                opt = row['socp']['result']['margin']
                np.testing.assert_allclose(opt, row['qp']['result']['margin'], atol=1e-7, rtol=0)
                if sampled['best_margin'] is not None:
                    assert sampled['best_margin'] <= opt+1e-7
                    row['sample_angle_margin_deg'] = float(np.rad2deg(np.arcsin(np.clip(sampled['best_margin'], 0, 1))))
                    row['optimal_angle_margin_deg'] = float(np.rad2deg(np.arcsin(np.clip(opt, 0, 1))))
            expected = {'face': 1., 'corner_translation': 1/np.sqrt(3),
                        'narrow_0p1deg': np.sin(np.deg2rad(.1))}.get(name)
            if expected is not None:
                np.testing.assert_allclose(row['socp']['result']['margin'], expected, atol=1e-8, rtol=0)
        if name in ('channel', 'shaft_ideal', 'shaft_tilted', 'narrow_0p1deg'):
            assert len(found), name
        if name == 'enclosed':
            assert not len(found) and sampled['count'] == 0
        if name in production:
            row['production_full'] = production[name]
        rows.append(row)
        print(json.dumps(dict(case=name, samples_hit=sampled['count'], lp_nonzero=bool(len(found)),
                              sample_ms=row['sample_cached']['median_ms'],
                              lp_ms=row['six_lp']['median_ms'],
                              socp_ms=row.get('socp', {}).get('timing', {}).get('median_ms'))), flush=True)
    kernels, large_solvers = [], []
    # Fixed full-dimensional cap of normals, 32..2048 rows; independent of M2.
    for count in (32, 256, 2048):
        angles = np.arange(count)*(2*np.pi/count)
        normals = unit_rows(np.column_stack((np.cos(angles), np.sin(angles), np.full(count, .3))))
        reference = margins(normals, directions)
        scalar = np.array([min(normals @ d) for d in directions[:37]])
        np.testing.assert_allclose(reference[:37], scalar, atol=1e-14, rtol=0)
        sampled = sample_result(normals, directions)
        for threads in ('default', 1):
            context = nullcontext() if threads == 'default' else threadpool_limits(limits=1, user_api='blas')
            with context:
                for method in ('matmul', 'einsum', 'blocked', 'progressive'):
                    if method != 'progressive':
                        result = margins(normals, directions, method)
                        np.testing.assert_allclose(result, reference, atol=1e-14, rtol=0)
                        np.testing.assert_array_equal(result >= -TOL, reference >= -TOL)
                    result = sample_result(normals, directions, method)
                    assert result['count'] == sampled['count']
                    assert result['best_index'] == sampled['best_index']
                    np.testing.assert_allclose(result['best_margin'], sampled['best_margin'], atol=1e-14, rtol=0)
                    kernels.append(dict(rows=count, samples=SAMPLES, method=method,
                                        blas_threads=threads, timing=timing(
                                            lambda: sample_result(normals, directions, method), REPEAT)))
        if clarabel:
            optimal = solve_margin(normals, clarabel)
            np.testing.assert_allclose(optimal['margin'], .3/np.sqrt(1.09), atol=1e-8, rtol=0)
            large_solvers.append(dict(rows=count, sample=sampled, optimal=optimal,
                                      six_lp=timing(lambda: six_lp(normals), REPEAT),
                                      socp=timing(lambda: solve_margin(normals, clarabel), REPEAT),
                                      qp=timing(lambda: solve_margin(normals, clarabel, 'qp'), REPEAT)))
        print('kernels rows='+str(count), flush=True)
    # Broad nearly parallel normals leave many more samples alive; this is a
    # countercheck against assuming the narrow-cap speedup is universal.
    broad = []
    angles = np.arange(2048)*(2*np.pi/2048)
    normals = unit_rows(np.column_stack((np.cos(angles), np.sin(angles), np.full(2048, 10.))))
    reference = sample_result(normals, directions)
    with threadpool_limits(limits=1, user_api='blas'):
        for method in ('matmul', 'blocked', 'progressive'):
            result = sample_result(normals, directions, method)
            assert result['count'] == reference['count']
            assert result['best_index'] == reference['best_index']
            np.testing.assert_allclose(result['best_margin'], reference['best_margin'], atol=1e-14, rtol=0)
            broad.append(dict(rows=2048, samples=SAMPLES, feasible_count=result['count'],
                              method=method, blas_threads=1, timing=timing(
                                  lambda: sample_result(normals, directions, method), REPEAT)))
    scaling = []
    normals = cases['corner_translation']
    for count in sorted(set((650, SAMPLES, 65000))):
        bank = fibonacci_sphere(count)
        sampled = sample_result(normals, bank)
        scaling.append(dict(samples=count, sample=sampled,
                            generate=timing(lambda: fibonacci_sphere(count), REPEAT),
                            cached_filter=timing(lambda: sample_result(normals, bank), REPEAT)))
    report = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(), python=sys.executable,
                  python_version=platform.python_version(), platform=platform.platform(), cpu=platform.processor(),
                  versions={key: metadata.version(key) for key in ('numpy','scipy','threadpoolctl')},
                  clarabel_version=clarabel.__version__ if clarabel else None,
                  blas=threadpool_info(), samples=SAMPLES, tolerance=TOL, repeat=REPEAT,
                  benchmark_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  scope='Matrix-ready kernels include allocations/filter/best selection; six LP and conic timings include model construction and residual checks. Production full also constructs constraints and candidate records. All are warm; no imports, contact extraction, rendering, dimension reduction or finite paths.',
                  timing_definition='Each reported sample is a batch mean. p95 is not individual-call p95.',
                  cases=rows, kernels=kernels, large_solvers=large_solvers,
                  broad_normal_distribution=broad, scaling=scaling)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(OUT, flush=True)


if __name__ == '__main__':
    main()
