"""STL/SDF gallery and reproducible cold/warm timings, with staged measurements."""
import argparse
from dataclasses import replace
from pathlib import Path
from time import perf_counter
import platform
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import (Assembly, ContactModel, ContactAnalyzer, SDFContactBackend,
                          SDFConfig, ToleranceContactPolicy, save_report)
from wrs.assembly.visualization import preview_case, write_contact_html
from contact_demo import cases
from stl_cases import make_case, FILES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=('all', 'shaft', *FILES), default='all')
    parser.add_argument('--backend', choices=('mesh', 'sdf', 'both'), default='sdf')
    parser.add_argument('--repeat', type=int, default=3, help='Warm repetitions, each recomputes the band')
    parser.add_argument('--resolution-mm', type=float, help='Override the terminal cell radius; smaller resolves finer boundaries')
    parser.add_argument('--contact-tol-mm', type=float, default=.5,
                        help='Extract this proximity band and offer it as tolerance contact')
    parser.add_argument('--allow-unsigned-contact', action='store_true',
                        help='Explicitly accept unsigned proximity patches for assembly contact')
    parser.add_argument('--out-dir', type=Path, default=ROOT/'examples/assembly/output/stl')
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('--repeat must be >=1')
    if args.resolution_mm is not None and (not np.isfinite(args.resolution_mm) or args.resolution_mm <= 0):
        parser.error('--resolution-mm must be finite and positive')
    if not np.isfinite(args.contact_tol_mm) or args.contact_tol_mm < 1e-7:
        parser.error('--contact-tol-mm must be finite and >=1e-7')
    args.out_dir.mkdir(parents=True, exist_ok=True)
    # Import overhead is measured once and excluded from per-model cold analysis.
    start = perf_counter()
    if args.backend != 'mesh':
        import open3d
    import_seconds = perf_counter()-start
    keys = ('shaft', *FILES) if args.case == 'all' else (args.case,)
    names = ('mesh', 'sdf') if args.backend == 'both' else (args.backend,)
    rows, previews = [], []
    for key in keys:
        start = perf_counter()
        if key == 'shaft':
            label, description, assembly, cfg, _ = cases()['shaft']
            models = tuple(ContactModel.from_part(p) for p in assembly.parts)
            cfg = replace(cfg, surface_resolution_m=.001, max_cells=60000, max_triangle_tests=500000)
        else:
            label, description, models, cfg = make_case(key)
        cfg = replace(cfg, near_tol_m=args.contact_tol_mm/1000,
                      contact_tol_m=min(cfg.contact_tol_m, args.contact_tol_mm/1000))
        description = description.replace('0.5 mm', f'{args.contact_tol_mm:g} mm')
        description += f' 本次提取容差带 {args.contact_tol_mm:g} mm；可切换原始分类与装配接触规则。'
        if args.resolution_mm is not None:
            cfg = replace(cfg, surface_resolution_m=args.resolution_mm/1000)
            description += f' 本次覆盖终止单元半径：{args.resolution_mm:g} mm。'
        load_seconds = perf_counter()-start
        assembly = Assembly(tuple(m.as_part() for m in models))
        for name in names:
            backend = 'mesh' if name == 'mesh' else SDFContactBackend(sdf_config=SDFConfig(
                open_surface='unsigned', max_query_points=500000))
            analyzer = ContactAnalyzer(backend, config=cfg)
            runs = []
            for iteration in range(args.repeat+1):
                start = perf_counter()
                result = analyzer.analyze(models)
                elapsed = perf_counter()-start
                stages = result.statistics['pairs'][0].get('timing_s', {})
                runs.append({'cache': 'cold' if iteration == 0 else 'warm', 'total_s': elapsed,
                             'stages_s': dict(stages)})
                print(f'{key}/{name} {runs[-1]["cache"]}: {elapsed:.4f}s', flush=True)
            coverage = [c for d in result.pair_diagnostics for c in d.get('curved_coverage', ())]
            unprocessed = sum(c['unprocessed_area_m2'] for c in coverage)
            row = {'case': key, 'backend': name, 'load_s': load_seconds,
                   'faces': [len(m.geometry.faces) for m in models],
                   'cold_s': runs[0]['total_s'],
                   'warm_median_s': float(np.median([r['total_s'] for r in runs[1:]])),
                   'warm_min_s': min(r['total_s'] for r in runs[1:]),
                   'warm_max_s': max(r['total_s'] for r in runs[1:]), 'runs': runs,
                   'unprocessed_area_mm2': unprocessed*1e6,
                   'area_b_mm2': sum(p.area_m2 for p in result.patches if p.sampling_side == 'b')*1e6,
                   'providers': [{k: d.get(k) for k in ('sdf_a', 'sdf_b')} for d in result.pair_diagnostics],
                   'config': cfg, 'sdf_config': None if name == 'mesh' else backend.sdf_config}
            rows.append(row)
            # Preserve geometry and raw classes; add a separate policy result.
            result = ToleranceContactPolicy(args.contact_tol_mm/1000,
                                             allow_unsigned=args.allow_unsigned_contact).apply(result)
            save_report(result, args.out_dir/f'{key}.{name}.contacts.json')
            preview = preview_case(f'{label} / {name}', description, assembly, assembly.initial_state(), result)
            preview['benchmark'] = {k: row[k] for k in ('cold_s', 'warm_median_s', 'warm_min_s', 'warm_max_s')}
            previews.append(preview)
            # Save after every case so longer benchmarks retain completed results.
            save_report({'python': sys.executable, 'python_version': sys.version,
                         'platform': platform.platform(), 'processor': platform.processor(),
                         'open3d_import_s': import_seconds, 'warm_repetitions': args.repeat,
                         'scope': 'CPU wall time; includes preparation/validation/integration; excludes input loading, serialization and rendering',
                         'results': rows}, args.out_dir/'timings.json')
            if unprocessed:
                print(f'  INCOMPLETE: {unprocessed*1e6:.6f} mm2 unprocessed', flush=True)
    write_contact_html(previews, args.out_dir/'contacts.html')
    print(f'Preview: {(args.out_dir/"contacts.html").resolve()}', flush=True)


if __name__ == '__main__':
    main()
