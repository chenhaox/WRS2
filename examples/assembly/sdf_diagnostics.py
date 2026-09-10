"""Compare SDF boundary resolutions and record geometry costs without rendering costs."""
import argparse
import cProfile
from dataclasses import replace
from pathlib import Path
from time import perf_counter
import pstats
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import Assembly, ContactAnalyzer, SDFContactBackend, SDFConfig, save_report
from wrs.assembly.visualization import preview_case, write_contact_html
from stl_cases import make_case, FILES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', nargs='+', choices=tuple(FILES), default=['flange', 'stl_cylinder'])
    parser.add_argument('--resolution-mm', nargs='+', type=float, default=[1, .25, .1])
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--profile', action='store_true', help='Extra profiled run, excluded from timing medians')
    parser.add_argument('--out-dir', type=Path, default=ROOT/'examples/assembly/output/diagnostics')
    args = parser.parse_args()
    if args.repeat < 1 or any(not np.isfinite(r) or r <= 0 for r in args.resolution_mm):
        parser.error('Repetitions and resolutions must be positive')
    args.out_dir.mkdir(parents=True, exist_ok=True)
    previews, rows = [], []
    for key in args.cases:
        name, description, models, cfg = make_case(key)
        assembly = Assembly(tuple(m.as_part() for m in models))
        backend = SDFContactBackend(sdf_config=SDFConfig(open_surface='unsigned', max_query_points=2000000))
        for model in models:
            backend.prepare(model)
        for resolution in args.resolution_mm:
            config = replace(cfg, surface_resolution_m=resolution/1000, max_cells=200000)
            analyzer = ContactAnalyzer(backend, config=config)
            runs = []
            for _ in range(args.repeat):
                start = perf_counter()
                result = analyzer.analyze(models)
                runs.append({'total_s': perf_counter()-start, 'stages': result.statistics['pairs'][0]['timing_s']})
            row = {'case': key, 'resolution_mm': resolution, 'runs': runs,
                   'median_s': float(np.median([r['total_s'] for r in runs])),
                   'areas_mm2': {side: sum(p.area_m2 for p in result.patches if p.sampling_side == side)*1e6
                                 for side in ('a', 'b')},
                   'coverage': result.pair_diagnostics[0]['curved_coverage'], 'config': config,
                   'sdf_config': backend.sdf_config}
            rows.append(row)
            print(f"{key} resolution={resolution}mm median={row['median_s']:.4f}s areas={row['areas_mm2']}", flush=True)
            save_report(result, args.out_dir/f'{key}.{resolution:g}mm.contacts.json')
            preview = preview_case(f'{name} / {resolution:g} mm',
                                   description+f' 终止单元半径 {resolution:g} mm；near 与法向阈值保持不变。',
                                   assembly, assembly.initial_state(), result)
            preview['benchmark'] = {'warm_median_s': row['median_s']}
            previews.append(preview)
            save_report({'python': sys.executable, 'scope': 'warm full analysis; excludes loading, rendering and profiler overhead',
                         'results': rows}, args.out_dir/'measurements.json')
            if args.profile:
                profiler = cProfile.Profile()
                profiler.runcall(analyzer.analyze, models)
                with (args.out_dir/f'{key}.{resolution:g}mm.profile.txt').open('w', encoding='utf-8') as stream:
                    pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats('cumulative').print_stats(50)
    write_contact_html(previews, args.out_dir/'contacts.html')


if __name__ == '__main__':
    main()
