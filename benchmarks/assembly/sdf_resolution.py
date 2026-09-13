"""Compare SDF boundary resolutions and record geometry costs without rendering costs."""
import cProfile
from dataclasses import replace
from pathlib import Path
from time import perf_counter
import pstats
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import ContactAnalyzer, SDFContactBackend, SDFConfig, save_report
from examples.assembly._shared.stl_cases import make_case


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
CASES = ['flange', 'stl_cylinder']
RESOLUTION_MM = [1, .25, .1]
REPEAT = 3
PROFILE = False
OUT_DIR = ROOT / "benchmark_results" / "assembly" / "sdf_resolution"


def main():
    if REPEAT < 1 or any(not np.isfinite(r) or r <= 0 for r in RESOLUTION_MM):
        raise ValueError('Repetitions and resolutions must be positive')
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    rows = []
    for key in CASES:
        _, _, models, cfg = make_case(key)
        backend = SDFContactBackend(sdf_config=SDFConfig(open_surface='unsigned', max_query_points=2000000))
        for model in models:
            backend.prepare(model)
        for resolution in RESOLUTION_MM:
            config = replace(cfg, surface_resolution_m=resolution/1000, max_cells=200000)
            analyzer = ContactAnalyzer(backend, config=config)
            runs = []
            for _ in range(REPEAT):
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
            save_report(result, OUT_DIR/f'{key}.{resolution:g}mm.contacts.json')
            save_report({'python': sys.executable, 'scope': 'warm full analysis; excludes loading, rendering and profiler overhead',
                         'results': rows}, OUT_DIR/'measurements.json')
            if PROFILE:
                profiler = cProfile.Profile()
                profiler.runcall(analyzer.analyze, models)
                with (OUT_DIR/f'{key}.{resolution:g}mm.profile.txt').open('w', encoding='utf-8') as stream:
                    pstats.Stats(profiler, stream=stream).strip_dirs().sort_stats('cumulative').print_stats(50)


if __name__ == '__main__':
    main()
