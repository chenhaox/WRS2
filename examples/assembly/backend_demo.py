"""Compare mesh/SDF contact backends, including an externally supplied SDF grid."""
import argparse
from pathlib import Path
from time import perf_counter
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from wrs.assembly import (Assembly, ContactModel, ContactAnalyzer, ContactConfig, GridSDF,
                          SDFContactBackend, SDFConfig, load_assembly, save_report)
from wrs.assembly.primitives import box, pose
from wrs.assembly.visualization import preview_case, write_contact_html
from contact_demo import cases
from stl_cases import make_case, FILES


def grid_case():
    """Two boxes represented by sampled analytical SDFs; x,y,z array ordering."""
    axis = np.linspace(-.008, .008, 81)
    xyz = np.stack(np.meshgrid(axis, axis, axis, indexing='ij'), axis=-1)
    q = np.abs(xyz)-.002
    values = np.linalg.norm(np.maximum(q, 0), axis=-1)+np.minimum(q.max(axis=-1), 0)
    spacing = float(axis[1]-axis[0])
    # For exact samples of a 1-Lipschitz SDF, trilinear interpolation error is
    # <= sqrt(sum(h_i**2))/2 by convex weights and the weighted variance bound.
    grid = GridSDF(values, [axis[0]]*3, spacing, error_bound_m=np.sqrt(3)*spacing/2)
    mesh = box((.004, .004, .004))
    a = ContactModel(mesh, 'A', representations={'sdf': grid})
    b = ContactModel(mesh, 'B', pose((0, 0, .005)), {'sdf': grid})
    return ('传入体素 SDF：箱体间隙',
            '两个 4 mm 箱体相距 1 mm。SDF 数组步长 0.2 mm，声明插值误差上界约 0.173 mm；'
            '面积在输入网格上积分，网格场的全局实体关系保留 unknown。',
            (a, b), ContactConfig(near_tol_m=.002, surface_resolution_m=.0002,
                                  normal_angle_rad=.55, max_cells=20000))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('mesh', 'sdf', 'both'), default='both')
    parser.add_argument('--case', choices=(*cases(), 'grid', *FILES), default='shaft')
    parser.add_argument('--all', action='store_true')
    parser.add_argument('--manifest', type=Path, help='Existing wrs.assembly/1 manifest')
    parser.add_argument('--out-dir', type=Path, default=Path(__file__).with_name('output')/'backends')
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    backends = {'mesh': 'mesh', 'sdf': SDFContactBackend(sdf_config=SDFConfig(open_surface='unsigned', max_query_points=500000))}
    selected = ('mesh', 'sdf') if args.backend == 'both' else (args.backend,)
    inputs = {}
    for key, (name, description, assembly, config, _) in cases().items():
        if key == 'shaft':
            from dataclasses import replace
            config = replace(config, surface_resolution_m=.001, max_cells=20000, max_triangle_tests=500000)
        inputs[key] = (name, description, tuple(ContactModel.from_part(p) for p in assembly.parts), config)
    inputs['grid'] = grid_case()
    inputs.update({key: make_case(key) for key in FILES})
    keys = tuple(inputs) if args.all else (args.case,)
    if args.manifest:
        assembly = load_assembly(args.manifest)
        inputs['manifest'] = (args.manifest.stem, '由装配清单导入的模型；SDF 结果为近接触带估计。',
                              tuple(ContactModel.from_part(p) for p in assembly.parts), ContactConfig())
        keys = ('manifest',)
    previews, summary = [], []
    for key in keys:
        name, description, models, config = inputs[key]
        assembly = Assembly(tuple(m.as_part() for m in models))
        for selected_backend in selected:
            analyzer = ContactAnalyzer(backends[selected_backend], config=config)
            start = perf_counter()
            result = analyzer.analyze(models)
            elapsed = perf_counter()-start
            suffix = (' SDF 输出是估计带，零间隙保持 unknown，未计算 active 面积；开放表面明确采用无符号距离。'
                      if selected_backend == 'sdf' else '')
            previews.append(preview_case(f'{name} / {selected_backend}', description+suffix,
                                         assembly, assembly.initial_state(), result))
            save_report(result, args.out_dir/f'{key}.{selected_backend}.contacts.json')
            row = {'case': key, 'backend': selected_backend, 'elapsed_s_including_prepare': elapsed,
                   'area_a_mm2': sum(p.area_m2 for p in result.patches if p.sampling_side == 'a')*1e6,
                   'area_b_mm2': sum(p.area_m2 for p in result.patches if p.sampling_side == 'b')*1e6,
                   'active_area_mm2': sum(p.area_m2 for p in result.patches if p.classification == 'active')*1e6,
                   'overlap': [d['overlap']['status'] for d in result.pair_diagnostics],
                   'unprocessed_area_mm2': sum(r['unprocessed_area_m2'] for d in result.pair_diagnostics
                                               for r in d.get('curved_coverage', ()))*1e6,
                   'statistics': result.statistics}
            if key == 'shaft':
                expected = 2*32*.0098*np.sin(np.pi/32)*.01*1e6
                band = [p for p in result.patches if p.sampling_side == 'b']
                assert np.isclose(row['area_b_mm2'], expected, rtol=1e-8), row
                assert sum(len(p.regions) for p in band) == 1
                assert row['active_area_mm2'] == 0 and row['unprocessed_area_mm2'] == 0
            summary.append(row)
            print(f"{key}/{selected_backend}: A={row['area_a_mm2']:.3f}, B={row['area_b_mm2']:.3f} mm2, "
                  f"active={row['active_area_mm2']:.3f}, overlap={row['overlap']}, {elapsed:.3f}s", flush=True)
    save_report(summary, args.out_dir/'summary.json')
    write_contact_html(previews, args.out_dir/'contacts.html')
    print(f'Preview: {(args.out_dir/"contacts.html").resolve()}', flush=True)


if __name__ == '__main__':
    main()
