"""Run M1 analytic examples and write manifests, contact evidence and an HTML gallery."""
import argparse
from pathlib import Path
import sys
from time import perf_counter

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np
from wrs.assembly import Assembly, Part, ContactConfig, GeometryConfig, analyze_contacts, load_assembly, save_assembly, save_report
from wrs.assembly.primitives import box, rectangle, rectangular_ring, cylinder, sphere, pose
from wrs.assembly.geometry.proximity import MeshProximity
from wrs.assembly.adapters.legacy import legacy_rotation
from wrs.assembly.visualization import preview_case, write_contact_html, build_wrs_scene


def cases():
    """Return independent deterministic cases; dimensions and expected areas in SI."""
    def pair(a,b,tf=None):
        return Assembly((Part('A',a,fixed=True),Part('B',b,pose() if tf is None else tf)))
    default = ContactConfig()
    curve = ContactConfig(near_tol_m=.0005, surface_resolution_m=.003,
                          normal_angle_rad=.55,max_cells=4000,max_triangle_tests=30000)
    shaft_config = ContactConfig(near_tol_m=.0005, surface_resolution_m=.003,
                                 normal_angle_rad=.55,max_cells=4000,max_triangle_tests=100000)
    return {
        'boxes': ('箱体的部分面接触', '两个 100 mm 箱体横向错开 25 mm。解析接触面积为 7,500 mm²。',
                  pair(box(),box(),pose((.025,0,.1))), default, .0075),
        'ring': ('带孔接触面', '100 mm 方环，中孔 40 mm；接触面积 8,400 mm²。俯视可检查孔洞和完整边界。',
                 pair(rectangular_ring(),rectangle(upward=False)), default, .0084),
        'gap': ('0.5 μm 正间隙', '虽然间隙很小，默认仍是 near，active 面积为零；没有静默吸附。',
                pair(box(),box(),pose((0,0,.1000005))), default, 0),
        'tilted': ('倾斜面的线接触', '绕 Y 轴旋转 0.02 rad；间隙沿平面变化。零间隙线、正间隙带和局部干涉分别记录。',
                   pair(rectangle(),rectangle(upward=False),pose(rotation=legacy_rotation([0,.02,0]))), default, 0),
        'penetration': ('独立的穿透诊断', '两个箱体重叠 10 mm。实体诊断必须报告 penetrating，不能仅依据局部接触区域判断。',
                        pair(box(),box(),pose((0,0,.09))), default, 0),
        'sphere': ('球面与平面切触', '半径 25 mm 的离散球面与平面切触。显示的是 0.5 mm 近接触带估计，不能当成有限承载面积。',
                   pair(rectangle(),sphere(sections=16,rings=8),pose((0,0,.025))),curve,0),
        'shaft': ('轴孔的正间隙', '轴半径 9.8 mm、孔半径 10 mm；径向设计间隙 0.2 mm。橙色是 0.5 mm 距离阈值内的近接触带，实际接触面积为零。A 是孔壁，B 是轴壁；孔壁端部的带边界仍有分辨率误差。',
                  pair(cylinder(radius=.015,inner_radius=.01,height=.02,sections=32),
                       cylinder(radius=.0098,height=.01,sections=32)),shaft_config,0),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=list(cases()),default='boxes')
    parser.add_argument('--all',action='store_true')
    parser.add_argument('--manifest',type=Path,help='Analyze a wrs.assembly/1 JSON manifest instead')
    parser.add_argument('--out-dir',type=Path,default=ROOT/'examples'/'assembly'/'output')
    parser.add_argument('--viewer',action='store_true',help='Publish the selected case through the optional WRS viewer')
    args = parser.parse_args()
    if args.all and args.viewer:
        parser.error('--viewer requires a single case')
    selected = cases() if args.all else {args.case:cases()[args.case]}
    if args.manifest:
        selected = {'manifest': ('输入装配',str(args.manifest),load_assembly(args.manifest),ContactConfig(),None)}
    previews, summary = [], []
    print(f'Python: {sys.executable}\nWRS checkout: {ROOT}',flush=True)
    for key,(name,description,assembly,config,expected) in selected.items():
        state = assembly.initial_state()
        backend = MeshProximity(geometry_config=GeometryConfig(smooth_angle_rad=.8) if key=='sphere' else None)
        start = perf_counter()
        result = analyze_contacts(assembly,state,config=config,backend=backend)
        elapsed = perf_counter()-start
        area = sum(p.area_m2 for p in result.patches if p.dimension==2 and p.classification=='active')
        if expected is not None and not np.isclose(area,expected,rtol=1e-7,atol=1e-12):
            raise AssertionError(f'{key}: active area {area}, expected {expected}')
        if key == 'shaft':
            wall_area = 2*32*.0098*np.sin(np.pi/32)*.01
            shaft_patches = [p for p in result.patches if p.measure_kind=='near_band' and p.sampling_side=='b']
            if (not np.isclose(sum(p.area_m2 for p in shaft_patches),wall_area,rtol=1e-8)
                    or sum(len(p.regions) for p in shaft_patches)!=1
                    or any('budget_exhausted' in c['reason'] for d in result.pair_diagnostics for c in d['curved_coverage'])):
                raise AssertionError('Shaft example must cover the full wall without exhausting either side budget')
        save_assembly(assembly,args.out_dir/f'{key}.assembly.json')
        save_report(result,args.out_dir/f'{key}.contacts.json')
        previews.append(preview_case(name,description,assembly,state,result))
        row = {'case':key,'active_area_mm2':area*1e6,'patches':len(result.patches),
               'overlap':[d['overlap']['status'] for d in result.pair_diagnostics],
               'unresolved_area_mm2':sum(d['unresolved_area_m2'] for d in result.pair_diagnostics)*1e6,
               'elapsed_s':elapsed,'config':config,'geometry_config':backend.geometry_config}
        summary.append(row)
        print(f"{key}: active={area*1e6:.3f} mm2, overlap={row['overlap']}, {elapsed:.3f}s",flush=True)
    save_report(summary,args.out_dir/'summary.json')
    write_contact_html(previews,args.out_dir/'contacts.html')
    print(f'Preview: {(args.out_dir/"contacts.html").resolve()}',flush=True)
    if args.viewer:
        from wrs import wvw
        base = wvw.World(cam_pos=(.25,.25,.2))
        base.set_scene(build_wrs_scene(assembly,state,result))
        base.set_caption(f'Assembly M1 - {args.case}')
        base.run()


if __name__ == '__main__':
    main()
