"""M2.04 local cones: face, channel, corner, enclosure, finite gap and SDF shaft."""
import argparse
from pathlib import Path
from dataclasses import replace
from time import perf_counter
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import (Assembly, Part, MatingRelation, ContactConfig, ConstraintConfig,
                          SDFContactBackend, SDFConfig, analyze_contacts, build_contact_graph,
                          candidate_motions, save_report)
from wrs.assembly.primitives import box, cylinder, pose
from wrs.assembly.visualization import preview_case, write_contact_html


def fixtures():
    """Return actual geometry fixtures and explicit moving IDs/backend controls."""
    part = Part('part', box())
    base = Part('floor', box((.18, .18, .02)), pose((0, 0, -.06)), fixed=True)
    top = replace(base, part_id='ceiling', assembled_tf=pose((0, 0, .06)))
    left = Part('left', box((.02, .1, .1)), pose((-.06, 0, 0)), fixed=True)
    back = Part('back', box((.1, .02, .1)), pose((0, -.06, 0)), fixed=True)
    rows = [
        ('face', '单面：允许分离与切向滑动，禁止向支撑内部运动。', Assembly((base, part)), 'mesh', ConstraintConfig()),
        ('channel', '对向面：上下均受约束，保留 XY 平面内滑动。', Assembly((base, part, top)), 'mesh', ConstraintConfig()),
        ('corner_twist', '角落六维候选：所有接触单元顶点参与旋转约束。紫色箭头表示角速度轴。',
         Assembly((base, left, back, part)), 'mesh', ConstraintConfig(mode='twist')),
        ('enclosed', '六面局部平移约束：没有非零平移方向。该结论只针对当前刚体接触模型。',
         Assembly((base, top, left, back, replace(left, part_id='right', assembled_tf=pose((.06, 0, 0))),
                   replace(back, part_id='front', assembled_tf=pose((0, .06, 0))), part)), 'mesh', ConstraintConfig()),
        ('near_gap', '零件下方有 0.2 mm 正间隙：没有当前 active 约束，向下候选报告预计间隙闭合参数。',
         Assembly((base, replace(part, assembled_tf=pose((0, 0, .0002))))), 'mesh', ConstraintConfig())]
    shaft = Part('part', cylinder(.0098, .01, 32))
    tube = Part('tube', cylinder(.015, .02, 32, inner_radius=.01), fixed=True)
    rows.append(('shaft_sdf', 'SDF 正间隙轴孔：保留径向 near 法线场和 coaxial mating；不生成虚假的承载/运动约束。',
                 Assembly((shaft, tube), mating_relations=(MatingRelation('part', 'tube', 'coaxial'),)),
                 SDFContactBackend(sdf_config=SDFConfig(max_query_points=50000, validate_mesh_overlap=False)),
                 ConstraintConfig()))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, default=ROOT/'examples/assembly/output/m2')
    args = parser.parse_args()
    previews, summary = [], []
    for key, description, assembly, backend, cfg in fixtures():
        state = assembly.initial_state()
        start = perf_counter()
        analysis = analyze_contacts(assembly, state, backend=backend,
                                    config=ContactConfig(surface_resolution_m=.001))
        analysis_s = perf_counter()-start
        start = perf_counter()
        graph = build_contact_graph(assembly, state, analysis)
        graph_s = perf_counter()-start
        times = []
        # First call can initialize HiGHS; record it separately from warm reuse.
        for _ in range(4):
            start = perf_counter()
            motions = candidate_motions(graph, ['part'], config=cfg)
            times.append(perf_counter()-start)
        row = {'case': key, 'analysis_s': analysis_s, 'graph_s': graph_s,
               'candidates_first_s': times[0], 'candidates_warm_median_s': float(np.median(times[1:])),
               'rows': len(motions.constraints.matrix_world), 'candidates': len(motions.candidates),
               'status': motions.status, 'issues': motions.constraints.issues}
        summary.append(row)
        save_report({'graph': graph, 'motions': motions, 'timing': row}, args.out_dir/f'{key}.json')
        preview = preview_case(key, description, assembly, state, analysis)
        preview.update(motion_candidates=motions, motion_timing_s=row['candidates_warm_median_s'])
        previews.append(preview)
        print(f"{key}: {row['status']}, rows={row['rows']}, candidates={row['candidates']}, "
              f"warm={row['candidates_warm_median_s']*1000:.2f}ms", flush=True)
    save_report({'python': sys.executable, 'cases': summary}, args.out_dir/'summary.json')
    write_contact_html(previews, args.out_dir/'contacts.html')


if __name__ == '__main__':
    main()
