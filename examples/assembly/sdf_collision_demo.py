"""Primitive-to-robot-STL collision matrix, with explicit SDF unknown cases."""
import argparse
from pathlib import Path
from time import perf_counter
import struct
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import Assembly, ContactAnalysis, ContactModel, MeshData, SDFCollisionChecker, save_report
from wrs.assembly.primitives import box, sphere, cylinder, pose
from wrs.assembly.visualization import preview_case, write_contact_html

STLS = {'bunny': 'bunny.stl', 'flange': 'link6.stl', 'cylinder_stl': 'examples/l1picking/cylinder.stl',
        'gripper_finger': 'wrs/robots/end_effectors/openarm_gripper/meshes/finger.stl',
        'gripper_hand': 'wrs/robots/end_effectors/openarm_gripper/meshes/hand.stl',
        'ur3_forearm': 'wrs/robots/manipulators/universal_robots/ur3/meshes/forearm.stl',
        'fr3_link7': 'wrs/robots/manipulators/franka/fr3/meshes/link7.stl',
        'xarm_link3': 'wrs/robots/manipulators/xarm/lite6/meshes/link3.stl'}


def torus():
    u, v = np.meshgrid(np.arange(48)*2*np.pi/48, np.arange(16)*2*np.pi/16, indexing='ij')
    vertices = np.stack(((.02+.006*np.cos(v))*np.cos(u), (.02+.006*np.cos(v))*np.sin(u), .006*np.sin(v)), axis=-1)
    faces = []
    for i in range(48):
        for j in range(16):
            a, b, c, d = i*16+j, ((i+1)%48)*16+j, ((i+1)%48)*16+(j+1)%16, i*16+(j+1)%16
            faces.extend(((a, b, c), (a, c, d)))
    return MeshData(vertices.reshape(-1, 3), faces)


def concave_l():
    xy = np.array([[0, 0], [3, 0], [3, 1], [1, 1], [1, 3], [0, 3]])*.01
    vertices = np.concatenate([np.column_stack((xy, np.full(6, z))) for z in (-.005, .005)])
    faces = []
    for i in range(1, 5):
        faces.extend(((0, i+1, i), (6, 6+i, 7+i)))
    for i in range(6):
        j = (i+1)%6
        faces.extend(((i, j, j+6), (i, j+6, i+6)))
    return MeshData(vertices, faces)


def save_stl(mesh, path):
    """Export generated fixtures, then use the same STL loading path as real files."""
    with path.open('wb') as stream:
        stream.write(b'WRS SDF collision fixture; units=m'.ljust(80, b' '))
        stream.write(struct.pack('<I', len(mesh.faces)))
        for triangle in mesh.vertices[mesh.faces]:
            n = np.cross(triangle[1]-triangle[0], triangle[2]-triangle[0])
            n /= max(np.linalg.norm(n), 1e-30)
            stream.write(struct.pack('<12fH', *n, *triangle.ravel(), 0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out-dir', type=Path, default=ROOT/'examples/assembly/output/collision')
    parser.add_argument('--repeat', type=int, default=3)
    parser.add_argument('--region-resolution-mm', type=float, default=.5,
                        help='Terminal radius for the separately timed penetration overlay')
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error('--repeat must be positive')
    if not np.isfinite(args.region_resolution_mm) or args.region_resolution_mm <= 0:
        parser.error('--region-resolution-mm must be finite and positive')
    (args.out_dir/'models').mkdir(parents=True, exist_ok=True)
    generated = {'box': box((.03, .025, .02)), 'sphere': sphere(.02), 'cylinder': cylinder(.012, .04),
                 'tube': cylinder(.02, .02, inner_radius=.01), 'torus': torus(), 'concave_l': concave_l()}
    files = {key: ROOT/value for key, value in STLS.items()}
    for key, mesh in generated.items():
        path = args.out_dir/'models'/f'{key}.stl'
        save_stl(mesh, path)
        files[key] = path
    rows, previews = [], []

    def run(key, models, expected, description, checker, prepare_s):
        if prepare_s is None:
            started = perf_counter()
            for model in models:
                checker.prepare(model)
            prepare_s = perf_counter()-started
        times = []
        for _ in range(args.repeat):
            result = checker.query(*models)
            times.append(result['timing_s']['total'])
        row = {'case': key, 'expected': expected, 'result': result, 'prepare_s': prepare_s,
               'warm_median_s': float(np.median(times)), 'warm_runs_s': times,
               'faces': [len(m.geometry.faces) for m in models]}
        # Full region extraction is deliberately outside the fast-query timer.
        regions = (checker.penetration_regions(*models, resolution_m=args.region_resolution_mm/1000,
                                               max_query_points=500000)
                   if result['status'] == 'penetrating' else None)
        row['penetration_regions'] = regions
        rows.append(row)
        diag = {'part_a': models[0].name, 'part_b': models[1].name, 'contact_backend': 'sdf_collision',
                'overlap': {'status': result['status']}, 'collision_query': result, 'expected': expected,
                'penetration_regions': regions}
        analysis = ContactAnalysis((), (diag,), result['state_digest'],
                                   statistics={'timing_s': {'backend_total': row['warm_median_s']}})
        assembly = Assembly(tuple(m.as_part() for m in models))
        preview = preview_case(key, description+f' 构造关系：{expected}。查询结果：{result["status"]}。',
                               assembly, assembly.initial_state(), analysis)
        previews.append(preview)
        if regions:
            print(f'  {key} overlay: {regions["timing_s"]["total"]:.3f}s; '
                  f'areas A/B={[round(s["area_m2"]*1e6, 3) for s in regions["sides"]]} mm2', flush=True)
        print(f"{key}: expected={expected}, got={result['status']}, {row['warm_median_s']*1000:.2f}ms, {result['reason']}", flush=True)

    for key, path in files.items():
        b = ContactModel.from_file(path, name='B', length_unit='m')
        lo, hi = b.geometry.vertices.min(0), b.geometry.vertices.max(0)
        a = ContactModel(box((*((hi-lo)[:2]+.02), .02)), 'A', pose((*(lo+hi)[:2]/2, -.01)))
        checker = SDFCollisionChecker(open_surface='unsigned', max_query_points=50000)
        start = perf_counter()
        checker.prepare(a)
        _, field = checker.prepare(b)
        prepared = perf_counter()-start
        description = (f'A=封闭支撑，B={path.name}；{len(b.geometry.faces)} 个三角形，单位 m。'
                       f'B 字段 signed={field.metadata["signed"]}。每对最多 50,000 查询点；未解析时返回 unknown。')
        for state, z, expected in (('gap', .002, 'separated'), ('touch', 0, 'touching'), ('penetration', -.002, 'penetrating')):
            run(key+'/'+state, (a, b.at(pose((0, 0, z-lo[2])))), expected, description, checker, prepared)
    tube = ContactModel(cylinder(.015, .02, 32, inner_radius=.01), 'A')
    for label, radius, x, expected in (('wide', .006, 0, 'separated'), ('narrow', .0098, 0, 'separated'),
                                      ('interference', .0098, .002, 'penetrating')):
        shaft = ContactModel(cylinder(radius, .01, 32), 'B', pose((x, 0, 0)))
        checker = SDFCollisionChecker(max_query_points=200000)
        start = perf_counter()
        for model in (tube, shaft): checker.prepare(model)
        run('shaft/'+label, (tube, shaft), expected, 'AABB 重叠的轴孔；考察窄间隙、预算和真实穿入。', checker, perf_counter()-start)
    large = ContactModel(box((.05, .05, .05)), 'A')
    for label, other in (('containment', ContactModel(box((.01, .01, .01)), 'B')),
                         ('identical', ContactModel(large.geometry, 'B'))):
        run(label, (large, other), 'penetrating', '完整包含/重合测试，不能只检查表面交线。', SDFCollisionChecker(), None)
    run('thin_crossing', (ContactModel(box((.1, .0002, .0002)), 'A'),
                          ContactModel(box((.0002, .1, .0002)), 'B')), 'penetrating',
        '两根薄杆正交穿过；不使用近接法向过滤。', SDFCollisionChecker(), None)
    run('sdf_only_separation', (large, ContactModel(large.geometry, 'B', pose((0, 0, .15)))),
        'separated', '关闭 AABB 快捷路径，用正 SDF 单元下界排除全部源表面。', SDFCollisionChecker(use_aabb=False), None)
    save_report({'scope': 'discrete collision evidence; optional penetration surfaces separately timed; warm times exclude STL loading, preparation and region extraction',
                 'python': sys.executable, 'results': rows}, args.out_dir/'results.json')
    write_contact_html(previews, args.out_dir/'contacts.html')


if __name__ == '__main__':
    main()
