"""Independently check bunny area using direct triangle/horizontal-plane clipping."""
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from wrs.assembly import ContactAnalyzer, SDFContactBackend, SDFConfig, save_report
from wrs.assembly.geometry.preprocess import prepare_mesh
from examples.assembly._shared.stl_cases import make_case


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
OUTPUT = ROOT / "benchmark_results" / "assembly" / "bunny_area.json"


def polygon_area(poly):
    return sum(np.linalg.norm(np.cross(poly[i]-poly[0], poly[i+1]-poly[0]))/2
               for i in range(1, len(poly)-1))


def main():
    rows = []
    for key in ('bunny', 'bunny_small'):
        _, _, models, config = make_case(key)
        b = models[1]
        prep = prepare_mesh(b.geometry)
        triangles = prep.mesh.vertices[prep.mesh.faces] @ b.tf[:3, :3].T+b.tf[:3, 3]
        reference = 0.0
        # The support top is z=0 and fully covers bunny's XY bounds. Every
        # input vertex has z>=0.2mm, so only the upper band plane clips it.
        for triangle, normal in zip(triangles, prep.normals @ b.tf[:3, :3].T):
            if normal[2] > -np.cos(config.normal_angle_rad):
                continue
            polygon = []
            for i, q in enumerate(triangle):
                p = triangle[i-1]
                dp, dq = config.near_tol_m-p[2], config.near_tol_m-q[2]
                if (dp >= 0) != (dq >= 0):
                    polygon.append(p+(q-p)*dp/(dp-dq))
                if dq >= 0:
                    polygon.append(q)
            reference += polygon_area(polygon)
        analysis = ContactAnalyzer(SDFContactBackend(sdf_config=SDFConfig(open_surface='unsigned',
                                  max_query_points=500000)), config=config).analyze(models)
        row = {'case': key, 'reference_b_area_mm2': reference*1e6, 'sides': {}}
        for side in ('a', 'b'):
            patches = [p for p in analysis.patches if p.sampling_side == side]
            cells = [c for p in patches for region in p.regions for c in region.cells_world_m]
            z = np.concatenate(cells)[:, 2]*1000
            row['sides'][side] = {'reported_area_mm2': sum(p.area_m2 for p in patches)*1e6,
                                  'cell_area_sum_mm2': sum(polygon_area(c) for c in cells)*1e6,
                                  'weights_sum_mm2': sum(p.weights.sum() for p in patches)*1e6,
                                  'z_range_mm': [float(z.min()), float(z.max())],
                                  'region_count': sum(len(p.regions) for p in patches)}
        row['b_difference_mm2'] = row['sides']['b']['reported_area_mm2']-row['reference_b_area_mm2']
        if abs(row['b_difference_mm2']) > .002:
            raise AssertionError(row)
        rows.append(row)
        print(row, flush=True)
    save_report(rows, OUTPUT)


if __name__ == '__main__':
    main()
