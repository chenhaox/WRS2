"""Measure mesh-resolution error and adaptive near-band coverage independently."""
from pathlib import Path
import sys
from time import perf_counter
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import numpy as np
from wrs.assembly import Part,ContactConfig,save_report
from wrs.assembly.geometry.primitives import cylinder,pose
from wrs.assembly.geometry.proximity import MeshProximity
from wrs.assembly.geometry.mesh_bvh import QueryBudget
from wrs.assembly.contact.mesh import analyze_mesh_pair


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
OUT = ROOT / "benchmark_results" / "assembly" / "convergence.json"


def main():
    backend = MeshProximity()
    distance_rows,band_rows = [],[]
    for n in (12,24,48):
        shaft = Part('shaft',cylinder(radius=.0098,height=.01,sections=n))
        tube = Part('tube',cylinder(radius=.015,inner_radius=.01,height=.02,sections=n))
        d = backend.pair_distance(shaft,pose(),tube,pose(),budget=30000)
        row = {'sections':n,'distance_m':d.upper_bound_m,'analytic_mesh_distance_m':.0002*np.cos(np.pi/n),
               'ideal_cylinder_distance_m':.0002,'error_to_ideal_m':abs(d.upper_bound_m-.0002),'status':d.status}
        distance_rows.append(row)
        print(f'n={n}: distance={d.upper_bound_m*1000:.9f} mm',flush=True)
    # Fixed polygonal surface: this changes sampling resolution, not CAD geometry.
    shaft = Part('shaft',cylinder(radius=.0098,height=.01,sections=32))
    tube = Part('tube',cylinder(radius=.015,inner_radius=.01,height=.02,sections=32))
    pa,sa = backend.prepare(shaft.geometry)
    pb,sb = backend.prepare(tube.geometry)
    outer = next(s for s in sa if s.kind=='general')
    inner = min((s for s in sb if s.kind=='general'),key=lambda s:s.area_m2)
    for resolution in (.004,.002,.001):
        cfg = ContactConfig(surface_resolution_m=resolution,normal_angle_rad=.55,max_triangle_tests=100000,max_cells=10000)
        start = perf_counter()
        patches,coverage = analyze_mesh_pair(shaft,pose(),pa,outer,tube,pose(),pb,inner,
                                             config=cfg,backend=backend,budget=QueryBudget(100000),
                                             distance_lower_bound_m=.0002*np.cos(np.pi/32))
        area = sum(p.area_m2 for p in patches if p.sampling_side=='a')
        band_rows.append({'resolution_m':resolution,'shaft_estimated_band_area_m2':area,
                          'shaft_full_wall_area_m2':outer.area_m2,'coverage':coverage,
                          'elapsed_s':perf_counter()-start})
        print(f'resolution={resolution*1000:g} mm: shaft band={area*1e6:.6f} mm2; wall={outer.area_m2*1e6:.6f} mm2',flush=True)
    save_report({'mesh_refinement':distance_rows,'band_refinement':band_rows,
                 'note':'Near-band boundary uncertainty remains explicit; area estimates are not physical contact area.'},OUT)


if __name__=='__main__':
    main()
