"""Reproducible CPU BVH cold/warm and bounded-memory measurements (not a speed claim)."""
from pathlib import Path
import sys
import platform
from time import perf_counter
import tracemalloc

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
import numpy as np
import scipy
from wrs.assembly import Part, save_report
from wrs.assembly.primitives import sphere, pose
from wrs.assembly.geometry.proximity import MeshProximity


# 直接修改参数；报告只写到仓库根目录的 benchmark_results/。
OUT = ROOT / "benchmark_results" / "assembly" / "proximity.json"


def main():
    mesh = sphere(sections=32,rings=16)
    a,b = Part('a',mesh),Part('b',mesh)
    backend = MeshProximity()
    measurements = []
    for temperature in ('cold','warm'):
        # Tracemalloc adds overhead; measure latency separately from instrumented peak.
        start = perf_counter()
        result = backend.pair_distance(a,pose(),b,pose((.12,.03,.02)),budget=30000)
        elapsed = perf_counter()-start
        measurements.append({'cache':temperature,'elapsed_s':elapsed,'result':result})
    tracemalloc.start()
    MeshProximity().pair_distance(a,pose(),b,pose((.12,.03,.02)),budget=30000)
    _,peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    report = {'python':sys.executable,'version':sys.version,'platform':platform.platform(),
              'numpy':np.__version__,'scipy':scipy.__version__,'faces_per_instance':len(mesh.faces),
              'dense_pair_count':len(mesh.faces)**2,'tracemalloc_peak_bytes':peak,
              'peak_scope':'Python/NumPy traced allocations for cold query; not total process RSS',
              'queries':measurements,'cache_hits':backend.cache_hits}
    save_report(report,OUT)
    for m in measurements:
        print(f'{m["cache"]}: {m["elapsed_s"]:.4f}s; {m["result"].triangle_tests} triangle pairs; {m["result"].status}')
    print(f'Peak traced: {peak/2**20:.3f} MiB; dense pair count: {len(mesh.faces)**2}')


if __name__ == '__main__':
    main()
