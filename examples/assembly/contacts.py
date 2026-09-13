"""接触面：primitive、STL 和外部 SDF，比较 mesh / SDF 后端。

直接运行本文件；在下面修改参数。
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from dataclasses import replace
from time import perf_counter
from wrs.assembly import Assembly, ContactAnalyzer, SDFContactBackend, SDFConfig, ToleranceContactPolicy
from examples.assembly._shared.contact_cases import make_case
from examples.assembly._shared.contact_display import contact_layers
from examples.assembly._shared.contact_viewer import show

CASE = "shaft"  # boxes, ring, gap, tilted, penetration, sphere, shaft, grid,
                # bunny, bunny_small, flange, stl_cylinder
BACKEND = "sdf"  # mesh / sdf；grid 使用 sdf
RESOLUTION_M = None  # None 使用场景默认值；例如 .0005（米）
SAMPLING_SIDE = "b"  # 曲面显示 a / b / both；两侧面积不能相加
CONTACT_TOLERANCE_M = None  # 例如 .0005：将容差内 near 另记为装配接触
ALLOW_UNSIGNED_CONTACT = False  # True 允许 bunny 等符号不可靠模型的邻近表面假设
PORT = 8901


def main():
    title, description, models, config = make_case(CASE)
    if RESOLUTION_M is not None:
        config = replace(config, surface_resolution_m=RESOLUTION_M)
    if CONTACT_TOLERANCE_M is not None:
        config = replace(config, near_tol_m=CONTACT_TOLERANCE_M)
    if SAMPLING_SIDE not in ("a", "b", "both"):
        raise ValueError("SAMPLING_SIDE must be a / b / both")
    backend = BACKEND
    if BACKEND == "sdf":
        backend = SDFContactBackend(sdf_config=SDFConfig(
            open_surface="unsigned", max_query_points=500000))
    start = perf_counter()
    result = ContactAnalyzer(backend, config=config).analyze(models)
    elapsed = perf_counter() - start
    if CONTACT_TOLERANCE_M is not None:
        result = ToleranceContactPolicy(CONTACT_TOLERANCE_M, ALLOW_UNSIGNED_CONTACT).apply(result)
        accepted = sum(p.provenance["contact_policy"]["is_contact"] is True for p in result.patches)
        description += f" 容差策略接受 {accepted} 块；颜色仍是原始几何分类，不代表承载面。"
    print(title, f"{elapsed:.3f} s", dict(result.statistics), flush=True)
    assembly = Assembly(tuple(model.as_part() for model in models))
    patches = [p for p in result.patches if p.measure_kind != "near_band"
               or SAMPLING_SIDE == "both" or p.sampling_side == SAMPLING_SIDE]
    show(assembly, contact_layers(patches),
         description=f"{title} · {BACKEND} · {elapsed:.3f}s。{description}", port=PORT)


if __name__ == "__main__":
    main()
