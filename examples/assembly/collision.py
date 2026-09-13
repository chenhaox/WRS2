"""SDF 碰撞：显示实体关系、相交表面和 touch 的点 / 线 / 面。

直接运行本文件；在下面修改参数。
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from wrs.assembly import Assembly, SDFCollisionChecker
from examples.assembly._shared.collision_cases import make_case
from examples.assembly._shared.collision_display import collision_layers
from examples.assembly._shared.contact_viewer import show

CASE = "box"  # box, sphere, cylinder, tube, torus, concave_l, shaft；
              # bunny, flange, cylinder_stl；其它 STL 见 _shared/collision_cases.py
STATE = "penetration"  # gap / touch / penetration
REGION_RESOLUTION_M = .0005
MAX_QUERY_POINTS = 500000  # 预算耗尽时保留 unknown；可按模型复杂度调大
PORT = 8902


def main():
    models, expected = make_case(CASE, STATE)
    checker = SDFCollisionChecker(open_surface="unsigned", max_query_points=MAX_QUERY_POINTS)
    result = checker.query(*models)
    print(f"{CASE}: {result['status']} (预期 {expected})", result, flush=True)
    layers = collision_layers(checker, models, result, REGION_RESOLUTION_M)
    assembly = Assembly(tuple(model.as_part() for model in models))
    show(assembly, layers, port=PORT,
         description=f"{CASE} / {STATE}：{result['status']}。{result.get('reason', '')}")


if __name__ == "__main__":
    main()
