"""静力平衡：接触点、摩擦锥、作用力和有限辅助支撑。

直接运行本文件；在下面修改参数。
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from examples.assembly._shared.stability import show

CASE = "bridge"  # 窗口内可切换全部稳定性场景
REDUCE_CONTACT_POINTS = True
AUX_SUPPORTS = True
CURVED_POINT_BUDGET = 64
PORT = 8893


def main():
    show(CASE, port=PORT, reduce_contact_points=REDUCE_CONTACT_POINTS,
         aux_supports=AUX_SUPPORTS, curved_point_budget=CURVED_POINT_BUDGET)


if __name__ == "__main__":
    main()
