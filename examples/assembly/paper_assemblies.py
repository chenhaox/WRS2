"""论文中的块状装配：接触面、装配方向和中间步骤。

直接运行本文件；在下面修改参数。
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from examples.assembly._shared.paper_cases import CATALOG
from examples.assembly._shared.paper import compute
from examples.assembly._shared.direction_viewer import show_gallery

CASE = "fig08_soma3"  # 全部场景可在窗口中切换
STAGE = 0  # 0 完整装配；1..N 为前 N 个零件
SAMPLES = 6500
NOMINAL = True  # False 检查原始网格；校正记录见 assets/paper2021/README.md
PORT = 8900


def main():
    show_gallery(tuple(CATALOG),
                 lambda key, stage: compute(key, stage, SAMPLES, NOMINAL),
                 initial=CASE, stage=STAGE, port=PORT, title="论文装配：接触面与方向")


if __name__ == "__main__":
    main()
