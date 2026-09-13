"""九类装配方向：WRS 中切换 SOCP 与 Fibonacci，显示实际接触面。

直接运行本文件；在下面修改参数。
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from examples.assembly._shared.direction_cases import CASES
from examples.assembly._shared.directions import load_case
from examples.assembly._shared.direction_viewer import show_gallery

CASE = "g"  # a..i；窗口内也可以切换
SAMPLES = 6500
PORT = 8899


def main():
    show_gallery(tuple(CASES), lambda key, stage: load_case(key, SAMPLES),
                 initial=CASE, port=PORT, title="九类装配方向")


if __name__ == "__main__":
    main()
