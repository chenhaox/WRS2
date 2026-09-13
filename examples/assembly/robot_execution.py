"""M3：RS007L + OR2FG7 机器人装配验证与 WRS 回放。

直接运行本文件；在下面修改参数。
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from examples.assembly._shared.robot import compute, show

AUXILIARY = False  # True 使用第二台机器人辅助支撑
PORT = 8895  # 同时启动双臂例子时可改成 8896


def main():
    assembly, cell, result = compute(AUXILIARY)
    show(assembly, cell, result, port=PORT)


if __name__ == "__main__":
    main()
