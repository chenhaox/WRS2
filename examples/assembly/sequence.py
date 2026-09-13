"""M2 装配序列与路径回放；窗口内可修改移动距离。

直接运行本文件；在下面修改参数。
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from examples.assembly._shared.sequence import compute_plan
from examples.assembly._shared.sequence_display import show_sequence

CASE = "gantry"  # stack / bridge / floating / gantry；其它场景见 _shared/sequence_cases.py
METHOD = "dfs"  # dfs / beam
OUTSIDE_DISTANCE_MM = 100
PORT = 8894


def main():
    data = compute_plan(CASE, METHOD, OUTSIDE_DISTANCE_MM)
    assembly, state, graph, plan, replay = data
    print("计划:", plan.status, "独立回放:", replay["status"], flush=True)
    if plan.status != "success" or replay["status"] != "valid":
        raise RuntimeError(f"不能回放无效计划：{dict(plan.diagnostics)}")
    show_sequence(data, case=CASE, method=METHOD, outside_mm=OUTSIDE_DISTANCE_MM, port=PORT)


if __name__ == "__main__":
    main()
