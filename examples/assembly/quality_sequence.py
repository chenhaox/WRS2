"""S/G/A 序列搜索：稳定性、合格抓取数量与可装配性。

直接运行本文件；在下面修改参数。
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from wrs.assembly import replay_sequence
from examples.assembly._shared.quality import compute, show

CASE = "counterweight"  # stack / bridge / counterweight
BACKEND = "cuda"  # cuda / numpy / highs
DIRECTION_COUNT = 300
PRUNE = True
PORT = 8898


def main():
    data = compute(CASE, BACKEND, DIRECTION_COUNT, PRUNE)
    assembly, analyzer, hand, result = data
    print(result.status, "score:", result.score, flush=True)
    if result.plan is None:
        raise RuntimeError(f"未找到计划：{dict(result.diagnostics)}")
    replay = replay_sequence(assembly, result.plan)
    print("顺序:", [step.part_id for step in result.plan.assembly_steps],
          "独立回放:", replay["status"], flush=True)
    if replay["status"] != "valid":
        raise RuntimeError(f"计划回放失败：{replay}")
    show(*data, PORT)


if __name__ == "__main__":
    main()
