"""多方向稳定性：六维载荷的力 / 力矩投影，GPU 批量求解。

直接运行本文件；在下面修改参数。
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from wrs.assembly import StabilitySweepConfig, DirectionalStabilityAnalyzer
from examples.assembly._shared.stability_cases import make_case, case_supports
from examples.assembly._shared.stability_sweep_display import show_sweep

CASE = "bridge"  # stack / bridge / floating / tripod
BACKEND = "cuda"  # cuda / numpy / highs
MODE = "wrench"  # wrench / force / torque / legacy_coupled
DIRECTION_COUNT = 300
MAX_SCORE_N = 100
FORCE_REFERENCE_N = 10.
TORQUE_REFERENCE_NM = 1.
PORT = 8897


def main():
    assembly, state, graph = make_case(CASE)
    config = StabilitySweepConfig(
        backend=BACKEND, mode=MODE, direction_count=DIRECTION_COUNT,
        max_score_n=MAX_SCORE_N, force_reference_n=FORCE_REFERENCE_N,
        torque_length_m=TORQUE_REFERENCE_NM / FORCE_REFERENCE_N)
    analyzer = DirectionalStabilityAnalyzer(
        assembly, state, graph, config=config, supports=case_supports(CASE))
    result = analyzer.analyze()
    print(result.status, "采样方向的最小承载极限:", result.sampled_minimum_n,
          "N-equivalent", dict(result.diagnostics), flush=True)
    show_sweep(analyzer, result, port=PORT)


if __name__ == "__main__":
    main()
