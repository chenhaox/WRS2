# Assembly benchmark 与批量审查

这些脚本不启动 WRS 窗口。参数在文件顶部修改，直接运行：

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' benchmarks/assembly/planning_api.py
```

| 文件 | 测量 / 检查 |
|---|---|
| proximity.py | CPU BVH 冷热查询与内存 |
| convergence.py | 网格精度与近接触带分辨率 |
| bunny_area.py | Bunny 面积的独立几何复核 |
| sdf_resolution.py | STL SDF 分辨率、计时、可选 profile |
| candidate_directions.py | 旧候选 API 分阶段 profile |
| direction_methods.py | Fibonacci / LP / SOCP / QP 算法实验 |
| planning_api.py | 完整生产方向 API 与静力求解计时 |
| stability.py | 静力模型各阶段计时 |
| stability_cases.py | 多种稳定性场景及接触点约简对照 |
| stability_sweep.py | 六维载荷的 CPU / GPU 批量计算对照 |
| execution.py | M2 计划与回放，按需检查 M3 单 / 双臂 |
| readability.py | 可读性重构前后相同输入的结果、搜索工作量与耗时对照 |
| nine_cases.py | 九种接触、方向、维数和可装配性分数 |
| paper_cases.py | 论文完整装配、各前缀及 raw / nominal 证据 |

报告集中写到仓库根目录 `benchmark_results/assembly/`，已被 Git 忽略。普通 WRS 例子不会写报告。

独立 HTML 导出已移除。`sdf_resolution.py` 保留分辨率、面积、覆盖率和各阶段计时的 JSON 报告；可视化使用 [WRS 例子](../../examples/assembly/README.md)。

`paper_cases.py` 的 `PREFIXES = True` 检查中间步骤，`NOMINAL = False` 检查原始数据；结果保留未知和缺失来源。`execution.py` 的 `ROBOT` / `AUXILIARY` 控制是否做较慢的机器人验证。`_candidate_fixtures.py` 和 `_direction_fixtures.py` 仅保存历史比较的输入，不是启动入口。

计时报告注明运行范围和配置，性能数字依赖机器和负载；本次整理未替换核心求解方法。
