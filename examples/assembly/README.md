# Assembly：直接运行的 WRS 例子

在 WRS2 根目录运行；**参数直接在脚本顶部修改，无需命令行参数**。也可在 IDE 中选用 `D:\code\venv312\.venv\Scripts\python.exe` 后运行文件。

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/contacts.py
```

| 文件 | 内容 | 默认端口 |
|---|---|---:|
| [contacts.py](contacts.py) | primitive / STL / 外部 GridSDF；mesh、SDF 接触面 | 8901 |
| [collision.py](collision.py) | SDF 间隙、touch、穿透；点 / 线 / 面及穿入表面 | 8902 |
| [directions.py](directions.py) | 九类真实接触；窗口切换 SOCP / Fibonacci | 8899 |
| [paper_assemblies.py](paper_assemblies.py) | 论文 20 个图号入口、装配前缀、接触与方向 | 8900 |
| [stability.py](stability.py) | 多种结构的受力点、摩擦锥、平衡与辅助支撑 | 8893 |
| [stability_sweep.py](stability_sweep.py) | 六维扰动承载极限；CUDA / NumPy / HiGHS | 8897 |
| [sequence.py](sequence.py) | M2 DFS / beam、复杂结构、移动距离与路径回放 | 8894 |
| [quality_sequence.py](quality_sequence.py) | S/G/A 正向质量搜索与实际合格抓取 | 8898 |
| [robot_execution.py](robot_execution.py) | M3 单 / 双臂机器人验证与回放 | 8895 |

例如在 `contacts.py` 顶部设置 `CASE = "flange"`、`BACKEND = "sdf"`；在 `collision.py` 设置 `STATE = "touch"`。接触例子的 `SAMPLING_SIDE` 选择曲面 A/B 侧；`CONTACT_TOLERANCE_M` 和 `ALLOW_UNSIGNED_CONTACT` 保留容差接触策略。目录内只有这九个启动入口。窗口运行于本地端口；再次运行同一例子前先停止旧进程，或修改 `PORT`。

普通例子只计算并显示，不保存 JSON、HTML 或日志，不创建 `output/`。`_shared/` 保存复用的场景、计算及控件，核心算法仍在 `wrs/assembly/`。接触显示保持原始分类：近接触带不是承载面；穿入表面不是相交体积；unknown 不会被画成已验证方向。曲面 A / B 侧都是独立估计，不能将两侧面积相加作为接触面积。

论文输入保留在 [assets/paper2021](assets/paper2021/README.md)，包含原始文件、哈希和名义几何校正说明；缺少原数据的替代场景仍有明确标注。

[benchmarks/assembly](../../benchmarks/assembly/README.md) 单独存放计时、收敛、批量论文检查。唯一保留的独立 HTML 生成器是 `sdf_resolution.py`：比较不同 SDF 分辨率，可将顶部 `EXPORT_HTML` 设为 `True`；结果写到仓库根目录 `benchmark_results/assembly/`。

清单规划和旧数据盘点属于工具，分别在 [tools/assembly/plan_manifest.py](../../tools/assembly/plan_manifest.py)、[legacy_inventory.py](../../tools/assembly/legacy_inventory.py)，保留它们所需的 CLI。算法说明与历史测量见 [assembly 文档](../../docs/assembly_planner/README.zh.md)。
