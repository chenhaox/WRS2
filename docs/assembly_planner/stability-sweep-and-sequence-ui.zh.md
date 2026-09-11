# 多方向稳定性、CUDA 批量求解与 M2 距离控制

2026-09-11，指定 Python `D:/code/venv312/.venv/Scripts/python.exe`。无新增或升级依赖。当前环境 PyTorch 2.9.0+cu128、CUDA 12.8，可用 RTX 5060 Laptop GPU，约 8 GB 显存。CPU 接口不会导入 PyTorch；显式 `backend='cuda'` 才加载它。

## 直接运行

在 WRS2 根目录：

```powershell
$assemblyPython = 'D:\code\venv312\.venv\Scripts\python.exe'
# 8897：选择部件、方向或最弱方向；查看接触点/摩擦锥、方向球与载荷数值
& $assemblyPython examples/assembly/stability_sweep_demo.py --case bridge --backend cuda
# 复现旧版两个独立幅值的扰动极限结构
& $assemblyPython examples/assembly/stability_sweep_demo.py --mode legacy_coupled --backend cuda --headless
# 可设置方向数；查询上限不是物理承载上限
& $assemblyPython examples/assembly/stability_sweep_demo.py --directions 6500 --max-score 100 --headless
# 8894：9 活动零件框架，改变余量、重新规划、选择步骤/移动距离
& $assemblyPython examples/assembly/sequence_demo.py --case gantry --outside-mm 100
& $assemblyPython examples/assembly/sequence_demo.py --case sleeve --headless
# CPU/GPU 同模型对照；无 CUDA 的环境使用 --backends highs numpy
& $assemblyPython examples/assembly/benchmark_stability_sweep.py --counts 300 6500 --repeats 3
```

输出在 `examples/assembly/output/stability_sweep/`、`output/sequence/`。报告含输入摘要、每方向结果、回退数量、准备/查询/证书复核时间。方向固定，可重复；没有每次换一组随机点。

## 算的是什么

旧版每次选一个自由部件和一个扰动方向，求最大可承受幅值。新版保留这一含义：**同一批 LP 独立求解，不把它们的外力叠加**。模型位置、质量、COM、重力、接触区域、摩擦和支撑在整批内固定。若要将整个装配体旋转后计算，需要另建姿态/重力输入，本接口不做该变换。

所有自由部件共同满足 `A λ + w₀ + D α = 0`，并有 `λ >= 0`、有限接触/辅助支撑容量。`A` 的每个内部接触列同时向两物体施加等大反向力及各自 COM 上的力矩。没有给未接触部件凭空加支撑。接触点和 16 棱摩擦锥沿用现有静力模型；方向极限始终使用完整曲面力点，平面的精确凸包约简仍可保留。

| mode | 扰动 | 最大化的分数 |
| --- | --- | --- |
| `force` | `F = α u`，`τ = 0`，u 为单位向量 | α，单位 N |
| `torque` | `F = 0`，`τ = L α v` | α，单位 N 等效；实际力矩为 Lα N·m |
| `wrench` | 固定六维方向 `[u,v]` 归一化后，`[F,τ]=[αu,Lαv]` | α，保持选定力/力矩比例 |
| `legacy_coupled` | `F=a u, τ=L b v`，u/v 各自归一化，`Lb <= r a` | a+b，即 F+T/L；允许两个幅值独立变化 |

L 默认 0.1 m，是声明的评分尺度；r 是该部件局部网格顶点到 COM 的最大距离。旧版 `kf+kt` 的牛顿/牛顿米直接相加未照搬，所以新的 legacy 分数不应直接与旧分数混排。

纯力/纯力矩方向包括六个坐标方向，其余用 Fibonacci 公式。混合模式用固定、不扰乱的 Halton 序列生成两个球面方向，另显式加入同向/反向轴。它覆盖的是有限的力/力矩方向组合；`wrench` 的默认采样不是整个六维单位球均匀覆盖。需要其它比例可直接传 N×6 方向数组。

```python
from wrs.assembly import StabilitySweepConfig, DirectionalStabilityAnalyzer

analyzer = DirectionalStabilityAnalyzer(
    assembly, state, graph,
    config=StabilitySweepConfig(backend='cuda', mode='force', direction_count=300),
    supports=support_candidates,
)
result = analyzer.analyze()               # 每个自由部件各 300 个方向
selected = analyzer.analyze([[1,0,0]], part_ids=['upper'])  # 复用同一接触模型
print(result.sampled_minimum_n, result.diagnostics)
```

`optimal` 是当前离散接触模型上的数值最优解；`limit_reached` 只证明可以承受到 `max_score_n` 查询上限，并未确定更大极限，更不是证明无穷大。`unknown` 表示本项没算可靠；存在未知项时整批为 `partial`、`sampled_minimum_n=None`。基础平衡未通过则整个查询为 `unknown`。有限采样集合的最小值不能叫作“所有可能方向都保证稳定”的半径。

## 为什么能加速

1. 接触点、摩擦锥、平衡矩阵及有限容量只建一次；每个方向只改变 1 或 2 列外载荷系数。方向列用 NumPy 广播构建。
2. 用一次 HiGHS 求共同的零扰动可行基。各 LP 从同一个基开始，省去每项重复建模和 Phase I。
3. 在小批次内同时更新 simplex tableau，NumPy/CUDA 进行向量化 rank-one pivot。前 128 次采用最大检验数，之后使用确定性的 Bland 规则；达到迭代预算不伪造结果。原地更新减少大数组复制；NumPy 将每批 tableau 限制在约 2 MiB，避免把 GPU 的大批次直接套在 CPU 上。
4. CUDA 使用 PyTorch FP64 张量实际执行 pivot；不是只把数据放上 GPU 后继续调 CPU 求解器。每八轮检查一次是否整批完成，减少 CPU/GPU 同步；默认批量 256，控制显存。
5. 返回 CPU 后，用原始模型检查平衡残差、非负性、对偶可行性和目标 gap。不通过的 LP 默认单独回退 HiGHS，并记录 `fallback_problems`。公共基秩不足，例如无摩擦退化，也使用该回退。可显式关闭回退以获得 `unknown`，不能把超时当成零承载。

NumPy 仍是默认值，减少一次性小查询的 GPU 启动开销。多次查询应复用 `DirectionalStabilityAnalyzer`。CUDA 首次导入/初始化约 2 s，首次 kernel 还有额外开销；只看热运行不能代表首次用户等待时间。

批量 LP 的 GPU 解法有现有研究可参考：[Batched LP on a GPU](https://arxiv.org/abs/1802.08557)。本实现是针对共同接触模型的有界 simplex，并非直接移植该论文。参考求解器为 [SciPy HiGHS LP](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.linprog.html)。GPU 计时包含结果复制到 CPU 与复核，等待实际完成，符合 [PyTorch CUDA 计时说明](https://docs.pytorch.org/docs/stable/notes/cuda.html#asynchronous-execution)；不是仅计入异步提交 kernel 的时间。

## M2 页面距离含义

- **外部位置额外离开距离**：默认示例 100 mm，可设 10—300 mm，传入真实 `MotionConfig.outside_margin_m`。outside 终点超过其他零件在所选方向的投影边界，再加该余量，因此它不等于整步路程。库本身仍默认 20 mm。
- **应用距离 / 例子并重新规划**：后台工作线程重新计算和正向复核，主线程继续显示。新结果未通过就没有新可播放路径；输入改动本身不篡改已验证路径。
- **本步已移动距离**：以 mm 选择已验证路径上最近的一帧。包含绕行时是平移弧长，不是起终点直线距离。纯旋转请用“路径帧（含旋转）”控件。
- 接触面是最终装配位姿的参考，透明度仅影响显示。M2 的通过不能代替 M3 机器人可达性与抓持验证。

新增 `gantry`：四根下柱、平台、两根上柱、横梁和载荷，共九个活动零件。新增 `sleeve`：有真实正间隙的轴/套筒/垫片，加两根销，共五个活动零件；孔由原始非凸三角网格表示，不用凸包代替。两者均完成序列和独立正向复核。

DFS 现在遇到首个可行子节点就继续深入，失败后才恢复剩余兄弟候选；保留回溯与原确定性顺序。方块框架从本轮初测约 9.30 s 降到 1.36 s，9 次状态展开。没有删掉失败分支检查的责任，只延迟不需要的求值。

轴套的主要耗时在运动路径的三角形距离证据。已将 BVH 叶节点中至多约 8×8 对三角形的顶点/面、边/边及穿面检测批量化；仍不构造整个网格的 F×F 矩阵。极小预算下保留逐对裁剪和部分上下界。原先单次候选常在 20 s 预算后返回 unknown；优化后整套序列约 24.0 s，通过另一次独立正向回放。预算、真实净空和求解精度未放宽，复杂凹形路径仍明显慢于平面方块。

## 基准与回归

基准原始结果为 `examples/assembly/output/stability_sweep/benchmark_legacy_coupled.json`。同一输入、相同 16 棱锥、查询上限 100 N 等效、批量上限 256；NumPy 按矩阵大小进一步限制内存。使用 `--repeats 3 --large-repeats 1`：先测一次，300 方向取三次热运行中位数，6500 方向另测一次热运行。方向生成、几何接触提取与准备时间单独记录；查询时间包含传输、求解、CPU 证书复核与结构化结果。当前机器还运行 WRS 显示服务，墙钟数值会波动。

纳入版本管理的[完整基准快照](assets/stability-sweep-benchmark.json)保留每次计时、环境、误差和回退数量。

| 场景 | 每件方向数 | 独立 LP 总数 | HiGHS (s) | NumPy (s) | CUDA (s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| stack | 300 | 600 | 0.984 | 0.161 | 0.121 |
| stack | 6,500 | 13,000 | 21.867 | 3.298 | 2.013 |
| bridge | 300 | 1,200 | 3.426 | 2.603 | 0.515 |
| bridge | 6,500 | 26,000 | 76.013 | 58.734 | 9.981 |

全部 12 组均为 complete、CPU 回退 0 项；与逐项 HiGHS 的分数最大差异 4.93e-11 N 等效。CUDA 首次准备 2.116 s，后续同进程复用不再支付该初始化开销。

CPU 批次在 stack / bridge 分别为 131 / 29，GPU 为 256。桥梁的约束和反力变量更多（320 列，而堆叠为 128 列），pivot 轮数也增加；同样的方向数不能跨模型假定固定耗时。

同模型基准不能直接当成旧 SLSQP 多进程系统的端到端加速倍数。旧模型单位、锥分辨率、质量与异常处理均不同，尚未完成旧数据的同输入迁移。功能缺口与后续顺序见[新旧功能核对](old-new-feature-audit.zh.md)。

新增验证覆盖：解析滑动/抬升/倾覆、实际极限及略超极限复核、内部载荷传递、真实有限容量、无摩擦秩退化、迭代失败与 CPU 回退、重复性与单位、CUDA/HiGHS 同输入对照；三角形批量算法与独立标量参考在随机、退化、共面、穿面及不同尺度下逐项比较；DFS 真实回溯、九件框架、距离参数变化后的独立回放。

最终回归：装配套件 151 项通过；批量求解后续参数调整后，其 9 项定向测试再次通过；材质与回放测试 2 项通过。页面已实测 100→150 mm 重新规划、75 mm 帧定位、gantry/floating 切换、最终九件装配与透明度、部件/方向切换及最弱方向按钮。两套服务 stderr 无异常。
