# M1 审查与 M2.04 交接

2026-09-11；基于 `b4c6ced` 审查。开发分支 `codex/assembly-planner`。
实现与验收提交：`e73e6f4`。
全部计算使用 `D:\code\venv312\.venv\Scripts\python.exe`，没有新增或升级依赖。

## 结论与修复

现有实现不是“已经证明任何网格都正确”的几何内核。本轮在现有名义网格、公差和预算契约内检查正确性，修复实际问题，再落地 M2 的第一项 04。05 静力平衡、06 有限路径、07 序列搜索尚未实现。

1. **SDF 缓存失效错误**：旧 key 只有模型几何。先允许 unsigned，再把 `sdf_config.open_surface` 改为 `error`，仍可能得到旧 unsigned 字段。现在 key 包含几何预处理配置、符号策略和 nsamples；独立网格验证器同步几何配置。查询预算等不影响字段本身的设置不触发无谓重建。
2. **实例身份遗漏**：碰撞查询摘要没有 A/B 名称。共享几何、相同位姿的不同实例会产生相同摘要。现在包含实例名称，并更新碰撞实现标识。
3. **清理后的孤立顶点**：退化面清理后未压缩顶点数组，远处的无用顶点会放大字段归一化尺度及误差保护量。现在只保留存活三角形引用的顶点，同时保留原面映射和清理诊断。
4. **边界拼接速度**：`cell_regions` 原来逐顶点建立索引、逐边查询 KD-tree 并投影。现在批量索引、每批 256 条边查询稀疏邻域，并向量化线段投影和 T 接点筛选。保持原先首出现顶点、孔洞、分组及公差语义，不做轮廓平滑。
5. **距离搜索冗余**：BVH 已有距离见证后，下界等于当前最优上界的候选不可能改善结果。由 `>` 改为 `>=` 剪枝，避免共面/等距面的重复检查。没有放宽容差。一个轴对齐箱体案例现在一个三角形测试就能完成；预算测试另用倾斜案例验证耗尽语义。

## 性能记录

使用同一解释器、同一输入及配置，几何缓存预热后完整分析三次取中位数。包含独立网格检查和区域提取，不含加载、HTML 写入、渲染。profiling 另跑，不计入中位数。

| STL / 终止半径 | 优化前 | 优化后 | 加速 |
| --- | ---: | ---: | ---: |
| 法兰 / 0.1 mm | 2.1999 s | 1.2721 s | 1.73× |
| 圆柱 / 0.1 mm | 1.4753 s | 0.8403 s | 1.76× |

`output/audit-before` 与 `output/audit-after` 保存运行数据和 profile。两个案例的完整 `patches` JSON 精确相等，包括所有单元、边界、面积和法线，不仅面积总和相等。独立距离检查三角形次数会减少，因此统计项不要求相同。

优化前 profile 的主要热点是 `cell_regions` 和 `pair_distance`。优化后区域边界、SDF 单元上界/裁剪、不可变数组复制仍有成本；当前不宣称是最快实现。大型、扭曲或退化网格的稀疏邻域可能很密，批量计算不能消除几何复杂度。进一步优化应重新 profile，而非全量广播成 F×F 矩阵。

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/sdf_diagnostics.py --resolution-mm .1 --repeat 3 --profile --out-dir examples/assembly/output/audit-after
```

## 验证范围

- 73 项 assembly 单元/回归测试，包括原有几何、SDF、容差策略、接触维度测试以及新增缓存、孤立顶点、T 接点、孔洞和 M2 用例。
- 49 个碰撞案例重跑：16 separated、14 touching、18 penetrating、1 unknown；全部判定及 touch/penetration 区域面积与原结果一致。
- 窄间隙轴孔仍为 unknown：保留预算/覆盖不足，不把 unknown 当作 free。
- M2 六个示例和碰撞 49 个案例的页面切换无 JavaScript 错误；候选切换与 390 px 布局已检查。

继续保留的限制：Open3D float32 距离保护量是工程估计，不是形式化误差证明；开放或自交网格不能自动获得可靠体内外；独立支持平面提供的 touching 是名义 mesh 证据，不是 SDF 提取出了真实 active 面。曲面 SDF near 面積仍不能直接进入承载求解。

## M2.04 公共接口

```python
from wrs.assembly import (
    analyze_contacts, build_contact_graph,
    ConstraintConfig, contact_constraints, candidate_motions, rebase_twist,
)

state = assembly.initial_state()
analysis = analyze_contacts(assembly, state, backend='mesh')  # 也支持 sdf/backend 对象
graph = build_contact_graph(assembly, state, analysis)
graph.assert_matches(assembly, state)

config = ConstraintConfig(mode='twist', characteristic_length_m=.1)
constraints = contact_constraints(graph, ['part'], config=config)
motions = candidate_motions(graph, ['part'], config=config)
```

实现文件：`contact/graph.py` 与 `constraints.py`。

- 节点是实例，包含当前位姿、几何 ID 和 fixed；边保留全部 patches、mating 和 pair diagnostics，即使某对只有 separated/unknown。
- `analyze_contacts` 新增不可变 `input_binding`，记录当前存在实例的几何、位姿和 world_revision。未绑定的旧 JSON 或仅调用 `analyze_pair` 得到的报告不能直接构图；需要重新调用 `analyze_contacts`。这样避免把新姿态贴到旧世界坐标见证上。
- 构图会拒绝不匹配的状态。图摘要包含分析配置/后端摘要、状态、固定支撑与 mating。`assert_matches` 检查位姿、几何、修订、fixed 和 mating。当前没有跨调用的图/候选结果缓存；可用这些摘要建立外部缓存。后续静力结果缓存还必须加入质量、COM、摩擦、重力、扰动与支撑能力，不能只使用 graph key。
- 约束只从 active、analytic/bounded 证据生成。near、mating、容差策略接受的近接面不会混入 active；显式理想化需要再次选择 `allow_idealized_contact=True`。
- 若 A 静止、B 运动，约束为 `n_A · [v + ω × (p-reference)] >= 0`；运动 A 时取负号。整个 moving group 做刚体运动，组内接触不形成相对约束；不能移动 fixed 零件。
- 平面/均匀法线区域使用全部单元顶点，包含孔洞周围的实际边界，避免只有质心时漏掉倾覆方向。变化法线逐点保留，覆盖仅为采样时记录 `sampled_contact_coverage`，不平均法线。
- 平移和六维候选都排除零向量。用 SciPy HiGHS 在有界立方体中分别优化正负坐标方向，以发现随机采样不易命中的低维可行锥；增加切向、法线、法线叉积和确定性随机候选。法线种子、叉积和随机数均有上限；约束检查仍保留所有约束行，分批矩阵乘法验证残差。
- 六维归一化 `u=[v,Lω]`，`||u||=1`；`twist_world` 的角分量是每米归一化路径参数对应的弧度。参考点改变用 `rebase_twist` 保持同一速度场。
- `near_activation_parameter_m` 是近接样本按当前方向线性预测的闭合参数，不是安全步长或有限碰撞证明。
- `local_candidates` 仅意味着通过已报告 active 约束；仍须检查 `constraints.issues`。`locally_blocked` 仅针对当前局部运动模式，不能称为无装配路径。`geometry_validated`、`execution_validated` 固定为 False；数值失败/覆盖缺失保留诊断。

## 示例与后续交接

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/m2_contact_graph_demo.py
& 'D:\code\venv312\.venv\Scripts\python.exe' -m unittest discover -s tests/assembly -q
```

生成 `examples/assembly/output/m2/contacts.html`、六个 JSON 和 `summary.json`。页面可切换候选方向；蓝色是平移、紫色是角速度轴。快照如下，耗时仅为候选生成，不含接触分析：

| 案例 | 原始约束行 | 候选数 | 预热中位数 |
| --- | ---: | ---: | ---: |
| 单面 | 5 | 49 | 7.36 ms |
| 对向面通道 | 10 | 7 | 6.08 ms |
| 角落六维 | 15 | 11 | 11.89 ms |
| 六面包围，纯平移 | 30 | 0 | 6.85 ms |
| 0.2 mm 下方间隙 | 0 | 70 | 1.48 ms |
| SDF 正间隙轴孔 | 0 | 70 | 2.31 ms |

轴孔的 70 个候选并不意味着可以沿任意方向取出轴：当前没有 active 约束，正间隙运动何时碰壁需由 06 检查。此例还保留了 SDF unknown/估计诊断。

下一项为 [05 静力平衡](tasks/05-stability.zh.md)。力求解必须使用实际 active 单元上的合法点、共享接触力变量和逐零件平衡，不要将运动矩阵去重后的行直接当成力变量，也不要让 near/unknown 承载。随后按 [06 有限路径](tasks/06-part-motion.zh.md)、[07 序列](tasks/07-sequence.zh.md) 实施。
