# 静力、扰动、辅助支撑：当前实现与旧版对照

2026-09-11。本轮核对代码并补充阶段基准，不修改静力求解算法。

**历史阶段记录。** 后续已实现多方向极限和 CUDA 批量后端，见[新实现及计时](stability-sweep-and-sequence-ui.zh.md)；M2/M3 当前状态见[完整功能核对](old-new-feature-audit.zh.md)。下面保留当时版本的描述和数据。

后续的可视化、材质同步修复、复杂场景与独立 SOCP 验证见 [稳定性可视化与计算核查](stability-validation.zh.md)。下文旧/新算法比较与原始基准保留。

旧版依据 `D:/code/ch/asp/asp_old/assembly_planner` 的 Git 对象，HEAD 为 `9b886441d75f7ea79eafa4cdd5dc90d3c14dbac3`。该目录目前只有 `.git`，使用 `git show HEAD:asp/...` 读取，无须改变检出状态。`D:/code/ch/asp/assembly_planner` 是相同 commit 的已检出参考副本，下面的旧源码链接指向该副本。比较的是 `asp/utils.py` 默认 `GWS=0` 路径；其他实验分支不能混为一种实现。

## 当前做了什么

入口为 [check_equilibrium](../../wrs/assembly/stability.py)。它寻找“是否存在一组合法接触力，让每个自由零件的合力和合力矩都为零”。各体独立满足：

```text
Σ contact_force + mass * gravity + external_force = 0
Σ (contact_point - COM) × contact_force + external_torque = 0
```

相邻零件共用同一组接触力变量，在同一作用点施加等大反向力。每个作用点将力写为 16 个摩擦锥边缘方向的非负组合：

```text
g_j = n + μ * (cos(θ_j) * tangent_1 + sin(θ_j) * tangent_2)
f = Σ λ_j * g_j, λ_j >= 0
```

`n` 为单位法线，`θ_j` 在圆周均匀分布。这是内接多面体锥，保证得到的力不会超出理想 Coulomb 摩擦锥，但可能保守地排除圆锥内的一小部分方向。默认 16 边，最坏横向径向比例 `cos(pi/16)≈0.981`；旧版 4 边约为 `0.707`。

这些约束均为线性，使用稀疏 `scipy.optimize.linprog(method='highs')`。目标函数为最小化所有非负生成系数之和，以挑出一组平衡力；这不是最大扰动评分。只有可信 active 区域生成力点；near/unknown 不承载。平面用实际单元的去重顶点，曲面用已有点与法线场。孔内不补点，质心、质量、摩擦未知会有 unknown 诊断。力矩按明确长度缩放求解，再以 N、N·m 检查残差。

### 扰动检查

`StabilityConfig.disturbances` 默认空，结果为 `not_tested`，不会自动生成方向。一个 `LoadCase` 指定同时作用的额外力/力矩，多组 `LoadCase` 分别求解：

```python
config = StabilityConfig(disturbances=(
    LoadCase('push_x', (ExternalWrench('upper', force_world_n=(1,0,0)),)),
    LoadCase('push_y', (ExternalWrench('upper', force_world_n=(0,1,0)),)),
))
result = check_equilibrium(assembly, state, graph, config=config)
```

每次把扰动加到重力和基础外力上，接触几何保持不变。每个载荷允许重新分配接触力；不要求所有载荷共用同一套力。结果仅说明名义载荷和已声明扰动能否平衡，不是动态仿真，也不自动计算所有方向的最大抗扰量。当前 `stability_demo.py` 默认只做名义平衡。

### 有限辅助支撑

调用者提供 `SupportCandidate` 的零件、位置、推力方向、最大法向力和摩擦。它提供一个有限的单侧力锥，不提供任意方向无限 wrench，也不提供点上的自由力矩。

`find_support_requirements` 按 0 个、1 个、2 个……枚举候选子集。默认最多选 2 个、最多测 64 个子集，每个子集都验证名义及配置的扰动载荷。输出仅是满足声明力模型的支撑需求，没有自动生成支撑位置或验证机器人抓取/可达性。

例子总质量 3 kg、总重 29.43 N：悬空时失败；单个 20 N 向上支撑仍失败；两侧各一个 20 N 支撑可以平衡。需检验空集、左、右、左右，共 4 个子集。最小性仅限已声明候选集和力模型，预算/数值 unknown 不支持全局结论。

## 与旧版的实际区别

旧调用链为 `asp/utils.py::StabilityAnalyzer` → `asp/stabilitylib/stability.py::is_stable/getsolution`。旧版已经有逐体力/力矩平衡、共享内力变量和摩擦锥；这些不是新版才增加的思想。

| 项目 | 旧版默认路径 | 当前版本 |
| --- | --- | --- |
| 静力求解 | 线性力模型包装成回调，交给 SLSQP；未提供解析 Jacobian | 直接组装稀疏线性模型，HiGHS 求解并复核物理残差 |
| 摩擦 | 4 条射线；主调用全局 `U_COEFF=0.4`，表中逐对摩擦仍是 TODO | 默认 16 条射线，可配置；接触对取双方声明值较小者并记录假设 |
| 重力/质心 | 每个零件使用相同 `G*[0,0,-1,0,0,0]`，COM 来自网格 `center_mass` | 逐零件质量、局部 COM 和世界重力；未提供时 unknown |
| 数值 | grasp matrix 舍入至 3 位小数 | 不作该舍入；力矩尺度与残差单位分开 |
| 扰动 | 对预生成方向逐体搜索最大扰动力/力矩，再汇总为分数 | 检验明确给定的载荷集合；目前没有复刻最大扰动评分 |
| 辅助夹持 | `skipobjname` 跳过被夹零件平衡式，随后筛选辅助抓取的碰撞 | 显式有限力支撑点和受限组合搜索；机器人执行尚未实现 |
| 缓存/加速 | 接触全局表、按零件名字集合缓存稳定性；扰动计算用 Process/Manager，代码 `gpu=False` | 接触输入外置、稀疏矩阵、部分向量化、单次调用中复用载荷矩阵；尚无结果或 LP basis 缓存 |
| 返回含义 | 主要是布尔/None/评分；部分调用有 999 哨兵 | 名义、各扰动、力分配、残差、数值 unknown 和假设分开记录 |

跳过某个零件的六条平衡方程，相当于允许外部夹持补足其缺失合力和力矩，未显式约束这个补偿 wrench 的能力。它不等同于已验证的有限机器人夹持。新版普通 fixed 件仍是理想固定边界；只有辅助候选明确带有限能力。默认接触法向力上限也仍是无限，结果会记录这个假设。

旧版扰动评分值得迁移：其核心是固定扰动方向，对目标零件最大化 `k_force + k_torque`，并以模型最大半径关联二者；最终取各零件、各采样方向中的较小评分。它比新版“给定 1 N 是否扛得住”的查询多了求临界值这一步。迁移时应重新定义力和力矩的归一化尺度，不直接相加不同单位，也应保留数值失败状态。固定方向之后仍可整理为 LP，不必继续使用 SLSQP。

`asp_exp/test.py` 等另有 Bullet 重力仿真实验，它们不属于上述默认静力评分求解链。不能把当前静力载荷检查说成已经替代了这些动态实验。

## 这次重新测得的时间

`D:/code/venv312/.venv/Scripts/python.exe`；每例预热后 51 次独立完整 API 调用，不含接触提取、SDF、导入或 viewer；无结果缓存。堆叠有 2 个自由体、8 个力点、默认 128 个生成系数、12 条平衡等式。

| 查询 | 中位数 | P95 | LP 次数 |
| --- | ---: | ---: | ---: |
| 堆叠名义平衡，16 边 | 3.58 ms | 3.98 ms | 1 |
| 名义 + 6 个 1 N 坐标轴扰动 | 19.38 ms | 23.24 ms | 7 |
| 名义 + 24 个 1 N 水平扰动 | 54.56 ms | 65.67 ms | 25 |
| 两个箱体悬空，无辅助支撑 | 2.35 ms | 3.17 ms | 1 |
| 从两个 20 N 候选中选支撑 | 12.21 ms | 13.74 ms | 4 |

摩擦锥分辨率对照：4 边（32 变量）中位 3.59 ms、32 边（256 变量）4.77 ms。该小案例 4 边未比 16 边更快，表明固定建模/求解开销和运行波动不可忽略；不能为了假定提速直接降低摩擦分辨率。

没有在同一输入、同一目标和同样缓存条件下完整运行旧版，**没有新旧速度倍率结论**。特别是旧版扫描最大扰动，新版默认只求名义平衡，工作量不同。

## 现在如何加速，还剩什么开销

已经实现：

1. 采用线性模型专用求解器，直接给矩阵，不让通用优化器通过 Python 回调估计导数。
2. 接触生成方向内部用数组 sin/cos、力矩用批量 `np.cross`、行列索引用广播，构造 COO 后转换 CSC；只放有连接关系的零件块。
3. 同一次 `check_equilibrium` 中，各扰动复用接触力生成矩阵、平衡矩阵和能力矩阵，只改变载荷；重复作用点先去重。
4. 输入已有接触图，求静力时不会重复 SDF 接触提取。

还不能称为充分向量化：每个力点仍有 Python 循环；相同法线/摩擦会重复构造 16 条生成方向；每个 LP 都重新通过 SciPy 建立 HiGHS 模型；每个支撑子集重做状态校验、摘要和建模；暂未实现 GPU、JIT、跨查询缓存和 warm start。

独立插桩运行的平均阶段时间（不是上表中位数的精确拆分）：名义平衡中 SciPy/HiGHS 约 1.76 ms，摩擦生成约 0.38 ms，点处理约 0.20 ms，摘要约 0.33 ms，状态校验约 0.07 ms，其他建模/残差/输出/插桩约 0.97 ms。24 个扰动中 SciPy/HiGHS 累计约 46.89 ms，占插桩总时间约 85%。

因此后续优先级是：

1. **复用 LP 模型与 basis。** 建立可复用的静力模型，载荷变化时只改 RHS。原生 highspy 提供 `changeRowsBounds`、`getBasis/setBasis`；需要接入后实测，当前未实现。[HiGHS 官方接口](https://ergo-code.github.io/HiGHS/stable/interfaces/python/example-py/)
2. **批量建模。** 按法线、摩擦分组计算生成方向；对全部作用点批量计算力矩，并缓存不可变接触模型的摘要。仍需严格绑定几何/状态与物理配置，不能只按零件名缓存。
3. **支撑搜索复用基础模型。** 支撑列预先加入，通过列/能力上下界启闭候选；候选变多后再考虑剪枝或混合整数优化，保留求解预算与证明范围。
4. **迁移旧版临界扰动评分。** 在统一力矩尺度后，对每个指定扰动方向加入一个幅值变量，用 LP 最大化它；保留明确载荷集合检查作为另一种查询。

求解器依据：[SciPy HiGHS](https://docs.scipy.org/doc/scipy/reference/optimize.linprog-highs.html)、[SLSQP 数值 Jacobian](https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html)。这是模型与接口层面的选择理由，不是未经测试的倍速保证。

复现：[benchmark_stability.py](../../examples/assembly/benchmark_stability.py)。原始逐次与阶段数据在 `examples/assembly/output/stability/benchmark.json`：

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/benchmark_stability.py --repeat 51
```

旧版源码参考：[stability.py](../../../assembly_planner/asp/stabilitylib/stability.py)、[utils.py](../../../assembly_planner/asp/utils.py)、[sysplanner.py](../../../assembly_planner/asp/sysplanner.py)。
