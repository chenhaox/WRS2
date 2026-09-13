# 抓取数量、可装配性评分与论文式正向 DFS

> 入口整理说明（2026-09-13）：本文的旧脚本命令、输出路径和计时保留为历史记录。当前请按 [WRS 例子使用说明](../../examples/assembly/README.md) 运行，参数在文件顶部修改；benchmark 已移出 examples。

2026-09-12；在 `codex/assembly-planner` 上实现。统一解释器为 `D:\code\venv312\.venv\Scripts\python.exe`。

此次增加的是评分与搜索组合层。复用现有 WRS 抓取生成、SOCP 方向、GPU 多方向稳定性、M2 路径与 M3 回放，不重复建立另一套机器人或接触模型。

## 论文与旧实现的对应

核对了用户提供的 Chen / Wan / Koyama / Harada《Planning to Build Block Structures With Unstable Intermediate States Using Two Manipulators》最终投稿稿，重点为第 5 页式 (2)、Algorithm 1，第 7–8 页 graspability、assemblability 和辅助支撑策略。正式文献：[TASE，DOI 10.1109/TASE.2021.3136006](https://doi.org/10.1109/TASE.2021.3136006)。也核对了此前的 [Wan 等 2016 年预印本](https://arxiv.org/abs/1609.03108v1)第 5 页 Fig. 6 和第 6 页分类。

旧代码对应 `asp/sysplanner.py::searchSequence`、`graspabilityAnalyzer`，`asp/utils.py::AssemblabilityAnalyzer` 和 `get_score`；参考副本 commit 为 `9b886441d75f7ea79eafa4cdd5dc90d3c14dbac3`。没有执行旧 pickle。

| 项目 | 此次实现 | 与原文/旧代码的差别 |
| --- | --- | --- |
| G | 固定候选库中，力闭合且到位时夹爪无碰撞的抓取数量 | 使用现有 WRS 数据和实际指腹区域；不直接把两个 antipodal 点当成完整 6D 力闭合 |
| A | 独立函数按已求解方向集合形状赋分 | 保留 SOCP 的合法方向；不恢复旧法线求均值、随机 Qhull joggle 的做法 |
| S | 不加辅助支撑时，有限 6D 扰动集的最小归一化径向承载系数 | 使用此前修正的无量纲指标，不恢复原先直接混合力/力矩或双幅值改变射线的指标 |
| 辅助支撑 | S=0 时，枚举显式有限支撑候选并验证所有零件平衡、资源交接 | 不省略被持件的平衡方程；候选尚未自动从辅助夹爪生成 |
| 综合分数 | 论文式 (2) 的分段函数，默认 λ=100 | 默认 A 数值表采用 ASP_OLD 的有限表，并明确来源 |
| 搜索 | 正向逐件添加、DFS 回溯、保存最佳完整序列 | 用可靠的分支上界替代不总成立的“当前分数必然递减”剪枝 |
| 执行 | 返回标准 `SequenceResult`，可直接交给 `validate_execution` | 不自动将 M3 失败反馈到 DFS；M2 的最优不代表机器人可执行性最优 |

## WRS 已有的抓取模块

- `wrs/grasp/grasp.py`：`Grasp`，包括物体局部抓取/预抓取位姿、TCP、闭合/张开关节配置和来源。
- `wrs/grasp/antipodal.py`：对向抓取生成；`polypodal.py`、`monocontact.py` 提供其它接触形式的生成工具。
- `wrs/grasp/reasoner.py`：结合机器人 IK、碰撞与约束筛选抓取。
- `wrs/manipulation/arm.py`、`pick_place.py`：机器人抓取筛选与 pick-and-place。
- 当前 `wrs/assembly/execution.py::generate_execution_grasps` 已包装这些能力：盒体采用可重复的侧向夹持，其它网格调用 WRS antipodal；对真实夹爪指腹间距进行校准。

新增 `GraspabilityAnalyzer` 复用这个入口，也接受调用者通过 `grasps={part_id: [Grasp, ...]}` 传入已有 WRS 抓取。每个实例复制夹爪，抓取记录和计算缓存绑定几何、配置与状态，不改变调用者的机器人/夹爪姿态。

G 的定义为

\[
G_i=\#\{g\in\mathcal G_i:\text{force closure}(g),\;\text{gripper at goal is collision-free}\}.
\]

这是**固定候选库内的数量**。不能把不同候选预算、生成参数或夹爪的 G 直接当成同一尺度比较。默认最多 12 个校准候选；盒体通常只有 2 或 4 个侧向抓取。更大的候选库应固定后再比较装配顺序。它也不是“所有连续抓取的总数”。

完整 6D 力闭合采用 wrench 正生成判据。实际指腹接触区域提供点和朝向物体的法线；离散摩擦锥给出列向量

\[
w_j=\begin{bmatrix}f_j\\(p_j-c)\times f_j/L\end{bmatrix},\qquad W=[w_1\cdots w_m].
\]

先检查 `rank(W)=6`，再解 `max t`，满足 `Wλ=0, Σλ=1, λ_j≥t≥0`。获得可靠的 `t>0` 才通过。L 为接触点尺度，用于数值条件改善；正生成与坐标原点/正尺度变换无关。这个判据参考 [Modern Robotics 的 Force Closure 与 LP 推导](https://modernrobotics.northwestern.edu/nu-gm-book-resource/12-2-3-force-closure/)。

两个理想硬点接触通常缺少绕两点连线的扭矩能力；测试明确拒绝这种 rank=5 情况。实际指腹多点模型可以产生力偶，但力闭合本身不保证指定重量可被有限指力搬运，后者仍由 M3 检查。

夹爪自身与目标仅允许有证据的指腹接触；其它夹爪部位不能穿入目标。夹爪与已装配零件按原始网格检查，AABB 先排除远离的对象。缺少可认证曲面指腹斑块、网格内外不明或预算不足时返回 `partial`、`count=None`，同时给出 `accepted_ids` 和 `known_count`；搜索记录这些未决项，不把未知当成零抓取或声称已证明最优。

此阶段不检查 IK、预抓取/张开动作、夹爪整个插入路径，也不将尚未实例化的辅助夹爪作为障碍。这些是 M3 的义务，与论文“先筛抓取、后算机器人运动”的层次一致。

## A 与方向求解解耦

```python
from wrs.assembly import assembly_directions, score_assemblability

direction = assembly_directions(graph, (part_id,))
quality = score_assemblability(direction)  # 不再解一次 SOCP
```

矩阵级也可用 `solve_directions(normals)` 后再评分。分类依据可行锥维数、法线秩、子空间是否有剩余边界、轴两端的可行性。新增的求解诊断只保存这些小量；G/A 评分不依赖可视化点数。

| Wan Fig.6 类别 | 可行方向 | 默认 `asp_old` | `wan2016` |
| --- | --- | ---: | ---: |
| a / c / f | 球面上的面积区域 | 10 | ∞ |
| b | 完整大圆 | 9 | 10 |
| d / g | 半圆 / 圆弧 | 3 | 3 |
| e | 两个相反轴向点 | 2 | 2 |
| h | 一个轴向点 | 1 | 1 |
| i | 空集，平移锁死 | 0 | 0 |
| 扩展 free | 无约束，整个球面 | 10 | ∞ |

用户此次论文 Fig.6 的例子给出面积 10、圆弧 3；其全文没有把此前预印本九类完整数值表逐项重列。**整圆 9 来源于 ASP_OLD；不能说成 2016 预印本原值。** 所以两张表显式分开。无约束 free 是工程扩展。

`score_assemblability(..., profile='wan2016')` 用字符串 `"infinity"` 保存无限值，以兼容严格 JSON。当前序列搜索使用有限 `asp_old` 表；不能把字符串无穷大直接传给 `StepQuality`。如需无限值参与搜索，应另行定义扩展实数比较/零乘无穷的规则，不能悄悄设一个大数。

A 和 SOCP 的角度容差是两个指标。面积区域可以很窄但仍是 A=10；需要衡量装配方向误差时，读取 `angular_margin_rad` / `intrinsic_angular_margin_rad`。本次遵循论文分类评分，没有偷偷改为球面采样命中率。

A 是局部纯平移集合分数。DFS 用 SOCP 给出的一个代表方向完成有限拆出路径，再反向验证插入。某条代表路径未通过不证明所有平移/旋转都不可能；这类失败保留为未决项，限制最优性声明。图中的方向是拆出方向，插入方向取反。

同时修复了方向求解中的一个浮点秩问题：旋转法线的成比例重复副本归一化后可能产生极小的额外精确秩。允许 machine-roundoff 模型时以 SVD 有效秩处理并记录证据；关闭该选项时返回数值不确定，不把平面错误收缩成轴线。测试覆盖九类、旋转、倍数重复和两种显示方式。

## 论文分数与 DFS

对一条完整序列，令 `s=min S_i, g=min G_i, a=min A_i`，`n` 为 S=0 的步骤数：

\[
\xi=\begin{cases}sga,& n=0,\\ga/(\lambda n),&n>0.\end{cases}
\]

当前 S 是此前稳定性模块输出的 `sampled_minimum_load_factor`：固定 F_ref、L、300 个 S⁵ 方向，在每个活动零件上施加扰动，保留所有零件的力/矩平衡，取所有测试的最小值。默认 CUDA，无可用 CUDA 时使用已验证的 HiGHS 回退。分数受方向集和求解上限影响，既不是严格全空间半径，也不能与旧混合单位 S 数字直接比较。

`S <= zero_stability_tol`（默认 1e-8）按零处理；名义失稳也按零处理，但必须找到能通过平衡复核的有限辅助支撑，否则不扩展该分支。unknown 不能替换成零并靠辅助支撑掩盖。n 数的是需要辅助的中间状态数量，不是手数或辅助抓取次数。默认最后一步必须无需辅助支撑。

λ=100 体现惩罚强度，但有限 λ 不等价于“无条件先最少辅助次数，再比较其它分数”的字典序目标。例如 g、a 不同仍可能改变支持次数的取舍。

### 为什么修改原剪枝条件

如果当前前缀 `s=0.0001,g=1,a=10`，分数是 0.001；后续出现首次需要支撑的状态，分数却可能变为 `10/100=0.1`。因此“当前分数低于已找到完整序列就剪掉”会漏解。

新代码在尚未需要支撑且还有步骤时使用

\[
U=ga\max(s,1/\lambda),
\]

在已需要支撑、或已经到叶子时使用当前分数。仅当 U 不高于已有完整解时剪枝。G/A 的前缀最小值只能下降；已经有支撑时 n 只能增加，因此这是一个可用上界。

`quality_depth_first` 是与物理模型解耦的搜索内核，`expand(context, part_id)` 返回带 `StepQuality` 的状态转移。`plan_quality_sequence` 把真实抓取、方向、稳定性与 M2 连接起来。没有用“访问过同一个零件集合就跳过”的全局 visited，因为不同前缀的最小分数/支撑次数不同。只缓存与前缀分数无关的几何/力学结果。

只有完整枚举声明的分支、没有未决评估、且独立正向回放通过时，才报告 `optimal_within_declared_catalogue_and_model`。时间/展开预算耗尽时可以保留已有完整方案，但最优性为 `not_proven`。时间限制在评估调用之间检查，单次几何/优化查询受各自预算约束，不是可中断的硬实时截止。

## 使用和可视化

```python
from wrs import or_2fg7
from wrs.assembly import GraspabilityAnalyzer, plan_quality_sequence

grasps = GraspabilityAnalyzer(assembly, or_2fg7.OR2FG7())
result = plan_quality_sequence(assembly, grasps, supports=finite_supports)
if result.plan is not None:
    print(result.score, result.qualities)
    # 标准 M2 结果，可继续 validate_execution(result.plan, workcell)
```

在仓库目录执行：

```powershell
$assemblyPython = 'D:\code\venv312\.venv\Scripts\python.exe'
& $assemblyPython examples/assembly/quality_sequence_demo.py --case bridge --headless
& $assemblyPython examples/assembly/quality_sequence_demo.py --case bridge --no-prune --headless
& $assemblyPython examples/assembly/quality_sequence_demo.py --case counterweight --port 8898
```

例子包括小盒堆叠、双桥墩/横梁/载荷，以及悬臂梁/两层配重。WRS 窗口可选择步骤、查看 M2 路径帧、循环显示真实 OR2FG7 的合格抓取，显示 S/G/A、辅助支撑 ID 与序列分数。夹爪展示沿零件路径的刚性跟随姿态，不代表这些夹爪路径已通过 M3。接触层是最终装配位置的参考。

JSON 输出到 `examples/assembly/output/quality/`，包含每个前缀 S/G/A、incumbent 更新、上界剪枝、未决原因和实际抓取证据。`--no-prune` 单独保存 `_exhaustive.json`。默认后端 CUDA；`--backend highs` 可做独立 CPU 对照。

本机 Python 3.12 实测一次完整调用：桥梁约 **6.20 s**，配重约 **5.20 s**，均使用 300 个 6D 方向、CUDA 默认配置，包含首次抓取准备、搜索及候选正向复核，不含模块导入、JSON 保存及界面启动。不是纯 LP 耗时，也不是多次统计中位数。

- 桥梁：left_pier → right_pier → beam → payload；G=2,2,2,4；A 都为 10；综合分数约 0.346836。
- 配重：beam → weight_lower → weight_upper；S=0,0.260127,0.086709；G=2,4,4；综合分数 0.2。梁先由 20 N 有限支撑托住；装第一块配重后撤掉支撑。把容量改为 0.01 N，测试必须拒绝该序列。

加速点：夹爪局部校准/指腹/力闭合每件只做一次；碰撞按抓取、目标位姿、障碍位姿缓存；A 复用方向求解结果；同一装配状态的 GPU sweep 只做一次；DFS 转移与接触图复用；支撑组合惰性产生并受预算限制，指定组合时直接检查。法线退化处理避免生成与约束数平方增长的完整 SVD 左矩阵。这里的小型力闭合 LP 留在 CPU，避免 GPU 启动/传输成本；300 方向的承载优化继续批量 GPU 求解。

## 检查与尚缺功能

新增回归包括九类方向、两硬点不能 6D 力闭合、真实夹爪的障碍筛选与调用者状态保持、分段目标的错误剪枝反例、独立排列穷举对照、真实桥梁有/无剪枝对照、有限支撑正反例，以及新质量序列直接接入真实 RS007L/OR2FG7 的 M3 模拟执行。完整原有 158 项与首批 6 项新检查一起运行：164 项通过（96.11 s）；补充支撑、实际穷举和 M3 后，新增文件的全部 8 项通过（18.97 s）。本次累计覆盖原有 158 + 新增 8 项，不把同一测试的重复执行重复计数。

稳定性已有可用且经过独立方程检查的基线，但“差不多了”只适用于声明的刚体、准静态、离散摩擦锥与有限扰动集模型。还没有弹性接触、真实压力分布、动力学或全六维连续鲁棒性保证。

最后对 `test_directions`、`test_sequence`、`test_quality` 的联合复核：30 项通过（24.16 s），覆盖最终的 SVD 内存和指定支撑组合优化。WRS 页面实际检查了步骤切换、夹爪切换、到位按钮与滑条同步、临时支撑在到位前后的切换；服务错误日志为空。

下一步主要缺项：

1. 从真实辅助夹爪自动生成支撑/抓取候选，联合验证有限承载和与主臂的交接干涉。
2. M3 失败反馈到质量 DFS，尝试其它抓取、装配方向和序列，形成联合回溯。
3. 曲面指腹、软指接触与更通用夹爪的可靠力闭合模型；本次不能认证时明确保留 unknown。
4. 旧 Soma/Burr/Leonardo 等完整数据集的同参数迁移与基准；中途换抓、整体重定向、CAD/B-Rep 和连续转动装配。

代码入口：[独立评分](../../wrs/assembly/quality.py)、[WRS 抓取计数](../../wrs/assembly/graspability.py)、[质量 DFS](../../wrs/assembly/quality_search.py)、[例子](../../examples/assembly/quality_sequence.py)。
