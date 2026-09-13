# 装配方向、Gaussian sphere 与 assemblability

2026-09-11。代码核对基线：WRS2 `e8cfd7e`；旧参考仓库 `assembly_planner`。这是研究和后续设计建议，不表示下列扩展已经实现。

[交互方向球示意](assets/direction-space.html) · [当前 M2.04 实现交接](audit-and-m2-04.zh.md)

## 当前实现算的是什么

入口为 `wrs/assembly/motion/constraints.py` 的 `contact_constraints` 和 `candidate_motions`。结果是**已报告接触约束下的局部非零运动候选**，并非完整插入路径，也没有实现论文的 assemblability 分数。

设 A 固定，B 移动，接触法线 n_i 为 A 的外法线，指向 B 所在侧。纯平移 d 在接触处满足一阶不穿透条件：

\[
n_i^T d\geq0,\qquad C=\{d\in\mathbb R^3:Nd\geq0\},\qquad D=C\cap S^2.
\]

严格正值表示该接触分离；零值表示一阶切向运动；负值表示进入障碍。这里零残差不是有限运动无碰撞的证明。曲面、后续接触以及远处障碍还需要路径检查。

Gaussian sphere 在这里指单位方向球，与高斯概率分布无关。一个法线确定一个允许的闭半球，多个约束对应半球求交。**在接触法线和建模假设一致时，当前矩阵不等式与这种球面表达描述相同的局部方向集合**；当前代码使用隐式不等式表示，没有显式计算球面多边形、圆弧或立体角。

| 理想接触情形（仅纯平移） | 约束 | 球面上的允许方向 |
| --- | --- | --- |
| 放在平面上 | d_z ≥ 0 | 含赤道的上半球 |
| 被两平行面夹住 | d_x ≥ 0 且 −d_x ≥ 0 | 大圆 |
| 无端面接触的零间隙轴孔 | d_x = d_y = 0 | +z、−z 两个点 |
| 同上，另有孔底接触 | d_x = d_y = 0，d_z ≥ 0 | +z 一个点 |
| 三个正交支承面 | d_x,d_y,d_z ≥ 0 | 八分之一球面 |
| 六向完全限制 | d_x=d_y=d_z=0 | 空集，因为零向量没有方向 |

零间隙轴孔的方向球面积为零，但有轴向平移方向。**面积为零不等于不可装配。** 有正间隙的轴孔则不同：未接触侧壁不会限制瞬时微小平移，但会限制后续移动距离。

当前候选生成过程：

1. 从移动组和静止组之间的合格 `active` 接触构建约束；保留来源、点位置、覆盖不足等诊断。`near` 记录间隙，不直接当作零间隙约束。理想化接触需显式允许。
2. 对约束行归一化、去除精确重复。在有界盒内向各坐标正负方向做线性规划，寻找非零解；纯平移为 6 次 LP，twist 为 12 次。
3. 添加零空间、切向、法线叉积边界种子和固定随机种子。LP 和零空间步骤用于避免只靠随机球面采样漏掉圆弧或孤立方向。
4. 用 NumPy 矩阵乘法批量检查所有约束残差；候选集合受预算限制，并非所有极射线的完整枚举。

页面里的候选顺序不是优劣排序，`minimum_residual` 是约束残差，不是已经校准的 assemblability。`geometry_validated=False`、`execution_validated=False`；`near_activation_parameter_m` 也只是线性估计，不是安全步长。

允许转动时使用：

\[
n_i^T\big(v+\omega\times(p_i-r)\big)\geq0,
\qquad A_i=[n_i^T,((p_i-r)\times n_i)^T].
\]

因此不能只保留法线而丢掉接触位置。程序用 u=[v,Lω] 统一尺度，单位方向属于六维空间中的 S⁵；三维 Gaussian sphere 只完整表达纯平移部分。L 和参考点要明确记录。

方向符号也必须区分：当前 d 是从装配终态出发的局部分离/滑动方向。如果已验证位置路径 x(s)=x_goal+s d 可拆出，则反向几何路径给出插入运动，其速度方向为 −d。动力学、抓取和支撑条件不能仅靠反向路径继承。

## 用户所附论文的具体定义

Weiwei Wan, Kensuke Harada, Kazuyuki Nagata, *Assembly Sequence Planning for Motion Planning*, [arXiv:1609.03108v1](https://arxiv.org/abs/1609.03108v1)（2016）。本节核对的是用户本地 v1 PDF 第 4—6 页、IV.D、Fig. 6–7，不能把后续版本的内容默认为 v1。

论文用 constraint sphere 解释方向空间，实际按**接触法线凸包与原点的相对关系**分九类。旧代码将原点显式加入凸包点集。Fig. 6 的结果如下：

| 图中情况 | 可行方向集合 | 论文 assemblability |
| --- | --- | ---: |
| a：单法线或同向法线 | 半球 | ∞ |
| b：两个相反法线 | 大圆 | 10 |
| c：共面，原点在凸包顶点 | 有面积的球面楔形区域 | ∞ |
| d：共面，原点在凸包边上 | 半圆 | 3 |
| e：共面，原点在凸包内部 | 两个相反方向 | 2 |
| f：三维，原点在凸包顶点 | 有面积的球面区域 | ∞ |
| g：三维，原点在凸包边上 | 圆弧 | 3 |
| h：三维，原点在凸包面上 | 一个方向 | 1 |
| i：三维，原点严格在凸包内部 | 无非零方向 | 0 |

方向选择使用法线求和、叉积、投影等分类规则。这里的分值是**按可行方向形态设定的启发式指标**，不是成功概率，也不是严格的球面面积。大圆虽然有无限多个点，分数却是 10；不同宽度的有面积区域都得 ∞，因此不能直接衡量方向扰动容限。

论文随后还有关键的 Step 10：从指定起始偏移沿选定方向到装配位置构造扫掠体，检查与已装配零件的碰撞；碰撞则将 assemblability 置零。Fig. 7 专门说明局部接触方向可行但途中受阻的情况。当前 M2.04 尚未实现这一步，计划中的 06 才负责有限运动。IV.E 再将序列中的稳定性、抓取和 assemblability 最小值组合用于序列选择。

旧源码 `../assembly_planner/asp/utils.py` 的 `AssemblabilityAnalyzer` 延续了凸包分类思想，但分值已调整：例如单法线返回 10、相反法线返回 9，并不等于 v1 的 ∞ 和 10。实际调用处 `asp/sysplanner.py` 另外用法线均值生成方向，并调用 `check_obj_on_the_way`。

新实现应保留逐约束验证，不直接照搬“平均法线就是最佳方向”。一个自行构造的反例：XY 平面单位法线角度为 0°、10°、170°。归一化均值方向约 19.152°，与第三个法线点积约 −0.873，违反约束；85° 却是可行方向。凸包分类、代表方向选择和代表方向验证是三件不同的事。

## 相关原始文献与可借鉴内容

以下不是引用数排名。部分论文使用 admissible motion、accessibility、physical feasibility 等术语，并没有统一的 assemblability 标量。

| 原始文献 | 概念与方法 | 对 WRS2 的启发 |
| --- | --- | --- |
| [Ikeuchi & Suehiro, 1994, *Toward an Assembly Plan from Observation. Part I*](https://www.cvl.iis.u-tokyo.ac.jp/~ki/papers/APO-TRA94.pdf) | 接触法线在 Gaussian sphere 上区分分离半球、保持接触赤道和禁止半球；组合不等式定义装配接触关系及其转换。 | 给局部方向集合建立明确几何解释，同时保留保持接触的低维方向。 |
| [Wilson & Latombe, 1992, *On the Qualitative Structure of a Mechanical Assembly*](https://cdn.aaai.org/AAAI/1992/AAAI92-108.pdf)；[1994, *Geometric Reasoning About Mechanical Assembly*](https://www.sciencedirect.com/science/article/pii/0004370294900485) | NDBG 将方向空间划分为具有相同阻挡关系的区域。1992 文区分无限小平移与沿方向拆至无穷远的平移；后者要检查扫掠区域，可受未接触零件阻挡。 | 当前接触图不等于方向阻挡图。序列搜索需要“谁阻挡谁、在哪类运动下阻挡”的信息。 |
| [Halperin, Latombe & Wilson, 2000, *A General Framework for Assembly Planning: The Motion Space Approach*](https://link.springer.com/article/10.1007/s004539910025) | 用 motion space 参数化装配运动，使一个点代表一种 mating motion，而不绑定移动的零件子集。 | 未来可以从单方向扩展到参数化运动族；不把所有运动硬塞进 S²。 |
| [Hsu & Lin, 2002, *Quantitative measurement of component accessibility and product assemblability for design for assembly application*](https://www.sciencedirect.com/science/article/abs/pii/S0736584501000205) | component accessibility 是理想装配代理实现无碰撞直线接近所需信息量的倒数；结合构型空间可见性，并由零件可达性构成 product assemblability。 | 借鉴“可接近范围”的量化思路；它仍不是具体机器人的可执行性。 |
| [Ostrovsky-Berman & Joskowicz, 2006, *Relative Position Computation for Assembly Planning With Planar Toleranced Parts*](https://sage.cnpereading.com/doi/10.1177/0278364906060133) | 对平面公差零件做最坏情况规划，考虑尺寸、形状和相对放置误差如何缩减允许运动空间。 | 制造公差和定位误差应进入几何可行性模型，不能只靠增大 contact tolerance。本文范围是平面模型。 |
| [Tian et al., 2022, *Assemble Them All: Physics-Based Planning for Generalizable Assembly by Disassembly*](https://people.csail.mit.edu/yunsheng/Assemble-Them-All/paper.pdf) | SDF 提供穿入距离及梯度法线；物理仿真配合动作搜索生成有限拆卸路径，再反向构造装配。 | 最贴近当前 SDF 技术栈，可研究其路径搜索和接触处理；它没有用方向球面积当统一评分。 |
| [Tian et al., 2024, *ASAP: Automated Sequence Planning for Complex Robotic Assembly with Physical Feasibility*](https://www.research.autodesk.com/app/uploads/2024/07/ASAP.pdf) | 在序列搜索中验证路径、重力稳定性和需要扶持的零件，并结合抓取、机器人运动；使用启发式或学习方法引导搜索。 | 对接 05 稳定性、06 路径和 07 序列，把几何、支撑、机器人条件分开验证。 |

阅读范围：Ikeuchi、Wilson 1992、Assemble Them All、ASAP 核对原文相关章节；Halperin、Hsu、Ostrovsky-Berman 核对出版页摘要/可用正文片段。后续复现时仍需逐条核对完整算法及假设。用户所附 v1 则同时核对了正文和图示。

## 建议怎样扩展当前系统

下面是设计建议，尚未新增这些接口：

1. **局部方向集合**：保留 N/A 作为统一约束真值；纯平移输出 empty、isolated directions、arcs/circles、area region 等形态，再用 Gaussian sphere 展示。不能仅看 N 的秩来判断被锁住，因为满秩约束既可能留下八分之一球面，也可能完全限制运动。
2. **方向质量**：为有球面内部的情况计算边界余量，例如在单位 d 上最大化 min_i n_iᵀd，并报告角度余量。欧氏单位球约束不是现有盒约束 LP；正余量情形可以研究 SOCP。退化集合必须另外处理，不能因最小残差恒为零就判坏。
3. **分维度描述自由度**：有面积时报告立体角；弧/圆报告长度；孤立点报告方向数量。不同维度的量不能直接相加，也不应将球面面积零解释为不可装配。子空间内自由和离开该子空间的误差容忍度也要分别描述。
4. **有限路径可行性**：对候选执行直线或 twist 积分路径检查，报告通过/碰撞/未知、碰撞零件和位置。对预算耗尽保留 unknown。先实现这一项，比增加漂亮的单值评分更影响实际正确性。
5. **公差与物理可行性**：另行建模制造/位姿误差、静力平衡、摩擦、夹持和机器人条件。near 作为目标配合可以影响排序，但不能自动成为物理上的 active；SDF 符号未知也不能变成已确认接触。
6. **评分作为策略**：可选保留 `legacy_assemblability` 便于对比论文；新排序使用经过验证的可行性、方向余量、路径条件和支撑成本。不要把一个启发式数值作为全部可装配性的证明。

效率上，方向求解只有 3 或 6 个变量。现有批量残差检查应保留；可按状态缓存约束和候选，谨慎去冗余。建立完整球面细网格既增加成本，也会漏掉测度为零的轴向方向。LP/SVD 或显式多面锥结构处理这些方向更合适。增加方向分类前需要单独测量它与接触提取、有限路径检查的耗时，不能沿用接触面的 benchmark 宣称方向算法最快。

建议顺序：解释方向集合并增加拓扑/质量诊断 → 完成 05/06 的稳定性及有限路径验证 → 在 07 中评价序列。SDF 继续作为几何查询后端；Gaussian sphere 是上层方向表达，两者互补。

## 本页配套示意的验证

使用本机 Edge 无界面检查七个预设切换、拖动旋转、390 px 布局；无 JavaScript 错误和横向溢出。用项目指定 Python 解释器及 NumPy 独立核对均值反例的角度和残差。本轮只增加研究文档和教学 HTML，没有修改求解器或重跑规划回归测试。
