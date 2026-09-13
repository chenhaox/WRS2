# 多方向稳定性：六维采样、方程、量纲与 GPU 独立审查

> 入口整理说明（2026-09-13）：本文的旧脚本命令、输出路径和计时保留为历史记录。当前请按 [WRS 例子使用说明](../../examples/assembly/README.md) 运行，参数在文件顶部修改；benchmark 已移出 examples。

2026-09-11；审查起点 `c51cc3f`，分支 `codex/assembly-planner`。解释器固定为 `D:/code/venv312/.venv/Scripts/python.exe`。本次阅读旧仓库 `9b886441d75f7ea79eafa4cdd5dc90d3c14dbac3`，未运行其 pickle 或旧优化器。

## 1. 结论及实际修改

上一版的平衡矩阵、共享接触力变量和有限容量约束，通过了本轮独立物理模型复核。需要修正的是**采样范围、优化目标的解释和展示**，不能笼统说此前整个“六维稳定性”已经正确。

| 审查项 | 原先行为 | 本轮处理 |
| --- | --- | --- |
| 示例默认 | `force`，只在三维力方向上采样 | 默认 `wrench`，在归一化六维空间的单位超球面 S⁵ 采样 |
| 原 `wrench` 生成器 | 力、力矩分别单位化，合并后再整体单位化；两半长度恒为 1/√2 | 包含全部 ±6 坐标轴及不同力/力矩比例，覆盖纯力、纯力矩、混合方向 |
| 旧式目标 | 两幅值分别优化 `F+T/L`，可能把其中一个压到零 | 默认固定六维方向，只优化一个径向幅值；保留 `legacy_coupled` 对照，明确不是固定六维方向 |
| 尺度 | 声明 L，但只显示 N 等效分数 | 新增 `force_reference_n`，显示无量纲任务承载倍数；示例可直接传 F_ref、T_ref |
| 可视化 | 一张三维力方向球，混合模式丢失力矩方向和幅值比例 | 力与力矩两个三维投影，保留各半部的长度；选中项显示全部六个分量 |
| 后端 | 默认 NumPy | 按本次要求默认 CUDA FP64 批量 LP；CPU 后端仍可显式选择 |

API：`wrs/assembly/stability_sweep.py`。输出版本升为 `wrs.assembly.stability_sweep/2`，摘要版本同时更新。旧 JSON 不应与新的采样指标直接混用。

## 2. 究竟在哪里采样

物理 wrench 为 `w=[F_x,F_y,F_z,τ_x,τ_y,τ_z]`。前三项单位 N，后三项 N·m，直接在它的原始数值上画“单位球”没有统一的物理含义。

先声明任务中有意义的外力与外力矩尺度：

\[
F_0>0,\quad T_0>0,\quad L=T_0/F_0.
\]

然后在**无量纲坐标** `z=[F/F₀,τ/T₀]` 中取单位方向 `d=[d_F,d_T]`，满足：

\[
\|d_F\|_2^2+\|d_T\|_2^2=1.
\]

这是六维向量空间中的五维超球面 S⁵；不是两个三维球面 S²×S²。后者少了“力和力矩之间的比例”这一自由度。原生成器正好遗漏了这一维。

新生成器 `disturbance_directions(count, 'wrench')`：

1. 前十二项是 `+Fx,-Fx,+Fy,-Fy,+Fz,-Fz,+τx,-τx,+τy,-τy,+τz,-τz`。
2. 用六维、非扰乱 Halton 序列生成 `(0,1)^6` 的确定性点；跳过全零点。
3. 每个坐标通过标准正态逆 CDF，得到六维向量，**只对完整六维向量归一化一次**。
4. 加入 `d,-d` 对称配对；增加点数保留已有前缀。没有每次重跑改变的随机种子。

归一化独立标准正态向量具有旋转对称的球面分布；这里采用其确定性 QMC 对应构造。有限 Halton 点不是独立随机样本，也没有全局最大覆盖空洞的保证。测试检查单位长度、正反配对、前缀一致性、坐标二阶矩以及两半长度的分布，避免再次退化成固定比例。

页面下方显示 `d_F`，上方显示 `d_T`。**点可以在球内部**：其距中心的长度就是该半部占比。纯力矩在力投影的中心，纯力在力矩投影的中心。同一编号在两个投影对应同一个六维试验，不能把任一投影当成完整六维球面。

`force`、`torque` 保留独立三维分析；`legacy_coupled` 保留原乘积球面和两幅值目标作历史对照。

## 3. 每个优化问题的完整方程

### 3.1 单个接触及作用反作用

接触点 k 处，取从 A 指向 B 的单位法线 n_k、切向基 t₁,t₂、摩擦系数 μ_k。默认 K=16 的内接多棱锥生成元为：

\[
g_{kq}=n_k+\mu_k\left(\cos\theta_q\,t_1+\sin\theta_q\,t_2\right),
\quad\theta_q=2\pi q/K.
\]

只建立**一组**非负系数 λ_kq：

\[
f_{k,B}=\sum_q\lambda_{kq}g_{kq},\qquad
f_{k,A}=-f_{k,B},\qquad\lambda_{kq}\ge0.
\]

因为每个生成元在法线上的分量为 1，法向力就是 `Σ_q λ_kq`。因此没有拉力，摩擦力位于圆锥的内接近似内。不是给两侧各分配一组力，然后依赖优化器“碰巧”求出相反的力。

**确实减少了变量**：同一 K 下，内部接触点从两侧 2K 列变成 K 列，不再需要额外的三条作用反作用等式。桥梁示例有 20 个力点，320 个摩擦锥系数；其中 12 个点连接两个自由物体，复用消除了 192 个重复系数，物体平衡仍有 4×6=24 行。这里是与“重复建两侧同分辨率变量”比较，不能当成对旧版 4 棱锥变量数的直接降幅。

旧仓库有不同分支：`asp/stability.py` 曾显式建立两侧力相加为零的约束；默认调用的 `asp/stabilitylib/stability.py::gengraspmap` **已经复用两侧变量并对反向基取负号**。因此变量复用不是本次才从无到有的改进。新版将其明确实现为共享矩阵列，避免旧版矩阵的三位小数舍入，并建立回归证据。

### 3.2 每个自由物体分别平衡

令 `s_ik` 表示接触点 k 的力对物体 i 取正或负，c_i 为该物体世界坐标下的质心，p_k 为作用点。每个自由物体都有完整六条等式：

\[
\sum_k s_{ik}f_k+m_i g+F_i^{base}+F_i^{dist}=0,
\]
\[
\sum_k (p_k-c_i)\times(s_{ik}f_k)
+\tau_i^{base}+\tau_i^{dist}=0.
\]

外力矩字段指**关于物体质心的力矩**；若外力 F 作用于别处 p，应传 `τ=(p-c_i)×F`，另有自由力偶则再加上。重力在质心处作用，对质心没有额外力矩。外载荷和几何都在世界坐标系。

不要误加 `τ_A+τ_B=0`，因为这里关于两个不同质心计算。实际上：

\[
\tau_A+\tau_B=(c_A-c_B)\times f_B.
\]

将两侧力矩换到同一个世界原点，内部接触的力和力矩才同时抵消。本轮测试专门验证了这一点。

固定物体代表理想边界，不列其平衡行。若不设置接触容量，它的反力可以无界；因此某些向下压载的方向只会达到查询上限。不能把这解释成真实材料无限承载。有限辅助支撑的作用点、法线、摩擦及容量均显式参与模型，不省略被支撑物体的重力平衡。

### 3.3 容量约束

若 patch 或辅助支撑组有法向力上限 C_g，施加：

\[
\sum_{k\in g}\sum_q\lambda_{kq}\le C_g.
\]

这个上限由组内**所有点、所有生成元共同分享**，不是每个顶点都有一份 C_g。故多采几个接触点不会凭空增加该组的总容量。模型尚无局部压力上限、柔顺性或应力模型；要引入它们，需要改变约束，而不只是增大采样数。

### 3.4 固定一个六维方向，最大化一个幅值

对指定物体 i 与指定归一化方向 d_j，外扰为：

\[
F_i^{dist}=\alpha d_{F,j},\qquad
\tau_i^{dist}=L\alpha d_{T,j},\qquad\alpha\ge0.
\]

其余物体的额外扰动为零，但全部物体的接触力都要重新求解。压缩写成：

\[
\begin{array}{ll}
\operatorname{maximize}_{\lambda,\alpha}&\alpha\\
\text{subject to}&A\lambda+w_0+E_i\operatorname{diag}(I_3,L I_3)d_j\alpha=0,\\
&H\lambda\le c,\quad\lambda\ge0,\quad0\le\alpha\le\alpha_{query}.
\end{array}
\]

A 包含每个物体的力及关于其 COM 的力矩；w₀ 是重力和基础外载荷；E_i 将六维扰动插入目标物体的六行；H 是共享容量约束。

所有系数都固定，只有 λ、α 是未知数，所以这是 **LP**，不需要 SLSQP，也无需为了“六维”改成非线性优化。若把多棱锥换成精确圆锥，同样的径向问题才成为 SOCP。

默认自由 wrench 模式不继承旧约束 `T≤radius*F`：外部工具可能独立施加力偶，纯力矩本来就是应测试的方向。若任务限定为表面某一点施加外力，则应使用更精确的关系 `τ=(p−c)×F`，而非只用一个半径不等式猜测力矩方向；这属于下文尚待加入的 `surface_force` 任务模型。

配置 `StabilityConfig.characteristic_length_m` 只用于把力矩方程的行缩放，改善数值条件；**不是**上面的物理任务长度 `StabilitySweepConfig.torque_length_m`。前者改变不应改变物理解，后者改变会改变要比较的外扰集合。本轮分别改变数值缩放验证了结果不变。

## 4. 最后是否 N 次优化取最小值

是，但默认目标已修正为固定六维方向的径向极限：

\[
\widehat\rho=\min_{i,j}\frac{\alpha^*_{ij}}{F_0}.
\]

每件 N 个方向，检查 M 个自由物体，共 M×N 个独立 LP。例如桥梁 4 件、每件 6,500 方向，共 26,000 次优化；GPU 将它们分批一起算。

它不是所有方向同时加载，不是旋转装配体后重算重力，也不是任意多物体同时扰动。若需要任意联合多物体扰动，完整外载空间是 6M 维，须另定义联合任务集合。

还必须区分有限采样与连续最坏情况。对完整连续 S⁵ 的真实最小径向值 ρ*，若采样最小项是真实有限最优而非查询截断，则：

\[
\widehat\rho\ge\rho^*.
\]

采样可能漏掉更弱方向，所以它是相对于当前离散接触模型的**乐观上界**。不能把“采样承载倍数大于 1”直接宣布为“任意六维扰动都稳”。若所有项都达到查询上限，只知道被测射线能承受至该值；对连续最坏方向没有因此得到下界。任何未知项均使正式 `sampled_minimum_n` / `sampled_minimum_load_factor` 为 `None`。

此外，16 棱摩擦锥是物理圆锥的保守内接近似，而方向漏采可能偏乐观；两种误差不能相互抵消后称为严格物理保证。

## 5. 为什么 F+T/L 仍可能被 F 支配

旧入口 `asp/stabilitylib/stability.py:216` 返回 `-(kf+kt)`；381 行附近限制 `kt<=radius*kf`。它存在两层问题：

- N 和 N·m 直接相加，改变长度单位就改变指标。
- 两个幅值独立选择：即使改成同单位的 `F+T/L`，优化器仍可选择只增大更容易承受的一项。

本轮加入了一个具有解析解的真实方块反例：2 kg、底面 0.1×0.1 m、μ=0.5，L=0.1 m；外扰方向为向上的 F_z 和绕 x 的力矩。极限由：

\[
F_z+T_x/0.05\le19.62
\]

决定。旧式 `max(F+T/L)` 得到 **F=19.62 N、T=0**，分数 19.62 N；已经不沿原本想检查的混合六维方向了。

固定 `d_F=+z/√2`、`d_T=+x/√2` 后：

\[
\alpha^*=19.62\sqrt2/3=9.249\ldots\text{ N},\quad
F=6.54\text{ N},\quad T=0.654\text{ N·m}.
\]

这个解必须保留同一力／力矩比例。归一化和径向目标要**同时**处理，单独换一个 L 不能解决独立两幅值“偏科”的问题。

### 推荐指标：按任务定义无量纲载荷

默认示例声明 `F₀=10 N、T₀=1 N·m`，对应 L=0.1 m，然后报告 `ρ=α/F₀`。它描述任务椭球：

\[
\mathcal E(\rho)=\left\{(F,\tau):
\|F/F_0\|_2^2+\|\tau/T_0\|_2^2\le\rho^2\right\}.
\]

纯力 10 N 与纯力矩 1 N·m 都是该示例任务的单位强度。改成 N·mm 时，力矩数值与 T₀ 必须一起乘 1,000，归一化向量和承载倍数不会改变。**这两个默认值是示例尺度，不是所有装配任务的通用正确参数。**

应优先根据插装力、工具力矩、夹具误差等任务载荷给 F₀/T₀；没有任务数据时，可暂用几何特征长度，但必须明确选择并做尺度敏感性分析。不同大小物体采用同一任务负载，和按各自尺度比较相对质量，是两个问题，不能混用分数。

力或力矩中某一项仍可能成为真正薄弱因素，这是合理结果；要消除的是量纲或优化目标造成的虚假支配，而不是强行让两种物理作用各占一半。若实际任务是“每个轴都有独立最大值”的盒形集合，或 F/T 存在相关性，当前各向同性椭球也不等价；后续可扩展为显式任务多面体或一般缩放矩阵 S。

## 6. 可借鉴的文献与迁移边界

| 原始文献 | 方法与可借鉴部分 | 在本系统中的状态 |
| --- | --- | --- |
| [Ferrari & Canny, Planning Optimal Grasps, ICRA 1992](https://people.eecs.berkeley.edu/~jfc/papers/92/FCicra92.pdf) | 用 wrench 空间衡量对最不利扰动的抵抗能力，区分不同手指力约束 | 借鉴最坏方向思想；不能把当前含重力、多个物体和部分无界支撑的可行集合直接叫作经典归一化 GWS 的精确 ε 指标 |
| [Li & Sastry, Task-Oriented Optimal Grasping；1986 技术报告](https://www2.eecs.berkeley.edu/Pubs/TechRpts/1986/Archive/ERL-86-43.pdf)，期刊版 1988，DOI 10.1109/56.769 | 用任务椭球表示不同扰动方向及相对尺度 | 本轮采用显式任务力/力矩尺度；一般非对角椭球尚未实现 |
| [Borst, Fischer & Hirzinger, Grasp Planning: How to Choose a Suitable Task Wrench Space, ICRA 2004](https://www.robotic.dlr.de/fileadmin/robotic/borst/Borst-ICRA2004-TaskWrenchSpace.pdf) | 将任务/物体可产生的扰动集合与抵抗集合比较；以椭球近似和变换辅助求值 | 可借鉴物体几何生成任务集，以及在薄弱区域增量求值；当前未移植其 OWS 构造或椭球近似算法 |
| [Strandberg & Wahlberg, A Method for Grasp Evaluation Based on Disturbance Force Rejection, T-RO 2006](https://kth.diva-portal.org/smash/get/diva2%3A380383/FULLTEXT01.pdf) | 对物体表面施加扰动力，通过 min-max 衡量抵抗能力；结合物体几何与任务信息 | 很适合增加 `surface_force` 模式：在可受扰表面取作用点和允许力方向，令 τ=(p−c)×F。尚未实现；其物理任务不同于任意六维自由 wrench |

上表是对原始论文的借鉴判断，不声称现有代码完整复现了任意一篇论文。尤其是“只由表面单点外力产生的力矩”，和“外部工具能独立施加力偶”，不能用同一任务集合代替。

如果后续获得抵抗集合的精确半空间表示 `C={w:h_kᵀw≤b_k}`，且 w 表示相对于当前基础载荷的额外扰动、原点可行，那么任务集合 `w=ρSu, ||u||≤1` 的连续精确半径可直接由：

\[
\rho^*=\min_k\frac{b_k}{\|S^T h_k\|_2}
\]

得到，零分母行按其可行性单独处理。这是半空间上最大化线性函数的直接推导，不需要球面采样；但把多物体接触变量投影掉并生成全部面可能非常昂贵。当前没有该投影算法，不能把采样结果写成这个精确值。增量分离/薄弱方向加密可作为下一步研究。

## 7. GPU 实现和独立验证

共享接触几何、A、容量行及基础载荷，只让目标物体的六行扰动列变化。先求一个 α=0 的公共可行基，之后以 PyTorch CUDA FP64 批量执行 simplex 的 rank-one pivot；每八轮检查整批是否完成。NumPy 使用相同算法并控制 CPU 工作集大小；HiGHS 是逐 LP 参考后端。

GPU 解回到 CPU 后，在**原始矩阵**上重新检查非负性、平衡残差、容量松弛、对偶可行性和 primal-dual gap。迭代失败或数值复核失败，默认单项回退 HiGHS，并计数；关闭回退则为 unknown。秩退化模型也有明确回退。这里是浮点容差下的数值复核，不是区间算术证明。

默认采用 CUDA，是本机批量任务的实测选择；不宣称它对任意 GPU、任意小问题都比 CPU 快。首次导入/初始化需要单独付出时间，实际循环应复用 `DirectionalStabilityAnalyzer`。CUDA 不可用会明确报错；CPU 机器请传 `backend='numpy'` 或 `'highs'`。

验证分成两层，避免“两个求解器在同一错误矩阵上得到同一个答案”：

1. **独立物理模型**：`tests/assembly/test_wrench_stability_audit.py::cartesian_reference` 给两侧分别建立三维 Cartesian 力，显式加入 `f_A+f_B=0`；用摩擦多边形半空间约束；关于世界原点重写力矩。没有复用生产矩阵、锥生成元或扰动列。堆叠、桥梁、斜面、有限辅助支撑、基础力和力偶均逐项比较。默认 CUDA 的桥梁也与此独立模型逐项对照。
2. **求解器规模对照**：同一完整六维方向集合分别运行 HiGHS、NumPy、CUDA，包含每件 300 和 6,500 方向。此层验证批量 LP 的数值一致性，不能代替第一层物理建模检查。

其他测试包括解析抬升/滑动/倾覆、真实 `F+T/L` 反例、作用反作用的公共原点抵消、行缩放不变性、任务尺度改变、载荷重构、有限容量、秩退化和失败状态。已有独立精确 Coulomb SOCP 测试继续检查 16 棱锥的保守性。

### 实测结果

本机 NVIDIA GeForce RTX 5060 Laptop GPU，FP64，16 棱锥、批量 256、查询上限 100 N 等效；已运行的 WRS 页面保留，耗时会随机器负载变化。每件 300 方向取三次热运行中位数，6,500 方向取一次热运行；另记录第一次查询。几何提取及公共模型准备分别计时，方向数组在查询计时外生成；查询包含归一化、上传、求解、下载、CPU 原矩阵复核及结构化结果。

| 场景 | 每件方向数 | LP 总数 | HiGHS | NumPy 批量 | CUDA 批量 |
| --- | ---: | ---: | ---: | ---: | ---: |
| stack | 300 | 600 | 1.024 s | 0.156 s | 0.106 s |
| stack | 6,500 | 13,000 | 23.427 s | 3.182 s | 1.703 s |
| bridge | 300 | 1,200 | 3.485 s | 2.068 s | 0.482 s |
| bridge | 6,500 | 26,000 | 73.177 s | 47.667 s | 9.035 s |

12 组均为 complete，NumPy/CUDA 回退数均为零；对逐项 HiGHS 的最大分数差 **2.56×10⁻¹¹ N 等效**。首次 CUDA 准备约 **2.283 s**，首次 stack 查询 0.304 s；不能用热查询时间代替首次启动等待。桥梁 26,000 LP 的 CUDA 查询中，批量求解及传输 7.302 s、CPU 复核 1.456 s，剩余为矩阵/结果组织；已做向量化，但仍是实际求解很多 LP，不是仅筛选球面点。

完整快照：[wrench-stability-benchmark.json](assets/wrench-stability-benchmark.json)。该基准是同一新版物理模型上的后端比较，不是旧系统端到端加速比。

桥梁采样最小值由 300 方向的 6.2430 N 等效降为 6,500 方向的 6.1229 N 等效，直接说明增加方向可能找到更弱项。它仍不能证明 6.1229 就是连续六维最坏值。

完整装配回归 **158 项通过**；其中 32 项覆盖静力、方向承载与本轮独立方程审查。无新增或升级依赖。

显示材质与回放测试另有 **2 项通过**。8897 页面已实测纯力、纯力矩、混合方向、部件切换及最弱方向按钮，选中载荷的重新平衡均为 feasible；示例 F_ref=20 N、T_ref=0.5 N·m 的命令及 JSON 字段也已验证。页面保留 nominal 工况的摩擦锥/反力，明确区别于当前选中的额外外载荷。

## 8. 使用与交接

```python
from wrs.assembly import StabilitySweepConfig, DirectionalStabilityAnalyzer

cfg = StabilitySweepConfig(
    backend='cuda', mode='wrench', direction_count=6500,
    force_reference_n=10., torque_length_m=1./10.,
)
analyzer = DirectionalStabilityAnalyzer(assembly, state, graph, config=cfg)
result = analyzer.analyze()
print(result.sampled_minimum_load_factor)  # 无量纲；仍只是采样集合的指标
```

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/stability_sweep_demo.py --case bridge
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/stability_sweep_demo.py --case stack --force-reference-n 20 --torque-reference-nm 0.5 --headless
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/benchmark_stability_sweep.py --mode wrench --counts 300 6500 --repeats 3 --large-repeats 1
& 'D:\code\venv312\.venv\Scripts\python.exe' -m unittest discover -s tests/assembly -p 'test_*.py'
```

当前本地页面为 `http://127.0.0.1:8897/`。本次不改变 M2 序列的排序目标，也不把该采样指标自动升级为序列/机器人执行的全局鲁棒性保证。
