# 两种方向方法与静力平衡：实现和交接

2026-09-11。M2.04 增加确定性的 Fibonacci / SOCP 公共接口；M2.05 静力平衡、扰动检查和有限辅助支撑需求已实现。有限路径（06）、序列（07）和机器人执行尚未实现。

## 直接看 WRS 示例

在 WRS2 根目录、两个终端分别运行：

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/directions_fibonacci.py --samples 6500
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/directions_socp.py
```

两个脚本各约 35 行，均为“创建案例 → 调用求解器 → WRS 绘图 → base.run()”。共用的模型和绘图放在 `_direction_example.py`。使用本仓库原生 `wvw.World` / `wssop`，浏览器是当前 WRS 的 WebGPU viewer；端口分别为 8891、8892。

左边显示零件和选中方向；右边是方向球。**绿点表示 Fibonacci 筛出的方向，蓝箭头表示选中方向，红箭头表示约束法线；灰点仅画球的轮廓。** SOCP 只画一个蓝箭头，没有把显示点伪装成优化候选。

主分支 UI 更新后，左边也默认画出接触面；面板可切换显示并调整零件透明度。检测区域按 active/near/interference/unknown 着色，轴孔中声明的理想配合面单独用蓝色标注。[合并与绘图说明](ui-merge-and-contact-display.zh.md)

`--case` 支持 `plane`（桌面）、`corner`（三面角）、`channel`（上下夹住）、`shaft`（通孔轴）、`blind_shaft`（带底轴孔）、`blocked`（六面封闭）。加 `--headless` 只计算。例：

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/directions_fibonacci.py --case shaft
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/directions_socp.py --case shaft
```

箱体案例从真实接触分析提取法线。**轴孔示例明确输入理想零间隙圆柱约束**，用于检验退化方向：通孔允许 ±轴向，SOCP 按偏好只选一个；盲孔只允许朝外一个方向。它不代表 SDF 已能从带间隙 STL 证明完整 active 圆柱面。已有接触的 unknown/near 语义继续保留。

本轮已经实际渲染角落、轴孔，并保存 WRS 截图到 `examples/assembly/output/direction-methods/wrs-*.png`；这是忽略的可再生成输出目录。

## 程序怎么调用

真实装配优先从接触图调用：

```python
from wrs.assembly import DirectionConfig, assembly_directions

sampled = assembly_directions(graph, ['part'],
    config=DirectionConfig(method='fibonacci', sample_count=6500))
optimal = assembly_directions(graph, ['part'],
    config=DirectionConfig(method='socp', preferred_direction=(0, 0, 1)))

print(optimal.status, optimal.best_direction)
```

已有世界坐标系法线矩阵 `N` 时，调用 `solve_directions(N, config=...)`。每行的符号满足 `N @ d >= 0`。接口只求三维纯平移；旧 `candidate_motions` 保留，用于原有候选和六维 twist，不改变其调用者。

`DirectionResult` 保存 `directions`、`best_direction`、可行空间维数 `dimension`、正交基 `basis`、角度余量、求解证据和计时。数组和元数据只读。`assembly_directions` 遇到不确定接触时，外层状态保持 `unknown`，同时记录理想约束模型下的结果，不能据此直接执行动作。

## 两种方法的共同边界

Fibonacci 用中点 z 和黄金角生成固定球面点，缓存只读点集；用 NumPy `einsum` 分块计算点积，先测区分度较大的法线，再逐批剔除不满足约束的点。默认每次最多 2048 个方向 × 32 条后续法线，避免一次分配巨大矩阵。这里没有随机种子，也不使用逐点 Python 循环或 `np.vectorize`。

SOCP 使用原生 Clarabel，求解：

```text
maximize    t
subject to  N d >= t
            ||d||₂ <= 1
```

单位法线下，正余量解在单位球上，`asin(t)` 是到最近约束边界的角度余量。它优化的是局部方向对角度偏差的容忍程度，尚未优化有限路径、行程、机器人或整套装配质量。每次重建小型求解模型，并复核单位长度、约束残差和求解器状态。

SOCP 是可选依赖：`pyproject.toml` 的 `assembly-planning` extra 包含 `clarabel>=0.11.1`；此次指定解释器已安装 Clarabel 0.11.1。Fibonacci、静力和接触模块不需要导入 Clarabel，缺少它时 SOCP 给出明确安装提示。

## 退化情况如何处理

不能把最大余量为零解释成锁死：平面内运动、轴向运动的三维角度余量本来就是零。

1. 成对精确相反的法线直接推出等式，例如 `dx>=0` 与 `-dx>=0` 得到 `dx=0`，无需 LP。
2. 一般法线用带预算的 LP 寻找始终取等号的约束，并验证非负线性依赖证据。仅凭“目标值接近零”不降维。
3. 在等式零空间中重新求解。二维时 Fibonacci 改为确定性圆周点，SOCP 优化平面内角度余量；一维时直接检验两个轴端点；零维有可靠证据时才是 `blocked`。
4. 一维或整个自由平面中没有可优化的剩余边界时，按 `preferred_direction` 选一个；偏好垂直于空间时按固定坐标次序选择。不会把轴向可行性写成三维大角度容差。

`angular_margin_rad` 是原始三维约束余量；`intrinsic_angular_margin_rad` 是允许子空间内的余量。一维轴上的后者是 `None`，不是 90°。没有约束时余量也不伪造。

**浮点边界：** 旋转后的圆柱法线可能不再精确共面。默认允许以约 `2.84e-14` 的机器运算量级验证近似依赖，并把 `dimension_evidence` 标成 `arithmetic_roundoff_model`，附残差上界；这不是用户指定的几何误差容许量。严格按输入浮点系数求等式时，设 `allow_roundoff_reduction=False`。近似降维不能用于证明锁死；输出仍逐条检查原始法线。不确定的窄锥、数值失败或预算不足返回 `unknown`。

有限采样漏掉已确认的三维窄可行锥，返回 `no_sample_hit`，不返回 `blocked`。测试中 0.1° 窄锥会被 6500 球面点漏掉，而 SOCP 能求出其中心。

## 完整生产 API 的实测耗时

指定 Python 3.12.0、NumPy 1.26.4、SciPy 1.16.2、Clarabel 0.11.1；6500 点，预热后 31 次**独立调用**，下表为中位数。包含输入归一化、精确去重、分块筛选、退化判断、结果构造和残差检查；不包含接触提取、SDF 构建、绘图或路径验证。Fibonacci 的固定点集已缓存，求解结果没有缓存。

| 案例 | Fibonacci | SOCP | 输出 |
| --- | ---: | ---: | --- |
| 桌面 | 0.351 ms | 0.322 ms | 半球 / 一个最优方向 |
| 三面角 | 0.355 ms | 0.307 ms | 813 点 / 一个最优方向 |
| 平行槽 | 0.965 ms | 0.385 ms | 圆周 6500 点 / 一个偏好方向 |
| 理想通孔轴 | 0.431 ms | 0.545 ms | 两个轴向 / 一个轴向 |
| 理想盲孔轴 | 0.488 ms | 0.491 ms | 一个轴向 |
| 六面锁死 | 0.350 ms | 0.315 ms | blocked |
| 32 条旋转后的径向法线 | 4.774 ms | 5.038 ms | 带机器精度诊断的轴向 |

前六例 P95 约 0.47–1.34 ms；32 条旋转法线仍需要 LP 依赖证据，P95 约 5.49–5.68 ms。精确相反法线快速路径将这台机器上的轴孔 API 从约 5 ms 降到约 0.5 ms。复杂证据判断不保证一律亚毫秒，旧实验中的单个 SOCP 核心时间也不等于完整 API 时间。

复现和逐次数据：

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/benchmark_planning_api.py --samples 6500 --repeat 31
```

结果为 `examples/assembly/output/direction-methods/production-api.json`，记录首调、单次列表、中位数、P95、维数和 LP 次数。此处首调列包含该调用触发的惰性依赖导入，不能视为热调用时间。更早的 `benchmark_direction_methods.py` 仍保留为算法实验。

## M2.05 静力和有限支撑

新增可切换的七种 WRS 场景、实际求解点与摩擦锥显示，以及独立圆锥求解对照，见 [稳定性可视化与计算核查](stability-validation.zh.md)。`stability_demo.py` 现在默认显示双墩场景；原堆叠场景用 `--case stack`。

代码对照与追加实测：[当前静力、扰动和支撑与旧版的区别](stability-old-new.zh.md)，含 6/24 个载荷及辅助支撑搜索的阶段耗时。

```python
from wrs.assembly import check_equilibrium, StabilityConfig, LoadCase, ExternalWrench

config = StabilityConfig(disturbances=(
    LoadCase('push', (ExternalWrench('upper', force_world_n=(1, 0, 0)),)),
))
balance = check_equilibrium(assembly, state, graph, config=config)
print(balance.status, balance.robustness_status)
```

每个自由体单独建立 3 条力平衡和 3 条 COM 力矩平衡。内部接触共享变量，在共同作用点施加等大反向力；不提供虚构的自由力矩。均匀平面/线接触用实际单元顶点；曲面保留对应点和法线场；孔洞内不添加力点。孔周围的真实支撑仍可以托住位于孔上方的 COM，这不等于填孔。

使用 SciPy HiGHS 和稀疏矩阵，默认 16 边内接摩擦锥，横向能力的最坏径向比例为 `cos(pi/16)≈0.981`。当前接触对摩擦取两侧已声明材料值的较小值，这是明确模型假设。COM、质量或所需摩擦未知返回 `unknown`；near/unknown 区域不承载；理想化 active 默认禁用，需显式开启。固定件是理想边界，缺少接触力上限时也会注明理想无限承载假设。

`max_contact_normal_force_n` 是**每块 contact patch 所有力点的法向力总上限**。辅助 `SupportCandidate` 则有各自的有限法向能力、摩擦和作用点。外力矩单位 N·m，关于该体 COM。力矩行按 `characteristic_length_m` 缩放求解，再以 N、N·m 复核残差。

`status=feasible` 只表示声明离散模型的名义平衡；扰动结果在 `disturbances` 与 `robustness_status` 中另报。`infeasible` 只说明该内接摩擦锥/支撑点模型不可行，不证明所有真实接触模式都不可能。数值失败为 `unknown`。

`find_support_requirements` 按有限候选数量递增枚举，并验证名义及声明扰动载荷。输出 `requirements_found` 是支撑需求，尚非可执行机器人抓取；最小性只在已声明候选集、已完成检验和该力模型内成立。预算耗尽不会改写成“无解”。物理输入、几何/状态和完整接触证据进入 `input_digest`，当前没有结果缓存。

静力 WRS 示例（端口 8893）：

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/stability_demo.py
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/stability_demo.py --case floating --headless
```

1 kg + 2 kg 堆叠，桌面合支持力 29.43 N；去掉桌面则无支撑不可行，两个各限 20 N 的支撑候选共同满足需求，单个不足。名义静力 API 热调用中位数：堆叠 3.499 ms，悬空 2.168 ms，均不含接触提取。图中红箭头是重力、绿箭头是接触在自由体一侧的力（双方都自由时显示 B 侧），比例 0.004 m/N；完整成对反力在结果中保留。

## 验证与下一步

新增方向测试覆盖解析最优方向、旋转协变、平面/轴/盲孔、无相反对的径向约束、机器精度退化、弱约束、漏采与锁死区分、图证据 unknown 传播。静力测试覆盖成对内力、多自由体、斜面摩擦、倾覆、点/线力矩、孔洞、有限支撑、扰动、输入变化、数值失败和不可变结果。

最终回归：指定解释器运行 `-m unittest discover -s tests/assembly -p 'test_*.py'`，**100 项通过，15.439 s**（含 14 项新方向测试、13 项新静力测试）。额外验证 Clarabel 缺失时 Fibonacci 仍能使用、求解器数值失败返回可 JSON 序列化的 unknown。WRS 的两种角落、两种轴孔和静力堆叠场景均实际渲染检查；API 索引已更新。

下一任务是 [06 有限零件运动](tasks/06-part-motion.zh.md)：把局部方向送入有限步长的路径验证，保留 unknown/碰撞证据，再为 07 提供动作结果。Assemble Them All 的迁移候选仍是待实现项，见原有迁移文档。
