# 九类方向与论文装配例子

2026-09-13。本次检查用户提供的 *Planning to Build Block Structures With Unstable Intermediate States Using Two Manipulators*（IEEE T-ASE，DOI 10.1109/TASE.2021.3136006，17 页提交稿），以及旧代码 `asp/tool/tool_stability_comparasion.py` 的场景清单。例子验证接触与局部方向，不把论文的 S/G/A 数值、搜索性能或实机实验当作已复现的结果。

## 直接运行

在 `D:\code\ch\asp\WRS2` 中：

```powershell
$assemblyPython = 'D:\code\venv312\.venv\Scripts\python.exe'
& $assemblyPython examples/assembly/directions.py
& $assemblyPython examples/assembly/paper_assemblies.py
```

WRS 控件保持简单：选择图号后点“加载所选场景”；选择当前移动零件；选择 `socp` 或 `fibonacci`。中间状态 `0` 是完整结构，`1..N` 是检查顺序的前 N 件。每步重新绑定几何和位姿、提取当前接触，地面保持最终装配位置的地面高度。前缀里有悬空件并不奇怪；本例未模拟辅助机械臂托持，不能据此认定该中间状态稳定。

左边画实际点/线/面接触，保留孔洞及干涉区域，滑条调零件不透明度。右边是世界坐标方向球：蓝箭头为拆出方向，紫箭头为反向插入方向。SOCP 画一个连续最优方向；Fibonacci 画筛选后的可行集合，并显示其选中方向。文件顶部默认 `SAMPLES = 6500`，退化情形使用可行子空间上的圆或轴端点。灰点仅勾勒球面。分数来自可行锥分类，不来自绿色显示点的数量。

`unknown` 时保留接触证据与原因，不画通过验证的方向；`blocked` 是当前局部纯平移模型锁死；`unconstrained` 是当前没有有效约束。它们都不是“稳定性”标签。即使 `feasible`，也仍需有限路径和机器人执行验证。

## 九种情况

入口：[directions.py](../../examples/assembly/directions.py)。用一个可动立方体和若干固定挡块构造实际几何，再提取接触法线；期望表不直接传给方向求解器。

| 类别 | 场景 | 可行空间维数 | ASP_OLD 分数 A |
|---|---|---:|---:|
| a | 单面接触，半球 | 3 | 10 |
| b | 相反双面，完整大圆 | 2 | 9 |
| c | 两面转角，球面楔形 | 3 | 10 |
| d | 双面槽加挡板，半圆 | 2 | 3 |
| e | 四面导向，双向轴 | 1 | 2 |
| f | 三面转角，球面区域 | 3 | 10 |
| g | 双面槽加两挡板，圆弧 | 2 | 3 |
| h | 五面盲槽，单向轴 | 1 | 1 |
| i | 六面包围，纯平移锁死 | 0 | 0 |

两种算法都验证了类别、分数、维数、方向约束残差和解析接触面积。这里采用现有 `asp_old` 有限评分表；早期 2016 论文的无穷大分支仍可通过独立评分函数的 `wan2016` profile 获得。

## 论文图号覆盖与数据来源

入口：[paper_assemblies.py](../../examples/assembly/paper_assemblies.py)。共 **20 个图号入口**，包含重复使用同一结构的实验变体；不是 20 组互不相同的原始装配。

| 图号 / 入口 | 结构、来源 | 复现范围 |
|---|---|---|
| Fig.8 `fig08_soma3` | Soma 3；`datainfo2` | 原始 STL、位姿，名义模式修复网格与角度 |
| Fig.9 `fig09_domino3` | Domino 3；`domino_5` | 原始 STL，20° 名义相切位姿 |
| Fig.10 `fig10_burr6` | Burr 6；`burrpuzzle` | raw 保留原始穿插；nominal 使用显式槽面修复，见 [Burr 修复](burr-fix.zh.md) |
| Fig.11 `fig11_bridge6` | 六件达芬奇桥；`bridge` | 缺原 STL，声明的实心梁重建 |
| Fig.12(a/b) `fig12a/b` | 两件搭接结构 | 两份 JSON 缺失，按图作示意重建；不是精确比较模型 |
| Fig.12(c) `fig12c` | 三件 Soma；`datainfo7` | 原始数据的名义网格模式 |
| Fig.12(d) `fig12d` | 三件 Soma；`datainfo0209_3` | 论文故意选的**不稳定**结构；方向可行与稳定性无关 |
| Fig.12(e) `fig12e` | 四件 Soma；`datainfo4` | 原始数据的名义网格模式 |
| Fig.12(f) `fig12f` | `domino_5` | 与 Fig.9 相同模型 |
| Fig.12(g) `fig12g` | 偏置叠放；`domino_7` | 保留原始位置，恢复导出角度 |
| Fig.12(h) `fig12h` | 两腿承梁；`domino_8` | 保留原始位置，恢复导出角度 |
| Fig.13(a) `fig13a` | 单 BL | 原始块的网格表示 |
| Fig.13(b) `fig13b` | BL + Z；`datainfo_ss` | 相应旧场景候选，未核实所有图中位姿 |
| Fig.13(c/d) `fig13c_replacement`、`fig13d_replacement` | 5 / 7 件连接体素结构 | 固定种子替代样本，非原图数据 |
| Fig.15(a) `fig15a_soma4` | 四件 Soma；`datainfo4` | 实机结构的旧场景候选，未核实所有原图位姿 |
| Fig.15(b) `fig15b_domino4` | 四件 Domino；`domino_6` | 实机变体的旧场景候选；两个零件接触仍未解决 |
| Fig.15(c/d) `fig15c_burr6`、`fig15d_bridge6` | Burr / 桥 | 复用 Fig.10 / Fig.11，非机器人回放 |

Fig.1 的结构属于 Soma 主例，Fig.7 的零件集用于 Fig.8–11。Fig.3 等流程示意不产生额外实物测试。Fig.14 是随机样本的性能统计，原种子和逐次目标结构未提供，因此**没有声称复现其十次实验曲线**。Fig.13 的替代生成器采用 24 种正交旋转、无体素重叠和面连通组合，记录种子与完整体素列表；尚未加入论文的最终稳定性筛选和矩阵长宽深约束。

检查顺序用于观察中间状态。Soma3、Domino3 参考论文高亮序列；Burr 使用对应零件名的检查序列；其余默认旧数据顺序。桥的 `Alframe` 实例尚未逐一与论文 AF 颜色编号对应。**这些顺序不表示本次重新求得并验证了论文的最优双臂序列。**

## 名义版本究竟修复了什么

原输入随例子打包为 13 个 STL、12 个 JSON，来源 commit 为 `9b886441d75f7ea79eafa4cdd5dc90d3c14dbac3`；[provenance.json](../../examples/assembly/assets/paper2021/provenance.json) 保存逐文件 SHA-256。运行例子不需要旧仓库。

- Soma：网格边长 18.5 mm，保留原始 JSON 平移。37 mm 可以是两个方块长度，不能据此把位姿缩小一半。将微小导出误差恢复到该网格；只对已知的轴对齐体素块计算中心点绕数，要求整数占据，再提取体素并集外表面。消除内部面和 T 形接缝，保持方块数量、体积及装配位置；不是任意 STL 自动补洞。
- 导出角度：仅当角度距相应整度/45° 网格小于 `6e-7 rad` 时恢复。每个零件记录原始顶点在修改前后世界位置的最大变化；Soma 再三角化后的顶点编号不再与原 STL 一一对应，原文件保留独立证据。
- Domino3：按 20° 倾斜、20×60×120 mm 原始块解析恢复与地面和中央块相切的位置，避免把六位小数导出误差当成实际间隙。
- 桥：原 `alframe.stl` 缺失，采用 **30×30×300 mm 实心梁**，参考旧方向与布局，解析放置相切位置。这不是原铝型材，不用于声称复现其质量、夹持或稳定性数值。
- Burr：raw 保留旧文件的间隙和干涉；nominal 离线对齐槽面并重建封闭边界，记录每件修正量，最大约 0.28 mm。这是名义配合模型，不是原作者 CAD 的精确恢复。下文初次发布的统计和待补项目为本次修复前的历史基线，当前结果见 [Burr 修复](burr-fix.zh.md)。

将文件顶部 `NOMINAL = False` 完全保留原 STL/JSON，只补明确声明的固定地面。缺原输入时报告 `missing_source`，不回退到重建模式。

## 本次发现并修复的接触问题

默认例子使用现有 mesh 接触分析：共面/近平面区域裁剪，加 BVH 整体相交检查。**不是 SDF 点云采样实验，也没有用手填法线代替检测。**

新增 [support.py](../../wrs/assembly/contact/support.py)：在整体已判定 touching、且原分析没有接触区域时，识别封闭实体的“支撑面内部—棱边/顶点”接触。完整两物体必须分别位于同一平面的两个半空间；B 的接触特征维数至多一维；线段/点再与 A 的实际三角形区域相交，不能越过孔洞。支撑面的外法线和 B 顶点/棱边法线锥中的反向法线形成作用反作用方向。面积为零，线接触保留真实长度，不伪造有限接触面积。

该分支不处理一般棱—棱、角—角接触，也不把穿透改成接触。前者可能需要多个运动锥的并集。预算耗尽、几何误差和其他未解决特征继续向上传递；未解决场景不能作为已验证方向。接口结果仍只是局部纯平移约束。

另外修复 [planar.py](../../wrs/assembly/geometry/planar.py) 的边界重建：邻点到边的距离曾放宽到 `2*tol`，会把真实相邻边界顶点误当成 T 形接缝，形成分叉；改为严格使用所声明的 `tol`。Fig.12(h) 两条支腿的有效重叠宽度分别为 10、20 mm，接触面积按真实重叠计算，不强行对称。

## 检查结果与计时

```powershell
& $assemblyPython benchmarks/assembly/nine_cases.py
& $assemblyPython benchmarks/assembly/paper_cases.py
# 检查中间步骤：先在 paper_cases.py 顶部设置 PREFIXES = True，再运行。
# 检查原始数据：设置 NOMINAL = False，再运行。
& $assemblyPython -m unittest discover -s tests/assembly -v
```

177 项 assembly 回归通过（本机约 121 s），包括新增的 11 项测试方法：实际九类、资产哈希、网格体积/封闭性、固定种子连通性、位姿绑定、点线接触、刚体变换/交换次序、间隙/穿透、预算耗尽、梁接触面积与 Burr 不能误报成功。

20 个完整名义场景入口中，76 个逐件方向查询有 61 个 `feasible`、1 个 `blocked`、14 个 `unknown`；两种算法都执行了。14 个未知来自 Burr 的两个重复入口（各六件）和四件 Domino 中的两件。76 个前缀另检查了 209 个逐件方向组合。它们不是 76 个独立原始装配，也没有把 `unknown` 算作“成功复现”。

本机一次完整名义场景运行的接触计时（含模型准备，不含网页绘图、JSON 写盘和方向求解）：

| 场景 | 接触时间 |
|---|---:|
| 单 BL | 54 ms |
| Soma 3 | 390 ms |
| Domino 3 | 129 ms |
| 两腿承梁 | 105 ms |
| 重建桥 6 | 812 ms |
| 5 / 7 件替代体素结构 | 1.25 / 1.85 s |
| 原始 Burr 6 | 2.42 s，仍有不确定性 |

这次 6500 点运行中，单件 SOCP 最多约 11.52 ms，Fibonacci 最多约 7.82 ms；小场景通常为毫秒级。主要成本在几何接触检查，不在方向点筛选。结果含冷启动和系统负载波动，不是跨机器基准。图库后台单线程计算；重复访问同一场景/前缀使用最多 48 项的 LRU 缓存，两种方法和不同零件切换直接使用该状态结果。核心保留向量化半空间检查、稀疏 BVH、平面裁剪和已有批量方向筛选。

报告在 `benchmark_results/assembly/nine_directions`、`benchmark_results/assembly/paper2021/{nominal,raw}`。`summary.json` 是完整状态，`prefix_summary.json` 是所有前缀；每步 JSON 同时保存接触证据、来源、更改记录、两种方向结果与耗时。输出属于可再生成文件，不随源代码提交。

下一步需要原始 `alframe.stl`、两份比较场景和经确认的 Burr 装配位姿；四件 Domino 需校验顶部倾斜接触。补齐这些后，再验证论文的稳定性、可抓取性和完整序列，不能仅凭当前方向例子宣布全部实验复现。
