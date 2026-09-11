# 稳定性可视化与计算核查

2026-09-11。解释器：`D:\code\venv312\.venv\Scripts\python.exe`。

## 查看例子

在 WRS2 根目录运行：

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/stability_demo.py --case bridge
```

打开 `http://127.0.0.1:8893/`。左下切换案例；右侧选择要观察的自由零件和载荷工况。零件透明度、接触面、求解点、摩擦锥和力箭头分别控制。

| 案例 | 物理问题 | 结果 |
| --- | --- | --- |
| stack | 两层 1 kg + 2 kg 堆叠 | 可行，桌面承载 29.43 N |
| floating | 同样堆叠但无桌面 | 无支撑不可行；两个各限 20 N 的声明支撑可行，单个不足 |
| bridge | 两个自由桥墩、横梁、偏心载荷 | 名义及 1 N 侧推可行；30 N 侧推不可行 |
| overhang | 梁与载荷偏向右侧 | 合成质心 x=0.1575 m，超出桥墩接触边界 x=0.11 m，不可行 |
| incline | 20° 斜面，μ=0.6 | 可行 |
| slippery | 同一斜面，μ=0.2 | μ 小于 tan(20°)≈0.364，不可行 |
| tripod | 三个分离支撑区、平台和圆柱载荷 | 名义及 0.1 N·m 扭矩可行；1 N·m 扭矩不可行 |

这些接触面来自当前 `analyze_contacts`，没有为演示虚构接触点。模型尺寸、质量、局部质心、摩擦在 [_stability_cases.py](../../examples/assembly/_stability_cases.py) 中显式给出。辅助支撑是声明的有限施力能力，尚不代表机器人的可执行抓取。

## 图中内容与求解器一致吗

- 黄点来自 `EquilibriumResult.force_sites`，即实际进入力平衡矩阵的接触/辅助作用点。包含解中力为零的点；不同接触条目可能在空间重合。
- 蓝色是 **实际 16 边内接摩擦锥**。顶点在力作用点，生成方向直接来自求解器的 `rays_on_b_world`。每条射线的法向分量为 1，切向分量长度为 μ。绘制高度固定 25 mm，半角为 `atan(μ)`；高度不代表承载上限，也不是零件中的空间障碍物。
- 法线与射线约定为“作用在 B 上的力”；A 上为反向。选择单个零件时只画该零件的锥和反力。选择全部自由零件时内部接触两侧都画，可能出现共顶点的两个反向锥。
- 绿色箭头是该工况求解得到的接触/辅助力；红色是重力，紫色是额外外力，比例 3 mm/N。外力矩关于该体 COM、用世界 XYZ 分量显示在面板中。
- 不可行工况保留黄色候选力点和蓝锥，但不显示伪造的反力解。求解结果可能不唯一，零力点没有绿色箭头是正常现象；箭头不应解释为测得的唯一压力分布。
- `feasible` 是当前工况名义平衡，`failed_tested_set` 表示所列扰动中至少有一个失败；两个字段并不矛盾。

```python
result = check_equilibrium(assembly, state, graph)
for site in result.force_sites:
    print(site['part_a'], site['part_b'], site['point_world_m'])
    print(site['normal_on_b_world'], site['rays_on_b_world'])
    print(site['friction'], site['capacity_group'], site['max_group_normal_force_n'])

# 可行解通过 site_index 回指同一套力点。作用于 A/B 的力已成对保留。
if result.nominal is not None and result.nominal.status == 'feasible':
    for force in result.nominal.contact_forces:
        site = result.force_sites[force['site_index']]
```

结果中的数组和字典不可修改。缺少质量/质心等必要信息时不会虚构完整的力模型。

## 对照公开实现后的结论

现有模型寻找每个自由体满足以下约束的一组接触力：

```text
Σ f + mg + f_external = 0
Σ (p - COM) × f + τ_external = 0
f_n >= 0, ||f_t|| <= μ f_n
```

内部接触共享变量，在同一个世界作用点对相邻两体施加等大反向力；每个点不额外提供自由力矩。生产实现把圆锥换成内接 16 边锥，以非负射线权重求 LP。

[pymanoid 的 Contact 实现](https://github.com/stephane-caron/pymanoid/blob/master/pymanoid/contact.py) 使用接触顶点、摩擦锥射线的非负组合，并以位置叉乘力构造接触 wrench。其默认四边锥也是内接近似。我们核对了 `force_span`、`wrench_rays` 和 `wrench_span` 所体现的建模方式；没有复制其源码或安装 OpenRAVE。

[Drake StaticFrictionConeConstraint](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_static_friction_cone_constraint.html) 定义了圆锥 Coulomb 条件。[Drake StaticEquilibriumProblem](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_static_equilibrium_problem.html) 还同时处理位姿、非穿透和接触互补条件。我们目前固定已给定位姿和接触模式，所以只核查该状态下的受力可行性，不等同于 Drake 的完整位姿优化问题。

本次没有发现已测试范围内的力平衡符号、力矩臂、内力配对或有限支撑容量错误。新增独立参考求解位于 [test_stability_reference.py](../../tests/assembly/test_stability_reference.py)：每个点直接用三个笛卡尔力分量作为变量，使用 Clarabel 的精确圆锥约束，不调用生产 LP 的射线生成和矩阵构造函数。它读取同一套公开力点，独立构造平衡矩阵，所以验证的是固定接触力模型，不独立证明接触提取正确，也不是执行 Drake 库本身。

检查包含：

1. 七种场景的名义工况和六个扰动工况，与解析预期及圆锥参考解对照。
2. 辅助支撑单个/两个的能力上限，及每个 patch 的共享容量语义。
3. 用输出力重新计算各体关于世界原点的力矩，区别于生产矩阵关于 COM 的力矩行；检查等大反向力、非负法向力、圆锥不等式及物理残差。
4. 保守性边界回归：载荷方向位于两条摩擦锥射线之间，大小为精确摩擦极限的 99%；16 边 LP 不可行，精确圆锥和 64 边 LP 可行。这是预期差别。16 边锥最坏径向比例 `cos(pi/16)≈0.980785`，不应把它的失败说成真实系统必定失稳。

这里的“稳定性”仍是静力可行性：没有模拟弹性压力分布、滑移后的运动或被动接触力如何建立。[Haas-Heger 等的被动稳定性研究](https://arxiv.org/abs/1806.01384) 进一步处理粘着/滑动与被动响应，说明它需要额外的接触行为模型。当前结果不能替代这类检查。

## 透明度修复与计算耗时

失效原因是流式场景只跟踪新增/删除和位姿，Python 的 alpha 改变没有传给浏览器。现在 RenderModel 对材质维护轻量版本号，发布器仅发送变化的模型描述，浏览器更新已有颜色缓冲和透明/不透明渲染分组。材质更新不重传网格、不重建几何 GPU 缓冲，刷新后 hub 回放最新材质。滑条范围修正为 0～1，遵循现有 WRS UI 的“松开提交”行为。

摩擦锥按数组构造并合并绘制，不为每个锥的每条射线创建独立场景对象。显示开关只增删现有图层，不触发接触提取或静力 LP。

指定解释器，31 次预热后独立调用的完整 API 墙钟计时（含返回数据冻结，不含几何/接触提取和绘图）：

| 案例 | 力点条目 | 工况数 | 中位数 | P95 |
| --- | ---: | ---: | ---: | ---: |
| stack | 8 | 1 | 3.66 ms | 3.99 ms |
| floating，含两个有限支撑 | 6 | 1 | 3.44 ms | 3.96 ms |
| bridge | 52 | 3 | 25.18 ms | 26.56 ms |
| overhang | 44 | 3 | 17.02 ms | 18.11 ms |
| incline | 8 | 1 | 3.55 ms | 4.19 ms |
| slippery | 8 | 1 | 2.82 ms | 3.34 ms |
| tripod | 72 | 3 | 33.03 ms | 34.43 ms |

三个工况是名义 + 两个扰动，不是单个 LP 的耗时。floating 此处直接传入两个声明支撑，不含子集搜索。逐次数据保存在 `examples/assembly/output/stability/cases-benchmark.json`，也记录单次场景创建/接触提取时间。

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/stability_demo.py --all
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/benchmark_stability_cases.py --repeat 31
& 'D:\code\venv312\.venv\Scripts\python.exe' -m unittest discover -s tests/assembly -p 'test_*.py'
& 'D:\code\venv312\.venv\Scripts\python.exe' -m unittest discover -s tests/viewer -p 'test_*.py'
```

回归结果：装配 103 项通过（17.147 s）；材质同步/回放 2 项通过。浏览器实际检查了摩擦锥、受力点、单体筛选、工况切换以及不透明度 0/1 和刷新保持。截图在 `examples/assembly/output/stability-*.png`。三个此前按用户要求采用主分支的 API 索引没有重新生成。
