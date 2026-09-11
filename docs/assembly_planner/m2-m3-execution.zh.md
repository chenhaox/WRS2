# M2 有限路径、序列与 M3 机器人执行交接

2026-09-11；分支 `codex/assembly-planner`。06、07 和 08 的模拟执行基线已实现。使用 `D:\code\venv312\.venv\Scripts\python.exe`，未安装新依赖。

后续增加了 M2 的真实离开距离/路径移动距离控件、九零件框架和轴套例子，优化了 DFS 及网格距离查询，并新增独立多方向 GPU 稳定性接口。见[最新用法与计时](stability-sweep-and-sequence-ui.zh.md)；[与旧版的功能差异](old-new-feature-audit.zh.md)。本页较早的性能数据保留为基线。

## 直接运行

在 WRS2 根目录执行：

```powershell
$assemblyPython = 'D:\code\venv312\.venv\Scripts\python.exe'
# M2：零件路径与最终接触面；滑条和播放按钮
& $assemblyPython examples/assembly/sequence_demo.py
& $assemblyPython examples/assembly/sequence_demo.py --case floating --method beam
# M3：真实 RS007L + OR2FG7；单臂取放、插入和释放
& $assemblyPython examples/assembly/robot_execution_demo.py
# 双臂：辅助臂接管悬空下层零件，主臂继续安装上层
& $assemblyPython examples/assembly/robot_execution_demo.py --auxiliary --port 8896
```

M2 默认端口 8894；单臂默认端口 8895。加 `--headless` 只计算并写 JSON。接触面叠加显示的是**最终装配态的参考位置**，机器人与零件随轨迹帧运动；透明度仅影响显示。回放速度不表示实际机器人速度。

例子只有显式几何、源位姿、机器人和能力参数。OR2FG7 的每指 30 N、摩擦系数 0.5 是**示例声明值**，没有把它们当作厂家额定值或实测值。

```powershell
# manifest 必须带单位和质量/质心/摩擦；本入口默认不增加辅助支撑
& $assemblyPython examples/assembly/plan_assembly.py path/to/assembly.json --method dfs
# 生成基准和可用于上条命令的 stack_manifest.json
& $assemblyPython examples/assembly/benchmark_execution.py --auxiliary
& $assemblyPython examples/assembly/plan_assembly.py examples/assembly/output/execution/stack_manifest.json
```

报告位于 `examples/assembly/output/sequence/` 和 `output/execution/`，输出可重建，不纳入 Git。结果带 schema version、输入摘要、状态、路径、事件、验证级别及失败原因。

## M2：计算什么

`part_motion.py` 提供 `plan_removal`、`validate_object_path` 和 `ContactPolicy`：

- 先使用 SOCP 最优方向和确定性坐标轴，再尝试有预算的平移绕行、旋转候选。旋转用有效的四元数插值；不保证穷尽 SE(3)。
- outside 终点超过其他零件在所选方向上的几何投影边界，再加 20 mm 默认余量；没有固定旧行程。
- 对**整段**路径使用扫掠 AABB、支撑平面，或者“中点真实网格距离下界减去整段最大点位移”。不能证明时自适应细分；达到预算或最小步长仍不能证明则返回 unknown。
- 平移中的合法初始平面接触使用具名 surface pair：两件完整网格必须处于平面两侧，所有零间隙支撑三角形都属于该 surface。不能用配合关系将整个零件 exclude。
- 有间隙的孔使用原始非凸 mesh；零间隙曲面滑动尚无通用连续接触认证，不能因候选方向存在就宣布路径成功。

默认自由区净空 0.2 mm，长度/角度输出步长 10 mm / 0.05 rad；验证另外有节点、三角形、时间和最小区间预算。`bounded_nominal_mesh` 指输入名义三角网格的区间证据，不是对真实制造误差的保证。预算为合作式检查，单次后端查询可能超过剩余墙钟预算。

`sequence.py` 提供确定性 DFS / beam：

1. 状态保存各件位姿、当前存在集合、辅助支撑和资源占用。
2. 拆除前当前装配必须平衡；转移期间剩余装配独立平衡，主搬运资源以声明的有限力/力矩承载移动件。移动件与其余件的接触力可取零，构成保守的全过程准静态证据。
3. 接管新支撑后才释放旧支撑；两者交接时的资源并集也不能超限。主臂不能同时冒充辅助臂。
4. 反转拆卸路径和支撑事件，再独立复验正向几何、平衡、状态链与最终目标。

固定整体装配朝向、单件刚体是当前搜索范围。`success` 不代表全局最优；有限搜索失败是 unknown/exhausted。M2 始终保留 `execution_validated=False`，主资源在 COM 的有限 wrench 和抽象 staging 交接交给 M3 细化。

```python
from wrs.assembly import plan_sequence, replay_sequence, SequenceConfig

plan = plan_sequence(assembly, config=SequenceConfig(method='dfs'))
if plan.status == 'success':
    print([s.part_id for s in plan.assembly_steps])
    print(replay_sequence(assembly, plan))
```

## M3：哪些证据来自实际机器人

`ExecutionWorkcell` 显式提供真实源位姿、主/辅助 WRS Arm 和有限夹持能力。源位姿独立于 M2 的 outside 终点；不能凭空把拆除终点当成可抓取台面。

执行使用现有 `Grasp`、`reason_common_gids`、`Arm.moveto`、`Arm.insert` 和 `Workcell.activate`。主流程：到达预抓取 → 接近 → 逐步闭合 → 抬升 → 自由转移 → 插入 → 必要的辅助臂接管 → 松手及退让。每阶段返回实际关节轨迹、夹爪配置、对象位姿和具名事件。当前使用显式事件循环组织原语，未增加新的通用 Recipe 类型。

真实模型核查修正了 OR2FG7 指垫与名义开口的偏移：25 mm 箱体的闭合命令约 24 mm。程序从两侧实际指垫三角形测量并复核，示例残差约 `1.86e-10 m`；不是仅修改一个能让例子通过的宽度常数。

夹持受力点来自实际指垫与零件的平面交集，经已有 hull 约简，每指的多个点**共用一个有限法向力上限**。使用 16 棱内接摩擦锥 LP，复核重力、声明扰动和辅助支撑需要覆盖的有限力集合。两侧各一个理想点会丢失实际指垫传递扭矩的能力，故不以这种简化伪造夹持成功。

所有机器人、夹爪及零件都加入 MuJoCo 碰撞场景。移动目标额外进行原始网格验证；指垫只能接触声明的目标表面，夹爪根部和其它面仍受检查。MuJoCo 对凹形目标产生的凸代理假阳性由完整目标网格复核处理。**机器人本体仍使用 WRS 自身碰撞模型和静止位姿 ACM**；不声称替换了所有机器人的碰撞几何。

最后在新建碰撞世界中独立回放，复核：工作站/机器人基座绑定、关节和夹爪界限、轨迹连续性、TCP 持物变换、辅助臂确实位于夹持位置、全过程碰撞、实际抓取承载、放手平衡、最终目标及支撑状态。任一步未通过，都不能设置 `execution_validated=True`。

验证级别为 **`sampled_joint_nominal_mesh`**：默认关节步距 0.015 rad；这是有声明分辨率的运动学检查，尚非机器人连续碰撞检测、动力学仿真或实机控制。WRS 位姿使用 float32，最终落座和指垫平面投影限定在显式 2 μm 数值舍入范围，并记录残差；不是把普通 SDF near 带升级成承载接触。

## 性能与加速

同一指定 Python 环境，Windows 11、Intel Family 6 Model 198；NumPy 1.26.4、SciPy 1.16.2、MuJoCo 3.5.0。原始记录：[基准 JSON](assets/m2-m3-benchmark.json)。M2 每例 3 次，表中为中位数；M3 为各一次完整规划加独立复验，不含导出和 viewer 启动。

| 计算 | 新建分析器 | 复用分析器 | 另行独立 replay |
| --- | ---: | ---: | ---: |
| stack，36 三角形、2 次展开 | 119.91 ms | 10.12 ms | 109.34 ms |
| bridge，60 三角形、4 次展开 | 587.63 ms | 32.59 ms | 276.02 ms |
| floating，24 三角形、2 次展开 | 80.57 ms | 8.75 ms | 69.17 ms |

| 机器人例子 | 完整时间 | 回放帧 | 规划/记录碰撞查询 | 静止目标精确缓存命中 |
| --- | ---: | ---: | ---: | ---: |
| 单 RS007L | 7.28 s | 670 | 7,605 | 6,830 |
| 双 RS007L | 33.12 s | 1,156 | 52,811 | 51,917 |

查询列不含独立回放新增的 670 / 1,156 次。运行条件会影响秒数，未用这些小例子推断大型 STL 吞吐率。

已做的加速：实例持有的真实状态 contact/equilibrium 缓存；预计算局部法线；分块向量化几何投影并限制廉价排除阶段的轴数，避免生成大型 V×F 数组；先用 AABB/支撑平面排除再调用 BVH；机器人搜索中复用位姿、几何和接触策略完全相同的静止目标与静止夹爪验证。

双臂同轨迹在增加静态精确缓存前约 52 s、之后约 28–33 s。关节近似缓存在 execution 中关闭，不能把这种加速误解为三位小数取整。后续主要优化机会是 WRS IK/RRT 查询数和持物网格复核，而非继续降低精度。

## 回归与 WRS 兼容性

```powershell
& $assemblyPython -m unittest discover -s tests/assembly -p 'test_*.py'
& $assemblyPython -m unittest discover -s tests/viewer -p 'test_*.py'
```

新增回归覆盖：薄障碍穿越、旋转中间碰撞、完全包含、真实孔、封闭笼、平面切向滑动/向内阻挡、预算、完整序列与错误事件、有限资源、实际单/双臂运行、夹持不足、不可达、错误面穿入、轨迹缺段、机器人基座改变及 release 事件遗漏。M2 另在禁止 MuJoCo/Open3D/wgpu/robot/viewer/grasp 导入的独立进程中完成实际规划。

最终完整运行：**139 项 assembly 测试通过（73.24 s），2 项 viewer 回归通过（0.10 s）**。未将可选能力的未测项计作通过。浏览器实际操作检查了播放/暂停、末帧定位、接触面控制和零件透明度。

WRS 公共变更仅三处：

- `PlanningContext(cache_size=0, cache_decimals=None)` 支持禁用/精确缓存，默认仍为原来的三位小数；Arm 对应参数默认行为保持兼容。
- 无文件路径的内存 MeshCollisionShape 生成实际 inline mesh asset，且不同几何不再共用 `None` key。真实 MuJoCo 编译回归验证两个不同大小模型。
- 新增 SceneObject adapter，合并所有 visual 并应用局部变换；更新实例 pose 不修改共享 geometry。

## 后续范围

任务 09 CAD/B-Rep、旧 domino/burr 的物理参数核验与完整迁移、ATA 物理 rollout、成组拆装、螺纹/弹性/压配、自由曲面指垫接触、自动换抓及全局任务最优性仍未实现。本次交付完成的是 M2 基线和可独立验证的 M3 单/双臂模拟执行。抓取或机器人路径失败当前返回结构化证据；尚未自动触发跨序列的机器人约束回溯。
