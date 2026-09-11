# Assemble Them All：迁移到 WRS2 的候选设计

状态：**已研究并标注，待后续实施**。2026-09-11。没有安装或运行原项目，没有迁入它的求解器，也没有改变 M2.05—07 的完成状态。

结论：方法具备迁移价值，尤其是复杂窄通道中的接触引导扩展和渐进搜索预算；建议增加可选路径提议后端，与当前几何验证器组合，而非把仿真替代几何判定。

## 阅读依据与版本

- [原论文，ACM TOG 2022](https://people.csail.mit.edu/yunsheng/Assemble-Them-All/paper.pdf)：核对 §3、§4.1—4.4、§6—7、Appendix A/B/C，以及 Fig.2—4 和预处理页的渲染。
- [作者仓库](https://github.com/yunshengtian/Assemble-Them-All)：2026-09-11 API 查询 main 为 `29db7df4d7f2e6055f3f26fc5c11a424954f7308`。下列源代码阅读为当时 main 页面；后续实现先检出并复核该版本，不默认跟随 main 更新。
- [路径与仿真封装源码](https://github.com/yunshengtian/Assemble-Them-All/blob/main/examples/run_joint_plan.py)：`PhysicsPlanner`、`BFSPlanner`、`get_xml_string`。
- [序列源码](https://github.com/yunshengtian/Assemble-Them-All/blob/main/examples/run_multi_plan.py)：`ProgressiveQueueSequencePlanner`。
- [环境文件](https://github.com/yunshengtian/Assemble-Them-All/blob/main/environment.yml)、[编译入口](https://github.com/yunshengtian/Assemble-Them-All/blob/main/simulation/setup.py)：核对运行兼容性。

## 论文与源码中的核心信息

论文 Fig.2 的关键直觉是：理想无摩擦情形下，斜轴上的环受到不完全对准轴线的力，接触仍可引导它沿轴滑动。论文忽略重力和操纵限制，采用刚体与罚力接触。Appendix A/B 还记录了栅格 SDF、非封闭/薄模型筛选、顶点加密和初始穿透容差；因此不能把其成功结果解释为任意原始 STL 的严格无穿透保证。[原文](https://people.csail.mit.edu/yunsheng/Assemble-Them-All/paper.pdf)

源码 `BFSPlanner` 尝试六个平移力方向，开放旋转后尝试十二个力/力矩方向，通过仿真扩展路径。`set_state` 会清零速度，扩展中也重复调用；`is_disassembled` 用凸包与包含检查。`PhysicsPlanner` 按初始重叠调整 collision threshold，故它的成功状态具有自己的容差语义。以上是源码事实，不应不加区分地复制为 WRS 的运动/验证契约。[路径源码](https://github.com/yunshengtian/Assemble-Them-All/blob/main/examples/run_joint_plan.py)

序列实现将待尝试零件放入队列；失败的零件增加搜索深度预算后重试，拆除成功后继续处理剩余零件。可借鉴预算分配，减少一次在受阻零件上耗尽时间。[序列源码](https://github.com/yunshengtian/Assemble-Them-All/blob/main/examples/run_multi_plan.py)

这里的“试六个力”与我们“解六个坐标目标的 LP”含义不同：前者的位移由积分和接触反力产生；后者直接解满足局部不等式的速度方向。都不意味着最终运动只能沿世界坐标轴。

## 迁移标记与落点

以下表格是本项目的设计判断，不是论文的性能保证。

| 标记 | 内容 | 可迁移性与建议落点 |
| --- | --- | --- |
| ATA-01 | 局部几何候选 + 接触引导扩展 | 高；06 增加可选 proposal backend。保留现有候选，普通直线先验证，受限通道再使用新扩展。 |
| ATA-02 | 真正的 SDF 物理 rollout | 有条件；06 的实验后端。需要质量/惯量、积分器、接触力及数值收敛处理，现有距离查询还不等于物理仿真。 |
| ATA-03 | 小预算先试、逐步加深 | 高；07 增加可配置预算调度，和现有 DFS/beam 候选策略对照。保持有限预算 unknown。 |
| ATA-04 | 明确 outside 终态 | 高；06。明确整体移出判据，与路径中每个状态/区段的几何验证分开。 |
| ATA-05 | 紧配合和旋转装配测试集 | 高；10。先选少量有代表性的环轴、弯管、槽道、旋转锁扣，保留初始几何质量与尺度记录。 |
| ATA-06 | 刚体分离路径反转 | 已有设计基础；06/07 复用路径证据，但正向支撑事件、抓取和机器人条件必须重验。 |

## 两种扩展后端，分别验收

### 方案 A：约束投影扩展，先做轻量实验

这是受到“接触引导”启发的**本项目建议**，不是 Assemble Them All 原算法的复现。

给出期望局部运动 u_des，在当前接触约束下求离它最近的可行运动：

\[
\min_u\tfrac12\|u-u_{des}\|_W^2\quad\text{s.t.}\quad Au\geq0.
\]

直观上是把“想走的方向”修正为“接触允许、又尽量接近原意图”的方向。轴孔中，斜向意图可被修正为轴向分量。W 定义平移与旋转代价，六维量继续使用现有特征长度；不能把力向量直接当成速度向量混用。

每次只作短距离积分，更新接触，再继续扩展。near 必须通过间隙与有限步长验证处理；当前接口不能凭正间隙就建立 active 等式。投影可能为零，也可能陷入局部停滞，需更换候选/搜索或明确预算耗尽。弯曲通道需要反复更新，不能用一次局部投影保证整条路。

优点是能复用 `LocalConstraints`，兼容现有 NumPy/SciPy 架构，不先引入外部动力学内核。限制是它不模拟接触力、摩擦和惯性；真实曲面 active 约束未充分覆盖时，也不能宣称投影正确。

### 方案 B：外部动力学 rollout，后续独立实验

通过适配器驱动 RedMax 或另一个支持所需 SDF 接触的动力学内核，返回对象 SE(3) 路径。这个后端才涉及物理响应，与方案 A 的数值投影分别命名、分别记录结果。

需要补齐：质量与惯量、坐标和单位转换、接触点覆盖、稳定的时间积分、速度状态/重置规则、穿透与收敛诊断。若隐式积分要求接触导数，还要定义梯度/Jacobian 的一致性；当前 `SignedDistanceField.query` 返回单位法线及误差字段，并不承诺任意阶连续导数。

先确认 Python 3.12/Windows 编译与 ABI，再选择依赖接入方式。原环境锁定 Python 3.7、NumPy 1.21.4、SciPy 1.7.3 等版本；不能整体覆盖用户现有 venv312。[环境文件](https://github.com/yunshengtian/Assemble-Them-All/blob/main/environment.yml) 编译入口通过 CMake 生成 `redmax_py`，有 Windows 分支和旧 `distutils` 导入；这意味着需要兼容性验证，不等于已验证不可用。[编译入口](https://github.com/yunshengtian/Assemble-Them-All/blob/main/simulation/setup.py)

## 与现有 SDF 的接点

WRS 已有 `SignedDistanceField.query` 的批量距离、单位法线、signed/valid 和距离误差；已有按局部几何缓存的 `Open3DMeshSDF`，以及原生 `GridSDF`。它们可用于提议后端的几何查询。

当前默认 Open3D 字段是按需网格距离查询，不是预计算体素 SDF；GridSDF 有域与误差约定。现有 `SDFCollisionChecker` 只支持 mesh-derived 字段，原生字段缺少全局 zero-set/domain 验证。因此接入栅格物理后端时，必须额外确定用于复验的几何表示与误差关系。

路径扩展每个微步只请求必要的距离/法线批次，不调用完整 contact 区域细分、裁剪和边界重建。详细接触报告只在需要的状态生成。这是可测量的工程优化方向，尚未测得新后端加速倍数。

推荐后续协议（名称为草案，尚不存在）：

```text
MotionProposalBackend.propose(scene, moving_ids, seed, budget)
    -> pose_path, backend_parameters, coverage, diagnostics

MotionValidator.validate(scene, pose_path, contact_policy, budget)
    -> validated / blocked / unknown, witnesses, validation_level
```

两步共享状态摘要，但保持不同职责。仿真说成功不能跳过几何复验；复验需覆盖离散帧之间的区段和包含情况。零件全部移出与初始局部运动可行也分别记录。

## 后续验收门槛

1. 原有解析案例方向与残差不退化；斜轴、对向面、仅有一个允许轴向的盲孔都有解释。
2. 比较直线、约束投影和物理 rollout 在同一几何、同一预算下的成功率与总时间，计入字段准备及最终验证。
3. 远处障碍、薄边相交、窄槽、中途旋转碰撞能被识别或返回 unknown；不能通过增大容差来刷成功率。
4. 保存质量/惯量、步长、力或期望速度、接触容差和随机种子；明确哪些是数值参数，哪些是物理参数。
5. 初始穿透、开放 STL、SDF 域外、法线不确定必须有显式结果，不自动改成 active 或 free。
6. 搜索状态如果继承速度，就必须包含速度；若每段重置速度，要在契约和 replay 中明确。旋转距离需使用一致的 SO(3) 表示。
7. 07 的不同姿态、支撑和剩余零件集合不得复用错误缓存；拆出动作导致不稳时仍调用 05 判断。

## 可复制的后续任务提示

```text
阅读 docs/assembly_planner/assemble-them-all-transfer.zh.md、direction-solver-explained.zh.md、
tasks/06-part-motion.zh.md，以及当前 M1/M2.04 交接。统一使用指定 venv312 解释器。
先完成 06 的路径结果协议和独立验证，再选择一个 ATA 候选做隔离实验。
从 ATA-01 的约束投影扩展起步时，明确它不是物理仿真或原论文复现。
研究 ATA-02 时先验证 Python 3.12/Windows 构建，不能覆盖现有环境。
不把仿真允许穿透参数转换成接触面积或无碰撞证明。
提供同输入、同预算的基线比较、replay、失败证据和 unknown 传播。
本提示只用于后续明确启动该任务时；当前研究轮不实施这些后端。
```
