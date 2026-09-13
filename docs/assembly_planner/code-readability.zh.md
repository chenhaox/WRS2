# Assembly planner 代码阅读指南

这次整理集中在搜索、单步评估、静力平衡和机器人执行四个文件。公开入口、结果字段和搜索策略保留；内部状态改成具名对象，长流程按物理阶段拆分。NumPy 批量计算、稀疏矩阵、CUDA 批量 LP、接触和碰撞缓存继续复用。

## 从哪里开始读

| 想了解什么 | 文件与入口 | 下一步 |
|---|---|---|
| 找到一条可行装配序列 | [sequence.py](../../wrs/assembly/sequence.py)：plan_sequence | SequenceEvaluator.evaluate，然后 replay_sequence |
| 按 S/G/A 评分搜索序列 | [quality_search.py](../../wrs/assembly/quality_search.py)：plan_quality_sequence | _AssemblyQualityEvaluator.expand，然后 _QualityDepthFirstSearch.visit |
| 单次静力平衡 | [stability.py](../../wrs/assembly/stability.py)：check_equilibrium | _collect_force_sites → _assemble_force_model → _solve_load_case |
| 六维扰动承载极限 | [stability_sweep.py](../../wrs/assembly/stability_sweep.py)：DirectionalStabilityAnalyzer | 复用力模型，再调用 [_batched_lp.py](../../wrs/assembly/_batched_lp.py) |
| WRS 机器人执行 | [execution.py](../../wrs/assembly/execution.py)：validate_execution | _ExecutionSession.run → _execute_step → replay_execution |
| 运行与查看结果 | [WRS 例子](../../examples/assembly/README.md) | sequence.py、quality_sequence.py、robot_execution.py |

建议按“公开入口 → 主流程 → 当前需要的物理检查”阅读。配置和不可变结果定义在各文件顶部；下划线开头的类、函数是内部实现，不需要在例子里手工创建。

## 搜索和物理评估各负责什么

质量搜索有两个独立角色：

- **_QualityDepthFirstSearch**：决定先探索哪个零件，维护序列前缀和当前最好解，检查预算，用质量上界剪枝。
- **_AssemblyQualityEvaluator**：检查这一步能否实际装入；持有接触、稳定性、支撑和转换缓存。

一次扩展按这个顺序读：

~~~text
expand(context, part_id)
  构建装入后的状态
  _qualified_grasps          → G：还有多少合格抓取
  _directions_and_score      → 方向与 A
  _stability_without_support→ S：无辅助支撑时的承载能力
  _support_subsets           → 必要时选择有限辅助支撑
  _insertion_step            → 验证支撑交接和有限装配路径
  yield QualityTransition
~~~

_AssemblyContext 只有“已装零件状态”和“当前辅助支撑 ID”。搜索前缀的最小 S/G/A 和辅助步骤数属于 DFS。

**物理结果可以按状态缓存，质量搜索不能简单地用“访问过这个状态”跳过前缀。** 相同已装零件集合可能由不同顺序到达，前缀评分不同。现有安全上界仍在 quality_upper_bound 中，未替换成当前前缀分数。

expand 保留生成器：找到一个可用支撑方案后马上进入下一层，后面的兄弟方案按需计算。反向可行序列搜索也用 _RemovalSearchNode 保存尚未尝试的兄弟，遇到死路后恢复。它只求第一条可行解，可以使用原有的状态去重；它与质量搜索的目的不同。

## 安装前后与拆卸前后

SequenceStep 保留原来的**拆卸存储格式**。现在可用安装侧属性直接读取：

| 安装侧属性 | 对应原字段 | 含义 |
|---|---|---|
| assembly_before | after | 安装前已在装配体中的零件；不含正在搬运的零件 |
| assembly_after | before | 新零件到达目标位姿后的装配体 |
| assembly_supports_before | supports_after | 安装前的辅助支撑 |
| assembly_supports_after | supports_before | 安装后的辅助支撑 |
| assembly_poses | 反转 removal.poses | 从外部位置走向目标位姿 |
| assembly_events | 反转事件顺序并反转动作 | 获取支撑、插入、交接、释放 |

这些属性直接引用已有状态，不复制网格或位姿数组。

~~~python
step, failure = evaluator.evaluate_insertion(
    assembled_state,  # 包含新零件最终位姿
    part_id,
    supports_before=current_support_ids,
    supports_after=chosen_support_ids,
    removal_directions=outward_directions,
)

if step is not None:
    installed_state = step.assembly_after
    active_support_ids = step.assembly_supports_after
~~~

removal_directions 仍明确指向**装配体外部**；安装反向使用该路径。原 evaluate(state, active, part_id, ...) 保持拆卸语义，已有自定义评估器仍能使用。evaluate_removal 是显式的拆卸命名。

## 静力方程怎样组织

1. _collect_force_sites：接收合格 active 接触，清理／约简受力点，记录法线、摩擦和每个接触面的总力容量。
2. _assemble_force_model：一次构建稀疏矩阵，所有载荷共用。
3. _solve_load_case：改变重力／外载荷右端，求解非负摩擦锥权重，随后用物理单位复核残差。
4. _equilibrium_result：整理力、受力点、假设和诊断，不参与求解。

_ForceSites 保存点、面级容量和点约简证据；_ForceModel 保存矩阵和变量到力点的映射。具名上下文只在每次状态求解时创建；射线和矩阵仍使用原来的数组／稀疏存储。

每个自由物体占六行：前三行力平衡，后三行关于 COM 的力矩平衡。一个接触共用同一组射线权重，对 A 取负、对 B 取正，因此作用与反作用由变量共享直接保证。力矩行除以特征长度 L 改善求解尺度；最终仍分别用 N 和 N·m 检查残差。容量限制作用于整张接触面／辅助支撑的权重之和。

曲面子集失败时，check_equilibrium 仍回退到完整曲面力点重算所有载荷；返回的点和力始终对应最后使用的公共模型。此处未修改接触分类、力点约简、摩擦近似或 GPU 求解算法。

## 机器人执行怎样组织

validate_execution 只创建一次 _ExecutionSession 并调用 run。会变化的机器人、目标零件、轨迹、辅助支撑、抓取缓存都属于这个 session，不再依赖一个巨大函数中的隐式闭包变量。

_execute_step 逐个执行 M2 的装配事件：

~~~text
take_from_staging → _pick_from_staging
                     _acquire
                       _grasp_has_capacity
                       _approach_grasp
insert_part       → _insert_part
                     抬起 → 高处转移 → 插入 → 舍入误差内就位
acquire_auxiliary → _acquire(..., support=...)
release_part      → 恢复主手附件状态 → _release
release_auxiliary → _release(..., auxiliary=True)
步骤结束          → _verify_released_assembly
~~~

_GraspAttachment 明确保存主手的抓取、接触点、受力点、TCP 和附着变换，在辅助臂接管后恢复主手上下文。机器人规划、稠密碰撞检查、夹爪开闭检查仍走原来的 WRS 调用。

最后仍由独立的 replay_execution 复核轨迹。没有把生成器自己的判断当作回放证据。run 的 finally 恢复随机数状态、机器人碰撞设置和工作区初始位姿，失败和异常退出同样执行。

## 后续修改时遵循的约定

- 主流程使用具体名字，例如 removal_state、active_support_ids、pending_parts；F、T、L 等公式变量可以保留数学记号并注明单位。
- 新物理检查放进对应评估阶段；遍历器只处理转换、分数、预算和结果。
- 保留状态摘要与配置绑定，未知结果继续标为 unknown。有限搜索失败不能改成全局无解。
- 在“每个状态／每个执行阶段”创建少量具名上下文；点、三角形、射线、批量载荷仍使用数组。
- 不把数组向量化改成逐元素对象调用；不提前计算全部兄弟；不通过降低采样数、求解预算或回放精度换计时。
- 调整接触／稳定性／抓取规则时检查缓存键是否包含相应配置。整理命名不应清空更多缓存。
- 修改后同时比较物理结果、搜索工作量和耗时。只看一条最终序列或一次时间不足以判断等价。

## 回归与性能对照

指定解释器：

~~~powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' -m unittest discover -s tests/assembly
& 'D:\code\venv312\.venv\Scripts\python.exe' benchmarks/assembly/readability.py
~~~

新增 [test_readability_contracts.py](../../tests/assembly/test_readability_contracts.py) 覆盖：惰性 DFS、完整候选回放拒绝后继续搜索、安装侧支撑交接与缓存复用、机器人异常后的状态恢复。原来的物理方程、搜索、点约简、GPU、单／双臂测试继续保留。

2026-09-13：完整回归 181 项通过（132.270 s）。另外检查了原有函数签名和 18 个带注解的数据类型字段：均保持一致；15 个未调整职责的函数，包括两个独立回放器，去除格式差异后的 AST 与基线相同。新增拆卸命名也覆盖了只接受三个位置参数的旧评估器。

[readability.py](../../benchmarks/assembly/readability.py) 的 LABEL 控制报告名称，REPEATS 控制重复次数，ROBOT 控制机器人验证。报告写到仓库根目录 benchmark_results/assembly/readability，普通 examples 不生成 output。使用 before 保存整理前基线，再用 after 保存整理后结果；文件存在时自动比较结果摘要。

计时包含相同物理输入和求解预算下的新评估器，不包含 Python 导入。普通搜索取 3 次中位数，单次平衡取 15 次中位数，机器人单／双臂各 1 次；首次 CUDA 初始化单列在原始次数里，不把它当成稳定运行耗时。下面的数字仅说明这组固定场景的结果，不能将运行波动解释为算法加速。

| 固定场景 | 整理前（s） | 整理后（s） |
|---|---:|---:|
| 桥梁静力平衡 | 0.0130 | 0.0088 |
| 堆叠可行序列 | 0.3592 | 0.2748 |
| 九零件 gantry 可行序列 | 3.4269 | 2.6654 |
| 配重 S/G/A 搜索（CUDA，300 方向） | 2.6233 | 1.9577 |
| 桥梁 S/G/A 搜索（CUDA，300 方向） | 4.0265 | 3.0530 |
| 单臂 M2 + M3 | 8.8573 | 6.1182 |
| 双臂 M2 + M3 | 37.5350 | 26.4780 |

整理前基线为 158a60f，2026-09-13 同一指定解释器下测量。七组结果摘要全部一致，包括序列、支撑切换、评分、展开／剪枝次数、缓存规模，以及机器人事件顺序、轨迹帧数和碰撞查询次数。单臂保持 670 帧／7,605 次查询，双臂保持 1,156 帧／52,811 次查询。这组输入未观察到性能退化；以上时间下降不作为算法加速结论。
