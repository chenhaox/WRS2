# 04：Contact graph 与装配运动约束

状态：待实施。前置：02；完整曲面和穿透验收需 03。

## 目标和修改范围

把 contact analysis 变成后续规划可使用的局部运动约束，同时保留 contact、near、mating 和 interference 的区别。

负责 `contact/graph.py`、`constraints.py`、串行整合 `contact/analysis.py`，以及 `tests/assembly/test_contact_graph.py`、`test_constraints.py`。

## 实施步骤

1. 以零件实例和支撑为节点，每条边保存全部 patch、mating 和诊断。多个实例共享 mesh 不能合并成同一节点。
2. 将分析结果绑定明确 AssemblyState；实现几何/相对位姿/世界位姿和配置相关的缓存规则，避免旧版全局 lookup table。
3. 从每个 active 接触点的外法线构建单侧 twist 约束。单位和参考点严格遵循 contracts；对近接触使用有限 gap 或激活规则，不能当作当前固定约束。
4. 纯平移先求可行方向锥和候选边界方向；六维模式使用线性约束加归一化/采样/优化生成候选，防止零向量被当成可行运动。纯平移与角速度的尺度通过 characteristic length 处理。
5. 圆柱面的约束使用法线场，圆柱轴只作为候选。上下相反约束的切向滑动、角落方向和局部锁死应可解释。
6. 候选不是有限可行路径；给每个候选附约束残差和适用的局部 contact mode。不可判定的交集不包装为全局锁死。

## 验收

- 单面接触允许分离和切向滑动，拒绝向实体内部运动；交换 A/B 符号一致。
- 两个对向面约束允许正确的切向移动，不能因平均法线为零产生任意方向。
- 改变 twist 参考点但描述同一刚体运动时判定相同；已知 box/corner 场景验证转动项符号。
- 正间隙轴孔保持 mating/near，当前稳定性可用的 active 集合不被污染。
- 同一状态重复结果确定；移动一个对象、改变公差或几何后缓存失效。
- 任务 02 与 03 的结果以同一 graph 协议消费；存在 interference/unknown 时暴露给调用者。

## 可复制提示词

```text
在 D:\code\ch\asp\WRS2 实施 docs/assembly_planner/tasks/04-contact-graph.zh.md。
统一使用 D:\code\venv312\.venv\Scripts\python.exe；安装依赖用该解释器的 -m pip，先核对 sys.executable，不另建虚拟环境。
先读 README.zh.md、contracts.zh.md、02/03 的实际交接。建立实例级 contact graph 和基于接触法线场的平移/twist 候选。
严格使用 contracts 的法线方向、twist 参考点、gap 与结果语义；不要平均法线，不要把 mating 或 near 当成 active 承载接触。
保持局部约束与有限运动验证的边界，带状态/几何/公差的缓存必须可验证。完成对应解析与缓存验收后记录交接。
```

## 交接记录

待填：graph API、候选格式、参考点/归一化约定、缓存摘要、测试与基线。当前未实施。
