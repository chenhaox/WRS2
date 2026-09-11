# 共同接口约定 v1（M1 已落地，后续规划接口仍为设计）

本文件约束不同对话的模块边界。输入 schema 为 `wrs.assembly/1`，分析输出为 `wrs.assembly.contact/1`。M1 的实际类型以 `wrs/assembly/model.py` 和 [M1 交接](m1.zh.md) 为准。下表和包布局仍包含 04—10 的未来接口，不代表全部符号都已经存在。

多后端扩展已落地，见 [ContactModel / ContactAnalyzer / SDF 协议](contact-backends.zh.md)。新后端实现高层 `ContactBackend`，不受下面旧 `ProximityBackend` 的 BVH 类型约束。原生数据放在 `ContactModel.representations` 中；当前 JSON v1 不序列化这些运行时字段。SDF 输出 `near_band` / `estimated`，active、点/线接触尚未求解，不能将空 active 列表理解成没有真实接触。

2026-09-11 补充：独立距离证据可标记 `pair_diagnostics.active_area={status:known_zero, area_m2:0, reason:...}`；否则为 `not_solved` / null。新增 `SDFCollisionChecker`，返回独立的 `wrs.assembly.sdf_collision/1` 字典与 `separated/penetrating/unknown` 状态，不生成 ContactPatch，不做 near/法向过滤，不调用 mesh 碰撞兜底；支持 mesh-derived 字段，原生场暂显式拒绝。数据项、误差与提前退出语义见 [SDF 碰撞与批量化](sdf-collision-and-vectorization.zh.md)。

M1 实际边界：`SurfacePatch.kind` 为 `plane/general`，`face_ids` 索引 PreparedMesh，原面映射在 `PreparedMesh.source_face_ids`；`Part` 当前用 `part_id` 显示名字、`fixed` 标记固定支撑、`friction` 表示摩擦；`AssemblyState` 当前只有显式 `poses` 与 `world_revision`。夹持、资源、ContactGraph、MotionConfig、StabilityConfig、规划/执行结果留给后续任务。有限开放支撑用显式 `orientation='trusted'` 的 mesh 输入。

## 包边界

```text
wrs/assembly/
  __init__.py             # 轻量导出，不启动窗口/物理/服务
  model.py                # 共享数据类型、结果与配置
  io.py                   # assembly manifest、单位和版本
  primitives.py           # 已实现：确定性解析案例的 mesh 生成器
  adapters/
    legacy.py             # 旧 JSON + STL 输入转换
    wrs_scene.py          # 待任务 08：SceneObject 输入与分析几何的桥梁
  geometry/
    preprocess.py         # 清理、拓扑、法线与面 ID 映射
    surfaces.py           # SurfacePatch 分割与参数
    planar.py             # 平面投影裁剪与二维拓扑
    proximity.py          # 距离协议
    mesh_bvh.py           # 纯 CPU mesh 加速结构
  contact/
    planar.py             # 平面 contact region
    mesh.py               # 曲面/通用网格后端
    analysis.py           # 后端调度与区域语义
    graph.py              # 零件间关系
    cad.py                # 可选 CAD 后端
  constraints.py          # 接触运动学，候选方向/twist
  stability.py            # 静力平衡与扰动检验
  part_motion.py          # 零件 SE(3) 路径及有限运动检查
  sequence.py             # 任务状态、动作和序列搜索
  execution.py            # WRS 机器人执行验证
  visualization.py       # 已实现：HTML 与可选 WRS 输出场景，不作可行性判断
```

几何/contact/constraints/stability/sequence 核心不依赖 `base`、Panda3D、viewer 或 MuJoCo。通用几何原语在此验证稳定后再考虑上移到 `wrs.geom`，不要在第一步重写 WRS 其他调用者依赖的函数。

核心仅使用 NumPy、SciPy、Python 标准库。CAD 等扩展按后端注册和可选依赖隔离。测试优先使用标准库 `unittest`，避免仅为测试框架扩大依赖。

## 数据与单位

所有数组在构建时复制并设为只读，或采用同等不可变方案；`frozen=True` 并不能阻止 NumPy 数组内容被修改。对外结果不能暴露可改变共享缓存的数组。

| 类型 | 必需字段 / 含义 |
| --- | --- |
| `MeshData` | `vertices (N,3) float64`、`faces (F,3) integer`、`geometry_id`、单位米；预处理后保留 `source_face_ids` 的一对多映射 |
| `Part` | `part_id` 唯一实例 ID、`geometry`、`assembled_tf (4,4)`、`mass_kg` 可空、`com_local_m (3,)` 可空、材料参数；名字与 ID 分开 |
| `Assembly` | parts、固定支撑、`gravity_world_m_s2`、可选 mating/fixture 元数据、输入版本与几何摘要 |
| `AssemblyState` | `present_part_ids`、各实例当前 tf、`world_revision`、hold/fixture 状态与资源占用；机器人阶段另有配置/抓取/占用证据 |
| `SurfacePatch` | 几何内 patch ID、source face IDs、`kind=plane/cylinder/sphere/cone/general`、局部解析参数或样本、边界组件、残差/拓扑诊断 |
| `ContactPatch` | A/B 实例和 surface IDs、维度、双侧几何证据、法线场、区域/面积或线/点、间隙区间、分类、质量与后端来源 |
| `MatingRelation` | A/B、`kind`、关联 surface IDs、轴/设计间隙/装配模式、来源 `user/cad_metadata/inferred`；推断置信信息与物理接触独立 |
| `ContactAnalysis` | 当前状态摘要、patch 列表、mating 列表、pair diagnostics、未解析区域、覆盖/精度证据 |
| `ContactGraph` | 节点为 part/support，边持有多块 patch 和独立 mating；保留状态摘要 |

单位：m、rad、kg、N、N·m、m²。内置旋转采用右手系；矩阵左乘列向量；点变换 `p_world = R @ p_local + t`。`assembled_tf` 是零件局部到世界的变换。`SceneObject` 多 visual 导入时合并 `sobj.tf @ visual.loc_tf`；分析输入不能拿 collision proxy 替代 visual/original 几何。

旧 manifest 的 `location` 和 STL 顶点必须采用同一长度转换；rotation 数组的轴顺序/内外禀含义要依据旧 loader + `CMesh.RPY` 核验，不仅因有三个数就套用任意 Euler API。质量未知不能自动等于 1 kg。

固定支撑既可为合法 mesh，也可为显式解析平面。解析平面具有明确法线和作用范围；开放支撑面不需要假装成封闭实体。

## ContactPatch 的最低要求

```text
part_a, part_b                 # 保持调用者指定的顺序
surface_a_id, surface_b_id
dimension                     # 2 面，1 线，0 点
points_a_world_m: (K,3)
points_b_world_m: (K,3)        # 一一对应，可不重合
normals_a_world: (K,3)
normals_b_world: (K,3)         # 曲面保留每点法线，必须为单位向量
regions                       # 全部组件；二维外环/内环或交集单元及 chart->world
area_m2 / length_m            # 面/线分开；点没有伪造的面积
weights                       # 若提供面积积分权重，其和须等于对应面积；点力不是压力
gap_interval_m                # 区域范围和输入误差传播后的证据
classification                # active / near / interference / unknown
quality                       # analytic / bounded / estimated / unresolved
provenance                    # 后端版本、几何/状态摘要、source face IDs、公差摘要
```

不存在接触时返回空 patch 列表及 pair 状态；pair 的 `separated` 必须有排除/覆盖证据。两个零件可能一处 active、一处 near、另一处 interference，不允许用一个 pair 布尔值丢失区域信息。orientation 不可靠时允许缺失法线并标 unknown，但不得输出看似正常的零向量单位法线。

近平行对应点的局部 gap 约定为 `n_A · (p_B - p_A)`，正值为间隙。任意非凸实体对不存在仅靠一个最近点就可靠定义的全局有向间隙；穿透诊断必须由有内外/相交证据的几何查询给出，不能把这个局部 gap 当作全局穿透深度。

`active` 表示在显式接触模型中可激活的几何接触；必须携带 contact mode 的假设/证据。已知正间隙即使小于 near 阈值，也默认只能是 near。若仅因为误差区间跨零而可能接触，记录 ambiguous/unknown，稳定性不得默认激活。由用户选择的零间隙理想化需要单独记录，不能静默吸附位姿。

理想刚体切触的维度与 near 带的维度分别记录。near 带即使有面积，也不能改变点/线接触的受力模型；柔性接触斑块需要额外材料/形变模型，不由阈值自动生成。

## 配置和查询预算

`GeometryConfig` 包含焊接长度、退化面积、平面拟合长度误差、法线夹角、几何误差与是否允许修复。

`ContactConfig` 分别包含 numerical/contact/near/penetration 长度阈值、normal angle、最小可报告面积/长度、自适应误差目标及查询预算。验证阈值为有限非负数，near 不小于 contact。低于报告分辨率的区域单独记入 unresolved/subresolution，不能变成“已经证明不存在”。

`MotionConfig` 单独包含净空要求、路径空间界限、最大平移/转动步长、验证精度、时间/节点预算和随机 seed。`StabilityConfig` 单独包含力/力矩残差、摩擦锥分辨率、扰动力集合、支撑/夹持能力。

不同长度尺度的模型使用明确配置与元数据校准。所有默认数值必须由解析 fixture 标定并解释适用尺度；真实任务不能直接照搬旧 `D<1`、面积 `>30`、负 margin 或固定 `300` 路径长度。

## 公共调用边界

以下签名是模块协作目标，具体 dataclass 字段由 00 落地后成为单一真源。

下面 io / preprocess / surfaces / proximity / analyze_pair / analyze_contacts、`build_contact_graph`、`contact_constraints`、`candidate_motions` 已存在；静力、有限路径与序列层尚不存在。M2.04 的状态绑定、归一化和有限性说明见 [交接](audit-and-m2-04.zh.md)。M1 dispatcher 使用的 MeshProximity 还需要 `geometry_config`、`prepare(mesh)`、`index(mesh)`；CAD 后端不能只实现三个距离函数就假装兼容 mesh dispatcher。

```python
# io / preprocess / surfaces
load_assembly(manifest_path, *, length_unit=None) -> Assembly
prepare_mesh(mesh, *, config) -> PreparedMesh
extract_surfaces(prepared_mesh, *, config) -> tuple[SurfacePatch, ...]

# 共享 ProximityBackend 协议；每个结果携带 witnesses / bounds / status
backend.closest_points(mesh, points_local_m, *, budget) -> DistanceQuery
backend.pair_distance(part_a, tf_a, part_b, tf_b, *, budget) -> PairDistance
backend.classify_overlap(part_a, tf_a, part_b, tf_b, *, budget) -> OverlapResult

# contact（state 包含显式位姿；不得隐式读取 viewer 的当前 pose）
analyze_pair(part_a, tf_a, part_b, tf_b, *, config, backend) -> ContactAnalysis
analyze_contacts(assembly, state, *, config, backend) -> ContactAnalysis
build_contact_graph(assembly, state, analysis) -> ContactGraph

# 后续规划层
candidate_motions(graph, moving_part_ids, *, config) -> MotionCandidates
check_equilibrium(assembly, state, graph, *, config) -> EquilibriumResult
plan_removal(assembly, state, action, *, graph, backend, config) -> RemovalResult
plan_sequence(assembly, initial_state, *, evaluator, config) -> SequenceResult
validate_execution(plan, workcell, *, config) -> ExecutionResult
```

`PreparedMesh` 包含新的 geometry、原面映射、拓扑诊断、法线可靠性。`DistanceQuery` 包含对应点、距离、primitive IDs、覆盖/误差；`OverlapResult` 必须区分 separated/touching/penetrating/unknown，并覆盖完全包含而无表面相交的情况。允许后端缓存局部 BVH；姿态变化不能改变缓存内的局部顶点。

核心 API 返回结构化结果；预期规划失败不抛笼统异常。WRS 原有返回 `MotionData | None` 的原语在 execution adapter 中转换为结果和诊断，保持原接口兼容。损坏输入/不合法配置用带上下文的 `ValueError`，后端缺依赖用可操作的明确异常。

## 运动、受力和成功语义

运动 twist 使用 `[v_x,v_y,v_z,ω_x,ω_y,ω_z]`，同一世界坐标系，v 是指定参考点速度。为数值条件做长度归一化时保存 characteristic length，不能直接混合米与弧度距离。A 固定时对 B 的每个接触采样点使用 `n_A · (v_B + ω_B × r_B) >= 0`。

稳定性中若 A 的外法线 n_A 指向 B，接触力 `f_on_B = λ*n_A + f_t`，`λ>=0`，A 受力为 `-f_on_B`。力矩关于各自 COM 计算。支撑点来自实际 active 区域，near 和孔洞不生成未知力变量。线性摩擦锥采用内接近似；误差随锥分辨率报告。

`EquilibriumResult`：`feasible/infeasible/unknown`、力分配、残差、扰动检验、contact mode、支撑需求和假设。优化数值失败应是 unknown，不能解释为已证明不稳定。

`RemovalAction`：moving IDs、固定/夹持集合、motion primitive、支撑资源。`RemovalResult`：`success/infeasible/unknown/unsupported`、SE(3) 路径、实际验证精度、阻挡证据、状态转移。只有目标件已真正离开目标区域且路径验证达到声明级别，才算 success；试了几个方向失败不构成一般 infeasible 证明。

`SequenceResult`：动作序列、状态链、几何/稳定性证据、搜索预算、失败类别。分别标记 `geometry_validated`、`equilibrium_validated`、`execution_validated`。预算耗尽是 unknown/exhausted，不能输出“无可行装配序列”的全局结论。

`ExecutionResult`：机器人/工具 ID、抓取和关节轨迹、夹持/释放事件、ContactPolicy、验证报告。contact policy 仅允许特定零件、区域、阶段、距离/穿透阈值内的预期接触；不是整件碰撞豁免。支撑切换全过程有状态和资源约束。

## 缓存、可重复性与并行

容差近接可以通过独立 `ToleranceContactPolicy` 显式接受为装配接触，详见
[容差接触与穿入表面](tolerance-contact-and-penetration.zh.md)。此策略在 provenance 和 pair diagnostics
中附加判断，保留原始 classification / quality / overlap / active 面积；不改变上述实际承载语义。
`SDFCollisionChecker.penetration_regions()` 另返回 `wrs.assembly.penetration_regions/1`，
记录双侧穿入表面单元、面积、预算与不确定性；无符号目标侧标记 unavailable，不能当作零穿入。
开启独立几何路径后，`SDFCollisionChecker.query()` 还可返回 `touching / nominal_mesh`；
证据必须包含完整网格的分离支撑平面及表面交集。`touch_regions()` 的点、线、面提取见
[Touching 约定](touching.zh.md)。纯 SDF 近零样本不会单独触发 touching。

- 几何缓存 key：几何摘要 + 预处理配置 + 后端版本。
- contact key：A/B 几何、相对姿态、误差/公差、状态/几何 revision。若缓存的是世界坐标结果，还需世界位姿；否则缓存局部结果并重新变换。
- 稳定性 key：部件集合、位姿、重力、质量/COM、摩擦、接触激活模式、支撑状态和配置。
- 运动/序列 key：完整状态、动作、障碍、持物/夹持/机器人配置（若适用）、净空、预算和 seed。失败缓存不能把一次超时固定成永远不可行。
- 不能仅按零件名字或 unordered part subset 缓存。浮点量化必须有误差分析和边界复核，不能沿用三位小数缓存精密插入结果。
- 结果按稳定 ID 排序；随机模块显式传 `numpy.random.Generator` 或 seed；序列化包含 schema/version/units/provenance。

## 验收命令的约定

所有任务统一使用 `D:\code\venv312\.venv\Scripts\python.exe`，包括依赖安装（`-m pip`）、测试和示例。不要切换系统 Python 或新建另一套虚拟环境。每次交接记录 `sys.executable`、相关包版本和实际加载的 WRS checkout。

下面命令已在 M1 中实际运行；范围和结果见 M1 交接：

```powershell
$assemblyPython = 'D:\code\venv312\.venv\Scripts\python.exe'
& $assemblyPython -m unittest discover -s tests/assembly -p 'test_*.py'
& $assemblyPython tools/gen_api_index.py
```

具体任务添加自己能独立运行的测试文件和命令。核心测试必须验证不依赖 MuJoCo、CAD、显示器和机器人连接；指定环境已经安装部分可选包，可在同一解释器的隔离子进程中阻断可选模块导入来验收，不卸载用户环境中的包。环境扩展用单独集成测试，缺少依赖时显式 skip 并报告未测范围。迁移旧数据只读取已核验 JSON/mesh；不把 `pickle.load` / `eval` 作为新格式入口。
