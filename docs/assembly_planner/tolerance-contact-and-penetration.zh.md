# 容差接触规则与穿入表面显示

2026-09-11；分支 `codex/assembly-planner`。统一解释器：`D:\code\venv312\.venv\Scripts\python.exe`。

后续补充：[Touching 判定与点、线、面显示](touching.zh.md)。其中新增独立支撑平面接触路径；本文末尾 14 个 touch 返回 unknown 是该补充前的历史结果。

## 为什么以前碰撞页没有红色相交面

`SDFCollisionChecker.query()` 只需要找到一个可靠的负距离点，就可返回 `penetrating`。它之前返回的是红色证据点，没有提取区域。现在新增独立调用：

```python
from wrs.assembly import SDFCollisionChecker

checker = SDFCollisionChecker(open_surface='unsigned')
collision = checker.query(model_a, model_b)
regions = checker.penetration_regions(
    model_a, model_b, resolution_m=0.0005, max_query_points=200000)
```

`collision` 仍保留快速提前退出。`regions` 在两侧分别遍历源三角形，批量查询目标 SDF，通过距离上下界排除外部、接受内部，并在边界细分和线性裁剪。它不使用近接带的法向过滤，也不受 `near_tol_m` 限制。

红色是 **源表面位于另一实体内部的部分**，不是布尔交集体积、接触面或相交线。它包括嵌入后在实体内部的表面；边界受采样半径和距离误差影响，标记 `estimated`。A/B 面积独立报告，不合并。

无符号目标没有可信内部，因此该侧返回 `unavailable`，不能填成面积 0 的已求解结果。例如 bunny 为 B、封闭支撑为 A：A→B 无法提取内部；B→A 可以显示 bunny 穿入支撑的表面。全重合实体可能只有公共内部证据点，严格位于对方内部的源表面为空。

每侧保留 `query_points`、`boundary_uncertain_area_m2`、`unprocessed_area_m2`、`invalid_area_m2`。预算不足返回 `incomplete`；空白不是分离证明。默认碰撞示例只对发现穿透的姿态做区域提取，且区域计算不计入快速查询耗时。

## Bunny 的 unknown 与容差是两件事

当前仓库 bunny STL 检出一对自交三角形。无符号距离仍可查询“距表面多远”，但无法作为可信的实体内外判据。字段元数据现在直接给出 `solid_validation`：闭合、定向、水密以及自交三角形对数。Open3D 对符号距离的实体假设见 [Distance Queries](https://www.open3d.org/docs/release/tutorial/geometry/distance_queries.html)。

支撑 A 表面查询 bunny B 的场时使用无符号距离，所以近接区域的几何分类是 `unknown`。反过来查询有效的支撑字段，可以得到有符号距离。此处不是 SDF 无法处理兔子形状，也不是还没找到邻近区域。

现在可附加一层明确的装配规则：

```python
from wrs.assembly import (
    ContactAnalyzer, ContactConfig, SDFContactBackend, SDFConfig,
    ToleranceContactPolicy,
)

contact_tol_m = 0.0005  # 0.5 mm
analyzer = ContactAnalyzer(
    SDFContactBackend(sdf_config=SDFConfig(
        open_surface='unsigned', max_query_points=500000)),
    config=ContactConfig(near_tol_m=contact_tol_m, normal_angle_rad=0.55),
)
geometry = analyzer.analyze_pair(model_a, model_b)
result = ToleranceContactPolicy(
    contact_tol_m=contact_tol_m,
    allow_unsigned=True,
).apply(geometry)

for patch in result.patches:
    decision = patch.provenance['contact_policy']
    if decision['is_contact'] is True:
        print(patch.sampling_side, patch.area_m2, decision['reason'])
```

- `contact_tol_m`：允许按装配接触处理的距离阈值，单位 m。
- `allow_unsigned=False` 默认保留无符号区域为未定；明确设为 True，才接受其表面邻近关系。这是工程假设，不认证实体内部。
- `is_contact`：True / False / None，分别为接受、拒绝、当前证据不足。
- `classification`、`gap_interval_m`、`quality`、实体相交关系和实际 active 面积全部保留。蓝色容差接触与物理 active 是不同结果。
- 已判明的 `interference` 不会因阈值增大变成 contact；有符号区间跨过零但无法排除穿入时也不会直接接受。
- 面积来自当前已提取区域，存在未遍历部分时不宣称完整。空候选集不是碰撞或接触不存在的证明。

`ToleranceContactPolicy` 与后端解耦，也可应用到 mesh 后端输出。SDF 记录 `band_limit_m` 说明已提取带的阈值。对已有 0.5 mm 区域应用更小容差，若整个区域无法判定，会给出 `reextract_band_at_contact_tolerance`；不会把部分满足的重心样本算成整片面积。应将 `near_tol_m` 同步设置为所需阈值并重新提取。

已有 `ContactConfig.contact_tol_m / idealize_contact` 属于平面求解器的零间隙理想化选项，不会自动把一般 SDF 曲面变成 active。本次策略适用于装配邻近关系；把它用于支撑力或刚体约束需要后续规划层显式采用相应模型。

## 页面与复现

```powershell
Set-Location D:\code\ch\asp\WRS2
$assemblyPython = 'D:\code\venv312\.venv\Scripts\python.exe'
& $assemblyPython examples/assembly/sdf_collision_demo.py
& $assemblyPython examples/assembly/stl_demo.py --contact-tol-mm 0.5 --allow-unsigned-contact
# 容差小于 bunny 的 0.2 mm 初始间隙，重新提取会得到空带：
& $assemblyPython examples/assembly/stl_demo.py --case bunny --contact-tol-mm 0.1 --allow-unsigned-contact --out-dir examples/assembly/output/tolerance-small
& $assemblyPython -m unittest discover -s tests/assembly -v
```

- `output/collision/contacts.html`：默认打开首个具有穿入区域的姿态。红点、红面同时显示；可切换单侧/双侧；快速查询和区域提取分开计时。
- `output/stl/contacts.html`：蓝色显示按规则接受的区域；紫色边界提示无符号假设。取消“接受无符号近接面”可看严格策略，取消“容差接触显示”恢复原始颜色。切换只影响显示，不重新查询。
- bunny 示例实际最低点离支撑 **0.2 mm**。0.5 mm 容差允许其中的近接区域成为装配规则的 contact，但其实际零间隙接触面积仍为 0。

测试覆盖解析盒子穿入面积、完整包含、单侧无符号、预算不足、显式位姿，以及容差阈值、原始证据不变、禁止将干涉或零点不确定性直接接受等行为。

本次验收：**58 项测试通过**。49 个碰撞姿态仍为 18 penetrating / 16 separated / 15 unknown；18 个穿透案例已附区域结果。默认演示区域预算为 500,000 点、终止半径 0.5 mm，所有可用的有符号目标侧均遍历完成；无符号目标侧明确标为 unavailable。Bunny 的穿入区域在 B 面约 4307.21 mm²（向支撑穿入 2 mm），与正间隙示例的近接面积不是同一量。

Bunny 快速碰撞查询暖运行中位数约 4.17 ms，附加区域提取约 0.264 s；这是本机墙钟时间，快速查询不含准备、文件读取、写报告与区域提取。0.5 mm 容差的正间隙示例接受 A≈491.19、B≈491.51 mm²；0.1 mm 对照例子的区域数为 0。
