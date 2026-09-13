# SDF 批量化、active 语义、Bunny 面积与碰撞测试

> 入口整理说明（2026-09-13）：本文的旧脚本命令、输出路径和计时保留为历史记录。当前请按 [WRS 例子使用说明](../../examples/assembly/README.md) 运行，参数在文件顶部修改；benchmark 已移出 examples。

2026-09-11，分支 `codex/assembly-planner`。解释器固定为 `D:\code\venv312\.venv\Scripts\python.exe`。

## 三种任务的边界

1. **近接带**：在源表面寻找距离小于 near 且法向相对的区域，并重建多边形、边界和面积。`ContactAnalyzer('sdf')` 执行此任务。
2. **实际接触面积**：零间隙的接触面域，不能由非零厚度的 near 带直接替代。当前 SDF 后端没有一般的零集接触面提取器。
3. **碰撞查询**：判断给定位姿是否存在穿入，或是否有足够的证据证明分离。新 `SDFCollisionChecker` 不计算 near 带，不过滤相对法向，不组装区域面积，也不调用 mesh 碰撞判定来替代 SDF。

因此，之前几秒的区域计算时间不能当作 SDF 碰撞检测本身的耗时。

## 细分、裁剪在做什么

源三角形跨越 near / 法向阈值时，不能仅凭一个采样点决定整张三角形是否属于区域：

- 细分：把大三角形沿最长边拆成两个，以缩小未确定区域。
- 裁剪：对终止单元按插值阈值求边界交点，只保留满足条件的多边形部分。
- 量测：计算该多边形面积及重心，并在重心重新查询 SDF。
- 区域组装：把保留单元连接起来，恢复边界、洞和连通组件。

本轮把逐单元的几何操作改为数组批处理：

- 距离、有效性、法向、是否继续细分等判断变成布尔数组。
- 最长边选择与子三角形生成使用 `(N,3,3)` / `(N,2,3,3)` 数组。
- 三个线性条件逐批裁剪；每个三角形最多保留六个顶点，用 `(N,6,3)` 加顶点数表示变长多边形。
- 三角扇面积及重心批量计算，避免每个单元反复创建小数组。

只在收集有效多边形和队列记录时保留轻量 Python 循环。连通性与 T 型接缝处理仍在 `cell_regions` 中，不宣称整个算法已经完全向量化。没有通过改大 near、减少检查或平滑显示来换速度。

同一台 Ultra 9 275HX、同一 0.1 mm 终止半径、三次暖查询中位数：

| 模型 | 本轮优化前 | 本轮优化后 | 双侧面积 |
| --- | ---: | ---: | --- |
| 法兰 | 4.781 s | 1.742 s | A=1167.321450、B=1140.062842 mm²，保持一致 |
| 圆柱 STL | 2.724 s | 1.278 s | A=500.122048、B=490.718105 mm²，保持一致 |

原始数据在 `output/vectorize-before/measurements.json` 和 `output/vectorize-after/measurements.json`，后者另有 profiler。其他批次存在运行波动，不能跨参数和批次拼接加速比。向量化后仍有独立 mesh 距离验证、边界组装和报告构造开销；碰撞调用若不需要面域，应使用下面的独立接口。

## 为什么 active 原来全部“未求解”

旧界面只要看到 SDF 后端，就统一显示“未求解”，没有区分已知分离和仍需求解的状态。

现新增 `pair_diagnostics.active_area`：

- `known_zero` / `area_m2=0`：独立 mesh 距离下界明确大于几何、位姿及数值保护量，且全局关系是 separated。此时没有实际接触，界面显示 0.00 mm²。
- `not_solved` / `area_m2=null`：其余情况，包括零间隙、干涉、原生 SDF 的全局定义未验证等。界面保留“未求解”，不能根据空 active 列表推断零面积。

默认 STL 支撑例子有 0.2 mm 正间隙，因此可以显示实际接触面积为零。需要平面零间隙面域时可使用已有 mesh 后端；它的平面裁剪结果与 SDF near 带是不同量。一般曲面实际接触求解仍是后续工作。

## Bunny 面积的独立复核

本例支撑上表面为 z=0，XY 范围完全覆盖 bunny。可绕过 SDF，直接按向下法向条件筛选原始 STL 三角形，再用水平面 z=0.5 mm 裁剪，得到 B 面面积参考值：

| 模型 | SDF B 面面积 | 独立三角形裁剪 | 差值 |
| --- | ---: | ---: | ---: |
| Bunny 大 | 491.506632 mm² | 491.506512 mm² | 0.000120 mm² |
| Bunny 小 | 131.711500 mm² | 131.711491 mm² | 0.000009 mm² |

另核对了报告面积、绘制单元面积总和、积分权重总和，三者一致。A 面区域顶点全部在 z=0；B 面在 z=0.2–0.5 mm（含浮点误差）。没有发现区域漂浮到兔子躯干或面积被重复累计的证据。

检测到的唯一自交三角形对位于模型局部 z=40.79–55.81 mm，远离这个底部高度带。491.51 mm² 是当前 B 面候选带面积；侧栏的 2349.52 mm² 则是双侧不确定性汇总，其中包含已着色但法向尚未认证的面积，不能当作另一块接触面积。near=0.5 mm、间隙=0.2 mm 实际只选中了最低约 0.3 mm 的高度带，并不等于整只脚掌。大小两只兔子使用相同绝对阈值，所以选中面积也不必严格按模型缩放比例的平方变化。

透明显示使后方/底部区域可透过模型看到，容易误读位置。现增加“底视”和“仅当前侧零件”，可结合“显示 B 面结果”检查；取消“零件”可单独查看区域。候选带表面积、屏幕投影面积与承载接触面积不能混用。

`bunny_area_audit.py` 的参考算法不查询 SDF；审计 JSON 保存在 `output/stl/bunny-area-audit.json`。它验证的是当前水平支撑案例，不证明任意模型与位姿都已达到同样精度。

## 独立 SDFCollisionChecker

```python
from wrs.assembly import ContactModel, SDFCollisionChecker

a = ContactModel.from_file('part_a.stl', name='A', length_unit='m')
b = ContactModel.from_file('part_b.stl', name='B', length_unit='m')
checker = SDFCollisionChecker(resolution_m=0.0002, max_query_points=200000)
checker.prepare(a)  # 可选：预先建场，重复位姿查询复用局部几何
checker.prepare(b)
report = checker.query(a, b, tf_a=pose_a, tf_b=pose_b)
print(report['status'], report['reason'])
```

返回值为 `wrs.assembly.sdf_collision/1` 的 JSON-compatible 字典：

| 状态 / 原因 | 所用证据 | 限制 |
| --- | --- | --- |
| separated / disjoint_aabb | 含几何及浮点保护量的包围盒分离 | 此快捷路径没有执行 SDF 查询，报告 query_points=0 |
| penetrating / negative_sdf_surface_witness | 源三角形上的点在目标有符号场内部，距离超过保护量与穿入阈值 | 来源是表面穿入证据，不认证开放源网格包围了体积 |
| penetrating / shared_sdf_interior_witness | 一个候选点经两个字段分别验证，均位于内部 | 用于重合/共同内部，不能只把面心向内偏移就当作内部点 |
| separated / positive_sdf_cell_bounds | 两侧源表面全部由正 SDF 下界排除，且两个目标字段都有合法符号 | 受字段误差模型约束，quality=estimated |
| unknown | 无符号目标、零附近单元、分辨率限制或预算不足 | 不能解释为 collision-free，也不返回易误用的布尔值 |

面积提取会做法向筛选和 near 截断；碰撞检查不会这样做，所以深穿入、薄杆交叉、包含也在测试范围内。预算和提前退出都记录在 sides 中；找到穿入证据后的未遍历区域不影响这条正证据，但不能宣称完成了整面扫描。

当前接口仅支持 mesh-derived SDF。传入原生 GridSDF 时显式拒绝：仅在积分网格顶点检查零集配准，不足以定义整个碰撞实体和场覆盖范围。它也不是连续碰撞检测；两个离散位姿之间的运动仍需另行检查。

Open3D 的 signed/occupancy 要求明确的内外，float32 数值保护不是形式化误差证书，参见 [官方文档](https://www.open3d.org/docs/release/python_api/open3d.t.geometry.RaycastingScene.html)。SDF 分类均保留 estimated，包围盒路径另标 `aabb_bound`。

## 从 primitive 到真实 STL：49 个姿态

6 个生成后保存并重新读取的 STL：盒子、球、圆柱、管、环面、凹 L 形。

8 个仓库原始 STL：bunny、法兰、圆柱、OpenArm 手指与手掌、UR3 前臂、FR3 link7、xArm link3。面数从 12 到 12,420，没有用凸包替代。Bunny、UR3、FR3、xArm 在当前验证下不能建立可靠有符号实体；示例显式开启 unsigned，并保存字段标记。

每个模型做 2 mm 正间隙、零间隙和 2 mm 穿入，共 42 个姿态；另有宽间隙轴孔、窄间隙轴孔、轴孔干涉、完全包含、完全重合、薄杆交叉以及禁用 AABB 的 SDF 分离，共 7 个。

| 构造关系 | 数量 | 实际结果 |
| --- | ---: | --- |
| 穿入/包含/重合 | 18 | 18 个 penetrating，均有负 SDF 证据 |
| 分离 | 17 | 16 个 separated；1 个窄间隙轴孔 unknown |
| 零间隙贴合 | 14 | 14 个 unknown，未将其误报为确定分离 |

这不是“49 个全部准确判定”。窄间隙轴孔虽真实分离，但在 0.2 mm 单元半径及 200,000 点预算内没有完成正距离覆盖，返回 unknown；零接触处也不能靠有限精度符号唯一确定接触类别。

暖查询穿入约 1–19 ms；宽间隙轴孔通过纯 SDF 单元界限确认分离约 21 ms；窄间隙轴孔耗尽预算约 261 ms。14 个正间隙模型走的是 AABB 快捷路径，其时间不能拿来宣称 SDF 查询吞吐量。输入读取与建场成本单列，首个模型准备还包含 Open3D 首次导入。所有三次原始计时、字段标志、查询数、预算和证据在 `output/collision/results.json`。

## 运行与交接

```powershell
Set-Location 'D:\code\ch\asp\WRS2'
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/sdf_collision_demo.py
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/bunny_area_audit.py
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/stl_demo.py --repeat 3
& 'D:\code\venv312\.venv\Scripts\python.exe' -m unittest discover -s tests/assembly -v
```

碰撞交互页为 `output/collision/contacts.html`，红点表示穿入/共同内部证据，不伪造碰撞面面积。原 near 带图库仍为 `output/stl/contacts.html`。

后续优先研究近零和窄间隙的局部界限、平面/圆柱解析后端及真正的零间隙面域。不能只增加点数后就把所有 unknown 强制改成 free 或 active。

验收：53 项 assembly 测试通过；49 个碰撞姿态完成并保留 unknown；Bunny 独立面积审计通过；公共 API 索引已生成；浏览器中检查了碰撞面积指标隐藏、穿入证据点、Bunny 底视与 active=0。
