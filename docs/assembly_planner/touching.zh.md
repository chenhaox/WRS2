# Touching 判定与点、线、面显示

> 入口整理说明（2026-09-13）：本文的旧脚本命令、输出路径和计时保留为历史记录。当前请按 [WRS 例子使用说明](../../examples/assembly/README.md) 运行，参数在文件顶部修改；benchmark 已移出 examples。

2026-09-11，分支 `codex/assembly-planner`。

## 之前为何所有 touch 都是 unknown

旧 `SDFCollisionChecker.query()` 只有 separated / penetrating / unknown 三种返回路径：负距离证据判穿入；完整正下界判分离；零距离单元无法归入两者，就一直细分，最终成为 unknown。页面又只为 penetrating 提取红面。因此示例中虽构造了 touch 姿态，判定器和显示都没有对应路径。容差策略此前只应用于 STL 接触分析页。

这不是“零距离一定无法检测”，也不能通过把所有接近零的样本改成 touching 来解决：同一对模型可能在一处相切，在另一处穿透。

## 新增的独立几何检查

开启 `use_aabb=True`（默认）时，检查世界坐标轴、两零件局部坐标轴及中心连线方向是否存在分离支撑平面。所有 A 顶点位于平面一侧、所有 B 顶点位于另一侧，可同时约束完整三角网格，排除其他位置的横向穿过。再对平面上的原始三角形、边、顶点求交；只有找到真实交集才返回 touching。

边界包围盒相切本身不够。例如小盒位于圆环孔上方，虽然上下包围盒相切，平面内没有交集，仍不能判 touching。正间隙或穿入超过数值容差也不会被吸附成接触。

这是**独立的网格几何路径**，不是纯 SDF 的零距离认证，输出 `quality='nominal_mesh'`、`scope='supplied_mesh_surfaces'` 和 `separating_support_plane` 证据。`use_aabb=False` 保留纯 SDF 查询路径，接近零仍可能 unknown。无符号模型可有明确的名义表面接触，但其实体内部有效性仍未认证；该状态不自动构成真实承载能力。

候选支撑平面是有限集合，不保证识别任意曲面配合。轴孔整圈重合等没有分离平面的场景仍需其他接触后端。明确声明了几何误差的模型不会通过此路径认证零间隙。

## 程序调用

```python
from wrs.assembly import SDFCollisionChecker

checker = SDFCollisionChecker(
    open_surface='unsigned',
    max_query_points=50000,  # SDF 查询预算
    max_touch_tests=50000,   # 独立接触几何预算
)
relation = checker.query(model_a, model_b)
if relation['status'] == 'touching':
    contact = checker.touch_regions(model_a, model_b, max_triangle_tests=50000)
    print(contact['area_m2'], contact['cell_dimensions'], contact['complete'])
```

快速 query 找到一个有效交集即可结束；`touch_regions()` 独立提取全部交集，单独计时。数据版本为 `wrs.assembly.support_contact/1`，保留原面对应、数值容差、支撑平面及预算。

每个 cell 的 dimension 为 0 / 1 / 2；二维 cell 面积相加。共享边或顶点可能作为低维见证重复出现在面域边界上，不能把 cell 数量当作独立接触组件数，也不在面积上重复计数。区域预算耗尽时 `complete=False`，面积只是已提取部分。

## 显示与例子

- bunny、球体：此姿态下是点接触，显示绿色点，面积 0 正确。
- xArm link3、离散圆环 torus：此姿态下包含线接触，显示绿色线。
- 法兰、圆柱端面、箱体等：显示绿色面域，保留孔洞。
- 红色仍表示穿入；蓝色仍属于另外显式采用的容差接触规则，不能把它们当作绿色名义零间隙面。

碰撞页 `examples/assembly/output/collision/contacts.html` 现在默认打开 bunny/touch，并显示接触维度、名义网格接触面积、独立几何检查数和区域提取时间。

```powershell
Set-Location D:\code\ch\asp\WRS2
$assemblyPython = 'D:\code\venv312\.venv\Scripts\python.exe'
& $assemblyPython examples/assembly/sdf_collision_demo.py
& $assemblyPython -m unittest discover -s tests/assembly -v
```

当前 49 个姿态中，14 个构造接触全部返回 touching；18 penetrating、16 separated、1 unknown（窄间隙轴孔预算不足）。法兰名义接触面积约 1140.062842 mm²，圆柱 STL 约 490.718105 mm²。这些是输入网格的零间隙面积，与 0.5 mm near 带面积不同。

回归覆盖面、线、点、圆孔、旋转/交换顺序、正间隙与浅穿入不吸附、几何误差、区域预算和包围盒接触但位于孔内的反例。纯 SDF 模式仍保留 unknown 语义。

验收：指定 Python 3.12 环境下 **60 项测试通过**。浏览器检查 bunny 点接触与法兰带孔面接触，绿色几何和面积显示一致。
