# 接触面方法选择与轴孔示例修正

2026-09-10，依据用户对轴孔截图的反馈。方法选择是工程建议；没有证据支持某个算法在所有模型、精度和硬件上同时最快、最准。

## 应区分的三类结果

- 几何接触：给定刚体姿态，哪些表面真正相接，属于点、线还是面。
- 近接触/配合：间隙在某阈值内，或具有同轴、插入等设计关系；可以没有实际承载接触。
- 受力接触斑块：需要载荷、材料或柔顺性模型，不能由纯几何阈值确定面积。

## 主要方法与适用边界

| 方法 | 适用问题 | 不能直接替代的内容 |
| --- | --- | --- |
| CAD B-Rep / 解析面、裁剪边界运算 | 有原始 CAD 的工业零件；保留平面、圆柱、NURBS 和 trim 边界 | 最小距离函数本身不返回整个接触区域；仍需面匹配、边界裁剪与误差处理 |
| BVH / AABB tree + 三角形表面查询 | STL / mesh 的最近点、相交和候选筛选；再重建区域 | BVH 是加速结构，不是完整的 contact surface 算法；结果精度受输入网格限制 |
| GJK/EPA + manifold 构造 | 凸体的距离、相交、穿透以及动力学接触点 | 不直接给完整接触面；凹件的孔不能简单用整体凸包替代 |
| SDF / 隐式表面 | 通过距离与梯度查询复杂表面，支持隐式几何 | 解析 SDF、体素 SDF、学习 SDF 的误差不同；局部优化和离散化仍需验证 |
| 接触力学 / hydroelastic | 要求接触压力、力和有限接触斑块的仿真 | 需要额外物理假设；不是只凭 STL 推出实际承载面 |

Open CASCADE 的 `BRepAdaptor_Surface` 提供 B-Rep face 的几何/参数域访问，`BRepExtrema_DistShapeShape` 返回最小距离、对应点和支撑 feature。两者都不是一键“提取完整装配接触面”。[面访问](https://dev.opencascade.org/doc/refman/html/class_b_rep_adaptor___surface.html)、[形状距离](https://dev.opencascade.org/doc/refman/html/class_b_rep_extrema___dist_shape_shape.html)。

CGAL AABB tree 提供三角形最近点及 primitive ID、相交等基础查询；其该组件的距离接口以点查询为主，不应把它描述为完整双 mesh contact patch 检测器。[CGAL 文档](https://doc.cgal.org/latest/AABB_tree/index.html)。

MuJoCo 的原生凸碰撞管线采用 GJK/EPA，并另有多接触点生成。碰撞检测本身是现代基础技术；旧 planner 的问题是用有限碰撞点推断完整几何表面。[MuJoCo 碰撞文档](https://mujoco.readthedocs.io/en/stable/computation/index.html#collision-detection)。其 SDF 插件用距离场与梯度，非凸情况需多起点搜索。[SDF 文档](https://mujoco.readthedocs.io/en/stable/programming/extension.html#sdf)。

Drake hydroelastic 会生成接触面与压力场；刚性/柔顺表示和压力模型参与定义，不能把它当作所有刚体 CAD 配合面的识别器。[Drake 文档](https://drake.mit.edu/doxygen_cxx/group__hydroelastic__user__guide.html)。

## 当前代码使用什么

当前 M1 是自建的 NumPy/SciPy mesh 基线，未调用 CGAL、Open CASCADE 或 MuJoCo 来提取接触区域，也尚未与成熟几何内核做综合性能/鲁棒性对照。

后续更新：现已增加 `ContactAnalyzer("sdf")`，使用 Open3D 网格 SDF 或外部 GridSDF；mesh 基线仍可独立选择。接口、已验证范围和误差限制见 [多后端与 SDF 实现](contact-backends.zh.md)。下面四步描述原 mesh 后端，不能当作新 SDF 后端的实现说明。

1. 清理网格、建立邻接、按整体残差判断 plane/general。
2. 平面：真实三角形投影与凸裁剪、单元拼接、孔/多组件、仿射 gap 分带。
3. 曲面：BVH 最近点、双向三角细分、法线筛选，重建距离阈值内的区域。
4. 独立运行三角相交和实体包含检查，防止局部区域掩盖另一处穿透。

对新 planner 的建议是混合路线：原始 CAD 优先做面类型与真实 trim 区域处理，STL 用经残差验证的面拟合和通用 BVH 后端补充。平面使用二维裁剪；同轴圆柱可以在角度/轴向参数域中处理配合范围与径向间隙，并显式处理周期接缝和端部；非同轴、倾斜或自由曲面再回到通用查询。这是后续方案，当前尚未实现 CAD 或圆柱专用后端。

## 轴孔例子哪里不对

模型是管孔半径 10 mm、轴半径 9.8 mm、轴长 10 mm，两者同轴，存在 0.2 mm 径向间隙。橙色代表距离阈值 0.5 mm 内的 near 带，实际 active 面积应为零。32 段多边形网格的最近距离约 0.199036945 mm；与理想圆柱的 0.2 mm 有离散差异。

原例子的曲面阶段预算为 30,000 次，深度优先细分在遍历完整圆周之前用完。管孔侧只显示 110.881890 mm²，轴侧只显示 387.228965 mm²；轴侧正确完整值是 614.763504 mm²。截图中的周向缺口是未处理区域，不是物理接触只发生在那里。最初的测试验证距离、面积为零等性质，却漏掉了完整轴壁覆盖，这个验收缺项已经补上。

## 修正与验证

- 用同一个目标三角形上的三个见证点给源三角形建立距离上界：目标三角形为凸集，重心插值得到的对应点仍在其内部。上界为三个顶点距离的最大值；不以跨越孔洞的目标凸包替代原表面。
- 对所有可能成为最近面的 primitive 检查法线；整块单元可认证时提前接收，避免继续均匀细分。共用顶点最近点查询复用结果。
- 轴孔示例上限提高至 100,000 次，实际约 33,996 次，两个方向都未耗尽预算。轴侧 614.763504 mm²，单一连通周向区域，完整轴壁覆盖已认证。
- 管孔侧输出 627.309698 mm² 的确定带内区域；端部之外还存在近接触距离带，其范围保留分辨率不确定性。不能把该面积当作孔壁完整 near 带的精确面积。
- 诊断区分未遍历面积与已遍历但边界不确定面积。页面直接显示当前侧完成状态，并修正 B 侧法线的绘制位置/方向。
- 30 项测试通过；新增完整面积、单连通区域、两侧预算未耗尽、低预算不能伪装完成的回归检查。4/2/1 mm 分辨率实验均恢复完整轴壁面积；孔壁端部边界不确定面积随细分缩小。

运行 `examples/assembly/contact_demo.py --all` 更新 `output/contacts.html`。选择“轴孔的正间隙”，勾选“曲面 B 侧采样”可查看完整轴壁；A 侧提示孔壁端部的边界误差。后续 graph/stability 必须继续把 near、active 和 mating 分开。
