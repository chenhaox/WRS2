# SDF 轴孔边界修正、真实 STL 与计时

2026-09-10，分支 `codex/assembly-planner`。使用 `D:\code\venv312\.venv\Scripts\python.exe`。

本页记录上轮的 v2 修复与计时。后续针对 STL 锯齿做了分辨率对比、距离上界批量化并澄清 A/B 与 unknown，见 [接触诊断与性能](contact-diagnostics.zh.md)；当前法兰和圆柱的示例默认分辨率改为 0.1 mm，下面旧表对应 1 mm。

## 截图中的问题

用户看到的梳齿和不连续端部属于几何区域提取问题，不应解释为物理接触的真实形状。上一轮只验证轴侧全周面积，没有把孔侧端部边界作为验收条件。

两个原因叠加：

1. 轴的端盖和侧壁共享边缘，最近点落在边缘时对应多个三角形。旧实现用 Open3D 返回的其中一个三角形法线代替 SDF 梯度；在同一圈 128 个查询点上，97 个返回侧壁法线，31 个返回端盖法线，产生不连续筛选。
2. 终止细分后，仅按三角形中心决定整块接收或丢弃，边界沿三角网格跳动，产生梳齿。

修正：有符号场在远离零集时使用 `sign(phi) * (p-q) / ||p-q||`；零附近/无符号模式仍保留面法线并声明限制。中心与三个顶点都查询距离/法线，终止单元按距离阈值和法线阈值裁剪，裁剪后的区域重心重新查询真实对应点。端部现在是切出的多边形边界，不是整块三角形台阶。

真实圆柱 STL 还暴露了细长三角形的性能问题。四分细化同时切短边，会创建大量无必要单元。改为最长边二分，结合每层单元 AABB 剔除；预算耗尽仍报告未遍历面积，不会把部分结果宣告完成。

SDF 边界仍是插值估计，不是精确 CAD 配合区域。当前例子是径向间隙 0.2 mm、near 阈值 0.5 mm、法线角度 0.55 rad。距离与法线阈值会允许轴端附近少量扩展，不能把该带解释为严格的轴向配合长度或承载面。两侧面积独立，轴侧完整面积仍为 614.763504 mm²。

## 新增 STL 示例

所有文件直接读取仓库原始 STL，以 WRS 中使用的米为显式单位，没有简化、凸包替代或静默修复。零件底部放在封闭箱形支撑上方 0.2 mm，near 阈值 0.5 mm。

| 例子 | 原文件 | 三角形数 | 有符号场条件 |
| --- | --- | ---: | --- |
| Bunny / 170 mm | `bunny.stl` | 2,780 | 检测到一对自交三角形；bunny 作为目标采用显式 unsigned 模式 |
| Bunny / 48 mm | `bunny_small.stl` | 2,780 | 同样有一对自交三角形 |
| 法兰 | `link6.stl` | 490 | 封闭、无检测到的自交，可查询 signed SDF |
| 圆柱 | `examples/l1picking/cylinder.stl` | 576 | 封闭、无检测到的自交，可查询 signed SDF |

默认 `SDFContactBackend` 会拒绝上述 bunny 的有符号场构建。示例明确设置 `open_surface="unsigned"`：可计算表面距离和近接触区域，但不能推断 bunny 内外。Bunny 作为源、封闭支撑作为目标的反向查询仍可使用支撑的有符号场。本例有 0.2 mm 正间隙，AABB 不相交，独立包围盒检查可确认 separated；若包围盒相交，则不能用这份自交网格认证实体内部，全局结果保留 unknown。

## 运行与计时定义

```powershell
Set-Location 'D:\code\ch\asp\WRS2'
# 五个案例，SDF，一次首次计算 + 三次缓存复用。
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/stl_demo.py --repeat 3
# 单独选择真实 STL，也可以比较两个后端。
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/stl_demo.py --case bunny --backend both
# 更新主图库，已有案例和四个真实 STL 均可选择。
& 'D:\code\venv312\.venv\Scripts\python.exe' examples/assembly/backend_demo.py --all
```

结果位于 `examples/assembly/output/stl/contacts.html`、`timings.json` 和各案例的 `*.contacts.json`。

- **首次计算**：新 backend 的局部几何准备、Open3D 建场、独立 mesh 验证与双侧区域积分。
- **缓存复用**：同一个 backend 保留局部几何；仍然重新计算距离验证、区域和边界，不是读取缓存的最终结果。报告三次的中位数、最小值、最大值。
- **分阶段**：`prepare`、`mesh_validation`、`band_integration`；对外总耗时还包括统一报告组装。
- **排除项**：STL 读取、JSON/HTML 写入、浏览器渲染。STL 读取单列 `load_s`。Open3D 导入单独测量一次，不混入每个模型的首次计算。
- **计时范围**：本机 CPU 墙钟时间，无并行模型测试；结果只代表这些数据和配置。帧率、单点 SDF 查询和完整区域分析不是同一种耗时。

当前例子使用 1 mm 终止分辨率、每侧 60,000 单元、每对最多 500,000 SDF 查询点；实际查询数与未遍历面积同时记录。图库直接显示实测的缓存复用中位数。

实测 CPU：**Intel Core Ultra 9 275HX**；Python 3.12，Open3D 0.19.0。独立导入 Open3D 为 **0.946 s**。以下是同一进程中逐模型串行测试，首次计算排除这次导入；缓存复用列为三次中位数。

| 模型 | 首次计算 | 缓存复用中位数 | 缓存复用范围 | 未遍历面积 |
| --- | ---: | ---: | ---: | ---: |
| 轴孔 | 2.408 s | 3.062 s | 2.663–3.121 s | 0 |
| Bunny / 170 mm | 3.474 s | 3.026 s | 2.930–3.050 s | 0 |
| Bunny / 48 mm | 0.913 s | 0.732 s | 0.710–0.754 s | 0 |
| 法兰 STL | 1.740 s | 1.737 s | 1.696–1.845 s | 0 |
| 圆柱 STL | 3.522 s | 3.888 s | 3.458–3.947 s | 0 |

缓存复用不保证每次更快：主要耗时仍是区域重算，运行波动可能超过省掉的建场时间。`timings.json` 保存全部单次数据、阶段、配置、面数与环境。这里的零未遍历面积表示预算覆盖完成，并非采样或边界误差为零。

## 验收

新增回归覆盖：轴端梯度对三角形次序不敏感；孔侧边界使用裁剪多边形；倾斜解析 SDF 的带面积与解析值一致；规则网格 SDF 的箱体尖角不导致整面遗漏；真实 bunny 的 signed 模式拒绝；封闭圆柱的内部/外部符号正确。原有网格、状态、单位、缓存与预算测试继续保留。

源侧的积分单元使用内部法向（网格源使用面法向，原生 SDF 使用单元中心梯度），目标侧在中心与顶点采样法向。这样避免把源表面共享尖角处不唯一或为零的梯度，当作整张面的法向。网格 SDF 的箱体例子两侧均恢复为 16 mm²；此处理仍归类为采样估计。

当前总计 46 项测试。STL 图库的五个案例全部在预算内遍历完成；SDF 输出仍为 estimated near band，不是 active 承载区域。

官方查询语义参考：[Open3D RaycastingScene](https://www.open3d.org/docs/release/python_api/open3d.t.geometry.RaycastingScene.html)、[距离查询教程](https://www.open3d.org/docs/release/tutorial/geometry/distance_queries.html)。接口设计与后续 CAD/active 区域工作见 [多后端文档](contact-backends.zh.md)。
