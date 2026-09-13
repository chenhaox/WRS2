# wrs/assembly 文件审查与类型标注

当前目录见[新的包目录说明](../../wrs/assembly/README.md)。本页是第一轮审查的历史记录：之后旧数据转换已移到 tools，运行库 legacy adapter、12 个旧路径转发模块和独立 HTML 导出已删除；下方关于保留它们的结论仅对应当时状态。

2026-09-13，基线 26b3651。审查范围包括 wrs/assembly 的 42 个 Python 文件及 HTML 资源，并检查了 wrs、examples、tests、benchmarks、tools 和公开 API 文档中的引用。动态导出、协议、按文件名读取的资源也计入用途。

## 删除了什么

本轮没有确认可安全整文件删除的模块。删除的是以下已确认冗余的代码：

| 位置 | 删除内容 | 判断依据 |
|---|---|---|
| stability.py | 私有辅助函数 _contact_points | 全库无调用或动态绑定；生产代码和计时脚本都直接使用 prepare_force_points |
| contact/analysis.py | 未使用的 numpy 导入 | 文件内没有使用 |
| visualization.py | 未使用的 numpy 导入 | 显示逻辑通过已有数组对象操作 |
| contact/planar.py | 未使用的 clean_polygon 导入 | 本模块没有调用；geometry/planar.py 的实际函数保留 |
| adapters/legacy.py | x, y, z = r | 三个变量没有使用；旋转矩阵由 sin(r)、cos(r) 计算 |

没有把“没有普通函数调用”直接等同于“无用”。以下内容需要保留：

| 文件／接口 | 当前用途 |
|---|---|
| adapters/wrs_scene.py | 机器人执行、可抓取性、接触／方向／稳定性／序列 WRS 示例 |
| adapters/legacy.py | tools/assembly/legacy_inventory.py 的扫描与转换；论文场景、测试和示例的旧旋转约定 |
| visualization.py、_contact_viewer.html | benchmarks/assembly/sdf_resolution.py 的可选独立 HTML；WRS 接触显示复用 COLORS |
| visualization.build_wrs_scene | 仍有公开 API 文档的独立场景构造入口；实际构造已验证可用 |
| constraints.py | 通用平移／twist 约束和旧候选 API；纯平移 directions.py 并不替代它的全部功能 |
| geometry/proximity.py 的 ProximityBackend | 低层距离查询协议；现在补全并用于类型标注 |
| contact/backends.py 的 ContactBackend | mesh／SDF／自定义接触算法的高层扩展协议 |
| geometry/sdf.py 的 SignedDistanceField | 原生 SDF 数据接口，不能以“没有直接实例化”判断无用 |
| _batched_lp.py | 多方向稳定性的 NumPy／CUDA 批量求解，按运行路径导入 |
| __init__.py | Python 包边界和惰性公开导出；只有 docstring 的子包入口也保留 |

旧基准脚本、论文工具和公开接口不因最近的普通示例未调用就删除。本轮没有创建新的运行时子系统，也没有修改公共名称或输入文件 schema。

## Adapter 是什么

Adapter 是**数据转换层**。规划器使用不可变的 Part、MeshData、AssemblyState；WRS 使用可更新的 SceneObject、visual 和机器人 link。它们的用途、精度和坐标表示不同，需要显式转换。

adapters/wrs_scene.py 的四个入口：

| 函数 | 转换内容 |
|---|---|
| part_from_scene_object | 合并原始 visual 网格及各自局部变换，生成分析 Part；机器人 Link 也是 SceneObject 的子类 |
| scene_object_from_part | 将 Part 构造成 WRS 对象，供显示和机器人碰撞检查使用 |
| rigid_tf_from_wrs | 校验刚体变换，修正允许范围内的 float32 旋转舍入误差，拒绝缩放／剪切 |
| apply_state_to_scene | 将 AssemblyState 的位姿写回现有 WRS 对象 |

输入分析使用 visual 几何，保留零件的真实表面。质量、质心和摩擦需要调用者提供；缺失值继续保持未知。

~~~python
from wrs.assembly.adapters.wrs_scene import (
    part_from_scene_object,
    scene_object_from_part,
)

part = part_from_scene_object(scene_object, "peg", mass_kg=0.1)
display_object = scene_object_from_part(part, collision=False)
~~~

adapters/legacy.py 处理 ASP_OLD 的 JSON + STL。它要求显式提供 m/mm/cm，将位移和 STL 顶点统一换算为米，并复现旧的外禀 xyz 旋转顺序 Rz @ Ry @ Rx。多个零件实例复用同一 STL 时共享分析几何；缺失模型会在清单中标出，完整转换会报错。

即使以后不再运行 ASP_OLD，现有论文数据的旋转约定仍需要这个转换。两个 adapter 当前都不宜删除。

## 补充了哪些类型

重点覆盖模型、输入输出、adapter、接触分析和后端、接触图、距离协议、方向与评分、序列搜索、静力／扰动、受力点、抓取和机器人执行。

主要变化：

- 入口明确使用 Assembly、AssemblyState、ContactGraph、ContactModel、各自 Config 和 Result。
- SequenceStep.removal 从 object 改为 RemovalResult；序列步骤、质量列表、扰动结果和缓存注明元素类型。
- 安装侧状态属性返回 AssemblyState；路径返回 tuple[FloatArray, ...]。
- 接触点和摩擦受力点区分浮点数组与整数分组；TCP、Grasp、机器人封装注明对应 WRS 类型。
- 自定义报告和扩展回调保留必要的开放类型；没有把所有参数统一写成 Any。
- FloatArray、IntArray 是 model.py 中的 NumPy dtype 别名。数组形状和物理单位继续由文档与原有运行时校验保证。
- 使用延迟求值的 annotations；WRS 类型放在 TYPE_CHECKING 内，保持导入阶段不启动渲染、机器人或 GPU。

例如现在的核心入口可以直接看到输入和结果：

~~~python
def plan_quality_sequence(
    assembly: Assembly,
    grasp_analyzer: GraspabilityAnalyzer,
    goal_state: AssemblyState | None = None,
    *,
    supports: Iterable[AuxiliarySupport] = (),
    config: QualitySearchConfig | None = None,
) -> QualitySequenceResult:
    ...
~~~

类型提示没有增加逐点或逐帧运行时类型检查。数组批量计算、稀疏矩阵、GPU 求解和缓存算法维持原样。

## 验证

- 对 313 个保留函数／方法做 AST 对比：忽略类型标注和 import 排版，以及上述确认冗余的变量赋值后，执行逻辑与基线相同。
- 全包 Ruff F 检查通过，检查未定义名称和未使用导入等问题。
- 无界面子进程测试现在解析全部 assembly 公共导出及两个 adapter，禁止可选渲染／仿真依赖导入。
- 新增 legacy adapter 集成测试：检查单位换算、多个实例共享网格、未知物理量保留，以及缺失模型不能被静默丢弃。
- 独立构造公开 build_wrs_scene 场景，并验证 HTML 模板能生成报告；临时输出使用临时目录，不生成 examples/assembly/output。

完整回归 **183 项通过，125.610 s**，使用指定的 Python 3.12 解释器。

运行方式：

~~~powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' -m unittest discover -s tests/assembly
~~~
