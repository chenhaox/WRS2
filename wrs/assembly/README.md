# Assembly 包的代码分类

从 `from wrs.assembly import ...` 使用常用 API。查看实现时，按下面的职责进入目录。

```text
assembly/
├── model.py                 共享的不可变数据、单位与校验
├── io.py                    新版 assembly JSON 与 STL 读写
├── geometry/                网格、距离、SDF、基础几何计算
├── contact/                 接触区域、接触分类和接触图
├── motion/
│   ├── constraints.py       局部平移／twist 约束
│   ├── directions.py        Fibonacci、SOCP 与退化方向空间
│   └── part_motion.py       有限零件路径与独立路径检查
├── mechanics/
│   ├── equilibrium.py      静力平衡与有限辅助支撑
│   ├── force_points.py     受力点、冗余约简与曲面回退
│   ├── stability_sweep.py  多方向扰动承载极限
│   └── _batched_lp.py       共用约束矩阵的批量 LP 内核
├── planning/
│   ├── sequence.py         可行装配／拆卸序列
│   ├── quality.py          独立的 assemblability 与 S/G/A 评分
│   └── quality_search.py   质量引导的深度优先搜索
├── robotics/
│   ├── graspability.py     当前装配状态下的合格抓取计数
│   └── execution.py        WRS 抓取、搬运、插入、交接和回放
├── adapters/wrs_scene.py   WRS 场景对象与 Part／位姿的转换
└── visualization/
    ├── contact.py          接触显示与可选 HTML 报告
    └── _contact_viewer.html HTML 报告的本地模板
```

根目录中原来的 `sequence.py`、`stability.py` 等是短兼容入口，算法已经搬到对应目录。它们把旧导入指向同一个模块对象，不复制实现或缓存，也不在每次数值调用外包一层函数。新增代码使用分类后的路径；已有脚本暂时不用批量改名。

兼容入口用 `TYPE_CHECKING` 显式导出公开类型，方便编辑器识别旧导入中的类和函数；这些类型导出不在运行时执行。

例如：

```python
from wrs.assembly import plan_sequence, check_equilibrium

# 需要读实现或使用更具体接口时：
from wrs.assembly.planning.sequence import SequenceEvaluator
from wrs.assembly.mechanics.stability_sweep import DirectionalStabilityAnalyzer
```

## contact 与 geometry 的区别

`geometry` 解决几何问题：清理网格、建立 BVH、计算最近点／距离、查询 SDF、做多边形相交。它不决定某块区域是否可以承载力。

`contact` 解决两个零件的接触问题：结合位姿、距离、法线与容差，形成点／线／面区域，区分 active、near、interference、unknown，最后建立 ContactGraph。

主要使用关系：

```text
geometry → contact → motion / mechanics → planning → robotics
    └────────────────→ 有限路径与机器人几何检查
```

箭头表示结果供谁使用，不是完整的 Python 导入图。`contact` 调用 `geometry`；`motion/part_motion.py`、`robotics/execution.py` 还直接使用几何查询。删除任一目录都会破坏主流程。

两个 `planar.py` 也不重复：`geometry/planar.py` 提供平面坐标、多边形裁剪与区域连通工具；`contact/planar.py` 利用这些工具计算两个零件的平面接触证据。

## 下划线文件是什么意思

单下划线是 Python 的内部实现命名约定，不是“没用”或“待删除”的标记；Python 也没有禁止外部导入它。

| 文件 | 调用者与用途 |
|---|---|
| mechanics/_batched_lp.py | stability_sweep 调用；共用基、批量单纯形和 NumPy／CUDA 运算。原始 FP64 残差复核，不确定时回退 HiGHS |
| visualization/_contact_viewer.html | write_contact_html 读取；旋转、缩放、颜色和接触证据显示，不参与物理判定 |
| geometry/_triangle_batch.py | proximity 调用；批量三角形距离与交叉计算 |
| contact/_sdf_cells.py | SDF 接触／相交区域调用；向量化细分、裁剪和面积积分 |
| contact/_support_contact.py | SDF 碰撞查询调用；识别平面支承下的点、线和面接触 |
| contact/_penetration_regions.py | 相交显示调用；提取进入另一个物体内部的表面片段，不是实体布尔交体积 |

`__init__.py` 的双下划线是另一回事：这是 Python 的包入口文件。只有说明文字的入口也有包边界作用。

## ASP_OLD 数据如何处理

运行库里的 `adapters/legacy.py` 已删除。它的职责分成两部分：

- 旧模型名映射、JSON/STL 和单位转换：位于仓库工具 `tools/assembly/legacy_data.py`，由 `legacy_inventory.py` 调用。
- 通用外禀 xyz 旋转：位于 `geometry/transforms.py` 的 `rotation_xyz`，明确采用弧度和 `Rz @ Ry @ Rx`，不依赖任何旧格式。

论文例子现在只加载 `examples/assembly/assets/paper2021/assemblies/*.assembly.json`。这些是 `wrs.assembly/1` 格式，几何与位姿用米，旋转保存为矩阵。34 份文件覆盖全部可用 raw／nominal 组合；其余 6 个 raw 模式缺少源模型或原始场景，继续明确报错。

raw 和 nominal 分别导出，已知的网格修复、角度恢复、重建说明及修正量都记录在 provenance 中。运行例子时不再转换旧 JSON、重建体素或生成替代场景；不可变装配文件加载后按例子缓存。

原始 JSON/STL 和摘要保留为复现输入，原 ASP_OLD 仓库也保持原样。这些源文件供转换工具和来源校验使用。需要重建新版论文数据时，从 WRS2 根目录运行：

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' -m tools.assembly.paper_assets
```

其它旧装配数据仍可用 `tools/assembly/legacy_inventory.py` 转换，必须显式指定 m/mm/cm。新程序使用 `load_assembly`，不需要安装或导入 tools。

## 整理范围与验证

本次只搬迁和分类已有数值实现，保留 type hint、批量数组、GPU 求解、延迟导入和缓存。14 个迁移模块中的 183 个函数／方法经 AST 对比，除导入路径外执行代码保持一致。

新增回归检查新旧模块身份一致、34 份装配数据与转换结果一致、示例运行不导入转换工具，以及 HTML 模板在新路径可用并列入安装包资源。

2026-09-13：指定 Python 3.12 下，187 项测试通过，耗时 111.849 s。此耗时是测试套件时间，不是单次规划性能基准。

完整回归命令：

```powershell
& 'D:\code\venv312\.venv\Scripts\python.exe' -m unittest discover -s tests/assembly
```
