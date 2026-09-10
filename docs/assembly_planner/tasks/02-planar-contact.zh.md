# 02：平面 contact surface 的确定性实现

状态：待实施。前置：01。可与 03 从同一已提交基线并行。

## 目标和修改范围

让平面 contact surface 完全由几何确定，输出完整交集区域和间隙，而不调用 collision manifold。

负责 `geometry/planar.py` 的裁剪/区域拼接、`contact/planar.py`、`contact/analysis.py` 的最小平面调度、`tests/assembly/test_planar_contact.py`。03 写 mesh 后端，二者只通过既定模型连接。

## 实施步骤

1. 对零件/patch 使用按 near 阈值扩张的 AABB 做候选筛选；扩张不修改 mesh。
2. 对近似相对的平面选择共同的局部 chart，将原 mesh 的三角形投影。用可靠的二维定向判断和三角形凸裁剪求交。
3. 依据已清理、内部不重叠的三角覆盖，收集交集单元并计算面积/质心。去除数值重复单元，拼接所有边界环；不能只取一个 polygon 或只取 exterior。
4. 在两侧原平面上计算对应点和有向 gap。倾斜平面的 gap 在 chart 内是仿射量，按 contact/near/penetration 分界裁剪单元，保留各区域。
5. 输出 ContactPatch 的维度、边界、面积/长度、法线和间隙证据。线/点交集采用独立分类，不能用报告面积阈值直接抹掉。
6. 全局零件穿透结论暂交 03 的 overlap query；本阶段的局部面匹配不能宣告“整对零件无穿透”。对未覆盖的几何/无效输入明确 unknown。
7. 导出机器可读结果和可供后续 viewer 使用的 overlay 数组。不要在几何函数里绘图。

## 验收

- 0.1 m 箱体完整接触面积为 0.01 m²；横向错开 0.025 m 时为 0.0075 m²。
- 交换 A/B：面积相同、法线和双侧证据按约定互换；整体刚体变换不改变面积和间隙。
- 公差范围内外的 gap、已知正间隙、倾斜窄带、深穿透分别得到正确分类。
- 方环孔保持为空；两个不连通交集全部输出；相同面不同三角化得到等价结果。
- 线/点接触面积为零且维度正确；近退化情况不能制造大面积。
- 不安装任何碰撞/物理引擎即可通过验收；输出 provenance 和数值误差，而不是只输出 True/False。

## 可复制提示词

```text
在 D:\code\ch\asp\WRS2 实施 docs/assembly_planner/tasks/02-planar-contact.zh.md。
统一使用 D:\code\venv312\.venv\Scripts\python.exe；安装依赖用该解释器的 -m pip，先核对 sys.executable，不另建虚拟环境。
先读 README.zh.md、contracts.zh.md、00/01 交接及本任务。仅实现平面 contact surface 和必要的二维裁剪。
使用真实三角形覆盖的投影交集、对应点 gap 和明确公差，保留凹形、孔、多组件及线/点接触；不要调用 Bullet/MuJoCo/FCL 来决定 contact patch。
特别测试倾斜表面只能局部接近的情况，不能用平均平面距离把整面当成接触。局部面分析不得伪装成全局无穿透证明。
按任务范围实现、运行解析验收，在任务文档记录实际结果和交接接口。与任务 03 的共享类型严格采用 contracts。
```

## 交接记录

待填：裁剪策略、容差标定、解析误差、overlay 格式、测试结果、基线。当前未实施。
