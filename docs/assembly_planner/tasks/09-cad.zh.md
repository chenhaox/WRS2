# 09：可选 STEP / B-Rep 接触后端

状态：待实施。前置：02、03 的后端协议稳定。可在 STL 基线完成后单独执行。

## 目标和修改范围

直接利用 CAD 原始曲面和 trim 拓扑获取 contact / mating 候选，并输出相同 ContactAnalysis，避免把解析面先损失成粗 mesh 再反推。

负责 `contact/cad.py`、CAD 专属 importer/helper、`tests/assembly/test_cad_contact.py`。可选依赖声明与共享输入类型扩展需串行整合，不能把 CAD import 加进核心包入口。

## 实施步骤

1. 在目标 Python/Windows 环境核验 OCP 或 pythonocc 的可安装版本及官方 API；选一个后端先实现，不把两个都设为核心依赖。
2. 从 STEP assembly 读取单位、零件实例及其 placement。保留原 TopoDS face、surface 参数、trim loops、面方向和 CAD tolerance；为绘图生成 mesh 时保存 face 映射。
3. 对 plane-plane 进行真实 trim 区域交集和 gap 分析；处理孔、多组件、反向 face orientation 和近似共面。
4. 对 cylinder-cylinder 检查轴线/半径/轴向与角度覆盖，并处理周期 seam。正径向间隙是 mating/near，不能直接产生 active 承载接触。
5. 用 BRepExtrema 的最短距离和 support points 做候选/证据，不把一个最短点当成整块 contact surface。一般曲面区域追踪达到误差要求，做不到则保留 unknown 或回退有标识的 mesh 后端。
6. 输出 source CAD face IDs、CAD/几何误差、后端版本和标准 ContactPatch 字段。与 mesh 后端在相同几何和公差下比较，解释 mesh 弦误差。

## 验收

- 程序生成简单 STEP fixture，包含平面孔、同轴圆柱、相同几何的多个实例和不同单位；无需等待外部 CAD 样本才能做基础测试。
- CAD 平面接触与解析面积一致；网格细化后 mesh 结果向 CAD 基准收敛。
- 圆柱 seam 不丢区域/不重复计面积，正间隙轴孔不被激活承载。
- 没有 CAD 可选依赖时核心模块可导入，调用 CAD 功能才给出清晰安装说明。
- 一般 B-spline contact 的能力边界及 fallback 的误差来源写入结果，不声称已支持全部工业 CAD。

## 可复制提示词

```text
在 D:\code\ch\asp\WRS2 实施 docs/assembly_planner/tasks/09-cad.zh.md。
先读 README.zh.md、contracts.zh.md、02/03 的后端交接；为现有 contact 协议增加可选 STEP/B-Rep 后端。
先核验目标环境可用的 OCP/pythonocc 版本和官方 API，再实现实例/单位/trim/面方向导入、平面接触和圆柱配合。最短距离只是证据之一，不等于接触区域。
保留孔、多组件、周期 seam 和 CAD face 追溯；正间隙 mating 不得当成承载接触。保持核心无 CAD 依赖，生成小 STEP fixture 做验收，并记录 mesh/CAD 精度对照与能力边界。
```

## 交接记录

待填：后端版本/安装命令、CAD 几何句柄生命周期、fixture、支持曲面、误差对照、基线。当前未实施。
