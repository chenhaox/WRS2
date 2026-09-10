# 03：表面距离、穿透证据和曲面近接触

状态：待实施。前置：01。可与 02 并行；整合时复用 02 已实现的平面后端。

## 目标和修改范围

建立真实 mesh 查询后端，并提供有覆盖和误差信息的曲面 contact 分析。它也是任务 06 有限路径检查的基础。

负责 `geometry/proximity.py`、`mesh_bvh.py`、`contact/mesh.py`、`tests/assembly/test_proximity.py`、`test_curved_contact.py`。不并发改写 02 的 dispatcher；返回协议先独立测试，整合时串行接线。

## 实施步骤

1. 实现局部 mesh BVH，利用节点包围盒距离下界剪枝；刚体实例共享几何索引。避免构造全量 F×F 距离数组。
2. 实现 point-to-triangle 和 triangle-to-triangle 最近点，包括 vertex-face 和 edge-edge 最小距离、退化输入和相交情况。返回 primitive IDs、对应点和可靠性。
3. 对封闭且朝向有效的 mesh，结合相交与内部包含检查诊断 overlap；完全包含但无表面相交也必须检出。开放/非流形网格不伪造全局 sign，必要时 unknown。
4. 双向查询候选曲面区域并自适应细分，用保守距离界判定排除/需细分。单点距离 d、三角形半径 r 可用于 `d-r` 形式的排除下界，但法线/曲率和最近 feature 切换也必须检查；只有距离界不足以认证面接触。
5. 保留法线场、来源、区域连通性及间隙范围。面积是输入 mesh/近接触定义下的估计，必须提供收敛/覆盖信息；未知区域计入诊断。
6. 可增加 cylinder/sphere 的拟合候选，用残差和真实 mesh 复核。SDF、GPU、Open3D 对照可作为独立实验，第一版不引入为必需后端。
7. 为 `analyze_pair` 的全局穿透诊断提供协议；与平面算法组合时不得因局部 active patch 掩盖另一处 penetration。

## 验收

- 小 mesh 与 brute-force 三角形距离 oracle 一致；oracle 只用于小测试，不作生产实现。
- 最近点位于面内部、边中部而非顶点时仍正确；两个相交三角形及完全包含实体正确分类。
- 窄接触条、小三角区域和薄壁不会因没有随机采中就报告 separated。
- 轴孔、曲面法线变化、重三角化及网格加密：距离和区域结果收敛，并报告未收敛部分。
- 球面与平面理想切触：near 带面积随距离阈值变化，但不能因此生成有限面积 active 承载区域；无法可靠恢复点接触时明确 unknown。
- 预算耗尽、坏法线、开放面返回明确 unknown/unresolved。线/点接触未能可靠恢复时标 unsupported，不能宣告无接触。
- 冷/热查询性能、内存和候选数量有实测；证明不会因 F×F 广播耗尽内存。

## 可复制提示词

```text
在 D:\code\ch\asp\WRS2 实施 docs/assembly_planner/tasks/03-proximity.zh.md。
统一使用 D:\code\venv312\.venv\Scripts\python.exe；安装依赖用该解释器的 -m pip，先核对 sys.executable，不另建虚拟环境。
先读 README.zh.md、contracts.zh.md 和 00/01 交接。任务 02 可能同时进行，只写本任务声明的文件范围。
建立真实 mesh BVH/最近点/overlap 协议及曲面自适应 contact 分析。不能把顶点 KD-tree 当表面距离，不能把稀疏采样阴性当成无接触证明，也不能对开放 mesh 使用未经验证的 signed distance。
完整包含、edge-edge 距离、细小接触、法线场和预算耗尽是必须验收的场景。以结构化证据、误差/覆盖和 unknown 边界为交付标准。
完成测试和性能记录后交接；若与 02 共用 dispatcher，整合时串行修改，不覆盖另一任务工作。
```

## 交接记录

待填：ProximityBackend 实际 API、overlap 假设、精度/覆盖语义、性能、未支持场景、基线。当前未实施。
