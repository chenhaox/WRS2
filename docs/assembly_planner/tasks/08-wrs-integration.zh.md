# 08：WRS 场景、机器人执行与可视化

状态：单/双臂模拟执行基线已实现（2026-09-11）。实际 RS007L + OR2FG7 规划和独立采样 replay 通过；详见 [交接](../m2-m3-execution.zh.md)。

## 目标和修改范围

将候选装配计划转成 WRS 的场景、抓取、IK、关节运动和支撑事件，并显示接触分析及失败证据。交付模拟执行与 replay，不包含实机控制命令。

负责 `adapters/wrs_scene.py`、`execution.py`、`visualization.py`、`examples/assembly/` 中的机器人示例、`tests/assembly/test_wrs_adapter.py` 和相应集成测试。确有必要时可小范围扩展 `PlanningContext`/collider 的策略或缓存配置，保持现有调用兼容，不无差别重写物理模块。

## 实施步骤

1. SceneObject 导入支持多个 visual、非单位 `loc_tf`、重复 mesh 实例和 `sobj.tf @ visual.loc_tf`。更新 pose 时不修改共享 geometry。
2. 用 `wrs.grasp` 和 reasoner 生成/筛选抓取，用 `Arm`、`Recipe` 和 `Workcell` 组合 reach、hold、transfer、insert、release 与退让。
3. 从物体路径和抓取变换推导 TCP 目标；夹爪本体、持物和其他臂都要进入全过程运动验证。
4. 检查 MuJoCo 碰撞近似对孔/凹槽的影响；采用适当分解或真实 mesh 距离补充验证。为预期接触做精确的区域/阶段策略，不能用 `exclude` 整件放行。
5. 精密插入绕开三位关节缓存的风险：配置精度或禁用近似缓存，并在最终路径上使用声明的净空/接触验证。世界 revision、持物/释放和辅助臂状态变化需使缓存失效。
6. 正向执行逐步验证 IK 连续性、夹持能力、辅助臂可达性、支撑接管和对象实际目标 pose。机器人失败可回传 07 作为预算有界的再次搜索信息。
7. 用 WRS viewer 绘制 patch、全部边界/孔、法线场、near/penetration/unknown 和 motion witnesses。颜色图例明确，几何判断仍由核心模块产生。
8. 先选仓库已有抓取与 pick-place 示例能使用的机器人/夹爪组合，例如 RS007L + OR2FG7；如果模型/环境不齐，以实际可运行组合为准并记录，不让核心分析依赖特定硬件。

## 验收

- 多 visual 与非单位局部变换的 mesh，在核心分析和 viewer 中位置一致。
- 同一装配计划完成正向模拟 replay；每步记录对象目标、机器人关节轨迹和夹持事件。
- 接触最终面可以合法到达，但另一面穿透仍会拒绝；孔不会因错误凸包模型被当成实心而产生无解释失败。
- 辅助臂不可达/夹持力不足/抓取冲突时正确失败，支撑事件不会在手未到达时生效。
- 本任务对 WRS 公共接口的修改运行相关现有示例或 headless 检查；只在新模块跑测试不能证明兼容。
- 缺少 GPU、MuJoCo 或显示环境时分别记录未测项目；纯核心测试仍可运行。

## 可复制提示词

```text
在 D:\code\ch\asp\WRS2 实施 docs/assembly_planner/tasks/08-wrs-integration.zh.md。
统一使用 D:\code\venv312\.venv\Scripts\python.exe；安装依赖用该解释器的 -m pip，先核对 sys.executable，不另建虚拟环境。
先读 README.zh.md、contracts.zh.md 和 07 交接；本次把计划接到 WRS SceneObject、grasp/reasoner、Arm/Recipe/Workcell，并完成模拟 replay 和可视化。
重点检查 sobj.tf @ visual.loc_tf、多 visual、持物碰撞、孔/凹槽的凸包近似、精密插入缓存和区域级 ContactPolicy。不能靠全局 exclude 配合件让规划通过。
正向执行要验证 IK/抓取与支撑切换，未验证的执行不可标成功。保持已有 WRS 接口兼容，运行受影响示例，记录实际环境和验收证据。不要发送实机控制命令。
```

## 交接记录

已交付 scene adapter、execution、单/双臂 WRS 示例及 11 项场景/机器人回归。多 visual、内存 mesh 碰撞编译、关闭近似缓存、实际指垫区域、有限夹持能力、错误穿透、不可达、轨迹缺段、基座变更、辅助接管均有检查。机器人采样步距 0.015 rad；显式 float32 舍入容差 2 μm。当前使用 Arm/Workcell 与明确事件循环；未新增通用 Recipe 类型。机器人失败返回可诊断结果，自动跨序列回溯仍待扩展。[命令、性能和限制](../m2-m3-execution.zh.md)。
