# Agriculture / Citrus V2

轻量植物几何与 WRS 构建适配层。当前真正支持实验室 citrus reference tree、通用程序化树、
静态构建以及少量 branch-cluster 被动弯曲。没有引入 Newton、Warp、外部植物资产或新的依赖。
完整设计、OrchardBench 阅读记录、core 变更和范围见 [ARCHITECTURE.md](ARCHITECTURE.md)。

## 运行

在 WRS2 根目录，使用已安装项目依赖的 Python 3.12+：

```powershell
# 原 V1 入口仍然可用
.\.venv\Scripts\python.exe -m examples.agriculture.citrus_tree.demo_static_tree

# 同一份 PlantSpec：静态、枝簇检查、真实接触推动
.\.venv\Scripts\python.exe -m examples.agriculture.dynamic_citrus --case static
.\.venv\Scripts\python.exe -m examples.agriculture.dynamic_citrus --case clusters
.\.venv\Scripts\python.exe -m examples.agriculture.dynamic_citrus --case push

# FAFU 笛卡尔移动 + 实时叶簇/果实接触
.\.venv\Scripts\python.exe -m examples.agriculture.robot_citrus_interaction

# 确定性通用程序化路径
.\.venv\Scripts\python.exe -m examples.agriculture.generic_plant_demo --seed 7
.\.venv\Scripts\python.exe -m examples.agriculture.generic_plant_demo --seed 21

# 无 viewer 的真实 MuJoCo 接触验证及轨迹导出
.\.venv\Scripts\python.exe -m examples.agriculture.dynamic_citrus --case push --headless --report push_report.json --trace push_trace.json

# 回归测试：工作区 tests（含 agriculture / 机械臂交互）+ 原 V1 的 19 项
.\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_*.py" -v
.\.venv\Scripts\python.exe -m unittest discover -s examples/agriculture/citrus_tree -t . -p "test_*.py" -v
```

新例子也支持直接运行脚本。`--port` 选择 viewer 端口。
静态/枝簇例子支持 `--duration 5` 有界退出；push 跑完接触、回弹和稳定阶段后退出，网页保留最终场景。
无浏览器自动弹出时设置 `$env:WRS_VIEWER_NO_BROWSER='1'`。

## 机械臂交互示例

`robot_citrus_interaction.py` 使用现有 FAFURobotArm 裸臂和固定在法兰上的蓝色圆头触碰工具。
机械臂安装在 0.30 m 高的底座上，树的尺度不变。左侧提供前后、左右、上下六个按钮，
`Move per click` 调节每次移动 1～30 mm，默认 10 mm。方向使用固定世界坐标：
前为 +Y（朝树）、后为 −Y、右为 +X、左为 −X、上为 +Z、下为 −Z，不随相机旋转改变。
TCP 是蓝色球心，工具朝向保持固定。右侧显示目标/实际 TCP 坐标、接触类别、植物偏转和果实世界坐标。

使用 WRS NumIKSolver，从相邻解出发检查整条直线路径；默认每 5 mm 一个 waypoint。
关节解之间插值近似直线，同时限制 TCP 命令速度与各关节命令速度。UI 不直接修改 FK/qpos，
机械臂实际位姿由 MuJoCo 位置伺服与接触共同决定，受重力和接触影响可有毫米级跟踪误差。
越界、不可达或出现过大 IK 跳变时，整次请求被拒绝并显示原因，原目标/原路径保留。
这不使用默认 SELIK，因此首次运行不生成采样数据库。

可以先点击 **Touch leaves** 或 **Touch orange_000**，观察枝簇/果实让位，再点击
**Retract / ready** 观察回弹。**Stop motion / hold** 取消后续移动并保持当前伺服命令；
**Pause physics** 暂停仿真，暂停时可调整待执行目标；
**Reset robot and tree** 恢复初始化后的完整物理状态。复位保留暂停和显示开关状态。
接触代理、机械臂/枝条/果实碰撞形状和接触力箭头均有独立显示开关。

交互参数集中在 `configs/presets/lab_citrus_robot.json`，继承 lab citrus preset：
机械臂基座、工具尺寸、示范目标、位置伺服增益、笛卡尔/关节速度、工作区与相机均可调整。
示范目标同样走上述笛卡尔路径。它是便于探索接触的局部控制，不是避障轨迹规划。
只有机械臂有六个 actuator，植物保持 11 个 passive DOF。叶片通过 V2 的枝簇代理接触；
橙子仍与枝条刚性连接，不会抓取或脱落。触碰工具的 ACTIVE 碰撞角色显式设置，
不会因为它是固定 mount 而与植物的 STATIC 角色互相过滤。

无界面接触与撤回验证、或限制 viewer 运行时间：

```powershell
python -m examples.agriculture.robot_citrus_interaction --headless --report robot_contact.json
python -m examples.agriculture.robot_citrus_interaction --port 8001 --duration 30
python -m unittest discover -s tests -p "test_robot_citrus_interaction.py" -v
```

未修改 WRS core、FAFU 的几何/关节定义或现有植物动力学参数。
FAFU 可用于当前定性接触演示，但质量/惯量/电机参数尚未标定，原生双指夹爪的 mimic 也尚未
转换成 MuJoCo 联动约束，因此本例使用圆头工具，不装双指夹爪。详细调查见
[FAFU 支持范围](../docs/tutorials/fafu_robot_arm.md#用于-citrus-交互仿真的支持范围)。

## 数据与 API

```python
from agriculture.config import load_config
from agriculture.generator import generate, ReferenceFittedGenerator, ProceduralTreeGenerator
from agriculture.spec import PlantSpec
from agriculture.dynamics import PlantDynamicsSpec

config = load_config()                       # species + lab preset，普通 dict
spec = generate(config, seed=20260916)        # 不导入 WRS / MuJoCo
spec.validate()
spec.to_json('plant.json')
restored = PlantSpec.load('plant.json')

skel = spec.skeleton
print(skel.roots, skel.terminals)             # segment ID 列表
print(skel.children['fruit_support_001'])     # 派生邻接关系，无重复拓扑状态
print(skel.descendants('front_middle'))
print(skel.depth('cover_upper'), skel.total_length(), skel.summary())

from agriculture.static import StaticPlantBuilder
from agriculture.dynamic import DynamicPlantBuilder
static = StaticPlantBuilder(config).build(spec, pos=(0.5, 0, 0))
dynamics = PlantDynamicsSpec.from_config(spec, config)
dynamic = DynamicPlantBuilder(config).build(spec, dynamics, pos=(0.5, 0, 0))
# static.add_to_scene(world.scene) 或 dynamic.add_to_scene(world.scene)
orange_world_position = dynamic.fruits['orange_001'].pos
orange_rest_position = spec.fruits[1].position
```

- `StemSegment`：id、parent_id、起止点、两端半径、order、attachment_t、rotmat、semantic_role。
  children / depth 从 skeleton 推导，避免与 parent 信息失步。生成器把 3×3 rest frame 保存进 rotmat；
  手写 segment 可省略 rotmat，`frame` 属性按同一 convention 生成。
- `PlantSkeleton`：segments、roots、terminals、children、descendants、depth、bounds、height、total_length、validate、summary。
  支持多个根，用于未来灌木/基生茎；兼容 TreeSpec 仍要求单根位于原点。
- `PlantSpec`：skeleton、leaves、fruits、species、preset_name、metadata、leaf_shape；JSON、校验、bbox、缩放。
  `.branches` 是兼容 V1 的别名。
- 叶和果实使用 `parent_segment` / `attachment_t`；保留 `parent_branch` 属性兼容旧调用。
  叶片保存 rotmat / color_class；二者可带仅含 JSON 值的 visual_metadata，供后续视觉适配使用。
- `PlantGenerator` 是 Protocol：`generate(config, seed) -> PlantSpec`。
  `ReferenceFittedGenerator` 消费手工 macro skeleton；`ProceduralTreeGenerator` 随 seed 改变宏观枝条、叶和果实。
- `apply_pipe_model(skeleton, terminal_radius, beta, tip_taper)` 返回新的 skeleton，默认不用于 reference preset。

全部位置和尺寸使用 metre，角度在数据层使用 rad，配置中 `_deg` 参数使用 degree。
segment 的 local +Z 是生长方向，X/Y 是稳定垂直基；3×3 矩阵列向量表示 local axes 在 plant-local 中的方向。
叶片沿用 V1 blade +X 基部到尖端、+Y 宽度、+Z 正面。两种局部 frame 用途不同。
`T_world_plant` 只由 builder / instance 保存；spec 始终为 rest plant-local。

## 配置与反复调参

真正的参数在 `configs/species/citrus.json` 和 `configs/presets/lab_citrus_v2.json`。
原 `examples/agriculture/citrus_tree/configs/*.json` 是兼容引用，支持继续叠加 override。
配置 `extends` 按 dict 递归合并，列表整项替换；循环引用会报错。

1. 固定 seed，编辑 lab preset 的 macro_branches / growth_profiles / fruit_placements。
2. 静态比较继续使用 V1 demo 的固定相机和 watch：
   `python -m examples.agriculture.citrus_tree.demo_static_tree --config agriculture/configs/presets/lab_citrus_v2.json --watch`。
   watch 监视指定 preset 文件；修改继承的 species 文件后需要重启。
3. `dynamics.cluster_roots` 只列选中的枝簇根；成员自动取其子树，嵌套根归最近的 cluster。
   动态子树不能留下仍固定的后代，独立 dynamics.validate 会拒绝这种断连。
4. 用 `--case clusters` 检查分组与代理，再调 profile 的 stiffness / damping / limits / mass 参数。
5. 用 headless push 报告比较偏转、真实代理接触、叶果位移、释放后的振荡与稳定。

species 包含典型形态默认值，generator 决定生成方法，preset 决定这一个场景的拟合几何和动态子树。
scale_multiplier 统一缩放几何；动力学系数不自动做相似缩放，proxy padding 是缩放后的米制参数。
V2 目前没有 dynamics 热更新；修改动态配置后重启，以重新编译模型。

## 显示开关

`dynamic_citrus.py` 提供用户要求的九个 `SHOW_*` 常量。命令行有对应开关，例如：

```powershell
python -m examples.agriculture.dynamic_citrus --case push --show-foliage-proxy --show-branch-collision --show-joint-axes
python -m examples.agriculture.dynamic_citrus --case clusters --show-leaves --no-show-skeleton
```

完整集合是 full-plant、skeleton、static-branches、dynamic-clusters、leaves、fruits、
branch-collision、foliage-proxy、joint-axes。full-plant 控制正常枝条显示，叶/果由各自开关控制。
clusters case 默认显示骨架、着色动态簇、proxy 和 joint axes，保持 rest pose 便于检查。
push case 使用真实 MuJoCo 接触；蓝色测试球是唯一有 actuator 的物体。

## 当前数量与边界

lab preset：274 segments、1,528 leaves、3 fruits、8 dynamic clusters、11 passive DOF、12 foliage proxies。
静态叶片 3 个 batch；动态叶片 20 个 batch，按 cluster 和 shade 分组。WRS articulated links 共 12 个，
其中 3 个是两自由度弯曲的轻量中间 link。叶片没有独立 body/joint；果实固定 mount，没有额外 DOF。

默认静态形态 bbox 约 `[-.4187,-.3409,-.014] -> [.3288,.2885,1.1762] m`。
所有 bbox/summary 几何数值是 rest spec 的统计；运行中的世界位姿从 instance 读取。
视频尺度仍是未经实测的估计；动力学参数只是定性调节，没有系统辨识。

植物使用 WRS 原生 STATIC **碰撞角色**，因此它与 ACTIVE 机器人/可操作物体接触，
植物内部没有接触，不需要 O(N²) exclude。关节仍然被动物理运动。
该策略也排除了不同植物之间、植物与 STATIC 场景物体的接触；需要这类交互时，
应明确扩展通用碰撞过滤，而不是把叶片换成逐叶 collider。

未实现 wind、detachment、tether、branch breaking、逐叶关节、FEM、soft body、完整 L-system、
pruning/trellis 操作、系统辨识、传感器仿真或 RL。现有 camera tests 仅作为本次 core 修改的回归验证。
