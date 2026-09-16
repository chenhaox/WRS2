# FAFURobotArm

FAFU 六轴机械臂与双指夹爪从旧版 PantheraHT Python 定义迁移，采用当前
`MechStruct`、`MechBase`、`SingleArmManipulation` 和 `GripperMixin` 接口。

```python
from wrs.robots.manipulators.fafu import FAFURobotArm, fafu_with_gripper

arm, gripper = fafu_with_gripper(jaw_width=0.05)
arm.fk(qs=[0, 1.1, 1.5, -0.4, 0.3, 0.2])
gripper.set_opening(0.03)
tcp = gripper.tcp('grasp_center')
solutions = arm.ik(tcp.pos, tcp.rotmat, tcp=tcp, ref_qs=arm.qs.copy())
collider = arm.build_collider()
collided = collider.is_collided(arm.qs)
```

`FAFURobotArm()` 创建裸臂；`fafu_with_gripper()` 返回已装配的 `(arm, gripper)`。
`arm.end_effector` 由安装关系自动得到。关节单位为弧度，长度单位为米。
`arm.add_to_scene(world.scene)` 同时显示已安装的夹爪。
可运行 `python examples/fafu_robot_arm.py` 查看模型和两个 TCP。

## 接口变化

- `fk(qs=...)` 更新连杆及夹爪位姿；TCP 位姿通过 `tcp.pos/rotmat/tf` 获取。
- 默认 `arm.ik(pos, rotmat)` 的目标是法兰。抓取目标需显式传入夹爪 TCP，
  如上例。IK 返回解列表，无解时返回 `[]`。
- `fix_to` 改为 `set_pos_rotmat`；`goto_given_conf` 改为 `fk`。
- `change_jaw_width` 改为 `gripper.set_opening`；也可使用 `open()`、`close()`。
- 碰撞检测通过独立的 `collider` 进行，沿用新版 `build_collider()` 的结构重叠
  自动排除机制。应在已知合理的初始姿态编译，并检查其 `acm`；旧版完全不注册
  内部自碰撞对的策略没有沿用。
- 如编译碰撞器后改变夹爪开口，使用
  `collider.set_mecba_qpos(gripper, gripper.qs)` 同步该非 actor 机构。

默认 IK 复用 WRS 的 SELIK，首次调用会在机械臂包的 `data/` 下生成采样数据库，
初始化可能较慢且需要该目录可写。构造参数 `solver=` 可传入接受 chain 的求解器
工厂。若用 `NumIKSolver`，通过 `qs_active_init=` 提供初值；它不使用 `ref_qs`。
本次迁移未修改公共解析求解器，也不依赖 `trac_ik`。

## 保留的模型约定

- 第一关节高度为 **0.0634 m**，沿用旧 Python 定义；随附 URDF 是 0.0584 m。
- 第六关节局部旋转为 `Ry(pi/2)`，局部运动轴为 Z；法兰 Z 轴沿工具前向。
- link6 保留为无网格的法兰连杆，掌部外观仅由夹爪提供，避免重复网格。
- link2 的原始网格超过 MuJoCo 的 STL 面数限制；显示保留原网格，碰撞使用
  新版已有的凸包代理，不修改公共碰撞后端。
- 夹爪最大开口为 **0.085 m**；双指在掌部两侧沿 ±Y 各移动一半开口。
- 夹爪根坐标系与法兰重合。旧耦合的 **0.005 m** 偏移体现在掌部网格及手指
  关节原点；抓取 TCP 距根坐标系 **0.17 m**，不是 0.175 m。
- 保留全零 home 配置。裸臂无网格的法兰使用很小的数值惯性，供 MuJoCo 碰撞
  模型编译；其他网格也未做动力学校准，本迁移面向运动学、显示与碰撞规划。

## 用于 Citrus 交互仿真的支持范围

已在 `examples/agriculture/robot_citrus_interaction.py` 接入 **FAFURobotArm 裸臂**，
使用法兰上固定的圆头工具，跑通 MuJoCo 真实叶簇/果实接触与撤回后的回弹。
无需修改机械臂网格、关节定义或 WRS core。

```powershell
python -m examples.agriculture.robot_citrus_interaction
python -m examples.agriculture.robot_citrus_interaction --headless --report fafu_contact.json
```

UI 使用固定世界坐标的六向笛卡尔移动，默认每次 10 mm，工具朝向固定。
使用 `NumIKSolver` 和明确的相邻初值检查短距离 waypoint，随后通过有速度限制的
MuJoCo 位置伺服执行。越界/不可达/IK 跳变时拒绝整条新路径，保留旧目标。
`Touch leaves`、`Touch orange_000`、撤回、暂停和复位均可使用。

| 模块 | 已有支持与当前限制 |
| --- | --- |
| 模型、FK、TCP、mount、clone | 已实现并有回归测试。无网格 link6 是法兰 frame 的有意设计，不是遗漏网格。 |
| IK | 可使用现有 SELIK/NumIKSolver；交互例子显式用 NumIK，避免首次建库。局部数值 IK 不保证找到所有可达解，奇异点附近可能拒绝移动。 |
| 碰撞与 MuJoCo 编译 | 裸臂与装配体都能编译；link2 用凸包规避 STL 面数限制。已验证裸臂带工具的动态接触。碰撞包络保守，不能视为精确零件接触几何。 |
| 动力学参数 | **尚未标定**质量、质心、惯量、关节摩擦和电机性能。多数网格连杆未显式设置惯量，由 MuJoCo 从几何及默认密度估计；法兰仅有数值占位惯量。示例伺服增益是定性值，不代表真实 FAFU 响应或额定力矩。 |
| 双指夹爪 | FK/mount/开口和碰撞规划可用；**物理 mimic 联动尚未接通**：当前转换器未输出 joint equality，装配体编译为 8 DOF、8 actuators、0 equality，而 WRS 夹爪运动学只有 1 个独立 DOF。不可直接据此宣称单驱动同步夹持仿真已完成。 |
| 实机控制 | 本次迁移没有导入旧控制器，当前包没有对应的 FAFU 实机通信/执行接口。 |

当前 Citrus 演示使用 6 个机械臂 actuator 和 11 个植物 passive DOF；蓝色工具无独立关节。
模型在当前配置下可稳定推动枝簇和果实，能用于定性交互及路径探索；尚不支持真实硬件
动力学预测、标定后的力控或物理双指抓取。后续若使用原夹爪，应先补通用 mimic→MuJoCo
约束/驱动映射，再标定连杆与驱动参数；本次没有通过两个独立伺服伪装单驱动夹爪。

验证命令：

```powershell
python -m unittest discover -s tests -p "test_fafu_robot_arm.py" -v
python -m unittest discover -s tests -p "test_robot_citrus_interaction.py" -v
```

## Citrus 中的 VirtualD405 安装

`robot_citrus_interaction` 默认包含腕部 RGB-D；实现位于
`examples/agriculture/fafu_d405.py`，不修改 FAFU 模型或 WRS core。
参考 [FAFU SDK 的 fafu_baseV1_d405.urdf](https://github.com/FAFU-Robotics/fafu_arm_sdk/blob/4f4a246efe97c51ded89d3fd8a9b9a595238e77f/fafu_robot_python/fafu_robot_description/urdf/fafu_baseV1_d405.urdf)，
固定提交 `4f4a246efe97c51ded89d3fd8a9b9a595238e77f`。其中 `tool_joint` 是
`link6 -> tool_link` 的固定变换：xyz = `(0.170, 0, 0)` m，rpy = `(0, 0, 0)`。
文件没有 camera link、optical frame 或相机标定外参；不能把文件名中的 d405 当作光学标定。

当前 WRS 裸臂沿用旧模型的腕部轴约定：WRS flange 相对 URDF link6 旋转 `Ry(pi/2)`。
因此 helper 使用 `T_flange_optical = Ry(-pi/2) @ T_link6_tool @ T_tool_optical`。
这里 `T_A_B` 将 B 坐标转换到 A，所有平移单位为 metre。
名义光轴指向 URDF tool +X，光学 +X = tool −Y，光学 +Y = tool −Z；
光心相对 tool 位于 `(-0.050, 0.055, 0)`，后退并侧移以避开示例的蓝色触碰球。
这两个偏移是可替换的示例安装估计，**不是从 URDF 提取的测量值**。

默认合成的 `T_flange_optical` 为：

```text
 0   1   0   0
-1   0   0   0.055
 0   0   1   0.120
 0   0   0   1
```

`agriculture/configs/presets/lab_citrus_robot.json` 的 `robot_demo.d405` 集中保存来源、
工具固定变换、`T_tool_optical`、采集频率、分辨率、量程和示意外壳尺寸。
标定后替换 `T_tool_optical` 并重启；相机内参目前仍使用 VirtualD405 名义模型。
`demo.camera` 通过原生 mount 随实际法兰移动；直接调用 `capture(scene)`，不再重复传入
`T_world_mount`。RGB、depth 和反投影点云来自同一 DepthFrame，世界点云使用该帧保存的位姿。

右侧面板显示 RGB、深度和点云统计，也可叠加世界坐标点云。
点云仅有一个显示对象，刷新时替换；复位清除旧观测。传感器外壳及调试图形被排除在成像之外，
外壳无碰撞/惯量，机械臂仍为 6 个 actuator，植物仍为 11 个 passive DOF。
这提供虚拟腕部观测，不包含真实 D405 驱动、实测外参或视觉伺服。

## 来源

源仓库：[blanketxx/wrs-sealp-assemble](https://github.com/blanketxx/wrs-sealp-assemble/tree/7260d56e7eba7176eabc23063210d341583671ae)，
固定提交 `7260d56e7eba7176eabc23063210d341583671ae`。

- 旧整机：`wrs/robot_sim/robots/robot_panthera_ht/panthera_ht.py`
- 旧机械臂：`wrs/robot_sim/manipulators/panthera_ht/panthera_ht.py`
- 旧夹爪：`wrs/robot_sim/end_effectors/grippers/panthera_gripper/panthera_gripper.py`

导入 6 个机械臂网格（底座、link1～link5）和 3 个夹爪网格。STL 内容保持不变，
扩展名统一为小写；未导入未使用的 link6、glink6 变体或旧控制器。
两个资源包均保留上游 MIT LICENSE，打包配置包含网格与许可证。
