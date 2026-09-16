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

`examples/agriculture/robot_citrus_interaction.py` 使用 **FAFURobotArm + FAFUGripper**，
原生掌部和双指参与 MuJoCo 叶片/果实接触。夹爪保持配置中的固定开口（默认 50 mm）；
不再使用蓝色圆头工具。原始 STL 和 WRS 物理转换器保持不变。

`FAFUGripper(fixed_opening=0.05)` 为该实例生成固定手指关节，保留同一套网格、碰撞与 TCP。
其 `ndof=0`，`jaw_range=[0.05,0.05]`，不能在运行时改变开口；修改参数后重启场景。
不传 `fixed_opening` 时仍是原来的可动运动学模型，`set_opening/open/close` 行为不变。

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
| 碰撞与 MuJoCo 编译 | 裸臂与装配体都能编译；link2 用凸包规避 STL 面数限制。已验证固定开口原生夹爪的动态接触。碰撞包络保守，不能视为精确零件接触几何。 |
| 动力学参数 | **尚未标定**质量、质心、惯量、关节摩擦和电机性能。多数网格连杆未显式设置惯量，由 MuJoCo 从几何及默认密度估计；法兰仅有数值占位惯量。示例伺服增益是定性值，不代表真实 FAFU 响应或额定力矩。 |
| 双指夹爪 | FK/mount/开口和碰撞规划可用；**物理 mimic 联动尚未接通**：当前转换器未输出 joint equality，装配体编译为 8 DOF、8 actuators、0 equality，而 WRS 夹爪运动学只有 1 个独立 DOF。不可直接据此宣称单驱动同步夹持仿真已完成。 |
| 实机控制 | 本次迁移没有导入旧控制器，当前包没有对应的 FAFU 实机通信/执行接口。 |

当前 Citrus 演示使用 6 个机械臂 actuator 和 19 个植物 passive DOF；固定开口夹爪无活动 DOF。
模型可用于定性接触与路径探索；尚不支持真实硬件动力学预测、标定力控或物理双指抓取。
后续实现物理开合需补通用 mimic→MuJoCo 约束/驱动映射，再标定连杆与驱动参数。

验证命令：

```powershell
python -m unittest discover -s tests -p "test_fafu_robot_arm.py" -v
python -m unittest discover -s tests -p "test_robot_citrus_interaction.py" -v
```

## Citrus 中的 VirtualD405 安装

`examples/agriculture/fafu_d405.py` 从 `agriculture/configs/wrist_d405_handeye.json`
读取用户提供的手眼外参。配置入口是 `robot_demo.d405.handeye_file`：相对路径基于
`agriculture/configs`，或使用绝对路径。标定文件随项目保存，不依赖原始微信目录。

文件标记为 **`T_flange_cam`**，约定为：

```text
p_base = T_base_flange @ T_flange_cam @ p_cam
T_world_cam = T_world_flange @ T_flange_cam
```

完整使用 JSON 的旋转和平移，不把它解释为 `T_tcp_cam`。平移为
`[-0.046916616930479806, 0.007351220423231052, 0.050142992752958415]` m；
光学 +Z 大致沿法兰 +Z，光学 +X 大致沿法兰 −Y。
先前根据 URDF 估计的安装位姿已被替换，不再应用 `Ry(-pi/2)` 或工具长度偏移。
loader 检查 frame、convention、刚性变换与重复矩阵的一致性，错误标记会被拒绝。

夹爪抓取 TCP 位于法兰 +Z 前方 0.17 m，与法兰姿态相同。
外部 `ee_state7` 若提供 TCP 的 xyz/rpy，先用项目约定把 rpy 转为 `R_tcp`，再计算：

```python
flange_pos = tcp_pos - R_tcp @ np.array([0, 0, 0.17])
flange_rot = R_tcp
T_base_flange = wum.tf_from_pos_rotmat(flange_pos, flange_rot)
T_base_cam = T_base_flange @ T_flange_cam
```

当前仿真直接读取 `robot.tcp('flange').tf`，无需从 TCP 反推。
相机原生 mount 在此法兰上，`capture(scene)` 不再传入额外的 `T_world_mount`。
相机外壳是无质量/碰撞的示意模型，安装支架尚未建模；掌部与双指仍参与成像和遮挡。

JSON 的 `color_intrinsics.K/dist` 保留在标定文件中，但没有图像分辨率，本例没有猜测缩放关系；
RGB-D 内参暂用 VirtualD405 名义模型。界面区分“calibrated mount / nominal intrinsics”。
已应用的外参以用户提供的 frame 约定为依据，未独立验证实机标定误差。

右侧显示 RGB、深度和点云统计，可叠加世界坐标点云与距离相关深度噪声。
RGB、depth、点云来自同一 DepthFrame，世界点云用该帧保存的位姿。
显示点云、调试图形和传感器自身外壳不参与成像；复位清除旧观测。
机械臂仍为 6 个 actuator，植物为 19 个 passive DOF。
尚未接入真实 D405 驱动或视觉伺服。

## 来源

源仓库：[blanketxx/wrs-sealp-assemble](https://github.com/blanketxx/wrs-sealp-assemble/tree/7260d56e7eba7176eabc23063210d341583671ae)，
固定提交 `7260d56e7eba7176eabc23063210d341583671ae`。

- 旧整机：`wrs/robot_sim/robots/robot_panthera_ht/panthera_ht.py`
- 旧机械臂：`wrs/robot_sim/manipulators/panthera_ht/panthera_ht.py`
- 旧夹爪：`wrs/robot_sim/end_effectors/grippers/panthera_gripper/panthera_gripper.py`

导入 6 个机械臂网格（底座、link1～link5）和 3 个夹爪网格。STL 内容保持不变，
扩展名统一为小写；未导入未使用的 link6、glink6 变体或旧控制器。
两个资源包均保留上游 MIT LICENSE，打包配置包含网格与许可证。
