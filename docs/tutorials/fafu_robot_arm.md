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

## 来源

源仓库：[blanketxx/wrs-sealp-assemble](https://github.com/blanketxx/wrs-sealp-assemble/tree/7260d56e7eba7176eabc23063210d341583671ae)，
固定提交 `7260d56e7eba7176eabc23063210d341583671ae`。

- 旧整机：`wrs/robot_sim/robots/robot_panthera_ht/panthera_ht.py`
- 旧机械臂：`wrs/robot_sim/manipulators/panthera_ht/panthera_ht.py`
- 旧夹爪：`wrs/robot_sim/end_effectors/grippers/panthera_gripper/panthera_gripper.py`

导入 6 个机械臂网格（底座、link1～link5）和 3 个夹爪网格。STL 内容保持不变，
扩展名统一为小写；未导入未使用的 link6、glink6 变体或旧控制器。
两个资源包均保留上游 MIT LICENSE，打包配置包含网格与许可证。
