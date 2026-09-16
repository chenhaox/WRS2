"""Cartesian jogging of a FAFU arm in contact with Citrus V2 foliage/fruit.

TCP pose paths and gripper opening become rate-limited position-servo targets.
The arm and fingers have actuators; the plant retains its passive V2 joints.
The wrist camera switches between raw-like agriculture noise and clean depth.
"""
from pathlib import Path
import argparse
import json
import sys

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import mujoco
import numpy as np
from wrs import wss, wssop, wuc, wum, wvw
from wrs.robots.manipulators.fafu import FAFURobotArm
from wrs.robots.end_effectors.fafu_gripper import FAFUGripper
from wrs.robots.base.kine.numik import NumIKSolver
from wrs.physics.mj_env import MJEnv
from wrs.viewer.web_ui import Anchor
from agriculture.config import CONFIG_DIR, load_config
from agriculture.generator import generate
from agriculture.dynamics import PlantDynamicsSpec
from agriculture.dynamic import DynamicPlantBuilder
from examples.agriculture.fafu_d405 import MountedD405

DEFAULT_CONFIG = CONFIG_DIR / 'presets/lab_citrus_robot.json'


class _ServoGripper(FAFUGripper):
    """Simulate both fingers; equal targets allow asymmetric contact response."""

    _structure = None

    @classmethod
    def _build_structure(cls):
        structure = super()._build_structure()
        # Kinematic mimic would hide one finger's independently simulated pose.
        for joint in structure.jnts:
            joint.mmc = None
        structure.compile()
        return structure


class RobotPlantInteraction:
    """One example scene/controller, also usable in a headless contact smoke test."""

    def __init__(self, config=None):
        config = load_config(DEFAULT_CONFIG) if config is None else config
        self.settings = p = config['robot_demo']
        self.scene = wss.Scene()
        size = np.asarray(p['ground_size'])
        ground = wssop.box(pos=(*p['ground_center_xy'], -size[2] / 2), xyz_lengths=size,
                          rgb=p['ground_color'], collision_type=wuc.CollisionType.AABB,
                          name='ground')
        ground.add_to_scene(self.scene)
        spec = generate(config)
        self.plant = DynamicPlantBuilder(config).build(spec, PlantDynamicsSpec.from_config(spec, config))
        plant_summary = self.plant.summary()
        self.foliage_coverage = {key: plant_summary[key] for key in
            ('foliage_contact_leaf_count', 'visual_only_leaf_count', 'foliage_proxy_count', 'foliage_proxy_object_count')}
        self.plant.add_to_scene(self.scene)
        # Pick the demonstration poses from the gravity-loaded tree, not q=0.
        warmup = MJEnv(self.scene, require_ctrl=True)
        warmup.step(p['settle_seconds'])
        # NumIK uses explicit neighbouring seeds, with no SELIK database writes.
        self.robot = FAFURobotArm(pos=p['base_position'], solver=NumIKSolver,
                                  rotmat=wum.rotmat_from_axangle((0, 0, 1), np.deg2rad(p['base_yaw_deg'])))
        compiled = self.robot.structure.compiled
        self.limits = np.array([compiled.jlmt_low_by_idx, compiled.jlmt_high_by_idx], dtype=float)
        if p['base_position'][2] > 0:
            pedestal = wssop.box(pos=(*p['base_position'][:2], p['base_position'][2] / 2),
                xyz_lengths=(p['pedestal_width'], p['pedestal_width'], p['base_position'][2]),
                rgb=p['pedestal_color'], collision_type=wuc.CollisionType.AABB, name='arm_pedestal')
            pedestal.add_to_scene(self.scene)
        self.gripper = _ServoGripper()
        self.gripper.set_opening(p['gripper']['opening_m'])
        self.robot.mount(self.gripper, self.robot.tcp('flange').parent_lnk, update=True)
        self.tcp = self.gripper.tcp('grasp_center')
        self.rgbd = MountedD405(self.robot, self.scene, p['d405'], exclude=self.plant.foliage_proxy_visuals)
        self.camera = self.rgbd.camera
        self.presets = self._make_presets()
        self._ready_q = self._solve_tcp(self.presets['ready'], np.deg2rad(p['ik_seed_deg']))
        self.robot.fk(self._ready_q)
        self.robot.add_to_scene(self.scene)
        # Palm and the wrist housing meet at their mechanical mounting surface.
        excludes = [(self.gripper.runtime_root_lnk, self.robot.runtime_lnks[-2])]
        self.env = MJEnv(self.scene, require_ctrl=True, extra_excludes=excludes)
        model = self.env.model
        # Explicit mapping: scene insertion order is not an actuator ordering API.
        joint_ids = [model.joint(self.env.sync.mecj2jnt[self.robot, i].name).id
                     for i in range(self.robot.ndof)]
        self.actuators = np.array([np.flatnonzero(model.actuator_trnid[:, 0] == jid).item()
                                   for jid in joint_ids])
        finger_ids = [model.joint(self.env.sync.mecj2jnt[self.gripper, i].name).id
                      for i in range(self.gripper.ndof)]
        self.gripper_actuators = np.array([np.flatnonzero(model.actuator_trnid[:, 0] == jid).item()
                                          for jid in finger_ids])
        if model.nu != len(self.actuators) + len(self.gripper_actuators):
            raise RuntimeError('Only the arm and fingers may have actuators')
        # Example-specific native position-servo tuning; no changes to robot defaults.
        model.actuator_gainprm[self.actuators, 0] = p['servo_kp']
        model.actuator_biasprm[self.actuators, 1] = -np.asarray(p['servo_kp'])
        model.actuator_biasprm[self.actuators, 2] = -np.asarray(p['servo_kv'])
        model.actuator_gainprm[self.gripper_actuators, 0] = 800
        model.actuator_biasprm[self.gripper_actuators, 1:3] = [-800, -10]
        model.actuator_forcelimited[self.gripper_actuators] = True
        model.actuator_forcerange[self.gripper_actuators] = [-20, 20]
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.command = self._ready_q.copy()
        self.command_position = self.presets['ready'].copy()
        self.command_rotation = self.tool_rotation.copy()
        self.gripper_command = p['gripper']['opening_m']
        self.stop_motion()
        self.env.ctrl[self.actuators] = self.command
        self.env.ctrl[self.gripper_actuators] = self.gripper_command / 2
        self.paused = False
        self.show_forces = False
        self._remainder = 0.0
        self.robot_bodies = {model.body(self.env.sync.rutl2bdy[link].name).id
                             for link in (*self.robot.runtime_lnks, *self.gripper.runtime_lnks)}
        self.plant_bodies = {}
        for role, objects in self.plant.semantic_groups.items():
            for obj in objects:
                node = self.env.sync.rutl2bdy.get(obj) or self.env.sync.sobj2bdy.get(obj)
                self.plant_bodies[model.body(node.name).id] = role
        self.contacts = {}
        self.contact_samples = {}
        self.step(p['settle_seconds'])
        self._state_kind = mujoco.mjtState.mjSTATE_INTEGRATION
        self._initial_state = np.empty(mujoco.mj_stateSize(model, self._state_kind))
        mujoco.mj_getState(model, self.env.data, self._initial_state, self._state_kind)
        self._initial_time = self.env.data.time
        self._initial_fruit = {key: obj.pos.copy() for key, obj in self.plant.fruits.items()}
        self._initial_plant_q = self.plant.mech.qs.copy()
        self.reset()

    def _make_presets(self):
        p = self.settings
        direction = np.asarray(p['approach_direction'], dtype=float)
        direction /= np.linalg.norm(direction)
        self.tool_rotation = (wum.rotmat_from_normal(direction) @
                              wum.rotmat_from_axangle((0, 0, 1), np.deg2rad(p['tool_roll_deg'])))
        fruit = self.plant.fruits[p['fruit_id']]
        radius = next(f.radius for f in self.plant.spec.fruits if f.id == p['fruit_id'])
        proxy = self.plant.foliage_proxies[p['foliage_cluster']][p['foliage_proxy_index']]
        shape = proxy.collisions[p['foliage_shape_index']]
        shape_tf = proxy.tf @ shape.loc_tf
        extent = np.sum(np.abs(direction @ shape_tf[:3, :3]) * shape.half_extents)
        # Aim a real fingertip, not the empty grasp centre, at the surface.
        # Native finger geometry is measured in gripper-local coordinates here.
        finger = next(link for link in self.gripper.runtime_lnks
                      if link.name == p['gripper']['contact_link'])
        vertices = []
        for visual in finger.visuals:
            tf = np.linalg.inv(self.gripper.tf) @ finger.tf @ visual.loc_tf
            vertices.append(visual.geom.vs @ tf[:3, :3].T + tf[:3, 3])
        vertices = np.concatenate(vertices)
        tip_z = vertices[:, 2].max()
        tip = vertices[vertices[:, 2] >= tip_z - p['gripper']['contact_tip_band_m']].mean(axis=0)
        tip[2] = tip_z
        self.contact_point_tcp = tip - self.tcp.loc_tf[:3, 3]
        tip_offset = self.tool_rotation @ self.contact_point_tcp
        positions = dict(ready=fruit.pos + p['ready_offset'],
                         foliage=shape_tf[:3, 3] - direction * (extent - p['foliage_push_depth']) - tip_offset,
                         fruit=fruit.pos - direction * (radius - p['fruit_push_depth']) - tip_offset)
        return {key: np.asarray(value, dtype=float) for key, value in positions.items()}

    def _solve_tcp(self, position, seed, rotation=None):
        rotation = self.tool_rotation if rotation is None else rotation
        solutions = self.robot.ik(position, rotation, tcp=self.tcp,
                                  qs_active_init=seed, max_iter=self.settings['ik_max_iter'])
        if not solutions:
            raise ValueError('No nearby IK solution for this TCP pose')
        q = np.asarray(solutions[0], dtype=float)
        if (not np.isfinite(q).all() or np.any(q < self.limits[0]) or np.any(q > self.limits[1])):
            raise ValueError('IK solution exceeds joint limits')
        return q

    def set_cartesian_target(self, position, rotation=None):
        """Check the entire straight TCP path before changing any command/state.

        Short Cartesian waypoints keep seeded IK on the same branch. Linear
        interpolation between their joint solutions approximates the straight
        path while enforcing both Cartesian and joint command speed limits.
        This is not an obstacle-avoiding planner: contact is the purpose here.
        """
        p = self.settings
        goal = np.asarray(position, dtype=float)
        if goal.shape != (3,) or not np.isfinite(goal).all():
            raise ValueError('Expected three finite world coordinates in metres')
        if np.any(goal < p['workspace_min']) or np.any(goal > p['workspace_max']):
            raise ValueError('Target outside the configured jogging workspace')
        rotation = np.asarray(self.target_rotation if rotation is None else rotation, dtype=float)
        if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
                or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
            raise ValueError('Expected a 3 by 3 rotation matrix')
        distance = np.linalg.norm(goal - self.command_position)
        angle = np.linalg.norm(wum.delta_rotvec_between_rotmats(self.command_rotation, rotation))
        if distance < 1e-9 and angle < 1e-9:
            self.stop_motion()
            return
        n = max(1, int(np.ceil(distance / p['cartesian_waypoint_m'])),
                int(np.ceil(angle / np.deg2rad(2))))
        points = np.linspace(self.command_position, goal, n + 1)
        rotations = wum.rotmat_slerp(self.command_rotation, rotation, n + 1)
        angles, times = [self.command.copy()], [0.0]
        for i, point in enumerate(points[1:], 1):
            q = self._solve_tcp(point, angles[-1], rotations[i])
            max_delta = float(np.max(np.abs(q - angles[-1])))
            if max_delta > np.deg2rad(p['ik_max_step_deg']):
                raise ValueError('IK jump near a singularity; try a smaller move or another direction')
            duration = max(np.linalg.norm(point - points[i - 1]) / p['cartesian_speed_m_s'],
                           angle / n / np.deg2rad(20),
                           max_delta / np.deg2rad(p['joint_speed_deg_s']))
            angles.append(q)
            times.append(times[-1] + duration)
        self._path_times = np.asarray(times)
        self._path_angles = np.asarray(angles)
        self._path_positions = points
        self._path_rotations = rotations
        self._path_rotvecs = np.array([wum.delta_rotvec_between_rotmats(a, b)
                                      for a, b in zip(rotations[:-1], rotations[1:])])
        self._motion_elapsed = 0.0
        self.target = angles[-1].copy()
        self.target_position = goal.copy()
        self.target_rotation = rotation.copy()
        self.motion_status = 'Moving'

    @property
    def motion_duration(self):
        return float(self._path_times[-1])

    def jog(self, axis, distance):
        if axis not in (0, 1, 2) or not np.isfinite(distance):
            raise ValueError('Jog requires axis 0/1/2 and a finite distance')
        # Keep held input at most one step ahead instead of building a long path.
        position = self.command_position.copy()
        position[axis] += distance
        self.set_cartesian_target(position, self.command_rotation)

    def jog_rotation(self, axis, angle):
        """Rotate about a world axis at the current TCP position (radians)."""
        if axis not in (0, 1, 2) or not np.isfinite(angle):
            raise ValueError('Rotation requires axis 0/1/2 and a finite angle')
        rotation = wum.rotmat_from_axangle(np.eye(3)[axis], angle) @ self.command_rotation
        self.set_cartesian_target(self.command_position, rotation)

    def set_gripper_opening(self, width):
        """Set the total opening in metres; native servos move both fingers."""
        if not np.isfinite(width) or not self.gripper.jaw_range[0] <= width <= self.gripper.jaw_range[1]:
            raise ValueError('Gripper opening must be between 0 and 0.085 metres')
        self.gripper_target = float(width)

    def stop_motion(self):
        """Hold the current servo command without teleporting the physical arm."""
        self.target = self.command.copy()
        self.target_position = self.command_position.copy()
        self.target_rotation = self.command_rotation.copy()
        self.gripper_target = self.gripper_command
        self._path_times = np.array([0.0])
        self._path_angles = self.command[None, :].copy()
        self._path_positions = self.command_position[None, :].copy()
        self._path_rotations = self.command_rotation[None, :, :].copy()
        self._path_rotvecs = np.empty((0, 3))
        self._motion_elapsed = 0.0
        self.motion_status = 'Holding target'

    def _advance_command(self, dt):
        if self._motion_elapsed >= self.motion_duration:
            return
        self._motion_elapsed = min(self.motion_duration, self._motion_elapsed + dt)
        i = min(int(np.searchsorted(self._path_times, self._motion_elapsed, side='right')),
                len(self._path_times) - 1)
        u = ((self._motion_elapsed - self._path_times[i - 1]) /
             (self._path_times[i] - self._path_times[i - 1]))
        self.command = (1 - u) * self._path_angles[i - 1] + u * self._path_angles[i]
        self.command_position = (1 - u) * self._path_positions[i - 1] + u * self._path_positions[i]
        self.command_rotation = wum.rotmat_from_rotvec(u * self._path_rotvecs[i - 1]) @ self._path_rotations[i - 1]
        if self._motion_elapsed >= self.motion_duration:
            self.motion_status = 'Holding target'

    def select_pose(self, name):
        self.set_cartesian_target(self.presets[name], self.tool_rotation)

    def _read_contacts(self):
        counts = {}
        for contact in self.env.data.contact:
            a, b = (int(self.env.model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2))
            plant_body = b if a in self.robot_bodies else a if b in self.robot_bodies else None
            role = self.plant_bodies.get(plant_body)
            if role:
                counts[role] = counts.get(role, 0) + 1
        self.contacts = counts
        for role, count in counts.items():
            self.contact_samples[role] = self.contact_samples.get(role, 0) + count

    def _sync_scene(self):
        self.env.sync.pull_all_sobj_pose()
        self.env.sync.pull_all_mecba_qpos()
        if self.show_forces:
            self.env.contact_viz.update_from_data(self.env.model, self.env.data)
        else:
            self.env.contact_viz.clear()

    def step(self, dt):
        if not np.isfinite(dt) or dt < 0:
            raise ValueError('dt must be finite and nonnegative')
        if self.paused:
            return
        self._remainder += dt
        h = self.env.get_timestep()
        n = int((self._remainder + 1e-12) / h)
        self._remainder -= n * h
        for _ in range(n):
            self._advance_command(h)
            self.env.ctrl[self.actuators] = self.command
            # 40 mm/s total opening, with 20 N force limits on each finger.
            self.gripper_command += np.clip(self.gripper_target - self.gripper_command, -.04 * h, .04 * h)
            self.env.ctrl[self.gripper_actuators] = self.gripper_command / 2
            self.env.runtime.step()
            self._read_contacts()
        # Physics runs at its native timestep; publish rigid mounts once per frame.
        self._sync_scene()

    def reset(self):
        mujoco.mj_setState(self.env.model, self.env.data, self._initial_state, self._state_kind)
        self.env.runtime.forward()
        self.command = self._ready_q.copy()
        self.command_position = self.presets['ready'].copy()
        self.command_rotation = self.tool_rotation.copy()
        self.gripper_command = self.settings['gripper']['opening_m']
        self.stop_motion()
        self._remainder = 0.0
        self.contacts, self.contact_samples = {}, {}
        self._sync_scene()
        self.rgbd.invalidate()

    def capture_rgbd(self, dt=0):
        return self.rgbd.capture(simulation_time=float(self.env.data.time - self._initial_time))

    def close(self):
        self.rgbd.close()

    def report(self):
        return dict(simulated_seconds=float(self.env.data.time - self._initial_time),
                    robot_actuators=len(self.actuators), plant_actuators=0,
                    contacts=self.contacts, contact_samples=self.contact_samples.copy(),
                    foliage_coverage=self.foliage_coverage.copy(),
                    joint_target_deg=np.rad2deg(self.target).tolist(),
                    joint_actual_deg=np.rad2deg(self.robot.qs).tolist(),
                    tcp_target_m=self.target_position.tolist(), tcp_actual_m=self.tcp.pos.tolist(),
                    gripper_opening_m=float(np.sum(self.gripper.qs)), gripper_target_m=self.gripper_target,
                    gripper_actuators=len(self.gripper_actuators), gripper_mode='position_servo',
                    T_flange_cam=self.rgbd.mount_tf.tolist(),
                    motion_status=self.motion_status,
                    plant_deflection_rad=float(np.linalg.norm(self.plant.mech.qs - self._initial_plant_q)),
                    fruit_displacement_m={k: float(np.linalg.norm(o.pos - self._initial_fruit[k]))
                                          for k, o in self.plant.fruits.items()},
                    camera=self.rgbd.statistics())


def add_controls(base, demo):
    arm = base.ui.add_panel('arm', title='FAFU Cartesian jog', anchor=Anchor.TOP_LEFT,
                            width=300, columns=6, font_size=12, movable=True,
                            description='Hold a key or button to jog. World axes: +Y toward the tree, +X right, +Z up.')
    status = base.ui.add_panel('interaction', title='VirtualD405 / Citrus contact', anchor=Anchor.TOP_RIGHT,
                               width=340, font_size=12, movable=True,
                               description='Live wrist RGB-D. Switch Clean / Noise to compare depth. Black depth = invalid/out of range.')
    demo.rgbd.add_controls(status)
    jog_step = [demo.settings['jog_step_m']]
    rotation_step = [2.0]

    def request(action):
        try:
            action()
        except ValueError as error:
            # A rejected path never replaces the previous valid target/path.
            arm.set_value('request', f'Not applied: {error}')
            raise  # Let the button stop repeating on a rejected move.
        else:
            arm.set_value('request', 'Applied')
        refresh(0)

    def set_step(value):
        jog_step[0] = value / 1000

    arm.add_slider('step', label='Move per step', unit='mm', min_value=1, max_value=30, group='Move',
                   step=1, value=jog_step[0] * 1000, on_change=set_step)
    for key, label, axis, sign, shortcut in [
            ('up', 'Up +Z', 2, 1, 'q'), ('forward', 'Forward +Y', 1, 1, 'w'), ('down', 'Down −Z', 2, -1, 'e'),
            ('left', 'Left −X', 0, -1, 'a'), ('backward', 'Back −Y', 1, -1, 's'), ('right', 'Right +X', 0, 1, 'd')]:
        arm.add_button(key, label=label, group='Move', variant='keycap', column_span=2,
                       repeat=True, repeat_hz=10, shortcut=shortcut,
                       on_click=lambda axis=axis, sign=sign: request(lambda: demo.jog(axis, sign * jog_step[0])))

    def set_rotation_step(value):
        rotation_step[0] = value

    arm.add_slider('rotation_step', label='Rotate per step', unit='°', min_value=1, max_value=10,
                   step=1, value=rotation_step[0], group='Rotate',
                   on_change=set_rotation_step)
    for axis, sign, shortcut in [(0, 1, 'i'), (1, 1, 'j'), (2, 1, 'u'),
                                 (0, -1, 'k'), (1, -1, 'l'), (2, -1, 'o')]:
        arm.add_button(f'rotate_{shortcut}', label=f'{"XYZ"[axis]} {"+" if sign > 0 else "−"}',
                       group='Rotate', variant='keycap', column_span=2, repeat=True, repeat_hz=10,
                       shortcut=shortcut, on_click=lambda axis=axis, sign=sign: request(
                           lambda: demo.jog_rotation(axis, np.deg2rad(sign * rotation_step[0]))))

    arm.add_label('gripper', label='Opening · actual / target', group='Gripper')
    arm.add_button('open', label='Open', group='Gripper', variant='keycap', column_span=3, shortcut='r',
                   on_click=lambda: request(lambda: demo.set_gripper_opening(demo.gripper.jaw_range[1])))
    arm.add_button('close', label='Close', group='Gripper', variant='keycap', column_span=3, shortcut='f',
                   on_click=lambda: request(lambda: demo.set_gripper_opening(demo.gripper.jaw_range[0])))

    def reset():
        demo.reset()
        arm.set_value('request', 'Reset')
        refresh(0)

    def proxies(checked):
        demo.plant.show_foliage_proxies(checked)

    def collision(checked):
        demo.robot.toggle_render_collision = checked
        for obj in (*demo.plant.branch_objects, *demo.plant.fruit_objects):
            obj.toggle_render_collision = checked

    def forces(checked):
        demo.show_forces = checked
        demo._sync_scene()

    arm.add_button('stop', label='Stop', group='Actions', column_span=3, shortcut='Escape',
                   on_click=lambda: request(demo.stop_motion))
    arm.add_button('ready', label='Retract / ready', group='Actions', column_span=3,
                   on_click=lambda: request(lambda: demo.select_pose('ready')))
    arm.add_button('foliage', label='Touch leaves', group='Actions', column_span=3,
                   on_click=lambda: request(lambda: demo.select_pose('foliage')))
    arm.add_button('fruit', label=f'Touch {demo.settings["fruit_id"]}', group='Actions', column_span=3,
                   on_click=lambda: request(lambda: demo.select_pose('fruit')))
    arm.add_button('reset', label='Reset robot and tree', group='Actions', on_click=reset)
    arm.add_label('request', label='Last request', value='Ready')
    status.add_checkbox('paused', label='Pause physics', on_change=lambda value: setattr(demo, 'paused', value))
    status.add_checkbox('proxies', label='Show foliage contact proxies', on_change=proxies)
    coverage = demo.foliage_coverage
    status.add_label('foliage_coverage', label='Foliage contact coverage',
        value=f'{coverage["foliage_contact_leaf_count"]:,} contact leaves / '
              f'{coverage["visual_only_leaf_count"]:,} visual-only\n'
              f'{coverage["foliage_proxy_object_count"]} compound objects · '
              f'{demo.plant.dynamics.foliage_proxy.sections_per_leaf} strips per leaf')
    status.add_checkbox('collisions', label='Show arm / branch / fruit collisions', on_change=collision)
    status.add_checkbox('forces', label='Show contact force arrows', on_change=forces)
    for key, label in [('time', 'Simulation'), ('motion', 'Motion'),
                       ('target', 'Target TCP · X / Y / Z metres'), ('actual', 'Actual TCP · X / Y / Z metres'),
                       ('contacts', 'Arm ↔ plant contact'), ('bend', 'Plant deflection'),
                       ('fruit_position', f'{demo.settings["fruit_id"]} · world metres')]:
        status.add_label(key, label=label)

    def refresh(dt):
        report = demo.report()
        arm.set_value('gripper', f'{report["gripper_opening_m"] * 1000:.0f} / '
                                f'{report["gripper_target_m"] * 1000:.0f} mm')
        status.set_value('time', f'{report["simulated_seconds"]:.1f} s' + (' · paused' if demo.paused else ''))
        status.set_value('motion', report['motion_status'])
        status.set_value('target', ' / '.join(f'{x:.3f}' for x in report['tcp_target_m']))
        status.set_value('actual', ' / '.join(f'{x:.3f}' for x in report['tcp_actual_m']))
        status.set_value('contacts', ', '.join(f'{k}: {v}' for k, v in demo.contacts.items()) or 'No contact')
        status.set_value('bend', f'{report["plant_deflection_rad"]:.3f} rad')
        status.set_value('fruit_position', ' / '.join(f'{x:.3f}' for x in demo.plant.fruits[demo.settings['fruit_id']].pos))

    refresh(0)
    base.schedule_interval(refresh, 1 / demo.settings['status_hz'])
    demo.capture_rgbd()
    base.schedule_interval(demo.capture_rgbd, 1 / demo.camera.fps)


def run_smoke(demo, *, capture_rgbd=False):
    results = {}
    for name in ('foliage', 'fruit'):
        demo.reset()
        demo.select_pose(name)
        travel = demo.motion_duration
        demo.step(travel + demo.settings['smoke_hold_seconds'])
        if capture_rgbd:
            demo.capture_rgbd()
        touch = demo.report()
        demo.select_pose('ready')
        demo.step(demo.motion_duration + demo.settings['smoke_release_seconds'])
        if capture_rgbd:
            demo.capture_rgbd()
        results[name] = dict(touch=touch, released=demo.report())
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--duration', type=float, help='Close viewer after this many wall-clock seconds')
    parser.add_argument('--headless', action='store_true', help='Run both contact presets and release without a viewer')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if args.duration is not None and (not np.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be finite and positive')
    demo = RobotPlantInteraction(load_config(args.config))
    try:
        if args.headless:
            report = run_smoke(demo, capture_rgbd=True)
        else:
            p = demo.settings
            base = wvw.World(cam_pos=p['camera_pos'], cam_lookat_pos=p['camera_lookat'], port=args.port)
            base.set_scene(demo.scene)
            base.set_caption('FAFU / Citrus V2 / VirtualD405')
            add_controls(base, demo)
            base.schedule_interval(lambda dt: demo.step(min(dt, p['max_frame_dt'])), 1 / p['control_hz'])
            if args.duration is not None:
                base.schedule_once(lambda dt: base.close(), args.duration)
            base.run()
            report = demo.report()
    finally:
        demo.close()
    print(json.dumps(report, indent=2), flush=True)
    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
