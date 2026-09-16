"""Cartesian jogging of a FAFU arm in contact with Citrus V2 foliage/fruit.

Fixed-orientation TCP paths become rate-limited native position-servo targets.
Only the arm has actuators; the plant retains its passive V2 joints.
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
        self.gripper = FAFUGripper(fixed_opening=p['gripper']['opening_m'])
        self.robot.mount(self.gripper, self.robot.tcp('flange').parent_lnk, update=True)
        self.tcp = self.gripper.tcp('grasp_center')
        self.rgbd = MountedD405(self.robot, self.scene, p['d405'])
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
        if model.nu != self.robot.ndof:
            raise RuntimeError('Only the six arm joints may have actuators')
        # Example-specific native position-servo tuning; no changes to robot defaults.
        model.actuator_gainprm[self.actuators, 0] = p['servo_kp']
        model.actuator_biasprm[self.actuators, 1] = -np.asarray(p['servo_kp'])
        model.actuator_biasprm[self.actuators, 2] = -np.asarray(p['servo_kv'])
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        self.command = self._ready_q.copy()
        self.command_position = self.presets['ready'].copy()
        self.stop_motion()
        self.env.ctrl[self.actuators] = self.command
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

    def _solve_tcp(self, position, seed):
        solutions = self.robot.ik(position, self.tool_rotation, tcp=self.tcp,
                                  qs_active_init=seed, max_iter=self.settings['ik_max_iter'])
        if not solutions:
            raise ValueError('No nearby IK solution at the fixed tool orientation')
        q = np.asarray(solutions[0], dtype=float)
        if (not np.isfinite(q).all() or np.any(q < self.limits[0]) or np.any(q > self.limits[1])):
            raise ValueError('IK solution exceeds joint limits')
        return q

    def set_cartesian_target(self, position):
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
        distance = np.linalg.norm(goal - self.command_position)
        if distance < 1e-9:
            self.stop_motion()
            return
        n = max(1, int(np.ceil(distance / p['cartesian_waypoint_m'])))
        points = np.linspace(self.command_position, goal, n + 1)
        angles, times = [self.command.copy()], [0.0]
        for i, point in enumerate(points[1:], 1):
            q = self._solve_tcp(point, angles[-1])
            max_delta = float(np.max(np.abs(q - angles[-1])))
            if max_delta > np.deg2rad(p['ik_max_step_deg']):
                raise ValueError('IK jump near a singularity; try a smaller move or another direction')
            duration = max(np.linalg.norm(point - points[i - 1]) / p['cartesian_speed_m_s'],
                           max_delta / np.deg2rad(p['joint_speed_deg_s']))
            angles.append(q)
            times.append(times[-1] + duration)
        self._path_times = np.asarray(times)
        self._path_angles = np.asarray(angles)
        self._path_positions = points
        self._motion_elapsed = 0.0
        self.target = angles[-1].copy()
        self.target_position = goal.copy()
        self.motion_status = 'Moving'

    @property
    def motion_duration(self):
        return float(self._path_times[-1])

    def jog(self, axis, distance):
        if axis not in (0, 1, 2) or not np.isfinite(distance):
            raise ValueError('Jog requires axis 0/1/2 and a finite distance')
        position = self.target_position.copy()
        position[axis] += distance
        self.set_cartesian_target(position)

    def stop_motion(self):
        """Hold the current servo command without teleporting the physical arm."""
        self.target = self.command.copy()
        self.target_position = self.command_position.copy()
        self._path_times = np.array([0.0])
        self._path_angles = self.command[None, :].copy()
        self._path_positions = self.command_position[None, :].copy()
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
        if self._motion_elapsed >= self.motion_duration:
            self.motion_status = 'Holding target'

    def select_pose(self, name):
        self.set_cartesian_target(self.presets[name])

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
            self.env.runtime.step()
            self._read_contacts()
        # Physics runs at its native timestep; publish rigid mounts once per frame.
        self._sync_scene()

    def reset(self):
        mujoco.mj_setState(self.env.model, self.env.data, self._initial_state, self._state_kind)
        self.env.runtime.forward()
        self.command = self._ready_q.copy()
        self.command_position = self.presets['ready'].copy()
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
                    joint_target_deg=np.rad2deg(self.target).tolist(),
                    joint_actual_deg=np.rad2deg(self.robot.qs).tolist(),
                    tcp_target_m=self.target_position.tolist(), tcp_actual_m=self.tcp.pos.tolist(),
                    gripper_opening_m=self.settings['gripper']['opening_m'], gripper_mode='fixed_opening',
                    T_flange_cam=self.rgbd.mount_tf.tolist(),
                    motion_status=self.motion_status,
                    plant_deflection_rad=float(np.linalg.norm(self.plant.mech.qs - self._initial_plant_q)),
                    fruit_displacement_m={k: float(np.linalg.norm(o.pos - self._initial_fruit[k]))
                                          for k, o in self.plant.fruits.items()},
                    camera=self.rgbd.statistics())


def add_controls(base, demo):
    arm = base.ui.add_panel('arm', title='FAFU Cartesian jog', anchor=Anchor.TOP_LEFT,
                            width=285, font_size=12, movable=True,
                            description='World axes: +Y toward the tree, +X right, +Z up. Tool orientation stays fixed.')
    status = base.ui.add_panel('interaction', title='VirtualD405 / Citrus contact', anchor=Anchor.TOP_RIGHT,
                               width=340, font_size=12, movable=True,
                               description='Live wrist RGB-D. Scroll for contact details. Black depth = invalid/out of range.')
    demo.rgbd.add_controls(status)
    jog_step = [demo.settings['jog_step_m']]

    def request(action):
        try:
            action()
        except ValueError as error:
            # A rejected path never replaces the previous valid target/path.
            arm.set_value('request', f'Not applied: {error}')
        else:
            arm.set_value('request', 'Applied')
        refresh(0)

    def set_step(value):
        jog_step[0] = value / 1000

    arm.add_slider('step', label='Move per click', unit='mm', min_value=1, max_value=30,
                   step=1, value=jog_step[0] * 1000, on_change=set_step)
    for key, label, axis, sign in [('forward', 'Forward +Y', 1, 1), ('backward', 'Backward −Y', 1, -1),
                                  ('left', 'Left −X', 0, -1), ('right', 'Right +X', 0, 1),
                                  ('up', 'Up +Z', 2, 1), ('down', 'Down −Z', 2, -1)]:
        arm.add_button(key, label=label,
                       on_click=lambda axis=axis, sign=sign: request(lambda: demo.jog(axis, sign * jog_step[0])))

    def reset():
        demo.reset()
        arm.set_value('request', 'Reset')
        refresh(0)

    def proxies(checked):
        for group in demo.plant.foliage_proxies.values():
            for obj in group:
                obj.toggle_render_collision = checked

    def collision(checked):
        demo.robot.toggle_render_collision = checked
        for obj in (*demo.plant.branch_objects, *demo.plant.fruit_objects):
            obj.toggle_render_collision = checked

    def forces(checked):
        demo.show_forces = checked
        demo._sync_scene()

    arm.add_button('stop', label='Stop motion / hold', on_click=lambda: request(demo.stop_motion))
    arm.add_button('ready', label='Retract / ready', on_click=lambda: request(lambda: demo.select_pose('ready')))
    arm.add_button('foliage', label='Touch leaves', on_click=lambda: request(lambda: demo.select_pose('foliage')))
    arm.add_button('fruit', label=f'Touch {demo.settings["fruit_id"]}', on_click=lambda: request(lambda: demo.select_pose('fruit')))
    arm.add_button('reset', label='Reset robot and tree', on_click=reset)
    arm.add_label('gripper', label='FAFUGripper · fixed opening',
                  value=f'{demo.settings["gripper"]["opening_m"] * 1000:.0f} mm · TCP at flange +Z 170 mm')
    arm.add_label('request', label='Last request', value='Ready')
    status.add_checkbox('paused', label='Pause physics', on_change=lambda value: setattr(demo, 'paused', value))
    status.add_checkbox('proxies', label='Show foliage contact proxies', on_change=proxies)
    status.add_checkbox('collisions', label='Show arm / branch / fruit collisions', on_change=collision)
    status.add_checkbox('forces', label='Show contact force arrows', on_change=forces)
    for key, label in [('time', 'Simulation'), ('motion', 'Motion'),
                       ('target', 'Target TCP · X / Y / Z metres'), ('actual', 'Actual TCP · X / Y / Z metres'),
                       ('contacts', 'Arm ↔ plant contact'), ('bend', 'Plant deflection'),
                       ('fruit_position', f'{demo.settings["fruit_id"]} · world metres')]:
        status.add_label(key, label=label)

    def refresh(dt):
        report = demo.report()
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
