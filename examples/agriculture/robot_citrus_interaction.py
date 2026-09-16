"""Move an RS007L with joint sliders and physically touch Citrus V2 foliage/fruit.

UI values are position-servo targets, never FK teleports into the canopy.
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
from wrs import khi_rs007l, wss, wssop, wuc, wum, wvw
from wrs.physics.mj_env import MJEnv
from wrs.viewer.web_ui import Anchor
from agriculture.config import CONFIG_DIR, load_config
from agriculture.generator import generate
from agriculture.dynamics import PlantDynamicsSpec
from agriculture.dynamic import DynamicPlantBuilder

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
        self.robot = khi_rs007l.RS007L(pos=p['base_position'])
        self.tool = wssop.icosphere(radius=p['tool_radius'], rgb=p['tool_color'],
                                   mass=p['tool_mass'], collision_type=wuc.CollisionType.SPHERE,
                                   name='rounded_contact_tool')
        self.shaft = wssop.cylinder(epos=(0, 0, p['tool_length']), radius=p['shaft_radius'],
                                   rgb=p['tool_color'], mass=p['shaft_mass'],
                                   collision_type=wuc.CollisionType.CAPSULE, name='contact_tool_shaft')
        # Mounting changes pose ownership, not the primitive's default STATIC role.
        for obj in (self.tool, self.shaft):
            obj.collision_group = wuc.CollisionGroup.ACTIVE
        self.robot.mount(self.shaft, self.robot.runtime_lnks[-1], update=True)
        self.robot.mount(self.tool, self.robot.runtime_lnks[-1],
                         wum.tf_from_pos_rotmat((0, 0, p['tool_length'])), update=True)
        self.presets = self._make_presets()
        self.robot.fk(self.presets['ready'])
        self.robot.add_to_scene(self.scene)
        excludes = [(obj, link) for obj in (self.tool, self.shaft) for link in self.robot.runtime_lnks[-2:]]
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
        compiled = self.robot.structure.compiled
        self.limits = np.array([compiled.jlmt_low_by_idx, compiled.jlmt_high_by_idx], dtype=float)
        self.target = self.presets['ready'].copy()
        self.command = self.target.copy()
        self.env.ctrl[self.actuators] = self.command
        self.paused = False
        self.show_forces = False
        self._remainder = 0.0
        self.robot_bodies = {model.body(self.env.sync.rutl2bdy[link].name).id
                             for link in self.robot.runtime_lnks}
        self.robot_bodies.update(model.body(self.env.sync.sobj2bdy[obj].name).id
                                 for obj in (self.tool, self.shaft))
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
        rotation = wum.rotmat_from_normal(direction)
        fruit = self.plant.fruits[p['fruit_id']]
        radius = next(f.radius for f in self.plant.spec.fruits if f.id == p['fruit_id'])
        proxy = self.plant.foliage_proxies[p['foliage_cluster']][p['foliage_proxy_index']]
        extent = np.sum(np.abs(direction @ proxy.rotmat) * proxy.collisions[0].half_extents)
        positions = dict(ready=fruit.pos + p['ready_offset'],
                         foliage=proxy.pos - direction * (extent + p['tool_radius'] - p['foliage_push_depth']),
                         fruit=fruit.pos - direction * (radius + p['tool_radius'] - p['fruit_push_depth']))
        reference = np.deg2rad(p['ik_seed_deg'])
        result = {}
        for name, point in positions.items():
            solutions = self.robot.ik(point - rotation[:, 2] * p['tool_length'], rotation)
            if not solutions:
                raise ValueError(f'{name} pose is unreachable; adjust robot_demo base/target settings')
            # Select the same elbow branch for these nearby poses, without modifying the robot.
            result[name] = min(solutions, key=lambda q: np.linalg.norm(q - reference)).astype(float)
            if name == 'ready':
                reference = result[name]
        return result

    def set_target(self, angles):
        angles = np.asarray(angles, dtype=float)
        if angles.shape != self.target.shape or not np.isfinite(angles).all():
            raise ValueError('Expected six finite joint angles in radians')
        self.target = np.clip(angles, *self.limits)

    def set_joint_degrees(self, index, degrees):
        angles = self.target.copy()
        angles[index] = np.deg2rad(degrees)
        self.set_target(angles)

    def select_pose(self, name):
        self.set_target(self.presets[name])

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
        speed = np.deg2rad(self.settings['joint_speed_deg_s'])
        for _ in range(n):
            self.command += np.clip(self.target - self.command, -speed * h, speed * h)
            self.env.ctrl[self.actuators] = self.command
            self.env.runtime.step()
            self._read_contacts()
        # Physics runs at its native timestep; publish rigid mounts once per frame.
        self._sync_scene()

    def reset(self):
        mujoco.mj_setState(self.env.model, self.env.data, self._initial_state, self._state_kind)
        self.env.runtime.forward()
        self.target = self.presets['ready'].copy()
        self.command = self.target.copy()
        self._remainder = 0.0
        self.contacts, self.contact_samples = {}, {}
        self._sync_scene()

    def report(self):
        return dict(simulated_seconds=float(self.env.data.time - self._initial_time),
                    robot_actuators=len(self.actuators), plant_actuators=0,
                    contacts=self.contacts, contact_samples=self.contact_samples.copy(),
                    joint_target_deg=np.rad2deg(self.target).tolist(),
                    joint_actual_deg=np.rad2deg(self.robot.qs).tolist(),
                    plant_deflection_rad=float(np.linalg.norm(self.plant.mech.qs - self._initial_plant_q)),
                    fruit_displacement_m={k: float(np.linalg.norm(o.pos - self._initial_fruit[k]))
                                          for k, o in self.plant.fruits.items()})


def add_controls(base, demo):
    arm = base.ui.add_panel('arm', title='RS007L joint targets', anchor=Anchor.TOP_LEFT,
                            width=285, font_size=12, movable=True,
                            description='Drag in degrees. The arm moves with a limited target speed.')
    status = base.ui.add_panel('interaction', title='Citrus contact', anchor=Anchor.TOP_RIGHT,
                               width=290, font_size=12, movable=True,
                               description='The blue rounded tool pushes leaves and oranges. Retract to see rebound.')
    for i, angle in enumerate(np.rad2deg(demo.target)):
        arm.add_slider(f'joint_{i + 1}', label=f'Joint {i + 1}', unit='°',
                       min_value=round(float(np.rad2deg(demo.limits[0, i])), 1),
                       max_value=round(float(np.rad2deg(demo.limits[1, i])), 1),
                       step=.1, value=round(float(angle), 1),
                       continuous=True, update_hz=30,
                       on_change=lambda value, i=i: demo.set_joint_degrees(i, value))

    def update_sliders():
        for i, angle in enumerate(np.rad2deg(demo.target)):
            arm.set_value(f'joint_{i + 1}', round(float(angle), 1))

    def pose(name):
        demo.select_pose(name)
        update_sliders()

    def reset():
        demo.reset()
        update_sliders()
        refresh(0)

    def proxies(checked):
        for group in demo.plant.foliage_proxies.values():
            for obj in group:
                obj.toggle_render_collision = checked

    def collision(checked):
        demo.robot.toggle_render_collision = checked
        for obj in (*demo.plant.branch_objects, *demo.plant.fruit_objects, demo.tool, demo.shaft):
            obj.toggle_render_collision = checked

    def forces(checked):
        demo.show_forces = checked
        demo._sync_scene()

    arm.add_button('ready', label='Retract / ready', on_click=lambda: pose('ready'))
    arm.add_button('foliage', label='Touch leaves', on_click=lambda: pose('foliage'))
    arm.add_button('fruit', label=f'Touch {demo.settings["fruit_id"]}', on_click=lambda: pose('fruit'))
    arm.add_button('reset', label='Reset robot and tree', on_click=reset)
    status.add_checkbox('paused', label='Pause physics', on_change=lambda value: setattr(demo, 'paused', value))
    status.add_checkbox('proxies', label='Show foliage contact proxies', on_change=proxies)
    status.add_checkbox('collisions', label='Show arm / branch / fruit collisions', on_change=collision)
    status.add_checkbox('forces', label='Show contact force arrows', on_change=forces)
    for key, label in [('time', 'Simulation'), ('actual', 'Actual joints · degrees'),
                       ('contacts', 'Arm ↔ plant contact'), ('bend', 'Plant deflection'),
                       ('fruit_position', f'{demo.settings["fruit_id"]} · world metres')]:
        status.add_label(key, label=label)

    def refresh(dt):
        report = demo.report()
        status.set_value('time', f'{report["simulated_seconds"]:.1f} s' + (' · paused' if demo.paused else ''))
        status.set_value('actual', ' / '.join(f'{x:.1f}' for x in report['joint_actual_deg']))
        status.set_value('contacts', ', '.join(f'{k}: {v}' for k, v in demo.contacts.items()) or 'No contact')
        status.set_value('bend', f'{report["plant_deflection_rad"]:.3f} rad')
        status.set_value('fruit_position', ' / '.join(f'{x:.3f}' for x in demo.plant.fruits[demo.settings['fruit_id']].pos))

    refresh(0)
    base.schedule_interval(refresh, 1 / demo.settings['status_hz'])


def run_smoke(demo):
    results = {}
    for name in ('foliage', 'fruit'):
        demo.reset()
        demo.select_pose(name)
        travel = float(np.max(np.abs(demo.target - demo.command)) / np.deg2rad(demo.settings['joint_speed_deg_s']))
        demo.step(travel + demo.settings['smoke_hold_seconds'])
        touch = demo.report()
        demo.select_pose('ready')
        demo.step(travel + demo.settings['smoke_release_seconds'])
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
    if args.headless:
        report = run_smoke(demo)
    else:
        p = demo.settings
        base = wvw.World(cam_pos=p['camera_pos'], cam_lookat_pos=p['camera_lookat'], port=args.port)
        base.set_scene(demo.scene)
        base.set_caption('RS007L / Citrus V2 interactive contact')
        add_controls(base, demo)
        base.schedule_interval(lambda dt: demo.step(min(dt, p['max_frame_dt'])), 1 / p['control_hz'])
        if args.duration is not None:
            base.schedule_once(lambda dt: base.close(), args.duration)
        base.run()
        report = demo.report()
    print(json.dumps(report, indent=2), flush=True)
    if args.report:
        args.report.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
