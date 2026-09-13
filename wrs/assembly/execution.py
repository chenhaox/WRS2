"""WRS nominal robot execution: grasp, reach, carry, insert, handoff and replay.

No hardware transport is imported. Robot motion is checked at an explicit joint
resolution; this is a sampled kinematic replay, not a dynamics/CCD certificate.
The carried original mesh is checked separately from MuJoCo convex proxies.
"""

from dataclasses import dataclass, field
from time import perf_counter
import numpy as np
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation
from .model import AssemblyState, Part, MeshData, ContactConfig, freeze, digest
from .part_motion import ContactPolicy, MotionConfig, validate_object_path
from .sequence import replay_sequence, _restricted_config, _balanced
from .contact.analysis import analyze_contacts
from .contact.analysis import analyze_pair
from .contact.graph import build_contact_graph
from .geometry.proximity import MeshProximity
from .stability import check_equilibrium, _generators
from .force_points import prepare_force_points
from .adapters.wrs_scene import scene_object_from_part, part_from_scene_object, rigid_tf_from_wrs


@dataclass(frozen=True)
class ExecutionConfig:
    joint_step_rad: float = 0.015
    cartesian_step_m: float = 0.008
    position_tolerance_m: float = 2e-5
    rotation_tolerance_rad: float = 5e-4
    contact_roundoff_m: float = 2e-6
    max_joint_jump_rad: float = 0.35
    approach_distance_m: float = 0.06
    lift_distance_m: float = 0.10
    max_grasps: int = 12
    planning_time_s: float = 3.0
    time_limit_s: float = 180.0
    seed: int = 7

    def __post_init__(self):
        for k in (
            "joint_step_rad",
            "cartesian_step_m",
            "position_tolerance_m",
            "rotation_tolerance_rad",
            "contact_roundoff_m",
            "max_joint_jump_rad",
            "approach_distance_m",
            "lift_distance_m",
            "planning_time_s",
            "time_limit_s",
        ):
            if not np.isfinite(getattr(self, k)) or getattr(self, k) <= 0:
                raise ValueError(k + " must be positive")
        if type(self.max_grasps) is not int or self.max_grasps < 1 or type(self.seed) is not int:
            raise ValueError("Invalid grasp budget/seed")


@dataclass(frozen=True)
class ExecutionArm:
    arm: object
    finger_normal_force_n: float = 30.0
    contact_friction: float = 0.5

    def __post_init__(self):
        if (
            not np.isfinite(self.finger_normal_force_n)
            or self.finger_normal_force_n <= 0
            or not np.isfinite(self.contact_friction)
            or self.contact_friction < 0
        ):
            raise ValueError("Finite declared finger capacity/friction required")


@dataclass(frozen=True, eq=False)
class ExecutionResult:
    status: str
    frames: tuple
    events: tuple
    diagnostics: dict
    input_digest: str
    execution_validated: bool = False
    validation_level: str = "not_validated"
    contexts: tuple = ()
    config: ExecutionConfig = field(default_factory=ExecutionConfig)
    schema_version: str = "wrs.assembly.execution/1"

    def __post_init__(self):
        for k in ("frames", "events", "diagnostics", "contexts"):
            object.__setattr__(self, k, freeze(getattr(self, k)))


def _box_grasps(part, gripper):
    """Deterministic central side pinches for a verified axis-aligned box mesh."""
    from wrs.grasp.grasp import Grasp

    v = part.geometry.vertices
    lo, hi = v.min(0), v.max(0)
    size = hi - lo
    corners = np.all((np.abs(v - lo) < 1e-9) | (np.abs(v - hi) < 1e-9), axis=1)
    if len(np.unique(v, axis=0)) != 8 or len(part.geometry.faces) != 12 or not np.all(corners):
        return []
    result = []
    for angle in (0.0, np.pi / 2, np.pi, 3 * np.pi / 2):
        r = Rotation.from_euler("z", angle).as_matrix() @ np.diag([1.0, -1.0, -1.0])
        opening = r @ np.asarray(gripper.open_dir)
        width = float(np.abs(opening) @ size)
        if not gripper.jaw_range[0] <= width <= gripper.jaw_range[1]:
            continue
        center = (lo + hi) / 2
        center[2] += 0.2 * size[2]
        pose = np.eye(4)
        pose[:3, :3] = r
        pose[:3, 3] = center
        pre = pose.copy()
        pre[:3, 3] -= r[:, 2] * 0.06
        contacts = np.array([center - opening * width / 2, center + opening * width / 2])
        result.append(
            Grasp.from_jaw(
                gripper,
                pose,
                pre,
                width,
                float(gripper.jaw_range[1]),
                extra_provenance={
                    "contacts_local_m": contacts,
                    "normals_on_object_local": np.array([opening, -opening]),
                    "generator": "deterministic_box_side_pinch",
                },
            )
        )
    return result


def generate_execution_grasps(part, gripper, *, max_grasps=12, seed=7, candidates=None):
    """WRS Grasp records; deterministic boxes, seeded antipodal for other meshes."""
    q = np.asarray(gripper.qs).copy()
    try:
        grasps = list(candidates) if candidates is not None else _box_grasps(part, gripper)
        if not grasps and candidates is None:
            from wrs.grasp.antipodal import antipodal

            saved = np.random.get_state()
            try:
                np.random.seed(seed)
                grasps = antipodal(
                    gripper,
                    scene_object_from_part(part),
                    density=0.015,
                    roll_step_deg=45,
                    clearance=0.0,
                )[:max_grasps]
            finally:
                np.random.set_state(saved)
        # Nominal jaw width need not equal the STL pad-to-pad opening (OR2FG7
        # has a 0.5 mm pad offset per side). Calibrate then VERIFY on the mesh.
        probe = gripper.clone()
        backend = MeshProximity(numerical_tol_m=2e-6)
        finger_parts = [
            part_from_scene_object(link, f"finger_{i}")
            for i, link in enumerate(probe.runtime_lnks[1:])
        ]
        accepted = []
        for g in grasps[:max_grasps]:
            contacts = _grasp_contacts(part, g, gripper, backend, 4e-6)
            if contacts is None:
                continue
            points, inward = contacts
            width = g.provenance["jaw_width"]

            def pad_query(command):
                probe.grip_at(g.pose[:3, 3], g.pose[:3, :3], command)
                distances = []
                witnesses = []
                for fp, link in zip(finger_parts, probe.runtime_lnks[1:]):
                    tf = rigid_tf_from_wrs(link.tf)
                    query = backend.closest_points(fp.geometry, (points - tf[:3, 3]) @ tf[:3, :3])
                    if query.status != "complete":
                        return None
                    distances.append(query.distances_m)
                    witnesses.append(query.points_local_m @ tf[:3, :3].T + tf[:3, 3])
                choice = np.argmin(distances, axis=0)
                nearest = np.asarray(witnesses)[choice, np.arange(2)]
                return nearest, np.min(distances, axis=0)

            measured = pad_query(width)
            if measured is None:
                continue
            nearest, distances = measured
            gap = np.einsum("ij,ij->i", nearest - points, -inward)
            if np.max(np.abs(distances - gap)) > 4e-6:
                continue
            command = float(width - 2 * np.mean(gap))
            if not probe.jaw_range[0] <= command <= probe.jaw_range[1]:
                continue
            verified = pad_query(command)
            if verified is None or np.max(verified[1]) > 4e-6:
                continue
            from wrs.grasp.grasp import Grasp

            accepted.append(
                Grasp.from_jaw(
                    gripper,
                    g.pose,
                    g.pre_pose,
                    command,
                    float(gripper.jaw_range[1]),
                    extra_provenance=dict(
                        g.provenance,
                        jaw_width=command,
                        contacts_local_m=points,
                        normals_on_object_local=inward,
                        pad_residual_m=float(np.max(verified[1])),
                        nominal_width_before_calibration_m=width,
                    ),
                )
            )
        return tuple(accepted)
    finally:
        gripper.fk(qs=q)


def _grasp_contacts(part, grasp, gripper, backend, tolerance):
    c = np.asarray(grasp.pose[:3, 3], dtype=float)
    axis = np.asarray(grasp.pose[:3, :3], dtype=float) @ np.asarray(gripper.open_dir, dtype=float)
    axis /= np.linalg.norm(axis)
    width = grasp.provenance.get("jaw_width")
    if width is None:
        return None
    points = np.asarray(
        grasp.provenance.get("contacts_local_m", [c - axis * width / 2, c + axis * width / 2]),
        dtype=float,
    )
    if points.shape != (2, 3):
        return None
    q = backend.closest_points(part.geometry, points)
    if q.status != "complete" or np.max(q.distances_m) > tolerance:
        return None
    tri = part.geometry.vertices[part.geometry.faces[q.face_ids]]
    outward = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    outward /= np.linalg.norm(outward, axis=1)[:, None]
    inward = np.array([axis, -axis])
    if np.max(np.einsum("ij,ij->i", outward, inward)) > -0.99:
        return None
    return q.points_local_m, inward


def _grasp_wrench_feasible(part, tf, contacts, binding, force_world, torque_at_com):
    if part.com_local_m is None:
        return False
    p, n, groups = contacts
    p = p @ tf[:3, :3].T + tf[:3, 3]
    n = n @ tf[:3, :3].T
    com = tf[:3, :3] @ part.com_local_m + tf[:3, 3]
    rays = np.concatenate([_generators(x, binding.contact_friction, 16) for x in n])
    points = np.repeat(p, 16 if binding.contact_friction else 1, axis=0)
    matrix = np.vstack((rays.T, np.cross(points - com, rays).T))
    finger_ids = np.repeat(groups, 16 if binding.contact_friction else 1)
    cap = np.array([finger_ids == i for i in np.unique(groups)], dtype=float)
    rhs = np.r_[force_world, torque_at_com]
    solved = linprog(
        np.ones(len(rays)),
        A_eq=matrix,
        b_eq=rhs,
        A_ub=cap,
        b_ub=np.full(len(cap), binding.finger_normal_force_n),
        bounds=(0, None),
        method="highs",
    )
    return bool(solved.success and np.max(np.abs(matrix @ solved.x - rhs)) < 1e-6)


def _finger_force_contacts(part, grasp, gripper, backend, tolerance):
    """Actual pad/part overlap polygons; each finger shares ONE force capacity."""
    probe = gripper.clone()
    probe.grip_at(grasp.pose[:3, 3], grasp.pose[:3, :3], grasp.provenance["jaw_width"])
    centers = _grasp_contacts(part, grasp, gripper, backend, tolerance * 2)
    if centers is None:
        return None
    points = []
    normals = []
    groups = []
    cfg = ContactConfig(max_triangle_tests=200000, max_cells=1000)
    for index, link in enumerate(probe.runtime_lnks[1:]):
        finger = part_from_scene_object(link, f"finger_{index}")
        ftf = finger.assembled_tf
        fvertices = finger.geometry.vertices @ ftf[:3, :3].T + ftf[:3, 3]
        for center, inward in zip(*centers):
            # Select the target's supporting facet and actual coplanar pad
            # triangles. The whole finger is checked separately for overlap;
            # its mounting flange need not lie in the pad's halfspace.
            if np.min((part.geometry.vertices - center) @ inward) < -tolerance:
                continue
            patches = []
            for vertices, faces, pid in (
                (part.geometry.vertices, part.geometry.faces, "target_pad"),
                (fvertices, finger.geometry.faces, "finger_pad"),
            ):
                tri = vertices[faces]
                selected = tri[np.max(np.abs((tri - center) @ inward), axis=1) <= tolerance]
                if not len(selected):
                    break
                # Explicit WRS float32 nominal seating, bounded by tolerance;
                # never promote an arbitrary SDF near band into a force patch.
                selected = selected - ((selected - center) @ inward)[..., None] * inward
                vertices, indices = np.unique(selected.reshape(-1, 3), axis=0, return_inverse=True)
                mesh = MeshData(vertices, indices.reshape(-1, 3), orientation="trusted")
                patches.append(Part(pid, mesh, np.eye(4)))
            if len(patches) != 2:
                continue
            report = analyze_pair(
                patches[0], np.eye(4), patches[1], np.eye(4), config=cfg, backend=backend
            )
            for patch in report.patches:
                if patch.classification != "active" or patch.quality not in ("analytic", "bounded"):
                    continue
                prepared, _ = prepare_force_points(patch)
                points.extend(prepared.points)
                normals.extend(-prepared.normals)
                groups.extend([index] * len(prepared.points))
    if len(set(groups)) < 2:
        return None
    return np.asarray(points), np.asarray(normals), np.asarray(groups, dtype=int)


class ExecutionWorkcell:
    """Explicit WRS arms, real staged objects, and finite declared grasp capacity.

    Arms map resource IDs (matching SequenceConfig/supports) to ExecutionArm.
    Geometry and source poses are part of the execution input, not implied by
    the removal path's outside endpoint. Existing WRS Workcell owns activation.
    """

    def __init__(self, assembly, arms, source_poses, *, grasps=None):
        from wrs.manipulation.workcell import Workcell

        self.assembly = assembly
        self.bindings = {
            k: (v if isinstance(v, ExecutionArm) else ExecutionArm(v)) for k, v in arms.items()
        }
        self.wrs_cell = Workcell()
        self.wrs_cell.arms = {k: v.arm for k, v in self.bindings.items()}
        self.parts = {p.part_id: p for p in assembly.parts}
        free = {p.part_id for p in assembly.parts if not p.fixed}
        if set(source_poses) != free:
            raise ValueError("Provide an explicit source pose for every movable part")
        self.source_poses = {k: rigid_tf_from_wrs(v) for k, v in source_poses.items()}
        self.initial_poses = {
            p.part_id: p.assembled_tf for p in assembly.parts if p.fixed
        } | self.source_poses
        self.objects = {
            k: scene_object_from_part(self.parts[k], tf) for k, tf in self.initial_poses.items()
        }
        self.grasps = grasps or {}
        self._initial_q = {
            k: np.asarray(b.arm.body.qs, dtype=float).copy() for k, b in self.bindings.items()
        }
        self._initial_ee = {
            k: np.asarray(b.arm.end_effector.qs, dtype=float).copy()
            for k, b in self.bindings.items()
        }

    def reset(self):
        for k, b in self.bindings.items():
            b.arm.body.fk(qs=self._initial_q[k])
            b.arm.end_effector.fk(qs=self._initial_ee[k])
        for k, p in self.initial_poses.items():
            self.objects[k].tf = p


def _execution_binding(plan, cell, cfg):
    # Include bases and concrete collision geometry; equal robot class names
    # alone do not identify a workcell. Never quantize configuration bindings.
    robots = []
    for rid, b in sorted(cell.bindings.items()):
        geometry = []
        for mechanism in (b.arm.body, b.arm.end_effector):
            for link in mechanism.runtime_lnks:
                geometry.append(part_from_scene_object(link, "link").geometry.geometry_id)
        robots.append(
            (
                rid,
                type(b.arm.body).__name__,
                np.asarray(b.arm.body.tf),
                b.finger_normal_force_n,
                b.contact_friction,
                geometry,
                cell._initial_q[rid],
                cell._initial_ee[rid],
            )
        )
    return digest(
        (
            "execution/2",
            plan.input_digest,
            cell.assembly,
            cell.initial_poses,
            cfg,
            robots,
            tuple((k, [g.to_dict() for g in v]) for k, v in sorted(cell.grasps.items())),
        )
    )


class _PolicyCollider:
    """MuJoCo broad collision plus original-mesh checks for the active target.

    No whole mate is excluded. Contacts involving that mesh are accepted only
    after the independent full-mesh check. Gripper contacts additionally need
    an intended finger/target surface and a nonpenetrating true-mesh result.
    """

    def __init__(self, cell, cfg):
        from wrs.collider.mj_collider import MJCollider

        self.cell, self.cfg = cell, cfg
        self.base = MJCollider()
        for b in cell.bindings.values():
            self.base.append(b.arm.body)
        for o in cell.objects.values():
            self.base.append(o)
        self.base.actors = [b.arm.body for b in cell.bindings.values()]
        self.base.compile(auto_acm=True)
        cell.wrs_cell.collider = self.base
        self.backend = MeshProximity(numerical_tol_m=cfg.contact_roundoff_m)
        self.nominal = dict(cell.initial_poses)
        self.active = None
        self.target = None
        self.policy = None
        self.held = False
        self.hold_tf = None
        self.tcp = None
        self.contacts = None
        self.grasp = None
        self.support_holds = {}
        self.force_contacts = None
        self.last_failure = {}
        self.checks = 0
        runtime = self.base._mjenv.runtime
        sync = self.base._mjenv.sync
        self.bodies = {
            runtime.model.body(v.name).id: ("part", k)
            for k, o in cell.objects.items()
            for obj, v in sync.sobj2bdy.items()
            if obj is o
        }
        self.link_objects = {}
        for rid, b in cell.bindings.items():
            for mech, kind in ((b.arm.body, "robot"), (b.arm.end_effector, "hand")):
                for index, link in enumerate(mech.runtime_lnks):
                    body = sync.rutl2bdy[link]
                    bid = runtime.model.body(body.name).id
                    self.bodies[bid] = (kind, rid, index)
                    self.link_objects[bid] = link
        self.link_parts = {}
        self._target_cache = None
        self._finger_cache = {}
        self.exact_target_cache_hits = 0

    def __getattr__(self, k):
        return getattr(self.base, k)

    def set_mecba_qpos(self, mechanism, qs):
        # WRS reasoner sets the hand through the collider. Keep the mechanism
        # used by the exact-mesh verifier synchronized with that same command.
        mechanism.fk(qs=qs)
        self.base.set_mecba_qpos(mechanism, qs)

    def activate(self, rid):
        self.cell.wrs_cell.activate(self.cell.bindings[rid].arm)
        self.active = rid

    def _target_valid(self):
        if self.target is None:
            return True
        tf = rigid_tf_from_wrs(self.cell.objects[self.target].tf)
        key = (
            self.target,
            tf.tobytes(),
            id(self.policy),
            tuple((k, np.asarray(v).tobytes()) for k, v in sorted(self.nominal.items())),
        )
        if self._target_cache is not None and self._target_cache[0] == key:
            self.exact_target_cache_hits += 1
            valid, self.last_failure = self._target_cache[1:]
            return valid
        state = AssemblyState(dict(self.nominal, **{self.target: tf}))
        motion = MotionConfig(
            clearance_m=0.0,
            numerical_tol_m=self.cfg.contact_roundoff_m,
            min_step_m=self.cfg.contact_roundoff_m,
            max_nodes=32,
            max_triangle_tests=100000,
        )
        v = validate_object_path(
            self.cell.assembly,
            state,
            self.target,
            [tf],
            policy=self.policy,
            config=motion,
            backend=self.backend,
        )
        if v.status != "valid":
            self.last_failure = dict(reason="target_mesh_" + v.reason, details=v.witness)
        valid = v.status == "valid"
        self._target_cache = (key, valid, dict(self.last_failure))
        return valid

    def _finger_contact_valid(self, bid, pid):
        kind, rid, index = self.bodies[bid]
        if kind != "hand" or index == 0:
            return False
        hold = self.support_holds.get(rid)
        if rid == self.active and pid == self.target:
            contacts = self.contacts
            grasp = self.grasp
        elif hold and hold["part_id"] == pid:
            contacts = hold["contacts"]
            grasp = hold["grasp"]
        else:
            return False
        if contacts is None or grasp is None:
            return False
        link = self.link_objects[bid]
        if bid not in self.link_parts:
            self.link_parts[bid] = part_from_scene_object(link, f"link_{bid}")
        part = self.cell.parts[pid]
        tf = rigid_tf_from_wrs(self.cell.objects[pid].tf)
        other = self.link_parts[bid]
        ltf = rigid_tf_from_wrs(link.tf)
        key = (tf.tobytes(), ltf.tobytes(), np.asarray(contacts).tobytes())
        previous = self._finger_cache.get((bid, pid))
        if previous is not None and previous[0] == key:
            return previous[1]

        def finish(valid):
            self._finger_cache[bid, pid] = (key, bool(valid))
            return bool(valid)

        p, n = contacts
        p = p @ tf[:3, :3].T + tf[:3, 3]
        n = n @ tf[:3, :3].T
        object_vertices = part.geometry.vertices @ tf[:3, :3].T + tf[:3, 3]
        finger_vertices = other.geometry.vertices @ ltf[:3, :3].T + ltf[:3, 3]
        # Exact supporting halfspace of the actual target and finger meshes;
        # permits the intended pad surface, never an overlapping finger volume.
        for point, inward in zip(p, n):
            if (
                np.min((object_vertices - point) @ inward) >= -self.cfg.contact_roundoff_m
                and np.max((finger_vertices - point) @ inward) <= self.cfg.contact_roundoff_m
            ):
                return finish(True)
        overlap = self.backend.classify_overlap(part, tf, other, ltf, budget=150000)
        if overlap.status == "separated":
            return finish(True)
        if overlap.status != "touching" or overlap.witness_world_m is None:
            return finish(False)
        point = np.asarray(overlap.witness_world_m)
        return finish(
            np.min(np.abs(np.einsum("ij,ij->i", point - p, n))) <= self.cfg.contact_roundoff_m * 2
        )

    def is_collided(self, qs):
        self.checks += 1
        self.last_failure = {}
        binding = self.cell.bindings[self.active]
        binding.arm.body.fk(qs=qs)
        for b in self.cell.bindings.values():
            self.base.set_mecba_qpos(b.arm.body, b.arm.body.qs)
            self.base.set_mecba_qpos(b.arm.end_effector, b.arm.end_effector.qs)
        if self.held:
            self.cell.objects[self.target].tf = np.asarray(self.tcp.tf, dtype=float) @ self.hold_tf
        if not self._target_valid():
            return True
        hit = self.base.is_collided(qs)
        if not hit:
            return False
        runtime = self.base._mjenv.runtime
        model = runtime.model
        for c in runtime.data.contact[: runtime.data.ncon]:
            bid1, bid2 = int(model.geom_bodyid[c.geom1]), int(model.geom_bodyid[c.geom2])
            x, y = self.bodies.get(bid1), self.bodies.get(bid2)
            if x is None or y is None:
                self.last_failure = dict(reason="unmapped_collision_body")
                return True
            if x[0] == "part" and y[0] == "part":
                # Stationary part/fixture contacts are checked by the equilibrium
                # graph. The active target was checked above using true meshes.
                continue
            if x[0] == "part":
                x, y = y, x
                bid1, bid2 = bid2, bid1
            if y[0] == "part" and self._finger_contact_valid(bid1, y[1]):
                continue
            self.last_failure = dict(
                reason="robot_or_gripper_collision", bodies=(x, y), proxy_depth_m=float(c.dist)
            )
            return True
        return False


@dataclass
class _InsertionTarget:
    part_id: str
    source_tf: np.ndarray
    goal_tf: np.ndarray
    source_policy: ContactPolicy
    goal_policy: ContactPolicy


@dataclass
class _GraspAttachment:
    """Main-hand contact permission saved while the auxiliary arm takes over."""

    grasp: object
    contacts: object
    force_contacts: object
    tcp: object
    hold_tf: np.ndarray

    def restore(self, collider):
        collider.grasp = self.grasp
        collider.contacts = self.contacts
        collider.force_contacts = self.force_contacts
        collider.tcp = self.tcp
        collider.hold_tf = self.hold_tf


class _ExecutionSession:
    """Mutable state for one simulated execution; public results stay immutable.

    Steps are executed in event order. Independent replay remains outside this
    class, and robot settings, poses and RNG are restored on every exit.
    """

    def __init__(self, plan, workcell, config):
        self.plan = plan
        self.cell = workcell
        self.config = config
        self.assembly = workcell.assembly
        self.frames = []
        self.events = []
        self.failures = []
        self.contexts = []
        self.context_keys = {}
        self.started = perf_counter()
        self.input_digest = _execution_binding(plan, workcell, config)
        self.supports = {support.candidate.support_id: support for support in plan.supports}
        self.active_supports = set()
        self.grasps_cache = {}
        self.main_resource = plan.config.handling.resource_id

    def _finish(self, status, reason, **extra):
        if status == "unknown" and perf_counter() - self.started > self.config.time_limit_s:
            status, reason = "exhausted", "execution_time_budget"
        return ExecutionResult(
            status=status,
            frames=tuple(self.frames),
            events=tuple(self.events),
            diagnostics=dict(
                reason=reason,
                elapsed_s=perf_counter() - self.started,
                failures=self.failures,
                **extra,
            ),
            input_digest=self.input_digest,
            execution_validated=status == "success",
            validation_level="sampled_joint_nominal_mesh"
            if status == "success"
            else "not_validated",
            contexts=tuple(self.contexts),
            config=self.config,
        )

    def _contact_policy(self, pid, tf):
        state = AssemblyState(dict(self.collider.nominal, **{pid: tf}))
        graph = build_contact_graph(
            self.assembly,
            state,
            analyze_contacts(self.assembly, state, backend=self.collider.backend),
        )
        return ContactPolicy.from_graph(
            self.assembly,
            state,
            pid,
            graph,
            backend=self.collider.backend,
            tolerance_m=self.config.contact_roundoff_m,
        )

    def _snapshot(self, phase):
        context = dict(
            active=self.collider.active,
            target=self.collider.target,
            policy=self.collider.policy,
            nominal=dict(self.collider.nominal),
            contacts=self.collider.contacts,
            force_contacts=self.collider.force_contacts,
            grasp=self.collider.grasp.to_dict() if self.collider.grasp else None,
            support_holds={
                rid: dict(
                    part_id=h["part_id"],
                    contacts=h["contacts"],
                    grasp=h["grasp"].to_dict(),
                    hold_tf=h["hold_tf"],
                    force_contacts=h["force_contacts"],
                    support_id=h.get("support_id"),
                )
                for rid, h in self.collider.support_holds.items()
            },
        )
        context_key = digest(context)
        if context_key not in self.context_keys:
            self.context_keys[context_key] = len(self.contexts)
            self.contexts.append(freeze(context))
        self.frames.append(
            dict(
                phase=phase,
                active_arm=self.collider.active,
                target=self.collider.target,
                held=self.collider.held,
                context_id=self.context_keys[context_key],
                hold_tf=None if self.collider.hold_tf is None else self.collider.hold_tf.copy(),
                tcp=None if self.collider.grasp is None else np.asarray(self.collider.grasp.tcp),
                robot_qs={
                    k: np.asarray(b.arm.body.qs).copy() for k, b in self.cell.bindings.items()
                },
                gripper_qs={
                    k: np.asarray(b.arm.end_effector.qs).copy()
                    for k, b in self.cell.bindings.items()
                },
                object_poses={k: rigid_tf_from_wrs(o.tf) for k, o in self.cell.objects.items()},
                active_supports=tuple(sorted(self.active_supports)),
            )
        )

    def _record_motion(self, motion, phase):
        if motion is None:
            self.failures.append(
                dict(phase=phase, reason="WRS_planner_failed", details=self.collider.last_failure)
            )
            return False
        qlist = motion.robot_qpos_list
        for i, q in enumerate(qlist):
            previous = q if i == 0 else qlist[i - 1]
            count = max(
                1,
                int(np.ceil(np.linalg.norm(np.asarray(q) - previous) / self.config.joint_step_rad)),
            )
            for t in range(1, count + 1):
                qq = np.asarray(previous) + (np.asarray(q) - previous) * t / count
                if self.collider.is_collided(qq):
                    self.failures.append(
                        dict(
                            phase=phase,
                            reason="dense_replay_collision",
                            details=self.collider.last_failure,
                        )
                    )
                    return False
                self._snapshot(phase)
        return True

    def _move_linear(self, world_tcp, phase):
        b = self.cell.bindings[self.collider.active]
        arm = b.arm
        q = np.asarray(arm.body.qs).copy()
        seg = arm.insert(
            world_tcp[:3, 3],
            world_tcp[:3, :3],
            collider=self.collider,
            tcp=self.collider.tcp,
            start_qs=q,
            granularity=self.config.cartesian_step_m,
            ee_qpos=np.asarray(arm.end_effector.qs).copy(),
        )
        if seg is not None and any(
            np.linalg.norm(np.asarray(v) - u) > self.config.max_joint_jump_rad
            for u, v in zip(seg.robot_qpos_list, seg.robot_qpos_list[1:])
        ):
            self.failures.append(dict(phase=phase, reason="IK_branch_jump"))
            return False
        return self._record_motion(seg, phase)

    def _move_jaw(self, qgoal, phase):
        b = self.cell.bindings[self.collider.active]
        ee = b.arm.end_effector
        start = np.asarray(ee.qs).copy()
        q = np.asarray(b.arm.body.qs).copy()
        for t in np.linspace(0, 1, 9):
            ee.fk(qs=start + (qgoal - start) * t)
            if self.collider.is_collided(q):
                self.failures.append(
                    dict(
                        phase=phase,
                        reason="jaw_sweep_collision",
                        details=self.collider.last_failure,
                    )
                )
                return False
            self._snapshot(phase)
        return True

    def _acquire(self, rid, pid, policy, *, support=None):
        self.collider.activate(rid)
        self.collider.target = pid
        self.collider.policy = policy
        self.collider.held = False
        b = self.cell.bindings[rid]
        ee = b.arm.end_effector
        part = self.cell.parts[pid]
        cache = (rid, pid)
        if cache not in self.grasps_cache:
            self.grasps_cache[cache] = generate_execution_grasps(
                part,
                ee,
                max_grasps=self.config.max_grasps,
                seed=self.config.seed,
                candidates=self.cell.grasps.get(cache),
            )
        start_q = np.asarray(b.arm.body.qs).copy()
        pose = self.collider.nominal[pid]
        rejected = []
        for grasp in self.grasps_cache[cache]:
            if perf_counter() - self.started > self.config.time_limit_s:
                return False
            contacts = _grasp_contacts(
                part, grasp, ee, self.collider.backend, self.config.contact_roundoff_m * 2
            )
            if contacts is None:
                rejected.append("missing_surface_contacts")
                continue
            force_contacts = _finger_force_contacts(
                part, grasp, ee, self.collider.backend, self.config.contact_roundoff_m
            )
            if force_contacts is None:
                rejected.append("missing_force_patch")
                continue
            if not self._grasp_has_capacity(part, pose, force_contacts, b, support):
                rejected.append("finite_grasp_capacity")
                continue
            self.collider.grasp = grasp
            self.collider.contacts = contacts
            self.collider.force_contacts = force_contacts
            self.collider.tcp = grasp.make_tcp(ee)
            if not self._approach_grasp(b, grasp, pose, start_q, rejected):
                continue
            actual = rigid_tf_from_wrs(self.collider.tcp.tf)
            self.collider.hold_tf = np.linalg.inv(actual) @ pose
            if support is None:
                self.collider.held = True
            else:
                self.collider.support_holds[rid] = dict(
                    part_id=pid,
                    grasp=grasp,
                    contacts=contacts,
                    tcp=self.collider.tcp,
                    hold_tf=self.collider.hold_tf,
                    force_contacts=force_contacts,
                    support_id=support.candidate.support_id,
                )
            self.events.append(
                dict(
                    kind="acquire_auxiliary" if support else "acquire_part",
                    resource_id=rid,
                    part_id=pid,
                    frame=len(self.frames) - 1,
                    grasp=grasp.to_dict(),
                    contact_points_local_m=force_contacts[0],
                    finger_capacity_n=b.finger_normal_force_n,
                    friction=b.contact_friction,
                )
            )
            return True
        self.failures.append(
            dict(
                reason="no_reachable_capacity_valid_grasp",
                resource_id=rid,
                part_id=pid,
                candidates=len(self.grasps_cache[cache]),
                rejected=rejected,
            )
        )
        return False

    def _grasp_has_capacity(self, part, pose, force_contacts, b, support):
        """Check gravity for carrying, or every declared support-wrench extreme."""
        force = -part.mass_kg * self.assembly.gravity_world_m_s2
        loadset = [(force, np.zeros(3))]
        if support is not None:
            s = support.candidate
            com = pose[:3, :3] @ part.com_local_m + pose[:3, 3]
            loadset = [
                (
                    r * s.max_normal_force_n,
                    np.cross(s.point_world_m - com, r * s.max_normal_force_n),
                )
                for r in _generators(s.normal_world, s.friction, 16)
            ]
        return all(_grasp_wrench_feasible(part, pose, force_contacts, b, f, t) for f, t in loadset)

    def _approach_grasp(self, b, grasp, pose, start_q, rejected):
        """Validate grasp IK, reach the pregrasp, approach and close the fingers."""
        ee = b.arm.end_effector
        ee.fk(qs=grasp.pre_qpos)
        b.arm.body.fk(qs=start_q)
        target = pose @ np.asarray(grasp.pose, dtype=float)
        pre = target.copy()
        pre[:3, 3] -= target[:3, 2] * self.config.approach_distance_m
        # Real WRS common-grasp reasoning precedes reach and approach.
        ctx, _ = b.arm._context(self.collider)
        from wrs.grasp.reasoner import reason_common_gids

        reachable = reason_common_gids(
            b.arm.body, ctx, [grasp], [pose], gripper=ee, which="grasp", max_solutions=4
        )
        if not reachable:
            rejected.append(
                dict(reason="grasp_IK_or_collision", details=self.collider.last_failure)
            )
            return False
        ee.fk(qs=grasp.pre_qpos)
        b.arm.body.fk(qs=start_q)
        if not ctx.is_state_valid(start_q):
            rejected.append(
                dict(reason="reach_start_collision", details=self.collider.last_failure)
            )
            return False
        if (
            b.arm.reachable(pre, collider=self.collider, tcp=self.collider.tcp, ref_qs=start_q)
            is None
        ):
            rejected.append(
                dict(reason="pregrasp_IK_or_collision", details=self.collider.last_failure)
            )
            return False
        seg = b.arm.moveto(
            pre,
            collider=self.collider,
            tcp=self.collider.tcp,
            start_qs=start_q,
            ee_qpos=grasp.pre_qpos,
            time_limit=self.config.planning_time_s,
            shortcut=False,
        )
        mark = len(self.frames)
        if not self._record_motion(seg, "reach"):
            del self.frames[mark:]
            return False
        if not self._move_linear(target, "approach"):
            del self.frames[mark:]
            return False
        if not self._move_jaw(np.asarray(grasp.qpos), "close"):
            del self.frames[mark:]
            return False
        return True

    def _release(self, rid, pid, *, auxiliary=False):
        self.collider.activate(rid)
        self.collider.target = pid
        self.collider.held = False
        if auxiliary:
            hold = self.collider.support_holds[rid]
            self.collider.grasp = hold["grasp"]
            self.collider.contacts = hold["contacts"]
            self.collider.tcp = hold["tcp"]
        # The receiver is already supporting the object at this event. Opening
        # ends rigid ownership before the releasing robot retreats.
        self.collider.support_holds.pop(rid, None)
        self.collider.policy = self._contact_policy(pid, self.collider.nominal[pid])
        if not self._move_jaw(np.asarray(self.collider.grasp.pre_qpos), "open"):
            return False
        target = rigid_tf_from_wrs(self.collider.tcp.tf)
        target[:3, 3] -= target[:3, 2] * self.config.approach_distance_m
        if not self._move_linear(target, "retreat"):
            return False
        self.events.append(
            dict(
                kind="release_auxiliary" if auxiliary else "release_part",
                resource_id=rid,
                part_id=pid,
                frame=len(self.frames) - 1,
            )
        )
        return True

    def _insert_part(self, step, target):
        """Lift from staging, transfer above the scene, insert, then seat within roundoff."""
        lift = target.source_tf.copy()
        lift[:3, 3] -= (target.source_tf @ self.collider.grasp.pose)[
            :3, 2
        ] * self.config.lift_distance_m
        self.collider.policy = target.source_policy
        if not self._move_linear(lift @ np.linalg.inv(self.collider.hold_tf), "lift"):
            return self._finish("unknown", "lift_failed")
        self.collider.policy = None
        outside = step.assembly_poses[0]
        # Conservative three-leg free transfer above all geometry.
        high = (
            max(
                float(max((p.geometry.vertices @ tf[:3, :3].T + tf[:3, 3])[:, 2]))
                for p in self.assembly.parts
                for tf in [self.collider.nominal[p.part_id]]
            )
            + self.config.lift_distance_m
        )
        up = lift.copy()
        up[2, 3] = max(high, lift[2, 3], outside[2, 3])
        across = outside.copy()
        across[2, 3] = up[2, 3]
        for waypoint in (up, across, outside):
            if not self._move_linear(waypoint @ np.linalg.inv(self.collider.hold_tf), "transfer"):
                return self._finish("unknown", "transfer_failed")
        self.collider.policy = target.goal_policy
        for waypoint in step.assembly_poses[1:]:
            if not self._move_linear(waypoint @ np.linalg.inv(self.collider.hold_tf), "insert"):
                return self._finish("unknown", "insert_failed")
        actual = rigid_tf_from_wrs(self.cell.objects[target.part_id].tf)
        error = float(np.linalg.norm(actual[:3, 3] - target.goal_tf[:3, 3]))
        angle = float(Rotation.from_matrix(actual[:3, :3] @ target.goal_tf[:3, :3].T).magnitude())
        if error > self.config.position_tolerance_m or angle > self.config.rotation_tolerance_rad:
            return self._finish(
                "unknown",
                "object_target_residual",
                position_error_m=error,
                rotation_error_rad=angle,
            )
        # Explicit nominal seating is limited to the declared WRS
        # float32 roundoff, never a macroscopic teleport/press fit.
        radius = float(
            np.max(np.linalg.norm(self.cell.parts[target.part_id].geometry.vertices, axis=1))
        )
        if error + radius * angle > self.config.contact_roundoff_m:
            return self._finish("unknown", "seating_exceeds_roundoff")
        self.collider.nominal[target.part_id] = target.goal_tf
        self.cell.objects[target.part_id].tf = target.goal_tf
        # Keep the main hand's scoped contact permission while a
        # second arm approaches and takes over this stationary part.
        self.collider.support_holds[self.main_resource] = dict(
            part_id=target.part_id,
            grasp=self.collider.grasp,
            contacts=self.collider.contacts,
            tcp=self.collider.tcp,
            hold_tf=self.collider.hold_tf,
            force_contacts=self.collider.force_contacts,
        )
        self.events.append(
            dict(
                kind="nominal_seat",
                part_id=target.part_id,
                frame=len(self.frames) - 1,
                translation_m=error,
                rotation_rad=angle,
            )
        )

    def _pick_from_staging(self, target):
        """The remaining staging assembly must balance without the carried part."""
        remainder = AssemblyState(
            {k: v for k, v in self.collider.nominal.items() if k != target.part_id}
        )
        rest_graph = build_contact_graph(
            self.assembly,
            remainder,
            analyze_contacts(self.assembly, remainder, backend=self.collider.backend),
        )
        rest = check_equilibrium(
            self.assembly,
            remainder,
            rest_graph,
            config=_restricted_config(self.plan.config.stability, remainder),
            supports=tuple(
                self.supports[i].candidate
                for i in self.active_supports
                if self.supports[i].candidate.part_id != target.part_id
            ),
        )
        if not _balanced(rest):
            return self._finish("unknown", "staging_remainder_unstable_during_pick")
        if not self._acquire(self.main_resource, target.part_id, target.source_policy):
            return self._finish("unknown", "pick_failed")

    def _execute_step(self, step):
        part_id = step.part_id
        source = self.collider.nominal[part_id]
        source_policy = self._contact_policy(part_id, source)
        goal = step.assembly_after.poses[part_id]
        goal_policy = self._contact_policy(part_id, goal)
        target = _InsertionTarget(part_id, source, goal, source_policy, goal_policy)
        main_hold = None
        for event in step.assembly_events:
            if perf_counter() - self.started > self.config.time_limit_s:
                return self._finish("exhausted", "execution_time_budget")
            kind = event["kind"]
            if kind == "take_from_staging":
                failure = self._pick_from_staging(target)
                if failure is not None:
                    return failure
                main_hold = _GraspAttachment(
                    self.collider.grasp,
                    self.collider.contacts,
                    self.collider.force_contacts,
                    self.collider.tcp,
                    self.collider.hold_tf,
                )
            elif kind == "insert_part":
                failure = self._insert_part(step, target)
                if failure is not None:
                    return failure
            elif kind == "acquire_auxiliary":
                support = self.supports[event["support_id"]]
                support_part = support.candidate.part_id
                policy = self._contact_policy(support_part, self.collider.nominal[support_part])
                if not self._acquire(support.resource_id, support_part, policy, support=support):
                    return self._finish("unknown", "auxiliary_unreachable_or_capacity_failed")
                self.active_supports.add(support.candidate.support_id)
            elif kind == "release_part":
                main_hold.restore(self.collider)
                if not self._release(self.main_resource, part_id):
                    return self._finish("unknown", "release_or_retreat_failed")
            elif kind == "release_auxiliary":
                support = self.supports[event["support_id"]]
                self.active_supports.remove(support.candidate.support_id)
                if not self._release(
                    support.resource_id, support.candidate.part_id, auxiliary=True
                ):
                    return self._finish("unknown", "auxiliary_release_failed")
        return self._verify_released_assembly(part_id)

    def _verify_released_assembly(self, part_id):
        """After release, only installed contacts and declared auxiliary holds remain."""
        state = AssemblyState(self.collider.nominal)
        graph = build_contact_graph(
            self.assembly,
            state,
            analyze_contacts(self.assembly, state, backend=self.collider.backend),
        )
        eq = check_equilibrium(
            self.assembly,
            state,
            graph,
            config=_restricted_config(self.plan.config.stability, state),
            supports=tuple(self.supports[i].candidate for i in self.active_supports),
        )
        if not _balanced(eq):
            return self._finish("unknown", "post_release_equilibrium_failed")
        self.events.append(
            dict(
                kind="release_equilibrium_verified",
                part_id=part_id,
                frame=len(self.frames) - 1,
                equilibrium_digest=eq.input_digest,
            )
        )

    def run(self):
        if replay_sequence(self.assembly, self.plan)["status"] != "valid":
            return self._finish("unknown", "M2_forward_replay_failed")
        required = {self.plan.config.handling.resource_id} | {
            s.resource_id
            for s in self.plan.supports
            if any(
                s.candidate.support_id in st.supports_before + st.supports_after
                for st in self.plan.removal_steps
            )
        }
        if not required.issubset(self.cell.bindings):
            return self._finish(
                "unsupported",
                "missing_robot_resources",
                missing=sorted(required - set(self.cell.bindings)),
            )
        self.cell.reset()
        initial = AssemblyState(self.cell.initial_poses)
        analysis = analyze_contacts(self.assembly, initial)
        graph = build_contact_graph(self.assembly, initial, analysis)
        if not _balanced(
            check_equilibrium(
                self.assembly,
                initial,
                graph,
                config=_restricted_config(self.plan.config.stability, initial),
            )
        ):
            return self._finish("unknown", "staging_not_independently_supported")
        self.collider = _PolicyCollider(self.cell, self.config)
        arm_settings = {
            k: (b.arm.collision_cache_size, b.arm.collision_cache_decimals, b.arm.cd_step_size)
            for k, b in self.cell.bindings.items()
        }
        for b in self.cell.bindings.values():
            b.arm.collision_cache_size = 0
            b.arm.collision_cache_decimals = None
            b.arm.cd_step_size = self.config.joint_step_rad
        saved_rng = np.random.get_state()
        np.random.seed(self.config.seed)

        try:
            for step in self.plan.assembly_steps:
                failure = self._execute_step(step)
                if failure is not None:
                    return failure
            provisional = self._finish("pending", "independent_replay_pending")
            replay = replay_execution(provisional, self.cell, plan=self.plan)
            if replay["status"] != "valid":
                return self._finish("unknown", "independent_replay_failed", replay=replay)
            return self._finish(
                "success",
                "forward_nominal_robot_replay_passed",
                frames_checked=replay["frames_checked"],
                collision_queries=self.collider.checks,
                joint_step_rad=self.config.joint_step_rad,
                exact_static_target_cache_hits=self.collider.exact_target_cache_hits,
                contact_roundoff_m=self.config.contact_roundoff_m,
                robot_ids=list(self.cell.bindings),
                assumptions=[
                    "quasistatic_declared_grasp_capacity",
                    "sampled_joint_collision_checks",
                    "WRS_rest_pose_self_collision_matrix",
                    "explicit_roundoff_only_nominal_seating",
                    "actual_pad_polygon_with_bounded_float32_projection",
                ],
            )
        finally:
            np.random.set_state(saved_rng)
            for k, b in self.cell.bindings.items():
                b.arm.collision_cache_size, b.arm.collision_cache_decimals, b.arm.cd_step_size = (
                    arm_settings[k]
                )
            self.cell.reset()


def validate_execution(plan, workcell, *, config=None):
    """Plan and independently replay a complete forward WRS assembly candidate."""
    return _ExecutionSession(plan, workcell, config or ExecutionConfig()).run()


def replay_execution(result, workcell, *, plan):
    """Fresh world, no rounded joint cache: verify state chain, FK and loads."""
    from wrs.grasp.grasp import Grasp
    from wrs.robots.base.tcp import TCP

    cell = workcell
    cfg = result.config
    if not result.frames:
        return dict(status="unknown", reason="no_frames")
    if result.input_digest != _execution_binding(plan, cell, cfg):
        raise ValueError("Execution belongs to another plan/workcell/configuration")
    if replay_sequence(cell.assembly, plan)["status"] != "valid":
        return dict(status="unknown", reason="M2_forward_replay_failed")
    cell.reset()
    collider = _PolicyCollider(cell, cfg)
    checked = 0
    loads_cache = {}
    previous_q = cell._initial_q
    previous_objects = cell.initial_poses
    supports = {s.candidate.support_id: s for s in plan.supports}
    release_frames = {
        e["frame"] for e in result.events if e["kind"] == "release_equilibrium_verified"
    }
    try:
        for frame in result.frames:
            context = result.contexts[frame["context_id"]]
            for rid, b in cell.bindings.items():
                q = np.asarray(frame["robot_qs"][rid])
                low, high = b.arm.body.chain_joint_limits(b.arm.arm_chain)
                if not np.all(np.isfinite(q)) or not np.all(np.isfinite(frame["gripper_qs"][rid])):
                    return dict(status="unknown", reason="nonfinite_configuration", frame=checked)
                if np.any(q < low - 1e-7) or np.any(q > high + 1e-7):
                    return dict(status="unknown", reason="joint_bounds", frame=checked)
                if np.linalg.norm(q - previous_q[rid]) > cfg.joint_step_rad + 1e-6:
                    return dict(status="unknown", reason="trajectory_gap", frame=checked)
                ee_limits = b.arm.end_effector.structure.compiled
                eq = np.asarray(frame["gripper_qs"][rid])
                if np.any(eq < ee_limits.jlmt_low_by_idx - 1e-7) or np.any(
                    eq > ee_limits.jlmt_high_by_idx + 1e-7
                ):
                    return dict(status="unknown", reason="gripper_bounds", frame=checked)
                b.arm.body.fk(qs=q)
                b.arm.end_effector.fk(qs=frame["gripper_qs"][rid])
            if set(frame["object_poses"]) != set(cell.objects):
                return dict(status="unknown", reason="object_set_mismatch")
            for k, tf in frame["object_poses"].items():
                if not (frame["held"] and k == frame["target"]):
                    if not np.allclose(
                        tf, previous_objects[k], atol=cfg.contact_roundoff_m, rtol=0
                    ):
                        return dict(
                            status="unknown", reason="unheld_object_moved", frame=checked, part_id=k
                        )
                cell.objects[k].tf = tf
            collider.activate(frame["active_arm"])
            collider.target = frame["target"]
            collider.policy = context["policy"]
            collider.nominal = dict(context["nominal"])
            collider.held = frame["held"]
            collider.hold_tf = frame["hold_tf"]
            collider.contacts = context["contacts"]
            collider.force_contacts = context["force_contacts"]
            collider.grasp = Grasp.from_dict(context["grasp"]) if context["grasp"] else None
            ee = cell.bindings[frame["active_arm"]].arm.end_effector
            collider.tcp = (
                TCP(ee.runtime_root_lnk, np.asarray(frame["tcp"], dtype=np.float32))
                if frame["tcp"] is not None
                else None
            )
            collider.support_holds = {}
            for rid, h in context["support_holds"].items():
                g = Grasp.from_dict(h["grasp"])
                tcp = g.make_tcp(cell.bindings[rid].arm.end_effector)
                if not np.allclose(
                    np.asarray(tcp.tf) @ h["hold_tf"],
                    frame["object_poses"][h["part_id"]],
                    atol=cfg.contact_roundoff_m,
                    rtol=0,
                ):
                    return dict(
                        status="unknown", reason="auxiliary_not_at_support_pose", frame=checked
                    )
                collider.support_holds[rid] = dict(h, grasp=g, tcp=tcp)
                sid = h["support_id"]
                if sid is not None:
                    support = supports[sid].candidate
                    part = cell.parts[h["part_id"]]
                    tf = np.asarray(frame["object_poses"][part.part_id])
                    com = tf[:3, :3] @ part.com_local_m + tf[:3, 3]
                    cache_key = digest((sid, tf, h["force_contacts"]))
                    if cache_key not in loads_cache:
                        loads_cache[cache_key] = all(
                            _grasp_wrench_feasible(
                                part,
                                tf,
                                h["force_contacts"],
                                cell.bindings[rid],
                                ray * support.max_normal_force_n,
                                np.cross(
                                    support.point_world_m - com, ray * support.max_normal_force_n
                                ),
                            )
                            for ray in _generators(support.normal_world, support.friction, 16)
                        )
                    if not loads_cache[cache_key]:
                        return dict(
                            status="unknown", reason="auxiliary_grasp_capacity", frame=checked
                        )
            for sid in frame["active_supports"]:
                support = supports[sid]
                hold = collider.support_holds.get(support.resource_id)
                if hold is None or hold.get("support_id") != sid:
                    return dict(
                        status="unknown", reason="support_active_without_grasp", frame=checked
                    )
            if frame["held"]:
                actual = np.asarray(collider.tcp.tf, dtype=float) @ frame["hold_tf"]
                if not np.allclose(
                    actual,
                    frame["object_poses"][frame["target"]],
                    atol=cfg.contact_roundoff_m,
                    rtol=0,
                ):
                    return dict(status="unknown", reason="held_FK_mismatch", frame=checked)
                part = cell.parts[frame["target"]]
                tf = rigid_tf_from_wrs(actual)
                binding = cell.bindings[frame["active_arm"]]
                cache_key = digest(
                    (part.part_id, tf[:3, :3], collider.contacts, frame["active_arm"])
                )
                if cache_key not in loads_cache:
                    loadset = [(-part.mass_kg * cell.assembly.gravity_world_m_s2, np.zeros(3))]
                    for case in plan.config.stability.disturbances:
                        ws = [w for w in case.wrenches if w.part_id == part.part_id]
                        loadset.append(
                            (
                                -part.mass_kg * cell.assembly.gravity_world_m_s2
                                - sum((w.force_world_n for w in ws), np.zeros(3)),
                                -sum((w.torque_world_nm for w in ws), np.zeros(3)),
                            )
                        )
                    loads_cache[cache_key] = all(
                        _grasp_wrench_feasible(part, tf, collider.force_contacts, binding, f, t)
                        for f, t in loadset
                    )
                if not loads_cache[cache_key]:
                    return dict(status="unknown", reason="carried_grasp_capacity", frame=checked)
            if collider.is_collided(frame["robot_qs"][frame["active_arm"]]):
                return dict(
                    status="unknown",
                    reason="replay_collision",
                    frame=checked,
                    details=collider.last_failure,
                )
            if checked in release_frames:
                if any(
                    not np.allclose(
                        tf, frame["object_poses"][pid], atol=cfg.contact_roundoff_m, rtol=0
                    )
                    for pid, tf in context["nominal"].items()
                ):
                    return dict(status="unknown", reason="release_seating_residual", frame=checked)
                # WRS stores transforms in float32. Reapply only the explicitly
                # bounded nominal seating checked above, including fixed bases.
                state = AssemblyState(context["nominal"])
                graph = build_contact_graph(
                    cell.assembly,
                    state,
                    analyze_contacts(cell.assembly, state, backend=collider.backend),
                )
                balance = check_equilibrium(
                    cell.assembly,
                    state,
                    graph,
                    config=_restricted_config(plan.config.stability, state),
                    supports=tuple(supports[sid].candidate for sid in frame["active_supports"]),
                )
                if not _balanced(balance):
                    return dict(
                        status="unknown", reason="release_equilibrium_failed", frame=checked
                    )
            previous_q = frame["robot_qs"]
            previous_objects = frame["object_poses"]
            checked += 1
        final = result.frames[-1]
        if any(
            not np.allclose(final["object_poses"][pid], tf, atol=cfg.contact_roundoff_m, rtol=0)
            for pid, tf in plan.initial_state.poses.items()
        ):
            return dict(status="unknown", reason="final_assembly_pose_mismatch")
        if set(final["active_supports"]) != set(plan.initial_support_ids):
            return dict(status="unknown", reason="final_support_mismatch")
        free = {p.part_id for p in cell.assembly.parts if not p.fixed}
        released = {
            e["part_id"] for e in result.events if e["kind"] == "release_equilibrium_verified"
        }
        if free != released or len(release_frames) != len(free):
            return dict(status="unknown", reason="missing_release_events")
        return dict(status="valid", frames_checked=checked, collision_checks=collider.checks)
    finally:
        cell.reset()
