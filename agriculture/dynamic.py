"""PlantSpec + cluster dynamics -> ordinary WRS MechBase with rigid mounts.

No MJCF strings, engine body IDs, plant controllers, or per-leaf bodies here.
"""
from dataclasses import dataclass, replace
import numpy as np
from wrs import wum, wuc, wsso, wssop
from wrs.scene.collision_shape import CapsuleCollisionShape, OBBCollisionShape
from wrs.robots.base.mech_structure import MechStruct, Link, Joint
from wrs.robots.base.mech_base import MechBase
from wrs.physics.inertial import inertia_from_collisions, inertia_sphere
from .geometry import leaf_batches, leaf_mesh
from .static import make_branch, make_fruit, validate_render_config


@dataclass
class DynamicPlantInstance:
    spec: object
    dynamics: object
    mech: MechBase
    cluster_links: dict
    branch_objects: list
    leaf_objects: list
    fruits: dict
    foliage_proxies: dict
    semantic_groups: dict
    collision_roles: dict
    segment_clusters: dict
    leaf_clusters: dict
    cluster_frames: dict

    @property
    def fruit_objects(self):
        return list(self.fruits.values())

    def add_to_scene(self, scene):
        self.mech.add_to_scene(scene)
        return self

    def remove_from_scene(self, scene):
        self.mech.remove_from_scene(scene)

    def set_pos_rotmat(self, pos=None, rotmat=None):
        self.mech.set_pos_rotmat(pos, rotmat)
        return self

    def summary(self):
        return self.spec.summary() | dict(dynamic_cluster_count=len(self.dynamics.clusters),
            passive_dof=self.mech.ndof, foliage_proxy_count=sum(map(len, self.foliage_proxies.values())),
            leaf_batch_count=len(self.leaf_objects), physics_link_count=len(self.mech.runtime_lnks))


def _proxies(leaves, shape, frame, settings):
    """1..3 boxes fit blade vertices in cluster axes, split along longest span.

    This is a coarse contact envelope, not a claim of leaf-level contact fidelity.
    """
    if not leaves:
        return []
    points = [(leaf_mesh(p.length, p.width, shape)[0] @ np.asarray(p.rotmat).T + p.position - frame[:3, 3]) @ frame[:3, :3]
              for p in leaves]
    centers = np.array([p.mean(axis=0) for p in points])
    axis = int(np.argmax(np.ptp(centers, axis=0)))
    groups = np.array_split(np.argsort(centers[:, axis], kind='stable'), min(settings.per_cluster, len(leaves)))
    result = []
    for group in groups:
        vertices = np.concatenate([points[i] for i in group])
        lo, hi = vertices.min(axis=0), vertices.max(axis=0)
        half = np.maximum((hi - lo) / 2 + settings.padding, settings.minimum_half_extent)
        result.append(((lo + hi) / 2, half))
    return result


class DynamicPlantBuilder:
    def __init__(self, config):
        self.config = config

    def build(self, spec, dynamics, *, pos=None, rotmat=None):
        dynamics.validate(spec)
        visual = validate_render_config(spec, self.config)
        owners, by_id = dynamics.segment_clusters(spec), spec.skeleton.by_id
        ordered = sorted(dynamics.clusters, key=lambda c: spec.skeleton.depth(c.root_segment))
        frames = {None: np.eye(4)}
        frames.update({c.id: wum.tf_from_pos_rotmat(by_id[c.root_segment].start, by_id[c.root_segment].frame) for c in ordered})
        structure, links = MechStruct(), {None: Link()}
        links[None].name = 'plant_static'
        structure.add_lnk(links[None])
        for cluster in ordered:
            profile = dynamics.profiles[cluster.profile]
            parent = links[cluster.parent_cluster]
            joint_tf = np.linalg.inv(frames[cluster.parent_cluster]) @ frames[cluster.id]
            for index in range(cluster.dof):
                link = Link()
                link.name = cluster.id if index == cluster.dof - 1 else f'{cluster.id}_bend_x'
                if index < cluster.dof - 1:
                    link.set_inertia(inertia_sphere(profile.spacer_mass, profile.spacer_radius),
                                     np.zeros(3), profile.spacer_mass)
                axis = (1, 0, 0) if index == 0 else (0, 1, 0)
                structure.add_jnt(Joint(wuc.JntType.REVOLUTE, parent, link, axis,
                    pos=joint_tf[:3, 3], rotmat=joint_tf[:3, :3],
                    lmt_lo=profile.limits[0], lmt_up=profile.limits[1], actuated=False,
                    stiffness=profile.stiffness, damping=profile.damping, springref=profile.springref,
                    frictionloss=profile.frictionloss, armature=profile.armature))
                parent, joint_tf = link, np.eye(4)
            links[cluster.id] = parent
        inertia_shapes = {c.id: [] for c in ordered}
        volumes = {c.id: 0.0 for c in ordered}
        roles = {}
        for stem in spec.skeleton.segments:
            owner = owners[stem.id]
            branch = make_branch(stem, visual, spec.metadata.get('scale_multiplier', 1.0))
            local = np.linalg.inv(frames[owner]) @ branch.tf
            link = links[owner]
            for model in branch.visuals:
                tf = local @ model.loc_tf
                model.set_pos_rotmat(tf[:3, 3], tf[:3, :3])
                link.add_visual(model, auto_make_collision=False)
            shape = CapsuleCollisionShape(max(stem.radius_start, stem.radius_end), stem.length / 2,
                         rotmat=local[:3, :3], pos=local[:3, 3] + local[:3, :3] @ [0, 0, stem.length / 2])
            if stem.collidable:
                link.add_collision(shape)
                roles[shape] = 'TRUNK' if stem.order == 0 else ('COMPLIANT_BRANCH' if owner else 'HARD_BRANCH')
            if owner is not None:
                inertia_shapes[owner].append(shape)
                volumes[owner] += np.pi * stem.length / 3 * (stem.radius_start ** 2 + stem.radius_start * stem.radius_end + stem.radius_end ** 2)
        leaf_groups = {key: [leaf for leaf in spec.leaves if owners[leaf.parent_segment] == key] for key in links}
        for cluster in ordered:
            p = dynamics.profiles[cluster.profile]
            mass = max(p.minimum_mass, p.wood_density * volumes[cluster.id] + p.leaf_mass * len(leaf_groups[cluster.id]))
            com, inertia = inertia_from_collisions(inertia_shapes[cluster.id], mass)
            links[cluster.id].set_inertia(inertia, com, mass)
        structure.compile()
        mech = MechBase(structure=structure, pos=pos, rotmat=rotmat, is_floating=False)
        # Native STATIC collision role is a filter, independent of DOF: it
        # excludes plant/plant pairs while robot/manipulable ACTIVE bodies hit it.
        for link in mech.runtime_lnks:
            link.collision_group = wuc.CollisionGroup.STATIC
        runtime = {key: mech.runtime_lnks[structure.compiled.lidx_map[link]] for key, link in links.items()}
        # clone() copied collision shapes; re-key roles to actual runtime shapes.
        collision_roles = {actual: roles[original] for key, link in links.items()
                           for original, actual in zip(link.collisions, runtime[key].collisions)}
        leaves, leaf_owners, proxies, fruits = [], {}, {}, {}
        for owner, placements in leaf_groups.items():
            frame = frames[owner]
            for shade, (vs, fs) in leaf_batches(replace(spec, leaves=placements)).items():
                vs = (vs - frame[:3, 3]) @ frame[:3, :3]
                obj = wssop.mesh(vs, fs, rgb=visual['leaf_colors'][shade], name=f'leaves_{owner or "static"}_{shade}')
                mech.mount(obj, runtime[owner], update=True)
                leaves.append(obj)
                leaf_owners[obj] = owner
            if owner is None:
                continue
            proxies[owner] = []
            for i, (center, half) in enumerate(_proxies(placements, spec.leaf_shape, frame, dynamics.foliage_proxy)):
                obj = wsso.SceneObject(name=f'foliage_proxy_{owner}_{i}')
                shape = OBBCollisionShape(half_extents=half)
                obj.add_collision(shape)
                # The cluster's explicit inertia already includes lumped leaves.
                obj.set_inertia(np.zeros((3, 3)), np.zeros(3), 0.0)
                mech.mount(obj, runtime[owner], wum.tf_from_pos_rotmat(center), update=True)
                proxies[owner].append(obj)
                collision_roles[shape] = 'FOLIAGE'
        cluster_map = {c.id: c for c in ordered}
        for fruit in spec.fruits:
            owner = owners[fruit.parent_segment]
            obj = make_fruit(fruit, visual)
            if owner is not None:
                density = dynamics.profiles[cluster_map[owner].profile].fruit_density
                mass = density * 4 / 3 * np.pi * fruit.radius ** 3
                obj.set_inertia(inertia_sphere(mass, fruit.radius), np.zeros(3), mass)
            local = np.linalg.inv(frames[owner]) @ wum.tf_from_pos_rotmat(fruit.position)
            mech.mount(obj, runtime[owner], local, update=True)
            fruits[fruit.id] = obj
            collision_roles[obj.collisions[0]] = 'FRUIT'
        semantics = dict(TRUNK=[runtime[None]], HARD_BRANCH=[runtime[None]],
                         COMPLIANT_BRANCH=[runtime[c.id] for c in ordered],
                         FOLIAGE=[obj for group in proxies.values() for obj in group], FRUIT=list(fruits.values()))
        return DynamicPlantInstance(spec, dynamics, mech, runtime, list(runtime.values()), leaves, fruits,
                                    proxies, semantics, collision_roles, owners, leaf_owners, frames)
