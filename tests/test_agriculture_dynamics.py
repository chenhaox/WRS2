"""Native WRS/MuJoCo integration without viewer/UI tests."""
from copy import deepcopy
import unittest
import numpy as np
from wrs import wss, wum, wssop, wuc
from agriculture.config import load_config
from agriculture.generator import generate
from agriculture.dynamics import PlantDynamicsSpec
from agriculture.dynamic import DynamicPlantBuilder
from agriculture.static import StaticPlantBuilder
from agriculture.spec import PlantSpec, PlantSkeleton, StemSegment, LeafPlacement, LeafShape
from wrs.physics.mj_env import MJEnv
from wrs.scene.collision_shape import CapsuleCollisionShape, SphereCollisionShape, OBBCollisionShape
from examples.agriculture.push_experiment import PushExperiment


class PlantIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config()
        cls.spec = generate(cls.config)
        cls.dynamics = PlantDynamicsSpec.from_config(cls.spec, cls.config)

    def build(self):
        return DynamicPlantBuilder(self.config).build(self.spec, self.dynamics)

    def test_same_spec_static_dynamic_rest_and_rigid_following(self):
        original = self.spec.to_json()
        static = StaticPlantBuilder(self.config).build(self.spec)
        dynamic = self.build()
        for key in static.fruits:
            np.testing.assert_allclose(static.fruits[key].pos, dynamic.fruits[key].pos, atol=2e-7)
        positions = [v.geom.vs @ (o.tf @ v.loc_tf)[:3, :3].T + (o.tf @ v.loc_tf)[:3, 3]
                     for o in dynamic.branch_objects + dynamic.leaf_objects + dynamic.fruit_objects for v in o.visuals]
        lo, hi = self.spec.bounds()
        self.assertTrue(np.all(np.concatenate(positions) >= lo - 5e-7))
        self.assertTrue(np.all(np.concatenate(positions) <= hi + 5e-7))
        locals_before = {obj: np.linalg.inv(m.plnk.tf) @ obj.tf for obj, m in dynamic.mech._mountings.items()}
        dynamic.mech.fk(np.linspace(-.1, .1, dynamic.mech.ndof))
        dynamic.set_pos_rotmat((.7, -.3, .2), wum.rotmat_from_axangle([0, 0, 1], .5))
        for obj, m in dynamic.mech._mountings.items():
            np.testing.assert_allclose(obj.tf, m.plnk.tf @ locals_before[obj], atol=3e-7)
        self.assertEqual(original, self.spec.to_json())
        self.assertLess(len(dynamic.leaf_objects), 3 * (len(self.dynamics.clusters) + 1) + 1)

    def test_native_contact_shapes_no_self_contact_no_leaf_dofs(self):
        plant = self.build(); scene = wss.Scene(); plant.add_to_scene(scene)
        env = MJEnv(scene, require_ctrl=True)
        self.assertEqual((env.model.nv, env.model.nu), (11, 0))
        self.assertTrue(all(not leaf.collisions for leaf in plant.leaf_objects))
        for shape, role in plant.collision_roles.items():
            self.assertIsInstance(shape, {'TRUNK': CapsuleCollisionShape, 'HARD_BRANCH': CapsuleCollisionShape,
                'COMPLIANT_BRANCH': CapsuleCollisionShape, 'FRUIT': SphereCollisionShape, 'FOLIAGE': OBBCollisionShape}[role])
        env.step(.1)
        self.assertEqual(env.data.ncon, 0)
        self.assertEqual(env.model.nmesh, 0)
        self.assertTrue(all(obj.mounted_by is plant.mech and not obj.is_floating for obj in plant.fruits.values()))
        self.assertEqual(plant.summary()['foliage_proxy_count'], 465)
        self.assertEqual(plant.summary()['foliage_proxy_object_count'], 7)
        self.assertLessEqual(sum(map(len, plant.foliage_proxies.values())), len(self.dynamics.clusters))

    def test_empty_foliage_fruit_and_nested_clusters(self):
        spec = deepcopy(self.spec)
        spec.leaves.clear(); spec.fruits.clear()
        c = deepcopy(self.config)
        c['dynamics']['cluster_roots'] = [dict(id='outer', root_segment='front_middle_shoot_01', dof=2, profile='foliage'),
             dict(id='tip', root_segment='twig_front_middle_shoot_01_00', dof=1, profile='foliage')]
        d = PlantDynamicsSpec.from_config(spec, c)
        self.assertEqual(d.clusters[1].parent_cluster, 'outer')
        plant = DynamicPlantBuilder(c).build(spec, d)
        scene = wss.Scene(); plant.add_to_scene(scene)
        env = MJEnv(scene, require_ctrl=True)
        self.assertEqual(env.model.nv, 3)
        self.assertEqual(plant.leaf_objects, [])
        self.assertEqual(plant.fruits, {})
        env.step(.01)
        self.assertTrue(np.isfinite(env.data.qpos).all())

    def test_real_proxy_push_rebound_and_settling(self):
        plant = self.build(); scene = wss.Scene(); plant.add_to_scene(scene)
        experiment = PushExperiment(plant, scene, self.config['push_demo'])
        experiment.step(experiment.duration)
        result = experiment.report()
        self.assertGreater(result['proxy_contact_samples'], 0)
        self.assertGreater(result['peak_joint_displacement_rad'], .03)
        self.assertGreater(result['peak_leaf_displacement_m'], .005)
        self.assertGreater(result['peak_fruit_displacement_m'], .005)
        self.assertGreaterEqual(result['release_oscillation_crossings'], 2)
        self.assertLess(result['final_joint_residual_rad'], .01)
        self.assertLess(result['late_joint_peak_to_peak_rad'], .001)

    def test_leaf_gaps_and_normal_clearance_in_native_collision(self):
        # Three separated thin leaves on one moving shoot. A probe can pass
        # between them and above them, but must hit each actual blade.
        spec = PlantSpec('three_leaves', PlantSkeleton([
            StemSegment('root', None, (0, 0, 0), (0, 0, .2), .01, .008, 0),
            StemSegment('shoot', 'root', (0, 0, .2), (0, 0, .4), .003, .002, 1),
            *[StemSegment(f'twig_{i}', 'shoot', (0, 0, .32), (.02, y, .32), .0005, .0004, 2,
                          collidable=False, attachment_t=.6) for i, y in enumerate((-.06, 0, .06))]]),
            leaves=[LeafPlacement(f'twig_{i}', (.02, y, .32), np.eye(3).tolist(), .1, .02, 0)
                    for i, y in enumerate((-.06, 0, .06))], leaf_shape=LeafShape(fold=0, curl=0, droop=0))
        config = deepcopy(self.config)
        config['dynamics']['cluster_roots'] = [dict(id='shoot', root_segment='shoot', dof=1, profile='foliage')]
        plant = DynamicPlantBuilder(config).build(spec, PlantDynamicsSpec.from_config(spec, config),
            pos=(.4, -.3, .1), rotmat=wum.rotmat_from_euler(.4, -.2, .7))
        plant.mech.fk([.25])
        scene = wss.Scene()
        plant.add_to_scene(scene)
        probe = wssop.sphere(radius=.003, collision_type=wuc.CollisionType.SPHERE,
                             mass=.01, is_floating=True)
        probe.collision_group = wuc.CollisionGroup.ACTIVE
        probe.add_to_scene(scene)
        env = MJEnv(scene)
        target = plant.foliage_proxies['shoot'][0]
        self.assertEqual(len(plant.foliage_proxies['shoot']), 1)
        self.assertEqual(len(target.collisions), 9)
        self.assertEqual(len(plant.leaf_objects), 1)
        # Keep the moving cluster and nontrivial world transform in the test;
        # wrong leaf/cluster/world transforms otherwise pass at the origin.
        tf = plant.cluster_links['shoot'].tf @ np.linalg.inv(plant.cluster_frames['shoot'])
        for position, should_hit in [((.07, -.06, .32), True), ((.07, .06, .32), True),
                                     ((.07, 0, .32), True), ((.07, -.03, .30), False),
                                     ((.07, -.03, .32), False), ((.07, -.03, .34), False),
                                     ((.07, .03, .32), False), ((.07, -.06, .335), False),
                                     ((.14, -.06, .32), False)]:
            with self.subTest(position=position):
                probe.pos = tf[:3, :3] @ position + tf[:3, 3]
                env.sync.push_one_sobj_qpos(probe, probe.quat, probe.pos)
                env.runtime.forward()
                self.assertEqual(env.data.ncon > 0, should_hit)


if __name__ == '__main__': unittest.main()
