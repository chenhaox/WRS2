import unittest
import numpy as np
from wrs.assembly import MeshData, Part, GeometryConfig
from wrs.assembly.geometry.primitives import box, rectangle, pose, combine
from wrs.assembly.geometry.mesh_bvh import closest_on_triangle, triangle_pair
from wrs.assembly.geometry.proximity import MeshProximity
from wrs.assembly.geometry.transforms import rotation_xyz


class ProximityTests(unittest.TestCase):
    def test_primitive_witnesses(self):
        tri = np.array([[0.,0,0], [2,0,0], [0,2,0]])
        np.testing.assert_allclose(closest_on_triangle(np.array([.5,.5,1]), tri), [.5,.5,0])
        flat = np.array([[0.,0,0], [1,0,0], [2,0,0]])
        np.testing.assert_allclose(closest_on_triangle(np.array([.5,1,0]), flat), [.5,0,0])
        a = np.array([[-1.,0,0], [1,0,0], [0,-1,-1]])
        b = np.array([[0.,-1,1], [0,1,1], [1,0,2]])
        pa, pb, _ = triangle_pair(a, b)
        self.assertAlmostEqual(np.linalg.norm(pa-pb), 1)
        self.assertGreater(min(np.linalg.norm(a-pa, axis=1)), .5)
        self.assertGreater(min(np.linalg.norm(b-pb, axis=1)), .5)
        piercing = np.array([[.25,.25,-1], [.25,.25,1], [3,3,1]])
        pa, pb, crossed = triangle_pair(tri, piercing)
        self.assertTrue(crossed)
        np.testing.assert_allclose(pa, pb)

    def test_bvh_against_small_bruteforce(self):
        mesh = box()
        a, b = Part('a', mesh), Part('b', mesh)
        backend = MeshProximity()
        rng = np.random.default_rng(113)
        for translation in ((.13,.025,.03), (.06,.01,.02), (-.11,.08,.04)):
            ta, tb = pose(), pose(translation, rotation_xyz(rng.uniform(-.4,.4,3)))
            result = backend.pair_distance(a, ta, b, tb)
            av, bv = mesh.vertices, mesh.vertices @ tb[:3,:3].T + tb[:3,3]
            expected = min(np.linalg.norm(p-q) for x in av[mesh.faces] for y in bv[mesh.faces]
                           for p, q, _ in [triangle_pair(x,y)])
            self.assertEqual(result.status, 'complete')
            self.assertAlmostEqual(result.upper_bound_m, expected, places=11)
            self.assertEqual(result.lower_bound_m, result.upper_bound_m)

    def test_containment_identical_crossing_and_open(self):
        backend = MeshProximity()
        a, b = Part('outer',box()), Part('inner',box((.02,.02,.02)))
        self.assertEqual(backend.classify_overlap(a,pose(),b,pose()).status, 'penetrating')
        self.assertEqual(backend.classify_overlap(a,pose(),Part('same',box()),pose()).status, 'penetrating')
        crossed = backend.classify_overlap(a,pose(),Part('cross',box()),pose((.06,0,0),rotation_xyz([.1,.2,.3])))
        self.assertEqual(crossed.status, 'penetrating')
        self.assertIsNotNone(crossed.witness_world_m)
        self.assertEqual(backend.classify_overlap(a,pose(),Part('open',rectangle()),pose()).status, 'unknown')

    def test_budget_and_shared_cache_pose(self):
        backend = MeshProximity(cache_size=2)
        mesh = box()
        a, b = Part('a',mesh), Part('b',mesh)
        initial = backend.index(mesh)
        before = mesh.vertices.copy()
        for z, expected in ((.2,.1), (.3,.2)):
            q = backend.pair_distance(a,pose(),b,pose((0,0,z)))
            self.assertAlmostEqual(q.upper_bound_m, expected)
        self.assertIs(initial, backend.index(mesh))
        np.testing.assert_array_equal(mesh.vertices, before)
        q = backend.pair_distance(a,pose(),b,pose((0,0,.2)),budget=1)
        self.assertEqual(q.status, 'complete')
        self.assertEqual(q.triangle_tests, 1)
        self.assertAlmostEqual(q.lower_bound_m, .1)
        tilted = pose((.02, .01, .2), rotation_xyz([.1, .2, .3]))
        reference = backend.pair_distance(a, pose(), b, tilted)
        q = backend.pair_distance(a, pose(), b, tilted, budget=1)
        self.assertEqual(q.status, 'budget_exhausted')
        self.assertLessEqual(q.lower_bound_m, reference.upper_bound_m+1e-12)
        self.assertGreaterEqual(q.upper_bound_m, reference.upper_bound_m-1e-12)
        points = backend.closest_points(mesh, [[0,0,.2]], budget=1)
        self.assertEqual(points.status, 'budget_exhausted')
        self.assertEqual(points.completed_count, 0)
        self.assertEqual(backend.classify_overlap(a,pose(),b,pose((0,0,.1)),budget=1).status, 'unknown')

    def test_nested_shells_and_unrepaired_normals(self):
        outer, inner = box(), box((.04,.04,.04))
        shells = combine([outer, MeshData(inner.vertices, inner.faces[:,::-1])])
        backend = MeshProximity()
        prep, _ = backend.prepare(shells)
        self.assertFalse(prep.orientation_reliable)
        self.assertIn('nested_shells_require_trusted_winding', prep.diagnostics)
        faces = outer.faces.copy()
        faces[0] = faces[0,::-1]
        backend = MeshProximity(geometry_config=GeometryConfig(repair_orientation=False))
        self.assertFalse(backend.prepare(MeshData(outer.vertices,faces))[0].orientation_reliable)
