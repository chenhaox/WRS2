"""Collision evidence is independent of near bands and retains unknown outcomes."""
import importlib.util
import unittest
from unittest.mock import patch
import numpy as np
from wrs.assembly import ContactModel, SDFCollisionChecker
from wrs.assembly.primitives import box, cylinder, rectangle, pose


@unittest.skipUnless(importlib.util.find_spec('open3d'), 'optional open3d is absent')
class CollisionTests(unittest.TestCase):
    def test_separation_touching_penetration_and_explicit_pose(self):
        a = ContactModel(box(), 'a')
        b = ContactModel(box(), 'b', pose((0, 0, .2)))
        checker = SDFCollisionChecker(max_query_points=4000)
        with patch('wrs.assembly.geometry.proximity.MeshProximity.classify_overlap', side_effect=AssertionError('mesh fallback')):
            clear = checker.query(a, b)
            self.assertEqual(clear['status'], 'separated')
            self.assertEqual(clear['query_points'], 0)
            touch = checker.query(a, b, tf_b=pose((0, 0, .1)))
            self.assertEqual(touch['status'], 'unknown')
            hit = checker.query(a, b, tf_b=pose((0, 0, .09)))
            self.assertEqual(hit['status'], 'penetrating')
            self.assertLess(hit['witness']['distance_m'], -hit['witness']['error_guard_m'])
            self.assertNotEqual(clear['state_digest'], hit['state_digest'])
            self.assertEqual(checker.query(b.at(pose((0, 0, .09))), a)['status'], 'penetrating')

    def test_containment_identical_and_thin_crossing(self):
        checker = SDFCollisionChecker()
        a = ContactModel(box(), 'a')
        self.assertEqual(checker.query(a, ContactModel(box((.01, .01, .01)), 'b'))['status'], 'penetrating')
        same = checker.query(a, ContactModel(a.geometry, 'b'))
        self.assertEqual(same['reason'], 'shared_sdf_interior_witness')
        self.assertLess(same['witness']['source_distance_m'], 0)
        x = ContactModel(box((.1, .0002, .0002)), 'x')
        y = ContactModel(box((.0002, .1, .0002)), 'y')
        self.assertEqual(checker.query(x, y)['status'], 'penetrating')

    def test_positive_sdf_bounds_and_budget_limit(self):
        a, b = ContactModel(box(), 'a'), ContactModel(box(), 'b', pose((0, 0, .3)))
        result = SDFCollisionChecker(use_aabb=False).query(a, b)
        self.assertEqual(result['reason'], 'positive_sdf_cell_bounds')
        self.assertTrue(all(s['unresolved_area_m2'] == 0 for s in result['sides']))
        tube = ContactModel(cylinder(.015, .02, 32, inner_radius=.01), 'tube')
        shaft = ContactModel(cylinder(.0098, .01, 32), 'shaft')
        result = SDFCollisionChecker(max_query_points=8).query(tube, shaft)
        self.assertEqual(result['status'], 'unknown')
        self.assertLessEqual(result['query_points'], 8)
        self.assertTrue(any(s['unresolved_area_m2'] > 0 for s in result['sides']))

    def test_unsigned_target_does_not_imply_free_but_surface_witness_is_usable(self):
        plane = ContactModel(rectangle((.01, .01)), 'plane')
        solid = ContactModel(box(), 'solid')
        with self.assertRaisesRegex(ValueError, 'watertight'):
            SDFCollisionChecker().prepare(plane)
        hit = SDFCollisionChecker(open_surface='unsigned').query(plane, solid)
        self.assertEqual(hit['status'], 'penetrating')
        self.assertFalse(hit['fields']['a']['signed'])
        self.assertEqual(hit['reason'], 'negative_sdf_surface_witness')

    def test_invalid_options(self):
        for kwargs in ({'resolution_m': 0}, {'resolution_m': np.nan}, {'max_query_points': 0},
                       {'batch_size': -1}, {'penetration_tol_m': -1}):
            with self.assertRaises(ValueError):
                SDFCollisionChecker(**kwargs)


class CellKernelsTests(unittest.TestCase):
    def test_clipped_area_centroid_winding_and_empty_cells(self):
        from wrs.assembly.contact._sdf_cells import clip_cells, measure_cells, split_triangles
        triangle = np.array([[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]])
        triangles = np.tile(triangle, (3, 1, 1))
        scores = np.array([[[-.2, .8, -.2], [-.3, -.3, .7], [1, 1, 1]],
                           [[1, 1, 1]]*3, [[-1, -1, -1]]*3])
        polygons, counts = clip_cells(triangles, scores)
        areas, centers = measure_cells(polygons, counts)
        np.testing.assert_allclose(areas, [.125, .5, 0], atol=1e-14)
        np.testing.assert_allclose(centers[0], [.2+.5/3, .3+.5/3, 0], atol=1e-14)
        children = split_triangles(triangles).reshape(-1, 3, 3)
        normals = np.cross(children[:, 1]-children[:, 0], children[:, 2]-children[:, 0])
        self.assertTrue(np.all(normals[:, 2] > 0))
        np.testing.assert_allclose(np.linalg.norm(normals, axis=1).reshape(-1, 2).sum(1)/2, .5)
