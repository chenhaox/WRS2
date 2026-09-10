"""SDF geometry, uncertainty, budgets and full shaft-wall regression."""
import importlib.util
import unittest
from dataclasses import replace
import numpy as np
from wrs.assembly import (ContactModel, ContactAnalyzer, ContactConfig, SDFContactBackend,
                          SDFConfig, GridSDF, MeshData)
from wrs.assembly.primitives import box, cylinder, rectangle, pose


def plane_grid(direction=1, offset=0):
    axes = np.linspace(-.002, .002, 9)
    _, _, z = np.meshgrid(axes, axes, axes, indexing='ij')
    return GridSDF(direction*z+offset, [-.002]*3, .0005, error_bound_m=0)


class GridTests(unittest.TestCase):
    def test_trilinear_sign_gradient_domain_and_immutable_identity(self):
        axes = np.arange(4)*.001
        x, y, z = np.meshgrid(axes, axes, axes, indexing='ij')
        values = (x+2*y+2*z-.005)/3
        grid = GridSDF(values, [0, 0, 0], .001, error_bound_m=1e-9)
        q = np.array([[.0003, .0012, .0004], [.003, .003, .003], [-.0001, 0, 0]])
        answer = grid.query(q)
        np.testing.assert_allclose(answer.values_m[:2], (q[:2] @ [1, 2, 2]-.005)/3, atol=1e-15)
        np.testing.assert_allclose(answer.normals_local[:2], np.tile(np.array([1, 2, 2])/3, (2, 1)))
        self.assertEqual(answer.valid.tolist(), [True, True, False])
        key = grid.cache_key
        values[:] = 0
        self.assertEqual(grid.cache_key, key)
        with self.assertRaises(ValueError):
            grid.values_m[0, 0, 0] = 1
        self.assertNotEqual(key, replace(grid, error_bound_m=1e-6).cache_key)

    def test_native_fields_use_local_frames_and_detect_mismatch(self):
        a = ContactModel(rectangle((.001, .001)), 'a', representations={'sdf': plane_grid()})
        b = ContactModel(rectangle((.001, .001), upward=False), 'b',
                         pose((0, 0, .0002)), {'sdf': plane_grid(-1)})
        backend = SDFContactBackend(sdf_config=SDFConfig(validate_mesh_overlap=False))
        analyzer = ContactAnalyzer(backend, config=ContactConfig(surface_resolution_m=.00005))
        result = analyzer.analyze_pair(a, b)
        self.assertTrue(result.patches)
        self.assertTrue(all(p.classification == 'near' for p in result.patches))
        for side in ('a', 'b'):
            self.assertAlmostEqual(sum(p.area_m2 for p in result.patches if p.sampling_side == side), 1e-6)
        self.assertEqual(result.pair_diagnostics[0]['overlap']['status'], 'unknown')
        self.assertTrue(all(p.quality == 'estimated' for p in result.patches))
        rotation = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        shifted = analyzer.analyze_pair(a, b, tf_a=pose((1, 2, 3), rotation),
                                        tf_b=pose((1.0002, 2, 3), rotation))
        self.assertAlmostEqual(sum(p.area_m2 for p in shifted.patches), 2e-6)
        self.assertTrue(all(np.allclose(p.normals_a_world, [1, 0, 0]) for p in shifted.patches))
        self.assertGreater(backend.cache_hits, 0)
        bad = ContactModel(a.geometry, 'bad', representations={'sdf': plane_grid(offset=.001)})
        with self.assertRaisesRegex(ValueError, 'does not match'):
            analyzer.analyze_pair(a, bad)

    def test_native_grid_domain_and_budget_stay_uncertain(self):
        a = ContactModel(rectangle((.001, .001)), 'a', representations={'sdf': plane_grid()})
        b = ContactModel(rectangle((.001, .001), upward=False), 'b',
                         pose((0, 0, .004)), {'sdf': plane_grid(-1)})
        result = ContactAnalyzer(SDFContactBackend(sdf_config=SDFConfig(validate_mesh_overlap=False))).analyze_pair(a, b)
        self.assertFalse(result.patches)
        self.assertGreater(sum(r['invalid_field_area_m2'] for r in result.pair_diagnostics[0]['curved_coverage']), 0)
        low = SDFContactBackend(sdf_config=SDFConfig(max_query_points=1, validate_mesh_overlap=False))
        result = ContactAnalyzer(low).analyze_pair(a, b.at(pose((0, 0, .0002))))
        self.assertLessEqual(result.statistics['sdf_query_points'], 1)
        self.assertGreater(sum(r['unprocessed_area_m2'] for r in result.pair_diagnostics[0]['curved_coverage']), 0)


@unittest.skipUnless(importlib.util.find_spec('open3d'), 'optional open3d dependency is absent')
class MeshSDFTests(unittest.TestCase):
    def test_box_distances_sign_and_large_local_origin(self):
        offset = np.array([1e6, -2e6, 3e6])
        raw = box()
        model = ContactModel(MeshData(raw.vertices+offset, raw.faces), 'box')
        _, field = SDFContactBackend().prepare(model)
        answer = field.query(np.array([[0, 0, 0], [0, 0, .2]])+offset)
        np.testing.assert_allclose(answer.values_m, [-.05, .15], atol=1e-8)
        self.assertTrue(np.all(answer.signed))
        self.assertEqual(field.query(np.empty((0, 3))).values_m.shape, (0,))

    def test_open_surface_requires_explicit_unsigned_mode(self):
        model = ContactModel(rectangle(), 'plane')
        with self.assertRaisesRegex(ValueError, 'watertight'):
            SDFContactBackend().prepare(model)
        _, field = SDFContactBackend(sdf_config=SDFConfig(open_surface='unsigned')).prepare(model)
        answer = field.query([[0, 0, -.001]])
        self.assertFalse(answer.signed[0])
        self.assertAlmostEqual(answer.values_m[0], .001, places=8)

    def test_shaft_full_band_clearance_and_budget_accounting(self):
        a = ContactModel(cylinder(radius=.015, inner_radius=.01, height=.02, sections=32), 'tube')
        b = ContactModel(cylinder(radius=.0098, height=.01, sections=32), 'shaft')
        cfg = ContactConfig(surface_resolution_m=.001, normal_angle_rad=.55, max_cells=20000,
                            max_triangle_tests=100000)
        backend = SDFContactBackend()
        result = ContactAnalyzer(backend, config=cfg).analyze_pair(a, b)
        patches = [p for p in result.patches if p.sampling_side == 'b']
        self.assertAlmostEqual(sum(p.area_m2 for p in patches), 2*32*.0098*np.sin(np.pi/32)*.01, places=11)
        self.assertEqual(sum(len(p.regions) for p in patches), 1)
        self.assertTrue(all(p.classification == 'near' for p in patches))
        self.assertFalse(any(p.classification == 'active' for p in result.patches))
        self.assertEqual(result.pair_diagnostics[0]['overlap']['status'], 'separated')
        for report in result.pair_diagnostics[0]['curved_coverage']:
            self.assertEqual(report['unprocessed_area_m2'], 0)
            total = sum(report[k] for k in ('estimated_band_area_m2', 'excluded_area_m2',
                                             'boundary_uncertain_area_m2', 'invalid_field_area_m2',
                                             'unprocessed_area_m2'))
            self.assertAlmostEqual(total, report['source_area_m2'], places=11)
        low = ContactAnalyzer(SDFContactBackend(sdf_config=SDFConfig(max_query_points=8)), config=cfg).analyze_pair(a, b)
        self.assertLessEqual(low.statistics['sdf_query_points'], 8)
        self.assertTrue(all(r['unprocessed_area_m2'] > 0 for r in low.pair_diagnostics[0]['curved_coverage']))
        self.assertNotEqual(low.state_digest, result.state_digest)

    def test_penetration_is_not_hidden_by_local_band(self):
        a, b = ContactModel(box(), 'a'), ContactModel(box(), 'b', pose((0, 0, .09)))
        cfg = ContactConfig(surface_resolution_m=.004, max_cells=10000)
        result = ContactAnalyzer('sdf', config=cfg).analyze_pair(a, b)
        self.assertEqual(result.pair_diagnostics[0]['overlap']['status'], 'penetrating')
        self.assertTrue(result.pair_diagnostics[0]['negative_sdf_witnesses'])
        self.assertFalse(any(p.classification == 'active' for p in result.patches))
