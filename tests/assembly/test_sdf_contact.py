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
    def test_batched_convex_upper_bound_matches_scalar_geometry(self):
        from wrs.assembly.contact.sdf_backend import _triangle_vertex_upper
        from wrs.assembly.geometry.mesh_bvh import closest_on_triangle
        rng = np.random.default_rng(20260910)
        sources = rng.normal(size=(256, 3, 3))
        targets = rng.normal(size=(256, 3, 3))
        targets[0] = 0
        targets[1, 2] = targets[1, 1]
        targets[2, 2] = targets[2, 0]+.5*(targets[2, 1]-targets[2, 0])
        for scale in (1, 1e-6, 1e4):
            actual = _triangle_vertex_upper(sources*scale, targets*scale)
            expected = [max(np.linalg.norm(p-closest_on_triangle(p, t*scale)) for p in s*scale)
                        for s, t in zip(sources, targets)]
            np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=scale*1e-14)

    def test_native_box_corners_do_not_remove_opposing_faces(self):
        axis = np.linspace(-.008, .008, 81)
        xyz = np.stack(np.meshgrid(axis, axis, axis, indexing='ij'), axis=-1)
        q = np.abs(xyz)-.002
        values = np.linalg.norm(np.maximum(q, 0), axis=-1)+np.minimum(q.max(axis=-1), 0)
        spacing = float(axis[1]-axis[0])
        grid = GridSDF(values, [axis[0]]*3, spacing, error_bound_m=np.sqrt(3)*spacing/2)
        a = ContactModel(box((.004, .004, .004)), 'a', representations={'sdf': grid})
        b = ContactModel(a.geometry, 'b', pose((0, 0, .005)), {'sdf': grid})
        backend = SDFContactBackend(sdf_config=SDFConfig(validate_mesh_overlap=False))
        cfg = ContactConfig(near_tol_m=.002, surface_resolution_m=.0002, normal_angle_rad=.55)
        result = ContactAnalyzer(backend, config=cfg).analyze_pair(a, b)
        for side in ('a', 'b'):
            patches = [p for p in result.patches if p.sampling_side == side]
            self.assertAlmostEqual(sum(p.area_m2 for p in patches), .004**2, places=12)
        for report in result.pair_diagnostics[0]['curved_coverage']:
            self.assertEqual(report['unprocessed_area_m2'], 0)

    def test_tilted_sdf_band_is_clipped_to_analytical_area(self):
        axis = np.linspace(-.002, .002, 9)
        x, _, z = np.meshgrid(axis, axis, axis, indexing='ij')
        slope, offset = .3, .0002
        norm = np.sqrt(1+slope*slope)
        field = GridSDF((offset+slope*x-z)/norm, [-.002]*3, .0005, error_bound_m=0)
        mesh = rectangle((.001, .001), upward=False)
        vertices = mesh.vertices.copy()
        vertices[:, 2] = offset+slope*vertices[:, 0]
        a = ContactModel(rectangle((.001, .001)), 'a', representations={'sdf': plane_grid()})
        b = ContactModel(MeshData(vertices, mesh.faces, 'trusted'), 'b', representations={'sdf': field})
        config = ContactConfig(near_tol_m=.0002, surface_resolution_m=.0002, normal_angle_rad=.55)
        backend = SDFContactBackend(sdf_config=SDFConfig(validate_mesh_overlap=False))
        report = ContactAnalyzer(backend, config=config).analyze_pair(a, b)
        area = sum(p.area_m2 for p in report.patches if p.sampling_side == 'a')
        expected = .001*(.0005+(config.near_tol_m*norm-offset)/slope)
        self.assertAlmostEqual(area, expected, places=13)

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
    def test_edge_gradient_does_not_jump_between_cap_and_wall(self):
        mesh = cylinder(radius=.0098, height=.01, sections=32)
        _, field = SDFContactBackend().prepare(ContactModel(mesh, 'shaft'))
        angles = np.arange(128)*2*np.pi/128
        points = np.column_stack((.01*np.cos(angles), .01*np.sin(angles), np.full(128, .0052)))
        answer = field.query(points)
        expected = points-answer.points_local_m
        expected /= np.linalg.norm(expected, axis=1)[:, None]
        np.testing.assert_allclose(answer.normals_local, expected, atol=1e-7)
        self.assertTrue(np.all((answer.normals_local[:, 2] > .62) & (answer.normals_local[:, 2] < .72)))
        shuffled = MeshData(mesh.vertices, mesh.faces[::-1])
        _, again = SDFContactBackend().prepare(ContactModel(shuffled, 'reordered'))
        np.testing.assert_allclose(again.query(points).normals_local, answer.normals_local, atol=1e-5)

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
        wall_a = [p for p in result.patches if p.sampling_side == 'a']
        # Every patch boundary is represented by cut polygons, not a ring of
        # full-triangle spikes. End extension is limited by the normal angle.
        all_vertices = np.concatenate([cell for p in wall_a for r in p.regions for cell in r.cells_world_m])
        self.assertLess(np.max(np.abs(all_vertices[:, 2])), .0055)
        self.assertTrue(any(len(cell) != 3 for p in wall_a for r in p.regions for cell in r.cells_world_m))
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

    def test_real_stl_bunny_is_explicitly_unsigned_and_closed_cylinder_signed(self):
        from pathlib import Path
        root = Path(__file__).resolve().parents[2]
        bunny = ContactModel.from_file(root/'bunny.stl', name='bunny', length_unit='m')
        with self.assertRaisesRegex(ValueError, 'watertight'):
            SDFContactBackend().prepare(bunny)
        _, field = SDFContactBackend(sdf_config=SDFConfig(open_surface='unsigned')).prepare(bunny)
        self.assertFalse(field.metadata['signed'])
        real = ContactModel.from_file(root/'examples/l1picking/cylinder.stl', name='cylinder', length_unit='m')
        _, field = SDFContactBackend().prepare(real)
        self.assertTrue(field.metadata['signed'])
        q = field.query([[0, 0, .03], [0, 0, .08]])
        self.assertLess(q.values_m[0], 0)
        self.assertGreater(q.values_m[1], 0)
