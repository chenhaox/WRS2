"""Regressions from the M1 audit: cache identity and sparse topology kernels."""
import importlib.util
import unittest
from dataclasses import replace
import numpy as np
from wrs.assembly import (MeshData, GeometryConfig, ContactModel, SDFConfig,
                          SDFContactBackend, SDFCollisionChecker)
from wrs.assembly.primitives import box, rectangle, rectangular_ring, pose
from wrs.assembly.geometry.preprocess import prepare_mesh
from wrs.assembly.geometry.planar import cell_regions, polygon_measure


class AuditTests(unittest.TestCase):
    def test_orphans_and_removed_faces_do_not_change_surface_bounds(self):
        mesh = box()
        dirty = MeshData(np.vstack((mesh.vertices, [[1e6, 2e6, 3e6], [99, 99, 99]])),
                         np.vstack((mesh.faces, [[8, 8, 8]])))
        clean = prepare_mesh(dirty)
        self.assertEqual(len(clean.mesh.vertices), 8)
        self.assertAlmostEqual(clean.areas_m2.sum(), .06)
        self.assertTrue(clean.is_closed and clean.orientation_reliable)
        self.assertEqual(clean.source_face_ids, tuple((i,) for i in range(12)))
        np.testing.assert_allclose(np.ptp(clean.mesh.vertices, axis=0), [.1]*3)

    def test_batched_t_junctions_holes_and_permutations(self):
        # One long edge meets two shorter edges; all three cells form a square.
        cells = [np.array(c, float) for c in (
            [[0, 0], [1, 0], [1, 2], [0, 2]],
            [[1, 0], [2, 0], [2, 1], [1, 1]],
            [[1, 1], [2, 1], [2, 2], [1, 2]])]
        for ordered in (cells, cells[::-1], [c[::-1] for c in cells]):
            groups, valid = cell_regions(ordered, 1e-9)
            self.assertTrue(valid)
            self.assertEqual(len(groups), 1)
            self.assertAlmostEqual(sum(polygon_measure(p)[0] for p in groups[0][1]), 4)
        ring = rectangular_ring()
        cells = list(ring.vertices[ring.faces][:, :, :2])
        groups, valid = cell_regions(cells, 1e-10)
        self.assertTrue(valid)
        self.assertEqual(len(groups[0][1]), 2)
        self.assertAlmostEqual(sum(polygon_measure(p)[0] for p in groups[0][1]), .0084)
        # Exercise multiple batches with a known boundary and many T junctions.
        all_cells = [c+[3*i, 0] for i in range(100) for c in cells]
        groups, valid = cell_regions(all_cells, 1e-10)
        self.assertTrue(valid)
        self.assertEqual(len(groups), 100)

    def test_intersections_on_rounding_boundary_share_one_vertex(self):
        # Two triangles cover one rectangle. Their shared corner differs only
        # by floating-point roundoff, on opposite sides of a grid half-step.
        width, height, tol = .0100000002, .02, 4e-10
        a = np.array([[0, 0], [width, 0], [np.nextafter(width, np.inf), height]])
        b = np.array([[0, 0], [np.nextafter(width, -np.inf), height], [0, height]])
        for dimension in (2, 3):
            cells = (a, b) if dimension == 2 else tuple(np.column_stack((p, np.zeros(3))) for p in (a, b))
            for ordered in (cells, cells[::-1]):
                groups, valid = cell_regions(ordered, tol)
                self.assertTrue(valid)
                self.assertEqual(len(groups), 1)
                self.assertEqual(len(groups[0][1]), 1)
                self.assertAlmostEqual(abs(polygon_measure(groups[0][1][0][:, :2])[0]), width*height)

    def test_topology_welding_does_not_close_a_gap_larger_than_tolerance(self):
        tol = 1e-6
        square = np.array([[0., 0.], [.01, 0.], [.01, .01], [0., .01]])
        groups, valid = cell_regions((square, square + [.01 + 1.1*tol, 0]), tol)
        self.assertTrue(valid)
        self.assertEqual(len(groups), 2)

    @unittest.skipUnless(importlib.util.find_spec('open3d'), 'optional open3d is absent')
    def test_field_reconfiguration_and_instance_identity(self):
        backend = SDFContactBackend(sdf_config=SDFConfig(open_surface='unsigned'))
        open_model = ContactModel(rectangle(), 'open')
        old = backend.prepare(open_model)
        self.assertIs(backend.prepare(open_model)[0], old[0])
        self.assertIs(backend.prepare(open_model)[1], old[1])
        backend.sdf_config = SDFConfig(open_surface='error')
        with self.assertRaisesRegex(ValueError, 'watertight'):
            backend.prepare(open_model)
        a = ContactModel(box(), 'a')
        before = backend.prepare(a)
        backend.geometry_config = GeometryConfig(weld_m=1e-8)
        self.assertIsNot(backend.prepare(a)[1], before[1])
        self.assertEqual(backend._mesh.geometry_config, backend.geometry_config)
        checker = SDFCollisionChecker()
        b = ContactModel(a.geometry, 'b', pose((0, 0, .2)))
        report = checker.query(a, b)
        renamed = checker.query(replace(a, name='other'), b)
        self.assertNotEqual(report['state_digest'], renamed['state_digest'])
