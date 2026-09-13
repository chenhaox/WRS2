import unittest
import numpy as np
from wrs.assembly import MeshData, GeometryConfig
from wrs.assembly.geometry.primitives import box, rectangular_ring, cylinder
from wrs.assembly.geometry.preprocess import prepare_mesh
from wrs.assembly.geometry.surfaces import extract_surfaces
from wrs.assembly.geometry.planar import polygon_measure


class SurfaceTests(unittest.TestCase):
    def test_box_and_ring(self):
        prep = prepare_mesh(box())
        self.assertTrue(prep.is_closed and prep.orientation_reliable)
        patches = extract_surfaces(prep)
        self.assertEqual(len(patches), 6)
        self.assertTrue(all(p.kind == 'plane' for p in patches))
        ring = extract_surfaces(prepare_mesh(rectangular_ring()))[0]
        self.assertEqual(len(ring.boundary_loops), 2)
        areas = [polygon_measure((loop-ring.origin) @ ring.basis)[0] for loop in ring.boundary_loops]
        self.assertAlmostEqual(sum(areas), .01-.0016)
        self.assertTrue(min(areas) < 0 < max(areas))

    def test_winding_and_provenance(self):
        original = box()
        faces = original.faces.copy()
        faces[0] = faces[0, ::-1]
        faces = np.vstack((faces, faces[1], [0,0,0]))
        prep = prepare_mesh(MeshData(original.vertices, faces))
        self.assertEqual(len(prep.mesh.faces), 12)
        self.assertIn(12, prep.source_face_ids[1])
        self.assertTrue(prep.orientation_reliable)
        self.assertIn('winding_repaired', prep.diagnostics)

    def test_open_mesh_unknown_unless_declared(self):
        ring = rectangular_ring()
        prep = prepare_mesh(MeshData(ring.vertices, ring.faces))
        self.assertFalse(prep.orientation_reliable)
        self.assertIn('open_boundary', prep.diagnostics)

    def test_cylinder_does_not_become_a_plane(self):
        patches = extract_surfaces(prepare_mesh(cylinder(sections=48)))
        self.assertEqual(sum(p.kind == 'plane' for p in patches), 2)
        self.assertEqual(sum(p.kind == 'general' for p in patches), 1)
        self.assertGreater(max(p.residual_m for p in patches), .001)

    def test_narrow_gap_not_welded_by_default(self):
        vs = [[0,0,0], [1,0,0], [0,1,0], [0,0,1e-7], [1,0,1e-7], [0,1,1e-7]]
        mesh = MeshData(vs, [[0,1,2], [3,4,5]], 'trusted')
        prep = prepare_mesh(mesh)
        self.assertEqual(len(prep.components), 2)
        self.assertEqual(prep.max_vertex_displacement_m, 0)
        merged = prepare_mesh(mesh, config=GeometryConfig(weld_m=2e-7))
        self.assertLessEqual(merged.max_vertex_displacement_m, 2e-7)
        self.assertEqual(len(merged.mesh.faces), 1)

    def test_nonmanifold_and_face_permutation(self):
        mesh = box()
        vs = np.vstack((mesh.vertices,[[0,-.08,0]]))
        bad = MeshData(vs,np.vstack((mesh.faces,[[0,1,len(vs)-1]])))
        prep = prepare_mesh(bad)
        self.assertIn('nonmanifold_edges',prep.diagnostics)
        self.assertFalse(prep.orientation_reliable)
        permuted = MeshData(mesh.vertices,mesh.faces[::-1])
        patches = extract_surfaces(prepare_mesh(permuted))
        self.assertEqual(len(patches),6)
        self.assertTrue(all(abs(p.area_m2-.01)<1e-12 for p in patches))
