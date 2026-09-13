import unittest
import numpy as np
from wrs.assembly import Part, ContactConfig, MeshData, analyze_pair
from wrs.assembly.primitives import box, rectangle, rectangular_ring, pose, combine
from wrs.assembly.geometry.transforms import rotation_xyz


def active_area(result):
    return sum(p.area_m2 for p in result.patches if p.classification == 'active' and p.dimension == 2)


class PlanarTests(unittest.TestCase):
    def test_full_and_partial_boxes(self):
        a, b = Part('a', box()), Part('b', box())
        for x, expected in ((0, .01), (.025, .0075)):
            result = analyze_pair(a, pose(), b, pose((x,0,.1)))
            self.assertAlmostEqual(active_area(result), expected, places=10)
            self.assertEqual(result.pair_diagnostics[0]['overlap']['status'], 'touching')

    def test_ring_hole_and_retriangulation(self):
        a = Part('ring', rectangular_ring())
        for diagonal in (False, True):
            b = Part('cover', rectangle(upward=False, alternate_diagonal=diagonal))
            result = analyze_pair(a, pose(), b, pose())
            self.assertAlmostEqual(active_area(result), .0084, places=10)
            patch = next(p for p in result.patches if p.dimension == 2)
            self.assertEqual(len(patch.regions), 1)
            self.assertEqual(len(patch.regions[0].boundary_loops_world_m), 2)
            self.assertTrue(all(np.any(np.abs(q[:2]) >= .02-1e-10) for q in patch.points_a_world_m))
            self.assertAlmostEqual(patch.weights.sum(), patch.area_m2)

    def test_disconnected_regions(self):
        rect = rectangle((.02,.02))
        mesh = combine([MeshData(rect.vertices + [-.03,0,0], rect.faces, 'trusted'),
                        MeshData(rect.vertices + [.03,0,0], rect.faces, 'trusted')])
        result = analyze_pair(Part('a',mesh), pose(), Part('b',rectangle(upward=False)), pose())
        self.assertAlmostEqual(active_area(result), .0008)
        self.assertEqual(sum(len(p.regions) for p in result.patches if p.dimension == 2), 2)

    def test_positive_gap_is_not_active(self):
        a, b = Part('a',box()), Part('b',box())
        result = analyze_pair(a,pose(),b,pose((0,0,.1000005)))
        self.assertEqual(active_area(result), 0)
        self.assertTrue(any(p.classification == 'near' for p in result.patches))
        result = analyze_pair(a,pose(),b,pose((0,0,.1000005)), config=ContactConfig(idealize_contact=True))
        self.assertAlmostEqual(active_area(result), .01)

    def test_interference_and_far_gap(self):
        a, b = Part('a',box()), Part('b',box())
        self.assertEqual(analyze_pair(a,pose(),b,pose((0,0,.09))).pair_diagnostics[0]['overlap']['status'], 'penetrating')
        result = analyze_pair(a,pose(),b,pose((0,0,.2)))
        self.assertEqual(len(result.patches), 0)
        self.assertEqual(result.pair_diagnostics[0]['overlap']['status'], 'separated')

    def test_transform_and_pair_symmetry(self):
        a, b = Part('a',box()), Part('b',box())
        ta, tb = pose(), pose((.025,0,.1))
        world = pose((.2,-.3,.4), rotation_xyz([.3,.7,-.4]))
        for aa, at, bb, bt in ((a,world@ta,b,world@tb), (b,tb,a,ta)):
            result = analyze_pair(aa,at,bb,bt)
            self.assertAlmostEqual(active_area(result), .0075, places=9)

    def test_line_and_point_contact(self):
        a, b = Part('a',rectangle()), Part('b',rectangle(upward=False))
        for delta, dim in (((.1,0,0),1), ((.1,.1,0),0)):
            result = analyze_pair(a,pose(),b,pose(delta))
            self.assertEqual(active_area(result), 0)
            self.assertTrue(any(p.dimension == dim and p.classification == 'active' for p in result.patches))
            if dim == 1:
                self.assertAlmostEqual(sum(p.length_m for p in result.patches if p.dimension == 1), .1)

    def test_tilted_contact_is_not_a_full_face(self):
        a, b = Part('a', rectangle()), Part('b', rectangle(upward=False))
        tf = pose(rotation=rotation_xyz([0,.02,0]))
        result = analyze_pair(a,pose(),b,tf)
        self.assertEqual(active_area(result), 0)
        self.assertTrue(any(p.dimension == 1 for p in result.patches))
        self.assertLess(sum(p.area_m2 for p in result.patches if p.classification == 'near'), .005)
