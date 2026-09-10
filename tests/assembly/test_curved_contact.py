import unittest
import numpy as np
from wrs.assembly import Part, MeshData, GeometryConfig, ContactConfig, analyze_pair
from wrs.assembly.primitives import sphere, rectangle, cylinder, pose, box
from wrs.assembly.geometry.proximity import MeshProximity
from wrs.assembly.geometry.mesh_bvh import QueryBudget
from wrs.assembly.contact.mesh import analyze_mesh_pair


class CurvedTests(unittest.TestCase):
    def test_full_shaft_band_and_explicit_incomplete_coverage(self):
        tube = Part('A',cylinder(radius=.015,inner_radius=.01,height=.02,sections=32))
        shaft = Part('B',cylinder(radius=.0098,height=.01,sections=32))
        cfg = ContactConfig(surface_resolution_m=.003,normal_angle_rad=.55,
                            max_cells=4000,max_triangle_tests=100000)
        result = analyze_pair(tube,pose(),shaft,pose(),config=cfg)
        self.assertEqual(result.pair_diagnostics[0]['overlap']['status'],'separated')
        self.assertFalse(any(p.classification=='active' for p in result.patches))
        band = [p for p in result.patches if p.sampling_side=='b']
        expected = 2*32*.0098*np.sin(np.pi/32)*.01
        self.assertAlmostEqual(sum(p.area_m2 for p in band),expected,places=11)
        self.assertEqual(sum(len(p.regions) for p in band),1)
        self.assertTrue(all(p.quality=='bounded' and all(p.provenance['cell_band_bounded']) for p in band))
        reports = result.pair_diagnostics[0]['curved_coverage']
        self.assertTrue(all(c['unprocessed_area_m2']==0 for c in reports))
        self.assertEqual(sum(c['unresolved_area_m2'] for c in reports if c['sampling_side']=='b'),0)
        # A tiny budget must remain distinguishable from a complete band result.
        from dataclasses import replace
        partial = analyze_pair(tube,pose(),shaft,pose(),config=replace(cfg,max_triangle_tests=10))
        self.assertGreater(sum(c['unprocessed_area_m2'] for c in partial.pair_diagnostics[0]['curved_coverage']),0)

    def test_sphere_band_threshold_and_budget(self):
        ball = Part('sphere',sphere(sections=16,rings=8))
        support = Part('support',rectangle())
        backend = MeshProximity(geometry_config=GeometryConfig(smooth_angle_rad=.8))
        prep_a, sa = backend.prepare(ball.geometry)
        prep_b, sb = backend.prepare(support.geometry)
        areas = []
        for eps in (.0005,.001):
            cfg = ContactConfig(near_tol_m=eps,surface_resolution_m=.003,
                                normal_angle_rad=.6,max_cells=3000,max_triangle_tests=50000)
            patches, coverage = analyze_mesh_pair(ball,pose((0,0,.025)),prep_a,sa[0],
                                                  support,pose(),prep_b,sb[0],
                                                  config=cfg,backend=backend,budget=QueryBudget(50000))
            self.assertFalse(any(p.classification == 'active' for p in patches))
            self.assertTrue(all(p.measure_kind == 'near_band' for p in patches))
            areas.append(sum(p.area_m2 for p in patches if p.sampling_side == 'a'))
            self.assertGreater(sum(c['unresolved_area_m2'] for c in coverage), 0)
        self.assertGreater(areas[0],0)
        self.assertGreater(areas[1],areas[0])
        result = analyze_pair(ball,pose((0,0,.025)),support,pose(),backend=backend,
                              config=ContactConfig(max_cells=1,max_triangle_tests=100,normal_angle_rad=.6))
        diag = result.pair_diagnostics[0]
        self.assertGreater(diag['unresolved_area_m2'],0)
        self.assertTrue(any('budget_exhausted' in c['reason'] for c in diag['curved_coverage']))
        self.assertFalse(any(p.classification == 'active' and p.dimension == 2 for p in result.patches))

    def test_axial_hole_distance_and_retriangulation(self):
        backend = MeshProximity()
        # A shaft in a tube is contained by the tube's AABB, but outside its solid.
        errors = []
        for n in (12,24):
            shaft = Part('shaft',cylinder(radius=.0098,height=.01,sections=n))
            tube = Part('tube',cylinder(radius=.015,inner_radius=.01,height=.02,sections=n))
            distance = backend.pair_distance(shaft,pose(),tube,pose(),budget=20000)
            self.assertEqual(distance.status,'complete')
            self.assertAlmostEqual(distance.upper_bound_m,.0002*np.cos(np.pi/n),places=10)
            errors.append(abs(distance.upper_bound_m-.0002))
            overlap = backend.classify_overlap(shaft,pose(),tube,pose(),budget=20000)
            self.assertEqual(overlap.status,'separated')
            # Splitting every facet preserves the underlying piecewise planar surface.
            mesh = shaft.geometry
            vs = mesh.vertices.tolist()
            fs = []
            for tri in mesh.faces:
                c = len(vs)
                vs.append(mesh.vertices[tri].mean(0).tolist())
                fs.extend([[tri[i],tri[(i+1)%3],c] for i in range(3)])
            refined = Part('refined',MeshData(vs,fs))
            again = backend.pair_distance(refined,pose(),tube,pose(),budget=60000)
            self.assertEqual(again.status,'complete')
            self.assertAlmostEqual(again.upper_bound_m,distance.upper_bound_m,places=10)
        self.assertLess(errors[1],errors[0])

    def test_planar_budget_and_geometry_uncertainty(self):
        a, b = Part('a',box()), Part('b',box())
        result = analyze_pair(a,pose(),b,pose((0,0,.1)),config=ContactConfig(max_triangle_tests=1))
        self.assertGreater(result.pair_diagnostics[0]['unresolved_area_m2'],0)
        self.assertTrue(any(c['status']=='budget_exhausted' for c in result.pair_diagnostics[0]['planar_coverage']))
        mesh = box()
        uncertain = Part('uncertain',MeshData(mesh.vertices,mesh.faces,geometry_error_m=1e-5))
        result = analyze_pair(a,pose(),uncertain,pose((0,0,.1000005)))
        self.assertFalse(any(p.classification=='active' for p in result.patches))
        self.assertTrue(any(p.classification=='unknown' for p in result.patches))
