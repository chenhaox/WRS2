"""Analytic sign, tangent-cone, reference and finite-gap regressions for M2.04."""
import importlib.util
import unittest
from dataclasses import replace
import numpy as np
from wrs.assembly import (Assembly, Part, MatingRelation, ContactConfig, ContactPatch, ContactAnalysis,
                          ConstraintConfig, Region, SDFContactBackend, SDFConfig,
                          analyze_contacts, build_contact_graph, contact_constraints,
                          candidate_motions, rebase_twist)
from wrs.assembly.contact.graph import state_geometry_binding
from wrs.assembly.model import digest
from wrs.assembly.geometry.primitives import box, cylinder, pose


def planar_scene():
    assembly = Assembly((Part('a', box(), fixed=True), Part('b', box(), pose((0, 0, .1)))))
    state = assembly.initial_state()
    report = analyze_contacts(assembly, state)
    return assembly, state, report


def graph_with_normals(normals):
    assembly = Assembly((Part('a', box(), fixed=True), Part('b', box())))
    state = assembly.initial_state()
    patches = tuple(ContactPatch('a', 'b', str(i), str(i), 0, 'active', 'analytic',
                                [[0, 0, 0]], [[0, 0, 0]], [n], [-n],
                                (Region((np.zeros((1, 3)),)),), (0, 0)) for i, n in enumerate(normals))
    result = ContactAnalysis(patches, ({'part_a': 'a', 'part_b': 'b', 'overlap': {'status': 'touching'}},),
                             digest(normals), input_binding=state_geometry_binding(assembly, state))
    return build_contact_graph(assembly, state, result)


class ConstraintTests(unittest.TestCase):
    def test_single_face_sign_tangent_swap_and_nonzero_candidates(self):
        assembly, state, report = planar_scene()
        graph = build_contact_graph(assembly, state, report)
        rows = contact_constraints(graph, ['b'])
        self.assertTrue(np.all(rows.residuals([[0, 0, 1, 0, 0, 0]]) > 0))
        self.assertTrue(np.all(rows.residuals([[0, 0, -1, 0, 0, 0]]) < 0))
        np.testing.assert_allclose(rows.residuals([[1, 0, 0, 0, 0, 0]]), 0)
        patches = tuple(replace(p, part_a=p.part_b, part_b=p.part_a,
                                points_a_world_m=p.points_b_world_m, points_b_world_m=p.points_a_world_m,
                                normals_a_world=p.normals_b_world, normals_b_world=p.normals_a_world) for p in report.patches)
        swapped = contact_constraints(build_contact_graph(assembly, state, replace(report, patches=patches)), ['b'])
        np.testing.assert_allclose(rows.matrix_world, swapped.matrix_world)
        candidates = candidate_motions(graph, ['b'])
        self.assertEqual(candidates.status, 'local_candidates')
        self.assertFalse(candidates.geometry_validated)
        self.assertTrue(any(c['contact_mode'] == 'tangent' for c in candidates.candidates))
        for c in candidates.candidates:
            self.assertGreater(np.linalg.norm(c['twist_world']), 0)
            self.assertGreaterEqual(c['minimum_residual'], -1e-9)
        with self.assertRaisesRegex(ValueError, 'fixed'):
            contact_constraints(graph, ['a'])

    def test_opposing_normals_tangent_and_six_sided_local_lock(self):
        graph = graph_with_normals(np.array([[0., 0, 1], [0, 0, -1]]))
        found = candidate_motions(graph, ['b'])
        self.assertGreater(len(found.candidates), 0)
        self.assertTrue(all(abs(c['twist_world'][2]) <= 1e-9 for c in found.candidates))
        locked = candidate_motions(graph_with_normals(np.vstack((np.eye(3), -np.eye(3)))), ['b'])
        self.assertEqual(locked.status, 'locally_blocked')
        self.assertEqual(len(locked.candidates), 0)

    def test_region_vertices_reject_rotation_and_reference_invariance(self):
        assembly, state, report = planar_scene()
        graph = build_contact_graph(assembly, state, report)
        cfg = ConstraintConfig(mode='twist')
        old = np.array([0., 0, .1])
        new = np.array([.2, -.13, .06])
        rows = contact_constraints(graph, ['b'], config=cfg, reference_world_m=old)
        # Centroid-only constraints would wrongly admit tipping into the table.
        rotation = np.array([0., 0, 0, 1, 0, 0])
        self.assertLess(np.min(rows.matrix_world @ rotation), -.049)
        self.assertGreater(np.max(rows.matrix_world @ rotation), .049)
        other = contact_constraints(graph, ['b'], config=cfg, reference_world_m=new)
        rng = np.random.default_rng(32)
        for twist in rng.normal(size=(10, 6)):
            np.testing.assert_allclose(rows.matrix_world @ twist,
                                       other.matrix_world @ rebase_twist(twist, old, new), atol=1e-14)
        found = candidate_motions(graph, ['b'], config=cfg)
        self.assertTrue(any(np.linalg.norm(c['twist_world'][3:]) > 0 for c in found.candidates))
        for c in found.candidates:
            self.assertAlmostEqual(np.linalg.norm(c['normalized_direction']), 1)
            self.assertGreaterEqual(np.min(found.constraints.residuals([c['twist_world']])), -cfg.residual_tol)

    def test_normal_field_not_averaged_and_uniform_cells_extrema(self):
        assembly, state, report = planar_scene()
        p = report.patches[0]
        n = np.array([[1., 0, 0], [-1, 0, 0]])
        field = replace(p, points_a_world_m=np.zeros((2, 3)), points_b_world_m=np.zeros((2, 3)),
                        normals_a_world=n, normals_b_world=-n, regions=())
        rows = contact_constraints(build_contact_graph(assembly, state, replace(report, patches=(field,))), ['b'])
        np.testing.assert_array_equal(rows.matrix_world[:, :3], n)
        self.assertTrue(any('sampled_contact_coverage' in issue for issue in rows.issues))

    def test_finite_gap_activation_and_tolerance_policy_not_active(self):
        assembly, state, report = planar_scene()
        part = replace(assembly.parts[1], assembled_tf=pose((0, 0, .1002)))
        assembly = replace(assembly, parts=(assembly.parts[0], part))
        state = assembly.initial_state()
        report = analyze_contacts(assembly, state)
        from wrs.assembly import ToleranceContactPolicy
        annotated = ToleranceContactPolicy(.0005).apply(report)
        graph = build_contact_graph(assembly, state, annotated)
        found = candidate_motions(graph, ['b'])
        self.assertEqual(len(found.constraints.matrix_world), 0)
        down = next(c for c in found.candidates if np.allclose(c['twist_world'], [0, 0, -1, 0, 0, 0]))
        self.assertAlmostEqual(down['near_activation_parameter_m'], .0002)
        idealized = analyze_contacts(assembly, state, config=ContactConfig(contact_tol_m=.0003, idealize_contact=True))
        graph = build_contact_graph(assembly, state, idealized)
        self.assertEqual(len(contact_constraints(graph, ['b']).matrix_world), 0)
        self.assertGreater(len(contact_constraints(graph, ['b'], config=ConstraintConfig(allow_idealized_contact=True)).matrix_world), 0)

    @unittest.skipUnless(importlib.util.find_spec('open3d'), 'optional open3d is absent')
    def test_sdf_shaft_near_and_mate_are_not_current_constraints(self):
        assembly = Assembly((Part('shaft', cylinder(.0098, .01, 16)),
                             Part('tube', cylinder(.015, .02, 16, inner_radius=.01), fixed=True)),
                            mating_relations=(MatingRelation('shaft', 'tube', 'coaxial'),))
        state = assembly.initial_state()
        backend = SDFContactBackend(sdf_config=SDFConfig(max_query_points=20000, validate_mesh_overlap=False))
        report = analyze_contacts(assembly, state, backend=backend)
        graph = build_contact_graph(assembly, state, report)
        rows = contact_constraints(graph, ['shaft'])
        self.assertEqual(len(rows.matrix_world), 0)
        self.assertGreater(len(rows.near_matrix_world), 0)
        self.assertEqual(len(graph.edges[0].mating_relations), 1)
        self.assertTrue(rows.issues)

    def test_determinism_group_cut_and_config_validation(self):
        assembly, state, report = planar_scene()
        assembly = replace(assembly, parts=tuple(replace(p, fixed=False) for p in assembly.parts))
        graph = build_contact_graph(assembly, state, report)
        group = candidate_motions(graph, ['a', 'b'])
        self.assertEqual(len(group.constraints.matrix_world), 0)
        first, second = candidate_motions(graph, ['b']), candidate_motions(graph, ['b'])
        self.assertEqual(first.constraints.state_digest, second.constraints.state_digest)
        np.testing.assert_array_equal([c['twist_world'] for c in first.candidates], [c['twist_world'] for c in second.candidates])
        for kwargs in ({'characteristic_length_m': 0}, {'residual_tol': np.nan}, {'mode': 'bad'}, {'seed': -1}):
            with self.assertRaises(ValueError):
                ConstraintConfig(**kwargs)
