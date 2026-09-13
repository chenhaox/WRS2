"""Analytic optima, exact degeneracy certificates and sampling miss semantics."""
from dataclasses import replace
from fractions import Fraction
import importlib.util
import json
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from wrs.assembly import (Assembly, Part, ContactConfig, DirectionConfig,
                          fibonacci_directions, solve_directions, assembly_directions,
                          analyze_contacts, build_contact_graph)
from wrs.assembly.geometry.primitives import box, pose
from wrs.assembly.model import to_dict


class DirectionTests(unittest.TestCase):
    def test_optional_solver_dependency_is_isolated(self):
        with patch.dict(sys.modules, {'clarabel':None}):
            result = solve_directions([[0,0,1]],config=DirectionConfig(method='fibonacci'))
            self.assertEqual(result.status,'feasible')
            with self.assertRaisesRegex(ImportError,'assembly-planning'):
                solve_directions([[0,0,1]])

    @unittest.skipUnless(importlib.util.find_spec('clarabel'), 'optional clarabel absent')
    def test_solver_failure_is_unknown_and_serializable(self):
        failed = SimpleNamespace(status='NumericalError',iterations=3,r_prim=np.nan,r_dual=np.inf)
        with patch('clarabel.DefaultSolver') as solver:
            solver.return_value.solve.return_value = failed
            result = solve_directions([[0,0,1]])
        self.assertEqual(result.status,'unknown')
        self.assertIsNone(result.best_direction)
        json.dumps(to_dict(result),allow_nan=False)

    def methods(self):
        return ('fibonacci', 'socp') if importlib.util.find_spec('clarabel') else ('fibonacci',)

    def check_result(self, normals, result):
        if not len(result.directions):
            return
        np.testing.assert_allclose(np.linalg.norm(result.directions, axis=1), 1, atol=1e-12)
        n = np.asarray(normals)
        n = n/np.linalg.norm(n, axis=1)[:, None]
        self.assertGreaterEqual(np.min(n @ result.directions.T), -1e-9)
        self.assertFalse(result.geometry_validated)
        with self.assertRaises(ValueError):
            result.directions.setflags(write=True)

    def test_bank_validation_determinism_and_immutability(self):
        first = fibonacci_directions(6500)
        self.assertIs(first, fibonacci_directions(6500))
        np.testing.assert_allclose(np.linalg.norm(first, axis=1), 1, atol=1e-14)
        with self.assertRaises(ValueError):
            first.setflags(write=True)
        for kwargs in ({'method':'bad'}, {'sample_count':0}, {'sample_count':True},
                       {'residual_tol':np.nan}, {'preferred_direction':(0,0,0)}, {'max_dimension_lps':0}):
            with self.assertRaises(ValueError):
                DirectionConfig(**kwargs)
        for n in ([[0,0,0]], [[np.nan,0,1]], np.zeros((2,6))):
            with self.assertRaises(ValueError):
                solve_directions(n)

    @unittest.skipUnless(importlib.util.find_spec('clarabel'), 'optional clarabel absent')
    def test_continuous_optima_and_rotation_covariance(self):
        for n, target, angle in (([[0,0,1]], [0,0,1], np.pi/2),
                                 (np.eye(3), np.ones(3)/np.sqrt(3), np.arcsin(1/np.sqrt(3)))):
            result = solve_directions(n)
            self.assertEqual(result.status, 'feasible')
            self.assertEqual(result.dimension, 3)
            self.assertEqual(result.optimality, 'continuous_margin')
            self.assertEqual(result.diagnostics['dimension_lps'], 0)
            np.testing.assert_allclose(result.best_direction, target, atol=1e-7)
            self.assertAlmostEqual(result.angular_margin_rad, angle, places=7)
            self.check_result(n, result)
        rot = Rotation.from_rotvec([.2,.5,-.3]).as_matrix()
        result = solve_directions(np.eye(3) @ rot.T)
        np.testing.assert_allclose(result.best_direction, rot @ (np.ones(3)/np.sqrt(3)), atol=1e-7)

    def test_plane_circle_and_fixed_preference(self):
        n = [[1.,0,0],[-1,0,0]]
        for method in self.methods():
            cfg = DirectionConfig(method=method)
            result = solve_directions(n, config=cfg)
            self.assertEqual(result.status, 'feasible')
            self.assertEqual(result.dimension, 2)
            self.assertAlmostEqual(result.angular_margin_rad, 0)
            self.assertIsNone(result.intrinsic_angular_margin_rad)
            np.testing.assert_allclose(result.best_direction, [0,0,1], atol=1e-12)
            if method == 'fibonacci':
                self.assertEqual(result.diagnostics['sample_domain'], 'circle')
                self.assertEqual(len(result.directions), 6500)
            np.testing.assert_array_equal(result.directions, solve_directions(n[::-1], config=cfg).directions)
            self.check_result(n, result)

    def test_shaft_blind_shaft_and_tilt(self):
        base = np.array([[1.,0,0],[-1,0,0],[0,1,0],[0,-1,0]])
        rot = Rotation.from_rotvec([.7,.2,.5]).as_matrix()
        for method in self.methods():
            for R in (np.eye(3), rot):
                for blind in (False, True):
                    n = np.vstack((base, [0,0,1])) if blind else base
                    n = n @ R.T
                    cfg = DirectionConfig(method=method, preferred_direction=tuple(R @ [0,0,-1]))
                    result = solve_directions(n, config=cfg)
                    self.assertEqual(result.status, 'feasible')
                    self.assertEqual(result.dimension, 1)
                    np.testing.assert_allclose(result.best_direction, R @ [0,0,1 if blind else -1], atol=1e-9)
                    self.assertLess(result.angular_margin_rad, 1e-8)
                    self.assertIsNone(result.intrinsic_angular_margin_rad)
                    self.assertEqual(len(result.directions), 1 if method == 'socp' or blind else 2)
                    self.check_result(n, result)

    def test_three_radial_normals_without_opposite_pairs_have_exact_certificates(self):
        angles = np.arange(3)*2*np.pi/3
        n = np.column_stack((np.cos(angles), np.sin(angles), np.zeros(3)))
        normalized = np.unique(n/np.linalg.norm(n, axis=1)[:, None], axis=0)
        for method in self.methods():
            result = solve_directions(n, config=DirectionConfig(method=method))
            self.assertEqual(result.dimension, 1)
            np.testing.assert_allclose(result.best_direction, [0,0,1], atol=1e-12)
            for cert in result.diagnostics['equality_certificates']:
                total = [Fraction(float(x)) for x in normalized[cert['row']]]
                for row, weight in zip(cert['support'], cert['weights']):
                    w = Fraction(weight)
                    self.assertGreaterEqual(w, 0)
                    total = [a+w*Fraction(float(b)) for a,b in zip(total, normalized[row])]
                self.assertEqual(total, [0,0,0])

    def test_many_rotated_cylinder_normals_and_explicit_roundoff_model(self):
        a = 2*np.pi*np.arange(32)/32
        R = Rotation.from_rotvec([.37,.23,.61]).as_matrix()
        n = np.column_stack((np.cos(a),np.sin(a),np.zeros(32))) @ R.T
        for method in self.methods():
            result = solve_directions(n,config=DirectionConfig(method=method))
            self.assertEqual((result.status,result.dimension),('feasible',1))
            self.assertTrue(result.diagnostics['roundoff_reduction'])
            np.testing.assert_allclose(result.best_direction,R[:,2],atol=1e-12)
            self.check_result(n,result)
            strict = solve_directions(n,config=DirectionConfig(method=method,allow_roundoff_reduction=False))
            self.assertEqual(strict.status,'unknown')

    def test_blocked_budget_and_numerically_narrow_are_distinct(self):
        cage = np.vstack((np.eye(3), -np.eye(3)))
        for method in self.methods():
            result = solve_directions(cage, config=DirectionConfig(method=method))
            self.assertEqual((result.status, result.dimension), ('blocked', 0))
            self.assertIsNone(result.best_direction)
            tetrahedron = [[1.,1,1],[1,-1,-1],[-1,1,-1],[-1,-1,1]]
            budget = solve_directions(tetrahedron, config=DirectionConfig(method=method, max_dimension_lps=1))
            self.assertEqual(budget.status, 'unknown')
            thin = solve_directions([[1.,0,0],[-1,0,1e-10]], config=DirectionConfig(method=method))
            self.assertEqual(thin.status, 'unknown')
            self.assertIsNone(thin.dimension)

    def test_weak_nonzero_projected_row_is_not_discarded(self):
        n = [[1.,0,0],[-1,0,0],[0,1,0],[0,-1,0],[1,0,1e-12]]
        for method in self.methods():
            result = solve_directions(n, config=DirectionConfig(method=method, preferred_direction=(0,0,-1)))
            self.assertEqual(result.dimension, 1)
            np.testing.assert_allclose(result.best_direction, [0,0,1], atol=1e-12)

    def test_sampling_miss_does_not_prove_blockage(self):
        e = np.tan(np.deg2rad(.1))
        n = [[1,0,e],[-1,0,e],[0,1,e],[0,-1,e]]
        result = solve_directions(n, config=DirectionConfig(method='fibonacci'))
        self.assertEqual((result.status, result.dimension), ('no_sample_hit', 3))
        if importlib.util.find_spec('clarabel'):
            result = solve_directions(n)
            self.assertEqual(result.status, 'feasible')
            self.assertAlmostEqual(result.angular_margin_rad, np.deg2rad(.1), places=8)

    def test_intrinsic_margin_differs_from_ambient_margin(self):
        n = [[1.,0,0],[-1,0,0],[0,1,0],[0,0,1]]
        for method in self.methods():
            result = solve_directions(n, config=DirectionConfig(method=method, sample_count=720))
            self.assertEqual(result.dimension, 2)
            self.assertAlmostEqual(result.angular_margin_rad, 0)
            self.assertAlmostEqual(result.intrinsic_angular_margin_rad, np.pi/4, places=7)
            np.testing.assert_allclose(result.best_direction, [0,1/np.sqrt(2),1/np.sqrt(2)], atol=1e-7)

    def test_contact_graph_adapter_and_unknown_evidence(self):
        assembly = Assembly((Part('floor', box(), fixed=True), Part('part', box(), pose((0,0,.1)))))
        state = assembly.initial_state()
        analysis = analyze_contacts(assembly, state)
        graph = build_contact_graph(assembly, state, analysis)
        for method in self.methods():
            cfg = DirectionConfig(method=method)
            result = assembly_directions(graph, ['part'], config=cfg)
            self.assertEqual(result.status, 'feasible')
            self.assertIsNotNone(result.constraints)
            unresolved = replace(graph, edges=tuple(replace(e, issues=('overlap_unknown',)) for e in graph.edges))
            result = assembly_directions(unresolved, ['part'], config=cfg)
            self.assertEqual(result.status, 'unknown')
            self.assertEqual(result.diagnostics['constraint_status'], 'feasible')
            self.assertIsNotNone(result.best_direction)
        with self.assertRaises(ValueError):
            assembly_directions(graph, ['floor'])

    def test_no_constraints_and_no_fake_near_constraints(self):
        for method in self.methods():
            result = solve_directions(np.empty((0,3)), config=DirectionConfig(method=method))
            self.assertEqual((result.status, result.dimension), ('unconstrained', 3))
            self.assertIsNone(result.angular_margin_rad)
        assembly = Assembly((Part('floor', box(), fixed=True), Part('part', box(), pose((0,0,.1002)))))
        state = assembly.initial_state()
        graph = build_contact_graph(assembly, state, analyze_contacts(assembly, state))
        result = assembly_directions(graph, ['part'], config=DirectionConfig(method='fibonacci', sample_count=32))
        self.assertEqual(result.status, 'unconstrained')
        self.assertEqual(len(result.constraints.matrix_world), 0)
        self.assertGreater(len(result.constraints.near_matrix_world), 0)


if __name__ == '__main__':
    unittest.main()
