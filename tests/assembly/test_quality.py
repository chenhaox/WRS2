"""Paper taxonomy, physical grasp counts and branch-and-bound correctness."""
from dataclasses import replace
from itertools import permutations
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'examples'/'assembly'))
from wrs.assembly import (solve_directions, DirectionConfig, score_assemblability, StepQuality,
    sequence_quality, quality_upper_bound, QualityTransition, quality_depth_first, QualitySearchConfig,
    check_force_closure, GraspabilityAnalyzer, GraspabilityConfig, AssemblyState, StabilitySweepConfig,
    plan_quality_sequence, replay_sequence)
from wrs.assembly.model import digest


CASES = {
    'a': ([[0,0,1]],10),
    'b': ([[1,0,0],[-1,0,0]],9),
    'c': ([[1,0,0],[0,1,0]],10),
    'd': ([[1,0,0],[-1,0,0],[0,1,0]],3),
    'e': ([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0]],2),
    'f': ([[1,0,0],[0,1,0],[0,0,1]],10),
    'g': ([[1,0,0],[-1,0,0],[0,1,0],[0,0,1]],3),
    'h': ([[1,0,0],[-1,0,0],[0,1,0],[0,-1,0],[0,0,1]],1),
    'i': (np.vstack((np.eye(3),-np.eye(3))),0),
}


class QualityTests(unittest.TestCase):
    def test_nine_cases_rotations_duplicates_and_display_independence(self):
        R = Rotation.from_euler('xyz', [.32,.65,.87]).as_matrix()
        for case,(normals,expected) in CASES.items():
            for rotated in (False, True):
                n = np.array(normals)@(R.T if rotated else np.eye(3))
                n = np.concatenate((n, n*7))
                for method in ('socp','fibonacci'):
                    result = solve_directions(n,config=DirectionConfig(method=method,sample_count=101))
                    scored = score_assemblability(result)
                    self.assertEqual((scored.case,scored.score),(case,expected),(case,method,dict(result.diagnostics)))
        d = solve_directions([[0,0,1]])
        self.assertEqual(score_assemblability(d,profile='wan2016').score,'infinity')
        digest(score_assemblability(d,profile='wan2016'))  # strict JSON
        self.assertIsNone(score_assemblability(replace(d,status='unknown')).score)

    def test_correct_support_switch_bound_beats_naive_prefix_pruning(self):
        cfg = QualitySearchConfig()
        def expand(order,pid):
            order = order or ()
            s = (.0002 if pid=='a' else .0001) if not order else (1. if order[0]=='a' else 0.)
            yield QualityTransition(order+(pid,),StepQuality(s,1,10))
        bounded = quality_depth_first(('a','b'),expand,config=cfg)
        exhaustive = quality_depth_first(('a','b'),expand,config=replace(cfg,prune=False))
        self.assertEqual(bounded['best'][0],('b','a'))
        self.assertAlmostEqual(bounded['score'],.1)
        self.assertEqual(bounded['score'],exhaustive['score'])
        prefix = [StepQuality(.0001,1,10)]
        self.assertLess(sequence_quality(prefix),.002)
        self.assertGreater(quality_upper_bound(prefix,remaining_steps=1),.002)

    def test_search_matches_independent_permutation_objective_and_budget(self):
        def scores(order):
            return tuple(StepQuality(0 if p=='b' and i>0 else .3+i,
                                   1+(ord(p)-97),10 if i<2 else 3) for i,p in enumerate(order))
        def expand(order,pid):
            new = (order or ())+(pid,)
            yield QualityTransition(new,scores(new)[-1])
        expected = max(sequence_quality(scores(p)) for p in permutations('abcd'))
        result = quality_depth_first(tuple('abcd'),expand)
        self.assertAlmostEqual(result['score'],expected)
        self.assertTrue(result['search_complete'])
        short = quality_depth_first(tuple('abcd'),expand,config=QualitySearchConfig(max_expansions=1))
        self.assertFalse(short['search_complete'])
        self.assertIsNone(short['best'])

    def test_force_closure_needs_six_dimensions_and_friction(self):
        p = np.array([[-.02,0,0],[.02,0,0]])
        n = np.array([[1,0,0],[-1,0,0]])
        self.assertEqual(check_force_closure(p,n)['rank'],5)
        points = np.array([[x,y,z] for x in (-.02,.02) for y in (-.01,.01) for z in (-.01,.01)])
        normals = np.array([[-np.sign(x),0,0] for x,y,z in points])
        self.assertEqual(check_force_closure(points,normals)['status'],'force_closed')
        self.assertEqual(check_force_closure(points,normals,friction=0)['status'],'not_force_closed')

    def test_wrs_catalogue_state_collision_and_caller_preservation(self):
        from robot_execution_demo import make_demo
        a, cell = make_demo(); hand = cell.bindings['main_arm'].arm.end_effector
        before = (hand.tf.copy(),hand.qs.copy())
        analyzer = GraspabilityAnalyzer(a,hand)
        s = AssemblyState({p.part_id:p.assembled_tf for p in a.parts if p.part_id!='upper'})
        r = analyzer.analyze(s,'lower')
        self.assertEqual(r.count,4)
        self.assertIs(analyzer.analyze(s,'lower'),r)
        self.assertEqual(len(analyzer.accepted_grasps(r)),4)
        # Move the other block onto the verified gripper root, not the object.
        root = analyzer._catalogue('lower')[0][1][0]
        v = root.geometry.vertices@root.assembled_tf[:3,:3].T+root.assembled_tf[:3,3]
        tf = np.eye(4); tf[:3,3] = (v.min(0)+v.max(0))/2+s.poses['lower'][:3,3]
        blocked = analyzer.analyze(AssemblyState(dict(s.poses,upper=tf)),'lower')
        self.assertNotIn(0,blocked.accepted_ids)
        self.assertNotEqual(r.input_digest,blocked.input_digest)
        np.testing.assert_array_equal(hand.tf,before[0]); np.testing.assert_array_equal(hand.qs,before[1])

    def test_forward_quality_plan_uses_real_grasps_and_replays(self):
        from robot_execution_demo import make_demo
        a, cell = make_demo()
        g = GraspabilityAnalyzer(a,cell.bindings['main_arm'].arm.end_effector)
        cfg = QualitySearchConfig(sweep=StabilitySweepConfig(backend='highs',direction_count=12))
        result = plan_quality_sequence(a,g,config=cfg)
        self.assertEqual(result.status,'success',dict(result.diagnostics))
        self.assertEqual([s.part_id for s in result.plan.assembly_steps],['lower','upper'])
        self.assertEqual(replay_sequence(a,result.plan)['status'],'valid')
        self.assertTrue(result.plan.geometry_validated and result.plan.equilibrium_validated)
        self.assertFalse(result.plan.execution_validated)
        self.assertTrue(all(q.graspability>0 and q.assemblability==10 for q in result.qualities))
        digest(result)
        from wrs.assembly import validate_execution
        robot = validate_execution(result.plan,cell)
        self.assertEqual(robot.status,'success',dict(robot.diagnostics))
        self.assertTrue(robot.execution_validated)

    def test_unstable_intermediate_requires_finite_support_then_releases(self):
        from quality_sequence_demo import make_quality_case
        from wrs import or_2fg7
        a,supports = make_quality_case('counterweight')
        g = GraspabilityAnalyzer(a,or_2fg7.OR2FG7())
        cfg = QualitySearchConfig(sweep=StabilitySweepConfig(backend='highs',direction_count=12))
        result = plan_quality_sequence(a,g,supports=supports,config=cfg)
        self.assertEqual(result.status,'success',dict(result.diagnostics))
        self.assertEqual([q.stability==0 for q in result.qualities],[True,False,False])
        self.assertAlmostEqual(result.score,.2)
        self.assertEqual(result.plan.assembly_steps[0].supports_before,('beam_support',))
        self.assertFalse(result.plan.assembly_steps[-1].supports_before)
        self.assertEqual(replay_sequence(a,result.plan)['status'],'valid')
        weak = (replace(supports[0],candidate=replace(supports[0].candidate,max_normal_force_n=.01)),)
        failed = plan_quality_sequence(a,g,supports=weak,config=cfg)
        self.assertIsNone(failed.plan)

    def test_physical_bridge_pruning_agrees_with_exhaustive_orders(self):
        from quality_sequence_demo import make_quality_case
        from wrs import or_2fg7
        a,supports = make_quality_case('bridge'); g = GraspabilityAnalyzer(a,or_2fg7.OR2FG7())
        cfg = QualitySearchConfig(sweep=StabilitySweepConfig(backend='highs',direction_count=12))
        one = plan_quality_sequence(a,g,config=cfg)
        two = plan_quality_sequence(a,g,config=replace(cfg,prune=False))
        self.assertEqual(one.status,'success')
        self.assertEqual(one.diagnostics['optimality'],'optimal_within_declared_catalogue_and_model')
        self.assertAlmostEqual(one.score,two.score,places=9)
        self.assertGreater(one.diagnostics['pruned'],0)


if __name__ == '__main__': unittest.main()
