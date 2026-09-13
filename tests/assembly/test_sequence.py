from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'examples'/'assembly'))
from _shared.stability_cases import make_case,case_supports
from wrs.assembly.model import AssemblyState,digest
from wrs.assembly.planning.sequence import (plan_sequence,replay_sequence,SequenceConfig,HandlingCapability,AuxiliarySupport)


class SequenceTests(unittest.TestCase):
    def test_dfs_resumes_untried_siblings_after_a_dead_end(self):
        from types import SimpleNamespace
        from unittest.mock import patch
        from wrs.assembly.planning.sequence import SequenceEvaluator
        a,s,g=make_case('stack'); cfg=SequenceConfig(); oracle=SequenceEvaluator(a,cfg)
        trace=[]
        def evaluate(state,active,pid):
            trace.append((tuple(sorted(state.poses)),pid))
            if 'upper' not in state.poses and pid=='lower': return None,{'reason':'test_dead_end'}
            after=AssemblyState({k:v for k,v in state.poses.items() if k!=pid})
            return SimpleNamespace(after=after,supports_after=(),cost=1.,part_id=pid),None
        with patch.object(oracle,'evaluate',side_effect=evaluate),patch('wrs.assembly.planning.sequence.replay_sequence',return_value={'status':'valid'}):
            p=plan_sequence(a,s,config=cfg,evaluator=oracle)
        self.assertEqual(p.status,'success')
        self.assertEqual([st.part_id for st in p.removal_steps],['lower','upper'])
        self.assertEqual([pid for _,pid in trace],['upper','lower','lower','upper'])

    def test_complex_gantry_and_distance_change_are_revalidated(self):
        from _shared.sequence_cases import make_case as complex_case
        from wrs.assembly import MotionConfig
        a,s,g=complex_case('gantry')
        p=plan_sequence(a,s,config=SequenceConfig(motion=MotionConfig(outside_margin_m=.10)))
        self.assertEqual(p.status,'success'); self.assertEqual(len(p.assembly_steps),9)
        self.assertEqual(replay_sequence(a,p)['status'],'valid')
        a,s,g=make_case('stack')
        short=plan_sequence(a,s,config=SequenceConfig(motion=MotionConfig(outside_margin_m=.02)))
        long=plan_sequence(a,s,config=SequenceConfig(motion=MotionConfig(outside_margin_m=.15)))
        self.assertNotEqual(short.input_digest,long.input_digest)
        for left,right in zip(short.removal_steps,long.removal_steps):
            length=lambda step:sum(np.linalg.norm(b[:3,3]-a[:3,3]) for a,b in zip(step.removal.poses,step.removal.poses[1:]))
            self.assertAlmostEqual(length(right)-length(left),.13,places=7)
        self.assertEqual(replay_sequence(a,long)['status'],'valid')

    def test_stack_forward_replay_and_immutable_state(self):
        a,s,g=make_case('stack'); before=digest((a,s))
        plan=plan_sequence(a,s)
        self.assertEqual(plan.status,'success')
        self.assertEqual([x.part_id for x in plan.assembly_steps],['lower','upper'])
        self.assertEqual(replay_sequence(a,plan)['status'],'valid')
        self.assertTrue(plan.geometry_validated and plan.equilibrium_validated)
        self.assertFalse(plan.execution_validated)
        self.assertEqual(digest((a,s)),before)
        self.assertEqual(plan.assembly_steps[0].assembly_events[0]['kind'],'take_from_staging')

    def test_beam_method_repeatability_and_capacity(self):
        a,s,g=make_case('stack'); cfg=SequenceConfig(method='beam')
        p=plan_sequence(a,s,config=cfg); q=plan_sequence(a,s,config=cfg)
        self.assertEqual(p.status,'success'); self.assertEqual(p.input_digest,q.input_digest)
        self.assertEqual([x.part_id for x in p.removal_steps],[x.part_id for x in q.removal_steps])
        weak=plan_sequence(a,s,config=replace(cfg,handling=HandlingCapability(max_force_n=1)))
        self.assertNotEqual(weak.status,'success')

    def test_floating_requires_initial_support_and_finite_resources(self):
        a,s,g=make_case('floating')
        support=tuple(AuxiliarySupport(f'aux_{i}',v) for i,v in enumerate(case_supports('floating')))
        self.assertNotEqual(plan_sequence(a,s,supports=support).status,'success')
        plan=plan_sequence(a,s,supports=support,initial_support_ids=('left','right'))
        self.assertEqual(plan.status,'success')
        self.assertEqual(replay_sequence(a,plan)['status'],'valid')
        with self.assertRaises(ValueError):
            plan_sequence(a,s,supports=support,initial_support_ids=('left','right'),config=SequenceConfig(max_auxiliary_resources=1))
        kinds=[e['kind'] for st in plan.removal_steps for e in st.events]
        self.assertIn('release_auxiliary',kinds)

    def test_same_resource_cannot_be_used_twice(self):
        a,s,g=make_case('floating')
        support=tuple(AuxiliarySupport('same',v) for v in case_supports('floating'))
        with self.assertRaises(ValueError): plan_sequence(a,s,supports=support,initial_support_ids=('left','right'))

    def test_budget_stale_plan_and_pose_identity(self):
        a,s,g=make_case('stack')
        exhausted=plan_sequence(a,s,config=SequenceConfig(max_expansions=1))
        self.assertEqual(exhausted.status,'exhausted')
        p=plan_sequence(a,s)
        changed=replace(a,gravity_world_m_s2=(0,0,-8))
        with self.assertRaises(ValueError): replay_sequence(changed,p)
        pose=s.poses['upper'].copy(); pose[0,3]+=.01
        moved=AssemblyState(dict(s.poses,upper=pose))
        q=plan_sequence(a,moved)
        self.assertNotEqual(p.input_digest,q.input_digest)

    def test_incomplete_sequence_and_changed_events_rejected(self):
        a,s,g=make_case('stack'); plan=plan_sequence(a,s)
        empty=replace(plan,removal_steps=())
        self.assertNotEqual(replay_sequence(a,empty)['status'],'valid')
        step=replace(plan.removal_steps[0],events=plan.removal_steps[0].events[:-1])
        wrong=replace(plan,removal_steps=(step,)+plan.removal_steps[1:])
        self.assertEqual(replay_sequence(a,wrong)['reason'],'invalid_handoff_events')


if __name__=='__main__': unittest.main()
