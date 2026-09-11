"""Real RS007L/OR2FG7/MuJoCo planning and independent execution replay."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'examples'/'assembly'))
from robot_execution_demo import make_demo
from wrs.assembly import AuxiliarySupport,SupportCandidate,plan_sequence
from wrs.assembly.execution import (validate_execution,replay_execution,ExecutionArm,
                                    generate_execution_grasps,_PolicyCollider,ExecutionConfig)


class ExecutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.assembly,cls.cell=make_demo()
        cls.plan=plan_sequence(cls.assembly)
        cls.result=validate_execution(cls.plan,cls.cell)

    def test_real_forward_replay_and_state_restoration(self):
        r=self.result; self.assertEqual(r.status,'success',dict(r.diagnostics))
        self.assertTrue(r.execution_validated); self.assertGreater(len(r.frames),100)
        self.assertEqual(r.validation_level,'sampled_joint_nominal_mesh')
        self.assertGreater(r.diagnostics['exact_static_target_cache_hits'],0)
        for rid,b in self.cell.bindings.items():
            np.testing.assert_array_equal(b.arm.body.qs,self.cell._initial_q[rid])
            self.assertEqual(b.arm.collision_cache_decimals,3)
        self.assertEqual({e['part_id'] for e in r.events if e['kind']=='release_part'},{'lower','upper'})

    def test_actual_pad_geometry_and_shared_finite_capacity(self):
        grasp=generate_execution_grasps(self.assembly.parts[1],self.cell.bindings['main_arm'].arm.end_effector)[0]
        self.assertLess(grasp.provenance['pad_residual_m'],1e-6)
        self.assertAlmostEqual(grasp.provenance['jaw_width'],.024,places=6)
        event=next(e for e in self.result.events if e['kind']=='acquire_part')
        self.assertGreaterEqual(len(event['contact_points_local_m']),6)
        a,cell=make_demo(); b=cell.bindings['main_arm']
        cell.bindings['main_arm']=ExecutionArm(b.arm,finger_normal_force_n=.01)
        r=validate_execution(self.plan,cell)
        self.assertFalse(r.execution_validated); self.assertNotEqual(r.status,'success')

    def test_workcell_binding_rejects_changed_base(self):
        b=self.cell.bindings['main_arm']; original=b.arm.body.tf.copy()
        try:
            b.arm.body.pos=np.asarray(b.arm.body.pos)+[.01,0,0]
            with self.assertRaises(ValueError): replay_execution(self.result,self.cell,plan=self.plan)
        finally:
            b.arm.body.pos=original[:3,3]; b.arm.body.rotmat=original[:3,:3]; self.cell.reset()

    def test_replay_rejects_missing_motion_and_release(self):
        r=self.result
        skipped=replace(r,frames=(r.frames[0],)+r.frames[10:])
        self.assertNotEqual(replay_execution(skipped,self.cell,plan=self.plan)['status'],'valid')
        missing=replace(r,events=tuple(e for e in r.events if e['kind']!='release_equilibrium_verified'))
        self.assertEqual(replay_execution(missing,self.cell,plan=self.plan)['reason'],'missing_release_events')

    def test_unreachable_staging_fails(self):
        a,cell=make_demo()
        for pid in cell.source_poses:
            tf=cell.source_poses[pid].copy(); tf[0,3]+=4
            cell.source_poses[pid]=tf; cell.initial_poses[pid]=tf
        r=validate_execution(self.plan,cell)
        self.assertNotEqual(r.status,'success')

    def test_true_mesh_still_blocks_wrong_target_face(self):
        cell=self.cell; cell.reset(); collider=_PolicyCollider(cell,ExecutionConfig())
        collider.activate('main_arm'); collider.target='lower'
        tf=cell.initial_poses['lower'].copy(); tf[2,3]-=.02
        cell.objects['lower'].tf=tf
        try: self.assertFalse(collider._target_valid())
        finally: cell.reset()

    def test_two_real_arms_and_finite_auxiliary_takeover(self):
        a,cell=make_demo(True)
        support=AuxiliarySupport('aux_arm',SupportCandidate('lower_support','lower',a.parts[1].assembled_tf[:3,3],(0,0,1),5.))
        plan=plan_sequence(a,supports=(support,),initial_support_ids=('lower_support',))
        r=validate_execution(plan,cell)
        self.assertEqual(r.status,'success',dict(r.diagnostics))
        self.assertIn('acquire_auxiliary',[e['kind'] for e in r.events])
        acquire=next(e for e in r.events if e['kind']=='acquire_auxiliary')
        release=next(e for e in r.events if e['kind']=='release_part')
        self.assertLess(acquire['frame'],release['frame'])
        self.assertEqual(r.frames[-1]['active_supports'],('lower_support',))
        saved=cell.bindings.pop('aux_arm')
        try:
            unsupported=validate_execution(plan,cell)
            self.assertEqual(unsupported.status,'unsupported')
        finally: cell.bindings['aux_arm']=saved


if __name__=='__main__': unittest.main()
