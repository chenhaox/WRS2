from dataclasses import replace
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from wrs.assembly import Assembly,Part,AssemblyState,analyze_contacts,build_contact_graph
from wrs.assembly.primitives import box,pose,cylinder
from wrs.assembly.model import digest
from wrs.assembly.part_motion import (MotionConfig,ContactPolicy,plan_removal,validate_object_path,
                                       interpolate_pose,RemovalAction)


def scene(obstacles=(),size=(.02,.02,.02),start=(0,0,.02)):
    return Assembly((Part('part',box(size),pose(start),mass_kg=.1,com_local_m=(0,0,0),friction=.5),*obstacles))


class PartMotionTests(unittest.TestCase):
    def test_removal_and_reverse_preserve_input(self):
        floor=Part('floor',box((.3,.3,.02)),pose((0,0,-.01)),fixed=True)
        a=scene((floor,),start=(0,0,.01)); s=a.initial_state(); before=digest((a,s))
        r=plan_removal(a,s,'part')
        self.assertEqual(r.status,'success'); self.assertEqual(digest((a,s)),before)
        self.assertTrue(r.policy.planes)
        outside=AssemblyState(dict(s.poses,part=r.poses[-1]))
        v=validate_object_path(a,outside,'part',r.poses[::-1],policy=r.policy)
        self.assertEqual(v.status,'valid')
        with self.assertRaises(ValueError): r.poses[0][0,0]=7

    def test_slide_and_inward_motion(self):
        floor=Part('floor',box((.3,.3,.02)),pose((0,0,-.01)),fixed=True)
        a=scene((floor,),start=(0,0,.01)); s=a.initial_state()
        g=build_contact_graph(a,s,analyze_contacts(a,s)); policy=ContactPolicy.from_graph(a,s,'part',g)
        end=pose((.06,0,.01))
        self.assertEqual(validate_object_path(a,s,'part',[s.poses['part'],end],policy=policy).status,'valid')
        bad=pose((0,0,-.01))
        self.assertEqual(validate_object_path(a,s,'part',[s.poses['part'],bad],policy=policy).status,'blocked')
        self.assertNotEqual(validate_object_path(a,s,'part',[s.poses['part'],end]).status,'valid')

    def test_remote_thin_obstacle_no_tunneling(self):
        wall=Part('wall',box((.0002,.2,.2)),pose((.123,0,.02)),fixed=True)
        a=scene((wall,)); s=a.initial_state()
        v=validate_object_path(a,s,'part',[s.poses['part'],pose((.3,0,.02))])
        self.assertEqual(v.status,'blocked')
        self.assertEqual(v.witness['obstacle_id'],'wall')

    def test_containment_is_not_free(self):
        enclosing=Part('solid',box((1,1,1)),pose((0,0,0)),fixed=True)
        a=scene((enclosing,)); s=a.initial_state()
        v=validate_object_path(a,s,'part',[s.poses['part']])
        self.assertEqual(v.status,'blocked')

    def test_rotation_middle_collision(self):
        obstacle=Part('peg',box((.025,.025,.10)),pose((.07,.07,.02)),fixed=True)
        a=scene((obstacle,),size=(.22,.015,.015)); s=a.initial_state()
        end=pose((0,0,.02),Rotation.from_euler('z',90,degrees=True).as_matrix())
        v=validate_object_path(a,s,'part',[s.poses['part'],end])
        self.assertEqual(v.status,'blocked')
        np.testing.assert_allclose(interpolate_pose(s.poses['part'],end,.5)[:3,:3].T@
                                   interpolate_pose(s.poses['part'],end,.5)[:3,:3],np.eye(3),atol=1e-12)

    def test_budget_and_unsupported_are_explicit(self):
        wall=Part('wall',box((.0002,.2,.2)),pose((.123,0,.02)),fixed=True)
        a=scene((wall,)); s=a.initial_state()
        v=validate_object_path(a,s,'part',[s.poses['part'],pose((.3,0,.02))],config=MotionConfig(max_nodes=1))
        self.assertEqual(v.status,'unknown')
        r=plan_removal(a,s,RemovalAction(('part','wall')))
        self.assertEqual(r.status,'unsupported')

    def test_stale_policy_and_invalid_input(self):
        a=scene(); s=a.initial_state(); r=plan_removal(a,s,'part')
        moved=replace(s,world_revision=1)
        with self.assertRaises(ValueError): validate_object_path(a,moved,'part',r.poses,policy=r.policy)
        with self.assertRaises(ValueError): MotionConfig(max_nodes=0)
        with self.assertRaises(ValueError): plan_removal(a,s,'part',directions=[[0,0,0]])

    def test_repeatable_finite_search_failure(self):
        wall=Part('wall',box((.01,.2,.2)),pose((.1,0,.02)),fixed=True)
        a=scene((wall,)); s=a.initial_state()
        x=plan_removal(a,s,'part',directions=[[1,0,0]])
        y=plan_removal(a,s,'part',directions=[[1,0,0]])
        self.assertEqual(x.status,'unknown'); self.assertEqual(x.input_digest,y.input_digest)
        self.assertEqual(x.diagnostics['attempts'][0]['status'],'blocked')

    def test_real_annular_hole_and_closed_cage(self):
        sleeve=Part('sleeve',cylinder(.04,.025,8,inner_radius=.025),pose(),fixed=True)
        a=scene((sleeve,),size=(.01,.01,.02),start=(0,0,0)); state=a.initial_state()
        result=validate_object_path(a,state,'part',[pose(),pose((0,0,.08))])
        self.assertEqual(result.status,'valid',result.reason)
        walls=[]
        for axis in range(3):
            size=np.full(3,.12); size[axis]=.01
            for sign in (-1,1):
                pos=np.zeros(3); pos[axis]=sign*.05
                walls.append(Part(f'wall_{axis}_{sign}',box(size),pose(pos),fixed=True))
        closed=scene(walls,start=(0,0,0))
        r=plan_removal(closed,closed.initial_state(),'part',config=MotionConfig(search_se3=False,max_candidates=6))
        self.assertEqual(r.status,'unknown')
        self.assertTrue(all(x['status']=='blocked' for x in r.diagnostics['attempts']))


if __name__=='__main__': unittest.main()
