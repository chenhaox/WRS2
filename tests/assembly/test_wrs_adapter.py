"""Actual WRS interfaces and compatibility regressions, without a renderer."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from wrs.assembly import Part,Assembly,AssemblyState
from wrs.assembly.primitives import box,pose
from wrs.assembly.adapters.wrs_scene import (part_from_scene_object,scene_object_from_part,
                                            apply_state_to_scene,rigid_tf_from_wrs)


class WRSAdapterTests(unittest.TestCase):
    def test_multiple_visuals_and_shared_geometry(self):
        from wrs.scene.scene_object import SceneObject
        from wrs.scene.render_model import RenderModel
        mesh=box((.02,.03,.04)); first=RenderModel(geom=(mesh.vertices.astype('f4'),mesh.faces.astype('u4')))
        second=RenderModel(geom=first.geom,pos=(.06,0,0),rotmat=Rotation.from_euler('z',.3).as_matrix())
        obj=SceneObject(); obj.add_visual(first); obj.add_visual(second)
        obj.tf=pose((.2,.3,.4),Rotation.from_euler('y',.7).as_matrix()).astype('f4')
        before=first.geom.vs.copy(); part=part_from_scene_object(obj,'two')
        self.assertEqual(len(part.geometry.faces),24)
        expected=np.vstack([np.asarray(v.geom.vs)@np.asarray(v.loc_tf[:3,:3]).T+v.loc_tf[:3,3] for v in obj.visuals])
        np.testing.assert_allclose(part.geometry.vertices,expected,atol=1e-8)
        copy=scene_object_from_part(part,collision=False)
        apply_state_to_scene(AssemblyState({'two':pose((1,2,3))}),{'two':copy})
        np.testing.assert_array_equal(first.geom.vs,before)
        np.testing.assert_allclose(obj.pos,[.2,.3,.4],atol=1e-7)
        np.testing.assert_allclose(copy.pos,[1,2,3])

    def test_nonrigid_rejected_roundoff_allowed(self):
        tf=pose(rotation=Rotation.from_euler('xyz',[.3,.2,.5]).as_matrix()).astype('f4')
        r=rigid_tf_from_wrs(tf)
        np.testing.assert_allclose(r[:3,:3].T@r[:3,:3],np.eye(3),atol=1e-14)
        bad=np.eye(4); bad[0,0]=1.01
        with self.assertRaises(ValueError): rigid_tf_from_wrs(bad)

    def test_inline_mesh_collision_assets_remain_distinct(self):
        from wrs.collider.mj_collider import MJCollider
        small=scene_object_from_part(Part('small',box((.02,.02,.02)),pose()))
        large=scene_object_from_part(Part('large',box((.1,.1,.1)),pose((.4,0,0)),fixed=True))
        collider=MJCollider(); collider.append(small); collider.append(large); collider.compile()
        model=collider._mjenv.runtime.model
        self.assertEqual(model.nmesh,2)
        extents=[np.ptp(model.mesh_vert[start:start+count],axis=0) for start,count in zip(model.mesh_vertadr,model.mesh_vertnum)]
        np.testing.assert_allclose(sorted(float(x.max()) for x in extents),[.02,.1],atol=1e-7)

    def test_planning_context_exact_and_disabled_cache(self):
        from wrs.motion.core.planning_context import PlanningContext
        class Collider:
            actors=[object()]
            def __init__(self): self.calls=0; self.threshold=.1002
            def is_collided(self,q): self.calls+=1; return q[0]>self.threshold
        for cache_size in (0,100):
            c=Collider(); ctx=PlanningContext(c,([-1],[1]),cache_size=cache_size,cache_decimals=None)
            self.assertTrue(ctx.is_state_valid([.1001])); self.assertFalse(ctx.is_state_valid([.1003]))
            self.assertEqual(c.calls,2)
            c.threshold=0; ctx.clear_cache(); self.assertFalse(ctx.is_state_valid([.1001]))
        old=PlanningContext(Collider(),([-1],[1]))
        self.assertEqual(old.cache_decimals,3)
        self.assertTrue(old.is_motion_valid(np.array([-.3]),np.array([-.1])))


if __name__=='__main__': unittest.main()
