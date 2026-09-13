"""Reduction preserves patch capacities; curved failures retry the full model."""
from dataclasses import replace
import gc
from pathlib import Path
import sys
import unittest
import weakref
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'examples'/'assembly'))
from _shared.stability_cases import make_case,case_config
from test_stability import scene,declared_contact
from wrs.assembly import (ContactPatch,ContactAnalysis,Region,StabilityConfig,ExternalWrench,
                          LoadCase,check_equilibrium,build_contact_graph)
from wrs.assembly.model import digest
from wrs.assembly.contact.graph import state_geometry_binding
from wrs.assembly.force_points import prepare_force_points


def patch_with(points,normals=None,dimension=2,cells=None):
    p = np.asarray(points,dtype=float)
    n = np.tile([0.,0,1.],(len(p),1)) if normals is None else np.asarray(normals)
    regions = () if cells is None else (Region(tuple(cells)),)
    return ContactPatch('floor','part','top','bottom',dimension,'active','bounded',p,p,n,-n,regions,(0.,0.))


class ForcePointTests(unittest.TestCase):
    def test_cleanup_is_unconditional_but_hull_is_optional(self):
        p = np.array([[-.1,-.1,0],[.1,-.1,0],[.1,.1,0],[-.1,.1,0],[0,0,0],
                      [.1,.1,0],[.1+1e-17,.1,0]])
        patch = patch_with(p,cells=[p])
        full,_ = prepare_force_points(patch,reduce=False)
        hull,_ = prepare_force_points(patch)
        self.assertEqual(len(full.points),5)
        self.assertEqual(len(hull.points),4)
        self.assertEqual(hull.info['method'],'planar_hull')
        with self.assertRaises(ValueError): hull.points[0,0] = 0
        with self.assertRaises(TypeError): hull.info['method'] = 'changed'

    def test_point_line_bent_line_and_normal_discontinuities(self):
        point,_ = prepare_force_points(patch_with([[0,0,0],[0,0,0]],dimension=0))
        self.assertEqual(len(point.points),1)
        line = patch_with([[-.1,0,0],[0,0,0],[.1,0,0]],dimension=1)
        a,_ = prepare_force_points(line); b,_ = prepare_force_points(line,reduce=False)
        self.assertEqual((len(a.points),len(b.points)),(2,3))
        bent,_ = prepare_force_points(patch_with([[-.1,0,0],[0,.01,0],[.1,0,0]],dimension=1))
        self.assertEqual(len(bent.points),3)
        sharp,_ = prepare_force_points(patch_with([[0,0,0],[0,0,0]],[[0,0,1],[1,0,0]],dimension=1))
        self.assertEqual(len(sharp.points),2)

    def test_hole_keeps_real_vertices_and_does_not_change_contact_geometry(self):
        a,s = scene()
        rects = [(-.05,.05,-.05,-.02),(-.05,.05,.02,.05),(-.05,-.02,-.02,.02),(.02,.05,-.02,.02)]
        cells = [np.array([[x0,y0,0],[x1,y0,0],[x1,y1,0],[x0,y1,0]]) for x0,x1,y0,y1 in rects]
        g = declared_contact(a,s,2,cells)
        before = digest(g)
        r = check_equilibrium(a,s,g)
        self.assertEqual(r.status,'feasible')
        self.assertEqual(len(r.force_sites),4)
        self.assertEqual(digest(g),before)
        source = np.concatenate(cells)
        for site in r.force_sites:
            self.assertTrue(np.any(np.all(site['point_world_m']==source,axis=1)))

    def test_complex_scenes_and_capacity_groups_with_both_policies(self):
        for name,expected_count in (('bridge',20),('tripod',36),('incline',4)):
            with self.subTest(name=name):
                a,s,g = make_case(name)
                cfg = case_config(name)
                reduced = check_equilibrium(a,s,g,config=cfg)
                full = check_equilibrium(a,s,g,config=replace(cfg,reduce_contact_points=False))
                self.assertEqual(len(reduced.force_sites),expected_count)
                self.assertGreater(len(full.force_sites),expected_count)
                self.assertEqual([c.status for c in (reduced.nominal,*reduced.disturbances)],
                                 [c.status for c in (full.nominal,*full.disturbances)])
                self.assertNotEqual(reduced.input_digest,full.input_digest)
                self.assertEqual({v['capacity_group'] for v in reduced.force_sites},
                                 {v['capacity_group'] for v in full.force_sites})
                for cap in (2.,100.):
                    statuses = [check_equilibrium(a,s,g,config=replace(cfg,reduce_contact_points=mode,
                        max_contact_normal_force_n=cap)).status for mode in (True,False)]
                    self.assertEqual(*statuses)

    def test_cache_is_immutable_patch_scoped_and_weak(self):
        patch = patch_with([[-.1,-.1,0],[.1,-.1,0],[.1,.1,0],[-.1,.1,0],[0,0,0]])
        first,hit = prepare_force_points(patch)
        self.assertFalse(hit)
        again,hit = prepare_force_points(patch)
        self.assertTrue(hit); self.assertIs(first,again)
        changed = replace(patch,points_a_world_m=patch.points_a_world_m+[1,0,0],
                          points_b_world_m=patch.points_b_world_m+[1,0,0])
        moved,hit = prepare_force_points(changed)
        self.assertFalse(hit)
        np.testing.assert_allclose(moved.points,first.points+[1,0,0])
        ref = weakref.ref(patch)
        del patch
        gc.collect()
        self.assertIsNone(ref())

    def test_curved_subset_is_deterministic_and_keeps_input_normals(self):
        angles = np.linspace(0,2*np.pi,80,endpoint=False)
        p = np.column_stack((.04*np.cos(angles),.04*np.sin(angles),np.zeros(80)))
        n = np.column_stack((.6*np.cos(angles),.6*np.sin(angles),np.full(80,.8)))
        patch = patch_with(p,n)
        selected,_ = prepare_force_points(patch,curved_budget=8,spacing=.0001,normal_angle=.001)
        clone = replace(patch)
        repeat,_ = prepare_force_points(clone,curved_budget=8,spacing=.0001,normal_angle=.001)
        np.testing.assert_array_equal(selected.points,repeat.points)
        self.assertEqual(len(selected.points),8)
        self.assertTrue(selected.info['curved_subset'])
        for point,normal in zip(selected.points,selected.normals):
            i = np.flatnonzero(np.all(p==point,axis=1))[0]
            np.testing.assert_array_equal(normal,n[i])

    def test_missing_curved_normal_falls_back_on_disturbance_failure(self):
        a,s = scene(friction=0.)
        a = replace(a,gravity_world_m_s2=(0,0,0))
        angles = np.linspace(0,2*np.pi,48,endpoint=False)
        p = np.column_stack((.04*np.cos(angles),.04*np.sin(angles),np.zeros(48)))
        n = np.column_stack((.6*np.cos(angles),.6*np.sin(angles),np.full(48,.8)))
        patch = patch_with(p,n)
        selected,_ = prepare_force_points(patch,curved_budget=8,spacing=.0001,normal_angle=.001)
        omitted = next(i for i in range(len(p)) if not np.any(np.all(selected.points==p[i],axis=1)))
        # With sum(fn)<=1 and friction=0, a force of unit length n[i] can only
        # be supplied by that exact normal (strict triangle inequality).
        com = s.poses['part'][:3,3]
        load = ExternalWrench('part',-n[omitted],-np.cross(p[omitted]-com,n[omitted]))
        analysis = ContactAnalysis((patch,),({'part_a':'floor','part_b':'part','overlap':{'status':'touching'}},),
            digest(p),input_binding=state_geometry_binding(a,s))
        g = build_contact_graph(a,s,analysis)
        cfg = StabilityConfig(curved_point_budget=8,curved_point_spacing_m=.0001,
            curved_normal_angle_rad=.001,max_contact_normal_force_n=1.,
            disturbances=(LoadCase('needs_missing_normal',(load,)),))
        r = check_equilibrium(a,s,g,config=cfg)
        self.assertTrue(r.diagnostics['curved_fallback'])
        self.assertEqual(r.diagnostics['reduced_attempt']['failed_loads'],('needs_missing_normal',))
        self.assertEqual(r.status,'feasible')
        self.assertEqual(r.robustness_status,'passed_tested_set')
        self.assertEqual(len(r.force_sites),48)
        for case in (r.nominal,*r.disturbances):
            for f in case.contact_forces:
                np.testing.assert_array_equal(r.force_sites[f['site_index']]['point_world_m'],f['point_world_m'])
        # An already-feasible subset needs no full retry.
        nominal = check_equilibrium(a,s,g,config=replace(cfg,disturbances=()))
        self.assertEqual(nominal.status,'feasible')
        self.assertFalse(nominal.diagnostics['curved_fallback'])
        self.assertEqual(len(nominal.force_sites),8)

    def test_config_validation(self):
        for kwargs in ({'reduce_contact_points':1},{'curved_point_budget':True},
                       {'curved_point_budget':3},{'curved_point_spacing_m':0},
                       {'curved_normal_angle_rad':np.pi}):
            with self.assertRaises(ValueError): StabilityConfig(**kwargs)


if __name__=='__main__': unittest.main()
