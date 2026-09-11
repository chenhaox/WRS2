"""Analytic balances, support capacities, friction and contact dimensionality."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
from wrs.assembly import (Assembly, Part, AssemblyState, Region, ContactPatch, ContactAnalysis,
                          StabilityConfig, ExternalWrench, LoadCase, SupportCandidate,
                          check_equilibrium, find_support_requirements, analyze_contacts, build_contact_graph)
from wrs.assembly.contact.graph import state_geometry_binding
from wrs.assembly.model import digest, to_dict
from wrs.assembly.primitives import box, pose


def scene(*, mass=1., friction=.5, floor=True, com=(0,0,0), floor_width=.3):
    part = Part('part', box(), pose((0,0,.05)), mass_kg=mass,
                com_local_m=com, friction=friction)
    parts = (part,)
    if floor:
        parts += (Part('floor', box((floor_width,.3,.02)), pose((0,0,-.01)), fixed=True, friction=friction),)
    assembly = Assembly(parts)
    return assembly, assembly.initial_state()


def graph_for(assembly, state):
    return build_contact_graph(assembly,state,analyze_contacts(assembly,state))


def declared_contact(assembly, state, dimension, cells):
    # Isolate the wrench space of declared point/line/ring contact models.
    points = np.concatenate(cells)
    normals = np.tile([0.,0,1],(len(points),1))
    p = ContactPatch('floor','part','top','bottom',dimension,'active','analytic',
                     points,points,normals,-normals,(Region(tuple(cells)),),(0,0))
    report = ContactAnalysis((p,),({'part_a':'floor','part_b':'part','overlap':{'status':'touching'}},),
                              digest((dimension,points)), input_binding=state_geometry_binding(assembly,state))
    return build_contact_graph(assembly,state,report)


class StabilityTests(unittest.TestCase):
    def test_box_weight_and_paired_reactions(self):
        a,s = scene(mass=2.)
        r = check_equilibrium(a,s,graph_for(a,s))
        self.assertEqual(r.status,'feasible')
        total = np.zeros(3)
        for f in r.nominal.contact_forces:
            np.testing.assert_allclose(f['force_on_a_world_n']+f['force_on_b_world_n'],0,atol=1e-12)
            total += f['force_on_b_world_n'] if f['part_b']=='part' else f['force_on_a_world_n']
        np.testing.assert_allclose(total,[0,0,19.62],atol=1e-8)
        for residual in r.nominal.body_residuals.values():
            np.testing.assert_allclose(residual['force_world_n'],0,atol=1e-8)
            np.testing.assert_allclose(residual['torque_world_nm'],0,atol=1e-8)
        self.assertEqual(r.robustness_status,'not_tested')
        self.assertFalse(r.execution_validated)

    def test_two_free_layers_and_no_internal_force_levitation(self):
        a,s = scene(mass=1.)
        upper = Part('upper',box(),pose((0,0,.15)),mass_kg=2.,com_local_m=(0,0,0),friction=.5)
        a = replace(a,parts=a.parts+(upper,)); s = a.initial_state()
        r = check_equilibrium(a,s,graph_for(a,s))
        self.assertEqual(r.status,'feasible')
        reaction = np.zeros(3)
        for f in r.nominal.contact_forces:
            if f['part_a']=='floor': reaction -= f['force_on_a_world_n']
            if f['part_b']=='floor': reaction -= f['force_on_b_world_n']
        np.testing.assert_allclose(reaction,[0,0,29.43],atol=1e-8)
        a = replace(a,parts=tuple(p for p in a.parts if p.part_id!='floor')); s = a.initial_state()
        self.assertEqual(check_equilibrium(a,s,graph_for(a,s)).status,'infeasible')

    def test_incline_friction_bounds(self):
        R = Rotation.from_rotvec([0,np.deg2rad(20),0]).as_matrix()
        for mu, expected in ((.2,'infeasible'),(.6,'feasible')):
            a,s = scene(friction=mu)
            transforms = {}
            for k,tf in s.poses.items():
                new = tf.copy(); new[:3,:3] = R @ tf[:3,:3]; new[:3,3] = R @ tf[:3,3]
                transforms[k] = new
            s = AssemblyState(transforms)
            r = check_equilibrium(a,s,graph_for(a,s))
            self.assertEqual(r.status,expected)

    def test_overhang_and_force_capacity(self):
        a,s = scene(com=(.04,0,0),floor_width=.04)
        self.assertEqual(check_equilibrium(a,s,graph_for(a,s)).status,'infeasible')
        a,s = scene()
        graph = graph_for(a,s)
        self.assertEqual(check_equilibrium(a,s,graph,config=StabilityConfig(max_contact_normal_force_n=8)).status,'infeasible')
        self.assertEqual(check_equilibrium(a,s,graph,config=StabilityConfig(max_contact_normal_force_n=12)).status,'feasible')

    def test_point_and_line_do_not_gain_free_moments(self):
        a,s = scene()
        for dimension,cells,torque in (
                (0,[np.array([[0.,0,0]])],[0,0,.1]),
                (1,[np.array([[-.04,0,0],[.04,0,0]])],[.1,0,0])):
            graph = declared_contact(a,s,dimension,cells)
            self.assertEqual(check_equilibrium(a,s,graph).status,'feasible')
            r = check_equilibrium(a,s,graph,external_wrenches=(ExternalWrench('part',torque_world_nm=torque),))
            self.assertEqual(r.status,'infeasible')

    def test_hole_is_not_filled_with_force_points(self):
        a,s = scene()
        rectangles = [(-.05,.05,-.05,-.02),(-.05,.05,.02,.05),(-.05,-.02,-.02,.02),(.02,.05,-.02,.02)]
        cells = [np.array([[x0,y0,0],[x1,y0,0],[x1,y1,0],[x0,y1,0]]) for x0,x1,y0,y1 in rectangles]
        r = check_equilibrium(a,s,declared_contact(a,s,2,cells))
        self.assertEqual(r.status,'feasible')  # The rim can support a COM above the hole.
        for force in r.nominal.contact_forces:
            x,y,z = force['point_world_m']
            self.assertFalse(abs(x)<.019 and abs(y)<.019)

    def test_unknown_mass_com_friction_and_near_do_not_carry(self):
        for kwargs in ({'mass':None},{'com':None},{'friction':None}):
            a,s = scene(**kwargs)
            self.assertEqual(check_equilibrium(a,s,graph_for(a,s)).status,'unknown')
        a,s = scene()
        moved = dict(s.poses); moved['part'] = pose((0,0,.0502)); s = AssemblyState(moved)
        r = check_equilibrium(a,s,graph_for(a,s))
        self.assertEqual(r.status,'infeasible')
        self.assertEqual(r.diagnostics['force_points'],0)
        a,s = scene(); graph = graph_for(a,s)
        unknown = replace(graph,edges=tuple(replace(e,issues=('overlap_unknown',)) for e in graph.edges))
        self.assertEqual(check_equilibrium(a,s,unknown).status,'unknown')

    def test_disturbances_separate_from_nominal(self):
        a,s = scene(friction=.3)
        cases = (LoadCase('small_push',(ExternalWrench('part',(1,0,0)),)),
                 LoadCase('large_push',(ExternalWrench('part',(10,0,0)),)))
        r = check_equilibrium(a,s,graph_for(a,s),config=StabilityConfig(disturbances=cases))
        self.assertEqual(r.status,'feasible')
        self.assertEqual([c.status for c in r.disturbances],['feasible','infeasible'])
        self.assertEqual(r.robustness_status,'failed_tested_set')

    def test_finite_support_search_and_budget(self):
        a,s = scene(floor=False); graph = graph_for(a,s)
        candidates = (SupportCandidate('left','part',[-.03,0,0],[0,0,1],5),
                      SupportCandidate('right','part',[.03,0,0],[0,0,1],5))
        limited = find_support_requirements(a,s,graph,candidates,max_subsets=3)
        self.assertEqual(limited.status,'budget_exhausted')
        found = find_support_requirements(a,s,graph,candidates,max_subsets=4)
        self.assertEqual(found.status,'requirements_found')
        self.assertEqual(found.selected_support_ids,('left','right'))
        self.assertTrue(found.minimal_within_declared_candidates)
        self.assertFalse(found.execution_validated)
        self.assertEqual(found.equilibrium.status,'feasible')
        for f in found.equilibrium.nominal.support_forces:
            self.assertLessEqual(f['normal_force_n'],5+1e-8)
        load_generator = (w for w in (ExternalWrench('part',(0,0,-10)),))
        insufficient = find_support_requirements(a,s,graph,candidates,external_wrenches=load_generator,max_subsets=4)
        self.assertEqual(insufficient.status,'no_subset_within_limits')

    def test_state_binding_and_physics_digest(self):
        a,s = scene(); graph = graph_for(a,s)
        baseline = check_equilibrium(a,s,graph)
        for changed in (replace(a,gravity_world_m_s2=(0,0,9.81)),
                        replace(a,parts=tuple(replace(p,mass_kg=2.) if not p.fixed else p for p in a.parts)),
                        replace(a,parts=tuple(replace(p,friction=.2) for p in a.parts))):
            r = check_equilibrium(changed,s,graph)
            self.assertNotEqual(r.input_digest,baseline.input_digest)
        moved = AssemblyState({k:pose((0,0,.2)) if k=='part' else tf for k,tf in s.poses.items()})
        with self.assertRaisesRegex(ValueError,'stale'):
            check_equilibrium(a,moved,graph)
        changed_graph = replace(graph,edges=tuple(replace(e,issues=('overlap_unknown',)) for e in graph.edges))
        self.assertNotEqual(check_equilibrium(a,s,changed_graph).input_digest,baseline.input_digest)

    def test_solver_failure_is_unknown_and_inputs_are_checked(self):
        a,s = scene(); graph = graph_for(a,s)
        failure = SimpleNamespace(success=False,status=4,message='numerical failure')
        with patch('wrs.assembly.stability.linprog',return_value=failure):
            self.assertEqual(check_equilibrium(a,s,graph).status,'unknown')
        def nonfinite(c, **kwargs):
            return SimpleNamespace(success=True,status=0,message='bad numeric result',x=np.full(len(c),np.nan))
        with patch('wrs.assembly.stability.linprog',side_effect=nonfinite):
            self.assertEqual(check_equilibrium(a,s,graph).status,'unknown')
        for kwargs in ({'friction_sides':3},{'force_tol_n':0},{'max_contact_normal_force_n':float('inf')}):
            with self.assertRaises(ValueError): StabilityConfig(**kwargs)
        with self.assertRaises(ValueError):
            SupportCandidate('bad','part',[0,0,0],[0,0,0],10)
        with self.assertRaises(ValueError):
            check_equilibrium(a,s,graph,external_wrenches=(ExternalWrench('absent'),))

    def test_all_fixed_is_vacuous_and_results_are_immutable(self):
        a,s = scene()
        a = replace(a,parts=tuple(replace(p,fixed=True) for p in a.parts))
        self.assertEqual(check_equilibrium(a,s,graph_for(a,s)).status,'feasible')
        a,s = scene(); r = check_equilibrium(a,s,graph_for(a,s))
        with self.assertRaises(ValueError):
            r.nominal.contact_forces[0]['point_world_m'].setflags(write=True)
        import json
        json.dumps(to_dict(r),allow_nan=False)

    def test_varying_normals_are_not_averaged(self):
        a,s = scene(friction=0.)
        points = np.array([[-.04,0,0],[.04,0,0]])
        normals = np.array([[.8,0,.6],[-.8,0,.6]])
        p = ContactPatch('floor','part','support','body',1,'active','bounded',
                         points,points,normals,-normals,(),(0,0))
        report = ContactAnalysis((p,),({'part_a':'floor','part_b':'part','overlap':{'status':'touching'}},),
                                  digest(normals),input_binding=state_geometry_binding(a,s))
        r = check_equilibrium(a,s,build_contact_graph(a,s,report))
        self.assertEqual(r.status,'feasible')
        for f,n in zip(r.nominal.contact_forces,normals):
            np.testing.assert_allclose(np.cross(f['force_on_b_world_n'],n),0,atol=1e-9)


if __name__ == '__main__':
    unittest.main()
