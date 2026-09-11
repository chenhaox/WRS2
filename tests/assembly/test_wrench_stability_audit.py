"""Independent physics audit: Cartesian forces, explicit Newton III equations.

The oracle does not use production wrench matrices, cone rays, LP columns,
or paired-variable elimination. It writes moments about the world origin,
and uses polygon HALFSPACES rather than nonnegative cone generators.
"""
import unittest
import numpy as np
from scipy.optimize import linprog
from test_stability_sweep import single,make_case,case_supports
from wrs.assembly import (StabilitySweepConfig,StabilityConfig,DirectionalStabilityAnalyzer,
                         disturbance_directions,limit_wrench,check_equilibrium,ExternalWrench)


def cross_matrix(p):
    x,y,z=p
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def cartesian_reference(analyzer,part_id,direction):
    """One LP with separate f_A and f_B, no production coefficient matrix."""
    cfg=analyzer.config
    assert cfg.mode=='wrench'
    sites=analyzer.nominal.force_sites
    slots=[]; size=0
    for site in sites:
        b=slice(size,size+3); size+=3
        a=slice(size,size+3) if site['part_a'] is not None else None
        if a is not None: size+=3
        slots.append((a,b))
    size+=1  # one radial amplitude
    eq=[]; rhs=[]; ub=[]; caps={}
    parts={p.part_id:p for p in analyzer.assembly.parts}
    for pid in analyzer.free:
        part=parts[pid]; tf=analyzer.state.poses[pid]
        com=tf[:3,:3]@part.com_local_m+tf[:3,3]
        row=np.zeros((6,size))
        for site,(a,b) in zip(sites,slots):
            for body,sl in ((site['part_a'],a),(site['part_b'],b)):
                if body==pid:
                    row[:3,sl]+=np.eye(3)
                    row[3:,sl]+=cross_matrix(site['point_world_m'])
        force=part.mass_kg*analyzer.assembly.gravity_world_m_s2
        torque=np.cross(com,force)
        for w in analyzer.external_wrenches:
            if w.part_id==pid:
                force=force+w.force_world_n
                torque=torque+w.torque_world_nm+np.cross(com,w.force_world_n)
        if pid==part_id:
            row[:3,-1]=direction[:3]
            row[3:,-1]=np.cross(com,direction[:3])+cfg.torque_length_m*direction[3:]
        eq.extend(row); rhs.extend(-np.r_[force,torque])
    for site,(a,b) in zip(sites,slots):
        if a is not None:
            row=np.zeros((3,size)); row[:,a]=np.eye(3); row[:,b]=np.eye(3)
            eq.extend(row); rhs.extend([0.,0.,0.])
        n=site['normal_on_b_world']; mu=site['friction']; sides=cfg.stability.friction_sides
        t1=np.cross(n,np.eye(3)[np.argmin(abs(n))]); t1/=np.linalg.norm(t1)
        t2=np.cross(n,t1)
        row=np.zeros(size); row[b]=-n; ub.append((row,0.))
        for angle in (2*np.arange(sides)+1)*np.pi/sides:
            row=np.zeros(size)
            row[b]=np.cos(angle)*t1+np.sin(angle)*t2-mu*np.cos(np.pi/sides)*n
            ub.append((row,0.))
        if site['max_group_normal_force_n'] is not None:
            group=caps.setdefault(site['capacity_group'],[np.zeros(size),site['max_group_normal_force_n']])
            group[0][b]+=n
    ub.extend(caps.values())
    objective=np.zeros(size); objective[-1]=-1.
    solved=linprog(objective,A_eq=np.array(eq),b_eq=np.array(rhs),
        A_ub=np.array([r for r,_ in ub]),b_ub=np.array([v for _,v in ub]),
        bounds=[(None,None)]*(size-1)+[(0,cfg.max_score_n)],method='highs',
        options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
    if not solved.success: raise AssertionError(solved.message)
    np.testing.assert_allclose(np.array(eq)@solved.x,rhs,atol=1e-7)
    assert np.max(np.array([r for r,_ in ub])@solved.x-np.array([v for _,v in ub]))<1e-7
    return solved.x[-1]


class WrenchStabilityAuditTests(unittest.TestCase):
    def test_true_six_dimensional_sampling_and_nested_antipodes(self):
        d=disturbance_directions(6500)
        np.testing.assert_allclose(np.linalg.norm(d,axis=1),1,atol=1e-14)
        np.testing.assert_array_equal(d[0::2],-d[1::2])
        np.testing.assert_array_equal(d[:61],disturbance_directions(61))
        np.testing.assert_array_equal(d[:12:2],np.eye(6))
        q=np.sum(d[12:,:3]**2,axis=1)
        self.assertLess(q.min(),.02); self.assertGreater(q.max(),.98)
        self.assertAlmostEqual(q.mean(),.5,delta=.01)
        self.assertAlmostEqual(q.var(),1/16,delta=.005)
        np.testing.assert_allclose(np.mean(d*d,axis=0),np.full(6,1/6),atol=.01)
        self.assertEqual(StabilitySweepConfig().backend,'cuda')
        self.assertEqual(StabilitySweepConfig().mode,'wrench')
        with self.assertRaises(ValueError): disturbance_directions(6,'wrench')

    def test_independent_cartesian_action_reaction_and_origin_equations(self):
        for name in ('stack','bridge','incline','floating'):
            with self.subTest(scene=name):
                a,s,g=make_case(name)
                cfg=StabilitySweepConfig(backend='numpy',direction_count=28,
                    stability=StabilityConfig(max_contact_normal_force_n=80))
                # Include a nonzero base force AND free moment about COM.
                loads=() if name=='floating' else (ExternalWrench(
                    next(p.part_id for p in a.parts if not p.fixed),(.1,0,0),(0,.001,0)),)
                analyzer=DirectionalStabilityAnalyzer(a,s,g,config=cfg,
                    supports=case_supports(name),external_wrenches=loads)
                result=analyzer.analyze()
                self.assertEqual(result.status,'complete')
                for case in result.cases:
                    reference=cartesian_reference(analyzer,case.part_id,result.directions[case.direction_index])
                    self.assertAlmostEqual(case.score_n,reference,delta=1e-6)

    def test_internal_wrenches_cancel_at_common_origin(self):
        a,s,g=make_case('bridge')
        analyzer=DirectionalStabilityAnalyzer(a,s,g,config=StabilitySweepConfig(backend='numpy'))
        sites=analyzer.nominal.force_sites; col=0; coms=[]
        for pid in analyzer.free:
            tf=s.poses[pid]; coms.append(tf[:3,:3]@analyzer.parts[pid].com_local_m+tf[:3,3])
        internal_columns=0
        for site in sites:
            width=len(site['rays_on_b_world'])
            if site['part_a'] in analyzer.index and site['part_b'] in analyzer.index:
                internal_columns+=width
                loads=(analyzer.physical[:,col:col+width]@np.arange(1,width+1)).reshape(-1,6)
                np.testing.assert_allclose(loads[:,:3].sum(axis=0),0,atol=1e-12)
                moment=loads[:,3:]+np.cross(coms,loads[:,:3])
                np.testing.assert_allclose(moment.sum(axis=0),0,atol=1e-12)
            col+=width
        result=analyzer.analyze(np.eye(6),part_ids=['beam'])
        self.assertEqual(result.diagnostics['eliminated_duplicate_ray_variables'],internal_columns)
        self.assertEqual(result.diagnostics['force_variables'],col)
        self.assertEqual(result.diagnostics['equilibrium_rows'],24)

    def test_numerical_row_scaling_is_not_the_physical_wrench_metric(self):
        a,s,g=single(); d=disturbance_directions(30)
        scores=[]
        for length in (.01,.1,1.):
            cfg=StabilitySweepConfig(backend='numpy',stability=StabilityConfig(characteristic_length_m=length))
            result=DirectionalStabilityAnalyzer(a,s,g,config=cfg).analyze(d)
            self.assertEqual(result.status,'complete')
            scores.append([c.score_n for c in result.cases])
        np.testing.assert_allclose(scores,np.broadcast_to(scores[0],np.shape(scores)),atol=1e-7)

    def test_legacy_sum_can_drop_torque_and_overrate_a_mixed_ray(self):
        a,s,g=single(); d=np.array([[0,0,1,1,0,0]])
        legacy=DirectionalStabilityAnalyzer(a,s,g,config=StabilitySweepConfig(
            backend='numpy',mode='legacy_coupled')).analyze(d)
        radial=DirectionalStabilityAnalyzer(a,s,g,config=StabilitySweepConfig(backend='numpy')).analyze(d)
        self.assertAlmostEqual(legacy.cases[0].force_n,19.62,places=7)
        self.assertAlmostEqual(legacy.cases[0].torque_nm,0,places=7)
        self.assertAlmostEqual(radial.cases[0].score_n,19.62*np.sqrt(2)/3,places=7)
        self.assertAlmostEqual(radial.cases[0].torque_nm/radial.cases[0].force_n,.1,places=7)
        self.assertIsNone(legacy.sampled_minimum_load_factor)
        self.assertAlmostEqual(radial.sampled_minimum_load_factor,radial.sampled_minimum_n/10)

    def test_metric_scales_and_limit_reconstruction(self):
        a,s,g=single(); scores=[]
        for length,ref in ((.1,10),(.2,10),(.1,20)):
            cfg=StabilitySweepConfig(backend='numpy',torque_length_m=length,force_reference_n=ref)
            r=DirectionalStabilityAnalyzer(a,s,g,config=cfg).analyze([[0,0,0,1,0,0]])
            load=limit_wrench(r,0)
            self.assertEqual(check_equilibrium(a,s,g,external_wrenches=(load,)).status,'feasible')
            self.assertAlmostEqual(load.torque_world_nm[0],.981,places=7)
            scores.append(r.sampled_minimum_load_factor)
        np.testing.assert_allclose(scores,[.981,.4905,.4905],atol=1e-7)

    def test_default_cuda_against_independent_cartesian_formulation(self):
        try: import torch
        except ImportError: self.skipTest('optional PyTorch unavailable')
        if not torch.cuda.is_available(): self.skipTest('optional CUDA unavailable')
        a,s,g=make_case('bridge')
        analyzer=DirectionalStabilityAnalyzer(a,s,g,config=StabilitySweepConfig(direction_count=36))
        r=analyzer.analyze()
        self.assertEqual(r.status,'complete'); self.assertIsNotNone(r.diagnostics['gpu_name'])
        for c in r.cases:
            self.assertAlmostEqual(c.score_n,cartesian_reference(analyzer,c.part_id,r.directions[c.direction_index]),delta=1e-6)
        self.assertLess(r.diagnostics['fallback_problems'],len(r.cases))


if __name__=='__main__': unittest.main()
