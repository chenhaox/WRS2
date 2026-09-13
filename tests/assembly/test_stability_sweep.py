from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'examples'/'assembly'))
from _shared.stability_cases import make_case,block,case_supports
from wrs.assembly import (Assembly,StabilityConfig,StabilitySweepConfig,DirectionalStabilityAnalyzer,
    analyze_directional_stability,disturbance_directions,limit_wrench,ExternalWrench,
    analyze_contacts,build_contact_graph,check_equilibrium)


def single(mu=.5):
    a=Assembly((block('box',(.1,.1,.1),(0,0,.05),2.,friction=mu),
                block('floor',(.3,.3,.02),(0,0,-.01),fixed=True,friction=mu)))
    s=a.initial_state(); return a,s,build_contact_graph(a,s,analyze_contacts(a,s))


def cpu_config(**kwargs):
    return StabilitySweepConfig(**(dict(backend='numpy',mode='force')|kwargs))


class DirectionalStabilityTests(unittest.TestCase):
    def test_force_limits_against_analytic_sliding_and_lift(self):
        a,s,g=single(); r=analyze_directional_stability(a,s,g,config=cpu_config(direction_count=6))
        self.assertEqual(r.status,'complete')
        np.testing.assert_allclose([c.score_n for c in r.cases[:5]], [9.81,9.81,19.62,9.81,9.81],atol=1e-7)
        self.assertEqual(r.cases[5].status,'limit_reached')
        self.assertEqual(r.diagnostics['fallback_problems'],0)
        load=limit_wrench(r,0)
        self.assertEqual(check_equilibrium(a,s,g,external_wrenches=(load,)).status,'feasible')
        above=ExternalWrench('box',load.force_world_n*1.01)
        self.assertEqual(check_equilibrium(a,s,g,external_wrenches=(above,)).status,'infeasible')

    def test_torque_units_and_tipping(self):
        a,s,g=single(); r=analyze_directional_stability(a,s,g,directions=[[1,0,0]],
                          config=cpu_config(mode='torque',torque_length_m=.2))
        self.assertAlmostEqual(r.cases[0].torque_nm,19.62*.05,places=7)
        self.assertAlmostEqual(r.cases[0].score_n,19.62*.05/.2,places=7)
        self.assertEqual(r.cases[0].force_n,0.)

    def test_stack_transfer_and_legacy_vs_highs(self):
        a,s,g=make_case('stack')
        for mode in ('force','torque','wrench','legacy_coupled'):
            cfg=cpu_config(direction_count=36,mode=mode)
            reference=analyze_directional_stability(a,s,g,config=replace(cfg,backend='highs'))
            batched=analyze_directional_stability(a,s,g,config=cfg)
            self.assertEqual(reference.status,'complete'); self.assertEqual(batched.status,'complete')
            np.testing.assert_allclose([c.score_n for c in batched.cases],[c.score_n for c in reference.cases],atol=1e-6)
            if mode=='legacy_coupled':
                for c in batched.cases: self.assertLessEqual(c.torque_nm,np.sqrt(3)*.05*c.force_n+1e-7)
        up=analyze_directional_stability(a,s,g,directions=[[0,0,1]],config=cpu_config())
        np.testing.assert_allclose([c.force_n for c in up.cases],[29.43,19.62],atol=1e-7)

    def test_capacity_ceiling_is_not_reported_as_physical_limit(self):
        a,s,g=single(); cfg=cpu_config(max_score_n=1)
        r=analyze_directional_stability(a,s,g,directions=[[0,0,1]],config=cfg)
        self.assertTrue(r.diagnostics['minimum_is_lower_bound'])
        self.assertEqual(r.cases[0].status,'limit_reached')
        capped=analyze_directional_stability(a,s,g,directions=[[0,0,-1]],
                 config=replace(cfg,max_score_n=100,stability=StabilityConfig(max_contact_normal_force_n=25)))
        self.assertAlmostEqual(capped.cases[0].force_n,25-19.62,places=7)

    def test_frictionless_rank_deficiency_falls_back(self):
        a,s,g=single(0); r=analyze_directional_stability(a,s,g,directions=[[1,0,0],[0,0,1]],config=cpu_config())
        self.assertEqual(r.status,'complete'); self.assertEqual(r.diagnostics['fallback_problems'],2)
        np.testing.assert_allclose([c.force_n for c in r.cases],[0,19.62],atol=1e-7)

    def test_iteration_failure_never_means_zero_capacity(self):
        a,s,g=single(); cfg=cpu_config(max_iterations=1,fallback_to_highs=False)
        r=analyze_directional_stability(a,s,g,config=cfg,directions=[[1,0,0]])
        self.assertEqual(r.status,'partial'); self.assertIsNone(r.sampled_minimum_n)
        self.assertEqual(r.cases[0].status,'unknown'); self.assertIsNone(r.cases[0].score_n)
        ok=analyze_directional_stability(a,s,g,config=replace(cfg,fallback_to_highs=True),directions=[[1,0,0]])
        self.assertEqual(ok.status,'complete'); self.assertEqual(ok.diagnostics['fallback_problems'],1)

    def test_finite_auxiliary_supports_and_unstable_base(self):
        a,s,g=make_case('floating')
        self.assertEqual(analyze_directional_stability(a,s,g,config=cpu_config()).status,'unknown')
        r=analyze_directional_stability(a,s,g,supports=case_supports('floating'),directions=[[0,0,-1]],part_ids=['lower'],config=cpu_config())
        self.assertEqual(r.status,'complete'); self.assertAlmostEqual(r.cases[0].force_n,40-29.43,places=6)

    def test_repeatability_inputs_and_reused_model(self):
        a,s,g=single(); analyzer=DirectionalStabilityAnalyzer(a,s,g,config=cpu_config())
        p=analyzer.analyze([[1,0,0]]); q=analyzer.analyze([[1,0,0]])
        self.assertEqual(p.input_digest,q.input_digest); self.assertEqual(p.cases,q.cases)
        np.testing.assert_array_equal(disturbance_directions(37,'wrench'),disturbance_directions(37,'wrench'))
        with self.assertRaises(ValueError): analyzer.analyze([[0,0,0]])
        with self.assertRaises(ValueError): analyzer.analyze([[1,0,0]],part_ids=['floor'])
        with self.assertRaises(ValueError): analyzer.analyze([[float('nan'),0,0]])
        with self.assertRaises(ValueError): p.directions[0,0]=4
        with self.assertRaises(ValueError): StabilitySweepConfig(direction_count=2)

    def test_cuda_agrees_when_available(self):
        try: import torch
        except ImportError: self.skipTest('optional PyTorch unavailable')
        if not torch.cuda.is_available(): self.skipTest('optional CUDA unavailable')
        a,s,g=make_case('bridge'); cfg=StabilitySweepConfig(mode='legacy_coupled',direction_count=24)
        cpu=analyze_directional_stability(a,s,g,config=replace(cfg,backend='highs'))
        gpu=analyze_directional_stability(a,s,g,config=replace(cfg,backend='cuda'))
        self.assertEqual(gpu.status,'complete'); self.assertIsNotNone(gpu.diagnostics['gpu_name'])
        np.testing.assert_allclose([c.score_n for c in gpu.cases],[c.score_n for c in cpu.cases],atol=1e-6)
        self.assertLess(gpu.diagnostics['fallback_problems'],len(gpu.cases))


if __name__=='__main__': unittest.main()
