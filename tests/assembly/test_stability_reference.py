"""Independent Cartesian-force SOCP oracle and physical scene regressions.

Friction equation: Drake StaticFrictionConeConstraint, ||ft|| <= mu fn.
Unlike production, this uses 3 Cartesian unknowns per site and exact round
cones, not nonnegative weights over polyhedral generators. It checks fixed
contact equilibrium only, not Drake's variable-pose complementarity problem.
"""
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy import sparse

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'examples'/'assembly'))
from _stability_cases import make_case, case_config, case_supports
from wrs.assembly import check_equilibrium, ExternalWrench, StabilityConfig


def round_cone_reference(assembly,state,sites,loads=()):
    import clarabel
    free = [p for p in assembly.parts if not p.fixed]
    count = 3*len(sites)
    equilibrium = np.zeros((6*len(free),count))
    external = np.zeros((len(free),6))
    for i,part in enumerate(free):
        tf = state.poses[part.part_id]
        com = tf[:3,:3] @ part.com_local_m+tf[:3,3]
        external[i,:3] = part.mass_kg*assembly.gravity_world_m_s2
        for w in loads:
            if w.part_id == part.part_id:
                external[i] += np.r_[w.force_world_n,w.torque_world_nm]
        for j,site in enumerate(sites):
            sign = int(site['part_b']==part.part_id)-int(site['part_a']==part.part_id)
            x,y,z = site['point_world_m']-com
            moment = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            equilibrium[6*i:6*i+6,3*j:3*j+3] = sign*np.vstack((np.eye(3),moment))
    if not count:
        return 'infeasible' if np.max(np.abs(external),initial=0)>1e-8 else 'feasible'
    A, b = [equilibrium], [-external.ravel()]
    cones = [clarabel.ZeroConeT(6*len(free))]
    q = np.zeros(count)
    caps = {}
    for j,site in enumerate(sites):
        n = site['normal_on_b_world']
        sl = slice(3*j,3*j+3)
        q[sl] = n
        row = np.zeros((1,count)); row[0,sl] = -n
        A.append(row); b.append(np.zeros(1)); cones.append(clarabel.NonnegativeConeT(1))
        cone = np.zeros((4,count))
        cone[0,sl] = -site['friction']*n
        cone[1:,sl] = -(np.eye(3)-np.outer(n,n))
        A.append(cone); b.append(np.zeros(4)); cones.append(clarabel.SecondOrderConeT(4))
        cap = site['max_group_normal_force_n']
        if cap is not None:
            group = site['capacity_group']
            if group not in caps: caps[group] = (np.zeros(count),cap)
            caps[group][0][sl] = n
    for row,cap in caps.values():
        A.append(row[None]); b.append(np.array([cap])); cones.append(clarabel.NonnegativeConeT(1))
    settings = clarabel.DefaultSettings()
    settings.verbose = False
    settings.tol_gap_abs = settings.tol_feas = settings.tol_gap_rel = 1e-10
    # A strictly convex force cost avoids a flat optimum producing AlmostSolved
    # after removing redundant sites. Feasibility constraints are unchanged.
    result = clarabel.DefaultSolver(.001*sparse.eye(count,format='csc'),q,
        sparse.csc_matrix(np.vstack(A)),np.concatenate(b),cones,settings).solve()
    if str(result.status)=='PrimalInfeasible': return 'infeasible'
    if str(result.status)!='Solved': return 'unknown'
    if np.max(np.abs(equilibrium @ result.x+external.ravel()))>1e-6: return 'unknown'
    return 'feasible'


class StabilityReferenceTests(unittest.TestCase):
    def test_complex_scenes_against_analytic_outcomes_and_round_cones(self):
        expected = dict(stack='feasible',floating='infeasible',bridge='feasible',
                        overhang='infeasible',incline='feasible',slippery='infeasible',tripod='feasible')
        for name,status in expected.items():
            with self.subTest(name=name):
                a,s,g = make_case(name)
                cfg = case_config(name)
                result = check_equilibrium(a,s,g,config=cfg)
                self.assertEqual(result.status,status)
                self.assertEqual(round_cone_reference(a,s,result.force_sites),status)
                for case,load in zip(result.disturbances,cfg.disturbances):
                    expected_load = 'infeasible' if name=='overhang' or load==cfg.disturbances[-1] else 'feasible'
                    self.assertEqual(case.status,expected_load)
                    self.assertEqual(round_cone_reference(a,s,result.force_sites,load.wrenches),expected_load)
                self.check_returned_forces(a,s,result)

    def check_returned_forces(self,a,s,result):
        # Recompute world moments about the origin, not the production COM rows.
        totals = {p.part_id:np.r_[p.mass_kg*a.gravity_world_m_s2,
                  np.cross(s.poses[p.part_id][:3,:3] @ p.com_local_m+s.poses[p.part_id][:3,3],
                           p.mass_kg*a.gravity_world_m_s2)] for p in a.parts if not p.fixed}
        if result.status!='feasible':
            self.assertTrue(result.force_sites) # infeasible still has candidate force sites
            self.assertFalse(result.nominal.contact_forces)
            return
        for f in (*result.nominal.contact_forces,*result.nominal.support_forces):
            site = result.force_sites[f['site_index']]
            p,n,force = f['point_world_m'],site['normal_on_b_world'],f['force_on_b_world_n']
            fn = force @ n
            self.assertGreaterEqual(fn,-1e-8)
            self.assertLessEqual(np.linalg.norm(force-fn*n),site['friction']*fn+1e-8)
            for pid,sign in ((f.get('part_a'),-1),(f['part_b'],1)):
                if pid in totals: totals[pid] += sign*np.r_[force,np.cross(p,force)]
        for v in totals.values(): np.testing.assert_allclose(v,0,atol=1e-7)
        with self.assertRaises(ValueError): result.force_sites[0]['rays_on_b_world'][0,0] = 9

    def test_capacity_includes_all_generators_and_auxiliary_points(self):
        a,s,g = make_case('floating')
        supports = case_supports('floating')
        for selected,expected in ((supports[:1],'infeasible'),(supports,'feasible')):
            r = check_equilibrium(a,s,g,supports=selected)
            self.assertEqual(r.status,expected)
            self.assertEqual(round_cone_reference(a,s,r.force_sites),expected)
            self.assertEqual(sum(site['kind']=='support' for site in r.force_sites),len(selected))
            self.check_returned_forces(a,s,r)

    def test_inner_polygon_is_conservative_near_friction_limit(self):
        a,s,g = make_case('stack')
        # Only the top body receives a horizontal load, slightly inside its
        # exact Coulomb limit but outside the 16-sided inscribed polygon.
        phi = np.pi/16
        load = ExternalWrench('upper',.99*.5*2*9.81*np.array([np.cos(phi),np.sin(phi),0]))
        r = check_equilibrium(a,s,g,external_wrenches=(load,))
        self.assertEqual(r.status,'infeasible')
        self.assertEqual(round_cone_reference(a,s,r.force_sites,(load,)),'feasible')
        finer = check_equilibrium(a,s,g,external_wrenches=(load,),config=StabilityConfig(friction_sides=64))
        self.assertEqual(finer.status,'feasible')


if __name__=='__main__': unittest.main()
