"""Geometric regressions exposed by the paper's physical assemblies."""
from dataclasses import replace
from hashlib import sha256
from pathlib import Path
import json
import sys
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'examples'/'assembly'))
from wrs.assembly import Assembly,Part,ContactConfig,analyze_contacts,build_contact_graph,assembly_directions
from wrs.assembly.geometry.primitives import box,pose
from wrs.assembly.geometry.preprocess import prepare_mesh
from _shared.paper_cases import ASSETS,CATALOG,make_case,prefix_state
from tools.assembly.paper_assets import _random_parts,mesh


class PaperExamplesTests(unittest.TestCase):
    def test_nine_physical_cases_both_methods(self):
        from _shared.directions import compute
        from _shared.direction_cases import CASES
        for key in CASES:
            with self.subTest(case=key): compute(key)

    def test_source_assets_are_unmodified_and_missing_sources_explicit(self):
        provenance=json.loads((ASSETS/'provenance.json').read_text(encoding='utf-8'))
        for item in provenance['assets']:
            path=ASSETS/item['copy']
            self.assertEqual(sha256(path.read_bytes()).hexdigest(),item['sha256'])
        for key in ('fig11_bridge6','fig12a','fig12b','fig13c_replacement'):
            with self.assertRaises(FileNotFoundError): make_case(key,nominal=False)

    def test_soma_grid_repair_preserves_poses_volume_and_closed_boundary(self):
        scene=json.loads((ASSETS/'scenes'/'datainfo2.json').read_text())
        a=make_case('fig08_soma3')
        for p in a.parts:
            if p.fixed: continue
            np.testing.assert_allclose(p.assembled_tf[:3,3],np.array(scene[p.part_id]['location'])*.001,atol=0)
        for name,count in (('bigL',4),('smallL_2',3),('Z',4),('T',4),('cross',4),('crossleft',4)):
            prepared=prepare_mesh(mesh(name,True))
            self.assertTrue(prepared.is_closed)
            self.assertTrue(prepared.orientation_reliable)
            triangles=prepared.mesh.vertices[prepared.mesh.faces]
            volume=np.einsum('ij,ij->i',triangles[:,0],np.cross(triangles[:,1],triangles[:,2])).sum()/6
            self.assertAlmostEqual(volume,count*.0185**3,places=14)

    def test_seeded_voxels_repeat_without_overlap_and_are_face_connected(self):
        a,ma=_random_parts(7,42); b,mb=_random_parts(7,42)
        self.assertEqual(ma,mb)
        self.assertEqual([p.geometry.geometry_id for p in a],[p.geometry.geometry_id for p in b])
        occupied=set()
        for cells in ma['voxels']:
            new=set(map(tuple,cells))
            self.assertFalse(occupied & new)
            if occupied:
                self.assertTrue(any(sum(abs(np.array(x)-y))==1 for x in occupied for y in new))
            occupied|=new

    def test_twenty_figures_and_explicit_prefixes(self):
        self.assertEqual(len(CATALOG),20)
        for key in CATALOG:
            a=make_case(key); order=a.provenance['order']
            for stage in range(1,len(order)+1):
                self.assertEqual(set(prefix_state(a,stage).poses),set(order[:stage])|{'ground'})
            with self.assertRaises(ValueError): prefix_state(a,len(order)+1)

    def test_nominal_soma_domino_bridge_and_offset_beam_directions(self):
        from _shared.paper import compute
        for key in ('fig08_soma3','fig09_domino3','fig11_bridge6','fig12g','fig12h'):
            data=compute(key)
            for pid,results in data[3].items():
                for method,result in results.items():
                    self.assertEqual(result.status,'feasible',(key,pid,method,result.issues))
            if key=='fig12h':
                # The beam overhang leaves only 10mm on its left leg, 20mm on right.
                patches=[p for p in data[2].patches if 'ground' not in (p.part_a,p.part_b)]
                self.assertEqual(len(patches),2)
                for patch,width in zip(sorted(patches,key=lambda p:p.area_m2),(.01,.02)):
                    self.assertEqual(patch.quality,'bounded')
                    self.assertAlmostEqual(patch.area_m2,width*(.06-2e-9),places=13)

    def test_burr_interference_is_not_a_certified_direction(self):
        from _shared.paper import compute
        data=compute('fig10_burr6',nominal=False)
        self.assertTrue(any(d['overlap']['status']=='penetrating' for d in data[2].pair_diagnostics))
        self.assertTrue(all(r['socp'].status=='unknown' for r in data[3].values()))


class SupportingFeatureTests(unittest.TestCase):
    def scene(self, *, vertex=False, offset=0., rotation=None):
        r=Rotation.from_euler('xyz',[.23,.35,.18] if vertex else [0,.35,0]).as_matrix()
        block=box((.04,.06,.08))
        height=-(block.vertices@r.T)[:,2].min()+offset
        parts=[Part('floor',box((.3,.3,.02)),pose((0,0,-.01)),fixed=True),Part('moving',block,pose((0,0,height),r))]
        if rotation is not None:
            parts=[replace(p,assembled_tf=rotation@p.assembled_tf) for p in parts]
        return Assembly(tuple(parts))

    def test_tilted_edge_and_vertex_are_zero_area_contacts(self):
        for vertex in (False,True):
            a=self.scene(vertex=vertex); s=a.initial_state(); c=analyze_contacts(a,s)
            self.assertEqual(len(c.patches),1)
            p=c.patches[0]
            self.assertEqual((p.dimension,p.area_m2),(0 if vertex else 1,0))
            self.assertAlmostEqual(p.length_m,0 if vertex else .06,places=12)
            np.testing.assert_allclose(p.normals_a_world,[[0,0,1]],atol=1e-12)
            np.testing.assert_allclose(p.normals_a_world,-p.normals_b_world,atol=0)
            result=assembly_directions(build_contact_graph(a,s,c),('moving',))
            self.assertEqual(result.status,'feasible')
            self.assertGreater(result.best_direction[2],.99)

    def test_rigid_transform_covariance_and_swapped_parts(self):
        from wrs.assembly.contact.analysis import analyze_pair
        transform=pose((.2,-.3,.5),Rotation.from_euler('xyz',[.2,.5,.7]).as_matrix())
        a=self.scene(rotation=transform); p,q=a.parts
        expected=transform[:3,2]
        for left,right in ((p,q),(q,p)):
            c=analyze_pair(left,left.assembled_tf,right,right.assembled_tf)
            self.assertEqual(len(c.patches),1)
            patch=c.patches[0]
            n=patch.normals_a_world[0]*(1 if patch.part_a=='floor' else -1)
            np.testing.assert_allclose(n,expected,atol=1e-12)
            self.assertAlmostEqual(patch.length_m,.06,places=12)

    def test_gap_and_penetration_never_promote_to_active_feature(self):
        for gap in (3e-7,-.001):
            a=self.scene(offset=gap); c=analyze_contacts(a,a.initial_state())
            self.assertFalse(any(p.provenance.get('backend')=='support_face/1' for p in c.patches))
            self.assertEqual(c.pair_diagnostics[0]['overlap']['status'],'separated' if gap>0 else 'penetrating')

    def test_boundary_corner_and_budget_do_not_create_unsupported_normals(self):
        from wrs.assembly.geometry.proximity import MeshProximity
        from wrs.assembly.geometry.mesh_bvh import QueryBudget
        from wrs.assembly.contact.support import support_face_contacts
        a=self.scene(); backend=MeshProximity(); p,q=a.parts
        pp,sp=backend.prepare(p.geometry); pq,_=backend.prepare(q.geometry)
        spent=QueryBudget(1); spent.consume()
        patches,exhausted=support_face_contacts(p,p.assembled_tf,pp,sp,q,q.assembled_tf,pq,
            config=ContactConfig(),budget=spent)
        self.assertTrue(exhausted); self.assertEqual(patches,())
        q=Part('corner',box((.04,.04,.04)),pose((.17,.17,.02)))
        pq,_=backend.prepare(q.geometry)
        patches,exhausted=support_face_contacts(p,p.assembled_tf,pp,sp,q,q.assembled_tf,pq,
            config=ContactConfig(),budget=QueryBudget(100))
        self.assertFalse(exhausted); self.assertEqual(patches,())


if __name__=='__main__': unittest.main()
