import unittest
import numpy as np
from wrs.assembly import Assembly, AssemblyState, Part, MatingRelation, analyze_contacts
from wrs.assembly.geometry.primitives import box, pose
from wrs.assembly.geometry.proximity import MeshProximity


class StateTests(unittest.TestCase):
    def test_explicit_state_and_design_mate(self):
        mesh = box()
        mate = MatingRelation('a','b','coaxial',{'clearance_m':.0002})
        assembly = Assembly((Part('a',mesh),Part('b',mesh,pose((0,0,.1)))),mating_relations=(mate,))
        backend = MeshProximity()
        touching = analyze_contacts(assembly,assembly.initial_state(),backend=backend)
        moved = AssemblyState({'a':pose(),'b':pose((0,0,.2))},world_revision=1)
        apart = analyze_contacts(assembly,moved,backend=backend)
        self.assertNotEqual(touching.state_digest,apart.state_digest)
        self.assertEqual(len(apart.patches),0)
        self.assertEqual(apart.mating_relations,(mate,))
        missing = analyze_contacts(assembly,AssemblyState({'a':pose()}),backend=backend)
        self.assertEqual(len(missing.pair_diagnostics),0)
        self.assertEqual(len(missing.mating_relations),0)
        with self.assertRaises(ValueError):
            analyze_contacts(assembly,AssemblyState({'other':pose()}))
        np.testing.assert_array_equal(assembly.parts[1].assembled_tf,pose((0,0,.1)))

    def test_cache_arrays_are_readonly(self):
        backend = MeshProximity()
        mesh = box()
        prep, surfaces = backend.prepare(mesh)
        for arr in (prep.normals,prep.areas_m2,surfaces[0].face_ids,backend.index(mesh).nodes[0].lo):
            with self.assertRaises(ValueError):
                arr.setflags(write=True)
