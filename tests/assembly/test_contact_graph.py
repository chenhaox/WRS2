"""State, provenance and backend-neutral contact graph contracts."""
import unittest
from dataclasses import replace
import numpy as np
from wrs.assembly import (Assembly, AssemblyState, Part, MatingRelation, ContactConfig,
                          ContactAnalysis, analyze_contacts, build_contact_graph)
from wrs.assembly.geometry.primitives import box, pose


def fixture():
    mesh = box()
    return Assembly((Part('base', mesh, fixed=True), Part('part', mesh, pose((0, 0, .1)))))


class GraphTests(unittest.TestCase):
    def test_instances_shared_geometry_and_complete_evidence(self):
        assembly = fixture()
        assembly = replace(assembly, mating_relations=(MatingRelation('base', 'part', 'planar'),))
        state = assembly.initial_state()
        result = analyze_contacts(assembly, state)
        graph = build_contact_graph(assembly, state, result)
        self.assertEqual(set(graph.nodes), {'base', 'part'})
        self.assertEqual(len(graph.edges), 1)
        self.assertEqual(graph.edges[0].patches, result.patches)
        self.assertEqual(graph.edges[0].mating_relations, assembly.mating_relations)
        self.assertEqual(graph.edges[0].diagnostics, result.pair_diagnostics)
        graph.assert_matches(assembly, state)
        with self.assertRaises(ValueError):
            graph.nodes['base']['tf'][0, 0] = 2
        other = analyze_contacts(assembly, state)
        self.assertEqual(build_contact_graph(assembly, state, other).state_digest, graph.state_digest)
        self.assertEqual(len(build_contact_graph(assembly, AssemblyState({'part': pose()}),
                                                analyze_contacts(assembly, AssemblyState({'part': pose()}))).edges), 0)

    def test_stale_geometry_pose_revision_and_unbound_input_rejected(self):
        assembly = fixture()
        state = assembly.initial_state()
        result = analyze_contacts(assembly, state, backend='mesh')
        graph = build_contact_graph(assembly, state, result)
        changes = [AssemblyState({**state.poses, 'part': pose((0, 0, .1001))}),
                   replace(state, world_revision=1), AssemblyState({'base': pose()})]
        for changed in changes:
            with self.assertRaisesRegex(ValueError, 'stale'):
                build_contact_graph(assembly, changed, result)
            with self.assertRaisesRegex(ValueError, 'stale'):
                graph.assert_matches(assembly, changed)
        changed = replace(assembly, parts=(assembly.parts[0], replace(assembly.parts[1], geometry=box((.1, .1, .09)))))
        with self.assertRaisesRegex(ValueError, 'stale'):
            build_contact_graph(changed, state, result)
        with self.assertRaisesRegex(ValueError, 'unbound'):
            build_contact_graph(assembly, state, ContactAnalysis((), (), 'fake'))
        cfg = ContactConfig(near_tol_m=.002)
        new = build_contact_graph(assembly, state, analyze_contacts(assembly, state, config=cfg, backend='mesh'))
        self.assertNotEqual(graph.state_digest, new.state_digest)
        changed = replace(assembly, parts=(replace(assembly.parts[0], fixed=False), assembly.parts[1]))
        with self.assertRaisesRegex(ValueError, 'fixed supports'):
            graph.assert_matches(changed, state)
        changed = replace(assembly, mating_relations=(MatingRelation('base', 'part', 'new'),))
        with self.assertRaisesRegex(ValueError, 'mating'):
            graph.assert_matches(changed, state)

    def test_unknown_interference_and_uncomputed_active_are_exposed(self):
        assembly = fixture()
        state = assembly.initial_state()
        result = analyze_contacts(assembly, state)
        diag = {**result.pair_diagnostics[0], 'overlap': {'status': 'unknown'}, 'unresolved_area_m2': .1}
        patch = replace(result.patches[0], classification='interference')
        graph = build_contact_graph(assembly, state, replace(result, patches=(patch,), pair_diagnostics=(diag,)))
        self.assertTrue({'overlap_unknown', 'unresolved_contact_area', 'patch_interference'} <= set(graph.edges[0].issues))
        missing = build_contact_graph(assembly, state, replace(result, patches=()))
        self.assertIn('active_contact_not_solved', missing.edges[0].issues)
