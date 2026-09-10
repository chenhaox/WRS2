"""Contracts shared by backend selection, data adapters and planner state."""
import subprocess
import sys
import unittest
from types import SimpleNamespace
import numpy as np
from wrs.assembly import (ContactModel, ContactAnalyzer, MeshContactBackend, ContactAnalysis,
                          Assembly, AssemblyState, Part, analyze_pair, analyze_contacts)
from wrs.assembly.primitives import box, pose


class BackendContractTests(unittest.TestCase):
    def test_mesh_facade_legacy_dispatch_and_absent_state(self):
        a = Part('a', box())
        b = Part('b', box(), pose((.025, 0, .1)))
        analyzer = ContactAnalyzer('mesh')
        result = analyzer.analyze_pair(a, b)
        self.assertAlmostEqual(sum(p.area_m2 for p in result.patches if p.classification == 'active'), .0075)
        other = analyze_pair(a, a.assembled_tf, b, b.assembled_tf, backend='mesh')
        self.assertEqual(result.state_digest, other.state_digest)
        self.assertEqual(other.pair_diagnostics[0]['contact_backend'], 'mesh')
        assembly = Assembly((a, b))
        empty = analyze_contacts(assembly, AssemblyState({'a': pose()}), backend=MeshContactBackend())
        self.assertEqual(empty.statistics['pair_count'], 0)
        rev = analyze_contacts(assembly, AssemblyState({'a': pose()}, world_revision=1), backend='mesh')
        self.assertNotEqual(empty.state_digest, rev.state_digest)
        with self.assertRaises(ValueError):
            analyzer.analyze((a, b), poses={'missing': pose()})

    def test_explicit_unit_adapter_and_pose_override(self):
        mesh = box()
        vertices = mesh.vertices*1000
        a = ContactModel.from_arrays(vertices, mesh.faces, name='a', length_unit='mm')
        vertices[:] = 0
        np.testing.assert_allclose(a.geometry.vertices, mesh.vertices)
        b = ContactModel(mesh, 'b')
        analyzer = ContactAnalyzer()
        touching = analyzer.analyze_pair(a, b, tf_b=pose((0, 0, .1)))
        away = analyzer.analyze_pair(a, b, tf_b=pose((0, 0, .2)))
        self.assertNotEqual(touching.state_digest, away.state_digest)
        self.assertFalse(away.patches)
        np.testing.assert_array_equal(b.tf, np.eye(4))
        with self.assertRaises(ValueError):
            analyzer.analyze_pair(a, a)
        with self.assertRaises(ValueError):
            ContactModel.from_arrays(mesh.vertices, mesh.faces, name='a', length_unit='auto')

    def test_wrs_visual_snapshot_preserves_local_offsets(self):
        mesh = box()
        geom = SimpleNamespace(vs=mesh.vertices.copy(), fs=mesh.faces)
        visual = SimpleNamespace(geom=geom, loc_tf=pose((.02, 0, 0)))
        obj = SimpleNamespace(visuals=[visual], name='obj', tf=pose((0, .3, 0)),
                              collisions=[object()])
        adapted = ContactModel.from_scene_object(obj)
        np.testing.assert_allclose(adapted.geometry.vertices, mesh.vertices+[.02, 0, 0])
        np.testing.assert_allclose(adapted.tf[:3, 3], [0, .3, 0])
        geom.vs[:] = 99
        self.assertLess(adapted.geometry.vertices.max(), .1)
        geom.fs = None
        with self.assertRaises(ValueError):
            ContactModel.from_scene_object(obj)

    def test_custom_backend_and_explicit_rejection(self):
        class Custom:
            name = 'custom'
            cache_key = 'custom-v1'
            def analyze_pair(self, a, ta, b, tb, *, config):
                return ContactAnalysis((), ({'part_a': a.name, 'part_b': b.name},), 'local')
        a, b = ContactModel(box(), 'a'), ContactModel(box(), 'b')
        result = ContactAnalyzer(Custom()).analyze_pair(a, b)
        self.assertEqual(result.pair_diagnostics[0]['contact_backend'], 'custom')
        with self.assertRaises(ValueError):
            ContactAnalyzer('missing')
        with self.assertRaises(TypeError):
            ContactAnalyzer(object())

    def test_imports_do_not_load_optional_geometry_or_viewer(self):
        code = ('import sys; from wrs.assembly import ContactAnalyzer, ContactModel, '
                'SDFContactBackend, GridSDF; '
                'assert not any(m in sys.modules for m in ("open3d","mujoco","wgpu"))')
        subprocess.run([sys.executable, '-c', code], check=True)
