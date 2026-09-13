import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from wrs.assembly import Assembly, MeshData, Part, load_assembly, save_assembly
from wrs.assembly.io import read_mesh
from wrs.assembly.primitives import box, pose
from wrs.assembly.geometry.transforms import rotation_xyz


class InputTests(unittest.TestCase):
    def test_roundtrip_and_units(self):
        assembly = Assembly((Part('a', box(), pose((.1,.2,.3))), Part('b', box())))
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)/'assembly.json'
            save_assembly(assembly, p)
            m = load_assembly(p)
            data = json.loads(p.read_text())
            data['length_unit'] = 'mm'
            for item in data['parts']:
                item['geometry']['vertices'] = (np.asarray(item['geometry']['vertices'])*1000).tolist()
                tf = np.asarray(item['assembled_tf'])
                tf[:3,3] *= 1000
                item['assembled_tf'] = tf.tolist()
            p.write_text(json.dumps(data))
            mm = load_assembly(p)
            for a, b in zip(m.parts, mm.parts):
                np.testing.assert_allclose(a.geometry.vertices, b.geometry.vertices)
                np.testing.assert_allclose(a.assembled_tf, b.assembled_tf)
            self.assertIsNone(mm.parts[0].mass_kg)
            with self.assertRaises(ValueError):
                load_assembly(p, length_unit='m')
            del data['length_unit']
            p.write_text(json.dumps(data))
            with self.assertRaises(ValueError):
                load_assembly(p)

    def test_validation_and_immutable_arrays(self):
        a = np.array([[0.,0,0], [1,0,0], [0,1,0]])
        mesh = MeshData(a, [[0,1,2]])
        a[0] = 99
        self.assertEqual(mesh.vertices[0,0], 0)
        with self.assertRaises(ValueError):
            mesh.vertices.setflags(write=True)
        for faces in ([[0,1,3]], [[0,1,-1]], [[0,1,1.5]]):
            with self.assertRaises(ValueError):
                MeshData(a, faces)
        with self.assertRaises(ValueError):
            MeshData([[np.nan,0,0], [1,0,0], [0,1,0]], [[0,1,2]])
        for tf in (np.diag([2,1,1,1]), np.diag([-1,1,1,1]), np.ones((4,4))):
            with self.assertRaises(ValueError):
                Part('bad', mesh, tf)
        with self.assertRaises(ValueError):
            Assembly((Part('x', mesh), Part('x', mesh)))

    def test_ascii_precision_and_stl_units(self):
        text = ('solid test\nfacet normal 0 0 1\nouter loop\nvertex 0.123456789123 0 0\n'
                'vertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid test\n')
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)/'test.stl'
            p.write_text(text)
            mesh = read_mesh(p, length_unit='m')
            self.assertAlmostEqual(mesh.vertices[0,0], .123456789123, places=12)
            with self.assertRaises(ValueError):
                read_mesh(p, length_unit=None)

    def test_legacy_rotation_noncommuting_axes(self):
        r, p, y = .2, .7, -.3
        basis = np.eye(3)
        # Apply three fixed world-axis rotations successively to column vectors.
        def rotate(v, axis, angle):
            axis = np.asarray(axis)
            return v*np.cos(angle) + np.cross(axis, v)*np.sin(angle) + axis*np.dot(axis, v)*(1-np.cos(angle))
        expected = np.column_stack([rotate(rotate(rotate(v, [1,0,0], r), [0,1,0], p), [0,0,1], y) for v in basis])
        np.testing.assert_allclose(rotation_xyz([r,p,y]), expected)

    def test_headless_import_in_child(self):
        script = r'''
import sys, importlib.abc
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'mujoco','wgpu','glfw','rendercanvas','panda3d','OCP','OCC'}:
            raise RuntimeError('Optional import attempted: '+fullname)
sys.meta_path.insert(0, Block())
from wrs.assembly import MeshData, Part, analyze_pair
import wrs.assembly as assembly_api
# Type-only WRS references must not pull rendering or simulation into imports.
for name in assembly_api.__all__:
    getattr(assembly_api, name)
from wrs.assembly.adapters import wrs_scene
from wrs.assembly.primitives import box, pose
result=analyze_pair(Part('a',box()),pose(),Part('b',box()),pose((0,0,.1)))
assert any(p.classification=='active' for p in result.patches)
assert 'wrs.viewer.world' not in sys.modules
'''
        result = subprocess.run([sys.executable, '-c', script], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
