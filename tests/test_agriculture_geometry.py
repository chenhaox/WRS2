"""Engine-independent contracts: run with stdlib unittest and NumPy."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import unittest
import numpy as np

from agriculture import PlantSkeleton, StemSegment, PlantSpec
from agriculture.config import load_config, GENERIC_CONFIG
from agriculture.generator import generate
from agriculture.morphology import apply_pipe_model
from agriculture.foliage import place_leaves
from agriculture.geometry import leaf_mesh
from agriculture.dynamics import PlantDynamicsSpec


def skeleton():
    return PlantSkeleton([
        StemSegment('root', None, (0, 0, 0), (0, 0, 1), .03, .02, 0),
        StemSegment('left', 'root', (0, 0, 1), (-.3, 0, 1.2), .01, .005, 1),
        StemSegment('right', 'root', (0, 0, 1), (.3, 0, 1.2), .01, .005, 1),
        StemSegment('tip', 'right', (.3, 0, 1.2), (.4, .1, 1.3), .003, .001, 2)])


class PlantGeometryTests(unittest.TestCase):
    def test_topology_queries_frames_and_bounds(self):
        s = skeleton().validate()
        self.assertEqual(s.roots, ['root'])
        self.assertEqual(s.terminals, ['left', 'tip'])
        self.assertEqual(s.descendants('right'), ['tip'])
        self.assertEqual(s.descendants('right', include_self=True), ['right', 'tip'])
        self.assertEqual(s.depth('tip'), 2)
        self.assertAlmostEqual(s.total_length(), 1 + 2 * np.hypot(.3, .2) + np.sqrt(.03))
        lo, hi = s.bounds()
        self.assertAlmostEqual(s.height(), hi[2] - lo[2])
        for stem in s.segments:
            np.testing.assert_allclose(stem.frame[:, 2] * stem.length, np.subtract(stem.end, stem.start), atol=1e-7)
            np.testing.assert_allclose(stem.frame.T @ stem.frame, np.eye(3), atol=1e-6)
        s.segments.reverse()
        s.validate()
        self.assertEqual(s.depth('tip'), 2)

    def test_invalid_skeleton_and_authored_frame(self):
        for key, value in [('end', (0, 0, 0)), ('radius_start', -1), ('parent_id', 'missing'),
                           ('rotmat', np.eye(3)), ('rotmat', np.zeros((3, 3)))]:
            s = skeleton()
            target = s.segments[1] if key == 'rotmat' else s.segments[0]
            setattr(target, key, value)
            with self.subTest(key=key), self.assertRaises(ValueError):
                s.validate()

    def test_multiple_root_shoots_are_supported(self):
        s = skeleton()
        s.segments.append(StemSegment('basal_shoot', None, (.03, 0, 0), (.1, 0, .5), .004, .002, 0))
        plant = PlantSpec('shrub', s).validate()
        self.assertEqual(len(plant.skeleton.roots), 2)
        self.assertEqual(PlantSpec.from_json(plant.to_json()).skeleton.roots, plant.skeleton.roots)

    def test_optional_pipe_model_preserves_geometry_and_topology(self):
        s = skeleton()
        radii = [p.radius_start for p in s.segments]
        out = apply_pipe_model(s, .002, beta=2)
        self.assertEqual(radii, [p.radius_start for p in s.segments])
        for stem in out.segments:
            self.assertEqual(stem.start, s.by_id[stem.id].start)
            if out.children[stem.id]:
                self.assertAlmostEqual(stem.radius_start ** 2,
                                      sum(out.by_id[key].radius_start ** 2 for key in out.children[stem.id]))

    def test_generators_serialization_and_seed_variation(self):
        for config in (load_config(), load_config(GENERIC_CONFIG)):
            untouched = deepcopy(config)
            a, b = generate(config, 12), generate(config, 12)
            self.assertEqual(a.to_json(), b.to_json())
            self.assertEqual(a.to_json(), PlantSpec.from_json(a.to_json()).to_json())
            self.assertEqual(config, untouched)
            self.assertEqual([f.id for f in a.fruits], [f.id for f in generate(config, 13).fruits])
            self.assertTrue(all(s.radius_end > 0 for s in a.branches))
        a, b = generate(load_config(GENERIC_CONFIG), 12), generate(load_config(GENERIC_CONFIG), 13)
        self.assertNotEqual([s.end for s in a.branches], [s.end for s in b.branches])
        self.assertNotEqual([f.position for f in a.fruits], [f.position for f in b.fruits])

    def test_foliage_standalone_density_winding_and_parents(self):
        config = load_config(GENERIC_CONFIG)
        plant = generate(config)
        parameters = config['leaf_generation']
        a = place_leaves(plant.skeleton, parameters, plant.leaf_shape, seed=99)
        b = place_leaves(plant.skeleton, parameters, plant.leaf_shape, seed=99)
        self.assertEqual(a, b)
        self.assertEqual(place_leaves(plant.skeleton, parameters | {'density': 0}, plant.leaf_shape), [])
        for leaf in a:
            self.assertIn(leaf.parent_segment, plant.skeleton.terminals)
        vs, fs = leaf_mesh(a[0].length, a[0].width, plant.leaf_shape)
        tri = vs[fs]
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        np.testing.assert_allclose(n[:len(n)//2], -n[len(n)//2:], atol=1e-12)

    def test_dynamics_partition_and_roundtrip(self):
        c = load_config()
        plant = generate(c)
        d = PlantDynamicsSpec.from_config(plant, c)
        self.assertEqual(d.to_json(), PlantDynamicsSpec.from_json(d.to_json()).validate(plant).to_json())
        self.assertEqual(len(d.clusters), 8)
        self.assertEqual(sum(c.dof for c in d.clusters), 11)
        for key in plant.skeleton.roots:
            self.assertIsNone(d.segment_clusters(plant)[key])
        for mutation in ('missing', 'overlap', 'static_descendant', 'parent', 'dof'):
            bad = deepcopy(d)
            if mutation == 'missing': bad.clusters[0].member_segments.append('missing')
            elif mutation == 'overlap': bad.clusters[1].member_segments.append(bad.clusters[0].root_segment)
            elif mutation == 'static_descendant': bad.clusters[1].member_segments.pop()
            elif mutation == 'parent': bad.clusters[1].parent_cluster = bad.clusters[0].id
            else: bad.clusters[0].dof = 3
            with self.subTest(mutation=mutation), self.assertRaises(ValueError): bad.validate(plant)

    def test_pure_generation_and_export_block_engine_imports(self):
        code = """
import importlib.abc, sys
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('wrs','mujoco','newton','warp'):
            raise AssertionError(fullname)
sys.meta_path.insert(0, Guard())
from agriculture.config import load_config, GENERIC_CONFIG
from agriculture.generator import generate
from agriculture.spec import PlantSpec
from agriculture.dynamics import PlantDynamicsSpec
for c in (load_config(), load_config(GENERIC_CONFIG)):
    p = PlantSpec.from_json(generate(c).to_json()).scaled(.8)
    p.summary()
    if 'dynamics' in c:
        PlantDynamicsSpec.from_config(p, c).to_json()
"""
        result = subprocess.run([sys.executable, '-c', code], cwd=Path(__file__).resolve().parents[1], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__': unittest.main()
