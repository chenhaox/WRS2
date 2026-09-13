"""Guard the public API, WRS scene construction and native paper inputs."""

from importlib import import_module
import subprocess
import sys
import unittest

import numpy as np


class PackageLayoutTests(unittest.TestCase):
    def test_public_api_uses_classified_implementations(self):
        import wrs.assembly as assembly

        modules = {
            "contact_constraints": "motion.constraints",
            "solve_directions": "motion.directions",
            "plan_removal": "motion.part_motion",
            "check_equilibrium": "mechanics.equilibrium",
            "DirectionalStabilityAnalyzer": "mechanics.stability_sweep",
            "plan_sequence": "planning.sequence",
            "score_assemblability": "planning.quality",
            "plan_quality_sequence": "planning.quality_search",
            "validate_execution": "robotics.execution",
            "GraspabilityAnalyzer": "robotics.graspability",
        }
        for name, module in modules.items():
            with self.subTest(name=name):
                self.assertIs(
                    getattr(assembly, name),
                    getattr(import_module(f"wrs.assembly.{module}"), name),
                )

    def test_native_paper_inputs_preserve_all_supported_scenes(self):
        from examples.assembly._shared.paper_cases import CATALOG, make_case
        from tools.assembly.paper_assets import make_case as convert_case

        supported = unavailable = 0
        for key in CATALOG:
            for nominal in (False, True):
                with self.subTest(figure=key, nominal=nominal):
                    try:
                        expected = convert_case(key, nominal=nominal)
                    except FileNotFoundError:
                        with self.assertRaises(FileNotFoundError):
                            make_case(key, nominal=nominal)
                        unavailable += 1
                        continue
                    actual = make_case(key, nominal=nominal)
                    self.assertEqual(len(actual.parts), len(expected.parts))
                    for a, b in zip(actual.parts, expected.parts, strict=True):
                        self.assertEqual(a.part_id, b.part_id)
                        self.assertEqual(a.geometry.geometry_id, b.geometry.geometry_id)
                        np.testing.assert_array_equal(a.assembled_tf, b.assembled_tf)
                        self.assertEqual(
                            (a.fixed, a.mass_kg, a.friction), (b.fixed, b.mass_kg, b.friction)
                        )
                        np.testing.assert_equal(a.com_local_m, b.com_local_m)
                    np.testing.assert_array_equal(
                        actual.gravity_world_m_s2, expected.gravity_world_m_s2
                    )
                    for name, value in expected.provenance.items():
                        self.assertEqual(actual.provenance[name], value)
                    supported += 1
        self.assertEqual((supported, unavailable), (34, 6))

    def test_paper_runtime_does_not_import_conversion_tools(self):
        script = """
import sys, importlib.abc
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] == 'tools' or fullname == 'wrs.assembly.adapters.legacy':
            raise RuntimeError('Runtime attempted old-data conversion: ' + fullname)
sys.meta_path.insert(0, Block())
from examples.assembly._shared.paper_cases import CATALOG, make_case
for key in CATALOG:
    assert make_case(key).parts
assert make_case('fig08_soma3', nominal=False).parts
"""
        result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_wrs_scene_contains_parts_and_contact_overlay(self):
        from wrs.assembly import Assembly, Part, analyze_contacts
        from wrs.assembly.geometry.primitives import box, pose
        from wrs.assembly.visualization import build_wrs_scene

        assembly = Assembly(
            (
                Part("base", box((0.1, 0.1, 0.1)), pose(), fixed=True),
                Part("top", box((0.1, 0.1, 0.1)), pose((0, 0, 0.1))),
            )
        )
        state = assembly.initial_state()
        analysis = analyze_contacts(assembly, state)
        self.assertTrue(analysis.patches)
        scene = build_wrs_scene(assembly, state, analysis)
        self.assertGreater(len(tuple(scene)), len(assembly.parts))


if __name__ == "__main__":
    unittest.main()
