"""Guard import compatibility, portable previews and native paper inputs."""

from importlib import import_module
from pathlib import Path
import subprocess
import sys
import tempfile
import tomllib
import unittest

import numpy as np


class PackageLayoutTests(unittest.TestCase):
    def test_previous_imports_share_modules_and_caches(self):
        modules = {
            "constraints": "motion.constraints",
            "directions": "motion.directions",
            "part_motion": "motion.part_motion",
            "stability": "mechanics.equilibrium",
            "stability_sweep": "mechanics.stability_sweep",
            "force_points": "mechanics.force_points",
            "sequence": "planning.sequence",
            "quality": "planning.quality",
            "quality_search": "planning.quality_search",
            "execution": "robotics.execution",
            "graspability": "robotics.graspability",
            "primitives": "geometry.primitives",
        }
        for old, new in modules.items():
            with self.subTest(module=old):
                self.assertIs(
                    import_module(f"wrs.assembly.{old}"),
                    import_module(f"wrs.assembly.{new}"),
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

    def test_preview_template_is_declared_and_loads_from_new_package(self):
        from wrs.assembly.visualization import write_contact_html

        root = Path(__file__).resolve().parents[2]
        config = tomllib.loads((root / "pyproject.toml").read_text(encoding="utf-8"))
        resources = config["tool"]["setuptools"]["package-data"]
        self.assertIn("_contact_viewer.html", resources["wrs.assembly.visualization"])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contacts.html"
            write_contact_html([{"name": "fixture <tag>", "meshes": [], "analysis": {}}], path)
            html = path.read_text(encoding="utf-8")
        self.assertIn("fixture \\u003ctag>", html)
        self.assertNotIn("__ASSEMBLY_DATA__", html)


if __name__ == "__main__":
    unittest.main()
