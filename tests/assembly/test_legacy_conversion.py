"""One-time old-data conversion lives in tools, outside the runtime library."""

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from tools.assembly.legacy_data import legacy_asset_inventory, load_legacy_assembly


STL = """solid domino
facet normal 0 0 1
outer loop
vertex 0 0 0
vertex 100 0 0
vertex 0 100 0
endloop
endfacet
endsolid domino
"""


class LegacyAdapterTests(unittest.TestCase):
    def test_instance_units_and_shared_stl_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "asp/data").mkdir(parents=True)
            (root / "asp/objects").mkdir(parents=True)
            (root / "asp/objects/domino.stl").write_text(STL, encoding="utf-8")
            data = {
                "Domino_A": {"location": [10, 20, 30], "rotation": [0, 0, 0]},
                "Domino_B": {"location": [40, 50, 60], "rotation": [0, 0, 0]},
            }
            (root / "asp/data/scene").write_text(json.dumps(data), encoding="utf-8")
            report = legacy_asset_inventory(root, "scene", length_unit="mm")
            self.assertEqual(report["part_count"], 2)
            self.assertTrue(all(part["exists"] for part in report["parts"]))
            assembly = load_legacy_assembly(root, "scene", length_unit="mm")
            first, second = assembly.parts
            self.assertIs(first.geometry, second.geometry)
            np.testing.assert_allclose(first.assembled_tf[:3, 3], [0.01, 0.02, 0.03])
            np.testing.assert_allclose(second.assembled_tf[:3, 3], [0.04, 0.05, 0.06])
            np.testing.assert_allclose(np.ptp(first.geometry.vertices, axis=0), [0.1, 0.1, 0])
            self.assertIsNone(first.mass_kg)
            self.assertIsNone(first.friction)
            self.assertIsNone(first.com_local_m)

    def test_missing_model_is_reported_and_cannot_be_silently_dropped(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "asp/data").mkdir(parents=True)
            (root / "asp/data/scene").write_text(
                json.dumps({"missing": {"location": [0, 0, 0], "rotation": [0, 0, 0]}}),
                encoding="utf-8",
            )
            report = legacy_asset_inventory(root, "scene", length_unit="m")
            self.assertFalse(report["parts"][0]["exists"])
            with self.assertRaisesRegex(FileNotFoundError, "missing.stl"):
                load_legacy_assembly(root, "scene", length_unit="m")


if __name__ == "__main__":
    unittest.main()
