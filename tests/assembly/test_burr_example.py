"""Separate genuine raw-data overlap from the declared nominal burr fit."""

import unittest

import numpy as np

from examples.assembly._shared.paper import compute
from examples.assembly._shared.paper_cases import make_case
from wrs.assembly import MeshData
from wrs.assembly.geometry.mesh_bvh import QueryBudget
from wrs.assembly.geometry.preprocess import prepare_mesh
from wrs.assembly.geometry.proximity import MeshProximity


class BurrExampleTests(unittest.TestCase):
    def test_raw_overlap_has_an_interior_witness_and_nominal_meshes_are_closed(self):
        raw = make_case("fig10_burr6", nominal=False)
        nominal = make_case("fig10_burr6")
        backend = MeshProximity()
        # A point strictly inside the overlapping left arms, not on a surface.
        witness = np.array([-0.020, -0.00559, -0.025])
        for part in raw.parts:
            if part.part_id in ("2L left", "2u"):
                placed = backend.index(part.geometry).placed(part.assembled_tf)
                self.assertEqual(
                    backend.point_location(placed, witness, QueryBudget(10000)), "inside"
                )
        part = next(p for p in nominal.parts if p.part_id == "2u")
        placed = backend.index(part.geometry).placed(part.assembled_tf)
        self.assertEqual(backend.point_location(placed, witness, QueryBudget(10000)), "outside")
        for part in nominal.parts:
            prepared = prepare_mesh(part.geometry)
            self.assertTrue(prepared.is_closed and prepared.orientation_reliable)
            np.testing.assert_allclose(np.max(abs(prepared.normals), axis=1), 1.0, atol=1e-12)
        self.assertEqual(nominal.provenance["reproduction"], "nominal_repaired")
        corrections = nominal.provenance["corrections"]
        self.assertEqual(len(corrections), 6)
        self.assertTrue(all(c["max_world_vertex_change_m"] <= 0.0003 for c in corrections))
        raw_key = next(p for p in raw.parts if p.part_id == "1")
        nominal_key = next(p for p in nominal.parts if p.part_id == "1")
        self.assertAlmostEqual(
            np.ptp(raw_key.geometry.vertices[:, 2]),
            np.ptp(nominal_key.geometry.vertices[:, 2]),
            places=10,
        )

    def test_nominal_full_assembly_and_prefixes_have_qualified_directions(self):
        expected = {
            "u": "feasible",
            "3u": "feasible",
            "2L right": "feasible",
            "2L left": "blocked",
            "2u": "blocked",
            "1": "feasible",
        }
        for stage in range(1, 7):
            _, _, contacts, results, _ = compute("fig10_burr6", stage)
            self.assertTrue(
                all(
                    d["overlap"]["status"] in ("touching", "separated")
                    for d in contacts.pair_diagnostics
                )
            )
            self.assertTrue(
                all(
                    p.classification in ("active", "near") and p.quality == "bounded"
                    for p in contacts.patches
                )
            )
            for methods in results.values():
                for result in methods.values():
                    self.assertIn(result.status, ("feasible", "blocked"))
                    self.assertFalse(result.issues)
            if stage == 6:
                for key, methods in results.items():
                    self.assertTrue(all(r.status == expected[key] for r in methods.values()))
                # The plain key keeps its longitudinal sliding direction.
                matrix = results["1"]["socp"].constraints.matrix_world[:, :3]
                self.assertTrue(np.all(matrix @ np.array([0.0, 1.0, 0.0]) >= -1e-10))
        _, _, _, duplicate, _ = compute("fig15c_burr6")
        self.assertEqual({k: r["socp"].status for k, r in duplicate.items()}, expected)

    def test_profile_does_not_silently_accept_different_source_geometry(self):
        from tools.assembly.burr_assets import repair_burr_mesh

        raw = make_case("fig10_burr6", nominal=False)
        part = next(p for p in raw.parts if p.part_id == "3u")
        shifted = MeshData(part.geometry.vertices + [0.01, 0, 0], part.geometry.faces)
        with self.assertRaisesRegex(ValueError, "source differs"):
            repair_burr_mesh(shifted, "3u")


if __name__ == "__main__":
    unittest.main()
