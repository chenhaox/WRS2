"""A surface outline must not turn triangulation seams into apparent line contacts."""

from dataclasses import replace
import unittest
from unittest.mock import patch

import numpy as np

from examples.assembly._shared.contact_display import contact_layers, draw_contacts
from examples.assembly._shared.paper_cases import make_case
from wrs import wssop
from wrs.assembly import Part, analyze_contacts, analyze_pair
from wrs.assembly.geometry.primitives import pose, rectangle, rectangular_ring
from wrs.viewer.world import World


class ContactOutlineTests(unittest.TestCase):
    def draw(self, patches, **options):
        world = World(auto_start_hub=False)
        with patch.object(wssop, "linsegs", wraps=wssop.linsegs) as lines:
            overlays = draw_contacts(world, contact_layers(patches), [], **options)
        segments = [np.asarray(call.args[0]) for call in lines.call_args_list]
        return world, overlays, segments

    def test_soma_faces_have_only_rectangle_outlines(self):
        assembly = make_case("fig08_soma3")
        analysis = analyze_contacts(assembly, assembly.initial_state())
        self.assertEqual([p.dimension for p in analysis.patches], [2] * 6)
        _, _, outlines = self.draw(analysis.patches, offset=(0.3, -0.2, 0.1))
        self.assertEqual(len(outlines), 6)
        for edges in outlines:
            delta = edges[:, 1] - edges[:, 0]
            # All six patches are 18.5 mm squares aligned with the coordinate axes.
            # Long diagonal seams used to survive the viewer's exact edge matching.
            self.assertTrue(np.all(np.count_nonzero(abs(delta) > 1e-9, axis=1) == 1))
            self.assertAlmostEqual(np.linalg.norm(delta, axis=1).sum(), 4 * 0.0185)

    def test_ring_keeps_inner_hole_outline(self):
        analysis = analyze_pair(
            Part("ring", rectangular_ring()), pose(), Part("cover", rectangle(upward=False)), pose()
        )
        self.assertEqual(len(analysis.patches), 1)
        _, _, outlines = self.draw(analysis.patches)
        edges = np.concatenate(outlines)
        # Outer square perimeter .4 m plus the inner square perimeter .16 m.
        self.assertAlmostEqual(np.linalg.norm(edges[:, 1] - edges[:, 0], axis=1).sum(), 0.56)

    def test_missing_or_invalid_loops_do_not_fall_back_to_cell_edges(self):
        analysis = analyze_pair(
            Part("a", rectangle()), pose(), Part("b", rectangle(upward=False)), pose()
        )
        face = analysis.patches[0]
        for invalid in (False, True):
            with self.subTest(invalid=invalid):
                changed = (
                    replace(face, provenance={**face.provenance, "boundary_valid": False})
                    if invalid
                    else replace(
                        face,
                        regions=tuple(
                            replace(region, boundary_loops_world_m=()) for region in face.regions
                        ),
                    )
                )
                _, overlays, outlines = self.draw((changed,))
                self.assertEqual(len(overlays), 2)  # Both sides of the area still render.
                self.assertEqual(outlines, [])

    def test_real_line_and_point_contacts_remain_visible(self):
        for shift, dimension in (((0.1, 0, 0), 1), ((0.1, 0.1, 0), 0)):
            with self.subTest(dimension=dimension):
                analysis = analyze_pair(
                    Part("a", rectangle()), pose(), Part("b", rectangle(upward=False)), pose(shift)
                )
                world, overlays, outlines = self.draw(analysis.patches, draw_boundaries=False)
                self.assertTrue(overlays)
                if dimension == 1:
                    edges = np.concatenate(outlines)
                    self.assertAlmostEqual(
                        np.linalg.norm(edges[:, 1] - edges[:, 0], axis=1).sum(), 0.1
                    )
                else:
                    self.assertEqual(outlines, [])
                controls = world.ui._panels["contacts"]._snapshot()["controls"]
                count = next(control["value"] for control in controls if control["id"] == "count")
                self.assertIn("面 0", count)
                self.assertIn("线 1" if dimension == 1 else "点 1", count)


if __name__ == "__main__":
    unittest.main()
