"""Assembly checkboxes must change the scene and solver options, not just labels."""

import unittest
from unittest.mock import patch

from wrs import wssop
from wrs.viewer.world import World

from examples.assembly._shared.contact_display import draw_contacts
from examples.assembly._shared import stability
from examples.assembly._shared.stability_display import draw_stability


class AssemblyControlTests(unittest.TestCase):
    def setUp(self):
        self.world = World(auto_start_hub=False)
        self.event_number = 0

    def send(self, panel, control_id, value=None):
        self.event_number += 1
        state = panel._snapshot()
        result = self.world.ui._handle_event(
            dict(
                type="ui_event",
                panel_id=state["id"],
                session=state["session"],
                event_id=str(self.event_number),
                id=control_id,
                value=value,
            )
        )
        self.assertTrue(result["ok"], result)

    def controls(self, panel):
        return {control["id"]: control for control in panel._snapshot()["controls"]}

    def test_contact_checkbox_and_live_opacity_update_scene(self):
        model = wssop.box()
        self.world.scene.add(model)
        layers = [
            dict(
                name="fixture",
                kind="active",
                dimension=2,
                cells=[[(0, 0, 0), (0.1, 0, 0), (0, 0.1, 0)]],
                points=[],
            )
        ]
        overlays = draw_contacts(self.world, layers, [model])
        panel = self.world.ui._panels["contacts"]
        controls = self.controls(panel)
        self.assertEqual(controls["show"]["kind"], "checkbox")
        self.assertTrue(controls["opacity"]["continuous"])
        for visible in (False, True, False, True):
            self.send(panel, "show", visible)
            self.assertEqual(len(tuple(self.world.scene)), 1 + visible * len(overlays))
        self.send(panel, "opacity", 0.65)
        self.assertAlmostEqual(model.alpha, 0.65, places=6)

    def test_force_layer_checkboxes_and_restore_stay_in_sync(self):
        assembly, state, _, config, result = stability.solve_case("stack")
        panel = draw_stability(self.world, assembly, state, result, config)
        original_count = len(tuple(self.world.scene))
        for key in ("points", "cones", "forces"):
            with self.subTest(layer=key):
                self.assertEqual(self.controls(panel)[key]["kind"], "checkbox")
                self.send(panel, key, False)
                self.assertLess(len(tuple(self.world.scene)), original_count)
                self.send(panel, "restore")
                self.assertEqual(len(tuple(self.world.scene)), original_count)
                for name in ("points", "cones", "forces"):
                    self.assertIs(self.controls(panel)[name]["value"], True)

    def test_stability_options_pass_booleans_and_recompute_supports(self):
        with (
            patch("wrs.viewer.world.World", return_value=self.world),
            patch.object(self.world, "run"),
            patch.object(stability, "solve_case", wraps=stability.solve_case) as solve,
        ):
            stability.show("floating", aux_supports=False)
            panel = self.world.ui._panels["scenario"]
            self.assertEqual(self.controls(panel)["reduce"]["kind"], "checkbox")
            self.assertEqual(self.controls(panel)["supports"]["kind"], "checkbox")
            self.send(panel, "reduce", False)
            self.assertIs(solve.call_args.kwargs["reduce_contact_points"], False)
            self.assertIs(solve.call_args.kwargs["with_supports"], False)
            self.send(panel, "supports", True)
            self.assertIs(solve.call_args.kwargs["with_supports"], True)
            forces = self.controls(self.world.ui._panels["stability"])
            self.assertIn("auxiliary", forces)
            self.assertTrue(forces["status"]["value"].startswith("feasible"))
            self.send(panel, "supports", False)
            self.assertNotIn("auxiliary", self.controls(self.world.ui._panels["stability"]))


if __name__ == "__main__":
    unittest.main()
