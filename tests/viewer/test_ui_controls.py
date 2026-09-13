"""UI merge regressions: boolean events, rollback and slider compatibility."""

import unittest

from wrs.viewer.web_ui import UIManager


class UIControlTests(unittest.TestCase):
    def setUp(self):
        self.ui = UIManager()
        self.panel = self.ui.add_panel("controls")
        self.events = 0

    def event(self, control_id, value):
        self.events += 1
        return dict(
            type="ui_event",
            panel_id="controls",
            session=self.panel._snapshot()["session"],
            event_id=str(self.events),
            id=control_id,
            value=value,
        )

    def value(self, control_id):
        return next(c["value"] for c in self.panel._snapshot()["controls"] if c["id"] == control_id)

    def test_checkbox_routes_both_booleans_and_deduplicates_events(self):
        received = []
        self.panel.add_checkbox("axes", on_change=received.append)
        for value in (True, False):
            event = self.event("axes", value)
            result = self.ui._handle_event(event)
            self.assertTrue(result["ok"])
            self.assertIs(self.value("axes"), value)
            self.assertEqual(self.ui._handle_event(event), result)
        self.assertEqual(received, [True, False])
        self.panel.set_value("axes", True)
        self.assertEqual(received, [True, False])

    def test_checkbox_rejects_non_booleans_without_changing_state(self):
        self.panel.add_checkbox("axes", value=True)
        for value in (0, 1, "false", None):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    self.panel.add_checkbox("invalid", value=value)
                with self.assertRaises(ValueError):
                    self.panel.set_value("axes", value)
                with self.assertLogs("wrs.viewer.web_ui.panel", level="WARNING"):
                    result = self.ui._handle_event(self.event("axes", value))
                self.assertFalse(result["ok"])
                self.assertIs(self.value("axes"), True)

    def test_failed_checkbox_callback_restores_false(self):
        def fail(value):
            raise RuntimeError("scene update failed")

        self.panel.add_checkbox("axes", on_change=fail)
        with self.assertLogs("wrs.viewer.web_ui.panel", level="WARNING"):
            result = self.ui._handle_event(self.event("axes", True))
        self.assertFalse(result["ok"])
        self.assertIs(self.value("axes"), False)

    def test_disabled_and_removed_controls_do_not_invoke_callbacks(self):
        received = []
        self.panel.add_checkbox("axes", on_change=received.append, enabled=False)
        with self.assertLogs("wrs.viewer.web_ui.panel", level="WARNING"):
            self.assertFalse(self.ui._handle_event(self.event("axes", True))["ok"])
            self.panel.remove("axes")
            self.assertFalse(self.ui._handle_event(self.event("axes", True))["ok"])
        self.assertEqual(received, [])

    def test_old_and_live_sliders_keep_snapping_and_callback_contract(self):
        received = []
        self.panel.add_slider("old", step=0.1, on_change=received.append)
        self.panel.add_slider(
            "live", step=0.1, continuous=True, update_hz=20, on_change=received.append
        )
        old, live = self.panel._snapshot()["controls"]
        self.assertEqual((old["continuous"], old["update_hz"]), (False, 30))
        self.assertEqual((live["continuous"], live["update_hz"]), (True, 20))
        for control_id in ("old", "live"):
            result = self.ui._handle_event(self.event(control_id, 0.26))
            self.assertTrue(result["ok"])
            self.assertEqual(self.value(control_id), 0.3)
        self.assertEqual(received, [0.3, 0.3])

    def test_invalid_continuous_slider_options(self):
        for hz in (0, -1, float("nan"), float("inf"), True, "30"):
            with self.subTest(update_hz=hz), self.assertRaises(ValueError):
                self.panel.add_slider("invalid", update_hz=hz)
        for continuous in (0, 1, "true", None):
            with self.subTest(continuous=continuous), self.assertRaises(ValueError):
                self.panel.add_slider("invalid", continuous=continuous)


if __name__ == "__main__":
    unittest.main()
