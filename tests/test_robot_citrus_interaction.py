"""Headless control/contact regressions for the interactive arm example."""
import unittest
import numpy as np
from wrs import wuc
from examples.agriculture.robot_citrus_interaction import RobotPlantInteraction, run_smoke


class RobotCitrusInteractionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.demo = RobotPlantInteraction()

    def setUp(self):
        self.demo.paused = False
        self.demo.reset()

    def test_targets_limits_rate_pause_and_physics_reset(self):
        d = self.demo
        qpos = d.env.data.qpos.copy()
        command = d.command.copy()
        d.set_joint_degrees(0, np.rad2deg(d.target[0]) + 10)
        np.testing.assert_array_equal(d.env.data.qpos, qpos)
        np.testing.assert_array_equal(d.command, command)
        d.step(.04)
        self.assertLessEqual(abs(d.command[0] - command[0]), np.deg2rad(d.settings['joint_speed_deg_s']) * .04 + 1e-12)
        self.assertGreater(np.linalg.norm(d.env.data.qpos - qpos), 1e-4)
        time, paused_q = d.env.data.time, d.env.data.qpos.copy()
        d.paused = True
        d.step(.5)
        self.assertEqual(d.env.data.time, time)
        np.testing.assert_array_equal(d.env.data.qpos, paused_q)
        d.set_joint_degrees(0, 1e6)
        self.assertEqual(d.target[0], d.limits[1, 0])
        for invalid in ([0], [np.nan] * 6):
            with self.assertRaises(ValueError):
                d.set_target(invalid)
        d.reset()
        np.testing.assert_array_equal(d.env.data.qpos, qpos)
        np.testing.assert_array_equal(d.command, command)
        self.assertEqual(d.report()['simulated_seconds'], 0)
        self.assertEqual(d.contact_samples, {})

    def test_arm_contacts_both_foliage_and_fruit_then_releases(self):
        d = self.demo
        self.assertEqual(d.env.model.nu, 6)
        self.assertEqual(d.env.model.nv, d.robot.ndof + d.plant.mech.ndof)
        self.assertEqual(d.tool.collision_group, wuc.CollisionGroup.ACTIVE)
        result = run_smoke(d)
        for name, role, fruit in [('foliage', 'FOLIAGE', 'orange_001'),
                                   ('fruit', 'FRUIT', d.settings['fruit_id'])]:
            with self.subTest(name=name):
                contact, released = result[name]['touch'], result[name]['released']
                self.assertGreater(contact['contact_samples'].get(role, 0), 0)
                self.assertGreater(contact['fruit_displacement_m'][fruit], .01)
                self.assertGreater(contact['plant_deflection_rad'], .03)
                self.assertEqual(contact['plant_actuators'], 0)
                self.assertEqual(released['contacts'], {})
                self.assertLess(released['fruit_displacement_m'][fruit], .002)
                self.assertLess(released['plant_deflection_rad'], .01)
        self.assertTrue(np.isfinite(d.env.data.qpos).all())
        self.assertTrue(np.isfinite(d.env.data.qvel).all())
        self.assertEqual(sum(int(w.number) for w in d.env.data.warning), 0)


if __name__ == '__main__':
    unittest.main()
