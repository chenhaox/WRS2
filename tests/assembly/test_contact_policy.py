"""Contact tolerances change assembly policy, never the geometric evidence."""
import unittest
from dataclasses import replace
import numpy as np
from wrs.assembly import (ContactAnalysis, ContactPatch, Region, ToleranceContactPolicy,
                          ContactAnalyzer, ContactConfig, ContactModel)
from wrs.assembly.primitives import box, pose


def report(label='near', unsigned=False, band=.0005, gap=(.0002, .0005)):
    p = ContactPatch('a', 'b', 'a0', 'b0', 2, label, 'estimated',
                     np.array([[0, 0, 0]]), np.array([[0, 0, .0002]]),
                     np.array([[0, 0, 1]]), np.array([[0, 0, -1]]),
                     (Region((np.array([[0., 0, 0], [.01, 0, 0], [0, .01, 0]]),)),),
                     gap, area_m2=.00005, weights=np.array([.00005]), measure_kind='near_band',
                     provenance={'band_limit_m': band, 'distance_interval': 'unsigned' if unsigned else 'signed_sdf'})
    return ContactAnalysis((p,), ({'part_a': 'a', 'part_b': 'b', 'overlap': {'status': 'unknown'}},), 'raw')


class ContactPolicyTests(unittest.TestCase):
    def test_unsigned_requires_opt_in_without_overwriting_geometry(self):
        raw = report('unknown', unsigned=True)
        strict = ToleranceContactPolicy(.0005).apply(raw)
        allowed = ToleranceContactPolicy(.0005, allow_unsigned=True).apply(raw)
        self.assertIsNone(strict.patches[0].provenance['contact_policy']['is_contact'])
        self.assertTrue(allowed.patches[0].provenance['contact_policy']['is_contact'])
        self.assertEqual(allowed.patches[0].classification, 'unknown')
        self.assertEqual(allowed.pair_diagnostics[0]['overlap']['status'], 'unknown')
        self.assertNotIn('contact_policy', raw.patches[0].provenance)
        self.assertNotEqual(strict.state_digest, allowed.state_digest)
        self.assertEqual(allowed.pair_diagnostics[0]['contact_policy']['sides']['b']['accepted_area_m2'], 0)

    def test_tolerance_does_not_select_whole_wider_patch_or_negative_regions(self):
        self.assertTrue(ToleranceContactPolicy(.0005).apply(report()).patches[0].provenance['contact_policy']['is_contact'])
        smaller = ToleranceContactPolicy(.0003).apply(report()).patches[0].provenance['contact_policy']
        self.assertIsNone(smaller['is_contact'])
        self.assertEqual(smaller['reason'], 'reextract_band_at_contact_tolerance')
        far = ToleranceContactPolicy(.0001).apply(report()).patches[0]
        self.assertFalse(far.provenance['contact_policy']['is_contact'])
        negative = report('interference', gap=(-.0003, -.0002))
        self.assertFalse(ToleranceContactPolicy(.001, True).apply(negative).patches[0].provenance['contact_policy']['is_contact'])
        crossing = report('unknown', gap=(-.0003, .0002))
        self.assertIsNone(ToleranceContactPolicy(.001, True).apply(crossing).patches[0].provenance['contact_policy']['is_contact'])

    def test_mesh_backend_near_is_accepted_without_changing_active(self):
        a = ContactModel(box(), 'a')
        b = ContactModel(box(), 'b', pose((0, 0, .1002)))
        raw = ContactAnalyzer('mesh', config=ContactConfig(near_tol_m=.0005)).analyze_pair(a, b)
        applied = ToleranceContactPolicy(.0005).apply(raw)
        self.assertTrue(any(p.provenance['contact_policy']['is_contact'] is True for p in applied.patches))
        self.assertFalse(any(p.classification == 'active' for p in applied.patches))
        for value in (0, -1, np.nan, np.inf):
            with self.assertRaises(ValueError):
                ToleranceContactPolicy(value)

