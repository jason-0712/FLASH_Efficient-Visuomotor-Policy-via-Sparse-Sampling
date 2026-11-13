"""Analytic and envelope checks for the fixed commissioning probe; no ROS."""
import unittest

import numpy as np

from flash_fr3_model_plan import evaluate_segment
from flash_fr3_torque_probe_plan import probe_plan


class ProbePlanTest(unittest.TestCase):
    def setUp(self):
        self.q = np.array([-.16, -.77, .11, -2.38, .07, 1.57, .66])

    def test_physical_derivative_and_static_boundaries(self):
        plan = probe_plan(self.q)
        coefficients = np.asarray(plan['coefficients'])
        for t in np.linspace(0, 3., 31):
            u = t/3.
            q = .001*(10*u**3-15*u**4+6*u**5)
            dq = .001/3.*30*u**2*(1-u)**2
            value = evaluate_segment(coefficients[0], 3., t)
            velocity = evaluate_segment(coefficients[0], 3., t, 1)
            self.assertAlmostEqual(value[3], q, places=14)
            self.assertAlmostEqual(velocity[3], dq, places=14)
            np.testing.assert_array_equal(np.delete(value, 3), 0.)
        for order in (0, 1, 2):
            left = evaluate_segment(coefficients[0], 3., 3., order)
            right = evaluate_segment(coefficients[1], .75, 0., order)
            np.testing.assert_allclose(left, right, atol=1e-13)
        self.assertEqual(sum(plan['segment_durations']), 3.75)

    def test_entire_curve_and_watchdog_brake_stay_in_torque_envelope(self):
        bounds = probe_plan(self.q)['bounds']
        self.assertTrue(bounds['pass'])
        limits = np.array([.002, .005, .02, .2])[:, None]
        for key in ('nominal_bounds', 'watchdog_brake_bounds'):
            self.assertTrue((np.asarray(bounds[key]) <= limits).all(), bounds)

    def test_bad_start_and_workspace_edge_are_rejected(self):
        for q in (self.q[:6], np.full(7, np.nan), self.q+np.array([0, 0, 0, .56, 0, 0, 0])):
            with self.assertRaises(ValueError):
                probe_plan(q)


if __name__ == '__main__':
    unittest.main()
