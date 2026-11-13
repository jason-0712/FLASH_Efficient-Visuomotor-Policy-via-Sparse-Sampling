#!/usr/bin/env python3
"""Check physical derivatives, C2 handoffs and rejection without robot IO."""
import copy
import unittest

import numpy as np

from flash_fr3_reference import DelayedJointReference
from flash_fr3_shadow_math import evaluate_curve


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(42)
        self.curve = {
            "basis": "legendre", "coefficients": rng.normal(0, .04, (7, 8)).tolist(),
            "anchor_s": 2 / 11, "polynomial_duration_seconds": 44 / 15,
            "valid_future_seconds": 32 / 15,
        }
        self.initial = rng.normal(0, .1, (3, 7))

    def test_initial_and_blend_end_match_position_velocity_acceleration(self):
        ref = DelayedJointReference(self.curve, .4, self.initial)
        np.testing.assert_allclose(ref.evaluate(0), self.initial, atol=1e-14)
        expected = np.asarray(evaluate_curve(self.curve, .4 + 1.2))[:, :7]
        np.testing.assert_allclose(ref.evaluate(1.2), expected, atol=1e-14)
        # Check the left limit independently of the exact endpoint branch.
        np.testing.assert_allclose(ref.evaluate(1.2 - 1e-8), expected, atol=1e-6)
        raw = np.asarray(evaluate_curve(self.curve, .4 + 1.3))[:, :7]
        np.testing.assert_allclose(ref.evaluate(1.3), raw, atol=1e-14)

    def test_replanning_preserves_all_three_boundary_values(self):
        previous = DelayedJointReference(self.curve, .4, self.initial)
        boundary = previous.evaluate(.27)
        changed = copy.deepcopy(self.curve)
        changed["coefficients"][0][1] += .2
        following = DelayedJointReference(changed, .5, boundary)
        np.testing.assert_allclose(following.evaluate(0), boundary, atol=1e-14)

    def test_derivatives_have_physical_time_units(self):
        ref = DelayedJointReference(self.curve, .4, self.initial)
        h = 1e-5
        for t in (.15, .6, 1.3):
            values = ref.evaluate_with_jerk(t)
            plus, minus = ref.evaluate(t + h), ref.evaluate(t - h)
            for order in (1, 2, 3):
                difference = (plus[order - 1] - minus[order - 1]) / (2 * h)
                np.testing.assert_allclose(values[order], difference, atol=2e-7, rtol=2e-6)

    def test_reject_stale_prediction_horizon_overrun_and_invalid_states(self):
        for age in (-.1, .71, np.nan):
            with self.assertRaises(ValueError):
                DelayedJointReference(self.curve, age, self.initial)
        for blend in (0, -1, 1.8, np.inf):
            with self.assertRaises(ValueError):
                DelayedJointReference(self.curve, .4, self.initial, blend_seconds=blend)
        for state in (np.zeros((3, 8)), np.full((3, 7), np.nan)):
            with self.assertRaises(ValueError):
                DelayedJointReference(self.curve, .4, state)
        ref = DelayedJointReference(self.curve, .4, self.initial)
        for t in (-.01, ref.remaining_seconds + .01, np.nan):
            with self.assertRaises(ValueError):
                ref.evaluate(t)


if __name__ == "__main__":
    unittest.main()
