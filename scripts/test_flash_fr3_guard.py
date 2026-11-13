#!/usr/bin/env python3
"""Fault injection without ROS, network, simulation or robot commands."""
import copy
import threading
import unittest

import numpy as np

from flash_fr3_guard import GuardLimits, ShadowGuard, joint_state_vectors, preview
from flash_fr3_reference import DelayedJointReference
from flash_fr3_shadow_math import evaluate_curve, validate_reply


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.q = np.array([-.16, -.78, .11, -2.38, .07, 1.56, .66])
        self.coeff = np.zeros((7, 8))
        self.coeff[0, :7] = self.q

    def state(self, now):
        return {"q": self.q.copy(), "dq": np.zeros(7), "receipt_monotonic": now - .002,
                "source_age_at_receipt_seconds": .003}

    def reply(self, request_id=0, offset=0):
        coeff = self.coeff.copy()
        coeff[0, 1] += offset
        curve = {"basis": "legendre", "coefficients": coeff.tolist(), "anchor_s": 2 / 11,
                 "polynomial_duration_seconds": 44 / 15, "valid_future_seconds": 32 / 15}
        return {"request_id": request_id, "curve": curve}

    def start(self, offset=0):
        guard = ShadowGuard()
        guard.tick(1., self.state(1.))
        self.assertTrue(guard.offer(self.reply(offset=offset), .8, 1., self.state(1.)))
        return guard

    def test_reorder_measured_positions_and_velocities_and_reject_missing_values(self):
        order = [1, 3, 6, 7, 2, 4, 5]
        names = [f"fr3_joint{i}" for i in order]
        q, dq = joint_state_vectors(names, order, [-i for i in order])
        np.testing.assert_array_equal(q, np.arange(1, 8))
        np.testing.assert_array_equal(dq, -np.arange(1, 8))
        for bad in ([], [np.nan] * 7):
            with self.assertRaises(ValueError):
                joint_state_vectors(names, order, bad)

    def test_entire_candidate_is_rejected_without_clipping_or_automatic_restart(self):
        guard = ShadowGuard()
        guard.tick(1., self.state(1.))
        self.assertFalse(guard.offer(self.reply(offset=.20), .8, 1., self.state(1.)))
        self.assertEqual(guard.fault["code"], "TRAJECTORY_LIMIT")
        self.assertIn("local_excursion_rad", guard.last_preview["violations"])
        self.assertIn("velocity_rad_s", guard.last_preview["violations"])
        self.assertIsNone(guard.reference)
        self.assertFalse(guard.offer(self.reply(1), 1., 1.1, self.state(1.1)))
        self.assertIsNone(guard.tick(1.1, self.state(1.1)))

    def test_timeout_checks_keep_running_during_blocked_inference_worker(self):
        release, entered = threading.Event(), threading.Event()

        def stalled_io():
            entered.set()
            release.wait(2)

        worker = threading.Thread(target=stalled_io)
        worker.start()
        try:
            self.assertTrue(entered.wait(.5))
            guard = self.start()
            # Virtual monotonic time exercises the same owner-thread tick as
            # live mode, while the separate inference worker remains blocked.
            for now in np.arange(1.01, 1.72, .01):
                guard.tick(float(now), self.state(float(now)))
            self.assertTrue(worker.is_alive())
            self.assertEqual(guard.fault["code"], "PREDICTION_TIMEOUT")
        finally:
            release.set()
            worker.join(1)

    def test_stale_state_and_old_source_new_receipt_both_block(self):
        for receipt, source in ((.80, .003), (.999, .2), (.95, .06)):
            guard = self.start()
            state = self.state(1.01)
            state.update(receipt_monotonic=receipt, source_age_at_receipt_seconds=source)
            self.assertIsNone(guard.tick(1.01, state))
            self.assertEqual(guard.fault["code"], "STATE_OR_REFERENCE_INVALID")

    def test_late_callback_and_out_of_order_prediction_are_rejected(self):
        guard = self.start()
        guard.tick(1.101, self.state(1.101))
        self.assertEqual(guard.fault["code"], "LOOP_DEADLINE")
        for request_id, observation in ((0, .9), (1, .8)):
            guard = self.start()
            guard.tick(1.01, self.state(1.01))
            self.assertFalse(guard.offer(self.reply(request_id), observation, 1.01, self.state(1.01)))
            self.assertEqual(guard.fault["code"], "PREDICTION_INVALID")
        guard = ShadowGuard()
        guard.tick(1., self.state(1.))
        self.assertFalse(guard.offer(self.reply(), .29, 1., self.state(1.)))
        self.assertEqual(guard.fault["code"], "PREDICTION_INVALID")

    def test_tracking_error_is_not_disabled_for_readonly_shadow(self):
        guard = self.start(offset=.06)
        for now in np.arange(1.01, 1.69, .01):
            guard.tick(float(now), self.state(float(now)))
        self.assertEqual(guard.fault["code"], "TRACKING_ERROR")

    def test_moving_initial_state_cannot_assume_zero_acceleration(self):
        guard, state = ShadowGuard(), self.state(1.)
        state["dq"][0] = .02
        guard.tick(1., state)
        self.assertFalse(guard.offer(self.reply(), .8, 1., state))
        self.assertEqual(guard.fault["code"], "PREDICTION_INVALID")

    def test_vectorized_preview_matches_scalar_physical_derivatives(self):
        rng = np.random.default_rng(42)
        curve = self.reply()["curve"]
        coefficients = self.coeff + rng.normal(0, .015, (7, 8))
        curve["coefficients"] = coefficients.tolist()
        initial = np.stack([self.q, np.zeros(7), np.zeros(7)])
        reference = DelayedJointReference(curve, .37, initial)
        limits = GuardLimits(preview_dt_seconds=.01)
        result = preview(reference, self.q, limits)
        times = np.unique(np.r_[np.arange(0, reference.remaining_seconds, .01),
                                reference.remaining_seconds, reference.blend])
        scalar = np.asarray([reference.evaluate_with_jerk(t) for t in times])
        for order, key in enumerate(("velocity_max_rad_s", "acceleration_max_rad_s2", "jerk_max_rad_s3"), 1):
            np.testing.assert_allclose(result["per_joint_" + key], np.abs(scalar[:, order]).max(axis=0), atol=1e-12)
        np.testing.assert_allclose(result["per_joint_excursion_max_rad"],
                                   np.abs(scalar[:, 0] - self.q).max(axis=0), atol=1e-12)

    def test_nonfinite_response_timestamp_does_not_bypass_validation(self):
        reply = self.reply()
        reply.update(protocol="flash_fr3_shadow_v1", status="SHADOW_PREDICTION", weights="ema_model",
                     robot_motion_commanded=False, checkpoint_step=15000,
                     observation_target_stamp_seconds=42.,
                     action=[evaluate_curve(reply["curve"], i * 4 / 15)[0].tolist() for i in range(8)])
        validate_reply(reply, 0, 42., 15000)
        for field, value in (("observation_target_stamp_seconds", np.nan), ("checkpoint_step", 1)):
            invalid = copy.deepcopy(reply)
            invalid[field] = value
            with self.assertRaises(ValueError):
                validate_reply(invalid, 0, 42., 15000)
        invalid = copy.deepcopy(reply)
        invalid["curve"]["polynomial_duration_seconds"] *= 2
        with self.assertRaises(ValueError):
            validate_reply(invalid, 0, 42., 15000)
        invalid = copy.deepcopy(reply)
        invalid["action"][7][0] += .1
        with self.assertRaises(ValueError):
            validate_reply(invalid, 0, 42., 15000)


if __name__ == "__main__":
    unittest.main()
