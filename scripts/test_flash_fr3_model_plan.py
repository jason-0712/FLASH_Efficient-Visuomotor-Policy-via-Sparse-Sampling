#!/usr/bin/env python3
"""Independent polynomial conversion, C2 and native preview checks; no robot IO."""
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np

from flash_fr3_model_plan import make_segments, evaluate_segment, write_preview_csv


class ModelPlanTests(unittest.TestCase):
    def setUp(self):
        self.curve = {"basis":"legendre","coefficients":np.random.default_rng(42).normal(0,.002,(7,8)).tolist(),
                      "anchor_s":2/11,"polynomial_duration_seconds":44/15,"valid_future_seconds":32/15}
        self.q = np.array([-.16,-.78,.11,-2.38,.07,1.56,.66])
        self.curve["coefficients"][0][:7] = self.q.tolist()

    def test_conversion_and_native_preview_agree_and_stop(self):
        p = make_segments(self.curve,.4,self.q)
        c = np.array(p["coefficients"]); d = p["segment_durations"]
        for k in range(3):
            np.testing.assert_allclose(evaluate_segment(c[0],d[0],d[0],k),
                                       evaluate_segment(c[1],d[1],0,k),atol=1e-12)
        for k in (1,2):
            np.testing.assert_allclose(evaluate_segment(c[1],d[1],d[1],k),0,atol=1e-12)
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/"input.txt").write_text(" ".join(str(v) for v in d+c.ravel().tolist()+self.q.tolist()))
            binary="/home/ros/ros2_ws/install/flash_fr3_model_controller/lib/flash_fr3_model_controller/preview_model"
            r=subprocess.run([binary,str(root/"input.txt"),str(root/"cpp.csv")],capture_output=True,text=True)
            self.assertEqual(r.returncode,0,r.stderr)
            self.assertEqual(json.loads(r.stdout)["status"],"BOUNDED_MODEL_PLAN_PASS")
            write_preview_csv(root/"python.csv",p)
            np.testing.assert_allclose(np.loadtxt(root/"cpp.csv",delimiter=",",skiprows=1),
                                       np.loadtxt(root/"python.csv",delimiter=",",skiprows=1),atol=1e-12)

    def test_invalid_age_time_or_state_rejected(self):
        for age in (-.1,.8,float("nan")):
            with self.assertRaises(ValueError): make_segments(self.curve,age,self.q)
        for kwargs in ({"source_prefix":.3},{"time_scale":1},{"stop_seconds":0}):
            with self.assertRaises(ValueError): make_segments(self.curve,.4,self.q,**kwargs)
        with self.assertRaises(ValueError): make_segments(self.curve,.4,np.full(7,np.nan))


if __name__ == "__main__":
    unittest.main()
