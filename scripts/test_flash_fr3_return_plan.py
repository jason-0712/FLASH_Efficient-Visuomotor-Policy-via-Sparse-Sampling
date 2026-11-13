"""Return geometry/bounds and independent native validation. No robot IO."""
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

import numpy as np

from flash_fr3_model_plan import evaluate_segment
from flash_fr3_return_plan import (JOINT_NAMES, MAX_STEP_RAD, load_pose,
                                  make_return_segment, return_waypoints)

PREVIEW = '/home/ros/ros2_ws/install/flash_fr3_model_controller/lib/flash_fr3_model_controller/preview_task'


class ReturnPlanTests(unittest.TestCase):
    def setUp(self):
        self.home = np.array([-.16, -.77, .11, -2.38, .07, 1.57, .66])

    def test_rest_to_rest_monotone_position_and_physical_derivatives(self):
        end = self.home+np.array([.02, -.01, 0., .03, -.02, .035, -.03])
        plan = make_return_segment(self.home, end)
        c = np.array(plan['coefficients']); d = plan['segment_durations']
        for k in range(3):
            np.testing.assert_allclose(evaluate_segment(c[0], d[0], 0., k), 0., atol=1e-12)
            np.testing.assert_allclose(evaluate_segment(c[0], d[0], d[0], k),
                                       evaluate_segment(c[1], d[1], 0., k), atol=1e-12)
        np.testing.assert_allclose(evaluate_segment(c[1], d[1], d[1])+self.home, end, atol=1e-12)
        for k in (1, 2):
            np.testing.assert_allclose(evaluate_segment(c[1], d[1], d[1], k), 0., atol=1e-12)
        for t in np.linspace(0, d[0], 101):
            position = self.home+evaluate_segment(c[0], d[0], t)
            self.assertTrue((position >= np.minimum(self.home, end)-1e-12).all())
            self.assertTrue((position <= np.maximum(self.home, end)+1e-12).all())
        h = 1e-5
        for k in (1, 2, 3):
            finite_difference = (evaluate_segment(c[0], d[0], 1.+h, k-1)-
                                 evaluate_segment(c[0], d[0], 1.-h, k-1))/(2*h)
            np.testing.assert_allclose(finite_difference, evaluate_segment(c[0], d[0], 1., k), atol=1e-8)

    def test_complete_multisegment_path_agrees_with_native_validator(self):
        start = self.home+np.array([.15, .3, -.15, .3, -.03, .5, -.2])
        points = return_waypoints(start, self.home)
        self.assertEqual(len(points), 4)
        with tempfile.TemporaryDirectory() as temp:
            for point in points:
                self.assertLessEqual(abs(point-start).max(), MAX_STEP_RAD)
                plan = make_return_segment(start, point)
                source = Path(temp)/'input.txt'; output = Path(temp)/'preview.csv'
                values = plan['segment_durations']+np.asarray(plan['coefficients']).ravel().tolist()+start.tolist()
                source.write_text(' '.join(map(str, values))+'\n')
                p = subprocess.run([PREVIEW, str(source), str(output)], capture_output=True, text=True, check=True)
                native = json.loads(p.stdout)
                self.assertEqual(native['status'], 'TASK_SEGMENT_PLAN_PASS')
                for key in ('nominal_bounds', 'watchdog_brake_bounds'):
                    np.testing.assert_allclose(native[key], plan['bounds'][key], rtol=1e-10, atol=1e-12)
                rows = np.loadtxt(output, delimiter=',', skiprows=1)
                np.testing.assert_allclose(rows[-1, 1:8], point, atol=1e-12)
                np.testing.assert_allclose(rows[-1, 8:22], 0., atol=1e-12)
                start = point
        np.testing.assert_allclose(points[-1], self.home, atol=1e-12)

    def test_already_home_does_not_create_motion(self):
        self.assertEqual(return_waypoints(self.home+.001, self.home), [])

    def test_bad_pose_unit_order_robot_and_nonfinite_are_rejected(self):
        pose = {'format': 'flash_fr3_saved_joint_pose_v1', 'robot_ip': '172.16.0.2',
                'joint_names': JOINT_NAMES, 'q_rad': self.home.tolist(), 'units': 'rad'}
        np.testing.assert_array_equal(load_pose(pose), self.home)
        for key, value in [('units', 'deg'), ('robot_ip', 'other'), ('joint_names', JOINT_NAMES[::-1]),
                           ('q_rad', [float('nan')]*7), ('q_rad', [0.]*6)]:
            with self.subTest(key=key, value=value):
                with self.assertRaises(ValueError):
                    load_pose(pose | {key: value})

    def test_outside_deployment_box_and_excessive_step_are_rejected(self):
        with self.assertRaises(ValueError):
            make_return_segment(self.home, self.home+np.array([.18, 0., 0., 0., 0., 0., 0.]))
        with self.assertRaises(ValueError):
            return_waypoints(self.home, [0.]*7)
        with self.assertRaises(ValueError):
            make_return_segment(self.home, self.home+[0, 0, 0, 0, .1, 0, 0])


if __name__ == '__main__':
    unittest.main()
