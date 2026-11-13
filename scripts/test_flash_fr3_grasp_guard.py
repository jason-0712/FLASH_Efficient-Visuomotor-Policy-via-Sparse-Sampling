"""Geometric grasp gate checks against explicit kinematic fixtures; no robot IO."""
import tempfile
import unittest
from pathlib import Path

import numpy as np

from flash_fr3_grasp_guard import GraspGeometryGate


class GraspGeometryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name)/'fixture.urdf'
        links = ''.join(f'<link name="fr3_link{i}"/>' for i in range(9))
        joints = []
        for i in range(1, 8):
            origin = '<origin xyz="0 0 .2"/>' if i == 1 else ('<origin xyz="1 0 0"/>' if i == 2 else '')
            axis = '0 1 0' if i == 1 else '0 0 1'
            joints.append(f'<joint name="fr3_joint{i}" type="revolute"><parent link="fr3_link{i-1}"/>'
                          f'<child link="fr3_link{i}"/>{origin}<axis xyz="{axis}"/></joint>')
        fixed = '<joint name="flange" type="fixed"><parent link="fr3_link7"/><child link="fr3_link8"/></joint>'
        self.path.write_text('<robot name="geometry_fixture">'+links+''.join(joints)+fixed+'</robot>')
        self.zero = np.zeros(7)

    def tearDown(self):
        self.temp.cleanup()

    def test_exact_event_and_small_perturbation_pass(self):
        gate = GraspGeometryGate(self.path, [self.zero])
        self.assertTrue(gate.check(self.zero)['accepted'])
        q = self.zero.copy(); q[0] = .005
        self.assertTrue(gate.check(q)['accepted'])

    def test_too_high_rejected_even_when_joint_position_rotation_checks_pass(self):
        gate = GraspGeometryGate(self.path, [self.zero])
        q = self.zero.copy(); q[0] = -.015
        result = gate.check(q)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['compatible_events'], 1)
        self.assertEqual(result['reasons'], ['法兰高度超出示教抓取高度带'])

    def test_rotation_rejected_even_with_same_position_and_small_each_joint_error(self):
        gate = GraspGeometryGate(self.path, [self.zero])
        q = np.array([0., .03, .03, .03, .03, .03, .03])
        result = gate.check(q)
        self.assertFalse(result['accepted'])
        self.assertLess(result['nearest_position_distance_m'], 1e-12)
        self.assertGreater(result['nearest_position_rotation_error_deg'], 10.)

    def test_different_events_cannot_satisfy_separate_checks(self):
        a, b = self.zero.copy(), self.zero.copy()
        a[0] = .03; b[1] = .10
        gate = GraspGeometryGate(self.path, [a, b])
        result = gate.check(self.zero)
        self.assertFalse(result['accepted'])
        self.assertEqual(result['compatible_events'], 0)
        self.assertLess(result['nearest_position_distance_m'], 1e-12)
        self.assertLess(result['nearest_joint_gap_rad'], .06)

    def test_bad_joint_state_and_empty_events_rejected(self):
        gate = GraspGeometryGate(self.path, [self.zero])
        for q in ([0.]*6, [float('nan')]*7):
            with self.assertRaises(ValueError):gate.check(q)
        with self.assertRaises(ValueError):GraspGeometryGate(self.path, [])


if __name__ == '__main__':
    unittest.main()
