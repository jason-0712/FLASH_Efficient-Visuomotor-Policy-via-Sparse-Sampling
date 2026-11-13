"""Cross-language effort admission checks; prescribed states, no dynamics."""
import argparse
import copy
import json
import subprocess
import unittest
from pathlib import Path

import numpy as np

from flash_fr3_continuous_effort import admission_input, effort_message
from flash_fr3_continuous import raw_packet

Q = [-.16, -.77, .11, -2.38, .07, 1.57, .66]


def packets(count=21):
    output = []
    for i in range(count):
        observation = 100+i*4/15
        c = np.zeros((7, 8)); c[0, :7] = Q
        v = .00005; c[1, 3] = v*(44/15)/2
        c[0, 3] += v*(observation-100)-c[1, 3]*(2*(2/11)-1)
        c[:, 7] = np.arange(7)+50  # gripper never enters the seven effort joints
        output.append({'sequence': i, 'observation_monotonic': observation,
            'ready_monotonic': observation+.02, 'handoff_monotonic': observation+.1,
            'time_policy': 'physical_1x_no_history_freeze_no_coefficient_correction',
            'curve': {'basis': 'legendre', 'coefficients': c.tolist(), 'anchor_s': 2/11,
                      'polynomial_duration_seconds': 44/15, 'valid_future_seconds': 32/15}})
    return output


class EffortWireTest(unittest.TestCase):
    def test_exact_57_field_contract_preserves_arm_coefficients(self):
        packet = packets(1)[0]; before = copy.deepcopy(packet)
        message = effort_message(packet, 'isolated-wire-test-session', 1100.)
        self.assertEqual(message['layout']['dim'][0]['size'], 57)
        self.assertEqual(len(message['data']), 57)
        np.testing.assert_array_equal(np.asarray(message['data'][8:]).reshape(7, 7),
                                      np.asarray(packet['curve']['coefficients'])[:, :7])
        self.assertEqual(packet, before)

    def test_schema_clock_and_retiming_cannot_be_hidden(self):
        for name in ('sequence', 'duration', 'basis', 'nan', 'future', 'late', 'retimed'):
            packet = packets(1)[0]
            if name == 'sequence': packet['sequence'] = 1.5
            if name == 'duration': packet['curve']['polynomial_duration_seconds'] = 10.
            if name == 'basis': packet['curve']['basis'] = 'power'
            if name == 'nan': packet['curve']['coefficients'][3][2] = float('nan')
            if name == 'future': packet['observation_monotonic'] += 1.
            if name == 'late': packet['ready_monotonic'] += 1.
            if name == 'retimed': packet['time_policy'] = 'frozen'
            with self.subTest(name=name), self.assertRaises(ValueError):
                effort_message(packet, 'isolated-wire-test-session', 1100.)
        with self.assertRaises(ValueError): effort_message(packets(1)[0], 'short', 1100.)

    def replay(self, raw):
        result = subprocess.run([str(ARGS.native)], text=True, capture_output=True,
            input=admission_input(raw, Q, 1000., 'isolated-wire-test-session'), timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads(result.stdout)

    def test_python_wire_to_native_admission_twenty_handoffs(self):
        result = self.replay(packets())
        self.assertEqual(result['accepted_packets'], 21, result)
        self.assertEqual(result['handoffs'], 20, result)
        self.assertFalse(result['hardware_ready'])
        self.assertTrue(result['prescribed_measured_state_fixed'])

    def test_entire_future_envelope_rejected_without_scaling(self):
        raw = packets(1)
        # q is exactly at measured q at the handoff, but moves >0.002 later.
        c = np.asarray(raw[0]['curve']['coefficients']); c[:, :7] = 0.; c[0, :7] = Q
        c[1, 3] = .01; c[0, 3] -= .01*(2*((2/11)+.1/(44/15))-1)
        raw[0]['curve']['coefficients'] = c.tolist()
        result = self.replay(raw)
        self.assertLess(result['events'][0]['raw_handoff_gap_to_prescribed_q_rad'], 1e-12)
        self.assertEqual(result['events'][0]['rejection'], 5)
        self.assertEqual(result['accepted_packets'], 0)

    def test_raw_boundary_jump_rejected_without_repair(self):
        raw = packets(2); raw[1]['curve']['coefficients'][0][2] += .001
        result = self.replay(raw)
        self.assertEqual(result['accepted_packets'], 1)
        self.assertEqual(result['events'][1]['rejection'], 6)
        self.assertEqual(result['handoffs'], 0)

    def test_startup_anchor_keeps_real_source_age_and_raw_coefficients(self):
        old = packets(1)[0]
        c = np.asarray(old['curve']['coefficients'])
        c[:, :7] = 0.; c[0, :7] = Q
        c[1, 3] = .15*(44/15)/2
        c[0, 3] -= c[1, 3]*(2*(2/11)-1)
        old['curve']['coefficients'] = c.tolist()
        reply = dict(request_id=0, observation_target_stamp_seconds=1100., curve=old['curve'])
        legacy = raw_packet(reply,100.33,1000.)
        first = raw_packet(reply,100.33,1000.,start_at_anchor=True)
        self.assertGreater(abs(legacy['raw_q_at_handoff'][3]-Q[3]),.05)
        self.assertAlmostEqual(first['raw_q_at_handoff'][3],Q[3])
        self.assertEqual(first['observation_monotonic'],100.)
        self.assertEqual(first['curve'],reply['curve'])
        encoded = effort_message(first,'isolated-wire-test-session',1100.33)
        self.assertEqual(len(encoded['data']),58)
        self.assertEqual(encoded['data'][0],2.)
        np.testing.assert_array_equal(np.asarray(encoded['data'][8:57]).reshape(7,7),c[:,:7])
        def admission(packet):
            result = subprocess.run([str(ARGS.native),'--task-rollout'],text=True,capture_output=True,
                input=admission_input([packet],Q,1000.,'isolated-wire-test-session'),timeout=5)
            self.assertEqual(result.returncode,0,result.stderr)
            return json.loads(result.stdout)['events'][0]
        self.assertEqual(admission(legacy)['rejection'],6)
        self.assertTrue(admission(first)['accepted'])
        stale=copy.deepcopy(first);stale['observation_monotonic']-=1.
        with self.assertRaises(ValueError): effort_message(stale,'isolated-wire-test-session',1100.33)
        later=copy.deepcopy(first);later['sequence']=1
        with self.assertRaises(ValueError): effort_message(later,'isolated-wire-test-session',1100.33)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', type=Path, required=True)
    ARGS, remaining = parser.parse_known_args()
    unittest.main(argv=[__file__, *remaining])
