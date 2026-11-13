"""Physical-time and native raw-curve replay checks. No ROS or dynamics."""
import argparse
import copy
import json
import subprocess
import unittest
from pathlib import Path

import numpy as np

from flash_fr3_continuous import DT,native_input,physical_history,physical_history_after_handoff,physical_reply,raw_packet
from flash_fr3_shadow_math import evaluate_curve


def curve():
    return {'basis':'legendre','coefficients':np.zeros((7,8)).tolist(),
            'anchor_s':2/11,'polynomial_duration_seconds':44/15,'valid_future_seconds':32/15}


def metadata():
    keys=('arm','hand','primary_cam','wrist_cam')
    samples=[]
    for i in range(8):
        t=100+i*DT
        samples.append({'relative_seconds':i*DT,'target_stamp_seconds':t,
            'stamp_seconds':{key:t+.001 for key in keys},
            'source_age_at_receipt_seconds':{key:.01 for key in keys}})
    return {'format':'flash_fr3_live_observation_v1','sample_dt_seconds':DT,'n_obs_steps':8,
            'color':'RGB','joint_order':[f'fr3_joint{i}' for i in range(1,8)],
            'topics':{key:key for key in keys},'samples':samples}


class ContinuousTest(unittest.TestCase):
    def test_uniform_physical_time_and_duplicate_rejection(self):
        meta=metadata();anchor=physical_history(meta)
        self.assertAlmostEqual(anchor,100+7*DT)
        with self.assertRaises(ValueError): physical_history(meta,anchor)
        meta['samples'][3]['target_stamp_seconds']+=.01
        with self.assertRaises(ValueError): physical_history(meta)

    def test_frozen_history_and_source_gap_are_rejected(self):
        for change in ('clock','repeat','offset','age'):
            meta=metadata()
            if change=='clock': meta['history_clock']={'idle_time_policy':'frozen'}
            if change=='repeat': meta['samples'][2]['stamp_seconds']['hand']=meta['samples'][1]['stamp_seconds']['hand']
            if change=='offset': meta['samples'][2]['stamp_seconds']['primary_cam']+=.1
            if change=='age': meta['samples'][2]['source_age_at_receipt_seconds']['arm']=1.
            with self.assertRaises(ValueError): physical_history(meta)

    def test_new_grid_before_executed_handoff_is_not_feedback(self):
        meta=metadata();before=copy.deepcopy(meta);anchor=physical_history(meta)
        # A distinct new grid can still be older than the actual first motion.
        with self.assertRaisesRegex(ValueError,'尚无上一段接入后的真实观测'):
            physical_history_after_handoff(meta,anchor-DT,anchor+.1-10.,10.)
        self.assertEqual(meta,before)
        fresh=copy.deepcopy(meta)
        for sample in fresh['samples']:
            sample['target_stamp_seconds']+=DT
            for key in sample['stamp_seconds']:sample['stamp_seconds'][key]+=DT
        self.assertAlmostEqual(physical_history_after_handoff(fresh,anchor,anchor+.1-10.,10.),anchor+DT)
        # The target alone is insufficient if an actual image still precedes
        # handoff, even when that image meets the original 80 ms grid bound.
        fresh['samples'][-1]['stamp_seconds']['wrist_cam']=anchor+DT-.03
        with self.assertRaisesRegex(ValueError,'尚无上一段接入后的真实观测'):
            physical_history_after_handoff(fresh,anchor,anchor+DT-.02-10.,10.)

    def test_packet_preserves_raw_curve_and_physical_derivative(self):
        reply={'request_id':0,'observation_target_stamp_seconds':1100.,'curve':curve()}
        reply['curve']['coefficients'][1][3]=.02
        before=copy.deepcopy(reply)
        packet=raw_packet(reply,100.1,1000.)
        self.assertEqual(reply,before)
        values=evaluate_curve(reply['curve'],packet['handoff_monotonic']-100.)
        np.testing.assert_allclose(packet['raw_q_at_handoff'],values[0][:7])
        self.assertAlmostEqual(packet['raw_dq_at_handoff'][3],.02*2/(44/15))

    def test_stale_and_future_predictions_are_rejected(self):
        reply={'request_id':0,'observation_target_stamp_seconds':1100.,'curve':curve()}
        for ready in (99.,101.):
            with self.assertRaises(ValueError): raw_packet(reply,ready,1000.)

    def test_server_must_echo_the_physical_clock_contract(self):
        reply={'protocol':'flash_fr3_shadow_v1','status':'SHADOW_PREDICTION',
               'robot_motion_commanded':False,'weights':'ema_model','request_id':0,
               'checkpoint_step':15000,'observation_target_stamp_seconds':1100.,
               'action':np.zeros((8,8)).tolist(),'curve':curve(),
               'observation_format':'flash_fr3_live_observation_v1','history_clock':None}
        physical_reply(reply,0,1100.,15000)
        reply['history_clock']={'time_scale':.5}
        with self.assertRaises(ValueError): physical_reply(reply,0,1100.,15000)
        reply['history_clock']=None;reply['observation_format']='flash_fr3_retimed_observation_v1'
        with self.assertRaises(ValueError): physical_reply(reply,0,1100.,15000)
        reply['observation_format']='flash_fr3_live_observation_v1';del reply['history_clock']
        with self.assertRaises(ValueError): physical_reply(reply,0,1100.,15000)

    def test_native_input_cannot_hide_retimed_curve(self):
        reply={'request_id':0,'observation_target_stamp_seconds':1100.,'curve':curve()}
        packet=raw_packet(reply,100.1,1000.)
        encoded=native_input([packet])
        self.assertEqual(len(encoded.splitlines()[1].split()),53)
        for invalid_duration in (6.,float('nan'),float('inf')):
            packet['curve']['polynomial_duration_seconds']=invalid_duration
            with self.assertRaises(ValueError): native_input([packet])

    def replay(self,packets):
        result=subprocess.run([str(ARGS.native)],input=native_input(packets),
                              text=True,capture_output=True,timeout=5)
        self.assertEqual(result.returncode,0,result.stderr)
        return json.loads(result.stdout)

    def test_native_consumes_twenty_physical_handoffs_with_nonzero_velocity(self):
        packets=[]
        for i in range(21):
            observed=100+i*DT;c=curve();v=.001
            c['coefficients'][1][3]=v*(44/15)/2
            c['coefficients'][0][3]=v*(observed-100)-c['coefficients'][1][3]*(2*(2/11)-1)
            reply={'request_id':i,'observation_target_stamp_seconds':observed+1000,'curve':c}
            packets.append(raw_packet(reply,observed+.1,1000.,100.))
        result=self.replay(packets)
        self.assertEqual(result['handoffs'],20,result)
        self.assertEqual(result['phase'],1,result)
        self.assertTrue(all(event['accepted'] for event in result['events']),result)
        self.assertGreater(result['samples'],5000)

    def test_native_rejects_raw_jump_and_expires_without_coefficient_repair(self):
        packets=[]
        for i in range(12):
            observed=100+i*DT;c=curve();c['coefficients'][0][3]=.03 if i else 0.
            reply={'request_id':i,'observation_target_stamp_seconds':observed+1000,'curve':c}
            packets.append(raw_packet(reply,observed+.1,1000.,100.))
        result=self.replay(packets)
        self.assertEqual(result['handoffs'],0,result)
        self.assertEqual(result['phase'],2,result)
        self.assertFalse(result['events'][1]['accepted'])
        self.assertAlmostEqual(result['events'][1]['position_gap_rad'],.03)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native',type=Path,required=True)
    ARGS,remaining=parser.parse_known_args()
    unittest.main(argv=[__file__,*remaining])
