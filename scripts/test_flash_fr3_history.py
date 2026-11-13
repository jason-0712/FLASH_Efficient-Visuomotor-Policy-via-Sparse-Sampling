"""Executed clock, real-source schema and image/state selection regression.

All message fixtures remain in memory. No ROS initialization, driver, robot
output or robot dynamics is used.
"""
import copy
import json
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np
from sensor_msgs.msg import JointState, CompressedImage
from std_msgs.msg import Header

from capture_flash_fr3_observation import JOINTS, FINGERS, measured_gripper_closure, stamp_seconds
from flash_fr3_history_clock import DT, ExecutedHistoryClock, arm_interval_from_status
from flash_fr3_retimed_buffer import HistoryCoverageError, RetimedObservationBuffer
from flash_fr3_retimed_schema import validate_retimed_observation
from run_flash_fr3_shadow_client import ObservationBuffer


def make_clock():
    clock=ExecutedHistoryClock(100.)
    for start in (101.,108.15,113.3):
        clock.record_arm(start,start+3.15,4.)
    return clock


def stamped(stamp):
    h=Header();h.stamp.sec=int(stamp);h.stamp.nanosec=int(round((stamp-int(stamp))*1e9))
    if h.stamp.nanosec==1000000000:h.stamp.sec+=1;h.stamp.nanosec=0
    return h


def fixture_buffer(clock=None,latest=120.45):
    node=SimpleNamespace(create_subscription=lambda *args,**kwargs:None)
    buffer=RetimedObservationBuffer(node)
    buffer.execution_clock=clock or make_clock();buffer.wall_offset=1000.
    _,targets=buffer.execution_clock.targets(latest)
    records=[(t,i) for i,t in enumerate(targets[:-1])]
    # During a four-second idle gap all recent wall-clock frames are static.
    records.extend((float(t),7) for t in np.arange(116.46,latest+.071,.02))
    records.append((latest,7));records.sort()
    for t,index in records:
        wall=t+1000.;h=stamped(wall)
        jpeg=cv2.imencode('.jpg',np.full((256,256,3),20*index,np.uint8))[1].tobytes()
        q=np.zeros(7);q[1]=index*.01
        messages={'arm':JointState(header=h,name=JOINTS,position=q.tolist(),velocity=[0.]*7),
                  'hand':JointState(header=h,name=FINGERS,position=[.04,.04]),
                  'primary_cam':CompressedImage(header=h,format='jpeg',data=jpeg),
                  'wrist_cam':CompressedImage(header=h,format='jpeg',data=jpeg)}
        for key,message in messages.items():buffer.history[key].append((message,t+.005,wall+.005))
    return buffer


def observe(buffer,latest=120.45):
    with patch('flash_fr3_retimed_buffer.time.monotonic',return_value=latest+.08), \
         patch('flash_fr3_retimed_buffer.time.time',return_value=latest+1000.08):
        return buffer.observation(17)


class ClockTests(unittest.TestCase):
    def test_idle_gaps_do_not_replace_motion_history(self):
        clock=make_clock();policy,targets=clock.targets(120.45)
        np.testing.assert_allclose(np.diff(policy),DT,atol=1e-14)
        self.assertAlmostEqual(clock.cursor,3*3.15/4)
        self.assertTrue(any(b-a>2 for a,b in zip(targets,targets[1:])))
        policy2,targets2=clock.targets(130.)
        np.testing.assert_array_equal(policy,policy2)
        np.testing.assert_array_equal(targets[:-1],targets2[:-1])
        self.assertEqual(targets2[-1],130.)

    def test_variable_playback_scales_and_bootstrap(self):
        clock=ExecutedHistoryClock(100.)
        p,t=clock.targets(102.)
        np.testing.assert_allclose(t[:-1],100.+np.arange(-7,0)*DT)
        clock.record_arm(103.,106.75,6.)
        clock.record_arm(110.,115.55,8.)
        self.assertAlmostEqual(clock.cursor,3.75/6+5.55/8)
        self.assertAlmostEqual(clock.source_time(.5),106.)
        self.assertAlmostEqual(clock.source_time(.625),106.75)

    def test_reject_overlap_expiration_unknown_scale_and_future(self):
        clock=make_clock()
        for args in ((110,113.15,4),(120,123.15,1),(120,121,4),(float('nan'),125,4)):
            with self.assertRaises(ValueError):clock.record_arm(*args)
        with self.assertRaises(ValueError):clock.targets(110)
        with self.assertRaises(ValueError):clock.targets(251)
        with self.assertRaises(ValueError):clock.source_time(clock.cursor+.01)
        with self.assertRaises(ValueError):ExecutedHistoryClock(100.).source_time(1e-10)

    def test_snapshot_rejects_changed_progress(self):
        clock=make_clock();restored=ExecutedHistoryClock.restore(clock.snapshot())
        self.assertEqual(restored.snapshot(),clock.snapshot())
        bad=clock.snapshot();bad['intervals'][0]['policy_end_seconds']+=.1
        with self.assertRaises(ValueError):ExecutedHistoryClock.restore(bad)

    def test_native_successful_interval_not_cli_runtime(self):
        rows=[{'data':[100.,0.,0.]},{'data':[101.,1.,0.]},{'data':[104.1502,4.,0.]}]
        plan={'total_motion_seconds':3.15,'time_scale':4.}
        value=arm_interval_from_status(rows,plan)
        self.assertEqual(value['start_monotonic'],101.)
        self.assertEqual(value['end_monotonic'],104.1502)
        with self.assertRaises(ValueError):arm_interval_from_status(rows[:-1],plan)
        rows[-1]['data'][2]=1
        with self.assertRaises(ValueError):arm_interval_from_status(rows,plan)

    def test_bad_start_transition_timing_rejected(self):
        rows=[{'data':[101.1,1.,0.]},{'data':[104.15,4.,0.]}]
        with self.assertRaises(ValueError):
            arm_interval_from_status(rows,{'total_motion_seconds':3.15,'time_scale':4.})


class SelectionTests(unittest.TestCase):
    def test_measured_closure_matches_bounded_recording_convention(self):
        for width, expected in ((.0808, 0.), (.08, 0.), (.04, .5), (0., 1.), (-.0008, 1.)):
            with self.subTest(width=width):
                self.assertAlmostEqual(float(measured_gripper_closure(width)), expected, places=7)
        for width in (.0811, -.0011, float('nan'), float('inf')):
            with self.subTest(width=width), self.assertRaises(ValueError):
                measured_gripper_closure(width)

    def test_retimed_receiver_bounds_measurement_without_changing_joint_history(self):
        buffer=fixture_buffer()
        for message, _, _ in buffer.history['hand']:
            message.position=[.0404,.0404]
        arrays,meta=observe(buffer)
        np.testing.assert_array_equal(arrays['agent_pos'][:,7],np.zeros(8))
        np.testing.assert_allclose(arrays['agent_pos'][:,1],np.arange(8)*.01,atol=1e-8)
        validate_retimed_observation(arrays,meta)

    def test_physical_receiver_bounds_measurement_without_changing_time_grid(self):
        latest=120.45
        buffer=fixture_buffer(clock=ExecutedHistoryClock(100.))
        # Fill the common physical history with the same fresh measured widths.
        for key in buffer.history:
            buffer.history[key].clear()
        for i,t in enumerate(latest+np.arange(-7,1)*DT):
            h=stamped(t+1000.)
            jpeg=cv2.imencode('.jpg',np.full((256,256,3),20*i,np.uint8))[1].tobytes()
            messages={'arm':JointState(header=h,name=JOINTS,position=[i*.01]*7),
                'hand':JointState(header=h,name=FINGERS,position=[.0404,.0404]),
                'primary_cam':CompressedImage(header=h,format='jpeg',data=jpeg),
                'wrist_cam':CompressedImage(header=h,format='jpeg',data=jpeg)}
            for key,message in messages.items():
                buffer.history[key].append((message,t+.005,t+1000.005))
        buffer.grid_origin=latest+1000.
        with patch('run_flash_fr3_shadow_client.time.monotonic',return_value=latest+.08), \
             patch('run_flash_fr3_shadow_client.time.time',return_value=latest+1000.08):
            arrays,meta=ObservationBuffer.observation(buffer,17)
        np.testing.assert_array_equal(arrays['agent_pos'][:,7],np.zeros(8))
        np.testing.assert_allclose(arrays['agent_pos'][:,0],np.arange(8)*.01,atol=1e-8)
        np.testing.assert_allclose(np.diff([s['target_stamp_seconds'] for s in meta['samples']]),DT,atol=1e-6)

    def test_large_joint_history_uses_bounded_timestamp_reads(self):
        buffer=fixture_buffer();buffer.history['arm'].clear()
        for stamp in np.linspace(1099.,1120.52,100000):
            message=SimpleNamespace(header=stamped(float(stamp)),name=JOINTS,position=[0.]*7)
            buffer.history['arm'].append((message,float(stamp)-1000.+.005,float(stamp)+.005))
        with patch('flash_fr3_retimed_buffer.stamp_seconds',wraps=stamp_seconds) as reads:
            arrays,meta=observe(buffer)
        self.assertLess(reads.call_count,1000)
        validate_retimed_observation(arrays,meta)

    def test_receipt_order_is_checked_before_binary_search(self):
        callbacks={};depths={}
        def subscribe(kind,topic,callback,qos):
            callbacks[topic]=callback;depths[topic]=qos.depth
        buffer=RetimedObservationBuffer(SimpleNamespace(create_subscription=subscribe))
        topic='/franka/joint_states';callback=callbacks[topic]
        self.assertEqual(depths[topic],256)
        for stamp in (100.,99.):callback(JointState(header=stamped(stamp),name=JOINTS,position=[0.]*7))
        with self.assertRaisesRegex(ValueError,'时间戳倒退'):
            buffer.observation(0)

    def test_newest_anchor_uses_received_common_time(self):
        buffer=fixture_buffer()
        while buffer.history['primary_cam'][-1][1]>.305+120.:
            buffer.history['primary_cam'].pop()
        newest_camera=buffer.history['primary_cam'][-1][0]
        camera_stamp=newest_camera.header.stamp.sec+newest_camera.header.stamp.nanosec*1e-9
        # Original fixed now-80ms target is >80ms ahead of this real frame.
        self.assertGreater(1120.45-camera_stamp,.08)
        arrays,meta=observe(buffer)
        self.assertLessEqual(meta['samples'][-1]['target_stamp_seconds'],camera_stamp+1e-6)
        expected=buffer.execution_clock.targets(120.45)[1][:-1]
        np.testing.assert_allclose([s['target_monotonic'] for s in meta['samples'][:-1]],expected)
        validate_retimed_observation(arrays,meta)

    def test_historical_gap_is_not_filled_or_hidden(self):
        buffer=fixture_buffer();target=buffer.execution_clock.targets(120.45)[1][4]
        kept=[value for value in buffer.history['primary_cam'] if abs(value[1]-.005-target)>.10]
        buffer.history['primary_cam'].clear();buffer.history['primary_cam'].extend(kept)
        with self.assertRaises(HistoryCoverageError) as caught:observe(buffer)
        d=caught.exception.diagnostic
        self.assertEqual(d['sample_index'],4)
        self.assertEqual(d['sample_kind'],'historical')
        self.assertEqual(d['uncovered_sources'],['primary_cam'])
        self.assertEqual(d['maximum_offset_seconds'],.08)
        self.assertGreater(abs(d['sources']['primary_cam']['selected_offset_seconds']),.08)

    def test_transport_context_separates_received_info_from_missing_image(self):
        buffer=fixture_buffer();target=buffer.execution_clock.targets(120.45)[1][4]
        for key in buffer.color_info_history:
            buffer.color_info_history[key].extend((stamp_seconds(item[0]),item[1]) for item in buffer.history[key])
        kept=[item for item in buffer.history['primary_cam'] if abs(item[1]-.005-target)>.10]
        buffer.history['primary_cam'].clear();buffer.history['primary_cam'].extend(kept)
        with self.assertRaises(HistoryCoverageError) as caught:observe(buffer)
        before=buffer.execution_clock.snapshot()
        buffer.transport_initial={'pid':123,'monotonic':99.,'udp_sockets':[{'inode':'10','drops':2}]}
        with patch('flash_fr3_retimed_buffer.own_udp_sockets',return_value=[{'inode':'10','drops':5}]):
            context=buffer.transport_gap_context(caught.exception.diagnostic)
        self.assertEqual(context['own_udp_drop_increment'],3)
        comparison=context['camera_comparisons']['primary_cam']
        self.assertIn(target+1000.,comparison['camera_info_stamps_inside_compressed_gap'])
        self.assertEqual(len(comparison['compressed_neighbors_payload_bytes']),2)
        self.assertEqual(buffer.execution_clock.snapshot(),before)
        with self.assertRaises(HistoryCoverageError):observe(buffer)

    def test_transport_context_does_not_invent_missing_camera_info(self):
        buffer=fixture_buffer();target=buffer.execution_clock.targets(120.45)[1][4]
        kept=[item for item in buffer.history['primary_cam'] if abs(item[1]-.005-target)>.10]
        buffer.history['primary_cam'].clear();buffer.history['primary_cam'].extend(kept)
        with self.assertRaises(HistoryCoverageError) as caught:observe(buffer)
        context=buffer.transport_gap_context(caught.exception.diagnostic)
        comparison=context['camera_comparisons']['primary_cam']
        self.assertEqual(comparison['camera_info_messages_retained'],0)
        self.assertEqual(comparison['camera_info_stamps_inside_compressed_gap'],[])
        self.assertIsNone(comparison['camera_info_before_target'])

    def test_retiming_retains_aligned_real_motion_frames(self):
        buffer=fixture_buffer();arrays,metadata=observe(buffer)
        np.testing.assert_allclose(arrays['agent_pos'][:,1],np.arange(8)*.01,atol=1e-7)
        for key in ('primary_cam','wrist_cam'):
            np.testing.assert_allclose(arrays[key][:,0,0,0],np.arange(8)*20,atol=1)
        self.assertEqual(metadata['request_id'],17)
        validate_retimed_observation(arrays,metadata)

    def test_legacy_wall_clock_history_contains_only_idle_frames(self):
        buffer=fixture_buffer()
        with patch('run_flash_fr3_shadow_client.time.monotonic',return_value=120.53), \
             patch('run_flash_fr3_shadow_client.time.time',return_value=1120.53):
            arrays,_=ObservationBuffer.observation(buffer,0)
        np.testing.assert_allclose(arrays['agent_pos'][:,1],.07)

    def test_missing_old_frames_fail_instead_of_repeating_latest(self):
        buffer=fixture_buffer()
        for key in buffer.history:
            first=buffer.history[key].popleft()
        with self.assertRaises(ValueError):observe(buffer)

    def test_failed_initial_warmup_does_not_freeze_the_clock(self):
        buffer=fixture_buffer();buffer.execution_clock=None;buffer.wall_offset=None
        for key in buffer.history:
            last=buffer.history[key][-1]
            buffer.history[key].clear();buffer.history[key].append(last)
        with self.assertRaises(ValueError):observe(buffer)
        self.assertIsNone(buffer.execution_clock)

    def test_wall_clock_jump_and_stale_sources_fail(self):
        buffer=fixture_buffer();buffer.wall_offset=999.
        with self.assertRaises(ValueError):observe(buffer)
        buffer=fixture_buffer()
        for key in buffer.history:
            while buffer.history[key] and buffer.history[key][-1][1]>120.:
                buffer.history[key].pop()
        with self.assertRaises(ValueError):observe(buffer)

    def test_metadata_cannot_relabel_physical_time_or_repeat_stamps(self):
        arrays,meta=observe(fixture_buffer())
        for kind in ('progress','source_time','duplicate','stale','skew','nan_state','rgb_dtype'):
            m=copy.deepcopy(meta);a={k:v.copy() for k,v in arrays.items()}
            if kind=='progress':m['samples'][0]['policy_time_seconds']+=.1
            elif kind=='source_time':m['samples'][0]['target_stamp_seconds']+=.2
            elif kind=='duplicate':m['samples'][1]['stamp_seconds']['arm']=m['samples'][0]['stamp_seconds']['arm']
            elif kind=='stale':m['samples'][0]['source_age_at_receipt_seconds']['arm']=.31
            elif kind=='skew':
                t=m['samples'][0]['target_stamp_seconds']
                m['samples'][0]['stamp_seconds']['arm']=t-.079
                m['samples'][0]['stamp_seconds']['wrist_cam']=t+.079
            elif kind=='nan_state':a['agent_pos'][0,0]=np.nan
            else:a['primary_cam']=a['primary_cam'].astype(np.float32)
            with self.subTest(kind=kind):
                with self.assertRaises(ValueError):validate_retimed_observation(a,m)


if __name__=='__main__':unittest.main()
