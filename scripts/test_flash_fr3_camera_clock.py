"""Frame identity, real arrival time and unchanged physical history; no robot."""
import copy
import json
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from realsense2_camera_msgs.msg import Metadata
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Header

from capture_flash_fr3_observation import JOINTS, FINGERS, TOPICS
from flash_fr3_camera_clock import SDK_TIME_BASIS, frame_clock
from flash_fr3_continuous import DT, physical_history
from run_flash_fr3_shadow_client import ObservationBuffer

EPOCH=1794387600.


def header(seconds):
    h=Header();ns=round(seconds*1e9);h.stamp.sec=ns//1000000000;h.stamp.nanosec=ns%1000000000
    return h


class Node:
    def __init__(self):self.callbacks={}
    def create_subscription(self,typ,topic,callback,qos):
        self.callbacks[topic]=callback;return callback


class CameraClockTest(unittest.TestCase):
    def observe(self):
        node=Node();buffer=ObservationBuffer(node,camera_time_basis=SDK_TIME_BASIS)
        buffer.grid_origin=EPOCH+7*DT
        jpeg=cv2.imencode('.jpg',np.zeros((32,32,3),np.uint8))[1].tobytes()
        with patch('run_flash_fr3_shadow_client.time') as clock:
            for i in range(8):
                stamp=EPOCH+i*DT;clock.time.return_value=stamp+.02;clock.monotonic.return_value=50+i*DT+.02
                h=header(stamp)
                node.callbacks[TOPICS['arm']](JointState(header=h,name=JOINTS,position=[0.]*7))
                node.callbacks[TOPICS['hand']](JointState(header=h,name=FINGERS,position=[.04,.04]))
                for key in ('primary_cam','wrist_cam'):
                    original=header(stamp+.15)
                    image=CompressedImage(header=original,format='jpeg',data=jpeg)
                    meta=Metadata(header=original,json_data=json.dumps(dict(time_of_arrival=round(stamp*1000),
                        frame_number=i+1,frame_counter=i+1,clock_domain='hardware_clock')))
                    image_cb=node.callbacks[TOPICS[key]]
                    meta_cb=node.callbacks[TOPICS[key].rsplit('/image_raw/compressed',1)[0]+'/metadata']
                    # Exercise both transport arrival orders.
                    if i%2:image_cb(image);meta_cb(meta)
                    else:meta_cb(meta);image_cb(image)
            clock.time.return_value=EPOCH+7*DT+.08;clock.monotonic.return_value=50+7*DT+.08
            arrays,metadata=buffer.observation(0)
        return buffer,arrays,metadata

    def test_matches_same_frame_with_150_ms_header_drift_and_preserves_pixels(self):
        buffer,arrays,metadata=self.observe()
        self.assertEqual(metadata['camera_time_basis'],SDK_TIME_BASIS)
        self.assertEqual(arrays['wrist_cam'].shape[0],8)
        self.assertTrue((arrays['wrist_cam']==0).all())
        physical_history(metadata)
        for sample in metadata['samples']:
            for key in ('primary_cam','wrist_cam'):
                clock=sample['camera_clock'][key]
                self.assertAlmostEqual(sample['stamp_seconds'][key],clock['sdk_time_of_arrival_ms']/1000.)
                self.assertAlmostEqual(clock['original_ros_header_stamp_seconds']-sample['stamp_seconds'][key],.15,delta=.001)
                self.assertGreater(sample['source_age_at_receipt_seconds'][key],0.)
        self.assertAlmostEqual(buffer.history['wrist_cam'][-1][0].header.stamp.sec+
            buffer.history['wrist_cam'][-1][0].header.stamp.nanosec*1e-9,EPOCH+7*DT+.15,delta=1e-6)

    def test_metadata_for_another_header_does_not_unblock_an_image(self):
        node=Node();buffer=ObservationBuffer(node,keys=('wrist_cam',),camera_time_basis=SDK_TIME_BASIS)
        topic=TOPICS['wrist_cam'];h=header(EPOCH+.15)
        node.callbacks[topic](CompressedImage(header=h,format='jpeg',data=b'image'))
        wrong=header(EPOCH+.16)
        node.callbacks[topic.rsplit('/image_raw/compressed',1)[0]+'/metadata'](
            Metadata(header=wrong,json_data=json.dumps(dict(time_of_arrival=int(EPOCH*1000),frame_number=1))))
        with self.assertRaisesRegex(ValueError,'缺少观测'):buffer.observation(0)
        self.assertNotIn('source_stamp_seconds',buffer.history['wrist_cam'][-1][3])

    def test_sdk_units_and_frame_counter_are_checked(self):
        for value in (EPOCH, EPOCH*1e6, True, float('nan')):
            with self.subTest(value=value),self.assertRaises(ValueError):
                frame_clock(int(EPOCH),0,json.dumps(dict(time_of_arrival=value,frame_number=1)))
        with self.assertRaises(ValueError):
            frame_clock(int(EPOCH),0,json.dumps(dict(time_of_arrival=int(EPOCH*1000),frame_number=2,frame_counter=3)))

    def test_reused_header_for_distinct_frames_is_rejected(self):
        node=Node();buffer=ObservationBuffer(node,keys=('wrist_cam',),camera_time_basis=SDK_TIME_BASIS)
        topic=TOPICS['wrist_cam'];h=header(EPOCH+.15)
        image=CompressedImage(header=h,format='jpeg',data=b'image')
        node.callbacks[topic](image)
        for frame in (1,2):
            node.callbacks[topic.rsplit('/image_raw/compressed',1)[0]+'/metadata'](Metadata(header=h,
                json_data=json.dumps(dict(time_of_arrival=int(EPOCH*1000)+frame,frame_number=frame))))
        with self.assertRaisesRegex(ValueError,'缺少观测'):buffer.observation(0)
        self.assertIn('error',buffer.history['wrist_cam'][-1][3])

    def test_physical_contract_checks_provenance_frame_order_and_age(self):
        _,_,metadata=self.observe()
        for change in ('provenance','frame_repeat','expired','future'):
            meta=copy.deepcopy(metadata)
            if change=='provenance':meta['samples'][0]['camera_clock']['wrist_cam']['sdk_time_of_arrival_ms']+=1
            if change=='frame_repeat':meta['samples'][1]['camera_clock']['wrist_cam']['frame_number']=1
            if change=='expired':meta['samples'][0]['source_age_at_receipt_seconds']['wrist_cam']=.31
            if change=='future':meta['samples'][0]['source_age_at_receipt_seconds']['wrist_cam']=-.06
            with self.subTest(change=change),self.assertRaises(ValueError):physical_history(meta)


if __name__=='__main__':unittest.main()
