#!/usr/bin/env python3
"""Read-only real sensor capture and optional new-protocol inference check."""
import argparse
import json
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import rclpy

from flash_fr3_retimed_schema import read_retimed_observation
from flash_fr3_sensor_process import RetimedIsolatedObservationBuffer
from flash_fr3_shadow_math import validate_reply
from flash_fr3_stream_io import connection
from run_flash_fr3_grasp_probe import require_inactive, save


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--server')
    args=parser.parse_args()
    if args.output.exists():parser.error('输出目录须不存在')
    args.output.mkdir(parents=True)
    cv2.setNumThreads(1);rclpy.init()
    node=rclpy.create_node('flash_fr3_retimed_capture_check');buffer=None
    report={'status':'PENDING','robot_motion_commanded':False,'gripper_motion_commanded':False}
    try:
        report['controllers_before']=require_inactive(node)
        buffer=RetimedIsolatedObservationBuffer(node,local_keys=('arm','hand'))
        deadline=time.monotonic()+20
        while True:
            rclpy.spin_once(node,timeout_sec=.005)
            try:arrays,metadata=buffer.observation(0);break
            except ValueError:
                if time.monotonic()>deadline:raise
        if np.ptp(arrays['agent_pos'][:,:7],axis=0).max()>.002:
            raise ValueError('机械臂没有保持同一静止姿态')
        path=args.output/'observation.npz'
        np.savez(path,**arrays,metadata_json=np.asarray(json.dumps(metadata)))
        read_retimed_observation(path)
        save(args.output/'metadata.json',metadata)
        for key in ('primary_cam','wrist_cam'):
            if not cv2.imwrite(str(args.output/(key+'_latest.png')),arrays[key][-1,:,:,::-1]):
                raise RuntimeError('保存相机预览失败')
        if args.server:
            opener,health=connection(args.server)
            if health.get('retimed_observations') is not True:
                raise ValueError('服务尚未支持重定时观测')
            request=urllib.request.Request(args.server+'/predict',data=path.read_bytes(),
                                           headers={'Content-Type':'application/octet-stream'})
            with opener.open(request,timeout=5) as response:reply=json.load(response)
            validate_reply(reply,0,metadata['samples'][-1]['target_stamp_seconds'],15000)
            if reply.get('history_clock')!=metadata['history_clock']:
                raise ValueError('服务器未确认实际历史时钟')
            save(args.output/'prediction.json',reply)
            report['inference']='PASS'
        report['controllers_after']=require_inactive(node)
        report.update(status='RETIMED_REAL_CAPTURE_PASS',observations=8,
                      history_clock=metadata['history_clock'],current_q_rad=arrays['agent_pos'][-1,:7].tolist())
    except Exception as error:
        report.update(status='RETIMED_REAL_CAPTURE_FAILED',error=str(error))
    finally:
        if buffer is not None:buffer.close()
        node.destroy_node();rclpy.shutdown()
        save(args.output/'report.json',report)
        print('FLASH_RETIMED_CAPTURE='+json.dumps(report,ensure_ascii=False),flush=True)
    if report['status']!='RETIMED_REAL_CAPTURE_PASS':raise SystemExit(1)


if __name__=='__main__':main()
