#!/usr/bin/env python3
"""Read-only live sensor selection stress check using a recorded time schedule.

The recorded arm intervals are translated into the current time solely as a
software selection fixture. No robot motion, dynamics or inference is used.
All incoming images, joint states and timestamps remain actual sensor data.
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions

from capture_flash_fr3_observation import stamp_seconds
from flash_fr3_history_clock import ExecutedHistoryClock
from flash_fr3_retimed_buffer import HistoryCoverageError, RetimedObservationBuffer
from run_flash_fr3_grasp_probe import save


def read_control_state():
    # The installed controller services require Fast DDS; sensor reception
    # stays in Cyclone. The helper only makes the two read-only service calls.
    env=dict(os.environ,RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    env['PYTHONPATH']=str(Path(__file__).resolve().parent)+os.pathsep+env.get('PYTHONPATH','')
    code=('import json,rclpy; from run_flash_fr3_grasp_probe import require_inactive; '
          'rclpy.init(); n=rclpy.create_node("flash_history_audit_readonly_guard"); '
          'print(json.dumps(require_inactive(n))); n.destroy_node(); rclpy.try_shutdown()')
    result=subprocess.run([sys.executable,'-c',code],env=env,capture_output=True,text=True,timeout=10)
    if result.returncode:raise RuntimeError('只读控制器检查失败: '+result.stderr)
    return json.loads(result.stdout.strip())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recorded-trial',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('输出目录须不存在')
    files=sorted(args.recorded_trial.glob('segment_*/executed_history_interval.json'))
    if not files:parser.error('缺少已记录的运动区间')
    original=json.loads(files[-1].read_text())['clock_after_segment']
    schedule=ExecutedHistoryClock.restore(original)
    args.output.mkdir(parents=True);cv2.setNumThreads(1)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('flash_readonly_history_retention_audit')
    report={'status':'PENDING','robot_motion_commanded':False,'gripper_motion_commanded':False,
            'model_inference':False,'robot_dynamics':False,
            'intervals_are_software_selection_fixture':True,'source_trial':str(args.recorded_trial),
            'checks':[],'coverage_failures':[]}
    stop=threading.Event();worker=None;buffer=None;errors=[]
    try:
        report['controllers_before']=read_control_state()
        buffer=RetimedObservationBuffer(node)
        def spin():
            try:
                while not stop.is_set():rclpy.spin_once(node,timeout_sec=.002)
            except Exception as error:errors.append(str(error))
        worker=threading.Thread(target=spin);worker.start()
        deadline=time.monotonic()+20
        while True:
            try:initial,metadata=buffer.observation(-1);break
            except ValueError:
                if time.monotonic()>deadline:raise
                time.sleep(.05)
        initial_q=initial['agent_pos'][-1,:7].copy()
        shift=buffer.execution_clock.origin-schedule.origin
        for index,part in enumerate(schedule.intervals):
            end=part['end_monotonic']+shift
            while time.monotonic()<end+.45:
                if errors:raise RuntimeError(str(errors))
                time.sleep(.02)
            # This clock is a test fixture only; it is never sent to a model
            # or a robot command endpoint.
            buffer.execution_clock.record_arm(part['start_monotonic']+shift,end,part['time_scale'])
            try:
                started=time.monotonic()
                arrays,meta=buffer.observation(index)
                capture_ms=(time.monotonic()-started)*1000
                gap=float(abs(arrays['agent_pos'][-1,:7]-initial_q).max())
                if gap>.002:raise RuntimeError('只读检查期间机械臂姿态发生变化')
                result={'stage':index,'status':'PASS','capture_ms':capture_ms,'maximum_current_q_change_rad':gap,
                        'target_monotonic':[s['target_monotonic'] for s in meta['samples']],
                        'maximum_offsets_seconds':{key:max(abs(s['source_offset_from_target_seconds'][key])
                                                          for s in meta['samples']) for key in meta['topics']}}
            except HistoryCoverageError as error:
                report['coverage_failures'].append(error.diagnostic)
                result={'stage':index,'status':'COVERAGE_FAILED','diagnostic':error.diagnostic}
            report['checks'].append(result)
            print('FLASH_HISTORY_RETENTION_STAGE='+json.dumps(result),flush=True)
        stop.set();worker.join(timeout=2);worker=None
        if errors:raise RuntimeError(str(errors))
        report['controllers_after']=read_control_state()
        report['status']='HISTORY_RETENTION_PASS' if not report['coverage_failures'] else 'HISTORY_RETENTION_FAILED'
    except Exception as error:
        report.update(status='HISTORY_RETENTION_FAILED',error=str(error))
    finally:
        stop.set()
        if worker is not None:worker.join(timeout=2)
        if buffer is not None:
            with buffer.lock:history={key:list(values) for key,values in buffer.history.items()}
            stamps={key:np.array([[stamp_seconds(m),receipt,wall] for m,receipt,wall in values])
                    for key,values in history.items()}
            np.savez_compressed(args.output/'source_timestamps.npz',**stamps)
            report['sources']={key:{'message_count':len(values),
                'retained_seconds':float(values[-1,0]-values[0,0]) if len(values)>1 else 0,
                'maximum_source_gap_seconds':float(np.diff(values[:,0]).max()) if len(values)>1 else None}
                for key,values in stamps.items()}
        node.destroy_node();rclpy.try_shutdown()
        save(args.output/'report.json',report)
        print('FLASH_HISTORY_RETENTION='+json.dumps(report),flush=True)
    if report['status']!='HISTORY_RETENTION_PASS':raise SystemExit(1)


if __name__=='__main__':main()
