#!/usr/bin/env python3
"""Observe/predict on one physical clock and replay raw curves in native code.

No motion publishers, heartbeat, action clients or controller switches. Native
replay is numerical evaluation, not robot dynamics or a live effort controller.
"""
import argparse
import io
import json
import subprocess
import time
import urllib.request
from pathlib import Path

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions

from flash_fr3_continuous import native_input, physical_history, physical_reply, raw_packet
from flash_fr3_sensor_process import IsolatedObservationBuffer
from flash_fr3_shadow_math import evaluate_curve
from flash_fr3_stream_io import connection
from run_flash_fr3_grasp_probe import require_inactive


def save(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--server',default='http://192.168.1.113:10132')
    parser.add_argument('--steps',type=int,default=20)
    parser.add_argument('--native',type=Path,default=Path('/home/ros/ros2_ws/build/flash_fr3_model_controller/preview_continuous'))
    args=parser.parse_args()
    if args.output.exists() or not 3<=args.steps<=100 or not args.native.is_file():
        parser.error('输出目录须不存在；需要 3–100 个预测和已编译的原生数值求值器')
    args.output.mkdir(parents=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('flash_fr3_physical_continuous_shadow')
    buffer=None;packets=[];previous=None;last_anchor=None;offset=None;origin=None
    report={'status':'PENDING','robot_motion_commanded':False,'gripper_motion_commanded':False,
            'scope':'raw physical-clock predictions and 1kHz numerical replay; no live effort loop',
            'time_scale':1.,'history_idle_time_frozen':False,'coefficients_corrected':False,
            'arm_motion_controller_inactive':True,'requested_predictions':args.steps}
    try:
        report['controllers_before']=require_inactive(node)
        opener,health=connection(args.server)
        report['server']=health
        buffer=IsolatedObservationBuffer(node,local_keys=('arm','hand'))
        report['receiver']=buffer.receiver_info
        deadline=time.monotonic()+20
        while len(packets)<args.steps:
            try:
                arrays,metadata=buffer.observation(len(packets))
                anchor=physical_history(metadata,last_anchor)
            except ValueError as error:
                if time.monotonic()>deadline:
                    raise RuntimeError('未取得下一帧物理历史：'+str(error)) from error
                rclpy.spin_once(node,timeout_sec=.005)
                continue
            current_offset=time.time()-time.monotonic()
            if offset is None: offset=current_offset
            if abs(current_offset-offset)>.02: raise RuntimeError('源时钟与单调时钟偏移发生变化')
            require_inactive(node)
            blob=io.BytesIO();np.savez(blob,**arrays,metadata_json=np.asarray(json.dumps(metadata)))
            encoded=blob.getvalue();started=time.monotonic()
            request=urllib.request.Request(args.server+'/predict',data=encoded,
                                          headers={'Content-Type':'application/octet-stream'})
            with opener.open(request,timeout=2) as response: reply=json.load(response)
            ready=time.monotonic();physical_reply(reply,len(packets),anchor,health['checkpoint_step'])
            packet=raw_packet(reply,ready,offset,origin)
            if origin is None: origin=packet['observation_monotonic']
            if previous is not None:
                elapsed=packet['handoff_monotonic']-previous['observation_monotonic']
                try:
                    left=evaluate_curve(previous['curve'],elapsed)
                    packet['adjacent_raw_position_jump_rad']=float(abs(left[0][:7]-np.asarray(packet['raw_q_at_handoff'])).max())
                    packet['adjacent_raw_velocity_jump_rad_s']=float(abs(left[1][:7]-np.asarray(packet['raw_dq_at_handoff'])).max())
                except ValueError:
                    packet['adjacent_curve_overlap_missing']=True
            packet['current_measured_q_rad']=arrays['agent_pos'][-1,:7].astype(float).tolist()
            packet['raw_handoff_to_measured_gap_rad']=float(abs(np.asarray(packet['raw_q_at_handoff'])-arrays['agent_pos'][-1,:7]).max())
            packet['raw_observation_anchor_gap_rad']=float(abs(evaluate_curve(reply['curve'],0.)[0][:7]-arrays['agent_pos'][-1,:7]).max())
            packet['rpc_ms']=(ready-started)*1000
            index=len(packets)
            save(args.output/f'metadata_{index:03d}.json',metadata)
            save(args.output/f'prediction_{index:03d}.json',reply)
            if index==0: (args.output/'first_observation.npz').write_bytes(encoded)
            (args.output/'latest_observation.npz').write_bytes(encoded)
            packets.append(packet);save(args.output/'packets.json',packets)
            print('FLASH_CONTINUOUS_SHADOW_SAMPLE='+json.dumps({'sequence':index,
                'raw_observation_anchor_gap_rad':packet['raw_observation_anchor_gap_rad'],
                'adjacent_raw_position_jump_rad':packet.get('adjacent_raw_position_jump_rad'),
                'rpc_ms':packet['rpc_ms']},allow_nan=False),flush=True)
            previous=packet;last_anchor=anchor;deadline=time.monotonic()+5
        encoded=native_input(packets)
        (args.output/'native_packets.txt').write_text(encoded)
        result=subprocess.run([str(args.native)],input=encoded,text=True,capture_output=True,timeout=10)
        (args.output/'native.stderr').write_text(result.stderr)
        if result.returncode: raise RuntimeError('原生数值重放失败，exit='+str(result.returncode))
        replay=json.loads(result.stdout);save(args.output/'native_replay.json',replay)
        if replay.get('status')!='RAW_CONTINUOUS_NUMERICAL_REPLAY_COMPLETE':
            raise RuntimeError('原生重放结果格式异常')
        report.update(status='PHYSICAL_CONTINUOUS_SHADOW_COMPLETE',predictions=len(packets),
            observation_span_seconds=packets[-1]['observation_monotonic']-packets[0]['observation_monotonic'],
            maximum_raw_handoff_to_measured_gap_rad=max(p['raw_handoff_to_measured_gap_rad'] for p in packets),
            maximum_raw_observation_anchor_gap_rad=max(p['raw_observation_anchor_gap_rad'] for p in packets),
            measured_q_range_over_anchors_rad=np.ptp(np.asarray([p['current_measured_q_rad'] for p in packets]),axis=0).tolist(),
            maximum_adjacent_raw_position_jump_rad=max((p.get('adjacent_raw_position_jump_rad',0.) for p in packets)),
            maximum_adjacent_raw_velocity_jump_rad_s=max((p.get('adjacent_raw_velocity_jump_rad_s',0.) for p in packets)),
            native_replay=replay,
            readiness='DIAGNOSTIC_ONLY; native continuity acceptance is not hardware safety approval',
            live_effort_loop_implemented=False)
    except (Exception,KeyboardInterrupt) as error:
        report.update(status='PHYSICAL_CONTINUOUS_SHADOW_FAILED',error=str(error) or '用户中断',predictions=len(packets))
    finally:
        if buffer is not None: buffer.close()
        try:
            report['controllers_after']=require_inactive(node)
            report['all_command_interfaces_unclaimed']=True
        except Exception as error:
            report.update(status='PHYSICAL_CONTINUOUS_SHADOW_FAILED',final_state_error=str(error))
        save(args.output/'report.json',report)
        print('FLASH_CONTINUOUS_SHADOW_RESULT='+json.dumps(report,ensure_ascii=False,allow_nan=False),flush=True)
        node.destroy_node();rclpy.try_shutdown()
    if report['status']!='PHYSICAL_CONTINUOUS_SHADOW_COMPLETE': raise SystemExit(1)


if __name__=='__main__':
    main()
