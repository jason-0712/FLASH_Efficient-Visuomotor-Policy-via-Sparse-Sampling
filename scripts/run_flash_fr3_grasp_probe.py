#!/usr/bin/env python3
"""Capture/replay one observation and diagnose grasp predictions; no commands.

Only ROS subscriptions and read-only controller/parameter services are used.
The returned polynomial is inspected offline and never sent to a controller.
"""
import argparse
import hashlib
import json
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from rcl_interfaces.srv import GetParameters

from flash_fr3_grasp_guard import GraspGeometryGate
from flash_fr3_sensor_process import IsolatedObservationBuffer
from flash_fr3_shadow_math import evaluate_curve, validate_reply
from flash_fr3_stream_io import connection


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def query(node, service, name, request):
    client = node.create_client(service, name)
    try:
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('只读服务不可用: ' + name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError('只读服务查询失败: ' + name)
        return future.result()
    finally:
        node.destroy_client(client)


def require_inactive(node):
    controllers = query(node, ListControllers, '/controller_manager/list_controllers',
                        ListControllers.Request()).controller
    if any(c.state == 'active' and c.name != 'joint_state_broadcaster' for c in controllers):
        raise RuntimeError('请先停止其他运动控制程序，再做同姿态对照')
    interfaces = query(node, ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces',
                       ListHardwareInterfaces.Request()).command_interfaces
    if len(interfaces) != 21 or any(i.is_claimed for i in interfaces):
        raise RuntimeError('命令接口未全部释放，不能记录静止姿态对照')
    return [{'name': c.name, 'state': c.state} for c in controllers]


def capture(output):
    rclpy.init()
    node = rclpy.create_node('flash_fr3_readonly_grasp_probe')
    buffer = None
    try:
        before = require_inactive(node)
        values = query(node, GetParameters, '/controller_manager/get_parameters',
                       GetParameters.Request(names=['robot_description'])).values
        if len(values) != 1 or values[0].type != 4 or not values[0].string_value:
            raise ValueError('无法读取当前机器人 URDF')
        (output / 'robot_description.urdf').write_text(values[0].string_value)
        buffer = IsolatedObservationBuffer(node, local_keys=('arm', 'hand'))
        deadline = time.monotonic() + 20
        while True:
            rclpy.spin_once(node, timeout_sec=.005)
            try:
                arrays, metadata = buffer.observation(0)
                break
            except ValueError:
                if time.monotonic() > deadline:
                    raise
        after = require_inactive(node)
        return arrays, metadata, {'before': before, 'after': after}
    finally:
        if buffer is not None:
            buffer.close()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--events', type=Path, required=True)
    parser.add_argument('--label', default='unspecified')
    parser.add_argument('--server', default='http://192.168.1.113:10131')
    parser.add_argument('--reference', type=Path, help='Previous probe report for a same-pose comparison')
    parser.add_argument('--observation', type=Path, help='Replay an existing observation without ROS capture')
    parser.add_argument('--urdf', type=Path, help='Required with --observation')
    args = parser.parse_args()
    if args.output.exists() or (args.observation is not None and args.urdf is None):
        parser.error('输出目录必须不存在；回放已有观测时必须提供 --urdf')
    args.output.mkdir(parents=True)
    if args.observation is None:
        arrays, metadata, controllers = capture(args.output)
    else:
        with np.load(args.observation, allow_pickle=False) as data:
            arrays = {key: data[key].copy() for key in ('agent_pos', 'primary_cam', 'wrist_cam')}
            metadata = json.loads(str(data['metadata_json'].item()))
        (args.output / 'robot_description.urdf').write_bytes(args.urdf.read_bytes())
        controllers = None
    if arrays['agent_pos'].shape != (8, 8) or not np.isfinite(arrays['agent_pos']).all():
        raise ValueError('需要八帧有限的八维状态')
    motion = float(np.ptp(arrays['agent_pos'][:, :7], axis=0).max())
    if motion > .002:
        raise ValueError('八帧历史中机械臂发生运动，不能当作静止姿态对照')
    observation = args.output / 'observation.npz'
    metadata['request_id'] = 0
    np.savez(observation, **arrays, metadata_json=np.asarray(json.dumps(metadata)))
    save(args.output / 'metadata.json', metadata)
    for key in ('primary_cam', 'wrist_cam'):
        if not cv2.imwrite(str(args.output / (key + '_latest.png')), arrays[key][-1, :, :, ::-1]):
            raise RuntimeError('保存相机预览失败')
    opener, health = connection(args.server)
    request = urllib.request.Request(args.server + '/predict', data=observation.read_bytes(),
                                    headers={'Content-Type': 'application/octet-stream'})
    start = time.monotonic()
    with opener.open(request, timeout=5) as response:
        reply = json.load(response)
    rpc_ms = (time.monotonic() - start) * 1000
    anchor = metadata['samples'][-1]['target_stamp_seconds']
    validate_reply(reply, 0, anchor, health['checkpoint_step'])
    save(args.output / 'prediction.json', reply)
    gate = GraspGeometryGate(args.output / 'robot_description.urdf',
                             json.loads(args.events.read_text())['grasp_q'])
    q = arrays['agent_pos'][-1, :7].astype(float)
    geometry = gate.check(q)
    times = np.linspace(0, reply['curve']['valid_future_seconds'], 161)
    predicted = np.stack([evaluate_curve(reply['curve'], t)[0] for t in times])
    positions = gate.fk.transforms(predicted[:, :7])[:, :3, 3]
    raw = evaluate_curve(reply['curve'], 0.)[0]
    reference = None
    if args.reference:
        previous = json.loads(args.reference.read_text())
        gap = float(abs(q - np.asarray(previous['current_q_rad'])).max())
        reference = {'report': str(args.reference), 'reference_label': previous['label'],
                     'max_joint_gap_rad': gap, 'same_pose_within_0_002_rad': gap <= .002,
                     'first_closure_change': float(raw[7] - previous['first_closure_raw'])}
    report = {'status': 'READONLY_GRASP_PROBE_COMPLETE', 'label': args.label,
              'robot_motion_commanded': False, 'gripper_motion_commanded': False,
              'mode': 'saved_observation_replay' if args.observation else 'live_capture',
              'observation_sha256': hashlib.sha256(observation.read_bytes()).hexdigest(),
              'current_q_rad': q.tolist(), 'measured_closure': float(arrays['agent_pos'][-1, 7]),
              'history_max_joint_range_rad': motion, 'current_geometry': geometry,
              'first_closure_raw': float(raw[7]), 'model_requests_closure_at_anchor': bool(raw[7] >= .7),
              'first_predicted_joint_gap_rad': float(abs(raw[:7] - q).max()),
              'predicted_flange_z_sampled_range_m': [float(positions[:, 2].min()),
                                                    float(positions[:, 2].max())],
              'predicted_closure_sampled_range': [float(predicted[:, 7].min()),
                                                 float(predicted[:, 7].max())],
              'reference_comparison': reference, 'controllers': controllers,
              'checkpoint_step': reply['checkpoint_step'], 'weights': reply['weights'],
              'server_predict_and_decode_ms': reply['server_predict_and_decode_ms'], 'rpc_ms': rpc_ms,
              'scope': 'Only a prediction comparison. Sampled polynomial range is not a certified '
                       'extremum; demonstration geometry does not establish object contact or collision safety. '
                       'Same-pose comparison also requires unchanged cameras, object position and scene.'}
    save(args.output / 'report.json', report)
    print('FLASH_GRASP_PROBE=' + json.dumps({key: report[key] for key in (
        'status', 'label', 'first_closure_raw', 'model_requests_closure_at_anchor',
        'predicted_flange_z_sampled_range_m', 'reference_comparison',
        'robot_motion_commanded', 'gripper_motion_commanded')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
