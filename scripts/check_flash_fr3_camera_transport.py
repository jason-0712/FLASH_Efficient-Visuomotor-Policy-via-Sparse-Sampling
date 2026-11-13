#!/usr/bin/env python3
"""Passive camera transport audit; subscriptions only, no robot commands.

Compare color CameraInfo with compressed image source stamps, record payload
sizes and this process's UDP socket counters, and correlate with task status.
CameraInfo is another ROS publication, not a direct device-side counter; its
delivery alone cannot certify where compressed image loss occurred.
"""
import argparse
import bisect
import json
import os
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float64MultiArray
from flash_fr3_transport_diagnostics import own_udp_sockets


def describe(rows):
    stamps = [row['source_stamp_seconds'] for row in rows]
    sizes = [row['payload_bytes'] for row in rows if 'payload_bytes' in row]
    gaps = [{'before': a['source_stamp_seconds'], 'after': b['source_stamp_seconds'],
             'gap_seconds': b['source_stamp_seconds'] - a['source_stamp_seconds'],
             'after_receipt_monotonic': b['receipt_monotonic']}
            for a, b in zip(rows, rows[1:])]
    ages = [row['receipt_wall_seconds'] - row['source_stamp_seconds'] for row in rows]
    return {'messages': len(rows),
            'source_hz': (len(stamps) - 1) / (stamps[-1] - stamps[0])
            if len(stamps) > 1 and stamps[-1] > stamps[0] else None,
            'source_order_errors': sum(b <= a for a, b in zip(stamps, stamps[1:])),
            'maximum_source_gap_seconds': max((g['gap_seconds'] for g in gaps), default=None),
            'largest_source_gaps': sorted(gaps, key=lambda g: g['gap_seconds'], reverse=True)[:12],
            'payload_bytes': {'min': min(sizes), 'median': statistics.median(sizes), 'max': max(sizes)}
            if sizes else None,
            'maximum_source_age_at_receipt_seconds': max(ages, default=None)}


def compare(info, compressed):
    if not info or not compressed:
        return {'status': 'MISSING_STREAM'}
    reference = sorted(row['source_stamp_seconds'] for row in info)
    images = sorted(row['source_stamp_seconds'] for row in compressed)
    # Ignore discovery and termination edges; compare the common retained span.
    begin, end = max(reference[0], images[0]) + .1, min(reference[-1], images[-1]) - .1
    targets = [t for t in reference if begin <= t <= end]
    missing = []
    for stamp in targets:
        index = bisect.bisect_left(images, stamp)
        nearby = images[max(0, index - 1):index + 1]
        if not nearby or min(abs(t - stamp) for t in nearby) > 1e-5:
            missing.append(stamp)
    return {'status': 'COMPARED', 'camera_info_frames_in_common_span': len(targets),
            'camera_info_frames_without_same_stamp_compressed': len(missing),
            'missing_compressed_stamps': missing,
            'stamp_match_tolerance_seconds': 1e-5,
            'scope': 'received ROS publications; CameraInfo is not a device-side counter'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=120)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 600 or args.output.exists():
        parser.error('采集时长须为 5–600 秒，输出目录须不存在')
    args.output.mkdir(parents=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('flash_fr3_passive_camera_transport_audit')
    qos = QoSProfile(depth=128, reliability=ReliabilityPolicy.BEST_EFFORT)
    rows = {key: [] for key in ('primary_compressed', 'primary_info', 'wrist_compressed', 'wrist_info')}
    subscriptions, status_rows, udp_rows = [], [], []
    events = (args.output / 'events.jsonl').open('w', buffering=1)
    start = time.monotonic()
    report = {'status': 'PENDING', 'robot_motion_commanded': False,
              'gripper_motion_commanded': False, 'model_inference': False,
              'pid': os.getpid(), 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
              'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION'),
              'scope': 'passive subscriptions; does not activate or control robot',
              'diagnostic_overhead': 'additional image subscriptions; payloads are not decoded or retained'}
    try:
        for camera, name in (('primary', 'right_third_person_camera'), ('wrist', 'right_wrist_camera')):
            prefix = '/right/' + name + '/color/'
            for suffix, message_type, topic in (
                    ('compressed', CompressedImage, prefix + 'image_raw/compressed'),
                    ('info', CameraInfo, prefix + 'camera_info')):
                key = camera + '_' + suffix

                def receive(message, key=key):
                    row = {'kind': key, 'source_stamp_seconds':
                           message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                           'receipt_monotonic': time.monotonic(), 'receipt_wall_seconds': time.time()}
                    if key.endswith('_compressed'):
                        row['payload_bytes'] = len(message.data)
                    rows[key].append(row)
                    events.write(json.dumps(row) + '\n')

                subscriptions.append(node.create_subscription(message_type, topic, receive, qos))

        def status(message):
            if len(message.data) != 32:
                return
            row = {'kind': 'task_status', 'receipt_monotonic': time.monotonic(),
                   'native_monotonic': message.data[0], 'phase': int(message.data[1]),
                   'reason': int(message.data[2])}
            status_rows.append(row)
            events.write(json.dumps(row) + '\n')

        subscriptions.append(node.create_subscription(
            Float64MultiArray, '/flash_fr3_task/status', status, qos))
        next_udp = start
        print('FLASH_CAMERA_TRANSPORT_AUDIT_READY=' + json.dumps({
            'seconds': args.seconds, 'output': str(args.output),
            'robot_motion_commanded': False, 'gripper_motion_commanded': False}), flush=True)
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if now >= next_udp:
                row = {'kind': 'own_udp', 'receipt_monotonic': now, 'sockets': own_udp_sockets()}
                udp_rows.append(row)
                events.write(json.dumps(row) + '\n')
                next_udp = now + 1
        report['status'] = 'READONLY_CAMERA_TRANSPORT_AUDIT_COMPLETE'
    except KeyboardInterrupt:
        report['status'] = 'READONLY_CAMERA_TRANSPORT_AUDIT_INTERRUPTED'
    except Exception as error:
        report.update(status='READONLY_CAMERA_TRANSPORT_AUDIT_FAILED', error=str(error))
    finally:
        report['elapsed_seconds'] = time.monotonic() - start
        report['sources'] = {key: describe(value) for key, value in rows.items()}
        report['comparisons'] = {key: compare(rows[key + '_info'], rows[key + '_compressed'])
                                 for key in ('primary', 'wrist')}
        report['task_status_samples'] = len(status_rows)
        report['task_phases_observed'] = sorted({row['phase'] for row in status_rows})
        baseline, maximum = {}, {}
        for sample in udp_rows:
            for row in sample['sockets']:
                inode, drops = row['inode'], row['drops']
                baseline.setdefault(inode, drops)
                maximum[inode] = max(maximum.get(inode, drops), drops)
        report['own_udp_drop_increment'] = sum(maximum[key] - value for key, value in baseline.items())
        report['own_udp_snapshots'] = len(udp_rows)
        events.close()
        node.destroy_node()
        rclpy.try_shutdown()
        (args.output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        compact = {key: report[key] for key in ('status', 'elapsed_seconds', 'robot_motion_commanded',
                   'gripper_motion_commanded', 'own_udp_drop_increment', 'task_phases_observed')}
        compact['sources'] = {key: {field: value[field] for field in
                              ('messages', 'source_hz', 'maximum_source_gap_seconds', 'payload_bytes')}
                              for key, value in report['sources'].items()}
        compact['comparisons'] = {key: {field: value for field, value in result.items()
                                  if field != 'missing_compressed_stamps'}
                                  for key, result in report['comparisons'].items()}
        print('FLASH_CAMERA_TRANSPORT_AUDIT=' + json.dumps(compact, ensure_ascii=False), flush=True)
    if report['status'] == 'READONLY_CAMERA_TRANSPORT_AUDIT_FAILED':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
