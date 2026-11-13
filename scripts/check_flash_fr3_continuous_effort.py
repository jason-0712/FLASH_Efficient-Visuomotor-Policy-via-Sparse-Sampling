#!/usr/bin/env python3
"""Offline admission of saved real FLASH curves. No robot/ROS/network IO."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

from flash_fr3_continuous_effort import admission_input


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='已完成的 continuous_shadow_runs 子目录')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--native', type=Path, default=Path(
        '/home/ros/ros2_ws/build/flash_fr3_model_controller/preview_continuous_effort'))
    args = parser.parse_args()
    report = json.loads((args.run/'report.json').read_text())
    if report['status'] != 'PHYSICAL_CONTINUOUS_SHADOW_COMPLETE' or report['robot_motion_commanded']:
        parser.error('需要一轮完整的只读真实观测归档')
    packets = json.loads((args.run/'packets.json').read_text())
    prediction = json.loads((args.run/'prediction_000.json').read_text())
    q = packets[0]['current_measured_q_rad']
    offset = prediction['observation_target_stamp_seconds']-packets[0]['observation_monotonic']
    encoded = admission_input(packets, q, offset, 'offline-archive-admission')
    result = subprocess.run([str(args.native)], input=encoded, text=True, capture_output=True, timeout=15)
    if result.returncode:
        raise RuntimeError(f'原生准入检查失败：exit={result.returncode}，{result.stderr}')
    replay = json.loads(result.stdout)
    replay.update(source_run=str(args.run), prescribed_initial_measured_q_rad=q,
                  scope='archived real predictions, prescribed fixed measured state; no dynamics or live control',
                  wire_wall_time_checked_live=False,
                  source_packets_sha256=hashlib.sha256((args.run/'packets.json').read_bytes()).hexdigest())
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'native_input.txt').write_text(encoded)
    (args.output/'native.stderr').write_text(result.stderr)
    (args.output/'report.json').write_text(json.dumps(replay, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print('FLASH_CONTINUOUS_EFFORT_ADMISSION='+json.dumps({
        'status': replay['status'], 'packets': len(packets), 'accepted_packets': replay['accepted_packets'],
        'handoffs': replay['handoffs'], 'first_rejection': replay['events'][0]['rejection'],
        'first_raw_handoff_gap_rad': replay['events'][0]['raw_handoff_gap_to_prescribed_q_rad'],
        'robot_motion_commanded': False, 'hardware_ready': False,
        'report': str(args.output/'report.json')}, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
