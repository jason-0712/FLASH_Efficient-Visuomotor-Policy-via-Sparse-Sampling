#!/usr/bin/env python3
"""Audit cached command/state timing without a model, ROS, or robot commands.

Measured closure thresholds indicate finger response, not object contact or a
successful grasp. Sparse-cache timing is quantized at 4/15 second.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from flash_fr3_kinematics import UrdfKinematics


def statistics(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return None
    return {'count': len(values), 'min': float(values.min()), 'median': float(np.median(values)),
            'p95': float(np.percentile(values, 95)), 'max': float(values.max())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('输出文件已存在，拒绝覆盖')
    manifest = json.loads((args.cache/'manifest.json').read_text())
    state = np.load(args.cache/'state.npy', allow_pickle=False)
    action = np.load(args.cache/'action.npy', allow_pickle=False)
    timestamps = np.load(args.cache/'timestamp.npy', allow_pickle=False)
    assert state.shape == action.shape == (manifest['total_samples'], 8)
    assert np.isfinite(state).all() and np.isfinite(action).all()
    assert timestamps.shape == (len(state),) and np.isfinite(timestamps).all()
    fk = UrdfKinematics(args.urdf)
    xyz = fk.positions(state[:, :7])
    episodes = []
    for ep in manifest['episodes']:
        begin, end = ep['start'], ep['end']
        command, measured = action[begin:end, 7], state[begin:end, 7]
        times = timestamps[begin:end]
        assert (np.diff(times) > 0).all()
        rises = np.flatnonzero((command[1:] >= .7) & (command[:-1] < .7))+1
        event = {'episode_index': ep['episode_index'],
                 'split': 'val' if ep['episode_index'] in manifest['splits']['val'] else 'train',
                 'command_close_rising_edges': rises.tolist()}
        if len(rises):
            i = int(rises[0])
            falls = np.flatnonzero(command[i:] < .7)
            stop = i+int(falls[0]) if len(falls) else len(command)
            event.update(command_frame=i, command_timestamp_seconds=float(times[i]),
                         measured_closure_at_command=float(measured[i]),
                         flange_z_at_command_m=float(xyz[begin+i, 2]))
            for label, threshold in (('finger_response', .1), ('half_closure', .5)):
                hits = np.flatnonzero(measured[i:stop] >= threshold)
                event[label+'_delay_seconds'] = float(times[i+hits[0]]-times[i]) if len(hits) else None
            window = np.flatnonzero((times >= times[i]) & (times <= times[i]+1.1))
            event['minimum_flange_z_delta_within_1_1s_m'] = float(
                xyz[begin+window, 2].min()-xyz[begin+i, 2])
        episodes.append(event)
    summary = {
        'status': 'OFFLINE_LABEL_TIMING_AUDIT_COMPLETE', 'episodes': len(episodes),
        'episodes_with_close_edge': sum(bool(ep['command_close_rising_edges']) for ep in episodes),
        'sample_dt_seconds': manifest['sample_dt_seconds'],
        'joint_action_equals_current_measured_state': bool(np.array_equal(action[:, :7], state[:, :7])),
        'max_abs_joint_action_minus_state_rad': float(abs(action[:, :7]-state[:, :7]).max()),
        'gripper_state_range': [float(state[:, 7].min()), float(state[:, 7].max())],
        'gripper_command_range': [float(action[:, 7].min()), float(action[:, 7].max())]}
    for key in ('finger_response_delay_seconds', 'half_closure_delay_seconds',
                'minimum_flange_z_delta_within_1_1s_m'):
        summary[key] = statistics([ep[key] for ep in episodes if ep.get(key) is not None])
    report = {'summary': summary, 'episodes': episodes,
              'manifest_sha256': hashlib.sha256((args.cache/'manifest.json').read_bytes()).hexdigest(),
              'urdf_sha256': hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
              'scope': 'sparse cache labels only; no model prediction, robot IO or contact estimate; '
                       'delays are frame-level indicators, not calibrated hardware latency',
              'robot_motion_commanded': False, 'gripper_motion_commanded': False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print('FLASH_LABEL_TIMING_AUDIT='+json.dumps(summary, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
