#!/usr/bin/env python3
"""All held-out windows and a bounded observation-convention diagnostic.

Offline HTTP inference only. No ROS, hardware connections or motion commands.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from audit_flash_fr3_alignment import (actual_window_function, digest, nominal_metadata,
                                      predict, save)
from flash_fr3_stream_io import connection


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--live-run', type=Path, required=True)
    parser.add_argument('--cache', type=Path, default=Path('artifacts/flash_fr3/pickplace_74eps_stride4'))
    parser.add_argument('--server', default='http://192.168.1.113:10132')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[1]
    earlier = json.loads((args.audit/'report.json').read_text())
    if earlier['status'] != 'ALIGNMENT_AUDIT_COMPLETE':
        raise ValueError('需要已完成的数据映射审计')
    manifest = json.loads((args.cache/'manifest.json').read_text())
    for filename, expected in manifest['cache_sha256'].items():
        if digest(args.cache/filename) != expected:
            raise ValueError('缓存 SHA 不一致：'+filename)
    arrays = {key: np.load(args.cache/(key+'.npy'), mmap_mode='r')
              for key in ('state', 'action', 'primary_cam', 'wrist_cam')}
    window = actual_window_function(root/'scripts/flash_fr3_dataset.py')
    opener, health = connection(args.server)
    report = {'status': 'PENDING', 'robot_motion_commanded': False, 'gripper_motion_commanded': False,
              'server': health, 'remote_checkpoint_sha_independently_verified': False,
              'scope': 'all held-out recorded windows; no executed future or physical task success evaluation'}
    rows = []; joint_sum = grip_sum = 0.; action_points = 0
    by_phase = {phase: {'windows': 0, 'joint_sum_rad': 0., 'gripper_sum': 0., 'arm_values': 0}
                for phase in ('close_near', 'release_near', 'other')}
    try:
        for ep in manifest['episodes']:
            if ep['episode_index'] not in manifest['splits']['val']:
                continue
            start, end = ep['start'], ep['end']
            labels = arrays['action'][start:end, 7]
            close = start+np.flatnonzero((labels[1:] > .5)&(labels[:-1] <= .5))+1
            release = start+np.flatnonzero((labels[1:] <= .5)&(labels[:-1] > .5))+1
            for current in range(start, end):
                seq = len(rows); indices = window(current, start, end)
                obs = {'agent_pos': arrays['state'][indices[1:9]].copy()}
                obs.update({key: arrays[key][indices[1:9]].copy() for key in ('primary_cam', 'wrist_cam')})
                reply = predict(opener, args.server, obs, nominal_metadata(seq), health['checkpoint_step'])
                save(args.output/f'prediction_{seq:03d}.json', reply)
                p = np.asarray(reply['action'], float); target = np.asarray(arrays['action'][indices[8:16]], float)
                error = abs(p-target); joint_sum += error[:, :7].sum(); grip_sum += error[:, 7].sum(); action_points += 8
                phase = ('close_near' if np.any(abs(close-current) <= 2) else
                         'release_near' if np.any(abs(release-current) <= 2) else 'other')
                group = by_phase[phase]; group['windows'] += 1; group['joint_sum_rad'] += error[:, :7].sum()
                group['gripper_sum'] += error[:, 7].sum(); group['arm_values'] += 56
                rows.append({'sequence': seq, 'episode': ep['episode_index'], 'sample': current,
                             'phase': phase, 'chunk_joint_mae_deg': float(np.rad2deg(error[:, :7]).mean()),
                             'chunk_gripper_mae': float(error[:, 7].mean()),
                             'anchor_joint_error_deg': np.rad2deg(p[0, :7]-target[0, :7]).tolist(),
                             'anchor_gripper_prediction': float(p[0, 7]),
                             'anchor_gripper_target': float(target[0, 7])})
                if len(rows) % 25 == 0:
                    print(f'离线完整验证：{len(rows)}/483', flush=True)
        save(args.output/'windows.json', rows)
        report['metrics'] = {'validation_windows': len(rows), 'val_action_points': action_points,
                             'joint_mae_rad': float(joint_sum/(action_points*7)),
                             'joint_mae_deg': float(np.rad2deg(joint_sum/(action_points*7))),
                             'gripper_mae': float(grip_sum/action_points)}
        report['phase_chunk_metrics'] = {
            phase: {'windows': group['windows'],
                    'joint_mae_deg': float(np.rad2deg(group['joint_sum_rad']/group['arm_values'])),
                    'gripper_mae': float(group['gripper_sum']/(group['windows']*8))}
            for phase, group in by_phase.items() if group['windows']}
        report['historical_offline_evaluation_comparison'] = {
            'reference_source': 'user supplied 15000-step EMA evaluation, 483 windows',
            'joint_mae_deg_absolute_difference': abs(report['metrics']['joint_mae_deg']-1.532312715005595),
            'gripper_mae_absolute_difference': abs(report['metrics']['gripper_mae']-.08440642483825515)}
        with np.load(args.live_run/'first_observation.npz', allow_pickle=False) as archive:
            live = {key: archive[key].copy() for key in ('agent_pos', 'primary_cam', 'wrist_cam')}
            metadata = json.loads(str(archive['metadata_json'].item()))
        baseline = predict(opener, args.server, live, metadata, health['checkpoint_step'])
        bounded = {key: value.copy() for key, value in live.items()}
        bounded['agent_pos'][:, 7] = np.clip(bounded['agent_pos'][:, 7], 0., 1.)
        corrected = predict(opener, args.server, bounded, metadata, health['checkpoint_step'])
        save(args.output/'live_original_prediction.json', baseline)
        save(args.output/'live_bounded_observation_prediction.json', corrected)
        origin = live['agent_pos'][-1, :7]
        before, after = np.asarray(baseline['action']), np.asarray(corrected['action'])
        report['gripper_observation_convention_probe'] = {
            'scope': 'paired offline diagnostic: bound measured closure only; original joint states and RGB unchanged; no action clamp',
            'changed_input_elements': int(np.count_nonzero(live['agent_pos'] != bounded['agent_pos'])),
            'max_input_closure_change': float(abs(live['agent_pos'][:, 7]-bounded['agent_pos'][:, 7]).max()),
            'before_anchor_joint_max_error_deg': float(np.rad2deg(abs(before[0, :7]-origin)).max()),
            'after_anchor_joint_max_error_deg': float(np.rad2deg(abs(after[0, :7]-origin)).max()),
            'maximum_joint_prediction_change_deg': float(np.rad2deg(abs(before[:, :7]-after[:, :7])).max()),
            'maximum_gripper_prediction_change': float(abs(before[:, 7]-after[:, 7]).max())}
        report['status'] = 'FULL_HELDOUT_ALIGNMENT_EVALUATION_COMPLETE'
    except Exception as error:
        report.update(status='FULL_HELDOUT_ALIGNMENT_EVALUATION_FAILED', error=str(error))
        raise
    finally:
        report['completed_windows'] = len(rows)
        report['source_sha256'] = {str(Path(__file__).relative_to(root)): digest(Path(__file__)),
                                   'scripts/audit_flash_fr3_alignment.py': digest(root/'scripts/audit_flash_fr3_alignment.py')}
        save(args.output/'report.json', report)
        print('FLASH_FULL_ALIGNMENT_EVAL='+json.dumps(report, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
