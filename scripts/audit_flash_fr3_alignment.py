#!/usr/bin/env python3
"""Audit saved data/time contracts and replay recorded inputs through HTTP.

No ROS, hardware connection, controller/action publisher or dynamics. Replayed
dataset timestamps are nominal schema fields, not live sensor measurements.
"""
import argparse
import ast
import hashlib
import io
import json
import time
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
from numpy.polynomial import legendre

from flash_fr3_continuous import physical_history, physical_reply
from flash_fr3_shadow_math import evaluate_curve, validate_reply
from flash_fr3_stream_io import connection

DT = 4/15
KEYS = ('arm', 'hand', 'primary_cam', 'wrist_cam')


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8*1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def actual_window_function(script):
    """Load just this pure function from the actual dataset module, no torch."""
    tree = ast.parse(script.read_text())
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'window_indices']
    if len(functions) != 1:
        raise ValueError('训练窗口函数缺失或重复')
    scope = {'np': np}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(script), 'exec'), scope)
    return scope['window_indices']


def nominal_metadata(sequence):
    samples = []
    for i in range(8):
        stamp = 1000+i*DT
        samples.append({'relative_seconds': i*DT, 'target_stamp_seconds': stamp,
                        'stamp_seconds': {key: stamp for key in KEYS},
                        'source_age_at_receipt_seconds': {key: 0. for key in KEYS}})
    return {'format': 'flash_fr3_live_observation_v1', 'history_clock': None,
            'capture_mode': 'offline_recorded_dataset_replay', 'request_id': sequence,
            'nominal_replay_not_sensor_capture': True, 'joint_order': [f'fr3_joint{i}' for i in range(1, 8)],
            'n_obs_steps': 8, 'color': 'RGB', 'sample_dt_seconds': DT,
            'topics': {key: 'offline recorded data; nominal stamps only' for key in KEYS}, 'samples': samples}


def expert_projection(action, full_indices):
    """Independent float64 reconstruction of audited upstream KKT targets.

    Affine per-dimension normalization commutes with this physical polynomial
    fit. This is an oracle target calculation, not a new model or correction.
    """
    x = np.linspace(-1., 1., 12)
    basis = legendre.legvander(x, 6)
    derivative = legendre.legval(x, legendre.legder(np.eye(7), axis=0)).T*2
    pins = [2, 10]
    constraint = np.concatenate([basis[pins], derivative[pins]])
    inv_at = np.linalg.solve(basis.T@basis, constraint.T)
    correction = inv_at@np.linalg.inv(constraint@inv_at)
    targets = np.asarray(action[full_indices[:, 6:18]], float)
    coefficients = np.einsum('kl,bld->bkd', np.linalg.pinv(basis), targets)
    velocity = (targets[:, np.array(pins)+1]-targets[:, np.array(pins)-1])*11/2
    b = np.concatenate([targets[:, pins], velocity], axis=1)
    residual = np.einsum('lk,bkd->bld', constraint, coefficients)-b
    coefficients -= np.einsum('kl,bld->bkd', correction, residual)
    projected = np.einsum('lk,bkd->bld', basis, coefficients)
    chunk = projected[:, 2:10]
    expected = np.asarray(action[full_indices[:, 8:16]], float)
    return {'method': 'independent float64 upstream target-fit reconstruction; not model predictions',
            'maximum_anchor_joint_residual_rad': float(abs(projected[:, 2, :7]-targets[:, 2, :7]).max()),
            'maximum_constraint_residual': float(abs(np.einsum('lk,bkd->bld', constraint, coefficients)-b).max()),
            'action_chunk_joint_mae_deg': float(np.rad2deg(abs(chunk[:, :, :7]-expected[:, :, :7])).mean()),
            'action_chunk_joint_max_error_deg': float(np.rad2deg(abs(chunk[:, :, :7]-expected[:, :, :7])).max())}


def audit_cache(root, cache, source):
    manifest = json.loads((cache/'manifest.json').read_text())
    if manifest['sample_dt_seconds'] != DT:
        raise ValueError('缓存物理采样间隔错误')
    for name, expected in manifest['cache_sha256'].items():
        if digest(cache/name) != expected:
            raise ValueError('缓存 SHA 不一致：'+name)
    arrays = {key: np.load(cache/(key+'.npy'), mmap_mode='r')
              for key in ('state', 'action', 'timestamp', 'raw_frame_index', 'primary_cam', 'wrist_cam')}
    info = json.loads((source/'meta/info.json').read_text())
    window = actual_window_function(root/'scripts/flash_fr3_dataset.py')
    all_indices = []; source_max = np.zeros(4); episodes = []
    for ep in manifest['episodes']:
        idx, start, end = ep['episode_index'], ep['start'], ep['end']
        filename = info['data_path'].format(episode_chunk=idx//info.get('chunks_size', 1000), episode_index=idx)
        if digest(source/filename) != manifest['source_sha256'][filename]:
            raise ValueError('原始 Parquet SHA 不一致：'+filename)
        table = pq.read_table(source/filename)
        values = lambda name: np.asarray(table[name].to_pylist())
        q = values('observation.state.joints').astype(np.float32)
        grip = values('observation.state.gripper').astype(np.float32).reshape(-1, 1)
        command = values('action').astype(np.float32)
        expected_state = np.concatenate([q, grip], axis=1)[::4]
        expected_action = np.concatenate([q, 1-command[:, -1:]], axis=1)[::4]
        times = values('timestamp').astype(float)[::4]
        checks = [np.max(abs(expected_state-arrays['state'][start:end])),
                  np.max(abs(expected_action-arrays['action'][start:end])),
                  np.max(abs(times-arrays['timestamp'][start:end])),
                  np.max(abs(np.arange(len(q))[::4]-arrays['raw_frame_index'][start:end]))]
        source_max = np.maximum(source_max, checks)
        if any(value != 0 for value in checks):
            raise ValueError(f'episode {idx} 原始数据到缓存映射不一致')
        for current in range(start, end):
            full = window(current, start, end)
            expected = np.clip(current+np.arange(-8, 10), start, end-1)
            if not np.array_equal(full, expected):
                raise ValueError('训练窗口偏移错误')
            if full[8] != current or np.any(full[1:9] > current):
                raise ValueError('训练观测锚点错误或含未来帧')
            all_indices.append(full)
        episodes.append({'episode': idx, 'raw_frames': len(q), 'sparse_frames': end-start})
    ids = np.asarray(all_indices)
    if len(ids) != manifest['total_samples']:
        raise ValueError('窗口计数错误')
    train_ids = np.concatenate([np.arange(ep['start'], ep['end']) for ep in manifest['episodes']
                                if ep['episode_index'] in manifest['splits']['train']])
    low, high = arrays['state'][train_ids].min(axis=0), arrays['state'][train_ids].max(axis=0)
    report = {'status': 'PASS', 'episodes': len(episodes), 'raw_frames': sum(ep['raw_frames'] for ep in episodes),
              'sparse_windows_checked': len(ids), 'cache_sha256_all_verified': True,
              'source_parquet_sha256_all_verified': True,
              'source_to_cache_max_abs_errors_state_action_timestamp_frame_index': source_max.tolist(),
              'joint_action_equals_same_frame_measured_q': bool(np.array_equal(arrays['state'][:, :7], arrays['action'][:, :7])),
              'normalizer_fit_scope': 'training episodes only, limits; inferred from local training source',
              'training_state_min': low.tolist(), 'training_state_max': high.tolist(),
              'train_windows': len(train_ids), 'val_windows': len(ids)-len(train_ids),
              'full_action_offsets': list(range(-8, 10)), 'observation_offsets': list(range(-7, 1)),
              'expert_poly_offsets': list(range(-2, 10)), 'output_action_offsets': list(range(8)),
              'anchor_ext_index': 2, 'overlap_ext_index': 10, 'duration_seconds': 44/15,
              'history_initial_flow_support_offsets': list(range(-1, 7)),
              'history_support_note': 'fixed upstream maps eight past states to first eight polynomial rows as flow source; this is not an executed future trajectory',
              'expert_projection_all_windows': expert_projection(arrays['action'], ids),
              'sensor_capture_synchronization_proven': False}
    return manifest, arrays, window, report


def select_cases(manifest, arrays, live):
    result = []
    selected = set(manifest['splits']['val'])|{57, 62, 70}
    for ep in manifest['episodes']:
        if ep['episode_index'] not in selected:
            continue
        start, end = ep['start'], ep['end']
        grip = arrays['action'][start:end, 7]
        close = np.flatnonzero((grip[1:] > .5)&(grip[:-1] <= .5))+1
        if len(close) != 1:
            raise ValueError('需要唯一的首次闭爪事件以固定诊断取样规则')
        close = start+int(close[0])
        release = np.flatnonzero((grip[1:] <= .5)&(grip[:-1] > .5))+1
        phases = [('start', start), ('before_close_2', max(start, close-2)),
                  ('close', close), ('after_close_2', min(end-1, close+2))]
        if len(release): phases.append(('release', start+int(release[0])))
        for phase, current in phases:
            result.append({'name': f'ep_{ep["episode_index"]:03d}_{phase}', 'episode': ep['episode_index'],
                           'split': 'val' if ep['episode_index'] in manifest['splits']['val'] else 'train_control',
                           'phase': phase, 'current': current, 'start': start, 'end': end})
    live_q = live['agent_pos'][-1, :7]
    train_ids = np.concatenate([np.arange(ep['start'], ep['end']) for ep in manifest['episodes']
                                if ep['episode_index'] in manifest['splits']['train']])
    distance = np.max(abs(arrays['state'][train_ids, :7]-live_q), axis=1)
    nearest = int(train_ids[np.argmin(distance)])
    ep = next(ep for ep in manifest['episodes'] if ep['start'] <= nearest < ep['end'])
    result.append({'name': 'nearest_train_joint_state', 'episode': ep['episode_index'], 'split': 'train_control',
                   'phase': 'nearest_joint_state', 'current': nearest, 'start': ep['start'], 'end': ep['end']})
    return result, {'sample': nearest, 'episode': ep['episode_index'],
                    'max_joint_distance_deg': float(np.rad2deg(distance.min())),
                    'scope': 'nearest seven-joint state only; not image, task phase or history equivalence'}


def predict(opener, server, arrays, metadata, checkpoint_step):
    blob = io.BytesIO(); np.savez(blob, **arrays, metadata_json=np.asarray(json.dumps(metadata)))
    import urllib.request
    request = urllib.request.Request(server+'/predict', data=blob.getvalue(),
                                     headers={'Content-Type': 'application/octet-stream'})
    with opener.open(request, timeout=5) as response: reply = json.load(response)
    anchor = metadata['samples'][-1]['target_stamp_seconds']
    physical_reply(reply, metadata['request_id'], anchor, checkpoint_step)
    return reply


def aggregate(rows):
    summary = {}
    for split in sorted({row['split'] for row in rows}):
        members = [r for r in rows if r['split'] == split]
        summary[split] = {'cases': len(members),
            'mean_anchor_joint_mae_deg': float(np.mean([r['anchor_joint_mae_deg'] for r in members])),
            'maximum_anchor_joint_error_deg': max(r['anchor_joint_max_error_deg'] for r in members),
            'mean_chunk_joint_mae_deg': float(np.mean([r['chunk_joint_mae_deg'] for r in members])),
            'mean_gripper_anchor_mae': float(np.mean([r['gripper_anchor_abs_error'] for r in members]))}
        eligible = [r for r in members if r['shift_diagnostic_joint_mae_deg'] is not None]
        summary[split]['shift_eligible_cases'] = len(eligible)
        if eligible:
            shift = {key: float(np.mean([r['shift_diagnostic_joint_mae_deg'][key] for r in eligible]))
                     for key in ('-2', '-1', '0', '1', '2')}
            summary[split]['paired_shift_joint_mae_deg'] = shift
            summary[split]['lowest_error_shift_samples'] = min(shift, key=shift.get)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, default=Path('artifacts/flash_fr3/pickplace_74eps_stride4'))
    parser.add_argument('--dataset', type=Path, default=Path('dataset/snkdjn/quest3_franka_dualcam_pickplace_74eps'))
    parser.add_argument('--live-run', type=Path, required=True)
    parser.add_argument('--upstream-policy', type=Path, required=True)
    parser.add_argument('--server', default='http://192.168.1.113:10132')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'PENDING', 'robot_motion_commanded': False, 'gripper_motion_commanded': False,
              'scope': 'offline alignment audit, stratified diagnostic cases, not a full held-out benchmark',
              'model_and_coefficients_modified': False}
    try:
        expected = json.loads((root/'artifacts/flash_fr3/reproduction_audits/20261030_01/summary.json').read_text())
        if digest(args.upstream_policy) != expected['fetched_upstream_file_sha256']['flash_policy.py']:
            raise ValueError('审计用固定版本源码 SHA 不一致')
        report['fixed_upstream_policy_sha256'] = digest(args.upstream_policy)
        manifest, arrays, window, data_report = audit_cache(root, args.cache, args.dataset)
        report['data_and_index_audit'] = data_report; save(args.output/'data_audit.json', data_report)
        opener, health = connection(args.server); report['server'] = health
        report['remote_checkpoint_sha_independently_verified'] = False
        with np.load(args.live_run/'first_observation.npz', allow_pickle=False) as archive:
            live = {key: archive[key].copy() for key in ('agent_pos', 'primary_cam', 'wrist_cam')}
            live_metadata = json.loads(str(archive['metadata_json'].item()))
        physical_history(live_metadata)
        cases, nearest = select_cases(manifest, arrays, live); report['nearest_train_state'] = nearest
        rows = []
        for sequence, case in enumerate(cases):
            indices = window(case['current'], case['start'], case['end'])
            obs_ids = indices[1:9]
            observation = {'agent_pos': arrays['state'][obs_ids].copy()}
            observation.update({key: arrays[key][obs_ids].copy() for key in ('primary_cam', 'wrist_cam')})
            metadata = nominal_metadata(sequence)
            reply = predict(opener, args.server, observation, metadata, health['checkpoint_step'])
            save(args.output/(case['name']+'_prediction.json'), reply)
            predicted = np.asarray(reply['action'], float)
            target = np.asarray(arrays['action'][indices[8:16]], float)
            anchor_error = predicted[0, :7]-observation['agent_pos'][-1, :7]
            shift = None
            if case['current']-2 >= case['start'] and case['current']+9 < case['end']:
                shift = {str(k): float(np.rad2deg(abs(predicted[:, :7]-arrays['action'][
                    case['current']+np.arange(8)+k, :7])).mean()) for k in range(-2, 3)}
            predicted_close = np.flatnonzero(predicted[:, 7] > .5)
            true_close = np.flatnonzero(target[:, 7] > .5)
            row = {**case, 'anchor_joint_error_deg': np.rad2deg(anchor_error).tolist(),
                   'anchor_joint_mae_deg': float(np.rad2deg(abs(anchor_error)).mean()),
                   'anchor_joint_max_error_deg': float(np.rad2deg(abs(anchor_error)).max()),
                   'chunk_joint_mae_deg': float(np.rad2deg(abs(predicted[:, :7]-target[:, :7])).mean()),
                   'gripper_anchor_prediction': float(predicted[0, 7]),
                   'gripper_anchor_target': float(target[0, 7]),
                   'gripper_anchor_abs_error': float(abs(predicted[0, 7]-target[0, 7])),
                   'predicted_first_closed_index_in_chunk': int(predicted_close[0]) if len(predicted_close) else None,
                   'target_first_closed_index_in_chunk': int(true_close[0]) if len(true_close) else None,
                   'shift_diagnostic_joint_mae_deg': shift}
            rows.append(row); save(args.output/'cases.json', rows)
            print(json.dumps({'case': case['name'], 'anchor_max_deg': row['anchor_joint_max_error_deg'],
                              'closure': row['gripper_anchor_prediction']}, ensure_ascii=False), flush=True)
        replay = predict(opener, args.server, live, live_metadata, health['checkpoint_step'])
        save(args.output/'archived_live_replay_prediction.json', replay)
        original = json.loads((args.live_run/'prediction_000.json').read_text())
        delta = float(np.max(abs(np.asarray(replay['action'])-np.asarray(original['action']))))
        state = live['agent_pos'][-1]
        low, high = np.asarray(data_report['training_state_min']), np.asarray(data_report['training_state_max'])
        live_q_error = np.asarray(replay['action'])[0, :7]-state[:7]
        report['live_archive_replay'] = {
            'source': str(args.live_run), 'max_action_abs_difference_from_archived_reply': delta,
            'anchor_joint_error_deg': np.rad2deg(live_q_error).tolist(),
            'anchor_joint_max_error_deg': float(np.rad2deg(abs(live_q_error)).max()),
            'input_state': state.astype(float).tolist(),
            'outside_training_range_zero_based': np.flatnonzero((state < low)|(state > high)).tolist(),
            'maximum_past_joint_excursion_rad': float(np.ptp(live['agent_pos'][:, :7], axis=0).max()),
            'interpretation': 'archived stationary live history, no future ground truth for its unexecuted predicted motion'}
        # Verify every original saved live curve against its discrete action.
        curve_residuals = []
        for path in sorted(args.live_run.glob('prediction_*.json')):
            reply = json.loads(path.read_text())
            validate_reply(reply, reply['request_id'], reply['observation_target_stamp_seconds'], health['checkpoint_step'])
            reconstructed = np.asarray([evaluate_curve(reply['curve'], k*DT)[0] for k in range(8)])
            curve_residuals.append(float(abs(reconstructed-np.asarray(reply['action'])).max()))
        report.update(status='ALIGNMENT_AUDIT_COMPLETE', diagnostic_cases=len(rows),
                      aggregate=aggregate(rows), archived_live_action_curve_max_residual=max(curve_residuals),
                      recorded_history_only=True, static_or_frozen_history_ablation_used=False,
                      data_sensor_sync_not_established=True)
    except Exception as error:
        report.update(status='ALIGNMENT_AUDIT_FAILED', error=str(error))
        raise
    finally:
        source_files = ['scripts/audit_flash_fr3_alignment.py', 'scripts/flash_fr3_dataset.py',
                        'scripts/prepare_flash_fr3_data.py', 'scripts/train_flash_fr3.py',
                        'scripts/infer_flash_fr3_observation.py', 'scripts/serve_flash_fr3_shadow.py',
                        'scripts/flash_fr3_shadow_math.py', 'scripts/flash_fr3_retimed_buffer.py']
        report['source_sha256'] = {name: digest(root/name) for name in source_files}
        save(args.output/'report.json', report)
        print('FLASH_ALIGNMENT_AUDIT='+json.dumps({key: report[key] for key in (
            'status', 'robot_motion_commanded', 'gripper_motion_commanded')}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
