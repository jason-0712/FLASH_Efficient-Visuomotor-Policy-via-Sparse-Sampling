"""Strict physical-source metadata for retimed observation histories.

Model time and real camera/state time are separate fields. Physical source
stamps are never relabelled to pretend that slowed motion happened in real time.
"""
import json

import numpy as np

from flash_fr3_history_clock import DT, ExecutedHistoryClock

OBSERVATION_FORMAT = 'flash_fr3_retimed_observation_v1'
KEYS = ('arm', 'hand', 'primary_cam', 'wrist_cam')


def validate_retimed_observation(arrays, metadata):
    if metadata.get('format') != OBSERVATION_FORMAT:
        raise ValueError('需要重定时观测格式')
    if metadata.get('joint_order') != [f'fr3_joint{i}' for i in range(1, 8)]:
        raise ValueError('关节顺序不一致')
    if (metadata.get('n_obs_steps') != 8 or metadata.get('color') != 'RGB'
            or metadata.get('sample_dt_seconds') != DT or set(metadata['topics']) != set(KEYS)):
        raise ValueError('重定时观测配置不一致')
    state = arrays['agent_pos']
    if state.shape != (8, 8) or state.dtype != np.float32 or not np.isfinite(state).all():
        raise ValueError('状态必须为有限 float32 [8,8]')
    for key in ('primary_cam', 'wrist_cam'):
        if arrays[key].shape != (8, 256, 256, 3) or arrays[key].dtype != np.uint8:
            raise ValueError('图像必须为 uint8 [8,256,256,3]')
    clock = ExecutedHistoryClock.restore(metadata['history_clock'])
    samples = metadata['samples']
    if len(samples) != 8:
        raise ValueError('历史帧数错误')
    policy, physical = clock.targets(samples[-1]['target_monotonic'])
    relative = np.asarray([s['relative_seconds'] for s in samples], float)
    actual_policy = np.asarray([s['policy_time_seconds'] for s in samples], float)
    actual_physical = np.asarray([s['target_monotonic'] for s in samples], float)
    offset = float(metadata['wall_minus_monotonic_seconds'])
    targets = np.asarray([s['target_stamp_seconds'] for s in samples], float)
    if (not np.isfinite(np.r_[relative, actual_policy, actual_physical, offset, targets]).all()
            or not np.allclose(relative, np.arange(8)*DT, atol=1e-8, rtol=0)
            or not np.allclose(actual_policy, policy, atol=1e-8, rtol=0)
            or not np.allclose(actual_physical, physical, atol=1e-7, rtol=0)
            or not np.allclose(targets-offset, physical, atol=1e-5, rtol=0)):
        raise ValueError('模型时间网格与真实执行区间不一致')
    all_stamps = []
    for key in KEYS:
        stamps = np.asarray([s['stamp_seconds'][key] for s in samples], float)
        ages = np.asarray([s['source_age_at_receipt_seconds'][key] for s in samples], float)
        if (not np.isfinite(np.r_[stamps, ages]).all() or np.any(np.diff(stamps) <= 0)
                or np.max(abs(stamps-targets)) > .080001
                or np.any(ages < -.05) or np.any(ages > .3)):
            raise ValueError('真实传感器时间戳重复、未覆盖历史或到达时过期: '+key)
        all_stamps.append(stamps)
    if np.max(np.ptp(np.asarray(all_stamps), axis=0)) > .150001:
        raise ValueError('真实传感器时间偏差超过 0.15 秒')
    return arrays, metadata


def read_retimed_observation(path):
    with np.load(path, allow_pickle=False) as data:
        metadata = json.loads(str(data['metadata_json'].item()))
        arrays = {key: data[key].copy() for key in ('agent_pos', 'primary_cam', 'wrist_cam')}
    return validate_retimed_observation(arrays, metadata)
