#!/usr/bin/env python3
"""Add phase metrics to an existing complete EMA offline evaluation."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from flash_fr3_phase_metrics import expected_samples, phase_metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation', type=Path, required=True)
    parser.add_argument('--cache', type=Path, required=True)
    args = parser.parse_args()
    report = json.loads((args.evaluation/'report.json').read_text())
    raw = (args.cache/'manifest.json').read_bytes(); manifest = json.loads(raw)
    if (report.get('status') != 'OFFLINE_EVALUATION_COMPLETE' or report.get('weights') != 'ema_model' or
            report.get('robot_motion') is not False or report['cache_manifest_sha256'] != hashlib.sha256(raw).hexdigest()):
        raise ValueError('需要相同缓存、完整留出集和 EMA 离线评估')
    if report.get('training_log_reproduction', {}).get('status') == 'MISMATCH':
        raise ValueError('原始评估与训练日志不一致')
    for filename in ('state.npy', 'action.npy'):
        if hashlib.sha256((args.cache/filename).read_bytes()).hexdigest() != manifest['cache_sha256'][filename]:
            raise ValueError('状态或动作缓存 SHA 不一致')
    with np.load(args.evaluation/'predictions.npz', allow_pickle=False) as data:
        prediction = data['prediction'].copy(); current = data['current_cache_index'].copy()
        target = data['target'].copy()
    state = np.load(args.cache/'state.npy', mmap_mode='r')
    action = np.load(args.cache/'action.npy', mmap_mode='r')
    if not np.array_equal(current, expected_samples(manifest)):
        raise ValueError('验证窗口集合不一致')
    expected = np.empty_like(target)
    for ep in manifest['episodes']:
        mask = (current >= ep['start'])&(current < ep['end'])
        expected[mask] = action[np.minimum(current[mask, None]+np.arange(8), ep['end']-1)]
    if target.shape != prediction.shape or not np.array_equal(expected, target):
        raise ValueError('保存的目标与缓存动作窗口不一致')
    metrics = phase_metrics(prediction, current, state, action, manifest)
    actual = metrics['all_points_including_end_padding']
    original = report['metrics']['all_points_including_end_padding']
    if abs(actual['joint_mae_deg']-original['joint_mae_deg']) > 1e-5 or abs(actual['gripper_mae']-original['gripper_mae']) > 1e-6:
        raise ValueError('阶段汇总与原始完整评估指标不一致')
    result = {'status': 'COMPLETE_PHASE_EVALUATION', 'checkpoint': report['checkpoint'],
              'checkpoint_sha256': report['checkpoint_sha256'], 'checkpoint_step': report['checkpoint_step'],
              'cache_manifest_sha256': report['cache_manifest_sha256'], 'source': report['source'],
              'weights': 'ema_model', 'robot_motion_commanded': False, 'phase_metrics': metrics,
              'phase_module_sha256': hashlib.sha256(Path(__file__).with_name('flash_fr3_phase_metrics.py').read_bytes()).hexdigest()}
    path = args.evaluation/'phase_report.json'
    # Exclusive write prevents silently replacing results from another version.
    with path.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False); stream.write('\n')
    print('FLASH_PHASE_EVAL='+json.dumps(result, ensure_ascii=False, allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
