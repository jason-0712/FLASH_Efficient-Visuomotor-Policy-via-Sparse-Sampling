"""Independently verify both completed runs and export paired results."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def read_json(path):
    return json.loads(path.read_text())


def wilson(k, n):
    z = 1.959963984540054
    center = (k / n + z*z/(2*n)) / (1 + z*z/n)
    half = z * math.sqrt(k/n * (1-k/n)/n + z*z/(4*n*n)) / (1 + z*z/n)
    return [center - half, center + half]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', required=True, type=Path)
    ap.add_argument('--frozen-root', required=True, type=Path)
    args = ap.parse_args()
    root, frozen = args.root, args.frozen_root
    plan = read_json(root / 'configs/frozen-evaluation-plan.json')
    state_file = frozen / plan['test_initials_relative_path']
    assert hashlib.sha256(state_file.read_bytes()).hexdigest() == plan['test_initials_sha256']
    states = read_json(state_file)['states']
    assert read_json(root / 'results/smoke-validation.json')['status'] == 'PASS'
    rows, summaries, controllers = {}, {}, {}
    verified = {'status': 'PASS', 'episodes_independently_recomputed': 0,
                'paired_resets_and_initial_rgb_bitwise_equal': 0, 'smoke_full_prefix_episodes_bitwise_equal': 0,
                'actual_resets_independently_compared_with_frozen_cpu_trace': 0,
                'native_controller_arrays_equal': True, 'all_loaded_ema_tensors_and_normalizers_verified': True,
                'all_episode_videos_and_official_records_present': True,
                'gpu_uuid': 'GPU-eb1c016b-9505-66b6-0362-57dea67de4eb'}
    for policy in ('flash', 'fm_dit'):
        run = root / 'runs' / policy
        summary = read_json(run / 'summary.json')
        assert summary['rollouts'] == 50
        summaries[policy] = summary
        controllers[policy] = read_json(run / 'native-controller-and-loaded-policy.json')
        assert controllers[policy]['loaded_ema_all_checkpoint_tensors_equal']
        assert controllers[policy]['own_process_gpu_uuids'] == [verified['gpu_uuid']]
        assert controllers[policy]['num_inference_steps'] == (1 if policy == 'flash' else 10)
        raw = [json.loads(line) for line in (run / 'rollouts.jsonl').read_text().splitlines()]
        assert len(raw) == 50
        rows[policy] = raw
        directories = [p for p in (run / 'official-style/eval').rglob('10000_*.ckpt_*') if p.is_dir()]
        assert len(directories) == 1
        official = directories[0]
        for index, row in enumerate(raw):
            assert row['test_id'] == index and row['initial_state_sha256'] == states[index]['state_sha256']
            assert row['reset_matches_frozen_cpu_trace']
            with np.load(run / 'resets' / f'{index:04d}.npz', allow_pickle=False) as reset:
                source = frozen / 'frozen-v1/candidate-traces' / f"{states[index]['candidate_id']:04d}.npz"
                with np.load(source, allow_pickle=False) as original:
                    for key, oldkey in [('native_qpos', 'native_qpos'), ('native_qvel', 'native_qvel'),
                                        ('joint_positions', 'joint_positions'), ('bowl_pose', 'akita_black_bowl_pose'), ('plate_pose', 'plate_pose')]:
                        np.testing.assert_array_equal(reset[key], original[oldkey][0])
                for key in reset.files:
                    assert hashlib.sha256(np.ascontiguousarray(reset[key]).tobytes()).hexdigest() == row['reset_fingerprints'][key + '_sha256']
            verified['actual_resets_independently_compared_with_frozen_cpu_trace'] += 1
            with np.load(run / 'episodes' / f'{index:04d}.npz', allow_pickle=False) as episode:
                assert all(np.isfinite(episode[k]).all() for k in episode.files)
                bowl, plate = episode['bowl_pose'], episode['plate_pose']
                dz = bowl[:, 2] - plate[:, 2]
                success = (np.linalg.norm(bowl[:, :2] - plate[:, :2], axis=1) < .06) & (dz > 0) & (dz < .03)
                np.testing.assert_array_equal(success, episode['geometric_success'])
                assert bool(success.any()) == row['geometric_success_once']
                assert len(success) == row['control_steps']
                assert (success[-1] and not success[:-1].any()) if success.any() else len(success) == 300
                np.testing.assert_array_equal(episode['step'], np.arange(1, len(success) + 1))
                np.testing.assert_array_equal(episode['time_s'], episode['step'] * .015)
                assert int(episode['model_call'].sum()) == row['actual_model_calls']
                with np.load(official / 'vel_diag/npz_data' / f'diag_data_demo{index}.npz') as diagnostics:
                    np.testing.assert_array_equal(episode['action_joint_targets'], diagnostics['pos_desired'])
                    np.testing.assert_array_equal(episode['joint_positions'], diagnostics['pos_actual'])
                if index < 3:
                    with np.load(root / 'smoke' / policy / 'episodes' / f'{index:04d}.npz') as smoke:
                        for key in episode.files:
                            if key != 'get_action_latency_ms':
                                np.testing.assert_array_equal(episode[key], smoke[key], err_msg=f'smoke/full {policy}/{index}/{key}')
                    verified['smoke_full_prefix_episodes_bitwise_equal'] += 1
                row['last_bowl_linear_speed_m_s'] = float(np.linalg.norm(episode['bowl_linear_velocity_world'][-1]))
                row['last_bowl_angular_speed_rad_s'] = float(np.linalg.norm(episode['bowl_angular_velocity_world'][-1]))
            record = (official / f'{index:04d}.txt').read_text()
            assert f"SuccessOnce: {row['geometric_success_once']}" in record
            assert f"Total Steps: {row['control_steps']}\n" in record
            assert (official / f'{index}.mp4').stat().st_size > 1000
            verified['episodes_independently_recomputed'] += 1
        assert sum(r['geometric_success_once'] for r in raw) == summary['successes']
        summary['descriptive_wilson_95'] = wilson(summary['successes'], 50)
        summary['actual_model_calls_total'] = sum(row['actual_model_calls'] for row in raw)
        summary['control_steps_total'] = sum(row['control_steps'] for row in raw)
        summary['mean_control_steps'] = summary['control_steps_total'] / 50
        summary['num_inference_steps'] = 1 if policy == 'flash' else 10
        summary['ds_ratio'] = 4 if policy == 'flash' else 1
    for key in ['joint_order', 'actuator_names', 'actuator_gainprm', 'actuator_biasprm', 'dof_damping', 'physics_dt_s', 'decimation']:
        assert controllers['flash'][key] == controllers['fm_dit'][key], key
    pairs = []
    counts = {'both_success': 0, 'flash_only_success': 0, 'fm_dit_only_success': 0, 'both_fail': 0}
    for index in range(50):
        a, b = rows['flash'][index], rows['fm_dit'][index]
        assert a['reset_fingerprints'] == b['reset_fingerprints']
        with np.load(root / 'runs/flash/resets' / f'{index:04d}.npz') as x, np.load(root / 'runs/fm_dit/resets' / f'{index:04d}.npz') as y:
            for key in x.files:
                np.testing.assert_array_equal(x[key], y[key])
        verified['paired_resets_and_initial_rgb_bitwise_equal'] += 1
        sa, sb = a['geometric_success_once'], b['geometric_success_once']
        category = 'both_success' if sa and sb else 'flash_only_success' if sa else 'fm_dit_only_success' if sb else 'both_fail'
        counts[category] += 1
        pairs.append({'test_id': index, 'candidate_id': states[index]['candidate_id'],
                      'initial_state_sha256': states[index]['state_sha256'], 'paired_actual_reset_equal': True,
                      'flash_success': sa, 'fm_dit_success': sb, 'category': category,
                      'flash_steps': a['control_steps'], 'fm_dit_steps': b['control_steps']})
    with (root / 'results/paired-outcomes.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(pairs[0]))
        writer.writeheader(); writer.writerows(pairs)
    flat_rows = [{k: v for k, v in row.items() if k != 'reset_fingerprints'} for policy in ('flash', 'fm_dit') for row in rows[policy]]
    with (root / 'results/all-rollouts.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(flat_rows[0]))
        writer.writeheader(); writer.writerows(flat_rows)
    result = {'task': plan['task'], 'test_initials_sha256': plan['test_initials_sha256'],
              'policies': summaries, 'paired_counts': counts,
              'success_rate_difference_percentage_points': 100 * (summaries['flash']['success_rate'] - summaries['fm_dit']['success_rate']),
              'scope': 'Single training seed; fixed robot, unseen object XY inside training empirical bounds; fresh native reset for both. Separate from original reference-50 results.',
              'stable_placement_metric_evaluated': False,
              'interval_interpretation': 'Descriptive binomial intervals within this sample; not training-seed uncertainty, cross-task evidence, or a causal architecture comparison.'}
    (root / 'results/summary.json').write_text(json.dumps(result, indent=2) + '\n')
    (root / 'results/verification.json').write_text(json.dumps(verified, indent=2) + '\n')
    print(json.dumps(result, indent=2)); print(json.dumps(verified, indent=2))


if __name__ == '__main__':
    main()
