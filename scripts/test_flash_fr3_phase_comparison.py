"""Phase statistics, checkpoint orchestration and busy-GPU rejection checks.

Prescribed arrays/process outputs only. No training, GPU or robot execution.
"""
import copy
import hashlib
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

from flash_fr3_phase_metrics import expected_samples, pareto_frontier, phase_metrics
from flash_fr3_cache_identity import CACHE_FILES, verify_cache_identity
import run_flash_fr3_phase_comparison as runner
from run_flash_fr3_phase_comparison import gpu_preflight, run_command, training_arguments, write_comparison


def fixture():
    manifest = {'sample_dt_seconds': 4/15, 'splits': {'train': [0], 'val': [1]},
                'episodes': [{'episode_index': 0, 'start': 0, 'end': 4, 'group': 'a'},
                             {'episode_index': 1, 'start': 4, 'end': 14, 'group': 'b'}]}
    state = np.zeros((14, 8), np.float32); action = state.copy()
    state[:, 0] = np.arange(14)*.01; action[:, :7] = state[:, :7]
    action[7:11, 7] = 1.; action[:4, 7] = 1.  # no cross-episode release event
    current = expected_samples(manifest)
    prediction = action[np.minimum(current[:, None]+np.arange(8), 13)].copy()
    return manifest, state, action, current, prediction


class PhaseTests(unittest.TestCase):
    def test_perfect_predictions_and_no_episode_boundary_event(self):
        m, s, a, i, p = fixture(); result = phase_metrics(p, i, s, a, m)
        self.assertEqual(result['validation_windows'], 10)
        self.assertEqual(result['all_points_including_end_padding']['joint_mae_deg'], 0.)
        self.assertEqual(result['all_points_including_end_padding']['gripper_mae'], 0.)
        self.assertEqual([e['cache_index'] for e in result['event_anchors']], [7, 11])
        self.assertEqual(result['phase']['close_anchor']['wrong_anchor_open_closed'], 0)
        self.assertEqual(result['phase']['preclose_2']['windows'], 1)

    def test_early_close_and_delayed_release_are_visible(self):
        m, s, a, i, p = fixture()
        p[1, 0, 7] = .9; p[7, 0, 7] = .9
        result = phase_metrics(p, i, s, a, m)
        self.assertEqual(result['phase']['preclose_2']['wrong_anchor_open_closed'], 1)
        self.assertEqual(result['phase']['release_anchor']['wrong_anchor_open_closed'], 1)
        self.assertEqual(result['phase']['close_anchor']['wrong_anchor_open_closed'], 0)

    def test_anchor_error_is_not_repaired(self):
        m, s, a, i, p = fixture(); p[:, 0, :7] += .01
        before = p.copy(); result = phase_metrics(p, i, s, a, m)
        self.assertAlmostEqual(result['current_anchor']['joint_p95_deg'], np.rad2deg(.01), places=5)
        np.testing.assert_array_equal(p, before)

    def test_full_window_contract_rejects_partial_replay_duplicates_and_nonfinite(self):
        m, s, a, i, p = fixture()
        for bad_i, bad_p in ((i[:-1], p[:-1]), (i[::-1], p[::-1]), (np.r_[i[:-1], i[0]], p),
                             (i, np.full_like(p, np.nan))):
            with self.assertRaises(ValueError): phase_metrics(bad_p, bad_i, s, a, m)

    def test_end_padding_and_transition_union_not_counted_twice(self):
        m, s, a, i, p = fixture(); result = phase_metrics(p, i, s, a, m)
        self.assertEqual(result['phase']['transition_union']['windows'], 9)
        self.assertEqual(result['phase']['other']['windows'], 1)
        self.assertEqual(result['all_points_including_end_padding']['gripper_mae'], 0.)

    def test_pareto_retains_tradeoffs_and_ties_without_selecting_a_policy(self):
        m, s, a, i, p = fixture(); base = phase_metrics(p, i, s, a, m)
        first = copy.deepcopy(base); first['current_anchor']['joint_p95_deg'] = 1.
        second = copy.deepcopy(base); second['all_points_including_end_padding']['joint_mae_deg'] = 1.
        worse = copy.deepcopy(first); worse['all_points_including_end_padding']['joint_mae_deg'] = 2.
        candidates = [{'candidate': str(n), 'seed': 42+n, 'checkpoint_step': 15000, 'phase_metrics': v}
                      for n, v in enumerate((first, second, worse, first))]
        self.assertEqual(pareto_frontier(candidates), [0, 1, 3])
        with tempfile.TemporaryDirectory() as temp:
            result = write_comparison(Path(temp), candidates)
            self.assertEqual(result['pareto_candidates'], ['0', '1', '3'])
            self.assertFalse(result['robot_motion_commanded'])
            self.assertTrue((Path(temp)/'comparison.csv').exists())


class OrchestrationTests(unittest.TestCase):
    def test_busy_gpu_rejected_and_exact_physical_uuid_preserved(self):
        with patch('run_flash_fr3_phase_comparison.subprocess.check_output', side_effect=[
                'GPU-abc, NVIDIA H100 PCIe\n', 'GPU-other, 5, python\n']):
            self.assertEqual(gpu_preflight(2)['uuid'], 'GPU-abc')
        with patch('run_flash_fr3_phase_comparison.subprocess.check_output', side_effect=[
                'GPU-abc, NVIDIA H100 PCIe\n', 'GPU-abc, 7, python\n']):
            with self.assertRaises(RuntimeError): gpu_preflight(2)

    def test_new_seed_inherits_existing_budget_without_resuming(self):
        baseline = {'arguments': {'seed': 42, 'steps': 15000, 'smoke_test': False, 'batch_size': 32,
                    'workers': 0, 'val_every': 250, 'val_batches': 1000000,
                    'checkpoint_every': 1250, 'cpu_threads': 4},
                    'scheduler': {'name': 'cosine', 'warmup_steps': 500, 'total_steps': 15000},
                    'checkpoint_purpose': 'offline_training_requires_real_robot_validation'}
        flags = training_arguments(baseline)
        self.assertIn('32', flags); self.assertNotIn('--resume', flags); self.assertNotIn('--seed', flags)
        baseline['scheduler']['total_steps'] = 30000
        with self.assertRaises(ValueError): training_arguments(baseline)

    def test_failed_child_is_reported_with_log(self):
        process = unittest.mock.Mock(); process.stdout = iter(['diagnostic\n']); process.wait.return_value = 3
        process.poll.return_value = 3
        with tempfile.TemporaryDirectory() as temp, patch(
                'run_flash_fr3_phase_comparison.subprocess.Popen', return_value=process):
            path = Path(temp)/'child.log'
            with self.assertRaises(RuntimeError): run_command(['fixture'], path, {})
            self.assertEqual(path.read_text(), 'diagnostic\n')

    def prescribed_experiment(self, smoke_pass):
        m, s, a, i, p = fixture(); metrics = phase_metrics(p, i, s, a, m)
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp); package=root/'package'; (package/'scripts').mkdir(parents=True)
            repro=root/'repro'; cache=repro/'data/pickplace_74eps_stride4'; cache.mkdir(parents=True)
            (cache/'manifest.json').write_text('{}')
            (package/'cache_reference_manifest.json').write_text('{}')
            old=repro/'runs/train_15k_seed42_20261022_160102'; old.mkdir(parents=True)
            config={'arguments':{'seed':42,'steps':15000,'smoke_test':False,'batch_size':32,
                'workers':0,'val_every':250,'val_batches':1000000,'checkpoint_every':1250,'cpu_threads':4},
                'scheduler':{'name':'cosine','warmup_steps':500,'total_steps':15000},
                'checkpoint_purpose':'offline_training_requires_real_robot_validation',
                'policy':{},'source':{'commit':'prescribed-fixture'}}
            (old/'config.json').write_text(json.dumps(config))
            for name in ('step_010000.pt','step_015000.pt'):(old/name).touch()
            (package/'experiment.json').write_text(json.dumps({'cache_manifest_sha256':runner.sha(cache/'manifest.json'),'policy':{}}))
            output=package/'result'; calls=[]
            def prescribed(command,log,env,training_steps=None):
                command=[str(x) for x in command];calls.append((command,env.copy()))
                log.write_text('prescribed process output; no training\n')
                if '-c' in command:return
                script=Path(command[2]).name
                def value(flag):return Path(command[command.index(flag)+1])
                if script=='eval_flash_fr3.py':
                    target=value('--output');target.mkdir(parents=True)
                    cp=value('--checkpoint');step=15000 if cp.stem=='best' else int(cp.stem.split('_')[-1])
                    (target/'report.json').write_text(json.dumps({'checkpoint_step':step}))
                elif script=='summarize_flash_fr3_phases.py':
                    target=value('--evaluation');step=json.loads((target/'report.json').read_text())['checkpoint_step']
                    (target/'phase_report.json').write_text(json.dumps({'phase_metrics':metrics,'source':config['source'],'checkpoint_step':step}))
                elif script=='train_flash_fr3.py':
                    target=value('--output');target.mkdir()
                    if '--smoke-test' in command:
                        if not smoke_pass:raise RuntimeError('prescribed smoke failure')
                        (target/'smoke_result.json').write_text('{"status":"PASS"}')
                    else:
                        self.assertEqual(training_steps,15000)
                        for name in ('step_001250.pt','step_015000.pt','best.pt'):(target/name).touch()
                else:raise AssertionError(script)
            argv=['runner','--mode','all','--reproduction-root',str(repro),'--output',str(output)]
            with patch.object(runner,'__file__',str(package/'scripts/runner.py')), \
                 patch.object(runner.sys,'argv',argv), \
                 patch.object(runner,'gpu_preflight',return_value={'physical_index':2,'uuid':'GPU-fixture','name':'NVIDIA H100'}), \
                 patch.object(runner.shutil,'disk_usage',return_value=unittest.mock.Mock(free=20*1024**3)), \
                 patch.object(runner,'verify_cache_identity',return_value={'status':'PASS','cache_files':{}}), \
                 patch.object(runner,'run_command',side_effect=prescribed):
                if smoke_pass:runner.main()
                else:
                    with self.assertRaises(RuntimeError):runner.main()
            status=json.loads((output/'run_status.json').read_text())
            self.assertTrue(all(env['CUDA_VISIBLE_DEVICES']=='GPU-fixture' for _,env in calls))
            trained=[cmd for cmd,_ in calls if 'train_flash_fr3.py' in cmd[2] and '--smoke-test' not in cmd]
            if smoke_pass:
                self.assertEqual(status['status'],'EXPERIMENT_COMPLETE');self.assertEqual(status['candidates'],5)
                self.assertEqual(len(trained),1);self.assertEqual(trained[0][trained[0].index('--seed')+1],'43')
                self.assertNotIn('--resume',trained[0]);self.assertTrue((output/'comparison.md').is_file())
            else:
                self.assertEqual(status['status'],'EXPERIMENT_FAILED');self.assertEqual(trained,[])

    def test_full_prescribed_workflow_preserves_original_and_evaluates_snapshots(self):
        self.prescribed_experiment(True)

    def test_failed_smoke_never_launches_full_training(self):
        self.prescribed_experiment(False)


class CacheIdentityTests(unittest.TestCase):
    def check_fixture(self, mutation=None, damage=None, change_after_training=False):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); cache = root/'cache'; cache.mkdir()
            for name in CACHE_FILES:
                (cache/name).write_bytes(('prescribed bytes for '+name).encode())
            manifest = {'format': 'prescribed_fixture_not_a_dataset', 'source_dataset': '/laptop/data',
                        'splits': {'train': [0, 1], 'val': [2]}, 'total_samples': 10,
                        'sample_dt_seconds': 4/15, 'source_sha256': {'example': 'abc'},
                        'cache_sha256': {name: hashlib.sha256((cache/name).read_bytes()).hexdigest()
                                         for name in CACHE_FILES}}
            reference = root/'reference.json'; reference.write_text(json.dumps(manifest, indent=2))
            remote = copy.deepcopy(manifest); remote['source_dataset'] = '/server/data'
            if mutation: mutation(remote)
            raw = json.dumps(remote, sort_keys=True).encode(); (cache/'manifest.json').write_bytes(raw)
            baseline = {'data': copy.deepcopy(remote), 'cache_manifest_sha256': hashlib.sha256(raw).hexdigest()}
            if damage: damage(cache)
            if change_after_training:
                (cache/'manifest.json').write_text(json.dumps(remote, indent=4))
            before = {p.name: p.read_bytes() for p in cache.iterdir()}
            report = verify_cache_identity(cache, reference, baseline)
            self.assertEqual(before, {p.name: p.read_bytes() for p in cache.iterdir()})
            return report

    def test_cross_machine_path_and_json_layout_allowed_but_bytes_unchanged(self):
        report = self.check_fixture()
        self.assertEqual(report['status'], 'PASS')
        self.assertNotEqual(report['actual_manifest_sha256'], report['reference_manifest_sha256'])
        self.assertEqual(report['actual_manifest_sha256'], report['training_manifest_sha256'])
        self.assertTrue(all(item['matches'] for item in report['cache_files'].values()))

    def test_same_size_but_different_episode_split_rejected(self):
        report = self.check_fixture(mutation=lambda m: m.update(splits={'train': [0, 2], 'val': [1]}))
        self.assertEqual(report['status'], 'FAIL')
        self.assertIn('splits', report['reference_content_fields_differing'])

    def test_modified_array_rejected_even_when_manifests_unchanged(self):
        report = self.check_fixture(damage=lambda c: (c/'action.npy').write_bytes(b'changed'))
        self.assertEqual(report['status'], 'FAIL')
        self.assertFalse(report['cache_files']['action.npy']['matches'])

    def test_missing_array_rejected_with_diagnostic(self):
        report = self.check_fixture(damage=lambda c: (c/'wrist_cam.npy').unlink())
        self.assertEqual(report['status'], 'FAIL')
        self.assertIn('error', report['cache_files']['wrist_cam.npy'])

    def test_changed_since_training_raw_manifest_is_not_bypassed(self):
        report = self.check_fixture(change_after_training=True)
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(report['reference_content_fields_differing'], [])
        self.assertTrue(any('原训练记录不同' in failure for failure in report['failures']))

    def test_sampling_interval_and_source_provenance_not_ignored(self):
        for mutation, field in ((lambda m: m.update(sample_dt_seconds=.5), 'sample_dt_seconds'),
                                (lambda m: m.update(source_sha256={'example': 'changed'}), 'source_sha256')):
            with self.subTest(field=field):
                report = self.check_fixture(mutation=mutation)
                self.assertEqual(report['status'], 'FAIL')
                self.assertIn(field, report['reference_content_fields_differing'])

    def test_cache_diagnosis_never_touches_gpu_and_failure_blocks_all_mode(self):
        for mode, damage in (('check-cache', False), ('check-cache', True), ('all', True)):
            with self.subTest(mode=mode, damage=damage), tempfile.TemporaryDirectory() as temp:
                root=Path(temp); package=root/'package'; (package/'scripts').mkdir(parents=True)
                repro=root/'repro'; cache=repro/'data/pickplace_74eps_stride4'; cache.mkdir(parents=True)
                for name in CACHE_FILES: (cache/name).write_bytes(b'prescribed fixture')
                manifest={'source_dataset':'/fixture','cache_sha256':{
                    name:hashlib.sha256((cache/name).read_bytes()).hexdigest() for name in CACHE_FILES}}
                raw=json.dumps(manifest).encode(); (cache/'manifest.json').write_bytes(raw)
                (package/'cache_reference_manifest.json').write_bytes(raw)
                (package/'experiment.json').write_text(json.dumps({
                    'cache_manifest_sha256':hashlib.sha256(raw).hexdigest()}))
                old=repro/'runs/train_15k_seed42_20261022_160102';old.mkdir(parents=True)
                (old/'config.json').write_text(json.dumps({'data':manifest,
                    'cache_manifest_sha256':hashlib.sha256(raw).hexdigest()}))
                if damage:(cache/'state.npy').write_bytes(b'changed')
                with patch.object(runner,'__file__',str(package/'scripts/runner.py')), \
                     patch.object(runner.sys,'argv',['runner','--mode',mode,'--reproduction-root',str(repro)]), \
                     patch.object(runner,'gpu_preflight') as gpu, patch.object(runner,'run_command') as child:
                    if damage:
                        with self.assertRaisesRegex(ValueError,'数据一致性检查失败'):runner.main()
                    else:runner.main()
                    gpu.assert_not_called();child.assert_not_called()
                reports=list((package/'runs').glob('cache_identity_*.json'))
                self.assertEqual(len(reports),1)
                self.assertEqual(json.loads(reports[0].read_text())['status'],'FAIL' if damage else 'PASS')


if __name__ == '__main__':
    unittest.main()
