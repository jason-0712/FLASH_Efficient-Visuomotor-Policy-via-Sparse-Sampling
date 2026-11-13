"""Run official early-stop evaluation on frozen inputs with reviewed source hooks.

The repository is not edited. Exact source substitutions change input loading,
native reset, output routing, and observation only; action/checker logic stays.
"""
import argparse
import copy
import datetime
import difflib
import hashlib
import inspect
import json
import os
from pathlib import Path
import platform
import subprocess
import textwrap
import time
import types
import xml.etree.ElementTree as ET

import dill
import imageio.v2 as imageio
import mujoco
import numpy as np
import torch
from omegaconf import OmegaConf, open_dict
from roboverse_learn.il.runners import default_runner as dr
from load_frozen_initials import load_frozen_initials

GPU_UUIDS = {0: 'GPU-7425c8dc-71ce-0ff9-760d-a5bcb025a8ab', 2: 'GPU-eb1c016b-9505-66b6-0362-57dea67de4eb'}
GPU_UUID = None


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def array_hash(values):
    return hashlib.sha256(np.ascontiguousarray(values).tobytes()).hexdigest()


def numpy(tensor):
    return tensor.detach().cpu().numpy().copy()


def nested(flat):
    state = copy.deepcopy(flat)
    for entity in state.values():
        for key in ('pos', 'rot'):
            entity[key] = torch.tensor(entity[key], dtype=torch.float64)
        if 'dof_pos' in entity:
            entity['dof_pos'] = {k: float(np.asarray(v).reshape(-1)[0]) for k, v in entity['dof_pos'].items()}
    return {'robots': {'franka': state.pop('franka')}, 'objects': state}


class Recorder:
    def __init__(self, out, states, initials, checkpoint, checkpoint_sha, policy, expected_weights, started):
        self.out, self.states, self.initials = out, states, initials
        self.checkpoint, self.checkpoint_sha, self.policy = checkpoint, checkpoint_sha, policy
        self.expected_weights, self.started, self.rows = expected_weights, started, []
        for name in ('episodes', 'resets'):
            (out / name).mkdir()

    def trajectory(self, env):
        assert len(self.states) in (3, 50)
        return [nested(row['initial_state']) for row in self.states], None, None

    def reset_native(self, env, index):
        mujoco.mj_resetData(env.handler.physics.model.ptr, env.handler.physics.data.ptr)
        env.handler._current_vel_target = None

    def start(self, env, obs, index, policy_runner):
        self.env, self.index, self.samples = env, index, []
        self.episode_started = time.monotonic()
        self.physics = env.handler.physics
        self.joints = env.handler._get_joint_names('franka', sort=True)
        assert len(self.joints) == 9
        self.bowl_body = self.physics.model.body(f"{env.handler.mj_objects['akita_black_bowl'].model}/").id
        self.bowl_geoms, self.hand_geoms = set(), set()
        for i in range(self.physics.model.ngeom):
            body = self.physics.model.body(int(self.physics.model.geom_bodyid[i])).name
            if body.startswith(env.handler.mj_objects['akita_black_bowl'].model + '/'):
                self.bowl_geoms.add(i)
            if any(n in body for n in ('panda_hand', 'panda_leftfinger', 'panda_rightfinger')):
                self.hand_geoms.add(i)
        self.policy_runner = policy_runner
        self.reset_arrays = {
            'native_qpos': self.physics.data.ptr.qpos.copy(),
            'native_qvel': self.physics.data.ptr.qvel.copy(),
            'joint_positions': numpy(obs.robots['franka'].joint_pos[0]),
            'bowl_pose': numpy(obs.objects['akita_black_bowl'].root_state[0, :7]),
            'plate_pose': numpy(obs.objects['plate'].root_state[0, :7]),
        }
        row = self.states[index]
        with np.load(self.initials.parent / 'candidate-traces' / f"{row['candidate_id']:04d}.npz", allow_pickle=False) as frozen:
            for key, oldkey in [('native_qpos', 'native_qpos'), ('native_qvel', 'native_qvel'),
                                ('joint_positions', 'joint_positions'), ('bowl_pose', 'akita_black_bowl_pose'), ('plate_pose', 'plate_pose')]:
                np.testing.assert_array_equal(self.reset_arrays[key], frozen[oldkey][0], err_msg=f'reset mismatch {index}/{key}')
        self.reset_arrays['rgb'] = numpy(obs.cameras['camera0'].rgb[0])
        np.savez_compressed(self.out / 'resets' / f'{index:04d}.npz', **self.reset_arrays)
        imageio.imwrite(self.out / 'resets' / f'{index:04d}.png', self.reset_arrays['rgb'])
        self.reset_fingerprints = {key + '_sha256': array_hash(val) for key, val in self.reset_arrays.items()}
        if index == 0:
            assert policy_runner.policy.num_inference_steps == (1 if self.policy == 'flash' else 10)
            actual = policy_runner.policy.state_dict()
            for key, value in self.expected_weights.items():
                assert key in actual and torch.equal(actual[key].detach().cpu(), value), f'EMA state mismatch: {key}'
            normalizer_keys = [k for k in self.expected_weights if k.startswith('normalizer.')]
            assert normalizer_keys
            xml = ET.fromstring(subprocess.check_output(['nvidia-smi', '-q', '-x'], text=True))
            own_gpu_uuids = sorted({g.findtext('uuid') for g in xml.findall('gpu')
                                    for process in g.findall('./processes/process_info')
                                    if process.findtext('pid') == str(os.getpid())})
            assert own_gpu_uuids == [GPU_UUID], own_gpu_uuids
            controller = {'joint_order': self.joints,
                'actuator_names': [self.physics.model.actuator(i).name for i in range(self.physics.model.nu)],
                'actuator_gainprm': self.physics.model.actuator_gainprm.tolist(),
                'actuator_biasprm': self.physics.model.actuator_biasprm.tolist(),
                'dof_damping': self.physics.model.dof_damping.tolist(),
                'physics_dt_s': float(self.physics.model.ptr.opt.timestep), 'decimation': env.handler.decimation,
                'bowl_contact_geometry_ids': sorted(self.bowl_geoms), 'hand_contact_geometry_ids': sorted(self.hand_geoms),
                'loaded_ema_all_checkpoint_tensors_equal': True, 'loaded_ema_tensor_count': len(self.expected_weights),
                'normalizer_tensor_count': len(normalizer_keys), 'own_process_gpu_uuids': own_gpu_uuids,
                'n_action_steps': policy_runner.policy.n_action_steps,
                'num_inference_steps': policy_runner.policy.num_inference_steps,
                'normalizer_hashes': {k: array_hash(numpy(actual[k])) for k in normalizer_keys}}
            assert abs(controller['physics_dt_s'] * controller['decimation'] - .015) < 1e-12
            (self.out / 'native-controller-and-loaded-policy.json').write_text(json.dumps(controller, indent=2) + '\n')
            self.expected_weights = None

    def sample(self, obs, success, step, action, latency_ms):
        body = self.bowl_body
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(self.physics.model.ptr, self.physics.data.ptr, mujoco.mjtObj.mjOBJ_BODY, body, velocity, 0)
        contacts = 0
        for contact in self.physics.data.ptr.contact[:self.physics.data.ncon]:
            a, b = int(contact.geom1), int(contact.geom2)
            if (a in self.bowl_geoms and b in self.hand_geoms) or (b in self.bowl_geoms and a in self.hand_geoms):
                contacts += 1
        targets = action[0]['franka']
        qtargets = np.asarray([float(np.asarray(numpy(v) if torch.is_tensor(v) else v).reshape(-1)[0])
                               for name in self.joints for v in [targets['dof_pos_target'][name]]])
        self.samples.append({
            'step': step + 1, 'time_s': (step + 1) * .015,
            'bowl_pose': numpy(obs.objects['akita_black_bowl'].root_state[0, :7]),
            'plate_pose': numpy(obs.objects['plate'].root_state[0, :7]),
            'joint_positions': numpy(obs.robots['franka'].joint_pos[0]),
            'native_qpos': self.physics.data.ptr.qpos.copy(), 'native_qvel': self.physics.data.ptr.qvel.copy(),
            'action_joint_targets': qtargets,
            'bowl_linear_velocity_world': velocity[3:].copy(), 'bowl_angular_velocity_world': velocity[:3].copy(),
            'bowl_hand_contact_count': contacts,
            'geometric_success': bool(success[0]), 'model_call': bool(self.policy_runner.last_call_was_inference),
            'get_action_latency_ms': latency_ms,
        })

    def end(self):
        arrays = {key: np.asarray([sample[key] for sample in self.samples]) for key in self.samples[0]}
        assert all(np.isfinite(value).all() for value in arrays.values())
        bowl, plate = arrays['bowl_pose'], arrays['plate_pose']
        dz = bowl[:, 2] - plate[:, 2]
        formula = (np.linalg.norm(bowl[:, :2] - plate[:, :2], axis=1) < .06) & (dz > 0) & (dz < .03)
        np.testing.assert_array_equal(formula, arrays['geometric_success'])
        success = bool(formula.any())
        assert (success and formula[-1] and not formula[:-1].any()) or (not success and len(formula) == 300)
        np.savez_compressed(self.out / 'episodes' / f'{self.index:04d}.npz', **arrays)
        source = self.states[self.index]
        row = {'test_id': self.index, 'candidate_id': source['candidate_id'], 'initial_state_sha256': source['state_sha256'],
               'policy': self.policy, 'checkpoint_step': int(self.checkpoint.stem), 'checkpoint_sha256': self.checkpoint_sha,
               'geometric_success_once': success, 'first_success_step': len(formula) if success else None,
               'control_steps': len(formula), 'episode_simulated_seconds': len(formula) * .015,
               'termination': 'first_geometric_success' if success else '300_step_limit',
               'actual_model_calls': int(arrays['model_call'].sum()), 'reset_matches_frozen_cpu_trace': True,
               'reset_fingerprints': self.reset_fingerprints,
               'episode_wall_seconds_with_observer': time.monotonic() - self.episode_started}
        self.rows.append(row)
        with (self.out / 'rollouts.jsonl').open('a') as f:
            f.write(json.dumps(row) + '\n')
        print('HELDOUT_ROLLOUT ' + json.dumps(row), flush=True)

    def finish(self):
        assert len(self.rows) == len(self.states)
        summary = {'policy': self.policy, 'checkpoint_step': int(self.checkpoint.stem), 'seed': 42, 'checkpoint_sha256': self.checkpoint_sha,
                   'test_initials_sha256': digest(self.initials), 'rollouts': len(self.rows),
                   'successes': sum(row['geometric_success_once'] for row in self.rows),
                   'success_rate': sum(row['geometric_success_once'] for row in self.rows) / len(self.rows),
                   'primary_metric': 'original geometric SuccessOnce, official early stop',
                   'stable_placement_metric_evaluated': False,
                   'all_resets_match_frozen_cpu_traces': True, 'raw_observations_and_actions_saved': True,
                   'wall_seconds_through_final_artifacts': time.monotonic() - self.started,
                   'completed_utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}
        (self.out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print('HELDOUT_COMPLETE ' + json.dumps(summary), flush=True)


def main():
    global GPU_UUID
    started = time.monotonic()
    ap = argparse.ArgumentParser()
    ap.add_argument('--policy', choices=['flash', 'fm_dit'], required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--frozen-root', type=Path, required=True)
    ap.add_argument('--plan', type=Path, required=True)
    ap.add_argument('--rollouts', type=int, choices=[3, 50], required=True)
    ap.add_argument('--step', type=int, choices=[2500, 5000, 7500, 10000], required=True)
    ap.add_argument('--physical-gpu', type=int, choices=[0, 2], required=True)
    args = ap.parse_args()
    GPU_UUID = GPU_UUIDS[args.physical_gpu]
    assert os.environ.get('CUDA_VISIBLE_DEVICES') == GPU_UUID
    assert os.environ.get('MUJOCO_EGL_DEVICE_ID') == str(args.physical_gpu)
    assert os.environ.get('MUJOCO_GL') == 'egl'
    assert torch.cuda.device_count() == 1 and torch.cuda.is_available()
    plan = json.loads(args.plan.read_text())
    entry = next(row for row in plan['checkpoints'] if row['policy'] == args.policy and row['checkpoint_step'] == args.step)
    if args.step < 10000:
        assert entry['physical_gpu_index'] == args.physical_gpu
    checkpoint = Path(entry['checkpoint_path'])
    assert digest(checkpoint) == entry['checkpoint_sha256']
    initials = args.frozen_root / plan['test_initials_relative_path']
    states = load_frozen_initials(initials, expected_sha256=plan['test_initials_sha256'])[:args.rollouts]
    args.output.mkdir(parents=True, exist_ok=False)
    out = args.output
    payload = torch.load(checkpoint, pickle_module=dill, map_location='cpu')
    assert dill.loads(payload['pickles']['global_step']) == args.step
    cfg = copy.deepcopy(payload['cfg'])
    assert cfg.policy_name == args.policy and cfg.train_config.training_params.seed == 42
    assert cfg.train_config.training_params.use_ema
    assert cfg.policy_config.num_inference_steps == entry['num_inference_steps']
    expected_weights = payload['state_dicts']['ema_model']
    del payload
    OmegaConf.register_new_resolver('eval', eval, replace=True)
    with open_dict(cfg):
        cfg.train_enable = False
        cfg.eval_enable = True
        cfg.eval_path = str(checkpoint)
        cfg.train_config.training_params.device = 0
        cfg.eval_config.eval_args.gpu_id = 0
        cfg.eval_config.eval_args.max_demo = args.rollouts
        cfg.eval_config.eval_args.max_step = 300
        cfg.eval_config.eval_args.level = 0
        cfg.eval_config.eval_args.downsample_ratio = entry['ds_ratio']
    OmegaConf.resolve(cfg)
    (out / 'resolved-config.yaml').write_text(OmegaConf.to_yaml(cfg, resolve=True))
    environment = {'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(), 'pid': os.getpid(),
                   'hostname': platform.node(), 'python': platform.python_version(), 'numpy': np.__version__,
                   'torch': torch.__version__, 'mujoco': mujoco.__version__,
                   'CUDA_VISIBLE_DEVICES': os.environ['CUDA_VISIBLE_DEVICES'],
                   'MUJOCO_EGL_DEVICE_ID': os.environ['MUJOCO_EGL_DEVICE_ID'], 'torch_device': 0,
                   'physical_gpu_index': args.physical_gpu, 'physical_gpu_uuid': GPU_UUID,
                   'cuda_device_name': torch.cuda.get_device_name(0),
                   'script_sha256': digest(__file__), 'plan_sha256': digest(args.plan),
                   'source_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                   'source_diff_sha256': hashlib.sha256(subprocess.check_output(['git', 'diff', '--binary'])).hexdigest()}
    assert environment['source_commit'] == 'f47e490f2b820f835d1b9145da3cdb3df69f50a0'
    assert environment['source_diff_sha256'] == '56fec88a58c6475de026bc2a4bf81fb4a797679c6a7ff104676149accfb677d9'
    (out / 'environment.json').write_text(json.dumps(environment, indent=2) + '\n')
    recorder = Recorder(out, states, initials, checkpoint, entry['checkpoint_sha256'], args.policy, expected_weights, started)
    original = textwrap.dedent(inspect.getsource(dr.DefaultRunner.evaluate))
    transformed = original
    hooks = [
        ('    args = self.eval_args\n', '    args = self.eval_args\n    self._output_dir = _heldout_output\n'),
        ('    # Capture actual PD gains for end-of-eval logging', '    self._output_dir = _heldout_output\n    # Capture actual PD gains for end-of-eval logging'),
        ('    init_states, all_actions, all_states = get_traj(env.traj_filepath, robot, env.handler)',
         '    init_states, all_actions, all_states = _heldout.trajectory(env)'),
        ('        obs, extras = env.reset(states=init_states[demo_start_idx:demo_end_idx])',
         '        _heldout.reset_native(env, demo_start_idx)\n        obs, extras = env.reset(states=init_states[demo_start_idx:demo_end_idx])'),
        ('        policyRunner.reset()\n', '        policyRunner.reset()\n        _heldout.start(env, obs, demo_start_idx, policyRunner)\n'),
        ('            SuccessOnce = [SuccessOnce[i] or success[i] for i in range(num_envs)]',
         '            _heldout.sample(obs, success, step, action, inference_time_ms)\n            SuccessOnce = [SuccessOnce[i] or success[i] for i in range(num_envs)]'),
        ('        episode_end_time = time.time()', '        _heldout.end()\n        episode_end_time = time.time()'),
        ('    # env.close() often hangs', '    _heldout.finish()\n    # env.close() often hangs'),
    ]
    for old, new in hooks:
        assert transformed.count(old) == 1, (old, transformed.count(old))
        transformed = transformed.replace(old, new, 1)
    (out / 'official-evaluate-original.py').write_text(original)
    (out / 'evaluate-instrumented.py').write_text(transformed)
    (out / 'instrumentation.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True), transformed.splitlines(True), fromfile='official-evaluate', tofile='heldout-evaluate')))
    namespace = dict(dr.__dict__)
    namespace.update(_heldout=recorder, _heldout_output=str(out / 'official-style'))
    exec(compile(transformed, str(out / 'evaluate-instrumented.py'), 'exec'), namespace)
    runner = dr.DefaultRunner(cfg, output_dir=str(out / 'official-style'))
    runner.evaluate = types.MethodType(namespace['evaluate'], runner)
    runner.evaluate(str(checkpoint))
    assert (out / 'summary.json').is_file()
    import sys
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == '__main__':
    main()
