"""Observe an extended official rollout without changing actions or the task checker.

An exact source transform disables only early success termination and inserts
observers. The original/transformed function and diff are saved for review.
"""
import argparse
import copy
import difflib
import hashlib
import inspect
import json
import random
import textwrap
import types
from pathlib import Path

import dill
import mujoco
import numpy as np
import torch
from omegaconf import OmegaConf, open_dict
from roboverse_learn.il.runners import default_runner as dr


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(4*1024*1024), b''): h.update(b)
    return h.hexdigest()


def rng_snapshot():
    return (random.getstate(), np.random.get_state(), torch.get_rng_state(), torch.cuda.get_rng_state_all())


def rng_restore(state):
    random.setstate(state[0])
    np.random.set_state(state[1])
    torch.set_rng_state(state[2])
    torch.cuda.set_rng_state_all(state[3])


class Observer:
    def __init__(self, directory, protocol, policy, checkpoint):
        self.directory = directory
        self.protocol = protocol
        self.policy = policy
        self.checkpoint = checkpoint
        self.rows = []
        (directory/'episodes').mkdir(parents=True, exist_ok=True)

    def start(self, env, index):
        self.env, self.index = env, index
        self.samples = []
        self.first_success_step = None
        self.saved_rng = None
        self.streak = self.max_streak = 0
        self.stable_first_step = None
        physics = env.handler.physics
        model = physics.model
        bowl_name = env.handler.mj_objects['akita_black_bowl'].model
        self.bowl_body = model.body(f'{bowl_name}/').id
        self.bowl_geoms, self.hand_geoms = set(), set()
        for i in range(model.ngeom):
            body = model.body(int(model.geom_bodyid[i])).name
            if body.startswith(bowl_name+'/'): self.bowl_geoms.add(i)
            if any(k in body for k in ['panda_hand', 'panda_leftfinger', 'panda_rightfinger']): self.hand_geoms.add(i)
        assert self.bowl_geoms and self.hand_geoms
        joints = env.handler._get_joint_names('franka', sort=True)
        assert joints[:2] == ['panda_finger_joint1', 'panda_finger_joint2'], joints
        if index == 0:
            native = {'joint_order': joints, 'bowl_body': model.body(self.bowl_body).name,
                      'bowl_contact_geom_ids': sorted(self.bowl_geoms), 'hand_contact_geom_ids': sorted(self.hand_geoms),
                      'hand_geom_bodies': sorted(set(model.body(int(model.geom_bodyid[i])).name for i in self.hand_geoms)),
                      'actuator_names': [model.actuator(i).name for i in range(model.nu)],
                      'actuator_gainprm': model.actuator_gainprm.tolist(),
                      'actuator_biasprm': model.actuator_biasprm.tolist(), 'dof_damping': model.dof_damping.tolist(),
                      'velocity_measurement': 'mj_objectVelocity(mjOBJ_BODY, flg_local=0): angular then linear, world-oriented, body inertial COM',
                      'mujoco_version': mujoco.__version__}
            (self.directory/'native-controller-and-contact-map.json').write_text(json.dumps(native, indent=2))

    def sample(self, obs, success, step):
        step = step + 1  # original loop increments immediately after this observer
        if bool(success[0]) and self.first_success_step is None:
            self.first_success_step = step
            self.saved_rng = rng_snapshot()
        physics = self.env.handler.physics
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(physics.model.ptr, physics.data.ptr, mujoco.mjtObj.mjOBJ_BODY,
                                self.bowl_body, velocity, 0)
        bowl = obs.objects['akita_black_bowl'].root_state[0].cpu().numpy().copy()
        plate = obs.objects['plate'].root_state[0].cpu().numpy().copy()
        q = obs.robots['franka'].joint_pos[0].cpu().numpy().copy()
        contacts = 0
        for c in physics.data.ptr.contact[:physics.data.ncon]:
            a, b = int(c.geom1), int(c.geom2)
            if (a in self.bowl_geoms and b in self.hand_geoms) or (b in self.bowl_geoms and a in self.hand_geoms): contacts += 1
        geometric = bool(success[0])
        released = bool(np.all(q[:2] >= self.protocol['each_finger_opening_min_m']) and contacts == 0)
        slow = bool(np.linalg.norm(velocity[3:]) <= self.protocol['bowl_world_linear_speed_max_m_per_s'] and
                    np.linalg.norm(velocity[:3]) <= self.protocol['bowl_world_angular_speed_max_rad_per_s'])
        qualified = geometric and released and slow
        self.streak = self.streak+1 if qualified else 0
        self.max_streak = max(self.max_streak, self.streak)
        if (self.streak-1)*self.protocol['control_dt_seconds'] >= self.protocol['consecutive_hold_seconds'] and self.stable_first_step is None:
            self.stable_first_step = step
        self.samples.append({'step': step, 'time_s': step*.015, 'bowl_pose': bowl[:7], 'plate_pose': plate[:7],
                             'bowl_com_position': physics.data.xipos[self.bowl_body].copy(),
                             'bowl_linear_velocity_world': velocity[3:].copy(), 'bowl_angular_velocity_world': velocity[:3].copy(),
                             'finger_positions': q[:2], 'bowl_hand_contact_count': contacts,
                             'geometric_success': geometric, 'released': released, 'slow': slow, 'stable_sample': qualified})

    def end(self):
        s = self.samples
        assert len(s) == self.protocol['maximum_control_steps']
        arrays = {k: np.asarray([r[k] for r in s]) for k in s[0]}
        for k, v in arrays.items(): assert np.isfinite(v).all(), k
        np.savez_compressed(self.directory/'episodes'/f'{self.index:04d}.npz', **arrays)
        released_at_goal = bool(np.any(arrays['released'] & arrays['geometric_success']))
        reason = ('stable_hold_met' if self.stable_first_step is not None else
                  'never_geometric_success' if self.first_success_step is None else
                  'never_released_at_goal' if not released_at_goal else 'insufficient_continuous_stable_hold')
        row = {'rollout_id': self.index, 'geometric_success_once': self.first_success_step is not None,
               'first_geometric_success_step': self.first_success_step,
               'stable_placement_success': self.stable_first_step is not None,
               'stable_hold_first_completed_step': self.stable_first_step,
               'longest_stable_sample_span_s': max(0, self.max_streak-1)*.015,
               'released_at_goal_once': released_at_goal, 'termination': '300_step_supplemental_horizon',
               'stable_failure_reason': reason, 'checkpoint_path': str(self.checkpoint),
               'bowl_final_linear_speed_m_s': float(np.linalg.norm(arrays['bowl_linear_velocity_world'][-1])),
               'bowl_final_angular_speed_rad_s': float(np.linalg.norm(arrays['bowl_angular_velocity_world'][-1]))}
        self.rows.append(row)
        with (self.directory/'rollouts.jsonl').open('a') as f: f.write(json.dumps(row)+'\n')
        if self.saved_rng is not None: rng_restore(self.saved_rng)


p = argparse.ArgumentParser()
p.add_argument('--checkpoint', required=True)
p.add_argument('--output', required=True)
p.add_argument('--protocol', required=True)
p.add_argument('--rollouts', type=int, default=50)
p.add_argument('--control-only', action='store_true', help='Unchanged official early-stop loop, with artifact path routing only')
a = p.parse_args()
out, checkpoint = Path(a.output), Path(a.checkpoint).resolve()
assert not out.exists(), out
out.mkdir(parents=True)
protocol = json.loads(Path(a.protocol).read_text())
assert protocol['minimum_consecutive_samples'] == 135
payload = torch.load(checkpoint, pickle_module=dill, map_location='cpu')
assert dill.loads(payload['pickles']['global_step']) == 10000
cfg = copy.deepcopy(payload['cfg'])
policy = cfg.policy_name
assert policy in ['flash', 'fm_dit']
del payload
OmegaConf.register_new_resolver('eval', eval, replace=True)
with open_dict(cfg):
    cfg.train_enable = False
    cfg.eval_enable = True
    cfg.eval_path = str(checkpoint)
    cfg.train_config.training_params.device = 0
    cfg.eval_config.eval_args.gpu_id = 0
    cfg.eval_config.eval_args.max_demo = a.rollouts
    cfg.eval_config.eval_args.max_step = 300
    cfg.eval_config.eval_args.level = 0
    cfg.eval_config.eval_args.downsample_ratio = 4 if policy == 'flash' else 1
OmegaConf.resolve(cfg)
(out/'resolved-config.yaml').write_text(OmegaConf.to_yaml(cfg, resolve=True))
(out/'protocol.json').write_text(json.dumps(protocol, indent=2))
observer = Observer(out, protocol, policy, checkpoint)
original = textwrap.dedent(inspect.getsource(dr.DefaultRunner.evaluate))
source = original
changes = [
    ('    args = self.eval_args\n', '    args = self.eval_args\n    self._output_dir = _stable_output_dir\n'),
    ('    # Capture actual PD gains for end-of-eval logging', '    self._output_dir = _stable_output_dir\n    # Capture actual PD gains for end-of-eval logging'),
    ('        policyRunner.reset()\n', '        policyRunner.reset()\n        _observer.start(env, demo_start_idx)\n'),
    ('            SuccessOnce = [SuccessOnce[i] or success[i] for i in range(num_envs)]',
     '            _observer.sample(obs, success, step)\n            SuccessOnce = [SuccessOnce[i] or success[i] for i in range(num_envs)]'),
    ('            if all(SuccessOnce):\n                break', '            if False:  # supplemental: keep policy running through step 300\n                break'),
    ('        episode_end_time = time.time()', '        _observer.end()\n        episode_end_time = time.time()'),
]
if a.control_only:
    changes = changes[:2]
# Explicit source locations; fail if the upstream function no longer matches.
for old, new in changes:
    if old == new: continue
    assert source.count(old) == 1, (old, source.count(old))
    source = source.replace(old, new, 1)
(out/'official-evaluate-original.py').write_text(original)
(out/'evaluate-instrumented.py').write_text(source)
(out/'instrumentation.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True), source.splitlines(True), fromfile='official-evaluate', tofile='instrumented-evaluate')))
namespace = dict(dr.__dict__)
namespace.update(_observer=observer, _stable_output_dir=str(out/'official-style'))
exec(compile(source, str(out/'evaluate-instrumented.py'), 'exec'), namespace)
runner = dr.DefaultRunner(cfg, output_dir=str(out/'official-style'))
runner.evaluate = types.MethodType(namespace['evaluate'], runner)
runner.evaluate(str(checkpoint))
if a.control_only:
    import sys,os
    print('OFFICIAL_EARLY_STOP_CONTROL_COMPLETE')
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)
assert len(observer.rows) == a.rollouts
summary = {'policy': policy, 'checkpoint_global_step': 10000, 'checkpoint_sha256': digest(checkpoint),
           'rollouts': len(observer.rows), 'geometric_successes_once': sum(r['geometric_success_once'] for r in observer.rows),
           'stable_placement_successes': sum(r['stable_placement_success'] for r in observer.rows),
           'protocol_id': protocol['protocol_id'], 'protocol_sha256': digest(out/'protocol.json'),
           'rng_restored_after_continuation': True, 'original_checker_unchanged': True}
(out/'summary.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
import sys, os
sys.stdout.flush()
sys.stderr.flush()
os._exit(0)
