"""CPU-only reference replay and physical fixtures. Never a learned-policy score."""
import argparse
import ast
import copy
import hashlib
import inspect
import json
import os
import platform
import subprocess
import time
from pathlib import Path

assert os.environ.get('CUDA_VISIBLE_DEVICES') == '', 'CUDA must be hidden before imports'
import numpy as np
import torch
import mujoco
from loguru import logger as log
from metasim.task.registry import get_task_class
from metasim.scenario.cameras import PinholeCameraCfg
from metasim.utils.demo_util import get_traj
from metasim.utils.setup_util import get_robot


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def masks(x, p):
    delta = x['bowl_pose'][:, :3] - x['plate_pose'][:, :3]
    geometric = ((np.linalg.norm(delta[:, :2], axis=1) < p['xy_distance_max_m']) &
                 (delta[:, 2] > p['height_difference_min_m_exclusive']) &
                 (delta[:, 2] < p['height_difference_max_m_exclusive']))
    released = ((x['finger_positions'] >= p['each_finger_opening_min_m']).all(axis=1) &
                (x['bowl_hand_contact_count'] == 0))
    slow = ((np.linalg.norm(x['bowl_linear_velocity_world'], axis=1) <= p['bowl_world_linear_speed_max_m_per_s']) &
            (np.linalg.norm(x['bowl_angular_velocity_world'], axis=1) <= p['bowl_world_angular_speed_max_rad_per_s']))
    return geometric, released, slow, geometric & released & slow


def longest_span(mask, dt=.015):
    best = streak = 0
    for v in mask:
        streak = streak + 1 if v else 0
        best = max(best, streak)
    return max(0, best - 1) * dt


def summarize(x, p, limit=None):
    if limit is not None:
        x = {k: v[:limit] for k, v in x.items()}
    g, r, s, qualified = masks(x, p)
    def first(a):
        ids = np.flatnonzero(a)
        return int(ids[0]+1) if len(ids) else None
    span = longest_span(qualified)
    return {'samples': len(g), 'geometric_once': bool(g.any()), 'released_at_goal_once': bool((g&r).any()),
            'stable_samples': int(qualified.sum()), 'longest_stable_span_s': span,
            'stable_success': span >= p['consecutive_hold_seconds'],
            'first_geometric_step': first(g), 'first_release_at_goal_step': first(g&r),
            'geometry_and_detached_slow_span_s': longest_span(g & s & (x['bowl_hand_contact_count'] == 0)),
            'final_geometric': bool(g[-1]), 'final_fingers_m': x['finger_positions'][-1].tolist(),
            'final_linear_speed_m_s': float(np.linalg.norm(x['bowl_linear_velocity_world'][-1])),
            'final_angular_speed_rad_s': float(np.linalg.norm(x['bowl_angular_velocity_world'][-1])),
            'geometric_samples': int(g.sum()), 'geometry_and_open_samples': int((g & (x['finger_positions']>=.03).all(axis=1)).sum()),
            'geometric_no_hand_contact_samples': int((g & (x['bowl_hand_contact_count']==0)).sum()),
            'geometric_slow_samples': int((g & s).sum())}


class NoRenderer:
    """No GL context: images are disabled and any attempt to render must fail."""
    def __init__(self, *args, **kwargs):
        self.width, self.height = kwargs.get('width'), kwargs.get('height')
    def close(self): pass
    def render(self, *args, **kwargs): raise RuntimeError('Rendering prohibited in CPU audit')
    def update_scene(self, *args, **kwargs): raise RuntimeError('Rendering prohibited in CPU audit')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--plan', required=True)
    a = ap.parse_args()
    root, out = Path(a.root), Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    (out/'episodes').mkdir()
    (out/'fixtures').mkdir()
    plan = json.loads(Path(a.plan).read_text())
    p = json.loads((root/'comparison_20261028/configs/stable-placement-protocol.json').read_text())
    (out/'original-stable-protocol.json').write_text(json.dumps(p, indent=2))
    (out/'audit-plan.json').write_text(json.dumps(plan, indent=2))
    assert not torch.cuda.is_available()
    torch.set_num_threads(2)
    np.random.seed(42)
    torch.manual_seed(42)
    info = {'start_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
            'hostname': platform.node(), 'pid': os.getpid(), 'python': platform.python_version(),
            'torch': torch.__version__, 'mujoco': mujoco.__version__,
            'CUDA_VISIBLE_DEVICES': os.environ['CUDA_VISIBLE_DEVICES'], 'torch_cuda_available': torch.cuda.is_available(),
            'renderer': 'NoRenderer stub, camera retained but data_types empty; no OpenGL or CUDA context',
            'source_commit': subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
            'script_sha256': sha(__file__), 'audit_plan_sha256': sha(a.plan),
            'physical_reset': 'mj_resetData before each env.reset; stock collector ensure_clean_state before references',
            'no_policy_checkpoint_loaded': True}
    (out/'environment.json').write_text(json.dumps(info,indent=2))
    mujoco.Renderer = NoRenderer
    task_cls = get_task_class(plan['task'])
    camera = PinholeCameraCfg(name='camera0', data_types=[], width=256, height=256,
                             pos=(2.,0.,2.), look_at=(0.,0.,0.))
    scenario = task_cls.scenario.update(robots=['franka'], simulator='mujoco', renderer='mujoco',
                                       cameras=[camera], num_envs=1, headless=True)
    env = task_cls(scenario, device='cpu')
    handler, physics = env.handler, env.handler.physics
    assert handler.device.type == 'cpu'
    m, d = physics.model.ptr, physics.data.ptr
    dt = float(m.opt.timestep * handler.decimation)
    assert abs(dt - .015) < 1e-12, dt
    robot = get_robot('franka')
    initials, actions, states = get_traj(env.traj_filepath, robot, handler)
    info['reference_sha256'] = sha(env.traj_filepath)
    info['reference_count'] = len(actions)
    info['physics_dt_s'] = float(m.opt.timestep)
    info['decimation'] = handler.decimation
    info['control_dt_s'] = dt
    info['original_checker_source'] = inspect.getsource(task_cls._terminated)
    (out/'environment.json').write_text(json.dumps(info,indent=2))
    # Extract only the stock stabilization functions, avoiding the collector CLI/main.
    collector = root/'FLASH-Policy/scripts/advanced/collect_demo.py'
    tree = ast.parse(collector.read_text())
    funcs = [n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in
             ['ensure_clean_state','_validate_state_correctness']]
    namespace = {'torch':torch,'log':log}
    exec(compile(ast.Module(body=funcs,type_ignores=[]), str(collector), 'exec'), namespace)
    (out/'collector-stabilization-source.py').write_text('\n\n'.join(ast.get_source_segment(collector.read_text(),n) for n in funcs))
    ensure_clean = namespace['ensure_clean_state']
    names = handler._get_joint_names('franka', sort=True)
    assert names[:2] == ['panda_finger_joint1','panda_finger_joint2']
    bowl_name, plate_name = (handler.mj_objects[n].model for n in ['akita_black_bowl','plate'])
    bowl_body = physics.model.body(f'{bowl_name}/').id
    bowl_geoms = {i for i in range(m.ngeom) if physics.model.body(int(m.geom_bodyid[i])).name.startswith(bowl_name+'/')}
    hand_geoms = {i for i in range(m.ngeom) if any(s in physics.model.body(int(m.geom_bodyid[i])).name for s in
                  ['panda_hand','panda_leftfinger','panda_rightfinger'])}
    contact_map = {'joint_order':names, 'bowl_body':physics.model.body(bowl_body).name,
                   'bowl_geoms':sorted(bowl_geoms),'hand_geoms':sorted(hand_geoms),
                   'geom_bodies':{str(i):physics.model.body(int(m.geom_bodyid[i])).name for i in bowl_geoms|hand_geoms},
                   'actuator_gainprm':m.actuator_gainprm.tolist(),'actuator_biasprm':m.actuator_biasprm.tolist(),
                   'dof_damping':m.dof_damping.tolist()}
    (out/'native-controller-and-contact-map.json').write_text(json.dumps(contact_map,indent=2))
    def reset(state, stabilize=False):
        mujoco.mj_resetData(m,d)
        handler._current_vel_target = None
        obs,_ = env.reset(states=[copy.deepcopy(state)])
        if stabilize:
            ensure_clean(handler)
            env._episode_steps[:] = 0
            obs = handler.get_states()
        return obs
    def sample(obs, success, step, target):
        velocity = np.zeros(6)
        mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,bowl_body,velocity,0)
        count = sum(1 for c in d.contact[:d.ncon] if
                    (int(c.geom1) in bowl_geoms and int(c.geom2) in hand_geoms) or
                    (int(c.geom2) in bowl_geoms and int(c.geom1) in hand_geoms))
        q = obs.robots['franka'].joint_pos[0].numpy().copy()
        return {'step': step,'time_s':step*dt,
                'bowl_pose':obs.objects['akita_black_bowl'].root_state[0,:7].numpy().copy(),
                'plate_pose':obs.objects['plate'].root_state[0,:7].numpy().copy(),
                'bowl_linear_velocity_world':velocity[3:].copy(), 'bowl_angular_velocity_world':velocity[:3].copy(),
                'finger_positions':q[:2], 'joint_positions':q,'joint_targets':target,
                'bowl_hand_contact_count':count,'official_geometric_success':bool(success[0]),
                'native_qpos':d.qpos.copy(),'native_qvel':d.qvel.copy()}
    def run(state, sequence, total, stabilize=False):
        initial_obs = reset(state, stabilize)
        initial_q = initial_obs.robots['franka'].joint_pos[0].numpy().copy()
        samples=[]
        for k in range(total):
            action = sequence[min(k,len(sequence)-1)]
            target = np.array([action['franka']['dof_pos_target'][n] for n in names],dtype=float)
            obs,_,success,_,_ = env.step([action])
            samples.append(sample(obs,success,k+1,target))
        x = {key:np.asarray([s[key] for s in samples]) for key in samples[0]}
        g,r,s,q = masks(x,p)
        assert np.array_equal(g,x['official_geometric_success']), 'geometric recomputation differs'
        x.update(geometric_success=g,released=r,slow=s,stable_sample=q)
        for key, val in x.items(): assert np.isfinite(val).all(), key
        return x, initial_q
    rows=[]
    for idx in plan['reference_ids']:
        count=len(actions[idx]); total=max(300,count+plan['hold_last_action_steps'])
        x, initial_q=run(initials[idx],actions[idx],total,stabilize=True)
        np.savez_compressed(out/'episodes'/f'{idx:04d}.npz',**x)
        row={'reference_id':idx,'reference_action_steps':count,'tail_rule':'repeat unmodified last reference action',
             'extended_total_steps':total, 'first_300':summarize(x,p,300), 'full_reference':summarize(x,p,count),
             'reference_plus_hold':summarize(x,p), 'tail_only':summarize({k:v[count:] for k,v in x.items()},p)}
        raw=root/'data-formal/success'/f'demo_{idx:04d}'/'metadata.json'
        if raw.exists():
            meta=json.loads(raw.read_text()); rawq=np.asarray(meta['joint_qpos']); rawa=np.asarray(meta['joint_qpos_target'])
            nq=min(len(rawq),len(x['joint_positions'])+1)
            # Collector stores initial observation followed by post-action observations.
            aligned=np.concatenate([initial_q[None],x['joint_positions']],axis=0)[:nq]
            row['training_demo']={'frames':len(rawq),'metadata_sha256':sha(raw),
                'q_replay_prefix_max_abs_difference':float(np.max(np.abs(rawq[:nq]-aligned))),
                'final_actual_fingers':rawq[-1,:2].tolist(),'final_target_fingers':rawa[-1,:2].tolist(),
                'post_success_frames_available':len(rawq)-1-(row['reference_plus_hold']['first_geometric_step'] or len(rawq)),
                'reference_actions_remaining_at_collection_end':max(0,count-(len(rawq)-1))}
        rows.append(row)
        with (out/'reference-rollouts.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
        print(json.dumps({'reference_id':idx,'steps':total,'stable_300':row['first_300']['stable_success'],
                          'stable_extended':row['reference_plus_hold']['stable_success'],
                          'geom_extended':row['reference_plus_hold']['geometric_once'],
                          'release_extended':row['reference_plus_hold']['released_at_goal_once']}),flush=True)
    # Explicitly initialized physical fixtures, not reference trajectories or policy actions.
    fixture_rows=[]
    for idx in plan['fixture_initial_reference_ids']:
        for condition in plan['fixture_conditions']:
            state=copy.deepcopy(initials[idx])
            closed=condition=='on_plate_closed_detached_hand'
            offgoal=condition=='off_goal_open_hand'
            state['objects']['akita_black_bowl']['pos']=state['objects']['plate']['pos'].clone()
            state['objects']['akita_black_bowl']['pos'][2]+=.05  # settle under gravity, no clamping afterward
            if offgoal: state['objects']['akita_black_bowl']['pos'][0]+=.2
            state['objects']['akita_black_bowl']['rot']=torch.tensor([1.,0.,0.,0.])
            joint={n:float(np.asarray(v).reshape(-1)[0]) for n,v in state['robots']['franka']['dof_pos'].items()}
            state['robots']['franka']['dof_pos']=joint
            joint['panda_finger_joint1']=0.0 if closed else .04
            joint['panda_finger_joint2']=0.0 if closed else .04
            seq=[{'franka':{'dof_pos_target':copy.deepcopy(joint)}}]
            x,_=run(state,seq,300)
            key=f'{idx:04d}-{condition}'
            np.savez_compressed(out/'fixtures'/f'{key}.npz',**x)
            row={'fixture':key,'reference_base_id':idx,'condition':condition,
                 'is_expert_demonstration':False,'is_policy_rollout':False,**summarize(x,p)}
            fixture_rows.append(row)
            print(json.dumps(row),flush=True)
    (out/'fixtures.json').write_text(json.dumps(fixture_rows,indent=2))
    summary={'reference_rollouts':len(rows),'original_300_stable':sum(r['first_300']['stable_success'] for r in rows),
             'extended_stable':sum(r['reference_plus_hold']['stable_success'] for r in rows),
             'primary_10_reference_ids':plan['primary_reference_ids'],
             'primary_10_extended_stable':sum(r['reference_plus_hold']['stable_success'] for r in rows if r['reference_id'] in plan['primary_reference_ids']),
             'collected_demo_count':sum('training_demo' in r for r in rows),
             'fixtures':fixture_rows,'all_reference_results_are_diagnostics_not_learned_policy_scores':True}
    (out/'summary.json').write_text(json.dumps(summary,indent=2))
    env.close()
    info['end_utc']=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
    info['torch_cuda_initialized']=torch.cuda.is_initialized()
    assert not info['torch_cuda_initialized']
    (out/'environment.json').write_text(json.dumps(info,indent=2))
    print('CPU_REFERENCE_AUDIT_COMPLETE',flush=True)


if __name__=='__main__': main()
