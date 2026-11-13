"""Same-GPU synchronized get_action timing on a frozen raw-frequency observation stream."""
import argparse,copy,csv,gc,hashlib,json,random,time
from pathlib import Path
import dill,numpy as np,torch,zarr
from omegaconf import OmegaConf,open_dict
from roboverse_learn.il.runners.default_runner import DefaultRunner
from roboverse_learn.il.runners.default_eval_runner import DefaultEvalRunner
from metasim.task.registry import get_task_class

p=argparse.ArgumentParser()
p.add_argument('--root',required=True)
a=p.parse_args()
root=Path(a.root);out=root/'comparison_20261028/results/latency'
assert not out.exists();out.mkdir(parents=True)
task='libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate'
z=zarr.open(str(root/f'FLASH-Policy/data_policy/{task}FrankaL0_obs:joint_pos_act:joint_pos_ds1_40.zarr'),'r')
ends=z['meta/episode_ends'][:]
start,end=int(ends[-2]),int(ends[-1])
rgb=np.moveaxis(z['data/head_camera'][start:end],1,-1).copy()
q=z['data/state'][start:end].copy()
observations=[{'rgb':torch.from_numpy(x[None]),'joint_qpos':torch.from_numpy(y[None])} for x,y in zip(rgb,q,strict=True)]
OmegaConf.register_new_resolver('eval',eval,replace=True)
rows=[];specs={}
# Reverse order on the second round to expose rather than conceal ordering effects.
for round_id,policies in enumerate([['flash','fm_dit'],['fm_dit','flash']]):
    for policy in policies:
        run='flash_bowl_s42_v1_20261028' if policy=='flash' else 'fm_dit_bowl_s42_v1_20261028'
        checkpoint=root/f'FLASH-Policy/il_outputs/{policy}/{task}/{run}/checkpoints/10000.ckpt'
        payload=torch.load(checkpoint,pickle_module=dill,map_location='cpu')
        cfg=copy.deepcopy(payload['cfg']);del payload
        with open_dict(cfg):
            cfg.train_enable=False;cfg.eval_enable=True
            cfg.train_config.training_params.device=0
            cfg.eval_config.eval_args.gpu_id=0
        OmegaConf.resolve(cfg)
        default=DefaultRunner(cfg,output_dir=str(out))
        scenario=get_task_class(task).scenario.update(robots=['franka'],simulator='mujoco',num_envs=1,headless=True)
        runner=DefaultEvalRunner(default,scenario=scenario,num_envs=1,checkpoint_path=checkpoint,device='cuda:0',
                                 task_name=task,downsample_ratio=4 if policy=='flash' else 1,send_vel_target=policy=='flash')
        assert runner.policy.num_inference_steps==(1 if policy=='flash' else 10)
        calls=steps=0
        while calls<10:
            runner.get_action(observations[steps%len(observations)])
            calls+=int(runner.last_call_was_inference);steps+=1
        torch.cuda.synchronize()
        runner.reset();random.seed(42);np.random.seed(42);torch.manual_seed(42);torch.cuda.manual_seed_all(42)
        chunk=runner.policy_cfg.action_config.action_chunk_steps
        specs[policy]={'actual_inference_steps':runner.policy.num_inference_steps,'chunk_control_steps':chunk,
                       'warmup_actual_calls_each_round':10,'measured_control_steps_each_round':3200}
        for i in range(3200):
            observation=observations[i%len(observations)]
            torch.cuda.synchronize();t=time.perf_counter_ns()
            runner.get_action(observation)
            torch.cuda.synchronize();ms=(time.perf_counter_ns()-t)/1e6
            rows.append({'round':round_id,'policy':policy,'control_step':i,'inference':runner.last_call_was_inference,'elapsed_ms':ms})
        del runner,default,cfg
        gc.collect();torch.cuda.empty_cache()
with (out/'calls.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def stats(x):
    x=np.asarray(x);return {'n':len(x),'mean_ms':float(x.mean()),'median_ms':float(np.median(x)),'p90_ms':float(np.quantile(x,.9)),'min_ms':float(x.min()),'max_ms':float(x.max())}
result={'gpu':torch.cuda.get_device_name(0),'pytorch':torch.__version__,'cuda':torch.version.cuda,
        'source_reference_index':49,'source_raw_frames':len(q),'input_rgb_sha256':hashlib.sha256(rgb.tobytes()).hexdigest(),
        'input_state_sha256':hashlib.sha256(q.tobytes()).hexdigest(),
        'measurement':'CUDA synchronized wall time of official DefaultEvalRunner.get_action, including CPU-to-GPU observation transfer, normalization, observation history, model forward on actual inference calls, action resampling and action dictionary conversion; excludes simulator/render/reset/model loading. Frozen expert observations, not a closed-loop score.',
        'control_dt_seconds':.015,'specs':specs,
        'policies':{policy:{'all_control_calls':stats([r['elapsed_ms'] for r in rows if r['policy']==policy]),
                           'actual_inference_calls':stats([r['elapsed_ms'] for r in rows if r['policy']==policy and r['inference']]),
                           'cache_calls':stats([r['elapsed_ms'] for r in rows if r['policy']==policy and not r['inference']]),
                           'rounds':{str(j):stats([r['elapsed_ms'] for r in rows if r['policy']==policy and r['round']==j and r['inference']]) for j in range(2)}} for policy in ['flash','fm_dit']}}
(out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
