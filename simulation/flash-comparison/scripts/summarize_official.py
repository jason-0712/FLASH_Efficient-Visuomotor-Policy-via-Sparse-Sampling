import argparse
import csv
import hashlib
import json
import shutil
from pathlib import Path

import cv2
import dill
import numpy as np
import torch
from omegaconf import OmegaConf


def sha256(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''): h.update(b)
    return h.hexdigest()


p=argparse.ArgumentParser()
p.add_argument('--root',required=True)
p.add_argument('--policy',choices=['flash','fm_dit'],required=True)
p.add_argument('--step',type=int,required=True)
p.add_argument('--eval-root')
p.add_argument('--log',required=True)
a=p.parse_args()
root=Path(a.root)
out=root/'comparison_20261028'
task='libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate'
run_name='flash_bowl_s42_v1_20261028' if a.policy=='flash' else 'fm_dit_bowl_s42_v1_20261028'
run=root/f'FLASH-Policy/il_outputs/{a.policy}/{task}/{run_name}'
evaluation_root=Path(a.eval_root) if a.eval_root else run
dirs=list((evaluation_root/'eval').rglob(f'{a.step}_*.ckpt_*'))
assert len(dirs)==1,dirs
d=dirs[0]
checkpoint=run/f'checkpoints/{a.step}.ckpt'
payload=torch.load(checkpoint,pickle_module=dill,map_location='cpu')
assert dill.loads(payload['pickles']['global_step'])==a.step
cfg=payload['cfg']
assert cfg.policy_name==a.policy and cfg.train_config.training_params.seed==42
assert cfg.train_config.dataloader.batch_size==32
assert cfg.policy_config.num_inference_steps==(1 if a.policy=='flash' else 10)
target=out/f'results/official/{a.policy}-{a.step}'
target.mkdir(parents=True,exist_ok=True)
(target/'checkpoint-config.yaml').write_text(OmegaConf.to_yaml(cfg,resolve=True))
del payload
ckpt_sha=sha256(checkpoint)
log=Path(a.log).read_text()
assert f'[Checkpoint loaded] path={checkpoint} global_step={a.step} seed=42 use_ema=True' in log
assert 'Falling back to latest checkpoint' not in log and 'Run failed with error:' not in log
split=json.loads((root/'manifests/reference-split.json').read_text())['initial_states']
rows=[]
manifest=[]
raw=target/'raw'
raw.mkdir(exist_ok=True)
for i in range(50):
    f=d/f'{i:04d}.txt'
    fields=dict(line.split(':',1) for line in f.read_text().splitlines() if ':' in line)
    fields={k.strip():v.strip() for k,v in fields.items()}
    assert int(fields['Demo Index'])==i
    steps=int(fields['Total Steps'])
    assert 0<steps<=300
    assert int(fields['Actual Inferences'])==(steps+(32 if a.policy=='flash' else 8)-1)//(32 if a.policy=='flash' else 8)
    video=d/f'{i}.mp4'
    diag=d/f'vel_diag/npz_data/diag_data_demo{i}.npz'
    with np.load(diag) as data:
        for k in ['pos_desired','pos_actual']:
            assert data[k].shape==(steps,9) and np.isfinite(data[k]).all()
        for k in ['vel_desired','vel_actual']:
            if k in data: assert data[k].shape==(steps,9) and np.isfinite(data[k]).all()
    cap=cv2.VideoCapture(str(video))
    assert cap.isOpened() and int(cap.get(cv2.CAP_PROP_FRAME_COUNT))==steps
    ok,frame=cap.read()
    assert ok and frame.std()>5
    cap.release()
    for source in [f,video,diag]:
        destination=raw/source.name
        shutil.copy2(source,destination)
        manifest.append({'path':str(destination.relative_to(out)),'source_path':str(source),
                         'bytes':destination.stat().st_size,'sha256':sha256(destination)})
    state=split[i]
    rows.append({'policy':a.policy,'checkpoint_step':a.step,'rollout_id':i,
                 'initial_state_sha256':state['initial_state_sha256'],
                 'scene_group':'optimization' if state['used_for_optimization'] else 'validation' if state['used_for_validation'] else 'uncollected',
                 'success':'True' in fields['SuccessOnce'],'success_at_end':'True' in fields['SuccessEnd'],
                 'steps':steps,'actual_policy_calls':int(fields['Actual Inferences']),
                 'episode_wall_ms':float(fields['Episode Duration'].removesuffix('ms')),
                 'actual_policy_call_total_ms':float(fields['Episode Total Inference Time'].removesuffix('ms')),
                 'checkpoint_sha256':ckpt_sha,'raw_video':str((raw/video.name).relative_to(out)),
                 'raw_record':str((raw/f.name).relative_to(out))})
final={k.strip():v.strip() for k,v in (line.split(':',1) for line in (d/'00_final_stats.txt').read_text().splitlines() if ':' in line)}
k=sum(r['success'] for r in rows)
assert int(final['Total Success'])==k and int(final['Total Completed'])==50
assert float(final['Average Success Rate'])==k/50
assert sum(r['steps'] for r in rows)==int(final['Total Inference Steps'])
for f in [d/'00_final_stats.txt',d/'00_eval_config.txt']: shutil.copy2(f,target/f.name)
with (target/'rollouts.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
(target/'rollouts.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
(target/'files.json').write_text(json.dumps(manifest,indent=2))
summary={'policy':a.policy,'checkpoint_step':a.step,'seed':42,'rollouts':50,'successes':k,'success_rate':k/50,
         'checkpoint_path':str(checkpoint),'checkpoint_sha256':ckpt_sha,'evaluation_dir':str(d),
         'ds_ratio':4 if a.policy=='flash' else 1,'num_inference_steps':1 if a.policy=='flash' else 10,
         'groups':{g:{'successes':sum(r['success'] for r in rows if r['scene_group']==g),'rollouts':sum(r['scene_group']==g for r in rows)} for g in ['optimization','validation','uncollected']}}
(target/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
