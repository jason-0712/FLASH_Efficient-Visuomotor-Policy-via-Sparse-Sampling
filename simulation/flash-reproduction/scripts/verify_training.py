import argparse
import hashlib
import json
import math
from pathlib import Path

import dill
import torch
from omegaconf import OmegaConf


def sha256(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):
            h.update(b)
    return h.hexdigest()


p=argparse.ArgumentParser()
p.add_argument('--root',required=True)
a=p.parse_args()
root=Path(a.root)
run=root/'FLASH-Policy/il_outputs/flash/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate/flash_bowl_s42_v1_20261028'
rows=[json.loads(s) for s in (run/'logs.json.txt').read_text().splitlines() if s]
assert [r['global_step'] for r in rows]==list(range(len(rows)))
assert len(rows)>=15000
assert all(math.isfinite(r['train_loss']) and math.isfinite(r['lr']) for r in rows)
checkpoints=sorted((run/'checkpoints').glob('*.ckpt'),key=lambda x:int(x.stem))
final=checkpoints[-1]
records=[]
for f in [run/'checkpoints/10000.ckpt',final]:
    payload=torch.load(f,pickle_module=dill,map_location='cpu')
    step=dill.loads(payload['pickles']['global_step'])
    cfg=payload['cfg']
    assert step==int(f.stem)
    assert cfg.policy_name=='flash'
    assert cfg.train_config.training_params.seed==42
    assert cfg.train_config.training_params.num_steps==15000
    assert cfg.train_config.dataloader.batch_size==32
    assert list(cfg.shape_meta.action.shape)==[9]
    assert 'model' in payload['state_dicts'] and 'ema_model' in payload['state_dicts']
    if f==final:
        assert step==len(rows)
        (root/'configs/formal-checkpoint-config-resolved.yaml').write_text(OmegaConf.to_yaml(cfg,resolve=True))
    records.append({'path':str(f),'global_step':step,'state_dicts':list(payload['state_dicts'])})
    del payload
hashes={f.name:{'bytes':f.stat().st_size,'sha256':sha256(f)} for f in checkpoints}
(root/'manifests/checkpoints.json').write_text(json.dumps({'run_dir':str(run),'files':hashes,'verified_payloads':records},indent=2))
last=rows[-1]
span=last['elapsed_seconds']-rows[-501]['elapsed_seconds']
summary={'run_dir':str(run),'requested_steps':15000,'actual_updates':len(rows),
         'last_global_step_logged':last['global_step'],'final_checkpoint_step':int(final.stem),
         'all_training_losses_finite':True,'training_loop_elapsed_seconds':last['elapsed_seconds'],
         'last_500_updates_seconds':span,'first_loss':rows[0]['train_loss'],'last_loss':last['train_loss'],
         'last_100_losses_mean':sum(r['train_loss'] for r in rows[-100:])/100,
         'last_learning_rate':last['lr'],'evaluation_checkpoint':records[0],
         'scope':'one task, one training seed; not a success-rate evaluation'}
(root/'results/training-summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
