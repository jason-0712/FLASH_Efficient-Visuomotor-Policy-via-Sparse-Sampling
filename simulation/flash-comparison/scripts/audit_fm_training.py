import hashlib,json,math,shutil,sys
from pathlib import Path
import dill,numpy as np,torch
from omegaconf import OmegaConf

root=Path(sys.argv[1]);out=root/'comparison_20261028'
task='libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate'
run=root/f'FLASH-Policy/il_outputs/fm_dit/{task}/fm_dit_bowl_s42_v1_20261028'
receipt=json.loads((out/'results/fm-training-exit.json').read_text());assert receipt['exit_code']==0
rows=[json.loads(s) for s in (run/'logs.json.txt').read_text().splitlines()]
assert len(rows)==15015 and [r['global_step'] for r in rows]==list(range(15015))
assert all(math.isfinite(r['train_loss']) for r in rows)
payload=torch.load(run/'checkpoints/15015.ckpt',pickle_module=dill,map_location='cpu')
assert dill.loads(payload['pickles']['global_step'])==15015
cfg=payload['cfg'];assert cfg.policy_name=='fm_dit'
assert cfg.train_config.training_params.seed==42 and cfg.train_config.dataloader.batch_size==32
assert cfg.train_config.training_params.num_steps==15000 and cfg.policy_config.num_inference_steps==10
assert cfg.dataset_config.downsample_ratio==1 if 'downsample_ratio' in cfg.dataset_config else 'ds1_40.zarr' in cfg.dataset_config.zarr_path
(out/'configs/fm-final-checkpoint-config.yaml').write_text(OmegaConf.to_yaml(cfg,resolve=True))
del payload
checkpoints=[]
for step in [2500,5000,7500,10000,15015]:
    file=run/f'checkpoints/{step}.ckpt';h=hashlib.sha256()
    with file.open('rb') as f:
        for block in iter(lambda:f.read(4*1024*1024),b''):h.update(block)
    checkpoints.append({'step':step,'path':str(file),'bytes':file.stat().st_size,'sha256':h.hexdigest()})
summary={'policy':'fm_dit','run_dir':str(run),'requested_steps':15000,'actual_updates':len(rows),
         'last_global_step_logged':rows[-1]['global_step'],'final_checkpoint_step':15015,'all_training_losses_finite':True,
         'training_loop_elapsed_seconds':rows[-1]['elapsed_seconds'],'last_500_updates_seconds':rows[-1]['elapsed_seconds']-rows[-501]['elapsed_seconds'],
         'first_loss':rows[0]['train_loss'],'last_loss':rows[-1]['train_loss'],'last_100_losses_mean':float(np.mean([r['train_loss'] for r in rows[-100:]])),
         'last_learning_rate':rows[-1].get('lr'),'checkpoints':checkpoints,'training_seed':42,'batch_size':32,'inference_steps':10}
(out/'results/fm-training-summary.json').write_text(json.dumps(summary,indent=2))
shutil.copy2(run/'logs.json.txt',out/'logs/fm-updates.jsonl')
dataset=Path(cfg.dataset_config.zarr_path)
if not dataset.is_absolute():dataset=root/'FLASH-Policy'/dataset
assert dataset.exists()
manifest=[]
for file in sorted(p for p in dataset.rglob('*') if p.is_file()):
    h=hashlib.sha256(file.read_bytes()).hexdigest()
    manifest.append({'path':str(file.relative_to(dataset)),'bytes':file.stat().st_size,'sha256':h})
(out/'manifests/ds1-dataset-files.json').write_text(json.dumps({'root':str(dataset),'files':manifest},indent=2))
print(json.dumps(summary,indent=2))
