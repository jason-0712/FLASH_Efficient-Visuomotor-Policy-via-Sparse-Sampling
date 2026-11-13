import argparse
import csv
import hashlib
import json
import math
import re
from pathlib import Path


def sha256(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):
            h.update(b)
    return h.hexdigest()


def wilson(k,n):
    z=1.959963984540054
    phat=k/n
    center=(phat+z*z/(2*n))/(1+z*z/n)
    half=z*math.sqrt(phat*(1-phat)/n+z*z/(4*n*n))/(1+z*z/n)
    return [max(0,center-half),min(1,center+half)]


p=argparse.ArgumentParser()
p.add_argument('--root',required=True)
a=p.parse_args()
root=Path(a.root)
run=root/'FLASH-Policy/il_outputs/flash/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate/flash_bowl_s42_v1_20261028'
dirs=list((run/'eval/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate/flash/franka').glob('10000_*.ckpt_*'))
assert len(dirs)==1,dirs
d=dirs[0]
split=json.loads((root/'manifests/reference-split.json').read_text())
ckpts=json.loads((root/'manifests/checkpoints.json').read_text())
ckpt=run/'checkpoints/10000.ckpt'
expected_sha=ckpts['files']['10000.ckpt']['sha256']
assert sha256(ckpt)==expected_sha
eval_log=(root/'logs/eval-formal-10000.log').read_text()
assert f'[Checkpoint loaded] path={ckpt.resolve()} global_step=10000 seed=42 use_ema=True' in eval_log
assert 'Falling back to latest checkpoint' not in eval_log
assert 'Run failed with error:' not in eval_log
assert '[Eval] Model rebuilt from checkpoint config successfully.' in eval_log
assert len(list(d.glob('[0-9][0-9][0-9][0-9].txt')))==50
records=[]
for idx in range(50):
    f=d/f'{idx:04d}.txt'
    s=f.read_text()
    fields={}
    for l in s.splitlines():
        if ':' in l:
            k,v=l.split(':',1)
            fields[k.strip()]=v.strip()
    assert int(fields['Demo Index'])==idx
    successful='True' in fields['SuccessOnce']
    timed_out='True' in fields['TimeOut']
    steps=int(fields['Total Steps'])
    assert 0<steps<=300
    video=d/f'{idx}.mp4'
    diag=d/'vel_diag/npz_data'/f'diag_data_demo{idx}.npz'
    assert video.is_file() and video.stat().st_size>0 and diag.is_file()
    origin=split['initial_states'][idx]
    group='optimization' if origin['used_for_optimization'] else ('validation' if origin['used_for_validation'] else 'uncollected')
    r={'rollout_id':idx,'reference_initial_state_index':idx,'initial_state_sha256':origin['initial_state_sha256'],
       'scene_group':group,'success':successful,'success_at_end':'True' in fields['SuccessEnd'],
       'timeout':timed_out,'steps':steps,'termination_reason':'success_checker' if successful else ('max_steps' if steps==300 else 'other'),
       'checkpoint_path':str(ckpt),'checkpoint_sha256':expected_sha,'checkpoint_global_step':10000,
       'episode_wall_time_ms':float(fields['Episode Duration'].removesuffix('ms')),
       'actual_policy_calls':int(fields['Actual Inferences']),
       'total_actual_policy_call_ms':float(fields['Episode Total Inference Time'].removesuffix('ms')),
       'video_path':str(video),'video_sha256':sha256(video),'raw_record_path':str(f),
       'trajectory_diagnostics_path':str(diag)}
    records.append(r)
for key in ['episode_wall_time_ms','total_actual_policy_call_ms']:
    assert all(math.isfinite(r[key]) and r[key]>=0 for r in records)
with (root/'results/rollouts.jsonl').open('w') as f:
    for r in records:
        f.write(json.dumps(r)+'\n')
with (root/'results/rollouts.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(records[0]))
    w.writeheader()
    w.writerows(records)
k=sum(r['success'] for r in records)
final_fields={}
for line in (d/'00_final_stats.txt').read_text().splitlines():
    if ':' in line:
        key,value=line.split(':',1)
        final_fields[key.strip()]=value.strip()
assert int(final_fields['Total Success'])==k
assert int(final_fields['Total Completed'])==50
assert float(final_fields['Average Success Rate'])==k/50
assert int(final_fields['Total Inference Steps'])==sum(r['steps'] for r in records)
assert int(final_fields['Total Actual Inferences'])==sum(r['actual_policy_calls'] for r in records)
groups={}
for g in ['optimization','validation','uncollected']:
    rr=[r for r in records if r['scene_group']==g]
    groups[g]={'successes':sum(r['success'] for r in rr),'rollouts':len(rr)}
summary={'task':'libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate','policy':'flash','training_seed':42,
         'checkpoint_global_step':10000,'checkpoint_sha256':expected_sha,'evaluation_dir':str(d),
         'successes':k,'rollouts':50,'success_rate':k/50,'wilson_95_interval_descriptive':wilson(k,50),
         'scene_groups':groups,'max_control_steps':300,'control_dt_seconds':0.015,'dr_level':0,
         'all_50_raw_records_and_videos_present':True,
         'protocol_notes':['Single task and single training seed.','Official 50-scene pool overlaps 40 collected demonstration initial states.',
                           'Two training demonstrations used zero post-success recording frames; see collection-deviations.json.',
                           'Official success checker tests bowl/plate positions and does not require gripper release.',
                           'Timing includes cold calls and policy preprocessing; no warmed-up network-only speed claim is made.',
                           'MuJoCo handler caches velocity targets but does not consume them in _simulate; configuration flags do not establish effective velocity feedforward.']}
(root/'results/evaluation-summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
