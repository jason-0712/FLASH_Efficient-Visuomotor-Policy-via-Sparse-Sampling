"""Retain and audit all completedGPU0 results, separately from GPU2 curve.

Compare full2500 across devices without choosing whichever score is higher.
"""
import argparse,csv,json
from pathlib import Path
import numpy as np
ap=argparse.ArgumentParser();ap.add_argument('--root',type=Path,required=True);a=ap.parse_args();root=a.root
records=[];scores={};verified=0;smoke=0
for policy in ['flash','fm_dit']:
 gpu0=root/'supplemental-gpu0/runs/2500'/policy;gpu2=root/'runs/2500'/policy
 rows=[json.loads(x) for x in (gpu0/'rollouts.jsonl').read_text().splitlines()];refs=[json.loads(x) for x in (gpu2/'rollouts.jsonl').read_text().splitlines()];assert len(rows)==len(refs)==50
 c=json.loads((gpu0/'native-controller-and-loaded-policy.json').read_text());assert c['own_process_gpu_uuids']==['GPU-7425c8dc-71ce-0ff9-760d-a5bcb025a8ab'] and c['loaded_ema_all_checkpoint_tensors_equal']
 official,=[p for p in (gpu0/'official-style/eval').rglob('2500_*.ckpt_*') if p.is_dir()]
 for i,(row,ref) in enumerate(zip(rows,refs)):
  assert row['test_id']==ref['test_id']==i and row['checkpoint_sha256']==ref['checkpoint_sha256']
  assert row['reset_fingerprints']==ref['reset_fingerprints']
  with np.load(gpu0/'episodes'/f'{i:04d}.npz') as x:
   b,p=x['bowl_pose'],x['plate_pose'];dz=b[:,2]-p[:,2];goal=(np.linalg.norm(b[:,:2]-p[:,:2],axis=1)<.06)&(dz>0)&(dz<.03)
   np.testing.assert_array_equal(goal,x['geometric_success']);assert bool(goal.any())==row['geometric_success_once'];assert len(goal)==row['control_steps']
   assert (goal[-1] and not goal[:-1].any()) if goal.any() else len(goal)==300
   with np.load(official/'vel_diag/npz_data'/f'diag_data_demo{i}.npz') as y:
    np.testing.assert_array_equal(x['action_joint_targets'],y['pos_desired']);np.testing.assert_array_equal(x['joint_positions'],y['pos_actual'])
   if i<3:
    with np.load(root/'supplemental-gpu0/smoke/2500'/policy/'episodes'/f'{i:04d}.npz') as y:
     for k in x.files:
      if k!='get_action_latency_ms':np.testing.assert_array_equal(x[k],y[k])
    smoke+=1
  text=(official/f'{i:04d}.txt').read_text();assert f"SuccessOnce: {row['geometric_success_once']}" in text and f"Total Steps: {row['control_steps']}\n" in text
  records.append({'policy':policy,'test_id':i,'gpu0_success':row['geometric_success_once'],'gpu2_success':ref['geometric_success_once'],'same_success':row['geometric_success_once']==ref['geometric_success_once'],'gpu0_steps':row['control_steps'],'gpu2_steps':ref['control_steps'],'reset_and_rgb_bitwise_equal':True});verified+=1
 scores[policy]={'gpu0':sum(x['geometric_success_once'] for x in rows),'gpu2':sum(x['geometric_success_once'] for x in refs),'n_each':50,'success_disagreements':sum(not x['same_success'] for x in records if x['policy']==policy)}
with (root/'results/hardware-2500-paired.csv').open('w',newline='') as f:
 w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
report={'status':'PASS','supplemental_gpu0_episodes_recomputed':verified,'supplemental_smoke_full_bitwise_equal':smoke,'same_input_and_checkpoint_full2500_hardware_comparison':scores,'formal_selection_rule':'All-GPU2 per user scheduling reply; GPU0 is supplemental regardless of score. Do not select larger values.'}
(root/'results/supplemental-gpu0-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
