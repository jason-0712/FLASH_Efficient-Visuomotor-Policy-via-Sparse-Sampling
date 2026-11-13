"""Compare fixed first-three 10000-step runs across the two H100 variants.

Checks source adapters and hardware sensitivity, outside formal denominators.
Exact reset/controller agreement is required. Post-action trajectories may have
floating-point differences across GPU models; report them without filtering.
"""
import argparse,json
from pathlib import Path
import numpy as np
ap=argparse.ArgumentParser(); ap.add_argument('--root',required=True,type=Path);ap.add_argument('--reference-root',required=True,type=Path)
a=ap.parse_args(); records=[]
for policy in ['flash','fm_dit']:
 new=a.root/'bridge/10000'/policy; old=a.reference_root/'smoke'/policy
 c=json.loads((new/'native-controller-and-loaded-policy.json').read_text());d=json.loads((old/'native-controller-and-loaded-policy.json').read_text())
 assert c['loaded_ema_all_checkpoint_tensors_equal'] and c['normalizer_hashes']==d['normalizer_hashes']
 for k in ['joint_order','actuator_names','actuator_gainprm','actuator_biasprm','dof_damping','physics_dt_s','decimation','num_inference_steps','n_action_steps']:
  assert c[k]==d[k],k
 rows=[json.loads(x) for x in (new/'rollouts.jsonl').read_text().splitlines()]; refs=[json.loads(x) for x in (old/'rollouts.jsonl').read_text().splitlines()]
 assert len(rows)==len(refs)==3
 for i,(r,s) in enumerate(zip(rows,refs)):
  assert r['reset_fingerprints']==s['reset_fingerprints']
  record={'policy':policy,'test_id':i,'reset_and_rgb_bitwise_equal':True,'gpu0_success':r['geometric_success_once'],'gpu2_success':s['geometric_success_once'],'gpu0_control_steps':r['control_steps'],'gpu2_control_steps':s['control_steps'],'same_success':r['geometric_success_once']==s['geometric_success_once'],'same_length':r['control_steps']==s['control_steps'],'trajectory_differences':{}}
  with np.load(new/'episodes'/f'{i:04d}.npz') as x,np.load(old/'episodes'/f'{i:04d}.npz') as y:
   for k in x.files:
    if k=='get_action_latency_ms':continue
    n=min(len(x[k]),len(y[k]));p,q=x[k][:n],y[k][:n]
    record['trajectory_differences'][k]={'compared_prefix':n,'bitwise_equal_on_prefix':bool(np.array_equal(p,q)),'max_absolute_difference_on_prefix':float(np.max(np.abs(p.astype(float)-q.astype(float))))}
  records.append(record)
report={'status':'PASS','controls':6,'all_resets_and_controllers_equal':True,'all_successes_match':all(r['same_success'] for r in records),'all_trajectories_bitwise_equal':all(r['same_length'] and all(x['bitwise_equal_on_prefix'] for x in r['trajectory_differences'].values()) for r in records),'interpretation':'Six hardware sensitivity controls, not a hardware equivalence proof. No latency comparison across devices.','records':records}
(a.root/'results/bridge-validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
