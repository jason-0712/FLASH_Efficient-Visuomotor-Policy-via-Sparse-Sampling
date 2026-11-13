"""Aggregate six verified new runs with the immutable completed10000 point."""
import argparse,csv,hashlib,json
from pathlib import Path
import numpy as np
ap=argparse.ArgumentParser();ap.add_argument('--root',required=True,type=Path);ap.add_argument('--reference-root',required=True,type=Path)
a=ap.parse_args();root=a.root;old=a.reference_root
read=lambda p:json.loads(p.read_text())
plan=read(root/'configs/frozen-evaluation-plan.json')
assert hashlib.sha256((old/'SHA256SUMS').read_bytes()).hexdigest()==plan['reference_package_sha256sums_sha256']
points=[];paired=[];allrows=[];checks=[];controllers=[];resetcomparisons=0
ref=old/'runs/flash/resets'
for step in [2500,5000,7500,10000]:
 reused=step==10000
 resultdir=old/'results' if reused else root/'results/by-step'/str(step)
 s=read(resultdir/'summary.json');v=read(resultdir/'verification.json');assert v['status']=='PASS'
 assert s['test_initials_sha256']==plan['test_initials_sha256']
 points.append({'checkpoint_step':step,'provenance':'reused_heldout10000' if reused else 'new_heldout_evaluation','physical_gpu_index':2,**s})
 checks.append({'checkpoint_step':step,'provenance':'prior_verified' if reused else 'newly_verified',**v})
 for policy in ['flash','fm_dit']:
  run=old/'runs'/policy if reused else root/'runs'/str(step)/policy
  controllers.append(read(run/'native-controller-and-loaded-policy.json'))
  for i in range(50):
   with np.load(run/'resets'/f'{i:04d}.npz') as x,np.load(ref/f'{i:04d}.npz') as y:
    for k in x.files: np.testing.assert_array_equal(x[k],y[k],err_msg=f'cross-checkpoint reset {step}/{policy}/{i}/{k}')
   resetcomparisons+=1
 for name,target in [('all-rollouts.csv',allrows),('paired-outcomes.csv',paired)]:
  with (resultdir/name).open() as f:
   for r in csv.DictReader(f):target.append({**r,'checkpoint_step':step,'provenance':'reused_heldout10000' if reused else 'new_heldout_evaluation','physical_gpu_index':2})
for i,c in enumerate(controllers):
 assert c['normalizer_hashes']==controllers[i % 2]['normalizer_hashes']
 for k in ['joint_order','actuator_names','actuator_gainprm','actuator_biasprm','dof_damping','physics_dt_s','decimation','n_action_steps']:
  assert c[k]==controllers[0][k],k
assert len(allrows)==400 and len(paired)==200
for filename,rows in [('all-rollouts.csv',allrows),('paired-outcomes.csv',paired)]:
 with (root/'results'/filename).open('w',newline='') as f:
  fields=list(dict.fromkeys(k for r in rows for k in r));w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
result={'task':plan['task'],'test_initials_sha256':plan['test_initials_sha256'],'seed':42,'formal_rollouts_new':300,'formal_rollouts_reused':100,'curve_points':points,'stable_placement_metric_evaluated':False,'interpretation':'Single training seed, fixed robot, unseen object XY within empirical training bounds; original geometric SuccessOnce. Repeated checkpoints on the same frozen50 are correlated. Descriptive Wilson intervals concern scene sample, not training-seed variability. Native method horizons differ; not an isolated architecture causal comparison.'}
verification={'status':'PASS','new_episode_recomputations':sum(v['episodes_independently_recomputed'] for v in checks[:-1]),'previously_verified_10000_episodes':100,'cross_checkpoint_resets_and_rgb_bitwise_compared':resetcomparisons,'new_smoke_full_episodes_bitwise_equal':sum(v['smoke_full_prefix_episodes_bitwise_equal'] for v in checks[:-1]),'native_controller_arrays_equal_all_eight_checkpoints':True,'normalizer_hashes_equal_across_checkpoints_within_each_policy':True,'normalizers_shared_between_methods':False,'by_step':checks}
(root/'results/convergence.json').write_text(json.dumps(result,indent=2)+'\n');(root/'results/verification.json').write_text(json.dumps(verification,indent=2)+'\n')
normalizer_audit={'status':'PASS','within_method_all_four_checkpoints_identical':True,'cross_method_different_keys':[k for k in controllers[0]['normalizer_hashes'] if controllers[0]['normalizer_hashes'][k]!=controllers[1]['normalizer_hashes'].get(k)],'cross_method_equality_not_required':True,'all_loaded_parameters_verified_against_own_checkpoint':True,'initial_aggregate_check_failure':'An overly strict cross-method normalizer-equality assertion failed. Corrected to within-method across-checkpoint stability; original failure log preserved. No policy or input changes and no evaluation rerun needed.'}
(root/'results/normalizer-audit.json').write_text(json.dumps(normalizer_audit,indent=2)+'\n')
print(json.dumps({'scores':[{ 'step':r['checkpoint_step'],**{p:r['policies'][p]['successes'] for p in ['flash','fm_dit']},'pairs':r['paired_counts']} for r in points],'verification':{k:v for k,v in verification.items() if k!='by_step'}},indent=2))
