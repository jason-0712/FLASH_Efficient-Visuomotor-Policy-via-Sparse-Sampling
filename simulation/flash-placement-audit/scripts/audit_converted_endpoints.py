"""Read-only ds1/ds4 endpoint audit and final summary plot layout."""
import argparse,csv,json,os
from pathlib import Path
import numpy as np
import zarr
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
assert os.environ.get('CUDA_VISIBLE_DEVICES') == ''
p=argparse.ArgumentParser();p.add_argument('--root',required=True);a=p.parse_args()
root=Path(a.root);out=root/'placement_audit_20261028/results'
raw=sorted((root/'data-formal/success').glob('demo_*'));reports={}
for stride in [1,4]:
    path=root/'FLASH-Policy/data_policy'/f'libero_90.kitchen_scene1_put_the_black_bowl_on_the_plateFrankaL0_obs:joint_pos_act:joint_pos_ds{stride}_40.zarr'
    z=zarr.open(path,mode='r');ends=z['meta/episode_ends'][:];start=0;rows=[]
    for i,d in enumerate(raw):
        meta=json.loads((d/'metadata.json').read_text())
        q=np.asarray(meta['joint_qpos'],dtype=np.float32);act=np.asarray(meta['joint_qpos_target'],dtype=np.float32)
        storedq=z['data/state'][start:ends[i]];storeda=z['data/action'][start:ends[i]]
        np.testing.assert_array_equal(storedq,q[::stride]);np.testing.assert_array_equal(storeda,act[::stride])
        row={'reference_id':int(d.name.split('_')[1]),'samples':len(storedq),
             'last_raw_index_stored':(len(storedq)-1)*stride,'last_raw_index_available':len(q)-1,
             'last_both_actual_open':bool((storedq[-1,:2]>=.03).all()),
             'last_both_targets_open':bool((storeda[-1,:2]>=.03).all())}
        rows.append(row);start=int(ends[i])
    reports[f'ds{stride}']={'path':str(path),'all_state_action_arrays_exactly_match_raw_stride':True,
        'samples':start,'last_actual_open_demos':sum(r['last_both_actual_open'] for r in rows),
        'last_targets_open_demos':sum(r['last_both_targets_open'] for r in rows),'demonstrations':rows}
(out/'converted-dataset-endpoints.json').write_text(json.dumps(reports,indent=2))
with (out/'reference-summary.csv').open() as f:rows=list(csv.DictReader(f))
ids=np.array([int(r['reference_id']) for r in rows]);span=np.array([float(r['longest_stable_span_s']) for r in rows])
fig,axes=plt.subplots(2,1,figsize=(12,6),gridspec_kw={'height_ratios':[1,1.5]},constrained_layout=True)
axes[0].scatter(ids,[int(r['first300_stable']=='True') for r in rows],marker='s',label='First 300 steps',s=35)
axes[0].scatter(ids,[int(r['extended_stable']=='True')+.08 for r in rows],marker='x',label='Full reference + at least 200-step hold',s=38)
axes[0].set(yticks=[0,1],yticklabels=['Fail','Pass'],ylim=(-.2,1.3),ylabel='Stable placement')
axes[0].legend(loc='lower right',bbox_to_anchor=(1,1.01),ncol=2,fontsize=8)
axes[1].bar(ids,span,color=['#267eab' if r['in_collected_40']=='True' else '#aaa' for r in rows])
axes[1].axhline(2,color='firebrick',linestyle='--',label='2 s threshold')
axes[1].set(xlabel='Reference initial-state ID',ylabel='Longest qualifying span (s)');axes[1].legend()
fig.suptitle('CPU reference diagnostics: unchanged v1 stability conditions (not policy scores)')
fig.savefig(out/'reference-stability.png',dpi=160);fig.savefig(out/'reference-stability.pdf');plt.close(fig)
print(json.dumps({k:{key:val for key,val in v.items() if key!='demonstrations'} for k,v in reports.items()},indent=2))
print('CPU_CONVERTED_ENDPOINT_AUDIT_COMPLETE')
