"""Independent saved-array verification, endpoint audit, and CPU-only plots."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import numpy as np
assert os.environ.get('CUDA_VISIBLE_DEVICES') == ''
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def intervals(mask, times):
    runs=[]; start=None
    for i, ok in enumerate(mask):
        if ok and start is None: start=i
        if start is not None and (not ok or i==len(mask)-1):
            end=i if ok else i-1
            runs.append((float(times[end]-times[start]),start,end))
            start=None
    return sorted(runs,reverse=True)


def check(path,p):
    with np.load(path,allow_pickle=False) as z: x={k:z[k] for k in z.files}
    for k,v in x.items(): assert np.isfinite(v).all(),(str(path),k)
    delta=x['bowl_pose'][:,:3]-x['plate_pose'][:,:3]
    g=(np.sum(delta[:,:2]**2,axis=1)<p['xy_distance_max_m']**2)&(delta[:,2]>0)&(delta[:,2]<.03)
    opening=np.min(x['finger_positions'],axis=1)>=p['each_finger_opening_min_m']
    detached=x['bowl_hand_contact_count']==0
    v=np.linalg.norm(x['bowl_linear_velocity_world'],axis=1)
    w=np.linalg.norm(x['bowl_angular_velocity_world'],axis=1)
    slow=(v<=.02)&(w<=.2)
    qualified=g&opening&detached&slow
    for k,val in [('geometric_success',g),('released',opening&detached),('slow',slow),('stable_sample',qualified)]:
        assert np.array_equal(x[k],val),(str(path),k)
    times=x['time_s']
    assert np.allclose(np.diff(times),.015,rtol=0,atol=1e-12)
    stages={'geometry':g,'geometry_open':g&opening,'geometry_no_contact':g&detached,
            'geometry_no_contact_linear_slow':g&detached&(v<=.02),
            'geometry_no_contact_slow':g&detached&slow,'v1':qualified}
    spans={k:(intervals(b,times)[0][0] if b.any() else 0.) for k,b in stages.items()}
    return x,{'samples':len(g),'qualified_samples':int(qualified.sum()),'longest_spans_s':spans,
              'v1_success':spans['v1']>=2.-1e-12,
              'opening_removed_sensitivity_success':spans['geometry_no_contact_slow']>=2.-1e-12}


def write_csv(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--root',required=True);a=ap.parse_args()
    root=Path(a.root);o=root/'placement_audit_20261028';formal=o/'formal';out=o/'results'
    out.mkdir(exist_ok=False)
    p=json.loads((formal/'original-stable-protocol.json').read_text())
    plan=json.loads((formal/'audit-plan.json').read_text())
    rows=[json.loads(v) for v in (formal/'reference-rollouts.jsonl').read_text().splitlines()]
    checks=[];rawrows=[];plotrows=[];prefix_differences=[]
    for row in rows:
        idx=row['reference_id'];x,c=check(formal/'episodes'/f'{idx:04d}.npz',p)
        assert c['v1_success']==row['reference_plus_hold']['stable_success']
        c.update(kind='reference',reference_id=idx);checks.append(c)
        raw=row.get('training_demo')
        if raw:
            cut=raw['frames']-1
            prefix_differences.append(raw['q_replay_prefix_max_abs_difference'])
            tail_target=x['joint_targets'][row['reference_action_steps']-1,:2]
            rawrows.append({'reference_id':idx,**raw,
                'final_actual_fingers':json.dumps(raw['final_actual_fingers']),
                'final_target_fingers':json.dumps(raw['final_target_fingers']),
                'replay_prefix_exact':raw['q_replay_prefix_max_abs_difference']==0,
                'recorded_both_actual_open':bool(min(raw['final_actual_fingers'])>=.03),
                'recorded_both_targets_open':bool(min(raw['final_target_fingers'])>=.03),
                'full_reference_last_both_targets_open':bool(np.min(tail_target)>=.03),
                'full_reference_release_at_goal_once':row['full_reference']['released_at_goal_once'],
                'complete_reference_plus_hold_stable':row['reference_plus_hold']['stable_success'],
                'replayed_recorded_prefix_qualified_samples':int(x['stable_sample'][:cut].sum()),
                'replayed_recorded_prefix_stable_span_s':max([r[0] for r in intervals(x['stable_sample'][:cut],x['time_s'][:cut])] or [0.]),
                'replayed_recorded_prefix_released_at_goal_once':bool((x['geometric_success'][:cut]&x['released'][:cut]).any())})
        plotrows.append({'reference_id':idx,'in_collected_40':raw is not None,'primary_10':idx in plan['primary_reference_ids'],
                         'reference_action_steps':row['reference_action_steps'],
                         'first300_geometric':row['first_300']['geometric_once'],
                         'first300_stable':row['first_300']['stable_success'],
                         'extended_geometric':row['reference_plus_hold']['geometric_once'],
                         'extended_release':row['reference_plus_hold']['released_at_goal_once'],
                         'extended_stable':row['reference_plus_hold']['stable_success'],
                         'longest_stable_span_s':row['reference_plus_hold']['longest_stable_span_s'],
                         'opening_removed_diagnostic':c['opening_removed_sensitivity_success']})
    for path in sorted((formal/'fixtures').glob('*.npz')):
        _,c=check(path,p);c.update(kind='fixture',name=path.stem);checks.append(c)
    write_csv(out/'training-demo-endpoint-audit.csv',rawrows)
    write_csv(out/'reference-summary.csv',plotrows)
    groups={}
    for label,pop in [('all_50',plotrows),('collected_40',[r for r in plotrows if r['in_collected_40']]),
                      ('primary_10',[r for r in plotrows if r['primary_10']])]:
        groups[label]={'count':len(pop),**{key:sum(r[key] for r in pop) for key in
                       ['first300_geometric','first300_stable','extended_geometric','extended_release','extended_stable','opening_removed_diagnostic']},
                       'extension_only_stable_ids':[r['reference_id'] for r in pop if r['extended_stable'] and not r['first300_stable']],
                       'extended_stable_ids':[r['reference_id'] for r in pop if r['extended_stable']]}
    summary={'groups':groups,'raw_training_demo_endpoint':{
        'count':len(rawrows),'exact_robot_prefix_matches':sum(r['replay_prefix_exact'] for r in rawrows),
        'max_robot_prefix_abs_difference':max(prefix_differences),
        'recorded_actual_open_at_end':sum(r['recorded_both_actual_open'] for r in rawrows),
        'recorded_targets_open_at_end':sum(r['recorded_both_targets_open'] for r in rawrows),
        'full_reference_last_targets_open':sum(r['full_reference_last_both_targets_open'] for r in rawrows),
        'full_reference_release_at_goal_once':sum(r['full_reference_release_at_goal_once'] for r in rawrows),
        'recorded_prefix_release_at_goal_once_from_replay':sum(r['replayed_recorded_prefix_released_at_goal_once'] for r in rawrows),
        'recorded_prefix_stable_from_replay':sum(r['replayed_recorded_prefix_stable_span_s']>=2. for r in rawrows),
        'recorded_prefix_qualified_samples_from_replay':sum(r['replayed_recorded_prefix_qualified_samples'] for r in rawrows),
        'longest_recorded_prefix_stable_span_s':max(r['replayed_recorded_prefix_stable_span_s'] for r in rawrows),
        'post_success_frames_available_min':min(r['post_success_frames_available'] for r in rawrows),
        'post_success_frames_available_max':max(r['post_success_frames_available'] for r in rawrows),
        'raw_has_unrecorded_reference_actions':sum(r['reference_actions_remaining_at_collection_end']>0 for r in rawrows)},
        'sampling_note':'Reference replays are diagnostics with fresh physics reset; longer runs repeat last reference command, not a new expert controller. Robot prefixes do not prove exact reconstruction of all original object/contact states.',
        'sensitivity_note':'Removing the opening gate is post-hoc diagnostic only. It is not an accepted replacement success metric; no-contact alone need not establish withdrawal or adequate support.'}
    old=[]
    for policy in ['flash','fm_dit']:
        for path in sorted((root/'comparison_20261028'/f'stability-{policy}-10000'/'episodes').glob('*.npz')):
            _,c=check(path,p);c.update(kind='existing_policy',policy=policy,rollout_id=int(path.stem));old.append(c)
    assert len(old)==100,len(old)
    old_summary={policy:{'rollouts':50,'v1_successes':sum(r['v1_success'] for r in old if r['policy']==policy),
                       'opening_removed_diagnostic_successes':sum(r['opening_removed_sensitivity_success'] for r in old if r['policy']==policy),
                       'max_span_by_condition_s':{k:max(r['longest_spans_s'][k] for r in old if r['policy']==policy) for k in old[0]['longest_spans_s']}}
                 for policy in ['flash','fm_dit']}
    (out/'existing-policy-sensitivity.json').write_text(json.dumps({'summary':old_summary,'rollouts':old,
        'no_new_model_execution':True,'post_hoc_diagnostic_only':True},indent=2))
    (out/'audit-summary.json').write_text(json.dumps(summary,indent=2))
    (out/'array-verification.json').write_text(json.dumps({'new_npz_files':len(checks),'existing_npz_files':len(old),
           'new_samples':sum(r['samples'] for r in checks),'existing_samples':sum(r['samples'] for r in old),
           'all_finite_and_all_saved_flags_exact':True,'new_rollouts':checks},indent=2))
    # A scientific figure; CPU Agg only.
    fig,axes=plt.subplots(2,1,figsize=(12,6),gridspec_kw={'height_ratios':[1,1.5]},constrained_layout=True)
    ids=np.array([r['reference_id'] for r in plotrows]);span=np.array([r['longest_stable_span_s'] for r in plotrows])
    axes[0].scatter(ids,[int(r['first300_stable']) for r in plotrows],marker='s',label='First 300 steps',s=35)
    axes[0].scatter(ids,[int(r['extended_stable'])+.08 for r in plotrows],marker='x',label='Full reference + 200-step hold',s=38)
    axes[0].set(yticks=[0,1],yticklabels=['Fail','Pass'],ylim=(-.2,1.35),ylabel='Stable placement')
    axes[0].legend(loc='upper right',fontsize=8)
    axes[1].bar(ids,span,color=['#267eab' if r['in_collected_40'] else '#aaa' for r in plotrows])
    axes[1].axhline(2,color='firebrick',linestyle='--',label='2 s threshold')
    axes[1].set(xlabel='Reference initial-state ID',ylabel='Longest qualifying span (s)')
    axes[1].legend()
    fig.suptitle('CPU reference diagnostics: unchanged v1 stability conditions (not policy scores)')
    fig.savefig(out/'reference-stability.png',dpi=160);fig.savefig(out/'reference-stability.pdf');plt.close(fig)
    # Predetermined source ID 1 illustrates exactly what collection omitted.
    r=next(r for r in rows if r['reference_id']==1)
    x=np.load(formal/'episodes/0001.npz');t=x['time_s'];v=np.linalg.norm(x['bowl_linear_velocity_world'],axis=1);w=np.linalg.norm(x['bowl_angular_velocity_world'],axis=1)
    fig,axes=plt.subplots(4,1,figsize=(10,8),sharex=True,constrained_layout=True)
    delta=x['bowl_pose'][:,:3]-x['plate_pose'][:,:3]
    axes[0].plot(t,np.linalg.norm(delta[:,:2],axis=1),label='XY distance');axes[0].plot(t,delta[:,2],label='Height difference');axes[0].set_ylabel('m');axes[0].legend(fontsize=8)
    axes[1].plot(t,x['finger_positions'][:,0],label='Finger 1');axes[1].plot(t,x['finger_positions'][:,1],label='Finger 2');axes[1].axhline(.03,color='r',ls='--');axes[1].set_ylabel('Finger m');axes[1].legend(fontsize=8)
    axes[2].semilogy(t,np.maximum(v,1e-8),label='Linear m/s');axes[2].semilogy(t,np.maximum(w,1e-8),label='Angular rad/s');axes[2].axhline(.02,color='C0',ls='--');axes[2].axhline(.2,color='C1',ls='--');axes[2].set_ylabel('Speed');axes[2].legend(fontsize=8)
    axes[3].plot(t,x['bowl_hand_contact_count'],label='Hand contacts');axes[3].step(t,x['stable_sample'].astype(int),where='post',label='Qualified sample');axes[3].set(xlabel='Simulation time (s)',ylabel='Count / flag');axes[3].legend(fontsize=8)
    cutoff=(r['training_demo']['frames']-1)*.015
    for ax in axes:
        ax.axvline(cutoff,color='black',ls='--',alpha=.7)
        ax.axvline(r['reference_action_steps']*.015,color='purple',ls=':',alpha=.7)
    fig.suptitle(f'Reference ID 1: black = saved-demo end ({cutoff:.3f}s); purple = reference end; then hold')
    fig.savefig(out/'reference1-endpoint.png',dpi=160);fig.savefig(out/'reference1-endpoint.pdf');plt.close(fig)
    print(json.dumps({'audit':summary,'existing_policy_sensitivity':old_summary},indent=2))
    print('CPU_ARRAY_ANALYSIS_COMPLETE')


if __name__=='__main__':main()
