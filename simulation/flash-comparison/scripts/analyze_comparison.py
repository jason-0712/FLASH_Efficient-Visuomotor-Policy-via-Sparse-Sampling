"""Recompute scores from per-rollout records and immutable supplemental arrays."""
import argparse,csv,hashlib,json,math,shutil
from collections import Counter
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

p=argparse.ArgumentParser();p.add_argument('--root',required=True);a=p.parse_args()
root=Path(a.root);out=root/'comparison_20261028';results=out/'results'
def write_csv(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def wilson(k,n):
    z=1.959963984540054;p=k/n;d=1+z*z/n
    c=(p+z*z/(2*n))/d;r=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d
    return [c-r,c+r]
steps=[2500,5000,7500,10000];policies=['flash','fm_dit'];all_rows=[];summaries=[];paired=[]
for step in steps:
    sets={}
    for policy in policies:
        folder=results/f'official/{policy}-{step}'
        summary=json.loads((folder/'summary.json').read_text())
        rows=[json.loads(s) for s in (folder/'rollouts.jsonl').read_text().splitlines()]
        assert len(rows)==50 and [r['rollout_id'] for r in rows]==list(range(50))
        assert sum(r['success'] for r in rows)==summary['successes']
        summary['descriptive_wilson_95']=wilson(summary['successes'],50)
        summaries.append(summary);sets[policy]=rows;all_rows.extend(rows)
    for f,m in zip(sets['flash'],sets['fm_dit'],strict=True):
        assert f['initial_state_sha256']==m['initial_state_sha256']
        paired.append({'checkpoint_step':step,'rollout_id':f['rollout_id'],'initial_state_sha256':f['initial_state_sha256'],
                       'scene_group':f['scene_group'],'flash_success':f['success'],'fm_dit_success':m['success'],
                       'outcome':'both_success' if f['success'] and m['success'] else 'flash_only' if f['success'] else 'fm_dit_only' if m['success'] else 'both_failure'})
write_csv(results/'convergence.csv',[{'policy':s['policy'],'checkpoint_step':s['checkpoint_step'],'successes':s['successes'],'rollouts':50,'success_rate':s['success_rate'],
          'descriptive_wilson_95_low':s['descriptive_wilson_95'][0],'descriptive_wilson_95_high':s['descriptive_wilson_95'][1]} for s in summaries])
write_csv(results/'all-official-rollouts.csv',all_rows);write_csv(results/'paired-outcomes.csv',paired)
(results/'convergence.json').write_text(json.dumps(summaries,indent=2))
pair_summary={str(step):dict(Counter(r['outcome'] for r in paired if r['checkpoint_step']==step)) for step in steps}
(results/'paired-summary.json').write_text(json.dumps(pair_summary,indent=2))
plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'savefig.dpi':160})
fig,ax=plt.subplots(figsize=(8,4.8))
for policy,color,label in [('flash','#2665ac','FLASH: ds4, 1 inference step'),('fm_dit','#db741e','FM-DiT: ds1, 10 inference steps')]:
    s=[x for x in summaries if x['policy']==policy]
    ax.plot(steps,[x['success_rate']*100 for x in s],'o-',color=color,label=label,lw=2)
    for x in s: ax.annotate(f"{x['successes']}/50",(x['checkpoint_step'],100*x['success_rate']),xytext=(0,8 if policy=='flash' else -17),textcoords='offset points',ha='center',color=color,fontsize=9)
ax.set(xlabel='Training updates (checkpoint)',ylabel='Original geometric success (%)',xticks=steps,ylim=(-3,108),title='Pick-Place Bowl: closed-loop convergence, training seed 42')
ax.legend(loc='lower right');ax.grid(alpha=.2)
fig.text(.5,.015,'Same 50 reference initial states; evaluated on GPU 2. Training scenes overlap evaluation.',ha='center',fontsize=9)
fig.tight_layout(rect=[0,.035,1,1]);fig.savefig(results/'convergence.png');fig.savefig(results/'convergence.pdf');plt.close(fig)
protocol=json.loads((out/'configs/stable-placement-protocol.json').read_text())
stable_summaries={};stable_rows=[];audits={};controllers={}
for policy in policies:
    folder=out/f'stability-{policy}-10000'
    saved=json.loads((folder/'summary.json').read_text())
    rows=[json.loads(s) for s in (folder/'rollouts.jsonl').read_text().splitlines()]
    assert len(rows)==50
    controllers[policy]=json.loads((folder/'native-controller-and-contact-map.json').read_text())
    for row in rows:
        i=row['rollout_id']
        with np.load(folder/f'episodes/{i:04d}.npz') as d:
            assert all(np.isfinite(d[k]).all() for k in d.files)
            assert len(d['step'])==300
            np.testing.assert_array_equal(d['step'],np.arange(1,301))
            xy=np.linalg.norm(d['bowl_pose'][:,:2]-d['plate_pose'][:,:2],axis=1)
            dz=d['bowl_pose'][:,2]-d['plate_pose'][:,2]
            geometric=(xy<protocol['xy_distance_max_m'])&(dz>protocol['height_difference_min_m_exclusive'])&(dz<protocol['height_difference_max_m_exclusive'])
            released=(d['finger_positions']>=protocol['each_finger_opening_min_m']).all(axis=1)&(d['bowl_hand_contact_count']==0)
            speed=np.linalg.norm(d['bowl_linear_velocity_world'],axis=1);angular=np.linalg.norm(d['bowl_angular_velocity_world'],axis=1)
            slow=(speed<=protocol['bowl_world_linear_speed_max_m_per_s'])&(angular<=protocol['bowl_world_angular_speed_max_rad_per_s'])
            qualified=geometric&released&slow
            for k,x in [('geometric_success',geometric),('released',released),('slow',slow),('stable_sample',qualified)]: np.testing.assert_array_equal(d[k],x)
            streak=longest=0;first=None
            for j,x in enumerate(qualified):
                streak=streak+1 if x else 0;longest=max(longest,streak)
                if (streak-1)*.015>=2 and first is None:first=j+1
            assert row['stable_hold_first_completed_step']==first
            assert row['stable_placement_success']==(first is not None)
            assert abs(row['longest_stable_sample_span_s']-max(0,longest-1)*.015)<1e-12
            indices=np.flatnonzero(geometric)
            assert row['first_geometric_success_step']==(int(indices[0]+1) if len(indices) else None)
            row.update(policy=policy,checkpoint_step=10000,geometric_at_final_step=bool(geometric[-1]),
                       geometric_samples=int(geometric.sum()),released_at_goal_samples=int((geometric&released).sum()),
                       stable_qualified_samples=int(qualified.sum()),max_linear_speed_m_s=float(speed.max()),max_angular_speed_rad_s=float(angular.max()))
            stable_rows.append(row)
    enriched={'policy':policy,'rollouts':50,'original_early_stop_successes':next(s['successes'] for s in summaries if s['policy']==policy and s['checkpoint_step']==10000),
              'continued_geometric_successes':sum(r['geometric_success_once'] for r in rows),'stable_placement_successes':sum(r['stable_placement_success'] for r in rows),
              'released_at_goal_once_count':sum(r['released_at_goal_once'] for r in rows),'failure_reasons':dict(Counter(r['stable_failure_reason'] for r in rows)),
              'final_linear_speed_median_m_s':float(np.median([r['bowl_final_linear_speed_m_s'] for r in rows])),
              'final_angular_speed_median_rad_s':float(np.median([r['bowl_final_angular_speed_rad_s'] for r in rows])),
              'longest_stable_sample_span_s':max(r['longest_stable_sample_span_s'] for r in rows),'checkpoint_sha256':saved['checkpoint_sha256']}
    assert enriched['continued_geometric_successes']==saved['geometric_successes_once']
    assert enriched['stable_placement_successes']==saved['stable_placement_successes']
    stable_summaries[policy]=enriched;audits[policy]={'episodes_recomputed':50,'samples_per_episode':300,'all_saved_flags_exactly_recomputed':True}
controller_check={key:controllers['flash'][key]==controllers['fm_dit'][key] for key in ['joint_order','actuator_names','actuator_gainprm','actuator_biasprm','dof_damping']}
assert all(controller_check.values()),controller_check
(results/'native-controller-comparison.json').write_text(json.dumps({'equal':controller_check,'native_parameters':controllers['flash']},indent=2))
write_csv(results/'stability-rollouts.csv',stable_rows)
(results/'stability-summary.json').write_text(json.dumps(stable_summaries,indent=2))
(results/'stability-recomputation-audit.json').write_text(json.dumps(audits,indent=2))
fig,ax=plt.subplots(figsize=(8,4.5));x=np.arange(2);width=.24
for offset,key,label,color in [(-width,'original_early_stop_successes','Original early stop (GPU 2)','#8db6d9'),(0,'continued_geometric_successes','Continued: geometric once (GPU 1)','#507aa1'),(width,'stable_placement_successes','Released and stable 2 s (GPU 1)','#b56b48')]:
    values=[stable_summaries[p][key]*2 for p in policies];bars=ax.bar(x+offset,values,width,label=label,color=color)
    ax.bar_label(bars,labels=[f'{int(v/2)}/50' for v in values],padding=3,fontsize=9)
ax.set(xticks=x,xticklabels=['FLASH','FM-DiT'],ylim=(0,115),ylabel='Rollout success (%)',title='10,000 updates: geometry and complete stable placement')
ax.legend(loc='upper right',fontsize=8);fig.tight_layout();fig.savefig(results/'stability-comparison.png');fig.savefig(results/'stability-comparison.pdf');plt.close(fig)
# Representative trajectories are preselected by scene ID, not by favorable score.
fig,axes=plt.subplots(5,2,figsize=(11,11),sharex=True)
for col,policy in enumerate(policies):
    with np.load(out/f'stability-{policy}-10000/episodes/0000.npz') as d:
        t=d['time_s'];speed=np.linalg.norm(d['bowl_linear_velocity_world'],axis=1);angular=np.linalg.norm(d['bowl_angular_velocity_world'],axis=1)
        axes[0,col].plot(t,np.linalg.norm(d['bowl_pose'][:,:2]-d['plate_pose'][:,:2],axis=1),label='XY distance')
        axes[0,col].plot(t,d['bowl_pose'][:,2]-d['plate_pose'][:,2],label='Height difference');axes[0,col].axhline(.06,ls=':',color='gray');axes[0,col].axhline(.03,ls='--',color='gray')
        axes[1,col].plot(t,d['finger_positions'][:,0],label='Finger 1');axes[1,col].plot(t,d['finger_positions'][:,1],label='Finger 2');axes[1,col].axhline(.03,ls='--',color='gray')
        axes[2,col].plot(t,speed,label='Linear speed');axes[2,col].axhline(.02,ls='--',color='gray',label='0.02 m/s limit')
        axes[3,col].plot(t,angular,label='Angular speed');axes[3,col].axhline(.2,ls='--',color='gray',label='0.2 rad/s limit')
        axes[4,col].step(t,d['bowl_hand_contact_count'],where='post',label='Bowl-hand contacts')
        axes[4,col].plot(t,d['geometric_success']*5,label='Geometry (x5)',alpha=.7);axes[4,col].plot(t,d['stable_sample']*5,label='Stable sample (x5)')
        axes[0,col].set_title(f'{policy}: fixed reference scene 0')
        for ax in axes[:,col]:ax.legend(fontsize=7,loc='upper right');ax.grid(alpha=.15)
        axes[4,col].set_xlabel('Simulation time after reset (s)')
for ax,ylabel in zip(axes[:,0],['Distance (m)','Finger position (m)','Linear speed (m/s)','Angular speed (rad/s)','Contact count / flag']):ax.set_ylabel(ylabel)
fig.tight_layout();fig.savefig(results/'scene0-stability-traces.png');fig.savefig(results/'scene0-stability-traces.pdf');plt.close(fig)
videos=out/'videos';videos.mkdir(exist_ok=True)
for policy in policies:
    d=next((out/f'stability-{policy}-10000/official-style/eval').rglob('10000_*.ckpt_*'))
    shutil.copy2(d/'0.mp4',videos/f'{policy}-10000-scene0-continued-300steps.mp4')
    official=results/f'official/{policy}-10000'
    rows=[r for r in all_rows if r['policy']==policy and r['checkpoint_step']==10000]
    for status in [True,False]:
        matches=[r for r in rows if r['success']==status]
        if matches:
            r=matches[0];i=r['rollout_id'];suffix='success' if status else 'failure'
            shutil.copy2(official/f'raw/{i}.mp4',videos/f'{policy}-10000-scene{i}-{suffix}.mp4')
print(json.dumps({'convergence':[(s['policy'],s['checkpoint_step'],s['successes']) for s in summaries],'paired':pair_summary,'stability':stable_summaries,'controller_equal':controller_check},indent=2))
