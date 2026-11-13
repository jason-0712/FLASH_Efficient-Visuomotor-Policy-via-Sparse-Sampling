#!/usr/bin/env bash
set -euxo pipefail
root=/home/hanyu/flash_fr3/flash_sim_reproduction_20261028
out="$root/comparison_20261028"
"$root/.venv/bin/python" - "$root" "$out" <<'PY'
import json,sys,numpy as np
from pathlib import Path
root,out=map(Path,sys.argv[1:])
original=out/'stability-smoke-control-flash/official-style/eval'
old=next(original.rglob('10000_*.ckpt_*'))
new=next((out/'stability-smoke-flash/official-style/eval').rglob('10000_*.ckpt_*'))
row=json.loads((out/'stability-smoke-flash/rollouts.jsonl').read_text())
with np.load(old/'vel_diag/npz_data/diag_data_demo0.npz') as a,np.load(new/'vel_diag/npz_data/diag_data_demo0.npz') as b:
    steps=len(a['pos_desired'])
    assert row['first_geometric_success_step']==steps
    errors={}
    for key in ['pos_desired','pos_actual','vel_desired','vel_actual']:
        errors[key]=float(np.max(np.abs(a[key]-b[key][:steps])))
        np.testing.assert_array_equal(a[key],b[key][:steps])
cross_gpu=next((root/'FLASH-Policy/il_outputs/flash/libero_90.kitchen_scene1_put_the_black_bowl_on_the_plate/flash_bowl_s42_v1_20261028/eval').rglob('10000_*.ckpt_*'))
with np.load(cross_gpu/'vel_diag/npz_data/diag_data_demo0.npz') as c,np.load(new/'vel_diag/npz_data/diag_data_demo0.npz') as b:
    n=len(c['pos_desired'])
    cross_gpu_result={'gpu3_original_first_success_step':n,'gpu1_first_success_step':steps,
        'prefix_max_errors':{k:float(np.max(np.abs(c[k]-b[k][:n]))) for k in ['pos_desired','pos_actual']},
        'interpretation':'Cross-GPU numerical differences accumulated in closed-loop control. This is retained separately; instrumentation is checked against an unmodified early-stop control on the same GPU.'}
with np.load(out/'stability-smoke-flash/episodes/0000.npz') as d:
    xy=np.linalg.norm(d['bowl_pose'][:,:2]-d['plate_pose'][:,:2],axis=1)
    height=d['bowl_pose'][:,2]-d['plate_pose'][:,2]
    np.testing.assert_array_equal(d['geometric_success'],(xy<.06)&(height>0)&(height<.03))
    speed=np.linalg.norm(d['bowl_linear_velocity_world'],axis=1)
    angular=np.linalg.norm(d['bowl_angular_velocity_world'],axis=1)
    result={'same_gpu_original_first_success_step':steps,'prefix_max_errors':errors,'prefix_exactly_equal':True,'cross_gpu_comparison':cross_gpu_result,'smoke_stability_result':row,
            'geometric_position_formula_matches':True,'native_linear_speed_range_m_s':[float(speed.min()),float(speed.max())],
            'native_angular_speed_range_rad_s':[float(angular.min()),float(angular.max())],
            'finger_position_ranges_m':[[float(d['finger_positions'][:,i].min()),float(d['finger_positions'][:,i].max())] for i in range(2)],
            'maximum_hand_contact_count':int(d['bowl_hand_contact_count'].max())}
(out/'results/stability-smoke-validation.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
PY
