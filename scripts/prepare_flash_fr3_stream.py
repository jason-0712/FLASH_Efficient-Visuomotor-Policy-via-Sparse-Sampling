#!/usr/bin/env python3
"""Prepare the first prefix for two bounded online replacements; no activation."""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
import numpy as np
from flash_fr3_model_plan import write_preview_csv
from flash_fr3_stream_plan import make_stream_segments,validate_stream_bounds
from prepare_flash_fr3_model_prefix import PREVIEW

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--server',default='http://192.168.1.113:10131')
    args=p.parse_args()
    try:
        subprocess.run([sys.executable,str(Path(__file__).with_name('prepare_flash_fr3_model_prefix.py')),
                        '--output',str(args.output),'--server',args.server],check=True,
                       stdout=subprocess.PIPE,text=True)
    except subprocess.CalledProcessError as error:
        print(error.stdout or '',end='',flush=True)
        raise
    original=json.loads((args.output/'plan.json').read_text())
    prediction=json.loads((args.output/'prediction.json').read_text())
    q=np.asarray(original['expected_start_q'])
    plan=make_stream_segments(prediction['curve'],original['observation_age_at_plan_seconds'],
                             np.stack([q,np.zeros(7),np.zeros(7)]),q,
                             source_prefix=.04,time_scale=12.5)
    if not validate_stream_bounds(plan)['pass']:raise ValueError('首段超出滚动试运行边界')
    coeff=np.asarray(plan['coefficients'])
    (args.output/'preview_input.txt').write_text(' '.join(map(str,plan['segment_durations']+coeff.ravel().tolist()+q.tolist()))+'\n')
    result=subprocess.run([str(PREVIEW),str(args.output/'preview_input.txt'),
                str(args.output/'reference_cpp.csv')],check=True,capture_output=True,text=True)
    write_preview_csv(args.output/'reference_python.csv',plan)
    a=np.loadtxt(args.output/'reference_cpp.csv',delimiter=',',skiprows=1)
    b=np.loadtxt(args.output/'reference_python.csv',delimiter=',',skiprows=1)
    error=float(np.abs(a-b).max()) if a.shape==b.shape else float('inf')
    if error>1e-9:raise ValueError('C++/Python 首段求值不一致')
    keep=('created_wall_seconds','max_plan_age_seconds','joint_order','checkpoint_step','weights',
          'server','plan_id','heartbeat_timeout_seconds','watchdog_brake_seconds')
    plan.update({key:original[key] for key in keep})
    plan.update(format='flash_fr3_stream_plan_v1',controller='flash_fr3_stream',
        plugin='flash_fr3_model_controller/StreamController',max_replacements=2,
        preview=json.loads(result.stdout),cpp_python_max_abs_error=error,
        preview_binary_sha256=hashlib.sha256(PREVIEW.read_bytes()).hexdigest(),
        scope='two live C2 replacements at 0.5s intervals; 12.5x time scaling; 0.5s stop; gripper held',
        robot_motion_commanded=False,gripper_motion_commanded=False)
    params={'expected_start_q':q.tolist(),'segment_durations':plan['segment_durations'],
            'segment_coefficients':coeff.ravel().tolist(),'plan_id':plan['plan_id'],
            'max_replacements':plan['max_replacements']}
    (args.output/'controller_params.yaml').write_text(json.dumps(
        {'flash_fr3_stream':{'ros__parameters':params}},indent=2,allow_nan=False)+'\n')
    (args.output/'plan.json').write_text(json.dumps(plan,indent=2,allow_nan=False)+'\n')
    print('FLASH_STREAM_PLAN='+json.dumps({'status':'STREAM_FIRST_PLAN_READY',
        'plan_id':plan['plan_id'],'max_replacements':2,
        'maximum_trial_seconds':2.,'robot_motion_commanded':False}),flush=True)

if __name__=='__main__':main()
