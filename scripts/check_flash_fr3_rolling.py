#!/usr/bin/env python3
"""Live scheduled prediction/handoff rehearsal. No commands, heartbeat or activation."""
import argparse
import json
import math
import threading
import time
from pathlib import Path

import numpy as np
import cv2
import rclpy
from controller_manager_msgs.srv import ListControllers,ListHardwareInterfaces

from flash_fr3_model_plan import evaluate_segment
from capture_flash_fr3_observation import DT
from flash_fr3_stream_plan import make_stream_segments,prefix_end,validate_stream_bounds
from flash_fr3_stream_io import connection,prediction
from run_flash_fr3_guard_shadow import measured_state
from flash_fr3_sensor_process import IsolatedObservationBuffer
from run_flash_fr3_model_prefix import call


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--server',default='http://192.168.1.113:10131')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--trials',type=int,default=6)
    p.add_argument('--replacements',type=int,choices=(1,2),default=1)
    args=p.parse_args()
    if args.output.exists() or not 1<=args.trials<=20: p.error('输出目录须不存在；trials 为1到20')
    opener,health=connection(args.server)
    args.output.mkdir(parents=True)
    cv2.setNumThreads(1)
    rclpy.init();node=rclpy.create_node('flash_fr3_rolling_readonly')
    buffer=IsolatedObservationBuffer(node)
    def check_inactive():
        listing=node.create_client(ListControllers,'/controller_manager/list_controllers')
        hardware=node.create_client(ListHardwareInterfaces,'/controller_manager/list_hardware_interfaces')
        controllers=call(node,listing,ListControllers.Request()).controller
        interfaces=call(node,hardware,ListHardwareInterfaces.Request()).command_interfaces
        node.destroy_client(listing);node.destroy_client(hardware)
        if any(c.state=='active' and c.name!='joint_state_broadcaster' for c in controllers):
            raise RuntimeError('存在运行中的运动控制器')
        if any(i.is_claimed for i in interfaces): raise RuntimeError('命令接口仍被占用')
        return {'controllers':[{"name":c.name,"state":c.state} for c in controllers],
                'unclaimed_command_interfaces':len(interfaces)}
    before=check_inactive()
    stop=threading.Event();errors=[]
    def spin():
        try:
            while not stop.is_set(): rclpy.spin_once(node,timeout_sec=.005)
        except Exception as error: errors.append(str(error))
    worker=threading.Thread(target=spin);worker.start()
    rows=[]
    try:
        deadline=time.monotonic()+15
        while True:
            try: buffer.observation(-1);break
            except ValueError:
                if time.monotonic()>deadline or errors: raise RuntimeError('实时历史不完整: '+str(errors))
                stop.wait(.05)
        for trial in range(args.trials):
            row={'trial':trial,'requested_handoffs':args.replacements,
                 'virtual_handoffs_checked':0,'replacements':[],'robot_motion_commanded':False}
            try:
                request_base=trial*(args.replacements+1)
                first=prediction(buffer,opener,args.server,request_base)
                current=measured_state(buffer)
                if np.abs(current['dq']).max()>.01: raise ValueError('机械臂未静止')
                q=np.asarray(current['q'],float)
                initial=np.stack([q,np.zeros(7),np.zeros(7)])
                start=time.monotonic()
                source_prefix,scale=(.04,12.5) if args.replacements==2 else (.05,10.)
                plan0=make_stream_segments(first['reply']['curve'],start-first['observation_monotonic'],initial,q,
                                          source_prefix=source_prefix,time_scale=scale)
                check0=validate_stream_bounds(plan0)
                if not check0['pass']: raise ValueError('首段越过现有短程试验边界')
                # The first prefix is frozen, as in the inactive physical
                # preloaded plan. Align its start before a future grid sample.
                now=time.monotonic();grid_anchor=first['observation_monotonic']
                next_anchor=grid_anchor+math.ceil((now+.08-grid_anchor)/DT)*DT
                start=next_anchor-.04
                (args.output/f'trial_{trial:02d}_plans.json').write_text(json.dumps(
                    {'first':plan0,'first_prediction':first},indent=2,allow_nan=False)+'\n')
                previous_plan=plan0
                for sequence in range(1,args.replacements+1):
                    prefix_start=start+(sequence-1)*.5;handoff=prefix_start+.5
                    stop.wait(max(0.,prefix_start-time.monotonic()))
                    following=prediction(buffer,opener,args.server,request_base+sequence,
                                         handoff_monotonic=handoff,
                                         minimum_observation_monotonic=prefix_start)
                    age=handoff-following['observation_monotonic']
                    plan1=make_stream_segments(following['reply']['curve'],age,prefix_end(previous_plan),q,
                                              source_prefix=source_prefix,time_scale=scale)
                    bounds=validate_stream_bounds(plan1)
                    lead=handoff-time.monotonic()
                    gap=np.asarray([evaluate_segment(np.asarray(plan1['coefficients'])[0],.5,0,k)
                                    for k in range(3)])
                    gap[0]+=q;gap-=prefix_end(previous_plan)
                    entry={'sequence':sequence,'lead_seconds':lead,
                        'observation_age_at_handoff_seconds':age,
                        'next_rpc_ms':following['rpc_ms'],'next_prepare_ms':following['prepare_ms'],
                        'minimum_newest_input_after_prefix_start_seconds':
                        following['minimum_latest_arm_and_image_stamp_monotonic']-prefix_start,
                        'c2_max_abs':np.abs(gap).max(axis=1).tolist(),'bounds':bounds,
                        'conversion_max_abs_error':plan1['conversion_max_abs_error']}
                    row['replacements'].append(entry)
                    (args.output/f'trial_{trial:02d}_replacement_{sequence:02d}.json').write_text(json.dumps(
                        {'plan':plan1,'prediction':following,**entry},indent=2,allow_nan=False)+'\n')
                    if not bounds['pass']: raise ValueError('衔接段或任意时刻停车界超出现有试验边界')
                    if lead<.04: raise ValueError('新预测未在切换前40毫秒就绪；应使用已有停车段')
                    if np.abs(gap).max()>1e-9: raise ValueError('C2 衔接误差过大')
                    row['virtual_handoffs_checked']=sequence
                    previous_plan=plan1
                row.update(status='PASS',lead_seconds=min(x['lead_seconds'] for x in row['replacements']))
                stop.wait(max(0.,handoff+1.-time.monotonic()))
            except Exception as error:
                row.update(status='STOP_REQUIRED',reason=str(error))
                stop.wait(.3)
            rows.append(row)
            print('FLASH_ROLLING_READONLY_TRIAL='+json.dumps(
                {k:v for k,v in row.items() if k!='bounds'},allow_nan=False),flush=True)
            if errors: raise RuntimeError(str(errors))
    finally:
        stop.set();worker.join(timeout=3)
        after=check_inactive()
        buffer.close()
        node.destroy_node();rclpy.shutdown()
    passed=sum(r['status']=='PASS' for r in rows)
    report={'status':'ROLLING_READONLY_PASS' if passed==args.trials else 'ROLLING_READONLY_HAS_STOPS',
            'passed':passed,'trials':len(rows),'requested_handoffs_per_trial':args.replacements,
            'server':health,'before':before,'after':after,'rows':rows,
            'robot_motion_commanded':False,'gripper_motion_commanded':False,
            'scope':'live wall-clock scheduling and exact C2 reference calculation; measured robot remains stationary; no dynamics or closed-loop task evaluation'}
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('FLASH_ROLLING_READONLY='+json.dumps({k:report[k] for k in ('status','passed','trials','robot_motion_commanded')}),flush=True)
    if passed!=args.trials: raise SystemExit(1)


if __name__=='__main__': main()
