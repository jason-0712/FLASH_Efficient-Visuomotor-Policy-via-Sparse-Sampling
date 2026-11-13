"""Stationary grasp client against isolated ROS endpoints; no hardware/dynamics."""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy

from test_flash_fr3_task_client import Fixture


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='225' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        parser.error('需要隔离 domain225 与 localhost only')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();fixture=Fixture();original=fixture.q.copy();results=[]
    plan_path=args.output/'plan.json'
    try:
        for case in ('check','execute_once','missing_alignment','moved_pose','arm_active','already_closed','sdk_failure',
                     'open_check','open_empty','refuse_open_nonempty'):
            fixture.reset('sdk_false' if case=='sdk_failure' else 'check');fixture.q=original.copy()
            plan={'format':'flash_fr3_stationary_grasp_v1','expected_q_rad':original.tolist(),
                  'source_model_closure':.94,'gripper_target_width_m':.029,
                  'onsite_alignment':{'cube_between_fingers':case!='missing_alignment',
                                      'fingertips_below_cube_top':True}}
            if case=='moved_pose':fixture.q[1]+=.01
            if case=='arm_active':fixture.present=True;fixture.active=True
            if case=='already_closed':fixture.width=.029
            opening=case in ('open_check','open_empty','refuse_open_nonempty')
            if opening:
                fixture.width=.029 if case=='refuse_open_nonempty' else .001
                plan.update(empty_grasp_confirmed=True,empty_grasp_final_width_m=.001)
                plan['onsite_alignment']['fingertips_below_cube_top']=False
            plan_path.write_text(json.dumps(plan))
            out=args.output/case
            command=[sys.executable,str(Path(__file__).with_name('run_flash_fr3_stationary_grasp.py')),
                     '--plan',str(plan_path),'--output',str(out)]
            if opening:command.append('--open')
            if case not in ('check','open_check'):command.append('--execute')
            with (args.output/(case+'.log')).open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+20
                while process.poll() is None and time.monotonic()<deadline:
                    rclpy.spin_once(fixture.n,timeout_sec=.003)
                if process.poll() is None:
                    process.kill();process.wait();raise AssertionError('隔离场景超时：'+case)
            report=json.loads((out/'report.json').read_text())
            expected=('STATIONARY_OPEN_PREFLIGHT_PASS' if case=='open_check' else
                      'GRIPPER_OPEN_FINISHED' if case=='open_empty' else
                      'STATIONARY_GRASP_PREFLIGHT_PASS' if case=='check' else
                      'GRIPPER_ACTION_FINISHED' if case=='execute_once' else 'GRIPPER_TRIAL_FAILED')
            assert report['status']==expected,report
            assert process.returncode==(0 if case in ('check','execute_once','open_check','open_empty') else 1),report
            assert not fixture.switches and not report['robot_motion_commanded'],report
            assert report['arm_controller_activation_requested'] is False,report
            if case in ('execute_once','sdk_failure','open_empty'):
                assert fixture.hand_actions==(['release'] if case=='open_empty' else ['grasp']) and report['gripper_goals_sent']==1,report
                assert report['gripper_motion_commanded'] is True,report
                assert abs(report['final_width_m']-(.08 if case=='open_empty' else .001 if case=='sdk_failure' else .029))<1e-6,report
                if case=='sdk_failure':assert report['gripper_action']['sdk_success'] is False,report
                elif case=='execute_once':assert report['object_held']=='REQUIRES_ONSITE_OBSERVATION',report
            else:
                assert not fixture.hand_actions and report['gripper_goals_sent']==0,report
                assert report['gripper_motion_commanded'] is False,report
            if case=='arm_active':assert fixture.active,report
            elif case!='missing_alignment':
                assert not fixture.active and report['all_command_interfaces_unclaimed'],report
            if case=='missing_alignment':assert '缺少现场' in report['error'],report
            if case=='moved_pose':assert '离开现场确认位置' in report['error'],report
            if case=='already_closed':assert '拒绝重复闭爪' in report['error'],report
            if case=='refuse_open_nonempty':assert '当前开度与空夹记录不符' in report['error'],report
            results.append({'case':case,'status':'PASS','arm_activations':len(fixture.switches),
                            'gripper_goals_sent':report['gripper_goals_sent']})
            print(json.dumps(results[-1]),flush=True)
    finally:
        fixture.g.destroy();fixture.m.destroy();fixture.n.destroy_node();rclpy.try_shutdown()
    summary={'status':'ISOLATED_STATIONARY_GRASP_PASS','cases':results,
             'hardware_driver_loaded':False,'robot_dynamics':False}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('FLASH_STATIONARY_GRASP_TEST='+json.dumps(summary),flush=True)


if __name__=='__main__':main()
