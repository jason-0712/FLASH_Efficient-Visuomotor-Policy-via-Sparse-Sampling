#!/usr/bin/env python3
"""One onsite-aligned gripper trial at a fixed arm pose; default is read-only.

No arm command publisher, heartbeat, controller switch or automatic lift/release.
This manual alignment experiment does not change the autonomous geometry gate.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions

from capture_flash_fr3_observation import FINGERS, ordered_positions, stamp_seconds
from flash_fr3_task_gripper import TaskGripper
from run_flash_fr3_grasp_probe import require_inactive
from run_flash_fr3_guard_shadow import measured_state
from run_flash_fr3_shadow_client import ObservationBuffer


def checked_plan(path,opening=False):
    plan=json.loads(path.read_text())
    if plan.get('format')!='flash_fr3_stationary_grasp_v1':raise ValueError('夹爪试验计划格式错误')
    alignment=plan.get('onsite_alignment',{})
    if not opening and not (alignment.get('cube_between_fingers') is True and
            alignment.get('fingertips_below_cube_top') is True):
        raise ValueError('缺少现场方块与手指对准事实确认')
    if opening and not (plan.get('empty_grasp_confirmed') is True and
                        0<=float(plan.get('empty_grasp_final_width_m',1.))<.003):
        raise ValueError('缺少本次空夹记录，不能按空夹复位')
    q=np.asarray(plan['expected_q_rad'],dtype=float)
    if q.shape!=(7,) or not np.isfinite(q).all():raise ValueError('预期姿态异常')
    if not .7<=float(plan['source_model_closure'])<=1.:raise ValueError('原接近记录尚未到闭爪阶段')
    if not .015<=float(plan['gripper_target_width_m'])<=.05:raise ValueError('夹爪目标宽度异常')
    return plan,q


def fresh_hand(node,buffer,after=None):
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.005)
        with buffer.lock:row=buffer.history['hand'][-1] if buffer.history['hand'] else None
        if row is None:continue
        message,receipt,wall=row
        if after is not None and receipt<=after:continue
        if time.monotonic()-receipt>.3 or not -.05<=wall-stamp_seconds(message)<=.3:continue
        width=float(ordered_positions(message,FINGERS).sum())
        if not -.001<=width<=.081:raise ValueError('实测夹爪开度异常')
        return width
    raise ValueError('未取得新鲜夹爪状态')


def stationary_start(node,buffer,expected):
    deadline=time.monotonic()+4;stable=None;last='等待关节状态'
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.003)
        try:
            state=measured_state(buffer);now=time.monotonic()
            if (now-state['receipt_monotonic']>.1 or
                    not -.05<=state['source_age_at_receipt_seconds']+now-state['receipt_monotonic']<=.1):
                raise ValueError('关节状态过期')
            if abs(state['dq']).max()>.01:raise ValueError('机械臂尚未静止')
        except ValueError as error:
            stable=None;last=str(error);continue
        gap=float(abs(state['q']-expected).max())
        if gap>.002:raise ValueError('当前姿态已离开现场确认位置，拒绝闭爪')
        if stable is None:stable=now
        if now-stable>=.3:return state,gap
    raise ValueError('未取得连续静止的新鲜状态：'+last)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--open',action='store_true',help='Reset the confirmed empty grasp to 80 mm')
    args=parser.parse_args()
    if args.output.exists():parser.error('输出目录必须不存在')
    args.output.mkdir(parents=True)
    report={'status':'PREFLIGHT_PENDING','execution_requested':args.execute,
            'robot_motion_commanded':False,'gripper_motion_commanded':False,
            'arm_controller_activation_requested':False,'gripper_goals_sent':0,
            'operation':'open_empty_grasp' if args.open else 'grasp',
            'scope':'one stationary gripper action; no arm motion or lift; not a complete FLASH task'}
    node=None;gripper=None;buffer=None
    try:
        plan,expected=checked_plan(args.plan,opening=args.open)
        report['plan']=plan
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node=rclpy.create_node('flash_fr3_stationary_grasp_client')
        buffer=ObservationBuffer(node,keys=('arm','hand'))
        gripper=TaskGripper(node)
        report['controllers_before']=require_inactive(node)
        gripper.ready()
        state,gap=stationary_start(node,buffer,expected)
        width=fresh_hand(node,buffer)
        report.update(initial_q_rad=state['q'].tolist(),max_start_gap_rad=gap,initial_width_m=width)
        if args.open:
            if .003<=width<.075:raise ValueError('当前开度与空夹记录不符，拒绝按空夹复位')
        elif width<.07:raise ValueError('夹爪未张开到 70 mm，拒绝重复闭爪')
        require_inactive(node)
        # Recheck after the service queries, immediately before the single goal.
        stationary_start(node,buffer,expected)
        report['status']='STATIONARY_OPEN_PREFLIGHT_PASS' if args.open else 'STATIONARY_GRASP_PREFLIGHT_PASS'
        if args.execute and args.open and width>=.075:
            report['status']='GRIPPER_ALREADY_OPEN'
        elif args.execute:
            print('即将原地张开空夹夹爪至 80 mm：0.02 m/s；机械臂保持停用。' if args.open else
                  '即将原地执行一次闭爪：20 N、0.02 m/s；机械臂保持停用。',flush=True)
            report.update(gripper_motion_commanded=True,gripper_goals_sent=1)
            action=gripper.perform('release' if args.open else 'grasp',plan)
            report['gripper_action']=action
            report['final_width_m']=fresh_hand(node,buffer,after=time.monotonic())
            if args.open:
                if report['final_width_m']<.075:raise ValueError('复位后夹爪开度未达到 75 mm')
                report['status']='GRIPPER_OPEN_FINISHED'
            else:
                report.update(status='GRIPPER_ACTION_FINISHED',
                              object_held='REQUIRES_ONSITE_OBSERVATION',lift_verified=False)
    except (Exception,KeyboardInterrupt) as error:
        report.update(status='GRIPPER_TRIAL_FAILED',error=str(error) or '用户中断')
    finally:
        if node is not None:
            if gripper is not None:
                if gripper.last_action is not None:report['gripper_action']=dict(gripper.last_action)
                try:gripper.stop_if_needed()
                except Exception as error:report.setdefault('cleanup_errors',[]).append(str(error))
            if buffer is not None:
                # A failed SDK result may still have moved the fingers. Wait
                # for feedback received after completion/stop, even on failure.
                after=time.monotonic() if report['gripper_motion_commanded'] else None
                try:report['final_width_m']=fresh_hand(node,buffer,after=after)
                except Exception as error:
                    report.update(status='GRIPPER_TRIAL_FAILED',final_width_error=str(error))
            try:
                report['controllers_after']=require_inactive(node)
                report.update(controller_left_inactive=True,all_command_interfaces_unclaimed=True)
            except Exception as error:
                report.update(controller_left_inactive=False)
                report.setdefault('cleanup_errors',[]).append(str(error))
            node.destroy_node();rclpy.try_shutdown()
        if report.get('cleanup_errors'):report['status']='GRIPPER_TRIAL_FAILED'
        (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        print('FLASH_STATIONARY_GRASP_RESULT='+json.dumps(report,ensure_ascii=False,allow_nan=False),flush=True)
    if report['status'] not in ('STATIONARY_GRASP_PREFLIGHT_PASS','GRIPPER_ACTION_FINISHED',
                              'STATIONARY_OPEN_PREFLIGHT_PASS','GRIPPER_OPEN_FINISHED','GRIPPER_ALREADY_OPEN'):
        raise SystemExit(1)


if __name__=='__main__':main()
