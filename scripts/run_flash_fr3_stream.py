#!/usr/bin/env python3
"""Default read-only; --execute requests a bounded series of C2 FLASH replans."""
import argparse
import concurrent.futures
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers,ListHardwareInterfaces,SwitchController
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Header,Float64MultiArray,MultiArrayDimension

from flash_fr3_stream_io import connection,prediction
from capture_flash_fr3_observation import DT
from flash_fr3_stream_plan import make_stream_segments,prefix_end,validate_stream_bounds
from run_flash_fr3_model_prefix import call
from run_flash_fr3_guard_shadow import measured_state
from flash_fr3_sensor_process import IsolatedObservationBuffer

CONTROLLER='flash_fr3_stream'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if args.output.exists():parser.error('输出目录必须不存在')
    plan=json.loads(args.plan.read_text())
    if (plan.get('format')!='flash_fr3_stream_plan_v1' or plan.get('controller')!=CONTROLLER or
            plan.get('plugin')!='flash_fr3_model_controller/StreamController' or
            plan.get('checkpoint_step')!=15000 or plan.get('weights')!='ema_model' or
            plan.get('max_plan_age_seconds')!=600. or
            type(plan.get('max_replacements')) is not int or plan['max_replacements'] not in (1,2)):
        raise ValueError('滚动试运行计划格式异常')
    max_replacements=plan['max_replacements']
    expected=np.asarray(plan['expected_start_q'],float)
    coefficients=np.asarray(plan['coefficients'],float)
    durations=np.asarray(plan['segment_durations'],float)
    if (expected.shape!=(7,) or coefficients.shape!=(2,7,7) or
            not np.array_equal(durations,[.5,.5]) or not np.isfinite(expected).all() or
            not np.isfinite(coefficients).all() or not validate_stream_bounds(plan)['pass']):
        raise ValueError('滚动计划参数或边界异常')
    args.output.mkdir(parents=True)
    cv2.setNumThreads(1)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('flash_fr3_stream_client')
    buffer=IsolatedObservationBuffer(node);rows=[]
    def status(message):
        d=np.asarray(message.data,float)
        if d.shape!=(53,) or not np.isfinite(d).all():
            report['status_error']='控制器状态消息异常';return
        rows.append({'receipt_monotonic':time.monotonic(),'data':d.tolist()})
    subscription=node.create_subscription(Float64MultiArray,'/'+CONTROLLER+'/status',status,20)
    listing=node.create_client(ListControllers,'/controller_manager/list_controllers')
    hardware=node.create_client(ListHardwareInterfaces,'/controller_manager/list_hardware_interfaces')
    switching=node.create_client(SwitchController,'/controller_manager/switch_controller')
    params=node.create_client(GetParameters,'/'+CONTROLLER+'/get_parameters')
    publisher=node.create_publisher(Header,'/'+CONTROLLER+'/heartbeat',
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    candidate_pub=node.create_publisher(Float64MultiArray,'/'+CONTROLLER+'/candidate',1)
    clock=Clock(clock_type=ClockType.SYSTEM_TIME)
    pool=concurrent.futures.ThreadPoolExecutor(max_workers=1)
    future=None;failure=None
    report={'status':'PREFLIGHT_PENDING','execution_requested':args.execute,
            'activation_requested':False,'robot_motion_commanded':False,
            'gripper_motion_commanded':False,'plan_id':plan['plan_id'],
            'client_deactivation_requested':False,'candidate_published':False,
            'candidate_packets_published':0,'requested_handoffs':max_replacements,
            'maximum_trial_seconds':.5*(max_replacements+2),'replacements':[],
            'scope':'bounded online C2 replacements; not complete pick-and-place'}
    def controllers():return call(node,listing,ListControllers.Request()).controller
    def switch(activate):
        request=SwitchController.Request()
        request.activate_controllers=[CONTROLLER] if activate else []
        request.deactivate_controllers=[] if activate else [CONTROLLER]
        request.strictness=SwitchController.Request.STRICT
        request.activate_asap=True;request.timeout=Duration(sec=3)
        if not call(node,switching,request).ok:raise RuntimeError('控制器切换失败；不自动重试')
    def fresh(static=False):
        s=measured_state(buffer);now=time.monotonic()
        if (now-s['receipt_monotonic']>.1 or
                not -.05<=s['source_age_at_receipt_seconds']+now-s['receipt_monotonic']<=.1 or
                np.abs(s['dq']).max()>(.01 if static else .05)):
            raise RuntimeError('机械臂状态过期或速度超出本次试运行范围')
        return s
    def replacement(sequence,handoff_mono,origin,initial,prefix_start_mono):
        result=prediction(buffer,opener,plan['server'],sequence,handoff_monotonic=handoff_mono,
                          minimum_observation_monotonic=prefix_start_mono)
        next_plan=make_stream_segments(result['reply']['curve'],
                    handoff_mono-result['observation_monotonic'],initial,origin,
                    source_prefix=plan['source_prefix_seconds'],time_scale=plan['time_scale'])
        bounds=validate_stream_bounds(next_plan)
        if not bounds['pass']:raise ValueError('新预测或任意时刻停车界超出现有短程约束')
        return {'plan':next_plan,'prediction':result,'bounds':bounds,'ready_monotonic':time.monotonic()}
    try:
        available=controllers();current=next((c for c in available if c.name==CONTROLLER),None)
        if (current is None or current.state!='inactive' or current.type!=plan['plugin'] or
                current.claimed_interfaces or
                any(c.state=='active' and c.name!='joint_state_broadcaster' for c in available)):
            raise RuntimeError('滚动控制器尚未正确配置为 inactive，或有其他运动控制器运行')
        if any(i.is_claimed for i in call(node,hardware,ListHardwareInterfaces.Request()).command_interfaces):
            raise RuntimeError('硬件命令接口仍被占用')
        names=['expected_start_q','segment_durations','segment_coefficients','plan_id','max_replacements']
        values=call(node,params,GetParameters.Request(names=names)).values
        if (len(values)!=5 or not np.array_equal(values[0].double_array_value,expected) or
                not np.array_equal(values[1].double_array_value,durations) or
                not np.array_equal(values[2].double_array_value,coefficients.ravel()) or
                values[3].string_value!=plan['plan_id'] or
                values[4].integer_value!=max_replacements):raise RuntimeError('已配置参数与计划不一致')
        if not 0<=time.time()-plan['created_wall_seconds']<=600:
            raise RuntimeError('快照计划已过期，请重新 --prepare')
        deadline=time.monotonic()+15
        while True:
            rclpy.spin_once(node,timeout_sec=.002)
            try:
                state=fresh(static=True);_,warmup_metadata=buffer.observation(-1)
                if publisher.get_subscription_count()==0 or candidate_pub.get_subscription_count()==0:
                    raise ValueError('等待命令传输端点发现')
                break
            except (ValueError,RuntimeError):
                if time.monotonic()>deadline:raise RuntimeError('缺少完整新鲜的双相机/状态或控制器端点')
        if np.abs(state['q']-expected).max()>.002:raise RuntimeError('起点已变化，请重新 --prepare')
        opener,health=connection(plan['server'])
        grid_anchor=time.monotonic()-(time.time()-warmup_metadata['samples'][-1]['target_stamp_seconds'])
        report.update(status='READONLY_PREFLIGHT_PASS',current_q=state['q'].tolist(),server=health,
            max_start_gap_rad=float(np.abs(state['q']-expected).max()))
        if args.execute:
            print(f'即将真机执行：最多 {max_replacements+1} 个0.5秒的 FLASH 短段，'
                  f'尝试 {max_replacements} 次新观测切换，然后0.5秒停车；'
                  '任一次预测不满足条件则沿已有轨迹停车。夹爪不动。',flush=True)
            report.update(activation_requested=True,robot_motion_commanded=True)
            rows.clear();switch(True)
            started=time.monotonic();next_heartbeat=None;next_check=0.;complete_seen=None
            previous=None;attempted=set();observed_handoffs=0;plans_by_sequence={0:plan}
            while time.monotonic()-started<6:
                now=time.monotonic()
                if next_heartbeat is None and rows and int(rows[-1]['data'][1])==0:
                    next_anchor=grid_anchor+math.ceil((now+.08-grid_anchor)/DT)*DT
                    next_heartbeat=next_anchor-.04
                if next_heartbeat is not None and now>=next_heartbeat:
                    beat=Header();beat.stamp=clock.now().to_msg();beat.frame_id=plan['plan_id']
                    publisher.publish(beat);next_heartbeat=now+.02
                rclpy.spin_once(node,timeout_sec=.001)
                fresh()
                if report.get('status_error'):raise RuntimeError(report['status_error'])
                if not rows:continue
                row=rows[-1];d=np.asarray(row['data']);phase=int(d[1]);seq=int(d[42])
                marker=(phase,seq,int(d[43]),int(d[45]))
                if marker!=previous:
                    print('FLASH_STREAM_PHASE='+json.dumps({'phase':phase,'reason':int(d[2]),
                        'sequence':seq,'pending_sequence':int(d[43]),'rejection':int(d[45])}),flush=True)
                    previous=marker
                if phase in (2,3,5) or int(d[2])!=0:raise RuntimeError('控制器触发了停车或故障: '+str(marker))
                if not 0<=seq<=max_replacements:raise RuntimeError('控制器切换序号超出本次范围')
                observed_handoffs=max(observed_handoffs,seq)
                if phase==1 and seq<max_replacements and seq+1 not in attempted:
                    target_sequence=seq+1;attempted.add(target_sequence)
                    attempt={'sequence':target_sequence,'status':'PENDING'}
                    report['replacements'].append(attempt)
                    if now-row['receipt_monotonic']>.05 or now-d[0]>.05:
                        attempt.update(status='SKIPPED',reason='控制器时序样本过期')
                        report['replacement_skipped_reason']=attempt['reason']
                    else:
                        if future is not None:raise RuntimeError('上一预测请求仍未结束')
                        handoff_robot=float(d[41]);handoff_mono=float(d[0]+d[41]-d[39])
                        origin=d[46:53].copy()
                        local_plan=dict(plans_by_sequence[seq],expected_start_q=origin.tolist())
                        prefix_start_mono=float(d[0]+d[40]-d[39])
                        future=pool.submit(replacement,target_sequence,handoff_mono,origin,
                                           prefix_end(local_plan),prefix_start_mono)
                if future is not None and future.done():
                    completed=future;future=None
                    try:
                        result=completed.result()
                        lead=handoff_mono-time.monotonic()
                        report['replacement_ready_lead_seconds']=lead
                        attempt['ready_lead_seconds']=lead
                        if phase!=1 or seq!=target_sequence-1 or lead<.04:
                            raise ValueError('预测未在切换前40毫秒就绪，使用已有停车段')
                        packet=Float64MultiArray()
                        packet.layout.dim=[MultiArrayDimension(label=plan['plan_id'],size=104,stride=104)]
                        packet.data=[1.,float(target_sequence),handoff_robot,time.time()]+result['plan']['segment_durations']+np.asarray(result['plan']['coefficients']).ravel().tolist()
                        candidate_pub.publish(packet);report['candidate_published']=True
                        report['candidate_packets_published']+=1
                        attempt.update(status='PUBLISHED',handoff_robot_time=handoff_robot,
                            minimum_input_after_prefix_start_seconds=
                            result['prediction']['minimum_latest_arm_and_image_stamp_monotonic']-prefix_start_mono)
                        plans_by_sequence[target_sequence]=result['plan']
                        raw=json.dumps(result,indent=2,allow_nan=False)+'\n'
                        (args.output/f'replacement_{target_sequence:02d}.json').write_text(raw)
                        if target_sequence==1:(args.output/'replacement.json').write_text(raw)
                    except Exception as error:
                        attempt.update(status='SKIPPED',reason=str(error))
                        report['replacement_skipped_reason']=str(error)
                if int(d[45])!=0:report['native_candidate_rejection']=int(d[45])
                if phase==4:
                    if complete_seen is None:complete_seen=time.monotonic()
                    now=time.monotonic()
                    if now>=next_check:
                        next_check=now+.05
                        c=next((c for c in controllers() if c.name==CONTROLLER),None)
                        if c is not None and c.state=='inactive':
                            outcome=('ROLLING_TRIAL_FINISHED' if observed_handoffs==max_replacements else
                                     'STOPPED_WITHOUT_HANDOFF' if observed_handoffs==0 else
                                     'STOPPED_BEFORE_ALL_HANDOFFS')
                            report.update(status=outcome,handoffs_observed=observed_handoffs,
                                native_deactivation_observed_without_client_request=True)
                            break
                    if time.monotonic()-complete_seen>1.5:raise RuntimeError('未观察到 C++ 自行停用')
            else:raise RuntimeError('试运行超时')
    except (Exception,KeyboardInterrupt) as error:
        failure=str(error) or '用户中断';report.update(status='TRIAL_FAILED',error=failure)
    finally:
        if report['activation_requested']:
            # Heartbeats stop here. Give the native watchdog time to brake and
            # request deactivation before a client-side fallback is considered.
            deadline=time.monotonic()+.6
            while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.005)
            try:
                c=next((c for c in controllers() if c.name==CONTROLLER),None)
                if c is not None and c.state=='active':
                    report['client_deactivation_requested']=True;switch(False)
                c=next((c for c in controllers() if c.name==CONTROLLER),None)
                report['controller_left_inactive']=c is not None and c.state=='inactive'
                interfaces=call(node,hardware,ListHardwareInterfaces.Request()).command_interfaces
                report['all_command_interfaces_unclaimed']=not any(i.is_claimed for i in interfaces)
                if not (report['controller_left_inactive'] and report['all_command_interfaces_unclaimed']):
                    raise RuntimeError('未确认 inactive 与全部命令接口释放')
            except Exception as error:
                failure=str(error);report.update(status='TRIAL_FAILED',cleanup_error=failure)
        pool.shutdown(wait=True)
        if rows:
            data=np.asarray([r['data'] for r in rows])
            report['measured_excursion_from_first_status_rad']=np.abs(data[:,4:11]-data[0,4:11]).max(axis=0).tolist()
            report['max_abs_tracking_error_rad']=np.abs(data[:,4:11]-data[:,18:25]).max(axis=0).tolist()
            report['last_reference_velocity_rad_s']=data[-1,25:32].tolist()
            with (args.output/'status.jsonl').open('w') as stream:
                for row in rows:stream.write(json.dumps(row,allow_nan=False)+'\n')
        report['report']=str(args.output/'report.json')
        (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print('FLASH_STREAM_RESULT='+json.dumps(report,allow_nan=False),flush=True)
        buffer.close()
        node.destroy_node();rclpy.shutdown()
    if failure:raise SystemExit(1)


if __name__=='__main__':main()
