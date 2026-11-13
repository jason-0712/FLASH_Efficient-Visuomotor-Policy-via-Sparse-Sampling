#!/usr/bin/env python3
"""Full task attempt via bounded stop/observe/replan segments and gripper actions.

Default is read-only plus inactive configuration. Only --execute activates motion.
This is a first physical deployment adapter, not the paper's timing benchmark.
"""
import argparse
import json
import subprocess
import time
import uuid
from pathlib import Path
import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers,ListHardwareInterfaces,ListControllerTypes,UnloadController,SwitchController
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Header,Float64MultiArray
from capture_flash_fr3_observation import FINGERS,ordered_positions,stamp_seconds
from flash_fr3_model_plan import write_preview_csv
from flash_fr3_sensor_process import IsolatedObservationBuffer
from flash_fr3_shadow_math import evaluate_curve
from flash_fr3_stream_io import connection,prediction
from flash_fr3_task_plan import select_task_plan,TaskSequence,TASK_LOWER,TASK_UPPER,TASK_LIMITS
from flash_fr3_task_gripper import TaskGripper
from flash_fr3_grasp_guard import GraspGeometryGate
from run_flash_fr3_guard_shadow import measured_state
from run_flash_fr3_model_prefix import call

CONTROLLER='flash_fr3_task'
PLUGIN='flash_fr3_model_controller/TaskController'
PREVIEW='/home/ros/ros2_ws/install/flash_fr3_model_controller/lib/flash_fr3_model_controller/preview_task'


def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


def controller_parameters(plan):
    return {'expected_start_q':plan['expected_start_q'],'segment_durations':plan['segment_durations'],
            'segment_coefficients':np.asarray(plan['coefficients']).ravel().tolist(),'plan_id':plan['plan_id']}


class TaskRunner:
    def __init__(self,args,buffer_factory=IsolatedObservationBuffer):
        self.args=args;self.node=rclpy.create_node('flash_fr3_pickplace_client')
        self.buffer=buffer_factory(self.node,local_keys=('arm','hand'))
        self.result_prefix='FLASH_PICKPLACE_RESULT'
        self.gripper=TaskGripper(self.node);self.rows=[];self.bad_status=False
        def status(message):
            d=np.asarray(message.data,float)
            if d.shape!=(32,) or not np.isfinite(d).all():self.bad_status=True;return
            self.rows.append({'receipt_monotonic':time.monotonic(),'data':d.tolist()})
        self.subscription=self.node.create_subscription(Float64MultiArray,'/'+CONTROLLER+'/status',status,20)
        self.heartbeat=self.node.create_publisher(Header,'/'+CONTROLLER+'/heartbeat',
                    QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.listing=self.node.create_client(ListControllers,'/controller_manager/list_controllers')
        self.hardware=self.node.create_client(ListHardwareInterfaces,'/controller_manager/list_hardware_interfaces')
        self.switching=self.node.create_client(SwitchController,'/controller_manager/switch_controller')
        self.types=self.node.create_client(ListControllerTypes,'/controller_manager/list_controller_types')
        self.unload=self.node.create_client(UnloadController,'/controller_manager/unload_controller')
        self.params=self.node.create_client(GetParameters,'/'+CONTROLLER+'/get_parameters')
        self.clock=Clock(clock_type=ClockType.SYSTEM_TIME)
        self.report={'status':'PREFLIGHT_PENDING','execution_requested':args.execute,
                     'sensor_receiver':getattr(self.buffer,'receiver_info',None),
                     'robot_motion_commanded':False,'gripper_motion_commanded':False,
                     'segments':[],'gripper_actions':[],'max_wall_seconds':120.,'max_arm_segments':30,
                     'execution_style':'stop, observe, replan; gripper actions only while arm inactive',
                     'scope':'task attempt; physical object placement success requires observation'}
        self.active_directory=None;self.first_activation=False
    def controllers(self):return call(self.node,self.listing,ListControllers.Request()).controller
    def inactive(self):
        controllers=self.controllers()
        if any(c.state=='active' and c.name!='joint_state_broadcaster' for c in controllers):
            raise RuntimeError('仍有运动控制器 active；任务不自动接管')
        commands=call(self.node,self.hardware,ListHardwareInterfaces.Request()).command_interfaces
        if len(commands)!=21 or any(i.is_claimed for i in commands):raise RuntimeError('命令接口未全部释放')
        return [{'name':c.name,'state':c.state} for c in controllers]
    def state(self,static=False):
        s=measured_state(self.buffer);now=time.monotonic()
        age=s['source_age_at_receipt_seconds']+now-s['receipt_monotonic']
        if not -.05<=age<=.1 or not 0<=now-s['receipt_monotonic']<=.1:
            raise ValueError('机械臂状态过期')
        if np.abs(s['dq']).max()>(.01 if static else .30):raise ValueError('机械臂速度超出任务范围')
        if ((s['q']<TASK_LOWER)|(s['q']>TASK_UPPER)).any():raise ValueError('实测关节超出任务示教范围')
        return s
    def spin_static(self,seconds=.5):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.003)
            try:return self.state(static=True)
            except ValueError:pass
        raise RuntimeError('机械臂尚未静止或缺少新鲜状态')
    def width(self):
        with self.buffer.lock:
            if not self.buffer.history['hand']:raise ValueError('缺少夹爪状态')
            m,receipt,wall=self.buffer.history['hand'][-1]
        if time.monotonic()-receipt>.3 or wall-stamp_seconds(m)>.3:raise ValueError('夹爪状态过期')
        value=float(ordered_positions(m,FINGERS).sum())
        if not -.001<=value<=.081:raise ValueError('夹爪开度异常')
        return value
    def switch(self,activate):
        request=SwitchController.Request(strictness=SwitchController.Request.STRICT,
            activate_controllers=[CONTROLLER] if activate else [],
            deactivate_controllers=[] if activate else [CONTROLLER],activate_asap=True,timeout=Duration(sec=3))
        if not call(self.node,self.switching,request).ok:raise RuntimeError('任务控制器切换失败')
    def configure(self,plan,directory):
        self.inactive()
        current=next((x for x in self.controllers() if x.name==CONTROLLER),None)
        if current and (current.state!='inactive' or current.type!=PLUGIN):raise RuntimeError('同名任务控制器异常')
        if current and not call(self.node,self.unload,UnloadController.Request(name=CONTROLLER)).ok:
            raise RuntimeError('旧任务控制器卸载失败')
        params=controller_parameters(plan)
        path=directory/'controller_params.yaml';save(path,{CONTROLLER:{'ros__parameters':params}})
        result=subprocess.run(['ros2','run','controller_manager','spawner',CONTROLLER,
             '--controller-type',PLUGIN,'--param-file',str(path),'--inactive','--controller-manager-timeout','10'],
             capture_output=True,text=True,timeout=25)
        if result.returncode:raise RuntimeError('加载任务控制器失败: '+result.stdout+result.stderr)
        self.inactive()
        self.verify_configured_plan(plan)
    def verify_configured_plan(self,plan):
        params=controller_parameters(plan)
        values=call(self.node,self.params,GetParameters.Request(names=list(params))).values
        if (len(values)!=4 or not np.array_equal(values[0].double_array_value,params['expected_start_q']) or
            not np.array_equal(values[1].double_array_value,params['segment_durations']) or
            not np.array_equal(values[2].double_array_value,params['segment_coefficients']) or
            values[3].string_value!=plan['plan_id']):raise RuntimeError('任务控制器参数核对失败')
    def activation_timestamp(self,plan):
        return plan['created_monotonic']
    def reference_age(self,result):
        return time.monotonic()-result['observation_monotonic']
    def prepare(self,result,index):
        state=self.spin_static();q=state['q'];age=self.reference_age(result)
        plan=select_task_plan(result['reply']['curve'],age,q)
        directory=self.args.output/f'segment_{index:03d}';directory.mkdir()
        plan.update(plan_id=uuid.uuid4().hex,created_monotonic=time.monotonic(),format='flash_fr3_task_segment_v1')
        save(directory/'plan.json',plan);save(directory/'prediction.json',result)
        blob=plan['segment_durations']+np.asarray(plan['coefficients']).ravel().tolist()+q.tolist()
        (directory/'preview_input.txt').write_text(' '.join(map(str,blob))+'\n')
        native=subprocess.run([PREVIEW,str(directory/'preview_input.txt'),str(directory/'reference_cpp.csv')],
                              capture_output=True,text=True,check=True,timeout=5)
        save(directory/'native_bounds.json',json.loads(native.stdout))
        write_preview_csv(directory/'reference_python.csv',plan)
        a=np.loadtxt(directory/'reference_cpp.csv',delimiter=',',skiprows=1)
        b=np.loadtxt(directory/'reference_python.csv',delimiter=',',skiprows=1)
        if a.shape!=b.shape or abs(a-b).max()>1e-9:raise RuntimeError('任务多项式 C++/Python 求值不一致')
        self.configure(plan,directory)
        return plan,directory
    def arm_segment(self,plan,directory):
        self.inactive();state=self.spin_static()
        if np.max(abs(state['q']-plan['expected_start_q']))>.002:raise RuntimeError('任务段起点变化')
        age=time.monotonic()-self.activation_timestamp(plan)
        if not 0<=age<=5:raise RuntimeError(f'任务计划过期：{age:.3f} 秒，限制为 5.000 秒')
        deadline=time.monotonic()+1
        while self.heartbeat.get_subscription_count()==0 and time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.01)
        if self.heartbeat.get_subscription_count()==0:raise RuntimeError('任务心跳端点未发现')
        self.rows.clear();self.bad_status=False;self.active_directory=directory
        self.report['robot_motion_commanded']=True;self.first_activation=True;self.switch(True)
        started=time.monotonic();next_beat=0.;complete=None;native_inactive=False
        while time.monotonic()-started<plan['total_motion_seconds']+3:
            now=time.monotonic()
            if now>=next_beat:
                msg=Header(stamp=self.clock.now().to_msg(),frame_id=plan['plan_id'])
                self.heartbeat.publish(msg);next_beat=now+.02
            rclpy.spin_once(self.node,timeout_sec=.002);self.state()
            if self.bad_status:raise RuntimeError('任务状态消息异常')
            if not self.rows:continue
            d=self.rows[-1]['data'];phase=int(d[1])
            if now-d[0]>.1:raise RuntimeError('控制器状态停止更新')
            if phase in (2,3,5) or int(d[2])!=0:raise RuntimeError(f'原生任务停车/故障 phase={phase} reason={d[2]}')
            if phase==4:
                if complete is None:complete=time.monotonic()
                c=next((c for c in self.controllers() if c.name==CONTROLLER),None)
                if c is not None and c.state=='inactive':native_inactive=True;break
                if time.monotonic()-complete>1.5:raise RuntimeError('未观察到任务控制器自行停用')
        if not native_inactive:raise RuntimeError('任务段执行超时')
        self.inactive();self.spin_static()
        self.archive_rows()
        data=np.array([x['data'] for x in self.rows])
        result={'plan_id':plan['plan_id'],'directory':str(directory),'status':'SEGMENT_FINISHED',
                'native_deactivation_observed':True,'duration_seconds':plan['total_motion_seconds'],
                'max_measured_excursion_deg':float(np.rad2deg(abs(data[:,4:11]-data[0,4:11]).max())),
                'max_sampled_tracking_error_deg':float(np.rad2deg(abs(data[:,4:11]-data[:,18:25]).max()))}
        self.report['segments'].append(result)
        print('FLASH_TASK_SEGMENT='+json.dumps(result),flush=True)
        return time.monotonic()
    def archive_rows(self):
        if self.active_directory is not None:
            with (self.active_directory/'status.jsonl').open('w') as out:
                for row in self.rows:out.write(json.dumps(row,allow_nan=False)+'\n')
    def get_prediction(self,request_id,after):
        deadline=time.monotonic()+3
        while True:
            rclpy.spin_once(self.node,timeout_sec=.003)
            try:
                result=prediction(self.buffer,self.opener,self.args.server,request_id)
                if result['minimum_latest_arm_and_image_stamp_monotonic']<=after:
                    if time.monotonic()>deadline:raise RuntimeError('缺少停车后的新图像')
                    continue
                return result
            except ValueError:
                if time.monotonic()>deadline:raise
    def run(self):
        self.report['controllers_before']=self.inactive()
        if PLUGIN not in call(self.node,self.types,ListControllerTypes.Request()).types:
            raise RuntimeError('任务插件尚未安装或只读管理器尚未重启')
        self.gripper.ready();self.opener,health=connection(self.args.server);self.report['server']=health
        config=json.loads(self.args.events.read_text())
        if (config.get('format')!='flash_fr3_task_demonstration_events_v1' or
                not .015<=config.get('gripper_target_width_m',0)<=.05):raise RuntimeError('示教夹爪事件配置异常')
        sequence=TaskSequence(config['grasp_q'],config['release_q'])
        description_client=self.node.create_client(GetParameters,'/controller_manager/get_parameters')
        description=call(self.node,description_client,GetParameters.Request(names=['robot_description'])).values
        if len(description)!=1 or description[0].type!=4 or not description[0].string_value:
            raise RuntimeError('无法读取当前 FR3 URDF，不能验证抓取几何位置')
        urdf=self.args.output/'robot_description.urdf';urdf.write_text(description[0].string_value)
        grasp_gate=GraspGeometryGate(urdf,config['grasp_q'])
        self.report['grasp_geometry_gate']=grasp_gate.description
        self.report['gripper_config']=config|{'grasp_q_count':len(config['grasp_q']),'release_q_count':len(config['release_q'])}
        del self.report['gripper_config']['grasp_q'];del self.report['gripper_config']['release_q']
        deadline=time.monotonic()+15
        while True:
            rclpy.spin_once(self.node,timeout_sec=.003)
            try:
                self.state(static=True);width=self.width();self.buffer.observation(-1);break
            except (ValueError,RuntimeError):
                if time.monotonic()>deadline:raise RuntimeError('相机/状态历史不完整或机械臂未静止')
        if width<.07:raise RuntimeError('任务起点要求空手且夹爪张开；当前开度不足 70 mm')
        first=self.get_prediction(0,time.monotonic()-.3)
        plan,directory=self.prepare(first,0)
        self.report.update(status='TASK_PREFLIGHT_PASS',first_plan=str(directory/'plan.json'),
            first_segment_seconds=plan['total_motion_seconds'],task_joint_lower_rad=TASK_LOWER.tolist(),
            task_joint_upper_rad=TASK_UPPER.tolist(),reference_limits=TASK_LIMITS[:,0].tolist())
        if not self.args.execute:return
        if getattr(self.args,'arm_only',False):
            print('即将执行仅机械臂接近测试：每段运动后停车并重新观察；'
                  '模型要求闭爪时检查抓取几何并结束测试。最多120秒、30段，夹爪不动。',flush=True)
        else:
            print('即将尝试整任务：每段运动后停车并重新观察；模型决定抓取/释放。'
                  '最多120秒、30段，夹爪20N、0.02m/s。',flush=True)
        started=time.monotonic();after=self.arm_segment(plan,directory);segments=1;request_id=1
        while time.monotonic()-started<120 and segments<30:
            result=self.get_prediction(request_id,after);request_id+=1
            q=self.spin_static()['q'];raw=evaluate_curve(result['reply']['curve'],0.)[0]
            closure=float(np.clip(raw[7],0,1))
            geometry=grasp_gate.check(q) if sequence.mode=='approach' and closure>=.7 else None
            decision=('grasp_blocked' if geometry is not None and not geometry['accepted'] else
                      sequence.observe(closure,q,raw[:7],result['observation_target_stamp_seconds']))
            event={'request_id':request_id-1,'mode':sequence.mode,'decision':decision,
                   'closure':closure,'q_target_gap_rad':float(abs(q-raw[:7]).max()),
                   'actual_q_rad':q.tolist(),'grasp_geometry':geometry}
            save(self.args.output/f'decision_{request_id-1:03d}.json',{'event':event,'prediction':result})
            print('FLASH_TASK_DECISION='+json.dumps(event),flush=True)
            if decision=='grasp_blocked':
                self.report['blocked_grasp']=event
                raise RuntimeError('模型要求闭爪，但实测抓取几何位置未通过：'+'；'.join(geometry['reasons'])+
                                   '。机械臂保持停用，不闭爪、不自动下移补偿。')
            if getattr(self.args,'arm_only',False) and sequence.mode=='approach' and closure>=.7:
                aligned_status=('APPROACH_GEOMETRY_REACHED' if decision in ('confirm','grasp')
                                else 'APPROACH_REQUEST_NOT_ALIGNED')
                self.report.update(status=aligned_status,approach_decision=event,
                                   physical_placement_success=False)
                return
            if decision=='confirm':after=time.monotonic();continue
            if decision in ('grasp','release'):
                self.inactive();self.spin_static();self.report['gripper_motion_commanded']=True
                try:action=self.gripper.perform(decision,config)
                except Exception:
                    action=dict(self.gripper.last_action or {'kind':decision,'success':False})
                    # Preserve failed attempts too: a failed SDK result can
                    # still have physically closed the fingers.
                    rclpy.spin_once(self.node,timeout_sec=.01)
                    try:action['measured_width_m']=self.width()
                    except ValueError:pass
                    self.report['gripper_actions'].append(action)
                    raise
                # Wait for a joint-state message newer than the completed action.
                ended=time.monotonic();deadline=ended+1
                while time.monotonic()<deadline:
                    rclpy.spin_once(self.node,timeout_sec=.01)
                    with self.buffer.lock:receipt=self.buffer.history['hand'][-1][1]
                    if receipt>ended:break
                else:raise RuntimeError('夹爪动作后状态未更新')
                action['measured_width_m']=self.width();self.report['gripper_actions'].append(action)
                if decision=='grasp':sequence.grasped(action['measured_width_m'],action['success'])
                else:
                    if action['measured_width_m']<.07:raise RuntimeError('释放后夹爪未张开')
                    sequence.released(action['success'])
                    self.report.update(status='GRASP_AND_RELEASE_SEQUENCE_FINISHED',
                        physical_placement_success='REQUIRES_USER_OBSERVATION')
                    break
                after=time.monotonic();continue
            plan,directory=self.prepare(result,segments)
            # Avoid beginning another segment beyond the wall-time budget.
            if time.monotonic()-started+plan['total_motion_seconds']>120:break
            after=self.arm_segment(plan,directory);segments+=1
            if sequence.mode=='carrying':sequence.carry_segments+=1
        if sequence.mode!='released':self.report.update(status='TASK_STOPPED_LIMIT',task_mode=sequence.mode)
        self.report['elapsed_seconds']=time.monotonic()-started
    def close(self):
        errors=[]
        if self.first_activation:
            # No further heartbeats. Let native 0.2s timeout + 0.5s brake complete.
            until=time.monotonic()+.9
            while time.monotonic()<until:rclpy.spin_once(self.node,timeout_sec=.005)
            try:
                c=next((c for c in self.controllers() if c.name==CONTROLLER),None)
                if c is not None and c.state=='active':
                    self.report['client_deactivation_requested']=True;self.switch(False)
            except Exception as error:errors.append(str(error))
        try:self.gripper.stop_if_needed()
        except Exception as error:errors.append(str(error))
        try:
            self.report['controllers_after']=self.inactive()
            self.report['controller_left_inactive']=True;self.report['all_command_interfaces_unclaimed']=True
        except Exception as error:errors.append(str(error))
        self.archive_rows()
        if errors:self.report.update(status='TASK_FAILED',cleanup_errors=errors)
        save(self.args.output/'report.json',self.report)
        print(self.result_prefix+'='+json.dumps(self.report,ensure_ascii=False,allow_nan=False),flush=True)
        self.buffer.close();self.node.destroy_node()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--events',type=Path,required=True)
    parser.add_argument('--server',default='http://192.168.1.113:10131')
    parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    if args.output.exists():parser.error('输出目录须不存在')
    args.output.mkdir(parents=True);cv2.setNumThreads(1)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);runner=TaskRunner(args)
    try:runner.run()
    except (Exception,KeyboardInterrupt) as error:runner.report.update(status='TASK_FAILED',error=str(error) or '用户中断')
    finally:runner.close();rclpy.shutdown()
    if runner.report['status'] not in ('TASK_PREFLIGHT_PASS','GRASP_AND_RELEASE_SEQUENCE_FINISHED'):raise SystemExit(1)


if __name__=='__main__':main()
