#!/usr/bin/env python3
"""Actual client + isolated ROS/HTTP fixtures; no robot driver or dynamics."""
import argparse
import io
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import rclpy
from controller_manager_msgs.msg import ControllerState,HardwareInterface
from controller_manager_msgs.srv import ListControllers,ListHardwareInterfaces,SwitchController
from rcl_interfaces.msg import ParameterValue,ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import CompressedImage,JointState
from std_msgs.msg import Float64MultiArray,Header
from capture_flash_fr3_observation import JOINTS,FINGERS,TOPICS
from flash_fr3_model_plan import evaluate_segment
from flash_fr3_stream_plan import make_stream_segments,validate_stream_bounds


class Fixture:
    def __init__(self,plan):
        self.n=rclpy.create_node('flash_stream_client_fixture',start_parameter_services=False)
        self.plan=plan;self.q=np.asarray(plan['expected_start_q']);self.reset('check')
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pubs={key:self.n.create_publisher(CompressedImage if key.endswith('_cam') else JointState,topic,qos)
                   for key,topic in TOPICS.items()}
        self.status=self.n.create_publisher(Float64MultiArray,'/flash_fr3_stream/status',20)
        self.subs=[self.n.create_subscription(Header,'/flash_fr3_stream/heartbeat',self.beat,qos),
                   self.n.create_subscription(Float64MultiArray,'/flash_fr3_stream/candidate',self.packet,1)]
        self.services=[self.n.create_service(ListControllers,'/controller_manager/list_controllers',self.listing),
            self.n.create_service(ListHardwareInterfaces,'/controller_manager/list_hardware_interfaces',self.interfaces),
            self.n.create_service(SwitchController,'/controller_manager/switch_controller',self.switch),
            self.n.create_service(GetParameters,'/flash_fr3_stream/get_parameters',self.parameters)]
        self.jpeg=cv2.imencode('.jpg',np.full((256,256,3),100,np.uint8))[1].tobytes()
        self.timer=self.n.create_timer(.01,self.publish)
    def reset(self,scenario):
        self.scenario=scenario;self.active=False;self.start=None;self.heartbeats=[]
        self.switches=[];self.candidates=[];self.errors=[];self.plans={0:self.plan};self.native_stops=0
        self.last_camera=0.
    def listing(self,request,response):
        response.controller=[ControllerState(name='joint_state_broadcaster',state='active'),
            ControllerState(name='flash_fr3_stream',state='active' if self.active else 'inactive',
                            type=self.plan['plugin'],claimed_interfaces=JOINTS if self.active else [])]
        return response
    def interfaces(self,request,response):
        response.command_interfaces=[HardwareInterface(name=j+'/'+kind,is_available=True,
            is_claimed=self.active and kind=='position') for j in JOINTS for kind in ('position','velocity','effort')]
        return response
    def parameters(self,request,response):
        response.values=[ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,double_array_value=self.plan[k])
                         for k in ('expected_start_q','segment_durations')]
        response.values+=[ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                double_array_value=np.asarray(self.plan['coefficients']).ravel().tolist()),
            ParameterValue(type=ParameterType.PARAMETER_STRING,
                string_value=self.plan['plan_id']+('_wrong' if self.scenario=='mismatch' else '')),
            ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                integer_value=1 if self.scenario=='limit_mismatch' else self.plan['max_replacements'])]
        return response
    def switch(self,request,response):
        a=request.activate_controllers==['flash_fr3_stream'] and not request.deactivate_controllers
        b=request.deactivate_controllers==['flash_fr3_stream'] and not request.activate_controllers
        if not (a or b):self.errors.append('bad switch')
        self.switches.append(bool(a));self.active=bool(a);response.ok=True;return response
    def beat(self,message):
        source=message.stamp.sec+message.stamp.nanosec/1e9
        if message.frame_id!=self.plan['plan_id'] or not -.01<=time.time()-source<=.1:
            self.errors.append('bad heartbeat')
        self.heartbeats.append(time.monotonic())
        if self.start is None:self.start=time.monotonic()
    def packet(self,message):
        d=message.data
        sequence=len(self.candidates)+1
        if (len(d)!=104 or message.layout.dim[0].label!=self.plan['plan_id'] or
                list(d[:2])!=[1.,float(sequence)] or sequence>self.plan['max_replacements'] or
                abs(d[2]-(self.start+sequence*.5))>1e-6):
            self.errors.append('bad packet');return
        initial=np.asarray([evaluate_segment(np.asarray(self.plans[sequence-1]['coefficients'])[0],.5,.5,k) for k in range(3)])
        c=np.asarray(d[6:]).reshape(2,7,7)
        observed=np.asarray([evaluate_segment(c[0],.5,0,k) for k in range(3)])
        if np.abs(initial-observed).max()>1e-9:self.errors.append('C2 mismatch')
        next_plan=dict(self.plan,coefficients=c.tolist())
        if not validate_stream_bounds(next_plan)['pass']:self.errors.append('bad bounds')
        self.candidates.append(time.monotonic());self.plans[sequence]=next_plan
    def publish(self):
        now=time.monotonic();stamp=self.n.get_clock().now().to_msg()
        order=[0,2,5,6,1,3,4]
        self.pubs['arm'].publish(JointState(header=Header(stamp=stamp),name=[JOINTS[j] for j in order],
            position=[float(self.q[j]) for j in order],velocity=[0.]*7))
        if now-self.last_camera>=.03:
            self.last_camera=now
            self.pubs['hand'].publish(JointState(header=Header(stamp=stamp),name=FINGERS,position=[.04,.04]))
            for key in ('primary_cam','wrist_cam'):
                self.pubs[key].publish(CompressedImage(header=Header(stamp=stamp),format='jpeg',data=self.jpeg))
        if not self.active:return
        seq=0;prefix=self.start if self.start else now;phase=0 if self.start is None else 1
        elapsed=0. if self.start is None else now-self.start
        plan=self.plan
        for index,candidate in sorted(self.plans.items()):
            if index and elapsed>=index*.5:
                seq=index;prefix=self.start+index*.5;plan=candidate
        local=0. if self.start is None else now-prefix
        if local>=1.:phase=4;local=1.
        if self.scenario=='controller_error' and elapsed>.1:phase=5
        segment=0 if local<=.5 else 1;t=local if segment==0 else local-.5
        c=np.asarray(plan['coefficients'])[segment]
        ref=np.asarray([evaluate_segment(c,.5,t,k) for k in range(3)]);ref[0]+=self.q
        d=[now,float(phase),0.,0.]+self.q.tolist()+[0.]*7+ref[0].tolist()+ref[1].tolist()+ref[2].tolist()
        last_candidate=max(self.plans)
        d+=[now,prefix,prefix+.5,float(seq),float(last_candidate) if last_candidate>seq else -1.,
            float(last_candidate),0.]+self.q.tolist()
        self.status.publish(Float64MultiArray(data=d))
        if phase==4 and self.scenario!='no_native_stop':
            self.active=False;self.native_stops+=1


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prediction',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path);args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='232' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        p.error('仅允许隔离 domain 232 与 ROS_LOCALHOST_ONLY=1')
    args.output.mkdir(parents=True,exist_ok=False)
    source=json.loads(args.prediction.read_text())
    q=np.array([-.16097,-.77796,.111985,-2.383927,.071423,1.562963,.664019])
    plan=make_stream_segments(source['curve'],.5,np.stack([q,np.zeros(7),np.zeros(7)]),q)
    plan.update(format='flash_fr3_stream_plan_v1',controller='flash_fr3_stream',
                plugin='flash_fr3_model_controller/StreamController',checkpoint_step=15000,weights='ema_model',
                max_plan_age_seconds=600.,max_replacements=1,plan_id='isolated-stream-client-session')
    health={'status':'FLASH_SHADOW_SERVER_READY','checkpoint_step':15000,'weights':'ema_model',
            'robot_motion_commanded':False}
    rclpy.init();fixture=Fixture(plan)
    class Handler(BaseHTTPRequestHandler):
        def send(self,data):
            raw=json.dumps(data).encode();self.send_response(200);self.send_header('Content-Length',str(len(raw)))
            self.end_headers();self.wfile.write(raw)
        def do_GET(self):self.send(health)
        def do_POST(self):
            with np.load(io.BytesIO(self.rfile.read(int(self.headers['Content-Length']))),allow_pickle=False) as z:
                metadata=json.loads(str(z['metadata_json']))
            sequence=metadata['request_id']
            prefix_start_wall=time.time()-(time.monotonic()-(fixture.start+(sequence-1)*.5))
            newest=metadata['samples'][-1]['stamp_seconds']
            if not all(newest[key]>prefix_start_wall for key in ('arm','primary_cam','wrist_cam')):
                fixture.errors.append('replacement reused observations from an earlier prefix')
            if fixture.scenario=='late_prediction' or (fixture.scenario=='late_second' and sequence==2):
                time.sleep(.6)
            reply=dict(json.loads(json.dumps(source)),request_id=metadata['request_id'],
                observation_target_stamp_seconds=metadata['samples'][-1]['target_stamp_seconds'])
            if fixture.scenario=='second_bounds' and sequence==2:
                reply['curve']['coefficients'][0][1]+=1000.
            self.send(reply)
        def log_message(self,*args):pass
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    plan['server']='http://127.0.0.1:'+str(server.server_port)
    results=[]
    try:
        for scenario in ('check','mismatch','expired','complete','late_prediction','controller_error','no_native_stop',
                         'two_handoffs','late_second','second_bounds','limit_mismatch'):
            plan['max_replacements']=2 if scenario in ('two_handoffs','late_second','second_bounds','limit_mismatch') else 1
            source_prefix,scale=(.04,12.5) if plan['max_replacements']==2 else (.05,10.)
            plan.update(make_stream_segments(source['curve'],.5,np.stack([q,np.zeros(7),np.zeros(7)]),q,
                        source_prefix=source_prefix,time_scale=scale))
            fixture.reset(scenario)
            plan['created_wall_seconds']=time.time()-(601 if scenario=='expired' else 0)
            path=args.output/'fixture_plan.json';path.write_text(json.dumps(plan))
            command=[sys.executable,str(Path(__file__).with_name('run_flash_fr3_stream.py')),
                     '--plan',str(path),'--output',str(args.output/scenario)]
            if scenario!='check':command+=['--execute']
            with (args.output/(scenario+'.log')).open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+22
                while process.poll() is None and time.monotonic()<deadline:rclpy.spin_once(fixture.n,timeout_sec=.002)
                if process.poll() is None:process.kill();process.wait();raise AssertionError('timeout: '+scenario)
            report=json.loads((args.output/scenario/'report.json').read_text())
            assert not fixture.errors,fixture.errors
            assert not report['gripper_motion_commanded']
            if scenario in ('check','mismatch','expired','limit_mismatch'):
                assert not fixture.switches and not fixture.heartbeats and not fixture.candidates
                assert not report['activation_requested']
            else:
                assert fixture.switches[0] and not fixture.active
                assert report['controller_left_inactive'] and report['all_command_interfaces_unclaimed']
                if scenario in ('complete','late_prediction','two_handoffs','late_second','second_bounds'):
                    assert fixture.switches==[True] and fixture.native_stops==1
                    assert not report['client_deactivation_requested']
                else:
                    assert fixture.switches==[True,False] and report['client_deactivation_requested']
            expected={'check':'READONLY_PREFLIGHT_PASS','complete':'ROLLING_TRIAL_FINISHED',
                      'two_handoffs':'ROLLING_TRIAL_FINISHED',
                      'late_prediction':'STOPPED_WITHOUT_HANDOFF',
                      'late_second':'STOPPED_BEFORE_ALL_HANDOFFS',
                      'second_bounds':'STOPPED_BEFORE_ALL_HANDOFFS'}.get(scenario,'TRIAL_FAILED')
            assert report['status']==expected,report
            assert process.returncode==(1 if expected=='TRIAL_FAILED' else 0),(scenario,report)
            if scenario=='complete':assert len(fixture.candidates)==1 and report['handoffs_observed']==1
            if scenario=='late_prediction':assert not fixture.candidates and report['handoffs_observed']==0
            if scenario=='two_handoffs':assert len(fixture.candidates)==2 and report['handoffs_observed']==2
            if scenario in ('late_second','second_bounds'):
                assert len(fixture.candidates)==1 and report['handoffs_observed']==1
            result={'case':scenario,'status':'PASS','switch_requests':fixture.switches,
                    'candidate_packets':len(fixture.candidates),'heartbeats':len(fixture.heartbeats)}
            results.append(result);print(json.dumps(result),flush=True)
    finally:
        server.shutdown();server.server_close();fixture.n.destroy_node();rclpy.shutdown()
    summary={'status':'ISOLATED_STREAM_CLIENT_PASS','cases':results,'ros_domain_id':232,
             'hardware_driver_loaded':False,'physical_robot_motion_commanded':False}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('FLASH_STREAM_CLIENT_TEST='+json.dumps(summary),flush=True)

if __name__=='__main__':main()
