#!/usr/bin/env python3
"""Actual task client with isolated ROS/HTTP peers. No driver or dynamics."""
import argparse,io,json,os,subprocess,sys,threading,time
from pathlib import Path
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import cv2,numpy as np,rclpy
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile,ReliabilityPolicy
from controller_manager_msgs.msg import ControllerState,HardwareInterface
from controller_manager_msgs.srv import ListControllers,ListControllerTypes,ListHardwareInterfaces,LoadController,ConfigureController,UnloadController,SwitchController
from rcl_interfaces.srv import GetParameters,SetParameters
from rcl_interfaces.msg import ParameterValue,ParameterType,SetParametersResult
from franka_msgs.action import Grasp,Move
from std_srvs.srv import Trigger
from sensor_msgs.msg import CameraInfo,CompressedImage,JointState
from std_msgs.msg import Header,Float64MultiArray
from capture_flash_fr3_observation import JOINTS,FINGERS,TOPICS
from run_flash_fr3_pickplace import CONTROLLER,PLUGIN

class Fixture:
    def __init__(self):
        self.n=rclpy.create_node('task_client_fixture',start_parameter_services=False)
        self.q=np.array([-.16,-.77,.11,-2.38,.07,1.57,.66]);self.reset('check')
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pubs={k:self.n.create_publisher(CompressedImage if k.endswith('_cam') else JointState,t,qos) for k,t in TOPICS.items()}
        self.info_pubs={k:self.n.create_publisher(CameraInfo,
            TOPICS[k].rsplit('/image_raw/compressed',1)[0]+'/camera_info',qos)
            for k in ('primary_cam','wrist_cam')}
        self.pub=self.n.create_publisher(Float64MultiArray,'/'+CONTROLLER+'/status',20)
        self.sub=self.n.create_subscription(Header,'/'+CONTROLLER+'/heartbeat',self.beat,qos)
        self.services=[]
        for typ,name,callback in [(ListControllers,'list_controllers',self.listing),(ListControllerTypes,'list_controller_types',self.types),
          (ListHardwareInterfaces,'list_hardware_interfaces',self.interfaces),(LoadController,'load_controller',self.load),
          (ConfigureController,'configure_controller',self.configure),(UnloadController,'unload_controller',self.unload),
          (SwitchController,'switch_controller',self.switch),(SetParameters,'set_parameters',self.setparams)]:
            self.services.append(self.n.create_service(typ,'/controller_manager/'+name,callback))
        self.services.append(self.n.create_service(GetParameters,'/'+CONTROLLER+'/get_parameters',self.getparams))
        self.services.append(self.n.create_service(GetParameters,'/controller_manager/get_parameters',self.description))
        self.services.append(self.n.create_service(Trigger,'/franka_gripper/stop',self.stop))
        self.g=ActionServer(self.n,Grasp,'/franka_gripper/grasp',self.grasp)
        self.m=ActionServer(self.n,Move,'/franka_gripper/move',self.move)
        self.jpeg=cv2.imencode('.jpg',np.full((256,256,3),128,np.uint8))[1].tobytes()
        self.last_camera=0.;self.timer=self.n.create_timer(.01,self.publish)
    def reset(self,scenario):
        self.scenario=scenario;self.present=False;self.active=False;self.start=None
        self.params={};self.plan={};self.width=.08;self.hand_actions=[];self.switches=[];self.native_stops=0;self.errors=[]
    def listing(self,q,r):
        r.controller=[ControllerState(name='joint_state_broadcaster',state='active')]
        if self.present:r.controller.append(ControllerState(name=CONTROLLER,type=PLUGIN,state='active' if self.active else 'inactive',claimed_interfaces=JOINTS if self.active else []))
        return r
    def types(self,q,r):r.types=[PLUGIN];r.base_classes=['controller_interface::ControllerInterface'];return r
    def interfaces(self,q,r):
        r.command_interfaces=[HardwareInterface(name=j+'/'+k,is_available=True,is_claimed=self.active and k=='position') for j in JOINTS for k in ('position','velocity','effort')];return r
    def setparams(self,q,r):
        for p in q.parameters:self.params[p.name]=p.value
        r.results=[SetParametersResult(successful=True) for p in q.parameters];return r
    def load(self,q,r):
        if self.scenario=='slow_configure':time.sleep(5.5)
        path=self.params[CONTROLLER+'.params_file'].string_array_value[0]
        self.plan=json.loads(Path(path).read_text())[CONTROLLER]['ros__parameters']
        self.present=True;r.ok=True;return r
    def configure(self,q,r):r.ok=self.present and not self.active;return r
    def unload(self,q,r):r.ok=not self.active;self.present=False;return r
    def getparams(self,q,r):
        r.values=[ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=self.plan[k]) if k=='plan_id' else
                  ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,double_array_value=self.plan[k]) for k in q.names];return r
    def switch(self,q,r):
        activate=q.activate_controllers==[CONTROLLER]
        self.switches.append(activate);self.active=activate;self.start=None;r.ok=True;return r
    def description(self,q,r):
        # Prescribed kinematics for message fixtures; no driver or dynamics.
        links=''.join(f'<link name="fr3_link{i}"/>' for i in range(9))
        joints=''.join(f'<joint name="fr3_joint{i}" type="revolute"><parent link="fr3_link{i-1}"/>'
                       f'<child link="fr3_link{i}"/><axis xyz="0 0 1"/></joint>' for i in range(1,8))
        xml=f'<robot name="message_fixture">{links}{joints}<joint name="flange" type="fixed">'
        xml+='<parent link="fr3_link7"/><child link="fr3_link8"/></joint></robot>'
        r.values=[ParameterValue(type=ParameterType.PARAMETER_STRING,string_value=xml) for _ in q.names]
        return r
    def beat(self,msg):
        if not self.active:return
        if msg.frame_id!=self.plan['plan_id']:self.errors.append('heartbeat session mismatch')
        if self.start is None:self.start=time.monotonic()
    def grasp(self,handle):
        if self.active:self.errors.append('grasp while arm active')
        self.hand_actions.append('grasp');self.width=.001 if self.scenario in ('empty','sdk_false') else .029
        if self.scenario=='sdk_false':handle.abort();return Grasp.Result(success=False,error='')
        handle.succeed();return Grasp.Result(success=True)
    def move(self,handle):
        if self.active:self.errors.append('release while arm active')
        self.hand_actions.append('release');self.width=.08;handle.succeed();return Move.Result(success=True)
    def stop(self,q,r):r.success=True;return r
    def publish(self):
        now=time.monotonic();stamp=self.n.get_clock().now().to_msg();h=Header(stamp=stamp)
        self.pubs['arm'].publish(JointState(header=h,name=JOINTS,position=self.q.tolist(),velocity=[0.]*7))
        if now-self.last_camera>.03:
            self.last_camera=now;self.pubs['hand'].publish(JointState(header=h,name=FINGERS,position=[self.width/2]*2))
            for k in ('primary_cam','wrist_cam'):
                image_header=h
                if self.scenario=='delayed_camera' and k=='primary_cam':
                    ns=self.n.get_clock().now().nanoseconds-180000000
                    image_header=Header();image_header.stamp.sec=ns//1000000000
                    image_header.stamp.nanosec=ns%1000000000
                self.info_pubs[k].publish(CameraInfo(header=image_header))
                if (self.scenario=='historical_gap' and k=='primary_cam' and self.active and
                        self.start is not None and .85<now-self.start<1.2):continue
                self.pubs[k].publish(CompressedImage(header=image_header,format='jpeg',data=self.jpeg))
        if not self.active:return
        phase=0 if self.start is None else (4 if now-self.start>=sum(self.plan['segment_durations']) else 1)
        self.pub.publish(Float64MultiArray(data=[now,float(phase),0.,0.]+self.q.tolist()+[0.]*7+self.q.tolist()+[0.]*7))
        if phase==4:self.active=False;self.native_stops+=1

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='227' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':p.error('需要隔离 domain227 与 localhost only')
    args.output.mkdir(parents=True,exist_ok=False);rclpy.init();f=Fixture()
    class HTTP(BaseHTTPRequestHandler):
        def reply(self,d):
            raw=json.dumps(d).encode();self.send_response(200);self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def do_GET(self):self.reply({'status':'FLASH_SHADOW_SERVER_READY','checkpoint_step':15000,'weights':'ema_model','robot_motion_commanded':False})
        def do_POST(self):
            with np.load(io.BytesIO(self.rfile.read(int(self.headers['Content-Length']))),allow_pickle=False) as z:meta=json.loads(str(z['metadata_json']))
            grip=1. if meta['request_id']>0 and not f.hand_actions else 0.
            c=np.zeros((7,8));c[0,:7]=f.q;c[0,7]=grip
            curve={'basis':'legendre','coefficients':c.tolist(),'anchor_s':2/11,'polynomial_duration_seconds':44/15,'valid_future_seconds':32/15}
            self.reply({'status':'SHADOW_PREDICTION','protocol':'flash_fr3_shadow_v1','robot_motion_commanded':False,
                'checkpoint_step':15000,'weights':'ema_model','request_id':meta['request_id'],
                'observation_target_stamp_seconds':meta['samples'][-1]['target_stamp_seconds'],
                'curve':curve,'action':[c[0].tolist()]*8})
        def log_message(self,*args):pass
    server=ThreadingHTTPServer(('127.0.0.1',0),HTTP);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    events=args.output/'events.json';events.write_text(json.dumps({'format':'flash_fr3_task_demonstration_events_v1',
         'grasp_q':[f.q.tolist()],'release_q':[f.q.tolist()],'gripper_target_width_m':.029}))
    results=[]
    try:
        for scenario in ('check','complete','empty','sdk_false','misaligned_grasp','slow_configure'):
            f.reset(scenario);out=args.output/scenario
            event_q=f.q.copy()
            if scenario=='misaligned_grasp':event_q[1]+=.10
            events.write_text(json.dumps({'format':'flash_fr3_task_demonstration_events_v1',
                'grasp_q':[event_q.tolist()],'release_q':[f.q.tolist()],'gripper_target_width_m':.029}))
            command=[sys.executable,str(Path(__file__).with_name('run_flash_fr3_pickplace.py')),'--output',str(out),
                     '--events',str(events),'--server','http://127.0.0.1:'+str(server.server_port)]
            if scenario!='check':command+=['--execute']
            with (args.output/(scenario+'.log')).open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT);deadline=time.monotonic()+40
                while process.poll() is None and time.monotonic()<deadline:rclpy.spin_once(f.n,timeout_sec=.002)
                if process.poll() is None:process.kill();process.wait();raise AssertionError('fixture timed out: '+scenario)
            report=json.loads((out/'report.json').read_text());assert not f.errors,f.errors
            assert report['sensor_receiver']['actual_rmw']=='rmw_cyclonedds_cpp',report
            assert not f.active and report['all_command_interfaces_unclaimed'],report
            expected={'check':'TASK_PREFLIGHT_PASS','complete':'GRASP_AND_RELEASE_SEQUENCE_FINISHED',
                      'empty':'TASK_FAILED','sdk_false':'TASK_FAILED','misaligned_grasp':'TASK_FAILED',
                      'slow_configure':'TASK_FAILED'}[scenario]
            assert report['status']==expected,report
            assert process.returncode==(0 if scenario in ('check','complete') else 1)
            if scenario=='check':assert not f.switches and not f.hand_actions and not report['robot_motion_commanded']
            if scenario=='complete':assert f.hand_actions==['grasp','release'] and f.native_stops==2 and len(report['segments'])==2
            if scenario=='empty':assert f.hand_actions==['grasp'] and len(report['segments'])==1
            if scenario=='sdk_false':
                assert f.hand_actions==['grasp'] and len(report['gripper_actions'])==1
                assert report['gripper_actions'][0]['sdk_success'] is False
                assert 'status=' in report['error'] and report['gripper_actions'][0]['success'] is False
            if scenario=='misaligned_grasp':
                assert not f.hand_actions and report['gripper_motion_commanded'] is False
                assert report['blocked_grasp']['decision']=='grasp_blocked'
            if scenario=='slow_configure':
                assert '任务计划过期' in report['error'],report
                assert not f.switches and not f.hand_actions and not report['robot_motion_commanded'],report
            results.append({'case':scenario,'status':'PASS','hand_actions':f.hand_actions,'arm_activations':f.switches})
            print(json.dumps(results[-1]),flush=True)
    finally:
        server.shutdown();server.server_close();f.g.destroy();f.m.destroy();f.n.destroy_node();rclpy.shutdown()
    summary={'status':'ISOLATED_TASK_CLIENT_PASS','cases':results,'hardware_driver_loaded':False,'robot_dynamics':False}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('FLASH_TASK_CLIENT_TEST='+json.dumps(summary),flush=True)

if __name__=='__main__':main()
