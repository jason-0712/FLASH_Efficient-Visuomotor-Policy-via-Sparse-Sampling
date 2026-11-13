"""Production client with isolated ROS/HTTP messages; no driver or dynamics."""
import argparse
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterValue, ParameterType
from sensor_msgs.msg import CameraInfo, CompressedImage, JointState
from realsense2_camera_msgs.msg import Metadata
from franka_msgs.action import Grasp,Move
from std_msgs.msg import Float64MultiArray, Header

import test_flash_fr3_task_client as base
from capture_flash_fr3_observation import JOINTS, FINGERS
from flash_fr3_shadow_math import evaluate_curve
from run_flash_fr3_raw_policy import CONTROLLER, PLUGIN

base.CONTROLLER = CONTROLLER
base.PLUGIN = PLUGIN


class Fixture(base.Fixture):
    def __init__(self):
        super().__init__()
        self.candidate_sub = self.n.create_subscription(Float64MultiArray, '/'+CONTROLLER+'/candidate', self.candidate, 1)
        self.finish_sub=self.n.create_subscription(Header,'/'+CONTROLLER+'/finish',self.finish,10)
        self.metadata_pubs={key:self.n.create_publisher(Metadata,
            base.TOPICS[key].rsplit('/image_raw/compressed',1)[0]+'/metadata',10)
            for key in ('primary_cam','wrist_cam')}

    def reset(self, scenario):
        super().reset(scenario)
        self.received = []; self.current = None; self.pending = None; self.handoffs = 0
        self.activated_at = None; self.last_beat = None; self.rpc_count = 0
        self.frame_number=0
        self.finished_at=None
        self.rejected_sequence=-1

    def finish(self,msg):
        assert msg.frame_id==self.plan['session_id']
        self.finished_at=time.monotonic()
    def grasp(self,handle):
        assert self.active
        self.hand_actions.append('grasp');self.width=.029
        ok=self.scenario!='full_task_grasp_failure'
        if ok:handle.succeed()
        else:handle.abort()
        return Grasp.Result(success=ok,error='' if ok else 'prescribed empty grasp')
    def move(self,handle):
        assert self.active
        self.hand_actions.append('release');self.width=.08
        handle.succeed();return Move.Result(success=True)

    def switch(self, request, response):
        response = super().switch(request, response)
        if self.active:
            self.activated_at = time.monotonic()
        return response

    def beat(self, msg):
        if self.active:
            assert msg.frame_id == self.plan['session_id']
            self.last_beat = time.monotonic()

    def candidate(self, msg):
        assert self.active and msg.layout.dim[0].label == self.plan['session_id']
        assert (len(msg.data), msg.data[0]) == ((58, 2.) if msg.data[1] == 0 else (57, 1.))
        if msg.data[1] == 0:
            assert msg.data[57] == msg.data[4] and msg.data[3] < msg.data[57]
        if self.current is not None:
            assert msg.data[3] >= self.current[4]-1e-5, 'update used pre-handoff observation'
        self.received.append(list(msg.data))
        if self.scenario=='rejected_update' and msg.data[1]==1:
            self.rejected_sequence=1
            return
        self.pending = list(msg.data)

    def getparams(self, request, response):
        for key in request.names:
            value = self.plan[key]
            if self.scenario == 'wrong_profile' and key == 'execution_profile':
                value = 'commissioning'
            if isinstance(value, bool):
                response.values.append(ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))
            elif isinstance(value, str):
                response.values.append(ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value))
            elif isinstance(value, float):
                response.values.append(ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
            else:
                response.values.append(ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=value))
        return response

    def publish(self):
        now = time.monotonic(); stamp = self.n.get_clock().now().to_msg(); header = Header(stamp=stamp)
        self.pubs['arm'].publish(JointState(header=header, name=JOINTS, position=self.q.tolist(), velocity=[0.]*7))
        self.pubs['hand'].publish(JointState(header=header, name=FINGERS, position=[self.width/2,self.width/2]))
        if now-self.last_camera > .03:
            self.last_camera = now
            self.frame_number+=1
            for key in ('primary_cam','wrist_cam'):
                camera_header=header
                if self.scenario=='camera_header_drift':
                    ns=self.n.get_clock().now().nanoseconds+150000000
                    camera_header=Header();camera_header.stamp.sec=ns//1000000000;camera_header.stamp.nanosec=ns%1000000000
                self.metadata_pubs[key].publish(Metadata(header=camera_header,json_data=json.dumps(dict(
                    time_of_arrival=int(time.time()*1000),frame_number=self.frame_number,
                    frame_counter=self.frame_number,clock_domain='fixture_hardware_clock'))))
                self.info_pubs[key].publish(CameraInfo(header=camera_header))
                self.pubs[key].publish(CompressedImage(header=camera_header, format='jpeg', data=self.jpeg))
        if not self.active:
            return
        phase = 0; reason = 0
        if self.pending is not None and now >= self.pending[4]:
            if self.current is not None:
                self.handoffs += 1
            self.current = self.pending; self.pending = None
        qref = self.q.copy(); vref = np.zeros(7)
        if self.current is not None:
            phase = 1
            curve = dict(basis='legendre', coefficients=np.column_stack((np.asarray(self.current[8:57]).reshape(7,7), np.zeros(7))).tolist(),
                         anchor_s=2/11, polynomial_duration_seconds=44/15, valid_future_seconds=32/15)
            origin = self.current[57] if len(self.current)==58 else self.current[3]
            qref, vref, _ = evaluate_curve(curve, min(now-origin,32/15))
            qref = qref[:7]; vref = vref[:7]
        if self.finished_at is not None and now-self.finished_at>=.5:
            phase=4;reason=10
        elif now-self.activated_at >= self.plan['maximum_session_seconds']:
            phase = 4; reason = 9
        elif self.last_beat is not None and now-self.last_beat > .7:
            phase = 3; reason = 1
        if self.scenario == 'clock_fault' and self.current is not None:
            phase = 5; reason = 3
        sequence = -1 if self.current is None else int(self.current[1])
        pending = -1 if self.pending is None else int(self.pending[1])
        candidate = -1 if not self.received else int(self.received[-1][1])
        data = [now,float(phase),float(reason),.01]+self.q.tolist()+[0.]*7+qref.tolist()+vref.tolist()
        data += [.01]*7+[.01]*7+[0.]*7+[0.,float(sequence),float(pending),float(candidate),0.,float(self.handoffs),1.,now,0.,0.,0.,now,self.plan['maximum_session_seconds']]
        if candidate==self.rejected_sequence and candidate>=0:
            data[57]=6.;data[61]=.116
        if self.scenario != 'camera_header_drift':
            data += [.003 if self.scenario == 'clock_fault' else .001,.003,.001,0.]
        if self.scenario == 'clock_fault' and phase == 5:
            data[53] = 4.
        self.pub.publish(Float64MultiArray(data=data))
        if phase in (3,4,5):
            self.active = False; self.native_stops += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '223' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('必须使用隔离 domain223、本机通信')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(); fixture = Fixture(); cases = []

    class Handler(BaseHTTPRequestHandler):
        def send(self, status, value):
            encoded = json.dumps(value).encode(); self.send_response(status)
            self.send_header('Content-Length',str(len(encoded))); self.end_headers(); self.wfile.write(encoded)
        def do_GET(self):
            self.send(200, dict(status='FLASH_SHADOW_SERVER_READY',checkpoint_step=15000,weights='ema_model',robot_motion_commanded=False))
        def do_POST(self):
            with np.load(io.BytesIO(self.rfile.read(int(self.headers['Content-Length']))), allow_pickle=False) as data:
                metadata = json.loads(str(data['metadata_json']))
            fixture.rpc_count += 1
            if fixture.scenario == 'runtime_http_failure' and fixture.rpc_count > 1:
                self.send(500,{'error':'prescribed inference failure'}); return
            if fixture.scenario == 'active_http_failure' and fixture.rpc_count > 2:
                self.send(500,{'error':'prescribed active inference failure'}); return
            c = np.zeros((7,8)); c[0,:7] = fixture.q
            if fixture.scenario == 'large_initial_gap':
                c[0,3] += .06
            else:
                c[1,3] = .01*(44/15)/2; c[0,3] -= c[1,3]*(2*(2/11)-1)
            if fixture.scenario.startswith('full_task'):
                c[0,7]=0. if fixture.hand_actions else 1.
            curve = dict(basis='legendre',coefficients=c.tolist(),anchor_s=2/11,
                         polynomial_duration_seconds=44/15,valid_future_seconds=32/15)
            self.send(200,dict(status='SHADOW_PREDICTION',protocol='flash_fr3_shadow_v1',
                checkpoint_step=15000,weights='ema_model',robot_motion_commanded=False,
                request_id=metadata['request_id'],observation_format=metadata['format'],history_clock=None,
                observation_target_stamp_seconds=metadata['samples'][-1]['target_stamp_seconds'],
                curve=curve,action=[evaluate_curve(curve,i*4/15)[0].tolist() for i in range(8)]))
        def log_message(self,*args):
            pass

    server = ThreadingHTTPServer(('127.0.0.1',0),Handler)
    worker = threading.Thread(target=server.serve_forever,daemon=True); worker.start()
    try:
        for case in ('prepare','rolling','camera_header_drift','large_initial_gap','wrong_profile','runtime_http_failure','active_http_failure','clock_fault','full_task','full_task_grasp_failure','rejected_update'):
            fixture.reset(case); output = args.output/case
            command = [sys.executable,str(Path(__file__).with_name('run_flash_fr3_raw_policy.py')),
                '--output',str(output),'--server',f'http://127.0.0.1:{server.server_port}']
            if case.startswith('full_task'):
                events=args.output/'events.json'
                events.write_text(json.dumps(dict(gripper_target_width_m=.029,gripper_epsilon_m=.012,gripper_force_N=20.,gripper_speed_m_s=.02)))
                command += ['--seconds','5','--events',str(events)]
            else:command += ['--arm-only','--seconds','3']
            if case not in ('prepare','large_initial_gap'):
                command.append('--execute')
            with (args.output/(case+'.log')).open('w') as log:
                process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
                deadline = time.monotonic()+45
                while process.poll() is None and time.monotonic()<deadline:
                    rclpy.spin_once(fixture.n,timeout_sec=.002)
                if process.poll() is None:
                    process.kill(); process.wait(); raise AssertionError('隔离客户端超时：'+case)
            report = json.loads((output/'report.json').read_text())
            expected = ('TASK_SEQUENCE_FINISHED' if case=='full_task' else 'RAW_POLICY_PREFLIGHT_READY' if case in ('prepare','large_initial_gap') else 'RAW_POLICY_TRIAL_FINISHED' if case in ('rolling','camera_header_drift','rejected_update')
                        else 'RAW_POLICY_TRIAL_FAILED')
            assert report['status']==expected,(case,report)
            assert process.returncode==(0 if case in ('prepare','large_initial_gap','rolling','camera_header_drift','full_task','rejected_update') else 1),(case,report)
            if case.startswith('full_task'):
                assert report['gripper_motion_commanded']
                assert fixture.hand_actions==(['grasp','release'] if case=='full_task' else ['grasp'])
                assert report['task_sequence_complete']==(case=='full_task')
                if case=='full_task':assert report['handoffs_observed']>=1
            else:assert not fixture.hand_actions and not report['gripper_motion_commanded']
            assert report['all_command_interfaces_unclaimed'] and not fixture.active
            if case in ('prepare','large_initial_gap','wrong_profile','runtime_http_failure'):
                assert not fixture.switches and not report['robot_motion_commanded']
            else:
                assert fixture.switches==[True] and fixture.native_stops==1,(case,fixture.switches,report)
            if case in ('rolling','camera_header_drift'):
                assert report['handoffs_observed']>=1 and len(fixture.received)>=2,report
                assert fixture.plan['execution_profile']=='raw_policy_rollout'
                assert fixture.plan['k_gains']==[150.,150.,150.,150.,50.,30.,30.]
                assert np.allclose(fixture.plan['d_gains'],2*np.sqrt(fixture.plan['k_gains']))
                assert fixture.plan['torque_limits_nm']==[87.,87.,87.,87.,12.,12.,12.]
                assert fixture.plan['torque_rates_nm_s']==[999.999]*7
                assert max(report['peak_analytic_reference_velocity_rad_s'])>.005
            if case == 'clock_fault':
                event = report['protection_event']
                assert event['stream_reason'] == 3 and event['torque_fault'] == 4
                assert event['clock']['sdk_period_seconds'] == .003
            if case == 'rejected_update':
                assert report['rejected_updates']==[dict(sequence=1,code=6,q_gap_rad=.116,dq_gap_rad_s=0.)]
                assert any(x[1]>=2 for x in fixture.received) and fixture.handoffs>=1
            cases.append(dict(case=case,status='PASS',activations=fixture.switches,candidate_count=len(fixture.received)))
            print(json.dumps(cases[-1]),flush=True)
    finally:
        server.shutdown(); server.server_close(); worker.join(timeout=2)
        fixture.g.destroy(); fixture.m.destroy(); fixture.n.destroy_node(); rclpy.try_shutdown()
    (args.output/'summary.json').write_text(json.dumps(dict(status='ISOLATED_RAW_POLICY_CLIENT_PASS',cases=cases,
        robot_driver_loaded=False,robot_dynamics=False),indent=2)+'\n')


if __name__=='__main__':
    main()
