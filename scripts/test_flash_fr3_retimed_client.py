"""Prescribed ROS message endpoints only; no hardware or robot dynamics."""
import argparse
import io
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import rclpy

from capture_flash_fr3_observation import JOINTS
from flash_fr3_retimed_schema import validate_retimed_observation
from flash_fr3_sensor_process import RetimedIsolatedObservationBuffer
from test_flash_fr3_task_client import Fixture


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='228' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        parser.error('需要隔离 domain228 与 localhost only')
    args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();fixture=Fixture();seen=[]

    class HTTP(BaseHTTPRequestHandler):
        def reply(self,value):
            raw=json.dumps(value).encode();self.send_response(200)
            self.send_header('Content-Length',str(len(raw)));self.end_headers();self.wfile.write(raw)
        def do_GET(self):
            self.reply({'status':'FLASH_SHADOW_SERVER_READY','checkpoint_step':15000,'weights':'ema_model',
                        'robot_motion_commanded':False,'retimed_observations':fixture.scenario!='old_server'})
        def do_POST(self):
            raw=self.rfile.read(int(self.headers['Content-Length']))
            with np.load(io.BytesIO(raw),allow_pickle=False) as archive:
                meta=json.loads(str(archive['metadata_json'].item()))
                arrays={k:archive[k].copy() for k in ('agent_pos','primary_cam','wrist_cam')}
            validate_retimed_observation(arrays,meta)
            assert meta['receiver_rmw_implementation']=='rmw_cyclonedds_cpp',meta
            seen.append(meta)
            coefficients=np.zeros((7,8));coefficients[0,:7]=fixture.q
            coefficients[0,7]=float(meta['request_id']>0)
            if fixture.scenario=='misaligned_prediction' and meta['request_id']>0:
                coefficients[0,1]+=.10
            curve={'basis':'legendre','coefficients':coefficients.tolist(),'anchor_s':2/11,
                   'polynomial_duration_seconds':44/15,'valid_future_seconds':32/15}
            echo=dict(meta['history_clock'])
            if fixture.scenario=='bad_echo':echo['cursor_seconds']+=1
            force_stale=(fixture.scenario=='all_stale' or
                         (fixture.scenario=='stale_then_fresh' and len(seen)==1) or
                         (fixture.scenario=='stale_after_arm' and meta['request_id']==1 and
                          sum(m['request_id']==1 for m in seen)<=2))
            if force_stale:
                # Force the physical observation beyond .7 s while preserving
                # a valid clock echo and staying inside the .8 s HTTP timeout.
                time.sleep(max(0.,meta['samples'][-1]['target_stamp_seconds']+.75-time.time()))
            self.reply({'status':'SHADOW_PREDICTION','protocol':'flash_fr3_shadow_v1',
                        'robot_motion_commanded':False,'checkpoint_step':15000,'weights':'ema_model',
                        'request_id':meta['request_id'],'observation_format':meta['format'],
                        'history_clock':echo,'action':[coefficients[0].tolist()]*8,'curve':curve,
                        'observation_target_stamp_seconds':meta['samples'][-1]['target_stamp_seconds']})
        def log_message(self,*args):pass

    server=ThreadingHTTPServer(('127.0.0.1',0),HTTP)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    events=args.output/'events.json';home=args.output/'home.json'
    results=[]
    try:
        for scenario in ('check','approach','old_server','bad_echo','geometry_fail','wrong_start',
                         'misaligned_prediction','stale_then_fresh','all_stale','stale_after_arm',
                         'delayed_camera','historical_gap'):
            fixture.reset(scenario);seen.clear()
            event_q=fixture.q.copy();saved_q=fixture.q.copy()
            if scenario=='geometry_fail':event_q[1]+=.10
            if scenario=='wrong_start':saved_q[1]+=.10
            events.write_text(json.dumps({'format':'flash_fr3_task_demonstration_events_v1',
                'grasp_q':[event_q.tolist()],'release_q':[fixture.q.tolist()],'gripper_target_width_m':.029}))
            home.write_text(json.dumps({'format':'flash_fr3_saved_joint_pose_v1','joint_names':JOINTS,
                'units':'rad','q_rad':saved_q.tolist(),'robot_ip':'172.16.0.2'}))
            out=args.output/scenario
            command=[sys.executable,str(Path(__file__).with_name('run_flash_fr3_retimed_approach.py')),
                '--events',str(events),'--home',str(home),'--output',str(out),
                '--server','http://127.0.0.1:'+str(server.server_port)]
            if scenario in ('approach','geometry_fail','misaligned_prediction','stale_after_arm','historical_gap'):
                command.append('--execute')
            with (args.output/(scenario+'.log')).open('w') as log:
                process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+45
                while process.poll() is None and time.monotonic()<deadline:
                    rclpy.spin_once(fixture.n,timeout_sec=.002)
                if process.poll() is None:
                    process.kill();process.wait();raise AssertionError('隔离场景超时: '+scenario)
            report=json.loads((out/'report.json').read_text())
            assert report['sensor_receiver']['actual_rmw']=='rmw_cyclonedds_cpp',report
            expected={'check':'TASK_PREFLIGHT_PASS','approach':'APPROACH_GEOMETRY_REACHED',
                      'old_server':'TASK_FAILED','bad_echo':'TASK_FAILED','geometry_fail':'TASK_FAILED',
                      'wrong_start':'RETURN_TO_SAVED_START_REQUIRED',
                      'misaligned_prediction':'APPROACH_REQUEST_NOT_ALIGNED',
                      'stale_then_fresh':'TASK_PREFLIGHT_PASS','all_stale':'TASK_FAILED',
                      'stale_after_arm':'APPROACH_GEOMETRY_REACHED',
                      'delayed_camera':'TASK_PREFLIGHT_PASS','historical_gap':'TASK_FAILED'}[scenario]
            assert report['status']==expected,report
            assert process.returncode==(0 if scenario in ('check','approach','stale_then_fresh','stale_after_arm','delayed_camera') else 1),report
            assert not fixture.errors,fixture.errors
            assert not fixture.hand_actions and report['gripper_motion_commanded'] is False,report
            assert not fixture.active and report['all_command_interfaces_unclaimed'],report
            if scenario in ('approach','geometry_fail','misaligned_prediction','stale_after_arm','historical_gap'):
                assert fixture.switches==[True] and fixture.native_stops==1,report
                assert len(report['segments'])==1,report
                assert (out/'segment_000/executed_history_interval.json').is_file()
                assert report['completed_history_clock']['intervals'],report
                if scenario!='historical_gap':
                    assert any(m['history_clock']['intervals'] for m in seen),report
                    assert (out/'observation_001.npz').is_file()
            else:assert not fixture.switches,report
            if scenario=='bad_echo':assert len(seen)==1,seen
            if scenario=='historical_gap':
                failures=report['sensor_history_coverage_failures']
                d=failures[-1]
                assert d['request_id']==1 and d['sample_index']==5 and d['sample_kind']=='historical',d
                assert d['uncovered_sources']==['primary_cam'],d
                assert d['maximum_offset_seconds']==.08,d
                assert len(seen)==1,seen
                # The bounded retry log may discard the first failure. Its
                # transport context must survive separately across the pipe.
                contexts=report['sensor_transport_gap_contexts']
                assert len(contexts)==1 and (out/'sensor_transport_diagnostics.json').is_file(),report
                context=contexts[0]
                assert context['request_id']==1 and context['sample_index']==5,context
                assert context['own_udp_drop_increment']>=0,context
                assert context['udp_sockets_at_failure'],context
                assert context['camera_comparisons']['primary_cam']['camera_info_stamps_inside_compressed_gap'],context
            if scenario in ('stale_then_fresh','all_stale','stale_after_arm'):
                discarded=report['discarded_stale_predictions']
                if scenario=='stale_after_arm':
                    # The first post-stop capture may predate the client's
                    # after marker and be rejected by the base freshness gate.
                    assert 1<=len(discarded)<=2,report
                else:assert len(discarded)==(3 if scenario=='all_stale' else 1),report
                assert all(d['observation_wall_age_seconds']>.7 for d in discarded),report
                assert all(d['maximum_age_seconds']==.7 for d in discarded),report
                if scenario=='all_stale':
                    assert len(seen)==3 and not report['robot_motion_commanded'],report
                else:
                    assert report['latest_observation_wall_age_seconds']<=.7,report
                    assert seen[-1]['samples'][-1]['target_stamp_seconds']>discarded[-1]['observation_target_stamp_seconds']
                    assert seen[-1]['history_clock']==seen[-2]['history_clock']
            log_text=(args.output/(scenario+'.log')).read_text()
            assert 'Traceback' not in log_text,log_text
            result={'case':scenario,'status':'PASS','gripper_goals_sent':len(fixture.hand_actions),
                    'arm_activations':len(fixture.switches),'observation_requests':len(seen)}
            results.append(result);print(json.dumps(result),flush=True)
        for force_terminate in (False,True):
            buffer=RetimedIsolatedObservationBuffer(fixture.n,local_keys=('arm',))
            if force_terminate:
                buffer.process.terminate();buffer.process.join(timeout=2)
                assert not buffer.process.is_alive(),'SIGTERM did not terminate the subscriber'
            buffer.close()
            assert not buffer.process.is_alive(),'subscriber did not stop'
            assert buffer.process.exitcode==(-15 if force_terminate else 0),buffer.process.exitcode
            result={'case':'sensor_forced_exit' if force_terminate else 'sensor_orderly_exit',
                    'status':'PASS','child_exitcode':buffer.process.exitcode,'gripper_goals_sent':0,'arm_activations':0}
            results.append(result);print(json.dumps(result),flush=True)
    finally:
        server.shutdown();server.server_close();fixture.g.destroy();fixture.m.destroy()
        fixture.n.destroy_node();rclpy.shutdown()
    summary={'status':'ISOLATED_RETIMED_APPROACH_PASS','cases':results,
             'hardware_driver_loaded':False,'robot_dynamics':False}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('FLASH_RETIMED_CLIENT_TEST='+json.dumps(summary),flush=True)


if __name__=='__main__':main()
