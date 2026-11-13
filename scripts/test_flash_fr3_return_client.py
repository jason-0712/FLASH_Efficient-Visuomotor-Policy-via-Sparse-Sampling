"""Real return client against isolated ROS message fixtures; no driver/dynamics."""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header

from capture_flash_fr3_observation import JOINTS, FINGERS
from franka_msgs.action import Move
from test_flash_fr3_task_client import Fixture


class ReturnFixture(Fixture):
    def move(self, handle):
        self.last_move_request = (handle.request.width, handle.request.speed)
        if self.scenario == 'open_failure':
            if self.active:self.errors.append('open while arm active')
            self.hand_actions.append('release');handle.abort()
            return Move.Result(success=False, error='prescribed open failure')
        return super().move(handle)

    def load(self, request, response):
        delay=getattr(self,'load_delay_once',0.)
        self.load_delay_once=0.
        if delay:time.sleep(delay)
        result=super().load(request,response)
        shift=getattr(self,'start_shift_on_load',0.)
        self.start_shift_on_load=0.
        self.q[1]+=shift
        return result

    def publish(self):
        # Publish prescribed endpoint fixtures only; this is not a robot
        # dynamics model, controller tracking test or collision test.
        now = time.monotonic()
        phase = 0 if self.start is None else (4 if now-self.start >= sum(self.plan['segment_durations']) else 1)
        if self.active and phase == 4:
            c = np.array(self.plan['segment_coefficients']).reshape(2, 7, 7)
            self.q = np.array(self.plan['expected_start_q'])+c[1].sum(axis=0)
        h = Header(stamp=self.n.get_clock().now().to_msg())
        self.pubs['arm'].publish(JointState(header=h, name=JOINTS[::-1], position=self.q[::-1].tolist(), velocity=[0.]*7))
        self.pubs['hand'].publish(JointState(header=h, name=FINGERS, position=[self.width/2]*2))
        if self.active:
            self.pub.publish(Float64MultiArray(data=[now, float(phase), 0., 0.]+self.q.tolist()+[0.]*7+self.q.tolist()+[0.]*7))
            if phase == 4:
                self.active = False
                self.native_stops += 1


def main():
    p = argparse.ArgumentParser(); p.add_argument('--output', type=Path, required=True); args = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '226' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        p.error('需要隔离 domain226 与 localhost only')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init(); f = ReturnFixture(); results = []
    original = f.q.copy(); pose = args.output/'home.json'

    def run_case(name, flags, expected, returncode=0):
        output = args.output/name
        command = [sys.executable, str(Path(__file__).with_name('run_flash_fr3_return.py')),
                   '--output', str(output), '--pose', str(pose)]+flags
        with (args.output/(name+'.log')).open('w') as log:
            proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+40
            while proc.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(f.n, timeout_sec=.002)
            if proc.poll() is None:
                proc.kill(); proc.wait(); raise AssertionError('fixture timeout: '+name)
        assert proc.returncode == returncode, (name, (args.output/(name+'.log')).read_text())
        report=json.loads((output/'report.json').read_text()) if (output/'report.json').exists() else None
        if expected is not None:assert report['status']==expected,report
        if returncode == 0:
            assert report['all_command_interfaces_unclaimed'] and not f.active
            if '--execute' in flags:
                assert report['gripper_motion_commanded'] and report['gripper_open_success'],report
                assert f.hand_actions == ['release'] and not f.errors,f.hand_actions
                assert f.last_move_request == (.08, .1),f.last_move_request
                assert abs(report['final_gripper_width_m']-.08)<1e-6,report
            else:
                assert not report['gripper_motion_commanded'] and not f.hand_actions
        results.append({'case': name, 'status': 'PASS'})
        print(json.dumps(results[-1]), flush=True)
        return report

    try:
        run_case('save_readonly', ['--save'], 'POSE_SAVED')
        saved = pose.read_bytes()
        np.testing.assert_allclose(json.loads(saved)['q_rad'], original, atol=1e-7)
        assert not f.switches and not f.hand_actions
        run_case('refuse_overwrite', ['--save'], None, returncode=2)
        assert pose.read_bytes() == saved and not f.switches
        f.q = original+np.array([0., .20, 0., 0., 0., 0., 0.])
        run_case('check_readonly', [], 'RETURN_PREFLIGHT_PASS')
        assert not f.switches and not f.hand_actions
        f.width=.03
        result=run_case('execute_fewer_segments_and_open', ['--execute'], 'RETURN_FINISHED')
        assert f.switches == [True, True] and f.native_stops == 2, f.switches
        assert result['planned_segments']==2 and result['nominal_motion_seconds']<7.5,result
        np.testing.assert_allclose(f.q, original, atol=1e-7)
        f.reset('delayed_fixed_return');f.q=original+np.array([0.,.03,0.,0.,0.,0.,0.])
        f.load_delay_once=5.5
        run_case('execute_slow_configure', ['--execute'], 'RETURN_FINISHED')
        validation=json.loads((args.output/'execute_slow_configure/segment_000/activation_validation.json').read_text())
        assert validation['generation_to_validation_seconds']>5.,validation
        assert validation['max_start_gap_rad']<=.002 and validation['controller_parameters_match'],validation
        assert f.switches==[True] and f.native_stops==1 and f.hand_actions==['release']
        np.testing.assert_allclose(f.q,original,atol=1e-7)
        f.reset('already_home');f.q=original.copy();f.width=.03
        run_case('already_home_still_opens',['--execute'],'ALREADY_AT_SAVED_POSE')
        assert not f.switches and f.hand_actions==['release']
        f.reset('open_failure');f.q=original.copy();f.width=.03
        failed=run_case('open_failure_is_reported',['--execute'],'RETURN_FAILED',returncode=1)
        assert failed['arm_return_completed'] and not failed['robot_motion_commanded'],failed
        assert 'prescribed open failure' in failed['error'],failed
        assert failed['gripper_actions'][-1]['success'] is False and not f.switches,failed
        f.reset('changed_start');f.q=original+np.array([0.,.03,0.,0.,0.,0.,0.])
        f.start_shift_on_load=.01
        changed=run_case('refuse_changed_start_after_configure',['--execute'],'RETURN_FAILED',returncode=1)
        assert '起点在配置期间发生变化' in changed['error'],changed
        assert not f.switches and not f.hand_actions and not changed['robot_motion_commanded'],changed
        assert changed['all_command_interfaces_unclaimed'] and not f.active,changed
        f.reset('active'); f.present = True; f.active = True
        # Stale/other active controller must never be taken over or switched off.
        run_case('refuse_active_controller', [], None, returncode=1)
        assert not f.switches and f.active
    finally:
        f.g.destroy(); f.m.destroy(); f.n.destroy_node(); rclpy.shutdown()
    summary = {'status': 'ISOLATED_RETURN_CLIENT_PASS', 'cases': results,
               'hardware_driver_loaded': False, 'robot_dynamics': False}
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print('FLASH_RETURN_CLIENT_TEST='+json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
