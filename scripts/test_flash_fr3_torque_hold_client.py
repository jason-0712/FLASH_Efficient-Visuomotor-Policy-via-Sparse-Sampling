"""Torque hold client against isolated services/messages; no hardware/dynamics."""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from controller_manager_msgs.msg import HardwareInterface
from rcl_interfaces.msg import ParameterType, ParameterValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header

import test_flash_fr3_task_client as fixture_module
from capture_flash_fr3_observation import JOINTS
from run_flash_fr3_torque_hold import CONTROLLER, PLUGIN
from flash_fr3_model_plan import evaluate_segment

# This rename affects the isolated fixture process only. Production modules
# and controller-manager parameters are never patched by this test.
fixture_module.CONTROLLER = CONTROLLER
fixture_module.PLUGIN = PLUGIN


class Fixture(fixture_module.Fixture):
    def __init__(self):
        super().__init__()
        self.sdk_pub = self.n.create_publisher(Float64MultiArray, '/'+CONTROLLER+'/sdk_status', 20)

    def switch(self, request, response):
        response = super().switch(request, response)
        if self.active:
            self.activation_stamp = time.monotonic()
        return response

    def types(self, request, response):
        response.types = [] if self.scenario == 'missing_plugin' else [PLUGIN]
        response.base_classes = [] if not response.types else ['controller_interface::ControllerInterface']
        return response

    def interfaces(self, request, response):
        response = super().interfaces(request, response)
        response.state_interfaces = [HardwareInterface(name=j+'/'+kind, is_available=True)
                                     for j in JOINTS for kind in ('position', 'velocity')]
        response.state_interfaces += [HardwareInterface(name='fr3/robot_state', is_available=True)]
        if self.scenario != 'missing_model':
            response.state_interfaces += [HardwareInterface(name='fr3/robot_model', is_available=True)]
        return response

    def load(self, request, response):
        response = super().load(request, response)
        if self.scenario == 'start_changed':
            self.q[1] += .003
        return response

    def getparams(self, request, response):
        for name in request.names:
            expected = self.plan[name]
            if isinstance(expected, bool):
                value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=expected)
            elif isinstance(expected, str):
                value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=expected)
            else:
                value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                       double_array_value=expected)
            response.values.append(value)
        if self.scenario == 'wrong_parameters':
            response.values[0].double_array_value[0] += .001
        return response

    def publish(self):
        now = time.monotonic()
        stamp = self.n.get_clock().now().to_msg()
        if self.scenario == 'stale_arm':
            stamp.sec -= 1
        self.pubs['arm'].publish(JointState(header=Header(stamp=stamp), name=JOINTS[::-1],
            position=self.q[::-1].tolist(), velocity=[0.]*7))
        if not self.active or not self.plan:
            return
        elapsed = 0. if self.start is None else now-self.start
        duration = sum(self.plan['segment_durations'])
        phase = 0 if self.start is None else 4 if elapsed >= duration else 1
        if self.start is None and now-self.activation_stamp > 2.:
            phase = 3
        if self.scenario == 'native_fault' and elapsed > .2:
            phase = 5
        coefficients = np.asarray(self.plan['segment_coefficients']).reshape(2, 7, 7)
        first_duration = self.plan['segment_durations'][0]
        index = 0 if elapsed <= first_duration else 1
        local = min(elapsed, duration)-(first_duration if index else 0.)
        q_ref = self.q+evaluate_segment(coefficients[index], self.plan['segment_durations'][index], local)
        dq_ref = evaluate_segment(coefficients[index], self.plan['segment_durations'][index], local, 1)
        if self.scenario == 'velocity_missing':
            dq_ref[:] = 0.
        measured = self.q.copy()
        if self.scenario == 'unsettled_prehold' and self.start is None:
            measured[3] += .001*(now-self.activation_stamp)
        values = [now, float(phase), 0., .01]+measured.tolist()+[0.]*7+q_ref.tolist()+dq_ref.tolist()
        values += [.01]*7+[.01]*7+[0.]*7+[7. if phase == 5 else 0.]
        self.pub.publish(Float64MultiArray(data=values))
        if self.scenario.startswith('sdk_') and self.scenario != 'sdk_missing':
            sdk = [1., now, now, float(phase), 1., .001]
            sdk += [.005]*7+[.01]*7+[.01]*7+[2.]*7+[.2]*7+[.01]*7
            sdk += measured.tolist()+[0.]*7+[.73, 0., .73, 1.]
            if self.scenario == 'sdk_malformed':
                sdk[0] = 2.
            self.sdk_pub.publish(Float64MultiArray(data=sdk))
        if phase in (3, 4, 5):
            self.active = False
            self.native_stops += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '222' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('需要隔离 domain222 与 localhost only')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    fixture = Fixture()
    original = fixture.q.copy()
    cases = []
    try:
        for case in ('check', 'hold', 'foreign_active', 'missing_plugin', 'missing_model',
                     'stale_arm', 'start_changed', 'wrong_parameters', 'native_fault',
                     'check_velocity', 'velocity_probe', 'velocity_missing',
                     'check_settled_velocity', 'settled_velocity', 'unsettled_prehold',
                     'sdk_check', 'sdk_complete', 'sdk_missing', 'sdk_malformed'):
            fixture.reset(case)
            fixture.q = original.copy()
            if case == 'foreign_active':
                fixture.present = True
                fixture.active = True
            output = args.output/case
            command = [sys.executable, str(Path(__file__).with_name('run_flash_fr3_torque_hold.py')),
                       '--output', str(output)]
            sdk_audit = case.startswith('sdk_')
            settled = case in ('check_settled_velocity', 'settled_velocity', 'unsettled_prehold') or sdk_audit
            velocity = case in ('check_velocity', 'velocity_probe', 'velocity_missing') or settled
            readonly = case in ('check', 'check_velocity', 'check_settled_velocity', 'sdk_check')
            finished = case in ('hold', 'velocity_probe', 'settled_velocity', 'sdk_complete')
            if not readonly:
                command.append('--execute')
            if velocity:
                command.append('--velocity-probe')
            if settled:
                command.append('--settle-before-probe')
            if sdk_audit:
                command.append('--sdk-torque-audit')
            with (args.output/(case+'.log')).open('w') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                deadline = time.monotonic()+40
                while process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(fixture.n, timeout_sec=.002)
                if process.poll() is None:
                    process.kill()
                    process.wait()
                    raise AssertionError('隔离场景超时：'+case)
            report = json.loads((output/'report.json').read_text())
            prefix = ('TORQUE_SDK_VELOCITY' if sdk_audit else 'TORQUE_SETTLED_VELOCITY' if settled else
                      'TORQUE_VELOCITY' if velocity else 'TORQUE_HOLD')
            expected = prefix+('_PREFLIGHT_PASS' if readonly else '_FINISHED' if finished else '_FAILED')
            assert report['status'] == expected, (case, report, (args.output/(case+'.log')).read_text())
            assert process.returncode == (0 if readonly or finished else 1), report
            assert not fixture.hand_actions and not report['gripper_motion_commanded'], report
            assert report['reference_displacement_commanded'] == (velocity and not readonly), report
            if case in ('hold', 'native_fault', 'velocity_probe', 'velocity_missing', 'settled_velocity',
                        'unsettled_prehold', 'sdk_complete', 'sdk_missing', 'sdk_malformed'):
                if case == 'sdk_missing':
                    assert fixture.switches == [True, False] and fixture.native_stops == 0, (case, report, fixture.switches)
                else:
                    assert fixture.switches == [True] and fixture.native_stops == 1, (case, report, fixture.switches)
                assert report['effort_control_requested'] is True, report
                assert fixture.plan['allow_activation'] is True, fixture.plan
                assert bool(any(fixture.plan['segment_coefficients'])) == velocity, fixture.plan
                if case == 'unsettled_prehold':
                    assert '不开始微动' in report['error'], report
                    assert fixture.start is None, report
                elif case in ('sdk_missing', 'sdk_malformed'):
                    assert 'SDK' in report['error'], report
                    assert fixture.start is None, report
                elif case == 'native_fault':
                    assert 'torque_fault=7' in report['error'], report
                else:
                    assert report['native_deactivation_observed'], report
                    np.testing.assert_allclose(report['per_joint_peak_command_nm'], .01)
                if case in ('velocity_probe', 'settled_velocity', 'sdk_complete'):
                    assert report['nonzero_analytic_velocity_observed'], report
                    np.testing.assert_allclose(report['per_joint_peak_reference_velocity_rad_s'][3],
                                               .000625, rtol=.01)
                if case in ('settled_velocity', 'sdk_complete'):
                    assert report['activation_to_first_heartbeat_seconds'] >= 1.5, report
                    assert report['incremental_response']['tracking_validation'] == 'NOT_CERTIFIED', report
                    assert not report['incremental_response']['joint4_increment_direction_matches'], report
                if case == 'sdk_complete':
                    assert report['sdk_torque_audit']['tracking_validation'] == 'NOT_CERTIFIED', report
                    np.testing.assert_allclose(report['sdk_torque_audit']['sdk_minus_previous_written_mae_nm'], .005)
                    assert (output/'sdk_status.jsonl').stat().st_size > 0
                if case == 'velocity_missing':
                    assert not report['nonzero_analytic_velocity_observed'], report
            else:
                assert not fixture.switches and not report['activation_requested'], report
            if case == 'foreign_active':
                assert fixture.active, report
            else:
                assert report['controller_left_inactive'] and not fixture.active, report
            cases.append({'case': case, 'status': 'PASS', 'arm_activations': fixture.switches,
                          'gripper_actions': fixture.hand_actions})
            print(json.dumps(cases[-1]), flush=True)
    finally:
        fixture.g.destroy()
        fixture.m.destroy()
        fixture.n.destroy_node()
        rclpy.try_shutdown()
    summary = {'status': 'ISOLATED_TORQUE_HOLD_CLIENT_PASS', 'cases': cases,
               'hardware_driver_loaded': False, 'robot_dynamics': False}
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print('FLASH_TORQUE_HOLD_CLIENT_TEST='+json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
