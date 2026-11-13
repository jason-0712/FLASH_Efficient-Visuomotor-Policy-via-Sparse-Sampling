#!/usr/bin/env python3
"""Effort preflight, explicit 2.75 s hold, or bounded analytic-velocity probe.

No policy request, grasp, controller takeover or automatic retry. Only the
explicit velocity-probe mode requests a target displacement. These entries
commission the low-level interface, not a FLASH task rollout.
"""
import argparse
import json
import subprocess
import time
import uuid
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import (ListControllers, ListControllerTypes,
                                         ListHardwareInterfaces, SwitchController, UnloadController)
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64MultiArray, Header

from flash_fr3_task_plan import TASK_LOWER, TASK_UPPER
from flash_fr3_torque_probe_plan import probe_plan
from flash_fr3_torque_diagnostics import (baseline_window, incremental_response,
                                         sdk_prehold_window, sdk_torque_summary)
from run_flash_fr3_grasp_probe import query
from run_flash_fr3_guard_shadow import measured_state
from run_flash_fr3_shadow_client import ObservationBuffer

CONTROLLER = 'flash_fr3_torque'
PLUGIN = 'flash_fr3_model_controller/TorqueController'
KP = [150., 150., 150., 150., 50., 30., 10.]
KD = (2*np.sqrt(KP)).tolist()
LIMITS = [10., 10., 10., 10., 4., 4., 4.]
RATES = [100.]*7


class TorqueHold:
    def __init__(self, args):
        self.args = args
        self.velocity_probe = getattr(args, 'velocity_probe', False)
        self.settled_probe = getattr(args, 'settle_before_probe', False)
        self.sdk_audit = getattr(args, 'sdk_torque_audit', False)
        self.prehold_seconds = 1.5 if self.settled_probe else 0.
        self.prefix = ('TORQUE_SDK_VELOCITY' if self.sdk_audit else
                       'TORQUE_SETTLED_VELOCITY' if self.settled_probe else
                       'TORQUE_VELOCITY' if self.velocity_probe else 'TORQUE_HOLD')
        self.node = rclpy.create_node('flash_fr3_torque_hold_client')
        self.buffer = ObservationBuffer(self.node, keys=('arm',))
        self.rows = []
        self.sdk_rows = []
        self.bad_sdk_status = False
        self.bad_status = False
        self.requested_activation = False
        self.clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.subscription = self.node.create_subscription(Float64MultiArray,
            '/'+CONTROLLER+'/status', self.status, 20)
        self.sdk_subscription = self.node.create_subscription(Float64MultiArray,
            '/'+CONTROLLER+'/sdk_status', self.sdk_status, 20)
        self.heartbeat = self.node.create_publisher(Header, '/'+CONTROLLER+'/heartbeat',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.report = {'status': self.prefix+'_PREFLIGHT_PENDING', 'execution_requested': args.execute,
            'activation_requested': False, 'robot_motion_commanded': False,
            'effort_control_requested': False, 'reference_displacement_commanded': False,
            'gripper_motion_commanded': False,
            'duration_seconds': self.prehold_seconds+(3.75 if self.velocity_probe else 2.75),
            'pre_motion_hold_seconds_requested': self.prehold_seconds,
            'velocity_probe_requested': self.velocity_probe,
            'sdk_torque_audit_requested': self.sdk_audit,
            'scope': 'bounded low-level effort commissioning; not a FLASH policy rollout',
            'gains_source': 'FR3 commissioning adaptation; not author-reported paper gains',
            'k_gains': KP, 'd_gains': KD, 'torque_limits_nm': LIMITS, 'torque_rates_nm_s': RATES}

    def service(self, kind, endpoint, request):
        return query(self.node, kind, endpoint, request)

    def controllers(self):
        return self.service(ListControllers, '/controller_manager/list_controllers',
                            ListControllers.Request()).controller

    def inactive(self, *, require_capabilities=True):
        controllers = self.controllers()
        if any(c.state == 'active' and c.name != 'joint_state_broadcaster' for c in controllers):
            raise RuntimeError('仍有运动控制器 active；不自动接管')
        hardware = self.service(ListHardwareInterfaces, '/controller_manager/list_hardware_interfaces',
                                ListHardwareInterfaces.Request())
        commands = hardware.command_interfaces
        if len(commands) != 21 or any(i.is_claimed for i in commands):
            raise RuntimeError('机械臂命令接口未全部释放')
        if require_capabilities:
            for name in [f'fr3_joint{i}/effort' for i in range(1, 8)]:
                if not any(i.name == name and i.is_available for i in commands):
                    raise RuntimeError('缺少可用力矩接口：'+name)
            for name in ('fr3/robot_state', 'fr3/robot_model'):
                if not any(i.name == name and i.is_available for i in hardware.state_interfaces):
                    raise RuntimeError('缺少可用模型状态接口：'+name)
        return [{'name': c.name, 'state': c.state, 'type': c.type} for c in controllers]

    def state(self):
        state = measured_state(self.buffer)
        now = time.monotonic()
        age = state['source_age_at_receipt_seconds']+now-state['receipt_monotonic']
        if not 0 <= now-state['receipt_monotonic'] <= .1 or not -.05 <= age <= .1:
            raise ValueError('机械臂状态过期')
        if ((state['q'] < TASK_LOWER) | (state['q'] > TASK_UPPER)).any():
            raise ValueError('当前关节超出已验证任务范围')
        if abs(state['dq']).max() > .05:
            raise ValueError('当前速度超出本次力矩保持范围')
        return state

    def stable(self):
        deadline = time.monotonic()+4
        beginning = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.003)
            try:
                state = self.state()
                if abs(state['dq']).max() > .01:
                    raise ValueError('机械臂尚未静止')
                if beginning is None:
                    beginning = time.monotonic()
                if time.monotonic()-beginning >= .3:
                    return state
            except ValueError:
                beginning = None
        raise RuntimeError('未取得连续静止的新鲜机械臂状态')

    def status(self, message):
        data = np.asarray(message.data, dtype=float)
        if data.shape != (54,) or not np.isfinite(data).all():
            self.bad_status = True
        else:
            self.rows.append({'receipt_monotonic': time.monotonic(), 'data': data.tolist()})

    def sdk_status(self, message):
        data = np.asarray(message.data, dtype=float)
        if data.shape != (66,) or not np.isfinite(data).all():
            self.bad_sdk_status = True
        else:
            self.sdk_rows.append({'receipt_monotonic': time.monotonic(), 'data': data.tolist()})

    def switch(self, activate):
        request = SwitchController.Request(strictness=SwitchController.Request.STRICT,
            activate_controllers=[CONTROLLER] if activate else [],
            deactivate_controllers=[] if activate else [CONTROLLER],
            activate_asap=True, timeout=Duration(sec=3))
        if not self.service(SwitchController, '/controller_manager/switch_controller', request).ok:
            raise RuntimeError('力矩控制器切换失败')

    def verify_parameters(self, params):
        values = self.service(GetParameters, '/'+CONTROLLER+'/get_parameters',
            GetParameters.Request(names=list(params))).values
        if len(values) != len(params):
            raise RuntimeError('力矩参数回复长度不符')
        for (name, expected), actual in zip(params.items(), values):
            if isinstance(expected, bool):
                okay = actual.type == 1 and actual.bool_value == expected
            elif isinstance(expected, str):
                okay = actual.type == 4 and actual.string_value == expected
            else:
                okay = actual.type == 8 and np.array_equal(actual.double_array_value, expected)
            if not okay:
                raise RuntimeError('力矩控制器参数不一致：'+name)

    def run(self):
        self.report['controllers_before'] = self.inactive()
        kinds = self.service(ListControllerTypes, '/controller_manager/list_controller_types',
                             ListControllerTypes.Request()).types
        self.report['plugin_available'] = PLUGIN in kinds
        initial = self.stable()['q']
        self.report.update(initial_q_rad=initial.tolist(), status=self.prefix+'_PREFLIGHT_PASS')
        if PLUGIN not in kinds:
            raise RuntimeError('新力矩插件尚未被当前管理器发现；需在全 inactive 状态刷新插件列表')
        coefficients = np.zeros((2, 7, 7))
        durations = [2., .75]
        if self.velocity_probe:
            plan = probe_plan(initial)
            coefficients = np.asarray(plan['coefficients'])
            durations = plan['segment_durations']
            (self.args.output/'probe_plan.json').write_text(json.dumps(plan, indent=2)+'\n')
            self.report.update(desired_joint_displacement_rad=[0., 0., 0., .001, 0., 0., 0.],
                reference_source='fixed commissioning quintic; not a FLASH prediction',
                plan_bounds=plan['bounds'])
        if not self.args.execute:
            return
        current = next((c for c in self.controllers() if c.name == CONTROLLER), None)
        if current:
            if current.state != 'inactive' or current.type != PLUGIN:
                raise RuntimeError('同名控制器状态或类型异常')
            response = self.service(UnloadController, '/controller_manager/unload_controller',
                                    UnloadController.Request(name=CONTROLLER))
            if not response.ok:
                raise RuntimeError('旧力矩保持实例卸载失败')
        plan_id = uuid.uuid4().hex
        params = {'expected_start_q': initial.tolist(), 'segment_durations': durations,
            'segment_coefficients': coefficients.ravel().tolist(), 'plan_id': plan_id, 'allow_activation': True,
            'k_gains': KP, 'd_gains': KD, 'torque_limits_nm': LIMITS, 'torque_rates_nm_s': RATES}
        path = self.args.output/'controller_params.yaml'
        path.write_text(json.dumps({CONTROLLER: {'ros__parameters': params}}, indent=2)+'\n')
        loaded = subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', CONTROLLER,
            '--controller-type', PLUGIN, '--param-file', str(path), '--inactive',
            '--controller-manager-timeout', '10'], capture_output=True, text=True, timeout=25)
        if loaded.returncode:
            raise RuntimeError('配置力矩控制器失败：'+loaded.stdout+loaded.stderr)
        self.inactive()
        self.verify_parameters(params)
        state = self.stable()
        gap = float(abs(state['q']-initial).max())
        if gap > .002:
            raise RuntimeError('配置期间起点发生变化，拒绝激活')
        self.report.update(plan_id=plan_id, max_start_gap_rad=gap)
        deadline = time.monotonic()+2
        while not self.heartbeat.get_subscription_count() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.005)
        if not self.heartbeat.get_subscription_count():
            raise RuntimeError('未发现本次力矩心跳端点')
        description = ('即将进行力矩解析速度测试：第4关节参考 +0.001 rad，3秒运动、0.75秒保持；夹爪不动，不自动返回。'
                       if self.velocity_probe else
                       '即将进行 2.75 秒力矩定点保持：参考位置不变，夹爪不动；此入口不执行 FLASH 任务。')
        if self.settled_probe:
            description = '即将进行同次激活诊断：先保持1.5秒，末段稳定才启动原 +0.001 rad 微动；增益和曲线不变，夹爪不动。'
        if self.sdk_audit:
            description += ' 同时记录 SDK 反馈力矩；遥测缺失则不开始微动。'
        print(description, flush=True)
        self.rows.clear()
        self.sdk_rows.clear()
        self.requested_activation = True
        # Effort activation can produce physical motion despite zero target
        # displacement. Record it as commanded actuation, not a read-only hold.
        self.report.update(activation_requested=True, effort_control_requested=True, robot_motion_commanded=True,
                           reference_displacement_commanded=self.velocity_probe)
        self.switch(True)
        start = time.monotonic()
        next_beat = start+self.prehold_seconds
        first_beat = None
        completed = False
        while time.monotonic()-start < 6:
            now = time.monotonic()
            if now >= next_beat:
                if first_beat is None:
                    if self.settled_probe:
                        if not self.rows or now-self.rows[-1]['receipt_monotonic'] > .1:
                            raise RuntimeError('预保持状态过期，不开始微动')
                        self.report['pre_motion_baseline'] = baseline_window(self.rows)
                        if self.sdk_audit:
                            self.report['sdk_pre_motion_baseline'] = sdk_prehold_window(self.sdk_rows)
                    first_beat = now
                    self.report['activation_to_first_heartbeat_seconds'] = now-start
                self.heartbeat.publish(Header(stamp=self.clock.now().to_msg(), frame_id=plan_id))
                next_beat = now+.02
            rclpy.spin_once(self.node, timeout_sec=.002)
            self.state()
            if self.bad_status:
                raise RuntimeError('力矩控制器状态格式异常')
            if self.sdk_audit:
                if self.bad_sdk_status:
                    raise RuntimeError('SDK 力矩遥测格式异常')
                if not self.sdk_rows:
                    if now-start > .5:
                        raise RuntimeError('未收到 SDK 力矩遥测，不开始微动；检查诊断插件是否已更新')
                elif now-self.sdk_rows[-1]['receipt_monotonic'] > .15:
                    raise RuntimeError('SDK 力矩遥测停止更新')
            if not self.rows:
                if now-start > .5:
                    raise RuntimeError('激活后没有收到力矩控制器状态')
                continue
            latest = self.rows[-1]
            data = latest['data']
            if now-latest['receipt_monotonic'] > .15:
                raise RuntimeError('力矩控制器状态停止更新')
            if int(data[1]) in (2, 3, 5) or int(data[2]) or int(data[53]):
                raise RuntimeError(f'力矩保持停车/故障：phase={data[1]}, reason={data[2]}, torque_fault={data[53]}')
            if int(data[1]) == 4:
                controller = next((c for c in self.controllers() if c.name == CONTROLLER), None)
                if controller is not None and controller.state == 'inactive':
                    completed = True
                    break
        if not completed:
            raise RuntimeError('未观察到力矩保持正常完成并自行停用')
        final = self.stable()['q']
        data = np.asarray([r['data'] for r in self.rows])
        tracking = abs(data[:, 4:11]-data[:, 18:25])
        velocity = abs(data[:, 25:32]).max(axis=0)
        self.report.update(status=self.prefix+'_FINISHED', final_q_rad=final.tolist(),
            native_deactivation_observed=True,
            measured_per_joint_excursion_rad=abs(data[:, 4:11]-data[0, 4:11]).max(axis=0).tolist(),
            per_joint_peak_command_nm=abs(data[:, 39:46]).max(axis=0).tolist(),
            per_joint_tracking_mae_rad=tracking.mean(axis=0).tolist(),
            per_joint_peak_tracking_error_rad=tracking.max(axis=0).tolist(),
            per_joint_peak_reference_velocity_rad_s=velocity.tolist(),
            per_joint_peak_target_velocity_term_nm=(velocity*np.asarray(KD)).tolist(),
            sampled_rate_limited_status_rows=int((data[:, 46:53] > .5).any(axis=1).sum()),
            final_max_gap_rad=float(abs(final-initial).max()))
        if self.velocity_probe:
            self.report['nonzero_analytic_velocity_observed'] = bool(velocity[3] > 1e-5)
            if not self.report['nonzero_analytic_velocity_observed']:
                raise RuntimeError('测试虽已停用，但未观察到非零解析目标速度；不能记为通过')
        if self.settled_probe:
            self.report['incremental_response'] = incremental_response(
                self.rows, self.report['pre_motion_baseline'])
        if self.sdk_audit:
            self.report['sdk_torque_audit'] = sdk_torque_summary(self.sdk_rows)

    def close(self):
        errors = []
        if self.requested_activation:
            # No more heartbeat: the independent native brake/stop path owns
            # the first response. Client cleanup only stops this controller.
            deadline = time.monotonic()+.9
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=.005)
            try:
                controller = next((c for c in self.controllers() if c.name == CONTROLLER), None)
                if controller and controller.state == 'active':
                    self.report['client_deactivation_requested'] = True
                    self.switch(False)
            except Exception as error:
                errors.append(str(error))
        try:
            # A missing capability can reject preflight while all commands
            # remain released. Report that cleanup state independently.
            self.report['controllers_after'] = self.inactive(require_capabilities=False)
            self.report.update(controller_left_inactive=True, all_command_interfaces_unclaimed=True)
        except Exception as error:
            errors.append(str(error))
        if errors:
            self.report.update(status=self.prefix+'_FAILED', cleanup_errors=errors)
        (self.args.output/'status.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in self.rows))
        (self.args.output/'sdk_status.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in self.sdk_rows))
        (self.args.output/'report.json').write_text(json.dumps(self.report, ensure_ascii=False, indent=2)+'\n')
        print('FLASH_'+self.prefix+'_RESULT='+json.dumps(self.report, ensure_ascii=False), flush=True)
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--velocity-probe', action='store_true',
                        help='固定第4关节 +0.001 rad 解析速度测试；无 --execute 时只读检查')
    parser.add_argument('--settle-before-probe', action='store_true',
                        help='与 --velocity-probe 同用；在原生2秒等待超时内预保持1.5秒，不改变轨迹或增益')
    parser.add_argument('--sdk-torque-audit', action='store_true',
                        help='与预保持微动同用；必须收到 SDK 力矩反馈后才允许开始原微动')
    args = parser.parse_args()
    if args.settle_before_probe and not args.velocity_probe:
        parser.error('--settle-before-probe 必须与 --velocity-probe 同用')
    if args.sdk_torque_audit and not (args.velocity_probe and args.settle_before_probe):
        parser.error('--sdk-torque-audit 必须与 --velocity-probe、--settle-before-probe 同用')
    if args.output.exists():
        parser.error('输出目录须不存在')
    args.output.mkdir(parents=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    runner = TorqueHold(args)
    try:
        runner.run()
    except (Exception, KeyboardInterrupt) as error:
        runner.report.update(status=runner.prefix+'_FAILED', error=str(error) or '用户中断')
    finally:
        runner.close()
        rclpy.try_shutdown()
    if runner.report['status'] not in (runner.prefix+'_PREFLIGHT_PASS', runner.prefix+'_FINISHED'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
