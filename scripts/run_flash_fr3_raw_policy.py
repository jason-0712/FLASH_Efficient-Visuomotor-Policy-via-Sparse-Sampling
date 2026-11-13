#!/usr/bin/env python3
"""FLASH continuous pick-and-place; default is observation-only.

Uses the existing 15k EMA endpoint and real physical-time history. No history
freeze, coefficient correction, speed change or per-chunk stops. The gripper
follows the active polynomial while arm torque control continues.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import io
import json
from pathlib import Path
import subprocess
import time
import urllib.request
import uuid

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController, UnloadController
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout, Header

from flash_fr3_continuous import physical_history_after_handoff, physical_reply, raw_packet
from flash_fr3_continuous_effort import admission_input, effort_message
from flash_fr3_camera_clock import SDK_TIME_BASIS
from flash_fr3_sensor_process import IsolatedObservationBuffer
from flash_fr3_shadow_math import evaluate_curve
from flash_fr3_task_gripper import TaskGripper
from capture_flash_fr3_observation import FINGERS, ordered_positions, stamp_seconds
from flash_fr3_stream_io import connection
from run_flash_fr3_grasp_probe import query, require_inactive
from run_flash_fr3_guard_shadow import measured_state

CONTROLLER = 'flash_fr3_continuous_effort'
PLUGIN = 'flash_fr3_model_controller/ContinuousEffortController'
# Formal rollout settings are independent of the old micro-motion/hold budget.
# FLASH does not publish numerical gains. No policy output is corrected.
KP = [150., 150., 150., 150., 50., 30., 30.]
KD = (2*np.sqrt(KP)).tolist()
# Franka FR3 robot_specifications.html and installed libfranka rate_limiting.h.
# Torque amplitude is a manufacturer recommendation; rate is an FCI requirement.
LIMITS = [87., 87., 87., 87., 12., 12., 12.]
RATES = [1000.-1e-3]*7


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


class RawPolicyTrial:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node('flash_fr3_raw_policy_trial')
        self.buffer = None
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.pending = None
        self.gripper = None
        self.gripper_mode = "approach"
        self.active_packets = {}
        self.finish_requested = False
        self.rows = []
        self.bad_status = False
        self.activated = False
        self.session = uuid.uuid4().hex
        self.clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.sub = self.node.create_subscription(Float64MultiArray, '/'+CONTROLLER+'/status', self.status, 30)
        self.heartbeat = self.node.create_publisher(Header, '/'+CONTROLLER+'/heartbeat',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.finish = self.node.create_publisher(Header, "/"+CONTROLLER+"/finish", 1)
        self.candidate = self.node.create_publisher(Float64MultiArray, '/'+CONTROLLER+'/candidate', 1)
        self.report = dict(status='RAW_POLICY_PENDING', execution_requested=args.execute,
            robot_motion_commanded=False, gripper_motion_commanded=False,
            checkpoint_step=15000, weights='ema_model', maximum_session_seconds=args.seconds,
            execution_profile='raw_policy_rollout', time_scale=1., history_frozen=False,
            coefficients_corrected=False, intermediate_chunk_stops=False,
            startup_clock='first curve starts at action anchor; real source time retained; later curves compensate latency',
            update_observation_policy='latest real sources must follow the previous executed handoff',
            camera_time_basis=SDK_TIME_BASIS,
            scope='raw FLASH continuous arm-only rollout' if getattr(args,'arm_only',False) else 'autonomous pick-and-place attempt with raw FLASH polynomials',
            task_sequence_complete=False, physical_placement_success='requires observation', gripper_actions=[],
            rejected_updates=[],
            local_FR3_adaptations={'k_gains': KP, 'd_gains': KD, 'torque_limits_nm': LIMITS,
                'torque_rates_nm_s': RATES, 'tracking_limit_rad': None,
                'position_handoff_limit_rad': None, 'position_error_policy': 'record_only',
                'reference_velocity_acceleration_jerk_limits': [.8, 10., 100.],
                'prediction_deadline': 'physical curve horizon minus native braking time',
                'position_limits_source': 'active FR3 URDF and manufacturer robot_specifications.html',
                'torque_limits_source': 'FR3 manufacturer recommended amplitude: 87 Nm joints 1-4, 12 Nm joints 5-7; reject at boundary',
                'torque_rates_source': 'libfranka kMaxTorqueRate = 1000 - kLimitEps = 999.999 Nm/s; original driver limitRate remains enabled',
                'gain_revision': 'raw_rollout_j7_kp30_kd_2sqrtkp_20261111',
                'description': 'local gains/reference budgets and manufacturer torque constraints; no inherited commissioning torque budget; original Franka protection remains enabled'})

    def service(self, typ, endpoint, request):
        return query(self.node, typ, endpoint, request)

    def controllers(self):
        return self.service(ListControllers, '/controller_manager/list_controllers',
                            ListControllers.Request()).controller

    def status(self, msg):
        a = np.asarray(msg.data, float)
        if a.shape not in ((66,), (70,)) or not np.isfinite(a).all():
            self.bad_status = True
        else:
            self.rows.append({'receipt_monotonic': time.monotonic(), 'data': a.tolist()})

    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=.002)

    def state(self):
        state = measured_state(self.buffer)
        elapsed = time.monotonic()-state['receipt_monotonic']
        if not 0 <= elapsed <= .1 or not -.05 <= state['source_age_at_receipt_seconds']+elapsed <= .1:
            raise ValueError('机械臂状态过期')
        return state

    def fresh_start(self):
        deadline = time.monotonic()+1.
        while time.monotonic() < deadline:
            self.spin()
            try:
                state = self.state()
                if abs(state['dq']).max() > .01:
                    raise ValueError('机械臂尚未静止')
                return state['q']
            except ValueError:
                pass
        raise RuntimeError('未取得新鲜静止起点')

    def infer(self, sequence, previous_anchor=None, label=None, minimum_anchor_monotonic=None):
        deadline = time.monotonic()+(15 if previous_anchor is None and not self.activated else 1.)
        while True:
            try:
                arrays, metadata = self.buffer.observation(sequence)
                anchor = physical_history_after_handoff(metadata, previous_anchor,
                    minimum_anchor_monotonic, time.time()-time.monotonic())
                break
            except ValueError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.005)
        offset = time.time()-time.monotonic()
        blob = io.BytesIO()
        np.savez(blob, **arrays, metadata_json=np.asarray(json.dumps(metadata)))
        request = urllib.request.Request(self.args.server+'/predict', data=blob.getvalue(),
                                        headers={'Content-Type': 'application/octet-stream'})
        with self.opener.open(request, timeout=.8) as response:
            reply = json.load(response)
        physical_reply(reply, sequence, anchor, 15000)
        label = label or f'{sequence:03d}'
        save(self.args.output/f'prediction_{label}.json', reply)
        save(self.args.output/f'metadata_{label}.json', metadata)
        np.savez(self.args.output/f'observation_{label}.npz', **arrays)
        # Large image archives must not consume the command's 50 ms lead.
        # Source timestamps stay unchanged; their age is checked again here.
        ready = time.monotonic()
        packet = raw_packet(reply, ready, offset, start_at_anchor=(sequence == 0), continuous_update=True)
        packet['minimum_observation_monotonic'] = minimum_anchor_monotonic
        save(self.args.output/f'packet_{label}.json', packet)
        return packet, anchor

    def await_prediction(self, sequence, label=None):
        self.pending = self.pool.submit(self.infer, sequence, None, label)
        while not self.pending.done():
            self.spin()
        return self.pending.result()

    def switch(self, activate):
        request = SwitchController.Request(strictness=SwitchController.Request.STRICT,
            activate_controllers=[CONTROLLER] if activate else [],
            deactivate_controllers=[] if activate else [CONTROLLER],
            activate_asap=True, timeout=Duration(sec=3))
        if not self.service(SwitchController, '/controller_manager/switch_controller', request).ok:
            raise RuntimeError('持续力矩控制器切换失败')

    def configure(self, initial):
        current = next((c for c in self.controllers() if c.name == CONTROLLER), None)
        if current is not None:
            if current.state != 'inactive' or current.type != PLUGIN:
                raise RuntimeError('同名控制器状态或类型异常')
            if not self.service(UnloadController, '/controller_manager/unload_controller',
                                UnloadController.Request(name=CONTROLLER)).ok:
                raise RuntimeError('不能卸载旧的 inactive 会话')
        params = dict(expected_start_q=initial.tolist(), session_id=self.session,
            maximum_session_seconds=self.args.seconds, allow_activation=self.args.execute,
            execution_profile='raw_policy_rollout', k_gains=KP, d_gains=KD,
            torque_limits_nm=LIMITS, torque_rates_nm_s=RATES)
        path = self.args.output/'controller_params.yaml'
        save(path, {CONTROLLER: {'ros__parameters': params}})
        result = subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', CONTROLLER,
            '--controller-type', PLUGIN, '--param-file', str(path), '--inactive',
            '--controller-manager-timeout', '10'], capture_output=True, text=True, timeout=25)
        (self.args.output/'spawner.log').write_text(result.stdout+result.stderr)
        if result.returncode:
            raise RuntimeError('持续力矩控制器配置失败；见 spawner.log')
        require_inactive(self.node)
        values = self.service(GetParameters, '/'+CONTROLLER+'/get_parameters',
                             GetParameters.Request(names=list(params))).values
        if len(values) != len(params):
            raise RuntimeError('控制器参数回复不完整')
        for (key, expected), actual in zip(params.items(), values):
            if isinstance(expected, bool):
                okay = actual.type == 1 and actual.bool_value == expected
            elif isinstance(expected, str):
                okay = actual.type == 4 and actual.string_value == expected
            elif isinstance(expected, float):
                okay = actual.type == 3 and actual.double_value == expected
            else:
                okay = actual.type == 8 and np.array_equal(actual.double_array_value, expected)
            if not okay:
                raise RuntimeError('控制器参数不一致：'+key)

    def check_admission(self, packet, initial, label):
        result = subprocess.run([str(self.args.native), '--task-rollout'],
            input=admission_input([packet], initial, time.time()-time.monotonic(), self.session, self.args.seconds),
            text=True, capture_output=True, timeout=5)
        if result.returncode:
            raise RuntimeError('原始模型任务准入求值失败：'+result.stderr)
        admission = json.loads(result.stdout)
        save(self.args.output/(label+'_admission.json'), admission)
        return admission['events'][0]

    def publish_candidate(self, candidate):
        encoded = effort_message(candidate, self.session, time.time())
        dim = MultiArrayDimension(**encoded['layout']['dim'][0])
        self.candidate.publish(Float64MultiArray(layout=MultiArrayLayout(dim=[dim]), data=encoded['data']))
        self.active_packets[candidate['sequence']] = candidate

    def gripper_width(self):
        with self.buffer.lock:
            if not self.buffer.history['hand']:raise RuntimeError('缺少夹爪状态')
            message,receipt,wall=self.buffer.history['hand'][-1]
        if time.monotonic()-receipt>.3 or not -.05<=wall-stamp_seconds(message)<=.3:
            raise RuntimeError('夹爪状态过期')
        width=float(ordered_positions(message,FINGERS).sum())
        if not np.isfinite(width) or not -.001<=width<=.081:raise RuntimeError('非法夹爪开度')
        return width

    def update_gripper(self, status):
        if self.gripper is None:return
        result=self.gripper.poll()
        if result is not None:
            result['measured_width_m']=self.gripper_width()
            self.report['gripper_actions'].append(result)
            print('FLASH_TASK_GRIPPER='+json.dumps(result,ensure_ascii=False),flush=True)
            if result['kind']=='grasp':self.gripper_mode='carrying'
            elif result['kind']=='release':
                self.gripper_mode='released';self.report['task_sequence_complete']=True
                self.finish.publish(Header(stamp=self.clock.now().to_msg(),frame_id=self.session))
                self.finish_requested=True
        if self.gripper.may_be_active or self.finish_requested or int(status[1])!=1:return
        packet=self.active_packets.get(int(status[54]))
        if packet is None:return
        origin=packet.get('curve_origin_monotonic',packet['observation_monotonic'])
        elapsed=status[0]-origin
        if not 0<=elapsed<=packet['curve']['valid_future_seconds']:raise RuntimeError('夹爪参考已过期')
        closure=float(np.clip(evaluate_curve(packet['curve'],elapsed)[0][7],0.,1.))
        kind=None
        if self.gripper_mode=='approach':
            if closure>=.7:kind='grasp'
            elif closure<=.3 and self.gripper_width()<.075:kind='open'
        elif self.gripper_mode=='carrying' and closure<=.3:kind='release'
        if kind is not None:
            self.gripper.begin(kind,self.gripper_config)
            self.report['gripper_motion_commanded']=True

    def run(self):
        self.report['controllers_before'] = require_inactive(self.node)
        self.opener, self.report['server'] = connection(self.args.server)
        if not getattr(self.args,'arm_only',False):
            self.gripper_config=json.loads(self.args.events.read_text())
            if not .0<=self.gripper_config['gripper_target_width_m']<=.08:raise ValueError('非法目标夹爪开度')
            self.report['gripper_config']={k:self.gripper_config[k] for k in
                ('gripper_target_width_m','gripper_epsilon_m','gripper_force_N','gripper_speed_m_s')}
            self.gripper=TaskGripper(self.node);self.gripper.ready()
        self.buffer = IsolatedObservationBuffer(self.node, local_keys=('arm', 'hand'),camera_time_basis=SDK_TIME_BASIS)
        self.report['receiver'] = self.buffer.receiver_info
        packet, _ = self.await_prediction(0, 'preflight')
        initial = self.fresh_start()
        self.report['preflight_measured_q_rad'] = initial.tolist()
        first = self.check_admission(packet, initial, 'preflight')
        self.report['first_prediction_admission'] = first
        if not first['accepted']:
            self.report['status'] = 'RAW_POLICY_START_REJECTED'
            self.report['reason'] = {5:'原始参考或停车范围超出关节或导数运行边界', 6:'位置接入被拒绝（当前正式任务版本不启用此检查）',
                                     7:'速度切换被拒绝', 3:'预测时间已过期'}.get(first['rejection'], '原生准入拒绝')
            return
        self.report['status'] = 'RAW_POLICY_PREFLIGHT_READY'
        if not self.args.execute:
            return
        self.configure(initial)
        deadline = time.monotonic()+3
        while (not self.candidate.get_subscription_count() or not self.heartbeat.get_subscription_count()):
            if time.monotonic() > deadline:
                raise RuntimeError('持续力矩会话端点未就绪')
            self.spin()
        # A fresh runtime prediction must be usable before taking command
        # interfaces. Camera failure must not start a waiting torque hold.
        startup, last_anchor = self.await_prediction(0, 'startup')
        actual = self.fresh_start()
        if np.max(abs(actual-initial)) > .002:
            raise RuntimeError('配置期间起点发生变化')
        admitted = self.check_admission(startup, actual, 'startup')
        self.report['startup_prediction_admission'] = admitted
        if not admitted['accepted']:
            raise RuntimeError('激活前最新预测被原生准入拒绝：'+str(admitted['rejection']))
        if startup['handoff_monotonic']-time.monotonic() < .08:
            raise RuntimeError('激活前首段调度余量不足；未激活控制器')
        print(f'即将执行原始 FLASH 连续任务（最长 {self.args.seconds:g} 秒）；'+
              ('夹爪不动。' if self.gripper is None else '模型控制抓取与释放，机械臂持续更新。'),flush=True)
        self.rows.clear()
        self.activated = True
        self.report['robot_motion_commanded'] = True
        self.switch(True)
        start = time.monotonic(); next_beat = start; last_published = 0
        self.publish_candidate(startup)
        self.pending = None
        last_printed = None; last_rejection = -1; stop_new_predictions = False
        while time.monotonic()-start < self.args.seconds+2.:
            now = time.monotonic()
            if now >= next_beat:
                self.heartbeat.publish(Header(stamp=self.clock.now().to_msg(), frame_id=self.session))
                next_beat = now+.02
            self.spin()
            self.state()
            if self.bad_status:
                raise RuntimeError('持续控制器状态格式异常')
            if not self.rows:
                if now-start > .5:
                    raise RuntimeError('持续控制器状态缺失')
                continue
            latest = self.rows[-1]; d = latest['data']
            if now-latest['receipt_monotonic'] > .15:
                raise RuntimeError('持续控制器状态停止更新')
            phase = (int(d[1]), int(d[2]), int(d[54]))
            if phase != last_printed:
                print('FLASH_RAW_POLICY_PHASE='+json.dumps(dict(phase=phase[0], reason=phase[1], sequence=phase[2])), flush=True)
                last_printed = phase
            if d[53] or int(d[1]) == 5:
                tracking = np.asarray(d[18:25])-np.asarray(d[4:11])
                self.report['protection_event'] = dict(torque_fault=int(d[53]),
                    stream_reason=int(d[2]),sequence=int(d[54]),handoffs=int(d[58]),
                    tracking_error_rad=tracking.tolist(),
                    tracking_limit_rad=None,position_error_policy='record_only',
                    torque_status_note='torque columns retain the last successful evaluation when the current evaluation fails')
                if len(d) == 70:
                    self.report['protection_event']['clock'] = dict(zip(
                        ('sdk_period_seconds','converted_seconds_difference','host_period_seconds','cumulative_clock_difference_seconds'),d[66:70]))
                raise RuntimeError(f'持续力矩控制器停车（本机检查）：reason={int(d[2])}, torque_fault={int(d[53])}')
            if int(d[63]):
                raise RuntimeError(f'持续力矩接收端拒绝了协议或曲线准备异常消息：invalid_packets={int(d[63])}, '
                                   f'最后发送序号={last_published}, 最后处理序号={int(d[56])}')
            self.update_gripper(d)
            if int(d[57]) and int(d[56]) == last_published and last_rejection != last_published:
                # A rejected candidate has never replaced the current curve.
                # Continue that admitted curve and request genuinely newer
                # observations; the native curve deadline remains unchanged.
                # Malformed/protocol/out-of-range commands still fail closed.
                event=dict(sequence=last_published,code=int(d[57]),q_gap_rad=d[61],dq_gap_rad_s=d[62])
                if last_published == 0 or int(d[57]) not in (3,6):
                    raise RuntimeError('原始模型更新被拒绝：'+json.dumps(event))
                self.report['rejected_updates'].append(event);last_rejection=last_published
                print('FLASH_RAW_UPDATE_DISCARDED='+json.dumps(event),flush=True)
            if int(d[1]) in (3, 4):
                current = next((c for c in self.controllers() if c.name == CONTROLLER), None)
                if current is not None and current.state == 'inactive':
                    if int(d[1]) != 4:
                        reason=int(d[2])
                        detail={1:'心跳中断',2:'首段等待超时',8:'当前模型曲线到期，未接入后续预测'}.get(reason,'持续控制器异常停车')
                        raise RuntimeError(detail+'，reason='+str(reason))
                    a = np.asarray([row['data'] for row in self.rows])
                    if self.gripper is not None and not self.report['task_sequence_complete']:
                        raise RuntimeError('任务时间预算结束，模型尚未完成抓取与释放')
                    self.report.update(status='TASK_SEQUENCE_FINISHED' if self.gripper is not None else 'RAW_POLICY_TRIAL_FINISHED',
                        handoffs_observed=int(a[:,58].max()), native_deactivation_observed=True,
                        joint_tracking_mae_rad=abs(a[:,4:11]-a[:,18:25]).mean(axis=0).tolist(),
                        peak_analytic_reference_velocity_rad_s=abs(a[:,25:32]).max(axis=0).tolist(),
                        sampled_rate_limited_status_rows=int(np.any(a[:,46:53]>.5,axis=1).sum()))
                    return
                continue
            if self.finish_requested:stop_new_predictions=True
            if self.pending is not None and self.pending.done():
                candidate, last_anchor = self.pending.result()
                if self.finish_requested or candidate['handoff_monotonic'] > start+self.args.seconds-.6:
                    stop_new_predictions = True
                else:
                    self.publish_candidate(candidate)
                    last_published = candidate['sequence']
                self.pending = None
            if (self.pending is None and int(d[54]) >= 0 and int(d[56]) == last_published and int(d[55]) == -1 and
                    not stop_new_predictions and now-start < self.args.seconds-.8):
                active=self.active_packets[int(d[54])]
                self.pending = self.pool.submit(self.infer, last_published+1, last_anchor,
                    minimum_anchor_monotonic=active['handoff_monotonic'])
        raise RuntimeError('未在期限内观察到原生停用')

    def close(self):
        errors = []
        if self.activated:
            deadline = time.monotonic()+.9
            while time.monotonic() < deadline:
                self.spin()
            try:
                current = next((c for c in self.controllers() if c.name == CONTROLLER), None)
                if current is not None and current.state == 'active':
                    self.report['client_deactivation_requested'] = True
                    self.switch(False)
            except Exception as error:
                errors.append(str(error))
        if self.gripper is not None:
            try:self.gripper.stop_if_needed()
            except Exception as error:errors.append(str(error))
            if self.gripper.last_action is not None and not self.gripper.last_action['success']:
                self.report['gripper_actions'].append(dict(self.gripper.last_action))
        self.pool.shutdown(wait=True, cancel_futures=True)
        if self.buffer is not None:
            self.report['sensor_history_coverage_failures'] = self.buffer.coverage_failures
            self.buffer.close()
        try:
            self.report['controllers_after'] = require_inactive(self.node)
            self.report['all_command_interfaces_unclaimed'] = True
        except Exception as error:
            errors.append(str(error))
        if errors:
            self.report.update(status='RAW_POLICY_TRIAL_FAILED', cleanup_errors=errors)
        (self.args.output/'status.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in self.rows))
        save(self.args.output/'report.json', self.report)
        print('FLASH_RAW_POLICY_RESULT='+json.dumps(self.report, ensure_ascii=False), flush=True)
        self.node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--seconds', type=float, default=120.,help='整任务超时预算，非轨迹慢放')
    parser.add_argument('--arm-only', action='store_true',help='显式保留旧的仅机械臂检查方式')
    parser.add_argument('--events',type=Path,default=Path(__file__).with_name('events.json'))
    parser.add_argument('--server', default='http://192.168.1.113:10132')
    parser.add_argument('--native', type=Path, default=Path('/home/ros/ros2_ws/build/flash_fr3_model_controller/preview_continuous_effort'))
    args = parser.parse_args()
    if args.output.exists() or not np.isfinite(args.seconds) or args.seconds < 1.5 or not args.native.is_file():
        parser.error('需要新目录、已构建的求值器和 至少 1.5 秒的有限任务预算')
    args.output.mkdir(parents=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    trial = RawPolicyTrial(args)
    try:
        trial.run()
    except (Exception, KeyboardInterrupt) as error:
        trial.report.update(status='RAW_POLICY_TRIAL_FAILED', error=str(error) or '用户中断')
    finally:
        trial.close()
        rclpy.try_shutdown()
    if trial.report['status'] not in ('RAW_POLICY_PREFLIGHT_READY', 'RAW_POLICY_TRIAL_FINISHED', 'TASK_SEQUENCE_FINISHED'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
