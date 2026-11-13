#!/usr/bin/env python3
"""Save/check a joint pose; only --execute activates a bounded physical return.

Uses the already commissioned native TaskController and its independent
heartbeat brake. No camera or H100 request is needed. Successful execution
opens the gripper at the saved pose. Joint interpolation does not perform
collision planning against the table or objects.
"""
import argparse
import json
import socket
import subprocess
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from controller_manager_msgs.srv import (ListControllerTypes, LoadController,
                                        ConfigureController, UnloadController)
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from capture_flash_fr3_observation import stamp_seconds
from flash_fr3_model_plan import write_preview_csv
from flash_fr3_return_plan import (JOINT_NAMES, MAX_SEGMENTS, RETURN_TOLERANCE_RAD,
                                   load_pose, make_return_segment, return_waypoints)
from run_flash_fr3_model_prefix import call
from run_flash_fr3_pickplace import (TaskRunner, CONTROLLER, PLUGIN, PREVIEW,
                                    controller_parameters, save)
from run_flash_fr3_shadow_client import ObservationBuffer


class ArmStateBuffer(ObservationBuffer):
    def __init__(self, node, local_keys=None):
        super().__init__(node, keys=('arm', 'hand'))

    def close(self):
        pass  # Subscriptions are destroyed with their owner node; no child process.


class ReturnRunner(TaskRunner):
    def __init__(self, args):
        super().__init__(args, buffer_factory=ArmStateBuffer)
        self.result_prefix = 'FLASH_RETURN_RESULT'
        self.report.update(status='RETURN_PENDING', scope='return to saved joint pose',
                           execution_style='bounded rest-to-rest joint interpolation',
                           max_wall_seconds=240., max_arm_segments=MAX_SEGMENTS,
                           pose_file=str(args.pose), gripper_policy='open_after_successful_return',
                           return_gripper_width_m=.08, return_gripper_speed_m_s=.1,
                           collision_clearance_checked=False,
                           final_joint_tolerance_rad=RETURN_TOLERANCE_RAD)
        self.return_parameters = self.node.create_client(SetParameters, '/controller_manager/set_parameters')
        self.return_load = self.node.create_client(LoadController, '/controller_manager/load_controller')
        self.return_configure = self.node.create_client(ConfigureController, '/controller_manager/configure_controller')

    def configure(self, plan, directory):
        # Reuse this ROS node instead of starting/discovering a spawner process
        # for every leg. The same parameter file and readback remain in use.
        self.inactive()
        current = next((c for c in self.controllers() if c.name == CONTROLLER), None)
        if current and (current.state != 'inactive' or current.type != PLUGIN):
            raise RuntimeError('同名任务控制器异常')
        if current and not call(self.node, self.unload, UnloadController.Request(name=CONTROLLER)).ok:
            raise RuntimeError('旧任务控制器卸载失败')
        path = directory/'controller_params.yaml'
        save(path, {CONTROLLER: {'ros__parameters': controller_parameters(plan)}})
        request = SetParameters.Request(parameters=[
            Parameter(CONTROLLER+'.type', value=PLUGIN).to_parameter_msg(),
            Parameter(CONTROLLER+'.params_file', value=[str(path)]).to_parameter_msg()])
        result = call(self.node, self.return_parameters, request)
        if len(result.results) != 2 or not all(r.successful for r in result.results):
            raise RuntimeError('回位控制器参数配置失败')
        if not call(self.node, self.return_load, LoadController.Request(name=CONTROLLER), seconds=10).ok:
            raise RuntimeError('加载回位控制器失败')
        if not call(self.node, self.return_configure, ConfigureController.Request(name=CONTROLLER)).ok:
            raise RuntimeError('配置回位控制器失败')
        self.inactive()
        self.verify_configured_plan(plan)

    def open_after_return(self):
        if not self.args.execute:
            return
        self.inactive()
        self.gripper.ready()
        self.report['gripper_motion_commanded'] = True
        try:
            action = self.gripper.perform('open', {'gripper_speed_m_s': .1})
        finally:
            if self.gripper.last_action is not None:
                self.report['gripper_actions'].append(dict(self.gripper.last_action))
        completed = time.monotonic()
        completed_wall = time.time()
        deadline = completed+2.
        last_error = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.005)
            try:
                with self.buffer.lock:
                    latest = self.buffer.history['hand'][-1] if self.buffer.history['hand'] else None
                if latest is None or latest[1] <= completed or stamp_seconds(latest[0]) < completed_wall:
                    continue
                self.report['final_gripper_width_m'] = self.width()
                self.report['gripper_open_success'] = action['success']
                return
            except ValueError as error:
                last_error = error
        raise RuntimeError('张开后未取得新鲜夹爪状态：'+str(last_error))

    def stable_state(self):
        deadline = time.monotonic()+4
        beginning = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=.003)
            try:
                state = self.state(static=True)
                if beginning is None:
                    beginning = time.monotonic()
                if time.monotonic()-beginning >= .3:
                    return state
            except ValueError:
                beginning = None
        raise RuntimeError('未取得连续静止的新鲜机械臂状态')

    def verify_plan(self, plan, directory, keep_csv):
        directory.mkdir()
        save(directory/'plan.json', plan)
        data = plan['segment_durations']+np.asarray(plan['coefficients']).ravel().tolist()+plan['expected_start_q']
        (directory/'preview_input.txt').write_text(' '.join(map(str, data))+'\n')
        result = subprocess.run([PREVIEW, str(directory/'preview_input.txt'),
                                 str(directory/'reference_cpp.csv')],
                                capture_output=True, text=True, timeout=5, check=True)
        save(directory/'native_bounds.json', json.loads(result.stdout))
        write_preview_csv(directory/'reference_python.csv', plan)
        a = np.loadtxt(directory/'reference_cpp.csv', delimiter=',', skiprows=1)
        b = np.loadtxt(directory/'reference_python.csv', delimiter=',', skiprows=1)
        if a.shape != b.shape or abs(a-b).max() > 1e-9:
            raise RuntimeError('回位 C++/Python 曲线求值不一致')
        save(directory/'verification.json', {'status': 'PASS', 'max_abs_error': float(abs(a-b).max())})
        if not keep_csv:
            (directory/'reference_cpp.csv').unlink()
            (directory/'reference_python.csv').unlink()

    def fresh_plan(self, q, target):
        plan = make_return_segment(q, target)
        plan.update(plan_id=uuid.uuid4().hex, created_monotonic=time.monotonic())
        return plan

    def validate_return_activation(self, plan, directory):
        if plan.get('format')!='flash_fr3_return_segment_v1':
            raise RuntimeError('只有固定目标回位轨迹允许执行前重新验证')
        # Configuration/discovery can outlast a visual prediction's lifetime.
        # A fixed return target has no camera anchor. Recheck the configured
        # coefficients and fresh stationary start before granting a short
        # activation window; preserve the original generation timestamp.
        self.inactive()
        self.verify_configured_plan(plan)
        state=self.stable_state()
        gap=float(abs(state['q']-plan['expected_start_q']).max())
        if gap>.002:
            raise RuntimeError('回位段起点在配置期间发生变化，拒绝激活')
        checked=time.monotonic()
        plan['activation_validated_monotonic']=checked
        save(directory/'activation_validation.json',{
            'status':'PASS','plan_id':plan['plan_id'],
            'created_monotonic':plan['created_monotonic'],
            'activation_validated_monotonic':checked,
            'generation_to_validation_seconds':checked-plan['created_monotonic'],
            'measured_q_rad':state['q'].tolist(),'max_start_gap_rad':gap,
            'controller_parameters_match':True,'controller_inactive':True,
            'activation_window_seconds':5.,'scope':'fixed saved-pose return only; no model observation'})

    def activation_timestamp(self, plan):
        if plan.get('format')!='flash_fr3_return_segment_v1' or 'activation_validated_monotonic' not in plan:
            raise RuntimeError('回位段缺少执行前状态及配置验证')
        return plan['activation_validated_monotonic']

    def run(self):
        self.report['controllers_before'] = self.inactive()
        state = self.stable_state()
        if self.args.save_pose:
            pose = {'format': 'flash_fr3_saved_joint_pose_v1', 'robot_ip': '172.16.0.2',
                    'joint_names': JOINT_NAMES, 'q_rad': state['q'].tolist(), 'units': 'rad',
                    'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
                    'hostname': socket.gethostname(), 'source_topic': '/franka/joint_states',
                    'gripper_policy': 'open_after_successful_return', 'robot_motion_commanded': False,
                    'gripper_motion_commanded': False}
            self.inactive()
            with self.args.pose.open('x') as out:
                out.write(json.dumps(pose, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
            self.report.update(status='POSE_SAVED', saved_pose=pose)
            return
        pose = json.loads(self.args.pose.read_text())
        target = load_pose(pose)
        q = state['q']
        self.report.update(target_q_rad=target.tolist(), initial_q_rad=q.tolist(),
                           initial_max_gap_deg=float(np.rad2deg(abs(q-target).max())))
        points = return_waypoints(q, target)
        self.report['planned_segments'] = len(points)
        if not points:
            self.report.update(status='ALREADY_AT_SAVED_POSE', final_q_rad=q.tolist(),
                               final_max_gap_rad=float(abs(q-target).max()), arm_return_completed=True)
            self.open_after_return()
            return
        if PLUGIN not in call(self.node, self.types, ListControllerTypes.Request()).types:
            raise RuntimeError('既有任务控制器插件不可用')
        # Validate the complete proposed path before starting any motion.
        nominal_seconds = 0.
        for i, point in enumerate(points):
            plan = self.fresh_plan(q, point)
            self.verify_plan(plan, self.args.output/f'preview_{i:03d}', keep_csv=False)
            nominal_seconds += plan['total_motion_seconds']
            q = point
        self.inactive()
        state = self.stable_state()
        if abs(state['q']-self.report['initial_q_rad']).max() > .002:
            raise RuntimeError('检查期间起点发生变化，请重新运行')
        self.report.update(status='RETURN_PREFLIGHT_PASS', nominal_motion_seconds=nominal_seconds)
        if not self.args.execute:
            return
        print('即将快速返回保存的关节姿态；成功到位并停用机械臂控制器后自动张开夹爪至80毫米。此入口不检测桌面或物体碰撞。', flush=True)
        started = time.monotonic()
        # Follow the checked waypoints. Each segment is rebuilt from fresh
        # stationary feedback and checked again in Python and native C++.
        for i in range(MAX_SEGMENTS):
            self.inactive()
            q = self.stable_state()['q']
            if abs(q-target).max() <= RETURN_TOLERANCE_RAD:
                self.report.update(status='RETURN_FINISHED', final_q_rad=q.tolist(),
                                   final_max_gap_rad=float(abs(q-target).max()),
                                   elapsed_seconds=time.monotonic()-started, arm_return_completed=True)
                self.open_after_return()
                self.report['elapsed_seconds'] = time.monotonic()-started
                return
            destination = points[i] if i < len(points) else target
            plan = self.fresh_plan(q, destination)
            if time.monotonic()-started+plan['total_motion_seconds'] > 240:
                raise RuntimeError('达到回位时长上限')
            directory = self.args.output/f'segment_{i:03d}'
            self.verify_plan(plan, directory, keep_csv=True)
            self.configure(plan, directory)
            self.validate_return_activation(plan, directory)
            self.arm_segment(plan, directory)
            if abs(self.stable_state()['q']-destination).max() > .002:
                raise RuntimeError('本段终点误差超过 0.002 rad，停止回位')
        raise RuntimeError('达到回位段数上限')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pose', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--save', dest='save_pose', action='store_true')
    mode.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('输出目录须不存在')
    if args.save_pose and args.pose.exists():
        parser.error('保存姿态已存在，拒绝覆盖；请指定另一个文件名')
    args.output.mkdir(parents=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    runner = ReturnRunner(args)
    try:
        runner.run()
    except (Exception, KeyboardInterrupt) as error:
        runner.report.update(status='RETURN_FAILED', error=str(error) or '用户中断')
    finally:
        runner.close()
        rclpy.shutdown()
    if runner.report['status'] not in ('POSE_SAVED', 'ALREADY_AT_SAVED_POSE', 'RETURN_PREFLIGHT_PASS', 'RETURN_FINISHED'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
