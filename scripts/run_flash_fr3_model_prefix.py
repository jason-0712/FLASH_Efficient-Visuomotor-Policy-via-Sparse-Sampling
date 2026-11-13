#!/usr/bin/env python3
"""Default: read-only preflight. --execute activates a REAL frozen FLASH prefix."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header

from capture_flash_fr3_observation import stamp_seconds
from flash_fr3_guard import joint_state_vectors


def call(node, client, request, seconds=5):
    if not client.wait_for_service(timeout_sec=seconds):
        raise RuntimeError("服务不可用: " + client.srv_name)
    future = client.call_async(request)
    deadline = time.monotonic() + seconds
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.01)
    if not future.done():
        raise RuntimeError("服务响应超时: " + client.srv_name)
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true", help="明确启用真机模型短段；默认只读")
    parser.add_argument("--watchdog-test", action="store_true",
                        help="配合 --execute：运动约0.25秒后停止心跳，验证控制器独立减速并停用")
    args = parser.parse_args()
    if args.watchdog_test and not args.execute:
        parser.error("--watchdog-test 必须与 --execute 同时使用")
    if args.output.exists():
        parser.error("输出目录必须不存在")
    plan = json.loads(args.plan.read_text())
    if plan.get("format") != "flash_fr3_model_prefix_plan_v1" or plan.get("controller") != "flash_fr3_model":
        raise ValueError("试运行计划格式异常")
    expected = np.asarray(plan["expected_start_q"], dtype=float)
    coefficients = np.asarray(plan["coefficients"], dtype=float)
    durations = np.asarray(plan["segment_durations"], dtype=float)
    if (expected.shape != (7,) or coefficients.shape != (2,7,7) or durations.shape != (2,) or
            not all(np.isfinite(x).all() for x in (expected,coefficients,durations)) or
            plan.get("checkpoint_step") != 15000 or plan.get("weights") != "ema_model" or
            plan.get("max_plan_age_seconds") != 600.):
        raise ValueError("模型短段计划参数异常")
    args.output.mkdir(parents=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("flash_fr3_model_client")
    state, rows = {}, []

    def arm(message):
        q, dq = joint_state_vectors(message.name, message.position, message.velocity)
        state.update(q=q, dq=dq, receipt=time.monotonic(),
                     source_age=time.time()-stamp_seconds(message))

    def status(message):
        d = np.asarray(message.data, dtype=float)
        if d.shape != (32,) or not np.isfinite(d).all():
            state["status_error"] = "控制器状态消息异常"
            return
        rows.append({"receipt_monotonic": time.monotonic(), "data": d.tolist()})

    subscriptions = [node.create_subscription(JointState, "/franka/joint_states", arm,
                         QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
                     node.create_subscription(Float64MultiArray, "/flash_fr3_model/status", status, 10)]
    listing = node.create_client(ListControllers, "/controller_manager/list_controllers")
    switching = node.create_client(SwitchController, "/controller_manager/switch_controller")
    params = node.create_client(GetParameters, "/flash_fr3_model/get_parameters")
    report = {"status": "PREFLIGHT_PENDING", "execution_requested": args.execute,
              "activation_requested": False, "robot_motion_commanded": False,
              "gripper_motion_commanded": False, "plan_id": plan["plan_id"],
              "watchdog_test_requested": args.watchdog_test,
              "client_deactivation_requested": False}
    failure = None

    def controllers():
        return call(node, listing, ListControllers.Request()).controller

    def switch(activate):
        request = SwitchController.Request()
        request.activate_controllers = ["flash_fr3_model"] if activate else []
        request.deactivate_controllers = [] if activate else ["flash_fr3_model"]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout = Duration(sec=3)
        if not call(node, switching, request, seconds=5).ok:
            raise RuntimeError("控制器切换未成功；不自动重试")

    def fresh_state():
        if (not state or "q" not in state or time.monotonic() - state["receipt"] > .1 or
                state["source_age"] < -.05 or state["source_age"] + time.monotonic() - state["receipt"] > .1):
            raise RuntimeError("缺少新鲜机械臂状态")
        if np.abs(state["dq"]).max() > .01:
            raise RuntimeError("机械臂尚未近似静止")

    try:
        age = time.time()-float(plan["created_wall_seconds"])
        if not np.isfinite(age) or not 0 <= age <= plan["max_plan_age_seconds"]:
            raise RuntimeError("快照计划已过期或时间异常，请重新 --prepare；不重用旧预测")
        report["snapshot_plan_age_seconds"] = age
        report["scope"] = plan["scope"]
        available = controllers()
        pilot = next((c for c in available if c.name == "flash_fr3_model"), None)
        if pilot is None or pilot.state != "inactive" or pilot.claimed_interfaces or pilot.type != plan["plugin"]:
            raise RuntimeError("试运行控制器未正确配置为 inactive，或已有命令接口被占用")
        if any(c.state == "active" and c.name != "joint_state_broadcaster" for c in available):
            raise RuntimeError("另一个控制器正在运行")
        request = GetParameters.Request(names=["expected_start_q", "segment_durations", "segment_coefficients", "plan_id"])
        values = call(node, params, request).values
        if (len(values) != 4 or not np.array_equal(values[0].double_array_value, expected) or
                not np.array_equal(values[1].double_array_value, durations) or
                not np.array_equal(values[2].double_array_value, coefficients.ravel()) or
                values[3].string_value != plan["plan_id"]):
            raise RuntimeError("已加载控制器参数与当前计划不一致")
        deadline = time.monotonic() + 3
        while "q" not in state and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        fresh_state()
        if np.abs(state["q"] - expected).max() > .002:
            raise RuntimeError("机器人已离开记录的起点，需重新生成计划")
        report.update(status="READONLY_PREFLIGHT_PASS", current_q=state["q"].tolist(),
                      max_start_gap_rad=float(np.abs(state["q"] - expected).max()))
        if args.execute:
            if args.watchdog_test:
                print("即将执行模型短段心跳检查：运动约0.25秒后停止心跳；减速停车，不返回起点，夹爪不动。", flush=True)
            else:
                print("即将执行真实 FLASH 快照的限速短段：总时长 %.3f 秒；停车后不返回，夹爪不动。" % plan["total_motion_seconds"], flush=True)
            publisher = node.create_publisher(
                Header, "/flash_fr3_model/heartbeat",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            clock = Clock(clock_type=ClockType.SYSTEM_TIME)
            report["activation_requested"] = True
            report["robot_motion_commanded"] = True
            rows.clear()
            switch(True)
            started = time.monotonic()
            previous_phase = None
            moving_seen = cut_at = None
            braking_seen = latched_seen = False
            next_native_check = 0.
            while time.monotonic() - started < 10:
                rclpy.spin_once(node, timeout_sec=.002)
                fresh_state()
                if "status_error" in state:
                    raise RuntimeError(state["status_error"])
                if args.watchdog_test and cut_at is not None:
                    now = time.monotonic()
                    if now >= next_native_check and now-cut_at >= .50:
                        next_native_check = now + .05
                        current = next((c for c in controllers() if c.name == "flash_fr3_model"), None)
                        if current is not None and current.state == "inactive":
                            # The service wait also spins status subscriptions.
                            braking_seen = braking_seen or any(int(row["data"][1]) == 2 and int(row["data"][2]) == 1 for row in rows)
                            latched_seen = latched_seen or any(int(row["data"][1]) == 3 and int(row["data"][2]) == 1 for row in rows)
                            # No client stop request has been sent at this point.
                            if not (braking_seen and latched_seen):
                                raise RuntimeError("控制器已停用，但未记录到预期的心跳超时减速与锁存状态")
                            report.update(status="WATCHDOG_TRIAL_FINISHED",
                                native_deactivation_observed_without_client_request=True,
                                heartbeat_timeout_braking_observed=braking_seen,
                                heartbeat_timeout_latched_observed=latched_seen,
                                heartbeat_cut_to_inactive_observation_seconds=time.monotonic()-cut_at)
                            break
                    if time.monotonic()-cut_at > 1.5:
                        raise RuntimeError("停止心跳后1.5秒内未确认控制器独立停用；进入客户端兜底停用")
                if rows:
                    latest = rows[-1]
                    if time.monotonic() - latest["receipt_monotonic"] > .20:
                        raise RuntimeError("控制器状态停止更新")
                    phase = int(latest["data"][1])
                    reason = int(latest["data"][2])
                    if phase != previous_phase:
                        print("FLASH_MODEL_PREFIX_PHASE=" + json.dumps({"phase": phase, "reason": int(latest["data"][2])}), flush=True)
                        previous_phase = phase
                    if phase == 1 and moving_seen is None:
                        moving_seen = time.monotonic()
                    if args.watchdog_test and moving_seen is not None and cut_at is None and time.monotonic()-moving_seen >= .25:
                        cut_at = time.monotonic()
                        report.update(heartbeat_cut_requested=True, heartbeat_cut_monotonic=cut_at)
                        print("FLASH_MODEL_PREFIX_HEARTBEAT_CUT=1", flush=True)
                    if phase in (2, 3, 5):
                        if args.watchdog_test and cut_at is not None and phase in (2, 3) and reason == 1:
                            braking_seen = braking_seen or phase == 2
                            latched_seen = latched_seen or phase == 3
                            report.update(heartbeat_timeout_braking_observed=braking_seen,
                                          heartbeat_timeout_latched_observed=latched_seen)
                        else:
                            raise RuntimeError("控制器触发停止或错误，见状态记录")
                    if phase == 4:
                        if args.watchdog_test:
                            raise RuntimeError("预期心跳中断停车，却收到短段完成状态")
                        report["status"] = "MOTION_TRIAL_FINISHED"
                        break
                elif time.monotonic() - started > .5:
                    raise RuntimeError("激活后未收到控制器状态")
                if cut_at is None:
                    message = Header(stamp=clock.now().to_msg(), frame_id=plan["plan_id"])
                    publisher.publish(message)
                    report["robot_motion_commanded"] = True
                # This loop is the sole heartbeat producer. It stops sending
                # on stale inputs, any exception, Ctrl-C, or process termination.
                until = time.monotonic() + .02
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=.002)
            else:
                raise RuntimeError("试运行未在10秒内正常完成")
    except (Exception, KeyboardInterrupt) as error:
        failure = str(error) or "KeyboardInterrupt"
        report.update(status="TRIAL_FAILED" if args.execute else "PREFLIGHT_FAILED", error=failure)
    finally:
        if report["activation_requested"]:
            # Stop heartbeat first; allow the independent 0.2s watchdog + 0.25s
            # reference brake to finish before requesting native mode shutdown.
            deadline = time.monotonic() + .60
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.01)
            try:
                available = controllers()
                if any(c.name == "flash_fr3_model" and c.state == "inactive" for c in available):
                    report["native_deactivation_observed_without_client_request"] = True
                if any(c.name == "flash_fr3_model" and c.state == "active" for c in available):
                    report["client_deactivation_requested"] = True
                    switch(False)
                report["controller_left_inactive"] = any(c.name == "flash_fr3_model" and c.state == "inactive" for c in controllers())
                if not report["controller_left_inactive"]:
                    raise RuntimeError("未确认试运行控制器已退回 inactive")
            except Exception as error:
                report["deactivation_error"] = str(error)
                failure = failure or str(error)
                report["status"] = "STOP_STATE_UNCONFIRMED"
        (args.output / "status_samples.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
        if rows:
            measured = np.asarray([row["data"][4:11] for row in rows])
            reference = np.asarray([row["data"][18:25] for row in rows])
            report["measured_per_joint_excursion_max_rad"] = np.abs(measured - expected).max(axis=0).tolist()
            report["measured_final_gap_rad"] = (measured[-1] - expected).tolist()
            report["measured_excursion_from_first_status_rad"] = np.abs(measured-measured[0]).max(axis=0).tolist()
            report["measured_final_minus_first_status_rad"] = (measured[-1]-measured[0]).tolist()
            report["max_abs_tracking_error_rad"] = np.abs(measured-reference).max(axis=0).tolist()
        report["report"] = str(args.output / "report.json")
        (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        node.destroy_node()
        rclpy.shutdown()
    print("FLASH_MODEL_PREFIX_RESULT=" + json.dumps(report), flush=True)
    if failure:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
