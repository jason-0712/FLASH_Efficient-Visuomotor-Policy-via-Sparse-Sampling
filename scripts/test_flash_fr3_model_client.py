#!/usr/bin/env python3
"""Exercise the actual pilot client with ROS service fixtures; no hardware or dynamics.

Requires ROS_DOMAIN_ID=232 and ROS_LOCALHOST_ONLY=1. The fixture offers the same
absolute service names as the real manager, so isolation is mandatory.
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Header


class ManagerFixture(Node):
    def __init__(self, plan):
        super().__init__("flash_fr3_model_client_fixture", start_parameter_services=False)
        self.plan = plan
        self.reset("check")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joints = self.create_publisher(JointState, "/franka/joint_states", qos)
        self.status = self.create_publisher(Float64MultiArray, "/flash_fr3_model/status", 10)
        self.heartbeat = self.create_subscription(Header, "/flash_fr3_model/heartbeat", self.receive, qos)
        self.fixture_services = [
            self.create_service(ListControllers, "/controller_manager/list_controllers", self.listing),
            self.create_service(SwitchController, "/controller_manager/switch_controller", self.switch),
            self.create_service(GetParameters, "/flash_fr3_model/get_parameters", self.parameters),
        ]
        self.timer = self.create_timer(.01, self.publish)

    def reset(self, scenario):
        self.scenario = scenario
        self.active = False
        self.switches = []
        self.heartbeats = []
        self.errors = []
        self.native_deactivations = 0

    def listing(self, request, response):
        response.controller = [
            ControllerState(name="joint_state_broadcaster", state="active",
                            type="joint_state_broadcaster/JointStateBroadcaster"),
            ControllerState(name="flash_fr3_model", state="active" if self.active else "inactive",
                            type=self.plan["plugin"],
                            claimed_interfaces=[f"fr3_joint{i}/position" for i in range(1, 8)] if self.active else []),
        ]
        if self.scenario == "missing_after_stop" and self.switches and not self.switches[-1]["activate"]:
            response.controller = response.controller[:1]
        return response

    def parameters(self, request, response):
        if request.names != ["expected_start_q", "segment_durations", "segment_coefficients", "plan_id"]:
            self.errors.append("Unexpected parameter names")
        response.values = [
            ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                           double_array_value=self.plan["expected_start_q"]),
            ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                           double_array_value=self.plan["segment_durations"]),
            ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                           double_array_value=[v for segment in self.plan["coefficients"] for row in segment for v in row]),
            ParameterValue(type=ParameterType.PARAMETER_STRING,
                           string_value=self.plan["plan_id"] + ("_mismatch" if self.scenario == "mismatch" else "")),
        ]
        return response

    def switch(self, request, response):
        activating = request.activate_controllers == ["flash_fr3_model"] and not request.deactivate_controllers
        deactivating = request.deactivate_controllers == ["flash_fr3_model"] and not request.activate_controllers
        if (not (activating or deactivating) or request.strictness != SwitchController.Request.STRICT or
                not request.activate_asap or request.timeout.sec != 3):
            self.errors.append("Unexpected switch request")
            response.ok = False
            return response
        self.switches.append({"activate": activating, "time": time.monotonic()})
        self.active = activating
        response.ok = True
        return response

    def receive(self, message):
        stamp = message.stamp.sec + message.stamp.nanosec / 1e9
        age = time.time() - stamp
        if not self.active or message.frame_id != self.plan["plan_id"] or not -.05 <= age <= .1:
            self.errors.append("Inactive, wrong-plan or stale heartbeat")
        if self.heartbeats and stamp <= self.heartbeats[-1]["stamp"]:
            self.errors.append("Non-increasing heartbeat timestamp")
        self.heartbeats.append({"stamp": stamp, "receipt": time.monotonic()})

    def publish(self):
        # Deliberately shuffled names check the real client's ordering path.
        order = [1, 3, 6, 7, 2, 4, 5]
        q = self.plan["expected_start_q"]
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f"fr3_joint{i}" for i in order]
        message.position = [q[i-1] for i in order]
        message.velocity = [0.] * 7
        self.joints.publish(message)
        if self.active:
            phase = 0 if not self.heartbeats else 1
            reason = 0
            age = time.monotonic()-self.heartbeats[-1]["receipt"] if self.heartbeats else 0.
            if self.scenario.startswith("watchdog_"):
                if age > .2:
                    phase, reason = 2, 1
                if age > .45:
                    phase, reason = 3, 1
            elif len(self.heartbeats) >= 4:
                phase = 5 if self.scenario == "controller_error" else 4
                reason = 1 if phase == 5 else 0
            data = [time.monotonic(), float(phase), float(reason), age] + q + [0.]*7 + q + [0.]*7
            self.status.publish(Float64MultiArray(data=data))
            if self.scenario == "watchdog_native_stop" and age > .50:
                self.active = False
                self.native_deactivations += 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") != "232" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("Only isolated domain 232 with ROS_LOCALHOST_ONLY=1 is permitted")
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    plan = json.loads(args.plan.read_text())
    plan["created_wall_seconds"] = time.time()
    fixture_plan = args.output/"fixture_plan.json"
    fixture_plan.write_text(json.dumps(plan))
    fixture = ManagerFixture(plan)
    results = []
    try:
        for scenario in ("check", "mismatch", "complete", "controller_error", "watchdog_native_stop", "watchdog_no_native_stop", "missing_after_stop"):
            fixture.reset(scenario)
            output = args.output / scenario
            command = [sys.executable, str(Path(__file__).with_name("run_flash_fr3_model_prefix.py")),
                       "--plan", str(fixture_plan), "--output", str(output)]
            # Even mismatch requests --execute: failed preflight must gate activation.
            if scenario != "check":
                command.append("--execute")
            if scenario.startswith("watchdog_"):
                command.append("--watchdog-test")
            with (args.output / (scenario + ".log")).open("w") as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 20
                    while process.poll() is None and time.monotonic() < deadline:
                        rclpy.spin_once(fixture, timeout_sec=.005)
                    if process.poll() is None:
                        raise AssertionError("Client timed out: " + scenario)
                finally:
                    if process.poll() is None:
                        process.kill()
                    process.wait()
            report = json.loads((output / "report.json").read_text())
            assert not fixture.errors, fixture.errors
            assert not report["gripper_motion_commanded"]
            if scenario in ("check", "mismatch"):
                assert not fixture.switches and not fixture.heartbeats
                assert not report["activation_requested"] and not report["robot_motion_commanded"]
                assert process.returncode == (0 if scenario == "check" else 1)
                assert report["status"] == ("READONLY_PREFLIGHT_PASS" if scenario == "check" else "TRIAL_FAILED")
            else:
                expected_status = {"complete": "MOTION_TRIAL_FINISHED", "controller_error": "TRIAL_FAILED",
                                   "watchdog_native_stop": "WATCHDOG_TRIAL_FINISHED",
                                   "watchdog_no_native_stop": "TRIAL_FAILED", "missing_after_stop": "STOP_STATE_UNCONFIRMED"}[scenario]
                assert process.returncode == (0 if scenario in ("complete", "watchdog_native_stop") else 1)
                assert report["status"] == expected_status, report
                assert not fixture.active
                assert len(fixture.heartbeats) >= 4
                if scenario == "watchdog_native_stop":
                    assert [event["activate"] for event in fixture.switches] == [True]
                    assert fixture.native_deactivations == 1 and not report["client_deactivation_requested"]
                    assert report["native_deactivation_observed_without_client_request"]
                    assert report["heartbeat_timeout_braking_observed"] and report["heartbeat_timeout_latched_observed"]
                else:
                    assert [event["activate"] for event in fixture.switches] == [True, False]
                    assert report["client_deactivation_requested"]
                    assert fixture.switches[-1]["time"] - fixture.heartbeats[-1]["receipt"] >= .45
                assert report["controller_left_inactive"] == (scenario != "missing_after_stop")
                if scenario.startswith("watchdog_"):
                    assert report["heartbeat_cut_requested"] and len(fixture.heartbeats) >= 8
                if scenario == "watchdog_no_native_stop":
                    assert "1.5" in report["error"]
            result = {"case": scenario, "status": "PASS", "received_heartbeats": len(fixture.heartbeats),
                      "switch_requests": [event["activate"] for event in fixture.switches],
                      "fixture_native_deactivations": fixture.native_deactivations}
            results.append(result)
            print(json.dumps(result), flush=True)
    finally:
        fixture.destroy_node()
        rclpy.shutdown()
    summary = {"status": "ISOLATED_CLIENT_TEST_PASS", "cases": results, "ros_domain_id": 232,
               "hardware_driver_loaded": False, "physical_robot_motion_commanded": False,
               "scope": "Actual Python client subprocess + ROS message/service fixtures; no hardware or robot dynamics"}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print("FLASH_MODEL_PREFIX_CLIENT_TEST=" + json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
