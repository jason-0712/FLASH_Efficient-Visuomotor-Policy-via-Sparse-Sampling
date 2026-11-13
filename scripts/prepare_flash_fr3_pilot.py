#!/usr/bin/env python3
"""Read joint states and render the compiled pilot reference; never commands IO."""
import argparse
import hashlib
import json
import subprocess
import time
import uuid
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from flash_fr3_guard import joint_state_vectors
from capture_flash_fr3_observation import JOINTS, stamp_seconds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("输出目录已存在")
    rclpy.init()
    node = rclpy.create_node("flash_fr3_prepare_pilot")
    states = []

    def receive(message):
        q, dq = joint_state_vectors(message.name, message.position, message.velocity)
        wall, mono = time.time(), time.monotonic()
        source = stamp_seconds(message)
        if not -.05 <= wall - source <= .1 or (states and source <= states[-1]["source_stamp_seconds"]):
            return
        states.append({"q": q.tolist(), "dq": dq.tolist(), "source_stamp_seconds": source,
                       "receipt_monotonic": mono, "source_age_at_receipt_seconds": wall-source})

    subscription = node.create_subscription(JointState, "/franka/joint_states", receive,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        deadline = time.monotonic() + 10
        while len(states) < 50 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        if len(states) < 50 or time.monotonic() - states[-1]["receipt_monotonic"] > .1:
            raise RuntimeError("未获取50条新鲜实测关节状态")
        q, dq = np.asarray([s["q"] for s in states]), np.asarray([s["dq"] for s in states])
        if np.abs(dq).max() > .01 or np.ptp(q, axis=0).max() > .0005:
            raise RuntimeError("机械臂未保持近似静止")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    args.output.mkdir(parents=True)
    plan_id = uuid.uuid4().hex
    expected = states[-1]["q"]
    binary = Path("/home/ros/ros2_ws/install/flash_fr3_pilot_controller/lib/flash_fr3_pilot_controller/preview_pilot")
    completed = subprocess.run([str(binary), str(args.output / "reference.csv"),
                               *[str(v) for v in expected], ".005"], check=True, capture_output=True, text=True)
    preview = json.loads(completed.stdout)
    parameters = {"flash_fr3_pilot": {"ros__parameters": {
        "expected_start_q": expected, "joint2_delta_rad": .005, "plan_id": plan_id}}}
    # JSON is accepted by the ROS YAML parser; no hand-quoted parameter strings.
    (args.output / "controller_params.yaml").write_text(json.dumps(parameters, indent=2) + "\n")
    plan = {"format": "flash_fr3_pilot_plan_v1", "plan_id": plan_id,
            "created_wall_seconds": time.time(), "controller": "flash_fr3_pilot",
            "plugin": "flash_fr3_pilot_controller/PilotController", "command_interface": "position",
            "joint_order": JOINTS, "expected_start_q": expected,
            "joint2_delta_rad": .005, "joint2_delta_deg": float(np.rad2deg(.005)),
            "each_leg_seconds": 3, "total_motion_seconds": 6,
            "heartbeat_timeout_seconds": .20, "reference_braking_seconds": .25,
            "preview": preview, "observations": states,
            "preview_binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
            "scope": "execution-chain commissioning; fixed small reference, no FLASH task execution",
            "robot_motion_commanded": False, "gripper_motion_commanded": False,
            "physical_stopping_verified": False}
    path = args.output / "plan.json"
    path.write_text(json.dumps(plan, indent=2, allow_nan=False) + "\n")
    print("FLASH_PILOT_PLAN=" + json.dumps({"plan": str(path), "plan_id": plan_id, "preview": preview,
                                         "robot_motion_commanded": False}), flush=True)


if __name__ == "__main__":
    main()
