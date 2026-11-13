#!/usr/bin/env python3
"""Capture fresh real observations and render a frozen FLASH prefix; no commands."""
import argparse
import hashlib
import io
import json
import subprocess
import threading
import time
import urllib.request
import uuid
from pathlib import Path

import numpy as np
import rclpy

from capture_flash_fr3_observation import JOINTS
from flash_fr3_model_plan import make_segments, write_preview_csv
from flash_fr3_shadow_math import validate_reply
from run_flash_fr3_guard_shadow import measured_state
from run_flash_fr3_shadow_client import ObservationBuffer

PREVIEW = Path("/home/ros/ros2_ws/install/flash_fr3_model_controller/lib/flash_fr3_model_controller/preview_model")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--server", default="http://192.168.1.113:10131")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("输出目录已存在")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(args.server+"/health", timeout=3) as response:
        health = json.load(response)
    if (health.get("status") != "FLASH_SHADOW_SERVER_READY" or health.get("checkpoint_step") != 15000 or
            health.get("weights") != "ema_model" or health.get("robot_motion_commanded") is not False):
        raise ValueError("需要第15000步 EMA 只读推理服务")
    rclpy.init()
    node = rclpy.create_node("flash_fr3_prepare_model_prefix")
    buffer = ObservationBuffer(node)
    stop, errors = threading.Event(), []

    def spin():
        try:
            while not stop.is_set():
                rclpy.spin_once(node, timeout_sec=.01)
        except Exception as error:
            errors.append(str(error))

    worker = threading.Thread(target=spin)
    worker.start()
    args.output.mkdir(parents=True)
    try:
        deadline = time.monotonic()+15
        while True:
            try:
                arrays, metadata = buffer.observation(0)
                break
            except ValueError:
                if errors or time.monotonic() > deadline:
                    raise RuntimeError("缺少完整新鲜双相机和状态历史: "+str(errors))
                stop.wait(.05)
        if np.ptp(arrays["agent_pos"][:, :7], axis=0).max() > .0005:
            raise ValueError("采集历史期间机械臂未静止")
        blob = io.BytesIO()
        np.savez(blob, **arrays, metadata_json=np.asarray(json.dumps(metadata)))
        (args.output/"observation.npz").write_bytes(blob.getvalue())
        anchor = metadata["samples"][-1]["target_stamp_seconds"]
        wall, mono = time.time(), time.monotonic()
        observation_mono = mono-(wall-anchor)
        request = urllib.request.Request(args.server+"/predict", data=blob.getvalue(),
                                         headers={"Content-Type":"application/octet-stream"})
        with opener.open(request, timeout=3) as response:
            reply = json.load(response)
        validate_reply(reply, 0, anchor, 15000)
        (args.output/"prediction.json").write_text(json.dumps(reply, indent=2, allow_nan=False)+"\n")
        state = measured_state(buffer)
        now = time.monotonic()
        if (now-state["receipt_monotonic"] > .1 or
                not -.05 <= state["source_age_at_receipt_seconds"]+now-state["receipt_monotonic"] <= .1 or
                np.abs(state["dq"]).max() > .01 or errors):
            raise ValueError("当前状态不新鲜或机械臂未静止")
        plan = make_segments(reply["curve"], now-observation_mono, state["q"])
        coefficients = np.asarray(plan["coefficients"])
        vector = plan["segment_durations"]+coefficients.ravel().tolist()+plan["expected_start_q"]
        (args.output/"preview_input.txt").write_text(" ".join(str(v) for v in vector)+"\n")
        result = subprocess.run([str(PREVIEW), str(args.output/"preview_input.txt"),
                                 str(args.output/"reference_cpp.csv")], check=True, capture_output=True, text=True)
        preview = json.loads(result.stdout)
        write_preview_csv(args.output/"reference_python.csv",plan)
        cpp = np.loadtxt(args.output/"reference_cpp.csv",delimiter=",",skiprows=1)
        python = np.loadtxt(args.output/"reference_python.csv",delimiter=",",skiprows=1)
        error = float(np.abs(cpp-python).max()) if cpp.shape == python.shape else float("inf")
        if error > 1e-9:
            raise ValueError("C++ 执行求值与 Python 独立求值不一致")
        plan.update(format="flash_fr3_model_prefix_plan_v1", plan_id=uuid.uuid4().hex,
            controller="flash_fr3_model", plugin="flash_fr3_model_controller/ModelController",
            created_wall_seconds=time.time(), max_plan_age_seconds=600., joint_order=JOINTS,
            checkpoint_step=15000, weights="ema_model", server=args.server,
            preview=preview, cpp_python_max_abs_error=error,
            preview_binary_sha256=hashlib.sha256(PREVIEW.read_bytes()).hexdigest(),
            sampled_per_joint_excursion_rad=np.abs(cpp[:,1:8]-state["q"]).max(axis=0).tolist(),
            heartbeat_timeout_seconds=.2, watchdog_brake_seconds=.25,
            scope="frozen snapshot commissioning; time-scaled FLASH prefix plus C2 stop; no online replanning",
            robot_motion_commanded=False, gripper_motion_commanded=False,
            physical_stopping_verified_for_this_controller=False)
        params = {"expected_start_q":plan["expected_start_q"],"plan_id":plan["plan_id"],
                  "segment_durations":plan["segment_durations"],"segment_coefficients":coefficients.ravel().tolist()}
        (args.output/"controller_params.yaml").write_text(json.dumps(
            {"flash_fr3_model":{"ros__parameters":params}},indent=2,allow_nan=False)+"\n")
        (args.output/"plan.json").write_text(json.dumps(plan,indent=2,allow_nan=False)+"\n")
        print("FLASH_MODEL_PREFIX_PLAN="+json.dumps({"plan":str(args.output/"plan.json"),
            "status":"MODEL_PREFIX_PLAN_READY", "total_motion_seconds":plan["total_motion_seconds"],
            "max_joint_excursion_deg":float(np.rad2deg(np.abs(cpp[:,1:8]-state["q"]).max())),
            "robot_motion_commanded":False}),flush=True)
    finally:
        stop.set(); worker.join(timeout=3)
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
