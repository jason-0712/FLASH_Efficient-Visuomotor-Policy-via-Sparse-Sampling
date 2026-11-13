#!/usr/bin/env python3
"""Real observations + H100 + guarded reference, with ROS subscriptions ONLY.

HTTP runs in its own worker; the owner checks reference and state at 100 Hz.
This is not the 1kHz robot controller and cannot implement a hardware watchdog.
"""
import argparse
import csv
import io
import json
import queue
import threading
import time
import urllib.request
from pathlib import Path

import numpy as np
import rclpy

from capture_flash_fr3_observation import DT, stamp_seconds
from flash_fr3_guard import ShadowGuard, joint_state_vectors
from flash_fr3_shadow_math import evaluate_curve, validate_reply
from run_flash_fr3_shadow_client import ObservationBuffer


def measured_state(buffer):
    with buffer.lock:
        if not buffer.history["arm"]:
            raise ValueError("缺少机械臂状态")
        message, receipt, wall = buffer.history["arm"][-1]
    q, dq = joint_state_vectors(message.name, message.position, message.velocity)
    return {"q": q, "dq": dq, "receipt_monotonic": receipt,
            "source_age_at_receipt_seconds": wall - stamp_seconds(message)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="http://192.168.1.113:10131")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seconds", type=float, default=8)
    args = parser.parse_args()
    if args.output.exists() or not np.isfinite(args.seconds) or not 0 < args.seconds <= 30:
        parser.error("请使用新输出目录，诊断时长须在 (0,30] 秒")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(args.server + "/health", timeout=2) as response:
        health = json.load(response)
    if (health.get("status") != "FLASH_SHADOW_SERVER_READY" or health.get("robot_motion_commanded") is not False
            or health.get("weights") != "ema_model" or health.get("checkpoint_step") != 15000):
        raise RuntimeError("服务未就绪或不是预期的第15000步EMA模型")
    args.output.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node("flash_fr3_guard_shadow")
    buffer = ObservationBuffer(node)
    stop = threading.Event()
    responses = queue.Queue(maxsize=1)
    spin_errors = []

    def spin():
        try:
            while not stop.is_set():
                rclpy.spin_once(node, timeout_sec=.01)
        except Exception as error:
            spin_errors.append(str(error))

    def infer():
        request_id, next_query = 0, time.monotonic()
        while not stop.is_set():
            if stop.wait(max(0, next_query - time.monotonic())):
                break
            item = {"request_id": request_id}
            try:
                arrays, metadata = buffer.observation(request_id)
                anchor = metadata["samples"][-1]["target_stamp_seconds"]
                # Convert the local source-clock anchor once. All deadlines
                # thereafter use this host's monotonic clock, not H100's clock.
                wall, mono = time.time(), time.monotonic()
                item["observation_monotonic"] = mono - (wall - anchor)
                blob = io.BytesIO()
                np.savez(blob, **arrays, metadata_json=np.asarray(json.dumps(metadata)))
                request = urllib.request.Request(args.server + "/predict", data=blob.getvalue(),
                                                 headers={"Content-Type": "application/octet-stream"})
                with opener.open(request, timeout=2) as response:
                    reply = json.load(response)
                validate_reply(reply, request_id, anchor, health["checkpoint_step"])
                item["response"] = reply
                item["observation_to_response_ms"] = (time.monotonic() - item["observation_monotonic"]) * 1000
                if request_id == 0:
                    (args.output / "first_observation.npz").write_bytes(blob.getvalue())
            except Exception as error:
                item["error"] = str(error)
            # Bounded mailbox and no silent dropping of responses. A stuck
            # owner must not create an unbounded queue of obsolete predictions.
            while not stop.is_set():
                try:
                    responses.put(item, timeout=.02)
                    break
                except queue.Full:
                    continue
            if "error" in item:
                break
            request_id += 1
            next_query = max(next_query + DT, time.monotonic())

    spin_worker = threading.Thread(target=spin, name="ros_observations")
    io_worker = threading.Thread(target=infer, name="h100_inference")
    spin_worker.start()
    guard, iterations, samples, events = ShadowGuard(), [], 0, []
    failure = None
    try:
        warmup_end = time.monotonic() + 15
        while True:
            try:
                buffer.observation(-1)
                measured_state(buffer)
                break
            except ValueError:
                if spin_errors or time.monotonic() > warmup_end:
                    raise RuntimeError("未获得完整的新鲜观测历史: " + str(spin_errors))
                time.sleep(.05)
        io_worker.start()
        start = next_tick = time.monotonic()
        previous_tick = None
        with (args.output / "reference_samples.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["elapsed_s"] + [f"{key}{i}" for key in ("q", "dq", "ddq", "measured_q") for i in range(1, 8)])
            while time.monotonic() - start < args.seconds and guard.mode != "BLOCKED":
                if stop.wait(max(0, next_tick - time.monotonic())):
                    break
                state = measured_state(buffer)
                now = time.monotonic()
                if previous_tick is not None:
                    iterations.append(now - previous_tick)
                previous_tick = now
                reference = guard.tick(now, state)
                if spin_errors:
                    guard.block("ROS_SUBSCRIPTION_ERROR", spin_errors)
                try:
                    item = responses.get_nowait()
                except queue.Empty:
                    item = None
                if item is not None:
                    item["state_at_admission"] = {key: value.tolist() if isinstance(value, np.ndarray) else value
                                                  for key, value in state.items()}
                    item["admission_monotonic"] = now
                    if "error" in item:
                        guard.block("OBSERVATION_OR_INFERENCE_ERROR", item["error"])
                    else:
                        accepted = guard.offer(item["response"], item["observation_monotonic"], now, state)
                        item["accepted"] = accepted
                        item["preview"] = guard.last_preview
                        if accepted:
                            reference = guard.reference.evaluate(0)
                        raw_gripper = float(evaluate_curve(item["response"]["curve"], 0)[0][7])
                        item["gripper_diagnostic_only"] = {"raw_closure_at_anchor": raw_gripper,
                            "bounded_closure_at_anchor": float(np.clip(raw_gripper, 0, 1)),
                            "width_m_at_anchor": float(.08 * (1 - np.clip(raw_gripper, 0, 1))),
                            "gripper_goal_sent": False}
                    item["elapsed_seconds"] = now - start
                    events.append(item)
                    with (args.output / "events.jsonl").open("a") as log:
                        log.write(json.dumps(item, allow_nan=False) + "\n")
                    print("FLASH_GUARD_EVENT=" + json.dumps({key: value for key, value in item.items()
                          if key not in ("response", "preview")}), flush=True)
                if reference is not None and guard.mode == "REFERENCE_ACTIVE":
                    writer.writerow([now - start] + np.r_[reference[0], reference[1], reference[2], state["q"]].tolist())
                    samples += 1
                next_tick = max(next_tick + .01, time.monotonic())
    except Exception as error:
        failure = str(error)
        guard.block("CLIENT_ERROR", failure)
    finally:
        stop.set()
        if io_worker.ident is not None:
            io_worker.join(timeout=3)
        spin_worker.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
    report = guard.report()
    report.update(status="SHADOW_GUARD_BLOCKED" if guard.mode == "BLOCKED" else "SHADOW_DIAGNOSTIC_COMPLETE",
                  reference_samples=samples, responses=len(events), server=health,
                  execution_eligible=False, reference_monitor_hz=100,
                  initial_acceleration="zero assumed only after measured |dq|<=0.01 rad/s; not measured acceleration",
                  scope="read-only real inputs; hypothetical references do not feed back into robot state",
                  monitor_gap_seconds=dict(zip(("median", "p95", "max"),
                    np.quantile(iterations, [.5, .95, 1]).tolist())) if iterations else {},
                  report=str((args.output / "report.json").resolve()))
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print("FLASH_GUARD_SHADOW=" + json.dumps(report), flush=True)
    if failure:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
