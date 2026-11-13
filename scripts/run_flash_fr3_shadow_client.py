#!/usr/bin/env python3
"""Stream real ROS observations to FLASH; log predictions without commanding IO."""
from __future__ import annotations

import argparse
import bisect
import io
import json
import threading
import time
import urllib.error
import urllib.request
from collections import Counter, deque
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState

from capture_flash_fr3_observation import (DT, FINGERS, JOINTS, TOPICS, measured_gripper_closure,
                                          ordered_positions, resize_rgb, stamp_seconds)
from flash_fr3_shadow_math import evaluate_curve, validate_reply
from flash_fr3_camera_clock import SDK_TIME_BASIS, frame_clock, observation_stamp


class ObservationFreshnessError(ValueError):
    def __init__(self, diagnostic):
        super().__init__('观测到达时已过期或时间戳异常: '+json.dumps(diagnostic,ensure_ascii=False))
        self.diagnostic = diagnostic


class ObservationBuffer:
    def __init__(self, node, keys=None, *, queue_depths=None, track_source_order=False,
                 camera_time_basis='ros_header'):
        if camera_time_basis not in ('ros_header', SDK_TIME_BASIS):
            raise ValueError('未知相机源时钟')
        self.camera_time_basis = camera_time_basis
        self.camera_records = {}
        self.node, self.lock = node, threading.Lock()
        sources = TOPICS if keys is None else {key: TOPICS[key] for key in keys}
        self.history = {key: deque(maxlen=4000 if key == "arm" else 240) for key in sources}
        self.subscriptions = []
        self.bridge, self.rgb_cache = CvBridge(), {}
        self.grid_origin = None
        self.source_order_errors = set()
        for key, topic in sources.items():
            if key.endswith('_cam') and camera_time_basis == SDK_TIME_BASIS:
                from realsense2_camera_msgs.msg import Metadata
                self.camera_records[key] = {}
                def receive_metadata(message, key=key):
                    stamp = message.header.stamp
                    with self.lock:
                        record = self.camera_record(key, stamp)
                        try:
                            parsed = frame_clock(stamp.sec, stamp.nanosec, message.json_data)
                            if 'source_stamp_seconds' in record and record != parsed:
                                raise ValueError('同一图像时间戳对应不同元数据')
                            record.update(parsed)
                        except (ValueError, KeyError, TypeError) as error:
                            record['error'] = str(error)
                self.subscriptions.append(node.create_subscription(Metadata,
                    topic.rsplit('/image_raw/compressed',1)[0]+'/metadata', receive_metadata,
                    QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)))
            def receive(message, key=key):
                item = (message, time.monotonic(), time.time())
                with self.lock:
                    if key in self.camera_records:
                        item = (*item, self.camera_record(key, message.header.stamp))
                    if (track_source_order and self.history[key] and
                            stamp_seconds(message)<stamp_seconds(self.history[key][-1][0])):
                        self.source_order_errors.add(key)
                    self.history[key].append(item)
            self.subscriptions.append(node.create_subscription(
                CompressedImage if key.endswith("_cam") else JointState,
                topic, receive, QoSProfile(depth=(queue_depths or {}).get(key,10 if key != "arm" else 1),
                                         reliability=ReliabilityPolicy.BEST_EFFORT)))

    def camera_record(self, key, stamp):
        records = self.camera_records[key]
        record = records.setdefault((stamp.sec, stamp.nanosec), {})
        while len(records)>480:
            records.pop(next(iter(records)))
        return record

    def observation(self, request_id):
        now, wall = time.monotonic(), time.time()
        with self.lock:
            # Images wait for metadata with the identical original header.
            # Never fall back to callback time or to a different frame's time.
            history = {key: [(*item[:3],dict(item[3])) if len(item)==4 else item
                for item in value if len(item)!=4 or
                ('source_stamp_seconds' in item[3] and 'error' not in item[3])]
                for key, value in self.history.items()}
        missing = [key for key, value in history.items() if not value]
        if missing:
            raise ValueError("缺少观测: " + ",".join(missing))
        if any(now - value[-1][1] > 0.3 for value in history.values()):
            raise ValueError("至少一路观测已停止更新超过 0.3s")
        # Decode each stamp once instead of scanning the 4,000-entry arm ring
        # eight times. Keep the same nearest-sample rule (earlier wins ties).
        stamps_by_key = {key: [observation_stamp(item) for item in value]
                         for key, value in history.items()}
        if any(any(b < a for a, b in zip(stamps, stamps[1:]))
               for stamps in stamps_by_key.values()):
            raise ValueError("传感器历史时间戳倒退")

        def nearest(key, target):
            stamps = stamps_by_key[key]
            index = bisect.bisect_left(stamps, target)
            if index == len(stamps) or (index > 0 and
                    target - stamps[index-1] <= stamps[index] - target):
                index -= 1
            return history[key][index]
        latest_target = wall - 0.08
        if self.grid_origin is None:
            self.grid_origin = latest_target
        # Keep one fixed 4/15s grid across requests. Overlapping histories then
        # reuse source frames instead of re-decoding eight shifted RGB pairs.
        anchor = self.grid_origin + np.floor((latest_target - self.grid_origin) / DT) * DT
        arrays = {key: [] for key in ("agent_pos", "primary_cam", "wrist_cam")}
        samples, previous = [], {}
        used_cache_keys = set()
        for i in range(8):
            target = anchor - (7 - i) * DT
            selected = {key: nearest(key, target) for key in history}
            messages = {key: value[0] for key, value in selected.items()}
            stamps = {key: observation_stamp(value) for key, value in selected.items()}
            offsets = {key: value - target for key, value in stamps.items()}
            ages = {key: selected[key][2] - value for key, value in stamps.items()}
            if max(abs(v) for v in offsets.values()) > 0.08:
                raise ValueError("传感器历史未覆盖当前采样网格: "+json.dumps(offsets))
            if any(v < -0.05 or v > 0.3 for v in ages.values()):
                raise ObservationFreshnessError(dict(kind='source_age_at_receipt',sample_index=i,
                    target_stamp_seconds=target,source_age_at_receipt_seconds=ages,
                    source_offset_from_target_seconds=offsets,limits_seconds=[-.05,.3]))
            if max(stamps.values()) - min(stamps.values()) > 0.15:
                raise ValueError("传感器之间时间偏差超过 0.15s")
            if any(value <= previous.get(key, -1) for key, value in stamps.items()):
                raise ValueError("历史观测重复或时间倒退")
            q = ordered_positions(messages["arm"], JOINTS)
            width = float(ordered_positions(messages["hand"], FINGERS).sum())
            arrays["agent_pos"].append(np.r_[q, measured_gripper_closure(width)].astype(np.float32))
            for key in ("primary_cam", "wrist_cam"):
                cache_key = (key, messages[key].header.stamp.sec, messages[key].header.stamp.nanosec)
                used_cache_keys.add(cache_key)
                if cache_key not in self.rgb_cache:
                    rgb = self.bridge.compressed_imgmsg_to_cv2(messages[key], desired_encoding="rgb8")
                    self.rgb_cache[cache_key] = resize_rgb(np.asarray(rgb))
                arrays[key].append(self.rgb_cache[cache_key])
            samples.append({"relative_seconds": i * DT, "target_stamp_seconds": target,
                            "stamp_seconds": stamps, "source_age_at_receipt_seconds": ages,
                            "source_offset_from_target_seconds": offsets})
            if self.camera_time_basis == SDK_TIME_BASIS:
                samples[-1]['camera_clock'] = {key:selected[key][3] for key in ('primary_cam','wrist_cam')}
            previous = stamps
        self.rgb_cache = {key: value for key, value in self.rgb_cache.items() if key in used_cache_keys}
        metadata = {"format": "flash_fr3_live_observation_v1", "capture_mode": "rolling_shadow",
                    "request_id": request_id, "joint_order": JOINTS, "topics": TOPICS,
                    "n_obs_steps": 8, "sample_dt_seconds": DT, "color": "RGB", "samples": samples,
                    "robot_motion_commanded": False}
        if self.camera_time_basis == SDK_TIME_BASIS:
            metadata['camera_time_basis'] = self.camera_time_basis
            metadata['camera_clock_metadata_topics'] = {key:TOPICS[key].rsplit('/image_raw/compressed',1)[0]+'/metadata'
                                                       for key in ('primary_cam','wrist_cam')}
        return {key: np.stack(value) for key, value in arrays.items()}, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="http://192.168.1.113:10131")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=40)
    parser.add_argument("--wait-server", type=float, default=600)
    args = parser.parse_args()
    if args.output.exists() or args.steps < 1 or args.wait_server <= 0:
        parser.error("输出目录必须不存在，步数和等待时间必须为正")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    deadline = time.monotonic() + args.wait_server
    print("FLASH_SHADOW_CLIENT_WAITING_FOR_SERVER=" + args.server, flush=True)
    while True:
        try:
            with opener.open(args.server + "/health", timeout=2) as response:
                health = json.load(response)
            if health.get("status") != "FLASH_SHADOW_SERVER_READY" or health.get("robot_motion_commanded") is not False:
                raise RuntimeError("服务器不符合影子推理约定")
            break
        except (urllib.error.URLError, TimeoutError):
            if time.monotonic() >= deadline:
                raise RuntimeError("等待 H100 推理服务器超时")
            time.sleep(2)
    args.output.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node("flash_fr3_shadow_client")
    buffer = ObservationBuffer(node)
    spin_error = []
    stopping = threading.Event()

    def spin():
        try:
            while not stopping.is_set():
                rclpy.spin_once(node, timeout_sec=0.01)
        except Exception as error:
            spin_error.append(error)

    worker = threading.Thread(target=spin)
    worker.start()
    rows, previous = [], None
    try:
        warmup_deadline = time.monotonic() + 15
        while True:
            try:
                buffer.observation(-1)
                break
            except ValueError as error:
                if spin_error:
                    raise spin_error[0]
                if time.monotonic() >= warmup_deadline:
                    raise RuntimeError("未获得完整实时观测历史: " + str(error))
                time.sleep(0.05)
        next_query = time.monotonic()
        for request_id in range(args.steps):
            time.sleep(max(0, next_query - time.monotonic()))
            if spin_error:
                raise spin_error[0]
            started = time.perf_counter()
            row = {"request_id": request_id}
            try:
                arrays, metadata = buffer.observation(request_id)
                blob = io.BytesIO()
                # Avoid CPU compression latency; maximum request remains <4 MiB.
                np.savez(blob, **arrays, metadata_json=np.asarray(json.dumps(metadata)))
                encoded = blob.getvalue()
                prepare_ms = (time.perf_counter() - started) * 1000
                request = urllib.request.Request(args.server + "/predict", data=encoded,
                                                 headers={"Content-Type": "application/octet-stream"})
                rpc_start = time.perf_counter()
                with opener.open(request, timeout=2) as response:
                    reply = json.load(response)
                anchor = metadata["samples"][-1]["target_stamp_seconds"]
                action = validate_reply(reply, request_id, anchor, health["checkpoint_step"])
                current = arrays["agent_pos"][-1]
                row.update({"status": "PREDICTION", "rpc_ms": (time.perf_counter() - rpc_start) * 1000,
                            "prepare_observation_and_encode_ms": prepare_ms,
                            "payload_bytes": len(encoded),
                            "observation_to_response_ms": (time.time() - anchor) * 1000,
                            "server_predict_and_decode_ms": reply["server_predict_and_decode_ms"],
                            "max_abs_first_q_gap_deg": float(np.rad2deg(np.abs(action[0, :7] - current[:7])).max()),
                            "gripper_prediction_range": [float(action[:, 7].min()), float(action[:, 7].max())],
                            "response": reply})
                if previous is not None:
                    elapsed = anchor - previous["observation_target_stamp_seconds"]
                    if 0 < elapsed <= previous["curve"]["valid_future_seconds"]:
                        old_q, old_dq, _ = evaluate_curve(previous["curve"], elapsed)
                        new_q, new_dq, _ = evaluate_curve(reply["curve"], 0)
                        row["hypothetical_handoff_max_q_gap_deg"] = float(np.rad2deg(np.abs(old_q[:7] - new_q[:7])).max())
                        row["hypothetical_handoff_max_dq_gap_rad_s"] = float(np.abs(old_dq[:7] - new_dq[:7]).max())
                previous = reply
                if not (args.output / "first_observation.npz").exists():
                    (args.output / "first_observation.npz").write_bytes(encoded)
                (args.output / "latest_observation.npz").write_bytes(encoded)
            except (ValueError, KeyError, urllib.error.URLError, TimeoutError) as error:
                row.update({"status": "SKIPPED", "reason": str(error)})
            row["client_iteration_ms"] = (time.perf_counter() - started) * 1000
            rows.append(row)
            with (args.output / "predictions.jsonl").open("a") as stream:
                stream.write(json.dumps(row, allow_nan=False) + "\n")
            print(json.dumps({key: value for key, value in row.items() if key != "response"}), flush=True)
            next_query = max(next_query + DT, time.monotonic())
    finally:
        stopping.set()
        worker.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
    valid = [row for row in rows if row["status"] == "PREDICTION"]
    metrics = {}
    for key in ("rpc_ms", "observation_to_response_ms", "server_predict_and_decode_ms", "client_iteration_ms",
                "prepare_observation_and_encode_ms",
                "max_abs_first_q_gap_deg", "hypothetical_handoff_max_q_gap_deg", "hypothetical_handoff_max_dq_gap_rad_s"):
        values = [row[key] for row in valid if key in row]
        if values:
            metrics[key] = dict(zip(("median", "p95", "max"), np.quantile(values, [.5, .95, 1]).tolist()))
    report = {"status": "SHADOW_DIAGNOSTIC_COMPLETE" if len(valid) >= 5 else "INSUFFICIENT_VALID_PREDICTIONS",
              "robot_motion_commanded": False, "attempts": len(rows), "valid_predictions": len(valid),
              "skipped": len(rows) - len(valid), "server": health, "metrics": metrics,
              "skip_reasons": dict(Counter(row["reason"] for row in rows if row["status"] == "SKIPPED")),
              "scope": "real observation shadow predictions; handoff gaps compare hypothetical curves, not executed robot jumps",
              "report": str((args.output / "report.json").resolve())}
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print("FLASH_SHADOW_DIAGNOSTIC=" + json.dumps(report), flush=True)
    if len(valid) < 5:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
