#!/usr/bin/env python3
"""Capture eight real ROS observations. Creates subscriptions only; no commands."""
from __future__ import annotations

import argparse
import hashlib
import json
import socket
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, JointState

JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
FINGERS = ["fr3_finger_joint1", "fr3_finger_joint2"]
TOPICS = {
    "primary_cam": "/right/right_third_person_camera/color/image_raw/compressed",
    "wrist_cam": "/right/right_wrist_camera/color/image_raw/compressed",
    "arm": "/franka/joint_states",
    "hand": "/franka_gripper/joint_states",
}
DT = 4 / 15


def measured_gripper_closure(width):
    """Match CRISP's bounded observation convention, not an action clamp."""
    width = float(width)
    if not np.isfinite(width) or not -.001 <= width <= .081:
        raise ValueError(f"夹爪宽度异常: {width}")
    return np.float32(np.clip(1-width/.08, 0., 1.))


def ordered_positions(message, names):
    if len(message.name) != len(set(message.name)) or len(message.name) != len(message.position):
        raise ValueError("JointState 名称重复或 position 长度不一致")
    values = dict(zip(message.name, message.position))
    result = np.asarray([values[name] for name in names], dtype=np.float32)
    if not np.isfinite(result).all():
        raise ValueError("关节位置含非有限数")
    return result


def resize_rgb(image):
    # Matches CRISP Camera._resize_with_aspect_ratio, with dual_cam_franka.yaml:
    # 256x256, no pre-crop, INTER_AREA and aspect-preserving central crop.
    height, width = image.shape[:2]
    if (height, width) == (256, 256):
        return np.ascontiguousarray(image)
    scale = max(256 / width, 256 / height)
    new_w, new_h = int(width * scale), int(height * scale)
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    x, y = (new_w - 256) // 2, (new_h - 256) // 2
    return np.ascontiguousarray(resized[y:y + 256, x:x + 256])


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=20)
    parser.add_argument("--max-age", type=float, default=0.3)
    parser.add_argument("--max-skew", type=float, default=0.15)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("输出已存在，请使用新目录")
    if not all(np.isfinite(v) and v > 0 for v in (args.timeout, args.max_age, args.max_skew)):
        parser.error("时间参数必须为有限正数")
    rclpy.init()
    node = rclpy.create_node("flash_fr3_capture_observation")
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT)
    history = {key: deque(maxlen=4000) for key in TOPICS}
    subscriptions = []
    for key, topic in TOPICS.items():
        def receive(message, key=key):
            history[key].append((message, time.monotonic(), time.time()))
        subscriptions.append(node.create_subscription(
            CompressedImage if key.endswith("_cam") else JointState, topic, receive, qos))
    bridge = CvBridge()
    arrays = {key: [] for key in ("agent_pos", "primary_cam", "wrist_cam")}
    samples, previous = [], {}
    deadline = time.monotonic() + args.timeout
    first = None
    last_reason = "等待相机、机械臂和夹爪状态"
    try:
        # Buffer first, then select the closest source-stamped messages to the
        # fixed training grid. Image conversion cannot delay that sampling grid.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.002)
            missing = [key for key, values in history.items() if not values]
            if missing:
                last_reason = "缺少话题: " + ", ".join(missing)
                continue
            if first is None:
                first = time.time() + 0.25  # let DDS discovery finish before the first target
            if time.time() < first + 7 * DT + 0.1:
                continue
            break
        else:
            raise RuntimeError("采集超时；" + last_reason)
        for index in range(8):
            target = first + index * DT
            selected = {key: min(values, key=lambda value: abs(stamp_seconds(value[0]) - target))
                        for key, values in history.items()}
            messages = {key: value[0] for key, value in selected.items()}
            stamps = {key: stamp_seconds(msg) for key, msg in messages.items()}
            ages = {key: selected[key][2] - stamp for key, stamp in stamps.items()}
            offsets = {key: stamp - target for key, stamp in stamps.items()}
            if any(age < -0.05 or age > args.max_age for age in ages.values()):
                raise RuntimeError(f"观测到达时已过期或时钟不一致: {ages}")
            if any(abs(offset) > 0.08 for offset in offsets.values()):
                raise RuntimeError(f"传感器未覆盖所需历史采样时间: {offsets}")
            if max(stamps.values()) - min(stamps.values()) > args.max_skew:
                raise RuntimeError("四路观测时间戳间隔过大")
            if any(stamps[key] <= previous.get(key, -1) for key in TOPICS):
                raise RuntimeError("传感器历史帧重复或时间倒退")
            q = ordered_positions(messages["arm"], JOINTS)
            fingers = ordered_positions(messages["hand"], FINGERS)
            width = float(fingers.sum())
            state = np.r_[q, measured_gripper_closure(width)].astype(np.float32)
            arrays["agent_pos"].append(state)
            image_shapes = {}
            for key in ("primary_cam", "wrist_cam"):
                rgb = np.asarray(bridge.compressed_imgmsg_to_cv2(messages[key], desired_encoding="rgb8"))
                image_shapes[key] = list(rgb.shape)
                arrays[key].append(resize_rgb(rgb))
            samples.append({"relative_seconds": index * DT, "target_stamp_seconds": target,
                            "stamp_seconds": stamps, "source_age_at_receipt_seconds": ages,
                            "source_offset_from_target_seconds": offsets,
                            "source_arm_name_order": list(messages["arm"].name),
                            "image_shapes": image_shapes})
            previous = stamps
    finally:
        node.destroy_node()
        rclpy.shutdown()
    arrays = {key: np.stack(value) for key, value in arrays.items()}
    metadata = {"format": "flash_fr3_live_observation_v1", "host": socket.gethostname(),
                "robot_motion_commanded": False, "topics": TOPICS, "joint_order": JOINTS,
                "sample_dt_seconds": DT, "n_obs_steps": 8, "color": "RGB",
                "preprocessing": "CRISP compressed transport -> cv_bridge RGB8 -> aspect-preserving resize INTER_AREA -> center crop 256x256; no pre-crop",
                "gripper_state": "closure = clip(1 - (finger1 + finger2) / 0.08, 0, 1)",
                "freshness_limits_seconds": {"source_age_at_receipt": args.max_age,
                                             "source_offset_from_target": 0.08, "sensor_skew": args.max_skew},
                "samples": samples}
    args.output.mkdir(parents=True)
    path = args.output / "observation.npz"
    np.savez_compressed(path, **arrays, metadata_json=np.asarray(json.dumps(metadata)))
    (args.output / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n")
    for key in ("primary_cam", "wrist_cam"):
        if not cv2.imwrite(str(args.output / f"{key}_latest.png"), arrays[key][-1, :, :, ::-1]):
            raise RuntimeError("保存相机预览失败")
    print("FLASH_LIVE_OBSERVATION=" + json.dumps({"status": "PASS", "observations": 8,
          "output": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
          "last_state": arrays["agent_pos"][-1].tolist(), "robot_motion_commanded": False}), flush=True)


if __name__ == "__main__":
    main()
