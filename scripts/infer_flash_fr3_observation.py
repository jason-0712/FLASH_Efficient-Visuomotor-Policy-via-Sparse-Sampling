#!/usr/bin/env python3
"""Run saved FLASH EMA weights on a real observation file. No robot/network IO."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import torch
from hydra.utils import instantiate
from omegaconf import OmegaConf

from flash_fr3_dataset import physical_time_spec, verify_cache
from train_flash_fr3 import policy_config, source_info


def read_observation(path):
    with np.load(path, allow_pickle=False) as data:
        metadata = json.loads(str(data["metadata_json"].item()))
        arrays = {key: data[key].copy() for key in ("agent_pos", "primary_cam", "wrist_cam")}
    if metadata.get("format") != "flash_fr3_live_observation_v1":
        raise ValueError("不支持的观测格式")
    if metadata.get("joint_order") != [f"fr3_joint{i}" for i in range(1, 8)]:
        raise ValueError("关节顺序不一致")
    if metadata.get("n_obs_steps") != 8 or metadata.get("color") != "RGB":
        raise ValueError("观测历史或颜色格式不一致")
    state = arrays["agent_pos"]
    if state.shape != (8, 8) or state.dtype != np.float32 or not np.isfinite(state).all():
        raise ValueError("状态必须为有限 float32 [8,8]")
    for key in ("primary_cam", "wrist_cam"):
        if arrays[key].shape != (8, 256, 256, 3) or arrays[key].dtype != np.uint8:
            raise ValueError(f"{key} 必须为 uint8 [8,256,256,3]")
    dt = metadata["sample_dt_seconds"]
    if not math.isclose(dt, 4 / 15, rel_tol=1e-9):
        raise ValueError("实时观测采样间隔与训练不一致")
    samples = metadata["samples"]
    relative = np.asarray([sample["relative_seconds"] for sample in samples])
    if relative.shape != (8,) or not np.isfinite(relative).all() or np.max(np.abs(relative - np.arange(8) * dt)) > 0.05:
        raise ValueError("历史观测时序不一致")
    keys = ("primary_cam", "wrist_cam", "arm", "hand")
    if set(metadata["topics"]) != set(keys):
        raise ValueError("观测话题集合不完整")
    targets = np.asarray([sample["target_stamp_seconds"] for sample in samples], dtype=np.float64)
    if not np.isfinite(targets).all() or not np.allclose(np.diff(targets), dt, atol=1e-5, rtol=0):
        raise ValueError("目标时间戳网格异常")
    all_stamps = []
    for key in keys:
        stamps = np.asarray([sample["stamp_seconds"][key] for sample in samples])
        if not np.isfinite(stamps).all() or np.any(np.diff(stamps) <= 0):
            raise ValueError(f"{key} 时间戳不递增")
        if np.max(np.abs(stamps - targets)) > 0.080001:
            raise ValueError(f"{key} 源时间戳与采样网格偏差过大")
        ages = np.asarray([sample["source_age_at_receipt_seconds"][key] for sample in samples])
        if not np.isfinite(ages).all() or np.any(ages < -0.05) or np.any(ages > 0.3):
            raise ValueError(f"{key} 源消息到达时已过期或时钟异常")
        all_stamps.append(stamps)
    if np.max(np.ptp(np.asarray(all_stamps), axis=0)) > 0.150001:
        raise ValueError("不同传感器时间戳偏差过大")
    return arrays, metadata


def load_policy(flash_root, dataset_cache, checkpoint, device_name):
    """Shared strict loading path for file inference and the shadow server."""
    root = Path(flash_root).resolve()
    source = source_info(root)
    sys.path.insert(0, str(root))
    manifest, cache_id = verify_cache(dataset_cache)
    # Only load the user's own training checkpoint.
    payload = torch.load(checkpoint, map_location="cpu", weights_only=False, mmap=True)
    if payload.get("format") != "flash_fr3_checkpoint_v1":
        raise ValueError("检查点格式不一致")
    config, step = payload["config"], payload["step"]
    if config.get("checkpoint_purpose") != "offline_training_requires_real_robot_validation":
        raise ValueError("需要正式训练检查点")
    if config["source"] != source or config["cache_manifest_sha256"] != cache_id or config["policy"] != policy_config(root, manifest):
        raise ValueError("训练源码、数据或策略配置不一致")
    torch.set_num_threads(4)
    device = torch.device(device_name)
    if device.type == "cuda":
        torch.cuda.set_device(device)
    model = instantiate(OmegaConf.create(config["policy"]))
    model.load_state_dict(payload["ema_model"], strict=True)
    del payload
    model.to(device).eval().requires_grad_(False)
    model.eval_history_noise_std = 0.0
    times = physical_time_spec(model, manifest)
    if any(not math.isclose(value, config["physical_time"][key], rel_tol=1e-10, abs_tol=1e-12)
           for key, value in times.items()):
        raise ValueError("检查点物理时间尺度不一致")
    return model, {"source": source, "checkpoint_step": step, "physical_time": times}


@torch.no_grad()
def predict_arrays(model, arrays):
    device = model.device
    observation = {"agent_pos": torch.from_numpy(arrays["agent_pos"]).unsqueeze(0).to(device)}
    for key in ("primary_cam", "wrist_cam"):
        observation[key] = torch.from_numpy(arrays[key]).permute(0, 3, 1, 2).unsqueeze(0).to(device).float().div_(255)
    prediction = model.predict_action(observation)
    if any(not torch.isfinite(value).all() for value in prediction.values()):
        raise RuntimeError("模型预测出现非有限数")
    return prediction


@torch.no_grad()
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flash-root", type=Path, required=True)
    parser.add_argument("--dataset-cache", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--observation", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("输出目录已存在，请使用新目录")
    arrays, metadata = read_observation(args.observation)
    model, context = load_policy(args.flash_root, args.dataset_cache, args.checkpoint, args.device)
    source, step, times = context["source"], context["checkpoint_step"], context["physical_time"]
    device = model.device
    prediction = predict_arrays(model, arrays)
    duration = times["polynomial_duration_seconds"]
    output = {key: value.detach().cpu().numpy() for key, value in prediction.items()}
    output["velocity_rad_s"] = output["velocity"][..., :7] / duration
    output["acceleration_rad_s2"] = output["acceleration"][..., :7] / duration**2
    action = output["action"][0]
    if action.shape != (8, 8):
        raise ValueError("预测动作形状异常")
    current = arrays["agent_pos"][-1]
    gap = action[0, :7] - current[:7]
    stats = model.normalizer["agent_pos"].get_input_stats()
    state_min = stats["min"].cpu().numpy()
    state_max = stats["max"].cpu().numpy()
    outside = (current < state_min - 1e-5) | (current > state_max + 1e-5)
    report = {"status": "REAL_OBSERVATION_INFERENCE_COMPLETE", "robot_motion_commanded": False,
              "scope": "one captured real observation history; no closed-loop execution or task success measurement",
              "checkpoint": str(args.checkpoint.resolve()), "checkpoint_step": step, "weights": "ema_model",
              "observation": str(args.observation.resolve()),
              "observation_sha256": hashlib.sha256(args.observation.read_bytes()).hexdigest(),
              "device": str(device), "source": source, "physical_time": times,
              "prediction_shape": list(action.shape), "current_state": current.tolist(),
              "first_prediction": action[0].tolist(), "first_q_gap_deg": np.rad2deg(gap).tolist(),
              "max_abs_first_q_gap_deg": float(np.rad2deg(np.abs(gap).max())),
              "gripper_prediction_range": [float(action[:, 7].min()), float(action[:, 7].max())],
              "state_outside_training_observed_range": np.flatnonzero(outside).tolist(),
              "training_state_min": state_min.tolist(), "training_state_max": state_max.tolist(),
              "observation_metadata": metadata}
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output / "prediction.npz", **output)
    (args.output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    summary = {key: report[key] for key in ("status", "checkpoint_step", "weights", "prediction_shape",
               "max_abs_first_q_gap_deg", "gripper_prediction_range", "state_outside_training_observed_range", "robot_motion_commanded")}
    summary["report"] = str((args.output / "report.json").resolve())
    print("FLASH_REAL_OBSERVATION_INFERENCE=" + json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
