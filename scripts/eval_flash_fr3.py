#!/usr/bin/env python3
"""Evaluate saved FLASH EMA weights on held-out demonstrations; no robot IO."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch
from hydra.utils import instantiate
from omegaconf import OmegaConf
from torch.utils.data import DataLoader, Subset

from flash_fr3_dataset import FlashFR3Dataset, inference_obs, physical_time_spec, verify_cache
from train_flash_fr3 import move, policy_config, source_info


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def error_summary(prediction, target, mask=None):
    error = np.abs(np.asarray(prediction, dtype=np.float64) - target)
    selected = error.reshape(-1, 8) if mask is None else error[mask]
    if not len(selected):
        return {"action_points": 0}
    q, grip = selected[:, :7], selected[:, 7]
    return {"action_points": len(selected), "joint_mae_rad": float(q.mean()),
            "joint_mae_deg": float(np.rad2deg(q.mean())),
            "joint_p95_abs_error_deg": float(np.rad2deg(np.percentile(q, 95))),
            "joint_max_abs_error_deg": float(np.rad2deg(q.max())),
            "per_joint_mae_deg": np.rad2deg(q.mean(axis=0)).tolist(),
            "gripper_mae": float(grip.mean())}


def absolute_summary(values):
    values = np.abs(np.asarray(values, dtype=np.float64))
    if not values.size:
        return {"pairs": 0}
    return {"pairs": len(values), "mean_abs": float(values.mean()),
            "p95_abs": float(np.percentile(values, 95)), "max_abs": float(values.max()),
            "per_joint_max_abs": values.max(axis=0).tolist()}


def handoff_summary(current, ends, position, velocity, offset, old_index, anchor):
    lookup = {int(c): i for i, c in enumerate(current)}
    pairs = [(i, lookup[int(c + offset)]) for i, c in enumerate(current)
             if c + offset < ends[i] and int(c + offset) in lookup]
    if not pairs:
        return {"position_rad": {"pairs": 0}, "velocity_rad_s": {"pairs": 0}}
    left, right = np.array(pairs).T
    return {
        "position_rad": absolute_summary(position[left, old_index, :7] - position[right, anchor, :7]),
        "velocity_rad_s": absolute_summary(velocity[left, old_index, :7] - velocity[right, anchor, :7]),
    }


def build_basis(model, s):
    builder = model._build_legendre_basis if model.basis_type == "legendre" else model._build_power_basis
    return builder(s.detach().cpu().double(), model.poly_order)


@torch.no_grad()
def derivative_check(model, prediction, duration):
    s = model._time_points.detach().cpu().double()
    eps = 1e-4
    plus, dplus, _ = build_basis(model, s + eps)
    minus, dminus, _ = build_basis(model, s - eps)
    coeff = prediction["coefficients"].double() * model._coeff_scale.double()
    scale = model.normalizer["action"].params_dict["scale"].double()
    result = {}
    for key, matrix, power in (("velocity", (plus - minus) / (2 * eps), 1),
                               ("acceleration", (dplus - dminus) / (2 * eps), 2)):
        numeric = torch.einsum("tc,bcd->btd", matrix.to(coeff.device), coeff) / scale / duration**power
        analytic = prediction[key].double() / duration**power
        delta = (numeric[..., :7] - analytic[..., :7]).abs()
        if not torch.allclose(numeric[..., :7], analytic[..., :7], atol=5e-3, rtol=3e-3):
            raise RuntimeError(f"{key} 物理时间导数校验失败，max_error={delta.max().item()}")
        result[f"{key}_max_abs_error"] = delta.max().item()
    result["status"] = "PASS"
    result["units"] = "velocity: rad/s; acceleration: rad/s^2; seven joints only"
    return result


@torch.no_grad()
def measure_latency(model, obs, device, warmup, repeats):
    def sync():
        if device.type == "cuda":
            torch.cuda.synchronize(device)
    for _ in range(warmup):
        model.predict_action(obs)
    sync()
    durations = []
    for _ in range(repeats):
        sync()
        start = time.perf_counter()
        model.predict_action(obs)
        sync()
        durations.append((time.perf_counter() - start) * 1000)
    return {"batch_size": 1, "warmup": warmup, "repeats": repeats,
            "median_ms": float(np.median(durations)), "p95_ms": float(np.percentile(durations, 95)),
            "max_ms": max(durations),
            "scope": "predict_action with device-resident inputs and device synchronization; excludes camera IO, image conversion, network and controller"}


@torch.no_grad()
def evaluate(model, data, device, batch_size, max_windows, times):
    n = min(max_windows, len(data)) if max_windows else len(data)
    loader = DataLoader(Subset(data, range(n)), batch_size=batch_size, shuffle=False,
                        num_workers=0, pin_memory=device.type == "cuda")
    duration = times["polynomial_duration_seconds"]
    # Sample the entire current-to-next-block-boundary interval, including the suffix.
    anchor = model._anchor_idx
    boundary = anchor + model.n_action_steps
    if boundary >= model._H_poly:
        raise ValueError("当前评估器需要官方 poly_suffix=1 的配置")
    s = torch.linspace(float(model._time_points[anchor]), float(model._time_points[boundary]), 161)
    basis = [m.to(device=device, dtype=torch.float32) for m in build_basis(model, s)]
    scale = model.normalizer["action"].params_dict["scale"].view(1, 1, -1)
    collected = {key: [] for key in ("prediction", "target", "coefficients", "core_position",
                 "core_velocity_rad_s", "current_state", "dense_q_min", "dense_q_max",
                 "dense_dq_max", "dense_ddq_max", "dense_grip_min", "dense_grip_max")}
    checks, latency_obs = None, None
    for i, batch in enumerate(loader):
        batch = move(batch, device)
        obs = inference_obs(batch, model.fit_pad, model.n_obs_steps)
        pred = model.predict_action(obs)
        if any(not torch.isfinite(t).all() for t in pred.values()):
            raise RuntimeError("预测包含非有限数")
        if checks is None:
            checks = derivative_check(model, pred, duration)
            repeat = model.predict_action(obs)
            repeat_error = (repeat["action"] - pred["action"]).abs().max().item()
            if repeat_error > 1e-6:
                raise RuntimeError(f"重复推理结果不一致：{repeat_error}")
            checks["repeat_inference_max_abs_difference"] = repeat_error
            latency_obs = {key: value[:1].contiguous() for key, value in obs.items()}
        start = model.fit_pad + model.n_obs_steps - 1
        target = batch["action"][:, start:start + model.n_action_steps]
        if pred["action"].shape != target.shape:
            raise RuntimeError("预测和目标的形状不一致")
        raw = pred["coefficients"] * model._coeff_scale
        dense_q = model.normalizer["action"].unnormalize(torch.einsum("tc,bcd->btd", basis[0], raw))
        dense_dq = torch.einsum("tc,bcd->btd", basis[1], raw) / scale / duration
        dense_ddq = torch.einsum("tc,bcd->btd", basis[2], raw) / scale / duration**2
        values = {"prediction": pred["action"], "target": target,
                  "coefficients": pred["coefficients"], "core_position": pred["action_pred"],
                  "core_velocity_rad_s": pred["velocity"] / duration,
                  "current_state": obs["agent_pos"][:, -1],
                  "dense_q_min": dense_q[..., :7].amin(dim=1), "dense_q_max": dense_q[..., :7].amax(dim=1),
                  "dense_dq_max": dense_dq[..., :7].abs().amax(dim=1),
                  "dense_ddq_max": dense_ddq[..., :7].abs().amax(dim=1),
                  "dense_grip_min": dense_q[..., 7].amin(dim=1), "dense_grip_max": dense_q[..., 7].amax(dim=1)}
        if any(not torch.isfinite(t).all() for t in values.values()):
            raise RuntimeError("轨迹解码包含非有限数")
        for key, value in values.items():
            collected[key].append(value.detach().cpu().numpy())
        print(f"离线评估：{min((i + 1) * batch_size, n)}/{n} 个验证窗口", flush=True)
    return {key: np.concatenate(value) for key, value in collected.items()}, checks, latency_obs


def summarize(arrays, data, model, times):
    n = len(arrays["prediction"])
    current, starts, ends = data.samples[:n].T
    offsets = np.arange(model.n_action_steps)
    valid = current[:, None] + offsets < ends[:, None]
    complete = (current - starts >= model.n_obs_steps - 1) & valid.all(axis=1)
    pred, target = arrays["prediction"], arrays["target"]
    result = {"all_points_including_end_padding": error_summary(pred, target),
              "without_end_padding": error_summary(pred, target, valid),
              "complete_history_and_action_windows": error_summary(pred[complete], target[complete]),
              "complete_windows": int(complete.sum()), "per_horizon": [], "per_episode": []}
    for h in offsets:
        row = error_summary(pred[:, h:h+1], target[:, h:h+1], valid[:, h:h+1])
        row.update({"offset": int(h), "seconds_from_current": float(h * times["sample_dt_seconds"])})
        result["per_horizon"].append(row)
    ep_by_start = {ep["start"]: ep for ep in data.episodes}
    for start in np.unique(starts):
        ids = starts == start
        row = {key: ep_by_start[start][key] for key in ("episode_index", "source_id", "group")}
        row.update(error_summary(pred[ids], target[ids], valid[ids]))
        result["per_episode"].append(row)
    result["current_position_gap_rad"] = absolute_summary(pred[:, 0, :7] - arrays["current_state"][:, :7])
    position, velocity = arrays["core_position"], arrays["core_velocity_rad_s"]
    anchor = model._anchor_idx
    result["teacher_history_handoff_after_one_sparse_step"] = handoff_summary(
        current, ends, position, velocity, 1, anchor + 1, anchor)
    result["teacher_history_handoff_after_full_action_chunk"] = handoff_summary(
        current, ends, position, velocity, model.n_action_steps, anchor + model.n_action_steps, anchor)
    train_stats = model.normalizer["action"].get_input_stats()
    qmin = train_stats["min"][:7].detach().cpu().numpy()
    qmax = train_stats["max"][:7].detach().cpu().numpy()
    outside = (arrays["dense_q_min"] < qmin - 1e-5) | (arrays["dense_q_max"] > qmax + 1e-5)
    result["dense_curve_diagnostics"] = {
        "grid_points_per_window": 161, "interval_seconds": times["action_chunk_interval_seconds"],
        "q_min_rad": arrays["dense_q_min"].min(axis=0).tolist(),
        "q_max_rad": arrays["dense_q_max"].max(axis=0).tolist(),
        "peak_abs_velocity_rad_s": arrays["dense_dq_max"].max(axis=0).tolist(),
        "peak_abs_acceleration_rad_s2": arrays["dense_ddq_max"].max(axis=0).tolist(),
        "gripper_raw_min": float(arrays["dense_grip_min"].min()),
        "gripper_raw_max": float(arrays["dense_grip_max"].max()),
        "windows_outside_training_observed_joint_range": int(outside.any(axis=1).sum()),
        "training_observed_q_min_rad": qmin.tolist(), "training_observed_q_max_rad": qmax.tolist(),
        "interpretation": "sampled grid, not certified extrema; training observed range is not a hardware joint limit"}
    arrays.update({"current_cache_index": current, "episode_start": starts, "episode_end": ends,
                   "nonpadding_target_mask": valid})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flash-root", type=Path, required=True)
    parser.add_argument("--dataset-cache", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--cpu-threads", type=int, default=4)
    parser.add_argument("--max-windows", type=int, default=0, help="0 means all; nonzero is a partial diagnostic")
    parser.add_argument("--latency-repeats", type=int, default=100)
    parser.add_argument("--latency-warmup", type=int, default=10)
    args = parser.parse_args()
    if min(args.batch_size, args.cpu_threads, args.latency_repeats) < 1 or min(args.max_windows, args.latency_warmup) < 0:
        parser.error("数值参数超出有效范围")
    if args.output.exists():
        parser.error("输出目录已存在，请使用新的目录")
    device = torch.device(args.device)
    if device.type == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError("当前环境没有可用 CUDA")
        torch.cuda.set_device(device)
    torch.set_num_threads(args.cpu_threads)
    root = args.flash_root.resolve()
    source = source_info(root)
    sys.path.insert(0, str(root))
    print("核对缓存和检查点，加载 EMA 权重……", flush=True)
    manifest, cache_id = verify_cache(args.dataset_cache)
    # Only load the user's own training checkpoint, never an untrusted pickle.
    payload = torch.load(args.checkpoint, map_location="cpu", weights_only=False, mmap=True)
    if payload.get("format") != "flash_fr3_checkpoint_v1":
        raise ValueError("不支持的检查点格式")
    config, step = payload["config"], payload["step"]
    if config.get("checkpoint_purpose") != "offline_training_requires_real_robot_validation":
        raise ValueError("此入口只接受正式离线训练检查点，不接受 smoke 检查点")
    if source != config["source"] or cache_id != config["cache_manifest_sha256"]:
        raise ValueError("源码或缓存与训练时不一致")
    if config["policy"] != policy_config(root, manifest):
        raise ValueError("检查点策略配置与本次 FR3 适配配置不一致")
    model = instantiate(OmegaConf.create(config["policy"]))
    model.load_state_dict(payload["ema_model"], strict=True)
    del payload
    model.to(device).eval()
    model.requires_grad_(False)
    model.eval_history_noise_std = 0.0
    times = physical_time_spec(model, manifest)
    if any(not math.isclose(value, config["physical_time"][key], rel_tol=1e-10, abs_tol=1e-12)
           for key, value in times.items()):
        raise ValueError("物理时间尺度与训练检查点不一致")
    data = FlashFR3Dataset(args.dataset_cache, "val", n_obs=model.n_obs_steps,
                           horizon=model.horizon, fit_pad=model.fit_pad)
    arrays, checks, latency_obs = evaluate(model, data, device, args.batch_size, args.max_windows, times)
    metrics = summarize(arrays, data, model, times)
    partial = len(arrays["prediction"]) != len(data)
    reproduction = {"status": "NOT_CHECKED_PARTIAL" if partial else "NO_MATCHING_TRAINING_LOG"}
    training_log = args.checkpoint.parent / "metrics.jsonl"
    if not partial and training_log.exists():
        matches = [json.loads(line) for line in training_log.read_text().splitlines() if line.strip()]
        matches = [row for row in matches if row.get("step") == step and "val_joint_mae_rad" in row]
        if matches:
            actual = metrics["all_points_including_end_padding"]
            deltas = {key: abs(actual[key] - matches[-1]["val_" + key])
                      for key in ("joint_mae_rad", "gripper_mae")}
            reproduction = {"status": "PASS" if max(deltas.values()) <= 1e-5 else "MISMATCH",
                            "absolute_differences": deltas, "tolerance": 1e-5}
    report = {"format": "flash_fr3_offline_eval_v1", "status": "PARTIAL_DIAGNOSTIC" if partial else "OFFLINE_EVALUATION_COMPLETE",
              "robot_motion": False, "checkpoint": str(args.checkpoint.resolve()),
              "checkpoint_sha256": file_sha256(args.checkpoint), "checkpoint_step": step,
              "evaluation_script_sha256": file_sha256(Path(__file__)),
              "versions": {"python": sys.version, "torch": torch.__version__, "numpy": np.__version__},
              "weights": "ema_model", "source": source, "cache_manifest_sha256": cache_id,
              "device": str(device), "gpu_name": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
              "validation_windows": len(arrays["prediction"]), "total_validation_windows": len(data),
              "physical_time": times, "derivative_check": checks, "training_log_reproduction": reproduction,
              "metrics": metrics, "latency": measure_latency(model, latency_obs, device, args.latency_warmup, args.latency_repeats),
              "scope": "held-out demonstration histories, overlapping windows; not closed-loop rollout or robot deployment qualification"}
    args.output.mkdir(parents=True)
    (args.output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    np.savez_compressed(args.output / "predictions.npz", **arrays)
    for key in ("per_episode", "per_horizon"):
        rows = metrics[key]
        with (args.output / f"{key}.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    summary = {"status": report["status"], "checkpoint_step": step, "validation_windows": report["validation_windows"],
               "weights": "ema_model", "joint_mae_deg": metrics["all_points_including_end_padding"]["joint_mae_deg"],
               "gripper_mae": metrics["all_points_including_end_padding"]["gripper_mae"],
               "training_log_reproduction": reproduction, "derivative_check": checks,
               "current_position_gap_rad": metrics["current_position_gap_rad"],
               "handoff_after_one_sparse_step": metrics["teacher_history_handoff_after_one_sparse_step"],
               "handoff_after_full_action_chunk": metrics["teacher_history_handoff_after_full_action_chunk"],
               "dense_curve_diagnostics": metrics["dense_curve_diagnostics"], "latency": report["latency"],
               "report": str((args.output / "report.json").resolve()), "robot_motion": False}
    print("FLASH_OFFLINE_EVAL=" + json.dumps(summary, ensure_ascii=False, allow_nan=False), flush=True)
    if reproduction["status"] == "MISMATCH":
        raise SystemExit("重载结果与训练日志不一致，请先检查报告")


if __name__ == "__main__":
    main()
