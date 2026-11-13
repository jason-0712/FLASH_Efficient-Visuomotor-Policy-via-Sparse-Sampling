#!/usr/bin/env python3
"""Train the upstream FLASH policy on FR3 demonstrations, without a simulator.

This entry point never imports ROS, robot controllers, or simulation runners.
Smoke checkpoints are diagnostic artifacts and cannot qualify for deployment.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.metadata
import json
import math
import random
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import torch
from hydra.utils import instantiate
from omegaconf import OmegaConf
from torch.utils.data import DataLoader

from flash_fr3_dataset import FlashFR3Dataset, inference_obs, physical_time_spec, verify_cache

SOURCE_COMMIT = "f47e490f2b820f835d1b9145da3cdb3df69f50a0"


def source_info(root):
    root = root.resolve()
    if (root / ".git").exists():
        commit = subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    else:
        commit = (root / "FLASH_SOURCE_COMMIT").read_text().strip()
    if commit != SOURCE_COMMIT:
        raise ValueError(f"请使用固定 FLASH 版本 {SOURCE_COMMIT}，当前为 {commit}")
    paths = sorted(p for p in (root / "roboverse_learn/il").rglob("*") if p.suffix in (".py", ".yaml"))
    digest = hashlib.sha256()
    for path in paths:
        digest.update(str(path.relative_to(root)).encode() + b"\0" + path.read_bytes())
    return {"commit": commit, "il_source_sha256": digest.hexdigest()}


def policy_config(root, manifest):
    height, width, _ = manifest["image_shape_hwc"]
    shape = {"action": {"shape": [8]}, "obs": {
        "primary_cam": {"shape": [3, height, width], "type": "rgb"},
        "wrist_cam": {"shape": [3, height, width], "type": "rgb"},
        "agent_pos": {"shape": [8], "type": "low_dim"}}}
    cfg = OmegaConf.create({"shape_meta": shape, "horizon": 16, "n_obs_steps": 8,
                            "n_action_steps": 8, "n_latency_steps": 0, "obs_as_global_cond": True})
    cfg.policy = OmegaConf.load(root / "roboverse_learn/il/configs/policy_config/flash.yaml")
    # Replace the upstream eval interpolation with its numeric result; no eval resolver needed.
    cfg.policy.n_action_steps = cfg.n_action_steps
    return OmegaConf.to_container(cfg.policy, resolve=True)


def move(batch, device):
    return {"obs": {key: value.to(device, non_blocking=True) for key, value in batch["obs"].items()},
            "action": batch["action"].to(device, non_blocking=True)}


def cosine_lr_factor(step, total_steps, warmup_steps=500):
    if step < warmup_steps:
        return step / warmup_steps
    annealed_step = min(step, total_steps) - warmup_steps
    return max(0.0, 0.5 * (1 + math.cos(math.pi * annealed_step / (total_steps - warmup_steps))))


@torch.no_grad()
def validate(model, loader, device, max_batches):
    model.eval()
    joint_error = grip_error = count = 0
    anchor_error = anchor_max = anchor_count = 0
    for i, batch in enumerate(loader):
        if i >= max_batches:
            break
        batch = move(batch, device)
        pred = model.predict_action(inference_obs(batch, model.fit_pad, model.n_obs_steps))["action"]
        start = model.fit_pad + model.n_obs_steps - 1
        target = batch["action"][:, start:start + model.n_action_steps]
        if pred.shape != target.shape or not torch.isfinite(pred).all():
            raise RuntimeError("验证预测形状或数值异常")
        joint_error += (pred[..., :7] - target[..., :7]).abs().sum().item()
        grip_error += (pred[..., 7] - target[..., 7]).abs().sum().item()
        count += target.shape[0] * target.shape[1]
        anchor = (pred[:, 0, :7] - target[:, 0, :7]).abs()
        anchor_error += anchor.sum().item()
        anchor_max = max(anchor_max, anchor.max().item())
        anchor_count += target.shape[0]
    if count == 0:
        raise RuntimeError("验证集为空")
    return {"val_joint_mae_rad": joint_error / (7 * count), "val_gripper_mae": grip_error / count,
            "val_action_points": count,
            "val_anchor_joint_mae_rad": anchor_error / (7 * anchor_count),
            "val_anchor_joint_max_rad": anchor_max}


@torch.no_grad()
def check_derivative(model, prediction, duration):
    """Check analytic normalized-s derivatives against physical-time differences."""
    # The upstream basis builder constructs CPU tensors; evaluate there before
    # moving the resulting matrix to the prediction device.
    s = model._time_points.detach().cpu().double()
    eps = 1e-4
    plus, _, _ = model._build_legendre_basis((s + eps).double(), model.poly_order)
    minus, _, _ = model._build_legendre_basis((s - eps).double(), model.poly_order)
    coeff = prediction["coefficients"].double() * model._coeff_scale.double()
    scale = model.normalizer["action"].params_dict["scale"].double()
    numeric = torch.einsum("tc,bcd->btd", (plus - minus).to(coeff.device) / (2 * eps), coeff) / scale / duration
    analytic = prediction["velocity"].double() / duration
    if not torch.allclose(numeric, analytic, atol=5e-3, rtol=3e-3):
        raise RuntimeError(f"物理时间速度校验失败，max_abs_error={(numeric-analytic).abs().max().item()}")
    return (numeric - analytic).abs().max().item()


def save_checkpoint(path, model, ema, optimizer, scheduler, step, config, best):
    payload = {"format": "flash_fr3_checkpoint_v1", "step": step, "config": config,
               "model": model.state_dict(), "ema_model": ema.averaged_model.state_dict(),
               "optimizer": optimizer.state_dict(), "scheduler": scheduler.state_dict(),
               "ema_step": ema.optimization_step, "best_val_joint_mae_rad": best,
               "rng_torch": torch.get_rng_state(), "rng_numpy": np.random.get_state(),
               "rng_python": random.getstate(),
               "rng_cuda": torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None}
    temporary = path.with_suffix(".tmp")
    torch.save(payload, temporary)
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flash-root", type=Path, required=True)
    parser.add_argument("--dataset-cache", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--steps", type=int, default=15000)
    parser.add_argument("--schedule-steps", type=int, default=15000,
                        help="Cosine horizon; 15000 preserves the published recipe")
    parser.add_argument("--resume", type=Path,
                        help="Continue this dataset/policy's model, EMA and AdamW state in a new output directory")
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--val-every", type=int, default=250)
    parser.add_argument("--val-batches", type=int, default=1000000)
    parser.add_argument("--checkpoint-every", type=int, default=1250)
    parser.add_argument("--cpu-threads", type=int, default=4)
    parser.add_argument("--smoke-test", action="store_true", help="Two updates plus checkpoint reload; never deploy this output")
    args = parser.parse_args()
    if min(args.steps, args.batch_size, args.val_every, args.val_batches, args.checkpoint_every, args.cpu_threads) < 1:
        parser.error("步数、batch、检查间隔和线程数必须为正整数")
    if args.workers < 0:
        parser.error("workers 不能为负")
    if args.schedule_steps <= 500 or args.steps > args.schedule_steps:
        parser.error("余弦日程必须超过500步，且覆盖目标总步数")
    if args.smoke_test and args.resume is not None:
        parser.error("smoke-test 不用于续训；请直接指定正式目标总步数")
    if args.output.exists():
        parser.error("输出目录已存在，请使用新的目录，防止覆盖检查点")
    torch.set_num_threads(args.cpu_threads)
    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("当前 Python 环境没有可用 CUDA，请检查激活的环境和 GPU")
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    # Do not perturb other GPUs on this host. CUDA_VISIBLE_DEVICES selects the one to use.
    device = torch.device(args.device)
    if device.type == "cuda":
        torch.cuda.set_device(device)
    root = args.flash_root.resolve()
    source = source_info(root)
    sys.path.insert(0, str(root))
    from roboverse_learn.il.utils.ema_model import EMAModel
    manifest, cache_id = verify_cache(args.dataset_cache)
    config_policy = policy_config(root, manifest)
    train = FlashFR3Dataset(args.dataset_cache, "train")
    val = FlashFR3Dataset(args.dataset_cache, "val")
    model = instantiate(OmegaConf.create(config_policy))
    model.set_normalizer(train.get_normalizer())
    model.to(device)
    ema = EMAModel(copy.deepcopy(model), power=0.75)
    optimizer = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad],
                                 lr=1e-4, betas=(0.95, 0.999), eps=1e-8, weight_decay=1e-6)
    schedule_steps = args.schedule_steps
    start_step, best, resume_info = 0, float("inf"), None
    if args.resume is not None:
        checkpoint = args.resume.resolve()
        payload = torch.load(checkpoint, map_location="cpu", weights_only=False, mmap=True)
        previous = payload["config"]
        if (payload.get("format") != "flash_fr3_checkpoint_v1" or
                previous.get("checkpoint_purpose") != "offline_training_requires_real_robot_validation" or
                previous["source"] != source or previous["cache_manifest_sha256"] != cache_id or
                previous["policy"] != config_policy):
            raise ValueError("续训检查点的源码、数据或策略配置与当前训练不一致")
        if previous["arguments"]["batch_size"] != args.batch_size or previous["arguments"]["seed"] != args.seed:
            raise ValueError("续训沿用检查点的 batch-size 和 seed")
        start_step = int(payload["step"])
        if not 0 < start_step < args.steps:
            raise ValueError("续训目标总步数必须大于检查点步数")
        model.load_state_dict(payload["model"], strict=True)
        ema.averaged_model.load_state_dict(payload["ema_model"], strict=True)
        # Upstream DictOfTensorMixin recreates normalizer ParameterDicts on
        # the checkpoint's CPU device during load_state_dict. Move again after
        # loading, as the existing inference loader does.
        model.to(device)
        ema.averaged_model.to(device)
        ema.optimization_step = int(payload["ema_step"])
        optimizer.load_state_dict(payload["optimizer"])
        best = float(payload["best_val_joint_mae_rad"])
        torch.set_rng_state(payload["rng_torch"])
        np.random.set_state(payload["rng_numpy"])
        random.setstate(payload["rng_python"])
        if device.type == "cuda" and payload["rng_cuda"] is not None:
            torch.cuda.set_rng_state_all(payload["rng_cuda"])
        digest = hashlib.sha256()
        with checkpoint.open("rb") as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(block)
        resume_info = {"checkpoint": str(checkpoint), "sha256": digest.hexdigest(),
                       "step": start_step, "previous_scheduler": previous["scheduler"],
                       "schedule_changed": previous["scheduler"]["total_steps"] != schedule_steps,
                       "sampler": "new shuffled epoch; previous checkpoint has no sampler cursor",
                       "model_ema_optimizer_rng_restored": True}
        del payload
    # Reconstruct the cosine LR at the absolute optimizer step. A completed
    # 15k scheduler must not leave extended training at its saved zero LR.
    scheduler = torch.optim.lr_scheduler.LambdaLR(
        optimizer, lambda step: cosine_lr_factor(step, schedule_steps), last_epoch=start_step - 1)
    loader_args = dict(batch_size=args.batch_size, num_workers=args.workers, pin_memory=device.type == "cuda")
    loader = DataLoader(train, shuffle=True, **loader_args)
    val_loader = DataLoader(val, shuffle=False, **loader_args)
    times = physical_time_spec(model, manifest)
    config = {"source": source, "cache_manifest_sha256": cache_id, "policy": config_policy,
              "data": manifest, "physical_time": times, "arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
              "optimizer": {"lr": 1e-4, "betas": [0.95, 0.999], "weight_decay": 1e-6},
              "scheduler": {"name": "cosine", "warmup_steps": 500, "total_steps": schedule_steps},
              "resume": resume_info,
              "training_budget_scope": "published_15000_step_schedule" if schedule_steps == 15000
                                       else "additional_FR3_training_experiment_not_paper_reported_budget",
              "rgb_normalization": "upstream get_image_range_normalizer followed by upstream encoder imagenet_norm",
              "checkpoint_purpose": "smoke_test_only" if args.smoke_test else "offline_training_requires_real_robot_validation",
              "versions": {k: importlib.metadata.version(k) for k in ("torch", "torchvision", "numpy", "hydra-core")}}
    args.output.mkdir(parents=True)
    (args.output / "config.json").write_text(json.dumps(config, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"device": str(device), "train_windows": len(train), "val_windows": len(val),
                      "physical_time": times, "smoke_test": args.smoke_test,
                      "start_step": start_step, "target_step": args.steps,
                      "remaining_optimizer_updates": args.steps - start_step,
                      "initial_lr": optimizer.param_groups[0]["lr"], "resume": resume_info},
                     ensure_ascii=False), flush=True)
    iterator = iter(loader)
    steps = 2 if args.smoke_test else args.steps
    tick = time.monotonic()
    # A zero-output initialization can make some gradients zero on the first step;
    # verify finite, nonzero aggregate gradients and an actual parameter update.
    initial_head = None
    with (args.output / "metrics.jsonl").open("w", buffering=1) as log:
        for step in range(start_step + 1, steps + 1):
            try:
                batch = next(iterator)
            except StopIteration:
                iterator = iter(loader)
                batch = next(iterator)
            batch = move(batch, device)
            model.train()
            optimizer.zero_grad(set_to_none=True)
            loss = model.compute_loss(batch)
            if not torch.isfinite(loss):
                raise RuntimeError(f"step={step} loss 不是有限数")
            loss.backward()
            grads = [p.grad for p in model.parameters() if p.grad is not None]
            if not grads or not torch.stack([torch.isfinite(g).all() for g in grads]).all():
                raise RuntimeError("梯度缺失或包含非有限数")
            grad_norm = torch.linalg.vector_norm(torch.stack([g.norm() for g in grads])).item()
            if grad_norm == 0:
                raise RuntimeError("所有梯度均为零")
            if args.smoke_test and step == 2:
                changed_param = next(p for p in model.parameters() if p.grad is not None and p.grad.abs().max() > 0)
                initial_head = changed_param.detach().clone()
            used_lr = optimizer.param_groups[0]["lr"]
            optimizer.step()
            scheduler.step()
            ema.step(model)
            if args.smoke_test and step == 2 and torch.equal(initial_head, changed_param):
                raise RuntimeError("优化器没有更新参数")
            metrics = {"step": step, "loss": loss.item(), "grad_norm": grad_norm, "lr": used_lr,
                       "elapsed_seconds": time.monotonic() - tick,
                       "elapsed_optimizer_updates": step - start_step}
            final = step == steps
            if final or step % args.val_every == 0:
                metrics.update(validate(ema.averaged_model, val_loader, device,
                                        1 if args.smoke_test else args.val_batches))
            is_best = "val_joint_mae_rad" in metrics and metrics["val_joint_mae_rad"] < best
            if is_best:
                best = metrics["val_joint_mae_rad"]
            log.write(json.dumps(metrics) + "\n")
            if step <= start_step + 2 or final or step % 25 == 0:
                print(json.dumps(metrics), flush=True)
            if not args.smoke_test and (final or step % args.checkpoint_every == 0):
                # Keep step snapshots: the 10000-step checkpoint remains available.
                save_checkpoint(args.output / f"step_{step:06d}.pt", model, ema, optimizer, scheduler, step, config, best)
            if is_best and not args.smoke_test:
                save_checkpoint(args.output / "best.pt", model, ema, optimizer, scheduler, step, config, best)
    if args.smoke_test:
        checkpoint = args.output / "smoke_only.pt"
        save_checkpoint(checkpoint, model, ema, optimizer, scheduler, steps, config, best)
        payload = torch.load(checkpoint, map_location=device, weights_only=False)
        restored = instantiate(OmegaConf.create(config_policy)).to(device)
        restored.load_state_dict(payload["ema_model"])
        restored.eval()
        # Use a held-out real sequence, with no future images or future state.
        check_batch = move(next(iter(val_loader)), device)
        obs = inference_obs(check_batch, model.fit_pad, model.n_obs_steps)
        with torch.no_grad():
            before = ema.averaged_model.predict_action(obs)
            after = restored.predict_action(obs)
        if not torch.allclose(before["action"], after["action"], atol=1e-6, rtol=1e-6):
            raise RuntimeError("检查点重载预测不一致")
        derivative_error = check_derivative(restored, after, times["polynomial_duration_seconds"])
        result = {"status": "PASS", "optimizer_updates": steps, "checkpoint_reload": "PASS",
                  "physical_velocity_check_max_abs_error": derivative_error,
                  "prediction_shape": list(after["action"].shape), "device": str(device),
                  "robot_motion": False, "purpose": "pipeline verification, not a trained usable policy"}
        (args.output / "smoke_result.json").write_text(json.dumps(result, indent=2) + "\n")
        print("FLASH_TRAINING_SMOKE_TEST=PASS " + json.dumps(result), flush=True)
    else:
        print(f"FLASH_OFFLINE_TRAINING=COMPLETE steps={steps} best_val_joint_mae_rad={best}", flush=True)


if __name__ == "__main__":
    main()
