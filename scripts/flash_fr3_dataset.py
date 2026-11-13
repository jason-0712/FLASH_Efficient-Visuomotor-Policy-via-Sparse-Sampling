"""Episode-safe sparse windows for the unmodified upstream FLASH policy."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import Dataset

CAMERAS = ("primary_cam", "wrist_cam")


def window_indices(current, start, end, n_obs=8, horizon=16, fit_pad=1):
    # Full action window is [-8,...,+9] at the default settings. The policy
    # removes fit_pad from observations, leaving [-7,...,0], never future RGB.
    indices = current - (n_obs - 1) - fit_pad + np.arange(horizon + 2 * fit_pad)
    return np.clip(indices, start, end - 1)


def verify_cache(root):
    root = Path(root)
    raw = (root / "manifest.json").read_bytes()
    manifest = json.loads(raw)
    if manifest["format"] != "flash_fr3_memmap_v1":
        raise ValueError("不支持的缓存格式")
    for name, expected in manifest["cache_sha256"].items():
        digest = hashlib.sha256()
        with (root / name).open("rb") as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
                digest.update(block)
        if digest.hexdigest() != expected:
            raise ValueError(f"缓存校验失败：{name}")
    train, val = map(set, (manifest["splits"]["train"], manifest["splits"]["val"]))
    if train & val or train | val != {ep["episode_index"] for ep in manifest["episodes"]}:
        raise ValueError("训练/验证 episode 划分不完整或重叠")
    return manifest, hashlib.sha256(raw).hexdigest()


class FlashFR3Dataset(Dataset):
    def __init__(self, root, split, n_obs=8, horizon=16, fit_pad=1):
        self.root = Path(root)
        self.manifest = json.loads((self.root / "manifest.json").read_text())
        self.n_obs, self.horizon, self.fit_pad = n_obs, horizon, fit_pad
        selected = set(self.manifest["splits"][split])
        self.episodes = [ep for ep in self.manifest["episodes"] if ep["episode_index"] in selected]
        self.samples = np.array([(i, ep["start"], ep["end"]) for ep in self.episodes
                                 for i in range(ep["start"], ep["end"])], dtype=np.int64)
        self.arrays = None

    def _open(self):
        if self.arrays is None:
            self.arrays = {key: np.load(self.root / f"{key}.npy", mmap_mode="r")
                           for key in ("state", "action", *CAMERAS)}

    def __getstate__(self):
        state = self.__dict__.copy()
        state["arrays"] = None
        return state

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        self._open()
        current, start, end = self.samples[index]
        indices = window_indices(current, start, end, self.n_obs, self.horizon, self.fit_pad)
        past = indices[:self.fit_pad + self.n_obs]
        obs = {"agent_pos": torch.from_numpy(self.arrays["state"][past].copy())}
        for key in CAMERAS:
            obs[key] = torch.from_numpy(self.arrays[key][past].copy()).permute(0, 3, 1, 2).float().div_(255)
        return {"obs": obs, "action": torch.from_numpy(self.arrays["action"][indices].copy())}

    def get_normalizer(self):
        # Fit ONLY training episodes; held-out data must not determine ranges.
        from roboverse_learn.il.utils.normalizer import LinearNormalizer
        from roboverse_learn.il.utils.normalize_util import get_image_range_normalizer
        self._open()
        ids = self.samples[:, 0]
        normalizer = LinearNormalizer()
        normalizer.fit({"action": self.arrays["action"][ids].copy(),
                        "agent_pos": self.arrays["state"][ids].copy()}, mode="limits")
        # Preserve upstream RobotImageDataset's RGB normalization exactly.
        for key in CAMERAS:
            normalizer[key] = get_image_range_normalizer()
        return normalizer


def inference_obs(batch, fit_pad=1, n_obs=8):
    return {key: value[:, fit_pad:fit_pad + n_obs] for key, value in batch["obs"].items()}


def physical_time_spec(model, manifest):
    dt = manifest["sample_dt_seconds"]
    extended_points = model._H_poly + 2 * model.fit_pad
    return {"sample_dt_seconds": dt,
            "polynomial_duration_seconds": (extended_points - 1) * dt,
            "action_chunk_interval_seconds": model.n_action_steps * dt,
            "anchor_s": (model.fit_pad + model._anchor_idx) / (extended_points - 1),
            "velocity_to_per_second": 1 / ((extended_points - 1) * dt),
            "acceleration_to_per_second_squared": 1 / ((extended_points - 1) * dt) ** 2}
