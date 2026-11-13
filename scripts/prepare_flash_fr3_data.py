#!/usr/bin/env python3
"""Prepare audited CRISP LeRobot v2 data for offline FLASH training only."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import tempfile
from pathlib import Path

import av
import numpy as np
import pyarrow.parquet as pq

from flash_inspect_lerobot_dataset import column, inspect_dataset, read_json

CAMERAS = {"primary_cam": "observation.images.primary", "wrist_cam": "observation.images.wrist"}


def split_episodes(episodes, seed=42, val_fraction=0.1):
    """Stratify by acquisition batch; never split frames from an episode."""
    rng = np.random.default_rng(seed)
    groups = sorted({ep["group"] for ep in episodes})
    train, val = [], []
    for group in groups:
        ids = np.array([ep["episode_index"] for ep in episodes if ep["group"] == group])
        if len(ids) < 2:
            raise ValueError(f"分组 {group} 至少需要两条示教")
        ids = rng.permutation(ids)
        count = min(len(ids) - 1, max(1, math.ceil(len(ids) * val_fraction)))
        val.extend(ids[:count].tolist())
        train.extend(ids[count:].tolist())
    return {"train": sorted(train), "val": sorted(val)}


def supervision(table):
    """CRISP observation gripper=closure, original action gripper=openness.

    The joint target is reconstructed from measured demonstration joints, NOT
    from the original 6D Cartesian delta action. It is an adaptation to FR3.
    """
    q = column(table, "observation.state.joints", 7).astype(np.float32)
    closure = column(table, "observation.state.gripper", 1).astype(np.float32)
    original = column(table, "action", 7).astype(np.float32)
    return np.concatenate([q, closure], -1), np.concatenate([q, 1 - original[:, -1:]], -1)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def prepare(root: Path, output: Path, stride: int, seed: int):
    if stride < 1:
        raise ValueError("stride 必须为正整数")
    if output.exists():
        raise FileExistsError(f"输出已存在，请换一个目录：{output}")
    report = inspect_dataset(root, check_videos=False)
    if report["errors"]:
        raise ValueError(json.dumps(report["errors"], ensure_ascii=False))
    info = read_json(root / "meta/info.json")
    fps = float(info["fps"])
    source = read_json(root / "meta/merge_manifest.json")
    source_eps = {int(ep["episode_index"]): ep for ep in source["episodes"]}
    new_ids = set(source["new_wrist_camera_source_ids"])
    original_eps = sorted(
        [json.loads(line) for line in (root / "meta/episodes.jsonl").read_text().splitlines() if line.strip()],
        key=lambda ep: int(ep["episode_index"]),
    )
    episodes, offset = [], 0
    for ep in original_eps:
        idx, length = int(ep["episode_index"]), int(ep["length"])
        src = source_eps[idx]
        count = len(range(0, length, stride))
        episodes.append({"episode_index": idx, "source_id": src["source_id"],
                         "group": "new_wrist_batch" if src["source_id"] in new_ids else "base_batch",
                         "start": offset, "end": offset + count, "raw_length": length})
        offset += count
    height, width = info["features"][next(iter(CAMERAS.values()))]["shape"][:2]
    output.parent.mkdir(parents=True, exist_ok=True)
    building = Path(tempfile.mkdtemp(prefix=output.name + ".building-", dir=output.parent))
    arrays = {}
    try:
        def create(name, dtype, shape):
            arrays[name] = np.lib.format.open_memmap(building / f"{name}.npy", mode="w+", dtype=dtype, shape=shape)
        create("state", "float32", (offset, 8))
        create("action", "float32", (offset, 8))
        create("raw_frame_index", "int64", (offset,))
        create("timestamp", "float64", (offset,))
        for name in CAMERAS:
            create(name, "uint8", (offset, height, width, 3))
        source_files = {}
        for meta in ("info.json", "episodes.jsonl", "merge_manifest.json"):
            path = root / "meta" / meta
            source_files[str(path.relative_to(root))] = sha256_file(path)
        max_pts_error = 0.0
        for ep in episodes:
            idx, length = ep["episode_index"], ep["raw_length"]
            sl = slice(ep["start"], ep["end"])
            chunk = idx // int(info.get("chunks_size", 1000))
            fmt = {"episode_chunk": chunk, "episode_index": idx}
            parquet = root / info["data_path"].format(**fmt)
            table = pq.read_table(parquet)
            state, action = supervision(table)
            times = column(table, "timestamp")
            if not np.allclose(times - times[0], np.arange(length) / fps, atol=1e-4):
                raise ValueError(f"episode {idx} 时间列不符合固定 fps，不能直接使用固定时间尺度")
            arrays["state"][sl] = state[::stride]
            arrays["action"][sl] = action[::stride]
            arrays["raw_frame_index"][sl] = np.arange(0, length, stride)
            arrays["timestamp"][sl] = times[::stride]
            source_files[str(parquet.relative_to(root))] = sha256_file(parquet)
            for name, video_key in CAMERAS.items():
                video = root / info["video_path"].format(video_key=video_key, **fmt)
                count, pts0 = 0, None
                with av.open(str(video)) as container:
                    for j, frame in enumerate(container.decode(video=0)):
                        if j >= length or frame.pts is None:
                            raise ValueError(f"{video}: 帧数或 PTS 异常")
                        pts = float(frame.pts * frame.time_base)
                        if pts0 is None:
                            pts0 = pts
                        error = abs(pts - pts0 - float(times[j] - times[0]))
                        max_pts_error = max(max_pts_error, error)
                        if error > 1e-4:
                            raise ValueError(f"{video}: PTS 与标称时间列不匹配，frame={j}, error={error}")
                        if j % stride == 0:
                            rgb = frame.to_ndarray(format="rgb24")
                            if rgb.shape != (height, width, 3):
                                raise ValueError(f"{video}: 图像大小发生变化 {rgb.shape}")
                            arrays[name][ep["start"] + j // stride] = rgb
                        count += 1
                if count != length:
                    raise ValueError(f"{video}: 完整解码 {count} 帧，预期 {length}")
                source_files[str(video.relative_to(root))] = sha256_file(video)
            print(f"episode={idx:02d} raw={length} sparse={ep['end']-ep['start']} OK", flush=True)
        for array in arrays.values():
            array.flush()
        splits = split_episodes(episodes, seed)
        manifest = {
            "format": "flash_fr3_memmap_v1", "source_dataset": str(root.resolve()),
            "source_fps": fps, "stride": stride, "sample_dt_seconds": stride / fps,
            "image_shape_hwc": [height, width, 3], "total_samples": offset,
            "episodes": episodes, "splits": splits, "split_seed": seed,
            "state_schema": ["q1", "q2", "q3", "q4", "q5", "q6", "q7", "measured_gripper_closure"],
            "action_schema": ["measured_q1", "measured_q2", "measured_q3", "measured_q4", "measured_q5", "measured_q6", "measured_q7", "commanded_gripper_closure"],
            "gripper_convention": "0=open, 1=closed; command=1-original_action[-1]",
            "joint_supervision": "measured demonstration joints; original Cartesian deltas are not joint commands",
            "time_basis": "nominal timestamps and video PTS; sensor capture synchronization is not recorded",
            "max_video_pts_error_seconds": max_pts_error, "source_sha256": source_files,
        }
        # Hash the actual portable cache too: training validates its bytes, not just metadata.
        manifest["cache_sha256"] = {p.name: sha256_file(p) for p in sorted(building.glob("*.npy"))}
        (building / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
        (building / "source_audit.json").write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
        building.rename(output)
        print(f"FLASH_CACHE_CHECK=PASS samples={offset} train_episodes={len(splits['train'])} val_episodes={len(splits['val'])} output={output}")
    except BaseException:
        arrays.clear()
        shutil.rmtree(building, ignore_errors=True)
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stride", type=int, default=4)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    prepare(args.dataset.resolve(), args.output.resolve(), args.stride, args.seed)
