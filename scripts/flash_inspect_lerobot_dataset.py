#!/usr/bin/env python3
"""Inspect CRISP LeRobot v2 demonstrations before adapting them to FLASH.

Reads dataset files only. Video checks inspect container metadata and decode
the first frame; they do not establish task success or camera synchronization.
Requires numpy, pyarrow, and (with --check-videos) av. No ROS or simulator.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def column(table, key: str, width: int | None = None) -> np.ndarray:
    if key not in table.column_names:
        raise ValueError(f"缺少字段 {key}")
    values = np.asarray(table[key].to_pylist(), dtype=np.float64)
    if width == 1 and values.ndim == 1:
        values = values[:, None]
    if width is not None and values.shape != (len(table), width):
        raise ValueError(f"{key} 的形状为 {values.shape}，预期 {(len(table), width)}")
    if not np.isfinite(values).all():
        raise ValueError(f"{key} 存在 NaN 或无穷值")
    return values


def inspect_dataset(root: Path, check_videos: bool) -> dict:
    info = read_json(root / "meta/info.json")
    episodes = [
        json.loads(line)
        for line in (root / "meta/episodes.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    fps = float(info["fps"])
    if not np.isfinite(fps) or fps <= 0:
        raise ValueError("数据集 fps 必须为正数")
    action_dim = int(info["features"]["action"]["shape"][0])
    video_keys = [k for k, v in info["features"].items() if v["dtype"] == "video"]
    report = {
        "dataset": str(root.resolve()),
        "codebase_version": info.get("codebase_version"),
        "declared_fps": fps,
        "original_action_dimension": action_dim,
        "original_action_names": info["features"]["action"].get("names"),
        "errors": [],
        "notes": [],
        "episodes": [],
        "video_check": "container_metadata_and_first_decoded_frame" if check_videos else "not_requested",
    }
    errors = report["errors"]
    ids = [int(ep["episode_index"]) for ep in episodes]
    if len(ids) != len(set(ids)):
        errors.append("episodes.jsonl 存在重复 episode_index")
    if len(episodes) != int(info["total_episodes"]):
        errors.append("示教条数与 info.json 不一致")
    if len(list((root / "data").rglob("*.parquet"))) != len(episodes):
        errors.append("Parquet 文件数量与示教条数不一致")
    if not video_keys:
        errors.append("没有图像视频字段")

    manifest_path = root / "meta/merge_manifest.json"
    source_by_id = {}
    new_source_ids = set()
    if manifest_path.exists():
        manifest = read_json(manifest_path)
        source_by_id = {int(ep["episode_index"]): ep for ep in manifest.get("episodes", [])}
        new_source_ids = set(manifest.get("new_wrist_camera_source_ids", []))
        report["source_groups"] = {
            "base_episode_count": manifest.get("base_episode_count"),
            "new_wrist_camera_episode_count": manifest.get("new_wrist_camera_episode_count"),
        }
    for key in video_keys:
        legacy_fps = info["features"][key].get("video_info", {}).get("video.fps")
        if legacy_fps is not None and not np.isclose(float(legacy_fps), fps):
            report["notes"].append(f"{key} 的旧 video_info 帧率为 {legacy_fps}；需以视频及时间列另行核对")

    if check_videos:
        import av

    chunks_size = int(info.get("chunks_size", 1000))
    total_rows = 0
    all_joints = []
    all_grippers = []
    all_action_grippers = []
    all_dt = []
    all_speed = []
    checked_videos = 0
    for ep in sorted(episodes, key=lambda item: int(item["episode_index"])):
        index = int(ep["episode_index"])
        chunk = index // chunks_size
        entry = {"episode_index": index}
        source = source_by_id.get(index, {})
        if source:
            entry["source_id"] = source.get("source_id")
            entry["source_group"] = "new_wrist_camera" if source.get("source_id") in new_source_ids else "base"
        try:
            parquet = root / info["data_path"].format(episode_chunk=chunk, episode_index=index)
            table = pq.read_table(parquet)
            size = len(table)
            if size != int(ep["length"]) or size < 2:
                raise ValueError(f"帧数 {size} 与元数据不符或少于两帧")
            if not np.array_equal(column(table, "frame_index").reshape(-1), np.arange(size)):
                raise ValueError("frame_index 不是从零开始的连续序列")
            if not np.all(column(table, "episode_index") == index):
                raise ValueError("表内 episode_index 与文件不符")
            joints = column(table, "observation.state.joints", 7)
            gripper = column(table, "observation.state.gripper", 1)
            action = column(table, "action", action_dim)
            timestamp = column(table, "timestamp").reshape(-1)
            dt = np.diff(timestamp)
            if not np.all(dt > 0):
                raise ValueError("时间戳未严格递增")
            if np.any(gripper < -1e-4) or np.any(gripper > 1.0001):
                raise ValueError("夹爪状态超出归一化的 [0,1] 范围")
            if "observation.state" in table.column_names:
                state_names = info["features"]["observation.state"].get("names") or []
                joint_names = info["features"]["observation.state.joints"].get("names") or []
                if len(joint_names) == 7 and all(name in state_names for name in joint_names):
                    state = column(table, "observation.state", len(state_names))
                    selected = state[:, [state_names.index(name) for name in joint_names]]
                    if not np.allclose(selected, joints, rtol=0, atol=1e-5):
                        raise ValueError("合并状态与单独的关节状态字段不一致")
            speed = np.abs(np.diff(joints, axis=0) / dt[:, None])
            entry.update({
                "frames": size,
                "duration_from_timestamp_s": float(timestamp[-1] - timestamp[0]),
                "median_dt_s": float(np.median(dt)),
                "max_joint_step_rad": float(np.max(np.abs(np.diff(joints, axis=0)))),
                "max_joint_speed_from_nominal_timestamps_rad_s": float(speed.max()),
                "max_timestamp_deviation_from_frame_index_over_fps_s": float(np.max(np.abs(timestamp - np.arange(size) / fps))),
            })
            if check_videos:
                entry["videos"] = {}
                for key in video_keys:
                    video = root / info["video_path"].format(episode_chunk=chunk, episode_index=index, video_key=key)
                    with av.open(str(video)) as container:
                        stream = container.streams.video[0]
                        if not stream.frames:
                            raise ValueError(f"{key} 未提供可核对的视频帧数")
                        if stream.frames != size:
                            raise ValueError(f"{key} 视频 {stream.frames} 帧，Parquet {size} 帧")
                        rate = float(stream.average_rate) if stream.average_rate else None
                        if rate is None or not np.isclose(rate, fps):
                            raise ValueError(f"{key} 视频帧率 {rate} 与数据集 {fps} 不符")
                        shape = info["features"][key]["shape"]
                        if [stream.height, stream.width, 3] != shape:
                            raise ValueError(f"{key} 视频尺寸与元数据不符")
                        first = next(container.decode(video=0))
                        if first.time is None or not np.isclose(first.time, timestamp[0], atol=1 / fps):
                            raise ValueError(f"{key} 视频首帧与表内起始时间不符")
                        entry["videos"][key] = {"frames": stream.frames, "fps": rate, "first_frame_decoded": True}
                        checked_videos += 1
            total_rows += size
            all_joints.append(joints)
            all_grippers.append(gripper)
            all_action_grippers.append(action[:, -1:])
            all_dt.append(dt)
            all_speed.append(speed)
        except Exception as exc:
            errors.append(f"episode_{index:06d}: {type(exc).__name__}: {exc}")
        report["episodes"].append(entry)

    if not errors and total_rows != int(info["total_frames"]):
        errors.append(f"总帧数 {total_rows} 与 info.json 的 {info['total_frames']} 不一致")
    report["summary"] = {
        "episode_count": len(episodes),
        "validated_frames": total_rows,
        "checked_videos": checked_videos,
        "error_count": len(errors),
    }
    if all_joints:
        joints = np.concatenate(all_joints)
        grippers = np.concatenate(all_grippers)
        action_grippers = np.concatenate(all_action_grippers)
        report["summary"].update({
            "joint_min_rad": joints.min(axis=0).tolist(),
            "joint_max_rad": joints.max(axis=0).tolist(),
            "gripper_state_range": [float(grippers.min()), float(grippers.max())],
            "gripper_original_command_range": [float(action_grippers.min()), float(action_grippers.max())],
            "median_dt_s": float(np.median(np.concatenate(all_dt))),
            "max_joint_speed_from_nominal_timestamps_rad_s": float(np.concatenate(all_speed).max()),
        })
    report["notes"].append("表内规则时间戳不能证明实际采集频率、传感器同步或无丢帧；当前采集代码未向 LeRobot 显式提交传感器采样时间")
    report["status"] = "PASS" if not errors else "FAIL"
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="检查用于 FLASH 适配的 LeRobot v2 真机示教，不发送机器人指令")
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--check-videos", action="store_true", help="核对全部视频的元数据并解码各自首帧")
    parser.add_argument("--output", type=Path, help="JSON 报告输出路径")
    args = parser.parse_args()
    report = inspect_dataset(args.dataset, args.check_videos)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"报告: {args.output}")
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))
    for message in report["errors"]:
        print(f"错误: {message}")
    for message in report["notes"]:
        print(f"备注: {message}")
    print(f"FLASH_DATA_CHECK={report['status']}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
