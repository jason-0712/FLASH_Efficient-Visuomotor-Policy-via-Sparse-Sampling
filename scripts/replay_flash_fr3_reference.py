#!/usr/bin/env python3
"""Replay recorded shadow predictions as numerical joint references, offline.

No robot dynamics, camera, networking or ROS is used. Recorded observations do
not change in response to the hypothetical reference. This is not a closed-loop
policy evaluation and does not establish task success or execution readiness.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from flash_fr3_reference import DelayedJointReference
from flash_fr3_shadow_math import evaluate_curve, validate_reply


def max_per_joint(values):
    return np.abs(np.asarray(values)).max(axis=0).tolist()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--shadow-run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--checkpoint-step", type=int, default=15000)
    parser.add_argument("--blend-seconds", type=float, default=1.2)
    parser.add_argument("--max-observation-age", type=float, default=.7)
    parser.add_argument("--sample-dt", type=float, default=.001)
    args = parser.parse_args()
    if not np.isfinite(args.sample_dt) or not 0 < args.sample_dt <= .01:
        parser.error("sample-dt 必须在 (0, .01] 秒内")
    predictions = args.shadow_run / "predictions.jsonl"
    source_report = json.loads((args.shadow_run / "report.json").read_text())
    if source_report.get("robot_motion_commanded") is not False:
        raise ValueError("输入必须是明确未发送动作的影子记录")
    if source_report.get("server", {}).get("checkpoint_step") != args.checkpoint_step:
        raise ValueError("影子报告模型步数不一致")
    rows = [json.loads(line) for line in predictions.read_text().splitlines() if line.strip()]
    valid = [r for r in rows if r["status"] == "PREDICTION"]
    if len(valid) < 2:
        raise ValueError("至少需要两次有效预测")
    if len(valid) != source_report["valid_predictions"]:
        raise ValueError("预测条数与影子报告不一致")
    origin = float(valid[0]["response"]["observation_target_stamp_seconds"])
    events, measured = [], []
    for row in valid:
        response = row["response"]
        stamp = float(response["observation_target_stamp_seconds"])
        action = validate_reply(response, row["request_id"], stamp, args.checkpoint_step)
        age = float(row["observation_to_response_ms"]) / 1000
        arrival = stamp - origin + age
        if events and (arrival <= events[-1]["arrival"] or stamp <= events[-1]["stamp"]
                       or row["request_id"] <= events[-1]["request_id"]):
            raise ValueError("影子事件编号或时间没有严格递增")
        gap = np.asarray(response["first_q_gap_deg"], dtype=np.float64)
        if gap.shape != (7,) or not np.isfinite(gap).all():
            raise ValueError("无法重建观测关节角")
        measured.append(action[0, :7] - np.deg2rad(gap))
        events.append({"arrival": arrival, "stamp": stamp, "age": age,
                       "request_id": row["request_id"], "curve": response["curve"]})

    args.output.mkdir(parents=True, exist_ok=False)
    previous = None
    jumps, raw_jumps, latency_effect, states, sample_times, samples = [], [], [], [], [], []
    for i, event in enumerate(events):
        arrival, age, curve = event["arrival"], event["age"], event["curve"]
        initial = (np.stack([measured[0], np.zeros(7), np.zeros(7)]) if previous is None
                   else np.asarray(previous.evaluate(arrival - events[i - 1]["arrival"])))
        reference = DelayedJointReference(curve, age, initial, args.blend_seconds,
                                          args.max_observation_age)
        raw = np.asarray(evaluate_curve(curve, age))[:, :7]
        raw_jumps.append(raw - initial)
        jumps.append(np.asarray(reference.evaluate(0)) - initial)
        latency_effect.append(raw[0] - np.asarray(evaluate_curve(curve, 0))[0, :7])
        duration = (events[i + 1]["arrival"] - arrival if i + 1 < len(events)
                    else args.blend_seconds)
        # Fail instead of silently filling a missing/expired interval.
        reference.evaluate(duration)
        offsets = np.unique(np.append(np.arange(0, duration, args.sample_dt), duration))
        for offset in offsets:
            state = np.asarray(reference.evaluate_with_jerk(float(offset)))
            states.append(state)
            t = arrival - events[0]["arrival"] + offset
            sample_times.append(float(t))
            samples.append([float(t), event["request_id"], *state.reshape(-1).tolist()])
        previous = reference

    states = np.asarray(states)
    jumps, raw_jumps = np.asarray(jumps), np.asarray(raw_jumps)
    boundary_maxima = {name: max_per_joint(jumps[:, order]) for order, name in enumerate(
        ("position_rad", "velocity_rad_s", "acceleration_rad_s2"))}
    if np.max(np.abs(jumps)) > 1e-10:
        raise RuntimeError("数值边界连续性检查未通过")
    report = {
        "status": "OFFLINE_REFERENCE_REPLAY_COMPLETE", "robot_motion_commanded": False,
        "checkpoint_step": args.checkpoint_step, "weights": "ema_model",
        "source_predictions": str(predictions.resolve()),
        "source_predictions_sha256": hashlib.sha256(predictions.read_bytes()).hexdigest(),
        "predictions": len(events), "samples": len(samples),
        "replayed_seconds": max(sample_times),
        "parameters": {"blend_seconds": args.blend_seconds,
                       "max_observation_age_seconds": args.max_observation_age,
                       "sample_dt_seconds": args.sample_dt},
        "initial_assumption": "q reconstructed from first recorded input; dq=ddq=0 assumed, not measured",
        "final_tail_seconds": args.blend_seconds,
        "observed_q_peak_to_peak_rad": np.ptp(measured, axis=0).tolist(),
        "c2_boundary_check": {"status": "PASS", "max_abs_by_joint": boundary_maxima},
        "uncorrected_delayed_boundary_max_abs_by_joint": {
            name: max_per_joint(raw_jumps[:, order]) for order, name in enumerate(
                ("position_rad", "velocity_rad_s", "acceleration_rad_s2"))},
        "delay_compensation_position_change_max_abs_deg_by_joint":
            np.rad2deg(np.max(np.abs(latency_effect), axis=0)).tolist(),
        "sampled_reference": {
            "q_min_rad": states[:, 0].min(axis=0).tolist(),
            "q_max_rad": states[:, 0].max(axis=0).tolist(),
            "peak_abs_velocity_rad_s": max_per_joint(states[:, 1]),
            "peak_abs_acceleration_rad_s2": max_per_joint(states[:, 2]),
            "peak_abs_jerk_rad_s3": max_per_joint(states[:, 3]),
        },
        "scope": "numerical reference replay on recorded observations; no dynamics or feedback; not task success or hardware approval",
        "limitations": [
            "Sampled extrema only; C2 matching does not enforce limits or avoid collisions.",
            "Jerk may jump at replans and blend completion.",
            "Gripper excluded; no runtime watchdog, braking or measured tracking-error protection.",
            "Recorded stationary inputs do not reflect the hypothetical moving reference.",
            "A too-old or out-of-horizon evaluation raises; this is not a hardware stop mechanism.",
        ],
    }
    with (args.output / "reference.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["seconds_from_first_arrival", "request_id"]
                        + [f"{name}{j}" for name in ("q", "dq", "ddq", "jerk") for j in range(1, 8)])
        writer.writerows(samples)
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print("FLASH_OFFLINE_REFERENCE_REPLAY=" + json.dumps({
        "status": report["status"], "predictions": len(events),
        "c2_boundary_check": "PASS", "robot_motion_commanded": False,
        "report": str((args.output / "report.json").resolve()),
    }), flush=True)


if __name__ == "__main__":
    main()
