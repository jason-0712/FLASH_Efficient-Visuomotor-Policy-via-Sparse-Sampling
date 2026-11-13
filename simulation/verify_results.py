#!/usr/bin/env python3
"""Verify published experiment evidence with the Python standard library."""

import csv
import hashlib
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean


ROOT = Path(__file__).resolve().parent


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def read_csv(path):
    with path.open(encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def flag(value):
    require(value in (True, False, "True", "False"), f"Invalid boolean: {value!r}")
    return value is True or value == "True"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_files():
    listed = set()
    for line in (ROOT / "SHA256SUMS").read_text(encoding="utf-8").splitlines():
        expected, relative = line.split("  ", 1)
        path = ROOT / relative
        require(path.is_file(), f"Missing published file: {relative}")
        require(sha256(path) == expected, f"Checksum mismatch: {relative}")
        listed.add(relative)
    actual = {
        str(path.relative_to(ROOT)) for path in ROOT.rglob("*")
        if path.is_file() and path.name != ".DS_Store"
        and "__pycache__" not in path.parts and path != ROOT / "SHA256SUMS"
    }
    require(actual == listed, "Published checksum inventory differs from directory")
    manifest = read_json(ROOT / "export-manifest.json")
    for entry in manifest["files"]:
        require(sha256(ROOT / entry["path"]) == entry["export_sha256"],
                f"Export manifest mismatch: {entry['path']}")
    print(f"PASS: {len(listed)} published file checksums and export provenance")


def verify_curve(package, csv_name, success_key, id_key, heldout=False):
    directory = ROOT / package
    rows = read_csv(directory / "results" / csv_name)
    require(len(rows) == 400, f"{package}: expected 400 formal rollouts")
    groups = defaultdict(dict)
    for row in rows:
        group = groups[(int(row["checkpoint_step"]), row["policy"])]
        episode_id = int(row[id_key])
        require(episode_id not in group, f"Duplicate episode: {package}, {episode_id}")
        group[episode_id] = row
    summary = read_json(directory / "results/convergence.json")
    entries = ([policy for point in summary["curve_points"]
                for policy in point["policies"].values()] if heldout else summary)
    expected_keys = {(step, policy) for step in (2500, 5000, 7500, 10000)
                     for policy in ("flash", "fm_dit")}
    require(set(groups) == expected_keys, f"{package}: unexpected curve points")
    for entry in entries:
        step, policy = entry["checkpoint_step"], entry["policy"]
        group = groups[(step, policy)]
        require(set(group) == set(range(50)), f"{package}: incomplete test IDs")
        successes = sum(flag(row[success_key]) for row in group.values())
        require(successes == entry["successes"] and entry["rollouts"] == 50,
                f"{package}: summary counts differ at {step}/{policy}")
        require(math.isclose(successes / 50, entry["success_rate"]),
                f"{package}: success rate differs at {step}/{policy}")
        require(all(row["checkpoint_sha256"] == entry["checkpoint_sha256"]
                    for row in group.values()), f"{package}: checkpoint mismatch")
        if heldout:
            raw_path = (directory / f"runs/{step}/{policy}/rollouts.jsonl" if step != 10000
                        else ROOT / f"flash-heldout-evaluation/runs/{policy}/rollouts.jsonl")
        else:
            raw_path = directory / f"results/official/{policy}-{step}/rollouts.jsonl"
        raw_rows = [json.loads(line) for line in raw_path.read_text().splitlines() if line]
        require(len(raw_rows) == 50, f"{raw_path}: expected 50 raw records")
        require({int(row[id_key]) for row in raw_rows} == set(range(50)),
                f"{raw_path}: duplicate or missing raw IDs")
        for raw in raw_rows:
            row = group[int(raw[id_key])]
            for key in ("initial_state_sha256", "checkpoint_sha256"):
                require(row[key] == raw[key], f"{raw_path}: raw {key} mismatch")
            require(flag(row[success_key]) == flag(raw[success_key]),
                    f"{raw_path}: raw outcome mismatch")
    for step in (2500, 5000, 7500, 10000):
        for episode_id in range(50):
            left = groups[(step, "flash")][episode_id]
            right = groups[(step, "fm_dit")][episode_id]
            require(left["initial_state_sha256"] == right["initial_state_sha256"],
                    f"{package}: unpaired initial state")
    if heldout:
        frozen = directory / "inputs/test-initials.json"
        require(sha256(frozen) == summary["test_initials_sha256"], "Frozen input hash mismatch")
        states = {entry["test_id"]: entry["state_sha256"]
                  for entry in read_json(frozen)["states"]}
        for group in groups.values():
            for episode_id, row in group.items():
                require(row["initial_state_sha256"] == states[episode_id],
                        "Rollout differs from frozen input")
                require(row["physical_gpu_index"] == "2", "Formal curve includes another GPU")
                require(flag(row["reset_matches_frozen_cpu_trace"]), "Reset check failed")
    print(f"PASS: {package}, 400 formal rollouts, raw records, counts and paired initial states")


def verify_stability_and_latency():
    directory = ROOT / "flash-comparison/results"
    rows = read_csv(directory / "stability-rollouts.csv")
    require(len(rows) == 100, "Expected 100 stability rollouts")
    summary = read_json(directory / "stability-summary.json")
    for policy in ("flash", "fm_dit"):
        group = [row for row in rows if row["policy"] == policy]
        require(len(group) == 50, f"{policy}: expected 50 stability rollouts")
        for key, summary_key in (
            ("stable_placement_success", "stable_placement_successes"),
            ("geometric_success_once", "continued_geometric_successes"),
            ("released_at_goal_once", "released_at_goal_once_count"),
        ):
            require(sum(flag(row[key]) for row in group) == summary[policy][summary_key],
                    f"{policy}: stability summary mismatch for {key}")
    print("PASS: 100 stability records agree with all three outcome summaries")
    calls = read_csv(directory / "latency/calls.csv")
    timing = read_json(directory / "latency/summary.json")
    require(len(calls) == 12800, "Expected 12,800 timing samples")
    for policy in ("flash", "fm_dit"):
        group = [row for row in calls if row["policy"] == policy]
        for category, subset in (
            ("all_control_calls", group),
            ("actual_inference_calls", [r for r in group if flag(r["inference"])]),
            ("cache_calls", [r for r in group if not flag(r["inference"])]),
        ):
            values = [float(row["elapsed_ms"]) for row in subset]
            require(all(math.isfinite(value) and value >= 0 for value in values),
                    "Invalid timing sample")
            expected = timing["policies"][policy][category]
            require(len(values) == expected["n"], f"{policy}/{category}: count mismatch")
            require(math.isclose(mean(values), expected["mean_ms"], abs_tol=1e-9),
                    f"{policy}/{category}: mean mismatch")
    print("PASS: 12,800 timing samples reproduce all reported aggregate means")


def main():
    verify_files()
    verify_curve("flash-heldout-convergence", "all-rollouts.csv",
                 "geometric_success_once", "test_id", heldout=True)
    verify_curve("flash-comparison", "all-official-rollouts.csv", "success", "rollout_id")
    verify_stability_and_latency()
    print("PASS: published evidence verified; no model or simulator was executed")


if __name__ == "__main__":
    main()
