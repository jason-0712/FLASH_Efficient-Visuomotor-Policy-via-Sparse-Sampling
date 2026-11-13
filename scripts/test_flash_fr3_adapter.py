#!/usr/bin/env python3
"""Offline regression checks for units, episode boundaries and leakage."""
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pyarrow as pa
import torch
from torch.utils.data import DataLoader

from flash_fr3_dataset import FlashFR3Dataset, inference_obs, physical_time_spec, window_indices
from prepare_flash_fr3_data import split_episodes, supervision


class AdapterTests(unittest.TestCase):
    def test_all_windows_stay_in_episode_and_observations_are_past(self):
        for start, end in ((0, 3), (3, 19), (19, 45)):
            for current in range(start, end):
                indices = window_indices(current, start, end)
                self.assertEqual(len(indices), 18)
                self.assertTrue(((indices >= start) & (indices < end)).all())
                self.assertTrue((indices[:9] <= current).all())
                self.assertEqual(indices[8], current)
                self.assertEqual(indices[9], min(current + 1, end - 1))

    def test_gripper_polarity_and_joint_targets(self):
        q = np.arange(21, dtype=np.float32).reshape(3, 7)
        table = pa.table({"observation.state.joints": q.tolist(),
                          "observation.state.gripper": [0.0, 0.4, 0.6],
                          "action": [[99.0] * 6 + [g] for g in [1.0, 0.0, 0.0]]})
        state, action = supervision(table)
        np.testing.assert_array_equal(action[:, :7], q)
        np.testing.assert_allclose(state[:, 7], [0, 0.4, 0.6])
        np.testing.assert_array_equal(action[:, 7], [0, 1, 1])

    def test_stratified_split_is_disjoint_reproducible(self):
        eps = [{"episode_index": i, "group": "base" if i < 50 else "new"} for i in range(74)]
        split = split_episodes(eps)
        self.assertEqual(split, split_episodes(eps))
        self.assertEqual(len(split["train"]), 66)
        self.assertEqual(len(split["val"]), 8)
        self.assertFalse(set(split["train"]) & set(split["val"]))
        self.assertEqual(sum(i < 50 for i in split["val"]), 5)
        self.assertEqual(sum(i >= 50 for i in split["val"]), 3)

    def test_rgb_alignment_and_no_future_state(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            state = np.tile(np.arange(10, dtype=np.float32)[:, None], (1, 8))
            np.save(root / "state.npy", state)
            np.save(root / "action.npy", state)
            rgb = np.broadcast_to(np.arange(10, dtype=np.uint8)[:, None, None, None], (10, 2, 2, 3)).copy()
            for key in ("primary_cam", "wrist_cam"):
                np.save(root / f"{key}.npy", rgb)
            (root / "manifest.json").write_text(json.dumps({"splits": {"train": [0], "val": [1]},
                "episodes": [{"episode_index": 0, "start": 0, "end": 5},
                             {"episode_index": 1, "start": 5, "end": 10}]}))
            data = FlashFR3Dataset(root, "val", n_obs=2, horizon=4, fit_pad=1)
            batch = next(iter(DataLoader(data, batch_size=3)))
            obs = inference_obs(batch, fit_pad=1, n_obs=2)
            torch.testing.assert_close(obs["agent_pos"][:, :, 0], torch.tensor([[5., 5.], [5., 6.], [6., 7.]]))
            torch.testing.assert_close(obs["primary_cam"][:, :, 0, 0, 0] * 255, obs["agent_pos"][:, :, 0])
            self.assertEqual(tuple(batch["action"].shape), (3, 6, 8))
            self.assertTrue((batch["action"] >= 5).all())

    def test_physical_duration_includes_fit_padding(self):
        class Model:
            _H_poly, fit_pad, _anchor_idx, n_action_steps = 10, 1, 1, 8
        result = physical_time_spec(Model(), {"sample_dt_seconds": 4 / 15})
        self.assertAlmostEqual(result["polynomial_duration_seconds"], 44 / 15)
        self.assertAlmostEqual(result["action_chunk_interval_seconds"], 32 / 15)
        self.assertAlmostEqual(result["anchor_s"], 2 / 11)
        self.assertAlmostEqual(result["velocity_to_per_second"], 15 / 44)


if __name__ == "__main__":
    unittest.main()
