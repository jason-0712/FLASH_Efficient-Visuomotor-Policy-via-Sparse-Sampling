#!/usr/bin/env python3
"""Check shadow transport and physical decoding against the upstream policy.

Uses an untrained in-memory model on CPU; this verifies code, not policy quality.
No ROS or robot IO is imported.
"""
import argparse
import copy
import io
import json
import sys
import threading
import unittest
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np
import torch
from hydra.utils import instantiate
from omegaconf import OmegaConf

from flash_fr3_dataset import FlashFR3Dataset, physical_time_spec
from flash_fr3_shadow_math import evaluate_curve, validate_reply
from infer_flash_fr3_observation import predict_arrays, read_observation
from serve_flash_fr3_shadow import make_server, physical_curve
from train_flash_fr3 import policy_config


class ShadowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        torch.set_num_threads(2)
        torch.manual_seed(42)
        sys.path.insert(0, str(ARGS.flash_root.resolve()))
        data = FlashFR3Dataset(ARGS.dataset_cache, "train")
        cls.model = instantiate(OmegaConf.create(policy_config(ARGS.flash_root, data.manifest)))
        cls.model.set_normalizer(data.get_normalizer())
        cls.model.eval().requires_grad_(False)
        cls.model.eval_history_noise_std = 0
        cls.arrays, cls.metadata = read_observation(ARGS.observation)
        cls.metadata["request_id"] = 17
        cls.times = physical_time_spec(cls.model, data.manifest)
        cls.prediction = predict_arrays(cls.model, cls.arrays)
        cls.curve = physical_curve(cls.model, cls.prediction, cls.times)

    def test_physical_curve_matches_upstream_positions_and_derivatives(self):
        duration = self.times["polynomial_duration_seconds"]
        dt = self.times["sample_dt_seconds"]
        anchor = self.model._anchor_idx
        for index in range(8):
            q, dq, ddq = evaluate_curve(self.curve, index * dt)
            for actual, key, factor in ((q, "action_pred", 1), (dq, "velocity", duration),
                                        (ddq, "acceleration", duration**2)):
                expected = self.prediction[key][0, anchor + index].numpy() / factor
                np.testing.assert_allclose(actual, expected, atol=3e-5, rtol=2e-4)
        t, eps = 0.2, 1e-4
        q, dq, ddq = evaluate_curve(self.curve, t)
        qplus, vplus, _ = evaluate_curve(self.curve, t + eps)
        qminus, vminus, _ = evaluate_curve(self.curve, t - eps)
        np.testing.assert_allclose(dq, (qplus - qminus) / (2 * eps), atol=1e-5, rtol=1e-4)
        np.testing.assert_allclose(ddq, (vplus - vminus) / (2 * eps), atol=1e-5, rtol=1e-4)
        for invalid in (-0.01, self.times["action_chunk_interval_seconds"] + 0.01):
            with self.assertRaises(ValueError):
                evaluate_curve(self.curve, invalid)

    def test_http_npz_and_replay_rejection(self):
        context = {"physical_time": self.times, "checkpoint_step": 0}
        server = make_server(("127.0.0.1", 0), self.model, context, {"127.0.0.1"})
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.05})
        thread.start()
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with opener.open(base + "/health", timeout=5) as response:
                self.assertIs(json.load(response)["robot_motion_commanded"], False)
            blob = io.BytesIO()
            np.savez(blob, **self.arrays, metadata_json=np.asarray(json.dumps(self.metadata)))
            with opener.open(urllib.request.Request(base + "/predict", data=blob.getvalue()), timeout=15) as response:
                reply = json.load(response)
            target_stamp = self.metadata["samples"][-1]["target_stamp_seconds"]
            action = validate_reply(reply, 17, target_stamp, 0)
            np.testing.assert_allclose(action, self.prediction["action"][0].numpy(), atol=1e-6)
            for invalid_id, invalid_time, invalid_step in ((18, target_stamp, 0), (17, target_stamp + 1, 0), (17, target_stamp, 1)):
                with self.assertRaises(ValueError):
                    validate_reply(reply, invalid_id, invalid_time, invalid_step)
            invalid = copy.deepcopy(self.arrays)
            invalid["agent_pos"][0, 0] = np.nan
            blob = io.BytesIO()
            np.savez(blob, **invalid, metadata_json=np.asarray(json.dumps(self.metadata)))
            with self.assertRaises(urllib.error.HTTPError) as error:
                opener.open(urllib.request.Request(base + "/predict", data=blob.getvalue()), timeout=5)
            self.assertEqual(error.exception.code, 400)
        finally:
            server.shutdown()
            thread.join(timeout=5)
            server.server_close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flash-root", type=Path, required=True)
    parser.add_argument("--dataset-cache", type=Path, required=True)
    parser.add_argument("--observation", type=Path, required=True)
    ARGS, test_args = parser.parse_known_args()
    unittest.main(argv=[sys.argv[0], *test_args])
