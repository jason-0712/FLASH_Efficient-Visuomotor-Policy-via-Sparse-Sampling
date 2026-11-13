#!/usr/bin/env python3
"""Serve FLASH predictions for observation only. No ROS or robot connection."""
from __future__ import annotations

import argparse
import io
import json
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import numpy as np
import torch

from infer_flash_fr3_observation import load_policy, predict_arrays, read_observation


def read_server_observation(path, allow_retimed):
    if hasattr(path, 'seek'):
        path.seek(0)
    with np.load(path, allow_pickle=False) as archive:
        observation_format = json.loads(str(archive['metadata_json'].item())).get('format')
    if hasattr(path, 'seek'):
        path.seek(0)
    if observation_format == 'flash_fr3_retimed_observation_v1':
        if not allow_retimed:
            raise ValueError('此服务未启用真实时间与模型时间分开的历史格式')
        from flash_fr3_retimed_schema import read_retimed_observation
        return read_retimed_observation(path)
    return read_observation(path)


def physical_curve(model, prediction, times):
    field = model.normalizer["action"].params_dict
    scale, offset = field["scale"], field["offset"]
    coefficients = (prediction["coefficients"] * model._coeff_scale)[0]
    coefficients = coefficients / scale
    coefficients = coefficients.clone()
    if model.basis_type != "legendre":
        raise ValueError("当前真机影子接口仅支持已验证的 Legendre 配置")
    coefficients[0] -= offset / scale
    return {"basis": model.basis_type, "coefficients": coefficients.detach().cpu().tolist(),
            "anchor_s": times["anchor_s"],
            "polynomial_duration_seconds": times["polynomial_duration_seconds"],
            "valid_future_seconds": times["action_chunk_interval_seconds"]}


def make_server(address, model, context, allowed_clients, allow_retimed=False):
    times = context["physical_time"]
    stats = model.normalizer["agent_pos"].get_input_stats()
    low, high = (stats[key].detach().cpu().numpy() for key in ("min", "max"))

    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, value):
            encoded = json.dumps(value, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)

        def allowed(self):
            if self.client_address[0] not in allowed_clients:
                self.reply(403, {"error": "client not allowed"})
                return False
            return True

        def do_GET(self):
            if not self.allowed():
                return
            if self.path != "/health":
                self.reply(404, {"error": "unknown endpoint"})
                return
            self.reply(200, {"status": "FLASH_SHADOW_SERVER_READY", "protocol": "flash_fr3_shadow_v1",
                            "checkpoint_step": context["checkpoint_step"], "weights": "ema_model",
                            "robot_motion_commanded": False, "physical_time": times,
                            "retimed_observations": allow_retimed})

        def do_POST(self):
            if not self.allowed():
                return
            if self.path != "/predict":
                self.reply(404, {"error": "unknown endpoint"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 4 * 1024 * 1024:
                    raise ValueError("请求大小不合法")
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise ValueError("请求未传完整")
                arrays, metadata = read_server_observation(io.BytesIO(raw), allow_retimed)
                start = time.perf_counter()
                prediction = predict_arrays(model, arrays)
                action = prediction["action"][0].detach().cpu().numpy()
                curve = physical_curve(model, prediction, times)
                if action.shape != (8, 8):
                    raise ValueError("预测形状异常")
                current = arrays["agent_pos"][-1]
                outside = (current < low - 1e-5) | (current > high + 1e-5)
                self.reply(200, {"status": "SHADOW_PREDICTION", "protocol": "flash_fr3_shadow_v1",
                                "request_id": metadata.get("request_id"),
                                "checkpoint_step": context["checkpoint_step"], "weights": "ema_model",
                                "robot_motion_commanded": False,
                                "observation_format": metadata['format'],
                                "history_clock": metadata.get('history_clock'),
                                "observation_target_stamp_seconds": metadata["samples"][-1]["target_stamp_seconds"],
                                "action": action.tolist(), "curve": curve,
                                "first_q_gap_deg": np.rad2deg(action[0, :7] - current[:7]).tolist(),
                                "state_outside_training_observed_range": np.flatnonzero(outside).tolist(),
                                "server_predict_and_decode_ms": (time.perf_counter() - start) * 1000})
            except (ValueError, KeyError, TypeError, OSError) as error:
                self.reply(400, {"error": str(error)})
            except Exception as error:
                self.reply(500, {"error": str(error)})

        def setup(self):
            super().setup()
            self.connection.settimeout(10)

        def log_message(self, *args):
            pass

    return HTTPServer(address, Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flash-root", type=Path, required=True)
    parser.add_argument("--dataset-cache", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--warmup-observation", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--bind", default="192.168.1.113")
    parser.add_argument("--port", type=int, default=10131)
    parser.add_argument("--allowed-client", default="192.168.1.107")
    parser.add_argument("--allow-retimed-observations", action="store_true")
    args = parser.parse_args()
    model, context = load_policy(args.flash_root, args.dataset_cache, args.checkpoint, args.device)
    arrays, _ = read_server_observation(args.warmup_observation, args.allow_retimed_observations)
    with torch.no_grad():
        for _ in range(3):
            predict_arrays(model, arrays)
    server = make_server((args.bind, args.port), model, context, {args.allowed_client, "127.0.0.1"},
                         allow_retimed=args.allow_retimed_observations)
    print("FLASH_SHADOW_SERVER_READY=" + json.dumps({"bind": args.bind, "port": server.server_port,
          "checkpoint_step": context["checkpoint_step"], "weights": "ema_model",
          "robot_motion_commanded": False, "retimed_observations": args.allow_retimed_observations}), flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
