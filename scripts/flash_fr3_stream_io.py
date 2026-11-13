"""Read-only HTTP helpers for the short rolling trial."""
import io
import json
import time
import urllib.request

import numpy as np
from flash_fr3_shadow_math import validate_reply


def connection(server):
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(server+"/health",timeout=3) as response:
        health=json.load(response)
    if (health.get("checkpoint_step")!=15000 or health.get("weights")!="ema_model" or
            health.get("status")!="FLASH_SHADOW_SERVER_READY" or
            health.get("robot_motion_commanded") is not False):
        raise ValueError("需要第15000步 EMA 推理服务")
    return opener,health


def prediction(buffer,opener,server,request_id,handoff_monotonic=None,
               minimum_observation_monotonic=None):
    started=time.monotonic()
    # Send as early as the fixed sparse grid permits, rather than waiting an
    # arbitrary 0.15s into a prefix. Some grid phases already permit a request
    # at movement start; others need the next camera/state sample first.
    while True:
        arrays,metadata=buffer.observation(request_id)
        anchor=metadata["samples"][-1]["target_stamp_seconds"]
        observation_mono=time.monotonic()-(time.time()-anchor)
        wall,mono=time.time(),time.monotonic()
        newest=metadata['samples'][-1]['stamp_seconds']
        minimum_input=min(mono-(wall-newest[key]) for key in ('arm','primary_cam','wrist_cam'))
        age_ok=handoff_monotonic is None or handoff_monotonic-observation_mono<=.7
        fresh_ok=minimum_observation_monotonic is None or minimum_input>minimum_observation_monotonic
        if age_ok and fresh_ok:
            break
        if handoff_monotonic is not None and time.monotonic()>=handoff_monotonic-.04:
            raise ValueError('没有满足切换时龄的观测；保留已有停车段')
        time.sleep(.01)
    captured=time.monotonic()
    blob=io.BytesIO()
    np.savez(blob,**arrays,metadata_json=np.asarray(json.dumps(metadata)))
    encoded=blob.getvalue()
    rpc_start=time.monotonic()
    request=urllib.request.Request(server+"/predict",data=encoded,
                                   headers={"Content-Type":"application/octet-stream"})
    with opener.open(request,timeout=.8) as response:
        reply=json.load(response)
    validate_reply(reply,request_id,anchor,15000)
    if metadata.get('format')=='flash_fr3_retimed_observation_v1':
        if (reply.get('observation_format')!=metadata['format'] or
                reply.get('history_clock')!=metadata['history_clock']):
            raise RuntimeError('服务器未确认本次执行历史时钟；拒绝使用预测')
    return {"reply":reply,"observation_monotonic":observation_mono,
            "observation_target_stamp_seconds":anchor,"payload_bytes":len(encoded),
            "prepare_ms":(rpc_start-started)*1000,"rpc_ms":(time.monotonic()-rpc_start)*1000,
            "wait_and_capture_ms":(captured-started)*1000,
            "minimum_latest_arm_and_image_stamp_monotonic":minimum_input,
            "required_after_monotonic":minimum_observation_monotonic,
            "observation_format":metadata['format'],
            "history_clock":metadata.get('history_clock'),
            "observation_to_response_ms":(time.monotonic()-observation_mono)*1000}
