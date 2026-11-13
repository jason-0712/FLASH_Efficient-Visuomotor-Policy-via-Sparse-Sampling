"""Decode physical FLASH curves for diagnostics; contains no robot IO."""
import numpy as np
from numpy.polynomial import legendre


def evaluate_curve(curve, seconds):
    if curve.get("basis") != "legendre":
        raise ValueError("只支持已验证的 Legendre 曲线")
    coefficients = np.asarray(curve["coefficients"], dtype=np.float64)
    duration = float(curve["polynomial_duration_seconds"])
    anchor = float(curve["anchor_s"])
    valid_future = float(curve["valid_future_seconds"])
    if coefficients.shape != (7, 8) or not np.isfinite(coefficients).all():
        raise ValueError("曲线系数异常")
    if not all(np.isfinite(v) for v in (duration, anchor, valid_future, seconds)) or duration <= 0:
        raise ValueError("曲线时间尺度异常")
    if not 0 <= seconds <= valid_future + 1e-8:
        raise ValueError("请求时刻超出本动作块范围")
    s = anchor + seconds / duration
    if not 0 <= s <= 1 + 1e-8:
        raise ValueError("请求时刻超出多项式定义范围")
    x = 2 * s - 1
    q = legendre.legval(x, coefficients)
    dq = legendre.legval(x, legendre.legder(coefficients, axis=0)) * 2 / duration
    ddq = legendre.legval(x, legendre.legder(coefficients, m=2, axis=0)) * 4 / duration**2
    return q, dq, ddq


def validate_reply(reply, request_id, target_stamp, checkpoint_step):
    if reply.get("protocol") != "flash_fr3_shadow_v1" or reply.get("status") != "SHADOW_PREDICTION":
        raise ValueError("推理响应格式异常")
    if reply.get("robot_motion_commanded") is not False or reply.get("weights") != "ema_model":
        raise ValueError("响应不符合只读 EMA 推理约定")
    if reply.get("request_id") != request_id or reply.get("checkpoint_step") != checkpoint_step:
        raise ValueError("响应请求编号或模型步数不一致")
    returned_stamp = float(reply["observation_target_stamp_seconds"])
    if not np.isfinite([returned_stamp, target_stamp]).all() or abs(returned_stamp - target_stamp) > 1e-6:
        raise ValueError("响应观测时间不一致")
    action = np.asarray(reply["action"], dtype=np.float64)
    if action.shape != (8, 8) or not np.isfinite(action).all():
        raise ValueError("动作数组异常")
    curve = reply["curve"]
    for key, expected in (("anchor_s", 2 / 11), ("polynomial_duration_seconds", 44 / 15),
                          ("valid_future_seconds", 32 / 15)):
        actual = float(curve[key])
        if not np.isfinite(actual) or abs(actual - expected) > 1e-7:
            raise ValueError("响应曲线与已训练模型的物理时间尺度不一致")
    for i in range(8):
        q, _, _ = evaluate_curve(curve, i * 4 / 15)
        if not np.allclose(q, action[i], atol=5e-5, rtol=1e-5):
            raise ValueError("连续曲线与动作数组不一致")
    return action
