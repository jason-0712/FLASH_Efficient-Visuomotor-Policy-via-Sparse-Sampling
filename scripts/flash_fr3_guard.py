"""Reference admission and fault latching for READ-ONLY FLASH experiments.

No ROS, networking, command publisher, actuator saturation or hardware stop.
Sampled trajectory checks are diagnostics, not continuous-time certificates.
The application limits below are proposals for a small first experiment.
"""
from dataclasses import asdict, dataclass

import numpy as np
from numpy.polynomial import legendre, polynomial

from flash_fr3_reference import DelayedJointReference

# Suggested rectangular position limits from the FR3 FCI specification.
# These are narrower than hardware joint limits. Velocity is further restricted
# by the application here, not set to the nominal hardware maximum.
LIMIT_SOURCE = "https://frankarobotics.github.io/docs/robot_specifications.html"
Q_MIN = np.array([-2.3476, -1.5454, -2.4937, -2.7714, -2.51, .7773, -2.7045])
Q_MAX = np.array([2.3476, 1.5454, 2.4937, -.4226, 2.51, 4.2841, 2.7045])


def joint_state_vectors(names, positions, velocities):
    if len(names) != len(set(names)) or len(names) != len(positions) or len(names) != len(velocities):
        raise ValueError("JointState 名称重复或位置/速度长度不一致")
    indices = {name: i for i, name in enumerate(names)}
    try:
        order = [indices[f"fr3_joint{i}"] for i in range(1, 8)]
    except KeyError as error:
        raise ValueError("JointState 缺少所需关节") from error
    q, dq = np.asarray(positions, dtype=float)[order], np.asarray(velocities, dtype=float)[order]
    if not np.isfinite(np.r_[q, dq]).all():
        raise ValueError("JointState 位置或速度非有限")
    return q, dq


@dataclass(frozen=True)
class GuardLimits:
    local_excursion_rad: float = .10
    velocity_rad_s: float = .25
    acceleration_rad_s2: float = 1.0
    jerk_rad_s3: float = 20.0
    tracking_error_rad: float = .035
    initial_velocity_rad_s: float = .01
    state_age_seconds: float = .10
    prediction_age_seconds: float = .70
    prediction_silence_seconds: float = .70
    loop_gap_seconds: float = .10
    blend_seconds: float = 1.2
    preview_dt_seconds: float = .001

    def __post_init__(self):
        if not all(np.isfinite(v) and v > 0 for v in asdict(self).values()):
            raise ValueError("保护参数必须为有限正数")


def preview(reference, origin, limits):
    """Vectorized 1ms samples of the WHOLE remaining candidate, including C2 blend.

    Uses the physical time derivatives of the original and correction
    polynomials. Never clips joint references independently of derivatives.
    """
    times = np.unique(np.r_[np.arange(0, reference.remaining_seconds, limits.preview_dt_seconds),
                            reference.remaining_seconds, reference.blend])
    curve = reference.curve
    duration = float(curve["polynomial_duration_seconds"])
    x = 2 * (float(curve["anchor_s"]) + (reference.age + times) / duration) - 1
    coefficients = np.asarray(curve["coefficients"], dtype=np.float64)[:, :7]
    values = []
    blend_mask = times < reference.blend
    for order in range(4):
        raw = legendre.legval(x, legendre.legder(coefficients, m=order, axis=0)).T * (2 / duration)**order
        raw[blend_mask] += polynomial.polyval(
            times[blend_mask] / reference.blend,
            polynomial.polyder(reference.correction, m=order, axis=0)
        ).T / reference.blend**order
        values.append(raw)
    q, dq, ddq, jerk = values
    if not all(np.isfinite(v).all() for v in values):
        raise ValueError("参考采样出现非有限数")
    violations = {}
    tests = {
        "rectangular_position_range": ((q < Q_MIN) | (q > Q_MAX), q),
        "local_excursion_rad": (np.abs(q - origin) > limits.local_excursion_rad, np.abs(q - origin)),
        "velocity_rad_s": (np.abs(dq) > limits.velocity_rad_s, np.abs(dq)),
        "acceleration_rad_s2": (np.abs(ddq) > limits.acceleration_rad_s2, np.abs(ddq)),
        "jerk_rad_s3": (np.abs(jerk) > limits.jerk_rad_s3, np.abs(jerk)),
    }
    for name, (mask, actual) in tests.items():
        if mask.any():
            i, j = np.argwhere(mask)[0]
            violations[name] = {"first_seconds": float(times[i]), "joint": int(j + 1),
                                "value": float(actual[i, j])}
    return {
        "status": "REJECTED" if violations else "SAMPLED_LIMITS_PASS",
        "samples": len(times), "duration_seconds": reference.remaining_seconds,
        "per_joint_excursion_max_rad": np.abs(q - origin).max(axis=0).tolist(),
        "per_joint_velocity_max_rad_s": np.abs(dq).max(axis=0).tolist(),
        "per_joint_acceleration_max_rad_s2": np.abs(ddq).max(axis=0).tolist(),
        "per_joint_jerk_max_rad_s3": np.abs(jerk).max(axis=0).tolist(),
        "violations": violations,
        "scope": "sampled reference only; no collision, dynamics, torque or hardware-stop validation",
    }


class ShadowGuard:
    """Single-thread owner. BLOCKED latches until a new diagnostic instance.

    Calling tick/offer cannot stop a robot. A real controller requires its own
    watchdog if this Python process or its host stops running.
    """

    def __init__(self, limits=None):
        self.limits = limits or GuardLimits()
        self.mode, self.fault = "WAITING", None
        self.reference = self.origin = self.started = self.last_tick = None
        self.last_request, self.last_observation = -1, -np.inf
        self.last_preview = None
        self.accepted = 0

    def block(self, code, detail=""):
        if self.mode != "BLOCKED":
            self.mode = "BLOCKED"
            self.fault = {"code": code, "detail": str(detail)}
        return None

    def _state(self, state, now):
        q, dq = np.asarray(state["q"], dtype=float), np.asarray(state["dq"], dtype=float)
        ages = np.asarray([now - state["receipt_monotonic"], state["source_age_at_receipt_seconds"]])
        if q.shape != (7,) or dq.shape != (7,) or not np.isfinite(np.r_[q, dq, ages]).all():
            raise ValueError("关节状态形状或有限性异常")
        # Account for both transit age and time elapsed since receipt. An old
        # source message arriving just now must not masquerade as fresh state.
        if ages[0] < 0 or ages[1] < -.05 or max(ages[0], ages.sum()) > self.limits.state_age_seconds:
            raise ValueError("关节状态过期或时间戳异常")
        if ((q < Q_MIN) | (q > Q_MAX)).any():
            raise ValueError("实测位置超出本次实验的矩形关节范围")
        return q, dq

    def tick(self, now, state):
        if self.mode == "BLOCKED":
            return None
        if not np.isfinite(now) or (self.last_tick is not None and
                not 0 <= now - self.last_tick <= self.limits.loop_gap_seconds):
            return self.block("LOOP_DEADLINE", "参考检查线程时钟倒退或未及时运行")
        self.last_tick = now
        try:
            q, _ = self._state(state, now)
            if self.reference is None:
                return None
            elapsed = now - self.started
            if elapsed > self.limits.prediction_silence_seconds:
                return self.block("PREDICTION_TIMEOUT", "超过允许时间未接受新预测")
            if elapsed > self.reference.remaining_seconds:
                return self.block("REFERENCE_EXPIRED")
            reference = self.reference.evaluate(elapsed)
            if np.abs(reference[0] - q).max() > self.limits.tracking_error_rad:
                return self.block("TRACKING_ERROR", "实测 q 与假设参考偏差超过阈值；只读模式下不会跟随参考")
            return reference
        except (ValueError, KeyError, TypeError) as error:
            return self.block("STATE_OR_REFERENCE_INVALID", error)

    def offer(self, reply, observation_monotonic, now, state):
        if self.mode == "BLOCKED":
            return False
        try:
            if self.last_tick != now:
                raise ValueError("装入预测前必须在同一时刻完成 tick 检查")
            q, dq = self._state(state, now)
            request_id = reply["request_id"]
            if not isinstance(request_id, int) or request_id <= self.last_request or observation_monotonic <= self.last_observation:
                raise ValueError("重复、乱序或倒退的预测")
            age = now - float(observation_monotonic)
            if self.reference is None:
                if np.abs(dq).max() > self.limits.initial_velocity_rad_s:
                    raise ValueError("只允许从近似静止的实测状态初始化；不假设运动中的加速度为零")
                initial = np.stack([q, dq, np.zeros(7)])
            else:
                initial = self.reference.evaluate(now - self.started)
            candidate = DelayedJointReference(reply["curve"], age, initial,
                                             blend_seconds=self.limits.blend_seconds,
                                             max_observation_age=self.limits.prediction_age_seconds)
            origin = q.copy() if self.origin is None else self.origin
            self.last_preview = preview(candidate, origin, self.limits)
            if self.last_preview["violations"]:
                self.block("TRAJECTORY_LIMIT", ",".join(self.last_preview["violations"]))
                return False
            self.reference, self.origin, self.started = candidate, origin, now
            self.last_request, self.last_observation = request_id, observation_monotonic
            self.mode = "REFERENCE_ACTIVE"
            self.accepted += 1
            return True
        except (ValueError, KeyError, TypeError) as error:
            self.block("PREDICTION_INVALID", error)
            return False

    def report(self):
        return {"mode": self.mode, "fault": self.fault, "accepted": self.accepted,
                "limits": asdict(self.limits), "position_min_rad": Q_MIN.tolist(),
                "position_max_rad": Q_MAX.tolist(), "limit_source": LIMIT_SOURCE,
                "last_preview": self.last_preview, "robot_motion_commanded": False,
                "hardware_watchdog_implemented": False}
