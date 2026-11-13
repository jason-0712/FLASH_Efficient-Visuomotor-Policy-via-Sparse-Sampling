"""Offline joint-reference construction. No ROS, networking or robot IO.

This C2 correction is an execution-adaptation experiment, not part of FLASH.
It does not enforce actuator limits, track measured states, or stop a robot.
"""
import copy

import numpy as np
from numpy.polynomial import legendre, polynomial

from flash_fr3_shadow_math import evaluate_curve


class DelayedJointReference:
    """Start a delayed prediction at a supplied q/dq/ddq, then remove correction.

    Times use seconds and all seven joint dimensions use physical units.
    The gripper is deliberately excluded. An expired evaluation raises rather
    than extending the polynomial beyond its validated prediction horizon.
    """

    def __init__(self, curve, observation_age, initial, blend_seconds=1.2,
                 max_observation_age=0.7):
        self.curve = copy.deepcopy(curve)
        self.age = float(observation_age)
        self.blend = float(blend_seconds)
        max_age = float(max_observation_age)
        if not np.isfinite([self.age, self.blend, max_age]).all():
            raise ValueError("参考轨迹时间参数必须有限")
        if self.age < 0 or self.age > max_age or max_age <= 0:
            raise ValueError("预测时龄超出离线实验接受范围")
        if self.blend <= 0:
            raise ValueError("过渡时间必须为正")
        # evaluate_curve also validates the physical polynomial and horizon.
        raw = evaluate_curve(self.curve, self.age)
        evaluate_curve(self.curve, self.age + self.blend)
        state = np.asarray(initial, dtype=np.float64)
        if state.shape != (3, 7) or not np.isfinite(state).all():
            raise ValueError("初始状态必须包含七关节的有限 q、dq、ddq")
        delta = state - np.asarray(raw)[:, :7]
        d0, d1, d2 = delta * np.array([1, self.blend, self.blend**2 / 2])[:, None]
        # Quintic Hermite correction, expressed in u=elapsed/blend.
        self.correction = np.stack([
            d0, d1, d2,
            -10 * d0 - 6 * d1 - 3 * d2,
            15 * d0 + 8 * d1 + 3 * d2,
            -6 * d0 - 3 * d1 - d2,
        ])
        self.remaining_seconds = float(self.curve["valid_future_seconds"]) - self.age

    def _evaluate(self, elapsed, include_jerk):
        t = float(elapsed)
        if not np.isfinite(t) or not 0 <= t <= self.remaining_seconds + 1e-9:
            raise ValueError("参考轨迹已过期或请求时刻无效；不继续外推")
        values = [v[:7] for v in evaluate_curve(self.curve, self.age + t)]
        if include_jerk:
            coefficients = np.asarray(self.curve["coefficients"], dtype=np.float64)[:, :7]
            duration = float(self.curve["polynomial_duration_seconds"])
            x = 2 * (float(self.curve["anchor_s"]) + (self.age + t) / duration) - 1
            values.append(legendre.legval(x, legendre.legder(coefficients, m=3, axis=0))
                          * (2 / duration)**3)
        if t < self.blend:
            u = t / self.blend
            for order in range(len(values)):
                values[order] = values[order] + polynomial.polyval(
                    u, polynomial.polyder(self.correction, m=order, axis=0)
                ) / self.blend**order
        return tuple(values)

    def evaluate(self, elapsed):
        return self._evaluate(elapsed, include_jerk=False)

    def evaluate_with_jerk(self, elapsed):
        # C2 does not imply jerk continuity at a handoff or blend completion.
        return self._evaluate(elapsed, include_jerk=True)
