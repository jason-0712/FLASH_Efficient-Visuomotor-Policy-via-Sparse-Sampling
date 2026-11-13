"""Frozen FLASH prefix -> two C2 position segments. No ROS or robot IO.

This commissioning adapter slows a prefix of the actual polynomial. It is not
online replanning, an original-paper configuration, or a complete task policy.
Coefficients have shape [segment, normalized-time power, joint].
"""
import numpy as np
from numpy.polynomial import Polynomial, Legendre, polynomial

from flash_fr3_reference import DelayedJointReference


def evaluate_segment(coefficients, duration, elapsed, order=0):
    return polynomial.polyval(elapsed / duration,
                              polynomial.polyder(coefficients, m=order, axis=0)) / duration**order


def make_segments(curve, age, start_q, source_prefix=.075, time_scale=8., stop_seconds=.5):
    q = np.asarray(start_q, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError("起始关节位置异常")
    if not np.isfinite([source_prefix, time_scale, stop_seconds]).all() or not (
            .05 <= source_prefix <= .25 and 2 <= time_scale <= 10 and .25 <= stop_seconds <= 2):
        raise ValueError("短段试运行的时间参数异常")
    initial = np.stack([q, np.zeros(7), np.zeros(7)])
    reference = DelayedJointReference(curve, age, initial, blend_seconds=1.2)
    if source_prefix >= reference.blend or source_prefix > reference.remaining_seconds:
        raise ValueError("短段超过预测或连续过渡的定义范围")
    # Exact polynomial composition, not a sampled waypoint fit. x is affine in
    # u; physical playback duration is source_prefix * time_scale.
    duration = float(curve["polynomial_duration_seconds"])
    x = Polynomial([2*(curve["anchor_s"] + age/duration)-1, 2*source_prefix/duration])
    correction_u = Polynomial([0, source_prefix/reference.blend])
    coefficients = np.zeros((2, 7, 7))
    for joint in range(7):
        raw = Legendre(np.asarray(curve["coefficients"])[:, joint]).convert(kind=Polynomial)(x)
        correction = Polynomial(reference.correction[:, joint])(correction_u)
        relative = raw + correction - q[joint]
        coefficients[0, :len(relative.coef), joint] = relative.coef
    # Numerical cancellation can leave roundoff; require a truly static origin.
    if np.abs(coefficients[0, :3]).max() > 1e-10:
        raise ValueError("轨迹起点未满足静止 C2 条件")
    coefficients[0, :3] = 0
    durations = [source_prefix*time_scale, float(stop_seconds)]
    end = np.asarray([evaluate_segment(coefficients[0], durations[0], durations[0], k)
                      for k in range(3)])
    # Minimum-integrated-squared-jerk stopping position for this quintic family.
    destination = end[0] + .5*stop_seconds*end[1] + stop_seconds**2*end[2]/12
    c0, c1, c2 = end * np.array([1, stop_seconds, stop_seconds**2/2])[:, None]
    delta = destination-c0
    coefficients[1, :6] = np.stack([c0, c1, c2, 10*delta-6*c1-3*c2,
                                   -15*delta+8*c1+3*c2, 6*delta-3*c1-c2])
    # Cross-check independent evaluation against the original Legendre adapter.
    error = 0.
    for u in np.linspace(0, 1, 101):
        for order in range(4):
            actual = evaluate_segment(coefficients[0], durations[0], u*durations[0], order)
            expected = reference.evaluate_with_jerk(u*source_prefix)[order] / time_scale**order
            if order == 0:
                expected = expected-q
            error = max(error, float(np.abs(actual-expected).max()))
    if error > 1e-9:
        raise ValueError("模型多项式转换与独立求值不一致")
    return {"segment_durations": durations, "coefficients": coefficients.tolist(),
            "source_prefix_seconds": source_prefix, "time_scale": time_scale,
            "nominal_stop_seconds": stop_seconds, "conversion_max_abs_error": error,
            "total_motion_seconds": sum(durations), "observation_age_at_plan_seconds": age,
            "expected_start_q": q.tolist()}


def write_preview_csv(path, segments):
    import csv
    coefficients = np.asarray(segments["coefficients"])
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s"] + [f"{key}{j}" for key in ("q", "dq", "ddq", "jerk") for j in range(1, 8)])
        offset = 0.
        for c, duration in zip(coefficients, segments["segment_durations"]):
            for t in np.linspace(0, duration, int(np.ceil(duration/.001))+1):
                values = [evaluate_segment(c, duration, t, order) for order in range(4)]
                values[0] += np.asarray(segments["expected_start_q"])
                writer.writerow([offset+t] + np.r_[tuple(values)].tolist())
            offset += duration
