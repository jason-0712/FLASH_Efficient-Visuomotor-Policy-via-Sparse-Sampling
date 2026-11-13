"""Exact C2 rolling FLASH prefixes, in physical playback time. No robot IO."""
import numpy as np
from numpy.polynomial import Polynomial, Legendre

from flash_fr3_model_plan import evaluate_segment
from flash_fr3_reference import DelayedJointReference


def make_stream_segments(curve, age_at_handoff, initial, origin,
                         source_prefix=.05, time_scale=10., stop_seconds=.5):
    initial, origin = np.asarray(initial, float), np.asarray(origin, float)
    if (initial.shape != (3, 7) or origin.shape != (7,) or
            not np.isfinite(initial).all() or not np.isfinite(origin).all() or
            not np.isfinite([source_prefix, time_scale, stop_seconds]).all() or
            not (.04 <= source_prefix <= .075 and 8 <= time_scale <= 12.5 and
                 .5 <= stop_seconds <= 1)):
        raise ValueError("滚动短段的起点或时间参数异常")
    # The source curve runs time_scale times faster than playback. Its initial
    # derivatives must be multiplied before correcting it, not set to zero.
    initial_source = initial * np.array([1., time_scale, time_scale**2])[:, None]
    reference = DelayedJointReference(curve, age_at_handoff, initial_source, blend_seconds=1.2)
    if source_prefix > reference.remaining_seconds:
        raise ValueError("滚动短段超出预测范围")
    duration = float(curve["polynomial_duration_seconds"])
    x = Polynomial([2*(curve["anchor_s"]+age_at_handoff/duration)-1,
                    2*source_prefix/duration])
    correction_u = Polynomial([0, source_prefix/reference.blend])
    coefficients = np.zeros((2, 7, 7))
    for joint in range(7):
        raw = Legendre(np.asarray(curve["coefficients"])[:, joint]).convert(kind=Polynomial)(x)
        relative = raw + Polynomial(reference.correction[:, joint])(correction_u) - origin[joint]
        coefficients[0, :len(relative.coef), joint] = relative.coef
    durations = [source_prefix*time_scale, float(stop_seconds)]
    boundary = np.asarray([evaluate_segment(coefficients[0], durations[0], 0., k)
                           for k in range(3)])
    expected = initial.copy(); expected[0] -= origin
    if np.abs(boundary-expected).max() > 1e-10:
        raise ValueError("滚动曲线未匹配给定的 q/dq/ddq")
    # Remove cancellation at the join while preserving all nonzero derivatives.
    coefficients[0, :3] = expected * np.array([1., durations[0], durations[0]**2/2])[:, None]
    end = np.asarray([evaluate_segment(coefficients[0], durations[0], durations[0], k)
                      for k in range(3)])
    c0, c1, c2 = end * np.array([1., stop_seconds, stop_seconds**2/2])[:, None]
    coefficients[1, :5] = np.stack([c0, c1, c2, -c1-4*c2/3, .5*c1+.5*c2])
    error = 0.
    for t in np.linspace(0, source_prefix, 7):
        for order, value in enumerate(reference.evaluate_with_jerk(t)):
            actual = evaluate_segment(coefficients[0], durations[0], t*time_scale, order)
            expected_value = value/time_scale**order - (origin if order == 0 else 0.)
            error = max(error, float(np.abs(actual-expected_value).max()))
    if error > 1e-9:
        raise ValueError("滚动曲线转换与独立 Legendre 求值不一致")
    return {"segment_durations": durations, "coefficients": coefficients.tolist(),
            "expected_start_q": origin.tolist(), "source_prefix_seconds": source_prefix,
            "time_scale": time_scale, "nominal_stop_seconds": stop_seconds,
            "initial_state": initial.tolist(), "observation_age_at_handoff_seconds": age_at_handoff,
            "conversion_max_abs_error": error, "total_motion_seconds": sum(durations)}


def prefix_end(plan):
    c = np.asarray(plan["coefficients"])[0]
    duration = plan["segment_durations"][0]
    values = np.asarray([evaluate_segment(c, duration, duration, k) for k in range(3)])
    values[0] += np.asarray(plan["expected_start_q"])
    return values


def absolute_bound(coefficients, duration, order):
    from math import comb, factorial
    c = np.asarray(coefficients)
    n = 6-order
    derivative = np.asarray([c[k+order]*factorial(k+order)/factorial(k)/duration**order
                             for k in range(n+1)])
    bernstein = np.asarray([sum(comb(i,k)/comb(n,k)*derivative[k] for k in range(i+1))
                            for i in range(n+1)])
    return np.abs(bernstein).max(axis=0)


def validate_stream_bounds(plan):
    """Same Bernstein/triangle bounds as native code; relative to run origin."""
    c = np.asarray(plan["coefficients"], float)
    d = np.asarray(plan["segment_durations"], float)
    if (c.shape != (2,7,7) or d.shape != (2,) or not np.isfinite(c).all() or
            not np.isfinite(d).all() or (d < .1).any() or (d > 5).any()):
        raise ValueError("滚动计划结构异常")
    left = np.asarray([evaluate_segment(c[0],d[0],d[0],k) for k in range(3)])
    right = np.asarray([evaluate_segment(c[1],d[1],0,k) for k in range(3)])
    tail = np.asarray([evaluate_segment(c[1],d[1],d[1],k) for k in (1,2)])
    if np.abs(left-right).max() > 1e-9 or np.abs(tail).max() > 1e-9:
        raise ValueError("滚动计划缺少 C2 连接或静止终点")
    bounds = np.max([[absolute_bound(p,t,k) for k in range(4)] for p,t in zip(c,d)],axis=0)
    unit_v, unit_a = np.zeros((7,7)), np.zeros((7,7))
    b = .25
    unit_v[1] = b; unit_v[3] = -b; unit_v[4] = .5*b
    unit_a[2] = .5*b*b; unit_a[3] = -2*b*b/3; unit_a[4] = .25*b*b
    brake = np.asarray([(bounds[0] if k == 0 else 0.) +
                         absolute_bound(unit_v,b,k)*bounds[1] +
                         absolute_bound(unit_a,b,k)*bounds[2] for k in range(4)])
    limits = np.array([.01,.02,.1,5.])[:,None]
    return {"pass": bool((bounds < limits-1e-10).all() and (brake < limits-1e-10).all()),
            "nominal_bounds": bounds.tolist(), "watchdog_brake_bounds": brake.tolist()}
