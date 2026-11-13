"""Fixed low-level velocity-feedforward probe; no policy or robot IO.

Joint 4 moves by +0.001 rad over 3 s using a rest-to-rest quintic, then
holds for 0.75 s. This is commissioning, not a FLASH-generated trajectory.
"""
import numpy as np

from flash_fr3_task_plan import task_bounds


def probe_plan(start_q):
    coefficients = np.zeros((2, 7, 7))
    coefficients[0, 3:6, 3] = np.array([10., -15., 6.]) * .001
    coefficients[1, 0, 3] = .001
    plan = {'expected_start_q': np.asarray(start_q, float).tolist(),
            'segment_durations': [3., .75], 'coefficients': coefficients.tolist()}
    bounds = task_bounds(plan)
    limits = np.array([.002, .005, .02, .2])[:, None]
    if (not bounds['pass'] or
            (np.asarray(bounds['nominal_bounds']) > limits).any() or
            (np.asarray(bounds['watchdog_brake_bounds']) > limits).any()):
        raise ValueError('解析速度测试曲线未通过现有力矩接入范围；不放宽限值')
    plan['bounds'] = bounds
    return plan
