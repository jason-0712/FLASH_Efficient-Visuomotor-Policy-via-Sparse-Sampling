"""Bounded rest-to-rest joint return; no ROS, model inference or robot IO."""
import numpy as np

from flash_fr3_task_plan import TASK_LOWER, TASK_UPPER, task_bounds

JOINT_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
# Larger rest-to-rest legs stay within the unchanged native 0.20 rad
# displacement/braking envelope; the extra 0.02 rad covers start feedback.
MAX_STEP_RAD = .16
WAYPOINT_STEP_RAD = .14
RETURN_TOLERANCE_RAD = .002
MAX_SEGMENTS = 45


def checked_joints(value):
    q = np.asarray(value, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError('回位关节角必须是 7 个有限数')
    if ((q < TASK_LOWER) | (q > TASK_UPPER)).any():
        raise ValueError('回位起点或终点超出当前任务部署范围；不能从任意姿态自动返回')
    return q


def load_pose(value):
    if (value.get('format') != 'flash_fr3_saved_joint_pose_v1' or
            value.get('robot_ip') != '172.16.0.2' or
            value.get('joint_names') != JOINT_NAMES or value.get('units') != 'rad'):
        raise ValueError('保存姿态的格式、机器人地址、关节顺序或单位不匹配')
    return checked_joints(value.get('q_rad'))


def return_waypoints(start, target):
    start, target = checked_joints(start), checked_joints(target)
    gap = float(abs(target-start).max())
    if gap <= RETURN_TOLERANCE_RAD:
        return []
    count = int(np.ceil(gap/WAYPOINT_STEP_RAD))
    if count > MAX_SEGMENTS:
        raise ValueError('所需回位段数超出上限')
    return [start+(target-start)*(i/count) for i in range(1, count+1)]


def make_return_segment(start, target):
    start, target = checked_joints(start), checked_joints(target)
    delta = target-start
    if abs(delta).max() > MAX_STEP_RAD+1e-12:
        raise ValueError('单个回位段逐关节位移超过 0.16 rad')
    # Quintic smoothstep has zero velocity/acceleration at both endpoints.
    # The existing native TaskController then holds the endpoint for 0.75 s.
    coefficients = np.zeros((2, 7, 7))
    coefficients[0, 3:6] = np.array([10., -15., 6.])[:, None]*delta
    coefficients[1, 0] = delta
    # Select the shortest duration certified by the existing Python/native
    # derivative and heartbeat-brake bounds. No controller limit is increased.
    for duration in (.8, 1., 1.25, 1.5, 2., 2.5, 3., 3.5, 4., 4.8):
        plan = {'format': 'flash_fr3_return_segment_v1',
                'expected_start_q': start.tolist(), 'target_q_rad': target.tolist(),
                'coefficients': coefficients.tolist(), 'segment_durations': [duration, .75],
                'total_motion_seconds': duration+.75}
        bounds = task_bounds(plan)
        if bounds['pass']:
            plan['bounds'] = bounds
            return plan
    raise ValueError('回位段或心跳停车曲线未通过既有关节/导数范围检查')
