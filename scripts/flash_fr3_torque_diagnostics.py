"""Read-only sampled torque diagnostics. No ROS, actuation or reference repair."""
import numpy as np


def status_array(rows):
    a = np.asarray([row['data'] for row in rows], dtype=float)
    if a.ndim != 2 or a.shape[1] != 54 or not np.isfinite(a).all():
        raise ValueError('力矩诊断状态形状或数值异常')
    if len(a) < 2 or np.any(np.diff(a[:, 0]) <= 0):
        raise ValueError('力矩诊断采样时钟非递增')
    return a


def baseline_window(rows):
    a = status_array(rows)
    a = a[a[:, 0] >= a[-1, 0]-.4]
    if len(a) < 7 or a[-1, 0]-a[0, 0] < .25 or np.max(np.diff(a[:, 0])) > .1:
        raise ValueError('预保持基线采样不足或存在缺口')
    if (np.any(a[:, 1] != 0) or np.any(a[:, 2] != 0) or np.any(a[:, 53] != 0) or
            np.max(np.ptp(a[:, 18:25], axis=0)) > 1e-8 or
            np.max(abs(a[:, 25:32])) > 1e-8):
        raise ValueError('预保持参考或控制器状态异常')
    span = np.ptp(a[:, 4:11], axis=0)
    if np.max(span) > .0002:
        raise ValueError('预保持末段仍有明显漂移，不开始微动')
    return {'sample_count': len(a), 'span_seconds': float(a[-1, 0]-a[0, 0]),
            'measured_q_mean_rad': a[:, 4:11].mean(axis=0).tolist(),
            'reference_q_mean_rad': a[:, 18:25].mean(axis=0).tolist(),
            'measured_q_span_rad': span.tolist(),
            'baseline_span_limit_rad': .0002,
            'scope': 'local diagnostic stability condition, not paper or hardware tracking tolerance'}


def incremental_response(rows, baseline):
    a = status_array(rows)
    end = a[a[:, 0] >= a[-1, 0]-.3]
    if len(end) < 5 or end[-1, 0]-end[0, 0] < .15 or np.max(np.diff(end[:, 0])) > .1:
        raise ValueError('微动结束段采样不足或存在缺口')
    q = end[:, 4:11].mean(axis=0); ref = end[:, 18:25].mean(axis=0)
    actual = q-np.asarray(baseline['measured_q_mean_rad'])
    desired = ref-np.asarray(baseline['reference_q_mean_rad'])
    return {'status': 'INCREMENTAL_DIAGNOSTIC_ONLY',
            'measured_increment_rad': actual.tolist(), 'reference_increment_rad': desired.tolist(),
            'joint4_increment_ratio': float(actual[3]/desired[3]) if abs(desired[3]) > 1e-8 else None,
            'joint4_increment_direction_matches': bool(actual[3]*desired[3] > 0),
            'final_mean_absolute_reference_error_rad': abs(q-ref).tolist(),
            'tracking_validation': 'NOT_CERTIFIED',
            'scope': 'same-activation sampled response after prehold; absolute error retained; no offset compensation'}


def sdk_status_array(rows):
    """SDK schema v1: metadata 6, eight joint vectors, masses 3, success rate."""
    a = np.asarray([row['data'] for row in rows], dtype=float)
    if a.ndim != 2 or a.shape[1] != 66 or not np.isfinite(a).all():
        raise ValueError('SDK 力矩遥测形状或数值异常')
    if len(a) < 2 or np.any(np.diff(a[:, 1]) <= 0) or np.any(np.diff(a[:, 2]) <= 0):
        raise ValueError('SDK 力矩遥测时钟非递增')
    if (np.any(a[:, 0] != 1) or np.any(~np.isin(a[:, 3], [0, 1, 4])) or
            np.any(~np.isin(a[:, 4], [0, 1])) or np.any(a[:, 5] <= 0) or
            np.any(a[:, 5] > .002) or np.any(a[:, 62:65] < 0) or
            np.max(abs(a[:, 62]+a[:, 63]-a[:, 64])) > 1e-5 or
            np.any(a[:, 65] < 0) or np.any(a[:, 65] > 1)):
        raise ValueError('SDK 力矩遥测版本、状态、周期或负载元数据异常')
    return a


def sdk_prehold_window(rows):
    a = sdk_status_array(rows)
    a = a[a[:, 1] >= a[-1, 1]-.4]
    if (len(a) < 7 or a[-1, 1]-a[0, 1] < .25 or
            np.max(np.diff(a[:, 1])) > .1 or np.any(a[:, 3] != 0) or
            np.any(a[:, 4] != 1)):
        raise ValueError('预保持 SDK 力矩遥测不足或阶段异常，不开始微动')
    return {'sample_count': len(a), 'span_seconds': float(a[-1, 1]-a[0, 1]),
            'sdk_desired_torque_mean_nm': a[:, 6:13].mean(axis=0).tolist(),
            'scope': 'presence and clock check; not proof of torque tracking'}


def sdk_torque_summary(rows):
    a = sdk_status_array(rows)
    sdk_prehold_window([row for row in rows if row['data'][3] == 0])
    if len(a) < 20 or a[-1, 1]-a[0, 1] < 5. or np.max(np.diff(a[:, 1])) > .15:
        raise ValueError('SDK 力矩遥测未覆盖试验或存在缺口')
    hold = a[a[:, 3] == 0]
    hold = hold[hold[:, 1] >= hold[-1, 1]-.4]
    end = a[a[:, 1] >= a[-1, 1]-.3]
    if np.count_nonzero(a[:, 3] == 1) < 10 or len(end) < 5 or end[-1, 1]-end[0, 1] < .15:
        raise ValueError('SDK 力矩遥测运动或终点采样不足')
    valid = a[a[:, 4] == 1]
    difference = abs(valid[:, 6:13]-valid[:, 13:20])
    return {'status': 'SDK_TORQUE_AUDIT_COMPLETE', 'schema_version': 1,
            'sample_count': len(a), 'span_seconds': float(a[-1, 1]-a[0, 1]),
            'sdk_minus_previous_written_mae_nm': difference.mean(axis=0).tolist(),
            'sdk_minus_previous_written_peak_abs_nm': difference.max(axis=0).tolist(),
            'sdk_desired_torque_increment_nm': (end[:, 6:13].mean(axis=0)-hold[:, 6:13].mean(axis=0)).tolist(),
            'written_torque_increment_nm': (end[:, 20:27].mean(axis=0)-hold[:, 20:27].mean(axis=0)).tolist(),
            'sdk_measured_position_increment_rad': (end[:, 48:55].mean(axis=0)-hold[:, 48:55].mean(axis=0)).tolist(),
            'sdk_desired_torque_peak_abs_nm': abs(a[:, 6:13]).max(axis=0).tolist(),
            'sdk_measured_torque_peak_abs_nm': abs(a[:, 27:34]).max(axis=0).tolist(),
            'sdk_external_torque_final_mean_nm': end[:, 34:41].mean(axis=0).tolist(),
            'configured_mass_ee_load_total_kg': a[-1, 62:65].tolist(),
            'command_success_rate_min': float(a[:, 65].min()),
            'tracking_validation': 'NOT_CERTIFIED',
            'scope': 'same-update SDK tau_J_d versus preceding successful interface write; '
                     'filtering and hardware timing can contribute to differences; '
                     'measured tau_J includes different compensation and is not equated to user torque'}
