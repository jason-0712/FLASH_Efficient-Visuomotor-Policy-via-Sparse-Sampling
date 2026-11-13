"""Continuous effort wire contract and offline replay. No ROS or hardware IO."""
import math
import re

import numpy as np


def effort_message(packet, session_id, sent_wall_seconds):
    """Return the exact Float64MultiArray fields; do not publish anything.

    The native subscriber additionally checks activation, current source age,
    heartbeat/session identity, envelopes and raw boundary continuity.
    """
    if not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{16,128}', session_id):
        raise ValueError('需要独立的 16–128 字符会话 ID')
    sequence = packet['sequence']
    if isinstance(sequence, bool) or not isinstance(sequence, int) or not 0 <= sequence <= 1000000:
        raise ValueError('预测序号异常')
    if packet.get('time_policy') != 'physical_1x_no_history_freeze_no_coefficient_correction':
        raise ValueError('只接受未经修正或重定时的原始物理时间预测')
    curve = packet['curve']
    coefficients = np.asarray(curve['coefficients'], dtype=float)
    if curve.get('basis') != 'legendre' or coefficients.shape != (7, 8) or not np.isfinite(coefficients).all():
        raise ValueError('需要原始 7×8 Legendre 系数')
    physical = []
    for key, expected in (('anchor_s', 2/11), ('polynomial_duration_seconds', 44/15),
                          ('valid_future_seconds', 32/15)):
        value = float(curve[key])
        if not math.isfinite(value) or abs(value-expected) > 1e-7:
            raise ValueError('物理时间尺度不匹配')
        physical.append(value)
    observation, ready, handoff, sent = map(float, (
        packet['observation_monotonic'], packet['ready_monotonic'],
        packet['handoff_monotonic'], sent_wall_seconds))
    if (not np.isfinite([observation, ready, handoff, sent]).all() or observation < 0 or
            not 0 <= ready-observation <= .7 or handoff-ready < .02 or not 0 <= sent <= 9e9):
        raise ValueError('预测时龄或切换时间异常')
    origin = packet.get('curve_origin_monotonic')
    if origin is not None and (not math.isfinite(origin) or sequence != 0 or abs(origin-handoff)>1e-8):
        raise ValueError('独立曲线起点只允许首条预测在切换时从动作锚点启动')
    data = [1., float(sequence), sent, observation, handoff, *physical,
            *coefficients[:, :7].ravel().tolist()]
    if origin is not None:
        data[0] = 2.
        data.append(float(origin))
    return {'layout': {'dim': [{'label': session_id, 'size': len(data), 'stride': len(data)}], 'data_offset': 0},
            'data': data}


def admission_input(packets, measured_q, wall_minus_monotonic, session_id, session_seconds=8.):
    if not 1 <= len(packets) <= 512 or not math.isfinite(session_seconds) or session_seconds < 1:
        raise ValueError('离线重放数量或会话时长异常')
    q = np.asarray(measured_q, float)
    if q.shape != (7,) or not np.isfinite(q).all() or not math.isfinite(wall_minus_monotonic):
        raise ValueError('需要七关节实测起点和归档时钟偏移')
    lines = [' '.join(f'{x:.17g}' for x in q), f'{len(packets)} {session_seconds:.17g}']
    for packet in packets:
        sent = packet['ready_monotonic']+wall_minus_monotonic
        message = effort_message(packet, session_id, sent)
        fields = [packet['ready_monotonic'], *message['data']]
        lines.append(' '.join(f'{x:.17g}' for x in fields))
    return '\n'.join(lines)+'\n'
