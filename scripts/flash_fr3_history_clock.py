"""Executed-motion clock for observation selection; no robot or ROS imports.

An arm interval advances policy time by elapsed real time / playback scale.
Idle gaps do not advance it. The newest sample always comes from the current
scene; older samples map to real source times inside completed intervals.
"""
import bisect
import math

DT = 4 / 15
FORMAT = 'flash_fr3_executed_history_clock_v1'


class ExecutedHistoryClock:
    def __init__(self, origin_monotonic):
        origin = float(origin_monotonic)
        if not math.isfinite(origin) or origin < 0:
            raise ValueError('历史时钟起点无效')
        self.origin = origin
        self.intervals = []

    @property
    def cursor(self):
        return self.intervals[-1]['policy_end_seconds'] if self.intervals else 0.

    def record_arm(self, start_monotonic, end_monotonic, time_scale):
        start, end, scale = map(float, (start_monotonic, end_monotonic, time_scale))
        if not all(map(math.isfinite, (start, end, scale))) or scale not in (4., 6., 8.):
            raise ValueError('已执行运动的时间参数无效')
        previous_end = self.intervals[-1]['end_monotonic'] if self.intervals else self.origin
        if start < previous_end or not 1.53 <= end-start <= 5.57:
            raise ValueError('运动时间重叠、倒退或超出既有任务段时长')
        if end-self.origin > 150:
            raise ValueError('历史时钟超过本次任务保留范围')
        beginning = self.cursor
        self.intervals.append({'kind': 'completed_arm_segment', 'start_monotonic': start,
                               'end_monotonic': end, 'time_scale': scale,
                               'policy_start_seconds': beginning,
                               'policy_end_seconds': beginning + (end-start)/scale})

    def source_time(self, policy_seconds):
        t = float(policy_seconds)
        if not math.isfinite(t) or t > self.cursor:
            raise ValueError('历史取样不能使用未来的轨迹进度')
        if t <= 0:
            return self.origin+t
        ends = [part['policy_end_seconds'] for part in self.intervals]
        index = min(bisect.bisect_left(ends, t), len(self.intervals)-1)
        part = self.intervals[index]
        return part['start_monotonic'] + (t-part['policy_start_seconds'])*part['time_scale']

    def targets(self, latest_source_monotonic):
        latest = float(latest_source_monotonic)
        last_end = self.intervals[-1]['end_monotonic'] if self.intervals else self.origin
        if not math.isfinite(latest) or latest < last_end or latest-self.origin > 150:
            raise ValueError('当前观测早于运动结束或历史保留范围已过期')
        policy = [self.cursor-(7-i)*DT for i in range(8)]
        targets = [self.source_time(t) for t in policy[:-1]] + [latest]
        if any(b <= a for a, b in zip(targets, targets[1:])):
            raise ValueError('重定时后的真实观测时间不递增')
        return policy, targets

    def snapshot(self):
        return {'format': FORMAT, 'origin_monotonic': self.origin,
                'cursor_seconds': self.cursor, 'sample_dt_seconds': DT,
                'intervals': [dict(part) for part in self.intervals],
                'idle_time_policy': 'frozen', 'newest_sample_policy': 'current_scene',
                'stopping_tail_policy': 'elapsed_time_divided_by_arm_playback_scale'}

    @classmethod
    def restore(cls, value):
        if value.get('format') != FORMAT or value.get('sample_dt_seconds') != DT:
            raise ValueError('不支持的历史时钟格式')
        clock = cls(value['origin_monotonic'])
        for part in value['intervals']:
            if part.get('kind') != 'completed_arm_segment':
                raise ValueError('未知的已执行历史区间')
            clock.record_arm(part['start_monotonic'], part['end_monotonic'], part['time_scale'])
            actual = clock.intervals[-1]
            if any(abs(float(part[key])-actual[key]) > 1e-8
                   for key in ('policy_start_seconds', 'policy_end_seconds')):
                raise ValueError('轨迹进度与真实执行时间不一致')
        if abs(clock.cursor-float(value['cursor_seconds'])) > 1e-8:
            raise ValueError('历史时钟末端不一致')
        return clock


def arm_interval_from_status(rows, plan):
    """Bracket real motion from native phase transitions, not CLI elapsed time.

The native publisher emits phase changes immediately. Refuse missing start/end
transitions or a duration discrepancy larger than 20 ms, rather than inventing
an interval after an interrupted/failed segment.
"""
    moving = [row['data'] for row in rows if int(row['data'][1]) == 1]
    completed = [row['data'] for row in rows if int(row['data'][1]) == 4]
    if not moving or not completed or any(int(row['data'][2]) != 0 for row in rows):
        raise ValueError('缺少成功运动的原生开始/结束状态')
    start, end = float(moving[0][0]), float(completed[0][0])
    if abs(end-start-float(plan['total_motion_seconds'])) > .02:
        raise ValueError('原生运动区间与计划时长偏差超过 20 ms')
    return {'start_monotonic': start, 'end_monotonic': end,
            'time_scale': float(plan['time_scale'])}
