"""RealSense per-frame host-arrival clock. No ROS, retiming or pixel changes."""
import json
import math

SDK_TIME_BASIS = 'realsense_sdk_time_of_arrival'


def frame_clock(sec, nanosec, payload):
    data = json.loads(payload)
    def integer(name):
        value = data[name]
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) or value != int(value) or value < 0):
            raise ValueError('相机元数据整数异常：'+name)
        return int(value)
    arrival = integer('time_of_arrival')
    number = integer('frame_number')
    if not 1e12 < arrival < 9e12:
        raise ValueError('需要 SDK 提供的 Unix 毫秒到达时间')
    if 'frame_counter' in data and integer('frame_counter') != number:
        raise ValueError('相机元数据帧号不一致')
    if (not isinstance(sec, int) or sec < 0 or not isinstance(nanosec, int) or
            not 0 <= nanosec < 1000000000):
        raise ValueError('原始相机 ROS 时间戳异常')
    return dict(basis=SDK_TIME_BASIS, source_stamp_seconds=arrival/1000.,
        sdk_time_of_arrival_ms=arrival, frame_number=number,
        original_ros_header_stamp_seconds=sec+nanosec*1e-9,
        device_clock_domain=str(data.get('clock_domain','unknown')))


def observation_stamp(item):
    if len(item) == 4:
        return item[3]['source_stamp_seconds']
    stamp = item[0].header.stamp
    return stamp.sec+stamp.nanosec*1e-9
