"""Real ROS history selection on an executed-motion clock. Subscriptions only."""
import bisect
import os
import time
from collections import deque

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from sensor_msgs.msg import CameraInfo

from capture_flash_fr3_observation import (DT, FINGERS, JOINTS, TOPICS, measured_gripper_closure,
                                          ordered_positions, resize_rgb, stamp_seconds)
from flash_fr3_history_clock import ExecutedHistoryClock
from flash_fr3_retimed_schema import OBSERVATION_FORMAT, validate_retimed_observation
from flash_fr3_transport_diagnostics import own_udp_sockets, udp_drop_increment
from run_flash_fr3_shadow_client import ObservationBuffer


class HistoryCoverageError(ValueError):
    def __init__(self,diagnostic):
        self.diagnostic=diagnostic
        failed=','.join(diagnostic['uncovered_sources'])
        super().__init__(f"真实传感器历史尚未覆盖重定时网格：样本 {diagnostic['sample_index']}，来源 {failed}")


class RetimedObservationBuffer(ObservationBuffer):
    def __init__(self, node):
        super().__init__(node,queue_depths={'arm':256},track_source_order=True)
        # Whole bounded task history must survive idle gaps. The buffers hold
        # compressed camera messages, not decoded full-resolution RGB arrays.
        self.history = {key: deque(maxlen=155000 if key == 'arm' else 4650) for key in TOPICS}
        self.execution_clock = None
        self.wall_offset = None
        # Small CameraInfo messages provide a comparison stream for the actual
        # policy receiver, without duplicating compressed image subscriptions.
        self.color_info_history = {key: deque(maxlen=4650) for key in ('primary_cam', 'wrist_cam')}
        for key in self.color_info_history:
            topic = TOPICS[key].rsplit('/image_raw/compressed', 1)[0] + '/camera_info'

            def receive_info(message, key=key):
                item = (stamp_seconds(message), time.monotonic())
                with self.lock:
                    self.color_info_history[key].append(item)

            self.subscriptions.append(node.create_subscription(
                CameraInfo, topic, receive_info,
                QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)))
        self.transport_initial = {'pid': os.getpid(), 'monotonic': time.monotonic(),
                                  'rmw_implementation': get_rmw_implementation_identifier(),
                                  'udp_sockets': own_udp_sockets()}

    def transport_gap_context(self, diagnostic):
        """Snapshot the first post-initialization gap; do not change selection.

        Called at most once by the child process. No full joint-history scan,
        image decoding, extra image subscription or model request is involved.
        CameraInfo remains a ROS publication, not a device-side frame counter.
        """
        target = diagnostic['target_stamp_seconds']
        comparisons = {}
        with self.lock:
            for key in ('primary_cam', 'wrist_cam'):
                values = self.color_info_history[key]
                index = bisect.bisect_left(values, target, key=lambda item: item[0])
                source = diagnostic['sources'][key]
                low, high = source['before_stamp_seconds'], source['after_stamp_seconds']
                left = bisect.bisect_right(values, low, key=lambda item: item[0]) if low is not None else 0
                right = bisect.bisect_left(values, high, key=lambda item: item[0]) if high is not None else 0
                stamps_inside = [values[i][0] for i in range(left, right)] if low is not None and high is not None else []
                images = self.history[key]
                image_index = bisect.bisect_left(images, target, key=lambda item: stamp_seconds(item[0]))
                nearby = [images[i] for i in range(max(0, image_index - 1), min(len(images), image_index + 1))]
                comparisons[key] = {
                    'camera_info_messages_retained': len(values),
                    'camera_info_before_target': values[index - 1][0] if index else None,
                    'camera_info_after_target': values[index][0] if index < len(values) else None,
                    'compressed_before_target': low, 'compressed_after_target': high,
                    'camera_info_stamps_inside_compressed_gap': stamps_inside,
                    'compressed_neighbors_payload_bytes': [len(item[0].data) for item in nearby]}
        failure_sockets = own_udp_sockets()
        return {'receiver_pid': os.getpid(), 'captured_monotonic': time.monotonic(),
                'request_id': diagnostic['request_id'], 'sample_index': diagnostic['sample_index'],
                'target_stamp_seconds': target, 'initial_transport': self.transport_initial,
                'udp_sockets_at_failure': failure_sockets,
                'own_udp_drop_increment': udp_drop_increment(self.transport_initial['udp_sockets'], failure_sockets),
                'camera_comparisons': comparisons,
                'scope': 'actual policy receiver; CameraInfo is another ROS stream, not a device-side counter'}

    def record_arm(self, **interval):
        if self.execution_clock is None:
            raise ValueError('初始观测尚未就绪，不能记录执行历史')
        self.execution_clock.record_arm(**interval)
        return self.execution_clock.snapshot()

    def observation(self, request_id):
        now, wall = time.monotonic(), time.time()
        offset = wall-now
        if self.wall_offset is not None and abs(offset-self.wall_offset) > .02:
            raise ValueError('任务期间系统墙钟相对单调时钟变化超过 20 ms')
        # Only copy the 32 selected message references while holding the lock.
        # Walking every retained 1kHz joint message stalled the Python receiver
        # and its former depth-1 queue could lose the very history being read.
        # Source ordering is checked on receipt, rather than by a full scan.
        with self.lock:
            history=self.history
            if self.source_order_errors:
                raise ValueError('传感器历史时间戳倒退: '+','.join(sorted(self.source_order_errors)))
            if any(not value or now-value[-1][1]>.3 for value in history.values()):
                raise ValueError('缺少新鲜的相机/机械臂/夹爪状态')
            latest=min(now-.08,min(stamp_seconds(value[-1][0]) for value in history.values())-offset)
            clock=self.execution_clock or ExecutedHistoryClock(latest)
            policy,target_mono=clock.targets(latest)
            targets=[t+offset for t in target_mono]
            selections=[]
            for i,target in enumerate(targets):
                selected={};sources={}
                for key,values in history.items():
                    index=bisect.bisect_left(values,target,key=lambda item:stamp_seconds(item[0]))
                    before=values[index-1] if index else None
                    after=values[index] if index<len(values) else None
                    value=(before if after is None or (before is not None and
                           target-stamp_seconds(before[0])<=stamp_seconds(after[0])-target) else after)
                    selected[key]=value
                    stamp=stamp_seconds(value[0])
                    sources[key]={'message_count':len(values),
                        'earliest_stamp_seconds':stamp_seconds(values[0][0]),
                        'latest_stamp_seconds':stamp_seconds(values[-1][0]),
                        'before_stamp_seconds':stamp_seconds(before[0]) if before is not None else None,
                        'after_stamp_seconds':stamp_seconds(after[0]) if after is not None else None,
                        'selected_stamp_seconds':stamp,'selected_offset_seconds':stamp-target}
                uncovered=[key for key in TOPICS if abs(sources[key]['selected_offset_seconds'])>.08]
                if uncovered:
                    raise HistoryCoverageError({'request_id':request_id,'sample_index':i,
                        'sample_kind':'newest' if i==7 else 'historical',
                        'target_monotonic':target_mono[i],'target_stamp_seconds':target,
                        'fixed_lag_anchor_monotonic':now-.08,'common_anchor_monotonic':latest,
                        'maximum_offset_seconds':.08,'uncovered_sources':uncovered,'sources':sources})
                selections.append(selected)

        arrays = {key: [] for key in ('agent_pos', 'primary_cam', 'wrist_cam')}
        samples, cache_keys = [], set()
        for i, target in enumerate(targets):
            selected = selections[i]
            messages = {key: value[0] for key, value in selected.items()}
            stamps = {key: stamp_seconds(value) for key, value in messages.items()}
            ages = {key: selected[key][2]-stamps[key] for key in TOPICS}
            if any(v < -.05 or v > .3 for v in ages.values()):
                raise ValueError('历史源消息到达时已经过期')
            q = ordered_positions(messages['arm'], JOINTS)
            width = float(ordered_positions(messages['hand'], FINGERS).sum())
            arrays['agent_pos'].append(np.r_[q, measured_gripper_closure(width)].astype(np.float32))
            for key in ('primary_cam', 'wrist_cam'):
                m = messages[key]
                cache_key = (key, m.header.stamp.sec, m.header.stamp.nanosec)
                cache_keys.add(cache_key)
                if cache_key not in self.rgb_cache:
                    rgb = self.bridge.compressed_imgmsg_to_cv2(m, desired_encoding='rgb8')
                    self.rgb_cache[cache_key] = resize_rgb(np.asarray(rgb))
                arrays[key].append(self.rgb_cache[cache_key])
            samples.append({'relative_seconds': i*DT, 'policy_time_seconds': policy[i],
                            'target_monotonic': target_mono[i], 'target_stamp_seconds': target,
                            'stamp_seconds': stamps, 'source_age_at_receipt_seconds': ages,
                            'source_offset_from_target_seconds': {key: stamps[key]-target for key in TOPICS}})
        metadata = {'format': OBSERVATION_FORMAT, 'request_id': request_id,
                    'joint_order': JOINTS, 'topics': TOPICS, 'n_obs_steps': 8,
                    'sample_dt_seconds': DT, 'color': 'RGB', 'samples': samples,
                    'history_clock': clock.snapshot(), 'wall_minus_monotonic_seconds': offset,
                    'robot_motion_commanded': False, 'capture_mode': 'executed_motion_retimed'}
        result = validate_retimed_observation({key: np.stack(value) for key, value in arrays.items()}, metadata)
        # Seed only after all eight real source frames are available. A failed
        # warm-up attempt must not freeze an origin before camera discovery.
        if self.execution_clock is None:
            self.execution_clock = clock
            self.wall_offset = offset
        self.rgb_cache = {key: value for key, value in self.rgb_cache.items() if key in cache_keys}
        return result
