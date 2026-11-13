"""Physical-clock contract and raw FLASH packet encoding. No ROS or robot IO."""
import math

import numpy as np

from flash_fr3_shadow_math import evaluate_curve, validate_reply
from flash_fr3_camera_clock import SDK_TIME_BASIS

DT = 4/15
KEYS = ('arm','hand','primary_cam','wrist_cam')


def physical_reply(reply, request_id, anchor, checkpoint_step):
    validate_reply(reply,request_id,anchor,checkpoint_step)
    if ('history_clock' not in reply or
            reply.get('observation_format')!='flash_fr3_live_observation_v1' or
            reply.get('history_clock') is not None):
        raise ValueError('服务未确认未重定时的物理历史格式')


def physical_history(metadata, previous_anchor=None):
    if (metadata.get('format')!='flash_fr3_live_observation_v1' or
            metadata.get('history_clock') is not None or metadata.get('sample_dt_seconds')!=DT or
            metadata.get('n_obs_steps')!=8 or metadata.get('color')!='RGB' or
            metadata.get('joint_order')!=[f'fr3_joint{i}' for i in range(1,8)] or
            set(metadata['topics'])!=set(KEYS)):
        raise ValueError('连续输入必须使用真实物理时间网格，不能冻结或重定时历史')
    samples=metadata['samples']
    if len(samples)!=8: raise ValueError('需要八帧历史')
    basis=metadata.get('camera_time_basis','ros_header')
    if basis not in ('ros_header',SDK_TIME_BASIS):raise ValueError('未知相机时间来源')
    if basis==SDK_TIME_BASIS:
        previous_frames={}
        for sample in samples:
            clocks=sample.get('camera_clock',{})
            if set(clocks)!={'primary_cam','wrist_cam'}:raise ValueError('缺少逐帧 SDK 时钟来源')
            for key,clock in clocks.items():
                arrival=clock.get('sdk_time_of_arrival_ms');number=clock.get('frame_number')
                if (clock.get('basis')!=SDK_TIME_BASIS or type(arrival) is not int or not 1e12<arrival<9e12 or
                        type(number) is not int or number<=previous_frames.get(key,-1) or
                        not math.isfinite(clock.get('original_ros_header_stamp_seconds',float('nan'))) or
                        abs(arrival/1000.-clock.get('source_stamp_seconds',float('inf')))>1e-6 or
                        abs(arrival/1000.-sample['stamp_seconds'][key])>1e-6):
                    raise ValueError('SDK 相机时钟来源与观测帧不一致')
                previous_frames[key]=number
    targets=np.asarray([s['target_stamp_seconds'] for s in samples],float)
    relative=np.asarray([s['relative_seconds'] for s in samples],float)
    if (not np.isfinite(np.r_[targets,relative]).all() or
            not np.allclose(np.diff(targets),DT,rtol=0,atol=1e-5) or
            not np.allclose(relative,np.arange(8)*DT,rtol=0,atol=1e-8)):
        raise ValueError('物理时间历史网格异常')
    if previous_anchor is not None and targets[-1]<=previous_anchor+1e-5:
        raise ValueError('同一历史锚点不能重复推理')
    stamps=[]
    for key in KEYS:
        source=np.asarray([s['stamp_seconds'][key] for s in samples],float)
        age=np.asarray([s['source_age_at_receipt_seconds'][key] for s in samples],float)
        if (not np.isfinite(np.r_[source,age]).all() or np.any(np.diff(source)<=0) or
                max(abs(source-targets))>.080001 or np.any(age<-.05) or np.any(age>.3)):
            raise ValueError('源时间戳重复、过期或未覆盖网格：'+key)
        stamps.append(source)
    if np.ptp(np.asarray(stamps),axis=0).max()>.150001:
        raise ValueError('跨传感器时差超过当前输入契约')
    return float(targets[-1])


def physical_history_after_handoff(metadata, previous_anchor, minimum_monotonic, wall_minus_monotonic):
    """Keep the original history; wait until its latest sources follow handoff.

    A new target-grid index alone is insufficient during startup: that grid
    can still precede execution of the first command. Never relabel old frames.
    """
    anchor = physical_history(metadata, previous_anchor)
    if minimum_monotonic is not None:
        minimum, offset = float(minimum_monotonic), float(wall_minus_monotonic)
        if not np.isfinite([minimum, offset]).all():
            raise ValueError('上一段执行时间或源时钟偏移异常')
        latest = metadata['samples'][-1]
        actual = min(float(latest['stamp_seconds'][key]) for key in KEYS)-offset
        if anchor-offset < minimum-1e-5 or actual < minimum-1e-5:
            raise ValueError('尚无上一段接入后的真实观测')
    return anchor


def raw_packet(reply, ready, wall_minus_monotonic, origin=None, *, start_at_anchor=False, continuous_update=False):
    ready,offset=float(ready),float(wall_minus_monotonic)
    observation=float(reply['observation_target_stamp_seconds'])-offset
    if not np.isfinite([ready,offset,observation]).all() or not 0<=ready-observation<=.7:
        raise ValueError('推理时龄异常')
    if origin is None: origin=observation
    handoff = ready+.15 if start_at_anchor else (ready+.05 if continuous_update else
        float(origin)+math.ceil((ready+.05-float(origin))/DT)*DT)
    if start_at_anchor and int(reply['request_id']) != 0:
        raise ValueError('只有首条预测可以从动作起点启动')
    curve_origin = handoff if start_at_anchor else observation
    q,dq,ddq=evaluate_curve(reply['curve'],handoff-curve_origin)
    packet = {'sequence':int(reply['request_id']),'observation_monotonic':observation,
            'ready_monotonic':ready,'handoff_monotonic':handoff,
            'curve':reply['curve'],'raw_q_at_handoff':q[:7].tolist(),
            'raw_dq_at_handoff':dq[:7].tolist(),'raw_ddq_at_handoff':ddq[:7].tolist(),
            'gripper_raw_at_handoff':float(q[7]),
            'time_policy':'physical_1x_no_history_freeze_no_coefficient_correction'}
    if start_at_anchor:
        # Source time stays real. Only the first curve starts at its action
        # anchor; later curves retain physical observation-time compensation.
        packet['curve_origin_monotonic'] = curve_origin
    return packet


def native_input(packets):
    if not 1<=len(packets)<=512: raise ValueError('数值重放需要 1–512 个预测')
    finish=packets[-1]['handoff_monotonic']+DT
    lines=[f'{len(packets)} {finish:.17g}']
    for packet in packets:
        c=np.asarray(packet['curve']['coefficients'],float)
        if c.shape!=(7,8) or not np.isfinite(c).all(): raise ValueError('原始系数形状异常')
        for key,expected in (('anchor_s',2/11),('polynomial_duration_seconds',44/15),('valid_future_seconds',32/15)):
            value=float(packet['curve'][key])
            if not math.isfinite(value) or abs(value-expected)>1e-7:
                raise ValueError('数值求值器仅支持已训练的物理时间尺度')
        fields=[packet['sequence'],packet['observation_monotonic'],packet['handoff_monotonic'],
                packet['ready_monotonic'],*c[:,:7].ravel().tolist()]
        if not np.isfinite(fields).all(): raise ValueError('数值重放输入非有限')
        lines.append(' '.join(f'{x:.17g}' for x in fields))
    return '\n'.join(lines)+'\n'
